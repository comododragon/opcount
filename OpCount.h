//===-- lib/Transforms/OpCount/OpCount.h ---------------------- -*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
// Copyright (c) 2018 André Bannwart Perina
//
//===----------------------------------------------------------------------===//
//
// This class implements a pass to find the longest execution path of a LLVM
// IR. If loops are found, its trip count is used. If such information is
// not available, a default trip count is used. If a undefined function is
// used, a default count is used.
//
// Note: The SimplifiedGraph's depth first traversal and longest path
// algorithms were taken from geeksforgeeks.org:
// https://www.geeksforgeeks.org/find-longest-path-directed-acyclic-graph/
//
//===----------------------------------------------------------------------===//

#ifndef LIB_TRANSFORMS_OPCOUNT_OPCOUNT_H
#define LIB_TRANSFORMS_OPCOUNT_OPCOUNT_H

// Standard C++ includes
#include <set>
#include <stack>

#include "llvm/Pass.h"
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/Analysis/ScalarEvolution.h"
// For FunctionPass
#include "llvm/IR/Module.h"

//===--------------------------------------------------------------------===//
// Default values.
//===--------------------------------------------------------------------===//

/// If a top-level loop has unspecified trip count and no default trip count was specified, this value is used.
#define DEFAULT_TRIP_COUNT 300
/// If a non-top-level loop has unspecified trip count and no default trip count was specified, this value is used.
#define DEFAULT_INNER_TRIP_COUNT 300
/// If an undefined function is called and no default undefined function count was specifiec, this value is used.
#define DEFAULT_UNDEFINED_FUNCTION_COUNT 10
// Defines the size of the console line for printing
#define LINE_WIDTH 60

using namespace llvm;

//===--------------------------------------------------------------------===//
// Possible pass arguments.
//===--------------------------------------------------------------------===//

/// -def-trip-count=<TP>: if a loop (top-level) trip count was not possible to infer, use <TP> instead.
static cl::opt<unsigned int> DefTripCount("def-trip-count", cl::desc("Specify default trip count for unresolved outer loops (depth == 1)"));
/// -def-inner-trip-count=<TP>: if a loop (inner) trip count was not possible to infer, use <TP> instead.
static cl::opt<unsigned int> DefInnerTripCount("def-inner-trip-count", cl::desc("Specify default trip count for unresolved inner loops (depth > 1)"));
/// -def-undefined-function-count=<FC>: if an undefined function was called, use <FC> as default function count.
static cl::opt<unsigned int> DefUndefinedFunctionCount("def-undefined-function-count", cl::desc("Specify default count for undefined functions"));
/// -count-mode=OPT: select which instructions to be counted
static cl::opt<std::string> CountMode("count-mode", cl::desc("Specify which instructions should be counted (supported: all, fp)"));
/// -verbose: print a lot of messages.
static cl::opt<bool> Verbose("verbose", cl::desc("Show all performed calculations"));

namespace {

//===--------------------------------------------------------------------===//
// Helper functions for fancy console printing.
//===--------------------------------------------------------------------===//

/// Generate a shiny separator in the console log.
std::string generateSeparator(unsigned int lineWidth = LINE_WIDTH) {
	return std::string(lineWidth + 4, '*') + "\n";
}

/// Generate a fancy bordered line in the console log. Level defines the amount of space to put before the string.
/// If trim is true, the input string will be trimmed to fit in lineWidth + level.
std::string generateLine(std::string line, unsigned int level = 0, bool trim = true, unsigned int lineWidth = LINE_WIDTH) {
	std::string levelString = level? std::string(level, ' ') : "";

	// Trim line if needed
	if(trim) {
		if((((int) lineWidth) - ((int) line.length()) - ((int) levelString.length()) - 3) < 0)
			line = line.substr(0, LINE_WIDTH - 3) + "...";
	}

	// Add spaces and final touches before returning it
	int space = lineWidth - line.length() - levelString.length();
	std::string spaceString = (space > 0)? std::string(space, ' ') : "";
	return "* " + levelString + line + spaceString + " *\n";
}

//===--------------------------------------------------------------------===//
// Main class.
//===--------------------------------------------------------------------===//

/// Main class.
struct OpCount : public ModulePass {
	static char ID;
	/// Count mode enum.
	enum {
		COUNT_MODE_ALL = 0,
		COUNT_MODE_FP = 1
	};

	//===--------------------------------------------------------------------===//
	// Public methods.
	//===--------------------------------------------------------------------===//

	/// Default constructor with possible arguments for this pass.
	OpCount(
		Optional<unsigned int> DefTripCountOp = None,
		Optional<unsigned int> DefInnerTripCountOp = None,
		Optional<unsigned int> DefUndefinedFunctionCountOp = None,
		Optional<std::string> CountModeOp = None,
		Optional<bool> VerboseOp = None
	);

	void getAnalysisUsage(AnalysisUsage &AU) const override;

	/// Run pass on module M.
	bool runOnModule(Module &M) override;

private:

	//===--------------------------------------------------------------------===//
	// Inner class: AnalyserInterface
	//===--------------------------------------------------------------------===//

	/// The getAnalysis<>() method used to retrieve LoopInfo and ScalarEvolution are
	/// recreated every time it is called. If such method was called for function F
	/// and for function F2 afterwards, the last call invalidates all information that
	/// was created for function F. Therefore, if after getAnalysis<>() was called for F2
	/// the developer needs data that getAnalysis<>() generated for F, getAnalysis<>()
	/// must be re-called. This class performs this automatically. Instead of creating
	/// references for LI and SE, developer should always call AnalyserInterface::getLoopInfo()
	/// or AnalyserInterface::getSE(). This class guarantees that analyses are re-created
	/// every time it is needed (e.g. if analyses were invalidated).
	class AnalyserInterface {
		/// OpCount Instance. A pass is needed to call getAnalysis, this is why OpCount is stored.
		OpCount *inst;
		/// Current active function for the analyser.
		static Function *activeFunction;
		/// LoopInfo for current active function
		static LoopInfo *LI;
		/// ScalarEvolution for current active function
		static ScalarEvolution *SE;
		/// Each instance will have a requested function. See getLoopInfo() or getSE() for more information.
		Function *requestedFunction;

	public:

		// Constructor
		AnalyserInterface(OpCount &inst, Function &requestedFunction);

		/// Request for LoopInfo. Each instance of this object has its own requestedFunction. If it differs
		/// from the static activeFunction, LoopInfo and ScalarEvolution are not pointing to the right data.
		/// Therefore such analysis must be recreated for the requested function and activeFunction is
		/// updated.
		LoopInfo &getLoopInfo(void);

		/// Request for ScalarEvolution. Each instance of this object has its own requestedFunction. If it differs
		/// from the static activeFunction, LoopInfo and ScalarEvolution are not pointing to the right data.
		/// Therefore such analysis must be recreated for the requested function and activeFunction is
		/// updated.
		ScalarEvolution &getSE(void);
	};

	//===--------------------------------------------------------------------===//
	// Inner types.
	//===--------------------------------------------------------------------===//

	/// Loop descriptor.
	struct LoopDescription {
		/// BasicBlocks contained in this loop.
		std::vector<std::string> BBs;
		/// Loop depth (greater than 1).
		unsigned int depth;
		/// Longest path inside this loop.
		unsigned int count;
		/// Amount of times this loop is repeated.
		unsigned int tripCount;

		LoopDescription() : depth(0), count(0), tripCount(0) { }
		LoopDescription(unsigned int depth, unsigned int tripCount) : depth(depth), count(0), tripCount(tripCount) { }
	};
	/// Defines a (string; LoopDescription) pair.
	typedef std::pair<std::string, LoopDescription> LoopDescriptionPair;
	/// Defines a dictionary of LoopDescriptions. Each loop is indexed by its name (((Loop *) L)->getName()).
	typedef std::map<std::string, LoopDescription> LoopsDescription;
	/// Function descriptor. The int value defines this function's longest path.
	typedef std::pair<std::string, int> FunctionDescriptionPair;
	/// Defines a dictionary of FunctionDescriptionPairs. Each function is indexed by its name (((Function *) F)->getName()).
	typedef std::map<std::string, int> FunctionsDescription;
	// Functions descriptor cache
	FunctionsDescription FD;
	// Values read from arguments
	unsigned int defaultTripCount;
	unsigned int defaultInnerTripCount;
	unsigned int defaultUndefinedFunctionCount;
	unsigned int countMode;
	std::string countModeStr;
	bool verbose;

	//===--------------------------------------------------------------------===//
	// Private methods.
	//===--------------------------------------------------------------------===//

	/// Add a loop to LoopsDescription. If such loop has a subloop, this function
	/// is called recursively.
	unsigned int handleLoop(Loop &L, AnalyserInterface &AI, LoopsDescription &LD, unsigned int level);

	/// Find the longest path for a function.
	unsigned int handleFunction(Function &F, unsigned int level = 0);

	//===--------------------------------------------------------------------===//
	// Inner class: SimplifiedGraph
	//===--------------------------------------------------------------------===//

	class SimplifiedGraph {
		/// SimplifiedGraph needs an instance of OpCount to work.
		OpCount *opCountInst;
		/// Adjacency list.
		std::map<std::string, std::map<std::string, int>> adj;
		/// Defines count mode when calculating longest path.
		unsigned int countMode;
		/// If true, a lot of messages are printed in the output console.
		bool verbose;
		/// Defines the base level for the messaging hierarchy.
		unsigned int baseLevel;

		// Count amount of instructions inside BB. If this BB makes a call, its count is also considered.
		unsigned int countNodeInsts(const BasicBlock &BB);

		// Perform a topological sort in this graph starting from node v
		void topologicalSort(std::string v, std::set<std::string> &visited, std::stack<std::string> &S);

	public:

		/// Based on a control-flow graph of a Function/Loop, create a graph (starting node H) substituting inner loops
		/// by single nodes. AI is the AnalyserInterface created by the function. LD is a cache for Loops with resolved trip counts.
		/// By transforming inner loops into nodes, back edges are removed and longest path search is simplified.
		SimplifiedGraph(
			OpCount *inst, BasicBlock &H, AnalyserInterface &AI, OpCount::LoopsDescription &LD,
			int depth, unsigned int countMode = 0, bool verbose = false, unsigned int baseLevel = 0
		);

		/// Adds an edge between node u and v.
		void addEdge(std::string u, std::string v, int weight);

		/// Get longest path for this graph starting from node s.
		int getLongestPath(std::string s);
	};
};

}

#endif
