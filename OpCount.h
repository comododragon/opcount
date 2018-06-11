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
// This class implements several metrics that may be calculated by traversing
// the longest execution path of a LLVM IR. If loops are found, its trip count
// is used. If such information is not available, a default trip count is used.
// If a undefined function is used, a default count is used.
//
// Note: The SimplifiedGraph's depth first traversal and longest path
// algorithms were taken from geeksforgeeks.org:
// https://www.geeksforgeeks.org/find-longest-path-directed-acyclic-graph/
//
//===----------------------------------------------------------------------===//

#ifndef LIB_TRANSFORMS_OPCOUNT_OPCOUNT_H
#define LIB_TRANSFORMS_OPCOUNT_OPCOUNT_H

// Local includes
#include "int4.h"
#include "FunctionsDescription.h"
#include "LoopsDescription.h"

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
/// If an undefined function is called and no default undefined function count was specified, this value is used.
#define DEFAULT_UNDEFINED_FUNCTION_COUNT 10
// Defines the size of the console line for printing
#define LINE_WIDTH 70

using namespace llvm;
using namespace opcountutils;

//===--------------------------------------------------------------------===//
// Possible pass arguments.
//===--------------------------------------------------------------------===//

/// -def-trip-count=<TP>: if a loop (top-level) trip count was not possible to infer, use <TP> instead.
static cl::opt<unsigned int> DefTripCount("def-trip-count", cl::desc("Specify default trip count for unresolved outer loops (depth == 1)"));
/// -def-inner-trip-count=<TP>: if a loop (inner) trip count was not possible to infer, use <TP> instead.
static cl::opt<unsigned int> DefInnerTripCount("def-inner-trip-count", cl::desc("Specify default trip count for unresolved inner loops (depth > 1)"));
/// -def-undefined-function-count=<FC>: if an undefined function was called, use <FC> as default function count.
static cl::opt<unsigned int> DefUndefinedFunctionCount("def-undefined-function-count", cl::desc("Specify default count for undefined functions"));
/// -count-mode=OPT: specify how nodes should be counted.
static cl::opt<std::string> CountMode("count-mode", cl::desc("Specify how nodes should be counted (check pass docs for more info)"));
#if 0
/// -mem-analysis: perform memory analysis (e.g. global read/write accesses).
static cl::opt<bool> MemAnalysis("mem-analysis", cl::desc("Perform memory analysis (e.g. global read/write accesses)"));
#endif
/// -verbose: print a lot of messages.
static cl::opt<bool> Verbose("verbose", cl::desc("Show all performed calculations"));

//===--------------------------------------------------------------------===//
// Helper function implemented in CFG.cpp (CFG.h has no header for it).
//===--------------------------------------------------------------------===//

namespace llvm {
	void FindFunctionBackedges(const Function &F, SmallVectorImpl<std::pair<const BasicBlock*,const BasicBlock*> > &Result);
}

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
		line = line.substr(0, LINE_WIDTH - levelString.length() - 3) + "...";
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
		// All instructions are counted for longest path.
		COUNT_MODE_ALL = 0,
		// Only floating-point ops are counted for longest path.
		COUNT_MODE_FP = 1,
		// Naive operational intensity: all instructions are counted for longest path.
		// Along this path, all load and stores are counted as well.
		COUNT_MODE_NOI = 2,
		// Naive memory intensity: the longest path is decided by the longest amount
		// of loads and stores. Along this path, all instructions are counted as well.
		COUNT_MODE_NMI = 3
	};

	//===--------------------------------------------------------------------===//
	// Public methods.
	//===--------------------------------------------------------------------===//

	/// Default constructor.
	OpCount();

	void getAnalysisUsage(AnalysisUsage &AU) const override;

	/// Run pass on module M.
	bool runOnModule(Module &M) override;

private:

	//===--------------------------------------------------------------------===//
	// Private attributes
	//===--------------------------------------------------------------------===//

	/// Functions descriptor cache.
	FunctionsDescription FD;
	/// DataLayout used to get size of types referred by load/stores.
	DataLayout *DL;
	// Values read from arguments
	unsigned int defaultTripCount;
	unsigned int defaultInnerTripCount;
	int4 defaultUndefinedFunctionCount;
	unsigned int countMode;
	std::string countModeStr;
	bool verbose;

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

#if 0
		/// Similar to calling getLoopInfo() and getSE() without actually using the return value.
		/// This method refreshes LoopInfo and ScalarEvolution to the correct function.
		void refresh(void);
#endif
	};

	//===--------------------------------------------------------------------===//
	// Inner class: SimplifiedGraph
	//===--------------------------------------------------------------------===//

	class SimplifiedGraph {
		/// SimplifiedGraph needs an instance of OpCount to work.
		OpCount *opCountInst;
		/// DataLayout used to get size of types referred by load/stores.
		DataLayout *DL;
		/// Adjacency list.
		std::map<std::string, std::map<std::string, int4>> adj;
		/// Defines count mode when calculating longest path.
		unsigned int countMode;
		/// If true, a lot of messages are printed in the output console.
		bool verbose;
		/// Defines the base level for the messaging hierarchy.
		unsigned int baseLevel;

		// Check if basic block BB is inside one or more loops. If positive, return the product of all loop trip counts
		// that contains BB
		int tripCountsFactor(const BasicBlock &BB, LoopsDescription &LD);

		// Count specified metric by countMode for this BB/Node. If this BB makes a call, its count is also considered
		int4 countNodeInsts(const BasicBlock &BB);

		// Perform a topological sort in this graph starting from node v
		void topologicalSort(std::string v, std::set<std::string> &visited, std::stack<std::string> &S);

	public:

		/// Starting from basic block H inside a control-flow graph, find the longest path considering the metric specified in countMode.
		/// Back-edges are ignored according to FBP. If a visited node is inside one or more loops, its weight is multiplied by all loop
		/// trip counts where it is contained (trip counts are stored in LD).
		SimplifiedGraph(
			OpCount *inst, BasicBlock &H, LoopsDescription &LD, DataLayout &DL, FunctionBackedgesPairs &FBP, 
			unsigned int countMode = COUNT_MODE_ALL, bool verbose = false, unsigned int baseLevel = 0
		);

		/// Adds an edge between node u and v.
		void addEdge(std::string u, std::string v, int4 weight);

		/// Get longest path for this graph starting from node s. Since SimplifiedGraph does not have back-edges,
		/// this problem is not NP-Hard (phew...). Graph weights are int tuples. By definition, the first value
		/// is used to calculate the longest path. The other 3 integer values can be used to carry useful information
		/// along the longest path.
		int4 getLongestPath(std::string s);
	};

	//===--------------------------------------------------------------------===//
	// Private methods.
	//===--------------------------------------------------------------------===//

	/// Add a loop to LoopsDescription. If such loop has a subloop, this function is called recursively.
	void handleLoop(Loop &L, AnalyserInterface &AI, LoopsDescription &LD, unsigned int level);

	/// Calculate metrics for this function.
	int4 handleFunction(Function &F, DataLayout &DL, unsigned int level = 0);
};

}

#endif
