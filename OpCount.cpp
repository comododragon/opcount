//===-- lib/Transforms/OpCount/OpCount.cpp -------------------- -*- C++ -*-===//
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

// Local includes
#include "OpCount.h"
#include "int4.h"
#include "FunctionsDescription.h"
#include "LoopsDescription.h"

// Standard C++ includes
#include <set>
#include <stack>

#include "llvm/Pass.h"
#include "llvm/ADT/Optional.h"
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/Analysis/ScalarEvolution.h"
#include "llvm/Analysis/ScalarEvolutionExpressions.h"
#include "llvm/IR/CFG.h"
#include "llvm/IR/Module.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;
using namespace opcountutils;

namespace {

//===--------------------------------------------------------------------===//
// OpCount public methods.
//===--------------------------------------------------------------------===//

/// Default constructor with possible arguments for this pass.
OpCount::OpCount() : ModulePass(ID) {
	Optional<unsigned int> DefTripCountProvided(DefTripCount);
	Optional<unsigned int> DefInnerTripCountProvided(DefInnerTripCount);
	Optional<unsigned int> DefUndefinedFunctionCountProvided(DefUndefinedFunctionCount);
	Optional<std::string> CountModeProvided(CountMode);
	Optional<bool> VerboseProvided(Verbose);

	defaultTripCount = (DefTripCountProvided.hasValue() && (*DefTripCountProvided != 0))?
		*DefTripCountProvided : DEFAULT_TRIP_COUNT;
	defaultInnerTripCount = (DefInnerTripCountProvided.hasValue() && (*DefInnerTripCountProvided != 0))?
		*DefInnerTripCountProvided : DEFAULT_INNER_TRIP_COUNT;
	defaultUndefinedFunctionCount[0] = (DefUndefinedFunctionCountProvided.hasValue() && (*DefUndefinedFunctionCountProvided != 0))?
		*DefUndefinedFunctionCountProvided : DEFAULT_UNDEFINED_FUNCTION_COUNT;
	verbose = VerboseProvided.hasValue()?
		*VerboseProvided : false;

	// Select count mode (or use "all" if not defined)
	if(CountModeProvided.hasValue()) {
		if("fp" == *CountModeProvided) {
			// TODO
			countMode = COUNT_MODE_FP;
			countModeStr = "fp";
		}
		else if("noi" == *CountModeProvided) {
			countMode = COUNT_MODE_NOI;
			countModeStr = "noi";

			// Assuming load/stores are 30% of all instructions and all transfers 4 bytes (0.3 * 4)
			defaultUndefinedFunctionCount[1] = defaultUndefinedFunctionCount[0] * 1.2;
		}
		else if("nmi" == *CountModeProvided) {
			countMode = COUNT_MODE_NMI;
			countModeStr = "nmi";

			defaultUndefinedFunctionCount[1] = (DefUndefinedFunctionCountProvided.hasValue() && (*DefUndefinedFunctionCountProvided != 0))?
				*DefUndefinedFunctionCountProvided : DEFAULT_UNDEFINED_FUNCTION_COUNT;
			// Assuming load/stores are 30% of all instructions and all transfers 4 bytes (0.3 * 4)
			defaultUndefinedFunctionCount[0] = defaultUndefinedFunctionCount[1] * 1.2;
		}
		else {
			countMode = COUNT_MODE_ALL;
			countModeStr = "all";

		}
	}
	else {
		countMode = COUNT_MODE_ALL;
		countModeStr = "all";
	}
}

void OpCount::getAnalysisUsage(AnalysisUsage &AU) const {
	AU.addRequired<LoopInfoWrapperPass>();
	AU.addRequired<ScalarEvolutionWrapperPass>();
}

/// Run pass on module M.
bool OpCount::runOnModule(Module &M) {
	errs() << "************************************\n";
	errs() << "* OpCounter: IR Operations Counter *\n";
	errs() << "* ************* v0.3 ************* *\n";
	errs() << "* Author: Andre Bannwart Perina    *\n";
	errs() << "************************************\n";

	// Print arguments that are being considered
	errs() << generateSeparator();
	errs() << generateLine("OpCounter is using these values:");
	errs() << generateLine("Default trip count: " + std::to_string(defaultTripCount), 1);
	errs() << generateLine("Default inner trip count: " + std::to_string(defaultInnerTripCount), 1);
	errs() << generateLine("Count mode: " + countModeStr, 1);
	errs() << generateLine("Verbose: " + std::to_string(verbose), 1);
	// Print assumptions specific to countMode
	switch(countMode) {
		case COUNT_MODE_NOI:
			errs() << generateLine("Default undefined function count: " + std::to_string(defaultUndefinedFunctionCount[0]), 1);
			errs() << generateLine("Default undefined function byte store count: " + std::to_string(defaultUndefinedFunctionCount[1]), 1);
			break;
		case COUNT_MODE_NMI:
			errs() << generateLine("Default undefined function count: " + std::to_string(defaultUndefinedFunctionCount[1]), 1);
			errs() << generateLine("Default undefined function byte store count: " + std::to_string(defaultUndefinedFunctionCount[0]), 1);
			break;
		default:
			errs() << generateLine("Default undefined function count: " + std::to_string(defaultUndefinedFunctionCount[0]), 1);
			break;
	}
	errs() << generateSeparator();

	// Generate DataLayout
	DataLayout DL = DataLayout(&M);

	// Iterate through all functions within this module
	int4 count;
	for(Function &F : M) {
		// __kernel function found
		if(CallingConv::SPIR_KERNEL == F.getCallingConv()) {
			errs() << generateSeparator();
			errs() << generateLine("Found __kernel function: " + F.getName().str());

			if(F.size())
				count = handleFunction(F, DL);
			else
				if(verbose) errs() << generateLine("No basic blocks found. Skipping", 1);

			errs() << generateSeparator();
		}
	}

	// If count was found, print result (may differ based on countMode)
	if(count[0]) {
		errs() << generateSeparator();
		errs() << generateLine("Count mode: " + countModeStr);
		switch(countMode) {
			case COUNT_MODE_NOI:
				errs() << generateLine("Longest path for __kernel function is " + std::to_string(count[0]));
				errs() << generateLine("Number of bytes transferred in this path is " + std::to_string(count[1]));
				errs() << generateLine("Naive operational intensity is " + std::to_string(count[1] / (float) count[0]) + " bytes/insts");
				errs() << generateSeparator();
				break;
			case COUNT_MODE_NMI:
				errs() << generateLine("Longest path (bytes transferred) for __kernel function is " + std::to_string(count[0]));
				errs() << generateLine("Number of instructions in this path is " + std::to_string(count[1]));
				errs() << generateLine("Naive memory intensity is " + std::to_string(count[0] / (float) count[1]) + " bytes/insts");
				errs() << generateSeparator();
				break;
			default:
				errs() << generateLine("Longest path for __kernel function is " + std::to_string(count[0]));
				errs() << generateSeparator();
				break;
		}
	}

	// This pass performed no modifications in this code
	return false;
}

//===--------------------------------------------------------------------===//
// SimplifiedGraph methods.
//===--------------------------------------------------------------------===//

/// Starting from basic block H inside a control-flow graph, find the longest path considering the metric specified in countMode.
/// Back-edges are ignored according to FBP. If a visited node is inside one or more loops, its weight is multiplied by all loop
/// trip counts where it is contained (trip counts are stored in LD).
OpCount::SimplifiedGraph::SimplifiedGraph(
	OpCount *inst, BasicBlock &H, LoopsDescription &LD, DataLayout &DL, FunctionBackedgesPairs &FBP,
	unsigned int countMode, bool verbose, unsigned int baseLevel
) {
	this->opCountInst = inst;
	this->DL = &DL;
	this->countMode = countMode;
	this->verbose = verbose;
	this->baseLevel = baseLevel;
	std::stack<const BasicBlock *> S;
	std::set<std::string> visited;

	// Push the starting node
	S.push(&H);

	while(!S.empty()) {
		// Get top BasicBlock from stack
		const BasicBlock *T = S.top();
		S.pop();

		// Mark node as visited
		if(0 == visited.count(T->getName()))
			visited.insert(T->getName());

		// Iterate through all successors
		bool isLeaf = true;
		for(succ_const_iterator sit = succ_begin(T); sit != succ_end(T); sit++) {
			const BasicBlock *sT = *sit;

			// Consider only successors that are not accessed by a back-edge
			if(FBP.end() == std::find(FBP.begin(), FBP.end(), BackedgePair(T->getName(), sT->getName()))) {

				// If this node was not visited, add edge and push sT to stack
				if(0 == visited.count(sT->getName()))
					S.push(sT);

				// Add edge leaving from T to sT
				isLeaf = false;
				addEdge(T->getName(), sT->getName(), countNodeInsts(*T) * tripCountsFactor(*T, LD));
			}
		}

		// If no edges were added, this is a leaf basic block. Connect to terminator node
		if(isLeaf)
			addEdge(T->getName(), "terminator", countNodeInsts(*T) * tripCountsFactor(*T, LD));
	}
}

/// Adds an edge between node u and v.
void OpCount::SimplifiedGraph::addEdge(std::string u, std::string v, int4 weight) {
	// Node u does not exist in the graph. Create a dictionary for its destinations
	// and insert node v
	if(0 == adj.count(u)) {
		std::map<std::string, int4> newNode;
		newNode.insert(std::pair<std::string, int4>(v, weight));
		adj.insert(std::pair<std::string, std::map<std::string, int4>>(u, newNode));
	}
	// Insert node v in node u dictionary if such edge does not exist
	else if(0 == adj[u].count(v)) {
		adj[u].insert(std::pair<std::string, int4>(v, weight));
	}
	// Replace node v in node u dictionary
	else {
		adj[u][v] = weight;
	}
}

/// Get longest path for this graph starting from node s. Since SimplifiedGraph does not have back edges,
/// this problem is not NP-Hard (phew...). Graph weights are int tuples. By definition, the first value
/// is used to calculate the longest path. The other 3 integer values can be used to carry useful information
/// along the longest path.
int4 OpCount::SimplifiedGraph::getLongestPath(std::string s) {
	if(verbose) errs() << generateLine("From node " + s, baseLevel);
	std::stack<std::string> S;
	std::map<std::string, int4> distances;
	std::set<std::string> visited;

	// Calculate topological sort for all nodes
	if(verbose) errs() << generateLine("Calculating topological sort", baseLevel);
	for(auto &U : adj) {
		if(0 == visited.count(U.first)) {
			if(verbose) errs() << generateLine("For node " + U.first, baseLevel + 1);
			topologicalSort(U.first, visited, S);
		}
	}

	// Reset distances vector
	if(verbose) errs() << generateLine("Resetting distances vector", baseLevel);
	for(auto &U : adj)
		distances[U.first][0] = std::numeric_limits<int>::min();
	distances[s][0] = 0;

	if(verbose) errs() << generateLine("Popping from stack", baseLevel);
	while(!S.empty()) {
		// Get top node from stack
		std::string u = S.top();
		S.pop();
		if(verbose) errs() << generateLine("Popped " + u, baseLevel + 1);

		// Update distances if applicable. Only the first value from distance tuple is used
		// in the verification, however all values in the tuple are updated
		if(distances[u][0] != std::numeric_limits<int>::min()) {
			if(verbose) errs() << generateLine("Finding succeeders (loops are nodes)", baseLevel + 2);
			for(auto &V : adj[u]) {
				if(verbose) errs() << generateLine("Found " + V.first, baseLevel + 3);
				if(distances[V.first][0] < (distances[u][0] + V.second[0]))
					distances[V.first] = distances[u] + V.second;
			}
		}
	}

	// Get the longest distance in the distances vector
	int4 finally;
	finally[0] = std::numeric_limits<int>::min();
	for(auto &d : distances) {
		if(d.second[0] > finally[0])
			finally = d.second;
	}

	switch(countMode) {
		case COUNT_MODE_NOI:
			if(verbose) errs() << generateLine("Longest path is " + std::to_string(finally[0]), baseLevel);
			if(verbose) errs() << generateLine("Number of transferred bytes is " + std::to_string(finally[1]), baseLevel);
			break;
		case COUNT_MODE_NMI:
			if(verbose) errs() << generateLine("Longest path (transferred bytes) is " + std::to_string(finally[0]), baseLevel);
			if(verbose) errs() << generateLine("Number of instructions is " + std::to_string(finally[1]), baseLevel);
			break;
		default:
			if(verbose) errs() << generateLine("Longest path is " + std::to_string(finally[0]), baseLevel);
			break;
	}

	// Return the longest distance found
	return finally;
}

//===--------------------------------------------------------------------===//
// SimplifiedGraph private methods.
//===--------------------------------------------------------------------===//

// Check if basic block BB is inside one or more loops. If positive, return the product of all loop trip counts
// that contains BB
int OpCount::SimplifiedGraph::tripCountsFactor(const BasicBlock &BB, LoopsDescription &LD) {
	int factor = 1;

	// Search all loops that this BB is contained. Multiply the factor by these loops trip counts
	for(auto &L : LD) {
		if(L.second.contains(BB))
			factor *= L.second.tripCount;
	}

	return factor;
}

// Count specified metric by countMode for this BB/Node. If this BB makes a call, its count is also considered
int4 OpCount::SimplifiedGraph::countNodeInsts(const BasicBlock &BB) {
	int4 count;

	// Count based on count mode
	switch(countMode) {
		// Floating-point arithmetic
		case OpCount::COUNT_MODE_FP:
			// TODO
			break;
		// Naive Operational Intensity
		case OpCount::COUNT_MODE_NOI:
			for(const Instruction &I : BB) {
				// First element of int4 holds the total instruction count
				(count[0])++;

				// Second element of int4 holds the amount of byte transferred based on load/stores
				PointerType *pointerType = NULL;
				if(isa<LoadInst>(I))
					pointerType = cast<PointerType>(cast<LoadInst>(I).getPointerOperand()->getType());
				if(isa<StoreInst>(I))
					pointerType = cast<PointerType>(cast<StoreInst>(I).getPointerOperand()->getType());
				// If I is load or store, pointerType is not NULL. Calculate the amount of transferred data
				if(pointerType)
					count[1] += DL->getTypeStoreSize(pointerType->getPointerElementType());

				// If this instruction is a call method, its longest path must be calculated as well
				if(isa<CallInst>(I)) {
					Function *IF = cast<CallInst>(I).getCalledFunction();
					errs() << generateLine("Found function: " + IF->getName().str(), baseLevel);
					count += opCountInst->handleFunction(*IF, *DL, baseLevel + 1);
				}
			}
			break;
		// Naive Memory Intensity
		case OpCount::COUNT_MODE_NMI:
			for(const Instruction &I : BB) {
				// Second element of int4 holds the total instruction count
				(count[1])++;

				// First element of int4 holds the amount of byte transferred based on load/stores
				PointerType *pointerType = NULL;
				if(isa<LoadInst>(I))
					pointerType = cast<PointerType>(cast<LoadInst>(I).getPointerOperand()->getType());
				if(isa<StoreInst>(I))
					pointerType = cast<PointerType>(cast<StoreInst>(I).getPointerOperand()->getType());
				// If I is load or store, pointerType is not NULL. Calculate the amount of transferred data
				if(pointerType)
					count[0] += DL->getTypeStoreSize(pointerType->getPointerElementType());

				// If this instruction is a call method, its longest path must be calculated as well
				if(isa<CallInst>(I)) {
					Function *IF = cast<CallInst>(I).getCalledFunction();
					errs() << generateLine("Found function: " + IF->getName().str(), baseLevel);
					count += opCountInst->handleFunction(*IF, *DL, baseLevel + 1);
				}
			}
			break;
		// All
		default:
			for(const Instruction &I : BB) {
				count++;

				// If this instruction is a call method, its longest path must be calculated as well
				if(isa<CallInst>(I)) {
					Function *IF = cast<CallInst>(I).getCalledFunction();
					errs() << generateLine("Found function: " + IF->getName().str(), baseLevel);
					count += opCountInst->handleFunction(*IF, *DL, baseLevel + 1);
				}
			}
			break;
	}

	return count;
}

// Perform a topological sort in this graph starting from node v
void OpCount::SimplifiedGraph::topologicalSort(std::string v, std::set<std::string> &visited, std::stack<std::string> &S) {
	// Mark node as visited
	visited.insert(v);

	// Visit all unvisited successors
	for(auto &V : adj[v]) {
		if(0 == visited.count(V.first))
			topologicalSort(V.first, visited, S);
	}

	// Push this node in the longest path calculation stack
	S.push(v);
}

//===--------------------------------------------------------------------===//
// AnalyserInterface methods.
//===--------------------------------------------------------------------===//

/// Initialise static attributes to NULL
Function *OpCount::AnalyserInterface::activeFunction = NULL;
LoopInfo *OpCount::AnalyserInterface::LI = NULL;
ScalarEvolution *OpCount::AnalyserInterface::SE = NULL;

// Constructor
OpCount::AnalyserInterface::AnalyserInterface(OpCount &inst, Function &requestedFunction) {
	this->inst = &inst;
	this->requestedFunction = &requestedFunction;
}

/// Request for LoopInfo. Each instance of this object has its own requestedFunction. If it differs
/// from the static activeFunction, LoopInfo and ScalarEvolution are not pointing to the right data.
/// Therefore such analysis must be recreated for the requested function and activeFunction is
/// updated.
LoopInfo &OpCount::AnalyserInterface::getLoopInfo(void) {
	if(activeFunction && LI && SE) {
		// Active function is different from requested function. Recreate analyses
		if(activeFunction->getName() != requestedFunction->getName()) {
			activeFunction = requestedFunction;
			LI = &(inst->getAnalysis<LoopInfoWrapperPass>(*activeFunction).getLoopInfo());
			SE = &(inst->getAnalysis<ScalarEvolutionWrapperPass>(*activeFunction).getSE());
		}
	}
	// LI and/or SE are NULL. Create them for the first time
	else {
		activeFunction = requestedFunction;
		LI = &(inst->getAnalysis<LoopInfoWrapperPass>(*activeFunction).getLoopInfo());
		SE = &(inst->getAnalysis<ScalarEvolutionWrapperPass>(*activeFunction).getSE());
	}

	return *LI;
}

/// Request for ScalarEvolution. Each instance of this object has its own requestedFunction. If it differs
/// from the static activeFunction, LoopInfo and ScalarEvolution are not pointing to the right data.
/// Therefore such analysis must be recreated for the requested function and activeFunction is
/// updated.
ScalarEvolution &OpCount::AnalyserInterface::getSE(void) {
	if(activeFunction && LI && SE) {
		// Active function is different from requested function. Recreate analyses
		if(activeFunction->getName() != requestedFunction->getName()) {
			activeFunction = requestedFunction;
			LI = &(inst->getAnalysis<LoopInfoWrapperPass>(*activeFunction).getLoopInfo());
			SE = &(inst->getAnalysis<ScalarEvolutionWrapperPass>(*activeFunction).getSE());
		}
	}
	// LI and/or SE are NULL. Create them for the first time
	else {
		activeFunction = requestedFunction;
		LI = &(inst->getAnalysis<LoopInfoWrapperPass>(*activeFunction).getLoopInfo());
		SE = &(inst->getAnalysis<ScalarEvolutionWrapperPass>(*activeFunction).getSE());
	}

	return *SE;
}

#if 0
/// Similar to calling getLoopInfo() and getSE() without actually using the return value.
/// This method refreshes LoopInfo and ScalarEvolution to the correct function.
void OpCount::AnalyserInterface::refresh(void) {
	if(activeFunction && LI && SE) {
		// Active function is different from requested function. Recreate analyses
		if(activeFunction->getName() != requestedFunction->getName()) {
			activeFunction = requestedFunction;
			LI = &(inst->getAnalysis<LoopInfoWrapperPass>(*activeFunction).getLoopInfo());
			SE = &(inst->getAnalysis<ScalarEvolutionWrapperPass>(*activeFunction).getSE());
		}
	}
	// LI and/or SE are NULL. Create them for the first time
	else {
		activeFunction = requestedFunction;
		LI = &(inst->getAnalysis<LoopInfoWrapperPass>(*activeFunction).getLoopInfo());
		SE = &(inst->getAnalysis<ScalarEvolutionWrapperPass>(*activeFunction).getSE());
	}
}
#endif

//===--------------------------------------------------------------------===//
// Private methods.
//===--------------------------------------------------------------------===//

/// Add a loop to LoopsDescription. If such loop has a subloop, this function
/// is called recursively.
void OpCount::handleLoop(Loop &L, AnalyserInterface &AI, LoopsDescription &LD, unsigned int level) {

	// Iterate through all loops with depth + 1 relative to this loop
	for(Loop *SL : L.getSubLoops()) {
		unsigned int depth = SL->getLoopDepth();

		// Get trip count if available, use default if not
		const SCEV *backedgeCount = AI.getSE().hasLoopInvariantBackedgeTakenCount(&L)? AI.getSE().getMaxBackedgeTakenCount(&L) : NULL;
		unsigned int tripCount;
		if(backedgeCount && !isa<SCEVCouldNotCompute>(backedgeCount))
			tripCount = cast<SCEVConstant>(backedgeCount)->getValue()->getZExtValue() + 1;
		else
			tripCount = (1 == depth)? defaultTripCount : defaultInnerTripCount;

		// Create and populate a LoopDescription. Insert it afterwards in LD
		LoopDescription desc(depth, tripCount);
		for(BasicBlock *B : SL->blocks())
			desc.BBs.push_back(B->getName());
		LD.insert(LoopDescriptionPair(SL->getName(), desc));

		if(verbose) errs() << generateLine("Found loop " + SL->getName().str() + " (trip count: " + std::to_string(tripCount) + ")", level);

		// Call handleLoop for this subloop
		handleLoop(*SL, AI, LD, level + 1);
	}
}

/// Calculate metrics for this function.
int4 OpCount::handleFunction(Function &F, DataLayout &DL, unsigned int level) {
	AnalyserInterface AI(*this, F);
	LoopsDescription LD;

	// If this function was already calculated, use this value instead
	if(FD.count(F.getName()) != 0) {
		int4 count = FD[F.getName()];

		switch(countMode) {
			case COUNT_MODE_NOI:
				if(verbose) errs() << generateLine("Found cached inst count: " + std::to_string(count[0]), level);
				if(verbose) errs() << generateLine("Found cached byte transfer count: " + std::to_string(count[1]), level);
				break;
			case COUNT_MODE_NMI:
				if(verbose) errs() << generateLine("Found cached byte transfer count: " + std::to_string(count[0]), level);
				if(verbose) errs() << generateLine("Found cached inst count: " + std::to_string(count[1]), level);
				break;
			default:
				if(verbose) errs() << generateLine("Found cached inst count: " + std::to_string(count[0]), level);
				break;
		}

		return count;
	}
	// If this function is empty, use defaultUndefinedFunctionCount
	else if(!F.size()) {
		switch(countMode) {
			case COUNT_MODE_NOI:
				if(verbose) errs() << generateLine("Function is undefined. Assuming default count: " + std::to_string(defaultUndefinedFunctionCount[0]), level);
				if(verbose) errs() << generateLine("Function is undefined. Assuming default byte transfer count: " + std::to_string(defaultUndefinedFunctionCount[1]), level);
				break;
			case COUNT_MODE_NMI:
				if(verbose) errs() << generateLine("Function is undefined. Assuming default byte transfer count: " + std::to_string(defaultUndefinedFunctionCount[0]), level);
				if(verbose) errs() << generateLine("Function is undefined. Assuming default count: " + std::to_string(defaultUndefinedFunctionCount[1]), level);
				break;
			default:
				if(verbose) errs() << generateLine("Function is undefined. Assuming default count: " + std::to_string(defaultUndefinedFunctionCount[0]), level);
				break;
		}

		FD.insert(FunctionDescriptionPair(F.getName(), defaultUndefinedFunctionCount));
		return defaultUndefinedFunctionCount;
	}
		
	// Generate LoopDescription for all loops in this function
	if(verbose) errs() << generateLine("Generating loops database", level);
#if 0
	// These 5 lines are similar to getAnalysis<LoopInfoWrapperPass>(F).getLoopInfo(). However, since this
	// function may be called several times recursively, each call to getLoopInfo() destroys its older
	// reference. This does not happen when LoopInfo is created based on the DominatorTree
	DominatorTree DT = DominatorTree();
	DT.recalculate(F);
	LoopInfo LI;
	LI.releaseMemory();
	LI.analyze(DT);
#endif
	// Iterate through all top-level loops
	for(Loop *L : AI.getLoopInfo()) {
		unsigned int depth = L->getLoopDepth();

		// Get trip count if available, use default if not
		const SCEV *backedgeCount = AI.getSE().hasLoopInvariantBackedgeTakenCount(L)? AI.getSE().getMaxBackedgeTakenCount(L) : NULL;
		unsigned int tripCount;
		if(backedgeCount && !isa<SCEVCouldNotCompute>(backedgeCount))
			tripCount = cast<SCEVConstant>(backedgeCount)->getValue()->getZExtValue() + 1;
		else
			tripCount = (1 == depth)? defaultTripCount : defaultInnerTripCount;

		// Create and populate a LoopDescription. Insert it afterwards in LD
		LoopDescription desc(depth, tripCount);
		for(BasicBlock *B : L->blocks())
			desc.BBs.push_back(B->getName());
		LD.insert(LoopDescriptionPair(L->getName(), desc));

		if(verbose) errs() << generateLine("Found loop " + L->getName().str() + " (trip count: " + std::to_string(tripCount) + ")", level + 1);

		// Call handleLoop for this loop
		handleLoop(*L, AI, LD, level + 2);
	}

	// Generate back edges database for eliminating cycles in SimplifiedGraph generation
	if(verbose) errs() << generateLine("Generating back-edges database", level);
	SmallVector<std::pair<const BasicBlock *, const BasicBlock *>, 4> FBPSV;
	FunctionBackedgesPairs FBP;
	FindFunctionBackedges(F, FBPSV);
	for(std::pair<const BasicBlock *, const BasicBlock *> P : FBPSV)
		FBP.push_back(BackedgePair(P.first->getName(), P.second->getName()));

	// Calculate longest path for this function
	if(verbose) errs() << generateLine("Calculating longest path of function", level);
	if(verbose) errs() << generateLine("Generating simplified graph", level + 1);
	// Create a SimplifiedGraph for this function
	SimplifiedGraph G(this, F.getEntryBlock(), LD, DL, FBP, countMode, verbose, level + 2);
	if(verbose) errs() << generateLine("Finding longest path", level + 1);
	// Find the longest path for this function and cache it in FD
	int4 count = G.getLongestPath(F.getEntryBlock().getName());
	FD.insert(FunctionDescriptionPair(F.getName(), count));

	// Return the longest path count for this function
	return count;
}

}

/// The number doesn't matter. Its address does
char OpCount::ID = 0;

/// Register this pass
static RegisterPass<OpCount> X("opcount", "Operation Count pass");
