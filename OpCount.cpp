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
				errs() << generateLine("Naive operational intensity is " + std::to_string(count[1] / (float) count[0]) + " Bytes/Insts");
				errs() << generateSeparator();
				break;
			case COUNT_MODE_NMI:
				errs() << generateLine("Longest path (bytes transferred) for __kernel function is " + std::to_string(count[0]));
				errs() << generateLine("Number of instructions in this path is " + std::to_string(count[1]));
				errs() << generateLine("Naive memory intensity is " + std::to_string(count[0] / (float) count[1]) + " Bytes/Insts");
				errs() << generateSeparator();
				break;
			default:
				errs() << generateLine("Longest path for __kernel function is " + std::to_string(count[0]));
				errs() << generateSeparator();
				break;
		}
	}

	// This pass performed not modifications in this code
	return false;
}

//===--------------------------------------------------------------------===//
// SimplifiedGraph methods.
//===--------------------------------------------------------------------===//

/// Based on a control-flow graph of a Function/Loop, create a graph (starting node H) substituting inner loops
/// by single nodes. AI is the AnalyserInterface created by the function. LD is a cache for Loops with resolved trip counts.
/// By transforming inner loops into nodes, back edges are removed and longest path search is simplified.
OpCount::SimplifiedGraph::SimplifiedGraph(
	OpCount *inst, BasicBlock &H, AnalyserInterface &AI, LoopsDescription &LD, DataLayout &DL,
	int depth, unsigned int countMode, bool verbose, unsigned int baseLevel
) {
	opCountInst = inst;
	this->DL = &DL;
	Loop *HL = AI.getLoopInfo().getLoopFor(&H);
	unsigned int depthHL = HL? LD[HL->getName()].depth : 0;
	int terminatorCount = 0;
	std::stack<const BasicBlock *> S;
	std::stack<bool> SLoop;
	std::set<std::string> visited;
	this->countMode = countMode;
	this->verbose = verbose;
	this->baseLevel = baseLevel;

	// Push the starting node, which is not a loop
	S.push(&H);
	SLoop.push(false);

	while(!S.empty()) {
		// Get top BasicBlock from stack and if it is entry node for a loop
		const BasicBlock *T = S.top();
		bool isLoop = SLoop.top();
		Loop *TL = AI.getLoopInfo().getLoopFor(T);
		S.pop();
		SLoop.pop();

		// Mark node as visited
		if(0 == visited.count(T->getName()))
			visited.insert(T->getName());

		// If popped node is a loop (i.e. it is the entry block for a loop deeper than the loop where H resides, if any)
		if(isLoop) {
			// Get exiting edges for this subloop
			SmallVector<LoopBase<BasicBlock, Loop>::Edge, 4> EEV;
			TL->getExitEdges(EEV);

			// Iterate through destination nodes from this subloop
			bool isLeafNode = true;
			for(LoopBase<BasicBlock, Loop>::Edge &EE : EEV) {
				const BasicBlock *sT = EE.second;
				Loop *sTL = AI.getLoopInfo().getLoopFor(sT);
				unsigned int depthSTL = sTL? LD[sTL->getName()].depth : 0;

				// If destination was not visited, add it to stack
				isLeafNode = false;
				if(0 == visited.count(sT->getName())) {
					// XXX: The loop database must have been generated before generating this graph.
					// If count was not calculated, something is very likely wrong
					if(!(LD[T->getName()].calculated)) errs() << "Warning: count of " << T->getName() << " not calculated\n";
					addEdge(T->getName(), sT->getName(), LD[T->getName()].count * LD[T->getName()].tripCount);
					S.push(sT);
					SLoop.push(depthSTL != depthHL);
				}
			}

			// If no destination nodes were found or all were visited, connect this node to a terminator. A terminator
			// is used to take this node's weight into account in the longest path calculation
			if(isLeafNode) {
				// XXX: The loop database must have been generated before generating this graph.
				// If count was not calculated, something is very likely wrong
				if(!(LD[T->getName()].calculated)) errs() << "Warning: count of " << T->getName() << " not calculated\n";
				addEdge(T->getName(), "terminator" + std::to_string(terminatorCount++), LD[T->getName()].count * LD[T->getName()].tripCount);
			}
		}
		// Popped node is not a loop (i.e. this node resides in the same loop depth as H, if any)
		else {
			// Iterate through destination nodes from this node
			bool isLeafNode = true;
			for(succ_const_iterator sit = succ_begin(T); sit != succ_end(T); sit++) {
				const BasicBlock *sT = *sit;
				Loop *sTL = AI.getLoopInfo().getLoopFor(sT);
				unsigned int depthTL = TL? LD[TL->getName()].depth : 0;
				unsigned int depthSTL = sTL? LD[sTL->getName()].depth : 0;

				// Ignore if the successor is the own node
				if(sT->getName() == T->getName())
					continue;

				// Successor is in a different loop than source
				if((!TL && sTL) || (TL && !sTL) || ((TL && sTL) && (TL->getName() != sTL->getName()))) {
					// Successor is in a loop with greater depth (subloop). If it hasn't been visited, add this
					// successor and notify through SLoop that this inserted node is in fact a subloop
					if(depthTL + 1 == depthSTL) {
						isLeafNode = false;
						if(0 == visited.count(sTL->getName())) {
							addEdge(T->getName(), sTL->getName(), countNodeInsts(*T));
							S.push(sTL->getHeader());
							// XXX: Apparently always true
							SLoop.push(depthSTL != depthHL);
						}
					}
					// Successor is in a loop with smaller depth (outer loop)
					else if(depthSTL + 1 == depthTL) {
						// If this graph being analysed is a loop, this successor is part of another
						// loop. It should therefore be ignored
						if(HL) {
							// Leaf node
						}
						// If this graph being analysed is a function, then such successor must be considered
						else {
							// If destination was not visited, add it to stack
							isLeafNode = false;
							if(0 == visited.count(sT->getName())) {
								addEdge(T->getName(), sT->getName(), countNodeInsts(*T));
								S.push(sT);
								// XXX: Apparently always false
								SLoop.push(depthSTL != depthHL);
							}
						}
					}
					// Successor node is in the same depth as the source
					else if(depthSTL == depthTL) {
						// If this graph is a loop, the successor is in the same depth but each resides
						// in a different loop. It should therefore be ignored
						if(HL) {
							// Leaf node (same depth but different loops)
						}
						// If this graph being analysed is a function, then such successor must be considered
						else {
							// If destination was not visited, add it to stack
							isLeafNode = false;
							if(0 == visited.count(sT->getName())) {
								addEdge(T->getName(), sT->getName(), countNodeInsts(*T));
								S.push(sT);
								// XXX: Apparently always false
								SLoop.push(depthSTL != depthHL);
							}
						}
					}
					else {
						// XXX: We are assuming that all edges connecting BasicBlocks have a depth difference of at
						// most 1. If this is not respected, a warning is raised and this pass may have invalid results
						errs() << "Warning: the loop depth difference between this BB and its successor is more than 1\n";
						errs() << "TL " << depthTL << " STL " << depthSTL << "\n";
					}
				}
				// Successor is in the same loop (if any) than source
				else {
					// If destination was not visited, add it to stack
					isLeafNode = false;
					if(0 == visited.count(sT->getName())) {
						addEdge(T->getName(), sT->getName(), countNodeInsts(*T));
						S.push(sT);
						// XXX: Apparently always false
						SLoop.push(depthSTL != depthHL);
					}
				}
			}

			// If no destination nodes were found or all were visited, connect this node to a terminator. A terminator
			// is used to take this node's weight into account in the longest path calculation
			if(isLeafNode)
				addEdge(T->getName(), "terminator" + std::to_string(terminatorCount++), countNodeInsts(*T));
		}
	}

	// If this graph is a loop, check if every exiting block has terminators. If not, add them
	if(HL) {
		SmallVector<LoopBase<BasicBlock, Loop>::Edge, 4> EEV;
		HL->getExitEdges(EEV);
		for(LoopBase<BasicBlock, Loop>::Edge &EE : EEV)
			addEdge(EE.first->getName(), "terminator" + std::to_string(terminatorCount++), countNodeInsts(*(EE.first)));
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
	// Insert node v in node u dictionary
	else {
		adj[u].insert(std::pair<std::string, int4>(v, weight));
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
			if(verbose) errs() << generateLine("Longest path is " + std::to_string(finally[0]));
			if(verbose) errs() << generateLine("Number of transferred bytes is " + std::to_string(finally[1]));
			break;
		case COUNT_MODE_NMI:
			if(verbose) errs() << generateLine("Longest path (transferred bytes) is " + std::to_string(finally[0]));
			if(verbose) errs() << generateLine("Number of instructions is " + std::to_string(finally[1]));
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

// Count specified metric by countMode for this BB/Node. If this BB makes a call, its count is also considered.
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

//===--------------------------------------------------------------------===//
// Private methods.
//===--------------------------------------------------------------------===//

/// Add a loop to LoopsDescription. If such loop has a subloop, this function
/// is called recursively.
unsigned int OpCount::handleLoop(Loop &L, AnalyserInterface &AI, LoopsDescription &LD, unsigned int level) {
	unsigned int maxDepth = 0;

	// Iterate through all loops with depth + 1 relative to this loop
	for(Loop *SL : L.getSubLoops()) {
		unsigned int depth = SL->getLoopDepth();

		// Update maximum depth if applicable
		if(depth > maxDepth)
			maxDepth = depth;

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
		depth = handleLoop(*SL, AI, LD, level + 1);

		// Update maximum depth if applicable
		if(depth > maxDepth)
			maxDepth = depth;
	}

	return maxDepth;
}

/// Calculate metrics for this function.
int4 OpCount::handleFunction(Function &F, DataLayout &DL, unsigned int level) {
	AnalyserInterface AI(*this, F);
	LoopsDescription LD;
	unsigned int maxDepth = 0;

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

		// Update maximum depth if applicable
		if(depth > maxDepth)
			maxDepth = depth;

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
		depth = handleLoop(*L, AI, LD, level + 2);

		// Update maximum depth if applicable
		if(depth > maxDepth)
			maxDepth = depth;
	}

#if 0
	if(memAnalysis) {
		MDNode *M = F.getMetadata("kernel_arg_addr_space");
		if(M) {
			// Generate MemoryDescription for this function's arguments
			if(verbose) errs() << generateLine("Generating global memory database", level);
			// TODO: CONSIDERAR TAMBEM __constant (2 == addrSpace)?
			int i = 0;
			for(Function::arg_iterator ait = F.arg_begin(); ait != F.arg_end(); ait++) {
				Argument &A = *ait;
				const MDOperand &O = M->getOperand(i++);
				Constant *C = cast<ConstantAsMetadata>(O)->getValue();
				unsigned int addrSpace = cast<ConstantInt>(C)->getZExtValue();
				if(1 == addrSpace) {
					if(verbose) errs() << generateLine("Found argument " + A.getName().str() + " with __global qualifier", level + 1);
					GMemArgs.push_back(A.getName());
				}
			}

		}
	}
#endif

	// Starting from the deepest loops, calculate the longest path for all
	for(unsigned int i = maxDepth; i > 0; i--) {
		if(verbose) errs() << generateLine("Calculating longest path of loops (depth " + std::to_string(i) + ")", level);
		// Find for loops in LD with depth i
		for(auto &L : LD) {
			if(L.second.depth == i) {
				// TODO: achar um jeito melhor de achar headers de loops
				// Get entry block for this loop
				BasicBlock *H = NULL;
				for(BasicBlock &BB : F) {
					if(L.first == BB.getName())
						H = &BB;
				}

				if(verbose) errs() << generateLine("Loop " + L.first, level + 1);
				if(verbose) errs() << generateLine("Generating simplified graph", level + 2);
				// Create a SimplifiedGraph for this loop
				SimplifiedGraph G(this, *H, AI, LD, DL, i, countMode, verbose, level + 3);
				if(verbose) errs() << generateLine("Finding longest path", level + 2);
				// Find the longest path for this loop and cache it in LD
				LD[H->getName()].calculate(G.getLongestPath(H->getName()));
			}
		}
	}

	if(verbose) errs() << generateLine("Calculating longest path of function", level);
	if(verbose) errs() << generateLine("Generating simplified graph", level + 1);
	// Create a SimplifiedGraph for this function
	SimplifiedGraph G(this, F.getEntryBlock(), AI, LD, DL, 0, countMode, verbose, level + 2);
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
