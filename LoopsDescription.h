//===-- lib/Transforms/OpCount/LoopsDescription.h ------------- -*- C++ -*-===//
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
// Describes a loop in terms of its trip count, depth and all contained BBs.
//
//===----------------------------------------------------------------------===//

#ifndef LIB_TRANSFORMS_OPCOUNT_LOOPSDESCRIPTION_H
#define LIB_TRANSFORMS_OPCOUNT_LOOPSDESCRIPTION_H

// Local includes
#include "int4.h"

#include <map>
#include <vector>

#include "llvm/Pass.h"
#include "llvm/IR/Module.h"

using namespace llvm;

namespace opcountutils {

/// Loop descriptor.
struct LoopDescription {
	/// Loop depth (greater than 1).
	unsigned int depth;
	/// Name of all basic blocks contained in this loop.
	std::vector<std::string> BBs;

	/// Longest path inside this loop. The other 3 int values can carry some useful information.
	int4 count;

	/// Amount of times this loop is repeated.
	unsigned int tripCount;

	LoopDescription();

	LoopDescription(unsigned int depth, unsigned int tripCount);

	/// Return true if this loop contains basic block BB.
	bool contains(const BasicBlock &BB);
};

/// Defines a (string; LoopDescription) pair.
typedef std::pair<std::string, LoopDescription> LoopDescriptionPair;

/// Defines a dictionary of LoopDescriptions. Each loop is indexed by its name (((Loop *) L)->getName()).
typedef std::map<std::string, LoopDescription> LoopsDescription;

}

#endif
