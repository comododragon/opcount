//===-- lib/Transforms/OpCount/LoopsDescription.cpp ----------- -*- C++ -*-===//
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

// Local includes
#include "LoopsDescription.h"
#include "int4.h"

#include "llvm/Pass.h"
#include "llvm/IR/Module.h"

using namespace llvm;

namespace opcountutils {

LoopDescription::LoopDescription() : depth(0), tripCount(0) { }

LoopDescription::LoopDescription(unsigned int depth, unsigned int tripCount) : depth(depth), tripCount(tripCount) { }

/// Return true if this loop contains basic block BB.
bool LoopDescription::contains(const BasicBlock &BB) {
	return (std::find(BBs.begin(), BBs.end(), BB.getName()) != BBs.end());
}

}
