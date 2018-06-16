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
#include "helper.h"
#include "int4.h"

#include "llvm/Pass.h"
#include "llvm/IR/Module.h"

#include <cstdint>

using namespace llvm;

namespace opcountutils {

LoopDescription::LoopDescription() : depth(0), tripCount(0) { }

LoopDescription::LoopDescription(int64_t depth, int64_t tripCount) : depth(depth), tripCount(tripCount) { }

/// Return true if this loop contains basic block BB.
bool LoopDescription::contains(const BasicBlock &BB) {
	return (std::find(BBs.begin(), BBs.end(), getBBID(BB)) != BBs.end());
}

}
