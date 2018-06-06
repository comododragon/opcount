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
// Describes a loop in terms of its longest path, trip count, etc.
//
//===----------------------------------------------------------------------===//

// Local includes
#include "LoopsDescription.h"
#include "int4.h"

namespace opcountutils {

LoopDescription::LoopDescription() : depth(0), tripCount(0) {
	calculated = false;
}

LoopDescription::LoopDescription(unsigned int depth, unsigned int tripCount) : depth(depth), tripCount(tripCount) {
	calculated = false;
}

/// Calculate longest path count.
void LoopDescription::calculate(int4 count) {
	this->count = count;
	calculated = true;
}

}
