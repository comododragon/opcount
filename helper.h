//===-- lib/Transforms/OpCount/helper.h ----------------------- -*- C++ -*-===//
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
// This header contains several helper functions used by this pass.
//
//===----------------------------------------------------------------------===//

#ifndef LIB_TRANSFORMS_OPCOUNT_HELPER_H
#define LIB_TRANSFORMS_OPCOUNT_HELPER_H

#include "llvm/Pass.h"
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/IR/Module.h"

// Defines the size of the console line for printing
#define LINE_WIDTH 70

using namespace llvm;

//===--------------------------------------------------------------------===//
// Helper function implemented in CFG.cpp (CFG.h has no header for it).
//===--------------------------------------------------------------------===//

namespace llvm {
	void FindFunctionBackedges(const Function &F, SmallVectorImpl<std::pair<const BasicBlock*,const BasicBlock*> > &Result);
}

namespace opcountutils {

//===--------------------------------------------------------------------===//
// Helper functions for fancy console printing.
//===--------------------------------------------------------------------===//

/// Generate a shiny separator in the console log.
std::string generateSeparator(unsigned int lineWidth = LINE_WIDTH);

/// Generate a fancy bordered line in the console log. Level defines the amount of space to put before the string.
/// If trim is true, the input string will be trimmed to fit in lineWidth + level.
std::string generateLine(std::string line, unsigned int level = 0, bool trim = true, unsigned int lineWidth = LINE_WIDTH);

//===--------------------------------------------------------------------===//
// Helper function for label retrieval.
//===--------------------------------------------------------------------===//

/// Return the BasicBlock's name if available. If not, use its representation as a string
/// (adapted from CFGPrinter.h).
std::string getBBID(const BasicBlock &BB);

/// Return the ID for a Loop. Loops are identified by using the header's ID. This ID is unique
/// inside a function.
std::string getLoopID(const Loop &L);

//===--------------------------------------------------------------------===//
// Helper functions for FPOps count mode.
//===--------------------------------------------------------------------===//

/// Return true if any operand or return value of this function uses floating-point type.
bool hasFPOperandOrReturn(const Function &F);

/// Return true if any operand of this instruction uses floating-point type.
bool hasFPOperand(const Instruction &I);

}

#endif
