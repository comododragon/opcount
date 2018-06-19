//===-- lib/Transforms/OpCount/helper.cpp --------------------- -*- C++ -*-===//
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
// This source contains implementation of several helper functions
// used by this pass.
//
//===----------------------------------------------------------------------===//

// Local includes
#include "helper.h"

#include "llvm/Pass.h"
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/IR/Module.h"

using namespace llvm;

namespace opcountutils {

//===--------------------------------------------------------------------===//
// Helper functions for fancy console printing.
//===--------------------------------------------------------------------===//

/// Generate a shiny separator in the console log.
std::string generateSeparator(unsigned int lineWidth) {
	return std::string(lineWidth + 4, '*') + "\n";
}

/// Generate a fancy bordered line in the console log. Level defines the amount of space to put before the string.
/// If trim is true, the input string will be trimmed to fit in lineWidth + level.
std::string generateLine(std::string line, unsigned int level, bool trim, unsigned int lineWidth) {
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
// Helper function for label retrieval.
//===--------------------------------------------------------------------===//

/// Return the BasicBlock's name if available. If not, use its representation as a string
/// (adapted from CFGPrinter.h). This ID is unique inside a function.
std::string getBBID(const BasicBlock &BB) {
	// Return the BB name if available
	if(BB.hasName())
		return BB.getName();

	// Else, get the BB representation as a string and return
	std::string name;
	raw_string_ostream OS(name);
	BB.printAsOperand(OS, false);
	return OS.str();
}

/// Return the ID for a Loop. Loops are identified by using the header's ID. This ID is unique
/// inside a function.
std::string getLoopID(const Loop &L) {
	return getBBID(*(L.getHeader()));
}

//===--------------------------------------------------------------------===//
// Helper functions for FPOps count mode.
//===--------------------------------------------------------------------===//

/// Return true if any operand or return value of this function uses floating-point type.
bool hasFPOperandOrReturn(const Function &F) {
	if(F.getReturnType()->isFPOrFPVectorTy())
		return true;

	for(Function::const_arg_iterator ait = F.arg_begin(); ait != F.arg_end(); ait++) {
		const Argument &A = *ait;
		if(A.getType()->isFPOrFPVectorTy())
			return true;
	}

	return false;
}

/// Return true if any operand of this instruction uses floating-point type.
bool hasFPOperand(const Instruction &I) {
	for(User::const_op_iterator oit = I.op_begin(); oit != I.op_end(); oit++) {
		const Value &V = **oit;
		if(V.getType()->isFPOrFPVectorTy())
			return true;
	}

	return false;
}

}
