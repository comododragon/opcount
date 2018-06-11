//===-- lib/Transforms/OpCount/FunctionsDescription.h --------- -*- C++ -*-===//
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
// Caches longest path for functions and some other useful information.
//
//===----------------------------------------------------------------------===//

#ifndef LIB_TRANSFORMS_OPCOUNT_FUNCTIONSDESCRIPTION_H
#define LIB_TRANSFORMS_OPCOUNT_FUNCTIONSDESCRIPTION_H

// Local includes
#include "int4.h"

#include <map>
#include <vector>

namespace opcountutils {

/// Function descriptor. The int[0] value defines this function's longest path.
/// The other 3 int values can carry some useful information.
typedef std::pair<std::string, int4> FunctionDescriptionPair;

/// Defines a dictionary of FunctionDescriptionPairs. Each function is indexed by its name (((Function *) F)->getName()).
typedef std::map<std::string, int4> FunctionsDescription;

/// Defines a pair source-destination for a back-edge.
typedef std::pair<std::string, std::string> BackedgePair;

/// Defines a vector containing all back-edges for a given function.
typedef std::vector<BackedgePair> FunctionBackedgesPairs;

}

#endif
