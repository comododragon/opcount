//===-- lib/Transforms/OpCount/int4.h ------------------------- -*- C++ -*-===//
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
// Describes a 4-tuple of integers with some useful operators.
//
//===----------------------------------------------------------------------===//

#ifndef LIB_TRANSFORMS_OPCOUNT_INT4_H
#define LIB_TRANSFORMS_OPCOUNT_INT4_H

#include <cstdint>

namespace opcountutils {

class int4 {
	// The famous 4 elements vector
	int64_t v[4];

public:

	/// Create an int4 with all values assigned to 0.
	int4(void);

	/// Create an int4 using an int[4] array.
	int4(int64_t input[4]);

	/// Operator int4 + int4: for i = [0..4], sum the i-th element of this int4 with the i-th element of b.
	int4 operator+(const int4 &b);

	/// Operator int4 * int: for i = [0..4], multiply the i-th element of this int4 by b.
	int4 operator*(const int64_t &b);

	/// Operator int++: increment every value of int4 in 1.
	int4 &operator++(int);

	/// Operator int4 += int4: similar to int4 = int4 + int4.
	int4 &operator+=(const int4 &b);

	/// Operator []: get i-th element of int4.
	int64_t &operator[](int i);
};

}

#endif
