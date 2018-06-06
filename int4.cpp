//===-- lib/Transforms/OpCount/int4.cpp ----------------------- -*- C++ -*-===//
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

#include "int4.h"

namespace opcountutils {

/// Create an int4 with all values assigned to 0.
int4::int4(void) {
	for(int i = 0; i < 4; i++)
		this->v[i] = 0;
}

/// Create an int4 using an int[4] array.
int4::int4(int input[4]) {
	for(int i = 0; i < 4; i++)
		this->v[i] = input[i];
}

/// Operator int4 + int4: for i = [0..4], sum the i-th element of this int4 with the i-th element of b.
int4 int4::operator+(const int4 &b) {
	int4 result;
	for(int i = 0; i < 4; i++)
		result[i] = this->v[i] + b.v[i];
	return result;
}

/// Operator int4 * int: for i = [0..4], multiply the i-th element of this int4 by b.
int4 int4::operator*(const int &b) {
	int4 result;
	for(int i = 0; i < 4; i++)
		result[i] = this->v[i] * b;
	return result;
}

/// Operator int++: increment every value of int4 in 1.
int4 &int4::operator++(int) {
	for(int i = 0; i < 4; i++)
		(this->v[i])++;
	return *this;
}

/// Operator int4 += int4: similar to int4 = int4 + int4.
int4 &int4::operator+=(const int4 &b) {
	for(int i = 0; i < 4; i++)
		this->v[i] += b.v[i];
	return *this;
}

/// Operator []: get i-th element of int4.
int &int4::operator[](int i) {
	return v[i];
}

}
