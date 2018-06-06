# OpCount LLVM Pass

## Authors

* André Bannwart Perina

## Introduction

The OpCount is an LLVM opt pass that can be used to count several metrics in an OpenCL kernel, considering the worst case scenario by using the longest path from the control-flow graph. The longest path is calculated using the selected metric.

## Licence

This project is distributed under the University of Illinois Open Source License. See LICENSE.TXT for details.

## Algorithms used in this project

The following algorithms were used and adapted:

* https://www.geeksforgeeks.org/find-longest-path-directed-acyclic-graph/
* https://www.geeksforgeeks.org/iterative-depth-first-traversal/

## Files description

This repository should be placed inside the LLVM source tree under the folder ```/path/to/llvm/sources/lib/Transforms/OpCount``` as described in the next section. This repository is composed by:

* ```README.md```: this file;
* ```LICENSE.txt```: description of licence used by this repository;
* ```CMakeLists.txt```: CMakeList file for generating build files;
* ```LoopsDescription.cpp|h```: source and header files for LoopsDescription type;
* ```FunctionsDescription.h```: header file for FunctionsDescription type;
* ```int4.cpp|h```: source and header files for int4 (4-tuple integer) class;
* ```OpCount.cpp|h```: source and header files for the pass itself and some inner classes.

## How to compile

LLVM and Clang versions used: 7.0.0

* Build LLVM and Clang by following steps 1 to 13 from this guide (https://llvm.org/docs/GettingStarted.html). Optional repositories are not required;
* Access your LLVM source directory:
```
$ cd /path/to/llvm/sources
```
* Clone this repository:
```
$ cd lib/Transforms/
$ git clone https://github.com/comododragon/opcount.git OpCount
```
* Append the following line to ```lib/Transforms/CMakeLists.txt```:
```
add_subdirectory(OpCount)
```
* Go to your LLVM build directory and build:
```
$ cd /path/to/llvm/build
$ make
```
* The library will be compiled in ```/path/to/llvm/build/lib/libLLVMOpCount.so```.

## How to use

* This pass is performed on LLVM IR. To transform an OpenCL kernel code ```kernel.cl``` to IR ```kernel.ll```:
```
$ clang -S -emit-llvm -x cl -include clc/clc.h kernel.cl -o kernel.ll
```
* Load the shared library into opt and execute the pass:
```
$ opt -load /path/to/libLLVMOpCount.so -opcount < kernel.ll > /dev/null
```
* A report should be printed similar to:
```
************************************
* OpCounter: IR Operations Counter *
* ************* v0.1 ************* *
* Author: Andre Bannwart Perina    *
************************************
****************************************************************
* OpCounter is using these values:                             *
*  Default Trip Count: 300                                     *
*  Default Inner Trip Count: 300                               *
*  Default Undefined Function Count: 10                        *
*  Verbose: 0                                                  *
****************************************************************
****************************************************************
* Found __kernel function: olar                                *
*    Found function: mulAdd                                    *
*    Found function: mulAdd                                    *
*    Found function: mulAdd                                    *
*   Found function: get_global_id                              *
****************************************************************
****************************************************************
* Longest path for __kernel function is 907354                 *
****************************************************************
```

Some arguments can be passed to the pass:

* Default trip count (```-def-trip-count=N```): for top-level loops with unknown trip count, use N;
* Default inner trip count (```-def-inner-trip-count=N```): for non top-level loops with unknown trip count, use N;
* Default undefined function count (```-def-undefined-function-count=N```): for functions that are undefined (IR not available), use N as longest path count;
* Count mode (```-count-mode=OPT```): select count mode OPT, where OPT may be:
	* ```all```: count all types of IR instructions.
	* ```fp```: count only floating-point arithmetic (IN PROGRESS);
	* ```noi```: naive operational intensity. Use all instruction count for longest path, but count the number of bytes transferred from/to memory (any address space) along this path as well. At last print the number of bytes divided by the number of total instructions in the path. If an undefined function is found, default undefined function count is used for instruction count. It is assumed that 30% of such instructions are load/stores transferring 4 bytes each;
	* ```nmi```: naive memory intensity. Use the number of bytes transferred from/to memory (any address space) for longest path, but count all instructions along this path as well. At last print the number of bytes transferred divided by the number of total instructions in the path. If an undefined function is found, default undefined function count is used for instruction count. It is assumed that 30% of such instructions are load/stores transferring 4 bytes each.
* Verbose (```-verbose```): print a lot of stuff.

## Description

This opt pass is mainly divided into two steps:

* Calculate the number of instructions in each basic block;
* Find the longest path in the control-flow graph considering known trip counts for loops or default values when loops bounds are unknown;

Finding the longest path in a graph with cycles is NP-Hard. However, simplifications in the control-flow graphs can be performed to remove cycles:

* Find all loops in the control-flow graph;
* Find the longest path for all innermost loops (IL) considering the partial control-flow graph related to this loop (i.e. there are no cycles);
* For all loops (CL) containing the loops IL, substitute the partial control-flow graph related to IL by an abstract basic block with the longest path cost for IL multiplied by IL trip count (if known, otherwise use a customisable default value), thus removing IL back-edge;
* Perform the previous step by iteratively increasing the loop nesting level;
* After all loops were calculated and substituted by abstract basic blocks, the kernel function control-flow graph won't have any back edges. Therefore, the longest path algorithm for acyclic directed graphs can be applied.

For now, current counts are supported:
* All count: all instruction types are considered.
* Floating-point count: only floating-point arithmetic and control-flow instructions are considered (IN PROGRESS);
* Naive operational intensity: After traversing the longest path using all count, divide the number of bytes transferred from/to memory in this path by the all instruction count;
* Naive memory intensity: After traversing the longest path considering bytes transferred from/to memory, divide this number by the all instruction count in this path.

## Acknowledgements

The project author would like to thank São Paulo Research Foundation (FAPESP), who funds the research where this project is insetred (Grant 2016/18937-7).
