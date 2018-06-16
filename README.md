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
* ```helper.cpp|h```: source and header files for helper functions, such as console printing and basic block label retrieval;
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
* ************* v0.3 ************* *
* Author: Andre Bannwart Perina    *
************************************
**************************************************************************
* OpCounter is using these values:                                       *
*  Default trip count: 300                                               *
*  Default inner trip count: 300                                         *
*  Count mode: all                                                       *
*  Verbose: 0                                                            *
*  Default undefined function count: 10                                  *
**************************************************************************
**************************************************************************
* Found __kernel function: olar                                          *
*    Found function: mulAdd                                              *
*    Found function: mulAdd                                              *
*    Found function: mulAdd                                              *
*   Found function: get_global_id                                        *
**************************************************************************
**************************************************************************
* Count mode: all                                                        *
* Longest path for __kernel function is 907354                           *
**************************************************************************
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

As an example, to calculate naive operational intensity using default inner trip count 32:
```
$ opt -load /path/to/libLLVMOpCount.so -opcount -count-mode=noi -def-inner-trip-count=32 < kernel.ll > /dev/null
```

A report similar to this will be printed:
```
************************************
* OpCounter: IR Operations Counter *
* ************* v0.3 ************* *
* Author: Andre Bannwart Perina    *
************************************
**************************************************************************
* OpCounter is using these values:                                       *
*  Default trip count: 300                                               *
*  Default inner trip count: 32                                          *
*  Count mode: noi                                                       *
*  Verbose: 0                                                            *
*  Default undefined function count: 10                                  *
*  Default undefined function byte store count: 12                       *
**************************************************************************
**************************************************************************
* Found __kernel function: olar                                          *
*    Found function: mulAdd                                              *
*    Found function: mulAdd                                              *
*    Found function: mulAdd                                              *
*   Found function: get_global_id                                        *
**************************************************************************
**************************************************************************
* Count mode: noi                                                        *
* Longest path for __kernel function is 907354                           *
* Number of bytes transferred in this path is 1241876                    *
* Naive operational intensity is 1.368679 bytes/insts                    *
**************************************************************************
```

By using the ```-verbose``` flag, informations such as loop trip counts inferring and longest path for each function are exposed.

## Description

This opt pass is mainly divided into two steps:

* Calculate the number of instructions in each basic block;
* Find the longest path in the control-flow graph considering known trip counts for loops or default values when loops bounds are unknown;

Finding the longest path in a graph with cycles is NP-Hard. However, by using finite trip counts in the back-edges, it is possible to simplify
the graph into acyclic. The flow of OpCount is:

* Calculate trip counts for all loops in this control-flow graph. If it's not possible to calculate, use a default value;
* Find all loop back-edges in this control-flow graph;
* Connect all leaf basic blocks ```l``` to terminator nodes;
* Add weights to the CFG edges as follows: count specified metric in a basic block ```u``` (e.g. ```all``` count mode would return the number of instructions inside ```u```). Use this count as weight for all edges leaving ```u```. If ```u``` is inside one or more loops, multiply the count result by all loop's trip counts that contains ```u```;
	* If ```u``` contains a function call, this procedure is recursively called to handle such function as well;
* Use the longest path search algorithm for acyclic graphs, ignoring back-edges.

For now, current counts are supported:
* All count: all instruction types are considered.
* Floating-point count: only floating-point arithmetic and control-flow instructions are considered (IN PROGRESS);
* Naive operational intensity: after traversing the longest path using all count, divide the number of bytes transferred from/to memory in this path by the all instruction count;
* Naive memory intensity: after traversing the longest path considering bytes transferred from/to memory, divide this number by the all instruction count in this path.

## Acknowledgements

The project author would like to thank São Paulo Research Foundation (FAPESP), who funds the research where this project is inserted (Grant 2016/18937-7).
