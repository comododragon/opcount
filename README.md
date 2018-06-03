# OpCount LLVM Pass

## Authors

* André Bannwart Perina

## Introduction

The OpCount is an LLVM opt pass that can be used to count instructions in an OpenCL kernel considering the worst scenario. The longest path from a graph in the control-flow graph is used.

## Licence

This project is distributed under the University of Illinois Open Source License. See LICENSE.TXT for details.

## How to compile

LLVM and Clang versions used: 7.0.0

* Build LLVM and Clang by following steps 1 to 13 from this guide (https://llvm.org/docs/GettingStarted.html). Optional repositories are not required;.
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
* Default undefined function count (```-def-undefined-function-count=N```): for functions that are undefined (IR not available), use N;
* Verbose (```-verbose```): print a lot of stuff
* Count mode (```-count-mode=OPT```): select count mode OPT, where OPT may be:
	* ```all```: count all types of IR instructions.

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
* All-count: all instruction types are considered.

## Acknowledgements

The project author would like to thank São Paulo Research Foundation (FAPESP), who funds the research where this project is instered (Grant 2016/18937-7).
