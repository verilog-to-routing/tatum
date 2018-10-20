# Tatum: A Fast, Flexible Static Timing Analysis (STA) Engine for Digital Circuits

[![Build Status](https://travis-ci.org/kmurray/tatum.svg?branch=master)](https://travis-ci.org/kmurray/tatum)

## Overview
Tatum is primarily a library (`libtatum`) which provides a fast and flexible Static Timing Analysis (STA) engine for digital circuits.

Tatum is a block-based timing analyzer suitable for integration with Computer-Aided Design (CAD) tools used to optimize and implement digital circuits.
Tatum supports both setup (max) and hold (min) analysis, clock skew and multiple clocks.

Tatum operates on an abstract *timing graph* constructed by the host application, and can be configured to use an application defined delay calculator.

Tatum is optimized for high performance, as required by optimizing CAD tools.
In particular:
  * Tatum performs only a single set of graph traversals to calculate timing information for all clocks.
  * Tatum's data structures are cache optimized
  * Tatum supports parallel analysis using multiple CPU cores

## How to Cite
If your work uses Tatum please cite the following paper as a general citation:

K. E. Murray and V. Betz, "Tatum: Parallel Timing Analysis for Faster Design Cycles and Improved Optimization", *IEEE International Conference on Field-Programmable Technology (FPT)*, 2018

Bibtex:
```
@inproceedings{c:tatum,
    author = {Murray, Kevin E. and Betz, Vaughn},
    title = {Tatum: Parallel Timing Analysis for Faster Design Cycles and Improved Optimization},
    booktitle = {IEEE International Conference on Field-Programmable Technology (FPT)},
    year = {2018}
}
```

## Download
Comming soon.

## Documentation
Comming soon.

## Uses of Tatum

Tatum is designed to be re-usable in a variety of appliations.

Some of the known uses are:
  * The [Verilog to Routing (VTR)](https://verilogtorouting.org) project for Field-Programmable Gate Array (FPGA) Architecture and CAD research. Tatum is used as the STA engine in the VPR placement and routing tool.
  * The [CGRA-ME](http://cgra-me.ece.utoronto.ca/) framework for Coarse-Grained Reconfigurable Array (CGRA) Architecture research.

## History

### Why was Tatum created?
I had need for a high performance, flexible STA engine for my research into FPGA architecture and CAD tools.
I could find no suitable open source STA engines, and wrote my own.

### Name Origin
A *tatum* is a unit of time used in the computational analysis of music \[[1]\], named after Jazz pianist [Art Tatum](https://en.wikipedia.org/wiki/Art_Tatum).

[1]: http://web.media.mit.edu/~tristan/phd/dissertation/chapter3.html#x1-390003.4.3
