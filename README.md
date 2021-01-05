# qp_spline_demo

## introduction
This repo simply achieve qp spline algorithm base on apollo 3.5 source code. Apollo code has universal interface for everyone but I only use part of it. So I simplified part of it.

## Link
* [spline concept](https://en.wikipedia.org/wiki/Spline_(mathematics))


* [qpOASES](https://github.com/coin-or/qpOASES)

## Installation list
* [glog](https://github.com/google/glog#building-from-source) 

* [gflags](https://gflags.github.io/gflags/#cmake)

* [qpOASES-3.2](https://github.com/coin-or/qpOASES/releases) <br>
  ( mkdir bin && cmake .. <br>
  cp libs/libqpOASES.a /usr/local/lib/ <br>
  cp -r include/* /usr/local/include/ )

* [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page) <br>
  ( cp -r Eigen/ /usr/local/include/)
