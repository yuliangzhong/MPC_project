# Changelog

## 0.1.19
- Re-structured optimizerFORCES function
- Added variant to only dump the solver object into a MAT file without generating code
- Added option to perform some basic anonymization on the problem data

## 0.1.18
- Added static keyword in variable declaration in generated interfaces and main solver
- Added thread_local option in main solver file generation

## 0.1.17
- updated interface to use library files instead of object files

## 0.1.16
- Added support for MinGW mex compiler
- Updated product name and fixed dates
- not saving path for intel libs

## 0.1.15

- Fixed a compilation bug in Matlab versions <= R2016a.
- Added printing of stage dimensions.

## 0.1.14

- Y2F generates MISRA C compliant code.
- MEX and Simulink interfaces are compiled with optimization turned on if `codeoptions.optlevel` > 0.

## 0.1.13

- Y2F uses sparse matrices internally to be able to deal with larger problems.

## 0.1.12

- Fixed a compilation bug in newer Matlab versions

## 0.1.11

- Fixed a compilation bug under Windows

## 0.1.10

- Fixed a compilation bug under Windows when using the new Azure servers

## 0.1.9

- Added `y2f_version` function that returns currently installed version of Y2F

## 0.1.8

- Fixed compilation for Visual Studio 2015 users

## 0.1.7

- Fixed bug in MEX file compilation on Windows machines

## 0.1.6

- Fixed bug affecting parsing of problems with parameters influencing cost


## 0.1.5

- Changed method Y2F uses to store Simulink blocks:  All blocks are now stored in a single library that can be accessed from the Simulink Library Browser. If the user re-generates a solver, the Simulink block gets updated automatically.



## 0.1.4

*skipped*


## 0.1.3

- Added basic ADMM example
- Fixed bug that occurred in Matlab versions < 2014a when generating valid names for solvers


## 0.1.2

- Fixed a glitch that caused the build to fail when no solver libs are shipped
