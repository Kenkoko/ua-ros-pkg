#!/bin/bash
ROSLISP_CURRENT_PACKAGE=gbbopen sbcl --noinform --end-runtime-options --noprint --no-userinit --disable-debugger --load `(rospack find gbbopen)`/load-gbbopen.lisp --end-toplevel-options $*
