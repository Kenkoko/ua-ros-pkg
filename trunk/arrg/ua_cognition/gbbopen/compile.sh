#!/bin/bash
ROSLISP_CURRENT_PACKAGE=gbbopen `(rospack find sbcl)`/scripts/run-sbcl.sh --noinform --end-runtime-options --noprint --no-userinit --disable-debugger --load `(rospack find gbbopen)`/load-gbbopen.lisp --end-toplevel-options $*
