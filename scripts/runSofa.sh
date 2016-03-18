#!/usr/bin/env bash
SELF=`readlink -f $0`
SOFA_ROOT=`dirname $SELF`/..
LD_LIBRARY_PATH=$SOFA_ROOT/lib exec $SOFA_ROOT/bin/runSofa $* -msaa 8
