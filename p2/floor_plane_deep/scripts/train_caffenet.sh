#!/usr/bin/env sh

SOLVER=solver.prototxt

TOOLS=/home/gpu_user/caffe/build/tools
#TOOLS=/cs-share/pradalier/caffe/build/tools
$TOOLS/caffe train \
    --solver=$SOLVER 2>&1 | tee log.txt
