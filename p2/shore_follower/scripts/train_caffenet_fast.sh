#!/usr/bin/env sh

TOOLS=/home/gpu_user/caffe/build/tools
#TOOLS=/cs-share/pradalier/caffe/build/tools
$TOOLS/caffe train \
    --solver=solver_fast.prototxt 2>&1 | tee log.txt
