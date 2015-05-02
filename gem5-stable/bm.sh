#! /bin/bash

build/ARM/gem5.opt configs/example/se.py  --cmd="401.bzip2" --options="input.program 2 > input.random.out 2> input.random.err" --cpu-type="MinorCPU" --caches --l2cache > markov_bzip2.txt
mkdir m5out_bzip2_markov 
cp -r m5out/ m5out_bzip2_markov/

