#!/bin/bash
source ~/.bashrc
source ~/miniconda3/bin/activate
conda activate droid
cd /home/x02lite/deploy/
python sim2real.py
