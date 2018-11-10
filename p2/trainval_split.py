#!/usr/bin/env python3

"""
Split input data to training and validation.
"""

import os
import random
import shutil
from argparse import ArgumentParser

#  ROOT = os.path.dirname(os.path.realpath(__file__))
ROOT = os.getcwd()
LABEL_FILE = 'labels.txt'
TRAIN_DIR = os.path.join(ROOT, 'train')
VAL_DIR = os.path.join(ROOT, 'val')

parser = ArgumentParser()
parser.add_argument('--split', type=float, required=False, default=0.2, help='Fraction of data to put in validation set')

if __name__ == '__main__':
    args = parser.parse_args()
    if not os.path.exists(LABEL_FILE):
        print("Error: Labels file not found")
        exit()
    with open(os.path.join(ROOT, LABEL_FILE)) as f:
        labels = f.read().splitlines()
    random.shuffle(labels)
    ix = int(args.split * len(labels))
    val = sorted(labels[:ix])
    train = sorted(labels[ix:])

    if os.path.exists(VAL_DIR):
        shutil.rmtree(VAL_DIR)
    os.makedirs(VAL_DIR)
    with open(os.path.join(VAL_DIR, LABEL_FILE), 'w') as f:
        f.write('\n'.join(val))
    for line in val:
        filename = line.split()[0]
        dst = os.path.join(VAL_DIR, filename)
        try:
            shutil.copyfile(os.path.join(ROOT, filename), dst)
        except FileNotFoundError:
            os.makedirs(os.path.dirname(dst))
            shutil.copyfile(os.path.join(ROOT, filename), dst)

    if os.path.exists(TRAIN_DIR):
        shutil.rmtree(TRAIN_DIR)
    os.makedirs(TRAIN_DIR)
    with open(os.path.join(TRAIN_DIR, LABEL_FILE), 'w') as f:
        f.write('\n'.join(train))
    for line in train:
        filename = line.split()[0]
        dst = os.path.join(TRAIN_DIR, filename)
        try:
            shutil.copyfile(os.path.join(ROOT, filename), dst)
        except FileNotFoundError:
            os.makedirs(os.path.dirname(dst))
            shutil.copyfile(os.path.join(ROOT, filename), dst)
