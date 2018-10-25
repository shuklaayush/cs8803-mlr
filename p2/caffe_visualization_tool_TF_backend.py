#!/usr/bin/env python
import numpy
import os
import time
#import tensorflow as tf
from argparse import ArgumentParser
import matplotlib.pyplot as plt

"""
### README ###

This code helps to interprate caffes outputs by ploting them.
Two backends are available:
    ++ MATPLOTLIB with PYPLOT:
        For that solution to work you MUST logon to the GPU using ssh -X
    ++ TENSORFLOW with TENSORBOARD (BROKEN)
        For that solution to work you must use tensorboard see running
        examples

To run that script with MATPLOTLIB do:
    ssh -X yourname@192.93.8.19(1-2) (forwards the display)
    ./your_caffe_script 2&>1 | tee path2savefile.txt
    python caffe_visualization_tool_TF_backend.py --txt_file path2savefile.txt

    alternatively one can use

    python caffe_visualization_tool_TF_backend.py --txt_file path2savefile.txt
    --update_freq X 

    This increases the refreshment speed to XHz, the default value is 0.1

To run that script with tensorflow do:
    Nothing ! It does not work (yet) !
"""

def get_file_as_list(fname):
    with open(fname) as f:
        content = f.readlines()
    return content

class Plot_Loss_Acc:
    def __init__(self):
        self.parser = self.make_parser()
        self.args = self.parser.parse_args()
        # Data placeholder
        self.data = None
        # Losses/Iteration/Accuracy placeholders
        self.train_iteration = None
        self.train_loss = None
        self.train_accuracy = None
        self.test_loss = None
        self.test_iteration = None
        self.test_accuracy = None
        # Run
        self.run()

    def make_parser(self):
        p = ArgumentParser()
        p.add_argument('--txt_file',
                type = str,
                required = True,
                help='full path to the caffe output file: /path/2/file.txt')
        p.add_argument('--update_freq',
                type = float,
                required = False,
                default = 0.1,
                help='Update frequency in Hertz: 0.1 updates every 10 seconds')
        p.add_argument('--tensorflow_backend',
                type = bool,
                required = False,
                default = False,
                help='uses tensorflow backend to generate figures instead of matplotlib')
        p.add_argument('--output_dir',
                type = str,
                required = False,
                help='full path to the output directory file, only needed when using the tensorflow backend')
        return p

    def run_check(self):
        if not os.path.exists(self.args.txt_file):
            print('Specified caffe output file does not exist please check the provided path...')
            print('Exiting...')
            exit(1)
        else:
            print('Caffe output file found !')
        if self.args.tensorflow_backend:
            if not os.path.exists(self.args.output_dir):
                print('Output folder not found...')
                print('Creating one for you...')
                try:
                    os.mkdir(self.args.output_dir)
                except:
                    print('Failed to create folder please provide a correct path')
                    print('Exiting...')
                    exit(1)
            else:
                print('Output directory found !')
        checkOK = True
        return checkOK

    def extract_test_loss(self):
        self.test_loss = []
        self.test_iteration = []
        for line in self.data:
            iteration_string = line.split('Testing net (#0)')
            if len(iteration_string) > 1:
                iteration = self.get_iteration('Iteration', line, self.test_iteration)
                if iteration:
                    self.test_iteration.append(iteration)
            loss = self.get_loss('Test net output #1:', line)
            if loss:
                self.test_loss.append(loss)

    def extract_train_loss(self):
        self.train_loss = []
        self.train_iteration = []
        for line in self.data:
            iteration = self.get_iteration('Iteration', line, self.train_iteration)
            if iteration:
                self.train_iteration.append(iteration)
            loss = self.get_loss('Train net output #0:', line)
            if loss:
                self.train_loss.append(loss)
     
    def extract_test_accuracy(self):
        self.test_accuracy = []
        for line in self.data:
            L = line.split('Test net output #0:')
            if len(L) > 1:
                L2 = L[1].split('=')
                L3 = L2[1].split('\n')
                self.test_accuracy.append(float(L3[0]))

    def get_iteration(self, string, line, prev):
        iteration_string = line.split('Iteration')
        if len(iteration_string) > 1:
            iteration_string2 = iteration_string[1].split(' ')
            iteration_int = int(iteration_string2[1].strip(','))
            if len(prev) > 1:
                if prev[-1] != iteration_int:
                    return iteration_int
                else:
                    return False
            else:
                return iteration_int
        else:
            return False
    
    def get_loss(self, string, line):
        loss_string = line.split(string)
        if len(loss_string) > 1:
            loss_string2 = loss_string[1].split('=')
            loss_float = loss_string2[1].split('(*')
            return float(loss_float[0])
        else:
            return False
    
    def update(self):
        self.data = get_file_as_list(self.args.txt_file)
        self.extract_test_loss()
        self.extract_train_loss()
        self.extract_test_accuracy()
        
    def run(self):
        self.run_check()
        if self.args.tensorflow_backend:
            print('BROKEN')
            exit(1)
        else:
            self.run_mpl()

    def run_mpl(self):
        plt.ion()
        # Create figure
        f = plt.figure()
        graph_loss = f.add_subplot(211)
        graph_acc = f.add_subplot(212, sharex=graph_loss)
        # Loop for eveeeeer
        while True:
            # Acquires data
            self.update()
            # Updates plots
            graph_loss.set_title('Loss')
            test_min = min(len(self.test_loss),len(self.test_iteration),len(self.test_accuracy))
            train_min = min(len(self.train_loss),len(self.train_iteration))
            graph_loss.plot(self.test_iteration[:test_min],self.test_loss[:test_min],label='test_loss')
            graph_loss.plot(self.train_iteration[:train_min],self.train_loss[:train_min],label='train_loss')
            handles, labels = graph_loss.get_legend_handles_labels()
            graph_loss.legend(handles, labels)
            graph_acc.set_title('Accuracy')
            graph_acc.plot(self.test_iteration[:test_min],self.test_accuracy[:test_min],label="test_accuracy")
            f.canvas.draw()
            f.canvas.flush_events()
            # Wait 
            time.sleep(1/self.args.update_freq)
            # Clears graphs
            graph_loss.clear()
            graph_acc.clear()

if __name__ == '__main__':
    PLA = Plot_Loss_Acc()
    PLA.run()
