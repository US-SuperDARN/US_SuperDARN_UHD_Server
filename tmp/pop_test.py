#!/usr/bin/env python3

import numpy as np
from numpy import sin,cos

import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter, MaxNLocator
import matplotlib
import datetime
import argparse
import os
import copy


test_list=[]

def fill_test(j):
    class test_class:
        a=[]
        b=[]
        
    test=test_class()
    for i in range(3):
        test.a.append(10*j+i)
        test.b.append(i+j)

    return(test)


        
for j in range(3):
    test_list.insert(0,fill_test(j))

atest=test_list.pop()
print("Popped:",atest.a,atest.b)
    
for atest in test_list:
    print(atest.a,atest.b)

    

