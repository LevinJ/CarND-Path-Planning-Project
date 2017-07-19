
import sys
import os
# from _pickle import dump
sys.path.insert(0, os.path.abspath('..'))

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import math

class MapData(object):
    def __init__(self):
        return
    def get_df(self):
        filepath = '../data/highway_map.csv'
        names = ['x','y','s','dx','dy']
        df = pd.read_csv(filepath, sep=None, header=None, names=names,engine='python')
        print(df.describe())
        return df
    def disp_landmarks(self,df):
        fig, ax = plt.subplots()
        ax.scatter(df['x'], df['y'])
        
        ax.scatter(df['x'][:5], df['y'][:5],color='green')
       
        
        
        ax.set_title('map')
        return
    
    def run(self):
        df = self.get_df()
        self.disp_landmarks(df)
        plt.show()
        return




if __name__ == "__main__":   
    obj= MapData()
    obj.run()
  