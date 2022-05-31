import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# 文件名 储存名 x轴键值 y轴键值 标题  颜色 横坐标 纵坐标
# python visualize.py t v v-t red t[s] v[m/s]

if __name__ =='__main__':
    arg = []
    for k in range(9):
        arg.append(sys.argv[k])
    file_name = str(arg[1])
    save_name = str(arg[2])
    first_element = str(arg[3])# x轴
    second_element = str(arg[4])# y轴
    title = str(arg[5])
    color = str(arg[6])
    xlabel = str(arg[7])
    ylabel = str(arg[8])

    path = os.path.dirname(os.path.abspath(__file__))
    data = pd.read_csv(path + "/data/" + file_name + ".csv")
    fig = plt.figure()
    ax = plt.axes()
    ax.plot(data[first_element], data[second_element], color=color, label=title)    
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(file_name)
    plt.legend()
    plt.grid()
    plt.savefig(path + "/pic/" + save_name + ".jpg")
    plt.show()
    
