#A*算法是一个最优路径查找的算法
#可以根据不同动作的损失来找到达到目标最近的方法
#本算法目前就考虑了路径的上下左右移动，尚未涉及斜对角移动
#版本说明 V1.0
#日期   2022/5/13
#作者   Qiyao
#https://gitee.com/C-Qiyao/astar-algorithm-py

import numpy as np
from responses import target
import matplotlib.pyplot as plt
class Astar:
    def __init__(self,grid):
        self.grid=np.array(grid,dtype=np.int32)
        print('Imput Grid:\n',self.grid)
        self.found = False  # 到达标记
        self.resign = False  # 启动标记
        self.cost=1#每移动一步花费的cost
        self.begin_point=np.array([0,0],dtype=np.int32)
        self.target_point=np.array([0,0],dtype=np.int32)
        self.W=len(grid[0])#grid宽度
        self.H=len(grid)#grid高度
        self.shape=(self.H,self.W)
        self.delta=[[-1, 0],  # 上
                     [0, -1],  # 左
                     [1, 0],  # 下
                     [0, 1]]  # 右
        self.heuristic = np.zeros(self.shape,dtype=np.int32)#生成与grid同大小的0矩阵

        #print(self.heuristic)
    def set_start(self,startx,starty):#输入起始点X，Y坐标
        self.begin_point[0]=startx
        self.begin_point[1]=starty
    def set_target(self,targetx,targety):#设置终点函数
        self.target_point[0]=targetx
        self.target_point[1]=targety
    def x_y_inrange(self,xin,yin):#检查x、yin是否超出地图边界
        if xin>=0 and xin<self.H and yin>=0 and yin<self.W:
            return True
        else:
            return False
    def calculate(self):
        assert(self.grid[self.begin_point[0]][self.begin_point[1]]!=1)and(self.grid[self.target_point[0]][self.target_point[1]]!=1)
        #检查起点和终点是不是在障碍物区内
        target_point=self.target_point
        heuristic=self.heuristic
        delta=self.delta
        close_matrix= np.zeros(self.shape,dtype=np.int32)#生成与grid同大小的0矩阵,用来保存周围坐标
        action_matrix= np.zeros(self.shape,dtype=np.int32)#生成与grid同大小的0矩阵，用来保存已经计算过的点
        grid_copy=self.grid.copy()
        #计算H-cost
        for i in range(self.H):#逐个计算与终点的距离值
            for j in range(self.W):
                heuristic[i][j] = abs(i - target_point[0]) + abs(j - target_point[1])
                if grid[i][j] == 1:#如果遇到障碍，默认给一个巨大的值，让他不可取
                    heuristic[i][j] = 999
        self.heuristic=heuristic  
        print('H-coat Grid:\n',self.heuristic)
        x = self.begin_point[0]
        y = self.begin_point[1]
        g = 0
        f = g + heuristic[self.begin_point[0]][self.begin_point[1]]
        cell = np.array([[f, g, x, y]],dtype=np.int32)
        print(cell)

        found = False  # 到达标记
        resign = False  # 启动标记
        steps=1
        while not found and not resign:
            if len(cell)==0:
                resign=True
                print('寻路失败~\n请检查路径是否封闭')
                exit()
            else:
                cell=cell[np.argsort(cell[:,0])]
                next_point=cell[0]
                cell=cell[1:]
                #print(cell)
                #print(next_point)
                ######$$$$$$$
                f=next_point[0]
                g=next_point[1]
                x=next_point[2]
                y=next_point[3]
    
                if x==target_point[0] and y==target_point[1]:
                    found=True
                    self.realcost=cell[0][0]
                    #print(cell)
                else:
                    for i in range(len(delta)):#计算周围四个坐标参数
                        x_new=x+delta[i][0]#x坐标跟新
                        y_new=y+delta[i][1]#y坐标跟新
                        #判断新坐标合法性
                        if self.x_y_inrange(x_new,y_new):
                            if close_matrix[x_new][y_new]==0 and self.grid[x_new][y_new]==0:
                                g_new=g+self.cost
                                f_new=g_new+heuristic[x_new][y_new]
                                c_new=np.array([[f_new,g_new,x_new,y_new]],dtype=np.int32)
                                print('STEPS=',steps,'xnew=',x_new,'ynew=',y_new)
                                cell=np.r_[cell,c_new]#numpy数组中加入新的点位信息
                                #print('new cell\n',cell)
                                close_matrix[x_new][y_new]=1
                                action_matrix[x_new][y_new]=i
                                steps+=1
                
        path=np.array([[target_point[0],target_point[1]]],dtype=np.int32)
        x=target_point[0]
        y=target_point[1]
        
        self.steps=steps
        #开始根据action回溯路径
        while x!=self.begin_point[0] or y!=self.begin_point[1]:
            x_pre=x-delta[action_matrix[x][y]][0]
            y_pre=y-delta[action_matrix[x][y]][1]
            x=x_pre
            y=y_pre
            grid_copy[x][y]=10#路径颜色
            path=np.r_[[[x,y]],path]
        self.path=path
        grid_copy[grid_copy==1]=5#障碍物颜色
        self.path_grid=grid_copy
        #print('path is :\n',path)
        #print(grid_copy)
    def show_mat(self,title):
        plt.matshow(self.path_grid)
        plt.title(title)
        plt.show()




if __name__ == "__main__":
    #grid为地图矩阵，其中1为障碍物，0表示空旷
    grid = [
       [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
       [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0],
       [0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0],
       [0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
       [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1],
       [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0],
       [0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0],
       [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
       [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0],
       [0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0]]
    
    #生成算法实例对象
    k=Astar(grid)
    #设置路径起点
    k.set_start(9,0)
    #设置目标终点
    k.set_target(9,28)
    #计算路径
    k.calculate()
    #矩阵绘制显示
    k.show_mat('Path Output,STEPS='+str(k.steps)+'    COST='+str(k.realcost))