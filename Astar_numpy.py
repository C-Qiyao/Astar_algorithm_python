import numpy as np
from responses import target

class Astar:
    def __init__(self,grid):
        self.grid=np.array(grid,dtype=np.int32)
        print('Imput Grid:\n',self.grid)
        self.found = False  # 到达标记
        self.resign = False  # 启动标记
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
        self.close_matrix= np.zeros(self.shape,dtype=np.int32)#生成与grid同大小的0矩阵,用来保存周围坐标
        self.action_matrix= np.zeros(self.shape,dtype=np.int32)#生成与grid同大小的0矩阵，用来保存已经计算过的点
        #print(self.heuristic)
    def set_start(self,startx,starty):#输入起始点X，Y坐标
        self.begin_point[0]=startx
        self.begin_point[1]=starty
    def set_target(self,targetx,targety):
        self.target_point[0]=targetx
        self.target_point[1]=targety
    def calculate(self):

        assert(self.grid[self.begin_point[0]][self.begin_point[1]]!=1)and(self.grid[self.target_point[0]][self.target_point[1]]!=1)
        #检查起点和终点是不是在障碍物区内
        target_point=self.target_point
        heuristic=self.heuristic
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

        while not found and not resign:
            if len(cell)==0:
                resign=True
                return
            else:
                cell=abs(np.sort(-cell))
                ######$$$$$$$



if __name__ == "__main__":
    grid = [[0, 1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0], 
            [0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0]]
    k=Astar(grid)
    k.set_start(3,0)
    k.set_target(2,2)
    k.calculate()