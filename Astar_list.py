#A*算法是一个最优路径查找的算法
#可以根据不同动作的损失来找到达到目标最近的方法
#本算法目前就考虑了路径的上下左右移动，尚未涉及斜对角移动
#使用list替换numpy，比较速度影响
#版本说明 V1.1
#日期   2022/5/20
#作者   Qiyao
#https://gitee.com/C-Qiyao/astar-algorithm-py
from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
import time
#图像二值化
def read_img(filename):
    im=Image.open(filename)
    Img = im.convert('L')
    threshold = 200
    table = []
    for i in range(256):
        if i < threshold:
            table.append(1)
        else:
            table.append(0)
    photo = Img.point(table, '1')
    imag=np.array(photo,dtype=np.int0)
    return imag.tolist()

class Astar:
    def __init__(self,grid):
        self.grid=grid
        print('Imput Grid:\n',len(self.grid),len(self.grid[0]))
        self.found = False  # 到达标记
        self.resign = False  # 启动标记
        self.cost=1#每移动一步花费的cost
        self.begin_point=[0,0]
        self.target_point=[0,0]
        self.W=len(grid[0])#grid宽度
        self.H=len(grid)#grid高度
        self.shape=(self.H,self.W)
        self.delta=[[-1, 0],  # 上
                     [0, -1],  # 左
                     [1, 0],  # 下
                     [0, 1]]  # 右
        self.heuristic = [[0 for row in range(self.W)] for col in range(self.H)]#生成与grid同大小的0矩阵
        self.close_matrix= [[0 for row in range(self.W)] for col in range(self.H)]#生成与grid同大小的0矩阵,用来保存周围坐标
        self.action_matrix= [[0 for row in range(self.W)] for col in range(self.H)]#生成与grid同大小的0矩阵，用来保存已经计算过的点

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
        delta=self.delta
        grid_copy=np.array(self.grid.copy(),dtype=np.int32)#生成grid的副本用于添加路径信息以及可视化内容
        #print(grid_copy)
        #计算H-cost
        self.start=time.time()
        for i in range(self.H):#逐个计算与终点的距离值
            for j in range(self.W):
                self.heuristic[i][j] = abs(i - target_point[0]) + abs(j - target_point[1])
                if grid[i][j] == 1:#如果遇到障碍，默认给一个巨大的值，让他不可取
                    self.heuristic[i][j] = 999999 
        #print('H-coat Grid:\n',self.heuristic)
        x = self.begin_point[0]
        y = self.begin_point[1]
        g = 0
        f = g + self.heuristic[self.begin_point[0]][self.begin_point[1]]
        cell = [[f, g, x, y]]
        #print(cell)
        #print(cell)

        found = False  # 到达标记
        resign = False  # 启动标记
        steps=1
        while not found and not resign:
            if len(cell)==0:
                resign=True
                print('寻路失败~\n请检查路径是否封闭')
                exit()
            else:
                cell.sort()  # 从大到小排序
                cell.reverse()#反转排序方向，从小到大
                next_point = cell.pop()#取出当前的最小值
                #print(next_point)
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
                            if self.close_matrix[x_new][y_new]==0 and self.grid[x_new][y_new]==0:
                                g_new=g+self.cost
                                f_new=g_new+self.heuristic[x_new][y_new]
                                c_new=[f_new,g_new,x_new,y_new]
                                #print('STEPS=',steps,'xnew=',x_new,'ynew=',y_new)
                                cell.append(c_new)#数组中加入新的点位信息
                                #print('new cell\n',cell)
                                self.close_matrix[x_new][y_new]=1
                                self.action_matrix[x_new][y_new]=i
                                steps+=1
                
        path=[[target_point[0],target_point[1]]]
        x=target_point[0]
        y=target_point[1]
        self.steps=steps
        #开始根据action回溯路径
        while x!=self.begin_point[0] or y!=self.begin_point[1]:
            x_pre=x-delta[self.action_matrix[x][y]][0]
            y_pre=y-delta[self.action_matrix[x][y]][1]
            x=x_pre
            y=y_pre
            grid_copy[x][y]=10#路径颜色
            path.append([x,y])
        self.end=time.time()
        path_zheng=[]#此处pasth为倒叙
        for i in range(len(path)):
            path_zheng.append(path[len(path) - 1 - i])
        grid_copy[grid_copy==1]=5#障碍物颜色
        self.path_grid=grid_copy

    def show_mat(self,title,mat):
        plt.matshow(mat)
        plt.title(title)
        plt.show()




if __name__ == "__main__":
    #grid为地图矩阵，其中1为障碍物，0表示空旷
    '''grid = [
       [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
       [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0],
       [0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0],
       [0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
       [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1],
       [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0],
       [0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0],
       [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
       [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0],
       [0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0]]'''
    if 1:#方便改成while
        grid= read_img('bigmigong.png')
    #生成算法实例对象
        k=Astar(grid)
    #设置路径起点
        k.set_start(0,0)
    #设置目标终点
    #k.set_target(1100,1788)
        k.set_target(k.H-1,k.W-1)#设置终点为右下角
    #计算路径
        k.calculate()
    #矩阵绘制显示
        k.show_mat('Path Output,STEPS='+str(k.steps)+'    COST='+str(k.realcost)+'   Time cost='+str(round((k.end-k.start),3))+'s',k.path_grid)