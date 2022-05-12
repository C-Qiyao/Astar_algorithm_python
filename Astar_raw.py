def a_star_search(grid: list, begin_point: list, target_point: list, cost=1):
    assert ((grid[begin_point[0]][begin_point[1]] != 1) and (grid[target_point[0]][target_point[1]] != 1))

    # 终点距离计算函数
    heuristic = [[0 for row in range(len(grid[0]))] for col in range(len(grid))]#生成与grid同大小的0矩阵
    for i in range(len(grid)):#逐个计算与终点的距离值
        for j in range(len(grid[0])):
            heuristic[i][j] = abs(i - target_point[0]) + abs(j - target_point[1])
            if grid[i][j] == 1:#如果遇到障碍，默认给一个巨大的值，让他不可取
                heuristic[i][j] = 99  

    # 运动指令表
    delta = [[-1, 0],  # 上
             [0, -1],  # 左
             [1, 0],  # 下
             [0, 1]]  # 右

    close_matrix = [[0 for col in range(len(grid[0]))] for row in range(len(grid))]  # the referrence grid
    close_matrix[begin_point[0]][begin_point[1]] = 1
    action_matrix = [[0 for col in range(len(grid[0]))] for row in range(len(grid))]  # the action grid

    x = begin_point[0]
    y = begin_point[1]
    g = 0
    f = g + heuristic[begin_point[0]][begin_point[1]]
    cell = [[f, g, x, y]]

    found = False  # 到达标记
    resign = False  # 启动标记

    while not found and not resign:
        if len(cell) == 0:
            resign = True
            return None, None
        else:
            cell.sort()  # 从大到小排序
            cell.reverse()#反转排序方向，从小到大
            next = cell.pop()#取出当前的最小值
            x = next[2]
            y = next[3]
            g = next[1]
            f = next[0]

            if x == target_point[0] and y == target_point[1]:
                found = True
            else:
                # delta have four steps
                for i in range(len(delta)):  # to try out different valid actions
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]):  # 判断可否通过那个点
                        if close_matrix[x2][y2] == 0 and grid[x2][y2] == 0:
                            g2 = g + cost
                            f2 = g2 + heuristic[x2][y2]
                            cell.append([f2, g2, x2, y2])
                            close_matrix[x2][y2] = 1
                            action_matrix[x2][y2] = i
    invpath = []#运行操作记录
    x = target_point[0]
    y = target_point[1]
    invpath.append([x, y])  # 计算回溯经过的路径点
    while x != begin_point[0] or y != begin_point[1]:
        x2 = x - delta[action_matrix[x][y]][0]
        y2 = y - delta[action_matrix[x][y]][1]
        x = x2
        y = y2
        invpath.append([x, y])

    path = []
    for i in range(len(invpath)):
        path.append(invpath[len(invpath) - 1 - i])
    return path, action_matrix


if __name__ == "__main__":
	# 0为自由通行节点，1为障碍
    grid = [[0, 1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0], 
            [0, 1, 0, 0, 0, 0],
            [0, 1, 0, 0, 1, 0],
            [0, 0, 0, 0, 1, 0]]

    begin = [3, 0]  # 3行0列为起始点
    target = [1, 4]  # 1行4列为终点

    a_star_path, action_matrix = a_star_search(grid, begin, target)
    for path in a_star_path:
        print(path)
