from queue import Queue,LifoQueue,PriorityQueue
#基本FIFO队列  先进先出 FIFO即First in First Out,先进先出
#maxsize设置队列中，数据上限，小于或等于0则不限制，容器中大于这个数则阻塞，直到队列中的数据被消掉
q = Queue(maxsize=0)

#写入队列数据
q.put(0)
q.put(1)
q.put(2)

#输出当前队列所有数据
print(q.queue)
#取出一个队列数据
a=q.get()
print('取出',a)
print(q.queue)
a=q.get()
print('取出',a)
print(q.queue)
