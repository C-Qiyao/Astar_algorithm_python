from PIL import Image
import numpy as np
im=Image.open('migong.png')
Img = im.convert('L')
threshold = 200
table = []
for i in range(256):
    if i < threshold:
        table.append(1)
    else:
        table.append(0)
#plt.imshow(im)
photo = Img.point(table, '1')
print(photo)
imag=np.array(photo,dtype=int)
print(imag)
