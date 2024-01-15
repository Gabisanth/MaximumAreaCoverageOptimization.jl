from vpython import *
from time import *

myBox = box(color=color.blue, length=1,width=1,height=1, pos=vector(0,-5,0))
wall = box(color=color.white, length=1,width=10,height = 10)
deltaX = 0.1
xPos = 0



while True:
    rate(10)
    xPos = xPos + deltaX
    
    myBox.pos = vector(xPos, 0,0)
    pass