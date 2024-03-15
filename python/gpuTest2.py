 
from numba import jit, cuda
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.colors import Colormap
# to measure exec time
from timeit import default_timer as timer   

# normal function to run on cpu
def pointsToImage(points,image):                                
    for i in range(points.shape[0]):
        image[points[i,0],points[i,1]] = 1
 
# function optimized to run on gpu 
@jit(target_backend='cuda')                         
def pointsToImageGPU(points,image):
    for i in range(points.shape[0]):
        image[points[i,0],points[i,1]] = 1

def randPoints(n,p):
    points = np.zeros((p,2),dtype=np.int16)
    points[:,0] = np.random.randint(0,n[0],p)
    points[:,1] = np.random.randint(0,n[1],p)
    return points



if __name__=="__main__":
    start = timer()
    # n = (2160,3840)
    n = (200,200)
    p = 1000
    image = np.zeros((n[0],n[1]),dtype=np.uint8)
    print("Init Points:", timer()-start) 
    times = [[],[],[],[]]   

    plt.show(block=False)
    plt.ion()
    loopStart = timer()

    for i in range(10):
        start = timer()
        points = randPoints(n,p)
        times[0].append(timer()-start)
        
        start = timer()
        pointsToImage(points,image)
        times[1].append(timer()-start)
        
        start = timer()
        pointsToImageGPU(points,image)
        times[2].append(timer()-start)
        
        start = timer()
        plt.imshow(image,cmap='gray',interpolation="nearest")
        times[3].append(timer()-start)

        print(i)
        plt.pause(max(0.00001,1-timer()+loopStart))
        loopStart = timer()
        
        
    plt.show()

    print("Generate Points:", np.mean(times[0]))
    print("without GPU:", np.mean(times[1]))  
    print("with GPU:", np.mean(times[2]))
    print("Array to Image:", np.mean(times[3]))

    


