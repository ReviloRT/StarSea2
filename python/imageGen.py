from numba import jit, cuda
from matplotlib import pyplot as plt
import numpy as np

class Imager():
    def __init__(self,n_size=(600,600), n_center=(0,0), n_scale=1):
        self.imageSize = n_size
        self.center = n_center
        self.scale = n_scale
        self.imageArray = np.zeros((self.imageSize[0],self.imageSize[1]),dtype=np.uint8)
        plt.show(block=False)
        plt.ion()



    def check_valid_point_array(self,points):
        n = points.shape
        if len(n) != 2:
            print("Error in points to Image, not enough dims:", n)
            quit()
        if n[0] != 2:
            print("Error in points to Image, second dim not = 2:", n)
            quit()

        return
    

    def camera_transform_point(self,x,y,dx,dy,dmul):
        t_x = (x + dx)*dmul
        t_y = (y + dy)*dmul
        return (t_x,t_y)


    def point_to_pix_2D(self,x,y):
        p_x = min(max(0,x*self.imageSize[0]/2+self.imageSize[0]/2),self.imageSize[0]-1)
        p_y = min(max(0,y*self.imageSize[1]/2+self.imageSize[1]/2),self.imageSize[1]-1)
        return (int(p_x),int(p_y))


    # normal function to run on cpu
    def pointsToImage(self,points):   
        self.check_valid_point_array(points)
        for i in range(points.shape[0]):
            t_x,t_y = self.camera_transform_point(points[0,i],points[1,i],self.center[0],self.center[1],self.scale)
            p_x, p_y = self.point_to_pix_2D(t_x,t_y)
            self.imageArray[p_x,p_y] = 1


    # function optimized to run on gpu 
    @jit(target_backend='cuda')                         
    def pointsToImageGPU(self,points):
        self.check_valid_point_array(points)
        for i in range(points.shape[0]):
            t_x,t_y = self.camera_transform_point(points[0,i],points[1,i],self.center[0],self.center[1],self.scale)
            p_x, p_y = self.point_to_pix_2D(t_x,t_y)
            self.imageArray[p_x,p_y] = 1

    # convert the image array to an image and display
    def displayImage(self,dt = 0):
        plt.imshow(self.imageArray,cmap='gray',interpolation="nearest")
        if (dt == 0):
            dt = 100000
        plt.pause(dt)


if __name__ == "__main__":
    im = Imager()
    