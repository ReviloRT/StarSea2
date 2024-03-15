from utils import *
from imageGen import Imager
import orbitComputes as OC


if __name__ == "__main__":
    im = Imager((200,200))

    path = OC.calc_orbit_path([0.1,0,0],[0,10,0],0.01)
    print(path)
    im.pointsToImage(path[0:2,:])
    im.displayImage()

    