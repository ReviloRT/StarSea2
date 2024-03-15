from utils import *
from matplotlib import pyplot as plt
import numpy as np
import logging


# From polar as (r,theta)
def calcEffectivePotential(mu,rad,vel):
    1/2*(vel[0])*(vel[0]) - mu/rad

# Solvers Keplers Equation using the Newton_raphson's method
    # M = mean anomaly
    # e = eccentricity
    # tol = maximum solution tollerance
def kepler_NR(M,e,tol):

    # Solves for E in:  M = E - e*sin(E)
    # via equation: M - E + e*sin(E) = 0
    E = 0.1
    error = 10000
    while abs(error) > tol:
        error,dot_error = kepler_implicit(M,e,E)
        E += error/dot_error
        print(E,error)
    theta = np.arctan(np.sqrt((1+e)/(1-e))*np.tan(E/2))*2

def kepler_implicit(M,e,E):
    firstOrder = M - E - e*np.sin(E)
    secondOrder = 1 - e*np.cos(E)
    return (firstOrder,secondOrder)

def calc_circular_path(rad,samples=1000):
    path = np.zeros((samples,2))
    thetas = np.linspace(0,2*np.pi,samples)
    path[0,:] = rad*np.sin(thetas)
    path[1,:] = rad*np.cos(thetas)
    return path

def calc_eliptical_path(b,e,w,samples=1000):
    path = np.zeros((samples,2))
    thetas = np.linspace(0,2*np.pi,samples)
    a = b/np.sqrt(1-e*e)
    path[0,:] = a * np.cos(thetas)*np.cos(w) + b * np.sin(thetas)*np.sin(w)
    path[1,:] = a * np.cos(thetas)*np.sin(w) + b * np.sin(thetas)*np.cos(w)
    return path

def calc_hyperbolic_path(b,e,w,samples=1000):
    path = np.zeros((samples,2))
    thetas = np.linspace(0,2*np.pi,samples)
    a = b/np.sqrt(e*e-1)
    path[0,:] = a / np.cos(thetas)*np.cos(w) + b * np.tan(thetas)*np.sin(w)
    path[1,:] = a / np.cos(thetas)*np.sin(w) + b * np.tan(thetas)*np.cos(w)
    return path

def calc_eccentric_path(a,e,w,samples=1000):
    if e < 0:
        logging.error("Eccentricity less than 0 = ",e)
        return 0
    if e == 0: return calc_circular_path(a,samples)
    if e < 1: return calc_eliptical_path(a,e,w,samples)
    else: return calc_hyperbolic_path(a,e,w,samples)
    
def calc_orbit_path(pos_vec, vel_vec,mu,samples=1000):
    polar_path = np.zeros((3,samples))
    polar_path[1,:] = np.linspace(0,2*np.pi,samples)

    angular_momentum = np.cross(pos_vec, vel_vec)
    eccentricity = np.cross(vel_vec, angular_momentum) / mu - pos_vec/np.sqrt(np.dot(pos_vec,pos_vec))
    e_scalar = np.dot(eccentricity,eccentricity)
    polar_path[0,:] = np.dot(angular_momentum,angular_momentum) / mu / (1 + e_scalar*np.cos(polar_path[1,:]))

    return polar_to_cartesian(normalise(angular_momentum),normalise(pos_vec),polar_path)

def polar_to_cartesian2D(polar):
    cartesian = np.zeros_like(polar)
    cartesian[0,:] = polar[0,:] * np.cos(polar[1,:])
    cartesian[1,:] = polar[0,:] * np.sin(polar[1,:])
    return cartesian

def polar_to_cartesian(perp,inital,polar):
    points = polar.shape[0]
    cart_2d = polar_to_cartesian2D(polar)
    cart_3d = np.zeros((3,points))
    cos_polar = np.cos(polar[1,:])
    cos_polar = np.row_stack([cos_polar,cos_polar,cos_polar])
    perp_tile = np.tile(perp.T,points)
    cart_3d = cart_2d*cos_polar + np.cross(perp_tile,cart_2d,axis=0)*np.sin(polar[1,:]) + perp*(np.dot(perp,cart_2d))*(1-cos_polar)
    return cart_3d

def spherical_to_cartesian(sphereical):
    cartesian = np.zeros_like(sphereical)
    cartesian[0,:] = sphereical[0,:] * np.cos(sphereical[1,:]) * np.sin(sphereical[2,:])
    cartesian[1,:] = sphereical[0,:] * np.sin(sphereical[1,:]) * np.sin(sphereical[2,:])
    cartesian[2,:] = sphereical[0,:] * np.cos(sphereical[2,:])
    return cartesian

def project_cartesian3D_to_cartesian2D(perp,points):
    projected_points = points - np.dot(points)*np.tile(perp,points.shape[1])
    return projected_points


if __name__ == "__main__":
    scaleFactor = 10**12
    mu_sun = 1.3271244*(10**12)
    mu_earth = 3.986*(10**5)
    r_earth = 6378.14
    j2_earth = 1082.63*(10**(-6))

    # print("Theta:",kepler_NR(np.pi/2,0.9,0.0001))

    
   

