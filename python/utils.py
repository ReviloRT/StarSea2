from timeit import default_timer as timer   
import numpy as np

def timeis(func): 
    '''Decorator that reports the execution time.'''
  
    def wrap(*args, **kwargs): 
        start = timer()
        result = func(*args, **kwargs) 
        end = timer()
          
        print(func.__name__, end-start) 
        return result 
    return wrap

def normalise(vector):
    return np.sqrt(np.dot(vector,vector))