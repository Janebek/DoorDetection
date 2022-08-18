from numpy import *
from math import sqrt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


def rigid_transform_3D(a1, a2, a3, b1, b2, b3):
    print("Now is in python module")
    A = mat([0, 0, 0])
    A1 = mat([a1, a2, a3])
    print(A)
    print(A1) 
    assert len(A) == len(A1)
    N = A.shape[0];
    mu_A = mean(A, axis=0)
    mu_A1 = mean(A1, axis=0)
    print(mu_A)
    print(mu_A1) 

    AA = A - tile(mu_A, (N, 1))
    AA1 = A1 - tile(mu_A1, (N, 1))
    H = transpose(AA) * AA1

    U, S, Vt = linalg.svd(H)
    R = Vt.T * U.T

    if linalg.det(R) < 0:
        print("Reflection detected")
        Vt[2, :] *= -1
        R = Vt.T * U.T

    t = -R * mu_A.T + mu_A1.T

    B = mat([b1, b2, b3])
    B1 = (R*B.T)+ tile(t,(1,1))
    
    print (B1.T)

    #return B1[0], B2[1], B[2]
    return (B1[0], B1[1], B1[2])

def AdditionFc(a, b):
    print("Now is in python module")
    print("{} + {} = {}".format(a, b, a+b))
    return a + b

def printAB(A, B):
    print("Now is in python module")
    print(A)
    print(B)


#if __name__=='__main__':

 #   A = mat([0, 0, 0])
 #   print("points A")
 #   print(A)
  #  print("")
    
   # A1 = mat([21, 34, 45])
#    print("points A1")
 #   print(A1)
  #  print("")

#    ret_R, ret_t = rigid_transform_3D(A,A1)
 #   print("R")
  #  print(ret_R)
   # print("")
#    print("t")
 #   print(ret_t)
  #  print("")
    
   # B = mat([1, 2, 3])
#    print("points B")
 #   print(B)
  #  print("")
    
   # B1 = (ret_R*B.T)+ tile(ret_t,(1,1))
#    print("points B1")
 #   print(B1.T)
  #  print("")
