import cv2
import numpy as np
import sys
import os
from typing import Tuple

message = """
 ____    ___   ____    ____  
|  _ \  / _ \ / ___|  |___ \ 
| |_) || | | |\___ \    __) |
|  _ < | |_| | ___) |  / __/ 
|_| \_\ \___/ |____/  |_____|
                             
    _            _                 
   / \    _   _ | |_   ___         
  / _ \  | | | || __| / _ \  _____ 
 / ___ \ | |_| || |_ | (_) ||_____|
/_/   \_\ \__,_| \__| \___/        
                                   
__     __       _      _        _       
\ \   / /  ___ | |__  (_)  ___ | |  ___ 
 \ \ / /  / _ \| '_ \ | | / __|| | / _ \
  \ V /  |  __/| | | || || (__ | ||  __/
   \_/    \___||_| |_||_| \___||_| \___|  

"""
print(message)

print("ROS2 기반 자율주행 설계 및 구현")
print("Sungkyunkwan University Automation Lab.")

print("------------------Authors------------------")
print("Hyeong-Keun Hong <whaihong@g.skku.edu>")
print("Jinsun Lee <with23skku@g.skku.edu>")
print("Siwoo Lee <edenlee@g.skku.edu>")
print("Jae-Wook Jeon <jwjeon@skku.edu>")
print("------------------------------------------")




def calculate_slope_between_points(p1, p2):
    
    p1_x = p1[0]
    p1_y = p1[1]
    p2_x = p2[0]
    p2_y = p2[1]
    
    if p1_y == p2_y:
        slope = 'inf'
    else:
        slope = np.arctan((p2_x-p1_x)/(p1_y-p2_y))*180/np.pi
    
    return slope
