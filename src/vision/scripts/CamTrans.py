import numpy as np
from math import *

class CameraTrans:
    def __init__(self, size_xy, halfTan_xy):
        self.imgSize = np.array(size_xy, np.float64)
        self.imgHalfSize = self.imgSize / 2
        self.imgHalfTan = np.array(halfTan_xy, np.float64)
        
    def SetLocation(self, x, y, z, yaw, pitch):
        
        self.location = np.matrix([[x],
                                                            [y],
                                                            [z]])
        #yaw, pitch == 0, 0时,相机正前为y轴,右为x轴,上位为Z轴
        #yaw, pitch转轴分别为Z轴,X轴
        
        matrix_pitch   = np.matrix([[1,           0,           0          ],
                                                                [0,           cos(pitch),  -sin(pitch)],
                                                                [0,           sin(pitch),  cos(pitch) ]], np.float64)
        matrix_yaw     = np.matrix([[cos(yaw),    -sin(yaw),   0          ],
                                                                [sin(yaw),    cos(yaw),    0          ],
                                                                [0,           0,           1          ]], np.float64)   
                                                
        self.matrix_camera2global = matrix_yaw * matrix_pitch
        self.matrix_global2camera = self.matrix_camera2global.I
        
    def GetLength(self, vector):
        return np.sqrt(np.sum(vector * vector))
        
    def img2global(self, x_img, y_img, target_z = 0):
        vx_camera = (x_img - self.imgHalfSize[0]) / self.imgHalfSize[0] * self.imgHalfTan[0]
        vy_camera = 1
        vz_camera = (self.imgHalfSize[1] - y_img) / self.imgHalfSize[1] * self.imgHalfTan[1]
        
        if vz_camera == 0:
            return None
            
        v_xyz_camera = np.matrix([[vx_camera], [vy_camera], [vz_camera]], np.float64)
        v_xyz_global = self.matrix_camera2global * v_xyz_camera
        
        distance = (target_z - self.location[2, 0]) / vz_camera
        
        if distance < 0:  #不在视野范围内
            return None
        
        target_xyz_global = self.location + v_xyz_global * distance
        return np.array([target_xyz_global[0,0], target_xyz_global[1,0], target_xyz_global[2,0]], np.float64)
        
    def global2img(self, x_global, y_global, z_global):
        target_xyz_global = np.matrix([[x_global], [y_global], [z_global]], np.float64)
        target_xyz_camera = self.matrix_global2camera * (target_xyz_global - self.location)
        
        if target_xyz_camera[1] <= 0:  #不在视野范围内
            return None
            
        x_img_tan =  target_xyz_camera[0] / target_xyz_camera[1]
        y_img_tan = -target_xyz_camera[2] / target_xyz_camera[1]
        
        if abs(x_img_tan) > self.imgHalfTan[0] or abs(y_img_tan) > self.imgHalfTan[1]:  #不在视野范围内
            return None
        
        x_img = x_img_tan / self.imgHalfTan[0] * self.imgHalfSize[0] + self.imgHalfSize[0]
        y_img = y_img_tan / self.imgHalfTan[1] * self.imgHalfSize[1] + self.imgHalfSize[1]
        return (x_img, y_img)
