import numpy 
import math
import rospy
class Axis_Info:
    def __init__(self):
        self.difference = 0
        self.tempDiffer = 0 #上一个时刻
        self.differential = 0
        self.integral = 0
        

class PID:
    def __init__(self):
        self.y_target = int(rospy.get_param("/y_target"))  # 0-480  480
        self.x_target = int(rospy.get_param("/x_target"))  # 0-640  320
        self.vel_P = float(rospy.get_param("/vel_P"))
        self.vel_I = float(rospy.get_param("/vel_I"))
        self.vel_D = float(rospy.get_param("/vel_D"))
        self.w_P = float(rospy.get_param("/w_P"))
        self.max_vel = float(rospy.get_param("/max_vel"))
        self.Axis_Info_3 = [Axis_Info(),Axis_Info(),Axis_Info()]
        self.cmd_vel_linear = [0,0,0]
        self.cmd_vel_angular = [0,0,0]
        self.expectPos = [self.x_target,self.y_target,0]
    
    def VelPIDController(self,leafPos):
        for i in range(3):
            self.Axis_Info_3[i].difference = self.expectPos[i]-leafPos[i]
            self.Axis_Info_3[i].differential = self.Axis_Info_3[i].difference - self.Axis_Info_3[i].tempDiffer
            self.Axis_Info_3[i].integral += self.Axis_Info_3[i].difference
            self.Axis_Info_3[i].tempDiffer = self.Axis_Info_3[i].difference
            self.cmd_vel_linear[i] = self.vel_P*self.Axis_Info_3[i].difference + self.vel_D*self.Axis_Info_3[i].differential + self.vel_I*self.Axis_Info_3[i].integral
            if math.fabs(self.cmd_vel_linear[i])>=self.max_vel:
                if self.cmd_vel_linear[i] >0:
                    self.cmd_vel_linear[i]  = self.max_vel
                else:
                    self.cmd_vel_linear[i]  = -self.max_vel
        return [self.cmd_vel_linear[0],self.cmd_vel_linear[1],0]