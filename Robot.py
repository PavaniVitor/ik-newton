import numpy as np
 

class joint():
    def __init__(self, position, lenght, theta):
        self.position = position
        self.rotation = theta
        self.lenght = lenght

    def set_pos(self, position):
        self.position = position

    def get_end_pos(self):
        x = self.position[0] + self.lenght * np.sin(self.rotation)
        y = self.position[1] + self.lenght * np.cos(self.rotation)
        return (x, y)

class Robot():
    def __init__(self, position , arm_lenght, arm_orientation, wrist_lenght, wrist_orientation):
        self.arm = joint(position, arm_lenght, arm_orientation)
        self.wrist = joint(self.arm.get_end_pos(), wrist_lenght, wrist_orientation)
        
    def fx(self, theta, phi):
        # compute x final positioon, arm angle = theta, wrist angle = phi
        return (self.arm.lenght * np.cos(theta)) + (self.wrist.lenght * np.cos(theta + phi))  
        

    def fy(self, theta, phi):
        # compute y final position,  arm angle = theta, wrist angle = phi
        return (self.arm.lenght * np.sin(theta)) + (self.wrist.lenght * np.sin(theta + phi))
        
    def rotate(self, theta, phi):
        self.arm.rotation = theta
        self.wrist = joint(self.arm.get_end_pos(), self.wrist.lenght, phi + theta)
        


    def Jacobian(self):

        theta = self.arm.rotation
        phi = self.wrist.rotation

        J = np.ones((2,2))
        
        # row 1
        J[0,0] = -1 * (self.arm.lenght * np.sin(theta)) - self.wrist.lenght * np.sin(theta + phi) 
        J[0,1] = -1 * self.wrist.lenght * (np.sin(theta + phi))
        # row 2
        J[1,0] =  self.arm.lenght * np.cos(theta) + self.wrist.lenght * np.cos(theta + phi)
        J[1,1] = self.wrist.lenght * np.cos(theta + phi) 

        return J
