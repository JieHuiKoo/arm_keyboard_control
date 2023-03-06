#! /home/jiehui/anaconda3/envs/tensorflow/bin/python

import rospy

from std_msgs.msg import Float64

from pynput import keyboard

import time, math

timedelay = 1

class joint:
    def __init__(self, name, min, max):
        self.current = float(0)
        self.min = float(min)
        self.max = float(max)
        self.name = str(name)

    def update_angle(self, delta_angle):
        
        self.current = self.current + delta_angle

        if self.current > self.max:
            self.current = self.max
        if self.current < self.min:
            self.current = self.min
        # print("Joint name: " + self.name + " | Angle: " + str(self.current))

class arm_control:
    joint1 = joint('joint1', -2.09, 2.09)
    joint2 = joint('joint2', -1.57, 1.57)
    joint3 = joint('joint3', -2.09, 2.09)
    joint4 = joint('joint4', -2.09, 2.09)
    joint5 = joint('joint5', -2.09, 2.09)
    
    delta_angle_bindings = {
        'x' : 0.0174533, # 1 deg
        'c' : 0.0872665, # 5 deg
        'v' : 0.174533,  # 10 deg
        'b' : 0.785398   # 45 deg
    }

    currently_changing_joint = 1
    delta_angle = delta_angle_bindings['c']

    positionBindings = {
        'h' : [ 0, 0, 0, 0, 0],
        'm' : [ 1, 0, 0, 0, 0],
        'p' : [ 0, 1, 0, 0, 0],
        '[' : [ 0, 0, 1, 0, 0],
        ']' : [ 0, 0, 0, 1, 0],
        '\\': [ 0, 0, 0, 0, 1]
    }

    def rad_to_deg(self, rad):
        deg = rad * 180/math.pi
        return deg
    
    def deg_to_rad(self, deg):
        rad = deg * math.pi/180
        return rad

    def set_delta_angle(self, key):
        self.delta_angle = self.delta_angle_bindings[key]

    def set_currently_changing_joint(self, joint_num):
        self.currently_changing_joint = joint_num

    def increment_arm_joint(self):
        joint_num = self.currently_changing_joint
        delta_angle = self.delta_angle

        if joint_num == 1:
            self.joint1.update_angle(delta_angle)
        elif joint_num == 2:
            self.joint2.update_angle(delta_angle)
        elif joint_num == 3:
            self.joint3.update_angle(delta_angle)
        elif joint_num == 4:
            self.joint4.update_angle(delta_angle)
        elif joint_num == 5:
            self.joint5.update_angle(delta_angle)
            
    def decrement_arm_joint(self):
        joint_num = self.currently_changing_joint
        delta_angle = self.delta_angle * -1

        if joint_num == 1:
            self.joint1.update_angle(delta_angle)
        elif joint_num == 2:
            self.joint2.update_angle(delta_angle)
        elif joint_num == 3:
            self.joint3.update_angle(delta_angle)
        elif joint_num == 4:
            self.joint4.update_angle(delta_angle)
        elif joint_num == 5:
            self.joint5.update_angle(delta_angle)
        
    def set_position(self, key):
        array = self.positionBindings[key]

        self.joint1.current = array[0]
        self.joint2.current = array[1]
        self.joint3.current = array[2]
        self.joint4.current = array[3]
        self.joint5.current = array[4]

        print("\nSetting Position Binding: " + key)
        
    def print_joints(self):
        print('Positions Set: '\
            + str(round(self.rad_to_deg(self.joint1.current),3)) + ' | ' \
            + str(round(self.rad_to_deg(self.joint2.current),3)) + ' | ' \
            + str(round(self.rad_to_deg(self.joint3.current),3)) + ' | ' \
            + str(round(self.rad_to_deg(self.joint4.current),3)) + ' | ' \
            + str(round(self.rad_to_deg(self.joint5.current),3)))
        
    msg = '===\nSet Arm Joints by pressing 1, 2, 3, 4, 5\n\n\
    W: Increment\n\
    S: Decrement\n\
    X: 1  Deg Small Delta Angle\n\
    C: 5  Deg Delta Angle\n\
    V: 10 Deg Delta Angle\n\
    B: 45 Deg Delta Angle\n\
    \
    \n\nAvailable Position Bindings:\n\
    H: Home\n\
    M: ML Detection\n\
    P: Photo Position 1\n\
    [: Photo Position 2\n\
    ]: Photo Position 3\n\
    \\: Photo Position 4\n==='

hiwonder = arm_control()
hiwonder_joints = ['1','2','3','4','5']

def on_press(key):
    try:
        # print('alphanumeric key {0} pressed'.format(
            # key.char))
        pass
        
    except AttributeError:
        print('special key {0} pressed'.format(
            key))

def on_release(key):
    print('{0} released'.format(
        key))
    
    print(hiwonder.msg)

    try:
        # If the key is one of the preset positionBindings
        if key.char in hiwonder.positionBindings.keys():
            hiwonder.set_position(key.char)
        
        if key.char in hiwonder_joints:
            hiwonder.set_currently_changing_joint(int(key.char))
        
        if key.char == 'w':
            hiwonder.increment_arm_joint()
        
        if key.char == 's':
            hiwonder.decrement_arm_joint()

        if key.char in hiwonder.delta_angle_bindings.keys():
            hiwonder.set_delta_angle(key.char)
    
        print("Currently changing joint: " + str(hiwonder.currently_changing_joint))
        print("Delta Angle Set: " + str(hiwonder.rad_to_deg(hiwonder.delta_angle)))
        hiwonder.print_joints()

        # ADD SEND SERVICE REQUEST HERE

    
    except AttributeError:
        if key == keyboard.Key.esc:
            # Stop listener
            return False

def start_node():
    rospy.init_node('arm_keyboard_control')
    rospy.loginfo('[arm_keyboard_control]: node started')

    # Collect events until released
    print("Currently changing joint: " + str(hiwonder.currently_changing_joint))
    print("Delta Angle Set: " + str(hiwonder.rad_to_deg(hiwonder.delta_angle)))
    hiwonder.print_joints()

    with keyboard.Listener(
        on_press=on_press,
        on_release=on_release) as listener:
        listener.join()
    

if __name__ == '__main__':
    
    try:
        print(hiwonder.msg)
        start_node()

    except rospy.ROSInterruptException:
        pass