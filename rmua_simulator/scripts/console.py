#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import pygame
from pygame.locals import *
import time
import sys
import rospy

from sensor_msgs.msg import Joy
def main():
    # initialize pygame to get keyboard event
    pygame.init()
    window_size = Rect(0, 0, 300, 236)
    screen = pygame.display.set_mode(window_size.size)
    path = os.path.dirname(os.path.abspath(__file__))
    # print(path)
    img = pygame.image.load(path+"/terminal.jpg")
    
    joy_pub = rospy.Publisher('/joy', Joy, queue_size=10)
    rospy.init_node('console')
    rate = rospy.Rate(10)

    joy_ = Joy()    
    joy_.header.frame_id = 'map'
    for i in range(8):
      joy_.axes.append(0.0)
    for i in range(11):
      joy_.buttons.append(0)
    
    screen.blit(img, (1,1))
    pygame.display.flip()
    state_w = 0
    state_a = 0
    state_s = 0
    state_d = 0
    press_w_start = rospy.Time.now().to_sec()
    press_a_start = rospy.Time.now().to_sec()
    press_s_start = rospy.Time.now().to_sec()
    press_d_start = rospy.Time.now().to_sec()
    time_coeff = 10.0
    while not rospy.is_shutdown():
        rate.sleep()
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_1:
                    joy_.buttons[0] = 1
                if event.key == pygame.K_2:
                    joy_.buttons[0] = 2
                if event.key == pygame.K_3:
                    joy_.buttons[0] = 3
                if event.key == pygame.K_4:
                    joy_.buttons[0] = 4
                if event.key == pygame.K_5:
                    joy_.buttons[0] = 5                   
                if event.key == pygame.K_w:
                    if state_w == 0:
                        state_w = 1
                        press_w_start = rospy.Time.now().to_sec()
                if event.key == pygame.K_a:
                    if state_a == 0:
                        state_a = 1
                        press_a_start = rospy.Time.now().to_sec()
                if event.key == pygame.K_s:
                    if state_s == 0:
                        state_s = 1
                        press_s_start = rospy.Time.now().to_sec()
                if event.key == pygame.K_d:
                    if state_d == 0:
                        state_d = 1
                        press_d_start = rospy.Time.now().to_sec()
            elif event.type == pygame.KEYUP:
                if event.key == pygame.K_w:
                    state_w = 0
                if event.key == pygame.K_a:
                    state_a = 0
                if event.key == pygame.K_s:
                    state_s = 0
                if event.key == pygame.K_d:
                    state_d = 0
        current_time = rospy.Time.now().to_sec()
        # 前进后退
        if state_w == 1:
            value = (current_time - press_w_start)*time_coeff
            joy_.axes[1] = min(1.0, value)
        if state_s == 1:
            value = (current_time - press_s_start)*time_coeff
            joy_.axes[1] = -min(1.0, value)
        if state_w == 0 and state_s == 0:
            joy_.axes[1] = 0
        #左右
        if state_a == 1:
            value = (current_time - press_a_start)*time_coeff
            joy_.axes[2] = min(1.0, value)
        if state_d == 1:
            value = (current_time - press_d_start)*time_coeff
            joy_.axes[2] = -min(1.0, value)
        if state_a == 0 and state_d == 0:
            joy_.axes[2] = 0
        joy_pub.publish(joy_)
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
