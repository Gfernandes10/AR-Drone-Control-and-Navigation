#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from keyboard.msg import Key

def keyboard_callback(data):
    global is_turned_on
    if data.code == 112:  # Código da tecla 'P'
        is_turned_on = not is_turned_on
        if is_turned_on:
            rospy.loginfo("Estou ligado")
        else:
            rospy.loginfo("Estou desligado")

if __name__ == '__main__':
    rospy.init_node('keyboard_toggle_node')
    is_turned_on = False

    rospy.Subscriber('/keyboard/keydown', Key, keyboard_callback)

    rospy.loginfo("Node 'keyboard_toggle_node' em execução.")
    rospy.spin()
