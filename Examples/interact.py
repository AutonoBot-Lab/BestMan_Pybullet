# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : load_panda.py
# @Time           : 2024-08-03 15:04:37
# @Author         : yk
# @Email          : yangkui1127@gmail.com
# @Description:   : A example load panda robot
"""


import os
import sys
import termios
import tty

from Config import load_config
from Env.Client import Client
from Robotics_API.Bestman_sim_panda import Bestman_sim_panda
from Visualization.Visualizer import Visualizer

def get_key():
        """
        Get a single character from standard input. Does not require Enter to be pressed.
        """
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
            os.system('stty -echo')  # Disable echo
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            os.system('stty echo')  # Enable echo back
        return ch

def control_base_with_keyboard(robot):
    """
    Control the base using keyboard inputs: w, a, s, d, r, l.
    """
    print("Interact start")
    while True:
        key = get_key()
        if key == 'w':
            robot.sim_move_base_forward(0.05)
        elif key == 's':
            robot.sim_move_base_backward(0.05)
        elif key == 'a':
            robot.sim_rotate_base(5, 'counter-clockwise')
        elif key == 'd':
            robot.sim_rotate_base(5, 'clockwise')
        elif key == 'q':
            print("Interact end!")
            break

def main(filename):

    # load config
    config_path = "Config/interact.yaml"
    cfg = load_config(config_path)
    print(cfg)

    # Init client and visualizer
    client = Client(cfg.Client)
    visualizer = Visualizer(client, cfg.Visualizer)

    # Init robot
    panda = Bestman_sim_panda(client, visualizer, cfg)

    # Interact contorl
    control_base_with_keyboard(panda)
    
    # disconnect pybullet
    client.wait(5)
    client.disconnect()


if __name__ == "__main__":

    # set work dir to Examples
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    # get current file name
    file_name = os.path.splitext(os.path.basename(__file__))[0]

    main(file_name)