#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from tkinter import ttk
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tkinter as tk
import rospy
import threading

root = tk.Tk(className="Hector Quadrotor UI")
root.title("Hector Quadrotor UI")

# Icon Image
iconImg = """iVBORw0KGgoAAAANSUhEUgAAADAAAAAwCAYAAABXAvmHAAAABmJLR0QA/wD/AP+gvaeTAAAE4klEQVRoge2YW2wUVRjHf+fMXqHGECVB5Vq8EJW
        LAsG0knBpJZCQlmJIKKWSWEuMMSwPvvggMfpiTGTRNECiwbRbYxq6pfGhGisRuQQKNOXyYDS0QA0gGMQIvezuzOfDttDt7nZnZxtRsv992DPf9f+d
        b86cmQN55JFHHnnk8aCjtLKmo3TT68f/i/lcBw4elUxGdZ/vA1HYsS1fUazG0o93Pp3J4IFAaWWNlFbWZJyN+5FvzHYDBEPhDmBx/EpOBKrWv5QTu3HOZ
        +cWWnxvqJY4p2YbWeVL24EDB4+sAbUXmDpaV7ajGIDW9486IZiEDPF+E5HadStfbkulHKMDqcnfB0xVSu1Np3Q5ifjnE8+Czrh8/pV4ab1afjiyeqjyaU
        6JjRN6lVK1ZcuLvk2ltFV207Fjfne/PC5aHlOWnqaRhShKBOaPB0MFZxDaLdRp0VavstTVqF9d2VBU1G/D1zkOHDy8BPSXwByHIX4Ga0v5iqUnnHLI+UZ
        ubj9caGh9wYmvaVmz15cs7c4lf06vEh990fpQ7+9/VDr1v3ztxsa6pqaCXDg4LiDYuH+V12v+gsgHTmMo+DAacf0abNy/ymkMRwUEG8PbEd0GTAF+ROR6
        tjHE4roIh4ApiG7b1dCyzQmXrAsIhsJvIHwCmCi2BaoqlgtsAXqzCNOrDfXa9s0Vy5SoAGCKkuDOhuaabPlktYg/bQjPtxTHAY8oqrdvqmjMNmEqBEMtV
        SD1wCDaWhKofPWsXd+sOiCKzwAfqJ3jRR4gULUuhBAEfFhqt4jYnljbBexsbC4RWArcjFoTdjghOhbc3th7wE1QRcGvwivt+tl+F1Ki3gRQsPud6lV30t
        mVP//1K0rRAkwYpfpLi5Q0n994KpXfWxs23A42hvcgvDuUq90OL1sd+Lj+u4nAGsCyROrS2a1d+M0ErdiTgjzAw5ZSDVtm7vOl8ze0UQcIsHooZ0bY6oB
        X9xVb4FNwLrB5/dVhedWi1lKt2AvMikss8PnHCjXH8vn7qye3Dl/3gNTWnyxvB3h7Y9mVXaHweYG5bnWnCPg+EzdbHRDkRQBRcijBOYH8PcQsk77IILcH
        Brg9MEBfZJCYaaYKPUtIetf/CUAp9YIdbrY6IDAv/q9PjyYw2nYwGiVixhJkpiX0WxE84sLrcifoFBSOvLaUOqVEEGSBHW72FrFSTyGClrE3q5hlEjFjG
        IbBogXzKJwxHUHouXSZU13niMRiGErjMoy0MbTQO3Qc8eT4FSDyKIAg7cFQ+K64c1eiWSQan/lFC+ZRvOxpAG5dg+fmPIMIdHR2ETFjSQUEQ+G7RyjC8D
        CeMxPs7gOP2DGyJJ68cMZ0UDDyYGf2rBkAmJZlM6WyldPuPuAH8N9xT9y6dW3fsLB6cWvC4ZOMoHzrKgmQoeKUSt5kA1UVd4V1TU0F0YjrbyDt43Yk7Hb
        AAJg0qX9wTCMdD3fh4qUkXXdPXGakKGAkbkyePDA0tDW5OX2Rje5AzDTpj0YwDIOF8+dSOHM6AN0XL3P6zFlM08Lv8eDSiWug/mSZYx6OjlXSBjMMPOIi
        EovR0dlFR2dXgt5juJLI54pcT6d7Rgu8Ljd+tweX1qihn0tr/B4PXrc7KYDA/fsmBqlNRcBlGPg9Xgp8Pgp8Pvweb8qZF+jW6NrcOOSRRx7/a/wDNeu3t
        iRlt6UAAAAASUVORK5CYII="""
img = tk.PhotoImage(data=iconImg)
root.tk.call('wm', 'iconphoto', root._w, img)

mainframe = ttk.Frame(root, padding="3 3 12 12")
mainframe.grid(column=0, row=0, sticky=(tk.N, tk.W, tk.E, tk.S))
mainframe.columnconfigure(0, weight=1)

# Coordinates and Altitude
intermediate_z = 4.0

# Global position variables
current_x = 0.0
current_y = 0.0
current_z = 0.0

# Destination coordinates
destination_x = 0.0
destination_y = 0.0
destination_z = 0.0

# ROS Node Initialization
rospy.init_node('HectorQ_GUI', anonymous=False)

# Publishers
takeoff_pub = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1)
land_pub = rospy.Publisher('/ardrone/land', Empty, queue_size=1)
vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
box_attach_pub = rospy.Publisher('/box_attach', Empty, queue_size=1)  # For simulating box hold
box_release_pub = rospy.Publisher('/box_release', Empty, queue_size=1)  # For simulating box drop

# Get current position callback
def position_callback(data):
    global current_x, current_y, current_z
    current_x = data.pose.pose.position.x
    current_y = data.pose.pose.position.y
    current_z = data.pose.pose.position.z

# Subscribe to position data
rospy.Subscriber("/ground_truth/state", Odometry, position_callback)

def control_drone():
    global current_x, current_y, current_z, destination_x, destination_y, destination_z

    # Store initial takeoff coordinates
    initial_x = current_x
    initial_y = current_y
    initial_z = current_z+1

    # Ascend to intermediate altitude
    rospy.loginfo("Drone taking off to intermediate altitude")
    takeoff_pub.publish(Empty())
    rospy.sleep(5)  # Wait for the drone to start taking off

    while not rospy.is_shutdown():
        if abs(current_z - intermediate_z) < 0.1:
            rospy.loginfo("Reached intermediate altitude")
            break
        vel_msg = Twist()
        vel_msg.linear.z = 0.2
        vel_pub.publish(vel_msg)
        rospy.sleep(0.5)

    # Stop ascending
    hover_pub()
    rospy.sleep(1)

    # Simulate holding the box
    rospy.loginfo("Holding the box at the initial location")
    box_attach_pub.publish(Empty())
    rospy.sleep(1)

    # Move to the specified destination
    rospy.loginfo("Moving to destination")
    while not rospy.is_shutdown():
        if abs(current_x - destination_x) < 0.1 and abs(current_y - destination_y) < 0.1 and abs(current_z - destination_z) < 0.1:
            rospy.loginfo("Reached destination")
            break
        move_x = destination_x - current_x
        move_y = destination_y - current_y
        move_z = destination_z - current_z
        vel_msg = Twist()
        vel_msg.linear.x = move_x * 0.5
        vel_msg.linear.y = move_y * 0.5
        vel_msg.linear.z = move_z * 0.5
        vel_pub.publish(vel_msg)
        rospy.sleep(0.5)

    # Wait for 10 seconds at the destination
    rospy.loginfo("Waiting at the destination for 10 seconds")
    hover_pub()
    rospy.sleep(10)

    # Simulate dropping the box
    rospy.loginfo("Dropping the box at the destination")
    box_release_pub.publish(Empty())
    rospy.sleep(1)

    # Return to the initial location
    rospy.loginfo("Returning to the initial location")
    while not rospy.is_shutdown():
        if abs(current_x - initial_x) < 0.1 and abs(current_y - initial_y) < 0.1 and abs(current_z - initial_z) < 0.1:
            rospy.loginfo("Returned to the initial location")
            break
        move_x = initial_x - current_x
        move_y = initial_y - current_y
        move_z = initial_z - current_z
        vel_msg = Twist()
        vel_msg.linear.x = move_x * 0.5
        vel_msg.linear.y = move_y * 0.5
        vel_msg.linear.z = move_z * 0.5
        vel_pub.publish(vel_msg)
        rospy.sleep(0.5)

    # Land
    land_pub.publish(Empty())
    rospy.loginfo("Mission complete")

def hover_pub():
    vel_msg = Twist()
    vel_pub.publish(vel_msg)

def submit_coordinates():
    global destination_x, destination_y, destination_z
    try:
        destination_x = float(destination_x_entry.get())
        destination_y = float(destination_y_entry.get())
        destination_z = float(destination_z_entry.get())
        start_button['state'] = tk.NORMAL
        rospy.loginfo(f"Destination set to X: {destination_x}, Y: {destination_y}, Z: {destination_z}")
    except ValueError:
        rospy.logwarn("Invalid input for coordinates. Please enter valid numbers.")
        start_button['state'] = tk.DISABLED

def start_fun():
    threading.Thread(target=control_drone).start()

def stop_fun():
    hover_pub()

# Control Buttons
start_button = ttk.Button(mainframe, text="Start", command=start_fun, state=tk.DISABLED)
start_button.grid(column=1, row=1, sticky=tk.W)
ttk.Button(mainframe, text="Stop", command=stop_fun).grid(column=2, row=1, sticky=tk.W)

# Labels and Entry fields for destination coordinates
ttk.Label(mainframe, text="Destination X:").grid(column=1, row=2, sticky=tk.W)
destination_x_entry = ttk.Entry(mainframe)
destination_x_entry.grid(column=2, row=2, sticky=(tk.W, tk.E))

ttk.Label(mainframe, text="Destination Y:").grid(column=1, row=3, sticky=tk.W)
destination_y_entry = ttk.Entry(mainframe)
destination_y_entry.grid(column=2, row=3, sticky=(tk.W, tk.E))

ttk.Label(mainframe, text="Destination Z:").grid(column=1, row=4, sticky=tk.W)
destination_z_entry = ttk.Entry(mainframe)
destination_z_entry.grid(column=2, row=4, sticky=(tk.W, tk.E))

ttk.Button(mainframe, text="Submit", command=submit_coordinates).grid(column=1, row=5, columnspan=2)

ttk.Label(mainframe, text="Control the drone").grid(column=1, row=0, columnspan=2, sticky=tk.W)

for child in mainframe.winfo_children():
    child.grid_configure(padx=5, pady=5)

root.mainloop()

