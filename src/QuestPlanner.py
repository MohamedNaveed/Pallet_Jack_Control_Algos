#!/usr/bin/env python
from __future__ import division
import rospy
import pallet_jack_params #local package
from pallet_jack_state.msg import state_msg #local package
from geometry_msgs.msg import Twist

from Atsushi_reed_shepp import reeds_shepp_path_planning as RSP_func
import matplotlib.pyplot as plt
import numpy as np
import math as m
import random
import copy
import time
import datetime
import os
import sys
import casadi

from drawing_pallet_jack_rev import dpj

"""
GLOBAL DECLARATIONS TO ACT AS KNOBS MAJORLY
"""
#current state
rospy.init_node('robot_control_pallet_jack', anonymous=True)
state = pallet_jack_params.pallet_jack_state()
state_ini = pallet_jack_params.pallet_jack_state()

#initial state
from Tkinter import *

window = Tk()
window.title("AUTONOMOUS PALLET JACK")
window.geometry('550x250')

INITIAL_STATE_SET = False
DRAW_PALLET = True
ALIGN_TO_GOAL_LINE = False
ROTATE_AT_POINT = False
PURE_PURSUIT = False
PID = False
MPC = False
MOCAP_AVAILABLE = True
LIFT = False
REPLAN = False
DROP = False

################################GUI STARTS HERE#################################
"""
# **********************************MOCAP****************************************
"""
def clicked_mocap():
    global  MOCAP_AVAILABLE
    if btn_mocap['bg'] == 'green':
        col1 = 'grey'
        col2 = 'black'
        lbl_mocap['text'] = 'FALSE'
        MOCAP_AVAILABLE = False
    else:
        col1 = 'green'
        col2 = 'white'
        lbl_mocap['text'] = 'TRUE'
        MOCAP_AVAILABLE = True
    btn_mocap['bg'] = col1
    btn_mocap['fg'] = col2

btn_mocap = Button(window, text='MOCAP AVAILABLE',bg='green',fg = 'white',command=clicked_mocap,width=50)
btn_mocap.grid(column=0, row=0)
lbl_mocap = Label(window, text='TRUE')
lbl_mocap.grid(column=1,row=0)

"""
# **********************************INITIAL_STATE_SET****************************************
"""
def clicked_ini_state():
    global  INITIAL_STATE_SET
    if btn_ini_state['bg'] == 'green':
        col1 = 'grey'
        col2 = 'black'
        lbl_ini_state['text'] = 'FALSE'
        INITIAL_STATE_SET = False
    else:
        col1 = 'green'
        col2 = 'white'
        lbl_ini_state['text'] = 'TRUE'
        INITIAL_STATE_SET = True
    btn_ini_state['bg'] = col1
    btn_ini_state['fg'] = col2

btn_ini_state = Button(window, text='INITIAL_STATE_SET',bg='grey',fg = 'black',command=clicked_ini_state,width=50)
btn_ini_state.grid(column=0, row=3)
lbl_ini_state = Label(window, text='FALSE')
lbl_ini_state.grid(column=1,row=3)

"""
# **********************************PID*******************************************
"""
def clicked_pid():
    global  PID
    if btn_pid['bg'] == 'green':
        col1 = 'grey'
        col2 = 'black'
        lbl_pid['text'] = 'FALSE'
        PID = False
    else:
        col1 = 'green'
        col2 = 'white'
        lbl_pid['text'] = 'TRUE'
        PID = True
    btn_pid['bg'] = col1
    btn_pid['fg'] = col2

btn_pid = Button(window, text='PID',bg='grey',fg = 'black',command=clicked_pid,width=50)
btn_pid.grid(column=0, row=4)
lbl_pid = Label(window, text='FALSE')
lbl_pid.grid(column=1,row=4)

"""
# **********************************PURE_PURSUIT*******************************************
"""
def clicked_pure():
    global  PURE_PURSUIT
    if btn_pure['bg'] == 'green':
        col1 = 'grey'
        col2 = 'black'
        lbl_pure['text'] = 'FALSE'
        PURE_PURSUIT = False
    else:
        col1 = 'green'
        col2 = 'white'
        lbl_pure['text'] = 'TRUE'
        PURE_PURSUIT = True
    btn_pure['bg'] = col1
    btn_pure['fg'] = col2

btn_pure = Button(window, text='PURE PURSUIT',bg='grey',fg = 'black',command=clicked_pure,width=50)
btn_pure.grid(column=0, row=5)
lbl_pure = Label(window, text='FALSE')
lbl_pure.grid(column=1,row=5)

"""
# **********************************LIFT*******************************************
"""
def clicked_lift():
    global  LIFT
    if btn_lift['bg'] == 'green':
        col1 = 'grey'
        col2 = 'black'
        lbl_lift['text'] = 'FALSE'
        LIFT = False
    else:
        col1 = 'green'
        col2 = 'white'
        lbl_lift['text'] = 'TRUE'
        LIFT = True
    btn_lift['bg'] = col1
    btn_lift['fg'] = col2

btn_lift = Button(window, text='LIFT',bg='grey',fg = 'black',command=clicked_lift,width=50)
btn_lift.grid(column=0, row=6)
lbl_lift = Label(window, text='FALSE')
lbl_lift.grid(column=1,row=6)

"""
# **********************************DROP*******************************************
"""
def clicked_drop():
    global  DROP
    if btn_drop['bg'] == 'green':
        col1 = 'grey'
        col2 = 'black'
        lbl_drop['text'] = 'FALSE'
        DROP = False
    else:
        col1 = 'green'
        col2 = 'white'
        lbl_drop['text'] = 'TRUE'
        DROP = True
    btn_drop['bg'] = col1
    btn_drop['fg'] = col2

btn_drop = Button(window, text='DROP',bg='grey',fg = 'black',command=clicked_drop,width=50)
btn_drop.grid(column=0, row=7)
lbl_drop = Label(window, text='FALSE')
lbl_drop.grid(column=1,row=7)

"""
# **********************************REPLAN*******************************************
"""
def clicked_replan():
    global  REPLAN
    if btn_replan['bg'] == 'green':
        col1 = 'grey'
        col2 = 'black'
        lbl_replan['text'] = 'FALSE'
        REPLAN = False
    else:
        col1 = 'green'
        col2 = 'white'
        lbl_replan['text'] = 'TRUE'
        REPLAN = True
    btn_replan['bg'] = col1
    btn_replan['fg'] = col2

btn_replan = Button(window, text='REPLAN',bg='grey',fg = 'black',command=clicked_replan,width=50)
btn_replan.grid(column=0, row=8)
lbl_replan = Label(window, text='FALSE')
lbl_replan.grid(column=1,row=8)

"""
# **********************************MPC*****************************************
"""
def clicked_mpc():
    global  MPC
    if btn_mpc['bg'] == 'green':
        col1 = 'grey'
        col2 = 'black'
        lbl_mpc['text'] = 'FALSE'
        MPC = False
    else:
        col1 = 'green'
        col2 = 'white'
        lbl_mpc['text'] = 'TRUE'
        MPC = True
    btn_mpc['bg'] = col1
    btn_mpc['fg'] = col2

btn_mpc = Button(window, text='MPC',bg='grey',fg = 'black',command=clicked_mpc,width=50)
btn_mpc.grid(column=0, row=9)
lbl_mpc = Label(window, text='FALSE')
lbl_mpc.grid(column=1,row=9)

window.mainloop()
window.quit()
################################GUI ENDS HERE###################################

print("MOCAP VALUE IS: {}".format(MOCAP_AVAILABLE))
print("INITIAL_STATE VALUE IS: {}".format(INITIAL_STATE_SET))
print("PID VALUE IS: {}".format(PID))
print("PURE_PURSUIT VALUE IS: {}".format(PURE_PURSUIT))
print("LIFT VALUE IS: {}".format(LIFT))
print("DROP VALUE IS: {}".format(DROP))
print("REPLAN VALUE IS: {}".format(REPLAN))
print("MPC VALUE IS: {}".format(MPC))


"""
GLOBAL DECLARATIONS FOR PERFORMANCE EVALUATION
"""
TRAJ_ERROR = []
TIME_TAKEN_IND = []
TIME_TAKEN_TOT = 0
VELOCITIES  = []
STEERING_ANGLE = []
ACTUAL_X = []
ACTUAL_Y = []
ACTUAL_THETA = []
"""
******************************ALL FUNCTIONS*************************************
"""
def blockPrint():
    sys.stdout = open(os.devnull, 'w')

# Restore
def enablePrint():
    sys.stdout = sys.__stdout__
"""
ROS subscribers and publishers
"""
def state_subscriber():
    #rospy.Subscriber('/fused_state_publisher', state_msg, state_cb)
    rospy.Subscriber('/state_publisher_vio', state_msg, state_cb)

def cmd_vel_publisher():
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10) #publish rate.
    return cmd_pub, rate

cmd_pub, rate = cmd_vel_publisher()
def state_cb(data):
    global state, INITIAL_STATE_SET
    #state update
    state.x = data.data[0]#*(10**(-3)) #mm to meter conversion
    state.y = data.data[1]#*(10**(-3))
    state.theta = data.data[2]
    if not INITIAL_STATE_SET:
        state_ini.x = state.x
        state_ini.y = state.y
        state_ini.theta = state.theta
        INITIAL_STATE_SET = True
        enablePrint()
        print "Initial state set!"
        blockPrint()

def publish_control(velocity,steer_angle,fork, cmd_pub, rate):

    data = Twist()
    data.linear.x = velocity
    data.angular.z = steer_angle
    data.linear.z = fork
    cmd_pub.publish(data)

    rate.sleep()

"""
Quest Planner Specific functions
"""
def resolve_val(theta):
    final_theta = theta + m.pi
    # final_theta = wrapToPi(final_theta)
    return final_theta

def wrapToPi(theta):
    return m.atan2(m.sin(theta),m.cos(theta))

def segregate_paths(x_traj, y_traj):
    visualize = False
    master_path = []
    path = []
    for i in range(len(x_traj)-2):
        first = [x_traj[i], y_traj[i]]
        mid = [x_traj[i+1], y_traj[i+1]]
        last = [x_traj[i+2], y_traj[i+2]]
        check_angle = abs(wrapToPi(m.atan2((last[1]-mid[1]),(last[0]-mid[0])) - m.atan2((mid[1]-first[1]),(mid[0]-first[0]))))
        if visualize:
            plt.plot([first[0],mid[0],last[0]],[first[1],mid[1],last[1]])
            plt.pause(0.1)
        if check_angle > m.pi/2:
            path.append(first)
            path.append(mid)
            master_path.append(path)
            path = []
        else:
            path.append(first)
    master_path.append(path)
    return master_path


def RSP_path(start, goal):
    print(start, goal)
    radius = 3.3#4.0#2.0 #4 worked best #5
    curvature = 1/radius
    step_size = 0.1

    px, py, pyaw, mode, clen = RSP_func(
        start[0], start[1], start[2], goal[0], goal[1], goal[2], curvature, step_size)

    plt.plot(px, py, label="final course " + str(mode))

    # plotting
    plt.arrow(start[0],start[1],2*m.cos(start[2]),2*m.sin(start[2]),color='g',head_width=0.5, head_length=1)
    plt.arrow(goal[0],goal[1],2*m.cos(goal[2]),2*m.sin(goal[2]),color='g',head_width=0.5, head_length=1)

    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.xlim(-10,10)
    plt.ylim(-10,10)
    return(px,py)

def process_path(x_traj, y_traj):
    final_path = []
    for i in range(len(x_traj)):
        final_path.append([x_traj[i],y_traj[i]])
    #APPEND THE PATH SO THAT THE LAST THING BECOMES TRACKABLE
    last_theta = m.atan2((y_traj[-1]-y_traj[-2]),(x_traj[-1]-x_traj[-2]))
    x_ap = x_traj1[-1] + 2*m.cos(last_theta)
    y_ap = y_traj1[-1] + 2*m.sin(last_theta)
    # final_path.append([x_ap, y_ap])
    thetas = m.atan2((y_traj[1]-y_traj[0]),(x_traj[1]-x_traj[0]))

    return(final_path, thetas)

def obtain_rear(x,y,theta):
    L = 2.33
    x_rear = x + 2.33*m.cos(theta) #addition since the theta is already resolved
    y_rear = y + 2.33*m.sin(theta)

    return([x_rear,y_rear,theta])

def find_extended_point(goal):
    x, y, theta = goal
    x_new = x - 2.33*m.cos(theta)
    y_new = y - 2.33*m.sin(theta)
    theta_new = theta
    return([x_new, y_new, theta_new])

"""
RSP TRACKING DIFF TRAVERSAL FUNCTIONS
"""
def update_rear(x,y,theta,v,s):
    dt = 0.1
    L = 2.33
    x+= (v/2)*(m.cos(theta-s) + m.cos(theta+s))*dt
    y+= (v/2)*(m.sin(theta-s) + m.sin(theta+s))*dt
    theta+= -(v/L)*m.sin(s)*dt
    return(x,y,theta)

def plot_goals(goals):
    for goal in goals:
        plt.arrow(goal[0], goal[1], 1.5*m.cos(goal[2]), 1.5*m.sin(goal[2]),
          head_width=0.1, head_length=0.2)

def return_dist(pt1, pt2):
    x1, y1 = pt1
    x2, y2 = pt2
    return(m.sqrt((x1-x2)**2 + (y1-y2)**2))

def seek_one_pure_pursuit(start,goal,true_goal):
    global STEERING_ANGLE, VELOCITIES, ACTUAL_THETA, ACTUAL_X, ACTUAL_Y

    signal = False
    xs,ys,thetas = start
    xt, yt = goal #this is the target point
    xg, yg, thetag = true_goal #this is the final goal

    """
    EXPERIMENTAL FAILED
    """
    # shift_angle = m.atan2((perp[1] - yt), (perp[0]-xt))
    # xt = perp[0] - (2.33 + 0.85)*m.cos(shift_angle)
    # yt = perp[1] - (2.33 + 0.85)*m.sin(shift_angle)
    """
    NEED TO SEE IF THE Ld should be changed
    that will happen when the angle check_angle is more than 85 here
    MAYBE NOT -EXPERIMENTAL
    """
    phi = m.atan2((yt-ys),(xt-xs))
    heading_error = wrapToPi(phi - thetas)
    if heading_error > (m.pi/2-m.radians(1))  or heading_error <-(m.pi/2-m.radians(1)):
        signal = True
    #     shift_angle = m.atan2((perp[1] - yt), (perp[0]-xt))
    #     xt = perp[0] - (0.85)*m.cos(shift_angle)
    #     yt = perp[1] - (0.85)*m.sin(shift_angle)
    #     plt.plot(xt,yt,'bo')
    #     print("*******************REVERSING***********************************")
    """
    LONGITUDINAL CONTROL USING PID
    """

    Kpd = 1.5
    v = Kpd*m.sqrt((xs-true_goal[0])**2+ (ys-true_goal[1])**2)
    if v>0.325:
         v = 0.325
    if v<-0.325:
         v = -0.325

    """
    FULL SPEED CONTROL
    """
    #v = 0.325
    """
    LONGITUDINAL CONTROL USING CURVATURE BASED LAW
    """
    # add = 1/(ld/(2*m.sin(alpha+0.00001)))
    # scale_factor = 1.5
    # v = scale_factor*abs(0.325/(0.325 + add ))
    # if v>0.325:
    #     v = 0.325
    # if v<-0.325:
    #     v = -0.325
    """
    LATERAL CONTROL
    """
    ld = m.sqrt((xs - xt)**2 + (ys - yt)**2) #this is the lookahead distance
    #need to find the angle alpha
    beta = m.atan2((yt-ys),(xt - xs))
    alpha = wrapToPi(beta - thetas)
    s = -m.atan2((2*2.33*m.sin(alpha)), (ld)) #since we are travelling in reverse the steering angle would have to be originally negative



    """
    decision on front or back based on heading_error signal
    """
    if signal == True:
        v = -v

    if MOCAP_AVAILABLE:
        #NEED TO PUBLISH HERE
        cmd_pub, rate = cmd_vel_publisher()
        v = -v
        s = s - m.radians(8)
        publish_control(v, s,0, cmd_pub, rate)
        state_val = obtain_rear(state.x, state.y, resolve_val(state.theta))
        xs,ys,thetas = state_val
    else:
        xs,ys,thetas = update_rear(xs,ys,thetas,v,s)
        # "for checking"
        thetas = resolve_val(thetas)
        thetas = resolve_val(thetas)
    ACTUAL_X.append(xs)
    ACTUAL_Y.append(ys)
    ACTUAL_THETA.append(thetas)
    VELOCITIES.append(v)
    STEERING_ANGLE.append(s)
    return xs,ys,thetas,s

"""
MPC specific functions
"""
def dynamics_rear(X_prev, S, v):
    dt = 0.1
    L = 2.33
    X_new = X_prev + dt*casadi.blockcat([[casadi.mtimes(v/2.0,casadi.cos(X_prev[2] - S - 8*m.pi/180)+casadi.cos(X_prev[2] + (S + 8*m.pi/180)))],
                [casadi.mtimes(v/2.0,casadi.sin(X_prev[2] - S - 8*m.pi/180) + casadi.sin(X_prev[2] + (S + 8*m.pi/180)))],
                [casadi.mtimes(v,casadi.sin(-S - 8*m.pi/180)*1/L)]])

    X_new[2] = casadi.atan2(casadi.sin(X_new[2]), casadi.cos(X_new[2]))
    return X_new

def lower_bound(K):
    steer_angle_max = m.pi/2

    ones = casadi.DM.ones(1,K)
    lb = -steer_angle_max*ones
    lb = casadi.reshape(lb,K,1)

    return lb

def upper_bound(K):
    steer_angle_max = m.pi/2

    ones = casadi.DM.ones(1,K)
    ub = steer_angle_max*ones
    ub = casadi.reshape(ub,K,1)

    return ub

def acceleration_limit(S, S_i, K):

    c = casadi.MX(1,K) #constraint variable

    S = casadi.reshape(S,1,K)
    S_prev = S_i #initial condition
    i=0;

    for i in range(K):
        c[0,i] = casadi.mtimes((S[i]-S_prev).T,(S[i]-S_prev)) - 1.414**2
        S_prev = S[i]

    c = casadi.reshape(c,1,K)

    return c

def MPC_cost(v,K,S,S_i,x_i,goal,true_goal,signal_positive,signal_negative): #velocity, Horizon, Steering variables, Steering angle initial, start, goal, true_goal
    cost = 0

    S = casadi.reshape(S,1,K)

    R = casadi.DM([[0.5,0],[0,0.5]]) # s, s_dot
    Q = casadi.DM([35]) # heading_error
    Qf = casadi.DM([100])

    X = casadi.MX(3,K) #x,y,theta,heading_error

    S_vec = casadi.MX(2,1) # s and s_dot

    for i in range(K):

        if i==0:
            X[:,i] = dynamics_rear(x_i[:],S[i],casadi.DM([v])) #calculate new state

            S_vec[0] = S[i]
            S_vec[1] = S_i-S[i]

        else:
            X[:,i] = dynamics_rear(X[:,i-1],S[i],casadi.DM([v])) #calculate new state

            S_vec[0] = S[i]
            S_vec[1] = S[i]-S[i-1]

        """
        Longitudinal control
        """
        x = X[0,i]
        y = X[1,i]
        theta = X[2,i]

        """
        heading error
        """
        beta = casadi.atan2((goal[1]-y),(goal[0]-x))
        heading_error = casadi.atan2(casadi.sin(beta - theta), casadi.cos(beta - theta))
        e = heading_error #may be need MX here
        if signal_positive:
            e = (casadi.pi - e)
        if signal_negative:
            e = -(casadi.pi - casadi.fabs(e))
        cost = cost + (casadi.mtimes(casadi.mtimes(S_vec.T,R),S_vec) +
                    casadi.mtimes(casadi.mtimes(e.T,Q),e))


    x = X[0,i]
    y = X[1,i]
    theta = X[2,i]
    beta = casadi.atan2((goal[1]-y),(goal[0]-x))
    heading_error = casadi.atan2(casadi.sin(beta - theta), casadi.cos(beta - theta))
    e = heading_error #may be need MX here
    if signal_positive:
        e = (casadi.pi - e)
    if signal_negative:
        e = -(casadi.pi - casadi.fabs(e))

    cost = cost +  casadi.mtimes(casadi.mtimes(e.T,Q),e) #terminal cost

    return cost


def seek_one_MPC(start, goal, true_goal,S_i): #start, goal, true_goal, steering angle
    K = 5
    opti = casadi.Opti()

    S = opti.variable(K, 1)
    """
    Longitudinal control
    """
    x,y,theta = start
    Kpd = 1.5 #0.5
    v = Kpd*m.sqrt((x-true_goal[0])**2+ (y-true_goal[1])**2)
    if v>0.325:
        v = 0.325
    if v<-0.325:
        v = -0.325

    v_orig = v
    #thets calculations
    beta = m.atan2((goal[1]-y),(goal[0]-x))
    dist_error = m.sqrt((x-goal[0])**2 + (y-goal[1])**2)
    heading_error = wrapToPi(beta - theta)
    signal= False
    signal_positive = False
    signal_negative = False
    # print(m.degrees(heading_error),m.degrees(beta))
    if heading_error >= m.pi/2:
        heading_error = (m.pi - heading_error)
        signal = True
        signal_positive = True
    if heading_error <= -m.pi/2:
        heading_error = -(m.pi - abs(heading_error))
        signal = True
        signal_negative = True
    if signal == True:
        v_orig = -v
    opti.minimize(MPC_cost(v_orig,K,S,S_i,start,goal,true_goal,signal_positive,signal_negative)) #velocity, Horizon, Steering variables, Steering angle initial, start, goal, true_goal
    opti.subject_to(S <= upper_bound(K))
    opti.subject_to(S >= lower_bound(K))
    opti.subject_to(acceleration_limit(S, S_i, K) <= 0)
    opti.solver('ipopt')
    sol = opti.solve()
    S_opti = sol.value(S)
    S_opti = casadi.reshape(S_opti, 1, K)

    print("optimal steering angle is {}".format(S_opti))
    """
    Extract the first steering angle and first v to update
    """
    s = S_opti[0]
    if MOCAP_AVAILABLE:
        #NEED TO PUBLISH HERE
        cmd_pub, rate = cmd_vel_publisher()
        v = -v_orig
        s = s - m.radians(8)
        publish_control(v, s,0, cmd_pub, rate)
        state_val = obtain_rear(state.x, state.y, resolve_val(state.theta))
        x,y,theta = state_val
    else:
        x,y,theta = update_rear(x,y,theta,v_orig,s)

    ACTUAL_X.append(x)
    ACTUAL_Y.append(y)
    ACTUAL_THETA.append(theta)
    print("V: {}".format(v))
    VELOCITIES.append(v)
    STEERING_ANGLE.append(s)
    return x,y,theta,s

def calc_target_new(x,y,theta,goal_points):
    global TRAJ_ERROR
    last_flag = False
    if len(goal_points) >=3:
        goal_pts_considered = goal_points
    else:
        goal_pts_considered = goal_points

    best_dist = 999 #random
    best_pt = [x,y] #random
    best_target = [x,y] #random

    for i in range(len(goal_pts_considered)):
        x_path, y_path = goal_pts_considered[i]
        perp_dist = m.sqrt((x-x_path)**2 + (y-y_path)**2)
        if perp_dist < best_dist:
            best_dist = perp_dist
            best_pt = [x_path, y_path]
            best_index = i
    #After everything we need to find the target point
    #traverse forward from index i and find a point which is a certain distance ahead of it
    index = best_index
    if PID:
        lookahead_idx = 10
    if PURE_PURSUIT:
        lookahead_idx = 10
    if MPC:
        lookahead_idx = 7

    target_index = index + lookahead_idx
    push_off_val = 0.2#0.5#1.5#2.33 #5
    if target_index > len(goal_points)-1:
        last_flag = True

        last_pt = goal_points[-1]
        second_last_pt = goal_points[-2]
        angle = m.atan2((last_pt[1]-second_last_pt[1]),(last_pt[0]-second_last_pt[0]))
        target_ptx = last_pt[0] + push_off_val*m.cos(angle)
        target_pty = last_pt[1] + push_off_val*m.sin(angle)
        target_pt = [target_ptx, target_pty]

    else:
        target_pt = goal_points[target_index]

    TRAJ_ERROR.append(best_dist)
    return(target_pt, last_flag)

def seek_one_pid(start,goal,true_goal):
    global STEERING_ANGLE, VELOCITIES, ACTUAL_THETA, ACTUAL_X, ACTUAL_Y
    """
    ASSUMPTION: Here I am receving the rear point
    """
    x,y,theta = start
    Kp = 4.5 #5 works best
    # Kp = 0.85
    Kpd = 1.5 #0.5
    v = Kpd*m.sqrt((x-true_goal[0])**2+ (y-true_goal[1])**2)
    if v>0.325:
        v = 0.325
    if v<-0.325:
        v = -0.325
    #thets calculations
    beta = m.atan2((goal[1]-y),(goal[0]-x))
    dist_error = m.sqrt((x-goal[0])**2 + (y-goal[1])**2)
    heading_error = wrapToPi(beta - theta)
    signal= False
    # print(m.degrees(heading_error),m.degrees(beta))
    if heading_error >= m.pi/2:
        heading_error = (m.pi - heading_error)
        signal = True
    if heading_error <= -m.pi/2:
        heading_error = -(m.pi - abs(heading_error))
        signal = True
    s = -Kp*heading_error

    if s > m.pi/2:
        s = m.pi/2
    if s<-m.pi/2:
        s = -m.pi/2
    if signal == True:
        v = -v

    if MOCAP_AVAILABLE:
        #NEED TO PUBLISH HERE
        cmd_pub, rate = cmd_vel_publisher()
        v = -v
        s = s - m.radians(8)
        publish_control(v, s,0, cmd_pub, rate)
        state_val = obtain_rear(state.x, state.y, resolve_val(state.theta))
        x,y,theta = state_val
    else:
        x,y,theta = update_rear(x,y,theta,v,s)

    ACTUAL_X.append(x)
    ACTUAL_Y.append(y)
    ACTUAL_THETA.append(theta)
    print("V: {}".format(v))
    VELOCITIES.append(v)
    STEERING_ANGLE.append(s)
    return x,y,theta,s

def path_track4(path,thetas, x_traj_tot, y_traj_tot,goals):
    SAMPLING  = False
    # thetas = 0 #cancelling user defined theta
    win_zoom = 7
    """
    SAMPLING STARTS HERE
    """
    if SAMPLING:
        final_path = []
        x,y = path[0]
        sample_rate = 0.2 #best was 2 #changed again from 0.2
        final_path.append([x,y])
        for x,y in path:
            xf,yf = final_path[-1]
            if m.sqrt((xf-x)**2 + (yf-y)**2)>sample_rate:
                final_path.append([x,y])
            else:
                continue

        if path[-1] not in final_path:
            final_path.append(path[-1])
    else:
        final_path = path

    prev_path = path
    path = final_path
    prev_path_array = np.array(prev_path)

    tic = time.time()
    xstart, ystart = path[0]
    start = [xstart,ystart,thetas]
    goal_points = path

    dummy_gp = copy.deepcopy(goal_points)


    #need to calculate goal theta last two points
    #last_pt = dummy_gp[-1]
    #second_last_pt = dummy_gp[-2]
    #theta_g = m.atan2((last_pt[1]-second_last_pt[1]),(last_pt[0]-second_last_pt[0]))
    theta_g = -m.pi/2
    goalx,goaly = goal_points[-1]
    goal = [goalx,goaly, theta_g]

    x,y,theta = start
    v = 1
    s = 0
    gp_array = np.array(goal_points)
    x_traj = []
    y_traj = []

    skip = False
    dist_thresh = 0.2# 0.1#0.05
    while m.sqrt((x - goal[0])**2 + (y - goal[1])**2)>dist_thresh:
        #first step would be to find the target point
        [xt, yt], last_flag = calc_target_new(x,y,theta,dummy_gp)
        plt.cla()
        plt.axis('scaled')
        plt.xlim(x-win_zoom,x+win_zoom)
        plt.ylim(y-win_zoom,y+win_zoom)
        plt.plot(gp_array[:,0],gp_array[:,1],'m',label="Sampled-Target-path")
        plt.plot(prev_path_array[:,0],prev_path_array[:,1],'c--',label="Actual-Target-path")
        plt.plot(x_traj_tot, y_traj_tot, 'g--',label="REPLANNED PATH")
        plot_goals(goals)
        plt.plot(start[0],start[1],'co')
        plt.plot(xt,yt,'ro')

        if DRAW_PALLET:
            dpj(x,y,theta,s)

        x_traj.append(x)
        y_traj.append(y)
        plt.plot(x_traj,y_traj,'r',label="Actual-Path-taken")
        if PID:
            x,y,theta,s = seek_one_pid([x,y,theta],[xt,yt],goal)
        if PURE_PURSUIT:
            x,y,theta,s = seek_one_pure_pursuit([x,y,theta],[xt,yt],goal)
        if MPC:
            x,y,theta,s = seek_one_MPC([x,y,theta],[xt,yt],goal,s)
        plt.pause(0.0001)
        """
        BREAK logic
        """
        if last_flag:
            if m.sqrt((x-xt)**2 + (y-yt)**2)<0.2:
                break
    if ALIGN_TO_GOAL_LINE:
        pt1 = goal_points[-2]
        pt2 = goal_points[-1]
        while wrapToPi(abs(theta - goal[2]))>0.1:
            _,_,xt,yt,_ = calc_perp(x,y,theta,pt1,pt2)
            plt.cla()
            plt.axis('scaled')
            plt.xlim(x-win_zoom,x+win_zoom)
            plt.ylim(y-win_zoom,y+win_zoom)
            plt.plot(gp_array[:,0],gp_array[:,1],'m',label="Sampled-Target-path")
            plt.plot(prev_path_array[:,0],prev_path_array[:,1],'c--',label="Actual-Target-path")
            plt.plot(x_traj_tot, y_traj_tot, 'g--',label="REPLANNED PATH")
            plot_goals(goals)
            plt.plot(start[0],start[1],'co')
            plt.plot(xt,yt,'ro')

            if DRAW_PALLET:
                dpj(x,y,theta,s)

            x_traj.append(x)
            y_traj.append(y)
            plt.plot(x_traj,y_traj,'r',label="Actual-Path-taken")
            if PID:
                x,y,theta,s = seek_one_pid([x,y,theta],[xt,yt],goal)
            if PURE_PURSUIT:
                x,y,theta,s = seek_one_pure_pursuit([x,y,theta],[xt,yt],goal)
            plt.pause(0.0001)

    #if abs(wrapToPi(theta_g - theta))>m.pi/2:
    #    theta_g = m.atan2((-last_pt[1]+second_last_pt[1]),(-last_pt[0]+second_last_pt[0]))
    #else:
    #    pass

    ##check for actual goal if possible
    #if abs(wrapToPi(theta_g + m.pi/2))<0.5:
    #    theta_g = -m.pi/2

    heading_error = wrapToPi(theta_g - theta)
    print("heading_error initial value {}".format(m.degrees(heading_error)))
    print("thresh {}".format(m.degrees(0.005)))
    if ROTATE_AT_POINT:
        """
        This will come into picture when the final goal orienation has been met
        """

        #first need to decide whether to rotate clockwise or counter clockwise
        Kpv = 2

        while abs(heading_error) > 0.005:
            plt.cla()
            plt.axis('scaled')
            plt.xlim(x-win_zoom,x+win_zoom)
            plt.ylim(y-win_zoom,y+win_zoom)
            plt.plot(gp_array[:,0],gp_array[:,1],'m',label="Sampled-Target-path")
            plt.plot(prev_path_array[:,0],prev_path_array[:,1],'c--',label="Actual-Target-path")
            plt.plot(x_traj_tot, y_traj_tot, 'g--',label="REPLANNED PATH")
            plot_goals(goals)
            plt.plot(start[0],start[1],'co')
            v = Kpv*heading_error

            if v > 0.325:
                v = 0.325
            if v <-0.325:
                v = -0.325

            s = -m.pi/2
            if MOCAP_AVAILABLE:
                state_val = obtain_rear(state.x, state.y, resolve_val(state.theta))
                x,y,theta = state_val
                #NEED TO PUBLISH HERE
                cmd_pub, rate = cmd_vel_publisher()
                v = -v
                s = s - m.radians(8)
                publish_control(v, s,0, cmd_pub, rate)
            else:
                x,y,theta = update_rear(x,y,theta,v,s)
            heading_error = wrapToPi(theta_g - theta)
            if DRAW_PALLET:
                dpj(x,y,theta,s)
            x_traj.append(x)
            y_traj.append(y)
            plt.plot(x_traj,y_traj,'r',label="Actual-Path-taken")
            plt.pause(0.0001)
            enablePrint()
            print("HEADING ERROR: {}".format(m.degrees(heading_error)))


    print("Time taken: {} s".format(time.time()-tic))
    TIME_TAKEN_IND.append(time.time()-tic)
    # plt.title('PID BASED CONSTANT SPEED PATH TRACKING OF A PALLET JACK')
    if PURE_PURSUIT:
        plt.title('PURE-PURSUIT BASED PATH TRACKING OF A PALLET JACK')
    if PID:
        plt.title('PID BASED PATH TRACKING OF A PALLET JACK')
    if MPC:
        plt.title('MPC BASED PATH TRACKING OF A PALLET JACK')
    plt.legend()
    # plt.show()
    return x,y,theta

def quest_planner(): #model
    global TIME_TAKEN_TOT, TIME_TAKEN_IND, x_traj_planned, y_traj_planned
    """
    MAKING SURE THAT THE INITIAL STATE IS SET BEFORE PROCEEDING
    """
    if MOCAP_AVAILABLE:
        state_subscriber()

        while not INITIAL_STATE_SET:
            pass
        statex = state.x
        statey = state.y
        state_theta = state.theta
    else:
        statex = 3.992
        statey = -3.971
        state_theta = -0.91
    """
    UNPACKING
    """
    # vmax = model.vel_max
    # smax = model.steer_angle_max

    """
    DPD MANEUVER
    """
    tht = resolve_val(state_theta)
    print(type(tht))
    start = [statex,statey,tht] #this is fronts data we need to obtain rear
    start = obtain_rear(statex,statey,resolve_val(state_theta))
    # print(type(start))
    print("start: {}".format(start))
    #goals = [[2,-3.5,-m.pi/2],[2,-5.0,-m.pi/2],[0,-2.0,-m.pi/2],[0,-3.5,-m.pi/2],[0,-5.0,-m.pi/2]]
    #goal1 = obtain_rear(-0.8506,-0.4385,resolve_val(-1.589))
    """
    VICON GOALS
    """
    goal1 = obtain_rear(-1.026,-0.788,resolve_val(-1.647))
    goal2 = obtain_rear(-0.953,1.254,resolve_val(-1.658))
    goal3 = obtain_rear(0.852,-3.5,resolve_val(-1.650)) #-1.929
    goal4 = obtain_rear(0.993,1.331,resolve_val(-1.669))
    goals = [goal1,goal2,goal3,goal4]
    print(goal1)
    print(goal2)
    print(goal3)
    print(goal4)

    """
    VIO GOALS
    """
    goal1 = obtain_rear(2.456,0.122,resolve_val(0.0167))
    goal2 = obtain_rear(-0.250,0.00,resolve_val(0.000))
    goal3 = obtain_rear(2.504,2.052,resolve_val(0.003)) #-1.929
    goal4 = obtain_rear(-0.600,1.944,resolve_val(-0.009))
    goals = [goal1,goal2,goal3,goal4]

    theta = start[2]
    x_traj = []
    y_traj = []
    x_traj_global = []
    y_traj_global = []
    """
    OBTAINING THE FULL PATH PLAN
    """
    dummy_start = copy.deepcopy(start)
    for gl in goals:
        px, py = RSP_path(dummy_start, gl)


        x_traj_global.extend(px)
        y_traj_global.extend(py)
        dummy_start = gl
    """
    *********************SEQUENTIAL PATH PLANNING*******************************
    """
    goals = [goal1, goal2]
    glob_tic = time.time()
    for goal in goals:
        px, py = RSP_path(start, goal)
        master_path = segregate_paths(px, py)
        x_traj.extend(px)
        y_traj.extend(py)

        for paths in master_path:
            x,y,theta = path_track4(paths, theta,x_traj, y_traj,goals)
            path_array = np.array(paths)
            plt.plot(path_array[:,0],path_array[:,1])
            plt.pause(1)

        if REPLAN:
            start = [x,y,theta]
        else:
            start = goal
        """
        LOGIC FOR HEADING ERROR CORRECTION HERE
        """

    """
    Time to LIFT
    """
    if LIFT:
        enablePrint()
        print("[INFO] Initiating LIFT")
        publish_control(0,0,1,cmd_pub,rate)
        time.sleep(0.1)
        publish_control(0,0,1,cmd_pub,rate)
        time.sleep(0.1)
        publish_control(0,0,1,cmd_pub,rate)
        time.sleep(0.1)
        publish_control(0,0,1,cmd_pub,rate)
        time.sleep(0.1)
        publish_control(0,0,1,cmd_pub,rate)

        time.sleep(3)
        print("[INFO] Exiting LIFT Sequence")

    #second set of goals go here
    goals = [goal3, goal4]
    for goal in goals:
        px, py = RSP_path(start, goal)
        master_path = segregate_paths(px, py)
        x_traj.extend(px)
        y_traj.extend(py)

        for paths in master_path:
            x,y,theta = path_track4(paths, theta,x_traj, y_traj,goals)
            path_array = np.array(paths)
            plt.plot(path_array[:,0],path_array[:,1])
            plt.pause(1)

        if REPLAN:
            start = [x,y,theta]
        else:
            start = goal
        """
        LOGIC FOR HEADING ERROR CORRECTION HERE
        """

    if DROP:
        print("[INFO] Lowering down")
        #while time.time() - tic <1:
        publish_control(0,0,2,cmd_pub,rate)
        time.sleep(0.1)
        publish_control(0,0,2,cmd_pub,rate)
        time.sleep(0.1)
        publish_control(0,0,2,cmd_pub,rate)
        time.sleep(0.1)
        publish_control(0,0,2,cmd_pub,rate)
        time.sleep(0.1)
        publish_control(0,0,2,cmd_pub,rate)
        time.sleep(0.1)
        publish_control(0,0,2,cmd_pub,rate)
        time.sleep(0.1)
        publish_control(0,0,2,cmd_pub,rate)
        time.sleep(0.1)
        publish_control(0,0,2,cmd_pub,rate)
        time.sleep(0.1)
        publish_control(0,0,2,cmd_pub,rate)

        time.sleep(0.1)

    publish_control(0,0,0,cmd_pub,rate)

    TIME_TAKEN_TOT = time.time() - glob_tic

    """
    *********************PATH TRACKING ENDS HERE********************************
    """
    """
    PERFORMANCE EVALUATIONS
    """
    if MOCAP_AVAILABLE:
        evaluate_performance(x_traj, y_traj)
    plt.show()

def evaluate_performance(x_traj, y_traj):
    global TRAJ_ERROR, TIME_TAKEN_IND, TIME_TAKEN_TOT, VELOCITIES, STEERING_ANGLE, ACTUAL_X, ACTUAL_Y
    plt.cla()
    plt.title("Traj errors")
    plt.xlabel('index')
    plt.ylabel('error')
    plt.xlim(0,200)
    plt.ylim(0,10)
    sum_error = 0
    if PURE_PURSUIT:
        title = 'Pure pursuit ' + str(datetime.datetime.now())
    if PID:
        title = 'PID ' + str(datetime.datetime.now())
    if MPC:
        title = 'MPC ' + str(datetime.datetime.now())

    with open('performance' + title + '.txt','w') as file:
        file.write("Errors in trajectory")
        for i in range(len(TRAJ_ERROR)):
            #plt.plot(i,TRAJ_ERROR[i],'co',label="Trajectory error")
            sum_error+=TRAJ_ERROR[i]
            file.write(str(TRAJ_ERROR[i]) + '\n')
        file.write("\n Planned path \n")
        for i in range(len(x_traj)):
            file.write(str(x_traj[i]) + '  ' + str(y_traj[i]) + '\n')

        file.write("\n PATH_TAKEN \n")
        for i in range(len(ACTUAL_X)):
            file.write(str(ACTUAL_X[i]) + '  ' + str(ACTUAL_Y[i])  + '  ' + str(ACTUAL_THETA[i])+ '\n')

        file.write('sum_error: {} \n'.format(sum_error))
        avg_error = sum_error/len(TRAJ_ERROR)
        print("**********************PERFORMANCE***********************************\n")
        print("Average Error: {}".format(avg_error))

        tot_time_wo_pause = 0
        for i in range(len(TIME_TAKEN_IND)):
            plt.plot(i,TIME_TAKEN_IND[i],'co',label="Time taken")
            tot_time_wo_pause+=TIME_TAKEN_IND[i]
        print("Total time without considering pause: {}".format(tot_time_wo_pause))
        file.write('Time wo pause: {} \n'.format(tot_time_wo_pause))

        gross_time = TIME_TAKEN_TOT
        print("Gross time: {}".format(gross_time))
        file.write('Gross Time: {}\n'.format(gross_time))

    #with open('controls ' + title + '.txt','w') as file:
        file.write('velocity(m/s) |  steering angle(radians) \n')
        for i in range(len(VELOCITIES)):
            file.write(str(VELOCITIES[i]) + '  ' + str(STEERING_ANGLE[i]) + '\n')

        plt.cla()
        plt.plot(ACTUAL_X, ACTUAL_Y,'b--',label='ACTUAL PATH')
        plt.plot(x_traj,y_traj,'r--',label='Path Planned')
        plt.legend()
        plt.show()



quest_planner()
