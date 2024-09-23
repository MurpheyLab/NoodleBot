#! /usr/bin/env python
"""
imports
"""
# general
import numpy as np
import time

# rosenv
import rospy
from std_msgs.msg import Empty, Float32
from noodlebot.msg import swimmer_info, swimmer_reset, swimmer_command
import tf
from tf_conversions import transformations
from roscpp.srv import SetLoggerLevel

class SwimmerEnv(object):
    def __init__(self):
        #  env params        
        self.action_dim = 2 
        self.state_dim = 10 

        self.linearize = True
        if self.linearize: 
            self.state_dim += 1

        self._forward_reward_weight = 1.0
        self._ctrl_cost_weight = 1e-4

        # set up ros
        self.move = rospy.Publisher('/servo_cb',swimmer_command,queue_size=1,latch=True)
        self.move_reset = rospy.Publisher('/servo_reset',swimmer_reset,queue_size=1,latch=True)
        self.reward = rospy.Publisher('/reward',Float32,queue_size=1,latch=True)
        self.fwdreward = rospy.Publisher('/fwdreward',Float32,queue_size=1,latch=True)
        self.totreward = rospy.Publisher('/cumulative_reward',Float32,queue_size=1,latch=True)
        rospy.Subscriber('/joint_swimmer_info', swimmer_info, self.swimmerCallback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/setup_info', Empty, self.setupDoneCallback)
        self.state = np.zeros(self.state_dim)
        self.last_state = np.zeros(self.state_dim)
        self.total_reward = 0

        # coming from the GPS system
        self.current_pose = np.zeros(3) # (x, y, yaw/theta)
        self.current_vel = np.zeros(3)

        # coming from the actual swimmer 
        self.current_joint_pose = np.zeros(2) # (servo1, servo2)
        self.current_joint_vel = np.zeros(2)
        self.dt = 0.05
        self.min_joint_angle = -80
        self.max_joint_angle = 80

        self.got_pose = False
        self.setup_done = False
        self.use_tags = rospy.get_param('april_tags',False)

        if self.use_tags: 
            self.joint_info_at = rospy.Publisher('/joint_swimmer_info_at',swimmer_info,queue_size=1)
            for node in ['apriltag_ros_continuous_node1','usb_cam1','rosout']:
                logger = rospy.ServiceProxy(f'/{node}/set_logger_level', SetLoggerLevel)
                logger('ros','error')
                logger('ros.roscpp.superdebug','error')

            self.got_april_tags = False
            self.listener = tf.TransformListener() # default = interpoation: True, cache_duration = 10.0
            # wait to find first transforms 
            start = time.time()
            while time.time() - start < 10.: 
                self.get_transforms(print_debug=False)
                if self.got_april_tags: 
                    break 
                time.sleep(0.1)
            if not self.got_april_tags: 
                raise TimeoutError("Check that all april tags are visible, didn't any tags")
        else: 
            self.got_april_tags = True

        count = 0.
        while not(self.setup_done):
            time.sleep(0.2)
            count += 1.
            if count > 100: 
                raise ValueError('timed out before pose received')       


        print('swimmer env setup complete')

    def swimmerCallback(self,msg): 
        # reading values from the subscriber
        self.current_joint_pose[0] = msg.joint1pos*(np.pi/180.) # converting pose to radians to match simulator 
        self.current_joint_pose[1] = msg.joint2pos*(np.pi/180.)

        self.current_joint_vel[0] = msg.joint1vel*(np.pi/180.)
        self.current_joint_vel[1] = msg.joint2vel*(np.pi/180.)

        if not self.use_tags:
            self.current_pose[0] = msg.swimmer_x
            self.current_pose[1] = msg.swimmer_y
            self.current_pose[2] = msg.swimmer_direction*(np.pi/180.)

            self.current_vel[0] = msg.swimmervel_x
            self.current_vel[1] = msg.swimmervel_y
            self.current_vel[2] = msg.swimmervel_ang*(np.pi/180.) 
            self.got_april_tags = True
        else: 
            count = 0
            while not(self.got_april_tags) and (count < 10):
                self.get_transforms()
                count += 1
            if self.got_april_tags:
                # allows user to view full joint message with april tag info for debugging
                msg.swimmer_x = self.current_pose[0]
                msg.swimmer_y = self.current_pose[1]
                msg.swimmer_direction = self.current_pose[2]
                msg.swimmervel_x = self.current_vel[0]
                msg.swimmervel_y = self.current_vel[1]
                msg.swimmervel_ang = self.current_vel[2]

                self.joint_info_at.publish(msg)



        if self.linearize:
            x,y,angle = self.current_pose
            tmp_pose = np.hstack([x,y,np.sin(angle),np.cos(angle)])
            self.state = np.hstack([tmp_pose, self.current_joint_pose, self.current_vel, self.current_joint_vel])
        else:
            self.state = np.hstack([self.current_pose, self.current_joint_pose, self.current_vel, self.current_joint_vel])
        self.got_pose = True and self.got_april_tags
        

    def get_transforms(self,print_debug=True):
        x_idx = 0
        y_idx = 1
        angle_idx = 2
        name = 'swimmer'
        try:
            # lookupTransform(target_frame, source_frame, time) -> (position, quaternion)
            current_pos, current_orn = self.listener.lookupTransform(name,'world',rospy.Time(0))
            current_rpy = np.asarray(transformations.euler_from_quaternion(current_orn))

            test = current_pos[x_idx] # try to see if you can access item before saving it to state

            # lookupTwist(tracking_frame, observation_frame, time, averaging_interval) -> (position, quaternion)
            linear_vel, angular_vel = self.listener.lookupTwist(name,'world',rospy.Time(0),rospy.Duration(self.dt*4))

            test = linear_vel[x_idx] # try to see if you can access item before saving it to state

            self.current_pose[0] = current_pos[x_idx]
            self.current_pose[1] = current_pos[y_idx]
            self.current_pose[2] = current_rpy[angle_idx]
            self.current_vel[0] = linear_vel[x_idx]
            self.current_vel[1] = linear_vel[y_idx]
            self.current_vel[2] = angular_vel[angle_idx]

            self.got_april_tags = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            if print_debug:
                print("Check that all april tags are visible, didn't see tag for:",name)
            pass

    def setupDoneCallback(self,msg):
        self.setup_done = True

    def get_reward(self,action): 
        # from https://github.com/openai/gym/blob/master/gym/envs/mujoco/swimmer_v3.py
        x_position_before = self.last_state[0].copy()
        x_position_after = self.state[0].copy()
 
        x_velocity = (x_position_after - x_position_before) / self.dt

        forward_reward = self._forward_reward_weight * x_velocity

        ctrl_cost =  self._ctrl_cost_weight * np.sum(np.square(action))

        reward = forward_reward - ctrl_cost

        # for debugging
        self.reward.publish(Float32(reward))
        self.fwdreward.publish(Float32(forward_reward))
        self.total_reward += reward
        self.totreward.publish(Float32(self.total_reward))
        return reward

    def reset(self):

        start = np.random.randint(self.min_joint_angle,self.max_joint_angle,2)

        print("sending reset:", start)
        self.move_reset.publish(swimmer_reset(self.dt,self.min_joint_angle,self.max_joint_angle,*start))
        self.got_pose = False
        self.got_april_tags = False
        
        # checking if swimmer is done with reset
        while self.got_pose == False:
            time.sleep(0.02)
        self.state[-5:] = 0 # zero velocities

        self.total_reward = 0

        print("reset is done")
        # print("Current joint pose after reset:", self.current_joint_pose*(180/np.pi))
        return self.state.copy()

    def step(self, state, _a):
        scale_val = 50. # gear from mujoco_env
        action = np.clip(_a*scale_val, -scale_val, scale_val) # scale and clip

        # changing from radians to degrees 
        action = action*(180./np.pi)
        
        # publish action input
        command = swimmer_command(*action.astype(int))
        
        self.move.publish(command)
        
        # save state for next iteration
        self.last_state = state.copy()
        self.last_action = _a.copy()

        self.got_pose = False
        self.got_april_tags = False