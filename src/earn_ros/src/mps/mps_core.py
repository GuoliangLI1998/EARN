#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry, Path
import numpy as np
from std_msgs.msg import Float64, Bool
import cvxpy as cp
from visualization_msgs.msg import Marker

class mps_core:
    def __init__(self):

        rospy.init_node('mps_node', anonymous=True)

        self.clock = 0
        self.start_time = 0.0
        self.edge_state = rospy.get_param("edge_position", [255, 172])
        
        # edge position
        self.commun_coverage = rospy.get_param("~coverage", 500)
        self.C_th = rospy.get_param("~Cth", 30)
        
        ## load robot info
        self.robot_num = 2
        self.robot_info = [rospy.get_param(
            "~robot_1",
            {
                "receding": 15,
                "max_edge_num": 4,
                "max_obs_num": 4,
            },
        ), rospy.get_param(
            "~robot_2",
            {
                "receding": 15,
                "max_edge_num": 4,
                "max_obs_num": 4,
            },
        )]

        self.robot_state = [[0 for _ in range(3)] for _ in range(self.robot_num)]
        self.robot_path = [[0 for _ in range(3)] for _ in range(self.robot_num)]

        self.stuck = [0.0] * self.robot_num
        self.switching_gain = [0.0] * self.robot_num
        self.commun_time_list = [0.0] * self.robot_num
        self.compute_time_list = [0.0] * self.robot_num
        self.x_last = [0] * self.robot_num

        # Subscriber
        [setattr(self, f'sub_agent_{i}_{name}', rospy.Subscriber(f"/carla/agent_{i}/{topic}", msg_type, getattr(self, f'agent_{i}_{callback}')))
        for i in [1, 2]
        for name, topic, msg_type, callback in [
            ('odom', 'odometry', Odometry, 'state_callback'),
            ('path', 'pdd_opt_path', Path, 'path_callback'), 
            ('stuck', 'stuck', Float64, 'stuck_callback')
        ]]

        # publisher, edge accelerate or not
        self.pub_list = [rospy.Publisher(f'/carla/agent_{i}/edge_acc', Bool, queue_size=10) for i in range(1, self.robot_num+1)]
        
        # publisher, show position of edge
        self.pub_marker = rospy.Publisher('edge_marker', Marker, queue_size=10)

    def generate_marker(self, position=[0,0,0]):
        
        # create the marker. 
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.CUBE
        marker.header.stamp = rospy.get_rostime()
        marker.id = -1
        
        # set the pose of the marker. 
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = 1.0
        
        # set the size of the marker. 
        marker.scale.x = 5
        marker.scale.y = 5 
        marker.scale.z = 30

        # set the color of the marker. 
        color = [0, 0.7, 0.7]
        marker.color.a = 1
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]

        # set the orientation to default (no rotation)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        return marker

    def make_decision(self):
   
        # creat edge markers
        marker_array_bbox = self.generate_marker(position=self.edge_state)

        rate = rospy.Rate(1) # 1 Hz

        while not rospy.is_shutdown():

            if self.clock != 0:
                rospy.loginfo('Simulation starts ...')
                self.start_time = self.clock

            self.pub_marker.publish(marker_array_bbox)

            # compute switching gains for robots
            for i,_ in enumerate(self.robot_path):

                dist_vector = np.array([self.robot_path[i][0] - self.robot_state[i][0], \
                                        self.robot_path[i][1] - self.robot_state[i][1]])

                self.switching_gain[i] = self.stuck[i] * np.linalg.norm(dist_vector)

            # compute communication delays for robots
            for i,_ in enumerate(self.robot_state):
                commun_time = self.commun_model(self.robot_state[i])
                self.commun_time_list[i] = commun_time

            # computing models
            for i in range(self.robot_num):
                compute_delay = self.compute_model(self.robot_info[i]['receding'], self.robot_info[i]['max_obs_num'])
                self.compute_time_list[i] = compute_delay

            # define variables
            x = cp.Variable(self.robot_num, integer=True)

            # define objective
            objective = cp.Maximize(cp.sum(cp.multiply(self.switching_gain, x)))

            # define constraints
            basic_contraints = [0 <= x, x <= 1]
            # edge computation delay constraints
            comp_constraints = [cp.sum(cp.multiply(x, self.compute_time_list)) <= self.C_th]
            # computation delay constraints
            commun_constraints = [cp.multiply(x[k], self.commun_time_list[k]) <= 50 for k in range(self.robot_num)]
            # edge collaboration beneficial constraints
            gain_constraints = [cp.multiply(x[k], self.switching_gain[k]-5.0) >= 0.0 for k in range(self.robot_num)]
            # keep edge acceleration until finish overtaking
            keep_constraints = [ cp.multiply(1-x[k], self.stuck[k]* self.x_last[k]) <= 0.1 for k in range(self.robot_num)] 
            
            constraints = []
            constraints += basic_contraints
            constraints += comp_constraints
            constraints += commun_constraints
            constraints += gain_constraints
            constraints += keep_constraints

            # construct problem
            prob = cp.Problem(objective, constraints)

            # solve problem
            prob.solve(solver='ECOS_BB')

            # extract solution
            try:
                x_final = [round(x.value[l]) for l in range(self.robot_num)]
            except:
                x_final = self.x_last 

            # record last decision
            self.x_last = x_final 
            # convert to bool            
            edge_acc_decision = [bool(x) for x in x_final]

            # publish decision
            for i,decision in enumerate(edge_acc_decision):
                self.pub_list[i].publish(decision)
                print('Robot', i+1, 'adopts edge acceleration:', decision)
            
            # print switching gain 
            for i,gain in enumerate(self.switching_gain):
                print('The switching gain of Robot', i+1, 'is:', float(gain), 'm')

            # print communication delay
            for i,delay in enumerate(self.commun_time_list):
                print('The communication delay of Robot', i+1, 'is:', float(delay), 'ms')

            # print total compute delay
            compute_total = np.dot(x_final, self.compute_time_list)
            print('The compute time of Edge is:', float(compute_total), 'ms')

            print('*' * 50)

            rate.sleep()

    def compute_model(self, H, obs_num):
        gamma = 0.2
        tau = 12
        compute_delay = gamma * H * obs_num + tau

        return compute_delay

    def commun_model(self, robot_state):
        robot_edge_dist_vec = np.array([robot_state[0] - self.edge_state[0], \
                                    robot_state[1] - self.edge_state[1], \
                                ])
        
        robot_edge_dist = np.linalg.norm(robot_edge_dist_vec)

        if robot_edge_dist <= self.commun_coverage:
            commun_delay = 30 + np.random.randint(low=-20, high=20, size=1)
        else:
            commun_delay = 100 + np.random.randint(low=-20, high=20, size=1)
        
        return commun_delay

    def clock_callback(self, msg):
        self.clock = msg.clock.secs

    # callback functions of robot 1
    def agent_1_state_callback(self, msg):
        self.robot_state[0] = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]

    def agent_1_path_callback(self, msg):
        try:
            self.robot_path[0] = [msg.poses[-1].pose.position.x, msg.poses[-1].pose.position.y, msg.poses[-1].pose.position.z]   
        except Exception as e:
            print("no switching gain!")

    def agent_1_stuck_callback(self, msg):
        self.stuck[0] = msg.data

    # callback functions of robot 2
    def agent_2_state_callback(self, msg):
        self.robot_state[1] = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]  

    def agent_2_path_callback(self, msg):
        try:
            self.robot_path[1] = [msg.poses[-1].pose.position.x, msg.poses[-1].pose.position.y, msg.poses[-1].pose.position.z]
        except Exception as e:
            print("no switching gain!")

    def agent_2_stuck_callback(self, msg):
        self.stuck[1] = msg.data


if __name__ == '__main__':
    try:
        mk = mps_core() 
        mk.make_decision()
    except rospy.ROSInterruptException:
        pass