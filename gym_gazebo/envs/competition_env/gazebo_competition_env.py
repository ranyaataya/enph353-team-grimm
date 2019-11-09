
import cv2
import gym
import math
import rospy
import roslaunch
import time
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from gym import utils, spaces
from gym_gazebo.envs import gazebo_env
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

from sensor_msgs.msg import Image
from time import sleep

from gym.utils import seeding

# NEED TO EDIT ALL THE REFERENCES TO LAUNCH FILES AND WHATNOT
# LINES TO CHANGE: 29 --> abosolute path, use Zach's becase he is
# the only one working on the driving


class Gazebo_Competition_Env(gazebo_env.GazeboEnv):

    def __init__(self):
        # Launch the simulation with the given launchfile name
        LAUNCH_FILE = '/home/onehalf/enph353_gym-gazebo/gym_gazebo/envs/enph353/src/enph353_training/enph353/enph353_utils/launch/sim.launch'
        gazebo_env.GazeboEnv.__init__(self, LAUNCH_FILE)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world',
                                              Empty)

        self.action_space = spaces.Discrete(3)  # F,L,R
        self.reward_range = (-np.inf, np.inf)
        self.episode_history = []

        self._seed()

        self.bridge = CvBridge()
        self.timeout = 0  # Used to keep track of images with no line detected
        self.pastState = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]
        # Used to keep track of past states

        self.lower_blue = np.array([97,  0,   0])
        self.upper_blue = np.array([150, 255, 255])

    def process_image(self, data):
        '''
            @brief Coverts data into a opencv image and displays it
            @param data : Image data from ROS

            @retval (state, done)
        '''
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # The state array is a list of 13 elements indicating where in the
        # image the line is:
        # i.e.
        # [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] -> line is on the left
        # [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0] -> line is in the center
        # [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0] -> left intersection
        # [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0] -> right intersection
        # [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1] -> T intersection
        # The episode termination condition should be triggered when the line
        # is not detected for more than 30 frames. In this case set the done
        # variable to True.
        #
        # You can use the self.timeout variable to keep track of which frames
        # have no line detected.

        # Pulls image and finds the lane
        state = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        done = False

        # Setting up image masking and region of interest cropping

        lower = 80
        upper = 95
        lower_bgr = np.array([lower, lower, lower])
        upper_bgr = np.array([upper, upper, upper])

        mask = cv2.inRange(cv_image, lower_bgr, upper_bgr)
        newMask = mask

        h, w = newMask.shape[0:2]
        newMask = newMask[:, int(1*w/3):int(2*w/3)]
        h, w = newMask.shape[0:2]

        cv2.imshow('outline', newMask)
        cv2.waitKey(5)

        # Finds the line on each side by iterating through the "height"
        # pixel line of pixels up
        def centerLine(height):
            left = -34
            right = -34

            for x in range(int(w/2)):
                if (newMask[h - height, x] > 0):
                    left = x
                    break

            for x in range(int(w/2), w):
                if (newMask[h - height, w-x-1] > 0):
                    right = w-x
                    break

            # finds center of the lane and calcs amount of lane in camera view
            laneCenter = int((left+right)/2)
            laneCoverage = int((right-left)/w*100)

            return left, right, laneCoverage, laneCenter

        # Uses above function to get an average center lane position
        # for the heights listed in search
        # and give the max number of pixels covered by road (in %)
        search = [5, 7, 10]
        maxCoverage = 0
        centerSum = []
        leftSum = []
        rightSum = []
        for x in range(len(search)):
            left, right, coverage, center = centerLine(search[x])
            if (coverage > maxCoverage):
                maxCoverage = coverage
            centerSum.append(center)
            leftSum.append(left)
            rightSum.append(right)
        averageCenter = int(sum(centerSum)/len(search))
        averageRight = int(sum(rightSum)/len(search))
        averageLeft = int(sum(leftSum)/len(search))

        gap = int(w/10)
        stateIndex = int(averageCenter/gap)

        # Checks for (1) being lost, (2) intersections
        # (using history of past state)
        # and (3) assigns the position state
        coverageThreshold = 90
        if (averageLeft < 0 or averageRight < 0):
            self.timeout = self.timeout + 1
            if (self.timeout > 7):
                done = True
                self.timeout = 0
        elif (maxCoverage > coverageThreshold):
            if(averageRight > w - 5 and averageLeft < 5):
                # assigns state based on history
                useState = self.pastState[0]
                if (useState[11] == 1):
                    state[11] = 1
                elif (useState[10] == 1):
                    state[10] = 1
                else:
                    state[12] = 1
            elif(right == w):
                state[11] = 1  # assigns right interesection
            else:
                state[10] = 1  # assigns left interesection
            self.timeout = 0
        else:        # Goes through the line states and assignes position state
            self.timeout = 0
            state[stateIndex] = 1

        # Builds new state from past state (built like a queue)
        newState = [state]
        for x in range(3):
            newState.append(self.pastState[x])
        self.pastState = newState

        # Turns 4x13 list into a 1x52
        newState1D = []
        for x in range(52):
            useList = newState[int(x/13)]
            newState1D.append(useList[x % 4])

        return newState1D, done

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):

        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        self.episode_history.append(action)

        vel_cmd = Twist()

        if action == 0:  # FORWARD
            vel_cmd.linear.x = 0.4
            vel_cmd.angular.z = 0.0
        elif action == 1:  # LEFT
            vel_cmd.linear.x = 0.0
            vel_cmd.angular.z = 0.5
        elif action == 2:  # RIGHT
            vel_cmd.linear.x = 0.0
            vel_cmd.angular.z = -0.5

        self.vel_pub.publish(vel_cmd)

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/pi_camera/image_raw', Image,
                                              timeout=5)
            except:
                pass

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            # resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        state, done = self.process_image(data)

        # Set the rewards for your action
        if not done:
            if action == 0:  # FORWARD
                reward = 4
            elif action == 1:  # LEFT
                reward = 2
            else:
                reward = 2  # RIGHT
        else:
            reward = -200

        return state, reward, done, {}

    def reset(self):

        print("Episode history: {}".format(self.episode_history))
        self.episode_history = []
        print("Resetting simulation...")
        # Resets the state of the environment and returns an initial
        # observation.
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            # reset_proxy.call()
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print ("/gazebo/reset_simulation service call failed")

        # Unpause simulation to make observation
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            # resp_pause = pause.call()
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        # read image data
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/pi_camera/image_raw',
                                              Image, timeout=5)
            except:
                pass

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            # resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        self.timeout = 0
        state, done = self.process_image(data)

        return state
