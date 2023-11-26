'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    1. the local_trans has to consider different joint axes and link parameters for different joints
    2. Please use radians and meters as unit.
'''

# add PYTHONPATH
import os
import sys
from autograd.numpy import sin, cos, pi
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'joint_control')))

from numpy.matlib import matrix, identity

from recognize_posture import PostureRecognitionAgent


class ForwardKinematicsAgent(PostureRecognitionAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        self.chains = {
            'Head' : ['HeadYaw', 'HeadPitch'],
            'LArm' : ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw'],#, 'LHand'],
            'LLeg' : ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
            'RLeg' : ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'],
            'RArm' : ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw'],#, 'RHand']
        }
        
        alpha, a, d, theta = 0, 0, 0, 0 # For readability
        self.transform_elements = { # All coordinates in the previous joint's coordinate system
            'HeadYaw'           : (alpha, a, d, theta),
            'HeadPitch'         : (alpha -pi/2, a, d, theta -pi/2),
            
            'LShoulderPitch'    : (alpha -pi/2, a, d, theta),
            'LShoulderRoll'     : (alpha +pi/2, a, d, theta),
            'LElbowYaw'         : (alpha +pi/2, a, d +0.105, theta), #TODO: Add elbow offset
            'LElbowRoll'        : (alpha -pi/2, a, d, theta),
            'LWristYaw'         : (alpha +pi/2, a, d +0.055_95, theta),
        
            'RShoulderPitch'    : (alpha -pi/2, a, d, theta),
            'RShoulderRoll'     : (alpha +pi/2, a, d, theta),
            'RElbowYaw'         : (alpha +pi/2, a, d +0.105, theta),
            'RElbowRoll'        : (alpha -pi/2, a, d, theta),
            'RWristYaw'         : (alpha +pi/2, a, d +0.055_95, theta),
        
            'LHipYawPitch'      : (alpha -3/4*pi, a, d, theta -pi/2),
            'LHipRoll'          : (alpha -pi/2, a, d, theta +pi/4),
            'LHipPitch'         : (alpha +pi/2, a, d, theta),
            'LKneePitch'        : (alpha, a -0.100, d, theta),
            'LAnklePitch'       : (alpha, a -0.102_9, d, theta),
            'LAnkleRoll'        : (alpha -pi/2, a, d, theta),
            
            'RHipYawPitch'      : (alpha -pi/4, a, d, theta -pi/2),
            'RHipRoll'          : (alpha -pi/2, a, d, theta -pi/4),
            'RHipPitch'         : (alpha +pi/2, a, d, theta),
            'RKneePitch'        : (alpha, a -0.100, d, theta),
            'RAnklePitch'       : (alpha, a -0.102_9, d, theta),
            'RAnkleRoll'        : (alpha -pi/2, a, d, theta),
        }
        
    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        alpha, a, d, theta = self.transform_elements[joint_name]
        
        theta += joint_angle

        sa, ca = sin(alpha), cos(alpha)
        st, ct = sin(theta), cos(theta)

        return matrix([
            [   ct,     st,    0,      a],
            [st*ca,  ct*ca,  -sa,  -sa*d],
            [st*sa,  ct*sa,   ca,   ca*d],
            [    0,      0,    0,      1],
        ])

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                if joint == "LWristYaw" or joint == "RWristYaw":
                    angle = 0
                else:
                    angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                # YOUR CODE HERE
                T = T @ Tl

                self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
