'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''

import weakref
import xmlrpc.client
import threading
import json
import time
import sys
import os
import io
import numpy as np
from numpy import pi, identity
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))
from keyframes import hello #, rightBackToStand

class PostHandler(object):
    '''the post hander wraps function to be excuted in paralle
    '''

    def __init__(self, obj):
        self.proxy = weakref.proxy(obj)
        #self.threads = {}

    # I started writing a mechanism for asynchronous data retrieval, but it doesn't seem useful in our case.
    # def dispatch_thread(self, name, function, args):
    #     self.threads[name] = threading.Thread(target=function, args=args).start()
    
    # def collect_thread(self, name):
    #     result = self.threads.get(name)

    #     if result is not None and type(result) != threading.Thread:
    #         return result

    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        # YOUR CODE HERE
        threading.Thread(target=self.proxy.server.execute_keyframes, args=(keyframes,)).start()
        #self.dispatch_thread(self.proxy.server.execute_keyframes, (keyframes,)))

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        # YOUR CODE HERE
        threading.Thread(target=self.proxy.server.set_transform, args=( effector_name, self.proxy.serialize_np(transform) )).start()


class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''
    # YOUR CODE HERE
    def __init__(self):
        self.post = PostHandler(self)
        self.server = xmlrpc.client.ServerProxy('http://localhost:9000', allow_none=True)
        
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.server.get_angle(joint_name)
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.server.set_angle(joint_name, angle)

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.server.get_posture()

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        self.server.execute_keyframes(keyframes)

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        transform = self.server.get_transform(name)
        return self.deserialize_np(transform)

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.server.set_transform(effector_name, self.serialize_np(transform))

    def reset_joints(self):
        self.server.reset_joints()

    @staticmethod
    def serialize_np(array):
        return [[float(value) for value in array[i]] for i in range(len(array))] # Very simple serialization, convertion to python types, some things could go wrong, some precision is lost
    
    @staticmethod
    def deserialize_np(lists):
        return np.array(lists)

if __name__ == '__main__':
    agent = ClientAgent()
    # TEST CODE HERE
    time.sleep(2)

    # Non-blocking 1
    agent.post.execute_keyframes(hello())
    print("* Waving *")
    for _ in range(10):
        time.sleep(1)
        print("Hello!")

    # Angles
    print("Angle before:", agent.get_angle("HeadYaw"))
    agent.set_angle("HeadYaw", pi/2)
    print("Angle after:", agent.get_angle("HeadYaw"))

    # Non-blocking 2 (this may be slow, so the waiting time may need to be adjusted so that next request doesn't fail)
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = -0.26
    agent.post.set_transform("LLeg", T)
    for _ in range(20):
        time.sleep(1)
        print("Another non-blocking call test...")

    # Transforms
    print("HeadPitch transform:", agent.get_transform("HeadPitch"))
    print("LElbowRoll transform:", agent.get_transform("LElbowRoll"))
    print("RAnklePitch transform:", agent.get_transform("RAnklePitch"))
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = -0.26
    agent.set_transform("RLeg", T)
    print("RAnklePitch transform after movement:", agent.get_transform("RAnklePitch"))

    # Posture
    print(agent.get_posture())
    agent.reset_joints()
    #agent.execute_keyframes(rightBackToStand()) # Doesn't actually get back up, would have to fall completely flat for that
    # for _ in range(5):
    #     print(agent.get_posture())
    #     time.sleep(1)

    agent.execute_keyframes(hello())

    print("Hello World!\nMotion ended.")
    
