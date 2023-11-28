'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import io
import os
import sys
import json
import threading
from time import time
import numpy as np
from xmlrpc.server import SimpleXMLRPCServer
from xmlrpc.server import SimpleXMLRPCRequestHandler
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

class RequestHandler(SimpleXMLRPCRequestHandler):
   rpc_paths = ('/RPC2',)

from inverse_kinematics import InverseKinematicsAgent


class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    def init_server(self):
        self.server = SimpleXMLRPCServer(('localhost', 9000), requestHandler=RequestHandler, allow_none=True)
        
        self.server.register_introspection_functions()
        
        self.server.register_function(self.get_angle)
        self.server.register_function(self.set_angle)
        self.server.register_function(self.get_posture)
        self.server.register_function(self.execute_keyframes)
        self.server.register_function(self.get_transform)
        self.server.register_function(self.set_transform)

        self.server.register_function(self.reset_joints)
        
        self.server.timeout = 0
        self.server_thread = threading.Thread(target=self.serve).start()

    def serve(self):
        self.server.serve_forever()

    def close_server(self):
        self.server.server_close()

    def think(self, perception):
        #self.server.handle_request() # Another way of handling incoming packets, but without it would be really hard to implement blocking this way without halting the whole simulation / robot behaviour
        return super(InverseKinematicsAgent, self).think(perception)
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        return self.perception.joint[joint_name]
        # YOUR CODE HERE
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        self.keyframes = ([], [], []) # For it to have any effect
        self.target_joints[joint_name] = angle
        # YOUR CODE HERE

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.recognize_posture(None) # The argument is actually not used

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        self.keyframes = keyframes
        self.begin_motion()

        time_max = self.get_max_time(self.keyframes)
        while(time() - self.time_begin < time_max):
            pass # Blocking

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        self.forward_kinematics(self.perception.joint)
        return self.serialize_np( np.array(self.transforms[name]) )

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        transform = self.deserialize_np(transform)
        self.set_transforms(effector_name, transform)

        time_max = self.get_max_time(self.keyframes)
        while(time() - self.time_begin < time_max):
            pass # Blocking

    def reset_joints(self):
        joints = dict(self.perception.joint)
        times = [[0.1, 2.0] for _ in range(len(joints))] # time = 0.0 is added automatically
        keys = [[[joints[joint], [], []], [0.0, [], []]] for joint in joints]
        self.keyframes = (list(joints.keys()), times, keys) # the result joint angles have to fill in
        self.begin_motion()

        # self.keyframes = ([], [], [])
        # for joint_name in self.target_joints.keys():
        #     self.target_joints[joint_name] = 0

        time_max = self.get_max_time(self.keyframes) + 5
        while(time() - self.time_begin < time_max):
            pass # Blocking

        self.keyframes = ([], [], [])

    @staticmethod
    def serialize_np(array):
        return [[float(value) for value in array[i]] for i in range(len(array))] # Very simple serialization, convertion to python types, some things could go wrong, some precision is lost
    
    @staticmethod
    def deserialize_np(lists):
        return np.array(lists)

    @staticmethod
    def get_max_time(keyframes):
        return max([time[-1] for time in keyframes[1]])

if __name__ == '__main__':
    agent = ServerAgent()
    try:
        agent.init_server()
        #agent.execute_keyframes((0, [[0, 3],[0, 30],[2, 4]], 0)) # Blocking test
        agent.run()
    except Exception:
        pass
    finally:
        agent.close_server()

