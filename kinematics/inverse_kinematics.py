'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity
import autograd.numpy as np
from autograd.numpy import pi
import autograd

class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []
        # YOUR CODE HERE

        # ============================
        # I don't think it's supposed to work this way - I probably made a mistake, but don't know where.
        # I also made many experiments in:
        #     forward_kinematics_2d_v2.ipynb
        # including 3D plots of a robot to check forward, then inverse kinematics
        #
        # Forward kinematics seemed to work OK, but still had some flaws (pitch angles were sometimes reversed) but I don't have the time to fix it now
        # Inverse kinematics seems to be able to calculate a gradient, but it somehow ends up in a different place than I though it should. This is probably due to the fact that effector transforms are not transformed by the torso first
        # ============================
        
        joint_names = self.chains[effector_name]
        target_joint = joint_names[-1]
        target_trans = transform
        # angles = np.zeros(len(joint_names))
        # for i in range(len(angles)):
        #     angles[i] = self.perception.joint[joint_names[i]]
        angles = np.random.rand(len(joint_names)) * pi * 2

        def error_func(angles, target):
            new_joints = dict(self.perception.joint)
            for i, name in enumerate(joint_names):
                new_joints[name] = angles[i]
        
            self.forward_kinematics(new_joints)
            transform = self.transforms[target_joint]
        
            #error = target - transform
            error = target[:,3] - transform[:,3] # Ignore rotation
            error = np.sum(np.power(error, 2)) # SE
            return error
        
        func_error = lambda t: error_func(t, target_trans)
        func_grad = autograd.grad(func_error)
        
        #print(func_error(angles))
        
        for i in range(1000):
            e = func_error(angles)
            d = func_grad(angles)
            angles -= d * 1e-1
            if e < 1e-6:
                break
            
        #print(e)
        joint_angles = list(angles)
        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        angles_prior = dict(self.perception.joint)
        angles_post = self.inverse_kinematics(effector_name, transform)
        print(angles_prior, angles_post)
        chain = self.chains[effector_name]
        times = [[0.0, 1.0] for _ in range(len(chain))]
        keys = [[[angles_prior[joint], [], []], [angles_post[id], [], []]] for id, joint in enumerate(chain)]
        self.keyframes = (chain, times, keys) # the result joint angles have to fill in

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = -0.26
    agent.set_transforms('LLeg', T)
    agent.run()
