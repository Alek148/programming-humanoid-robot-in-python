'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello
from scipy import interpolate
from time import time


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.begin_motion()
    
    def begin_motion(self):
        self.time_begin = time()

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        if 'LHipYawPitch' in target_joints:
            target_joints['RHipYawPitch'] = target_joints['LHipYawPitch'] # copy missing joint in keyframes
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
        current_time = time() - self.time_begin
        names, times, keys = keyframes

        # Key dimensions: joint, key, value_type
        #print(len(keys), len(keys[0]), len(keys[0][0]))
        keyframe_angles = [[key[0] for key in joints] for joints in keys] # Ingore Bezier data
        
        #print(keyframe_angles[0])
        #print(times[0])

        target_angles = []
        for angle_list, time_list in zip(keyframe_angles, times):
            time_list = [0] + time_list + [time_list[-1] + 0.1] 
            angle_list = [angle_list[0]] + angle_list + [angle_list[-1]]
            
            angle = angle_list[-1]
            if current_time < time_list[-2]:
                # Simple cubic spline interpolation
                coefficients = interpolate.splrep(time_list, angle_list)
                angle = interpolate.splev(current_time, coefficients)
            
            target_angles.append(angle)

        return dict(zip(names, target_angles))

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
