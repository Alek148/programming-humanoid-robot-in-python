'''In this exercise you need to put all code together to make the robot be able to stand up by its own.

* Task:
    complete the `StandingUpAgent.standing_up` function, e.g. call keyframe motion corresponds to current posture

'''


from recognize_posture import PostureRecognitionAgent
from keyframes import hello, rightBackToStand, rightBellyToStand, wipe_forehead
from time import time

class StandingUpAgent(PostureRecognitionAgent):
    def think(self, perception):
        self.standing_up()
        return super(StandingUpAgent, self).think(perception)

    def standing_up(self):
        posture = self.posture
        print(posture)
        # YOUR CODE HERE


class TestStandingUpAgent(StandingUpAgent):
    '''this agent turns off all motor to falls down in fixed cycles
    '''
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(TestStandingUpAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.stiffness_on_off_time = 0
        self.stiffness_on_cycle = 10  # in seconds
        self.stiffness_off_cycle = 3  # in seconds
        self.last_posture = None

    def think(self, perception):
        action = super(TestStandingUpAgent, self).think(perception)
        time_now = perception.time
        if time_now - self.stiffness_on_off_time < self.stiffness_off_cycle:
            action.stiffness = {j: 0 for j in self.joint_names}  # turn off joints
        else:
            action.stiffness = {j: 1 for j in self.joint_names}  # turn on joints
        if time_now - self.stiffness_on_off_time > self.stiffness_on_cycle + self.stiffness_off_cycle:
            self.stiffness_on_off_time = time_now

        if time() - self.time_begin >= 10: # I could make a proper test for keyframe animation ending, but this already works
            self.last_posture = None

        # Simple ~state machine
        if self.posture == "Back" and self.last_posture != "Back":
            self.last_posture = "Back"
            self.keyframes = rightBackToStand()
            self.begin_motion()
        elif self.posture == "Belly" and self.last_posture != "Belly":
            self.last_posture = "Belly"
            self.keyframes = rightBellyToStand()
            self.begin_motion()
        elif self.posture == "Stand" and self.last_posture != "Stand":
            self.last_posture = "Stand"
            self.keyframes = wipe_forehead(None)
            self.begin_motion()

        return action


if __name__ == '__main__':
    agent = TestStandingUpAgent()
    agent.run()
