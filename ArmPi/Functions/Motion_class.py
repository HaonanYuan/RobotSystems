import ArmIK.Transform as AT
import HiwonderSDK.Board as Board
from ArmIK.ArmMoveIK import *
import time


class motion:

    def __init__(self):
        self.AK = ArmIK()
        self.servo = 500
        self.servo2 = 500

    def gripper_controller(self, angle=0):
        Board.setBusServoPulse(1, self.servo + angle, 500)  # open the gripper
        time.sleep(1)

    def palm_controller(self, world_X=None, world_Y=None, rotation_angle=None):
        if world_X is None and world_Y is None and rotation_angle is None:
            Board.setBusServoPulse(2, self.servo2, 500)
        else:
            # calculate the angle that gripper need to rotate
            servo2_angle = AT.getAngle(world_X, world_Y, rotation_angle)
            Board.setBusServoPulse(2, servo2_angle, 500)
        time.sleep(1)

    def arm_controller(self, world_X, world_Y, distance, alpha, alpha1, alpha2, movetime=None):
        result = self.AK.setPitchRangeMoving((world_X, world_Y, distance), alpha, alpha1, alpha2, movetime)  # raise hand
        time.sleep(result[2] / 1000)
        return result


if __name__ == '__main__':
    pass
