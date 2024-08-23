from robot_command.rpl import notify
from hal_utils import getTorques
from time import sleep

#FILE:torque_comparator.py

class JointTorqueComparator:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super(JointTorqueComparator, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self, torque_thresh):
        if self._initialized:
            return
        self._initialized = True

        self.thresh = torque_thresh
        self.torques_before = []

    def measure_relax_torque(self):
        sleep(1.0)
        self.torques_before = getTorques()

    def is_torque_in_bounds(self):
        sleep(0.5)
        torques_after = getTorques()
        torque_exceeded = False
        for t1, t2 in zip(self.torques_before, torques_after):
            if abs(t1 - t2) > self.thresh:
                torque_exceeded = True
        if torque_exceeded:
            # TODO: move robot up along z-axis
            # notify("Collision with workpiece.", error=True)
            return False
        return True

    @classmethod
    def get_instance(cls, torque_thresh=None):
        if cls._instance is None:
            if torque_thresh is None:
                raise ValueError("torque_thresh must be provided when creating the first instance")
            cls(torque_thresh)
        return cls._instance

# Usage example
torque_thresh = 20
joint_torque_comparator = JointTorqueComparator.get_instance(torque_thresh)
