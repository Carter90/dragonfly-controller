#!/usr/bin/env python
import rospy
from mavros_msgs.msg import State

from ActionQueue import ActionQueue
from .ActionState import ActionState


class ArmedStateAction:

    def __init__(self, log_publisher, id):
        self.log_publisher = log_publisher
        self.id = id
        self.armedQueue = ActionQueue()
        self.notarmedQueue = ActionQueue()
        self.status = ActionState.WORKING
        self.commanded = False
        self.disabled_update = None

    def step(self):
        if not self.commanded:
            print("Verifying disarmed...")
            self.commanded = True

            def updateState(state):

                if state.armed:
                    print("Is already armed, failed")
                    self.status = ActionState.FAILURE
                    self.log_publisher.publish("Arming failed, already armed")
                else:
                    print("Is not armed, continue")
                    self.status = ActionState.SUCCESS

                self.stop()

            self.disabled_update = rospy.Subscriber("{}/mavros/state".format(self.id), State, updateState)

        return self.status

    def stop(self):
        if self.disabled_update is not None:
            self.disabled_update.unregister()
            self.disabled_update = None
