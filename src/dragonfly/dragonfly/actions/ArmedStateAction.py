#!/usr/bin/env python
import rclpy
from mavros_msgs.msg import State

from .ActionQueue import ActionQueue
from .ActionState import ActionState
from std_msgs.msg import String


class ArmedStateAction:

    def __init__(self, log_publisher, id, node):
        self.node = node
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

                if self.status is ActionState.WORKING:
                    if state.armed:
                        print("Is already armed, failed")
                        self.status = ActionState.FAILURE
                        self.log_publisher.publish(String(data="Arming failed, already armed"))
                    else:
                        print("Is not armed, continue")
                        self.status = ActionState.SUCCESS

                    self.stop()

            self.disabled_update = self.node.create_subscription(
                State, "{}/mavros/state".format(self.id), updateState, 10)

        return self.status

    def stop(self):
        print("I'm here")
        if self.disabled_update is not None:
            pass #self.disabled_update.destroy()
            #self.disabled_update = None
