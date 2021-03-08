#!/usr/bin/env python
import rospy

from .ActionState import ActionState


class StopInPlaceAction:

    def __init__(self, id, log_publisher, local_setposition_publisher):
        self.id = id
        self.log_publisher = log_publisher
        self.local_setposition_publisher = local_setposition_publisher
        self.status = ActionState.WORKING
        self.commanded = False
        self.position_update = None

    def step(self):
        if not self.commanded:
            self.commanded = True

            def updatePosition(localposition):
                self.local_setposition_publisher.publish(localposition)
                self.status = ActionState.SUCCESS

                self.stop()

                print("Stop in place")
                self.log_publisher.publish("Stopped")

            self.position_update = rospy.Subscriber("{}/mavros/local_position/pose".format(self.id), PoseStamped,
                                                    updatePosition)

        return self.status

    def stop(self):
        if self.position_update is not None:
            self.position_update.unregister()
            self.position_update = None
