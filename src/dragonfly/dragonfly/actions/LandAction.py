#!/usr/bin/env python
from .ActionState import ActionState
from mavros_msgs.srv import CommandTOL
from std_msgs.msg import String

class LandAction:

    def __init__(self, log_publisher, land_service):
        self.log_publisher = log_publisher
        self.land_service = land_service

    def step(self):
        print("Land off")
        result = self.land_service.call(CommandTOL.Request(altitude=0.0))

        print("Land result {}".format(result))

        if result.success:
            self.log_publisher.publish(String(data="Landing..."))
        else:
            self.log_publisher.publish(String(data="Landing failed"))

        return ActionState.mapSuccess(result.success)

    def stop(self):
        pass