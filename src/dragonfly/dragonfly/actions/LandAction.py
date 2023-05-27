#!/usr/bin/env python3
from mavros_msgs.srv import CommandTOL
from std_msgs.msg import String

from .ActionState import ActionState

class LandAction:

    def __init__(self, logger, log_publisher, land_service):
        self.logger = logger
        self.log_publisher = log_publisher
        self.land_service = land_service

    def step(self):
        self.logger.info("Land action")
        result = self.land_service.call(CommandTOL.Request(altitude=0.0))

        if result.success:
            self.log_publisher.publish(String(data="Landing..."))
        else:
            self.log_publisher.publish(String(data="Landing failed"))

        return ActionState.mapSuccess(result.success)

    def stop(self):
        pass
