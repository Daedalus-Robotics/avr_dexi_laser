import rclpy
from rclpy.node import Node

from std_srvs.msg import Trigger, SetBool


class LaserNode(Node):
    def __init__(self) -> None:
        super().__init__("laser", namespace="laser")

        self.create_service(Trigger, "fire", self.fire_laser_callback)
        self.create_service(SetBool, "set_loop", self.set_loop_callback)

        self.loop = False
        self.timer_period = 0.5
        self.looper = self.create_timer(
            self.actually_fire, self.timer_period, autostart=False
        )

    def set_loop_callback(
        self, req: SetBool.Request, res: SetBool.Response
    ) -> SetBool.Response:
        self.loop = req.data

        res.message = "Success"
        res.success = True
        return res

    def fire_laser_callback(
        self, req: Trigger.Request, res: Trigger.Response
    ) -> Trigger.Response:
        self.looper.reset()

        res.message = "Success"
        res.success = True
        return res

    def actually_fire(self) -> None:
        # TODO: Firing laser!

        if not self.loop:
            self.looper.cancel()


def main() -> None:
    rclpy.init()
    node = LaserNode()
    executer = rclpy.executers.MultiThreadedExecuter()
    rclpy.spin(node, executer)

 
if __name__ == "__main__":
    main()
