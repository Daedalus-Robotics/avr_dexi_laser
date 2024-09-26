import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from std_srvs.msg import Trigger, SetBool

from threading import Thread, Event
import time


class LaserNode(Node):
    def __init__(self) -> None:
        super().__init__("laser", namespace="dexi")

        self.create_service(Trigger, "fire", self.fire_laser_callback)
        self.create_service(SetBool, "set_loop", self.set_loop_callback)

        self.pin_num = 2
        GPIO.setmode(GPIO.BCM)
        self.turn_off()  # Turn the laser off initially

        self.loop = Event()
        Thread(target=self.loop_fire).start()

    def set_loop_callback(
        self, req: SetBool.Request, res: SetBool.Response
    ) -> SetBool.Response:
        if req.data:
            self.loop.set()
        else:
            self.loop.clear()

        res.message = "Success"
        res.success = True
        return res

    def fire_laser_callback(
        self, req: Trigger.Request, res: Trigger.Response
    ) -> Trigger.Response:
        if not self.loop.is_set():
            self.single_fire()

        res.message = "Success"
        res.success = True
        return res

    def loop_fire(self) -> None:
        while True:
            self.single_fire()
            self.loop.wait()

    def single_fire(self) -> None:
        self.turn_on(self)
        time.sleep(0.25)
        self.turn_off(self)

    def turn_on(self) -> None:
        GPIO.output(self.pin_num, GPIO.HIGH)

    def turn_off(self) -> None:
        GPIO.output(self.pin_num, GPIO.LOW)


def main() -> None:
    rclpy.init()
    node = LaserNode()
    executer = rclpy.executers.MultiThreadedExecuter()
    rclpy.spin(node, executer)


if __name__ == "__main__":
    main()
