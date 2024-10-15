import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from std_srvs.srv import Trigger, SetBool

from threading import Thread
import time


class LaserNode(Node):
    def __init__(self) -> None:
        super().__init__("laser", namespace="dexi")

        self.create_service(Trigger, "fire", self.fire_laser_callback)
        self.create_service(SetBool, "set_loop", self.set_loop_callback)

        self.pin_num = 21
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin_num, GPIO.OUT)
        self.turn_off()  # Turn the laser off initially

        self.loop = False
        self.loop_state = False

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
        if not self.loop:
            self.single_fire()
        else:
            self.loop_state = True
            self.start_loop()

        res.message = "Success"
        res.success = True
        return res

    def single_fire(self) -> None:
        self.loop_state = True
        self.start_loop()
        self.loop_state = False

    def start_loop(self) -> None:
        Thread(target=self.run_loop_thread).start()

    def run_loop_thread(self) -> None:
        while self.loop_state:
            self.turn_on(self)
            time.sleep(0.25)
            self.turn_off(self)

        self.get_logger().info("Laser loop ended")

    def turn_on(self) -> None:
        GPIO.output(self.pin_num, GPIO.HIGH)

    def turn_off(self) -> None:
        GPIO.output(self.pin_num, GPIO.LOW)


def main() -> None:
    rclpy.init()
    node = LaserNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    rclpy.spin(node, executor)
    GPIO.cleanup()


if __name__ == "__main__":
    main()
