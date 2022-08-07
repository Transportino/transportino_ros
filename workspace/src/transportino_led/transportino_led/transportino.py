from transportino_interfaces.srv import StatusLedCtrl

import os

import rclpy
from rclpy.node import Node

from gpiozero import RGBLED

from enum import Enum

class LedMode(Enum):
    always_on = 0
    blink = 1
    pulse = 2

class LedService(Node):
    
    led: RGBLED = None

    def __init__(self, r : int, g : int, b : int) -> None:
        super().__init__(node_name='led_ctrl_node', namespace='transportino')
        self.led = RGBLED(r, g, b)
        self.srv = self.create_service(StatusLedCtrl, 'led_ctrl', self.led_ctrl_callback)
    
    def led_ctrl_callback(self, request, response):
        self.get_logger().info("led status changing.")
        self.get_logger().info("mode: %d, color: (%0.2f, %0.2f, %0.2f, %0.2f), on_time: %0.2f, off_time: %0.2f" 
            % (
                request.mode, 
                request.color.r, request.color.g, request.color.b, request.color.a,
                request.on_time, request.off_time
            ) 
        )
        
        alpha = request.color.a
        color = (request.color.r * alpha, request.color.g * alpha, request.color.b * alpha)
        
        mode = LedMode(request.mode)

        if mode == LedMode.always_on:
            self.led.color = color
        elif mode == LedMode.blink:
            self.led.blink(on_time=request.on_time, off_time=request.off_time, on_color=color)
        elif mode == LedMode.pulse:
            self.led.pulse(fade_in_time=request.on_time, fade_out_time=request.off_time, on_color=color)


        response.success = True

        return response
       


def main():
    print("Starting status led controller...")
    rclpy.init()

    led_r = int(os.getenv('TRANSPORTINO_LED_R', '26'))
    led_g = int(os.getenv('TRANSPORTINO_LED_G', '19'))
    led_b = int(os.getenv('TRANSPORTINO_LED_B', '13'))
    

    led_service = LedService(led_r, led_g, led_b)

    while rclpy.ok():
        try:
            rclpy.spin_once(led_service)
        except KeyboardInterrupt:
            break
    
    led_service.led.off()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
