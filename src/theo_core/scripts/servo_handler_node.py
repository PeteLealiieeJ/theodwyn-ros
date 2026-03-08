#!/usr/bin/python3
import rclpy
from rclpy.node import Node, Subscription
from math import degrees
import board

from theo_msgs.msg import TheoServoAddition
from theo_core.networks.pca9685 import PCA9685

class ServoHandlerNode( Node ):

    servo_driver : PCA9685
    subscriber_  : Subscription
    angle_targets: list[float,float]
    

    def __init__( self ):

        super().__init__( "servo_handler_node" )
        
        self.declare_parameter( 'address',              0x40            )
        self.declare_parameter( 'channels',             16              )
        self.declare_parameter( 'pan_init_cmd',         0               )
        self.declare_parameter( 'tilt_init_cmd',        0               )
        self.declare_parameter( 'pan_actuation_range',  270      	    )
        self.declare_parameter( 'tilt_actuation_range', 180       	    )
        self.declare_parameter( 'pan_min_max_pwr',      [ 500, 2500 ]   )
        self.declare_parameter( 'tilt_min_max_pwr',     [ 500, 2500 ]   )
        self.declare_parameter( 'pan_safety_bounds',    [ 0, 270 ]      )
        self.declare_parameter( 'tilt_safety_bounds',   [ 0, 180 ]      )
        
        address         = self.get_parameter('address').value
        channels        = self.get_parameter('channels').value
        init_cmd        = [
            self.get_parameter('pan_init_cmd').value,
            self.get_parameter('tilt_init_cmd').value
        ]
        actuation_range = [
            self.get_parameter('pan_actuation_range').value,
            self.get_parameter('tilt_actuation_range').value
        ]
        min_max_pwr     = [
            self.get_parameter('pan_min_max_pwr').value,
            self.get_parameter('tilt_min_max_pwr').value
        ]
        safety_bounds = [
            self.get_parameter('pan_safety_bounds').value,
            self.get_parameter('tilt_safety_bounds').value
        ]

        self.servo_driver = PCA9685(
            SDA             = board.SDA_1,
            SCL             = board.SCL_1,
            channels        = channels, 
            active_channels = 2,
            address         = address,
            init_cmd        = init_cmd,
            actuation_range = actuation_range,
            min_max_pwr     = min_max_pwr,
            safety_bounds   = safety_bounds,
        )


        self.subscriber_ = self.create_subscription(
            TheoServoAddition,
            'servoadd_topic',
            self.additionCallback,
            10
        )
        self.subscriber_
        
        
    def update_after_connection(self):
        self.angle_targets = self.servo_driver.get_angles()


    def additionCallback( self, msg ):
        self.angle_targets[0] = self.angle_targets[0] + degrees( msg.pan_angle )
        self.angle_targets[1] = self.angle_targets[1] + degrees( msg.tilt_angle )
        self.angle_targets    = self.servo_driver.send( self.angle_targets )




def main(args=None):
    
    rclpy.init(args=args)
    node = ServoHandlerNode()

    try:
        node.servo_driver.connect()
    except Exception as e:
        # import traceback
        # node.get_logger().error( f"Servo Driver raised the following error: {traceback.format_exc()}" )
        node.get_logger().error( "Failed to connect to PCA9685. Exiting." )
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
            return
    
    try:
        node.update_after_connection()
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass
    finally:
        node.servo_driver.disconnect()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()



if __name__ == '__main__':
    main()
