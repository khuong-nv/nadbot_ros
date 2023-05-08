#!/usr/bin/python
import rospy
from std_msgs.msg import Float32, String, Int32
from dynamic_reconfigure.server import Server
import nadbot_ctrl.cfg.ParamConfig as ConfigType
class MotorRpm:
    def __init__(self):
        rospy.init_node('nadbot_motor_rpm')
        self.Kp = 0
        self.Ki = 0
        self.Kd = 0

        self.dyn_reconf_server = Server(ConfigType, self.reconfigure)
        self.rate = rospy.get_param('~rate', 40)


        # approx 5.34 rad / s when motor_cmd = 255
        self.motor_max_angular_vel = rospy.get_param('~motor_max_angular_vel', 7.5)
        self.motor_min_angular_vel = rospy.get_param('~motor_min_angular_vel', 2.5)

        self.wheel_radius = rospy.get_param('~wheel_radius', 0.045)

        # Corresponding motor commands
        self.motor_cmd_max = rospy.get_param('~motor_cmd_max', 255)
        self.motor_cmd_min = rospy.get_param('~motor_cmd_min', 80)

        # Publish motor command
        self.wheel_motor_cmd_pub = rospy.Publisher('wheel_motor_cmd_pub', Int32, queue_size=10)

        # Read in encoders for PID control
        self.wheel_angular_vel_enc_sub = rospy.Subscriber(
            'wheel_angular_vel_enc',
            Float32,
            self.wheel_angular_vel_enc_callback)
        # Read in tangential velocity targets
        self.wheel_tangent_vel_target_sub = rospy.Subscriber(
            'wheel_tangent_vel_target',
            Float32,
            self.wheel_tangent_vel_target_callback)

        # Tangential velocity target
        self.wheel_tangent_vel_target = 0

        # Angular velocity target
        self.wheel_angular_vel_target = 0

        # Angular velocity encoder readings
        self.wheel_angular_vel_enc = 0

        # Value motor command
        self.wheel_motor_cmd = 0

        # PID control variables
        self.wheel_pid = {}

    # ==================================================
    # Read in encoder readings for PID
    # ==================================================
    def wheel_angular_vel_enc_callback(self, msg):
        self.wheel_angular_vel_enc = msg.data


    # ==================================================
    # Read in tangential velocity targets
    # ==================================================
    def wheel_tangent_vel_target_callback(self, msg):
        self.wheel_tangent_vel_target = msg.data


    # ==================================================
    # Update motor commands
    # ==================================================

    # Compute angular velocity target
    def tangentVelToAngularVel(self, tangent_vel):
        angular_vel = tangent_vel / self.wheel_radius;
        return angular_vel

    def pidControl(wheel_pid, target, state, wheel):
        # Initialize pid dictionary
        if len(wheel_pid) == 0:
            wheel_pid.update({'time_prev': rospy.Time.now(), 'derivative': 0, 'integral': [0] * 10, 'error_prev': 0,
                              'error_curr': 0})

        wheel_pid['time_curr'] = rospy.Time.now()
        # PID control
        wheel_pid['dt'] = (wheel_pid['time_curr'] - wheel_pid['time_prev']).to_sec()
        if wheel_pid['dt'] == 0: return 0
        wheel_pid['error_curr'] = target - state
        wheel_pid['integral'] = wheel_pid['integral'][1:] + [(wheel_pid['error_curr'] * wheel_pid['dt'])]
        wheel_pid['derivative'] = (wheel_pid['error_curr'] - wheel_pid['error_prev']) / wheel_pid['dt']
        wheel_pid['error_prev'] = wheel_pid['error_curr']
        control_signal = (self.Kp * wheel_pid['error_curr'] + self.Ki * sum(wheel_pid['integral']) + self.Kd * wheel_pid['derivative'])
        # print(wheel_pid['error_curr'])
        target_new = target + control_signal
        #if target > 0 and target_new < 0: target_new = target
        #if target < 0 and target_new > 0: target_new = target

        if (target == 0):  # Not moving
            target_new = 0
	    wheel_pid['time_prev'] = wheel_pid['time_curr']	
            return target_new

        wheel_pid['time_prev'] = wheel_pid['time_curr']
        return target_new

    # Mapping angular velocity targets to motor commands
    # Note: motor commands are ints between 0 - 255
    # We also assume motor commands are issues between motor_min_angular_vel and motor_max_angular_vel
    def angularVelToMotorCmd(self, angular_vel_target):
        if angular_vel_target == 0: return 0
        slope = (self.motor_cmd_max - self.motor_cmd_min) / (self.motor_max_angular_vel - self.motor_min_angular_vel)
        intercept = self.motor_cmd_max - slope * self.motor_max_angular_vel

        if angular_vel_target > 0:  # positive angular velocity
            motor_cmd = slope * angular_vel_target + intercept
            if motor_cmd > self.motor_cmd_max: motor_cmd = self.motor_cmd_max
            if motor_cmd < self.motor_cmd_min: motor_cmd = self.motor_cmd_min
        else:  # negative angular velocity
            motor_cmd = slope * abs(angular_vel_target) + intercept
            if motor_cmd > self.motor_cmd_max: motor_cmd = self.motor_cmd_max
            if motor_cmd < self.motor_cmd_min: motor_cmd = self.motor_cmd_min
            motor_cmd = -motor_cmd

        return motor_cmd

    def wheelUpdate(self):
        # Compute target angular velocity
        self.wheel_angular_vel_target = self.tangentVelToAngularVel(self.wheel_tangent_vel_target)
        self.wheel_angular_vel_target = self.pidControl(self.wheel_pid, self.wheel_angular_vel_target,
                                                         self.wheel_angular_vel_enc, 'left')

        # Compute motor command
        wheel_motor_cmd = self.angularVelToMotorCmd(self.wheel_angular_vel_target)
        self.wheel_motor_cmd_pub.publish(wheel_motor_cmd)

    def reconfigure(self, param_data, level):
        self.Kp = param_data['kp']
        self.Ki = param_data['ki']
        self.Kd = param_data['kd']
        rospy.loginfo("Gains pid changed![{} {} {}".format(self.Kp, self.Ki, self.Kd))
        return param_data

    def spin(self):
        rospy.loginfo("Start nadbot_motor_rpm")
        rate = rospy.Rate(self.rate)

        rospy.on_shutdown(self.shutdown)
        while not rospy.is_shutdown():
            self.wheelUpdate()
            rate.sleep()
        rospy.spin()

    def shutdown(self):
        rospy.loginfo("Stop nadbot_motor_rpm")
        self.wheel_motor_cmd_pub.publish(0);
        rospy.sleep(0.5)


def main():
    motor_rpm = MotorRpm();
    motor_rpm.spin()


if __name__ == '__main__':
    main()
