from pid import PID
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704



class Controller(object):
    def __init__(self, vehicle_mass, 
                    brake_deadband, 
                    decel_limit, 
                    accel_limit, 
                    wheel_radius, 
                    wheel_base, 
                    steer_ratio, 
                    max_lat_accel, 
                    max_steer_angle,
                    min_speed):

        self.vehicle_mass = vehicle_mass
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle
        self.min_speed = min_speed

        # controller parameters longitudinal controller
        P_long = 0.1
        I_long = 0.003
        D_long = 0.005
        # controller parameters lateral controller
        P_lat = 1
        I_lat = 0.01
        D_lat = 0.1


        self.__controller_longitudinal = PID(P_long, I_long, D_long)
        self.__controller_lateral = PID(P_lat, I_lat, D_lat)
        self.__yaw_controller = YawController(self.wheel_base, self.steer_ratio, self.min_speed, self.max_lat_accel, self.max_steer_angle)
        pass

    def control(self, current_vel, 
                    current_ang_vel, 
                    dbw_enabled, 
                    linear_vel, 
                    angular_vel):

        controller_command_long = 0
        controller_command_lat = 0
        throttle_command = 0
        brake_command = 10

        if dbw_enabled:
            #### Longitudinal control ####
            vel_controll_error = linear_vel - current_vel
            controller_command_long = self.__controller_longitudinal.step(vel_controll_error, 0.02)
            rospy.loginfo('Velocity setpoint: %f, Actual velocity: %f, Controller output: %f', linear_vel, current_vel, controller_command_long)

            if linear_vel == 0 and current_vel <= 0.05:
                brake_command = 700 # Apply 700 Nm torque to hold car in position if in standby
                throttle_command = 0
            elif controller_command_long <= 0.05 and vel_controll_error < 0:
            #elif vel_controll_error < 0:
                # Negative controller command == braking
                # Has to be converted to to a torque first
                brake_command = - max(vel_controll_error, self.decel_limit) * self.vehicle_mass * self.wheel_radius
                throttle_command = 0
            else:
                brake_command = 0
                throttle_command = controller_command_long

            #### Lateral control ####
            rospy.loginfo('Input yaw controller: l_vel = %f, a_vel = %f, c_vel = %f', linear_vel, angular_vel, current_vel)
            steering_angle_controll_error = self.__yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
            steering_angle_controll_error = self.__controller_lateral.step(steering_angle_controll_error, 0.02)
            controller_command_lat = steering_angle_controll_error

        else:
            self.__controller_longitudinal.reset()
    
        # Return throttle, brake, steer
        return throttle_command, brake_command, controller_command_lat
