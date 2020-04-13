""" driver.py 
    
    Author:     Carson Clarke-Magrab <ctc7359@rit.edu>
    Date:       03/23/2019
    Modified:
        03/29/2019 by Carson Clarke-Magrab <ctc7359@rit.edu>
        -   Adapted for IDE virtual NXP Cup.
"""
from client.car import Actuator, Sensor
from client.graph import Graph
import collections

MPS_PER_KMH = 1000 / 3600

class Driver:
    """ Car driving logic

        The Driver receives a set of Sensor data from the server and transmits
        a set of Actuator data in response. See car.py for more details on
        these structures.

        The drive() function is called approximately every 20ms and must return
        an Actuator object within 10ms.

        You may add new functions and modify this class as you see fit, but do not
        remove the drive() function or change the name of the class.
    """

    def __init__(self):
        """ If you need to initialize any variables, do it here and remove the
            'pass' statement.
        """

        # Previous sensor values
        self.old_dist_from_center = 0
        self.speed_err_int = 0
        self.old_speed_err = 0
        self.old_center_diffs = collections.deque([0] * 10, 10)

        # Shifting Parameters
        self.rpm_max = 8500
        self.rpm_min = 4500
        self.gear_max = 6
        self.gear_change_d = 0
        self.gear_last = 0

        # Graph Parameters
        self.steer_graph = Graph(labels=("Current Steering", "Distance from Center", "Angle from Track"),
                                 xmin=-1, xmax=1, title='Steering', hbar=True)
        self.cam_graph = Graph(title="Sensors", ymin=0, ymax=200)
        self.speed_graph = Graph(title="Speed (km/h)", ymin=-10, ymax=250, xmax=700, time=True, labels=("Actual", "Desired"))
        self.accel_graph = Graph(title="Accelerator Control", ymin=-0.4, ymax=1.1, xmax=700, time=True, labels=("Overall", "P", "I", "Brake"))

    def drive(self, sensor: Sensor) -> Actuator:
        """ Produces a set of Actuator commands in response to Sensor data from
            the server .

            Args:
                sensor  - A set of Sensor values received from the server

            Returns: An Actuator populated with commands to send to the server
        """

        command = Actuator()

        # Determine the shape of the track
        center_pos = 0
        for i, v in enumerate(sensor.distances_from_edge):
            center_pos += i*v
        center_pos /= sum(sensor.distances_from_edge)
        # adjust for angle of car
        angle_correction = 50*sensor.angle/180
        center_pos_corrected = center_pos + angle_correction
        center_diff = 9 - center_pos_corrected

        """ Steering Control """

        # want to flip the sign for steering command
        # .5 is good because its aggressive enough to make then turn, but doesnt throw off the car for future turns
        Kp_steer = -.5

        # want the angle to also be aggressive so we can correct ourselves, but we want this Kp term to still be the
        # dominant factor in steering
        Ka_steer = 4

        # derivative term starts out really tiny, so we need a big multiplier (also negative)
        Kd_steer = -25

        dist = Kp_steer*(sensor.distance_from_center)
        angle = (sensor.angle/180) * Ka_steer
        dist_derv = (sensor.distance_from_center - self.old_dist_from_center) * Kd_steer

        command.steering = dist + angle + dist_derv

        """ Accelerator Control """

        # control speed based on how curved the track is
        
        #speed_des = 150 * MPS_PER_KMH
        speed_des = 60 * MPS_PER_KMH

        if max(sensor.distances_from_edge) < 60:
            speed_des = 60 * MPS_PER_KMH

        # don't try to zoom if off the track
        if abs(sensor.distance_from_center) > 1 or abs(sensor.angle) > 90:
            speed_des = MPS_PER_KMH * 20

        # PI control
        speed_err = speed_des - sensor.speed_x

        # P
        Kp_accel = 0.8
        accel_p = Kp_accel * speed_err

        # I
        self.speed_err_int = min(max(self.speed_err_int, -50), 50)
        self.speed_err_int += speed_err

        Ki_accel = 0.02
        accel_i = Ki_accel * (self.speed_err_int)

        # combine terms
        command.accelerator = accel_p + accel_i

        if command.accelerator < 0:
            command.brake = -command.accelerator - 0.4

        command.accelerator = min(max(command.accelerator, 0), 1)
        command.brake = min(max(command.brake, 0), 1)

        """ Graphs and misc """

        # Select the gear
        command.gear = self.select_gear(sensor, command)

        # Update "old" values
        self.old_speed_err = speed_err
        self.old_center_diffs.append(center_diff)
        self.old_dist_from_center = sensor.distance_from_center

        # Plot the sensor and control data
        self.cam_graph.add(sensor.distances_from_edge)
        self.steer_graph.add([command.steering, sensor.distance_from_center, sensor.angle])
        self.speed_graph.add([sensor.speed_x / MPS_PER_KMH, speed_des / MPS_PER_KMH])
        self.accel_graph.add([command.accelerator, accel_p, accel_i, command.brake])

        return command

    def select_gear(self, sensor, command):
        """ Simple gear shifting algorithm. Feel free to adjust this as you see
            fit.

            Args:
                current_gear    - The current gear the car is in
                rpm             - The current RPM of the motor

            Returns: Which gear to shift to
        """
        rpm = sensor.rpm
        gear = sensor.gear
        d_since_shift = sensor.distance_raced - self.gear_change_d

        if abs(sensor.distance_from_center) > 1 or abs(sensor.angle) > 90:
            gear = 1
        elif d_since_shift < 10:
            # Don't shift if we just did.
            # Should probably be time based, but distance is easier
            pass
        elif rpm > self.rpm_max:
            # Hitting redline, we should shift up
            self.gear_last = gear
            gear = gear + 1
            self.gear_change_d = sensor.distance_raced
        elif rpm < self.rpm_min:
            self.gear_last = gear
            gear = gear - 1
            self.gear_change_d = sensor.distance_raced

        if gear < 1:
            gear = 1
        elif gear > self.gear_max:
            gear = self.gear_max

        return gear
