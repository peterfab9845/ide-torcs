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
import numpy as np
import time

""" Change this to 1 to use the safer driving mode """
SAFE_CAR = 0

MPS_PER_KMH = 1000 / 3600
WHEEL_RADIUS_M = 0.3276 # meters
# calculated from rear wheel parameters in share/games/torcs/cars/car1-trb1/car1-trb1.xml
WHEEL_CIRCUM_M = 2 * np.pi * WHEEL_RADIUS_M # meters

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
        self.center_dist_int = 0
        self.old_steer = 0
        self.old_speed_des = 300
        self.old_accel = 1
        self.speed_err_int = 0
        self.old_speed_err = 0
        self.forward_distances = collections.deque([0] * 7, 7)
        self.edge_indices = collections.deque([9] * 25, 25)

        # Shifting Parameters
        self.rpm_max = 8500
        self.rpm_min = 4500
        self.gear_max = 6
        self.gear_change_d = 0
        self.gear_last = 0

        # Graph Parameters
        self.steer_graph = Graph(title="Steering", labels=("Overall", "Location", "iLocation", "dLocation", "Angle"),
                                 ymin=-1, ymax=1, xmax=700, time=True)
        self.cam_graph = Graph(title="Sensors", ymin=0, ymax=200)
        self.edge_graph = Graph(title="Edge Location", ymin=0, ymax=18, xmax=700, time=True)
        self.speed_graph = Graph(title="Speed (km/h)", ymin=-10, ymax=300, xmax=700, time=True,
                                 labels=("Actual", "Desired"))
        self.accel_graph = Graph(title="Accelerator Control", ymin=-0.4, ymax=1.1, xmax=700, time=True,
                                 labels=("Overall", "P", "I", "Brake"))

    def drive(self, sensor: Sensor) -> Actuator:
        """ Produces a set of Actuator commands in response to Sensor data from
            the server .

            Args:
                sensor  - A set of Sensor values received from the server

            Returns: An Actuator populated with commands to send to the server
        """

        # record times to see if we're taking too long
        start_time = time.time()

        command = Actuator()

        # Using the clutch helps at the start of the race (added in car.py)
        if sensor.distance_raced < 20:
            command.clutch = max(0.8 - 0.2*sensor.current_lap_time, 0)
        else:
            command.clutch = 0

        """ Steering Control """

        # Look for where the inside edge of the track is (at the furthest distance)
        if sensor.distances_from_edge[0] == -1:
            self.edge_indices.append(9)
        else:
            edge_index = sensor.distances_from_edge.index(max(sensor.distances_from_edge))
            # average because it jumps around a lot
            self.edge_indices.append(edge_index)
        edge_pos = np.mean(self.edge_indices)
        desired_distance_from_center = 0.5 * (9 - edge_pos)

        distance_from_desired = sensor.distance_from_center - desired_distance_from_center

        # want to flip the sign for steering command
        # this is good because its aggressive enough to make then turn, but doesnt throw off the car for future turns
        Kp_steer = -.4

        # also use an integral term to stay away from the edge on turns
        Ki_steer = -.015

        # derivative term starts out really tiny, so we need a big multiplier (also negative)
        Kd_steer = -6

        # want the angle to also be aggressive so we can correct ourselves, but we want this Kp term to still be the
        # dominant factor in steering
        Ka_steer = 6

        self.center_dist_int = min(max(self.center_dist_int, -10), 10)
        if sensor.distance_raced > 30:
            self.center_dist_int += distance_from_desired
        if distance_from_desired * self.old_dist_from_center < 0:
            self.center_dist_int = 0
        if abs(distance_from_desired) < 0.05:
            self.center_dist_int = 0

        dist = Kp_steer*(distance_from_desired)
        dist_int = Ki_steer*(self.center_dist_int)
        dist_derv = Kd_steer*(distance_from_desired - self.old_dist_from_center)
        angle = (sensor.angle/180) * Ka_steer

        command.steering = dist + dist_int + dist_derv + angle

        # don't turn as much when we're going fast
        if sensor.speed_x > 1: # avoid div by 0
            steer_speed_coeff = 8 / sensor.speed_x
            if steer_speed_coeff < 1:
                command.steering *= steer_speed_coeff

        """ Accelerator Control """

        # control speed based on how curved the track is
        
        middle_distances = (1*sensor.distances_from_edge[7]
                          + 2*sensor.distances_from_edge[8]
                          + 2*sensor.distances_from_edge[9]
                          + 2*sensor.distances_from_edge[10]
                          + 1*sensor.distances_from_edge[11])/8
        self.forward_distances.append(middle_distances)
        average_forward_distance = np.mean(self.forward_distances)
        if SAFE_CAR:
            speed_des = 1 * MPS_PER_KMH * average_forward_distance
        else:
            speed_des = 3 * MPS_PER_KMH * average_forward_distance

        # make desired speed not jump up quickly - match the car's real acceleration
        # not perfect for higher gears, but they aren't affected as much
        if (speed_des - self.old_speed_des) > 0.15:
            speed_des = self.old_speed_des + 0.15

        # don't try to zoom if off the track
        if abs(sensor.distance_from_center) > 0.95 or abs(sensor.angle) > 30:
            speed_des = 30 * MPS_PER_KMH

        # PI control
        speed_wheels = (sensor.wheel_velocities[2] + sensor.wheel_velocities[3])/2
        speed_wheels = (speed_wheels / 360) * WHEEL_CIRCUM_M
        speed_average = (speed_wheels + sensor.speed_x) / 2
        speed_err = speed_des - speed_average

        # P
        Kp_accel = 0.3
        accel_p = Kp_accel * speed_err

        # I
        self.speed_err_int = min(max(self.speed_err_int, -50), 50)
        self.speed_err_int += speed_err

        Ki_accel = 0.02
        accel_i = Ki_accel * (self.speed_err_int)

        # combine terms
        command.accelerator = accel_p + accel_i

        # brake if we want to slow down fast
        if command.accelerator + 0.35 < 0:
            command.brake = 0.5*(-command.accelerator - 0.35)

        # avoid slipping and braking constantly when off the track
        if abs(sensor.distance_from_center) > 0.95 or abs(sensor.angle) > 90:
            command.accelerator *= 0.2
            command.brake = 0

        command.accelerator = min(max(command.accelerator, 0), 1)
        command.brake = min(max(command.brake, 0), 1)

        """ Graphs and misc """

        # Select the gear
        command.gear = self.select_gear(sensor, command)

        # Update "old" values
        self.old_speed_err = speed_err
        self.old_speed_des = speed_des
        self.old_accel = command.accelerator
        self.old_steer = command.steering
        #self.old_dist_from_center = sensor.distance_from_center
        self.old_dist_from_center = distance_from_desired

        # Plot the sensor and control data
        self.cam_graph.add(sensor.distances_from_edge)
        self.edge_graph.add(edge_pos)
        self.steer_graph.add([command.steering, dist, dist_int, dist_derv, angle])
        self.speed_graph.add([speed_average / MPS_PER_KMH, speed_des / MPS_PER_KMH])
        self.accel_graph.add([command.accelerator, accel_p, accel_i, command.brake])

        # see how long we're taking
        exec_time = time.time() - start_time
        if exec_time > 0.01:
            print("Too slow for 1x!")
        elif exec_time > 0.005:
            print("Too slow for 2x!")
        elif exec_time > 0.0025:
            print("Too slow for 4x!")
        elif exec_time > 0.00125:
            print("Too slow for 8x!")
        elif exec_time > 0.000625:
            print("Too slow for 16x!")
        elif exec_time > 0.0003125:
            print("Too slow for 32x!")
        #elif exec_time > 0.00015625:
        #    print("Too slow for 64x!")

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

        if d_since_shift < 10:
            # Don't shift if we just did.
            # Should probably be time based, but distance is easier
            pass
        elif rpm > self.rpm_max:
            # Hitting redline, we should shift up
            self.gear_last = gear
            gear = gear + 1
            self.gear_change_d = sensor.distance_raced
            command.accelerator = 0 # this happens anyway, but we need to keep track of it
        elif rpm < self.rpm_min:
            self.gear_last = gear
            gear = gear - 1
            self.gear_change_d = sensor.distance_raced
            command.accelerator = 0 # this happens anyway, but we need to keep track of it

        if gear < 1:
            gear = 1
        elif gear > self.gear_max:
            gear = self.gear_max

        return gear
