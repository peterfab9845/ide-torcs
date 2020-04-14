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
        self.old_dist_from_desired = 0
        self.speed_err_int = 0
        self.old_speed_err = 0
        self.old_max_distances_from_edge = collections.deque([0] * 40, 40)
        self.old_edge_indices = collections.deque([9] * 20, 20)
        self.old_sensor_area = collections.deque([0] * 20, 20)

        # Shifting Parameters
        self.rpm_max = 8500
        self.rpm_min = 4500
        self.gear_max = 6
        self.gear_change_d = 0
        self.gear_last = 0

        # Graph Parameters
        self.steer_graph = Graph(title="Steering", labels=("Overall", "Location", "dLocation", "Angle"),
                                 ymin=-1, ymax=1, xmax=700, time=True)
        self.cam_graph = Graph(title="Sensors", ymin=0, ymax=200)
        self.cam_graph2 = Graph(title="Edge Position", ymin=0, ymax=18, xmax=700, time=True)
        self.speed_graph = Graph(title="Speed (km/h)", ymin=-10, ymax=300, xmax=700, time=True, labels=("Actual", "Desired"))
        self.accel_graph = Graph(title="Accelerator Control", ymin=-0.4, ymax=1.1, xmax=700, time=True, labels=("Overall", "P", "I", "Brake"))
        self.area_graph = Graph(title="Sensor Area", ymin=0, ymax=1000, xmax=700, time=True)

    def drive(self, sensor: Sensor) -> Actuator:
        """ Produces a set of Actuator commands in response to Sensor data from
            the server .

            Args:
                sensor  - A set of Sensor values received from the server

            Returns: An Actuator populated with commands to send to the server
        """

        command = Actuator()

        # Determine the shape of the track
        # edge detection
        edge_index = sensor.distances_from_edge.index(max(sensor.distances_from_edge))
        edge_pos = edge_index
        #for i, v in enumerate(self.old_edge_indices):
        #    edge_pos += 0.5*v
        #edge_pos /= 1 + 0.5*len(self.old_edge_indices)
        desired_distance_from_center = 0.5 * (9 - edge_pos)
        distance_from_desired = sensor.distance_from_center - desired_distance_from_center


        """ Steering Control """

        # want to flip the sign for steering command
        # .5 is good because its aggressive enough to make then turn, but doesnt throw off the car for future turns
        Kp_steer = -.3

        # want the angle to also be aggressive so we can correct ourselves, but we want this Kp term to still be the
        # dominant factor in steering
        Ka_steer = 3

        # derivative term starts out really tiny, so we need a big multiplier (also negative)
        Kd_steer = 0.1

        dist = Kp_steer*(distance_from_desired)
        angle = (sensor.angle/180) * Ka_steer
        dist_derv = (distance_from_desired - self.old_dist_from_desired) * Kd_steer

        command.steering = dist + angle + dist_derv

        """ Accelerator Control """

        # control speed based on how curved the track is
        
        speed_des = 60 * MPS_PER_KMH
        speed_des += 0.2 * (max(sensor.distances_from_edge) + sum(self.old_max_distances_from_edge))/41

        #speed_des = 200 * MPS_PER_KMH * (sum(sensor.distances_from_edge) + sum(self.old_sensor_area))/(1 + len(self.old_sensor_area))/500

        #if max(sensor.distances_from_edge) < 60:
        #    speed_des = 60 * MPS_PER_KMH

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
            command.brake = -command.accelerator - 0.5

        # don't try to zoom if off the track
        if abs(sensor.distance_from_center) > 0.95 or abs(sensor.angle) > 90:
            command.accelerator = 0.2

        command.accelerator = min(max(command.accelerator, 0), 1)
        command.brake = min(max(command.brake, 0), 1)

        """ Graphs and misc """

        # Select the gear
        command.gear = self.select_gear(sensor, command)

        # Update "old" values
        self.old_speed_err = speed_err
        self.old_dist_from_desired = distance_from_desired
        self.old_max_distances_from_edge.append(max(sensor.distances_from_edge))
        self.old_edge_indices.append(edge_index)
        self.old_sensor_area.append(sum(sensor.distances_from_edge))

        # Plot the sensor and control data
        self.cam_graph.add(sensor.distances_from_edge)
        self.cam_graph2.add(edge_pos)
        self.steer_graph.add([command.steering, dist, dist_derv, angle])
        self.speed_graph.add([sensor.speed_x / MPS_PER_KMH, speed_des / MPS_PER_KMH])
        self.accel_graph.add([command.accelerator, accel_p, accel_i, command.brake])
        self.area_graph.add(sum(sensor.distances_from_edge))

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
        elif rpm < self.rpm_min:
            self.gear_last = gear
            gear = gear - 1
            self.gear_change_d = sensor.distance_raced

        if gear < 1:
            gear = 1
        elif gear > self.gear_max:
            gear = self.gear_max

        return gear
