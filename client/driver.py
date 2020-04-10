""" driver.py 
    
    Author:     Carson Clarke-Magrab <ctc7359@rit.edu>
    Date:       03/23/2019
    Modified:
        03/29/2019 by Carson Clarke-Magrab <ctc7359@rit.edu>
        -   Adapted for IDE virtual NXP Cup.
"""
from client.car import Actuator, Sensor
from client.graph import Graph

from matplotlib import pyplot
import collections

# Global variables
track_angle_turn = 0
offset_turn = 0
corner_turn = 0


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

        # Live Distance to Center
        pyplot.ion()
        self.fig = pyplot.figure()
        ax = self.fig.add_subplot(1, 1, 1)
        ax.set_ylim(-2,2)
        self.dists = collections.deque([0] * 200, 200)
        self.line, = ax.plot(self.dists)

    def drive(self, sensor: Sensor) -> Actuator:
        """ Produces a set of Actuator commands in response to Sensor data from
            the server .

            Args:
                sensor  - A set of Sensor values received from the server

            Returns: An Actuator populated with commands to send to the server
        """

        command = Actuator()

        """ REPLACE ALL CODE BETWEEN THESE COMMENTS """

        command.steering = 2*sensor.angle/180 - (0.2*sensor.distance_from_center)
        command.gear = self.select_gear(sensor, command)
        command.accelerator = 1 - abs(sensor.angle/180) - abs(sensor.speed_x) / 150
        command.brake = 3*abs(sensor.angle/180) - 0.1

        # Plot the camera and steering data
        self.cam_graph.add(sensor.distances_from_edge)
        self.steer_graph.add([command.steering, sensor.distance_from_center, sensor.angle])

        # Plotting Notes: add() takes an array
        #   ex.
        #       data = [item1, item2, item3]
        #       self.graph_name.add(data)
        #
        # It may be useful to see what each term in PID is contributing so you could graph:
        #   PID_params = [p_term, i_term, d_term]
        #   self.pid_graph.add(PID_params)

        self.dists.append(sensor.distance_from_center)
        self.line.set_ydata(self.dists)
        self.fig.canvas.flush_events()


        """ REPLACE ALL CODE BETWEEN THESE COMMENTS """

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
