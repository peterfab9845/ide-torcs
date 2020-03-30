""" driver.py 
    
    Author:     Carson Clarke-Magrab <ctc7359@rit.edu>
    Date:       03/23/2019
    Modified:
        03/29/2019 by Carson Clarke-Magrab <ctc7359@rit.edu>
        -   Adapted for IDE virtual NXP Cup.
"""

from client.car import Actuator, Sensor

# Global variables
EX_GLOBAL_VAR = 0   # Example global variable
BANG_BANG_TRN = 0.2


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

        # Define class variables here
        self.ex_class_var = 0   # Example class variables

        pass

    def drive(self, sensor: Sensor) -> Actuator:
        """ Produces a set of Actuator commands in response to Sensor data from
            the server .

            Args:
                sensor  - A set of Sensor values received from the server

            Returns: An Actuator populated with commands to send to the server
        """

        command = Actuator()

        """ REPLACE ALL CODE BETWEEN THESE COMMENTS """

        # Simple Bang-Bang controller
        if sensor.distance_from_center < 0:
            command.steering = BANG_BANG_TRN
        elif sensor.distance_from_center > 0:
            command.steering = -1 * BANG_BANG_TRN
        else:
            command.steering = 0

        command.gear = 1
        command.accelerator = 0.2

        """ REPLACE ALL CODE BETWEEN THESE COMMENTS """

        return command
