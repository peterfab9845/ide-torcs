""" driver.py 
    
    Author:     Mike Pagel <https://github.com/moltob>
    Date:       11/01/2016
    Modified:   
        03/23/2019 by Carson Clarke-Magrab <ctc7359@rit.edu>
        -   Adapted for IDE virtual NXP Cup.
"""

from client.car import Actuator, Sensor

# Global variables
TURN_AMT = 0.1;
ACCL_AMT = 0.3

class Driver:
    """ Car driving logic

        The Driver recieves a set of Sensor data from the server and transmits 
        a set of Actuator data in response. See car.py for more details on 
        these structures.

        The drive() function is called approximately every 20ms and must return 
        an Actuator object within 10ms.

        You may add new functions and modify this class as you see fit, but do not 
        remove the drive() function or change the name of the class.
    """

    def init(self):
        """ If you need to initialize any variables, do it here and remove the 
            'pass' statement. 
        """
        pass

    def drive(self, sensor: Sensor) -> Actuator:
        """ Produces a set of Actuator commands in response to Sensor data from
            the server .

            Args:
                sensor  - A set of Sensor values recived from the server

            Returns: An Actuator populated with commands to send to the server
        """

        command = Actuator()

        # REPLACE ALL CODE BETWEEN THESE COMMENTS
        if sensor.distance_from_center < 0:
            command.steering = TURN_AMT
            
        elif sensor.distance_from_center > 0:
            command.steering = -TURN_AMT

        else:
            command.steering = 0

        command.accelerator = ACCL_AMT
        command.gear = 1
        # REPLACE ALL CODE BETWEEN THESE COMMENTS

        return command

    