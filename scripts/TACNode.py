#!/usr/bin/python
# coding=utf-8

"""
    TAC node manage tac on the deck and execute step address to them.
"""

# parsing of string containing dictionary
import ast

# ros subscription, publishing and node capability
import rospy

# ros message
from std_msgs.msg import String

LOG_STEP_RECEIVED = 'received a step with id: {0} temperature : {1} spin : {2} '
LOG_STEP_STATE = 'time : {0} od: {1} temperature : {2}/{3}'


def is_close(a, b, rel_tol=1e-09, abs_tol=0.0):
    """
    Check if two floats are close enough to be consider equal.
    """
    return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)


class TACModule(object):
    """
        The tac module represent a tac on the deck
    """
    def __init__(self, tac_id):
        """
        Constructor for a TAC module representation

        Attributes:
            serial_id    The TAC id, which should be unique among all the tac.

            _spin_speed     The agitator speed
            _target_temp    The temperature to be reached.
            _target_od      The optical density to be reached.

            _current_temp   The last temperature received for this TAC
            _current_od     The last optical density received for this TAC

            is_connected    Is the tac known to be connected
            step_time       time where the step was started
        """
        self.serial_id = tac_id
        self._target_temp = None
        self._target_od = None
        self._spin_speed = 0

        self._current_temp = None
        self._current_od = None

        self.is_connected = False
        self.step_time = None

        self.pub_move_done = rospy.Publisher('Done_Module', String, queue_size=10)

    def start_step(self, temperature=None, spin=None):
        """
        Start a step for the given TAC.
            :param temperature: the temperature to be reached for this step
            :param spin:  the spin speed for this step
        """
        self.step_time = rospy.Time.now()
        if temperature:
            self.set_target_temp(temperature)
            # send it via CAN
        if spin:
            self.set_spin(spin)
            # send it via CAN

    def update(self):
        """
            Request for optical density, temperature and spin speed through the CAN
        """
        # ask for the value on the CAN channel
        self.id

    def update_simulation(self):
        """
            This function only purpose is to simulate a TAC operating.
            It updates the temperature with a load and unload
            capacitor function and the DO with a linear function
        """
        od_max = 1

        if self._current_od is None:
            self._current_od = 0.0

        if self._current_temp is None:
            self._current_temp = 0.0

        if self._target_temp is None:
            return

        if is_close(self._current_temp, self._target_temp):
            self.pub_move_done.publish('tac')
        else:
            if self._current_temp < self._target_temp:
                # update de la current temp selon une equation de sinus integral?
                # delta_t = rospy.Time.now()-self.step_time
                # self._current_temp += (sin(delta_t.to_sec())/delta_t.to_sec())/1.16
                self._current_temp += 0.1
            else:
                if self._current_temp > self._target_temp:
                    self._current_temp -= 0.1

        # For simulation purpose, od rise when temp is above 37
        if self._current_temp > 37 and self._current_od < od_max:
            self._current_od += 0.02

        # for debug purpose
        delta_t = rospy.Time.now() - self.step_time
        dt = round(delta_t.to_sec(), 2)
        rospy.logdebug(LOG_STEP_STATE.format(dt, self._current_od, self._current_temp, self._target_temp))

    def get_temp(self):
        """
        :return: the last temperature received through the CAN
        """
        # pull temperature from can
        return self._current_temp

    def get_od(self):
        # pull od from can
        return self._current_od

    def set_target_temp(self, temperature):
        """
            Set a temperature to be reached by the TAC module
        """
        if temperature > 90:
            rospy.logerr("TAC target temperature can't be above 90°C")
            return
        if temperature < -5:
            rospy.logerr("TAC target temperature can't be under -5°C")
            return

        self._target_temp = temperature

        rospy.loginfo("TAC({0}) target temperature set to {1}.".format(self.serial_id, self._target_temp))

    def set_spin(self, spin):
        """
            Set a spin speed to be reached by the TAC module and send it using CAN
            :param spin: the spin speed the tac should reach
            :return:
        """
        if spin > 100:
            rospy.logerr("TAC spin speed is in percentage, can't be > 100")
            return
        if spin < 0:
            rospy.logerr("TAC target spin speed is in percentage, can't be < 100")
            return
        self._spin_speed = spin
        # send using can


class TACNode(object):
    """
        TACNode is a ROS node managing the tacs module present on the biobot plateform.
    """
    def __init__(self):
        rospy.init_node('TAC_node', log_level=rospy.DEBUG)
        rospy.loginfo("Initializing TAC node")
        rospy.Subscriber('step', String, self.on_step_receive)

        self.pub = rospy.Publisher('moduleValue', String, queue_size=10)
        self.rate = rospy.Rate(10)  # 10Hz
        self.simulation = False

        self.tacs = dict()

        self.tacs['123'] = TACModule('123')

        rospy.loginfo("Waiting for node message")

    def main_loop(self):
        """
        execution loop for a tac node
        """
        while not rospy.is_shutdown():
            if self.simulation:
                for tac_id in self.tacs:
                    self.tacs[tac_id].update_simulation()
            self.rate.sleep()

    def on_step_receive(self, data):
        """
        Call when a step is received from the tac/module_step topic
        """
        data = data.data.replace('"', '\"')
        step = ast.literal_eval(data)
        rospy.loginfo("received a step message" + str(data))
        if step["module_type"] == "tac":
            if not ("module_id" in step):
                rospy.logerr("The step does not contain any tac id.")
                # send a message through error topic
                return

            if not ("params" in step):
                rospy.logerr("The step does not contain any params.")
                # send a message through error topic
                return

            if not ("args" in step['params']):
                rospy.logerr("The params of the step does not contain any args.")
                # send a message through error topic
                return

            if not ("stop" in step['params']['args']):
                rospy.logerr("The step does not contain any stop condition, the step would run forever.")
                # send a message through error topic
                return

            if not ("set" in step['params']['args']):
                rospy.logerr("The step does not contain anything to be set.")
                # send a message through error topic
                return

            if "temperature" in step['params']['args']['set']:
                temperature = step['params']['args']['set']['temperature']
            else:
                rospy.logerr("The step does not contain anything to be set.")
                # send a message through error topic
                return

            tac_id = step['module_id']
            spin = step['params']['args']['set'].get('spin', 0)
            rospy.loginfo(LOG_STEP_RECEIVED.format(tac_id, temperature, spin))
            if not (tac_id in self.tacs):
                self.tacs[tac_id] = TACModule(tac_id)

            self.tacs[tac_id].start_step(temperature, spin)


if __name__ == '__main__':
    """
    Start a TacNode to manage tacs
    """
    try:
        tac_node = TACNode()
        tac_node.simulation = True
        tac_node.main_loop()

    except rospy.ROSInterruptException as err:
        rospy.logerr(err)
