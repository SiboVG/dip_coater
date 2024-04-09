from TMC_2209.TMC_2209_StepperDriver import *
from TMC_2209._TMC_2209_logger import Loglevel
import TMC_2209._TMC_2209_math as tmc_math
from TMC_2209._TMC_2209_move import StopMode
import time

# ======== CONSTANTS ========
TRANS_PER_REV = 8  # The vertical translation in mm of the coater for one revolution of the motor


class TMC2209_MotorDriver:
    """ Class to control the TMC2209 motor driver for the dip coater"""
    def __init__(self, _stepmode=8, _loglevel=Loglevel.ERROR):
        """ Initialize the motor driver

        :param _stepmode: The step mode to set (1, 2, 4, 8, 16, 32, 64, 128, 256)
        :param _loglevel: The log level to set for the motor driver (NONE, ERROR, INFO, DEBUG, MOVEMENT, ALL)
        """
        # GPIO pins
        dir_pin = 26
        step_pin = 16
        en_pin = 21

        # Motor driver
        self.tmc = TMC_2209(en_pin, step_pin, dir_pin, loglevel=_loglevel)

        self.tmc.tmc_logger.set_loglevel(_loglevel)  # NONE, ERROR, INFO, DEBUG, MOVEMENT, ALL

        # Set motor driver settings
        self.tmc.set_vactual(True)      # Motor is controlled by UART
        self.tmc.set_direction_reg(False)
        self.tmc.set_current(1500)
        self.tmc.set_interpolation(True)
        self.tmc.set_spreadcycle(False)  # True: spreadcycle, False: stealthchop
        self.tmc.set_microstepping_resolution(_stepmode)  # 1, 2, 4, 8, 16, 32, 64, 128, 256
        self.tmc.set_internal_rsense(False)

    def set_vactual_dur_ns(self, vactual, duration=0, acceleration=0,
                             show_stallguard_result=False, show_tstep=False):
        """sets the register bit "VACTUAL" to to a given value
        VACTUAL allows moving the motor by UART control.
        It gives the motor velocity in +-(2^23)-1 [μsteps / t]
        0: Normal operation. Driver reacts to STEP input

        Args:
            vactual (int): value for VACTUAL
            duration (float): after this time in ms, vactual will be set to 0 (Default value = 0)
            acceleration (int): use this for a velocity ramp (Default value = 0)
            show_stallguard_result (bool): prints StallGuard Result during movement
                (Default value = False)
            show_tstep (bool): prints TStep during movement (Default value = False)

        Returns:
            stop (enum): how the movement was finished
        """
        self.tmc._stop = StopMode.NO
        current_vactual = 0
        sleeptime = 10  # ns
        if vactual<0:
            acceleration = -acceleration

        if duration != 0:
            self.tmc.tmc_logger.log(f"vactual: {vactual} for {duration} ms",
                                Loglevel.INFO)
        else:
            self.tmc.tmc_logger.log(f"vactual: {vactual}", Loglevel.INFO)
        self.tmc.tmc_logger.log(str(bin(vactual)), Loglevel.INFO)

        self.tmc.tmc_logger.log("writing vactual", Loglevel.INFO)
        if acceleration == 0:
            self.tmc.set_vactual(int(round(vactual)))

        if duration == 0:
            return -1

        self._starttime = time.time_ns()
        current_time = time.time_ns()
        while current_time < self._starttime+duration*1e6:
            if self.tmc._stop == StopMode.HARDSTOP:
                break
            if acceleration != 0:
                time_to_stop = self._starttime+duration*1e6-abs(current_vactual/acceleration)*1e9
                if self.tmc._stop == StopMode.SOFTSTOP:
                    time_to_stop = current_time-1
            if acceleration != 0 and current_time > time_to_stop:
                current_vactual -= acceleration*sleeptime
                self.tmc.set_vactual(int(round(current_vactual)))
                time.clock_nanosleep(sleeptime)
            elif acceleration != 0 and abs(current_vactual)<abs(vactual):
                current_vactual += acceleration*sleeptime
                self.tmc.set_vactual(int(round(current_vactual)))
                time.clock_nanosleep(sleeptime)
            if show_stallguard_result:
                self.tmc.tmc_logger.log(f"StallGuard result: {self.tmc.get_stallguard_result()}",
                                    Loglevel.INFO)
                time.sleep(0.1)
            if show_tstep:
                self.tmc.tmc_logger.log(f"TStep result: {self.tmc.get_tstep()}",
                                    Loglevel.INFO)
                time.sleep(0.1)
            current_time = time.time_ns()
        self.tmc.set_vactual(0)
        return self.tmc._stop

    def set_vactual_rps_ns(self, rps, duration=0, revolutions=0, acceleration=0):
        """converts the rps parameter to a vactual value which represents
        rotation speed in revolutions per second
        With internal oscillator:
        VACTUAL[2209] = v[Hz] / 0.715Hz

        Args:
            rps (float): value for vactual in rps
            duration (float): after this vactual will be set to 0 (Default value = 0)
            revolutions (float): after this vactual will be set to 0 (Default value = 0)
            acceleration (int): use this for a velocity ramp (Default value = 0)

        Returns:
            stop (enum): how the movement was finished
        """
        vactual = tmc_math.rps_to_vactual(rps, self.tmc._steps_per_rev)
        if revolutions !=0:
            duration = abs((revolutions*1e3)/rps)
        if revolutions < 0:
            vactual = -vactual
        return self.set_vactual_dur_ns(vactual, duration, acceleration=acceleration)

    def set_stepmode(self, _stepmode=4):
        """ Set the step mode of the motor driver

        :param stepmode: The step mode to set (1, 2, 4, 8, 16, 32, 64, 128, 256)
        """
        self.tmc.set_microstepping_resolution(_stepmode)

    def enable_motor(self):
        """ Arm the motor"""
        self.tmc.set_motor_enabled(True)

    def disable_motor(self):
        """ Disarm the motor """
        self.tmc.set_motor_enabled(False)

    def drive_motor(self, distance_mm, speed_mm_s):
        """ Drive the motor to move the coater up or down by the given distance at the given speed

        :param distance_mm: The distance to move the coater in mm (positive for up, negative for down)
        :param speed_mm_s: The speed at which to move the coater in mm/s (always positive)
        """
        revs, rps = self.calculate_revs_and_rps(distance_mm, speed_mm_s)
        self.set_vactual_rps_ns(rps, revolutions=revs)

    def move_up(self, distance_mm, speed_mm_s):
        """ Move the coater up by the given distance at the given speed

        :param distance_mm: The distance to move the coater up in mm
        :param speed_mm_s: The speed at which to move the coater up in mm/s
        """
        self.drive_motor(distance_mm, speed_mm_s)

    def move_down(self, distance_mm, speed_mm_s):
        """ Move the coater down by the given distance at the given speed

        :param distance_mm: The distance to move the coater down in mm
        :param speed_mm_s: The speed at which to move the coater down in mm/s
        """
        self.drive_motor(-distance_mm, speed_mm_s)

    def cleanup(self):
        """ Clean up the motor driver for shutdown"""
        self.disable_motor()
        del self.tmc

    @staticmethod
    def calculate_revs_and_rps(distance_mm, speed_mm_s):
        """ Calculate how many revolutions the motor should turn at what speed to
            achieve the desired vertical distance translation at the speed.

        :param distance_mm: The distance to move the coater in mm (positive for up, negative for down)
        :param speed_mm_s: The speed at which to move the coater in mm/s (always positive)
        """
        revs = distance_mm / TRANS_PER_REV
        rps = speed_mm_s / TRANS_PER_REV
        return revs, rps


if __name__ == "__main__":
    # ======== SETTINGS ========
    # Step mode
    stepmode = 16  # 1 (full), 2 (half), 4 (1/4), 8, 16, 32, 64, 128, 256

    # Movement speed in mm/s
    speed_up = 2
    speed_down = 5

    # Time to wait (in s) between going up and down
    wait_time = 5

    # Vertical travel distance in mm
    distance_up = 10
    distance_down = distance_up

    # ======== DEBUG SETTINGS =======
    loglevel = Loglevel.INFO  # NONE, ERROR, INFO, DEBUG, MOVEMENT, ALL

    # ======== INIT ========
    motor_driver = TMC2209_MotorDriver(_stepmode=stepmode, _loglevel=Loglevel.INFO)

    # ======== MOVE DOWN ========
    motor_driver.move_down(distance_down, speed_down)

    # ======== WAIT ========
    time.sleep(wait_time)

    # ======== MOVE UP ========
    motor_driver.move_up(distance_up, speed_up)

    # ======== FINISH ========
    motor_driver.disable_motor()
    motor_driver.cleanup()
    print("---\nSCRIPT FINISHED\n---")
