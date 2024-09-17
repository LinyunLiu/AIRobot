# Jacobus Burger (2024-07-04)
# Desc:
#   A rewrite of tacho.py that instead functions like an incremental
#   rotary encoder to provide not just tachometry data but also positonal
#   data of the motor (absolute angle and arc length traveled within one
#   revolution).
#   A rewrite to make it easier to use tacho.py code + helpful features
#   that allow this to function more like a wheel encoder and tachometer.
# Features:
#   - tachometry (RPM and Direction)
#   - wheel position (absolute angle in radians of current revolution)
#   - displacement (absolute arc length of current revolution)
# Info:
#   - https://en.wikipedia.org/wiki/Rotary_encoder
#   - https://www.ti.com/lit/ug/tiduc07/tiduc07.pdf?ts=1722819612354&ref_url=https%253A%252F%252Fwww.ti.com%252Ftool%252FTIDA-060040
#   - https://www.digikey.com/en/blog/using-bldc-hall-sensors-as-position-encoders-part-1
#   - https://www.vexforum.com/t/4-wheel-rotation-distance/20281
#   - https://en.wikipedia.org/wiki/Arc_length
#   For calculating velocities and displacments, refer to:
#   - https://en.wikipedia.org/wiki/Circular_motion
#   - https://openstax.org/books/physics/pages/6-1-angle-of-rotation-and-angular-velocity
# NOTE:
# - position 0 rad / 2pi rad is on right like (->)
# - CW and CCW must not be changed because they are always right with
#   respect to the rotation of the wheel itself.
# - interrupts come in order UWV CW, and UVW CCW
# - 2pi rad / 90 pulses ~= 4 deg per tick
# - at start, prev_pin is assumed to be U, which gives false direction for first
#   pulse if not going CW
import math
import time
import gpiozero


# CONSTANTS
CW = 1
CCW = -1
RADIUS = 0.0085  # meters
PULSES_PER_REV = 50  # not sure why it isn't 90 when it should be


# Tachometry class
class WheelEncoder:
    # public result values
    radius: float   # radius of the motor
    rpm: float      # speed of rotation (Revolutions Per Minute)
    direction: int  # direction of rotation (1 for CW, -1 for CCW)
    theta: float    # absolute angle of current revolution (in radians)
    length: float   # absolute arc length of current revolution
    # private variables
    #   constant pin devices
    __U: gpiozero.DigitalInputDevice
    __V: gpiozero.DigitalInputDevice
    __W: gpiozero.DigitalInputDevice
    #   last interrupt caller pin (for direction)
    __prev_pin: gpiozero.DigitalInputDevice
    #   public last interupt time
    __prev_time: float
    #   pulse period T (for rpm)
    __pulse_time_u: float
    __pulse_time_v: float
    __pulse_time_w: float
    __pulse_time_avg: float
    #   number of pulses elapsed (for theta and length)
    __pulse_count: int
    # smoothing value of Linear intERPolation (for rpm)
    __alpha: float      # between 0 and 1 inclusive
    # helpers
    __reversed: bool    # reverse returned RPM direction
    __debug: bool       # print state of variables and functions


    def __init__(self, u_pin, v_pin, w_pin, radius, reversed=False, debug=False, alpha=0.5):
        """
        u_pin: pin of u hall sensor
        v_pin: pin of v hall sensor
        w_pin: pin of w hall sensor
        radius: radius of the wheel
        reversed: whether to reverse all direction output (for opposite wheel)
        debug: whether to print debug messages (warning: walls of text)
        alpha: the alpha value of the lerp function by default


        An Incremental Encoder object with built-in Tachometry that can be used
        with any Brushless DC Motor with 3 hall effect sensors connected to 3
        GPIO input pins.

        The class is able to report on the RPM of the wheel, the direction of
        rotation, the absolute angle position within the current revoltion, and
        the absolute distance traveled by the wheel within the current revolution.

        public variables:
        - radius        the radius of the motor
        - rpm           the RPM of the motor
        - direction     the direction of rotation
        - theta         the absolute angle position (radians)
        - length        the absolute distance

        public functions:
        - lerp          Linear intERPolation smoothing function
        - get_rpm       calculates RPM from the pulse_time_avg
        - get_theta     calculates the absolute angle position (radians)
        - get_length    calculates the absolute distance
        - timed_out     returns True if there has been no pulses for some time
        """
        # define pins and info
        self.__U = gpiozero.DigitalInputDevice(u_pin)
        self.__V = gpiozero.DigitalInputDevice(v_pin)
        self.__W = gpiozero.DigitalInputDevice(w_pin)
        self.__reversed = reversed
        self.__debug = debug
        # setup interrupts
        self.__U.when_activated = self.__u_int
        self.__V.when_activated = self.__v_int
        self.__W.when_activated = self.__w_int
        # setup default values
        self.__prev_pin = None  # may cause false direction for first pulse if not CW
        self.__prev_time = 0
        self.__pulse_count = 0
        self.__pulse_time_u = 0
        self.__pulse_time_v = 0
        self.__pulse_time_w = 0
        self.__pulse_time_avg = 0
        self.__alpha = alpha
        # setup state values
        self.radius = radius
        self.rpm = 0
        self.direction = CW if not reversed else CCW
        self.theta = 0
        self.length = 0


    def lerp(self, previous, current, alpha=None):
        """
        lerp(previous, current, alpha)

        Linear intERPolation of values between previous and current where
        alpha determines weight
            - higher alpha prefers current value
            - lower alpha prefers previous value
        with alpha on interval [0.0, 1.0]

        Info:
            - https://en.wikipedia.org/wiki/Linear_interpolation
        """
        if alpha is None:
            alpha = self.__alpha
        result = (1 - alpha) * previous + alpha * current
        if self.__debug: print("lerp()")
        if self.__debug: print("  prev {}".format(previous))
        if self.__debug: print("  curr {}".format(current))
        if self.__debug: print("  alpha {}".format(alpha))
        if self.__debug: print("  lerp {}".format(result))
        return result


    def get_rpm(self, pulse_time_avg):
        """
        rpm(pulse_time_avg)
        
        RPM of the motor at the current instant given a pulse_time_avg
        as the time interval between pulses.

        Calculated with:
        2(f/90) * 60
        Where
            f = frequency (1 / T)
            T = pulses per second
            90 is the number of pulses per revolution
                30 poles, 3 sensors / pulsers. 30 * 3 = 90 pulses
            60 is the seconds per minute

        Info:
            - https://www.eevblog.com/forum/projects/calculating-rpm-from-bemf-frequency-of-bldc-motor/
        """
        f = 1 / pulse_time_avg  # f = 1 / T
        result = 2 * (f / PULSES_PER_REV) * 60
        if self.__debug: print("get_rpm()")
        if self.__debug: print("  pulse_time_avg {} seconds".format(pulse_time_avg))
        if self.__debug: print("  frequency {} Hertz".format(f))
        if self.__debug: print("  rpm {} RPM".format(result))
        return result


    def get_theta(self, pulse_count):
        """
        get_theta(pulse_count)

        Absolute position of the wheel in radians for the current revolution
        from its "zero point" which is the front of the wheel like (->).

        Calculated with:
        2pi*R
        Where
            2pi is the radians in 1 full revolution
            R is the amount of 1 revolution completed so far
            R = pulse_count / 90
            90 is the number of pulses in 1 full revolution

        Info:
            - https://en.wikipedia.org/wiki/Rotary_encoder
        """
        R = pulse_count / PULSES_PER_REV
        result = (2*math.pi) * R
        if self.__debug: print("get_theta()")
        if self.__debug: print("  pulse_count {} pulses".format(pulse_count))
        if self.__debug: print("  R {} revolutions".format(R))
        if self.__debug: print("  theta {} radians".format(result))
        return result


    def get_length(self, pulse_count):
        """
        get_length(pulse_count)

        Absolute distance traveled for the current revolution, using the arc
        length to rectify the wheel and find what length of it was traveled.

        Calculated with:
        r * theta
        Where
            r is the radius of the wheel
            theta is the position of the revolution in radians

        Info:
            - https://www.vexforum.com/t/4-wheel-rotation-distance/20281
            - https://en.wikipedia.org/wiki/Arc_length
        """
        theta = self.get_theta(pulse_count)
        result = self.radius * theta
        if self.__debug: print("get_length()")
        if self.__debug: print("  pulse_count {} pulses".format(pulse_count))
        if self.__debug: print("  r {} units".format(self.radius))
        if self.__debug: print("  theta {} radians".format(theta))
        if self.__debug: print("  length {} units".format(result))
        return result


    def __u_int(self):
        """
        __u_int()

        updates rpm, direction, theta position, and length traveled when the U
        pin receives an interrupt from its respective hall sensor.
        """
        # debug info before
        if self.__debug:
            print("__u_int() before")
            print("  __prev_pin {}".format(self.__prev_pin))
            print("  __prev_time {} seconds".format(self.__prev_time))
            print("  __pulse_time_u {} seconds".format(self.__pulse_time_u))
            print("  __pulse_time_v {} seconds".format(self.__pulse_time_v))
            print("  __pulse_time_w {} seconds".format(self.__pulse_time_w))
            print("  __pulse_time_avg {} seconds".format(self.__pulse_time_avg))
            print("  rpm {} RPM".format(self.rpm))
            print("  direction {}".format(self.direction))
            print("  theta {} radians".format(self.theta))
            print("  length {} units".format(self.length))
            print("  U V W {} {} {}".format(self.__U.value, self.__V.value, self.__W.value))
        # update pulse time
        elapsed_time = time.time()
        self.__pulse_time_u = elapsed_time - self.__prev_time
        self.__pulse_time_avg = (self.__pulse_time_u + self.__pulse_time_v + self.__pulse_time_w) / 3
        # update rpm
        prev_rpm = self.rpm
        self.rpm = self.lerp(prev_rpm, self.get_rpm(self.__pulse_time_avg), self.__alpha)
        self.__prev_time = elapsed_time
        # update direction
        if self.__prev_pin == self.__V:
            self.direction = CW if not self.__reversed else CCW
        if self.__prev_pin == self.__W:
            self.direction = CCW if not self.__reversed else CW
        self.__prev_pin = self.__U
        # if not self.__reversed:
        #     self.direction = CCW if self.__U.value == self.__W.value else CW
        # else:
        #    self.direction = CW if self.__U.value == self.__W.value else CCW
        # update pulse count
        self.__pulse_count = (self.__pulse_count + self.direction) % PULSES_PER_REV
        # update angle position theta
        self.theta = self.get_theta(self.__pulse_count)
        # update traveled arc length
        self.length = self.get_length(self.__pulse_count)
        # debug info after
        if self.__debug:
            print("__u_int() after")
            print("  __prev_pin {}".format(self.__prev_pin))
            print("  __prev_time {} seconds".format(self.__prev_time))
            print("  __pulse_time_u {} seconds".format(self.__pulse_time_u))
            print("  __pulse_time_v {} seconds".format(self.__pulse_time_v))
            print("  __pulse_time_w {} seconds".format(self.__pulse_time_w))
            print("  __pulse_time_avg {} seconds".format(self.__pulse_time_avg))
            print("  rpm {} RPM".format(self.rpm))
            print("  direction {}".format(self.direction))
            print("  theta {} radians".format(self.theta))
            print("  length {} units".format(self.length))
            print("  U V W {} {} {}".format(self.__U.value, self.__V.value, self.__W.value))



    def __v_int(self):
        """
        __v_int()

        updates rpm, direction, theta position, and length traveled when the V
        pin receives an interrupt from its respective hall sensor.
        """
        # debug info before
        if self.__debug:
            print("__v_int() before")
            print("  __prev_pin {}".format(self.__prev_pin))
            print("  __prev_time {} seconds".format(self.__prev_time))
            print("  __pulse_time_u {} seconds".format(self.__pulse_time_u))
            print("  __pulse_time_v {} seconds".format(self.__pulse_time_v))
            print("  __pulse_time_w {} seconds".format(self.__pulse_time_w))
            print("  __pulse_time_avg {} seconds".format(self.__pulse_time_avg))
            print("  rpm {} RPM".format(self.rpm))
            print("  direction {}".format(self.direction))
            print("  theta {} radians".format(self.theta))
            print("  length {} units".format(self.length))
            print("  U V W {} {} {}".format(self.__U.value, self.__V.value, self.__W.value))
        # update pulse time
        elapsed_time = time.time()
        self.__pulse_time_v = elapsed_time - self.__prev_time
        self.__pulse_time_avg = (self.__pulse_time_u + self.__pulse_time_v + self.__pulse_time_w) / 3
        # update rpm
        prev_rpm = self.rpm
        self.rpm = self.lerp(prev_rpm, self.get_rpm(self.__pulse_time_avg), self.__alpha)
        self.__prev_time = elapsed_time
        # update direction
        if self.__prev_pin == self.__W:
            self.direction = CW if not self.__reversed else CCW
        if self.__prev_pin == self.__U:
            self.direction = CCW if not self.__reversed else CW
        self.__prev_pin = self.__V
        # if not self.__reversed:
        #     self.direction = CCW if self.__V.value == self.__U.value else CW
        # else:
        #     self.direction = CW if self.__V.value == self.__U.value else CCW
        # update pulse count
        self.__pulse_count = (self.__pulse_count + self.direction) % PULSES_PER_REV
        # update angle position theta
        self.theta = self.get_theta(self.__pulse_count)
        # update traveled arc length
        self.length = self.get_length(self.__pulse_count)
        # debug info after
        if self.__debug:
            print("__v_int() after")
            print("  __prev_pin {}".format(self.__prev_pin))
            print("  __prev_time {} seconds".format(self.__prev_time))
            print("  __pulse_time_u {} seconds".format(self.__pulse_time_u))
            print("  __pulse_time_v {} seconds".format(self.__pulse_time_v))
            print("  __pulse_time_w {} seconds".format(self.__pulse_time_w))
            print("  __pulse_time_avg {} seconds".format(self.__pulse_time_avg))
            print("  rpm {} RPM".format(self.rpm))
            print("  direction {}".format(self.direction))
            print("  theta {} radians".format(self.theta))
            print("  length {} units".format(self.length))
            print("  U V W {} {} {}".format(self.__U.value, self.__V.value, self.__W.value))


    def __w_int(self):
        """
        __w_int()

        updates rpm, direction, theta position, and length traveled when the W
        pin receives an interrupt from its respective hall sensor.
        """
        # debug info before
        if self.__debug:
            print("__w_int() before")
            print("  __prev_time {} seconds".format(self.__prev_time))
            print("  __pulse_time_u {} seconds".format(self.__pulse_time_u))
            print("  __pulse_time_v {} seconds".format(self.__pulse_time_v))
            print("  __pulse_time_w {} seconds".format(self.__pulse_time_w))
            print("  __pulse_time_avg {} seconds".format(self.__pulse_time_avg))
            print("  rpm {} RPM".format(self.rpm))
            print("  direction {}".format(self.direction))
            print("  theta {} radians".format(self.theta))
            print("  length {} units".format(self.length))
            print("  U V W {} {} {}".format(self.__U.value, self.__V.value, self.__W.value))
        # update pulse time
        elapsed_time = time.time()
        self.__pulse_time_w = elapsed_time - self.__prev_time
        self.__pulse_time_avg = (self.__pulse_time_u + self.__pulse_time_v + self.__pulse_time_w) / 3
        # update rpm
        prev_rpm = self.rpm
        self.rpm = self.lerp(prev_rpm, self.get_rpm(self.__pulse_time_avg), self.__alpha)
        self.__prev_time = elapsed_time
        # update direction
        if self.__prev_pin == self.__U:
            self.direction = CW if not self.__reversed else CCW
        if self.__prev_pin == self.__V:
            self.direction = CCW if not self.__reversed else CW
        self.__prev_pin = self.__W
        # if not self.__reversed:
        #    self.direction = CCW if self.__W.value == self.__V.value else CW
        # else:
        #    self.direction = CW if self.__W.value == self.__V.value else CCW
        # update pulse count
        self.__pulse_count = (self.__pulse_count + self.direction) % PULSES_PER_REV
        # update angle position theta
        self.theta = self.get_theta(self.__pulse_count)
        # update traveled arc length
        self.length = self.get_length(self.__pulse_count)
        # debug info after
        if self.__debug:
            print("__v_int() after")
            print("  __prev_pin {}".format(self.__prev_pin))
            print("  __prev_time {} seconds".format(self.__prev_time))
            print("  __pulse_time_u {} seconds".format(self.__pulse_time_u))
            print("  __pulse_time_v {} seconds".format(self.__pulse_time_v))
            print("  __pulse_time_w {} seconds".format(self.__pulse_time_w))
            print("  __pulse_time_avg {} seconds".format(self.__pulse_time_avg))
            print("  rpm {} RPM".format(self.rpm))
            print("  direction {}".format(self.direction))
            print("  theta {} radians".format(self.theta))
            print("  length {} units".format(self.length))
            print("  U V W {} {} {}".format(self.__U.value, self.__V.value, self.__W.value))


    def timed_out(self, timeout):
        """
        timed_out(timeout)

        Returns True when pins have not triggered an interrupt for
        `timeout` seconds.
        """
        elapsed_time = time.time()
        if self.__debug: print("timed_out({})".format(timeout))
        if self.__debug: print("  elapsed_time: {}".format(elapsed_time))
        if self.__debug: print("  self.__prev_time: {}".format(self.__prev_time))
        if elapsed_time - self.__prev_time >= timeout:
            return True
        return False


# That's it! The rest of the logic is handled outside the class
