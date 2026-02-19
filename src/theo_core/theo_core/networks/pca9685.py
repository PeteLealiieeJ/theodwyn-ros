import  busio
from    adafruit_servokit                  import ServoKit
from    typing                             import Optional, List, Tuple

#----------------------------------------------------------------------------------------------------------
#  Functionality
#----------------------------------------------------------------------------------------------------------

# -> PCA9685. Adafruit Servokit Implementation (Easiest)
class PCA9685:
    """
    Network Class for Adafruit PCA9685
    :param n_channels: Number of channels of the PCA9685
    :param SDA: Identifier for the connected SDA pin
    :param SCL: Identifier for the connected SCL pin
    :param init_cmd: Initial command to send via the send method (in many cases this must be called prior to reading data)
    :param actuation_range: A list of actuation ranges for the connected servos
    :param min_max_pwr: A list of tuples containing the (min,max) pulse widths for a respective servo
    :param safety_bounds: An optional list of safety bounds to restrict the actuation of servos (not used for determining pwm signals)
    """    
    SDA             : int
    SCL             : int
    freq            : int = 100000
    i2c_bus         : busio.I2C 
    channels        : int 
    servokit        : Optional[ServoKit] = None
    init_cmd        : Optional[List[float]]
    actuation_range : Optional[List[float]]
    min_max_pwr     : Optional[List[Tuple[int,int]]]
    safety_bounds   : Optional[List[Optional[Tuple[float,float]]]]

    def __init__( 
        self,
        SDA             : int,
        SCL             : int,
        channels        : int = 16, 
        init_cmd        : Optional[List[float]] = None,
        actuation_range : Optional[List[float]] = None,
        min_max_pwr     : Optional[List[Tuple[int,int]]] = None,
        safety_bounds   : Optional[List[Optional[Tuple[float,float]]]] = None,
    ):
        self.channels        = channels,
        self.SDA             = SDA,
        self.SCL             = SCL,
        self.init_cmd        = init_cmd,
        self.actuation_range = actuation_range,
        self.min_max_pwr     = min_max_pwr,
        self.safety_bounds   = safety_bounds,    
    

    def connect( self ):
        # Connection
        self.i2c_bus = busio.I2C( scl=self.SCL, sda=self.SDA, frequency=self.freq )
        self.servokit = ServoKit( channels = self.channels , i2c = self.i2c_bus )
        if self.actuation_range is not None:
            for i, actuation_range_i in enumerate(self.actuation_range):
                self.servokit.servo[i].actuation_range = actuation_range_i
        if self.min_max_pwr is not None:
            for i, min_max_i in enumerate(self.min_max_pwr):
                self.servokit.servo[i].set_pulse_width_range(min_pulse=min_max_i[0],max_pulse=min_max_i[1])
        self.send( self.channels*[0.] ) if self.init_cmd is None else self.send( self.init_cmd )


    def disconnect( self ):
        # Disconnection
        self.send( self.channels*[0.] ) if self.init_cmd is None else self.send( self.init_cmd )
        self.i2c_bus.deinit()

    
    def _check_safe( 
        self,
        safety_bound : Optional[Tuple[float,float]], 
        cmd : float
    ) -> None:
        """
        Checks whether fulfilling a given command will violate saftey contraints
        """
        if safety_bound is not None:
            if cmd < safety_bound[0] or cmd > safety_bound[1]:
                raise ValueError('Angle is out of prescribed safety bound')
        return
    

    def send(
        self, 
        cmd : List[float], 
        **kwargs
    ) -> Tuple[bool,str]:
        """
        Send command through network
        :param cmd: Command to be sent
        """

        if self.servokit is None : raise RuntimeError('Connection to IIC has not been established')


        for i, cmd_i in enumerate( cmd ):
            try:
                if self.safety_bounds is not None: 
                    self._check_safe( 
                        safety_bound=self.safety_bounds[i], 
                        cmd=cmd_i 
                    )
                self.servokit.servo[i].angle = cmd_i
            except ValueError as e:
                return False, e
            return True, ""