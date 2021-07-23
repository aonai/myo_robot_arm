import struct
import math


class DataHandler:
    """
    EMG/IMU/Classifier data handler.
    """
    def __init__(self, config):
        # self.osc = udp_client.SimpleUDPClient(config.OSC_ADDRESS, config.OSC_PORT)
        self.printEmg = config.PRINT_EMG
        self.printImu = config.PRINT_IMU
        self.printGest = config.PRINT_GEST
    
    def handle_gest(self, payload):
        val = struct.unpack('6B',  payload['value'])
        if self.printGest:
            print("gest", payload['connection'], payload['atthandle'], val)
        return payload['connection'], val


    def handle_emg(self, payload):
        """
        Handle EMG data.
        :param payload: emg data as two samples in a single pack.
        """
        val = struct.unpack('<8b',  payload['value'][:8])
        val2 = struct.unpack('<8b',  payload['value'][8:]) 
        if self.printEmg:
            print("EMG", payload['connection'], payload['atthandle'], val)
        return payload['connection'], val

    def handle_imu(self, payload):
        """
        Handle IMU data.
        :param payload: imu data in a single byte array.
        """
        vals = struct.unpack('10h', payload['value'])
        quat = vals[:4]
        acc = vals[4:7]
        gyro = vals[7:10]

        if self.printImu:
            print("IMU", payload['connection'], payload['atthandle'], quat, acc, gyro)
        
        return payload['connection'], quat, acc, gyro


    @staticmethod
    def _euler_angle(w, x, y, z):
        """
        From https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles.
        """
        # roll (x-axis rotation)
        sinr_cosp = +2.0 * (w * x + y * z)
        cosr_cosp = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = +2.0 * (w * y - z * x)
        if math.fabs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # yaw (z-axis rotation)
        siny_cosp = +2.0 * (w * z + x * y)
        cosy_cosp = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    @staticmethod
    def _vector_magnitude(x, y, z):
        return math.sqrt(x * x + y * y + z * z)
