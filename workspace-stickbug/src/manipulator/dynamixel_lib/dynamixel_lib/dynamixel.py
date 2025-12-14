from dynamixel_lib.dynamixel_models import _DynamixelModel
from dynamixel_lib.ctrl_table_value import _CtrlTableValue
from dynamixel_sdk import Protocol2PacketHandler, PortHandler, COMM_SUCCESS, GroupBulkRead, GroupSyncWrite
import threading
import time

class U2D2():
    # This set ensures that new instances of the class do not have repeated device paths
    _instance_device_strings = set()
    # Lock is required to make sure other threads cannot change the device string set 
    _lock = threading.Lock() 
    # This lock is actually kind of wrong - makes it so that every instance of a u2d2 shares the lock 
    # only one motor can write across all u2d2 controllers on a per thread basis - this is fine for any 
    # reasonable use case of the library that will be encountered in this lab 


    def __init__(self, device_path: str, baudrate: int) -> None:

        with U2D2._lock:
            if device_path in U2D2._instance_device_strings:
                raise ValueError(f"The device path {device_path} is already in use {list(U2D2._instance_device_strings)}")

            self.port_handler = PortHandler(device_path)
            self.port_handler.openPort()
            self.port_handler.setBaudRate(baudrate)
            self.packet_handler = Protocol2PacketHandler()
            U2D2._instance_device_strings.add(device_path)
    
    def bulk_read(self, ids, field):
        """
        Perform a bulk read operation.

        :param ids: List of Dynamixel IDs to read from.
        :param fields: List of _CtrlTableValue objects to read for each ID.
        :return: Dictionary of read data keyed by ID.
        """
        with self._lock:
            bulk_read = GroupBulkRead(self.port_handler, self.packet_handler)

            for dxl_id in ids:
                bulk_read.addParam(dxl_id, field.address, field.length_bytes)

            bulk_read.txRxPacket()

            results = {}
            for dxl_id in ids:
                data = {}
                raw_data = bulk_read.getData(dxl_id, field.address, field.length_bytes)
                # Ensure the data is returned as a list of bytes
                data[field] = [(raw_data >> (8 * i)) & 0xFF for i in range(field.length_bytes)]
                results[dxl_id] = data

            return results

    def bulk_write(self, ids, fields, values):
        """
        Perform a bulk write operation.

        :param ids: List of Dynamixel IDs to write to.
        :param fields: List of _CtrlTableValue objects to write for each ID.
        :param values: List of values to write for each field.
        """
        with self._lock:
            for field in set(fields):
                sync_write = GroupSyncWrite(self.port_handler, self.packet_handler, field.address, field.length_bytes)

                for dxl_id, value in zip(ids, values):
                    sync_write.addParam(dxl_id, self._make_data(value,field.length_bytes))

                sync_write.txPacket()

    def _make_data(self, val, length_bytes):
        """
        Create a data array suitable for the provided value and expected byte length.

        :param val: Integer value to be converted to byte array.
        :param length_bytes: The number of bytes to convert the value into.
        :return: List of bytes representing the value.
        """
        data_array = []
        for i in range(length_bytes):
            shift_amount = 8 * i
            data_array.append((val >> shift_amount) & 0xFF)
        return data_array


class Dynamixel():
    # This set ensures that new instances of the class do not have repeated ids
    _unique_ids_used = set()
    # Lock is required to make sure other threads cannot change the unique ids used string set
    _lock = threading.Lock()


    def __init__(self, model: _DynamixelModel, id: int, u2d2: U2D2):
        min_id = 0
        max_id = 253
    
        with Dynamixel._lock:

            if not(min_id <= id <= max_id):
                raise ValueError(f"The id {id} is not in the range of allowed values ({min_id} to {max_id}) (inclusive) ")

            if id in Dynamixel._unique_ids_used:
                raise ValueError(f"The id {id} is already in use {list(Dynamixel._unique_ids_used)}")
            
            if not issubclass(model, _DynamixelModel):
                raise ValueError(f"The provided model {model} is not an instance of a _DynamixelModel. Please use the provided model classes.")
            
            self._id: int = id
            self._u2d2: U2D2 = u2d2
            self.ctrl_table: _DynamixelModel = model

            Dynamixel._unique_ids_used.add(id)

    def _make_data(self, val):
        return [0xFF & val, (0xFF00 & val) >> 8, (0xFF0000 & val) >> 16, (0xFF000000 & val) >> 24]

    def write(self, location: _CtrlTableValue, written_data):

        #TODO need to add error checking for written_data later - not sure if need bytes, int, etc. check later.

        #TODO parse and give to user.

        with self._u2d2._lock:
            # if location.access in {"RW","R"}:
            #     raise ValueError(f"The location you have provided {location} is not able to be written to.")
            return self._u2d2.packet_handler.writeTxRx(self._u2d2.port_handler,self._id, location.address, location.length_bytes, self._make_data(written_data))
    
    def read(self, location: _CtrlTableValue):

        #TODO parse and give to user.
        with self._u2d2._lock:
            return self._u2d2.packet_handler.readTxRx(self._u2d2.port_handler,self._id, location.address, location.length_bytes)
 
    @staticmethod
    def bytes_to_int(byte_list):
        """
            used for reading unsigned values from dynamixels

            Convert a list of bytes (little-endian format) to an integer.
            The first byte is the least significant byte.

            :param byte_list: List of bytes, where the first byte is the least significant.
            :return: The integer representation of the bytes.
        """
        return int.from_bytes(byte_list, byteorder='little', signed=False)

    @staticmethod
    def bytes_to_twos_complement(byte_list):
        """

            used for reading signed values from dynamixels


            Convert a list of bytes (little-endian format) to a signed integer using two's complement.
            The first byte is the least significant byte.

            :param byte_list: List of bytes, where the first byte is the least significant.
            :return: The signed integer representation of the bytes.
        """
        return int.from_bytes(byte_list, byteorder='little', signed=True)

    def reboot(self, attempts=3):
        """
        Clear errors and reboot the Dynamixel servo.

        :param attempts: Number of reboot attempts.
        :return: True if reboot succeeded, False otherwise.
        """
        for attempt in range(attempts):
            try:
                # Clear the hardware error status (adjust field name if necessary)
                self.write(self.ctrl_table.HardwareErrorStatus, 0)
                
                # Attempt to reboot the servo
                result, error = self._u2d2.packet_handler.reboot(self._u2d2.port_handler, self._id)
                if result != COMM_SUCCESS:
                    error_msg = self._u2d2.packet_handler.getTxRxResult(result)
                    print(f"Attempt {attempt+1}: Failed to reboot Dynamixel ID {self._id}: {error_msg}")
                elif error != 0:
                    error_msg = self._u2d2.packet_handler.getRxPacketError(error)
                    print(f"Attempt {attempt+1}: Error during reboot of Dynamixel ID {self._id}: {error_msg}")
                else:
                    print(f"Dynamixel ID {self._id} rebooted successfully on attempt {attempt+1}.")
                    return True  # Successful reboot
                
                # Wait a moment before retrying
                time.sleep(1)
            except Exception as e:
                print(f"Exception on attempt {attempt+1} while rebooting Dynamixel ID {self._id}: {str(e)}")
                time.sleep(1)
        return False  # Failed after all attempts
            
            
            
