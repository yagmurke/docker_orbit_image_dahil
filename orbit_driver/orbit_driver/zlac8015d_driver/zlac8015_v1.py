import minimalmodbus
import numpy as np
import time
import sys
from enum import Enum
from orbit_driver.zlac8015d_driver.address_ import Adresses

_RPM = 3
_QUIT_MODE = 0
_STOP_MOTOR = 0
_CCW = 1
_CW = 2
_ENABEL = 8
_DISABLE = 7

class MotorAddress(Enum):
    SOFTWARE_VERSION = 0
    BUS_VOLTAGE = 1
    STATUS_WORD = 2
    HALL_INPUT_STATUS = 3
    MOTOR_TEMP = 4
    LAST_DRIVE_FAULT_LEFT = 5
    LAST_DRIVE_FAULT_RIGHT = 6
    ACTUAL_POSITON_HIGH_LEFT = 7
    ACTUAL_POSITON_LOW_LEFT = 8
    ACTUAL_POSITON_HIGH_RIGHT = 9
    ACTUAL_POSITON_LOW_RIGHT = 10
    ACTUAL_SPEED_LEFT = 11
    ACTUAL_SPEED_RIGHT = 12
    ACTUAL_TORQUE_LEFT = 13
    ACTUAL_TORQUE_RIGHT = 14
    CONNECTION_FLAG = 15
    DRIVER_TEMP = 16
class Controller:
    def __init__(self,
                 port=None,
                 baudrate = 115200,
                 slave_id = 1):
        self.address = Adresses()
        self.servo = minimalmodbus.Instrument(port, slave_id)
        self.servo.serial.baudrate = baudrate
        self.servo.serial.bytesize = 8
        self.servo.serial.parity = minimalmodbus.serial.PARITY_NONE
        self.servo.serial.stopbits = 1
        self.servo.serial.timeout = 1
        self.servo.mode = minimalmodbus.MODE_RTU

        self.driving_mode = self.address.jog
    
    def hex_converter(self, hex):
        address = int(hex, base=16)
        return address
    
    def read_all(self):
        address = self.hex_converter("0x20A0")
        registers = self.servo.read_registers(address, 17)
        return registers
    
    def read_encoder(self, registers:list):
        l_pul_hi = registers[MotorAddress.ACTUAL_POSITON_HIGH_LEFT.value]
        l_pul_lo = registers[MotorAddress.ACTUAL_POSITON_LOW_LEFT.value]
        r_pul_hi = registers[MotorAddress.ACTUAL_POSITON_HIGH_RIGHT.value]
        r_pul_lo = registers[MotorAddress.ACTUAL_POSITON_LOW_RIGHT.value]

        l_tick = np.int32(((l_pul_hi & 0xFFFF) << 16) | (l_pul_lo & 0xFFFF))
        r_tick = np.int32(((r_pul_hi & 0xFFFF) << 16) | (r_pul_lo & 0xFFFF))

        return l_tick, r_tick
    
    def read_motor_speed(self, registers:list):
        left_speed_rpm = registers[MotorAddress.ACTUAL_SPEED_LEFT.value]
        right_speed_rpm = registers[MotorAddress.ACTUAL_SPEED_RIGHT.value]
        if left_speed_rpm >= 32768:
            left_speed_rpm = left_speed_rpm - 65536
        if right_speed_rpm >= 32768:
            right_speed_rpm = right_speed_rpm - 65536
        
        return left_speed_rpm, right_speed_rpm
    
    def read_temperature(self, registers:list):
        temp = registers[MotorAddress.MOTOR_TEMP.value]
        templ, tempr = divmod((temp), 0x100)
        return templ, tempr
    
    def read_driver_temp(self, registers:list):
        driver_temp = registers[MotorAddress.DRIVER_TEMP.value]
        return driver_temp / 10
    
    def read_current(self, registers:list):
        l_current = registers[MotorAddress.ACTUAL_TORQUE_LEFT.value]
        r_current = registers[MotorAddress.ACTUAL_TORQUE_RIGHT.value]
        
        if l_current >= 32768:
            l_current = l_current - 65536
        if r_current >= 32768:
            r_current = r_current - 65536
        return l_current/10, r_current/10
    
    def read_voltage(self, registers:list):
        voltage = registers[MotorAddress.BUS_VOLTAGE.value]
        return voltage / 100
    
    def set_acc(self, acc):
        address = self.hex_converter(self.address.acc)
        self.servo.write_registers(address, [acc,acc])
    
    def set_decc(self, decc):
        address = self.hex_converter(self.address.decc)
        self.servo.write_registers(address, [decc,decc])
        
    
    def set_mode(self, mode):
        address = self.hex_converter(self.address.mode)
        self.servo.write_register(address, mode)
    
    def servo_status(self):
        address = self.hex_converter(self.address.servo_status)
        self.servo.write_register(address, 4081)
        return self.servo.read_register(address)
    
    def enable_motor(self):
        address = self.hex_converter(self.address.enable_mot)
        self.servo.write_register(address, _ENABEL)
    
    def disable_motor(self):
        address = self.hex_converter(self.address.enable_mot)
        self.servo.write_register(address, _DISABLE)
    
    def set_rpm(self, right_motor, left_motor):
        address = self.hex_converter(self.address.rpm)
        if right_motor < 0:
            right_motor = right_motor + 2**16
        if left_motor < 0:
            left_motor = left_motor + 2**16
        self.servo.write_registers(address, [right_motor,left_motor])



if __name__ == "__main__":
    cntr = Controller(port= '/dev/hub_motor')
    # cntr.enable_motor()
    cntr.disable_motor()
    # cntr.set_rpm(-10,-10)
    while True:
        try:
            prev = time.time()
            registers = cntr.read_all()
            print(cntr.read_encoder(registers))
            print(cntr.read_driver_temp(registers))
            print(cntr.read_temperature(registers))
            time_diff = (time.time() - prev)
            print(time_diff * 10**3)
        except KeyboardInterrupt:
            cntr.disable_motor()
            time.sleep(1)
            break