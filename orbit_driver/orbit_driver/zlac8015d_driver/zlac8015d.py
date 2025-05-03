import minimalmodbus
import numpy as np
import time
import sys
from orbit_driver.zlac8015d_driver.address_ import Adresses

_RPM = 3
_QUIT_MODE = 0
_STOP_MOTOR = 0
_CCW = 1
_CW = 2
_ENABEL = 8
_DISABLE = 7

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
    
    def read_encoder(self):
        address = self.hex_converter(self.address.encoder)
        registers = self.servo.read_registers(address,4)
        l_pul_hi = registers[0]
        l_pul_lo = registers[1]
        r_pul_hi = registers[2]
        r_pul_lo = registers[3]

        l_tick = np.int32(((l_pul_hi & 0xFFFF) << 16) | (l_pul_lo & 0xFFFF))
        r_tick = np.int32(((r_pul_hi & 0xFFFF) << 16) | (r_pul_lo & 0xFFFF))

        return l_tick, r_tick
    
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
    cntr = Controller(port= '/dev/ttyUSB2')
    cntr.enable_motor()
    # cntr.set_rpm(-10,-10)
    while True:
        try:
            print(cntr.read_encoder())
        except KeyboardInterrupt:
            cntr.disable_motor()
            time.sleep(1)
            break
