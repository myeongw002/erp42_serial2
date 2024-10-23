#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#한글 주석 선언

'''
Specification : RS232
DATA Ordering : Little Endian (PCU to UPPER) / Big Endian (UPPER to PCU)
Cycle Time : 20 msec
Baud:115200, parity : None, Stop : 1
언맨드 제어기 (이하 PCU)
USER PC or 제어기 (이하 UPPER)

UPPER to PCU: pc에서 차로보내는 값
[S,T,X,AorM,E-STOP,GEAR,SPEED0,SPEED1,STEER0,STEER1,BRAKE,ALIVE,ETX0,ETX1]

PCU to UPPER: 차에서 PC로 오는 값
[S,T,X,AorM,E-STOP,GEAR,SPEED0,SPEED0,STEER0,STEER1,BRAKE,ALIVE,ETX0,ETX1]

STX -> Start of TEXT// [0x53, 0x54, 0x58]
AorM -> Auto or Manual, 0x00 : manual mode , 0x01 : auto mode
ESTOP -> Emergency STOP, 0x00 : E-STOP Off, 0x01 : E-STOP On
GEAR -> 0x00 : forward drive, 0x01 : neutral, 0x02 : backward drive
SPEED -> actual speed (KPH) * 10 // 0~200
STEER -> actual steering dgree (dgree) * 71, 오차율 : 4%, negative is left steer // -2000~2000
BRAKE -> 1 : no braking, 150 : full braking // 1~150
ENC -> encoder counting // -2^31~2^31
ALIVE -> increasing each one step // 0~255
ETX: End of TEXT // [0x0D,0x0A]

Packet Default
byte name : [[0x53, 0x54, 0x58], 0x01, 0x00, GEAR, SPEED0, SPEED1, STEER0, STEER1, BRAKE, 0, [0x0D, 0x0A]]
'''

import rospy
from std_msgs.msg import Int16MultiArray
from erp42_serial2.msg import ERP_CMD
from erp42_serial2.msg import ERP_STATUS
import struct
import time
import serial


class ERP42_Serial:

    def __init__(self):
          # Set a PORT Number & baud rate
        self.serial_begin()

        #Serial communication protocol variables
        self.STX = (0x53, 0x54, 0x58) #default !BBB
        self.AorM = 0x01 #default B
        self.ESTOP = 0x00 #B
        self.GEAR = 0 #0,1,2 B
        self.SPEED = 0 #0~200 H 
        self.STEER = 0 #-2000~2000 h
        self.BRAKE = 1 #1~150 B
        self.ALIVE = 0 #0~254 B
        self.ETX = (0x0D,0x0A) #default BB

        self.sub = rospy.Subscriber('/erp42_cmd', ERP_CMD, self.serial_callback, queue_size = 1)
        self.pub1 = rospy.Publisher('/erp42_status', ERP_STATUS, queue_size=1) #erp42_velocity
        self.pub2 = rospy.Publisher("/erp42_raw", Int16MultiArray, queue_size=1)

    def serial_begin(self):
        self.PORT = str(rospy.get_param("~PORT", "/dev/ttyERP42"))
        self.BAUDRATE = int(rospy.get_param("~BAUDRATE", "115200"))
        rospy.loginfo(f'PORT: {self.PORT}')
        rospy.loginfo(f"BAUDRATE: {self.BAUDRATE}")        
        self.SERIAL = serial.Serial(self.PORT, self.BAUDRATE)



    def set_params(self,GEAR=1, SPEED=0, STEER=0, BRAKE=0):
        self.ALIVE = (self.ALIVE + 1) % 255 
        data = [self.STX[0], self.STX[1], self.STX[2], self.AorM, self.ESTOP, 
                     int(GEAR), int(SPEED), int(STEER), int(BRAKE), self.ALIVE, self.ETX[0], self.ETX[1]]
        
        return data
       

    def Serial_write(self):
        data = self.set_params(self.GEAR, self.SPEED, self.STEER, self.BRAKE)
                
        data_pack = struct.pack('>BBBBBBHhBBBB', *data)
        self.SERIAL.write(data_pack)
        #print(data_pack) 
        rospy.loginfo(f'speed:{self.SPEED}, steer:{self.STEER}, brake:{self.BRAKE}, gear:{self.GEAR}')
            
            
    def Serial_read(self):
        data = self.SERIAL.read(18) #read 18byte
        raw_data = Int16MultiArray()
        raw_data.data = list(data)
        rospy.loginfo(f'data recieved:{data}')
        #rospy.loginfo(type(list(data)))
        data = struct.unpack('<BBBBBBHhBiBBB', data)
        rospy.loginfo(f'data recieved:{data}')

        status_msg = ERP_STATUS()
        status_msg.status_AorM = data[3]
        status_msg.status_gear = data[5]
        status_msg.status_speed = data[6]
        status_msg.status_steer = data[7]
        status_msg.status_brake = data[8]
        status_msg.status_enc = data[9]
        
        self.pub1.publish(status_msg)
        self.pub2.publish(raw_data)
              

    def serial_test(self):
        while True:
            if self.SERIAL.readable():
                data = self.SERIAL.read(18) #read 18byte
                stx = (data[0],data[1],data[2])
                rospy.loginfo(f'data recieved:{data}')

                if stx != self.STX:
                    rospy.logerr("Serial Communication Error")
                    self.close_serial()
                    time.sleep(2)
                    self.serial_begin()
                
                else:
                    rospy.loginfo("Serial Communication Success")
                    break  



    def serial_callback(self, msg:ERP_CMD):
        self.GEAR = msg.cmd_gear
        self.SPEED = msg.cmd_speed
        self.STEER = msg.cmd_steer
        self.BRAKE = msg.cmd_brake            


    def close_serial(self):
        if self.SERIAL.is_open:
            self.SERIAL.close()       
        

def main():
    rospy.init_node('erp42_serial')
    rospy.loginfo("Start ERP-42")
 
    
    erp42_serial = ERP42_Serial()
    erp42_serial.serial_test()
    rate = rospy.Rate(100)

    try:
        while not rospy.is_shutdown():
            #ESD.set_params(ESD.GEAR,ESD.SPEED,ESD.STEER,ESD.BRAKE)
            erp42_serial.Serial_read()
            erp42_serial.Serial_write()
            rate.sleep()
    finally:
        erp42_serial.close_serial()


if __name__ == '__main__':
    main()

    
    