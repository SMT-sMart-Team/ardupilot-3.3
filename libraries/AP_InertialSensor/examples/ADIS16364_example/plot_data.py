#!/usr/bin/env python
########################################################################################################
#  Analog Devices, Inc.
#  June 2012
#  By: Adam Gleason, and Brian Holford
########################################################################################################
#  plot_data.py 
########################################################################################################
#  NOTES:
#  Use this script to plot iSensor data from the Arduino. I'm not a python expert, and this code may
#  be sickening to view for some python gurus, but it's what I used to get the job done.
#  You will have to change serial_device to match the serial port on your computer, I did all my
#  development under Fedora Linux, please refer to the pyserial
#  documentation on how to access the serial port on your computer.
########################################################################################################

from numpy import *
from scipy.interpolate import spline
import matplotlib.pyplot as pl
import math
import serial
import time
import argparse 

# parse arguments
parser = argparse.ArgumentParser()
parser.add_argument('-p', '--port', required=True, help='Serial port, ie /dev/ttyACMX on Linux, or COMX on Windows')
parser.add_argument('-r', '--rate', required=True, help='Baud rate')
parser.add_argument('-b', '--bar', action='store_true', default=False, help='Show Bargraphs Instead of line plots')
parser.add_argument('-n', '--num', action='store', default=100, help='Window width, default is 100 points')
parser.add_argument('-s', '--smooth', action='store_true', default=False, help='Make lines look smoother')
args = parser.parse_args()
argz = vars(args)
bar = argz['bar'] 
serial_device = argz['port'] 
baud_rate = argz['rate']
smooth = argz['smooth']
num_p = int(argz['num'])

# Alows plot to be interactive
pl.ion()

# Open up serial device 
ser = serial.Serial(serial_device, baud_rate, timeout=1)

# Intialize 11xnum_p zero filled data matrix
data = zeros((11,num_p));

# intialize t array for plot
t = linspace(0,num_p,num_p) 
T = linspace(t.min(),t.max(),num_p*100)

count = 0

# Let python read the intial garbage sitting on serial buffer, before processing
#for bs in range(0,10):
#  ser.readline()

while(1):
  # Send D character, this will prompt Arduino to send a packet of data
  ser.write('D')

  # Read data from serial
  line = ser.readline()

  # This is to make sure that data packet is formed properly
  if(len(line)> 10):
    if(line[0] != '[' or line[-3] != ']'):
      continue
  else:
    continue

  # Split line by spaces 
  line = line.split(' ')
  # Splice line, and only get the inside 
  x = line[1:12]

  # if there are 11 data items
  if(len(x)==11):
    # loop through x
    for i in range(0,11):

      # except value error by skipping to the next loop iteration
      try:
        num = float(x[i])
      except ValueError:
        break

      if(count >= num_p - 1):
        # rotate array
        data[i] = roll(data[i],-1)
        data[i][-1] = num
        count = num_p - 1 
      else:
        data[i][count] = num 

    # gyro
    pl.subplot(211)
    if(bar):
      pl.bar(0,data[1][count], color='b')
      pl.bar(1,data[2][count], color='g')
      pl.bar(2,data[3][count], color='r')
      pl.axis([0,3,-300,300])
    elif(smooth):
      pl.plot(T,spline(t,data[1],T), 'b')
      pl.plot(T,spline(t,data[2],T), 'g')
      pl.plot(T,spline(t,data[3],T), 'r')
      pl.axis([0,num_p,-300,300])
    else:
      pl.plot(t,data[1],'b')
      pl.plot(t,data[2],'g')
      pl.plot(t,data[3],'r')
      pl.axis([0,num_p,-300,300])
    pl.title('Gyroscope')
    pl.ylabel('deg/s')

    # accel
    pl.subplot(212)
    if(bar):
      pl.bar(0,data[4][count], color='b')
      pl.bar(1,data[5][count], color='g')
      pl.bar(2,data[6][count], color='r')
      pl.axis([0,3,-5000,5000])
    elif(smooth):
      pl.plot(T, spline(t,data[4],T), 'b')
      pl.plot(T, spline(t,data[5],T), 'g')
      pl.plot(T, spline(t,data[6],T), 'r')
      pl.axis([0,num_p,-5000,5000])
    else:
      pl.plot(t,data[4],'b')
      pl.plot(t,data[5],'g')
      pl.plot(t,data[6],'r')
      pl.axis([0,num_p,-5000,5000])
    pl.title('Accelerometer')
    pl.ylabel('mg')

    # Refresh graphs
    pl.draw()
    pl.clf()
    # update count
    if(count < num_p):
      count = count + 1

ser.close()
