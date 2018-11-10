import struct
import time
import os
import pty

from collections import deque
from multiprocessing import Process

import matplotlib.pyplot as plt
import numpy as np
import serial

class Satellite:
    def __init__(self,
                 serialport='/dev/cu.wchusbserial1420',
                 baudrate=9600,
                 numread=30,
                 packsize=54,
                 frequency=30,
                 syncbyte1=b'FFFFFFFF',
                 syncbyte2=b'FFFF',
                 connection_timeout=5,
                 first_byte = b'A',
                 plotting=True,
                 plotsize=30,
                 emulated_serial=False):

        self.serialport = serialport
        self.baudrate = baudrate
        self.numread = numread
        self.packsize = packsize
        self.frequency = frequency
        self.syncbyte1 = syncbyte1
        self.syncbyte2 = syncbyte2
        self.timeout = connection_timeout
        self.first_byte = first_byte
        self.arduino = None
        self.plotting = plotting
        self.plotsize = plotsize
        self.r = 0

        self.emulated_serial = emulated_serial

        # Voltage data of 6 channels
        self.data = [deque(list(np.zeros(self.plotsize)),maxlen=self.plotsize) for x in range(0,11)]
        self.x_time = deque([x/self.frequency for x in range(self.plotsize)],maxlen=self.plotsize)

    def emulate_connection(self):
        self.emulated_serial = True

        master, slave = pty.openpty()
        s_name = os.ttyname(slave)
        self.arduino = serial.Serial(s_name)
        self.emulated_master = master
        print("Emulated connection established")

        data = b'ffff0494ffffFE70ffff3AC0ffffFF21ffffFF9BffffFF4Affff0005ffff0017ffff6844'
        print(int('0494', 16))

        self.arduino.write(data)


    def establish_connection(self):
        try:
            self.arduino = serial.Serial(self.serialport, self.baudrate)
            print('Serial port found. Trying to establish connection...')
        except Exception as e:
            print('Could not find serial port: error {}'.format(e))
            return None

        time_elapsed = 0
        connection_established = False
        first_byte = None

        t = time.time()
        while time_elapsed < self.timeout:

            time_elapsed = time.time() - t

            if self.arduino.inWaiting():
                first_byte = self.arduino.read(1)
                connection_established = True
                time_elapsed = self.timeout + 1

        if connection_established:
            print('Connection established in {} seconds.'.format(np.round(time.time() - t, 2)))
            return self.arduino
        else:
            print('Connection failed.')
            self.arduino.close()
            return None

    def clean_port(self):
        while self.arduino.inWaiting() >= 2 * self.packsize:
            self.read_pack()
        print('Serial port is cleaned')

    def typecast_swap_float(self, arr):
        '''
        Converts uint8 array to uint16 value and swap bytes.
        Returns float value
        '''
        arr = np.flip(np.uint8(arr),0)
        result = float(np.uint16(arr[1]/255*65535-arr[1]+arr[0]))
        return result

    def datasync(self, A):
        '''
        Resolve serial data flow desync
        '''
        print("Starting sync: {}".format(A))
        sb1_index = A.index(self.syncbyte1)
        # B = struct.unpack('{}B'.format(sb1_index),self.arduino.read(sb1_index))
        B = self.arduino.read(sb1_index)
        A = A[sb1_index:] + B
        print("Synchronization is done: {}".format(A))
        return(A)

    def read_pack(self):

        if self.emulated_serial:
            A = struct.unpack('{}B'.format(self.packsize),os.read(self.emulated_master,self.packsize))
            print("Data uint8 {}".format(A))
            A = self.typecast_swap_float(A[1:3])
            return(A)

        A = self.arduino.read(self.packsize)

        # Checks if what we just read is valid data, if desync resolve it
        while True:
            if self.syncbyte1 in A and A[-1] != 70:
                break
            A = self.arduino.read(8)
            A = self.arduino.read(self.packsize)

        if A[0:8] != self.syncbyte1 or A[-1] == 70:
            A = self.datasync(A)

        return A

    @staticmethod
    def convert_s16(value):
        return -(value & 0x8000) | (value & 0x7fff)

    def read_packs(self, num = None):
        if num == None:
            num = self.numread

        if self.arduino.inWaiting() >= num * self.packsize:
            for i in range(self.numread):
                A = self.read_pack()
                mult = 2**14
                Data = [int(A[i:i + 4],16) for i in range(0, len(A) - 4, 4)]
                _, _, ax, ay, az, gx, gy, gz, t, h, l, rb, r = Data
                print(A)
                print("Temp is {}, byte: {}".format(t,A[32:36]))

                gx = self.convert_s16(gx)
                gy = self.convert_s16(gy)
                gz = self.convert_s16(gz)

                self.data[0].append(ax/mult) # Acc_x
                self.data[1].append(ay/mult) # Acc_y
                self.data[2].append(az/mult) # Acc_z
                self.data[3].append(gx) # GX
                self.data[4].append(gy) # GY
                self.data[5].append(gz) # GZ
                self.data[6].append(t) # Temperature
                self.data[7].append(h) # Humidity
                self.data[8].append(l) # Light
                self.data[9].append(rb) # RAD_BOOL
                print(rb)
                if (rb != 0):
                    self.r = r
                self.data[10].append(self.r)  # Radiation

        return self.data

    def realtime_emg(self):
        '''
        Realtime reading and processing of data from Arduino electrodes
        '''
        plotsize = self.frequency
        # Plot initialization
        self.plot_init()

        p = Process(target=self.plot_update())

        while True:
            self.read_packs()

            if not self.plot_update():
                # Stop proccess when plot is closed by user
                break

        self.arduino.close()
        print('Connection closed.')

    def plot_init(self):
        self.fig = plt.figure()

        # Voltage channels plot
        self.ax = self.fig.add_subplot(321)
        self.ax.set_xlim(0,1)
        self.ax.set_xlabel('Time')
        self.ax.set_ylabel("Acceleration")
        self.ax.set_title("Acceleration Sensor")
        self.ax.grid()
        self.ax.ticklabel_format(axis='both', style='plain')

        self.ch1, = self.ax.plot(self.x_time, self.data[0], '-b', label ='Acceleration X', linewidth = 0.5)
        self.ch2, = self.ax.plot(self.x_time, self.data[1], '--r', label ='Acceleration Y', linewidth = 0.5)
        self.ch3, = self.ax.plot(self.x_time, self.data[2], '--g', label ='Acceleration Z', linewidth = 0.5)
        self.ax.legend(loc='upper left')

        # Gyroscope channels plot
        self.g = self.fig.add_subplot(322)
        self.g.set_xlim(0, 1)
        self.g.set_xlabel('Time')
        self.g.set_ylabel("Gyroscope")
        self.g.set_title("Gyroscope Sensor")
        self.g.grid()
        self.g.ticklabel_format(axis='both', style='plain')

        self.gx, = self.g.plot(self.x_time, self.data[3], '-b',  label='Gyroscope X', linewidth=0.5)
        self.gy, = self.g.plot(self.x_time, self.data[4], '--r', label='Gyroscope Y', linewidth=0.5)
        self.gz, = self.g.plot(self.x_time, self.data[5], '--g', label='Gyroscope Z', linewidth=0.5)
        self.g.legend(loc='upper left')

        # Temperature
        self.tempfig = self.fig.add_subplot(323)
        self.tempfig.set_xlim(0, 1)
        self.tempfig.set_xlabel('Time')
        self.tempfig.set_ylabel("Temperature")
        self.tempfig.set_title("Temperature Sensor")
        self.tempfig.grid()
        self.tempfig.ticklabel_format(axis='both', style='plain')

        self.tempplot, = self.tempfig.plot(self.x_time, self.data[6], '-b', label='Temperature', linewidth=0.5)
        self.tempfig.legend(loc='upper left')

        # Humidity
        self.hum = self.fig.add_subplot(324)
        self.hum.set_xlim(0, 1)
        self.hum.set_xlabel('Time')
        self.hum.set_ylabel("Humidity")
        self.hum.set_title("Humidity Sensor")
        self.hum.grid()
        self.hum.ticklabel_format(axis='both', style='plain')

        self.humplot, = self.hum.plot(self.x_time, self.data[7], '-b', label='Humidity', linewidth=0.5)
        self.hum.legend(loc='upper left')

        # Light
        self.light = self.fig.add_subplot(325)
        self.light.set_xlim(0, 1)
        self.light.set_xlabel('Time')
        self.light.set_ylabel("Light")
        self.light.grid()
        self.light.ticklabel_format(axis='both', style='plain')

        self.lplot, = self.light.plot(self.x_time, self.data[8], '-b', label='Light', linewidth=0.5)
        self.light.legend(loc='upper left')

        # Light
        self.rad = self.fig.add_subplot(326)
        self.rad.set_xlim(0, 1)
        self.rad.set_xlabel('Time')
        self.rad.set_ylabel("Radiation")
        self.rad.grid()
        self.rad.ticklabel_format(axis='both', style='plain')

        self.rplot, = self.rad.plot(self.x_time, self.data[10], '-b', label='Radiation', linewidth=0.5)
        self.rad.legend(loc='upper left')

        if self.plotting:
            print('Plot initialized')
            self.fig.show()

        return True

    def plot_update(self):
        if self.plotting:
            # Send all new data to plot
            self.ch1.set_ydata(self.data[0])
            self.ch2.set_ydata(self.data[1])
            self.ch3.set_ydata(self.data[2])
            self.ax.relim()
            self.ax.autoscale_view()

            self.gx.set_ydata(self.data[3])
            self.gy.set_ydata(self.data[4])
            self.gz.set_ydata(self.data[5])
            self.g.relim()
            self.g.autoscale_view()

            self.tempplot.set_ydata(self.data[6])
            self.tempfig.relim()
            self.tempfig.autoscale_view()

            self.humplot.set_ydata(self.data[7])
            self.hum.relim()
            self.hum.autoscale_view()

            self.lplot.set_ydata(self.data[8])
            self.light.relim()
            self.light.autoscale_view()

            self.rplot.set_ydata(self.data[10])
            self.rad.relim()
            self.rad.autoscale_view()

            # Checks if there are too many packets left in serial, e.g. if speed of processing is fast enough
            packets_inwaiting = self.inwaiting()
            if packets_inwaiting >= 50:
                print('Update rate is slow: {} packets inwaiting, {} second delay.'.format(packets_inwaiting, np.round(packets_inwaiting/256,2)))

            try:
                # Catches error when plot is closed by user
                plt.pause(0.0001)
            except:
                return False

            return True
        else:
            return False

    def __enter__(self):
        if self.emulated_serial:
            self.emulate_connection()
        else:
            self.establish_connection()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        try:
            self.arduino.close()
        except:
            print("Connection is already closed or doesn't exist")

    def inwaiting(self):
        return int(np.round(self.arduino.inWaiting()/self.packsize))


if __name__ == '__main__':
    with Satellite('/dev/cu.usbmodem14101', numread=1, plotting=True, packsize=54) as satellite:
        satellite.realtime_emg()
