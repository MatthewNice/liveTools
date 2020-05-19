#toy example that shows a live plot of incoming data if connected to rav4 with panda
#if not connected to a car, it will just plot randomly generated points
#Author: Matthew Nice

from panda import Panda
import binascii
import bitstring
import time
import datetime
import csv
import cantools
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import liveTools as lt

#direct the database function to your DBC file
db2 = lt.database('RAV4.dbc')
p = lt.connectPanda()

TURN_SIGNALS = 1556
RADAR = [385,386,387,388,389,390,391,392,393,394,395,396,397,398,399]
GEAR = 956

colors = np.random.rand(1000,3)#random colors for plots

tstart = time.time() #reference start time
num_plots = 0

width = 7 #how many meters left and right
depth = 15 #how many meters longitude
#ask for size of FOV if you'd like
width = int(input('Width: '))
depth = int(input('Depth: '))

#initialize variables
lat = 0
lon = 0
lat100 = [0]*30
lon100 = [0]*30
brake = 0
gear = ''
turnSignal = ''
steer = 0
leadDist = 0


#initialize figure
fig,ax = plt.subplots()
fig.suptitle('Live Radar')
fig.canvas.draw()
ax.set_xlim([-width,width])
ax.set_ylim([0,depth])
plt.show(block=False)
line, = ax.plot(lat,lon,marker='.',markersize = 15)
line2, = ax.plot(lat100[-15:-2],lon100[-15:-2],marker='.',ls = '',markersize = 10)
line3, = ax.plot(lat100[-30:-15],lon100[-30:-15],marker='.',ls='',markersize = 5)

try:
    while True:
        if p != 0:
            can_recv = p.can_recv()
            for addr, _, msg, src  in can_recv:
                if addr in RADAR: #get data and timestamp of radar measurement
                    #if that decoded radar message fits all of the .lon .lat
                    lat = list(db2.decode_message(addr,msg).values())[2] #latitude
                    lon = list(db2.decode_message(addr,msg).values())[1] #longitude
                    lat100.append(lat)
                    lon100.append(lon)
                if addr == TURN_SIGNALS:
                    turnSignal = list(db2.decode_message(addr,msg).values())[0]
                if addr == 166:
                    brake = list(db2.decode_message(addr,msg).values())[1]
                if addr == GEAR:
                    gear = list(db2.decode_message(addr,msg).values())[0]
                if addr == 37:
                    steer =  list(db2.decode_message(addr,msg).values())[0]
                if addr == 869:
                    leadDist = list(db2.decode_message(addr,msg).values())[6]
        elif p == 0:
            #fake data added when not connected to a vehicle
            lon = np.random.randint(10)
            lat = np.random.randint(10)
            lat100.append(lat)
            lon100.append(lon)

            turnSignal = 'You''re'
            brake = 'getting'
            gear = 'no'
            steer = 'data'

        if abs(lat) < width and lon < depth:
            lat100.append(lat)
            lon100.append(lon)
            # print(round(lat,1),round(lon,1),end = '\r')
            if len(lat100) > 50:
                lat100 = lat100[-50:]
                lon100 = lon100[-50:]

        # line, = ax.plot(lat,lon,marker='.',markersize = 15)
        line.set_xdata(lat)
        line.set_ydata(lon) #USING set_data IS FASTER than plt.plot()
        # line2, = ax.plot(lat100[-15:-2],lon100[-15:-2],marker='.',ls = '',markersize = 10)
        line2.set_xdata(lat100[-15:-2])
        line2.set_ydata(lon100[-15:-2])
        # line3, = ax.plot(lat100[-50:-15],lon100[-50:-15],marker='.',ls='',markersize = 5)
        line3.set_xdata(lat100[-50:-15])
        line3.set_ydata(lon100[-50:-15])

        ax.set_xlim([-width,width])
        ax.set_ylim([0,depth])
        ax.draw_artist(ax.patch)
        ax.draw_artist(line)
        ax.draw_artist(line2)
        ax.draw_artist(line3)
        fig.canvas.update()
        fig.canvas.flush_events()

        num_plots += 1
        fps = (num_plots/(time.time()-tstart))
        if fps < 15:
            plt.cla()
            tstart = time.time()
            num_plots = 0

        print('Floor Lead Dist: ',leadDist,'Turn Signal: ', turnSignal,'Brake: ', brake,'Steering: ',steer, end='\r')
except KeyboardInterrupt:
    print('\n Logging Ended')
