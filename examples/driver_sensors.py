#toy example showing the use of live streaming data from a Comma Panda when connected to a car
#shows turn signal, brake, geat, and steering -- all save sensors to play with in the driver's seat
import liveTools as lt

p = lt.connectPanda()
turnSignal=''
brake = 0
gear = ''
steer = 0

try:
    while True:
        if p != 0:

            can_recv = p.can_recv()
            if can_recv != []:
                if lt.getTurnSignal(can_recv) != None:
                    turnSignal = lt.getTurnSignal(can_recv)
                if lt.getBrake(can_recv) != None:
                    brake = lt.getBrake(can_recv)
                if lt.getGear(can_recv) != None:
                    gear = lt.getGear(can_recv)
                if lt.getSetSpeed(can_recv) != None:
                    set_speed = lt.getSetSpeed(can_recv)
        # x = lt.liveTest(p)
            # print(x)
            # if steer != 0:
                print('Turn Signal: ',turnSignal,'Brake: ',brake,'Set Speed (kph): ', set_speed,end='\r')
        else:
            print('No device connected.')
            break
except KeyboardInterrupt:
    print('\n Session Ended')
