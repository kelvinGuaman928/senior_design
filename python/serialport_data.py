import serial
import pandas as pd
import scipy.io as sio


device = serial.Serial("COM5", baudrate = 115200)
mag_acc = []
mag_acc_2 = []
mag_gyro= []
mag_gyro_2=[]
'''
instead of checking what activity the user is preforming use binary output
falling = 1
notfalling = 0
'''





dict = {
    'magnitude_acc': mag_acc,'magnitude_acc2' :mag_acc_2,
    'magnitude_gyro':mag_gyro,'magnitude_gyro2': mag_gyro_2,
    'activity':downstairs



        }

while 1:
    while device.inWaiting() == 0:
        pass

    byte = device.readline()
    s = byte[0:-2].decode("utf-8")
    x = s.split(",")
    print(x)

    mag_acc.append(float(x[0]))
    mag_acc_2.append(float(x[1]))

    mag_gyro.append(float(x[2]))
    mag_gyro_2.append(float(x[3]))

    df = pd.DataFrame(dict)
    #sio.savemat('mag_data.mat',{'dict':dict})
    df.to_csv('mag_data.csv')
