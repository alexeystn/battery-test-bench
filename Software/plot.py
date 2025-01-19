import numpy as np
from matplotlib import pyplot as plt

filenames = ['old_2w.txt', 'old_4b.txt', 'new_1w.txt', 'new_3b.txt']


for filename in filenames:

    d = { 'time': [],  'pwm': [], 'voltage': [], 'current': [], 'power': []}


    with open(filename, 'r') as f:
        for line in f:
            line = line.split()
            d['time'].append(int(line[3]))
            d['pwm'].append(int(line[4]))
            d['voltage'].append(float(line[5]))
            d['current'].append(float(line[6]))
            d['power'].append(float(line[7]))


    for k in d:
        d[k] = np.array(d[k])

    d['time'] = d['time'] / 1000


    i1 = np.where(np.diff(d['pwm']) > 0)[0][0] + 1
    i2 = np.where(np.diff(d['pwm']) < -10)[0][0] + 2

    i2 += 100
    
    for k in d:
        d[k] = np.array(d[k][i1:i2])

#    dischTime = d['time'][i1:i2]
#    dischCurrent = d['current'][i1:i2]
#    dischVoltage = d['voltage'][i1:i2]
#    dischPower = d['power'][i1:i2]

    #n = 40
    #offsetCurrent1 = d['current'][i1-n:i1]
    #offsetCurrent2 = d['current'][i2:i2+n]
    #offset1 = np.sum(offsetCurrent1) / n
    #offset2 = np.sum(offsetCurrent2) / n
    #print('Offsets: {0:.4f}-{1:.4f}'.format(offset1, offset2))

    #offset = np.linspace(offset1, offset2, len(dischTime))

    #dischCurrent = (dischCurrent - offset) * 1000 / 100

    AmpHours = np.sum(np.diff(d['time']) * d['current'][1:]) / 3600

    print('{0}: {1:.2f}'.format(filename, AmpHours*1000))


    plt.plot(d['time'], d['voltage'])
    #plt.plot(d['time'], d['current'])
    #plt.plot(d['time'], d['power'])
    #plt.plot(d['time'], d['pwm'])

plt.legend(filenames)

plt.grid(True)
plt.show()


