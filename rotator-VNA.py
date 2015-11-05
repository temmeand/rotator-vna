# Control the rotator and download data from a VNA

# Andrew Temme
# temmeand@gmail.com
# updated 2015-11-4

# currently written for HP 8753

from __future__ import division
import visa
import numpy as np
from datetime import datetime
from os import path, makedirs
import serial
from time import sleep
from math import trunc
# import skrf

#------------------------------------------------------------------------------
#------------------------------------------------------------------------------
#------------------------------------------------------------------------------
#------------------------------------------------------------------------------
def openRotatorPort(portNum=0, timeout=5):
    """
    Open a serial port for the rotator

    Open commport ``portNum`` with a timeout of ``timeout``.

    Parameters
    ----------
    portNum : integer, default: 0
        commport for the serial connection to the rotator
    timeout : number, default: 5 sec
        timeout for the commport

    Returns
    -------
    port : serial port object thingy
        object that you use to communicate now with the serial port.
    """

    ser = serial.Serial(0,timeout=5)
    
    return ser

def advanceRotator(deg):
    """
    Advance the rotator ``deg`` degrees

    Parameters
    ----------
    deg : number
        how many degrees to advance the rotator

    Returns
    -------
    """
    print "write this fuction"
    stepsPerRotation = 508000
    gearRatio = 6
    stepsPerDeg = stepsPerRotation * gearRatio
    steps = stepsPerDeg * deg

def startRotatorTurning(rpm):
    """
    Start the rotator turning at a set RPM

    Parameters
    ----------
    rpm : number
        RPM for the rotator

    Return
    ------
    """
    # gear ratio for rotator in the antenna chamber and arch range
    gearRatio = 6

    # gear ratio for APS Design Competition turntable
    # gearRatio=2.446

    delay = 0.01

    print "start"

    speed=1/(rpm/60*400)/2.04e-6/gearRatio
    print speed
    speed = trunc(round(speed))
    print speed
    print str(speed)


    ser.write('BD0\r')
    sleep(delay)
    ser.write('SH\r')
    sleep(delay)
    ser.write('SO\r')
    sleep(delay)
    ser.write('SA10\r')
    sleep(delay)
    ser.write('SM2000\r')
    sleep(delay)
    ser.write('SD'+str(speed)+'\r')
    sleep(delay)
    ser.write('H+\r')


def stopRotator():
    """
    Stop the rotator
    """

    ser.write('H0\r')
    ser.close()
    print "Port closed"


#------------------------------------------------------------------------------
#------------------------------------------------------------------------------
#------------------------------------------------------------------------------
#------------------------------------------------------------------------------

print "start"

# open the rotator
openRotatorPort(0, 5)

# start the rotator
# startRotatorTurning(16)

#------------------------------------------------------------------------------
dataPrefix = 'carpet-foam'
#------------------------------------------------------------------------------

# number of averages
numGrps = 4

# GPIB address for the VNA set below
ena = visa.instrument('GPIB::16', timeout = 120)
idn =  ena.ask('*IDN?')
print idn

optLine = "# Hz S RI R 50"

cmd8753D = {\
'basicInit':'HOLD;DUACOFF;CHAN1;S11;LOGM;CONT;AUTO',\
'corrQ':'CORR?',\
'freqSpanQ':'SPAN?',\
'freqStartQ':'STAR?',\
'freqStopQ':'STOP?',\
'getImag':'IMAG;OUTPFORM',\
'getLinMag':'LINM;OUTPFORM',\
'getLogMag':'LOGM;OUTPFORM',\
'getPhase':'PHAS;OUTPFORM',\
'getReal':'REAL;OUTPFORM',\
'hold':'HOLD',\
'IDStr':'HEWLETT PACKARD,8753D,0,6.14',\
'ifbwQ':'IFBW?',\
'numPtsQ':'POIN?',\
'powerQ':'POWE?',\
'preset':'PRES',\
'numGroups':'NUMG',\
'polar':'POLA',\
's11':'S11',\
's21':'S21',\
's12':'S12',\
's22':'S22'\
}

cmdDict = cmd8753D

#------------------------------------------------------------------------------
ena.write('form4')

# number of points
ena.write('POIN1601')

numPts = ena.ask_for_values(cmdDict['numPtsQ'])[0]
freqStart = ena.ask_for_values(cmdDict['freqStartQ'])[0]
freqStop = ena.ask_for_values(cmdDict['freqStopQ'])[0]
freq = np.linspace(freqStart,freqStop,num=numPts,endpoint=True)
ifbw = ena.ask_for_values(cmdDict['ifbwQ'])[0]
pwr = ena.ask_for_values(cmdDict['powerQ'])[0]
corr = ena.ask(cmdDict['corrQ'])

dateString = datetime.now().strftime("%Y-%m-%d")
timeString = datetime.now().strftime("%H:%M:%S")

dataDir = 'Data/' + dateString
if not path.exists(dataDir):
    makedirs(dataDir)

i = 0
#saveMeas = True
#for i in range(numMeasurements):
try:
    ena.write('pola;numg' + str(numGrps))
#    while saveMeas:    
    while True:

#------------------------------------------------------------------------------
#------------------------------------------------------------------------------
        # move the rotator
        # will need to decide about how to handle the first case. Do you advance
        # the rotator and then measure or do you measure and then advance? The
        # call to ``advanceRotator`` can be made at the end of the loop.
        advanceRotator(15)
#------------------------------------------------------------------------------
#------------------------------------------------------------------------------

        print('Starting Measurement Number: %d' % i)  
        
        # print "here"
        s11polar = np.array(ena.ask_for_values(cmdDict['s11']+cmdDict['polar']+';'+cmdDict['numGroups'] + str(numGrps)+';outpform'))
        # print "there"
        s11polReal = s11polar[::2]								# real values from the polar data
        s11polImag = s11polar[1::2]								# imaginary values from the polar data

        print "s21"
        s21polar = np.array(ena.ask_for_values(cmdDict['s21']+cmdDict['polar']+';'+cmdDict['numGroups'] + str(numGrps)+';outpform'))
        s21polReal = s21polar[::2]								# real values from the polar data
        s21polImag = s21polar[1::2]								# imaginary values from the polar data

        print "s12"
        s12polar = np.array(ena.ask_for_values(cmdDict['s12']+cmdDict['polar']+';'+cmdDict['numGroups'] + str(numGrps)+';outpform'))
        s12polReal = s12polar[::2]								# real values from the polar data
        s12polImag = s12polar[1::2]								# imaginary values from the polar data

        print "s22"
        s22polar = np.array(ena.ask_for_values(cmdDict['s22']+cmdDict['polar']+';'+cmdDict['numGroups'] + str(numGrps)+';outpform'))
        s22polReal = s22polar[::2]								# real values from the polar data
        s22polImag = s22polar[1::2]								# imaginary values from the polar data

        saveData = np.concatenate(([freq],
                                   [s11polReal],[s11polImag],
                                   [s21polReal],[s21polImag],
                                   [s12polReal],[s12polImag],
                                   [s22polReal],[s22polImag])).T

        
        timeAppend = datetime.now().strftime("-%Y-%m-%d-%H%M%S")
        dataName = dataDir + '/' + dataPrefix + timeAppend 
                
        touchFileName = dataName + ".s2p"    
        print touchFileName
        saveFile = open(touchFileName, "w")
        saveFile.write("!"+idn+"\n")
        saveFile.write("!Date: " + dateString + " " + timeString + "\n")
        saveFile.write("!Data & Calibration Information:\n")
        if corr == '0':
            saveFile.write("!Freq S11 S21 S12 S22\n")
        elif corr== '1':
            saveFile.write("!Freq S11:Cal(ON) S21:Cal(ON) S12:Cal(ON) S22:Cal(ON)\n")
        
        saveFile.write("!PortZ Port1:50+j0 Port2:50+j0\n")
        saveFile.write(("!Above PortZ is port z conversion or system Z0 "
                        "setting when saving the data.\n"))
        saveFile.write(("!When reading, reference impedance value at option "
                        "line is always used.\n"))
        saveFile.write("!\n")
        saveFile.write("!--Config file parameters\n")
        saveFile.write("!start = " + str(freqStart) + "\n")
        saveFile.write("!stop = " + str(freqStop) + "\n")
        saveFile.write("!numPts = " + str(numPts) + "\n")
        saveFile.write("!avgFact = " + str(numGrps) + "\n")
        saveFile.write("!power = " + str(pwr) + "\n")
        saveFile.write("!ifbw = " + str(ifbw) + "\n")
        saveFile.write("!\n")
        saveFile.write(optLine + "\n")
        np.savetxt(saveFile,saveData,delimiter=" ")
        saveFile.close()
        
        i += 1
        # balun = skrf.Network(touchFileName)
        # balun.plot_s_db()
        # legend(loc=0)
except KeyboardInterrupt:
    print('Ctrl-C Interrupt')

finally:
    print('Finally')

visa.vpp43.gpib_control_ren(ena.vi,2)
stopRotator()
print("Done")
