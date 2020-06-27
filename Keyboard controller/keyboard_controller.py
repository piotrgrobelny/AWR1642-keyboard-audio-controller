
from threading import Lock, Thread
import serial.tools.list_ports
import serial
import numpy as np
import keyboard
import time
import sys

#set keys that you want to trigg by radar
key = ["a","s","d","f","g","h","j","k","l","z"]

configFileName = '1642config.cfg'
move_lock = Lock()
hb100_data = ''
distance = 0
x = []
y = []

def configureSerialHB100():
    #connect automatically to arduino board
    #to work on linux just set up proper port_name
    port_name = ''
    serialPort = ''
    ports = serial.tools.list_ports.comports()
    #make list of available ports and search for arduino name
    for port, desc, hwid in sorted(ports):
        if "CH340" in desc:
            port_name = str(port)
    try:
        serialPort = serial.Serial(port_name, 9600)
        if serialPort == None: sys.exit(1)
    except:
        print("Change name of your arduino board in configureSerialHB100 from CH340 to one that is in the device menager")
    return serialPort

def Read_DataHB100(serialPort,):
    #read data from HB100 and write it with lock to hb100_data
    print ("Start reading data from HB100")
    global hb100_data
    serialPort.timeout = 1.0
    while True:
        b = serialPort.readline()
        string_n = b.decode("utf-8", "ignore")  # decode to unicode
        string = string_n.rstrip()  # strip from /n
        with move_lock:
            hb100_data = string

    serialPort.close()
    print ("Read_Data finished.")

def parseConfigFile(configFileName):
    configParameters = {} # Initialize an empty dictionary to store the configuration parameters

    # Read the configuration file and send it to the board
    config = [line.rstrip('\r\n') for line in open(configFileName)]
    for i in config:

        # Split the line
        splitWords = i.split(" ")

        # Hard code the number of antennas, change if other configuration is used
        numRxAnt = 4
        numTxAnt = 2

        # Get the information about the profile configuration
        if "profileCfg" in splitWords[0]:
            startFreq = int(float(splitWords[2]))
            idleTime = int(splitWords[3])
            rampEndTime = float(splitWords[5])
            freqSlopeConst = float(splitWords[8])
            numAdcSamples = int(splitWords[10])
            numAdcSamplesRoundTo2 = 1;

            while numAdcSamples > numAdcSamplesRoundTo2:
                numAdcSamplesRoundTo2 = numAdcSamplesRoundTo2 * 2;

            digOutSampleRate = int(splitWords[11]);

        # Get the information about the frame configuration
        elif "frameCfg" in splitWords[0]:

            chirpStartIdx = int(splitWords[1]);
            chirpEndIdx = int(splitWords[2]);
            numLoops = int(splitWords[3]);
            numFrames = int(splitWords[4]);
            framePeriodicity = int(splitWords[5]);

    # Combine the read data to obtain the configuration parameters
    numChirpsPerFrame = (chirpEndIdx - chirpStartIdx + 1) * numLoops
    configParameters["numDopplerBins"] = numChirpsPerFrame / numTxAnt
    configParameters["numRangeBins"] = numAdcSamplesRoundTo2
    configParameters["rangeResolutionMeters"] = (3e8 * digOutSampleRate * 1e3) / (
                2 * freqSlopeConst * 1e12 * numAdcSamples)
    configParameters["rangeIdxToMeters"] = (3e8 * digOutSampleRate * 1e3) / (
                2 * freqSlopeConst * 1e12 * configParameters["numRangeBins"])
    configParameters["dopplerResolutionMps"] = 3e8 / (
                2 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * configParameters["numDopplerBins"] * numTxAnt)
    configParameters["maxRange"] = (300 * 0.9 * digOutSampleRate) / (2 * freqSlopeConst * 1e3)
    configParameters["maxVelocity"] = 3e8 / (4 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * numTxAnt)

    return configParameters

def Read_DataAWR(Dataport, configParameters):
    print("Start reading data from AWR")
    global byteBuffer, byteBufferLength, hb100_data, high, key
    byteBuffer = np.zeros(2 ** 15, dtype='uint8')
    byteBufferLength = 0;
    # Constants
    MMWDEMO_UART_MSG_DETECTED_POINTS = 1;
    maxBufferSize = 2 ** 15;
    magicWord = [2, 1, 4, 3, 6, 5, 8, 7]

    # Initialize variables
    magicOK = 0  # Checks if magic number has been read
    detObj = {}
    tlv_type = 0

    #Detect falling edge on HB100
    high = True

    while True:
        readBuffer = Dataport.read(Dataport.in_waiting)
        byteVec = np.frombuffer(readBuffer, dtype='uint8')
        byteCount = len(byteVec)

        # Check that the buffer is not full, and then add the data to the buffer
        if (byteBufferLength + byteCount) < maxBufferSize:
            byteBuffer[byteBufferLength:byteBufferLength + byteCount] = byteVec[:byteCount]
            byteBufferLength = byteBufferLength + byteCount

        # Check that the buffer has some data
        if byteBufferLength > 16:

            # Check for all possible locations of the magic word
            possibleLocs = np.where(byteBuffer == magicWord[0])[0]

            # Confirm that is the beginning of the magic word and store the index in startIdx
            startIdx = []
            for loc in possibleLocs:
                check = byteBuffer[loc:loc + 8]
                if np.all(check == magicWord):
                    startIdx.append(loc)

            # Check that startIdx is not empty
            if startIdx:

                # Remove the data before the first start index
                if startIdx[0] > 0 and startIdx[0] < byteBufferLength:
                    byteBuffer[:byteBufferLength - startIdx[0]] = byteBuffer[startIdx[0]:byteBufferLength]
                    byteBuffer[byteBufferLength - startIdx[0]:] = np.zeros(len(byteBuffer[byteBufferLength - startIdx[0]:]),
                                                                           dtype='uint8')
                    byteBufferLength = byteBufferLength - startIdx[0]

                # Check that there have no errors with the byte buffer length
                if byteBufferLength < 0:
                    byteBufferLength = 0

                # word array to convert 4 bytes to a 32 bit number
                word = [1, 2 ** 8, 2 ** 16, 2 ** 24]

                # Read the total packet length
                totalPacketLen = np.matmul(byteBuffer[12:12 + 4], word)

                # Check that all the packet has been read
                if (byteBufferLength >= totalPacketLen) and (byteBufferLength != 0):
                    magicOK = 1

        # If magicOK is equal to 1 then process the message
        if magicOK:
            # word array to convert 4 bytes to a 32 bit number
            word = [1, 2 ** 8, 2 ** 16, 2 ** 24]

            # Initialize the pointer index
            idX = 0

            # Read the header
            magicNumber = byteBuffer[idX:idX + 8]
            idX += 8
            version = format(np.matmul(byteBuffer[idX:idX + 4], word), 'x')
            idX += 4
            totalPacketLen = np.matmul(byteBuffer[idX:idX + 4], word)
            idX += 4
            platform = format(np.matmul(byteBuffer[idX:idX + 4], word), 'x')
            idX += 4
            frameNumber = np.matmul(byteBuffer[idX:idX + 4], word)
            idX += 4
            timeCpuCycles = np.matmul(byteBuffer[idX:idX + 4], word)
            idX += 4
            numDetectedObj = np.matmul(byteBuffer[idX:idX + 4], word)
            idX += 4
            numTLVs = np.matmul(byteBuffer[idX:idX + 4], word)
            idX += 4
            subFrameNumber = np.matmul(byteBuffer[idX:idX + 4], word)
            idX += 4

            # Read the TLV messages
            for tlvIdx in range(numTLVs):

                # word array to convert 4 bytes to a 32 bit number
                word = [1, 2 ** 8, 2 ** 16, 2 ** 24]

                # Check the header of the TLV message
                try:
                    tlv_type = np.matmul(byteBuffer[idX:idX + 4], word)
                    idX += 4
                    tlv_length = np.matmul(byteBuffer[idX:idX + 4], word)
                    idX += 4
                except:
                    pass

                # Read the data depending on the TLV message
                if tlv_type == MMWDEMO_UART_MSG_DETECTED_POINTS:

                    # word array to convert 4 bytes to a 16 bit number
                    word = [1, 2 ** 8]
                    tlv_numObj = np.matmul(byteBuffer[idX:idX + 2], word)
                    idX += 2
                    tlv_xyzQFormat = 2 ** np.matmul(byteBuffer[idX:idX + 2], word)
                    idX += 2

                    # Initialize the arrays
                    rangeIdx = np.zeros(tlv_numObj, dtype='int16')
                    dopplerIdx = np.zeros(tlv_numObj, dtype='int16')
                    peakVal = np.zeros(tlv_numObj, dtype='int16')
                    x = np.zeros(tlv_numObj, dtype='int16')
                    y = np.zeros(tlv_numObj, dtype='int16')
                    z = np.zeros(tlv_numObj, dtype='int16')

                    for objectNum in range(tlv_numObj):
                        # Read the data for each object
                        rangeIdx[objectNum] = np.matmul(byteBuffer[idX:idX + 2], word)
                        idX += 2
                        dopplerIdx[objectNum] = np.matmul(byteBuffer[idX:idX + 2], word)
                        idX += 2
                        peakVal[objectNum] = np.matmul(byteBuffer[idX:idX + 2], word)
                        idX += 2
                        x[objectNum] = np.matmul(byteBuffer[idX:idX + 2], word)
                        idX += 2
                        y[objectNum] = np.matmul(byteBuffer[idX:idX + 2], word)
                        idX += 2
                        z[objectNum] = np.matmul(byteBuffer[idX:idX + 2], word)
                        idX += 2

                    # Make the necessary corrections and calculate the rest of the data
                    rangeVal = rangeIdx * configParameters["rangeIdxToMeters"]
                    dopplerIdx[dopplerIdx > (configParameters["numDopplerBins"] / 2 - 1)] = dopplerIdx[dopplerIdx > (
                                configParameters["numDopplerBins"] / 2 - 1)] - 65535
                    dopplerVal = dopplerIdx * configParameters["dopplerResolutionMps"]
                    x = x / tlv_xyzQFormat
                    y = y / tlv_xyzQFormat
                    z = z / tlv_xyzQFormat

                    # Store the data in the detObj dictionary
                    detObj = {"numObj": tlv_numObj, "rangeIdx": rangeIdx, "range": rangeVal, "dopplerIdx": dopplerIdx, \
                              "doppler": dopplerVal, "peakVal": peakVal, "x": x, "y": y, "z": z}

            # Remove already processed data
            try:
                if idX > 0 and byteBufferLength > idX:
                    shiftSize = totalPacketLen

                    byteBuffer[:byteBufferLength - shiftSize] = byteBuffer[shiftSize:byteBufferLength]
                    byteBuffer[byteBufferLength - shiftSize:] = np.zeros(len(byteBuffer[byteBufferLength - shiftSize:]),
                                                                         dtype='uint8')
                    byteBufferLength = byteBufferLength - shiftSize

                    # Check that there are no errors with the buffer length
                    if byteBufferLength < 0:
                        byteBufferLength = 0
            except:
                pass

            #update plot
            update_plot(detObj)

            ##################################################################################
            #Here is keyboard trigger part

            #read information from hb100
            with move_lock:
                move = hb100_data

            try:
                distance = int(100 * limit_area(detObj)) #calculate distance from object in centimeter
                if "1" in move:
                    high = True
                if "0" in move and high == True: #if falling edge
                    high = False
                    #if HB100 detect motion, trigger proper key
                    if distance < 8:
                        keyboard.press_and_release(key[0])
                        print(key[0])
                    elif distance >= 8 and distance < 16:
                        keyboard.press_and_release(key[1])
                        print(key[1])
                    elif distance >= 16 and distance < 24:
                        keyboard.press_and_release(key[2])
                        print(key[2])
                    elif distance >= 24 and distance < 32:
                        keyboard.press_and_release(key[3])
                        print(key[3])
                    elif distance >= 32 and distance < 40:
                        keyboard.press_and_release(key[4])
                        print(key[4])
                    elif distance >= 40 and distance < 48:
                        keyboard.press_and_release(key[5])
                        print(key[5])
                    elif distance >= 48 and distance < 56:
                        keyboard.press_and_release(key[6])
                        print(key[6])
                    elif distance >= 64 and distance < 72:
                        keyboard.press_and_release(key[7])
                        print(key[7])
                    elif distance >= 72 and distance < 80:
                        keyboard.press_and_release(key[8])
                        print(key[8])
                    elif distance >= 88 and distance < 96:
                        keyboard.press_and_release(key[9])
                        print(key[9])
                    elif distance >= 96:
                        keyboard.press_and_release(key[10])
                        print(key[10])
            except:
                pass

def update_plot(detObj):
    global x, y
    #write data to x,y lists, application_plotting will read them to update plot
    try:
        if len(detObj['x']) > 0:
            x = -detObj['x']
            y = detObj['y']
    except:
        pass

def configureSerialAWR():
    serial_port_1 = ''
    serial_port_2 = ''
    # Check which ports are used by awr1642
    ports = serial.tools.list_ports.comports()
    for port, desc, hwid in sorted(ports):
        if "Application/User" in desc:
            serial_port_1 = str(port)
        elif "Data" in desc:
            serial_port_2 = str(port)

    CLIport = serial.Serial(serial_port_1, 115200)
    if CLIport == None: sys.exit(1)

    Dataport = serial.Serial(serial_port_2, 921600)
    if Dataport == None: sys.exit(1)

    # Read the configuration file and send it to the board
    config = [line.rstrip('\r\n') for line in open(configFileName)]
    for i in config:
        CLIport.write((i + '\n').encode())
        print(i)
        time.sleep(0.01)

    return Dataport

def limit_area(detObj):
    global distance
    #calculate distance from points in selected area
    try:
        a = detObj["y"]
        b = detObj["x"]

        limit_x = np.where(b > 0.15) and np.where(b < -0.15)
        a = np.delete(a, limit_x)

        limit_y = np.where(a > 1) or np.where(a < 0.02)
        a = np.delete(a, limit_y)
    except:
        pass
    try:
        distance = np.sort(a)[0]
    except:
        distance = 0

    return distance

def main():
    #make configuration before running threads
    configParameters = parseConfigFile(configFileName)

    p1 = Thread(target=Read_DataHB100, args=(configureSerialHB100(), ))
    p2 = Thread(target=Read_DataAWR, args=(configureSerialAWR(), configParameters))
    p1.start()
    p2.start()

if __name__ == "__main__":
    main()



