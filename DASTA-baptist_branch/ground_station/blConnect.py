import socket
import time
import struct
import json
import asyncio
from concurrent.futures import ThreadPoolExecutor


STD_TYPE_KEY = ['c', 'i', 'Q', 'f', 'd']
VECTOR_KEY = 'v'
MATRIX_KEY = 'm'
VECTOR_CONTENT_TYPE_KEY = 'f'
VECTOR_CONTENT_TYPE_SIZE = 4

'''
_______________________________________________________
______________________CONNECTION__________________________
_______________________________________________________
'''
#connect to the Bluetooth device
def connect(address, port=1 , max_attempts=10):
    #disconeect the device if it was already connected
    try:
        s.close()
    except:
        pass
    
    print("Connecting to " + str(address) + " on port " + str(port) + "...")
    #create the socket
    s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    #connect to the server
    attempts = 0
    while attempts < max_attempts:
        try:
            s.connect((address, port))
            break
        except socket.error as e:
            print("Error: socket error, retrying...")
            print(e)
            attempts += 1
            time.sleep(1)
    if attempts == max_attempts:
        print("Error: can't connect to the server")
        return None
    
    print("Connected")
    return s

'''
_______________________________________________________
_____________________TRANSMISSIONS_____________________
_______________________________________________________
'''
END_LINE_KEY = "end_line\n".encode("utf-8")

#when we receive the header, we need to decode it to know how to decode the data
def decodeHeader(header):
    #decode the header witch is in ascii like "name:type:size, name:type:size, ..."
    try:
        header = header.decode("utf-8")
    except UnicodeDecodeError:
        print("Error: header is not in ascii")
        return None
    except AttributeError as e:
        print("Error: header is not a string")
        print(e)
        return None
    except Exception as e:
        print("Error: unknown error")
        print(e)
        return None

    #split the header by ","
    header = header.split(",")
    #remove empty string
    header = [header[i] for i in range(len(header)) if header[i] != '']
    #list of dict
    header_dict = [{"name":None, "type":None, "size":None} for i in range(len(header))]
    for i in range(len(header)):       
        header2 = header[i].split(":")
        if len(header2) < 3:
            print("Error: standard type need 3 arguments separated by ':' : " + str(header2))
            return None
        if header2[1] not in STD_TYPE_KEY and header2[1] != VECTOR_KEY and header2[1] != MATRIX_KEY:
            print("Error: unknown type : " + header2[1])
            return None
        if header2[1] == MATRIX_KEY and len(header2) != 4:
            print("Error: matrix type need 4 arguments separated by ':' : " + str(header2))
            return None
        if not header2[2].isdigit():
            print("Error: size must be an integer : " + str(header2))
            return None
        
        header_dict[i]["name"] = header2[0]
        header_dict[i]["type"] = str(header2[1])
        header_dict[i]["size"] = int(header2[2])
        if header2[1] == MATRIX_KEY:
            #add a "row" key to the dict
            header_dict[i]["row"] = int(header2[3])
    return header_dict


'''
_______________________________________________________
______________________SEND_____________________________
_______________________________________________________
'''
SEND_HEADER_KEY = "receive_stream:".encode("utf-8")
def getTypeKey(var):
    if type(var) == int:
        return 'i'
    elif type(var) == float:
        return 'f'
    elif type(var) == str:
        if len(var) == 1:
            return 'c'
        else:
            return 's'
    elif type(var) == list:
        oneElement = var[0]
        if type(oneElement) != list:
            return 'v'
        else:
            return 'm'
    else:
        print("Error: unknown type")
        return None
    
    
def packOneData(OneData, formatt):
    # print("packing : " + str(OneData) + " with format : " + str(formatt))
    #check if the type of the data is correct
    if getTypeKey(OneData) != formatt["type"]:
        print("Error: the type of the data is not correct")
        return None
    
    packedData = struct.pack(formatt["type"], OneData)
    if len(packedData) != formatt["size"]:
        print("Error: the size of the data is not correct")
        return None
    
    return packedData

def packData(data, header):
    #data is a dict
    #header is a list of dict
    
    packedData = b''
    send_register = 0
    for i in range(len(header)):
        #pack the data
        if header[i]["name"] in data:
            send_register += 1 << i
            packedData += packOneData(data[header[i]["name"]], header[i])
           
    return struct.pack("I", send_register) + packedData

send_head = None
def send(s, data, send_head):
    if send_head == None:
        print("Error: header not received yet, can't send the data")
        return None
    #pack the data
    packedData = packData(data, send_head)
    if packedData == None:
        return None
    #send the data
    to_send = packedData + END_LINE_KEY  
    s.sendall(to_send)    

fake_send_head = [
    {"name":"event code", "type":"c", "size":1},# event code, we should define a list of event code later (emergency stop (0), take off (1), land (2), ...)
    {"name":"posCommand", "type":"v", "size":16},# posCommand = [x, y, z, yaw]
    {"name":"newWaypoint", "type":"v", "size":20},# newWaypoint = [x, y, z, yaw, dt]
    {"name":"maxSpeed", "type":"f", "size":4},# maxSpeed
    {"name":"Xlimit", "type":"v", "size":8},# Xlimit = [min, max]
    {"name":"Ylimit", "type":"v", "size":8},# Ylimit = [min, max]
    {"name":"Zlimit", "type":"v", "size":8},# Zlimit = [min, max]
    {"name":"pidPosX", "type":"v", "size":12},# pidPosX = [kp, ki, kd]
    {"name":"pidPosY", "type":"v", "size":12},# pidPosY = [kp, ki, kd]
    {"name":"pidPosZ", "type":"v", "size":12},# pidPosZ = [kp, ki, kd]
    {"name":"pidYaw", "type":"v", "size":12},# pidYaw = [kp, ki, kd]
    {"name":"pidPitch", "type":"v", "size":12},# pidPitch = [kp, ki, kd]
    {"name":"pidRoll", "type":"v", "size":12},# pidRoll = [kp, ki, kd]
    {"name":"cam1Pos", "type":"v", "size":12},# cam1Pos = [x, y, z]
    {"name":"cam1Or", "type":"v", "size":12},# cam1Or = [yaw, pitch, roll]
    {"name":"cam2Pos", "type":"v", "size":12},# cam2Pos = [x, y, z]
    {"name":"cam2Or", "type":"v", "size":12},# cam2Or = [yaw, pitch, roll]
    {"name":"led1Pos", "type":"v", "size":12},# led1Pos = [x, y, z] in the drone frame
    {"name":"led2Pos", "type":"v", "size":12},# led2Pos = [x, y, z] in the drone frame
    {"name":"led3Pos", "type":"v", "size":12},# led3Pos = [x, y, z] in the drone frame
    {"name":"led4Pos", "type":"v", "size":12},# led4Pos = [x, y, z] in the drone frame
    #... non exhaustive
]   
def fakeSend(data, send_head):
    if send_head == None:
        print("Error: header not received yet, can't send the data")
        return None
    #pack the data
    packedData = packData(data, send_head)
    if packedData == None:
        return None
    #send the data
    to_send = packedData + END_LINE_KEY  
    print("fake sending the packet of size " + str(len(to_send))+ " bytes : " + str(to_send))   
'''
_______________________________________________________
______________________RECEIVE__________________________
_______________________________________________________
'''
RECEIVE_HEADER_KEY = "send_stream:".encode("utf-8")

#decode the data knowing his format (name, type, size and additional info)
def unpackOneData(oneData, formatt):
    # print("unpacking : " + str(oneData) + " with format : " + str(formatt))
    #check if the size of the data is correct
    if len(oneData) != formatt["size"]:
        print("Error: the size of the data is not correct")
        return None
    
    size = formatt["size"]
    for type_key in STD_TYPE_KEY:
        if formatt["type"] == type_key:
            return struct.unpack(type_key, oneData[:size])[0]
    if formatt["type"] == VECTOR_KEY:
        #unpack the vector
        vector = []
        for i in range(formatt["size"]//VECTOR_CONTENT_TYPE_SIZE):
            vector.append(struct.unpack(VECTOR_CONTENT_TYPE_KEY, oneData[i*4:(i+1)*4])[0])
        return vector
    elif formatt["type"] == MATRIX_KEY:
        #unpack the matrix
        matrix = []
        for i in range(formatt["row"]):
            vector = []
            cols = formatt["size"]//formatt["row"]//VECTOR_CONTENT_TYPE_SIZE
            for j in range(cols):
                index = i*formatt["size"]//formatt["row"] + j
                vector.append(struct.unpack(VECTOR_CONTENT_TYPE_KEY, oneData[i*formatt["row"]*VECTOR_CONTENT_TYPE_SIZE+j*VECTOR_CONTENT_TYPE_SIZE:(i*formatt["row"]+j+1)*VECTOR_CONTENT_TYPE_SIZE])[0])
            matrix.append(vector)
        return matrix
    else:
        print("Error: unknown type")
        return None
    
    
#decode an entire line of bytes of data according to the header
def unpackLine(line, header):
    #assumming that the line includes the stream register and the data one after the other, in bytes
    stream_register = struct.unpack("I", line[:4])[0]
    data = line[4:]
    #comput the size of the line exepted the stream register
    data_size = 0
    for i in range(len(header)):
        if stream_register & (1 << i):
            data_size += header[i]["size"]
    if data_size != len(data):
        print("Error: the size of the line is not correct")
        return None
    
    data_dict = {}
    #unpack the data
    for i in range(len(header)):
        #if the stream register is true for the i-th data meaning that the i-th data is present in the line
        if stream_register & (1 << i):
            oneData = data[:header[i]["size"]]
            data_dict[header[i]["name"]] = unpackOneData(oneData, header[i])
            #remove this data from the line
            data = data[header[i]["size"]:]
    return data_dict



receive_head = None
receive_buffer = b''

def receive(s):
    datas = []
    global receive_buffer
    receive_buffer += s.recv(1024)
    #split the receive_buffer by the end line key
    lines = receive_buffer.split(END_LINE_KEY)
    #décaler la liste de 1 pour enlever le dernier élément qui n'est pas complet
    lines = lines[:-1]
    for line in lines:
        #free this line from the receive_buffer
        receive_buffer = receive_buffer[len(line)+len(END_LINE_KEY):]
        #remove the end line key
        #we have a complete line
        #decode the header if the line begins with the header key
        if line[:len(RECEIVE_HEADER_KEY)] == RECEIVE_HEADER_KEY:
            head_bytes = line[len(RECEIVE_HEADER_KEY):]
            global receive_head
            # print("decoding header : " + str(head_bytes))
            receive_head = decodeHeader(head_bytes)
        elif line[:len(SEND_HEADER_KEY)] == SEND_HEADER_KEY:
            head_bytes = line[len(SEND_HEADER_KEY):]
            global send_head
            send_head = decodeHeader(head_bytes)
        else:
            if receive_head == None:
                print("Error: header not received yet, can't decode the data")
                return None
            #decode the data
            datas.append(unpackLine(line, receive_head))
    
    if len(datas) == 0:
        return None
    return datas

FAKE_RECEIVED_DATA = {
    "t": 0,
    "ekf_X":#position, velocity, orientation(quaternion)
        [
            0.0,#x
            0.0,#y
            0.3,#z
            0.0,#vx
            0.0,#vy
            0.1,#vz
            1,#qw
            0,#qx
            0,#qy
            0#qz
        ],
    "ekf_P":#covariance matrix
        [
            [0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1]
        ],
    "gyro":#gyro data
        [
            0.0,#wx
            0.0,#wy
            0.0#wz
        ],
    "acc":#accelerometer data
        [
            0.0,#ax
            0.0,#ay
            -9.81#az
        ],
    "cells":#cells voltage
        [3.7, 3.7, 3.7],
    
    "event code": 0,   
}


'''
_______________________________________________________
_____________________WRITE IN DB_______________________
_______________________________________________________
'''

# I don't think a json is efficient for real time telemetry, we should use a database like sqlite or mysql or something like that

def createDB(filePath = "realTimeTelemetry.json"):
    #for example, we can use a json file
    return open(filePath, "w+")

def writeInDB(data,db):
    #the data is a list of dict, the content can change depending what we receive from the device
    #it contain all the last data received from the device we didn't write in the dataBase yet
    #so this function is called every time we receive data from the device
    #for example, we can use a json file
    #clear the file
    db.seek(0)
    db.truncate()
    #write the data
    json.dump(data, db, indent=2)
    db.flush()


'''
_______________________________________________________
_____________________READ USER IN______________________
_______________________________________________________
'''
  
async def async_input(prompt=""):
    loop = asyncio.get_running_loop()
    return await loop.run_in_executor(ThreadPoolExecutor(), lambda: input(prompt))


async def userInput(send_head,db):
    '''
    this function is called nonstop
    args:
        send_head : a list of dict wich describe all the data the drone can understand (look at the fake_send_head above for an example)
        db : the dataBase object
    return:
        data_to_send : a dict with only the data the user want to send to the drone.
        For example :
            {
                "event code": 0,
                "posCommand": [0.0, 0.0, 0.0, 0.0]
            }
            
    for now, async_input on the console is used for testing, but we should use the dataBase :
    1 - wait for a new event in the dataBase (for new content)
    2 - pack the data as a dictionnary
    3 - clear the event in the dataBase (remove the data we just packed)
    4 - return the packed data         
    '''

    if send_head == None:
        return None
    
    data_to_send = {}
    for key in send_head:
        try:
            data_to_send[key["name"]] = float(await async_input("Enter " + key["name"] + " : "))
        except:
            print("the data is not correct, it will not be sent")
            continue
    return data_to_send

'''
_______________________________________________________
_________________MAIN THREADS__________________________
_______________________________________________________
'''
async def receiveTask(s, file):
    global receive_buffer
    while True:
        datas = receive(s)
        if datas is not None:
            writeInDB(datas, file)
            await asyncio.sleep(0.1)
            
async def fakeReceiveTask(fakeData, file):
    while True:
        writeInDB(fakeData, file)
        await asyncio.sleep(0.1)
    
async def sendTask(s):
    while True:
        data = await userInput()
        if data is not None:
            send(s, data, send_head)
            await asyncio.sleep(0.1)
            
async def fakeSendTask():
    while True:
        data = await userInput(fake_send_head)
        if data is not None:
            fakeSend(data, fake_send_head)
            await asyncio.sleep(0.1)
    


'''
_______________________________________________________
_____________________MAIN PROG_________________________
_______________________________________________________
'''

# for testing, we can use fake communication, so we don't need to connect to the drone to test the dataBase writing and reading
fake_communication = True

async def main():
    
    if fake_communication:
        file = createDB()
        await asyncio.gather(
            fakeReceiveTask(FAKE_RECEIVED_DATA, file),
            fakeSendTask()
        )
    else:
        connection = connect('C0:49:EF:CD:A7:D6')
        if connection == None:
            exit()

        file = createDB()

        await asyncio.gather(
            receiveTask(connection, file),
            sendTask(connection)
        )

# Exécutez le programme principal
if __name__ == "__main__":
    asyncio.run(main())