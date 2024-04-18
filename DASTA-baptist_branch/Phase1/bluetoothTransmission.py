import socket
import time
import struct
import threading
import pprint
pp = pprint.PrettyPrinter(depth=4)




STD_TYPE_KEY = ['c', 'i', 'Q', 'f', 'd','B']
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
    packedData = None
    if formatt["type"] in STD_TYPE_KEY:
        packedData = struct.pack(formatt["type"], OneData)
    elif formatt["type"] == VECTOR_KEY:
        #pack the vector
        packedData = b''
        for i in range(len(OneData)):
            packedData += struct.pack(VECTOR_CONTENT_TYPE_KEY, OneData[i])
    elif formatt["type"] == MATRIX_KEY:
        #pack the matrix
        packedData = b''
        for i in range(formatt["row"]):
            for j in range(len(OneData[i])):
                packedData += struct.pack(VECTOR_CONTENT_TYPE_KEY, OneData[i][j])
    else:
        print("Error: unknown type")
        return None
    
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
    if len(packedData) == 0:
        return None   
      
    return struct.pack("I", send_register) + packedData

send_head = None

def send(s, data, send_head):
    if send_head == None:
        print("Error: header not received yet, can't send the data")
        return None
    #pack the data
    print("sending : " + str(data[0]))
    packedData = packData(data[0], send_head)
    data.pop(0) 
    if packedData == None:
        return None
    #send the data
    to_send = packedData + END_LINE_KEY  
    s.sendall(to_send)
   
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

DEBUG = True

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
            tmp = decodeHeader(head_bytes)
            if DEBUG:
                print("receive_head : ")
                pp.pprint(tmp)
                
            global receive_head
            receive_head = tmp
            
        elif line[:len(SEND_HEADER_KEY)] == SEND_HEADER_KEY:
            head_bytes = line[len(SEND_HEADER_KEY):]
            
            tmp = decodeHeader(head_bytes)
            if DEBUG:
                print("send_head : ")
                pp.pprint(tmp)
            global send_head
            send_head = tmp
            global inited
            inited = True
        else:
            if receive_head == None:
                print("Error: header not received yet, can't decode the data")
                return None
            #decode the data
            new_datas = unpackLine(line, receive_head)
            datas.append(new_datas)
            # if DEBUG:
            #     print("new_datas : ")
            #     pp.pprint(new_datas)
    
    if len(datas) == 0:
        return None
    return datas


'''
_______________________________________________________
_____________________DATA BASE SET_____________________
_______________________________________________________
'''

def open_dbs()->tuple:
    '''
    open the dataBase objects
    return:
        telemetry_db : the dataBase object for the telemetry
        userCommand_db : the dataBase object for the userCommand
        
    '''
    return None, None



'''
_______________________________________________________
_____________________WRITE IN DB_______________________
_______________________________________________________
'''
def writeInDB(data,db)->None:
    '''
    add the data to the dataBase while keeping the old data
    
    args:
        data : a list of dict with the data to write in the dataBase
        db : the dataBase object
    return:
        None
    '''

    pass
    
    
'''
_______________________________________________________
_____________________READ USER IN______________________
_______________________________________________________
'''
  

def userInput(send_head,db)->dict:
    '''
    this function is called nonstop
    args:
        send_head : a list of dict wich describe all the data the drone can understand (look at the fake_send_head above for an example)
        db : the dataBase object
    return:
        data_to_send : a dict with only the data the user want to send to the drone, it can be uses or not
        For example :
            {
                "event code": 0,
                "posCommand": [0.0, 0.0, 0.0, 0.0]
            }
            
    1 - wait for a new event in the dataBase (for new content)
    2 - pack the data as a dictionnary
    3 - clear the event in the dataBase (remove the data we just packed)
    4 - return the packed data     
    '''
    return None

def userInputTest(send_head,db)->dict:
    '''
    use the terminal input to listen to the user
    
    Re : the last element is the userCommand 
    wich is in cpp :
        enum UserEvent
            {
                None,
                StartStateEstimate,
                StopStateEstimate,
                StartStream,
                StopStream,
                EnableStateEstimateStream,
                DisableStateEstimateStream,
                EnableSensorStream,
                DisableSensorStream,
            };  
    '''
    
    userKey =  input("Enter a key to transmite : ")
    keys = [send_head[i]["name"] for i in range(len(send_head))]
    if userKey not in keys:
        print("Error: unknown key")
        return userInputTest(send_head,db)
    expectedType = send_head[keys.index(userKey)]["type"]
    
    userValue = input("Enter a " + expectedType + " : ")
    #convert the value to the correct type
    try :
        if send_head[keys.index(userKey)]["type"] == "i":
            userValue = int(userValue)
        elif send_head[keys.index(userKey)]["type"] == "f":
            userValue = float(userValue)
        elif send_head[keys.index(userKey)]["type"] == "c":
            userValue = str(userValue)[0]
        elif send_head[keys.index(userKey)]["type"] == "s":
            userValue = str(userValue)
        elif send_head[keys.index(userKey)]["type"] == "v":
            userValue = [float(i) for i in userValue.split(" ")]
        elif send_head[keys.index(userKey)]["type"] == "m":
            userValue = [[float(j) for j in i.split(" ")] for i in userValue.split(";")]
        elif send_head[keys.index(userKey)]["type"] == "B":
            userValue = int(userValue)
        else:
            print("Error: unknown type")
            return userInputTest(send_head,db)
    except Exception as e:
        print("input incorrect")
        return userInputTest(send_head,db)
    
    
    print("returning : " + str({userKey:userValue}))
    return {userKey:userValue}
    # commandKey = "user_event"
    # commandCode = 0
    
    # if userInput == "StartStateEstimate":
    #     commandCode = 1
    # elif userInput == "StopStateEstimate":
    #     commandCode = 2
    # elif userInput == "StartStream":
    #     commandCode = 3
    # elif userInput == "StopStream":
    #     commandCode = 4
    # elif userInput == "EnableStateEstimateStream":
    #     commandCode = 5
    # elif userInput == "DisableStateEstimateStream":
    #     commandCode = 6
    # elif userInput == "EnableSensorStream":
    #     commandCode = 7
    # elif userInput == "DisableSensorStream":
    #     commandCode = 8
    # else:
    #     print("Error: unknown command")
    #     return None
    
    # ret = {commandKey:commandCode}
    

'''
_______________________________________________________
_________________MAIN THREADS__________________________
_______________________________________________________
using threading to run multiple tasks at the same time, so the communication is not blocked by the user input or the dataBase writing
'''
received_data = []

def receiveTask(s):
    global received_data
    while True:
        new_data = receive(s)
        if receive_head is None:
            continue
        if new_data is not None:
            received_data.append(new_data)
        #to let the other threads run
        time.sleep(0.001)
                    
def saveTask(file):
    global received_data
    while True:
        writeInDB(received_data, file)
        received_data = []
        time.sleep(0.05)

data_to_send = []
def sendTask(s,db):
    global data_to_send
    while inited == False or send_head is None:
        time.sleep(0.1)
    time.sleep(1) #to be sure all is stable
    while True:
        if len(data_to_send) > 0:
            send(s, data_to_send, send_head)
        #to let the other threads run
        time.sleep(0.1)

def userInputTask(db):
    global data_to_send
    while True:
        #data = userInput(send_head,db)
        new_data_to_send = userInputTest(send_head,db)
        if new_data_to_send is not None:
            data_to_send.append(new_data_to_send)
        #to let the other threads run
        time.sleep(0.1)


'''
_______________________________________________________
_____________________MAIN PROG_________________________
_______________________________________________________
'''
inited = False
def main():
    connection = connect('24:6F:28:7B:DB:22')
    if connection == None:
        exit()

    telemetry_db, userCommand_db = open_dbs()
    threading.Thread(target=receiveTask, args=(connection,)).start()
    threading.Thread(target=sendTask, args=(connection,userCommand_db)).start()
    # threading.Thread(target=saveTask, args=(telemetry_db,)).start()
    # threading.Thread(target=userInputTask, args=(userCommand_db,)).start()

# Exécutez le programme principal

"""
in cpp 
enum UserEvent : uint8_t
{
    None=0,
    StartStateEstimate,
    StopStateEstimate,
    StartStream,
    StopStream,
    EnableStateEstimateStream,
    DisableStateEstimateStream,
    EnableSensorStream,
    DisableSensorStream,
    StartGyroBiasEstimation,
    StopGyroBiasEstimation,
};
"""
if __name__ == "__main__":
    main()
    data_to_send.append({"user_event": 7, "send_stream_delay": 50})
    time.sleep(3)
    data_to_send.append({"user_event": 3})
    
    while True:
        if len(received_data) > 0:
            print(received_data)
            received_data = []