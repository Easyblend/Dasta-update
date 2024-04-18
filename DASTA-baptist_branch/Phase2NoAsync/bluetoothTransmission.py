import socket
import time
import struct
import pprint
import threading
pp = pprint.PrettyPrinter(depth=4)

fake_transmission = False
use_async_receive = True
print_transmission = False
print_reception = False

received_data = []
data_to_send = []
inited = False



STD_TYPE_KEY = ['c', 'i', 'Q', 'f', 'd','B','L']
VECTOR_KEY = 'v'
MATRIX_KEY = 'm'
VECTOR_CONTENT_TYPE_KEY = 'f'
VECTOR_CONTENT_TYPE_SIZE = 4


bl_connection = None
total_received_counter = 0
total_sent_counter = 0

'''
_______________________________________________________
______________________CONNECTION__________________________
_______________________________________________________
'''
#connect to the Bluetooth device
def connect(address, port=1 , max_attempts=10):
    global bl_connection
    if bl_connection is None:
        bl_connection = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    print("Connecting to " + str(address) + " on port " + str(port) + "...")
    #connect to the server
    attempts = 0
    while attempts < max_attempts:
        try:
            bl_connection.connect((address, port))
            break
        except socket.error as e:
            print("Error: socket error, retrying...")
            print(e)
            attempts += 1
            time.sleep(1)
    if attempts == max_attempts:
        print("Error: can't connect to the server")
        bl_connection = None
        return
    
    if not use_async_receive:
        bl_connection.settimeout(0.001)
    print("Connected")

'''
_______________________________________________________
_____________________TRANSMISSIONS_____________________
_______________________________________________________
'''
END_LINE_KEY = "end_line\n".encode("utf-8")

#when we receive the header, we need to decode it to know how to decode the data
def decode_header(header):
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

    
def packOne_data(One_data, formatt):
    packedData = None
    if formatt["type"] in STD_TYPE_KEY:
        packedData = struct.pack(formatt["type"], One_data)
    elif formatt["type"] == VECTOR_KEY:
        #pack the vector
        packedData = b''
        for i in range(len(One_data)):
            packedData += struct.pack(VECTOR_CONTENT_TYPE_KEY, One_data[i])
    elif formatt["type"] == MATRIX_KEY:
        #pack the matrix
        packedData = b''
        for i in range(formatt["row"]):
            for j in range(len(One_data[i])):
                packedData += struct.pack(VECTOR_CONTENT_TYPE_KEY, One_data[i][j])
    else:
        print("Error: unknown type")
        return None
    
    if len(packedData) != formatt["size"]:
        print("Error: the size of the data is not correct")
        return None
    
    return packedData

def pack_data(data, header):
    #data is a dict
    #header is a list of dict
    
    packed_data = b''
    send_register = 0
    for i in range(len(header)):
        #pack the data
        if header[i]["name"] in data:
            send_register += 1 << i
            packed_data += packOne_data(data[header[i]["name"]], header[i])
    if len(packed_data) == 0:
        return None   
      
    return struct.pack("I", send_register) + packed_data

send_head = None

def send(s, data, send_head):
    global total_sent_counter
    if send_head == None:
        print("Error: header not received yet, can't send the data")
        return None
    packed_data = pack_data(data[0], send_head)
    if print_transmission:
        print("sending : " + str(data[0]) + " packed : " + str(packed_data))
    data.pop(0) 
    if packed_data == None:
        return None
    #send the data
    to_send = packed_data + END_LINE_KEY  
    total_sent_counter += 1
    s.sendall(to_send)  
   
'''
_______________________________________________________
______________________RECEIVE__________________________
_______________________________________________________
'''
RECEIVE_HEADER_KEY = "send_stream:".encode("utf-8")

#decode the data knowing his format (name, type, size and additional info)
def unpack_one_data(one_data, formatt):
    # print("unpacking : " + str(one_data) + " with format : " + str(formatt))
    #check if the size of the data is correct
    if len(one_data) != formatt["size"]:
        # print("Error: the size of the data is not correct")
        return None
    
    size = formatt["size"]
    for type_key in STD_TYPE_KEY:
        if formatt["type"] == type_key:
            return struct.unpack(type_key, one_data[:size])[0]
    if formatt["type"] == VECTOR_KEY:
        #unpack the vector
        vector = []
        for i in range(formatt["size"]//VECTOR_CONTENT_TYPE_SIZE):
            vector.append(struct.unpack(VECTOR_CONTENT_TYPE_KEY, one_data[i*4:(i+1)*4])[0])
        return vector
    elif formatt["type"] == MATRIX_KEY:
        #unpack the matrix
        matrix = []
        for i in range(formatt["row"]):
            vector = []
            cols = formatt["size"]//formatt["row"]//VECTOR_CONTENT_TYPE_SIZE
            for j in range(cols):
                vector.append(struct.unpack(VECTOR_CONTENT_TYPE_KEY, one_data[i*formatt["row"]*VECTOR_CONTENT_TYPE_SIZE+j*VECTOR_CONTENT_TYPE_SIZE:(i*formatt["row"]+j+1)*VECTOR_CONTENT_TYPE_SIZE])[0])
            matrix.append(vector)
        return matrix
    else:
        print("Error: unknown type")
        return None
    
    
#decode an entire line of bytes of data according to the header
def unpack_line(line, header):
    #assumming that the line includes the stream register and the data one after the other, in bytes
    stream_register = struct.unpack("I", line[:4])[0]
    data = line[4:]
    #comput the size of the line exepted the stream register
    data_size = 0
    for i in range(len(header)):
        if stream_register & (1 << i):
            data_size += header[i]["size"]
    if data_size != len(data):
        # print("Error: the size of the line is not correct")
        return None
    
    data_dict = {}
    #unpack the data
    for i in range(len(header)):
        #if the stream register is true for the i-th data meaning that the i-th data is present in the line
        if stream_register & (1 << i):
            one_data = data[:header[i]["size"]]
            data_dict[header[i]["name"]] = unpack_one_data(one_data, header[i])
            #remove this data from the line
            data = data[header[i]["size"]:]
    return data_dict



receive_head = None
receive_buffer = b''

DEBUG = True

#import select
def receive(bl_connection):
    datas = []
    global receive_buffer, total_received_counter, receive_head, send_head, inited

    receive_buffer += bl_connection.recv(1024)    
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
            tmp = decode_header(head_bytes)
            if DEBUG:
                print("receive_head : ")
                pp.pprint(tmp)
                
            receive_head = tmp
            if send_head is not None:
                inited = True
            
        elif line[:len(SEND_HEADER_KEY)] == SEND_HEADER_KEY:
            head_bytes = line[len(SEND_HEADER_KEY):]
            
            tmp = decode_header(head_bytes)
            if DEBUG:
                print("send_head : ")
                pp.pprint(tmp)
            send_head = tmp
            if receive_head is not None:
                inited = True
        else:
            if receive_head == None:
                print("Error: header not received yet, can't decode the data")
                return None
            #decode the data
            new_datas = unpack_line(line, receive_head)
            if print_reception:
                print("received : " + str(new_datas))
            if new_datas is not None:
                datas.append(new_datas)
    
    if len(datas) == 0:
        return None
    
    total_received_counter += len(datas)
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
_________________MAIN THREADS__________________________
_______________________________________________________
using threading to run multiple tasks at the same time, so the communication is not blocked by the user input or the dataBase writing
'''

def receive_task(bl_connection):
    global received_data
    new_data = None
    if fake_transmission:
        return
    
    try:
        new_data = receive(bl_connection)
    except socket.timeout:
        return #no data available
    except Exception as e:
        print("Error durring receive the data :")
        print(e)
    if receive_head is None:
        return
    if new_data is not None:
        received_data += new_data

def receive_async_task(bl_connection):
    while True:
        receive_task(bl_connection)
        time.sleep(0.001)     
                   
def save_task(file):
    global received_data
    writeInDB(received_data, file)
    received_data = []

last_time_send = time.time()
def send_task(bl_connection):
    global data_to_send
    if fake_transmission:
        data_to_send.pop(0)
        return
    if inited == False or send_head is None:
        return
    if len(data_to_send) > 0:
        send(bl_connection, data_to_send, send_head)


'''
_______________________________________________________
_____________________MAIN PROG_________________________
_______________________________________________________
'''
def init_transmission(address):
    if fake_transmission:
        return
    connect(address)
    if use_async_receive:
        threading.Thread(target=receive_async_task, args=(bl_connection,)).start()
    
    while not inited:
        time.sleep(0.01)
        
def kill_transmission():
    global bl_connection
    if bl_connection is not None:
        bl_connection.close()
        bl_connection = None
    
def run_transmission():
    if bl_connection == None:
        return
    if not use_async_receive:
        receive_task(bl_connection)
        
    send_task(bl_connection)
    


# Exécutez le programme principal

# compute the send_stream_register (embedded)
#(a 'L', an unsigned long, is 4 bytes, 
# whitch each bits represent the state of the stream channel)
send_stream_register = 0
def enable_stream_channel(channels):
    global send_stream_register, receive_head
    if receive_head is None:
        print("Error: header not received yet, can't enable the stream channel")
        return
    channels_indexes = [i for i in range(len(receive_head)) if receive_head[i]["name"] in channels]
    for i in channels_indexes:
        send_stream_register += 1 << i
    data_to_send.append({"send_stream_register": send_stream_register})

def disable_stream_channel(channels):
    global send_stream_register, receive_head
    if receive_head is None:
        print("Error: header not received yet, can't disable the stream channel")
        return
    channels_indexes = [i for i in range(len(receive_head)) if receive_head[i]["name"] in channels]
    for i in channels_indexes:
        send_stream_register -= 1 << i
    data_to_send.append({"send_stream_register": send_stream_register})


user_event_dict = {
    "None": 0,
    "EnableStream": 1,
    "DisableStream": 2,
    "SwitchToRaceMode": 3,
    "SwitchToAttiMode": 4,
    "SwitchToVelMode": 5,
    "SwitchToPosMode": 6,
    "EngageMotors": 7,
    "DisengageMotors": 8,
    "EMERGENCY_STOP": 9
}

embedded_event_dict = {
    "None2": 0,
    "WaitingForBatteryPower": 1,
    "WaitingForCalibration": 2,
    "WaitingForKalmanConvergence": 3,
    "ReadyToFly": 4,
    "LowBattery": 5
}

if __name__ == "__main__":
    init_transmission("08:D1:F9:CE:C3:76")
    # enable_stream_channel(["time","gyro_raw", "acc_raw"])

    enable_stream_channel([receive_head[i]["name"] for i in range(len(receive_head))])

    data_to_send.append({"user_event": user_event_dict["EnableStream"],"send_stream_delay": 50})
    
    while True:
        run_transmission()
        if len(received_data) >= 100:
            print("last received : " + str(received_data[-1]))
            received_data = []
            print("total received : " + str(total_received_counter))
            print("flushing")