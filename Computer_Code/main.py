import paho.mqtt.client as mqtt
import numpy
import matplotlib.pyplot as plt
import time


terminate=0
fig=plt.figure()
ax=fig.add_subplot(111)
ax.set_aspect('equal')
def on_message(client, userdata, message):
    incoming_data=(message.payload.decode('utf-8'))
    global new_data
    new_data=[]
    split_array=incoming_data.split(',')
    for item in split_array:
        new_data.append(int(item))
    print(f'New data: {new_data}')
   
        
#Address of bot broker
broker_address="192.168.7.212"
#Create a client that will recieve messages
client=mqtt.Client()
#Set on_message funcion of client to my on_message function
client.on_message=on_message
#Connects to ev3 broker
client.connect(broker_address)
#Subscribe to channel
client.subscribe('Bot_to_computer')
#Starts loop to handle callbacks
client.loop_start()
file=open('coord_file', 'w')
new_data=[0,0]
previous_data=[0,0]
print('Entering loop')
if new_data[0]==0:
    while terminate<24:
        print(new_data)
        time.sleep(2)
        if new_data!=previous_data:
            print (new_data)
            plt.plot(new_data[1],new_data[2],"mo")
            file.write(f"Object location: {new_data[1]},{new_data[2]}\n")
            if len(new_data)>3:
                file.write(f"Bot location: {new_data[3]},{new_data[4]}\n")
            if len(new_data)>4:
                file.write(f'Gyro angle: {new_data[5]}\n')
            file.flush()
            terminate+=1
            previous_data=new_data
        plt.pause(0.1)
if new_data[0]==1:
    file.write(new_data)
file.close()