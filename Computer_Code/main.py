import paho.mqtt.client as mqtt
import numpy
import matplotlib.pyplot as plt
import time

terminate=0
map=plt.plot()
fig=plt.figure()
ax=fig.add_subplot(111)
ax.set_aspect('equal')
new_point=[0,0]
def on_message(client, userdata, message):
    incoming_array=(message.payload.decode('utf-8'))
    global new_point
    global split_array
    split_array=incoming_array.split(',')
    new_point=[int(split_array[0]),int(split_array[1])]
#Address of EV3 broker
broker_address="192.168.7.154"
#Create a client that will recieve messages
client=mqtt.Client()
#Set on_message funcion of client to my on_message function
client.on_message=on_message
#Connects to ev3 broker
client.connect(broker_address)
#Subscribe to channel
client.subscribe('snoofer_bot_transmission')
#Starts loop to handle callbacks
client.loop_start()
file=open('coord_file', 'w')
print(new_point)
previous_point=[0,0]
while terminate==0:
    time.sleep(2)
    if new_point==[440440440440,440440440440]:
        terminate=1
    if new_point!=previous_point and terminate==0:
        print (new_point)
        plt.plot(new_point[0],new_point[1],"mo")
        file.write(f"{new_point[0]},{new_point[1]}\n")
        file.flush()
        previous_point=new_point
 
    plt.pause(0.01)
file.close()
if terminate==1:
    file.close()