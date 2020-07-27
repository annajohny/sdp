
from tug_diagnosis_msgs.msg import diagnosis_set
from copy import deepcopy
import time
import json
import yaml
from rospy__message_converter import message_converter
from tug_observers_msgs.msg import observer_info

#!/usr/bin/env python
import rospy
import sqlite3
from flask import Flask, g
app = Flask(__name__)

# connection to data base
def connect_db():
    app.app_context().push()
    sql = sqlite3.connect('diagnoses.db')
    sql.row_factory = sqlite3.Row
    return sql

def get_db():
    app.app_context().push()
    if not hasattr(g,'sqlite3_db'):
    	g.sqlite_db = connect_db()
    return g.sqlite_db

# diagnosis subscriber function
def callback_diagnosis(msg):
    app.app_context().push()
    db = get_db()
    rospy.loginfo("we have received the following: (%s, %d, %s ,%s,%s)",msg.header.stamp,msg.header.seq,msg.header.frame_id,msg.type,msg.diagnoses)
    print(type(msg.header.stamp),type(msg.header.seq),type(msg.header.frame_id),type(msg.type),type(msg.diagnoses))
    db.execute('insert into diagnosis_data (stamp, seq, frame_id, msg_type, msg_diagnoses) values (?, ?, ?, ?, ?)', \
            [str(msg.header.stamp),int(msg.header.seq),str(msg.header.frame_id),str(msg.type),str(msg.diagnoses)])
    db.commit()
    db.close()

#observation Subscriber function
def callback_observation(msg):
    app.app_context().push()
    db=get_db() 

    # converting ROS message to dictionary
    dictionary = message_converter.convert_ros_message_to_dictionary(msg)
    observation_info = list(dictionary['observation_infos'])
    resources = observation_info[0]['resource']
    header = observation_info[0]['header']
    print(resources)

    #accesing the type dictionary element
    observation_info_type =observation_info[0]['type']

    #accesing the observation dictionary element
    observations = list(observation_info[0]['observation'])
    verbose_observation_msg = observations[0]['verbose_observation_msg']
    
    observation = observations[0]['observation']
    observation_msg = observations[0]['observation_msg']
    
    # inserting data to  database
    db.execute('insert into observation_data (header,resources,observation,observation_msg,verbose_observation_msg) values (?, ?, ?, ?, ?)', \
            [str(resources),str(header),str(verbose_observation_msg),str(observation),str(observation_msg)])
    db.commit()
    db.close()
    
    
rospy.init_node('listener', anonymous=True)

rospy.Subscriber("chatter", diagnosis_set, callback_diagnosis)
rospy.Subscriber('/observers/info', observer_info, callback_observation)
rospy.spin()



