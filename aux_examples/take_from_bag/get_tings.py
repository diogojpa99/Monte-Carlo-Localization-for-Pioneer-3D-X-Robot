import bagpy
from bagpy import bagreader

import pandas as pd

b =  bagreader('/home/paulo/Desktop/take_from_bag/bagtoplay.bag') #path to bag file
print(b.topic_table)

#take wanted topics (creates .csv files)
posemsg = b.message_by_topic(topic='/pose') 
scanmsg = b.message_by_topic(topic='/scan')

#open .csv files
posedf = pd.read_csv(posemsg)
scandf = pd.read_csv(scanmsg)

#take wanted columns from .csv (name of the columns)
x_coord = posedf['pose.pose.position.x']
y_coord = posedf['pose.pose.position.y']
z_rotation = posedf['pose.pose.orientation.z']
w_rotation = posedf['pose.pose.orientation.w']

#create .xlsx file
col1 = "pose_X"
col2 = "pose_Y"
col3 = "rotation_Z"
col4 = "rotation_W"

pose = pd.DataFrame({col1:x_coord, 
                    col2:y_coord,
                    col3:z_rotation,
                    col4:w_rotation}) 

pose.to_excel('pose_info.xlsx', sheet_name="sheet1", index=False)

#and repeat
_n110 = scandf['ranges_72']
_n90 = scandf['ranges_128']
_n60 = scandf['ranges_214']
_n30 = scandf['ranges_299']
_p0 = scandf['ranges_384']
_p30= scandf['ranges_470']
_p60 = scandf['ranges_555']
_p90 = scandf['ranges_640']
_p110 = scandf['ranges_697']

col1 = "-110"
col2 = "-90"
col3 = "-60"
col4 = "-30"
col5 = "-0"
col6 = "30"
col7 = "60"
col8 = "90"
col9 = "110"

scan = pd.DataFrame({col1:_n110,
                    col2:_n90,
                    col3:_n60,
                    col4:_n30,
                    col5:_p0,
                    col6:_p30,
                    col7:_p60,
                    col8:_p90,
                    col9:_p110})

scan.to_excel('scan_info.xlsx', sheet_name="sheet1", index=False)