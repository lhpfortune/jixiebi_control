#!/usr/bin/env python3
import sys,time,json,threading,os
from pathlib import Path
import cv2,rospy,rosgraph,numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image,JointState
ROOT=Path(__file__).parent;OUT=Path(os.environ.get('COURSE_RESULTS_DIR',str(ROOT.parent/'results')));OUT.mkdir(exist_ok=True)
source=os.environ['ROBOT_REPLAY_VIDEO']
rospy.init_node('course_video_replay_probe',anonymous=False)
bridge=CvBridge();pub=rospy.Publisher('/camera/image_raw',Image,queue_size=1)
times={t:[] for t in ['/joint_states','/camera/image_raw','/sfd_jixiebi/vision/debug_image']};example={};lock=threading.Lock()
def callback(msg,topic):
    with lock:
        times[topic].append(time.monotonic())
        if topic not in example:
            if isinstance(msg,Image):
                image=bridge.imgmsg_to_cv2(msg,'bgr8');example[topic]={'height':msg.height,'width':msg.width,'encoding':msg.encoding,'step':msg.step,'frame_id':msg.header.frame_id,'shape':list(image.shape),'pixel_bytes':len(msg.data)}
            else:example[topic]={'names':list(msg.name),'position':list(msg.position),'count':len(msg.name)}
subs=[rospy.Subscriber(t,JointState if t=='/joint_states' else Image,callback,callback_args=t,queue_size=20) for t in times]
cap=cv2.VideoCapture(source);assert cap.isOpened();t0=time.monotonic();k=0
while time.monotonic()-t0<12 and not rospy.is_shutdown():
    ok,frame=cap.read()
    if not ok:cap.set(cv2.CAP_PROP_POS_FRAMES,0);continue
    image=cv2.resize(frame[82:605,59:757],(640,480));msg=bridge.cv2_to_imgmsg(image,'bgr8');msg.header.stamp=rospy.Time.now();msg.header.frame_id='camera_optical_frame';pub.publish(msg)
    if k==20:(OUT/'camera_replay_frame.png').write_bytes(cv2.imencode('.png',image)[1].tobytes())
    k+=1;time.sleep(max(0,t0+k*.1-time.monotonic()))
cap.release();rates={t:{'messages':len(ts),'hz':(len(ts)-1)/(ts[-1]-ts[0]) if len(ts)>1 else 0} for t,ts in times.items()}
master=rosgraph.Master('/course_video_replay_probe');state=master.getSystemState()
data={'input_kind':'recorded real camera video replay','rates':rates,'example':example,'graph':{'publishers':state[0],'subscribers':state[1],'services':state[2]}}
(OUT/'ros_probe.json').write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps(data,ensure_ascii=False,indent=2))
