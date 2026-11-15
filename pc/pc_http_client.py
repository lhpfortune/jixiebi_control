"""HTTP adapter added during final integration. Recognition core is reused.
Does not import a serial SDK. STOP has an independent dispatch path.
"""
from __future__ import annotations
import argparse,json,os,queue,threading,time,uuid,urllib.request,urllib.error
from pathlib import Path
import cv2
from gesture_recognition.landmarker import HandLandmarkDetector
from gesture_recognition.classifier import HeuristicGestureClassifier
from gesture_recognition.stabilizer import GestureDebouncer
from gesture_recognition.types import GesturePrediction

MAP={'ONE':'SELECT_GRAB','TWO':'SELECT_RESET','OK':'CONFIRM','OPEN_PALM':'CANCEL'}
class StopPriorityGate:
    def __init__(self,frames=2):self.frames=frames;self.count=0;self.sent=False;self.release=0
    def update(self,label,confidence):
        if label=='FIST' and confidence>=.72:
            self.release=0;self.count+=1
            if self.count>=self.frames and not self.sent:self.sent=True;return True
        else:
            self.count=0;self.release+=1
            if self.release>=8:self.sent=False
        return False

def send(url,token,command,gesture='',confidence=0):
    body=json.dumps({'command':command,'gesture':gesture,'confidence':confidence,'request_id':str(uuid.uuid4())}).encode()
    req=urllib.request.Request(url.rstrip('/')+'/command',data=body,headers={'Content-Type':'application/json','X-Control-Token':token})
    try:r=urllib.request.urlopen(req,timeout=2)
    except urllib.error.HTTPError as e:r=e
    return r.code,json.loads(r.read())

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--robot-url',default='http://192.168.55.1:8765');ap.add_argument('--token',default=os.getenv('ROBOT_CONTROL_TOKEN',''));ap.add_argument('--model',type=Path,default=Path(__file__).parent/'models/hand_landmarker.task');ap.add_argument('--camera',type=int,default=0);ap.add_argument('--no-robot',action='store_true');args=ap.parse_args()
    if not args.no_robot and not args.token:ap.error('Set ROBOT_CONTROL_TOKEN or use --no-robot')
    deb=GestureDebouncer();classifier=HeuristicGestureClassifier();stop_gate=StopPriorityGate();jobs=queue.Queue(maxsize=1);halt=threading.Event();state={'state':'UNKNOWN','message':''}
    def dispatch(cmd,label,conf):
        if args.no_robot:print('DRY',cmd,label);return
        try:
            status,body=send(args.robot_url,args.token,cmd,label,conf);state.update(body);print(status,body.get('message',''))
        except Exception as exc:state['message']=str(exc);print('HTTP ERROR',repr(exc))
    def worker():
        while not halt.is_set():
            try:item=jobs.get(timeout=.2)
            except queue.Empty:continue
            dispatch(*item);jobs.task_done()
    threading.Thread(target=worker,daemon=True).start()
    cap=cv2.VideoCapture(args.camera,cv2.CAP_DSHOW);cap.set(3,1280);cap.set(4,720)
    try:
        with HandLandmarkDetector(args.model) as detector:
            start=time.monotonic()
            while True:
                ok,frame=cap.read()
                if not ok:raise RuntimeError('Camera read failed')
                frame=cv2.flip(frame,1);obs,_=detector.detect(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB),int((time.monotonic()-start)*1000));prediction=classifier.predict(obs.landmarks) if obs else GesturePrediction('NONE',0,{},{});
                if stop_gate.update(prediction.label,prediction.confidence):
                    threading.Thread(target=dispatch,args=('STOP','FIST',prediction.confidence),daemon=True).start()
                stable=deb.update(prediction)
                if stable.triggered and stable.triggered.label in MAP:
                    try:jobs.put_nowait((MAP[stable.triggered.label],stable.triggered.label,stable.triggered.confidence))
                    except queue.Full:print('CLIENT BUSY: command rejected')
                cv2.putText(frame,f'{prediction.label} {prediction.confidence:.2f} | {stable.state}',(20,35),cv2.FONT_HERSHEY_SIMPLEX,.8,(0,255,255),2)
                cv2.putText(frame,'Robot: '+str(state.get('state','UNKNOWN')),(20,70),cv2.FONT_HERSHEY_SIMPLEX,.8,(0,255,255),2)
                cv2.imshow('Gesture HTTP controller',frame)
                if cv2.waitKey(1)&255 in [27,ord('q')]:break
    finally:halt.set();cap.release();cv2.destroyAllWindows()
if __name__=='__main__':main()
