"""Paired offline replay of recorded real-camera yellow blocks; no image synthesis."""
from pathlib import Path
import cv2,numpy as np,csv,json,hashlib,sys,argparse
ROOT=Path(__file__).parent
ap=argparse.ArgumentParser();ap.add_argument('--video',type=Path);ap.add_argument('--out',type=Path,default=ROOT/'evidence');args=ap.parse_args()
OUT=args.out;OUT.mkdir(exist_ok=True,parents=True)
source=args.video or Path(json.loads((ROOT/'video_meta.json').read_text(encoding='utf-8'))[2]['path'])
cap=cv2.VideoCapture(str(source))
# Manual video annotation: strict, unambiguous one-second intervals. No calibration on test frames.
windows=[(f'Y{i}',float(i),float(i+1),'yellow') for i in [10,11,13,14,15]]
windows += [('Y38_6',38.6,39.6,'yellow')]
windows += [(f'R{i}',float(i),float(i+1),'not_yellow') for i in range(1,7)]
windows += [(f'G{i}',float(i),float(i+1),'not_yellow') for i in [19,20]]
windows += [(f'E{i}',float(i),float(i+1),'not_yellow') for i in list(range(32,38))+[40,41,42]]
windows += [(f'E{i}',float(i),float(i+1),'not_yellow') for i in range(48,51)]

def detect(frame,lo_h=26,hi_h=34):
    # Visible original image bounds, measured from 808x652 screen recording.
    image=cv2.resize(frame[82:605,59:757],(640,480))
    roi=image[100:400,160:480]
    hsv=cv2.cvtColor(roi,cv2.COLOR_BGR2HSV)
    m=cv2.inRange(hsv,np.uint8([lo_h,70,50]),np.uint8([hi_h,255,255]))
    m=cv2.morphologyEx(m,cv2.MORPH_OPEN,np.ones((5,5),np.uint8))
    m=cv2.morphologyEx(m,cv2.MORPH_CLOSE,np.ones((7,7),np.uint8))
    good=[]
    for c in cv2.findContours(m,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]:
        area=cv2.contourArea(c);x,y,w,h=cv2.boundingRect(c)
        if 800<=area<=120000 and 25<=w<=320 and 25<=h<=320 and area/max(1,w*h)>=.55:good.append(area)
    return bool(good),max(good,default=0)

rows=[];trials=[];annot=[]
for ident,start,end,truth in windows:
    seq={name:[] for name in ['baseline','expanded']}
    for k in range(10):
        ts=start+k*.1;cap.set(cv2.CAP_PROP_POS_MSEC,ts*1000);ok,f=cap.read()
        if not ok:raise RuntimeError('Cannot read '+str(ts))
        for name,hs in [('baseline',(26,34)),('expanded',(24,38))]:
            detected,area=detect(f,*hs);seq[name].append(detected)
            rows.append({'window':ident,'time_s':round(ts,3),'truth':truth,'variant':name,'detected':int(detected),'max_area':round(area,1)})
        if k==5:
            p=OUT/f'{ident}.png';p.write_bytes(cv2.imencode('.png',f)[1].tobytes());annot.append({'id':ident,'truth':truth,'time_s':ts,'path':str(p)})
    for name,values in seq.items():
        for n in [2,4,8]:
            run=0;at=None
            for k,v in enumerate(values):
                run=run+1 if v else 0
                if run>=n:at=k*.1;break
            trials.append({'window':ident,'truth':truth,'variant':name,'confirm_frames':n,'triggered':int(at is not None),'latency_s':at,'detected_frames':sum(values)})
cap.release()
for filename,data in [('frame_predictions.csv',rows),('paired_trials.csv',trials)]:
    with (OUT/filename).open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=list(data[0]));w.writeheader();w.writerows(data)
summary=[]
for name in ['baseline','expanded']:
    for n in [2,4,8]:
        a=[r for r in trials if r['variant']==name and r['confirm_frames']==n];pos=[r for r in a if r['truth']=='yellow'];neg=[r for r in a if r['truth']=='not_yellow'];ds=[r['latency_s'] for r in pos if r['triggered']]
        summary.append({'variant':name,'N':n,'positive_windows':len(pos),'misses':sum(1-r['triggered'] for r in pos),'negative_windows':len(neg),'false_triggers':sum(r['triggered'] for r in neg),'mean_success_latency_s':float(np.mean(ds)) if ds else None})
data={'source':str(source),'sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'sample_hz':10,'notes':'Video replay; consecutive windows are correlated. No physical hardware motion. Expanded H24-38 is a candidate comparison, not claimed deployed.','summary':summary,'annotation':annot}
(OUT/'ablation_summary.json').write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(summary,ensure_ascii=False,indent=2))
