"""Local HTTP/state regression only. FakeArm cannot move real hardware."""
from pathlib import Path
import importlib.util,sys,json,time,threading,urllib.request,urllib.error,uuid,csv,contextlib,io
import numpy as np
ROOT=Path(__file__).parent;OUT=ROOT.parent/'results';OUT.mkdir(exist_ok=True)
import argparse
ap=argparse.ArgumentParser();ap.add_argument('--server',type=Path,default=ROOT.parent/'robot/gesture_robot_server.py');args=ap.parse_args()
spec=importlib.util.spec_from_file_location('original_server',args.server);s=importlib.util.module_from_spec(spec);spec.loader.exec_module(s)
class FakeArm:
    def __init__(self):self.calls=[]
    def set_damping(self,power):self.calls.append((power,time.perf_counter_ns()))
c=s.GestureRobotController(move_on_start=False,allow_motion=False);fake=FakeArm();c.arm=fake;c.state=s.STATE_READY
server=s.build_http_server(c,host='127.0.0.1',port=0);thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start();url=f'http://127.0.0.1:{server.server_address[1]}/command'
records=[]
def post(command):
    data=json.dumps({'command':command,'request_id':str(uuid.uuid4()),'gesture':command}).encode();req=urllib.request.Request(url,data=data,headers={'Content-Type':'application/json','X-Control-Token':s.CONTROL_TOKEN})
    t0=time.perf_counter_ns()
    try:r=urllib.request.urlopen(req,timeout=3)
    except urllib.error.HTTPError as e:r=e
    body=json.loads(r.read());return r.code,body,(time.perf_counter_ns()-t0)/1e6
def record(case,expect,command):
    code,b,dt=post(command);ok=code==expect;records.append({'case':case,'command':command,'status':code,'expected':expect,'state':c.state,'pass':ok,'http_roundtrip_ms':round(dt,3)});assert ok
with contextlib.redirect_stdout(io.StringIO()) as log:
    for k in range(2):
        c.state=s.STATE_READY;c.last_color='yellow';c.last_frame_at=time.monotonic();record(f'cancel_{k+1}_select',200,'SELECT_GRAB');record(f'cancel_{k+1}',200,'CANCEL');assert c.selection is None and c.state==s.STATE_READY and c._job_queue.empty()
    c.state=s.STATE_READY;record('confirm_without_selection',409,'CONFIRM')
    c.last_color='red';c.last_frame_at=time.monotonic();record('select_red',200,'SELECT_GRAB');record('confirm_once',200,'CONFIRM');record('busy_duplicate_confirm',409,'CONFIRM');record('busy_cancel',409,'CANCEL');assert c._job_queue.qsize()==1
    latencies=[]
    for k in range(100):
        c.state=s.STATE_EXECUTING;c._stop_event.clear();t0=time.perf_counter_ns();code,b,dt=post('STOP');assert code==200 and c.state==s.STATE_ESTOP and c._stop_event.is_set();latencies.append((dt,(fake.calls[-1][1]-t0)/1e6))
    c.state=s.STATE_ESTOP;record('estop_cannot_confirm',409,'CONFIRM')
    c.state=s.STATE_FAULT;record('fault_cannot_select',409,'SELECT_RESET')
server.shutdown();server.server_close();(OUT/'controller_regression.log').write_text(log.getvalue(),encoding='utf-8')
with (OUT/'controller_cases.csv').open('w',newline='',encoding='utf-8-sig') as f:w=csv.DictWriter(f,fieldnames=list(records[0]));w.writeheader();w.writerows(records)
with (OUT/'stop_http_latency.csv').open('w',newline='',encoding='utf-8-sig') as f:w=csv.writer(f);w.writerow(['trial','http_roundtrip_ms','request_start_to_fake_damping_ms']);w.writerows([[i+1,round(a,4),round(b,4)]for i,(a,b) in enumerate(latencies)])
out={'kind':'software loopback HTTP + FakeArm, not physical stop','cases_passed':len(records),'cases_total':len(records),'cancel_cases':2,'stop_requests':100,'roundtrip_ms':{'median':float(np.median([a for a,b in latencies])),'p95':float(np.percentile([a for a,b in latencies],95))},'fake_call_ms':{'median':float(np.median([b for a,b in latencies])),'p95':float(np.percentile([b for a,b in latencies],95))}}
(OUT/'controller_summary.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps(out,ensure_ascii=False,indent=2))
