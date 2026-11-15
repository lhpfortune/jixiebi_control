from pathlib import Path
import sys,json,importlib.util
root=Path(__file__).resolve().parents[1] if Path(__file__).parent.name=='tests' else Path(__file__).parent
pc=root/'pc';
if not pc.exists():pc=Path(r'E:\agent\机械臂\手势识别')
sys.path.insert(0,str(pc));src=root/'pc/pc_http_client.py'
if not src.exists():src=Path(__file__).parent/'pc_http_client.py'
sp=importlib.util.spec_from_file_location('client',src);m=importlib.util.module_from_spec(sp);sp.loader.exec_module(m)
from gesture_recognition.stabilizer import GestureDebouncer
from gesture_recognition.types import GesturePrediction
deb=GestureDebouncer();gate=m.StopPriorityGate();p=lambda s:GesturePrediction(s,.95,{},{});
for _ in range(5):deb.update(p('ONE'))
assert deb.locked
assert gate.update('FIST',.95) is False
assert gate.update('FIST',.95) is True
assert sum(gate.update('FIST',.95) for _ in range(20))==0
for _ in range(8):gate.update('NONE',0)
assert gate.update('FIST',.95) is False
assert gate.update('FIST',.95) is True
gate=m.StopPriorityGate();assert not any(gate.update('FIST',.50) for _ in range(20))
result={'cases':4,'passed':4,'kind':'software label sequence','checks':['STOP during normal LOCKED','held FIST emits once','releases and can trigger again','low-confidence FIST rejected']}
out=root/'results' if (root/'results').exists() else root/'evidence';out.mkdir(exist_ok=True);(out/'stop_priority_test.json').write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result))
