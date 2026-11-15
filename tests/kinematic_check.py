from pathlib import Path
import sys,json,numpy as np,argparse,os
ROOT=Path(__file__).parent
ap=argparse.ArgumentParser(description='Validate the original DICE platform SDK kinematics without moving hardware')
ap.add_argument('--sdk-dir',type=Path,default=Path(os.environ['DICE_SDK_DIR']) if os.environ.get('DICE_SDK_DIR') else None,help='Directory containing platform arm5dof_kinematic.py and transform.py')
ap.add_argument('--out',type=Path,default=ROOT.parent/'results')
args=ap.parse_args()
if args.sdk_dir is None:ap.error('Pass --sdk-dir or set DICE_SDK_DIR to the installed DICE platform directory')
sys.path.insert(0,str(args.sdk_dir.resolve()))
from arm5dof_kinematic import Arm5DoFKinematic
k=Arm5DoFKinematic();rng=np.random.default_rng(20261115);errors=[];matrix_errors=[];counts=[]
for i in range(100):
    q=rng.uniform(k.JOINT_ANGLE_LOWERB+.05,k.JOINT_ANGLE_UPPERB-.05)
    pose=np.asarray(k.forward_kinematic_v2(q));sols=k.inverse_kinematic(*pose,last_joint_angle=q)
    assert len(sols)>0,(i,q,pose)
    qs=min(sols,key=lambda s:np.linalg.norm(np.asarray(s)-q));pose2=np.asarray(k.forward_kinematic_v2(qs));errors.append(float(np.linalg.norm(pose[:3]-pose2[:3])))
    pose1,mats=k.forward_kinematic_v1(q);matrix_errors.append(float(np.linalg.norm(mats[-1][:3,3]-pose[:3])));counts.append(len(sols))
out={'samples':100,'seed':20261115,'units':'mm','geometry':'DICE physical SDK modified DH; no external position measurement','inverse_forward_max_position_error':max(errors),'modified_dh_vs_explicit_max_position_error':max(matrix_errors),'max_candidate_solutions':max(counts),'limits_deg':[np.degrees(k.JOINT_ANGLE_LOWERB).tolist(),np.degrees(k.JOINT_ANGLE_UPPERB).tolist()]}
args.out.mkdir(exist_ok=True,parents=True)
(args.out/'kinematic_check.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps(out,ensure_ascii=False,indent=2))
