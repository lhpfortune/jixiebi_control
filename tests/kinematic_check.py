from pathlib import Path
import sys,json,numpy as np
ROOT=Path(__file__).parent;sys.path.insert(0,r'E:\agent\机械臂\手势识别')
from arm_sdk.arm5dof_kinematic import Arm5DoFKinematic
k=Arm5DoFKinematic();rng=np.random.default_rng(20261115);errors=[];matrix_errors=[];counts=[]
for i in range(100):
    q=rng.uniform(k.JOINT_ANGLE_LOWERB+.05,k.JOINT_ANGLE_UPPERB-.05)
    pose=np.asarray(k.forward_kinematic_v2(q));sols=k.inverse_kinematic(*pose,last_joint_angle=q)
    assert len(sols)>0,(i,q,pose)
    qs=min(sols,key=lambda s:np.linalg.norm(np.asarray(s)-q));pose2=np.asarray(k.forward_kinematic_v2(qs));errors.append(float(np.linalg.norm(pose[:3]-pose2[:3])))
    pose1,mats=k.forward_kinematic_v1(q);matrix_errors.append(float(np.linalg.norm(mats[-1][:3,3]-pose[:3])));counts.append(len(sols))
out={'samples':100,'seed':20261115,'units':'mm','geometry':'DICE physical SDK modified DH; no external position measurement','inverse_forward_max_position_error':max(errors),'modified_dh_vs_explicit_max_position_error':max(matrix_errors),'max_candidate_solutions':max(counts),'limits_deg':[np.degrees(k.JOINT_ANGLE_LOWERB).tolist(),np.degrees(k.JOINT_ANGLE_UPPERB).tolist()]}
(ROOT/'evidence/kinematic_check.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps(out,ensure_ascii=False,indent=2))
