import sys, glob, cv2
sys.path.insert(0,'/home/mspacman/IGVC_ROS2/src/avros_perception')
sys.path.insert(0,'/home/mspacman/IGVC_ROS2/docs/cv_sooner25_tuning_2026_06_01')
from avros_perception.pipelines.sooner25 import Sooner25Pipeline
from sooner25_eval import AUGS, metrics, DEFAULT_ROI as ROI
frames=sorted(glob.glob('frames/all/*.png'))
AUGSEQ=['none','bright_up','bright_down','gamma_hi','warp_tilt','rot','wb_warm','shadow']
def run_cfg(params,label):
    print(f"\n=== {label} ===")
    worst=0; wa=None
    for aug in AUGSEQ:
        fn=AUGS[aug]; pcts=[]
        for f in frames:
            b=cv2.imread(f)
            if b is None: continue
            res=Sooner25Pipeline({**params,'sky_roi_poly':ROI,'class_id_lane':1}).run(fn(b))
            pcts.append(metrics(res.mask,ROI)['lane_pct'])
        mx=max(pcts); mean=sum(pcts)/len(pcts)
        if mx>worst: worst=mx; wa=aug
        print(f"  {aug:11s} mean {mean:5.2f}  max {mx:6.2f}  {'FLOOD' if mx>5 else 'ok'}")
    print(f"  >>> WORST {worst:.2f}% @ {wa}  -> {'ROBUST (no flood)' if worst<=5 else 'NOT robust'}")
run_cfg({'sooner25_upper':[255,255,195],'sooner25_adaptive':'none'},"BASELINE fixed V195 (current prod)")
run_cfg({'sooner25_upper':[255,255,185],'sooner25_adaptive':'p93','sooner25_vfloor':185,
         'sooner25_vcap':255,'sooner25_band':[0.40,1.0],'sooner25_post_sdrop':80},
        "RECOMMENDED floored-adaptive p93 + Sdrop80 (REAL updated pipeline)")
