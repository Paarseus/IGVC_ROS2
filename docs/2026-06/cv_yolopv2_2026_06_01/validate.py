import numpy as np, cv2, time, glob, os
import onnxruntime as ort
import torch

OUT="/home/mspacman/IGVC_ROS2/docs/cv_yolopv2_2026_06_01/validation"
os.makedirs(OUT, exist_ok=True)

def letterbox(img, new_shape=(384,640), color=(114,114,114), stride=32, auto=True):
    shape=img.shape[:2]
    r=min(new_shape[0]/shape[0], new_shape[1]/shape[1])
    new_unpad=(int(round(shape[1]*r)), int(round(shape[0]*r)))
    dw,dh=new_shape[1]-new_unpad[0], new_shape[0]-new_unpad[1]
    if auto: dw,dh=np.mod(dw,stride),np.mod(dh,stride)
    dw/=2; dh/=2
    if shape[::-1]!=new_unpad: img=cv2.resize(img,new_unpad,interpolation=cv2.INTER_LINEAR)
    top,bottom=int(round(dh-0.1)),int(round(dh+0.1)); left,right=int(round(dw-0.1)),int(round(dw+0.1))
    img=cv2.copyMakeBorder(img,top,bottom,left,right,cv2.BORDER_CONSTANT,value=color)
    return img,(top,bottom,left,right)

def preprocess(bgr, pre_resize=(1280,720)):
    src = cv2.resize(bgr, pre_resize) if pre_resize else bgr
    lb, pads = letterbox(src,(384,640))
    rgb = lb[:,:,::-1].transpose(2,0,1)
    x = np.ascontiguousarray(rgb).astype(np.float32)/255.0
    return x[None], pads

# ONNX session: request lane(759) + drivable(677) only -> prunes detection head
so = ort.SessionOptions()
sess = ort.InferenceSession("yolopv2_384x640.onnx", so, providers=["CPUExecutionProvider"])
out_shapes = {o.name:o.shape for o in sess.get_outputs()}
lane_name = next(n for n,s in out_shapes.items() if list(s)==[1,1,384,640])
driv_name = next(n for n,s in out_shapes.items() if list(s)==[1,2,384,640])
print("lane output:", lane_name, "  drivable output:", driv_name)

mjit = torch.jit.load("yolopv2.pt", map_location="cpu").float().eval()

def decode_lane(ll, pads, orig_hw, thr):
    top,bottom,left,right = pads
    H,W = ll.shape[2], ll.shape[3]
    crop = ll[0,0, top:H-bottom, left:W-right]
    binm = (crop>thr).astype(np.uint8)
    return cv2.resize(binm,(orig_hw[1],orig_hw[0]),interpolation=cv2.INTER_NEAREST)

def run(bgr, thr=0.5, pre_resize=(1280,720)):
    x,pads = preprocess(bgr, pre_resize)
    ll,da = sess.run([lane_name,driv_name], {"images":x})
    lane = decode_lane(ll,pads,bgr.shape[:2],thr)
    # drivable argmax
    top,bottom,left,right=pads; H,W=da.shape[2],da.shape[3]
    dac=np.argmax(da[0],0)[top:H-bottom,left:W-right].astype(np.uint8)
    driv=cv2.resize(dac,(bgr.shape[1],bgr.shape[0]),interpolation=cv2.INTER_NEAREST)
    return lane, driv, ll, pads

# ---- parity check on demo image ----
demo = cv2.resize(cv2.imread("YOLOPv2/data/demo/lane1.jpg"),(1280,720))
x,pads = preprocess(demo, None)   # demo already 1280x720
ll_onnx,_ = sess.run([lane_name,driv_name],{"images":x})
with torch.no_grad():
    _,_,ll_torch = mjit(torch.from_numpy(x))
ll_torch=ll_torch.numpy()
print(f"PARITY torch vs onnx |maxdiff| = {np.abs(ll_onnx-ll_torch).max():.2e}")

# ---- IGVC frames ----
frames = [
  ("demo_lane1","YOLOPv2/data/demo/lane1.jpg"),
  ("igvc_exp_f00","/home/mspacman/IGVC_ROS2/exp_frames/f00_v134.png"),
  ("igvc_rgb001","/home/mspacman/IGVC_ROS2/docs/cv_adaptive_debug_2026_05_31/input_rgb_001.png"),
  ("igvc_rgb018","/home/mspacman/IGVC_ROS2/docs/cv_adaptive_debug_2026_05_31/input_rgb_018.png"),
  ("igvc_rgb044","/home/mspacman/IGVC_ROS2/docs/cv_adaptive_debug_2026_05_31/input_rgb_044.png"),
  ("igvc_rgbnow","/home/mspacman/IGVC_ROS2/rgb_now.png"),
]
print(f"\n{'frame':16s} {'lane%@0.5':>9s} {'lane%@0.3':>9s} {'lane%@0.1':>9s} {'driv%':>7s}")
for name,path in frames:
    bgr=cv2.imread(path)
    if bgr is None: print(name,"MISSING",path); continue
    lane5,driv,ll,pads = run(bgr,0.5)
    lane3 = decode_lane(ll,pads,bgr.shape[:2],0.3)
    lane1 = decode_lane(ll,pads,bgr.shape[:2],0.1)
    print(f"{name:16s} {100*lane5.mean():9.3f} {100*lane3.mean():9.3f} {100*lane1.mean():9.3f} {100*(driv>0).mean():7.2f}")
    ov=bgr.copy(); ov[lane5>0]=(0,255,255)                     # yellow = lane @0.5
    ov[(lane3>0)&(lane5==0)]=(0,128,255)                        # orange = extra @0.3
    cv2.imwrite(f"{OUT}/{name}_lane.png", ov)
    dv=bgr.copy(); dv[driv>0]=(0,200,0)
    cv2.imwrite(f"{OUT}/{name}_drivable.png", cv2.addWeighted(bgr,0.5,dv,0.5,0))
print("\noverlays written to", OUT)
