import time, numpy as np, cv2, torch
print("torch", torch.__version__)
t=time.time()
m = torch.jit.load("yolopv2.pt", map_location="cpu").float().eval()
print(f"loaded in {time.time()-t:.1f}s")

def letterbox(img, new_shape=(384,640), color=(114,114,114), stride=32, auto=True):
    shape = img.shape[:2]
    r = min(new_shape[0]/shape[0], new_shape[1]/shape[1])
    new_unpad = (int(round(shape[1]*r)), int(round(shape[0]*r)))
    dw, dh = new_shape[1]-new_unpad[0], new_shape[0]-new_unpad[1]
    if auto:
        dw, dh = np.mod(dw,stride), np.mod(dh,stride)
    dw/=2; dh/=2
    if shape[::-1]!=new_unpad:
        img=cv2.resize(img,new_unpad,interpolation=cv2.INTER_LINEAR)
    top,bottom=int(round(dh-0.1)),int(round(dh+0.1))
    left,right=int(round(dw-0.1)),int(round(dw+0.1))
    img=cv2.copyMakeBorder(img,top,bottom,left,right,cv2.BORDER_CONSTANT,value=color)
    return img,(top,bottom,left,right)

# Replicate demo: resize to 1280x720 then letterbox to 640 (auto -> 384x640)
bgr0 = cv2.imread("YOLOPv2/data/demo/lane1.jpg")
bgr = cv2.resize(bgr0,(1280,720))
lb, pads = letterbox(bgr,(640,640))   # demo uses img_size=640 square target, auto-letterbox
print("letterboxed shape", lb.shape, "pads(t,b,l,r)", pads)
rgb = lb[:,:,::-1].transpose(2,0,1)
x = np.ascontiguousarray(rgb).astype(np.float32)/255.0
x = torch.from_numpy(x).unsqueeze(0)
print("input tensor", tuple(x.shape))

t=time.time()
with torch.no_grad():
    out = m(x)
print(f"forward {time.time()-t:.2f}s ; n_outputs={len(out)}")
det, seg, ll = out
print("  det type:", type(det), "len" , len(det) if isinstance(det,(list,tuple)) else "-")
print("  seg (drivable) shape:", tuple(seg.shape), "min/max", float(seg.min()), float(seg.max()))
print("  ll  (lane)     shape:", tuple(ll.shape),  "min/max", float(ll.min()),  float(ll.max()))

# Decode lane exactly like utils.lane_line_mask but generic
top,bottom,left,right = pads
H,W = ll.shape[2], ll.shape[3]
ll_crop = ll[:,:,top:H-bottom, left:W-right]      # strip letterbox pad
prob = ll_crop[0,0].cpu().numpy()                  # (1,1,h,w) -> (h,w), already 0..1
print("  ll prob after crop:", prob.shape, "frac>0.5 =", float((prob>0.5).mean()))
binmask = (prob>0.5).astype(np.uint8)*255
binmask = cv2.resize(binmask,(bgr0.shape[1],bgr0.shape[0]),interpolation=cv2.INTER_NEAREST)
overlay = bgr0.copy(); overlay[binmask>0]=(0,255,255)
cv2.imwrite("demo_lane1_overlay.png", overlay)
cv2.imwrite("demo_lane1_mask.png", binmask)
print("frac lane px on demo image:", float((binmask>0).mean()))
print("WROTE demo_lane1_overlay.png")
