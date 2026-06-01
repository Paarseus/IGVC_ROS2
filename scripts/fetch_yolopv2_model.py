#!/usr/bin/env python3
"""
Fetch + export the YOLOPv2 lane-line model to ONNX for avros_perception.

Downloads the official TorchScript weights (CAIC-AD/YOLOPv2, MIT licence) and
exports a fixed-shape ONNX whose lane-line head feeds the `yolopv2` pipeline
(pipelines/yolopv2.py). Run this on any box with torch + onnx (e.g. the dev
laptop); the Jetson only needs onnxruntime-gpu to RUN the resulting .onnx.

    python3 scripts/fetch_yolopv2_model.py --out ~/yolopv2_lane_384x640.onnx

Then on the Jetson:
    scp ~/yolopv2_lane_384x640.onnx jetson:~/
    # perception.yaml: pipeline: 'yolopv2'  +  yolopv2_model_path: '~/yolopv2_lane_384x640.onnx'

The exported model has THREE+ outputs (the detection head traces to a few
stray tensors); the pipeline auto-selects the lane head by shape (1,1,H,W), so
the extra outputs are harmless (and pruned at inference since we request only
the lane output). The detection head is exported but never run.

Verified 2026-06-01: torch 2.11 CPU export, opset 12, ONNX==torch parity 2.6e-5.
"""

import argparse
import os
import sys
import urllib.request

WEIGHTS_URL = 'https://github.com/CAIC-AD/YOLOPv2/releases/download/V0.0.1/yolopv2.pt'


def download(url, dst):
    if os.path.isfile(dst) and os.path.getsize(dst) > 1_000_000:
        print(f'[skip] weights already present: {dst} '
              f'({os.path.getsize(dst) / 1e6:.0f} MB)')
        return
    print(f'[download] {url}\n        -> {dst}')

    def _hook(blk, bs, total):
        if total > 0:
            pct = min(100, blk * bs * 100 // total)
            sys.stdout.write(f'\r  {pct:3d}%')
            sys.stdout.flush()

    urllib.request.urlretrieve(url, dst, _hook)
    print(f'\n[done] {os.path.getsize(dst) / 1e6:.0f} MB')


def export(weights, out, h, w):
    import torch
    print(f'[load] {weights}')
    model = torch.jit.load(weights, map_location='cpu').float().eval()
    dummy = torch.zeros(1, 3, h, w)

    # Sanity: confirm the three heads and the lane shape BEFORE exporting.
    with torch.no_grad():
        det, seg, ll = model(dummy)
    print(f'[heads] drivable seg={tuple(seg.shape)}  lane ll={tuple(ll.shape)}')
    assert ll.shape[1] == 1, f'unexpected lane head shape {tuple(ll.shape)}'

    print(f'[export] -> {out}  (opset 12, fixed 1x3x{h}x{w})')
    torch.onnx.export(
        model, dummy, out,
        input_names=['images'],
        opset_version=12,
        do_constant_folding=True,
        dynamo=False,            # legacy tracer; the new dynamo path chokes on jit ScriptModule
    )

    # Report outputs + (optional) verify the lane head + ORT parity.
    try:
        import onnx
        m = onnx.load(out)
        onnx.checker.check_model(m)
        print('[onnx] outputs:')
        lane = None
        for o in m.graph.output:
            dims = [d.dim_value for d in o.type.tensor_type.shape.dim]
            tag = ''
            if dims == [1, 1, h, w]:
                tag, lane = '  <-- LANE (used)', o.name
            elif dims == [1, 2, h, w]:
                tag = '  <-- drivable (unused)'
            print(f'   {o.name} {dims}{tag}')
        if lane is None:
            print('[warn] no (1,1,H,W) lane output found — check the export')
    except ImportError:
        print('[onnx] python `onnx` not installed; skipped graph check')

    try:
        import numpy as np
        import onnxruntime as ort
        sess = ort.InferenceSession(out, providers=['CPUExecutionProvider'])
        lane_name = next(o.name for o in sess.get_outputs()
                         if list(o.shape) == [1, 1, h, w])
        x = np.random.rand(1, 3, h, w).astype(np.float32)
        o_onnx = sess.run([lane_name], {'images': x})[0]
        with torch.no_grad():
            _, _, o_torch = model(torch.from_numpy(x))
        diff = float(np.abs(o_onnx - o_torch.numpy()).max())
        print(f'[verify] ONNX vs torch lane |maxdiff| = {diff:.2e} '
              f'({"OK" if diff < 1e-3 else "HIGH — investigate"})')
    except ImportError:
        print('[verify] onnxruntime not installed; skipped parity check')

    print(f'[ok] wrote {out}  ({os.path.getsize(out) / 1e6:.0f} MB)')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--weights', default='yolopv2.pt',
                    help='TorchScript weights path (downloaded if missing)')
    ap.add_argument('--out', default='yolopv2_lane_384x640.onnx',
                    help='output ONNX path')
    ap.add_argument('--url', default=WEIGHTS_URL, help='weights download URL')
    ap.add_argument('--img-h', type=int, default=384)
    ap.add_argument('--img-w', type=int, default=640)
    args = ap.parse_args()

    args.weights = os.path.expanduser(args.weights)
    args.out = os.path.expanduser(args.out)
    download(args.url, args.weights)
    export(args.weights, args.out, args.img_h, args.img_w)


if __name__ == '__main__':
    main()
