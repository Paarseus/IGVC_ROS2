<!-- Source: https://github.com/wpilibsuite/sysid/issues/258#issuecomment-1010658237 (GitHub issue comment by Piphi5, 2022-01-12T05:44:00Z; fetched 2026-09-27 via GitHub API) -->

For the sake of documenting this, here's the official response from REV Support (Note: REV is working on lowering this latency so this may be subject to change in the future):

```
 That sensor is sampled every 32ms, and the sampling window is 8 samples, which means that the sampling window is 256ms. Also, the hall sensor decoding does not use a backward finite difference (unlike the quadrature encoder decoding), so I believe the formula that should be used here is (N - 1)/2 * T.

(8 - 1)/2 * 32 = 112ms

The latency of getting the hall sensor's velocity is something that we want to improve in the future.

```
