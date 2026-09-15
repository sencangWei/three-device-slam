# Docker2 stationary endpoint loop-verification candidate

Date: 2026-09-04  
Mode: isolated candidate build plus recorded-data replay  
Verdict: recorded-data candidate PASS; frozen release unchanged; RK3576 live deployment not yet performed

## Problem and correction

The lossless 70-second run stopped producing real VINS keyframes 33.436 seconds before capture ended. The final static interval therefore could not advance the existing `2/3` loop consensus, even though fresh 30 Hz stereo frames and accepted VIO poses continued to arrive.

The candidate publishes a fresh stereo verification probe every 0.75 seconds after real keyframes stop. A probe is query-only: it is never inserted into the DBoW database or pose graph, never increments the graph index, and can only finish an already-established pending loop. The accepted constraint remains owned by the latest real VINS keyframe.

Safety properties:

- The existing `LoopKeyFrame` wire layout remains unchanged. Probes use a separate reliable `/loop_fusion/verification_probe` message and topic, defined identically in the live and offline workspaces.
- The three-observation requirement remains unchanged. Every probe uses a fresh image/time sample and repeats DBoW retrieval, left-camera PnP, right-IR geometry, correction agreement, retrieval, correction-budget, and cooldown checks.
- A probe must remain within 20 mm and 1 degree of its owning real keyframe. Owner graph index and timestamp must both match the latest real keyframe.
- An intervening real keyframe that does not continue the candidate clears pending consensus. Stale probes fail closed.
- The derived owner edge is checked for finite values and the existing translation/yaw limits before publication. Updating a listed keyframe uses the same graph mutex as optimizer/path readers.

## Build and unit evidence

Isolated worktree: `/home/robot/worktrees/ego-loop-probe-20260904`  
Build root: `/home/robot/worktrees/ego-loop-probe-20260904/.loop_probe_build`

| Suite | Result |
|---|---:|
| Product-live VINS workspace | 29 tests, 0 failures |
| Offline/product-loop workspace | 51 tests, 0 failures |
| Independent second-pass review | 0 Critical, 0 Important |

Candidate binaries:

- Product-live VINS SHA-256: `51c7b913487a3b584d85e6d69de81985b93441553cdf78d39a00cb723d41090f`
- Offline 30 Hz VINS SHA-256: `ca75d47ea58933d5dcdbc7b6e761395ee4b8ebc4d9b920ffa3c2538a8c76b461`
- Loop fusion SHA-256: `595f776e7a6192c9ccf01a076531b650440d8ce2f6a1284dad3a757bc6a730fd`

The build emitted only the inherited pose-graph VLA/sign-comparison warnings; there were no compile errors.

## Positive regression: former `2/3` lossless run

Source: `/tmp/umi-v3-lossless-042624-slam-export`  
Run: `/tmp/umi-v3-lossless-042624-probe-candidate-5`  
Report SHA-256: `908c2ae93a2cc6a75d3cf81daed2aaabf1f0b7e6622a58639d1f9ba2a50bad79`

| Metric | Result |
|---|---:|
| Verdict / watchdog | PASS / `SLAM_HEALTHY` |
| Raw/corrected poses | 2,043 / 2,042 |
| Pose coverage | 99.3674% |
| Real graph keyframes / probes | 112 / 49 |
| Accepted automatic loops | 1 |
| Confirmation sequence | real keyframe 111: `1/3`; probes 6 and 7: `2/3`, `3/3` |
| Probe-to-owner displacement at acceptance | 3.0 mm |
| Input/backlog drops | 0 / 0 |
| Minimum tracked features | 42 |
| Maximum raw step | 7.339 mm |
| Corrected endpoint error | 5.423 mm |

The accepted log is `current=111 matched=0 confirmations=3 source=verification_probe`. The third observation arrived 1.534 seconds after the real owner keyframe. Only 112 real keyframes exist in the final graph; all 49 probes remained non-vertices.

## Negative control

Source: `/tmp/umi-v4-motion-slam-export`  
Run: `/tmp/umi-v4-motion-probe-negative-3`  
Report SHA-256: `7547ee0aea310e833461729cad4fced050021e378f449095130837faf6005699`

| Metric | Result |
|---|---:|
| Verdict / watchdog | PASS / `SLAM_HEALTHY` |
| Pose coverage | 98.5965% |
| Accepted automatic loops | 0 |
| Input/backlog drops | 0 / 0 |
| Minimum tracked features | 174 |
| Maximum raw step | 8.579 mm |

## Product-live cross-workspace smoke evidence

The exact split used by `product-live` was exercised: VINS from `vins_ws` and loop fusion from `loop_ws`. The two topics matched, 106 real keyframes plus 47 probes arrived with zero transport/backlog drops, and one verified loop was accepted. The generic offline report is intentionally retained as FAIL because that runner expects 30 Hz pose output while this existing live VINS path outputs about 15 Hz (`49.44%` of a 30 Hz expectation). It proves the repaired message transport but is not claimed as a 30 Hz acceptance result.

Run: `/tmp/umi-v3-lossless-042624-probe-product-live-1`  
Report SHA-256: `09facbdf362aecf96b40bf003d50c01fe0b8843d274f2b99d389522286ce2aa3`

## Frozen-release protection and remaining gate

No frozen release binary was overwritten. Its offline VINS and loop-fusion hashes remain:

- VINS: `5059b188401c28c44fac2808b48374c142f466d34b168c287fe31e578a037973`
- Loop fusion: `9db635f796164565b9b539fb54ccd3449611e23d9955c8f1d5e9a971ad88f333`

Before promotion, the candidate still requires an RK3576 live hardware run with D405 and STM32, followed by the normal versioned release/rollback procedure. The current result authorizes a candidate, not replacement of the frozen production package.
