# Adaptive application-centric congestion control for AR/VR traffic over Wi-Fi (Stages 1–3)

This directory implements the first three implementation stages of the project
plan, all with the bottleneck moved **to the AP's Wi-Fi egress** (no narrow
point-to-point bottleneck): a wired content server cluster feeds an 802.11ax AP,
and all AR/VR downlink flows contend at the AP's Wi-Fi output, where a **DualQ
Coupled PI2 queue disc** (L4S-inspired ECN/AQM) is installed. The AP's Wi-Fi MAC
queue is kept shallow (40 packets) so the AQM, not the MAC, controls the backlog;
Wi-Fi contention and the configured PHY rate shape the achievable capacity.

> **Caveat (project framing).** These scenarios are an **L4S-inspired ECN/AQM
> Wi-Fi evaluation**. They do *not* implement full Accurate ECN / L4S behaviour
> at the Wi-Fi bottleneck: endpoints use classic ECE/CWR ECN feedback
> (`TcpSocketBase::EcnMode = ClassicEcn`, DCTCP-style), and the DualQ's L4S
> queue uses this tree's threshold-plus-coupled-probability marking. Describe
> the work accordingly unless full AccECN/L4S behaviour is implemented.

## Scenarios

| File | Stage | Purpose |
|------|-------|---------|
| `wifi-l4s-equal-share.cc` | 1 | Equal-share baseline: N identical backlogged DCTCP (ECT(1)) flows share the Wi-Fi bottleneck. |
| `wifi-appcentric-static.cc` | 2 | Static priorities: per-flow fixed Type-1 (Eq. 6) / Type-2 (Eq. 7) mappings, paper-style. |
| `wifi-appcentric-dynamic.cc` | 3 | Dynamic utility: a gaze process switches the priority between two object flows live. |

All three share the same topology: one server per flow on its own fast wired
access link (deliberately not a bottleneck) → 802.11ax AP (DualQ at Wi-Fi
egress) → one station per flow, in a ring around the AP. Traffic is downlink
(server → station), so every data packet passes the AP's Wi-Fi egress queue.

## Running

```sh
# Stage 1: equal share (4 flows, Jain ≈ 0.999)
./waf --run "wifi-l4s-equal-share --stations=4 --simTime=30"

# Stage 2: static priorities (defaults: A = 2.0 / 0.5, Type-1 gamma ±0.5)
./waf --run "wifi-appcentric-static --flows=4 --simTime=30"
./waf --run "wifi-appcentric-static --flows=4 --classicBackground=true"
./waf --run "wifi-appcentric-static --flows=4 --a0=4 --a1=0.25"

# Stage 3: gaze-driven priorities (flows 0/1 = objects A/B; 3-4 add audio/haptics)
./waf --run "wifi-appcentric-dynamic --flows=2 --simTime=40 --gazePeriod=10"
./waf --run "wifi-appcentric-dynamic --flows=4 --gazePeriod=8 --aTop=6 --classicBackground=true"
```

Common options: `--wifiStandard=802.11ax-5GHz|802.11ac|802.11n-5GHz`,
`--wifiMcs=<index>` (fixed rate; stage 4 will add adaptive rate),
`--apStaDistance=<m>`, `--classicBackground=true|false` (competing NewReno
flow), `--wifiAggregation=true|false` (default off; see implementation notes).

## Verified results (this tree, default settings)

* **Stage 1** (`stations=4`, MCS 0, 30 s): ~2.6–2.8 Mb/s per flow,
  Jain fairness **0.999**, 0 losses, DualQ: 10163 L4S marks, 0 drops.
  Marks — not drops — carry the congestion signal, as intended.
* **Stage 2** (`flows=4`, no background): correct priority ordering —
  focus-video (A=2) 36.2% > audio (γ=+0.5) 25.0% > haptics (γ=−0.5) 21.5% >
  other-video (A=0.5) 17.2%; Jain **0.926** (below 1 by design).
  With `--classicBackground=true` the classic flow takes most of the
  capacity (coexistence artifact of this tree's DCTCP-based L4S flows; a
  known limitation to document, not hide).
* **Stage 3** (`flows=2`, gaze every 10 s): the gazed flow reaches ~89% of
  the pair's throughput (A=4/0.5), gaze switches flip the shares, **mean
  adaptation time 4.667 s** (3/3 switches recovered), 0 losses.

## Implementation notes (why things are the way they are)

1. **Bottleneck at the AP egress.** A root `DualQCoupledPi2QueueDisc` is
   installed on the AP's `WifiNetDevice` (`tch.Install (apDev)`); the
   `NetDeviceQueueInterface` (aggregated by the Wi-Fi helpers) ensures the
   qdisc is actually used. The server–AP wired links are 100 Mb/s / 1 ms so
   only the Wi-Fi hop queues.
2. **Shallow MAC queue.** `ns3::WifiMacQueue::MaxQueueSize = 40p` on the AP
   side; a large MAC queue would hide the backlog from the AQM and produce
   uncontrolled queuing delay and MAC-queue drops.
3. **Aggregation off by default.** A-MPDU bursts enqueue several frames at
   once, so L4S-queue sojourn regularly exceeded the 475 µs marking threshold
   even below saturation; any flow that overshot the threshold got marked
   with probability 1 (this tree's `Laqm` is a step function) and collapsed
   to the minimum cwnd. With per-packet transmission the queue dynamics are
   smooth and differentiation behaves as expected. The trade-off (fewer
   aggregate bursts, lower effective efficiency at high MCS) is accepted for
   stages 1–3; revisiting with adaptive rate is stage-4 work.
4. **No runtime congestion-control swaps.** Replacing a socket's congestion
   control mid-connection discards the DCTCP α state (α≈0 ⇒ ECE-triggered
   `ReduceCwnd` becomes a no-op), which broke convergence. Stage 2 therefore
   configures each flow's `TcpPragueAppCentric` at **socket creation** via
   the per-node `TcpL4Protocol::SocketType` plus `Config::SetDefault`s
   scheduled just before each app starts (one sender node per flow, so the
   defaults never race). Stage 3 installs its own ops object once, right
   after socket creation (before any marks flow), and then only mutates the
   `AppCentricA` attribute live — attributes are re-read at every ECN-driven
   reduction, so gaze updates take effect immediately with α history intact.
5. **Command-line convention.** `--typeI` is 1 or 2 as in the paper (Eq. 6 /
   Eq. 7); internally these map to `TYPE1`/`TYPE2` (whose C++ enum values
   differ from the paper numbering).

## Not yet implemented (stages 4–5)

Station mobility, adaptive rate (Minstrel), interference, dynamic background
traffic (stage 4); and the full evaluation harness (QoE-weighted utility,
adaptation-time sweeps, comparison against Cubic/NewReno legacy baselines;
stage 5). This tree has no `TcpCubic`, so the legacy baseline is NewReno.
