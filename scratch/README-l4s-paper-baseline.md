# Paper-style L4S equal-share baseline

This scenario recreates the conditions used for the paper's default equal-share
argument: `N` separate client/server pairs each send an identical, persistent,
backlogged L4S connection through one shared bottleneck. Every path has a 20-ms
base RTT. The program discards a 10-second TCP startup period and reports each
flow's throughput over the following 15 seconds, as the paper does.

```sh
./waf --run "l4s-paper-baseline --flows=4 --bottleneckRate=10Mbps"
```

It uses ns-3 TCP DCTCP with ECT(1), because this workspace does not include the
paper's Linux reference TCP Prague implementation. The experimental condition is
otherwise the same baseline: identical persistent L4S connections, one shared
DualQ bottleneck, and a steady-state measurement window.

## Application-centric differentiation (paper Type 2)

Make one connection application-centric with `A=2` while the other three use
reference L4S:

```sh
./waf --run "l4s-paper-baseline --flows=4 --bottleneckRate=10Mbps --modifiedFlow=0 --modifiedA=2"
```

For the paper's *reference TCP Prague* model, Equation (7) predicts the modified
connection will receive roughly twice each reference connection's throughput: at
10 Mb/s, about 4 Mb/s for flow 0 and 2 Mb/s for each remaining flow. This
workspace uses DCTCP as an approximation, not reference Prague, so its measured
allocation can differ substantially; use this scenario to test the implemented
mapping, not to claim the paper's quantitative result. `A=1` restores reference
behavior; values below 1 make the selected flow less aggressive.

## Application-centric differentiation (paper Type 1)

```sh
./waf --run "l4s-paper-baseline --flows=4 --modifiedFlow=0 --modifiedType=1 --modifiedP0=0.5 --modifiedGamma=0.5"
```

Type 1 implements Equation (6). Positive `gamma` makes the selected connection
more aggressive below `p0` and less aggressive above it; negative `gamma`
reverses that policy. `gamma=0` is the reference mapping.
