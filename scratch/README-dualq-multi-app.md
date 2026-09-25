# DualQ: simultaneous applications from one node

`dualq-multi-app-same-node.cc` creates this topology:

```
node 0 (one sender, N TCP applications) -- 100 Mb/s -- node 1 (DualQ) -- 10 Mb/s, 20 ms -- node 2 (N sinks)
```

Every application is installed on node 0, uses a separate TCP port/connection,
and starts at the same `appStart` time.  The DualQ Coupled PI2 queue disc is
installed only on node 1's outbound bottleneck device.

## Requirements

* Linux/macOS with a C++ compiler, Python 3, and `make`.
* This ns-3 tree configured with the required modules.  The default build
  includes them; enable examples as well if the tree has not yet been built.

From the root of this repository:

```sh
./waf configure --enable-examples
./waf build
```

## Run

```sh
./waf --run "dualq-multi-app-same-node --nApps=4 --appRate=5Mbps --l4s=1"
```

The command starts four applications at `1 s` from node 0.  With `5Mbps` per
application, their combined offered load is 20 Mb/s, which exceeds the 10 Mb/s
bottleneck and exercises DualQ.  The final output lists bytes received for
every application and the queue-disc statistics.

Useful variations:

```sh
# Classic ECN/NewReno flows instead of L4S/DCTCP flows
./waf --run "dualq-multi-app-same-node --nApps=4 --l4s=0"

# Six applications, optional packet capture and FlowMonitor XML
./waf --run "dualq-multi-app-same-node --nApps=6 --appRate=3Mbps --writePcap=1 --writeFlowMonitor=1 --outputPrefix=results/dualq"

# Show all supported options
./waf --run "dualq-multi-app-same-node --PrintHelp"
```

Create the `results` directory before using the last command.  `l4s=1` uses
TCP DCTCP, which sends ECT(1) packets and is placed in the L4S queue by this
repository's DualQ implementation.  `l4s=0` uses TCP NewReno and is placed in
the Classic queue.
