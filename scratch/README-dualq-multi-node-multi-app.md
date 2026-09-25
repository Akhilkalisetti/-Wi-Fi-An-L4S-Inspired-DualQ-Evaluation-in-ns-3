# DualQ: multiple applications on multiple nodes

`dualq-multi-node-multi-app.cc` creates `nPairs` pairs of endpoint nodes. Each
endpoint runs `nApps` independent TCP OnOff applications at the same time,
sending to its paired endpoint. Thus every endpoint is both a sender and a
receiver. All traffic crosses the bidirectional DualQ bottleneck between two
routers.

```
left host 0..N -- 100 Mb/s -- router 0 == DualQ, 10 Mb/s, 20 ms == router 1 -- 100 Mb/s -- right host 0..N
```

## Build and run

From the repository root:

```sh
./waf configure --enable-examples
./waf build
./waf --run "dualq-multi-node-multi-app --nPairs=3 --nApps=4 --appRate=3Mbps --l4s=1"
```

The example above creates six endpoint nodes. Every endpoint runs four clients
from `1 s` to `19 s`; each node therefore offers 12 Mb/s towards its peer,
which exceeds the 10 Mb/s bottleneck. The program prints bytes received at
each endpoint and DualQ statistics for both bottleneck directions.

Useful options:

```sh
# Classic traffic instead of L4S/DCTCP
./waf --run "dualq-multi-node-multi-app --nPairs=2 --nApps=3 --l4s=0"

# List every supported setting
./waf --run "dualq-multi-node-multi-app --PrintHelp"
```

`l4s=1` selects DCTCP, whose ECT(1) packets enter DualQ's L4S queue. `l4s=0`
selects NewReno, whose traffic enters the Classic queue.
