# Equal sharing between L4S connections

`l4s-equal-connection-share.cc` demonstrates the paper's default-case claim:
otherwise identical, continuously backlogged L4S connections sharing one
bottleneck receive approximately equal throughput.

Run from the repository root:

```sh
./waf --run "l4s-equal-connection-share --connections=4 --bottleneckRate=10Mbps"
```

The program creates four TCP DCTCP connections from one source to one receiver.
Their ECT(1) packets traverse one DualQ Coupled PI2 L4S bottleneck. It prints
each connection's throughput and percentage deviation from the mean. At 10 Mb/s,
the expected starting point is about 2.5 Mb/s per connection.

This is an experimental demonstration of equal share **per connection**, under
equal conditions. It does not establish equal share per application: an
application with four connections receives approximately four shares; one with
one connection receives one share.
