# Mixed L4S video-call traffic

Run the scenario with:

```sh
./waf --run "l4s-video-call-traffic --bottleneckRate=10Mbps"
```

Four independent TCP DCTCP (ECT(1)/L4S) connections share one DualQ bottleneck:
video (8 Mb/s), audio (128 kb/s), chat (64 kb/s), and image/file transfer
(8 Mb/s). The source bytes are synthetic: ns-3 models the transmission pattern,
not media codecs or real file contents.

The low-rate audio and chat connections should obtain their requested rates. The
two persistently backlogged high-rate connections compete for the remaining
capacity. This represents why application traffic does not normally get an equal
*application-type* allocation: only flows asking for more than a fair share need
congestion-controlled sharing.
