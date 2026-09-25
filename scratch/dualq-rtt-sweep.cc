/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Fixed-demand RTT sweep for DualQ / L4S experiments.
 *
 * Example:
 *   ./waf --run "dualq-rtt-sweep --bandwidth=10Mbps --offeredLoad=50Mbps"
 *
 * This writes a CSV and four dependency-free SVG plots.  The SVG files open
 * directly in a browser, an image viewer, or a report editor.
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/traffic-control-module.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("DualQRttSweep");

struct Result
{
  double baseRttMs;
  std::string kind;
  double throughputMbps;
  double achievedRttMs;
  double queueDelayMs;
  double markRatePercent;
  double lossRatePercent;
  uint64_t offeredPackets;
  uint64_t marks;
  uint64_t drops;
};

struct Measurements
{
  Time begin;
  Time end;
  double sojournSumMs {0.0};
  uint64_t sojournSamples {0};
  double rttSumMs {0.0};
  uint64_t rttSamples {0};
  uint64_t enqueues {0};
  uint64_t marks {0};
  uint64_t drops {0};
  uint64_t rxAtBegin {0};
};

static bool
InMeasurementWindow (const Measurements *m)
{
  return Simulator::Now () >= m->begin && Simulator::Now () < m->end;
}

static void
SojournTrace (Measurements *m, Time sojourn)
{
  if (InMeasurementWindow (m))
    {
      m->sojournSumMs += sojourn.GetMilliSeconds ();
      ++m->sojournSamples;
    }
}

static void
RttTrace (Measurements *m, Time, Time rtt)
{
  if (InMeasurementWindow (m) && !rtt.IsZero ())
    {
      m->rttSumMs += rtt.GetMilliSeconds ();
      ++m->rttSamples;
    }
}

static void
EnqueueTrace (Measurements *m, Ptr<const QueueDiscItem>)
{
  if (InMeasurementWindow (m))
    {
      ++m->enqueues;
    }
}

static void
MarkTrace (Measurements *m, Ptr<const QueueDiscItem>, const char *)
{
  if (InMeasurementWindow (m))
    {
      ++m->marks;
    }
}

static void
DropTrace (Measurements *m, Ptr<const QueueDiscItem>)
{
  if (InMeasurementWindow (m))
    {
      ++m->drops;
    }
}

// SocketList is populated when the OnOff application starts.  Connect just
// afterwards so RTT samples come from the sender socket, not the receiver.
static void
ConnectRttTrace (Measurements *m)
{
  Config::ConnectWithoutContext (
    "/NodeList/0/$ns3::TcpL4Protocol/SocketList/*/RTT",
    MakeBoundCallback (&RttTrace, m));
}

static void
TakeRxSnapshot (Measurements *m, Ptr<PacketSink> sink)
{
  m->rxAtBegin = sink->GetTotalRx ();
}

static Result
RunOne (double baseRttMs,
        bool l4s,
        const std::string &bandwidth,
        const std::string &offeredLoad,
        double duration,
        double warmup,
        uint32_t packetSize)
{
  const double start = 1.0;
  const double stop = start + duration;
  Measurements m;
  m.begin = Seconds (start + warmup);
  m.end = Seconds (stop);

  // Classic is intentionally not ECN-capable, so its congestion signals are
  // drops.  DCTCP uses ECT(1), which DualQ directs to the L4S queue and marks.
  Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (packetSize));
  Config::SetDefault ("ns3::TcpSocket::DelAckCount", UintegerValue (1));
  Config::SetDefault ("ns3::TcpSocketBase::EcnMode",
                      StringValue (l4s ? "ClassicEcn" : "NoEcn"));
  Config::SetDefault ("ns3::DualQCoupledPi2QueueDisc::Mtu", UintegerValue (packetSize));
  Config::SetDefault ("ns3::DualQCoupledPi2QueueDisc::QueueLimit", UintegerValue (100000));

  NodeContainer nodes;
  nodes.Create (4);
  Ptr<Node> sender = nodes.Get (0);
  Ptr<Node> leftRouter = nodes.Get (1);
  Ptr<Node> rightRouter = nodes.Get (2);
  Ptr<Node> receiver = nodes.Get (3);

  InternetStackHelper internet;
  internet.Install (nodes);
  Config::Set ("/NodeList/0/$ns3::TcpL4Protocol/SocketType",
               TypeIdValue (l4s ? TcpDctcp::GetTypeId () : TcpNewReno::GetTypeId ()));

  PointToPointHelper access;
  access.SetDeviceAttribute ("DataRate", StringValue ("1Gbps"));
  access.SetChannelAttribute ("Delay", StringValue ("0ms"));
  NetDeviceContainer senderLink = access.Install (sender, leftRouter);
  NetDeviceContainer receiverLink = access.Install (rightRouter, receiver);

  PointToPointHelper bottleneck;
  bottleneck.SetDeviceAttribute ("DataRate", StringValue (bandwidth));
  // The same p2p propagation delay is traversed by data and ACKs, hence the
  // requested base RTT is represented by baseRtt/2 in each direction.
  std::ostringstream oneWayDelay;
  oneWayDelay << std::setprecision (12) << baseRttMs / 2.0 << "ms";
  bottleneck.SetChannelAttribute ("Delay", StringValue (oneWayDelay.str ()));
  bottleneck.SetQueue ("ns3::DropTailQueue", "MaxSize", StringValue ("1p"));
  NetDeviceContainer routerLink = bottleneck.Install (leftRouter, rightRouter);

  TrafficControlHelper dualq;
  uint16_t handle = dualq.SetRootQueueDisc ("ns3::DualQCoupledPi2QueueDisc");
  dualq.AddInternalQueues (handle, 2, "ns3::DropTailQueue", "MaxSize", StringValue ("100000B"));
  QueueDiscContainer qdiscs = dualq.Install (routerLink.Get (0));
  Ptr<QueueDisc> qdisc = qdiscs.Get (0);
  qdisc->TraceConnectWithoutContext ("Enqueue", MakeBoundCallback (&EnqueueTrace, &m));
  qdisc->TraceConnectWithoutContext ("Mark", MakeBoundCallback (&MarkTrace, &m));
  qdisc->TraceConnectWithoutContext ("Drop", MakeBoundCallback (&DropTrace, &m));
  qdisc->TraceConnectWithoutContext (l4s ? "L4sSojournTime" : "ClassicSojournTime",
                                     MakeBoundCallback (&SojournTrace, &m));

  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  ipv4.Assign (senderLink);
  ipv4.SetBase ("10.1.2.0", "255.255.255.0");
  ipv4.Assign (routerLink);
  ipv4.SetBase ("10.1.3.0", "255.255.255.0");
  Ipv4InterfaceContainer receiverIf = ipv4.Assign (receiverLink);
  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  const uint16_t port = 50000;
  PacketSinkHelper sinkHelper ("ns3::TcpSocketFactory",
                               InetSocketAddress (Ipv4Address::GetAny (), port));
  ApplicationContainer sinkApp = sinkHelper.Install (receiver);
  sinkApp.Start (Seconds (0));
  sinkApp.Stop (Seconds (stop + 1));
  Ptr<PacketSink> sink = DynamicCast<PacketSink> (sinkApp.Get (0));

  OnOffHelper source ("ns3::TcpSocketFactory",
                      InetSocketAddress (receiverIf.GetAddress (1), port));
  source.SetAttribute ("PacketSize", UintegerValue (packetSize));
  source.SetAttribute ("DataRate", DataRateValue (DataRate (offeredLoad)));
  source.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  source.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
  ApplicationContainer sourceApp = source.Install (sender);
  sourceApp.Start (Seconds (start));
  sourceApp.Stop (Seconds (stop));
  Simulator::Schedule (Seconds (start + 0.01), &ConnectRttTrace, &m);
  Simulator::Schedule (Seconds (start + warmup), &TakeRxSnapshot, &m, sink);

  Simulator::Stop (Seconds (stop + 1));
  Simulator::Run ();

  const double measureSeconds = duration - warmup;
  Result r;
  r.baseRttMs = baseRttMs;
  r.kind = l4s ? "L4S" : "Classic";
  r.throughputMbps = (sink->GetTotalRx () - m.rxAtBegin) * 8.0 / measureSeconds / 1e6;
  r.achievedRttMs = m.rttSamples ? m.rttSumMs / m.rttSamples : 0.0;
  r.queueDelayMs = m.sojournSamples ? m.sojournSumMs / m.sojournSamples : 0.0;
  r.markRatePercent = m.enqueues ? 100.0 * m.marks / m.enqueues : 0.0;
  r.lossRatePercent = m.enqueues ? 100.0 * m.drops / m.enqueues : 0.0;
  r.offeredPackets = m.enqueues;
  r.marks = m.marks;
  r.drops = m.drops;
  Simulator::Destroy ();
  return r;
}

static std::string
Xml (const std::string &s)
{
  std::string out;
  for (char c : s)
    {
      if (c == '&') out += "&amp;";
      else if (c == '<') out += "&lt;";
      else if (c == '>') out += "&gt;";
      else out += c;
    }
  return out;
}

typedef double (*Metric) (const Result&);

static void
WriteSvg (const std::string &filename,
          const std::string &title,
          const std::string &yLabel,
          const std::vector<Result> &results,
          Metric metric,
          bool baseDiagonal)
{
  const double width = 920, height = 560, left = 100, right = 35, top = 65, bottom = 85;
  const double xMin = results.front ().baseRttMs;
  const double xMax = results.back ().baseRttMs;
  double yMax = baseDiagonal ? xMax : 0.0;
  for (const Result& r : results) yMax = std::max (yMax, metric (r));
  yMax = std::max (1.0, yMax * 1.15);
  auto x = [&] (double value) { return left + (value - xMin) * (width - left - right) / (xMax - xMin); };
  auto y = [&] (double value) { return height - bottom - value * (height - top - bottom) / yMax; };

  std::ofstream out (filename.c_str ());
  out << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"920\" height=\"560\" viewBox=\"0 0 920 560\">\n"
      << "<rect width=\"100%\" height=\"100%\" fill=\"white\"/>\n"
      << "<style>text{font-family:Arial,sans-serif;fill:#18212b}.axis{stroke:#28323c;stroke-width:1}.grid{stroke:#dce3e8;stroke-width:1}.l4s{stroke:#087e8b;fill:none;stroke-width:3}.classic{stroke:#dc5f00;fill:none;stroke-width:3}.base{stroke:#777;stroke-dasharray:7 5;fill:none;stroke-width:2}</style>\n"
      << "<text x=\"" << width / 2 << "\" y=\"32\" text-anchor=\"middle\" font-size=\"20\" font-weight=\"bold\">" << Xml (title) << "</text>\n";
  for (int i = 0; i <= 5; ++i)
    {
      double value = yMax * i / 5;
      out << "<line class=\"grid\" x1=\"" << left << "\" x2=\"" << width-right << "\" y1=\"" << y(value) << "\" y2=\"" << y(value) << "\"/>\n"
          << "<text x=\"" << left-10 << "\" y=\"" << y(value)+5 << "\" text-anchor=\"end\" font-size=\"12\">" << std::fixed << std::setprecision (1) << value << "</text>\n";
    }
  for (const Result& r : results)
    out << "<line class=\"grid\" x1=\"" << x(r.baseRttMs) << "\" x2=\"" << x(r.baseRttMs) << "\" y1=\"" << top << "\" y2=\"" << height-bottom << "\"/>\n"
        << "<text x=\"" << x(r.baseRttMs) << "\" y=\"" << height-bottom+20 << "\" text-anchor=\"middle\" font-size=\"11\">" << r.baseRttMs << "</text>\n";
  out << "<line class=\"axis\" x1=\"" << left << "\" x2=\"" << width-right << "\" y1=\"" << height-bottom << "\" y2=\"" << height-bottom << "\"/>\n"
      << "<line class=\"axis\" x1=\"" << left << "\" x2=\"" << left << "\" y1=\"" << top << "\" y2=\"" << height-bottom << "\"/>\n";
  for (const char *kind : {"L4S", "Classic"})
    {
      out << "<polyline class=\"" << (std::string(kind) == "L4S" ? "l4s" : "classic") << "\" points=\"";
      for (const Result& r : results) if (r.kind == kind) out << x(r.baseRttMs) << "," << y(metric(r)) << " ";
      out << "\"/>\n";
      for (const Result& r : results) if (r.kind == kind)
        out << "<circle cx=\"" << x(r.baseRttMs) << "\" cy=\"" << y(metric(r)) << "\" r=\"3.5\" fill=\"" << (r.kind == "L4S" ? "#087e8b" : "#dc5f00") << "\"/>\n";
    }
  if (baseDiagonal)
    out << "<polyline class=\"base\" points=\"" << x(xMin) << "," << y(xMin) << " " << x(xMax) << "," << y(xMax) << "\"/>\n";
  out << "<line x1=\"" << width-240 << "\" x2=\"" << width-210 << "\" y1=\"50\" y2=\"50\" class=\"l4s\"/><text x=\"" << width-202 << "\" y=\"55\" font-size=\"13\">L4S</text>\n"
      << "<line x1=\"" << width-155 << "\" x2=\"" << width-125 << "\" y1=\"50\" y2=\"50\" class=\"classic\"/><text x=\"" << width-117 << "\" y=\"55\" font-size=\"13\">Classic</text>\n"
      << "<text x=\"" << width/2 << "\" y=\"" << height-25 << "\" text-anchor=\"middle\" font-size=\"15\">Base RTT (ms)</text>\n"
      << "<text x=\"24\" y=\"" << height/2 << "\" text-anchor=\"middle\" font-size=\"15\" transform=\"rotate(-90 24 " << height/2 << ")\">" << Xml(yLabel) << "</text>\n</svg>\n";
}

static double QueueDelay (const Result& r) { return r.queueDelayMs; }
static double Throughput (const Result& r) { return r.throughputMbps; }
static double AchievedRtt (const Result& r) { return r.achievedRttMs; }
// Signal rate makes the requested ECN/loss plot readable on one scale:
// L4S points are mark rate; Classic points are loss rate.
static double PrimarySignalRate (const Result& r) { return r.kind == "L4S" ? r.markRatePercent : r.lossRatePercent; }

int
main (int argc, char *argv[])
{
  double minRttMs = 5.0, maxRttMs = 60.0, rttStepMs = 5.0;
  double duration = 25.0, warmup = 5.0;
  uint32_t packetSize = 1000;
  std::string bandwidth = "10Mbps", offeredLoad = "50Mbps", outputPrefix = "dualq-rtt-sweep";
  CommandLine cmd;
  cmd.AddValue ("minRttMs", "First base RTT in ms", minRttMs);
  cmd.AddValue ("maxRttMs", "Last base RTT in ms", maxRttMs);
  cmd.AddValue ("rttStepMs", "Base RTT increment in ms", rttStepMs);
  cmd.AddValue ("bandwidth", "Fixed bottleneck bandwidth", bandwidth);
  cmd.AddValue ("offeredLoad", "Fixed application demand (must exceed bandwidth)", offeredLoad);
  cmd.AddValue ("duration", "Traffic duration per run, in seconds", duration);
  cmd.AddValue ("warmup", "Excluded initial seconds per run", warmup);
  cmd.AddValue ("packetSize", "TCP payload size in bytes", packetSize);
  cmd.AddValue ("outputPrefix", "Path prefix for CSV and SVG output", outputPrefix);
  cmd.Parse (argc, argv);
  if (minRttMs <= 0 || maxRttMs < minRttMs || rttStepMs <= 0 || duration <= warmup)
    NS_FATAL_ERROR ("Require positive RTTs/step and duration > warmup");

  std::cout << "DualQ RTT sweep: bottleneck=" << bandwidth << ", offered demand=" << offeredLoad
            << ", measurement window=" << duration-warmup << " s\n\n"
            << std::left << std::setw(9) << "RTT(ms)" << std::setw(10) << "Traffic"
            << std::setw(16) << "Goodput(Mbps)" << std::setw(16) << "RTT(ms)"
            << std::setw(18) << "Queue delay(ms)" << std::setw(13) << "ECN mark(%)"
            << std::setw(11) << "Loss(%)" << "Signals (mark/drop)\n";

  std::vector<Result> results;
  for (double rtt = minRttMs; rtt <= maxRttMs + 1e-9; rtt += rttStepMs)
    for (bool l4s : {true, false})
      {
        Result r = RunOne (rtt, l4s, bandwidth, offeredLoad, duration, warmup, packetSize);
        results.push_back (r);
        std::cout << std::fixed << std::setprecision(2) << std::left << std::setw(9) << r.baseRttMs
                  << std::setw(10) << r.kind << std::setw(16) << r.throughputMbps
                  << std::setw(16) << r.achievedRttMs << std::setw(18) << r.queueDelayMs
                  << std::setw(13) << r.markRatePercent << std::setw(11) << r.lossRatePercent
                  << r.marks << "/" << r.drops << "\n";
      }

  std::ofstream csv ((outputPrefix + ".csv").c_str ());
  csv << "base_rtt_ms,traffic,throughput_mbps,achieved_rtt_ms,queue_delay_ms,ecn_mark_rate_percent,loss_rate_percent,enqueued_packets,marks,drops\n";
  csv << std::fixed << std::setprecision (6);
  for (const Result& r : results)
    csv << r.baseRttMs << ',' << r.kind << ',' << r.throughputMbps << ',' << r.achievedRttMs << ','
        << r.queueDelayMs << ',' << r.markRatePercent << ',' << r.lossRatePercent << ','
        << r.offeredPackets << ',' << r.marks << ',' << r.drops << '\n';
  WriteSvg (outputPrefix + "-queue-delay.svg", "Queuing delay vs base RTT", "Measured queuing delay (ms)", results, &QueueDelay, false);
  WriteSvg (outputPrefix + "-throughput.svg", "Throughput vs base RTT", "Goodput (Mbps)", results, &Throughput, false);
  WriteSvg (outputPrefix + "-achieved-rtt.svg", "Achieved RTT vs base RTT", "Achieved RTT (ms)", results, &AchievedRtt, true);
  WriteSvg (outputPrefix + "-signaling.svg", "ECN marking / loss rate vs RTT", "L4S ECN marks / Classic losses (%)", results, &PrimarySignalRate, false);
  std::cout << "\nWrote " << outputPrefix << ".csv and:\n  " << outputPrefix << "-queue-delay.svg\n  "
            << outputPrefix << "-throughput.svg\n  " << outputPrefix << "-achieved-rtt.svg\n  "
            << outputPrefix << "-signaling.svg\n";
  return 0;
}
