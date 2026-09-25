/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Stage 3: Dynamic utility-aware (gaze-driven) application-centric
 * congestion control over a Wi-Fi bottleneck.
 *
 * Same bottleneck as Stages 1-2 (downlink through an 802.11ax AP whose
 * Wi-Fi egress carries the DualQ Coupled PI2 queue disc; per-flow wired
 * access links are not bottlenecks).  The twist: flow priority is no
 * longer static.
 *
 * Model of dynamic importance: two virtual objects, A and B, are streamed by
 * flows 0 and 1.  A "gaze" process switches attention between the objects
 * every --gazePeriod seconds.  The flow of the currently gazed object gets
 * Type-2 weight A = --aTop (default 4.0); the other gets A = --aBottom
 * (default 0.5).  At every switch the application updates BOTH flows'
 * mapping parameters live by writing the AppCentricA attribute of each
 * flow's congestion-control object; TcpPragueAppCentric re-reads its
 * attributes at every ECN-driven cwnd reduction, so changes take effect
 * immediately while the DCTCP alpha history is preserved.
 *
 * Implementation: each AR/VR flow has its own sender node whose TCP sockets
 * are TcpPragueAppCentric by default.  Shortly after a flow's socket is
 * created (before any marks have flowed, so no alpha state is lost), the
 * scenario installs its own configured TcpPragueAppCentric object via
 * TcpSocketBase::SetCongestionControlAlgorithm and keeps the pointer for the
 * gaze process.  Flows 2 (audio) and 3 (haptics) keep static factory-
 * configured mappings, exactly as in Stage 2.
 *
 * Initial assignment: flow 0 (object A) starts as the gazed flow.  The first
 * gaze switch moves attention to object B at t = 1 + gazePeriod.
 *
 * Reported metrics: per-flow throughput/mean delay/loss, per-interval
 * throughput of the gaze pair, adaptation time after each gaze switch
 * (first sampled interval in which the newly gazed flow recovers its
 * expected share of the pair's throughput), and DualQ statistics.
 *
 * This is an L4S-inspired ECN/AQM Wi-Fi evaluation: it does not implement
 * full Accurate ECN / L4S behaviour at the Wi-Fi bottleneck.
 *
 * Run from the repository root:
 *   ./waf --run "wifi-appcentric-dynamic --flows=2 --simTime=40"
 *   ./waf --run "wifi-appcentric-dynamic --flows=4 --gazePeriod=8 --aTop=6"
 */
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/tcp-prague-app-centric.h"
#include "ns3/traffic-control-module.h"
#include "ns3/wifi-module.h"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("WifiAppCentricDynamic");

static const double SCENARIO_PI = 3.141592653589793;

// Scheduled just before a static flow's application starts: the socket (and
// its TcpPragueAppCentric object) is created inside StartApplication and
// reads these global defaults.
static void
ConfigureFlowDefaults (int type, double p0, double gamma, double a)
{
  const TcpPragueAppCentric::AppCentricType mapping =
    (type == 1) ? TcpPragueAppCentric::TYPE1 : TcpPragueAppCentric::TYPE2;
  Config::SetDefault ("ns3::TcpPragueAppCentric::AppCentricType",
                      EnumValue (mapping));
  Config::SetDefault ("ns3::TcpPragueAppCentric::AppCentricP0",
                      DoubleValue (p0));
  Config::SetDefault ("ns3::TcpPragueAppCentric::AppCentricGamma",
                      DoubleValue (gamma));
  Config::SetDefault ("ns3::TcpPragueAppCentric::AppCentricA",
                      DoubleValue (a));
}

// ---------------- Gaze process state ----------------
struct GazeState
{
  Ptr<TcpPragueAppCentric> ops0; //!< live ops of flow 0 (object A)
  Ptr<TcpPragueAppCentric> ops1; //!< live ops of flow 1 (object B)
  double aTop = 4.0;             //!< weight of the gazed object
  double aBottom = 0.5;          //!< weight of the non-gazed object
  double period = 10.0;          //!< gaze dwell time (s)
  double simTime = 40.0;
  bool gazeOnA = true;           //!< current gaze target
  std::vector<double> switchTimes;
  std::vector<bool> switchedToA; //!< gaze target after each switch
};

// Scheduled shortly after flows 0/1 create their sockets: install a
// configured TcpPragueAppCentric object and register it with the gaze
// process.  This happens before any marks have flowed, so no DCTCP alpha
// history is discarded.
static void
SetupGazeFlow (GazeState *gaze, uint32_t which, Ptr<BulkSendApplication> app,
               double a)
{
  Ptr<Socket> sock = app->GetSocket ();
  if (!sock)
    {
      NS_LOG_WARN ("No socket yet; gaze flow ops not installed");
      return;
    }
  Ptr<TcpPragueAppCentric> ops =
    CreateObjectWithAttributes<TcpPragueAppCentric> (
      "AppCentricType", EnumValue (TcpPragueAppCentric::TYPE2),
      "AppCentricP0", DoubleValue (0.5),
      "AppCentricGamma", DoubleValue (0.0),
      "AppCentricA", DoubleValue (a));
  DynamicCast<TcpSocketBase> (sock)->SetCongestionControlAlgorithm (ops);
  if (which == 0)
    {
      gaze->ops0 = ops;
    }
  else
    {
      gaze->ops1 = ops;
    }
}

static void
GazeToggle (GazeState *st)
{
  st->gazeOnA = !st->gazeOnA;
  const double t = Simulator::Now ().GetSeconds ();
  st->switchTimes.push_back (t);
  st->switchedToA.push_back (st->gazeOnA);

  // The application updates each flow's mapping parameters live.  The
  // objects re-read AppCentricA at every ECN-driven cwnd reduction.
  if (st->ops0)
    {
      st->ops0->SetAttribute ("AppCentricA",
                              DoubleValue (st->gazeOnA ? st->aTop : st->aBottom));
    }
  if (st->ops1)
    {
      st->ops1->SetAttribute ("AppCentricA",
                              DoubleValue (st->gazeOnA ? st->aBottom : st->aTop));
    }
  NS_LOG_INFO ("t=" << t << " gaze on " << (st->gazeOnA ? "A" : "B"));

  if (t + st->period < st->simTime - 0.5)
    {
      Simulator::Schedule (Seconds (st->period), &GazeToggle, st);
    }
}

// ---------------- Periodic throughput sampling ----------------
struct SampleState
{
  std::vector<Ptr<PacketSink>> sinks; //!< flows 0 and 1 (the gaze pair)
  std::vector<uint64_t> lastBytes;
  std::vector<double> series0; //!< flow 0 Mb/s per interval
  std::vector<double> series1; //!< flow 1 Mb/s per interval
  std::vector<double> times;   //!< end time of each interval
  double interval = 0.5;
  double stopTime = 40.0;
};

static void
SampleThroughput (SampleState *sm)
{
  const double now = Simulator::Now ().GetSeconds ();
  std::vector<double> rates (2, 0.0);
  for (uint32_t i = 0; i < 2; ++i)
    {
      const uint64_t rx = sm->sinks[i]->GetTotalRx ();
      rates[i] = (rx - sm->lastBytes[i]) * 8.0 / sm->interval / 1e6;
      sm->lastBytes[i] = rx;
    }
  sm->times.push_back (now);
  sm->series0.push_back (rates[0]);
  sm->series1.push_back (rates[1]);

  if (now + sm->interval <= sm->stopTime)
    {
      Simulator::Schedule (Seconds (sm->interval), &SampleThroughput, sm);
    }
}

int
main (int argc, char *argv[])
{
  // ---------------- Command line ----------------
  uint32_t flows = 2;             // 2 = gaze pair only; 3-4 add audio/haptics
  double simTime = 40.0;
  double warmup = 5.0;
  double gazePeriod = 10.0;       // dwell time per object (s)
  double aTop = 4.0;              // Type-2 A for the gazed flow
  double aBottom = 0.5;           // Type-2 A for the other flow
  double sampleInterval = 0.5;
  std::string wifiStandard = "802.11ax-5GHz";
  uint32_t wifiMcs = 0;
  double apStaDistance = 5.0;
  bool classicBackground = false;
  bool wifiAggregation = false;   // A-MPDU/MSDU aggregation on Wi-Fi
  std::string accessRate = "100Mbps"; // per-flow wired access links (not bottleneck)
  std::string accessDelay = "1ms";

  CommandLine cmd;
  cmd.AddValue ("flows", "Number of AR/VR flows (2-4; flows 0/1 form the gaze pair)", flows);
  cmd.AddValue ("simTime", "Total simulation time in seconds", simTime);
  cmd.AddValue ("warmup", "Startup transient excluded from measurements (s)", warmup);
  cmd.AddValue ("gazePeriod", "Gaze dwell time per object (s)", gazePeriod);
  cmd.AddValue ("aTop", "Type-2 weight A of the currently gazed flow", aTop);
  cmd.AddValue ("aBottom", "Type-2 weight A of the non-gazed flow", aBottom);
  cmd.AddValue ("sampleInterval", "Throughput sampling interval (s)", sampleInterval);
  cmd.AddValue ("wifiStandard", "802.11ax-5GHz, 802.11ac or 802.11n-5GHz", wifiStandard);
  cmd.AddValue ("wifiMcs", "Fixed HE/VHT/HT MCS index for all data", wifiMcs);
  cmd.AddValue ("apStaDistance", "Distance of each station from the AP (m)", apStaDistance);
  cmd.AddValue ("classicBackground", "Add a competing classic TCP (NewReno) flow", classicBackground);
  cmd.AddValue ("wifiAggregation",
                "Enable A-MPDU/MSDU aggregation on the Wi-Fi bottleneck",
                wifiAggregation);
  cmd.AddValue ("accessRate", "Per-flow wired access link data rate", accessRate);
  cmd.AddValue ("accessDelay", "Per-flow wired access link delay", accessDelay);
  cmd.Parse (argc, argv);

  if (flows < 2 || flows > 4 || simTime <= warmup || gazePeriod <= 0.0
      || aTop <= 0.0 || aBottom <= 0.0 || sampleInterval <= 0.0)
    {
      NS_FATAL_ERROR ("Need 2-4 flows, simTime > warmup, and positive periods/weights");
    }

  // ---------------- Scenario-wide defaults (same as Stages 1-2) ----------------
  Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (1000));
  Config::SetDefault ("ns3::TcpSocket::DelAckCount", UintegerValue (1));
  Config::SetDefault ("ns3::TcpSocketBase::EcnMode", StringValue ("ClassicEcn"));
  Config::SetDefault ("ns3::DualQCoupledPi2QueueDisc::Mtu", UintegerValue (1000));
  Config::SetDefault ("ns3::DualQCoupledPi2QueueDisc::QueueLimit", UintegerValue (100000));
  Config::SetDefault ("ns3::WifiMacQueue::MaxQueueSize", StringValue ("40p"));
  if (!wifiAggregation)
    {
      // Aggregation bursts enqueue several data frames at once, which pushes
      // L4S-queue sojourn over the marking threshold even when the bottleneck
      // is not really saturated.  Disabling aggregation gives the AQM
      // smoother, packet-paced queue dynamics.
      Config::SetDefault ("ns3::RegularWifiMac::VO_MaxAmpduSize", UintegerValue (0));
      Config::SetDefault ("ns3::RegularWifiMac::VI_MaxAmpduSize", UintegerValue (0));
      Config::SetDefault ("ns3::RegularWifiMac::BE_MaxAmpduSize", UintegerValue (0));
      Config::SetDefault ("ns3::RegularWifiMac::BK_MaxAmpduSize", UintegerValue (0));
      Config::SetDefault ("ns3::RegularWifiMac::VO_MaxAmsduSize", UintegerValue (0));
      Config::SetDefault ("ns3::RegularWifiMac::VI_MaxAmsduSize", UintegerValue (0));
      Config::SetDefault ("ns3::RegularWifiMac::BE_MaxAmsduSize", UintegerValue (0));
      Config::SetDefault ("ns3::RegularWifiMac::BK_MaxAmsduSize", UintegerValue (0));
    }

  // ---------------- Topology ----------------
  // One sender (AR/VR content server) per flow, each on its own fast wired
  // access link to the AP; one extra dedicated sender for the optional
  // classic background flow.  The AP's Wi-Fi egress is the only bottleneck.
  NodeContainer apNode, staNodes, serverNodes;
  apNode.Create (1);
  staNodes.Create (flows);
  serverNodes.Create (flows + (classicBackground ? 1 : 0));

  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positions = CreateObject<ListPositionAllocator> ();
  positions->Add (Vector (0.0, 0.0, 0.0)); // AP
  for (uint32_t i = 0; i < flows; ++i)
    {
      const double angle = 2.0 * SCENARIO_PI * i / flows;
      positions->Add (Vector (apStaDistance * std::cos (angle),
                              apStaDistance * std::sin (angle), 0.0));
    }
  for (uint32_t i = 0; i < serverNodes.GetN (); ++i)
    {
      positions->Add (Vector (0.0, 0.0, 0.0)); // wired servers (position unused)
    }
  mobility.SetPositionAllocator (positions);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  NodeContainer all (apNode, staNodes, serverNodes);
  mobility.Install (all);

  WifiPhyStandard standard = WIFI_PHY_STANDARD_80211ax_5GHZ;
  std::string mcsBase = "HeMcs";
  if (wifiStandard == "802.11ac")
    {
      standard = WIFI_PHY_STANDARD_80211ac;
      mcsBase = "VhtMcs";
    }
  else if (wifiStandard == "802.11n-5GHz")
    {
      standard = WIFI_PHY_STANDARD_80211n_5GHZ;
      mcsBase = "HtMcs";
    }
  else if (wifiStandard != "802.11ax-5GHz")
    {
      NS_FATAL_ERROR ("Unsupported wifiStandard '" << wifiStandard << "'");
    }
  const std::string dataMode = mcsBase + std::to_string (wifiMcs);

  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
  YansWifiPhyHelper phy = YansWifiPhyHelper::Default ();
  phy.SetChannel (channel.Create ());

  WifiHelper wifi;
  wifi.SetStandard (standard);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode", StringValue (dataMode),
                                "ControlMode", StringValue (mcsBase + "0"));

  WifiMacHelper mac;
  Ssid ssid = Ssid ("wifi-appcentric-dynamic");
  mac.SetType ("ns3::StaWifiMac",
               "Ssid", SsidValue (ssid),
               "ActiveProbing", BooleanValue (false));
  NetDeviceContainer staDevs = wifi.Install (phy, mac, staNodes);
  mac.SetType ("ns3::ApWifiMac", "Ssid", SsidValue (ssid));
  NetDeviceContainer apDev = wifi.Install (phy, mac, apNode);

  PointToPointHelper access;
  access.SetDeviceAttribute ("DataRate", StringValue (accessRate));
  access.SetChannelAttribute ("Delay", StringValue (accessDelay));
  std::vector<NetDeviceContainer> serverLinks (serverNodes.GetN ());
  for (uint32_t i = 0; i < serverNodes.GetN (); ++i)
    {
      serverLinks[i] = access.Install (serverNodes.Get (i), apNode.Get (0));
    }

  InternetStackHelper internet;
  internet.Install (all);
  Config::Set ("/NodeList/" + std::to_string (apNode.Get (0)->GetId ())
                 + "/$ns3::Ipv4L3Protocol/IpForward", BooleanValue (true));

  // ---------------- Bottleneck: AP Wi-Fi egress DualQ ----------------
  TrafficControlHelper tch;
  uint16_t handle = tch.SetRootQueueDisc ("ns3::DualQCoupledPi2QueueDisc");
  tch.AddInternalQueues (handle, 2, "ns3::DropTailQueue",
                         "MaxSize", StringValue ("100000B"));
  QueueDiscContainer qdiscs = tch.Install (apDev.Get (0));
  NS_UNUSED (handle);

  Ipv4AddressHelper ipv4;
  std::vector<Ipv4InterfaceContainer> serverIfs (serverNodes.GetN ());
  for (uint32_t i = 0; i < serverNodes.GetN (); ++i)
    {
      std::ostringstream subnet;
      subnet << "10.1." << (i + 1) << ".0";
      ipv4.SetBase (subnet.str ().c_str (), "255.255.255.0");
      serverIfs[i] = ipv4.Assign (serverLinks[i]);
    }
  ipv4.SetBase ("10.2.1.0", "255.255.255.0");
  NetDeviceContainer wifiDevs;
  wifiDevs.Add (apDev.Get (0));
  wifiDevs.Add (staDevs);
  Ipv4InterfaceContainer wifiIfs = ipv4.Assign (wifiDevs);

  Ipv4StaticRoutingHelper routingHelper;
  for (uint32_t i = 0; i < serverNodes.GetN (); ++i)
    {
      Ptr<Ipv4StaticRouting> serverRouting =
        routingHelper.GetStaticRouting (serverNodes.Get (i)->GetObject<Ipv4> ());
      serverRouting->SetDefaultRoute (serverIfs[i].GetAddress (1), 1);
    }
  for (uint32_t i = 0; i < flows; ++i)
    {
      Ptr<Ipv4StaticRouting> staRouting =
        routingHelper.GetStaticRouting (staNodes.Get (i)->GetObject<Ipv4> ());
      staRouting->SetDefaultRoute (wifiIfs.GetAddress (0), 1);
    }

  // ---------------- Applications ----------------
  static const char *flowNames[4] = {"object-A-video", "object-B-video", "audio", "haptics"};
  const uint32_t nFlows = flows + (classicBackground ? 1 : 0);

  std::vector<Ptr<PacketSink>> sinks;
  std::vector<std::string> names;
  std::vector<uint16_t> ports;
  std::vector<Ipv4Address> dsts;
  sinks.reserve (nFlows);
  names.reserve (nFlows);
  ports.reserve (nFlows);
  dsts.reserve (nFlows);

  std::vector<Ptr<BulkSendApplication>> bulkApps;
  bulkApps.reserve (flows);

  for (uint32_t i = 0; i < flows; ++i)
    {
      // This flow's sender uses the application-centric Prague variant.
      Config::Set ("/NodeList/" + std::to_string (serverNodes.Get (i)->GetId ())
                     + "/$ns3::TcpL4Protocol/SocketType",
                   TypeIdValue (TcpPragueAppCentric::GetTypeId ()));

      const uint16_t port = static_cast<uint16_t> (50000 + i);
      PacketSinkHelper sinkHelper ("ns3::TcpSocketFactory",
                                   InetSocketAddress (Ipv4Address::GetAny (), port));
      ApplicationContainer sinkApp = sinkHelper.Install (staNodes.Get (i));
      sinkApp.Start (Seconds (0.0));
      sinkApp.Stop (Seconds (simTime));
      sinks.push_back (DynamicCast<PacketSink> (sinkApp.Get (0)));
      names.push_back (flowNames[i]);
      ports.push_back (port);
      dsts.push_back (wifiIfs.GetAddress (1 + i));

      const double start = 0.5 + 0.05 * i;
      if (i >= 2)
        {
          // Static flows (audio, haptics): configure factory defaults just
          // before the socket is created, as in Stage 2.
          const int type = (i == 2) ? 2 : 1;
          const double p0 = (i == 2) ? 0.5 : 0.6;
          const double gamma = (i == 2) ? 0.0 : -0.5;
          Simulator::Schedule (Seconds (start - 0.04), &ConfigureFlowDefaults,
                               type, p0, gamma, 1.0);
        }

      BulkSendHelper bulk ("ns3::TcpSocketFactory",
                           InetSocketAddress (wifiIfs.GetAddress (1 + i), port));
      bulk.SetAttribute ("SendSize", UintegerValue (1000));
      bulk.SetAttribute ("MaxBytes", UintegerValue (0));
      ApplicationContainer sourceApp = bulk.Install (serverNodes.Get (i));
      sourceApp.Start (Seconds (start));
      sourceApp.Stop (Seconds (simTime - 0.5));
      bulkApps.push_back (DynamicCast<BulkSendApplication> (sourceApp.Get (0)));
    }

  // The gaze process must outlive the schedules; create it once here and
  // register the flows through dedicated schedules.
  GazeState *gaze = new GazeState ();
  for (uint32_t i = 0; i < 2 && i < flows; ++i)
    {
      const double t = 0.6 + 0.05 * i; // shortly after flow i's app start
      Simulator::Schedule (Seconds (t), &SetupGazeFlow, gaze, i, bulkApps[i],
                           (i == 0) ? aTop : aBottom);
    }
  for (uint32_t i = 2; i < flows; ++i)
    {
      // Static flows keep their factory-configured ops object.
    }

  if (classicBackground)
    {
      const uint16_t port = 51000;
      const uint32_t bgNode = serverNodes.GetN () - 1;
      Config::Set ("/NodeList/" + std::to_string (serverNodes.Get (bgNode)->GetId ())
                     + "/$ns3::TcpL4Protocol/SocketType",
                   TypeIdValue (TcpNewReno::GetTypeId ()));
      PacketSinkHelper sinkHelper ("ns3::TcpSocketFactory",
                                   InetSocketAddress (Ipv4Address::GetAny (), port));
      ApplicationContainer sinkApp = sinkHelper.Install (staNodes.Get (0));
      sinkApp.Start (Seconds (0.0));
      sinkApp.Stop (Seconds (simTime));
      sinks.push_back (DynamicCast<PacketSink> (sinkApp.Get (0)));
      names.push_back ("classic-background");
      ports.push_back (port);
      dsts.push_back (wifiIfs.GetAddress (1));

      BulkSendHelper bulk ("ns3::TcpSocketFactory",
                           InetSocketAddress (wifiIfs.GetAddress (1), port));
      bulk.SetAttribute ("SendSize", UintegerValue (1000));
      bulk.SetAttribute ("MaxBytes", UintegerValue (0));
      ApplicationContainer sourceApp = bulk.Install (serverNodes.Get (bgNode));
      sourceApp.Start (Seconds (0.5));
      sourceApp.Stop (Seconds (simTime - 0.5));
    }

  // Start the gaze process and the throughput sampler.
  gaze->period = gazePeriod;
  gaze->aTop = aTop;
  gaze->aBottom = aBottom;
  gaze->simTime = simTime;
  SampleState *sampler = new SampleState ();
  for (uint32_t i = 0; i < 2; ++i)
    {
      sampler->sinks.push_back (sinks[i]);
    }
  sampler->lastBytes.assign (2, 0);
  sampler->interval = sampleInterval;
  sampler->stopTime = simTime;

  Simulator::Schedule (Seconds (1.0 + gazePeriod), &GazeToggle, gaze);
  Simulator::Schedule (Seconds (sampleInterval), &SampleThroughput, sampler);

  FlowMonitorHelper flowmonHelper;
  Ptr<FlowMonitor> monitor = flowmonHelper.InstallAll ();

  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();

  monitor->CheckForLostPackets ();
  Ptr<Ipv4FlowClassifier> classifier =
    DynamicCast<Ipv4FlowClassifier> (flowmonHelper.GetClassifier ());
  const FlowMonitor::FlowStatsContainer &stats = monitor->GetFlowStats ();

  // ---------------- Reporting ----------------
  NS_UNUSED (warmup); // whole-run averages reported, as in Stages 1-2
  std::cout << std::fixed << std::setprecision (3);
  std::cout << "=== Stage 3: dynamic gaze-driven priorities over Wi-Fi ===\n";
  std::cout << "flows=" << flows << " gazePeriod=" << gazePeriod
            << "s aTop=" << aTop << " aBottom=" << aBottom
            << " standard=" << wifiStandard << " mcs=" << wifiMcs
            << " background=" << (classicBackground ? "classic" : "off")
            << " aggregation=" << (wifiAggregation ? "on" : "off") << "\n";

  std::vector<double> mbps (nFlows, 0.0);
  std::vector<double> delayMs (nFlows, 0.0);
  std::vector<uint32_t> lost (nFlows, 0);
  for (const auto &entry : stats)
    {
      const Ipv4FlowClassifier::FiveTuple tuple = classifier->FindFlow (entry.first);
      for (uint32_t f = 0; f < nFlows; ++f)
        {
          if (tuple.destinationPort == ports[f] && tuple.destinationAddress == dsts[f])
            {
              mbps[f] = sinks[f]->GetTotalRx () * 8.0 / simTime / 1e6;
              delayMs[f] = entry.second.rxPackets > 0
                             ? entry.second.delaySum.GetSeconds () * 1000.0
                                 / entry.second.rxPackets
                             : 0.0;
              lost[f] = entry.second.lostPackets;
            }
        }
    }

  double sumL4s = 0.0;
  for (uint32_t f = 0; f < flows; ++f)
    {
      sumL4s += mbps[f];
    }
  for (uint32_t f = 0; f < nFlows; ++f)
    {
      std::cout << "  " << names[f] << ": " << mbps[f] << " Mb/s"
                << " meanDelay=" << delayMs[f] << " ms"
                << " lost=" << lost[f];
      if (f < flows && sumL4s > 0)
        {
          std::cout << " share=" << 100.0 * mbps[f] / sumL4s << "%";
        }
      std::cout << "\n";
    }

  // Per-interval throughput of the gaze pair.
  std::cout << "  gaze-pair throughput per " << sampleInterval
            << "s interval (flow0 | flow1 Mb/s):\n";
  for (std::size_t k = 0; k < sampler->times.size (); ++k)
    {
      std::cout << "    t=" << sampler->times[k] << ": "
                << sampler->series0[k] << " | " << sampler->series1[k] << "\n";
    }

  // Adaptation time after each gaze switch: first sampled interval ending
  // after the switch in which the newly gazed flow carries at least 70% of
  // the way from an equal share to its expected priority share.
  const double expectedShare = aTop / (aTop + aBottom);
  const double threshold = 0.5 + 0.7 * (expectedShare - 0.5);
  double adaptationSum = 0.0;
  uint32_t adaptationCount = 0;
  for (std::size_t s = 0; s < gaze->switchTimes.size (); ++s)
    {
      const double switchTime = gaze->switchTimes[s];
      const bool toA = gaze->switchedToA[s];
      for (std::size_t k = 0; k < sampler->times.size (); ++k)
        {
          if (sampler->times[k] <= switchTime + sampleInterval)
            {
              continue; // interval containing or before the switch
            }
          const double total = sampler->series0[k] + sampler->series1[k];
          if (total <= 0.0)
            {
              continue;
            }
          const double share01 = toA ? sampler->series0[k] / total
                                     : sampler->series1[k] / total;
          if (share01 >= threshold)
            {
              adaptationSum += sampler->times[k] - switchTime;
              adaptationCount++;
              break;
            }
        }
    }
  std::cout << "  gaze switches: " << gaze->switchTimes.size ()
            << " (expected pair share of gazed flow: "
            << 100.0 * expectedShare << "%, adaptation threshold: "
            << 100.0 * threshold << "%)\n";
  if (adaptationCount > 0)
    {
      std::cout << "  mean adaptation time after switch: "
                << adaptationSum / adaptationCount << " s ("
                << adaptationCount << " of " << gaze->switchTimes.size ()
                << " switches recovered within the window)\n";
    }
  else
    {
      std::cout << "  mean adaptation time: not measurable in this run\n";
    }

  const QueueDisc::Stats qs = qdiscs.Get (0)->GetStats ();
  std::cout << "  AP egress DualQ: L4S-marks="
            << qs.GetNMarkedPackets (DualQCoupledPi2QueueDisc::UNFORCED_L4S_MARK)
            << " classic-marks="
            << qs.GetNMarkedPackets (DualQCoupledPi2QueueDisc::UNFORCED_CLASSIC_MARK)
            << " drops="
            << qs.GetNDroppedPackets (DualQCoupledPi2QueueDisc::UNFORCED_CLASSIC_DROP)
               + qs.GetNDroppedPackets (DualQCoupledPi2QueueDisc::FORCED_DROP)
            << "\n";

  Simulator::Destroy ();
  delete gaze;
  delete sampler;
  return 0;
}
