/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Stage 4: Adaptive application-centric congestion control for AR/VR
 * traffic over a DYNAMIC Wi-Fi network.
 *
 * Builds on wifi-appcentric-dynamic.cc (Stage 3) and adds the full
 * fluctuating-Wi-Fi environment.  The bottleneck remains the AP's Wi-Fi
 * egress, which carries the DualQ Coupled PI2 queue disc (L4S-inspired
 * ECN/AQM); per-flow wired access links are not bottlenecks.
 *
 * Dynamic Wi-Fi elements (each independently switchable):
 *   - Station mobility (--mobility): AR/VR stations move around the AP on
 *     waypoint rings (angular motion approximated by 2-second segments).
 *   - Adaptive PHY rate (--adaptiveRate): MinstrelHt rate control instead of
 *     a fixed MCS, so capacity follows distance/conditions.
 *   - Interference (--interference): a non-AR station generates UDP load
 *     that shares the channel and the DualQ; --udpBackgroundStart/Stop make
 *     that load appear and disappear mid-run.
 *   - Changing background traffic (--classicBackground): a classic (NewReno)
 *     TCP "web" flow comes and goes in three bursts.
 *
 * Schemes (--scheme):
 *   adaptive : the Stage 3 contribution - a gaze process switches the
 *              Type-2 weight A of the two object flows live (aTop/aBottom).
 *   fixed    : paper-style static priorities - flow 0 keeps A=aTop and
 *              flow 1 keeps A=aBottom for the whole run, no gaze.
 *   equal    : reference DCTCP/ECT(1) flows, no application-centric mapping.
 *
 * Utility metric: application-weighted log utility
 *   U = sum_i w_i * log2(1 + r_i(t) / x0), x0 = 1 Mb/s; reported as the mean
 *   over the whole run and over the steady window [max(warmup, T/2), T].
 *   Two weight vectors are reported:
 *     - static  : w = (4, 2, 1) for (object-A, object-B, haptics), i.e. the
 *       fixed quality hierarchy.  A fixed-priority run scores highest on
 *       this by construction.
 *     - gaze-aligned (adaptive scheme only): w = (3, 3, 1) on the object
 *       that currently holds the gaze, so a run is judged by how well it
 *       serves whoever is being looked at.
 *
 * This is an L4S-inspired ECN/AQM Wi-Fi evaluation: it does not implement
 * full Accurate ECN / L4S behaviour at the Wi-Fi bottleneck.
 *
 * Run from the repository root:
 *   ./waf --run "wifi-appcentric-dynamic-wifi --simTime=40"
 *   ./waf --run "wifi-appcentric-dynamic-wifi --scheme=fixed"
 *   ./waf --run "wifi-appcentric-dynamic-wifi --scheme=equal --mobility=false --interference=false"
 *   ./waf --run "wifi-appcentric-dynamic-wifi --simTime=40 --mobility=false --interference=false \
 *              --udpBackground=false --classicBackground=false" ; # static ablation
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

NS_LOG_COMPONENT_DEFINE ("WifiAppCentricDynamicWifi");

static const double SCENARIO_PI = 3.141592653589793;

// Scheduled just before a static app-centric flow starts: the socket (and
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

// ---------------- Gaze process (adaptive scheme only) ----------------
struct GazeState
{
  Ptr<TcpPragueAppCentric> ops0; //!< live ops of flow 0 (object A)
  Ptr<TcpPragueAppCentric> ops1; //!< live ops of flow 1 (object B)
  double aTop = 4.0;
  double aBottom = 0.5;
  double period = 10.0;
  double simTime = 40.0;
  bool gazeOnA = true;
  bool startedOnA = true; //!< flow that received aTop at setup
  std::vector<double> switchTimes;
  std::vector<bool> switchedToA;
};

// Scheduled shortly after flows 0/1 create their sockets: install a
// configured TcpPragueAppCentric object and register it with the gaze
// process.  Happens before any marks flow, so no DCTCP alpha history is lost.
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
  gaze->startedOnA = (which == 0);
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

// ---------------- Periodic sampling ----------------
// Samples per-interval throughput of the two object flows and the haptics
// flow (utilities are computed from these series after the run).
struct SampleState
{
  std::vector<Ptr<PacketSink>> sinks; //!< up to 3 flows
  std::vector<uint64_t> lastBytes;
  std::vector<std::vector<double>> series; //!< series[f][k] Mb/s
  std::vector<double> times;
  double interval = 0.5;
  double stopTime = 40.0;
};

static void
SampleThroughput (SampleState *sm)
{
  const double now = Simulator::Now ().GetSeconds ();
  std::vector<double> rates (sm->sinks.size (), 0.0);
  for (std::size_t i = 0; i < sm->sinks.size (); ++i)
    {
      const uint64_t rx = sm->sinks[i]->GetTotalRx ();
      rates[i] = (rx - sm->lastBytes[i]) * 8.0 / sm->interval / 1e6;
      sm->lastBytes[i] = rx;
    }
  sm->times.push_back (now);
  for (std::size_t i = 0; i < sm->sinks.size (); ++i)
    {
      sm->series[i].push_back (rates[i]);
    }
  if (now + sm->interval <= sm->stopTime)
    {
      Simulator::Schedule (Seconds (sm->interval), &SampleThroughput, sm);
    }
}

// Per-second distance of the AR station 0 from the AP (capacity proxy under
// mobility) and the DualQ mark/drop counts in that second.
struct TraceState
{
  Ptr<MobilityModel> apMob;   //!< AP position (distance reference)
  Ptr<MobilityModel> sta0Mob; //!< AR station 0 mobility
  Ptr<QueueDisc> qd;
  std::vector<double> times;
  std::vector<double> distances;
  std::vector<uint32_t> marksPerSec;
  std::vector<uint32_t> dropsPerSec;
  uint32_t lastMarks = 0;
  uint32_t lastDrops = 0;
  double stopTime = 40.0;
};

static void
SampleTrace (TraceState *ts)
{
  const double now = Simulator::Now ().GetSeconds ();
  double dist = 0.0;
  if (ts->sta0Mob)
    {
      dist = ts->sta0Mob->GetDistanceFrom (ts->apMob);
    }
  const QueueDisc::Stats qs = ts->qd->GetStats ();
  const uint32_t marks =
    qs.GetNMarkedPackets (DualQCoupledPi2QueueDisc::UNFORCED_L4S_MARK)
    + qs.GetNMarkedPackets (DualQCoupledPi2QueueDisc::UNFORCED_CLASSIC_MARK);
  const uint32_t drops =
    qs.GetNDroppedPackets (DualQCoupledPi2QueueDisc::UNFORCED_CLASSIC_DROP)
    + qs.GetNDroppedPackets (DualQCoupledPi2QueueDisc::FORCED_DROP);
  if (!ts->times.empty ())
    {
      ts->marksPerSec.push_back (marks - ts->lastMarks);
      ts->dropsPerSec.push_back (drops - ts->lastDrops);
    }
  ts->lastMarks = marks;
  ts->lastDrops = drops;
  ts->times.push_back (now);
  ts->distances.push_back (dist);
  if (now + 1.0 <= ts->stopTime)
    {
      Simulator::Schedule (Seconds (1.0), &SampleTrace, ts);
    }
}

int
main (int argc, char *argv[])
{
  // ---------------- Command line ----------------
  std::string scheme = "adaptive";   // adaptive | fixed | equal
  uint32_t nObjects = 2;             // 1-2 object flows (flows 0/1)
  double simTime = 40.0;
  double warmup = 5.0;
  double gazePeriod = 10.0;
  double aTop = 4.0;
  double aBottom = 0.5;
  double sampleInterval = 0.5;
  bool mobility = true;
  double mobilitySpeed = 1.0;        // rad/s angular speed on the ring
  bool adaptiveRate = true;          // MinstrelHt instead of fixed MCS
  uint32_t wifiMcs = 0;              // used when adaptiveRate=false
  bool interference = true;
  bool udpBackground = true;
  std::string udpBackgroundRate = "6Mbps";
  double udpBackgroundStart = 12.0;
  double udpBackgroundStop = 20.0;
  bool classicBackground = true;     // bursty NewReno "web" flow
  double apStaDistance = 5.0;
  std::string wifiStandard = "802.11ax-5GHz";
  bool wifiAggregation = false;
  std::string accessRate = "100Mbps";
  std::string accessDelay = "1ms";

  CommandLine cmd;
  cmd.AddValue ("scheme", "adaptive | fixed | equal", scheme);
  cmd.AddValue ("nObjects", "Number of gaze-object flows (1-2)", nObjects);
  cmd.AddValue ("simTime", "Total simulation time in seconds", simTime);
  cmd.AddValue ("warmup", "Startup transient excluded from utility window (s)", warmup);
  cmd.AddValue ("gazePeriod", "Gaze dwell time per object (s)", gazePeriod);
  cmd.AddValue ("aTop", "Type-2 weight A of the gazed flow", aTop);
  cmd.AddValue ("aBottom", "Type-2 weight A of the non-gazed flow", aBottom);
  cmd.AddValue ("sampleInterval", "Throughput sampling interval (s)", sampleInterval);
  cmd.AddValue ("mobility", "AR stations move around the AP", mobility);
  cmd.AddValue ("mobilitySpeed", "Angular speed of stations on the ring (rad/s)", mobilitySpeed);
  cmd.AddValue ("adaptiveRate", "Use MinstrelHt adaptive rate control", adaptiveRate);
  cmd.AddValue ("wifiMcs", "Fixed MCS index (used when adaptiveRate=false)", wifiMcs);
  cmd.AddValue ("interference", "Add a non-AR interferer station", interference);
  cmd.AddValue ("udpBackground", "Enable the UDP background load", udpBackground);
  cmd.AddValue ("udpBackgroundRate", "UDP background offered rate", udpBackgroundRate);
  cmd.AddValue ("udpBackgroundStart", "UDP background start time (s)", udpBackgroundStart);
  cmd.AddValue ("udpBackgroundStop", "UDP background stop time (s)", udpBackgroundStop);
  cmd.AddValue ("classicBackground", "Enable bursty classic TCP web traffic", classicBackground);
  cmd.AddValue ("apStaDistance", "Base ring distance of stations from the AP (m)", apStaDistance);
  cmd.AddValue ("wifiStandard", "802.11ax-5GHz, 802.11ac or 802.11n-5GHz", wifiStandard);
  cmd.AddValue ("wifiAggregation", "Enable A-MPDU/MSDU aggregation", wifiAggregation);
  cmd.AddValue ("accessRate", "Per-flow wired access link data rate", accessRate);
  cmd.AddValue ("accessDelay", "Per-flow wired access link delay", accessDelay);
  cmd.Parse (argc, argv);

  if ((scheme != "adaptive" && scheme != "fixed" && scheme != "equal")
      || nObjects == 0 || nObjects > 2 || simTime <= warmup
      || (scheme == "adaptive" && (gazePeriod <= 0.0 || aTop <= 0.0 || aBottom <= 0.0))
      || sampleInterval <= 0.0 || mobilitySpeed < 0.0
      || udpBackgroundStart >= udpBackgroundStop)
    {
      NS_FATAL_ERROR ("Invalid parameters");
    }

  // ---------------- Scenario-wide defaults ----------------
  Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (1000));
  Config::SetDefault ("ns3::TcpSocket::DelAckCount", UintegerValue (1));
  Config::SetDefault ("ns3::TcpSocketBase::EcnMode", StringValue ("ClassicEcn"));
  Config::SetDefault ("ns3::DualQCoupledPi2QueueDisc::Mtu", UintegerValue (1000));
  Config::SetDefault ("ns3::DualQCoupledPi2QueueDisc::QueueLimit", UintegerValue (100000));
  Config::SetDefault ("ns3::WifiMacQueue::MaxQueueSize", StringValue ("40p"));
  if (!wifiAggregation)
    {
      // Aggregation bursts push L4S-queue sojourn over the marking threshold
      // even below saturation (see README); keep per-packet transmission.
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
  // Stations: nObjects AR stations + haptics station + interferer station.
  // Servers: one per AR/haptics flow + one background server (UDP + web).
  const bool withHaptics = true;
  const uint32_t nSta = nObjects + (withHaptics ? 1 : 0) + (interference ? 1 : 0);
  const uint32_t nServers = nObjects + (withHaptics ? 1 : 0) + 1;

  NodeContainer apNode, staNodes, serverNodes;
  apNode.Create (1);
  staNodes.Create (nSta);
  serverNodes.Create (nServers);

  MobilityHelper mobilityHelper;
  mobilityHelper.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobilityHelper.Install (NodeContainer (apNode, serverNodes));

  Ptr<ListPositionAllocator> staPositions = CreateObject<ListPositionAllocator> ();
  for (uint32_t i = 0; i < nSta; ++i)
    {
      // Initial positions on a ring; movers get waypoints below.
      const double angle = 2.0 * SCENARIO_PI * i / nSta;
      const double r = apStaDistance + (i % 2) * 1.5;
      staPositions->Add (Vector (r * std::cos (angle), r * std::sin (angle), 0.0));
    }
  mobilityHelper.SetPositionAllocator (staPositions);
  if (mobility)
    {
      // Waypoints are added per node after Install (model must exist first).
      mobilityHelper.SetMobilityModel ("ns3::WaypointMobilityModel");
    }
  mobilityHelper.Install (staNodes);

  WifiPhyStandard standard = WIFI_PHY_STANDARD_80211ax_5GHZ;
  std::string mcsBase = "HeMcs";
  if (adaptiveRate && wifiStandard == "802.11ax-5GHz")
    {
      // This tree's MinstrelHt supports HT/VHT but not HE rates; fall back
      // to 802.11ac so that adaptive rate control is available.
      NS_LOG_WARN ("MinstrelHt does not support HE in this tree; using 802.11ac");
      wifiStandard = "802.11ac";
    }
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

  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
  YansWifiPhyHelper phy = YansWifiPhyHelper::Default ();
  phy.SetChannel (channel.Create ());

  WifiHelper wifi;
  wifi.SetStandard (standard);
  if (adaptiveRate)
    {
      // MinstrelHt supports HT/VHT/HE rates and reacts to distance/losses.
      wifi.SetRemoteStationManager ("ns3::MinstrelHtWifiManager");
    }
  else
    {
      const std::string dataMode = mcsBase + std::to_string (wifiMcs);
      wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                    "DataMode", StringValue (dataMode),
                                    "ControlMode", StringValue (mcsBase + "0"));
    }

  WifiMacHelper mac;
  Ssid ssid = Ssid ("wifi-appcentric-dynamic-wifi");
  mac.SetType ("ns3::StaWifiMac",
               "Ssid", SsidValue (ssid),
               "ActiveProbing", BooleanValue (false));
  NetDeviceContainer staDevs = wifi.Install (phy, mac, staNodes);
  mac.SetType ("ns3::ApWifiMac", "Ssid", SsidValue (ssid));
  NetDeviceContainer apDev = wifi.Install (phy, mac, apNode);

  PointToPointHelper access;
  access.SetDeviceAttribute ("DataRate", StringValue (accessRate));
  access.SetChannelAttribute ("Delay", StringValue (accessDelay));
  std::vector<NetDeviceContainer> serverLinks (nServers);
  for (uint32_t i = 0; i < nServers; ++i)
    {
      serverLinks[i] = access.Install (serverNodes.Get (i), apNode.Get (0));
    }

  InternetStackHelper internet;
  internet.Install (NodeContainer (apNode, staNodes, serverNodes));
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
  std::vector<Ipv4InterfaceContainer> serverIfs (nServers);
  for (uint32_t i = 0; i < nServers; ++i)
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
  for (uint32_t i = 0; i < nServers; ++i)
    {
      Ptr<Ipv4StaticRouting> serverRouting =
        routingHelper.GetStaticRouting (serverNodes.Get (i)->GetObject<Ipv4> ());
      serverRouting->SetDefaultRoute (serverIfs[i].GetAddress (1), 1);
    }
  for (uint32_t i = 0; i < nSta; ++i)
    {
      Ptr<Ipv4StaticRouting> staRouting =
        routingHelper.GetStaticRouting (staNodes.Get (i)->GetObject<Ipv4> ());
      staRouting->SetDefaultRoute (wifiIfs.GetAddress (0), 1);
    }

  // ---------------- Mobility waypoints ----------------
  if (mobility)
    {
      // AR stations (and, if present, the interferer) walk their rings at
      // mobilitySpeed rad/s; direction alternates per station.
      for (uint32_t i = 0; i < nSta; ++i)
        {
          Ptr<WaypointMobilityModel> model = staNodes.Get (i)
            ->GetObject<WaypointMobilityModel> ();
          if (!model)
            {
              continue;
            }
          const double r = apStaDistance + (i % 2) * 1.5;
          const double dir = (i % 2 == 0) ? 1.0 : -1.0;
          const double phase = 2.0 * SCENARIO_PI * i / nSta;
          double t = 0.0;
          while (t < simTime - 1.0)
            {
              const double angle = phase + dir * mobilitySpeed * t;
              model->AddWaypoint (Waypoint (Seconds (t),
                                            Vector (r * std::cos (angle),
                                                    r * std::sin (angle), 0.0)));
              t += 2.0; // linear interpolation between segment boundaries
            }
          model->AddWaypoint (Waypoint (Seconds (simTime),
                                        Vector (r * std::cos (phase + dir * mobilitySpeed * (simTime - 2.0)),
                                                r * std::sin (phase + dir * mobilitySpeed * (simTime - 2.0)),
                                                0.0)));
        }
    }

  // ---------------- Applications ----------------
  static const char *objectNames[2] = {"object-A-video", "object-B-video"};
  const uint32_t totalTcpFlows = nObjects + (withHaptics ? 1 : 0)
                                 + (classicBackground ? 1 : 0);

  std::vector<Ptr<PacketSink>> sinks;
  std::vector<std::string> names;
  std::vector<uint16_t> ports;
  std::vector<Ipv4Address> dsts;
  sinks.reserve (totalTcpFlows);
  names.reserve (totalTcpFlows);
  ports.reserve (totalTcpFlows);
  dsts.reserve (totalTcpFlows);

  std::vector<Ptr<BulkSendApplication>> bulkApps;
  bulkApps.reserve (nObjects + (withHaptics ? 1 : 0));

  const bool usePrague = (scheme != "equal");

  // Object flows (0..nObjects-1): the gaze pair.
  for (uint32_t i = 0; i < nObjects; ++i)
    {
      if (usePrague)
        {
          Config::Set ("/NodeList/" + std::to_string (serverNodes.Get (i)->GetId ())
                         + "/$ns3::TcpL4Protocol/SocketType",
                       TypeIdValue (TcpPragueAppCentric::GetTypeId ()));
        }
      else
        {
          // equal scheme: plain DCTCP/ECT(1) reference (no app-centric map).
          Config::Set ("/NodeList/" + std::to_string (serverNodes.Get (i)->GetId ())
                         + "/$ns3::TcpL4Protocol/SocketType",
                       TypeIdValue (TcpDctcp::GetTypeId ()));
        }
      const uint16_t port = static_cast<uint16_t> (50000 + i);
      PacketSinkHelper sinkHelper ("ns3::TcpSocketFactory",
                                   InetSocketAddress (Ipv4Address::GetAny (), port));
      ApplicationContainer sinkApp = sinkHelper.Install (staNodes.Get (i));
      sinkApp.Start (Seconds (0.0));
      sinkApp.Stop (Seconds (simTime));
      sinks.push_back (DynamicCast<PacketSink> (sinkApp.Get (0)));
      names.push_back (objectNames[i]);
      ports.push_back (port);
      dsts.push_back (wifiIfs.GetAddress (1 + i));

      const double start = 0.5 + 0.05 * i;
      if (usePrague && scheme == "fixed")
        {
          // Static priorities: flow 0 keeps aTop, flow 1 keeps aBottom.
          Simulator::Schedule (Seconds (start - 0.04), &ConfigureFlowDefaults,
                               2, 0.5, 0.0, (i == 0) ? aTop : aBottom);
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

  // Haptics flow (station/server index nObjects): static conservative
  // Type-1 mapping in the app-centric schemes, plain DCTCP in the equal
  // scheme.  Its flow index among sinks is nObjects (after the gaze pair).
  if (withHaptics)
    {
      const uint32_t staIdx = nObjects;
      const uint32_t srvIdx = nObjects;
      if (usePrague)
        {
          Config::Set ("/NodeList/" + std::to_string (serverNodes.Get (srvIdx)->GetId ())
                         + "/$ns3::TcpL4Protocol/SocketType",
                       TypeIdValue (TcpPragueAppCentric::GetTypeId ()));
        }
      else
        {
          Config::Set ("/NodeList/" + std::to_string (serverNodes.Get (srvIdx)->GetId ())
                         + "/$ns3::TcpL4Protocol/SocketType",
                       TypeIdValue (TcpDctcp::GetTypeId ()));
        }
      const uint16_t port = 50002;
      PacketSinkHelper sinkHelper ("ns3::TcpSocketFactory",
                                   InetSocketAddress (Ipv4Address::GetAny (), port));
      ApplicationContainer sinkApp = sinkHelper.Install (staNodes.Get (staIdx));
      sinkApp.Start (Seconds (0.0));
      sinkApp.Stop (Seconds (simTime));
      sinks.push_back (DynamicCast<PacketSink> (sinkApp.Get (0)));
      names.push_back ("haptics");
      ports.push_back (port);
      dsts.push_back (wifiIfs.GetAddress (1 + staIdx));

      const double start = 0.5 + 0.05 * (srvIdx);
      if (usePrague)
        {
          Simulator::Schedule (Seconds (start - 0.04), &ConfigureFlowDefaults,
                               1, 0.6, -0.5, 1.0);
        }
      BulkSendHelper bulk ("ns3::TcpSocketFactory",
                           InetSocketAddress (wifiIfs.GetAddress (1 + staIdx), port));
      bulk.SetAttribute ("SendSize", UintegerValue (1000));
      bulk.SetAttribute ("MaxBytes", UintegerValue (0));
      ApplicationContainer sourceApp = bulk.Install (serverNodes.Get (srvIdx));
      sourceApp.Start (Seconds (start));
      sourceApp.Stop (Seconds (simTime - 0.5));
      bulkApps.push_back (DynamicCast<BulkSendApplication> (sourceApp.Get (0)));
    }

  // Background server: UDP interferer load + bursty classic TCP web flow.
  const uint32_t bgServer = nServers - 1;
  const uint32_t bgSta = nSta - 1; // last station hosts the background sink(s)
  if (classicBackground)
    {
      Config::Set ("/NodeList/" + std::to_string (serverNodes.Get (bgServer)->GetId ())
                     + "/$ns3::TcpL4Protocol/SocketType",
                   TypeIdValue (TcpNewReno::GetTypeId ()));
      const uint16_t port = 50011;
      PacketSinkHelper sinkHelper ("ns3::TcpSocketFactory",
                                   InetSocketAddress (Ipv4Address::GetAny (), port));
      ApplicationContainer sinkApp = sinkHelper.Install (staNodes.Get (bgSta));
      sinkApp.Start (Seconds (0.0));
      sinkApp.Stop (Seconds (simTime));
      sinks.push_back (DynamicCast<PacketSink> (sinkApp.Get (0)));
      names.push_back ("web-classic");
      ports.push_back (port);
      dsts.push_back (wifiIfs.GetAddress (1 + bgSta));

      // Three bursts: changing background traffic on/off pattern.
      const double windows[3][2] = {{3.0, 7.0}, {11.0, 15.0}, {19.0, 23.0}};
      for (const auto &w : windows)
        {
          if (w[1] >= simTime)
            {
              continue;
            }
          BulkSendHelper bulk ("ns3::TcpSocketFactory",
                               InetSocketAddress (wifiIfs.GetAddress (1 + bgSta), port));
          bulk.SetAttribute ("SendSize", UintegerValue (1000));
          bulk.SetAttribute ("MaxBytes", UintegerValue (0));
          ApplicationContainer burst = bulk.Install (serverNodes.Get (bgServer));
          burst.Start (Seconds (w[0]));
          burst.Stop (Seconds (w[1]));
        }
    }
  if (udpBackground)
    {
      const uint16_t port = 50010;
      PacketSinkHelper sinkHelper ("ns3::UdpSocketFactory",
                                   InetSocketAddress (Ipv4Address::GetAny (), port));
      ApplicationContainer sinkApp = sinkHelper.Install (staNodes.Get (bgSta));
      sinkApp.Start (Seconds (0.0));
      sinkApp.Stop (Seconds (simTime));
      sinks.push_back (DynamicCast<PacketSink> (sinkApp.Get (0)));
      names.push_back ("udp-background");
      ports.push_back (port);
      dsts.push_back (wifiIfs.GetAddress (1 + bgSta));

      OnOffHelper onoff ("ns3::UdpSocketFactory",
                         InetSocketAddress (wifiIfs.GetAddress (1 + bgSta), port));
      onoff.SetConstantRate (DataRate (udpBackgroundRate), 1000);
      ApplicationContainer udpApp = onoff.Install (serverNodes.Get (bgServer));
      udpApp.Start (Seconds (udpBackgroundStart));
      udpApp.Stop (Seconds (std::min (udpBackgroundStop, simTime - 0.5)));
    }

  // ---------------- Gaze process (adaptive scheme) ----------------
  GazeState *gaze = nullptr;
  if (scheme == "adaptive")
    {
      gaze = new GazeState ();
      gaze->period = gazePeriod;
      gaze->aTop = aTop;
      gaze->aBottom = aBottom;
      gaze->simTime = simTime;
      for (uint32_t i = 0; i < nObjects && i < 2; ++i)
        {
          const double t = 0.6 + 0.05 * i; // shortly after flow i's app start
          Simulator::Schedule (Seconds (t), &SetupGazeFlow, gaze, i, bulkApps[i],
                               (i == 0) ? aTop : aBottom);
        }
      Simulator::Schedule (Seconds (1.0 + gazePeriod), &GazeToggle, gaze);
    }

  // ---------------- Samplers ----------------
  // Throughput series: flows 0 and 1, plus haptics when present.
  SampleState *sampler = new SampleState ();
  const uint32_t nSampled = std::min<uint32_t> (nObjects + (withHaptics ? 1 : 0), 3);
  for (uint32_t i = 0; i < nSampled; ++i)
    {
      sampler->sinks.push_back (sinks[i]);
      sampler->series.push_back (std::vector<double> ());
    }
  sampler->lastBytes.assign (nSampled, 0);
  sampler->interval = sampleInterval;
  sampler->stopTime = simTime;
  Simulator::Schedule (Seconds (sampleInterval), &SampleThroughput, sampler);

  TraceState *tracer = new TraceState ();
  tracer->apMob = apNode.Get (0)->GetObject<MobilityModel> ();
  tracer->sta0Mob = staNodes.Get (0)->GetObject<MobilityModel> ();
  tracer->qd = qdiscs.Get (0);
  tracer->stopTime = simTime;
  Simulator::Schedule (Seconds (1.0), &SampleTrace, tracer);

  FlowMonitorHelper flowmonHelper;
  Ptr<FlowMonitor> monitor = flowmonHelper.InstallAll ();

  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();

  monitor->CheckForLostPackets ();
  Ptr<Ipv4FlowClassifier> classifier =
    DynamicCast<Ipv4FlowClassifier> (flowmonHelper.GetClassifier ());
  const FlowMonitor::FlowStatsContainer &stats = monitor->GetFlowStats ();

  // ---------------- Reporting ----------------
  std::cout << std::fixed << std::setprecision (3);
  std::cout << "=== Stage 4: app-centric priorities over dynamic Wi-Fi ===\n";
  std::cout << "scheme=" << scheme << " nObjects=" << nObjects
            << " gazePeriod=" << gazePeriod << "s aTop=" << aTop
            << " aBottom=" << aBottom
            << " mobility=" << (mobility ? "on" : "off")
            << " speed=" << mobilitySpeed << "rad/s"
            << " rate=" << (adaptiveRate ? "MinstrelHt" : "fixed")
            << " interference=" << (interference ? "on" : "off")
            << " udpBg=" << (udpBackground ? udpBackgroundRate : "off")
            << " [" << udpBackgroundStart << ".." << udpBackgroundStop << "s]"
            << " webBg=" << (classicBackground ? "on" : "off")
            << " aggregation=" << (wifiAggregation ? "on" : "off") << "\n";

  const uint32_t nFlows = names.size ();
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
  for (uint32_t f = 0; f < nFlows; ++f)
    {
      std::cout << "  " << names[f] << ": " << mbps[f] << " Mb/s"
                << " meanDelay=" << delayMs[f] << " ms"
                << " lost=" << lost[f] << "\n";
    }

  // Per-interval throughput of the sampled flows.
  std::cout << "  per-" << sampleInterval << "s throughput (t | objA | objB | haptics Mb/s):\n";
  for (std::size_t k = 0; k < sampler->times.size (); ++k)
    {
      std::cout << "    t=" << sampler->times[k] << ":";
      for (uint32_t i = 0; i < nSampled; ++i)
        {
          std::cout << " " << sampler->series[i][k] << " |";
        }
      std::cout << "\n";
    }

  // Application-weighted log utility (see header comment).
  // Two weight vectors: the fixed quality hierarchy (static) and, when a
  // gaze process ran, one aligned with the flow that currently holds the
  // gaze (judges a run by how well it serves the gazed object).
  const double utilX0 = 1.0;
  const double utilWStatic[3] = {4.0, 2.0, 1.0};
  const double utilWGaze[3] = {3.0, 3.0, 1.0};
  struct UtilAccum
  {
    double sum = 0.0;
    double steadySum = 0.0;
    uint32_t steadyN = 0;
    void Add (double u, double t, double steadyFrom)
    {
      sum += u;
      count++;
      if (t >= steadyFrom)
        {
          steadySum += u;
          steadyN++;
        }
    }
    void Print (const char *label) const
    {
      const double n = static_cast<double> (count);
      std::cout << "  utility [" << label << "]: U=sum w_i log2(1+r_i/x0)"
                << " mean-over-run=" << (n > 0 ? sum / n : 0.0)
                << " mean-over-steady-window="
                << (steadyN > 0 ? steadySum / steadyN : 0.0) << "\n";
    }

    uint32_t count = 0;
  };
  const double steadyFrom = std::max (warmup, simTime / 2.0);
  UtilAccum utilStatic;
  UtilAccum utilGaze;
  const bool haveGazeWeights = static_cast<bool> (gaze);
  for (std::size_t k = 0; k < sampler->times.size (); ++k)
    {
      // Gaze holds flow 0 between switches while startedOnA holds, flow 1
      // otherwise; each toggle flips it.
      const bool onFlow0 = haveGazeWeights
                             ? ((gaze->switchTimes.size () % 2 == 0)
                                == gaze->startedOnA)
                             : true;
      const double utilWGazeK[3] = {onFlow0 ? utilWGaze[0] : utilWGaze[1],
                                    onFlow0 ? utilWGaze[1] : utilWGaze[0],
                                    utilWGaze[2]};
      double uS = 0.0;
      double uG = 0.0;
      for (uint32_t i = 0; i < nSampled; ++i)
        {
          const double term = std::log2 (1.0 + sampler->series[i][k] / utilX0);
          uS += utilWStatic[i] * term;
          uG += utilWGazeK[i] * term;
        }
      const double t = sampler->times[k];
      utilStatic.Add (uS, t, steadyFrom);
      utilGaze.Add (uG, t, steadyFrom);
    }
  if (!sampler->times.empty ())
    {
      utilStatic.Print ("static w=(4,2,1)");
      if (haveGazeWeights)
        {
          utilGaze.Print ("gaze-aligned w=(3,3,1) on gazed flow");
        }
    }

  // Adaptation time after each gaze switch (adaptive scheme only).
  if (gaze && nSampled >= 2)
    {
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
                  continue;
                }
              const double total = sampler->series[0][k] + sampler->series[1][k];
              if (total <= 0.0)
                {
                  continue;
                }
              const double share01 = toA ? sampler->series[0][k] / total
                                         : sampler->series[1][k] / total;
              if (share01 >= threshold)
                {
                  adaptationSum += sampler->times[k] - switchTime;
                  adaptationCount++;
                  break;
                }
            }
        }
      std::cout << "  gaze switches: " << gaze->switchTimes.size ()
                << " (expected gazed share " << 100.0 * expectedShare
                << "%, threshold " << 100.0 * threshold << "%)\n";
      if (adaptationCount > 0)
        {
          std::cout << "  mean adaptation time after switch: "
                    << adaptationSum / adaptationCount << " s ("
                    << adaptationCount << "/" << gaze->switchTimes.size ()
                    << " recovered)\n";
        }
      else
        {
          std::cout << "  mean adaptation time: not measurable in this run\n";
        }
    }

  // Per-second trace: distance of station 0 and DualQ activity.
  std::cout << "  per-second trace (t | sta0-AP dist m | marks | drops):\n";
  for (std::size_t k = 0; k < tracer->times.size (); ++k)
    {
      std::cout << "    t=" << tracer->times[k] << ": "
                << tracer->distances[k] << " | "
                << tracer->marksPerSec[k] << " | "
                << tracer->dropsPerSec[k] << "\n";
    }

  const QueueDisc::Stats qs = qdiscs.Get (0)->GetStats ();
  std::cout << "  AP egress DualQ totals: L4S-marks="
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
  delete tracer;
  return 0;
}
