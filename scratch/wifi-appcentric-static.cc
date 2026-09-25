/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Stage 2: Static application-centric priorities over a Wi-Fi bottleneck.
 *
 * Same bottleneck as wifi-l4s-equal-share.cc (downlink through an 802.11ax
 * AP whose Wi-Fi egress carries the DualQ Coupled PI2 queue disc), but each
 * AR/VR flow uses the application-centric Prague variant with its own FIXED
 * Type-1 (Eq. 6) / Type-2 (Eq. 7) mapping parameters, reproducing the
 * paper's static priority assignments:
 *
 *   flow 0 "focus video":   Type 2, A=2.0   (twice the reference share)
 *   flow 1 "other video":   Type 2, A=0.5   (half the reference share)
 *   flow 2 "audio":         Type 1, p0=0.4, gamma=+0.5 (aggressive below p0)
 *   flow 3 "haptics/chat":  Type 1, p0=0.6, gamma=-0.5 (conservative below p0)
 *
 * Each flow has its own sender node, so its mapping is configured per node:
 * the node's TcpL4Protocol SocketType is set to TcpPragueAppCentric at setup
 * time and the AppCentric* defaults are scheduled just before each flow's
 * application starts (the socket, and with it its congestion-control object,
 * is created inside StartApplication).  This avoids any runtime replacement
 * of congestion-control objects, which would discard the DCTCP alpha state.
 *
 * All parameters are overridable per flow on the command line
 * (--typeI --p0I --gammaI --aI, I in 0..3).  With all A=1 / gamma=0 the
 * scenario degenerates to the Stage 1 equal-share baseline.
 *
 * This is an L4S-inspired ECN/AQM Wi-Fi evaluation: it does not implement
 * full Accurate ECN / L4S behaviour at the Wi-Fi bottleneck.
 *
 * Run from the repository root:
 *   ./waf --run "wifi-appcentric-static --flows=4 --simTime=30"
 *   ./waf --run "wifi-appcentric-static --flows=4 --a0=4 --a1=0.25"
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

NS_LOG_COMPONENT_DEFINE ("WifiAppCentricStatic");

static const double SCENARIO_PI = 3.141592653589793;

// Scheduled just before flow i's application starts: the BulkSend socket (and
// its TcpPragueAppCentric object) is created inside StartApplication, which
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

int
main (int argc, char *argv[])
{
  // ---------------- Command line ----------------
  uint32_t flows = 4;
  double simTime = 30.0;
  double warmup = 5.0;
  std::string wifiStandard = "802.11ax-5GHz";
  uint32_t wifiMcs = 0;
  double apStaDistance = 5.0;
  bool classicBackground = false; // add a competing NewReno flow (station 0)
  std::string accessRate = "100Mbps"; // per-flow wired access links (not bottleneck)
  std::string accessDelay = "1ms";
  std::string l4sMarkThreshold = "475us"; // DualQ L4S sojourn marking threshold
  bool wifiAggregation = false;   // A-MPDU/MSDU aggregation on Wi-Fi

  // Per-flow static application-centric parameters (index by flow number).
  int type[4] = {2, 2, 1, 1};
  double p0[4] = {0.5, 0.5, 0.4, 0.6};
  double gamma[4] = {0.0, 0.0, 0.5, -0.5};
  double a[4] = {2.0, 0.5, 1.0, 1.0};

  CommandLine cmd;
  cmd.AddValue ("flows", "Number of AR/VR flows (1-4)", flows);
  cmd.AddValue ("simTime", "Total simulation time in seconds", simTime);
  cmd.AddValue ("warmup", "Startup transient excluded from measurements (s)", warmup);
  cmd.AddValue ("wifiStandard", "802.11ax-5GHz, 802.11ac or 802.11n-5GHz", wifiStandard);
  cmd.AddValue ("wifiMcs", "Fixed HE/VHT/HT MCS index for all data", wifiMcs);
  cmd.AddValue ("apStaDistance", "Distance of each station from the AP (m)", apStaDistance);
  cmd.AddValue ("classicBackground", "Add a competing classic TCP (NewReno) flow", classicBackground);
  cmd.AddValue ("accessRate", "Per-flow wired access link data rate", accessRate);
  cmd.AddValue ("accessDelay", "Per-flow wired access link delay", accessDelay);
  cmd.AddValue ("l4sMarkThreshold",
                "DualQ L4S sojourn marking threshold (e.g. 475us, 2ms)",
                l4sMarkThreshold);
  cmd.AddValue ("wifiAggregation",
                "Enable A-MPDU/MSDU aggregation on the Wi-Fi bottleneck",
                wifiAggregation);
  for (uint32_t i = 0; i < 4; ++i)
    {
      const std::string idx = std::to_string (i);
      cmd.AddValue (("type" + idx).c_str (),
                    ("Flow " + idx + " mapping: 1 (Eq. 6) or 2 (Eq. 7)").c_str (),
                    type[i]);
      cmd.AddValue (("p0" + idx).c_str (),
                    ("Flow " + idx + " Type-1 transition point p0").c_str (),
                    p0[i]);
      cmd.AddValue (("gamma" + idx).c_str (),
                    ("Flow " + idx + " Type-1 blend parameter gamma in [-1,1]").c_str (),
                    gamma[i]);
      cmd.AddValue (("a" + idx).c_str (),
                    ("Flow " + idx + " Type-2 throughput weight A").c_str (),
                    a[i]);
    }
  cmd.Parse (argc, argv);

  if (flows == 0 || flows > 4 || simTime <= warmup)
    {
      NS_FATAL_ERROR ("Need 1-4 flows and simTime > warmup");
    }
  for (uint32_t i = 0; i < flows; ++i)
    {
      if ((type[i] != 1 && type[i] != 2) || p0[i] < 0.0 || p0[i] > 1.0
          || gamma[i] < -1.0 || gamma[i] > 1.0 || a[i] <= 0.0)
        {
          NS_FATAL_ERROR ("Invalid per-flow parameters for flow " << i);
        }
    }

  // ---------------- Scenario-wide defaults (same as Stage 1) ----------------
  Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (1000));
  Config::SetDefault ("ns3::TcpSocket::DelAckCount", UintegerValue (1));
  Config::SetDefault ("ns3::TcpSocketBase::EcnMode", StringValue ("ClassicEcn"));
  Config::SetDefault ("ns3::DualQCoupledPi2QueueDisc::Mtu", UintegerValue (1000));
  Config::SetDefault ("ns3::DualQCoupledPi2QueueDisc::QueueLimit", UintegerValue (100000));
  Config::SetDefault ("ns3::DualQCoupledPi2QueueDisc::L4SMarkThresold",
                      TimeValue (Time (l4sMarkThreshold)));
  if (!wifiAggregation)
    {
      // Aggregation bursts enqueue several data frames in one go, which
      // pushes L4S-queue sojourn over the marking threshold even when the
      // bottleneck is not really saturated.  Disabling aggregation gives
      // the AQM smoother, packet-paced queue dynamics.
      Config::SetDefault ("ns3::RegularWifiMac::VO_MaxAmpduSize", UintegerValue (0));
      Config::SetDefault ("ns3::RegularWifiMac::VI_MaxAmpduSize", UintegerValue (0));
      Config::SetDefault ("ns3::RegularWifiMac::BE_MaxAmpduSize", UintegerValue (0));
      Config::SetDefault ("ns3::RegularWifiMac::BK_MaxAmpduSize", UintegerValue (0));
      Config::SetDefault ("ns3::RegularWifiMac::VO_MaxAmsduSize", UintegerValue (0));
      Config::SetDefault ("ns3::RegularWifiMac::VI_MaxAmsduSize", UintegerValue (0));
      Config::SetDefault ("ns3::RegularWifiMac::BE_MaxAmsduSize", UintegerValue (0));
      Config::SetDefault ("ns3::RegularWifiMac::BK_MaxAmsduSize", UintegerValue (0));
    }
  Config::SetDefault ("ns3::WifiMacQueue::MaxQueueSize", StringValue ("40p"));

  // ---------------- Topology ----------------
  // One sender (AR/VR content server) per flow, each on its own fast wired
  // access link to the AP.  The AP's Wi-Fi egress remains the only bottleneck.
  NodeContainer apNode, staNodes, serverNodes;
  apNode.Create (1);
  staNodes.Create (flows);
  // One extra dedicated sender for the optional classic background flow.
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
  Ssid ssid = Ssid ("wifi-appcentric-static");
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
  static const char *flowNames[4] = {"focus-video", "other-video", "audio", "haptics"};
  const uint32_t nFlows = flows + (classicBackground ? 1 : 0);

  std::vector<Ptr<PacketSink>> sinks;
  std::vector<std::string> names;
  std::vector<uint16_t> ports;
  std::vector<Ipv4Address> dsts;
  sinks.reserve (nFlows);
  names.reserve (nFlows);
  ports.reserve (nFlows);
  dsts.reserve (nFlows);

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
      Simulator::Schedule (Seconds (start - 0.04), &ConfigureFlowDefaults,
                           type[i], p0[i], gamma[i], a[i]);

      BulkSendHelper bulk ("ns3::TcpSocketFactory",
                           InetSocketAddress (wifiIfs.GetAddress (1 + i), port));
      bulk.SetAttribute ("SendSize", UintegerValue (1000));
      bulk.SetAttribute ("MaxBytes", UintegerValue (0));
      ApplicationContainer sourceApp = bulk.Install (serverNodes.Get (i));
      sourceApp.Start (Seconds (start));
      sourceApp.Stop (Seconds (simTime - 0.5));
    }

  if (classicBackground)
    {
      // A dedicated classic sender node keeps NewReno independent of the
      // app-centric per-flow default scheduling.
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

  FlowMonitorHelper flowmonHelper;
  Ptr<FlowMonitor> monitor = flowmonHelper.InstallAll ();

  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();

  monitor->CheckForLostPackets ();
  Ptr<Ipv4FlowClassifier> classifier =
    DynamicCast<Ipv4FlowClassifier> (flowmonHelper.GetClassifier ());
  const FlowMonitor::FlowStatsContainer &stats = monitor->GetFlowStats ();

  // ---------------- Reporting ----------------
  NS_UNUSED (warmup); // whole-run averages reported; see l4s-paper-baseline.cc
  std::cout << std::fixed << std::setprecision (3);
  std::cout << "=== Stage 2: static application-centric priorities over Wi-Fi ===\n";
  std::cout << "flows=" << flows << " standard=" << wifiStandard
            << " mcs=" << wifiMcs << " distance=" << apStaDistance << "m"
            << " background=" << (classicBackground ? "classic" : "off") << "\n";

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
      if (f < flows)
        {
          std::cout << " (type" << type[f]
                    << (type[f] == 2 ? " A=" + std::to_string (a[f])
                                     : " p0=" + std::to_string (p0[f])
                                           + " gamma=" + std::to_string (gamma[f]))
                    << " share=" << (sumL4s > 0 ? 100.0 * mbps[f] / sumL4s : 0.0) << "%)";
        }
      std::cout << "\n";
    }

  if (flows > 1 && sumL4s > 0)
    {
      double sumSq = 0.0;
      for (uint32_t f = 0; f < flows; ++f)
        {
          sumSq += mbps[f] * mbps[f];
        }
      std::cout << "  Jain fairness over app-centric flows: "
                << sumL4s * sumL4s / (flows * sumSq)
                << " (below 1 is intentional: static priorities)\n";
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
  return 0;
}
