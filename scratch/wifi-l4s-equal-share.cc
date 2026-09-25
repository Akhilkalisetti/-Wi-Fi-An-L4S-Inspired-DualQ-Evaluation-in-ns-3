/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Stage 1: Equal-share baseline for AR/VR traffic over Wi-Fi.
 *
 * N identical, continuously backlogged TCP DCTCP (ECT(1)) flows download from
 * one wired server through an 802.11ax access point.  Unlike
 * wifi_prague_test.cc, there is NO narrow point-to-point bottleneck: the
 * shared bottleneck is the AP's Wi-Fi egress itself, where a DualQ Coupled
 * PI2 queue disc (L4S-inspired ECN/AQM) is installed.  The AP's Wi-Fi MAC
 * queue is kept shallow so the AQM, not the MAC, controls the backlog.
 * Wi-Fi contention and the configured PHY rate therefore shape capacity.
 *
 * Traffic is DOWNLINK (server -> stations), so all data passes through the
 * AP's Wi-Fi egress queue.
 *
 * This is an L4S-inspired ECN/AQM Wi-Fi evaluation: it does not implement
 * full Accurate ECN / L4S behaviour at the Wi-Fi bottleneck (classic ECE/CWR
 * ECN feedback is used by the DCTCP endpoints).
 *
 * Run from the repository root:
 *   ./waf --run "wifi-l4s-equal-share --stations=3 --simTime=30"
 *   ./waf --run "wifi-l4s-equal-share --stations=3 --backgroundFlow=true"
 */
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/traffic-control-module.h"
#include "ns3/wifi-module.h"

#include <cmath>
#include <iomanip>
#include <string>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("WifiL4sEqualShare");

int
main (int argc, char *argv[])
{
  // ---------------- Command line ----------------
  uint32_t stations = 3;          // AR/VR client stations (one backlogged flow each)
  double simTime = 30.0;          // total simulated time (s)
  double warmup = 5.0;            // excluded TCP start-up transient (s)
  std::string wifiStandard = "802.11ax-5GHz";
  uint32_t wifiMcs = 0;           // fixed HE/VHT/HT MCS index (0-11)
  double apStaDistance = 5.0;     // constant STA distance from AP (m)
  bool backgroundFlow = false;    // add one competing classic (NewReno) flow
  bool apEgressAqm = true;        // false = plain drop-tail Wi-Fi (no AQM)
  bool wifiAggregation = false;   // A-MPDU/MSDU aggregation on Wi-Fi
  std::string wiredRate = "1Gbps"; // server-AP link (deliberately not a bottleneck)
  std::string wiredDelay = "1ms";

  CommandLine cmd;
  cmd.AddValue ("stations", "Number of AR/VR stations with one backlogged flow each", stations);
  cmd.AddValue ("simTime", "Total simulation time in seconds", simTime);
  cmd.AddValue ("warmup", "Startup transient excluded from measurements (s)", warmup);
  cmd.AddValue ("wifiStandard", "802.11ax-5GHz, 802.11ac or 802.11n-5GHz", wifiStandard);
  cmd.AddValue ("wifiMcs", "Fixed HE/VHT/HT MCS index for all data", wifiMcs);
  cmd.AddValue ("apStaDistance", "Distance of each station from the AP (m)", apStaDistance);
  cmd.AddValue ("backgroundFlow", "Add a competing classic TCP (NewReno) flow", backgroundFlow);
  cmd.AddValue ("apEgressAqm", "Install DualQ AQM on the AP Wi-Fi egress", apEgressAqm);
  cmd.AddValue ("wifiAggregation",
                "Enable A-MPDU/MSDU aggregation on the Wi-Fi bottleneck",
                wifiAggregation);
  cmd.AddValue ("wiredRate", "Server-AP wired link data rate", wiredRate);
  cmd.AddValue ("wiredDelay", "Server-AP wired link delay", wiredDelay);
  cmd.Parse (argc, argv);

  if (stations == 0 || simTime <= warmup)
    {
      NS_FATAL_ERROR ("Need at least one station and simTime > warmup");
    }

  // ---------------- Scenario-wide defaults ----------------
  // DCTCP endpoints send ECT(1); the DualQ in this tree classifies ECT(1)/CE
  // packets into its L4S queue.  This is the L4S-inspired part of the setup.
  Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (1000));
  Config::SetDefault ("ns3::TcpSocket::DelAckCount", UintegerValue (1));
  Config::SetDefault ("ns3::TcpSocketBase::EcnMode", StringValue ("ClassicEcn"));
  Config::SetDefault ("ns3::DualQCoupledPi2QueueDisc::Mtu", UintegerValue (1000));
  Config::SetDefault ("ns3::DualQCoupledPi2QueueDisc::QueueLimit", UintegerValue (100000));

  // The Wi-Fi MAC queue must be shallow so that the DualQ at the AP egress,
  // not the MAC, controls the backlog (the AP serves all downlink flows).
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
  NodeContainer serverNode, apNode, staNodes;
  serverNode.Create (1);
  apNode.Create (1);
  staNodes.Create (stations);

  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positions = CreateObject<ListPositionAllocator> ();
  positions->Add (Vector (0.0, 0.0, 0.0)); // AP
  positions->Add (Vector (0.0, 0.0, 0.0)); // server (irrelevant, wired)
  for (uint32_t i = 0; i < stations; ++i)
    {
      // Ring of stations around the AP at a fixed distance.
      const double angle = 2.0 * M_PI * i / stations;
      positions->Add (Vector (apStaDistance * std::cos (angle),
                              apStaDistance * std::sin (angle), 0.0));
    }
  mobility.SetPositionAllocator (positions);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (NodeContainer (apNode, serverNode, staNodes));

  // ---------------- Wi-Fi setup ----------------
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
  // Fixed rate: Wi-Fi capacity is a known quantity per scenario, which is
  // what stages 1-3 require.  (Stage 4 would replace this with Minstrel.)
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode", StringValue (dataMode),
                                "ControlMode", StringValue (mcsBase + "0"));

  WifiMacHelper mac;
  Ssid ssid = Ssid ("wifi-l4s-equal-share");
  mac.SetType ("ns3::StaWifiMac",
               "Ssid", SsidValue (ssid),
               "ActiveProbing", BooleanValue (false));
  NetDeviceContainer staDevs = wifi.Install (phy, mac, staNodes);
  mac.SetType ("ns3::ApWifiMac",
               "Ssid", SsidValue (ssid));
  NetDeviceContainer apDev = wifi.Install (phy, mac, apNode);

  // ---------------- Wired server-AP link (not a bottleneck) ----------------
  PointToPointHelper p2p;
  p2p.SetDeviceAttribute ("DataRate", StringValue (wiredRate));
  p2p.SetChannelAttribute ("Delay", StringValue (wiredDelay));
  NetDeviceContainer p2pDevs = p2p.Install (serverNode.Get (0), apNode.Get (0));

  InternetStackHelper internet;
  internet.Install (NodeContainer (serverNode, apNode, staNodes));
  Config::Set ("/NodeList/" + std::to_string (apNode.Get (0)->GetId ())
                 + "/$ns3::Ipv4L3Protocol/IpForward", BooleanValue (true));

  // ---------------- The bottleneck: AP Wi-Fi egress queue ----------------
  TrafficControlHelper tch;
  QueueDiscContainer qdiscs;
  if (apEgressAqm)
    {
      uint16_t handle = tch.SetRootQueueDisc ("ns3::DualQCoupledPi2QueueDisc");
      tch.AddInternalQueues (handle, 2, "ns3::DropTailQueue",
                             "MaxSize", StringValue ("100000B"));
      qdiscs = tch.Install (apDev.Get (0));
      NS_UNUSED (handle);
    }

  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer p2pIfs = ipv4.Assign (p2pDevs);
  ipv4.SetBase ("10.1.2.0", "255.255.255.0");
  NetDeviceContainer wifiDevs;
  wifiDevs.Add (apDev.Get (0));
  wifiDevs.Add (staDevs);
  Ipv4InterfaceContainer wifiIfs = ipv4.Assign (wifiDevs);

  Ipv4StaticRoutingHelper routingHelper;
  Ptr<Ipv4StaticRouting> serverRouting =
    routingHelper.GetStaticRouting (serverNode.Get (0)->GetObject<Ipv4> ());
  serverRouting->SetDefaultRoute (p2pIfs.GetAddress (1), 1);
  for (uint32_t i = 0; i < stations; ++i)
    {
      Ptr<Ipv4StaticRouting> staRouting =
        routingHelper.GetStaticRouting (staNodes.Get (i)->GetObject<Ipv4> ());
      staRouting->SetDefaultRoute (wifiIfs.GetAddress (0), 1);
    }

  // ---------------- Applications: one backlogged flow per station ----------------
  // Sockets are created by the applications at StartApplication time, so the
  // per-node TcpL4Protocol SocketType at that moment selects the variant.
  // All flows are L4S-flavoured DCTCP/ECT(1) in the equal-share baseline.
  const std::string serverTcpPath =
    "/NodeList/" + std::to_string (serverNode.Get (0)->GetId ())
    + "/$ns3::TcpL4Protocol/SocketType";
  Config::Set (serverTcpPath, TypeIdValue (TcpDctcp::GetTypeId ()));

  const uint32_t nClassic = backgroundFlow ? 1 : 0;
  const uint32_t nFlows = stations + nClassic;
  std::vector<Ptr<PacketSink>> sinks;
  sinks.reserve (nFlows);
  std::vector<std::string> flowNames;
  flowNames.reserve (nFlows);
  std::vector<uint16_t> flowPorts;
  flowPorts.reserve (nFlows);
  std::vector<Ipv4Address> flowDst;
  flowDst.reserve (nFlows);

  for (uint32_t i = 0; i < stations; ++i)
    {
      const uint16_t port = static_cast<uint16_t> (50000 + i);
      PacketSinkHelper sinkHelper ("ns3::TcpSocketFactory",
                                   InetSocketAddress (Ipv4Address::GetAny (), port));
      ApplicationContainer sinkApp = sinkHelper.Install (staNodes.Get (i));
      sinkApp.Start (Seconds (0.0));
      sinkApp.Stop (Seconds (simTime));
      sinks.push_back (DynamicCast<PacketSink> (sinkApp.Get (0)));
      flowNames.push_back ("l4s-flow-" + std::to_string (i));
      flowPorts.push_back (port);
      flowDst.push_back (wifiIfs.GetAddress (1 + i));

      BulkSendHelper bulk ("ns3::TcpSocketFactory",
                           InetSocketAddress (wifiIfs.GetAddress (1 + i), port));
      bulk.SetAttribute ("SendSize", UintegerValue (1000));
      bulk.SetAttribute ("MaxBytes", UintegerValue (0)); // persistent/backlogged
      ApplicationContainer sourceApp = bulk.Install (serverNode.Get (0));
      sourceApp.Start (Seconds (0.5));
      sourceApp.Stop (Seconds (simTime - 0.5));
    }

  if (backgroundFlow)
    {
      // One competing classic TCP flow (NewReno, ECT(0) => DualQ classic
      // queue).  Switch the node default before its socket is created.
      const uint16_t port = 51000;
      Config::Set (serverTcpPath, TypeIdValue (TcpNewReno::GetTypeId ()));
      PacketSinkHelper sinkHelper ("ns3::TcpSocketFactory",
                                   InetSocketAddress (Ipv4Address::GetAny (), port));
      ApplicationContainer sinkApp = sinkHelper.Install (staNodes.Get (0));
      sinkApp.Start (Seconds (0.0));
      sinkApp.Stop (Seconds (simTime));
      sinks.push_back (DynamicCast<PacketSink> (sinkApp.Get (0)));
      flowNames.push_back ("classic-background");
      flowPorts.push_back (port);
      flowDst.push_back (wifiIfs.GetAddress (1));

      BulkSendHelper bulk ("ns3::TcpSocketFactory",
                           InetSocketAddress (wifiIfs.GetAddress (1), port));
      bulk.SetAttribute ("SendSize", UintegerValue (1000));
      bulk.SetAttribute ("MaxBytes", UintegerValue (0));
      ApplicationContainer sourceApp = bulk.Install (serverNode.Get (0));
      sourceApp.Start (Seconds (0.5));
      sourceApp.Stop (Seconds (simTime - 0.5));
    }

  // ---------------- Reporting ----------------
  FlowMonitorHelper flowmonHelper;
  Ptr<FlowMonitor> monitor = flowmonHelper.InstallAll ();

  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();

  monitor->CheckForLostPackets ();
  Ptr<Ipv4FlowClassifier> classifier =
    DynamicCast<Ipv4FlowClassifier> (flowmonHelper.GetClassifier ());
  const FlowMonitor::FlowStatsContainer &stats = monitor->GetFlowStats ();

  std::cout << std::fixed << std::setprecision (3);
  std::cout << "=== Stage 1: equal-share L4S-inspired baseline over Wi-Fi ===\n";
  std::cout << "stations=" << stations << " standard=" << wifiStandard
            << " mcs=" << wifiMcs << " distance=" << apStaDistance << "m"
            << " apEgressAqm=" << (apEgressAqm ? "DualQ" : "none")
            << " background=" << (backgroundFlow ? "classic" : "off") << "\n";

  std::vector<double> mbps (nFlows, 0.0);
  std::vector<double> delayMs (nFlows, 0.0);
  std::vector<uint32_t> lost (nFlows, 0);
  for (const auto &entry : stats)
    {
      const Ipv4FlowClassifier::FiveTuple tuple = classifier->FindFlow (entry.first);
      for (uint32_t f = 0; f < nFlows; ++f)
        {
          if (tuple.destinationPort == flowPorts[f]
              && tuple.destinationAddress == flowDst[f])
            {
              // Sink apps are active for the whole simulation.
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
  for (uint32_t f = 0; f < stations; ++f)
    {
      sumL4s += mbps[f];
    }
  const double meanL4s = stations > 0 ? sumL4s / stations : 0.0;
  for (uint32_t f = 0; f < nFlows; ++f)
    {
      std::cout << "  " << flowNames[f]
                << ": " << mbps[f] << " Mb/s"
                << " meanDelay=" << delayMs[f] << " ms"
                << " lost=" << lost[f];
      if (f < stations && meanL4s > 0)
        {
          std::cout << " dev-from-mean=" << 100.0 * (mbps[f] - meanL4s) / meanL4s << "%";
        }
      std::cout << "\n";
    }

  if (stations > 1 && sumL4s > 0)
    {
      double sumSq = 0.0;
      for (uint32_t f = 0; f < stations; ++f)
        {
          sumSq += mbps[f] * mbps[f];
        }
      const double jain = sumL4s * sumL4s / (stations * sumSq);
      std::cout << "  Jain fairness over L4S flows: " << jain << "\n";
    }

  if (apEgressAqm)
    {
      const QueueDisc::Stats qs = qdiscs.Get (0)->GetStats ();
      const uint32_t l4sMarks =
        qs.GetNMarkedPackets (DualQCoupledPi2QueueDisc::UNFORCED_L4S_MARK);
      const uint32_t cMarks =
        qs.GetNMarkedPackets (DualQCoupledPi2QueueDisc::UNFORCED_CLASSIC_MARK);
      const uint32_t drops =
        qs.GetNDroppedPackets (DualQCoupledPi2QueueDisc::UNFORCED_CLASSIC_DROP)
        + qs.GetNDroppedPackets (DualQCoupledPi2QueueDisc::FORCED_DROP);
      std::cout << "  AP egress DualQ: L4S-marks=" << l4sMarks
                << " classic-marks=" << cMarks
                << " drops=" << drops << "\n";
    }

  Simulator::Destroy ();
  return 0;
}
