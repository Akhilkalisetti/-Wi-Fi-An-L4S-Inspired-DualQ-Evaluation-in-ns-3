/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Minimal Wi-Fi simulation for testing modified TCP Prague / DCTCP and
 * application-centric congestion control in ns-3.
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/tcp-dctcp.h"
#include "ns3/tcp-prague-app-centric.h"
#include "ns3/traffic-control-module.h"
#include "ns3/wifi-module.h"

#include <iomanip>
#include <sstream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("WifiPragueTest");

struct FlowSetup
{
  std::string name;
  uint32_t staIndex;
  bool appCentric;
  TcpPragueAppCentric::AppCentricType mappingType;
  double p0;
  double gamma;
  double a;
  uint16_t port;
  double startTime;
};

static void
SetNodeSocketType (uint32_t nodeId, TypeId socketType)
{
  std::ostringstream path;
  path << "/NodeList/" << nodeId << "/$ns3::TcpL4Protocol/SocketType";
  Config::Set (path.str (), TypeIdValue (socketType));
}

static void
ConfigureAppCentricDefaults (TcpPragueAppCentric::AppCentricType mappingType,
                             double p0,
                             double gamma,
                             double a)
{
  Config::SetDefault ("ns3::TcpPragueAppCentric::AppCentricType",
                      EnumValue (mappingType));
  Config::SetDefault ("ns3::TcpPragueAppCentric::AppCentricP0",
                      DoubleValue (p0));
  Config::SetDefault ("ns3::TcpPragueAppCentric::AppCentricGamma",
                      DoubleValue (gamma));
  Config::SetDefault ("ns3::TcpPragueAppCentric::AppCentricA",
                      DoubleValue (a));
}

static void
SampleThroughput (const std::vector<Ptr<PacketSink>> &sinks,
                  std::vector<uint64_t> *lastRxBytes,
                  double interval,
                  double stopTime)
{
  const double now = Simulator::Now ().GetSeconds ();
  for (std::size_t i = 0; i < sinks.size (); ++i)
    {
      const uint64_t current = sinks[i]->GetTotalRx ();
      const double mbps = static_cast<double> (current - (*lastRxBytes)[i]) * 8.0 / interval / 1e6;
      std::cout << std::fixed << std::setprecision (3)
                << "t=" << now << "s flow=" << i
                << " throughput=" << mbps << " Mbps" << std::endl;
      (*lastRxBytes)[i] = current;
    }

  if (now + interval <= stopTime)
    {
      Simulator::Schedule (Seconds (interval), &SampleThroughput,
                           sinks, lastRxBytes, interval, stopTime);
    }
}

static WifiPhyStandard
ResolveWifiStandard (const std::string &standardName)
{
  if (standardName == "WIFI_STANDARD_80211n" || standardName == "802.11n-5GHz")
    {
      return WIFI_PHY_STANDARD_80211n_5GHZ;
    }
  if (standardName == "802.11n-2.4GHz")
    {
      return WIFI_PHY_STANDARD_80211n_2_4GHZ;
    }
  if (standardName == "WIFI_STANDARD_80211ac" || standardName == "802.11ac")
    {
      return WIFI_PHY_STANDARD_80211ac;
    }
  if (standardName == "WIFI_STANDARD_80211ax" || standardName == "802.11ax-5GHz")
    {
      return WIFI_PHY_STANDARD_80211ax_5GHZ;
    }
  if (standardName == "802.11ax-2.4GHz")
    {
      return WIFI_PHY_STANDARD_80211ax_2_4GHZ;
    }

  if (standardName == "WIFI_STANDARD_80211be" || standardName == "802.11be")
    {
      NS_LOG_WARN ("802.11be is not available in this ns-3 tree; falling back to 802.11ax-5GHz");
      return WIFI_PHY_STANDARD_80211ax_5GHZ;
    }

  NS_LOG_WARN ("Unknown wifiStandard='" << standardName << "', falling back to 802.11ax-5GHz");
  return WIFI_PHY_STANDARD_80211ax_5GHZ;
}

static std::string
ResolveWifiDataMode (WifiPhyStandard standard)
{
  switch (standard)
    {
    case WIFI_PHY_STANDARD_80211n_2_4GHZ:
    case WIFI_PHY_STANDARD_80211n_5GHZ:
      return "HtMcs0";
    case WIFI_PHY_STANDARD_80211ac:
      return "VhtMcs0";
    case WIFI_PHY_STANDARD_80211ax_2_4GHZ:
    case WIFI_PHY_STANDARD_80211ax_5GHZ:
      return "HeMcs0";
    default:
      return "OfdmRate6Mbps";
    }
}

int
main (int argc, char *argv[])
{
  std::string wifiStandard = "802.11ax-5GHz";
  uint32_t nSta = 3;
  double simTime = 20.0;
  double sampleInterval = 1.0;
  std::string bottleneckRate = "20Mbps";
  std::string bottleneckDelay = "2ms";
  std::string wifiRate = "54Mbps";
  std::string wifiDelay = "1ms";
  bool useRedEcn = true;

  CommandLine cmd;
  cmd.AddValue ("wifiStandard", "802.11n-2.4GHz, 802.11n-5GHz, 802.11ac, 802.11ax-2.4GHz, 802.11ax-5GHz, or 802.11be (falls back to ax)", wifiStandard);
  cmd.AddValue ("nSta", "Number of STA senders (1-3)", nSta);
  cmd.AddValue ("simTime", "Simulation time in seconds", simTime);
  cmd.AddValue ("sampleInterval", "Throughput sampling interval in seconds", sampleInterval);
  cmd.AddValue ("bottleneckRate", "Point-to-point bottleneck data rate", bottleneckRate);
  cmd.AddValue ("bottleneckDelay", "Point-to-point bottleneck delay", bottleneckDelay);
  cmd.AddValue ("wifiRate", "Wi-Fi PHY data rate for the example", wifiRate);
  cmd.AddValue ("wifiDelay", "Wi-Fi channel delay for the example", wifiDelay);
  cmd.AddValue ("useRedEcn", "Enable ECN on the RED queue disc", useRedEcn);
  cmd.Parse (argc, argv);

  nSta = std::min<uint32_t> (std::max<uint32_t> (nSta, 1), 3);

  Config::SetDefault ("ns3::TcpSocketBase::EcnMode", StringValue ("ClassicEcn"));

  NodeContainer serverNode;
  serverNode.Create (1);
  NodeContainer apNode;
  apNode.Create (1);
  NodeContainer staNodes;
  staNodes.Create (nSta);

  NodeContainer allNodes;
  allNodes.Add (serverNode);
  allNodes.Add (apNode);
  allNodes.Add (staNodes);

  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positions = CreateObject<ListPositionAllocator> ();
  positions->Add (Vector (0.0, 0.0, 0.0));
  positions->Add (Vector (5.0, 0.0, 0.0));
  for (uint32_t i = 0; i < nSta; ++i)
    {
      positions->Add (Vector (15.0 + i * 2.0, 2.0 + i * 2.0, 0.0));
    }
  mobility.SetPositionAllocator (positions);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (allNodes);

  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
  YansWifiPhyHelper phy = YansWifiPhyHelper::Default ();
  phy.SetChannel (channel.Create ());

  WifiHelper wifi;
  const WifiPhyStandard standard = ResolveWifiStandard (wifiStandard);
  wifi.SetStandard (standard);
  if (standard == WIFI_PHY_STANDARD_80211ax_2_4GHZ || standard == WIFI_PHY_STANDARD_80211ax_5GHZ)
    {
      wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                    "DataMode", StringValue (ResolveWifiDataMode (standard)),
                                    "ControlMode", StringValue ("HeMcs0"));
    }
  else if (standard == WIFI_PHY_STANDARD_80211ac)
    {
      wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                    "DataMode", StringValue (ResolveWifiDataMode (standard)),
                                    "ControlMode", StringValue ("VhtMcs0"));
    }
  else if (standard == WIFI_PHY_STANDARD_80211n_2_4GHZ || standard == WIFI_PHY_STANDARD_80211n_5GHZ)
    {
      wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                    "DataMode", StringValue (ResolveWifiDataMode (standard)),
                                    "ControlMode", StringValue ("HtMcs0"));
    }
  else
    {
      wifi.SetRemoteStationManager ("ns3::IdealWifiManager");
    }

  WifiMacHelper mac;
  Ssid ssid = Ssid ("wifi-prague-test");

  mac.SetType ("ns3::StaWifiMac",
               "Ssid", SsidValue (ssid),
               "ActiveProbing", BooleanValue (false));
  NetDeviceContainer staDevs = wifi.Install (phy, mac, staNodes);

  mac.SetType ("ns3::ApWifiMac",
               "Ssid", SsidValue (ssid));
  NetDeviceContainer apDev = wifi.Install (phy, mac, apNode);

  PointToPointHelper p2p;
  p2p.SetDeviceAttribute ("DataRate", StringValue (bottleneckRate));
  p2p.SetChannelAttribute ("Delay", StringValue (bottleneckDelay));
  NetDeviceContainer p2pDevs = p2p.Install (serverNode.Get (0), apNode.Get (0));

  InternetStackHelper internet;
  internet.Install (allNodes);

  Config::Set ("/NodeList/1/$ns3::Ipv4L3Protocol/IpForward", BooleanValue (true));

  TrafficControlHelper tch;
  if (useRedEcn)
    {
      tch.SetRootQueueDisc ("ns3::RedQueueDisc",
                            "MaxSize", StringValue ("100p"),
                            "MinTh", DoubleValue (5),
                            "MaxTh", DoubleValue (15),
                            "QW", DoubleValue (0.002),
                            "Wait", BooleanValue (true),
                            "Gentle", BooleanValue (true),
                            "UseEcn", BooleanValue (true),
                            "MeanPktSize", UintegerValue (1000));
    }
  else
    {
      tch.SetRootQueueDisc ("ns3::RedQueueDisc",
                            "MaxSize", StringValue ("100p"),
                            "MinTh", DoubleValue (5),
                            "MaxTh", DoubleValue (15),
                            "QW", DoubleValue (0.002),
                            "Wait", BooleanValue (true),
                            "Gentle", BooleanValue (true),
                            "UseEcn", BooleanValue (false),
                            "MeanPktSize", UintegerValue (1000));
    }
  QueueDiscContainer qdiscs = tch.Install (p2pDevs.Get (0));
  Ptr<QueueDisc> rootQueueDisc = qdiscs.Get (0);

  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer p2pIfs = ipv4.Assign (p2pDevs);

  ipv4.SetBase ("10.1.2.0", "255.255.255.0");
  NetDeviceContainer wifiDevs;
  wifiDevs.Add (apDev.Get (0));
  wifiDevs.Add (staDevs);
  Ipv4InterfaceContainer wifiIfs = ipv4.Assign (wifiDevs);

  Ipv4StaticRoutingHelper staticRoutingHelper;
  Ptr<Ipv4StaticRouting> serverStaticRouting =
    staticRoutingHelper.GetStaticRouting (serverNode.Get (0)->GetObject<Ipv4> ());
  serverStaticRouting->SetDefaultRoute (p2pIfs.GetAddress (1), 1);

  for (uint32_t i = 0; i < nSta; ++i)
    {
      Ptr<Ipv4StaticRouting> staStaticRouting =
        staticRoutingHelper.GetStaticRouting (staNodes.Get (i)->GetObject<Ipv4> ());
      staStaticRouting->SetDefaultRoute (wifiIfs.GetAddress (0), 1);
    }

  const std::vector<FlowSetup> flows =
    {
      {"reference-dctcp", 0, false, TcpPragueAppCentric::TYPE1, 0.5, 8.0, 1.0, 50000, 1.0},
      {"app-centric-type1", 1, true, TcpPragueAppCentric::TYPE1, 0.35, 12.0, 1.0, 50001, 1.2},
      {"app-centric-type2", 2, true, TcpPragueAppCentric::TYPE2, 0.5, 8.0, 0.75, 50002, 1.4}
    };

  std::vector<Ptr<PacketSink>> sinks;
  std::vector<ApplicationContainer> sinkApps;
  std::vector<ApplicationContainer> sourceApps;
  sinks.reserve (flows.size ());
  sinkApps.reserve (flows.size ());
  sourceApps.reserve (flows.size ());

  for (const auto &flow : flows)
    {
      if (flow.staIndex >= nSta)
        {
          continue;
        }

      Address sinkLocalAddress (InetSocketAddress (Ipv4Address::GetAny (), flow.port));
      PacketSinkHelper sinkHelper ("ns3::TcpSocketFactory", sinkLocalAddress);
      ApplicationContainer sinkApp = sinkHelper.Install (serverNode.Get (0));
      sinkApp.Start (Seconds (0.0));
      sinkApp.Stop (Seconds (simTime));
      sinkApps.push_back (sinkApp);
      sinks.push_back (DynamicCast<PacketSink> (sinkApp.Get (0)));

      const uint32_t nodeId = 2 + flow.staIndex;
      if (flow.appCentric)
        {
          SetNodeSocketType (nodeId, TcpPragueAppCentric::GetTypeId ());
        }
      else
        {
          SetNodeSocketType (nodeId, TcpDctcp::GetTypeId ());
        }

      BulkSendHelper bulkSend ("ns3::TcpSocketFactory", InetSocketAddress (p2pIfs.GetAddress (0), flow.port));
      bulkSend.SetAttribute ("SendSize", UintegerValue (1448));
      bulkSend.SetAttribute ("MaxBytes", UintegerValue (0));

      const double appStart = flow.startTime;
      const double appStop = simTime;

      if (flow.appCentric)
        {
          Simulator::Schedule (Seconds (appStart - 0.05), &ConfigureAppCentricDefaults,
                               flow.mappingType, flow.p0, flow.gamma, flow.a);
        }

      ApplicationContainer sourceApp = bulkSend.Install (staNodes.Get (flow.staIndex));
      sourceApp.Start (Seconds (appStart));
      sourceApp.Stop (Seconds (appStop));
      sourceApps.push_back (sourceApp);
    }

  std::vector<uint64_t> lastRxBytes (sinks.size (), 0);
  if (!sinks.empty ())
    {
      Simulator::Schedule (Seconds (sampleInterval), &SampleThroughput,
                           sinks, &lastRxBytes, sampleInterval, simTime);
    }

  FlowMonitorHelper flowmonHelper;
  Ptr<FlowMonitor> monitor = flowmonHelper.InstallAll ();

  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();

  monitor->CheckForLostPackets ();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmonHelper.GetClassifier ());
  const FlowMonitor::FlowStatsContainer &stats = monitor->GetFlowStats ();

  std::cout << "\n=== FlowMonitor summary ===" << std::endl;
  for (const auto &entry : stats)
    {
      const Ipv4FlowClassifier::FiveTuple tuple = classifier->FindFlow (entry.first);
      const FlowMonitor::FlowStats &st = entry.second;
      const double duration = simTime - 1.0;
      const double throughputMbps = (st.rxBytes * 8.0) / duration / 1e6;
      const double meanDelayMs = st.rxPackets > 0 ? st.delaySum.GetSeconds () * 1000.0 / st.rxPackets : 0.0;

      std::cout << "Flow " << entry.first
                << " " << tuple.sourceAddress << ":" << tuple.sourcePort
                << " -> " << tuple.destinationAddress << ":" << tuple.destinationPort
                << " rxBytes=" << st.rxBytes
                << " txBytes=" << st.txBytes
                << " lostPackets=" << st.lostPackets
                << " throughput=" << std::fixed << std::setprecision (3) << throughputMbps << " Mbps"
                << " meanDelay=" << meanDelayMs << " ms"
                << std::endl;
    }

  const QueueDisc::Stats qStats = rootQueueDisc->GetStats ();
  const uint32_t marked = qStats.GetNMarkedPackets (RedQueueDisc::UNFORCED_MARK) +
                          qStats.GetNMarkedPackets (RedQueueDisc::FORCED_MARK);
  const uint32_t dropped = qStats.GetNDroppedPackets (RedQueueDisc::UNFORCED_DROP) +
                           qStats.GetNDroppedPackets (RedQueueDisc::FORCED_DROP);
  const double markingRatio = (marked + dropped) > 0 ?
    static_cast<double> (marked) / static_cast<double> (marked + dropped) : 0.0;

  std::cout << "\n=== Queue disc summary ===" << std::endl;
  std::cout << "Marked packets: " << marked << std::endl;
  std::cout << "Dropped packets: " << dropped << std::endl;
  std::cout << "Marking ratio (marked / (marked + dropped)): " << std::fixed << std::setprecision (4)
            << markingRatio << std::endl;

  Simulator::Destroy ();
  return 0;
}