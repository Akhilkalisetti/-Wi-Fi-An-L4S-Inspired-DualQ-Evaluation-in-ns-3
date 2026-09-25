/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Run with, for example:
 *   ./waf --run "dualq-multi-app-same-node --nApps=4 --l4s=1"
 *
 * All nApps TCP OnOff applications are installed on sender (node 0) and
 * started at exactly 1 s.  Each application has its own TCP connection and
 * destination port, but they share the DualQ bottleneck at node 1.
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/traffic-control-module.h"

#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("DualQMultiAppSameNode");

int
main (int argc, char *argv[])
{
  uint32_t nApps = 4;
  uint32_t packetSize = 1000;
  double appStart = 1.0;
  double appStop = 19.0;
  bool l4s = true;
  bool writePcap = false;
  bool writeFlowMonitor = false;
  std::string appRate = "5Mbps";
  std::string outputPrefix = "dualq-multi-app";

  CommandLine cmd;
  cmd.AddValue ("nApps", "Number of simultaneous applications on node 0", nApps);
  cmd.AddValue ("appRate", "Offered rate of each application", appRate);
  cmd.AddValue ("packetSize", "TCP payload size in bytes", packetSize);
  cmd.AddValue ("appStart", "Common start time for every client (seconds)", appStart);
  cmd.AddValue ("appStop", "Stop time for every client (seconds)", appStop);
  cmd.AddValue ("l4s", "Use DCTCP/ECT(1) traffic (true) or NewReno/Classic ECN traffic (false)", l4s);
  cmd.AddValue ("writePcap", "Write pcap files using outputPrefix", writePcap);
  cmd.AddValue ("writeFlowMonitor", "Write outputPrefix.flowmon.xml", writeFlowMonitor);
  cmd.AddValue ("outputPrefix", "Prefix for optional output files", outputPrefix);
  cmd.Parse (argc, argv);

  if (nApps == 0 || appStart >= appStop)
    {
      NS_FATAL_ERROR ("nApps must be greater than zero and appStart must be before appStop");
    }

  // DCTCP sends ECT(1), which this DualQ implementation classifies as L4S.
  Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (packetSize));
  Config::SetDefault ("ns3::TcpSocket::DelAckCount", UintegerValue (1));
  Config::SetDefault ("ns3::TcpSocketBase::EcnMode", StringValue ("ClassicEcn"));
  Config::SetDefault ("ns3::DualQCoupledPi2QueueDisc::Mtu", UintegerValue (packetSize));
  Config::SetDefault ("ns3::DualQCoupledPi2QueueDisc::QueueLimit", UintegerValue (100000));

  NodeContainer nodes;
  nodes.Create (3);
  Ptr<Node> sender = nodes.Get (0);
  Ptr<Node> router = nodes.Get (1);
  Ptr<Node> receiver = nodes.Get (2);

  InternetStackHelper internet;
  internet.Install (nodes);

  // Select one TCP variant for all independent sockets created on node 0.
  TypeId senderTcp = l4s ? TcpDctcp::GetTypeId () : TcpNewReno::GetTypeId ();
  Config::Set ("/NodeList/0/$ns3::TcpL4Protocol/SocketType", TypeIdValue (senderTcp));
  Config::Set ("/NodeList/2/$ns3::TcpL4Protocol/SocketType", TypeIdValue (senderTcp));

  PointToPointHelper access;
  access.SetDeviceAttribute ("DataRate", StringValue ("100Mbps"));
  access.SetChannelAttribute ("Delay", StringValue ("1ms"));
  NetDeviceContainer senderRouter = access.Install (sender, router);

  PointToPointHelper bottleneck;
  bottleneck.SetDeviceAttribute ("DataRate", StringValue ("10Mbps"));
  bottleneck.SetChannelAttribute ("Delay", StringValue ("20ms"));
  NetDeviceContainer routerReceiver = bottleneck.Install (router, receiver);

  TrafficControlHelper dualQ;
  uint16_t handle = dualQ.SetRootQueueDisc ("ns3::DualQCoupledPi2QueueDisc");
  dualQ.AddInternalQueues (handle, 2, "ns3::DropTailQueue", "MaxSize", StringValue ("100000B"));
  // Install only on node 1's egress device, so this is the shared bottleneck.
  QueueDiscContainer queueDiscs = dualQ.Install (routerReceiver.Get (0));

  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  ipv4.Assign (senderRouter);
  ipv4.SetBase ("10.1.2.0", "255.255.255.0");
  Ipv4InterfaceContainer receiverIf = ipv4.Assign (routerReceiver);
  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  std::vector<Ptr<PacketSink>> sinks;
  ApplicationContainer clientApps;
  for (uint32_t app = 0; app < nApps; ++app)
    {
      const uint16_t port = 50000 + app;
      PacketSinkHelper sinkHelper ("ns3::TcpSocketFactory",
                                   InetSocketAddress (Ipv4Address::GetAny (), port));
      ApplicationContainer sinkApp = sinkHelper.Install (receiver);
      sinkApp.Start (Seconds (0.0));
      sinkApp.Stop (Seconds (appStop + 1.0));
      sinks.push_back (DynamicCast<PacketSink> (sinkApp.Get (0)));

      OnOffHelper clientHelper ("ns3::TcpSocketFactory",
                                InetSocketAddress (receiverIf.GetAddress (1), port));
      clientHelper.SetAttribute ("PacketSize", UintegerValue (packetSize));
      clientHelper.SetAttribute ("DataRate", DataRateValue (DataRate (appRate)));
      clientHelper.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
      clientHelper.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
      clientApps.Add (clientHelper.Install (sender));
    }

  // These two calls give every source application the same simulation time.
  clientApps.Start (Seconds (appStart));
  clientApps.Stop (Seconds (appStop));

  FlowMonitorHelper flowMonitorHelper;
  Ptr<FlowMonitor> flowMonitor;
  if (writeFlowMonitor)
    {
      flowMonitor = flowMonitorHelper.InstallAll ();
    }
  if (writePcap)
    {
      access.EnablePcapAll (outputPrefix + "-access");
      bottleneck.EnablePcapAll (outputPrefix + "-bottleneck");
    }

  Simulator::Stop (Seconds (appStop + 1.0));
  Simulator::Run ();

  uint64_t totalRx = 0;
  std::cout << "Applications on node " << sender->GetId () << ": " << nApps
            << " (all started at " << appStart << " s)" << std::endl;
  for (uint32_t app = 0; app < sinks.size (); ++app)
    {
      const uint64_t bytes = sinks[app]->GetTotalRx ();
      totalRx += bytes;
      std::cout << "  app " << app << " port " << 50000 + app
                << " received " << bytes << " bytes" << std::endl;
    }
  std::cout << "Total received: " << totalRx << " bytes" << std::endl;
  std::cout << "DualQ stats at router node " << router->GetId () << ":\n"
            << queueDiscs.Get (0)->GetStats () << std::endl;

  if (writeFlowMonitor)
    {
      flowMonitor->CheckForLostPackets ();
      flowMonitor->SerializeToXmlFile (outputPrefix + ".flowmon.xml", true, true);
    }

  Simulator::Destroy ();
  return 0;
}
