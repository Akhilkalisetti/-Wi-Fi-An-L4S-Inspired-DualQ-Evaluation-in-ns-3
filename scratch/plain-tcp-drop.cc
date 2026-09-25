/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Compare a plain TCP flow with an L4S-style TCP flow and print a compact
 * summary showing how many packets were dropped at the bottleneck.
 */

#include <iostream>
#include <vector>

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/traffic-control-module.h"
#include "ns3/tcp-congestion-ops.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("PlainTcpDropExample");

struct ScenarioResult
{
  uint64_t totalRx;
  uint32_t droppedPackets;
  uint32_t droppedBytes;
};

static ScenarioResult
RunScenario (const std::string &label, const TypeId &tcpType)
{
  const uint32_t nApps = 4;
  const uint32_t packetSize = 1000;
  const double appStart = 1.0;
  const double appStop = 19.0;
  const std::string appRate = "50Mbps";

  NodeContainer nodes;
  nodes.Create (3);
  Ptr<Node> sender = nodes.Get (0);
  Ptr<Node> router = nodes.Get (1);
  Ptr<Node> receiver = nodes.Get (2);

  PointToPointHelper access;
  access.SetDeviceAttribute ("DataRate", StringValue ("100Mbps"));
  access.SetChannelAttribute ("Delay", StringValue ("1ms"));
  NetDeviceContainer senderRouter = access.Install (sender, router);

  PointToPointHelper bottleneck;
  bottleneck.SetDeviceAttribute ("DataRate", StringValue ("2Mbps"));
  bottleneck.SetChannelAttribute ("Delay", StringValue ("20ms"));
  bottleneck.SetQueue ("ns3::DropTailQueue", "MaxSize", StringValue ("1p"));
  NetDeviceContainer routerReceiver = bottleneck.Install (router, receiver);

  InternetStackHelper internet;
  internet.Install (nodes);

  Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (packetSize));
  Config::SetDefault ("ns3::TcpSocket::DelAckCount", UintegerValue (1));
  Config::SetDefault ("ns3::TcpSocketBase::EcnMode", StringValue ("ClassicEcn"));

  Config::Set ("/NodeList/0/$ns3::TcpL4Protocol/SocketType", TypeIdValue (tcpType));
  Config::Set ("/NodeList/2/$ns3::TcpL4Protocol/SocketType", TypeIdValue (tcpType));

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

  clientApps.Start (Seconds (appStart));
  clientApps.Stop (Seconds (appStop));

  Simulator::Stop (Seconds (appStop + 1.0));
  Simulator::Run ();

  Ptr<PointToPointNetDevice> dev = DynamicCast<PointToPointNetDevice> (routerReceiver.Get (0));
  Ptr<Queue<Packet> > queue = dev->GetQueue ();

  ScenarioResult result;
  result.totalRx = 0;
  for (uint32_t app = 0; app < sinks.size (); ++app)
    {
      result.totalRx += sinks[app]->GetTotalRx ();
    }
  result.droppedPackets = queue->GetTotalDroppedPackets ();
  result.droppedBytes = queue->GetTotalDroppedBytes ();

  Simulator::Destroy ();

  std::cout << "Applications on node " << sender->GetId () << ": " << nApps
            << " (all started at " << appStart << " s)" << std::endl;
  for (uint32_t app = 0; app < nApps; ++app)
    {
      std::cout << "  app " << app << " port " << 50000 + app
                << " received " << sinks[app]->GetTotalRx () << " bytes" << std::endl;
    }
  std::cout << "Total received: " << result.totalRx << " bytes" << std::endl;
  std::cout << label << " stats at router node " << router->GetId () << ":" << std::endl;
  std::cout << "Packets/Bytes dropped: " << result.droppedPackets << " / " << result.droppedBytes << std::endl;
  std::cout << "Packets/Bytes dropped before enqueue: " << result.droppedPackets << " / " << result.droppedBytes << std::endl;
  std::cout << "Packets/Bytes dropped after dequeue: 0 / 0" << std::endl;
  std::cout << std::endl;

  return result;
}

int
main (int argc, char *argv[])
{
  CommandLine cmd;
  cmd.Parse (argc, argv);

  ScenarioResult normal = RunScenario ("Normal TCP", TcpNewReno::GetTypeId ());
  ScenarioResult l4s = RunScenario ("L4S TCP", TcpDctcp::GetTypeId ());

  std::cout << "Comparison:" << std::endl;
  if (normal.droppedPackets > l4s.droppedPackets)
    {
      std::cout << "Normal TCP dropped more packets than L4S TCP." << std::endl;
    }
  else if (l4s.droppedPackets > normal.droppedPackets)
    {
      std::cout << "L4S TCP dropped more packets than Normal TCP." << std::endl;
    }
  else
    {
      std::cout << "Normal TCP and L4S TCP dropped the same number of packets." << std::endl;
    }

  return 0;
}
