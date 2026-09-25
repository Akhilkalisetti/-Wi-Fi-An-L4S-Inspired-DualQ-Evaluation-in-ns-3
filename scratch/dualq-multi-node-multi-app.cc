/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Each endpoint node runs nApps simultaneous TCP applications to its paired
 * endpoint.  Traffic in both directions crosses the DualQ bottleneck.
 *
 * Example:
 *   ./waf --run "dualq-multi-node-multi-app --nPairs=3 --nApps=4 --l4s=1"
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/traffic-control-module.h"

#include <sstream>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("DualQMultiNodeMultiApp");

int
main (int argc, char *argv[])
{
  uint32_t nPairs = 3;
  uint32_t nApps = 4;
  uint32_t packetSize = 1000;
  double appStart = 1.0;
  double appStop = 19.0;
  bool l4s = true;
  std::string appRate = "3Mbps";

  CommandLine cmd;
  cmd.AddValue ("nPairs", "Number of communicating endpoint pairs", nPairs);
  cmd.AddValue ("nApps", "Simultaneous applications installed on every endpoint", nApps);
  cmd.AddValue ("appRate", "Offered rate of every application", appRate);
  cmd.AddValue ("packetSize", "TCP payload size in bytes", packetSize);
  cmd.AddValue ("appStart", "Common start time for all clients (seconds)", appStart);
  cmd.AddValue ("appStop", "Stop time for all clients (seconds)", appStop);
  cmd.AddValue ("l4s", "Use DCTCP/ECT(1) traffic (true) or NewReno/Classic ECN traffic (false)", l4s);
  cmd.Parse (argc, argv);

  if (nPairs == 0 || nApps == 0 || appStart >= appStop)
    {
      NS_FATAL_ERROR ("nPairs and nApps must be greater than zero and appStart must be before appStop");
    }

  Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (packetSize));
  Config::SetDefault ("ns3::TcpSocket::DelAckCount", UintegerValue (1));
  Config::SetDefault ("ns3::TcpSocketBase::EcnMode", StringValue ("ClassicEcn"));
  Config::SetDefault ("ns3::DualQCoupledPi2QueueDisc::Mtu", UintegerValue (packetSize));
  Config::SetDefault ("ns3::DualQCoupledPi2QueueDisc::QueueLimit", UintegerValue (100000));

  NodeContainer leftHosts;
  NodeContainer rightHosts;
  leftHosts.Create (nPairs);
  rightHosts.Create (nPairs);
  NodeContainer routers;
  routers.Create (2);

  NodeContainer allNodes;
  allNodes.Add (leftHosts);
  allNodes.Add (rightHosts);
  allNodes.Add (routers);
  InternetStackHelper internet;
  internet.Install (allNodes);

  TypeId tcpType = l4s ? TcpDctcp::GetTypeId () : TcpNewReno::GetTypeId ();
  for (uint32_t pair = 0; pair < nPairs; ++pair)
    {
      std::ostringstream leftPath;
      leftPath << "/NodeList/" << leftHosts.Get (pair)->GetId ()
               << "/$ns3::TcpL4Protocol/SocketType";
      Config::Set (leftPath.str (), TypeIdValue (tcpType));
      std::ostringstream rightPath;
      rightPath << "/NodeList/" << rightHosts.Get (pair)->GetId ()
                << "/$ns3::TcpL4Protocol/SocketType";
      Config::Set (rightPath.str (), TypeIdValue (tcpType));
    }

  PointToPointHelper access;
  access.SetDeviceAttribute ("DataRate", StringValue ("100Mbps"));
  access.SetChannelAttribute ("Delay", StringValue ("1ms"));
  PointToPointHelper bottleneck;
  bottleneck.SetDeviceAttribute ("DataRate", StringValue ("10Mbps"));
  bottleneck.SetChannelAttribute ("Delay", StringValue ("20ms"));

  std::vector<NetDeviceContainer> leftLinks (nPairs);
  std::vector<NetDeviceContainer> rightLinks (nPairs);
  for (uint32_t pair = 0; pair < nPairs; ++pair)
    {
      leftLinks[pair] = access.Install (leftHosts.Get (pair), routers.Get (0));
      rightLinks[pair] = access.Install (routers.Get (1), rightHosts.Get (pair));
    }
  NetDeviceContainer routerLink = bottleneck.Install (routers.Get (0), routers.Get (1));

  TrafficControlHelper dualQ;
  uint16_t handle = dualQ.SetRootQueueDisc ("ns3::DualQCoupledPi2QueueDisc");
  dualQ.AddInternalQueues (handle, 2, "ns3::DropTailQueue", "MaxSize", StringValue ("100000B"));
  // One queue disc per direction of the shared inter-router bottleneck.
  QueueDiscContainer leftToRightQueue = dualQ.Install (routerLink.Get (0));
  QueueDiscContainer rightToLeftQueue = dualQ.Install (routerLink.Get (1));

  Ipv4AddressHelper ipv4;
  std::vector<Ipv4InterfaceContainer> leftIf (nPairs);
  std::vector<Ipv4InterfaceContainer> rightIf (nPairs);
  for (uint32_t pair = 0; pair < nPairs; ++pair)
    {
      std::ostringstream subnet;
      subnet << "10.1." << pair + 1 << ".0";
      ipv4.SetBase (subnet.str ().c_str (), "255.255.255.0");
      leftIf[pair] = ipv4.Assign (leftLinks[pair]);

      subnet.str ("");
      subnet.clear ();
      subnet << "10.2." << pair + 1 << ".0";
      ipv4.SetBase (subnet.str ().c_str (), "255.255.255.0");
      rightIf[pair] = ipv4.Assign (rightLinks[pair]);
    }
  ipv4.SetBase ("10.3.1.0", "255.255.255.0");
  ipv4.Assign (routerLink);
  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  std::vector<std::vector<Ptr<PacketSink>>> sinks (2 * nPairs);
  ApplicationContainer clients;
  for (uint32_t pair = 0; pair < nPairs; ++pair)
    {
      Ptr<Node> endpoints[2] = {leftHosts.Get (pair), rightHosts.Get (pair)};
      Ipv4Address destinations[2] = {rightIf[pair].GetAddress (1), leftIf[pair].GetAddress (0)};
      for (uint32_t endpoint = 0; endpoint < 2; ++endpoint)
        {
          for (uint32_t app = 0; app < nApps; ++app)
            {
              const uint16_t port = 50000 + app;
              // The paired endpoint receives one sink for every local client app.
              Ptr<Node> destinationNode = endpoints[1 - endpoint];
              PacketSinkHelper sinkHelper ("ns3::TcpSocketFactory",
                                           InetSocketAddress (Ipv4Address::GetAny (), port));
              ApplicationContainer sinkApp = sinkHelper.Install (destinationNode);
              sinkApp.Start (Seconds (0.0));
              sinkApp.Stop (Seconds (appStop + 1.0));
              sinks[2 * pair + (1 - endpoint)].push_back (
                DynamicCast<PacketSink> (sinkApp.Get (0)));

              OnOffHelper clientHelper ("ns3::TcpSocketFactory",
                                        InetSocketAddress (destinations[endpoint], port));
              clientHelper.SetAttribute ("PacketSize", UintegerValue (packetSize));
              clientHelper.SetAttribute ("DataRate", DataRateValue (DataRate (appRate)));
              clientHelper.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
              clientHelper.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
              clients.Add (clientHelper.Install (endpoints[endpoint]));
            }
        }
    }

  // This one schedule applies to every application on every endpoint node.
  clients.Start (Seconds (appStart));
  clients.Stop (Seconds (appStop));

  Simulator::Stop (Seconds (appStop + 1.0));
  Simulator::Run ();

  std::cout << "Endpoint nodes: " << 2 * nPairs << ", applications per endpoint: " << nApps
            << ", common start time: " << appStart << " s" << std::endl;
  for (uint32_t pair = 0; pair < nPairs; ++pair)
    {
      for (uint32_t endpoint = 0; endpoint < 2; ++endpoint)
        {
          const uint32_t index = 2 * pair + endpoint;
          uint64_t received = 0;
          for (const auto& sink : sinks[index])
            {
              received += sink->GetTotalRx ();
            }
          std::cout << "  node " << (endpoint == 0 ? leftHosts.Get (pair) : rightHosts.Get (pair))->GetId ()
                    << " received " << received << " bytes across " << nApps << " applications" << std::endl;
        }
    }
  std::cout << "DualQ left-to-right stats:\n" << leftToRightQueue.Get (0)->GetStats () << std::endl;
  std::cout << "DualQ right-to-left stats:\n" << rightToLeftQueue.Get (0)->GetStats () << std::endl;

  Simulator::Destroy ();
  return 0;
}
