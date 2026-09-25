/* Demonstrate default L4S sharing between otherwise identical connections.
 * Run: ./waf --run "l4s-equal-connection-share --connections=4"
 */
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/traffic-control-module.h"

#include <iomanip>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("L4sEqualConnectionShare");

int
main (int argc, char *argv[])
{
  uint32_t connections = 4;
  std::string bottleneckRate = "10Mbps";
  double start = 1.0;
  double stop = 21.0;
  CommandLine cmd;
  cmd.AddValue ("connections", "Number of identical L4S TCP connections", connections);
  cmd.AddValue ("bottleneckRate", "Shared bottleneck data rate", bottleneckRate);
  cmd.AddValue ("start", "Common source start time in seconds", start);
  cmd.AddValue ("stop", "Common source stop time in seconds", stop);
  cmd.Parse (argc, argv);
  if (connections == 0 || stop <= start)
    {
      NS_FATAL_ERROR ("connections must be positive and stop must exceed start");
    }

  // DCTCP emits ECT(1), which this repository's DualQ classifies as L4S.
  Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (1000));
  Config::SetDefault ("ns3::TcpSocket::DelAckCount", UintegerValue (1));
  Config::SetDefault ("ns3::TcpSocketBase::EcnMode", StringValue ("ClassicEcn"));
  Config::SetDefault ("ns3::DualQCoupledPi2QueueDisc::Mtu", UintegerValue (1000));
  Config::SetDefault ("ns3::DualQCoupledPi2QueueDisc::QueueLimit", UintegerValue (100000));

  NodeContainer nodes;
  nodes.Create (3);
  InternetStackHelper internet;
  internet.Install (nodes);
  Config::Set ("/NodeList/0/$ns3::TcpL4Protocol/SocketType",
               TypeIdValue (TcpDctcp::GetTypeId ()));

  PointToPointHelper access;
  access.SetDeviceAttribute ("DataRate", StringValue ("100Mbps"));
  access.SetChannelAttribute ("Delay", StringValue ("1ms"));
  NetDeviceContainer left = access.Install (nodes.Get (0), nodes.Get (1));
  PointToPointHelper bottleneck;
  bottleneck.SetDeviceAttribute ("DataRate", StringValue (bottleneckRate));
  bottleneck.SetChannelAttribute ("Delay", StringValue ("20ms"));
  NetDeviceContainer right = bottleneck.Install (nodes.Get (1), nodes.Get (2));

  TrafficControlHelper dualQ;
  uint16_t handle = dualQ.SetRootQueueDisc ("ns3::DualQCoupledPi2QueueDisc");
  dualQ.AddInternalQueues (handle, 2, "ns3::DropTailQueue", "MaxSize", StringValue ("100000B"));
  dualQ.Install (right.Get (0)); // Router egress: the shared bottleneck.

  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  ipv4.Assign (left);
  ipv4.SetBase ("10.1.2.0", "255.255.255.0");
  Ipv4InterfaceContainer destinationIf = ipv4.Assign (right);
  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  std::vector<Ptr<PacketSink>> sinks;
  ApplicationContainer sources;
  for (uint32_t i = 0; i < connections; ++i)
    {
      uint16_t port = 50000 + i;
      PacketSinkHelper sink ("ns3::TcpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), port));
      ApplicationContainer sinkApp = sink.Install (nodes.Get (2));
      sinkApp.Start (Seconds (0));
      sinkApp.Stop (Seconds (stop + 1));
      sinks.push_back (DynamicCast<PacketSink> (sinkApp.Get (0)));
      BulkSendHelper source ("ns3::TcpSocketFactory", InetSocketAddress (destinationIf.GetAddress (1), port));
      source.SetAttribute ("MaxBytes", UintegerValue (0)); // Infinite data: saturates bottleneck.
      sources.Add (source.Install (nodes.Get (0)));
    }
  sources.Start (Seconds (start));
  sources.Stop (Seconds (stop));
  Simulator::Stop (Seconds (stop + 1));
  Simulator::Run ();

  const double duration = stop - start;
  uint64_t totalBytes = 0;
  for (const auto& sink : sinks)
    totalBytes += sink->GetTotalRx ();
  const double meanMbps = totalBytes * 8.0 / duration / 1e6 / connections;
  std::cout << std::fixed << std::setprecision (3);
  std::cout << "Identical L4S connections: " << connections << '\n';
  std::cout << "Mean per-connection throughput: " << meanMbps << " Mb/s\n";
  std::cout << "Connection results (equal shares produce deviations near 0%):\n";
  for (uint32_t i = 0; i < connections; ++i)
    {
      double mbps = sinks[i]->GetTotalRx () * 8.0 / duration / 1e6;
      double deviation = 100.0 * (mbps - meanMbps) / meanMbps;
      std::cout << "  connection " << i << ": " << mbps << " Mb/s"
                << "  (deviation from mean: " << deviation << "%)\n";
    }
  Simulator::Destroy ();
  return 0;
}
