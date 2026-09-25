/*
 * Mixed video-call traffic over one DualQ L4S bottleneck.
 *
 * The payload contents are synthetic; congestion control depends on packet size,
 * timing and offered rate rather than image/audio/video byte values.
 * Run: ./waf --run "l4s-video-call-traffic"
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

struct TrafficFlow
{
  std::string name;
  uint32_t packetSize;
  std::string offeredRate;
};

int
main (int argc, char *argv[])
{
  std::string bottleneckRate = "10Mbps";
  double start = 1.0;
  double stop = 31.0;
  CommandLine cmd;
  cmd.AddValue ("bottleneckRate", "Shared L4S bottleneck rate", bottleneckRate);
  cmd.AddValue ("stop", "Application stop time in seconds", stop);
  cmd.Parse (argc, argv);

  // Video and image transfer are deliberately above their likely fair share.
  // Audio and chat are rate-limited, so they take only what they need.
  const std::vector<TrafficFlow> flows = {
      {"video", 1200, "8Mbps"},       // Large, continuous video frames
      {"audio", 160, "128kbps"},       // Small 20-ms-equivalent audio packets
      {"chat", 200, "64kbps"},         // Small message/control traffic
      {"image-file", 1200, "8Mbps"},   // Background image/file transfer
  };

  Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (1000));
  Config::SetDefault ("ns3::TcpSocket::DelAckCount", UintegerValue (1));
  Config::SetDefault ("ns3::TcpSocketBase::EcnMode", StringValue ("ClassicEcn"));
  Config::SetDefault ("ns3::DualQCoupledPi2QueueDisc::Mtu", UintegerValue (1200));
  Config::SetDefault ("ns3::DualQCoupledPi2QueueDisc::QueueLimit", UintegerValue (100000));

  NodeContainer nodes;
  nodes.Create (3); // sender, DualQ router, receiver
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
  dualQ.Install (right.Get (0));

  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  ipv4.Assign (left);
  ipv4.SetBase ("10.1.2.0", "255.255.255.0");
  Ipv4InterfaceContainer destinationIf = ipv4.Assign (right);
  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  std::vector<Ptr<PacketSink>> sinks;
  ApplicationContainer sources;
  for (uint32_t i = 0; i < flows.size (); ++i)
    {
      uint16_t port = 51000 + i;
      PacketSinkHelper sink ("ns3::TcpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), port));
      ApplicationContainer sinkApp = sink.Install (nodes.Get (2));
      sinkApp.Start (Seconds (0));
      sinkApp.Stop (Seconds (stop + 1));
      sinks.push_back (DynamicCast<PacketSink> (sinkApp.Get (0)));

      OnOffHelper source ("ns3::TcpSocketFactory", InetSocketAddress (destinationIf.GetAddress (1), port));
      source.SetAttribute ("PacketSize", UintegerValue (flows[i].packetSize));
      source.SetAttribute ("DataRate", DataRateValue (DataRate (flows[i].offeredRate)));
      source.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
      source.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
      sources.Add (source.Install (nodes.Get (0)));
    }
  sources.Start (Seconds (start));
  sources.Stop (Seconds (stop));
  Simulator::Stop (Seconds (stop + 1));
  Simulator::Run ();

  std::cout << std::fixed << std::setprecision (3);
  std::cout << "Mixed L4S video-call traffic; bottleneck=" << bottleneckRate << '\n';
  for (uint32_t i = 0; i < flows.size (); ++i)
    {
      double mbps = sinks[i]->GetTotalRx () * 8.0 / (stop - start) / 1e6;
      std::cout << "  " << flows[i].name << " (offered " << flows[i].offeredRate
                << "): " << mbps << " Mb/s received\n";
    }
  Simulator::Destroy ();
  return 0;
}
