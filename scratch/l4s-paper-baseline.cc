/*
 * Paper-style equal-share baseline: N identical persistent L4S connections
 * share one bottleneck. This uses ns-3 TCP DCTCP (ECT(1)), the L4S-capable
 * transport available in this tree, rather than Linux reference TCP Prague.
 *
 * ./waf --run "l4s-paper-baseline --flows=4 --bottleneckRate=10Mbps"
 */
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/tcp-prague-app-centric.h"
#include "ns3/traffic-control-module.h"

#include <iomanip>
#include <sstream>
#include <vector>

using namespace ns3;

static void
SnapshotReceived (std::vector<Ptr<PacketSink>> sinks, std::vector<uint64_t>* bytes)
{
  for (uint32_t i = 0; i < sinks.size (); ++i)
    {
      (*bytes)[i] = sinks[i]->GetTotalRx ();
    }
}

static std::string
FlowName (uint32_t index)
{
  static const std::vector<std::string> names = {"video", "audio", "chat", "file-transfer"};
  if (index < names.size ())
    {
      return names[index];
    }
  std::ostringstream name;
  name << "connection-" << index;
  return name.str ();
}

int
main (int argc, char *argv[])
{
  uint32_t flows = 4;
  std::string bottleneckRate = "10Mbps";
  double warmup = 10.0;
  double measureSeconds = 15.0;
  int32_t modifiedFlow = -1;
  uint32_t modifiedType = 2;
  double modifiedA = 1.0;
  double modifiedP0 = 0.5;
  double modifiedGamma = 0.0;
  CommandLine cmd;
  cmd.AddValue ("flows", "Number of identical, persistent L4S connections", flows);
  cmd.AddValue ("bottleneckRate", "Shared bottleneck capacity", bottleneckRate);
  cmd.AddValue ("warmup", "Seconds excluded as TCP startup transient", warmup);
  cmd.AddValue ("measureSeconds", "Steady-state measurement interval", measureSeconds);
  cmd.AddValue ("modifiedFlow", "Flow index using modified Prague; -1 keeps all reference", modifiedFlow);
  cmd.AddValue ("modifiedType", "Modified mapping type: 1 (Eq. 6) or 2 (Eq. 7)", modifiedType);
  cmd.AddValue ("modifiedA", "Type-2 application weight A for modifiedFlow", modifiedA);
  cmd.AddValue ("modifiedP0", "Type-1 transition point p0", modifiedP0);
  cmd.AddValue ("modifiedGamma", "Type-1 blend parameter gamma in [-1, 1]", modifiedGamma);
  cmd.Parse (argc, argv);
  if (flows == 0 || warmup <= 0 || measureSeconds <= 0 || modifiedFlow >= static_cast<int32_t> (flows)
      || modifiedFlow < -1 || modifiedA <= 0)
    NS_FATAL_ERROR ("invalid flows, interval, modifiedFlow, or modifiedA value");
  if ((modifiedType != 1 && modifiedType != 2) || modifiedP0 < 0 || modifiedP0 > 1
      || modifiedGamma < -1 || modifiedGamma > 1)
    NS_FATAL_ERROR ("modifiedType must be 1 or 2; p0 must be in [0,1]; gamma must be in [-1,1]");

  Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (1000));
  Config::SetDefault ("ns3::TcpSocket::DelAckCount", UintegerValue (1));
  Config::SetDefault ("ns3::TcpSocketBase::EcnMode", StringValue ("ClassicEcn"));
  Config::SetDefault ("ns3::DualQCoupledPi2QueueDisc::Mtu", UintegerValue (1000));
  Config::SetDefault ("ns3::DualQCoupledPi2QueueDisc::QueueLimit", UintegerValue (100000));
  Config::SetDefault ("ns3::TcpPragueAppCentric::AppCentricType",
                      EnumValue (modifiedType == 1 ? TcpPragueAppCentric::TYPE1
                                                    : TcpPragueAppCentric::TYPE2));
  Config::SetDefault ("ns3::TcpPragueAppCentric::AppCentricA", DoubleValue (modifiedA));
  Config::SetDefault ("ns3::TcpPragueAppCentric::AppCentricP0", DoubleValue (modifiedP0));
  Config::SetDefault ("ns3::TcpPragueAppCentric::AppCentricGamma", DoubleValue (modifiedGamma));

  NodeContainer clients, servers, routers;
  clients.Create (flows);
  servers.Create (flows);
  routers.Create (2);
  NodeContainer all;
  all.Add (clients); all.Add (servers); all.Add (routers);
  InternetStackHelper internet;
  internet.Install (all);
  for (uint32_t i = 0; i < flows; ++i)
    {
      std::ostringstream path;
      path << "/NodeList/" << clients.Get (i)->GetId () << "/$ns3::TcpL4Protocol/SocketType";
      const bool isModified = i == static_cast<uint32_t> (modifiedFlow);
      Config::Set (path.str (), TypeIdValue (isModified ? TcpPragueAppCentric::GetTypeId ()
                                                        : TcpDctcp::GetTypeId ()));
    }

  // Four 5-ms access traversals per RTT give each flow the paper's 20-ms base RTT.
  PointToPointHelper access;
  access.SetDeviceAttribute ("DataRate", StringValue ("100Mbps"));
  access.SetChannelAttribute ("Delay", StringValue ("5ms"));
  PointToPointHelper bottleneck;
  bottleneck.SetDeviceAttribute ("DataRate", StringValue (bottleneckRate));
  bottleneck.SetChannelAttribute ("Delay", StringValue ("1ns"));

  std::vector<NetDeviceContainer> clientLinks (flows), serverLinks (flows);
  for (uint32_t i = 0; i < flows; ++i)
    {
      clientLinks[i] = access.Install (clients.Get (i), routers.Get (0));
      serverLinks[i] = access.Install (routers.Get (1), servers.Get (i));
    }
  NetDeviceContainer core = bottleneck.Install (routers);
  TrafficControlHelper dualQ;
  uint16_t handle = dualQ.SetRootQueueDisc ("ns3::DualQCoupledPi2QueueDisc");
  dualQ.AddInternalQueues (handle, 2, "ns3::DropTailQueue", "MaxSize", StringValue ("100000B"));
  dualQ.Install (core.Get (0));

  Ipv4AddressHelper ipv4;
  std::vector<Ipv4InterfaceContainer> serverIf (flows);
  for (uint32_t i = 0; i < flows; ++i)
    {
      std::ostringstream subnet;
      subnet << "10.1." << i + 1 << ".0";
      ipv4.SetBase (subnet.str ().c_str (), "255.255.255.0"); ipv4.Assign (clientLinks[i]);
      subnet.str (""); subnet.clear (); subnet << "10.2." << i + 1 << ".0";
      ipv4.SetBase (subnet.str ().c_str (), "255.255.255.0"); serverIf[i] = ipv4.Assign (serverLinks[i]);
    }
  ipv4.SetBase ("10.3.1.0", "255.255.255.0"); ipv4.Assign (core);
  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  std::vector<Ptr<PacketSink>> sinks;
  ApplicationContainer sources;
  for (uint32_t i = 0; i < flows; ++i)
    {
      uint16_t port = 52000 + i;
      PacketSinkHelper sink ("ns3::TcpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), port));
      ApplicationContainer sinkApp = sink.Install (servers.Get (i));
      sinkApp.Start (Seconds (0)); sinks.push_back (DynamicCast<PacketSink> (sinkApp.Get (0)));
      BulkSendHelper source ("ns3::TcpSocketFactory", InetSocketAddress (serverIf[i].GetAddress (1), port));
      source.SetAttribute ("MaxBytes", UintegerValue (0)); // Persistent/backlogged flow.
      sources.Add (source.Install (clients.Get (i)));
    }
  sources.Start (Seconds (0.5));
  const double finish = warmup + measureSeconds;
  sources.Stop (Seconds (finish));
  std::vector<uint64_t> warmupBytes (flows, 0);
  Simulator::Schedule (Seconds (warmup), &SnapshotReceived, sinks, &warmupBytes);
  Simulator::Stop (Seconds (finish + 1));
  Simulator::Run ();

  std::vector<double> rates (flows);
  double total = 0;
  for (uint32_t i = 0; i < flows; ++i)
    {
      rates[i] = (sinks[i]->GetTotalRx () - warmupBytes[i]) * 8.0 / measureSeconds / 1e6;
      total += rates[i];
    }
  const double mean = total / flows;
  std::cout << std::fixed << std::setprecision (3);
  std::cout << "Paper-style scenario: " << flows << " persistent L4S connections\n";
  if (modifiedFlow >= 0)
    {
      std::cout << "  " << FlowName (modifiedFlow) << " uses modified Prague Type " << modifiedType;
      if (modifiedType == 1)
        std::cout << ", p0=" << modifiedP0 << ", gamma=" << modifiedGamma
                  << " (congestion-dependent differentiation)\n";
      else
        {
          const double expectedReference = DataRate (bottleneckRate).GetBitRate () / 1e6
                                         / (flows - 1 + modifiedA);
          std::cout << ", A=" << modifiedA << " (paper reference-Prague prediction: ~"
                    << modifiedA * expectedReference << " Mb/s; references ~" << expectedReference << " Mb/s)\n";
        }
      std::cout << "  Note: this ns-3 model is DCTCP-based, so its measured allocation need not match"
                << " the paper's reference-Prague prediction.\n";
    }
  else
    {
      std::cout << "  all flows use reference L4S/DCTCP (equal-share baseline)\n";
    }
  std::cout << "Steady-state window: " << warmup << " to " << finish << " s; mean=" << mean << " Mb/s\n";
  for (uint32_t i = 0; i < flows; ++i)
    std::cout << "  " << FlowName (i) << (i == static_cast<uint32_t> (modifiedFlow) ? " [modified]" : " [reference]")
              << ": " << rates[i] << " Mb/s (deviation " << 100 * (rates[i] - mean) / mean << "%)\n";
  Simulator::Destroy ();
  return 0;
}
