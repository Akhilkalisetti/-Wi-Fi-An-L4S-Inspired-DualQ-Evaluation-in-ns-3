
/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/tcp-prague-app-centric.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("NodPragueExample");

int
main (int argc, char *argv[])
{
	uint32_t mappingType = 2;
	double p0 = 0.5;
	double gamma = 0.0;
	double a = 1.0;
	double stopTime = 20.0;

	CommandLine cmd;
	cmd.AddValue ("mappingType", "1 for Type 1, 2 for Type 2", mappingType);
	cmd.AddValue ("p0", "Type 1 midpoint", p0);
	cmd.AddValue ("gamma", "Type 1 blend parameter in [-1, 1]", gamma);
	cmd.AddValue ("a", "Type 2 scale factor", a);
	cmd.AddValue ("stopTime", "Simulation stop time in seconds", stopTime);
	cmd.Parse (argc, argv);

	std::cout << "Running nod_prague with mappingType=" << mappingType
	          << ", p0=" << p0
	          << ", gamma=" << gamma
	          << ", a=" << a
	          << ", stopTime=" << stopTime << std::endl;

	NodeContainer nodes;
	nodes.Create (2);

	PointToPointHelper p2p;
	p2p.SetDeviceAttribute ("DataRate", StringValue ("10Mbps"));
	p2p.SetChannelAttribute ("Delay", StringValue ("20ms"));

	NetDeviceContainer devices = p2p.Install (nodes);

	InternetStackHelper internet;
	internet.Install (nodes);

	Ipv4AddressHelper ipv4;
	ipv4.SetBase ("10.1.1.0", "255.255.255.0");
	Ipv4InterfaceContainer interfaces = ipv4.Assign (devices);

	uint16_t port = 50000;
	Address sinkAddress (InetSocketAddress (Ipv4Address::GetAny (), port));
	PacketSinkHelper sinkHelper ("ns3::TcpSocketFactory", sinkAddress);
	ApplicationContainer sinkApp = sinkHelper.Install (nodes.Get (1));
	sinkApp.Start (Seconds (0.0));
	sinkApp.Stop (Seconds (stopTime));

	Config::SetDefault ("ns3::TcpL4Protocol::SocketType",
											TypeIdValue (TcpPragueAppCentric::GetTypeId ()));
	Config::SetDefault ("ns3::TcpPragueAppCentric::AppCentricType",
											EnumValue (mappingType == 1 ? TcpPragueAppCentric::TYPE1
																									: TcpPragueAppCentric::TYPE2));
	Config::SetDefault ("ns3::TcpPragueAppCentric::AppCentricP0", DoubleValue (p0));
	Config::SetDefault ("ns3::TcpPragueAppCentric::AppCentricGamma", DoubleValue (gamma));
	Config::SetDefault ("ns3::TcpPragueAppCentric::AppCentricA", DoubleValue (a));

	BulkSendHelper bulkSend ("ns3::TcpSocketFactory",
													 InetSocketAddress (interfaces.GetAddress (1), port));
	bulkSend.SetAttribute ("MaxBytes", UintegerValue (0));
	bulkSend.SetAttribute ("SendSize", UintegerValue (1448));

	ApplicationContainer sourceApp = bulkSend.Install (nodes.Get (0));
	sourceApp.Start (Seconds (1.0));
	sourceApp.Stop (Seconds (stopTime));

	Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

	Simulator::Stop (Seconds (stopTime));
	Simulator::Run ();
	std::cout << "nod_prague simulation completed" << std::endl;
	Simulator::Destroy ();
	return 0;
}
