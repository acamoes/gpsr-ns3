/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 IITP RAS
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * This is an example script for AODV manet routing protocol. 
 *
 * Authors: Pavel Boyko <boyko@iitp.ru>
 */

#include "ns3/netanim-module.h"
#include "ns3/aodv-module.h"
#include "ns3/aodv-helper.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/wifi-module.h"
#include "ns3/v4ping-helper.h"
#include "ns3/applications-module.h"
#include <iostream>
#include <cmath>
#include "ns3/flow-monitor-module.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/propagation-delay-model.h"
#include "ns3/propagation-loss-model.h"


using namespace ns3;

const uint32_t Napps=5;

int clientNode[Napps];
int serverNode[Napps];
int realclientNode[Napps];
int realserverNode[Napps];
int datatx_count = 0;
int datarx_count = 0;
int rreq_count = 0;
int rrep_count = 0;
int rrepack_count = 0;
int rerr_count = 0;
int rreq_counttx = 0;
int rrep_counttx = 0;
int rrepack_counttx = 0;
int rerr_counttx = 0;
int arp_counttx = 0;
int arp_count = 0;
int typeheadercounttx = 0;
int udp_count = 0;
int udp_counttx = 0;
int typeheadercount = 0;
int wifitrace = 0;
int rxtrace = 0;
int totaldatatx = 0;
int totaldatarx = 0;
bool first_packetRcvd[271];
uint32_t packetsRcvd[271];
uint32_t packetsSent[271];

//Z=0 é onde guarda o contador de pacotes, Z=1 guarda valor primeiro send or receive
uint64_t receivePackets[271][271][2];
uint64_t sendPackets[271][271][2];

uint32_t alt = 0;
double last_node_position[271];
double last_node_vel[271];
int count_rxDrop;
int count_txDrop;

/// Boolean representing the moment when a vehicle has just crashed (stopped)
bool just_crashed[271];
// Workaround CourseChange (2 changes per second)
uint32_t changes_per_sec[271];

double last_sec[271];
double time_of_crash[271];
double first_change[271];
double second_change[271];

class AodvExample
{
public:
  AodvExample ();
  /// Configure script parameters, \return true on successful configuration
  bool Configure (int argc, char **argv);
  /// Run simulation
  void Run ();
  /// Report results
  void Report (std::ostream & os);
  void SetupPacketSend (uint32_t nodeid);
  void DevTxTrace (std::string context, Ptr<const Packet> p);
  void WifiMacRxTrace (std::string context, Ptr<const Packet> p);
  void RxTrace (std::string context, Ptr<const Packet> packet, Ptr<Ipv4> ipv4, uint32_t i);




private:
  ///\name parameters
  //\{
  /// Number of nodes
  uint32_t size;

  /// Simulation time, seconds
  double totalTime;
  /// Write per-device PCAP traces if true
  bool pcap;
  /// Print routes if true
  bool printRoutes;
  /// Packet size (Bytes)
  uint32_t pkt_size;
  /// Inter-Packet Interval
  DataRate data_rate;
  /// Communication mode (available options: "v2v" or "v2i")
  string comm_mode;
  /// Traffic density (available options: "low" or "high")
  string traffic_density;
  /// Test number (available options: "1.1", "1.2", "2.1", "2.2" or "3.1")
  string test_id;
  /// Simulation run ID
  uint32_t runID;
  string runID_str;
  //\}

  /// Wifi channel settings
  double obu_tx_power;
  double obu_tx_gain;
  double obu_rx_gain;
  double obu_antenna_height;
  double rsu_tx_power;
  double rsu_tx_gain;
  double rsu_rx_gain;
  double rsu_antenna_height;
  string rsu_str;

  // Log Files
  std::string metrics;
  std::ofstream os_metrics;

  std::string txlogFile; // Frame Transmission Log File
  std::ofstream os_tlf;
  std::string rxlogFile; // Frame Reception Log File
  std::ofstream os_rlf;

  ///\name network
  //\{
  NodeContainer nodes;
  NodeContainer rsu_nodes;
  NodeContainer obu_nodes;

  NetDeviceContainer devices;
  NetDeviceContainer rsu_devices;
  NetDeviceContainer obu_devices;
  NetDeviceContainer crashed_device;
  NetDeviceContainer static_devices;
  Ipv4InterfaceContainer rsu_interfaces;
  Ipv4InterfaceContainer obu_interfaces;
  Ipv4InterfaceContainer interfaces;
  //\}

private:
  void CreateNodes ();
  void CreateDevices ();
  void InstallInternetStack ();
  void InstallApplications ();
};


static void
MacTxDrop (std::string context, Ptr<const Packet> packet)
{
   size_t first_delimit = context.find_first_of ('/', 1);
   size_t second_delimit = context.find_first_of ('/', first_delimit + 1);
   std::string nodeid_str = context.substr (first_delimit + 1, second_delimit - 1 - first_delimit);
   stringstream ss (nodeid_str);
   uint32_t nodeid;
   ss >> nodeid;

   count_txDrop+=1;
}

static void
MacRxDrop (std::string context, Ptr<const Packet> p)
{
  size_t first_delimit = context.find_first_of ('/', 1);
  size_t second_delimit = context.find_first_of ('/', first_delimit + 1);
  std::string nodeid_str = context.substr (first_delimit + 1, second_delimit - 1 - first_delimit);
  stringstream ss (nodeid_str);
  uint32_t nodeid;
  ss >> nodeid;

  count_rxDrop+=1;

  //std::cout << *p << std::endl;
}

void
AodvExample::WifiMacRxTrace (std::string context, Ptr<const Packet> p)
{
  size_t first_delimit = context.find_first_of ('/', 1);
  size_t second_delimit = context.find_first_of ('/', first_delimit + 1);
  std::string nodeid_str = context.substr (first_delimit + 1, second_delimit - 1 - first_delimit);
  stringstream ss (nodeid_str);
  uint32_t nodeid;
  ss >> nodeid;

  datarx_count +=1;

  NS_LOG_UNCOND("STRING WIFI " << context);

  Ptr<Node> network_node = nodes.Get(nodeid);
  Ptr<MobilityModel> node_mob = network_node->GetObject<MobilityModel>();

  Ptr<Node> crashed_node = nodes.Get(0);
  Ptr<MobilityModel> crashed_node_mob = crashed_node->GetObject<MobilityModel>();

  Ptr<Packet> copy = p->Copy ();
  PacketMetadata::ItemIterator i = copy->BeginItem ();
  while (i.HasNext())
      {


        struct PacketMetadata::Item item = i.Next();
        if (item.type == PacketMetadata::Item::HEADER)
          {
      	  //TRACE
      	  os_rlf << item.tid.GetName() << "\t";

            if (item.tid.GetName() == "ns3::Ipv4Header")
              {

              }

            if(item.tid.GetName() == "ns3::aodv::RreqHeader")
            {
          	  rreq_count = rreq_count + 1;
            }
            if(item.tid.GetName() == "ns3::ArpHeader")
            {
          	  arp_count +=1;
            }
            if(item.tid.GetName() == "ns3::UdpHeader")
            {
          	  udp_count += 1;
            }
            if(item.tid.GetName() == "ns3::aodv::TypeHeader")
            {
          	  typeheadercount += 1;
            }
            if(item.tid.GetName() == "ns3::aodv::RerrHeader")
            {
          	  rerr_count += 1;
            }
            if(item.tid.GetName() == "ns3::aodv::RrepHeader")
            {
          	  rrep_count += 1;
            }

            if(item.tid.GetName() == "ns3::aodv::RrepAckHeader")
            {
          	  rrepack_count +=1;
            }
            if (item.tid.GetName() == "ns3::SeqTsHeader")
            {
          	  Ipv4Header iph;
          	  copy->PeekHeader (iph);
          	  Ipv4Address src = iph.GetSource();
          	  //Ipv4Address dst = iph.GetDestination();
    //      	  uint8_t ttl = iph.GetTtl ();
  //        	  uint8_t hops = 64 - ttl;
      //    	  uint16_t packet_id = iph.GetIdentification ();

          	  totaldatarx +=1;

          	  uint32_t i = 0;
          	  uint32_t aux = 0;
          	  while(i<interfaces.GetN())
          	  {
          		  if(interfaces.GetAddress(i) == src)
          		  {
          			  aux = i;
          			  break;
          		  }

          		  i++;
          	  }

          	  if(src!=Ipv4Address("10.255.255.255"))
          	  {
          		  if(receivePackets[nodeid][aux][1]==0)
          		  {
          			  receivePackets[nodeid][aux][0] = receivePackets[nodeid][aux][0] + 1;
          			  receivePackets[nodeid][aux][1] = Simulator::Now().GetMicroSeconds();
          			  NS_LOG_UNCOND("RECV0 " << nodeid << " " << aux << " " << receivePackets[nodeid][aux][1]);
          		  }
          		  else
          		  {
          			  receivePackets[nodeid][aux][0] = receivePackets[nodeid][aux][0] + 1;
          			  NS_LOG_UNCOND("BUBU1 " << nodeid << " " << aux);
          		  }

          		  aux = 0;
          	  }
            }
          }
        }

  os_rlf << "\n";

}


void
AodvExample::RxTrace (std::string context, Ptr<const Packet> packet, Ptr<Ipv4> ipv4, uint32_t i)
{
  size_t first_delimit = context.find_first_of ('/', 1);
  size_t second_delimit = context.find_first_of ('/', first_delimit + 1);
  std::string nodeid_str = context.substr (first_delimit + 1, second_delimit - 1 - first_delimit);
  stringstream ss (nodeid_str);
  uint32_t nodeid;
  ss >> nodeid;

  Ptr<Node> network_node = nodes.Get(nodeid);
  Ptr<MobilityModel> node_mob = network_node->GetObject<MobilityModel>();

  Ptr<Node> crashed_node = nodes.Get(0);
  Ptr<MobilityModel> crashed_node_mob = crashed_node->GetObject<MobilityModel>();

  Ptr<Packet> copy = packet->Copy ();
  Ipv4Header iph;
  copy->PeekHeader (iph);
 // uint8_t ttl = iph.GetTtl ();
//  uint8_t hops = 64 - ttl;
  //uint16_t packet_id = iph.GetIdentification ();
  //Ipv4Address src = iph.GetSource ();


  rxtrace += 1;

}

void
AodvExample::DevTxTrace (std::string context, Ptr<const Packet> p)
{

  size_t first_delimit = context.find_first_of ('/', 1);
  size_t second_delimit = context.find_first_of ('/', first_delimit + 1);
  std::string nodeid_str = context.substr (first_delimit + 1, second_delimit - 1 - first_delimit);
  stringstream ss (nodeid_str);
  uint32_t nodeid;
  ss >> nodeid;

  datatx_count = datatx_count + 1;

  NS_LOG_UNCOND("CONTEXT DEVTXTRACE " << context << " nodeid " << nodeid);

  Ptr<Node> network_node = nodes.Get(nodeid);
  Ptr<MobilityModel> node_mob = network_node->GetObject<MobilityModel>();

  Ptr<Packet> copy = p->Copy ();
  LlcSnapHeader lsh;
  copy->RemoveHeader (lsh);
  PacketMetadata::ItemIterator i = copy->BeginItem ();
  while (i.HasNext())
    {


      struct PacketMetadata::Item item = i.Next();
      if (item.type == PacketMetadata::Item::HEADER)
        {
    	  //TRACE
    	  os_tlf << item.tid.GetName() << "\t";

          if (item.tid.GetName() == "ns3::Ipv4Header")
            {

            }

          if(item.tid.GetName() == "ns3::aodv::RreqHeader")
          {
        	  rreq_counttx = rreq_counttx + 1;
          }
          if(item.tid.GetName() == "ns3::ArpHeader")
          {
        	  arp_counttx +=1;
          }
          if(item.tid.GetName() == "ns3::UdpHeader")
          {
        	  udp_counttx += 1;
          }
          if(item.tid.GetName() == "ns3::aodv::TypeHeader")
          {
        	  typeheadercounttx += 1;
          }
          if(item.tid.GetName() == "ns3::aodv::RerrHeader")
          {
        	  rerr_counttx += 1;
          }
          if(item.tid.GetName() == "ns3::aodv::RrepHeader")
          {
        	  rrep_counttx += 1;
          }

          if(item.tid.GetName() == "ns3::aodv::RrepAckHeader")
          {
        	  rrepack_counttx +=1;
          }
          if (item.tid.GetName() == "ns3::SeqTsHeader")
          {
        	  Ipv4Header iph;
        	  copy->PeekHeader (iph);
        	  Ipv4Address src = iph.GetSource();
        	  Ipv4Address dst = iph.GetDestination();
  //      	  uint8_t ttl = iph.GetTtl ();
//        	  uint8_t hops = 64 - ttl;
    //    	  uint16_t packet_id = iph.GetIdentification ();

        	  packetsSent[nodeid] = packetsSent[nodeid] + 1;

        	  totaldatatx +=1;

        	  uint32_t i = 0;
        	  uint32_t aux = 0;
        	  while(i<interfaces.GetN())
        	  {
        		  if(interfaces.GetAddress(i) == dst)
        		  {
        			  aux = i;
        			  break;
        		  }

        		  i++;
        	  }

        	  NS_LOG_UNCOND("[DEVTXTRACE] src " << src << " node id " << nodeid << " dst " << dst << " " << aux
        			  << " Time " << Simulator::Now());

        	  if(dst!=Ipv4Address("10.255.255.255"))
        	  {
        		  if(sendPackets[nodeid][aux][1] == 0)
        		  {
        			  sendPackets[nodeid][aux][0] = sendPackets[nodeid][aux][0] + 1;
        			  sendPackets[nodeid][aux][1] = Simulator::Now().GetMicroSeconds();
        			  NS_LOG_UNCOND("[DEVTXTRACE] Adicionado " << nodeid << ":" << aux << " " << sendPackets[nodeid][aux]);
        			  NS_LOG_UNCOND("SEND0 " << nodeid << " " << aux << " " << sendPackets[nodeid][aux][1]);
        		  }
        		  else
        		  {
        		  sendPackets[nodeid][aux][0] = sendPackets[nodeid][aux][0] + 1;
        		  NS_LOG_UNCOND("[DEVTXTRACE] Adicionado " << nodeid << ":" << aux << " " << sendPackets[nodeid][aux]);
        		  }

        	  }

          }
        }

    }

  os_tlf << "\n";

}

int main (int argc, char **argv)
{
  AodvExample test;
  if (! test.Configure(argc, argv))
    NS_FATAL_ERROR ("Configuration failed. Aborted.");

  test.Run ();
  test.Report (std::cout);

  return 0;
}

//-----------------------------------------------------------------------------
AodvExample::AodvExample () :
		size (30),
		totalTime (150),
		pcap (true),
		printRoutes (true),
		pkt_size (512),
		data_rate (40960), // initially, 8192 bps
		comm_mode ("v2i"),
		traffic_density ("high"),
		test_id ("1.1"),
		runID (3),
		obu_tx_power (5), // initially, 16 dBm (+ 2 dBi =~ 1km range) Vs. 5 dBm (+ 2 dBi =~ 300m range)
		obu_tx_gain (2),
		obu_rx_gain (2),
		obu_antenna_height (1.7),
		rsu_tx_power (18),
		rsu_tx_gain (9),
		rsu_rx_gain (9),
		rsu_antenna_height (6.3)

{
}

bool
AodvExample::Configure (int argc, char **argv)
{
  // Enable AODV logs by default. Comment this if too noisy
   LogComponentEnable("AodvRoutingProtocol", LOG_LEVEL_ALL);

  Packet::EnablePrinting();

  CommandLine cmd;
  cmd.AddValue ("pcap", "Write PCAP traces.", pcap);
  cmd.AddValue ("printRoutes", "Print routing table dumps.", printRoutes);
  cmd.AddValue ("size", "Number of nodes.", size);
  cmd.AddValue ("time", "Simulation time, s.", totalTime);
  cmd.AddValue ("commMode", "Communication mode (v2v; v2i).", comm_mode);
  cmd.AddValue ("traffDensity", "Traffic density (low; high).", traffic_density);
  cmd.AddValue ("testID", "Test ID.", test_id);
  cmd.AddValue ("runID", "Simulation run number.", runID);
  cmd.Parse (argc, argv);

  SeedManager::SetSeed(12345);
  SeedManager::SetRun (runID);

  stringstream sstr;
  sstr << runID;
  runID_str = sstr.str();

  return true;
}

std::string i2string(int i) {
    std::ostringstream buffer;
    buffer << i;
    return buffer.str();
}

void
AodvExample::Run ()
{
//  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", UintegerValue (1)); // enable rts cts all the time.
  CreateNodes ();
  CreateDevices ();
  InstallInternetStack ();
  InstallApplications ();

  txlogFile = "/home/andre/workspace4/ns3-gpsr/logs/aodv-tx_packets_" + runID_str + ".log";
  rxlogFile = "/home/andre/workspace4/ns3-gpsr/logs/aodv-rx_packets_" + runID_str + ".log";

  // Opening logs files for output
  //std::ofstream os_ccl;
  os_rlf.open (rxlogFile.c_str());
  os_tlf.open (txlogFile.c_str());

  // Connecting to trace sources
  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRx", MakeCallback (&AodvExample::WifiMacRxTrace, this));
  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTx", MakeCallback (&AodvExample::DevTxTrace, this));
  Config::Connect ("/NodeList/2/$ns3::Ipv4L3Protocol/Rx", MakeCallback (&AodvExample::RxTrace, this));
  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxDrop", MakeCallback(&MacRxDrop));
  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxDrop", MakeCallback(&MacTxDrop));

  std::cout << "Starting simulation for " << totalTime << " s ...\n";

  Simulator::Stop (Seconds (totalTime));

  AnimationInterface anim ("/home/andre/workspace4/ns3-gpsr/logs/logaodv.xml");
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll();


  Simulator::Run ();


  monitor->CheckForLostPackets ();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
  std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();

  NS_LOG_UNCOND("== Flow Monitor ==");

  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator iter = stats.begin (); iter != stats.end (); ++iter)
  {
	   Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (iter->first);

	  NS_LOG_UNCOND(iter->first << ": " << t.sourceAddress << "->" << t.destinationAddress  << " transmitted "
			  << iter->second.txPackets << " received " << iter->second.rxPackets << " timefirsttx " << iter->second.timeFirstTxPacket
			  << " timefirstrx " << iter->second.timeFirstRxPacket << " D[s]: " << iter->second.delaySum.GetSeconds() / iter->second.rxPackets);
  }

  monitor->SerializeToXmlFile("lab-5.flowmon", true, true);

  Simulator::Destroy ();

  metrics = "/home/andre/workspace4/ns3-gpsr/logs/aodv-" + i2string(Napps) + "S" + i2string(size) + ".log";

  os_metrics.open (metrics.c_str());

  NS_LOG_UNCOND("== My-STATS ==");
  NS_LOG_UNCOND("NºApps : " << Napps);

  int c = 0;
   for(uint32_t i = 0; i < Napps; i++)
   {
 	  realserverNode[i]=0;
 	  realclientNode[i] = 0;
   }

   for(uint32_t i = 0; i < Napps; i++)
   {
 	  for(uint32_t j = 0; j< Napps; j++)
 	  {
 		  if(clientNode[i] == realclientNode[j] && serverNode[i] == realserverNode[j])
 			  c+=1;
 	  }

 	  if(c==0)
 	  {
 		  realclientNode[i] = clientNode[i];
 		  realserverNode[i] = serverNode[i];
 	  }

 	  c=0;
   }

  double EED = 0;
  int TTX = 0;
  int TRX = 0;
  double PLR = 0;
  double PDR = 0;

  os_metrics << "#E  T  TTX  TRX  DTX  DRX  OTX  ORX  ARPTX  ARPRX  EED  PLR  PDR			NumApps " << Napps << " NumNodes "
		  << size << "\n";

  for(uint32_t t = 0; t < Napps; t++)
  {
	  if(sendPackets[clientNode[t]][serverNode[t]][0]!=0 && realclientNode[t]!=realserverNode[t])
	  {
		  //EED = Subtracção de firstRX com firstTX / 1000 para estar em milisegundos
		  EED = (receivePackets[serverNode[t]][clientNode[t]][1] - sendPackets[clientNode[t]][serverNode[t]][1]);
		  EED = EED/1000;

		  TTX = sendPackets[clientNode[t]][serverNode[t]][0];
		  TRX = receivePackets[serverNode[t]][clientNode[t]][0];

		  PLR = (TTX - TRX)/(TTX);
		  PDR = TRX/TTX;

		  NS_LOG_UNCOND ("E " << realclientNode[t] << " R " << realserverNode[t]
		          << " TTX " << TTX
			      << " TRX " << TRX
			      << " DTX " << totaldatatx
			      << " DRX " << totaldatarx
			      << " OTX " << typeheadercounttx
			      << " ORX " << typeheadercount
			      << " ARPTX " << arp_counttx
			      << " ARPRX " << arp_count
			      << " EED " << EED
			      << " receive " << receivePackets[serverNode[t]][clientNode[t]][1]
			      << " send " << sendPackets[clientNode[t]][serverNode[t]][1]);

		  os_metrics << realclientNode[t] << "\t" << realserverNode[t] << "\t" << TTX << "\t" << TRX << "\t"
				  << totaldatatx << "\t" << totaldatarx  << "\t" << typeheadercounttx  << "\t" << typeheadercount  << "\t"
				  << arp_counttx  << "\t" << arp_count  << "\t" << EED  << "\t" << PLR  << "\t" << PDR << "\n";
	  }

  }

  NS_LOG_UNCOND("== DEV TX ==============");
  NS_LOG_UNCOND("DevTX send " << datatx_count);
  NS_LOG_UNCOND("RREQ TX " << rreq_counttx);
  NS_LOG_UNCOND("RERR TX " << rerr_counttx);
  NS_LOG_UNCOND("RREP TX " << rrep_counttx);
  NS_LOG_UNCOND("RREPACK" << rrepack_counttx );
  NS_LOG_UNCOND("ARP TX " << arp_counttx);
  NS_LOG_UNCOND("UDP TX " << udp_counttx);
  NS_LOG_UNCOND("TypeHeader TX " << typeheadercounttx);
  NS_LOG_UNCOND("DROP TX " << count_txDrop);
  NS_LOG_UNCOND("Data TX " << totaldatatx);

  NS_LOG_UNCOND("== WIFIMACRX ==============");
  NS_LOG_UNCOND ("WIFIMACRX received: " << datarx_count);
  NS_LOG_UNCOND ("RREQ TX " << rreq_count);
  NS_LOG_UNCOND("RERR TX " << rerr_count);
  NS_LOG_UNCOND("RREP TX " << rrep_count);
  NS_LOG_UNCOND("RREPACK" << rrepack_count);
  NS_LOG_UNCOND ("UDP: " << udp_count);
  NS_LOG_UNCOND("ARP TX " << arp_count);
  NS_LOG_UNCOND ("TypeHeader " << typeheadercount);
  NS_LOG_UNCOND ("WIFIRXTRACE " << wifitrace);
  NS_LOG_UNCOND ("RXTRACE: " << rxtrace);
  NS_LOG_UNCOND("DROP RX " << count_rxDrop);
  NS_LOG_UNCOND ("Rreq packets RX " << rreq_count);
  NS_LOG_UNCOND("Data RX " << totaldatarx);

}

void
AodvExample::Report (std::ostream &)
{ 
}

void
AodvExample::CreateNodes ()
{
  std::cout << "Creating " << (unsigned)size << " nodes.\n";
  nodes.Create (size);
  // Name nodes
  for (uint32_t i = 0; i < size; ++i)
     {
       std::ostringstream os;
       os << "node-" << i;
       Names::Add (os.str (), nodes.Get (i));
     }

 /*************************************************************************************
 *  MOBILITY
 * **********************************************************************************/

  std::string traceFile = "/home/andre/workspace4/ns3-gpsr/high-1.1-out.tcl";
  //std::string traceFile = "/home/andre/workspace4/ns3-Aodv-l/low-1.1-out.tcl";
  Ns2MobilityHelper mobility = Ns2MobilityHelper (traceFile);
  mobility.Install (); // configure movements for each node, while reading trace file

//  MobilityHelper mobility;
//
//  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
//  		  "MinX", DoubleValue (0.0),
//  		  "MinY", DoubleValue (0.0),
//  		  "DeltaX", DoubleValue (5.0),
//  		  "DeltaY", DoubleValue (10.0),
//  		  "GridWidth", UintegerValue (3),
//  		  "LayoutType", StringValue ("RowFirst"));
//
//  mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
//  mobility.Install (nodes);



  // Separating RSU from OBU nodes
  if (comm_mode == "v2v") // Watch out for obu_devices and crashed_device (0)
    {
       for (uint32_t i = 0; i < size; ++i) {
        // uint32_t node = nodes.Get (i)->GetId ();
        Ptr<Node> node = nodes.Get (i);
        if (node->GetId () != 0) {
          obu_nodes.Add (node);
          std::cout << "Node " << node->GetId () << " added to OBU's" << std::endl;
        }
      }
      obu_nodes.Add (nodes);
    }
  else
    {
      //rsu_nodes = NodeContainer (nodes.Get (1), nodes.Get (2), nodes.Get (3)); // Comment this if only using V2V communication
      for (uint32_t i = 0; i < size; ++i) {
        // uint32_t node = nodes.Get (i)->GetId ();
        Ptr<Node> node = nodes.Get (i);
        if (node->GetId () == 0 || node->GetId () == 1 || node->GetId () == 2)
          {
            rsu_nodes.Add (node);
             std::cout << "Node " << node->GetId () << " added to RSU's" << std::endl;
          }
        else {
          if (node->GetId () > 2) {
            obu_nodes.Add (node);
            std::cout << "Node " << node->GetId () << " added to OBU's" << std::endl;
          }
        }
      }
      std::cout << "OBU nodes = " << obu_nodes.GetN () << " | RSU nodes = " << rsu_nodes.GetN () << std::endl;
    }
}

void
AodvExample::CreateDevices ()
{
	NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
	wifiMac.SetType ("ns3::AdhocWifiMac");

	WifiHelper wifi;

	// wifi.SetStandard (WIFI_PHY_STANDARD_80211_10Mhz);
	wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", StringValue ("OfdmRate6MbpsBW10MHz"), "ControlMode", StringValue ("OfdmRate6MbpsBW10MHz"));

	YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
	Ptr<YansWifiChannel> channel = CreateObject<YansWifiChannel> ();
	Ptr<PropagationDelayModel> propDelay = CreateObject<ConstantSpeedPropagationDelayModel> ();
	Ptr<TwoRayGroundPropagationLossModel> determPropLoss = CreateObject<TwoRayGroundPropagationLossModel> ();
	determPropLoss->SetLambda (5.900e9, 300000000.0);
	Ptr<NakagamiPropagationLossModel> probabPropLoss = CreateObject<NakagamiPropagationLossModel> ();
	probabPropLoss->SetAttribute ("Distance1", DoubleValue (80));
	probabPropLoss->SetAttribute ("Distance2", DoubleValue (250));
	probabPropLoss->SetAttribute ("m0", DoubleValue (1.5));
	probabPropLoss->SetAttribute ("m1", DoubleValue (0.75));
	probabPropLoss->SetAttribute ("m2", DoubleValue (0.5));
	determPropLoss->SetNext(probabPropLoss);

	/* OBU Settings */
	wifiPhy.Set ("TxPowerStart", DoubleValue(obu_tx_power));
	wifiPhy.Set ("TxPowerEnd", DoubleValue(obu_tx_power));
	wifiPhy.Set ("TxPowerLevels", UintegerValue(1));
	wifiPhy.Set ("TxGain", DoubleValue(obu_tx_gain));
	wifiPhy.Set ("RxGain", DoubleValue(obu_rx_gain));

	determPropLoss->SetHeightAboveZ (obu_antenna_height);
	channel->SetPropagationDelayModel (propDelay);
	channel->SetPropagationLossModel (determPropLoss);
	wifiPhy.SetChannel (channel);

	wifiPhy.EnablePcapAll (std::string ("aodv"));

	obu_devices = wifi.Install (wifiPhy, wifiMac, obu_nodes);
	//crashed_device = wifi.Install (wifiPhy, wifiMac, nodes.Get (0));


	/* RSU Settings */
	if (comm_mode == "v2i")
	{
		wifiPhy.Set ("TxPowerStart", DoubleValue(rsu_tx_power));
		wifiPhy.Set ("TxPowerEnd", DoubleValue(rsu_tx_power));
		wifiPhy.Set ("TxPowerLevels", UintegerValue(1));
		wifiPhy.Set ("TxGain", DoubleValue(rsu_tx_gain));
		wifiPhy.Set ("RxGain", DoubleValue(rsu_rx_gain));

		determPropLoss->SetHeightAboveZ (rsu_antenna_height);
		channel->SetPropagationDelayModel (propDelay);
		channel->SetPropagationLossModel (determPropLoss);
		wifiPhy.SetChannel (channel);
		rsu_devices = wifi.Install (wifiPhy, wifiMac, rsu_nodes);

		static_devices = NetDeviceContainer (rsu_devices);
		devices = NetDeviceContainer (static_devices, obu_devices);
	}

//	AsciiTraceHelper ascii;
//	std::string trace = "trace";
//	wifi.EnableAscii(ascii.CreateFileStream("aodv.tr"),nodes);
}

void
AodvExample::InstallInternetStack ()
{
  AodvHelper aodv;
  // you can configure AODV attributes here using aodv.Set(name, value)
  InternetStackHelper stack;
  stack.SetRoutingHelper (aodv); // has effect on the next Install ()
  stack.Install (nodes);
  Ipv4AddressHelper address;
  address.SetBase ("10.0.0.0", "255.0.0.0");
  interfaces = address.Assign (devices);


  if (printRoutes)
    {
      Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> ("aodv.routes", std::ios::out);
      aodv.PrintRoutingTableAllAt (Seconds (8), routingStream);
    }
}

/*************************************************************************************
 *  APPLICATIONS
 * **********************************************************************************/

void
AodvExample::InstallApplications ()
{
	uint32_t iClient; // The node in which will be setup a client application
	uint32_t iServer;// The node in which will be setup a server application
	ApplicationContainer apps;

	//FIXME: Supostamente isto é para ser superior
	double ThresholdNeighbor = 1000;

	uint32_t srv_port = 9;

	//double interval = 0.04; // seconds
	Time interPacketInterval = Seconds (1.);
	uint32_t packetSize = 1024; // bytes

	uint32_t numNodes = nodes.GetN ();
	bool flag= true ;

	int time_step =2;
	for (uint32_t move = 0; move < Napps; move++)
	{
		flag= true ;

		while ( flag )
		{
			iClient = rand() % numNodes ;
			iServer = rand() % numNodes ;
			std::cout << " " << iClient << " "  << iServer << std::endl;

			Ptr<Node> iClientNode = nodes.Get (iClient);
			Ptr<Node> iServerNode = nodes.Get (iServer);

			if (iClientNode != iServerNode)
			{
				double dst = iClientNode->GetObject<MobilityModel> ()->GetDistanceFrom
						(iServerNode->GetObject<MobilityModel> ());

				std::cout << dst << std::endl ;

//				NS_LOG_UNCOND("Distance from " << nodes.Get (iClient)->GetId() << " to " << nodes.Get (iServer)->GetId()
//						<< " is " << dst);

				// if the distance is bellow the Threshold , is supposed to be a neighbor
				if(dst < ThresholdNeighbor)
				{
					clientNode[move] = iClient; // The node in which will be setup a client application
					serverNode[move] = iServer;

					NS_LOG_UNCOND("INSTALL APPLICATIONS Client Node " << nodes.Get (iClient)->GetId()
							<< " Server Node " << nodes.Get (iServer)->GetId());

					//NS_LOG_UNCOND("Distance entre cliente e nó abaixo do threshold");

					/**************************************************
					* SERVER APPLICATION
					**************************************************/

					UdpServerHelper server(srv_port);
					apps = server.Install( nodes.Get(iServer));
					apps.Start(Seconds(30));
					apps.Stop(Seconds(90));

					// Create one UdpClient application to send UDP datagrams from node to node

					/**************************************************
					* CLIENT APPLICATION
					**************************************************/

					UdpClientHelper client(interfaces.GetAddress(iServer), srv_port);

					client.SetAttribute("Interval", TimeValue(interPacketInterval));
					client.SetAttribute("PacketSize", UintegerValue(packetSize));
					apps = client.Install(nodes.Get(iClient));
					apps.Start(Seconds(30));
					apps.Stop(Seconds(90));
					flag=false;
					srv_port++;
					time_step += 2;

				}
			}

		}
	}

}

