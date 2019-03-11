/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "ns3/netanim-module.h"
#include "ns3/aodv-module.h"
#include "ns3/gpsr-helper.h"
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
#include <string.h>
#include <iostream>
#include <limits>
#include <stdlib.h>
#include <iomanip>
#include <fstream>
#include <vector>
#include <algorithm>

using namespace ns3;

const uint32_t Napps=5;

int clientNode[Napps];
int serverNode[Napps];
//int realclientNode[Napps];
//int realserverNode[Napps];
int datatx_count = 0;
int datarx_count = 0;
int ipcounttx = 0;
int ipcountrx = 0;
int arp_counttx = 0;
int arp_count = 0;
int hellocounttx = 0;
int hellocountrx = 0;
int typeheadercounttx = 0;
int udp_count = 0;
int udp_counttx = 0;
int typeheadercount = 0;
int wifitrace = 0;
int rxtrace = 0;
int totaldatatx = 0;
int totaldatarx = 0;
bool first_packetRcvd[271];
double last_node_position[271];
double last_node_vel[271];

//Z=0 é onde guarda o contador de pacotes, Z=1 guarda valor primeiro send or receive
uint64_t receiveDataPackets[271][271];
uint64_t sendDataPackets[271][271];

//Map e Struct para os Sequence Numbers
multimap<string, double> mapsend;
multimap<string, double> mapreceive;
multimap<double, uint64_t> eesend;
multimap<double, uint64_t> eereceive;

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

class GpsrExample
{
public:
  GpsrExample ();
  /// Configure script parameters, \return true on successful configuration
  bool Configure (int argc, char **argv);
  /// Run simulation
  void Run ();
  /// Report results
  void Report (std::ostream & os);
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
  std::ofstream os_headers;
  std::string headerslogfile;

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
GpsrExample::WifiMacRxTrace (std::string context, Ptr<const Packet> p)
{
	size_t first_delimit = context.find_first_of ('/', 1);
	size_t second_delimit = context.find_first_of ('/', first_delimit + 1);
	std::string nodeid_str = context.substr (first_delimit + 1, second_delimit - 1 - first_delimit);
	stringstream ss (nodeid_str);
	uint32_t nodeid;
	ss >> nodeid;

	datarx_count +=1;

	//NS_LOG_UNCOND("STRING WIFI node id " << nodeid << " " << context );

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

			if (item.tid.GetName() == "ns3::Ipv4Header")
			{
				ipcountrx +=1;
			}

			if(item.tid.GetName() == "ns3::ArpHeader")
			{
				arp_count +=1;
			}
			if(item.tid.GetName() == "ns3::UdpHeader")
			{
				udp_count += 1;
			}
			if(item.tid.GetName() == "ns3::gpsr::HelloHeader")
			{
				hellocountrx += 1;
			}

			if (item.tid.GetName() == "ns3::SeqTsHeader")
			{
				ns3::SeqTsHeader seq;
				copy->PeekHeader (seq);

				Ipv4Header iph;
				copy->PeekHeader (iph);
				Ipv4Address src = iph.GetSource();
				Ipv4Address dst = iph.GetDestination();
				//      	  uint8_t ttl = iph.GetTtl ();
				//        	  uint8_t hops = 64 - ttl;
				//    	  uint16_t packet_id = iph.GetIdentification ();

				totaldatarx +=1;

				uint32_t i = 0;
				uint32_t aux = 0;
				uint32_t i2 = 0;
				uint32_t aux2 = 0;

				while(i<interfaces.GetN())
				{
					if(interfaces.GetAddress(i) == src)
					{
						aux = i;
						break;
					}

					i++;
				}

				while(i2<interfaces.GetN())
				{
					if(interfaces.GetAddress(i2) == dst)
					{
						aux2 = i2;
						break;
					}

					i2++;
				}

				//NS_LOG_UNCOND("[WIFIMACRXTRACE] src " << src << " node id " << nodeid << " dst " << dst << " " << aux
						//<< " Time " << Simulator::Now() << receiveDataPackets[aux][nodeid]);

				if(src!=Ipv4Address("10.255.255.255"))
				{
					if(nodeid == aux2)
					{
						//TRACE
						//os_rlf << nodeid << "\t" << aux << "\t" << src << "\t" << dst << " timestamp " << seq.GetTs().GetDouble() << "\n" ;

						std::ostringstream buffer1;
						buffer1 << aux2;
						std::ostringstream buffer2;
						buffer2 << aux;

						//MapSequenceNumbers: Criar string com concatenação e inserir
						string key = buffer2.str() + ":" +  buffer1.str();

						buffer1.clear();
						buffer2.clear();

						mapreceive.insert(pair<string, double>(key, seq.GetTs().GetDouble()));

						int64_t time = Simulator::Now().GetMicroSeconds();

						os_rlf << key << "\t" << seq.GetTs().GetDouble() << "\t" << time << "\n";

						eereceive.insert(pair<double, uint64_t>(seq.GetTs().GetDouble(), time));

						//os_rlf << aux << "\t" << aux2 << "\t" << "TimeSt" << "\t" << seq.GetTs() << "\t" << "Simu" <<
								//"\t" << time << "\n";

						receiveDataPackets[aux][aux2] += 1;

					}
				}

			}
		}
	}

	//os_rlf << "\n";
}

void
GpsrExample::RxTrace (std::string context, Ptr<const Packet> packet, Ptr<Ipv4> ipv4, uint32_t i)
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
  uint8_t ttl = iph.GetTtl ();
  uint8_t hops = 64 - ttl;
  uint16_t packet_id = iph.GetIdentification ();
  Ipv4Address src = iph.GetSource ();

 // if (ttl > 1  && src == "10.0.0.1")
  if (ttl > 1)
    {
      if (!first_packetRcvd[nodeid]) // All nodes generating at the same time
        {
          os_rlf <<  Simulator::Now () << "|" << nodeid << "|" << packet_id << "|" << node_mob->GetPosition ().x << "|" << node_mob->GetDistanceFrom (crashed_node_mob) << "|" << (int) ttl << "|" << (int) hops << "|" << src << std::endl;
          first_packetRcvd[nodeid] = true;
        }
    }
}

void
GpsrExample::DevTxTrace (std::string context, Ptr<const Packet> p)
{
	size_t first_delimit = context.find_first_of ('/', 1);
	size_t second_delimit = context.find_first_of ('/', first_delimit + 1);
	std::string nodeid_str = context.substr (first_delimit + 1, second_delimit - 1 - first_delimit);
	stringstream ss (nodeid_str);
	uint32_t nodeid;
	ss >> nodeid;

	datatx_count = datatx_count + 1;

	//NS_LOG_UNCOND("CONTEXT DEVTXTRACE " << context << " nodeid " << nodeid);

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
			os_headers << item.tid.GetName() << "\t";

			if (item.tid.GetName() == "ns3::Ipv4Header")
			{
				ipcounttx +=1;
			}

			if(item.tid.GetName() == "ns3::ArpHeader")
			{
				arp_counttx +=1;
			}
			if(item.tid.GetName() == "ns3::UdpHeader")
			{
				udp_counttx += 1;
			}
			if(item.tid.GetName() == "ns3::gpsr::HelloHeader")
			{
				hellocounttx += 1;
			}

			if (item.tid.GetName() == "ns3::SeqTsHeader")
			{
				ns3::SeqTsHeader seq;
				copy->PeekHeader(seq);

				Ipv4Header iph;
				copy->PeekHeader (iph);
//				Ipv4Address src = iph.GetSource();
				Ipv4Address dst = iph.GetDestination();
				//uint8_t ttl = iph.GetTtl ();
				//uint8_t hops = 64 - ttl;
				//uint16_t packet_id = iph.GetIdentification ();

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

				//NS_LOG_UNCOND("[DEVTXTRACE] src " << src << " node id " << nodeid << " dst " << dst << " " << aux
						//<< " Time " << Simulator::Now());

				if(dst!=Ipv4Address("10.255.255.255"))
				{
					//TRACE
					//os_tlf << nodeid << "\t" << aux << "\t" << src << "\t" << dst << " seqnumber " << seq.GetTs().GetDouble() << "\n" ;

					std::ostringstream buffernodeid;
					buffernodeid << nodeid;
					std::ostringstream bufferaux;
					bufferaux << aux;

					//MapSequenceNumbers: Criar string com concatenação e inserir
					string key = buffernodeid.str() + ":" +  bufferaux.str();

					buffernodeid.clear();
					bufferaux.clear();

					mapsend.insert(pair<string, double>(key, seq.GetTs().GetDouble()));

					int64_t time = Simulator::Now().GetMicroSeconds();

					os_tlf << key << "\t" << seq.GetTs().GetDouble() << "\t" << time << "\n";

					//NS_LOG_UNCOND("Chave " << key << " seqTS " << seq.GetTs().GetDouble() << " time " << time );

					eesend.insert(pair<double, uint64_t>(seq.GetTs().GetDouble(), time));

					sendDataPackets[nodeid][aux] += 1;

					//NS_LOG_UNCOND("[DEVTXTRACE] Adicionado " << nodeid << ":" << aux << " " << sendDataPackets[nodeid][aux]);

				}

			}
		}

	}
	os_headers << "\n";
	//os_tlf << "\n";
}

std::string i2string(int i) {
    std::ostringstream buffer;
    buffer << i;
    return buffer.str();
}

bool equals(double d1, double d2, double precision)
{
double eps1 = fabs(d1), eps2 = fabs(d2), eps;
eps = (eps1 > eps2) ? eps1 : eps2;
if (eps == 0.0)
return true; //both numbers are 0.0
//eps hold the minimum distance between the values
//that will be considered as the numbers are equal
//considering the magnitude of the numbers
eps *= precision;
return (fabs(d1 - d2) < eps);
}

int main (int argc, char **argv)
{
  GpsrExample test;
  if (! test.Configure(argc, argv))
    NS_FATAL_ERROR ("Configuration failed. Aborted.");

  test.Run ();
  test.Report (std::cout);

  return 0;
}

//-----------------------------------------------------------------------------
//TODO
//AQUI
GpsrExample::GpsrExample () :
  size (20),
  totalTime (120),
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
GpsrExample::Configure (int argc, char **argv)
{
  // Enable AODV logs by default. Comment this if too noisy
   LogComponentEnable("GpsrRoutingProtocol", LOG_LEVEL_ALL);

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

//#define VERYSMALL  (1.0E+150)
//#define EPSILON    (1.0E+8)
//bool AreSame(double a, double b)
//{
//    double absDiff = fabs(a - b);
//    if (absDiff < VERYSMALL)
//    {
//        return true;
//    }
//
//    double maxAbs  = max(fabs(a) - fabs(b));
//    return (absDiff/maxAbs) < EPSILON;
//}

void
GpsrExample::Run ()
{
	  CreateNodes ();
	  CreateDevices ();
	  InstallInternetStack ();
	  InstallApplications ();

	  GpsrHelper gpsr;
	  gpsr.Install ();

	  txlogFile = "/home/andre/workspace4/ns3-gpsr/logs/gpsr-tx_packets_" + runID_str + ".log";
	  rxlogFile = "/home/andre/workspace4/ns3-gpsr/logs/gpsr-rx_packets_" + runID_str + ".log";
	  headerslogfile = "/home/andre/workspace4/ns3-gpsr/logs/gpsr-headers_" + runID_str + ".log";

	  // Opening logs files for output
	  //std::ofstream os_ccl;
	  os_rlf.open (rxlogFile.c_str());
	  os_tlf.open (txlogFile.c_str());
	  os_headers.open(headerslogfile.c_str());

	  // Connecting to trace sources
	  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRx", MakeCallback (&GpsrExample::WifiMacRxTrace, this));
	  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTx", MakeCallback (&GpsrExample::DevTxTrace, this));
	  Config::Connect ("/NodeList/2/$ns3::Ipv4L3Protocol/Rx", MakeCallback (&GpsrExample::RxTrace, this));
	  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxDrop", MakeCallback(&MacRxDrop));
	  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxDrop", MakeCallback(&MacTxDrop));

	  std::cout << "Starting simulation for " << totalTime << " s ...\n";

	  Simulator::Stop (Seconds (totalTime));

	  AnimationInterface anim ("/home/andre/workspace4/ns3-gpsr/logs/loggpsr.xml");

	  Simulator::Run ();

	  Simulator::Destroy ();

	  metrics = "/home/andre/workspace4/ns3-gpsr/logs/gpsr-" + i2string(Napps) + "S" + i2string(size) + ".log";

	  os_metrics.open (metrics.c_str());

	  NS_LOG_UNCOND("== My-STATS ==");
	  NS_LOG_UNCOND("NºApps : " << Napps);

	  //double EED = 0;
	  int DTX = 0;
	  int DRX = 0;
	  double PLR = 0;
	  double PDR = 0;
	  int EEDVector[Napps];

	  os_metrics << "#E  T  TTX  TRX  DTX  DRX  OTX  ORX  ARPTX  ARPRX  EED  PLR  PDR			NumApps " << Napps << " NumNodes "
			  << size << "\n";

	  for(uint32_t t = 0; t < Napps; t++)
	  {
		  if(sendDataPackets[clientNode[t]][serverNode[t]]!=0 && clientNode[t]!=serverNode[t])
		  {
			  //EED = Subtracção de firstRX com firstTX / 1000 para estar em milisegundos
//			  EED = (receiveDataPackets[serverNode[t]][clientNode[t]][1] - sendDataPackets[clientNode[t]][serverNode[t]][1]);
//			  EED = EED/1000;

			  string chave = i2string(clientNode[t]) + ":" + i2string(serverNode[t]);

			  DTX = mapsend.count(chave);
			  DRX = mapreceive.count(chave);

//			  uint64_t EESEND = sendDataPackets[clientNode[t]][serverNode[t]][1];
//			  uint64_t EERECV = sendDataPackets[clientNode[t]][serverNode[t]][1];

			  PLR = double((DTX - DRX))/(DTX);
			  PDR = double(DRX) /DTX;
			  PDR = PDR * 100;
			  PLR = PLR * 100;

			  uint64_t esend = 0;
			  uint64_t erecv = 0;

			  EEDVector[t] = 0;

			  os_tlf << " Chave " << chave << "\n";

			  for(multimap<string,double>::iterator iter1 = mapsend.begin(); iter1 != mapsend.end(); iter1++)
			  {
				  for(multimap<string,double>::iterator iter2 = mapreceive.begin(); iter2 != mapreceive.end(); iter2++)
				  {
					  if((*iter1).first == chave && (*iter2).first == chave)
					  {
						  if(equals((*iter1).second,(*iter2).second,0.000001))
						  {
							 // NS_LOG_UNCOND("Chave " << chave << " " << (*iter1).second << " " << (*iter2).second );
							  //Situação em que os pacotes tx são rx e sei quais são a partir do TS
							  //Mas a diferença do EED é do pacote enviado e não do TS.

							  for(multimap<double,uint64_t>::iterator iter3 = eesend.begin();
									  iter3 != eesend.end(); iter3++)
							  {
								  if(equals((*iter1).second,(*iter3).first,0.000001))
								  {
									  esend = (*iter3).second;
								  }
							  }

							  for(multimap<double,uint64_t>::iterator iter4 = eereceive.begin();
							 	iter4 != eereceive.end(); iter4++)
							  {
								  if(equals((*iter1).second,(*iter4).first,0.000001))
								  {
									  erecv = (*iter4).second;
								  }
							  }

							  //MicroSeconds
							  if(erecv>esend)
							  {
							  int sub = erecv - esend;
							  EEDVector[t] = EEDVector[t] + sub;
							  NS_LOG_UNCOND("TSEND: " << esend << " TRECEIVE: " << erecv << " SUB = " << sub << " EEDVector " <<
									  EEDVector[t] );
//							  os_metrics << (*iter1).first << "\t" << (*iter1).second << "\t"
//									  << (*iter2).first << "\t" << (*iter2).second << "\n";
							  }
						  }
					  }
				  }
			  }

			  int AEED = 0;
			  AEED = EEDVector[t] / Napps;
			  //MiliSeconds
			  AEED = AEED/1000;
			  NS_LOG_UNCOND("AEED: " << AEED);


			  NS_LOG_UNCOND ("E " << clientNode[t] << " R " << serverNode[t]
			          << " TTX " << datatx_count
				      << " TRX " << datarx_count
				      << " DTX " << DTX
				      << " DRX " << DRX
				      << " OTX " << mapsend.size()
				      << " ORX " << mapreceive.size()
				      << " ARPTX " << arp_counttx
				      << " ARPRX " << arp_count
				      << " PDR " << PDR
				      << " PLR " << PLR
				      << " AEED " << AEED
				      << " receive " << receiveDataPackets[serverNode[t]][clientNode[t]]
				      << " send " << sendDataPackets[clientNode[t]][serverNode[t]]);
//
//			  os_metrics << realclientNode[t] << "\t" << realserverNode[t] << "\t" << TTX << "\t" << TRX << "\t"
//					  << totaldatatx << "\t" << totaldatarx  << "\t" << typeheadercounttx  << "\t" << typeheadercount  << "\t"
//					  << arp_counttx  << "\t" << arp_count  << "\t" << EED  << "\t" << PLR  << "\t" << PDR << "\n";
		  }

	  }

	  NS_LOG_UNCOND("== DEV TX ==============");
	  NS_LOG_UNCOND("DevTX send " << datatx_count);
	  NS_LOG_UNCOND("ARP TX " << arp_counttx);
	  NS_LOG_UNCOND("UDP TX " << udp_counttx);
	  NS_LOG_UNCOND("IPv4 TX " << ipcounttx);
	  NS_LOG_UNCOND("DROP TX " << count_txDrop);
	  NS_LOG_UNCOND("Data TX " << totaldatatx);
	  NS_LOG_UNCOND("Hello TX " << hellocounttx);

	  NS_LOG_UNCOND("== WIFIMACRX ==============");
	  NS_LOG_UNCOND ("WIFIMACRX received: " << datarx_count);
	  NS_LOG_UNCOND("ARP TX " << arp_count);
	  NS_LOG_UNCOND ("UDP RX: " << udp_count);
	  NS_LOG_UNCOND("IPv4 RX " << ipcountrx);
	  NS_LOG_UNCOND ("Hello RX " << hellocountrx);
	  NS_LOG_UNCOND ("RXTRACE: " << rxtrace);
	  NS_LOG_UNCOND("DROP RX " << count_rxDrop);
	  NS_LOG_UNCOND("Data RX " << totaldatarx);
}

void
GpsrExample::Report (std::ostream &)
{
}

void
GpsrExample::CreateNodes ()
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

	  std::string traceFile = "/home/andre/workspace4/ns3-gpsr/teste1v2v.tcl";
	 // std::string traceFile = "/home/andre/workspace4/ns3-Aodv-l/low-1.1-out.tcl";
	  Ns2MobilityHelper mobility = Ns2MobilityHelper (traceFile);
	  mobility.Install (); // configure movements for each node, while reading trace file

//	  MobilityHelper mobility;

//	  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
//	  		  "MinX", DoubleValue (0.0),
//	  		  "MinY", DoubleValue (0.0),
//	  		  "DeltaX", DoubleValue (5.0),
//	  		  "DeltaY", DoubleValue (10.0),
//	  		  "GridWidth", UintegerValue (3),
//	  		  "LayoutType", StringValue ("RowFirst"));
//
//	  mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
//	  mobility.Install (nodes);



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
GpsrExample::CreateDevices ()
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

      static_devices = NetDeviceContainer (crashed_device, rsu_devices);
      devices = NetDeviceContainer (static_devices, obu_devices);
    }

  if (pcap && traffic_density == "low")
    {
      wifiPhy.EnablePcapAll (std::string ("broadcast-gpsr"));
    }
}

void
GpsrExample::InstallInternetStack ()
{
  GpsrHelper gpsr;
  InternetStackHelper stack;
  stack.SetRoutingHelper (gpsr);
  stack.Install (nodes);
  Ipv4AddressHelper address;
  address.SetBase ("10.0.0.0", "255.0.0.0");
  interfaces = address.Assign (devices);

//  for (uint32_t n = 0; n < nodes.GetN (); n++) {
//    Ptr<Node> rsu = nodes.Get (n);
//    Ptr<Ipv4> ipv4 = rsu->GetObject<Ipv4>();
//    Ipv4InterfaceAddress iaddr = ipv4->GetAddress (1,0);
//    Ipv4Address addri = iaddr.GetLocal ();
//    std::cout << "RSU (" << rsu->GetId () << ") address: " << addri << std::endl;
//  }

    if (printRoutes)
      {
        Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> ("gpsr.routes", std::ios::out);
        gpsr.PrintRoutingTableAllAt (Seconds (8), routingStream);
      }
}


void
GpsrExample::InstallApplications ()
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

	//uint32_t numNodes = nodes.GetN ();
	bool flag= true ;

	int time_step =2;
	for (uint32_t move = 0; move < Napps; move++)
	{
		flag= true ;

		while ( flag )
		{
//			iClient = rand() % numNodes ;
//			iServer = rand() % numNodes ;
			iClient = 2;
			iServer = 17;
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

					NS_LOG_UNCOND("INSTALL APPLICATIONS Client Node " << nodes.Get (iClient)->GetId() << " iclient "
							<< iClient << " Server Node " << nodes.Get (iServer)->GetId() <<
							" iServer " << iServer);

					//NS_LOG_UNCOND("Distance entre cliente e nó abaixo do threshold");

					/**************************************************
					* SERVER APPLICATION
					**************************************************/

					UdpServerHelper server(srv_port);
					apps = server.Install( nodes.Get(iServer));
					NS_LOG_UNCOND("Instalado server " << nodes.Get(iServer) << " na porta " << srv_port);
					apps.Start(Seconds(30));
					apps.Stop(Seconds(90));

					// Create one UdpClient application to send UDP datagrams from node to node

					/**************************************************
					* CLIENT APPLICATION
					**************************************************/

					UdpClientHelper client(interfaces.GetAddress(iServer), srv_port);
					NS_LOG_UNCOND("Instalado Cliente " << nodes.Get(iClient) << " linkado a Server " <<
							iServer << " interface: " << interfaces.GetAddress(iServer) );
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

/*************************************************************************************
 *  LOGS
 * **********************************************************************************/

//Criação de um ficheiro de log sempre que é recebida uma msg GPSR com o seu conteúdo


