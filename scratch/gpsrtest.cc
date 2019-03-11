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

using namespace ns3;

bool first_packetRcvd[271];
uint32_t packetsRcvd[271];
uint32_t packetsSent[271];
double last_node_position[271];
double last_node_vel[271];

uint32_t count_rxDrop[271];
uint32_t count_txDrop[271];

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
  void CourseChange (std::string context, Ptr<const MobilityModel> mobility);
  void SetupPacketSend (uint32_t nodeid);
  void DevTxTrace (std::string context, Ptr<const Packet> p);
  void SetupPacketReceive (Ipv4Address addr, Ptr <Node> node, uint16_t port);
  void ReceivePacket (Ptr<Socket> socket);
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
  std::string tx_rx_per_node;
  std::ofstream os_pdr;
  std::string txlogFile; // Frame Transmission Log File
  std::ofstream os_tlf;
  std::string rxlogFile; // Frame Reception Log File
  std::ofstream os_rlf;
  std::string coursechangelog; // Course Change Log File
  std::ofstream os_ccl;
  std::string finalcoursechangelog; // Course Change Log File - Final Positions
  std::ofstream os_fccl;

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

void
GpsrExample::CourseChange (std::string context, Ptr<const MobilityModel> mobility)
{
  size_t first_delimit = context.find_first_of ('/', 1);
  size_t second_delimit = context.find_first_of ('/', first_delimit + 1);
  std::string nodeid_str = context.substr (first_delimit + 1, second_delimit - 1 - first_delimit);
  stringstream ss (nodeid_str);
  uint32_t nodeid;
  ss >> nodeid;

  if (Simulator::Now ().GetSeconds () == 0) {
    std::cout << nodeid << " " << mobility->GetPosition () << std::endl;
  }

  //std::cout << "----- " << Simulator::Now () << "|" << changes_per_sec[nodeid] << "|" << nodeid << "|" << mobility->GetVelocity ().x << "|" << mobility->GetPosition ().x << std::endl;

  Ptr<Node> node0 = nodes.Get(0);
  Ptr<MobilityModel> mob0 = node0->GetObject<MobilityModel>();

  last_node_position[nodeid] = mobility->GetPosition ().x;

  // When all nodes generate traffic as soon as they stop (and node 0 crashes).
  // if (Simulator::Now ().GetSeconds () > 0)
  //   {
  //     if (changes_per_sec[nodeid] == 1 && floor (Simulator::Now ().GetSeconds () + 0.1) != last_sec[nodeid]) {
  //       if (first_change[nodeid] == 0 && nodeid != 0) {
  //         std::cout << Simulator::Now () << "|" << floor (Simulator::Now ().GetSeconds () + 0.1)<< "|" << changes_per_sec[nodeid] << "|" << last_sec[nodeid]<< "|" << nodeid << "|" << mobility->GetVelocity ().x << std::endl;
  //         just_crashed[nodeid] = true;
  //         Simulator::ScheduleNow (&GpsrExample::SetupPacketSend, this, nodeid);
  //       }
  //       changes_per_sec[nodeid] = 0;
  //     }

  //     if (!just_crashed[nodeid]) {
  //       changes_per_sec[nodeid] = changes_per_sec[nodeid] + 1;
  //     }

  //     if (changes_per_sec[nodeid] == 1)
  //       {
  //         last_sec[nodeid] = floor (Simulator::Now ().GetSeconds () + 0.1);
  //         first_change[nodeid] = mobility->GetVelocity ().x;
  //       }

  //     if (changes_per_sec[nodeid] == 2 && last_sec[nodeid] == floor (Simulator::Now ().GetSeconds () + 0.1)) {
  //       second_change[nodeid] = mobility->GetVelocity ().x;

  //       if (first_change[nodeid] == 0) {
  //       // std::cout << time_of_crash[nodeid] << "|" << floor (Simulator::Now ().GetSeconds () + 0.1) << std::endl;
  //         if ((second_change[nodeid] < 20) && nodeid != 0 && !just_crashed[nodeid])
  //           {
  //             just_crashed[nodeid] = true;
  //             time_of_crash[nodeid] = floor (Simulator::Now ().GetSeconds () + 0.1);
  //             std::cout << time_of_crash[nodeid] << "|" << floor (Simulator::Now ().GetSeconds () + 0.1) << "|" << nodeid << "|" << first_change[nodeid] << "|" << second_change[nodeid] << std::endl;
  //             Simulator::ScheduleNow (&GpsrExample::SetupPacketSend, this, nodeid);
  //         }
  //       }

  //       //if ((first_change[nodeid] == 0 || first_change[nodeid] < 20) && (second_change[nodeid]  == 0 || second_change[nodeid] < 20) && !just_crashed[nodeid])
  //       // if ((first_change[nodeid] == 0) && (second_change[nodeid] == 0) && !just_crashed[nodeid])
  //       //   {
  //       //     just_crashed[nodeid] = true;
  //       //     time_of_crash[nodeid] = floor (Simulator::Now ().GetSeconds () + 0.1);
  //       //     //std::cout << Simulator::Now () << "|" << changes_per_sec[nodeid] << "|" << first_change[nodeid] << "|" << second_change[nodeid] << "|" << nodeid << "|" << mobility->GetPosition() << std::endl;

  //       //     if (nodeid == 0)
  //       //       {
  //       //         double crashed_position = mobility->GetPosition ().x;
  //       //         os_ccl << Simulator::Now () << "|" << nodeid << "|" << crashed_position << std::endl;
  //       //         for (uint32_t n = 1; n < size; n++)
  //       //           {
  //       //             Ptr<Node> network_node = nodes.Get (n);
  //       //             Ptr<MobilityModel> node_mob = network_node->GetObject<MobilityModel>();
  //       //             os_ccl << Simulator::Now () << "|" << network_node->GetId () << "|" << node_mob->GetPosition ().x << std::endl;
  //       //           }
  //       //       }
  //           // else
  //           //   {
  //           //     SetupPacketSend (nodeid);
  //           //     Simulator::ScheduleNow (&GpsrExample::SetupPacketSend, this, nodeid);
  //           //   }
  //       // }
  //       changes_per_sec[nodeid] = 0;
  //     }
  //   }

  // When Node 0 is the only traffic generator.
  if (nodeid == 0 && Simulator::Now ().GetSeconds () > 1)
    {
       if (changes_per_sec[nodeid] == 1 && floor (Simulator::Now ().GetSeconds () + 0.1) != last_sec[nodeid])
         {
           if (first_change[nodeid] == 0) {
             // std::cout << Simulator::Now () << "|" << floor (Simulator::Now ().GetSeconds () + 0.1)<< "|" << changes_per_sec[nodeid] << "|" << last_sec[nodeid]<< "|" << nodeid << "|" << mobility->GetVelocity ().x << std::endl;
             just_crashed[nodeid] = true;
           }
           changes_per_sec[nodeid] = 0;
         }

       if (!just_crashed[nodeid])
         changes_per_sec[nodeid] = changes_per_sec[nodeid] + 1;
       if (changes_per_sec[nodeid] == 1)
         {
           first_change[nodeid] = mobility->GetVelocity ().x;
           last_sec[nodeid] = floor (Simulator::Now ().GetSeconds () + 0.1);
         }
       if (changes_per_sec[nodeid] == 2)
         {
           second_change[nodeid] = mobility->GetVelocity ().x;

           if (first_change[nodeid] == 0 &&  second_change[nodeid] == 0 && !just_crashed[nodeid]) {
             just_crashed[nodeid] = true;

             double crashed_position = mobility->GetPosition ().x;
             os_ccl << Simulator::Now () << "|" << nodeid << "|" << crashed_position << std::endl;
             for (uint32_t n = 1; n < size; n++)
               {
                 Ptr<Node> network_node = nodes.Get (n);
                 Ptr<MobilityModel> node_mob = network_node->GetObject<MobilityModel>();
                 os_ccl << Simulator::Now () << "|" << network_node->GetId () << "|" << node_mob->GetPosition ().x << std::endl;
               }
           }
           changes_per_sec[nodeid] = 0;
         }
    }
}

// void
// GpsrExample::CourseChange (std::string context, Ptr<const MobilityModel> mobility)
// {
//   size_t first_delimit = context.find_first_of ('/', 1);
//   size_t second_delimit = context.find_first_of ('/', first_delimit + 1);
//   std::string nodeid_str = context.substr (first_delimit + 1, second_delimit - 1 - first_delimit);
//   stringstream ss (nodeid_str);
//   uint32_t nodeid;
//   ss >> nodeid;

//   Ptr<Node> node0 = nodes.Get(0);
//   Ptr<MobilityModel> mob0 = node0->GetObject<MobilityModel>();

//   if (last_sec[nodeid] != floor (Simulator::Now ().GetSeconds () + 0.1))
//     {
//       std::cout << "----- " << Simulator::Now () << " " << last_sec[nodeid] << " " << nodeid << " " << mobility->GetPosition ().x << std::endl;

//       if (mobility->GetPosition ().x == last_node_position[nodeid]) {
//         std::cout << Simulator::Now () << " " << nodeid << " " << mobility->GetPosition ().x << " " << last_node_position[nodeid] << std::endl;
//         Simulator::ScheduleNow (&GpsrExample::SetupPacketSend, this, nodeid);
//       }
//       last_node_position[nodeid] = mobility->GetPosition ().x;

//       last_sec[nodeid] = floor (Simulator::Now ().GetSeconds () + 0.1);
//     }
// }

static void
MacTxDrop (std::string context, Ptr<const Packet> packet)
{
   size_t first_delimit = context.find_first_of ('/', 1);
   size_t second_delimit = context.find_first_of ('/', first_delimit + 1);
   std::string nodeid_str = context.substr (first_delimit + 1, second_delimit - 1 - first_delimit);
   stringstream ss (nodeid_str);
   uint32_t nodeid;
   ss >> nodeid;

   count_txDrop[nodeid] = count_txDrop[nodeid] + 1;

   NS_LOG_UNCOND("[MACTXDROP] DROPOU " << nodeid);
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

  count_rxDrop[nodeid] = count_rxDrop[nodeid] + 1;

  NS_LOG_UNCOND("[MACRXDROP] DROPOU " << nodeid);

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
              Ipv4Header iph;
              copy->PeekHeader (iph);
              uint8_t ttl = iph.GetTtl ();
              uint8_t hops = 64 - ttl;
              uint16_t packet_id = iph.GetIdentification ();
              Ipv4Address src = iph.GetSource ();

              //if (!first_packetRcvd[nodeid] && ttl > 1)
              //if (ttl > 1  && src == "10.0.0.1" && Simulator::Now ().GetSeconds () >= 96)
              if (ttl > 1)
                {
                  // if(src == "10.0.0.1")
                  //   std::cout << Simulator::Now () << "|" << nodeid << "|" << src << "|" << packet_id << "|" << node_mob->GetPosition ().x << "|" << node_mob->GetDistanceFrom (crashed_node_mob) << "|" << (int) ttl << "|" << (int) hops << std::endl;

                  //packetsRcvd[nodeid] = packetsRcvd[nodeid] + 1;
                  //if (!first_packetRcvd[nodeid] && (packetsRcvd[nodeid] == 0))
                  if (!first_packetRcvd[nodeid]) // All nodes generating at the same time
                    {
                      // os_rlf <<  Simulator::Now () << "|" << nodeid << "|" << packet_id << "|" << node_mob->GetPosition ().x << "|" << node_mob->GetDistanceFrom (crashed_node_mob) << "|" << (int) ttl << "|" << (int) hops << std::endl;
                      os_rlf <<  Simulator::Now () << "|" << nodeid << "|" << packet_id << "|" << node_mob->GetPosition ().x << "|" << node_mob->GetDistanceFrom (crashed_node_mob) << "|" << (int) ttl << "|" << (int) hops << "|" << src << std::endl;
                      first_packetRcvd[nodeid] = true;
                    }
                }
            }
        }
    }
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
          if (item.tid.GetName() == "ns3::Ipv4Header")
            {
              Ipv4Header iph;
              copy->PeekHeader (iph);
           //   Ipv4Address src = iph.GetSource();
              uint8_t ttl = iph.GetTtl ();
              uint8_t hops = 64 - ttl;
              uint16_t packet_id = iph.GetIdentification ();

             // if (ttl > 1 && src == "10.0.0.1" && Simulator::Now ().GetSeconds () >= 96)
             //   {
                  // if (src == "10.0.0.1")
                  //   std::cout << Simulator::Now () << "|" << src << "|" << nodeid << "|" << packet_id << "|" << node_mob->GetPosition ().x << "|" << (int) ttl << "|" << (int) hops << std::endl;

              packetsSent[nodeid] = packetsSent[nodeid] + 1;

              NS_LOG_UNCOND("[DEVTXTRACE] " << packetsSent[nodeid]);

              if (nodeid == 0) {
            	  os_tlf << Simulator::Now () << "|" << nodeid << "|" << packet_id << "|" << node_mob->GetPosition ().x << "|" << (int) ttl << "|" << (int) hops << std::endl;
              }

            }
        }
    }
}

void
GpsrExample::ReceivePacket (Ptr<Socket> socket)
{
  uint32_t nodeid = socket->GetNode ()->GetId ();
  Ptr <Packet> packet;

  Address from;
  while (packet = socket->RecvFrom(from)) {
    if (InetSocketAddress::IsMatchingType (from)) {
      //InetSocketAddress address = InetSocketAddress::ConvertFrom (from);
      //std::cout << "Received " << packet->GetSize() << " bytes from " << address.GetIpv4() << std::enjYsTY6weQmdXjYsTY6weQmdXdl;
    //  if (address.GetIpv4() == "10.0.0.1")
        packetsRcvd[nodeid] = packetsRcvd[nodeid] + 1;
        NS_LOG_UNCOND ("[RECEIVEPACKET] " << nodeid << " received one packet.");
      // else
       // NS_LOG_UNCOND (nodeid << " received one packet.");
    }
  }
}

void
GpsrExample::SetupPacketReceive (Ipv4Address addr, Ptr <Node> node, uint16_t port)
{
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr <Socket> sink = Socket::CreateSocket (node, tid);
  InetSocketAddress local = InetSocketAddress (addr, port);
  sink->Bind (local);
  sink->SetRecvCallback (MakeCallback (&GpsrExample::ReceivePacket, this));
}

void
GpsrExample::SetupPacketSend (uint32_t nodeid)
{
  std::cout << "Node " << nodeid << " started transmitting!" << std::endl;
  uint16_t port = 9;
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), port);
  Ptr<Socket> source = Socket::CreateSocket (nodes.Get (nodeid), tid);
  source->SetAllowBroadcast (true);
  source->Connect (remote);

  OnOffHelper onoff ("ns3::UdpSocketFactory", Address (remote));
  onoff.SetAttribute ("OnTime", RandomVariableValue (ConstantVariable (1)));
  onoff.SetAttribute ("OffTime", RandomVariableValue (ConstantVariable (0)));
  onoff.SetAttribute ("PacketSize", UintegerValue (pkt_size));
  onoff.SetAttribute ("DataRate", DataRateValue (DataRate (data_rate)));

  ApplicationContainer app = onoff.Install (nodes.Get (nodeid));
  // app.Start (Simulator::Now()); // Accident time
  //app.Start (Seconds (96));
  //std::cout << double(96 + ((double)nodeid / nodes.GetN ())) << "|" << (double)nodeid << std::endl;
  app.Start (Seconds (2.0));
  app.Stop (Seconds (totalTime-0.1));
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
GpsrExample::GpsrExample () :
  size (30),
  totalTime (180),
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

void
GpsrExample::Run ()
{
//  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", UintegerValue (1)); // enable rts cts all the time.
  CreateNodes ();
  CreateDevices ();
  InstallInternetStack ();
  InstallApplications ();

  GpsrHelper gpsr;
  gpsr.Install ();

  coursechangelog = "/home/andre/workspace4/ns3-gpsr/logs/course_changes_" + runID_str + ".log";
  finalcoursechangelog = "/home/andre/workspace4/ns3-gpsr/final_course_changes_" + runID_str + ".log";
  txlogFile = "/home/andre/workspace4/ns3-gpsr/logs/tx_packets_" + runID_str + ".log";
  rxlogFile = "/home/andre/workspace4/ns3-gpsr/logs/rx_packets_" + runID_str + ".log";

  // Opening logs files for output
  //std::ofstream os_ccl;
  os_ccl.open (coursechangelog.c_str());
  os_fccl.open (finalcoursechangelog.c_str ());
  os_rlf.open (rxlogFile.c_str());
  os_tlf.open (txlogFile.c_str());

  // Connecting to trace sources
  Config::Connect ("/NodeList/*/$ns3::MobilityModel/CourseChange", MakeCallback (&GpsrExample::CourseChange, this));
  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxDrop", MakeCallback(&MacTxDrop));
  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxDrop", MakeCallback(&MacRxDrop));
  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRx", MakeCallback (&GpsrExample::WifiMacRxTrace, this));
  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTx", MakeCallback (&GpsrExample::DevTxTrace, this));

  //Config::Connect ("/NodeList/2/$ns3::Ipv4L3Protocol/Rx", MakeCallback (&GpsrExample::RxTrace, this));

  std::cout << "Starting simulation for " << totalTime << " s ...\n";

  Simulator::Stop (Seconds (totalTime));

  AnimationInterface anim ("/home/andre/workspace4/ns3-gpsr/logs/log.xml");
  FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

  Simulator::Run ();

  monitor->CheckForLostPackets ();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
  std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();

  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator iter = stats.begin (); iter != stats.end (); ++iter)
  {
	   Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (iter->first);
	  //
	  //        if ((t.sourceAddress == Ipv4Address("10.1.1.1") && t.destinationAddress == Ipv4Address("10.1.1.25"))
	  //      	|| (t.sourceAddress == Ipv4Address("10.1.1.11") && t.destinationAddress == Ipv4Address("10.1.1.15"))
	  //      	|| (t.sourceAddress == Ipv4Address("10.1.1.21") && t.destinationAddress == Ipv4Address("10.1.1.5")))
	  //{
	  //NS_LOG_UNCOND("Flow ID: " << iter->first << " Src Addr " << t.sourceAddress << " Dst Addr " << t.destinationAddress);
	  //NS_LOG_UNCOND("Tx Packets = " << iter->second.txPackets);
	  //NS_LOG_UNCOND("Rx Packets = " << iter->second.rxPackets);
	  //NS_LOG_UNCOND("Throughput: " << iter->second.rxBytes * 8.0 / (iter->second.timeLastRxPacket.GetSeconds()-iter->second.timeFirstTxPacket.GetSeconds()) / 1024  << " Kbps");
	  // 	  NS_LOG_UNCOND("Delay Sum:   " << i->second.delaySum << "\n");
	  // NS_LOG_UNCOND("Average Delay:   " << i->second.delaySum / i-> > >second.rxPackets<< "\n");
	  //}

	  //	<< iter->second.delayHistogram << "\n";
	  //    	    << iter->second.rxBytes * 8.0 / (iter->second.timeLastRxPacket.GetSeconds()-iter->second.timeFirstTxPacket.GetSeconds()) / 1024  << " Kbps" << "/t"
	  //    	    << iter->second.delaySum / iter->second.rxPackets << "\n";

	  NS_LOG_UNCOND(iter->first << ": " << t.sourceAddress << "->" << t.destinationAddress  << " transmitted " << iter->second.txPackets );
  }

  monitor->SerializeToXmlFile("lab-5.flowmon", true, true);

  Simulator::Destroy ();

  tx_rx_per_node = "/home/andre/workspace4/ns3-gpsr/logs/pdr_info_" + runID_str + ".log";
  os_pdr.open (tx_rx_per_node.c_str());

  uint32_t total_tx_frames = 0;
  uint32_t total_packets_send = 0;
  uint32_t total_packets_received = 0;

  for (uint32_t i = 0; i < nodes.GetN (); i++)
    {
      Ptr<Node> node = nodes.Get (i);
      uint32_t nodeId = node->GetId ();

      NS_LOG_UNCOND ("Node: " << nodeId << " ---> sent: " << packetsSent[i] << " ---> received: " << packetsRcvd[i] << " ---> txDrop: " << count_txDrop[i]<< " ---> rxDrop: " << count_rxDrop[i] );
      //NS_LOG_UNCOND ("Node: " << nodeId << " ---> sent: " << packetsSent[i] << " ---> received: " << packetsRcvd[i]);

      total_packets_send = total_packets_send + packetsSent[i];
      total_packets_received = total_packets_received + packetsRcvd[i];

      os_pdr << nodeId << "|" << packetsSent[i] << "|" << packetsRcvd[i] << "|" << total_tx_frames << std::endl;
      os_fccl << nodeId << "|" << last_node_position[i] << std::endl;

    }

  NS_LOG_UNCOND ("Total packets send: " << total_packets_send);
  NS_LOG_UNCOND ("Total packets received: " << total_packets_received);
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

  // Sets the mobility model on each node

  std::string traceFile = "/home/andre/workspace4/ns3-gpsr/high-1.1-out.tcl";
  //std::string traceFile = "/home/andre/workspace4/ns3-gpsr-l/low-1.1-out.tcl";
  Ns2MobilityHelper mobility = Ns2MobilityHelper (traceFile);
  mobility.Install (); // configure movements for each node, while reading trace file

//  MobilityHelper mobility;
//
//  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
//		  "MinX", DoubleValue (0.0),
//		  "MinY", DoubleValue (0.0),
//		  "DeltaX", DoubleValue (5.0),
//		  "DeltaY", DoubleValue (10.0),
//		  "GridWidth", UintegerValue (3),
//		  "LayoutType", StringValue ("RowFirst"));
//
//  mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel",
//		  "Bounds", RectangleValue (Rectangle (-50, 50, -50, 50)));
//  mobility.Install(nodes);

  // Set Mobility for staticNode
//  //TODO
//  MobilityHelper mobility ;
////  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
////  		  "MinX", DoubleValue (0.0),
////  		  "MinY", DoubleValue (0.0),
////  		  "DeltaX", DoubleValue (5.0),
////  		  "DeltaY", DoubleValue (10.0),
////  		  "GridWidth", UintegerValue (3),
////  		  "LayoutType", StringValue ("RowFirst"));
//
//  mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
//  //mobility.Install (nodes);
//
//  Ptr<ListPositionAllocator> positionAlloc_static_0 = CreateObject <ListPositionAllocator>();
//  positionAlloc_static_0 ->Add(Vector(100, 100, 0));
//  mobility.SetPositionAllocator(positionAlloc_static_0);
//  mobility.Install (nodes.Get(0));
//  Ptr <ConstantVelocityMobilityModel> myConstantVelocityMobilityModel_static_0 = nodes.Get(0)->GetObject<ConstantVelocityMobilityModel>();
//  myConstantVelocityMobilityModel_static_0->SetVelocity(Vector3D(50,0,0));
//
//  Ptr<ListPositionAllocator> positionAlloc_static_1 = CreateObject <ListPositionAllocator>();
//  positionAlloc_static_1 ->Add(Vector(200, 300, 0));
//  mobility.SetPositionAllocator(positionAlloc_static_1);
//  mobility.Install (nodes.Get(1));
//  Ptr <ConstantVelocityMobilityModel> myConstantVelocityMobilityModel_static_1 = nodes.Get(1)->GetObject<ConstantVelocityMobilityModel>();
//  myConstantVelocityMobilityModel_static_1->SetVelocity(Vector3D(46,0,0));
//
//  Ptr<ListPositionAllocator> positionAlloc_static_2 = CreateObject <ListPositionAllocator>();
//  positionAlloc_static_2 ->Add(Vector(300, 600, 0));
//  mobility.SetPositionAllocator(positionAlloc_static_2);
//  mobility.Install (nodes.Get(2));
//  Ptr <ConstantVelocityMobilityModel> myConstantVelocityMobilityModel_static_2 = nodes.Get(2)->GetObject<ConstantVelocityMobilityModel>();
//  myConstantVelocityMobilityModel_static_2->SetVelocity(Vector3D(80,0,0));
//
//  Ptr<ListPositionAllocator> positionAlloc_static_3 = CreateObject <ListPositionAllocator>();
//  positionAlloc_static_3 ->Add(Vector(400, 900, 0));
//  mobility.SetPositionAllocator(positionAlloc_static_3);
//  mobility.Install (nodes.Get(3));
//  Ptr <ConstantVelocityMobilityModel> myConstantVelocityMobilityModel_static_3 = nodes.Get(3)->GetObject<ConstantVelocityMobilityModel>();
//  myConstantVelocityMobilityModel_static_3->SetVelocity(Vector3D(60,0,0));
//
//  Ptr<ListPositionAllocator> positionAlloc_static_4 = CreateObject <ListPositionAllocator>();
//  positionAlloc_static_4 ->Add(Vector(500, 1200, 0));
//  mobility.SetPositionAllocator(positionAlloc_static_4);
//  mobility.Install (nodes.Get(4));
//  Ptr <ConstantVelocityMobilityModel> myConstantVelocityMobilityModel_static_4 = nodes.Get(4)->GetObject<ConstantVelocityMobilityModel>();
//  myConstantVelocityMobilityModel_static_4->SetVelocity(Vector3D(50,0,0));
//
//  Ptr<ListPositionAllocator> positionAlloc_static_5 = CreateObject <ListPositionAllocator>();
//  positionAlloc_static_5 ->Add(Vector(600, 1500, 0));
//  mobility.SetPositionAllocator(positionAlloc_static_5);
//  mobility.Install (nodes.Get(5));
//  Ptr <ConstantVelocityMobilityModel> myConstantVelocityMobilityModel_static_5 = nodes.Get(5)->GetObject<ConstantVelocityMobilityModel>();
//  myConstantVelocityMobilityModel_static_5->SetVelocity(Vector3D(35,0,0));
//
//  Ptr<ListPositionAllocator> positionAlloc_static_6 = CreateObject <ListPositionAllocator>();
//  positionAlloc_static_6 ->Add(Vector(700, 1800, 0));
//  mobility.SetPositionAllocator(positionAlloc_static_6);
//  mobility.Install (nodes.Get(6));
//  Ptr <ConstantVelocityMobilityModel> myConstantVelocityMobilityModel_static_6 = nodes.Get(6)->GetObject<ConstantVelocityMobilityModel>();
//  myConstantVelocityMobilityModel_static_6->SetVelocity(Vector3D(80,0,0));
//
//  Ptr<ListPositionAllocator> positionAlloc_static_7 = CreateObject <ListPositionAllocator>();
//  positionAlloc_static_7 ->Add(Vector(800, 2100, 0));
//  mobility.SetPositionAllocator(positionAlloc_static_7);
//  mobility.Install (nodes.Get(7));
//  Ptr <ConstantVelocityMobilityModel> myConstantVelocityMobilityModel_static_7 = nodes.Get(7)->GetObject<ConstantVelocityMobilityModel>();
//  myConstantVelocityMobilityModel_static_7->SetVelocity(Vector3D(52,0,0));
//
//  Ptr<ListPositionAllocator> positionAlloc_static_8 = CreateObject <ListPositionAllocator>();
//  positionAlloc_static_8 ->Add(Vector(900, 900, 0));
//  mobility.SetPositionAllocator(positionAlloc_static_8);
//  mobility.Install (nodes.Get(8));
//  Ptr <ConstantVelocityMobilityModel> myConstantVelocityMobilityModel_static_8 = nodes.Get(8)->GetObject<ConstantVelocityMobilityModel>();
//  myConstantVelocityMobilityModel_static_8->SetVelocity(Vector3D(20,0,0));
//
//  Ptr<ListPositionAllocator> positionAlloc_static_9 = CreateObject <ListPositionAllocator>();
//  positionAlloc_static_9 ->Add(Vector(755, 2300, 0));
//  mobility.SetPositionAllocator(positionAlloc_static_9);
//  mobility.Install (nodes.Get(9));
//  Ptr <ConstantVelocityMobilityModel> myConstantVelocityMobilityModel_static_9 = nodes.Get(9)->GetObject<ConstantVelocityMobilityModel>();
//  myConstantVelocityMobilityModel_static_9->SetVelocity(Vector3D(50,0,0));


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
        if (node->GetId () == 1 || node->GetId () == 2 || node->GetId () == 3)
          {
            rsu_nodes.Add (node);
             std::cout << "Node " << node->GetId () << " added to RSU's" << std::endl;
          }
        else {
          if (node->GetId () != 0) {
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
  crashed_device = wifi.Install (wifiPhy, wifiMac, nodes.Get (0));


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
  else
    {
      static_devices = NetDeviceContainer (crashed_device);
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
  //AodvHelper aodv;
  // you can configure AODV attributes here using aodv.Set(name, value)
  InternetStackHelper stack;
  stack.SetRoutingHelper (gpsr);
  stack.Install (nodes);
  Ipv4AddressHelper address;
  address.SetBase ("10.0.0.0", "255.0.0.0");
  interfaces = address.Assign (devices);

  for (uint32_t n = 0; n < nodes.GetN (); n++) {
    Ptr<Node> rsu = nodes.Get (n);
    Ptr<Ipv4> ipv4 = rsu->GetObject<Ipv4>();
    Ipv4InterfaceAddress iaddr = ipv4->GetAddress (1,0);
    Ipv4Address addri = iaddr.GetLocal ();
    std::cout << "RSU (" << rsu->GetId () << ") address: " << addri << std::endl;
  }
}


void
GpsrExample::InstallApplications ()
{
//	TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
//	uint32_t nodeid;
//	uint16_t port = 9;
//	InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), port);
//
//	for (nodeid = 1; nodeid < nodes.GetN (); nodeid++) {
//		SetupPacketReceive (interfaces.GetAddress (nodeid), nodes.Get (nodeid), port);
//	}
//
//	for (nodeid = 4; nodeid < nodes.GetN (); nodeid++) {
//		//SetupPacketReceive (interfaces.GetAddress (nodeid), nodes.Get (nodeid), port);
//		Ptr<Socket> source = Socket::CreateSocket (nodes.Get (nodeid), tid);
//		source->SetAllowBroadcast (true);
//		source->Connect (remote);
//		SetupPacketSend (nodeid);
//	}
//
//
//	Ptr<Socket> source = Socket::CreateSocket (nodes.Get (0), tid);
//	source->SetAllowBroadcast (true);
//	source->Connect (remote);
//
//	//----------------------------------------------------//
//	// Create the OnOff application to send UDP datagrams //
//	//----------------------------------------------------//
//
//	OnOffHelper onoff ("ns3::UdpSocketFactory", Address (remote));
//	onoff.SetAttribute ("OnTime", RandomVariableValue (ConstantVariable (1)));
//	onoff.SetAttribute ("OffTime", RandomVariableValue (ConstantVariable (0)));
//	onoff.SetAttribute ("PacketSize", UintegerValue (pkt_size));
//	onoff.SetAttribute ("DataRate", DataRateValue (DataRate (data_rate)));
//	// onoff.SetAttribute ("MaxBytes", UintegerValue (pkt_size));
//
//	ApplicationContainer app = onoff.Install (nodes.Get (0));
//
//	app.Start (Seconds (2.0));
//	app.Stop (Seconds (totalTime-0.1));

	uint32_t nodeid;
	uint16_t port = 9;  // well-known echo port number
	uint32_t packetSize = 1024; // size of the packets being transmitted
	uint32_t maxPacketCount = 100; // number of packets to transmit
	Time interPacketInterval = Seconds (1.); // interval between packet transmissions

	for (nodeid = 1; nodeid < nodes.GetN (); nodeid++) {
		SetupPacketReceive (interfaces.GetAddress (nodeid), nodes.Get (nodeid), port);
	}

	// Set-up a server Application on the bottom-right node of the grid
	UdpEchoServerHelper server1 (port);
	uint16_t server1Position = 8; //bottom right
	ApplicationContainer apps = server1.Install (nodes.Get(server1Position));
	apps.Start (Seconds (1.0));
	apps.Stop (Seconds (totalTime-0.1));

	// Set-up a client Application, connected to 'server', to be run on the top-left node of the grid
	UdpEchoClientHelper client (interfaces.GetAddress (server1Position), port);
	client.SetAttribute ("MaxPackets", UintegerValue (maxPacketCount));
	client.SetAttribute ("Interval", TimeValue (interPacketInterval));
	client.SetAttribute ("PacketSize", UintegerValue (packetSize));
	uint16_t clientPosition = 4;
	apps = client.Install (nodes.Get (clientPosition));
	apps.Start (Seconds (2.0));
	apps.Stop (Seconds (totalTime-0.1));

}

/*************************************************************************************
 *  LOGS
 * **********************************************************************************/

//Criação de um ficheiro de log sempre que é recebida uma msg GPSR com o seu conteúdo



