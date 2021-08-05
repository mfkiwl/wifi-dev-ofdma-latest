#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/string.h"
#include "ns3/pointer.h"
#include "ns3/log.h"
#include "ns3/spectrum-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/mobility-helper.h"
#include "ns3/wifi-net-device.h"
#include "ns3/sta-wifi-mac.h"
#include "ns3/ap-wifi-mac.h"
#include "ns3/qos-txop.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/application-container.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/on-off-helper.h"
#include "ns3/v4ping-helper.h"
#include "ns3/wifi-acknowledgment.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/wifi-mac-queue.h"
#include "ns3/wifi-psdu.h"
#include "ns3/ctrl-headers.h"
#include "ns3/traffic-control-helper.h"
#include "ns3/he-phy.h"
#include <vector>
#include <map>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <numeric>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("MyExample");

class MyExample
{
public:
  
  MyExample ();

  void Setup (void);
  void Run (void);

  void StartAssociation (void);
  void EstablishBaAgreement (Mac48Address bssid);
  void StartOnOffClient (OnOffHelper client);
  void StartTraffic (void);

  void PlaceHooks (void);
  void RemoveHooks (void);

  void NotifyTxFailed (WifiMacDropReason reason, Ptr<const WifiMacQueueItem> mpdu);
  void NotifyMacDropped (std::string context, Ptr< const Packet > packet);

  void NotifyAPMacTxDropped (Ptr< const Packet > packet);
  void NotifyAPPhyTxDropped (Ptr< const Packet > psdu);
  void NotifyPhyDropped (std::string context, Ptr< const Packet > packet, WifiPhyRxfailureReason reason);
  void NotifyTxNAcked (Ptr<const WifiMacQueueItem> mpdu);

  uint32_t ContextToNodeId (const std::string & context);

private:
  uint32_t m_payloadSize;  
  double m_simulationTime;
  uint16_t m_nStations;
  uint16_t m_channelCenterFrequency;
  double appDataRate;        
  std::size_t m_currentSta;
  Ssid m_ssid;
  NodeContainer m_apNodes;
  NodeContainer m_staNodes;
  NetDeviceContainer m_staDevices;
  NetDeviceContainer m_apDevices;
  Ipv4InterfaceContainer m_staInterfaces;
  ApplicationContainer m_sinkApps;
  ApplicationContainer m_OnOffApps;
  uint16_t m_port;
  std::vector<uint64_t> m_rxStart, m_rxStop;
  uint32_t macRxDrop;
  uint64_t phyRxDrop;
  std::vector<uint64_t> phyDropReason;
  uint64_t macApTxDrop;
  uint64_t phyApTxDrop;
  std::map <uint32_t, std::vector<uint64_t>> m_phyRxDropMap;

  struct Stats
  {
    uint64_t failed {0};
    uint64_t nacked{0};
  };
  std::map<Mac48Address, Stats> m_Stats;

};

MyExample::MyExample ()
  : m_payloadSize (1000),
    m_simulationTime (4),
    m_nStations (4),
    m_channelCenterFrequency (0),
    m_currentSta (0),
    m_ssid (Ssid ("network-A")),
    m_port (7000),
    macRxDrop(0),
    phyRxDrop(0),
    macApTxDrop(0),
    phyApTxDrop(0)
{
}

void
MyExample::Setup (void)
{
  
  uint64_t phyRate = HePhy::GetHeMcs (7).GetDataRate (20, 800, 1);
  appDataRate = phyRate * 1.2 / 1e6 / m_nStations;
  uint32_t queueSize = 1000;

  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("999999"));
  Config::SetDefault ("ns3::HeConfiguration::GuardInterval", TimeValue (NanoSeconds (800)));
  Config::SetDefault ("ns3::ArpCache::AliveTimeout", TimeValue (Seconds (3600 * 24))); // ARP cache entries expire after one day
  Config::SetDefault ("ns3::WifiMacQueue::MaxSize", QueueSizeValue (QueueSize (PACKETS, queueSize)));
  Config::SetDefault ("ns3::WifiMacQueue::MaxDelay", TimeValue (MilliSeconds (m_simulationTime * 1000.))); // MSDUs never expire in the MAC Queue
  Config::SetDefault ("ns3::HeConfiguration::MpduBufferSize", UintegerValue (64));

  m_staNodes.Create (m_nStations);
  m_apNodes.Create (1);

  Ptr<MultiModelSpectrumChannel> spectrumChannel = CreateObject<MultiModelSpectrumChannel> ();
  SpectrumWifiPhyHelper phy;
  phy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);
  phy.SetChannel (spectrumChannel);
  phy.Set ("ChannelNumber", UintegerValue (36));
  phy.Set ("ChannelWidth", UintegerValue (20));

  WifiHelper wifi;

  wifi.SetStandard (WIFI_STANDARD_80211ax_5GHZ);

  std::ostringstream oss;
  oss << "HeMcs7";
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode", StringValue (oss.str ()),
                                "ControlMode", StringValue (oss.str ()));

  Config::SetDefault ("ns3::WifiDefaultAckManager::DlMuAckSequenceType", EnumValue (WifiAcknowledgment::DL_MU_AGGREGATE_TF));

  WifiMacHelper mac;

  mac.SetMultiUserScheduler ("ns3::RrMultiUserScheduler",
                                "NStations", UintegerValue(4),
                                "ForceDlOfdma", BooleanValue (false),
                                "EnableUlOfdma", BooleanValue (false),
                                "UlPsduSize", UintegerValue (1000),
                                "EnableBsrp", BooleanValue(false),
                                "UseCentral26TonesRus", BooleanValue(false));

  mac.SetType ("ns3::StaWifiMac",  "Ssid", SsidValue (Ssid ("non-existing-ssid")));  // prevent stations from automatically associating
  m_staDevices = wifi.Install (phy, mac, m_staNodes);

  mac.SetType ("ns3::ApWifiMac", "Ssid", SsidValue (m_ssid));
  m_apDevices = wifi.Install (phy, mac, m_apNodes);

  Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice> (m_apDevices.Get (0)); 
  dev->GetMac ()->SetAttribute ("BE_MaxAmsduSize", UintegerValue (0));
  dev->GetMac ()->SetAttribute ("BE_MaxAmpduSize", UintegerValue (256));
  
  m_channelCenterFrequency = dev->GetPhy ()->GetFrequency ();
  
  PointerValue ptr;
  dev->GetMac ()->GetAttribute ("BE_Txop", ptr);
  ptr.Get<QosTxop> ()->SetTxopLimit (MicroSeconds (5440));

  // Configure max A-MSDU size and max A-MPDU size on the stations
  for (uint32_t i = 0; i < m_staNodes.GetN (); i++)
    {
      dev = DynamicCast<WifiNetDevice> (m_staDevices.Get (i));
      dev->GetMac ()->SetAttribute ("BE_MaxAmsduSize", UintegerValue (0));
      dev->GetMac ()->SetAttribute ("BE_MaxAmpduSize", UintegerValue (256));
      m_Stats[dev->GetMac ()->GetAddress ()] = Stats ();
    }

  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));  // position of the AP
  mobility.SetPositionAllocator (positionAlloc);
  mobility.Install (m_apNodes);
 
  mobility.SetPositionAllocator ("ns3::UniformDiscPositionAllocator",
                                 "rho", DoubleValue (5));
  mobility.Install (m_staNodes);

  InternetStackHelper stack;
  stack.Install (m_apNodes);
  stack.Install (m_staNodes);

  Ipv4AddressHelper address;
  address.SetBase ("192.168.1.0", "255.255.255.0");
  Ipv4InterfaceContainer ApInterface;
  ApInterface = address.Assign (m_apDevices);
  m_staInterfaces = address.Assign (m_staDevices);

  TrafficControlHelper tch;

  PacketSinkHelper packetSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), m_port));
  m_sinkApps = packetSinkHelper.Install (m_staNodes);
  m_sinkApps.Stop (Seconds (2.0 + m_simulationTime + 100));

  m_rxStart.assign (m_nStations, 0.0);
  phyDropReason.assign(18, 0);
  m_rxStop.assign (m_nStations, 0.0);
  
  for (uint16_t i = 0; i < m_nStations; i++) {

    m_phyRxDropMap.insert(std::make_pair (i, std::vector<uint64_t> ()));
    auto it = m_phyRxDropMap.find(i);
    NS_ASSERT(it != m_phyRxDropMap.end());
    it->second.assign(18, 0);
  }

  Config::ConnectWithoutContext ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::StaWifiMac/Assoc",
                                 MakeCallback (&MyExample::EstablishBaAgreement, this));

}

void
MyExample::Run (void)
{
  NS_LOG_FUNCTION (this);

  Simulator::ScheduleNow (&MyExample::StartAssociation, this);

  Simulator::Stop (Seconds (2.0 + m_simulationTime + 100));
  Simulator::Run ();

  double totalTput = 0.0;
  double tput;
  std::cout << "Throughput (Mbps)" << std::endl
            << "-----------------" << std::endl;
  for (uint32_t i = 0; i < m_staNodes.GetN (); i++)
    {
      tput = ((m_rxStop[i] - m_rxStart[i]) * 8.) / (m_simulationTime * 1e6);
      totalTput += tput;
      std::cout << "STA_" << i << ": " << tput << "  ";
      std::cout<< "total Bytes tx:"<< m_rxStart[i] <<"   ";
      std::cout<< "total Bytes rx:"<<m_rxStop[i]<<"\n";
    }
  std::cout << std::endl << std::endl << "Total throughput: " << totalTput << std::endl;

  uint64_t totalFailed = 0;
  uint64_t failed;
  std::cout << std::endl << "Dropped MPDU (in AP Queue)" << std::endl
                         << "-----------" << std::endl;
  for (uint32_t i = 0; i < m_staNodes.GetN (); i++)
    {
      auto it = m_Stats.find (DynamicCast<WifiNetDevice> (m_staDevices.Get (i))->GetMac ()->GetAddress ());
      NS_ASSERT (it != m_Stats.end ());
      failed = it->second.failed;
      totalFailed += failed;
      std::cout << "STA_" << i << ": " << failed << " ";
    }
  std::cout << std::endl << std::endl << "Total MPDU dropped (in AP Queue): " << totalFailed << std::endl;

  uint64_t totalNAcked = 0;
  uint64_t nacked;
  std::cout << std::endl << "Negatively Acknowledged" << std::endl
                         << "-----------" << std::endl;
  for (uint32_t i = 0; i < m_staNodes.GetN (); i++)
    {
      auto it = m_Stats.find (DynamicCast<WifiNetDevice> (m_staDevices.Get (i))->GetMac ()->GetAddress ());
      NS_ASSERT (it != m_Stats.end ());
      nacked = it->second.nacked;
      totalNAcked += nacked;
      std::cout << "STA_" << i << ": " << nacked << " ";
    }
  std::cout << std::endl << std::endl << "Total Negatively Acknowledged: " << totalNAcked << std::endl;

  std::cout << std::endl << std::endl << "Total MAC layer drops: " << macRxDrop << std::endl;

  std::cout << std::endl << "STA PHY drops with reason" << std::endl
                         << "-----------" << std::endl;
  for (uint32_t j = 0; j < m_staNodes.GetN (); j++) {

    auto it = m_phyRxDropMap.find(j);
    NS_ASSERT (it != m_phyRxDropMap.end());

    std::cout << "\nSTA_" << j << "\n";
    for (uint32_t i = 0; i < 18; i++) {
      if ( i == 0 ) {
        std::cout << "UNKNOWN " << it->second[i] << " ";
      }
      else if ( i == 1 ) {
        std::cout << "UNSUPPORTED_SETTINGS " << it->second[i] << " ";
      }
      else if ( i == 2 ) {
        std::cout << "CHANNEL_SWITCHING " << it->second[i] << " ";
      }
      else if ( i == 3 ) {
        std::cout << "RXING " << it->second[i] << " ";
      }
      else if ( i == 4 ) {
        std::cout << "TXING " << it->second[i] << " "; // 73827
      }
      else if ( i == 5 ) {
        std::cout << "SLEEPING " << it->second[i] << " ";
      }
      else if ( i == 6 ) {
        std::cout << "BUSY_DECODING_PREAMBLE " << it->second[i] << " "; // 101
      }
      else if ( i == 7 ) {
        std::cout << "PREAMBLE_DETECT_FAILURE " << it->second[i] << " "; // 2066
      }
      else if ( i == 8 ) {
        std::cout << "RECEPTION_ABORTED_BY_TX " << it->second[i] << " ";
      }
      else if ( i == 9 ) {
        std::cout << "L_SIG_FAILURE " << it->second[i] << " ";
      }
      else if ( i == 10 ) {
        std::cout << "HT_SIG_FAILURE " << it->second[i] << " ";
      }
      else if ( i == 11 ) {
        std::cout << "SIG_A_FAILURE " << it->second[i] << " ";
      }
      else if ( i == 12 ) {
        std::cout << "SIG_B_FAILURE " << it->second[i] << " ";
      }
      else if ( i == 13 ) {
        std::cout << "PREAMBLE_DETECTION_PACKET_SWITCH " << it->second[i] << " ";
      }
      else if ( i == 14 ) {
        std::cout << "FRAME_CAPTURE_PACKET_SWITCH " << it->second[i] << " ";
      }
      else if ( i == 15 ) {
        std::cout << "OBSS_PD_CCA_RESET " << it->second[i] << " ";
      }
      else if ( i == 16 ) {
        std::cout << "HE_TB_PPDU_TOO_LATE " << it->second[i] << " ";
      }
      else if ( i == 17 ) {
        std::cout << "FILTERED " << it->second[i] << " ";
      }
    }
  }

  std::cout << std::endl << std::endl << "Total STA PHY layer drops: " << phyRxDrop << std::endl;

  std::cout << std::endl << std::endl << "Total AP MAC Tx Drops: " << macApTxDrop << std::endl;

  std::cout << std::endl << std::endl << "Total AP PHY Tx Drops: " << phyApTxDrop << std::endl;

  m_phyRxDropMap.clear();

  Simulator::Destroy ();
}

void
MyExample::StartAssociation (void)
{
  NS_LOG_FUNCTION (this << m_currentSta);
  NS_ASSERT (m_currentSta < m_nStations);

  std::cout << "STA " << m_currentSta << " associated with the AP.\n";
  Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice> (m_staDevices.Get (m_currentSta));
  NS_ASSERT (dev != 0);
  dev->GetMac ()->SetSsid (m_ssid); 
}

void
MyExample::EstablishBaAgreement (Mac48Address bssid)
{
  Time pingDuration = MilliSeconds (125);

  V4PingHelper ping (m_staInterfaces.GetAddress (m_currentSta));
  ping.SetAttribute ("Interval", TimeValue (MilliSeconds (50)));
  
  ApplicationContainer pingApps = ping.Install (m_apNodes);
  pingApps.Stop (pingDuration);

  uint16_t offInterval = 10;
  std::stringstream ss;
  ss << "ns3::ConstantRandomVariable[Constant=" << std::fixed << static_cast<double> (offInterval / 1000.) << "]";

  if ( m_currentSta < m_nStations ) {

    OnOffHelper client ("ns3::UdpSocketFactory", Ipv4Address::GetAny ());
    client.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
    client.SetAttribute ("OffTime", StringValue (ss.str ()));
    client.SetAttribute ("DataRate", DataRateValue (DataRate (appDataRate * 1e6))); // Saturate the channel
    client.SetAttribute ("PacketSize", UintegerValue (m_payloadSize));


    InetSocketAddress dest (m_staInterfaces.GetAddress (m_currentSta), m_port);
    client.SetAttribute ("Remote", AddressValue (dest));
    uint64_t startTime = std::ceil (Simulator::Now ().ToDouble (Time::MS) / offInterval) * offInterval;

    Simulator::Schedule (MilliSeconds (static_cast<uint64_t> (startTime) + 110) - Simulator::Now (),
                       &MyExample::StartOnOffClient, this, client);

  }   

  if (++m_currentSta < m_nStations) {
    Simulator::Schedule (pingDuration, &MyExample::StartAssociation, this);
  }
  else {
    Simulator::Schedule (pingDuration, &MyExample::StartTraffic, this);
  }
}

void
MyExample::StartOnOffClient (OnOffHelper client)
{
  m_OnOffApps.Add (client.Install (m_apNodes));
  m_OnOffApps.Stop (Seconds (2.0 + m_simulationTime + 100));
}

void
MyExample::StartTraffic (void)
{
  for (uint32_t i = 0; i < m_nStations; i++) {
      
    Ptr<Application> clientApp = m_OnOffApps.Get (i);
      
    clientApp->SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
    clientApp->SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));    
  }

  Simulator::Schedule (Seconds (2.0), &MyExample::PlaceHooks, this);
}

void
MyExample::PlaceHooks (void)
{
  NS_LOG_FUNCTION (this);
  Simulator::Schedule (Seconds (m_simulationTime), &MyExample::RemoveHooks, this);

  std::cout << "============== HOOK PLACED ============== \n";

  Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice> (m_apDevices.Get (0));
  
  DynamicCast<RegularWifiMac> (dev->GetMac ())->TraceConnectWithoutContext ("DroppedMpdu", MakeCallback (&MyExample::NotifyTxFailed, this));
  DynamicCast<RegularWifiMac> (dev->GetMac ())->TraceConnectWithoutContext("NAckedMpdu", MakeCallback (&MyExample::NotifyTxNAcked, this));
   
  DynamicCast<RegularWifiMac> (dev->GetMac ())->TraceConnectWithoutContext("MacTxDrop", MakeCallback (&MyExample::NotifyAPMacTxDropped, this));
  DynamicCast<WifiPhy> (dev->GetPhy())->TraceConnectWithoutContext("PhyTxDrop", MakeCallback (&MyExample::NotifyAPPhyTxDropped, this));

  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::WifiMac/MacRxDrop", MakeCallback (&MyExample::NotifyMacDropped, this));
  
  for (uint32_t i = 0; i < m_staNodes.GetN (); i++)
    {
      m_rxStart[i] = DynamicCast<PacketSink> (m_sinkApps.Get (i))->GetTotalRx ();

      Ptr<WifiNetDevice> staDev = DynamicCast<WifiNetDevice> (m_staDevices.Get (i));
      
      std::ostringstream oss;
      oss << "/NodeList/" << i << "/DeviceList/" << i << "/";
      DynamicCast<WifiPhy> (staDev->GetPhy())->TraceConnect("PhyRxDrop", oss.str(), MakeCallback (&MyExample::NotifyPhyDropped, this));
      
    }
}

void
MyExample::RemoveHooks (void)
{
  NS_LOG_FUNCTION (this);

  std::cout << "============== HOOK REMOVED ============== \n";

  Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice> (m_apDevices.Get (0));

  DynamicCast<RegularWifiMac> (dev->GetMac ())->TraceDisconnectWithoutContext ("DroppedMpdu", MakeCallback (&MyExample::NotifyTxFailed, this));
  DynamicCast<RegularWifiMac> (dev->GetMac ())->TraceDisconnectWithoutContext("MacTxDrop", MakeCallback (&MyExample::NotifyAPMacTxDropped, this));
  DynamicCast<WifiPhy> (dev->GetPhy())->TraceDisconnectWithoutContext("PhyTxDrop", MakeCallback (&MyExample::NotifyAPPhyTxDropped, this));

  Config::Disconnect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::WifiMac/MacRxDrop", MakeCallback (&MyExample::NotifyMacDropped, this));
  for (uint32_t i = 0; i < m_staNodes.GetN (); i++) {
      m_rxStop[i] = DynamicCast<PacketSink> (m_sinkApps.Get (i))->GetTotalRx ();

      Ptr<WifiNetDevice> staDev = DynamicCast<WifiNetDevice> (m_staDevices.Get (i));

      std::ostringstream oss;
      oss << "/NodeList/" << i << "/DeviceList/" << i << "/";
      DynamicCast<WifiPhy> (staDev->GetPhy())->TraceDisconnect("PhyRxDrop", oss.str(), MakeCallback (&MyExample::NotifyPhyDropped, this));
    }

  for (uint32_t i = 0; i < m_OnOffApps.GetN (); i++)
    {
      m_OnOffApps.Get (i)->Dispose ();
    }
}

void
MyExample::NotifyTxFailed (WifiMacDropReason reason, Ptr<const WifiMacQueueItem> mpdu)
{
  WifiMacHeader hdr = mpdu->GetHeader();
  auto it = m_Stats.find (hdr.GetAddr1 ());
  NS_ASSERT (it != m_Stats.end ());
  it->second.failed++;

  for (uint32_t i = 0; i < m_staNodes.GetN (); i++)
  {
    if ( DynamicCast<WifiNetDevice> (m_staDevices.Get (i))->GetMac ()->GetAddress () == hdr.GetAddr1() ) {
      
        std::cout << "====== MPDU DROPPED FOR STA " << i << " for reason: " << reason << " ===========\n";
        break;
    }
  }
}

void
MyExample::NotifyMacDropped(std::string context, Ptr< const Packet > packet) {

    macRxDrop++;
}

void
MyExample::NotifyPhyDropped(std::string context, Ptr< const Packet > packet, WifiPhyRxfailureReason reason) {
  
  phyRxDrop++;
  phyDropReason[reason]++;

  auto itStaDropMap = m_phyRxDropMap.find (ContextToNodeId (context));
  NS_ASSERT (itStaDropMap != m_phyRxDropMap.end ());
  itStaDropMap->second[reason]++;
}

void 
MyExample::NotifyAPMacTxDropped(Ptr< const Packet > packet) {

    macApTxDrop++;
}

void 
MyExample::NotifyAPPhyTxDropped(Ptr< const Packet > psdu) {

    phyApTxDrop++;
}

void
MyExample::NotifyTxNAcked (Ptr<const WifiMacQueueItem> mpdu)
{
  WifiMacHeader hdr = mpdu->GetHeader();
  auto it = m_Stats.find (hdr.GetAddr1 ());
  NS_ASSERT (it != m_Stats.end ());
  it->second.nacked++;
}

uint32_t
MyExample::ContextToNodeId (const std::string & context)
{
  std::string sub = context.substr (10);
  uint32_t pos = sub.find ("/Device");
  uint32_t nodeId = atoi (sub.substr (0, pos).c_str ());
  return nodeId;
}

int main (int argc, char *argv[])
{
  MyExample example;
  example.Setup ();
  example.Run ();

  return 0;
}
