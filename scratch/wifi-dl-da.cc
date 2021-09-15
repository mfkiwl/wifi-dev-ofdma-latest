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
#include "ns3/on-demand-helper.h"
#include "ns3/v4ping-helper.h"
#include "ns3/wifi-acknowledgment.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/he-frame-exchange-manager.h"
#include "ns3/da-multi-user-scheduler.h"
#include "ns3/drr-multi-user-scheduler.h"
#include "ns3/ht-frame-exchange-manager.h"
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

//./waf --run "wifi-dl-da --nStations=3 --warmup=2 --simulationTime=2 --dlAckType=3 --channelWidth=40 --mcs=11 --radius=5 --scheduler=1 --saturateChannel=false --dataRate=1.5"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("WifiDlOfdma");

class WifiDlOfdma
{
public:
  /**
   * Create an example instance.
   */
  WifiDlOfdma ();

  /**
   * Parse the options provided through command line.
   */
  void Config (int argc, char *argv[]);

  /**
   * Setup nodes, devices and internet stacks.
   */
  void Setup (void);

  /**
   * Run simulation and print results.
   */
  void Run (void);
  /**
   * Make the current station associate with the AP.
   */
  void StartAssociation (void);

  /**
   * Callback for when an AP establishises a BA agreement with the current station.
   */
  void EstablishBaAgreement (Mac48Address bssid);

  /**
   * Install On Off Client application. (Doesn't start sending traffic yet)
   */
  void InstallOnOffClient (OnOffHelper client);

  /**
   * Install On Demand Client application. (Doesn't start sending traffic yet)
   */
  void InstallOnDemandClient (OnDemandHelper client);

  /**
   * Start generating traffic for On Off Applications
   */
  void StartTraffic (void);

  /**
   * Start collecting statistics.
   */
  void StartStatistics (void);

  /**
   * Stop collecting statistics.
   */
  void StopStatistics (void);

  /**
   * Report that an MPDU was not correctly received.
   */
  void NotifyApDroppedMpdu (WifiMacDropReason reason, Ptr<const WifiMacQueueItem> mpdu);

  /**
   * Report that an MPDU was dropped upon reception by this particular station
   */
  void NotifyStaDroppedMpdu (std::string context, WifiMacDropReason reason, Ptr<const WifiMacQueueItem> mpdu);

  /**
   * Report that an MPDU was dropped upon reception by some station
   */
  void NotifyMacRxDropped (std::string context, Ptr< const Packet > packet);

  /**
   * Report that an MPDU was dropped before transmission by the AP (before being queued into the MAC)
   */
  void NotifyAPMacTxDropped (Ptr< const Packet > packet);

  /**
   * Report that an PPDU was dropped during transmission by the AP
   */
  void NotifyAPPhyTxDropped (Ptr< const Packet > psdu);

  /**
   * Report that an PPDU was dropped upon reception by some station
   */
  void NotifyPhyRxDropped (std::string context, Ptr< const Packet > packet, WifiPhyRxfailureReason reason);

  /**
   * Report that an MPDU was negatively acknowledged.
   */  
  void NotifyTxNAcked (Ptr<const WifiMacQueueItem> mpdu);
  
  /**
   * Report that PSDUs were forwarded down to the PHY.
   */
  void NotifyPsduForwardedDown (Ptr<const WifiPsdu> psdu, WifiTxVector txVector);

  /**
   * Report that MU PSDU map were forwarded down to the PHY.
   */
  void NotifyPsduMapForwardedDown (WifiConstPsduMap psduMap, WifiTxVector txVector);
  
  /**
   * Report that the application has received a new packet. (App layer)
   */
  void NotifyApplicationTx (std::string context, Ptr<const Packet> p, const Address &add1, const Address &add2, const SeqTsSizeHeader &tsheader);
  
  /**
   * Report that the application has received a new packet. (App layer)
   */
  void NotifyApplicationRx (std::string context, Ptr<const Packet> p, const Address &add1, const Address &add2, const SeqTsSizeHeader &tsheader);
  
  /**
   * Parse context strings of the form "/NodeList/x/ApplicationList/y/" to extract the NodeId
   */
  uint32_t AppContextToNodeId (const std::string & context);

  /**
   * Report that the application has created and sent a new packet. (MAC layer)
   */
  void NotifyMacTx (Ptr<const Packet> p);

  /**
   * Report that the application has received a new packet. (MAC layer)
   */
  void NotifyMacRx (std::string context, Ptr<const Packet> p);

  /**
   * Parse context strings of the form "/NodeList/x/DeviceList/y/" to extract the NodeId
   */
  uint32_t DeviceContextToNodeId (const std::string & context);

private:
  uint32_t m_payloadSize;   // bytes
  uint32_t m_ulPsduSize; // Bytes
  double m_simulationTime;  // seconds
  uint32_t m_scheduler;
  bool m_saturateChannel;
  uint16_t m_nStations;     // not including AP //The type of this must be changes to uint8_t when using PF, and make sure when you do that the stations are not too many
  double m_radius;          // meters
  bool m_enableDlOfdma;
  bool m_enableUlOfdma;
  uint16_t m_channelWidth;  // channel bandwidth
  uint8_t m_channelNumber;
  uint16_t m_channelCenterFrequency;
  uint16_t m_guardInterval; // GI in nanoseconds
  uint8_t m_maxNRus;        // max number of RUs per MU PPDU
  uint32_t m_mcs;           // MCS value
  uint16_t m_maxAmsduSize;  // maximum A-MSDU size
  uint32_t m_maxAmpduSize;  // maximum A-MSDU size
  double m_txopLimit;       // microseconds
  uint32_t m_macQueueSize;  // packets
  uint32_t m_msduLifetime;  // milliseconds
  double m_dataRate;        // Mb/s
  uint16_t m_dlAckSeqType;
  bool m_continueTxop;
  uint16_t m_baBufferSize;
  std::string m_transport;
  std::string m_queueDisc;
  double m_warmup;          // duration of the warmup period (seconds)
  std::size_t m_currentSta; // index of the current station
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
  double m_minAmpduRatio;
  double m_maxAmpduRatio;
  double m_avgAmpduRatio;
  uint64_t m_nAmpduRatioSamples;
  uint32_t macRxDrop;
  uint64_t phyRxDrop;
  uint32_t m_maxQueueSizeReached; // Indicates whether the MAC queue was filled up during the simulation
  std::vector<uint64_t> phyDropReason;
  uint64_t macApTxDrop;
  uint64_t phyApTxDrop;
  Ptr<UniformRandomVariable> m_randomVar;
  bool m_randomizePacketSize;
  uint32_t m_minSampleRange;
  uint32_t m_maxSampleRange;
  std::map <uint64_t /* uid */, Time /* start */> m_macPacketTxMap; // Map for MAC layer packets
  std::map <uint32_t /* nodeId */, std::vector<Time>  /* array of latencies */> m_macLatencyMap;
  std::map <uint32_t /* nodeId */, std::vector<Time>  /* array of latencies */> m_appLatencyMap;
  std::map <uint32_t /* nodeId */, std::vector<uint64_t> /* array of drop with reasons */> m_phyRxDropMap;
  std::map <uint32_t /* nodeId */, std::vector<uint64_t> /* array of drop with reasons */> m_staMacDropMap;
  std::vector <std::vector<uint32_t>> m_globalSchedule;
  std::map <uint32_t /* nodeId */, std::vector<uint32_t> /* Array of Packet Time period and Deadline */> m_staPacketInfo; 

  // Map for App layer packets corresponding to each station, 
  // this is necessary to avoid Seq no. collisions between packets of different STAs
  std::map<uint32_t /* nodeId */, std::map <uint32_t /* Seq no. */, Time /* start */>> m_appPacketTxMap;

  struct DlStats
  {
    uint64_t droppedAtAp {0};
    uint64_t nacked{0};
    uint64_t droppedOnReceive{0};
    uint32_t minAmpduSize {0};
    uint32_t maxAmpduSize {0};
    uint64_t nAmpdus {0};
    double minAmpduRatio {0.0};
    double maxAmpduRatio {0.0};
    double avgAmpduRatio {0.0};
    uint64_t nAmpduRatioSamples {0};
  };
  std::map<Mac48Address, DlStats> m_dlStats;

  uint128_t lastAid = 0;
  std::map<Mac48Address, u_int16_t> m_aidMap;
};

WifiDlOfdma::WifiDlOfdma ()
  : m_payloadSize (30),
    m_ulPsduSize(0),
    m_simulationTime (5),
    m_scheduler(0),
    m_nStations (6),
    m_radius (5),
    m_enableDlOfdma (true),
    m_enableUlOfdma(false),
    m_channelWidth (40),
    m_channelNumber (38),
    m_channelCenterFrequency (0),
    m_guardInterval (3200),
    m_maxNRus (4),
    m_mcs (0),
    m_maxAmsduSize (0),
    m_maxAmpduSize (0),
    m_txopLimit (5440),
    m_macQueueSize (0),  // invalid value
    m_msduLifetime (0),  // invalid value
    m_dataRate (0),      // invalid value
    m_dlAckSeqType (1),
    m_baBufferSize (64),
    m_transport ("Udp"),
    m_queueDisc ("default"),
    m_warmup (2.0),
    m_currentSta (0),
    m_ssid (Ssid ("network-A")),
    m_port (7000),
    m_minAmpduRatio (0.0),
    m_maxAmpduRatio (0.0),
    m_avgAmpduRatio (0.0),
    m_nAmpduRatioSamples (0),
    macRxDrop(0),
    phyRxDrop(0),
    m_maxQueueSizeReached(0),
    macApTxDrop(0),
    phyApTxDrop(0)
{
}

void
WifiDlOfdma::Config (int argc, char *argv[])
{
  NS_LOG_FUNCTION (this);

  CommandLine cmd;
  cmd.AddValue ("payloadSize", "Payload size in bytes", m_payloadSize);
  cmd.AddValue ("ulPsduSize", "Maximum size of UL PSDU", m_ulPsduSize);
  cmd.AddValue ("simulationTime", "Simulation time in seconds", m_simulationTime);
  cmd.AddValue ("scheduler", "0 = Deadline Round Robin (Default), 1 = Deadline Aware", m_scheduler);
  cmd.AddValue ("saturateChannel", "true = dataRate > channelCapacity, false = dataRate < channelCapacity", m_saturateChannel);
  cmd.AddValue ("nStations", "Number of non-AP stations", m_nStations);
  cmd.AddValue ("radius", "Radius of the disc centered in the AP and containing all the non-AP STAs", m_radius);
  cmd.AddValue ("enableDlOfdma", "Enable/disable DL OFDMA", m_enableDlOfdma);
  cmd.AddValue ("enableUlOfdma", "Enable/disable UL OFDMA", m_enableUlOfdma);
  cmd.AddValue ("dlAckType", "Ack sequence type for DL OFDMA (1-3)", m_dlAckSeqType);
  cmd.AddValue ("channelWidth", "Channel bandwidth (20, 40, 80, 160)", m_channelWidth);
  cmd.AddValue ("guardInterval", "Guard Interval (800, 1600, 3200)", m_guardInterval);
  cmd.AddValue ("maxRus", "Maximum number of RUs allocated per DL MU PPDU", m_maxNRus);
  cmd.AddValue ("mcs", "The constant MCS value to transmit HE PPDUs", m_mcs);
  cmd.AddValue ("maxAmsduSize", "Maximum A-MSDU size", m_maxAmsduSize);
  cmd.AddValue ("maxAmpduSize", "Maximum A-MPDU size", m_maxAmpduSize);
  cmd.AddValue ("txopLimit", "TXOP Limit", m_txopLimit);
  cmd.AddValue ("queueSize", "Maximum size of a WifiMacQueue (packets)", m_macQueueSize);
  cmd.AddValue ("msduLifetime", "Maximum MSDU lifetime in milliseconds", m_msduLifetime);
  cmd.AddValue ("baBufferSize", "Block Ack buffer size", m_baBufferSize);
  cmd.AddValue ("dataRate", "Per-station data rate (Mb/s)", m_dataRate); //application layer
  cmd.AddValue ("transport", "Transport layer protocol (Udp/Tcp)", m_transport);
  cmd.AddValue ("warmup", "Duration of the warmup period (seconds)", m_warmup);
  cmd.Parse (argc, argv);

  m_macQueueSize = 5000;
  m_msduLifetime = (m_warmup + m_simulationTime + 100) * 1000; 

  switch (m_channelWidth)
    {
    case 20:
      m_channelNumber = 36;
      break;
    case 40:
      m_channelNumber = 38;
      break;
    case 80:
      m_channelNumber = 42;
      break;
    case 160:
      m_channelNumber = 50;
      break;
    default:
      NS_FATAL_ERROR ("Invalid channel bandwidth (must be 20, 40, 80 or 160)");
    }

  std::cout << "Channel bw = " << m_channelWidth << " MHz" << std::endl
            << "MCS = " << m_mcs << std::endl
            << "Simulation Time = " << m_simulationTime << std::endl
            << "TXOP Limit = " << m_txopLimit << std::endl
            << "Number of stations = " << m_nStations << std::endl
            << "Channel Saturated = " << m_saturateChannel << std::endl
            << "Data rate = " << m_dataRate << " Mbps" << std::endl
            << "EDCA queue max size = " << m_macQueueSize << " MSDUs" << std::endl
            << "MSDU lifetime = " << m_msduLifetime << " ms" << std::endl
            << "BA buffer size = " << m_baBufferSize << std::endl;

  if (m_enableDlOfdma) {
      std::cout << "Ack sequence = " << m_dlAckSeqType << std::endl;
  }
  else {
      std::cout << "No OFDMA" << std::endl;
  }

  std::cout << std::endl;
}

void
WifiDlOfdma::Setup (void)
{
  NS_LOG_FUNCTION (this);

  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("999999"));
  Config::SetDefault ("ns3::HeConfiguration::GuardInterval", TimeValue (NanoSeconds (m_guardInterval)));
  Config::SetDefault ("ns3::ArpCache::AliveTimeout", TimeValue (Seconds (3600 * 24))); // ARP cache entries expire after one day
  Config::SetDefault ("ns3::WifiMacQueue::MaxSize", QueueSizeValue (QueueSize (PACKETS, m_macQueueSize)));
  Config::SetDefault ("ns3::WifiMacQueue::MaxDelay", TimeValue (MilliSeconds (m_msduLifetime))); // MSDUs never expire in the MAC Queue
  Config::SetDefault ("ns3::HeConfiguration::MpduBufferSize", UintegerValue (m_baBufferSize));

  m_staNodes.Create (m_nStations);
  m_apNodes.Create (1);

  Ptr<MultiModelSpectrumChannel> spectrumChannel = CreateObject<MultiModelSpectrumChannel> ();
  //Ptr<FriisPropagationLossModel> lossModel = CreateObject<FriisPropagationLossModel> ();
  //spectrumChannel->AddPropagationLossModel (lossModel);
  //Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel> ();
  //spectrumChannel->SetPropagationDelayModel (delayModel);
  SpectrumWifiPhyHelper phy;
  phy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11);
  phy.SetChannel (spectrumChannel);
  phy.Set ("ChannelNumber", UintegerValue (m_channelNumber));
  phy.Set ("ChannelWidth", UintegerValue (m_channelWidth));

  WifiHelper wifi;

  wifi.SetStandard (WIFI_STANDARD_80211ax_5GHZ);

  std::ostringstream oss;
  oss << "HeMcs" << m_mcs;
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode", StringValue (oss.str ()),
                                "ControlMode", StringValue (oss.str ()));

  switch (m_dlAckSeqType) {
    case 1:
      Config::SetDefault ("ns3::WifiDefaultAckManager::DlMuAckSequenceType", EnumValue (WifiAcknowledgment::DL_MU_BAR_BA_SEQUENCE));
      break;
    case 2:
      Config::SetDefault ("ns3::WifiDefaultAckManager::DlMuAckSequenceType", EnumValue (WifiAcknowledgment::DL_MU_TF_MU_BAR));
      break;
    case 3:
       Config::SetDefault ("ns3::WifiDefaultAckManager::DlMuAckSequenceType", EnumValue (WifiAcknowledgment::DL_MU_AGGREGATE_TF));
      break;

    default:
      NS_FATAL_ERROR ("Invalid DL ack sequence type (must be 1, 2 or 3)");
  }

  WifiMacHelper mac;
  if (m_enableDlOfdma)
    {
      if ( m_scheduler == 1 ) {
        
        // Reference to the scheduler can be obtain from the HeFrameExchangeManager::GetMultiUserScheduler method
        mac.SetMultiUserScheduler ("ns3::DaMultiUserScheduler",
                                "NStations", UintegerValue(m_nStations),
                                "ForceDlOfdma", BooleanValue (true),
                                "EnableUlOfdma", BooleanValue (false),
                                "UlPsduSize", UintegerValue (0),
                                "EnableBsrp", BooleanValue(false),
                                "UseCentral26TonesRus", BooleanValue(false));
      }
      else {

        mac.SetMultiUserScheduler ("ns3::DrrMultiUserScheduler",
                                "NStations", UintegerValue(m_nStations),
                                "ForceDlOfdma", BooleanValue (true),
                                "EnableUlOfdma", BooleanValue (false),
                                "UlPsduSize", UintegerValue (0),
                                "EnableBsrp", BooleanValue(false),
                                "UseCentral26TonesRus", BooleanValue(false));
      }
    }

  // How is the AIFSN, CWmin and CWmax set? 
  // WifiHelper.Install -> WifiMacHelper.Create -> RegularWifiMac.ConfigureStandard (CWmin, CWmax set) 
  // -> RegularWifiMac.ConfigureContentionWindow -> WifiMac.ConfigureDcf (AIFSN set)
  mac.SetType ("ns3::StaWifiMac",  "Ssid", SsidValue (Ssid ("non-existing-ssid")));  // prevent stations from automatically associating
  m_staDevices = wifi.Install (phy, mac, m_staNodes);

  mac.SetType ("ns3::ApWifiMac", "Ssid", SsidValue (m_ssid));
  m_apDevices = wifi.Install (phy, mac, m_apNodes);

  // Configure max A-MSDU size and max A-MPDU size on the AP
  Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice> (m_apDevices.Get (0)); // Read about this?
  dev->GetMac ()->SetAttribute ("BE_MaxAmsduSize", UintegerValue (m_maxAmsduSize));
  dev->GetMac ()->SetAttribute ("BE_MaxAmpduSize", UintegerValue (m_maxAmpduSize));
  
  m_channelCenterFrequency = dev->GetPhy ()->GetFrequency ();

  for ( uint16_t i = 0; i < m_nStations; i++ ) {
      m_staPacketInfo.insert(std::make_pair(i, std::vector<uint32_t>()));
      auto itz = m_staPacketInfo.find(i);
      NS_ASSERT(itz != m_staPacketInfo.end());
      itz->second.assign(3, 0);
  }

  // Assign Packet Generation Time periods, Deadlines and Penalities for each station
  // This map will be passed to the Deadline Aware Scheduler later
  // The minimum of all time periods (measured in ms) is also set as 
  // the TXOP Limit.
  // TXOP Limit = min(T1, T2, .. TN)
  // Packet Size = f(TXOP Limit) [18 packets should be transmissible in the set TXOP Limit]
  // Time Quanta = TXOP Limit [For time to round mapping]
  // These restrictions imply that one TXOP can span at most one round
  
  // Deadline < Time Period, this is a strict requirement
  uint32_t schedules[][3] = { { 1, 0, 5 },    { 2, 0, 10 },   { 4, 0, 15 } };
                              // { 1, 0, 60 },   { 5, 3, 45},    { 3, 0, 40 },
                              // { 6, 5, 12 },   { 20, 15, 50 }, { 12, 6, 100 },
                              // { 6, 5, 5 },    { 20, 15, 9 },  { 5, 2, 10 },
                              // { 12, 10, 16 }, { 12, 5, 31 },  { 4, 2, 31 },
                              // { 12, 10, 37 }, { 2, 0, 56 },  { 3, 1, 58 },
                              // { 6, 5, 64 },  { 12, 6, 93 } };

  // uint32_t schedules[][3] = { { 1, 0, 5 },    { 1, 0, 10 },   { 1, 0, 15 },
  //                             { 1, 0, 60 },   { 1, 0, 45},    { 1, 0, 40 },
  //                             { 1, 0, 12 },   { 1, 0, 50 }, { 1, 0, 100 },
  //                             { 1, 0, 5 },    { 1, 0, 9 },  { 1, 0, 10 },
  //                             { 1, 0, 16 }, { 1, 0, 31 },  { 1, 0, 31 },
  //                             { 1, 0, 37 }, { 1, 0, 56 },  { 1, 0, 58 },
  //                             { 1, 0, 64 },  { 1, 0, 93 } };                            

  m_globalSchedule.assign(m_nStations, std::vector<uint32_t>());


  for ( uint16_t i = 0; i < m_nStations; i++ ) {
    auto it = m_staPacketInfo.find(i);
    NS_ASSERT( it != m_staPacketInfo.end() );

    it->second[0] = schedules[i][0]; 
    it->second[1] = schedules[i][1];
    it->second[2] = schedules[i][2];

    m_globalSchedule[i].assign(3, 0);
    m_globalSchedule[i][0] = schedules[i][0];
    m_globalSchedule[i][1] = schedules[i][1];
    m_globalSchedule[i][2] = schedules[i][2];
  }
  
  // for ( uint16_t i = 0; i < m_nStations; i++ ) {
  //     double timePeriod = m_staPacketInfo.at(i)[0] * 1000; // convert to us
      
  //     if ( timePeriod < m_txopLimit )
  //       m_txopLimit = timePeriod;
  // }

  // uint16_t remainder = 0;
  // if ( m_txopLimit < 5440 ) {
  //   uint64_t intTxop = m_txopLimit;
  //   remainder = intTxop % 32;
  //   if ( remainder != 0 )
  //     m_txopLimit = m_txopLimit - remainder; // Do not allow the TXOP Limit to exceed the round duration
  // }

  // Configure TXOP Limit on the AP
  PointerValue ptr;
  dev->GetMac ()->GetAttribute ("BE_Txop", ptr);
  ptr.Get<QosTxop> ()->SetTxopLimit (MicroSeconds (m_txopLimit));
  std::cout << "TXOP Limit set to " << m_txopLimit << " microseconds" << std::endl;

  // if ( m_txopLimit < 5440 )
  //   m_txopLimit = m_txopLimit + remainder; // Reset since this value will now be used as the time quanta to define the duration of the round
  // else
  m_txopLimit = 10000; // time quanta in us, 10 ms 

  // Configure max A-MSDU size and max A-MPDU size on the stations
  for (uint32_t i = 0; i < m_staNodes.GetN (); i++)
    {
      dev = DynamicCast<WifiNetDevice> (m_staDevices.Get (i));
      dev->GetMac ()->SetAttribute ("BE_MaxAmsduSize", UintegerValue (m_maxAmsduSize));
      dev->GetMac ()->SetAttribute ("BE_MaxAmpduSize", UintegerValue (m_maxAmpduSize));
      m_dlStats[dev->GetMac ()->GetAddress ()] = DlStats ();
    }

  // Setting mobility model
  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));  // position of the AP
  mobility.SetPositionAllocator (positionAlloc);
  mobility.Install (m_apNodes);
 
  mobility.SetPositionAllocator ("ns3::UniformDiscPositionAllocator",
                                 "rho", DoubleValue (m_radius));
  mobility.Install (m_staNodes);

  /* Internet stack */
  InternetStackHelper stack;
  stack.Install (m_apNodes);
  stack.Install (m_staNodes);

  Ipv4AddressHelper address;
  address.SetBase ("192.168.1.0", "255.255.255.0");
  Ipv4InterfaceContainer ApInterface;
  ApInterface = address.Assign (m_apDevices);
  m_staInterfaces = address.Assign (m_staDevices);

  /* Traffic Control layer */
  TrafficControlHelper tch;
  if (m_queueDisc.compare ("default") != 0)
    {
      // Uninstall the root queue disc on the AP netdevice
      tch.Uninstall (m_apDevices);
    }

  /* Transport and application layer */
  std::string socketType = (m_transport.compare ("Tcp") == 0 ? "ns3::TcpSocketFactory" : "ns3::UdpSocketFactory");
  // Note that even in the case of randomized packet sized the TCP Segment Size is by default 1000
  Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (m_payloadSize));

  PacketSinkHelper packetSinkHelper (socketType, InetSocketAddress (Ipv4Address::GetAny (), m_port));
  m_sinkApps = packetSinkHelper.Install (m_staNodes);
  m_sinkApps.Stop (Seconds (m_warmup + m_simulationTime + 100)); // let the server be active for a long time
  for ( uint32_t i = 0; i < m_staNodes.GetN(); i++ ) {
      m_sinkApps.Get(i)->SetAttribute ("EnableSeqTsSizeHeader", BooleanValue(true));
  }

  // Keep a track of the bytes received by the stations at the
  // Start and End of the simulation, this is used for throughput calculation
  m_rxStart.assign (m_nStations, 0.0);
  phyDropReason.assign(18, 0);
  m_rxStop.assign (m_nStations, 0.0);
  
  // Keep track of the packet latencies in this map for each station
  for (uint16_t i = 0; i < m_nStations; i++) {

    m_macLatencyMap.insert (std::make_pair (i, std::vector<Time> ()));
    m_appPacketTxMap.insert(std::make_pair(i, std::map<uint32_t, Time>()));
    m_appLatencyMap.insert (std::make_pair (i, std::vector<Time> ()));
    m_phyRxDropMap.insert(std::make_pair (i, std::vector<uint64_t> ()));
    auto it = m_phyRxDropMap.find(i);
    NS_ASSERT(it != m_phyRxDropMap.end());
    it->second.assign(18, 0);

    m_staMacDropMap.insert(std::make_pair(i, std::vector<uint64_t> ()));
    it = m_staMacDropMap.find(i);
    NS_ASSERT(it != m_staMacDropMap.end());
    it->second.assign(3, 0);
  }

  // Callback triggered whenever a STA is associated with an AP
  Config::ConnectWithoutContext ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::StaWifiMac/Assoc",
                                 MakeCallback (&WifiDlOfdma::EstablishBaAgreement, this));
}

void
WifiDlOfdma::Run (void)
{
  NS_LOG_FUNCTION (this);

  Simulator::ScheduleNow (&WifiDlOfdma::StartAssociation, this);

  Simulator::Stop (Seconds (m_warmup + m_simulationTime + 100));
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

  uint64_t totalApDropped = 0;
  uint64_t apDropped;
  std::cout << std::endl << "Dropped MPDU (in AP Queue)" << std::endl
                         << "-----------" << std::endl;
  for (uint32_t i = 0; i < m_staNodes.GetN (); i++)
    {
      auto it = m_dlStats.find (DynamicCast<WifiNetDevice> (m_staDevices.Get (i))->GetMac ()->GetAddress ());
      NS_ASSERT (it != m_dlStats.end ());
      apDropped = it->second.droppedAtAp;
      totalApDropped += apDropped;
      std::cout << "STA_" << i << ": " << apDropped << " ";
    }
  std::cout << std::endl << std::endl << "Total MPDU dropped (in AP Queue): " << totalApDropped << std::endl;

  uint64_t totalDropped = 0;
  uint64_t dropped;
  std::cout << std::endl << "Dropped MPDU (in STA Queue)" << std::endl
                         << "-----------" << std::endl;
  for (uint32_t i = 0; i < m_staNodes.GetN (); i++)
    {
      auto it = m_dlStats.find (DynamicCast<WifiNetDevice> (m_staDevices.Get (i))->GetMac ()->GetAddress ());
      NS_ASSERT (it != m_dlStats.end ());
      dropped = it->second.droppedOnReceive;
      totalDropped += dropped;
      std::cout << "STA_" << i << ": " << dropped << " ";
    }
  std::cout << std::endl << std::endl << "Total MPDU dropped (in STA Queue): " << totalDropped << std::endl;

  std::cout << std::endl << "STA MAC drops with reason" << std::endl
                         << "-----------" << std::endl;
  for (uint32_t j = 0; j < m_staNodes.GetN (); j++) {

    auto it = m_staMacDropMap.find(j);
    NS_ASSERT (it != m_staMacDropMap.end());

    std::cout << "\nSTA_" << j << "\n";
    for (uint32_t i = 0; i < 3; i++) {
      if ( i == 0 ) {
        std::cout << "WIFI_MAC_DROP_FAILED_ENQUEUE " << it->second[i] << " ";
      }
      else if ( i == 1 ) {
        std::cout << "WIFI_MAC_DROP_EXPIRED_LIFETIME " << it->second[i] << " ";
      }
      else if ( i == 2 ) {
        std::cout << "WIFI_MAC_DROP_REACHED_RETRY_LIMIT " << it->second[i] << " ";
      }
    }
  }

  uint64_t totalNAcked = 0;
  uint64_t nacked;
  std::cout << std::endl << std::endl << "Negatively Acknowledged" << std::endl
                         << "-----------" << std::endl;
  for (uint32_t i = 0; i < m_staNodes.GetN (); i++)
    {
      auto it = m_dlStats.find (DynamicCast<WifiNetDevice> (m_staDevices.Get (i))->GetMac ()->GetAddress ());
      NS_ASSERT (it != m_dlStats.end ());
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
  
  std::cout << std::endl << "Latencies [APP] (ms)" << std::endl
                         << "--------------------" << std::endl;
  
  double overallAppLatency = 0.0;
  for (uint32_t i = 0; i < m_staNodes.GetN (); i++)
    {
      auto it = m_appLatencyMap.find (i);
      NS_ASSERT (it != m_appLatencyMap.end ());
      double average_latency_ms = (std::accumulate (it->second.begin (), it->second.end (), NanoSeconds (0))).ToDouble (Time::MS) / it->second.size ();
      
      overallAppLatency += average_latency_ms;
      std::cout << "STA_" << i << ": " << average_latency_ms << " ";
    }
  
  double averageOverallAppLatency = overallAppLatency / m_nStations;
  std::cout << std::endl << std::endl << "Average Latency [APP] (ms): " << averageOverallAppLatency << std::endl;

  std::cout << std::endl << "Latencies [MAC] (ms)" << std::endl
                         << "--------------------" << std::endl;
  
  double overallMacLatency = 0.0;
  std::ostringstream stream;

  for (uint32_t i = 0; i < m_staNodes.GetN (); i++)
    {
      auto it = m_macLatencyMap.find (i);
      NS_ASSERT (it != m_macLatencyMap.end ());
      double average_latency_ms = (std::accumulate (it->second.begin (), it->second.end (), NanoSeconds (0))).ToDouble (Time::MS) / it->second.size ();
      
      overallMacLatency += average_latency_ms;
      std::cout << "STA_" << i << ": " << average_latency_ms << " ";
    }
  
  double averageOverallMacLatency = overallMacLatency / m_nStations;
  std::cout << std::endl << std::endl << "Average Latency [MAC] (ms): " << averageOverallMacLatency << std::endl;
  
  std::cout << std::endl << std::endl << "Maximum BE_Txop Queue Size Reached (Packets): " << m_maxQueueSizeReached << std::endl;

  std::cout << std::endl << "(Min,Max,Count) A-MPDU size" << std::endl
                         << "---------------------------" << std::endl;
  for (uint32_t i = 0; i < m_staNodes.GetN (); i++)
    {
      auto it = m_dlStats.find (DynamicCast<WifiNetDevice> (m_staDevices.Get (i))->GetMac ()->GetAddress ());
      NS_ASSERT (it != m_dlStats.end ());
      std::cout << "STA_" << i << ": (" << it->second.minAmpduSize << "," << it->second.maxAmpduSize
                               << "," << it->second.nAmpdus << ") ";
    }

  std::cout << std::endl << "(Min,Max,Avg) A-MPDU size to max A-MPDU size in DL MU PPDU ratio" << std::endl
                         << "----------------------------------------------------------------" << std::endl;
  for (uint32_t i = 0; i < m_staNodes.GetN (); i++)
    {
      auto it = m_dlStats.find (DynamicCast<WifiNetDevice> (m_staDevices.Get (i))->GetMac ()->GetAddress ());
      NS_ASSERT (it != m_dlStats.end ());
      std::cout << std::fixed << std::setprecision (3)
                << "STA_" << i << ": (" << it->second.minAmpduRatio << ", " << it->second.maxAmpduRatio
                               << ", " << it->second.avgAmpduRatio << ") ";
    }

  std::cout << std::endl << std::endl << "DL MU PPDU completeness: ("
                                      << m_minAmpduRatio << ", "
                                      << m_maxAmpduRatio << ", "
                                      << m_avgAmpduRatio << ")" << std::endl;


  m_macPacketTxMap.clear ();
  m_appPacketTxMap.clear();
  m_macLatencyMap.clear ();
  m_appLatencyMap.clear();
  m_phyRxDropMap.clear();
  m_staMacDropMap.clear();

  Simulator::Destroy ();
}

void
WifiDlOfdma::StartAssociation (void)
{
  NS_LOG_FUNCTION (this << m_currentSta);
  NS_ASSERT (m_currentSta < m_nStations);

  m_aidMap[DynamicCast<WifiNetDevice> (m_staDevices.Get (m_currentSta))->GetMac()->GetAddress()] = ++lastAid;

  std::cout << "Station no. " << m_currentSta << " is associating with the AP\n";
  Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice> (m_staDevices.Get (m_currentSta));
  NS_ASSERT (dev != 0);
  dev->GetMac ()->SetSsid (m_ssid); 
}

// BA = Block Acknowledgement
void
WifiDlOfdma::EstablishBaAgreement (Mac48Address bssid)
{
  // Now that the current station is associated with the AP, let's trigger the creation
  // of an entry in the ARP cache (of both the AP and the STA) and the establishment of
  // a Block Ack agreement between the AP and the STA (and viceversa). This is done by
  // having the AP send 3 ICMP Echo Requests to the STA
  Time pingDuration = MilliSeconds (125);

  V4PingHelper ping (m_staInterfaces.GetAddress (m_currentSta));
  ping.SetAttribute ("Interval", TimeValue (MilliSeconds (50)));
  
  ApplicationContainer pingApps = ping.Install (m_apNodes);
  pingApps.Stop (pingDuration);

  // Install a client application on the current station. In case of TCP traffic,
  // this will trigger the establishment of a TCP connection. The client application
  // is initially quiet (i.e., it does not transmit packets -- this is achieved
  // by setting the duration of the "On" interval to zero).
  uint16_t offInterval = 10;  // milliseconds
  std::stringstream ss;
  ss << "ns3::ConstantRandomVariable[Constant=" << std::fixed << static_cast<double> (offInterval / 1000.) << "]";

  std::string socketType = (m_transport.compare ("Tcp") == 0 ? "ns3::TcpSocketFactory" : "ns3::UdpSocketFactory");
  
  if ( m_currentSta < m_nStations ) {

    std::cout << "Installing On Off App on AP\n";

    //OnOffHelper client (socketType, Ipv4Address::GetAny ());
    OnDemandHelper client (socketType, Ipv4Address::GetAny ());
    //client.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
    //client.SetAttribute ("OffTime", StringValue (ss.str ()));
    client.SetAttribute ("DataRate", DataRateValue (DataRate (m_dataRate * 1e6)));
    client.SetAttribute ("PacketSize", UintegerValue (m_payloadSize));
    client.SetAttribute ("EnableSeqTsSizeHeader", BooleanValue(true));


    InetSocketAddress dest (m_staInterfaces.GetAddress (m_currentSta), m_port);
    // dest.SetTos (0xb8); //AC_VI
    client.SetAttribute ("Remote", AddressValue (dest));
    uint64_t startTime = std::ceil (Simulator::Now ().ToDouble (Time::MS) / offInterval) * offInterval;

    //Simulator::Schedule (MilliSeconds (static_cast<uint64_t> (startTime) + 110) - Simulator::Now (),
    //                   &WifiDlOfdma::InstallOnOffClient, this, client);

    Simulator::Schedule (MilliSeconds (static_cast<uint64_t> (startTime) + 110) - Simulator::Now (),
                       &WifiDlOfdma::InstallOnDemandClient, this, client);
  }   

  if (++m_currentSta < m_nStations) {
    Simulator::Schedule (pingDuration, &WifiDlOfdma::StartAssociation, this);
  }
  else {
    Simulator::Schedule (pingDuration, &WifiDlOfdma::StartTraffic, this);
  }
}

void
WifiDlOfdma::InstallOnOffClient (OnOffHelper client)
{
  NS_LOG_FUNCTION (this << m_currentSta);

  m_OnOffApps.Add (client.Install (m_apNodes));
  m_OnOffApps.Stop (Seconds (m_warmup + m_simulationTime + 100)); // let clients be active for a long time
}

void
WifiDlOfdma::InstallOnDemandClient (OnDemandHelper client)
{
  NS_LOG_FUNCTION (this << m_currentSta);

  m_OnOffApps.Add (client.Install (m_apNodes));
  m_OnOffApps.Stop (Seconds (m_warmup + m_simulationTime + 100)); // let clients be active for a long time
}

void
WifiDlOfdma::StartTraffic (void)
{
  NS_LOG_FUNCTION (this);

  // for (uint32_t i = 0; i < m_nStations; i++) {
      
  //   Ptr<Application> clientApp = m_OnOffApps.Get (i);
    
  //   clientApp->SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0002]"));
  //   std::stringstream ss;
  //   ss << "ns3::ConstantRandomVariable[Constant=" << std::fixed << static_cast<double> (( m_globalSchedule[i][0] / 1000.) - 0.0002) << "]";
  //   std::cout << " off interval = " << ss.str() << std::endl;

  //   clientApp->SetAttribute ("OffTime", StringValue (ss.str())); // after x ms
  // }
      
  
  Ptr<HeFrameExchangeManager> heFem = DynamicCast<HeFrameExchangeManager>(DynamicCast<RegularWifiMac>(DynamicCast<WifiNetDevice>(m_apDevices.Get(0))->GetMac())->GetFrameExchangeManager());
  if ( m_scheduler == 1 ) {
    Ptr<DaMultiUserScheduler> muScheduler = DynamicCast<DaMultiUserScheduler>(heFem->GetMultiUserScheduler());
    muScheduler->SetStaPacketInfo(m_staPacketInfo);
    muScheduler->PassReferenceToOnDemandApps(m_OnOffApps);
    muScheduler->StartNextRound(true);
    muScheduler->NotifyDeadlineConstrainedTrafficStarted();
  }
  else {
    Ptr<DrrMultiUserScheduler> muScheduler = DynamicCast<DrrMultiUserScheduler>(heFem->GetMultiUserScheduler());
    muScheduler->SetStaPacketInfo(m_staPacketInfo);
    muScheduler->PassReferenceToOnDemandApps(m_OnOffApps);
    muScheduler->StartNextRound(true);
    muScheduler->NotifyDeadlineConstrainedTrafficStarted();
  }

  //double m_actualWarmup = m_warmup + Simulator::Now().ToDouble(Time::S);
  //Simulator::Schedule (Seconds(m_actualWarmup), &WifiDlOfdma::StartStatistics, this);
  StartStatistics();
}

void
WifiDlOfdma::StartStatistics (void)
{
  NS_LOG_FUNCTION (this);
  double m_actualSimulationTime = m_simulationTime + Simulator::Now().ToDouble(Time::S);
  std::cout << "m_actualSimulationTime " << m_actualSimulationTime << "\n";
  Simulator::Schedule (Seconds (m_actualSimulationTime), &WifiDlOfdma::StopStatistics, this);

  std::cout << "============== START STATISTICS ============== \n";

  Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice> (m_apDevices.Get (0));
  
  if ( m_enableDlOfdma ) {
      Ptr<HeFrameExchangeManager> fem = DynamicCast<HeFrameExchangeManager>(DynamicCast<RegularWifiMac> (dev->GetMac ())->GetFrameExchangeManager());
      fem->TraceConnectWithoutContext("PsduMapForwardDown", MakeCallback(&WifiDlOfdma::NotifyPsduMapForwardedDown, this));
  }
  else {
      Ptr<HtFrameExchangeManager> fem = DynamicCast<HtFrameExchangeManager>(DynamicCast<RegularWifiMac> (dev->GetMac ())->GetFrameExchangeManager());
      fem->TraceConnectWithoutContext("PsduForwardDown", MakeCallback(&WifiDlOfdma::NotifyPsduForwardedDown, this));
  }
  
  // Correct, simply update the global counter for AP MPDU Drops
  DynamicCast<RegularWifiMac> (dev->GetMac ())->TraceConnectWithoutContext ("DroppedMpdu", MakeCallback (&WifiDlOfdma::NotifyApDroppedMpdu, this));
  DynamicCast<RegularWifiMac> (dev->GetMac ())->TraceConnectWithoutContext("NAckedMpdu", MakeCallback (&WifiDlOfdma::NotifyTxNAcked, this));

  // Both correct, tracers specifically placed on the AP MAC and PHY
  DynamicCast<RegularWifiMac> (dev->GetMac ())->TraceConnectWithoutContext("MacTxDrop", MakeCallback (&WifiDlOfdma::NotifyAPMacTxDropped, this));
  DynamicCast<WifiPhy> (dev->GetPhy())->TraceConnectWithoutContext("PhyTxDrop", MakeCallback (&WifiDlOfdma::NotifyAPPhyTxDropped, this));

  // This callback always passes the same context in case of DL UDP, that of the AP, but in case of DL TCP, this is invoked also for the STA, need to fix
  //Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::WifiMac/MacTx", MakeCallback (&WifiDlOfdma::NotifyMacTx, this));
  DynamicCast<RegularWifiMac> (dev->GetMac ())->TraceConnectWithoutContext("MacTx", MakeCallback (&WifiDlOfdma::NotifyMacTx, this));
  
  // This callback passes the context of only the STAs in case of DL UDP, but in case of DL TCP it can also pass the context of the AP, need to fix
  //Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::WifiMac/MacRx", MakeCallback (&WifiDlOfdma::NotifyMacRx, this));

  // This callback is triggered for both AP and STAs incase of TCP
  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::WifiMac/MacRxDrop", MakeCallback (&WifiDlOfdma::NotifyMacRxDropped, this));
  
  for (uint32_t i = 0; i < m_staNodes.GetN (); i++) {

      std::ostringstream oss;
      oss << "/NodeList/" << i << "/ApplicationList/" << i << "/";
      //DynamicCast<OnOffApplication>(m_OnOffApps.Get(i))->TraceConnect("TxWithSeqTsSize", oss.str(),  MakeCallback (&WifiDlOfdma::NotifyApplicationTx, this));
      DynamicCast<OnDemandApplication>(m_OnOffApps.Get(i))->TraceConnect("TxWithSeqTsSize", oss.str(),  MakeCallback (&WifiDlOfdma::NotifyApplicationTx, this));
      DynamicCast<PacketSink>(m_sinkApps.Get(i))->TraceConnect("RxWithSeqTsSize", oss.str(),  MakeCallback (&WifiDlOfdma::NotifyApplicationRx, this));
  }

  for (uint32_t i = 0; i < m_staNodes.GetN (); i++)
    {
      m_rxStart[i] = DynamicCast<PacketSink> (m_sinkApps.Get (i))->GetTotalRx ();

      Ptr<WifiNetDevice> staDev = DynamicCast<WifiNetDevice> (m_staDevices.Get (i));
      
      std::ostringstream oss;
      oss << "/NodeList/" << i << "/DeviceList/" << i << "/";
      DynamicCast<WifiPhy> (staDev->GetPhy())->TraceConnect("PhyRxDrop", oss.str(), MakeCallback (&WifiDlOfdma::NotifyPhyRxDropped, this));
      
      // Must store reasons for MPDU drops
      DynamicCast<RegularWifiMac> (staDev->GetMac ())->TraceConnect ("DroppedMpdu", oss.str(), MakeCallback (&WifiDlOfdma::NotifyStaDroppedMpdu, this));
      
      // Only apply this trace for STAs, otherwise TCP ACKs cause this to be triggered for the AP too
      DynamicCast<RegularWifiMac> (staDev->GetMac ())->TraceConnect("MacRx", oss.str(), MakeCallback (&WifiDlOfdma::NotifyMacRx, this));
    }
}

void
WifiDlOfdma::StopStatistics (void)
{
  NS_LOG_FUNCTION (this);

  std::cout << "============== STOP STATISTICS ============== \n";
  std::cout << "Current Time: " << Simulator::Now().ToDouble(Time::S) << "\n";

  Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice> (m_apDevices.Get (0));
  
  if ( m_enableDlOfdma ) {
      Ptr<HeFrameExchangeManager> fem = DynamicCast<HeFrameExchangeManager>(DynamicCast<RegularWifiMac> (dev->GetMac ())->GetFrameExchangeManager());
      fem->TraceDisconnectWithoutContext("PsduMapForwardDown", MakeCallback(&WifiDlOfdma::NotifyPsduMapForwardedDown, this));
  }
  else {
      Ptr<HtFrameExchangeManager> fem = DynamicCast<HtFrameExchangeManager>(DynamicCast<RegularWifiMac> (dev->GetMac ())->GetFrameExchangeManager());
      fem->TraceDisconnectWithoutContext("PsduForwardDown", MakeCallback(&WifiDlOfdma::NotifyPsduForwardedDown, this));
  }

  DynamicCast<RegularWifiMac> (dev->GetMac ())->TraceDisconnectWithoutContext ("DroppedMpdu", MakeCallback (&WifiDlOfdma::NotifyApDroppedMpdu, this));
  DynamicCast<RegularWifiMac> (dev->GetMac ())->TraceDisconnectWithoutContext("NAckedMpdu", MakeCallback (&WifiDlOfdma::NotifyTxNAcked, this));

  DynamicCast<RegularWifiMac> (dev->GetMac ())->TraceDisconnectWithoutContext("MacTxDrop", MakeCallback (&WifiDlOfdma::NotifyAPMacTxDropped, this));
  DynamicCast<WifiPhy> (dev->GetPhy())->TraceDisconnectWithoutContext("PhyTxDrop", MakeCallback (&WifiDlOfdma::NotifyAPPhyTxDropped, this));

  // This callback always passes the same context in case of DL UDP, that of the AP, but in case of DL TCP, this is invoked also for the STA, need to fix
  //Config::Disconnect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::WifiMac/MacTx", MakeCallback (&WifiDlOfdma::NotifyMacTx, this));
  DynamicCast<RegularWifiMac> (dev->GetMac ())->TraceDisconnectWithoutContext("MacTx", MakeCallback (&WifiDlOfdma::NotifyMacTx, this));
  
  // This callback passes the context of only the STAs in case of DL UDP, but in case of DL TCP it can also pass the context of the AP, need to fix
  //Config::Disconnect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::WifiMac/MacRx", MakeCallback (&WifiDlOfdma::NotifyMacRx, this));

  // This callback is triggered for both AP and STAs incase of TCP
  Config::Disconnect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::WifiMac/MacRxDrop", MakeCallback (&WifiDlOfdma::NotifyMacRxDropped, this));
  
  for (uint32_t i = 0; i < m_staNodes.GetN (); i++) {

      std::ostringstream oss;
      oss << "/NodeList/" << i << "/ApplicationList/" << i << "/";
      //DynamicCast<OnOffApplication>(m_OnOffApps.Get(i))->TraceDisconnect("TxWithSeqTsSize", oss.str(),  MakeCallback (&WifiDlOfdma::NotifyApplicationTx, this));
      DynamicCast<OnDemandApplication>(m_OnOffApps.Get(i))->TraceDisconnect("TxWithSeqTsSize", oss.str(),  MakeCallback (&WifiDlOfdma::NotifyApplicationTx, this));
      DynamicCast<PacketSink>(m_sinkApps.Get(i))->TraceDisconnect("RxWithSeqTsSize", oss.str(),  MakeCallback (&WifiDlOfdma::NotifyApplicationRx, this));
  }

  for (uint32_t i = 0; i < m_staNodes.GetN (); i++)
    {
      m_rxStop[i] = DynamicCast<PacketSink> (m_sinkApps.Get (i))->GetTotalRx ();

      Ptr<WifiNetDevice> staDev = DynamicCast<WifiNetDevice> (m_staDevices.Get (i));

      std::ostringstream oss;
      oss << "/NodeList/" << i << "/DeviceList/" << i << "/";
      DynamicCast<WifiPhy> (staDev->GetPhy())->TraceDisconnect("PhyRxDrop", oss.str(), MakeCallback (&WifiDlOfdma::NotifyPhyRxDropped, this));
      
      DynamicCast<RegularWifiMac> (staDev->GetMac ())->TraceDisconnect ("DroppedMpdu", oss.str(), MakeCallback (&WifiDlOfdma::NotifyStaDroppedMpdu, this));

      // Only apply this trace for STAs, otherwise TCP ACKs cause this to be triggered for the AP too
      DynamicCast<RegularWifiMac> (staDev->GetMac ())->TraceDisconnect("MacRx", oss.str(), MakeCallback (&WifiDlOfdma::NotifyMacRx, this));
    }

  // (Brutally) stop client applications
  for (uint32_t i = 0; i < m_OnOffApps.GetN (); i++)
    {
      m_OnOffApps.Get (i)->Dispose ();
    }
}

void
WifiDlOfdma::NotifyApDroppedMpdu (WifiMacDropReason reason, Ptr<const WifiMacQueueItem> mpdu)
{
  WifiMacHeader hdr = mpdu->GetHeader();
  auto it = m_dlStats.find (hdr.GetAddr1 ());
  NS_ASSERT (it != m_dlStats.end ());
  it->second.droppedAtAp++;
}

void
WifiDlOfdma::NotifyStaDroppedMpdu (std::string context, WifiMacDropReason reason, Ptr<const WifiMacQueueItem> mpdu)
{
  WifiMacHeader hdr = mpdu->GetHeader();
  auto it = m_dlStats.find (hdr.GetAddr2 ());
  NS_ASSERT (it != m_dlStats.end ());
  it->second.droppedOnReceive++;

  auto itStaDropMap = m_staMacDropMap.find (DeviceContextToNodeId (context));
  NS_ASSERT (itStaDropMap != m_staMacDropMap.end ());
  itStaDropMap->second[reason]++;
}

void
WifiDlOfdma::NotifyMacRxDropped(std::string context, Ptr< const Packet > packet) {

    macRxDrop++;
}

void
WifiDlOfdma::NotifyPhyRxDropped(std::string context, Ptr< const Packet > packet, WifiPhyRxfailureReason reason) {
  
  phyRxDrop++;
  phyDropReason[reason]++;

  auto itStaDropMap = m_phyRxDropMap.find (DeviceContextToNodeId (context));
  NS_ASSERT (itStaDropMap != m_phyRxDropMap.end ());
  itStaDropMap->second[reason]++;
}

void 
WifiDlOfdma::NotifyAPMacTxDropped(Ptr< const Packet > packet) {

    macApTxDrop++;
}

void 
WifiDlOfdma::NotifyAPPhyTxDropped(Ptr< const Packet > psdu) {

    phyApTxDrop++;
}

void
WifiDlOfdma::NotifyTxNAcked (Ptr<const WifiMacQueueItem> mpdu)
{
  WifiMacHeader hdr = mpdu->GetHeader();
  auto it = m_dlStats.find (hdr.GetAddr1 ());
  NS_ASSERT (it != m_dlStats.end ());
  it->second.nacked++;
}

void
WifiDlOfdma::NotifyPsduForwardedDown (Ptr<const WifiPsdu> psdu, WifiTxVector txVector)
{
  Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice> (m_apDevices.Get (0));
  Mac48Address apAddress = dev->GetMac ()->GetAddress ();

  // Downlink frame
  if (psdu->GetAddr1 () != apAddress && psdu->GetHeader (0).IsQosData ())
    {
      uint32_t maxAmpduSize = 0;
      uint32_t ampduSizeSum = 0;

      uint32_t currSize = psdu->GetSize ();
      if (currSize > maxAmpduSize) {
            maxAmpduSize = currSize;
      }
        
      ampduSizeSum += currSize;

      auto it = m_dlStats.find (psdu->GetAddr1 ());
      NS_ASSERT (it != m_dlStats.end ());
      if (it->second.minAmpduSize == 0 || currSize < it->second.minAmpduSize)
      {
        it->second.minAmpduSize = currSize;
      }
      if (currSize > it->second.maxAmpduSize)
      {
        it->second.maxAmpduSize = currSize;
      }
          
      it->second.nAmpdus++;
    }
}

void
WifiDlOfdma::NotifyPsduMapForwardedDown (WifiConstPsduMap psduMap, WifiTxVector txVector)
{
  Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice> (m_apDevices.Get (0));
  Mac48Address apAddress = dev->GetMac ()->GetAddress ();

  // Downlink frame
  if (psduMap.begin ()->second->GetAddr1 () != apAddress && psduMap.begin ()->second->GetHeader (0).IsQosData ())
    {
      uint32_t maxAmpduSize = 0;
      uint32_t ampduSizeSum = 0;

      for (auto& psdu : psduMap)
        {
          uint32_t currSize = psdu.second->GetSize ();
          if (currSize > maxAmpduSize)
            {
              maxAmpduSize = currSize;
            }
          ampduSizeSum += currSize;

          auto it = m_dlStats.find (psdu.second->GetAddr1 ());
          NS_ASSERT (it != m_dlStats.end ());
          if (it->second.minAmpduSize == 0 || currSize < it->second.minAmpduSize)
            {
              it->second.minAmpduSize = currSize;
            }
          if (currSize > it->second.maxAmpduSize)
            {
              it->second.maxAmpduSize = currSize;
            }
          it->second.nAmpdus++;
        }

      // DL MU PPDU
      if (txVector.GetPreambleType () == WIFI_PREAMBLE_HE_MU)
        {
          std::size_t nRus = txVector.GetHeMuUserInfoMap ().size ();
          uint32_t maxBytes = maxAmpduSize * nRus;
          NS_ASSERT (maxBytes > 0);
          double currRatio = static_cast<double> (ampduSizeSum) / maxBytes;

          if (m_minAmpduRatio == 0 || currRatio < m_minAmpduRatio)
            {
              m_minAmpduRatio = currRatio;
            }
          if (currRatio > m_maxAmpduRatio)
            {
              m_maxAmpduRatio = currRatio;
            }
          m_avgAmpduRatio = (m_avgAmpduRatio * m_nAmpduRatioSamples + currRatio) / (m_nAmpduRatioSamples + 1);
          m_nAmpduRatioSamples++;

          dev = DynamicCast<WifiNetDevice> (m_apDevices.Get (0));
          Ptr<ApWifiMac> mac = DynamicCast<ApWifiMac> (dev->GetMac ());

          for (auto& userInfo : txVector.GetHeMuUserInfoMap ())
            {
              auto psduIt = psduMap.find (userInfo.first);

              if (psduIt == psduMap.end ())
                {
                  // the station that was assigned this RU did not transmit a PSDU
                  currRatio = 0.0;
                }
              else
                {
                  currRatio = static_cast<double> (psduIt->second->GetSize ()) / maxAmpduSize;
                }

              Mac48Address address = mac->GetStaList ().at (userInfo.first);
              auto it = m_dlStats.find (address);
              NS_ASSERT (it != m_dlStats.end ());

              if (it->second.minAmpduRatio == 0 || currRatio < it->second.minAmpduRatio)
                {
                  it->second.minAmpduRatio = currRatio;
                }
              if (currRatio > it->second.maxAmpduRatio)
                {
                  it->second.maxAmpduRatio = currRatio;
                }
              it->second.avgAmpduRatio = (it->second.avgAmpduRatio * it->second.nAmpduRatioSamples + currRatio)
                                         / (it->second.nAmpduRatioSamples + 1);
              it->second.nAmpduRatioSamples++;
            }
        }
    }
}

void
WifiDlOfdma::NotifyApplicationTx (std::string context, Ptr<const Packet> p, const Address &add1, const Address &add2, const SeqTsSizeHeader &tsheader) {

    //std::cout << "Packet transmitted to STA " << AppContextToNodeId(context) << " from APP with TS = " << tsheader.GetTs() << "\n";
    auto itStaPacketTxMap = m_appPacketTxMap.find(AppContextToNodeId(context));
    NS_ASSERT(itStaPacketTxMap != m_appPacketTxMap.end());
    itStaPacketTxMap->second.insert (std::make_pair (tsheader.GetSeq(), Simulator::Now ()));
}
  
void
WifiDlOfdma::NotifyApplicationRx(std::string context, Ptr<const Packet> p, const Address &add1, const Address &add2, const SeqTsSizeHeader &tsheader) {

  // If you check the packet size here it will be x-20 bytes if the Packet Size initially specified was x,
  // This is because this trace receives the packet with the SeqTsSizeHeader removed, so no need to worry,
  // the actual packet size is still x.

  auto itStaPacketTxMap = m_appPacketTxMap.find (AppContextToNodeId(context));
  NS_ASSERT(itStaPacketTxMap != m_appPacketTxMap.end());
  auto itTxPacket = itStaPacketTxMap->second.find(tsheader.GetSeq());
  if (itTxPacket != itStaPacketTxMap->second.end ()) // This has been done here to make sure only packets tracked after NotifyApplicationTx trace is enabled are measured for latency.
    {
      Time latency = (Simulator::Now () - itTxPacket->second);
      auto itStaLatencies = m_appLatencyMap.find (AppContextToNodeId (context));
      NS_ASSERT (itStaLatencies != m_appLatencyMap.end ());
      itStaLatencies->second.push_back (latency);
      itStaPacketTxMap->second.erase (itTxPacket); // To allow reuse of old Seq numbers
    }
}

uint32_t
WifiDlOfdma::AppContextToNodeId (const std::string & context)
{
  std::string sub = context.substr (10);
  uint32_t pos = sub.find ("/Application");
  uint32_t nodeId = atoi (sub.substr (0, pos).c_str ());
  return nodeId;
}

void
WifiDlOfdma::NotifyMacTx (Ptr<const Packet> p)
{
  // For DL UDP, the context passed it always the same, that of the AP
  if ( !m_randomizePacketSize ) {
    if (p->GetSize () < m_payloadSize)
      {
        return;
      }
  }
  else { 
    if (p->GetSize () < m_minSampleRange)
      {
        return;
      }
  }

  PointerValue ptr;
  Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice>(m_apDevices.Get(0));
  dev->GetMac()->GetAttribute("BE_Txop", ptr);

  // Note that this is the size of the packet queue associated with this Txop.
  uint32_t currMacQueueSize = ptr.Get<QosTxop> ()->GetWifiMacQueue()->GetNPackets();
  if ( currMacQueueSize > m_maxQueueSizeReached )
    m_maxQueueSizeReached = currMacQueueSize;

  m_macPacketTxMap.insert (std::make_pair (p->GetUid (), Simulator::Now ()));
}

void
WifiDlOfdma::NotifyMacRx (std::string context, Ptr<const Packet> p)
{
  if ( !m_randomizePacketSize ) {
    if (p->GetSize () < m_payloadSize)
      {
        return;
      }
  }
  else { 
    if (p->GetSize () < m_minSampleRange)
      {
        return;
      }
  }

  auto itTxPacket = m_macPacketTxMap.find (p->GetUid ());
  if (itTxPacket != m_macPacketTxMap.end ()) // This has been done here to make sure only packets tracked after MacTx trace is enabled are measured for latency here.
    {
      Time latency = (Simulator::Now () - itTxPacket->second);
      auto itStaLatencies = m_macLatencyMap.find (DeviceContextToNodeId (context));
      NS_ASSERT (itStaLatencies != m_macLatencyMap.end ());
      itStaLatencies->second.push_back (latency);
      m_macPacketTxMap.erase (itTxPacket);
    }
}

uint32_t
WifiDlOfdma::DeviceContextToNodeId (const std::string & context)
{
  std::string sub = context.substr (10);
  uint32_t pos = sub.find ("/Device");
  uint32_t nodeId = atoi (sub.substr (0, pos).c_str ());
  return nodeId;
}

int main (int argc, char *argv[])
{
  WifiDlOfdma example;
  example.Config (argc, argv);
  example.Setup ();
  example.Run ();

  return 0;
}
