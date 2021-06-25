/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2019
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
 * Authors: Stefano Avallone <stavallo@unina.it>
 *          Sébastien Deronne <sebastien.deronne@gmail.com>
 */

// ./waf --run "wifi-dl-qualcomm-cir --simulationTime=10 --nStations=5 --mcs=11 --enablePcap=false --dlAckType=2 --channelWidth=20 --guardInterval=800 --radius=1"

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
#include "ns3/bulk-send-helper.h"
#include "ns3/v4ping-helper.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/wifi-mac-queue.h"
#include "ns3/wifi-psdu.h"
#include "ns3/ctrl-headers.h"
#include "ns3/traffic-control-helper.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/he-phy.h"
#include "ns3/wifi-standards.h"
#include "ns3/wifi-acknowledgment.h"
#include <vector>
#include <map>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <numeric>
#include <time.h>

#define NUM_BULK_SEND_CLIENTS 10
#define NUM_HTTP_CLIENT 10
#define NUM__CLIENTS 10

using namespace ns3;


NS_LOG_COMPONENT_DEFINE ("WifiDlOfdmaExample");
void
ServerConnectionEstablished (Ptr<const ThreeGppHttpServer>, Ptr<Socket>)
{
  std::cout << "Client has established a connection to the server.\n";
  NS_LOG_INFO ("Client has established a connection to the server.");
}

void
MainObjectGenerated (uint32_t size)
{
  std::cout << "Server generated a main object of " << size << " bytes.\n";
  NS_LOG_INFO ("Server generated a main object of " << size << " bytes.");
}

void
EmbeddedObjectGenerated (uint32_t size)
{
  std::cout << "Server generated an embedded object of " << size << " bytes.\n";
  NS_LOG_INFO ("Server generated an embedded object of " << size << " bytes.");
}

void
ServerTx (Ptr<const Packet> packet)
{
  std::cout << "Server sent a packet of " << packet->GetSize () << " bytes.\n";
  NS_LOG_INFO ("Server sent a packet of " << packet->GetSize () << " bytes.");
}

void
ClientRx (Ptr<const Packet> packet, const Address &address)
{
  std::cout << "Client received a packet of " << packet->GetSize () << " bytes from " << address;
  NS_LOG_INFO ("Client received a packet of " << packet->GetSize () << " bytes from " << address);
}

void
ClientMainObjectReceived (Ptr<const ThreeGppHttpClient>, Ptr<const Packet> packet)
{
  Ptr<Packet> p = packet->Copy ();
  ThreeGppHttpHeader header;
  p->RemoveHeader (header);
  if (header.GetContentLength () == p->GetSize ()
      && header.GetContentType () == ThreeGppHttpHeader::MAIN_OBJECT)
    {
      std::cout << "Client has successfully received a main object of " << p->GetSize () << " bytes.\n";
      NS_LOG_INFO ("Client has successfully received a main object of "
                   << p->GetSize () << " bytes.");
    }
  else
    {
      NS_LOG_INFO ("Client failed to parse a main object. ");
    }
}

void
ClientEmbeddedObjectReceived (Ptr<const ThreeGppHttpClient>, Ptr<const Packet> packet)
{
  Ptr<Packet> p = packet->Copy ();
  ThreeGppHttpHeader header;
  p->RemoveHeader (header);
  if (header.GetContentLength () == p->GetSize ()
      && header.GetContentType () == ThreeGppHttpHeader::EMBEDDED_OBJECT)
    {
      std::cout << "Client has successfully received an embedded object of " << p->GetSize () << " bytes.\n";
      NS_LOG_INFO ("Client has successfully received an embedded object of "
                   << p->GetSize () << " bytes.");
    }
  else
    {
      NS_LOG_INFO ("Client failed to parse an embedded object. ");
    }
}


/**
 * \brief Example to test DL OFDMA
 *
 * Usage: ./waf --run "wifi-dl-ofdma [options]" > results.log
 *
 * To extract the list of space-separated throughput values:
 *
 * grep -A 2 Throughput results.log | grep STA_ | sed 's/STA_[0-9]*: //g'
 *
 * Similarly, it is possible to extract the list of per-station TX failures
 * (grep -A 2 failures...) and expired MSDUs (grep -A 2 Expired...)
 */
class WifiDlOfdmaExample
{
public:
  /**
   * Create an example instance.
   */
  WifiDlOfdmaExample ();
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
   * Make the AP establish a BA agreement with the current station.
   */
  void EstablishBaAgreement (Mac48Address bssid);
  /**
   * Start a client application.
   */
  void StartClient (OnOffHelper client);

  void StartHttpServer(ThreeGppHttpServerHelper server);

  void StartBulkClient(BulkSendHelper client);
  /**
   * Start generating traffic.
   */
  //void StartTraffic (void);
  void StartTraffic(ThreeGppHttpServerHelper server);
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
  void NotifyTxFailed (WifiMacDropReason reason, Ptr<const WifiMacQueueItem> mpdu);
  /**
   * Report that an MPDU was negative acknowledged
  */
  void WifiDlOfdmaExample::NotifyTxNAcked (Ptr<const WifiMacQueueItem> mpdu);
  /**
   * Report that the lifetime of an MSDU expired.
   */
  void NotifyMsduExpired (Ptr<const WifiMacQueueItem> item);
  /**
   * Report that an MSDU was dequeued from the EDCA queue.
   */
  void NotifyMsduDequeuedFromEdcaQueue (Ptr<const WifiMacQueueItem> item);
  /**
   * Report that PSDUs were forwarded down to the PHY.
   */
  void NotifyPsduForwardedDown (WifiPsduMap psduMap, WifiTxVector txVector);
  /**
   * Report that an MPDU was not correctly received.
   */
  void TxopDuration (Time startTime, Time duration);
  /**
   * Report that the application has created and sent a new packet.
   */
  void NotifyApplicationTx (std::string context, Ptr<const Packet>);
  /**
   * Report that the application has received a new packet.
   */
  void NotifyApplicationRx (std::string context, Ptr<const Packet> p);
  /**
   * Parse context strings of the form "/NodeList/x/DeviceList/y/" to extract the NodeId
   */
  uint32_t ContextToNodeId (const std::string & context);

private:
  uint32_t m_payloadSize;   // bytes
  double m_simulationTime;  // seconds
  uint16_t m_nStations;     // not including AP
  double m_radius;          // meters
  bool m_enableDlOfdma;
  bool m_forceDlOfdma;
  bool m_enableUlOfdma;
  bool m_enableBsrp;
  bool m_useCentral26TonesRus;
  uint32_t m_ulPsduSize;
  uint16_t m_channelWidth;  // channel bandwidth
  uint8_t m_channelNumber;
  uint16_t m_channelCenterFrequency;
  uint16_t m_guardInterval; // GI in nanoseconds
  uint32_t m_maxNRus;        // max number of RUs per MU PPDU
  uint32_t m_mcs;           // MCS value
  uint16_t m_maxAmsduSize;  // maximum A-MSDU size
  uint32_t m_maxAmpduSize;  // maximum A-MSDU size
  double m_txopLimit;       // microseconds
  uint32_t m_macQueueSize;  // packets
  uint32_t m_msduLifetime;  // milliseconds
  bool m_enableRts;
  double m_dataRate;        // Mb/s
  uint16_t m_dlAckSeqType;
  bool m_continueTxop;
  uint16_t m_baBufferSize;
  std::string m_transport;
  std::string m_queueDisc;
  bool m_enablePcap;
  double m_warmup;          // duration of the warmup period (seconds)
  std::size_t m_currentSta; // index of the current station
  Ssid m_ssid;
  NodeContainer m_apNodes;
  NodeContainer m_staNodes;
  NetDeviceContainer m_staDevices;
  NetDeviceContainer m_apDevices;
  Ipv4InterfaceContainer m_staInterfaces;
  Ipv4InterfaceContainer m_apInterface;
  ApplicationContainer m_sinkApps;
  ApplicationContainer m_onOffSourceApps;
  ApplicationContainer m_bulkSourceApps;
  ApplicationContainer m_httpServerApp;
  ApplicationContainer m_httpClientApps;
  uint16_t m_port;
  Time m_maxTxop;
  std::vector<uint64_t> m_rxStart, m_rxStop;
  double m_minAmpduRatio;
  double m_maxAmpduRatio;
  double m_avgAmpduRatio;
  uint64_t m_nAmpduRatioSamples;
  Time m_lastTxTime;
  double m_minHolDelay;     // milliseconds
  double m_maxHolDelay;     // milliseconds
  double m_avgHolDelay;     // milliseconds
  uint64_t m_nHolDelaySamples;
  std::map <uint64_t /* uid */, Time /* start */> m_appPacketTxMap;
  std::map <uint32_t /* nodeId */, std::vector<Time>  /* array of latencies */> m_appLatencyMap;
  bool m_verbose;
  uint64_t m_nBasicTriggerFramesSent;
  uint64_t m_nFailedTriggerFrames;  // no station responded
  double m_minLengthRatio;
  double m_maxLenghtRatio;
  double m_avgLengthRatio;
  Time m_tfUlLength;                // TX duration coded in UL Length subfield of Trigger Frame
  Time m_overallTimeGrantedByTf;    // m_tfUlLength times the number of addressed stations
  Time m_responsesToLastTfDuration; // sum of the durations of the HE TB PPDUs in response to last TF

  struct DlStats
  {
    uint64_t failed {0};
    uint64_t expired {0};
    uint32_t minAmpduSize {0};
    uint32_t maxAmpduSize {0};
    uint64_t nAmpdus {0};
    double minAmpduRatio {0.0};
    double maxAmpduRatio {0.0};
    double avgAmpduRatio {0.0};
    uint64_t nAmpduRatioSamples {0};
    Time lastTxTime {Seconds (0)};
    double minHolDelay {0.0};
    double maxHolDelay {0.0};
    double avgHolDelay {0.0};
    uint64_t nHolDelaySamples {0};
  };
  std::map<Mac48Address, DlStats> m_dlStats;

  struct UlStats
  {
    double minLengthRatio {0.0};
    double maxLenghtRatio {0.0};
    double avgLengthRatio {0.0};
    uint64_t nLengthRatioSamples {0};  // count of HE TB PPDUs sent
    uint64_t nSolicitingTriggerFrames {0};
  };
  std::map<Mac48Address, UlStats> m_ulStats;
};

WifiDlOfdmaExample::WifiDlOfdmaExample ()
  : m_payloadSize (1400),
    m_simulationTime (2),
    m_nStations (16),
    m_radius (10),
    m_enableDlOfdma (true),
    m_forceDlOfdma (true),
    m_enableUlOfdma (true),
    m_enableBsrp(false),
    m_useCentral26TonesRus(false),
    m_ulPsduSize (1600),
    m_channelWidth (20),
    m_channelNumber (36),
    m_channelCenterFrequency (0),
    m_guardInterval (3200),
    m_maxNRus (4),
    m_mcs (0),
    m_maxAmsduSize (7500),
    m_maxAmpduSize (6500631),
    m_txopLimit (5440),
    m_macQueueSize (0),  // invalid value
    m_msduLifetime (0),  // invalid value
    m_enableRts (false),
    m_dataRate (0),      // invalid value
    m_dlAckSeqType (1),
    m_continueTxop (false),
    m_baBufferSize (64),
    m_transport ("Tcp"),
    m_queueDisc ("default"),
    m_enablePcap (true),
    m_warmup (2.0),
    m_currentSta (0),
    m_ssid (Ssid ("network-A")),
    m_port (7000),
    m_maxTxop (Seconds (0)),
    m_minAmpduRatio (0.0),
    m_maxAmpduRatio (0.0),
    m_avgAmpduRatio (0.0),
    m_nAmpduRatioSamples (0),
    m_lastTxTime (Seconds (0)),
    m_minHolDelay (0.0),
    m_maxHolDelay (0.0),
    m_avgHolDelay (0.0),
    m_nHolDelaySamples (0),
    m_verbose (false),
    m_nBasicTriggerFramesSent (0),
    m_nFailedTriggerFrames (0),
    m_minLengthRatio (0.0),
    m_maxLenghtRatio (0.0),
    m_avgLengthRatio (0.0),
    m_tfUlLength (Seconds (0)),
    m_overallTimeGrantedByTf (Seconds (0)),
    m_responsesToLastTfDuration (Seconds (0))
{
}

void
WifiDlOfdmaExample::Config (int argc, char *argv[])
{
  NS_LOG_FUNCTION (this);

  CommandLine cmd;
  cmd.AddValue ("payloadSize", "Payload size in bytes", m_payloadSize);
  cmd.AddValue ("simulationTime", "Simulation time in seconds", m_simulationTime);
  cmd.AddValue ("nStations", "Number of non-AP stations", m_nStations);
  cmd.AddValue ("radius", "Radius of the disc centered in the AP and containing all the non-AP STAs", m_radius);
  cmd.AddValue ("enableDlOfdma", "Enable/disable DL OFDMA", m_enableDlOfdma);
  cmd.AddValue ("forceDlOfdma", "The RR scheduler always returns DL OFDMA", m_forceDlOfdma);
  cmd.AddValue ("dlAckType", "Ack sequence type for DL OFDMA (1-3)", m_dlAckSeqType);
  cmd.AddValue ("enableUlOfdma", "The RR scheduler returns UL OFDMA after DL OFDMA", m_enableUlOfdma);
  cmd.AddValue ("enableBsrp", "Enable Buffer Status Report TF", m_enableBsrp);
  cmd.AddValue ("useCentralRus", "Enable using Central 26 Tones Rus", m_useCentral26TonesRus);
  cmd.AddValue ("ulPsduSize", "Max size in bytes of HE TB PPDUs", m_ulPsduSize);
  cmd.AddValue ("channelWidth", "Channel bandwidth (20, 40, 80, 160)", m_channelWidth);
  cmd.AddValue ("guardInterval", "Guard Interval (800, 1600, 3200)", m_guardInterval);
  cmd.AddValue ("maxRus", "Maximum number of RUs allocated per DL MU PPDU", m_maxNRus);
  cmd.AddValue ("mcs", "The constant MCS value to transmit HE PPDUs", m_mcs);
  cmd.AddValue ("maxAmsduSize", "Maximum A-MSDU size", m_maxAmsduSize);
  cmd.AddValue ("maxAmpduSize", "Maximum A-MPDU size", m_maxAmpduSize);
  cmd.AddValue ("txopLimit", "TXOP duration in microseconds", m_txopLimit);
  cmd.AddValue ("queueSize", "Maximum size of a WifiMacQueue (packets)", m_macQueueSize);
  cmd.AddValue ("msduLifetime", "Maximum MSDU lifetime in milliseconds", m_msduLifetime);
  cmd.AddValue ("continueTxop", "Continue TXOP if no SU response after MU PPDU", m_continueTxop);
  cmd.AddValue ("baBufferSize", "Block Ack buffer size", m_baBufferSize);
//   cmd.AddValue ("enableRts", "Enable or disable RTS/CTS", m_enableRts);
  cmd.AddValue ("dataRate", "Per-station data rate (Mb/s)", m_dataRate);
  cmd.AddValue ("transport", "Transport layer protocol (Udp/Tcp)", m_transport);
  cmd.AddValue ("queueDisc", "Queuing discipline to install on the AP (default/none)", m_queueDisc);
  cmd.AddValue ("warmup", "Duration of the warmup period (seconds)", m_warmup);
  cmd.AddValue ("enablePcap", "Enable PCAP trace file generation.", m_enablePcap);
  cmd.AddValue ("verbose", "Enable/disable all Wi-Fi debug traces", m_verbose);
  cmd.Parse (argc, argv);

  srand(time(0));
  if ( m_mcs == 0 )
    m_mcs = 1 + ( std::rand () % 10 + 1 );

  uint64_t phyRate = HePhy::GetHeMcs (m_mcs).GetDataRate (m_channelWidth, m_guardInterval, 1);
  // Estimate the A-MPDU size as the number of bytes transmitted at the PHY rate in
  // an interval equal to the maximum PPDU duration
  uint32_t ampduSize = phyRate * GetPpduMaxTime (WIFI_PREAMBLE_HE_SU).GetSeconds () / 8;  // bytes
  // Estimate the number of MSDUs per A-MPDU as the ratio of the A-MPDU size to the MSDU size
  uint32_t nMsdus = ampduSize / m_payloadSize;
  // AP's EDCA queue must contain the number of MSDUs per A-MPDU times the number of stations,
  // times a surplus coefficient
  uint32_t queueSize = nMsdus * m_nStations * 2 /* surplus */;
  // The MSDU lifetime must exceed the time taken by the AP to empty its EDCA queue at the PHY rate
  uint32_t msduLifetime = queueSize * m_payloadSize * 8 * 1000. / phyRate * 2 /* surplus */;

  if (m_macQueueSize == 0)
    {
      m_macQueueSize = queueSize;
    }
  if (m_msduLifetime == 0)
    {
      m_msduLifetime = msduLifetime;
    }
  if (m_dataRate == 0)
    {
      m_dataRate = phyRate * 1.2 /* surplus */ / 1e6 / m_nStations;
    }

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
            << "maxRus = " << m_maxNRus << std::endl
            << "Number of stations = " << m_nStations << std::endl
            << "Data rate = " << m_dataRate << " Mbps" << std::endl
            << "EDCA queue max size = " << m_macQueueSize << " MSDUs" << std::endl
            << "MSDU lifetime = " << m_msduLifetime << " ms" << std::endl
            << "BA buffer size = " << m_baBufferSize << std::endl;
  if (m_enableDlOfdma)
    {
      std::cout << "Ack sequence = " << m_dlAckSeqType << std::endl;
    }
  else
    {
      std::cout << "No OFDMA" << std::endl;
    }
  std::cout << std::endl;
}

void
WifiDlOfdmaExample::Setup (void)
{
  NS_LOG_FUNCTION (this);
  LogComponentEnable ("WifiDlOfdmaExample", LOG_INFO);

  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", m_enableRts ? StringValue ("0") : StringValue ("999999"));
  Config::SetDefault ("ns3::HeConfiguration::GuardInterval", TimeValue (NanoSeconds (m_guardInterval)));
  //Config::SetDefault ("ns3::WifiPhy::GuardInterval", TimeValue (NanoSeconds (m_guardInterval)));
  //Config::SetDefault ("ns3::RegularWifiMac::ContinueTxopIfNoSuResponseAfterMuPpdu", BooleanValue (m_continueTxop));
  Config::SetDefault ("ns3::ArpCache::AliveTimeout", TimeValue (Seconds (3600 * 24))); // ARP cache entries expire after one day
  Config::SetDefault ("ns3::WifiMacQueue::MaxSize", QueueSizeValue (QueueSize (PACKETS, m_macQueueSize)));
  Config::SetDefault ("ns3::WifiMacQueue::MaxDelay", TimeValue (MilliSeconds (m_msduLifetime)));
  Config::SetDefault ("ns3::HeConfiguration::MpduBufferSize", UintegerValue (m_baBufferSize));

  m_staNodes.Create (m_nStations);
  m_apNodes.Create (1);

  Ptr<MultiModelSpectrumChannel> spectrumChannel = CreateObject<MultiModelSpectrumChannel> ();
  //Ptr<FriisPropagationLossModel> lossModel = CreateObject<FriisPropagationLossModel> ();
  //spectrumChannel->AddPropagationLossModel (lossModel);
  Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel> ();
  spectrumChannel->SetPropagationDelayModel (delayModel);
  SpectrumWifiPhyHelper phy;
  phy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);
  phy.SetChannel (spectrumChannel);
  phy.Set ("ChannelNumber", UintegerValue (m_channelNumber));
  phy.Set ("ChannelWidth", UintegerValue (m_channelWidth));

  WifiHelper wifi;
  if (m_verbose)
    {
      wifi.EnableLogComponents ();
    }
  wifi.SetStandard (WIFI_STANDARD_80211ax_5GHZ);
  std::ostringstream oss;
  oss << "HeMcs" << m_mcs;
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode", StringValue (oss.str ()),
                                "ControlMode", StringValue (oss.str ()));
  switch (m_dlAckSeqType)
    {
    case 1:
      //wifi.SetAckPolicySelectorForAc (AC_BE, "ns3::ConstantWifiAckPolicySelector", "DlAckSequenceType", UintegerValue (DlMuAckSequenceType::DL_SU_FORMAT));
      Config::SetDefault ("ns3::WifiDefaultAckManager::DlMuAckSequenceType", EnumValue (WifiAcknowledgment::DL_MU_BAR_BA_SEQUENCE));
      break;
    case 2:
      //wifi.SetAckPolicySelectorForAc (AC_BE, "ns3::ConstantWifiAckPolicySelector", "DlAckSequenceType", UintegerValue (DlMuAckSequenceType::DL_MU_BAR));
      Config::SetDefault ("ns3::WifiDefaultAckManager::DlMuAckSequenceType", EnumValue (WifiAcknowledgment::DL_MU_TF_MU_BAR));
      break;
    case 3:
      //wifi.SetAckPolicySelectorForAc (AC_BE, "ns3::ConstantWifiAckPolicySelector", "DlAckSequenceType", UintegerValue (DlMuAckSequenceType::DL_AGGREGATE_TF));
      Config::SetDefault ("ns3::WifiDefaultAckManager::DlMuAckSequenceType", EnumValue (WifiAcknowledgment::DL_MU_AGGREGATE_TF));
      break;

    default:
      NS_FATAL_ERROR ("Invalid DL ack sequence type (must be 1, 2 or 3)");
    }

  WifiMacHelper mac;
  if (m_enableDlOfdma)
    {
      /*
      mac.SetOfdmaManager ("ns3::RrOfdmaManager",
                           "NStations", UintegerValue (m_maxNRus),
                           "ForceDlOfdma", BooleanValue (m_forceDlOfdma),
                           "EnableUlOfdma", BooleanValue (m_enableUlOfdma),
                           "UlPsduSize", UintegerValue (m_ulPsduSize));
      */

      mac.SetMultiUserScheduler ("ns3::RrMultiUserScheduler",
                            "NStations", UintegerValue(m_maxNRus),
                            "ForceDlOfdma", BooleanValue (m_forceDlOfdma),
                            "EnableUlOfdma", BooleanValue (m_enableUlOfdma),
                            "UlPsduSize", UintegerValue (m_ulPsduSize),
                            "EnableBsrp", BooleanValue(m_enableBsrp),
                            "UseCentral26TonesRus", BooleanValue(m_useCentral26TonesRus));
    }

  mac.SetType ("ns3::StaWifiMac",
              "Ssid", SsidValue (Ssid ("non-existing-ssid")));  // prevent stations from automatically associating
  m_staDevices = wifi.Install (phy, mac, m_staNodes);

  mac.SetType ("ns3::ApWifiMac",
               "Ssid", SsidValue (m_ssid));
  m_apDevices = wifi.Install (phy, mac, m_apNodes);

  // Configure max A-MSDU size and max A-MPDU size on the AP
  Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice> (m_apDevices.Get (0));
  Ptr<RegularWifiMac> regularMac = DynamicCast<RegularWifiMac>(dev->GetMac());
  regularMac->SetAttribute ("BE_MaxAmsduSize", UintegerValue (m_maxAmsduSize));
  regularMac->SetAttribute ("BE_MaxAmpduSize", UintegerValue (m_maxAmpduSize));
  m_channelCenterFrequency = dev->GetPhy ()->GetFrequency ();
  // Configure TXOP Limit on the AP
  PointerValue ptr;
  dev->GetMac ()->GetAttribute ("BE_Txop", ptr);
  ptr.Get<QosTxop> ()->SetTxopLimit (MicroSeconds (m_txopLimit));

  // Increasing the contention window size for the stations to provide the AP
  // with more frequent access to the channel to solicit UL/DL Txs
  for ( uint32_t i = 0; i < m_staDevices.GetN(); i++ ) {
    Ptr<WifiNetDevice> devSta = DynamicCast<WifiNetDevice> (m_staDevices.Get(i));
    Ptr<RegularWifiMac> regularWifiMac = DynamicCast<RegularWifiMac>(devSta->GetMac());
    regularWifiMac->ConfigureContentionWindow(1024, 4096);
  }

  Ptr<RegularWifiMac> regularMac = DynamicCast<RegularWifiMac>(dev->GetMac());
  // Configure max A-MSDU size and max A-MPDU size on the stations
  for (uint32_t i = 0; i < m_staNodes.GetN (); i++)
    {
      dev = DynamicCast<WifiNetDevice> (m_staDevices.Get (i));
      dev->GetMac ()->SetAttribute ("BE_MaxAmsduSize", UintegerValue (m_maxAmsduSize));
      dev->GetMac ()->SetAttribute ("BE_MaxAmpduSize", UintegerValue (m_maxAmpduSize));
      m_dlStats[dev->GetMac ()->GetAddress ()] = DlStats ();
      m_ulStats[dev->GetMac ()->GetAddress ()] = UlStats ();
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

  m_apInterface = address.Assign (m_apDevices);
  m_staInterfaces = address.Assign (m_staDevices);

  /* Traffic Control layer */
  TrafficControlHelper tch;
  if (m_queueDisc.compare ("default") != 0)
    {
      // Uninstall the root queue disc on the AP netdevice
      tch.Uninstall (m_apDevices);
    }

  /* Transport and application layer */
  
  ThreeGppHttpClientHelper client (m_apInterface.GetAddress(0));
  m_httpClientApps.Add(client.Install(m_staNodes.Get(0)));
  m_httpClientApps.Add(client.Install(m_staNodes.Get(1)));
  m_httpClientApps.Add(client.Install(m_staNodes.Get(2)));
  m_httpClientApps.Stop (Seconds (m_warmup + m_simulationTime));


  for (int i = 0; i < 3; ++i)
  {
  	Ptr<ThreeGppHttpClient> httpClient = m_httpClientApps.Get (i)->GetObject<ThreeGppHttpClient> ();
	
    httpClient->TraceConnectWithoutContext ("RxMainObject", MakeCallback (&ClientMainObjectReceived));
    httpClient->TraceConnectWithoutContext ("RxEmbeddedObject", MakeCallback (&ClientEmbeddedObjectReceived));
    httpClient->TraceConnectWithoutContext ("Rx", MakeCallback (&ClientRx));
  }

  std::string socketType = (m_transport.compare ("Tcp") == 0 ? "ns3::TcpSocketFactory" : "ns3::UdpSocketFactory");
  Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (m_payloadSize));

  PacketSinkHelper packetSinkHelper (socketType, InetSocketAddress (Ipv4Address::GetAny (), m_port));
  for (int i = 3; i < m_nStations; ++i)
  {
  	m_sinkApps.Add(packetSinkHelper.Install (m_staNodes.Get(i))); // i
  }

  m_sinkApps.Stop (Seconds (m_warmup + m_simulationTime)); // let the server be active for a long time

  m_rxStart.assign (m_nStations, 0.0);
  m_rxStop.assign (m_nStations, 0.0);

  for (uint16_t i = 0; i < m_nStations; i++)
    {
      m_appLatencyMap.insert (std::make_pair (i, std::vector<Time> ()));
    }

  Config::ConnectWithoutContext ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::StaWifiMac/Assoc",
                                 MakeCallback (&WifiDlOfdmaExample::EstablishBaAgreement, this));

  if (m_enablePcap)
    {
      //AsciiTraceHelper ascii;
      //phy.EnableAscii(ascii.CreateFileStream("AP_ascii.tr"), m_apDevices);
      //phy.EnablePcap ("STA_pcap", m_staDevices);
      //phy.EnablePcap ("AP_pcap", m_apDevices);
    }
}

void
WifiDlOfdmaExample::Run (void)
{
  // Ptr<FlowMonitor> flowMonitor;
  // FlowMonitorHelper flowHelper;
  // flowMonitor = flowHelper.InstallAll();
  // Start the setup phase by having the first station associate with the AP
  Simulator::ScheduleNow (&WifiDlOfdmaExample::StartAssociation, this);

  Simulator::Stop (Seconds (m_warmup + m_simulationTime + 100));
  Simulator::Run ();
  // flowMonitor->SerializeToXmlFile("NameOfFile.xml", true, true);

  std::cout << "Simulation Complete\n";
  std::cout << "MCS = " << m_mcs <<"\n";
  std::cout << "maxRus = " << m_maxNRus << "\n";
  
  double totalTput = 0.0;
  double BSCtotalTput[10];
  for (int i = 0; i < 10; i++) BSCtotalTput[i] = 0.0;
  double tput;
  std::cout << "Throughput (Mbps)" << std::endl
            << "-----------------" << std::endl;
  for (uint32_t i = 0; i < m_staNodes.GetN (); i++)
    {
      tput = ((m_rxStop[i] - 0.0) * 8.) / (m_simulationTime * 1e6);
      // if (i < 10) {
      //     tput = ((m_rxStop[i] - 0.0) * 8.) / (m_simulationTime * 1e6);
      //     BSCtotalTput[i] += tput;
      // } else {
      //     tput = ((m_rxStop[i] - m_rxStart[i]) * 8.) / (m_simulationTime * 1e6);
      // }
      totalTput += tput;
      std::cout << "STA_" << i << ": " << tput << " ";
    }

  for (int i = 0; i < NUM_BULK_SEND_CLIENTS; i++) {
    std::cout << "BSC1: " << BSCtotalTput[i] << "\n";
  }
  std::cout << std::endl << std::endl << "Total throughput: " << totalTput << std::endl;

  uint64_t totalFailed = 0;
  uint64_t failed;
  std::cout << std::endl << "TX failures" << std::endl
                         << "-----------" << std::endl;
  for (uint32_t i = 0; i < m_staNodes.GetN (); i++)
    {
      auto it = m_dlStats.find (DynamicCast<WifiNetDevice> (m_staDevices.Get (i))->GetMac ()->GetAddress ());
      NS_ASSERT (it != m_dlStats.end ());
      failed = it->second.failed;
      totalFailed += failed;
      std::cout << "STA_" << i << ": " << failed << " ";
    }
  std::cout << std::endl << std::endl << "Total failed: " << totalFailed << std::endl;

  uint64_t totalExpired = 0;
  uint64_t expired;
  std::cout << std::endl << "Expired MSDUs" << std::endl
                         << "-------------" << std::endl;
  for (uint32_t i = 0; i < m_staNodes.GetN (); i++)
    {
      auto it = m_dlStats.find (DynamicCast<WifiNetDevice> (m_staDevices.Get (i))->GetMac ()->GetAddress ());
      NS_ASSERT (it != m_dlStats.end ());
      expired = it->second.expired;
      totalExpired += expired;
      std::cout << "STA_" << i << ": " << expired << " ";
    }
  std::cout << std::endl << std::endl << "Total expired: " << totalExpired << std::endl;

  std::cout << std::endl << "(Min,Max,Count) A-MPDU size" << std::endl
                         << "---------------------------" << std::endl;
  for (uint32_t i = 0; i < m_staNodes.GetN (); i++)
    {
      auto it = m_dlStats.find (DynamicCast<WifiNetDevice> (m_staDevices.Get (i))->GetMac ()->GetAddress ());
      NS_ASSERT (it != m_dlStats.end ());
      std::cout << "STA_" << i << ": (" << it->second.minAmpduSize << "," << it->second.maxAmpduSize
                               << "," << it->second.nAmpdus << ") ";
    }

  std::cout << std::endl << std::endl << "Maximum TXOP duration: " << m_maxTxop.ToDouble (Time::MS) << "ms" << std::endl;

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

  std::cout << std::endl << "(Min,Max,Avg) Pairwise head-of-line delay (ms)" << std::endl
                         << "----------------------------------------------" << std::endl;
  for (uint32_t i = 0; i < m_staNodes.GetN (); i++)
    {
      auto it = m_dlStats.find (DynamicCast<WifiNetDevice> (m_staDevices.Get (i))->GetMac ()->GetAddress ());
      NS_ASSERT (it != m_dlStats.end ());
      std::cout << std::fixed << std::setprecision (3)
                << "STA_" << i << ": (" << it->second.minHolDelay << ", " << it->second.maxHolDelay
                               << ", " << it->second.avgHolDelay << ") ";
    }

  std::cout << std::endl << std::endl << "Head-of-line delay (ms): ("
                                      << m_minHolDelay << ", "
                                      << m_maxHolDelay << ", "
                                      << m_avgHolDelay << ")" << std::endl;

  std::cout << std::endl << "Average latency (ms)" << std::endl
                         << "--------------------" << std::endl;

  for (uint32_t i = 0; i < m_staNodes.GetN (); i++)
    {
      auto it = m_appLatencyMap.find (i);
      NS_ASSERT (it != m_appLatencyMap.end ());
      double average_latency_ms = (std::accumulate (it->second.begin (), it->second.end (), NanoSeconds (0))).ToDouble (Time::MS) / it->second.size ();
      std::cout << "STA_" << i << ": " << average_latency_ms << " ";
    }

  std::cout << std::endl << std::endl << "Unresponded TFs ratio/(Min,Max,Avg) HE TB PPDU duration to UL Length ratio"
                         << std::endl << "--------------------------------------------------------------------------"
                         << std::endl;
  for (uint32_t i = 0; i < m_staNodes.GetN (); i++)
    {
      auto it = m_ulStats.find (DynamicCast<WifiNetDevice> (m_staDevices.Get (i))->GetMac ()->GetAddress ());
      NS_ASSERT (it != m_ulStats.end ());
      double unrespondedTfRatio = 0.0;
      if (it->second.nSolicitingTriggerFrames > 0)
        {
          unrespondedTfRatio = static_cast<double> (it->second.nSolicitingTriggerFrames - it->second.nLengthRatioSamples)
                               / it->second.nSolicitingTriggerFrames;
        }

      std::cout << std::fixed << std::setprecision (3)
                << "STA_" << i << ": " << unrespondedTfRatio << "/(" << it->second.minLengthRatio
                               << ", " << it->second.maxLenghtRatio
                               << ", " << it->second.avgLengthRatio << ") ";
    }

  std::cout << std::endl << std::endl << "(Failed, Sent) Basic Trigger Frames: ("
                                      << m_nFailedTriggerFrames << ", "
                                      << m_nBasicTriggerFramesSent << ")" << std::endl;

  uint64_t heTbPPduTotalCount = 0;
  uint64_t solicitingTriggerFrames = 0;
  for (auto& ulStaStats : m_ulStats)
    {
      heTbPPduTotalCount += ulStaStats.second.nLengthRatioSamples;
      solicitingTriggerFrames += ulStaStats.second.nSolicitingTriggerFrames;
    }
  double missingHeTbPpduRatio = 0.0;
  if (solicitingTriggerFrames > 0)
    {
      missingHeTbPpduRatio = static_cast<double> (solicitingTriggerFrames - heTbPPduTotalCount)
                             / solicitingTriggerFrames;
    }
  std::cout << std::endl << "Missing HE TB PPDUs ratio: " << missingHeTbPpduRatio << std::endl;
  std::cout << std::endl << "HE TB PPDU completeness: ("
                         << m_minLengthRatio << ", "
                         << m_maxLenghtRatio << ", "
                         << m_avgLengthRatio << ")" << std::endl << std::endl;

  m_appPacketTxMap.clear ();
  m_appLatencyMap.clear ();

  Simulator::Destroy ();
}

void
WifiDlOfdmaExample::StartAssociation (void)
{
  NS_LOG_FUNCTION (this << m_currentSta);
  NS_ASSERT (m_currentSta < m_nStations);
  // for (int i = 0; i < m_nStations; ++i)
  // {
	  std::cout << "Station no. " << m_currentSta << " is associated with the AP\n";
	  Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice> (m_staDevices.Get (m_currentSta));
	  NS_ASSERT (dev != 0);
	  dev->GetMac ()->SetSsid (m_ssid); // this will lead the station to associate with the AP
  // }
}

void
WifiDlOfdmaExample::EstablishBaAgreement (Mac48Address bssid)
{
  //std::cout << "bssid = " << bssid << "\n";
  std::cout << "Establish BA Agreement for client " << m_currentSta << "\n";
  NS_LOG_FUNCTION (this << bssid << m_currentSta);
  
  // Now that the current station is associated with the AP, let's trigger the creation
  // of an entry in the ARP cache (of both the AP and the STA) and the establishment of
  // a Block Ack agreement between the AP and the STA (and viceversa). This is done by
  // having the AP send 3 ICMP Echo Requests to the STA
  Time pingDuration = MilliSeconds (125);

  V4PingHelper ping (m_staInterfaces.GetAddress (m_currentSta));
  ping.SetAttribute ("Interval", TimeValue (MilliSeconds (50)));
  if (m_verbose)
    {
      ping.SetAttribute ("Verbose", BooleanValue (true));
    }
  ApplicationContainer pingApps = ping.Install (m_apNodes);
  pingApps.Stop (pingDuration);

  // Install a client application on the current station. In case of TCP traffic,
  // this will trigger the establishment of a TCP connection. The client application
  // is initially quiet (i.e., it does not transmit packets -- this is achieved
  // by setting the duration of the "On" interval to zero).
  if ( m_currentSta >= 3 ) { // only for stations 2, 3 [ON/OFF]
    uint16_t offInterval = 10;  // milliseconds
    std::stringstream ss;
    ss << "ns3::ConstantRandomVariable[Constant=" << std::fixed << static_cast<double> (offInterval / 1000.) << "]";

    std::string socketType = (m_transport.compare ("Tcp") == 0 ? "ns3::TcpSocketFactory" : "ns3::UdpSocketFactory");
    OnOffHelper client (socketType, Ipv4Address::GetAny ());
    client.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
    client.SetAttribute ("OffTime", StringValue (ss.str ()));

    if ( m_currentSta >= 12 ) // Gaming
      client.SetAttribute ("DataRate", DataRateValue (DataRate (1.5 * 1e6)));
    else // Video Call + Security Camera
      client.SetAttribute ("DataRate", DataRateValue (DataRate (3 * 1e6)));

    client.SetAttribute ("PacketSize", UintegerValue (m_payloadSize));

    InetSocketAddress dest (m_staInterfaces.GetAddress (m_currentSta), m_port);
    // dest.SetTos (0xb8); //AC_VI
    client.SetAttribute ("Remote", AddressValue (dest));

    // Make sure that the client application is started at a time that is an integer
    // multiple of the offInterval. In fact, the client application will wake every
    // offInterval milliseconds to check if a packet can be sent. Given that the duration
    // of the "On" interval will be modified for all the client applications
    // simultaneously, this ensures that all the client applications will actually
    // start sending packets at the same time.
    uint64_t startTime = std::ceil (Simulator::Now ().ToDouble (Time::MS) / offInterval) * offInterval;

    Simulator::Schedule (MilliSeconds (static_cast<uint64_t> (startTime) + 110) - Simulator::Now (),
                        &WifiDlOfdmaExample::StartClient, this, client);
  }
  if ( m_currentSta == 0 ) {

    //ThreeGppHttpServerHelper server(m_apInterface.GetAddress(0));
    //Simulator::Schedule (Seconds(m_warmup + 0.5), &WifiDlOfdmaExample::StartHttpServer, this, server);
  }

  // continue with the next station, if any is remaining
  if (++m_currentSta < m_nStations)
    {
      Simulator::Schedule (pingDuration, &WifiDlOfdmaExample::StartAssociation, this);
    }
  else
    {
      ThreeGppHttpServerHelper server(m_apInterface.GetAddress(0));
      Simulator::Schedule (pingDuration, &WifiDlOfdmaExample::StartTraffic, this, server);
      //Simulator::Schedule (pingDuration, &WifiDlOfdmaExample::StartTraffic, this);
      // Simulator::Schedule (pingDuration, &WifiDlOfdmaExample::StartStatistics, this);
    }
}

// Install the On/Off App on the AP corresponding to all the packet sink apps on stations
// (HTTP server already installed)
void
WifiDlOfdmaExample::StartClient (OnOffHelper client)
{
  NS_LOG_FUNCTION (this << m_currentSta);

  m_onOffSourceApps.Add (client.Install (m_apNodes));
  m_onOffSourceApps.Stop (Seconds (m_warmup + m_simulationTime)); // let clients be active for a long time
}

void
WifiDlOfdmaExample::StartHttpServer(ThreeGppHttpServerHelper serverHelper) {

  m_httpServerApp.Add(serverHelper.Install (m_apNodes));
  m_httpServerApp.Stop (Seconds (m_warmup + m_simulationTime));
  std::cout << "Server Installed on: " << m_apInterface.GetAddress(0) << "\n";
  Ptr<ThreeGppHttpServer> httpServer = m_httpServerApp.Get (0)->GetObject<ThreeGppHttpServer> ();

  // Example of connecting to the trace sources
  //httpServer->TraceConnectWithoutContext ("ConnectionEstablished", MakeCallback (&ServerConnectionEstablished));
  //httpServer->TraceConnectWithoutContext ("MainObject", MakeCallback (&MainObjectGenerated));
  //httpServer->TraceConnectWithoutContext ("EmbeddedObject", MakeCallback (&EmbeddedObjectGenerated));
  //httpServer->TraceConnectWithoutContext ("Tx", MakeCallback (&ServerTx));


  PointerValue varPtr1;
  httpServer->GetAttribute ("Variables", varPtr1);
  Ptr<ThreeGppHttpVariables> httpVariables1 = varPtr1.Get<ThreeGppHttpVariables> ();
  httpVariables1->SetMainObjectSizeMean (1024 * 100 * 10); // 100kB
  httpVariables1->SetMainObjectSizeStdDev (4); // 40kB
}

void
WifiDlOfdmaExample::StartBulkClient(BulkSendHelper client) {

    m_bulkSourceApps.Add(client.Install (m_apNodes));
    m_bulkSourceApps.Stop (Seconds (m_warmup + m_simulationTime));
}

//void
//WifiDlOfdmaExample::StartTraffic(void) 
void
WifiDlOfdmaExample::StartTraffic (ThreeGppHttpServerHelper serverHelper)
{
  m_httpServerApp.Add(serverHelper.Install (m_apNodes));
  m_httpServerApp.Stop (Seconds (m_warmup + m_simulationTime));
  std::cout << "Server Installed on: " << m_apInterface.GetAddress(0) << "\n";
  Ptr<ThreeGppHttpServer> httpServer = m_httpServerApp.Get (0)->GetObject<ThreeGppHttpServer> ();

  // Example of connecting to the trace sources
  httpServer->TraceConnectWithoutContext ("ConnectionEstablished", MakeCallback (&ServerConnectionEstablished));
  httpServer->TraceConnectWithoutContext ("MainObject", MakeCallback (&MainObjectGenerated));
  httpServer->TraceConnectWithoutContext ("EmbeddedObject", MakeCallback (&EmbeddedObjectGenerated));
  httpServer->TraceConnectWithoutContext ("Tx", MakeCallback (&ServerTx));


  PointerValue varPtr1;
  httpServer->GetAttribute ("Variables", varPtr1);
  Ptr<ThreeGppHttpVariables> httpVariables1 = varPtr1.Get<ThreeGppHttpVariables> ();
  httpVariables1->SetMainObjectSizeMean (1024 * 100 * 10); // 100kB
  httpVariables1->SetMainObjectSizeStdDev (4); // 40kB

  std::cout << "Traffic Started ...\n";
  NS_LOG_FUNCTION (this);

  // 
  uint32_t p = m_nStations - 3;
  //uint32_t p = m_nStations;
  for (uint32_t i = 0; i < p; i++)
    {
      Ptr<Application> sourceApp = m_onOffSourceApps.Get (i);
      sourceApp->SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
      sourceApp->SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
    }

  Simulator::Schedule (Seconds (m_warmup), &WifiDlOfdmaExample::StartStatistics, this);
}

void
WifiDlOfdmaExample::StartStatistics (void)
{
  std::cout << "*******************Start Statistics *****************\n";
  NS_LOG_FUNCTION (this);

  Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice> (m_apDevices.Get (0));
  PointerValue ptr;
  dev->GetMac ()->GetAttribute ("BE_Txop", ptr);

  // Trace TXOP duration for BE on the AP
  ptr.Get<QosTxop> ()->TraceConnectWithoutContext ("TxopTrace", MakeCallback (&WifiDlOfdmaExample::TxopDuration, this));
  // Trace expired MSDUs for BE on the AP
  ptr.Get<QosTxop> ()->GetWifiMacQueue ()->TraceConnectWithoutContext ("Expired", MakeCallback (&WifiDlOfdmaExample::NotifyMsduExpired, this));
  // Trace MSDUs dequeued from the BE EDCA queue on the AP
  ptr.Get<QosTxop> ()->GetWifiMacQueue ()->TraceConnectWithoutContext ("Dequeue",
                                                                       MakeCallback (&WifiDlOfdmaExample::NotifyMsduDequeuedFromEdcaQueue,
                                                                                     this));
  // Trace PSDUs forwarded down to the PHY on the AP
  //ptr.Get<QosTxop> ()->GetLow ()->TraceConnectWithoutContext ("ForwardDown", MakeCallback (&WifiDlOfdmaExample::NotifyPsduForwardedDown, this));
  // Trace TX failures on the AP
  DynamicCast<RegularWifiMac> (dev->GetMac ())->TraceConnectWithoutContext ("DroppedMpdu", MakeCallback (&WifiDlOfdmaExample::NotifyTxFailed, this));
  DynamicCast<RegularWifiMac> (dev->GetMac ())->TraceConnectWithoutContext("NAckedMpdu", MakeCallback (&WifiDlOfdmaExample::NotifyTxNAcked, this));
  // Retrieve the number of bytes received by each station until the end of the warmup period
  for (uint32_t i = 0; i < m_staDevices.GetN (); i++)
    {
      {
      if ( i < 3 ) {
        m_rxStart[i] = DynamicCast<ThreeGppHttpClient> (m_httpClientApps.Get (i))->GetTotalRx ();
      }
      else {
        m_rxStart[i] = DynamicCast<PacketSink> (m_sinkApps.Get (i - 3))->GetTotalRx ();
      }
      //m_rxStart[i] = DynamicCast<PacketSink> (m_sinkApps.Get (i))->GetTotalRx ();
    }

      std::cout << m_rxStart[i] << "bytes received by sink application " << i << " before the end of the warmup period\n";
    }

  // Trace PSDUs forwarded down to the PHY on each station
  for (uint32_t i = 0; i < m_staDevices.GetN (); i++)
    {
      dev = DynamicCast<WifiNetDevice> (m_staDevices.Get (i));
      dev->GetMac ()->GetAttribute ("BE_Txop", ptr);
      //ptr.Get<QosTxop> ()->GetLow ()->TraceConnectWithoutContext ("ForwardDown", MakeCallback (&WifiDlOfdmaExample::NotifyPsduForwardedDown, this));
    }

  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::WifiMac/MacTx", MakeCallback (&WifiDlOfdmaExample::NotifyApplicationTx, this));
  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::WifiMac/MacRx", MakeCallback (&WifiDlOfdmaExample::NotifyApplicationRx, this));

  Simulator::Schedule (Seconds (m_simulationTime), &WifiDlOfdmaExample::StopStatistics, this);
}

void
WifiDlOfdmaExample::StopStatistics (void)
{
  std::cout << "*******************Stop Statistics *****************\n";
  NS_LOG_FUNCTION (this);

  Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice> (m_apDevices.Get (0));
  PointerValue ptr;
  dev->GetMac ()->GetAttribute ("BE_Txop", ptr);

  // Stop tracing TXOP duration for BE on the AP
  ptr.Get<QosTxop> ()->TraceDisconnectWithoutContext ("TxopTrace", MakeCallback (&WifiDlOfdmaExample::TxopDuration, this));
  // Stop tracing expired MSDUs for BE on the AP
  ptr.Get<QosTxop> ()->GetWifiMacQueue ()->TraceDisconnectWithoutContext ("Expired", MakeCallback (&WifiDlOfdmaExample::NotifyMsduExpired, this));
  // Stop tracing MSDUs dequeued from the BE EDCA queue on the AP
  ptr.Get<QosTxop> ()->GetWifiMacQueue ()->TraceDisconnectWithoutContext ("Dequeue",
                                                                          MakeCallback (&WifiDlOfdmaExample::NotifyMsduDequeuedFromEdcaQueue,
                                                                                        this));
  // Stop tracing PSDUs forwarded down to the PHY on the AP
  //ptr.Get<QosTxop> ()->GetLow ()->TraceDisconnectWithoutContext ("ForwardDown", MakeCallback (&WifiDlOfdmaExample::NotifyPsduForwardedDown, this));
  // Stop tracing TX failures on the AP
  DynamicCast<RegularWifiMac> (dev->GetMac ())->TraceDisconnectWithoutContext ("DroppedMpdu", MakeCallback (&WifiDlOfdmaExample::NotifyTxFailed, this));
  DynamicCast<RegularWifiMac> (dev->GetMac ())->TraceDisConnectWithoutContext("NAckedMpdu", MakeCallback (&WifiDlOfdmaExample::NotifyTxNAcked, this));
  // Retrieve the number of bytes received by each station until the end of the simulation period
  for (uint32_t i = 0; i < m_staDevices.GetN (); i++)
    {
      {
      if ( i < 3 ) {
        m_rxStop[i] = DynamicCast<ThreeGppHttpClient> (m_httpClientApps.Get (i))->GetTotalRx ();
      }
      else {
        m_rxStop[i] = DynamicCast<PacketSink> (m_sinkApps.Get (i - 3))->GetTotalRx ();
      }
      //m_rxStop[i] = DynamicCast<PacketSink> (m_sinkApps.Get (i))->GetTotalRx ();
    }

      std::cout << m_rxStop[i] << " bytes received by sink application " << i << " at the end of the simulation\n";
    }

  // (Brutally) stop AP applications
  m_httpServerApp.Get(0)->Dispose();

  /*
  std::cout << "No http traffic";

  for (uint32_t i = 0; i < 2; i++)
  {
    std::cout << "No sigsev bulk " << i;
    m_bulkSourceApps.Get (i)->Dispose ();
  }

  for (uint32_t i = 0; i < 2; i++)
  {
    std::cout << "No sigsev onoff " << i;
    m_onOffSourceApps.Get (i)->Dispose ();
  }
  */

  // Stop tracing PSDUs forwarded down to the PHY on each station
  for (uint32_t i = 0; i < m_staDevices.GetN (); i++)
    {
      dev = DynamicCast<WifiNetDevice> (m_staDevices.Get (i));
      dev->GetMac ()->GetAttribute ("BE_Txop", ptr);
      //ptr.Get<QosTxop> ()->GetLow ()->TraceDisconnectWithoutContext ("ForwardDown", MakeCallback (&WifiDlOfdmaExample::NotifyPsduForwardedDown, this));
    }

  Config::Disconnect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::WifiMac/MacTx", MakeCallback (&WifiDlOfdmaExample::NotifyApplicationTx, this));
  Config::Disconnect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::WifiMac/MacRx", MakeCallback (&WifiDlOfdmaExample::NotifyApplicationRx, this));
}

void
WifiDlOfdmaExample::NotifyTxFailed (WifiMacDropReason reason, Ptr<const WifiMacQueueItem> mpdu)
{
  WifiMacHeader hdr = mpdu->GetHeader();
  auto it = m_dlStats.find (hdr.GetAddr1 ());
  NS_ASSERT (it != m_dlStats.end ());
  it->second.failed++;
}

void
WifiDlOfdmaExample::NotifyTxNAcked (Ptr<const WifiMacQueueItem> mpdu)
{
  WifiMacHeader hdr = mpdu->GetHeader();
  auto it = m_dlStats.find (hdr.GetAddr1 ());
  NS_ASSERT (it != m_dlStats.end ());
  it->second.failed++;
}

void
WifiDlOfdmaExample::NotifyMsduExpired (Ptr<const WifiMacQueueItem> item)
{
  auto it = m_dlStats.find (item->GetHeader ().GetAddr1 ());
  NS_ASSERT (it != m_dlStats.end ());
  it->second.expired++;
}

void
WifiDlOfdmaExample::NotifyMsduDequeuedFromEdcaQueue (Ptr<const WifiMacQueueItem> item)
{
  // std::cout << "**********************Notify ******************\n";
  Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice> (m_apDevices.Get (0));
  PointerValue ptr;
  dev->GetMac ()->GetAttribute ("BE_Txop", ptr);

  if (Simulator::Now () > item->GetTimeStamp () + ptr.Get<QosTxop> ()->GetWifiMacQueue ()->GetMaxDelay ())
    {
      // the MSDU lifetime is higher than the max queue delay, hence the MSDU has been
      // discarded. Do nothing in this case.
      return;
    }

  if (m_lastTxTime.IsStrictlyPositive ())
    {
      double newHolSample = (Simulator::Now () - m_lastTxTime).ToDouble (Time::MS);

      // if this is an MSDU that has been dequeued to be aggregated to a previously
      // dequeued MSDU, the HoL sample will be null. Do not count null HoL samples
      if (newHolSample > 0.0)
        {
          if (m_minHolDelay == 0.0 || newHolSample < m_minHolDelay)
            {
              m_minHolDelay = newHolSample;
            }
          if (newHolSample > m_maxHolDelay)
            {
              m_maxHolDelay = newHolSample;
            }
          m_avgHolDelay = (m_avgHolDelay * m_nHolDelaySamples + newHolSample) / (m_nHolDelaySamples + 1);
          m_nHolDelaySamples++;
        }
    }
  m_lastTxTime = Simulator::Now ();

  auto it = m_dlStats.find (item->GetHeader ().GetAddr1 ());
  NS_ASSERT (it != m_dlStats.end ());

  if (it->second.lastTxTime.IsStrictlyPositive ())
    {
      double newHolSample = (Simulator::Now () - it->second.lastTxTime).ToDouble (Time::MS);

      // if this is an MSDU that has been dequeued to be aggregated to a previously
      // dequeued MSDU, the HoL sample will be null. Do not count null HoL samples
      if (newHolSample > 0.0)
        {
          if (it->second.minHolDelay == 0.0 || newHolSample < it->second.minHolDelay)
            {
              it->second.minHolDelay = newHolSample;
            }
          if (newHolSample > it->second.maxHolDelay)
            {
              it->second.maxHolDelay = newHolSample;
            }
          it->second.avgHolDelay = (it->second.avgHolDelay * it->second.nHolDelaySamples + newHolSample) / (it->second.nHolDelaySamples + 1);
          it->second.nHolDelaySamples++;
        }
    }
  it->second.lastTxTime = Simulator::Now ();
}

void
WifiDlOfdmaExample::NotifyPsduForwardedDown (WifiPsduMap psduMap, WifiTxVector txVector)
{
  /*
  Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice> (m_apDevices.Get (0));
  Mac48Address apAddress = dev->GetMac ()->GetAddress ();

  if (psduMap.size () == 1 && psduMap.begin ()->second->GetAddr1 () == apAddress
      && psduMap.begin ()->second->GetHeader (0).IsQosData ())
    {
      // Uplink frame
      if (txVector.GetPreambleType () == WIFI_PREAMBLE_HE_TB)
        {
          // HE TB PPDU
          auto it = m_ulStats.find (psduMap.begin ()->second->GetAddr2 ());
          NS_ASSERT (it != m_ulStats.end ());
          //Time txDuration = WifiPhy::CalculateTxDuration (psduMap, txVector, m_channelCenterFrequency);
          Time txDuration = Seconds(2.0); // Doesn't matter, method not called
          m_responsesToLastTfDuration += txDuration;
          double currRatio = txDuration.GetSeconds () / m_tfUlLength.GetSeconds ();

          if (it->second.minLengthRatio == 0 || currRatio < it->second.minLengthRatio)
            {
              it->second.minLengthRatio = currRatio;
            }
          if (currRatio > it->second.maxLenghtRatio)
            {
              it->second.maxLenghtRatio = currRatio;
            }
          it->second.avgLengthRatio = (it->second.avgLengthRatio * it->second.nLengthRatioSamples + currRatio)
                                      / (it->second.nLengthRatioSamples + 1);
          it->second.nLengthRatioSamples++;
        }
    }
  // Downlink frame
  else if (psduMap.begin ()->second->GetHeader (0).IsQosData ())
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
  else if (psduMap.size () == 1 && psduMap.begin ()->second->GetHeader (0).IsTrigger ())
    {
      CtrlTriggerHeader trigger;
      psduMap.begin ()->second->GetPayload (0)->PeekHeader (trigger);

      if (trigger.IsBasic ())
        {
          if (m_tfUlLength.IsStrictlyPositive ())
            {
              // This is not the first Trigger Frame being sent
              if (m_responsesToLastTfDuration.IsZero ())
                {
                  // no station responded to the previous TF
                  m_nFailedTriggerFrames++;
                }
              else
                {
                  double currRatio = m_responsesToLastTfDuration.GetSeconds () / m_overallTimeGrantedByTf.GetSeconds ();

                  if (m_minLengthRatio == 0 || currRatio < m_minLengthRatio)
                    {
                      m_minLengthRatio = currRatio;
                    }
                  if (currRatio > m_maxLenghtRatio)
                    {
                      m_maxLenghtRatio = currRatio;
                    }
                  uint64_t samples = m_nBasicTriggerFramesSent - 1 - m_nFailedTriggerFrames;
                  m_avgLengthRatio = (m_avgLengthRatio * samples + currRatio) / (samples + 1);
                }
            }

          m_nBasicTriggerFramesSent++;
          m_responsesToLastTfDuration = Seconds (0);
          WifiTxVector heTbTxVector = trigger.GetHeTbTxVector (trigger.begin ()->GetAid12 ());
          m_tfUlLength = WifiPhy::ConvertLSigLengthToHeTbPpduDuration (trigger.GetUlLength (), heTbTxVector,
                                                                       m_channelCenterFrequency);
          m_overallTimeGrantedByTf = m_tfUlLength * trigger.GetNUserInfoFields ();

          dev = DynamicCast<WifiNetDevice> (m_apDevices.Get (0));
          Ptr<ApWifiMac> mac = DynamicCast<ApWifiMac> (dev->GetMac ());

          for (auto& userInfo : trigger)
            {
              Mac48Address address = mac->GetStaList ().at (userInfo.GetAid12 ());
              auto it = m_ulStats.find (address);
              NS_ASSERT (it != m_ulStats.end ());
              it->second.nSolicitingTriggerFrames++;
            }
        }
    }
    */
}

void
WifiDlOfdmaExample::TxopDuration (Time startTime, Time duration)
{
  if (duration > m_maxTxop)
    {
      m_maxTxop = duration;
    }
}

void
WifiDlOfdmaExample::NotifyApplicationTx (std::string context, Ptr<const Packet> p)
{
  if (p->GetSize () < m_payloadSize)
    {
      return;
    }
  m_appPacketTxMap.insert (std::make_pair (p->GetUid (), Simulator::Now ()));
}

void
WifiDlOfdmaExample::NotifyApplicationRx (std::string context, Ptr<const Packet> p)
{
  if (p->GetSize () < m_payloadSize)
    {
      return;
    }
  auto itTxPacket = m_appPacketTxMap.find (p->GetUid ());
  if (itTxPacket != m_appPacketTxMap.end ())
    {
      Time latency = (Simulator::Now () - itTxPacket->second);
      auto itStaLatencies = m_appLatencyMap.find (ContextToNodeId (context));
      NS_ASSERT (itStaLatencies != m_appLatencyMap.end ());
      itStaLatencies->second.push_back (latency); // Appending latency to vector of latencies
      m_appPacketTxMap.erase (itTxPacket);
    }
}

uint32_t
WifiDlOfdmaExample::ContextToNodeId (const std::string & context)
{
  std::string sub = context.substr (10);
  uint32_t pos = sub.find ("/Device");
  uint32_t nodeId = atoi (sub.substr (0, pos).c_str ());
  return nodeId;
}

int main (int argc, char *argv[])
{
  WifiDlOfdmaExample example;
  example.Config (argc, argv);
  example.Setup ();
  example.Run ();

  return 0;
}
