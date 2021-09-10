/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2020 Universita' degli Studi di Napoli Federico II
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
 * Author: Stefano Avallone <stavallo@unina.it>
 */

#include "ns3/log.h"
#include "drr-multi-user-scheduler.h"
#include "ns3/wifi-protection.h"
#include "ns3/wifi-acknowledgment.h"
#include "ns3/wifi-psdu.h"
#include "he-frame-exchange-manager.h"
#include "he-configuration.h"
#include "he-phy.h"
#include <algorithm>
#include <fstream>
#include <sstream>
#include <numeric>
#include <cstdlib>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("DrrMultiUserScheduler");

NS_OBJECT_ENSURE_REGISTERED (DrrMultiUserScheduler);

TypeId
DrrMultiUserScheduler::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::DrrMultiUserScheduler")
    .SetParent<MultiUserScheduler> ()
    .SetGroupName ("Wifi")
    .AddConstructor<DrrMultiUserScheduler> ()
    .AddAttribute ("NStations",
                   "The maximum number of stations that can be granted an RU in a DL MU OFDMA transmission",
                   UintegerValue (4),
                   MakeUintegerAccessor (&DrrMultiUserScheduler::m_nStations),
                   MakeUintegerChecker<uint16_t> (1, 74))
    .AddAttribute ("EnableTxopSharing",
                   "If enabled, allow A-MPDUs of different TIDs in a DL MU PPDU.",
                   BooleanValue (true),
                   MakeBooleanAccessor (&DrrMultiUserScheduler::m_enableTxopSharing),
                   MakeBooleanChecker ())
    .AddAttribute ("ForceDlOfdma",
                   "If enabled, return DL_MU_TX even if no DL MU PPDU could be built.",
                   BooleanValue (false),
                   MakeBooleanAccessor (&DrrMultiUserScheduler::m_forceDlOfdma),
                   MakeBooleanChecker ())
    .AddAttribute ("EnableUlOfdma",
                   "If enabled, return UL_MU_TX if DL_MU_TX was returned the previous time.",
                   BooleanValue (true),
                   MakeBooleanAccessor (&DrrMultiUserScheduler::m_enableUlOfdma),
                   MakeBooleanChecker ())
    .AddAttribute ("EnableBsrp",
                   "If enabled, send a BSRP Trigger Frame before an UL MU transmission.",
                   BooleanValue (true),
                   MakeBooleanAccessor (&DrrMultiUserScheduler::m_enableBsrp),
                   MakeBooleanChecker ())
    .AddAttribute ("UlPsduSize",
                   "The default size in bytes of the solicited PSDU (to be sent in a TB PPDU)",
                   UintegerValue (500),
                   MakeUintegerAccessor (&DrrMultiUserScheduler::m_ulPsduSize),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("UseCentral26TonesRus",
                   "If enabled, central 26-tone RUs are allocated, too, when the "
                   "selected RU type is at least 52 tones.",
                   BooleanValue (false),
                   MakeBooleanAccessor (&DrrMultiUserScheduler::m_useCentral26TonesRus),
                   MakeBooleanChecker ())
    .AddAttribute ("MaxCredits",
                   "Maximum amount of credits a station can have. When transmitting a DL MU PPDU, "
                   "the amount of credits received by each station equals the TX duration (in "
                   "microseconds) divided by the total number of stations. Stations that are the "
                   "recipient of the DL MU PPDU have to pay a number of credits equal to the TX "
                   "duration (in microseconds) times the allocated bandwidth share",
                   TimeValue (Seconds (1)),
                   MakeTimeAccessor (&DrrMultiUserScheduler::m_maxCredits),
                   MakeTimeChecker ())
  ;
  return tid;
}

DrrMultiUserScheduler::DrrMultiUserScheduler ()
  : m_hasDeadlineConstrainedTrafficStarted(false), m_currRound(0), m_havePacketsArrived(false), m_lastRoundTimestamp(0), m_roundsPerSchedule(0), m_packetsPerSchedule(0), m_ulTriggerType (TriggerFrameType::BASIC_TRIGGER)
{
  NS_LOG_FUNCTION (this);
}

DrrMultiUserScheduler::~DrrMultiUserScheduler ()
{
  NS_LOG_FUNCTION_NOARGS ();
}

void
DrrMultiUserScheduler::DoInitialize (void)
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT (m_apMac != nullptr);
  m_apMac->TraceConnectWithoutContext ("AssociatedSta",
                                       MakeCallback (&DrrMultiUserScheduler::NotifyStationAssociated, this));
  m_apMac->TraceConnectWithoutContext ("DeAssociatedSta",
                                       MakeCallback (&DrrMultiUserScheduler::NotifyStationDeassociated, this));
  for (const auto& ac : wifiAcList)
    {
      m_staList.insert ({ac.first, {}});
    }
  MultiUserScheduler::DoInitialize ();
}

void
DrrMultiUserScheduler::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
  m_staList.clear ();
  m_candidates.clear ();
  m_trigger = nullptr;
  m_txParams.Clear ();
  m_apMac->TraceDisconnectWithoutContext ("AssociatedSta",
                                          MakeCallback (&DrrMultiUserScheduler::NotifyStationAssociated, this));
  m_apMac->TraceDisconnectWithoutContext ("DeAssociatedSta",
                                          MakeCallback (&DrrMultiUserScheduler::NotifyStationDeassociated, this));
  MultiUserScheduler::DoDispose ();
}

void
DrrMultiUserScheduler::NotifyDeadlineConstrainedTrafficStarted(void)
{
  m_hasDeadlineConstrainedTrafficStarted = true;
}

uint32_t
DrrMultiUserScheduler::GetCurrRound(void) {
  return m_currRound;
}

void
DrrMultiUserScheduler::SetStaPacketInfo(std::map <uint32_t, std::vector<uint32_t>> packetInfo) {
  m_staPacketInfo = packetInfo;
}

void
DrrMultiUserScheduler::PassReferenceToOnDemandApps(ApplicationContainer apps) {
  m_OnDemandApps = apps;
}

uint32_t
DrrMultiUserScheduler::LCM(int arr[], int n)
{
    // Find the maximum value in arr[]
    int max_num = 0;
    for (int i=0; i<n; i++)
        if (max_num < arr[i])
            max_num = arr[i];
 
    // Initialize result
    uint32_t res = 1;
 
    // Find all factors that are present in
    // two or more array elements.
    int x = 2;  // Current factor.
    while (x <= max_num)
    {
        // To store indexes of all array
        // elements that are divisible by x.
        std::vector<int> indexes;
        for (int j=0; j<n; j++)
            if (arr[j]%x == 0)
                indexes.push_back(j);
 
        // If there are 2 or more array elements
        // that are divisible by x.
        if (indexes.size() >= 2)
        {
            // Reduce all array elements divisible
            // by x.
            for (int j=0; j< (int)indexes.size(); j++)
                arr[indexes[j]] = arr[indexes[j]]/x;
 
            res = res * x;
        }
        else
            x++;
    }
 
    // Then multiply all reduced array elements
    for (int i=0; i<n; i++)
        res = res*arr[i];
 
    return res;
}

uint32_t
DrrMultiUserScheduler::GetRoundsPerSchedule(void) {
  if ( m_roundsPerSchedule == 0 ) {

    // We assume that the packet info has arrived here
    int arr[m_nStations];
    for ( uint16_t i = 0; i < m_nStations; i++ ) {
      
      auto it = m_staPacketInfo.find(i);
      NS_ASSERT(it != m_staPacketInfo.end());

      arr[i] = it->second[0]; // Simply fill the time periods input by user into the array
    }

    m_roundsPerSchedule = LCM(arr, m_nStations); 
  }

  return m_roundsPerSchedule;
}

uint32_t
DrrMultiUserScheduler::GetPacketsPerSchedule(void) {
  if ( m_packetsPerSchedule == 0 ) {

    uint32_t packets = 0;
    for ( uint16_t i = 0; i < m_nStations; i++ ) {
      
      auto it = m_staPacketInfo.find(i);
      NS_ASSERT(it != m_staPacketInfo.end());
      
      packets = packets + ( GetRoundsPerSchedule() / it->second[0] );
    }

    m_packetsPerSchedule = packets;
  }

  return m_packetsPerSchedule;
}

void
DrrMultiUserScheduler::GeneratePacketScheduleForSetRounds(void) 
{
    m_packetSchedule.clear();
    for ( uint16_t i = 0; i < m_nStations; i++ ) {
      
      auto it = m_staPacketInfo.find(i);
      NS_ASSERT (it != m_staPacketInfo.end());

      uint32_t timePeriod = it->second[0];
      uint32_t deadline = it->second[1];
      uint32_t penalty = it->second[2];
      uint32_t packetsPerUser = ( GetRoundsPerSchedule() / timePeriod );

      int timePeriodFactor = 0;
      for ( uint32_t j = 0; j < packetsPerUser; j++ ) {

        //uint32_t arrivalRound = GetRoundFromTimestamp(currTimeUs) + timePeriodFactor * timePeriod;
        uint32_t arrivalRound = GetCurrRound() + timePeriodFactor * timePeriod;
        uint32_t deadlineRound = arrivalRound + deadline;
        std::vector<uint32_t> schedule{ arrivalRound, deadlineRound, penalty, i /* AID */ };

        m_packetSchedule.push_back(schedule);

        timePeriodFactor++;
      }
    }
}

HeRu::RuType
DrrMultiUserScheduler::GetRuTypePerRound(void) {

  //return HeRu::RU_484_TONE; // Testing

  uint32_t packetsPerSchedule = GetPacketsPerSchedule();

  switch(m_apMac->GetWifiPhy()->GetChannelWidth()) {
    case 20: {
      
      if ( packetsPerSchedule == 1 ) 
        return HeRu::RU_242_TONE;
      else if ( packetsPerSchedule == 2 )
        return HeRu::RU_106_TONE;
      else if ( packetsPerSchedule >= 3 && packetsPerSchedule <= 4 )
        return HeRu::RU_52_TONE;
      else
        return HeRu::RU_26_TONE;    
    }
    case 40: {

      if ( packetsPerSchedule == 1 ) 
        return HeRu::RU_484_TONE;
      else if ( packetsPerSchedule == 2 )
        return HeRu::RU_242_TONE;
      else if ( packetsPerSchedule >= 3 && packetsPerSchedule <= 4 )
        return HeRu::RU_106_TONE;
      else if ( packetsPerSchedule >= 5 && packetsPerSchedule <= 8 )
        return HeRu::RU_52_TONE;
      else
        return HeRu::RU_26_TONE;        
    }
    default: {
      NS_FATAL_ERROR("Only 40 Mhz and 20 Mhz supported with Deadline Aware Scheduler");
    }
  }

  return HeRu::RU_26_TONE; // Never called
}

uint32_t
DrrMultiUserScheduler::GetRusPerRound(HeRu::RuType ruType) {

  //return 1; // Testing

  switch(m_apMac->GetWifiPhy()->GetChannelWidth()) {
    case 20: {

      if ( ruType == HeRu::RU_242_TONE )
        return 1;
      else if ( ruType == HeRu::RU_106_TONE )
        return 2;
      else if ( ruType == HeRu::RU_52_TONE )
        return 4;
      else if ( ruType == HeRu::RU_26_TONE )
        return 9;  
    }
    case 40: {

      if ( ruType == HeRu::RU_484_TONE )
        return 1;
      else if ( ruType == HeRu::RU_242_TONE )
        return 2;
      else if ( ruType == HeRu::RU_106_TONE )
        return 4;
      else if ( ruType == HeRu::RU_52_TONE )
        return 8;
      else if ( ruType == HeRu::RU_26_TONE )
        return 18;    
    }
    default: {
      NS_FATAL_ERROR("Only 40 Mhz and 20 Mhz supported with Deadline Aware Scheduler");
    }
  }

  return 0; // Never called
}

uint32_t
DrrMultiUserScheduler::GetRuTypeIndex(HeRu::RuType ruType) {

  switch(m_apMac->GetWifiPhy()->GetChannelWidth()) {
    case 20: {

      if ( ruType == HeRu::RU_242_TONE )
        return 3;
      else if ( ruType == HeRu::RU_106_TONE )
        return 2;
      else if ( ruType == HeRu::RU_52_TONE )
        return 1;
      else if ( ruType == HeRu::RU_26_TONE )
        return 0;  
    }
    case 40: {

      if ( ruType == HeRu::RU_484_TONE )
        return 4;
      else if ( ruType == HeRu::RU_242_TONE )
        return 3;
      else if ( ruType == HeRu::RU_106_TONE )
        return 2;
      else if ( ruType == HeRu::RU_52_TONE )
        return 1;
      else if ( ruType == HeRu::RU_26_TONE )
        return 0;    
    }
    default: {
      NS_FATAL_ERROR("Only 40 Mhz and 20 Mhz supported with Deadline Aware Scheduler");
    }
  }

  return 0; // Never called
}

void
DrrMultiUserScheduler::StartNextRound(bool beginning) {
  if ( !beginning ) {
    
    m_currRound++;
    std::cout << "Time difference between rounds " << Simulator::Now().ToDouble(Time::US) - m_lastRoundTimestamp << std::endl;
  }

  m_lastRoundTimestamp = Simulator::Now().ToDouble(Time::US);
  
  if ( m_currRound >= 60) { // Only run for these many rounds

    m_hasDeadlineConstrainedTrafficStarted = false;
    return;
  }
    

  if ( m_currRound % GetRoundsPerSchedule() == 0 ) {
    for ( uint32_t i = 0; i < m_nStations; i++ ) {
      
      DynamicCast<OnDemandApplication>(m_OnDemandApps.Get(i))->SendPacket();
    }
  }
  else {

    for ( uint32_t i = 0; i < m_nStations; i++ ) {
      
      auto it = m_staPacketInfo.find(i);
      NS_ASSERT ( it != m_staPacketInfo.end() );

      if ( ( m_currRound % it->second[0] ) == 0 ) {
       
        DynamicCast<OnDemandApplication>(m_OnDemandApps.Get(i))->SendPacket();
      }
    }
  }

  //Simulator::Schedule(MilliSeconds(1), &DrrMultiUserScheduler::StartNextRound, this, false);
}

MultiUserScheduler::TxFormat
DrrMultiUserScheduler::SelectTxFormat (void)
{
  NS_LOG_FUNCTION (this);

  if (m_enableUlOfdma && m_enableBsrp && GetLastTxFormat () == DL_MU_TX)
    {
      return TrySendingBsrpTf ();
    }

  if (m_enableUlOfdma && (GetLastTxFormat () == DL_MU_TX
                          || m_ulTriggerType == TriggerFrameType::BSRP_TRIGGER))
    {
      TxFormat txFormat = TrySendingBasicTf ();

      if (txFormat != DL_MU_TX)
        {
          return txFormat;
        }
    }

  return TrySendingDlMuPpdu ();
}

MultiUserScheduler::TxFormat
DrrMultiUserScheduler::TrySendingBsrpTf (void)
{
  NS_LOG_FUNCTION (this);

  CtrlTriggerHeader trigger (TriggerFrameType::BSRP_TRIGGER, GetDlMuInfo ().txParams.m_txVector);

  WifiTxVector txVector = GetDlMuInfo ().txParams.m_txVector;
  txVector.SetGuardInterval (trigger.GetGuardInterval ());

  Ptr<Packet> packet = Create<Packet> ();
  packet->AddHeader (trigger);

  Mac48Address receiver = Mac48Address::GetBroadcast ();
  if (trigger.GetNUserInfoFields () == 1)
    {
      NS_ASSERT (m_apMac->GetStaList ().find (trigger.begin ()->GetAid12 ()) != m_apMac->GetStaList ().end ());
      receiver = m_apMac->GetStaList ().at (trigger.begin ()->GetAid12 ());
    }

  WifiMacHeader hdr (WIFI_MAC_CTL_TRIGGER);
  hdr.SetAddr1 (receiver);
  hdr.SetAddr2 (m_apMac->GetAddress ());
  hdr.SetDsNotTo ();
  hdr.SetDsNotFrom ();

  Ptr<WifiMacQueueItem> item = Create<WifiMacQueueItem> (packet, hdr);

  m_txParams.Clear ();
  // set the TXVECTOR used to send the Trigger Frame
  m_txParams.m_txVector = m_apMac->GetWifiRemoteStationManager ()->GetRtsTxVector (receiver);

  if (!m_heFem->TryAddMpdu (item, m_txParams, m_availableTime))
    {
      // sending the BSRP Trigger Frame is not possible, hence return NO_TX. In
      // this way, no transmission will occur now and the next time we will
      // try again sending a BSRP Trigger Frame.
      NS_LOG_DEBUG ("Remaining TXOP duration is not enough for BSRP TF exchange");
      return NO_TX;
    }

  // Compute the time taken by each station to transmit 8 QoS Null frames
  Time qosNullTxDuration = Seconds (0);
  for (const auto& userInfo : trigger)
    {
      Time duration = WifiPhy::CalculateTxDuration (m_sizeOf8QosNull, txVector,
                                                    m_apMac->GetWifiPhy ()->GetPhyBand (),
                                                    userInfo.GetAid12 ());
      qosNullTxDuration = Max (qosNullTxDuration, duration);
    }

  if (m_availableTime != Time::Min ())
    {
      // TryAddMpdu only considers the time to transmit the Trigger Frame
      NS_ASSERT (m_txParams.m_protection && m_txParams.m_protection->protectionTime != Time::Min ());
      NS_ASSERT (m_txParams.m_acknowledgment && m_txParams.m_acknowledgment->acknowledgmentTime.IsZero ());
      NS_ASSERT (m_txParams.m_txDuration != Time::Min ());

      if (m_txParams.m_protection->protectionTime
          + m_txParams.m_txDuration     // BSRP TF tx time
          + m_apMac->GetWifiPhy ()->GetSifs ()
          + qosNullTxDuration
          > m_availableTime)
        {
          NS_LOG_DEBUG ("Remaining TXOP duration is not enough for BSRP TF exchange");
          return NO_TX;
        }
    }

  NS_LOG_DEBUG ("Duration of QoS Null frames: " << qosNullTxDuration.As (Time::MS));
  trigger.SetUlLength (HePhy::ConvertHeTbPpduDurationToLSigLength (qosNullTxDuration,
                                                                    m_apMac->GetWifiPhy ()->GetPhyBand ()));
  trigger.SetCsRequired (true);
  m_heFem->SetTargetRssi (trigger);

  packet = Create<Packet> ();
  packet->AddHeader (trigger);
  m_trigger = Create<WifiMacQueueItem> (packet, hdr);

  m_ulTriggerType = TriggerFrameType::BSRP_TRIGGER;
  m_tbPpduDuration = qosNullTxDuration;

  return UL_MU_TX;
}

MultiUserScheduler::TxFormat
DrrMultiUserScheduler::TrySendingBasicTf (void)
{
  NS_LOG_FUNCTION (this);

  // check if an UL OFDMA transmission is possible after a DL OFDMA transmission
  NS_ABORT_MSG_IF (m_ulPsduSize == 0, "The UlPsduSize attribute must be set to a non-null value");

  // determine which of the stations served in DL have UL traffic
  uint32_t maxBufferSize = 0;
  // candidates sorted in decreasing order of queue size
  std::multimap<uint8_t, CandidateInfo, std::greater<uint8_t>> ulCandidates;

  for (const auto& candidate : m_candidates)
    {
      uint8_t queueSize = m_apMac->GetMaxBufferStatus (candidate.first->address);
      if (queueSize == 255)
        {
          NS_LOG_DEBUG ("Buffer status of station " << candidate.first->address << " is unknown");
          maxBufferSize = std::max (maxBufferSize, m_ulPsduSize);
        }
      else if (queueSize == 254)
        {
          NS_LOG_DEBUG ("Buffer status of station " << candidate.first->address << " is not limited");
          maxBufferSize = 0xffffffff;
        }
      else
        {
          NS_LOG_DEBUG ("Buffer status of station " << candidate.first->address << " is " << +queueSize);
          maxBufferSize = std::max (maxBufferSize, static_cast<uint32_t> (queueSize * 256));
        }
      // serve the station if its queue size is not null
      if (queueSize > 0)
        {
          ulCandidates.emplace (queueSize, candidate);
        }
    }

  // if the maximum buffer size is 0, skip UL OFDMA and proceed with trying DL OFDMA
  if (maxBufferSize > 0)
    {
      NS_ASSERT (!ulCandidates.empty ());
      std::size_t count = ulCandidates.size ();
      std::size_t nCentral26TonesRus;
      HeRu::RuType ruType = HeRu::GetEqualSizedRusForStations (m_apMac->GetWifiPhy ()->GetChannelWidth (),
                                                               count, nCentral26TonesRus);
      if (!m_useCentral26TonesRus || ulCandidates.size () == count)
        {
          nCentral26TonesRus = 0;
        }
      else
        {
          nCentral26TonesRus = std::min (ulCandidates.size () - count, nCentral26TonesRus);
        }

      WifiTxVector txVector;
      txVector.SetPreambleType (WIFI_PREAMBLE_HE_TB);
      auto candidateIt = ulCandidates.begin ();

      if (GetLastTxFormat () == DL_MU_TX)
        {
          txVector.SetChannelWidth (GetDlMuInfo ().txParams.m_txVector.GetChannelWidth ());
          txVector.SetGuardInterval (CtrlTriggerHeader ().GetGuardInterval ());

          for (std::size_t i = 0; i < count + nCentral26TonesRus; i++)
            {
              NS_ASSERT (candidateIt != ulCandidates.end ());
              uint16_t staId = candidateIt->second.first->aid;
              // AssignRuIndices will be called below to set RuSpec
              txVector.SetHeMuUserInfo (staId,
                                        {{(i < count ? ruType : HeRu::RU_26_TONE), 1, false},
                                        GetDlMuInfo ().txParams.m_txVector.GetMode (staId),
                                        GetDlMuInfo ().txParams.m_txVector.GetNss (staId)});

              candidateIt++;
            }
        }
      else
        {
          CtrlTriggerHeader trigger;
          GetUlMuInfo ().trigger->GetPacket ()->PeekHeader (trigger);

          txVector.SetChannelWidth (trigger.GetUlBandwidth ());
          txVector.SetGuardInterval (trigger.GetGuardInterval ());

          for (std::size_t i = 0; i < count + nCentral26TonesRus; i++)
            {
              NS_ASSERT (candidateIt != ulCandidates.end ());
              uint16_t staId = candidateIt->second.first->aid;
              auto userInfoIt = trigger.FindUserInfoWithAid (staId);
              NS_ASSERT (userInfoIt != trigger.end ());
              // AssignRuIndices will be called below to set RuSpec
              txVector.SetHeMuUserInfo (staId,
                                        {{(i < count ? ruType : HeRu::RU_26_TONE), 1, false},
                                        HePhy::GetHeMcs (userInfoIt->GetUlMcs ()),
                                        userInfoIt->GetNss ()});

              candidateIt++;
            }
        }

      // remove candidates that will not be served
      ulCandidates.erase (candidateIt, ulCandidates.end ());
      AssignRuIndices (txVector);

      CtrlTriggerHeader trigger (TriggerFrameType::BASIC_TRIGGER, txVector);
      Ptr<Packet> packet = Create<Packet> ();
      packet->AddHeader (trigger);

      Mac48Address receiver = Mac48Address::GetBroadcast ();
      if (ulCandidates.size () == 1)
        {
          receiver = ulCandidates.begin ()->second.first->address;
        }

      WifiMacHeader hdr (WIFI_MAC_CTL_TRIGGER);
      hdr.SetAddr1 (receiver);
      hdr.SetAddr2 (m_apMac->GetAddress ());
      hdr.SetDsNotTo ();
      hdr.SetDsNotFrom ();

      Ptr<WifiMacQueueItem> item = Create<WifiMacQueueItem> (packet, hdr);

      // compute the maximum amount of time that can be granted to stations.
      // This value is limited by the max PPDU duration
      Time maxDuration = GetPpduMaxTime (txVector.GetPreambleType ());

      m_txParams.Clear ();
      // set the TXVECTOR used to send the Trigger Frame
      m_txParams.m_txVector = m_apMac->GetWifiRemoteStationManager ()->GetRtsTxVector (receiver);

      if (!m_heFem->TryAddMpdu (item, m_txParams, m_availableTime))
        {
          // an UL OFDMA transmission is not possible, hence return NO_TX. In
          // this way, no transmission will occur now and the next time we will
          // try again performing an UL OFDMA transmission.
          NS_LOG_DEBUG ("Remaining TXOP duration is not enough for UL MU exchange");
          return NO_TX;
        }

      if (m_availableTime != Time::Min ())
        {
          // TryAddMpdu only considers the time to transmit the Trigger Frame
          NS_ASSERT (m_txParams.m_protection && m_txParams.m_protection->protectionTime != Time::Min ());
          NS_ASSERT (m_txParams.m_acknowledgment && m_txParams.m_acknowledgment->acknowledgmentTime != Time::Min ());
          NS_ASSERT (m_txParams.m_txDuration != Time::Min ());

          maxDuration = Min (maxDuration, m_availableTime
                                          - m_txParams.m_protection->protectionTime
                                          - m_txParams.m_txDuration
                                          - m_apMac->GetWifiPhy ()->GetSifs ()
                                          - m_txParams.m_acknowledgment->acknowledgmentTime);
          if (maxDuration.IsNegative ())
            {
              NS_LOG_DEBUG ("Remaining TXOP duration is not enough for UL MU exchange");
              return NO_TX;
            }
        }

      // Compute the time taken by each station to transmit a frame of maxBufferSize size
      Time bufferTxTime = Seconds (0);
      for (const auto& userInfo : trigger)
        {
          Time duration = WifiPhy::CalculateTxDuration (maxBufferSize, txVector,
                                                        m_apMac->GetWifiPhy ()->GetPhyBand (),
                                                        userInfo.GetAid12 ());
          bufferTxTime = Max (bufferTxTime, duration);
        }

      if (bufferTxTime < maxDuration)
        {
          // the maximum buffer size can be transmitted within the allowed time
          maxDuration = bufferTxTime;
        }
      else
        {
          // maxDuration may be a too short time. If it does not allow any station to
          // transmit at least m_ulPsduSize bytes, give up the UL MU transmission for now
          Time minDuration = Seconds (0);
          for (const auto& userInfo : trigger)
            {
              Time duration = WifiPhy::CalculateTxDuration (m_ulPsduSize, txVector,
                                                            m_apMac->GetWifiPhy ()->GetPhyBand (),
                                                            userInfo.GetAid12 ());
              minDuration = (minDuration.IsZero () ? duration : Min (minDuration, duration));
            }

          if (maxDuration < minDuration)
            {
              // maxDuration is a too short time, hence return NO_TX. In this way,
              // no transmission will occur now and the next time we will try again
              // performing an UL OFDMA transmission.
              NS_LOG_DEBUG ("Available time " << maxDuration.As (Time::MS) << " is too short");
              return NO_TX;
            }
        }

      // maxDuration is the time to grant to the stations. Finalize the Trigger Frame
      NS_LOG_DEBUG ("TB PPDU duration: " << maxDuration.As (Time::MS));
      trigger.SetUlLength (HePhy::ConvertHeTbPpduDurationToLSigLength (maxDuration,
                                                                       m_apMac->GetWifiPhy ()->GetPhyBand ()));
      trigger.SetCsRequired (true);
      m_heFem->SetTargetRssi (trigger);
      // set Preferred AC to the AC that gained channel access
      for (auto& userInfo : trigger)
        {
          userInfo.SetBasicTriggerDepUserInfo (0, 0, m_edca->GetAccessCategory ());
        }

      packet = Create<Packet> ();
      packet->AddHeader (trigger);
      m_trigger = Create<WifiMacQueueItem> (packet, hdr);

      m_ulTriggerType = TriggerFrameType::BASIC_TRIGGER;
      m_tbPpduDuration = maxDuration;

      return UL_MU_TX;
    }
  return DL_MU_TX;
}

void
DrrMultiUserScheduler::NotifyStationAssociated (uint16_t aid, Mac48Address address)
{
  NS_LOG_FUNCTION (this << aid << address);

  if (GetWifiRemoteStationManager ()->GetHeSupported (address))
    {
      for (auto& staList : m_staList)
        {
          staList.second.push_back (MasterInfo {aid, address, 0.0});
        }
    }
}

void
DrrMultiUserScheduler::NotifyStationDeassociated (uint16_t aid, Mac48Address address)
{
  NS_LOG_FUNCTION (this << aid << address);

  if (GetWifiRemoteStationManager ()->GetHeSupported (address))
    {
      for (auto& staList : m_staList)
        {
          staList.second.remove_if ([&aid, &address] (const MasterInfo& info)
                                    { return info.aid == aid && info.address == address; });
        }
    }
}

MultiUserScheduler::TxFormat
DrrMultiUserScheduler::TrySendingDlMuPpdu (void)
{
  NS_LOG_FUNCTION (this);

  AcIndex primaryAc = m_edca->GetAccessCategory ();

  if (m_staList[primaryAc].empty ())
    {
      NS_LOG_DEBUG ("No HE stations associated: return SU_TX");
      return TxFormat::SU_TX;
    }


  std::size_t count = std::min (static_cast<std::size_t> (m_nStations), m_staList[primaryAc].size ());
  std::size_t nCentral26TonesRus;
  /**
   * \todo This method is weird, if you have 3 STAs then instead of splitting the
   * the BW in 4 RUs, it splits it into 2 RUs, making the 3rd station wait for
   * an RU in the next round, in order to better compare this with out DA scheduler
   * I believe we should be performing the splits exactly as the DA scheduler does,
   * so change this.
   */ 
  HeRu::RuType ruType = HeRu::GetEqualSizedRusForStations (m_apMac->GetWifiPhy ()->GetChannelWidth (), count,
                                                           nCentral26TonesRus);
  
  // This is important, otherwise if 20 STAs are associated
  // to the AP and have packets to receive, only 18 would be 
  // added to m_candidates we do not want this, we want all
  // STAs that have packets to receive should be considered
  // candidates right now, in ComputeDlMuInfo() we will
  // assign the RUs to the first 18 stations and move
  // the remaining 2 in the pending list, checking for
  // drops (if any)
  // Also do not use havePacketsArrived condition here,
  // because that condition is set later, so for the first round
  // not having that set would lead to count being decided by
  // GetEqualSizedRusForStations, which is not correct
  if ( m_hasDeadlineConstrainedTrafficStarted )
    count = m_nStations;


  // One more thing to note is that, usually count * ruType = 484-tone, but because
  // of out modification, this equation no longer holds, which is fine because
  // the ruType here is only used to check if TxOp Limit isn't violated,
  // later in ComputeDlMuInfo() again the check is performed, so we are fine.

  NS_ASSERT (count >= 1);

  if (!m_useCentral26TonesRus)
    {
      nCentral26TonesRus = 0;
    }

  uint8_t currTid = wifiAcList.at (primaryAc).GetHighTid ();

  Ptr<const WifiMacQueueItem> mpdu = m_edca->PeekNextMpdu ();

  if (mpdu != nullptr && mpdu->GetHeader ().IsQosData ())
    {
      currTid = mpdu->GetHeader ().GetQosTid ();
    }

  // determine the list of TIDs to check
  std::vector<uint8_t> tids;

  if (m_enableTxopSharing)
    {
      for (auto acIt = wifiAcList.find (primaryAc); acIt != wifiAcList.end (); acIt++)
        {
          uint8_t firstTid = (acIt->first == primaryAc ? currTid : acIt->second.GetHighTid ());
          tids.push_back (firstTid);
          tids.push_back (acIt->second.GetOtherTid (firstTid));
        }
    }
  else
    {
      tids.push_back (currTid);
    }

  Ptr<HeConfiguration> heConfiguration = m_apMac->GetHeConfiguration ();
  NS_ASSERT (heConfiguration != 0);

  m_txParams.Clear ();
  m_txParams.m_txVector.SetPreambleType (WIFI_PREAMBLE_HE_MU);
  m_txParams.m_txVector.SetChannelWidth (m_apMac->GetWifiPhy ()->GetChannelWidth ());
  m_txParams.m_txVector.SetGuardInterval (heConfiguration->GetGuardInterval ().GetNanoSeconds ());
  m_txParams.m_txVector.SetBssColor (heConfiguration->GetBssColor ());

  // The TXOP limit can be exceeded by the TXOP holder if it does not transmit more
  // than one Data or Management frame in the TXOP and the frame is not in an A-MPDU
  // consisting of more than one MPDU (Sec. 10.22.2.8 of 802.11-2016).
  // For the moment, we are considering just one MPDU per receiver.
  Time actualAvailableTime = (m_initialFrame ? Time::Min () : m_availableTime); 

  // iterate over the associated stations until an enough number of stations is identified
  auto staIt = m_staList[primaryAc].begin ();
  m_candidates.clear ();

  while (staIt != m_staList[primaryAc].end ()
         && m_candidates.size () < std::min (static_cast<std::size_t> (m_nStations), count + nCentral26TonesRus))
  {
      NS_LOG_DEBUG ("Next candidate STA (MAC=" << staIt->address << ", AID=" << staIt->aid << ")");

      HeRu::RuType currRuType = (m_candidates.size () < count ? ruType : HeRu::RU_26_TONE);

      // check if the AP has at least one frame to be sent to the current station
      for (uint8_t tid : tids)
        {
          AcIndex ac = QosUtilsMapTidToAc (tid);
          NS_ASSERT (ac >= primaryAc);
          // check that a BA agreement is established with the receiver for the
          // considered TID, since ack sequences for DL MU PPDUs require block ack
          if (m_apMac->GetQosTxop (ac)->GetBaAgreementEstablished (staIt->address, tid))
            {
              mpdu = m_apMac->GetQosTxop (ac)->PeekNextMpdu (tid, staIt->address);

              // we only check if the first frame of the current TID meets the size
              // and duration constraints. We do not explore the queues further.
              if (mpdu != 0)
                {
                  // Use a temporary TX vector including only the STA-ID of the
                  // candidate station to check if the MPDU meets the size and time limits.
                  // An RU of the computed size is tentatively assigned to the candidate
                  // station, so that the TX duration can be correctly computed.
                  WifiTxVector suTxVector = GetWifiRemoteStationManager ()->GetDataTxVector (mpdu->GetHeader ()),
                               txVectorCopy = m_txParams.m_txVector;

                  m_txParams.m_txVector.SetHeMuUserInfo (staIt->aid,
                                                         {{currRuType, 1, false},
                                                          suTxVector.GetMode (),
                                                          suTxVector.GetNss ()});

                  if (!m_heFem->TryAddMpdu (mpdu, m_txParams, actualAvailableTime))
                    {
                      NS_LOG_DEBUG ("Adding the peeked frame violates the time constraints");
                      m_txParams.m_txVector = txVectorCopy;
                    }
                  else
                    {
                      // the frame meets the constraints
                      NS_LOG_DEBUG ("Adding candidate STA (MAC=" << staIt->address << ", AID="
                                    << staIt->aid << ") TID=" << +tid);
                      m_candidates.push_back ({staIt, mpdu});
                      break;    // terminate the for loop
                    }
                }
              else
                {
                  NS_LOG_DEBUG ("No frames to send to " << staIt->address << " with TID=" << +tid);
                }
            }
        }

      // move to the next station in the list
      staIt++;
  }

  if ( m_hasDeadlineConstrainedTrafficStarted && !m_candidates.empty() && !m_havePacketsArrived ) {
    
    m_havePacketsArrived = true; // Although the deadline constrained traffic had started arriving, we know that now the first packet has arrived
  }
 
                                                                                // Only generate packet schedules when
                                                                                // the set rounds have been finished
  if ( m_hasDeadlineConstrainedTrafficStarted && !m_candidates.empty() && ( GetCurrRound() % GetRoundsPerSchedule() == 0 ) ) {

     GeneratePacketScheduleForSetRounds(); // m_packetSchedule      
  }

  if (m_candidates.empty ())
    {
      if (m_forceDlOfdma)
        {
          NS_LOG_DEBUG ("The AP does not have suitable frames to transmit: return NO_TX");
          return NO_TX;
        }
      NS_LOG_DEBUG ("The AP does not have suitable frames to transmit: return SU_TX");
      return SU_TX;
    }

  return TxFormat::DL_MU_TX;
}

MultiUserScheduler::DlMuInfo
DrrMultiUserScheduler::ComputeDlMuInfo (void)
{
  NS_LOG_FUNCTION (this);

  if (m_candidates.empty ())
    {
      /**
       * \todo
       */
      // This made me wonder, if the packet time periods are such that
      // no packets are generated in certain rounds and the MAC queue
      // is also empty, would the simulation keep running or would it
      // stop? If the first condition is true, but second isn't, then
      // the line below is necessary to keep it going, but if both the
      // conditions are true then it seems the simulation could stop,
      // so need to see about this

      if ( m_hasDeadlineConstrainedTrafficStarted && m_havePacketsArrived )
        StartNextRound();

      return DlMuInfo ();
    }  

  uint16_t bw = m_apMac->GetWifiPhy ()->GetChannelWidth ();

  // compute how many stations can be granted an RU and the RU size
  std::size_t nRusAssigned = m_txParams.GetPsduInfoMap ().size ();
  std::size_t nCentral26TonesRus;
  HeRu::RuType ruType = HeRu::GetEqualSizedRusForStations (bw, nRusAssigned, nCentral26TonesRus);

  NS_LOG_DEBUG (nRusAssigned << " stations are being assigned a " << ruType << " RU");

  if (!m_useCentral26TonesRus || m_candidates.size () == nRusAssigned)
    {
      nCentral26TonesRus = 0;
    }
  else
    {
      nCentral26TonesRus = std::min (m_candidates.size () - nRusAssigned, nCentral26TonesRus);
      NS_LOG_DEBUG (nCentral26TonesRus << " stations are being assigned a 26-tones RU");
    }

  DlMuInfo dlMuInfo;

  // We have to update the TXVECTOR
  dlMuInfo.txParams.m_txVector.SetPreambleType (m_txParams.m_txVector.GetPreambleType ());
  dlMuInfo.txParams.m_txVector.SetChannelWidth (m_txParams.m_txVector.GetChannelWidth ());
  dlMuInfo.txParams.m_txVector.SetGuardInterval (m_txParams.m_txVector.GetGuardInterval ());
  dlMuInfo.txParams.m_txVector.SetBssColor (m_txParams.m_txVector.GetBssColor ());

  auto candidateIt = m_candidates.begin (); // iterator over the list of candidate receivers

  for (uint16_t i = 0; i < nRusAssigned + nCentral26TonesRus; i++)
    {
      NS_ASSERT (candidateIt != m_candidates.end ());

      uint16_t staId = candidateIt->first->aid;
      // AssignRuIndices will be called below to set RuSpec
      dlMuInfo.txParams.m_txVector.SetHeMuUserInfo (staId,
                                                    {{(i < nRusAssigned ? ruType : HeRu::RU_26_TONE), 1, false},
                                                      m_txParams.m_txVector.GetMode (staId),
                                                      m_txParams.m_txVector.GetNss (staId)});

      candidateIt++;
    }

  // Remove candidates that will not be served in this round
  // And move them to pending list to check which of them
  // have missed their deadline in this round
  if ( m_hasDeadlineConstrainedTrafficStarted && m_havePacketsArrived ) {

    m_pendingCandidates.clear();
    
    auto tempIt = candidateIt;
    while ( tempIt != m_candidates.end() ) {
      m_pendingCandidates.push_back({tempIt->first, tempIt->second});
      std::cout << "STA_" << tempIt->first->aid << " is a pending candidate in round " << GetCurrRound() << std::endl;
      tempIt++;
    }

    // The Deadline Round Robin scheduler drops packets whose deadlines
    // expired in this round
    for ( const auto& candidate : m_pendingCandidates ) {

      uint16_t aid = candidate.first->aid;
      aid--;

      bool dropFlag = false;
      for ( std::vector<uint32_t> x : m_packetSchedule ) {
        if ( x[3] == aid && x[1] == GetCurrRound() ) { // A packet belonging to this station had this round as it's deadline, drop

          Ptr<const WifiMacQueueItem> mpdu = candidate.second;
          uint8_t tid = mpdu->GetHeader().GetQosTid();
                  
          Ptr<WifiMacQueue> queue = m_apMac->GetQosTxop(QosUtilsMapTidToAc (tid))->GetWifiMacQueue();
                  
          WifiMacQueueItem::QueueIteratorPair queueIt = mpdu->GetQueueIteratorPairs ().front ();
          NS_ASSERT (queueIt.queue != nullptr);
          queue->Dequeue(queueIt.it);

          std::cout << "Dropped STA_" << (aid + 1) << " packet in round " << GetCurrRound() << std::endl;
          dropFlag = true;
          break;
        }
      }

      if ( !dropFlag )
        std::cout << "Buffered STA_" << (aid + 1) << " packet in round " << GetCurrRound() << std::endl;
    }
  }

  m_candidates.erase (candidateIt, m_candidates.end ());

  AssignRuIndices (dlMuInfo.txParams.m_txVector);

  m_txParams.Clear ();

  Ptr<const WifiMacQueueItem> mpdu;

  // Compute the TX params (again) by using the stored MPDUs and the final TXVECTOR
  Time actualAvailableTime = (m_initialFrame ? Time::Min () : m_availableTime);

  for (const auto& candidate : m_candidates)
    {
      mpdu = candidate.second;
      NS_ASSERT (mpdu != nullptr);

      bool ret = m_heFem->TryAddMpdu (mpdu, dlMuInfo.txParams, actualAvailableTime);
      NS_UNUSED (ret);
      NS_ASSERT_MSG (ret, "Weird that an MPDU does not meet constraints when "
                          "transmitted over a larger RU");
    }

  // We have to complete the PSDUs to send
  Ptr<WifiMacQueue> queue;
  Mac48Address receiver;

  for (const auto& candidate : m_candidates)
    {
      // Let us try first A-MSDU aggregation if possible
      mpdu = candidate.second;
      NS_ASSERT (mpdu != nullptr);
      uint8_t tid = mpdu->GetHeader ().GetQosTid ();
      receiver = mpdu->GetHeader ().GetAddr1 ();
      NS_ASSERT (receiver == candidate.first->address);

      NS_ASSERT (mpdu->IsQueued ());
      
      if ( mpdu->GetHeader().IsData() && m_hasDeadlineConstrainedTrafficStarted && m_havePacketsArrived ) {
        std::cout << "STA_" << candidate.first->aid << " is transmitting DATA in round " << GetCurrRound() <<"\n";         
      }

      WifiMacQueueItem::QueueIteratorPair queueIt = mpdu->GetQueueIteratorPairs ().front ();
      NS_ASSERT (queueIt.queue != nullptr);
      Ptr<WifiMacQueueItem> item = *queueIt.it;
      queueIt.it++;

      if (!mpdu->GetHeader ().IsRetry ())
        {
          // this MPDU must have been dequeued from the AC queue and we can try
          // A-MSDU aggregation
          item = m_heFem->GetMsduAggregator ()->GetNextAmsdu (mpdu, dlMuInfo.txParams, m_availableTime, queueIt);

          if (item == nullptr)
            {
              // A-MSDU aggregation failed or disabled
              item = *mpdu->GetQueueIteratorPairs ().front ().it;
            }
          m_apMac->GetQosTxop (QosUtilsMapTidToAc (tid))->AssignSequenceNumber (item);
        }

      // Now, let's try A-MPDU aggregation if possible
      std::vector<Ptr<WifiMacQueueItem>> mpduList = m_heFem->GetMpduAggregator ()->GetNextAmpdu (item, dlMuInfo.txParams, m_availableTime, queueIt);

      if (mpduList.size () > 1)
        {
          // A-MPDU aggregation succeeded, update psduMap
          dlMuInfo.psduMap[candidate.first->aid] = Create<WifiPsdu> (std::move (mpduList));
        }
      else
        {

          dlMuInfo.psduMap[candidate.first->aid] = Create<WifiPsdu> (item, true);
        }
    }

  /**
    * \todo This credit/debit system is bogus, instead of running a true round robin cycle,
    * it keeps running errand cycles, rewrite this code
    */

  AcIndex primaryAc = m_edca->GetAccessCategory ();

  // The amount of credits received by each station equals the TX duration (in
  // microseconds) divided by the number of stations.
  double creditsPerSta = dlMuInfo.txParams.m_txDuration.ToDouble (Time::US)
                        / m_staList[primaryAc].size ();
  // Transmitting stations have to pay a number of credits equal to the TX duration
  // (in microseconds) times the allocated bandwidth share.
  double debitsPerMhz = dlMuInfo.txParams.m_txDuration.ToDouble (Time::US)
                        / (nRusAssigned * HeRu::GetBandwidth (ruType)
                           + nCentral26TonesRus * HeRu::GetBandwidth (HeRu::RU_26_TONE));

  // assign credits to all stations
  for (auto& sta : m_staList[primaryAc])
    {
      sta.credits += creditsPerSta;
      sta.credits = std::min (sta.credits, m_maxCredits.ToDouble (Time::US));
    }

  // subtract debits to the selected stations
  candidateIt = m_candidates.begin ();

  for (std::size_t i = 0; i < nRusAssigned + nCentral26TonesRus; i++)
    {
      NS_ASSERT (candidateIt != m_candidates.end ());

      candidateIt->first->credits -= debitsPerMhz * HeRu::GetBandwidth (i < nRusAssigned ? ruType : HeRu::RU_26_TONE);

      candidateIt++;
    }

  // sort the list in decreasing order of credits
  m_staList[primaryAc].sort ([] (const MasterInfo& a, const MasterInfo& b)
                                { return a.credits > b.credits; });

  NS_LOG_DEBUG ("Next station to serve has AID=" << m_staList[primaryAc].front ().aid);
  
  // This is for the scenario where after the Tx is completed, some packets are still
  // in the queue waiting for scheduling in the next round, so we should now go to the next
  // round to allow their scheduling since all the expected user packets have arrived in
  // this round
  // Not just that, you see TrySendingDlMuPpdu() is not called unless there is atleast one 
  // packet in the MAC queue, so if after this Tx, the MAC queue us emptied, the simulation
  // will stop. In order to allow it to keep going, we must generate the next batch of packets
  // before this round ends, otherwise the next round will never begin!

  if ( m_hasDeadlineConstrainedTrafficStarted && m_havePacketsArrived )
     StartNextRound();

  return dlMuInfo;
}

void
DrrMultiUserScheduler::AssignRuIndices (WifiTxVector& txVector)
{
  NS_LOG_FUNCTION (this << txVector);

  uint8_t bw = txVector.GetChannelWidth ();

  // find the RU types allocated in the TXVECTOR
  std::set<HeRu::RuType> ruTypeSet;
  for (const auto& userInfo : txVector.GetHeMuUserInfoMap ())
    {
      ruTypeSet.insert (userInfo.second.ru.GetRuType ());
    }

  std::vector<HeRu::RuSpec> ruSet, central26TonesRus;

  // This scheduler allocates equal sized RUs and optionally the remaining 26-tone RUs
  if (ruTypeSet.size () == 2)
    {
      // central 26-tone RUs have been allocated
      NS_ASSERT (ruTypeSet.find (HeRu::RU_26_TONE) != ruTypeSet.end ());
      ruTypeSet.erase (HeRu::RU_26_TONE);
      NS_ASSERT (ruTypeSet.size () == 1);
      central26TonesRus = HeRu::GetCentral26TonesRus (bw, *ruTypeSet.begin ());
    }

  NS_ASSERT (ruTypeSet.size () == 1);
  ruSet = HeRu::GetRusOfType (bw, *ruTypeSet.begin ());

  auto ruSetIt = ruSet.begin ();
  auto central26TonesRusIt = central26TonesRus.begin ();

  for (const auto& userInfo : txVector.GetHeMuUserInfoMap ())
    {
      if (userInfo.second.ru.GetRuType () == *ruTypeSet.begin ())
        {
          NS_ASSERT (ruSetIt != ruSet.end ());
          txVector.SetRu (*ruSetIt, userInfo.first);
          ruSetIt++;
        }
      else
        {
          NS_ASSERT (central26TonesRusIt != central26TonesRus.end ());
          txVector.SetRu (*central26TonesRusIt, userInfo.first);
          central26TonesRusIt++;
        }
    }
}

MultiUserScheduler::UlMuInfo
DrrMultiUserScheduler::ComputeUlMuInfo (void)
{
  return UlMuInfo {m_trigger, m_tbPpduDuration, std::move (m_txParams)};
}

} //namespace ns3
