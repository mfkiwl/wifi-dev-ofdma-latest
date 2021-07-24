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
#include "rr-multi-user-scheduler.h"
#include "ns3/wifi-protection.h"
#include "ns3/wifi-acknowledgment.h"
#include "ns3/wifi-psdu.h"
#include "he-frame-exchange-manager.h"
#include "he-configuration.h"
#include "he-phy.h"
#include <algorithm>
#include <fstream>
#include <utility>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("RrMultiUserScheduler");

NS_OBJECT_ENSURE_REGISTERED (RrMultiUserScheduler);

TypeId
RrMultiUserScheduler::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::RrMultiUserScheduler")
    .SetParent<MultiUserScheduler> ()
    .SetGroupName ("Wifi")
    .AddConstructor<RrMultiUserScheduler> ()
    .AddAttribute ("NStations",
                   "The maximum number of stations that can be granted an RU in a DL MU OFDMA transmission",
                   UintegerValue (4),
                   MakeUintegerAccessor (&RrMultiUserScheduler::m_nStations),
                   MakeUintegerChecker<uint8_t> (1, 74))
    .AddAttribute ("EnableTxopSharing",
                   "If enabled, allow A-MPDUs of different TIDs in a DL MU PPDU.",
                   BooleanValue (true),
                   MakeBooleanAccessor (&RrMultiUserScheduler::m_enableTxopSharing),
                   MakeBooleanChecker ())
    .AddAttribute ("ForceDlOfdma",
                   "If enabled, return DL_MU_TX even if no DL MU PPDU could be built.",
                   BooleanValue (false),
                   MakeBooleanAccessor (&RrMultiUserScheduler::m_forceDlOfdma),
                   MakeBooleanChecker ())
    .AddAttribute ("EnableUlOfdma",
                   "If enabled, return UL_MU_TX if DL_MU_TX was returned the previous time.",
                   BooleanValue (true),
                   MakeBooleanAccessor (&RrMultiUserScheduler::m_enableUlOfdma),
                   MakeBooleanChecker ())
    .AddAttribute ("EnableBsrp",
                   "If enabled, send a BSRP Trigger Frame before an UL MU transmission.",
                   BooleanValue (true),
                   MakeBooleanAccessor (&RrMultiUserScheduler::m_enableBsrp),
                   MakeBooleanChecker ())
    .AddAttribute ("UlPsduSize",
                   "The default size in bytes of the solicited PSDU (to be sent in a TB PPDU)",
                   UintegerValue (500),
                   MakeUintegerAccessor (&RrMultiUserScheduler::m_ulPsduSize),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("UseCentral26TonesRus",
                   "If enabled, central 26-tone RUs are allocated, too, when the "
                   "selected RU type is at least 52 tones.",
                   BooleanValue (false),
                   MakeBooleanAccessor (&RrMultiUserScheduler::m_useCentral26TonesRus),
                   MakeBooleanChecker ())
    .AddAttribute ("MaxCredits",
                   "Maximum amount of credits a station can have. When transmitting a DL MU PPDU, "
                   "the amount of credits received by each station equals the TX duration (in "
                   "microseconds) divided by the total number of stations. Stations that are the "
                   "recipient of the DL MU PPDU have to pay a number of credits equal to the TX "
                   "duration (in microseconds) times the allocated bandwidth share",
                   TimeValue (Seconds (1)),
                   MakeTimeAccessor (&RrMultiUserScheduler::m_maxCredits),
                   MakeTimeChecker ())
  ;
  return tid;
}

RrMultiUserScheduler::RrMultiUserScheduler ()
  : loopOutput(false), m_startStation(0), m_ulTriggerType (TriggerFrameType::BASIC_TRIGGER) 
{
  NS_LOG_FUNCTION (this);
}

RrMultiUserScheduler::~RrMultiUserScheduler ()
{
  NS_LOG_FUNCTION_NOARGS ();
}

void
RrMultiUserScheduler::DoInitialize (void)
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT (m_apMac != nullptr);
  m_apMac->TraceConnectWithoutContext ("AssociatedSta",
                                       MakeCallback (&RrMultiUserScheduler::NotifyStationAssociated, this));
  m_apMac->TraceConnectWithoutContext ("DeAssociatedSta",
                                       MakeCallback (&RrMultiUserScheduler::NotifyStationDeassociated, this));
  for (const auto& ac : wifiAcList)
    {
      m_staList.insert ({ac.first, {}});
    }
  MultiUserScheduler::DoInitialize ();
}

void
RrMultiUserScheduler::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
  m_staList.clear ();
  m_candidates.clear ();
  m_simpleCandidates.clear();
  m_trigger = nullptr;
  m_txParams.Clear ();
  m_apMac->TraceDisconnectWithoutContext ("AssociatedSta",
                                          MakeCallback (&RrMultiUserScheduler::NotifyStationAssociated, this));
  m_apMac->TraceDisconnectWithoutContext ("DeAssociatedSta",
                                          MakeCallback (&RrMultiUserScheduler::NotifyStationDeassociated, this));
  MultiUserScheduler::DoDispose ();
}

MultiUserScheduler::TxFormat
RrMultiUserScheduler::SelectTxFormat (void)
{
  NS_LOG_FUNCTION (this);

  // After every DL, try performing an UL transmission
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
RrMultiUserScheduler::TrySendingBsrpTf (void)
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
RrMultiUserScheduler::TrySendingBasicTf (void)
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
RrMultiUserScheduler::NotifyStationAssociated (uint16_t aid, Mac48Address address)
{
  NS_LOG_FUNCTION (this << aid << address);

  // Whenever an HE station is associated, add it to the m_staList in each AC
  if (GetWifiRemoteStationManager ()->GetHeSupported (address))
    {
      for (auto& staList : m_staList)
        {
          staList.second.push_back (MasterInfo {aid, address, 0.0});
        }
    }
}

void
RrMultiUserScheduler::NotifyStationDeassociated (uint16_t aid, Mac48Address address)
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

// Returns a constant specifying the DL MU TX Format,
// The main purpose of calling this is to prepare a list of
// candidate stations for DL transmission, that is the
// m_candidates

MultiUserScheduler::TxFormat
RrMultiUserScheduler::TrySendingDlMuPpdu (void)
{
  NS_LOG_FUNCTION (this);

//   const std::map<uint16_t, Mac48Address>& staList = m_apMac->GetStaList ();
//   auto startIt = staList.find (m_startStation);

//   // iterate stations from where we last left off

//   // This may be the first invocation or the starting station left
//   if (startIt == staList.end ())
//     {
//       startIt = staList.begin ();
//       m_startStation = startIt->first;
// std::cout<<"SelectTxFormat: if (startIt == staList.end ()),  start station"<<m_startStation<<"\n";

//     }

  // The AC queue in the AP that gained access to the channel
  AcIndex primaryAc = m_edca->GetAccessCategory ();
  
  // Do we have any associated stations corresponding to this AC?
  if (m_staList[primaryAc].empty ())
    {
      NS_LOG_DEBUG ("No HE stations associated: return SU_TX");
      return TxFormat::SU_TX;
    }

  // min(Total Stations, Total Associated Stations with this AC)
  std::size_t count = std::min (static_cast<std::size_t> (m_nStations), m_staList[primaryAc].size ());
  //std::size_t count = static_cast<std::size_t> (m_nStations);
  std::size_t nCentral26TonesRus;
  HeRu::RuType ruType = HeRu::GetEqualSizedRusForStations (m_apMac->GetWifiPhy ()->GetChannelWidth (), count,
                                                           nCentral26TonesRus);
  NS_ASSERT (count >= 1);

  if (!m_useCentral26TonesRus)
    {
      nCentral26TonesRus = 0;
    }

  // Traffic ID corresponding to the AC
  uint8_t currTid = wifiAcList.at (primaryAc).GetHighTid ();

  // Get an MPDU from this AC queue
  Ptr<const WifiMacQueueItem> mpdu = m_edca->PeekNextMpdu ();

  if (mpdu != nullptr && mpdu->GetHeader ().IsQosData ())
    {
      currTid = mpdu->GetHeader ().GetQosTid ();
    }

  // determine the list of TIDs to check
  std::vector<uint8_t> tids;

  //if (m_enableTxopSharing)
  if ( false )
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


  // do
  //   {
  //     NS_LOG_DEBUG ("Next candidate STA (MAC=" << startIt->second << ", AID=" << startIt->first << ")");
  //     //std::cout<<"Next candidate STA (MAC=" << startIt->second << ", AID=" << startIt->first << ")";
  //     // check if the AP has at least one frame to be sent to the current station
  //     // Iterate over possible traffic category identifiers from low to high priority
  //     // and check if the current station has a frame belonging to the AC corresponding to
  //     // the Tid
  //     for (uint8_t tid : std::initializer_list<uint8_t> {currTid, 1, 2, 0, 3, 4, 5, 6, 7})
  //       {
  //         AcIndex ac = QosUtilsMapTidToAc (tid);
  //         // check that a BA agreement is established with the receiver for the
  //         // considered TID, since ack sequences for DL MU PPDUs require block ack
  //         if (ac >= primaryAc && m_apMac->GetQosTxop (ac)->GetBaAgreementEstablished (startIt->second, tid))
  //           {
  //             // Peek the next frame in the QoS queue corresponding to the AC of the current packet
  //             // We actually peek the frame corresponding to the MAC address of the current station 
  //             mpdu = m_apMac->GetQosTxop (ac)->PeekNextMpdu (tid, startIt->second);

  //             // we only check if the first frame of the current TID meets the size
  //             // and duration constraints. We do not explore the queues further.
  //             if (mpdu != 0)
  //               {
  //                 //std::cout<<"if mpdu!=0 \n";
  //                 // Use a temporary TX vector including only the STA-ID of the
  //                 // candidate station to check if the MPDU meets the size and time limits.
  //                 // An RU of the computed size is tentatively assigned to the candidate
  //                 // station, so that the TX duration can be correctly computed.
  //                 WifiTxVector suTxVector = GetWifiRemoteStationManager ()->GetDataTxVector (mpdu->GetHeader ()),
  //                              txVectorCopy = m_txParams.m_txVector;

  //                 m_txParams.m_txVector.SetHeMuUserInfo (startIt->first,
  //                                                        {{ruType, 1, false},
  //                                                         suTxVector.GetMode (),
  //                                                         suTxVector.GetNss ()});

  //                 if (!m_heFem->TryAddMpdu (mpdu, m_txParams, actualAvailableTime))
  //                   {
  //                     NS_LOG_DEBUG ("Adding the peeked frame violates the time constraints");
  //                     m_txParams.m_txVector = txVectorCopy;
  //                   }
  //                 else
  //                   {
  //                     // the frame meets the constraints
  //                     NS_LOG_DEBUG ("Adding candidate STA (MAC=" << startIt->second << ", AID="
  //                                   << startIt->first << ") TID=" << +tid);
  //                                             // AID, MACAddress, Credits, MPDU (TID can be peeked)         
  //                     //m_candidates.push_back ({{startIt->first, startIt->second, 0}, mpdu});
  //                     m_simpleCandidates.push_back({startIt->first, startIt->second, mpdu});
  //                     DlPerStaInfo info {startIt->first, tid};
  //                     //                                  // MAC Address, AID, TID
  //                     m_staInfo.push_back (std::make_pair (startIt->second, info));

  //                     // Packet Size | AID
  //                     dataStaPair.push_back(std::make_pair (mpdu->GetPacket()->GetSize(), startIt->first));
  //                     break;    // terminate the for loop
  //                   }
  //               }
  //             else
  //               {
  //                 NS_LOG_DEBUG ("No frames to send to " << startIt->second << " with TID=" << +tid);
  //               }
  //           }
  //       }

  //     // move to the next station in the map
  //     startIt++;
  //     if (startIt == staList.end ())
  //       {
  //         startIt = staList.begin ();
  //       }
  //   } while (m_staInfo.size () < m_nStations && startIt->first != m_startStation);

  // iterate over the associated stations until an enough number of stations is identified
  auto staIt = m_staList[primaryAc].begin ();
  m_staInfo.clear ();
  m_candidates.clear (); // Stores the list of candidate stations for transmission, less than equal to the number of RUs available
  dataStaPair.clear();

  while (staIt != m_staList[primaryAc].end ()
         && m_candidates.size () < std::min (static_cast<std::size_t> (m_nStations), count + nCentral26TonesRus))
    {
      NS_LOG_DEBUG ("Next candidate STA (MAC=" << staIt->address << ", AID=" << staIt->aid << ")");

      //HeRu::RuType currRuType = (m_candidates.size () < count ? ruType : HeRu::RU_26_TONE);

      // check if the AP has at least one frame to be sent to the current station
      for (uint8_t tid : tids) 
        {
          AcIndex ac = QosUtilsMapTidToAc (tid);
          NS_ASSERT (ac >= primaryAc); // This station's traffic category is higher or equal priority to the AC that gained channel access, so we will be transmitting to it
          // check that a BA agreement is established with the receiver for the
          // considered TID, since ack sequences for DL MU PPDUs require block ack
          if (m_apMac->GetQosTxop (ac)->GetBaAgreementEstablished (staIt->address, tid))
            {
              // Get the MPDU in the AC category of this station (This could be different from the AC that gained channel access)
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
                                                         {{ruType, 1, false},
                                                          suTxVector.GetMode (),
                                                          suTxVector.GetNss ()});

                 if (!m_heFem->TryAddMpdu (mpdu, m_txParams, actualAvailableTime))
                 //if ( false )
                    {
                      NS_LOG_DEBUG ("Adding the peeked frame violates the time constraints");
                      std::cout << "Adding STA " << staIt->aid << " peeked frame violates the time constraints\n";

                      m_txParams.m_txVector = txVectorCopy;
                    }
                  else
                    {
                      // the frame meets the constraints
                      NS_LOG_DEBUG ("Adding candidate STA (MAC=" << staIt->address << ", AID="
                                    << staIt->aid << ") TID=" << +tid);
                                              // AID, MACAddress, Credits, MPDU (TID can be peeked)         
                      m_candidates.push_back ({staIt, mpdu});
                      DlPerStaInfo info {staIt->aid, tid};
                                                        // MAC Address, AID, TID
                      m_staInfo.push_back (std::make_pair (staIt->address, info));

                      // Packet Size | AID
                      dataStaPair.push_back(std::make_pair (mpdu->GetPacket()->GetSize(), staIt->aid));
                      break;    // terminate the for loop
                    }
                }
              else
                {
                  NS_LOG_DEBUG ("No frames to send to " << staIt->address << " with TID=" << +tid);
                  std::cout << "No frames to send to STA " << staIt->aid << " with TID=" << tid << "\n";
                }
            }
            else {
              std::cout << "BA Agreement NOT established with STA " << staIt->aid << "\n";
            }
        }

      // move to the next station in the list
      staIt++;
    }

  if (m_candidates.empty ())
  //if(m_staInfo.empty())
    {
      if (m_forceDlOfdma)
        {
          NS_LOG_DEBUG ("The AP does not have suitable frames to transmit: return NO_TX");
          return NO_TX;
        }
      NS_LOG_DEBUG ("The AP does not have suitable frames to transmit: return SU_TX");
      return SU_TX;
    }

  //Naman////////////////////////////////////
  // This list stores the Packet Size | AID map
  // So it stores what the different sized packets are to
  // be sent to the AID
  std::cout<<"dataStaPair size :::::::::::::::::::::::::::::::::::::::::::;"<<dataStaPair.size()<<"\n";
  auto a = dataStaPair.begin();
  
  while( a != dataStaPair.end()) {
    std::cout<<"first= "<<a->first<<" second= "<<a->second<<"\n";
    a++;
  }
  /////////////////////////////////////////

  // added
  //m_startStation = startIt->first;

  return TxFormat::DL_MU_TX;
}

static double tone26(int modul) {
  switch(modul) {
    case 1: return 0.8;     //BPSK 1/2
    case 2: return 1.7;     //QPSK 1/2
    case 3: return 2.5;     //QPSK 3/4
    case 4: return 3.3;     //16-QAM 1/2
    case 5: return 5.0;     //16-QAM 3/4    
    case 6: return 6.7;     //64-QAM 2/3
    case 7: return 7.5;     //64-QAM 3/4
    case 8: return 8.3;     //64-QAM 5/6
    case 9: return 10.0;    //256-QAM 3/4
    case 10: return 11.1;   //256-QAM 5/6
    case 11: return 12.5;     //^^
    case 12: return 13.9;   //^^  
    default: return 0.0;
  }
}
static double tone52(int modul) {
  switch(modul) {
    case 1: return 1.7;     //BPSK 1/2
    case 2: return 3.3;     //QPSK 1/2
    case 3: return 5.0;     //QPSK 3/4
    case 4: return 6.7;     //16-QAM 1/2
    case 5: return 10.0;    //16-QAM 3/4    
    case 6: return 13.3;    //64-QAM 2/3
    case 7: return 15.0;    //64-QAM 3/4
    case 8: return 16.7;    //64-QAM 5/6
    case 9: return 20.0;    //256-QAM 3/4
    case 10: return 22.2;   //256-QAM 5/6
    case 11: return 25.0;     //^^
    case 12: return 27.8;   //^^    
    default: return 0.0;
  }
}
static double tone106(int modul) {
  switch(modul) {
    case 1: return 3.5;     //BPSK 1/2
    case 2: return 7.1;     //QPSK 1/2
    case 3: return 10.6;    //QPSK 3/4
    case 4: return 14.2;    //16-QAM 1/2
    case 5: return 21.3;    //16-QAM 3/4    
    case 6: return 28.3;    //64-QAM 2/3
    case 7: return 31.9;    //64-QAM 3/4
    case 8: return 35.4;    //64-QAM 5/6
    case 9: return 42.5;    //256-QAM 3/4
    case 10: return 47.2;   //256-QAM 5/6
    case 11: return 53.1;     //^^
    case 12: return 59.0;   //^^  
    default: return 0.0;
  }
}
static double tone242(int modul) {
  switch(modul) {
    case 1: return 8.1;     //BPSK 1/2
    case 2: return 16.3;    //QPSK 1/2
    case 3: return 24.4;    //QPSK 3/4
    case 4: return 32.5;    //16-QAM 1/2
    case 5: return 48.8;    //16-QAM 3/4    
    case 6: return 65.0;    //64-QAM 2/3
    case 7: return 73.1;    //64-QAM 3/4
    case 8: return 81.3;    //64-QAM 5/6
    case 9: return 97.5;    //256-QAM 3/4
    case 10: return 108.3;    //256-QAM 5/6
    case 11: return 121.9;    //1024-QAM 3/4
    case 12: return 135.4;    //1024-QAM 5/6  
    default: return 0.0;
  }
}
static double tone484(int modul) {
  switch(modul) {
    case 1: return 16.3;    //BPSK 1/2
    case 2: return 32.5;    //QPSK 1/2
    case 3: return 48.8;    //QPSK 3/4
    case 4: return 65.0;    //16-QAM 1/2
    case 5: return 97.5;    //16-QAM 3/4    
    case 6: return 130.0;   //64-QAM 2/3
    case 7: return 146.3;   //64-QAM 3/4
    case 8: return 162.5;   //64-QAM 5/6
    case 9: return 195.0;   //256-QAM 3/4
    case 10: return 216.7;    //256-QAM 5/6 
    case 11: return 243.8;    //1024-QAM 3/4
    case 12: return 270.8;    //1024-QAM 5/6
    default: return 0.0;
  }
}
static double tone996(int modul) {
  switch(modul) {
    case 1: return 34.0;    //BPSK 1/2
    case 2: return 68.1;    //QPSK 1/2
    case 3: return 102.1;   //QPSK 3/4
    case 4: return 136.1;   //16-QAM 1/2
    case 5: return 204.2;   //16-QAM 3/4    
    case 6: return 272.2;   //64-QAM 2/3
    case 7: return 306.3;   //64-QAM 3/4
    case 8: return 340.3;   //64-QAM 5/6
    case 9: return 408.3;   //256-QAM 3/4
    case 10: return 453.7;    //256-QAM 5/6 
    case 11: return 510.4;    //1024-QAM 3/4
    case 12: return 567.1;    //1024-QAM 5/6
    default: return 0.0;
  }
}


double RrMultiUserScheduler::getDataRate(int mcs, int ru) {
   
  switch(ru) {
     
    case 26: return tone26(mcs);
    case 52: return tone52(mcs);
    case 106: return tone106(mcs);
    case 242: return tone242(mcs);
    case 484: return tone484(mcs);
    case 996: return tone996(mcs);
    
    default: return 0.0;
 }
 
  return 0.0;
}

void RrMultiUserScheduler::ProportionalFair(std::vector<int> currRUset,int currMCS){
  
  int row=dataStaPair.size(); 
  int col=currRUset.size();

  if ( loopOutput ) {
    std::cout<<"\n\n\nrow="<<row;
    std::cout<<"\ncol="<<col;
  }
  //std::cout<<"\ncurrMCS="<<currMCS;

  costMatrix.clear();
  costMatrix.resize(row,std::vector<double> (col,0.0));
  
  AcIndex primaryAc = m_edca->GetAccessCategory();
  auto startItw = m_staList[primaryAc].begin ();
  //const std::map<uint16_t, Mac48Address>& staList = m_apMac->GetStaList ();
  //auto startItw = staList.find (m_startStation);

  for (int i=0;i<row;i++){
    if ( loopOutput ) {
      std::cout<<"\n";  
    }
    for (int j=0;j<col;j++){
      //if(dataTransmitted[startItw->first]==0.0 && totalTime[startItw->first]==0.0){
      if(dataTransmitted[startItw->aid]==0.0 && totalTime[startItw->aid]==0.0){
        costMatrix[i][j]=-MAX_COST;
      }else{
        double rate=getDataRate(randomMCS[i],currRUset[j]);
	//double avgThroughput=dataTransmitted[startItw->first]/totalTime[startItw->first];
  double avgThroughput=dataTransmitted[startItw->aid]/totalTime[startItw->aid];
	//if(totalTime[startItw->first]==0){avgThroughput=0;}
  if(totalTime[startItw->aid]==0){avgThroughput=0;}
  	if(avgThroughput==0){avgThroughput=1;}
    if ( loopOutput ) {
      //std::cout<<"\n rate="<<rate<<" data tx="<<dataTransmitted[startItw->first]<<" time="<<totalTime[startItw->first]<<" avgthr="<<avgThroughput;
      std::cout<<"\n rate="<<rate<<" data tx="<<dataTransmitted[startItw->aid]<<" time="<<totalTime[startItw->aid]<<" avgthr="<<avgThroughput;
    }
        costMatrix[i][j]=-(rate/avgThroughput);
      }
      if ( loopOutput ) {  
        std::cout<<" cost="<<costMatrix[i][j]<<"\t";
      }
    }
    startItw++;
  }

  if ( loopOutput ) {
    std::cout<<"\n HungarianAlgo";
  }
  HungarianAlgorithm HungAlgo;
  assignment.clear();

  double cost = HungAlgo.Solve(costMatrix, assignment);
  if ( loopOutput ) {
    std::cout<<"\n Cost:"<<cost<<"\n";

    std::cout<<"assignment size="<<assignment.size();
    std::cout<<"\ncost matrix size="<<costMatrix.size()<<"\n";
  

    for (unsigned int x = 0; x < assignment.size(); x++){
      std::cout << "STA="<<x<< "," <<"RU="<< assignment[x] << "\t";
    }
    std::cout<<"\noutside\n";
  }
}

double RrMultiUserScheduler::timeReq(int dataSize,int rusize,int mcs_QAM)
{
    if(dataSize==0)
        return 0;

NS_LOG_FUNCTION("dataSize=================================================================="<<dataSize);

    double symbols=0.0;  
    int bits=dataSize*8;
    double encodeingRate=5.0/6.0;
    double bitsPerSec=log2f(mcs_QAM)*encodeingRate*(rusize);
NS_LOG_FUNCTION("ruSize============================================="<<rusize);
NS_LOG_FUNCTION("bits==============================================="<<bits);
NS_LOG_FUNCTION("bitsPerSec========================================="<<bitsPerSec);

    symbols=bits/bitsPerSec;
NS_LOG_FUNCTION("symbol============================================="<<symbols);

    return symbols*0.0000136; //guard interval + symbol duration = 0.0000136sec
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////
double RrMultiUserScheduler::timeReq1(int dataSize,int rusize,int mcs)
{
    if(dataSize==0)
        return 0;

NS_LOG_FUNCTION("dataSize=================================================================="<<dataSize);



    double symbols=0.0;  
    int bits=dataSize*8;
 
   double encodeingRate=5.0/6.0;
int mcs_QAM=2;
switch(mcs){

case 0: 
encodeingRate=1.0/2.0;
mcs_QAM=2;
break;

case 1: 
encodeingRate=1.0/2.0;
mcs_QAM=4;
break;

case 2: 
encodeingRate=3.0/4.0;
mcs_QAM=4;
break;

case 3: 
encodeingRate=1.0/2.0;
mcs_QAM=16;
break;

case 4: 
encodeingRate=3.0/4.0;
mcs_QAM=16;
break;

case 5: 
encodeingRate=2.0/3.0;
mcs_QAM=64;
break;

case 6: 
encodeingRate=3.0/4.0;
mcs_QAM=64;
break;

case 7: 
encodeingRate=5.0/6.0;
mcs_QAM=64;
break;

case 8: 
encodeingRate=3.0/4.0;
mcs_QAM=256;
break;

case 9: 
encodeingRate=5.0/6.0;
mcs_QAM=256;
break;

case 10: 
encodeingRate=3.0/4.0;
mcs_QAM=1024;
break;

case 11: 
encodeingRate=5.0/6.0;
mcs_QAM=1024;
break;

default: 
break;

}
    double bitsPerSec=log2f(mcs_QAM)*encodeingRate*(rusize);
NS_LOG_FUNCTION("mcs_QAM============================================="<<mcs_QAM);
NS_LOG_FUNCTION("ruSize============================================="<<rusize);
NS_LOG_FUNCTION("bits==============================================="<<bits);
NS_LOG_FUNCTION("bitsPerSec========================================="<<bitsPerSec);

    symbols=bits/bitsPerSec;
NS_LOG_FUNCTION("symbol============================================="<<symbols);

    return symbols*0.0000136; //guard interval + symbol duration = 0.0000136sec
}



void RrMultiUserScheduler::MUTAX(){
  std::cout<<"\n MUTAX";


int lower=2,upper=11;

// PACKET SIZE | AID pairs
int count=dataStaPair.size();
//srand((int)time(0));
int i;

randomMCS.clear();
for(i=0;i<count;i++){
  int num= (rand()%(upper-lower+1))+lower;
  randomMCS.push_back(num);
  std::cout<<"num:"<<num<<"\n";
}


 // int bestRUConfig=0;
  double uBest = 0.0;
double bestLogQ=-100.0;
  std::vector<int> bestRUSET;
  std::vector<int> bestAssignment;

bestRUSET.clear();
bestAssignment.clear();

AcIndex primaryAc = m_edca->GetAccessCategory();
auto startItw = m_staList[primaryAc].begin ();
//const std::map<uint16_t, Mac48Address>& staList = m_apMac->GetStaList ();
//auto startItw = staList.find (m_startStation);
//auto t
//while(startItw!=staList.end())
while(startItw != m_staList[primaryAc].end())
{
//if(dataTransmitted.find(startItw->first)==dataTransmitted.end())
if(dataTransmitted.find(startItw->aid)==dataTransmitted.end())
{
	//dataTransmitted[startItw->first]=0;
  dataTransmitted[startItw->aid]=0;
	//totalTime[startItw->first]=0;
  totalTime[startItw->aid]=0;

}
//if(mapw.find(startItw->first)==mapw.end())
if(mapw.find(startItw->aid)==mapw.end())
{
	//mapw[startItw->first]=0;
  mapw[startItw->aid]=0;
}

startItw++;
}



  std::ifstream file;
  //if(m_low->GetPhy ()->GetChannelWidth ()==20){
  if(m_apMac->GetWifiPhy ()->GetChannelWidth () == 20) {  

    file.open("ru20.txt",std::ios::in);
  }
  //else if (m_low->GetPhy ()->GetChannelWidth ()==40){
  else if (m_apMac->GetWifiPhy ()->GetChannelWidth () == 40) {  

    file.open("ru40.txt",std::ios::in);
  }

  std::string line;

  if ( loopOutput ) {
    if(file.is_open()){
      std::cout<<"open";
    }else{
      std::cout<<"not open";
    }
  }

int bestConfig=0;
int configCount=0;
std::vector<int> RUset;
RUset.clear();
  
while(std::getline(file, line))
  {
    std::stringstream lineStream(line);
    if ( loopOutput ) {
      std::cout<<"line="<<line;
    }
    RUset.clear();
    int value;
    while(lineStream >> value){
      RUset.push_back(value);
    }

configCount++;

        // When all the integers have been read, add the 1D array, do your work
if((true) /*|| (RUset.size()== (dataStaPair.size()+1))*/){
    //for (int Mcs = 1; Mcs <= 12 ; Mcs++){
        //MaxRateRA(RUset,Mcs);
      ProportionalFair(RUset,1);

      double uCurr = 0.0;
      double logQ=0.0;

      if ( loopOutput ) {
        std::cout<<"\n currRUAlloc size="<<assignment.size();
        std::cout<<"\n dataStaPair size="<<dataStaPair.size();
        std::cout<<"\n must be same for MR and can be same for PF";
      }
	    //startItw = staList.find (m_startStation);
      AcIndex primaryAc = m_edca->GetAccessCategory();
      auto startItw = m_staList[primaryAc].begin ();

      for(unsigned int k=0;k<assignment.size();k++){
        if(assignment[k]!=-1){
          uCurr=uCurr+ (-(costMatrix[k][assignment[k]]));
          if(dataTransmitted[k]!=0){
		//double tp=(dataTransmitted[startItw->first]/totalTime[startItw->first]);
    double tp=(dataTransmitted[startItw->aid]/totalTime[startItw->aid]);
		//if(totalTime[startItw->first]==0){tp=0;}
    if(totalTime[startItw->aid]==0){tp=0;}
            double avgThroughput=tp+getDataRate(randomMCS[k],RUset[assignment[k]]);
if ( loopOutput ) {
  //std::cout<<"\n avgThroughput="<<avgThroughput<<" "<<dataTransmitted[startItw->first]<<" "<<totalTime[startItw->first];
  std::cout<<"\n avgThroughput="<<avgThroughput<<" "<<dataTransmitted[startItw->aid]<<" "<<totalTime[startItw->aid];
}

//if(log2(avgThroughput)<0)
 //           logQ=logQ+(-log2(avgThroughput));
//else
  //          logQ=logQ+log2(avgThroughput);
          }
	
        }
	startItw++;
      }
uCurr=(int)(uCurr*100)/100.0;
if ( loopOutput ) {
  std::cout<<"\nlogQ="<<logQ;
}
if(logQ!=0.0){
       //if(uCurr>uBest){       //Comparison with previous config set of ru, mcs
        if(logQ>bestLogQ){
        bestLogQ=logQ;
        uBest=uCurr;
       // bestMCS=Mcs;
        bestConfig=configCount;
        bestRUSET=RUset;
        if(bestAssignment.size()!=0){
          bestAssignment.clear();
          staAllocated.clear();
        }

        auto startIt = m_staInfo.begin();
        for (unsigned int v=0; v<assignment.size(); v++){
          bestAssignment.push_back(assignment[v]); 
          if(assignment[v]!=-1)
            staAllocated.push_back (*startIt);
          startIt++;
        } 
      } //if  end
}
else{
if(uCurr>=uBest){
  uBest=uCurr;
      //  bestMCS=Mcs;
        bestConfig=configCount;
        bestRUSET=RUset;
        if(bestAssignment.size()!=0){
          bestAssignment.clear();
          staAllocated.clear();
        }

        auto startIt = m_staInfo.begin();
        for (unsigned int v=0; v<assignment.size(); v++){
          bestAssignment.push_back(assignment[v]); 
          if(assignment[v]!=-1)
            staAllocated.push_back (*startIt);
          startIt++;
        } 
  }
} 
  //  } //mcs loop end 
  }//if condition before mcs end 
}//while file read loop end

file.close();

//std::cout<<"\n bestMCS="<<bestMCS;
if ( loopOutput ) {
  std::cout<<"\n bestRU="<<bestConfig;
  std::cout<<"\n bestLogQ="<<bestLogQ;
  std::cout<<"\n uBest="<<uBest;
}

minRuAlloc.clear();

std::vector<int>::size_type len = bestAssignment.size(); //client size
if ( loopOutput ) {
  std::cout<<"\n best assignment length="<<len<<"\n";
}

//startItw = staList.find (m_startStation);
startItw = m_staList[primaryAc].begin ();
double st=ns3::Simulator::Now().GetSeconds();
i=1;
int au=1;
int total=0;
std::vector<int>p1;
std::vector<int>p2;
//startItw = staList.find (m_startStation);
startItw = m_staList[primaryAc].begin ();
std::map<uint16_t,double>mapd;
std::string tt;
	std::ifstream MyMy("wt.txt");
	getline(MyMy,tt);
	std::stringstream check1(tt); 
	std::string in;
	while(getline(check1, in, ' ')) 
    	{ 
           mapd[au++]=stod(in);
	   total++;
    	}


for (unsigned i=0; i<len; i++){
  if(bestAssignment[i]!=-1){
	
    minRuAlloc.emplace_back(match(i+1,bestRUSET[bestAssignment[i]]));
	if(mapw[startItw->aid]!=0 && (st-mapw[startItw->aid])!=0)
  //if(mapw[startItw->first]!=0 && (st-mapw[startItw->first])!=0)
{
//mapd[startItw->first]=std::max(st-mapw[startItw->first],mapd[startItw->first]);
mapd[startItw->aid]=std::max(st-mapw[startItw->aid],mapd[startItw->aid]);
//std::cout<<"trans"<<i<<" "<<mapd[startItw->first];
std::cout<<"trans"<<i<<" "<<mapd[startItw->aid];
}
//mapw[startItw->first]=0;
mapw[startItw->aid]=0;

  }
else
{
	//if(mapw[startItw->first]==0){mapw[startItw->first]=st;}
  if(mapw[startItw->aid]==0){mapw[startItw->aid]=st;}



}
startItw++;
}

std::ofstream myfile;
myfile.open("wt.txt");

au=1;
std::cout<<"TT.....";
while(au<=total)
{
	std::cout<<mapd[au]<<" ";
	myfile<<mapd[au++]<<" ";
	

}
myfile<<"\n";
std::cout<<"\n";


/*Proportional fair:change this logic because size is equal to 
no. of clients under the assumption that all clients gets some resource*/
double tMax=0.0;
int jj=0;
int payloadSum=0;
unsigned kk=0;

auto b=dataStaPair.begin();
auto sti = staAllocated.begin();
//startItw = staList.find (m_startStation);
startItw = m_staList[primaryAc].begin ();

while(b!=dataStaPair.end()){
if(b->second==sti->second.aid){
  int p=b->first;
double x=(p*8.0)/1000000.0; //Mbits
  //dataTransmitted[startItw->first]=dataTransmitted[startItw->first]+x;
  dataTransmitted[startItw->aid]=dataTransmitted[startItw->aid]+x;
  payloadSum=payloadSum+(p-36);
  double tt=timeReq1(p,minRuAlloc[jj].b,randomMCS[jj]);
  if(tt>tMax)
    tMax=tt;
  jj++;
  sti++;
  }
 kk++;
startItw++;
 b++;
}
//startItw = staList.find (m_startStation);
startItw = m_staList[primaryAc].begin ();

b=dataStaPair.begin();
sti = staAllocated.begin();
kk=0;
while(b!=dataStaPair.end()){
  if(b->second==sti->second.aid){
   //totalTime[startItw->first]=totalTime[startItw->first]+tMax;
   totalTime[startItw->aid]=totalTime[startItw->aid]+tMax;
   sti++;
  }
startItw++;
kk++;
b++;
}



std::cout<<"payloadSum="<<payloadSum<<"\n";
//////////////////////////////////////////////////////////////////////////////////


int totalNoOfSTA=dataStaPair.size();
int staScheduled=minRuAlloc.size();

std::cout<<"Total no. of STA="<<totalNoOfSTA<<"\n";
std::cout<<"STA scheduled="<<staScheduled<<"\n";


tMax=tMax*1000000;
std::fstream f;
f.open("tmin.txt",std::ios::app|std::ios::out|std::ios::in);
f<<staScheduled<<" "<<payloadSum<<" "<<tMax<<"\n";
f.close();




//Mapping of rualloc to heruType
mappedRuAllocated.clear();
len = minRuAlloc.size(); //client size
std::cout<<"min ru length="<<len<<"\n";
for (unsigned i=0; i<len; i++){
 int ru=minRuAlloc[i].b;

 switch(ru){
  case 26: mappedRuAllocated.emplace_back(map(minRuAlloc[i].a,HeRu::RU_26_TONE));
           break;
  case 52: mappedRuAllocated.emplace_back(map(minRuAlloc[i].a,HeRu::RU_52_TONE));
           break;
  case 106: mappedRuAllocated.emplace_back(map(minRuAlloc[i].a,HeRu::RU_106_TONE));
           break;
  case 242:mappedRuAllocated.emplace_back(map(minRuAlloc[i].a,HeRu::RU_242_TONE));
           break;
  case 484: mappedRuAllocated.emplace_back(map(minRuAlloc[i].a,HeRu::RU_484_TONE));
           break;
  }
std::cout<<"Mapping============================="<<mappedRuAllocated[i].a<<" "<<mappedRuAllocated[i].b<<"\n";
}

std::cout<<"\nmapping done\n";
}

MultiUserScheduler::DlMuInfo
RrMultiUserScheduler::ComputeDlMuInfo (void)
{
  NS_LOG_FUNCTION (this);

  if (m_candidates.empty ())
  //if (m_staInfo.empty())
    {
      return DlMuInfo ();
    }

  uint16_t bw = m_apMac->GetWifiPhy ()->GetChannelWidth ();

  // compute how many stations can be granted an RU and the RU size
  // This number could be less than the available stations, it represents the number of RUs
  // available for assignment to candidate stations in the network
  std::size_t nRusAssigned = m_txParams.GetPsduInfoMap ().size (); // How does this map get built?
  //std::size_t nRusAssigned = mappedRuAllocated.size ();
  std::cout<<"m_sta_info size: ComputeDlOfdmaInfo "<<m_staInfo.size ()<<"\n";
  
  std::size_t nCentral26TonesRus;

  // This is being called the second time here after being called by TrySendingDlMuPpdu()
  // earlier for nRusAssigned we were passing count = min(Total Stations, Total Associated Stations to this AC)
  //HeRu::RuType ruType = HeRu::GetEqualSizedRusForStations (bw, nRusAssigned, nCentral26TonesRus);
  HeRu::RuType ruType;
  if(dataStaPair.size()>0) {
    while(mappedRuAllocated.size()==0){
      MUTAX();
    }
  }

  nRusAssigned=mappedRuAllocated.size();
  if(nRusAssigned==0) //in case no feasible ru allocation for current input exists.
  {
    std::cout<<"if(nRusAssigned==0) \n";
    //std::size_t nRusAs = m_staInfo.size ();
    nRusAssigned = m_staInfo.size ();
    //ruType = GetNumberAndTypeOfRus (bw, nRusAssigned);
    ruType = HeRu::GetEqualSizedRusForStations (bw, nRusAssigned, nCentral26TonesRus);
  }

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

  // = HeRu::RU_26_TONE;

///////////////////////////////////////////////////////

//std::cout<<nRusAssigned << " stations are being assigned a " << ruType << " RU\n";
 // NS_LOG_DEBUG (nRusAssigned << " stations are being assigned a " << ruType << " RU");
nRusAssigned=mappedRuAllocated.size();
  auto staInfoIt = m_staInfo.begin (); // iterator over the list of candidate receivers
  auto candidateIt = m_candidates.begin ();
  //auto simpleCandidateIt = m_simpleCandidates.begin();
  auto mapIt=mappedRuAllocated.begin();
  bool has[m_staInfo.size()+1];
  for ( u_int16_t j = 0; j <= m_staInfo.size(); j++ ) {
    has[j] = false;
  }
  std::cout<<"mappedRuAllocated size:::::::::::::::::::::::::::::::"<<mappedRuAllocated.size()<<"\n";
std::list<CandidateInfo> copy_m_candidates;
//std::list<CandInfo> copy_m_simpleCandidates;
std::list<std::pair<Mac48Address, DlPerStaInfo>> copy_m_staInfo;
std::list<std::pair<uint32_t, uint16_t>> copy_dataStaPair;
auto dataIt=dataStaPair.begin();

// Reordering of the stations in the m_staInfo and dataStaPair list
// Stations which have been allocated an RU are placed first,
// Stations not allocated an RU placed at the end of the list
for(std::size_t i=0;i<nRusAssigned;i++)
{
  std::cout<< "dataStaPair index " << mapIt->a << " assigned a RU\n";
  // map *it=(map)mapIt;
  has[mapIt->a]=true;
  mapIt++;
} 
for(u_int16_t i=1;i<=m_staInfo.size();i++)
{
  if(has[i])
  {
    std::cout << "STA " << staInfoIt->second.aid << " assigned an RU\n";

    copy_m_candidates.push_back(*candidateIt);
    //copy_m_simpleCandidates.push_back(*simpleCandidateIt);
    copy_m_staInfo.push_back(*staInfoIt);
    copy_dataStaPair.push_back(*dataIt);
  }
  candidateIt++;
  //simpleCandidateIt++;
  dataIt++;
  staInfoIt++;
}
candidateIt=m_candidates.begin();
//simpleCandidateIt = m_simpleCandidates.begin();
dataIt=dataStaPair.begin();
staInfoIt = m_staInfo.begin ();
for(u_int16_t i=1;i<=m_staInfo.size();i++)
{
  if(!has[i])
  {
    std::cout << "STA " << staInfoIt->second.aid << " NOT assigned an RU\n";

    copy_m_candidates.push_back(*candidateIt);
    //copy_m_simpleCandidates.push_back(*simpleCandidateIt);
    copy_dataStaPair.push_back(*dataIt);
    copy_m_staInfo.push_back(*staInfoIt);
  }
  candidateIt++;
  //simpleCandidateIt++;
  dataIt++;
  staInfoIt++;
}

//m_simpleCandidates.clear();
//m_simpleCandidates.assign(copy_m_simpleCandidates.begin(), copy_m_simpleCandidates.end());
m_candidates.clear();
m_candidates.assign(copy_m_candidates.begin(), copy_m_candidates.end());
m_staInfo.clear();
m_staInfo.assign(copy_m_staInfo.begin(),copy_m_staInfo.end());
dataStaPair.clear();
dataStaPair.assign(copy_dataStaPair.begin(),copy_dataStaPair.end());

staInfoIt = m_staInfo.begin ();

  std::map<Mac48Address,DlPerStaInfo> dlStaInfo;
  // Assign the reordered staInfo to the DlOfdmaInfo
  for (std::size_t i = 0; i < nRusAssigned; i++)
    {
      NS_ASSERT (staInfoIt != m_staInfo.end ());
      std::cout << "STA " << staInfoIt->second.aid << " inserted into dlStaInfo for RU assignment\n";

      dlStaInfo.insert (*staInfoIt);
      staInfoIt++;
    }
  

  // if not all the stations are assigned an RU, the first station to serve next
  // time is the first one that was not served this time
  if (nRusAssigned < m_staInfo.size ())
    {
      NS_ASSERT (staInfoIt != m_staInfo.end ());
      m_startStation = staInfoIt->second.aid;
      std::cout<<"Next station to serve has AID=" << m_startStation<<"\n";
    }

  NS_LOG_DEBUG ("Next station to serve has AID=" << m_startStation);

  DlMuInfo dlMuInfo;

  // We have to update the TXVECTOR
  dlMuInfo.txParams.m_txVector.SetPreambleType (m_txParams.m_txVector.GetPreambleType ());
  dlMuInfo.txParams.m_txVector.SetChannelWidth (m_txParams.m_txVector.GetChannelWidth ());
  dlMuInfo.txParams.m_txVector.SetGuardInterval (m_txParams.m_txVector.GetGuardInterval ());
  dlMuInfo.txParams.m_txVector.SetBssColor (m_txParams.m_txVector.GetBssColor ());

  candidateIt = m_candidates.begin (); // iterator over the list of candidate receivers

  for (std::size_t i = 0; i < nRusAssigned + nCentral26TonesRus; i++)
    {
      NS_ASSERT (candidateIt != m_candidates.end ());

      uint16_t staId = candidateIt->first->aid;
      //std::cout << "Attempting to get WifiMode for station = " << staId << "\n";
      // AssignRuIndices will be called below to set RuSpec
      // Is it here that the MCS and the RU are being finally assigned to the candidates
      if ( minRuAlloc.size() != 0 ) {
          dlMuInfo.txParams.m_txVector.SetHeMuUserInfo (staId,
                                                    {{mappedRuAllocated[i].b, 1, false},
                                                      m_txParams.m_txVector.GetMode (staId),
                                                      m_txParams.m_txVector.GetNss (staId)});
      }
      else {
          dlMuInfo.txParams.m_txVector.SetHeMuUserInfo (staId,
                                                    {{(i < nRusAssigned ? ruType : HeRu::RU_26_TONE), 1, false},
                                                      m_txParams.m_txVector.GetMode (staId),
                                                      m_txParams.m_txVector.GetNss (staId)});
      }
      
      candidateIt++;
    }

  // remove candidates that will not be served (they were not assigned any RU)
  m_candidates.erase (candidateIt, m_candidates.end ());

  // candidateIt = m_candidates.begin (); // iterator over the list of candidate receivers

  // for (std::size_t i = 0; i < nRusAssigned; i++)
  // {
  //   candidateIt++;
  // }

  // // remove candidates that will not be served (they were not assigned any RU)
  // m_candidates.erase (candidateIt, m_candidates.end ());

  // simpleCandidateIt = m_simpleCandidates.begin (); // iterator over the list of candidate receivers

  // for (std::size_t i = 0; i < nRusAssigned; i++)
  // {
  //   simpleCandidateIt++;
  // }

  // // remove candidates that will not be served (they were not assigned any RU)
  // m_simpleCandidates.erase (simpleCandidateIt, m_simpleCandidates.end ());
  
  // Ptr<const WifiMacQueueItem> mpdu = m_edca->PeekNextMpdu ();
  // WifiTxVector suTxVector = GetWifiRemoteStationManager ()->GetDataTxVector (mpdu->GetHeader ());

  // unsigned i = 0;
  // for (auto& sta : dlStaInfo)
  //   {
  //       ///////Naman///////////////////
  //       if(minRuAlloc.size()!=0){
  //         dlMuInfo.txParams.m_txVector.SetHeMuUserInfo (sta.second.aid, {{mappedRuAllocated[i].b, 1, false}, suTxVector.GetMode(), suTxVector.GetNss()});
  //         i++;
  //       }
  //       else      
  //         dlMuInfo.txParams.m_txVector.SetHeMuUserInfo (sta.second.aid, {{ruType, 1, false}, suTxVector.GetMode(), suTxVector.GetNss()});
  //       //////////////////////////////
  //   }

  
  //AssignRuIndices (dlMuInfo.txParams.m_txVector);

  if (ruType == HeRu::RU_2x996_TONE) // Not of our concern
    {
      HeRu::RuSpec ru = {ruType, 1, true};
      NS_LOG_DEBUG ("STA " << m_staInfo.front ().first << " assigned " << ru);
      dlMuInfo.txParams.m_txVector.SetRu (ru, m_staInfo.front ().second.aid);
    }
  else
    {
      std::vector<bool> primary80MHzSet {true};

      if (bw == 160)
        {
          primary80MHzSet.push_back (false);
          bw = 80;
        }

      auto mapItt = dlStaInfo.begin ();
      
      //Naman: change this logic to assign ru of your choie by simply manipulating the index
      if(mappedRuAllocated.size()!=0)
      {
        std::size_t ru26 = 0;
        std::size_t ru52 = 0;
        std::size_t ru106 = 0;
        std::size_t ru242 = 0;
        std::size_t ru484 = 0;
        std::vector<int>::size_type len = mappedRuAllocated.size();
        
        for (unsigned i=0; i<len; i++)
        {
          HeRu::RuSpec ru;
          
          switch(mappedRuAllocated[i].b)
          {
            case HeRu::RU_26_TONE: {
              ru26++;
              ru = {mappedRuAllocated[i].b,ru26, true};
              break;
            }
            case HeRu::RU_52_TONE: ru52++;
            ru = {mappedRuAllocated[i].b,ru52, true};
            break;
            case HeRu::RU_106_TONE: ru106++;
            ru = {mappedRuAllocated[i].b,ru106, true};
            break;
            case HeRu::RU_242_TONE: ru242++;
            ru = {mappedRuAllocated[i].b,ru242, true};
            break;
            case HeRu::RU_484_TONE: ru484++;
            ru = {mappedRuAllocated[i].b,ru484, true};
            break;
            default: break;
          }

          NS_LOG_DEBUG ("STA " << mapItt->first << " assigned " << ru);
          std::cout << "STA " << mapItt->second.aid << " assigned " << ru << "\n";
          dlMuInfo.txParams.m_txVector.SetRu (ru, mapItt->second.aid); // This is the crux, setting the RuType in our txVector
          mapItt++;
        }
      }
      else
      {
        for (auto primary80MHz : primary80MHzSet)
        {
          std::cout<<"RUType = "<< ruType <<"\n";
          for (std::size_t ruIndex = 1; ruIndex <= std::min(HeRu::m_heRuSubcarrierGroups.at ({bw, ruType}).size (), m_staInfo.size()); ruIndex++)
          {
            NS_ASSERT (mapItt != dlStaInfo.end ());
            HeRu::RuSpec ru = {ruType, ruIndex, primary80MHz};
            NS_LOG_DEBUG ("STA " << mapItt->first << " assigned " << ru);
            dlMuInfo.txParams.m_txVector.SetRu (ru, mapItt->second.aid);
            mapItt++;
          }
        }
      }
    }

  m_txParams.Clear ();
  minRuAlloc.clear();
  mappedRuAllocated.clear(); // Job done

  if( randomMCS.size() > 1 ) {
    
    unsigned int x=0;
    auto userInfoMap = dlMuInfo.txParams.m_txVector.GetHeMuUserInfoMap ();
    for (auto& userInfo : userInfoMap) {
          uint8_t mcs = randomMCS.at(x);
          NS_LOG_FUNCTION("MCS"<<mcs);
          dlMuInfo.txParams.m_txVector.SetHeMuUserInfo (userInfo.first, {userInfo.second.ru,
                                                        HePhy::GetHeMcs (mcs),
                                                        userInfo.second.nss});
          x++;
    } 
  }

randomMCS.clear();

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
  

  // AcIndex primaryAc = m_edca->GetAccessCategory ();

  // // This credit/Debit systems seems unnecessary for a PF scheduler

  // // The amount of credits received by each station equals the TX duration (in
  // // microseconds) divided by the number of stations.
  // double creditsPerSta = dlMuInfo.txParams.m_txDuration.ToDouble (Time::US)
  //                       / m_staList[primaryAc].size ();
  // // Transmitting stations have to pay a number of credits equal to the TX duration
  // // (in microseconds) times the allocated bandwidth share.
  // double debitsPerMhz = dlMuInfo.txParams.m_txDuration.ToDouble (Time::US)
  //                       / (nRusAssigned * HeRu::GetBandwidth (ruType)
  //                         + nCentral26TonesRus * HeRu::GetBandwidth (HeRu::RU_26_TONE));

  // // assign credits to all stations
  // for (auto& sta : m_staList[primaryAc])
  //   {
  //     sta.credits += creditsPerSta;
  //     sta.credits = std::min (sta.credits, m_maxCredits.ToDouble (Time::US));
  //   }

  // // subtract debits to the selected stations
  // candidateIt = m_candidates.begin ();

  // for (std::size_t i = 0; i < nRusAssigned + nCentral26TonesRus; i++)
  //   {
  //     NS_ASSERT (candidateIt != m_candidates.end ());

  //     candidateIt->first->credits -= debitsPerMhz * HeRu::GetBandwidth (i < nRusAssigned ? ruType : HeRu::RU_26_TONE);

  //     candidateIt++;
  //   }

  // // sort the list in decreasing order of credits
  // m_staList[primaryAc].sort ([] (const MasterInfo& a, const MasterInfo& b)
  //                             { return a.credits > b.credits; });

  //NS_LOG_DEBUG ("Next station to serve has AID=" << m_staList[primaryAc].front ().aid);

  return dlMuInfo;
}

void
RrMultiUserScheduler::AssignRuIndices (WifiTxVector& txVector)
{
  NS_LOG_FUNCTION (this << txVector);

  uint8_t bw = txVector.GetChannelWidth ();

  // find the RU types allocated in the TXVECTOR
  // No need to make any changes here once the correct RU Types have been set for the txVector
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

  // This assertion says that all the STAs would be allocated the same type of RU if not allocated centeral 26 tone RU
  // This is of course not the case when using PF
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
RrMultiUserScheduler::ComputeUlMuInfo (void)
{
  return UlMuInfo {m_trigger, m_tbPpduDuration, std::move (m_txParams)};
}

} //namespace ns3
