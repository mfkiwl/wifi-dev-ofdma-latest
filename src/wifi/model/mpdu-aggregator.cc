/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2013
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
 * Author: Ghada Badawy <gbadawy@gmail.com>
 *         Stefano Avallone <stavallo@unina.it>
 */

#include "ns3/log.h"
#include "ns3/packet.h"
#include "mpdu-aggregator.h"
#include "ampdu-subframe-header.h"
#include "wifi-phy.h"
#include "wifi-tx-vector.h"
#include "wifi-remote-station-manager.h"
#include "wifi-mac-queue-item.h"
#include "wifi-mac-queue.h"
#include "ns3/mac48-address.h"
#include "msdu-aggregator.h"
#include "wifi-net-device.h"
#include "ns3/ht-capabilities.h"
#include "ns3/vht-capabilities.h"
#include "ns3/he-capabilities.h"
#include "regular-wifi-mac.h"
#include "ctrl-headers.h"
#include "wifi-mac-trailer.h"
#include "wifi-tx-parameters.h"
#include <fstream>
#include <vector>
#include <map>

NS_LOG_COMPONENT_DEFINE ("MpduAggregator");

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (MpduAggregator);

TypeId
MpduAggregator::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::MpduAggregator")
    .SetParent<Object> ()
    .SetGroupName ("Wifi")
    .AddConstructor<MpduAggregator> ()
  ;
  return tid;
}

MpduAggregator::MpduAggregator ()
 : m_aggregationStatsEnabled(false)
{
}

MpduAggregator::~MpduAggregator ()
{
}

void
MpduAggregator::DoDispose ()
{
  m_mac = 0;
  Object::DoDispose ();
}

void
MpduAggregator::SetWifiMac (const Ptr<RegularWifiMac> mac)
{
  NS_LOG_FUNCTION (this << mac);
  m_mac = mac;
}

void
MpduAggregator::Aggregate (Ptr<const WifiMacQueueItem> mpdu, Ptr<Packet> ampdu, bool isSingle)
{
  NS_LOG_FUNCTION (mpdu << ampdu << isSingle);
  NS_ASSERT (ampdu);
  // if isSingle is true, then ampdu must be empty
  NS_ASSERT (!isSingle || ampdu->GetSize () == 0);

  // pad the previous A-MPDU subframe if the A-MPDU is not empty
  if (ampdu->GetSize () > 0)
    {
      uint8_t padding = CalculatePadding (ampdu->GetSize ());

      if (padding)
        {
          Ptr<Packet> pad = Create<Packet> (padding);
          ampdu->AddAtEnd (pad);
        }
    }

  // add MPDU header and trailer
  Ptr<Packet> tmp = mpdu->GetPacket ()->Copy ();
  tmp->AddHeader (mpdu->GetHeader ());
  AddWifiMacTrailer (tmp);

  // add A-MPDU subframe header and MPDU to the A-MPDU
  AmpduSubframeHeader hdr = GetAmpduSubframeHeader (static_cast<uint16_t> (tmp->GetSize ()), isSingle);

  tmp->AddHeader (hdr);
  ampdu->AddAtEnd (tmp);
}

uint32_t
MpduAggregator::GetSizeIfAggregated (uint32_t mpduSize, uint32_t ampduSize)
{
  NS_LOG_FUNCTION (mpduSize << ampduSize);

  return ampduSize + CalculatePadding (ampduSize) + 4 + mpduSize;
}

uint32_t
MpduAggregator::GetMaxAmpduSize (Mac48Address recipient, uint8_t tid,
                                 WifiModulationClass modulation) const
{
  NS_LOG_FUNCTION (this << recipient << +tid << modulation);

  AcIndex ac = QosUtilsMapTidToAc (tid);

  // Find the A-MPDU max size configured on this device
  UintegerValue size;

  switch (ac)
    {
      case AC_BE:
        m_mac->GetAttribute ("BE_MaxAmpduSize", size);
        break;
      case AC_BK:
        m_mac->GetAttribute ("BK_MaxAmpduSize", size);
        break;
      case AC_VI:
        m_mac->GetAttribute ("VI_MaxAmpduSize", size);
        break;
      case AC_VO:
        m_mac->GetAttribute ("VO_MaxAmpduSize", size);
        break;
      default:
        NS_ABORT_MSG ("Unknown AC " << ac);
        return 0;
    }

  uint32_t maxAmpduSize = size.Get ();

  if (maxAmpduSize == 0)
    {
      NS_LOG_DEBUG ("A-MPDU Aggregation is disabled on this station for AC " << ac);
      return 0;
    }

  Ptr<WifiRemoteStationManager> stationManager = m_mac->GetWifiRemoteStationManager ();
  NS_ASSERT (stationManager);

  // Retrieve the Capabilities elements advertised by the recipient
  Ptr<const HeCapabilities> heCapabilities = stationManager->GetStationHeCapabilities (recipient);
  Ptr<const VhtCapabilities> vhtCapabilities = stationManager->GetStationVhtCapabilities (recipient);
  Ptr<const HtCapabilities> htCapabilities = stationManager->GetStationHtCapabilities (recipient);

  // Determine the constraint imposed by the recipient based on the PPDU
  // format used to transmit the A-MPDU
  if (modulation == WIFI_MOD_CLASS_HE)
    {
      NS_ABORT_MSG_IF (!heCapabilities, "HE Capabilities element not received");

      maxAmpduSize = std::min (maxAmpduSize, heCapabilities->GetMaxAmpduLength ());
    }
  else if (modulation == WIFI_MOD_CLASS_VHT)
    {
      NS_ABORT_MSG_IF (!vhtCapabilities, "VHT Capabilities element not received");

      maxAmpduSize = std::min (maxAmpduSize, vhtCapabilities->GetMaxAmpduLength ());
    }
  else if (modulation == WIFI_MOD_CLASS_HT)
    {
      NS_ABORT_MSG_IF (!htCapabilities, "HT Capabilities element not received");

      maxAmpduSize = std::min (maxAmpduSize, htCapabilities->GetMaxAmpduLength ());
    }
  else  // non-HT PPDU
    {
      NS_LOG_DEBUG ("A-MPDU aggregation is not available for non-HT PHYs");

      maxAmpduSize = 0;
    }

  return maxAmpduSize;
}

uint8_t
MpduAggregator::CalculatePadding (uint32_t ampduSize)
{
  return (4 - (ampduSize % 4 )) % 4;
}

AmpduSubframeHeader
MpduAggregator::GetAmpduSubframeHeader (uint16_t mpduSize, bool isSingle)
{
  AmpduSubframeHeader hdr;
  hdr.SetLength (mpduSize);
  if (isSingle)
    {
      hdr.SetEof (1);
    }
  return hdr;
}

std::vector<Ptr<WifiMacQueueItem>>
MpduAggregator::GetNextAmpdu (Ptr<WifiMacQueueItem> mpdu, WifiTxParameters& txParams,
                              Time availableTime, WifiMacQueueItem::QueueIteratorPair queueIt)
{
  NS_LOG_FUNCTION (this << *mpdu << &txParams << availableTime);

  std::vector<Ptr<WifiMacQueueItem>> mpduList;
  Mac48Address recipient = mpdu->GetHeader ().GetAddr1 ();

  if ( m_aggregationStatsEnabled ) {

    // Initilaize the maps for storing the aggregation statistics
    // for each user
    auto statsIt = m_aggregationStats.find(recipient);
    if ( statsIt == m_aggregationStats.end() ) {

      std::vector<uint64_t> distributionVec;
      distributionVec.assign(64, 0);
      
      m_aggregationStats.insert(std::make_pair(recipient, distributionVec));
    }

    auto reasonsIt = m_aggregationStopReasons.find(recipient);
    if ( reasonsIt == m_aggregationStopReasons.end() ) {

      std::vector<uint64_t> distributionVec;
      distributionVec.assign(3, 0);
      
      m_aggregationStopReasons.insert(std::make_pair(recipient, distributionVec));
    }
  }

  std::vector<uint64_t> *reasonsVec;
  if ( m_aggregationStatsEnabled) 
    reasonsVec = &(m_aggregationStopReasons.at(recipient));
  
  NS_ASSERT (mpdu->GetHeader ().IsQosData () && !recipient.IsBroadcast ());
  uint8_t tid = mpdu->GetHeader ().GetQosTid ();

  Ptr<QosTxop> qosTxop = m_mac->GetQosTxop (tid);
  NS_ASSERT (qosTxop != 0);

  //Have to make sure that the block ack agreement is established and A-MPDU is enabled
  if (qosTxop->GetBaAgreementEstablished (recipient, tid)
      && GetMaxAmpduSize (recipient, tid, txParams.m_txVector.GetModulationClass ()) > 0)
    {
      /* here is performed MPDU aggregation */
      Ptr<WifiMacQueueItem> nextMpdu = mpdu;

      //std::ofstream aggOutputFile;
      //std::ostringstream oss;

      
      // oss << recipient << ".txt";
      // if ( oss.str() == "00:00:00:00:00:24.txt" || oss.str() == "00:00:00:00:00:1a.txt" || oss.str() == "00:00:00:00:00:08.txt" )
      //   aggOutputFile.open(oss.str(), std::ios_base::app | std::ios_base::out);

      // if ( oss.str() == "00:00:00:00:00:24.txt" || oss.str() == "00:00:00:00:00:1a.txt" || oss.str() == "00:00:00:00:00:08.txt" )
      //   aggOutputFile << "=========================== START ===================================\n";

      while (nextMpdu != 0)
        {
          // if ( oss.str() == "00:00:00:00:00:24.txt" || oss.str() == "00:00:00:00:00:1a.txt" || oss.str() == "00:00:00:00:00:08.txt") {
          //   aggOutputFile << "Adding packet with sequence number " << nextMpdu->GetHeader ().GetSequenceNumber ()
          //                 << " to A-MPDU (destined to " << recipient << "), packet size = " << nextMpdu->GetSize ()
          //                 << ", A-MPDU size = " << txParams.GetSize (recipient) << "\n";
          // }

          // if we are here, nextMpdu can be aggregated to the A-MPDU.
          NS_LOG_DEBUG ("Adding packet with sequence number " << nextMpdu->GetHeader ().GetSequenceNumber ()
                        << " to A-MPDU, packet size = " << nextMpdu->GetSize ()
                        << ", A-MPDU size = " << txParams.GetSize (recipient));

          mpduList.push_back (nextMpdu);

          // If allowed by the BA agreement, get the next MPDU
          nextMpdu = 0;

          Ptr<const WifiMacQueueItem> peekedMpdu;
          peekedMpdu = qosTxop->PeekNextMpdu (queueIt, tid, recipient);
          if (peekedMpdu != 0)
            {
              // PeekNextMpdu() does not return an MPDU that is beyond the transmit window
              NS_ASSERT (IsInWindow (peekedMpdu->GetHeader ().GetSequenceNumber (),
                                     qosTxop->GetBaStartingSequence (recipient, tid),
                                     qosTxop->GetBaBufferSize (recipient, tid)));

              // get the next MPDU to aggregate, provided that the constraints on size
              // and duration limit are met. Note that the returned MPDU differs from
              // the peeked MPDU if A-MSDU aggregation is enabled.
              NS_LOG_DEBUG ("Trying to aggregate another MPDU");
              nextMpdu = qosTxop->GetNextMpdu (peekedMpdu, txParams, availableTime, false, queueIt);
              if ( nextMpdu == 0 ) {
                
                if ( m_aggregationStatsEnabled )
                  (*reasonsVec)[1]++; // Aggregation stopped because aggregating more MPDUs violated TXOP limit
              }
            }
            else {
              if ( m_aggregationStatsEnabled )
                (*reasonsVec)[0]++; // Aggregation stopped because Sequence no. space full or because no more MPDUs to aggregate
            }
        }

      // if ( oss.str() == "00:00:00:00:00:24.txt" || oss.str() == "00:00:00:00:00:1a.txt" || oss.str() == "00:00:00:00:00:08.txt") {
      //   aggOutputFile << "=========================== End ===================================\n";
      //   aggOutputFile.close();
      // }

      // oss.str("");
      // oss.clear();

      std::vector<uint64_t> *statsVec;
      if ( m_aggregationStatsEnabled) {
        statsVec = &(m_aggregationStats.at(recipient));
        (*statsVec)[mpduList.size() - 1]++;
      }

      if (mpduList.size () == 1)
        {
          // return an empty vector if it was not possible to aggregate at least two MPDUs
          mpduList.clear ();
        }
    }

  return mpduList;
}

std::map<Mac48Address, std::vector<uint64_t>>
MpduAggregator::GetAggregationStats(void)
{
  return m_aggregationStats;
}

std::map<Mac48Address, std::vector<uint64_t>>
MpduAggregator::GetAggregationStopReasons(void)
{
  return m_aggregationStopReasons;
}

void
MpduAggregator::EnableAggregationStats(bool enable) {
  m_aggregationStatsEnabled = enable;
}

} //namespace ns3
