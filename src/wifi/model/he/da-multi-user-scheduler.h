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

#ifndef DA_MULTI_USER_SCHEDULER_H
#define DA_MULTI_USER_SCHEDULER_H

#include "multi-user-scheduler.h"
#include <list>
#include <iostream>
#include <vector>
#include <string>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/maximum_weighted_matching.hpp>
//#include <ilcplex/ilocplex.h>

using namespace boost;

namespace ns3 {

/**
 * \ingroup wifi
 *
 * DaMultiUserScheduler is a simple OFDMA scheduler that indicates to perform a DL OFDMA
 * transmission if the AP has frames to transmit to at least one station.
 * DaMultiUserScheduler assigns RUs of equal size (in terms of tones) to stations to
 * which the AP has frames to transmit belonging to the AC who gained access to the
 * channel or higher. The maximum number of stations that can be granted an RU
 * is configurable. Associated stations are served in a round robin fashion.
 *
 * \todo Take the supported channel width of the stations into account while selecting
 * stations and assigning RUs to them.
 */
class DaMultiUserScheduler : public MultiUserScheduler
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  DaMultiUserScheduler ();
  virtual ~DaMultiUserScheduler ();

  /**
   * \brief Check if Deadline Constrained Traffic has began
   * \return True if it STA association has ended, and deadline constrained traffic has began
   */
  bool CheckDeadlineConstrainedTrafficStarted(void);

  /**
   * \brief Inform the scheduler that Deadline Constrained Traffic has began
   *
   */
  void NotifyDeadlineConstrainedTrafficStarted(void);

  /**
   * \brief Set the Time Offset used to map Timestamps to Rounds
   */ 
  void SetRoundTimeOffset(double);

  void SetTimeQuantaForRound(double quanta); 

  uint32_t GetRoundFromTimestamp(double);

  /**
   * \brief Receive information related to packet generation rate from
   * the simulation, this is used to generate the packet schedule for
   * a set of rounds
   */
  void SetStaPacketInfo(std::map <uint32_t, std::vector<uint32_t>>);

  uint32_t LCM(int[], int);
  
  /**
   * \brief For how many rounds should a packet schedule be generated?
   */
  uint32_t GetRoundsPerSchedule(void);
  
  /**
   * \brief How many packets are there in a generated schedule?
   */
  uint32_t GetPacketsPerSchedule(void);

  void GeneratePacketScheduleForSetRounds(void);
  
  uint32_t GetRusPerRound(HeRu::RuType);

  HeRu::RuType GetRuTypePerRound(void);
  
  /**
   * \brief The Maximum Weighted Matching algorithm maps packet
   * index to ru index, this utility function is used to get the
   * round index from the ru index.
   */
  uint32_t GetRoundFromVertexIndex(uint32_t vj, uint32_t rus);

  void MaximumWeightedMatching(void);

  void ILPSolver(void);

  //void populatebyrow(IloModel, IloNumVarArray, IloRangeArray);
  
  /**
   * \brief Generates the Map for mapping incoming MPDU to
   * a specific packet index, see definition for a detailed
   * example
   */
  void GenerateMpduToCurrPacketMap(void);

protected:
  void DoDispose (void) override;
  void DoInitialize (void) override;

private:
  TxFormat SelectTxFormat (void) override;
  DlMuInfo ComputeDlMuInfo (void) override;
  UlMuInfo ComputeUlMuInfo (void) override;

  /**
   * Check if it is possible to send a BSRP Trigger Frame given the current
   * time limits.
   *
   * \return UL_MU_TX if it is possible to send a BSRP TF, NO_TX otherwise
   */
  virtual TxFormat TrySendingBsrpTf (void);

  /**
   * Check if it is possible to send a Basic Trigger Frame given the current
   * time limits.
   *
   * \return UL_MU_TX if it is possible to send a Basic TF, DL_MU_TX if we can try
   *         to send a DL MU PPDU and NO_TX if the remaining time is too short
   */
  virtual TxFormat TrySendingBasicTf (void);

  /**
   * Check if it is possible to send a DL MU PPDU given the current
   * time limits.
   *
   * \return DL_MU_TX if it is possible to send a DL MU PPDU, SU_TX if a SU PPDU
   *         can be transmitted (e.g., there are no HE stations associated or sending
   *         a DL MU PPDU is not possible and m_forceDlOfdma is false) or NO_TX otherwise
   */
  virtual TxFormat TrySendingDlMuPpdu (void);

  /**
   * Assign an RU index to all the RUs allocated by the given TXVECTOR. Allocated
   * RUs must all have the same size, except for allocated central 26-tone RUs.
   *
   * \param txVector the given TXVECTOR
   */
  void AssignRuIndices (WifiTxVector& txVector);

  /**
   * Notify the scheduler that a station associated with the AP
   *
   * \param aid the AID of the station
   * \param address the MAC address of the station
   */
  void NotifyStationAssociated (uint16_t aid, Mac48Address address);
  /**
   * Notify the scheduler that a station deassociated with the AP
   *
   * \param aid the AID of the station
   * \param address the MAC address of the station
   */
  void NotifyStationDeassociated (uint16_t aid, Mac48Address address);

  /**
   * Information used to sort stations
   */
  struct MasterInfo
  {
    uint16_t aid;                 //!< station's AID
    Mac48Address address;         //!< station's MAC Address
    double credits;               //!< credits accumulated by the station
  };

  /**
   * Information stored for candidate stations
   */
  typedef std::pair<std::list<MasterInfo>::iterator, Ptr<const WifiMacQueueItem>> CandidateInfo;

  typedef property< edge_weight_t, float, property< edge_index_t, int > > EdgeProperty; // Maximum Weighted Matching uses these
  typedef adjacency_list< vecS, vecS, undirectedS, no_property, EdgeProperty > my_graph;

  uint16_t m_nStations;                                  //!< Number of stations/slots to fill
  bool m_hasDeadlineConstrainedTrafficStarted;          //!< Has the deadline contrained traffic started or
                                                        //!< still waiting for STAs to associate?
  double roundTimeOffset;                               //!< Subtract this value from the current time to estimate round
  double currTimeUs;                                    //!< The current time expressed in microseconds
  double timeQuanta;
  bool isRoundOffsetSet;
  uint32_t m_roundsPerSchedule;                         //!< No. of rounds for which the schedule is generated        
  uint32_t m_packetsPerSchedule;                        //!< No. of packets for which the schedule is generated                              
  bool m_enableTxopSharing;                             //!< allow A-MPDUs of different TIDs in a DL MU PPDU
  bool m_forceDlOfdma;                                  //!< return DL_OFDMA even if no DL MU PPDU was built
  bool m_enableUlOfdma;                                 //!< enable the scheduler to also return UL_OFDMA
  bool m_enableBsrp;                                    //!< send a BSRP before an UL MU transmission
  bool m_useCentral26TonesRus;                          //!< whether to allocate central 26-tone RUs
  uint32_t m_ulPsduSize;                                //!< the size in byte of the solicited PSDU
  std::map<AcIndex, std::list<MasterInfo>> m_staList;   //!< Per-AC list of stations (next to serve first)
  std::map <uint32_t /* AID */, std::vector<uint32_t> /* Packet Time period, Deadline, Penalty */> m_staPacketInfo;
  std::vector<std::vector<uint32_t>> m_packetSchedule;
  std::map <uint32_t /* PID */, uint32_t /* ROUND ID */> m_packetToRoundMap;
  std::list<CandidateInfo> m_candidates;                //!< Candidate stations for MU TX
  std::list<CandidateInfo> m_roundCandidates;           //!< Candidate with packets to be scheduled this round
  std::map<uint32_t, uint32_t> m_mpduToCurrPacketMap;   //!< This maps a user to the current packet index for mpdu assignment
  Time m_maxCredits;                                    //!< Max amount of credits a station can have
  Ptr<WifiMacQueueItem> m_trigger;                      //!< Trigger Frame to send
  Time m_tbPpduDuration;                                //!< Duration of the solicited TB PPDUs
  WifiTxParameters m_txParams;                          //!< TX parameters
  TriggerFrameType m_ulTriggerType;                     //!< Trigger Frame type for UL MU
};

} //namespace ns3

#endif /* RR_MULTI_USER_SCHEDULER_H */
