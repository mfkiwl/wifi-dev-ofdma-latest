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

#ifndef PF_MULTI_USER_SCHEDULER_H
#define PF_MULTI_USER_SCHEDULER_H

#include "multi-user-scheduler.h"
#include <list>
#include "ns3/Hungarian.h"
#include <ilcplex/ilocplex.h>

namespace ns3 {

/**
 * \ingroup wifi
 *
 * PfMultiUserScheduler is a simple OFDMA scheduler that indicates to perform a DL OFDMA
 * transmission if the AP has frames to transmit to at least one station.
 * PfMultiUserScheduler assigns RUs in a proportionally fair manner.
 * 
 * Originally Implemented by: Naman Gupta & Abhishek Gupta
 * Ported to new version by: Harshal Dev
 *
 * \todo Take the supported channel width of the stations into account while selecting
 * stations and assigning RUs to them.
 */
class PfMultiUserScheduler : public MultiUserScheduler
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  PfMultiUserScheduler ();
  virtual ~PfMultiUserScheduler ();

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

  struct DlPerStaInfo
  {
    uint16_t aid;                //!< association ID
    uint8_t tid;                 //!< TID
  };

  struct CandInfo 
  {
    uint16_t aid;                 //!< station's AID
    Mac48Address address;         //!< station's MAC Address
    Ptr<const WifiMacQueueItem> mpdu;
  };

  /**
   * Information stored for candidate stations
   */
  typedef std::pair<std::list<MasterInfo>::iterator, Ptr<const WifiMacQueueItem>> CandidateInfo;

  bool loopOutput;
  bool specificOutput;
  uint8_t m_nStations;
  uint32_t m_mcs;                                  //!< Number of stations/slots to fill
  uint16_t m_startStation;
  std::list<std::pair<Mac48Address, DlPerStaInfo>> m_staInfo;
  bool m_enableTxopSharing;                             //!< allow A-MPDUs of different TIDs in a DL MU PPDU
  bool m_forceDlOfdma;                                  //!< return DL_OFDMA even if no DL MU PPDU was built
  bool m_enableUlOfdma;                                 //!< enable the scheduler to also return UL_OFDMA
  bool m_enableBsrp;                                    //!< send a BSRP before an UL MU transmission
  bool m_useCentral26TonesRus;                          //!< whether to allocate central 26-tone RUs
  uint32_t m_ulPsduSize;                                //!< the size in byte of the solicited PSDU
  std::map<AcIndex, std::list<MasterInfo>> m_staList;   //!< Per-AC list of stations (next to serve first)
  std::list<CandInfo> m_simpleCandidates;
  std::list<CandidateInfo> m_candidates;                //!< Candidate stations for MU TX
  Time m_maxCredits;                                    //!< Max amount of credits a station can have
  Ptr<WifiMacQueueItem> m_trigger;                      //!< Trigger Frame to send
  Time m_tbPpduDuration;                                //!< Duration of the solicited TB PPDUs
  WifiTxParameters m_txParams;                          //!< TX parameters
  TriggerFrameType m_ulTriggerType;                     //!< Trigger Frame type for UL MU

  // PF Scheduler
  struct match
  {
    int a;
    int b;
    match(int x, int y)
          : a(x), b(y){}
    
  };
  struct map
  {
    int a;
    HeRu::RuType b;
    map(int x,HeRu::RuType  y)
          : a(x), b(y){}
  };
  struct alloc
  {
    bool a;
    std::vector<match>b;
  };

  std::list<std::pair<uint32_t, uint16_t>> dataStaPair;
  std::vector<match> minRuAlloc;
  std::vector<int> randomMCS;
  std::vector<map> mappedRuAllocated;
  std::list<std::pair<Mac48Address, DlPerStaInfo>> staAllocated;
  std::map<uint16_t,double> dataTransmitted;
  std::map<uint16_t,double> totalTime;
  double MAX_COST=11111.0;
  std::vector<std::vector<double> > costMatrix;
    std::vector<int> assignment;
  std::map<uint16_t,double>mapw;

  int bestMCS=-1;
  double timeReq(int dataSize,int rusize,int mcs_QAM);
  double timeReq1(int dataSize,int rusize,int mcs);
  double getDataRate(int mcs, int ru);
  void MUTAX();
  void ProportionalFair(std::vector<int> currRUset,int currMCS);
};

} //namespace ns3

#endif /* RR_MULTI_USER_SCHEDULER_H */
