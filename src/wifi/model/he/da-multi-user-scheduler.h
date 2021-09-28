/* The contents of this file are part of an 
 * unpublished work, and must NOT be shared with
 * anyone without the permission of Dr. Mukulika Maity,
 * Indraprastha Institute of Information Technology.
 *
 * Author: Harshal Dev <harshal20086@iiitd.ac.in>
 * Adapted from RrMultiUserScheduler
 */

#ifndef DA_MULTI_USER_SCHEDULER_H
#define DA_MULTI_USER_SCHEDULER_H

#include "multi-user-scheduler.h"
#include <list>
#include <iostream>
#include <vector>
#include <string>
#include "ns3/application-container.h"
#include "ns3/on-demand-application.h"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/maximum_weighted_matching.hpp>
//#include <ilcplex/ilocplex.h>

using namespace boost;

namespace ns3 {

/**
 * \ingroup wifi
 *
 * DaMultiUserScheduler is a OFDMA scheduler that indicates to perform a DL OFDMA
 * transmission if the AP has frames to transmit to at least one station.
 * 
 * DaMultiUserScheduler uses a Maximum Weighted Matching solver to create
 * a schedule for a set number of rounds that gives priority to stations
 * with a higher penalty for drops. The schedule generated for the set number of rounds
 * is then used for those many rounds to schedule packets. Compared to other
 * schedulers, the DA scheduler always leads to the least number of drops
 * in high density scenarios for stations running critical applications.
 *
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
   * \brief Inform the scheduler that Deadline Constrained Traffic has began
   *
   */
  void NotifyDeadlineConstrainedTrafficStarted(void);

  /**
   * \brief Receive information related to packet generation rate from
   * the simulation, this is used to generate the packet schedule for
   * a set of rounds
   */
  void SetStaPacketInfo(std::map <uint32_t, std::vector<uint32_t>>);
  
  /**
   * \brief Pointer to the OnDemandApps container on the
   * created on the simulation side.
   */
  void PassReferenceToOnDemandApps(ApplicationContainer);

  /**
   * \brief Increment the round counter and generate
   * packets for the next round, this method is vital
   * to keep the simulation running, if this method is
   * not called after a particular round, the simulation
   * stops
   */
  void StartNextRound(bool beginning = false);

protected:
  void DoDispose (void) override;
  void DoInitialize (void) override;

private:

  uint32_t GetCurrRound(void);

  uint32_t LCM(int[], int);
  
  /**
   * \brief For how many rounds should a packet schedule be generated?
   */
  uint32_t GetRoundsPerSchedule(void);
  
  /**
   * \brief How many packets are there in a generated schedule?
   */
  uint32_t GetPacketsPerSchedule(void);

  /**
   * \brief Generate the schedule detailing the
   * arrival and deadline round for each packet
   * in the t rounds
   */
  void GeneratePacketScheduleForSetRounds(void);
  
  /**
   * \brief Get the RU Type to be used in each round
   */
  HeRu::RuType GetRuTypePerRound(void);

  /**
   * \brief Get the numbers of RUs to be used
   * in each round, calculated from the total
   * numbers of packets to be scheduled in the
   * t rounds
   */
  uint32_t GetRusPerRound(HeRu::RuType);
  
  /**
   * \brief Used by the ILP solver to index into
   * the splits array
   * 
   * \return The index to be used to index into
   * the splits array
   */
  uint32_t GetRuTypeIndex(HeRu::RuType);
  
  /**
   * \brief The Maximum Weighted Matching algorithm maps packet
   * index to ru index, this utility function is used to get the
   * round index from the ru index.
   */
  uint32_t GetRoundFromVertexIndex(uint32_t vj, uint32_t rus);

  void MaximumWeightedMatching(void);

  void MinimumCostFlow(void);
  
  void ILPSolver(void);

  //void populatebyrow(IloModel, IloNumVarArray, IloRangeArray);
  
  /**
   * \brief Generates the Map for mapping incoming MPDU to
   * a specific packet index, see definition for a detailed
   * example
   */
  void GenerateMpduToCurrPacketMap(void);

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
  uint32_t m_currRound;                                   //!< The current round
  bool m_havePacketsArrived;
  double m_lastRoundTimestamp;
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
  ApplicationContainer m_OnDemandApps;
  Time m_maxCredits;                                    //!< Max amount of credits a station can have
  Ptr<WifiMacQueueItem> m_trigger;                      //!< Trigger Frame to send
  Time m_tbPpduDuration;                                //!< Duration of the solicited TB PPDUs
  WifiTxParameters m_txParams;                          //!< TX parameters
  TriggerFrameType m_ulTriggerType;                     //!< Trigger Frame type for UL MU
};

} //namespace ns3

#endif /* RR_MULTI_USER_SCHEDULER_H */
