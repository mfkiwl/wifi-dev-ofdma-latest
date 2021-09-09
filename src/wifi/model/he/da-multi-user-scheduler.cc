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
#include "da-multi-user-scheduler.h"
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

using namespace boost;

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("DaMultiUserScheduler");

NS_OBJECT_ENSURE_REGISTERED (DaMultiUserScheduler);

TypeId
DaMultiUserScheduler::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::DaMultiUserScheduler")
    .SetParent<MultiUserScheduler> ()
    .SetGroupName ("Wifi")
    .AddConstructor<DaMultiUserScheduler> ()
    .AddAttribute ("NStations",
                   "The maximum number of stations that can be granted an RU in a DL MU OFDMA transmission",
                   UintegerValue (4),
                   MakeUintegerAccessor (&DaMultiUserScheduler::m_nStations),
                   MakeUintegerChecker<uint16_t> (1, 74))
    .AddAttribute ("EnableTxopSharing",
                   "If enabled, allow A-MPDUs of different TIDs in a DL MU PPDU.",
                   BooleanValue (true),
                   MakeBooleanAccessor (&DaMultiUserScheduler::m_enableTxopSharing),
                   MakeBooleanChecker ())
    .AddAttribute ("ForceDlOfdma",
                   "If enabled, return DL_MU_TX even if no DL MU PPDU could be built.",
                   BooleanValue (false),
                   MakeBooleanAccessor (&DaMultiUserScheduler::m_forceDlOfdma),
                   MakeBooleanChecker ())
    .AddAttribute ("EnableUlOfdma",
                   "If enabled, return UL_MU_TX if DL_MU_TX was returned the previous time.",
                   BooleanValue (true),
                   MakeBooleanAccessor (&DaMultiUserScheduler::m_enableUlOfdma),
                   MakeBooleanChecker ())
    .AddAttribute ("EnableBsrp",
                   "If enabled, send a BSRP Trigger Frame before an UL MU transmission.",
                   BooleanValue (true),
                   MakeBooleanAccessor (&DaMultiUserScheduler::m_enableBsrp),
                   MakeBooleanChecker ())
    .AddAttribute ("UlPsduSize",
                   "The default size in bytes of the solicited PSDU (to be sent in a TB PPDU)",
                   UintegerValue (500),
                   MakeUintegerAccessor (&DaMultiUserScheduler::m_ulPsduSize),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("UseCentral26TonesRus",
                   "If enabled, central 26-tone RUs are allocated, too, when the "
                   "selected RU type is at least 52 tones.",
                   BooleanValue (false),
                   MakeBooleanAccessor (&DaMultiUserScheduler::m_useCentral26TonesRus),
                   MakeBooleanChecker ())
    .AddAttribute ("MaxCredits",
                   "Maximum amount of credits a station can have. When transmitting a DL MU PPDU, "
                   "the amount of credits received by each station equals the TX duration (in "
                   "microseconds) divided by the total number of stations. Stations that are the "
                   "recipient of the DL MU PPDU have to pay a number of credits equal to the TX "
                   "duration (in microseconds) times the allocated bandwidth share",
                   TimeValue (Seconds (1)),
                   MakeTimeAccessor (&DaMultiUserScheduler::m_maxCredits),
                   MakeTimeChecker ())
  ;
  return tid;
}

DaMultiUserScheduler::DaMultiUserScheduler ()
  : m_hasDeadlineConstrainedTrafficStarted(false), m_currRound(0), isRoundOffsetSet(false), m_lastRoundTimestamp(0), m_arrivingUsersCount(0), m_roundsPerSchedule(0), m_packetsPerSchedule(0), m_ulTriggerType (TriggerFrameType::BASIC_TRIGGER)
{
  NS_LOG_FUNCTION (this);
}

DaMultiUserScheduler::~DaMultiUserScheduler ()
{
  NS_LOG_FUNCTION_NOARGS ();
}

void
DaMultiUserScheduler::DoInitialize (void)
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT (m_apMac != nullptr);
  m_apMac->TraceConnectWithoutContext ("AssociatedSta",
                                       MakeCallback (&DaMultiUserScheduler::NotifyStationAssociated, this));
  m_apMac->TraceConnectWithoutContext ("DeAssociatedSta",
                                       MakeCallback (&DaMultiUserScheduler::NotifyStationDeassociated, this));
  for (const auto& ac : wifiAcList)
    {
      m_staList.insert ({ac.first, {}});
    }
  MultiUserScheduler::DoInitialize ();
}

void
DaMultiUserScheduler::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
  m_staList.clear ();
  m_candidates.clear ();
  m_trigger = nullptr;
  m_txParams.Clear ();
  m_apMac->TraceDisconnectWithoutContext ("AssociatedSta",
                                          MakeCallback (&DaMultiUserScheduler::NotifyStationAssociated, this));
  m_apMac->TraceDisconnectWithoutContext ("DeAssociatedSta",
                                          MakeCallback (&DaMultiUserScheduler::NotifyStationDeassociated, this));
  MultiUserScheduler::DoDispose ();
}

bool
DaMultiUserScheduler::CheckDeadlineConstrainedTrafficStarted(void)
{
  return m_hasDeadlineConstrainedTrafficStarted;
}

void
DaMultiUserScheduler::NotifyDeadlineConstrainedTrafficStarted(void)
{
  m_hasDeadlineConstrainedTrafficStarted = true;
}

void
DaMultiUserScheduler::SetRoundTimeOffset(double offset) {
  roundTimeOffset = offset;
}

void 
DaMultiUserScheduler::SetTimeQuantaForRound(double quanta) {
  timeQuanta = quanta;
}

uint32_t
DaMultiUserScheduler::GetRoundFromTimestamp(double timestamp) {
  return std::floor((timestamp - roundTimeOffset) / timeQuanta );
}

uint32_t
DaMultiUserScheduler::GetCurrRound(void) {
  return m_currRound;
}

void
DaMultiUserScheduler::SetStaPacketInfo(std::map <uint32_t, std::vector<uint32_t>> packetInfo) {
  m_staPacketInfo = packetInfo;
}

void
DaMultiUserScheduler::PassReferenceToOnDemandApps(ApplicationContainer apps) {
  m_OnDemandApps = apps;
}

uint32_t
DaMultiUserScheduler::LCM(int arr[], int n)
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
DaMultiUserScheduler::GetRoundsPerSchedule(void) {
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
DaMultiUserScheduler::GetPacketsPerSchedule(void) {
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
DaMultiUserScheduler::GeneratePacketScheduleForSetRounds(void) 
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

    std::cout << "Packet schedule generated!" << std::endl;
}

HeRu::RuType
DaMultiUserScheduler::GetRuTypePerRound(void) {

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
DaMultiUserScheduler::GetRusPerRound(HeRu::RuType ruType) {

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
DaMultiUserScheduler::GetRuTypeIndex(HeRu::RuType ruType) {

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

uint32_t
DaMultiUserScheduler::GetRoundFromVertexIndex(uint32_t vj, uint32_t rus) {
  //return ( ( vj - GetPacketsPerSchedule() ) / rus ) + GetRoundFromTimestamp(currTimeUs);
  
  return ( ( vj - GetPacketsPerSchedule() ) / rus ) + GetCurrRound();
}

void
DaMultiUserScheduler::MaximumWeightedMatching(void) {

  uint32_t rounds = GetRoundsPerSchedule();
  uint32_t packets = GetPacketsPerSchedule();
  //uint32_t packetSchedule[packets][2] = { {0, 2}, {1, 3}, {2, 4}, {3, 5}, {4, 6}, {5, 7}, {0, 4}, {2, 6}, {4, 8}, {0, 3}, {3, 6} };
  
  //float penalties[] = { 2.0, 950.0, 900.0, 2.0, 800.0, 750.0, 1000.0, 2.0, 2.0, 2.0, 850.0 };
  HeRu::RuType ruType = GetRuTypePerRound();
  uint32_t rus = GetRusPerRound(ruType);
  const int n_vertices = rus * rounds + packets;
  graph_traits< my_graph >::vertex_iterator vi, vi_end;

  //uint32_t roundOffset = GetRoundFromTimestamp(currTimeUs);
  uint32_t roundOffset = GetCurrRound();
  std::cout << "MWM invoked in round " << roundOffset << "\n";

  my_graph g(n_vertices);

  for ( uint32_t i = 0; i < packets; i++ ) {
    //for ( uint32_t j = GetRoundFromTimestamp(currTimeUs); j < GetRoundFromTimestamp(currTimeUs) + rounds; j++ ) {
    for ( uint32_t j = GetCurrRound(); j < GetCurrRound() + rounds; j++ ) {


      if ( j >= m_packetSchedule[i][0] && j <= m_packetSchedule[i][1] ) {

        // Not j - round leads to munmap_chunk(): invalid pointer error        
        //uint32_t firstIndex = packets + ((j - GetRoundFromTimestamp(currTimeUs))*rus);
        uint32_t firstIndex = packets + ((j - GetCurrRound())*rus);
        for ( uint32_t k = firstIndex; k < firstIndex + rus; k++ ) {

            add_edge(i, k, EdgeProperty(m_packetSchedule[i][2]), g);
        }
      }
    }
  }

  std::vector< graph_traits< my_graph >::vertex_descriptor > mate1(n_vertices);
  maximum_weighted_matching(g, &mate1[0]);

  std::cout << "Found a weighted matching:" << std::endl;
  std::cout << "Matching size is " << matching_size(g, &mate1[0])
            << ", total weight is " << matching_weight_sum(g, &mate1[0])
            << std::endl;
  std::cout << std::endl;

  std::cout << "The matching is:" << std::endl;
  for (boost::tie(vi, vi_end) = vertices(g); vi != vi_end; ++vi) {
    if (mate1[*vi] != graph_traits< my_graph >::null_vertex() && *vi < mate1[*vi]) {
      std::cout << "{" << *vi << ", " << mate1[*vi] << ", " << GetRoundFromVertexIndex(mate1[*vi], rus) << "}" << std::endl;

      // Note that we are omitting information of the particular RU that is mapped in a particular round, since that isn't needed
      // for the scheduler, it just assigns RUs sequentially to all the packets scheduled in a particular round
      m_packetToRoundMap.insert(std::make_pair (*vi, GetRoundFromVertexIndex(mate1[*vi], rus)));
    }
  }

  std::cout << std::endl;
}

void
DaMultiUserScheduler::ILPSolver(void) {

  uint32_t rounds = GetRoundsPerSchedule();
  uint32_t packets = GetPacketsPerSchedule();
  uint32_t ruType = GetRuTypeIndex(GetRuTypePerRound()); // Used to index the splits array in the ILP \todo
  uint32_t totalTones = 242;
  if ( m_apMac->GetWifiPhy()->GetChannelWidth() == 40 )
    totalTones = 484; // Note that the ILP currently only supports 40 Mhz, if you enter 20 Mhz it will malfunction

  // Offset to add and subtract when inputting to the ILP
  // The ILP assumes that a map is being generated at the
  // start of round 0
  //uint32_t roundOffset = GetRoundFromTimestamp(currTimeUs);
  uint32_t roundOffset = GetCurrRound();
  std::cout << "ILP invoked in round " << roundOffset << "\n";

  std::ostringstream oss;
  oss << "java -Djava.library.path=/opt/ibm/ILOG/CPLEX_Studio201/cplex/bin/x86-64_linux -classpath /opt/ibm/ILOG/CPLEX_Studio201/cplex/lib/cplex.jar: DeadlineAwareILP " << rounds << " " << packets << " " << ruType << " " << totalTones << " ";
  for ( uint32_t i = 0; i < packets; i++ ) {
      if ( i < packets - 1 )
          oss << (m_packetSchedule[i][0] - roundOffset) << " " << (m_packetSchedule[i][1] - roundOffset) << " " << m_packetSchedule[i][2] << " ";
      else
          oss << (m_packetSchedule[i][0] - roundOffset) << " " << (m_packetSchedule[i][1] - roundOffset) << " " << m_packetSchedule[i][2];
  }

  std::string txt = oss.str();
  uint32_t length = txt.length();
    
  char cmd[length];
  std::copy(std::begin(txt), std::end(txt), cmd);
  cmd[length] = 0;

  std::cout << cmd;
  std::cout << std::endl;

  system(cmd);

  // Extract the values from the output file written by the ILP
  std::ifstream file;
  file.open("ilp.output", std::ios::in);

  std::string line;
  while(std::getline(file, line))
  {
      std::stringstream lineStream(line);
      uint32_t pos = lineStream.str().find(",");
        
      uint32_t packet = atoi(lineStream.str().substr(0, pos).c_str());
      uint32_t round = atoi(lineStream.str().substr(pos + 1).c_str()) + roundOffset;

      // This is all we need
      m_packetToRoundMap.insert(std::make_pair(packet, round));

      std::cout << "{" << packet << ", " << round << "}" << std::endl;
  }
}

// void
// DaMultiUserScheduler::ILPSolver(void) {
//   IloEnv   env;
//    try {
//       IloModel model(env);

//       IloNumVarArray var(env);
//       IloRangeArray con(env);

//       populatebyrow (model, var, con);

//       IloCplex cplex(model);
//       cplex.exportModel("lpex1.lp");

//       // Optimize the problem and obtain solution.
//       if ( !cplex.solve() ) {
//          env.error() << "Failed to optimize LP" << std::endl;
//          throw(-1);
//       }

//       IloNumArray vals(env);
//       env.out() << "Solution status = " << cplex.getStatus() << std::endl;
//       env.out() << "Solution value  = " << cplex.getObjValue() << std::endl;
//       cplex.getValues(vals, var);
//       env.out() << "Values        = " << vals << std::endl;
//       cplex.getSlacks(vals, con);
//       env.out() << "Slacks        = " << vals << std::endl;
//       cplex.getDuals(vals, con);
//       env.out() << "Duals         = " << vals << std::endl;
//       cplex.getReducedCosts(vals, var);
//       env.out() << "Reduced Costs = " << vals << std::endl;
//    }
//    catch (IloException& e) {
//       std::cout << "Concert exception caught: " << e << std::endl;
//    }
//    catch (...) {
//       std::cout << "Unknown exception caught" << std::endl;
//    }

//    env.end();
// }

// void
// DaMultiUserScheduler::populatebyrow(IloModel model, IloNumVarArray x, IloRangeArray c)
// {
//    IloEnv env = model.getEnv();

//    x.add(IloNumVar(env, 0.0, 40.0));
//    x.add(IloNumVar(env));
//    x.add(IloNumVar(env));

//    model.add(IloMaximize(env, x[0] + 2 * x[1] + 3 * x[2]));

//    c.add( - x[0] +     x[1] + x[2] <= 20);
//    c.add(   x[0] - 3 * x[1] + x[2] <= 30);

//    x[0].setName("x1");
//    x[1].setName("x2");
//    x[2].setName("x3");

//    c[0].setName("c1");
//    c[1].setName("c2");
//    model.add(c);
// }

void
DaMultiUserScheduler::GenerateMpduToCurrPacketMap(void)
{

  // I use this map to keep a pointer to the packet index (in the bipartite graph) 
  // that is supposed to be mapped to the next mpdu received by the user
  // ( * * ) ( * * ) ( * * ) ( * * ) ( * * ) ( * * ) Round Indices
  //    |       |       |       |       ^-------|
  //    *       *       *       *       *       *    Packet Indices
  //          USER 1        ||        USER 2
  //
  //            |____MSDU FOR USER 2____|
  //
  // Which Packet Index should I try to map this MSDU to?
  // Suppose that an earlier MSDU for user 2 has already 
  // been mapped to packet index 3 for user 2, so I will try
  // to map this MSDU to packet index 4, however, if packet index
  // 4 has not been mapped to any round, then I will be dropping
  // this MSDU and incrementing the pointer, so that the next MSDU
  // can attempt mapping to packet index 5.

  m_mpduToCurrPacketMap.clear();

  uint32_t nextIndex = 0;
  for ( uint32_t p = 0; p < m_nStations; p++ ) {
    m_mpduToCurrPacketMap.insert(std::make_pair(p, nextIndex));

    auto it = m_staPacketInfo.find(p);
    NS_ASSERT (it != m_staPacketInfo.end());

    uint32_t timePeriod = it->second[0];
                              // Packets for this user
    nextIndex = nextIndex + ( GetRoundsPerSchedule() / timePeriod );
  }
}

void
DaMultiUserScheduler::StartNextRound(bool beginning) {
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
      
      m_arrivingUsers.push_back(i);
      m_arrivingUsersCount++;
      DynamicCast<OnDemandApplication>(m_OnDemandApps.Get(i))->SendPacket();
    }
  }
  else {

    for ( uint32_t i = 0; i < m_nStations; i++ ) {
      
      auto it = m_staPacketInfo.find(i);
      NS_ASSERT ( it != m_staPacketInfo.end() );

      if ( ( m_currRound % it->second[0] ) == 0 ) {
        m_arrivingUsers.push_back(i);
        m_arrivingUsersCount++;
        DynamicCast<OnDemandApplication>(m_OnDemandApps.Get(i))->SendPacket();
      }
    }
  }

  Simulator::Schedule(MilliSeconds(1), &DaMultiUserScheduler::StartNextRound, this, false);
}

MultiUserScheduler::TxFormat
DaMultiUserScheduler::SelectTxFormat (void)
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
DaMultiUserScheduler::TrySendingBsrpTf (void)
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
DaMultiUserScheduler::TrySendingBasicTf (void)
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
DaMultiUserScheduler::NotifyStationAssociated (uint16_t aid, Mac48Address address)
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
DaMultiUserScheduler::NotifyStationDeassociated (uint16_t aid, Mac48Address address)
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
DaMultiUserScheduler::TrySendingDlMuPpdu (void)
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
  HeRu::RuType ruType = HeRu::GetEqualSizedRusForStations (m_apMac->GetWifiPhy ()->GetChannelWidth (), count,
                                                           nCentral26TonesRus);

  count = m_nStations; //GetEqualSizedRusForStations changes the value of count since it is passed by reference                                                           
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

  // I use the current time to track the rounds
  currTimeUs = Simulator::Now().ToDouble(Time::US);
  
  if ( m_hasDeadlineConstrainedTrafficStarted && !m_candidates.empty() && !isRoundOffsetSet ) {
    SetRoundTimeOffset(currTimeUs);
    isRoundOffsetSet = true; // Although the deadline constrained traffic had started arriving, we know that now the first packet has arrived
  }
 
                                                                                // This is important, otherwise if the packet to be transmitted in 
                                                                                // this round has been transmitted, and the next incoming packet is 
                                                                                // only scheduled to be dropped, then m_packetToRoundMap is empty and 
                                                                                // as soon as the next packet arrives, m_candidates is not empty, 
                                                                                // forcing the generated of a new packetSchedule in this round itself.
  //if ( m_hasDeadlineConstrainedTrafficStarted && m_packetToRoundMap.empty() && ( GetRoundFromTimestamp(currTimeUs) % GetRoundsPerSchedule() == 0 ) && !m_candidates.empty() ) {
  if ( m_hasDeadlineConstrainedTrafficStarted && m_packetToRoundMap.empty() && ( GetCurrRound() % GetRoundsPerSchedule() == 0 ) && !m_candidates.empty() ) {

    GeneratePacketScheduleForSetRounds(); // m_packetSchedule
    //MaximumWeightedMatching(); // m_packetToRoundMap
    ILPSolver();
    GenerateMpduToCurrPacketMap(); // m_mpduToCurrPacketMap

    for ( std::vector<uint32_t> x : m_packetSchedule ) {
      std::cout << "( " << x[0] << ", " << x[1] << ", " << x[2] << ", " << x[3] << ")\n"; 
    }
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
DaMultiUserScheduler::ComputeDlMuInfo (void)
{
  NS_LOG_FUNCTION (this);

  // The second condition is important, otherwise for scenarios where 
  // deadline constrained traffic is started, but the m_packetToRoundMap
  // has not been generated because one of the conditions for generation
  // of the map was not satisfied (particularly m_candidates is empty)
  // multi-user-scheduler.cc calls ComputeDlMuInfo() and that leads to
  // an attempt to index a map which has not yet been generated (m_packetToRoundMap)
  // However we can be sure that if isRoundOffSet is true, the map has been
  // generated because a candidate's packet arrived in the queue
  if ( m_hasDeadlineConstrainedTrafficStarted && isRoundOffsetSet ) {

    // Search the packetToRoundMap to find the packets scheduled
    // in this round, if you find one, find the AID of the sta
    // corresponding to the packet, then assign appropriate RU type to those
    // STAs, if no packet is scheduled in the current round,
    // return an empty DlMuInfo()
    //uint32_t currRound = GetRoundFromTimestamp(currTimeUs);
    uint32_t currRound = GetCurrRound();
    uint32_t unscheduledThisRound = 0;
    m_roundCandidates.clear();

    for (const auto& candidate : m_candidates) {

      uint16_t aid = candidate.first->aid;
      aid--;
      uint16_t packetsThisRound = 0;

      uint32_t *p = &(m_mpduToCurrPacketMap.at(aid));
      // OLD: We will iterate through the generated packet schedule to see if any packets
      // belong to this candidate (with an mpdu addressed to it in the queue)
      
      // NEW: We will look at the packet schedule to verify that the index of the packet
      // pointed at in the Weighted Bipartite Graph is actually a packet that
      // belongs to this user

      // One thing to realise here is that in any given round, a user has at most
      // one packet scheduled in that round, if the user is in m_candidates, it
      // implies that the user has a packet that it expects to schedule in this
      // round or some subsequent round (before it's next packet arrives of course)
      // However, it is possible that MaxiumWeightedMatching has decided to not
      // schedule this packet in any round, so we must drop it to avoid queing 
      // of user packets. 
      
      //for ( uint32_t p = 0; p < GetPacketsPerSchedule(); p++ ) { if ( m_packetSchedule[*p][3] == aid ) { }

      // If this assertion fails, it's because the new packet schedule has not been generated yet
      // or because a packet is not following it's expected arrival time periods
      // This implies a mismatch in the packet generation rate and the schedule generation time
      // period, verify your rates.
      //std::cout << "p = " << *p << "m_packetSchedule[*p][3] = " << m_packetSchedule[*p][3] << ", aid = " << aid << std::endl;

      NS_ASSERT( m_packetSchedule[*p][3] == aid );    
      //if ( m_packetSchedule[*p][3] == aid ) {

      for ( uint32_t m = 0; m < m_arrivingUsers.size(); m++ ) {
        if ( m_arrivingUsers[m] == aid ) {
          m_arrivingUsers[m] = -1;
          m_arrivingUsersCount--; // A decision is being made for a packet that was expected to arrive
        }
      }

      auto it = m_packetToRoundMap.find(*p);
      if ( it != m_packetToRoundMap.end() ) { // This packet has been scheduled in some round

        if ( it->second == currRound ) {// This packet has been scheduled in the current round
            
          packetsThisRound++;
          m_packetToRoundMap.erase(*p);
            
          (*p)++; // The next mpdu should be mapped to the next packet index
        }
        else 
          std::cout << "Buffered STA_" << (aid + 1) << " packet in round " << currRound << std::endl;
      }
      else { // This packet has not been scheduled in any round, drop it

        Ptr<const WifiMacQueueItem> mpdu = candidate.second;
        uint8_t tid = mpdu->GetHeader().GetQosTid();
                
        Ptr<WifiMacQueue> queue = m_apMac->GetQosTxop(QosUtilsMapTidToAc (tid))->GetWifiMacQueue();
                
        WifiMacQueueItem::QueueIteratorPair queueIt = mpdu->GetQueueIteratorPairs ().front ();
        NS_ASSERT (queueIt.queue != nullptr);
        queue->Dequeue(queueIt.it);

        //std::cout << "Dropped STA_" << (aid + 1) << " packet in round " << GetRoundFromTimestamp(currTimeUs) << std::endl;
        std::cout << "Dropped STA_" << (aid + 1) << " packet in round " << GetCurrRound() << std::endl;
        (*p)++; // The next mpdu should be mapped to the next packet index
      }
      //}

      if ( packetsThisRound == 0 )
        unscheduledThisRound++;
      else {
        m_roundCandidates.push_back(candidate);  
      }
    }

    if ( unscheduledThisRound == m_candidates.size() ) // None of the candidates have packets scheduled in this specific round
      m_candidates.clear(); 
    else { // Some of the candidates have packets scheduled in this round

      m_candidates.clear(); // We will remove the candidates who do not have a packet scheduled this round
      for (const auto& roundCandidate : m_roundCandidates)
          m_candidates.push_front(roundCandidate);    
    }  
  }

  if (m_candidates.empty ())
    {
      // No users were there with packets scheduled in this round
      // and we are not expecting any more user packets to
      // arrive in this round, so the round has ended it means
      // This can only happen if the original m_candidates
      // did not contain any users, which is not really possible
      // with the OnDemandApplication, since the packets are generated
      // instantaneously

      // if ( m_hasDeadlineConstrainedTrafficStarted && isRoundOffsetSet && ( m_arrivingUsersCount == 0 ) )
      //   StartNextRound();

      return DlMuInfo ();
    }  

  uint16_t bw = m_apMac->GetWifiPhy ()->GetChannelWidth ();

  // compute how many stations can be granted an RU and the RU size
  std::size_t nRusAssigned;
  if ( !m_hasDeadlineConstrainedTrafficStarted || !isRoundOffsetSet )
    nRusAssigned = m_txParams.GetPsduInfoMap ().size ();
  else  
    nRusAssigned = m_candidates.size();

  std::size_t nCentral26TonesRus;
  HeRu::RuType ruType;
  if ( !m_hasDeadlineConstrainedTrafficStarted || !isRoundOffsetSet )
    ruType = HeRu::GetEqualSizedRusForStations (bw, nRusAssigned, nCentral26TonesRus);
  else
    ruType = GetRuTypePerRound();

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

  std::vector<HeRu::RuSpec> ruSet = HeRu::GetRusOfType(m_apMac->GetWifiPhy()->GetChannelWidth(), ruType);  
  auto ruSetIt = ruSet.begin();

  for (uint16_t i = 0; i < nRusAssigned + nCentral26TonesRus; i++)
    {
      NS_ASSERT (candidateIt != m_candidates.end ());

      uint16_t staId = candidateIt->first->aid;
      // AssignRuIndices will be called below to set RuSpec
      dlMuInfo.txParams.m_txVector.SetHeMuUserInfo (staId,
                                                    {{(i < nRusAssigned ? ruType : HeRu::RU_26_TONE), 1, false},
                                                      m_txParams.m_txVector.GetMode (staId),
                                                      m_txParams.m_txVector.GetNss (staId)});
                                                   
      
      if ( m_hasDeadlineConstrainedTrafficStarted && isRoundOffsetSet ) {
        dlMuInfo.txParams.m_txVector.SetRu(*ruSetIt, staId);
        ruSetIt++; // It is important to increment this, because an RuSpec represents a distinct RU from the given bandwidth
      }

      candidateIt++;
    }

  // remove candidates that will not be served (redundant line)
  m_candidates.erase (candidateIt, m_candidates.end ());

  if ( !m_hasDeadlineConstrainedTrafficStarted || !isRoundOffsetSet ) // Otherwise we set RUs when constructing the DlMuInfo map in the above loop
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
      
      if ( mpdu->GetHeader().IsData() && m_hasDeadlineConstrainedTrafficStarted && isRoundOffsetSet ) {
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

  if ( !m_hasDeadlineConstrainedTrafficStarted || m_packetToRoundMap.empty() ) {
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
  }

  // This is for the scenario where after the Tx is completed, some packets are still
  // in the queue waiting for scheduling in the next round, so we should now go to the next
  // round to allow their scheduling since all the expected user packets have arrived in
  // this round
  // if ( m_hasDeadlineConstrainedTrafficStarted && isRoundOffsetSet && ( m_arrivingUsersCount == 0 ) )
  //   StartNextRound();

  return dlMuInfo;
}

void
DaMultiUserScheduler::AssignRuIndices (WifiTxVector& txVector)
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
DaMultiUserScheduler::ComputeUlMuInfo (void)
{
  return UlMuInfo {m_trigger, m_tbPpduDuration, std::move (m_txParams)};
}

} //namespace ns3
