#include <cstdint>
#include <fstream>
#include <ostream>
#include <iostream>
#include <chrono>
#include "ortools/graph/min_cost_flow.h"

// Run: ./bin/maximum_weighted_matching 4 7 4 484 0 0 5 1 1 5 2 2 5 3 3 5 0 0 10 2 2 10 0 0 15

//namespace operations_research;
using namespace operations_research;
using namespace std::chrono;

int64_t globalMinCost = 0;
std::vector<std::vector<uint32_t>> packetToRoundMap;
int64_t totalEdges;

void SimpleMinCostFlowProgram(uint32_t rounds, uint32_t packets, uint32_t rus, uint32_t totalTones, int packetSchedule[][3], int64_t supplyFlow) {

  // [START constraints]
  // Instantiate a SimpleMinCostFlow solver.
  SimpleMinCostFlow min_cost_flow;
  totalEdges = 0;

  for ( uint32_t i = 0; i < packets; i++ ) {
    for ( uint32_t j = 0; j < rounds; j++ ) {


      if ( j >= packetSchedule[i][0] && j <= packetSchedule[i][1] ) {

        uint32_t firstIndex = (packets + 1) + (j * rus);
        for ( uint32_t k = firstIndex; k < firstIndex + rus; k++ ) { // define rus

            min_cost_flow.AddArcWithCapacityAndUnitCost(i + 1, k, 1, -packetSchedule[i][2]);
            totalEdges++;
        }
      }
    }
  }

  // Source edges
  for ( uint32_t i = 1; i <= packets; i++ )
    min_cost_flow.AddArcWithCapacityAndUnitCost(0, i, 1, 0);

  // Sink edges
  for ( uint32_t i = packets + 1; i <= packets + (rus*rounds); i++ )
    min_cost_flow.AddArcWithCapacityAndUnitCost(i, packets + (rus*rounds) + 1, 1, 0);

  // Add node supplies.
  for (int i = 1; i <= packets + (rus*rounds); ++i) {
    min_cost_flow.SetNodeSupply(i, 0);
  }

  min_cost_flow.SetNodeSupply(0, supplyFlow);
  min_cost_flow.SetNodeSupply(packets + (rus*rounds) + 1, -supplyFlow);
  // [END constraints]

  // [START solve]
  // Find the min cost flow.
  int solve_status = min_cost_flow.Solve();
  // [END solve]

  // [START print_solution]
  if (solve_status == MinCostFlow::OPTIMAL) {

    int64_t optimalCost = min_cost_flow.OptimalCost();
    if ( optimalCost < globalMinCost ) {

      globalMinCost = optimalCost;

      packetToRoundMap.clear(); // Make a new one with the most recent optimal value

      //LOG(INFO) << "Optimal Value: " << min_cost_flow.OptimalCost();
      //LOG(INFO) << "";
      //LOG(INFO) << " Arc   Flow / Capacity  Cost";
      for (std::size_t i = 0; i < min_cost_flow.NumArcs(); ++i) {
        
        int64_t cost = min_cost_flow.Flow(i) * min_cost_flow.UnitCost(i);
        if ( cost < 0 ) {

          // LOG(INFO) << min_cost_flow.Tail(i) << " -> " << min_cost_flow.Head(i)
          //            << "  " << min_cost_flow.Flow(i) << "  / "
          //            << min_cost_flow.Capacity(i) << "       " << cost;

          std::vector<uint32_t> map;
          uint32_t packetIndex = min_cost_flow.Tail(i) - 1;
          uint32_t ruIndex = min_cost_flow.Head(i) - packets - 1;
          uint32_t roundIndex = ( ruIndex / rus );

          // convert ruIndex to roundIndex
          map.push_back(packetIndex);
          map.push_back(roundIndex);
          packetToRoundMap.push_back(map);

          //LOG(INFO) << "Packet " << map[0] << " mapped to Round " << map[1];        
        }              
      }

      //LOG(INFO) << "";
    }
  }
  // [END print_solution]
}

int main(int argc, char *argv[]) {
  
  uint32_t splits[] = { 18, 8, 4, 2, 1 };

  uint32_t rounds = atoi(argv[1]);
  uint32_t packets = atoi(argv[2]);
  uint32_t ruType = atoi(argv[3]);
  uint32_t totalTones = atoi(argv[4]);

  int packetSchedule[packets][3];

  int base = 5;
  int index = 0;
  for ( int b = base; b < base + packets*3; b = b + 3 ) {

    packetSchedule[index][0] = atoi(argv[b + 0]);
    packetSchedule[index][1] = atoi(argv[b + 1]);
    packetSchedule[index][2] = atoi(argv[b + 2]);

    index++;
  }

  auto start = high_resolution_clock::now();

  for ( int64_t i = 1; i <= packets; i++ )
    SimpleMinCostFlowProgram(rounds, packets, splits[ruType], totalTones, packetSchedule, i);

  auto stop = high_resolution_clock::now();
  auto duration = duration_cast<milliseconds>(stop - start);

  std::cout << "Global Optimal Value = " << globalMinCost << std::endl;
  
  std::cout << "==== Performance Metrics ====" << std::endl;
  std::cout << "Execution Time: " << duration.count() << std::endl;
  std::cout << "Total Nodes = " << packets + (splits[ruType] * rounds) << std::endl;
  std::cout << "Total Edges = " << totalEdges << std::endl;

  // Write output to file
  std::ofstream file;
  file.open("mcf.output");

  for ( uint32_t i = 0; i < packetToRoundMap.size(); i++ )
    file << packetToRoundMap[i][0] << "," << packetToRoundMap[i][1] << std::endl;

  file.close();

  return EXIT_SUCCESS;
}
// [END program]
