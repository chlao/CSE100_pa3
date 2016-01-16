#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

#include "UndirectedGraph.hpp" //for the graph

/*****************************************************************
 * Author: Christine Lao and Shaina Wan
 *
 * Entry point into the netplan program.
 *
 * -Reads a file from the filesystem according to the specification 
 *  for PA3, creating an UndirectedGraph.
 * -Finds the total cost & ping time of the graph as presented in the 
 *  input file.
 * -Determines the minimum cost graph from the original graph.
 * -Finds the total cost & ping time of the minimum cost graph.
 * -Finds the change of cost & ping time from the original graph to 
 *  the minimum cost graph.
 * -Prints the results to stdout.
 *
 * Usage:
 *   ./netplan infile
 *
 ********************************************************************/
int main(int argc, char **argv) {
  //number of arguments need to be 2
  if (argc != 2) 
  {
    std::cerr << "Usage: " << argv[0] << " infile" << std::endl;
    return EXIT_FAILURE;
  }
    
  std::ifstream in(argv[1]);

  if (!in) {
    std::cerr << "Unable to open file for reading." << std::endl;
    return EXIT_FAILURE;
  }
    
  std::string networkA; // Name of the first network 
  std::string networkB; // Name of the second network 
  double cost; // Cost of the edge
  double latency; // Distance between two vertices 

  UndirectedGraph graph; // Graph consisting of networks and vertices

  while(in.good()) // While not end of file 
  {
    // Read in input file, stored in four different variables 
    in >> networkA >> networkB >> cost >> latency;
    
    // If EOF is reached 
    if(!in.good()) 
      break;  

    // Add edges and vertices 
    graph.addEdge(networkA, networkB, cost, latency);
  }
  
  /** Prints the total cost of building all possible network links
   *  This is the sum of all values of column three in the input file
   */
  unsigned int totalCostOfEdges = graph.totalEdgeCost(); 

  std::cout << totalCostOfEdges << std::endl;

  /** Prints the total cost of building the cheapest network that will
   *  permite packets to travel from any computer to any other
   */
  // Creation of the minimum spanning tree from the original graph
  UndirectedGraph minTree = graph.minSpanningTree(); 

  unsigned int totalEdgeCostOfMin = minTree.totalEdgeCost();  

  std::cout << totalEdgeCostOfMin << std::endl; 

  /** Prints the amount of money saved by building the minimum-cost
   *  network instead of the all-possible-links network 
   */
  std::cout << totalCostOfEdges - totalEdgeCostOfMin << std::endl; 

  /* Total transit time to send a packet between all pairs of computers
   * if all possible network links were built
   */
  unsigned int totalDistanceOfGraph = graph.totalDistance(); 
  std::cout << totalDistanceOfGraph << std::endl;

  /** Total transit time it would take to send a packet between all 
   *  pais of computers in the minimum-cost network
   */
  unsigned int totalDistanceOfMin = minTree.totalDistance(); 
  std::cout << totalDistanceOfMin << std::endl; 

  /** The increase in the "total time" required for packet travel 
    * in the cheap network (compared to having built all possible links)
    */
  std::cout << totalDistanceOfMin - totalDistanceOfGraph << std::endl; 

  in.close(); // Close the file  

  return EXIT_SUCCESS;
}

