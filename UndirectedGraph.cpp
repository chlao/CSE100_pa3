#include "UndirectedGraph.hpp"
#include "Edge.hpp"
#include "Vertex.hpp"

#include <iostream>
#include <queue>
#include <limits>
/************************************************************
 * Author: Christine Lao and Shaina Wan
 * This file holds the constructor and the destructor of the 
 * undirected graph that we will be building and manipulating.
 * Also holds the minimum spanning tree, which will be used
 * to find the most least expensive and time consuming paths
 * in the undirected graph. Also holds properties of the 
 * Undirected Graph, such as determining the totalCost and 
 * total distance throughout the graph. 
 * *********************************************************/
/**
 * Constructs an empty UndirectedGraph with no vertices and
 * no edges.
 */
UndirectedGraph::UndirectedGraph(){}

/**
 * Destructs an UndirectedGraph.
 */
UndirectedGraph::~UndirectedGraph()
{
  for (auto v : vertices) //travels through the vertices
    delete v.second; 
}

/**
 * Inserts an edge into the graph. If an edge already exists between
 * the vertices, updates the cost and length of the edge to match the
 * passed parameters.
 *
 * If either of the named vertices does not exist, it is created.
 */
void UndirectedGraph::addEdge(const std::string &from, const std::string &to,
            unsigned int cost, unsigned int length)
{
  // Find the from vertex
  std::unordered_map<std::string, Vertex*>::const_iterator 
			iteratorFrom = vertices.find(from);
  // Find the to vertex
  std::unordered_map<std::string, Vertex*>::const_iterator 
			iteratorTo = vertices.find(to); 

  // Check if the from vertex already exists 
  if (iteratorFrom == vertices.end()) // From vertex doesn't exist
  {
    // Create a new vertex
    Vertex* vertexFrom = new Vertex(from);
    vertices.insert(std::make_pair(from, vertexFrom)); 
    
    // Check if the to vertex exists 
    if (iteratorTo == vertices.end()) // To Vertex doesn't exist
    {
      // Create new to vertex
      Vertex* vertexTo = new Vertex(to); 
      vertices.insert(std::make_pair(to, vertexTo)); 

      // Create a pair of edges (undirected)
      vertexTo->addEdge(vertexFrom, cost, length);
      vertexFrom->addEdge(vertexTo, cost, length); 
    }
    else // To Vertex exists
    {
      vertexFrom->addEdge(iteratorTo->second, cost, length);
      iteratorTo->second->addEdge(vertexFrom, cost, length); 
    }
  }
  else // From vertex does exist 
  {
    // Check if to vertex exists
    if (iteratorTo == vertices.end()) // To Vertex doesn't exist 
    {
       // Create new to vertex 
       Vertex* vertexTo = new Vertex(to); 
       vertices.insert(std::make_pair(to, vertexTo)); 

       iteratorFrom->second->addEdge(vertexTo, cost, length); 
       vertexTo->addEdge(iteratorFrom->second, cost, length); 
    }
    else // To Vertex does exist 
    {
       iteratorTo->second->addEdge(iteratorFrom->second, cost, length); 
       iteratorFrom->second->addEdge(iteratorTo->second, cost, length);
    }
  }
}

/**
 * Returns the total cost of all edges in the graph.
 *
 * Since this graph is undirected, is calcualted as the cost
 * of all Edges terminating at all Vertices, divided by 2.
 */
unsigned int UndirectedGraph::totalEdgeCost() const
{
  unsigned int totalCost = 0; 

  for (auto v : vertices) // For every vertex in graph 
  {
    // Get the set of all edges connecting to the vertex
    std::unordered_map<std::string, Edge> edgeSet = v.second->getEdges(); 

    for (auto e : edgeSet) //traverse through the edges
    {
      totalCost += e.second.getCost(); 
    }
  } 

  //undirected graph
  return totalCost/2; 
}
    
/**
 * Removes all edges from the graph except those necessary to
 * form a minimum cost spanning tree of all vertices using Prim's
 * algorithm.
 *
 * The graph must be in a state where such a spanning tree
 * is possible. To call this method when a spanning tree is
 * impossible is undefined behavior.
 */
UndirectedGraph UndirectedGraph::minSpanningTree()
{
  /** Intialiation */ 
  UndirectedGraph T; // Create an empty graph

  // If tree is empty
  if (vertices.empty() == true)
    return T; 

  // Set the vistied flags of vertices in graph to false 
  for(auto v: vertices)
  {
      v.second->setVisited(false); 
  }
  
  // Pick random vertex and set its visited field to true 
  Vertex* tempVertex = vertices.begin()->second;
  tempVertex->setVisited(true); 
  
  std::priority_queue<Edge, std::vector<Edge>, std::less<Edge>> pq; 
  
  // Get the set of all edges connecting to the first vertex
  std::unordered_map<std::string, Edge> tempEdgeSet = tempVertex->getEdges(); 
  
  // Push edges of the first index into the priority queue 
  for (auto e: tempEdgeSet)
  { 
    pq.push(e.second);  
  }

  unsigned int numberVisited = 1; 
 
  /** Loop through rest of the vertices and edges */ 
  while (pq.size() != 0 || numberVisited != vertices.size())  
 {
    Edge tempEdge = pq.top(); 
    pq.pop(); 

    if(tempEdge.getTo()->wasVisited() == true)
      continue;
    else
    {
      // Set the visited flag of the to vertex to true
      tempEdge.getTo()->setVisited(true); 
      numberVisited++; 

      // Name of the vertex the edge is going to 
      std::string toName = tempEdge.getTo()->getName(); 

      // Name of the vertex the edge is going from
      std::string fromName = tempEdge.getFrom()->getName(); 

      // Add the edge into the graph
      T.addEdge(fromName, toName, tempEdge.getCost(), tempEdge.getLength()); 

      // Get the set of all edges connecting to the vertex
      std::unordered_map<std::string, Edge> edgeSet = tempEdge.getTo()->getEdges(); 

      // Iterate through to vertex's edges
      for (auto vedge : edgeSet)
      {
        if (vedge.second.getTo()->wasVisited() == false)
        {
          pq.push(vedge.second);  
        }
      } 
    }
  }
  return T; // Return the minimum spanning tree 
}
    
/**
 * Determines the combined distance from the given Vertex to all
 * other Vertices in the graph using Dijkstra's algorithm.
 *
 * Returns max possible distance if the given Vertex does not appear
 * in the graph, or if any of the Vertices in the graph are not
 * reachable from the given Vertex. Otherwise, returns the combined
 * distance.
 */
unsigned int UndirectedGraph::totalDistance(const std::string &from)
{
  // If the graph is empty
  if (vertices.empty() == true) 
    return 0; 

  // Set all the vertices' distances to infinity and their visited flags to false 
  for (auto v : vertices)
  {
    v.second->setDistance(std::numeric_limits<unsigned int>::max()); 
    v.second->setVisited(false);
  }

  //create an unordered map
  std::unordered_map<std::string, Vertex*>::const_iterator 
			iteratorFrom = vertices.find(from);

  // If the vertex does not appear in graph 
  if (iteratorFrom == vertices.end())
    return std::numeric_limits<unsigned int>::max();

  //create a priority queue 
  std::priority_queue<std::pair<Vertex*, unsigned int>, 
                    std::vector<std::pair<Vertex*, 
                   unsigned int>>, DijkstraVertexComparator> pq;

  // Start at the given node 
  Vertex* start = iteratorFrom->second;
  // Set distance of starting node to 0 
  start->setDistance(0); 
  // Enqueue starting node
  pq.push(std::make_pair(start, start->getDistance())); 
 
  // Count of all the vertices visited 
  unsigned int numberVisited = 0; 

  //Priority quene is not empty or vertices still not visited
  while (pq.size() != 0 || numberVisited != vertices.size())
  {
    std::pair<Vertex*, unsigned int> curr = pq.top(); 
    pq.pop(); 

    //Been visited
    if (curr.first->wasVisited() == true)
      continue; 
    else
    {
      // Get the set of all edges connecting to the vertex
      std::unordered_map<std::string, Edge> edgeSet = curr.first->getEdges(); 

      for (auto e : edgeSet) //Traverses through the edges 
      {
         // Check if the adjacent nodes are visited 
         if (e.second.getTo()->wasVisited() == false)
         {
           // Calculate the Dijkstra score of the new edge 
           unsigned int newDistance = curr.first->getDistance() 
						+ e.second.getLength(); 

           // If the new distance is less than the old distance, update 
           if (newDistance < e.second.getTo()->getDistance())
           {
             e.second.getTo()->setDistance(newDistance);
             
             // Enqueue vertex with new distance 
             pq.push(std::make_pair(e.second.getTo(), newDistance)); 
           }
         }
       }
       curr.first->setVisited(true); //has been visited
       numberVisited++;  
    }
  }

  int totalDist = 0; // total distance from starting node  

  // Add up the distances from each node 
  for (auto v : vertices) 
  {
    totalDist += v.second->getDistance(); 
  }

  return totalDist;  
}
    
/**
 * Determines the combined distance from all Vertices to all other
 * Vertices in the graph.
 *
 * Returns max possible distance if the graph is not connected.
 */
unsigned int UndirectedGraph::totalDistance()
{
  int totalDist = 0; // Total distance over the whole graph 

  for (auto v : vertices) //traverse through the vertices
    totalDist += this->totalDistance(v.second->getName()); 

  return totalDist;
}

/**
 * Comparison functor for use with Dijkstra's algorithm. Allows Vertices
 * to be added to a priority queue more than once, with different weights.
 *
 * Each pair represents a Vertex and its weight when it was added to the
 * queue. This guarantees that the weight used to order the Vertices in
 * the queue never changes (a required invariant of a priority queue),
 * even though the weight of the Vertex itself may change.
 *
 * Returns true if left's weight when it was inserted into the queue is
 * higher than right's weight when it was inserted into the queue.
 */
bool UndirectedGraph::DijkstraVertexComparator::
         operator()(const std::pair<Vertex*, unsigned int> &left,
                    const std::pair<Vertex*, unsigned int> &right)
{
  if (left.second > right.second)
    return true; 
  return false;  
}

