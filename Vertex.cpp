#include "Vertex.hpp"

/********************************************************
 * Author: Christine Lao and Shaina Wan
 * Holds the constructor for a vertex of an undirected graph
 * and all its related functionality. Will be called by the
 * Undirected Graph to populate the graph with vertices and
 * to deal with the vertices properties. 
 * *******************************************************/

/**
 * Initialize the Vertex with the given name.
 */
Vertex::Vertex(const std::string &name)
{
  this->name = name; 
}

/**
 * Add an edge to this Vertex. If an edge already exists to the given
 * vertex, updates the cost and length of the edge to match the
 * passed parameters.
 */
bool Vertex::addEdge(Vertex *to, unsigned int cost, unsigned int length)
{
  // Create the new edge 
  Edge newEdge(this, to, cost, length); 
 
  // If the edge doesn't exist, add it
  if (edges.find(to->getName()) == edges.end() )
  {
    edges.insert(std::make_pair(to->getName(), newEdge));
    return true;
  }
 
  return false;  
}
    
/**
 * Returns the Vertex's name.
 */
const std::string& Vertex::getName() const
{
  return this->name; 
}
   
/**
 * Gets the Vertex's distance value.
 */
unsigned int Vertex::getDistance() const
{
  return this->distance; 
}

/**
 * Sets the Vertex's distance value.
 */
void Vertex::setDistance(unsigned int distance)
{
  this->distance = distance; 
}
    
/**
 * Gets the Vertex's visited state.
 */
bool Vertex::wasVisited() const
{
  return this->visited; 
}

/**
 * Sets the Vertex's visited state.
 */
void Vertex::setVisited(bool visited)
{
  this->visited = visited; 
}

/**
 * Clears all edges from this Vertex.
 */
void Vertex::clearEdges()
{
  this->edges.clear(); 
}

/**
 * Gets total cost of all edges terminating at this Vertex.
 */
unsigned int Vertex::totalEdgeCost() const
{
  return 0; 
}

/**
 * Returns a reference to the internal map of Edges.
 * Used by UndirectedGraph for Dijkstra/Prim algorithms.
 */
const std::unordered_map<std::string, Edge>& Vertex::getEdges() const
{
  return this->edges; 
}
