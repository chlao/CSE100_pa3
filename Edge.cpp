#include "Edge.hpp"
/*********************************************************
 * Author: Christine Lao and Shaina Wan 
 * This file holds the edge constructor for an edge in an
 * undirected graph. The edge has both a cost and a length,
 * which represent the amount of money spent to establish 
 * that connection and the the amount of time spent
 * traveling from the origin vertex to the destination 
 * vertex along the edge. Also has pointers to keep track of
 * the original vertex and to the destination vertex. 
 * ******************************************************/
/**
 * Edge Constructor. Constructs an Edge from the 
 * given parameters.
 */
Edge::Edge(Vertex *from, Vertex *to,
       unsigned int cost,
       unsigned int length)
{
  this->from = from; 
  this->to = to; 
  this->cost = cost; 
  this->length = length;  
}

/**
 * Returns a pointer to the Vertex that this Edge originates
 * from.
 */
Vertex* Edge::getFrom() const
{
  return this->from; 
}

/**
 * Returns a pointer to the Vertex that this Edge terminates
 * at.
 */
Vertex* Edge::getTo() const
{
  return this->to; 
}

/**
 * Sets the cost of this Edge
 */
void Edge::setCost(unsigned int cost)
{
  this->cost = cost; 
}

/** 
 * Returns the cost of this Edge
 */
unsigned int Edge::getCost() const
{
  return this->cost; 
}

/**
 * Sets the length of this Edge
 */
void Edge::setLength(unsigned int length)
{
  this->length = length; 
}

/** 
 * Returns the length of this Edge
 */
unsigned int Edge::getLength() const
{
  return this->length; 
}

/*
 * Compares this Edge to another Edge. Suitable for
 * use with a priority queue where Edges with the lowest
 * weight have the highest priority.
 *
 * Returns true if this Edge's cost is more than
 * right's cost.
 */
bool Edge::operator<(const Edge &right) const
{
  if (this->cost > right.cost) 
    return true;

  return false;  
}
