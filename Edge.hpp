/*********************************************************
 * Author: Christine Lao and Shaina Wan 
 * This header file holds the edge constructor for an edge 
 * in an undirected graph. The edge has both a cost and a 
 * length, which represent the amount of money spent to 
 * establish that connection and the the amount of time spent
 * traveling from the origin vertex to the destination 
 * vertex along the edge. Also has pointers to keep track of
 * the original vertex and to the destination vertex. 
 * ******************************************************/
#ifndef EDGE_HPP
#define EDGE_HPP

class Vertex;

/**
 * Represents an edge in a graph.
 *
 * Maintains pointers to the vertices that the edge originates 
 * from and terminates at. Edges have both a cost and a length,
 * which are both non-negative integers.
 *
 * Follows value semantics, so can be copy constructed.
 */
class Edge {
  public:
    /**
     * Constructs an Edge from the given parameters.
     */
    Edge(Vertex *from, Vertex *to,
            unsigned int cost,
            unsigned int length);

    /**
     * Returns a pointer to the Vertex that this Edge originates
     * from.
     */
    Vertex *getFrom() const;
    
    /**
     * Returns a pointer to the Vertex that this Edge terminates
     * at.
     */
    Vertex *getTo() const;

    /**
     * Sets the cost of this Edge.
     */
    void setCost(unsigned int cost);

    /**
     * Returns the cost of this Edge.
     */
    unsigned int getCost() const;

    /**
     * Sets the length of this Edge.
     */
    void setLength(unsigned int length);

    /**
     * Returns the length of this Edge.
     */
    unsigned int getLength() const;

    /*
     * Compares this Edge to another Edge. Suitable for
     * use with a priority queue where Edges with the lowest
     * weight have the highest priority.
     *
     * Returns true if this Edge's cost is more than
     * right's cost.
     */
    bool operator<(const Edge &right) const;
    
  private:
    /**
     * Vertex that this Edge originates from.
     */
    Vertex *from;

    /**
     * Vertex that this Edge terminates at.
     */
    Vertex *to;

    /**
     * Cost of this Edge.
     */
    unsigned int cost;

    /**
     * Length of this Edge.
     */
    unsigned int length;
};

#endif
