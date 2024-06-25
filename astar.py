"""
astar algorithm  (computes shortest path to a target or to a free cell)

Copyright (C) 2024 Alexander Grau

This file is part of VacuumCleaner.

VacuumCleaner is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.

VacuumCleaner is distributed in the hope that it will be useful,     
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

"""

from __future__ import annotations

import heapq

import numpy as np

import time

import matplotlib
matplotlib.use('TkAgg')

import matplotlib.pyplot as plt


class Node:
    """A Node object represent a point in a 2D array."""

    def __init__(self, point: tuple, *, parent: Node = None):
        """Initialize the Node object using a position tuple.

        Parameters
        ----------
        point: tuple
            The coordinate of the Node in a 2D array
        parent: Node = None
            A connection to the parent Node.
        """
        self.pos = point
        self.x, self.y = point

        self.g = 0  # distance between current node and start node
        self.h = 0  # heuristic distance between current node and end node
        self.f = 0  # total cost of utilizing the node

        self.parent = parent

    def __repr__(self):
        return '<Node %s, f=%s>' % (str(self.pos), str(round(self.f, 2)))

    def __lt__(self, other):
        return self.f < other.f

    def __gt__(self, other):
        return self.f > other.f

    def __le__(self, other):
        return self.f <= other.f

    def __ge__(self, other):
        return self.f >= other.f

    def __eq__(self, other):
        return self.f == other.f

    def __ne__(self, other):
        return self.f != other.f

    def manhattan_dist(self, node: Node) -> int:
        """Return the Manhattan distance given another node."""
        return abs(self.x - node.x) + abs(self.y - node.y)

    def euclidean_dist(self, node: Node) -> float:
        """Return the Euclidean distance given another node."""
        return ((self.x - node.x) ** 2 + (self.y - node.y) ** 2) ** 0.5

    def to_coord(self, vector: tuple, parent: bool = True) -> Node:
        """Return a new Node object from a direction vector."""
        return Node((self.x + vector[0], self.y + vector[1]), parent=self if parent else None)

    def pos_in_bdry(self, boundary: tuple) -> bool:
        """Return whether the Node is in the boundary."""
        x, y = boundary
        return (0 <= self.x < x) and (0 <= self.y < y)

    @staticmethod
    def trace_path(node: Node):
        """Find the path from a node to the start."""
        path = [node.pos]

        while node.parent:
            path.append(node.parent.pos)
            node = node.parent

        path.reverse()
        return path


def astar_find_target(graph: np.ndarray, start: Node, target: Node, barrier_value) -> Node:
    """Find the shortest path from a starting Node to a target Node.

    Parameters
    ----------
    graph: np.ndarray
        A 2D matrix of 1s (blocked) and other ints
    start: Node
        The starting node.
    target: Node
        The node to reach.
    Returns
    -------
    Node: the target node
    """
    vectors = [(1, 1), (-1, -1), (1, 0), (0, 1), (-1, 0), (0, -1), (1, -1), (-1, 1)]
    #vectors = [(1, 0), (-1, 0), (0, 1), (0, -1)]

    start.g = 0
    start.h = start.f = start.manhattan_dist(target)

    open_nodes = [start]
    heapq.heapify(open_nodes)
    closed_nodes = graph.copy()
    timeout = time.time() + 10.0
    nextInfoTime = time.time() + 1.0

    while open_nodes:
        curr_node = heapq.heappop(open_nodes)

        # exit if the target has been reached
        if curr_node.pos == target.pos:
            return curr_node

        if time.time() > nextInfoTime:
            nextInfoTime = time.time() + 1.0
            print('astar_find_target open_nodes=', len(open_nodes))

        if time.time() > timeout:
            print('astar_find_target timeout out')
            break

        # travel to neighboring nodes
        for vector in vectors:
            child = curr_node.to_coord(vector)

            # check if outside the graph
            if not child.pos_in_bdry(graph.shape):
                continue

            # check if traversed or if a barrier exists
            if closed_nodes[child.pos] == barrier_value:
                continue

            # update node parameters
            child.g = child.parent.g + child.euclidean_dist(curr_node)
            child.h = child.manhattan_dist(target)
            child.f = child.g + child.h

            heapq.heappush(open_nodes, child)
            # optional: mark child node pos as closed
            closed_nodes[child.pos] = barrier_value

        # close the current node
        closed_nodes[curr_node.pos] = barrier_value

    # exit if no target found
    #raise Warning('No path found.')
    print('No path found.')
    return None


def astar_find_free(graph: np.ndarray, start: Node, free_value, barrier_value) -> Node:
    """Find the shortest path from a starting Node to a target Node.

    Parameters
    ----------
    graph: np.ndarray
        A 2D matrix of 1s (blocked) and other ints
    start: Node
        The starting node.
    Returns
    -------
    Node: the target node
    """
    #vectors = [(1, 1), (-1, -1), (1, 0), (0, 1), (-1, 0), (0, -1), (1, -1), (-1, 1)]
    vectors = [(1, 0), (-1, 0), (0, 1), (0, -1)]
  
    start.g = 0
    start.h = start.f = 0 # start.manhattan_dist(target)

    open_nodes = [start]
    heapq.heapify(open_nodes)
    closed_nodes = graph.copy()
    timeout = time.time() + 10.0
    nextInfoTime = time.time() + 1.0

    while open_nodes:
        curr_node = heapq.heappop(open_nodes)

        # exit if the target has been reached
        if graph[curr_node.pos] == free_value:
            return curr_node
        
        if time.time() > nextInfoTime:
            nextInfoTime = time.time() + 1.0
            print('astar_find_free open_nodes=', len(open_nodes))

        if time.time() > timeout:
            print('astar_find_free timeout out')
            break

        # travel to neighboring nodes
        for vector in vectors:
            child = curr_node.to_coord(vector)

            # check if outside the graph
            if not child.pos_in_bdry(graph.shape):
                continue

            # check if traversed or if a barrier exists
            if closed_nodes[child.pos] == barrier_value:
                continue

            # update node parameters
            child.g = child.parent.g + child.euclidean_dist(curr_node)
            child.h = 0 # child.manhattan_dist(target)
            child.f = child.g + child.h

            heapq.heappush(open_nodes, child)
            # optional: mark child node pos as closed
            closed_nodes[child.pos] = barrier_value


        # close the current node
        closed_nodes[curr_node.pos] = barrier_value

    # exit if no target found
    #raise Warning('No path found.')
    print('No path found.')
    return None


if __name__ == '__main__':
    s = Node((4, 0))
    t = Node((4, 4))

    mtx = np.array([
        [2, 2, 2, 2, 0, 1, 1],
        [2, 2, 2, 2, 0, 1, 2],
        [2, 2, 0, 2, 2, 2, 2],
        [2, 0, 0, 2, 0, 0, 1],
        [2, 2, 0, 2, 2, 2, 2]
    ])

    end = astar_find_free(mtx, s, 1, 0)  # free, barrier   
    #end = astar_find_target(mtx, s, t, 0)  
    print(end.g)
    print('path', Node.trace_path(end))
    plt.imshow(mtx, interpolation='none')
    path = np.array( Node.trace_path(end) )
    #plt.plot(*path.T)    
    plt.plot(path[:,1], path[:,0], linestyle="", marker="o", markerfacecolor='red')        
    plt.show()
    

