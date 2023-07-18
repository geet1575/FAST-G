#include <iostream>
#include <Graphs.hpp>


int main() {
    // DirectedGraph directedGraph;
    // directedGraph.addNode(1);
    // directedGraph.addNode(2);
    // directedGraph.addNode(3);
    // directedGraph.addEdge(1, 2);
    // directedGraph.addEdge(2, 3);
    // // directedGraph.addEdge(3,1);

    // std::cout << "Directed Graph:" << std::endl;
    // directedGraph.printGraph();
    // // directedGraph.rootGraph(3);
    // std::cout << "Depth-First Search: ";
    // directedGraph.depthFirstSearch();

    // std::cout << "Breadth-First Search: ";
    // directedGraph.breadthFirstSearch();
    // directedGraph.~DirectedGraph();

    // std::cout << std::endl;

    // UndirectedGraph undirectedGraph;
    // undirectedGraph.addNode(10);
    // undirectedGraph.addNode(20);
    // undirectedGraph.addNode(30);
    // undirectedGraph.addEdge(10, 20);
    // undirectedGraph.addEdge(20, 30);

    // std::cout << "Undirected Graph:" << std::endl;
    // undirectedGraph.printGraph();

    // std::cout << "Depth-First Search: ";
    // undirectedGraph.depthFirstSearch();

    // std::cout << "Breadth-First Search: ";
    // undirectedGraph.breadthFirstSearch();
    // undirectedGraph.~UndirectedGraph();
    // DirectedGraph directedgraph;
    // directedgraph.InputDirectedGraph();

    DirectedWeightedGraph directedweightedgraph;
    directedweightedgraph.InputDirectedGraph();
    directedweightedgraph.Dijkstra();
    return 0;
}
