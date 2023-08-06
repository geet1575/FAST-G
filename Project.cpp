#include <iostream>
#include <Project.hpp>
int main() {
    BipartiteGraph bipartiteGraph;

    // Create nodes for the bipartite graph
    Node* node1 = new Node(1);
    Node* node2 = new Node(2);
    Node* node3 = new Node(3);
    Node* node4 = new Node(4);
    // Add nodes to the graph
    bipartiteGraph.addNode(node1);
    bipartiteGraph.addNode(node2);
    bipartiteGraph.addNode(node3);
    bipartiteGraph.addNode(node4);

    // Using the BipartiteGraph's addEdge function to add edges to the graph
    bipartiteGraph.addEdge(node1, node2, 5);
    bipartiteGraph.addEdge(node1, node3, 7);
    bipartiteGraph.addEdge(node4, node3, 3);

    int minWeightMatching = bipartiteGraph.FindMinWeightPerfectMatching();

    std::cout << "Minimum Weight Perfect Matching: " << minWeightMatching << std::endl;

    // Deleting the nodes and freeing up memory
    delete node1;
    delete node2;
    delete node3;
    delete node4;

    return 0;
}
