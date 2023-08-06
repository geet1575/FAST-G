// This file is just for the project
// I'm defining a separate class for the bipartite graph 

#include <iostream>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <climits> // For INT_MAX

class Node {
public:
    int label;
    std::pair<std::vector<Node*>, std::vector<int>> adjNodes; // Each weight corresponds with its particular adjacent Nodes

    Node(int lbl) : label(lbl) {}
};

class BipartiteGraph {
private:
    std::vector<Node*> nodes; // Vector to store the pointers to nodes of the graph
    std::unordered_set<Node*> partitionA; // Set to store nodes in partition A
    std::unordered_set<Node*> partitionB; // Set to store nodes in partition B

public:
    // Function to add an edge between two nodes with a given weight
    void addEdge(Node* fromNode, Node* toNode, int weight) {
        fromNode->adjNodes.first.push_back(toNode);
        fromNode->adjNodes.second.push_back(weight);

        toNode->adjNodes.first.push_back(fromNode);
        toNode->adjNodes.second.push_back(weight);

        partitionA.insert(fromNode);
        partitionB.insert(toNode);
    }

    // Function to add a node to the graph
    void addNode(Node* node) {
        nodes.push_back(node);
    }

    // Function to get the nodes in partition A
    std::unordered_set<Node*> getPartitionA() const {
        return partitionA;
    }

    // Function to get the nodes in partition B
    std::unordered_set<Node*> getPartitionB() const {
        return partitionB;
    }
    // Function to find a perfect matching of minimum weight in the bipartite graph using the Hungarian algorithm
    int FindMinWeightPerfectMatching() {
        // Implementation of the Hungarian algorithm
        int n = nodes.size();
        std::unordered_map<Node*, int> matchX; // Matching from partition A to partition B
        std::unordered_map<Node*, int> matchY; // Matching from partition B to partition A

        // Initialize matchX and matchY arrays with -1 (indicating no matching yet)
        for (Node* node : partitionA) {
            matchX[node] = -1;
        }

        for (Node* node : partitionB) {
            matchY[node] = -1;
        }

        // Step 1: Find an augmenting path using DFS
        for (Node* node : partitionA) {
            std::unordered_set<Node*> visited;
            dfs(node, visited, matchX, matchY);
        }

        // Step 2: Calculate the minimum weight perfect matching
        int minWeightMatching = 0;
        for (Node* node : partitionA) {
            if (matchX[node] != -1) {
                minWeightMatching += node->adjNodes.second[matchX[node]];
            }
        }

        return minWeightMatching;
    }

private:
    // Function to find an augmenting path using DFS
    bool dfs(Node* node, std::unordered_set<Node*>& visited, std::unordered_map<Node*, int>& matchX, std::unordered_map<Node*, int>& matchY) {
        if (visited.find(node) != visited.end()) {
            return false;
        }

        visited.insert(node);

        for (int i = 0; i < node->adjNodes.first.size(); i++) {
            Node* neighbor = node->adjNodes.first[i];

            if (matchY.find(neighbor) == matchY.end() || dfs(neighbor, visited, matchX, matchY)) {
                matchX[node] = i;
                matchY[neighbor] = i;
                return true;
            }
        }

        return false;
    }
};
 