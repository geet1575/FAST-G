#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <unordered_set>

class DirectedGraph {
private:
    // Structure representing a node in the directed graph
    struct Node {
        int label;
        std::vector<Node*> adjNodes;

    };

    std::vector<Node*> nodes; // Vector to store the nodes of the graph

public:
    // Constructor
    DirectedGraph() = default;

    // Destructor to deallocate memory
    ~DirectedGraph() {
        for(auto it = nodes.begin(); it !=nodes.end();++it)
            {
                delete *it;
            }
        nodes.clear();
    }

    // Function to add a directed edge from one node to another
    void addEdge(int fromNodeLabel, int toNodeLabel) {
        Node* fromNode = findOrCreateNode(fromNodeLabel);
        Node* toNode = findOrCreateNode(toNodeLabel);

        if (!fromNode || !toNode) {
            return; // Check if nodes are valid
        }

        fromNode->adjNodes.push_back(toNode);
}

    // Function to add a node to the graph
    void addNode(int label) {
        findOrCreateNode(label);
    }

    // Function to print the graph
    void printGraph() {
        for (auto it = nodes.begin(); it != nodes.end(); ++it) {
            std::cout << "Node " << (*it)->label << " -> ";

            if ((*it)->adjNodes.empty()) {
                std::cout << "No adjacent nodes";
            } else {
                for (auto itt = (*it)->adjNodes.begin(); itt != (*it)->adjNodes.end(); ++itt) {
                    std::cout << (*itt)->label << " ";
            }
        }

        std::cout << std::endl;
    }
}

    // Function to root the graph at a specific node
    void rootGraph(int rootLabel) {
        Node* rootNode = findOrCreateNode(rootLabel);

        // Swap the root node with the first node in the nodes vector
    auto it = nodes.begin();
    while (it != nodes.end()) {
        if (*it == rootNode) {
            break;
        }
        ++it; // Add this line to increment the iterator
    }

    if (it != nodes.begin()) {
        std::swap(*it, nodes[0]);
    }
}

    // Function to perform breadth-first search (BFS) traversal
    void breadthFirstSearch() {
        if (nodes.empty()) {
            std::cout << "Graph is empty." << std::endl;
            return;
        }

        std::unordered_set<Node*> visited; // Set to track visited nodes
        std::queue<Node*> queue; // Queue to store nodes for BFS

        Node* startNode = nodes[0];
        queue.push(startNode);
        visited.insert(startNode);

        while (!queue.empty()) {
            Node* currentNode = queue.front();
            queue.pop();

            std::cout << currentNode->label << " ";
            for(auto it = currentNode->adjNodes.begin(); it != currentNode->adjNodes.end();++it)
                {
                    if (visited.find(*it) == visited.end()) {
                        queue.push(*it);
                        visited.insert(*it);
                    }
                }
        }

        std::cout << std::endl;
    }

    // Function to perform depth-first search (DFS) traversal
    void depthFirstSearch() {
        if (nodes.empty()) {
            std::cout << "Graph is empty." << std::endl;
            return;
        }

        std::unordered_set<Node*> visited; // Set to track visited nodes
        std::stack<Node*> stack; // Stack to store nodes for DFS

        Node* startNode = nodes[0];
        stack.push(startNode);
        visited.insert(startNode);

        while (!stack.empty()) {
            Node* currentNode = stack.top();
            stack.pop();

            std::cout << currentNode->label << " ";
            for (auto it = currentNode->adjNodes.begin(); it != currentNode->adjNodes.end(); ++it)
                {
                    if (visited.find(*it) == visited.end()) {
                        stack.push(*it);
                        visited.insert(*it);
                    }
                }
        }

        std::cout << std::endl;
    }

        // Function to perform BFS given the start node
    void breadthFirstSearch(int StartNodelabel){
        rootGraph(StartNodelabel);
        breadthFirstSearch();
    }

    // Function to perform DFS given the start node 
    void depthFirstSearch(int StartNodelabel){
        rootGraph(StartNodelabel);
        depthFirstSearch();
    }    

private:
    // Function to find an existing node or create a new node with the given label
    Node* findOrCreateNode(int label) {
        for (auto it = nodes.begin(); it != nodes.end();++it) {
            if ((*it)->label == label) {
                return *it;
            }
        }

        // Create a new node and add it to the graph
        Node* newNode = new Node;
        newNode->label = label;
        nodes.push_back(newNode);
        return newNode;
    }
};

class UndirectedGraph {
private:
    // Structure representing a node in the undirected graph
    struct Node {
        int label;
        std::vector<Node*> adjNodes;

    };

    std::vector<Node*> nodes; // Vector to store the nodes of the graph

public:
    // Constructor
    UndirectedGraph() = default;

    // Destructor to deallocate memory
    ~UndirectedGraph() {
        for (auto it = nodes.begin(); it != nodes.end();++it) {
            delete *it;
        }
        nodes.clear();
    }

    // Function to add an undirected edge between two nodes
    void addEdge(int fromNodeLabel, int toNodeLabel) {
        Node* fromNode = findOrCreateNode(fromNodeLabel);
        Node* toNode = findOrCreateNode(toNodeLabel);

        if (!fromNode || !toNode) {
            return; // Check if nodes are valid
        }

    fromNode->adjNodes.push_back(toNode);
}

    // Function to add a node to the graph
    void addNode(int label) {
        findOrCreateNode(label);
    }

    // Function to print the graph
    void printGraph() {
        for (auto it = nodes.begin(); it != nodes.end(); ++it) {
            std::cout << "Node " << (*it)->label << " -> ";

            if ((*it)->adjNodes.empty()) {
                std::cout << "No adjacent nodes";
            } else {
                for (auto itt = (*it)->adjNodes.begin(); itt != (*it)->adjNodes.end(); ++itt) {
                    std::cout << (*itt)->label << " ";
                }
        }

        std::cout << std::endl;
    }
}


    // Function to root the graph at a specific node
    void rootGraph(int rootLabel) {
        Node* rootNode = findOrCreateNode(rootLabel);

        // Swap the root node with the first node in the nodes vector
        auto it = nodes.begin();
        while (it != nodes.end()) {
           if (*it == rootNode) {
                break;
            }   
        ++it;
        }
    }

    // Function to perform breadth-first search (BFS) traversal
    void breadthFirstSearch() {
        if (nodes.empty()) {
            std::cout << "Graph is empty." << std::endl;
            return;
        }

        std::unordered_set<Node*> visited; // Set to track visited nodes
        std::queue<Node*> queue; // Queue to store nodes for BFS

        Node* startNode = nodes[0];
        queue.push(startNode);
        visited.insert(startNode);

        while (!queue.empty()) {
            Node* currentNode = queue.front();
            queue.pop();

            std::cout << currentNode->label << " ";

            for(auto it = currentNode->adjNodes.begin(); it != currentNode->adjNodes.end();++it)
                {
                    if (visited.find(*it) == visited.end()) {
                        queue.push(*it);
                        visited.insert(*it);
                    }
                }
        }

        std::cout << std::endl;
    }

    // Function to perform depth-first search (DFS) traversal
    void depthFirstSearch() {
        if (nodes.empty()) {
            std::cout << "Graph is empty." << std::endl;
            return;
        }

        std::unordered_set<Node*> visited; // Set to track visited nodes
        std::stack<Node*> stack; // Stack to store nodes for DFS

        Node* startNode = nodes[0];
        stack.push(startNode);
        visited.insert(startNode);

        while (!stack.empty()) {
            Node* currentNode = stack.top();
            stack.pop();

            std::cout << currentNode->label << " ";

            for (auto it = currentNode->adjNodes.begin(); it != currentNode->adjNodes.end(); ++it)
                {
                    if (visited.find(*it) == visited.end()) {
                        stack.push(*it);
                        visited.insert(*it);
                    }
                }
        }

        std::cout << std::endl;
    }

    // Function to perform BFS given the start node
    void breadthFirstSearch(int StartNodelabel){
        rootGraph(StartNodelabel);
        breadthFirstSearch();
    }

    // Function to perform DFS given the start node 
    void depthFirstSearch(int StartNodelabel){
        rootGraph(StartNodelabel);
        depthFirstSearch();
    }    
private:
    // Function to find an existing node or create a new node with the given label
    Node* findOrCreateNode(int label) {
        for (auto it = nodes.begin(); it != nodes.end();++it) {
            if ((*it)->label == label) {
                return *it;
            }
        }

        // Create a new node and add it to the graph
        Node* newNode = new Node;
        newNode->label = label;
        nodes.push_back(newNode);
        return newNode;
    }
};

int main() {
    DirectedGraph directedGraph;
    directedGraph.addNode(1);
    directedGraph.addNode(2);
    directedGraph.addNode(3);
    directedGraph.addEdge(1, 2);
    directedGraph.addEdge(2, 3);

    std::cout << "Directed Graph:" << std::endl;
    directedGraph.printGraph();

    std::cout << "Depth-First Search: ";
    directedGraph.depthFirstSearch();

    std::cout << "Breadth-First Search: ";
    directedGraph.breadthFirstSearch();

    std::cout << std::endl;

    UndirectedGraph undirectedGraph;
    undirectedGraph.addNode(10);
    undirectedGraph.addNode(20);
    undirectedGraph.addNode(30);
    undirectedGraph.addEdge(10, 20);
    undirectedGraph.addEdge(20, 30);

    std::cout << "Undirected Graph:" << std::endl;
    undirectedGraph.printGraph();

    std::cout << "Depth-First Search: ";
    undirectedGraph.depthFirstSearch();

    std::cout << "Breadth-First Search: ";
    undirectedGraph.breadthFirstSearch();

    return 0;
}
