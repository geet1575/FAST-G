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

    std::vector<Node*> nodes; // Vector to store the pointers to nodes of the graph

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
    // Function to input a directed graph
    void InputDirectedGraph(){
        std::cout<<"This function inputs a directed graph"<<std::endl<<"Enter the number of nodes in the graph";
        int n;
        std::cin>>n;
        while(n--){
            std::cout<<std::endl<<"Enter the label of the next node "<<std::endl;
            int label;
            std::cin>>label;
            if(label == 0){
                std::cout<<std::endl<<"0 is not a valid label for the node"<<std::endl;
                n++;
            }
            else{
            if(NodeExist(label)){
                std::cout<<std::endl<<"This node already exists"<<std::endl;
                n++;
            }
            }

        }
        std::cout<<"Now enter the edges in pairs of two "<<std::endl<<"Enter 0 0 if you want to stop"<<std::endl;
        bool flag = true;
        while(flag){
            int fromlabel, tolabel;
            std::cin>>fromlabel>>tolabel;
            if(fromlabel == 0 && tolabel == 0){
                flag = false;
            }
            else{
                if(!NodeExist(fromlabel) || !NodeExist(tolabel)){
                    std::cout<<std::endl<<" Invalid input"<<std::endl;
                }
                else{
                    addEdge(fromlabel,tolabel);
                }


            }
        }

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
    // Function to find whether a node already exists in a graph
    bool NodeExist(int label){ 
        if(nodes.empty()){
            return false;
        }
        for(auto it = nodes.begin();it != nodes.end();++it){
            if((*it)->label == label){
                return true;
            }
        }
        return false;
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
    toNode->adjNodes.push_back(fromNode);
}

    // Function to add a node to the graph
    void addNode(int label) {
        findOrCreateNode(label);
    }
    // Function to input an undirected graph
    void InputUndirectedGraph(){
        std::cout<<"This is a function to input an undirected graph"<<std::endl<<"Enter the number of nodes in the graph";
        int n;
        std::cin>>n;
        while(n--){
            std::cout<<std::endl<<"Enter the label of the next node "<<std::endl;
            int label;
            std::cin>>label;
            if(label == 0){
                std::cout<<std::endl<<"0 is not a valid label for the node"<<std::endl;
                n++;
            }
            else{
            if(NodeExist(label)){
                std::cout<<std::endl<<"This node already exists"<<std::endl;
                n++;
            }
            }

        }
        std::cout<<"Now enter the edges in pairs of two "<<std::endl<<"Enter 0 0 if you want to stop"<<std::endl;
        bool flag = true;
        while(flag){
            int fromlabel, tolabel; // Not like the distinction of fromlabel, tolabel matters, it's an undirected graph
            std::cin>>fromlabel>>tolabel;
            if(fromlabel == 0 && tolabel == 0){
                flag = false;
            }
            else{
                if(!NodeExist(fromlabel) || !NodeExist(tolabel)){
                    std::cout<<std::endl<<" Invalid input"<<std::endl;
                }
                else{
                    addEdge(fromlabel,tolabel);
                }


            }
        }

    } 
    // Function to input a simple undirected graph (My definition of simple is - no two edges between the same nodes and no edge connecting a node to itself)
    void InputSimpleUndirectedGraph(){
        std::cout<<"This is a function to input a simple undirected graph"<<std::endl<<"Enter the number of nodes in the graph";
        int n;
        std::cin>>n;
        while(n--){
            std::cout<<std::endl<<"Enter the label of the next node "<<std::endl;
            int label;
            std::cin>>label;
            if(label == 0){
                std::cout<<std::endl<<"0 is not a valid label for the node"<<std::endl;
                n++;
            }
            else{
            if(NodeExist(label)){
                std::cout<<std::endl<<"This node already exists"<<std::endl;
                n++;
            }
            }

        }
        std::cout<<"Now enter the edges in pairs of two "<<std::endl<<"Enter 0 0 if you want to stop"<<std::endl;
        bool flag = true;
        while(flag){
            int fromlabel, tolabel; // Not like the distinction of fromlabel, tolabel matters, it's an undirected graph
            std::cin>>fromlabel>>tolabel;
            if(fromlabel == 0 && tolabel == 0){
                flag = false;
            }
            else{
                if(!NodeExist(fromlabel) || !NodeExist(tolabel)){
                    std::cout<<std::endl<<" Invalid input"<<std::endl;
                }
                else if(EdgeExist(fromlabel,tolabel)){
                    std::cout<<std::endl<<" This  edge already exists"<<std::endl;
                }
                else if(fromlabel == tolabel){
                    std::cout<<std::endl<<" A simple graph cannot have any loops consisting of one node"<<std::endl;
                }

                else{
                    addEdge(fromlabel,tolabel);
                }


            }
        }

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
    // Function to find whether a node already exists in a graph
    bool NodeExist(int label){ 
        if(nodes.empty()){
            return false;
        }
        for(auto it = nodes.begin();it != nodes.end();++it){
            if((*it)->label == label){
                return true;
            }
        }
        return false;
    }
    // Function to find whether an edge already exists (for the inputsimpleundirectedgraph)
    bool EdgeExist(int fromlabel, int tolabel){ 
        Node* node1 = findOrCreateNode(fromlabel);
        Node* node2 = findOrCreateNode(tolabel);
        if(node1->adjNodes.empty()){
            return false;
        }
        else{
            for(auto it = node1->adjNodes.begin();it != node1->adjNodes.end();++it){
                if((*it)->label == tolabel){
                    return true;
                }
            }
            return false;
        }
    }
};

class DirectedWeightedGraph{
    // This is mainly the same as the DirectedGraph class
    // There is one slight difference, adjNodes is a pair of all the pointers to nodes and their corresponding weights
private:
    // Structure representing a node in the directed graph
    struct Node {
        int label;
        std::pair<std::vector<Node*>,std::vector<int>> adjNodes; // Each weight corresponds with its particular adjacent Nodes

    };

    std::vector<Node*> nodes; // Vector to store the pointers to nodes of the graph

public:
    // Constructor
    DirectedWeightedGraph() = default;

    // Destructor to deallocate memory
    ~DirectedWeightedGraph() {
        for(auto it = nodes.begin(); it !=nodes.end();++it)
            {
                delete *it;
            }
        nodes.clear();
    }

    // Function to add a directed edge from one node to another
    void addEdge(int fromNodeLabel, int toNodeLabel, int weight) {
        Node* fromNode = findOrCreateNode(fromNodeLabel);
        Node* toNode = findOrCreateNode(toNodeLabel);

        if (!fromNode || !toNode) {
            return; // Check if nodes are valid
        }

        fromNode->adjNodes.first.push_back(toNode);
        fromNode->adjNodes.second.push_back(weight);
}

    // Function to add a node to the graph
    void addNode(int label) {
        findOrCreateNode(label);
    }
    // Function to input a directed graph
    void InputDirectedGraph(){
        std::cout<<"This function inputs a directed weighted graph"<<std::endl<<"Enter the number of nodes in the graph";
        int n;
        std::cin>>n;
        while(n--){
            std::cout<<std::endl<<"Enter the label of the next node "<<std::endl;
            int label;
            std::cin>>label;
            if(label == 0){
                std::cout<<std::endl<<"0 is not a valid label for the node"<<std::endl;
                n++;
            }
            else{
            if(NodeExist(label)){
                std::cout<<std::endl<<"This node already exists"<<std::endl;
                n++;
            }
            }

        }
        std::cout<<"Now enter the edges in pairs of two and the weight after that "<<std::endl<<"Enter 0 0 0 if you want to stop"<<std::endl;
        bool flag = true;
        while(flag){
            int fromlabel, tolabel, weight;
            std::cin>>fromlabel>>tolabel>>weight;
            if(fromlabel == 0 && tolabel == 0 && weight == 0){
                flag = false;
            }
            else{
                if(!NodeExist(fromlabel) || !NodeExist(tolabel)){
                    std::cout<<std::endl<<" Invalid input"<<std::endl;
                }
                else{
                    addEdge(fromlabel,tolabel,weight);
                }


            }
        }

    } 
        

    // Function to print the graph
    void printGraph() {
        for (auto it = nodes.begin(); it != nodes.end(); ++it) {
            std::cout << "Node " << (*it)->label << " -> ";

            if ((*it)->adjNodes.first.empty()) {
                std::cout << "No adjacent nodes";
            } else {
                auto itt2 = (*it)->adjNodes.second.begin();
                for (auto itt = (*it)->adjNodes.first.begin() ; itt != (*it)->adjNodes.first.end(); ++itt, ++itt2) {
                    std::cout<<std::endl << (*itt)->label << " with weight "<<(*itt2)<<std::endl;
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
            for(auto it = currentNode->adjNodes.first.begin(); it != currentNode->adjNodes.first.end();++it)
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
            for (auto it = currentNode->adjNodes.first.begin(); it != currentNode->adjNodes.first.end(); ++it)
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
            if ((*it)->label == label ) {
                return *it;
            }
        }

        // Create a new node and add it to the graph
        Node* newNode = new Node;
        newNode->label = label;
        nodes.push_back(newNode);
        return newNode;
    }
    // Function to find whether a node already exists in a graph
    bool NodeExist(int label){ 
        if(nodes.empty()){
            return false;
        }
        for(auto it = nodes.begin();it != nodes.end();++it){
            if((*it)->label == label){
                return true;
            }
        }
        return false;
    }


};

class UndirectedWeightedGraph {
private:
    // Structure representing a node in the undirected graph
    struct Node {
        int label;
        std::pair<std::vector<Node*>,std::vector<int>> adjNodes;

    };

    std::vector<Node*> nodes; // Vector to store the nodes of the graph

public:
    // Constructor
    UndirectedWeightedGraph() = default;

    // Destructor to deallocate memory
    ~UndirectedWeightedGraph() {
        for (auto it = nodes.begin(); it != nodes.end();++it) {
            delete *it;
        }
        nodes.clear();
    }

    // Function to add an undirected edge between two nodes
    void addEdge(int fromNodeLabel, int toNodeLabel, int weight) {
        Node* fromNode = findOrCreateNode(fromNodeLabel);
        Node* toNode = findOrCreateNode(toNodeLabel);

        if (!fromNode || !toNode) {
            return; // Check if nodes are valid
        }

    fromNode->adjNodes.first.push_back(toNode);
    fromNode->adjNodes.second.push_back(weight);
    toNode->adjNodes.first.push_back(fromNode);
    toNode->adjNodes.second.push_back(weight);

}

    // Function to add a node to the graph
    void addNode(int label) {
        findOrCreateNode(label);
    }
    // Function to input an undirected graph
    void InputUndirectedGraph(){
        std::cout<<"This is a function to input an undirected weighted graph"<<std::endl<<"Enter the number of nodes in the graph";
        int n;
        std::cin>>n;
        while(n--){
            std::cout<<std::endl<<"Enter the label of the next node "<<std::endl;
            int label;
            std::cin>>label;
            if(label == 0){
                std::cout<<std::endl<<"0 is not a valid label for the node"<<std::endl;
                n++;
            }
            else{
            if(NodeExist(label)){
                std::cout<<std::endl<<"This node already exists"<<std::endl;
                n++;
            }
            }

        }
        std::cout<<"Now enter the edges in pairs of two and the weight after that "<<std::endl<<"Enter 0 0 0 if you want to stop"<<std::endl;
        bool flag = true;
        while(flag){
            int fromlabel, tolabel; // Not like the distinction of fromlabel, tolabel matters, it's an undirected graph
            int weight;
            std::cin>>fromlabel>>tolabel>>weight;
            if(fromlabel == 0 && tolabel == 0){
                flag = false;
            }
            else{
                if(!NodeExist(fromlabel) || !NodeExist(tolabel)){
                    std::cout<<std::endl<<" Invalid input"<<std::endl;
                }
                else{
                    addEdge(fromlabel,tolabel,weight);
                }


            }
        }

    } 
    // Function to input a simple undirected graph (My definition of simple is - no two edges between the same nodes and no edge connecting a node to itself)
    void InputSimpleUndirectedGraph(){
        std::cout<<"This is a function to input a simple weighted undirected graph"<<std::endl<<"Enter the number of nodes in the graph";
        int n;
        std::cin>>n;
        while(n--){
            std::cout<<std::endl<<"Enter the label of the next node "<<std::endl;
            int label;
            std::cin>>label;
            if(label == 0){
                std::cout<<std::endl<<"0 is not a valid label for the node"<<std::endl;
                n++;
            }
            else{
            if(NodeExist(label)){
                std::cout<<std::endl<<"This node already exists"<<std::endl;
                n++;
            }
            }

        }
        std::cout<<"Now enter the edges in pairs of two and the weight after that "<<std::endl<<"Enter 0 0 0 if you want to stop"<<std::endl;
        bool flag = true;
        while(flag){
            int fromlabel, tolabel,weight; // Not like the distinction of fromlabel, tolabel matters, it's an undirected graph
            std::cin>>fromlabel>>tolabel>>weight;
            if(fromlabel == 0 && tolabel == 0 && weight == 0){
                flag = false;
            }
            else{
                if(!NodeExist(fromlabel) || !NodeExist(tolabel)){
                    std::cout<<std::endl<<" Invalid input"<<std::endl;
                }
                else if(EdgeExist(fromlabel,tolabel)){
                    std::cout<<std::endl<<" This edge already exists"<<std::endl;
                }
                else if(fromlabel == tolabel){
                    std::cout<<std::endl<<" A simple graph cannot have any loops consisting of one node"<<std::endl;
                }
                else{
                    addEdge(fromlabel,tolabel,weight);
                }


            }
        }

    } 


    // Function to print the graph
    void printGraph() {
        for (auto it = nodes.begin(); it != nodes.end(); ++it) {
            std::cout << "Node " << (*it)->label << " -> ";

            if ((*it)->adjNodes.first.empty()) {
                std::cout << "No adjacent nodes";
            } else {
                for (auto itt = (*it)->adjNodes.first.begin(); itt != (*it)->adjNodes.first.end(); ++itt) {
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

            for(auto it = currentNode->adjNodes.first.begin(); it != currentNode->adjNodes.first.end();++it)
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

            for (auto it = currentNode->adjNodes.first.begin(); it != currentNode->adjNodes.first.end(); ++it)
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
    // Function to find whether a node already exists in a graph
    bool NodeExist(int label){ 
        if(nodes.empty()){
            return false;
        }
        for(auto it = nodes.begin();it != nodes.end();++it){
            if((*it)->label == label){
                return true;
            }
        }
        return false;
    }
    // Function to find whether an edge already exists (for the inputsimpleundirectedgraph)
    bool EdgeExist(int fromlabel, int tolabel){ 
        Node* node1 = findOrCreateNode(fromlabel);
        Node* node2 = findOrCreateNode(tolabel);
        if(node1->adjNodes.first.empty()){
            return false;
        }
        else{
            for(auto it = node1->adjNodes.first.begin();it != node1->adjNodes.first.end();++it){
                if((*it)->label == tolabel){
                    return true;
                }
            }
            return false;
        }
    }
};
