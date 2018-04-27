#include<iostream>
#include<vector>
#include<assert.h>
#include <math.h>
#include <functional>
#include <cstdlib>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <fstream>
#include <stack>


bool constTrue_(std::vector<bool>& inputs) {
    return true;
}

bool constFalse_(std::vector<bool>& inputs) {
    return false;
}

bool migFunction_(std::vector<bool>& inputs) {
    int truesNum = 0;
    int falseNum = 0;
    for (bool val: inputs) {
        val ? ++truesNum : ++falseNum;
    }
    return truesNum > falseNum;
}

bool invFunction_(std::vector<bool>& inputs) {
    return !inputs[0];
}


// function extention
struct NamedFunction {
    std::function<bool(std::vector<bool>&)> function;
    std::string name;
    
    bool operator()(std::vector<bool>& input) {
        return function(input);
    }
};


NamedFunction constTrue({constTrue_, "constTrue"});
NamedFunction constFalse({constFalse_, "constFalse"});
NamedFunction migFunction({migFunction_, "mig"});
NamedFunction invFunction({invFunction_, "inv"});


class Visitor;
class Node;
class MigOptimizer;

void DumpMig(Node* node, std::ostream& stream);



// Abstract Scheme
class Node {
public:
    Node(NamedFunction function) : function(function) {}
    
    bool Compute(std::vector<bool> inputVal) {
        SetToZero();
        return Compute_(inputVal);
    }
    
    std::string GetName() {
        return function.name;
    }

    ~Node() {
        for (Node* input: inputs) {
            if (input) {
                delete input;
            }
        }
    }

private:
    NamedFunction function;
	
    bool val;
    bool computed;
    
	std::vector<Node*> inputs;
	std::vector<Node*> outputs;
    
    void SetToZero() {
        if (computed) {
            computed = false;
            for (auto& input: inputs) {
                input->SetToZero();
            }
        }
        return;
    }
    
    bool Compute_(std::vector<bool>& inputVals) {
        if (computed) {
            return val;
        } else if (inputs.empty()) {
            val = function(inputVals);
        } else {
            std::vector<bool> parentVals;
            for (int i = 0; i < inputs.size(); ++i) {
                parentVals.push_back(inputs[i]->Compute_(inputVals));
            }
            val = function(parentVals);
        }
        computed = true;
        return val;
    };


    friend class Synthezator;
    friend class MigOptimizer;
    friend class Reshaper;
    friend class ValidityChecker;
    friend class Dumper;
    friend class MigDepthCountor;
    friend class CopyMaker;
    friend class DepthBoolOptimizer;
    friend void bfs(Node* node, Visitor& visitor);
    friend void dfs_(Node* node, Visitor& visitor, std::unordered_set<Node*>& used);
    friend Node* CheckNodeValidity(Node* node);
};


struct Edge {
    Node* in;
    Node* out;
};


// Graph Traversal and Visitor Pattern

class Visitor {
public:
    virtual void Visit(Node* node) = 0;
};

class MigSizeCountor : public Visitor {
public:
    MigSizeCountor() : count(0) {}

    void Visit(Node* node) {
        if (node->GetName() == "mig") {
            ++count;
        }
    }
    
    int Count() {
        return count;
    }
private:
    int count;
};

class ValidityChecker : public Visitor {
public:
    void Visit(Node* node) {
        if (node->GetName() == "mig") {
            assert(node->inputs.size() == 3);
        } else if (node->GetName() == "inv") {
            assert(node->inputs.size() == 1);
        }
        for (Node* input: node->inputs) {
            assert(std::find(input->outputs.begin(),
                            input->outputs.end(),
                            node) != input->outputs.end());
        }
        for (Node* output: node->outputs) {
            assert(std::find(output->inputs.begin(),
                            output->inputs.end(),
                            node) != output->inputs.end());
        }
    }
};

class Dumper : public Visitor {
public:
	Dumper(std::ostream& stream) : stream(stream) {}

	void Visit(Node* node) {
        if (node->GetName() == "input") {
            stream << "Node" << node << " " << "[color=green];\n";
        } else if (node->GetName() == "inv") {
            stream << "Node" << node << " " << "[color=blue];\n";
        } else if (node->GetName() == "mig") {
		    stream << "Node" << node << " " << "[color=red];\n";
        } else {
            stream << "Node" << node << " " << "[color=gray];\n";
        }

		for (Node* input: node->inputs) {
			stream << "Node" << input << " -> Node" << node << ";\n";
		}
	}

private:
	std::ostream& stream;
};

class MigDepthCountor : public Visitor {
public:
    void Visit(Node* node) {
        int depth = 0;
        for (auto parent: node->inputs) {
            depth = std::max(depth, depthes[parent]);
        }
        if (node->GetName() == "mig") {
            ++depth;
        }
        depthes[node] = depth;
        return;
    }

    int GetDepth(Node* node) {
        return depthes[node];
    }

private:
    std::unordered_map<Node*, int> depthes;
};

class TopologicalSorter : public Visitor {
public:
    void Visit(Node* node) {
        stack.push(node);
        return;
    }
private:
    std::stack<Node*> stack;

    friend class MigOptimizer;
};

class NodeToVector : public Visitor {
public:
    void Visit(Node* node) {
        nodesVector.push_back(node);
        return;
    }

    std::vector<Node*> nodesVector;
};

void bfs(Node* node, Visitor& visitor) {
    std::unordered_set<Node*> used;
    std::queue<Node*> bfsQueue;
    bfsQueue.push(node);
    used.insert(node);
    while (!bfsQueue.empty()) {
            visitor.Visit(bfsQueue.front());
            for (auto parent: bfsQueue.front()->inputs) {
                if (used.find(parent) == used.end()) {
                    bfsQueue.push(parent);
                    used.insert(parent);
                }
            }
        bfsQueue.pop();
    }
    return;
}

void dfs_(Node* node, Visitor& visitor, std::unordered_set<Node*>& used) {
    used.insert(node);
    for (auto parent: node->inputs) {
        if (used.find(parent) == used.end()) {
            dfs_(parent, visitor, used);
        }
    }
    visitor.Visit(node);
    return;
}

void dfs(Node* node, Visitor& visitor) {
    std::unordered_set<Node*> used;
    dfs_(node, visitor, used);
}


// Abstract Synthezator
class Synthezator {
public:
    virtual Node* Synthez(std::vector<bool> function) = 0;

protected:
    void ConnectNodes(Node* in, Node* out) {
        (in->outputs).push_back(out);
        (out->inputs).push_back(in);
        return;
    }
    
    Node* SynthezCustomNode(std::vector<Node*> inputs,
                            NamedFunction namedFunction) {
        Node* newNode = new Node(namedFunction);
        for (auto& node: inputs) {
            ConnectNodes(node, newNode);
        }
        return newNode;
    }
};



class MIGSynthezator : public Synthezator {
public:
    MIGSynthezator() {}
    
    Node* Synthez(std::vector<bool> funcTable) {
        
        int inputsNum = int(log2(funcTable.size()));
        std::vector<Node*> inputs;
        for (int i = 0; i < inputsNum; ++i) {
            inputs.push_back(new Node({[=](std::vector<bool>& inputVals)->bool {
                return inputVals[i];
            }, "input" }));
        }
        
        std::vector<Node*> conjunctions;
        for (int num = 0; num < funcTable.size(); ++num) {
            if (funcTable[num]) {
                conjunctions.push_back(SynthezConj(inputs, num));
            }
        }
        
        return SynthezDisj(conjunctions);
    }

private:
    Node* SynthezInv(Node* node) {
        return SynthezCustomNode({node}, invFunction);
    }
    
    
    Node* SynthezOR(Node* node1, Node* node2) {
        Node* trueNode = new Node(constTrue);
        return SynthezCustomNode({node1, node2, trueNode}, migFunction);
    }
    
    Node* SynthezAND(Node* node1, Node* node2) {
        Node* falseNode = new Node(constFalse);
        return SynthezCustomNode({node1, node2, falseNode}, migFunction);
    }
    
    Node* SynthezConj(std::vector<Node*>& inputs, int num) {
        bool firstBit = bool((1 << 0) & num);
        std::vector<Node*> nodes;
        nodes.push_back(firstBit ? inputs[0] : SynthezInv(inputs[0]));
        
        for (int i = 1; i < inputs.size(); ++i) {
            bool bit = bool((1 << i) & num);
            nodes.push_back(SynthezAND(nodes.back(), bit ? inputs[i] : SynthezInv(inputs[i])));
        }
        
        return nodes.back();
    }
    
    Node* SynthezDisj(std::vector<Node*>& inputs) {
        if (inputs.empty()) {
            return new Node(constFalse);
        } else {
            std::vector<Node*> nodes;
            nodes.push_back(inputs[0]);
            for (int i = 1; i < inputs.size(); ++i) {
                nodes.push_back(SynthezOR(nodes.back(), inputs[i]));
            }
            return nodes.back();
        }
    }

    friend class DepthBoolOptimizer;
    friend class SizeBoolOptimizer;
};


// Optimizer

class MigOptimizer {
public:
    virtual
    
    Node* Optimize(Node*& node) = 0;
    
    int GetSize(Node* node) {
        MigSizeCountor migSizeVisitor;
        bfs(node, migSizeVisitor);
        return migSizeVisitor.Count();
    }
    
    int GetDepth(Node* node) {
        MigDepthCountor migDepthCountor;
        dfs(node, migDepthCountor);
        return migDepthCountor.GetDepth(node);
    }

    std::vector<Node*> GetAllParents(Node* node) {
        NodeToVector nodeToVector;
        bfs(node, nodeToVector);
        return nodeToVector.nodesVector;
    }

protected:
    Node* GetRandomNode(Node* node) {
        int nodeSize = GetSize(node);
        if (nodeSize == 0) {
            return node;
        } else if (node->GetName() == "mig" && rand() % nodeSize == 0) {
            return node;
        } else {
            int randIndex = rand() % (node->inputs).size();
            return GetRandomNode(node->inputs[randIndex]);
        }
    }
    
    void SwitchEdges(Edge first, Edge second) {
    	DeleteEdge(first);
    	DeleteEdge(second);
    	InsertEdge({first.in, second.out});
    	InsertEdge({second.in, first.out});
        return;
    }
    
    void DeleteEdge(Edge edge) {
        auto inPosition = std::find(edge.in->outputs.begin(), edge.in->outputs.end(), edge.out);
        auto outPosition = std::find(edge.out->inputs.begin(), edge.out->inputs.end(), edge.in);
        edge.in->outputs.erase(inPosition);
        edge.out->inputs.erase(outPosition);
        return;
    }
    
    static void InsertEdge(Edge edge) {
        edge.in->outputs.push_back(edge.out);
        edge.out->inputs.push_back(edge.in);
        return;
    }
    
    static void InsertEdges(std::vector<Edge> edges) {
        for (auto edge: edges) {
            InsertEdge(edge);
        }
        return;
    }
    
    void ReplaceEdge(Edge oldEdge, Edge newEdge) {
        DeleteEdge(oldEdge);
        InsertEdge(newEdge);
        return;
    }
    
    void DeleteNode(Node* node) {
    	std::vector<Node*> inputs = std::vector<Node*>(node->inputs);
        for (Node* input: inputs) {
            DeleteEdge({input, node});
        }
        std::vector<Node*> outputs = std::vector<Node*>(node->outputs);
        for (Node* output: outputs) {
            DeleteEdge({node, output});
        }
        delete node;
        return;
    }
    
    Node* ReplaceNode(Node* oldNode, Node* newNode) {
        for (Node* node: oldNode->outputs) {
        	InsertEdge({newNode, node});
        }
        DeleteNode(oldNode);
        return ElemenateInversions(newNode);
    }

    //
    
    // Axiom transformations
    Node* Commutativity(Node* node) {
        if (node->GetName() == "mig") {
            int i = rand() % 3;
            int j = (i + 1) % 3;
            std::swap(node->inputs[i], node->inputs[j]);
            //SwitchEdges({node->inputs[i], node}, {node->inputs[j], node});
        }
        return node;
    }
    
    
    Node* Associativity(Node* node) {
        if (node->GetName() == "mig" &&
            node->inputs[2]->GetName() == "mig" &&
            node->inputs[1] == node->inputs[2]->inputs[1]) {
            SwitchEdges({node->inputs[0], node}, {node->inputs[2]->inputs[2], node->inputs[2]});
        }
        return node;
    }
    
    Node* ComplementaryAssociativity(Node* node) {
        if (node->GetName() == "mig" &&
            node->inputs[2]->GetName() == "mig" &&
            node->inputs[2]->inputs[1]->GetName() == "inv" &&
            node->inputs[1] == node->inputs[2]->inputs[1]->inputs[0]) {
            ReplaceEdge({node->inputs[2]->inputs[1], node->inputs[2]},
                        {node->inputs[0], node->inputs[2]});
        }
        return CheckNodeValidity(node);
    }
    
    
    Node* Relevance(Node* node) {
        //TODO
        return node;
    }
    
    Node* Substitution(Node* node) {
        // TODO
        return node;
    }
    
    Node* Majority(Node* node) {
        if (node->GetName() == "mig") {
            if (node->inputs[0] == node->inputs[1]) {
                return ReplaceNode(node, node->inputs[0]);
            } else if (node->inputs[1]->GetName() == "inv" &&
                       node->inputs[0] == node->inputs[1]->inputs[0]) {
                return ReplaceNode(node, node->inputs[2]);
            } else if (node->inputs[0]->GetName() == "inv" &&
                        node->inputs[1]->GetName() == "inv" &&
                        node->inputs[0]->inputs[0] == node->inputs[1]->inputs[0]) {
                return ReplaceNode(node, node->inputs[0]);          
            }
        }
        return ElemenateInversions(node);
    }
    
    Node* DistributivityRL(Node* node) {
        if (node->GetName() == "mig" &&
            node->inputs[0]->GetName() == "mig" &&
            node->inputs[1]->GetName() == "mig" &&
            node->inputs[0]->inputs[0] == node->inputs[1]->inputs[0] &&
            node->inputs[0]->inputs[1] == node->inputs[1]->inputs[1]) {

            Node* x = node->inputs[0]->inputs[0];
            Node* y = node->inputs[0]->inputs[1];
            Node* z = node->inputs[2];
            Node* u = node->inputs[0]->inputs[2];
            Node* v = node->inputs[1]->inputs[2];

            Node* newZ = new Node(migFunction);
            InsertEdges({{u, newZ},{v, newZ},{z, newZ}});
            
            Node* newNode = new Node(migFunction);
            InsertEdges({{x, newNode},{y, newNode},{newZ, newNode}});
            
            return ReplaceNode(node, newNode);
        }
        return node;
    }
    
    Node* DistributivityLR(Node* node) {
        if (node->GetName() == "mig" &&
            node->inputs[2]->GetName() == "mig") {
            Node* x = node->inputs[0];
            Node* y = node->inputs[1];
            Node* u = node->inputs[2]->inputs[0];
            Node* v = node->inputs[2]->inputs[1];
            Node* z = node->inputs[2]->inputs[2];
            
            Node* newX = new Node(migFunction);
            InsertEdges({{x, newX}, {y, newX}, {u, newX}});
            Node* newY = new Node(migFunction);
            InsertEdges({{x, newY}, {y, newY}, {v, newY}});
            Node* newNode = new Node(migFunction);
            InsertEdges({{newX, newNode}, {newY, newNode}, {z, newNode}});

            return ReplaceNode(node, newNode);
        }
        return node;
    }

    Node* ElemenateInversions(Node* node) {
        bool optimized = false;
        while (!optimized) {
            optimized = true;
            for (Node* parent: GetAllParents(node)) {
                bool isNode = parent == node;
                Node* oldNode = parent;
                
                if (parent->GetName() == "inv" &&
                    parent->inputs[0]->GetName() == "inv") {
                        parent = ReplaceNode(parent, parent->inputs[0]->inputs[0]);
                }
                
                if (oldNode != parent) {
                    optimized = false;
                    if (isNode) {
                        node = parent;
                    }
                    break;
                }
            }
        }
        return node;
    }

    Node* ElemenateConstNodes(Node* node) {
        
        bool optimized = false;
        while (!optimized) {
            optimized = true;
            for (Node* parent: GetAllParents(node)) {
                if (parent->GetName() == "mig") {
                    bool isNode = parent == node;
                    Node* oldNode = parent;
                    for (int i = 0; i < 10; ++i) {
                        if (parent->inputs[0]->GetName() == "constTrue" &&
                            parent->inputs[1]->GetName() == "constTrue") {
                                parent = ReplaceNode(parent, parent->inputs[0]);
                                break;
                        } else if (parent->inputs[0]->GetName() == "constFalse" &&
                                    parent->inputs[1]->GetName() == "constFalse") {
                                parent = ReplaceNode(parent, parent->inputs[0]);
                                break;
                        } else if (parent->inputs[0]->GetName() == "constTrue" &&
                                    parent->inputs[1]->GetName() == "constFalse") {
                                parent = ReplaceNode(parent, parent->inputs[2]);
                                break;
                        } else if (parent->inputs[0]->GetName() == "constFalse" &&
                                    parent->inputs[1]->GetName() == "constTrue") {
                                parent = ReplaceNode(parent, parent->inputs[2]);
                                break;
                        }
                        Commutativity(parent);
                    }
                    if (oldNode != parent) {
                        optimized = false;
                        if (isNode) {
                            node = parent;
                        }
                        break;
                    }
                }
            }
        }
        return CheckNodeValidity(node);
    }

    std::vector<Node*> findCriticalVoters(Node* node) {
        std::unordered_map<Node*, double> voters;
        TopologicalSorter topologicalSorter;
        dfs(node, topologicalSorter);
        while (!topologicalSorter.stack.empty()) {
            double criticality = 0;
            Node* curNode = topologicalSorter.stack.top();
            for (Node* output: curNode->outputs) {
                if (output->GetName() == "mig") {
                    criticality += (1. + voters[output]) / 3;
                } else {
                    for (Node* output2: output->outputs) {
                        if (output2->GetName() == "mig") {
                            criticality += (1. + voters[output]) / 3;
                        }
                    }
                }
            }
            voters[curNode] = criticality;
            topologicalSorter.stack.pop();
        }
        std::vector<std::pair<double, Node*>> votersVector;
        for (auto el: voters) {
            votersVector.push_back({el.second, el.first});
        }
        std::sort(votersVector.begin(), votersVector.end(), [](std::pair<double, Node*>& a, 
                                                            std::pair<double, Node*>& b){return a.first > b.first;});
        std::vector<Node*> result;
        for (auto voter: votersVector) {
            if (voter.second->GetName() == "input") {
                result.push_back(voter.second);
            }
        }
        return result;
    }
    
    friend class CopyMaker;
};


class CopyMaker : public Visitor {
public:
    void Visit(Node* node) {
        hashCopy[node] = new Node(node->function);
        for (Node* input: node->inputs) {
            MigOptimizer::InsertEdge({hashCopy[input], hashCopy[node]});
        }
        return;
    }

    Node* copy(Node* node) {
        hashCopy.clear();
        dfs(node, *this);
        return getNodeCopy(node);
    }

    Node* getNodeCopy(Node* node) {
        if (hashCopy.find(node) != hashCopy.end()) {
            return hashCopy[node];
        } else {
            return nullptr;
        }
    }

private:
    std::unordered_map<Node*, Node*> hashCopy;
};


class Reshaper : public MigOptimizer {
public:
    Node* Optimize(Node*& node) {
        Commutativity(GetRandomNode(node));
        Associativity(GetRandomNode(node));
        Relevance(GetRandomNode(node));
        Substitution(GetRandomNode(node));
        ComplementaryAssociativity(GetRandomNode(node));
        return node;
    }
};



class DepthAlgOptimizer : public MigOptimizer {
public:
    Node* Optimize(Node*& node) {
        int efforts = GetSize(node) / 2;
        Reshaper reshaper;
        for (int i = 0; i < efforts; ++i) {
            Node* randNode = GetRandomNode(node);
            bool isNode = (randNode == node);

            randNode = CheckNodeValidity(Associativity(DistributivityLR(Majority(randNode))));
            reshaper.Optimize(randNode);
            randNode = CheckNodeValidity(Associativity(DistributivityLR(Majority(randNode))));

            if (isNode) {
            	node = randNode;
            }
        }
        return CheckNodeValidity(node);
    }
};

class SizeAlgOptimizer : public MigOptimizer {
public:
    Node* Optimize(Node*& node) {
        Reshaper reshaper;
        bool optimized = false;
        while (!optimized) {
            optimized = true;
            for (Node* parent: GetAllParents(node)) {
                bool isNode = parent == node;
                Node* oldNode = parent;
                for (int i = 0; i < 30; ++i) {
                    parent = DistributivityRL(Majority(parent));
                    reshaper.Optimize(parent);
                    parent = DistributivityRL(Majority(parent));
                }
                if (oldNode != parent) {
                    optimized = false;
                    if (isNode) {
                        node = parent;
                    }
                    break;
                }
            }
        }
        return ElemenateConstNodes(node);
    }
};


class DepthBoolOptimizer : public MigOptimizer {
public:
    Node* Optimize(Node*& node) {
        //DumpMig(node, std::cout);
        
        auto criticalVoters = findCriticalVoters(node);
        if (criticalVoters.size() < 2) {
            return node;
        }
        Node* nodeA = criticalVoters[0], *nodeB = criticalVoters[1];
        
        std::vector<Node*> samePolatiryNodes;
        for (Node* outA: nodeA->outputs) {
            if (std::find(nodeB->outputs.begin(),
                            nodeB->outputs.end(), outA) != nodeB->outputs.end()) {
                if (outA->GetName() != "inv") {
                    samePolatiryNodes.push_back(outA);
                } else {
                    for (Node* invOut: outA->outputs) {
                        samePolatiryNodes.push_back(invOut);
                    }
                }
            }
        }

        CopyMaker copyMaker;
        // First Error        
        Node* firstError = copyMaker.copy(node);
        Node* copyA = copyMaker.getNodeCopy(nodeA);
        Node* copyB = copyMaker.getNodeCopy(nodeB);

        MIGSynthezator synthezator;
        ReplaceNode(copyA, synthezator.SynthezInv(copyB));


        
        // Second Error
        Node* secondError = copyMaker.copy(node);
        copyA = copyMaker.getNodeCopy(nodeA);
        copyB = copyMaker.getNodeCopy(nodeB);

        std::vector<Node*> samePolatiryNodesCopy;
        for (Node* node_: samePolatiryNodes) {
            if (copyMaker.getNodeCopy(node_)) {
                samePolatiryNodesCopy.push_back(copyMaker.getNodeCopy(node_));
            }
        }

        if (!samePolatiryNodesCopy.empty()) {
            for (int i = 1; i < samePolatiryNodesCopy.size(); ++i) {
                ReplaceNode(samePolatiryNodesCopy[i], samePolatiryNodesCopy[0]);
            }
            ReplaceNode(samePolatiryNodesCopy[0], copyA);
        }

        //DumpMig(secondError, std::cout);
        std::cout << "Second Error Size: " << GetSize(secondError) << "\n";

        // Third error
        Node* thirdError = copyMaker.copy(node);
        copyA = copyMaker.getNodeCopy(nodeA);
        copyB = copyMaker.getNodeCopy(nodeB);

        samePolatiryNodesCopy.clear();
        for (Node* node_: samePolatiryNodes) {
            if (copyMaker.getNodeCopy(node_)) {
                samePolatiryNodesCopy.push_back(copyMaker.getNodeCopy(node_));
            }
        }

        if (!samePolatiryNodesCopy.empty()) {
            for (int i = 1; i < samePolatiryNodesCopy.size(); ++i) {
                ReplaceNode(samePolatiryNodesCopy[i], samePolatiryNodesCopy[0]);
            }
            ReplaceNode(samePolatiryNodesCopy[0], copyB);
        }
        std::cout << "Third Error Size: " << GetSize(thirdError) << "\n";

        SizeAlgOptimizer sizeAlgOptimizer;
        firstError = sizeAlgOptimizer.Optimize(firstError);
        secondError = sizeAlgOptimizer.Optimize(secondError);
        //DumpMig(secondError, std::cout);
        std::cout << "Second Error Size: " << GetSize(secondError) << "\n";
        thirdError = sizeAlgOptimizer.Optimize(thirdError);
        std::cout << "Third Error Size: " << GetSize(thirdError) << "\n";


        Node* result = synthezator.SynthezCustomNode({firstError, secondError, thirdError}, migFunction);

        if (GetDepth(result) < GetDepth(node)) {
            delete node;
            return result;
        } else {
            //delete result;
            return node;
        }
    }
};


class SizeBoolOptimizer : public MigOptimizer {
public:
    Node* Optimize(Node*& node) {
        auto criticalVoters = findCriticalVoters(node);
        if (criticalVoters.size() < 3) {
            return node;
        }
        Node* nodeA = criticalVoters[0], *nodeB = criticalVoters[1], *nodeC = criticalVoters[2];
        //DumpMig(node, std::cout);

        CopyMaker copyMaker;
        // First Error        
        Node* firstError = copyMaker.copy(node);
        Node* copyA = copyMaker.getNodeCopy(nodeA);
        Node* copyB = copyMaker.getNodeCopy(nodeB);
        Node* copyC = copyMaker.getNodeCopy(nodeC);

        ReplaceNode(copyA, copyB);

        // Second Error        
        Node* secondError = copyMaker.copy(node);
        copyA = copyMaker.getNodeCopy(nodeA);
        copyB = copyMaker.getNodeCopy(nodeB);
        copyC = copyMaker.getNodeCopy(nodeC);

        MIGSynthezator synthezator;
        ReplaceNode(copyB, copyA);
        ReplaceNode(copyA, synthezator.SynthezInv(copyC));

        // Third error
        Node* thirdError = copyMaker.copy(node);
        copyA = copyMaker.getNodeCopy(nodeA);
        copyB = copyMaker.getNodeCopy(nodeB);
        copyC = copyMaker.getNodeCopy(nodeC);

        ReplaceNode(copyB, copyA);
        ReplaceNode(copyC, copyA);

        //DumpMig(firstError, std::cout);
        SizeAlgOptimizer sizeAlgOptimizer;
        firstError = sizeAlgOptimizer.Optimize(firstError);
        //DumpMig(firstError, std::cout);
        secondError = sizeAlgOptimizer.Optimize(secondError);
        thirdError = sizeAlgOptimizer.Optimize(thirdError);


        Node* result = synthezator.SynthezCustomNode({firstError, secondError, thirdError}, migFunction);

        std::cout << "Result Depth: " << GetDepth(result) << "\n";
        std::cout << "Result Size: " << GetSize(result) << "\n";
        if (GetDepth(result) < GetDepth(node)) {
            //delete node;
            return result;
        } else {
            //delete result;
            return node;
        }
    }
};


class TopLevelMigOptimizer : public MigOptimizer {
public:
    Node* Optimize(Node*& node) {
        Reshaper reshaper;
        DepthAlgOptimizer depthAlgOptimizer;
        SizeAlgOptimizer sizeAlgOptimizer;
        DepthBoolOptimizer depthBoolOptimizer;
        SizeBoolOptimizer sizeBoolOptimizer;
        
        //depthAlgOptimizer.Optimize(node);
        reshaper.Optimize(node);
        node = sizeAlgOptimizer.Optimize(node);
        //node = depthBoolOptimizer.Optimize(node);
        node = sizeBoolOptimizer.Optimize(node);
        node = reshaper.Optimize(node);
        //depthAlgOptimizer.Optimize(node);
        //sizeBoolOptimizer.Optimize(node);
        node = sizeAlgOptimizer.Optimize(node);
        node = reshaper.Optimize(node);
        //depthAlgOptimizer.Optimize(node);
        node = sizeAlgOptimizer.Optimize(node);

        return node;
    }
};


Node* CheckNodeValidity(Node* node) {
    ValidityChecker checker;
    bfs(node, checker);
    return node;
}

void DumpMig(Node* node, std::ostream& stream=std::cout) {
	Dumper dumper(stream);
    stream << "digraph MIG {\n";
	bfs(node, dumper);
    stream << "}\n";
	return;
}

void CheckEqualNodes(Node* first, Node* second, int size) {
    std::vector<bool> input;
    for (int i = 0; i < std::pow(2, size); ++i) {
        input.clear();
        for (int j = 0; j < size; ++j) {
            input.push_back(bool((1 << j) & i));
        }
        assert(first->Compute(input) == second->Compute(input));
        std::cout << i << " passed\n";
    }
}



// TEST

int main() {
	MIGSynthezator mig =  MIGSynthezator();
    Node* testNode = mig.Synthez({true, false, true, true,
                                false, true, false, false });
    
    /*
    assert(testNode->Compute({false, true}));
    assert(!testNode->Compute({true, false}));
    assert(!testNode->Compute({true, true}));
    assert(testNode->Compute({false, false}));
    */
    
    DumpMig(testNode, std::cout);
    CopyMaker copyMaker;
    Node* oldNode = copyMaker.copy(testNode);
    

    TopLevelMigOptimizer optimizer = TopLevelMigOptimizer();
    std::cout << "old size: " << optimizer.GetSize(testNode) << "\n";
    std::cout << "old depth: " << optimizer.GetDepth(testNode) << "\n";
    optimizer.Optimize(testNode);
    std::cout << "new size: " << optimizer.GetSize(testNode) << "\n";
    std::cout << "new depth: " << optimizer.GetDepth(testNode) << "\n";

    DumpMig(testNode, std::cout);
    CheckEqualNodes(oldNode, testNode, 3);

     
	return 0;
}