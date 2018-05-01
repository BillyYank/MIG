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
#include "utils.cpp"


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
void dfs(Node* node, Visitor& visitor);

class Visitor {
public:
    virtual Node* Visit(Node* node) = 0;
};


// Abstract Scheme
class Node {
public:
    Node(NamedFunction function) : function(function) {}
    
    bool Compute(std::vector<bool> inputVal) {
        ComputeVisitor computeVisitor(inputVal);
        dfs(this, computeVisitor);
        return computeVisitor.Compute(this);
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

    bool HasInputs(std::vector<std::pair<Node*,bool>> nodes) {
        for (auto node: nodes) {
            bool hasNode = false;
            for (Node* input: inputs) {
                if (input == node.first || (node.second == false && input->GetName() == "inv" &&
                            input->inputs[0] == node.first)) {
                    hasNode = true;
                    break;
                }
            }
            if (!hasNode) {
                return false;
            }
        }
        return true;
    }

private:
    NamedFunction function;
	
    bool val;
    bool computed;
    
	std::vector<Node*> inputs;
	std::vector<Node*> outputs;

    class ComputeVisitor : public Visitor {
    public:
        ComputeVisitor(std::vector<bool>& inputValues) : inputValues(inputValues) {}

        Node* Visit(Node* node) {
            std::vector<bool> parentValues;
            for (Node* parent: node->inputs) {
                parentValues.push_back(values[parent]);
            }
            if (parentValues.empty()) {
                values[node] = node->function(inputValues);
            } else {
                values[node] = node->function(parentValues);
            }
            node->val = values[node];
            return node;
        }

        bool Compute(Node* node) {
            return values[node];
        }
    private:
        std::unordered_map<Node*, bool> values;
        std::vector<bool> inputValues;
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




class MigSizeCountor : public Visitor {
public:
    MigSizeCountor() : count(0) {}

    Node* Visit(Node* node) {
        if (node->GetName() == "mig") {
            ++count;
        }
        return node;
    }
    
    int Count() {
        return count;
    }
private:
    int count;
};

class Dumper : public Visitor {
public:
	Dumper(std::ostream& stream) : stream(stream) {}

	Node* Visit(Node* node) {
        if (node->GetName() == "input") {
            stream << "Node" << node << "_" << node->val << " " << "[color=green];\n";
        } else if (node->GetName() == "inv") {
            stream << "Node" << node << "_" << node->val << " " << "[color=blue];\n";
        } else if (node->GetName() == "mig") {
		    stream << "Node" << node << "_" << node->val << " " << "[color=red];\n";
        } else {
            stream << "Node" << node << "_" << node->val << " " << "[color=gray];\n";
        }

		for (Node* input: node->inputs) {
			stream << "Node" << input << "_" << input->val << " -> Node" << node << "_" << node->val << ";\n";
		}
        return node;
	}

private:
	std::ostream& stream;
};

class MigDepthCountor : public Visitor {
public:
    Node* Visit(Node* node) {
        int depth = 0;
        for (auto parent: node->inputs) {
            depth = std::max(depth, depthes[parent]);
        }
        if (node->GetName() == "mig") {
            ++depth;
        }
        depthes[node] = depth;
        return node;
    }

    int GetDepth(Node* node) {
        return depthes[node];
    }

private:
    std::unordered_map<Node*, int> depthes;
};

class TopologicalSorter : public Visitor {
public:
    Node* Visit(Node* node) {
        stack.push(node);
        return node;
    }
private:
    std::stack<Node*> stack;

    friend class MigOptimizer;
};

class NodeToVector : public Visitor {
public:
    Node* Visit(Node* node) {
        nodesVector.push_back(node);
        return node;
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
    static void ConnectNodes(Node* in, Node* out) {
        (in->outputs).push_back(out);
        (out->inputs).push_back(in);
        return;
    }
    
    static Node* SynthezCustomNode(std::vector<Node*> inputs,
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

    static Node* SynthezRandom(int inputsNum) {
        std::vector<Node*> prev_layer;
        for (int i = 0; i < inputsNum; ++i) {
            prev_layer.push_back(new Node({[=](std::vector<bool>& inputVals)->bool {
                return inputVals[i];
            }, "input" }));
        }

        do {
            std::vector<Node*> layer;
            for (int i = 0; i < prev_layer.size() / 2; ++i) {
                std::vector<Node*> sample = getSample(prev_layer, 3);
                layer.push_back(SynthezRandomMig(sample));
            }
            prev_layer = layer;
        } while (prev_layer.size() > 1);
        return prev_layer[0];
    }

private:
    static Node* SynthezInv(Node* node) {
        return SynthezCustomNode({node}, invFunction);
    }
    
    
    static Node* SynthezOR(Node* node1, Node* node2) {
        Node* trueNode = new Node(constTrue);
        return SynthezCustomNode({node1, node2, trueNode}, migFunction);
    }
    
    static Node* SynthezAND(Node* node1, Node* node2) {
        Node* falseNode = new Node(constFalse);
        return SynthezCustomNode({node1, node2, falseNode}, migFunction);
    }
    
    static Node* SynthezConj(std::vector<Node*>& inputs, int num) {
        bool firstBit = bool((1 << 0) & num);
        std::vector<Node*> nodes;
        nodes.push_back(firstBit ? inputs[0] : SynthezInv(inputs[0]));
        
        for (int i = 1; i < inputs.size(); ++i) {
            bool bit = bool((1 << i) & num);
            nodes.push_back(SynthezAND(nodes.back(), bit ? inputs[i] : SynthezInv(inputs[i])));
        }
        
        return nodes.back();
    }
    
    static Node* SynthezDisj(std::vector<Node*>& inputs) {
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

    static Node* SynthezRandomMig(std::vector<Node*>& sample) {
        Node* mig = new Node(migFunction);
        for (auto node: sample) {
            if (rand() % 2) {
                ConnectNodes(node, mig);
            } else {
                ConnectNodes(SynthezInv(node), mig);
            }
        }
        return mig;
    }

    friend class DepthBoolOptimizer;
    friend class SizeBoolOptimizer;
};


// Optimizer

class MigOptimizer {
public:
    virtual
    
    Node* Optimize(Node* node) = 0;
    
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

    static std::vector<Node*> GetAllParents(Node* node) {
        NodeToVector nodeToVector;
        bfs(node, nodeToVector);
        return nodeToVector.nodesVector;
    }

protected:
    // depricated
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
    
    static void SwitchEdges(Edge first, Edge second) {
    	DeleteEdge(first);
    	DeleteEdge(second);
    	InsertEdge({first.in, second.out});
    	InsertEdge({second.in, first.out});
        return;
    }
    
    static void DeleteEdge(Edge edge) {
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
    
    static void ReplaceEdge(Edge oldEdge, Edge newEdge) {
        DeleteEdge(oldEdge);
        InsertEdge(newEdge);
        return;
    }
    
    static void DeleteNode(Node* node) {
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
    
    static Node* ReplaceNode(Node* oldNode, Node* newNode) {
        for (Node* node: oldNode->outputs) {
        	InsertEdge({newNode, node});
        }
        DeleteNode(oldNode);
        return ElemenateInversions(newNode);
    }
    
    // Axiom transformations
    static Node* Commutativity(Node* node) {
        if (node->GetName() == "mig") {
            int i = rand() % 3;
            int j = (i + 1) % 3;
            std::swap(node->inputs[i], node->inputs[j]);
        }
        return node;
    }
    
    static Node* Associativity(Node* node) {
        if (node->GetName() == "mig" &&
            node->inputs[2]->GetName() == "mig" &&
            node->inputs[1] == node->inputs[2]->inputs[1]) {
            SwitchEdges({node->inputs[0], node}, {node->inputs[2]->inputs[2], node->inputs[2]});
        }
        return node;
    }
    
    static Node* ComplementaryAssociativity(Node* node) {
        if (node->GetName() == "mig" &&
            node->inputs[2]->GetName() == "mig" &&
            node->inputs[2]->inputs[1]->GetName() == "inv" &&
            node->inputs[1] == node->inputs[2]->inputs[1]->inputs[0]) {
            ReplaceEdge({node->inputs[2]->inputs[1], node->inputs[2]},
                        {node->inputs[0], node->inputs[2]});
        }
        return node;
    }
    
    static Node* Relevance(Node* node) {
        //TODO
        return node;
    }
    
    static Node* Substitution(Node* node) {
        // TODO
        return node;
    }
    
    static Node* Majority(Node* node) {
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
    
    static Node* DistributivityRL(Node* node) {
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
    
    static Node* DistributivityLR(Node* node) {
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

    static Node* IterateWhileNotOptimized(Node* node, Visitor& visitor) {
        bool optimized = false;
        while (!optimized) {
            optimized = true;
            for (Node* parent: GetAllParents(node)) {
                bool isNode = parent == node;
                Node* oldNode = parent;
                parent = visitor.Visit(parent);
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

    class ElemenateInversionsVisitor : public Visitor {
    public:
        Node* Visit(Node* node) {
            /*
            if (node->GetName() == "inv" &&
                node->inputs[0]->GetName() == "inv") {
                return  ReplaceNode(node, node->inputs[0]->inputs[0]);
            }
            */
            return node;
        }
    };

    static Node* ElemenateInversions(Node* node) {
        ElemenateInversionsVisitor visitor;
        return IterateWhileNotOptimized(node, visitor);
    }

    class ElemenateConstNodesVisitor : public Visitor {
    public:
        Node* Visit(Node* node) {
            if (node->GetName() == "mig") {
                std::sort(node->inputs.begin(), node->inputs.end(), [](Node*& a, Node*& b) {
                    return a->GetName() < b->GetName();
                });
                if ((node->inputs[0]->GetName() == "constTrue" &&
                    node->inputs[1]->GetName() == "constTrue") ||
                    (node->inputs[0]->GetName() == "constFalse" &&
                    node->inputs[1]->GetName() == "constFalse")) {
                        return ReplaceNode(node, node->inputs[0]);
                } else if (node->inputs[0]->GetName() == "constFalse" &&
                        node->inputs[1]->GetName() == "constTrue") {
                        return ReplaceNode(node, node->inputs[2]);
                }
            }
            return node;
        }
    };

    static Node* ElemenateConstNodes(Node* node) {
        ElemenateConstNodesVisitor visitor;
        return IterateWhileNotOptimized(node, visitor);
    }

    static Node* ElemenateTrivials(Node* node) {
        return ElemenateConstNodes(ElemenateInversions(node));
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
    Node* Visit(Node* node) {
        hashCopy[node] = new Node(node->function);
        for (Node* input: node->inputs) {
            MigOptimizer::InsertEdge({hashCopy[input], hashCopy[node]});
        }
        return hashCopy[node];
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
    Node* Optimize(Node* node) {
        node = Commutativity(node);
        node = Associativity(node);
        node = ComplementaryAssociativity(node);
        return node;
    }
};



class DepthAlgOptimizer : public MigOptimizer {
public:
    Node* Optimize(Node* node) {
        // Causes infinite size increase
        /*
        DepthAlgOptimizerVisitor depthAlgOptimizerVisitor;
        return IterateWhileNotOptimized(node, depthAlgOptimizerVisitor);
        */
       return ElemenateTrivials(node);
    }
private:
    class DepthAlgOptimizerVisitor : public Visitor {
    public:
        Node* Visit(Node* node) {
            Reshaper reshaper;
            for (int i = 0; i < 10; ++i) {
                node = Associativity(DistributivityLR(Majority(node)));
                reshaper.Optimize(node);
            }
            return node;
        }
    };
};

class SizeAlgOptimizer : public MigOptimizer {
public:
    Node* Optimize(Node* node) {
        SizeAlgOptimizerVisitor sizeAlgOptimizerVisitor;
        node = IterateWhileNotOptimized(node, sizeAlgOptimizerVisitor);
        return ElemenateTrivials(node);
    }
private:
    class SizeAlgOptimizerVisitor : public Visitor {
    public:
        Node* Visit(Node* node) {
            Reshaper reshaper;
            for (int i = 0; i < 10; ++i) {
                node = DistributivityRL(Majority(node));
                reshaper.Optimize(node);
            }
            return node;
        }
    };
};


class DepthBoolOptimizer : public MigOptimizer {
public:
    Node* Optimize(Node* node) {
        /*
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
            //delete node;
            return result;
        } else {
            //delete result;
            return node;
        }
        */
       return node;
    }
};


class SizeBoolOptimizer : public MigOptimizer {
public:
    Node* Optimize(Node* node) {
        auto criticalVoters = findCriticalVoters(node);
        if (criticalVoters.size() < 3) {
            return node;
        }
        Node* nodeX = criticalVoters[0], *nodeY = criticalVoters[1], *nodeZ = criticalVoters[2];

        CopyMaker copyMaker;
        // First Error        
        Node* firstError = copyMaker.copy(node);
        Node* copyX = copyMaker.getNodeCopy(nodeX);
        Node* copyY = copyMaker.getNodeCopy(nodeY);
        Node* copyZ = copyMaker.getNodeCopy(nodeZ);
        ReplaceNode(copyX, copyY);

        // Second Error        
        Node* secondError = copyMaker.copy(node);
        copyX = copyMaker.getNodeCopy(nodeX);
        copyY = copyMaker.getNodeCopy(nodeY);
        copyZ = copyMaker.getNodeCopy(nodeZ);
        SecondErrorVisitor secondErrorVisitor(nodeX, nodeY, nodeZ);
        secondError = IterateWhileNotOptimized(secondError, secondErrorVisitor);

        // Third error
        Node* thirdError = copyMaker.copy(node);
        copyX = copyMaker.getNodeCopy(nodeX);
        copyY = copyMaker.getNodeCopy(nodeY);
        copyZ = copyMaker.getNodeCopy(nodeZ);
        ThirdErrorVisitor thirdErrorVisitor(nodeX, nodeY, nodeZ);
        thirdError = IterateWhileNotOptimized(thirdError, thirdErrorVisitor);

        SizeAlgOptimizer sizeAlgOptimizer;
        firstError = sizeAlgOptimizer.Optimize(firstError);
        secondError = sizeAlgOptimizer.Optimize(secondError);
        thirdError = sizeAlgOptimizer.Optimize(thirdError);

        Node* result = MIGSynthezator::SynthezCustomNode({firstError, secondError, thirdError}, migFunction);

        if (GetDepth(result) < GetDepth(node)) {
            //delete node;
            return result;
        } else {
            //delete result;
            return node;
        }
    }
private:
    class SecondErrorVisitor : public Visitor {
    public:
        SecondErrorVisitor(Node* nodeX, Node* nodeY, Node* nodeZ) : nodeX(nodeX), nodeY(nodeY), nodeZ(nodeZ) {}

        Node* Visit(Node* node) {
            if (node->HasInputs({{nodeX, 1},{nodeY, 0},{nodeZ, 1}})) {
                return ReplaceNode(node, MIGSynthezator::SynthezInv(nodeY));
            } else if (node->HasInputs({{nodeX, 0},{nodeY, 1},{nodeZ, 0}})) {
                return ReplaceNode(node, nodeY);
            } else {
                return node;
            }
        }
    private:
        Node* nodeX;
        Node* nodeY;
        Node* nodeZ;
    };

    class ThirdErrorVisitor : public Visitor {
    public:
        ThirdErrorVisitor(Node* nodeX, Node* nodeY, Node* nodeZ) : nodeX(nodeX), nodeY(nodeY), nodeZ(nodeZ) {}

        Node* Visit(Node* node) {
            if (node->HasInputs({{nodeX, 1},{nodeY, 0},{nodeZ, 0}})) {
                return ReplaceNode(node, MIGSynthezator::SynthezInv(nodeY));
            } else if (node->HasInputs({{nodeX, 0},{nodeY, 1},{nodeZ, 1}})) {
                return ReplaceNode(node, nodeY);
            } else {
                return node;
            }
        }
    private:
        Node* nodeX;
        Node* nodeY;
        Node* nodeZ;
    };

};


class TopLevelMigOptimizer : public MigOptimizer {
public:
    Node* Optimize(Node* node) {
        std::cout << "INITIAL DEPTH: " << GetDepth(node) << "\n";

        Reshaper reshaper;
        DepthAlgOptimizer depthAlgOptimizer;
        SizeAlgOptimizer sizeAlgOptimizer;
        DepthBoolOptimizer depthBoolOptimizer;
        SizeBoolOptimizer sizeBoolOptimizer;
        
        node = depthAlgOptimizer.Optimize(node);
        node = reshaper.Optimize(node);
        node = sizeAlgOptimizer.Optimize(node);
        node = depthBoolOptimizer.Optimize(node);
        node = sizeBoolOptimizer.Optimize(node);
        node = reshaper.Optimize(node);
        node = depthAlgOptimizer.Optimize(node);
        node = sizeBoolOptimizer.Optimize(node);
        node = sizeAlgOptimizer.Optimize(node);
        node = reshaper.Optimize(node);
        node = depthAlgOptimizer.Optimize(node);
        node = sizeAlgOptimizer.Optimize(node);

        std::cout << "FINAL DEPTH: " << GetDepth(node) << "\n";
        return node;
    }
};

void DumpMig(Node* node, std::ostream& stream=std::cout) {
	Dumper dumper(stream);
    stream << "digraph MIG {\n";
	bfs(node, dumper);
    stream << "}\n";
	return;
}
