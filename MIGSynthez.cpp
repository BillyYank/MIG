#include<iostream>
#include<vector>
#include<assert.h>
#include <math.h>
#include <functional>
#include <cstdlib>
#include <queue>
#include <unordered_set>


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


// function extantion
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
    friend void bfs(Node* node, Visitor& visitor);
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

void bfs(Node* node, Visitor& visitor) {
    std::unordered_set<Node*> used;
    std::queue<Node*> bfsQueue;
    bfsQueue.push(node);
    while (!bfsQueue.empty()) {
        visitor.Visit(bfsQueue.front());
        for (auto parent: node->inputs) {
            if (used.find(parent) == used.end()) {
                bfsQueue.push(parent);
            }
        }
        used.insert(bfsQueue.front());
        bfsQueue.pop();
    }
    return;
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
        // TODO
        return 0;
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
        auto out1 = std::find(first.in->outputs.begin(), first.in->outputs.end(), first.out);
        auto out2 = std::find(second.in->outputs.begin(), second.in->outputs.end(), second.out);
        auto in1 = std::find(first.out->inputs.begin(), first.out->inputs.end(), first.in);
        auto in2 = std::find(second.out->inputs.begin(), second.out->inputs.end(), second.in);
        std::swap(*out1, *out2);
        std::swap(*in1, *in2);
        return;
    }
    
    void DeleteEdge(Edge edge) {
        auto inPosition = std::find(edge.in->outputs.begin(), edge.in->outputs.end(), edge.out);
        auto outPosition = std::find(edge.out->inputs.begin(), edge.out->inputs.end(), edge.in);
        edge.in->outputs.erase(inPosition);
        edge.out->inputs.erase(outPosition);
        return;
    }
    
    void InsertEdge(Edge edge) {
        edge.in->outputs.push_back(edge.out);
        edge.out->inputs.push_back(edge.in);
        return;
    }
    
    void ReplaceEdge(Edge oldEdge, Edge newEdge) {
        DeleteEdge(oldEdge);
        InsertEdge(newEdge);
        return;
    }
    
    void DeleteNode(Node* node) {
        for (Node* input: node->inputs) {
            DeleteEdge({input, node});
        }
        for (Node* output: node->outputs) {
            DeleteEdge({node, output});
        }
        delete node;
        return;
    }
    
    Node* ReplaceNode(Node* oldNode, Node* newNode) {
        newNode->outputs.insert(newNode->outputs.end(),
                                oldNode->outputs.begin(), oldNode->outputs.end());
        DeleteNode(oldNode);
        return newNode;
    }
    
    // Axiom transformations
    Node* Commutativity(Node* node) {
        if (node->GetName() == "mig") {
            int i = rand() % 3;
            int j = (i + 1) % 3;
            SwitchEdges({node->inputs[i], node}, {node->inputs[j], node});
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
        return node;
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
            }
        }
        return node;
    }
    
    Node* DistributivityRL(Node* node) {
        if (node->GetName() == "mig" &&
            node->inputs[0]->GetName() == "mig" &&
            node->inputs[1]->GetName() == "mig" &&
            node->inputs[0]->inputs[0] == node->inputs[1]->inputs[0] &&
            node->inputs[0]->inputs[1] == node->inputs[1]->inputs[1]) {
            Node* input0 = node->inputs[0];
            Node* input1 = node->inputs[1];
            Node* x = node->inputs[0]->inputs[0];
            Node* y = node->inputs[0]->inputs[1];
            Node* z = node->inputs[2];
            Node* u = node->inputs[0]->inputs[2];
            Node* v = node->inputs[1]->inputs[2];
            Node* newZ = new Node(migFunction);
            InsertEdge({u, newZ});
            InsertEdge({v, newZ});
            InsertEdge({z, newZ});
            ReplaceEdge({input0, node}, {x, node});
            ReplaceEdge({input1, node}, {y, node});
            ReplaceEdge({z, node}, {newZ, node});
        }
        return node;
    }
    
};


class Reshaper : public MigOptimizer {
public:
    Node* Optimize(Node* node) {
        int efforts = GetSize(node) / 2;

        for (int i = 0; i < efforts; ++i) {
            Commutativity(GetRandomNode(node));
            Associativity(GetRandomNode(node));
            Relevance(GetRandomNode(node));
            Substitution(GetRandomNode(node));
            ComplementaryAssociativity(GetRandomNode(node));
        }
        
        return node;
    }
};



class DepthAlgOptimizer : public MigOptimizer {
public:
    Node* Optimize(Node* node) {
        // TODO
        return node;
    }
};


class SizeAlgOptimizer : public MigOptimizer {
public:
    Node* Optimize(Node* node) {
        int efforts = GetSize(node) / 2;
        Reshaper reshaper;
        for (int i = 0; i < efforts; ++i) {
            Node* randNode = GetRandomNode(node);
            Majority(randNode);
            DistributivityRL(randNode);
            reshaper.Optimize(randNode);
            Majority(randNode);
            DistributivityRL(randNode);
        }
        return node;
    }
};


class DepthBoolOptimizer : public MigOptimizer {
public:
    Node* Optimize(Node* node) {
        // TODO
        return node;
    }
};


class SizeBoolOptimizer : public MigOptimizer {
public:
    Node* Optimize(Node* node) {
        // TODO
        return node;
    }
};


class TopLevelMigOptimizer : public MigOptimizer {
public:
    Node* Optimize(Node* node) {
        Reshaper reshaper;
        DepthAlgOptimizer depthAlgOptimizer;
        SizeAlgOptimizer sizeAlgOptimizer;
        DepthBoolOptimizer depthBoolOptimizer;
        SizeBoolOptimizer sizeBoolOptimizer;
        
        depthAlgOptimizer.Optimize(node);
        reshaper.Optimize(node);
        sizeAlgOptimizer.Optimize(node);
        depthBoolOptimizer.Optimize(node);
        reshaper.Optimize(node);
        depthAlgOptimizer.Optimize(node);
        sizeBoolOptimizer.Optimize(node);
        sizeAlgOptimizer.Optimize(node);
        reshaper.Optimize(node);
        depthAlgOptimizer.Optimize(node);
        sizeAlgOptimizer.Optimize(node);
        
        
        return node;
    }
};



// TEST

int main() {
	MIGSynthezator mig =  MIGSynthezator();
    
    Node* testNode = mig.Synthez({true, false, true, false});
    
    assert(testNode->Compute({false, true}));
    assert(!testNode->Compute({true, false}));
    assert(!testNode->Compute({true, true}));
    assert(testNode->Compute({false, false}));
    
    TopLevelMigOptimizer optimizer = TopLevelMigOptimizer();
    std::cout << "old size: " << optimizer.GetSize(testNode) << "\n";
    optimizer.Optimize(testNode);
    std::cout << "new size: " << optimizer.GetSize(testNode) << "\n";

     
	return 0;
}
