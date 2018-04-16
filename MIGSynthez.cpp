#include<iostream>
#include<vector>
#include<assert.h>
#include <math.h>
#include <functional>
#include <cstdlib>


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
    friend class Optimizer;
};


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
     Node* Optimize(Node* node) = 0;

private:
    int GetSize(Node* node);
    int GetDepth(Node* node);
    
    Node* GetRandomNode(Node* node);
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
    
private:
    Node* Commutativity(Node* node) {
        if (node->GetName() == "mig") {
            int i = rand() % 3;
            int j = (i + 1) % 3;
            std::swap(node->inputs[i], node->inputs[j]);
        }
        return node;
    }


    Node* Associativity(Node* node) {
        if (node->GetName() == "mig" &&
            node->inputs[2]->GetName() == "mig" &&
            node->inputs[1] == node->inputs[2]->inputs[1]) {
            std::swap(node->inputs[0], node->inputs[2]->inputs[2]);
        }
        return node;
    }
    
    Node* ComplementaryAssociativity(Node* node) {
        if (node->GetName() == "mig" &&
            node->inputs[2]->GetName() == "mig" &&
            node->inputs[2]->inputs[1]->GetName() == "inv" &&
            node->inputs[1] == node->inputs[2]->inputs[1]->inputs[0]) {
            node->inputs[2]->inputs[1] = node->inputs[0];
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
};


class DepthAlgOptimizer : public MigOptimizer {
public:
    Node* Optimize(Node* node);
};


class SizeAlgOptimizer : public MigOptimizer {
public:
    Node* Optimize(Node* node);
};


class DepthBoolOptimizer : public MigOptimizer {
public:
    Node* Optimize(Node* node);
};


class SizeBoolOptimizer : public MigOptimizer {
public:
    Node* Optimize(Node* node);
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

     
	return 0;
}
