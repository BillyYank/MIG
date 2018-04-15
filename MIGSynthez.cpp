#include<iostream>
#include<vector>
#include<assert.h>
#include <math.h>
#include <functional>


bool NotXor(bool x, bool y) {
    return (x && y) || (!x && !y);
}

bool constTrue(std::vector<bool>& inputs) {
    return true;
}

bool constFasle(std::vector<bool>& inputs) {
    return false;
}

bool migFunction(std::vector<bool>& inputs) {
    int truesNum = 0;
    int falseNum = 0;
    for (bool val: inputs) {
        val ? ++truesNum : ++falseNum;
    }
    return truesNum > falseNum;
}

bool invFunction(std::vector<bool>& inputs) {
    return !inputs[0];
}


// Abstract Scheme
class Node {
public:
    Node(std::function<bool(std::vector<bool>&)> function) : function(function) {}
    
    bool Compute(std::vector<bool> inputVal) {
        SetToZero();
        return Compute_(inputVal);
    }

private:
    std::function<bool(std::vector<bool>&)> function;
	
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
                            std::function<bool(std::vector<bool>&)> function) {
        Node* newNode = new Node(function);
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
            inputs.push_back(new Node([=](std::vector<bool>& inputVals)->bool {
                return inputVals[i];
            }));
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
        Node* falseNode = new Node(constFasle);
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
            return new Node(constFasle);
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
