#include "MIGSynthez.cpp"
#include <exception>


class ValidityChecker : public Visitor {
public:
    Node* Visit(Node* node) {
        if (node->GetName() == "mig") {
            if (node->inputs.size() != 3) {
                DumpMig(node, std::cout);
                throw std::logic_error("mig inputs not equal 3");
            }
        } else if (node->GetName() == "inv") {
            if (node->inputs.size() != 1) {
                DumpMig(node, std::cout);
                throw std::logic_error("inv node inputs not equals 1");
            }
        }
        for (Node* input: node->inputs) {
            if (std::find(input->outputs.begin(),
                            input->outputs.end(),
                            node) == input->outputs.end()) {
                DumpMig(node, std::cout);
                throw std::logic_error("input not found in outputs");
            };
        }
        for (Node* output: node->outputs) {
            if (std::find(output->inputs.begin(),
                            output->inputs.end(),
                            node) == output->inputs.end()){
                DumpMig(node, std::cout);
                throw std::logic_error("output not found in inputs");
            };
        }
        return node;
    }
};


Node* checkNodeValidity(Node* node) {
    ValidityChecker checker;
    bfs(node, checker);
    return node;
}

void checkEqualNodes(Node* first, Node* second, int size) {
    std::vector<bool> input;
    for (int i = 0; i < std::pow(2, size); ++i) {
        input.clear();
        for (int j = 0; j < size; ++j) {
            input.push_back(bool((1 << j) & i));
        }
        if (first->Compute(input) != second->Compute(input)) {
            throw std::logic_error("unequal Nodes");
        }
    }
}

Node* checkSynthezator(std::vector<bool>& funcTable) {
    std::cout << "checkSynthezator run\n";
    MIGSynthezator mig =  MIGSynthezator();
    Node* node = mig.Synthez(funcTable);
    
    std::vector<bool> input;
    int inputsNum = int(log2(funcTable.size()));
    for (int i = 0; i < funcTable.size(); ++i) {
        input.clear();
        for (int j = 0; j < inputsNum; ++j) {
            input.push_back(bool((1 << j) & i));
        }
        if (node->Compute(input) != funcTable[i]) {
            throw std::logic_error("synthezator fail");
        }
    }
    std::cout << "checkSynthezator passed\n";
    return node;
}

void checkCopyMaker(Node* node, int size) {
    std::cout << "copyMaker run\n";
    CopyMaker copyMaker;
    Node* nodeCopy = copyMaker.copy(node);
    try {
        checkEqualNodes(nodeCopy, node, size);
    } catch (std::logic_error e) {
        std::cout << "Copy Maker fail\n";
        DumpMig(node, std::cout);
        DumpMig(nodeCopy, std::cout);
        assert(false);
    }
    std::cout << "copyMaker passed\n";
    return;
}

struct NamedOptimizer {
    MigOptimizer* optimizer;
    std::string name;
};

void checkOptimizers(Node* node, int size) {
    std::vector<NamedOptimizer> optimizers = {
                                    {new Reshaper(), "Reshaper"},
                                    {new DepthAlgOptimizer(), "DepthAlgOptimizer"},
                                    {new SizeAlgOptimizer(), "SizeAlgOptimizer"},
                                    {new DepthBoolOptimizer(), "DepthBoolOptimizer"},
                                    {new SizeBoolOptimizer(), "SizeBoolOptimizer"},
                                    {new TopLevelMigOptimizer(), "TopLevelMigOptimizer"},
                                };

    CopyMaker copyMaker;
    for (auto& optimizer: optimizers) {
        std::cout << optimizer.name << " run\n";
        Node* nodeCopy = copyMaker.copy(node);
        nodeCopy = optimizer.optimizer->Optimize(nodeCopy);
        try {
            checkNodeValidity(nodeCopy);
            checkEqualNodes(nodeCopy, node, size);
        } catch (std::logic_error e) {
            std::cout << optimizer.name << " failed\n";
            std::cout << e.what() << "\n";
            DumpMig(node, std::cout);
            DumpMig(nodeCopy, std::cout);
            assert(false);
            return;
        }
        std::cout << optimizer.name << " passed\n";
    }
    
}


void run_test(std::vector<bool>& funcTable) {
    Node* node = checkSynthezator(funcTable);
    int size = int(log2(funcTable.size()));
    checkCopyMaker(node, size);
    checkOptimizers(node, size);
    return;
}

void run_stress_test(int size, int efforts) {
    std::vector<bool> funcTable;
    for (int effort = 0; effort < efforts; ++effort) {
        std::cout << "Test " << effort << "\n";
        funcTable.clear();
        for (int i = 0; i < std::pow(2, size); ++i) {
            funcTable.push_back(rand() % 2);
        }
        run_test(funcTable);
    }
    return;
}


int main () {
    run_stress_test(7, 100);
    return 0;
}