#include <vector>

// Выбор случайного подмножества, понадобится для генерации случайного графа
template <class C>
std::vector<C> getSample(std::vector<C> input, int size) {
    std::vector<C> res;
    for (int i = 0; i < size; ++i) {
        res.push_back(input[rand() % input.size()]);
    }
    return res;
}