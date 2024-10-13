#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>
#include <sstream>  // Для парсинга ввода

namespace Consts {
    uint32_t number_of_kolod = 8;
    uint32_t starting_number_of_cards = 9;
    int32_t big_number = 100;
}

template <typename T>
void Print(T&& container);

// Хэш-функция для вектора векторов
struct VectorHasher {
    std::size_t operator()(const std::vector<std::vector<int32_t>>& vv) const {
        std::size_t seed = 0;
        std::hash<int32_t> hasher;

        for (const auto& v : vv) {
            for (int32_t num : v) {
                // Комбинируем хэши каждого числа
                seed ^= hasher(num) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
        }

        return seed;
    }
};

using CardCode = int32_t;
using State = std::vector<std::vector<CardCode>>;

auto RemoveCombination = [](State& state, uint32_t index) {
    CardCode next = 1;
    // Если куча пуста, просто выходим
    if (state[index].empty()) return;

    // Проверим, что в куче достаточно карт для проверки комбинации
    if (state[index].size() < Consts::starting_number_of_cards) return;

    // Итератор на последнюю карту
    auto it = state[index].end() - 1;

    // Проверяем наличие комбинации от 1 до 9
    for (uint32_t i = 0; i < Consts::starting_number_of_cards; ++i) {
        if (*it != next) break;
        ++next;
        if (it == state[index].begin()) break;
        --it;
    }

    // Если найдена комбинация, удаляем её
    if (next == Consts::starting_number_of_cards + 1) {
        // Удаляем последние 9 карт
        for (uint32_t i = 0; i < Consts::starting_number_of_cards; ++i) {
            state[index].pop_back();
        }
    }
};

// Функция для генерации новых состояний
std::vector<State> GenerateStates(State& state) {
    std::vector<State> res;
    std::vector<std::pair<CardCode, uint32_t>> tops_of_kuchas;
    uint32_t i = 0;

    // Генерация массива с верхними картами кучек
    for (auto& kucha : state) {
        if (kucha.empty()) {
            tops_of_kuchas.push_back({-1, i});
        } else {
            tops_of_kuchas.push_back({*(kucha.end() - 1), i});
        }
        ++i;
    }

    std::sort(tops_of_kuchas.begin(), tops_of_kuchas.end());

    for (auto& [value, index] : tops_of_kuchas) {
        std::vector<std::pair<CardCode, uint32_t>> suitable_cards;
        for (auto& [value2, index2] : tops_of_kuchas) {
            if (value2 >= value) break;
            suitable_cards.push_back({value2, index2});
        }

        // Генерация новых состояний
        for (auto& pair : suitable_cards) {
            if (pair.first == -1) continue;
            State new_state = state;
            new_state[index].push_back(pair.first);
            RemoveCombination(new_state, index);

            new_state[pair.second].pop_back();
            res.push_back(new_state);
        }
    }
    return res;
}

template <typename Vertex>
void Astar(const Vertex& vertex) {
    Vertex termination_state;
    for (uint32_t i = 0; i < Consts::number_of_kolod; ++i) {
        termination_state.push_back(std::vector<CardCode>{Consts::big_number});
    }

    std::unordered_map<Vertex, uint64_t, VectorHasher> dist;
    std::unordered_map<Vertex, uint64_t, VectorHasher> heuristic;
    std::unordered_map<Vertex, Vertex, VectorHasher> parent;

    auto CalculateH = []<typename T>(T& state) {
        int64_t res = 0;
        for (auto& kucha : state) {
            res += kucha.size();
        }
        return res;
    };

    auto function = [&dist, &heuristic, &CalculateH]<typename T>(T& state) {
        return dist[state] + heuristic[state];
    };

    dist[vertex] = 0;
    heuristic[vertex] = CalculateH(vertex);
    std::priority_queue<std::pair<int64_t, Vertex>> queue;
    queue.push({function(vertex), vertex});
    while (!queue.empty()) {
        auto cur_state = queue.top().second;
        auto cur_d = -queue.top().first;
        queue.pop();

        auto derived_states = GenerateStates(cur_state);

        for (auto& state : derived_states) {
            int64_t found_g = dist[cur_state] + 1;
            bool update_g = true;
            if (!dist.contains(state)) {
                heuristic[state] = CalculateH(state);
                queue.push({-found_g, state});
            } else {
                update_g = (found_g < dist[state]);
            }

            if (update_g) {
                dist[state] = found_g;
                parent[state] = cur_state;
            }
        }
    }

    if (!dist.contains(termination_state)) {
        std::cout << "Unsolvable";
        return;
    }
    std::cout << "Minimal number of moves: " << dist[termination_state] << "\n";
};

template <typename Cont>
void Print(Cont&& container) {
    for (auto& elem : container) {
        std::cout << elem;
    }
}

// Функция для чтения стартового состояния из ввода
State ReadInitialState() {
    State starting_state;
    std::string line;

    for (uint32_t i = 0; i < Consts::number_of_kolod; ++i) {
        std::getline(std::cin, line);
        std::istringstream iss(line);
        std::vector<CardCode> kucha;
        CardCode card;

        while (iss >> card) {
            kucha.push_back(card);
        }

        // Проверка, что ровно `starting_number_of_cards` карт
        if (kucha.size() != Consts::starting_number_of_cards) {
            std::cerr << "Error: Each row must contain exactly " << Consts::starting_number_of_cards << " cards.\n";
            exit(1);
        }

        starting_state.push_back(kucha);
    }

    return starting_state;
}

int main() {
    // Считываем стартовое состояние
    State starting_state = ReadInitialState();

    // Удаление комбинаций в каждой куче, если они есть
    for (uint32_t i = 0; i < Consts::number_of_kolod; ++i) {
        RemoveCombination(starting_state, i);
    }
    
    // Запуск алгоритма A*
    Astar(starting_state);

    return 0;
}

