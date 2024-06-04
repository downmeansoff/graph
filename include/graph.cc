#include <vector>
#include <unordered_map>
#include <set>
#include <stdexcept>
#include <cmath>
#include <functional>
#include <queue>
#include <limits>
#include <iostream>


template<typename V, typename D = double>
class Graph {
public:
    struct Edge {
        V from;
        V to;
        D distance;

        friend std::ostream &operator<<(std::ostream& os, const Edge &edge) {
            return os << "(" << edge.from << " ," << edge.to << ") = " << edge.distance << "\n";
        }
    };
private:
    std::unordered_map <V, std::vector<Edge>> _edges;
    std::set <V> _vertices;

public:
    bool has_vertex(const V &v) const {
        // Метод проверяет наличие вершины v в графе
        return _vertices.contains(v);
    }

    void add_vertex(const V &v) {
        // Добавляет вершину v в граф
        _vertices.insert(v); // Вставляем вершину в контейнер _vertices
        _edges.insert({v, {}}); // Создаем пустой список ребер для вершины v и вставляем его в контейнер _edges
    }

};