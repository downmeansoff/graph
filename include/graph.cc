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

    std::vector<Edge> get_incoming_edges(const V &vert) const {
        // Возвращает вектор входящих ребер для заданной вершины vert
        std::vector<Edge> incoming_vert; // Создаем вектор для хранения входящих ребер
        for (const auto &v: _vertices) { // Перебираем все вершины в графе
            for (const auto &edge: _edges.at(v)) { // Перебираем ребра для текущей вершины v из контейнера _edges
                if (edge.to == vert) // Проверяем, является ли вершина "to" текущего ребра равной заданной вершине vert
                    incoming_vert.push_back(edge); // Если да, добавляем это ребро в вектор incoming_vert
            }
        }
        return incoming_vert; // Возвращаем вектор входящих ребер для заданной вершины vert
    }

    bool remove_vertex(const V &vert) {
        // Метод удаляет вершину vert из графа
        if (_vertices.erase(vert)) { // Проверяем, удалось ли удалить вершину vert из контейнера _vertices
            _edges.erase(vert); // Удаляем все ребра, связанные с вершиной vert из контейнера _edges
            for (auto edge: get_incoming_edges(vert)) { // Перебираем входящие ребра для вершины vert
                std::erase_if(_edges[edge.from], [vert](const Edge &edge) {
                    return edge.to == vert;
                }); // Удаляем ребра, указывающие на вершину vert
            }
            return true; // Возвращаем true, если вершина была успешно удалена
        }
        return false; // Возвращаем false, если вершина не была найдена и удалена
    }

    std::vector<V> vertices() const {
        // Метод возвращает вектор всех вершин в графе
        std::vector<V> vertices; // Создаем вектор для хранения вершин
        for (const auto &vert: _vertices)
            vertices.push_back(vert); // Добавляем каждую вершину из контейнера _vertices в вектор vertices
        return vertices; // Возвращаем вектор всех вершин
    }

};