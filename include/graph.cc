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

    void add_edge(const V &from, const V &to, const D &distance) {
        // Метод добавляет ребро между вершинами from и to с заданным расстоянием distance
        if (!_vertices.contains(from))
            throw std::invalid_argument("from is not exist"); // Проверяем, существует ли вершина from в графе
        if (!_vertices.contains(to))
            throw std::invalid_argument("to is not exist"); // Проверяем, существует ли вершина to в графе
        if (distance < 0)
            throw std::invalid_argument("invalid distance"); // Проверяем, что вес ребра неотприцательный,тк у нас используется алгоритм дейкстры
        _edges[from].push_back({from, to, distance}); // Добавляем ребро в контейнер _edges для вершины from
    }

    bool has_edge(const V &from, const V &to) const {
        // Метод проверяет наличие ребра между вершинами from и to
        if (!_vertices.contains(from) || !_vertices.contains(to))
            return false; // Проверяем, существуют ли вершины from и to в графе
        return std::any_of(_edges.at(from).begin(),
                           _edges.at(from).end(),
                           [to](const Edge &ed) {
                               return ed.to == to;
                           }); // Проверяем, есть ли ребро с вершиной "to" для вершины from
    }

    bool has_edge(const Edge &ed) const {
        // Метод проверяет наличие ребра ed в графе с учетом расстояния
        if (!_vertices.contains(ed.to) || !_vertices.contains(ed.from))
            return false; // Проверяем, существуют ли вершины from и to ребра ed в графе
        return std::any_of(_edges.at(ed.from).begin(),
                           _edges.at(ed.from).end(),
                           [ed](const Edge &edge) {
                               return ed.to == edge.to &&
                                      std::fabs(edge.distance - ed.distance) < std::numeric_limits<double>::epsilon();
                           }); // Проверяем, есть ли ребро ed в контейнере _edges для вершины ed.from
    }

    bool remove_edge(const V &from, const V &to) {
        // Метод удаляет ребро между вершинами from и to
        if (!_edges.contains(from) || !_vertices.contains(to))
            return false; // Проверяем, существуют ли вершины from и to в графе
        auto cnt_erased = 0; // Переменная для подсчета удаленных ребер
        cnt_erased += std::erase_if(_edges[from],
                                    [to](const Edge &edge) {
                                        return edge.to == to;
                                    }); // Удаляем ребро с вершиной "to" из контейнера _edges для вершины from
        if (cnt_erased) return true; // Возвращаем true, если было удалено хотя бы одно ребро
        return false; // Возвращаем false, если ребро не было найдено и удалено
    }

    bool remove_edge(const Edge &ed) {
        // Метод удаляет ребро ed из графа
        if (!_edges.contains(ed.from) || !_vertices.contains(ed.to))
            return false; // Проверяем, существуют ли вершины from и to ребра ed в графе
        auto cnt_erased = 0; // Переменная для подсчета удаленных ребер
        cnt_erased += std::erase_if(_edges[ed.from],
                                    [ed](const Edge &edge) {
                                        return edge.to == ed.to &&
                                               std::fabs(edge.distance - ed.distance) <
                                               std::numeric_limits<double>::epsilon();
                                    }); // Удаляем ребро ed из контейнера _edges для вершины ed.from
        if (cnt_erased) return true; // Возвращаем true, если было удалено хотя бы одно ребро
        return false; // Возвращаем false, если ребро не было найдено и удалено
    }
    std::vector<Edge> edges(const V &vertex) const {
        // Метод возвращает вектор ребер, инцидентных заданной вершине vertex
        if (!has_vertex(vertex)) return {}; // Если вершина не существует, возвращаем пустой вектор
        return _edges.at(vertex); // Возвращаем вектор ребер, инцидентных вершине vertex
    }

    size_t order() const {
        // Метод возвращает количество вершин в графе
        return _edges.size(); // Возвращаем размер контейнера _edges, который представляет количество вершин
    }

    size_t degree(const V &v) const {
        // Метод возвращает степень вершины v в графе
        if (!has_vertex(v)) return 0; // Если вершина не существует, возвращаем 0
        return _edges.at(v).size(); // Возвращаем размер вектора ребер, инцидентных вершине v
    }

    // Обход графа в глубину
    void walk(const V& start_vertex, std::function<void(const V&)> action) const {
        // Метод выполняет обход графа в глубину, начиная с вершины start_vertex и выполняя действие action на каждой вершине
        std::unordered_map<V, bool> visited;
        // visited - хранит информацию о посещенных вершинах
        std::function<void(const V&)> dfs = [&](const V& vertex) {
            // Внутренняя функция dfs для рекурсивного обхода графа
            visited[vertex] = true; // Помечаем текущую вершину как посещенную
            action(vertex); // Выполняем действие на текущей вершине
            for (const Edge& edge : _edges.at(vertex)) {
                // Перебираем ребра, инцидентные текущей вершине
                if (!visited[edge.to]) {
                    // Если смежная вершина еще не посещена
                    dfs(edge.to); // Рекурсивно вызываем обход для смежной вершины
                }
            }
        };
        dfs(start_vertex); // Начинаем обход с вершины start_vertex
    }


    // Поиск кратчайшего пути методом Дейкстры
    std::vector<Edge> shortest_path(const V& start, const V& end) const {
        // Метод находит кратчайший путь между вершинами start и end с использованием алгоритма Дейкстры
        if (!_vertices.contains(start) || !_vertices.contains(end)) throw std::invalid_argument("not found");
        // Проверяем, существуют ли вершины start и end в графе. Если нет, выбрасываем исключение.

        std::unordered_map<V, D> distances;
        // distances - хранит расстояния от начальной вершины до остальных вершин
        for (const auto& vert : _vertices) distances[vert] = std::numeric_limits<D>::infinity();
        // Инициализируем расстояния как бесконечность для всех вершин графа
        distances[start] = 0; // Расстояние от начальной вершины до самой себя равно 0

        std::vector<Edge> path; // Путь от start до end
        std::priority_queue<std::pair<D, V>> priority_queue;
        // priority_queue - очередь с приоритетом для обработки вершин, где первыми обрабатываются вершины с наименьшим расстоянием
        priority_queue.push({ 0, start }); // Добавляем начальную вершину в очередь с расстоянием 0

        std::unordered_map<V, V> prev;
        // prev - хранит предыдущие вершины для восстановления пути
        while (!priority_queue.empty()) {
            std::pair<D, V> current = priority_queue.top();
            priority_queue.pop();

            for (const auto& edge : _edges.at(current.second)) {
                // Перебираем ребра, инцидентные текущей вершине
                D new_dist = current.first + edge.distance;
                // Вычисляем новое расстояние до смежной вершины через текущее ребро
                if (new_dist < distances[edge.to]) {
                    // Если новое расстояние меньше, чем текущее известное расстояние до вершины
                    prev[edge.to] = current.second; // Обновляем предыдущую вершину для восстановления пути
                    distances[edge.to] = new_dist; // Обновляем расстояние до вершины
                    priority_queue.push({ new_dist, edge.to }); // Добавляем вершину в очередь для дальнейшей обработки
                }
            }
        }
        if (distances[end] == std::numeric_limits<D>::infinity()) return {};
        // Если путь до конечной вершины не найден, возвращаем пустой вектор

        V current = end;
        while (current != start) {
            // Восстанавливаем кратчайший путь, начиная с конечной вершины
            V prev_v = prev[current];
            D dist = distances[current] - distances[prev_v];
            path.push_back(Edge{ prev_v, current, dist });
            current = prev_v;
        }
        reverse(path.begin(), path.end()); // Разворачиваем путь, чтобы он шел от start к end
        return path; // Возвращаем кратчайший путь
    }
};