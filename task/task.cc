#include <graph.cc>
#include <iostream>
#include <numeric>

template<class V, class D = double>
V find_the_vertex_for_warehouse(const Graph<V, D>& graph) {
    D min_avg_distance = std::numeric_limits<D>::infinity(); // Минимальное среднее расстояние
    V optimal_warehouse; // Оптимальный склад
    bool found = false; // Флаг для обозначения нахождения оптимального склада
    auto vertices = graph.vertices(); // Получаем все вершины графа

    // Перебор всех вершин для поиска оптимального склада
    for (const auto& vertex : vertices) {
        D total_distance = 0; // Общее расстояние от текущего склада до остальных вершин
        bool reachable = true; // Флаг для обозначения достижимости всех вершин из текущего склада

        // Проверка расстояний от текущего склада до остальных вершин
        for (const auto& other : vertices) {
            if (vertex != other) {
                try {
                    // Находим кратчайший путь от текущего склада до другой вершины
                    auto path = graph.shortest_path(vertex, other);
                    // Вычисляем общее расстояние по пути
                    D path_distance = std::accumulate(path.begin(), path.end(), D(0),
                        [](D sum, const auto& e) { return sum + e.distance; });
                    total_distance += path_distance; // Обновляем общее расстояние
                }
                catch (const std::runtime_error&) {
                    reachable = false; // Если путь не найден, вершина недостижима
                    break;
                }

            }
        }

        // Если все вершины достижимы, вычисляем среднее расстояние и обновляем оптимальный склад
        if (reachable) {
            D avg_distance = total_distance / (graph.order() - 1); // Среднее расстояние
            if (avg_distance < min_avg_distance) {
                min_avg_distance = avg_distance; // Обновляем минимальное среднее расстояние
                optimal_warehouse = vertex; // Обновляем оптимальный склад
                found = true; // Устанавливаем флаг нахождения оптимального склада
            }
        }
    }

    // Если оптимальный склад не найден, выбрасываем исключение
    if (!found) throw std::runtime_error("Graph is not fully connected.");
    std::cout << min_avg_distance;
    return optimal_warehouse; // Возвращаем оптимальный склад
}

int main(){
    Graph<int,double> graph;
    graph.add_vertex(1);
    graph.add_vertex(2);
    graph.add_vertex(3);
    graph.add_vertex(4);
    graph.add_edge(1,3,1);
    graph.add_edge(3, 2, 2);
    graph.add_edge(2,4,5);
    graph.add_edge(2,1,4);
    graph.add_edge(4,1,6);
    graph.add_edge(4,3,3);

    graph.walk(1,[](int vert){std::cout<<vert<<" ";});

    std::cout<<std::endl;
    auto res=graph.shortest_path(1,4);
    std::cout<<"\nshortest path from 1 to 4:\n";
    for(const auto& edge:res) std::cout<<edge;
    std::cout<<"vertex_for_warehouse: "<<find_the_vertex_for_warehouse(graph);
    return 0;
}