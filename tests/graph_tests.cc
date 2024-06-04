#include <graph.cc>
#include <gtest/gtest.h>

TEST(graph_test,add_vertex){
    Graph<int,double> graph;
    graph.add_vertex(3);
    ASSERT_TRUE(graph.has_vertex(3));
}

TEST(graph_test,delete_vertex){
    Graph<int,double> graph;
    graph.add_vertex(3);
    ASSERT_TRUE(graph.has_vertex(3));
    graph.remove_vertex(3);
    ASSERT_FALSE(graph.has_vertex(3));
}

TEST(graph_test,add_edge){
    Graph<int,double> graph;
    graph.add_vertex(3);
    graph.add_vertex(4);
    graph.add_edge(3,4,6.0);
    ASSERT_TRUE(graph.has_edge(3,4));
    ASSERT_FALSE(graph.has_edge({3,4,5.0}));
}

TEST(graph_test,remove_edge){
    Graph<int,double> graph;
    graph.add_vertex(3);
    graph.add_vertex(4);
    graph.add_vertex(5);
    graph.add_edge(3,4,6.0);
    graph.add_edge(3,5,6.0);
    ASSERT_TRUE(graph.has_edge(3,4));
    ASSERT_TRUE(graph.has_edge({3,5,6.0}));
    ASSERT_FALSE(graph.remove_edge({3,4,5.0}));
    ASSERT_TRUE(graph.remove_edge(3,5));
}