#include <graph.cc>
#include <gtest/gtest.h>

TEST(graph_test,add_vertex){
    Graph<int,double> graph;
    graph.add_vertex(3);
    ASSERT_TRUE(graph.has_vertex(3));
}
