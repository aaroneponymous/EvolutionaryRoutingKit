#ifndef DIJKSTRA_HPP
#define DIJKSTRA_HPP

#include <routingkit/osm_simple.h>
#include <optional>
#include <algorithm>
#include <vector>
#include <queue>
#include <limits>

using namespace RoutingKit;

struct Edge
{
    unsigned node;
    unsigned weight;
    unsigned arc_id;

    Edge(unsigned n, unsigned w, unsigned a) : node(n), weight(w), arc_id(a) {}

    bool operator>(const Edge &other) const
    {
        return weight > other.weight;
    }
};

bool is_forbidden_turn(const SimpleOSMCarRoutingGraph &graph, unsigned from_arc, unsigned to_arc);

std::optional<std::vector<unsigned>> dijkstra(const SimpleOSMCarRoutingGraph &graph, unsigned source, unsigned destination);

#endif // DIJKSTRA_HPP