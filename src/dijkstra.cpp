#include <dijkstra.hpp>

bool is_forbidden_turn(const SimpleOSMCarRoutingGraph &graph, unsigned from_arc, unsigned to_arc)
{
    auto it_from = std::find(graph.forbidden_turn_from_arc.begin(), graph.forbidden_turn_from_arc.end(), from_arc);
    auto it_to = std::find(graph.forbidden_turn_to_arc.begin(), graph.forbidden_turn_to_arc.end(), to_arc);
    if (it_from != graph.forbidden_turn_from_arc.end() && it_to != graph.forbidden_turn_to_arc.end())
    {
        return std::distance(graph.forbidden_turn_from_arc.begin(), it_from) == std::distance(graph.forbidden_turn_to_arc.begin(), it_to);
    }
    return false;
}

std::optional<std::vector<unsigned>> dijkstra(const SimpleOSMCarRoutingGraph &graph, unsigned source, unsigned destination)
{
    const unsigned INF = std::numeric_limits<unsigned>::max();
    std::vector<unsigned> distance(graph.node_count(), INF);
    std::vector<int> predecessor(graph.node_count(), -1);
    std::vector<int> arc_predecessor(graph.node_count(), -1);
    distance[source] = 0;

    std::priority_queue<Edge, std::vector<Edge>, std::greater<Edge>> pq;
    pq.emplace(source, 0, -1);

    while (!pq.empty())
    {
        Edge current = pq.top();
        pq.pop();

        unsigned u = current.node;
        unsigned u_arc_id = current.arc_id;

        if (u == destination)
            break;

        if (current.weight > distance[u])
            continue;

        for (unsigned i = graph.first_out[u]; i < graph.first_out[u + 1]; ++i)
        {
            unsigned v = graph.head[i];
            unsigned weight = graph.travel_time[i];

            if (u_arc_id != -1 && is_forbidden_turn(graph, u_arc_id, i))
                continue;

            if (distance[u] + weight < distance[v])
            {
                distance[v] = distance[u] + weight;
                predecessor[v] = u;
                arc_predecessor[v] = i;
                pq.emplace(v, distance[v], i);
            }
        }
    }

    if (distance[destination] == INF)
        return std::nullopt;

    std::vector<unsigned> path;
    for (int at = destination; at != -1; at = predecessor[at])
    {
        path.push_back(at);
    }
    std::reverse(path.begin(), path.end());
    return path;
}