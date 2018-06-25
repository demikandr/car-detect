//
// Created by demikandr on 6/24/18.
//

#ifndef PROJECT_GRAPH_H
#define PROJECT_GRAPH_H


class Graph {
private:
    TVector<TVector<int>> Edges;
public:

    int getNumNodes() const {
        return Edges.size();
    }

    Graph(int numNodes): Edges(numNodes) {}
    Clusterization getClusterization();

    static Graph makeGraph(const CloudMask& mask, const std::function<bool(int, int, int, int)>& edgeExists);
};


#endif //PROJECT_GRAPH_H
