//
// Created by demikandr on 6/24/18.
//

#include "Graph.h"

Clusterization Graph::getClusterization() const {
    std::vector<int> visited(getNumNodes());
    std::vector<int> clusterIndices(getNumNodes);
    for (int nodeIdx = 0; nodeIdx < getNumNodes(); ++nodeIdx) {
        if (!visited[nodeIdx]) {
            runDFS(nodeIdx, visited, clusterIndices);
        }
    }
    return clusterIndices;
}

Graph Graph::makeGraph(const CloudMask& mask, const std::function<bool(int, int, int, int)>& edgeExists) {
    Graph graph(getNumRows(), getNumColumns());

    auto tryAddEdge = [&](rowIdx, columnIdx, otherRowIdx, otherColumnIdx) {
        if ((mask[otherRowIdx][otherColumnIdx])
            && edgeExist(cloud[rowIdx][columnIdx], cloud[otherRowIdx][otherColumnIdx])) {
            graph.addEdge(rowIdx, columnIdx, otherRowIdx, otherColumnIdx);
        }
    };
    for (int rowIdx = 0; rowIdx < cloud.numRows(); ++rowIdx) {
        for (int columnIdx = 0; columnIdx < cloud.numColumns; ++columnIdx) {
            if (mask[rowIdx][columnIdx]) {
                continue;
            }
            if (rowIdx > 0) {
                tryAddEdge(rowIdx, columnIdx, rowIdx - 1, columnIdx);
            }
            if (rowIdx + 1 < cloud.numRows()) {
                tryAddEdge(rowIdx, columnIdx, rowIdx + 1, columnIdx);
            }
            tryAddEdge(rowIdx, columnIdx, rowIdx + 1, columnIdx + 1);
            tryAddEdge(rowIdx, columnIdx, rowIdx + 1, columnIdx + 1);
        }
    }
    return graph;
}
