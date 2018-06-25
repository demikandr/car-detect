//
// Created by demikandr on 6/24/18.
//

#include "Segmentation.h"

Segmentation Segmentation::getSegmentation(const MyPointCloud& cloud, const ClousMask& mask) {
    Graph graph = Segmentation::makeGraph(cloud, mask);
    Segmentation segmentation = graph.getSegmentation();
    return segmentation;
}

Graph Segmentation::makeGraph(ConfigWrapper config, const MyPointCloud& cloud, const ClousMask& mask) {
    assert(cloud.getNumRows() == mask.getNumRows());
    assert(cloud.getNumColumns() == mask.getNumColumns());
    return makeGraph(mask, std::bind(&Segmentation::edgeExists, config, cloud, _1, _2, _3, _4));
}