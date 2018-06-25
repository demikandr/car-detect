//
// Created by demikandr on 6/24/18.
//

#ifndef PROJECT_SEGMENTATION_H
#define PROJECT_SEGMENTATION_H


class Segmentation: public TMatrix<int> {
    private:
        int numRows, numColumns;
    public:
        Segmentation(int numRows, int numColumns): TMatrix<int>(numRows, numColumns) {}
        static Segmentation getSegmentation(const MyPointCloud& cloud, const ClousMask& mask);
        static Graph makeGraph(const MyPointCloud& cloud, const ClousMask& mask);
};


#endif //PROJECT_SEGMENTATION_H
