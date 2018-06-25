//
// Created by demikandr on 6/23/18.
//

#ifndef PROJECT_CLOUDMASK_H
#define PROJECT_CLOUDMASK_H

class CloudMask: public Matrix<int> {
    public:
        CloudMask(int numRows, int numColums): Matrix<int>(numRows, numColumns) {}
        CloudMask getUnionWith(const CloudMask& otherMask) const;
};


#endif //PROJECT_CLOUDMASK_H
