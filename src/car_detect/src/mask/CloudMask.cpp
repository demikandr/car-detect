//
// Created by demikandr on 6/23/18.
//

#include "CloudMask.h"

CloudMask CloudMask::getUnionWith(const CloudMask& otherMask) {
    const cloudMask& thisMask= *this;
    assert(thisMask.getNumRows() == otherMask.getNumRows(), "Cannot make union of masks with different shape");
    assert(thisMask.getNumColums() == otherMask.getNumColumns(), "Cannot make union of masks with different shape");
    CloudMask unionMask(getNumRows(), getNumColumns());

    for (int i = 0; i < getNumRows(); ++i) {
        for (int j = 0; j < getNumColumns(); ++j) {
            unionMask[i][j] = std::max(thisMask[i][j], otherMask[i][j]);
        }
    }
    return unionMask;
}