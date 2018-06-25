//
// Created by demikandr on 6/24/18.
//

#ifndef PROJECT_COMMONMASK_H
#define PROJECT_COMMONMASK_H

class TCommonMask: public TPlantsMask, public TGroundMask {
public:
    static TCloudMask getEmptyMaskFor(const TMyPointCloud& cloud) {
        return TCloudMask(cloud.getNumRows(), TVector<int>(cloud.getNumColumns()));
    }
};


#endif //PROJECT_COMMONMASK_H
