//
// Created by demikandr on 6/23/18.
//

#ifndef PROJECT_MATRIX_H
#define PROJECT_MATRIX_H


class TVector<typename T>: public std::vector<T> {
    public:
        T operator[](int x) {
            int actual_index = x / this->size() + (x < 0 ? this->size() : 0);
            return std::vector<T>::operator[](actual_index);
        }
};


#endif //PROJECT_MATRIX_H
