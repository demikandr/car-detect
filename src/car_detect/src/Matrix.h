//
// Created by demikandr on 6/24/18.
//

#ifndef PROJECT_MATRIX_H
#define PROJECT_MATRIX_H

template<typename T>
class Matrix: public TVector<TVector<T>> {
private:
    int numRows, numColumns;

public:
    Matrix(int numRows, int numColums) :
            TVector<TVector<T>>(intRows, TVector<T>(intColums)),
            numRows(numRows),
            numColumns(numColumns) {
        assert(numRows > 0, "Empty CloudMask is not supported");
        assert(numColumns > 0, "Empty CloudMask is not supported");

    }

    int getNumRows() const {
        return numRows;
    }

    int getNumColumns() const {
        return numColumns;
    }
}


#endif //PROJECT_MATRIX_H
