//
// Created by bpiranda on 22/11/2019.
//

#ifndef GEOMETRY_TRIANGULATION_GL_DETERMINANT_H
#define GEOMETRY_TRIANGULATION_GL_DETERMINANT_H

class Matrix33;
class Matrix44;

class Matrix22 {
public:
    float m[2][2];

    void get2x2From3x3(const Matrix33 &mat33,int shadowLin, int shadowCol);
    inline float determinant() {
        return m[0][0]*m[1][1]-m[0][1]*m[1][0];
    }
};

class Matrix33 {
public:
    float m[3][3];

    void get3x3From4x4(const Matrix44 &mat44,int shadowLin, int shadowCol);
    float determinant();
};

class Matrix44 {
public:
    float m[4][4];

    float determinant();
};





#endif //GEOMETRY_TRIANGULATION_GL_DETERMINANT_H
