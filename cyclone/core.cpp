// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "core.h"

using namespace cyclone;

const Vector3 Vector3::GRAVITY			= {0,  -9.81, 0};
const Vector3 Vector3::HIGH_GRAVITY		= {0, -19.62, 0};
const Vector3 Vector3::UP				= {0, 1, 0};
const Vector3 Vector3::RIGHT			= {1, 0, 0};
const Vector3 Vector3::OUT_OF_SCREEN	= {0, 0, 1};
const Vector3 Vector3::X				= {0, 1, 0};
const Vector3 Vector3::Y				= {1, 0, 0};
const Vector3 Vector3::Z				= {0, 0, 1};

// Definition of the sleep epsilon extern.
double cyclone::sleepEpsilon = 0.3;

// Functions to change sleepEpsilon.
void cyclone::setSleepEpsilon(double value)
{
    cyclone::sleepEpsilon = value;
}

double cyclone::getSleepEpsilon()
{
    return cyclone::sleepEpsilon;
}

double Matrix4::getDeterminant() const
{
    return -data[8]*data[5]*data[2]+
        data[4]*data[9]*data[2]+
        data[8]*data[1]*data[6]-
        data[0]*data[9]*data[6]-
        data[4]*data[1]*data[10]+
        data[0]*data[5]*data[10];
}

void Matrix4::setInverse(const Matrix4 &m)
{
    // Make sure the determinant is non-zero.
    double det = getDeterminant();
    if (det == 0) 
		return;
    det = 1.0 / det;

    data[0] = (-m.data[9]*m.data[6]+m.data[5]*m.data[10])*det;
    data[4] = (m.data[8]*m.data[6]-m.data[4]*m.data[10])*det;
    data[8] = (-m.data[8]*m.data[5]+m.data[4]*m.data[9])*det;

    data[1] = (m.data[9]*m.data[2]-m.data[1]*m.data[10])*det;
    data[5] = (-m.data[8]*m.data[2]+m.data[0]*m.data[10])*det;
    data[9] = (m.data[8]*m.data[1]-m.data[0]*m.data[9])*det;

    data[2] = (-m.data[5]*m.data[2]+m.data[1]*m.data[6])*det;
    data[6] = (+m.data[4]*m.data[2]-m.data[0]*m.data[6])*det;
    data[10] = (-m.data[4]*m.data[1]+m.data[0]*m.data[5])*det;

    data[3] = (m.data[9]*m.data[6]*m.data[3]
               -m.data[5]*m.data[10]*m.data[3]
               -m.data[9]*m.data[2]*m.data[7]
               +m.data[1]*m.data[10]*m.data[7]
               +m.data[5]*m.data[2]*m.data[11]
               -m.data[1]*m.data[6]*m.data[11])*det;
    data[7] = (-m.data[8]*m.data[6]*m.data[3]
               +m.data[4]*m.data[10]*m.data[3]
               +m.data[8]*m.data[2]*m.data[7]
               -m.data[0]*m.data[10]*m.data[7]
               -m.data[4]*m.data[2]*m.data[11]
               +m.data[0]*m.data[6]*m.data[11])*det;
    data[11] =(m.data[8]*m.data[5]*m.data[3]
               -m.data[4]*m.data[9]*m.data[3]
               -m.data[8]*m.data[1]*m.data[7]
               +m.data[0]*m.data[9]*m.data[7]
               +m.data[4]*m.data[1]*m.data[11]
               -m.data[0]*m.data[5]*m.data[11])*det;
}

Matrix3 Matrix3::linearInterpolate(const Matrix3& a, const Matrix3& b, double prop)
{
    Matrix3 result;
    for (unsigned i = 0; i < 9; i++) {
        result.data[i] = a.data[i] * (1-prop) + b.data[i] * prop;
    }
    return result;
}
