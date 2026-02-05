#pragma once

#include "vector.h"

struct Matrix4
{
    float m[16]; // column-major

    static Matrix4 identity()
    {
        Matrix4 r{};
        r.m[0] = r.m[5] = r.m[10] = r.m[15] = 1.0f;
        return r;
    }
};

struct Quaternion
{
    float x, y, z, w;

    static Quaternion identity()
    {
        return { 0.0f, 0.0f, 0.0f, 1.0f };
    }
};


inline Matrix4 operator*( const Matrix4& a, const Matrix4& b )
{
    Matrix4 r{};

    for ( int c = 0; c < 4; ++c )
    {
        for ( int r0 = 0; r0 < 4; ++r0 )
        {
            r.m[c * 4 + r0] =
                a.m[0 * 4 + r0] * b.m[c * 4 + 0] +
                a.m[1 * 4 + r0] * b.m[c * 4 + 1] +
                a.m[2 * 4 + r0] * b.m[c * 4 + 2] +
                a.m[3 * 4 + r0] * b.m[c * 4 + 3];
        }
    }
    return r;
}

inline Matrix4 makeTranslation( const Vector3& t )
{
    Matrix4 r = Matrix4::identity();
    r.m[12] = t.x();
    r.m[13] = t.y();
    r.m[14] = t.z();
    return r;
}

inline Matrix4 makeScale( const Vector3& s )
{
    Matrix4 r{};
    r.m[0] = s.x();
    r.m[5] = s.y();
    r.m[10] = s.z();
    r.m[15] = 1.0f;
    return r;
}

inline Vector3 transformPoint( const Matrix4& m, const Vector3& v )
{    
    Vector3 r;
    r[0] = v.x() * m.m[0] + v.y() * m.m[4] + v.z() * m.m[8] + m.m[12];
    r[1] = v.x() * m.m[1] + v.y() * m.m[5] + v.z() * m.m[9] + m.m[13];
    r[2] = v.x() * m.m[2] + v.y() * m.m[6] + v.z() * m.m[10] + m.m[14];
    return r;
}

inline Vector3 transformVector( const Matrix4& m, const Vector3& v )
{
    Vector3 r;
    r[0] = v.x() * m.m[0] + v.y() * m.m[4] + v.z() * m.m[8];
    r[1] = v.x() * m.m[1] + v.y() * m.m[5] + v.z() * m.m[9];
    r[2] = v.x() * m.m[2] + v.y() * m.m[6] + v.z() * m.m[10];
    return r;
}
inline Matrix4 makeRotation( const Quaternion& q )
{
    const float x = q.x;
    const float y = q.y;
    const float z = q.z;
    const float w = q.w;

    Matrix4 r{};

    r.m[0] = 1 - 2 * y * y - 2 * z * z;
    r.m[1] = 2 * x * y + 2 * w * z;
    r.m[2] = 2 * x * z - 2 * w * y;
    r.m[3] = 0;

    r.m[4] = 2 * x * y - 2 * w * z;
    r.m[5] = 1 - 2 * x * x - 2 * z * z;
    r.m[6] = 2 * y * z + 2 * w * x;
    r.m[7] = 0;

    r.m[8] = 2 * x * z + 2 * w * y;
    r.m[9] = 2 * y * z - 2 * w * x;
    r.m[10] = 1 - 2 * x * x - 2 * y * y;
    r.m[11] = 0;

    r.m[15] = 1.0f;
    return r;
}


inline Matrix4 computeLocalMatrix( const Vector3& t, const Vector3& s, const Quaternion& r )
{
    return makeTranslation( t ) * makeRotation( r ) * makeScale( s );
}
