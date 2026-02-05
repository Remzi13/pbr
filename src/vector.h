#pragma once

#include <cmath>
#include <ostream>
#include <algorithm>

class Vector2 {
public:
	float d[2];

	Vector2( float x, float y ) : d{ x, y } {}

	float x() const { return d[0]; }
	float y() const { return d[1]; }

	Vector2 operator-() const { return Vector2( -d[0], -d[1]); }
	float operator[]( int i ) const { return d[i]; }
	float& operator[]( int i ) { return d[i]; }

	Vector2& operator+=( const Vector2& v ) {
		d[0] += v.d[0];
		d[1] += v.d[1];
		return *this;
	}
	Vector2& operator-=( const Vector2& u ) {
		d[0] -= u[0];
		d[1] -= u[1];
		return *this;
	}

	Vector2& operator*=( float t ) {
		d[0] *= t;
		d[1] *= t;

		return *this;
	}

	bool operator==( const Vector2& u )
	{
		return d[0] == u[0] && d[1] == u[1] && d[2] == u[2];
	}

	Vector2& operator/=( float t ) {
		return *this *= 1 / t;
	}
};

inline Vector2 operator*( float t, const Vector2& v ) {
	return Vector2( t * v.d[0], t * v.d[1]);
}

inline Vector2 operator+( float t, const Vector2& v ) {
	return Vector2( t + v.d[0], t + v.d[1] );
}

inline Vector2 operator*( const Vector2& v, float t ) {
	return t * v;
}

inline Vector2 operator/( const Vector2& v, float t ) {
	return ( 1 / t ) * v;
}

class Vector3 {
public:
	float d[3];

	Vector3() : d{ 0,0,0 } {}
	Vector3( float x, float y, float z ) : d{ x, y, z } {}

	float x() const { return d[0]; }
	float y() const { return d[1]; }
	float z() const { return d[2]; }

	Vector3 operator-() const { return Vector3( -d[0], -d[1], -d[2] ); }
	float operator[]( int i ) const { return d[i]; }
	float& operator[]( int i ) { return d[i]; }

	Vector3& operator+=( const Vector3& v ) {
		d[0] += v.d[0];
		d[1] += v.d[1];
		d[2] += v.d[2];
		return *this;
	}
	Vector3& operator-=(const Vector3& u) {
		d[0] -= u[0];
		d[1] -= u[1];
		d[2] -= u[2];
		return *this;
	}

	Vector3& operator*=( float t ) {
		d[0] *= t;
		d[1] *= t;
		d[2] *= t;
		return *this;
	}

	bool operator==(const Vector3& u)
	{
		return d[0] == u[0] && d[1] == u[1] && d[2] == u[2];
	}

	Vector3& operator/=( float t ) {
		return *this *= 1 / t;
	}

	float length() const {
		return std::sqrt( length_squared() );
	}

	float length_squared() const {
		return d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
	}
};

// point3 is just an alias for vec3, but useful for geometric clarity in the code.
using point3 = Vector3;


// Vector Utility Functions

inline std::ostream& operator<<( std::ostream& out, const Vector3& v ) {
	return out << v.d[0] << ' ' << v.d[1] << ' ' << v.d[2];
}

inline Vector3 operator+( const Vector3& u, const Vector3& v ) {
	return Vector3( u.d[0] + v.d[0], u.d[1] + v.d[1], u.d[2] + v.d[2] );
}

inline Vector3 operator-( const Vector3& u, const Vector3& v ) {
	return Vector3( u.d[0] - v.d[0], u.d[1] - v.d[1], u.d[2] - v.d[2] );
}

inline Vector3 operator*( const Vector3& u, const Vector3& v ) {
	return Vector3( u.d[0] * v.d[0], u.d[1] * v.d[1], u.d[2] * v.d[2] );
}

inline Vector3 operator/(const Vector3& u, const Vector3& v) {
	return Vector3(u.d[0] / v.d[0], u.d[1] / v.d[1], u.d[2] / v.d[2]);
}

inline Vector3 operator*( float t, const Vector3& v ) {
	return Vector3( t * v.d[0], t * v.d[1], t * v.d[2] );
}

inline Vector3 operator-( float t, const Vector3& v ) {
	return Vector3( t - v.d[0] , t - v.d[1], t - v.d[2] );
}

inline Vector3 operator*( const Vector3& v, float t ) {
	return t * v;
}

inline Vector3 operator/( const Vector3& v, float t ) {
	return ( 1 / t ) * v;
}



inline float dot( const Vector3& u, const Vector3& v ) {
	return u.d[0] * v.d[0]
		+ u.d[1] * v.d[1]
		+ u.d[2] * v.d[2];
}

inline Vector3 cross( const Vector3& u, const Vector3& v ) {
	return Vector3( u.d[1] * v.d[2] - u.d[2] * v.d[1],
		u.d[2] * v.d[0] - u.d[0] * v.d[2],
		u.d[0] * v.d[1] - u.d[1] * v.d[0] );
}

inline Vector3 unit_vector( const Vector3& v ) {
	return v / v.length();
}

inline Vector3 min(const Vector3& a, const Vector3& b)
{
	return Vector3{
		std::min(a.x(), b.x()),
		std::min(a.y(), b.y()),
		std::min(a.z(), b.z())
	};
}

inline Vector3 max(const Vector3& a, const Vector3& b)
{
	return Vector3{
		std::max(a.x(), b.x()),
		std::max(a.y(), b.y()),
		std::max(a.z(), b.z())
	};
}


// ----------------------------- Vector4 ---------------
class Vector4 {
public:
	float d[4];

	Vector4() : d{ 0,0,0,0 } {}
	Vector4( float x, float y, float z, float w ) : d{ x, y, z, w } {}

	float x() const { return d[0]; }
	float y() const { return d[1]; }
	float z() const { return d[2]; }
	float w() const { return d[3]; }

	Vector4 operator-() const { return Vector4( -d[0], -d[1], -d[2], -d[3] ); }
	float operator[]( int i ) const { return d[i]; }
	float& operator[]( int i ) { return d[i]; }

	Vector4& operator+=( const Vector4& v ) {
		d[0] += v.d[0];
		d[1] += v.d[1];
		d[2] += v.d[2];
		d[3] += v.d[3];
		return *this;
	}

	Vector4& operator*=( float t ) {
		d[0] *= t;
		d[1] *= t;
		d[2] *= t;
		d[3] *= t;
		return *this;
	}

	Vector4& operator/=( float t ) {
		return *this *= 1 / t;
	}

	float length() const {
		return std::sqrt( length_squared() );
	}

	float length_squared() const {
		return d[0] * d[0] + d[1] * d[1] + d[2] * d[2] + d[3]*d[3];
	}
};
