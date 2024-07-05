#pragma once

struct float3
{
    float x, y, z;

    float length() const { return sqrt(x*x + y*y + z*z); }

    float3 normalize() const
    {
        return (length() > 0)? float3{ x / length(), y / length(), z / length() }:*this;
    }
};
inline float3 operator*(const float3& a, float t)
{
    return { a.x * t, a.y * t, a.z * t };
}
inline float3 operator/(const float3& a, float t)
{
    return { a.x / t, a.y / t, a.z / t };
}

inline float3 operator+(const float3& a, const float3& b)
{
    return { a.x + b.x, a.y + b.y, a.z + b.z };
}

inline float3 operator-(const float3& a, const float3& b)
{
    return { a.x - b.x, a.y - b.y, a.z - b.z };
}