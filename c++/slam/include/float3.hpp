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