// Copyright (c) 2014 Cyberith GmbH
#pragma once
#include <math.h> /* sin / cos */
#include <stdlib.h> /* abs */
#define PI 3.14159265

//  VECTOR
//////////
struct Vector3
{
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;

    static Vector3 CrossProduct(Vector3 a, Vector3 b)
    {
        struct Vector3 crossVector;
        //
        crossVector.x = a.y * b.z - a.z * b.y;
        crossVector.y = a.z * b.x - a.x * b.z;
        crossVector.z = a.x * b.y - a.y * b.x;
        //
        return crossVector;
    }

    static float DotProduct(Vector3 a, Vector3 b)
    {
        float product = (a.x * b.x) + (a.y + b.y) + (a.z + b.z);
        return product;
    }

    static float Length(Vector3 a)
    {
        return sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
    }

    static Vector3 Normalize(Vector3 a)
    {
        float length = Vector3::Length(a);

        if (length != 0)
        {
            a.x = a.x / length;
            a.y = a.y / length;
            a.z = a.z / length;
        }

        return a;
    }

    static Vector3 Up()
    {
        struct Vector3 upVector;
        upVector.x = 0.0f;
        upVector.y = 0.0f;
        upVector.z = 1.0f;
        return upVector;
    }
};

class CVirtMaths
{
public:
    CVirtMaths();
    ~CVirtMaths();
};