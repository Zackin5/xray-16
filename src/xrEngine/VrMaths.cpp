#pragma once

#include "openvr\openvr.h"  // TODO: proper OpenVR external include
#include "xrCore/vector.h"

#define M_PI_DIV_180 (M_PI / 180.0)

// Transforms a HMD Matrix34 to a Vector3
// Math borrowed from https://github.com/Omnifinity/OpenVR-Tracking-Example
Fvector Matrix34ToFVector(vr::HmdMatrix34_t in) 
{
    Fvector vector{};

    vector.x = in.m[0][3];
    vector.y = in.m[1][3];
    vector.z = in.m[2][3];

    return vector;
}

// Transforms a HMD Matrix34 to a Vector3
// Math borrowed from https://github.com/Omnifinity/OpenVR-Tracking-Example
vr::HmdVector3_t Matrix34ToVector(vr::HmdMatrix34_t in)
{
    vr::HmdVector3_t vector;

    vector.v[0] = in.m[0][3];
    vector.v[1] = in.m[1][3];
    vector.v[2] = in.m[2][3];

    return vector;
}

// Transforms a HMD Matrix34 to a Quaternion
// Function logic nicked from https://github.com/Omnifinity/OpenVR-Tracking-Example
vr::HmdQuaternion_t Matrix34ToQuaternion(vr::HmdMatrix34_t in)
{
    vr::HmdQuaternion_t q{};

    q.w = sqrt(fmax(0, 1 + in.m[0][0] + in.m[1][1] + in.m[2][2])) / 2;
    q.x = sqrt(fmax(0, 1 + in.m[0][0] - in.m[1][1] - in.m[2][2])) / 2;
    q.y = sqrt(fmax(0, 1 - in.m[0][0] + in.m[1][1] - in.m[2][2])) / 2;
    q.z = sqrt(fmax(0, 1 - in.m[0][0] - in.m[1][1] + in.m[2][2])) / 2;
    q.x = copysign(q.x, in.m[2][1] - in.m[1][2]);
    q.y = copysign(q.y, in.m[0][2] - in.m[2][0]);
    q.z = copysign(q.z, in.m[1][0] - in.m[0][1]);
    return q;
}

// Rotates a vector by a quaternion and returns the results
// Based on math from https://gamedev.stackexchange.com/questions/28395/rotating-vector3-by-a-quaternion
vr::HmdVector3_t RotateVectorByQuaternion(vr::HmdVector3_t v, vr::HmdQuaternion_t q)
{
    vr::HmdVector3_t u, result;
    u.v[0] = q.x;
    u.v[1] = q.y;
    u.v[2] = q.z;
    float s = q.w;

    // Dot products of u,v and u,u
    float uvDot = (u.v[0] * v.v[0] + u.v[1] * v.v[1] + u.v[2] * v.v[2]);
    float uuDot = (u.v[0] * u.v[0] + u.v[1] * u.v[1] + u.v[2] * u.v[2]);

    // Calculate cross product of u, v
    vr::HmdVector3_t uvCross;
    uvCross.v[0] = u.v[1] * v.v[2] - u.v[2] * v.v[1];
    uvCross.v[1] = u.v[2] * v.v[0] - u.v[0] * v.v[2];
    uvCross.v[2] = u.v[0] * v.v[1] - u.v[1] * v.v[0];

    // Calculate each vectors' result individually because there aren't arthimetic functions for HmdVector3_t
    result.v[0] = u.v[0] * 2.0f * uvDot + (s * s - uuDot) * v.v[0] + 2.0f * s * uvCross.v[0];
    result.v[1] = u.v[1] * 2.0f * uvDot + (s * s - uuDot) * v.v[1] + 2.0f * s * uvCross.v[1];
    result.v[2] = u.v[2] * 2.0f * uvDot + (s * s - uuDot) * v.v[2] + 2.0f * s * uvCross.v[2];

    return result;
}

// Converts a HMD quaternion to a xray euler angle
Fvector QuaternionToAngles(const vr::HmdQuaternion_t q)
{
    _vector4<float> q2{};
    q2.x = q.x * q.x;
    q2.y = q.y * q.y;
    q2.z = q.z * q.z;
    q2.w = q.w * q.w;

    Fvector out{};
    out.x = (180.0 / M_PI) * atan2(2 * (q.z * q.w + q.y * q.x), (-q2[1] - q2[2] + q2[3] + q2[0]));  // Roll
    out.y = (180.0 / M_PI) * asin(-2 * (q.y * q.w - q.z * q.x));    // Pitch
    out.z = (180.0 / M_PI) * atan2(2 * (q.y * q.z + q.w * q.x), (q2[1] - q2[2] - q2[3] + q2[0]));   // Yaw
    return out;
}

// From https://steamcommunity.com/app/250820/discussions/0/1728711392744037419/
Fvector Matrix34ToYPR(vr::HmdMatrix34_t m)
{
    Fvector v{};
    vr::HmdQuaternion_t q = Matrix34ToQuaternion(m);
    // http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/index.htm
    // Conversion of the code under Code, Java code to do conversion:

    double test = q.x * q.y + q.z * q.w;
    if (test > 0.499)
    { // singularity at north pole
        v.z = 2 * atan2(q.x, q.w); // heading
        v.x = M_PI / 2; // attitude
        v.y = 0; // bank
        return v;
    }
    if (test < -0.499)
    { // singularity at south pole
        v.z = -2 * atan2(q.x, q.w); // headingq
        v.x = -M_PI / 2; // attitude
        v.y = 0; // bank
        return v;
    }
    double sqx = q.x * q.x;
    double sqy = q.y * q.y;
    double sqz = q.z * q.z;
    v.z = atan2(2 * q.y * q.w - 2 * q.x * q.z, 1 - 2 * sqy - 2 * sqz); // heading
    v.x = asin(2 * test); // attitude
    v.y = atan2(2 * q.x * q.w - 2 * q.y * q.z, 1 - 2 * sqx - 2 * sqz); // bank
    return v;
}

Fvector QuaternionToYawPitchRoll(const vr::HmdQuaternion_t q)
{
    float sqw = q.w * q.w;
    float sqx = q.x * q.x;
    float sqy = q.y * q.y;
    float sqz = q.z * q.z;
    float unit = sqx + sqy + sqz + sqw; // if normalised is one, otherwise is correction factor
    float test = q.x * q.y + q.z * q.w;

    Fvector out{};

    if (test > 0.499 * unit)
    { // singularity at north pole
        out.x = 2 * atan2(q.x, q.w) / M_PI_DIV_180; // Yaw
        out.z = -M_PI / 2 / M_PI_DIV_180;   // Roll
        out.y = 0;  // Pitch
    }
    else if (test < -0.499 * unit)
    { // singularity at south pole
        out.x = -2 * atan2(q.x, q.w) / M_PI_DIV_180;
        out.z = M_PI / 2 / M_PI_DIV_180;
        out.y = 0;
    }
    else
    {
        out.x = atan2(2 * q.y * q.w - 2 * q.x * q.z, sqx - sqy - sqz + sqw) / M_PI_DIV_180;
        out.z = -asin(2 * test / unit) / M_PI_DIV_180;
        out.y = -atan2(2 * q.x * q.w - 2 * q.y * q.z, -sqx + sqy - sqz + sqw) / M_PI_DIV_180;
    }

    return out;
}

// Converts a HMD 4x4 matrix to a xray 4x4 matrix
Fmatrix Matrix44ToFmatrix(const vr::HmdMatrix44_t in)
{ 
    Fmatrix out{};
    out.m[0][0] = in.m[0][0];
    out.m[0][1] = in.m[0][1];
    out.m[0][2] = in.m[0][2];
    out.m[0][3] = in.m[0][3];
    out.m[1][0] = in.m[1][0];
    out.m[1][1] = in.m[1][1];
    out.m[1][2] = in.m[1][2];
    out.m[1][3] = in.m[1][3];
    out.m[2][0] = in.m[2][0];
    out.m[2][1] = in.m[2][1];
    out.m[2][2] = in.m[2][2];
    out.m[2][3] = in.m[2][3];
    out.m[3][0] = in.m[3][0];
    out.m[3][1] = in.m[3][1];
    out.m[3][2] = in.m[3][2];
    out.m[3][3] = in.m[3][3];
    return out;
}

Fmatrix InvMatrix44ToFmatrix(const vr::HmdMatrix44_t in)
{
    Fmatrix out{};
    out.m[0][0] = in.m[0][0];
    out.m[0][1] = in.m[1][0];
    out.m[0][2] = in.m[2][0];
    out.m[0][3] = in.m[3][0];
    out.m[1][0] = in.m[0][1];
    out.m[1][1] = in.m[1][1];
    out.m[1][2] = in.m[2][1];
    out.m[1][3] = in.m[3][1];
    out.m[2][0] = in.m[0][2];
    out.m[2][1] = in.m[1][2];
    out.m[2][2] = in.m[2][2];
    out.m[2][3] = in.m[3][2];
    out.m[3][0] = in.m[0][3];
    out.m[3][1] = in.m[1][3];
    out.m[3][2] = in.m[2][3];
    out.m[3][3] = in.m[3][3];
    return out;
}

// OpenVR units to XRay
Fvector HmdVectorToFVector(vr::HmdVector3_t v) 
{ 
    Fvector out{};
    out.x = v.v[0];
    out.y = v.v[1];
    out.z = v.v[2];
    return out;
}
