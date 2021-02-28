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

Fmatrix Matrix34ToFmatrix(vr::HmdMatrix34_t in)
{
    Fmatrix out{};
    
    out._11 = in.m[0][0];
    out._12 = in.m[0][1];
    out._13 = in.m[0][2];
    out._14 = 0.0f;
    out._21 = in.m[1][0];
    out._22 = in.m[1][1];
    out._23 = in.m[1][2];
    out._24 = 0.0f;
    out._31 = in.m[2][0];
    out._32 = in.m[2][1];
    out._33 = in.m[2][2];
    out._34 = 0.0f;
    out._41 = in.m[0][3];
    out._42 = in.m[1][3];
    out._43 = in.m[2][3];
    out._44 = 1.0f;

    return out;
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
Fvector QuaternionToYawPitchRoll(const vr::HmdQuaternion_t q)
{
    const auto sqw = q.w * q.w;
    const auto sqx = q.x * q.x;
    const auto sqy = q.y * q.y;
    const auto sqz = q.z * q.z;

    Fvector out;

    out[2] = -atan2(2 * (q.x * q.y + q.w * q.z), sqw - sqx + sqy - sqz) / M_PI_DIV_180; // Roll
    out[0] = -asin(-2 * (q.y * q.z - q.w * q.x)) / M_PI_DIV_180;    // Pitch
    out[1] = atan2(2 * (q.x * q.z + q.w * q.y), sqw - sqx - sqy + sqz) / M_PI_DIV_180;  // Yaw

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
