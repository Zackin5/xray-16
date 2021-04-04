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
    out._11 = in.m[0][0];    out._12 = in.m[1][0];    out._13 = in.m[2][0];    out._14 = 0.0f;
    out._21 = in.m[0][1];    out._22 = in.m[1][1];    out._23 = in.m[2][1];    out._24 = 0.0f;
    out._31 = -in.m[0][2];    out._32 = -in.m[1][2];    out._33 = -in.m[2][2];    out._34 = 0.0f;
    out._41 = in.m[0][3];    out._42 = in.m[1][3];    out._43 = in.m[2][3];    out._44 = 1.0f;

    /*out._11 = in.m[0][0];    out._21 = in.m[1][0];    out._31 = in.m[2][0];    out._14 = 0.0f;
    out._12 = in.m[0][1];    out._22 = in.m[1][1];    out._32 = in.m[2][1];    out._24 = 0.0f;
    out._13 = in.m[0][2];    out._23 = in.m[1][2];    out._33 = in.m[2][2];    out._34 = 0.0f;
    out._41 = in.m[0][3];    out._42 = in.m[1][3];    out._43 = in.m[2][3];    out._44 = 1.0f;*/

    return out;
}

// Converts a HMD 4x4 matrix to a xray 4x4 matrix
Fmatrix Matrix44ToFmatrix(const vr::HmdMatrix44_t in)
{
    Fmatrix out{};
    out._11 = in.m[0][0];    out._12 = in.m[1][0];    out._13 = in.m[2][0];    out._14 = in.m[3][0];
    out._21 = in.m[0][1];    out._22 = in.m[1][1];    out._23 = in.m[2][1];    out._24 = in.m[3][1];
    out._31 = in.m[0][2];    out._32 = in.m[1][2];    out._33 = in.m[2][2];    out._34 = in.m[3][2];
    out._41 = in.m[0][3];    out._42 = in.m[1][3];    out._43 = in.m[2][3];    out._44 = in.m[3][3];

    /*out._11 = in.m[0][0];    out._21 = in.m[1][0];    out._31 = in.m[2][0];    out._14 = in.m[3][0];
    out._12 = in.m[0][1];    out._22 = in.m[1][1];    out._32 = in.m[2][1];    out._24 = in.m[3][1];
    out._13 = in.m[0][2];    out._23 = in.m[1][2];    out._33 = in.m[2][2];    out._34 = in.m[3][2];
    out._41 = in.m[0][3];    out._42 = in.m[1][3];    out._43 = in.m[2][3];    out._44 = in.m[3][3];*/
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

// OpenVR units to XRay
Fvector HmdVectorToFVector(vr::HmdVector3_t v) 
{ 
    Fvector out{};
    out.x = v.v[0];
    out.y = v.v[1];
    out.z = v.v[2];
    return out;
}

Fmatrix ComposeView(vr::HmdMatrix34_t devicePose, vr::HmdMatrix34_t eyePose, Fvector playerPos, Fvector playerRot) 
{
    // Get HMD orientation
    Fvector hmdForward = Fvector{   
        devicePose.m[0][2], 
        devicePose.m[1][2], 
        devicePose.m[2][2]};

    Fvector hmdUp = Fvector{
        devicePose.m[0][1], 
        devicePose.m[1][1], 
        devicePose.m[2][1]};

    Fvector hmdRight = Fvector{
        devicePose.m[0][0], 
        devicePose.m[1][0], 
        devicePose.m[2][0]};

    Fvector hmdTransaction = Fvector{
        devicePose.m[0][3], 
        devicePose.m[1][3], 
        devicePose.m[2][3]};

    // OVR uses RH coordinates, flip to XRay LH
    Fvector vForward = Fvector{0.0f, 0.0f, 1.0};
    hmdForward.reflect(hmdForward, vForward);
    hmdUp.reflect(hmdUp, vForward);
    hmdRight.reflect(hmdRight, vForward);
    hmdTransaction.reflect(hmdTransaction, vForward);

    Fmatrix viewMatrix{};
    viewMatrix._11 = hmdRight.x;
    viewMatrix._12 = hmdRight.y;
    viewMatrix._13 = hmdRight.z;
    viewMatrix._14 = 0.0f;
    viewMatrix._21 = hmdUp.x;
    viewMatrix._22 = hmdUp.y;
    viewMatrix._23 = hmdUp.z;
    viewMatrix._24 = 0.0f;
    viewMatrix._31 = hmdForward.x;
    viewMatrix._32 = hmdForward.y;
    viewMatrix._33 = hmdForward.z;
    viewMatrix._34 = 0.0f;
    viewMatrix._41 = hmdTransaction.x;
    viewMatrix._42 = hmdTransaction.y;
    viewMatrix._43 = hmdTransaction.z;
    viewMatrix._44 = 1.0f;

    // Apply eye offset
    auto eyeMatrix = Matrix34ToFmatrix(eyePose);
    viewMatrix.mul(viewMatrix, eyeMatrix);

    // TODO: Apply player rotation
    /*Fvector yRotation = playerRot;
    float angle = yRotation.dotproduct(yRotation.mul(vForward.div(vForward, vForward.magnitude())));
    Fmatrix yRotMatrix{};
    yRotMatrix.rotateY(180 - angle);

    viewMatrix.mulA_43(yRotMatrix);*/

    // Apply player position
    viewMatrix.translate_add(playerPos);

    return viewMatrix;
}

Fmatrix ComposeProjection(float fLeft, float fRight, float fTop, float fBottom, float zNear, float zFar, float zoomMult)
{
    float idx = 1.0f / (zoomMult * (fRight - fLeft));
    float idy = 1.0f / (zoomMult * (fBottom - fTop));
    float idz = 1.0f / (zFar - zNear);
    float sx = zoomMult * (fRight + fLeft);
    float sy = zoomMult * (fBottom + fTop);
    //float Q = zFar / (zFar - zNear);

    Fmatrix p{};
    /*p._11 = 2 * idx;
    p._12 = 0;
    p._13 = 0;
    p._14 = 0;
    p._21 = 0;
    p._22 = 2 * idy;
    p._23 = 0;
    p._24 = 0;
    p._31 = sx * idx;
    p._32 = sy * idy;
    p._33 = Q;
    p._34 = 1.0f;
    p._41 = 0;
    p._42 = 0;
    p._43 = -Q * zNear;
    p._44 = 0;*/

    p._11 = 2 * idx;
    p._21 = 0;
    p._31 = sx * idx;
    p._41 = 0;
    p._12 = 0;
    p._22 = 2 * idy;
    p._32 = sy * idy;
    p._42 = 0;
    p._13 = 0;
    p._23 = 0;
    p._33 = -zFar * idz;
    p._43 = -zFar * zNear * idz;
    p._14 = 0;
    p._24 = 0;
    p._34 = -1.0f;
    p._44 = 0;

    /*float zDelta = zFar * idz;
    p._11 = 2 * idx;
    p._12 = 0;
    p._13 = 0;
    p._14 = 0;
    p._21 = 0;
    p._22 = 2 * idy;
    p._23 = 0;
    p._24 = 0;
    p._31 = sx * idx;
    p._32 = sy * idy;
    p._33 = zDelta; // C
    p._34 = 1.0f;   // D
    p._41 = 0;
    p._42 = 0;
    p._43 = -zNear * zDelta; // E
    p._44 = 0;*/

    Fmatrix mirrorMatrix{};
    mirrorMatrix._11 = 1.0f;
    mirrorMatrix._12 = 0.0f;
    mirrorMatrix._13 = 0.0f;
    mirrorMatrix._14 = 0.0f;
    mirrorMatrix._21 = 0.0f;
    mirrorMatrix._22 = 1.0f;
    mirrorMatrix._23 = 0.0f;
    mirrorMatrix._24 = 0.0f;
    mirrorMatrix._31 = 0.0f;
    mirrorMatrix._32 = 0.0f;
    mirrorMatrix._33 = -1.0f;
    mirrorMatrix._34 = 0.0f;
    mirrorMatrix._41 = 0.0f;
    mirrorMatrix._42 = 0.0f;
    mirrorMatrix._43 = 0.0f;
    mirrorMatrix._44 = 1.0f;

    return p.mul(p, mirrorMatrix);
    //return p;
}

float GetPredictedPhotonTime(vr::IVRSystem* pVRSystem) 
{
    float fSecondsSinceLastVsync;
    pVRSystem->GetTimeSinceLastVsync(&fSecondsSinceLastVsync, NULL);

    float fDisplayFrequency =
        pVRSystem->GetFloatTrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_DisplayFrequency_Float);
    float fFrameDuration = 1.f / fDisplayFrequency;
    float fVsyncToPhotons = pVRSystem->GetFloatTrackedDeviceProperty(
        vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_SecondsFromVsyncToPhotons_Float);

    return fFrameDuration - fSecondsSinceLastVsync + fVsyncToPhotons;
}
