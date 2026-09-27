#include "quaternion.h"
#include <math.h>

static float clampf_local(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static float vec_norm(Vec3f v)
{
    return sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
}

static Vec3f vec_scale(Vec3f v, float s)
{
    Vec3f out = {v.x*s, v.y*s, v.z*s};
    return out;
}

static float vec_dot(Vec3f a, Vec3f b)
{
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

static Vec3f vec_cross(Vec3f a, Vec3f b)
{
    Vec3f out = {
        a.y*b.z - a.z*b.y,
        a.z*b.x - a.x*b.z,
        a.x*b.y - a.y*b.x
    };
    return out;
}

Quatf quat_identity(void)
{
    Quatf q = {1.0f, 0.0f, 0.0f, 0.0f};
    return q;
}

Quatf quat_multiply(Quatf a, Quatf b)
{
    Quatf q = {
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    };
    return q;
}

Quatf quat_conjugate(Quatf q)
{
    Quatf out = {q.w, -q.x, -q.y, -q.z};
    return out;
}

Quatf quat_normalize(Quatf q)
{
    float n = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (!isfinite(n) || n < 1.0e-8f) return quat_identity();
    float inv = 1.0f / n;
    Quatf out = {q.w*inv, q.x*inv, q.y*inv, q.z*inv};
    return out;
}

Quatf quat_from_rotation_vector(Vec3f dtheta)
{
    float angle = vec_norm(dtheta);
    if (angle < 1.0e-6f) {
        Quatf q = {1.0f, 0.5f*dtheta.x, 0.5f*dtheta.y, 0.5f*dtheta.z};
        return quat_normalize(q);
    }
    float half = 0.5f * angle;
    float s = sinf(half) / angle;
    Quatf q = {cosf(half), s*dtheta.x, s*dtheta.y, s*dtheta.z};
    return q;
}

Quatf quat_from_two_unit_vectors(Vec3f from, Vec3f to)
{
    float nf = vec_norm(from), nt = vec_norm(to);
    if (nf < 1.0e-6f || nt < 1.0e-6f) return quat_identity();
    from = vec_scale(from, 1.0f/nf);
    to = vec_scale(to, 1.0f/nt);
    float d = clampf_local(vec_dot(from, to), -1.0f, 1.0f);
    if (d < -0.9999f) {
        Vec3f axis = (fabsf(from.x) < 0.8f) ? (Vec3f){1,0,0} : (Vec3f){0,1,0};
        axis = vec_cross(from, axis);
        axis = vec_scale(axis, 1.0f/vec_norm(axis));
        Quatf q = {0.0f, axis.x, axis.y, axis.z};
        return q;
    }
    Vec3f c = vec_cross(from, to);
    Quatf q = {1.0f + d, c.x, c.y, c.z};
    return quat_normalize(q);
}

Vec3f quat_rotate(Quatf q_ab, Vec3f v_b)
{
    Quatf v = {0.0f, v_b.x, v_b.y, v_b.z};
    Quatf r = quat_multiply(quat_multiply(q_ab, v), quat_conjugate(q_ab));
    Vec3f out = {r.x, r.y, r.z};
    return out;
}

Vec3f quat_rotate_inverse(Quatf q_ab, Vec3f v_a)
{
    return quat_rotate(quat_conjugate(q_ab), v_a);
}

void quat_to_euler_zyx(Quatf q, float *roll, float *pitch, float *yaw)
{
    q = quat_normalize(q);
    *roll = atan2f(2.0f*(q.w*q.x + q.y*q.z),
                   1.0f - 2.0f*(q.x*q.x + q.y*q.y));
    float s = clampf_local(2.0f*(q.w*q.y - q.z*q.x), -1.0f, 1.0f);
    *pitch = asinf(s);
    *yaw = atan2f(2.0f*(q.w*q.z + q.x*q.y),
                  1.0f - 2.0f*(q.y*q.y + q.z*q.z));
}

