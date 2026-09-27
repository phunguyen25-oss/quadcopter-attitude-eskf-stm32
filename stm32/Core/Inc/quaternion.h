#ifndef QUATERNION_H
#define QUATERNION_H

#include "fusion_types.h"

Quatf quat_identity(void);
Quatf quat_multiply(Quatf a, Quatf b);
Quatf quat_conjugate(Quatf q);
Quatf quat_normalize(Quatf q);
Quatf quat_from_rotation_vector(Vec3f dtheta);
Quatf quat_from_two_unit_vectors(Vec3f from, Vec3f to);
Vec3f quat_rotate(Quatf q_ab, Vec3f v_b);
Vec3f quat_rotate_inverse(Quatf q_ab, Vec3f v_a);
void quat_to_euler_zyx(Quatf q_wb, float *roll, float *pitch, float *yaw);

#endif

