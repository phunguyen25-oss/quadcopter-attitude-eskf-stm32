#include "attitude_eskf.h"
#include "quaternion.h"
#include <math.h>
#include <string.h>

#define SQ(x) ((x)*(x))

static float norm3(Vec3f v)
{
    return sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
}

static void skew(Vec3f v, float S[3][3])
{
    S[0][0]=0.0f;  S[0][1]=-v.z;  S[0][2]= v.y;
    S[1][0]=v.z;   S[1][1]=0.0f;  S[1][2]=-v.x;
    S[2][0]=-v.y;  S[2][1]=v.x;   S[2][2]=0.0f;
}

static void identity6(float A[6][6])
{
    memset(A, 0, 36U*sizeof(float));
    for (int i=0; i<6; ++i) A[i][i] = 1.0f;
}

static void congruence6(float A[6][6], float P[6][6], float out[6][6])
{
    float temp[6][6] = {{0}};
    for (int i=0; i<6; ++i)
        for (int j=0; j<6; ++j)
            for (int k=0; k<6; ++k)
                temp[i][j] += A[i][k]*P[k][j];
    for (int i=0; i<6; ++i)
        for (int j=0; j<6; ++j) {
            out[i][j] = 0.0f;
            for (int k=0; k<6; ++k) out[i][j] += temp[i][k]*A[j][k];
        }
}

static void enforce_symmetry(float P[6][6])
{
    for (int i=0; i<6; ++i)
        for (int j=i+1; j<6; ++j) {
            float m = 0.5f*(P[i][j] + P[j][i]);
            P[i][j] = m;
            P[j][i] = m;
        }
}

static bool inverse3(float A[3][3], float inv[3][3])
{
    float det =
        A[0][0]*(A[1][1]*A[2][2]-A[1][2]*A[2][1]) -
        A[0][1]*(A[1][0]*A[2][2]-A[1][2]*A[2][0]) +
        A[0][2]*(A[1][0]*A[2][1]-A[1][1]*A[2][0]);
    if (!isfinite(det) || fabsf(det) < 1.0e-12f) return false;
    float d = 1.0f/det;
    inv[0][0]=(A[1][1]*A[2][2]-A[1][2]*A[2][1])*d;
    inv[0][1]=(A[0][2]*A[2][1]-A[0][1]*A[2][2])*d;
    inv[0][2]=(A[0][1]*A[1][2]-A[0][2]*A[1][1])*d;
    inv[1][0]=(A[1][2]*A[2][0]-A[1][0]*A[2][2])*d;
    inv[1][1]=(A[0][0]*A[2][2]-A[0][2]*A[2][0])*d;
    inv[1][2]=(A[0][2]*A[1][0]-A[0][0]*A[1][2])*d;
    inv[2][0]=(A[1][0]*A[2][1]-A[1][1]*A[2][0])*d;
    inv[2][1]=(A[0][1]*A[2][0]-A[0][0]*A[2][1])*d;
    inv[2][2]=(A[0][0]*A[1][1]-A[0][1]*A[1][0])*d;
    return true;
}

static float vector_nis(float y[3], float H[3][6],
                        float P[6][6], float R)
{
    float S[3][3] = {{0}}, Sinv[3][3];
    for (int i=0; i<3; ++i)
        for (int j=0; j<3; ++j) {
            for (int a=0; a<6; ++a)
                for (int b=0; b<6; ++b)
                    S[i][j] += H[i][a]*P[a][b]*H[j][b];
            if (i==j) S[i][j] += R;
        }
    if (!inverse3(S, Sinv)) return INFINITY;
    float nis = 0.0f;
    for (int i=0; i<3; ++i)
        for (int j=0; j<3; ++j)
            nis += y[i]*Sinv[i][j]*y[j];
    return nis;
}

/* Scalar Joseph update. H is 1x6; dx stores the accumulated error state. */
static bool scalar_update(float P[6][6], float dx[6], const float H[6],
                          float innovation, float R)
{
    float PHt[6] = {0}, K[6], A[6][6], Pnew[6][6];
    float residual = innovation;
    for (int j=0; j<6; ++j) residual -= H[j]*dx[j];
    for (int i=0; i<6; ++i)
        for (int j=0; j<6; ++j)
            PHt[i] += P[i][j]*H[j];
    float S = R;
    for (int j=0; j<6; ++j) S += H[j]*PHt[j];
    if (!isfinite(S) || S < 1.0e-10f) return false;
    for (int i=0; i<6; ++i) {
        K[i] = PHt[i]/S;
        dx[i] += K[i]*residual;
    }

    identity6(A);
    for (int i=0; i<6; ++i)
        for (int j=0; j<6; ++j)
            A[i][j] -= K[i]*H[j];
    congruence6(A, P, Pnew);
    for (int i=0; i<6; ++i)
        for (int j=0; j<6; ++j)
            Pnew[i][j] += K[i]*R*K[j];
    memcpy(P, Pnew, sizeof(Pnew));
    return true;
}

AttitudeEskfConfig_t attitude_eskf_default_config(void)
{
    AttitudeEskfConfig_t c = {
        /* Giá trị bring-up, phải thay bằng kết quả log/Allan của IMU thật. */
        .gyro_noise_rps_sqrt_hz = 0.0025f,
        .gyro_bias_rw_rps2_sqrt_hz = 0.00005f,
        .accel_dir_variance = 0.0025f,
        .accel_adaptive_lambda = 0.5f,
        .accel_norm_gate_mps2 = 2.5f,
        .accel_nis_gate = 16.266f,
        /* 2000 mẫu = 10 s @200 Hz; accel mất mỏ neo lâu như vậy mới báo fault. */
        .max_consecutive_rejects = 2000U
    };
    return c;
}

bool attitude_eskf_init(AttitudeEskf_t *f, Vec3f accel, Vec3f gyro_bias,
                        AttitudeEskfConfig_t cfg)
{
    memset(f, 0, sizeof(*f));
    float a = norm3(accel);
    if (!isfinite(a) || fabsf(a-FUSION_G_MSS) > 0.25f*FUSION_G_MSS) return false;
    Vec3f a_body = {accel.x/a, accel.y/a, accel.z/a};
    Vec3f up_world = {0.0f, 0.0f, 1.0f};
    f->q_wb = quat_from_two_unit_vectors(a_body, up_world);
    f->gyro_bias_rps = gyro_bias;
    f->cfg = cfg;
    float angle_std = 10.0f*FUSION_PI/180.0f;
    float bias_std = 5.0f*FUSION_PI/180.0f;
    for (int i=0; i<3; ++i) f->P[i][i] = angle_std*angle_std;
    for (int i=3; i<6; ++i) f->P[i][i] = bias_std*bias_std;
    f->initialized = true;
    f->healthy = true;
    return true;
}

bool attitude_eskf_predict(AttitudeEskf_t *f, Vec3f gyro, float dt)
{
    if (!f->initialized || !f->healthy || !isfinite(dt) || dt < 0.0002f || dt > 0.02f) {
        f->healthy = false;
        return false;
    }
    Vec3f w = {gyro.x-f->gyro_bias_rps.x,
               gyro.y-f->gyro_bias_rps.y,
               gyro.z-f->gyro_bias_rps.z};
    Vec3f rot = {w.x*dt, w.y*dt, w.z*dt};
    f->q_wb = quat_normalize(quat_multiply(f->q_wb, quat_from_rotation_vector(rot)));

    float Phi[6][6], Pnew[6][6], W[3][3];
    identity6(Phi);
    skew(w, W);
    for (int i=0; i<3; ++i)
        for (int j=0; j<3; ++j) {
            Phi[i][j] -= W[i][j]*dt;
            if (i==j) Phi[i][j+3] = -dt;
        }
    congruence6(Phi, f->P, Pnew);

    float qg = SQ(f->cfg.gyro_noise_rps_sqrt_hz);
    float qb = SQ(f->cfg.gyro_bias_rw_rps2_sqrt_hz);
    for (int i=0; i<3; ++i) {
        Pnew[i][i] += qg*dt + qb*dt*dt*dt/3.0f;
        Pnew[i][i+3] += -0.5f*qb*dt*dt;
        Pnew[i+3][i] += -0.5f*qb*dt*dt;
        Pnew[i+3][i+3] += qb*dt;
    }
    memcpy(f->P, Pnew, sizeof(Pnew));
    enforce_symmetry(f->P);
    return attitude_eskf_check_health(f);
}

bool attitude_eskf_update_accel(AttitudeEskf_t *f, Vec3f accel)
{
    if (!f->healthy) return false;
    float a_norm = norm3(accel);
    if (!isfinite(a_norm) || a_norm < 1.0e-4f) return false;
    float norm_error = fabsf(a_norm-FUSION_G_MSS);
    if (norm_error > f->cfg.accel_norm_gate_mps2) {
        f->accel_reject_count++;
        f->consecutive_rejects++;
        return false;
    }

    Vec3f z = {accel.x/a_norm, accel.y/a_norm, accel.z/a_norm};
    Vec3f up_world = {0.0f, 0.0f, 1.0f};
    Vec3f h = quat_rotate_inverse(f->q_wb, up_world);
    float Htheta[3][3];
    skew(h, Htheta); /* right-error: h(q*delta_q) ~= h + [h]x*dtheta */
    float H[3][6] = {{0}};
    for (int i=0; i<3; ++i)
        for (int j=0; j<3; ++j)
            H[i][j] = Htheta[i][j];
    float y[3] = {z.x-h.x, z.y-h.y, z.z-h.z};
    float R = f->cfg.accel_dir_variance *
              (1.0f + f->cfg.accel_adaptive_lambda*norm_error*norm_error);
    f->nis_acc = vector_nis(y, H, f->P, R);
    if (!isfinite(f->nis_acc) || f->nis_acc > f->cfg.accel_nis_gate) {
        f->accel_reject_count++;
        f->consecutive_rejects++;
        return false;
    }

    float dx[6] = {0};
    for (int axis=0; axis<3; ++axis) {
        if (!scalar_update(f->P, dx, H[axis], y[axis], R)) {
            f->healthy = false;
            return false;
        }
    }
    Vec3f dtheta = {dx[0], dx[1], dx[2]};
    if (norm3(dtheta) > 0.5f) {
        f->healthy = false;
        return false;
    }
    f->q_wb = quat_normalize(quat_multiply(f->q_wb, quat_from_rotation_vector(dtheta)));
    f->gyro_bias_rps.x += dx[3];
    f->gyro_bias_rps.y += dx[4];
    f->gyro_bias_rps.z += dx[5];

    /* Reset covariance sau khi đưa error state về zero. */
    float G[6][6], Sd[3][3], Pnew[6][6];
    identity6(G);
    skew(dtheta, Sd);
    for (int i=0; i<3; ++i)
        for (int j=0; j<3; ++j)
            G[i][j] -= 0.5f*Sd[i][j];
    congruence6(G, f->P, Pnew);
    memcpy(f->P, Pnew, sizeof(Pnew));
    enforce_symmetry(f->P);
    f->consecutive_rejects = 0U;
    return attitude_eskf_check_health(f);
}

bool attitude_eskf_check_health(AttitudeEskf_t *f)
{
    float qn = f->q_wb.w*f->q_wb.w + f->q_wb.x*f->q_wb.x +
               f->q_wb.y*f->q_wb.y + f->q_wb.z*f->q_wb.z;
    bool ok = isfinite(qn) && fabsf(qn-1.0f) < 0.02f;
    ok = ok && isfinite(f->gyro_bias_rps.x) && isfinite(f->gyro_bias_rps.y) &&
         isfinite(f->gyro_bias_rps.z);
    for (int i=0; i<6; ++i) {
        ok = ok && isfinite(f->P[i][i]) && f->P[i][i] > 0.0f && f->P[i][i] < 100.0f;
        for (int j=0; j<6; ++j) ok = ok && isfinite(f->P[i][j]);
    }
    if (f->consecutive_rejects > f->cfg.max_consecutive_rejects) ok = false;
    f->healthy = f->healthy && ok;
    return f->healthy;
}
