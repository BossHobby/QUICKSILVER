#pragma once

#include <math.h>
#include "util/util.h"
#include "util/vector.h"


typedef struct {
  float w, x, y, z;
} quat_t;

// Initialize to identity quaternion
static inline quat_t quat_identity(void) {
  return (quat_t){1.0f, 0.0f, 0.0f, 0.0f};
}

// Quaternion multiplication: q1 * q2
static inline quat_t quat_multiply(const quat_t *q1, const quat_t *q2) {
  return (quat_t){
      .w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z,
      .x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y,
      .y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x,
      .z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w};
}

// Quaternion conjugate
static inline quat_t quat_conjugate(const quat_t *q) {
  return (quat_t){q->w, -q->x, -q->y, -q->z};
}

// Quaternion normalization
static inline void quat_normalize(quat_t *q) {
  const float norm = sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
  if (norm > 0.0f) {
    const float inv_norm = 1.0f / norm;
    q->w *= inv_norm;
    q->x *= inv_norm;
    q->y *= inv_norm;
    q->z *= inv_norm;
  }
}

// Create quaternion from axis-angle representation
static inline quat_t quat_from_axis_angle(const vec3_t *axis, float angle) {
  const float half_angle = angle * 0.5f;
  const float s = sinf(half_angle);
  return (quat_t){
      .w = cosf(half_angle),
      .x = axis->axis[0] * s,
      .y = axis->axis[1] * s,
      .z = axis->axis[2] * s};
}

// Rotate vector by quaternion
static inline vec3_t quat_rotate_vector(const quat_t *q, const vec3_t *v) {
  // v' = q * v * q^*
  // Optimized version using vector triple product
  const vec3_t qv = {{q->x, q->y, q->z}};
  vec3_t cross = vec3_cross(qv, *v);
  const vec3_t t = vec3_mul(cross, 2.0f);
  vec3_t scaled = vec3_mul(t, q->w);
  const vec3_t result = vec3_add(vec3_add(*v, scaled), vec3_cross(qv, t));
  return result;
}

// Convert quaternion to Euler angles (ZYX convention)
static inline void quat_to_euler(const quat_t *q, float *roll, float *pitch, float *yaw) {
  const float sinr_cosp = 2.0f * (q->w * q->x + q->y * q->z);
  const float cosr_cosp = 1.0f - 2.0f * (q->x * q->x + q->y * q->y);
  *roll = atan2f(sinr_cosp, cosr_cosp);

  const float sinp = 2.0f * (q->w * q->y - q->z * q->x);
  if (fabsf(sinp) >= 1.0f) {
    *pitch = copysignf(M_PI_F / 2.0f, sinp);
  } else {
    *pitch = asinf(sinp);
  }

  const float siny_cosp = 2.0f * (q->w * q->z + q->x * q->y);
  const float cosy_cosp = 1.0f - 2.0f * (q->y * q->y + q->z * q->z);
  *yaw = atan2f(siny_cosp, cosy_cosp);
}

// Create quaternion from Euler angles (ZYX convention)
static inline quat_t quat_from_euler(float roll, float pitch, float yaw) {
  const float cr = cosf(roll * 0.5f);
  const float sr = sinf(roll * 0.5f);
  const float cp = cosf(pitch * 0.5f);
  const float sp = sinf(pitch * 0.5f);
  const float cy = cosf(yaw * 0.5f);
  const float sy = sinf(yaw * 0.5f);

  return (quat_t){
      .w = cr * cp * cy + sr * sp * sy,
      .x = sr * cp * cy - cr * sp * sy,
      .y = cr * sp * cy + sr * cp * sy,
      .z = cr * cp * sy - sr * sp * cy};
}

// Get rotation matrix from quaternion
static inline void quat_to_rotation_matrix(const quat_t *q, float matrix[3][3]) {
  matrix[0][0] = 1.0f - 2.0f * (q->y * q->y + q->z * q->z);
  matrix[0][1] = 2.0f * (q->x * q->y - q->w * q->z);
  matrix[0][2] = 2.0f * (q->x * q->z + q->w * q->y);
  
  matrix[1][0] = 2.0f * (q->x * q->y + q->w * q->z);
  matrix[1][1] = 1.0f - 2.0f * (q->x * q->x + q->z * q->z);
  matrix[1][2] = 2.0f * (q->y * q->z - q->w * q->x);
  
  matrix[2][0] = 2.0f * (q->x * q->z - q->w * q->y);
  matrix[2][1] = 2.0f * (q->y * q->z + q->w * q->x);
  matrix[2][2] = 1.0f - 2.0f * (q->x * q->x + q->y * q->y);
}

// Integrate angular velocity to update quaternion
static inline quat_t quat_integrate(const quat_t *q, const vec3_t *omega, float dt) {
  // q_dot = 0.5 * q * omega
  const quat_t omega_quat = {0, omega->axis[0] * 0.5f, omega->axis[1] * 0.5f, omega->axis[2] * 0.5f};
  const quat_t q_dot = quat_multiply(q, &omega_quat);
  
  // q_new = q + q_dot * dt
  quat_t result = {
      q->w + q_dot.w * dt,
      q->x + q_dot.x * dt,
      q->y + q_dot.y * dt,
      q->z + q_dot.z * dt};
  
  quat_normalize(&result);
  return result;
}