#include "gravity_compensation.h"

#include <math.h>

// 来自 3rrs_parameters.json，长度统一为 m、质量为 kg。
static const float kL1 = 0.060f;
static const float kL2 = 0.102f;
static const float kBaseRadius = 0.126f;
static const float kPlatformRadius = 0.137f;
static const float kActiveMass = 0.035f;
static const float kPassiveMass = 0.015f;
static const float kPlatformMass = 0.31035f;
static const float kActiveCom = 0.008265f;
static const float kPassiveCom = 0.061f;
static const float kGravity = 9.81f;

static void Cross(const float a[3], const float b[3], float c[3]) {
  c[0] = a[1] * b[2] - a[2] * b[1];
  c[1] = a[2] * b[0] - a[0] * b[2];
  c[2] = a[0] * b[1] - a[1] * b[0];
}

// 带主元的高斯消元，直接求解三元方程组；调用方提供非奇异位形。
static void Solve(float a[3][4], float x[3]) {
  for (int k = 0; k < 3; ++k) {
    int pivot = k;
    for (int i = k + 1; i < 3; ++i) {
      if (fabsf(a[i][k]) > fabsf(a[pivot][k])) {
        pivot = i;
      }
    }
    for (int j = k; j < 4; ++j) {
      float temp = a[k][j];
      a[k][j] = a[pivot][j];
      a[pivot][j] = temp;
    }
    for (int i = k + 1; i < 3; ++i) {
      float ratio = a[i][k] / a[k][k];
      for (int j = k + 1; j < 4; ++j) {
        a[i][j] -= ratio * a[k][j];
      }
    }
  }
  for (int i = 2; i >= 0; --i) {
    float value = a[i][3];
    for (int j = i + 1; j < 3; ++j) {
      value -= a[i][j] * x[j];
    }
    x[i] = value / a[i][i];
  }
}

static void BuildConstraints(const float q[3], const float st[3],
                             const float ct[3], float a[3][4],
                             float denominator[3]) {
  float sx = sinf(q[0]), cx = cosf(q[0]);
  float sy = sinf(q[1]), cy = cosf(q[1]);
  float yaw = atan2f(-sx * sy, cx + cy);
  float sz = sinf(yaw), cz = cosf(yaw);
  // R = Rx(psi_x) Ry(psi_y) Rz(yaw) 的前两列。
  float r[3][2] = {{cy * cz, -cy * sz},
                   {cx * sz + sx * sy * cz, cx * cz - sx * sy * sz},
                   {sx * sz - cx * sy * cz, sx * cz + cx * sy * sz}};
  float p[3] = {0.5f * kPlatformRadius * (r[0][0] - r[1][1]),
                -kPlatformRadius * r[1][0], q[2]};
  float axes[2][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, cx, sx}};
  float normal[3] = {sy, -sx * cy, cx * cy};
  float yaw_derivative[2] = {-sy / (1.0f + cx * cy),
                             -sx / (1.0f + cx * cy)};
  float dr[2][3][2];
  for (int k = 0; k < 2; ++k) {
    for (int j = 0; j < 2; ++j) {
      float column[3] = {r[0][j], r[1][j], r[2][j]};
      float tilt[3], spin[3];
      Cross(axes[k], column, tilt);
      Cross(normal, column, spin);
      for (int n = 0; n < 3; ++n) {
        dr[k][n][j] = tilt[n] + yaw_derivative[k] * spin[n];
      }
    }
  }

  const float radial[3][2] = {{1.0f, 0.0f},
                             {-0.5f, 0.8660254038f},
                             {-0.5f, -0.8660254038f}};
  for (int i = 0; i < 3; ++i) {
    float ex = radial[i][0], ey = radial[i][1];
    float point[3];
    for (int n = 0; n < 3; ++n) {
      point[n] = p[n] + kPlatformRadius * (r[n][0] * ex + r[n][1] * ey);
    }
    float s = ex * point[0] + ey * point[1] - kBaseRadius;
    float h = point[2];
    float u = s - kL1 * ct[i];
    float v = h + kL1 * st[i];
    denominator[i] = kL1 * (s * st[i] + h * ct[i]);
    for (int k = 0; k < 2; ++k) {
      float dx = 0.5f * kPlatformRadius * (dr[k][0][0] - dr[k][1][1]) +
          kPlatformRadius * (dr[k][0][0] * ex + dr[k][0][1] * ey);
      float dy = -kPlatformRadius * dr[k][1][0] +
          kPlatformRadius * (dr[k][1][0] * ex + dr[k][1][1] * ey);
      float dh = kPlatformRadius * (dr[k][2][0] * ex + dr[k][2][1] * ey);
      a[i][k] = u * (ex * dx + ey * dy) + v * dh;
    }
    a[i][2] = v;
    // 固定杆长约束 F=u^2+v^2-L2^2，a 的前三列为 (dF/dq)/2。
    a[i][3] = -0.5f * (u * u + v * v - kL2 * kL2);
  }

}

void GravityCompensation(const float theta_rad[3], float pose[3], float k_g,
                         float torque_nm[3], int16_t current_cmd[3]) {
  float st[3], ct[3];
  for (int i = 0; i < 3; ++i) {
    st[i] = sinf(theta_rad[i]);
    ct[i] = cosf(theta_rad[i]);
  }
  float constraints[3][4], denominator[3];
  // 由实测主动角求平台正解；上次位姿作为本次初值，做五次牛顿迭代。
  for (int iteration = 0; iteration < 5; ++iteration) {
    BuildConstraints(pose, st, ct, constraints, denominator);
    float delta[3];
    Solve(constraints, delta);
    for (int k = 0; k < 3; ++k) {
      pose[k] += delta[k];
    }
  }
  BuildConstraints(pose, st, ct, constraints, denominator);
  float a[3][4];
  for (int i = 0; i < 3; ++i) {
    for (int k = 0; k < 3; ++k) {
      // J_theta_q[i][k] = -(u*ds/dq_k + v*dh/dq_k) / denominator_i。
      a[k][i] = -constraints[i][k] / denominator[i];
    }
  }

  // 三根从动臂质量、质心比例相同，且平台质心位于原点：
  // V = g * (M * Zp - K * sum(sin(theta_i)))。
  // M = m0 + 3*m2*rho2/L2；K = m1*rho1 + m2*L1*(1-rho2/L2)。
  // 先解 J^T * load = [0, 0, M*g]^T，再加 -g*K*cos(theta_i)。
  float ratio = kPassiveCom / kL2;
  float moment = kActiveMass * kActiveCom + kPassiveMass * kL1 * (1.0f - ratio);
  a[0][3] = 0.0f;
  a[1][3] = 0.0f;
  a[2][3] = (kPlatformMass + 3.0f * kPassiveMass * ratio) * kGravity;
  Solve(a, torque_nm);
  for (int i = 0; i < 3; ++i) {
    // 输出端比例：tau_cmd = k_g * tau_g。
    torque_nm[i] = k_g * (torque_nm[i] - kGravity * moment * ct[i]);
    // DJI M3508 手册：输出轴 Kt=0.3 N*m/A，已含减速箱，不再乘 19。
    // C620 手册：指令 +/-16384 对应转矩电流 +/-20 A。
    // https://rm-static.djicdn.com/robomasters/public/document/RoboMaster%20M3508%20P19%20Brushless%20DC%20Gear%20Motor%20V1.0.pdf
    // https://cdn-hz.robomaster.com/robomasters/public/document/RoboMaster%20C620%20Brushless%20DC%20Motor%20Speed%20Controller%20V1.0.pdf
    current_cmd[i] = (int16_t)lroundf(torque_nm[i] * (16384.0f / (20.0f * 0.3f)));
  }
}
