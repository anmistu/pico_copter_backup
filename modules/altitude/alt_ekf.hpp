#ifndef ALT_EKF_1D_HPP
#define ALT_EKF_1D_HPP

#include <Eigen/Dense>
#include <math.h>

// 1D Altitude EKF (Down-positive, NED convention)
// State x = [ z_D, v_D, b_a ]^T
//   z_D : position (Down, m)
//   v_D : velocity (Down, m/s)
//   b_a : accel bias along Down (m/s^2)
// Input u = f_D : specific force along Down (m/s^2) = a_D - g
// Meas  z = z_D  (from ToF converted to Down: z_D = -h)
class AltEKF1D { 
public:
  AltEKF1D() { reset(); }

  void reset() {
    x_.setZero();
    P_.setIdentity(); P_ *= 1e-2f;
    Q_.setZero();
    R_ = 1e-3f;
    nis_ = 0.0f;
  }

  void init(float z0, float v0, float ba0,
            float pz0=0.05f, float pv0=0.1f, float pba0=0.5f) {
    x_ << z0, v0, ba0;
    P_.setZero();
    P_(0,0) = fmaxf(pz0, 1e-8f);
    P_(1,1) = fmaxf(pv0, 1e-8f);
    P_(2,2) = fmaxf(pba0,1e-8f);
    nis_ = 0.0f;
  }

  // Set process / measurement noise (diagonal Q)
  void set_noise(float qz, float qv, float qba, float r_meas) {
    Q_.setZero();
    Q_(0,0) = fmaxf(qz , 1e-12f);
    Q_(1,1) = fmaxf(qv , 1e-12f);
    Q_(2,2) = fmaxf(qba, 1e-12f);
    R_ = fmaxf(r_meas, 1e-12f);
  }

  // Predict with dt [s] and specific force fD = a_D - g  [m/s^2]
  void predict(float dt, float fD) {
    using namespace Eigen;
    // x_{k+1} = F x_k + B u ; u = fD
    const float dt2 = dt*dt*0.5f;
    Matrix3f F; F.setIdentity();
    F(0,1) = dt;
    F(1,2) = -dt; // v' = v + (fD - b_a)*dt  -> -ba*dt

    Vector3f B; 
    B << dt2, dt, 0.0f;  // z += 0.5*a*dt^2; v += a*dt

    x_ = F * x_ + B * fD;

    // Covariance: P' = F P F^T + Qd (simple discrete approx)
    Matrix3f FP = F * P_;
    P_ = FP * F.transpose() + Q_;
    symmetrize_(P_);
  }

  // Update with z_meas = z_D + v, returns NIS for gating
  // If gate_th > 0 and NIS > gate_th, update is skipped and returns false
  bool updateZ(float z_meas, float gate_th = 0.0f) {
    using namespace Eigen;
    // H = [1 0 0]
    const Vector3f H(1.0f, 0.0f, 0.0f);
    const float y = z_meas - H.dot(x_);
    const float S = H.transpose().dot(P_ * H) + R_;
    nis_ = (S > 1e-12f) ? (y*y / S) : 0.0f;

    if (gate_th > 0.0f && nis_ > gate_th) return false;

    const Vector3f K = (P_ * H) / S;     // 3x1
    x_ += K * y;
    P_ -= K * H.transpose() * P_;
    symmetrize_(P_);
    return true;
  }

  inline float z()   const { return x_(0); }
  inline float v()   const { return x_(1); }
  inline float ba()  const { return x_(2); }
  inline float nis() const { return nis_;   }
  inline const Eigen::Matrix3f& P() const { return P_; }

private:
  Eigen::Vector3f x_;
  Eigen::Matrix3f P_;
  Eigen::Matrix3f Q_;
  float R_;
  float nis_;

  static inline void symmetrize_(Eigen::Matrix3f& M) {
    M = 0.5f * (M + M.transpose());
  }
};

#endif // ALT_EKF_1D_HPP
