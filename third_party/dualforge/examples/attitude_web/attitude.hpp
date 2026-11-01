#pragma once
#include <algorithm>
#include <array>
#include <cmath>

namespace dualforge::attitude {
constexpr double kGravity = 9.80665;
constexpr double kPi = 3.14159265358979323846;
struct Vec3 { double x=0, y=0, z=0; };
inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x+b.x,a.y+b.y,a.z+b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
inline Vec3 operator*(Vec3 a, double s) { return {a.x*s,a.y*s,a.z*s}; }
inline double Dot(Vec3 a, Vec3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
inline double Norm(Vec3 v) { return std::sqrt(Dot(v,v)); }
inline Vec3 Cross(Vec3 a, Vec3 b) { return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x}; }
struct Quaternion { double w=1, x=0, y=0, z=0; };
inline Quaternion Normalize(Quaternion q) {
    const double n=std::sqrt(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z);
    return {q.w/n,q.x/n,q.y/n,q.z/n};
}
inline Quaternion Conjugate(Quaternion q) { return {q.w,-q.x,-q.y,-q.z}; }
inline Quaternion operator*(Quaternion a, Quaternion b) {
    return {a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z,
            a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,
            a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,
            a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w};
}
inline Vec3 Rotate(Quaternion q, Vec3 v) {
    const auto r=q*Quaternion{0,v.x,v.y,v.z}*Conjugate(q);
    return {r.x,r.y,r.z};
}
inline Quaternion FromGravity(Vec3 acceleration) {
    const Vec3 up={0,1,0}, a=acceleration*(1.0/Norm(acceleration));
    const auto axis=Cross(a,up);
    if (Dot(a,up)<-0.999999) return {0,1,0,0};
    return Normalize({1+Dot(a,up),axis.x,axis.y,axis.z});
}
// SDL coordinates: X right (pitch), Y up (yaw), Z toward player (roll).
// Body-to-world quaternion; Euler decomposition Ry(yaw)*Rx(pitch)*Rz(roll).
inline std::array<double,3> EulerDegrees(Quaternion q) {
    const double pitch=std::asin(std::clamp(2*(q.w*q.x-q.y*q.z),-1.0,1.0));
    const double yaw=std::atan2(2*(q.x*q.z+q.w*q.y),1-2*(q.x*q.x+q.y*q.y));
    const double roll=std::atan2(2*(q.x*q.y+q.w*q.z),1-2*(q.x*q.x+q.z*q.z));
    return {roll*180/kPi,pitch*180/kPi,yaw*180/kPi};
}

class Estimator {
public:
    explicit Estimator(bool calibrate=true):calibrating_(calibrate) {}
    void Calibrate() { calibrating_=true; calibration_time_=0; calibration_sum_={}; }
    void Zero() { reference_=Conjugate(orientation_); }
    void Update(Vec3 gyro, Vec3 acceleration, double dt) {
        const double magnitude=Norm(acceleration);
        if (!std::isfinite(magnitude) || magnitude<1.0) return;
        if (!initialized_) { orientation_=FromGravity(acceleration); initialized_=true; }
        // Integration uses sensor time; a pause/reconnection must not become a giant step.
        if (!(dt>0 && dt<=0.05) || !std::isfinite(Norm(gyro))) return;
        if (calibrating_) {
            if (Norm(gyro)<0.08 && std::abs(magnitude-kGravity)<0.5) {
                calibration_time_+=dt; calibration_sum_=calibration_sum_+gyro*dt;
                if (calibration_time_>=2.0) {
                    bias_=calibration_sum_*(1.0/calibration_time_); calibrating_=false;
                }
            } else { calibration_time_=0; calibration_sum_={}; }
        }
        const Vec3 measured=acceleration*(1.0/magnitude);
        const Vec3 predicted=Rotate(Conjugate(orientation_),{0,1,0});
        // Mahony-style proportional gravity feedback. Reject strong linear acceleration.
        const double trust=std::clamp(1.0-std::abs(magnitude-kGravity)/1.5,0.0,1.0);
        const Vec3 omega=gyro-bias_+Cross(measured,predicted)*(2.5*trust);
        const double speed=Norm(omega), half=speed*dt*0.5;
        const double scale=speed>1e-12?std::sin(half)/speed:dt*0.5;
        orientation_=Normalize(orientation_*Quaternion{std::cos(half),omega.x*scale,omega.y*scale,omega.z*scale});
    }
    Quaternion orientation() const { return orientation_; }
    Quaternion relative() const { return Normalize(reference_*orientation_); }
    Vec3 bias() const { return bias_; }
    bool initialized() const { return initialized_; }
    bool calibrating() const { return calibrating_; }
    double calibration_progress() const { return std::min(1.0,calibration_time_/2.0); }
private:
    Quaternion orientation_, reference_;
    Vec3 bias_, calibration_sum_;
    double calibration_time_=0;
    bool initialized_=false, calibrating_=true;
};
}
