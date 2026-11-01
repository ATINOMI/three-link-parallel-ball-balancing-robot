#include "attitude.hpp"
#include <cstdlib>
#include <iostream>
#include <stdexcept>
using namespace dualforge::attitude;
void Check(bool ok,const char* text) { if(!ok) throw std::runtime_error(text); }
bool Near(double a,double b,double tolerance=1e-5) { return std::abs(a-b)<tolerance; }
int main() try {
    const Quaternion tilt=Quaternion{std::cos(.15),std::sin(.15),0,0}*Quaternion{std::cos(.1),0,0,std::sin(.1)};
    const auto original=EulerDegrees(tilt);
    for(double yawAngle:{-.9,0.,1.4}){
        const auto turned=EulerDegrees(Quaternion{std::cos(yawAngle/2),0,std::sin(yawAngle/2),0}*tilt);
        Check(Near(original[0],turned[0])&&Near(original[1],turned[1]),"world yaw cannot change control roll/pitch");
    }
    Estimator flat(false);for(int i=0;i<2000;i++)flat.Update({},{0,kGravity,0},.005);
    Check(Near(flat.orientation().w,1),"stationary identity");
    Estimator pitch(false);pitch.Update({},{0,kGravity/std::sqrt(2.0),-kGravity/std::sqrt(2.0)},.005);
    Check(Near(EulerDegrees(pitch.relative())[1],45),"pitch SDL axis/sign");
    Estimator roll(false);roll.Update({},{kGravity*.5,kGravity*std::sqrt(.75),0},.005);
    Check(Near(EulerDegrees(roll.relative())[0],30),"roll SDL axis/sign");
    Estimator yaw(false);for(int i=0;i<500;i++)yaw.Update({0,1,0},{0,kGravity,0},.005);
    Check(Near(EulerDegrees(yaw.relative())[2],2.5*180/kPi),"sensor-time yaw integration");
    yaw.Zero();for(auto angle:EulerDegrees(yaw.relative()))Check(Near(angle,0),"relative zero");
    auto before=yaw.orientation();yaw.Update({1,2,3},{0,kGravity,0},.2);
    Check(Near(yaw.orientation().y,before.y),"pause rejected");
    Estimator settle(false);settle.Update({},{0,kGravity,0},.005);
    for(int i=0;i<2000;i++)settle.Update({},{0,kGravity*std::sqrt(.75),-kGravity*.5},.005);
    Check(Near(EulerDegrees(settle.relative())[1],30,.001),"gravity correction convergence");
    auto saved=settle.orientation();for(int i=0;i<100;i++)settle.Update({},{kGravity*2,0,0},.005);
    Check(Near(settle.orientation().x,saved.x),"linear acceleration rejection");
    Estimator calibration;for(int i=0;i<430;i++)calibration.Update({.012,.008,-.009},{0,kGravity,0},.005);
    Check(!calibration.calibrating()&&Near(calibration.bias().y,.008),"stationary gyro bias");
    calibration.Zero();for(int i=0;i<2000;i++)calibration.Update({.012,.008,-.009},{0,kGravity,0},.005);
    Check(std::abs(EulerDegrees(calibration.relative())[2])<.1,"calibrated yaw holds");
    calibration.Calibrate();for(int i=0;i<200;i++)calibration.Update({0,1,0},{0,kGravity,0},.005);
    Check(calibration.calibrating()&&calibration.calibration_progress()==0,"motion restarts calibration");
    Estimator tumble(false);Quaternion truth;
    for(int i=0;i<8000;i++) {
        Vec3 omega{.4,-.7,.3};const double speed=Norm(omega),half=speed*.005*.5;
        const auto accel=Rotate(Conjugate(truth),{0,kGravity,0});
        tumble.Update(omega,accel,.005);
        truth=Normalize(truth*Quaternion{std::cos(half),omega.x*std::sin(half)/speed,omega.y*std::sin(half)/speed,omega.z*std::sin(half)/speed});
    }
    auto q=tumble.orientation();Check(Near(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z,1),"unit quaternion");
    Check(std::abs(q.w*truth.w+q.x*truth.x+q.y*truth.y+q.z*truth.z)>.99999,"full 3D tumbling follows truth");
    std::cout<<"PASS: gravity alignment, SDL axes, yaw integration, reference zero, calibration/bias, acceleration rejection and 40s full 3D trajectory.\n";
    return 0;
} catch(const std::exception& e) { std::cerr<<e.what()<<'\n';return 1; }
