#include <winsock2.h>
#include <windows.h>
#include <shellapi.h>
#include <dualforge/dualforge.hpp>
#include <SDL3/SDL.h>
#include "attitude.hpp"
#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace {
using namespace dualforge::attitude;
volatile std::sig_atomic_t stopped=0;
std::string Quote(const std::string& text) {
    std::string out="\"";
    for (unsigned char c:text) {
        if(c=='"'||c=='\\') {out+='\\';out+=static_cast<char>(c);}
        else if(c=='\n') out+="\\n";
        else if(c=='\r') out+="\\r";
        else if(c>=32) out+=static_cast<char>(c);
    }
    return out+'"';
}
struct Shared {
    std::mutex mutex;
    std::string snapshot=R"({"app":"dual-control-3rrs","connected":false,"ready":false,"message":"Waiting for USB DualSense"})";
    std::atomic<int> feedback{0};
    std::atomic<Uint64> feedback_at{0};
};
class Server {
public:
    Server(int port,std::filesystem::path root,Shared& data):root_(std::move(root)),data_(data) {
        WSADATA wsa{};
        if(WSAStartup(MAKEWORD(2,2),&wsa)) throw std::runtime_error("WSAStartup failed");
        listener_=socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
        sockaddr_in address{};address.sin_family=AF_INET;address.sin_port=htons(static_cast<u_short>(port));address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
        if(bind(listener_,reinterpret_cast<sockaddr*>(&address),sizeof(address))||listen(listener_,8)) {
            closesocket(listener_);WSACleanup();throw std::runtime_error("Cannot bind localhost port "+std::to_string(port));
        }
        worker_=std::thread([this]{Serve();});
    }
    ~Server() {quit_=true;worker_.join();closesocket(listener_);WSACleanup();}
private:
    void Respond(SOCKET client,int status,const std::string& type,const std::string& body) {
        const std::string response="HTTP/1.1 "+std::to_string(status)+(status==200?" OK":" Not Found")+
            "\r\nContent-Type: "+type+"\r\nContent-Length: "+std::to_string(body.size())+
            "\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n"+body;
        std::size_t written=0;
        while(written<response.size()) {const int n=send(client,response.data()+written,static_cast<int>(response.size()-written),0);if(n<=0)break;written+=n;}
    }
    void Handle(SOCKET client) {
        DWORD timeout=1000;setsockopt(client,SOL_SOCKET,SO_RCVTIMEO,reinterpret_cast<char*>(&timeout),sizeof(timeout));
        setsockopt(client,SOL_SOCKET,SO_SNDTIMEO,reinterpret_cast<char*>(&timeout),sizeof(timeout));
        std::string request;char buffer[2048];
        while(request.find("\r\n\r\n")==std::string::npos&&request.size()<8192) {
            const int n=recv(client,buffer,sizeof(buffer),0);if(n<=0)return;request.append(buffer,n);
        }
        std::istringstream line(request);std::string method,path;line>>method>>path;
        if(method=="GET"&&path=="/api/state") {
            std::string json;{std::lock_guard<std::mutex> lock(data_.mutex);json=data_.snapshot;}
            Respond(client,200,"application/json; charset=utf-8",json);return;
        }
        if(method=="POST"&&(path=="/api/feedback/normal"||path=="/api/feedback/invalid"||path=="/api/feedback/idle")) {
            data_.feedback=path=="/api/feedback/invalid"?2:path=="/api/feedback/normal"?1:0;
            data_.feedback_at=SDL_GetTicksNS();
            Respond(client,200,"application/json","{\"ok\":true}");return;
        }
        std::string file;
        for(const std::string asset:{"index.html","app.js","scene.js","core.js","serial.js","kinematics.js","control.js","style.css"})
            if(path=="/"+asset||(path=="/"&&asset=="index.html"))file=asset;
        if(method=="GET"&&!file.empty()) {
            std::ifstream stream(root_/file,std::ios::binary);
            if(stream) {std::ostringstream content;content<<stream.rdbuf();Respond(client,200,file=="index.html"?"text/html; charset=utf-8":file=="style.css"?"text/css":"application/javascript",content.str());return;}
        }
        Respond(client,404,"text/plain","Not found");
    }
    void Serve() {
        while(!quit_) {
            fd_set read;FD_ZERO(&read);FD_SET(listener_,&read);timeval wait{0,100000};
            if(select(0,&read,nullptr,nullptr,&wait)<=0)continue;
            const SOCKET client=accept(listener_,nullptr,nullptr);if(client==INVALID_SOCKET)continue;
            Handle(client);closesocket(client);
        }
    }
    std::filesystem::path root_;
    Shared& data_;
    SOCKET listener_=INVALID_SOCKET;
    std::atomic<bool> quit_{false};
    std::thread worker_;
};
std::filesystem::path AssetRoot() {
    wchar_t path[32768];const auto length=GetModuleFileNameW(nullptr,path,32768);
    return std::filesystem::path(std::wstring(path,length)).parent_path()/"web";
}
Vec3 Vector(const float* values) {return {values[0],values[1],values[2]};}
}

int main(int argc,char** argv) try {
    int port=8877,seconds=0;bool browser=true,demo=false;
    for(int i=1;i<argc;i++) {
        const std::string arg=argv[i];
        if(arg=="--help") {std::cout<<"DualSense 3-RRS platform control\n--port N --seconds N --no-browser --demo\n";return 0;}
        if(arg=="--no-browser")browser=false;
        else if(arg=="--demo")demo=true;
        else if((arg=="--port"||arg=="--seconds")&&i+1<argc) {const int n=std::stoi(argv[++i]);if(arg=="--port")port=n;else seconds=n;}
        else throw std::runtime_error("Unknown argument: "+arg);
    }
    if(port<1024||port>65535||seconds<0)throw std::runtime_error("Invalid port or duration");
    std::signal(SIGINT,[](int){stopped=1;});std::signal(SIGTERM,[](int){stopped=1;});
    Shared shared;Server server(port,AssetRoot(),shared);
    dualforge::Context context(dualforge::EventMode::External);
    if(!context.status())throw std::runtime_error(context.status().message);
    Estimator estimator;std::shared_ptr<dualforge::Controller> controller;
    std::string name,message="Connect DualSense with USB";
    Vec3 gyro,acceleration;
    Uint64 previous_sensor=0,last_gyro=0,last_accel=0;
    const Uint64 start=SDL_GetTicksNS();Uint64 next_scan=0,last_publish=0,rate_start=start;
    std::uint64_t total_samples=0,rate_samples=0;double hz=0;
    std::uint64_t circle_seq=0,session=0,up_seq=0,down_seq=0;
    bool circle_down=false,up_down=false,down_down=false;
    int previous_feedback=-1;
    Uint64 last_rumble=0;
    const auto demo_orientation=[](double t) {
        const double pitch=.35*std::sin(t*.7),roll=.45*std::sin(t*.45),yaw=t*.22;
        return Quaternion{std::cos(yaw/2),0,std::sin(yaw/2),0}*Quaternion{std::cos(pitch/2),std::sin(pitch/2),0,0}*Quaternion{std::cos(roll/2),0,0,std::sin(roll/2)};
    };
    const std::string url="http://127.0.0.1:"+std::to_string(port)+"/";
    std::cout<<"DualSense 3-RRS: "<<url<<"\nKeep the controller still for 2 seconds to calibrate. Ctrl+C to stop.\n"<<std::flush;
    if(browser)ShellExecuteW(nullptr,L"open",std::wstring(url.begin(),url.end()).c_str(),nullptr,nullptr,SW_SHOWNORMAL);
    while(!stopped) {
        const Uint64 now=SDL_GetTicksNS();const double elapsed=(now-start)*1e-9;
        if(seconds&&elapsed>=seconds)break;
        if(!demo&&!controller&&now>=next_scan) {
            next_scan=now+1000000000ULL;
            for(const auto& device:context.enumerate()) {
                auto opened=context.open(device.id);
                if(!opened.status){message=opened.status.message;continue;}
                const auto caps=opened.controller->capabilities();
                if(caps.gyro!=dualforge::Availability::Available||caps.accelerometer!=dualforge::Availability::Available) {message="Device IMU is unavailable";opened.controller->close();continue;}
                controller=opened.controller;session++;circle_down=up_down=down_down=false;previous_feedback=-1;name=controller->device_info().name;message="";
                estimator=Estimator{};previous_sensor=last_gyro=last_accel=0;total_samples=rate_samples=0;rate_start=now;
                break;
            }
        }
        SDL_Event event;
        while(SDL_PollEvent(&event)) {
            const auto status=context.handle_event(event);if(!status)throw std::runtime_error(status.message);
            if(demo||!controller||event.type!=SDL_EVENT_GAMEPAD_SENSOR_UPDATE||event.gsensor.which!=controller->device_info().id)continue;
            const auto& sensor=event.gsensor;
            if(sensor.sensor==SDL_SENSOR_ACCEL) {acceleration=Vector(sensor.data);last_accel=sensor.timestamp;}
            if(sensor.sensor==SDL_SENSOR_GYRO) {
                gyro=Vector(sensor.data);last_gyro=sensor.timestamp;
                const Uint64 timestamp=sensor.sensor_timestamp?sensor.sensor_timestamp:sensor.timestamp;
                const double dt=previous_sensor&&timestamp>previous_sensor?(timestamp-previous_sensor)*1e-9:0;
                previous_sensor=timestamp;
                const auto sample_now=SDL_GetTicksNS();
                if(last_accel&&sample_now>=last_accel&&sample_now-last_accel<250000000ULL)estimator.Update(gyro,acceleration,dt);
                total_samples++;rate_samples++;
            }
        }
        const auto refresh=context.refresh();if(!refresh)throw std::runtime_error(refresh.message);
        if(controller&&controller->is_connected()) {
            const auto& state=controller->state();
            if(state.buttons.circle&&!circle_down)circle_seq++;
            circle_down=state.buttons.circle;
            // Rising edges only: holding the D-pad never repeats a height step.
            if(state.buttons.up&&!up_down&&!state.buttons.down)up_seq++;
            if(state.buttons.down&&!down_down&&!state.buttons.up)down_seq++;
            up_down=state.buttons.up;down_down=state.buttons.down;
            // Only the SDL main thread performs controller output operations.
            const Uint64 feedback_at=shared.feedback_at.load();
            const int feedback=feedback_at&&now>=feedback_at&&now-feedback_at<350000000ULL?shared.feedback.load():0;
            if(feedback!=previous_feedback) {
                if(feedback==2)controller->set_led(230,35,25);
                else controller->set_led(30,90,220);
                if(feedback!=2)controller->stop_rumble();
                previous_feedback=feedback;
            }
            if(feedback==2&&now-last_rumble>=100000000ULL) {
                controller->set_rumble(14000,24000,200);last_rumble=now;
            }
        }
        if(controller&&!controller->is_connected()) {controller.reset();message="Controller disconnected";last_gyro=last_accel=0;}
        if(demo) {
            // Explicit synthetic mode for viewing/testing; no controller opened.
            gyro={};acceleration={0,kGravity,0};last_gyro=last_accel=now;
        }
        if(now-rate_start>=500000000ULL) {hz=rate_samples/((now-rate_start)*1e-9);rate_samples=0;rate_start=now;}
        const auto publish_now=SDL_GetTicksNS();
        if(publish_now-last_publish>=16000000ULL) {
            last_publish=publish_now;const bool connected=demo||static_cast<bool>(controller);
            const double age=last_gyro&&publish_now>=last_gyro?(publish_now-last_gyro)*1e-6:1e9;
            const bool ready=demo||(connected&&estimator.initialized()&&age<250&&last_accel&&publish_now-last_accel<250000000ULL);
            auto q=estimator.orientation();
            if(demo) {
                q=demo_orientation(elapsed);
            }
            const auto angles=EulerDegrees(q);
            const auto bias=estimator.bias();
            std::ostringstream json;json<<std::fixed<<std::setprecision(7);
            json<<"{\"app\":\"dual-control-3rrs\",\"connected\":"<<(connected?"true":"false")<<",\"ready\":"<<(ready?"true":"false")
                <<",\"demo\":"<<(demo?"true":"false")<<",\"device\":"<<Quote(demo?"Synthetic orientation":name)<<",\"message\":"<<Quote(message)
                <<",\"calibrating\":"<<(!demo&&estimator.calibrating()?"true":"false")<<",\"calibration\":"<<(demo?1:estimator.calibration_progress())
                <<",\"quaternion\":["<<q.w<<','<<q.x<<','<<q.y<<','<<q.z<<"],\"angles\":["<<angles[0]<<','<<angles[1]<<','<<angles[2]
                <<"],\"gyro\":["<<gyro.x<<','<<gyro.y<<','<<gyro.z<<"],\"accel\":["<<acceleration.x<<','<<acceleration.y<<','<<acceleration.z
                <<"],\"bias\":["<<bias.x<<','<<bias.y<<','<<bias.z<<"],\"hz\":"<<hz<<",\"samples\":"<<total_samples<<",\"age_ms\":"<<age
                <<",\"session\":"<<session<<",\"circle_seq\":"<<circle_seq
                <<",\"input_version\":2,\"dpad_up_seq\":"<<up_seq<<",\"dpad_down_seq\":"<<down_seq<<'}';
            std::lock_guard<std::mutex> lock(shared.mutex);shared.snapshot=json.str();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if(controller){controller->stop_rumble();controller->set_led(0,0,0);}
    return 0;
} catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
