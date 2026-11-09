#include "glove_mouse/config.hpp"
#include "glove_mouse/ipc.hpp"
#include "glove_mouse/runtime.hpp"
#include <csignal>
#include <iostream>
#include <poll.h>

namespace {
volatile std::sig_atomic_t Interrupted=0;
void Signal(int) { Interrupted=1; }
}
int main(int argc,char** argv)
{
    using namespace glove;
    try {
        std::string port,socket_path; std::filesystem::path profile; bool dry=false;
        for(int i=1;i<argc;++i) {
            const std::string arg=argv[i];
            auto value=[&]() -> std::string { if(++i>=argc) throw std::runtime_error("Missing value for "+arg); return argv[i]; };
            if(arg=="--help" || arg=="-h") {
                std::cout<<"Usage: glove-mouse-driver [--port PATH] [--socket PATH] [--config PATH] [--dry-run]\n"
                    "Starts paused. Use glove-mouse-config to enable control.\n"
                    "--dry-run computes relative movement without opening /dev/uinput.\n";
                return 0;
            } else if(arg=="--port") port=value();
            else if(arg=="--socket") socket_path=value();
            else if(arg=="--config") profile=value();
            else if(arg=="--dry-run") dry=true;
            else throw std::runtime_error("Unknown argument: "+arg);
        }
        if(profile.empty()) profile=DefaultConfigPath();
        if(socket_path.empty()) socket_path=DefaultSocketPath();
        Config config; std::string profile_error;
        try { config=LoadConfig(profile); }
        catch(const std::exception& e) { profile_error="Profile: "+std::string(e.what())+"; using defaults"; }
        if(!port.empty()) config.serial_port=port;
        Validate(config);
        // Bind before creating any output device; a second instance cannot replace the first.
        LocalServer server(socket_path);
        Runtime runtime(config,dry,[&](const Config& c){SaveConfig(profile,c);});
        runtime.ProfileError(profile_error);
        SerialReader serial; std::string opened_port=config.serial_port; double retry_at=0;
        std::signal(SIGINT,Signal); std::signal(SIGTERM,Signal);
        std::cerr<<"glove-mouse-driver: "<<socket_path<<(dry?" (dry-run)":"")<<", control paused\n";
        if(!profile_error.empty()) std::cerr<<profile_error<<'\n';
        while(!Interrupted && !runtime.StopRequested()) {
            const double now=Monotonic(); runtime.Tick(now);
            if(runtime.Settings().serial_port!=opened_port) {
                serial.Close(); opened_port=runtime.Settings().serial_port; retry_at=0;
                runtime.Connected(false,"Serial port changed; reconnecting");
            }
            if(!serial.IsOpen() && now>=retry_at) {
                retry_at=now+1;
                try { serial.Open(opened_port); runtime.Connected(true); }
                catch(const std::exception& e) { runtime.Connected(false,e.what()); }
            }
            if(serial.IsOpen()) {
                try {
                    for(const auto& line:serial.Read()) {
                        if(auto sample=ParseSample(line)) runtime.Receive(*sample,Monotonic());
                        else if(line.find("Quaternion W=")!=std::string::npos) runtime.Rejected();
                    }
                } catch(const std::exception& e) {
                    serial.Close(); runtime.Connected(false,e.what()); retry_at=Monotonic()+1;
                }
            }
            runtime.Tick(Monotonic());
            server.Process([&](json_object* request){ return runtime.Request(request,Monotonic()); });
            if(!runtime.StopRequested()) poll(nullptr,0,5);
        }
        std::cerr<<"glove-mouse-driver: stopped\n";
    } catch(const std::exception& e) { std::cerr<<"glove-mouse-driver: "<<e.what()<<'\n'; return 1; }
}
