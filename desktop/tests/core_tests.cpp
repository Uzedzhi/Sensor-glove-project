#include "glove_mouse/config.hpp"
#include "glove_mouse/io.hpp"
#include "glove_mouse/runtime.hpp"
#include "glove_mouse/ipc.hpp"
#include <linux/input.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <fcntl.h>
#include <unistd.h>

#define CHECK(x) do { if (!(x)) throw std::runtime_error(std::string(__func__) + ": " + #x); } while (false)
template<class F> void Fails(F f) { bool failed = false; try { f(); } catch (const std::exception&) { failed = true; } CHECK(failed); }

void ConfigTests()
{
    using namespace glove;
    Config c;
    CHECK(c.x.plane == Plane::XY && c.y.plane == Plane::YZ && c.x.invert && c.y.invert);
    CHECK(c.serial_port == "/dev/ttyACM0" && c.x.gain == 20 && c.deadzone_deg_s == 2);
    c.serial_port = "/dev/a port with spaces"; c.x = {Plane::None, false, .1}; c.y.gain = 200;
    auto encoded = ConfigJson(c); auto decoded = ParseJson(DumpJson(encoded.get()));
    auto copy = ParseConfig(decoded.get());
    CHECK(copy.serial_port == c.serial_port && copy.x.plane == Plane::None && !copy.x.invert && copy.y.gain == 200);
    CHECK(!json_object_object_get(encoded.get(), "enabled") && !json_object_object_get(encoded.get(), "reference"));
    for (double n : {0., 201., std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
        Config bad; bad.x.gain = n; Fails([&] { Validate(bad); });
    }
    Fails([] { Config bad; bad.serial_port.clear(); Validate(bad); });
    Fails([] { Config bad; bad.deadzone_deg_s = -1; Validate(bad); });
    Fails([] { ParsePlane("ZX"); });
    Fails([] { ParseJson("{} trailing"); });
    auto invalid = ConfigJson(c); Put(invalid.get(), "version", Integer(2));
    Fails([&] { ParseConfig(invalid.get()); });
    invalid = ConfigJson(c); Put(invalid.get(), "serial_port", Integer(5));
    Fails([&] { ParseConfig(invalid.get()); });
    char pattern[] = "/tmp/glove-config-test-XXXXXX";
    const char* directory = mkdtemp(pattern); CHECK(directory);
    const std::filesystem::path base(directory), path = base / "config.json";
    CHECK(LoadConfig(path).x.gain == 20);
    SaveConfig(path, c); CHECK(LoadConfig(path).serial_port == c.serial_port);
    const auto before = DumpJson(ConfigJson(LoadConfig(path)).get());
    Config bad = c; bad.x.gain = 0; Fails([&] { SaveConfig(path, bad); });
    CHECK(DumpJson(ConfigJson(LoadConfig(path)).get()) == before);
    std::filesystem::create_directory(base / "occupied");
    Fails([&] { SaveConfig(base / "occupied", c); });
    CHECK(std::filesystem::is_directory(base / "occupied"));
    std::ofstream(path) << "{broken";
    Fails([&] { LoadConfig(path); });
    std::filesystem::remove_all(base);
}

void TelemetryTests()
{
    using namespace glove;
    const std::string line = "I (1234) bno055: Quaternion W=1.0000 X=0.0000 Y=0.0000 Z=0.0000 | Cal SYS=0 GYR=1 ACC=2 MAG=3";
    auto s = ParseSample("\x1b[32m" + line + "\x1b[0m");
    CHECK(s && s->device_ms == 1234 && s->quaternion[0] == 1);
    CHECK((s->calibration == std::array<unsigned,4>{0,1,2,3}));
    CHECK(!ParseSample("I (10) bno055: Timing: poll_hz=100"));
    for (const std::string value : {"0", "NaN", "Inf", "2", "junk"}) {
        std::string bad = line; bad.replace(bad.find("1.0000"), 6, value); CHECK(!ParseSample(bad));
    }
    CHECK(!ParseSample(line + " trailing"));
    CHECK(!ParseSample(line.substr(0, line.size()-1)));
    std::string bad = line; bad.back() = '4'; CHECK(!ParseSample(bad));
    LineFramer f;
    CHECK(f.Feed(line.substr(0,20)).empty());
    auto lines = f.Feed(line.substr(20)+"\r\nboot\n"+line+"\n");
    CHECK(lines.size()==3 && ParseSample(lines[0]) && ParseSample(lines[2]) && !ParseSample(lines[1]));
    CHECK(f.Feed(std::string(65537,'a') + line + "\n").empty());
    CHECK(f.Overflows()==1 && f.Feed(line+"\n").size()==1);
}
glove::Quat Rotation(unsigned axis, double degrees)
{
    const double half = degrees * 3.14159265358979323846 / 360;
    glove::Quat q{std::cos(half),0,0,0}; q[axis+1]=std::sin(half); return q;
}
void MotionTests()
{
    using namespace glove;
    for (const auto plane : {Plane::XY, Plane::XZ, Plane::YZ}) {
        const unsigned axis = plane==Plane::XY ? 2 : plane==Plane::XZ ? 1 : 0;
        Config c; c.x={plane,false,10}; c.y.plane=Plane::None; c.deadzone_deg_s=0;
        Mapper m; m.Step({1,0,0,0},.01,c);
        auto move=m.Step(Rotation(axis,5),.01,c); CHECK(std::abs(move[0]-50)<1e-8 && move[1]==0);
        CHECK(std::abs(m.Step(Rotation(axis,5),.01,c)[0])<1e-8);
        auto q=Rotation(axis,5); for(auto& x:q) x=-x;
        CHECK(std::abs(m.Step(q,.01,c)[0])<1e-8);
        c.x.invert=true; CHECK(std::abs(m.Step(Rotation(axis,10),.01,c)[0]+50)<1e-8);
    }
    Config c; c.x={Plane::XY,false,10}; c.y={Plane::XY,true,2}; c.deadzone_deg_s=0;
    Mapper m; m.Step({1,0,0,0},.01,c); auto move=m.Step(Rotation(2,10),.01,c);
    CHECK(std::abs(move[0]-100)<1e-8 && std::abs(move[1]+20)<1e-8);
    m.Center(Rotation(2,179)); m.Step(Rotation(2,179),.01,c);
    CHECK(std::abs(m.Step(Rotation(2,-179),.01,c)[0]-20)<1e-7);
    const auto reference=Multiply(Rotation(1,40),Rotation(2,60));
    m.Center(reference); m.Step(reference,.01,c);
    CHECK(std::abs(m.Step(Multiply(reference,Rotation(2,5)),.01,c)[0]-50)<1e-7);
    m.ResetReference(); m.Step({1,0,0,0},.01,c);
    CHECK(m.Step(Rotation(2,50),.01,c)[0]==0);
    CHECK(std::abs(m.Step(Rotation(2,51),.01,c)[0]-10)<1e-7);
    m.ResetPrevious(); CHECK(m.Step(Rotation(2,60),.01,c)[0]==0);
    c.deadzone_deg_s=2; CHECK(m.Step(Rotation(2,60.01),.01,c)[0]==0);
}
class FakeDevice : public glove::EventDevice {
public:
    bool open_fail=false, opened=false; int closes=0, failure=0;
    std::vector<input_event> events;
    void Open() override { if(open_fail) throw std::runtime_error("open failed"); opened=true; }
    ssize_t Write(const input_event& e) override {
        if(failure==1) { errno=EAGAIN; return -1; }
        if(failure==2) return sizeof(e)-1;
        events.push_back(e); return sizeof(e);
    }
    void Close() override { opened=false; ++closes; }
};
void OutputTests()
{
    using namespace glove;
    auto device=std::make_unique<FakeDevice>(); auto* d=device.get();
    MouseOutput out(false,std::move(device)); CHECK(out.Prepare());
    for(int i=0;i<3;++i) CHECK(out.MoveBy({.25,0})[0]==0);
    CHECK(out.MoveBy({.25,-2})[0]==1);
    CHECK(d->events.size()==3 && d->events[0].code==REL_X && d->events[1].code==REL_Y && d->events[1].value==-2);
    CHECK(d->events[2].type==EV_SYN && d->events[2].code==SYN_REPORT);
    for(int failure:{1,2}) {
        d->failure=failure; Fails([&]{out.MoveBy({1,0});}); CHECK(!out.Available() && !d->opened);
        d->failure=0; CHECK(out.Prepare());
    }
    auto dry_device=std::make_unique<FakeDevice>(); auto* dry_ptr=dry_device.get();
    MouseOutput dry(true,std::move(dry_device)); CHECK(dry.MoveBy({5,6})[1]==6); CHECK(!dry_ptr->opened && dry_ptr->events.empty());
    auto bad_device=std::make_unique<FakeDevice>(); bad_device->open_fail=true;
    MouseOutput bad(false,std::move(bad_device)); CHECK(!bad.Prepare() && !bad.Error().empty());
}
void SerialTests()
{
    using namespace glove;
    Fd master(posix_openpt(O_RDWR | O_NOCTTY | O_CLOEXEC)); CHECK(master);
    CHECK(grantpt(master.Get())==0 && unlockpt(master.Get())==0);
    const std::string path=ptsname(master.Get());
    SerialReader reader; reader.Open(path);
    Fd competing(open(path.c_str(),O_RDONLY|O_NOCTTY|O_NONBLOCK)); CHECK(!competing && errno==EBUSY);
    CHECK(write(master.Get(),"partial",7)==7); usleep(1000); CHECK(reader.Read().empty());
    CHECK(write(master.Get()," line\r\n",7)==7); usleep(1000);
    auto lines=reader.Read(); CHECK(lines.size()==1 && lines[0]=="partial line");
    reader.Close(); reader.Open(path); CHECK(reader.IsOpen());
    master.Reset(); Fails([&]{reader.Read();}); reader.Close();
}

void RuntimeTests()
{
    using namespace glove;
    Config c; c.deadzone_deg_s=0;
    bool save_fail=false; Config saved;
    Runtime r(c,true,[&](const Config& next){if(save_fail) throw std::runtime_error("Disk failure"); saved=next;});
    CHECK(!Boolean(r.Status(0).get(),"enabled"));
    r.Connected(true);
    CHECK(!r.Ready(0)); Fails([&]{r.Enable(true,0);});
    Sample s{{1,0,0,0},{0,1,2,3},0};
    for(unsigned i=0;i<=110;++i) { s.device_ms=i*10; r.Receive(s,i*.01); }
    CHECK(r.Ready(1.1)); r.Enable(true,1.1);
    s.device_ms=1110; r.Receive(s,1.11); s.device_ms=1120; s.quaternion=Rotation(2,5); r.Receive(s,1.12);
    CHECK(Number(r.Status(1.12).get(),"total_dx")< -99);
    const auto before=Number(r.Status(1.12).get(),"total_dx");
    r.Center(1.12); CHECK(Number(r.Status(1.12).get(),"total_dx")==before);
    save_fail=true; Config next=c; next.x.gain=40; Fails([&]{r.Apply(next);}); CHECK(r.Settings().x.gain==20);
    save_fail=false; r.Apply(next); CHECK(saved.x.gain==40 && r.Settings().x.gain==40);
    r.Tick(1.4); CHECK(!Boolean(r.Status(1.4).get(),"enabled"));
    s.device_ms=1500; r.Receive(s,1.5); CHECK(!Boolean(r.Status(1.5).get(),"enabled") && !r.Ready(1.5));
    for(unsigned i=1;i<=110;++i) { s.device_ms=1500+i*10; r.Receive(s,1.5+i*.01); }
    r.Enable(true,2.6); s.device_ms=0; r.Receive(s,2.61);
    CHECK(!Boolean(r.Status(2.61).get(),"enabled"));
    for(const auto& request : {"{\"version\":2,\"id\":1,\"command\":\"get_status\"}",
            "{\"version\":1,\"id\":2,\"command\":\"unknown\"}","{}"}) {
        auto j=ParseJson(request); CHECK(!Boolean(r.Request(j.get(),2.61).get(),"ok"));
    }
    auto j=ParseJson("{\"version\":1,\"id\":7,\"command\":\"get_config\"}");
    auto reply=r.Request(j.get(),2.61); CHECK(Boolean(reply.get(),"ok") && Number(reply.get(),"id")==7);
}
void IpcTests()
{
    using namespace glove;
    char pattern[]="/tmp/glove-ipc-test-XXXXXX"; CHECK(mkdtemp(pattern));
    const std::string path=std::string(pattern)+"/driver.sock";
    {
        LocalServer server(path); Fails([&]{LocalServer duplicate(path);});
        IpcClient client(path);
        auto handler=[](json_object* j) {auto r=Object(); Put(r.get(),"version",Integer(1)); Put(r.get(),"id",Integer(json_object_get_int64(Required(j,"id",json_type_int)))); Put(r.get(),"ok",Bool(true)); Put(r.get(),"result",Object()); return r;};
        for(int i=0;i<10 && !client.Connected();++i) {client.Tick(); server.Process(handler); usleep(1000);}
        CHECK(client.Connected() && client.Send("get_status"));
        bool received=false;
        for(int i=0;i<50 && !received;++i) {
            server.Process(handler);
            for(auto& reply:client.Tick()) {CHECK(reply.command=="get_status" && Boolean(reply.body.get(),"ok")); received=true;}
            usleep(1000);
        }
        CHECK(received);
    }
    CHECK(!std::filesystem::exists(path)); std::filesystem::remove_all(pattern);
}
int main()
{
    try { ConfigTests(); TelemetryTests(); MotionTests(); OutputTests(); SerialTests(); RuntimeTests(); IpcTests(); std::cout << "PASS: config, telemetry, motion, output, serial, runtime, IPC\n"; }
    catch (const std::exception& e) { std::cerr << "FAIL: " << e.what() << '\n'; return 1; }
}
