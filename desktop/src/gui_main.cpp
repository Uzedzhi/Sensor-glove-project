#include "glove_mouse/ipc.hpp"
#include "raylib.h"
#include <algorithm>
#include <cmath>
#include <deque>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace {
using namespace glove;
constexpr Color Background{13,19,29,255}, Card{24,33,47,255}, Border{46,59,76,255};
constexpr Color Ink{233,239,248,255}, Muted{144,163,184,255}, Accent{75,211,189,255};
constexpr Color Blue{92,155,255,255}, Amber{247,189,98,255}, Red{241,119,132,255};
std::string Format(double n,int precision=1)
{
    std::ostringstream s; s<<std::fixed<<std::setprecision(precision)<<n; return s.str();
}
std::string FieldNumber(double n) { std::ostringstream s; s<<std::setprecision(8)<<n; return s.str(); }
double FieldValue(const std::string& s)
{
    std::size_t end=0; const double n=std::stod(s,&end);
    if(end!=s.size() || !std::isfinite(n)) throw std::runtime_error("Enter a finite numeric value");
    return n;
}
bool Flag(json_object* j,const char* key)
{
    return j && json_object_get_boolean(json_object_object_get(j,key));
}
double Value(json_object* j,const char* key)
{
    return j ? json_object_get_double(json_object_object_get(j,key)) : 0;
}
std::string Label(json_object* j,const char* key)
{
    auto* v=j ? json_object_object_get(j,key) : nullptr;
    return v && json_object_is_type(v,json_type_string) ? json_object_get_string(v) : "";
}
class Ui {
public:
    Font font=GetFontDefault(); bool custom=false; Vector2 mouse{}; int focus=-1; bool selected=false;
    Ui()
    {
        for(const auto* path:{"/usr/share/fonts/TTF/DejaVuSans.ttf","/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"}) {
            if(!FileExists(path)) continue;
            auto loaded=LoadFontEx(path,32,nullptr,0);
            if(loaded.texture.id) {font=loaded;custom=true;SetTextureFilter(font.texture,TEXTURE_FILTER_BILINEAR);break;}
        }
    }
    ~Ui() {if(custom) UnloadFont(font);}
    void Text(float x,float y,float size,const std::string& text,Color color=Ink)
    {DrawTextEx(font,text.c_str(),{x,y},size,0,color);}
    std::string Fit(std::string text,float width,float size)
    {
        if(MeasureTextEx(font,text.c_str(),size,0).x<=width) return text;
        while(!text.empty() && MeasureTextEx(font,(text+"...").c_str(),size,0).x>width) text.pop_back();
        return text+"...";
    }
    void Panel(Rectangle r) {DrawRectangleRounded(r,.045f,10,Card);DrawRectangleRoundedLinesEx(r,.045f,10,1,Border);}
    void Badge(Rectangle r,const std::string& text,Color color)
    {DrawRectangleRounded(r,.35f,8,Fade(color,.12f));Text(r.x+12,r.y+8,13,text,color);}
    bool Button(Rectangle r,const std::string& text,Color color=Blue,bool enabled=true,bool solid=false)
    {
        const bool hover=enabled && CheckCollisionPointRec(mouse,r);
        if(hover) SetMouseCursor(MOUSE_CURSOR_POINTING_HAND);
        DrawRectangleRounded(r,.16f,8,enabled ? (solid ? Fade(color,hover?1.f:.85f) : Fade(color,hover?.24f:.12f)) : Fade(Border,.35f));
        DrawRectangleRoundedLinesEx(r,.16f,8,1,enabled?Fade(color,.65f):Border);
        const auto label=Fit(text,r.width-12,16);
        const float w=MeasureTextEx(font,label.c_str(),16,0).x;
        Text(r.x+(r.width-w)/2,r.y+(r.height-16)/2,16,label,enabled?(solid?Background:color):Muted);
        return hover && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
    }
    bool Edit(int id,Rectangle r,std::string& value,bool enabled=true,bool numeric=false)
    {
        const bool hover=enabled && CheckCollisionPointRec(mouse,r);
        if(hover) SetMouseCursor(MOUSE_CURSOR_IBEAM);
        if(hover && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {focus=id;selected=false;}
        bool changed=false;
        if(enabled && focus==id) {
            const bool ctrl=IsKeyDown(KEY_LEFT_CONTROL)||IsKeyDown(KEY_RIGHT_CONTROL);
            if(ctrl && IsKeyPressed(KEY_A)) selected=true;
            if(IsKeyPressed(KEY_BACKSPACE) || IsKeyPressedRepeat(KEY_BACKSPACE)) {
                if(selected) {value.clear();selected=false;changed=true;}
                else if(!value.empty()) {do {const auto c=static_cast<unsigned char>(value.back());value.pop_back();if((c&0xc0)!=0x80) break;} while(!value.empty());changed=true;}
            }
            for(int c=GetCharPressed();c;c=GetCharPressed()) {
                if(ctrl || c<32 || (numeric && !(c>='0'&&c<='9') && c!='.' && c!='-' && c!='+' && c!='e' && c!='E')) continue;
                if(selected) {value.clear();selected=false;}
                if(value.size()<(numeric?24u:4096u)) {int length=0;const char* utf8=CodepointToUTF8(c,&length);value.append(utf8,static_cast<std::size_t>(length));changed=true;}
            }
        }
        DrawRectangleRounded(r,.12f,8,Color{16,24,36,255});
        DrawRectangleRoundedLinesEx(r,.12f,8,focus==id?2:1,focus==id?Blue:Border);
        const auto text=Fit(value,r.width-20,18);
        if(focus==id && selected) DrawRectangleRec({r.x+8,r.y+8,std::min(r.width-16,MeasureTextEx(font,text.c_str(),18,0).x+4),23},Fade(Blue,.35f));
        Text(r.x+10,r.y+(r.height-18)/2,18,text,enabled?Ink:Muted);
        return changed;
    }
};
struct Draft {
    Config config;
    std::string port,gain_x,gain_y,deadzone;
    bool dirty=false;
    Draft() {Assign({});}
    void Assign(const Config& c)
    {config=c;port=c.serial_port;gain_x=FieldNumber(c.x.gain);gain_y=FieldNumber(c.y.gain);deadzone=FieldNumber(c.deadzone_deg_s);dirty=false;}
    Config Read() const
    {auto c=config;c.serial_port=port;c.x.gain=FieldValue(gain_x);c.y.gain=FieldValue(gain_y);c.deadzone_deg_s=FieldValue(deadzone);Validate(c);return c;}
};
void AxisRow(Ui& ui,float y,const std::string& name,AxisConfig& c,std::string& gain,int id,bool enabled,bool& dirty)
{
    ui.Text(44,y+13,18,name);
    unsigned i=0;
    for(auto plane:{Plane::XY,Plane::XZ,Plane::YZ,Plane::None}) {
        const char* label=plane==Plane::None?"OFF":PlaneName(plane);
        if(ui.Button({148.f+66*i,y,58,44},label,c.plane==plane?Accent:Muted,enabled,c.plane==plane)) {c.plane=plane;dirty=true;}
        ++i;
    }
    if(ui.Button({432,y,74,44},c.invert?"ON":"OFF",c.invert?Accent:Muted,enabled,c.invert)) {c.invert=!c.invert;dirty=true;}
    dirty=ui.Edit(id,{536,y,136,44},gain,enabled,true)||dirty;
}
int Run(const std::string& socket_path)
{
    SetTraceLogLevel(LOG_WARNING);
    SetConfigFlags(FLAG_WINDOW_RESIZABLE|FLAG_MSAA_4X_HINT);
    InitWindow(1120,840,"Glove mouse - Control");
    if(!IsWindowReady()) throw std::runtime_error("Cannot open raylib window");
    SetWindowMinSize(1120,840); SetTargetFPS(60); SetExitKey(KEY_NULL);
    {
        Ui ui; IpcClient client(socket_path); Draft draft; Config active;
        Json status; bool have_config=false; double last_update=-1,next_status=0;
        std::string notice,preview_session; std::int64_t preview_epoch=-1,total_x=0,total_y=0;
        Vector2 preview{}; std::deque<Vector2> trail;
        while(!WindowShouldClose()) {
            if((IsKeyDown(KEY_LEFT_CONTROL)||IsKeyDown(KEY_RIGHT_CONTROL)) && IsKeyPressed(KEY_Q)) break;
            const double now=Monotonic();
            auto replies=client.Tick();
            for(auto& reply:replies) {
                try {
                    if(!Boolean(reply.body.get(),"ok")) {
                        notice=String(Required(reply.body.get(),"error",json_type_object),"message"); continue;
                    }
                    auto* result=Required(reply.body.get(),"result",json_type_object);
                    status=Json(json_object_get(result)); last_update=now;
                    active=ParseConfig(Required(result,"config",json_type_object));
                    if(!have_config || !draft.dirty || reply.command=="set_config") {
                        if(!draft.dirty || reply.command=="set_config") draft.Assign(active);
                        have_config=true;
                    }
                    if(reply.command=="set_config") notice="Settings saved and applied.";
                    if(reply.command=="recenter") notice="Reference orientation updated.";
                    if(reply.command=="set_enabled") notice=Flag(result,"enabled")?"Motion enabled.":"Motion paused.";
                    const auto session=Label(result,"session_id");
                    const auto epoch=json_object_get_int64(json_object_object_get(result,"reference_epoch"));
                    const auto x=json_object_get_int64(json_object_object_get(result,"total_dx"));
                    const auto y=json_object_get_int64(json_object_object_get(result,"total_dy"));
                    if(session!=preview_session || epoch!=preview_epoch) {preview={};trail.clear();}
                    else {
                        preview.x=std::clamp(preview.x+static_cast<float>(x-total_x)/3,-150.f,150.f);
                        preview.y=std::clamp(preview.y+static_cast<float>(y-total_y)/3,-98.f,98.f);
                        if(x!=total_x || y!=total_y) {trail.push_back(preview);if(trail.size()>80) trail.pop_front();}
                    }
                    preview_session=session;preview_epoch=epoch;total_x=x;total_y=y;
                } catch(const std::exception& e) {notice=e.what();}
            }
            const bool online=client.Connected() && last_update>=0 && now-last_update<.5;
            if(client.Connected() && now>=next_status && !client.Pending("get_status")) {client.Send("get_status");next_status=now+.1;}
            const bool applying=client.Pending("set_config"), editable=!applying;
            const bool enabled=online && Flag(status.get(),"enabled"), dry=online && Flag(status.get(),"dry_run");
            const bool ready=online && Flag(status.get(),"ready");
            const bool control_pending=client.Pending("set_enabled") || client.Pending("recenter");
            const float scale=std::min(GetScreenWidth()/1120.f,GetScreenHeight()/840.f);
            const Vector2 offset{(GetScreenWidth()-1120*scale)/2,(GetScreenHeight()-840*scale)/2};
            const auto mouse=GetMousePosition(); ui.mouse={(mouse.x-offset.x)/scale,(mouse.y-offset.y)/scale};
            if(IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {ui.focus=-1;ui.selected=false;}
            if(IsKeyPressed(KEY_ESCAPE)) {ui.focus=-1;ui.selected=false;}
            SetMouseCursor(MOUSE_CURSOR_DEFAULT);
            BeginDrawing(); ClearBackground(Background);
            Camera2D camera{};camera.offset=offset;camera.zoom=scale; BeginMode2D(camera);
            ui.Text(26,24,34,"GLOVE / CONTROL");
            ui.Text(27,69,15,"Your hand turns. The pointer follows. Hold still to stop.",Muted);
            ui.Badge({24,96,190,28},online?"DRIVER ONLINE":"DRIVER OFFLINE",online?Accent:Muted);
            ui.Badge({226,96,220,28},online?(dry?"DRY RUN / PREVIEW":"SYSTEM MOUSE"):"WAITING FOR CONNECTION",online?Blue:Muted);
            ui.Badge({850,96,246,28},online?(enabled?"MOTION ENABLED":"MOTION PAUSED"):"MOTION STATE UNKNOWN",online?(enabled?Accent:Amber):Muted);
            ui.Panel({24,136,668,260});
            ui.Text(44,154,23,"Axis mapping");
            ui.Text(44,185,13,"Planes are measured in the reference orientation.",Muted);
            ui.Text(44,213,12,"POINTER",Muted);ui.Text(148,213,12,"ROTATION PLANE",Muted);
            ui.Text(432,213,12,"INVERT",Muted);ui.Text(536,213,12,"UNITS / DEG",Muted);
            AxisRow(ui,235,"X / right",draft.config.x,draft.gain_x,2,editable,draft.dirty);
            AxisRow(ui,299,"Y / down",draft.config.y,draft.gain_y,3,editable,draft.dirty);
            ui.Text(44,364,13,"XY = around Z     XZ = around Y     YZ = around X",Muted);
            ui.Panel({24,414,668,100}); ui.Text(44,430,15,"Serial port",Muted);
            draft.dirty=ui.Edit(1,{44,457,628,38},draft.port,editable)||draft.dirty;
            ui.Panel({24,530,668,104}); ui.Text(44,547,18,"Noise threshold");
            draft.dirty=ui.Edit(4,{44,579,130,36},draft.deadzone,editable,true)||draft.dirty;
            ui.Text(188,588,15,"deg/s",Muted);ui.Text(294,551,14,"Ignore small rotation noise.",Muted);
            ui.Text(294,578,13,"Sensitivity: 0.1 - 200 units/deg",Muted);ui.Text(294,599,13,"Noise threshold: 0 - 20 deg/s",Muted);
            ui.Panel({24,654,668,108});
            if(ui.Button({44,681,196,48},enabled?"Pause motion":"Enable motion",enabled?Red:Accent,
                online && !control_pending && (enabled || ready),true)) {
                auto fields=Object();Put(fields.get(),"enabled",Bool(!enabled));
                if(!client.Send("set_enabled",std::move(fields))) notice="Cannot send motion command.";
            }
            if(ui.Button({252,681,185,48},"Set reference",Blue,ready && !control_pending)) {
                if(!client.Send("recenter")) notice="Cannot send reference command.";
            }
            if(ui.Button({449,681,223,48},applying?"Applying...":"Apply settings",Blue,online && !applying,true)) {
                try {
                    auto fields=Object();Put(fields.get(),"config",ConfigJson(draft.Read()));
                    if(!client.Send("set_config",std::move(fields))) notice="Cannot send settings.";
                } catch(const std::exception& e) {notice=e.what();}
            }
            ui.Text(44,740,12,draft.dirty?"Changes pending - Apply to save and use them.":
                have_config?std::string("Active: X = ")+PlaneName(active.x.plane)+" / Y = "+PlaneName(active.y.plane):"Connect a driver to load active settings.",draft.dirty?Amber:Muted);

            ui.Panel({712,136,384,360}); ui.Text(732,154,23,"Motion preview");
            ui.Text(732,185,13,"Relative input only / not hand position",Muted);
            const Rectangle plot{732,220,344,246}; const Vector2 origin{904,343};
            DrawRectangleRounded(plot,.08f,8,Color{15,24,36,255});
            for(int i=-3;i<=3;++i) DrawLineV({origin.x+i*44,plot.y+12},{origin.x+i*44,plot.y+plot.height-12},Fade(Border,.65f));
            for(int i=-2;i<=2;++i) DrawLineV({plot.x+12,origin.y+i*44},{plot.x+plot.width-12,origin.y+i*44},Fade(Border,.65f));
            DrawLineV({origin.x,plot.y+12},{origin.x,plot.y+plot.height-12},Muted);
            DrawLineV({plot.x+12,origin.y},{plot.x+plot.width-12,origin.y},Muted);
            for(std::size_t i=1;i<trail.size();++i)
                DrawLineEx({origin.x+trail[i-1].x,origin.y+trail[i-1].y},{origin.x+trail[i].x,origin.y+trail[i].y},2,Fade(Accent,.15f+.65f*static_cast<float>(i)/trail.size()));
            const Vector2 marker{origin.x+preview.x,origin.y+preview.y};
            DrawCircleV(marker,13,Fade(Accent,.15f));DrawCircleV(marker,5,Accent);
            ui.Text(plot.x+10,plot.y+plot.height-20,11,"Preview scale 1:3 / OS acceleration may differ",Muted);
            ui.Panel({712,512,384,250}); ui.Text(732,529,21,"Live telemetry");
            const auto* raw=status ? json_object_object_get(status.get(),"sample") : nullptr;
            auto* sample=const_cast<json_object*>(raw);
            auto* angles=sample?json_object_object_get(sample,"angles_deg"):nullptr;
            auto* q=sample?json_object_object_get(sample,"quaternion"):nullptr;
            auto* cal=sample?json_object_object_get(sample,"calibration"):nullptr;
            ui.Text(732,563,13,online?"Stream: "+Label(status.get(),"telemetry_state"):"Stream: unknown",ready?Accent:Amber);
            std::string angle_text="--- / --- / ---";
            if(angles && json_object_array_length(angles)==3) angle_text=Format(json_object_get_double(json_object_array_get_idx(angles,0)))+" / "+Format(json_object_get_double(json_object_array_get_idx(angles,1)))+" / "+Format(json_object_get_double(json_object_array_get_idx(angles,2)));
            ui.Text(732,590,12,"YAW / PITCH / ROLL (deg)",Muted);ui.Text(732,611,18,angle_text);
            std::string quaternion="W/X/Y/Z: ---";
            if(q) {quaternion="Q:";for(unsigned i=0;i<4;++i) quaternion+=" "+Format(json_object_get_double(json_object_array_get_idx(q,i)),3);}
            ui.Text(732,640,13,quaternion,Muted);
            std::string calibration="SYS / GYR / ACC / MAG: ---";
            if(cal) {calibration="CAL:";for(unsigned i=0;i<4;++i) calibration+=" "+std::to_string(json_object_get_int(json_object_array_get_idx(cal,i)));}
            ui.Text(732,664,14,calibration,Muted);
            const double age=sample?Value(status.get(),"sample_age_ms")+(last_update<0?0:(now-last_update)*1000):0;
            ui.Text(732,690,13,sample?"Sample age: "+Format(age,0)+" ms":"Sample age: no data",sample && age<250?Muted:Amber);
            ui.Text(732,714,13,online?"Last step: "+Format(Value(status.get(),"last_dx"),0)+" / "+Format(Value(status.get(),"last_dy"),0):"Last step: unknown",Muted);
            ui.Text(732,738,11,"Calibration levels: 0 (none) to 3 (full)",Muted);
            const auto runtime_error=online?Label(status.get(),"last_error"):client.Error();
            const auto message=!notice.empty()?notice:"Changes are applied only after driver confirmation.";
            ui.Text(26,780,14,ui.Fit(message,1068,14),!notice.empty()?Amber:Muted);
            if(!runtime_error.empty()) ui.Text(26,802,12,ui.Fit(runtime_error,1068,12),Amber);
            ui.Text(26,826,10,ui.Fit("Socket: "+socket_path+"   |   Ctrl+Q: close window (driver keeps running)",1068,10),Muted);
            EndMode2D(); EndDrawing();
        }
    }
    CloseWindow(); return 0;
}
} // namespace
int main(int argc,char** argv)
{
    try {
        std::string path=glove::DefaultSocketPath();
        for(int i=1;i<argc;++i) {
            const std::string arg=argv[i];
            if(arg=="--help" || arg=="-h") {std::cout<<"Usage: glove-mouse-config [--socket PATH]\nStart glove-mouse-driver separately.\n";return 0;}
            if(arg!="--socket" || ++i>=argc) throw std::runtime_error("Expected --socket PATH or --help");
            path=argv[i];
        }
        return Run(path);
    } catch(const std::exception& e) {std::cerr<<"glove-mouse-config: "<<e.what()<<'\n';return 1;}
}
