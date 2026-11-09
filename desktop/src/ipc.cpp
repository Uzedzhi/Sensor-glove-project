#include "glove_mouse/ipc.hpp"
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

namespace glove {
namespace {
sockaddr_un Address(const std::string& path)
{
    sockaddr_un a{}; a.sun_family=AF_UNIX;
    if (path.empty() || path.size()>=sizeof(a.sun_path)) throw std::runtime_error("Unix socket path is empty or too long");
    std::memcpy(a.sun_path,path.c_str(),path.size()+1); return a;
}
Json ProtocolError(const std::string& message)
{
    auto j=Object(); Put(j.get(),"version",Integer(1)); Put(j.get(),"id",Json{}); Put(j.get(),"ok",Bool(false));
    auto error=Object(); Put(error.get(),"code",Text("invalid_json")); Put(error.get(),"message",Text(message));
    Put(j.get(),"error",std::move(error)); return j;
}
}
LocalServer::LocalServer(const std::string& path) : path_(path)
{
    const auto address=Address(path);
    const std::filesystem::path parent=std::filesystem::path(path).parent_path();
    if (!parent.empty() && std::filesystem::create_directories(parent)) chmod(parent.c_str(),0700);
    struct stat info{};
    if (lstat(path.c_str(),&info)==0) {
        if (!S_ISSOCK(info.st_mode) || info.st_uid!=getuid()) throw std::runtime_error("Socket path already exists and is not an owned socket");
        Fd probe(socket(AF_UNIX,SOCK_STREAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0));
        if (!probe) throw std::runtime_error(SystemError("Create socket probe"));
        if (connect(probe.Get(),reinterpret_cast<const sockaddr*>(&address),sizeof(address))==0 ||
            (errno!=ECONNREFUSED && errno!=ENOENT)) throw std::runtime_error("A driver is already running at " + path);
        if (unlink(path.c_str())!=0 && errno!=ENOENT) throw std::runtime_error(SystemError("Remove stale socket"));
    }
    listener_.Reset(socket(AF_UNIX,SOCK_STREAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0));
    if (!listener_) throw std::runtime_error(SystemError("Create Unix socket"));
    if (bind(listener_.Get(),reinterpret_cast<const sockaddr*>(&address),sizeof(address))!=0)
        throw std::runtime_error(SystemError("Bind driver socket (another driver may be running)"));
    owns_=true;
    if (chmod(path.c_str(),0600)!=0 || listen(listener_.Get(),8)!=0) {
        const auto error=SystemError("Listen on driver socket"); unlink(path.c_str()); owns_=false; throw std::runtime_error(error);
    }
}
LocalServer::~LocalServer() { if (owns_) unlink(path_.c_str()); }
void LocalServer::Process(const Handler& handler)
{
    for (int i=0;i<8;++i) {
        const int fd=accept4(listener_.Get(),nullptr,nullptr,SOCK_NONBLOCK|SOCK_CLOEXEC);
        if (fd<0) break;
        if (clients_.size()>=16) { close(fd); continue; }
        clients_.push_back({Fd(fd),{}, {}});
    }
    for (std::size_t i=0;i<clients_.size();) {
        auto& c=clients_[i]; bool live=true;
        char bytes[4096];
        for (int batch=0;batch<4 && live;++batch) {
            const ssize_t n=recv(c.fd.Get(),bytes,sizeof(bytes),0);
            if (n<0 && errno==EINTR) continue;
            if (n<0 && (errno==EAGAIN || errno==EWOULDBLOCK)) break;
            if (n<=0) { live=false; break; }
            const auto old_overflow=c.input.Overflows();
            auto lines=c.input.Feed(std::string_view(bytes,static_cast<std::size_t>(n)));
            if (old_overflow!=c.input.Overflows() || lines.size()>64) { live=false; break; }
            for (const auto& line:lines) {
                Json reply;
                try { auto request=ParseJson(line); reply=handler(request.get()); }
                catch(const std::exception& e) { reply=ProtocolError(e.what()); }
                c.output+=DumpJson(reply.get())+'\n';
                if(c.output.size()>262144) { live=false; break; }
            }
        }
        if (live && !c.output.empty()) {
            const ssize_t n=send(c.fd.Get(),c.output.data(),c.output.size(),MSG_NOSIGNAL);
            if (n>0) c.output.erase(0,static_cast<std::size_t>(n));
            else if (n==0 || (errno!=EAGAIN && errno!=EWOULDBLOCK && errno!=EINTR)) live=false;
        }
        if (live) ++i; else clients_.erase(clients_.begin()+static_cast<std::ptrdiff_t>(i));
    }
}
void IpcClient::Disconnect(const std::string& error)
{
    fd_.Reset(); connected_=connecting_=false; error_=error; output_.clear(); input_.Clear(); pending_.clear();
    next_retry_=Monotonic()+1;
}
bool IpcClient::Pending(const std::string& command) const
{
    for (const auto& p:pending_) if(p.second.command==command) return true;
    return false;
}
bool IpcClient::Send(const std::string& command,Json fields)
{
    if (!connected_ || pending_.size()>=8) return false;
    const auto id=next_id_++;
    Put(fields.get(),"version",Integer(1)); Put(fields.get(),"id",Integer(id)); Put(fields.get(),"command",Text(command));
    const auto line=DumpJson(fields.get())+'\n';
    if(line.size()>65536 || output_.size()+line.size()>65536) { error_="Outgoing command too large"; return false; }
    output_+=line; pending_.emplace(id,PendingRequest{command,Monotonic()+.5}); return true;
}
std::vector<ClientReply> IpcClient::Tick()
{
    std::vector<ClientReply> replies;
    const double now=Monotonic();
    if (!fd_ && now>=next_retry_) {
        try {
            const auto address=Address(path_);
            fd_.Reset(socket(AF_UNIX,SOCK_STREAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0));
            if(!fd_) throw std::runtime_error(SystemError("Create GUI socket"));
            if(connect(fd_.Get(),reinterpret_cast<const sockaddr*>(&address),sizeof(address))==0) {
                connected_=true; error_.clear();
            } else if(errno==EINPROGRESS || errno==EAGAIN) { connecting_=true; connect_deadline_=now+.5; }
            else throw std::runtime_error(SystemError("Connect driver"));
        } catch(const std::exception& e) { Disconnect(e.what()); }
    }
    if(connecting_) {
        pollfd p{fd_.Get(),POLLOUT,0};
        if(poll(&p,1,0)>0) {
            int error=0; socklen_t size=sizeof(error);
            if(getsockopt(fd_.Get(),SOL_SOCKET,SO_ERROR,&error,&size)!=0 || error) Disconnect("Driver connection failed");
            else { connecting_=false; connected_=true; error_.clear(); }
        } else if(now>=connect_deadline_) Disconnect("Driver connection timed out");
    }
    if(!connected_) return replies;
    if(!output_.empty()) {
        const ssize_t n=send(fd_.Get(),output_.data(),output_.size(),MSG_NOSIGNAL);
        if(n>0) output_.erase(0,static_cast<std::size_t>(n));
        else if(n==0 || (errno!=EAGAIN && errno!=EWOULDBLOCK && errno!=EINTR)) { Disconnect("Cannot send driver command"); return replies; }
    }
    char bytes[4096];
    for(int batch=0;batch<16;++batch) {
        const ssize_t n=recv(fd_.Get(),bytes,sizeof(bytes),0);
        if(n<0 && errno==EINTR) continue;
        if(n<0 && (errno==EAGAIN || errno==EWOULDBLOCK)) break;
        if(n<=0) { Disconnect("Driver disconnected"); return replies; }
        const auto old_overflow=input_.Overflows();
        const auto lines=input_.Feed(std::string_view(bytes,static_cast<std::size_t>(n)));
        if(old_overflow!=input_.Overflows()) { Disconnect("Driver reply too large"); return replies; }
        try {
            for(const auto& line:lines) {
                auto body=ParseJson(line);
                if(json_object_get_int64(Required(body.get(),"version",json_type_int))!=1) throw std::runtime_error("Unsupported driver protocol");
                const auto id=json_object_get_int64(Required(body.get(),"id",json_type_int));
                const auto it=pending_.find(id);
                if(it==pending_.end()) throw std::runtime_error("Unexpected driver reply id");
                (void)Boolean(body.get(),"ok");
                replies.push_back({it->second.command,std::move(body)}); pending_.erase(it);
            }
        } catch(const std::exception& e) { Disconnect(e.what()); return replies; }
    }
    for(const auto& p:pending_) if(now>=p.second.deadline) { Disconnect("Driver response timed out"); break; }
    return replies;
}
} // namespace glove
