#pragma once
#include "io.hpp"
#include <functional>
#include <map>

namespace glove {
class LocalServer {
public:
    using Handler = std::function<Json(json_object*)>;
    explicit LocalServer(const std::string& path);
    ~LocalServer();
    void Process(const Handler& handler);
    LocalServer(const LocalServer&) = delete;
    LocalServer& operator=(const LocalServer&) = delete;
private:
    struct Client { Fd fd; LineFramer input; std::string output; };
    std::string path_;
    Fd listener_;
    bool owns_ = false;
    std::vector<Client> clients_;
};
struct ClientReply { std::string command; Json body; };
class IpcClient {
public:
    explicit IpcClient(std::string path) : path_(std::move(path)) {}
    bool Connected() const { return connected_; }
    const std::string& Error() const { return error_; }
    bool Pending(const std::string& command) const;
    bool Send(const std::string& command, Json fields = Object());
    std::vector<ClientReply> Tick();
private:
    void Disconnect(const std::string& error);
    struct PendingRequest { std::string command; double deadline; };
    std::string path_, error_, output_;
    Fd fd_;
    bool connected_ = false, connecting_ = false;
    double next_retry_ = 0, connect_deadline_ = 0;
    std::int64_t next_id_ = 1;
    LineFramer input_;
    std::map<std::int64_t,PendingRequest> pending_;
};
} // namespace glove
