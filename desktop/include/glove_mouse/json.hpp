#pragma once
#include <json-c/json.h>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>

namespace glove {
struct JsonDelete { void operator()(json_object* p) const { if (p) json_object_put(p); } };
using Json = std::unique_ptr<json_object, JsonDelete>;
Json ParseJson(std::string_view text);
std::string DumpJson(json_object* value);
json_object* Required(json_object* object, const char* key, json_type type);
double Number(json_object* object, const char* key);
std::string String(json_object* object, const char* key);
bool Boolean(json_object* object, const char* key);
void Put(json_object* object, const char* key, Json value);
Json Object();
Json Text(const std::string& text);
Json Real(double number);
Json Integer(std::int64_t number);
Json Bool(bool value);
} // namespace glove
