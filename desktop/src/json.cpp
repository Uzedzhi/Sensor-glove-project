#include "glove_mouse/json.hpp"
#include <cctype>
#include <cmath>

namespace glove {
Json ParseJson(std::string_view text)
{
    if (text.empty() || text.size() > 65536) throw std::runtime_error("JSON message must be 1..65536 bytes");
    std::unique_ptr<json_tokener, decltype(&json_tokener_free)> tok(json_tokener_new(), json_tokener_free);
    if (!tok) throw std::runtime_error("Cannot allocate JSON parser");
    json_tokener_set_flags(tok.get(), JSON_TOKENER_STRICT | JSON_TOKENER_VALIDATE_UTF8);
    Json value(json_tokener_parse_ex(tok.get(), text.data(), static_cast<int>(text.size())));
    if (json_tokener_get_error(tok.get()) != json_tokener_success || !value)
        throw std::runtime_error("Invalid JSON message");
    for (std::size_t i = json_tokener_get_parse_end(tok.get()); i < text.size(); ++i)
        if (!std::isspace(static_cast<unsigned char>(text[i]))) throw std::runtime_error("Trailing JSON data");
    return value;
}
std::string DumpJson(json_object* value) { return json_object_to_json_string_ext(value, JSON_C_TO_STRING_PLAIN); }
json_object* Required(json_object* object, const char* key, json_type type)
{
    json_object* value = nullptr;
    if (!object || !json_object_is_type(object, json_type_object) ||
        !json_object_object_get_ex(object, key, &value) || !json_object_is_type(value, type))
        throw std::runtime_error(std::string("Missing or invalid field: ") + key);
    return value;
}
double Number(json_object* object, const char* key)
{
    json_object* value = nullptr;
    if (!object || !json_object_object_get_ex(object, key, &value) ||
        !(json_object_is_type(value, json_type_double) || json_object_is_type(value, json_type_int)))
        throw std::runtime_error(std::string("Expected number: ") + key);
    const double n = json_object_get_double(value);
    if (!std::isfinite(n)) throw std::runtime_error(std::string("Expected finite number: ") + key);
    return n;
}
std::string String(json_object* object, const char* key)
{
    auto* v = Required(object, key, json_type_string);
    std::string result(json_object_get_string(v), static_cast<std::size_t>(json_object_get_string_len(v)));
    if (result.find('\0') != std::string::npos) throw std::runtime_error(std::string("NUL in field: ") + key);
    return result;
}
bool Boolean(json_object* object, const char* key) { return json_object_get_boolean(Required(object, key, json_type_boolean)); }
void Put(json_object* object, const char* key, Json value) { json_object_object_add(object, key, value.release()); }
Json Object() { return Json(json_object_new_object()); }
Json Text(const std::string& text) { return Json(json_object_new_string_len(text.data(), static_cast<int>(text.size()))); }
Json Real(double n) { return Json(json_object_new_double(n)); }
Json Integer(std::int64_t n) { return Json(json_object_new_int64(n)); }
Json Bool(bool b) { return Json(json_object_new_boolean(b)); }
} // namespace glove
