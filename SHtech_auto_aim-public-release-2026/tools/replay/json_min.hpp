//
// Minimal JSON DOM for tools/replay（无第三方依赖）
//
// 目标：
//   - 解析 R1 的 records.jsonl（完整支持字符串转义、数字、对象、数组）
//   - 序列化 R2 输出：数字用 std::to_chars（最短往返表示），与 locale 无关，
//     同一份输入在同一构建上跑两次输出逐字节一致（B-2 验收①）
//
// 注意：仅用于回放工具的小数据量场景，不做性能优化。
//

#ifndef TOOLS_REPLAY_JSON_MIN_HPP
#define TOOLS_REPLAY_JSON_MIN_HPP

#include <charconv>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

namespace replay_json
{

    class Value;
    using Array = std::vector<Value>;
    // 保留插入顺序（写 R2 时按键序固定输出，不依赖 map 的排序）
    using Object = std::vector<std::pair<std::string, Value>>;

    class Value
    {
    public:
        using Storage = std::variant<std::nullptr_t, bool, int64_t, uint64_t, double, std::string, Array, Object>;

        Value() : data_(nullptr) {}
        Value(std::nullptr_t) : data_(nullptr) {}
        Value(bool v) : data_(v) {}
        Value(int v) : data_(static_cast<int64_t>(v)) {}
        Value(int64_t v) : data_(v) {}
        Value(uint64_t v) : data_(v) {}
        Value(double v) : data_(v) {}
        Value(const char* s) : data_(std::string(s)) {}
        Value(std::string s) : data_(std::move(s)) {}
        Value(Array a) : data_(std::move(a)) {}
        Value(Object o) : data_(std::move(o)) {}

        bool is_null() const { return std::holds_alternative<std::nullptr_t>(data_); }
        bool is_bool() const { return std::holds_alternative<bool>(data_); }
        bool is_int() const { return std::holds_alternative<int64_t>(data_) || std::holds_alternative<uint64_t>(data_); }
        bool is_number() const { return is_int() || std::holds_alternative<double>(data_); }
        bool is_string() const { return std::holds_alternative<std::string>(data_); }
        bool is_array() const { return std::holds_alternative<Array>(data_); }
        bool is_object() const { return std::holds_alternative<Object>(data_); }

        bool as_bool() const
        {
            if (std::holds_alternative<bool>(data_)) return std::get<bool>(data_);
            throw std::runtime_error("json: expected bool");
        }
        int64_t as_int64() const
        {
            if (std::holds_alternative<int64_t>(data_)) return std::get<int64_t>(data_);
            if (std::holds_alternative<uint64_t>(data_)) return static_cast<int64_t>(std::get<uint64_t>(data_));
            throw std::runtime_error("json: expected integer");
        }
        uint64_t as_uint64() const
        {
            if (std::holds_alternative<uint64_t>(data_)) return std::get<uint64_t>(data_);
            if (std::holds_alternative<int64_t>(data_)) return static_cast<uint64_t>(std::get<int64_t>(data_));
            throw std::runtime_error("json: expected unsigned integer");
        }
        int as_int() const { return static_cast<int>(as_int64()); }
        double as_double() const
        {
            if (std::holds_alternative<double>(data_)) return std::get<double>(data_);
            if (std::holds_alternative<int64_t>(data_)) return static_cast<double>(std::get<int64_t>(data_));
            if (std::holds_alternative<uint64_t>(data_)) return static_cast<double>(std::get<uint64_t>(data_));
            throw std::runtime_error("json: expected number");
        }
        const std::string& as_string() const
        {
            if (std::holds_alternative<std::string>(data_)) return std::get<std::string>(data_);
            throw std::runtime_error("json: expected string");
        }
        const Array& as_array() const
        {
            if (std::holds_alternative<Array>(data_)) return std::get<Array>(data_);
            throw std::runtime_error("json: expected array");
        }
        const Object& as_object() const
        {
            if (std::holds_alternative<Object>(data_)) return std::get<Object>(data_);
            throw std::runtime_error("json: expected object");
        }

        // 查找对象成员；不存在返回 nullptr
        const Value* find(std::string_view key) const
        {
            if (!is_object()) return nullptr;
            const auto& obj = as_object();
            for (const auto& kv : obj)
            {
                if (kv.first == key) return &kv.second;
            }
            return nullptr;
        }
        // 必须存在的成员
        const Value& at(std::string_view key) const
        {
            const Value* v = find(key);
            if (v == nullptr)
            {
                throw std::runtime_error(std::string("json: missing key ") + std::string(key));
            }
            return *v;
        }

        // ---------- 序列化（确定性） ----------
        std::string dump() const
        {
            std::string out;
            dump_to(out);
            return out;
        }

        void dump_to(std::string& out) const
        {
            if (std::holds_alternative<std::nullptr_t>(data_))
            {
                out += "null";
            }
            else if (std::holds_alternative<bool>(data_))
            {
                out += std::get<bool>(data_) ? "true" : "false";
            }
            else if (std::holds_alternative<int64_t>(data_))
            {
                char buf[32];
                auto r = std::to_chars(buf, buf + sizeof(buf), std::get<int64_t>(data_));
                out.append(buf, r.ptr);
            }
            else if (std::holds_alternative<uint64_t>(data_))
            {
                char buf[32];
                auto r = std::to_chars(buf, buf + sizeof(buf), std::get<uint64_t>(data_));
                out.append(buf, r.ptr);
            }
            else if (std::holds_alternative<double>(data_))
            {
                double v = std::get<double>(data_);
                if (!std::isfinite(v))
                {
                    out += "null";  // NaN/Inf 序列化为 null（R2 约定：失败输出为 null）
                }
                else
                {
                    char buf[64];
                    auto r = std::to_chars(buf, buf + sizeof(buf), v);  // 最短往返，确定性
                    out.append(buf, r.ptr);
                }
            }
            else if (std::holds_alternative<std::string>(data_))
            {
                dump_string(std::get<std::string>(data_), out);
            }
            else if (std::holds_alternative<Array>(data_))
            {
                out += '[';
                const auto& arr = std::get<Array>(data_);
                for (size_t i = 0; i < arr.size(); ++i)
                {
                    if (i > 0) out += ',';
                    arr[i].dump_to(out);
                }
                out += ']';
            }
            else
            {
                out += '{';
                const auto& obj = std::get<Object>(data_);
                for (size_t i = 0; i < obj.size(); ++i)
                {
                    if (i > 0) out += ',';
                    dump_string(obj[i].first, out);
                    out += ':';
                    obj[i].second.dump_to(out);
                }
                out += '}';
            }
        }

        static void dump_string(const std::string& s, std::string& out)
        {
            out += '"';
            for (unsigned char c : s)
            {
                switch (c)
                {
                    case '"':  out += "\\\""; break;
                    case '\\': out += "\\\\"; break;
                    case '\b': out += "\\b"; break;
                    case '\f': out += "\\f"; break;
                    case '\n': out += "\\n"; break;
                    case '\r': out += "\\r"; break;
                    case '\t': out += "\\t"; break;
                    default:
                        if (c < 0x20)
                        {
                            constexpr char hex[] = "0123456789abcdef";
                            out += "\\u00";
                            out += hex[(c >> 4) & 0xF];
                            out += hex[c & 0xF];
                        }
                        else
                        {
                            out += static_cast<char>(c);  // UTF-8 字节原样保留
                        }
                }
            }
            out += '"';
        }

        // ---------- 解析 ----------
        static Value parse(std::string_view text)
        {
            Parser p(text);
            Value v = p.parse_value();
            p.skip_ws();
            if (!p.at_end()) p.fail("unexpected trailing characters");
            return v;
        }

    private:
        struct Parser
        {
            std::string_view s;
            size_t pos = 0;

            explicit Parser(std::string_view text) : s(text) {}

            [[noreturn]] void fail(const std::string& msg) const
            {
                throw std::runtime_error("json parse error at offset " + std::to_string(pos) + ": " + msg);
            }
            bool at_end() const { return pos >= s.size(); }
            char peek() const
            {
                if (at_end()) fail("unexpected end of input");
                return s[pos];
            }
            char next()
            {
                char c = peek();
                ++pos;
                return c;
            }
            void skip_ws()
            {
                while (!at_end() && (s[pos] == ' ' || s[pos] == '\t' || s[pos] == '\n' || s[pos] == '\r')) ++pos;
            }

            Value parse_value()
            {
                skip_ws();
                char c = peek();
                switch (c)
                {
                    case '{': return parse_object();
                    case '[': return parse_array();
                    case '"': return Value(parse_string());
                    case 't': expect("true"); return Value(true);
                    case 'f': expect("false"); return Value(false);
                    case 'n': expect("null"); return Value(nullptr);
                    default: return parse_number();
                }
            }

            void expect(const char* lit)
            {
                for (const char* q = lit; *q; ++q)
                {
                    if (at_end() || s[pos] != *q) fail(std::string("expected ") + lit);
                    ++pos;
                }
            }

            Value parse_object()
            {
                next();  // '{'
                Object obj;
                skip_ws();
                if (peek() == '}')
                {
                    next();
                    return Value(std::move(obj));
                }
                while (true)
                {
                    skip_ws();
                    if (peek() != '"') fail("expected string key");
                    std::string key = parse_string();
                    skip_ws();
                    if (next() != ':') fail("expected ':'");
                    obj.emplace_back(std::move(key), parse_value());
                    skip_ws();
                    char c = next();
                    if (c == '}') break;
                    if (c != ',') fail("expected ',' or '}'");
                }
                return Value(std::move(obj));
            }

            Value parse_array()
            {
                next();  // '['
                Array arr;
                skip_ws();
                if (peek() == ']')
                {
                    next();
                    return Value(std::move(arr));
                }
                while (true)
                {
                    arr.push_back(parse_value());
                    skip_ws();
                    char c = next();
                    if (c == ']') break;
                    if (c != ',') fail("expected ',' or ']'");
                }
                return Value(std::move(arr));
            }

            std::string parse_string()
            {
                if (next() != '"') fail("expected '\"'");
                std::string out;
                while (true)
                {
                    if (at_end()) fail("unterminated string");
                    char c = next();
                    if (c == '"') break;
                    if (c != '\\')
                    {
                        out += c;
                        continue;
                    }
                    if (at_end()) fail("unterminated escape");
                    char e = next();
                    switch (e)
                    {
                        case '"':  out += '"'; break;
                        case '\\': out += '\\'; break;
                        case '/':  out += '/'; break;
                        case 'b':  out += '\b'; break;
                        case 'f':  out += '\f'; break;
                        case 'n':  out += '\n'; break;
                        case 'r':  out += '\r'; break;
                        case 't':  out += '\t'; break;
                        case 'u':
                        {
                            if (pos + 4 > s.size()) fail("invalid \\u escape");
                            uint32_t cp = 0;
                            for (int i = 0; i < 4; ++i)
                            {
                                char h = next();
                                cp <<= 4;
                                if (h >= '0' && h <= '9') cp |= static_cast<uint32_t>(h - '0');
                                else if (h >= 'a' && h <= 'f') cp |= static_cast<uint32_t>(h - 'a' + 10);
                                else if (h >= 'A' && h <= 'F') cp |= static_cast<uint32_t>(h - 'A' + 10);
                                else fail("invalid hex digit in \\u escape");
                            }
                            if (cp < 0x80)
                            {
                                out += static_cast<char>(cp);
                            }
                            else if (cp < 0x800)
                            {
                                out += static_cast<char>(0xC0 | (cp >> 6));
                                out += static_cast<char>(0x80 | (cp & 0x3F));
                            }
                            else
                            {
                                out += static_cast<char>(0xE0 | (cp >> 12));
                                out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
                                out += static_cast<char>(0x80 | (cp & 0x3F));
                            }
                            break;
                        }
                        default: fail("invalid escape character");
                    }
                }
                return out;
            }

            Value parse_number()
            {
                size_t start = pos;
                bool is_float = false;
                bool negative = false;
                if (!at_end() && s[pos] == '-')
                {
                    negative = true;
                    ++pos;
                }
                while (!at_end() && s[pos] >= '0' && s[pos] <= '9') ++pos;
                if (!at_end() && s[pos] == '.')
                {
                    is_float = true;
                    ++pos;
                    while (!at_end() && s[pos] >= '0' && s[pos] <= '9') ++pos;
                }
                if (!at_end() && (s[pos] == 'e' || s[pos] == 'E'))
                {
                    is_float = true;
                    ++pos;
                    if (!at_end() && (s[pos] == '+' || s[pos] == '-')) ++pos;
                    while (!at_end() && s[pos] >= '0' && s[pos] <= '9') ++pos;
                }
                if (pos == start || (negative && pos == start + 1)) fail("invalid number");

                std::string num(s.substr(start, pos - start));
                if (is_float)
                {
                    double d = 0.0;
                    auto r = std::from_chars(num.data(), num.data() + num.size(), d);
                    if (r.ec != std::errc()) fail("invalid float number");
                    return Value(d);
                }
                if (negative)
                {
                    int64_t i = 0;
                    auto r = std::from_chars(num.data(), num.data() + num.size(), i);
                    if (r.ec != std::errc()) fail("invalid integer");
                    return Value(i);
                }
                uint64_t u = 0;
                auto r = std::from_chars(num.data(), num.data() + num.size(), u);
                if (r.ec != std::errc()) fail("invalid integer");
                return Value(u);
            }
        };

        Storage data_;
    };

}  // namespace replay_json

#endif  // TOOLS_REPLAY_JSON_MIN_HPP
