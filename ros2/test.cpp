
#include <iostream>
#include <map>
#include <variant>
#include <cstdint>
#include <type_traits>

using namespace std;

template<class T>
struct A {
  A(uint16_t && idx0, uint16_t && idx1, T && t, const char * ch) : idx0(idx0), idx1(idx1), t(t), ch(ch) {}
  uint16_t idx0; 
  uint16_t idx1;
  const T t;
  const char * ch;
};

struct B {
  B(uint16_t && idx0, uint16_t && idx1) : idx0(idx0), idx1(idx1) {}
  const uint16_t idx0;
  const uint16_t idx1;
  bool operator < (const B & rhs) const
  { 
    if (idx0 < rhs.idx0 || (idx0 == rhs.idx0 && idx1 < rhs.idx1))
      return true;
    else
      return false;
  }
};

template<class...Ts> using var_t = std::variant<A<Ts>...>;
using variant_types=var_t<uint8_t, int8_t, const char *>;
template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;

static const map<const B, const variant_types> m = {
  {B(1, 0), A<uint8_t>(1, 42, 1, "Test")},
  {B(4,0), A<int8_t>(4, 34, 7, "Test2")},
  {B(400,0), A<const char *>(400, 0, "CONTENTS", "Test3")}
};  

int main() 
{
  for (const auto & [k, v] : m) {
    std::visit(overloaded {
      [&v = std::as_const(v)](const A<uint8_t> & arg) { cout << to_string(arg.t) << " as uint8_t" << endl; },
      [&v = std::as_const(v)](const A<int8_t> & arg) { cout << to_string(arg.t) << " as int8_t" << endl; },
      [&v = std::as_const(v)](const A<const char *> & arg) { cout << arg.t << " as const char *" << endl; },
    }, v);
  }
  return 0;
}

