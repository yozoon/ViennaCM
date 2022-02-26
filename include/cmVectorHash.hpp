#ifndef CM_VECTOR_HASH_HPP
#define CM_VECTOR_HASH_HPP

#include <hrleVectorType.hpp>
#include <utility>

template <class VectorType> class cmVectorHash {
  using T = typename VectorType::value_type;
  static constexpr int D = std::tuple_size<VectorType>::value;

private:
  //  https://stackoverflow.com/questions/5889238/why-is-xor-the-default-way-to-combine-hashes
  std::size_t hash_combine(std::size_t lhs, std::size_t rhs) const {
    lhs ^= rhs + 0x9e3779b9 + (lhs << 6) + (lhs >> 2);
    return lhs;
  }

public:
  std::size_t operator()(const VectorType &v) const {
    using std::hash;
    using std::size_t;
    std::size_t result = hash<T>()(v[0]);
    result = hash_combine(result, hash<T>()(v[1]));
    if constexpr (D == 3) {
      result = hash_combine(result, hash<T>()(v[2]));
    }
    return result;
  }
};

#endif