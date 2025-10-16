#include <array>
#include <cstddef>
#include <cstdint>

template <typename T, std::size_t N>
class SizedParameterBlock {
 public:
  SizedParameterBlock(uint64_t id);
  virtual ~SizedParameterBlock() = 0;

  T * ParameterPtr(std::size_t pos = 0);
  T const * ParameterPtr(std::size_t pos = 0) const;

 private:
  const uint64_t id_;
  std::array<T, N> parameters_;
};
