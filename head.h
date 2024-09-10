
template <typename T = int>
struct Point {
  T x;
  T y;
};

using Point32I = Point<int>;
using Point32F = Point<float>;
using Point64D = Point<double>;

template <typename T = int>
struct RotatedBox{
  T x_ctr; 
  T y_ctr; 
  T w;   
  T h;   
  float a;   
} ;

using RotatedBox32I = RotatedBox<int>;
using RotatedBox32F = RotatedBox<float>;
using RotatedBox64D = RotatedBox<double>;

std::vector<int> nms_rotated(std::vector<std::vector<float>>& boxes, std::vector<float>& scores, float threshold);