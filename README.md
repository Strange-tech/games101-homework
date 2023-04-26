# games101-homework

Environment: MacOS 13.3

Related software and libraries (use the latest version)

- homebrew (to install g++, eigen, opencv and cmake)
- lib: eigen, opencv
- compiler: cmake, g++/clang++
- code editor: sublime text

Notes (not mine): <a href="https://www.zhihu.com/column/c_1249465121615204352">计算机图形学系列笔记</a>

## Assignment1

CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.10)
project(Rasterizer)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)

include_directories("<YOUR_COMPUTER_PATH>/eigen/3.4.0_1/include/")

add_executable(Rasterizer main.cpp rasterizer.hpp rasterizer.cpp Triangle.hpp Triangle.cpp)
target_link_libraries(Rasterizer ${OpenCV_LIBRARIES})
```

### core code

model matrix:

```C++
cos(r), -sin(r),  0,  0,
sin(r), cos(r),   0,  0,
0,      0,        1,  0,
0,      0,        0,  1;
```

view matrix:

```C++
1, 0, 0, -eye_pos[0],
0, 1, 0, -eye_pos[1],
0, 0, 1, -eye_pos[2],
0, 0, 0, 1;
```

projection matrix:

```C++
2*n/(r-l),  0,          (l+r)/(l-r),  0,
0,          2*n/(t-b),  (b+t)/(b-t),  0,
0,          0,          (f+n)/(n-f),  2*f*n/(f-n),
0,          0,          1,            0;
```

## Assignment2

CMakeLists.txt is similar to the above one.

### cores code

Judge if a point is inside of a triangle or not:

```C++
static bool insideTriangle(double x, double y, const Eigen::Vector4f* _v) {
    // // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    double v_ab[2], v_bc[2], v_ca[2];
    double v_ap[2], v_bp[2], v_cp[2];

    v_ab[0] = _v[1].x()-_v[0].x(); v_ab[1] = _v[1].y()-_v[0].y();
    v_bc[0] = _v[2].x()-_v[1].x(); v_bc[1] = _v[2].y()-_v[1].y();
    v_ca[0] = _v[0].x()-_v[2].x(); v_ca[1] = _v[0].y()-_v[2].y();

    v_ap[0] = x-_v[0].x(); v_ap[1] = y-_v[0].y();
    v_bp[0] = x-_v[1].x(); v_bp[1] = y-_v[1].y();
    v_cp[0] = x-_v[2].x(); v_cp[1] = y-_v[2].y();


    bool dir1 = cross(v_ap, v_ab) >= 0;
    bool dir2 = cross(v_bp, v_bc) >= 0;
    bool dir3 = cross(v_cp, v_ca) >= 0;

    return (dir1==dir2 && dir2==dir3);
}
```

rasterizer (not implementing anti-aliasing):

```C++
// 不含反走样的基本代码
for(int x=min_x; x<=max_x; x++) {
    for(int y=min_y; y<=max_y; y++) {
        if(insideTriangle(x+0.5, y+0.5, v.data())) {
            auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
            float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
            float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
            z_interpolated *= w_reciprocal;

            auto ind = (height-1-y)*width + x;
            if (depth_buf[ind] < z_interpolated)
                continue;

            // depth_buf[ind] >= z_interpolated
            depth_buf[ind] = z_interpolated;
            set_pixel(Eigen::Vector3f(x, y, z_interpolated), t.getColor());
        }
    }
}
```

rasterizer (using Super Sampling AA to implement anti-aliasing):

```C++
for(int x=min_x; x<=max_x; x++) {
    for(int y=min_y; y<=max_y; y++) {

        double x1=x+0.25, y1=y+0.25,
                x2=x+0.25, y2=y+0.75,
                x3=x+0.75, y3=y+0.25,
                x4=x+0.75, y4=y+0.75;
        auto ind=get_index(x, y);

        if(insideTriangle(x1, y1, v.data())) {
            double z_interpolated = getInterpolatedZ(x1, y1, t);
            if (sampling_depth_buf[ind].s1_depth >= z_interpolated) {
                sampling_depth_buf[ind].s1_depth = z_interpolated;
                sampling_color_buf[ind].s1_color = t.getColor();
            }
        }
        if(insideTriangle(x2, y2, v.data())) {
            double z_interpolated = getInterpolatedZ(x2, y2, t);
            if (sampling_depth_buf[ind].s2_depth >= z_interpolated) {
                sampling_depth_buf[ind].s2_depth = z_interpolated;
                sampling_color_buf[ind].s2_color = t.getColor();
            }
        }
        if(insideTriangle(x3, y3, v.data())) {
            double z_interpolated = getInterpolatedZ(x3, y3, t);
            if (sampling_depth_buf[ind].s3_depth >= z_interpolated) {
                sampling_depth_buf[ind].s3_depth = z_interpolated;
                sampling_color_buf[ind].s3_color = t.getColor();
            }
        }
        if(insideTriangle(x4, y4, v.data())) {
            double z_interpolated = getInterpolatedZ(x4, y4, t);
            if (sampling_depth_buf[ind].s4_depth >= z_interpolated) {
                sampling_depth_buf[ind].s4_depth = z_interpolated;
                sampling_color_buf[ind].s4_color = t.getColor();
            }
        }
    }
}
```

Please notice the code is core but incomplete, you should add new variables into class "rasterizer" in order to compile successfully.

## Assignment3

CMakeLists.txt is similar to the above one.

### core code

There are so many codes in this assignment, so I just choose one famous shader -- "phong shader" to show.

phong_fragment_shader:

```C++
Eigen::Vector3f result_color = {0, 0, 0};
for (auto& light : lights) {
    // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular*
    // components are. Then, accumulate that result on the *result_color* object.

    // Ld: diffuse light
    Eigen::Vector3f l = (light.position - point).normalized();
    double r = (light.position - point).norm();
    Eigen::Vector3f Ld = kd.cwiseProduct(light.intensity / (r*r)) * std::max(0.0f, normal.dot(l));

    // Ls: specular light
    Eigen::Vector3f v = (eye_pos - point).normalized();
    Eigen::Vector3f h = (l + v).normalized();
    Eigen::Vector3f Ls = ks.cwiseProduct(light.intensity / (r*r)) * std::pow(std::max(0.0f, normal.dot(h)), p);

    result_color += Ld + Ls;
}

// La: ambient light
Eigen::Vector3f La = ka.cwiseProduct(amb_light_intensity);
result_color += La;

return result_color * 255.f;
```

As is shown, phong shader equation is: L_phong = L_ambient + L_diffuse + L_specular. It's approximate but good enough.

## Assignment4

CMakeLists.txt:

```cmake
cmake_minimum_required(VERSION 3.10)
project(BezierCurve)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)

add_executable(BezierCurve main.cpp)

target_link_libraries(BezierCurve ${OpenCV_LIBRARIES})
```

### core code

bezier curve's recursive function:

```c++
cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) {
    // TODO: Implement de Casteljau's algorithm
    if (control_points.size() == 1)
        return control_points[0];

    int size = control_points.size();
    std::vector<cv::Point2f> next_control_points(size-1);
    for (int i=0; i<size-1; i++) {
        next_control_points[i] = (1-t)*control_points[i]+t*control_points[i+1];
    }
    // std::cout<<next_control_points;
    return recursive_bezier(next_control_points, t);
}
```

bezier curve's draw function:

```C++
void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) {
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's
    // recursive Bezier algorithm.
    for(double t=0.0; t<=1.0; t+=0.001) {
        auto point = recursive_bezier(control_points, t);
        // window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
        // 遍历相邻的9个像素（包括point自身）
        for(int i=-1; i<=1; i++) {
            for(int j=-1; j<=1; j++) {
                double neighbor_x = point.x+i;
                double neighbor_y = point.y+j;
                double d = sqrt(pow(point.x-int(neighbor_x)-0.5,2) + pow(point.y-int(neighbor_y)-0.5,2));
                double r = 1 - sqrt(2)*d/3;
                window.at<cv::Vec3b>(neighbor_y, neighbor_x)[1] = fmax(window.at<cv::Vec3b>(neighbor_y, neighbor_x)[1], 255*r);
            }
        }
    }
}
```

You can simply use `window.at<cv::Vec3b>(point.y, point.x)[1] = 255` to draw a bezier curve with aliasing. And by iterating 9 adjacent pixels (shown in the code) you can implement anti-aliasing.

Be aware of the difference among SSAA, MSAA and this method!

## Assignment5

CMakeLists.txt keeps the same as the default.

### core code

在光栅化一节，我们用 正交/透视 投影矩阵把三维空间中的物体映射到屏幕的像素中。但在光线追踪一节，我们要用全新的视角看待这个问题。

光线总是从原点发射，打到近平面（也就是屏幕）上。因此，屏幕上经过每个像素点的光线都是向外扩散而非平行的，众多光线组成的轮廓就是视锥体。所以只要定义好每个像素对应的光线向量，就完成了透视投影，无需推导复杂的投影矩阵。

那有没有可能完成正交投影的光线追踪呢？目前代码框架应该是不行的，因为规定了光源的坐标是原点。

```C++
// 考虑如何把屏幕坐标映射到空间坐标
// 1. 获取像素中心坐标，按比例缩放为【-1，1】的平面坐标
// 2. 利用宽高比水平拉伸
// 3. 利用俯仰角同步拉伸x，y
float x = (2*(i+0.5)/width-1) * imageAspectRatio * scale;
float y = (1-2*(j+0.5)/height) * scale;

Vector3f dir = normalize(Vector3f(x, y, -1)); // Don't forget to normalize this direction!
framebuffer[m++] = castRay(eye_pos, dir, scene, 0);
```

Moller Trumbore algorithm:

```C++
Vector3f E1 = v1 - v0;
Vector3f E2 = v2 - v0;
Vector3f S = orig - v0;
Vector3f S1 = crossProduct(dir, E2);
Vector3f S2 = crossProduct(S, E1);
Vector3f res = 1.0/dotProduct(S1, E1) * Vector3f(dotProduct(S2, E2), dotProduct(S1, S), dotProduct(S2, dir));
tnear = res.x; u = res.y; v = res.z;
```

注意：u v 正是三角重心坐标公式中的系数 b1 b2。第三项系数 `b3 = 1 - b1 - b2` 可以由前两者得到，这也阐明了 二维空间中的贴图坐标 到 三维空间坐标 的转换原理。

## Assignment6

CMakeLists.txt keeps the same as the default.

### core code

update code in `render()` of render.cpp:

```C++
// generate primary ray direction
float x = (2 * (i + 0.5) / (float)scene.width - 1) *
            imageAspectRatio * scale;
float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;
// TODO: Find the x and y positions of the current pixel to get the
// direction
//  vector that passes through it.
// Also, don't forget to multiply both of them with the variable
// *scale*, and x (horizontal) variable with the *imageAspectRatio*

// Don't forget to normalize this direction!
Vector3f dir = normalize(Vector3f(x, y, -1));
Ray ray(eye_pos, dir);
framebuffer[m++] = scene.castRay(ray, 0);
```

update code in `Triangle::getIntersection` in Triangle.hpp:

```C++
inter.happened = true;
inter.coords = ray.origin + t_tmp * ray.direction;
inter.normal = normal;
inter.distance = t_tmp;
inter.obj = this;
inter.m = this->m;
```

check if the boundingbox intersects with ray:

```C++
// invDir: ray direction(x,y,z), invDir=(1.0/x,1.0/y,1.0/z), use this because Multiply is faster that Division
// dirIsNeg: ray direction(x,y,z), dirIsNeg=[int(x>0),int(y>0),int(z>0)], use this to simplify your logic
// TODO test if ray bound intersects
float t_xy_near = (pMax.z - ray.origin.z) * invDir.z;
float t_xy_far = (pMin.z - ray.origin.z) * invDir.z;
if(t_xy_near > t_xy_far) std::swap(t_xy_near,t_xy_far);

float t_xz_near = (pMax.y - ray.origin.y) * invDir.y;
float t_xz_far = (pMin.y - ray.origin.y) * invDir.y;
if(t_xz_near > t_xz_far) std::swap(t_xz_near,t_xz_far);

float t_yz_near = (pMax.x - ray.origin.x) * invDir.x;
float t_yz_far = (pMin.x - ray.origin.x) * invDir.x;
if(t_yz_near > t_yz_far) std::swap(t_yz_near,t_yz_far);

// 取交集
float t_enter = std::max(t_yz_near, std::max(t_xy_near, t_xz_near));
float t_exit = std::min(t_yz_far, std::min(t_xy_far, t_xz_far));

return (t_enter < t_exit && t_exit >= 0);
```

traverse the binary tree to get intersection:

```C++
Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection

    float x = ray.direction.x, y = ray.direction.y, z = ray.direction.z;
    std::array<int, 3> dirIsNeg = {int(x>0), int(y>0), int(z>0)};

    if(node->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg)) {
        // leaf node
        if(node->left == nullptr)
            return node->object->getIntersection(ray);

        Intersection inter_left = getIntersection(node->left, ray);
        Intersection inter_right = getIntersection(node->right, ray);

        if(inter_left.distance < inter_right.distance) return inter_left;
        return inter_right;
    }
    else {
        Intersection inter;
        return inter; // 返回空的交集
    }
}
```

注意：分支节点的左右孩子都有可能与光线相交，但实际的交点应位于距离较近的那个孩子。
