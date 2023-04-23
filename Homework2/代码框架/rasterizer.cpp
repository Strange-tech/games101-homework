// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

double cross(double v1[2], double v2[2]) {
    return v1[0]*v2[1] - v1[1]*v2[0];
}

// 只考虑三维坐标的x和y，z为深度，无需考虑
static bool insideTriangle(double x, double y, const Eigen::Vector4f* _v)
{   
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

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = -vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle_with_sample(t);
    }

    // 超采样之后，更新每个像素
    rasterize_triangle_with_pixel();
}

double getInterpolatedZ(double x, double y, const Triangle& t) {
    auto v = t.toVector4();

    auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    z_interpolated *= w_reciprocal;

    return z_interpolated;
}

Eigen::Vector3f getAverageColor(rst::sample_color_list scl) {
    Eigen::Vector3f color1, color2, color3, color4;
    color1 = scl.s1_color;
    color2 = scl.s2_color;
    color3 = scl.s3_color;
    color4 = scl.s4_color;

    double r=(color1.x()+color2.x()+color3.x()+color4.x())/4.0;
    double g=(color1.y()+color2.y()+color3.y()+color4.y())/4.0;
    double b=(color1.z()+color2.z()+color3.z()+color4.z())/4.0;
    return Eigen::Vector3f(r, g, b);
}   

void rst::rasterizer::rasterize_triangle_with_pixel() {
    for(int ind=0; ind<sampling_color_buf.size(); ind++){
        int x = ind % width;
        int y = -ind / width + height - 1;
        Eigen::Vector3f pixel_color = getAverageColor(sampling_color_buf[ind]);

        set_pixel(Eigen::Vector3f(x, y, 0), pixel_color);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle_with_sample(const Triangle& t) {
    auto v = t.toVector4();

    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    int min_x,max_x,min_y,max_y;
    min_x = std::min(std::min(v[0].x(), v[1].x()), v[2].x());
    max_x = std::max(std::max(v[0].x(), v[1].x()), v[2].x());
    min_y = std::min(std::min(v[0].y(), v[1].y()), v[2].y());
    max_y = std::max(std::max(v[0].y(), v[1].y()), v[2].y());

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

    // 不含反走样的基本代码
    // for(int x=min_x; x<=max_x; x++) {
    //     for(int y=min_y; y<=max_y; y++) {
    //         if(insideTriangle(x+0.5, y+0.5, v.data())) {
    //             auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //             float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //             float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //             z_interpolated *= w_reciprocal;

    //             auto ind = (height-1-y)*width + x;
    //             if (depth_buf[ind] < z_interpolated) 
    //                 continue;

    //             // depth_buf[ind] >= z_interpolated 
    //             depth_buf[ind] = z_interpolated;
    //             set_pixel(Eigen::Vector3f(x, y, z_interpolated), t.getColor());
    //         }
    //     }
    // }

    // If so, use the following code to get the interpolated z value.
    // auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    // float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    // float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    // z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        for(auto& it : sampling_color_buf) {
            it.s1_color = Eigen::Vector3f{0, 0, 0};
            it.s2_color = Eigen::Vector3f{0, 0, 0};
            it.s3_color = Eigen::Vector3f{0, 0, 0};
            it.s4_color = Eigen::Vector3f{0, 0, 0};
        }
        
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        for(auto& it : sampling_depth_buf) {
            it.s1_depth = std::numeric_limits<float>::infinity();
            it.s2_depth = std::numeric_limits<float>::infinity();
            it.s3_depth = std::numeric_limits<float>::infinity();
            it.s4_depth = std::numeric_limits<float>::infinity();            
        }
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);

    sampling_color_buf.resize(w * h);
    sampling_depth_buf.resize(w * h);

}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on