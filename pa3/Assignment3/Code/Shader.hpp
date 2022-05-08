//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_SHADER_H
#define RASTERIZER_SHADER_H
#include <eigen3/Eigen/Eigen>
#include "Texture.hpp"


struct fragment_shader_payload
{
    fragment_shader_payload()
    {
        texture = nullptr;
    }

    fragment_shader_payload(const Eigen::Vector3f& col, const Eigen::Vector3f& nor,const Eigen::Vector2f& tc, Texture* tex) :
         color(col), normal(nor), tex_coords(tc), texture(tex) {}

    Eigen::Vector3f view_pos; // 当前pixel的view_pos?
    Eigen::Vector3f color;  // 根据各个顶点的颜色，插值得到当前pixel的颜色
    Eigen::Vector3f normal; // 当前pixel的法向量方向
    Eigen::Vector2f tex_coords; // 当前pixel的纹理的uv？
    Texture* texture;       // 使用哪个纹理？
};

struct vertex_shader_payload
{
    Eigen::Vector3f position;
};

#endif //RASTERIZER_SHADER_H
