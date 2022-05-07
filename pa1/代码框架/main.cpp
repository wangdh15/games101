#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

// move the object to origin.
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

// 该函数的作用是，将物体沿着z轴旋转rotation_angle度数（角度）
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    Eigen::Matrix4f translate;
    float angle_pi = rotation_angle / 180.f * MY_PI;

    translate << std::cos(angle_pi), -std::sin(angle_pi), 0, 0,
                 std::sin(angle_pi), std::cos(angle_pi), 0, 0,
                 0, 0, 1, 0,
                 0, 0, 0, 1;
    model = translate * model;

    return model;
}

// 获取project矩阵
// eye_fov为角度
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    Eigen::Matrix4f M_ortho, M_persp_ortho;
    // first do persp->ortho
    float eye_fov_pi = eye_fov / 180.f * MY_PI;
    float t = fabs(zNear) * std::tan(eye_fov_pi / 2);
    float r = aspect_ratio * t;
    float b = -t, l = -r;
    float n = -std::fabs(zNear), f = -std::fabs(zFar);

    M_persp_ortho  << -std::fabs(zNear), 0, 0, 0,
               0, -std::fabs(zNear), 0, 0,
               0, 0, -std::fabs(zNear) - std::fabs(zFar), -std::fabs(zNear * zFar),
               0, 0, 1, 0;

    M_ortho << 2.0 / (r - l), 0, 0, 0,
                     0, 2.0 / (t - b), 0, 0,
                     0, 0, 2.0 / (n - f), 0,
                     0, 0, 0, 1;

    projection = M_ortho * M_persp_ortho * projection;

    return projection;
}


// 绕axis方向的轴旋转angle角度
Eigen::Matrix4f get_rotation(Vector3f axis, float angle) {
    float angle_pi = angle / 180 * MY_PI;
    Eigen::Matrix3f tmp1, tmp2, tmp3, tmp4;
    tmp1 = std::cos(angle_pi) * Eigen::Matrix3f::Identity();
    tmp2 = (1 - std::cos(angle_pi)) * axis * axis.transpose();
    tmp4 << 0, -axis[2], axis[1],
            axis[2], 0, -axis[0],
            -axis[1], axis[0], 0;
    tmp3 = std::sin(angle_pi) * tmp4;
    Eigen::Matrix3f ret3d = tmp1 + tmp2 + tmp3;
    Eigen::Matrix4f ret = Eigen::Matrix4f::Identity();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            ret(i,j) = ret3d(i, j);
        }
    }
    return ret;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // r.set_model(get_model_matrix(angle));
        r.set_model(get_rotation({0, 0, 1}, angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
