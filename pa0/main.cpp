#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>


Eigen::Vector2f transform2D(const Eigen::Vector2f& origin_vector, float angle, const Eigen::Vector2f& movement) {
    Eigen::Vector3f t;
    t << origin_vector[0], origin_vector[1], 1.0f;
    Eigen::Matrix3f transf;
    transf << std::cos(angle), -std::sin(angle), movement[0], \
              std::sin(angle), std::cos(angle), movement[1], \
              0, 0, 1;
    auto multi_result = transf * t;
    Eigen::Vector2f ans;
    ans << multi_result[0], multi_result[1];
    return ans;
}

int main(){

    // Basic Example of cpp
    std::cout << "Example of cpp \n";
    float a = 1.0, b = 2.0;
    std::cout << a << std::endl;
    std::cout << a/b << std::endl;
    std::cout << std::sqrt(b) << std::endl;
    std::cout << std::acos(-1) << std::endl;
    std::cout << std::sin(30.0/180.0*acos(-1)) << std::endl;

    // Example of vector
    std::cout << "Example of vector \n";
    // vector definition
    Eigen::Vector3f v(1.0f,2.0f,3.0f);
    Eigen::Vector3f w(1.0f,0.0f,0.0f);
    // vector output
    std::cout << "Example of output \n";
    std::cout << v << std::endl;
    // vector add
    std::cout << "Example of add \n";
    std::cout << v + w << std::endl;
    // vector scalar multiply
    std::cout << "Example of scalar multiply \n";
    std::cout << v * 3.0f << std::endl;
    std::cout << 2.0f * v << std::endl;

    // Example of matrix
    std::cout << "Example of matrix \n";
    // matrix definition
    Eigen::Matrix3f i,j;
    i << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
    j << 2.0, 3.0, 1.0, 4.0, 6.0, 5.0, 9.0, 7.0, 8.0;
    // matrix output
    std::cout << "Example of output \n";
    std::cout << i << std::endl;
    // matrix add i + j
    // matrix scalar multiply i * 2.0
    // matrix multiply i * j
    // matrix multiply vector i * v

    // mytest
    Eigen::Vector2f input;
    input << 2.0f, 1.0f;
    Eigen::Vector2f movement;
    movement << 1.0f, 2.0f;
    float PI = std::acos(-1);
    auto ret = transform2D(input, 45.0f / 180 * PI, movement);
    std::cout << "The result is: \n";
    std::cout << ret << std::endl;

    return 0;
}