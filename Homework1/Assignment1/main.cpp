#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    // Rz matrix is (cosa, -sina, 0, 0)(sina, cosa, 0, 0)(0, 0, 1, 0)(0, 0, 0, 1)
    Eigen::Matrix4f Rz;
    Rz << cos(rotation_angle / 180.0 * MY_PI), -sin(rotation_angle / 180.0 * MY_PI), 0, 0,
          sin(rotation_angle / 180.0 * MY_PI),  cos(rotation_angle / 180.0 * MY_PI), 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1;

    return Rz * model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    // Get t、r、l、b by eye_fov and aspect_ration
    float t = abs(zNear) * tanf(eye_fov / 2);     // tan需传入角度，tanf传入一个float弧度返回一个float,此处fov/2为弧度值
    float r = t * aspect_ratio;
    float l = -r;
    float b = -t;

    // Create the perspective projection matrix Mpo
    Eigen::Matrix4f Mpo = Eigen::Matrix4f::Identity();;
    Mpo << zNear, 0, 0, 0,
           0, zNear, 0, 0,
           0, 0, zNear + zFar, -(zFar * zFar),
           0, 0, 1, 0;
    
    // Create the orthographic projection matrix Mor
    Eigen::Matrix4f MorTran = Eigen::Matrix4f::Identity();    //Mor平移矩阵
    MorTran << 1, 0, 0, -((l + r) / 2),
               0, 1, 0, -((t + b) / 2),
               0, 0, 1, -((zNear + zFar) / 2),
               0, 0, 0, 1;

    Eigen::Matrix4f MorScal = Eigen::Matrix4f::Identity();    //Mor大小变换矩阵
    MorScal << 2 / (r - l), 0, 0, 0,                          //Notice: t\r\l\b need be float        
               0, 2 / (t - b), 0, 0,    
               0, 0, 2 / (zNear - zFar), 0,
               0, 0, 0, 1;

    projection = MorScal * MorTran * Mpo * projection;
    return projection;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle){
    //R1 = cosa * I
    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f R1 = cosf(angle) * I;
    
    //R2 = (1 - cosa) * n*nT 即 （1 - cosa）* (a[0], a[1], a[2])T * (a[0], a[1], a[2])
    Eigen::Matrix3f R2;
    R2 << axis[0] * axis[0], axis[0] * axis[1], axis[0] * axis[2],
          axis[1] * axis[0], axis[1] * axis[1], axis[1] * axis[2],
          axis[2] * axis[0], axis[2] * axis[1], axis[2] * axis[2];
    R2 = (1 - cosf(angle)) * R2;

    //R3 = sina * (0, -nz, ny)(nz, 0, -nx)(-ny, nx, 0)
    Eigen::Matrix3f R3;
    R3 << 0, -axis[2], axis[1],
          axis[2], 0, -axis[0],
          -axis[1], axis[0], 0;
    R3 = sinf(angle) * R3;

    Eigen::Matrix3f R = R1 + R2 + R3;
    Eigen::Matrix4f Res;
    Res << R(0,0), R(0,1), R(0,2), 0,
           R(1,0), R(1,1), R(1,2), 0,
           R(2,0), R(2,1), R(2,2), 0,
           0, 0, 0, 1;
    return Res;
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
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};    // 相机位置

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};  // 硬编码的三角形

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};    //

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        // 保存图片操作
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        //r.set_model(get_model_matrix(angle));
        Vector3f axis;
        axis << 1, 0, 0;
        r.set_model(get_rotation(axis, angle));
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
