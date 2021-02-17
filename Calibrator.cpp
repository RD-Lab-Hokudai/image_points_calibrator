#include <stdio.h>
#include <set>
#include <iostream>
#include <regex>

#include <Open3D/Open3D.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <eigen3/unsupported/Eigen/NonLinearOptimization>
#include <dirent.h>

using namespace std;

int width;
int height;
double f_x;
double f_y;

vector<cv::Mat> imgs;
vector<shared_ptr<open3d::geometry::PointCloud> > pcd_ptrs;
cv::Mat reprojected;

int X = 500;
int Y = 500;
int Z = 500;
int roll = 500;
int pitch = 500;
int yaw = 500;

int data_no = 0;

void reproject()
{
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            reprojected.at<cv::Vec3b>(i, j) = imgs[data_no].at<cv::Vec3b>(i, j);
        }
    }

    double rollVal = (roll - 500) / 1000.0;
    double pitchVal = (pitch - 500) / 1000.0;
    double yawVal = (yaw - 500) / 1000.0;
    Eigen::MatrixXd calibration_mtx(4, 4);
    calibration_mtx << cos(yawVal) * cos(pitchVal), cos(yawVal) * sin(pitchVal) * sin(rollVal) - sin(yawVal) * cos(rollVal), cos(yawVal) * sin(pitchVal) * cos(rollVal) + sin(yawVal) * sin(rollVal), (X - 500) / 100.0,
        sin(yawVal) * cos(pitchVal), sin(yawVal) * sin(pitchVal) * sin(rollVal) + cos(yawVal) * cos(rollVal), sin(yawVal) * sin(pitchVal) * cos(rollVal) - cos(yawVal) * sin(rollVal), (Y - 500) / 100.0,
        -sin(pitchVal), cos(pitchVal) * sin(rollVal), cos(pitchVal) * cos(rollVal), (Z - 500) / 100.0,
        0, 0, 0, 1;
    for (int i = 0; i < pcd_ptrs[data_no]->points_.size(); i++)
    {
        double rawX = pcd_ptrs[data_no]->points_[i][0];
        double rawY = pcd_ptrs[data_no]->points_[i][1];
        double rawZ = pcd_ptrs[data_no]->points_[i][2];

        double r = sqrt(rawX * rawX + rawZ * rawZ);
        double x = calibration_mtx(0, 0) * rawX + calibration_mtx(0, 1) * rawY + calibration_mtx(0, 2) * rawZ + calibration_mtx(0, 3);
        double y = calibration_mtx(1, 0) * rawX + calibration_mtx(1, 1) * rawY + calibration_mtx(1, 2) * rawZ + calibration_mtx(1, 3);
        double z = calibration_mtx(2, 0) * rawX + calibration_mtx(2, 1) * rawY + calibration_mtx(2, 2) * rawZ + calibration_mtx(2, 3);

        if (z > 0)
        {
            int u = (int)(width / 2 + f_x * x / z);
            int v = (int)(height / 2 + f_x * y / z);
            if (0 <= u && u < width && 0 <= v && v < height)
            {
                uchar color = (uchar)min(50 + z * 100, 255.0);
                cv::circle(reprojected, cv::Point(u, v), 2, cv::Scalar(color, color, 0));
            }
        }
    }
    cv::imshow("Image", reprojected);
}

void on_trackbarDataNo(int val, void *object)
{
    data_no = min(val, (int)imgs.size() - 1);
    reproject();
}
void on_trackbarX(int val, void *object)
{
    X = val;
    reproject();
}
void on_trackbarY(int val, void *object)
{
    Y = val;
    reproject();
}
void on_trackbarZ(int val, void *object)
{
    Z = val;
    reproject();
}
void on_trackbarRoll(int val, void *object)
{
    roll = val;
    reproject();
}
void on_trackbarPitch(int val, void *object)
{
    pitch = val;
    reproject();
}
void on_trackbarYaw(int val, void *object)
{
    yaw = val;
    reproject();
}

int main(int argc, char *argv[])
{
    if (argc <= 1)
    {
        cout << "Data folder is not specified!" << endl;
        return 1;
    }

    string data_folder_path = argv[1];
    DIR *dir;
    struct dirent *diread;
    set<string> file_names;
    if ((dir = opendir(data_folder_path.c_str())) != nullptr)
    {
        while ((diread = readdir(dir)) != nullptr)
        {
            file_names.insert(diread->d_name);
        }
        closedir(dir);
    }
    else
    {
        cout << "Invalid folder path!" << endl;
        return 1;
    }

    bool size_is_initialized = false;
    // Load image and pcd
    for (auto it = file_names.begin(); it != file_names.end(); it++)
    {
        try
        {
            string str = *it;
            size_t found = str.find(".png");
            if (found == string::npos)
            {
                throw 1;
            }

            string name = str.substr(0, found);
            string img_path = data_folder_path + name + ".png";
            cv::Mat img = cv::imread(img_path);

            string pcd_path = data_folder_path + name + ".pcd";
            open3d::geometry::PointCloud pointcloud;
            auto pcd_ptr = make_shared<open3d::geometry::PointCloud>();
            if (!open3d::io::ReadPointCloud(pcd_path, pointcloud))
            {
                throw 1;
            }

            for (int i = 0; i < pointcloud.points_.size(); i++)
            {
                // Assign position for camera coordinates
                double x = pointcloud.points_[i][1];
                double y = -pointcloud.points_[i][2];
                double z = -pointcloud.points_[i][0];

                pcd_ptr->points_.emplace_back(x, y, z);
            }

            imgs.emplace_back(img);
            pcd_ptrs.emplace_back(pcd_ptr);

            height = img.rows;
            width = img.cols;
            f_x = width / 2;
            f_y = f_x;
            size_is_initialized = true;
        }
        catch (int e)
        {
        }
    }
    if (imgs.size() == 0)
    {
        cout << "No data is found" << endl;
        return 1;
    }

    cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("Data No", "Image", &data_no, imgs.size(), &on_trackbarDataNo);
    cv::createTrackbar("X(-5,5)", "Image", &X, 1000, &on_trackbarX);
    cv::createTrackbar("Y(-5,5)", "Image", &Y, 1000, &on_trackbarY);
    cv::createTrackbar("Z(-5,5)", "Image", &Z, 1000, &on_trackbarZ);
    cv::createTrackbar("Roll(-1,1)", "Image", &roll, 1000, &on_trackbarRoll);
    cv::createTrackbar("Pitch(-1,1)", "Image", &pitch, 1000, &on_trackbarPitch);
    cv::createTrackbar("Yaw(-1,1)", "Image", &yaw, 1000, &on_trackbarYaw);

    reprojected = cv::Mat::zeros(height, width, CV_8UC3);
    reproject();

    cv::waitKey();
    cv::destroyAllWindows();

    return 0;
}