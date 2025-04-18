// #include <iostream>
// #include <opencv2/calib3d.hpp>
// #include <opencv2/viz.hpp>

// namespace {

// static void help() {
//   std::cout << "--------------------------------------------------------------------------\n"
//             << "This program shows how to visualize a cube rotated around (1,1,1) and shifted "
//             << "using Rodrigues vector.\n"
//             << "Usage:\n"
//             << "./widget_pose\n"
//             << std::endl;
// }

// }  // namespace

// /**
//  * The viz3d utilizes OpenGL for rendering. Data go through the OpenGL driver on the host. The
//  * window is managed by X11 while the content displayed by the window is managed by OpenGL.
//  */
// int main() {
//   help();

//   cv::viz::Viz3d myWindow("Coordinate Frame");

//   myWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());

//   cv::viz::WLine axis(cv::Point3f(-1.0f, -1.0f, -1.0f), cv::Point3f(1.0f, 1.0f, 1.0f));
//   axis.setRenderingProperty(cv::viz::LINE_WIDTH, 4.0);
//   myWindow.showWidget("Line Widget", axis);

//   cv::viz::WCube cube_widget(cv::Point3f(0.5, 0.5, 0.0), cv::Point3f(0.0, 0.0, -0.5), true,
//                              cv::viz::Color::blue());
//   cube_widget.setRenderingProperty(cv::viz::LINE_WIDTH, 4.0);
//   myWindow.showWidget("Cube Widget", cube_widget);

//   cv::Mat rot_vec = cv::Mat::zeros(1, 3, CV_32F);
//   float translation_phase = 0.0, translation = 0.0;

//   rot_vec.at<float>(0, 0) += (float)CV_PI * 0.01f;
//   rot_vec.at<float>(0, 1) += (float)CV_PI * 0.01f;
//   rot_vec.at<float>(0, 2) += (float)CV_PI * 0.01f;

//   translation_phase += (float)CV_PI * 0.01f;
//   translation = sin(translation_phase);

//   cv::Mat rot_mat;
//   Rodrigues(rot_vec, rot_mat);
//   std::cout << "rot_mat = " << rot_mat << std::endl;
//   cv::Affine3f pose(rot_mat, cv::Vec3f(translation, translation, translation));
//   cv::Affine3f pose2(pose.matrix);
//   std::cout << "pose = " << pose.matrix << std::endl;
//   std::cout << "pose = " << pose2.matrix << std::endl;

//   while (!myWindow.wasStopped()) {
//     /* Rotation using rodrigues */
//     rot_vec.at<float>(0, 0) += (float)CV_PI * 0.01f;
//     rot_vec.at<float>(0, 1) += (float)CV_PI * 0.01f;
//     rot_vec.at<float>(0, 2) += (float)CV_PI * 0.01f;

//     translation_phase += (float)CV_PI * 0.01f;
//     translation = sin(translation_phase);

//     cv::Mat rot_mat1;
//     Rodrigues(rot_vec, rot_mat1);

//     cv::Affine3f pose1(rot_mat1, cv::Vec3f(translation, translation, translation));

//     myWindow.setWidgetPose("Cube Widget", pose1);

//     myWindow.spinOnce(1, true);
//   }

//   return 0;
// }

#include <fstream>
#include <iostream>
#include <opencv2/viz.hpp>

using namespace cv;
using namespace std;

static void help() {
  cout << "--------------------------------------------------------------------------" << endl
       << "This program shows how to use makeTransformToGlobal() to compute required pose,"
       << "how to use makeCameraPose and Viz3d::setViewerPose. You can observe the scene "
       << "from camera point of view (C) or global point of view (G)" << endl
       << "Usage:" << endl
       << "./transformations [ G | C ]" << endl
       << endl;
}

static Mat cvcloud_load() {
  Mat cloud(1, 1889, CV_32FC3);
  ifstream ifs("bunny.ply");

  string str;
  for (size_t i = 0; i < 12; ++i) getline(ifs, str);

  Point3f * data = cloud.ptr<cv::Point3f>();
  float dummy1, dummy2;
  for (size_t i = 0; i < 1889; ++i) ifs >> data[i].x >> data[i].y >> data[i].z >> dummy1 >> dummy2;

  cloud *= 5.0f;
  return cloud;
}

int main(int argn, char ** argv) {
  // help();

  // if (argn < 2)
  // {
  //     cout << "Missing arguments." << endl;
  //     return 1;
  // }

  // bool camera_pov = (argv[1][0] == 'C');

  viz::Viz3d myWindow("Coordinate Frame");
  myWindow.showWidget("Coordinate Widget", viz::WCoordinateSystem());

  {
    Vec3f cam_pos(3.0f, 3.0f, 3.0f), cam_focal_point(3.0f, 3.0f, 2.0f),
        cam_y_dir(-1.0f, 0.0f, 0.0f);

    Affine3f cam_pose = viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);

    viz::WCameraPosition cpw(0.5);  // Coordinate axes
    myWindow.showWidget("CPW", cpw, cam_pose);
  }

  {
    Vec3f cam_pos(2.0f, 3.0f, 3.0f), cam_focal_point(3.0f, 3.0f, 2.0f),
        cam_y_dir(-1.0f, 0.0f, 0.0f);

    Affine3f cam_pose = viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);

    viz::WCameraPosition cpw(0.5);  // Coordinate axes
    myWindow.showWidget("CPW1", cpw, cam_pose);
  }

  // myWindow.showWidget("bunny", cloud_widget, cloud_pose_global);

  // myWindow.setViewerPose(cam_pose);

  myWindow.spin();

  return 0;
}
