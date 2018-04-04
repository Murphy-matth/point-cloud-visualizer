#include "Visualizer.h"

#include <boost/thread/thread.hpp>
#include <pcl/console/parse.h>

namespace visualizer {

using pcl::PointCloud;
using pcl::PointXYZRGB;
using pcl::RGB;
using pcl::visualization::Camera;
using pcl::visualization::KeyboardEvent;
using pcl::visualization::MouseEvent;
using pcl::visualization::PCLVisualizer;
using pcl::visualization::PCL_VISUALIZER_POINT_SIZE;

const std::string Visualizer::kPointCloudName = "My Point Cloud";
bool log_ = false;

Visualizer::Visualizer()
    : z_scale_factor_(1.0),
      use_color_(false),
      interactive_(false),
      show_axis_(false),
      play_(true),
      text_id_(0),
      up_x_(-2),
      camera_(Camera()),
      point_cloud_(new PointCloud<PointXYZRGB>()),
      viewer_(new PCLVisualizer("Capstone Viewer")),
      color_handler_(point_cloud_),
      on_update_callback_([]() {}) {
  viewer_->addPointCloud(point_cloud_, kPointCloudName);
  viewer_->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 3, kPointCloudName);
  set_background_color(0, 0, 0);
  // Window size of the camera (x, y).
  camera_.window_size[0] = 1280;
  camera_.window_size[1] =  880;
  // Window position of the camera (x, y)
  camera_.window_pos[0] = 50;
  camera_.window_pos[0] = 50;
  // Focal point of the camera (x, y, z)
  camera_.focal[0] = 0;
  camera_.focal[1] = 0;
  camera_.focal[2] = 1;
  // Clipping plane [near,far]
  camera_.clip[0] = 0;
  camera_.clip[1] = 10;
  // Position [x,y,z]
  camera_.pos[0] = 1.15061;
  camera_.pos[1] = 1.16022;
  camera_.pos[2] = 1.54807;
  // View up [x,y,z]
  camera_.view[0] = 0.740104;
  camera_.view[1] = -0.646487;
  camera_.view[2] = -0.185204;
}

void Visualizer::Display() {
  viewer_->setBackgroundColor(background_color_.r, background_color_.g, background_color_.b);
  if (show_axis_) {
    viewer_->addCoordinateSystem(1.0);
  }
  if (interactive_) {
    viewer_->registerKeyboardCallback(KeyboardEventOccured, this);
    viewer_->registerMouseCallback(mouseEventOccurred, this);
  }
  viewer_->createViewPortCamera(0);
  viewer_->initCameraParameters();

//  viewer_->setCameraPosition(1.15061, 1.16022, 1.54807, 0.740104, -0.646487, -0.185204);

  while (!viewer_->wasStopped()) {
    viewer_->spinOnce(kUpdateFrequency);
    boost::this_thread::sleep(boost::posix_time::microseconds(50000));
    if (play_) {
      on_update_callback_();
    }
  }
}


void Visualizer::show_axis(bool show_axis) {
  show_axis_ = show_axis;
}

void Visualizer::parse_command_line_args(int argc, char **argv) {
  use_color_ = pcl::console::find_argument(argc, argv, "-c") >= 0;
  show_axis_ = pcl::console::find_argument(argc, argv, "-a") >= 0;
  interactive_ = pcl::console::find_argument(argc, argv, "-i") >= 0;
  log_ = pcl::console::find_argument(argc, argv, "-l") >= 0;
  if (int s = pcl::console::find_argument(argc, argv, "-s") >= 0) {
    z_scale_factor_ = std::stof(argv[s + 1]);
  }
}

void Visualizer::set_background_color(int r, int g, int b) {
  assert(r < 256 && g < 256 && b < 256);
  background_color_.r = r;
  background_color_.g = g;
  background_color_.b = b;
}

void Visualizer::AddPointsFromFile(const std::string& file_path) {
//  point_cloud_->points.clear();
  ifstream input(file_path);
  std::string line;
  std::vector<std::string> values;
  pcl::PointXYZRGB point;
  assert(input.is_open() && !input.eof());

  while (std::getline(input, line)) {
    boost::split(values, line, boost::is_any_of(" "));
    point.x = std::stof(values[0]) * z_scale_factor_;
    point.y = std::stof(values[1]);
    point.z = std::stof(values[2]);
    if (use_color_) {
      point.r = std::stoi(values[3]);
      point.g = std::stoi(values[4]);
      point.b = std::stoi(values[5]);
      point.a = std::stoi(values[6]);
    } else {
      // The default color is grey.
      point.r = (192 + (0 + 5 * std::stoi(values[2]))) % 255;
      point.g = (192 + (0 + 5 * std::stoi(values[2]))) % 255;
      point.b = (192 + (0 + 5 * std::stoi(values[2]))) % 255;
      point.a = 1;
    };
    point_cloud_->points.push_back(point);
  }

  log("There are " + std::to_string(point_cloud_->points.size()) + " points in the point cloud");

  viewer_->updatePointCloud(point_cloud_, kPointCloudName);
  input.close();
}

void Visualizer::set_on_update_callback(std::function<void()> callback) {
  on_update_callback_ = callback;
}

 void Visualizer::KeyboardEventOccured(const KeyboardEvent& event, void* viewer_void) {
  Visualizer* vis = (Visualizer*)viewer_void;

  if (!event.keyDown()) {
    // We only care about key down events.
    return;
  }

  switch (event.getKeyCode()) {
    case 'd': // Move right.
//      vis->translateViewer(-1, 0, 0);
      vis->up_x_ -= 2;
      vis->viewer_->setCameraPosition(-1, 1, vis->up_x_, 0, 0, 0);
      break;
    case 's': // Move down.
      vis->translateViewer(0, -1, 0);
      break;
    case 'a': // Move left.
//      vis->translateViewer(1, 0, 0);
      vis->up_x_ += 2;
      vis->viewer_->setCameraPosition(-1, 1, vis->up_x_, 0, 0, 0);
      break;
    case 'w': // Move up.
      vis->translateViewer(0, 1, 0);
      break;
    case 'c':
      vis->viewer_->resetCameraViewpoint(kPointCloudName);
    case 'p':
      vis->play_or_pause();
    default: // Do nothing.
      break;
  }
}

void Visualizer::mouseEventOccurred(const pcl::visualization::MouseEvent& event, void* viewer_void) {
  Visualizer* vis = (Visualizer*)viewer_void;
  if (event.getButton() == pcl::visualization::MouseEvent::LeftButton
      && event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease) {
    log("Left mouse button released at position (" + std::to_string(event.getX()) + ", " + std::to_string(event.getY()) + ")");
  }
}

void Visualizer::translateViewer(int dx, int dy, int dz) {
  for (pcl::PointXYZRGB& point : point_cloud_->points) {
    point.x += dx;
    point.y += dy;
    point.z += dz;
  }
  viewer_->updatePointCloud(point_cloud_, kPointCloudName);
}

void Visualizer::play_or_pause() {
  play_ = !play_;
}

void Visualizer::log(const std::string& msg) {
  if (log_) {
    std::cout << msg << std::endl;
  }
}


}  // namespace visualizer
