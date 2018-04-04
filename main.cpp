#include <iostream>

#include "Visualizer.h"
#include <pcl/console/parse.h>


int main(int argc, char** argv) {

  visualizer::Visualizer visualizer;

  // Parse command line arguments.
  visualizer.parse_command_line_args(argc, argv);

  // Read in the data.
  ifstream file("../data/names_q.txt");
  std::string line;
  std::getline(file, line);

  if (pcl::console::find_argument(argc, argv, "-u") >= 0) {
    visualizer.set_on_update_callback([&file, &line, &visualizer]() {
      if (std::getline(file, line)) {
        visualizer.AddPointsFromFile("../data/output_q/" + line);
      }
    });
  }

  visualizer.AddPointsFromFile("../data/output_q/1.png");
  visualizer.Display();

  file.close();
}
