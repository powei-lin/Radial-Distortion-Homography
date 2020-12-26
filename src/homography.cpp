#include <iostream>
#include <vector>

#include <opencv2/core.hpp>
#include <Eigen/Core>

using namespace std;
using namespace cv;

int main(){
  vector<string> file_names;
  glob("../data/*.png", file_names);

  for(const auto &name:file_names)
    cout << name << endl;
  
  

  return 0;
}