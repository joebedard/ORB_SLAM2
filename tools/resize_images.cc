#include <time.h>
#include <iostream>

using namespace std;


int main(int argc, char **argv) {

  if (argc != 3)
  {
     cout << "Usage: ./resize_images scaling_ratio source_path destination_path";
     return -1;
  }

  double scalingRatio;

  cout << "Resize Images with scaling ratio " << scalingRatio << endl;

  return 0;
}
