#include <time.h>
#include <iostream>
#include <string>
#include <filesystem>
#include <opencv2/opencv.hpp>

using namespace std;

double gScalingRatio;
string gSourcePathStr;
string gDestinPathStr;

// this requires c++ 17
filesystem::path gSourcePath;
filesystem::path gDestinPath;

bool ParseParams(int argc, char * argv[])
{
   if (argc != 4)
   {
      cout << "Usage: ./resize_images scaling_ratio source_path destination_path";
      return false;
   }

   gScalingRatio = atof(argv[1]);
   if (0.0 >= gScalingRatio || 1.0 <= gScalingRatio)
   {
      cout << "scaling_ratio should be between 0.0 and 1.0";
      return false;
   }

   gSourcePathStr = string(argv[2]);
   filesystem::path sourcePath(gSourcePathStr);
   if (!filesystem::exists(sourcePath))
   {
      cout << "source_path does not exist";
      return false;
   }
   gSourcePath = filesystem::path(gSourcePathStr);

   gDestinPathStr = string(argv[3]);
   filesystem::path destinPath(gSourcePathStr);
   if (!filesystem::exists(destinPath))
   {
      cout << "destination_path does not exist";
      return false;
   }
   gDestinPath = filesystem::path(gDestinPathStr);

   return true;
}

int main(int argc, char **argv) try
{
   if (ParseParams(argc, argv))
   {
      cout << "Resize Images with scaling ratio " << gScalingRatio << endl;
      cout << "Source Path: " << gSourcePathStr << endl;
      cout << "Destination Path: " << gDestinPathStr << endl;

      vector<string> sourceFiles, destinFiles;
      for (auto & begit : filesystem::directory_iterator(gSourcePath))
      {
         filesystem::path p = begit.path();
         if (p.has_extension())
         {
            if (0 == p.extension().compare(string(".png")))
            {
               sourceFiles.push_back(p.string());
               destinFiles.push_back(gDestinPathStr + "\\" + p.filename().string());
            }
         }
      }

      for (int i = 0; i < sourceFiles.size(); ++i)
      {
         cv::Mat sourceImg = cv::imread(sourceFiles[i]);
         cv::Mat destinImg;
         cv::resize(sourceImg, destinImg, cv::Size(0,0), gScalingRatio, gScalingRatio, CV_INTER_AREA);
         cout << destinFiles[i] << endl;
         cv::imwrite(destinFiles[i], destinImg);
      }

      return EXIT_SUCCESS;
   }
   else
      return EXIT_FAILURE;
}
catch( cv::Exception & e ) 
{
   string msg = string("cv::Exception: ") + e.what();
   cerr << "main: " << msg << endl;
   return EXIT_FAILURE;
}
catch (const exception & e)
{
   string msg = string("exception: ") + e.what();
   cerr << "main: " << msg << endl;
   return EXIT_FAILURE;
}
catch (...)
{
   string msg = string("There was an unknown exception in the main thread.");
   cerr << "main: " << msg << endl;
   return EXIT_FAILURE;
}

