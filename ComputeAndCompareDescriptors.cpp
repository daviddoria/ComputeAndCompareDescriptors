/*=========================================================================
 *
 *  Copyright David Doria 2011 daviddoria@gmail.com
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0.txt
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *=========================================================================*/

#include <QApplication>
#include <QCleanlooksStyle>

#include "ComputeAndCompareDescriptorsWidget.h"

int main( int argc, char** argv )
{
  QApplication app( argc, argv );

  QApplication::setStyle(new QCleanlooksStyle);

  ComputeAndCompareDescriptorsWidget computeAndCompareDescriptorsWidget;

  std::cout << "argc: " << argc << std::endl;
  if(argc == 4)
    {
    std::string VTKFileName = argv[1]; // Should be either .vtp or .vts
    std::string maskFileName = argv[2];
    std::string outputFileName = argv[3];

    // Must load mask before point cloud, as the mask is used to mask the point cloud
    computeAndCompareDescriptorsWidget.LoadMask(maskFileName);
    computeAndCompareDescriptorsWidget.LoadData(VTKFileName);

    }
  computeAndCompareDescriptorsWidget.show();

  return app.exec();
}
