// Copyright (C) 2011, 2012 by Antonio El Khoury.
//
// This file is part of the hpp-geometry.
//
// hpp-geometry is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// hpp-geometry is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with hpp-geometry.  If not, see <http://www.gnu.org/licenses/>.

#define BOOST_TEST_MODULE component-segment

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include <KineoModel/kppLicense.h>

#include <hpp/geometry/component/segment.hh>

using boost::test_tools::output_test_stream;
using namespace hpp::geometry::component;

BOOST_AUTO_TEST_CASE (component_segment)
{
  // Validate Kineo license.
  if (!CkppLicense::initialize ())
    {
      std::cout << "Failed to validate Kineo license." << std::endl;
      return;
    }

  // Create default segment component from height and radius.
  double height1 = 1.;
  double radius1 = 0.3;
  unsigned int baseVertices1 = 32;
  unsigned int parallels1 = 32;

  SegmentShPtr segment1
    = Segment::create ("segment",height1, radius1, baseVertices1, parallels1);
  
  BOOST_CHECK_EQUAL (!segment1, 0);
  BOOST_CHECK_CLOSE (segment1->height (), height1, 1e-4);
  BOOST_CHECK_CLOSE (segment1->radius (), radius1, 1e-4);
  BOOST_CHECK_EQUAL (segment1->baseVertices (), baseVertices1);
  BOOST_CHECK_EQUAL (segment1->parallels (), parallels1);

  // Create segment component from end points and radius.
  CkitPoint3 endPoint1 (1, 2, 3);
  CkitPoint3 endPoint2 (4, 5, 6);
  double radius2 = 0.5;
  unsigned int baseVertices2 = 32;
  unsigned int parallels2 = 32;

  SegmentShPtr segment2
    = Segment::create ("segment", endPoint1, endPoint2, radius2,
		       baseVertices2, parallels2);
  
  BOOST_CHECK_EQUAL (!segment2, 0);

  // Create segment component from height, radius and absolute
  // transformation.
  double height3 = 1.;
  double radius3 = 0.3;
  CkitMat4 transformation3;
  transformation3.rotateX (M_PI);
  unsigned int baseVertices3 = 32;
  unsigned int parallels3 = 32;

  SegmentShPtr segment3
    = Segment::create ("segment", height1, radius1, transformation3,
		       baseVertices1, parallels1);

  BOOST_CHECK_EQUAL (!segment3, 0);
}
