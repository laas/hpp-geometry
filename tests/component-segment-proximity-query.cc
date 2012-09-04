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

#define BOOST_TEST_MODULE component-segment-proximity-query

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include <KineoModel/kppLicense.h>

#include <hpp/geometry/component/segment.hh>

using boost::test_tools::output_test_stream;
using namespace hpp::geometry::component;

BOOST_AUTO_TEST_CASE (component_segment_proximity_query)
{
  // Validate Kineo license.
  if (!CkppLicense::initialize ())
    {
      std::cout << "Failed to validate Kineo license." << std::endl;
      return;
    }

  // Validate collision part in component.
  unsigned int baseVertices = 32;
  unsigned int parallels = 32;

  // Build first segment component.
  CkitPoint3 e1 (0, 0, -1);
  CkitPoint3 e2 (0, 0 , 1);
  double radius1 = 0.1;

  SegmentShPtr segment1
    = Segment::create ("segment1", e1, e2, radius1, baseVertices, parallels);
  segment1->makeCollisionEntity (CkcdObject::IMMEDIATE_BUILD);
  
  // Build second segment component.
  CkitPoint3 e3 (0, 0, -1);
  CkitPoint3 e4 (0, 0, 1);
  double radius2 = 0.1;

  SegmentShPtr segment2
    = Segment::create ("segment2", e3, e4, radius2, baseVertices, parallels);
  segment2->makeCollisionEntity (CkcdObject::IMMEDIATE_BUILD);

  // Build analysis with segments.
  CkcdAnalysisShPtr analysis = CkcdAnalysis::create ();
  analysis->leftObject (segment1);
  analysis->rightObject (segment2);
  analysis->analysisData ()->analysisType (CkcdAnalysisType::EXACT_DISTANCE);

  CkitMat4 position;
  segment2->getAbsolutePosition (position);

  // Translate right segment on the x axis and compute analysis.
  hppReal distance = 0.25;
  while (distance < 2)
    {
      CkitMat4 trans = CkcdMat4 ().translate (distance, 0.f, 0.f);
      segment2->setAbsolutePosition (trans * position);
      analysis->compute ();
      BOOST_CHECK_EQUAL (analysis->countExactDistanceReports (), 1);
      CkcdObjectShPtr rightObject = analysis->rightObjects ()[0];
      BOOST_CHECK_EQUAL (!rightObject, 0);
      SegmentShPtr rightSegment = KIT_DYNAMIC_PTR_CAST (Segment, rightObject);
      BOOST_CHECK_EQUAL (!rightSegment, 0);
      CkcdObjectShPtr leftObject = analysis->leftObjects ()[0];
      BOOST_CHECK_EQUAL (!leftObject, 0);
      SegmentShPtr leftSegment = KIT_DYNAMIC_PTR_CAST (Segment, leftObject);
      BOOST_CHECK_EQUAL (!leftSegment, 0);
      BOOST_CHECK_CLOSE (analysis->exactDistanceReport (0)->distance ()
  			 - rightSegment->radius () - leftSegment->radius (),
  			 distance -radius1 - radius2,
  			 1e-4);
      distance += 0.01f;
    }

  // Rotate right segment around y axis and compute analysis.
  distance = 2.f;
  CkcdMat4 trans = CkcdMat4 ().rotateY (CkitGeometry::PI / 2);

  while (distance < 5.f)
    {
      trans.setComponent (0, 3, distance + 1.f);
      trans.setComponent (1, 3, 0.f);
      trans.setComponent (2, 3, 0.f);
      segment2->setAbsolutePosition (trans * position);
      analysis->compute ();
      BOOST_CHECK_EQUAL (analysis->countExactDistanceReports (), 1);
      CkcdObjectShPtr rightObject = analysis->rightObjects ()[0];
      BOOST_CHECK_EQUAL (!rightObject, 0);
      SegmentShPtr rightSegment = KIT_DYNAMIC_PTR_CAST (Segment, rightObject);
      BOOST_CHECK_EQUAL (!rightSegment, 0);
      CkcdObjectShPtr leftObject = analysis->leftObjects ()[0];
      BOOST_CHECK_EQUAL (!leftObject, 0);
      SegmentShPtr leftSegment = KIT_DYNAMIC_PTR_CAST (Segment, leftObject);
      BOOST_CHECK_EQUAL (!leftSegment, 0);
      BOOST_CHECK_CLOSE (analysis->exactDistanceReport (0)->distance ()
  			 - rightSegment->radius () - leftSegment->radius (),
  			 distance -radius1 - radius2,
  			 1e-4);
      distance += 0.01f;
    }
}
