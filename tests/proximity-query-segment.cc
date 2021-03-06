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

#define BOOST_TEST_MODULE proximity-query-segment

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include <KineoModel/kppLicense.h>

#include "hpp/geometry/collision/util.hh"
#include "hpp/geometry/collision/poly-segment.hh"

using boost::test_tools::output_test_stream;
using namespace hpp::geometry::collision;

BOOST_AUTO_TEST_CASE (proximity_query_segment)
{
  // Validate Kineo license.
  if (!CkppLicense::initialize ())
    {
      std::cout << "Failed to validate Kineo license." << std::endl;
      return;
    }

  // Build first poly segment.
  CkcdPoint e1 (0.f, 0.f, -1.f);
  CkcdPoint e2 (0.f, 0.f , 1.f);

  PolySegmentShPtr polySegment1 = PolySegment::create ();
  polySegment1->addSegment (e1, e2);
  polySegment1->makeCollisionEntity (CkcdObject::IMMEDIATE_BUILD);

  // Build second poly segment.
  CkcdPoint e3 (0.f, 0.f, -1.f);
  CkcdPoint e4 (0.f, 0.f, 1.f);

  PolySegmentShPtr polySegment2 = PolySegment::create ();
  polySegment2->addSegment (e3, e4);
  polySegment2->makeCollisionEntity (CkcdObject::IMMEDIATE_BUILD);

  // Build analysis with poly segments.
  CkcdAnalysisShPtr analysis = CkcdAnalysis::create ();
  analysis->leftObject (polySegment1);
  analysis->rightObject (polySegment2);
  analysis->analysisData ()->analysisType (CkcdAnalysisType::EXACT_DISTANCE);

  polySegment1->setAbsolutePosition (CkcdMat4 ().translate (0.f, 0.f, 0.f));

  // Translate right poly segment on the x axis and compute analysis.
  hppReal distance = 0.001f;
  while (distance < 2.f)
    {
      polySegment2->setAbsolutePosition (CkcdMat4 ().translate (distance, 0.f, 0.f));
      analysis->compute ();
      BOOST_CHECK_EQUAL (analysis->countExactDistanceReports (), 1);
      BOOST_CHECK_CLOSE (analysis->exactDistanceReport (0)->distance (),
			 distance,
			 1e-4);
      distance += 0.01f;
    }

  // Rotate right poly segment around y axis, translate and compute analysis.
  distance = 2.f;
  CkcdMat4 position =
    CkcdMat4 ().rotateY (static_cast<kcdReal> (CkitGeometry::PI) / 2);

  while (distance < 5.f)
    {
      position.setComponent (0, 3, distance + 1.f);
      position.setComponent (1, 3, 0.f);
      position.setComponent (2, 3, 0.f);
      polySegment2->setAbsolutePosition (position);
      analysis->compute ();
      BOOST_CHECK_EQUAL (analysis->countExactDistanceReports (), 1);
      BOOST_CHECK_CLOSE (analysis->exactDistanceReport (0)->distance (),
			 distance,
			 1e-4);
      distance += 0.01f;
    }
}
