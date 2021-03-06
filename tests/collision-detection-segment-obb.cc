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

#define BOOST_TEST_MODULE collision-detection-segment-obb

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include <KineoModel/kppLicense.h>

#include "hpp/geometry/collision/poly-segment.hh"
#include "hpp/geometry/collision/util.hh"

using boost::test_tools::output_test_stream;
using namespace hpp::geometry::collision;

BOOST_AUTO_TEST_CASE (collision_detection_segment_obb)
{
  // Validate Kineo license.
  if (!CkppLicense::initialize ())
    {
      std::cout << "Failed to validate Kineo license." << std::endl;
      return;
    }

  // Build poly segment.
  CkcdPoint e1 (0.f, 0.f, -1.f);
  CkcdPoint e2 (0.f, 0.f , 1.f);
  hppReal radius = 0.1f;

  PolySegmentShPtr polySegment = PolySegment::create ();
  polySegment->addSegment (e1, e2, radius);
  polySegment->makeCollisionEntity (CkcdObject::IMMEDIATE_BUILD);

  // Build cubic polyhedron.
  CkcdPolyhedronShPtr polyhedron = CkcdPolyhedron::create ();
  unsigned int rank;
  hppReal halfLength = 0.5;

  polyhedron->addPoint(-halfLength, -halfLength, -halfLength, rank);
  polyhedron->addPoint(-halfLength, -halfLength, halfLength, rank);
  polyhedron->addPoint(-halfLength, halfLength, -halfLength, rank);
  polyhedron->addPoint(-halfLength, halfLength, halfLength, rank);
  polyhedron->addPoint(halfLength, -halfLength, -halfLength, rank);
  polyhedron->addPoint(halfLength, -halfLength, halfLength, rank);
  polyhedron->addPoint(halfLength, halfLength, -halfLength, rank);
  polyhedron->addPoint(halfLength, halfLength, halfLength, rank);

  polyhedron->reserveNTriangles(12);
  polyhedron->addTriangle(3,7,5, rank);
  polyhedron->addTriangle(3,5,1, rank);
  polyhedron->addTriangle(2,3,1, rank);
  polyhedron->addTriangle(2,1,0, rank);
  polyhedron->addTriangle(6,2,0, rank);
  polyhedron->addTriangle(6,0,4, rank);
  polyhedron->addTriangle(7,6,4, rank);
  polyhedron->addTriangle(7,4,5, rank);
  polyhedron->addTriangle(2,6,7, rank);
  polyhedron->addTriangle(2,7,3, rank);
  polyhedron->addTriangle(1,5,4, rank);
  polyhedron->addTriangle(1,4,0, rank);

  polyhedron->makeCollisionEntity (CkcdObject::IMMEDIATE_BUILD);

  // Build analysis with poly segment and polyhedron.
  CkcdAnalysisShPtr analysis1 = CkcdAnalysis::create ();
  analysis1->leftObject (polySegment);
  analysis1->rightObject (polyhedron);
  analysis1->analysisData ()->analysisType (CkcdAnalysisType::FAST_BOOLEAN_COLLISION);

  CkcdAnalysisShPtr analysis2 = CkcdAnalysis::create ();
  analysis2->leftObject (polyhedron);
  analysis2->rightObject (polySegment);
  analysis2->analysisData ()->analysisType (CkcdAnalysisType::FAST_BOOLEAN_COLLISION);

  polySegment->setAbsolutePosition (CkcdMat4 ().translate (0.f, 0.f, 0.f));

  // Translate polyhedron on the x axis and compute analysis1.
  hppReal distance = 0;
  while (distance < 2.0)
    {
      polyhedron->setAbsolutePosition (CkcdMat4 ().translate (distance, 0.f, 0.f));
      analysis1->compute ();
      analysis2->compute ();
      if (distance < halfLength)
	{
	  BOOST_CHECK_EQUAL (analysis1->countCollisionReports (), 1);
	  BOOST_CHECK_EQUAL (analysis2->countCollisionReports (), 1);
	}
      distance += 0.01f;
    }
}
