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

#define BOOST_TEST_MODULE proximity-query-segment-obb

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include <KineoModel/kppLicense.h>

#include "hpp/geometry/collision/poly-segment.hh"
#include "hpp/geometry/collision/util.hh"

using boost::test_tools::output_test_stream;
using namespace hpp::geometry::collision;

BOOST_AUTO_TEST_CASE (gradient_segment_obb)
{
  // Validate Kineo license.
  if (!CkppLicense::initialize ())
    {
      std::cout << "Failed to validate Kineo license." << std::endl;
      return;
    }

  // Build poly segment.
  CkcdPoint e1 (0, 0, -1);
  CkcdPoint e2 (0, 0 , 1);

  PolySegmentShPtr polySegment = PolySegment::create ();
  polySegment->addSegment (e1, e2);
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
  CkcdAnalysisShPtr analysis = CkcdAnalysis::create ();
  analysis->leftObject (polySegment);
  analysis->rightObject (polyhedron);
  analysis->analysisData ()->analysisType (CkcdAnalysisType::EXACT_DISTANCE);

  polySegment->setAbsolutePosition (CkcdMat4 ().translate (0, 0, 0));

  // Translate polyhedron on the x axis and compute analysis.
  double FLOAT_PERTURBATION = sqrt(pow (2, -24));
  double STEP = FLOAT_PERTURBATION;
  double distance = halfLength + 1e-3;
  while (distance < 4.0)
    {
      polyhedron->setAbsolutePosition
	(CkcdMat4 ().translate (static_cast<kcdReal> (distance), 0, 0));
      analysis->compute ();
      if (distance > halfLength + 1e-3)
	{
	  BOOST_CHECK_EQUAL (analysis->countExactDistanceReports (), 1);
	  // BOOST_CHECK_CLOSE (analysis->exactDistanceReport (0)->distance(),
	  // 		     distance - halfLength,
	  // 		     1e-6);
	  std::cout << distance - halfLength << " " 
		    << analysis->exactDistanceReport (0)->distance()
		    << std::endl;
	}
      distance += STEP;
    }
}
