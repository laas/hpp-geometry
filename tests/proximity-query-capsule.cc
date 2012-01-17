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

#define BOOST_TEST_MODULE proximity-query-capsule

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include <KineoModel/kppLicense.h>

#include "hpp/geometry/collision/capsule.hh"
#include "hpp/geometry/collision/poly-capsule.hh"
#include "hpp/geometry/collision/test-tree-capsule.hh"
#include "hpp/geometry/collision/detector-capsule-capsule.hh"
#include "hpp/geometry/collision/util.hh"

using boost::test_tools::output_test_stream;
using namespace hpp::geometry::collision;

BOOST_AUTO_TEST_CASE (proximity_query_capsule)
{
  // Validate Kineo license.
  if (!CkppLicense::initialize ())
    {
      std::cout << "Failed to validate Kineo license." << std::endl;
      return;
    }

  // Register capsule test tree and detector.
  CkcdGlobal::instance ().registerTestTreeLocked
    (&CkcdGlobal::createTestTreeLocked<TestTreeCapsule>);
  CkcdGlobal::instance ().registerDetector
    (&CkcdGlobal::createDetector<DetectorCapsuleCapsule>);

  // Build first poly capsule.
  CkcdPoint e1 (0.f, 0.f, -1.f);
  CkcdPoint e2 (0.f, 0.f , 1.f);
  kcdReal radius1 = 0.1f;

  PolyCapsuleShPtr polyCapsule1 = PolyCapsule::create ();
  polyCapsule1->addCapsule (e1, e2, radius1);
  polyCapsule1->makeCollisionEntity (CkcdObject::IMMEDIATE_BUILD);

  // Build second poly capsule.
  CkcdPoint e3 (0.f, 0.f, -1.f);
  CkcdPoint e4 (0.f, 0.f, 1.f);
  kcdReal radius2 = 0.1f;

  PolyCapsuleShPtr polyCapsule2 = PolyCapsule::create ();
  polyCapsule2->addCapsule (e3, e4, radius2);
  polyCapsule2->makeCollisionEntity (CkcdObject::IMMEDIATE_BUILD);

  // Build analysis with poly capsules.
  CkcdAnalysisShPtr analysis = CkcdAnalysis::create ();
  analysis->leftObject (polyCapsule1);
  analysis->rightObject (polyCapsule2);
  analysis->analysisData ()->analysisType (CkcdAnalysisType::EXACT_DISTANCE);

  polyCapsule1->setAbsolutePosition (CkcdMat4 ().translate (0.f, 0.f, 0.f));

  // Translate right poly capsule on the x axis and compute analysis.
  kcdReal distance = 0.25f;
  while (distance < 2.f)
    {
      polyCapsule2->setAbsolutePosition (CkcdMat4 ().translate (distance, 0.f, 0.f));
      analysis->compute ();
      BOOST_CHECK_EQUAL (analysis->countExactDistanceReports (), 1);
      BOOST_CHECK_CLOSE (analysis->exactDistanceReport (0)->distance (),
			 distance - radius1 - radius2,
			 1e-4);
      distance += 0.01f;
    }

  // Rotate right poly capsule around y axis, translate and compute analysis.
  distance = 2.f;
  CkcdMat4 position = CkcdMat4 ().rotateY (CkitGeometry::PI / 2);

  while (distance < 5.f)
    {
      position.setComponent (0, 3, distance + 1.f);
      position.setComponent (1, 3, 0.f);
      position.setComponent (2, 3, 0.f);
      polyCapsule2->setAbsolutePosition (position);
      analysis->compute ();
      BOOST_CHECK_EQUAL (analysis->countExactDistanceReports (), 1);
      BOOST_CHECK_CLOSE (analysis->exactDistanceReport (0)->distance (),
			 distance - radius1 - radius2,
			 1e-4);
      distance += 0.01f;
    }
}
