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

#define BOOST_TEST_MODULE collision-detection-capsule-obb

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include <KineoModel/kppLicense.h>

#include "hpp/geometry/collision/poly-capsule.hh"
#include "hpp/geometry/collision/util.hh"

using boost::test_tools::output_test_stream;
using namespace hpp::geometry::collision;

BOOST_AUTO_TEST_CASE (collision_detection_capsule_obb)
{
  // Validate Kineo license.
  if (!CkppLicense::initialize ())
    {
      std::cout << "Failed to validate Kineo license." << std::endl;
      return;
    }

  // Build poly capsule.
  CkcdPoint e1 (0.f, 0.f, -1.f);
  CkcdPoint e2 (0.f, 0.f , 1.f);
  hppReal radius = 0.1f;

  PolyCapsuleShPtr polyCapsule = PolyCapsule::create ();
  polyCapsule->addCapsule (e1, e2, radius);
  polyCapsule->makeCollisionEntity (CkcdObject::IMMEDIATE_BUILD);

  // Build cubic polyhedron and put it in assembly.
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

  CkcdPolyhedronDataShPtr polyData = polyhedron->polyData ();
  CkcdPolyhedronShPtr polyhedron2
    = CkcdPolyhedron::create (polyData);
  CkcdPolyhedronShPtr polyhedron3
    = CkcdPolyhedron::create (polyData);
  CkcdPolyhedronShPtr polyhedron4
    = CkcdPolyhedron::create (polyData);

  polyhedron->setAbsolutePosition
    (CkcdMat4 ().translate (0.f, 0.f, 0.f));
  polyhedron2->setAbsolutePosition
    (CkcdMat4 ().translate (2 * halfLength, 0.f, 0.f));
  polyhedron3->setAbsolutePosition
    (CkcdMat4 ().translate (0.f, 2 * halfLength, 0.f));
  polyhedron4->setAbsolutePosition
    (CkcdMat4 ().translate (0.f, - 2 * halfLength, 0.f));

  CkcdAssemblyShPtr assembly = CkcdAssembly::create ();
  assembly->attach (polyhedron);
  assembly->attach (polyhedron2);
  assembly->attach (polyhedron3);
  assembly->attach (polyhedron4);
  polyhedron->makeCollisionEntity (CkcdObject::IMMEDIATE_BUILD);
  polyhedron2->makeCollisionEntity (CkcdObject::IMMEDIATE_BUILD);
  polyhedron3->makeCollisionEntity (CkcdObject::IMMEDIATE_BUILD);
  polyhedron4->makeCollisionEntity (CkcdObject::IMMEDIATE_BUILD);

  // Build analysis with poly capsule and assembly.
  CkcdAnalysisShPtr analysis1 = CkcdAnalysis::create ();
  analysis1->leftObject (polyCapsule);
  analysis1->rightObject (assembly);
  analysis1->analysisData ()->analysisType (CkcdAnalysisType::FAST_BOOLEAN_COLLISION);

  CkcdAnalysisShPtr analysis2 = CkcdAnalysis::create ();
  analysis2->leftObject (assembly);
  analysis2->rightObject (polyCapsule);
  analysis2->analysisData ()->analysisType (CkcdAnalysisType::FAST_BOOLEAN_COLLISION);

  polyCapsule->setAbsolutePosition (CkcdMat4 ().translate (0.f, 0.f, 0.f));

  // Translate assembly on the x axis and compute analysis1.
  hppReal distance = 0;
  while (distance < 2.0)
    {
      assembly->setAbsolutePosition (CkcdMat4 ().translate (distance, 0.f, 0.f));
      analysis1->compute ();
      analysis2->compute ();
      if (distance < (radius + halfLength ))
      	{
      	  BOOST_CHECK_EQUAL (analysis1->countCollisionReports (), 1);
	  BOOST_CHECK_EQUAL (analysis2->countCollisionReports (), 1);
      	}
      else
      	{
      	  BOOST_CHECK_EQUAL (analysis1->countCollisionReports (), 0);
	  BOOST_CHECK_EQUAL (analysis2->countCollisionReports (), 0);
	}
      distance += 0.01f;
    }
}
