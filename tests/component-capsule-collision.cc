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

#define BOOST_TEST_MODULE component-capsule

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include <KineoModel/kppLicense.h>

#include <hpp/geometry/component/capsule.hh>

using boost::test_tools::output_test_stream;
using namespace hpp::geometry::component;

BOOST_AUTO_TEST_CASE (component_capsule)
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

  // Build first capsule component.
  CkitPoint3 e1 (0, 0, -1);
  CkitPoint3 e2 (0, 0 , 1);
  double radius1 = 0.1;

  CapsuleShPtr capsule1
    = Capsule::create ("capsule1", e1, e2, radius1, baseVertices, parallels);
  
  // Build second capsule component.
  CkitPoint3 e3 (0, 0, -1);
  CkitPoint3 e4 (0, 0, 1);
  double radius2 = 0.1;

  CapsuleShPtr capsule2
    = Capsule::create ("capsule2", e3, e4, radius2, baseVertices, parallels);

  // Build analysis with capsules.
  CkcdAnalysisShPtr analysis = CkcdAnalysis::create ();
  analysis->leftObject (capsule1);
  analysis->rightObject (capsule2);
  analysis->analysisData ()->analysisType (CkcdAnalysisType::FAST_BOOLEAN_COLLISION);

  capsule1->setAbsolutePosition (CkcdMat4 ().translate (0.f, 0.f, 0.f));

  // Translate right capsule on the x axis and compute analysis.
  hppReal distance = 0;
  while (distance < 0.3)
    {
      capsule2->setAbsolutePosition (CkcdMat4 ().translate (distance, 0.f, 0.f));
      analysis->compute ();
      if (distance < (radius1 + radius2))
  	BOOST_CHECK_EQUAL (analysis->countCollisionReports (), 1);
      distance += 0.01f;
    }

  // Rotate right capsule around y axis and compute analysis.
  hppReal angle = 0;
  CkcdMat4 position;
  while (angle < CkitGeometry::PI / 2)
    {
      position = CkcdMat4 ().rotateY (angle);
      position.setComponent (0, 3, 1.f);
      position.setComponent (1, 3, 0.f);
      position.setComponent (2, 3, 0.f);
      capsule2->setAbsolutePosition (position);
      analysis->compute ();
      if (angle > (6 * CkitGeometry::PI / 20))
      	BOOST_CHECK_EQUAL (analysis->countCollisionReports (), 1);
      angle += CkitGeometry::PI / 20;
    }
}
