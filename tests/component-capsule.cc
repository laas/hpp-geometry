// Copyright (C) 2011 by Antonio El Khoury.
//
// This file is part of the kcd-capsule.
//
// kcd-capsule is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// kcd-capsule is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with kcd-capsule.  If not, see <http://www.gnu.org/licenses/>.

#define BOOST_TEST_MODULE component-capsule

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include <KineoModel/kppLicense.h>

#include <kpp/component/capsule.hh>

using boost::test_tools::output_test_stream;
using namespace kpp;

BOOST_AUTO_TEST_CASE (component_capsule)
{
  // Validate Kineo license.
  if (!CkppLicense::initialize ())
    {
      std::cout << "Failed to validate Kineo license." << std::endl;
      return;
    }

  // Create capsule component.
  double height = 1.;
  double radius = 0.1;
  unsigned int baseVertices = 10;
  unsigned int parallels = 5;

  CapsuleShPtr capsule
    = Capsule::create ("capsule",height, radius, baseVertices, parallels);
  
  BOOST_CHECK_EQUAL (!capsule, 0);
  BOOST_CHECK_CLOSE (capsule->height (), height, 1e-4);
  BOOST_CHECK_CLOSE (capsule->radius (), radius, 1e-4);
  BOOST_CHECK_EQUAL (capsule->baseVertices (), baseVertices);
  BOOST_CHECK_EQUAL (capsule->parallels (), parallels);
}
