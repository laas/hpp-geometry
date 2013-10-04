// Copyright (C) 2012 by Antonio El Khoury.
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

#define BOOST_TEST_MODULE segment-obb-proximity-query-performance

#include <time.h>
#include <sys/time.h>

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include <KineoModel/kppLicense.h>
#include <KineoKCDModel/kppKCDPolyhedron.h>

#include <hpp/geometry/collision/poly-segment.hh>
#include <hpp/geometry/collision/util.hh>

using boost::test_tools::output_test_stream;

#define NB_TESTS 1e5

BOOST_AUTO_TEST_CASE (segment_obb_proximity_query_performance)
{
  // Validate Kineo license.
  if (!CkppLicense::initialize ())
    {
      std::cout << "Failed to validate Kineo license." << std::endl;
      return;
    }

  // Validate collision part in component.
  // Build segment component.
  CkcdPoint e1 (0, 0, -1);
  CkcdPoint e2 (0, 0 , 1);
  hpp::geometry::collision::hppReal radius1 = 0.1f;

  hpp::geometry::collision::PolySegmentShPtr segment1
    = hpp::geometry::collision::PolySegment::create ();
  segment1->addSegment (e1, e2, radius1);
  segment1->makeCollisionEntity (CkcdObject::IMMEDIATE_BUILD);
  CkcdObjectEmbeddedShPtr embeddedSegment1
    = CkcdObjectEmbedded::create (segment1);
  
  // Build polyhedron component.
  CkppKCDPolyhedronShPtr polyhedron = CkppKCDPolyhedron::create ("polyhedron");
  unsigned int rank;
  hpp::geometry::collision::hppReal halfLength = 0.5f;

  polyhedron->CkcdPolyhedron::addPoint(-halfLength, -halfLength, -halfLength, rank);
  polyhedron->CkcdPolyhedron::addPoint(-halfLength, -halfLength, halfLength, rank);
  polyhedron->CkcdPolyhedron::addPoint(-halfLength, halfLength, -halfLength, rank);
  polyhedron->CkcdPolyhedron::addPoint(-halfLength, halfLength, halfLength, rank);
  polyhedron->CkcdPolyhedron::addPoint(halfLength, -halfLength, -halfLength, rank);
  polyhedron->CkcdPolyhedron::addPoint(halfLength, -halfLength, halfLength, rank);
  polyhedron->CkcdPolyhedron::addPoint(halfLength, halfLength, -halfLength, rank);
  polyhedron->CkcdPolyhedron::addPoint(halfLength, halfLength, halfLength, rank);

  polyhedron->reserveNTriangles(12);
  polyhedron->CkcdPolyhedron::addTriangle(3,7,5, rank);
  polyhedron->CkcdPolyhedron::addTriangle(3,5,1, rank);
  polyhedron->CkcdPolyhedron::addTriangle(2,3,1, rank);
  polyhedron->CkcdPolyhedron::addTriangle(2,1,0, rank);
  polyhedron->CkcdPolyhedron::addTriangle(6,2,0, rank);
  polyhedron->CkcdPolyhedron::addTriangle(6,0,4, rank);
  polyhedron->CkcdPolyhedron::addTriangle(7,6,4, rank);
  polyhedron->CkcdPolyhedron::addTriangle(7,4,5, rank);
  polyhedron->CkcdPolyhedron::addTriangle(2,6,7, rank);
  polyhedron->CkcdPolyhedron::addTriangle(2,7,3, rank);
  polyhedron->CkcdPolyhedron::addTriangle(1,5,4, rank);
  polyhedron->CkcdPolyhedron::addTriangle(1,4,0, rank);

  polyhedron->makeCollisionEntity (CkcdObject::IMMEDIATE_BUILD);
  CkcdObjectEmbeddedShPtr embeddedPoly
    = CkcdObjectEmbedded::create (polyhedron);

  // Build analysis with segments.
  CkcdAnalysisShPtr analysis = CkcdAnalysis::create ();
  analysis->leftObjectEmbedded (embeddedPoly);
  analysis->rightObjectEmbedded (embeddedSegment1);
  analysis->analysisData ()->analysisType (CkcdAnalysisType::EXACT_DISTANCE);
  analysis->analysisData ()->maximumSpawnedThreadCount (0);

  long TICKS_PER_SECOND = 1e6;
  struct timeval tv_start, tv_stop;

  long time_usec = 0;
  long inner_loop_time;

  srand (time (NULL));
  double randX, randY, randZ, randRoll, randPitch, randYaw;

  // Translate right segment on the x axis and compute analysis.
  for (unsigned i = 0; i < NB_TESTS; ++i)
    {
      randX = double (rand () % 1000) / 100 - 5;
      randY = double (rand () % 1000) / 100 - 5;
      randZ = double (rand () % 1000) / 100 - 5;
      randRoll = double (rand () % 1000) / 1000 * 2 * M_PI - M_PI;
      randPitch = double (rand () % 1000) / 1000 * 2 * M_PI - M_PI;
      randYaw = double (rand () % 1000) / 1000 * 2 * M_PI - M_PI;
      CkitMat4 trans = CkitMat4 ().translate (randX, randY, randZ)
	* CkitMat4 ().rotateZ (randYaw) * CkitMat4 ().rotateY (randPitch)
	* CkitMat4 ().rotateX (randRoll);

      embeddedSegment1->position (trans);

      ::gettimeofday(&tv_start, NULL);
      analysis->compute ();
      ::gettimeofday(&tv_stop, NULL);

      inner_loop_time = ( tv_stop.tv_sec - tv_start.tv_sec ) * TICKS_PER_SECOND
	+ ( tv_stop.tv_usec - tv_start.tv_usec );
      time_usec += inner_loop_time;
    }

  std::cout
    << "Mean execution time = " << (double)time_usec/(double)(NB_TESTS) << "µs"
    << std::endl;
}
