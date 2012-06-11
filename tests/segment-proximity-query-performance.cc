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

#define BOOST_TEST_MODULE segment-proximity-query-performance

#include <time.h>
#include <sys/time.h>
#include <fstream>

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include <KineoModel/kppLicense.h>

#include <hpp/geometry/collision/poly-segment.hh>
#include <hpp/geometry/collision/util.hh>

using boost::test_tools::output_test_stream;

#define NB_TESTS 1e5

BOOST_AUTO_TEST_CASE (segment_proximity_query_performance)
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
  CkcdPoint e1 (0, 0, -1);
  CkcdPoint e2 (0, 0 , 1);
  hpp::geometry::collision::hppReal radius1 = 0.;

  hpp::geometry::collision::PolySegmentShPtr segment1
    = hpp::geometry::collision::PolySegment::create ();
  segment1->addSegment (e1, e2, radius1);
  segment1->makeCollisionEntity (CkcdObject::IMMEDIATE_BUILD);
  CkcdObjectEmbeddedShPtr embeddedSegment1
    = CkcdObjectEmbedded::create (segment1);

  // Build second segment component.
  CkcdPoint e3 (0, 0, -1);
  CkcdPoint e4 (0, 0, 1);
  hpp::geometry::collision::hppReal radius2 = 0.1;

  hpp::geometry::collision::PolySegmentShPtr segment2
    = hpp::geometry::collision::PolySegment::create ();
  segment2->addSegment (e3, e4, radius2);
  segment2->makeCollisionEntity (CkcdObject::IMMEDIATE_BUILD);
  CkcdObjectEmbeddedShPtr embeddedSegment2
    = CkcdObjectEmbedded::create (segment2);

  // Build analysis with segments.
  CkcdAnalysisShPtr analysis = CkcdAnalysis::create ();
  analysis->leftObjectEmbedded (embeddedSegment1);
  analysis->rightObjectEmbedded (embeddedSegment2);

  analysis->analysisData ()->analysisType (CkcdAnalysisType::EXACT_DISTANCE);
  analysis->analysisData ()->maximumSpawnedThreadCount (0);

  long TICKS_PER_SECOND = 1e6;
  struct timeval tv_start, tv_stop;

  long time_usec = 0;
  long inner_loop_time;

  srand (time (NULL));
  double randX, randY, randZ, randRoll, randPitch, randYaw;

  std::ofstream distFile;
  distFile.open ("distance-of-x.dat");

  // Translate right segment on the x axis and compute analysis.
  for (unsigned i = 0; i < NB_TESTS; ++i)
    {
      randX = 1e-8;
      randY = 0;//double (rand () % int(1e5)) / 1e5 / 1e5 - 5e-6;
      randZ = 0;//double (rand () % int(1e5)) / 1e5 / 1e5 - 5e-6;
      randRoll = 0;//double (rand () % int(1e5)) / 1e5 / 1e5 - 5e-6;
      randPitch = 0;//double (rand () % int(1e5)) / 1e5 / 1e5 - 5e-6;
      randYaw = 0;//double (rand () % int(1e5)) / 1e5 / 1e5 - 5e-6;
      CkitMat4 trans = CkitMat4 ().translate (randX, randY, randZ)
	* CkitMat4 ().rotateZ (randYaw) * CkitMat4 ().rotateY (randPitch)
	* CkitMat4 ().rotateX (randRoll);

      embeddedSegment2->position (trans * embeddedSegment2->position ());

      ::gettimeofday(&tv_start, NULL);
      analysis->compute ();
      analysis->setPriorityReport(CkcdReportShPtr());
      ::gettimeofday(&tv_stop, NULL);

      double distance;
      if (analysis->exactDistanceReport (0))
      	distance = analysis->exactDistanceReport (0)->distance ();
      else
	{
	  distance = 0;
	  // std::cout << "epsilon "
	  // 	    << std::numeric_limits<hppReal>::epsilon () << std::endl;
	  // std::cout << "x " << embeddedSegment2->position ()(0,3) << std::endl;
	  // std::cin.get ();
	}

      distFile << embeddedSegment2->position ()(0,3)
	       << " " << distance
	       << std::endl;

      analysis->clearResults ();

      inner_loop_time = ( tv_stop.tv_sec - tv_start.tv_sec ) * TICKS_PER_SECOND
	+ ( tv_stop.tv_usec - tv_start.tv_usec );
      time_usec += inner_loop_time;
    }

  distFile.close ();
    
  std::cout
    << "Mean execution time = " << (double)time_usec/(double)(NB_TESTS) << "µs"
    << std::endl;
}
