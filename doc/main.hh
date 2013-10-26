// Copyright (C) 2011, 2012, 2013 LAAS-CNRS
//
// Author: Florent Lamiraux
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

/// \mainpage
///
/// \section hpp_geometry_intro Introduction
///
/// This package extends Kineo collision detector KCD by implementing a
/// new type of geometric primitive called capsule
///
/// A capsule is the surface defined by the points located at a
/// constant distance to a line segment in the workspace.
///
/// \section hpp_geometry_main_classes Main classes
///
/// \subsection hpp_geometry_geometric_primitives Geometric primitives
///
/// The following classes implement two new types of primitives in KCD.
/// \li hpp::geometry::collision::Capsule implements CkcdGeometrySubElement type
///     from KCD. This enables users to embed capsules in hierarchies of
///     bounding volumes and to test them against other KCD geometries deriving
///     from kcdObject.
/// \li hpp::geometry::collision::Segment also implements CkcdGeometrySubElement
///     type from KCD. KCD does not compute penetration distances. Segments
///     enable users to compute distances between capsules and other objects
///     with a negative value in case of penetration. For that users should
///     define a segment corresponding to the capsule axis and substract the
///     capsule radius to the non-negative distance returned by KCD.
///
/// \subsection hpp_geometry_poly Primitive container classes
///
/// The following classes are used to store several instances of the above
/// primitive classes into one geometry. They derive from CkcdGeometry
/// \li hpp::geometry::collision::PolyCapsule,
/// \li hpp::geometry::collision::PolySegment.
///
/// \subsection hpp_geometry_detection_classes Collision query classes
///
/// The following classes are required to compute geometric interactions between
/// capsules (and segments) and other KCD objects. They all derive from
/// CkcdDetector.
///
/// \li hpp::geometry::collision::DetectorBoxCapsule
/// \li hpp::geometry::collision::DetectorCapsuleOBB
/// \li hpp::geometry::collision::DetectorCapsuleCapsule
/// \li hpp::geometry::collision::DetectorCapsuleTriangle
/// \li hpp::geometry::collision::DetectorOBBSegment
/// \li hpp::geometry::collision::DetectorSegmentOBB
/// \li hpp::geometry::collision::DetectorSegmentTriangle
/// \li hpp::geometry::collision::DetectorTriangleSegment
///
/// \subsection hpp_geometry_component Component classes
///
/// The following classes implement components to insert in KineoModel model
/// tree. They derive from CkppPolyhedron. Moreover,
/// \li hpp::geometry::component::Capsule derives from
///     hpp::geometry::collision::PolyCapsule,
/// \li hpp::geometry::component::Segment derives from
///     hpp::geometry::collision::PolySegment.
