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


/**
 * \brief Declaration of Capsule component.
 */

#ifndef KPP_COMPONENT_CAPSULE_HH_
# define KPP_COMPONENT_CAPSULE_HH_

# include <KineoModel/KineoModel.h>

# include "hpp/geometry/collision/poly-capsule.hh"
# include "hpp/geometry/component/fwd.hh"

namespace hpp
{
  namespace geometry
  {
    namespace component
    {
      class Capsule : public CkppPolyhedron, public collision::PolyCapsule
      {
      public:
	KPP_DECLARE_PROPERTY(HEIGHT);
	KPP_DECLARE_PROPERTY(RADIUS);
	KPP_DECLARE_PROPERTY(BASE_VERTICES);
	KPP_DECLARE_PROPERTY(PARALLELS);

	/// \brief Get height of the capsule.
	double height () const;

	/// \brief Set height of the capsule.
	void height (const double& height);

	/// \brief Get raidus of the capsule.
	double radius () const;

	/// \brief Set radius of the capsule.
	void radius (const double& radius);

	/// \brief Get number of base vertices around the cylindrical
	/// base of the capsule.
	unsigned int baseVertices () const;

	/// \brief Set number of base vertices.
	void baseVertices (const unsigned int baseVertices);

	/// \brief Get number of parallels on the half-spheres of the
	/// capsule.
	unsigned int parallels () const;

	/// \brief Set number of parallels on half-spheres.
	void parallels (const unsigned int parallels);

	/// \brief Destructor
	virtual ~Capsule ();

	/// \brief Create a default capsule.
	///
	/// The capsule center is at the frame origin, and its main
	/// axis is oriented along the X axis.
	///
	/// \param name name of the capsule
	/// \param height height of the cylindrical base of the capsule
	/// \param radius radius of the capsule
	///
	/// \param baseVertices number of base vertices on the
	/// cylindrical base of the capsule
	///
	/// \param parallels number of parallels on each half-sphere of
	/// the capsule
	static CapsuleShPtr create (const std::string& name,
				    const double& height,
				    const double& radius,
				    const unsigned int baseVertices = 32,
				    const unsigned int parallels = 32);

	/// \brief Create a capsule with a given transformation.
	///
	/// \param name name of the capsule
	/// \param endPoint1 first end point of the capsule segment
	/// \param endPoint2 second end point of the capsule segment
	/// \param radius radius of the capsule
	///
	/// \param baseVertices number of base vertices on the
	/// cylindrical base of the capsule
	///
	/// \param parallels number of parallels on each half-sphere of
	/// the capsule
	static CapsuleShPtr create (const std::string& name,
				    const CkitPoint3& endPoint1,
				    const CkitPoint3& endPoint2,
				    const double& radius,
				    const unsigned int baseVertices = 32,
				    const unsigned int parallels = 32);

	/// \brief Create a capsule with a given transformation.
	///
	/// \param name name of the capsule
	/// \param endPoint1 first end point of the capsule segment
	/// \param endPoint2 second end point of the capsule segment
	/// \param radius radius of the capsule
	///
	/// \param baseVertices number of base vertices on the
	/// cylindrical base of the capsule
	///
	/// \param parallels number of parallels on each half-sphere of
	/// the capsule
	static CapsuleShPtr create (const std::string& name,
				    const double& height,
				    const double& radius,
				    const CkitMat4& transformation,
				    const unsigned int baseVertices = 32,
				    const unsigned int parallels = 32);

	/// \brief Create a copy of the component.
	virtual CkppComponentShPtr cloneComponent () const;

	/// \brief Tell whether the component can be cloned using
	/// cloneComponent().
	virtual bool isComponentClonable () const;

	/// \brief Set the absolute position of the component.
	virtual void setAbsolutePosition (const CkitMat4 &i_matrix);

	/// \brief Retrieve the absolute position of the component.
	virtual void getAbsolutePosition (CkitMat4 &o_matrix) const;

	/// \brief Retrieve the absolute position of the component.
	virtual void getAbsolutePosition (CkcdMat4 &o_matrix) const;

	/// \brief Set the relative position of the component.
	virtual void setRelativePosition (const CkitMat4 &i_matrix);

	/// \brief Retrieve the relative position of the component.
	virtual void getRelativePosition (CkitMat4 &o_matrix) const;

	/// \brief Retrieve the orientation of the bounding box
	/// around the component.
	virtual ktStatus getBBMatrixOrientation (CkitMat4 &o_matrix) const;

	/// \brief Retrieve the half-lengths of the bounding box
	/// around the component.
	virtual ktStatus getBBHalfLengths (float &x, float &y, float &z) const;

	/// \brief Return whether the component is a "collision leaf".
	virtual bool isCollisionLeaf () const;

	/// \brief Compute and returns the default motion frame.
	virtual CkitMat4 computeDefaultFrame () const;

	/// \brief Add a point to the polyhedron.
	virtual ktStatus addPoint (const CkitPoint3 &i_point);

	/// \brief Add a triangle to the polyhedron.
	virtual ktStatus addTriangle (unsigned int i_p1,
				      unsigned int i_p2,
				      unsigned int i_p3);

	/// \brief Add a polygon to the polyhedron.
	virtual void addPolygon (const std::vector< int > &i_verticesVector);

	/// \brief Return the number of points in the polyhedron.
	virtual unsigned int countPoints () const;

	/// \brief Return the number of triangles in the polyhedron.
	virtual unsigned int countTriangles () const;

	/// \brief Get the triangle corresponding to the given rank.
	virtual void getTriangle (const unsigned int i_rank,
				  unsigned int &o_p1,
				  unsigned int &o_p2,
				  unsigned int &o_p3) const;

	/// \brief Get the point corresponding to the given rank.
	virtual void getPoint (const unsigned int i_rank,
			       float &o_x, float &o_y, float &o_z) const;

	/// \brief Get the point corresponding to the given rank.
	virtual void getPoint (const unsigned int i_rank,
			       CkitPoint3 &o_point) const;

	/// \brief Explode the polyhedron, transforming it into an
	/// assembly.
	virtual CkppAssemblyComponentShPtr
	explode (const CkitProgressDelegateShPtr &i_delegate
		 =CkitProgressDelegateShPtr());

      protected:
	/// \brief Constructor.
	Capsule ();

	/// \brief Initialize capsule.
	ktStatus init (const CapsuleWkPtr weakPtr,
		       const std::string& name,
		       const double& height,
		       const double& radius,
		       const unsigned int baseVertices,
		       const unsigned int parallels);

	/// \brief Allow the component to return its user-visible
	/// properties.
	virtual void fillPropertyVector (std::vector<CkppPropertyShPtr>&
					 propertyVector) const;

	/// \brief Notify a component that the value of one of its
	/// properties has changed.
	virtual bool modifiedProperty (const CkppPropertyShPtr& property);

	/// \brief Let the component know that the value of one of its
	/// properties is about to be retrieved.
	virtual void updateProperty (const CkppPropertyShPtr& property);

	virtual CkppPolyhedronShPtr createPolyhedronFromPolyExpandingData
	(const CkppPolyExpandingDataShPtr& i_polyExpandingData,
	 unsigned int i_offset);

      private:
	CapsuleWkPtr weakPtr_;

	CkppDoublePropertyShPtr heightProperty_;
	CkppDoublePropertyShPtr radiusProperty_;
	CkppIntegerPropertyShPtr baseVerticesProperty_;
	CkppIntegerPropertyShPtr parallelsProperty_;

	CkcdPolyhedronShPtr polyhedron_;
      };

    } // end of namespace component.
  } // end of namespace geometry.
} // end of namespace hpp.

#endif //! KPP_COMPONENT_CAPSULE_HH_
