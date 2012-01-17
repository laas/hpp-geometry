// Copyright (C) 2011 by Antonio El Khoury.
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
# include <KineoKCDModel/KineoKCDModel.h>

# include "hpp/geometry/component/fwd.hh"

namespace hpp
{
  namespace component
  {
    class Capsule : public CkppKCDPolyhedron
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

      /// \brief Create a capsule.
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

      /// \brief Create a copy of the component.
      virtual CkppComponentShPtr cloneComponent () const;

      /// \brief Tell whether the component can be cloned using
      /// cloneComponent().
      virtual bool isComponentClonable () const;
      
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

    private:
      CapsuleWkPtr weakPtr_;

      CkppDoublePropertyShPtr heightProperty_;
      CkppDoublePropertyShPtr radiusProperty_;
      CkppIntegerPropertyShPtr baseVerticesProperty_;
      CkppIntegerPropertyShPtr parallelsProperty_;
    };
  } // end of namespace component.    
} // end of namespace hpp.

#endif //! KPP_COMPONENT_CAPSULE_HH_
