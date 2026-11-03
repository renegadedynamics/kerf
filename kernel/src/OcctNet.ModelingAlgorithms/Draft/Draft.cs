// Port of: src/ModelingAlgorithms/TKOffset/Draft/Draft.hxx
// Port of: src/ModelingAlgorithms/TKOffset/Draft/Draft.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;

namespace Occt;

/// <summary>The draft angle of a face.</summary>
public static class Draft
{
    //=================================================================================================

    /// <summary>
    /// The draft angle of the face <paramref name="F"/> with the direction <paramref name="D"/>: for planar faces, and
    /// for cylindrical and conical faces whose axis is colinear with the direction.
    /// </summary>
    /// <exception cref="DomainErrorException">Another surface, or an axis not colinear with the direction.</exception>
    public static double Angle(in Face F, in Direction3D D)
    {
        Surface S = Tool.Surface(F, out Placement Lo)!;
        Type TypeS = S.GetType();
        if (TypeS == typeof(TrimmedSurface))
        {
            S = ((TrimmedSurface)S).BasisSurface();
            TypeS = S.GetType();
        }

        if (TypeS != typeof(PlaneSurface) && TypeS != typeof(ConicalSurface)
            && TypeS != typeof(CylindricalSurface))
        {
            throw new DomainErrorException();
        }

        double Angle;
        S = (Surface)S.Transformed(Lo.Transformation());
        if (TypeS == typeof(PlaneSurface))
        {
            CoordinateSystem ax3 = ((PlaneSurface)S).Pln().Position();
            Vector3D normale = new(ax3.Direction());
            if (!ax3.Direct())
            {
                normale.Reverse();
            }
            if (F.Orientation() == Reversed)
            {
                normale.Reverse();
            }
            Angle = Math.Asin(normale.Dot(new Vector3D(D)));
        }
        else if (TypeS == typeof(CylindricalSurface))
        {
            Cylinder Cy = ((CylindricalSurface)S).Cylinder();
            double testdir = D.Dot(Cy.Axis().Direction());
            if (Math.Abs(testdir) <= 1.0 - global::Kerf.Engine.Foundation.Precision.Angular())
            {
                throw new DomainErrorException();
            }
            Angle = 0.0;
        }
        else
        { // STANDARD_TYPE(Geom_ConicalSurface)
            Cone Co = ((ConicalSurface)S).Cone();
            double testdir = D.Dot(Co.Axis().Direction());
            if (Math.Abs(testdir) <= 1.0 - global::Kerf.Engine.Foundation.Precision.Angular())
            {
                throw new DomainErrorException();
            }
            BrepTools.UVBounds(F, out double umin, out double umax, out double vmin, out double vmax);
            global::Kerf.Engine.Geometry.ElSLib.D1(umin + umax / 2.0, vmin + vmax / 2.0, Co, out Point3D _, out Vector3D d1u, out Vector3D d1v);
            d1u.Cross(d1v);
            d1u.Normalize();
            if (F.Orientation() == Reversed)
            {
                d1u.Reverse();
            }
            Angle = Math.Asin(d1u.Dot(new Vector3D(D)));
        }
        return Angle;
    }
}
