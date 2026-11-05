// Port of: src/ModelingAlgorithms/TKOffset/Draft/Draft.hxx
// Port of: src/ModelingAlgorithms/TKOffset/Draft/Draft.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Features.Draft;

/// <summary>The draft angle of a face.</summary>
internal static class DraftTools
{
    //=================================================================================================

    /// <summary>
    /// The draft angle of the face <paramref name="f"/> with the direction <paramref name="d"/>: for planar faces, and
    /// for cylindrical and conical faces whose axis is colinear with the direction.
    /// </summary>
    /// <exception cref="DomainErrorException">Another surface, or an axis not colinear with the direction.</exception>
    public static double Angle(in Face f, in Direction3D d)
    {
        Surface s = Tool.Surface(f, out Placement lo)!;
        Type typeS = s.GetType();
        if (typeS == typeof(TrimmedSurface))
        {
            s = ((TrimmedSurface)s).BasisSurface;
            typeS = s.GetType();
        }

        if (typeS != typeof(PlaneSurface) && typeS != typeof(ConicalSurface)
            && typeS != typeof(CylindricalSurface))
        {
            throw new DomainErrorException();
        }

        double angle;
        s = (Surface)s.Transformed(lo.Transformation);
        if (typeS == typeof(PlaneSurface))
        {
            CoordinateSystem ax3 = ((PlaneSurface)s).Plane.Position;
            Vector3D normale = new(ax3.Direction);
            if (!ax3.IsDirect)
            {
                normale = normale.Reversed();
            }
            if (f.Orientation == Reversed)
            {
                normale = normale.Reversed();
            }
            angle = Math.Asin(normale.Dot(new Vector3D(d)));
        }
        else if (typeS == typeof(CylindricalSurface))
        {
            Cylinder3D cy = ((CylindricalSurface)s).Cylinder;
            double testdir = d.Dot(cy.Axis.Direction);
            if (Math.Abs(testdir) <= 1.0 - global::Kerf.Engine.Foundation.Precision.Angular())
            {
                throw new DomainErrorException();
            }
            angle = 0.0;
        }
        else
        { // STANDARD_TYPE(Geom_ConicalSurface)
            Cone3D co = ((ConicalSurface)s).Cone;
            double testdir = d.Dot(co.Axis.Direction);
            if (Math.Abs(testdir) <= 1.0 - global::Kerf.Engine.Foundation.Precision.Angular())
            {
                throw new DomainErrorException();
            }
            BrepTools.UVBounds(f, out double umin, out double umax, out double vmin, out double vmax);
            global::Kerf.Engine.Geometry.ElSLib.D1(umin + umax / 2.0, vmin + vmax / 2.0, co, out Point3D _, out Vector3D d1u, out Vector3D d1v);
            d1u = d1u.Crossed(d1v);
            d1u = d1u.Normalized();
            if (f.Orientation == Reversed)
            {
                d1u = d1u.Reversed();
            }
            angle = Math.Asin(d1u.Dot(new Vector3D(d)));
        }
        return angle;
    }
}
