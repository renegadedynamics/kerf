// Port of: src/ModelingAlgorithms/TKOffset/Draft/Draft.hxx
// Port of: src/ModelingAlgorithms/TKOffset/Draft/Draft.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>The draft angle of a face.</summary>
public static class Draft
{
    //=================================================================================================

    /// <summary>
    /// The draft angle of the face <paramref name="F"/> with the direction <paramref name="D"/>: for planar faces, and
    /// for cylindrical and conical faces whose axis is colinear with the direction.
    /// </summary>
    /// <exception cref="Standard_DomainError">Another surface, or an axis not colinear with the direction.</exception>
    public static double Angle(in TopoDS_Face F, in gp_Dir D)
    {
        Geom_Surface S = BRep_Tool.Surface(F, out TopLoc_Location Lo)!;
        Type TypeS = S.GetType();
        if (TypeS == typeof(Geom_RectangularTrimmedSurface))
        {
            S = ((Geom_RectangularTrimmedSurface)S).BasisSurface();
            TypeS = S.GetType();
        }

        if (TypeS != typeof(Geom_Plane) && TypeS != typeof(Geom_ConicalSurface)
            && TypeS != typeof(Geom_CylindricalSurface))
        {
            throw new Standard_DomainError();
        }

        double Angle;
        S = (Geom_Surface)S.Transformed(Lo.Transformation());
        if (TypeS == typeof(Geom_Plane))
        {
            gp_Ax3 ax3 = ((Geom_Plane)S).Pln().Position();
            gp_Vec normale = new(ax3.Direction());
            if (!ax3.Direct())
            {
                normale.Reverse();
            }
            if (F.Orientation() == TopAbs_REVERSED)
            {
                normale.Reverse();
            }
            Angle = Math.Asin(normale.Dot(new gp_Vec(D)));
        }
        else if (TypeS == typeof(Geom_CylindricalSurface))
        {
            gp_Cylinder Cy = ((Geom_CylindricalSurface)S).Cylinder();
            double testdir = D.Dot(Cy.Axis().Direction());
            if (Math.Abs(testdir) <= 1.0 - Precision.Angular())
            {
                throw new Standard_DomainError();
            }
            Angle = 0.0;
        }
        else
        { // STANDARD_TYPE(Geom_ConicalSurface)
            gp_Cone Co = ((Geom_ConicalSurface)S).Cone();
            double testdir = D.Dot(Co.Axis().Direction());
            if (Math.Abs(testdir) <= 1.0 - Precision.Angular())
            {
                throw new Standard_DomainError();
            }
            BRepTools.UVBounds(F, out double umin, out double umax, out double vmin, out double vmax);
            ElSLib.D1(umin + umax / 2.0, vmin + vmax / 2.0, Co, out gp_Pnt _, out gp_Vec d1u, out gp_Vec d1v);
            d1u.Cross(d1v);
            d1u.Normalize();
            if (F.Orientation() == TopAbs_REVERSED)
            {
                d1u.Reverse();
            }
            Angle = Math.Asin(d1u.Dot(new gp_Vec(D)));
        }
        return Angle;
    }
}
