// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeCylindricalSurface.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeCylindricalSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>
/// Constructions of a Geom_CylindricalSurface: from a gp_Cylinder, an axis (system) and a radius, a circle, three
/// points, or coaxial to a cylinder at a distance or through a point.
/// </summary>
public sealed class GC_MakeCylindricalSurface : GC_Root
{
    private Geom_CylindricalSurface? TheCylinder;

    /// <summary>A cylinder from the gp_Cylinder C.</summary>
    public GC_MakeCylindricalSurface(in Cylinder C)
    {
        TheError = gce_Done;
        TheCylinder = new Geom_CylindricalSurface(C);
    }

    /// <summary>A cylinder in the axis system A2; fails with gce_NegativeRadius if Radius is negative.</summary>
    public GC_MakeCylindricalSurface(in Frame3D A2, double Radius)
    {
        if (Radius < 0.0)
        {
            TheError = gce_NegativeRadius;
        }
        else
        {
            TheError = gce_Done;
            TheCylinder = new Geom_CylindricalSurface(new CoordinateSystem(A2), Radius);
        }
    }

    /// <summary>A cylinder of axis A1; fails with the status of gce_MakeCylinder.</summary>
    public GC_MakeCylindricalSurface(in Axis3D A1, double Radius)
    {
        gce_MakeCylinder Cyl = new gce_MakeCylinder(A1, Radius);
        TheError = Cyl.Status();
        if (TheError == gce_Done)
        {
            TheCylinder = new Geom_CylindricalSurface(Cyl.Value());
        }
    }

    /// <summary>The cylinder whose axis and radius are those of Circ.</summary>
    public GC_MakeCylindricalSurface(in Circle3D Circ)
    {
        // gce_MakeCylinder(Circ) always succeeds, so its implicit conversion can't throw.
        Cylinder Cyl = new gce_MakeCylinder(Circ).Value();
        TheCylinder = new Geom_CylindricalSurface(Cyl);
        TheError = gce_Done;
    }

    /// <summary>A cylinder with axis P1P2 through P3; fails with the status of gce_MakeCylinder.</summary>
    public GC_MakeCylindricalSurface(in Point3D P1, in Point3D P2, in Point3D P3)
    {
        gce_MakeCylinder Cyl = new gce_MakeCylinder(P1, P2, P3);
        TheError = Cyl.Status();
        if (TheError == gce_Done)
        {
            TheCylinder = new Geom_CylindricalSurface(Cyl.Value());
        }
    }

    /// <summary>A cylinder coaxial to Cyl with radius |Cyl.Radius() − Dist|.</summary>
    public GC_MakeCylindricalSurface(in Cylinder Cyl, double Dist)
    {
        TheError = gce_Done;
        double R = Math.Abs(Cyl.Radius() - Dist);
        TheCylinder = new Geom_CylindricalSurface(Cyl);
        TheCylinder.SetRadius(R);
    }

    /// <summary>A cylinder coaxial to Cyl through Point.</summary>
    public GC_MakeCylindricalSurface(in Cylinder Cyl, in Point3D Point)
    {
        TheError = gce_Done;
        Cylinder C = Cyl;
        Line3D L = new Line3D(C.Axis());
        double R = L.Distance(Point);
        C.SetRadius(R);
        TheCylinder = new Geom_CylindricalSurface(C);
    }

    /// <summary>The constructed cylinder (frozen).</summary>
    /// <exception cref="NotDoneException">No cylinder was constructed.</exception>
    public Geom_CylindricalSurface Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "GC_MakeCylindricalSurface::Value() - no result");
        TheCylinder!.Freeze();
        return TheCylinder;
    }

    /// <summary>The constructed cylinder (OCCT's implicit conversion).</summary>
    public static explicit operator Geom_CylindricalSurface(GC_MakeCylindricalSurface theMaker) => theMaker.Value();
}
