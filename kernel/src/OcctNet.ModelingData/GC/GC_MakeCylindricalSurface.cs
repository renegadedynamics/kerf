// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeCylindricalSurface.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeCylindricalSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

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
    public GC_MakeCylindricalSurface(in gp_Cylinder C)
    {
        TheError = gce_Done;
        TheCylinder = new Geom_CylindricalSurface(C);
    }

    /// <summary>A cylinder in the axis system A2; fails with gce_NegativeRadius if Radius is negative.</summary>
    public GC_MakeCylindricalSurface(in gp_Ax2 A2, double Radius)
    {
        if (Radius < 0.0)
        {
            TheError = gce_NegativeRadius;
        }
        else
        {
            TheError = gce_Done;
            TheCylinder = new Geom_CylindricalSurface(new gp_Ax3(A2), Radius);
        }
    }

    /// <summary>A cylinder of axis A1; fails with the status of gce_MakeCylinder.</summary>
    public GC_MakeCylindricalSurface(in gp_Ax1 A1, double Radius)
    {
        gce_MakeCylinder Cyl = new gce_MakeCylinder(A1, Radius);
        TheError = Cyl.Status();
        if (TheError == gce_Done)
        {
            TheCylinder = new Geom_CylindricalSurface(Cyl.Value());
        }
    }

    /// <summary>The cylinder whose axis and radius are those of Circ.</summary>
    public GC_MakeCylindricalSurface(in gp_Circ Circ)
    {
        // gce_MakeCylinder(Circ) always succeeds, so its implicit conversion can't throw.
        gp_Cylinder Cyl = new gce_MakeCylinder(Circ).Value();
        TheCylinder = new Geom_CylindricalSurface(Cyl);
        TheError = gce_Done;
    }

    /// <summary>A cylinder with axis P1P2 through P3; fails with the status of gce_MakeCylinder.</summary>
    public GC_MakeCylindricalSurface(in gp_Pnt P1, in gp_Pnt P2, in gp_Pnt P3)
    {
        gce_MakeCylinder Cyl = new gce_MakeCylinder(P1, P2, P3);
        TheError = Cyl.Status();
        if (TheError == gce_Done)
        {
            TheCylinder = new Geom_CylindricalSurface(Cyl.Value());
        }
    }

    /// <summary>A cylinder coaxial to Cyl with radius |Cyl.Radius() − Dist|.</summary>
    public GC_MakeCylindricalSurface(in gp_Cylinder Cyl, double Dist)
    {
        TheError = gce_Done;
        double R = Math.Abs(Cyl.Radius() - Dist);
        TheCylinder = new Geom_CylindricalSurface(Cyl);
        TheCylinder.SetRadius(R);
    }

    /// <summary>A cylinder coaxial to Cyl through Point.</summary>
    public GC_MakeCylindricalSurface(in gp_Cylinder Cyl, in gp_Pnt Point)
    {
        TheError = gce_Done;
        gp_Cylinder C = Cyl;
        gp_Lin L = new gp_Lin(C.Axis());
        double R = L.Distance(Point);
        C.SetRadius(R);
        TheCylinder = new Geom_CylindricalSurface(C);
    }

    /// <summary>The constructed cylinder (frozen).</summary>
    /// <exception cref="StdFail_NotDone">No cylinder was constructed.</exception>
    public Geom_CylindricalSurface Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "GC_MakeCylindricalSurface::Value() - no result");
        TheCylinder!.Freeze();
        return TheCylinder;
    }

    /// <summary>The constructed cylinder (OCCT's implicit conversion).</summary>
    public static explicit operator Geom_CylindricalSurface(GC_MakeCylindricalSurface theMaker) => theMaker.Value();
}
