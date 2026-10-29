// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeTrimmedCylinder.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeTrimmedCylinder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>
/// Constructions of a trimmed cylinder as a Geom_RectangularTrimmedSurface: from three points, a circle and a
/// height, or an axis, a radius and a height.
/// </summary>
public sealed class GC_MakeTrimmedCylinder : GC_Root
{
    private Geom_RectangularTrimmedSurface? TheCyl;

    /// <summary>
    /// The cylinder with axis P1P2 through P3, of height P1P2; fails with the status of GC_MakeCylindricalSurface.
    /// </summary>
    public GC_MakeTrimmedCylinder(in gp_Pnt P1, in gp_Pnt P2, in gp_Pnt P3)
    {
        GC_MakeCylindricalSurface Cyl = new GC_MakeCylindricalSurface(P1, P2, P3);
        TheError = Cyl.Status();
        if (TheError == gce_Done)
        {
            TheCyl = new Geom_RectangularTrimmedSurface(Cyl.Value(), 0.0, 2.0 * M_PI, 0.0, P2.Distance(P1), true,
                                                        true);
        }
    }

    /// <summary>The cylinder on the circle Circ, of height Height.</summary>
    public GC_MakeTrimmedCylinder(in gp_Circ Circ, double Height)
    {
        GC_MakeCylindricalSurface Cyl = new GC_MakeCylindricalSurface(Circ);
        TheError = Cyl.Status();
        if (TheError == gce_Done)
        {
            TheCyl = new Geom_RectangularTrimmedSurface(Cyl.Value(), 0.0, 2.0 * M_PI, 0.0, Height, true, true);
        }
    }

    /// <summary>The cylinder of axis A1, radius Radius and height Height; fails with the status of
    /// GC_MakeCylindricalSurface.</summary>
    public GC_MakeTrimmedCylinder(in gp_Ax1 A1, double Radius, double Height)
    {
        GC_MakeCylindricalSurface Cyl = new GC_MakeCylindricalSurface(A1, Radius);
        TheError = Cyl.Status();
        if (TheError == gce_Done)
        {
            TheCyl = new Geom_RectangularTrimmedSurface(Cyl.Value(), 0.0, 2.0 * M_PI, 0.0, Height, true, true);
        }
    }

    /// <summary>The constructed trimmed cylinder (frozen).</summary>
    /// <exception cref="StdFail_NotDone">No cylinder was constructed.</exception>
    public Geom_RectangularTrimmedSurface Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "GC_MakeTrimmedCylinder::Value() - no result");
        TheCyl!.Freeze();
        return TheCyl;
    }

    /// <summary>The constructed trimmed cylinder (OCCT's implicit conversion).</summary>
    public static explicit operator Geom_RectangularTrimmedSurface(GC_MakeTrimmedCylinder theMaker) =>
        theMaker.Value();
}
