// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeTrimmedCone.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeTrimmedCone.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>
/// Constructions of a trimmed cone as a Geom_RectangularTrimmedSurface: from four points, or two points and two radii.
/// </summary>
public sealed class GC_MakeTrimmedCone : GC_Root
{
    private Geom_RectangularTrimmedSurface? TheCone;

    /// <summary>
    /// The cone with axis P1P2 through P3 and P4, trimmed between the sections through P3 and P4; fails with the
    /// status of GC_MakeConicalSurface.
    /// </summary>
    public GC_MakeTrimmedCone(in Point3D P1, in Point3D P2, in Point3D P3, in Point3D P4)
    {
        GC_MakeConicalSurface Cone = new GC_MakeConicalSurface(P1, P2, P3, P4);
        TheError = Cone.Status();
        if (TheError == gce_Done)
        {
            Direction3D D1 = new Direction3D(P2.XYZ() - P1.XYZ());
            Line3D L1 = new Line3D(P1, D1);
            Extrema_ExtPElC ext1 = new Extrema_ExtPElC(P3, L1, 1.0e-7, -2.0e+100, +2.0e+100);
            Extrema_ExtPElC ext2 = new Extrema_ExtPElC(P4, L1, 1.0e-7, -2.0e+100, +2.0e+100);
            Point3D P5 = ext1.Point(1).Value();
            Point3D P6 = ext2.Point(1).Value();
            double D = P6.Distance(P5) / Math.Cos(Cone.Value().SemiAngle());
            TheCone = new Geom_RectangularTrimmedSurface(Cone.Value(), 0.0, 2.0 * M_PI, 0.0, D, true, true);
        }
    }

    /// <summary>
    /// The cone with axis P1P2 and radii R1 at P1 and R2 at P2, trimmed between them; fails with the status of
    /// GC_MakeConicalSurface.
    /// </summary>
    public GC_MakeTrimmedCone(in Point3D P1, in Point3D P2, double R1, double R2)
    {
        GC_MakeConicalSurface Cone = new GC_MakeConicalSurface(P1, P2, R1, R2);
        TheError = Cone.Status();
        if (TheError == gce_Done)
        {
            double D = P2.Distance(P1) / Math.Cos(Cone.Value().SemiAngle());
            TheCone = new Geom_RectangularTrimmedSurface(Cone.Value(), 0.0, 2.0 * M_PI, 0.0, D, true, true);
        }
    }

    /// <summary>The constructed trimmed cone (frozen).</summary>
    /// <exception cref="NotDoneException">No cone was constructed.</exception>
    public Geom_RectangularTrimmedSurface Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "GC_MakeTrimmedCone::Value() - no result");
        TheCone!.Freeze();
        return TheCone;
    }

    /// <summary>The constructed trimmed cone (OCCT's implicit conversion).</summary>
    public static explicit operator Geom_RectangularTrimmedSurface(GC_MakeTrimmedCone theMaker) => theMaker.Value();
}
