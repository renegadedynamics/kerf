// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeTrimmedCone.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeTrimmedCone.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Engine.Construction.Curves;

/// <summary>
/// Constructions of a trimmed cone as a Geom_RectangularTrimmedSurface: from four points, or two points and two radii.
/// </summary>
internal sealed class MakeTrimmedCone : Root
{
    private TrimmedSurface? _theCone;

    /// <summary>
    /// The cone with axis P1P2 through P3 and P4, trimmed between the sections through P3 and P4; fails with the
    /// status of GC_MakeConicalSurface.
    /// </summary>
    public MakeTrimmedCone(in Point3D p1, in Point3D p2, in Point3D p3, in Point3D p4)
    {
        MakeConicalSurface cone = new MakeConicalSurface(p1, p2, p3, p4);
        _theError = cone.Status();
        if (_theError == Done)
        {
            Direction3D d1 = new Direction3D(p2.XYZ() - p1.XYZ());
            Line3D l1 = new Line3D(p1, d1);
            ExtPElC ext1 = new ExtPElC(p3, l1, 1.0e-7, -2.0e+100, +2.0e+100);
            ExtPElC ext2 = new ExtPElC(p4, l1, 1.0e-7, -2.0e+100, +2.0e+100);
            Point3D p5 = ext1.Point(1).Value();
            Point3D p6 = ext2.Point(1).Value();
            double d = p6.DistanceTo(p5) / Math.Cos(cone.Value().SemiAngle);
            _theCone = new TrimmedSurface(cone.Value(), 0.0, 2.0 * M_PI, 0.0, d, true, true);
        }
    }

    /// <summary>
    /// The cone with axis P1P2 and radii R1 at P1 and R2 at P2, trimmed between them; fails with the status of
    /// GC_MakeConicalSurface.
    /// </summary>
    public MakeTrimmedCone(in Point3D p1, in Point3D p2, double r1, double r2)
    {
        MakeConicalSurface cone = new MakeConicalSurface(p1, p2, r1, r2);
        _theError = cone.Status();
        if (_theError == Done)
        {
            double d = p2.DistanceTo(p1) / Math.Cos(cone.Value().SemiAngle);
            _theCone = new TrimmedSurface(cone.Value(), 0.0, 2.0 * M_PI, 0.0, d, true, true);
        }
    }

    /// <summary>The constructed trimmed cone (frozen).</summary>
    /// <exception cref="NotDoneException">No cone was constructed.</exception>
    public TrimmedSurface Value()
    {
        StdFail_NotDone_Raise_if(_theError != Done, "GC_MakeTrimmedCone::Value() - no result");
        _theCone!.Freeze();
        return _theCone;
    }

    /// <summary>The constructed trimmed cone (OCCT's implicit conversion).</summary>
    public static explicit operator TrimmedSurface(MakeTrimmedCone maker) => maker.Value();
}
