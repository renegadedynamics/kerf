// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeTrimmedCylinder.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeTrimmedCylinder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Engine.Construction.Curves;

/// <summary>
/// Constructions of a trimmed cylinder as a Geom_RectangularTrimmedSurface: from three points, a circle and a
/// height, or an axis, a radius and a height.
/// </summary>
internal sealed class MakeTrimmedCylinder : Root
{
    private TrimmedSurface? _theCyl;

    /// <summary>
    /// The cylinder with axis P1P2 through P3, of height P1P2; fails with the status of GC_MakeCylindricalSurface.
    /// </summary>
    public MakeTrimmedCylinder(in Point3D p1, in Point3D p2, in Point3D p3)
    {
        MakeCylindricalSurface cyl = new MakeCylindricalSurface(p1, p2, p3);
        _theError = cyl.Status();
        if (_theError == Done)
        {
            _theCyl = new TrimmedSurface(cyl.Value(), 0.0, 2.0 * M_PI, 0.0, p2.DistanceTo(p1), true,
                                                        true);
        }
    }

    /// <summary>The cylinder on the circle Circ, of height Height.</summary>
    public MakeTrimmedCylinder(in Circle3D circ, double height)
    {
        MakeCylindricalSurface cyl = new MakeCylindricalSurface(circ);
        _theError = cyl.Status();
        if (_theError == Done)
        {
            _theCyl = new TrimmedSurface(cyl.Value(), 0.0, 2.0 * M_PI, 0.0, height, true, true);
        }
    }

    /// <summary>The cylinder of axis A1, radius Radius and height Height; fails with the status of
    /// GC_MakeCylindricalSurface.</summary>
    public MakeTrimmedCylinder(in Axis3D a1, double radius, double height)
    {
        MakeCylindricalSurface cyl = new MakeCylindricalSurface(a1, radius);
        _theError = cyl.Status();
        if (_theError == Done)
        {
            _theCyl = new TrimmedSurface(cyl.Value(), 0.0, 2.0 * M_PI, 0.0, height, true, true);
        }
    }

    /// <summary>The constructed trimmed cylinder (frozen).</summary>
    /// <exception cref="NotDoneException">No cylinder was constructed.</exception>
    public TrimmedSurface Value()
    {
        StdFail_NotDone_Raise_if(_theError != Done, "GC_MakeTrimmedCylinder::Value() - no result");
        _theCyl!.Freeze();
        return _theCyl;
    }

    /// <summary>The constructed trimmed cylinder (OCCT's implicit conversion).</summary>
    public static explicit operator TrimmedSurface(MakeTrimmedCylinder maker) =>
        maker.Value();
}
