// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeHyperbola2d.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeHyperbola2d.cxx
// Port of: src/ModelingData/TKGeomBase/GCE2d/GCE2d_MakeHyperbola.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Values;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Engine.Construction.Curves;

/// <summary>
/// Constructions of a Geom2d_Hyperbola: from a gp_Hypr2d, an axis (system) and two radii, or two apexes and the
/// center.
/// </summary>
public sealed class MakeHyperbola2d : Root
{
    private HyperbolaCurve2D? _theHyperbola;

    /// <summary>A hyperbola from the gp_Hypr2d H.</summary>
    public MakeHyperbola2d(in Hyperbola2D h)
    {
        _theError = Done;
        _theHyperbola = new HyperbolaCurve2D(h);
    }

    /// <summary>A hyperbola on the major axis MajorAxis; fails with the status of gce_MakeHypr2d.</summary>
    public MakeHyperbola2d(in Axis2D majorAxis, double majorRadius, double minorRadius, bool sense)
    {
        MakeHypr2d h = new MakeHypr2d(majorAxis, majorRadius, minorRadius, sense);
        _theError = h.Status();
        if (_theError == Done)
        {
            _theHyperbola = new HyperbolaCurve2D(h.Value());
        }
    }

    /// <summary>A hyperbola in the axis system Axis; fails with the status of gce_MakeHypr2d.</summary>
    public MakeHyperbola2d(in Frame2D axis, double majorRadius, double minorRadius)
    {
        MakeHypr2d h = new MakeHypr2d(axis, majorRadius, minorRadius);
        _theError = h.Status();
        if (_theError == Done)
        {
            _theHyperbola = new HyperbolaCurve2D(h.Value());
        }
    }

    /// <summary>A hyperbola from two apexes and its center; fails with the status of gce_MakeHypr2d.</summary>
    public MakeHyperbola2d(in Point2D s1, in Point2D s2, in Point2D center)
    {
        MakeHypr2d h = new MakeHypr2d(s1, s2, center);
        _theError = h.Status();
        if (_theError == Done)
        {
            _theHyperbola = new HyperbolaCurve2D(h.Value());
        }
    }

    /// <summary>The constructed hyperbola (frozen).</summary>
    /// <exception cref="NotDoneException">No hyperbola was constructed.</exception>
    public HyperbolaCurve2D Value()
    {
        StdFail_NotDone_Raise_if(_theError != Done, "GC_MakeHyperbola2d::Value() - no result");
        _theHyperbola!.Freeze();
        return _theHyperbola;
    }

    /// <summary>The constructed hyperbola (OCCT's implicit conversion).</summary>
    public static explicit operator HyperbolaCurve2D(MakeHyperbola2d maker) => maker.Value();
}
