// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeHyperbola.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeHyperbola.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Values;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Engine.Construction.Curves;

/// <summary>
/// Constructions of a Geom_Hyperbola: from a gp_Hypr, an axis system and two radii, or two apexes and the center.
/// </summary>
internal sealed class MakeHyperbola : Root
{
    private HyperbolaCurve3D? _theHyperbola;

    /// <summary>A hyperbola from the gp_Hypr H.</summary>
    public MakeHyperbola(in Hyperbola3D h)
    {
        _theError = Done;
        _theHyperbola = new HyperbolaCurve3D(h);
    }

    /// <summary>A hyperbola in the axis system A2; fails with gce_NegativeRadius if a radius is negative.</summary>
    public MakeHyperbola(in Frame3D a2, double majorRadius, double minorRadius)
    {
        if (majorRadius < 0.0 || minorRadius < 0.0)
        {
            _theError = NegativeRadius;
        }
        else
        {
            _theError = Done;
            _theHyperbola = new HyperbolaCurve3D(new Hyperbola3D(a2, majorRadius, minorRadius));
        }
    }

    /// <summary>A hyperbola from two apexes and its center; fails with the status of gce_MakeHypr.</summary>
    public MakeHyperbola(in Point3D s1, in Point3D s2, in Point3D center)
    {
        MakeHypr h = new MakeHypr(s1, s2, center);
        _theError = h.Status();
        if (_theError == Done)
        {
            _theHyperbola = new HyperbolaCurve3D(h.Value());
        }
    }

    /// <summary>The constructed hyperbola (frozen).</summary>
    /// <exception cref="NotDoneException">No hyperbola was constructed.</exception>
    public HyperbolaCurve3D Value()
    {
        StdFail_NotDone_Raise_if(_theError != Done, "GC_MakeHyperbola::Value() - no result");
        _theHyperbola!.Freeze();
        return _theHyperbola;
    }

    /// <summary>The constructed hyperbola (OCCT's implicit conversion).</summary>
    public static explicit operator HyperbolaCurve3D(MakeHyperbola maker) => maker.Value();
}
