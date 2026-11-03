// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeArcOfHyperbola2d.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeArcOfHyperbola2d.cxx
// Port of: src/ModelingData/TKGeomBase/GCE2d/GCE2d_MakeArcOfHyperbola.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Engine.Construction.Curves;

/// <summary>Constructions of an arc of hyperbola in 2D as a Geom2d_TrimmedCurve.</summary>
public sealed class MakeArcOfHyperbola2d : Root
{
    private TrimmedCurve2D? _theArc;

    /// <summary>The arc of hyperbola in 2D between the parameters of P1 and P2, in the conic's sense if Sense is true.</summary>
    public MakeArcOfHyperbola2d(in Hyperbola2D hypr, in Point2D p1, in Point2D p2, bool sense = true)
    {
        double alpha1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(hypr, p1);
        double alpha2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(hypr, p2);
        HyperbolaCurve2D c = new HyperbolaCurve2D(hypr);
        _theArc = new TrimmedCurve2D(c, alpha1, alpha2, sense);
        _theError = Done;
    }

    /// <summary>The arc of hyperbola in 2D from the parameter of P to the parameter Alpha.</summary>
    public MakeArcOfHyperbola2d(in Hyperbola2D hypr, in Point2D p, double alpha, bool sense = true)
    {
        double alphafirst = global::Kerf.Engine.Geometry.ElCLib.Parameter(hypr, p);
        HyperbolaCurve2D c = new HyperbolaCurve2D(hypr);
        _theArc = new TrimmedCurve2D(c, alphafirst, alpha, sense);
        _theError = Done;
    }

    /// <summary>The arc of hyperbola in 2D between the parameters Alpha1 and Alpha2.</summary>
    public MakeArcOfHyperbola2d(in Hyperbola2D hypr, double alpha1, double alpha2, bool sense = true)
    {
        HyperbolaCurve2D c = new HyperbolaCurve2D(hypr);
        _theArc = new TrimmedCurve2D(c, alpha1, alpha2, sense);
        _theError = Done;
    }

    /// <summary>The constructed arc (frozen).</summary>
    /// <exception cref="NotDoneException">No arc was constructed.</exception>
    public TrimmedCurve2D Value()
    {
        StdFail_NotDone_Raise_if(_theError != Done, "GC_MakeArcOfHyperbola2d::Value() - no result");
        _theArc!.Freeze();
        return _theArc;
    }

    /// <summary>The constructed arc (OCCT's implicit conversion).</summary>
    public static explicit operator TrimmedCurve2D(MakeArcOfHyperbola2d maker) => maker.Value();
}
