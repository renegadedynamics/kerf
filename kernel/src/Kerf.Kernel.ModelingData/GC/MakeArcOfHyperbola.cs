// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeArcOfHyperbola.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeArcOfHyperbola.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Engine.Construction.Curves;

/// <summary>Constructions of an arc of hyperbola as a Geom_TrimmedCurve.</summary>
internal sealed class MakeArcOfHyperbola : Root
{
    private TrimmedCurve3D? _theArc;

    /// <summary>The arc of hyperbola between the parameters of P1 and P2, in the conic's sense if Sense is true.</summary>
    public MakeArcOfHyperbola(in Hyperbola3D hypr, in Point3D p1, in Point3D p2, bool sense)
    {
        double alpha1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(hypr, p1);
        double alpha2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(hypr, p2);
        HyperbolaCurve3D c = new HyperbolaCurve3D(hypr);
        _theArc = new TrimmedCurve3D(c, alpha1, alpha2, sense);
        _theError = Done;
    }

    /// <summary>The arc of hyperbola from the parameter of P to the parameter Alpha.</summary>
    public MakeArcOfHyperbola(in Hyperbola3D hypr, in Point3D p, double alpha, bool sense)
    {
        double alphafirst = global::Kerf.Engine.Geometry.ElCLib.Parameter(hypr, p);
        HyperbolaCurve3D c = new HyperbolaCurve3D(hypr);
        _theArc = new TrimmedCurve3D(c, alphafirst, alpha, sense);
        _theError = Done;
    }

    /// <summary>The arc of hyperbola between the parameters Alpha1 and Alpha2.</summary>
    public MakeArcOfHyperbola(in Hyperbola3D hypr, double alpha1, double alpha2, bool sense)
    {
        HyperbolaCurve3D c = new HyperbolaCurve3D(hypr);
        _theArc = new TrimmedCurve3D(c, alpha1, alpha2, sense);
        _theError = Done;
    }

    /// <summary>The constructed arc (frozen).</summary>
    /// <exception cref="NotDoneException">No arc was constructed.</exception>
    public TrimmedCurve3D Value()
    {
        StdFail_NotDone_Raise_if(_theError != Done, "GC_MakeArcOfHyperbola::Value() - no result");
        _theArc!.Freeze();
        return _theArc;
    }

    /// <summary>The constructed arc (OCCT's implicit conversion).</summary>
    public static explicit operator TrimmedCurve3D(MakeArcOfHyperbola maker) => maker.Value();
}
