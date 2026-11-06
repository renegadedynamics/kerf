// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeArcOfParabola2d.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeArcOfParabola2d.cxx
// Port of: src/ModelingData/TKGeomBase/GCE2d/GCE2d_MakeArcOfParabola.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Engine.Construction.Curves;

/// <summary>Constructions of an arc of parabola in 2D as a Geom2d_TrimmedCurve.</summary>
internal sealed class MakeArcOfParabola2d : Root
{
    private TrimmedCurve2D? _theArc;

    /// <summary>The arc of parabola in 2D between the parameters of P1 and P2, in the conic's sense if Sense is true.</summary>
    public MakeArcOfParabola2d(in Parabola2D parab, in Point2D p1, in Point2D p2, bool sense = true)
    {
        double alpha1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(parab, p1);
        double alpha2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(parab, p2);
        ParabolaCurve2D c = new ParabolaCurve2D(parab);
        _theArc = new TrimmedCurve2D(c, alpha1, alpha2, sense);
        _theError = Done;
    }

    /// <summary>The arc of parabola in 2D from the parameter of P to the parameter Alpha.</summary>
    public MakeArcOfParabola2d(in Parabola2D parab, in Point2D p, double alpha, bool sense = true)
    {
        double alphafirst = global::Kerf.Engine.Geometry.ElCLib.Parameter(parab, p);
        ParabolaCurve2D c = new ParabolaCurve2D(parab);
        _theArc = new TrimmedCurve2D(c, alphafirst, alpha, sense);
        _theError = Done;
    }

    /// <summary>The arc of parabola in 2D between the parameters Alpha1 and Alpha2.</summary>
    public MakeArcOfParabola2d(in Parabola2D parab, double alpha1, double alpha2, bool sense = true)
    {
        ParabolaCurve2D c = new ParabolaCurve2D(parab);
        _theArc = new TrimmedCurve2D(c, alpha1, alpha2, sense);
        _theError = Done;
    }

    /// <summary>The constructed arc (frozen).</summary>
    /// <exception cref="NotDoneException">No arc was constructed.</exception>
    public TrimmedCurve2D Value()
    {
        StdFail_NotDone_Raise_if(_theError != Done, "GC_MakeArcOfParabola2d::Value() - no result");
        _theArc!.Freeze();
        return _theArc;
    }

    /// <summary>The constructed arc (OCCT's implicit conversion).</summary>
    public static explicit operator TrimmedCurve2D(MakeArcOfParabola2d maker) => maker.Value();
}
