// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeArcOfCircle2d.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeArcOfCircle2d.cxx
// Port of: src/ModelingData/TKGeomBase/GCE2d/GCE2d_MakeArcOfCircle.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Values;
using Kerf.Engine.Foundation;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Engine.Construction.Curves;

/// <summary>
/// Constructions of an arc of circle in 2D as a Geom2d_TrimmedCurve: through three points, from two points and the
/// tangent at the first, or on a circle between two points or parameters.
/// </summary>
internal sealed class MakeArcOfCircle2d : Root
{
    private TrimmedCurve2D? _theArc;

    /// <summary>The arc through P1, P2 and P3, from P1 to P3; fails with the status of gce_MakeCirc2d.</summary>
    public MakeArcOfCircle2d(in Point2D p1, in Point2D p2, in Point2D p3)
    {
        MakeCirc2d cir = new MakeCirc2d(p1, p2, p3);
        _theError = cir.Status();
        if (_theError == Done)
        {
            Circle2D c = cir.Value();
            double alpha1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(c, p1);
            double alpha2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(c, p3);
            CircleCurve2D circ = new CircleCurve2D(c);
            _theArc = new TrimmedCurve2D(circ, alpha1, alpha2, true);
        }
    }

    /// <summary>
    /// The arc from P1 to P2 tangent to V at P1, in the sense of V; fails with gce_ConfusedPoints if the lines
    /// defining the center don't intersect.
    /// </summary>
    public MakeArcOfCircle2d(in Point2D p1, in Vector2D v, in Point2D p2)
    {
        bool sense;
        Circle2D cir = new();
        // OCCT converts gce_MakeLin2d implicitly; when P1 and P2 coincide its release build returns the default line
        // (the X axis) and goes on. The port reads that default line too, rather than throwing StdFail_NotDone. The
        // center is then P1 itself, and ElCLib::Parameter throws below (Standard_ConstructionError from gp_Dir2d),
        // where OCCT's release build computes NaN parameters.
        MakeLin2d cordeMaker = new MakeLin2d(p1, p2);
        Line2D corde = cordeMaker.IsDone ? cordeMaker.Value() : new Line2D();
        Direction2D dir = corde.Direction;
        Line2D bis = new Line2D(new Point2D((p1.X + p2.X) / 2.0, (p1.Y + p2.Y) / 2.0),
                                    new Direction2D(-dir.Y, dir.X));
        Line2D norm = new Line2D(p1, new Direction2D(-v.Y, v.X));
        _theError = ConfusedPoints;
        AnalyticIntersection intp = new AnalyticIntersection(bis, norm);
        if (intp.IsDone)
        {
            if (!intp.IsEmpty)
            {
                Point2D center = intp.Point(1).Value();
                double rad = (center.DistanceTo(p1) + center.DistanceTo(p2)) / 2.0;
                // gce_MakeCirc2d(center, rad) can't fail: rad is not negative.
                cir = new MakeCirc2d(center, rad).Value();
                _theError = Done;
            }
        }
        if (_theError == Done)
        {
            double alpha1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(cir, p1);
            double alpha2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(cir, p2);
            CircleCurve2D circ = new CircleCurve2D(cir);
            Vector2D vv = new Vector2D(dir);
            double cross = v ^ vv;
            sense = cross > 0.0;
            _theArc = new TrimmedCurve2D(circ, alpha1, alpha2, sense);
        }
    }

    /// <summary>The arc of Circ between the parameters of P1 and P2, in the circle's sense if Sense is true.</summary>
    public MakeArcOfCircle2d(in Circle2D circ, in Point2D p1, in Point2D p2, bool sense = true)
    {
        double alpha1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(circ, p1);
        double alpha2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(circ, p2);
        CircleCurve2D c = new CircleCurve2D(circ);
        _theArc = new TrimmedCurve2D(c, alpha1, alpha2, sense);
        _theError = Done;
    }

    /// <summary>The arc of Circ from the parameter of P to the parameter Alpha.</summary>
    public MakeArcOfCircle2d(in Circle2D circ, in Point2D p, double alpha, bool sense = true)
    {
        double alphafirst = global::Kerf.Engine.Geometry.ElCLib.Parameter(circ, p);
        CircleCurve2D c = new CircleCurve2D(circ);
        _theArc = new TrimmedCurve2D(c, alphafirst, alpha, sense);
        _theError = Done;
    }

    /// <summary>The arc of Circ between the parameters Alpha1 and Alpha2.</summary>
    public MakeArcOfCircle2d(in Circle2D circ, double alpha1, double alpha2, bool sense = true)
    {
        CircleCurve2D c = new CircleCurve2D(circ);
        _theArc = new TrimmedCurve2D(c, alpha1, alpha2, sense);
        _theError = Done;
    }

    /// <summary>The constructed arc (frozen).</summary>
    /// <exception cref="NotDoneException">No arc was constructed.</exception>
    public TrimmedCurve2D Value()
    {
        StdFail_NotDone_Raise_if(_theError != Done, "GC_MakeArcOfCircle2d::Value() - no result");
        _theArc!.Freeze();
        return _theArc;
    }

    /// <summary>The constructed arc (OCCT's implicit conversion).</summary>
    public static explicit operator TrimmedCurve2D(MakeArcOfCircle2d maker) => maker.Value();
}
