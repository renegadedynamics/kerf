// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeArcOfCircle2d.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeArcOfCircle2d.cxx
// Port of: src/ModelingData/TKGeomBase/GCE2d/GCE2d_MakeArcOfCircle.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>
/// Constructions of an arc of circle in 2D as a Geom2d_TrimmedCurve: through three points, from two points and the
/// tangent at the first, or on a circle between two points or parameters.
/// </summary>
public sealed class GC_MakeArcOfCircle2d : GC_Root
{
    private Geom2d_TrimmedCurve? TheArc;

    /// <summary>The arc through P1, P2 and P3, from P1 to P3; fails with the status of gce_MakeCirc2d.</summary>
    public GC_MakeArcOfCircle2d(in Point2D P1, in Point2D P2, in Point2D P3)
    {
        gce_MakeCirc2d Cir = new gce_MakeCirc2d(P1, P2, P3);
        TheError = Cir.Status();
        if (TheError == gce_Done)
        {
            Circle2D C = Cir.Value();
            double Alpha1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(C, P1);
            double Alpha2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(C, P3);
            Geom2d_Circle Circ = new Geom2d_Circle(C);
            TheArc = new Geom2d_TrimmedCurve(Circ, Alpha1, Alpha2, true);
        }
    }

    /// <summary>
    /// The arc from P1 to P2 tangent to V at P1, in the sense of V; fails with gce_ConfusedPoints if the lines
    /// defining the center don't intersect.
    /// </summary>
    public GC_MakeArcOfCircle2d(in Point2D P1, in Vector2D V, in Point2D P2)
    {
        bool Sense;
        Circle2D cir = new();
        // OCCT converts gce_MakeLin2d implicitly; when P1 and P2 coincide its release build returns the default line
        // (the X axis) and goes on. The port reads that default line too, rather than throwing StdFail_NotDone. The
        // center is then P1 itself, and ElCLib::Parameter throws below (Standard_ConstructionError from gp_Dir2d),
        // where OCCT's release build computes NaN parameters.
        gce_MakeLin2d aCordeMaker = new gce_MakeLin2d(P1, P2);
        Line2D corde = aCordeMaker.IsDone() ? aCordeMaker.Value() : new Line2D();
        Direction2D dir = corde.Direction();
        Line2D bis = new Line2D(new Point2D((P1.X() + P2.X()) / 2.0, (P1.Y() + P2.Y()) / 2.0),
                                    new Direction2D(-dir.Y(), dir.X()));
        Line2D norm = new Line2D(P1, new Direction2D(-V.Y(), V.X()));
        TheError = gce_ConfusedPoints;
        IntAna2d_AnaIntersection Intp = new IntAna2d_AnaIntersection(bis, norm);
        if (Intp.IsDone())
        {
            if (!Intp.IsEmpty())
            {
                Point2D center = Intp.Point(1).Value();
                double rad = (center.Distance(P1) + center.Distance(P2)) / 2.0;
                // gce_MakeCirc2d(center, rad) can't fail: rad is not negative.
                cir = new gce_MakeCirc2d(center, rad).Value();
                TheError = gce_Done;
            }
        }
        if (TheError == gce_Done)
        {
            double Alpha1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(cir, P1);
            double Alpha2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(cir, P2);
            Geom2d_Circle Circ = new Geom2d_Circle(cir);
            Vector2D vv = new Vector2D(dir);
            double cross = V ^ vv;
            Sense = cross > 0.0;
            TheArc = new Geom2d_TrimmedCurve(Circ, Alpha1, Alpha2, Sense);
        }
    }

    /// <summary>The arc of Circ between the parameters of P1 and P2, in the circle's sense if Sense is true.</summary>
    public GC_MakeArcOfCircle2d(in Circle2D Circ, in Point2D P1, in Point2D P2, bool Sense = true)
    {
        double Alpha1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(Circ, P1);
        double Alpha2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(Circ, P2);
        Geom2d_Circle C = new Geom2d_Circle(Circ);
        TheArc = new Geom2d_TrimmedCurve(C, Alpha1, Alpha2, Sense);
        TheError = gce_Done;
    }

    /// <summary>The arc of Circ from the parameter of P to the parameter Alpha.</summary>
    public GC_MakeArcOfCircle2d(in Circle2D Circ, in Point2D P, double Alpha, bool Sense = true)
    {
        double Alphafirst = global::Kerf.Engine.Geometry.ElCLib.Parameter(Circ, P);
        Geom2d_Circle C = new Geom2d_Circle(Circ);
        TheArc = new Geom2d_TrimmedCurve(C, Alphafirst, Alpha, Sense);
        TheError = gce_Done;
    }

    /// <summary>The arc of Circ between the parameters Alpha1 and Alpha2.</summary>
    public GC_MakeArcOfCircle2d(in Circle2D Circ, double Alpha1, double Alpha2, bool Sense = true)
    {
        Geom2d_Circle C = new Geom2d_Circle(Circ);
        TheArc = new Geom2d_TrimmedCurve(C, Alpha1, Alpha2, Sense);
        TheError = gce_Done;
    }

    /// <summary>The constructed arc (frozen).</summary>
    /// <exception cref="NotDoneException">No arc was constructed.</exception>
    public Geom2d_TrimmedCurve Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "GC_MakeArcOfCircle2d::Value() - no result");
        TheArc!.Freeze();
        return TheArc;
    }

    /// <summary>The constructed arc (OCCT's implicit conversion).</summary>
    public static explicit operator Geom2d_TrimmedCurve(GC_MakeArcOfCircle2d theMaker) => theMaker.Value();
}
