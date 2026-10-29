// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakeEdge.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakeEdge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BRepLib_EdgeError;
using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>
/// Makes an edge from a curve (gp or Geom) or a curve on a surface, bounded by parameters, points or vertices.
/// </summary>
/// <remarks>
/// <para>The curve is trimmed by the parameters; given points or vertices are projected on it. A missing vertex is
/// made at the end of a finite range; on a closed range one vertex serves both ends. Trimmed curves are replaced by
/// their basis curves.</para>
/// <para><see cref="Edge"/>, <see cref="Vertex1"/> and <see cref="Vertex2"/> freeze the edge (design 4.2); ported
/// code reads <see cref="InternalEdge"/>. The curve is frozen when the edge takes it.</para>
/// </remarks>
public class BRepLib_MakeEdge : BRepLib_MakeShape
{
    private BRepLib_EdgeError myError;
    private TopoDS_Vertex myVertex1 = new();
    private TopoDS_Vertex myVertex2 = new();

    private static bool Project(Geom_Curve C, in TopoDS_Vertex V, out double p)
    {
        p = 0.0;
        double Eps2 = BRep_Tool.Tolerance(V);
        Eps2 *= Eps2;

        gp_Pnt P = BRep_Tool.Pnt(V);
        var GAC = new GeomAdaptor_Curve(C);

        // Afin de faire les extremas, on verifie les distances en bout
        double D1, D2;
        gp_Pnt P1, P2;
        P1 = GAC.Value(GAC.FirstParameter());
        P2 = GAC.Value(GAC.LastParameter());
        D1 = P1.SquareDistance(P);
        D2 = P2.SquareDistance(P);
        if ((D1 < D2) && (D1 <= Eps2))
        {
            p = GAC.FirstParameter();
            return true;
        }
        else if ((D2 < D1) && (D2 <= Eps2))
        {
            p = GAC.LastParameter();
            return true;
        }

        // Sinon, on calcule les extremas.

        var extrema = new Extrema_ExtPC(P, GAC);
        if (extrema.IsDone())
        {
            int i, index = 0, n = extrema.NbExt();
            double Dist2 = RealLast(), dist2min;

            for (i = 1; i <= n; i++)
            {
                dist2min = extrema.SquareDistance(i);
                if (dist2min < Dist2)
                {
                    index = i;
                    Dist2 = dist2min;
                }
            }

            if (index != 0)
            {
                if (Dist2 <= Eps2)
                {
                    p = (extrema.Point(index)).Parameter();
                    return true;
                }
            }
        }
        return false;
    }

    // project a vertex on a curve on surface
    private static bool Project(Geom2d_Curve C, Geom_Surface S, in TopoDS_Vertex V, out double p)
    {
        p = 0.0;
        gp_Pnt P = BRep_Tool.Pnt(V);
        double Eps2 = BRep_Tool.Tolerance(V);
        Eps2 *= Eps2;

        var HG2AHC = new Geom2dAdaptor_Curve(C);
        var HGAHS = new GeomAdaptor_Surface(S);
        var ACOS = new Adaptor3d_CurveOnSurface(HG2AHC, HGAHS);

        double D1, D2;
        gp_Pnt P1, P2;
        P1 = ACOS.Value(ACOS.FirstParameter());
        P2 = ACOS.Value(ACOS.LastParameter());
        D1 = P1.SquareDistance(P);
        D2 = P2.SquareDistance(P);
        if ((D1 < D2) && (D1 <= Eps2))
        {
            p = ACOS.FirstParameter();
            return true;
        }
        else if ((D2 < D1) && (D2 <= Eps2))
        {
            p = ACOS.LastParameter();
            return true;
        }

        var extrema = new Extrema_ExtPC(P, ACOS);

        if (extrema.IsDone())
        {
            int i, index = 0, n = extrema.NbExt();
            double Dist2 = RealLast(), dist2min;

            for (i = 1; i <= n; i++)
            {
                dist2min = extrema.SquareDistance(i);
                if (dist2min < Dist2)
                {
                    index = i;
                    Dist2 = dist2min;
                }
            }

            if (index != 0)
            {
                Extrema_POnCurv POC = extrema.Point(index);
                if (P.SquareDistance(POC.Value()) <= Precision.SquareConfusion())
                {
                    p = POC.Parameter();
                    return true;
                }
            }
        }
        return false;
    }

    public BRepLib_MakeEdge()
    {
        myError = BRepLib_PointProjectionFailed;
    }

    public BRepLib_MakeEdge(in TopoDS_Vertex V1, in TopoDS_Vertex V2)
    {
        gp_Pnt P1 = BRep_Tool.Pnt(V1);
        gp_Pnt P2 = BRep_Tool.Pnt(V2);
        double l = P1.Distance(P2);
        if (l <= gp.Resolution())
        {
            myError = BRepLib_LineThroughIdenticPoints;
            return;
        }
        var L = new gp_Lin(P1, new gp_Dir(new gp_Vec(P1, P2)));
        var GL = new Geom_Line(L);
        Init(GL, V1, V2, 0, l);
    }

    public BRepLib_MakeEdge(in gp_Pnt P1, in gp_Pnt P2)
    {
        double l = P1.Distance(P2);
        if (l <= gp.Resolution())
        {
            myError = BRepLib_LineThroughIdenticPoints;
            return;
        }
        var L = new gp_Lin(P1, new gp_Dir(new gp_Vec(P1, P2)));
        var GL = new Geom_Line(L);
        Init(GL, P1, P2, 0, l);
    }

    public BRepLib_MakeEdge(in gp_Lin L) => Init(new Geom_Line(L));

    public BRepLib_MakeEdge(in gp_Lin L, double p1, double p2) => Init(new Geom_Line(L), p1, p2);

    public BRepLib_MakeEdge(in gp_Lin L, in gp_Pnt P1, in gp_Pnt P2) => Init(new Geom_Line(L), P1, P2);

    public BRepLib_MakeEdge(in gp_Lin L, in TopoDS_Vertex V1, in TopoDS_Vertex V2) => Init(new Geom_Line(L), V1, V2);

    public BRepLib_MakeEdge(in gp_Circ C) => Init(new Geom_Circle(C));

    public BRepLib_MakeEdge(in gp_Circ C, double p1, double p2) => Init(new Geom_Circle(C), p1, p2);

    public BRepLib_MakeEdge(in gp_Circ C, in gp_Pnt P1, in gp_Pnt P2) => Init(new Geom_Circle(C), P1, P2);

    public BRepLib_MakeEdge(in gp_Circ C, in TopoDS_Vertex V1, in TopoDS_Vertex V2) => Init(new Geom_Circle(C), V1, V2);

    public BRepLib_MakeEdge(in gp_Elips E) => Init(new Geom_Ellipse(E));

    public BRepLib_MakeEdge(in gp_Elips E, double p1, double p2) => Init(new Geom_Ellipse(E), p1, p2);

    public BRepLib_MakeEdge(in gp_Elips E, in gp_Pnt P1, in gp_Pnt P2) => Init(new Geom_Ellipse(E), P1, P2);

    public BRepLib_MakeEdge(in gp_Elips E, in TopoDS_Vertex V1, in TopoDS_Vertex V2) => Init(new Geom_Ellipse(E), V1, V2);

    public BRepLib_MakeEdge(in gp_Hypr H) => Init(new Geom_Hyperbola(H));

    public BRepLib_MakeEdge(in gp_Hypr H, double p1, double p2) => Init(new Geom_Hyperbola(H), p1, p2);

    public BRepLib_MakeEdge(in gp_Hypr H, in gp_Pnt P1, in gp_Pnt P2) => Init(new Geom_Hyperbola(H), P1, P2);

    public BRepLib_MakeEdge(in gp_Hypr H, in TopoDS_Vertex V1, in TopoDS_Vertex V2) => Init(new Geom_Hyperbola(H), V1, V2);

    public BRepLib_MakeEdge(in gp_Parab P) => Init(new Geom_Parabola(P));

    public BRepLib_MakeEdge(in gp_Parab P, double p1, double p2) => Init(new Geom_Parabola(P), p1, p2);

    public BRepLib_MakeEdge(in gp_Parab P, in gp_Pnt P1, in gp_Pnt P2) => Init(new Geom_Parabola(P), P1, P2);

    public BRepLib_MakeEdge(in gp_Parab P, in TopoDS_Vertex V1, in TopoDS_Vertex V2) => Init(new Geom_Parabola(P), V1, V2);

    public BRepLib_MakeEdge(Geom_Curve L) => Init(L);

    public BRepLib_MakeEdge(Geom_Curve L, double p1, double p2) => Init(L, p1, p2);

    public BRepLib_MakeEdge(Geom_Curve L, in gp_Pnt P1, in gp_Pnt P2) => Init(L, P1, P2);

    public BRepLib_MakeEdge(Geom_Curve L, in TopoDS_Vertex V1, in TopoDS_Vertex V2) => Init(L, V1, V2);

    public BRepLib_MakeEdge(Geom_Curve L, in gp_Pnt P1, in gp_Pnt P2, double p1, double p2) => Init(L, P1, P2, p1, p2);

    public BRepLib_MakeEdge(Geom_Curve L, in TopoDS_Vertex V1, in TopoDS_Vertex V2, double p1, double p2)
        => Init(L, V1, V2, p1, p2);

    public BRepLib_MakeEdge(Geom2d_Curve L, Geom_Surface S) => Init(L, S);

    public BRepLib_MakeEdge(Geom2d_Curve L, Geom_Surface S, double p1, double p2) => Init(L, S, p1, p2);

    public BRepLib_MakeEdge(Geom2d_Curve L, Geom_Surface S, in gp_Pnt P1, in gp_Pnt P2) => Init(L, S, P1, P2);

    public BRepLib_MakeEdge(Geom2d_Curve L, Geom_Surface S, in TopoDS_Vertex V1, in TopoDS_Vertex V2) => Init(L, S, V1, V2);

    public BRepLib_MakeEdge(Geom2d_Curve L, Geom_Surface S, in gp_Pnt P1, in gp_Pnt P2, double p1, double p2)
        => Init(L, S, P1, P2, p1, p2);

    public BRepLib_MakeEdge(Geom2d_Curve L, Geom_Surface S, in TopoDS_Vertex V1, in TopoDS_Vertex V2, double p1, double p2)
        => Init(L, S, V1, V2, p1, p2);

    public void Init(Geom_Curve C)
    {
        Init(C, C.FirstParameter(), C.LastParameter());
    }

    public void Init(Geom_Curve C, double p1, double p2)
    {
        TopoDS_Vertex V1 = new(), V2 = new();
        Init(C, V1, V2, p1, p2);
    }

    public void Init(Geom_Curve C, in gp_Pnt P1, in gp_Pnt P2)
    {
        double Tol = BRepLib.Precision();

        var B = new BRep_Builder();
        TopoDS_Vertex V1 = new(), V2 = new();
        B.MakeVertex(ref V1, P1, Tol);
        if (P1.Distance(P2) < Tol)
        {
            V2 = V1;
        }
        else
        {
            B.MakeVertex(ref V2, P2, Tol);
        }

        Init(C, V1, V2);
    }

    public void Init(Geom_Curve C, in TopoDS_Vertex V1, in TopoDS_Vertex V2)
    {
        // try projecting the vertices on the curve

        double p1, p2;

        if (V1.IsNull())
        {
            p1 = C.FirstParameter();
        }
        else if (!Project(C, V1, out p1))
        {
            myError = BRepLib_PointProjectionFailed;
            return;
        }
        if (V2.IsNull())
        {
            p2 = C.LastParameter();
        }
        else if (!Project(C, V2, out p2))
        {
            myError = BRepLib_PointProjectionFailed;
            return;
        }

        Init(C, V1, V2, p1, p2);
    }

    public void Init(Geom_Curve C, in gp_Pnt P1, in gp_Pnt P2, double p1, double p2)
    {
        double Tol = BRepLib.Precision();
        var B = new BRep_Builder();

        TopoDS_Vertex V1 = new(), V2 = new();
        B.MakeVertex(ref V1, P1, Tol);
        if (P1.Distance(P2) < Tol)
        {
            V2 = V1;
        }
        else
        {
            B.MakeVertex(ref V2, P2, Tol);
        }

        Init(C, V1, V2, p1, p2);
    }

    /// <summary>Makes the edge on the curve between the parameters, with the given vertices (or new ones).</summary>
    public void Init(Geom_Curve CC, in TopoDS_Vertex VV1, in TopoDS_Vertex VV2, double pp1, double pp2)
    {
        // kill trimmed curves
        Geom_Curve C = CC;
        Geom_TrimmedCurve? CT = C as Geom_TrimmedCurve;
        while (CT is not null)
        {
            C = CT.BasisCurve();
            CT = C as Geom_TrimmedCurve;
        }

        // check parameters
        double p1 = pp1;
        double p2 = pp2;
        double cf = C.FirstParameter();
        double cl = C.LastParameter();
        double epsilon = Precision.PConfusion();
        bool periodic = C.IsPeriodic();
        var aCA = new GeomAdaptor_Curve(C);

        TopoDS_Vertex V1, V2;
        if (periodic)
        {
            // adjust in period
            ElCLib.AdjustPeriodic(cf, cl, epsilon, ref p1, ref p2);
            V1 = VV1;
            V2 = VV2;
        }
        else
        {
            // reordonate
            if (p1 < p2)
            {
                V1 = VV1;
                V2 = VV2;
            }
            else
            {
                V2 = VV1;
                V1 = VV2;
                double x = p1;
                p1 = p2;
                p2 = x;
            }

            // check range
            if ((cf - p1 > epsilon) || (p2 - cl > epsilon))
            {
                myError = BRepLib_ParameterOutOfRange;
                return;
            }

            // check ponctuallity
            if ((p2 - p1) <= gp.Resolution())
            {
                myError = BRepLib_LineThroughIdenticPoints;
                return;
            }
        }

        // compute points on the curve
        bool p1inf = Precision.IsNegativeInfinite(p1);
        bool p2inf = Precision.IsPositiveInfinite(p2);
        gp_Pnt P1 = new(), P2 = new();
        if (!p1inf)
        {
            P1 = aCA.Value(p1);
        }
        if (!p2inf)
        {
            P2 = aCA.Value(p2);
        }

        double preci = BRepLib.Precision();
        var B = new BRep_Builder();

        // check for closed curve
        bool closed = false;
        bool degenerated = false;
        if (!p1inf && !p2inf)
        {
            closed = (P1.Distance(P2) <= preci);
        }

        // check if the vertices are on the curve
        if (closed)
        {
            if (V1.IsNull() && V2.IsNull())
            {
                B.MakeVertex(ref V1, P1, preci);
                V2 = V1;
            }
            else if (V1.IsNull())
            {
                V1 = V2;
            }
            else if (V2.IsNull())
            {
                V2 = V1;
            }
            else
            {
                if (!V1.IsSame(V2))
                {
                    myError = BRepLib_DifferentPointsOnClosedCurve;
                    return;
                }
                else if (P1.Distance(BRep_Tool.Pnt(V1)) > Max(preci, BRep_Tool.Tolerance(V1)))
                {
                    myError = BRepLib_DifferentPointsOnClosedCurve;
                    return;
                }
                else
                {
                    gp_Pnt PM = aCA.Value((p1 + p2) / 2);
                    if (P1.Distance(PM) < preci)
                    {
                        degenerated = true;
                    }
                }
            }
        }

        else
        { // not closed

            if (p1inf)
            {
                if (!V1.IsNull())
                {
                    myError = BRepLib_PointWithInfiniteParameter;
                    return;
                }
            }
            else
            {
                if (V1.IsNull())
                {
                    B.MakeVertex(ref V1, P1, preci);
                }
                else if (P1.Distance(BRep_Tool.Pnt(V1)) > Max(preci, BRep_Tool.Tolerance(V1)))
                {
                    myError = BRepLib_DifferentsPointAndParameter;
                    return;
                }
            }

            if (p2inf)
            {
                if (!V2.IsNull())
                {
                    myError = BRepLib_PointWithInfiniteParameter;
                    return;
                }
            }
            else
            {
                if (V2.IsNull())
                {
                    B.MakeVertex(ref V2, P2, preci);
                }
                else if (P2.Distance(BRep_Tool.Pnt(V2)) > Max(preci, BRep_Tool.Tolerance(V2)))
                {
                    myError = BRepLib_DifferentsPointAndParameter;
                    return;
                }
            }
        }

        V1.Orientation(TopAbs_FORWARD);
        V2.Orientation(TopAbs_REVERSED);
        myVertex1 = V1;
        myVertex2 = V2;

        // OCCT.NET: OCCT remakes the edge in myShape (TopoDS::Edge(myShape)); the port makes it in a new variable,
        // so that Init after a public (freezing) read of the edge doesn't throw TopoDS_LockedShape. The result is
        // the same: MakeEdge gives a new TShape with the identity location, FORWARD.
        TopoDS_Edge E = new();
        B.MakeEdge(ref E, C, preci);
        if (!V1.IsNull())
        {
            B.Add(E, V1);
        }
        if (!V2.IsNull())
        {
            B.Add(E, V2);
        }
        B.Range(E, p1, p2);
        B.Degenerated(E, degenerated);
        E.Closed(closed);
        myShape = E;

        myError = BRepLib_EdgeDone;
        Done();
    }

    public void Init(Geom2d_Curve C, Geom_Surface S)
    {
        Init(C, S, C.FirstParameter(), C.LastParameter());
    }

    public void Init(Geom2d_Curve C, Geom_Surface S, double p1, double p2)
    {
        TopoDS_Vertex V1 = new(), V2 = new();
        Init(C, S, V1, V2, p1, p2);
    }

    public void Init(Geom2d_Curve C, Geom_Surface S, in gp_Pnt P1, in gp_Pnt P2)
    {
        double Tol = BRepLib.Precision();

        var B = new BRep_Builder();
        TopoDS_Vertex V1 = new(), V2 = new();
        B.MakeVertex(ref V1, P1, Tol);
        if (P1.Distance(P2) < Tol)
        {
            V2 = V1;
        }
        else
        {
            B.MakeVertex(ref V2, P2, Tol);
        }

        Init(C, S, V1, V2);
    }

    public void Init(Geom2d_Curve C, Geom_Surface S, in TopoDS_Vertex V1, in TopoDS_Vertex V2)
    {
        // try projecting the vertices on the curve

        double p1, p2;

        if (V1.IsNull())
        {
            p1 = C.FirstParameter();
        }
        else if (!Project(C, S, V1, out p1))
        {
            myError = BRepLib_PointProjectionFailed;
            return;
        }
        if (V2.IsNull())
        {
            p2 = C.LastParameter();
        }
        else if (!Project(C, S, V2, out p2))
        {
            myError = BRepLib_PointProjectionFailed;
            return;
        }

        Init(C, S, V1, V2, p1, p2);
    }

    public void Init(Geom2d_Curve C, Geom_Surface S, in gp_Pnt P1, in gp_Pnt P2, double p1, double p2)
    {
        double Tol = BRepLib.Precision();
        var B = new BRep_Builder();

        TopoDS_Vertex V1 = new(), V2 = new();
        B.MakeVertex(ref V1, P1, Tol);
        if (P1.Distance(P2) < Tol)
        {
            V2 = V1;
        }
        else
        {
            B.MakeVertex(ref V2, P2, Tol);
        }

        Init(C, S, V1, V2, p1, p2);
    }

    /// <summary>
    /// Makes the edge on the curve on the surface between the parameters, with the given vertices (or new ones). The
    /// edge has no 3D curve.
    /// </summary>
    public void Init(Geom2d_Curve CC, Geom_Surface S, in TopoDS_Vertex VV1, in TopoDS_Vertex VV2, double pp1, double pp2)
    {
        // kill trimmed curves
        Geom2d_Curve C = CC;
        Geom2d_TrimmedCurve? CT = C as Geom2d_TrimmedCurve;
        while (CT is not null)
        {
            C = CT.BasisCurve();
            CT = C as Geom2d_TrimmedCurve;
        }

        // check parameters
        double p1 = pp1;
        double p2 = pp2;
        double cf = C.FirstParameter();
        double cl = C.LastParameter();
        double epsilon = Precision.PConfusion();
        bool periodic = C.IsPeriodic();

        TopoDS_Vertex V1, V2;
        bool reverse = false;

        if (periodic)
        {
            // adjust in period
            ElCLib.AdjustPeriodic(cf, cl, epsilon, ref p1, ref p2);
            V1 = VV1;
            V2 = VV2;
        }
        else
        {
            // reordonate
            if (p1 < p2)
            {
                V1 = VV1;
                V2 = VV2;
            }
            else
            {
                V2 = VV1;
                V1 = VV2;
                double x = p1;
                p1 = p2;
                p2 = x;
                reverse = true;
            }

            // check range
            if ((cf - p1 > epsilon) || (p2 - cl > epsilon))
            {
                myError = BRepLib_ParameterOutOfRange;
                return;
            }
        }

        // compute points on the curve
        bool p1inf = Precision.IsNegativeInfinite(p1);
        bool p2inf = Precision.IsPositiveInfinite(p2);
        gp_Pnt P1 = new(), P2 = new();
        gp_Pnt2d P2d1, P2d2;
        if (!p1inf)
        {
            P2d1 = C.Value(p1);
            P1 = S.Value(P2d1.X(), P2d1.Y());
        }
        if (!p2inf)
        {
            P2d2 = C.Value(p2);
            P2 = S.Value(P2d2.X(), P2d2.Y());
        }

        double preci = BRepLib.Precision();
        var B = new BRep_Builder();

        // check for closed curve
        bool closed = false;
        if (!p1inf && !p2inf)
        {
            closed = (P1.Distance(P2) <= preci);
        }

        // check if the vertices are on the curve
        if (closed)
        {
            if (V1.IsNull() && V2.IsNull())
            {
                B.MakeVertex(ref V1, P1, preci);
                V2 = V1;
            }
            else if (V1.IsNull())
            {
                V1 = V2;
            }
            else if (V2.IsNull())
            {
                V2 = V1;
            }
            else
            {
                if (!V1.IsSame(V2))
                {
                    myError = BRepLib_DifferentPointsOnClosedCurve;
                    return;
                }
                else if (P1.Distance(BRep_Tool.Pnt(V1)) > Max(preci, BRep_Tool.Tolerance(V1)))
                {
                    myError = BRepLib_DifferentPointsOnClosedCurve;
                    return;
                }
            }
        }

        else
        { // not closed

            if (p1inf)
            {
                if (!V1.IsNull())
                {
                    myError = BRepLib_PointWithInfiniteParameter;
                    return;
                }
            }
            else
            {
                if (V1.IsNull())
                {
                    B.MakeVertex(ref V1, P1, preci);
                }
                else if (P1.Distance(BRep_Tool.Pnt(V1)) > Max(preci, BRep_Tool.Tolerance(V1)))
                {
                    myError = BRepLib_DifferentsPointAndParameter;
                    return;
                }
            }

            if (p2inf)
            {
                if (!V2.IsNull())
                {
                    myError = BRepLib_PointWithInfiniteParameter;
                    return;
                }
            }
            else
            {
                if (V2.IsNull())
                {
                    B.MakeVertex(ref V2, P2, preci);
                }
                else if (P2.Distance(BRep_Tool.Pnt(V2)) > Max(preci, BRep_Tool.Tolerance(V2)))
                {
                    myError = BRepLib_DifferentsPointAndParameter;
                    return;
                }
            }
        }

        V1.Orientation(TopAbs_FORWARD);
        V2.Orientation(TopAbs_REVERSED);
        myVertex1 = V1;
        myVertex2 = V2;

        // OCCT.NET: a new variable, as in the 3D Init.
        TopoDS_Edge E = new();
        B.MakeEdge(ref E);
        B.UpdateEdge(E, C, S, new TopLoc_Location(), preci);

        if (!V1.IsNull())
        {
            B.Add(E, V1);
        }
        if (!V2.IsNull())
        {
            B.Add(E, V2);
        }
        B.Range(E, p1, p2);

        if (reverse)
        {
            E.Orientation(TopAbs_REVERSED);
        }
        myShape = E;

        myError = BRepLib_EdgeDone;
        Done();
    }

    /// <summary>Returns the construction status.</summary>
    public BRepLib_EdgeError Error()
    {
        return myError;
    }

    /// <summary>The edge (frozen).</summary>
    public TopoDS_Edge Edge()
    {
        return TopoDS.Edge(Shape());
    }

    /// <summary><see cref="Edge"/> without freezing, for ported code.</summary>
    internal TopoDS_Edge InternalEdge()
    {
        return TopoDS.Edge(InternalShape());
    }

    /// <summary>The first vertex of the edge (frozen with the edge). May be null.</summary>
    public TopoDS_Vertex Vertex1()
    {
        return TopoDS.Vertex(Published(InternalVertex1()));
    }

    /// <summary><see cref="Vertex1"/> without freezing, for ported code.</summary>
    internal TopoDS_Vertex InternalVertex1()
    {
        Check();
        return myVertex1;
    }

    /// <summary>The second vertex of the edge (frozen with the edge). May be null.</summary>
    public TopoDS_Vertex Vertex2()
    {
        return TopoDS.Vertex(Published(InternalVertex2()));
    }

    /// <summary><see cref="Vertex2"/> without freezing, for ported code.</summary>
    internal TopoDS_Vertex InternalVertex2()
    {
        Check();
        return myVertex2;
    }

    /// <summary>The edge (OCCT's conversion operator; frozen).</summary>
    public static explicit operator TopoDS_Edge(BRepLib_MakeEdge theMaker) => theMaker.Edge();
}
