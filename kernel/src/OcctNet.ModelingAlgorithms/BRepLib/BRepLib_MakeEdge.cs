// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakeEdge.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakeEdge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.BRepLib_EdgeError;
using static Kerf.Topology.Orientation;

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
    private Vertex myVertex1 = new();
    private Vertex myVertex2 = new();

    private static bool Project(Curve3D C, in Vertex V, out double p)
    {
        p = 0.0;
        double Eps2 = Tool.Tolerance(V);
        Eps2 *= Eps2;

        Point3D P = Tool.Pnt(V);
        var GAC = new CurveAdaptor(C);

        // Afin de faire les extremas, on verifie les distances en bout
        double D1, D2;
        Point3D P1, P2;
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

        var extrema = new CurvePointExtrema(P, GAC);
        if (extrema.IsDone)
        {
            int i, index = 0, n = extrema.ExtCount;
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
    private static bool Project(Curve2D C, Surface S, in Vertex V, out double p)
    {
        p = 0.0;
        Point3D P = Tool.Pnt(V);
        double Eps2 = Tool.Tolerance(V);
        Eps2 *= Eps2;

        var HG2AHC = new CurveAdaptor2D(C);
        var HGAHS = new SurfaceAdaptor(S);
        var ACOS = new CurveOnSurface(HG2AHC, HGAHS);

        double D1, D2;
        Point3D P1, P2;
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

        var extrema = new CurvePointExtrema(P, ACOS);

        if (extrema.IsDone)
        {
            int i, index = 0, n = extrema.ExtCount;
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
                POnCurv POC = extrema.Point(index);
                if (P.SquareDistance(POC.Value()) <= global::Kerf.Engine.Foundation.Precision.SquareConfusion())
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

    public BRepLib_MakeEdge(in Vertex V1, in Vertex V2)
    {
        Point3D P1 = Tool.Pnt(V1);
        Point3D P2 = Tool.Pnt(V2);
        double l = P1.Distance(P2);
        if (l <= Constants.Resolution())
        {
            myError = BRepLib_LineThroughIdenticPoints;
            return;
        }
        var L = new Line3D(P1, new Direction3D(new Vector3D(P1, P2)));
        var GL = new LineCurve3D(L);
        Init(GL, V1, V2, 0, l);
    }

    public BRepLib_MakeEdge(in Point3D P1, in Point3D P2)
    {
        double l = P1.Distance(P2);
        if (l <= Constants.Resolution())
        {
            myError = BRepLib_LineThroughIdenticPoints;
            return;
        }
        var L = new Line3D(P1, new Direction3D(new Vector3D(P1, P2)));
        var GL = new LineCurve3D(L);
        Init(GL, P1, P2, 0, l);
    }

    public BRepLib_MakeEdge(in Line3D L) => Init(new LineCurve3D(L));

    public BRepLib_MakeEdge(in Line3D L, double p1, double p2) => Init(new LineCurve3D(L), p1, p2);

    public BRepLib_MakeEdge(in Line3D L, in Point3D P1, in Point3D P2) => Init(new LineCurve3D(L), P1, P2);

    public BRepLib_MakeEdge(in Line3D L, in Vertex V1, in Vertex V2) => Init(new LineCurve3D(L), V1, V2);

    public BRepLib_MakeEdge(in Circle3D C) => Init(new CircleCurve3D(C));

    public BRepLib_MakeEdge(in Circle3D C, double p1, double p2) => Init(new CircleCurve3D(C), p1, p2);

    public BRepLib_MakeEdge(in Circle3D C, in Point3D P1, in Point3D P2) => Init(new CircleCurve3D(C), P1, P2);

    public BRepLib_MakeEdge(in Circle3D C, in Vertex V1, in Vertex V2) => Init(new CircleCurve3D(C), V1, V2);

    public BRepLib_MakeEdge(in Ellipse3D E) => Init(new EllipseCurve3D(E));

    public BRepLib_MakeEdge(in Ellipse3D E, double p1, double p2) => Init(new EllipseCurve3D(E), p1, p2);

    public BRepLib_MakeEdge(in Ellipse3D E, in Point3D P1, in Point3D P2) => Init(new EllipseCurve3D(E), P1, P2);

    public BRepLib_MakeEdge(in Ellipse3D E, in Vertex V1, in Vertex V2) => Init(new EllipseCurve3D(E), V1, V2);

    public BRepLib_MakeEdge(in Hyperbola3D H) => Init(new HyperbolaCurve3D(H));

    public BRepLib_MakeEdge(in Hyperbola3D H, double p1, double p2) => Init(new HyperbolaCurve3D(H), p1, p2);

    public BRepLib_MakeEdge(in Hyperbola3D H, in Point3D P1, in Point3D P2) => Init(new HyperbolaCurve3D(H), P1, P2);

    public BRepLib_MakeEdge(in Hyperbola3D H, in Vertex V1, in Vertex V2) => Init(new HyperbolaCurve3D(H), V1, V2);

    public BRepLib_MakeEdge(in Parabola3D P) => Init(new ParabolaCurve3D(P));

    public BRepLib_MakeEdge(in Parabola3D P, double p1, double p2) => Init(new ParabolaCurve3D(P), p1, p2);

    public BRepLib_MakeEdge(in Parabola3D P, in Point3D P1, in Point3D P2) => Init(new ParabolaCurve3D(P), P1, P2);

    public BRepLib_MakeEdge(in Parabola3D P, in Vertex V1, in Vertex V2) => Init(new ParabolaCurve3D(P), V1, V2);

    public BRepLib_MakeEdge(Curve3D L) => Init(L);

    public BRepLib_MakeEdge(Curve3D L, double p1, double p2) => Init(L, p1, p2);

    public BRepLib_MakeEdge(Curve3D L, in Point3D P1, in Point3D P2) => Init(L, P1, P2);

    public BRepLib_MakeEdge(Curve3D L, in Vertex V1, in Vertex V2) => Init(L, V1, V2);

    public BRepLib_MakeEdge(Curve3D L, in Point3D P1, in Point3D P2, double p1, double p2) => Init(L, P1, P2, p1, p2);

    public BRepLib_MakeEdge(Curve3D L, in Vertex V1, in Vertex V2, double p1, double p2)
        => Init(L, V1, V2, p1, p2);

    public BRepLib_MakeEdge(Curve2D L, Surface S) => Init(L, S);

    public BRepLib_MakeEdge(Curve2D L, Surface S, double p1, double p2) => Init(L, S, p1, p2);

    public BRepLib_MakeEdge(Curve2D L, Surface S, in Point3D P1, in Point3D P2) => Init(L, S, P1, P2);

    public BRepLib_MakeEdge(Curve2D L, Surface S, in Vertex V1, in Vertex V2) => Init(L, S, V1, V2);

    public BRepLib_MakeEdge(Curve2D L, Surface S, in Point3D P1, in Point3D P2, double p1, double p2)
        => Init(L, S, P1, P2, p1, p2);

    public BRepLib_MakeEdge(Curve2D L, Surface S, in Vertex V1, in Vertex V2, double p1, double p2)
        => Init(L, S, V1, V2, p1, p2);

    public void Init(Curve3D C)
    {
        Init(C, C.FirstParameter(), C.LastParameter());
    }

    public void Init(Curve3D C, double p1, double p2)
    {
        Vertex V1 = new(), V2 = new();
        Init(C, V1, V2, p1, p2);
    }

    public void Init(Curve3D C, in Point3D P1, in Point3D P2)
    {
        double Tol = BRepLib.Precision();

        var B = new BrepBuilder();
        Vertex V1 = new(), V2 = new();
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

    public void Init(Curve3D C, in Vertex V1, in Vertex V2)
    {
        // try projecting the vertices on the curve

        double p1, p2;

        if (V1.IsNull)
        {
            p1 = C.FirstParameter();
        }
        else if (!Project(C, V1, out p1))
        {
            myError = BRepLib_PointProjectionFailed;
            return;
        }
        if (V2.IsNull)
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

    public void Init(Curve3D C, in Point3D P1, in Point3D P2, double p1, double p2)
    {
        double Tol = BRepLib.Precision();
        var B = new BrepBuilder();

        Vertex V1 = new(), V2 = new();
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
    public void Init(Curve3D CC, in Vertex VV1, in Vertex VV2, double pp1, double pp2)
    {
        // kill trimmed curves
        Curve3D C = CC;
        TrimmedCurve3D? CT = C as TrimmedCurve3D;
        while (CT is not null)
        {
            C = CT.BasisCurve();
            CT = C as TrimmedCurve3D;
        }

        // check parameters
        double p1 = pp1;
        double p2 = pp2;
        double cf = C.FirstParameter();
        double cl = C.LastParameter();
        double epsilon = global::Kerf.Engine.Foundation.Precision.PConfusion();
        bool periodic = C.IsPeriodic;
        var aCA = new CurveAdaptor(C);

        Vertex V1, V2;
        if (periodic)
        {
            // adjust in period
            global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(cf, cl, epsilon, ref p1, ref p2);
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
            if ((p2 - p1) <= Constants.Resolution())
            {
                myError = BRepLib_LineThroughIdenticPoints;
                return;
            }
        }

        // compute points on the curve
        bool p1inf = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(p1);
        bool p2inf = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(p2);
        Point3D P1 = new(), P2 = new();
        if (!p1inf)
        {
            P1 = aCA.Value(p1);
        }
        if (!p2inf)
        {
            P2 = aCA.Value(p2);
        }

        double preci = BRepLib.Precision();
        var B = new BrepBuilder();

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
            if (V1.IsNull && V2.IsNull)
            {
                B.MakeVertex(ref V1, P1, preci);
                V2 = V1;
            }
            else if (V1.IsNull)
            {
                V1 = V2;
            }
            else if (V2.IsNull)
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
                else if (P1.Distance(Tool.Pnt(V1)) > Max(preci, Tool.Tolerance(V1)))
                {
                    myError = BRepLib_DifferentPointsOnClosedCurve;
                    return;
                }
                else
                {
                    Point3D PM = aCA.Value((p1 + p2) / 2);
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
                if (!V1.IsNull)
                {
                    myError = BRepLib_PointWithInfiniteParameter;
                    return;
                }
            }
            else
            {
                if (V1.IsNull)
                {
                    B.MakeVertex(ref V1, P1, preci);
                }
                else if (P1.Distance(Tool.Pnt(V1)) > Max(preci, Tool.Tolerance(V1)))
                {
                    myError = BRepLib_DifferentsPointAndParameter;
                    return;
                }
            }

            if (p2inf)
            {
                if (!V2.IsNull)
                {
                    myError = BRepLib_PointWithInfiniteParameter;
                    return;
                }
            }
            else
            {
                if (V2.IsNull)
                {
                    B.MakeVertex(ref V2, P2, preci);
                }
                else if (P2.Distance(Tool.Pnt(V2)) > Max(preci, Tool.Tolerance(V2)))
                {
                    myError = BRepLib_DifferentsPointAndParameter;
                    return;
                }
            }
        }

        V1.Orientation(Forward);
        V2.Orientation(Reversed);
        myVertex1 = V1;
        myVertex2 = V2;

        // OCCT.NET: OCCT remakes the edge in myShape (TopoDS::Edge(myShape)); the port makes it in a new variable,
        // so that Init after a public (freezing) read of the edge doesn't throw TopoDS_LockedShape. The result is
        // the same: MakeEdge gives a new TShape with the identity location, FORWARD.
        Edge E = new();
        B.MakeEdge(ref E, C, preci);
        if (!V1.IsNull)
        {
            B.Add(E, V1);
        }
        if (!V2.IsNull)
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

    public void Init(Curve2D C, Surface S)
    {
        Init(C, S, C.FirstParameter(), C.LastParameter());
    }

    public void Init(Curve2D C, Surface S, double p1, double p2)
    {
        Vertex V1 = new(), V2 = new();
        Init(C, S, V1, V2, p1, p2);
    }

    public void Init(Curve2D C, Surface S, in Point3D P1, in Point3D P2)
    {
        double Tol = BRepLib.Precision();

        var B = new BrepBuilder();
        Vertex V1 = new(), V2 = new();
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

    public void Init(Curve2D C, Surface S, in Vertex V1, in Vertex V2)
    {
        // try projecting the vertices on the curve

        double p1, p2;

        if (V1.IsNull)
        {
            p1 = C.FirstParameter();
        }
        else if (!Project(C, S, V1, out p1))
        {
            myError = BRepLib_PointProjectionFailed;
            return;
        }
        if (V2.IsNull)
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

    public void Init(Curve2D C, Surface S, in Point3D P1, in Point3D P2, double p1, double p2)
    {
        double Tol = BRepLib.Precision();
        var B = new BrepBuilder();

        Vertex V1 = new(), V2 = new();
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
    public void Init(Curve2D CC, Surface S, in Vertex VV1, in Vertex VV2, double pp1, double pp2)
    {
        // kill trimmed curves
        Curve2D C = CC;
        TrimmedCurve2D? CT = C as TrimmedCurve2D;
        while (CT is not null)
        {
            C = CT.BasisCurve();
            CT = C as TrimmedCurve2D;
        }

        // check parameters
        double p1 = pp1;
        double p2 = pp2;
        double cf = C.FirstParameter();
        double cl = C.LastParameter();
        double epsilon = global::Kerf.Engine.Foundation.Precision.PConfusion();
        bool periodic = C.IsPeriodic;

        Vertex V1, V2;
        bool reverse = false;

        if (periodic)
        {
            // adjust in period
            global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(cf, cl, epsilon, ref p1, ref p2);
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
        bool p1inf = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(p1);
        bool p2inf = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(p2);
        Point3D P1 = new(), P2 = new();
        Point2D P2d1, P2d2;
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
        var B = new BrepBuilder();

        // check for closed curve
        bool closed = false;
        if (!p1inf && !p2inf)
        {
            closed = (P1.Distance(P2) <= preci);
        }

        // check if the vertices are on the curve
        if (closed)
        {
            if (V1.IsNull && V2.IsNull)
            {
                B.MakeVertex(ref V1, P1, preci);
                V2 = V1;
            }
            else if (V1.IsNull)
            {
                V1 = V2;
            }
            else if (V2.IsNull)
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
                else if (P1.Distance(Tool.Pnt(V1)) > Max(preci, Tool.Tolerance(V1)))
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
                if (!V1.IsNull)
                {
                    myError = BRepLib_PointWithInfiniteParameter;
                    return;
                }
            }
            else
            {
                if (V1.IsNull)
                {
                    B.MakeVertex(ref V1, P1, preci);
                }
                else if (P1.Distance(Tool.Pnt(V1)) > Max(preci, Tool.Tolerance(V1)))
                {
                    myError = BRepLib_DifferentsPointAndParameter;
                    return;
                }
            }

            if (p2inf)
            {
                if (!V2.IsNull)
                {
                    myError = BRepLib_PointWithInfiniteParameter;
                    return;
                }
            }
            else
            {
                if (V2.IsNull)
                {
                    B.MakeVertex(ref V2, P2, preci);
                }
                else if (P2.Distance(Tool.Pnt(V2)) > Max(preci, Tool.Tolerance(V2)))
                {
                    myError = BRepLib_DifferentsPointAndParameter;
                    return;
                }
            }
        }

        V1.Orientation(Forward);
        V2.Orientation(Reversed);
        myVertex1 = V1;
        myVertex2 = V2;

        // OCCT.NET: a new variable, as in the 3D Init.
        Edge E = new();
        B.MakeEdge(ref E);
        B.UpdateEdge(E, C, S, new Placement(), preci);

        if (!V1.IsNull)
        {
            B.Add(E, V1);
        }
        if (!V2.IsNull)
        {
            B.Add(E, V2);
        }
        B.Range(E, p1, p2);

        if (reverse)
        {
            E.Orientation(Reversed);
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
    public Edge Edge()
    {
        return ShapeCasts.Edge(Shape());
    }

    /// <summary><see cref="Edge"/> without freezing, for ported code.</summary>
    internal Edge InternalEdge()
    {
        return ShapeCasts.Edge(InternalShape());
    }

    /// <summary>The first vertex of the edge (frozen with the edge). May be null.</summary>
    public Vertex Vertex1()
    {
        return ShapeCasts.Vertex(Published(InternalVertex1()));
    }

    /// <summary><see cref="Vertex1"/> without freezing, for ported code.</summary>
    internal Vertex InternalVertex1()
    {
        Check();
        return myVertex1;
    }

    /// <summary>The second vertex of the edge (frozen with the edge). May be null.</summary>
    public Vertex Vertex2()
    {
        return ShapeCasts.Vertex(Published(InternalVertex2()));
    }

    /// <summary><see cref="Vertex2"/> without freezing, for ported code.</summary>
    internal Vertex InternalVertex2()
    {
        Check();
        return myVertex2;
    }

    /// <summary>The edge (OCCT's conversion operator; frozen).</summary>
    public static explicit operator Edge(BRepLib_MakeEdge theMaker) => theMaker.Edge();
}
