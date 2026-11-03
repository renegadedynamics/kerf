// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_IntersectionTool.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_IntersectionTool.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_IntersectionTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.IntRes2d_Position;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// Tool for fixing the intersecting edges of a wire (and the intersecting wires of a face): it splits and cuts the
/// edges at the intersections, merges close vertices, and records the replacements in a context.
/// </summary>
/// <remarks>
/// A working object (decision 21). Its fixes write in place into the edges, vertices, wire data and face they are
/// given (ranges, vertex tolerances, the edges of a <see cref="ShapeExtend_WireData"/>), so they are internal: ported
/// callers (ShapeFix_Wire, ShapeFix_Face) run them on shapes they own (their copies, decision 583).
/// </remarks>
public sealed class ShapeFix_IntersectionTool
{
    private readonly ShapeBuild_ReShape? myContext;
    private readonly double              myPreci;
    private readonly double              myMaxTol;

    // gka 06.09.04 BUG 6555 shape is modified always independently either intersection was fixed or not

    /// <summary>Creates the tool with a context, a precision and a maximal tolerance.</summary>
    public ShapeFix_IntersectionTool(ShapeBuild_ReShape? context, double preci, double maxtol = 1.0)
    {
        myContext = context;
        myPreci   = preci;
        myMaxTol  = maxtol;
    }

    /// <summary>Returns the context.</summary>
    public ShapeBuild_ReShape? Context() => myContext;

    //=======================================================================
    // function : GetPointOnEdge
    // purpose  : auxiliary
    //: h0 abv 29 May 98: PRO10105 1949: like in BRepCheck, point is to be taken
    // from 3d curve (but only if edge is SameParameter)
    //=======================================================================
    private static Point3D GetPointOnEdge(in Edge edge, ShapeAnalysis_Surface surf, CurveAdaptor2D Crv2d,
                                         double param)
    {
        if (Tool.SameParameter(edge))
        {
            Curve3D? ConS = Tool.Curve(edge, out Placement L, out _, out _);
            if (ConS is not null)
            {
                return ConS.Value(param).Transformed(L.Transformation());
            }
        }
        Point2D aP2d = Crv2d.Value(param);
        return surf.Adaptor3d()!.Value(aP2d.X(), aP2d.Y());
    }

    /// <summary>
    /// Splits <paramref name="edge"/> at <paramref name="param"/> with the vertex <paramref name="vert"/> (whose
    /// tolerance is raised if needed) into two new edges; false if the vertex is one of the edge's or the parameter is
    /// at an end.
    /// </summary>
    internal bool SplitEdge(in Edge edge, double param, in Vertex vert, in Face face,
                            ref Edge newE1, ref Edge newE2, double preci)
    {
        var sae = new ShapeAnalysis_Edge();

        Vertex V1 = sae.FirstVertex(edge);
        Vertex V2 = sae.LastVertex(edge);
        if (V1.IsSame(vert) || V2.IsSame(vert))
        {
            return false;
        }

        sae.PCurve(edge, face, out Curve2D? c2d, out double a, out double b, true);
        if (Math.Abs(a - param) < 0.01 * preci || Math.Abs(b - param) < 0.01 * preci)
        {
            return false;
        }
        // check distance between edge and new vertex
        Point3D          P1;
        Placement L;
        if (Tool.SameParameter(edge) && !Tool.Degenerated(edge))
        {
            Curve3D? c3d = Tool.Curve(edge, out L, out _, out _);
            if (c3d is null)
            {
                return false;
            }
            P1 = c3d.Value(param);
            if (!L.IsIdentity)
            {
                P1 = P1.Transformed(L.Transformation());
            }
        }
        else
        {
            Surface? surf = Tool.Surface(face, out L);
            var           sas  = new ShapeAnalysis_Surface(surf);
            P1                 = sas.Value(c2d!.Value(param));
            if (!L.IsIdentity)
            {
                P1 = P1.Transformed(L.Transformation());
            }
        }
        Point3D P2 = Tool.Pnt(vert);
        if (P1.Distance(P2) > preci)
        {
            // return false;
            var B0 = new BrepBuilder();
            B0.UpdateVertex(vert, P1.Distance(P2));
        }

        var transferParameters = new ShapeAnalysis_TransferParametersProj();
        transferParameters.SetMaxTolerance(preci);
        transferParameters.Init(edge, face);
        double first, last;
        if (a < b)
        {
            first = a;
            last  = b;
        }
        else
        {
            first = b;
            last  = a;
        }

        var                sbe    = new ShapeBuild_Edge();
        Orientation orient = edge.Orientation();
        var                B      = new BrepBuilder();
        Edge        wE     = edge;
        wE.Orientation(Forward);
        Shape aTmpShape = vert.Oriented(Reversed); // for porting
        newE1 = sbe.InternalCopyReplaceVertices(wE, sae.FirstVertex(wE), ShapeCasts.Vertex(aTmpShape));
        sbe.CopyPCurves(newE1, wE);
        transferParameters.TransferRange(newE1, first, param, true);
        B.SameRange(newE1, false);
        B.SameParameter(newE1, false);
        aTmpShape = vert.Oriented(Forward);
        newE2     = sbe.InternalCopyReplaceVertices(wE, ShapeCasts.Vertex(aTmpShape), sae.LastVertex(wE));
        sbe.CopyPCurves(newE2, wE);
        transferParameters.TransferRange(newE2, param, last, true);
        B.SameRange(newE2, false);
        B.SameParameter(newE2, false);

        newE1.Orientation(orient);
        newE2.Orientation(orient);
        if (orient == Reversed)
        {
            Edge tmp = newE2;
            newE2           = newE1;
            newE1           = tmp;
        }

        return true;
    }

    /// <summary>
    /// Cuts the edge (its range, in place) from <paramref name="pend"/> to <paramref name="cut"/>; true if done.
    /// </summary>
    internal bool CutEdge(in Edge edge, double pend, double cut, in Face face, ref bool iscutline)
    {
        if (Math.Abs(cut - pend) < 10.0 * global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return false;
        }
        double aRange = Math.Abs(cut - pend);
        Tool.Range(edge, out double a, out double b);

        if (aRange < 10.0 * global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return false;
        }

        // case pcurve is trimm of line
        if (!Tool.SameParameter(edge))
        {
            var sae = new ShapeAnalysis_Edge();
            if (sae.PCurve(edge, face, out Curve2D? Crv, out double fp, out double lp, false))
            {
                if (Crv is TrimmedCurve2D tc)
                {
                    if (tc.BasisCurve() is LineCurve2D)
                    {
                        var B0 = new BrepBuilder();
                        B0.Range(edge, Min(pend, cut), Max(pend, cut));
                        if (Math.Abs(pend - lp) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                        { // cut from the beginning
                            double cut3d = (cut - fp) * (b - a) / (lp - fp);
                            B0.Range(edge, a + cut3d, b, true);
                            iscutline = true;
                        }
                        else if (Math.Abs(pend - fp) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                        { // cut from the end
                            double cut3d = (lp - cut) * (b - a) / (lp - fp);
                            B0.Range(edge, a, b - cut3d, true);
                            iscutline = true;
                        }
                    }

                    return true;
                }
            }
            return false;
        }

        // det-study on 03/12/01 checking the old and new ranges
        if (Math.Abs(Math.Abs(a - b) - aRange) < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return false;
        }
        if (aRange < 10.0 * global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return false;
        }

        var B = new BrepBuilder();
        B.Range(edge, Min(pend, cut), Max(pend, cut));

        return true;
    }

    //=======================================================================
    // function : SplitEdge1
    // purpose  : split edge[a,b] om two part e1[a,param]
    //           and e2[param,b] using vertex vert
    //=======================================================================
    private bool SplitEdge1(ShapeExtend_WireData sewd, in Face face, int num, double param,
                            in Vertex vert, double preci, NCollection_DataMap<Shape, Box2d> boxes)
    {
        // Standard_ASSERT_RETURN: "Edge index out of range"
        if (!(num > 0 && num <= sewd.NbEdges()))
        {
            return false;
        }

        Edge edge  = sewd.Edge(num);
        Edge newE1 = new(), newE2 = new();
        if (!SplitEdge(edge, param, vert, face, ref newE1, ref newE2, preci))
        {
            return false;
        }

        // change context
        var wd = new ShapeExtend_WireData();
        wd.Add(newE1);
        wd.Add(newE2);
        myContext?.Replace(edge, wd.InternalWire());
        for (var exp = new Explorer(wd.InternalWire(), ShapeType.Edge); exp.More(); exp.Next())
        {
            Edge E = ShapeCasts.Edge(exp.Current());
            BrepTools.Update(E);
        }

        // change sewd
        sewd.Set(newE1, num);
        if (num == sewd.NbEdges())
        {
            sewd.Add(newE2);
        }
        else
        {
            sewd.Add(newE2, num + 1);
        }

        // change boxes
        boxes.UnBind(edge);
        Surface? S = Tool.Surface(face, out Placement L);
        var           sae = new ShapeAnalysis_Edge();
        if (sae.PCurve(newE1, S, L, out Curve2D? c2d, out double cf, out double cl, false))
        {
            var    box    = new Box2d();
            var    gac    = new CurveAdaptor2D();
            double aFirst = c2d!.FirstParameter();
            double aLast  = c2d.LastParameter();
            if (c2d is BSplineCurve2D && (cf < aFirst || cl > aLast))
            {
                // pdn avoiding problems with segment in Bnd_Box
                gac.Load(c2d);
            }
            else
            {
                gac.Load(c2d, cf, cl);
            }
            Add2dCurve.Add(gac, global::Kerf.Engine.Foundation.Precision.Confusion(), ref box);
            boxes.Bind(newE1, box);
        }
        if (sae.PCurve(newE2, S, L, out c2d, out cf, out cl, false))
        {
            var    box    = new Box2d();
            var    gac    = new CurveAdaptor2D();
            double aFirst = c2d!.FirstParameter();
            double aLast  = c2d.LastParameter();
            if (c2d is BSplineCurve2D && (cf < aFirst || cl > aLast))
            {
                // pdn avoiding problems with segment in Bnd_Box
                gac.Load(c2d);
            }
            else
            {
                gac.Load(c2d, cf, cl);
            }
            Add2dCurve.Add(gac, global::Kerf.Engine.Foundation.Precision.Confusion(), ref box);
            boxes.Bind(newE2, box);
        }

        return true;
    }

    //=======================================================================
    // function : SplitEdge2
    // purpose  : auxiliary: split edge[a,b] om two part e1[a,param1]
    //                     and e2[param2,b] using vertex vert
    //                     (remove segment (param1,param2) from edge)
    //=======================================================================
    private bool SplitEdge2(ShapeExtend_WireData sewd, in Face face, int num, double param1, double param2,
                            in Vertex vert, double preci, NCollection_DataMap<Shape, Box2d> boxes)
    {
        Edge edge  = sewd.Edge(num);
        Edge newE1 = new(), newE2 = new();
        double      param = (param1 + param2) / 2;
        if (!SplitEdge(edge, param, vert, face, ref newE1, ref newE2, preci))
        {
            return false;
        }
        // cut new edges by param1 and param2
        bool IsCutLine = false;
        var  sae       = new ShapeAnalysis_Edge();
        if (sae.PCurve(newE1, face, out _, out double fp1, out double lp1, false))
        {
            if (sae.PCurve(newE2, face, out _, out double fp2, out double lp2, false))
            {
                if (lp1 == param)
                {
                    if ((lp1 - fp1) * (lp1 - param1) > 0)
                    {
                        CutEdge(newE1, fp1, param1, face, ref IsCutLine);
                        CutEdge(newE2, lp2, param2, face, ref IsCutLine);
                    }
                    else
                    {
                        CutEdge(newE1, fp1, param2, face, ref IsCutLine);
                        CutEdge(newE2, lp2, param1, face, ref IsCutLine);
                    }
                }
                else
                {
                    if ((fp1 - lp1) * (fp1 - param1) > 0)
                    {
                        CutEdge(newE1, lp1, param1, face, ref IsCutLine);
                        CutEdge(newE2, fp2, param2, face, ref IsCutLine);
                    }
                    else
                    {
                        CutEdge(newE1, lp1, param2, face, ref IsCutLine);
                        CutEdge(newE2, fp2, param1, face, ref IsCutLine);
                    }
                }
            }
        }

        // change context
        var wd = new ShapeExtend_WireData();
        wd.Add(newE1);
        wd.Add(newE2);
        myContext?.Replace(edge, wd.InternalWire());
        for (var exp = new Explorer(wd.InternalWire(), ShapeType.Edge); exp.More(); exp.Next())
        {
            Edge E = ShapeCasts.Edge(exp.Current());
            BrepTools.Update(E);
        }

        // change sewd
        sewd.Set(newE1, num);
        if (num == sewd.NbEdges())
        {
            sewd.Add(newE2);
        }
        else
        {
            sewd.Add(newE2, num + 1);
        }

        // change boxes
        boxes.UnBind(edge);
        Surface? S = Tool.Surface(face, out Placement L);
        if (sae.PCurve(newE1, S, L, out Curve2D? c2d, out double cf, out double cl, false))
        {
            var    box    = new Box2d();
            var    gac    = new CurveAdaptor2D();
            double aFirst = c2d!.FirstParameter();
            double aLast  = c2d.LastParameter();
            if (c2d is BSplineCurve2D && (cf < aFirst || cl > aLast))
            {
                // pdn avoiding problems with segment in Bnd_Box
                gac.Load(c2d);
            }
            else
            {
                gac.Load(c2d, cf, cl);
            }
            Add2dCurve.Add(gac, global::Kerf.Engine.Foundation.Precision.Confusion(), ref box);
            boxes.Bind(newE1, box);
        }
        if (sae.PCurve(newE2, S, L, out c2d, out cf, out cl, false))
        {
            var    box    = new Box2d();
            var    gac    = new CurveAdaptor2D();
            double aFirst = c2d!.FirstParameter();
            double aLast  = c2d.LastParameter();
            if (c2d is BSplineCurve2D && (cf < aFirst || cl > aLast))
            {
                // pdn avoiding problems with segment in Bnd_Box
                gac.Load(c2d);
            }
            else
            {
                gac.Load(c2d, cf, cl);
            }
            Add2dCurve.Add(gac, global::Kerf.Engine.Foundation.Precision.Confusion(), ref box);
            boxes.Bind(newE2, box);
        }

        return true;
    }

    private bool UnionVertexes(ShapeExtend_WireData sewd, ref Edge edge1, ref Edge edge2, int num2,
                               NCollection_DataMap<Shape, Box2d> boxes, in Box2d B2)
    {
        // union vertexes
        bool          res  = false;
        var           sbe  = new ShapeBuild_Edge();
        var           sae  = new ShapeAnalysis_Edge();
        var           B    = new BrepBuilder();
        Vertex V1F  = sae.FirstVertex(edge1);
        Point3D        PV1F = Tool.Pnt(V1F);
        Vertex V1L  = sae.LastVertex(edge1);
        Point3D        PV1L = Tool.Pnt(V1L);
        Vertex V2F  = sae.FirstVertex(edge2);
        Point3D        PV2F = Tool.Pnt(V2F);
        Vertex V2L  = sae.LastVertex(edge2);
        Point3D        PV2L = Tool.Pnt(V2L);
        double        d11  = PV1F.Distance(PV2F);
        double        d12  = PV1F.Distance(PV2L);
        double        d21  = PV1L.Distance(PV2F);
        double        d22  = PV1L.Distance(PV2L);
        if (d11 < d12 && d11 < d21 && d11 < d22)
        {
            double tolv = Max(Tool.Tolerance(V1F), Tool.Tolerance(V2F));
            if (!V2F.IsSame(V1F) && d11 < tolv)
            {
                // union vertexes V1F and V2F
                B.UpdateVertex(V1F, tolv);
                Edge NewE = sbe.InternalCopyReplaceVertices(edge2, V1F, V2L);
                myContext!.Replace(edge2, NewE);
                sewd.Set(NewE, num2);
                edge2 = NewE;
                boxes.Bind(NewE, B2); // update boxes
                // replace vertex in other edge
                int num21, num22;
                if (num2 > 1)
                {
                    num21 = num2 - 1;
                }
                else
                {
                    num21 = sewd.NbEdges();
                }
                if (num2 < sewd.NbEdges())
                {
                    num22 = num2 + 1;
                }
                else
                {
                    num22 = 1;
                }
                Edge   edge21 = sewd.Edge(num21);
                Edge   edge22 = sewd.Edge(num22);
                Vertex V21F   = sae.FirstVertex(edge21);
                Vertex V21L   = sae.LastVertex(edge21);
                Vertex V22F   = sae.FirstVertex(edge22);
                Vertex V22L   = sae.LastVertex(edge22);
                if (V21F.IsSame(V2F))
                {
                    NewE = sbe.InternalCopyReplaceVertices(edge21, V1F, V21L);
                    if (boxes.IsBound(edge21))
                    {
                        boxes.Bind(NewE, boxes.Find(edge21)); // update boxes
                    }
                    myContext!.Replace(edge21, NewE);
                    sewd.Set(NewE, num21);
                }
                if (V21L.IsSame(V2F))
                {
                    NewE = sbe.InternalCopyReplaceVertices(edge21, V21F, V1F);
                    if (boxes.IsBound(edge21))
                    {
                        boxes.Bind(NewE, boxes.Find(edge21)); // update boxes
                    }
                    myContext!.Replace(edge21, NewE);
                    sewd.Set(NewE, num21);
                }
                if (V22F.IsSame(V2F))
                {
                    NewE = sbe.InternalCopyReplaceVertices(edge22, V1F, V22L);
                    if (boxes.IsBound(edge22))
                    {
                        boxes.Bind(NewE, boxes.Find(edge22)); // update boxes
                    }
                    myContext!.Replace(edge22, NewE);
                    sewd.Set(NewE, num22);
                }
                if (V22L.IsSame(V2F))
                {
                    NewE = sbe.InternalCopyReplaceVertices(edge22, V22F, V1F);
                    if (boxes.IsBound(edge22))
                    {
                        boxes.Bind(NewE, boxes.Find(edge22)); // update boxes
                    }
                    myContext!.Replace(edge22, NewE);
                    sewd.Set(NewE, num22);
                }
                res = true;
            }
        }
        else if (d12 < d21 && d12 < d22)
        {
            double tolv = Max(Tool.Tolerance(V1F), Tool.Tolerance(V2L));
            if (!V2L.IsSame(V1F) && d12 < tolv)
            {
                // union vertexes V1F and V2L
                B.UpdateVertex(V1F, tolv);
                Edge NewE = sbe.InternalCopyReplaceVertices(edge2, V2F, V1F);
                myContext!.Replace(edge2, NewE);
                sewd.Set(NewE, num2);
                edge2 = NewE;
                boxes.Bind(NewE, B2); // update boxes
                // replace vertex in other edge
                int num21, num22;
                if (num2 > 1)
                {
                    num21 = num2 - 1;
                }
                else
                {
                    num21 = sewd.NbEdges();
                }
                if (num2 < sewd.NbEdges())
                {
                    num22 = num2 + 1;
                }
                else
                {
                    num22 = 1;
                }
                Edge   edge21 = sewd.Edge(num21);
                Edge   edge22 = sewd.Edge(num22);
                Vertex V21F   = sae.FirstVertex(edge21);
                Vertex V21L   = sae.LastVertex(edge21);
                Vertex V22F   = sae.FirstVertex(edge22);
                Vertex V22L   = sae.LastVertex(edge22);
                if (V21F.IsSame(V2L))
                {
                    NewE = sbe.InternalCopyReplaceVertices(edge21, V1F, V21L);
                    if (boxes.IsBound(edge21))
                    {
                        boxes.Bind(NewE, boxes.Find(edge21)); // update boxes
                    }
                    myContext!.Replace(edge21, NewE);
                    sewd.Set(NewE, num21);
                }
                if (V21L.IsSame(V2L))
                {
                    NewE = sbe.InternalCopyReplaceVertices(edge21, V21F, V1F);
                    if (boxes.IsBound(edge21))
                    {
                        boxes.Bind(NewE, boxes.Find(edge21)); // update boxes
                    }
                    myContext!.Replace(edge21, NewE);
                    sewd.Set(NewE, num21);
                }
                if (V22F.IsSame(V2L))
                {
                    NewE = sbe.InternalCopyReplaceVertices(edge22, V1F, V22L);
                    if (boxes.IsBound(edge22))
                    {
                        boxes.Bind(NewE, boxes.Find(edge22)); // update boxes
                    }
                    myContext!.Replace(edge22, NewE);
                    sewd.Set(NewE, num22);
                }
                if (V22L.IsSame(V2L))
                {
                    NewE = sbe.InternalCopyReplaceVertices(edge22, V22F, V1F);
                    if (boxes.IsBound(edge22))
                    {
                        boxes.Bind(NewE, boxes.Find(edge22)); // update boxes
                    }
                    myContext!.Replace(edge22, NewE);
                    sewd.Set(NewE, num22);
                }
                res = true;
            }
        }
        else if (d21 < d22)
        {
            double tolv = Max(Tool.Tolerance(V1L), Tool.Tolerance(V2F));
            if (!V2F.IsSame(V1L) && d21 < tolv)
            {
                // union vertexes V1L and V2F
                B.UpdateVertex(V1L, tolv);
                Edge NewE = sbe.InternalCopyReplaceVertices(edge2, V1L, V2L);
                myContext!.Replace(edge2, NewE);
                sewd.Set(NewE, num2);
                edge2 = NewE;
                boxes.Bind(NewE, B2); // update boxes
                // replace vertex in other edge
                int num21, num22;
                if (num2 > 1)
                {
                    num21 = num2 - 1;
                }
                else
                {
                    num21 = sewd.NbEdges();
                }
                if (num2 < sewd.NbEdges())
                {
                    num22 = num2 + 1;
                }
                else
                {
                    num22 = 1;
                }
                Edge   edge21 = sewd.Edge(num21);
                Edge   edge22 = sewd.Edge(num22);
                Vertex V21F   = sae.FirstVertex(edge21);
                Vertex V21L   = sae.LastVertex(edge21);
                Vertex V22F   = sae.FirstVertex(edge22);
                Vertex V22L   = sae.LastVertex(edge22);
                if (V21F.IsSame(V2F))
                {
                    NewE = sbe.InternalCopyReplaceVertices(edge21, V1L, V21L);
                    if (boxes.IsBound(edge21))
                    {
                        boxes.Bind(NewE, boxes.Find(edge21)); // update boxes
                    }
                    myContext!.Replace(edge21, NewE);
                    sewd.Set(NewE, num21);
                }
                if (V21L.IsSame(V2F))
                {
                    NewE = sbe.InternalCopyReplaceVertices(edge21, V21F, V1L);
                    if (boxes.IsBound(edge21))
                    {
                        boxes.Bind(NewE, boxes.Find(edge21)); // update boxes
                    }
                    myContext!.Replace(edge21, NewE);
                    sewd.Set(NewE, num21);
                }
                if (V22F.IsSame(V2F))
                {
                    NewE = sbe.InternalCopyReplaceVertices(edge22, V1L, V22L);
                    if (boxes.IsBound(edge22))
                    {
                        boxes.Bind(NewE, boxes.Find(edge22)); // update boxes
                    }
                    myContext!.Replace(edge22, NewE);
                    sewd.Set(NewE, num22);
                }
                if (V22L.IsSame(V2F))
                {
                    NewE = sbe.InternalCopyReplaceVertices(edge22, V22F, V1L);
                    if (boxes.IsBound(edge22))
                    {
                        boxes.Bind(NewE, boxes.Find(edge22)); // update boxes
                    }
                    myContext!.Replace(edge22, NewE);
                    sewd.Set(NewE, num22);
                }
                res = true;
            }
        }
        else
        {
            double tolv = Max(Tool.Tolerance(V1L), Tool.Tolerance(V2L));
            if (!V2L.IsSame(V1L) && d22 < tolv)
            {
                // union vertexes V1L and V2L
                B.UpdateVertex(V1L, tolv);
                Edge NewE = sbe.InternalCopyReplaceVertices(edge2, V2F, V1L);
                myContext!.Replace(edge2, NewE);
                sewd.Set(NewE, num2);
                edge2 = NewE;
                boxes.Bind(NewE, B2); // update boxes
                // replace vertex in other edge
                int num21, num22;
                if (num2 > 1)
                {
                    num21 = num2 - 1;
                }
                else
                {
                    num21 = sewd.NbEdges();
                }
                if (num2 < sewd.NbEdges())
                {
                    num22 = num2 + 1;
                }
                else
                {
                    num22 = 1;
                }
                Edge   edge21 = sewd.Edge(num21);
                Edge   edge22 = sewd.Edge(num22);
                Vertex V21F   = sae.FirstVertex(edge21);
                Vertex V21L   = sae.LastVertex(edge21);
                Vertex V22F   = sae.FirstVertex(edge22);
                Vertex V22L   = sae.LastVertex(edge22);
                if (V21F.IsSame(V2L))
                {
                    NewE = sbe.InternalCopyReplaceVertices(edge21, V1L, V21L);
                    if (boxes.IsBound(edge21))
                    {
                        boxes.Bind(NewE, boxes.Find(edge21)); // update boxes
                    }
                    myContext!.Replace(edge21, NewE);
                    sewd.Set(NewE, num21);
                }
                if (V21L.IsSame(V2L))
                {
                    NewE = sbe.InternalCopyReplaceVertices(edge21, V21F, V1L);
                    if (boxes.IsBound(edge21))
                    {
                        boxes.Bind(NewE, boxes.Find(edge21)); // update boxes
                    }
                    myContext!.Replace(edge21, NewE);
                    sewd.Set(NewE, num21);
                }
                if (V22F.IsSame(V2L))
                {
                    NewE = sbe.InternalCopyReplaceVertices(edge22, V1L, V22L);
                    if (boxes.IsBound(edge22))
                    {
                        boxes.Bind(NewE, boxes.Find(edge22)); // update boxes
                    }
                    myContext!.Replace(edge22, NewE);
                    sewd.Set(NewE, num22);
                }
                if (V22L.IsSame(V2L))
                {
                    NewE = sbe.InternalCopyReplaceVertices(edge22, V22F, V1L);
                    if (boxes.IsBound(edge22))
                    {
                        boxes.Bind(NewE, boxes.Find(edge22)); // update boxes
                    }
                    myContext!.Replace(edge22, NewE);
                    sewd.Set(NewE, num22);
                }
                res = true;
            }
        }

        return res;
    }

    private static Box2d CreateBoxes2d(ShapeExtend_WireData sewd, in Face face,
                                           NCollection_DataMap<Shape, Box2d> boxes)
    {
        // create box2d for edges from wire
        var           aTotalBox = new Box2d();
        Surface? S         = Tool.Surface(face, out Placement L);
        var           sae       = new ShapeAnalysis_Edge();
        for (int i = 1; i <= sewd.NbEdges(); i++)
        {
            Edge E = sewd.Edge(i);
            if (sae.PCurve(E, S, L, out Curve2D? c2d, out double cf, out double cl, false))
            {
                var    box    = new Box2d();
                var    gac    = new CurveAdaptor2D();
                double aFirst = c2d!.FirstParameter();
                double aLast  = c2d.LastParameter();
                if (c2d is BSplineCurve2D && (cf < aFirst || cl > aLast))
                {
                    // pdn avoiding problems with segment in Bnd_Box
                    gac.Load(c2d);
                }
                else
                {
                    gac.Load(c2d, cf, cl);
                }
                Add2dCurve.Add(gac, global::Kerf.Engine.Foundation.Precision.Confusion(), ref box);
                boxes.Bind(E, box);
                aTotalBox.Add(box);
            }
        }
        return aTotalBox;
    }

    private static void SelectIntPnt(Geom2dInt_GInter Inter, out IntRes2d_IntersectionPoint IP,
                                     out IntRes2d_Transition Tr1, out IntRes2d_Transition Tr2)
    {
        IP  = Inter.Point(1);
        Tr1 = IP.TransitionOfFirst();
        Tr2 = IP.TransitionOfSecond();
        if (Inter.NbPoints() == 2)
        {
            // possible second point is better?
            int status1 = 0, status2 = 0;
            if (Tr1.PositionOnCurve() == IntRes2d_Middle)
            {
                status1 += 1;
            }
            if (Tr2.PositionOnCurve() == IntRes2d_Middle)
            {
                status1 += 2;
            }
            IntRes2d_IntersectionPoint IP2  = Inter.Point(2);
            IntRes2d_Transition        Tr12 = IP2.TransitionOfFirst();
            IntRes2d_Transition        Tr22 = IP2.TransitionOfSecond();
            if (Tr12.PositionOnCurve() == IntRes2d_Middle)
            {
                status2 += 1;
            }
            if (Tr22.PositionOnCurve() == IntRes2d_Middle)
            {
                status2 += 2;
            }
            if (status2 > status1)
            {
                IP  = IP2;
                Tr1 = Tr12;
                Tr2 = Tr22;
            }
        }
    }

    private bool FindVertAndSplitEdge(double param1, in Edge edge1, in Edge edge2, Curve2D Crv1,
                                      ref double MaxTolVert, ref int num1, ShapeExtend_WireData sewd,
                                      in Face face, NCollection_DataMap<Shape, Box2d> boxes,
                                      bool aTmpKey)
    {
        // find needed vertex from edge2 and split edge1 using it
        var           sae       = new ShapeAnalysis_Edge();
        var           sas       = new ShapeAnalysis_Surface(Tool.Surface(face));
        Point3D        pi1       = GetPointOnEdge(edge1, sas, new CurveAdaptor2D(Crv1), param1);
        var           B         = new BrepBuilder();
        Vertex V;
        double        tolV;
        Vertex V1        = sae.FirstVertex(edge2);
        Point3D        PV1       = Tool.Pnt(V1);
        Vertex V2        = sae.LastVertex(edge2);
        Point3D        PV2       = Tool.Pnt(V2);
        Vertex V11       = sae.FirstVertex(edge1);
        Vertex V12       = sae.LastVertex(edge1);
        bool          NeedSplit = true;
        if (pi1.Distance(PV1) < pi1.Distance(PV2))
        {
            if (V1.IsSame(V11) || V1.IsSame(V12))
            {
                NeedSplit = false;
            }
            V    = V1;
            tolV = Max((pi1.Distance(PV1) / 2) * 1.00001, Tool.Tolerance(V1));
        }
        else
        {
            if (V2.IsSame(V11) || V2.IsSame(V12))
            {
                NeedSplit = false;
            }
            V    = V2;
            tolV = Max((pi1.Distance(PV2) / 2) * 1.00001, Tool.Tolerance(V2));
        }
        if (NeedSplit || aTmpKey)
        {
            if (SplitEdge1(sewd, face, num1, param1, V, tolV, boxes))
            {
                B.UpdateVertex(V, tolV);
                MaxTolVert = Max(MaxTolVert, tolV);
                //      NbSplit++;
                num1--;
                return true;
                // break;
            }
        }
        return false;
    }

    /// <summary>
    /// Fixes the self-intersections of a wire on a face: splits and cuts its edges, merges vertices, removes the
    /// overlapping segments. Works in place on <paramref name="sewd"/> and its edges, recording in the context.
    /// </summary>
    internal bool FixSelfIntersectWire(ShapeExtend_WireData sewd, in Face face, ref int NbSplit, ref int NbCut,
                                       ref int NbRemoved)
    {
        if (myContext is null || face.IsNull)
        {
            return false;
        }

        // double area2d = ShapeAnalysis::TotCross2D(sewd,face);
        // if(area2d<Precision::PConfusion()*Precision::PConfusion()) return false; //gka
        // 06.09.04 BUG 6555

        Shape SF         = Context()!.InternalApply(face);
        double       MaxTolVert = 0.0;
        for (var exp = new Explorer(SF, ShapeType.Vertex); exp.More(); exp.Next())
        {
            double tolV = Tool.Tolerance(ShapeCasts.Vertex(exp.Current()));
            MaxTolVert  = Max(MaxTolVert, tolV);
        }
        MaxTolVert = Min(MaxTolVert, myMaxTol);
        var sae = new ShapeAnalysis_Edge();

        // step 1 : intersection of adjacent edges

        // step 2 : intersection of non-adjacent edges
        var boxes = new NCollection_DataMap<Shape, Box2d>(ShapeHasher.Instance);
        _       = CreateBoxes2d(sewd, face, boxes);
        var sas = new ShapeAnalysis_Surface(Tool.Surface(face));

        NbSplit = 0;
        NbCut   = 0;
        int  nbReplaced = 0;
        bool isDone;
        for (int num1 = 1; num1 < sewd.NbEdges() && NbSplit < 30; num1++)
        {
            // for each edge from first wire
            for (int num2 = num1 + 2; num2 <= sewd.NbEdges() && NbSplit < 30; num2++)
            {
                // for each edge from second wire
                if (num1 == 1 && num2 == sewd.NbEdges())
                {
                    continue;
                }
                Edge edge1 = sewd.Edge(num1);
                Edge edge2 = sewd.Edge(num2);
                if (edge1.IsSame(edge2))
                {
                    continue;
                }
                if (Tool.Degenerated(edge1) || Tool.Degenerated(edge2))
                {
                    continue;
                }
                if (!boxes.IsBound(edge1) || !boxes.IsBound(edge2))
                {
                    continue;
                }
                Box2d B1 = boxes.Find(edge1);
                Box2d B2 = boxes.Find(edge2);
                if (!B1.IsOut(B2))
                {
                    // intersection is possible...
                    if (!sae.PCurve(edge1, face, out Curve2D? Crv1, out double a1, out double b1, false))
                    {
                        return false;
                    }
                    if (!sae.PCurve(edge2, face, out Curve2D? Crv2, out double a2, out double b2, false))
                    {
                        return false;
                    }
                    double tolint = 1.0e-10;
                    var    C1     = new CurveAdaptor2D(Crv1!);
                    var    C2     = new CurveAdaptor2D(Crv2!);
                    var    d1     = new IntRes2d_Domain(C1.Value(a1), a1, tolint, C1.Value(b1), b1, tolint);
                    var    d2     = new IntRes2d_Domain(C2.Value(a2), a2, tolint, C2.Value(b2), b2, tolint);
                    var    Inter  = new Geom2dInt_GInter();
                    Inter.Perform(C1, d1, C2, d2, tolint, tolint);
                    if (!Inter.IsDone())
                    {
                        continue;
                    }
                    // intersection is point
                    if (Inter.NbPoints() > 0 && Inter.NbPoints() < 3)
                    {
                        SelectIntPnt(Inter, out IntRes2d_IntersectionPoint IP, out IntRes2d_Transition Tr1,
                                     out IntRes2d_Transition Tr2);
                        if (Tr1.PositionOnCurve() == IntRes2d_Middle && Tr2.PositionOnCurve() == IntRes2d_Middle)
                        {
                            double        param1 = IP.ParamOnFirst();
                            double        param2 = IP.ParamOnSecond();
                            Point3D        pi1    = GetPointOnEdge(edge1, sas, C1, param1);
                            Point3D        pi2    = GetPointOnEdge(edge2, sas, C2, param2);
                            var           B      = new BrepBuilder();
                            Vertex V      = new();
                            double        tolV   = 0;
                            // analysis for edge1
                            bool          ModifE1 = false;
                            Vertex VF1     = sae.FirstVertex(edge1);
                            Point3D        PVF1    = Tool.Pnt(VF1);
                            Vertex VL1     = sae.LastVertex(edge1);
                            Point3D        PVL1    = Tool.Pnt(VL1);
                            double        dist1   = pi1.Distance(PVF1);
                            double        dist2   = pi1.Distance(PVL1);
                            double        distmin = Min(dist1, dist2);
                            if (dist1 != dist2 && distmin < MaxTolVert)
                            {
                                if (dist1 < dist2)
                                {
                                    tolV = Max(dist1 * 1.00001, Tool.Tolerance(VF1));
                                    B.UpdateVertex(VF1, tolV);
                                    V = VF1;
                                }
                                else
                                {
                                    tolV = Max(dist2 * 1.00001, Tool.Tolerance(VL1));
                                    B.UpdateVertex(VL1, tolV);
                                    V = VL1;
                                }

                                double dista     = Math.Abs(a1 - param1);
                                double distb     = Math.Abs(b1 - param1);
                                bool   IsCutLine = false;
                                ModifE1 = CutEdge(edge1, ((dista > distb) ? a1 : b1), param1, face, ref IsCutLine);
                                if (ModifE1)
                                {
                                    NbCut++;
                                }
                                // not needed split edge, if one of parts is too small
                                ModifE1 = ModifE1 || distmin < global::Kerf.Engine.Foundation.Precision.Confusion();
                            }
                            // analysis for edge2
                            bool          ModifE2 = false;
                            Vertex VF2     = sae.FirstVertex(edge2);
                            Point3D        PVF2    = Tool.Pnt(VF2);
                            Vertex VL2     = sae.LastVertex(edge2);
                            Point3D        PVL2    = Tool.Pnt(VL2);
                            dist1                 = pi2.Distance(PVF2);
                            dist2                 = pi2.Distance(PVL2);
                            distmin               = Min(dist1, dist2);
                            if (dist1 != dist2 && distmin < MaxTolVert)
                            {
                                if (dist1 < dist2)
                                {
                                    tolV = Max(dist1 * 1.00001, Tool.Tolerance(VF2));
                                    B.UpdateVertex(VF2, tolV);
                                    V = VF2;
                                }
                                else
                                {
                                    tolV = Max(dist2 * 1.00001, Tool.Tolerance(VL2));
                                    B.UpdateVertex(VL2, tolV);
                                    V = VL2;
                                }

                                double dista     = Math.Abs(a2 - param2);
                                double distb     = Math.Abs(b2 - param2);
                                bool   IsCutLine = false;
                                ModifE2 = CutEdge(edge2, ((dista > distb) ? a2 : b2), param2, face, ref IsCutLine);
                                if (ModifE2)
                                {
                                    NbCut++;
                                }
                                // not needed split edge, if one of parts is too small
                                ModifE2 = ModifE2 || distmin < global::Kerf.Engine.Foundation.Precision.Confusion();
                            }
                            if (ModifE1 && !ModifE2)
                            {
                                if (SplitEdge1(sewd, face, num2, param2, V, tolV, boxes))
                                {
                                    NbSplit++;
                                    num2--;
                                    continue;
                                }
                            }
                            if (!ModifE1 && ModifE2)
                            {
                                if (SplitEdge1(sewd, face, num1, param1, V, tolV, boxes))
                                {
                                    NbSplit++;
                                    num1--;
                                    break;
                                }
                            }
                            if (!ModifE1 && !ModifE2)
                            {
                                var P0 = new Point3D((pi1.X() + pi2.X()) / 2, (pi1.Y() + pi2.Y()) / 2, (pi1.Z() + pi2.Z()) / 2);
                                tolV = Max((pi1.Distance(pi2) / 2) * 1.00001, global::Kerf.Engine.Foundation.Precision.Confusion());
                                B.MakeVertex(ref V, P0, tolV);
                                MaxTolVert        = Max(MaxTolVert, tolV);
                                bool isEdgeSplit2 = SplitEdge1(sewd, face, num2, param2, V, tolV, boxes);
                                if (isEdgeSplit2)
                                {
                                    NbSplit++;
                                    num2--;
                                }
                                if (SplitEdge1(sewd, face, num1, param1, V, tolV, boxes))
                                {
                                    NbSplit++;
                                    num1--;
                                    break;
                                }
                                if (isEdgeSplit2)
                                {
                                    continue;
                                }
                            }
                        }
                        if (Tr1.PositionOnCurve() == IntRes2d_Middle && Tr2.PositionOnCurve() != IntRes2d_Middle)
                        {
                            // find needed vertex from edge2 and split edge1 using it
                            double param1 = IP.ParamOnFirst();
                            if (FindVertAndSplitEdge(param1, edge1, edge2, Crv1!, ref MaxTolVert, ref num1, sewd, face,
                                                     boxes, false))
                            {
                                NbSplit++;
                                break;
                            }
                        }
                        if (Tr1.PositionOnCurve() != IntRes2d_Middle && Tr2.PositionOnCurve() == IntRes2d_Middle)
                        {
                            // find needed vertex from edge1 and split edge2 using it
                            double param2 = IP.ParamOnSecond();
                            if (FindVertAndSplitEdge(param2, edge2, edge1, Crv2!, ref MaxTolVert, ref num2, sewd, face,
                                                     boxes, false))
                            {
                                NbSplit++;
                                continue;
                            }
                        }
                        if (Tr1.PositionOnCurve() != IntRes2d_Middle && Tr2.PositionOnCurve() != IntRes2d_Middle)
                        {
                            // union vertexes
                            if (UnionVertexes(sewd, ref edge1, ref edge2, num2, boxes, B2))
                            {
                                nbReplaced++; // gka 06.09.04
                            }
                        }
                    }
                    // intersection is segment
                    if (Inter.NbSegments() == 1)
                    {
                        IntRes2d_IntersectionSegment IS = Inter.Segment(1);
                        if (IS.HasFirstPoint() && IS.HasLastPoint())
                        {
                            bool                       IsModified1 = false;
                            bool                       IsModified2 = false;
                            Vertex              NewV        = new();
                            var                        B           = new BrepBuilder();
                            double                     newtol      = 0.0;
                            IntRes2d_IntersectionPoint IPF         = IS.FirstPoint();
                            double                     p11         = IPF.ParamOnFirst();
                            double                     p21         = IPF.ParamOnSecond();
                            IntRes2d_IntersectionPoint IPL         = IS.LastPoint();
                            double                     p12         = IPL.ParamOnFirst();
                            double                     p22         = IPL.ParamOnSecond();
                            Point3D                     Pnt11       = GetPointOnEdge(edge1, sas, C1, p11);
                            Point3D                     Pnt12       = GetPointOnEdge(edge1, sas, C1, p12);
                            Point3D                     Pnt21       = GetPointOnEdge(edge2, sas, C2, p21);
                            Point3D                     Pnt22       = GetPointOnEdge(edge2, sas, C2, p22);
                            // next string commented by skl 29.12.2004 for OCC7624
                            // if( Pnt11.Distance(Pnt21)>myPreci || Pnt12.Distance(Pnt22)>myPreci ) continue;
                            if (Pnt11.Distance(Pnt21) > MaxTolVert || Pnt12.Distance(Pnt22) > MaxTolVert)
                            {
                                continue;
                            }
                            // analysis for edge1
                            Vertex V1  = sae.FirstVertex(edge1);
                            Point3D        PV1 = Tool.Pnt(V1);
                            Vertex V2  = sae.LastVertex(edge1);
                            Point3D        PV2 = Tool.Pnt(V2);
                            double        dist1   = Pnt11.Distance(PV1);
                            double        dist2   = Pnt12.Distance(PV1);
                            double        maxdist = Max(dist1, dist2);
                            double        pdist;
                            if (edge1.Orientation() == Reversed)
                            {
                                pdist = Max(Math.Abs(b1 - p11), Math.Abs(b1 - p12));
                            }
                            else
                            {
                                pdist = Max(Math.Abs(a1 - p11), Math.Abs(a1 - p12));
                            }
                            if (maxdist < MaxTolVert || pdist < Math.Abs(b1 - a1) * 0.01)
                            {
                                newtol      = maxdist;
                                NewV        = V1;
                                IsModified1 = true;
                            }
                            dist1   = Pnt11.Distance(PV2);
                            dist2   = Pnt12.Distance(PV2);
                            maxdist = Max(dist1, dist2);
                            if (edge1.Orientation() == Reversed)
                            {
                                pdist = Max(Math.Abs(a1 - p11), Math.Abs(a1 - p12));
                            }
                            else
                            {
                                pdist = Max(Math.Abs(b1 - p11), Math.Abs(b1 - p12));
                            }
                            if (maxdist < MaxTolVert || pdist < Math.Abs(b1 - a1) * 0.01)
                            {
                                if ((IsModified1 && maxdist < newtol) || !IsModified1)
                                {
                                    newtol      = maxdist;
                                    NewV        = V2;
                                    IsModified1 = true;
                                }
                            }
                            if (IsModified1)
                            {
                                // cut edge1 and update tolerance NewV
                                double dista = Math.Abs(a1 - p11) + Math.Abs(a1 - p12);
                                double distb = Math.Abs(b1 - p11) + Math.Abs(b1 - p12);
                                double pend, cut;
                                if (dista > distb)
                                {
                                    pend = a1;
                                }
                                else
                                {
                                    pend = b1;
                                }
                                if (Math.Abs(pend - p11) > Math.Abs(pend - p12))
                                {
                                    cut = p12;
                                }
                                else
                                {
                                    cut = p11;
                                }
                                bool IsCutLine = false;
                                if (CutEdge(edge1, pend, cut, face, ref IsCutLine))
                                {
                                    NbCut++;
                                }
                                if (newtol > Tool.Tolerance(NewV))
                                {
                                    B.UpdateVertex(NewV, newtol);
                                }
                                else
                                {
                                    newtol = Tool.Tolerance(NewV);
                                }
                            }
                            // analysis for edge2
                            Vertex V12  = sae.FirstVertex(edge2);
                            Point3D        PV12 = Tool.Pnt(V12);
                            Vertex V22  = sae.LastVertex(edge2);
                            Point3D        PV22 = Tool.Pnt(V22);
                            dist1   = Pnt21.Distance(PV12);
                            dist2   = Pnt22.Distance(PV12);
                            maxdist = Max(dist1, dist2);
                            if (edge2.Orientation() == Reversed)
                            {
                                pdist = Max(Math.Abs(b2 - p21), Math.Abs(b2 - p22));
                            }
                            else
                            {
                                pdist = Max(Math.Abs(a2 - p21), Math.Abs(a2 - p22));
                            }
                            if (maxdist < MaxTolVert || pdist < Math.Abs(b2 - a2) * 0.01)
                            {
                                newtol      = maxdist;
                                NewV        = V12;
                                IsModified2 = true;
                            }
                            dist1   = Pnt21.Distance(PV22);
                            dist2   = Pnt22.Distance(PV22);
                            maxdist = Max(dist1, dist2);
                            if (edge2.Orientation() == Reversed)
                            {
                                pdist = Max(Math.Abs(a2 - p21), Math.Abs(a2 - p22));
                            }
                            else
                            {
                                pdist = Max(Math.Abs(b2 - p21), Math.Abs(b2 - p22));
                            }
                            if (maxdist < MaxTolVert || pdist < Math.Abs(b2 - a2) * 0.01)
                            {
                                if ((IsModified2 && maxdist < newtol) || !IsModified2)
                                {
                                    newtol      = maxdist;
                                    NewV        = V22;
                                    IsModified2 = true;
                                }
                            }
                            if (IsModified2)
                            {
                                // cut edge1 and update tolerance NewV
                                double dista = Math.Abs(a2 - p21) + Math.Abs(a2 - p22);
                                double distb = Math.Abs(b2 - p21) + Math.Abs(b2 - p22);
                                double pend, cut;
                                if (dista > distb)
                                {
                                    pend = a2;
                                }
                                else
                                {
                                    pend = b2;
                                }
                                if (Math.Abs(pend - p21) > Math.Abs(pend - p22))
                                {
                                    cut = p22;
                                }
                                else
                                {
                                    cut = p21;
                                }
                                bool IsCutLine = false;
                                if (CutEdge(edge2, pend, cut, face, ref IsCutLine))
                                {
                                    NbCut++;
                                }
                                if (newtol > Tool.Tolerance(NewV))
                                {
                                    B.UpdateVertex(NewV, newtol);
                                }
                                else
                                {
                                    newtol = Tool.Tolerance(NewV);
                                }
                            }

                            if (IsModified1 && !IsModified2)
                            {
                                if (SplitEdge2(sewd, face, num2, p21, p22, NewV, newtol, boxes))
                                {
                                    NbSplit++;
                                    num2--;
                                    continue;
                                }
                            }
                            if (!IsModified1 && IsModified2)
                            {
                                if (SplitEdge2(sewd, face, num1, p11, p12, NewV, newtol, boxes))
                                {
                                    NbSplit++;
                                    num1--;
                                    break;
                                }
                            }
                            if (!IsModified1 && !IsModified2)
                            {
                                double param1 = (p11 + p12) / 2;
                                double param2 = (p21 + p22) / 2;
                                Point3D Pnt10  = GetPointOnEdge(edge1, sas, C1, param1);
                                Point3D Pnt20  = GetPointOnEdge(edge2, sas, C2, param2);
                                var    P0     = new Point3D((Pnt10.X() + Pnt20.X()) / 2,
                                                           (Pnt10.Y() + Pnt20.Y()) / 2,
                                                           (Pnt10.Z() + Pnt20.Z()) / 2);
                                dist1           = Max(Pnt11.Distance(P0), Pnt12.Distance(P0));
                                dist2           = Max(Pnt21.Distance(P0), Pnt22.Distance(P0));
                                double tolV     = Max(dist1, dist2);
                                tolV            = Max(tolV, Pnt10.Distance(Pnt20)) * 1.00001;
                                bool FixSegment = true;
                                if (tolV < MaxTolVert)
                                {
                                    // create new vertex and split each intersecting edge on two edges
                                    B.MakeVertex(ref NewV, Pnt10, tolV);
                                    if (SplitEdge2(sewd, face, num2, p21, p22, NewV, tolV, boxes))
                                    {
                                        NbSplit++;
                                        num2--;
                                    }
                                    if (SplitEdge2(sewd, face, num1, p11, p12, NewV, tolV, boxes))
                                    {
                                        NbSplit++;
                                        num1--;
                                        break;
                                    }
                                }
                                else if (FixSegment)
                                {
                                    //  segment is big and we have to split each intersecting edge
                                    //  on 3 edges --> middle edge - edge based on segment
                                    //  after we can remove edges made from segment
                                    var P01 = new Point3D((Pnt11.X() + Pnt21.X()) / 2,
                                                         (Pnt11.Y() + Pnt21.Y()) / 2,
                                                         (Pnt11.Z() + Pnt21.Z()) / 2);
                                    var P02 = new Point3D((Pnt12.X() + Pnt22.X()) / 2,
                                                         (Pnt12.Y() + Pnt22.Y()) / 2,
                                                         (Pnt12.Z() + Pnt22.Z()) / 2);
                                    double tolV1 = Max(Pnt11.Distance(P01), Pnt21.Distance(P01));
                                    tolV1        = Max(tolV1, global::Kerf.Engine.Foundation.Precision.Confusion()) * 1.00001;
                                    double tolV2 = Max(Pnt12.Distance(P02), Pnt22.Distance(P02));
                                    tolV2        = Max(tolV2, global::Kerf.Engine.Foundation.Precision.Confusion()) * 1.00001;
                                    if (tolV1 > MaxTolVert || tolV2 > MaxTolVert)
                                    {
                                        continue;
                                    }
                                    Vertex NewV1 = new(), NewV2 = new();
                                    Edge   tmpE;
                                    // split edge1
                                    int    akey1 = 0, akey2 = 0;
                                    double newTolerance;
                                    // analysis fo P01
                                    newTolerance = Max(tolV1, Tool.Tolerance(V1));
                                    if (P01.Distance(PV1) < newTolerance)
                                    {
                                        B.MakeVertex(ref NewV1, Tool.Pnt(V1), newTolerance);
                                        NewV1.Orientation(V1.Orientation());
                                        akey1++;
                                    }
                                    newTolerance = Max(tolV1, Tool.Tolerance(V2));
                                    if (P01.Distance(PV2) < newTolerance)
                                    {
                                        B.MakeVertex(ref NewV1, Tool.Pnt(V2), newTolerance);
                                        NewV1.Orientation(V2.Orientation());
                                        akey1++;
                                    }
                                    // analysis fo P02
                                    newTolerance = Max(tolV2, Tool.Tolerance(V1));
                                    if (P02.Distance(PV1) < newTolerance)
                                    {
                                        B.MakeVertex(ref NewV2, Tool.Pnt(V1), newTolerance);
                                        NewV2.Orientation(V1.Orientation());
                                        akey2++;
                                    }
                                    newTolerance = Max(tolV2, Tool.Tolerance(V2));
                                    if (P02.Distance(PV2) < newTolerance)
                                    {
                                        B.MakeVertex(ref NewV2, Tool.Pnt(V2), newTolerance);
                                        NewV2.Orientation(V2.Orientation());
                                        akey2++;
                                    }
                                    if (akey1 > 1 || akey2 > 1)
                                    {
                                        continue;
                                    }
                                    int dnum1 = 0, numseg1 = num1;
                                    // prepare vertices
                                    if (akey1 == 0)
                                    {
                                        B.MakeVertex(ref NewV1, P01, tolV1);
                                    }
                                    if (akey2 == 0)
                                    {
                                        B.MakeVertex(ref NewV2, P02, tolV2);
                                    }
                                    // split
                                    if (akey1 == 0 && akey2 > 0)
                                    {
                                        if (SplitEdge1(sewd, face, num1, p11, NewV1, tolV1, boxes))
                                        {
                                            NbSplit++;
                                            dnum1   = 1;
                                            numseg1 = num1 + 1;
                                        }
                                    }
                                    if (akey1 > 0 && akey2 == 0)
                                    {
                                        if (SplitEdge1(sewd, face, num1, p12, NewV2, tolV2, boxes))
                                        {
                                            NbSplit++;
                                            dnum1   = 1;
                                            numseg1 = num1;
                                        }
                                    }
                                    if (akey1 == 0 && akey2 == 0)
                                    {
                                        if (SplitEdge1(sewd, face, num1, p11, NewV1, tolV1, boxes))
                                        {
                                            NbSplit++;
                                            dnum1 = 1;
                                        }
                                        tmpE = sewd.Edge(num1);
                                        sae.PCurve(tmpE, face, out _, out double a, out double b, false);
                                        if ((a - p12) * (b - p12) > 0)
                                        { // p12 - external for [a,b] => split next edge
                                            if (SplitEdge1(sewd, face, num1 + 1, p12, NewV2, tolV2, boxes))
                                            {
                                                NbSplit++;
                                                dnum1++;
                                                numseg1 = num1 + 1;
                                            }
                                        }
                                        else
                                        {
                                            if (SplitEdge1(sewd, face, num1, p12, NewV2, tolV2, boxes))
                                            {
                                                NbSplit++;
                                                dnum1++;
                                                numseg1 = num1 + 1;
                                            }
                                        }
                                    }
                                    // SegE = sewd->Edge(numseg1); // get edge from segment
                                    //  split edge2
                                    //  replace vertices if it is necessary
                                    var sbe = new ShapeBuild_Edge();
                                    akey1 = 0;
                                    akey2 = 0;

                                    if (P01.Distance(PV12) < tolV1)
                                    {
                                        tolV1 += P01.Distance(PV12);
                                        B.UpdateVertex(NewV1, tolV1);
                                        if (V12.Orientation() == NewV1.Orientation())
                                        {
                                            myContext.Replace(V12, NewV1);
                                            V12 = NewV1;
                                        }
                                        else
                                        {
                                            myContext.Replace(V12, NewV1.Reversed());
                                            V12 = ShapeCasts.Vertex(NewV1.Reversed());
                                        }
                                        nbReplaced++; // gka 06.09.04
                                        Edge NewE = sbe.InternalCopyReplaceVertices(edge2, NewV1, V22);
                                        myContext.Replace(edge2, NewE);
                                        sewd.Set(NewE, num2 + dnum1);
                                        boxes.Bind(NewE, B2); // update boxes
                                        edge2 = NewE;
                                        akey1 = 1;
                                    }
                                    if (P01.Distance(PV22) < tolV1)
                                    {
                                        tolV1 += P01.Distance(PV22);
                                        B.UpdateVertex(NewV1, tolV1);
                                        if (V22.Orientation() == NewV1.Orientation())
                                        {
                                            myContext.Replace(V22, NewV1);
                                            V22 = NewV1;
                                        }
                                        else
                                        {
                                            myContext.Replace(V22, NewV1.Reversed());
                                            V22 = ShapeCasts.Vertex(NewV1.Reversed());
                                        }
                                        nbReplaced++; // gka 06.09.04
                                        Edge NewE = sbe.InternalCopyReplaceVertices(edge2, V12, NewV1);
                                        myContext.Replace(edge2, NewE);
                                        sewd.Set(NewE, num2 + dnum1);
                                        boxes.Bind(NewE, B2); // update boxes
                                        edge2 = NewE;
                                        akey1 = 2;
                                    }
                                    if (P02.Distance(PV12) < tolV2)
                                    {
                                        tolV2 += P02.Distance(PV12);
                                        B.UpdateVertex(NewV2, tolV2);
                                        if (V12.Orientation() == NewV2.Orientation())
                                        {
                                            myContext.Replace(V12, NewV2);
                                            V12 = NewV2;
                                        }
                                        else
                                        {
                                            myContext.Replace(V12, NewV2.Reversed());
                                            V12 = ShapeCasts.Vertex(NewV2.Reversed());
                                        }
                                        nbReplaced++; // gka 06.09.04
                                        Edge NewE = sbe.InternalCopyReplaceVertices(edge2, NewV2, V22);
                                        myContext.Replace(edge2, NewE);
                                        sewd.Set(NewE, num2 + dnum1);
                                        boxes.Bind(NewE, B2); // update boxes
                                        edge2 = NewE;
                                        akey2 = 1;
                                    }
                                    if (P02.Distance(PV22) < tolV2)
                                    {
                                        tolV2 += P02.Distance(PV22);
                                        B.UpdateVertex(NewV2, tolV2);
                                        if (V22.Orientation() == NewV2.Orientation())
                                        {
                                            myContext.Replace(V22, NewV2);
                                            V22 = NewV2;
                                        }
                                        else
                                        {
                                            myContext.Replace(V22, NewV2.Reversed());
                                            V22 = ShapeCasts.Vertex(NewV2.Reversed());
                                        }
                                        nbReplaced++; // gka 06.09.04
                                        Edge NewE = sbe.InternalCopyReplaceVertices(edge2, V12, NewV2);
                                        myContext.Replace(edge2, NewE);
                                        sewd.Set(NewE, num2 + dnum1);
                                        boxes.Bind(NewE, B2); // update boxes
                                        edge2 = NewE;
                                        akey2 = 2;
                                    }
                                    int dnum2 = 0, numseg2 = num2 + dnum1;
                                    // split
                                    if (akey1 == 0 && akey2 > 0)
                                    {
                                        if (SplitEdge1(sewd, face, num2 + dnum1, p21, NewV1, tolV1, boxes))
                                        {
                                            NbSplit++;
                                            dnum2 = 1;
                                            // numseg2=num2+dnum1+1;
                                            numseg2 = num2 + dnum1;
                                        }
                                    }
                                    if (akey1 > 0 && akey2 == 0)
                                    {
                                        if (SplitEdge1(sewd, face, num2 + dnum1, p22, NewV2, tolV2, boxes))
                                        {
                                            NbSplit++;
                                            dnum2 = 1;
                                            // numseg2=num2+dnum1;
                                            numseg2 = num2 + dnum1 + 1;
                                        }
                                    }
                                    if (akey1 == 0 && akey2 == 0)
                                    {
                                        if (SplitEdge1(sewd, face, num2 + dnum1, p21, NewV1, tolV1, boxes))
                                        {
                                            NbSplit++;
                                            dnum2 = 1;
                                        }
                                        tmpE = sewd.Edge(num2 + dnum1);
                                        sae.PCurve(tmpE, face, out _, out double a, out double b, false);
                                        if ((a - p22) * (b - p22) > 0)
                                        { // p22 - external for [a,b] => split next edge
                                            if (SplitEdge1(sewd, face, num2 + dnum1 + dnum2, p22, NewV2, tolV2, boxes))
                                            {
                                                NbSplit++;
                                                numseg2 = num2 + dnum1 + dnum2;
                                                dnum2++;
                                            }
                                        }
                                        else
                                        {
                                            if (SplitEdge1(sewd, face, num2 + dnum1, p22, NewV2, tolV2, boxes))
                                            {
                                                NbSplit++;
                                                dnum2++;
                                                numseg2 = num2 + dnum1 + 1;
                                            }
                                        }
                                    }
                                    // remove segment
                                    sewd.Remove(numseg2);
                                    sewd.Remove(numseg1);
                                    NbRemoved = NbRemoved + 2; // added to the caller's count (not reset here, as in OCCT)
                                    // num1--;
                                    // break;
                                }
                            }
                        }
                    }
                }
            }
        }
        isDone = (NbSplit != 0 || NbCut != 0 || nbReplaced != 0 || NbRemoved != 0);
        return isDone;
    }

    /// <summary>
    /// Fixes the intersections between the wires of a face: splits and cuts their edges, and replaces the face (in
    /// the context and in <paramref name="face"/>) by a new face holding the fixed wires. Works in place on the face's
    /// edges and vertices.
    /// </summary>
    internal bool FixIntersectingWires(ref Face face)
    {
        if (myContext is null || face.IsNull)
        {
            return false;
        }
        // TopoDS_Shape S = context->Apply(face);
        // TopoDS_Shape SF = TopoDS::Face(S);
        Shape       SF          = face;
        Orientation ori         = face.Orientation();
        var                SeqWir      = new NCollection_Sequence<Shape>();
        var                SeqNMShapes = new NCollection_Sequence<Shape>();
        for (var iter = new Iterator(SF, false); iter.More(); iter.Next())
        {
            if (iter.Value().ShapeType() != ShapeType.Wire
                || (iter.Value().Orientation() != Forward && iter.Value().Orientation() != Reversed))
            {
                SeqNMShapes.Append(iter.Value());
                continue;
            }
            Wire wire = ShapeCasts.Wire(iter.Value());
            SeqWir.Append(wire);
        }
        if (SeqWir.Length() < 2)
        {
            return false; // gka 06.09.04
        }

        double MaxTolVert = 0.0;
        for (var exp = new Explorer(SF, ShapeType.Vertex); exp.More(); exp.Next())
        {
            double tolV = Tool.Tolerance(ShapeCasts.Vertex(exp.Current()));
            MaxTolVert  = Max(MaxTolVert, tolV);
        }
        bool isDone = false; // gka 06.09.04
        var  sae    = new ShapeAnalysis_Edge();
        var  sas    = new ShapeAnalysis_Surface(Tool.Surface(face));

        // precompute edge boxes for all wires
        var aSeqWirEdgeBoxes = new NCollection_Sequence<NCollection_DataMap<Shape, Box2d>>();
        var aSeqWirBoxes     = new NCollection_Sequence<Box2d>();
        for (int n = 1; n <= SeqWir.Length(); n++)
        {
            Wire aWire  = ShapeCasts.Wire(SeqWir.Value(n));
            var         aSewd  = new ShapeExtend_WireData(aWire);
            var         aBoxes = new NCollection_DataMap<Shape, Box2d>(ShapeHasher.Instance);
            Box2d   aTotalBox = CreateBoxes2d(aSewd, face, aBoxes);
            aSeqWirEdgeBoxes.Append(aBoxes);
            aSeqWirBoxes.Append(aTotalBox);
        }

        for (int n1 = 1; n1 <= SeqWir.Length() - 1; n1++)
        {
            Wire wire1 = ShapeCasts.Wire(SeqWir.Value(n1));
            var         sewd1 = new ShapeExtend_WireData(wire1);
            NCollection_DataMap<Shape, Box2d> boxes1 = aSeqWirEdgeBoxes.ChangeValue(n1);
            Box2d aBox1 = aSeqWirBoxes[n1];
            for (int n2 = n1 + 1; n2 <= SeqWir.Length(); n2++)
            {
                Wire wire2 = ShapeCasts.Wire(SeqWir.Value(n2));
                var         sewd2 = new ShapeExtend_WireData(wire2);
                NCollection_DataMap<Shape, Box2d> boxes2 = aSeqWirEdgeBoxes.ChangeValue(n2);
                Box2d aBox2 = aSeqWirBoxes[n2];
                if (!aBox1.IsVoid && !aBox2.IsVoid && aBox1.IsOut(aBox2))
                {
                    continue;
                }
                // detect possible intersections:
                int  NbModif      = 0;
                int  nbReplaced   = 0;     // gka 06.09.04
                bool hasModifWire = false; // gka 06.09.04
                for (int num1 = 1; num1 <= sewd1.NbEdges() && NbModif < 30; num1++)
                {
                    // for each edge from first wire
                    Edge edge1 = sewd1.Edge(num1); // gka 06.09.04

                    for (int num2 = 1; num2 <= sewd2.NbEdges() && NbModif < 30; num2++)
                    {
                        // for each edge from second wire
                        Edge edge2 = sewd2.Edge(num2);
                        if (edge1.IsSame(edge2))
                        {
                            continue;
                        }
                        if (Tool.Degenerated(edge1) || Tool.Degenerated(edge2))
                        {
                            continue;
                        }
                        if (!boxes1.IsBound(edge1) || !boxes2.IsBound(edge2))
                        {
                            continue;
                        }
                        Box2d B1 = boxes1.Find(edge1);
                        Box2d B2 = boxes2.Find(edge2);
                        if (!B1.IsOut(B2))
                        {
                            // intersection is possible...
                            if (!sae.PCurve(edge1, face, out Curve2D? Crv1, out double a1, out double b1, false))
                            {
                                continue; // return false; gka 06.09.04
                            }
                            if (!sae.PCurve(edge2, face, out Curve2D? Crv2, out double a2, out double b2, false))
                            {
                                continue; // return false;gka 06.09.04
                            }
                            double tolint = 1.0e-10;
                            var    C1     = new CurveAdaptor2D(Crv1!);
                            var    C2     = new CurveAdaptor2D(Crv2!);
                            var    d1     = new IntRes2d_Domain(C1.Value(a1), a1, tolint, C1.Value(b1), b1, tolint);
                            var    d2     = new IntRes2d_Domain(C2.Value(a2), a2, tolint, C2.Value(b2), b2, tolint);
                            var    Inter  = new Geom2dInt_GInter();
                            Inter.Perform(C1, d1, C2, d2, tolint, tolint);
                            if (!Inter.IsDone())
                            {
                                continue;
                            }
                            // intersection is point
                            if (Inter.NbPoints() > 0 && Inter.NbPoints() < 3)
                            {
                                SelectIntPnt(Inter, out IntRes2d_IntersectionPoint IP, out IntRes2d_Transition Tr1,
                                             out IntRes2d_Transition Tr2);
                                if (Tr1.PositionOnCurve() == IntRes2d_Middle && Tr2.PositionOnCurve() == IntRes2d_Middle)
                                {
                                    // create new vertex and split both edges
                                    double        param1 = IP.ParamOnFirst();
                                    double        param2 = IP.ParamOnSecond();
                                    Point3D        pi1    = GetPointOnEdge(edge1, sas, C1, param1);
                                    Point3D        pi2    = GetPointOnEdge(edge2, sas, C2, param2);
                                    var           P0     = new Point3D((pi1.X() + pi2.X()) / 2,
                                                                      (pi1.Y() + pi2.Y()) / 2,
                                                                      (pi1.Z() + pi2.Z()) / 2);
                                    var           B      = new BrepBuilder();
                                    Vertex V      = new();
                                    double tolV = Max((pi1.Distance(pi2) / 2) * 1.00001, global::Kerf.Engine.Foundation.Precision.Confusion());
                                    B.MakeVertex(ref V, P0, tolV);
                                    MaxTolVert        = Max(MaxTolVert, tolV);
                                    bool isSplitEdge2 = SplitEdge1(sewd2, face, num2, param2, V, tolV, boxes2);
                                    if (isSplitEdge2)
                                    {
                                        NbModif++;
                                        num2--;
                                    }
                                    if (SplitEdge1(sewd1, face, num1, param1, V, tolV, boxes1))
                                    {
                                        NbModif++;
                                        num1--;
                                        break;
                                    }
                                    if (isSplitEdge2)
                                    {
                                        continue;
                                    }
                                }
                                if (Tr1.PositionOnCurve() == IntRes2d_Middle && Tr2.PositionOnCurve() != IntRes2d_Middle)
                                {
                                    // find needed vertex from edge2 and split edge1 using it
                                    double param1 = IP.ParamOnFirst();
                                    if (FindVertAndSplitEdge(param1, edge1, edge2, Crv1!, ref MaxTolVert, ref num1, sewd1,
                                                             face, boxes1, true))
                                    {
                                        NbModif++;
                                        break;
                                    }
                                }
                                if (Tr1.PositionOnCurve() != IntRes2d_Middle && Tr2.PositionOnCurve() == IntRes2d_Middle)
                                {
                                    // find needed vertex from edge1 and split edge2 using it
                                    double param2 = IP.ParamOnSecond();
                                    if (FindVertAndSplitEdge(param2, edge2, edge1, Crv2!, ref MaxTolVert, ref num2, sewd2,
                                                             face, boxes2, true))
                                    {
                                        NbModif++;
                                        continue;
                                    }
                                }
                                if (Tr1.PositionOnCurve() != IntRes2d_Middle && Tr2.PositionOnCurve() != IntRes2d_Middle)
                                {
                                    // union vertexes
                                    if (UnionVertexes(sewd2, ref edge1, ref edge2, num2, boxes2, B2))
                                    {
                                        nbReplaced++; // gka 06.09.04
                                    }
                                }
                            }
                            hasModifWire = (hasModifWire || NbModif != 0 || nbReplaced != 0);
                            // intersection is segment
                            if (Inter.NbSegments() == 1)
                            {
                                IntRes2d_IntersectionSegment IS = Inter.Segment(1);
                                if (IS.HasFirstPoint() && IS.HasLastPoint())
                                {
                                    bool                       IsModified1 = false;
                                    bool                       IsModified2 = false;
                                    Vertex              NewV        = new();
                                    var                        B           = new BrepBuilder();
                                    double                     newtol      = 0.0;
                                    IntRes2d_IntersectionPoint IPF         = IS.FirstPoint();
                                    double                     p11         = IPF.ParamOnFirst();
                                    double                     p21         = IPF.ParamOnSecond();
                                    IntRes2d_IntersectionPoint IPL         = IS.LastPoint();
                                    double                     p12         = IPL.ParamOnFirst();
                                    double                     p22         = IPL.ParamOnSecond();
                                    Point3D                     Pnt11       = GetPointOnEdge(edge1, sas, C1, p11);
                                    Point3D                     Pnt12       = GetPointOnEdge(edge1, sas, C1, p12);
                                    Point3D                     Pnt21       = GetPointOnEdge(edge2, sas, C2, p21);
                                    Point3D                     Pnt22       = GetPointOnEdge(edge2, sas, C2, p22);

                                    // analysis for edge1
                                    Vertex V1      = sae.FirstVertex(edge1);
                                    Point3D        PV1     = Tool.Pnt(V1);
                                    Vertex V2      = sae.LastVertex(edge1);
                                    Point3D        PV2     = Tool.Pnt(V2);
                                    double        dist1   = Pnt11.Distance(PV1);
                                    double        dist2   = Pnt12.Distance(PV1);
                                    double        maxdist = Max(dist1, dist2);
                                    double        pdist;
                                    if (edge1.Orientation() == Reversed)
                                    {
                                        pdist = Max(Math.Abs(b1 - p11), Math.Abs(b1 - p12));
                                    }
                                    else
                                    {
                                        pdist = Max(Math.Abs(a1 - p11), Math.Abs(a1 - p12));
                                    }
                                    if (maxdist < MaxTolVert || pdist < Math.Abs(b1 - a1) * 0.01)
                                    {
                                        newtol      = maxdist;
                                        NewV        = V1;
                                        IsModified1 = true;
                                    }
                                    dist1   = Pnt11.Distance(PV2);
                                    dist2   = Pnt12.Distance(PV2);
                                    maxdist = Max(dist1, dist2);
                                    if (edge1.Orientation() == Reversed)
                                    {
                                        pdist = Max(Math.Abs(a1 - p11), Math.Abs(a1 - p12));
                                    }
                                    else
                                    {
                                        pdist = Max(Math.Abs(b1 - p11), Math.Abs(b1 - p12));
                                    }
                                    if (maxdist < MaxTolVert || pdist < Math.Abs(b1 - a1) * 0.01)
                                    {
                                        if ((IsModified1 && maxdist < newtol) || !IsModified1)
                                        {
                                            newtol      = maxdist;
                                            NewV        = V2;
                                            IsModified1 = true;
                                        }
                                    }
                                    if (IsModified1)
                                    {
                                        // cut edge1 and update tolerance NewV
                                        double dista = Math.Abs(a1 - p11) + Math.Abs(a1 - p12);
                                        double distb = Math.Abs(b1 - p11) + Math.Abs(b1 - p12);
                                        double pend, cut;
                                        if (dista > distb)
                                        {
                                            pend = a1;
                                        }
                                        else
                                        {
                                            pend = b1;
                                        }
                                        if (Math.Abs(pend - p11) > Math.Abs(pend - p12))
                                        {
                                            cut = p12;
                                        }
                                        else
                                        {
                                            cut = p11;
                                        }
                                        bool IsCutLine = false;
                                        if (!CutEdge(edge1, pend, cut, face, ref IsCutLine))
                                        {
                                            IsModified1 = false;
                                            continue;
                                        }
                                        if (newtol > Tool.Tolerance(NewV))
                                        {
                                            B.UpdateVertex(NewV, newtol * 1.00001);
                                        }
                                    }

                                    // analysis for edge2
                                    Vertex V12  = sae.FirstVertex(edge2);
                                    Point3D        PV12 = Tool.Pnt(V12);
                                    Vertex V22  = sae.LastVertex(edge2);
                                    Point3D        PV22 = Tool.Pnt(V22);
                                    dist1   = Pnt21.Distance(PV12);
                                    dist2   = Pnt22.Distance(PV12);
                                    maxdist = Max(dist1, dist2);
                                    if (edge2.Orientation() == Reversed)
                                    {
                                        pdist = Max(Math.Abs(b2 - p21), Math.Abs(b2 - p22));
                                    }
                                    else
                                    {
                                        pdist = Max(Math.Abs(a2 - p21), Math.Abs(a2 - p22));
                                    }
                                    if (maxdist < MaxTolVert || pdist < Math.Abs(b2 - a2) * 0.01)
                                    {
                                        newtol      = maxdist;
                                        NewV        = V12;
                                        IsModified2 = true;
                                    }
                                    dist1   = Pnt21.Distance(PV22);
                                    dist2   = Pnt22.Distance(PV22);
                                    maxdist = Max(dist1, dist2);
                                    if (edge2.Orientation() == Reversed)
                                    {
                                        pdist = Max(Math.Abs(a2 - p21), Math.Abs(a2 - p22));
                                    }
                                    else
                                    {
                                        pdist = Max(Math.Abs(b2 - p21), Math.Abs(b2 - p22));
                                    }
                                    if (maxdist < MaxTolVert || pdist < Math.Abs(b2 - a2) * 0.01)
                                    {
                                        if ((IsModified2 && maxdist < newtol) || !IsModified2)
                                        {
                                            newtol      = maxdist;
                                            NewV        = V22;
                                            IsModified2 = true;
                                        }
                                    }
                                    if (IsModified2)
                                    {
                                        // cut edge1 and update tolerance NewV
                                        double dista = Math.Abs(a2 - p21) + Math.Abs(a2 - p22);
                                        double distb = Math.Abs(b2 - p21) + Math.Abs(b2 - p22);
                                        double pend, cut;
                                        if (dista > distb)
                                        {
                                            pend = a2;
                                        }
                                        else
                                        {
                                            pend = b2;
                                        }
                                        if (Math.Abs(pend - p21) > Math.Abs(pend - p22))
                                        {
                                            cut = p22;
                                        }
                                        else
                                        {
                                            cut = p21;
                                        }
                                        bool IsCutLine = false;
                                        if (!CutEdge(edge2, pend, cut, face, ref IsCutLine))
                                        {
                                            IsModified2 = false;
                                            continue;
                                        }
                                        if (newtol > Tool.Tolerance(NewV))
                                        {
                                            B.UpdateVertex(NewV, newtol * 1.00001);
                                        }
                                    }

                                    if (IsModified1 || IsModified2)
                                    {
                                        // necessary to make intersect with the same pair of the edges once again with
                                        // modified ranges
                                        num2--;
                                        hasModifWire = true; // gka 06.09.04
                                        continue;
                                    }
                                    else
                                    {
                                        // create new vertex and split edge1 and edge2 using it
                                        if (Math.Abs(p12 - p11) > Math.Abs(b1 - a1) / 2
                                            || Math.Abs(p22 - p21) > Math.Abs(b2 - a2) / 2)
                                        {
                                            // segment is big and we have to split each intersecting edge
                                            // on 3 edges --> middle edge - edge based on segment
                                            var P01 = new Point3D((Pnt11.X() + Pnt21.X()) / 2,
                                                                 (Pnt11.Y() + Pnt21.Y()) / 2,
                                                                 (Pnt11.Z() + Pnt21.Z()) / 2);
                                            var P02 = new Point3D((Pnt12.X() + Pnt22.X()) / 2,
                                                                 (Pnt12.Y() + Pnt22.Y()) / 2,
                                                                 (Pnt12.Z() + Pnt22.Z()) / 2);
                                            double tolV1 = Max(Pnt11.Distance(P01), Pnt21.Distance(P01));
                                            tolV1        = Max(tolV1, global::Kerf.Engine.Foundation.Precision.Confusion()) * 1.00001;
                                            double tolV2 = Max(Pnt12.Distance(P02), Pnt22.Distance(P02));
                                            tolV2        = Max(tolV2, global::Kerf.Engine.Foundation.Precision.Confusion()) * 1.00001;
                                            if (tolV1 > MaxTolVert || tolV2 > MaxTolVert)
                                            {
                                                continue;
                                            }

                                            hasModifWire = true; // gka 06.09.04
                                            Vertex NewV1 = new(), NewV2 = new();
                                            Edge   tmpE, SegE;
                                            // split edge1
                                            int akey1 = 0, akey2 = 0;
                                            // analysis fo P01
                                            if (P01.Distance(PV1) < Max(tolV1, Tool.Tolerance(V1)))
                                            {
                                                NewV1 = V1;
                                                if (tolV1 > Tool.Tolerance(V1))
                                                {
                                                    B.UpdateVertex(NewV1, tolV1);
                                                }
                                                akey1++;
                                            }
                                            if (P01.Distance(PV2) < Max(tolV1, Tool.Tolerance(V2)))
                                            {
                                                NewV1 = V2;
                                                if (tolV1 > Tool.Tolerance(V2))
                                                {
                                                    B.UpdateVertex(NewV1, tolV1);
                                                }
                                                akey1++;
                                            }
                                            // analysis fo P02
                                            if (P02.Distance(PV1) < Max(tolV2, Tool.Tolerance(V1)))
                                            {
                                                NewV2 = V1;
                                                if (tolV2 > Tool.Tolerance(V1))
                                                {
                                                    B.UpdateVertex(NewV2, tolV2);
                                                }
                                                akey2++;
                                            }
                                            if (P02.Distance(PV2) < Max(tolV2, Tool.Tolerance(V2)))
                                            {
                                                NewV2 = V2;
                                                if (tolV2 > Tool.Tolerance(V2))
                                                {
                                                    B.UpdateVertex(NewV2, tolV2);
                                                }
                                                akey2++;
                                            }
                                            if (akey1 > 1 || akey2 > 1)
                                            {
                                                continue;
                                            }
                                            // prepare vertices
                                            if (akey1 == 0)
                                            {
                                                B.MakeVertex(ref NewV1, P01, tolV1);
                                            }
                                            if (akey2 == 0)
                                            {
                                                B.MakeVertex(ref NewV2, P02, tolV2);
                                            }
                                            // split
                                            int numseg1 = num1;
                                            if (akey1 == 0 && akey2 > 0)
                                            {
                                                if (SplitEdge1(sewd1, face, num1, p11, NewV1, tolV1, boxes1))
                                                {
                                                    NbModif++;
                                                    numseg1 = num1 + 1;
                                                }
                                            }
                                            if (akey1 > 0 && akey2 == 0)
                                            {
                                                if (SplitEdge1(sewd1, face, num1, p12, NewV2, tolV2, boxes1))
                                                {
                                                    NbModif++;
                                                    numseg1 = num1;
                                                }
                                            }
                                            if (akey1 == 0 && akey2 == 0)
                                            {
                                                int num1split2 = num1; // what edge to split by point 2
                                                if (SplitEdge1(sewd1, face, num1, p11, NewV1, tolV1, boxes1))
                                                {
                                                    NbModif++;
                                                    tmpE = sewd1.Edge(num1);
                                                    sae.PCurve(tmpE, face, out _, out double a, out double b, false);
                                                    if ((a - p12) * (b - p12) > 0)
                                                    { // p12 - external for [a,b] => split next edge
                                                        num1split2++;
                                                    }
                                                }
                                                if (SplitEdge1(sewd1, face, num1split2, p12, NewV2, tolV2, boxes1))
                                                {
                                                    NbModif++;
                                                    numseg1 = num1 + 1;
                                                }
                                            }
                                            SegE = sewd1.Edge(numseg1); // get edge from segment
                                            // split edge2
                                            // replace vertices if it is necessary
                                            var sbe = new ShapeBuild_Edge();
                                            akey1 = 0;
                                            akey2 = 0;
                                            if (P01.Distance(PV12) < tolV1)
                                            {
                                                tolV1 += P01.Distance(PV12);
                                                B.UpdateVertex(NewV1, tolV1);
                                                V12              = NewV1;
                                                Edge NewE = sbe.InternalCopyReplaceVertices(edge2, NewV1, V22);
                                                myContext.Replace(edge2, NewE);
                                                sewd2.Set(NewE, num2);
                                                boxes2.Bind(NewE, B2); // update boxes2
                                                edge2 = NewE;
                                                akey1 = 1;
                                            }
                                            if (P01.Distance(PV22) < tolV1)
                                            {
                                                tolV1 += P01.Distance(PV22);
                                                B.UpdateVertex(NewV1, tolV1);
                                                V22              = NewV1;
                                                Edge NewE = sbe.InternalCopyReplaceVertices(edge2, V12, NewV1);
                                                myContext.Replace(edge2, NewE);
                                                sewd2.Set(NewE, num2);
                                                boxes2.Bind(NewE, B2); // update boxes2
                                                edge2 = NewE;
                                                akey1 = 2;
                                            }
                                            if (P02.Distance(PV12) < tolV2)
                                            {
                                                tolV2 += P02.Distance(PV12);
                                                B.UpdateVertex(NewV2, tolV2);
                                                V12              = NewV2;
                                                Edge NewE = sbe.InternalCopyReplaceVertices(edge2, NewV2, V22);
                                                myContext.Replace(edge2, NewE);
                                                sewd2.Set(NewE, num2);
                                                boxes2.Bind(NewE, B2); // update boxes2
                                                edge2 = NewE;
                                                akey2 = 1;
                                            }
                                            if (P02.Distance(PV22) < tolV2)
                                            {
                                                tolV2 += P02.Distance(PV22);
                                                B.UpdateVertex(NewV2, tolV2);
                                                V22              = NewV2;
                                                Edge NewE = sbe.InternalCopyReplaceVertices(edge2, V12, NewV2);
                                                myContext.Replace(edge2, NewE);
                                                sewd2.Set(NewE, num2);
                                                boxes2.Bind(NewE, B2); // update boxes2
                                                edge2 = NewE;
                                                akey2 = 2;
                                            }
                                            // split
                                            int numseg2 = num2;
                                            if (akey1 == 0 && akey2 > 0)
                                            {
                                                if (SplitEdge1(sewd2, face, num2, p21, NewV1, tolV1, boxes2))
                                                {
                                                    NbModif++;
                                                    numseg2 = num2 + 1;
                                                }
                                            }
                                            if (akey1 > 0 && akey2 == 0)
                                            {
                                                if (SplitEdge1(sewd2, face, num2, p22, NewV2, tolV2, boxes2))
                                                {
                                                    NbModif++;
                                                    numseg2 = num2;
                                                }
                                            }
                                            if (akey1 == 0 && akey2 == 0)
                                            {
                                                int num2split2 = num2;
                                                if (SplitEdge1(sewd2, face, num2, p21, NewV1, tolV1, boxes2))
                                                {
                                                    NbModif++;
                                                    numseg2 = num2 + 1;
                                                    tmpE    = sewd2.Edge(num2);
                                                    sae.PCurve(tmpE, face, out _, out double a, out double b, false);
                                                    if ((a - p22) * (b - p22) > 0)
                                                    { // p22 - external for [a,b] => split next edge
                                                        num2split2++;
                                                    }
                                                }
                                                if (SplitEdge1(sewd2, face, num2split2, p22, NewV2, tolV2, boxes2))
                                                {
                                                    NbModif++;
                                                    numseg2 = num2 + 1;
                                                }
                                            }
                                            tmpE = sewd2.Edge(numseg2);
                                            boxes2.Bind(tmpE, boxes1.Find(SegE)); // update boxes2
                                            if (!sae.FirstVertex(SegE).IsSame(sae.FirstVertex(tmpE)))
                                            {
                                                SegE.Reverse();
                                            }
                                            myContext.Replace(tmpE, SegE);
                                            sewd2.Set(SegE, numseg2);
                                            num1--;
                                            break;
                                        }
                                        else
                                        {
                                            // split each intersecting edge on two edges
                                            var P0 = new Point3D((Pnt11.X() + Pnt12.X()) / 2,
                                                                (Pnt11.Y() + Pnt12.Y()) / 2,
                                                                (Pnt11.Z() + Pnt12.Z()) / 2);
                                            double param1 = (p11 + p12) / 2;
                                            double param2 = (p21 + p22) / 2;
                                            Point3D Pnt10  = GetPointOnEdge(edge1, sas, C1, param1);
                                            Point3D Pnt20  = GetPointOnEdge(edge2, sas, C2, param2);
                                            dist1         = Max(Pnt11.Distance(P0), Pnt12.Distance(Pnt10));
                                            dist2         = Max(Pnt21.Distance(P0), Pnt22.Distance(Pnt10));
                                            double tolV   = Max(dist1, dist2);
                                            tolV          = Max(tolV, Pnt10.Distance(Pnt20)) * 1.00001;
                                            B.MakeVertex(ref NewV, Pnt10, tolV);
                                            MaxTolVert   = Max(MaxTolVert, tolV);
                                            hasModifWire = true;
                                            if (SplitEdge2(sewd2, face, num2, p21, p22, NewV, tolV, boxes2))
                                            {
                                                NbModif++;
                                                num2--;
                                            }
                                            if (SplitEdge2(sewd1, face, num1, p11, p12, NewV, tolV, boxes1))
                                            {
                                                NbModif++;
                                                num1--;
                                                break;
                                            }
                                        }
                                    }
                                }
                            } // end if(Inter.NbSegments()==1)
                        }
                    }
                }
                if (hasModifWire)
                {
                    isDone = true;
                    // OCCT: each call of Wire() builds a new wire, so the face holds one wire, the context records
                    // another as the replacement and wire1 is a third.
                    SeqWir.SetValue(n1, sewd1.InternalWire());
                    myContext.Replace(wire1, sewd1.InternalWire());
                    wire1 = sewd1.InternalWire();
                    // recompute boxes for wire1
                    boxes1.Clear();
                    Box2d aNewBox1 = CreateBoxes2d(sewd1, face, boxes1);
                    aSeqWirBoxes.SetValue(n1, aNewBox1);
                    SeqWir.SetValue(n2, sewd2.InternalWire());
                    myContext.Replace(wire2, sewd2.InternalWire());
                    // wire2 = sewd2->Wire(): a local not read afterwards
                    // recompute boxes for wire2
                    boxes2.Clear();
                    Box2d aNewBox2 = CreateBoxes2d(sewd2, face, boxes2);
                    aSeqWirBoxes.SetValue(n2, aNewBox2);
                }
            }
        }

        if (isDone)
        {
            // update face
            Shape emptyCopied = face.EmptyCopied();
            Face  newface     = ShapeCasts.Face(emptyCopied);
            newface.Orientation(Forward);
            var B = new BrepBuilder();
            int i;
            for (i = 1; i <= SeqWir.Length(); i++)
            {
                Wire wire = ShapeCasts.Wire(SeqWir.Value(i));
                B.Add(newface, wire);
            }
            for (i = 1; i <= SeqNMShapes.Length(); i++)
            {
                Shape aNMS = SeqNMShapes.Value(i);
                B.Add(newface, aNMS);
            }
            newface.Orientation(ori);
            myContext.Replace(face, newface);
            face = newface;
        }
        return isDone;
    }
}
