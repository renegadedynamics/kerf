// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_IntersectionTool.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_IntersectionTool.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_IntersectionTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Engine.Intersection.TwoD.Position2D;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Healing.Fixes;

/// <summary>
/// Tool for fixing the intersecting edges of a wire (and the intersecting wires of a face): it splits and cuts the
/// edges at the intersections, merges close vertices, and records the replacements in a context.
/// </summary>
/// <remarks>
/// A working object (decision 21). Its fixes write in place into the edges, vertices, wire data and face they are
/// given (ranges, vertex tolerances, the edges of a <see cref="ExtendWireData"/>), so they are internal: ported
/// callers (ShapeFix_Wire, ShapeFix_Face) run them on shapes they own (their copies, decision 583).
/// </remarks>
internal sealed class IntersectionToolFixer
{
    private readonly ReShapeBuilder? _context;
    private readonly double              _preci;
    private readonly double              _maxTol;

    // gka 06.09.04 BUG 6555 shape is modified always independently either intersection was fixed or not

    /// <summary>Creates the tool with a context, a precision and a maximal tolerance.</summary>
    public IntersectionToolFixer(ReShapeBuilder? context, double preci, double maxtol = 1.0)
    {
        _context = context;
        _preci   = preci;
        _maxTol  = maxtol;
    }

    /// <summary>Returns the context.</summary>
    public ReShapeBuilder? Context() => _context;

    //=======================================================================
    // function : GetPointOnEdge
    // purpose  : auxiliary
    //: h0 abv 29 May 98: PRO10105 1949: like in BRepCheck, point is to be taken
    // from 3d curve (but only if edge is SameParameter)
    //=======================================================================
    private static Point3D GetPointOnEdge(in Edge edge, SurfaceAnalysis surf, CurveAdaptor2D crv2d,
                                         double param)
    {
        if (Tool.SameParameter(edge))
        {
            Curve3D? conS = Tool.Curve(edge, out Placement l, out _, out _);
            if (conS is not null)
            {
                return conS.Value(param).Transformed(l.Transformation);
            }
        }
        Point2D p2d = crv2d.Value(param);
        return surf.Adaptor3d()!.Value(p2d.X, p2d.Y);
    }

    /// <summary>
    /// Splits <paramref name="edge"/> at <paramref name="param"/> with the vertex <paramref name="vert"/> (whose
    /// tolerance is raised if needed) into two new edges; false if the vertex is one of the edge's or the parameter is
    /// at an end.
    /// </summary>
    internal bool SplitEdge(in Edge edge, double param, in Vertex vert, in Face face,
                            ref Edge newE1, ref Edge newE2, double preci)
    {
        var sae = new EdgeAnalysis();

        Vertex v1 = sae.FirstVertex(edge);
        Vertex v2 = sae.LastVertex(edge);
        if (v1.IsSame(vert) || v2.IsSame(vert))
        {
            return false;
        }

        sae.PCurve(edge, face, out Curve2D? c2d, out double a, out double b, true);
        if (Math.Abs(a - param) < 0.01 * preci || Math.Abs(b - param) < 0.01 * preci)
        {
            return false;
        }
        // check distance between edge and new vertex
        Point3D          p1;
        Placement l;
        if (Tool.SameParameter(edge) && !Tool.Degenerated(edge))
        {
            Curve3D? c3d = Tool.Curve(edge, out l, out _, out _);
            if (c3d is null)
            {
                return false;
            }
            p1 = c3d.Value(param);
            if (!l.IsIdentity)
            {
                p1 = p1.Transformed(l.Transformation);
            }
        }
        else
        {
            Surface? surf = Tool.Surface(face, out l);
            var           sas  = new SurfaceAnalysis(surf);
            p1                 = sas.Value(c2d!.Value(param));
            if (!l.IsIdentity)
            {
                p1 = p1.Transformed(l.Transformation);
            }
        }
        Point3D p2 = Tool.Pnt(vert);
        if (p1.DistanceTo(p2) > preci)
        {
            // return false;
            var b0 = new BrepBuilder();
            b0.UpdateVertex(vert, p1.DistanceTo(p2));
        }

        var transferParameters = new TransferParametersProjAnalysis();
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

        var                sbe    = new BuildEdgeBuilder();
        Orientation orient = edge.Orientation;
        var                B      = new BrepBuilder();
        Edge        wE     = edge;
        wE.Orient(Forward);
        Shape tmpShape = vert.Oriented(Reversed); // for porting
        newE1 = sbe.InternalCopyReplaceVertices(wE, sae.FirstVertex(wE), ShapeCasts.Vertex(tmpShape));
        sbe.CopyPCurves(newE1, wE);
        transferParameters.TransferRange(newE1, first, param, true);
        B.SameRange(newE1, false);
        B.SameParameter(newE1, false);
        tmpShape = vert.Oriented(Forward);
        newE2     = sbe.InternalCopyReplaceVertices(wE, ShapeCasts.Vertex(tmpShape), sae.LastVertex(wE));
        sbe.CopyPCurves(newE2, wE);
        transferParameters.TransferRange(newE2, param, last, true);
        B.SameRange(newE2, false);
        B.SameParameter(newE2, false);

        newE1.Orient(orient);
        newE2.Orient(orient);
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
        double range = Math.Abs(cut - pend);
        Tool.Range(edge, out double a, out double b);

        if (range < 10.0 * global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return false;
        }

        // case pcurve is trimm of line
        if (!Tool.SameParameter(edge))
        {
            var sae = new EdgeAnalysis();
            if (sae.PCurve(edge, face, out Curve2D? crv, out double fp, out double lp, false))
            {
                if (crv is TrimmedCurve2D tc)
                {
                    if (tc.BasisCurve is LineCurve2D)
                    {
                        var b0 = new BrepBuilder();
                        b0.Range(edge, Min(pend, cut), Max(pend, cut));
                        if (Math.Abs(pend - lp) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                        { // cut from the beginning
                            double cut3d = (cut - fp) * (b - a) / (lp - fp);
                            b0.Range(edge, a + cut3d, b, true);
                            iscutline = true;
                        }
                        else if (Math.Abs(pend - fp) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                        { // cut from the end
                            double cut3d = (lp - cut) * (b - a) / (lp - fp);
                            b0.Range(edge, a, b - cut3d, true);
                            iscutline = true;
                        }
                    }

                    return true;
                }
            }
            return false;
        }

        // det-study on 03/12/01 checking the old and new ranges
        if (Math.Abs(Math.Abs(a - b) - range) < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return false;
        }
        if (range < 10.0 * global::Kerf.Engine.Foundation.Precision.PConfusion())
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
    private bool SplitEdge1(ExtendWireData sewd, in Face face, int num, double param,
                            in Vertex vert, double preci, Dictionary<Shape, Box2d> boxes)
    {
        // Standard_ASSERT_RETURN: "Edge index out of range"
        if (!(num > 0 && num <= sewd.EdgeCount))
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
        var wd = new ExtendWireData();
        wd.Add(newE1);
        wd.Add(newE2);
        _context?.Replace(edge, wd.InternalWire());
        for (var exp = new Explorer(wd.InternalWire(), ShapeType.Edge); exp.More(); exp.Next())
        {
            Edge e = ShapeCasts.Edge(exp.Current());
            BrepTools.Update(e);
        }

        // change sewd
        sewd.Set(newE1, num);
        if (num == sewd.EdgeCount)
        {
            sewd.Add(newE2);
        }
        else
        {
            sewd.Add(newE2, num + 1);
        }

        // change boxes
        boxes.Remove(edge);
        Surface? s = Tool.Surface(face, out Placement l);
        var           sae = new EdgeAnalysis();
        if (sae.PCurve(newE1, s, l, out Curve2D? c2d, out double cf, out double cl, false))
        {
            var    box    = new Box2d();
            var    gac    = new CurveAdaptor2D();
            double first = c2d!.FirstParameter;
            double last  = c2d.LastParameter;
            if (c2d is BSplineCurve2D && (cf < first || cl > last))
            {
                // pdn avoiding problems with segment in Bnd_Box
                gac.Load(c2d);
            }
            else
            {
                gac.Load(c2d, cf, cl);
            }
            Add2dCurve.Add(gac, global::Kerf.Engine.Foundation.Precision.Confusion(), ref box);
            boxes[newE1] = box;
        }
        if (sae.PCurve(newE2, s, l, out c2d, out cf, out cl, false))
        {
            var    box    = new Box2d();
            var    gac    = new CurveAdaptor2D();
            double first = c2d!.FirstParameter;
            double last  = c2d.LastParameter;
            if (c2d is BSplineCurve2D && (cf < first || cl > last))
            {
                // pdn avoiding problems with segment in Bnd_Box
                gac.Load(c2d);
            }
            else
            {
                gac.Load(c2d, cf, cl);
            }
            Add2dCurve.Add(gac, global::Kerf.Engine.Foundation.Precision.Confusion(), ref box);
            boxes[newE2] = box;
        }

        return true;
    }

    //=======================================================================
    // function : SplitEdge2
    // purpose  : auxiliary: split edge[a,b] om two part e1[a,param1]
    //                     and e2[param2,b] using vertex vert
    //                     (remove segment (param1,param2) from edge)
    //=======================================================================
    private bool SplitEdge2(ExtendWireData sewd, in Face face, int num, double param1, double param2,
                            in Vertex vert, double preci, Dictionary<Shape, Box2d> boxes)
    {
        Edge edge  = sewd.Edge(num);
        Edge newE1 = new(), newE2 = new();
        double      param = (param1 + param2) / 2;
        if (!SplitEdge(edge, param, vert, face, ref newE1, ref newE2, preci))
        {
            return false;
        }
        // cut new edges by param1 and param2
        bool isCutLine = false;
        var  sae       = new EdgeAnalysis();
        if (sae.PCurve(newE1, face, out _, out double fp1, out double lp1, false))
        {
            if (sae.PCurve(newE2, face, out _, out double fp2, out double lp2, false))
            {
                if (lp1 == param)
                {
                    if ((lp1 - fp1) * (lp1 - param1) > 0)
                    {
                        CutEdge(newE1, fp1, param1, face, ref isCutLine);
                        CutEdge(newE2, lp2, param2, face, ref isCutLine);
                    }
                    else
                    {
                        CutEdge(newE1, fp1, param2, face, ref isCutLine);
                        CutEdge(newE2, lp2, param1, face, ref isCutLine);
                    }
                }
                else
                {
                    if ((fp1 - lp1) * (fp1 - param1) > 0)
                    {
                        CutEdge(newE1, lp1, param1, face, ref isCutLine);
                        CutEdge(newE2, fp2, param2, face, ref isCutLine);
                    }
                    else
                    {
                        CutEdge(newE1, lp1, param2, face, ref isCutLine);
                        CutEdge(newE2, fp2, param1, face, ref isCutLine);
                    }
                }
            }
        }

        // change context
        var wd = new ExtendWireData();
        wd.Add(newE1);
        wd.Add(newE2);
        _context?.Replace(edge, wd.InternalWire());
        for (var exp = new Explorer(wd.InternalWire(), ShapeType.Edge); exp.More(); exp.Next())
        {
            Edge e = ShapeCasts.Edge(exp.Current());
            BrepTools.Update(e);
        }

        // change sewd
        sewd.Set(newE1, num);
        if (num == sewd.EdgeCount)
        {
            sewd.Add(newE2);
        }
        else
        {
            sewd.Add(newE2, num + 1);
        }

        // change boxes
        boxes.Remove(edge);
        Surface? s = Tool.Surface(face, out Placement l);
        if (sae.PCurve(newE1, s, l, out Curve2D? c2d, out double cf, out double cl, false))
        {
            var    box    = new Box2d();
            var    gac    = new CurveAdaptor2D();
            double first = c2d!.FirstParameter;
            double last  = c2d.LastParameter;
            if (c2d is BSplineCurve2D && (cf < first || cl > last))
            {
                // pdn avoiding problems with segment in Bnd_Box
                gac.Load(c2d);
            }
            else
            {
                gac.Load(c2d, cf, cl);
            }
            Add2dCurve.Add(gac, global::Kerf.Engine.Foundation.Precision.Confusion(), ref box);
            boxes[newE1] = box;
        }
        if (sae.PCurve(newE2, s, l, out c2d, out cf, out cl, false))
        {
            var    box    = new Box2d();
            var    gac    = new CurveAdaptor2D();
            double first = c2d!.FirstParameter;
            double last  = c2d.LastParameter;
            if (c2d is BSplineCurve2D && (cf < first || cl > last))
            {
                // pdn avoiding problems with segment in Bnd_Box
                gac.Load(c2d);
            }
            else
            {
                gac.Load(c2d, cf, cl);
            }
            Add2dCurve.Add(gac, global::Kerf.Engine.Foundation.Precision.Confusion(), ref box);
            boxes[newE2] = box;
        }

        return true;
    }

    private bool UnionVertexes(ExtendWireData sewd, ref Edge edge1, ref Edge edge2, int num2,
                               Dictionary<Shape, Box2d> boxes, in Box2d b2)
    {
        // union vertexes
        bool          res  = false;
        var           sbe  = new BuildEdgeBuilder();
        var           sae  = new EdgeAnalysis();
        var           b    = new BrepBuilder();
        Vertex v1F  = sae.FirstVertex(edge1);
        Point3D        pv1F = Tool.Pnt(v1F);
        Vertex v1L  = sae.LastVertex(edge1);
        Point3D        pv1L = Tool.Pnt(v1L);
        Vertex v2F  = sae.FirstVertex(edge2);
        Point3D        pv2F = Tool.Pnt(v2F);
        Vertex v2L  = sae.LastVertex(edge2);
        Point3D        pv2L = Tool.Pnt(v2L);
        double        d11  = pv1F.DistanceTo(pv2F);
        double        d12  = pv1F.DistanceTo(pv2L);
        double        d21  = pv1L.DistanceTo(pv2F);
        double        d22  = pv1L.DistanceTo(pv2L);
        if (d11 < d12 && d11 < d21 && d11 < d22)
        {
            double tolv = Max(Tool.Tolerance(v1F), Tool.Tolerance(v2F));
            if (!v2F.IsSame(v1F) && d11 < tolv)
            {
                // union vertexes V1F and V2F
                b.UpdateVertex(v1F, tolv);
                Edge newE = sbe.InternalCopyReplaceVertices(edge2, v1F, v2L);
                _context!.Replace(edge2, newE);
                sewd.Set(newE, num2);
                edge2 = newE;
                boxes[newE] = b2; // update boxes
                // replace vertex in other edge
                int num21, num22;
                if (num2 > 1)
                {
                    num21 = num2 - 1;
                }
                else
                {
                    num21 = sewd.EdgeCount;
                }
                if (num2 < sewd.EdgeCount)
                {
                    num22 = num2 + 1;
                }
                else
                {
                    num22 = 1;
                }
                Edge   edge21 = sewd.Edge(num21);
                Edge   edge22 = sewd.Edge(num22);
                Vertex v21F   = sae.FirstVertex(edge21);
                Vertex v21L   = sae.LastVertex(edge21);
                Vertex v22F   = sae.FirstVertex(edge22);
                Vertex v22L   = sae.LastVertex(edge22);
                if (v21F.IsSame(v2F))
                {
                    newE = sbe.InternalCopyReplaceVertices(edge21, v1F, v21L);
                    if (boxes.ContainsKey(edge21))
                    {
                        boxes[newE] = boxes.GetValue(edge21); // update boxes
                    }
                    _context!.Replace(edge21, newE);
                    sewd.Set(newE, num21);
                }
                if (v21L.IsSame(v2F))
                {
                    newE = sbe.InternalCopyReplaceVertices(edge21, v21F, v1F);
                    if (boxes.ContainsKey(edge21))
                    {
                        boxes[newE] = boxes.GetValue(edge21); // update boxes
                    }
                    _context!.Replace(edge21, newE);
                    sewd.Set(newE, num21);
                }
                if (v22F.IsSame(v2F))
                {
                    newE = sbe.InternalCopyReplaceVertices(edge22, v1F, v22L);
                    if (boxes.ContainsKey(edge22))
                    {
                        boxes[newE] = boxes.GetValue(edge22); // update boxes
                    }
                    _context!.Replace(edge22, newE);
                    sewd.Set(newE, num22);
                }
                if (v22L.IsSame(v2F))
                {
                    newE = sbe.InternalCopyReplaceVertices(edge22, v22F, v1F);
                    if (boxes.ContainsKey(edge22))
                    {
                        boxes[newE] = boxes.GetValue(edge22); // update boxes
                    }
                    _context!.Replace(edge22, newE);
                    sewd.Set(newE, num22);
                }
                res = true;
            }
        }
        else if (d12 < d21 && d12 < d22)
        {
            double tolv = Max(Tool.Tolerance(v1F), Tool.Tolerance(v2L));
            if (!v2L.IsSame(v1F) && d12 < tolv)
            {
                // union vertexes V1F and V2L
                b.UpdateVertex(v1F, tolv);
                Edge newE = sbe.InternalCopyReplaceVertices(edge2, v2F, v1F);
                _context!.Replace(edge2, newE);
                sewd.Set(newE, num2);
                edge2 = newE;
                boxes[newE] = b2; // update boxes
                // replace vertex in other edge
                int num21, num22;
                if (num2 > 1)
                {
                    num21 = num2 - 1;
                }
                else
                {
                    num21 = sewd.EdgeCount;
                }
                if (num2 < sewd.EdgeCount)
                {
                    num22 = num2 + 1;
                }
                else
                {
                    num22 = 1;
                }
                Edge   edge21 = sewd.Edge(num21);
                Edge   edge22 = sewd.Edge(num22);
                Vertex v21F   = sae.FirstVertex(edge21);
                Vertex v21L   = sae.LastVertex(edge21);
                Vertex v22F   = sae.FirstVertex(edge22);
                Vertex v22L   = sae.LastVertex(edge22);
                if (v21F.IsSame(v2L))
                {
                    newE = sbe.InternalCopyReplaceVertices(edge21, v1F, v21L);
                    if (boxes.ContainsKey(edge21))
                    {
                        boxes[newE] = boxes.GetValue(edge21); // update boxes
                    }
                    _context!.Replace(edge21, newE);
                    sewd.Set(newE, num21);
                }
                if (v21L.IsSame(v2L))
                {
                    newE = sbe.InternalCopyReplaceVertices(edge21, v21F, v1F);
                    if (boxes.ContainsKey(edge21))
                    {
                        boxes[newE] = boxes.GetValue(edge21); // update boxes
                    }
                    _context!.Replace(edge21, newE);
                    sewd.Set(newE, num21);
                }
                if (v22F.IsSame(v2L))
                {
                    newE = sbe.InternalCopyReplaceVertices(edge22, v1F, v22L);
                    if (boxes.ContainsKey(edge22))
                    {
                        boxes[newE] = boxes.GetValue(edge22); // update boxes
                    }
                    _context!.Replace(edge22, newE);
                    sewd.Set(newE, num22);
                }
                if (v22L.IsSame(v2L))
                {
                    newE = sbe.InternalCopyReplaceVertices(edge22, v22F, v1F);
                    if (boxes.ContainsKey(edge22))
                    {
                        boxes[newE] = boxes.GetValue(edge22); // update boxes
                    }
                    _context!.Replace(edge22, newE);
                    sewd.Set(newE, num22);
                }
                res = true;
            }
        }
        else if (d21 < d22)
        {
            double tolv = Max(Tool.Tolerance(v1L), Tool.Tolerance(v2F));
            if (!v2F.IsSame(v1L) && d21 < tolv)
            {
                // union vertexes V1L and V2F
                b.UpdateVertex(v1L, tolv);
                Edge newE = sbe.InternalCopyReplaceVertices(edge2, v1L, v2L);
                _context!.Replace(edge2, newE);
                sewd.Set(newE, num2);
                edge2 = newE;
                boxes[newE] = b2; // update boxes
                // replace vertex in other edge
                int num21, num22;
                if (num2 > 1)
                {
                    num21 = num2 - 1;
                }
                else
                {
                    num21 = sewd.EdgeCount;
                }
                if (num2 < sewd.EdgeCount)
                {
                    num22 = num2 + 1;
                }
                else
                {
                    num22 = 1;
                }
                Edge   edge21 = sewd.Edge(num21);
                Edge   edge22 = sewd.Edge(num22);
                Vertex v21F   = sae.FirstVertex(edge21);
                Vertex v21L   = sae.LastVertex(edge21);
                Vertex v22F   = sae.FirstVertex(edge22);
                Vertex v22L   = sae.LastVertex(edge22);
                if (v21F.IsSame(v2F))
                {
                    newE = sbe.InternalCopyReplaceVertices(edge21, v1L, v21L);
                    if (boxes.ContainsKey(edge21))
                    {
                        boxes[newE] = boxes.GetValue(edge21); // update boxes
                    }
                    _context!.Replace(edge21, newE);
                    sewd.Set(newE, num21);
                }
                if (v21L.IsSame(v2F))
                {
                    newE = sbe.InternalCopyReplaceVertices(edge21, v21F, v1L);
                    if (boxes.ContainsKey(edge21))
                    {
                        boxes[newE] = boxes.GetValue(edge21); // update boxes
                    }
                    _context!.Replace(edge21, newE);
                    sewd.Set(newE, num21);
                }
                if (v22F.IsSame(v2F))
                {
                    newE = sbe.InternalCopyReplaceVertices(edge22, v1L, v22L);
                    if (boxes.ContainsKey(edge22))
                    {
                        boxes[newE] = boxes.GetValue(edge22); // update boxes
                    }
                    _context!.Replace(edge22, newE);
                    sewd.Set(newE, num22);
                }
                if (v22L.IsSame(v2F))
                {
                    newE = sbe.InternalCopyReplaceVertices(edge22, v22F, v1L);
                    if (boxes.ContainsKey(edge22))
                    {
                        boxes[newE] = boxes.GetValue(edge22); // update boxes
                    }
                    _context!.Replace(edge22, newE);
                    sewd.Set(newE, num22);
                }
                res = true;
            }
        }
        else
        {
            double tolv = Max(Tool.Tolerance(v1L), Tool.Tolerance(v2L));
            if (!v2L.IsSame(v1L) && d22 < tolv)
            {
                // union vertexes V1L and V2L
                b.UpdateVertex(v1L, tolv);
                Edge newE = sbe.InternalCopyReplaceVertices(edge2, v2F, v1L);
                _context!.Replace(edge2, newE);
                sewd.Set(newE, num2);
                edge2 = newE;
                boxes[newE] = b2; // update boxes
                // replace vertex in other edge
                int num21, num22;
                if (num2 > 1)
                {
                    num21 = num2 - 1;
                }
                else
                {
                    num21 = sewd.EdgeCount;
                }
                if (num2 < sewd.EdgeCount)
                {
                    num22 = num2 + 1;
                }
                else
                {
                    num22 = 1;
                }
                Edge   edge21 = sewd.Edge(num21);
                Edge   edge22 = sewd.Edge(num22);
                Vertex v21F   = sae.FirstVertex(edge21);
                Vertex v21L   = sae.LastVertex(edge21);
                Vertex v22F   = sae.FirstVertex(edge22);
                Vertex v22L   = sae.LastVertex(edge22);
                if (v21F.IsSame(v2L))
                {
                    newE = sbe.InternalCopyReplaceVertices(edge21, v1L, v21L);
                    if (boxes.ContainsKey(edge21))
                    {
                        boxes[newE] = boxes.GetValue(edge21); // update boxes
                    }
                    _context!.Replace(edge21, newE);
                    sewd.Set(newE, num21);
                }
                if (v21L.IsSame(v2L))
                {
                    newE = sbe.InternalCopyReplaceVertices(edge21, v21F, v1L);
                    if (boxes.ContainsKey(edge21))
                    {
                        boxes[newE] = boxes.GetValue(edge21); // update boxes
                    }
                    _context!.Replace(edge21, newE);
                    sewd.Set(newE, num21);
                }
                if (v22F.IsSame(v2L))
                {
                    newE = sbe.InternalCopyReplaceVertices(edge22, v1L, v22L);
                    if (boxes.ContainsKey(edge22))
                    {
                        boxes[newE] = boxes.GetValue(edge22); // update boxes
                    }
                    _context!.Replace(edge22, newE);
                    sewd.Set(newE, num22);
                }
                if (v22L.IsSame(v2L))
                {
                    newE = sbe.InternalCopyReplaceVertices(edge22, v22F, v1L);
                    if (boxes.ContainsKey(edge22))
                    {
                        boxes[newE] = boxes.GetValue(edge22); // update boxes
                    }
                    _context!.Replace(edge22, newE);
                    sewd.Set(newE, num22);
                }
                res = true;
            }
        }

        return res;
    }

    private static Box2d CreateBoxes2d(ExtendWireData sewd, in Face face,
                                           Dictionary<Shape, Box2d> boxes)
    {
        // create box2d for edges from wire
        var           totalBox = new Box2d();
        Surface? s         = Tool.Surface(face, out Placement l);
        var           sae       = new EdgeAnalysis();
        for (int i = 1; i <= sewd.EdgeCount; i++)
        {
            Edge e = sewd.Edge(i);
            if (sae.PCurve(e, s, l, out Curve2D? c2d, out double cf, out double cl, false))
            {
                var    box    = new Box2d();
                var    gac    = new CurveAdaptor2D();
                double first = c2d!.FirstParameter;
                double last  = c2d.LastParameter;
                if (c2d is BSplineCurve2D && (cf < first || cl > last))
                {
                    // pdn avoiding problems with segment in Bnd_Box
                    gac.Load(c2d);
                }
                else
                {
                    gac.Load(c2d, cf, cl);
                }
                Add2dCurve.Add(gac, global::Kerf.Engine.Foundation.Precision.Confusion(), ref box);
                boxes[e] = box;
                totalBox.Add(box);
            }
        }
        return totalBox;
    }

    private static void SelectIntPnt(GInter inter, out IntersectionPoint2D ip,
                                     out Transition tr1, out Transition tr2)
    {
        ip  = inter.Point(1);
        tr1 = ip.TransitionOfFirst();
        tr2 = ip.TransitionOfSecond();
        if (inter.PointCount == 2)
        {
            // possible second point is better?
            int status1 = 0, status2 = 0;
            if (tr1.PositionOnCurve() == Position2D.Middle)
            {
                status1 += 1;
            }
            if (tr2.PositionOnCurve() == Position2D.Middle)
            {
                status1 += 2;
            }
            IntersectionPoint2D ip2  = inter.Point(2);
            Transition        tr12 = ip2.TransitionOfFirst();
            Transition        tr22 = ip2.TransitionOfSecond();
            if (tr12.PositionOnCurve() == Position2D.Middle)
            {
                status2 += 1;
            }
            if (tr22.PositionOnCurve() == Position2D.Middle)
            {
                status2 += 2;
            }
            if (status2 > status1)
            {
                ip  = ip2;
                tr1 = tr12;
                tr2 = tr22;
            }
        }
    }

    private bool FindVertAndSplitEdge(double param1, in Edge edge1, in Edge edge2, Curve2D crv1,
                                      ref double maxTolVert, ref int num1, ExtendWireData sewd,
                                      in Face face, Dictionary<Shape, Box2d> boxes,
                                      bool aTmpKey)
    {
        // find needed vertex from edge2 and split edge1 using it
        var           sae       = new EdgeAnalysis();
        var           sas       = new SurfaceAnalysis(Tool.Surface(face));
        Point3D        pi1       = GetPointOnEdge(edge1, sas, new CurveAdaptor2D(crv1), param1);
        var           b         = new BrepBuilder();
        Vertex v;
        double        tolV;
        Vertex v1        = sae.FirstVertex(edge2);
        Point3D        pv1       = Tool.Pnt(v1);
        Vertex v2        = sae.LastVertex(edge2);
        Point3D        pv2       = Tool.Pnt(v2);
        Vertex v11       = sae.FirstVertex(edge1);
        Vertex v12       = sae.LastVertex(edge1);
        bool          needSplit = true;
        if (pi1.DistanceTo(pv1) < pi1.DistanceTo(pv2))
        {
            if (v1.IsSame(v11) || v1.IsSame(v12))
            {
                needSplit = false;
            }
            v    = v1;
            tolV = Max((pi1.DistanceTo(pv1) / 2) * 1.00001, Tool.Tolerance(v1));
        }
        else
        {
            if (v2.IsSame(v11) || v2.IsSame(v12))
            {
                needSplit = false;
            }
            v    = v2;
            tolV = Max((pi1.DistanceTo(pv2) / 2) * 1.00001, Tool.Tolerance(v2));
        }
        if (needSplit || aTmpKey)
        {
            if (SplitEdge1(sewd, face, num1, param1, v, tolV, boxes))
            {
                b.UpdateVertex(v, tolV);
                maxTolVert = Max(maxTolVert, tolV);
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
    internal bool FixSelfIntersectWire(ExtendWireData sewd, in Face face, ref int nbSplit, ref int nbCut,
                                       ref int nbRemoved)
    {
        if (_context is null || face.IsNull)
        {
            return false;
        }

        // double area2d = ShapeAnalysis::TotCross2D(sewd,face);
        // if(area2d<Precision::PConfusion()*Precision::PConfusion()) return false; //gka
        // 06.09.04 BUG 6555

        Shape sf         = Context()!.InternalApply(face);
        double       maxTolVert = 0.0;
        for (var exp = new Explorer(sf, ShapeType.Vertex); exp.More(); exp.Next())
        {
            double tolV = Tool.Tolerance(ShapeCasts.Vertex(exp.Current()));
            maxTolVert  = Max(maxTolVert, tolV);
        }
        maxTolVert = Min(maxTolVert, _maxTol);
        var sae = new EdgeAnalysis();

        // step 1 : intersection of adjacent edges

        // step 2 : intersection of non-adjacent edges
        var boxes = new Dictionary<Shape, Box2d>(ShapeHasher.Instance);
        _       = CreateBoxes2d(sewd, face, boxes);
        var sas = new SurfaceAnalysis(Tool.Surface(face));

        nbSplit = 0;
        nbCut   = 0;
        int  nbReplaced = 0;
        bool isDone;
        for (int num1 = 1; num1 < sewd.EdgeCount && nbSplit < 30; num1++)
        {
            // for each edge from first wire
            for (int num2 = num1 + 2; num2 <= sewd.EdgeCount && nbSplit < 30; num2++)
            {
                // for each edge from second wire
                if (num1 == 1 && num2 == sewd.EdgeCount)
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
                if (!boxes.ContainsKey(edge1) || !boxes.ContainsKey(edge2))
                {
                    continue;
                }
                Box2d B1 = boxes.GetValue(edge1);
                Box2d B2 = boxes.GetValue(edge2);
                if (!B1.IsOut(B2))
                {
                    // intersection is possible...
                    if (!sae.PCurve(edge1, face, out Curve2D? crv1, out double a1, out double b1, false))
                    {
                        return false;
                    }
                    if (!sae.PCurve(edge2, face, out Curve2D? crv2, out double a2, out double b2, false))
                    {
                        return false;
                    }
                    double tolint = 1.0e-10;
                    var    c1     = new CurveAdaptor2D(crv1!);
                    var    c2     = new CurveAdaptor2D(crv2!);
                    var    d1     = new TwoDDomain(c1.Value(a1), a1, tolint, c1.Value(b1), b1, tolint);
                    var    d2     = new TwoDDomain(c2.Value(a2), a2, tolint, c2.Value(b2), b2, tolint);
                    var    inter  = new GInter();
                    inter.Perform(c1, d1, c2, d2, tolint, tolint);
                    if (!inter.IsDone)
                    {
                        continue;
                    }
                    // intersection is point
                    if (inter.PointCount > 0 && inter.PointCount < 3)
                    {
                        SelectIntPnt(inter, out IntersectionPoint2D ip, out Transition tr1,
                                     out Transition tr2);
                        if (tr1.PositionOnCurve() == Position2D.Middle && tr2.PositionOnCurve() == Position2D.Middle)
                        {
                            double        param1 = ip.ParamOnFirst();
                            double        param2 = ip.ParamOnSecond();
                            Point3D        pi1    = GetPointOnEdge(edge1, sas, c1, param1);
                            Point3D        pi2    = GetPointOnEdge(edge2, sas, c2, param2);
                            var           B      = new BrepBuilder();
                            Vertex v      = new();
                            double        tolV   = 0;
                            // analysis for edge1
                            bool          modifE1 = false;
                            Vertex vf1     = sae.FirstVertex(edge1);
                            Point3D        pvf1    = Tool.Pnt(vf1);
                            Vertex vl1     = sae.LastVertex(edge1);
                            Point3D        pvl1    = Tool.Pnt(vl1);
                            double        dist1   = pi1.DistanceTo(pvf1);
                            double        dist2   = pi1.DistanceTo(pvl1);
                            double        distmin = Min(dist1, dist2);
                            if (dist1 != dist2 && distmin < maxTolVert)
                            {
                                if (dist1 < dist2)
                                {
                                    tolV = Max(dist1 * 1.00001, Tool.Tolerance(vf1));
                                    B.UpdateVertex(vf1, tolV);
                                    v = vf1;
                                }
                                else
                                {
                                    tolV = Max(dist2 * 1.00001, Tool.Tolerance(vl1));
                                    B.UpdateVertex(vl1, tolV);
                                    v = vl1;
                                }

                                double dista     = Math.Abs(a1 - param1);
                                double distb     = Math.Abs(b1 - param1);
                                bool   isCutLine = false;
                                modifE1 = CutEdge(edge1, ((dista > distb) ? a1 : b1), param1, face, ref isCutLine);
                                if (modifE1)
                                {
                                    nbCut++;
                                }
                                // not needed split edge, if one of parts is too small
                                modifE1 = modifE1 || distmin < global::Kerf.Engine.Foundation.Precision.Confusion();
                            }
                            // analysis for edge2
                            bool          modifE2 = false;
                            Vertex vf2     = sae.FirstVertex(edge2);
                            Point3D        pvf2    = Tool.Pnt(vf2);
                            Vertex vl2     = sae.LastVertex(edge2);
                            Point3D        pvl2    = Tool.Pnt(vl2);
                            dist1                 = pi2.DistanceTo(pvf2);
                            dist2                 = pi2.DistanceTo(pvl2);
                            distmin               = Min(dist1, dist2);
                            if (dist1 != dist2 && distmin < maxTolVert)
                            {
                                if (dist1 < dist2)
                                {
                                    tolV = Max(dist1 * 1.00001, Tool.Tolerance(vf2));
                                    B.UpdateVertex(vf2, tolV);
                                    v = vf2;
                                }
                                else
                                {
                                    tolV = Max(dist2 * 1.00001, Tool.Tolerance(vl2));
                                    B.UpdateVertex(vl2, tolV);
                                    v = vl2;
                                }

                                double dista     = Math.Abs(a2 - param2);
                                double distb     = Math.Abs(b2 - param2);
                                bool   isCutLine = false;
                                modifE2 = CutEdge(edge2, ((dista > distb) ? a2 : b2), param2, face, ref isCutLine);
                                if (modifE2)
                                {
                                    nbCut++;
                                }
                                // not needed split edge, if one of parts is too small
                                modifE2 = modifE2 || distmin < global::Kerf.Engine.Foundation.Precision.Confusion();
                            }
                            if (modifE1 && !modifE2)
                            {
                                if (SplitEdge1(sewd, face, num2, param2, v, tolV, boxes))
                                {
                                    nbSplit++;
                                    num2--;
                                    continue;
                                }
                            }
                            if (!modifE1 && modifE2)
                            {
                                if (SplitEdge1(sewd, face, num1, param1, v, tolV, boxes))
                                {
                                    nbSplit++;
                                    num1--;
                                    break;
                                }
                            }
                            if (!modifE1 && !modifE2)
                            {
                                var p0 = new Point3D((pi1.X + pi2.X) / 2, (pi1.Y + pi2.Y) / 2, (pi1.Z + pi2.Z) / 2);
                                tolV = Max((pi1.DistanceTo(pi2) / 2) * 1.00001, global::Kerf.Engine.Foundation.Precision.Confusion());
                                B.MakeVertex(ref v, p0, tolV);
                                maxTolVert        = Max(maxTolVert, tolV);
                                bool isEdgeSplit2 = SplitEdge1(sewd, face, num2, param2, v, tolV, boxes);
                                if (isEdgeSplit2)
                                {
                                    nbSplit++;
                                    num2--;
                                }
                                if (SplitEdge1(sewd, face, num1, param1, v, tolV, boxes))
                                {
                                    nbSplit++;
                                    num1--;
                                    break;
                                }
                                if (isEdgeSplit2)
                                {
                                    continue;
                                }
                            }
                        }
                        if (tr1.PositionOnCurve() == Position2D.Middle && tr2.PositionOnCurve() != Position2D.Middle)
                        {
                            // find needed vertex from edge2 and split edge1 using it
                            double param1 = ip.ParamOnFirst();
                            if (FindVertAndSplitEdge(param1, edge1, edge2, crv1!, ref maxTolVert, ref num1, sewd, face,
                                                     boxes, false))
                            {
                                nbSplit++;
                                break;
                            }
                        }
                        if (tr1.PositionOnCurve() != Position2D.Middle && tr2.PositionOnCurve() == Position2D.Middle)
                        {
                            // find needed vertex from edge1 and split edge2 using it
                            double param2 = ip.ParamOnSecond();
                            if (FindVertAndSplitEdge(param2, edge2, edge1, crv2!, ref maxTolVert, ref num2, sewd, face,
                                                     boxes, false))
                            {
                                nbSplit++;
                                continue;
                            }
                        }
                        if (tr1.PositionOnCurve() != Position2D.Middle && tr2.PositionOnCurve() != Position2D.Middle)
                        {
                            // union vertexes
                            if (UnionVertexes(sewd, ref edge1, ref edge2, num2, boxes, B2))
                            {
                                nbReplaced++; // gka 06.09.04
                            }
                        }
                    }
                    // intersection is segment
                    if (inter.SegmentCount == 1)
                    {
                        IntersectionSegment2D IS = inter.Segment(1);
                        if (IS.HasFirstPoint && IS.HasLastPoint)
                        {
                            bool                       isModified1 = false;
                            bool                       isModified2 = false;
                            Vertex              newV        = new();
                            var                        B           = new BrepBuilder();
                            double                     newtol      = 0.0;
                            IntersectionPoint2D ipf         = IS.FirstPoint();
                            double                     p11         = ipf.ParamOnFirst();
                            double                     p21         = ipf.ParamOnSecond();
                            IntersectionPoint2D ipl         = IS.LastPoint();
                            double                     p12         = ipl.ParamOnFirst();
                            double                     p22         = ipl.ParamOnSecond();
                            Point3D                     pnt11       = GetPointOnEdge(edge1, sas, c1, p11);
                            Point3D                     pnt12       = GetPointOnEdge(edge1, sas, c1, p12);
                            Point3D                     pnt21       = GetPointOnEdge(edge2, sas, c2, p21);
                            Point3D                     pnt22       = GetPointOnEdge(edge2, sas, c2, p22);
                            // next string commented by skl 29.12.2004 for OCC7624
                            // if( Pnt11.Distance(Pnt21)>myPreci || Pnt12.Distance(Pnt22)>myPreci ) continue;
                            if (pnt11.DistanceTo(pnt21) > maxTolVert || pnt12.DistanceTo(pnt22) > maxTolVert)
                            {
                                continue;
                            }
                            // analysis for edge1
                            Vertex v1  = sae.FirstVertex(edge1);
                            Point3D        pv1 = Tool.Pnt(v1);
                            Vertex v2  = sae.LastVertex(edge1);
                            Point3D        pv2 = Tool.Pnt(v2);
                            double        dist1   = pnt11.DistanceTo(pv1);
                            double        dist2   = pnt12.DistanceTo(pv1);
                            double        maxdist = Max(dist1, dist2);
                            double        pdist;
                            if (edge1.Orientation == Reversed)
                            {
                                pdist = Max(Math.Abs(b1 - p11), Math.Abs(b1 - p12));
                            }
                            else
                            {
                                pdist = Max(Math.Abs(a1 - p11), Math.Abs(a1 - p12));
                            }
                            if (maxdist < maxTolVert || pdist < Math.Abs(b1 - a1) * 0.01)
                            {
                                newtol      = maxdist;
                                newV        = v1;
                                isModified1 = true;
                            }
                            dist1   = pnt11.DistanceTo(pv2);
                            dist2   = pnt12.DistanceTo(pv2);
                            maxdist = Max(dist1, dist2);
                            if (edge1.Orientation == Reversed)
                            {
                                pdist = Max(Math.Abs(a1 - p11), Math.Abs(a1 - p12));
                            }
                            else
                            {
                                pdist = Max(Math.Abs(b1 - p11), Math.Abs(b1 - p12));
                            }
                            if (maxdist < maxTolVert || pdist < Math.Abs(b1 - a1) * 0.01)
                            {
                                if ((isModified1 && maxdist < newtol) || !isModified1)
                                {
                                    newtol      = maxdist;
                                    newV        = v2;
                                    isModified1 = true;
                                }
                            }
                            if (isModified1)
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
                                bool isCutLine = false;
                                if (CutEdge(edge1, pend, cut, face, ref isCutLine))
                                {
                                    nbCut++;
                                }
                                if (newtol > Tool.Tolerance(newV))
                                {
                                    B.UpdateVertex(newV, newtol);
                                }
                                else
                                {
                                    newtol = Tool.Tolerance(newV);
                                }
                            }
                            // analysis for edge2
                            Vertex v12  = sae.FirstVertex(edge2);
                            Point3D        pv12 = Tool.Pnt(v12);
                            Vertex v22  = sae.LastVertex(edge2);
                            Point3D        pv22 = Tool.Pnt(v22);
                            dist1   = pnt21.DistanceTo(pv12);
                            dist2   = pnt22.DistanceTo(pv12);
                            maxdist = Max(dist1, dist2);
                            if (edge2.Orientation == Reversed)
                            {
                                pdist = Max(Math.Abs(b2 - p21), Math.Abs(b2 - p22));
                            }
                            else
                            {
                                pdist = Max(Math.Abs(a2 - p21), Math.Abs(a2 - p22));
                            }
                            if (maxdist < maxTolVert || pdist < Math.Abs(b2 - a2) * 0.01)
                            {
                                newtol      = maxdist;
                                newV        = v12;
                                isModified2 = true;
                            }
                            dist1   = pnt21.DistanceTo(pv22);
                            dist2   = pnt22.DistanceTo(pv22);
                            maxdist = Max(dist1, dist2);
                            if (edge2.Orientation == Reversed)
                            {
                                pdist = Max(Math.Abs(a2 - p21), Math.Abs(a2 - p22));
                            }
                            else
                            {
                                pdist = Max(Math.Abs(b2 - p21), Math.Abs(b2 - p22));
                            }
                            if (maxdist < maxTolVert || pdist < Math.Abs(b2 - a2) * 0.01)
                            {
                                if ((isModified2 && maxdist < newtol) || !isModified2)
                                {
                                    newtol      = maxdist;
                                    newV        = v22;
                                    isModified2 = true;
                                }
                            }
                            if (isModified2)
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
                                bool isCutLine = false;
                                if (CutEdge(edge2, pend, cut, face, ref isCutLine))
                                {
                                    nbCut++;
                                }
                                if (newtol > Tool.Tolerance(newV))
                                {
                                    B.UpdateVertex(newV, newtol);
                                }
                                else
                                {
                                    newtol = Tool.Tolerance(newV);
                                }
                            }

                            if (isModified1 && !isModified2)
                            {
                                if (SplitEdge2(sewd, face, num2, p21, p22, newV, newtol, boxes))
                                {
                                    nbSplit++;
                                    num2--;
                                    continue;
                                }
                            }
                            if (!isModified1 && isModified2)
                            {
                                if (SplitEdge2(sewd, face, num1, p11, p12, newV, newtol, boxes))
                                {
                                    nbSplit++;
                                    num1--;
                                    break;
                                }
                            }
                            if (!isModified1 && !isModified2)
                            {
                                double param1 = (p11 + p12) / 2;
                                double param2 = (p21 + p22) / 2;
                                Point3D pnt10  = GetPointOnEdge(edge1, sas, c1, param1);
                                Point3D pnt20  = GetPointOnEdge(edge2, sas, c2, param2);
                                var    p0     = new Point3D((pnt10.X + pnt20.X) / 2,
                                                           (pnt10.Y + pnt20.Y) / 2,
                                                           (pnt10.Z + pnt20.Z) / 2);
                                dist1           = Max(pnt11.DistanceTo(p0), pnt12.DistanceTo(p0));
                                dist2           = Max(pnt21.DistanceTo(p0), pnt22.DistanceTo(p0));
                                double tolV     = Max(dist1, dist2);
                                tolV            = Max(tolV, pnt10.DistanceTo(pnt20)) * 1.00001;
                                bool fixSegment = true;
                                if (tolV < maxTolVert)
                                {
                                    // create new vertex and split each intersecting edge on two edges
                                    B.MakeVertex(ref newV, pnt10, tolV);
                                    if (SplitEdge2(sewd, face, num2, p21, p22, newV, tolV, boxes))
                                    {
                                        nbSplit++;
                                        num2--;
                                    }
                                    if (SplitEdge2(sewd, face, num1, p11, p12, newV, tolV, boxes))
                                    {
                                        nbSplit++;
                                        num1--;
                                        break;
                                    }
                                }
                                else if (fixSegment)
                                {
                                    //  segment is big and we have to split each intersecting edge
                                    //  on 3 edges --> middle edge - edge based on segment
                                    //  after we can remove edges made from segment
                                    var p01 = new Point3D((pnt11.X + pnt21.X) / 2,
                                                         (pnt11.Y + pnt21.Y) / 2,
                                                         (pnt11.Z + pnt21.Z) / 2);
                                    var p02 = new Point3D((pnt12.X + pnt22.X) / 2,
                                                         (pnt12.Y + pnt22.Y) / 2,
                                                         (pnt12.Z + pnt22.Z) / 2);
                                    double tolV1 = Max(pnt11.DistanceTo(p01), pnt21.DistanceTo(p01));
                                    tolV1        = Max(tolV1, global::Kerf.Engine.Foundation.Precision.Confusion()) * 1.00001;
                                    double tolV2 = Max(pnt12.DistanceTo(p02), pnt22.DistanceTo(p02));
                                    tolV2        = Max(tolV2, global::Kerf.Engine.Foundation.Precision.Confusion()) * 1.00001;
                                    if (tolV1 > maxTolVert || tolV2 > maxTolVert)
                                    {
                                        continue;
                                    }
                                    Vertex newV1 = new(), newV2 = new();
                                    Edge   tmpE;
                                    // split edge1
                                    int    akey1 = 0, akey2 = 0;
                                    double newTolerance;
                                    // analysis fo P01
                                    newTolerance = Max(tolV1, Tool.Tolerance(v1));
                                    if (p01.DistanceTo(pv1) < newTolerance)
                                    {
                                        B.MakeVertex(ref newV1, Tool.Pnt(v1), newTolerance);
                                        newV1.Orient(v1.Orientation);
                                        akey1++;
                                    }
                                    newTolerance = Max(tolV1, Tool.Tolerance(v2));
                                    if (p01.DistanceTo(pv2) < newTolerance)
                                    {
                                        B.MakeVertex(ref newV1, Tool.Pnt(v2), newTolerance);
                                        newV1.Orient(v2.Orientation);
                                        akey1++;
                                    }
                                    // analysis fo P02
                                    newTolerance = Max(tolV2, Tool.Tolerance(v1));
                                    if (p02.DistanceTo(pv1) < newTolerance)
                                    {
                                        B.MakeVertex(ref newV2, Tool.Pnt(v1), newTolerance);
                                        newV2.Orient(v1.Orientation);
                                        akey2++;
                                    }
                                    newTolerance = Max(tolV2, Tool.Tolerance(v2));
                                    if (p02.DistanceTo(pv2) < newTolerance)
                                    {
                                        B.MakeVertex(ref newV2, Tool.Pnt(v2), newTolerance);
                                        newV2.Orient(v2.Orientation);
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
                                        B.MakeVertex(ref newV1, p01, tolV1);
                                    }
                                    if (akey2 == 0)
                                    {
                                        B.MakeVertex(ref newV2, p02, tolV2);
                                    }
                                    // split
                                    if (akey1 == 0 && akey2 > 0)
                                    {
                                        if (SplitEdge1(sewd, face, num1, p11, newV1, tolV1, boxes))
                                        {
                                            nbSplit++;
                                            dnum1   = 1;
                                            numseg1 = num1 + 1;
                                        }
                                    }
                                    if (akey1 > 0 && akey2 == 0)
                                    {
                                        if (SplitEdge1(sewd, face, num1, p12, newV2, tolV2, boxes))
                                        {
                                            nbSplit++;
                                            dnum1   = 1;
                                            numseg1 = num1;
                                        }
                                    }
                                    if (akey1 == 0 && akey2 == 0)
                                    {
                                        if (SplitEdge1(sewd, face, num1, p11, newV1, tolV1, boxes))
                                        {
                                            nbSplit++;
                                            dnum1 = 1;
                                        }
                                        tmpE = sewd.Edge(num1);
                                        sae.PCurve(tmpE, face, out _, out double a, out double b, false);
                                        if ((a - p12) * (b - p12) > 0)
                                        { // p12 - external for [a,b] => split next edge
                                            if (SplitEdge1(sewd, face, num1 + 1, p12, newV2, tolV2, boxes))
                                            {
                                                nbSplit++;
                                                dnum1++;
                                                numseg1 = num1 + 1;
                                            }
                                        }
                                        else
                                        {
                                            if (SplitEdge1(sewd, face, num1, p12, newV2, tolV2, boxes))
                                            {
                                                nbSplit++;
                                                dnum1++;
                                                numseg1 = num1 + 1;
                                            }
                                        }
                                    }
                                    // SegE = sewd->Edge(numseg1); // get edge from segment
                                    //  split edge2
                                    //  replace vertices if it is necessary
                                    var sbe = new BuildEdgeBuilder();
                                    akey1 = 0;
                                    akey2 = 0;

                                    if (p01.DistanceTo(pv12) < tolV1)
                                    {
                                        tolV1 += p01.DistanceTo(pv12);
                                        B.UpdateVertex(newV1, tolV1);
                                        if (v12.Orientation == newV1.Orientation)
                                        {
                                            _context.Replace(v12, newV1);
                                            v12 = newV1;
                                        }
                                        else
                                        {
                                            _context.Replace(v12, newV1.Reversed());
                                            v12 = ShapeCasts.Vertex(newV1.Reversed());
                                        }
                                        nbReplaced++; // gka 06.09.04
                                        Edge newE = sbe.InternalCopyReplaceVertices(edge2, newV1, v22);
                                        _context.Replace(edge2, newE);
                                        sewd.Set(newE, num2 + dnum1);
                                        boxes[newE] = B2; // update boxes
                                        edge2 = newE;
                                        akey1 = 1;
                                    }
                                    if (p01.DistanceTo(pv22) < tolV1)
                                    {
                                        tolV1 += p01.DistanceTo(pv22);
                                        B.UpdateVertex(newV1, tolV1);
                                        if (v22.Orientation == newV1.Orientation)
                                        {
                                            _context.Replace(v22, newV1);
                                            v22 = newV1;
                                        }
                                        else
                                        {
                                            _context.Replace(v22, newV1.Reversed());
                                            v22 = ShapeCasts.Vertex(newV1.Reversed());
                                        }
                                        nbReplaced++; // gka 06.09.04
                                        Edge newE = sbe.InternalCopyReplaceVertices(edge2, v12, newV1);
                                        _context.Replace(edge2, newE);
                                        sewd.Set(newE, num2 + dnum1);
                                        boxes[newE] = B2; // update boxes
                                        edge2 = newE;
                                        akey1 = 2;
                                    }
                                    if (p02.DistanceTo(pv12) < tolV2)
                                    {
                                        tolV2 += p02.DistanceTo(pv12);
                                        B.UpdateVertex(newV2, tolV2);
                                        if (v12.Orientation == newV2.Orientation)
                                        {
                                            _context.Replace(v12, newV2);
                                            v12 = newV2;
                                        }
                                        else
                                        {
                                            _context.Replace(v12, newV2.Reversed());
                                            v12 = ShapeCasts.Vertex(newV2.Reversed());
                                        }
                                        nbReplaced++; // gka 06.09.04
                                        Edge newE = sbe.InternalCopyReplaceVertices(edge2, newV2, v22);
                                        _context.Replace(edge2, newE);
                                        sewd.Set(newE, num2 + dnum1);
                                        boxes[newE] = B2; // update boxes
                                        edge2 = newE;
                                        akey2 = 1;
                                    }
                                    if (p02.DistanceTo(pv22) < tolV2)
                                    {
                                        tolV2 += p02.DistanceTo(pv22);
                                        B.UpdateVertex(newV2, tolV2);
                                        if (v22.Orientation == newV2.Orientation)
                                        {
                                            _context.Replace(v22, newV2);
                                            v22 = newV2;
                                        }
                                        else
                                        {
                                            _context.Replace(v22, newV2.Reversed());
                                            v22 = ShapeCasts.Vertex(newV2.Reversed());
                                        }
                                        nbReplaced++; // gka 06.09.04
                                        Edge newE = sbe.InternalCopyReplaceVertices(edge2, v12, newV2);
                                        _context.Replace(edge2, newE);
                                        sewd.Set(newE, num2 + dnum1);
                                        boxes[newE] = B2; // update boxes
                                        edge2 = newE;
                                        akey2 = 2;
                                    }
                                    int dnum2 = 0, numseg2 = num2 + dnum1;
                                    // split
                                    if (akey1 == 0 && akey2 > 0)
                                    {
                                        if (SplitEdge1(sewd, face, num2 + dnum1, p21, newV1, tolV1, boxes))
                                        {
                                            nbSplit++;
                                            dnum2 = 1;
                                            // numseg2=num2+dnum1+1;
                                            numseg2 = num2 + dnum1;
                                        }
                                    }
                                    if (akey1 > 0 && akey2 == 0)
                                    {
                                        if (SplitEdge1(sewd, face, num2 + dnum1, p22, newV2, tolV2, boxes))
                                        {
                                            nbSplit++;
                                            dnum2 = 1;
                                            // numseg2=num2+dnum1;
                                            numseg2 = num2 + dnum1 + 1;
                                        }
                                    }
                                    if (akey1 == 0 && akey2 == 0)
                                    {
                                        if (SplitEdge1(sewd, face, num2 + dnum1, p21, newV1, tolV1, boxes))
                                        {
                                            nbSplit++;
                                            dnum2 = 1;
                                        }
                                        tmpE = sewd.Edge(num2 + dnum1);
                                        sae.PCurve(tmpE, face, out _, out double a, out double b, false);
                                        if ((a - p22) * (b - p22) > 0)
                                        { // p22 - external for [a,b] => split next edge
                                            if (SplitEdge1(sewd, face, num2 + dnum1 + dnum2, p22, newV2, tolV2, boxes))
                                            {
                                                nbSplit++;
                                                numseg2 = num2 + dnum1 + dnum2;
                                                dnum2++;
                                            }
                                        }
                                        else
                                        {
                                            if (SplitEdge1(sewd, face, num2 + dnum1, p22, newV2, tolV2, boxes))
                                            {
                                                nbSplit++;
                                                dnum2++;
                                                numseg2 = num2 + dnum1 + 1;
                                            }
                                        }
                                    }
                                    // remove segment
                                    sewd.Remove(numseg2);
                                    sewd.Remove(numseg1);
                                    nbRemoved = nbRemoved + 2; // added to the caller's count (not reset here, as in OCCT)
                                    // num1--;
                                    // break;
                                }
                            }
                        }
                    }
                }
            }
        }
        isDone = (nbSplit != 0 || nbCut != 0 || nbReplaced != 0 || nbRemoved != 0);
        return isDone;
    }

    /// <summary>
    /// Fixes the intersections between the wires of a face: splits and cuts their edges, and replaces the face (in
    /// the context and in <paramref name="face"/>) by a new face holding the fixed wires. Works in place on the face's
    /// edges and vertices.
    /// </summary>
    internal bool FixIntersectingWires(ref Face face)
    {
        if (_context is null || face.IsNull)
        {
            return false;
        }
        // TopoDS_Shape S = context->Apply(face);
        // TopoDS_Shape SF = TopoDS::Face(S);
        Shape       sf          = face;
        Orientation ori         = face.Orientation;
        var                seqWir      = new NCollection_Sequence<Shape>();
        var                seqNMShapes = new NCollection_Sequence<Shape>();
        for (var iter = new Iterator(sf, false); iter.More(); iter.Next())
        {
            if (iter.Value().Type != ShapeType.Wire
                || (iter.Value().Orientation != Forward && iter.Value().Orientation != Reversed))
            {
                seqNMShapes.Append(iter.Value());
                continue;
            }
            Wire wire = ShapeCasts.Wire(iter.Value());
            seqWir.Append(wire);
        }
        if (seqWir.Length() < 2)
        {
            return false; // gka 06.09.04
        }

        double maxTolVert = 0.0;
        for (var exp = new Explorer(sf, ShapeType.Vertex); exp.More(); exp.Next())
        {
            double tolV = Tool.Tolerance(ShapeCasts.Vertex(exp.Current()));
            maxTolVert  = Max(maxTolVert, tolV);
        }
        bool isDone = false; // gka 06.09.04
        var  sae    = new EdgeAnalysis();
        var  sas    = new SurfaceAnalysis(Tool.Surface(face));

        // precompute edge boxes for all wires
        var seqWirEdgeBoxes = new NCollection_Sequence<Dictionary<Shape, Box2d>>();
        var seqWirBoxes     = new NCollection_Sequence<Box2d>();
        for (int n = 1; n <= seqWir.Length(); n++)
        {
            Wire aWire  = ShapeCasts.Wire(seqWir.Value(n));
            var         sewd  = new ExtendWireData(aWire);
            var         boxes = new Dictionary<Shape, Box2d>(ShapeHasher.Instance);
            Box2d   totalBox = CreateBoxes2d(sewd, face, boxes);
            seqWirEdgeBoxes.Append(boxes);
            seqWirBoxes.Append(totalBox);
        }

        for (int n1 = 1; n1 <= seqWir.Length() - 1; n1++)
        {
            Wire wire1 = ShapeCasts.Wire(seqWir.Value(n1));
            var         sewd1 = new ExtendWireData(wire1);
            Dictionary<Shape, Box2d> boxes1 = seqWirEdgeBoxes.ChangeValue(n1);
            Box2d box1 = seqWirBoxes[n1];
            for (int n2 = n1 + 1; n2 <= seqWir.Length(); n2++)
            {
                Wire wire2 = ShapeCasts.Wire(seqWir.Value(n2));
                var         sewd2 = new ExtendWireData(wire2);
                Dictionary<Shape, Box2d> boxes2 = seqWirEdgeBoxes.ChangeValue(n2);
                Box2d box2 = seqWirBoxes[n2];
                if (!box1.IsVoid && !box2.IsVoid && box1.IsOut(box2))
                {
                    continue;
                }
                // detect possible intersections:
                int  nbModif      = 0;
                int  nbReplaced   = 0;     // gka 06.09.04
                bool hasModifWire = false; // gka 06.09.04
                for (int num1 = 1; num1 <= sewd1.EdgeCount && nbModif < 30; num1++)
                {
                    // for each edge from first wire
                    Edge edge1 = sewd1.Edge(num1); // gka 06.09.04

                    for (int num2 = 1; num2 <= sewd2.EdgeCount && nbModif < 30; num2++)
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
                        if (!boxes1.ContainsKey(edge1) || !boxes2.ContainsKey(edge2))
                        {
                            continue;
                        }
                        Box2d B1 = boxes1.GetValue(edge1);
                        Box2d B2 = boxes2.GetValue(edge2);
                        if (!B1.IsOut(B2))
                        {
                            // intersection is possible...
                            if (!sae.PCurve(edge1, face, out Curve2D? crv1, out double a1, out double b1, false))
                            {
                                continue; // return false; gka 06.09.04
                            }
                            if (!sae.PCurve(edge2, face, out Curve2D? crv2, out double a2, out double b2, false))
                            {
                                continue; // return false;gka 06.09.04
                            }
                            double tolint = 1.0e-10;
                            var    c1     = new CurveAdaptor2D(crv1!);
                            var    c2     = new CurveAdaptor2D(crv2!);
                            var    d1     = new TwoDDomain(c1.Value(a1), a1, tolint, c1.Value(b1), b1, tolint);
                            var    d2     = new TwoDDomain(c2.Value(a2), a2, tolint, c2.Value(b2), b2, tolint);
                            var    inter  = new GInter();
                            inter.Perform(c1, d1, c2, d2, tolint, tolint);
                            if (!inter.IsDone)
                            {
                                continue;
                            }
                            // intersection is point
                            if (inter.PointCount > 0 && inter.PointCount < 3)
                            {
                                SelectIntPnt(inter, out IntersectionPoint2D ip, out Transition tr1,
                                             out Transition tr2);
                                if (tr1.PositionOnCurve() == Position2D.Middle && tr2.PositionOnCurve() == Position2D.Middle)
                                {
                                    // create new vertex and split both edges
                                    double        param1 = ip.ParamOnFirst();
                                    double        param2 = ip.ParamOnSecond();
                                    Point3D        pi1    = GetPointOnEdge(edge1, sas, c1, param1);
                                    Point3D        pi2    = GetPointOnEdge(edge2, sas, c2, param2);
                                    var           p0     = new Point3D((pi1.X + pi2.X) / 2,
                                                                      (pi1.Y + pi2.Y) / 2,
                                                                      (pi1.Z + pi2.Z) / 2);
                                    var           B      = new BrepBuilder();
                                    Vertex v      = new();
                                    double tolV = Max((pi1.DistanceTo(pi2) / 2) * 1.00001, global::Kerf.Engine.Foundation.Precision.Confusion());
                                    B.MakeVertex(ref v, p0, tolV);
                                    maxTolVert        = Max(maxTolVert, tolV);
                                    bool isSplitEdge2 = SplitEdge1(sewd2, face, num2, param2, v, tolV, boxes2);
                                    if (isSplitEdge2)
                                    {
                                        nbModif++;
                                        num2--;
                                    }
                                    if (SplitEdge1(sewd1, face, num1, param1, v, tolV, boxes1))
                                    {
                                        nbModif++;
                                        num1--;
                                        break;
                                    }
                                    if (isSplitEdge2)
                                    {
                                        continue;
                                    }
                                }
                                if (tr1.PositionOnCurve() == Position2D.Middle && tr2.PositionOnCurve() != Position2D.Middle)
                                {
                                    // find needed vertex from edge2 and split edge1 using it
                                    double param1 = ip.ParamOnFirst();
                                    if (FindVertAndSplitEdge(param1, edge1, edge2, crv1!, ref maxTolVert, ref num1, sewd1,
                                                             face, boxes1, true))
                                    {
                                        nbModif++;
                                        break;
                                    }
                                }
                                if (tr1.PositionOnCurve() != Position2D.Middle && tr2.PositionOnCurve() == Position2D.Middle)
                                {
                                    // find needed vertex from edge1 and split edge2 using it
                                    double param2 = ip.ParamOnSecond();
                                    if (FindVertAndSplitEdge(param2, edge2, edge1, crv2!, ref maxTolVert, ref num2, sewd2,
                                                             face, boxes2, true))
                                    {
                                        nbModif++;
                                        continue;
                                    }
                                }
                                if (tr1.PositionOnCurve() != Position2D.Middle && tr2.PositionOnCurve() != Position2D.Middle)
                                {
                                    // union vertexes
                                    if (UnionVertexes(sewd2, ref edge1, ref edge2, num2, boxes2, B2))
                                    {
                                        nbReplaced++; // gka 06.09.04
                                    }
                                }
                            }
                            hasModifWire = (hasModifWire || nbModif != 0 || nbReplaced != 0);
                            // intersection is segment
                            if (inter.SegmentCount == 1)
                            {
                                IntersectionSegment2D IS = inter.Segment(1);
                                if (IS.HasFirstPoint && IS.HasLastPoint)
                                {
                                    bool                       isModified1 = false;
                                    bool                       isModified2 = false;
                                    Vertex              newV        = new();
                                    var                        B           = new BrepBuilder();
                                    double                     newtol      = 0.0;
                                    IntersectionPoint2D ipf         = IS.FirstPoint();
                                    double                     p11         = ipf.ParamOnFirst();
                                    double                     p21         = ipf.ParamOnSecond();
                                    IntersectionPoint2D ipl         = IS.LastPoint();
                                    double                     p12         = ipl.ParamOnFirst();
                                    double                     p22         = ipl.ParamOnSecond();
                                    Point3D                     pnt11       = GetPointOnEdge(edge1, sas, c1, p11);
                                    Point3D                     pnt12       = GetPointOnEdge(edge1, sas, c1, p12);
                                    Point3D                     pnt21       = GetPointOnEdge(edge2, sas, c2, p21);
                                    Point3D                     pnt22       = GetPointOnEdge(edge2, sas, c2, p22);

                                    // analysis for edge1
                                    Vertex v1      = sae.FirstVertex(edge1);
                                    Point3D        pv1     = Tool.Pnt(v1);
                                    Vertex v2      = sae.LastVertex(edge1);
                                    Point3D        pv2     = Tool.Pnt(v2);
                                    double        dist1   = pnt11.DistanceTo(pv1);
                                    double        dist2   = pnt12.DistanceTo(pv1);
                                    double        maxdist = Max(dist1, dist2);
                                    double        pdist;
                                    if (edge1.Orientation == Reversed)
                                    {
                                        pdist = Max(Math.Abs(b1 - p11), Math.Abs(b1 - p12));
                                    }
                                    else
                                    {
                                        pdist = Max(Math.Abs(a1 - p11), Math.Abs(a1 - p12));
                                    }
                                    if (maxdist < maxTolVert || pdist < Math.Abs(b1 - a1) * 0.01)
                                    {
                                        newtol      = maxdist;
                                        newV        = v1;
                                        isModified1 = true;
                                    }
                                    dist1   = pnt11.DistanceTo(pv2);
                                    dist2   = pnt12.DistanceTo(pv2);
                                    maxdist = Max(dist1, dist2);
                                    if (edge1.Orientation == Reversed)
                                    {
                                        pdist = Max(Math.Abs(a1 - p11), Math.Abs(a1 - p12));
                                    }
                                    else
                                    {
                                        pdist = Max(Math.Abs(b1 - p11), Math.Abs(b1 - p12));
                                    }
                                    if (maxdist < maxTolVert || pdist < Math.Abs(b1 - a1) * 0.01)
                                    {
                                        if ((isModified1 && maxdist < newtol) || !isModified1)
                                        {
                                            newtol      = maxdist;
                                            newV        = v2;
                                            isModified1 = true;
                                        }
                                    }
                                    if (isModified1)
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
                                        bool isCutLine = false;
                                        if (!CutEdge(edge1, pend, cut, face, ref isCutLine))
                                        {
                                            isModified1 = false;
                                            continue;
                                        }
                                        if (newtol > Tool.Tolerance(newV))
                                        {
                                            B.UpdateVertex(newV, newtol * 1.00001);
                                        }
                                    }

                                    // analysis for edge2
                                    Vertex v12  = sae.FirstVertex(edge2);
                                    Point3D        pv12 = Tool.Pnt(v12);
                                    Vertex v22  = sae.LastVertex(edge2);
                                    Point3D        pv22 = Tool.Pnt(v22);
                                    dist1   = pnt21.DistanceTo(pv12);
                                    dist2   = pnt22.DistanceTo(pv12);
                                    maxdist = Max(dist1, dist2);
                                    if (edge2.Orientation == Reversed)
                                    {
                                        pdist = Max(Math.Abs(b2 - p21), Math.Abs(b2 - p22));
                                    }
                                    else
                                    {
                                        pdist = Max(Math.Abs(a2 - p21), Math.Abs(a2 - p22));
                                    }
                                    if (maxdist < maxTolVert || pdist < Math.Abs(b2 - a2) * 0.01)
                                    {
                                        newtol      = maxdist;
                                        newV        = v12;
                                        isModified2 = true;
                                    }
                                    dist1   = pnt21.DistanceTo(pv22);
                                    dist2   = pnt22.DistanceTo(pv22);
                                    maxdist = Max(dist1, dist2);
                                    if (edge2.Orientation == Reversed)
                                    {
                                        pdist = Max(Math.Abs(a2 - p21), Math.Abs(a2 - p22));
                                    }
                                    else
                                    {
                                        pdist = Max(Math.Abs(b2 - p21), Math.Abs(b2 - p22));
                                    }
                                    if (maxdist < maxTolVert || pdist < Math.Abs(b2 - a2) * 0.01)
                                    {
                                        if ((isModified2 && maxdist < newtol) || !isModified2)
                                        {
                                            newtol      = maxdist;
                                            newV        = v22;
                                            isModified2 = true;
                                        }
                                    }
                                    if (isModified2)
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
                                        bool isCutLine = false;
                                        if (!CutEdge(edge2, pend, cut, face, ref isCutLine))
                                        {
                                            isModified2 = false;
                                            continue;
                                        }
                                        if (newtol > Tool.Tolerance(newV))
                                        {
                                            B.UpdateVertex(newV, newtol * 1.00001);
                                        }
                                    }

                                    if (isModified1 || isModified2)
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
                                            var p01 = new Point3D((pnt11.X + pnt21.X) / 2,
                                                                 (pnt11.Y + pnt21.Y) / 2,
                                                                 (pnt11.Z + pnt21.Z) / 2);
                                            var p02 = new Point3D((pnt12.X + pnt22.X) / 2,
                                                                 (pnt12.Y + pnt22.Y) / 2,
                                                                 (pnt12.Z + pnt22.Z) / 2);
                                            double tolV1 = Max(pnt11.DistanceTo(p01), pnt21.DistanceTo(p01));
                                            tolV1        = Max(tolV1, global::Kerf.Engine.Foundation.Precision.Confusion()) * 1.00001;
                                            double tolV2 = Max(pnt12.DistanceTo(p02), pnt22.DistanceTo(p02));
                                            tolV2        = Max(tolV2, global::Kerf.Engine.Foundation.Precision.Confusion()) * 1.00001;
                                            if (tolV1 > maxTolVert || tolV2 > maxTolVert)
                                            {
                                                continue;
                                            }

                                            hasModifWire = true; // gka 06.09.04
                                            Vertex newV1 = new(), newV2 = new();
                                            Edge   tmpE, segE;
                                            // split edge1
                                            int akey1 = 0, akey2 = 0;
                                            // analysis fo P01
                                            if (p01.DistanceTo(pv1) < Max(tolV1, Tool.Tolerance(v1)))
                                            {
                                                newV1 = v1;
                                                if (tolV1 > Tool.Tolerance(v1))
                                                {
                                                    B.UpdateVertex(newV1, tolV1);
                                                }
                                                akey1++;
                                            }
                                            if (p01.DistanceTo(pv2) < Max(tolV1, Tool.Tolerance(v2)))
                                            {
                                                newV1 = v2;
                                                if (tolV1 > Tool.Tolerance(v2))
                                                {
                                                    B.UpdateVertex(newV1, tolV1);
                                                }
                                                akey1++;
                                            }
                                            // analysis fo P02
                                            if (p02.DistanceTo(pv1) < Max(tolV2, Tool.Tolerance(v1)))
                                            {
                                                newV2 = v1;
                                                if (tolV2 > Tool.Tolerance(v1))
                                                {
                                                    B.UpdateVertex(newV2, tolV2);
                                                }
                                                akey2++;
                                            }
                                            if (p02.DistanceTo(pv2) < Max(tolV2, Tool.Tolerance(v2)))
                                            {
                                                newV2 = v2;
                                                if (tolV2 > Tool.Tolerance(v2))
                                                {
                                                    B.UpdateVertex(newV2, tolV2);
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
                                                B.MakeVertex(ref newV1, p01, tolV1);
                                            }
                                            if (akey2 == 0)
                                            {
                                                B.MakeVertex(ref newV2, p02, tolV2);
                                            }
                                            // split
                                            int numseg1 = num1;
                                            if (akey1 == 0 && akey2 > 0)
                                            {
                                                if (SplitEdge1(sewd1, face, num1, p11, newV1, tolV1, boxes1))
                                                {
                                                    nbModif++;
                                                    numseg1 = num1 + 1;
                                                }
                                            }
                                            if (akey1 > 0 && akey2 == 0)
                                            {
                                                if (SplitEdge1(sewd1, face, num1, p12, newV2, tolV2, boxes1))
                                                {
                                                    nbModif++;
                                                    numseg1 = num1;
                                                }
                                            }
                                            if (akey1 == 0 && akey2 == 0)
                                            {
                                                int num1split2 = num1; // what edge to split by point 2
                                                if (SplitEdge1(sewd1, face, num1, p11, newV1, tolV1, boxes1))
                                                {
                                                    nbModif++;
                                                    tmpE = sewd1.Edge(num1);
                                                    sae.PCurve(tmpE, face, out _, out double a, out double b, false);
                                                    if ((a - p12) * (b - p12) > 0)
                                                    { // p12 - external for [a,b] => split next edge
                                                        num1split2++;
                                                    }
                                                }
                                                if (SplitEdge1(sewd1, face, num1split2, p12, newV2, tolV2, boxes1))
                                                {
                                                    nbModif++;
                                                    numseg1 = num1 + 1;
                                                }
                                            }
                                            segE = sewd1.Edge(numseg1); // get edge from segment
                                            // split edge2
                                            // replace vertices if it is necessary
                                            var sbe = new BuildEdgeBuilder();
                                            akey1 = 0;
                                            akey2 = 0;
                                            if (p01.DistanceTo(pv12) < tolV1)
                                            {
                                                tolV1 += p01.DistanceTo(pv12);
                                                B.UpdateVertex(newV1, tolV1);
                                                v12              = newV1;
                                                Edge newE = sbe.InternalCopyReplaceVertices(edge2, newV1, v22);
                                                _context.Replace(edge2, newE);
                                                sewd2.Set(newE, num2);
                                                boxes2[newE] = B2; // update boxes2
                                                edge2 = newE;
                                                akey1 = 1;
                                            }
                                            if (p01.DistanceTo(pv22) < tolV1)
                                            {
                                                tolV1 += p01.DistanceTo(pv22);
                                                B.UpdateVertex(newV1, tolV1);
                                                v22              = newV1;
                                                Edge newE = sbe.InternalCopyReplaceVertices(edge2, v12, newV1);
                                                _context.Replace(edge2, newE);
                                                sewd2.Set(newE, num2);
                                                boxes2[newE] = B2; // update boxes2
                                                edge2 = newE;
                                                akey1 = 2;
                                            }
                                            if (p02.DistanceTo(pv12) < tolV2)
                                            {
                                                tolV2 += p02.DistanceTo(pv12);
                                                B.UpdateVertex(newV2, tolV2);
                                                v12              = newV2;
                                                Edge newE = sbe.InternalCopyReplaceVertices(edge2, newV2, v22);
                                                _context.Replace(edge2, newE);
                                                sewd2.Set(newE, num2);
                                                boxes2[newE] = B2; // update boxes2
                                                edge2 = newE;
                                                akey2 = 1;
                                            }
                                            if (p02.DistanceTo(pv22) < tolV2)
                                            {
                                                tolV2 += p02.DistanceTo(pv22);
                                                B.UpdateVertex(newV2, tolV2);
                                                v22              = newV2;
                                                Edge newE = sbe.InternalCopyReplaceVertices(edge2, v12, newV2);
                                                _context.Replace(edge2, newE);
                                                sewd2.Set(newE, num2);
                                                boxes2[newE] = B2; // update boxes2
                                                edge2 = newE;
                                                akey2 = 2;
                                            }
                                            // split
                                            int numseg2 = num2;
                                            if (akey1 == 0 && akey2 > 0)
                                            {
                                                if (SplitEdge1(sewd2, face, num2, p21, newV1, tolV1, boxes2))
                                                {
                                                    nbModif++;
                                                    numseg2 = num2 + 1;
                                                }
                                            }
                                            if (akey1 > 0 && akey2 == 0)
                                            {
                                                if (SplitEdge1(sewd2, face, num2, p22, newV2, tolV2, boxes2))
                                                {
                                                    nbModif++;
                                                    numseg2 = num2;
                                                }
                                            }
                                            if (akey1 == 0 && akey2 == 0)
                                            {
                                                int num2split2 = num2;
                                                if (SplitEdge1(sewd2, face, num2, p21, newV1, tolV1, boxes2))
                                                {
                                                    nbModif++;
                                                    numseg2 = num2 + 1;
                                                    tmpE    = sewd2.Edge(num2);
                                                    sae.PCurve(tmpE, face, out _, out double a, out double b, false);
                                                    if ((a - p22) * (b - p22) > 0)
                                                    { // p22 - external for [a,b] => split next edge
                                                        num2split2++;
                                                    }
                                                }
                                                if (SplitEdge1(sewd2, face, num2split2, p22, newV2, tolV2, boxes2))
                                                {
                                                    nbModif++;
                                                    numseg2 = num2 + 1;
                                                }
                                            }
                                            tmpE = sewd2.Edge(numseg2);
                                            boxes2[tmpE] = boxes1.GetValue(segE); // update boxes2
                                            if (!sae.FirstVertex(segE).IsSame(sae.FirstVertex(tmpE)))
                                            {
                                                segE.Reverse();
                                            }
                                            _context.Replace(tmpE, segE);
                                            sewd2.Set(segE, numseg2);
                                            num1--;
                                            break;
                                        }
                                        else
                                        {
                                            // split each intersecting edge on two edges
                                            var p0 = new Point3D((pnt11.X + pnt12.X) / 2,
                                                                (pnt11.Y + pnt12.Y) / 2,
                                                                (pnt11.Z + pnt12.Z) / 2);
                                            double param1 = (p11 + p12) / 2;
                                            double param2 = (p21 + p22) / 2;
                                            Point3D pnt10  = GetPointOnEdge(edge1, sas, c1, param1);
                                            Point3D pnt20  = GetPointOnEdge(edge2, sas, c2, param2);
                                            dist1         = Max(pnt11.DistanceTo(p0), pnt12.DistanceTo(pnt10));
                                            dist2         = Max(pnt21.DistanceTo(p0), pnt22.DistanceTo(pnt10));
                                            double tolV   = Max(dist1, dist2);
                                            tolV          = Max(tolV, pnt10.DistanceTo(pnt20)) * 1.00001;
                                            B.MakeVertex(ref newV, pnt10, tolV);
                                            maxTolVert   = Max(maxTolVert, tolV);
                                            hasModifWire = true;
                                            if (SplitEdge2(sewd2, face, num2, p21, p22, newV, tolV, boxes2))
                                            {
                                                nbModif++;
                                                num2--;
                                            }
                                            if (SplitEdge2(sewd1, face, num1, p11, p12, newV, tolV, boxes1))
                                            {
                                                nbModif++;
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
                    seqWir.SetValue(n1, sewd1.InternalWire());
                    _context.Replace(wire1, sewd1.InternalWire());
                    wire1 = sewd1.InternalWire();
                    // recompute boxes for wire1
                    boxes1.Clear();
                    Box2d newBox1 = CreateBoxes2d(sewd1, face, boxes1);
                    seqWirBoxes.SetValue(n1, newBox1);
                    seqWir.SetValue(n2, sewd2.InternalWire());
                    _context.Replace(wire2, sewd2.InternalWire());
                    // wire2 = sewd2->Wire(): a local not read afterwards
                    // recompute boxes for wire2
                    boxes2.Clear();
                    Box2d newBox2 = CreateBoxes2d(sewd2, face, boxes2);
                    seqWirBoxes.SetValue(n2, newBox2);
                }
            }
        }

        if (isDone)
        {
            // update face
            Shape emptyCopied = face.EmptyCopied();
            Face  newface     = ShapeCasts.Face(emptyCopied);
            newface.Orient(Forward);
            var B = new BrepBuilder();
            int i;
            for (i = 1; i <= seqWir.Length(); i++)
            {
                Wire wire = ShapeCasts.Wire(seqWir.Value(i));
                B.Add(newface, wire);
            }
            for (i = 1; i <= seqNMShapes.Length(); i++)
            {
                Shape nms = seqNMShapes.Value(i);
                B.Add(newface, nms);
            }
            newface.Orient(ori);
            _context.Replace(face, newface);
            face = newface;
        }
        return isDone;
    }
}
