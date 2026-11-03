// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_SplitTool.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_SplitTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2004-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Healing.Fixes;

/// <summary>Splits an edge in two (or three) at given parameters and vertices, and cuts an edge's range.</summary>
/// <remarks>
/// A stateless working object, used by ShapeFix_Wire and ShapeFix_ComposeShell on shapes they own: its methods write
/// into the edges and vertices they are given (ranges, tolerances, the context), so they are internal. The new edges
/// are unfrozen.
/// </remarks>
public sealed class SplitToolFixer
{
    /// <summary>Empty constructor.</summary>
    public SplitToolFixer()
    {
    }

    /// <summary>
    /// Splits the edge at <paramref name="param"/> (of its pcurve on the face) with the vertex
    /// <paramref name="vert"/>, whose tolerance is raised to its distance from the edge if beyond
    /// <paramref name="tol3d"/>. False if the parameter is within <paramref name="tol2d"/> of an end.
    /// </summary>
    internal bool SplitEdge(in Edge edge, double param, in Vertex vert, in Face face,
                            ref Edge newE1, ref Edge newE2, double tol3d, double tol2d)
    {
        var sae = new EdgeAnalysis();
        sae.PCurve(edge, face, out Curve2D? c2d, out double a, out double b, true);
        if (Math.Abs(a - param) < tol2d || Math.Abs(b - param) < tol2d)
        {
            return false;
        }
        // check distance between edge and new vertex
        Point3D          p1;
        Placement L;
        if (Tool.SameParameter(edge))
        {
            Curve3D? c3d = Tool.Curve(edge, out L, out double f, out double l);
            if (c3d is null)
            {
                return false;
            }
            p1 = c3d.Value(param);
            if (!L.IsIdentity)
            {
                p1 = p1.Transformed(L.Transformation());
            }
        }
        else
        {
            Surface? surf = Tool.Surface(face, out L);
            var           sas  = new SurfaceAnalysis(surf);
            p1 = sas.Value(c2d!.Value(param));
            if (!L.IsIdentity)
            {
                p1 = p1.Transformed(L.Transformation());
            }
        }
        Point3D p2 = Tool.Pnt(vert);
        if (p1.Distance(p2) > tol3d)
        {
            // return false;
            var bv = new BrepBuilder();
            bv.UpdateVertex(vert, p1.Distance(p2));
        }

        var transferParameters = new TransferParametersProjAnalysis();
        transferParameters.SetMaxTolerance(tol3d);
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
        var                sfe    = new EdgeFixer();
        Orientation orient = edge.Orientation();
        var                B      = new BrepBuilder();
        Edge        wE     = edge;
        wE.Orientation(Forward);
        Shape tmpShape = vert.Oriented(Reversed); // for porting
        newE1 = sbe.InternalCopyReplaceVertices(wE, sae.FirstVertex(wE), ShapeCasts.Vertex(tmpShape));
        sbe.CopyPCurves(newE1, wE);
        transferParameters.TransferRange(newE1, first, param, true);
        B.SameRange(newE1, false);
        sfe.FixSameParameter(newE1);
        // B.SameParameter(newE1,false);
        tmpShape = vert.Oriented(Forward);
        newE2 = sbe.InternalCopyReplaceVertices(wE, ShapeCasts.Vertex(tmpShape), sae.LastVertex(wE));
        sbe.CopyPCurves(newE2, wE);
        transferParameters.TransferRange(newE2, param, last, true);
        B.SameRange(newE2, false);
        sfe.FixSameParameter(newE2);
        // B.SameParameter(newE2,false);

        newE1.Orientation(orient);
        newE2.Orientation(orient);
        if (orient == Reversed)
        {
            Edge tmp = newE2;
            newE2 = newE1;
            newE1 = tmp;
        }

        return true;
    }

    /// <summary>
    /// Splits the edge at the middle of [param1, param2] with the vertex, then cuts the two parts at param1 and
    /// param2.
    /// </summary>
    internal bool SplitEdge(in Edge edge, double param1, double param2, in Vertex vert, in Face face,
                            ref Edge newE1, ref Edge newE2, double tol3d, double tol2d)
    {
        double param = (param1 + param2) / 2;
        if (SplitEdge(edge, param, vert, face, ref newE1, ref newE2, tol3d, tol2d))
        {
            // cut new edges by param1 and param2
            bool isCutLine = false;
            var  sae       = new EdgeAnalysis();
            if (sae.PCurve(newE1, face, out Curve2D? crv1, out double fp1, out double lp1, false))
            {
                if (sae.PCurve(newE2, face, out Curve2D? crv2, out double fp2, out double lp2, false))
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
            return true;
        }
        return false;
    }

    /// <summary>
    /// Cuts the edge's range to [pend, cut] (as ordered); <paramref name="iscutline"/> tells whether a trimmed line
    /// pcurve of a non-SameParameter edge was cut by its 3D range. Writes into the edge.
    /// </summary>
    internal bool CutEdge(in Edge edge, double pend, double cut, in Face face, ref bool iscutline)
    {
        if (Math.Abs(cut - pend) < 10.0 * Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return false;
        }
        double range = Math.Abs(cut - pend);
        Tool.Range(edge, out double a, out double b);
        iscutline = false;
        if (range < 10.0 * Kerf.Engine.Foundation.Precision.PConfusion())
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
                    if (tc.BasisCurve() is LineCurve2D)
                    {
                        var bl = new BrepBuilder();
                        bl.Range(edge, Min(pend, cut), Max(pend, cut));
                        if (Math.Abs(pend - lp) < Kerf.Engine.Foundation.Precision.PConfusion())
                        { // cut from the beginning
                            double cut3d = (cut - fp) * (b - a) / (lp - fp);
                            if (cut3d <= Kerf.Engine.Foundation.Precision.PConfusion())
                            {
                                return false;
                            }
                            bl.Range(edge, a + cut3d, b, true);
                            iscutline = true;
                        }
                        else if (Math.Abs(pend - fp) < Kerf.Engine.Foundation.Precision.PConfusion())
                        { // cut from the end
                            double cut3d = (lp - cut) * (b - a) / (lp - fp);
                            if (cut3d <= Kerf.Engine.Foundation.Precision.PConfusion())
                            {
                                return false;
                            }
                            bl.Range(edge, a, b - cut3d, true);
                            iscutline = true;
                        }
                    }
                }
            }
            return true;
        }

        // det-study on 03/12/01 checking the old and new ranges
        if (Math.Abs(Math.Abs(a - b) - range) < Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return false;
        }
        if (range < 10.0 * Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return false;
        }

        Curve3D? c   = Tool.Curve(edge, out a, out b);
        var         sac = new CurveAnalysis();
        a = Min(pend, cut);
        b = Max(pend, cut);
        double na = a, nb = b;

        var B = new BrepBuilder();
        // ValidateRange reverses the curve only when the range is reversed, which min/max excludes: the edge's
        // (frozen) curve is only read.
        if (!Tool.Degenerated(edge) && c is not null
            && sac.ValidateRange(c, ref na, ref nb, Kerf.Engine.Foundation.Precision.PConfusion()) && (na != a || nb != b))
        {
            B.Range(edge, na, nb, true);
            var sae = new EdgeAnalysis();
            if (sae.HasPCurve(edge, face))
            {
                B.SameRange(edge, false);
            }

            var sfe = new EdgeFixer();
            sfe.FixSameParameter(edge);
        }
        else
        {
            B.Range(edge, a, b, false);
        }

        return true;
    }

    /// <summary>
    /// Splits the edge between the parameters <paramref name="fp"/> and <paramref name="lp"/> with the vertices
    /// <paramref name="v1"/> and <paramref name="v2"/> (merging them with the edge's own ends where they coincide,
    /// through the context); the parts are added to <paramref name="seqE"/>, <paramref name="aNum"/> is the index of
    /// the middle part, and the context records the edge's replacement by the wire of the parts.
    /// </summary>
    internal bool SplitEdge(in Edge edge, double fp, in Vertex v1, double lp, in Vertex v2,
                            in Face face, NCollection_Sequence<Shape> seqE, ref int aNum,
                            ReShapeBuilder context, double tol3d, double tol2d)
    {
        if (Math.Abs(lp - fp) < tol2d)
        {
            return false;
        }
        aNum = 0;
        seqE.Clear();
        var B   = new BrepBuilder();
        var sae = new EdgeAnalysis();
        sae.PCurve(edge, face, out _, out double a, out double b, true);
        Vertex vf    = sae.FirstVertex(edge);
        Vertex vl    = sae.LastVertex(edge);
        double        tolVF = Tool.Tolerance(vf);
        double        tolVL = Tool.Tolerance(vl);
        double        tolV1 = Tool.Tolerance(v1);
        double        tolV2 = Tool.Tolerance(v2);
        Point3D        pvf   = Tool.Pnt(vf);
        Point3D        pvl   = Tool.Pnt(vl);
        Point3D        pv1   = Tool.Pnt(v1);
        Point3D        pv2   = Tool.Pnt(v2);

        double par1, par2;
        bool   isReverse = false;
        if ((b - a) * (lp - fp) > 0)
        {
            par1 = fp;
            par2 = lp;
        }
        else
        {
            par1      = lp;
            par2      = fp;
            isReverse = true;
        }

        if (Math.Abs(a - par1) <= tol2d && Math.Abs(b - par2) <= tol2d)
        {
            if (isReverse)
            {
                double newtol = tolVF + pvf.Distance(pv2);
                if (tolV2 < newtol)
                {
                    B.UpdateVertex(v2, newtol);
                }
                if (vf.Orientation() == v2.Orientation())
                {
                    context.Replace(vf, v2);
                    vf = v2;
                }
                else
                {
                    context.Replace(vf, v2.Reversed());
                    vf = ShapeCasts.Vertex(v2.Reversed());
                }
                newtol = tolVL + pvl.Distance(pv1);
                if (tolV1 < newtol)
                {
                    B.UpdateVertex(v1, newtol);
                }
                if (vl.Orientation() == v1.Orientation())
                {
                    context.Replace(vl, v1);
                    vl = v1;
                }
                else
                {
                    context.Replace(vl, v1.Reversed());
                    vl = ShapeCasts.Vertex(v1.Reversed());
                }
            }
            else
            {
                double newtol = tolVF + pvf.Distance(pv1);
                if (tolV1 < newtol)
                {
                    B.UpdateVertex(v1, newtol);
                }
                if (vf.Orientation() == v1.Orientation())
                {
                    context.Replace(vf, v1);
                    vf = v1;
                }
                else
                {
                    context.Replace(vf, v1.Reversed());
                    vf = ShapeCasts.Vertex(v1.Reversed());
                }
                newtol = tolVL + pvl.Distance(pv2);
                if (tolV2 < newtol)
                {
                    B.UpdateVertex(v2, newtol);
                }
                if (vl.Orientation() == v2.Orientation())
                {
                    context.Replace(vl, v2);
                    vl = v2;
                }
                else
                {
                    context.Replace(vl, v2.Reversed());
                    vl = ShapeCasts.Vertex(v2.Reversed());
                }
            }
            seqE.Append(edge);
            aNum = 1;
        }

        if (Math.Abs(a - par1) <= tol2d && Math.Abs(b - par2) > tol2d)
        {
            Edge newE1 = new Edge(), newE2 = new Edge();
            if (isReverse)
            {
                if (!SplitEdge(edge, par2, v1, face, ref newE1, ref newE2, tol3d, tol2d))
                {
                    return false;
                }
                double newtol = tolVF + pvf.Distance(pv2);
                if (tolV2 < newtol)
                {
                    B.UpdateVertex(v2, newtol);
                }
                if (vf.Orientation() == v2.Orientation())
                {
                    context.Replace(vf, v2);
                    vf = v2;
                }
                else
                {
                    context.Replace(vf, v2.Reversed());
                    vf = ShapeCasts.Vertex(v2.Reversed());
                }
            }
            else
            {
                if (!SplitEdge(edge, par2, v2, face, ref newE1, ref newE2, tol3d, tol2d))
                {
                    return false;
                }
                double newtol = tolVF + pvf.Distance(pv1);
                if (tolV1 < newtol)
                {
                    B.UpdateVertex(v1, newtol);
                }
                if (vf.Orientation() == v1.Orientation())
                {
                    context.Replace(vf, v1);
                    vf = v1;
                }
                else
                {
                    context.Replace(vf, v1.Reversed());
                    vf = ShapeCasts.Vertex(v1.Reversed());
                }
            }
            seqE.Append(newE1);
            seqE.Append(newE2);
            aNum = 1;
        }

        if (Math.Abs(a - par1) > tol2d && Math.Abs(b - par2) <= tol2d)
        {
            Edge newE1 = new Edge(), newE2 = new Edge();
            if (isReverse)
            {
                if (!SplitEdge(edge, par1, v2, face, ref newE1, ref newE2, tol3d, tol2d))
                {
                    return false;
                }
                double newtol = tolVL + pvl.Distance(pv1);
                if (tolV1 < newtol)
                {
                    B.UpdateVertex(v1, newtol);
                }
                if (vl.Orientation() == v1.Orientation())
                {
                    context.Replace(vl, v1);
                    vl = v1;
                }
                else
                {
                    context.Replace(vl, v1.Reversed());
                    vl = ShapeCasts.Vertex(v1.Reversed());
                }
            }
            else
            {
                if (!SplitEdge(edge, par1, v1, face, ref newE1, ref newE2, tol3d, tol2d))
                {
                    return false;
                }
                double newtol = tolVL + pvl.Distance(pv2);
                if (tolV2 < newtol)
                {
                    B.UpdateVertex(v2, newtol);
                }
                if (vl.Orientation() == v2.Orientation())
                {
                    context.Replace(vl, v2);
                    vl = v2;
                }
                else
                {
                    context.Replace(vl, v2.Reversed());
                    vl = ShapeCasts.Vertex(v2.Reversed());
                }
            }
            seqE.Append(newE1);
            seqE.Append(newE2);
            aNum = 2;
        }

        if (Math.Abs(a - par1) > tol2d && Math.Abs(b - par2) > tol2d)
        {
            Edge newE1 = new Edge(), newE2 = new Edge(), newE3 = new Edge(),
                        newE4 = new Edge();
            if (isReverse)
            {
                if (!SplitEdge(edge, par1, v2, face, ref newE1, ref newE2, tol3d, tol2d))
                {
                    return false;
                }
                if (!SplitEdge(newE2, par2, v1, face, ref newE3, ref newE4, tol3d, tol2d))
                {
                    return false;
                }
            }
            else
            {
                if (!SplitEdge(edge, par1, v1, face, ref newE1, ref newE2, tol3d, tol2d))
                {
                    return false;
                }
                if (!SplitEdge(newE2, par2, v2, face, ref newE3, ref newE4, tol3d, tol2d))
                {
                    return false;
                }
            }
            seqE.Append(newE1);
            seqE.Append(newE3);
            seqE.Append(newE4);
            aNum = 2;
        }

        if (aNum == 0)
        {
            return false;
        }

        var sewd = new ExtendWireData();
        for (int i = 1; i <= seqE.Length(); i++)
        {
            sewd.Add(seqE.Value(i));
        }
        context.Replace(edge, sewd.InternalWire());
        for (var exp = new Explorer(sewd.InternalWire(), ShapeType.Edge); exp.More(); exp.Next())
        {
            Edge e = ShapeCasts.Edge(exp.Current());
            BrepTools.Update(e);
        }

        return true;
    }
}
