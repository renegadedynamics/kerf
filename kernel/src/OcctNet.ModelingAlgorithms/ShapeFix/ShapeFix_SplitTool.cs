// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_SplitTool.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_SplitTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2004-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>Splits an edge in two (or three) at given parameters and vertices, and cuts an edge's range.</summary>
/// <remarks>
/// A stateless working object, used by ShapeFix_Wire and ShapeFix_ComposeShell on shapes they own: its methods write
/// into the edges and vertices they are given (ranges, tolerances, the context), so they are internal. The new edges
/// are unfrozen.
/// </remarks>
public sealed class ShapeFix_SplitTool
{
    /// <summary>Empty constructor.</summary>
    public ShapeFix_SplitTool()
    {
    }

    /// <summary>
    /// Splits the edge at <paramref name="param"/> (of its pcurve on the face) with the vertex
    /// <paramref name="vert"/>, whose tolerance is raised to its distance from the edge if beyond
    /// <paramref name="tol3d"/>. False if the parameter is within <paramref name="tol2d"/> of an end.
    /// </summary>
    internal bool SplitEdge(in TopoDS_Edge edge, double param, in TopoDS_Vertex vert, in TopoDS_Face face,
                            ref TopoDS_Edge newE1, ref TopoDS_Edge newE2, double tol3d, double tol2d)
    {
        var sae = new ShapeAnalysis_Edge();
        sae.PCurve(edge, face, out Geom2d_Curve? c2d, out double a, out double b, true);
        if (Math.Abs(a - param) < tol2d || Math.Abs(b - param) < tol2d)
        {
            return false;
        }
        // check distance between edge and new vertex
        gp_Pnt          P1;
        TopLoc_Location L;
        if (BRep_Tool.SameParameter(edge))
        {
            Geom_Curve? c3d = BRep_Tool.Curve(edge, out L, out double f, out double l);
            if (c3d is null)
            {
                return false;
            }
            P1 = c3d.Value(param);
            if (!L.IsIdentity())
            {
                P1 = P1.Transformed(L.Transformation());
            }
        }
        else
        {
            Geom_Surface? surf = BRep_Tool.Surface(face, out L);
            var           sas  = new ShapeAnalysis_Surface(surf);
            P1 = sas.Value(c2d!.Value(param));
            if (!L.IsIdentity())
            {
                P1 = P1.Transformed(L.Transformation());
            }
        }
        gp_Pnt P2 = BRep_Tool.Pnt(vert);
        if (P1.Distance(P2) > tol3d)
        {
            // return false;
            var BV = new BRep_Builder();
            BV.UpdateVertex(vert, P1.Distance(P2));
        }

        var transferParameters = new ShapeAnalysis_TransferParametersProj();
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

        var                sbe    = new ShapeBuild_Edge();
        var                sfe    = new ShapeFix_Edge();
        TopAbs_Orientation orient = edge.Orientation();
        var                B      = new BRep_Builder();
        TopoDS_Edge        wE     = edge;
        wE.Orientation(TopAbs_FORWARD);
        TopoDS_Shape aTmpShape = vert.Oriented(TopAbs_REVERSED); // for porting
        newE1 = sbe.InternalCopyReplaceVertices(wE, sae.FirstVertex(wE), TopoDS.Vertex(aTmpShape));
        sbe.CopyPCurves(newE1, wE);
        transferParameters.TransferRange(newE1, first, param, true);
        B.SameRange(newE1, false);
        sfe.FixSameParameter(newE1);
        // B.SameParameter(newE1,false);
        aTmpShape = vert.Oriented(TopAbs_FORWARD);
        newE2 = sbe.InternalCopyReplaceVertices(wE, TopoDS.Vertex(aTmpShape), sae.LastVertex(wE));
        sbe.CopyPCurves(newE2, wE);
        transferParameters.TransferRange(newE2, param, last, true);
        B.SameRange(newE2, false);
        sfe.FixSameParameter(newE2);
        // B.SameParameter(newE2,false);

        newE1.Orientation(orient);
        newE2.Orientation(orient);
        if (orient == TopAbs_REVERSED)
        {
            TopoDS_Edge tmp = newE2;
            newE2 = newE1;
            newE1 = tmp;
        }

        return true;
    }

    /// <summary>
    /// Splits the edge at the middle of [param1, param2] with the vertex, then cuts the two parts at param1 and
    /// param2.
    /// </summary>
    internal bool SplitEdge(in TopoDS_Edge edge, double param1, double param2, in TopoDS_Vertex vert, in TopoDS_Face face,
                            ref TopoDS_Edge newE1, ref TopoDS_Edge newE2, double tol3d, double tol2d)
    {
        double param = (param1 + param2) / 2;
        if (SplitEdge(edge, param, vert, face, ref newE1, ref newE2, tol3d, tol2d))
        {
            // cut new edges by param1 and param2
            bool IsCutLine = false;
            var  sae       = new ShapeAnalysis_Edge();
            if (sae.PCurve(newE1, face, out Geom2d_Curve? Crv1, out double fp1, out double lp1, false))
            {
                if (sae.PCurve(newE2, face, out Geom2d_Curve? Crv2, out double fp2, out double lp2, false))
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
            return true;
        }
        return false;
    }

    /// <summary>
    /// Cuts the edge's range to [pend, cut] (as ordered); <paramref name="iscutline"/> tells whether a trimmed line
    /// pcurve of a non-SameParameter edge was cut by its 3D range. Writes into the edge.
    /// </summary>
    internal bool CutEdge(in TopoDS_Edge edge, double pend, double cut, in TopoDS_Face face, ref bool iscutline)
    {
        if (Math.Abs(cut - pend) < 10.0 * Occt.Precision.PConfusion())
        {
            return false;
        }
        double aRange = Math.Abs(cut - pend);
        BRep_Tool.Range(edge, out double a, out double b);
        iscutline = false;
        if (aRange < 10.0 * Occt.Precision.PConfusion())
        {
            return false;
        }

        // case pcurve is trimm of line
        if (!BRep_Tool.SameParameter(edge))
        {
            var sae = new ShapeAnalysis_Edge();
            if (sae.PCurve(edge, face, out Geom2d_Curve? Crv, out double fp, out double lp, false))
            {
                if (Crv is Geom2d_TrimmedCurve tc)
                {
                    if (tc.BasisCurve() is Geom2d_Line)
                    {
                        var BL = new BRep_Builder();
                        BL.Range(edge, Min(pend, cut), Max(pend, cut));
                        if (Math.Abs(pend - lp) < Occt.Precision.PConfusion())
                        { // cut from the beginning
                            double cut3d = (cut - fp) * (b - a) / (lp - fp);
                            if (cut3d <= Occt.Precision.PConfusion())
                            {
                                return false;
                            }
                            BL.Range(edge, a + cut3d, b, true);
                            iscutline = true;
                        }
                        else if (Math.Abs(pend - fp) < Occt.Precision.PConfusion())
                        { // cut from the end
                            double cut3d = (lp - cut) * (b - a) / (lp - fp);
                            if (cut3d <= Occt.Precision.PConfusion())
                            {
                                return false;
                            }
                            BL.Range(edge, a, b - cut3d, true);
                            iscutline = true;
                        }
                    }
                }
            }
            return true;
        }

        // det-study on 03/12/01 checking the old and new ranges
        if (Math.Abs(Math.Abs(a - b) - aRange) < Occt.Precision.PConfusion())
        {
            return false;
        }
        if (aRange < 10.0 * Occt.Precision.PConfusion())
        {
            return false;
        }

        Geom_Curve? c   = BRep_Tool.Curve(edge, out a, out b);
        var         sac = new ShapeAnalysis_Curve();
        a = Min(pend, cut);
        b = Max(pend, cut);
        double na = a, nb = b;

        var B = new BRep_Builder();
        // ValidateRange reverses the curve only when the range is reversed, which min/max excludes: the edge's
        // (frozen) curve is only read.
        if (!BRep_Tool.Degenerated(edge) && c is not null
            && sac.ValidateRange(c, ref na, ref nb, Occt.Precision.PConfusion()) && (na != a || nb != b))
        {
            B.Range(edge, na, nb, true);
            var sae = new ShapeAnalysis_Edge();
            if (sae.HasPCurve(edge, face))
            {
                B.SameRange(edge, false);
            }

            var sfe = new ShapeFix_Edge();
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
    /// <paramref name="V1"/> and <paramref name="V2"/> (merging them with the edge's own ends where they coincide,
    /// through the context); the parts are added to <paramref name="SeqE"/>, <paramref name="aNum"/> is the index of
    /// the middle part, and the context records the edge's replacement by the wire of the parts.
    /// </summary>
    internal bool SplitEdge(in TopoDS_Edge edge, double fp, in TopoDS_Vertex V1, double lp, in TopoDS_Vertex V2,
                            in TopoDS_Face face, NCollection_Sequence<TopoDS_Shape> SeqE, ref int aNum,
                            ShapeBuild_ReShape context, double tol3d, double tol2d)
    {
        if (Math.Abs(lp - fp) < tol2d)
        {
            return false;
        }
        aNum = 0;
        SeqE.Clear();
        var B   = new BRep_Builder();
        var sae = new ShapeAnalysis_Edge();
        sae.PCurve(edge, face, out _, out double a, out double b, true);
        TopoDS_Vertex VF    = sae.FirstVertex(edge);
        TopoDS_Vertex VL    = sae.LastVertex(edge);
        double        tolVF = BRep_Tool.Tolerance(VF);
        double        tolVL = BRep_Tool.Tolerance(VL);
        double        tolV1 = BRep_Tool.Tolerance(V1);
        double        tolV2 = BRep_Tool.Tolerance(V2);
        gp_Pnt        PVF   = BRep_Tool.Pnt(VF);
        gp_Pnt        PVL   = BRep_Tool.Pnt(VL);
        gp_Pnt        PV1   = BRep_Tool.Pnt(V1);
        gp_Pnt        PV2   = BRep_Tool.Pnt(V2);

        double par1, par2;
        bool   IsReverse = false;
        if ((b - a) * (lp - fp) > 0)
        {
            par1 = fp;
            par2 = lp;
        }
        else
        {
            par1      = lp;
            par2      = fp;
            IsReverse = true;
        }

        if (Math.Abs(a - par1) <= tol2d && Math.Abs(b - par2) <= tol2d)
        {
            if (IsReverse)
            {
                double newtol = tolVF + PVF.Distance(PV2);
                if (tolV2 < newtol)
                {
                    B.UpdateVertex(V2, newtol);
                }
                if (VF.Orientation() == V2.Orientation())
                {
                    context.Replace(VF, V2);
                    VF = V2;
                }
                else
                {
                    context.Replace(VF, V2.Reversed());
                    VF = TopoDS.Vertex(V2.Reversed());
                }
                newtol = tolVL + PVL.Distance(PV1);
                if (tolV1 < newtol)
                {
                    B.UpdateVertex(V1, newtol);
                }
                if (VL.Orientation() == V1.Orientation())
                {
                    context.Replace(VL, V1);
                    VL = V1;
                }
                else
                {
                    context.Replace(VL, V1.Reversed());
                    VL = TopoDS.Vertex(V1.Reversed());
                }
            }
            else
            {
                double newtol = tolVF + PVF.Distance(PV1);
                if (tolV1 < newtol)
                {
                    B.UpdateVertex(V1, newtol);
                }
                if (VF.Orientation() == V1.Orientation())
                {
                    context.Replace(VF, V1);
                    VF = V1;
                }
                else
                {
                    context.Replace(VF, V1.Reversed());
                    VF = TopoDS.Vertex(V1.Reversed());
                }
                newtol = tolVL + PVL.Distance(PV2);
                if (tolV2 < newtol)
                {
                    B.UpdateVertex(V2, newtol);
                }
                if (VL.Orientation() == V2.Orientation())
                {
                    context.Replace(VL, V2);
                    VL = V2;
                }
                else
                {
                    context.Replace(VL, V2.Reversed());
                    VL = TopoDS.Vertex(V2.Reversed());
                }
            }
            SeqE.Append(edge);
            aNum = 1;
        }

        if (Math.Abs(a - par1) <= tol2d && Math.Abs(b - par2) > tol2d)
        {
            TopoDS_Edge newE1 = new TopoDS_Edge(), newE2 = new TopoDS_Edge();
            if (IsReverse)
            {
                if (!SplitEdge(edge, par2, V1, face, ref newE1, ref newE2, tol3d, tol2d))
                {
                    return false;
                }
                double newtol = tolVF + PVF.Distance(PV2);
                if (tolV2 < newtol)
                {
                    B.UpdateVertex(V2, newtol);
                }
                if (VF.Orientation() == V2.Orientation())
                {
                    context.Replace(VF, V2);
                    VF = V2;
                }
                else
                {
                    context.Replace(VF, V2.Reversed());
                    VF = TopoDS.Vertex(V2.Reversed());
                }
            }
            else
            {
                if (!SplitEdge(edge, par2, V2, face, ref newE1, ref newE2, tol3d, tol2d))
                {
                    return false;
                }
                double newtol = tolVF + PVF.Distance(PV1);
                if (tolV1 < newtol)
                {
                    B.UpdateVertex(V1, newtol);
                }
                if (VF.Orientation() == V1.Orientation())
                {
                    context.Replace(VF, V1);
                    VF = V1;
                }
                else
                {
                    context.Replace(VF, V1.Reversed());
                    VF = TopoDS.Vertex(V1.Reversed());
                }
            }
            SeqE.Append(newE1);
            SeqE.Append(newE2);
            aNum = 1;
        }

        if (Math.Abs(a - par1) > tol2d && Math.Abs(b - par2) <= tol2d)
        {
            TopoDS_Edge newE1 = new TopoDS_Edge(), newE2 = new TopoDS_Edge();
            if (IsReverse)
            {
                if (!SplitEdge(edge, par1, V2, face, ref newE1, ref newE2, tol3d, tol2d))
                {
                    return false;
                }
                double newtol = tolVL + PVL.Distance(PV1);
                if (tolV1 < newtol)
                {
                    B.UpdateVertex(V1, newtol);
                }
                if (VL.Orientation() == V1.Orientation())
                {
                    context.Replace(VL, V1);
                    VL = V1;
                }
                else
                {
                    context.Replace(VL, V1.Reversed());
                    VL = TopoDS.Vertex(V1.Reversed());
                }
            }
            else
            {
                if (!SplitEdge(edge, par1, V1, face, ref newE1, ref newE2, tol3d, tol2d))
                {
                    return false;
                }
                double newtol = tolVL + PVL.Distance(PV2);
                if (tolV2 < newtol)
                {
                    B.UpdateVertex(V2, newtol);
                }
                if (VL.Orientation() == V2.Orientation())
                {
                    context.Replace(VL, V2);
                    VL = V2;
                }
                else
                {
                    context.Replace(VL, V2.Reversed());
                    VL = TopoDS.Vertex(V2.Reversed());
                }
            }
            SeqE.Append(newE1);
            SeqE.Append(newE2);
            aNum = 2;
        }

        if (Math.Abs(a - par1) > tol2d && Math.Abs(b - par2) > tol2d)
        {
            TopoDS_Edge newE1 = new TopoDS_Edge(), newE2 = new TopoDS_Edge(), newE3 = new TopoDS_Edge(),
                        newE4 = new TopoDS_Edge();
            if (IsReverse)
            {
                if (!SplitEdge(edge, par1, V2, face, ref newE1, ref newE2, tol3d, tol2d))
                {
                    return false;
                }
                if (!SplitEdge(newE2, par2, V1, face, ref newE3, ref newE4, tol3d, tol2d))
                {
                    return false;
                }
            }
            else
            {
                if (!SplitEdge(edge, par1, V1, face, ref newE1, ref newE2, tol3d, tol2d))
                {
                    return false;
                }
                if (!SplitEdge(newE2, par2, V2, face, ref newE3, ref newE4, tol3d, tol2d))
                {
                    return false;
                }
            }
            SeqE.Append(newE1);
            SeqE.Append(newE3);
            SeqE.Append(newE4);
            aNum = 2;
        }

        if (aNum == 0)
        {
            return false;
        }

        var sewd = new ShapeExtend_WireData();
        for (int i = 1; i <= SeqE.Length(); i++)
        {
            sewd.Add(SeqE.Value(i));
        }
        context.Replace(edge, sewd.InternalWire());
        for (var exp = new TopExp_Explorer(sewd.InternalWire(), TopAbs_ShapeEnum.TopAbs_EDGE); exp.More(); exp.Next())
        {
            TopoDS_Edge E = TopoDS.Edge(exp.Current());
            BRepTools.Update(E);
        }

        return true;
    }
}
