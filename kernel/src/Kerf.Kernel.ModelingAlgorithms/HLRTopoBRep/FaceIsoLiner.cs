// Port of: src/ModelingAlgorithms/TKHLR/HLRTopoBRep/HLRTopoBRep_FaceIsoLiner.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRTopoBRep/HLRTopoBRep_FaceIsoLiner.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Classification.Hatching;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.HiddenLines.Topology;

/// <summary>
/// Builds the isoparametric lines of a face for the outlining: nbIsos lines in each direction, hatched by the face's
/// edges and internal outlines, with their end vertices inserted on those edges.
/// </summary>
/// <remarks>
/// OCCT's static class. It writes into the face's edges' vertices (<see cref="MakeIsoLine"/> adds the point
/// representations of an isoline's ends, which may be the face's own vertices): HLRTopoBRep_OutLiner runs it in a
/// working overlay (unit 10H6). <c>OCCT_DEBUG</c> traces are left out.
/// </remarks>
internal static class FaceIsoLiner
{
    private const double IntersectorConfusion = 1.0e-10;
    private const double IntersectorTangency = 1.0e-10;
    private const double HatcherConfusion2d = 1.0e-8;
    private const double HatcherConfusion3d = 1.0e-8;
    private const double Infinite = 100.0;

    /// <summary>Builds nbIsos isolines in each direction on the face F (the face number FI is only for traces).</summary>
    public static void Perform(int fi, in Face f, TopologyData ds, int nbIsos)
    {
        _ = fi; // avoid compiler warning

        double uMin, uMax, vMin, vMax, u1, u2;
        int ne = 0;

        Explorer expEdges = new();
        Face tf = f;
        tf.Orient(Forward);
        Point2D p = new();
        Point3D p1, p2;
        Vertex v1U = new(), v2U = new(), v1V = new(), v2V = new();

        Intersector2D intersector = new(IntersectorConfusion, IntersectorTangency);
        Hatcher2D hatcher = new(intersector, HatcherConfusion2d, HatcherConfusion3d, true);

        BrepTools.UVBounds(tf, out uMin, out uMax, out vMin, out vMax);
        bool infiniteUMin = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(uMin);
        bool infiniteUMax = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(uMax);
        bool infiniteVMin = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(vMin);
        bool infiniteVMax = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(vMax);

        if (infiniteUMin && infiniteUMax)
        {
            uMin = -Infinite;
            uMax = Infinite;
        }
        else if (infiniteUMin)
        {
            uMin = uMax - Infinite;
        }
        else if (infiniteUMax)
        {
            uMax = uMin + Infinite;
        }

        if (infiniteVMin && infiniteVMax)
        {
            vMin = -Infinite;
            vMax = Infinite;
        }
        else if (infiniteVMin)
        {
            vMin = vMax - Infinite;
        }
        else if (infiniteVMax)
        {
            vMax = vMin + Infinite;
        }

        for (expEdges.Init(tf, ShapeType.Edge); // Edges of the face TF
             expEdges.More();
             expEdges.Next())
        {
            ne++;
        }

        if (ds.FaceHasIntL(tf))
        { // OutLines built on face TF.
            NCollection_List<Shape>.Iterator itE = new();
            for (itE.Initialize(ds.FaceIntL(tf)); itE.More(); itE.Next())
            {
                ne++;
            }
        }

        NCollection_Array1<Shape> sh = new(1, ne);
        NCollection_Array1<bool> il = new(1, ne); // internal OutLine

        for (expEdges.Init(tf, ShapeType.Edge); expEdges.More(); expEdges.Next())
        {
            int indE;
            Edge newE = ShapeCasts.Edge(expEdges.Current());
            Curve2D pc = Tool.CurveOnSurface(newE, tf, out u1, out u2)!;
            if (Abs(pc.FirstParameter - u1) <= global::Kerf.Engine.Foundation.Precision.PConfusion()
                && Abs(pc.LastParameter - u2) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                indE = hatcher.AddElement(pc, newE.Orientation);
            }
            else
            {
                TrimmedCurve2D tpc = new(pc, u1, u2);
                CurveAdaptor2D gac = new(tpc);
                indE = hatcher.AddElement(gac, newE.Orientation);
            }
            sh[indE] = newE;
            if (ds.IsOutLFaceEdge(tf, newE))
            {
                il[indE] = true;
            }
            else
            {
                il[indE] = false;
            }
        }

        if (ds.FaceHasIntL(tf))
        { // get the internal OutLines built on face F.
            NCollection_List<Shape>.Iterator itE = new();
            for (itE.Initialize(ds.FaceIntL(tf)); itE.More(); itE.Next())
            {
                int indE;
                Edge newE = ShapeCasts.Edge(itE.Value());
                Curve2D pc = Tool.CurveOnSurface(newE, tf, out u1, out u2)!;
                if (Abs(pc.FirstParameter - u1) <= global::Kerf.Engine.Foundation.Precision.PConfusion()
                    && Abs(pc.LastParameter - u2) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    indE = hatcher.AddElement(pc, Internal);
                }
                else
                {
                    TrimmedCurve2D tpc = new(pc, u1, u2);
                    CurveAdaptor2D gac = new(tpc);
                    indE = hatcher.AddElement(gac, Internal);
                }
                sh[indE] = newE;
                il[indE] = true;
            }
        }

        //-----------------------------------------------------------------------
        // Creation des hachures.
        //-----------------------------------------------------------------------

        FaceSurfaceAdaptor surface = new(tf);
        double tolerance = Tool.Tolerance(tf);

        int iIso;
        double deltaU = Abs(uMax - uMin);
        double deltaV = Abs(vMax - vMin);
        double confusion = Min(deltaU, deltaV) * HatcherConfusion3d;
        hatcher.Confusion3d(confusion);

        //-----------------------------------------------------------------------
        // Courbes Iso U.
        //-----------------------------------------------------------------------

        double stepU = deltaU / (double)nbIsos;
        if (stepU > confusion)
        {
            double uPrm = uMin + stepU / 2.0;
            Direction2D dir = new(Direction2D.D.Y);

            for (iIso = 1; iIso <= nbIsos; iIso++)
            {
                Point2D ori = new(uPrm, 0.0);
                LineCurve2D isoLine = new(ori, dir);

                CurveAdaptor2D gac = new(isoLine);
                int indH = hatcher.AddHatching(gac);
                hatcher.Trim(indH);
                if (hatcher.TrimDone(indH) && !hatcher.TrimFailed(indH))
                {
                    hatcher.ComputeDomains(indH);
                }
                if (!hatcher.IsDone(indH))
                {
                    // OCCT_DEBUG trace left out
                    hatcher.RemHatching(indH);
                    continue;
                }

                int nbDom = hatcher.NbDomains(indH);
                if (nbDom > 0)
                {

                    for (int iDom = 1; iDom <= nbDom; iDom++)
                    {
                        HatchingDomain dom = hatcher.Domain(indH, iDom);
                        double u11 = dom.HasFirstPoint ? dom.FirstPoint().Parameter() : vMin - Infinite;
                        double u21 = dom.HasSecondPoint ? dom.SecondPoint().Parameter() : vMax + Infinite;
                        isoLine.D0(u11, out p);
                        surface.D0(p.X, p.Y, out p1);
                        isoLine.D0(u21, out p);
                        surface.D0(p.X, p.Y, out p2);
                        if (dom.HasFirstPoint)
                        { // Iso U - Premier point
                            PointOnHatching pntH = dom.FirstPoint();

                            for (int iPntE = 1; iPntE <= pntH.PointCount; iPntE++)
                            {
                                PointOnElement pntE = pntH.Point(iPntE);
                                v1U = MakeVertex(ShapeCasts.Edge(sh[pntE.Index()]), p1, pntE.Parameter(), tolerance, ds);
                                if (il[pntE.Index()])
                                {
                                    ds.AddOutV(v1U);
                                }
                            }
                        }
                        if (dom.HasSecondPoint)
                        { // Iso U - Deuxieme point
                            PointOnHatching pntH = dom.SecondPoint();

                            for (int iPntE = 1; iPntE <= pntH.PointCount; iPntE++)
                            {
                                PointOnElement pntE = pntH.Point(iPntE);
                                v2U = MakeVertex(ShapeCasts.Edge(sh[pntE.Index()]), p2, pntE.Parameter(), tolerance, ds);
                                if (il[pntE.Index()])
                                {
                                    ds.AddOutV(v2U);
                                }
                            }
                        }
                        if (!v1U.IsNull && !v2U.IsNull)
                        {
                            MakeIsoLine(f, isoLine, ref v1U, ref v2U, u11, u21, tolerance, ds);
                        }
                    }
                }

                hatcher.RemHatching(indH);
                uPrm += stepU;
            }
        }

        //-----------------------------------------------------------------------
        // Courbes Iso V.
        //-----------------------------------------------------------------------

        double stepV = deltaV / (double)nbIsos;
        if (stepV > confusion)
        {
            double vPrm = vMin + stepV / 2.0;
            Direction2D dir = new(Direction2D.D.X);

            for (iIso = 1; iIso <= nbIsos; iIso++)
            {
                Point2D ori = new(0.0, vPrm);
                LineCurve2D isoLine = new(ori, dir);

                CurveAdaptor2D gac = new(isoLine);
                int indH = hatcher.AddHatching(gac);
                hatcher.Trim(indH);
                if (hatcher.TrimDone(indH) && !hatcher.TrimFailed(indH))
                {
                    hatcher.ComputeDomains(indH);
                }
                if (!hatcher.IsDone(indH))
                {
                    // OCCT_DEBUG trace left out
                    hatcher.RemHatching(indH);
                    continue;
                }

                int nbDom = hatcher.NbDomains(indH);
                if (nbDom > 0)
                {

                    for (int iDom = 1; iDom <= nbDom; iDom++)
                    {
                        HatchingDomain dom = hatcher.Domain(indH, iDom);
                        double u12 = dom.HasFirstPoint ? dom.FirstPoint().Parameter() : vMin - Infinite;
                        double u22 = dom.HasSecondPoint ? dom.SecondPoint().Parameter() : vMax + Infinite;
                        isoLine.D0(u12, out p);
                        surface.D0(p.X, p.Y, out p1);
                        isoLine.D0(u22, out p);
                        surface.D0(p.X, p.Y, out p2);
                        if (dom.HasFirstPoint)
                        { // Iso V - Premier point
                            PointOnHatching pntH = dom.FirstPoint();

                            for (int iPntE = 1; iPntE <= pntH.PointCount; iPntE++)
                            {
                                PointOnElement pntE = pntH.Point(iPntE);
                                v1V = MakeVertex(ShapeCasts.Edge(sh[pntE.Index()]), p1, pntE.Parameter(), tolerance, ds);
                                if (il[pntE.Index()])
                                {
                                    ds.AddOutV(v1V);
                                }
                            }
                        }
                        if (dom.HasSecondPoint)
                        { // Iso V - Deuxieme point
                            PointOnHatching pntH = dom.SecondPoint();

                            for (int iPntE = 1; iPntE <= pntH.PointCount; iPntE++)
                            {
                                PointOnElement pntE = pntH.Point(iPntE);
                                v2V = MakeVertex(ShapeCasts.Edge(sh[pntE.Index()]), p2, pntE.Parameter(), tolerance, ds);
                                if (il[pntE.Index()])
                                {
                                    ds.AddOutV(v2V);
                                }
                            }
                        }
                        if (!v1V.IsNull && !v2V.IsNull)
                        {
                            MakeIsoLine(f, isoLine, ref v1V, ref v2V, u12, u22, tolerance, ds);
                        }
                    }
                }

                hatcher.RemHatching(indH);
                vPrm += stepV;
            }
        }
    }

    /// <summary>
    /// The vertex at the point P of parameter Par on the edge E: one of E's end vertices or of the vertices already
    /// inserted on it if P is on it, else a new internal vertex inserted in E's list.
    /// </summary>
    public static Vertex MakeVertex(in Edge e, in Point3D p, double par, double tol,
                                           TopologyData ds)
    {
        Vertex v = new(), vf, vl;
        BrepBuilder b = new();
        Exploration.Vertices(e, out vf, out vl);
        if (p.IsEqual(Tool.Pnt(vf), Tool.Tolerance(vf)))
        {
            return vf;
        }
        if (p.IsEqual(Tool.Pnt(vl), Tool.Tolerance(vl)))
        {
            return vl;
        }

        for (ds.InitVertex(e); ds.MoreVertex(); ds.NextVertex())
        {
            Vertex curV = ds.Vertex();
            double curP = ds.Parameter();
            if (p.IsEqual(Tool.Pnt(curV), Tool.Tolerance(curV)))
            {
                v = curV;
                break;
            }
            else if (par < curP)
            {
                b.MakeVertex(ref v, p, tol);
                v.Orient(Internal);
                ds.InsertBefore(v, par);
                break;
            }
        }

        if (!ds.MoreVertex())
        {
            b.MakeVertex(ref v, p, tol);
            v.Orient(Internal);
            ds.Append(v, par);
        }

        return v;
    }

    /// <summary>Builds the isoline edge between V1 and V2 (parameters U1 and U2 on Iso) and records it as F's isoline.</summary>
    public static void MakeIsoLine(in Face f, LineCurve2D iso, ref Vertex v1, ref Vertex v2,
                                   double u1, double u2, double tol, TopologyData ds)
    {
        BrepBuilder b = new();
        Edge e = new();
        e.Orient(Internal);
        v1.Orient(Forward);
        v2.Orient(Reversed);
        b.MakeEdge(ref e);
        b.UpdateEdge(e, iso, f, tol);
        b.Add(e, v1);
        b.UpdateVertex(v1, u1, e, tol);
        b.Add(e, v2);
        b.UpdateVertex(v2, u2, e, tol);
        ds.AddIsoL(f).Append(e);
    }
}
