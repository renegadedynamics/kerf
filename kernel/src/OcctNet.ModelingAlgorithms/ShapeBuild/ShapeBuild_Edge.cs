// Port of: src/ModelingAlgorithms/TKShHealing/ShapeBuild/ShapeBuild_Edge.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeBuild/ShapeBuild_Edge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.TransformForm;
using static Kerf.Topology.Orientation;

namespace Occt;

/// <summary>
/// Tools for building and changing edges: copies with other vertices, copying ranges and pcurves between edges,
/// removing, replacing and reassigning pcurves, transforming a pcurve, building the 3D curve, and making edges.
/// </summary>
/// <remarks>
/// <para>The methods that create an edge (<see cref="CopyReplaceVertices"/>, <see cref="Copy"/>, the
/// <c>MakeEdge</c> overloads) return it frozen; their Internal forms return it unfrozen, for ported code that goes on
/// writing into it.</para>
/// <para>The methods that write into the edges they are given (CopyRanges, SetRange3d, CopyPCurves, RemovePCurve,
/// ReplacePCurve, ReassignPCurve, RemoveCurve3d, BuildCurve3d) are internal: they throw on a frozen edge. Their public
/// functional twins (ShapeBuild_Edge.Twins.cs) return the changed edge and leave theirs unchanged.</para>
/// </remarks>
public sealed partial class ShapeBuild_Edge
{
    /// <summary>
    /// A copy of the edge (new TShape, the same representations) with the vertices <paramref name="V1"/> (first) and
    /// <paramref name="V2"/> (last); a null one keeps the edge's own. With both null, INTERNAL and EXTERNAL vertices
    /// are kept as well. Frozen.
    /// </summary>
    public Edge CopyReplaceVertices(in Edge edge, in Vertex V1, in Vertex V2)
    {
        return frozen(InternalCopyReplaceVertices(edge, V1, V2));
    }

    /// <summary>OCCT's CopyReplaceVertices: the new edge, not frozen.</summary>
    internal Edge InternalCopyReplaceVertices(in Edge edge, in Vertex V1, in Vertex V2)
    {
        var aNMVertices = new NCollection_Sequence<Shape>();
        Vertex newV1 = V1, newV2 = V2;
        if (newV1.IsNull || newV2.IsNull)
        {
            var it = new Iterator();
            if (edge.Orientation() == Forward || edge.Orientation() == Reversed)
            {
                it.Initialize(edge, true, true);
            }
            else
            {
                it.Initialize(edge, false, true);
            }

            for (; it.More(); it.Next())
            {
                Vertex V = ShapeCasts.Vertex(it.Value());
                if (V.Orientation() == Forward)
                {
                    if (newV1.IsNull)
                    {
                        newV1 = V;
                    }
                }
                else if (V.Orientation() == Reversed)
                {
                    if (newV2.IsNull)
                    {
                        newV2 = V;
                    }
                }
                else if (V1.IsNull && V2.IsNull)
                {
                    aNMVertices.Append(V);
                }
            }
        }

        newV1.Orientation(Forward);
        newV2.Orientation(Reversed);

        Shape sh = edge.EmptyCopied();
        Edge E = ShapeCasts.Edge(sh);

        var B = new BrepBuilder();
        if (!newV1.IsNull)
        {
            B.Add(E, newV1);
        }

        if (!newV2.IsNull)
        {
            B.Add(E, newV2);
        }

        // addition of the internal or external vertices to edge
        int i = 1;
        for (; i <= aNMVertices.Length(); i++)
        {
            B.Add(E, ShapeCasts.Vertex(aNMVertices.Value(i)));
        }

        // S4054, rln 17.11.98 annie_surf.igs entity D77, 3D and pcurve have different
        // ranges, after B.Range all the ranges become as 3D
        CopyRanges(E, edge);

        return E;
    }

    // Added, cause invoke ShapeAnalysis leads to cyclic dependency.
    // (OCCT's own copy of ShapeAnalysis::AdjustByPeriod, which truncates with (int) where that one takes floor.)
    private static double AdjustByPeriod(double Val, double ToVal, double Period)
    {
        double diff = Val - ToVal;
        double D = Math.Abs(diff);
        double P = Math.Abs(Period);
        if (D <= 0.5 * P)
        {
            return 0.0;
        }

        if (P < 1e-100)
        {
            return diff;
        }

        return (diff > 0 ? -P : P) * (int)(D / P + 0.5);
    }

    private static bool IsPeriodic(Curve3D theCurve)
    {
        // 15.11.2002 PTV OCC966
        // remove regressions in DE tests (diva, divb, divc, toe3) in KAS:dev
        // ask IsPeriodic on BasisCurve
        Curve3D aTmpCurve = theCurve;
        while ((aTmpCurve is OffsetCurve3D) || (aTmpCurve is TrimmedCurve3D))
        {
            if (aTmpCurve is OffsetCurve3D anOffset)
            {
                aTmpCurve = anOffset.BasisCurve();
            }

            if (aTmpCurve is TrimmedCurve3D aTrimmed)
            {
                aTmpCurve = aTrimmed.BasisCurve();
            }
        }

        return aTmpCurve.IsPeriodic;
    }

    private static bool IsPeriodic(Curve2D theCurve)
    {
        // 15.11.2002 PTV OCC966
        // remove regressions in DE tests (diva, divb, divc, toe3) in KAS:dev
        // ask IsPeriodic on BasisCurve
        Curve2D aTmpCurve = theCurve;
        while ((aTmpCurve is OffsetCurve2D) || (aTmpCurve is TrimmedCurve2D))
        {
            if (aTmpCurve is OffsetCurve2D anOffset)
            {
                aTmpCurve = anOffset.BasisCurve();
            }

            if (aTmpCurve is TrimmedCurve2D aTrimmed)
            {
                aTmpCurve = aTrimmed.BasisCurve();
            }
        }

        return aTmpCurve.IsPeriodic;
    }

    /// <summary>
    /// Sets the ranges of the 3D curve and pcurves of <paramref name="toedge"/> to the part [alpha, beta] (as
    /// fractions) of the ranges of the matching curves of <paramref name="fromedge"/> (same surface and location for
    /// pcurves); on periodic curves the range is shifted into the period, and the edge loses SameRange and
    /// SameParameter.
    /// </summary>
    /// <remarks>Internal: it writes into <paramref name="toedge"/>, which throws if it is frozen.</remarks>
    internal void CopyRanges(in Edge toedge, in Edge fromedge, double alpha = 0, double beta = 1)
    {
        // OCCT.NET: frozen: fromedge's list is only read (OCCT takes it with ChangeCurves)
        for (var fromitcr = new NCollection_List<CurveRepresentation>.Iterator(((BrepTEdge)fromedge.TShape()!).Curves());
             fromitcr.More();
             fromitcr.Next())
        {
            if (fromitcr.Value() is not GCurve fromGC)
            {
                continue;
            }

            bool isC3d = fromGC.IsCurve3D;
            if (isC3d)
            {
                if (fromGC.Curve3D() is null)
                {
                    continue;
                }
            }
            else
            {
                if (fromGC.PCurve() is null)
                {
                    continue;
                }
            }

            if (!isC3d && !fromGC.IsCurveOnSurface())
            {
                continue; // only 3d curves and pcurves are treated
            }

            Surface? surface = null;
            Placement L = new Placement();
            if (!isC3d)
            {
                surface = fromGC.Surface();
                L = fromGC.Location();
            }

            NCollection_List<CurveRepresentation> tolist = ((BrepTEdge)toedge.TShape()!).ChangeCurves();
            for (var toitcr = new NCollection_List<CurveRepresentation>.Iterator(tolist); toitcr.More(); toitcr.Next())
            {
                if (toitcr.Value() is not GCurve toGC)
                {
                    continue;
                }

                if (isC3d)
                {
                    if (!toGC.IsCurve3D)
                    {
                        continue;
                    }
                }
                else if (!toGC.IsCurveOnSurface() || surface != toGC.Surface() || L != toGC.Location())
                {
                    continue;
                }

                double first = fromGC.First();
                double last = fromGC.Last();
                double len = last - first;
                double newF = first + alpha * len;
                double newL = first + beta * len;

                // PTV: 22.03.2002 fix for edge range.
                // file test-m020306-v2.step Shell #665 (Faces #40110. #40239).
                double aPeriod = 1.0, aCrvF = 0.0, aCrvL = 1.0;
                bool doCheck = false;
                if (toGC is Curve3DRepresentation aToC3d)
                {
                    Curve3D? aCrv3d = aToC3d.Curve3D();
                    // 15.11.2002 PTV OCC966
                    if (aCrv3d is not null && IsPeriodic(aCrv3d))
                    {
                        aPeriod = aCrv3d.Period();
                        aCrvF = aCrv3d.FirstParameter();
                        aCrvL = aCrv3d.LastParameter();
                        doCheck = true;
                    }
                }
                else if (toGC is CurveOnSurfaceRepresentation aToCOS)
                {
                    Curve2D? aCrv2d = aToCOS.PCurve();
                    // 15.11.2002 PTV OCC966
                    if (aCrv2d is not null && IsPeriodic(aCrv2d))
                    {
                        aPeriod = aCrv2d.Period();
                        aCrvF = aCrv2d.FirstParameter();
                        aCrvL = aCrv2d.LastParameter();
                        doCheck = true;
                    }
                }

                if (doCheck && ((Math.Abs(newF - aCrvF) > global::Kerf.Engine.Foundation.Precision.PConfusion() && newF < aCrvF) || newF >= aCrvL))
                {
                    double aShift = AdjustByPeriod(newF, 0.5 * (aCrvF + aCrvL), aPeriod);
                    newF += aShift;
                    newL += aShift;
                    new BrepBuilder().SameRange(toedge, false);
                    new BrepBuilder().SameParameter(toedge, false);
                }

                toGC.SetRange(newF, newL);
                break;
            }
        }
    }

    /// <summary>Sets the range of the edge's 3D curve.</summary>
    /// <remarks>Internal: it writes into the edge, which throws if it is frozen.</remarks>
    internal void SetRange3d(in Edge edge, double first, double last)
    {
        for (var itcr = new NCollection_List<CurveRepresentation>.Iterator(((BrepTEdge)edge.TShape()!).ChangeCurves());
             itcr.More();
             itcr.Next())
        {
            if (itcr.Value() is not GCurve GC || !GC.IsCurve3D)
            {
                continue;
            }

            GC.SetRange(first, last);
            break;
        }
    }

    /// <summary>
    /// Copies the pcurves of <paramref name="fromedge"/> into <paramref name="toedge"/> (each pcurve copied, replacing
    /// a pcurve on the same surface and location, with the location adjusted to the edges' locations).
    /// </summary>
    /// <remarks>Internal: it writes into <paramref name="toedge"/>, which throws if it is frozen.</remarks>
    internal void CopyPCurves(in Edge toedge, in Edge fromedge)
    {
        Placement fromLoc = fromedge.Location();
        Placement toLoc = toedge.Location();
        // OCCT.NET: frozen: fromedge's list is only read (OCCT takes it with ChangeCurves)
        for (var fromitcr = new NCollection_List<CurveRepresentation>.Iterator(((BrepTEdge)fromedge.TShape()!).Curves());
             fromitcr.More();
             fromitcr.Next())
        {
            if (fromitcr.Value() is not GCurve fromGC)
            {
                continue;
            }

            if (fromGC.IsCurveOnSurface())
            {
                Surface? surface = fromGC.Surface();
                Placement L = fromGC.Location();
                bool found = false;
                NCollection_List<CurveRepresentation> tolist = ((BrepTEdge)toedge.TShape()!).ChangeCurves();
                GCurve? toGC = null;
                for (var toitcr = new NCollection_List<CurveRepresentation>.Iterator(tolist); toitcr.More() && !found; toitcr.Next())
                {
                    toGC = toitcr.Value() as GCurve;
                    if (toGC is null || !toGC.IsCurveOnSurface() || surface != toGC.Surface() || L != toGC.Location())
                    {
                        continue;
                    }

                    found = true;
                    break;
                }

                if (!found)
                {
                    toGC = (GCurve)fromGC.Copy();
                    tolist.Append(toGC);
                }

                Curve2D pcurve = fromGC.PCurve()!;
                toGC!.PCurve((Curve2D)pcurve.Copy());

                // bug OCC209 invalid location of pcurve in the edge after copying
                Placement newLoc = (fromLoc * L).Predivided(toLoc); //(L * fromLoc).Predivided(toLoc);
                toGC.Location(newLoc);
                if (fromGC.IsCurveOnClosedSurface)
                {
                    pcurve = fromGC.PCurve2()!;
                    toGC.PCurve2((Curve2D)pcurve.Copy());
                }
            }
        }
    }

    /// <summary>
    /// A copy of the edge with its vertices (see <see cref="CopyReplaceVertices"/>); unless
    /// <paramref name="sharepcurves"/>, its pcurves are copies. Frozen.
    /// </summary>
    public Edge Copy(in Edge edge, bool sharepcurves = true)
    {
        return frozen(InternalCopy(edge, sharepcurves));
    }

    /// <summary>OCCT's Copy: the new edge, not frozen.</summary>
    internal Edge InternalCopy(in Edge edge, bool sharepcurves = true)
    {
        Vertex dummy1 = new Vertex(), dummy2 = new Vertex();
        Edge newedge = InternalCopyReplaceVertices(edge, dummy1, dummy2);
        if (!sharepcurves)
        {
            CopyPCurves(newedge, edge);
        }

        return newedge;
    }

    /// <summary>Removes the edge's pcurve (both, for a seam) on the face.</summary>
    /// <remarks>Internal: it writes into the edge, which throws if it is frozen.</remarks>
    internal void RemovePCurve(in Edge edge, in Face face)
    {
        var B = new BrepBuilder();
        Curve2D? c2dNull = null;
        //: S4136  double tol = BRep_Tool::Tolerance ( edge );
        if (Tool.IsClosed(edge, face))
        {
            B.UpdateEdge(edge, c2dNull, c2dNull, face, 0.0); //: S4136: tol
        }
        else
        {
            B.UpdateEdge(edge, c2dNull, face, 0.0); //: S4136: tol
        }
    }

    /// <summary>Removes the edge's pcurve on the surface (identity location).</summary>
    /// <remarks>Internal: it writes into the edge, which throws if it is frozen.</remarks>
    internal void RemovePCurve(in Edge edge, Surface? surf)
    {
        RemovePCurve(edge, surf, new Placement());
    }

    /// <summary>Removes the edge's pcurve (both, for a seam) on the surface at the location.</summary>
    /// <remarks>Internal: it writes into the edge, which throws if it is frozen.</remarks>
    internal void RemovePCurve(in Edge edge, Surface? surf, in Placement loc)
    {
        var B = new BrepBuilder();
        Curve2D? c2dNull = null;
        //: S4136  double tol = BRep_Tool::Tolerance ( edge );
        if (Tool.IsClosed(edge, surf, loc))
        {
            B.UpdateEdge(edge, c2dNull, c2dNull, surf, loc, 0.0); //: S4136: tol
        }
        else
        {
            B.UpdateEdge(edge, c2dNull, surf, loc, 0.0); //: S4136: tol
        }
    }

    /// <summary>
    /// Replaces the edge's pcurve on the face by <paramref name="pcurve"/> (for a seam, the one of the edge's
    /// orientation), keeping its range.
    /// </summary>
    /// <remarks>Internal: it writes into the edge, which throws if it is frozen.</remarks>
    internal void ReplacePCurve(in Edge edge, Curve2D? pcurve, in Face face)
    {
        var B = new BrepBuilder();
        double f, l;
        Shape dummy = edge.Reversed();
        Edge edgerev = ShapeCasts.Edge(dummy);
        // reverse face to take second pcurve for seams like SA_Edge::PCurve() does
        Face F = ShapeCasts.Face(face.Oriented(Forward));
        Curve2D? pcurve0 = Tool.CurveOnSurface(edge, F, out f, out l);
        Curve2D? c2d = Tool.CurveOnSurface(edgerev, F, out f, out l);
        // Add pcurve to edge (either as single, or as seam)
        if (c2d is null || c2d == pcurve0)
        { // non-seam
            B.UpdateEdge(edge, pcurve, face, 0);
        }
        else
        { // seam
            if (edge.Orientation() == Forward)
            {
                B.UpdateEdge(edge, pcurve, c2d, face, 0);
            }
            else
            {
                B.UpdateEdge(edge, c2d, pcurve, face, 0);
            }
        }

        B.Range(edge, face, f, l);
    }

    // Count exact number of pcurves STORED in edge for face
    // This makes difference for faces based on plane surfaces where pcurves can be
    // not stored but returned by BRep_Tools::CurveOnSurface
    private static int CountPCurves(in Edge edge, in Face face)
    {
        Surface? S = Tool.Surface(face, out Placement L);
        Placement l = L.Predivided(edge.Location());

        // OCCT.NET: frozen: the list is only read (OCCT takes it with ChangeCurves)
        for (var itcr = new NCollection_List<CurveRepresentation>.Iterator(((BrepTEdge)edge.TShape()!).Curves());
             itcr.More();
             itcr.Next())
        {
            if (itcr.Value() is GCurve GC && GC.IsCurveOnSurface(S, l))
            {
                return (GC.IsCurveOnClosedSurface ? 2 : 1);
            }
        }

        return 0;
    }

    /// <summary>
    /// Moves the edge's pcurve from face <paramref name="old"/> to face <paramref name="sub"/> (for a seam on old, the
    /// second pcurve stays there; on sub it is added as the first pcurve of a seam if one is there already). False if
    /// the edge has no pcurve on old.
    /// </summary>
    /// <remarks>Internal: it writes into the edge, which throws if it is frozen.</remarks>
    internal bool ReassignPCurve(in Edge edge, in Face old, in Face sub)
    {
        int npcurves = CountPCurves(edge, old);
        // if ( npcurves <1 ) return false; //gka

        double f, l;
        Curve2D? pc;
        pc = Tool.CurveOnSurface(edge, old, out f, out l);
        if (pc is null)
        {
            return false;
        }
        else if (npcurves == 0)
        {
            npcurves = 1; // gka
        }

        var B = new BrepBuilder();

        // if the pcurve was only one, remove; else leave second one
        if (npcurves > 1)
        {
            // smh#8 Porting AIX
            Shape tmpshape = edge.Reversed();
            Edge erev = ShapeCasts.Edge(tmpshape);
            Curve2D? pc2 = Tool.CurveOnSurface(erev, old, out f, out l);
            B.UpdateEdge(edge, pc2, old, 0.0);
            B.Range(edge, old, f, l);
        }
        else
        {
            RemovePCurve(edge, old);
        }

        // if edge does not have yet pcurves on sub, just add; else add as first
        int npcs = CountPCurves(edge, sub);
        if (npcs < 1)
        {
            B.UpdateEdge(edge, pc, sub, 0.0);
        }
        else
        {
            // smh#8 Porting AIX
            Shape tmpshape = edge.Reversed();
            Edge erev = ShapeCasts.Edge(tmpshape);
            Curve2D? pcs = Tool.CurveOnSurface(erev, sub, out _, out _);
            if (edge.Orientation() == Reversed)
            { // because B.UpdateEdge does not check edge orientation
                B.UpdateEdge(edge, pcs, pc, sub, 0.0);
            }
            else
            {
                B.UpdateEdge(edge, pc, pcs, sub, 0.0);
            }
        }

        B.Range(edge, sub, f, l);

        return true;
    }

    /// <summary>
    /// A new pcurve: <paramref name="pcurve"/> transformed by <paramref name="trans"/>, then scaled by
    /// <paramref name="uFact"/> along U (lines stay lines, Bezier curves Bezier curves; other curves become
    /// B-splines, conics approximated). The range in <paramref name="aFirst"/>, <paramref name="aLast"/> is
    /// transformed with it. The curve returned is a new object, not frozen, like a Geom Copy().
    /// </summary>
    public Curve2D TransformPCurve(Curve2D pcurve, in Transform2D trans, double uFact, ref double aFirst, ref double aLast)
    {
        Curve2D result = (Curve2D)pcurve.Copy();
        if (trans.Form() != Identity)
        {
            result.Transform(trans);
            aFirst = result.TransformedParameter(aFirst, trans);
            aLast = result.TransformedParameter(aLast, trans);
        }

        if (uFact == 1.0)
        {
            return result;
        }

        if (result is TrimmedCurve2D thecurve)
        {
            // OCCT.NET: frozen: BasisCurve() freezes the basis it hands out; OCCT changes the basis of its own discarded
            // copy, so the port changes a copy of it
            result = (Curve2D)thecurve.BasisCurve().Copy();
        }

        GeneralTransform2D tMatu = new GeneralTransform2D();
        tMatu.SetAffinity(new Axis2D(new Point2D(0, 0), new Direction2D(0, 1)), uFact); // gp::OY2d()
        Xy pXY;

        if (result is LineCurve2D aLine2d)
        {
            aLine2d.D0(aFirst, out Point2D Pf);
            pXY = Pf.XY();
            tMatu.Transforms(ref pXY);
            Pf.SetXY(pXY);
            aLine2d.D0(aLast, out Point2D Pl);
            pXY = Pl.XY();
            tMatu.Transforms(ref pXY);
            Pl.SetXY(pXY);
            Line2D line2d = new Line2D(Pf, new Direction2D(new Vector2D(Pf, Pl)));
            aFirst = global::Kerf.Engine.Geometry.ElCLib.Parameter(line2d, Pf);
            aLast = global::Kerf.Engine.Geometry.ElCLib.Parameter(line2d, Pl);
            LineCurve2D Gline2d = new LineCurve2D(line2d);
            return Gline2d;
        }
        else if (result is BezierCurve2D bezier)
        {
            // transform the Poles of the BSplineCurve
            int nbPol = bezier.PoleCount;
            Point2D Pt1 = new Point2D();
            for (int i = 1; i <= nbPol; i++)
            {
                pXY = bezier.Pole(i).XY();
                tMatu.Transforms(ref pXY);
                Pt1.SetXY(pXY);
                bezier.SetPole(i, Pt1);
            }

            return bezier;
        }
        else
        {
            BSplineCurve2D aBSpline2d;
            if (result is ConicCurve2D)
            {
                Curve2D tcurve = new TrimmedCurve2D(result, aFirst, aLast); // protection against parabols ets
                var approx = new ApproxCurve2D(tcurve, global::Kerf.Engine.Foundation.Precision.Approximation(), C1, 100, 6);
                if (approx.HasResult)
                {
                    aBSpline2d = approx.InternalCurve()!;
                }
                else
                {
                    aBSpline2d = Geom2dConvert2D.CurveToBSplineCurve(tcurve, QuasiAngular);
                }

                aFirst = aBSpline2d.FirstParameter();
                aLast = aBSpline2d.LastParameter();
            }
            else if (result is not BSplineCurve2D)
            {
                aBSpline2d = Geom2dConvert2D.CurveToBSplineCurve(result, QuasiAngular);
            }
            else
            {
                aBSpline2d = (BSplineCurve2D)result;
            }

            // transform the Poles of the BSplineCurve
            int nbPol = aBSpline2d.PoleCount;
            Point2D Pt1 = new Point2D();
            for (int i = 1; i <= nbPol; i++)
            {
                pXY = aBSpline2d.Pole(i).XY();
                tMatu.Transforms(ref pXY);
                Pt1.SetXY(pXY);
                aBSpline2d.SetPole(i, Pt1);
            }

            return aBSpline2d;
        }
    }

    /// <summary>Removes the edge's 3D curve.</summary>
    /// <remarks>Internal: it writes into the edge, which throws if it is frozen.</remarks>
    internal void RemoveCurve3d(in Edge edge)
    {
        var B = new BrepBuilder();
        Curve3D? c3dNull = null;
        //: S4136  double tol = BRep_Tool::Tolerance (edge);
        B.UpdateEdge(edge, c3dNull, 0.0); //: S4136: tol
    }

    /// <summary>
    /// Computes the edge's 3D curve from its pcurve (BRepLib::BuildCurve3d, tolerance at least 1e-5), and fits its
    /// range within a non-periodic curve. False if it fails.
    /// </summary>
    /// <remarks>
    /// Internal: it writes into the edge. OCCT catches every Standard_Failure; the exception of a write into a frozen
    /// edge is let through (as decision 423), so it can't pass as a plain failure.
    /// </remarks>
    internal bool BuildCurve3d(in Edge edge)
    {
        try
        {
            // #48 rln 10.12.98 S4054 UKI60107-5 entity 365
            // C0 surface (but curve 3d is required as C1) and tolerance is 1e-07
            // lets use maximum of tolerance and default parameter 1.e-5
            // Another solutions: use quite big Tolerance or require C0 curve on C0 surface
            if (BRepLib.BuildCurve3d(edge, Max(1.0e-5, Tool.Tolerance(edge))))
            {
                // #50 S4054 rln 14.12.98 write cylinder in BRep mode into IGES and read back
                // with 2DUse_Forced - pcurve and removed 3D curves have different ranges
                if (Tool.SameRange(edge))
                {
                    Tool.Range(edge, out double first, out double last);
                    new BrepBuilder().Range(edge, first, last); // explicit setting for all reps
                }

                Curve3D? c3d;
                c3d = Tool.Curve(edge, out double f, out double l);
                if (c3d is null)
                {
                    return false;
                }

                // 15.11.2002 PTV OCC966
                if (!IsPeriodic(c3d))
                {
                    bool isLess = false;
                    if (f < c3d.FirstParameter())
                    {
                        isLess = true;
                        f = c3d.FirstParameter();
                    }

                    if (l > c3d.LastParameter())
                    {
                        isLess = true;
                        l = c3d.LastParameter();
                    }

                    if (isLess)
                    {
                        SetRange3d(edge, f, l);
                        new BrepBuilder().SameRange(edge, false);
                    }
                }

                return true;
            }
        }
        catch (FailureException) when (!edge.TShape()!.IsFrozen)
        {
        }

        return false;
    }

    /// <summary>
    /// Makes <paramref name="edge"/> a new edge on the 3D curve (over its whole range) placed at
    /// <paramref name="L"/>; unchanged if BRepBuilderAPI_MakeEdge fails. Frozen.
    /// </summary>
    public void MakeEdge(ref Edge edge, Curve3D curve, in Placement L)
    {
        InternalMakeEdge(ref edge, curve, L);
        freezeNew(edge);
    }

    /// <summary>As above, over the range [p1, p2]. Frozen.</summary>
    public void MakeEdge(ref Edge edge, Curve3D curve, in Placement L, double p1, double p2)
    {
        InternalMakeEdge(ref edge, curve, L, p1, p2);
        freezeNew(edge);
    }

    /// <summary>Makes <paramref name="edge"/> a new edge on the pcurve on the face (over its whole range). Frozen.</summary>
    public void MakeEdge(ref Edge edge, Curve2D pcurve, in Face face)
    {
        InternalMakeEdge(ref edge, pcurve, face);
        freezeNew(edge);
    }

    /// <summary>As above, over the range [p1, p2]. Frozen.</summary>
    public void MakeEdge(ref Edge edge, Curve2D pcurve, in Face face, double p1, double p2)
    {
        InternalMakeEdge(ref edge, pcurve, face, p1, p2);
        freezeNew(edge);
    }

    /// <summary>Makes <paramref name="edge"/> a new edge on the pcurve on the surface at the location. Frozen.</summary>
    public void MakeEdge(ref Edge edge, Curve2D pcurve, Surface S, in Placement L)
    {
        InternalMakeEdge(ref edge, pcurve, S, L);
        freezeNew(edge);
    }

    /// <summary>As above, over the range [p1, p2]. Frozen.</summary>
    public void MakeEdge(ref Edge edge, Curve2D pcurve, Surface S, in Placement L, double p1, double p2)
    {
        InternalMakeEdge(ref edge, pcurve, S, L, p1, p2);
        freezeNew(edge);
    }

    /// <summary>OCCT's MakeEdge: the new edge, not frozen.</summary>
    internal void InternalMakeEdge(ref Edge edge, Curve3D curve, in Placement L)
    {
        InternalMakeEdge(ref edge, curve, L, curve.FirstParameter(), curve.LastParameter());
    }

    /// <summary>OCCT's MakeEdge: the new edge, not frozen.</summary>
    internal void InternalMakeEdge(ref Edge edge, Curve3D curve, in Placement L, double p1, double p2)
    {
        var ME = new BRepBuilderAPI_MakeEdge(curve, p1, p2);
        if (!ME.IsDone())
        {
            return;
        }

        Edge E = ME.InternalEdge();
        if (!L.IsIdentity)
        {
            var B = new BrepBuilder();
            B.UpdateEdge(E, curve, L, 0.0);
            B.Range(E, p1, p2);

            Exploration.Vertices(E, out Vertex V1, out Vertex V2);
            Point3D P1 = Tool.Pnt(V1), P2 = Tool.Pnt(V2);
            B.UpdateVertex(V1, P1.Transformed(L.Transformation()), 0.0);
            B.UpdateVertex(V2, P2.Transformed(L.Transformation()), 0.0);
        }

        edge = E;
    }

    /// <summary>OCCT's MakeEdge: the new edge, not frozen.</summary>
    internal void InternalMakeEdge(ref Edge edge, Curve2D pcurve, in Face face)
    {
        InternalMakeEdge(ref edge, pcurve, face, pcurve.FirstParameter(), pcurve.LastParameter());
    }

    /// <summary>OCCT's MakeEdge: the new edge, not frozen.</summary>
    internal void InternalMakeEdge(ref Edge edge, Curve2D pcurve, in Face face, double p1, double p2)
    {
        Surface S = Tool.Surface(face, out Placement L)!;
        InternalMakeEdge(ref edge, pcurve, S, L, p1, p2);
    }

    /// <summary>OCCT's MakeEdge: the new edge, not frozen.</summary>
    internal void InternalMakeEdge(ref Edge edge, Curve2D pcurve, Surface S, in Placement L)
    {
        InternalMakeEdge(ref edge, pcurve, S, L, pcurve.FirstParameter(), pcurve.LastParameter());
    }

    /// <summary>OCCT's MakeEdge: the new edge, not frozen.</summary>
    internal void InternalMakeEdge(ref Edge edge, Curve2D pcurve, Surface S, in Placement L, double p1, double p2)
    {
        var ME = new BRepBuilderAPI_MakeEdge(pcurve, S, p1, p2);
        if (!ME.IsDone())
        {
            return;
        }

        Edge E = ME.InternalEdge();
        if (!L.IsIdentity)
        {
            RemovePCurve(E, S);
            var B = new BrepBuilder();
            B.UpdateEdge(E, pcurve, S, L, 0.0);
            B.Range(E, S, L, p1, p2);

            Exploration.Vertices(E, out Vertex V1, out Vertex V2);
            Point3D P1 = Tool.Pnt(V1), P2 = Tool.Pnt(V2);
            B.UpdateVertex(V1, P1.Transformed(L.Transformation()), 0.0);
            B.UpdateVertex(V2, P2.Transformed(L.Transformation()), 0.0);
        }

        edge = E;
    }

    private static Edge frozen(Edge E)
    {
        E.TShape()!.FreezeGraph();
        return E;
    }

    private static void freezeNew(in Edge E)
    {
        if (!E.IsNull)
        {
            E.TShape()!.FreezeGraph();
        }
    }
}
