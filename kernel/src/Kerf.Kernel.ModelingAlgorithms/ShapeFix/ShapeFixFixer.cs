// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2000-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Library;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Foundation.Gravity;
using static Kerf.Engine.Healing.Extend.ExtendStatus;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Healing.Fixes;

/// <summary>
/// Shape Healing's fixing package: the tools fixing shapes (ShapeFix_Edge, ShapeFix_Wire, ...) and these statics.
/// </summary>
/// <remarks>
/// <c>SameParameter</c>, <c>EncodeRegularity</c> and <c>FixVertexPosition</c> write into the shape they are given, so
/// they are internal (ported code on shapes it owns); callers use their functional twins
/// (<see cref="SameParametered"/>, <see cref="RegularityEncoded"/>, <see cref="VertexPositionFixed"/>), which work on
/// a copy of a frozen shape, give back its unchanged sub-shapes and freeze the result (plan 2.7, decision 583).
/// <see cref="RemoveSmallEdges"/> runs ShapeFix_Shape, which copies a frozen shape itself, and freezes its result.
/// </remarks>
public static partial class ShapeFixFixer
{
    /// <summary>
    /// Runs ShapeFix_Edge's FixSameParameter on every edge of the shape (on each of its faces), then updates the
    /// tolerances of the edges on planes. Returns false if an edge couldn't be made SameParameter. With
    /// <paramref name="enforce"/>, the SameRange and SameParameter flags are reset first. Writes into the shape.
    /// </summary>
    internal static bool SameParameter(in Shape shape, bool enforce, double preci = 0.0,
                                       ProgressRange progress = default,
                                       BasicMsgRegistrator? msgReg = null)
    {
        // Calculate number of edges
        int nbEdges = 0;
        for (var edgeExp = new Explorer(shape, ShapeType.Edge); edgeExp.More(); edgeExp.Next())
        {
            ++nbEdges;
        }

        // Calculate number of faces
        int nbFaces = 0;
        for (var edgeExp = new Explorer(shape, ShapeType.Face); edgeExp.More(); edgeExp.Next())
        {
            ++nbFaces;
        }

        var mapEF = new NCollection_IndexedDataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(shape, ShapeType.Edge, ShapeType.Face, mapEF);

        var    b       = new BrepBuilder();
        bool   status  = true;
        double tol     = preci;
        bool   iatol   = (tol > 0);
        var    sfe     = new EdgeFixer();
        var    ex      = new Explorer(shape, ShapeType.Edge);
        var    doneMsg = new Msg("FixEdge.SameParameter.MSG0");

        // Start progress scope (no need to check if progress exists -- it is safe)
        using var psForSameParam = new ProgressScope(progress, "Fixing same parameter problem", 2);

        {
            // Start progress scope (no need to check if progress exists -- it is safe)
            using var ps = new ProgressScope(psForSameParam.Next(), "Fixing edge", nbEdges);

            while (ex.More())
            {
                Edge e;
                while (ex.More() && ps.More())
                {
                    int ierr = 0;
                    e = ShapeCasts.Edge(ex.Current());
                    ex.Next();

                    if (!iatol)
                    {
                        tol = Tool.Tolerance(e);
                    }
                    if (enforce)
                    {
                        b.SameRange(e, false);
                        b.SameParameter(e, false);
                    }

                    mapEF.FindFromKey(e, out NCollection_List<Shape> listOfFaces);
                    if (listOfFaces.Extent() != 0)
                    {
                        var listOfFacesIt = new NCollection_List<Shape>.Iterator(listOfFaces);
                        for (; listOfFacesIt.More(); listOfFacesIt.Next())
                        {
                            Face aF = ShapeCasts.Face(listOfFacesIt.Value());
                            sfe.FixSameParameter(e, aF);
                        }
                    }
                    else
                    {
                        sfe.FixSameParameter(e); // K2-SEP97
                    }

                    if (!Tool.SameParameter(e))
                    {
                        ierr = 1;
                    }

                    if (ierr != 0)
                    {
                        status = false;
                        b.SameRange(e, false);
                        b.SameParameter(e, false);
                    }
                    else if (msgReg is not null && !sfe.Status(ExtendStatus.OK))
                    {
                        msgReg.Send(e, doneMsg, Warning);
                    }

                    // Complete step in current progress scope
                    ps.Next();
                } // -- end while

                // Halt algorithm in case of user's abort
                if (!ps.More())
                {
                    return false;
                }
            }
        }

        {
            // Start progress scope (no need to check if progress exists -- it is safe)
            using var ps = new ProgressScope(psForSameParam.Next(), "Update tolerances", nbFaces);

            //: i2 abv 21 Aug 98: ProSTEP TR8 Motor.rle face 710:
            // Update tolerance of edges on planes (no pcurves are stored)
            for (var exp = new Explorer(shape, ShapeType.Face); exp.More() && ps.More(); exp.Next(), ps.Next())
            {
                Face   face = ShapeCasts.Face(exp.Current());
                Surface? surf = Tool.Surface(face);

                PlaneSurface? plane = surf as PlaneSurface;
                if (plane is null)
                {
                    if (surf is TrimmedSurface grts)
                    {
                        plane = grts.BasisSurface() as PlaneSurface;
                    }
                    if (plane is null)
                    {
                        continue;
                    }
                }

                var AS = new SurfaceAdaptor(plane);
                for (var ed = new Explorer(face, ShapeType.Edge); ed.More(); ed.Next())
                {
                    Edge edge = ShapeCasts.Edge(ed.Current());
                    Curve3D? crv  = Tool.Curve(edge, out double f, out double l);
                    if (crv is null)
                    {
                        continue;
                    }

                    Curve2D? c2d = Tool.CurveOnSurface(edge, face, out f, out l);
                    if (c2d is null)
                    {
                        continue;
                    }
                    var ghpc = new CurveAdaptor2D(c2d, f, l);
                    var acs  = new CurveOnSurface(ghpc, AS);

                    double    tol0      = Tool.Tolerance(edge);
                    tol = tol0;
                    double    tol2      = tol * tol;
                    bool      isChanged = false;
                    const int ncontrol  = 23;
                    for (int i = 0; i < ncontrol; i++)
                    {
                        double par  = (f * (ncontrol - 1 - i) + l * i) / (ncontrol - 1);
                        Point3D pnt  = crv.Value(par);
                        Point3D prj  = acs.Value(par);
                        double dist = pnt.SquareDistance(prj);
                        if (tol2 < dist)
                        {
                            tol2      = dist;
                            isChanged = true;
                        }
                    }
                    if (isChanged)
                    {
                        tol = 1.00005 * Math.Sqrt(tol2); // coeff: see trj3_pm1-ct-203.stp #19681, edge 10
                        if (tol >= tol0)
                        {
                            b.UpdateEdge(edge, tol);
                            for (var itV = new Iterator(edge); itV.More(); itV.Next())
                            {
                                Shape s = itV.Value();
                                b.UpdateVertex(ShapeCasts.Vertex(s), tol);
                            }
                        }
                    }
                }
            }
            // Halt algorithm in case of user's abort
            if (!ps.More())
            {
                return false;
            }
        }

        return status;
    }

    /// <summary>Runs BRepLib's EncodeRegularity on the shape. Writes into the shape.</summary>
    internal static void EncodeRegularity(in Shape shape, double tolang = 1.0e-10)
    {
        BrepLibrary.EncodeRegularity(shape, tolang);
    }

    /// <summary>
    /// Removes the edges of the shape shorter than <paramref name="tolerance"/>, with ShapeFix_Wire's FixSmall run by
    /// ShapeFix_Shape (the other wire and face fixes off but reordering). Returns the result, frozen, and the context
    /// of the fix in <paramref name="context"/>, which answers for the caller's sub-shapes. The shape itself isn't
    /// changed (ShapeFix_Shape copies a frozen shape first).
    /// </summary>
    public static Shape RemoveSmallEdges(in Shape shape, double tolerance, out ReShapeBuilder context)
    {
        ShapeFixer sfs = RemoveSmallEdgesCore(shape, tolerance);
        context = sfs.Context()!;
        return sfs.Shape();
    }

    /// <summary>
    /// <see cref="RemoveSmallEdges"/> without freezing the result, for ported code fixing a shape it owns (fixed in
    /// place, as in OCCT).
    /// </summary>
    internal static Shape InternalRemoveSmallEdges(in Shape shape, double tolerance, out ReShapeBuilder context)
    {
        ShapeFixer sfs = RemoveSmallEdgesCore(shape, tolerance);
        context = sfs.Context()!;
        return sfs.InternalShape();
    }

    private static ShapeFixer RemoveSmallEdgesCore(in Shape shape, double tolerance)
    {
        var sfs = new ShapeFixer();
        sfs.Init(shape);
        sfs.SetPrecision(tolerance);
        sfs.FixFaceTool().FixMissingSeamMode()   = 0;
        sfs.FixFaceTool().FixOrientationMode()   = 0;
        sfs.FixFaceTool().FixSmallAreaWireMode() = 0;
        sfs.FixWireTool().ModifyTopologyMode()   = true;
        // sfs.FixWireTool().FixReorderMode() = false;
        sfs.FixWireTool().FixConnectedMode()        = 0;
        sfs.FixWireTool().FixEdgeCurvesMode()       = 0;
        sfs.FixWireTool().FixDegeneratedMode()      = 0;
        sfs.FixWireTool().FixSelfIntersectionMode() = 0;
        sfs.FixWireTool().FixLackingMode()          = 0;
        sfs.FixWireTool().FixSmallMode()            = 1;
        sfs.Perform();
        return sfs;
    }

    //=======================================================================
    // function : ReplaceVertex
    // purpose  : auxiliary for FixVertexPosition
    //=======================================================================
    private static Edge ReplaceVertex(in Edge edge, in Point3D p, bool fwd)
    {
        var newVertex = new Vertex();
        var b         = new BrepBuilder();
        b.MakeVertex(ref newVertex, p, Kerf.Engine.Foundation.Precision.Confusion());
        var v1 = new Vertex();
        var v2 = new Vertex();
        if (fwd)
        {
            v1 = newVertex;
            v1.Orientation(Forward);
        }
        else
        {
            v2 = newVertex;
            v2.Orientation(Reversed);
        }
        var                sbe = new BuildEdgeBuilder();
        Edge        e1   = edge;
        Orientation ori  = e1.Orientation();
        e1.Orientation(Forward);
        Edge newEdge = sbe.InternalCopyReplaceVertices(e1, v1, v2);
        newEdge.Orientation(ori);
        return newEdge;
    }

    //=======================================================================
    // function : getNearPoint
    // purpose  : auxiliary for FixVertexPosition
    //=======================================================================
    private static double GetNearPoint(NCollection_Sequence<Point3D> aSeq1, NCollection_Sequence<Point3D> aSeq2,
                                       ref Xyz acent)
    {
        int    i    = 1;
        int    ind1 = 0, ind2 = 0;
        double mindist = RealLast();
        for (; i <= aSeq1.Length(); i++)
        {
            Point3D p1 = aSeq1.Value(i);
            int    j  = 1;
            for (; j <= aSeq2.Length(); j++)
            {
                Point3D p2 = aSeq2.Value(j);
                double d  = p1.Distance(p2);
                if (Math.Abs(d - mindist) <= Kerf.Engine.Foundation.Precision.Confusion())
                {
                    continue;
                }
                if (d < mindist)
                {
                    mindist = d;
                    ind1    = i;
                    ind2    = j;
                }
            }
        }
        if (ind1 != 0 && ind2 != 0)
        {
            acent = (aSeq1.Value(ind1).XYZ() + aSeq2.Value(ind2).XYZ()) / 2.0;
        }
        return mindist;
    }

    //=======================================================================
    // function : getNearestEdges
    // purpose  : auxiliary for FixVertexPosition
    //=======================================================================
    private static bool GetNearestEdges(NCollection_List<Shape> lEdges,
                                        in Vertex vert,
                                        NCollection_Sequence<Shape> suitEdges,
                                        NCollection_Sequence<Shape> rejectEdges,
                                        double tolerance,
                                        ref Xyz thecentersuit,
                                        ref Xyz thecenterreject)
    {
        if (lEdges.IsEmpty())
        {
            return false;
        }
        var mapEdges = new HashSet<Shape>(ShapeHasher.Instance);

        var atempList = new NCollection_List<Shape>(lEdges);
        var alIter    = new NCollection_List<Shape>.Iterator(atempList);

        Edge edge1 = ShapeCasts.Edge(alIter.Value());
        Exploration.Vertices(edge1, out Vertex vert11, out Vertex vert12);
        mapEdges.Add(edge1);
        Curve3D? curve1  = Tool.Curve(edge1, out double first1, out double last1);
        var         p11      = new Point3D();
        var         p12      = new Point3D();
        bool        isFirst1 = vert.IsSame(vert11);
        bool        isSame1  = vert11.IsSame(vert12);
        if (curve1 is not null)
        {
            if (isFirst1)
            {
                p11 = curve1.Value(first1);
            }
            else if (!isSame1)
            {
                p11 = curve1.Value(last1);
            }
            if (isSame1)
            {
                p12 = curve1.Value(last1);
            }
        }
        else
        {
            return false;
        }
        alIter.Next();
        var aseqreject = new NCollection_Sequence<Shape>();
        var aseqsuit   = new NCollection_Sequence<Shape>();

        int anumLoop = 0;
        for (; alIter.More();)
        {
            Edge edge = ShapeCasts.Edge(alIter.Value());
            if (mapEdges.Contains(edge))
            {
                atempList.Remove(alIter);
                continue;
            }

            Exploration.Vertices(edge, out Vertex vert1, out Vertex vert2);
            bool isFirst = vert.IsSame(vert1); // OCCT declares it double
            bool isSame  = vert1.IsSame(vert2);

            bool isLoop = ((vert1.IsSame(vert11) && vert2.IsSame(vert12))
                           || (vert1.IsSame(vert12) && vert2.IsSame(vert11)));
            if (isLoop /*&& !aseqsuit.Length()*/ && (atempList.Extent() > anumLoop))
            {
                atempList.Append(edge);
                atempList.Remove(alIter);
                anumLoop++;
                continue;
            }
            mapEdges.Add(edge);
            Curve3D? curve = Tool.Curve(edge, out double first, out double last);
            if (curve is not null)
            {
                Point3D p1;
                var    p2 = new Point3D();
                if (isFirst)
                {
                    p1 = curve.Value(first);
                }
                else
                {
                    p1 = curve.Value(last);
                }
                if (isSame)
                {
                    p2 = curve.Value(last);
                }
                double minDist = RealLast();
                var    acent    = new Xyz();
                if (!isSame && !isSame1)
                {
                    minDist = p1.Distance(p11);
                    acent    = (p1.XYZ() + p11.XYZ()) / 2.0;
                }
                else
                {
                    var seq1 = new NCollection_Sequence<Point3D>();
                    var seq2 = new NCollection_Sequence<Point3D>();
                    seq1.Append(p11);
                    if (isSame1)
                    {
                        seq1.Append(p12);
                    }
                    seq2.Append(p1);
                    if (isSame)
                    {
                        seq2.Append(p2);
                    }
                    minDist = GetNearPoint(seq1, seq2, ref acent);
                }

                if (minDist > tolerance)
                {
                    if (aseqreject.Length() == 0)
                    {
                        thecenterreject = acent;
                    }
                    aseqreject.Append(edge);
                }
                else
                {
                    if (aseqsuit.Length() == 0)
                    {
                        thecentersuit = acent;
                        aseqsuit.Append(edge);
                    }
                    else if (!isSame1)
                    {
                        aseqsuit.Append(edge);
                    }
                    else if ((thecentersuit - acent).Modulus() < tolerance)
                    {
                        aseqsuit.Append(edge);
                    }
                    else
                    {
                        aseqreject.Append(edge);
                    }
                }
            }
            atempList.Remove(alIter);
        }

        bool isDone = (!aseqsuit.IsEmpty() || !aseqreject.IsEmpty());
        if (isDone)
        {
            if (aseqsuit.IsEmpty())
            {
                rejectEdges.Append(edge1);
                lEdges.RemoveFirst();

                GetNearestEdges(lEdges,
                                vert,
                                suitEdges,
                                rejectEdges,
                                tolerance,
                                ref thecentersuit,
                                ref thecenterreject);
            }
            else
            {
                suitEdges.Append(edge1);
                suitEdges.Append(aseqsuit);
                rejectEdges.Append(aseqreject);
            }
        }
        else
        {
            rejectEdges.Append(edge1);
        }

        return isDone;
    }

    /// <summary>
    /// Moves the vertices more tolerant than <paramref name="tolerance"/> to the ends of the curves near them, by
    /// replacing them (and edges whose ends are too far apart) in the context; the shape becomes the context applied
    /// to it if anything was done. Writes the vertices' Modified flag.
    /// </summary>
    internal static bool FixVertexPosition(ref Shape theshape, double tolerance, ReShapeBuilder thecontext)
    {
        var mapVertEdges = new NCollection_IndexedDataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        var exp1         = new Explorer(theshape, ShapeType.Edge);
        for (; exp1.More(); exp1.Next())
        {
            var vert1 = new Shape();
            int nV     = 1;
            var exp3  = new Iterator(exp1.Current());
            for (; exp3.More(); exp3.Next(), nV++)
            {
                Shape vert = exp3.Value();
                if (nV == 1)
                {
                    vert1 = vert;
                }
                else if (vert1.IsSame(vert))
                {
                    continue;
                }
                if (mapVertEdges.Contains(vert))
                {
                    mapVertEdges.ChangeFromKey(vert).Append(exp1.Current());
                }
                else
                {
                    var alEdges = new NCollection_List<Shape>();
                    alEdges.Append(exp1.Current());
                    mapVertEdges.Add(vert, alEdges);
                }
            }
        }
        bool isDone = false;
        int  i      = 1;
        for (; i <= mapVertEdges.Extent(); i++)
        {
            Vertex vert    = ShapeCasts.Vertex(mapVertEdges.FindKey(i));
            double        tolVert = Tool.Tolerance(vert);
            if (tolVert <= tolerance)
            {
                continue;
            }

            var b1 = new BrepBuilder();
            b1.UpdateVertex(vert, tolerance);
            Point3D pvert        = Tool.Pnt(vert);
            Xyz acenter       = pvert.XYZ();
            Xyz acenterreject = pvert.XYZ();

            var suitEdges   = new NCollection_Sequence<Shape>();
            var rejectEdges = new NCollection_Sequence<Shape>();
            var aledges      = new NCollection_List<Shape>(mapVertEdges.FindFromIndex(i));
            if (aledges.Extent() == 1)
            {
                continue;
            }
            // if tolerance of vertex is more than specified tolerance
            //  check distance between curves and vertex

            if (!GetNearestEdges(aledges, vert, suitEdges, rejectEdges, tolerance, ref acenter, ref acenterreject))
            {
                continue;
            }

            // update vertex by nearest point
            bool isAdd = false;
            int  k     = 1;
            for (; k <= suitEdges.Length(); k++)
            {
                Edge edgeOld = ShapeCasts.Edge(suitEdges.Value(k));
                Exploration.Vertices(edgeOld, out Vertex vert1, out Vertex vert2);

                bool isFirst = (vert1.IsSame(vert));
                bool isLast  = (vert2.IsSame(vert));
                if (!isFirst && !isLast)
                {
                    continue;
                }
                Edge edge = ShapeCasts.Edge(thecontext.InternalApply(edgeOld));

                Exploration.Vertices(edge, out Vertex vert1n, out Vertex vert2n);
                Curve3D? curve = Tool.Curve(edge, out double first, out double last);
                if (curve is not null)
                {
                    Point3D p1 = curve.Value(first);
                    Point3D p2 = curve.Value(last);

                    // if distance between ends of curve more than specified tolerance
                    // but vertices are the same that one of the vertex will be replaced.

                    bool isReplace = (vert1n.IsSame(vert2n) && p1.Distance(p2) > tolerance);

                    if (isFirst)
                    {
                        if (k > 2)
                        {
                            acenter += p1.XYZ();
                            acenter /= 2.0;
                        }
                        if (isReplace)
                        {
                            Edge enew;
                            if (p1.Distance(new Point3D(acenter)) < p2.Distance(new Point3D(acenter)))
                            {
                                enew = ReplaceVertex(edge, p2, false);
                            }
                            else
                            {
                                enew = ReplaceVertex(edge, p1, true);
                            }
                            thecontext.Replace(edge, enew);
                            isDone = true;
                        }
                    }
                    else
                    {
                        if (k > 2)
                        {
                            acenter += p2.XYZ();
                            acenter /= 2.0;
                        }
                        if (isReplace)
                        {
                            Edge enew;
                            if (p1.Distance(new Point3D(acenter)) < p2.Distance(new Point3D(acenter)))
                            {
                                enew = ReplaceVertex(edge, p2, false);
                            }
                            else
                            {
                                enew = ReplaceVertex(edge, p1, true);
                            }
                            thecontext.Replace(edge, enew);
                            isDone = true;
                        }
                    }

                    isAdd = true;
                }
            }

            if (isAdd && pvert.Distance(new Point3D(acenter)) > tolerance)
            {
                var b = new BrepBuilder();

                //  aB.UpdateVertex(aVert,Precision::Confusion());
                // else {
                isDone = true;
                var newVertex = new Vertex();
                b.MakeVertex(ref newVertex, new Point3D(acenter), Kerf.Engine.Foundation.Precision.Confusion());
                newVertex.Orientation(vert.Orientation());
                thecontext.Replace(vert, newVertex);
            }

            for (k = 1; k <= rejectEdges.Length(); k++)
            {
                Edge edgeOld = ShapeCasts.Edge(rejectEdges.Value(k));
                Exploration.Vertices(edgeOld, out Vertex vert1, out Vertex vert2);

                bool isFirst = (vert1.IsSame(vert));
                bool isLast  = (vert2.IsSame(vert));
                if (!isFirst && !isLast)
                {
                    continue;
                }
                bool        isSame = vert1.IsSame(vert2);
                Edge edge  = ShapeCasts.Edge(thecontext.InternalApply(edgeOld));

                Exploration.Vertices(edge, out Vertex vert1n, out Vertex vert2n);

                Curve3D? curve = Tool.Curve(edge, out double first, out double last);
                if (curve is not null)
                {
                    Point3D      p1 = curve.Value(first);
                    Point3D      p2 = curve.Value(last);
                    Edge enew;
                    if (isFirst)
                    {
                        enew = ReplaceVertex(edge, p1, true);
                        if (isSame)
                        {
                            enew = ReplaceVertex(enew, p2, false);
                        }
                    }
                    else
                    {
                        enew = ReplaceVertex(edge, p2, false);
                        if (isSame)
                        {
                            enew = ReplaceVertex(enew, p1, true);
                        }
                    }

                    thecontext.Replace(edge, enew);
                    isDone = true;
                }
            }
        }
        if (isDone)
        {
            theshape = thecontext.InternalApply(theshape);
        }
        return isDone;
    }

    /// <summary>
    /// The smallest size of the edges of the shape: the diagonal of the box of each 3D curve's ends and middle.
    /// </summary>
    public static double LeastEdgeSize(in Shape shape)
    {
        double res = RealLast();
        for (var exp = new Explorer(shape, ShapeType.Edge); exp.More(); exp.Next())
        {
            Edge edge = ShapeCasts.Edge(exp.Current());
            Curve3D? c3d  = Tool.Curve(edge, out double first, out double last);
            if (c3d is not null)
            {
                var bb = new Box();
                bb.Add(c3d.Value(first));
                bb.Add(c3d.Value(last));
                bb.Add(c3d.Value((last + first) / 2.0));
                bb.Get(out double x1, out double y1, out double z1, out double x2, out double y2, out double z2);
                double size = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) + (z2 - z1) * (z2 - z1);
                if (size < res)
                {
                    res = size;
                }
            }
        }
        res = Math.Sqrt(res);
        return res;
    }
}
