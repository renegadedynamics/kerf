// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2000-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using static Kerf.Engine.Foundation.Gravity;
using static Occt.ShapeExtend_Status;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

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
public static partial class ShapeFix
{
    /// <summary>
    /// Runs ShapeFix_Edge's FixSameParameter on every edge of the shape (on each of its faces), then updates the
    /// tolerances of the edges on planes. Returns false if an edge couldn't be made SameParameter. With
    /// <paramref name="enforce"/>, the SameRange and SameParameter flags are reset first. Writes into the shape.
    /// </summary>
    internal static bool SameParameter(in TopoDS_Shape shape, bool enforce, double preci = 0.0,
                                       ProgressRange theProgress = default,
                                       ShapeExtend_BasicMsgRegistrator? theMsgReg = null)
    {
        // Calculate number of edges
        int aNbEdges = 0;
        for (var anEdgeExp = new TopExp_Explorer(shape, TopAbs_EDGE); anEdgeExp.More(); anEdgeExp.Next())
        {
            ++aNbEdges;
        }

        // Calculate number of faces
        int aNbFaces = 0;
        for (var anEdgeExp = new TopExp_Explorer(shape, TopAbs_FACE); anEdgeExp.More(); anEdgeExp.Next())
        {
            ++aNbFaces;
        }

        var aMapEF = new NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, aMapEF);

        var    B       = new BRep_Builder();
        bool   status  = true;
        double tol     = preci;
        bool   iatol   = (tol > 0);
        var    sfe     = new ShapeFix_Edge();
        var    ex      = new TopExp_Explorer(shape, TopAbs_EDGE);
        var    doneMsg = new Msg("FixEdge.SameParameter.MSG0");

        // Start progress scope (no need to check if progress exists -- it is safe)
        using var aPSForSameParam = new ProgressScope(theProgress, "Fixing same parameter problem", 2);

        {
            // Start progress scope (no need to check if progress exists -- it is safe)
            using var aPS = new ProgressScope(aPSForSameParam.Next(), "Fixing edge", aNbEdges);

            while (ex.More())
            {
                TopoDS_Edge E;
                while (ex.More() && aPS.More())
                {
                    int ierr = 0;
                    E = TopoDS.Edge(ex.Current());
                    ex.Next();

                    if (!iatol)
                    {
                        tol = BRep_Tool.Tolerance(E);
                    }
                    if (enforce)
                    {
                        B.SameRange(E, false);
                        B.SameParameter(E, false);
                    }

                    aMapEF.FindFromKey(E, out NCollection_List<TopoDS_Shape> aListOfFaces);
                    if (aListOfFaces.Extent() != 0)
                    {
                        var aListOfFacesIt = new NCollection_List<TopoDS_Shape>.Iterator(aListOfFaces);
                        for (; aListOfFacesIt.More(); aListOfFacesIt.Next())
                        {
                            TopoDS_Face aF = TopoDS.Face(aListOfFacesIt.Value());
                            sfe.FixSameParameter(E, aF);
                        }
                    }
                    else
                    {
                        sfe.FixSameParameter(E); // K2-SEP97
                    }

                    if (!BRep_Tool.SameParameter(E))
                    {
                        ierr = 1;
                    }

                    if (ierr != 0)
                    {
                        status = false;
                        B.SameRange(E, false);
                        B.SameParameter(E, false);
                    }
                    else if (theMsgReg is not null && !sfe.Status(ShapeExtend_OK))
                    {
                        theMsgReg.Send(E, doneMsg, Warning);
                    }

                    // Complete step in current progress scope
                    aPS.Next();
                } // -- end while

                // Halt algorithm in case of user's abort
                if (!aPS.More())
                {
                    return false;
                }
            }
        }

        {
            // Start progress scope (no need to check if progress exists -- it is safe)
            using var aPS = new ProgressScope(aPSForSameParam.Next(), "Update tolerances", aNbFaces);

            //: i2 abv 21 Aug 98: ProSTEP TR8 Motor.rle face 710:
            // Update tolerance of edges on planes (no pcurves are stored)
            for (var exp = new TopExp_Explorer(shape, TopAbs_FACE); exp.More() && aPS.More(); exp.Next(), aPS.Next())
            {
                TopoDS_Face   face = TopoDS.Face(exp.Current());
                Geom_Surface? Surf = BRep_Tool.Surface(face);

                Geom_Plane? plane = Surf as Geom_Plane;
                if (plane is null)
                {
                    if (Surf is Geom_RectangularTrimmedSurface GRTS)
                    {
                        plane = GRTS.BasisSurface() as Geom_Plane;
                    }
                    if (plane is null)
                    {
                        continue;
                    }
                }

                var AS = new GeomAdaptor_Surface(plane);
                for (var ed = new TopExp_Explorer(face, TopAbs_EDGE); ed.More(); ed.Next())
                {
                    TopoDS_Edge edge = TopoDS.Edge(ed.Current());
                    Geom_Curve? crv  = BRep_Tool.Curve(edge, out double f, out double l);
                    if (crv is null)
                    {
                        continue;
                    }

                    Geom2d_Curve? c2d = BRep_Tool.CurveOnSurface(edge, face, out f, out l);
                    if (c2d is null)
                    {
                        continue;
                    }
                    var GHPC = new Geom2dAdaptor_Curve(c2d, f, l);
                    var ACS  = new Adaptor3d_CurveOnSurface(GHPC, AS);

                    double    tol0      = BRep_Tool.Tolerance(edge);
                    tol = tol0;
                    double    tol2      = tol * tol;
                    bool      isChanged = false;
                    const int NCONTROL  = 23;
                    for (int i = 0; i < NCONTROL; i++)
                    {
                        double par  = (f * (NCONTROL - 1 - i) + l * i) / (NCONTROL - 1);
                        Point3D pnt  = crv.Value(par);
                        Point3D prj  = ACS.Value(par);
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
                            B.UpdateEdge(edge, tol);
                            for (var itV = new TopoDS_Iterator(edge); itV.More(); itV.Next())
                            {
                                TopoDS_Shape S = itV.Value();
                                B.UpdateVertex(TopoDS.Vertex(S), tol);
                            }
                        }
                    }
                }
            }
            // Halt algorithm in case of user's abort
            if (!aPS.More())
            {
                return false;
            }
        }

        return status;
    }

    /// <summary>Runs BRepLib's EncodeRegularity on the shape. Writes into the shape.</summary>
    internal static void EncodeRegularity(in TopoDS_Shape shape, double tolang = 1.0e-10)
    {
        BRepLib.EncodeRegularity(shape, tolang);
    }

    /// <summary>
    /// Removes the edges of the shape shorter than <paramref name="Tolerance"/>, with ShapeFix_Wire's FixSmall run by
    /// ShapeFix_Shape (the other wire and face fixes off but reordering). Returns the result, frozen, and the context
    /// of the fix in <paramref name="context"/>, which answers for the caller's sub-shapes. The shape itself isn't
    /// changed (ShapeFix_Shape copies a frozen shape first).
    /// </summary>
    public static TopoDS_Shape RemoveSmallEdges(in TopoDS_Shape Shape, double Tolerance, out ShapeBuild_ReShape context)
    {
        ShapeFix_Shape sfs = removeSmallEdges(Shape, Tolerance);
        context = sfs.Context()!;
        return sfs.Shape();
    }

    /// <summary>
    /// <see cref="RemoveSmallEdges"/> without freezing the result, for ported code fixing a shape it owns (fixed in
    /// place, as in OCCT).
    /// </summary>
    internal static TopoDS_Shape InternalRemoveSmallEdges(in TopoDS_Shape Shape, double Tolerance, out ShapeBuild_ReShape context)
    {
        ShapeFix_Shape sfs = removeSmallEdges(Shape, Tolerance);
        context = sfs.Context()!;
        return sfs.InternalShape();
    }

    private static ShapeFix_Shape removeSmallEdges(in TopoDS_Shape Shape, double Tolerance)
    {
        var sfs = new ShapeFix_Shape();
        sfs.Init(Shape);
        sfs.SetPrecision(Tolerance);
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
    private static TopoDS_Edge ReplaceVertex(in TopoDS_Edge theEdge, in Point3D theP, bool theFwd)
    {
        var aNewVertex = new TopoDS_Vertex();
        var aB         = new BRep_Builder();
        aB.MakeVertex(ref aNewVertex, theP, Kerf.Engine.Foundation.Precision.Confusion());
        var aV1 = new TopoDS_Vertex();
        var aV2 = new TopoDS_Vertex();
        if (theFwd)
        {
            aV1 = aNewVertex;
            aV1.Orientation(TopAbs_FORWARD);
        }
        else
        {
            aV2 = aNewVertex;
            aV2.Orientation(TopAbs_REVERSED);
        }
        var                aSbe = new ShapeBuild_Edge();
        TopoDS_Edge        e1   = theEdge;
        TopAbs_Orientation Ori  = e1.Orientation();
        e1.Orientation(TopAbs_FORWARD);
        TopoDS_Edge aNewEdge = aSbe.InternalCopyReplaceVertices(e1, aV1, aV2);
        aNewEdge.Orientation(Ori);
        return aNewEdge;
    }

    //=======================================================================
    // function : getNearPoint
    // purpose  : auxiliary for FixVertexPosition
    //=======================================================================
    private static double getNearPoint(NCollection_Sequence<Point3D> aSeq1, NCollection_Sequence<Point3D> aSeq2,
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
    private static bool getNearestEdges(NCollection_List<TopoDS_Shape> theLEdges,
                                        in TopoDS_Vertex theVert,
                                        NCollection_Sequence<TopoDS_Shape> theSuitEdges,
                                        NCollection_Sequence<TopoDS_Shape> theRejectEdges,
                                        double theTolerance,
                                        ref Xyz thecentersuit,
                                        ref Xyz thecenterreject)
    {
        if (theLEdges.IsEmpty())
        {
            return false;
        }
        var aMapEdges = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);

        var atempList = new NCollection_List<TopoDS_Shape>(theLEdges);
        var alIter    = new NCollection_List<TopoDS_Shape>.Iterator(atempList);

        TopoDS_Edge aEdge1 = TopoDS.Edge(alIter.Value());
        TopExp.Vertices(aEdge1, out TopoDS_Vertex aVert11, out TopoDS_Vertex aVert12);
        aMapEdges.Add(aEdge1);
        Geom_Curve? aCurve1  = BRep_Tool.Curve(aEdge1, out double aFirst1, out double aLast1);
        var         p11      = new Point3D();
        var         p12      = new Point3D();
        bool        isFirst1 = theVert.IsSame(aVert11);
        bool        isSame1  = aVert11.IsSame(aVert12);
        if (aCurve1 is not null)
        {
            if (isFirst1)
            {
                p11 = aCurve1.Value(aFirst1);
            }
            else if (!isSame1)
            {
                p11 = aCurve1.Value(aLast1);
            }
            if (isSame1)
            {
                p12 = aCurve1.Value(aLast1);
            }
        }
        else
        {
            return false;
        }
        alIter.Next();
        var aseqreject = new NCollection_Sequence<TopoDS_Shape>();
        var aseqsuit   = new NCollection_Sequence<TopoDS_Shape>();

        int anumLoop = 0;
        for (; alIter.More();)
        {
            TopoDS_Edge aEdge = TopoDS.Edge(alIter.Value());
            if (aMapEdges.Contains(aEdge))
            {
                atempList.Remove(alIter);
                continue;
            }

            TopExp.Vertices(aEdge, out TopoDS_Vertex aVert1, out TopoDS_Vertex aVert2);
            bool isFirst = theVert.IsSame(aVert1); // OCCT declares it double
            bool isSame  = aVert1.IsSame(aVert2);

            bool isLoop = ((aVert1.IsSame(aVert11) && aVert2.IsSame(aVert12))
                           || (aVert1.IsSame(aVert12) && aVert2.IsSame(aVert11)));
            if (isLoop /*&& !aseqsuit.Length()*/ && (atempList.Extent() > anumLoop))
            {
                atempList.Append(aEdge);
                atempList.Remove(alIter);
                anumLoop++;
                continue;
            }
            aMapEdges.Add(aEdge);
            Geom_Curve? aCurve = BRep_Tool.Curve(aEdge, out double aFirst, out double aLast);
            if (aCurve is not null)
            {
                Point3D p1;
                var    p2 = new Point3D();
                if (isFirst)
                {
                    p1 = aCurve.Value(aFirst);
                }
                else
                {
                    p1 = aCurve.Value(aLast);
                }
                if (isSame)
                {
                    p2 = aCurve.Value(aLast);
                }
                double aMinDist = RealLast();
                var    acent    = new Xyz();
                if (!isSame && !isSame1)
                {
                    aMinDist = p1.Distance(p11);
                    acent    = (p1.XYZ() + p11.XYZ()) / 2.0;
                }
                else
                {
                    var aSeq1 = new NCollection_Sequence<Point3D>();
                    var aSeq2 = new NCollection_Sequence<Point3D>();
                    aSeq1.Append(p11);
                    if (isSame1)
                    {
                        aSeq1.Append(p12);
                    }
                    aSeq2.Append(p1);
                    if (isSame)
                    {
                        aSeq2.Append(p2);
                    }
                    aMinDist = getNearPoint(aSeq1, aSeq2, ref acent);
                }

                if (aMinDist > theTolerance)
                {
                    if (aseqreject.Length() == 0)
                    {
                        thecenterreject = acent;
                    }
                    aseqreject.Append(aEdge);
                }
                else
                {
                    if (aseqsuit.Length() == 0)
                    {
                        thecentersuit = acent;
                        aseqsuit.Append(aEdge);
                    }
                    else if (!isSame1)
                    {
                        aseqsuit.Append(aEdge);
                    }
                    else if ((thecentersuit - acent).Modulus() < theTolerance)
                    {
                        aseqsuit.Append(aEdge);
                    }
                    else
                    {
                        aseqreject.Append(aEdge);
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
                theRejectEdges.Append(aEdge1);
                theLEdges.RemoveFirst();

                getNearestEdges(theLEdges,
                                theVert,
                                theSuitEdges,
                                theRejectEdges,
                                theTolerance,
                                ref thecentersuit,
                                ref thecenterreject);
            }
            else
            {
                theSuitEdges.Append(aEdge1);
                theSuitEdges.Append(aseqsuit);
                theRejectEdges.Append(aseqreject);
            }
        }
        else
        {
            theRejectEdges.Append(aEdge1);
        }

        return isDone;
    }

    /// <summary>
    /// Moves the vertices more tolerant than <paramref name="theTolerance"/> to the ends of the curves near them, by
    /// replacing them (and edges whose ends are too far apart) in the context; the shape becomes the context applied
    /// to it if anything was done. Writes the vertices' Modified flag.
    /// </summary>
    internal static bool FixVertexPosition(ref TopoDS_Shape theshape, double theTolerance, ShapeBuild_ReShape thecontext)
    {
        var aMapVertEdges = new NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
        var aExp1         = new TopExp_Explorer(theshape, TopAbs_EDGE);
        for (; aExp1.More(); aExp1.Next())
        {
            var aVert1 = new TopoDS_Shape();
            int nV     = 1;
            var aExp3  = new TopoDS_Iterator(aExp1.Current());
            for (; aExp3.More(); aExp3.Next(), nV++)
            {
                TopoDS_Shape aVert = aExp3.Value();
                if (nV == 1)
                {
                    aVert1 = aVert;
                }
                else if (aVert1.IsSame(aVert))
                {
                    continue;
                }
                if (aMapVertEdges.Contains(aVert))
                {
                    aMapVertEdges.ChangeFromKey(aVert).Append(aExp1.Current());
                }
                else
                {
                    var alEdges = new NCollection_List<TopoDS_Shape>();
                    alEdges.Append(aExp1.Current());
                    aMapVertEdges.Add(aVert, alEdges);
                }
            }
        }
        bool isDone = false;
        int  i      = 1;
        for (; i <= aMapVertEdges.Extent(); i++)
        {
            TopoDS_Vertex aVert    = TopoDS.Vertex(aMapVertEdges.FindKey(i));
            double        aTolVert = BRep_Tool.Tolerance(aVert);
            if (aTolVert <= theTolerance)
            {
                continue;
            }

            var aB1 = new BRep_Builder();
            aB1.UpdateVertex(aVert, theTolerance);
            Point3D aPvert        = BRep_Tool.Pnt(aVert);
            Xyz acenter       = aPvert.XYZ();
            Xyz acenterreject = aPvert.XYZ();

            var aSuitEdges   = new NCollection_Sequence<TopoDS_Shape>();
            var aRejectEdges = new NCollection_Sequence<TopoDS_Shape>();
            var aledges      = new NCollection_List<TopoDS_Shape>(aMapVertEdges.FindFromIndex(i));
            if (aledges.Extent() == 1)
            {
                continue;
            }
            // if tolerance of vertex is more than specified tolerance
            //  check distance between curves and vertex

            if (!getNearestEdges(aledges, aVert, aSuitEdges, aRejectEdges, theTolerance, ref acenter, ref acenterreject))
            {
                continue;
            }

            // update vertex by nearest point
            bool isAdd = false;
            int  k     = 1;
            for (; k <= aSuitEdges.Length(); k++)
            {
                TopoDS_Edge aEdgeOld = TopoDS.Edge(aSuitEdges.Value(k));
                TopExp.Vertices(aEdgeOld, out TopoDS_Vertex aVert1, out TopoDS_Vertex aVert2);

                bool isFirst = (aVert1.IsSame(aVert));
                bool isLast  = (aVert2.IsSame(aVert));
                if (!isFirst && !isLast)
                {
                    continue;
                }
                TopoDS_Edge aEdge = TopoDS.Edge(thecontext.InternalApply(aEdgeOld));

                TopExp.Vertices(aEdge, out TopoDS_Vertex aVert1n, out TopoDS_Vertex aVert2n);
                Geom_Curve? aCurve = BRep_Tool.Curve(aEdge, out double aFirst, out double aLast);
                if (aCurve is not null)
                {
                    Point3D p1 = aCurve.Value(aFirst);
                    Point3D p2 = aCurve.Value(aLast);

                    // if distance between ends of curve more than specified tolerance
                    // but vertices are the same that one of the vertex will be replaced.

                    bool isReplace = (aVert1n.IsSame(aVert2n) && p1.Distance(p2) > theTolerance);

                    if (isFirst)
                    {
                        if (k > 2)
                        {
                            acenter += p1.XYZ();
                            acenter /= 2.0;
                        }
                        if (isReplace)
                        {
                            TopoDS_Edge enew;
                            if (p1.Distance(new Point3D(acenter)) < p2.Distance(new Point3D(acenter)))
                            {
                                enew = ReplaceVertex(aEdge, p2, false);
                            }
                            else
                            {
                                enew = ReplaceVertex(aEdge, p1, true);
                            }
                            thecontext.Replace(aEdge, enew);
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
                            TopoDS_Edge enew;
                            if (p1.Distance(new Point3D(acenter)) < p2.Distance(new Point3D(acenter)))
                            {
                                enew = ReplaceVertex(aEdge, p2, false);
                            }
                            else
                            {
                                enew = ReplaceVertex(aEdge, p1, true);
                            }
                            thecontext.Replace(aEdge, enew);
                            isDone = true;
                        }
                    }

                    isAdd = true;
                }
            }

            if (isAdd && aPvert.Distance(new Point3D(acenter)) > theTolerance)
            {
                var aB = new BRep_Builder();

                //  aB.UpdateVertex(aVert,Precision::Confusion());
                // else {
                isDone = true;
                var aNewVertex = new TopoDS_Vertex();
                aB.MakeVertex(ref aNewVertex, new Point3D(acenter), Kerf.Engine.Foundation.Precision.Confusion());
                aNewVertex.Orientation(aVert.Orientation());
                thecontext.Replace(aVert, aNewVertex);
            }

            for (k = 1; k <= aRejectEdges.Length(); k++)
            {
                TopoDS_Edge aEdgeOld = TopoDS.Edge(aRejectEdges.Value(k));
                TopExp.Vertices(aEdgeOld, out TopoDS_Vertex aVert1, out TopoDS_Vertex aVert2);

                bool isFirst = (aVert1.IsSame(aVert));
                bool isLast  = (aVert2.IsSame(aVert));
                if (!isFirst && !isLast)
                {
                    continue;
                }
                bool        isSame = aVert1.IsSame(aVert2);
                TopoDS_Edge aEdge  = TopoDS.Edge(thecontext.InternalApply(aEdgeOld));

                TopExp.Vertices(aEdge, out TopoDS_Vertex aVert1n, out TopoDS_Vertex aVert2n);

                Geom_Curve? aCurve = BRep_Tool.Curve(aEdge, out double aFirst, out double aLast);
                if (aCurve is not null)
                {
                    Point3D      p1 = aCurve.Value(aFirst);
                    Point3D      p2 = aCurve.Value(aLast);
                    TopoDS_Edge enew;
                    if (isFirst)
                    {
                        enew = ReplaceVertex(aEdge, p1, true);
                        if (isSame)
                        {
                            enew = ReplaceVertex(enew, p2, false);
                        }
                    }
                    else
                    {
                        enew = ReplaceVertex(aEdge, p2, false);
                        if (isSame)
                        {
                            enew = ReplaceVertex(enew, p1, true);
                        }
                    }

                    thecontext.Replace(aEdge, enew);
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
    public static double LeastEdgeSize(in TopoDS_Shape theShape)
    {
        double aRes = RealLast();
        for (var exp = new TopExp_Explorer(theShape, TopAbs_EDGE); exp.More(); exp.Next())
        {
            TopoDS_Edge edge = TopoDS.Edge(exp.Current());
            Geom_Curve? c3d  = BRep_Tool.Curve(edge, out double first, out double last);
            if (c3d is not null)
            {
                var bb = new Box();
                bb.Add(c3d.Value(first));
                bb.Add(c3d.Value(last));
                bb.Add(c3d.Value((last + first) / 2.0));
                bb.Get(out double x1, out double y1, out double z1, out double x2, out double y2, out double z2);
                double size = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) + (z2 - z1) * (z2 - z1);
                if (size < aRes)
                {
                    aRes = size;
                }
            }
        }
        aRes = Math.Sqrt(aRes);
        return aRes;
    }
}
