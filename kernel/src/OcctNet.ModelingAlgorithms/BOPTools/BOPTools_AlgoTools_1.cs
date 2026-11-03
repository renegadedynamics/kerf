// Port of: src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_AlgoTools_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

// The tolerance corrections of BOPTools_AlgoTools (BOPTools_AlgoTools_1.cxx).
//
// They are in-place utilities (design family 1): they grow the tolerances of the edges and vertices of the shape
// they are given (BRep_Builder.UpdateEdge/UpdateVertex in UpdateShape, the only write site), except those in the map
// to avoid. A frozen sub-shape throws TopoDS_LockedShape. Ported callers (BOPAlgo's post-passes) give them results
// they are still building (plan 2.3).
//
// Decision 6430: OCCT runs the five loops in parallel when asked. Their bodies write the tolerances of sub-shapes
// shared between the loop's items (a vertex of several edges or faces, an edge of two faces) with an unsynchronised
// read-compare-write, and the new values depend on the tolerances read first (CheckEdge's margin 0.1*Tol and its
// threshold, CorrectWires' and CorrectEdgeTolerance's comparisons). So native's parallel result depends on the
// threads' timing and can differ from its serial one (occt-issues.md). The port runs them serially whatever the flag,
// which is native's serial result (decision 580).
public static partial class BOPTools_AlgoTools
{
    // The loops write shared sub-shapes: always serial (decision 6430).
    private const bool theToRunCorrectionsInParallel = false;

    /// <summary>
    /// Provides valid tolerances for the shape: <see cref="CorrectPointOnCurve"/>, then
    /// <see cref="CorrectCurveOnSurface"/>. A tolerance that would reach <paramref name="aMaxTol"/> is not set; the
    /// sub-shapes in <paramref name="aMapToAvoid"/> are not changed. Writes into the shape's edges and vertices.
    /// </summary>
    /// <remarks><paramref name="bRunParallel"/> is ignored: the corrections run serially (decision 6430).</remarks>
    public static void CorrectTolerances(in TopoDS_Shape aShape, NCollection_IndexedMap<TopoDS_Shape> aMapToAvoid, double aMaxTol = 0.0001,
                                         bool bRunParallel = false)
    {
        CorrectPointOnCurve(aShape, aMapToAvoid, aMaxTol, bRunParallel);
        CorrectCurveOnSurface(aShape, aMapToAvoid, aMaxTol, bRunParallel);
    }

    /// <summary>
    /// Provides valid tolerances for the shape in terms of BRepCheck_InvalidPointOnCurve: each vertex's tolerance
    /// grows to cover its distance to the points of its edges' 3D curves. Writes into the shape's vertices.
    /// </summary>
    /// <remarks><paramref name="bRunParallel"/> is ignored: the corrections run serially (decision 6430).</remarks>
    public static void CorrectPointOnCurve(in TopoDS_Shape aS, NCollection_IndexedMap<TopoDS_Shape> aMapToAvoid, double aMaxTol = 0.0001,
                                           bool bRunParallel = false)
    {
        TopExp_Explorer aExp = new();
        List<BOPTools_CPC> aVCPC = [];
        //
        aExp.Init(aS, TopAbs_EDGE);
        for (; aExp.More(); aExp.Next())
        {
            TopoDS_Edge aE = TopoDS.Edge(aExp.Current());
            BOPTools_CPC aCPC = new();
            aVCPC.Add(aCPC);
            aCPC.SetEdge(aE);
            aCPC.SetMaxTol(aMaxTol);
            aCPC.SetMapToAvoid(aMapToAvoid);
        }
        //
        //======================================================
        _ = bRunParallel;
        BOPTools_Parallel.Perform(theToRunCorrectionsInParallel, aVCPC);
        //======================================================
    }

    /// <summary>
    /// Provides valid tolerances for the shape in terms of BRepCheck_InvalidCurveOnSurface: vertices grow to cover
    /// their faces' wires (pcurve ends and 2D self-intersections near them), then edges grow to cover the deviation
    /// of their pcurves from their 3D curves. Writes into the shape's edges and vertices.
    /// </summary>
    /// <remarks><paramref name="bRunParallel"/> is ignored: the corrections run serially (decision 6430).</remarks>
    public static void CorrectCurveOnSurface(in TopoDS_Shape aS, NCollection_IndexedMap<TopoDS_Shape> aMapToAvoid, double aMaxTol = 0.0001,
                                             bool bRunParallel = false)
    {
        TopExp_Explorer aExpF = new(), aExpE = new();
        List<BOPTools_CWT> aVCWT = [];
        List<BOPTools_CDT> aVCDT = [];
        //
        aExpF.Init(aS, TopAbs_FACE);
        for (; aExpF.More(); aExpF.Next())
        {
            TopoDS_Face aF = TopoDS.Face(aExpF.Current());
            //
            BOPTools_CWT aCWT = new();
            aVCWT.Add(aCWT);
            aCWT.SetFace(aF);
            aCWT.SetMapToAvoid(aMapToAvoid);
            //
            aExpE.Init(aF, TopAbs_EDGE);
            for (; aExpE.More(); aExpE.Next())
            {
                TopoDS_Edge aE = TopoDS.Edge(aExpE.Current());
                //
                BOPTools_CDT aCDT = new();
                aVCDT.Add(aCDT);
                aCDT.SetEdge(aE);
                aCDT.SetFace(aF);
                aCDT.SetMaxTol(aMaxTol);
                aCDT.SetMapToAvoid(aMapToAvoid);
            }
        }
        //
        //======================================================
        _ = bRunParallel;
        BOPTools_Parallel.Perform(theToRunCorrectionsInParallel, aVCWT);
        //======================================================
        BOPTools_Parallel.Perform(theToRunCorrectionsInParallel, aVCDT);
        //======================================================
    }

    /// <summary>
    /// Corrects the tolerances of the shape's sub-shapes where needed: each vertex at least its edges' tolerances,
    /// each edge and vertex of a face at least the face's. Writes into the shape's edges and vertices.
    /// </summary>
    /// <remarks><paramref name="bRunParallel"/> is ignored: the corrections run serially (decision 6430).</remarks>
    public static void CorrectShapeTolerances(in TopoDS_Shape aShape, NCollection_IndexedMap<TopoDS_Shape> aMapToAvoid,
                                              bool bRunParallel = false)
    {
        TopExp_Explorer aExp = new();
        List<BOPTools_CVT> aVCVT = [];
        List<BOPTools_CET> aVCET = [];
        //
        aExp.Init(aShape, TopAbs_EDGE);
        for (; aExp.More(); aExp.Next())
        {
            TopoDS_Edge aE = TopoDS.Edge(aExp.Current());
            BOPTools_CVT aCVT = new();
            aVCVT.Add(aCVT);
            aCVT.SetEdge(aE);
            aCVT.SetMapToAvoid(aMapToAvoid);
        }
        //
        //======================================================
        _ = bRunParallel;
        BOPTools_Parallel.Perform(theToRunCorrectionsInParallel, aVCVT);
        //======================================================
        //
        aExp.Init(aShape, TopAbs_FACE);
        for (; aExp.More(); aExp.Next())
        {
            TopoDS_Face aF = TopoDS.Face(aExp.Current());
            BOPTools_CET aCET = new();
            aVCET.Add(aCET);
            aCET.SetFace(aF);
            aCET.SetMapToAvoid(aMapToAvoid);
        }
        //
        //======================================================
        BOPTools_Parallel.Perform(theToRunCorrectionsInParallel, aVCET);
        //======================================================
    }

    /// <summary>
    /// Computes the tolerance the edge needs on the face (<see cref="BRepLib_CheckCurveOnSurface"/>): the maximal
    /// distance between its 3D curve and its pcurve, and the parameter where it is reached. False if the check
    /// fails (the outputs are then 0, where OCCT leaves them unchanged).
    /// </summary>
    public static bool ComputeTolerance(in TopoDS_Face theFace, in TopoDS_Edge theEdge, out double theMaxDist, out double theMaxPar)
    {
        BRepLib_CheckCurveOnSurface aCS = new();
        //
        aCS.Init(theEdge, theFace);
        aCS.Perform();
        if (!aCS.IsDone())
        {
            theMaxDist = 0.0;
            theMaxPar = 0.0;
            return false;
        }
        //
        theMaxDist = aCS.MaxDistance();
        theMaxPar = aCS.MaxParameter();
        //
        return true;
    }

    // The loop bodies (OCCT's file classes, with the map to avoid held by reference).

    private sealed class BOPTools_CPC : BOPTools_Parallel.ISolver
    {
        private double myMaxTol = 1.0e-7;
        private TopoDS_Edge myEdge;
        private NCollection_IndexedMap<TopoDS_Shape>? mypMapToAvoid;

        internal void SetEdge(in TopoDS_Edge aE) => myEdge = aE;

        internal TopoDS_Edge Edge() => myEdge;

        internal void SetMaxTol(double aMaxTol) => myMaxTol = aMaxTol;

        internal double MaxTol() => myMaxTol;

        internal void SetMapToAvoid(NCollection_IndexedMap<TopoDS_Shape> aMapToAvoid) => mypMapToAvoid = aMapToAvoid;

        public void Perform()
        {
            Standard_ProgramError_Raise_if(mypMapToAvoid is null, "mypMapToAvoid is null");
            CheckEdge(myEdge, myMaxTol, mypMapToAvoid!);
        }
    }

    private sealed class BOPTools_CWT : BOPTools_Parallel.ISolver
    {
        private TopoDS_Face myFace;
        private NCollection_IndexedMap<TopoDS_Shape>? mypMapToAvoid;

        internal void SetFace(in TopoDS_Face aF) => myFace = aF;

        internal void SetMapToAvoid(NCollection_IndexedMap<TopoDS_Shape> aMapToAvoid) => mypMapToAvoid = aMapToAvoid;

        public void Perform()
        {
            Standard_ProgramError_Raise_if(mypMapToAvoid is null, "mypMapToAvoid is null");
            CorrectWires(myFace, mypMapToAvoid!);
        }
    }

    private sealed class BOPTools_CDT : BOPTools_Parallel.ISolver
    {
        private double myMaxTol = 1.0e-7;
        private TopoDS_Edge myEdge;
        private TopoDS_Face myFace;
        private NCollection_IndexedMap<TopoDS_Shape>? mypMapToAvoid;

        internal void SetEdge(in TopoDS_Edge aE) => myEdge = aE;

        internal void SetFace(in TopoDS_Face aF) => myFace = aF;

        internal void SetMaxTol(double aMaxTol) => myMaxTol = aMaxTol;

        internal void SetMapToAvoid(NCollection_IndexedMap<TopoDS_Shape> aMapToAvoid) => mypMapToAvoid = aMapToAvoid;

        public void Perform()
        {
            Standard_ProgramError_Raise_if(mypMapToAvoid is null, "mypMapToAvoid is null");
            CorrectEdgeTolerance(myEdge, myFace, myMaxTol, mypMapToAvoid!);
        }
    }

    private sealed class BOPTools_CVT : BOPTools_Parallel.ISolver
    {
        private TopoDS_Edge myEdge;
        private NCollection_IndexedMap<TopoDS_Shape>? mypMapToAvoid;

        internal void SetEdge(in TopoDS_Edge aE) => myEdge = aE;

        internal void SetMapToAvoid(NCollection_IndexedMap<TopoDS_Shape> aMapToAvoid) => mypMapToAvoid = aMapToAvoid;

        public void Perform()
        {
            Standard_ProgramError_Raise_if(mypMapToAvoid is null, "mypMapToAvoid is null");
            CorrectVertexTolerance(myEdge, mypMapToAvoid!);
        }
    }

    private sealed class BOPTools_CET : BOPTools_Parallel.ISolver
    {
        private TopoDS_Face myFace;
        private NCollection_IndexedMap<TopoDS_Shape>? mypMapToAvoid;

        internal void SetFace(in TopoDS_Face aF) => myFace = aF;

        internal void SetMapToAvoid(NCollection_IndexedMap<TopoDS_Shape> aMapToAvoid) => mypMapToAvoid = aMapToAvoid;

        public void Perform()
        {
            Standard_ProgramError_Raise_if(mypMapToAvoid is null, "mypMapToAvoid is null");
            UpdateEdges(myFace, mypMapToAvoid!);
        }
    }

    // Correct tolerances for Vertices on Edge
    private static void CheckEdge(in TopoDS_Edge Ed, double aMaxTol, NCollection_IndexedMap<TopoDS_Shape> aMapToAvoid)
    {
        TopoDS_Edge aE = Ed;
        aE.Orientation(TopAbs_FORWARD);
        double aTolE = BRep_Tool.Tolerance(aE);
        //
        BRep_TEdge TE = (BRep_TEdge)aE.TShape()!;
        //
        Placement Eloc = aE.Location();

        TopoDS_Iterator aItS = new(aE);
        for (; aItS.More(); aItS.Next())
        {
            TopoDS_Vertex aV = TopoDS.Vertex(aItS.Value());
            //
            BRep_TVertex TV = (BRep_TVertex)aV.TShape()!;
            Point3D aPV = TV.Pnt();
            //
            double aTol = BRep_Tool.Tolerance(aV);
            aTol = Max(aTol, aTolE);
            double dd = 0.1 * aTol;
            aTol *= aTol;
            //
            NCollection_List<BRep_CurveRepresentation>.Iterator aItCR = new(TE.Curves());
            while (aItCR.More())
            {
                BRep_CurveRepresentation aCR = aItCR.Value();
                //
                if (aCR.IsCurve3D())
                {
                    Geom_Curve? aC = aCR.Curve3D();
                    if (aC is not null)
                    {
                        Placement L = (Eloc * aCR.Location()).Predivided(aV.Location());
                        NCollection_List<BRep_PointRepresentation>.Iterator aItPR = new(TV.Points());
                        while (aItPR.More())
                        {
                            BRep_PointRepresentation aPR = aItPR.Value();
                            if (aPR.IsPointOnCurve(aC, L))
                            {
                                Point3D aPC = aC.Value(aPR.Parameter());
                                aPC.Transform(L.Transformation());
                                double aD2 = aPV.SquareDistance(aPC);
                                if (aD2 > aTol)
                                {
                                    double aNewTolerance = Math.Sqrt(aD2) + dd;
                                    if (aNewTolerance < aMaxTol)
                                    {
                                        UpdateShape(aV, aNewTolerance, aMapToAvoid);
                                    }
                                }
                            }
                            aItPR.Next();
                        }
                        //
                        TopAbs_Orientation aOrV = aV.Orientation();
                        if (aOrV == TopAbs_FORWARD || aOrV == TopAbs_REVERSED)
                        {
                            BRep_GCurve aGC = (BRep_GCurve)aCR;
                            Point3D aPC;
                            if (aOrV == TopAbs_FORWARD)
                            {
                                aPC = aC.Value(aGC.First());
                            }
                            else
                            {
                                aPC = aC.Value(aGC.Last());
                            }
                            aPC.Transform(L.Transformation());
                            //
                            double aD2 = aPV.SquareDistance(aPC);
                            if (aD2 > aTol)
                            {
                                double aNewTolerance = Math.Sqrt(aD2) + dd;
                                if (aNewTolerance < aMaxTol)
                                {
                                    UpdateShape(aV, aNewTolerance, aMapToAvoid);
                                }
                            }
                        }
                    }
                }
                aItCR.Next();
            } //  while (itcr.More()) {
        } // for (; aVExp.More(); aVExp.Next()) {
    }

    // Compute edge length and cache it in the map
    private static double MapEdgeLength(in TopoDS_Edge theEdge, NCollection_DataMap<TopoDS_Shape, double> theMapEdgeLen)
    {
        if (!theMapEdgeLen.Find(theEdge, out double aLen))
        {
            aLen = 0.0;
            if (!BRep_Tool.Degenerated(theEdge))
            {
                BRepAdaptor_Curve aCurve = new(theEdge);
                aLen = GCPnts_AbscissaPoint.Length(aCurve);
            }
            theMapEdgeLen.Bind(theEdge, aLen);
        }
        return aLen;
    }

    // OCCT's anonymous EdgeData: the data of an edge around a vertex, kept to avoid recomputing it.
    private readonly struct EdgeData(in TopoDS_Edge theEdge, double theVParameter, bool theIsClosed, Geom2dAdaptor_Curve theGAdaptor,
                                     double theFirst, double theLast)
    {
        public readonly TopoDS_Edge Edge = theEdge;                   // Edge
        public readonly double VParameter = theVParameter;            // Parameter of the vertex on the edge
        public readonly bool IsClosed = theIsClosed;                  // Closed flag of the edge
        public readonly Geom2dAdaptor_Curve GAdaptor = theGAdaptor;   // 2D adaptor for PCurve of the edge on the face
        public readonly double First = theFirst;                      // First parameter in the range
        public readonly double Last = theLast;                        // Last parameter in the rage
    }

    private static double IntersectCurves2d(in TopoDS_Vertex theV, Geom_Surface theS, in EdgeData theEData1, in EdgeData theEData2,
                                            NCollection_DataMap<TopoDS_Shape, double> theMapEdgeLen)
    {
        Geom2dInt_GInter anInter = new();
        // Range of the first edge
        double aT11 = theEData1.First;
        double aT12 = theEData1.Last;
        // Range of the second edge
        double aT21 = theEData2.First;
        double aT22 = theEData2.Last;

        double aMaxDist = 0.0;
        double aTol2d = 1.0e-10;
        //
        IntRes2d_Domain aDom1 = new(theEData1.GAdaptor.Value(aT11),
                                    aT11,
                                    aTol2d,
                                    theEData1.GAdaptor.Value(aT12),
                                    aT12,
                                    aTol2d);
        IntRes2d_Domain aDom2 = new(theEData2.GAdaptor.Value(aT21),
                                    aT21,
                                    aTol2d,
                                    theEData2.GAdaptor.Value(aT22),
                                    aT22,
                                    aTol2d);
        //
        anInter.Perform(theEData1.GAdaptor, aDom1, theEData2.GAdaptor, aDom2, aTol2d, aTol2d);
        if (!anInter.IsDone() || (anInter.NbSegments() == 0 && anInter.NbPoints() == 0))
        {
            return aMaxDist;
        }
        //
        double aT1, aT2, aTint1, aTint2, aHalfR1, aHalfR2, aDist;
        int i, aNb;
        Point3D aP, aPV;
        Point2D aP2d;
        NCollection_List<IntRes2d_IntersectionPoint> aLP = new();
        NCollection_List<IntRes2d_IntersectionPoint>.Iterator aItLP;
        //
        aPV = BRep_Tool.Pnt(theV);
        aT1 = theEData1.VParameter;
        aT2 = theEData2.VParameter;
        //
        aHalfR1 = (aT12 - aT11) / 2.0;
        aHalfR2 = (aT22 - aT21) / 2.0;
        //
        aNb = anInter.NbSegments();
        for (i = 1; i <= aNb; ++i)
        {
            IntRes2d_IntersectionSegment aSeg = anInter.Segment(i);
            aLP.Append(aSeg.FirstPoint());
            aLP.Append(aSeg.LastPoint());
        }
        //
        aNb = anInter.NbPoints();
        for (i = 1; i <= aNb; ++i)
        {
            IntRes2d_IntersectionPoint aPnt = anInter.Point(i);
            aLP.Append(aPnt);
        }
        //
        // evaluate the length of the smallest edge, so that not to return too large distance
        double aLen1 = MapEdgeLength(theEData1.Edge, theMapEdgeLen);
        double aLen2 = MapEdgeLength(theEData1.Edge, theMapEdgeLen); // OCCT: the first edge's length again
        const double MaxEdgePartCoveredByVertex = 0.3;
        double aMaxThresDist = Min(aLen1, aLen2) * MaxEdgePartCoveredByVertex;
        aMaxThresDist *= aMaxThresDist;
        aItLP = new(aLP);
        for (; aItLP.More(); aItLP.Next())
        {
            IntRes2d_IntersectionPoint aPnt = aItLP.Value();
            //
            aTint1 = aPnt.ParamOnFirst();
            aTint2 = aPnt.ParamOnSecond();
            //
            if ((aTint1 < aT11 || aTint1 > aT12) || (aTint2 < aT21 || aTint2 > aT22))
            {
                // out of range;
                continue;
            }
            //
            if ((!theEData1.IsClosed && Math.Abs(aTint1 - aT1) > aHalfR1)
                || (!theEData2.IsClosed && Math.Abs(aTint2 - aT2) > aHalfR2))
            {
                // intersection is on the other end of the edge
                continue;
            }
            //
            aP2d = aPnt.Value();
            theS.D0(aP2d.X(), aP2d.Y(), out aP);
            aDist = aPV.SquareDistance(aP);
            if (aDist > aMaxDist && aDist < aMaxThresDist)
            {
                aMaxDist = aDist;
            }
        }
        //
        return aMaxDist;
    }

    private static void CorrectWires(in TopoDS_Face aFx, NCollection_IndexedMap<TopoDS_Shape> aMapToAvoid)
    {
        int i, aNbV;
        double aTol, aTol2, aD2, aD2max;
        Point3D aP, aPV;
        Point2D aP2D;
        TopoDS_Face aF;
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMVE = new(TopTools_ShapeMapHasher.Instance);
        NCollection_List<TopoDS_Shape>.Iterator aIt = new();
        //
        aF = aFx;
        aF.Orientation(TopAbs_FORWARD);
        Geom_Surface aS = BRep_Tool.Surface(aFx)!;

        TopExp.MapShapesAndUniqueAncestors(aF, TopAbs_VERTEX, TopAbs_EDGE, aMVE, true);

        // OCCT's NCollection_DataMap<TopoDS_Shape, double> hashes with std::hash and compares with operator== (orientation included)
        NCollection_DataMap<TopoDS_Shape, double> aMapEdgeLen = new(EqualityComparer<TopoDS_Shape>.Default);
        aNbV = aMVE.Extent();
        for (i = 1; i <= aNbV; ++i)
        {
            TopoDS_Vertex aV = TopoDS.Vertex(aMVE.FindKey(i));
            aPV = BRep_Tool.Pnt(aV);
            aTol = BRep_Tool.Tolerance(aV);
            aTol2 = aTol * aTol;
            //
            aD2max = -1.0;
            // Save edge's data to avoid its recalculation during intersection of 2d curves
            List<EdgeData> aLEPars = [];
            NCollection_List<TopoDS_Shape> aLE = aMVE.FindFromIndex(i);
            aIt.Initialize(aLE);
            for (; aIt.More(); aIt.Next())
            {
                TopoDS_Edge aE = TopoDS.Edge(aIt.Value());
                Geom2d_Curve? aC2D = BRep_Tool.CurveOnSurface(aE, aF, out double aT1, out double aT2);
                if (aC2D is null)
                {
                    continue;
                }

                double aT = BRep_Tool.Parameter(aV, aE);
                bool isClosed = false;
                {
                    TopExp.Vertices(aE, out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);
                    isClosed = aV1.IsSame(aV2);
                }
                //
                aC2D.D0(aT, out aP2D);
                aS.D0(aP2D.X(), aP2D.Y(), out aP);
                aD2 = aPV.SquareDistance(aP);
                if (aD2 > aD2max)
                {
                    aD2max = aD2;
                }
                EdgeData anEData = new(aE, aT, isClosed, new Geom2dAdaptor_Curve(aC2D), aT1, aT2);
                aLEPars.Add(anEData);
            }
            //
            // check wires on self interference by intersecting 2d curves of the edges
            for (int i1 = 0; i1 < aLEPars.Count; ++i1)
            {
                EdgeData aEData1 = aLEPars[i1];
                TopoDS_Shape aE1 = aEData1.Edge;

                for (int i2 = i1 + 1; i2 < aLEPars.Count; ++i2)
                {
                    EdgeData aEData2 = aLEPars[i2];
                    TopoDS_Shape aE2 = aEData2.Edge;

                    if (aE1.IsSame(aE2))
                    {
                        continue;
                    }

                    aD2 = IntersectCurves2d(aV, aS, aEData1, aEData2, aMapEdgeLen);
                    if (aD2 > aD2max)
                    {
                        aD2max = aD2;
                    }
                }
            }
            //
            if (aD2max > aTol2)
            {
                aTol = 1.01 * Math.Sqrt(aD2max);
                UpdateShape(aV, aTol, aMapToAvoid);
            }
        } // for (i=1; i<=aNbV; ++i) {
    }

    private static void CorrectEdgeTolerance(in TopoDS_Edge myShape, in TopoDS_Face S, double aMaxTol,
                                             NCollection_IndexedMap<TopoDS_Shape> aMapToAvoid)
    {
        //
        // 1. Minimum of conditions to Perform
        BRep_CurveRepresentation? myCref = null;
        Adaptor3d_Curve? myHCurve = null;

        BRep_TEdge TEx = (BRep_TEdge)myShape.TShape()!;
        NCollection_List<BRep_CurveRepresentation>.Iterator itcrx = new(TEx.Curves());
        bool Degenerated, SameParameterx, SameRangex;

        int unique = 0;

        Degenerated = TEx.Degenerated();
        SameParameterx = TEx.SameParameter();
        SameRangex = TEx.SameRange();

        if (!SameRangex && SameParameterx)
        {
            return;
        }

        while (itcrx.More())
        {
            BRep_CurveRepresentation cr = itcrx.Value();
            if (cr.IsCurve3D())
            {
                unique++;
                if (myCref is null && cr.Curve3D() is not null)
                {
                    myCref = cr;
                }
            }
            itcrx.Next();
        }

        if (unique == 0)
        {
            return; //...No3DCurve
        }
        if (unique > 1)
        {
            return; //...Multiple3DCurve;
        }

        if (myCref is null && !Degenerated)
        {
            itcrx.Initialize(TEx.Curves());
            while (itcrx.More())
            {
                BRep_CurveRepresentation cr = itcrx.Value();
                if (cr.IsCurveOnSurface())
                {
                    myCref = cr;
                    break;
                }
                itcrx.Next();
            }
        }

        else if (myCref is not null && Degenerated)
        {
            return; //...InvalidDegeneratedFlag;
        }

        if (myCref is not null)
        {
            BRep_GCurve GCref = (BRep_GCurve)myCref;
            GCref.Range(out double First, out double Last);
            if (Last <= First)
            {
                return; // InvalidRange;
            }

            else
            {
                if (myCref.IsCurve3D())
                {
                    Geom_Curve C3dx = (Geom_Curve)myCref.Curve3D()!.Transformed(myCref.Location().Transformation());
                    GeomAdaptor_Curve GAC3d = new(C3dx, First, Last);
                    myHCurve = GAC3d;
                }
                else
                { // curve on surface
                    Geom_Surface Sref = myCref.Surface()!;
                    Sref = (Geom_Surface)Sref.Transformed(myCref.Location().Transformation());
                    Geom2d_Curve PCref = myCref.PCurve()!;
                    GeomAdaptor_Surface GAHSref = new(Sref);
                    Geom2dAdaptor_Curve GHPCref = new(PCref, First, Last);
                    Adaptor3d_CurveOnSurface ACSref = new(GHPCref, GAHSref);
                    myHCurve = ACSref;
                }
            }
        }

        //===============================================
        // 2. Tolerances in InContext
        {
            if (myCref is null)
            {
                return;
            }

            BRep_TEdge TE = (BRep_TEdge)myShape.TShape()!;
            double Tol = BRep_Tool.Tolerance(TopoDS.Edge(myShape));
            double aNewTol = Tol;

            bool SameParameter = TE.SameParameter();
            bool SameRange = TE.SameRange();
            double First = myHCurve!.FirstParameter();
            double Last = myHCurve.LastParameter();

            BRep_TFace TF = (BRep_TFace)S.TShape()!;
            Placement Floc = S.Location();
            Placement TFloc = TF.Location();
            Geom_Surface? Su = TF.Surface();
            Placement L = (Floc * TFloc).Predivided(myShape.Location());
            bool pcurvefound = false;

            NCollection_List<BRep_CurveRepresentation>.Iterator itcr = new(TE.Curves());
            while (itcr.More())
            {
                BRep_CurveRepresentation cr = itcr.Value();
                if (!ReferenceEquals(cr, myCref) && cr.IsCurveOnSurface(Su, L))
                {
                    pcurvefound = true;
                    BRep_GCurve GC = (BRep_GCurve)cr;
                    GC.Range(out double f, out double l);
                    if (SameRange && (f != First || l != Last))
                    {
                        return; // BRepCheck_InvalidSameRangeFlag;
                    }

                    Geom_Surface Sb;
                    _ = cr.Surface();
                    Sb = (Geom_Surface)Su!.Transformed(L.Transformation());
                    Geom2d_Curve PC = cr.PCurve()!;
                    GeomAdaptor_Surface GAHS = new(Sb);
                    Geom2dAdaptor_Curve GHPC = new(PC, f, l);
                    Adaptor3d_CurveOnSurface ACS = new(GHPC, GAHS);

                    BRepLib_ValidateEdge aValidateEdge = new(myHCurve, ACS, SameParameter);
                    aValidateEdge.Process();
                    aValidateEdge.UpdateTolerance(ref aNewTol);
                    if (aValidateEdge.IsDone() && !aValidateEdge.CheckTolerance(Tol))
                    {
                        if (aNewTol < aMaxTol)
                        {
                            UpdateShape(myShape, aNewTol, aMapToAvoid);
                            CorrectVertexTolerance(myShape, aMapToAvoid);
                        }
                    }

                    if (cr.IsCurveOnClosedSurface())
                    {
                        GHPC.Load(cr.PCurve2()!, f, l); // same bounds
                        ACS.Load(GHPC, GAHS);           // sans doute inutile

                        BRepLib_ValidateEdge aValidateEdgeOnClosedSurf = new(myHCurve, ACS, SameParameter);
                        aValidateEdgeOnClosedSurf.Process();
                        aValidateEdgeOnClosedSurf.UpdateTolerance(ref aNewTol);
                        if (aValidateEdgeOnClosedSurf.IsDone() && !aValidateEdgeOnClosedSurf.CheckTolerance(Tol))
                        {
                            if (aNewTol < aMaxTol)
                            {
                                UpdateShape(myShape, aNewTol, aMapToAvoid);
                                CorrectVertexTolerance(myShape, aMapToAvoid);
                            }
                        }
                    }
                }
                itcr.Next();
            }

            if (!pcurvefound)
            {
                Geom_Plane? P;
                if (Su is Geom_RectangularTrimmedSurface aRTS)
                {
                    P = aRTS.BasisSurface() as Geom_Plane;
                }
                else
                {
                    P = Su as Geom_Plane;
                }
                if (P is null)
                { // not a plane
                    return;
                }

                else
                { // on fait la projection a la volee, comme BRep_Tool
                    P = (Geom_Plane)P.Transformed(L.Transformation());
                    // on projette Cref sur ce plan
                    GeomAdaptor_Surface GAHS = new(P);

                    // Dub - Normalement myHCurve est une GeomAdaptor_Curve
                    // OCCT down-casts and dereferences: a reference curve on a surface gives a null handle here
                    GeomAdaptor_Curve Gac = myHCurve as GeomAdaptor_Curve
                                            ?? throw new NullObjectException("BOPTools_AlgoTools::CorrectEdgeTolerance - no 3D curve");
                    Geom_Curve C3dx = Gac.Curve()!;
                    Geom_Curve ProjOnPlane =
                        GeomProjLib.ProjectOnPlane(new Geom_TrimmedCurve(C3dx, First, Last),
                                                   P,
                                                   P.Position().Direction(),
                                                   true)!;

                    GeomAdaptor_Curve aHCurve = new(ProjOnPlane);

                    ProjLib_ProjectedCurve proj = new(GAHS, aHCurve);
                    Geom2d_Curve PC = Geom2dAdaptor.MakeCurve(proj);
                    Geom2dAdaptor_Curve GHPC = new(PC, myHCurve.FirstParameter(), myHCurve.LastParameter());

                    Adaptor3d_CurveOnSurface ACS = new(GHPC, GAHS);

                    BRepLib_ValidateEdge aValidateProjEdge = new(myHCurve, ACS, true);
                    aValidateProjEdge.Process();
                    aValidateProjEdge.UpdateTolerance(ref aNewTol);
                    if (aValidateProjEdge.IsDone() && !aValidateProjEdge.CheckTolerance(Tol))
                    {
                        if (aNewTol < aMaxTol)
                        {
                            UpdateShape(myShape, aNewTol, aMapToAvoid);
                            CorrectVertexTolerance(myShape, aMapToAvoid);
                        }
                    }
                }
            } // end of if (!pcurvefound) {
        } // end of  2. Tolerances in InContext
    }

    private static void CorrectVertexTolerance(in TopoDS_Edge aE, NCollection_IndexedMap<TopoDS_Shape> aMapToAvoid)
    {
        double aTolE, aTolV;
        TopoDS_Iterator aIt = new();
        //
        aTolE = BRep_Tool.Tolerance(aE);
        aIt.Initialize(aE);
        for (; aIt.More(); aIt.Next())
        {
            TopoDS_Vertex aV = TopoDS.Vertex(aIt.Value());
            aTolV = BRep_Tool.Tolerance(aV);
            if (aTolV < aTolE)
            {
                UpdateShape(aV, aTolE, aMapToAvoid);
            }
        }
    }

    private static void UpdateEdges(in TopoDS_Face aF, NCollection_IndexedMap<TopoDS_Shape> aMapToAvoid)
    {
        double aTolF, aTolE, aTolV;
        TopoDS_Iterator aItF = new(), aItW = new();
        //
        aTolE = aTolF = BRep_Tool.Tolerance(aF);
        aItF.Initialize(aF);
        for (; aItF.More(); aItF.Next())
        {
            TopoDS_Shape aS = aItF.Value();
            if (aS.ShapeType() == TopAbs_WIRE)
            {
                aItW.Initialize(aS);
                for (; aItW.More(); aItW.Next())
                {
                    TopoDS_Edge aE = TopoDS.Edge(aItW.Value());
                    aTolE = BRep_Tool.Tolerance(aE);
                    if (aTolE < aTolF)
                    {
                        UpdateShape(aE, aTolF, aMapToAvoid);
                        aTolE = aTolF;
                    }
                }
            }
            else
            {
                TopoDS_Vertex aV = TopoDS.Vertex(aItF.Value());
                aTolV = BRep_Tool.Tolerance(aV);
                if (aTolV < aTolE)
                {
                    UpdateShape(aV, aTolF, aMapToAvoid);
                }
            }
        }
    }

    // The only write site of the corrections: grows the edge's or vertex's tolerance (never shrinks it).
    private static void UpdateShape(in TopoDS_Shape aS, double aTol, NCollection_IndexedMap<TopoDS_Shape> aMapToAvoid)
    {
        if (aMapToAvoid.Contains(aS))
        {
            return;
        }
        //
        TopAbs_ShapeEnum aType;
        BRep_Builder aBB = new();
        //
        aType = aS.ShapeType();
        if (aType == TopAbs_EDGE)
        {
            TopoDS_Edge aE = TopoDS.Edge(aS);
            aBB.UpdateEdge(aE, aTol);
        }
        else if (aType == TopAbs_VERTEX)
        {
            TopoDS_Vertex aV = TopoDS.Vertex(aS);
            aBB.UpdateVertex(aV, aTol);
        }
    }
}
