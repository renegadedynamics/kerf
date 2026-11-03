// Port of: src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_AlgoTools_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Projection;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

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
    public static void CorrectTolerances(in Shape aShape, NCollection_IndexedMap<Shape> aMapToAvoid, double aMaxTol = 0.0001,
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
    public static void CorrectPointOnCurve(in Shape aS, NCollection_IndexedMap<Shape> aMapToAvoid, double aMaxTol = 0.0001,
                                           bool bRunParallel = false)
    {
        Explorer aExp = new();
        List<BOPTools_CPC> aVCPC = [];
        //
        aExp.Init(aS, ShapeType.Edge);
        for (; aExp.More(); aExp.Next())
        {
            Edge aE = ShapeCasts.Edge(aExp.Current());
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
    public static void CorrectCurveOnSurface(in Shape aS, NCollection_IndexedMap<Shape> aMapToAvoid, double aMaxTol = 0.0001,
                                             bool bRunParallel = false)
    {
        Explorer aExpF = new(), aExpE = new();
        List<BOPTools_CWT> aVCWT = [];
        List<BOPTools_CDT> aVCDT = [];
        //
        aExpF.Init(aS, ShapeType.Face);
        for (; aExpF.More(); aExpF.Next())
        {
            Face aF = ShapeCasts.Face(aExpF.Current());
            //
            BOPTools_CWT aCWT = new();
            aVCWT.Add(aCWT);
            aCWT.SetFace(aF);
            aCWT.SetMapToAvoid(aMapToAvoid);
            //
            aExpE.Init(aF, ShapeType.Edge);
            for (; aExpE.More(); aExpE.Next())
            {
                Edge aE = ShapeCasts.Edge(aExpE.Current());
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
    public static void CorrectShapeTolerances(in Shape aShape, NCollection_IndexedMap<Shape> aMapToAvoid,
                                              bool bRunParallel = false)
    {
        Explorer aExp = new();
        List<BOPTools_CVT> aVCVT = [];
        List<BOPTools_CET> aVCET = [];
        //
        aExp.Init(aShape, ShapeType.Edge);
        for (; aExp.More(); aExp.Next())
        {
            Edge aE = ShapeCasts.Edge(aExp.Current());
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
        aExp.Init(aShape, ShapeType.Face);
        for (; aExp.More(); aExp.Next())
        {
            Face aF = ShapeCasts.Face(aExp.Current());
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
    public static bool ComputeTolerance(in Face theFace, in Edge theEdge, out double theMaxDist, out double theMaxPar)
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
        private Edge myEdge;
        private NCollection_IndexedMap<Shape>? mypMapToAvoid;

        internal void SetEdge(in Edge aE) => myEdge = aE;

        internal Edge Edge() => myEdge;

        internal void SetMaxTol(double aMaxTol) => myMaxTol = aMaxTol;

        internal double MaxTol() => myMaxTol;

        internal void SetMapToAvoid(NCollection_IndexedMap<Shape> aMapToAvoid) => mypMapToAvoid = aMapToAvoid;

        public void Perform()
        {
            Standard_ProgramError_Raise_if(mypMapToAvoid is null, "mypMapToAvoid is null");
            CheckEdge(myEdge, myMaxTol, mypMapToAvoid!);
        }
    }

    private sealed class BOPTools_CWT : BOPTools_Parallel.ISolver
    {
        private Face myFace;
        private NCollection_IndexedMap<Shape>? mypMapToAvoid;

        internal void SetFace(in Face aF) => myFace = aF;

        internal void SetMapToAvoid(NCollection_IndexedMap<Shape> aMapToAvoid) => mypMapToAvoid = aMapToAvoid;

        public void Perform()
        {
            Standard_ProgramError_Raise_if(mypMapToAvoid is null, "mypMapToAvoid is null");
            CorrectWires(myFace, mypMapToAvoid!);
        }
    }

    private sealed class BOPTools_CDT : BOPTools_Parallel.ISolver
    {
        private double myMaxTol = 1.0e-7;
        private Edge myEdge;
        private Face myFace;
        private NCollection_IndexedMap<Shape>? mypMapToAvoid;

        internal void SetEdge(in Edge aE) => myEdge = aE;

        internal void SetFace(in Face aF) => myFace = aF;

        internal void SetMaxTol(double aMaxTol) => myMaxTol = aMaxTol;

        internal void SetMapToAvoid(NCollection_IndexedMap<Shape> aMapToAvoid) => mypMapToAvoid = aMapToAvoid;

        public void Perform()
        {
            Standard_ProgramError_Raise_if(mypMapToAvoid is null, "mypMapToAvoid is null");
            CorrectEdgeTolerance(myEdge, myFace, myMaxTol, mypMapToAvoid!);
        }
    }

    private sealed class BOPTools_CVT : BOPTools_Parallel.ISolver
    {
        private Edge myEdge;
        private NCollection_IndexedMap<Shape>? mypMapToAvoid;

        internal void SetEdge(in Edge aE) => myEdge = aE;

        internal void SetMapToAvoid(NCollection_IndexedMap<Shape> aMapToAvoid) => mypMapToAvoid = aMapToAvoid;

        public void Perform()
        {
            Standard_ProgramError_Raise_if(mypMapToAvoid is null, "mypMapToAvoid is null");
            CorrectVertexTolerance(myEdge, mypMapToAvoid!);
        }
    }

    private sealed class BOPTools_CET : BOPTools_Parallel.ISolver
    {
        private Face myFace;
        private NCollection_IndexedMap<Shape>? mypMapToAvoid;

        internal void SetFace(in Face aF) => myFace = aF;

        internal void SetMapToAvoid(NCollection_IndexedMap<Shape> aMapToAvoid) => mypMapToAvoid = aMapToAvoid;

        public void Perform()
        {
            Standard_ProgramError_Raise_if(mypMapToAvoid is null, "mypMapToAvoid is null");
            UpdateEdges(myFace, mypMapToAvoid!);
        }
    }

    // Correct tolerances for Vertices on Edge
    private static void CheckEdge(in Edge Ed, double aMaxTol, NCollection_IndexedMap<Shape> aMapToAvoid)
    {
        Edge aE = Ed;
        aE.Orientation(Forward);
        double aTolE = Tool.Tolerance(aE);
        //
        BrepTEdge TE = (BrepTEdge)aE.TShape()!;
        //
        Placement Eloc = aE.Location();

        Iterator aItS = new(aE);
        for (; aItS.More(); aItS.Next())
        {
            Vertex aV = ShapeCasts.Vertex(aItS.Value());
            //
            BrepTVertex TV = (BrepTVertex)aV.TShape()!;
            Point3D aPV = TV.Pnt();
            //
            double aTol = Tool.Tolerance(aV);
            aTol = Max(aTol, aTolE);
            double dd = 0.1 * aTol;
            aTol *= aTol;
            //
            NCollection_List<CurveRepresentation>.Iterator aItCR = new(TE.Curves());
            while (aItCR.More())
            {
                CurveRepresentation aCR = aItCR.Value();
                //
                if (aCR.IsCurve3D)
                {
                    Curve3D? aC = aCR.Curve3D();
                    if (aC is not null)
                    {
                        Placement L = (Eloc * aCR.Location()).Predivided(aV.Location());
                        NCollection_List<PointRepresentation>.Iterator aItPR = new(TV.Points());
                        while (aItPR.More())
                        {
                            PointRepresentation aPR = aItPR.Value();
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
                        Orientation aOrV = aV.Orientation();
                        if (aOrV == Forward || aOrV == Reversed)
                        {
                            GCurve aGC = (GCurve)aCR;
                            Point3D aPC;
                            if (aOrV == Forward)
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
    private static double MapEdgeLength(in Edge theEdge, NCollection_DataMap<Shape, double> theMapEdgeLen)
    {
        if (!theMapEdgeLen.Find(theEdge, out double aLen))
        {
            aLen = 0.0;
            if (!Tool.Degenerated(theEdge))
            {
                EdgeCurveAdaptor aCurve = new(theEdge);
                aLen = AbscissaPoint.Length(aCurve);
            }
            theMapEdgeLen.Bind(theEdge, aLen);
        }
        return aLen;
    }

    // OCCT's anonymous EdgeData: the data of an edge around a vertex, kept to avoid recomputing it.
    private readonly struct EdgeData(in Edge theEdge, double theVParameter, bool theIsClosed, CurveAdaptor2D theGAdaptor,
                                     double theFirst, double theLast)
    {
        public readonly Edge Edge = theEdge;                   // Edge
        public readonly double VParameter = theVParameter;            // Parameter of the vertex on the edge
        public readonly bool IsClosed = theIsClosed;                  // Closed flag of the edge
        public readonly CurveAdaptor2D GAdaptor = theGAdaptor;   // 2D adaptor for PCurve of the edge on the face
        public readonly double First = theFirst;                      // First parameter in the range
        public readonly double Last = theLast;                        // Last parameter in the rage
    }

    private static double IntersectCurves2d(in Vertex theV, Surface theS, in EdgeData theEData1, in EdgeData theEData2,
                                            NCollection_DataMap<Shape, double> theMapEdgeLen)
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
        aPV = Tool.Pnt(theV);
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

    private static void CorrectWires(in Face aFx, NCollection_IndexedMap<Shape> aMapToAvoid)
    {
        int i, aNbV;
        double aTol, aTol2, aD2, aD2max;
        Point3D aP, aPV;
        Point2D aP2D;
        Face aF;
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aMVE = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator aIt = new();
        //
        aF = aFx;
        aF.Orientation(Forward);
        Surface aS = Tool.Surface(aFx)!;

        Exploration.MapShapesAndUniqueAncestors(aF, ShapeType.Vertex, ShapeType.Edge, aMVE, true);

        // OCCT's NCollection_DataMap<TopoDS_Shape, double> hashes with std::hash and compares with operator== (orientation included)
        NCollection_DataMap<Shape, double> aMapEdgeLen = new(EqualityComparer<Shape>.Default);
        aNbV = aMVE.Extent();
        for (i = 1; i <= aNbV; ++i)
        {
            Vertex aV = ShapeCasts.Vertex(aMVE.FindKey(i));
            aPV = Tool.Pnt(aV);
            aTol = Tool.Tolerance(aV);
            aTol2 = aTol * aTol;
            //
            aD2max = -1.0;
            // Save edge's data to avoid its recalculation during intersection of 2d curves
            List<EdgeData> aLEPars = [];
            NCollection_List<Shape> aLE = aMVE.FindFromIndex(i);
            aIt.Initialize(aLE);
            for (; aIt.More(); aIt.Next())
            {
                Edge aE = ShapeCasts.Edge(aIt.Value());
                Curve2D? aC2D = Tool.CurveOnSurface(aE, aF, out double aT1, out double aT2);
                if (aC2D is null)
                {
                    continue;
                }

                double aT = Tool.Parameter(aV, aE);
                bool isClosed = false;
                {
                    Exploration.Vertices(aE, out Vertex aV1, out Vertex aV2);
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
                EdgeData anEData = new(aE, aT, isClosed, new CurveAdaptor2D(aC2D), aT1, aT2);
                aLEPars.Add(anEData);
            }
            //
            // check wires on self interference by intersecting 2d curves of the edges
            for (int i1 = 0; i1 < aLEPars.Count; ++i1)
            {
                EdgeData aEData1 = aLEPars[i1];
                Shape aE1 = aEData1.Edge;

                for (int i2 = i1 + 1; i2 < aLEPars.Count; ++i2)
                {
                    EdgeData aEData2 = aLEPars[i2];
                    Shape aE2 = aEData2.Edge;

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

    private static void CorrectEdgeTolerance(in Edge myShape, in Face S, double aMaxTol,
                                             NCollection_IndexedMap<Shape> aMapToAvoid)
    {
        //
        // 1. Minimum of conditions to Perform
        CurveRepresentation? myCref = null;
        CurveAdaptorBase? myHCurve = null;

        BrepTEdge TEx = (BrepTEdge)myShape.TShape()!;
        NCollection_List<CurveRepresentation>.Iterator itcrx = new(TEx.Curves());
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
            CurveRepresentation cr = itcrx.Value();
            if (cr.IsCurve3D)
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
                CurveRepresentation cr = itcrx.Value();
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
            GCurve GCref = (GCurve)myCref;
            GCref.Range(out double First, out double Last);
            if (Last <= First)
            {
                return; // InvalidRange;
            }

            else
            {
                if (myCref.IsCurve3D)
                {
                    Curve3D C3dx = (Curve3D)myCref.Curve3D()!.Transformed(myCref.Location().Transformation());
                    CurveAdaptor GAC3d = new(C3dx, First, Last);
                    myHCurve = GAC3d;
                }
                else
                { // curve on surface
                    Surface Sref = myCref.Surface()!;
                    Sref = (Surface)Sref.Transformed(myCref.Location().Transformation());
                    Curve2D PCref = myCref.PCurve()!;
                    SurfaceAdaptor GAHSref = new(Sref);
                    CurveAdaptor2D GHPCref = new(PCref, First, Last);
                    CurveOnSurface ACSref = new(GHPCref, GAHSref);
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

            BrepTEdge TE = (BrepTEdge)myShape.TShape()!;
            double Tol = Tool.Tolerance(ShapeCasts.Edge(myShape));
            double aNewTol = Tol;

            bool SameParameter = TE.SameParameter();
            bool SameRange = TE.SameRange();
            double First = myHCurve!.FirstParameter();
            double Last = myHCurve.LastParameter();

            BrepTFace TF = (BrepTFace)S.TShape()!;
            Placement Floc = S.Location();
            Placement TFloc = TF.Location();
            Surface? Su = TF.Surface();
            Placement L = (Floc * TFloc).Predivided(myShape.Location());
            bool pcurvefound = false;

            NCollection_List<CurveRepresentation>.Iterator itcr = new(TE.Curves());
            while (itcr.More())
            {
                CurveRepresentation cr = itcr.Value();
                if (!ReferenceEquals(cr, myCref) && cr.IsCurveOnSurface(Su, L))
                {
                    pcurvefound = true;
                    GCurve GC = (GCurve)cr;
                    GC.Range(out double f, out double l);
                    if (SameRange && (f != First || l != Last))
                    {
                        return; // BRepCheck_InvalidSameRangeFlag;
                    }

                    Surface Sb;
                    _ = cr.Surface();
                    Sb = (Surface)Su!.Transformed(L.Transformation());
                    Curve2D PC = cr.PCurve()!;
                    SurfaceAdaptor GAHS = new(Sb);
                    CurveAdaptor2D GHPC = new(PC, f, l);
                    CurveOnSurface ACS = new(GHPC, GAHS);

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

                    if (cr.IsCurveOnClosedSurface)
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
                PlaneSurface? P;
                if (Su is TrimmedSurface aRTS)
                {
                    P = aRTS.BasisSurface() as PlaneSurface;
                }
                else
                {
                    P = Su as PlaneSurface;
                }
                if (P is null)
                { // not a plane
                    return;
                }

                else
                { // on fait la projection a la volee, comme BRep_Tool
                    P = (PlaneSurface)P.Transformed(L.Transformation());
                    // on projette Cref sur ce plan
                    SurfaceAdaptor GAHS = new(P);

                    // Dub - Normalement myHCurve est une GeomAdaptor_Curve
                    // OCCT down-casts and dereferences: a reference curve on a surface gives a null handle here
                    CurveAdaptor Gac = myHCurve as CurveAdaptor
                                            ?? throw new NullObjectException("BOPTools_AlgoTools::CorrectEdgeTolerance - no 3D curve");
                    Curve3D C3dx = Gac.Curve()!;
                    Curve3D ProjOnPlane =
                        global::Kerf.Engine.Geometry.Projection.GeomProjLib.ProjectOnPlane(new TrimmedCurve3D(C3dx, First, Last),
                                                   P,
                                                   P.Position().Direction(),
                                                   true)!;

                    CurveAdaptor aHCurve = new(ProjOnPlane);

                    ProjectedCurve proj = new(GAHS, aHCurve);
                    Curve2D PC = global::Kerf.Engine.Geometry.Adaptors.Geom2dAdaptor.MakeCurve(proj);
                    CurveAdaptor2D GHPC = new(PC, myHCurve.FirstParameter(), myHCurve.LastParameter());

                    CurveOnSurface ACS = new(GHPC, GAHS);

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

    private static void CorrectVertexTolerance(in Edge aE, NCollection_IndexedMap<Shape> aMapToAvoid)
    {
        double aTolE, aTolV;
        Iterator aIt = new();
        //
        aTolE = Tool.Tolerance(aE);
        aIt.Initialize(aE);
        for (; aIt.More(); aIt.Next())
        {
            Vertex aV = ShapeCasts.Vertex(aIt.Value());
            aTolV = Tool.Tolerance(aV);
            if (aTolV < aTolE)
            {
                UpdateShape(aV, aTolE, aMapToAvoid);
            }
        }
    }

    private static void UpdateEdges(in Face aF, NCollection_IndexedMap<Shape> aMapToAvoid)
    {
        double aTolF, aTolE, aTolV;
        Iterator aItF = new(), aItW = new();
        //
        aTolE = aTolF = Tool.Tolerance(aF);
        aItF.Initialize(aF);
        for (; aItF.More(); aItF.Next())
        {
            Shape aS = aItF.Value();
            if (aS.ShapeType() == ShapeType.Wire)
            {
                aItW.Initialize(aS);
                for (; aItW.More(); aItW.Next())
                {
                    Edge aE = ShapeCasts.Edge(aItW.Value());
                    aTolE = Tool.Tolerance(aE);
                    if (aTolE < aTolF)
                    {
                        UpdateShape(aE, aTolF, aMapToAvoid);
                        aTolE = aTolF;
                    }
                }
            }
            else
            {
                Vertex aV = ShapeCasts.Vertex(aItF.Value());
                aTolV = Tool.Tolerance(aV);
                if (aTolV < aTolE)
                {
                    UpdateShape(aV, aTolF, aMapToAvoid);
                }
            }
        }
    }

    // The only write site of the corrections: grows the edge's or vertex's tolerance (never shrinks it).
    private static void UpdateShape(in Shape aS, double aTol, NCollection_IndexedMap<Shape> aMapToAvoid)
    {
        if (aMapToAvoid.Contains(aS))
        {
            return;
        }
        //
        ShapeType aType;
        BrepBuilder aBB = new();
        //
        aType = aS.ShapeType();
        if (aType == ShapeType.Edge)
        {
            Edge aE = ShapeCasts.Edge(aS);
            aBB.UpdateEdge(aE, aTol);
        }
        else if (aType == ShapeType.Vertex)
        {
            Vertex aV = ShapeCasts.Vertex(aS);
            aBB.UpdateVertex(aV, aTol);
        }
    }
}
