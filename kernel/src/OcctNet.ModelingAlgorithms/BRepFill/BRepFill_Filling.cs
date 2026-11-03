// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Filling.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Filling.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Schematiq.Collections;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.Orientation;

namespace Occt;

/// <summary>
/// N-sided filling: a face bounded by edges (each possibly on a support face it must meet with G0, G1 or G2
/// continuity), passing through further curve and point constraints, built on a plate surface
/// (GeomPlate_BuildPlateSurface) approximated by a B-spline surface.
/// </summary>
/// <remarks>
/// A working object (decision 21), as OCCT's: constraints are added, then <see cref="Build"/>. It never writes
/// into the caller's shapes: the result's edges are new (empty copies of the boundary edges) on new vertices.
/// <see cref="Face"/> freezes the face it returns, and <see cref="Generated"/> the shapes it lists; ported code
/// reads <see cref="InternalFace"/> and <see cref="InternalGenerated"/>. As in OCCT, each <see cref="Build"/>
/// starts a new plate from the constraints, but keeps what earlier builds recorded: a boundary edge is then rebuilt
/// from the previous build's edge, and the bounds found for free constraints stay in the boundary.
/// </remarks>
public class BRepFill_Filling
{
    private GeomPlate_BuildPlateSurface? myBuilder;
    private readonly NCollection_Sequence<BRepFill_EdgeFaceAndOrder> myBoundary = new();
    private readonly NCollection_Sequence<BRepFill_EdgeFaceAndOrder> myConstraints = new();
    private readonly NCollection_Sequence<BRepFill_FaceAndOrder> myFreeConstraints = new();
    private readonly NCollection_Sequence<GeomPlate_PointConstraint> myPoints = new();
    private readonly NCollection_DataMap<Shape, Shape> myOldNewMap = new(ShapeHasher.Instance);
    // OCCT.NET: frozen. The boundary's vertices BuildWires' MakeWire replaced by its copies for write (see BuildWires).
    private readonly NCollection_DataMap<Shape, Shape> myVertexCopies = new(ShapeHasher.Instance);
    private readonly NCollection_List<Shape> myGenerated = new();
    private Face myFace = new();
    private Face myInitFace = new();
    private double myTol2d;
    private double myTol3d;
    private double myTolAng;
    private double myTolCurv;
    private int myMaxDeg;
    private int myMaxSegments;
    private int myDegree;
    private int myNbPtsOnCur;
    private int myNbIter;
    private bool myAnisotropie;
    private bool myIsInitFaceGiven;
    private bool myIsDone;

    private static Vector3D MakeFinVec(in Wire aWire, in Vertex aVertex)
    {
        Vertex Vfirst, Vlast, Origin = new Vertex();
        WireExplorer Explo = new WireExplorer(aWire);
        for (; Explo.More(); Explo.Next())
        {
            Exploration.Vertices(Explo.Current(), out Vfirst, out Vlast);
            if (Vfirst.IsSame(aVertex))
            {
                Origin = Vlast;
                break;
            }
            if (Vlast.IsSame(aVertex))
            {
                Origin = Vfirst;
                break;
            }
        }
        return new Vector3D(Tool.Pnt(Origin), Tool.Pnt(aVertex));
    }

    private static Wire WireFromList(NCollection_List<Shape> Edges)
    {
        BrepBuilder BB = new BrepBuilder();
        Wire aWire = new Wire();
        BB.MakeWire(ref aWire);
        Edge anEdge = ShapeCasts.Edge(Edges.First());
        BB.Add(aWire, anEdge);
        Edges.RemoveFirst();

        Vertex V1, V2;
        Exploration.Vertices(anEdge, out V1, out V2, true); // with orientation

        while (!Edges.IsEmpty())
        {
            NCollection_List<Shape>.Iterator itl = new NCollection_List<Shape>.Iterator(Edges);
            for (; itl.More(); itl.Next())
            {
                anEdge = ShapeCasts.Edge(itl.Value());
                Vertex V3, V4;
                Exploration.Vertices(anEdge, out V3, out V4, true); // with orientation
                if (V1.IsSame(V3) || V1.IsSame(V4) || V2.IsSame(V3) || V2.IsSame(V4))
                {
                    if (V1.IsSame(V3))
                    {
                        anEdge.Reverse();
                        V1 = V4;
                    }
                    else if (V1.IsSame(V4))
                    {
                        V1 = V3;
                    }
                    else if (V2.IsSame(V3))
                    {
                        V2 = V4;
                    }
                    else
                    {
                        anEdge.Reverse();
                        V2 = V3;
                    }
                    break;
                }
            }
            if (!itl.More())
            {
                global::Kerf.Engine.Foundation.Message.SendWarning("Warning: WireFromList: can't find the next edge. The wire is not "
                                    + "complete, some edges are lost.");
                break;
            }
            BB.Add(aWire, anEdge);
            Edges.Remove(itl);
        }

        aWire.Closed(true);
        return aWire;
    }

    public BRepFill_Filling(int Degree = 3, int NbPtsOnCur = 15, int NbIter = 2, bool Anisotropie = false,
                            double Tol2d = 0.00001, double Tol3d = 0.0001, double TolAng = 0.01, double TolCurv = 0.1,
                            int MaxDeg = 8, int MaxSegments = 9)
    {
        myDegree = Degree;
        myNbPtsOnCur = NbPtsOnCur;
        myNbIter = NbIter;
        myAnisotropie = Anisotropie;

        myTol2d = Tol2d;
        myTol3d = Tol3d;
        myTolAng = TolAng;
        myTolCurv = TolCurv;

        myMaxDeg = MaxDeg;
        myMaxSegments = MaxSegments;

        myIsInitFaceGiven = false;

        myIsDone = false;
    }

    /// <summary>Sets the tolerances of the constraints.</summary>
    public void SetConstrParam(double Tol2d = 0.00001, double Tol3d = 0.0001, double TolAng = 0.01, double TolCurv = 0.1)
    {
        myTol2d = Tol2d;
        myTol3d = Tol3d;
        myTolAng = TolAng;
        myTolCurv = TolCurv;
    }

    /// <summary>Sets the parameters of the plate's resolution.</summary>
    public void SetResolParam(int Degree = 3, int NbPtsOnCur = 15, int NbIter = 2, bool Anisotropie = false)
    {
        myDegree = Degree;
        myNbPtsOnCur = NbPtsOnCur;
        myNbIter = NbIter;
        myAnisotropie = Anisotropie;
    }

    /// <summary>Sets the parameters of the approximation of the plate surface.</summary>
    public void SetApproxParam(int MaxDeg = 8, int MaxSegments = 9)
    {
        myMaxDeg = MaxDeg;
        myMaxSegments = MaxSegments;
    }

    /// <summary>Loads the face whose surface the filling starts from.</summary>
    public void LoadInitSurface(in Face aFace)
    {
        myInitFace = aFace;
        myIsInitFaceGiven = true;
    }

    /// <summary>Adds a boundary edge (or, with <paramref name="IsBound"/> false, a free edge) of order G0 (no face).</summary>
    /// <returns>The index of the constraint.</returns>
    public int Add(in Edge anEdge, Continuity Order, bool IsBound = true)
    {
        Face NullFace = new Face();
        BRepFill_EdgeFaceAndOrder EdgeFaceAndOrder = new BRepFill_EdgeFaceAndOrder(anEdge, NullFace, Order);
        if (IsBound)
        {
            myBoundary.Append(EdgeFaceAndOrder);
            return myBoundary.Length();
        }
        else
        {
            myConstraints.Append(EdgeFaceAndOrder);
            return (myBoundary.Length() + myFreeConstraints.Length() + myConstraints.Length());
        }
    }

    /// <summary>Adds an edge with its support face as a constraint of order <paramref name="Order"/>.</summary>
    /// <returns>The index of the constraint.</returns>
    public int Add(in Edge anEdge, in Face Support, Continuity Order, bool IsBound = true)
    {
        BRepFill_EdgeFaceAndOrder EdgeFaceAndOrder = new BRepFill_EdgeFaceAndOrder(anEdge, Support, Order);
        if (IsBound)
        {
            myBoundary.Append(EdgeFaceAndOrder);
            return myBoundary.Length();
        }
        else
        {
            myConstraints.Append(EdgeFaceAndOrder);
            return (myBoundary.Length() + myFreeConstraints.Length() + myConstraints.Length());
        }
    }

    /// <summary>Adds a "free constraint": a face that bounds the filling where its boundary has a gap.</summary>
    /// <returns>The index of the constraint.</returns>
    public int Add(in Face Support, Continuity Order)
    {
        BRepFill_FaceAndOrder FaceAndOrder = new BRepFill_FaceAndOrder(Support, Order);
        myFreeConstraints.Append(FaceAndOrder);
        return (myBoundary.Length() + myFreeConstraints.Length());
    }

    /// <summary>Adds a point the filling passes through.</summary>
    /// <returns>The index of the constraint.</returns>
    public int Add(in Point3D Point)
    {
        GeomPlate_PointConstraint aPC = new GeomPlate_PointConstraint(Point, (int)C0, myTol3d);
        myPoints.Append(aPC);
        return (myBoundary.Length() + myFreeConstraints.Length() + myConstraints.Length() + myPoints.Length());
    }

    /// <summary>Adds a point constraint on a face, of order <paramref name="Order"/>.</summary>
    /// <returns>The index of the constraint.</returns>
    public int Add(double U, double V, in Face Support, Continuity Order)
    {
        FaceSurfaceAdaptor HSurf = new FaceSurfaceAdaptor();
        HSurf.Initialize(Support);
        GeomPlate_PointConstraint aPC = new GeomPlate_PointConstraint(U, V, Tool.Surface(HSurf.Face())!, (int)Order,
                                                                      myTol3d, myTolAng, myTolCurv);
        myPoints.Append(aPC);
        return (myBoundary.Length() + myFreeConstraints.Length() + myConstraints.Length() + myPoints.Length());
    }

    // OCCT.NET: OCCT's sweeps of an edge (MakePrism, MakeRevol, ...) write the new face's pcurve into the caller's edge,
    // which the face shares; the port sweeps a frozen edge as a copy (decision 468), so the caller's edge has no pcurve
    // on the face the sweep made, and the constraint would read a null pcurve (OCCT's comment below). For a frozen edge
    // without a pcurve on the support, the support's edge on the same 3D curve, location and range stands for it, as it
    // is OCCT's edge itself (decision 1103).
    private static Edge EdgeOnSupport(in Edge theEdge, in Face theSupport)
    {
        if (!theEdge.TShape()!.IsFrozen || Tool.CurveOnSurface(theEdge, theSupport, out _, out _) is not null)
        {
            return theEdge;
        }
        Curve3D? aCurve = Tool.Curve(theEdge, out Placement aLoc, out double aFirst, out double aLast);
        if (aCurve is null)
        {
            return theEdge;
        }
        for (Explorer anExp = new(theSupport, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Edge anEdge = ShapeCasts.Edge(anExp.Current());
            Curve3D? aC = Tool.Curve(anEdge, out Placement aL, out double aF, out double aLa);
            if (ReferenceEquals(aC, aCurve) && aL == aLoc && aF == aFirst && aLa == aLast)
            {
                return ShapeCasts.Edge(anEdge.Oriented(theEdge.Orientation()));
            }
        }
        return theEdge;
    }

    private void AddConstraints(NCollection_Sequence<BRepFill_EdgeFaceAndOrder> SeqOfConstraints)
    {
        Edge CurEdge;
        Face CurFace;
        Continuity CurOrder;

        GeomPlate_CurveConstraint Constr;
        int i;
        for (i = 1; i <= SeqOfConstraints.Length(); i++)
        {
            CurEdge = SeqOfConstraints.Value(i).myEdge;
            CurFace = SeqOfConstraints.Value(i).myFace;
            CurOrder = SeqOfConstraints.Value(i).myOrder;

            if (CurFace.IsNull)
            {
                if (CurOrder == C0)
                {
                    EdgeCurveAdaptor HCurve = new EdgeCurveAdaptor();
                    HCurve.Initialize(CurEdge);
                    CurveAdaptorBase aHCurve = HCurve; // to avoid ambiguity
                    Constr = new BRepFill_CurveConstraint(aHCurve, (int)CurOrder, myNbPtsOnCur, myTol3d);
                }
                else
                { // Pas de representation Topologique
                  // On prend une representation Geometrique : au pif !
                    Surface? Surface;
                    Curve2D? C2d;
                    Placement loc;
                    double f, l;
                    Tool.CurveOnSurface(CurEdge, out C2d, out Surface, out loc, out f, out l);
                    if (Surface is null)
                    {
                        throw new FailureException("Add");
                    }
                    Surface = (Surface)Surface.Copy();
                    Surface.Transform(loc.Transformation());
                    SurfaceAdaptor Surf = new SurfaceAdaptor(Surface);
                    CurveAdaptor2D Curve2d = new CurveAdaptor2D(C2d!);

                    CurveOnSurface CurvOnSurf = new CurveOnSurface(Curve2d, Surf);
                    CurveOnSurface HCurvOnSurf = CurvOnSurf;

                    Constr = new GeomPlate_CurveConstraint(HCurvOnSurf, (int)CurOrder, myNbPtsOnCur, myTol3d, myTolAng,
                                                           myTolCurv);
                }
            }
            else
            {
                FaceSurfaceAdaptor Surf = new FaceSurfaceAdaptor();
                Surf.Initialize(CurFace);
                EdgeCurve2DAdaptor Curve2d = new EdgeCurve2DAdaptor();
                Curve2d.Initialize(EdgeOnSupport(CurEdge, CurFace), CurFace);
                // If CurEdge has no 2d representation on CurFace,
                // there will be exception "Attempt to access to null object"
                // in this initialization (null pcurve).
                CurveOnSurface CurvOnSurf = new CurveOnSurface(Curve2d, Surf);
                CurveOnSurface HCurvOnSurf = CurvOnSurf;

                Constr = new BRepFill_CurveConstraint(HCurvOnSurf, (int)CurOrder, myNbPtsOnCur, myTol3d, myTolAng,
                                                      myTolCurv);
            }
            if (myIsInitFaceGiven)
            {
                Curve2D? Curve2d;
                double FirstPar, LastPar;
                Curve2d = Tool.CurveOnSurface(CurEdge, myInitFace, out FirstPar, out LastPar);
                if (Curve2d is not null)
                {
                    Curve2d = new TrimmedCurve2D(Curve2d, FirstPar, LastPar);
                    Constr.SetCurve2dOnSurf(Curve2d);
                }
            }
            myBuilder!.Add(Constr);
        }
    }

    private void BuildWires(NCollection_List<Shape> EdgeList, NCollection_List<Shape> WireList)
    {
        NCollection_List<Shape>.Iterator Itl = new NCollection_List<Shape>.Iterator();
        int i, j;

        while (!EdgeList.IsEmpty())
        {
            BRepLib_MakeWire MW = new BRepLib_MakeWire();
            Edge FirstEdge = ShapeCasts.Edge(EdgeList.First());
            MW.Add(FirstEdge);
            var anAdded = new List<(Edge, Edge)> { (FirstEdge, FirstEdge) }; // OCCT.NET: frozen
            EdgeList.RemoveFirst();
            Vertex[] V_wire = new Vertex[2], V_edge = new Vertex[2];

            for (; ; )
            {
                Wire CurWire = MW.InternalWire();
                Exploration.Vertices(CurWire, out V_wire[0], out V_wire[1]);
                bool found = false;
                for (Itl.Initialize(EdgeList); Itl.More(); Itl.Next())
                {
                    Edge CurEdge = ShapeCasts.Edge(Itl.Value());
                    Exploration.Vertices(CurEdge, out V_edge[0], out V_edge[1]);
                    for (i = 0; i < 2; i++)
                    {
                        for (j = 0; j < 2; j++)
                        {
                            if (V_wire[i].IsSame(V_edge[j]))
                            {
                                MW.Add(CurEdge);
                                anAdded.Add((CurEdge, CurEdge));
                                EdgeList.Remove(Itl);
                                found = true;
                                break;
                            }
                        }
                        if (found)
                        {
                            break;
                        }
                    }
                    if (found)
                    {
                        break;
                    }
                }
                if (!found) // try to find geometric coincidence
                {
                    Point3D[] P_wire = new Point3D[2];
                    P_wire[0] = Tool.Pnt(V_wire[0]);
                    P_wire[1] = Tool.Pnt(V_wire[1]);
                    for (Itl.Initialize(EdgeList); Itl.More(); Itl.Next())
                    {
                        Edge CurEdge = ShapeCasts.Edge(Itl.Value());
                        Exploration.Vertices(CurEdge, out V_edge[0], out V_edge[1]);
                        for (i = 0; i < 2; i++)
                        {
                            for (j = 0; j < 2; j++)
                            {
                                double aDist = P_wire[i].Distance(Tool.Pnt(V_edge[j]));
                                if (aDist < Tool.Tolerance(V_wire[i]) && aDist < Tool.Tolerance(V_edge[j]))
                                {
                                    MW.Add(CurEdge);
                                    Edge NewEdge = MW.InternalEdge();
                                    myOldNewMap.Bind(CurEdge.Oriented(Forward), NewEdge.Oriented(Forward));
                                    anAdded.Add((CurEdge, NewEdge));
                                    EdgeList.Remove(Itl);
                                    found = true;
                                    break;
                                }
                            }
                            if (found)
                            {
                                break;
                            }
                        }
                        if (found)
                        {
                            break;
                        }
                    }
                }
                if (!found) // end of current wire, begin next wire
                {
                    WireList.Append(MW.InternalWire());
                    BindCopiesForWrite(MW, anAdded);
                    break;
                }
            } // end of for (;;)
        } // end of while (! EdgeList.IsEmpty())
    }

    // OCCT.NET: frozen. Merging an edge's vertex onto a wire vertex that must move or grow, OCCT's MakeWire changes that
    // vertex in place, so the edges added before it (the boundary's own edges, or the copies bound in myOldNewMap) end at
    // the moved vertex, and the final edges share it. The port's MakeWire goes on with a copy for write of the wire built
    // so far when the vertex is frozen (BRepLib_MakeWire's remarks): each boundary edge is bound here to the edge
    // standing for it in the finished wire, and each of its vertices that was copied is recorded for Generated.
    private void BindCopiesForWrite(BRepLib_MakeWire theMW, List<(Edge, Edge)> theAdded)
    {
        History? aHistory = theMW.InternalHistory();
        if (aHistory is null)
        {
            return;
        }

        static Shape Final(History theHistory, Shape theS)
        {
            for (int aGuard = 0; aGuard < 100; aGuard++)
            {
                ISchematiqList<Shape> aModified = theHistory.Modified(theS);
                if (aModified.Count == 0 || aModified[0].IsSame(theS))
                {
                    break;
                }
                theS = aModified[0].Oriented(theS.Orientation());
            }
            return theS;
        }

        foreach ((Edge anOriginal, Edge anInWire) in theAdded)
        {
            Shape aFinal = Final(aHistory, anInWire.Oriented(Forward));
            if (!aFinal.IsSame(anInWire))
            {
                myOldNewMap.Bind(anOriginal.Oriented(Forward), aFinal);
            }
            for (var anIt = new Iterator(anOriginal.Oriented(Forward)); anIt.More(); anIt.Next())
            {
                Shape aV = anIt.Value().Oriented(Forward);
                Shape aFinalV = Final(aHistory, aV);
                if (!aFinalV.IsSame(aV))
                {
                    myVertexCopies.Bind(aV, aFinalV);
                }
            }
        }
    }

    private void FindExtremitiesOfHoles(NCollection_List<Shape> WireList, NCollection_Sequence<Shape> VerSeq)
    {
        NCollection_Sequence<Shape> WireSeq = new NCollection_Sequence<Shape>();
        NCollection_List<Shape>.Iterator Itl = new NCollection_List<Shape>.Iterator(WireList);
        for (; Itl.More(); Itl.Next())
        {
            WireSeq.Append(Itl.Value());
        }

        Wire theWire;
        theWire = ShapeCasts.Wire(WireSeq.Value(1));
        WireSeq.Remove(1);

        if (Tool.IsClosed(theWire))
        {
            return;
        }

        Vertex Vfirst, Vlast;
        Exploration.Vertices(theWire, out Vfirst, out Vlast);

        if (Vfirst.IsSame(Vlast))
        {
            // The Wire is closed indeed despite its
            // being not detected earlier.

            return;
        }

        Vector3D FinVec = MakeFinVec(theWire, Vlast);
        Vertex theVertex = Vlast;
        VerSeq.Append(Vlast);

        while (!WireSeq.IsEmpty())
        {
            Vertex MinVtx = new Vertex();
            int i, MinInd = 1;
            bool IsLast = false;
            double MinAngle = M_PI;

            for (i = 1; i <= WireSeq.Length(); i++)
            {
                Wire CurWire = ShapeCasts.Wire(WireSeq.Value(i));
                Exploration.Vertices(CurWire, out Vfirst, out Vlast);

                double angle = FinVec.Angle(new Vector3D(Tool.Pnt(theVertex), Tool.Pnt(Vfirst)));
                if (angle < MinAngle)
                {
                    MinAngle = angle;
                    MinVtx = Vfirst;
                    MinInd = i;
                    IsLast = true;
                }
                angle = FinVec.Angle(new Vector3D(Tool.Pnt(theVertex), Tool.Pnt(Vlast)));
                if (angle < MinAngle)
                {
                    MinAngle = angle;
                    MinVtx = Vlast;
                    MinInd = i;
                    IsLast = false;
                }
            }
            VerSeq.Append(MinVtx);
            Wire MinWire = ShapeCasts.Wire(WireSeq.Value(MinInd));
            Exploration.Vertices(MinWire, out Vfirst, out Vlast);
            if (IsLast)
            {
                FinVec = MakeFinVec(MinWire, Vlast);
                theVertex = Vlast;
            }
            else
            {
                FinVec = MakeFinVec(MinWire, Vfirst);
                theVertex = Vfirst;
            }
            VerSeq.Append(theVertex);
            WireSeq.Remove(MinInd);
        }
        Exploration.Vertices(theWire, out Vfirst, out Vlast);
        VerSeq.Append(Vfirst);
    }

    /// <summary>Builds the filling.</summary>
    public void Build()
    {
        myBuilder = new GeomPlate_BuildPlateSurface(myDegree, myNbPtsOnCur, myNbIter, myTol2d, myTol3d, myTolAng, myTolCurv,
                                                    myAnisotropie);

        if (myBoundary.IsEmpty())
        {
            myIsDone = false;
            return;
        }

        Face CurFace;
        int i, j;

        // Creating array of vertices: extremities of wires
        NCollection_Sequence<Shape> VerSeq = new NCollection_Sequence<Shape>();

        // Building missing bounds
        NCollection_List<Shape> EdgeList = new NCollection_List<Shape>(),
                                       WireList = new NCollection_List<Shape>();
        for (i = 1; i <= myBoundary.Length(); i++)
        {
            EdgeList.Append(myBoundary.Value(i).myEdge);
        }

        BuildWires(EdgeList, WireList);
        FindExtremitiesOfHoles(WireList, VerSeq);

        // Searching for surfaces for missing bounds
        for (j = 1; j <= myFreeConstraints.Length(); j++)
        {
            GeomAPI_ProjectPointOnSurf Projector = new GeomAPI_ProjectPointOnSurf();
            double U1, V1, U2, V2;

            CurFace = myFreeConstraints.Value(j).myFace;
            FaceSurfaceAdaptor HSurf = new FaceSurfaceAdaptor();
            HSurf.Initialize(CurFace);
            Surface CurSurface = Tool.Surface(HSurf.Face())!;

            for (i = 1; i <= VerSeq.Length(); i += 2)
            {
                Vertex FirstVtx = ShapeCasts.Vertex(VerSeq.Value(i));
                Vertex LastVtx = ShapeCasts.Vertex(VerSeq.Value(i + 1));

                Point3D FirstPnt = Tool.Pnt(FirstVtx);
                Projector.Init(FirstPnt, CurSurface);
                if (Projector.LowerDistance() > global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    continue;
                }
                Projector.LowerDistanceParameters(out U1, out V1);

                Point3D LastPnt = Tool.Pnt(LastVtx);
                Projector.Init(LastPnt, CurSurface);
                if (Projector.LowerDistance() > global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    continue;
                }
                Projector.LowerDistanceParameters(out U2, out V2);

                // Making the constraint
                NCollection_Array1<Point2D> Points = new NCollection_Array1<Point2D>(1, 2);
                Points[1] = new Point2D(U1, V1);
                Points[2] = new Point2D(U2, V2);
                BezierCurve2D Line2d = new BezierCurve2D(Points);
                Edge E = new BRepLib_MakeEdge(Line2d, CurSurface, FirstVtx, LastVtx).InternalEdge();
                Add(E, CurFace, myFreeConstraints.Value(j).myOrder);
                VerSeq.Remove(i, i + 1);
                break;
            } // for (i = 1; i <= VerSeq.Length(); i += 2)
        } // for (j = 1; j <= myFreeConstraints.Length(); j++)

        // Load initial surface to myBuilder if it is given
        if (myIsInitFaceGiven)
        {
            FaceSurfaceAdaptor HSurfInit = new FaceSurfaceAdaptor();
            HSurfInit.Initialize(myInitFace);
            myBuilder.LoadInitSurface(Tool.Surface(HSurfInit.Face())!);
        }

        // Adding constraints to myBuilder
        AddConstraints(myBoundary);
        myBuilder.SetNbBounds(myBoundary.Length());
        AddConstraints(myConstraints);
        for (i = 1; i <= myPoints.Length(); i++)
        {
            myBuilder.Add(myPoints.Value(i));
        }

        myBuilder.Perform();
        if (myBuilder.IsDone())
        {
            myIsDone = true;
        }
        else
        {
            myIsDone = false;
            return;
        }

        GeomPlate_Surface GPlate = myBuilder.Surface()!;
        BSplineSurface Surface;
        // Approximation
        double dmax = 1.1 * myBuilder.G0Error(); //???????????
        if (!myIsInitFaceGiven)
        {
            double seuil; //?????

            NCollection_Sequence<Xy> S2d = new NCollection_Sequence<Xy>();
            NCollection_Sequence<Xyz> S3d = new NCollection_Sequence<Xyz>();
            myBuilder.Disc2dContour(4, S2d);
            myBuilder.Disc3dContour(4, 0, S3d);
            seuil = Max(myTol3d, 10 * myBuilder.G0Error()); //????????
            GeomPlate_PlateG0Criterion Criterion = new GeomPlate_PlateG0Criterion(S2d, S3d, seuil);
            GeomPlate_MakeApprox Approx = new GeomPlate_MakeApprox(GPlate, Criterion, myTol3d, myMaxSegments, myMaxDeg);
            Surface = Approx.Surface()!;
        }
        else
        {
            GeomPlate_MakeApprox Approx = new GeomPlate_MakeApprox(GPlate, myTol3d, myMaxSegments, myMaxDeg, dmax, 0); //?????????????
            Surface = Approx.Surface()!;
        }

        // Build the final wire and final face
        NCollection_List<Shape> FinalEdges = new NCollection_List<Shape>();
        NCollection_HArray1<Curve2D?> CurvesOnPlate = myBuilder.Curves2d();
        BrepBuilder BB = new BrepBuilder();
        for (i = 1; i <= myBoundary.Length(); i++)
        {
            Edge InitEdge = myBoundary.Value(i).myEdge;
            Edge anEdge = InitEdge;
            anEdge.Orientation(Forward);
            if (myOldNewMap.IsBound(anEdge))
            {
                anEdge = ShapeCasts.Edge(myOldNewMap.Find(anEdge));
            }

            Curve2D? aCurveOnPlate = CurvesOnPlate.Value(i);

            Edge NewEdge = ShapeCasts.Edge(anEdge.EmptyCopied());

            Vertex V1, V2, NewV1, NewV2;
            Exploration.Vertices(anEdge, out V1, out V2);

            if (myOldNewMap.IsBound(V1))
            {
                NewV1 = ShapeCasts.Vertex(myOldNewMap.Find(V1));
            }
            else
            {
                Point3D aPnt = Tool.Pnt(V1);
                NewV1 = new BRepLib_MakeVertex(aPnt).InternalVertex();
                BB.UpdateVertex(NewV1, dmax);
                myOldNewMap.Bind(V1.Oriented(Forward), NewV1);
            }

            if (myOldNewMap.IsBound(V2))
            {
                NewV2 = ShapeCasts.Vertex(myOldNewMap.Find(V2));
            }
            else
            {
                Point3D aPnt = Tool.Pnt(V2);
                NewV2 = new BRepLib_MakeVertex(aPnt).InternalVertex();
                BB.UpdateVertex(NewV2, dmax);
                myOldNewMap.Bind(V2.Oriented(Forward), NewV2);
            }

            NewV1.Orientation(Forward);
            BB.Add(NewEdge, NewV1);
            NewV2.Orientation(Reversed);
            BB.Add(NewEdge, NewV2);
            Placement Loc = new Placement();
            BB.UpdateEdge(NewEdge, aCurveOnPlate, Surface, Loc, dmax);
            BRepLib.SameParameter(NewEdge, dmax, true);
            FinalEdges.Append(NewEdge);
            myOldNewMap.Bind(InitEdge.Oriented(Forward), NewEdge.Oriented(Forward));
        }

        Wire FinalWire = WireFromList(FinalEdges);

        myFace = new BRepLib_MakeFace(Surface, FinalWire).InternalFace();
    }

    public bool IsDone() => myIsDone;

    /// <summary>The filling face (frozen; a null face if the filling wasn't built).</summary>
    public Face Face()
    {
        myFace.TShape()?.FreezeGraph();
        return myFace;
    }

    /// <summary>The filling face, not frozen, for ported code (BRepOffsetAPI_MakeFilling).</summary>
    internal Face InternalFace() => myFace;

    /// <summary>
    /// The new edge or vertex made from the boundary edge or vertex <paramref name="S"/> (a list of one shape, or
    /// empty; frozen, with the face).
    /// </summary>
    public ISchematiqList<Shape> Generated(in Shape S)
    {
        NCollection_List<Shape> aList = InternalGenerated(S);
        myFace.TShape()?.FreezeGraph();
        if (aList.IsEmpty())
        {
            return PackedList<Shape>.Empty;
        }
        Shape[] aShapes = new Shape[aList.Extent()];
        int i = 0;
        for (NCollection_List<Shape>.Iterator anIt = new(aList); anIt.More(); anIt.Next())
        {
            aShapes[i] = anIt.Value();
            aShapes[i].TShape()?.FreezeGraph();
            i++;
        }
        return PackedList.Own(aShapes);
    }

    /// <summary><see cref="Generated"/>: the filling's own list, unfrozen (OCCT's reference), for ported code.</summary>
    internal NCollection_List<Shape> InternalGenerated(in Shape S)
    {
        myGenerated.Clear();

        if (myOldNewMap.IsBound(S))
        {
            myGenerated.Append(myOldNewMap.Find(S));
        }
        // OCCT.NET: frozen. A boundary vertex BuildWires' MakeWire copied (OCCT moves it in place): its copy's
        else if (myVertexCopies.Find(S, out Shape aCopy) && myOldNewMap.IsBound(aCopy))
        {
            myGenerated.Append(myOldNewMap.Find(aCopy));
        }

        return myGenerated;
    }

    /// <summary>The plate builder of the last <see cref="Build"/>, for ported code and tests.</summary>
    internal GeomPlate_BuildPlateSurface? InternalBuilder() => myBuilder;

    /// <summary>The largest distance from the boundary to the surface.</summary>
    public double G0Error() => myBuilder!.G0Error();

    /// <summary>The largest angle between the surface and the support faces at the boundary.</summary>
    public double G1Error() => myBuilder!.G1Error();

    /// <summary>The largest curvature gap between the surface and the support faces at the boundary.</summary>
    public double G2Error() => myBuilder!.G2Error();

    /// <summary>The largest distance between the constraint <paramref name="Index"/> and the surface.</summary>
    public double G0Error(int Index) => myBuilder!.G0Error(Index);

    /// <summary>The largest angle between the constraint <paramref name="Index"/> and the surface.</summary>
    public double G1Error(int Index) => myBuilder!.G1Error(Index);

    /// <summary>The largest curvature gap between the constraint <paramref name="Index"/> and the surface.</summary>
    public double G2Error(int Index) => myBuilder!.G2Error(Index);
}
