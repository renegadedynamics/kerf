// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Filling.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Filling.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Kerf.Topology;
using Schematiq.Collections;
using static Kerf.Geometry.Continuity;
using static Occt.TopAbs_Orientation;

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
    private readonly NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> myOldNewMap = new(TopTools_ShapeMapHasher.Instance);
    // OCCT.NET: frozen. The boundary's vertices BuildWires' MakeWire replaced by its copies for write (see BuildWires).
    private readonly NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> myVertexCopies = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_List<TopoDS_Shape> myGenerated = new();
    private TopoDS_Face myFace = new();
    private TopoDS_Face myInitFace = new();
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

    private static Vector3D MakeFinVec(in TopoDS_Wire aWire, in TopoDS_Vertex aVertex)
    {
        TopoDS_Vertex Vfirst, Vlast, Origin = new TopoDS_Vertex();
        BRepTools_WireExplorer Explo = new BRepTools_WireExplorer(aWire);
        for (; Explo.More(); Explo.Next())
        {
            TopExp.Vertices(Explo.Current(), out Vfirst, out Vlast);
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
        return new Vector3D(BRep_Tool.Pnt(Origin), BRep_Tool.Pnt(aVertex));
    }

    private static TopoDS_Wire WireFromList(NCollection_List<TopoDS_Shape> Edges)
    {
        BRep_Builder BB = new BRep_Builder();
        TopoDS_Wire aWire = new TopoDS_Wire();
        BB.MakeWire(ref aWire);
        TopoDS_Edge anEdge = TopoDS.Edge(Edges.First());
        BB.Add(aWire, anEdge);
        Edges.RemoveFirst();

        TopoDS_Vertex V1, V2;
        TopExp.Vertices(anEdge, out V1, out V2, true); // with orientation

        while (!Edges.IsEmpty())
        {
            NCollection_List<TopoDS_Shape>.Iterator itl = new NCollection_List<TopoDS_Shape>.Iterator(Edges);
            for (; itl.More(); itl.Next())
            {
                anEdge = TopoDS.Edge(itl.Value());
                TopoDS_Vertex V3, V4;
                TopExp.Vertices(anEdge, out V3, out V4, true); // with orientation
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
    public void LoadInitSurface(in TopoDS_Face aFace)
    {
        myInitFace = aFace;
        myIsInitFaceGiven = true;
    }

    /// <summary>Adds a boundary edge (or, with <paramref name="IsBound"/> false, a free edge) of order G0 (no face).</summary>
    /// <returns>The index of the constraint.</returns>
    public int Add(in TopoDS_Edge anEdge, Continuity Order, bool IsBound = true)
    {
        TopoDS_Face NullFace = new TopoDS_Face();
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
    public int Add(in TopoDS_Edge anEdge, in TopoDS_Face Support, Continuity Order, bool IsBound = true)
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
    public int Add(in TopoDS_Face Support, Continuity Order)
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
    public int Add(double U, double V, in TopoDS_Face Support, Continuity Order)
    {
        BRepAdaptor_Surface HSurf = new BRepAdaptor_Surface();
        HSurf.Initialize(Support);
        GeomPlate_PointConstraint aPC = new GeomPlate_PointConstraint(U, V, BRep_Tool.Surface(HSurf.Face())!, (int)Order,
                                                                      myTol3d, myTolAng, myTolCurv);
        myPoints.Append(aPC);
        return (myBoundary.Length() + myFreeConstraints.Length() + myConstraints.Length() + myPoints.Length());
    }

    // OCCT.NET: OCCT's sweeps of an edge (MakePrism, MakeRevol, ...) write the new face's pcurve into the caller's edge,
    // which the face shares; the port sweeps a frozen edge as a copy (decision 468), so the caller's edge has no pcurve
    // on the face the sweep made, and the constraint would read a null pcurve (OCCT's comment below). For a frozen edge
    // without a pcurve on the support, the support's edge on the same 3D curve, location and range stands for it, as it
    // is OCCT's edge itself (decision 1103).
    private static TopoDS_Edge EdgeOnSupport(in TopoDS_Edge theEdge, in TopoDS_Face theSupport)
    {
        if (!theEdge.TShape()!.IsFrozen || BRep_Tool.CurveOnSurface(theEdge, theSupport, out _, out _) is not null)
        {
            return theEdge;
        }
        Geom_Curve? aCurve = BRep_Tool.Curve(theEdge, out Placement aLoc, out double aFirst, out double aLast);
        if (aCurve is null)
        {
            return theEdge;
        }
        for (TopExp_Explorer anExp = new(theSupport, TopAbs_ShapeEnum.TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            TopoDS_Edge anEdge = TopoDS.Edge(anExp.Current());
            Geom_Curve? aC = BRep_Tool.Curve(anEdge, out Placement aL, out double aF, out double aLa);
            if (ReferenceEquals(aC, aCurve) && aL == aLoc && aF == aFirst && aLa == aLast)
            {
                return TopoDS.Edge(anEdge.Oriented(theEdge.Orientation()));
            }
        }
        return theEdge;
    }

    private void AddConstraints(NCollection_Sequence<BRepFill_EdgeFaceAndOrder> SeqOfConstraints)
    {
        TopoDS_Edge CurEdge;
        TopoDS_Face CurFace;
        Continuity CurOrder;

        GeomPlate_CurveConstraint Constr;
        int i;
        for (i = 1; i <= SeqOfConstraints.Length(); i++)
        {
            CurEdge = SeqOfConstraints.Value(i).myEdge;
            CurFace = SeqOfConstraints.Value(i).myFace;
            CurOrder = SeqOfConstraints.Value(i).myOrder;

            if (CurFace.IsNull())
            {
                if (CurOrder == C0)
                {
                    BRepAdaptor_Curve HCurve = new BRepAdaptor_Curve();
                    HCurve.Initialize(CurEdge);
                    Adaptor3d_Curve aHCurve = HCurve; // to avoid ambiguity
                    Constr = new BRepFill_CurveConstraint(aHCurve, (int)CurOrder, myNbPtsOnCur, myTol3d);
                }
                else
                { // Pas de representation Topologique
                  // On prend une representation Geometrique : au pif !
                    Geom_Surface? Surface;
                    Geom2d_Curve? C2d;
                    Placement loc;
                    double f, l;
                    BRep_Tool.CurveOnSurface(CurEdge, out C2d, out Surface, out loc, out f, out l);
                    if (Surface is null)
                    {
                        throw new FailureException("Add");
                    }
                    Surface = (Geom_Surface)Surface.Copy();
                    Surface.Transform(loc.Transformation());
                    GeomAdaptor_Surface Surf = new GeomAdaptor_Surface(Surface);
                    Geom2dAdaptor_Curve Curve2d = new Geom2dAdaptor_Curve(C2d!);

                    Adaptor3d_CurveOnSurface CurvOnSurf = new Adaptor3d_CurveOnSurface(Curve2d, Surf);
                    Adaptor3d_CurveOnSurface HCurvOnSurf = CurvOnSurf;

                    Constr = new GeomPlate_CurveConstraint(HCurvOnSurf, (int)CurOrder, myNbPtsOnCur, myTol3d, myTolAng,
                                                           myTolCurv);
                }
            }
            else
            {
                BRepAdaptor_Surface Surf = new BRepAdaptor_Surface();
                Surf.Initialize(CurFace);
                BRepAdaptor_Curve2d Curve2d = new BRepAdaptor_Curve2d();
                Curve2d.Initialize(EdgeOnSupport(CurEdge, CurFace), CurFace);
                // If CurEdge has no 2d representation on CurFace,
                // there will be exception "Attempt to access to null object"
                // in this initialization (null pcurve).
                Adaptor3d_CurveOnSurface CurvOnSurf = new Adaptor3d_CurveOnSurface(Curve2d, Surf);
                Adaptor3d_CurveOnSurface HCurvOnSurf = CurvOnSurf;

                Constr = new BRepFill_CurveConstraint(HCurvOnSurf, (int)CurOrder, myNbPtsOnCur, myTol3d, myTolAng,
                                                      myTolCurv);
            }
            if (myIsInitFaceGiven)
            {
                Geom2d_Curve? Curve2d;
                double FirstPar, LastPar;
                Curve2d = BRep_Tool.CurveOnSurface(CurEdge, myInitFace, out FirstPar, out LastPar);
                if (Curve2d is not null)
                {
                    Curve2d = new Geom2d_TrimmedCurve(Curve2d, FirstPar, LastPar);
                    Constr.SetCurve2dOnSurf(Curve2d);
                }
            }
            myBuilder!.Add(Constr);
        }
    }

    private void BuildWires(NCollection_List<TopoDS_Shape> EdgeList, NCollection_List<TopoDS_Shape> WireList)
    {
        NCollection_List<TopoDS_Shape>.Iterator Itl = new NCollection_List<TopoDS_Shape>.Iterator();
        int i, j;

        while (!EdgeList.IsEmpty())
        {
            BRepLib_MakeWire MW = new BRepLib_MakeWire();
            TopoDS_Edge FirstEdge = TopoDS.Edge(EdgeList.First());
            MW.Add(FirstEdge);
            var anAdded = new List<(TopoDS_Edge, TopoDS_Edge)> { (FirstEdge, FirstEdge) }; // OCCT.NET: frozen
            EdgeList.RemoveFirst();
            TopoDS_Vertex[] V_wire = new TopoDS_Vertex[2], V_edge = new TopoDS_Vertex[2];

            for (; ; )
            {
                TopoDS_Wire CurWire = MW.InternalWire();
                TopExp.Vertices(CurWire, out V_wire[0], out V_wire[1]);
                bool found = false;
                for (Itl.Initialize(EdgeList); Itl.More(); Itl.Next())
                {
                    TopoDS_Edge CurEdge = TopoDS.Edge(Itl.Value());
                    TopExp.Vertices(CurEdge, out V_edge[0], out V_edge[1]);
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
                    P_wire[0] = BRep_Tool.Pnt(V_wire[0]);
                    P_wire[1] = BRep_Tool.Pnt(V_wire[1]);
                    for (Itl.Initialize(EdgeList); Itl.More(); Itl.Next())
                    {
                        TopoDS_Edge CurEdge = TopoDS.Edge(Itl.Value());
                        TopExp.Vertices(CurEdge, out V_edge[0], out V_edge[1]);
                        for (i = 0; i < 2; i++)
                        {
                            for (j = 0; j < 2; j++)
                            {
                                double aDist = P_wire[i].Distance(BRep_Tool.Pnt(V_edge[j]));
                                if (aDist < BRep_Tool.Tolerance(V_wire[i]) && aDist < BRep_Tool.Tolerance(V_edge[j]))
                                {
                                    MW.Add(CurEdge);
                                    TopoDS_Edge NewEdge = MW.InternalEdge();
                                    myOldNewMap.Bind(CurEdge.Oriented(TopAbs_FORWARD), NewEdge.Oriented(TopAbs_FORWARD));
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
    private void BindCopiesForWrite(BRepLib_MakeWire theMW, List<(TopoDS_Edge, TopoDS_Edge)> theAdded)
    {
        BRepTools_History? aHistory = theMW.InternalHistory();
        if (aHistory is null)
        {
            return;
        }

        static TopoDS_Shape Final(BRepTools_History theHistory, TopoDS_Shape theS)
        {
            for (int aGuard = 0; aGuard < 100; aGuard++)
            {
                ISchematiqList<TopoDS_Shape> aModified = theHistory.Modified(theS);
                if (aModified.Count == 0 || aModified[0].IsSame(theS))
                {
                    break;
                }
                theS = aModified[0].Oriented(theS.Orientation());
            }
            return theS;
        }

        foreach ((TopoDS_Edge anOriginal, TopoDS_Edge anInWire) in theAdded)
        {
            TopoDS_Shape aFinal = Final(aHistory, anInWire.Oriented(TopAbs_FORWARD));
            if (!aFinal.IsSame(anInWire))
            {
                myOldNewMap.Bind(anOriginal.Oriented(TopAbs_FORWARD), aFinal);
            }
            for (var anIt = new TopoDS_Iterator(anOriginal.Oriented(TopAbs_FORWARD)); anIt.More(); anIt.Next())
            {
                TopoDS_Shape aV = anIt.Value().Oriented(TopAbs_FORWARD);
                TopoDS_Shape aFinalV = Final(aHistory, aV);
                if (!aFinalV.IsSame(aV))
                {
                    myVertexCopies.Bind(aV, aFinalV);
                }
            }
        }
    }

    private void FindExtremitiesOfHoles(NCollection_List<TopoDS_Shape> WireList, NCollection_Sequence<TopoDS_Shape> VerSeq)
    {
        NCollection_Sequence<TopoDS_Shape> WireSeq = new NCollection_Sequence<TopoDS_Shape>();
        NCollection_List<TopoDS_Shape>.Iterator Itl = new NCollection_List<TopoDS_Shape>.Iterator(WireList);
        for (; Itl.More(); Itl.Next())
        {
            WireSeq.Append(Itl.Value());
        }

        TopoDS_Wire theWire;
        theWire = TopoDS.Wire(WireSeq.Value(1));
        WireSeq.Remove(1);

        if (BRep_Tool.IsClosed(theWire))
        {
            return;
        }

        TopoDS_Vertex Vfirst, Vlast;
        TopExp.Vertices(theWire, out Vfirst, out Vlast);

        if (Vfirst.IsSame(Vlast))
        {
            // The Wire is closed indeed despite its
            // being not detected earlier.

            return;
        }

        Vector3D FinVec = MakeFinVec(theWire, Vlast);
        TopoDS_Vertex theVertex = Vlast;
        VerSeq.Append(Vlast);

        while (!WireSeq.IsEmpty())
        {
            TopoDS_Vertex MinVtx = new TopoDS_Vertex();
            int i, MinInd = 1;
            bool IsLast = false;
            double MinAngle = M_PI;

            for (i = 1; i <= WireSeq.Length(); i++)
            {
                TopoDS_Wire CurWire = TopoDS.Wire(WireSeq.Value(i));
                TopExp.Vertices(CurWire, out Vfirst, out Vlast);

                double angle = FinVec.Angle(new Vector3D(BRep_Tool.Pnt(theVertex), BRep_Tool.Pnt(Vfirst)));
                if (angle < MinAngle)
                {
                    MinAngle = angle;
                    MinVtx = Vfirst;
                    MinInd = i;
                    IsLast = true;
                }
                angle = FinVec.Angle(new Vector3D(BRep_Tool.Pnt(theVertex), BRep_Tool.Pnt(Vlast)));
                if (angle < MinAngle)
                {
                    MinAngle = angle;
                    MinVtx = Vlast;
                    MinInd = i;
                    IsLast = false;
                }
            }
            VerSeq.Append(MinVtx);
            TopoDS_Wire MinWire = TopoDS.Wire(WireSeq.Value(MinInd));
            TopExp.Vertices(MinWire, out Vfirst, out Vlast);
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
        TopExp.Vertices(theWire, out Vfirst, out Vlast);
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

        TopoDS_Face CurFace;
        int i, j;

        // Creating array of vertices: extremities of wires
        NCollection_Sequence<TopoDS_Shape> VerSeq = new NCollection_Sequence<TopoDS_Shape>();

        // Building missing bounds
        NCollection_List<TopoDS_Shape> EdgeList = new NCollection_List<TopoDS_Shape>(),
                                       WireList = new NCollection_List<TopoDS_Shape>();
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
            BRepAdaptor_Surface HSurf = new BRepAdaptor_Surface();
            HSurf.Initialize(CurFace);
            Geom_Surface CurSurface = BRep_Tool.Surface(HSurf.Face())!;

            for (i = 1; i <= VerSeq.Length(); i += 2)
            {
                TopoDS_Vertex FirstVtx = TopoDS.Vertex(VerSeq.Value(i));
                TopoDS_Vertex LastVtx = TopoDS.Vertex(VerSeq.Value(i + 1));

                Point3D FirstPnt = BRep_Tool.Pnt(FirstVtx);
                Projector.Init(FirstPnt, CurSurface);
                if (Projector.LowerDistance() > global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    continue;
                }
                Projector.LowerDistanceParameters(out U1, out V1);

                Point3D LastPnt = BRep_Tool.Pnt(LastVtx);
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
                Geom2d_BezierCurve Line2d = new Geom2d_BezierCurve(Points);
                TopoDS_Edge E = new BRepLib_MakeEdge(Line2d, CurSurface, FirstVtx, LastVtx).InternalEdge();
                Add(E, CurFace, myFreeConstraints.Value(j).myOrder);
                VerSeq.Remove(i, i + 1);
                break;
            } // for (i = 1; i <= VerSeq.Length(); i += 2)
        } // for (j = 1; j <= myFreeConstraints.Length(); j++)

        // Load initial surface to myBuilder if it is given
        if (myIsInitFaceGiven)
        {
            BRepAdaptor_Surface HSurfInit = new BRepAdaptor_Surface();
            HSurfInit.Initialize(myInitFace);
            myBuilder.LoadInitSurface(BRep_Tool.Surface(HSurfInit.Face())!);
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
        Geom_BSplineSurface Surface;
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
        NCollection_List<TopoDS_Shape> FinalEdges = new NCollection_List<TopoDS_Shape>();
        NCollection_HArray1<Geom2d_Curve?> CurvesOnPlate = myBuilder.Curves2d();
        BRep_Builder BB = new BRep_Builder();
        for (i = 1; i <= myBoundary.Length(); i++)
        {
            TopoDS_Edge InitEdge = myBoundary.Value(i).myEdge;
            TopoDS_Edge anEdge = InitEdge;
            anEdge.Orientation(TopAbs_FORWARD);
            if (myOldNewMap.IsBound(anEdge))
            {
                anEdge = TopoDS.Edge(myOldNewMap.Find(anEdge));
            }

            Geom2d_Curve? aCurveOnPlate = CurvesOnPlate.Value(i);

            TopoDS_Edge NewEdge = TopoDS.Edge(anEdge.EmptyCopied());

            TopoDS_Vertex V1, V2, NewV1, NewV2;
            TopExp.Vertices(anEdge, out V1, out V2);

            if (myOldNewMap.IsBound(V1))
            {
                NewV1 = TopoDS.Vertex(myOldNewMap.Find(V1));
            }
            else
            {
                Point3D aPnt = BRep_Tool.Pnt(V1);
                NewV1 = new BRepLib_MakeVertex(aPnt).InternalVertex();
                BB.UpdateVertex(NewV1, dmax);
                myOldNewMap.Bind(V1.Oriented(TopAbs_FORWARD), NewV1);
            }

            if (myOldNewMap.IsBound(V2))
            {
                NewV2 = TopoDS.Vertex(myOldNewMap.Find(V2));
            }
            else
            {
                Point3D aPnt = BRep_Tool.Pnt(V2);
                NewV2 = new BRepLib_MakeVertex(aPnt).InternalVertex();
                BB.UpdateVertex(NewV2, dmax);
                myOldNewMap.Bind(V2.Oriented(TopAbs_FORWARD), NewV2);
            }

            NewV1.Orientation(TopAbs_FORWARD);
            BB.Add(NewEdge, NewV1);
            NewV2.Orientation(TopAbs_REVERSED);
            BB.Add(NewEdge, NewV2);
            Placement Loc = new Placement();
            BB.UpdateEdge(NewEdge, aCurveOnPlate, Surface, Loc, dmax);
            BRepLib.SameParameter(NewEdge, dmax, true);
            FinalEdges.Append(NewEdge);
            myOldNewMap.Bind(InitEdge.Oriented(TopAbs_FORWARD), NewEdge.Oriented(TopAbs_FORWARD));
        }

        TopoDS_Wire FinalWire = WireFromList(FinalEdges);

        myFace = new BRepLib_MakeFace(Surface, FinalWire).InternalFace();
    }

    public bool IsDone() => myIsDone;

    /// <summary>The filling face (frozen; a null face if the filling wasn't built).</summary>
    public TopoDS_Face Face()
    {
        myFace.TShape()?.FreezeGraph();
        return myFace;
    }

    /// <summary>The filling face, not frozen, for ported code (BRepOffsetAPI_MakeFilling).</summary>
    internal TopoDS_Face InternalFace() => myFace;

    /// <summary>
    /// The new edge or vertex made from the boundary edge or vertex <paramref name="S"/> (a list of one shape, or
    /// empty; frozen, with the face).
    /// </summary>
    public ISchematiqList<TopoDS_Shape> Generated(in TopoDS_Shape S)
    {
        NCollection_List<TopoDS_Shape> aList = InternalGenerated(S);
        myFace.TShape()?.FreezeGraph();
        if (aList.IsEmpty())
        {
            return PackedList<TopoDS_Shape>.Empty;
        }
        TopoDS_Shape[] aShapes = new TopoDS_Shape[aList.Extent()];
        int i = 0;
        for (NCollection_List<TopoDS_Shape>.Iterator anIt = new(aList); anIt.More(); anIt.Next())
        {
            aShapes[i] = anIt.Value();
            aShapes[i].TShape()?.FreezeGraph();
            i++;
        }
        return PackedList.Own(aShapes);
    }

    /// <summary><see cref="Generated"/>: the filling's own list, unfrozen (OCCT's reference), for ported code.</summary>
    internal NCollection_List<TopoDS_Shape> InternalGenerated(in TopoDS_Shape S)
    {
        myGenerated.Clear();

        if (myOldNewMap.IsBound(S))
        {
            myGenerated.Append(myOldNewMap.Find(S));
        }
        // OCCT.NET: frozen. A boundary vertex BuildWires' MakeWire copied (OCCT moves it in place): its copy's
        else if (myVertexCopies.Find(S, out TopoDS_Shape aCopy) && myOldNewMap.IsBound(aCopy))
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
