// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck_Face.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck_Face.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.BRepCheck_Status;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Occt;

/// <summary>
/// The checks of a face: it has a surface (<see cref="Minimum"/>); its wires don't intersect
/// (<see cref="IntersectWires"/>), one of them contains all the others (<see cref="ClassifyWires"/>), and they are
/// oriented accordingly (<see cref="OrientationOfWires"/>).
/// </summary>
/// <remarks>
/// <see cref="ClassifyWires"/> classifies each wire in a face of its own, an unfrozen copy of the face's TShape
/// sharing its surface, built for the call and never handed out; the checked face is only read.
/// </remarks>
public class BRepCheck_Face : BRepCheck_Result
{
    private bool myIntdone;
    private BRepCheck_Status myIntres;
    private bool myImbdone;
    private BRepCheck_Status myImbres;
    private bool myOridone;
    private BRepCheck_Status myOrires;

    private readonly NCollection_DataMap<Shape, NCollection_List<Shape>> myMapImb =
        new(ShapeHasher.Instance);

    private bool myGctrl;

    /// <summary>Checks the face on its own (<see cref="Minimum"/>).</summary>
    public BRepCheck_Face(in Face F)
    {
        myIntres = BRepCheck_NoError;
        myImbres = BRepCheck_NoError;
        myOrires = BRepCheck_NoError;
        Init(F);
        myIntdone = false;
        myImbdone = false;
        myOridone = false;
        myGctrl = true;
    }

    // OCCT's DataMapOfShapeBox2d: NCollection_DataMap<TopoDS_Shape, Bnd_Box2d> with the default hasher, which
    // compares shapes with operator== (IsEqual: the orientation counts)
    private static NCollection_DataMap<Shape, Box2d> NewDataMapOfShapeBox2d()
        => new(EqualityComparer<Shape>.Default);

    /// <summary>The status list of the face itself, read under the lock in parallel mode.</summary>
    private NCollection_List<BRepCheck_Status> lockedStatusList()
    {
        bool aLocked = myIsParallel;
        if (aLocked)
        {
            myMutex.Enter();
        }

        try
        {
            return myMap.Find(myShape);
        }
        finally
        {
            if (aLocked)
            {
                myMutex.Exit();
            }
        }
    }

    /// <summary>Checks that the face has a surface.</summary>
    public override void Minimum()
    {
        if (!myMin)
        {
            var aNewList = new NCollection_List<BRepCheck_Status>();
            NCollection_List<BRepCheck_Status> lst = myMap.Bound(myShape, aNewList);

            var TF = (BrepTFace)myShape.TShape()!;
            if (TF.Surface() is null)
            {
                BRepCheck.Add(lst, BRepCheck_NoSurface);
            }
            else
            {
                // Flag natural restriction???
            }

            if (lst.IsEmpty())
            {
                lst.Append(BRepCheck_NoError);
            }

            myMin = true;
        }
    }

    /// <summary>Checks only that the face is in <paramref name="S"/>.</summary>
    public override void InContext(in Shape S)
    {
        NCollection_List<BRepCheck_Status> aHList;
        {
            bool aLocked = myIsParallel;
            if (aLocked)
            {
                myMutex.Enter();
            }

            try
            {
                if (myMap.IsBound(S))
                {
                    return;
                }

                var aNewList = new NCollection_List<BRepCheck_Status>();
                aHList = myMap.Bound(S, aNewList);
            }
            finally
            {
                if (aLocked)
                {
                    myMutex.Exit();
                }
            }
        }

        NCollection_List<BRepCheck_Status> lst = aHList;

        var exp = new Explorer(S, ShapeType.Face);
        for (; exp.More(); exp.Next())
        {
            if (exp.Current().IsSame(myShape))
            {
                break;
            }
        }

        if (!exp.More())
        {
            BRepCheck.Add(lst, BRepCheck_SubshapeNotInShape);
            return;
        }

        if (lst.IsEmpty())
        {
            lst.Append(BRepCheck_NoError);
        }
    }

    /// <summary>Nothing more than the minimum.</summary>
    public override void Blind()
    {
        if (!myBlind)
        {
            // nothing more than in the minimum
            myBlind = true;
        }
    }

    /// <summary>
    /// Checks that no wire is met twice and that no two wires intersect in the parametric space of the face, except
    /// at common vertices. With <paramref name="Update"/>, the status is added to the face's statuses.
    /// </summary>
    public BRepCheck_Status IntersectWires(bool Update = false)
    {
        NCollection_List<BRepCheck_Status> aStatusList = lockedStatusList();
        if (myIntdone)
        {
            if (Update)
            {
                BRepCheck.Add(aStatusList, myIntres);
            }

            return myIntres;
        }

        myIntdone = true;
        myIntres = BRepCheck_NoError;
        // This method has to be called by an analyzer. It is assumed that
        // each edge has a correct 2d representation on the face.

        var exp1 = new Explorer();
        var exp2 = new Explorer();

        // the wires are mapped
        exp1.Init(myShape.Oriented(Forward), ShapeType.Wire);
        while (exp1.More())
        {
            if (!myMapImb.IsBound(exp1.Current()))
            {
                // OCCT.NET: OCCT binds a copy of one empty list; each wire gets its own
                myMapImb.Bind(exp1.Current(), new NCollection_List<Shape>());
            }
            else
            {
                // the same wire is met twice...
                myIntres = BRepCheck_RedundantWire;
                if (Update)
                {
                    BRepCheck.Add(aStatusList, myIntres);
                }

                return myIntres;
            }

            exp1.Next();
        }

        var aC = new CurveAdaptor2D();
        double aFirst, aLast;
        NCollection_DataMap<Shape, Box2d> aMapShapeBox2d = NewDataMapOfShapeBox2d();
        for (exp1.Init(myShape, ShapeType.Wire); exp1.More(); exp1.Next())
        {
            Wire aWire = ShapeCasts.Wire(exp1.Current());
            // create 2d boxes for all edges from wire
            var aBoxW = new Box2d();
            for (exp2.Init(aWire, ShapeType.Edge); exp2.More(); exp2.Next())
            {
                Edge anEdge = ShapeCasts.Edge(exp2.Current());
                aC.Load(Tool.CurveOnSurface(anEdge, ShapeCasts.Face(myShape), out aFirst, out aLast)!);
                // To avoid exception in Segment if C1 is BSpline
                if (aC.FirstParameter() > aFirst)
                {
                    aFirst = aC.FirstParameter();
                }

                if (aC.LastParameter() < aLast)
                {
                    aLast = aC.LastParameter();
                }

                var aBoxE = new Box2d();
                Add2dCurve.Add(aC, aFirst, aLast, 0.0, ref aBoxE);
                aMapShapeBox2d.Bind(anEdge, aBoxE);
                aBoxW.Add(aBoxE);
            }

            aMapShapeBox2d.Bind(aWire, aBoxW);
        }

        int Nbwire, Index, Indexbis;
        Nbwire = myMapImb.Extent();

        Index = 1;
        while (Index < Nbwire)
        {
            for (exp1.Init(myShape, ShapeType.Wire), Indexbis = 0; exp1.More(); exp1.Next())
            {
                Indexbis++;
                if (Indexbis == Index)
                {
                    break;
                }
            }

            Wire wir1 = ShapeCasts.Wire(exp1.Current());
            // to reduce the number of calls Intersect(wir1,wir2)
            Box2d aBox1 = new(), aBox2 = new();
            if (aMapShapeBox2d.IsBound(wir1))
            {
                aBox1 = aMapShapeBox2d.Find(wir1);
            }

            exp1.Next();
            for (; exp1.More(); exp1.Next())
            {
                Wire wir2 = ShapeCasts.Wire(exp1.Current());
                if (aMapShapeBox2d.IsBound(wir2))
                {
                    aBox2 = aMapShapeBox2d.Find(wir2);
                }

                if (!aBox1.IsVoid && !aBox2.IsVoid && aBox1.IsOut(aBox2))
                {
                    continue;
                }

                if (Intersect(wir1, wir2, ShapeCasts.Face(myShape), aMapShapeBox2d))
                {
                    myIntres = BRepCheck_IntersectingWires;
                    if (Update)
                    {
                        BRepCheck.Add(aStatusList, myIntres);
                    }

                    return myIntres;
                }
            }

            Index++;
        }

        if (Update)
        {
            BRepCheck.Add(aStatusList, myIntres);
        }

        return myIntres;
    }

    /// <summary>
    /// Checks that one wire contains all the others, which contain nothing, or that no wire contains another (holes
    /// in an infinite face). Runs <see cref="IntersectWires"/> first. With <paramref name="Update"/>, the status is
    /// added to the face's statuses.
    /// </summary>
    public BRepCheck_Status ClassifyWires(bool Update = false)
    {
        NCollection_List<BRepCheck_Status> aStatusList = lockedStatusList();

        // It is assumed that each wire does not intersect any other one.
        if (myImbdone)
        {
            if (Update)
            {
                BRepCheck.Add(aStatusList, myImbres);
            }

            return myImbres;
        }

        myImbdone = true;
        myImbres = IntersectWires();
        if (myImbres != BRepCheck_NoError)
        {
            if (Update)
            {
                BRepCheck.Add(aStatusList, myImbres);
            }

            return myImbres;
        }

        int Nbwire = myMapImb.Extent();
        if (Nbwire < 1)
        {
            if (Update)
            {
                BRepCheck.Add(aStatusList, myImbres);
            }

            return myImbres;
        }

        var B = new BrepBuilder();
        var exp1 = new Explorer();
        var exp2 = new Explorer();

        Face aForwardFace = ShapeCasts.Face(myShape.Oriented(Forward));
        NCollection_DataMap<Shape, Box2d> aWireBoxMap = NewDataMapOfShapeBox2d();
        // Precompute reliable UV bounds for cheap rejection of disjoint wire pairs.
        for (exp1.Init(aForwardFace, ShapeType.Wire); exp1.More(); exp1.Next())
        {
            Wire aWire = ShapeCasts.Wire(exp1.Current());
            var aBox = new Box2d();
            if (ComputeWireUVBounds(aForwardFace, aWire, ref aBox))
            {
                aWireBoxMap.Bind(aWire, aBox);
            }
        }

        for (exp1.Init(aForwardFace, ShapeType.Wire); exp1.More(); exp1.Next())
        {
            Wire wir1 = ShapeCasts.Wire(exp1.Current());
            // OCCT.NET: an unfrozen copy of the TShape, sharing the surface, never handed out
            Shape aLocalShape = myShape.EmptyCopied();
            Face newFace = ShapeCasts.Face(aLocalShape);

            newFace.Orientation(Forward);
            B.Add(newFace, wir1);

            var FClass2d = new BRepTopAdaptor_FClass2d(newFace, global::Kerf.Engine.Foundation.Precision.PConfusion());
            bool WireBienOriente = false;
            if (FClass2d.PerformInfinitePoint() != Out)
            {
                WireBienOriente = true;
                // the given wire defines a hole
                myMapImb.UnBind(wir1);
                myMapImb.Bind(wir1.Reversed(), new NCollection_List<Shape>());
            }

            bool hasBox1 = aWireBoxMap.Find(wir1, out Box2d aBox1);

            for (exp2.Init(aForwardFace, ShapeType.Wire); exp2.More(); exp2.Next())
            {
                Wire wir2 = ShapeCasts.Wire(exp2.Current());
                if (wir2.IsSame(wir1))
                {
                    continue;
                }

                bool hasBox2 = aWireBoxMap.Find(wir2, out Box2d aBox2);
                // Wires with disjoint UV bounds cannot contain one another.
                if (hasBox1 && hasBox2 && aBox1.IsOut(aBox2))
                {
                    continue;
                }

                if (IsInside(wir2, WireBienOriente, FClass2d, newFace))
                {
                    myMapImb[wir1].Append(wir2);
                }
            }
        }

        // It is required to have 1 wire that contains all others, and the others should not
        // contain anything (case solid ended) or
        // the wires do not contain anything : in this case the wires should be
        // holes in an infinite face.
        Wire Wext = new();
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> itm in myMapImb)
        {
            if (!itm.Value.IsEmpty())
            {
                if (Wext.IsNull)
                {
                    Wext = ShapeCasts.Wire(itm.Key);
                }
                else
                {
                    myImbres = BRepCheck_InvalidImbricationOfWires;
                    if (Update)
                    {
                        BRepCheck.Add(aStatusList, myImbres);
                    }

                    return myImbres;
                }
            }
        }

        if (!Wext.IsNull)
        {
            // verifies that the list contains nbwire-1 elements
            if (myMapImb.Find(Wext).Extent() != Nbwire - 1)
            {
                myImbres = BRepCheck_InvalidImbricationOfWires;
                if (Update)
                {
                    BRepCheck.Add(aStatusList, myImbres);
                }

                return myImbres;
            }
        }

        // quit without errors
        if (Update)
        {
            BRepCheck.Add(aStatusList, myImbres);
        }

        return myImbres;
    }

    /// <summary>
    /// Checks that the outer wire is oriented as the face and the holes the other way. Runs
    /// <see cref="ClassifyWires"/> first. With <paramref name="Update"/>, the status is added to the face's statuses.
    /// </summary>
    public BRepCheck_Status OrientationOfWires(bool Update = false)
    {
        NCollection_List<BRepCheck_Status> aStatusList = lockedStatusList();
        // WARNING : it is assumed that the edges of a wire are correctly oriented
        bool Infinite = myShape.Infinite();
        if (myOridone)
        {
            if (Update)
            {
                BRepCheck.Add(aStatusList, myOrires);
            }

            return myOrires;
        }

        myOridone = true;
        myOrires = ClassifyWires();
        if (myOrires != BRepCheck_NoError)
        {
            if (Update)
            {
                BRepCheck.Add(aStatusList, myOrires);
            }

            return myOrires;
        }

        int Nbwire = myMapImb.Extent();
        Wire Wext = new();
        // OCCT's map iterator: the keys in the map's order (at most one wire contains others, ClassifyWires checked)
        Shape[] aKeys = [.. myMapImb.Select(p => p.Key)];
        int itm = 0;
        if (Nbwire == 1)
        {
            if (!Infinite)
            {
                Wext = ShapeCasts.Wire(aKeys[itm]);
            }
        }
        else
        {
            for (; itm < aKeys.Length; itm++)
            {
                if (!myMapImb.Find(aKeys[itm]).IsEmpty())
                {
                    Wext = ShapeCasts.Wire(aKeys[itm]);
                }
            }
        }

        if (Wext.IsNull && !Infinite)
        {
            if (Nbwire > 0)
            {
                myOrires = BRepCheck_InvalidImbricationOfWires;
            }

            if (Update)
            {
                BRepCheck.Add(aStatusList, myOrires);
            }

            return myOrires;
        }

        var exp = new Explorer(myShape.Oriented(Forward), ShapeType.Wire);
        for (; exp.More(); exp.Next())
        {
            Wire wir = ShapeCasts.Wire(exp.Current());
            if (!Wext.IsNull && wir.IsSame(Wext))
            {
                if (wir.Orientation() != Wext.Orientation())
                {
                    // the exterior wire defines a hole
                    if (CheckThin(wir, myShape.Oriented(Forward)))
                    {
                        return myOrires;
                    }

                    myOrires = BRepCheck_BadOrientationOfSubshape;
                    if (Update)
                    {
                        BRepCheck.Add(aStatusList, myOrires);
                    }

                    return myOrires;
                }
            }
            else
            {
                for (itm = 0; itm < aKeys.Length; itm++)
                {
                    if (aKeys[itm].IsSame(wir))
                    {
                        break;
                    }
                }

                // No control on More()
                // OCCT.NET: OCCT reads the key of an exhausted iterator; every wire of the face is a key, so the
                // port throws if one isn't
                if (itm == aKeys.Length)
                {
                    throw new NoSuchObjectException("BRepCheck_Face::OrientationOfWires");
                }

                if (aKeys[itm].Orientation() == wir.Orientation())
                {
                    // the given wire does not define a hole
                    myOrires = BRepCheck_BadOrientationOfSubshape;
                    if (Update)
                    {
                        BRepCheck.Add(aStatusList, myOrires);
                    }

                    return myOrires;
                }
            }
        }

        // quit without error
        // OCCT: adding NoError appends it after a fault already recorded (a face without surface, checked with
        // Update by the analyzer, has NoSurface then NoError)
        if (Update)
        {
            BRepCheck.Add(aStatusList, myOrires);
        }

        return myOrires;
    }

    /// <summary>Adds <see cref="BRepCheck_UnorientableShape"/> to the face's statuses.</summary>
    public void SetUnorientable() => SetStatus(BRepCheck_UnorientableShape);

    /// <summary>Adds <paramref name="theStatus"/> to the face's statuses.</summary>
    public void SetStatus(BRepCheck_Status theStatus)
    {
        bool aLocked = myIsParallel;
        if (aLocked)
        {
            myMutex.Enter();
        }

        try
        {
            BRepCheck.Add(myMap.Find(myShape), theStatus);
        }
        finally
        {
            if (aLocked)
            {
                myMutex.Exit();
            }
        }
    }

    /// <summary>
    /// True if <see cref="OrientationOfWires"/> found a fault, or (when it hasn't run) if the face is marked
    /// unorientable.
    /// </summary>
    public bool IsUnorientable()
    {
        if (myOridone)
        {
            return myOrires != BRepCheck_NoError;
        }

        for (var itl = new NCollection_List<BRepCheck_Status>.Iterator(myMap.Find(myShape)); itl.More(); itl.Next())
        {
            if (itl.Value() == BRepCheck_UnorientableShape)
            {
                return true;
            }
        }

        return false;
    }

    /// <summary>Sets the geometric controls; turning them on makes the wire checks run again.</summary>
    public void GeometricControls(bool B)
    {
        if (myGctrl != B)
        {
            if (B)
            {
                myIntdone = false;
                myImbdone = false;
                myOridone = false;
            }

            myGctrl = B;
        }
    }

    /// <summary>Whether the geometric controls are on.</summary>
    public bool GeometricControls() => myGctrl;

    private static bool Intersect(in Wire wir1,
                                  in Wire wir2,
                                  in Face F,
                                  NCollection_DataMap<Shape, Box2d> theMapEdgeBox)
    {
        double Inter2dTol = 1.0e-10;
        var exp1 = new Explorer();
        var exp2 = new Explorer();

        // Find common vertices of two wires - non-manifold case
        var MapW1 = new NCollection_Map<Shape>(ShapeHasher.Instance);
        var CommonVertices = new NCollection_Sequence<Shape>();
        for (exp1.Init(wir1, ShapeType.Vertex); exp1.More(); exp1.Next())
        {
            MapW1.Add(exp1.Current());
        }

        for (exp2.Init(wir2, ShapeType.Vertex); exp2.More(); exp2.Next())
        {
            Shape V = exp2.Current();
            if (MapW1.Contains(V))
            {
                CommonVertices.Append(V);
            }
        }

        // MSV 03.04.2002: create pure surface adaptor to avoid UVBounds computation
        //                 due to performance problem
        var Surf = new FaceSurfaceAdaptor(F, false);

        var PntSeq = new NCollection_Sequence<Point3D>();
        int i;
        for (i = 1; i <= CommonVertices.Length(); i++)
        {
            Vertex V = ShapeCasts.Vertex(CommonVertices[i]);
            Point2D P2d = Tool.Parameters(V, F);
            Point3D P = Surf.Value(P2d.X(), P2d.Y());
            PntSeq.Append(P);
        }

        var C1 = new CurveAdaptor2D();
        var C2 = new CurveAdaptor2D();
        Point2D pfirst1, plast1, pfirst2, plast2;
        double first1, last1, first2, last2;
        var Inter = new Geom2dInt_GInter();
        var myDomain1 = new IntRes2d_Domain();
        var myDomain2 = new IntRes2d_Domain();
        Box2d Box1 = new(), Box2 = new();

        for (exp1.Init(wir1, ShapeType.Edge); exp1.More(); exp1.Next())
        {
            Edge edg1 = ShapeCasts.Edge(exp1.Current());
            C1.Load(Tool.CurveOnSurface(edg1, F, out first1, out last1)!);
            // To avoid exception in Segment if C1 is BSpline - IFV
            if (C1.FirstParameter() > first1)
            {
                first1 = C1.FirstParameter();
            }

            if (C1.LastParameter() < last1)
            {
                last1 = C1.LastParameter();
            }

            Box1.SetVoid();
            if (theMapEdgeBox.IsBound(edg1))
            {
                Box1 = theMapEdgeBox.Find(edg1);
            }

            if (Box1.IsVoid)
            {
                Add2dCurve.Add(C1, first1, last1, 0.0, ref Box1);
            }

            for (exp2.Init(wir2, ShapeType.Edge); exp2.More(); exp2.Next())
            {
                Edge edg2 = ShapeCasts.Edge(exp2.Current());
                if (!edg1.IsSame(edg2))
                {
                    C2.Load(Tool.CurveOnSurface(edg2, F, out first2, out last2)!);
                    // To avoid exception in Segment if C2 is BSpline - IFV
                    if (C2.FirstParameter() > first2)
                    {
                        first2 = C2.FirstParameter();
                    }

                    if (C2.LastParameter() < last2)
                    {
                        last2 = C2.LastParameter();
                    }

                    Box2.SetVoid();
                    if (theMapEdgeBox.IsBound(edg2))
                    {
                        Box2 = theMapEdgeBox.Find(edg2);
                    }

                    if (Box2.IsVoid)
                    {
                        Add2dCurve.Add(C2, first2, last2, 0.0, ref Box2);
                    }

                    if (!Box1.IsOut(Box2))
                    {
                        Tool.UVPoints(edg1, F, out pfirst1, out plast1);
                        myDomain1.SetValues(pfirst1, first1, Inter2dTol, plast1, last1, Inter2dTol);
                        Tool.UVPoints(edg2, F, out pfirst2, out plast2);
                        myDomain2.SetValues(pfirst2, first2, Inter2dTol, plast2, last2, Inter2dTol);
                        Inter.Perform(C1, myDomain1, C2, myDomain2, Inter2dTol, Inter2dTol);
                        if (!Inter.IsDone())
                        {
                            return true;
                        }

                        if (Inter.NbSegments() > 0)
                        {
                            if (PntSeq.IsEmpty())
                            {
                                return true;
                            }
                            else
                            {
                                int NbCoinc = 0;
                                for (i = 1; i <= Inter.NbSegments(); i++)
                                {
                                    if (!Inter.Segment(i).HasFirstPoint() || !Inter.Segment(i).HasLastPoint())
                                    {
                                        return true;
                                    }

                                    Point2D FirstP2d = Inter.Segment(i).FirstPoint().Value();
                                    Point2D LastP2d = Inter.Segment(i).LastPoint().Value();
                                    Point3D FirstP = Surf.Value(FirstP2d.X(), FirstP2d.Y());
                                    Point3D LastP = Surf.Value(LastP2d.X(), LastP2d.Y());
                                    for (int j = 1; j <= PntSeq.Length(); j++)
                                    {
                                        double tolv = Tool.Tolerance(ShapeCasts.Vertex(CommonVertices[j]));
                                        if (FirstP.IsEqual(PntSeq[j], tolv) || LastP.IsEqual(PntSeq[j], tolv))
                                        {
                                            NbCoinc++;
                                            break;
                                        }
                                    }
                                }

                                return NbCoinc != Inter.NbSegments();
                            }
                        }

                        if (Inter.NbPoints() > 0)
                        {
                            if (PntSeq.IsEmpty())
                            {
                                return true;
                            }
                            else
                            {
                                int NbCoinc = 0;
                                for (i = 1; i <= Inter.NbPoints(); i++)
                                {
                                    Point2D P2d = Inter.Point(i).Value();
                                    Point3D P = Surf.Value(P2d.X(), P2d.Y());
                                    for (int j = 1; j <= PntSeq.Length(); j++)
                                    {
                                        double tolv = Tool.Tolerance(ShapeCasts.Vertex(CommonVertices[j]));
                                        tolv += 1.0e-8; // possible tolerance of intersection point
                                        double dd = P.SquareDistance(PntSeq[j]);
                                        if (dd <= tolv * tolv)
                                        {
                                            NbCoinc++;
                                            break;
                                        }
                                    }
                                }

                                return NbCoinc != Inter.NbPoints();
                            }
                        }
                    }
                }
            }
        }

        return false;
    }

    private static bool ComputeWireUVBounds(in Face theFace, in Wire theWire, ref Box2d theBox)
    {
        if (!Tool.IsClosed(theWire))
        {
            return false;
        }

        var aBox = new Box2d();
        for (var anExp = new Explorer(theWire, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Edge anEdge = ShapeCasts.Edge(anExp.Current());
            Curve2D? aPCurve = Tool.CurveOnSurface(anEdge, theFace, out double aFirst, out double aLast);
            if (aPCurve is null)
            {
                return false;
            }

            var aCurveAdaptor = new CurveAdaptor2D(aPCurve);
            // IsInside() uses the original edge range, so a clamped box cannot safely reject the wire.
            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(aFirst) || global::Kerf.Engine.Foundation.Precision.IsInfinite(aLast)
                || aFirst < aCurveAdaptor.FirstParameter() || aLast > aCurveAdaptor.LastParameter())
            {
                return false;
            }

            Add2dCurve.Add(aCurveAdaptor, aFirst, aLast, global::Kerf.Engine.Foundation.Precision.PConfusion(), ref aBox);
        }

        if (aBox.IsVoid || aBox.IsOpenXmin || aBox.IsOpenXmax || aBox.IsOpenYmin
            || aBox.IsOpenYmax)
        {
            return false;
        }

        theBox = aBox;
        return true;
    }

    private static bool IsInside(in Wire theWire,
                                 bool WireBienOriente,
                                 BRepTopAdaptor_FClass2d FClass2d,
                                 in Face theFace)
    {
        double aParameter, aFirst, aLast;

        var anExplorer = new Explorer(theWire, ShapeType.Edge);
        for (; anExplorer.More(); anExplorer.Next())
        {
            Edge anEdge = ShapeCasts.Edge(anExplorer.Current());
            Curve2D? aCurve2D = Tool.CurveOnSurface(anEdge, theFace, out aFirst, out aLast);

            // Selects the parameter of point on the curve
            if (!global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(aFirst) && !global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(aLast))
            {
                aParameter = (aFirst + aLast) * 0.5;

                // Edge is skipped if its parametric range is too small
                if (Math.Abs(aParameter - aFirst) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    continue;
                }

                // Edge is skipped if its length is too small
                Curve3D? aCurve = Tool.Curve(anEdge, out double aFirst3D, out double aLast3D);
                if (aCurve is null)
                {
                    continue;
                }

                // Compute start point of edge
                // OCCT evaluates the 3D curve at the pcurve's first parameter
                Point3D aPoints0 = aCurve.Value(aFirst);
                // Compute middle point of edge
                Point3D aPoints1 = aCurve.Value((aFirst3D + aLast3D) / 2.0);
                if (aPoints0.Distance(aPoints1) < global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    continue;
                }
            }
            else
            {
                if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(aFirst) && global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(aLast))
                {
                    aParameter = 0.0;
                }
                else if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(aFirst))
                {
                    aParameter = aLast - 1.0;
                }
                else
                {
                    aParameter = aFirst + 1.0;
                }
            }

            // Find point on curve (edge)
            Point2D aPoint2D = aCurve2D!.Value(aParameter);
            // Compute the topological position of a point relative to face
            Containment aState = FClass2d.Perform(aPoint2D, false);

            if (WireBienOriente)
            {
                return aState == Out;
            }
            else
            {
                return aState == In;
            }
        }

        return false;
    }

    private static bool CheckThin(in Shape w, in Shape f)
    {
        Face aF = ShapeCasts.Face(f);
        Wire aW = ShapeCasts.Wire(w);

        int nbE = 0;
        var lE = new NCollection_List<Shape>();
        var exp = new Explorer(aW, ShapeType.Edge);
        for (; exp.More(); exp.Next())
        {
            Shape s = exp.Current();
            lE.Append(s);
            nbE++;
        }

        if (nbE != 2)
        {
            return false;
        }

        Edge e1 = ShapeCasts.Edge(lE.First());
        Edge e2 = ShapeCasts.Edge(lE.Last());

        Exploration.Vertices(e1, out Vertex v1, out Vertex v2);
        Exploration.Vertices(e2, out Vertex v3, out Vertex v4);

        if (v1.IsNull || v2.IsNull || v3.IsNull || v4.IsNull)
        {
            return false;
        }

        if (v1.IsSame(v2) || v3.IsSame(v4))
        {
            return false;
        }

        bool sF = false, sL = false;
        if (v1.IsSame(v3) || v1.IsSame(v4))
        {
            sF = true;
        }

        if (v2.IsSame(v3) || v2.IsSame(v4))
        {
            sL = true;
        }

        if (!sF || !sL)
        {
            return false;
        }

        Orientation e1or = e1.Orientation();
        Orientation e2or = e2.Orientation();

        Curve2D? pc1 = Tool.CurveOnSurface(e1, aF, out double f1, out double l1);
        Curve2D? pc2 = Tool.CurveOnSurface(e2, aF, out double f2, out double l2);

        if (pc1 is null || pc2 is null)
        {
            return false;
        }

        double d1 = Math.Abs(l1 - f1) / 100.0;
        double d2 = Math.Abs(l2 - f2) / 100.0;
        double m1 = (l1 + f1) * 0.5;
        double m2 = (l2 + f2) * 0.5;

        Point2D p1f = pc1.Value(m1 - d1);
        Point2D p1l = pc1.Value(m1 + d1);
        Point2D p2f = pc2.Value(m2 - d2);
        Point2D p2l = pc2.Value(m2 + d2);

        var vc1 = new Vector2D(p1f, p1l);
        var vc2 = new Vector2D(p2f, p2l);

        return (vc1 * vc2) < 0.0 || e1or != e2or;
    }
}
