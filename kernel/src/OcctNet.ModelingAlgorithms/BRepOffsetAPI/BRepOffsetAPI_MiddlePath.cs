// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_MiddlePath.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_MiddlePath.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2012-2014 OPEN CASCADE SAS; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Curves;
using Kerf.Engine.Construction.Values;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Schematiq.Collections;
using static Occt.BRepExtrema_SupportType;
using static Kerf.Geometry.CurveKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// The middle path of a pipe-like shape: the wire through the centres of mass of the sections between a start and an
/// end section (wires or faces of the shape).
/// </summary>
/// <remarks>
/// It only reads the shape: ShapeUpgrade_UnifySameDomain works on a copy of a frozen shape (decision 928), and every
/// edge it builds is new. The result is frozen when returned (design 4.2).
/// </remarks>
public class BRepOffsetAPI_MiddlePath : BRepBuilderAPI_MakeShape
{
    private readonly Shape myInitialShape;
    private readonly Wire myStartWire;
    private readonly Wire myEndWire;
    private readonly bool myClosedSection;
    private readonly bool myClosedRing;
    private readonly NCollection_Map<Shape> myStartWireEdges = new(ShapeHasher.Instance);
    private readonly NCollection_Map<Shape> myEndWireEdges = new(ShapeHasher.Instance);
    private readonly NCollection_Sequence<NCollection_Sequence<Shape>> myPaths = new();

    private static bool IsLinear(in Edge anEdge, out Line3D aLine)
    {
        aLine = new Line3D();
        Curve3D aCurve = Tool.Curve(anEdge, out _, out _)!;
        if (aCurve.GetType() == typeof(TrimmedCurve3D))
        {
            aCurve = ((TrimmedCurve3D)aCurve).BasisCurve();
        }

        Point3D Pnt1, Pnt2;
        if (aCurve is LineCurve3D aGeomLine)
        {
            aLine = aGeomLine.Lin();
            return true;
        }
        else if (aCurve is BezierCurve3D theBezier)
        {
            if (theBezier.PoleCount == 2)
            {
                Pnt1 = theBezier.Pole(1);
                Pnt2 = theBezier.Pole(2);
                aLine = new MakeLin(Pnt1, Pnt2).Value();
                return true;
            }
        }
        else if (aCurve is BSplineCurve3D theBSpline)
        {
            if (theBSpline.PoleCount == 2)
            {
                Pnt1 = theBSpline.Pole(1);
                Pnt2 = theBSpline.Pole(2);
                aLine = new MakeLin(Pnt1, Pnt2).Value();
                return true;
            }
        }

        return false;
    }

    private static CurveKind TypeOfEdge(in Edge anEdge)
    {
        if (IsLinear(anEdge, out _))
        {
            return Line;
        }

        EdgeCurveAdaptor BAcurve = new(anEdge);
        return BAcurve.GetType();
    }

    private static Vector3D TangentOfEdge(in Shape aShape, bool OnFirst)
    {
        Edge anEdge = ShapeCasts.Edge(aShape);
        Orientation anOr = anEdge.Orientation();

        Curve3D aCurve = Tool.Curve(anEdge, out double fpar, out double lpar)!;
        double thePar;
        if (OnFirst)
        {
            thePar = (anOr == Forward) ? fpar : lpar;
        }
        else
        {
            thePar = (anOr == Forward) ? lpar : fpar;
        }

        aCurve.D1(thePar, out _, out Vector3D theTangent);
        if (anOr == Reversed)
        {
            theTangent.Reverse();
        }

        return theTangent;
    }

    private static bool IsValidEdge(in Edge theEdge, in Face theFace)
    {
        Exploration.Vertices(theEdge, out Vertex V1, out Vertex V2);

        double Tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        int i;

        Explorer Explo = new(theFace, ShapeType.Edge);
        for (; Explo.More(); Explo.Next())
        {
            Shape anEdge = Explo.Current();
            BRepExtrema_DistShapeShape DistMini = new(theEdge, anEdge);
            if (DistMini.Value() <= Tol)
            {
                for (i = 1; i <= DistMini.NbSolution(); i++)
                {
                    BRepExtrema_SupportType theType = DistMini.SupportTypeShape2(i);
                    if (theType == BRepExtrema_IsOnEdge)
                    {
                        return false;
                    }
                    // theType is "IsVertex"
                    Shape aVertex = DistMini.SupportOnShape2(i);
                    if (!(aVertex.IsSame(V1) || aVertex.IsSame(V2)))
                    {
                        return false;
                    }
                }
            }
        }

        return true;
    }

    // (OCCT's commented-out constructors from a start wire or a start edge are left out.)

    //=================================================================================================

    private static Wire GetUnifiedWire(in Wire theWire, ShapeUpgrade_UnifySameDomain theUnifier)
    {
        BRepLib_MakeWire aWMaker = new();
        WireExplorer wexp = new(theWire);
        NCollection_Map<Shape> aGeneratedEdges = new(ShapeHasher.Instance);
        for (; wexp.More(); wexp.Next())
        {
            Shape anEdge = wexp.Current();
            ISchematiqList<Shape> aLS = theUnifier.History()!.Modified(anEdge);
            if (aLS.Count != 0)
            {
                foreach (Shape aShape in aLS)
                {
                    // wire shouldn't contain duplicated generated edges
                    if (aGeneratedEdges.Add(aShape))
                    {
                        aWMaker.Add(ShapeCasts.Edge(aShape));
                    }
                }
            }
            else
            {
                // no change, put original edge
                aWMaker.Add(ShapeCasts.Edge(anEdge));
            }
        }
        return aWMaker.Wire();
    }

    //=================================================================================================

    /// <summary>
    /// The middle path of <paramref name="aShape"/> (a shell or a solid) from <paramref name="StartShape"/> to
    /// <paramref name="EndShape"/>, each a wire or a face of it (the face's outer wire).
    /// </summary>
    public BRepOffsetAPI_MiddlePath(in Shape aShape, in Shape StartShape, in Shape EndShape)
    {
        ShapeUpgrade_UnifySameDomain Unifier = new(aShape);
        Unifier.Build();
        myInitialShape = Unifier.Shape();

        Wire aStartWire, anEndWire;
        if (StartShape.ShapeType() == ShapeType.Face)
        {
            Face StartFace = ShapeCasts.Face(StartShape);
            aStartWire = BrepTools.OuterWire(StartFace);
        }
        else
        {
            aStartWire = ShapeCasts.Wire(StartShape);
        }

        if (EndShape.ShapeType() == ShapeType.Face)
        {
            Face EndFace = ShapeCasts.Face(EndShape);
            anEndWire = BrepTools.OuterWire(EndFace);
        }
        else
        {
            anEndWire = ShapeCasts.Wire(EndShape);
        }

        myStartWire = GetUnifiedWire(aStartWire, Unifier);
        myEndWire = GetUnifiedWire(anEndWire, Unifier);

        myClosedSection = myStartWire.Closed();
        myClosedRing = myStartWire.IsSame(myEndWire);
        _ = myClosedRing;
    }

    //=================================================================================================

    /// <summary>Builds the middle path.</summary>
    public override void Build(ProgressRange theRange = default)
    {
        NCollection_List<Shape>.Iterator itl = new();

        NCollection_Sequence<Shape> StartVertices = new();
        NCollection_Map<Shape> EndVertices = new(ShapeHasher.Instance);
        NCollection_Map<Shape> EndEdges = new(ShapeHasher.Instance);
        NCollection_Sequence<NCollection_Sequence<Shape>> SectionsEdges = new();

        WireExplorer wexp = new(myStartWire);
        NCollection_Sequence<Shape> EdgeSeq = new();
        for (; wexp.More(); wexp.Next())
        {
            StartVertices.Append(wexp.CurrentVertex());
            EdgeSeq.Append(wexp.Current());
        }
        if (!myClosedSection)
        {
            StartVertices.Append(wexp.CurrentVertex());
        }
        SectionsEdges.Append(new NCollection_Sequence<Shape>(EdgeSeq));

        for (wexp.Init(myEndWire); wexp.More(); wexp.Next())
        {
            EndVertices.Add(wexp.CurrentVertex());
            EndEdges.Add(wexp.Current());
        }
        if (!myClosedSection)
        {
            EndVertices.Add(wexp.CurrentVertex());
        }

        Iterator itw = new(myStartWire);
        for (; itw.More(); itw.Next())
        {
            myStartWireEdges.Add(itw.Value());
        }
        for (itw.Initialize(myEndWire); itw.More(); itw.Next())
        {
            myEndWireEdges.Add(itw.Value());
        }

        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> VEmap = new(ShapeHasher.Instance);
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> EFmap = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(myInitialShape, ShapeType.Vertex, ShapeType.Edge, VEmap);
        Exploration.MapShapesAndAncestors(myInitialShape, ShapeType.Edge, ShapeType.Face, EFmap);

        NCollection_Map<Shape> CurVertices = new(ShapeHasher.Instance);

        int i, j, k;
        Edge anEdge = new();
        Vertex V1, V2, NextVertex;
        // Initialization of <myPaths>
        for (i = 1; i <= StartVertices.Length(); i++)
        {
            NCollection_Sequence<Shape> Edges = new();
            NCollection_List<Shape> LE = VEmap.FindFromKey(StartVertices[i]);
            for (itl.Initialize(LE); itl.More(); itl.Next())
            {
                anEdge = ShapeCasts.Edge(itl.Value());
                if (!myStartWireEdges.Contains(anEdge))
                {
                    Exploration.Vertices(anEdge, out V1, out V2, true);
                    if (V1.IsSame(StartVertices[i]))
                    {
                        CurVertices.Add(V2);
                    }
                    else
                    {
                        anEdge.Reverse();
                        CurVertices.Add(V1);
                    }
                    Edges.Append(anEdge);
                    break;
                }
            }
            if (!Edges.IsEmpty())
            {
                myPaths.Append(Edges);
            }
            else
            {
                return;
            }
        }

        // Filling of "myPaths"
        NCollection_List<Shape> NextVertices = new();
        for (;;)
        {
            for (i = 1; i <= myPaths.Length(); i++)
            {
                Shape theShape = myPaths[i].Last();
                Edge theEdge;
                Vertex theVertex;
                if (theShape.ShapeType() == ShapeType.Edge)
                {
                    theEdge = ShapeCasts.Edge(theShape);
                    theVertex = Exploration.LastVertex(theEdge, true);
                }
                else // last segment of path was punctual
                {
                    theEdge = ShapeCasts.Edge(myPaths[i][myPaths[i].Length() - 1]);
                    theVertex = ShapeCasts.Vertex(theShape);
                }

                if (EndVertices.Contains(theVertex))
                {
                    continue;
                }
                NCollection_List<Shape> LE = VEmap.FindFromKey(theVertex);
                NCollection_Map<Shape> NextEdgeCandidates = new(ShapeHasher.Instance);
                for (itl.Initialize(LE); itl.More(); itl.Next())
                {
                    anEdge = ShapeCasts.Edge(itl.Value());
                    if (anEdge.IsSame(theEdge))
                    {
                        continue;
                    }
                    Exploration.Vertices(anEdge, out V1, out V2, true);
                    if (V1.IsSame(theVertex))
                    {
                        NextVertex = V2;
                    }
                    else
                    {
                        anEdge.Reverse();
                        NextVertex = V1;
                    }
                    if (!CurVertices.Contains(NextVertex))
                    {
                        NextEdgeCandidates.Add(anEdge);
                    }
                }
                if (!NextEdgeCandidates.IsEmpty())
                {
                    if (NextEdgeCandidates.Extent() > 1)
                    {
                        myPaths[i].Append(theVertex); // punctual segment of path
                    }
                    else
                    {
                        foreach (Shape aKey in NextEdgeCandidates)
                        {
                            anEdge = ShapeCasts.Edge(aKey);
                        }
                        myPaths[i].Append(anEdge);
                        NextVertex = Exploration.LastVertex(anEdge, true);
                        NextVertices.Append(NextVertex);
                    }
                }
            }
            if (NextVertices.IsEmpty())
            {
                break;
            }
            for (itl.Initialize(NextVertices); itl.More(); itl.Next())
            {
                CurVertices.Add(itl.Value());
            }
            NextVertices.Clear();
        }

        // Building of set of sections
        int NbE = EdgeSeq.Length();
        int NbPaths = myPaths.Length();
        int NbVer = myPaths.Length();
        if (myClosedSection)
        {
            NbVer++;
        }
        i = 1;
        for (;;)
        {
            for (j = 1; j <= EdgeSeq.Length(); j++)
            {
                EdgeSeq[j].Nullify();
            }

            bool ToInsertVertex = false;

            for (j = 2; j <= NbVer; j++)
            {
                if (!EdgeSeq[j - 1].IsNull)
                {
                    continue;
                }

                NCollection_Sequence<Shape> aPath1 = myPaths[j - 1];
                NCollection_Sequence<Shape> aPath2 = myPaths[(j <= NbPaths) ? j : 1];

                // for the end of initial shape
                if (aPath1.Length() < i)
                {
                    Edge aE1 = ShapeCasts.Edge(aPath1[i - 1]);
                    Shape LastVer = Exploration.LastVertex(aE1, true);
                    aPath1.Append(LastVer);
                }
                if (aPath2.Length() < i)
                {
                    Edge aE2 = ShapeCasts.Edge(aPath2[i - 1]);
                    Shape LastVer = Exploration.LastVertex(aE2, true);
                    aPath2.Append(LastVer);
                }
                //////////////////////////////

                if (ToInsertVertex)
                {
                    if (aPath1[i].ShapeType() == ShapeType.Edge)
                    {
                        Edge aE1 = ShapeCasts.Edge(aPath1[i]);
                        Shape fver = Exploration.FirstVertex(aE1, true);
                        aPath1.InsertBefore(i, fver);
                    }
                    if (aPath2[i].ShapeType() == ShapeType.Edge)
                    {
                        Edge aE2 = ShapeCasts.Edge(aPath2[i]);
                        Shape fver = Exploration.FirstVertex(aE2, true);
                        aPath2.InsertBefore(i, fver);
                    }
                    ToInsertVertex = false;
                }

                Edge E1 = new(), E2 = new();
                if (aPath1[i].ShapeType() == ShapeType.Edge)
                {
                    E1 = ShapeCasts.Edge(aPath1[i]);
                }
                if (aPath2[i].ShapeType() == ShapeType.Edge)
                {
                    E2 = ShapeCasts.Edge(aPath2[i]);
                }
                Edge E12 = ShapeCasts.Edge(SectionsEdges[i][j - 1]);
                // Find the face on which (E1 or E2) and E12 lie
                Shape E1orE2 = (E1.IsNull) ? E2 : E1;
                if (E1orE2.IsNull) // both E1 and E2 are vertices =>
                {
                    EdgeSeq[j - 1] = E12; // => proper edge is the edge of previous section between them
                    continue;
                }
                NCollection_List<Shape> LF = EFmap.FindFromKey(E1orE2);
                Face theFace = new();
                for (itl.Initialize(LF); itl.More(); itl.Next())
                {
                    Shape aFace = itl.Value();
                    NCollection_List<Shape> LF2 = EFmap.FindFromKey(E12);
                    NCollection_List<Shape>.Iterator itl2 = new(LF2);
                    for (; itl2.More(); itl2.Next())
                    {
                        Shape aFace2 = itl2.Value();
                        if (aFace.IsSame(aFace2))
                        {
                            theFace = ShapeCasts.Face(aFace);
                            break;
                        }
                    }
                    if (!theFace.IsNull)
                    {
                        break;
                    }
                }

                Vertex PrevVertex = (E1.IsNull) ? ShapeCasts.Vertex(aPath1[i]) : Exploration.LastVertex(E1, true);
                Vertex CurVertex = (E2.IsNull) ? ShapeCasts.Vertex(aPath2[i]) : Exploration.LastVertex(E2, true);

                Edge ProperEdge = new();
                NCollection_List<Shape> LE = VEmap.FindFromKey(PrevVertex);
                NCollection_IndexedMap<Shape> EdgesOfTheFace = new(ShapeHasher.Instance);
                Exploration.MapShapes(theFace, ShapeType.Edge, EdgesOfTheFace);
                for (itl.Initialize(LE); itl.More(); itl.Next())
                {
                    anEdge = ShapeCasts.Edge(itl.Value());
                    Exploration.Vertices(anEdge, out V1, out V2);
                    if (((V1.IsSame(PrevVertex) && V2.IsSame(CurVertex)) || (V1.IsSame(CurVertex) && V2.IsSame(PrevVertex)))
                        && EdgesOfTheFace.Contains(anEdge) && // this condition is for a section of two edges
                        !anEdge.IsSame(E1))                   // the last condition is for torus-like shape
                    {
                        ProperEdge = anEdge;
                        break;
                    }
                }

                if ((aPath1[i]).ShapeType() == ShapeType.Vertex && (aPath2[i]).ShapeType() == ShapeType.Vertex)
                {
                    EdgeSeq[j - 1] = ProperEdge;
                    continue;
                }

                Vertex PrevPrevVer = (E1.IsNull) ? PrevVertex : Exploration.FirstVertex(E1, true);
                Vertex PrevCurVer = (E2.IsNull) ? CurVertex : Exploration.FirstVertex(E2, true);
                if (ProperEdge.IsNull) // no connection between these two vertices
                {
                    // Find the face on which E1, E2 and E12 lie
                    // ToInsertVertex = false;
                    NCollection_List<Shape> ListOneFace = new();
                    ListOneFace.Append(theFace);

                    if (E1.IsNull || E2.IsNull)
                    {
                        if (E1.IsNull)
                        {
                            E1 = ShapeCasts.Edge(aPath1[i - 1]);
                        }
                        if (E2.IsNull)
                        {
                            E2 = ShapeCasts.Edge(aPath2[i - 1]);
                        }
                        double LastPar1, LastPar2;
                        Curve2D PCurve1 = Tool.CurveOnSurface(E1, theFace, out double fpar1, out double lpar1)!;
                        Curve2D PCurve2 = Tool.CurveOnSurface(E2, theFace, out double fpar2, out double lpar2)!;
                        if (E1.Orientation() == Forward)
                        {
                            LastPar1 = lpar1;
                        }
                        else
                        {
                            LastPar1 = fpar1;
                        }
                        if (E2.Orientation() == Forward)
                        {
                            LastPar2 = lpar2;
                        }
                        else
                        {
                            LastPar2 = fpar2;
                        }
                        Point2D FirstPnt2d = PCurve1.Value(LastPar1);
                        Point2D LastPnt2d = PCurve2.Value(LastPar2);
                        Surface theSurf = Tool.Surface(theFace)!;
                        LineCurve2D theLine = new MakeLine2d(FirstPnt2d, LastPnt2d).Value();
                        double len_ne = FirstPnt2d.Distance(LastPnt2d);
                        Edge NewEdge = new BRepLib_MakeEdge(theLine, theSurf, PrevVertex, CurVertex, 0.0, len_ne).InternalEdge();
                        BRepLib.BuildCurve3d(NewEdge);
                        EdgeSeq[j - 1] = NewEdge;
                        EFmap.Add(NewEdge, ListOneFace);
                    }
                    else // E1 is edge
                    {
                        // Extract points 2d
                        double FirstPar1, LastPar1, FirstPar2, LastPar2;
                        Curve2D PCurve1 = Tool.CurveOnSurface(E1, theFace, out double fpar1, out double lpar1)!;
                        Curve2D PCurve2 = Tool.CurveOnSurface(E2, theFace, out double fpar2, out double lpar2)!;
                        if (E1.Orientation() == Forward)
                        {
                            FirstPar1 = fpar1;
                            LastPar1 = lpar1;
                        }
                        else
                        {
                            FirstPar1 = lpar1;
                            LastPar1 = fpar1;
                        }
                        if (E2.Orientation() == Forward)
                        {
                            FirstPar2 = fpar2;
                            LastPar2 = lpar2;
                        }
                        else
                        {
                            FirstPar2 = lpar2;
                            LastPar2 = fpar2;
                        }
                        Point2D FirstPnt2d = PCurve1.Value(LastPar1);
                        Point2D LastPnt2d = PCurve2.Value(LastPar2);
                        Surface theSurf = Tool.Surface(theFace)!;
                        LineCurve2D theLine = new MakeLine2d(FirstPnt2d, LastPnt2d).Value();
                        double len_ne = FirstPnt2d.Distance(LastPnt2d);
                        Edge NewEdge = new BRepLib_MakeEdge(theLine, theSurf, PrevVertex, CurVertex, 0.0, len_ne).InternalEdge();
                        BRepLib.BuildCurve3d(NewEdge);
                        Point2D PrevFirstPnt2d = PCurve1.Value(FirstPar1);
                        Point2D PrevLastPnt2d = PCurve2.Value(FirstPar2);
                        LineCurve2D Line1 = new MakeLine2d(PrevFirstPnt2d, LastPnt2d).Value();
                        LineCurve2D Line2 = new MakeLine2d(FirstPnt2d, PrevLastPnt2d).Value();
                        double len_ne1 = PrevFirstPnt2d.Distance(LastPnt2d);
                        Edge NewEdge1 = new BRepLib_MakeEdge(Line1, theSurf, PrevPrevVer, CurVertex, 0.0, len_ne1).InternalEdge();
                        BRepLib.BuildCurve3d(NewEdge1);
                        double len_ne2 = FirstPnt2d.Distance(PrevLastPnt2d);
                        Edge NewEdge2 = new BRepLib_MakeEdge(Line2, theSurf, PrevVertex, PrevCurVer, 0.0, len_ne2).InternalEdge();
                        BRepLib.BuildCurve3d(NewEdge2);
                        bool good_ne = IsValidEdge(NewEdge, theFace);
                        bool good_ne1 = IsValidEdge(NewEdge1, theFace);

                        CurveKind type_E1 = TypeOfEdge(E1);
                        CurveKind type_E2 = TypeOfEdge(E2);

                        int ChooseEdge = 0;

                        if (!good_ne || type_E1 != type_E2)
                        {
                            if (type_E1 == type_E2) //! good_ne
                            {
                                if (good_ne1)
                                {
                                    ChooseEdge = 1;
                                }
                                else
                                {
                                    ChooseEdge = 2;
                                }
                            }
                            else // types are different
                            {
                                if (type_E1 == Line)
                                {
                                    ChooseEdge = 1;
                                }
                                else if (type_E2 == Line)
                                {
                                    ChooseEdge = 2;
                                }
                                else // to be developed later...
                                {
                                }
                            }
                        }

                        if (ChooseEdge == 0)
                        {
                            EdgeSeq[j - 1] = NewEdge;
                            EFmap.Add(NewEdge, ListOneFace);
                        }
                        else if (ChooseEdge == 1)
                        {
                            EdgeSeq[j - 1] = NewEdge1;
                            EFmap.Add(NewEdge1, ListOneFace);
                            for (k = 1; k < j - 1; k++)
                            {
                                EdgeSeq[k].Nullify();
                            }
                            for (k = 1; k <= j - 1; k++)
                            {
                                Edge aLastEdge = ShapeCasts.Edge(myPaths[k][i]);
                                Shape VertexAsEdge = Exploration.FirstVertex(aLastEdge, true);
                                myPaths[k].InsertBefore(i, VertexAsEdge);
                            }
                            j = 1; // start from beginning
                        }
                        else if (ChooseEdge == 2)
                        {
                            EdgeSeq[j - 1] = NewEdge2;
                            EFmap.Add(NewEdge2, ListOneFace);
                            ToInsertVertex = true;
                        }
                    } // else //E1 is edge
                } // if (ProperEdge.IsNull())
                else // connecting edge exists
                {
                    EdgeSeq[j - 1] = ProperEdge;
                }
            } // for (j = 2; j <= NbVer; j++)
            SectionsEdges.Append(new NCollection_Sequence<Shape>(EdgeSeq));

            // check for exit from for(;;)
            int NbEndEdges = 0;
            for (j = 1; j <= EdgeSeq.Length(); j++)
            {
                if (EndEdges.Contains(EdgeSeq[j]))
                {
                    NbEndEdges++;
                }
            }
            if (NbEndEdges == NbE)
            {
                break;
            }

            i++;
        } // for (;;)

        // final phase: building of middle path
        int NbSecFaces = SectionsEdges.Length();
        NCollection_Array1<Shape> SecFaces = new(1, NbSecFaces);
        for (i = 1; i <= NbSecFaces; i++)
        {
            BRepLib_MakeWire MW = new();
            for (j = 1; j <= NbE; j++)
            {
                anEdge = ShapeCasts.Edge(SectionsEdges[i][j]);
                MW.Add(anEdge);
            }
            if (!myClosedSection)
            {
                Exploration.Vertices(MW.InternalWire(), out V1, out V2);
                anEdge = new BRepLib_MakeEdge(V2, V1).InternalEdge();
                MW.Add(anEdge);
            }
            Wire aWire = MW.InternalWire();
            BRepLib_MakeFace MF = new(aWire, true); // Only plane
            if (MF.IsDone())
            {
                SecFaces[i] = MF.InternalFace();
            }
            else
            {
                SecFaces[i] = aWire;
            }
        }

        NCollection_Array1<Point3D> Centers = new(1, NbSecFaces);
        for (i = 1; i <= NbSecFaces; i++)
        {
            Properties Properties = new();
            if (SecFaces[i].ShapeType() == ShapeType.Face)
            {
                BRepGProp.SurfaceProperties(SecFaces[i], Properties);
            }
            else
            { // wire
                BRepGProp.LinearProperties(SecFaces[i], Properties);
            }

            Centers[i] = Properties.CentreOfMass();
        }

        NCollection_Array1<Shape> MidEdges = new(1, NbSecFaces - 1);
        double LinTol = 1.0e-5;
        double AngTol = 1.0e-7;
        Point3D Pnt1, Pnt2;
        for (i = 1; i < NbSecFaces; i++)
        {
            CurveKind TypeOfMidEdge = OtherCurve;
            for (j = 1; j <= myPaths.Length(); j++)
            {
                Shape aShape = myPaths[j][i];
                if (aShape.ShapeType() == ShapeType.Vertex)
                {
                    TypeOfMidEdge = OtherCurve;
                    break;
                }
                anEdge = ShapeCasts.Edge(aShape);
                CurveKind aType = TypeOfEdge(anEdge);
                if (j == 1)
                {
                    TypeOfMidEdge = aType;
                }
                else
                {
                    if (aType != TypeOfMidEdge)
                    {
                        TypeOfMidEdge = OtherCurve;
                        break;
                    }
                }
            }
            if (TypeOfMidEdge == Line)
            {
                MidEdges[i] = new BRepLib_MakeEdge(Centers[i], Centers[i + 1]).InternalEdge();
            }
            else if (TypeOfMidEdge == Circle)
            {
                Axis3D theAxis = new();
                Direction3D theDir1 = new(), theDir2 = new();
                double theAngle = 0.0;
                Vector3D theTangent = new();
                bool SimilarArcs = true;
                for (j = 1; j <= myPaths.Length(); j++)
                {
                    anEdge = ShapeCasts.Edge(myPaths[j][i]);
                    Curve3D aCurve = Tool.Curve(anEdge, out double fpar, out double lpar)!;
                    if (aCurve.GetType() == typeof(TrimmedCurve3D))
                    {
                        aCurve = ((TrimmedCurve3D)aCurve).BasisCurve();
                    }
                    Pnt1 = aCurve.Value(fpar);
                    Pnt2 = aCurve.Value(lpar);
                    CircleCurve3D aCircle = (CircleCurve3D)aCurve;
                    Circle3D aCirc = aCircle.Circ();
                    if (j == 1)
                    {
                        theAxis = aCirc.Axis();
                        theDir1 = new Direction3D(new Vector3D(aCirc.Location(), Pnt1));
                        theDir2 = new Direction3D(new Vector3D(aCirc.Location(), Pnt2));
                        theAngle = lpar - fpar;
                        double theParam = (anEdge.Orientation() == Forward) ? fpar : lpar;
                        aCurve.D1(theParam, out Pnt1, out theTangent);
                        if (anEdge.Orientation() == Reversed)
                        {
                            theTangent.Reverse();
                        }
                    }
                    else
                    {
                        Axis3D anAxis = aCirc.Axis();
                        Line3D aLin = new(anAxis);
                        if (!aLin.Contains(theAxis.Location(), LinTol) || !anAxis.IsParallel(theAxis, AngTol))
                        {
                            SimilarArcs = false;
                            break;
                        }
                        Direction3D aDir1 = new(new Vector3D(aCirc.Location(), Pnt1));
                        Direction3D aDir2 = new(new Vector3D(aCirc.Location(), Pnt2));
                        if (!((aDir1.IsEqual(theDir1, AngTol) && aDir2.IsEqual(theDir2, AngTol))
                              || (aDir1.IsEqual(theDir2, AngTol) && aDir2.IsEqual(theDir1, AngTol))))
                        {
                            SimilarArcs = false;
                            break;
                        }
                    }
                }
                if (SimilarArcs)
                {
                    Xyz AxisLoc = theAxis.Location().XYZ();
                    Xyz AxisDir = theAxis.Direction().XYZ();
                    double Parameter = (Centers[i].XYZ() - AxisLoc) * AxisDir;
                    Point3D theCenterOfCirc = new(AxisLoc + Parameter * AxisDir);

                    Vector3D Vec1 = new(theCenterOfCirc, Centers[i]);
                    Vector3D Vec2 = new(theCenterOfCirc, Centers[i + 1]);

                    double anAngle = Vec1.AngleWithRef(Vec2, new Vector3D(theAxis.Direction()));
                    if (anAngle < 0.0)
                    {
                        anAngle += 2.0 * M_PI;
                    }
                    if (Math.Abs(anAngle - theAngle) > AngTol)
                    {
                        theAxis.Reverse();
                    }
                    Frame3D theAx2 = new(theCenterOfCirc, theAxis.Direction(), new Direction3D(Vec1));
                    CircleCurve3D theCircle = new MakeCircle(theAx2, Vec1.Magnitude()).Value();
                    theCircle.D1(0.0, out Pnt1, out Vector3D aTangent);
                    if (aTangent * theTangent < 0.0)
                    {
                        theAxis.Reverse();
                        theAx2 = new Frame3D(theCenterOfCirc, theAxis.Direction(), new Direction3D(Vec1));
                        theCircle = new MakeCircle(theAx2, Vec1.Magnitude()).Value();
                    }
                    BRepLib_MakeEdge aME = new(theCircle, 0.0, theAngle);
                    aME.Build();

                    MidEdges[i] = aME.IsDone() ? aME.InternalEdge() : new Edge();
                }
            }
        }

        // Build missed edges
        for (i = 1; i < NbSecFaces; i++)
        {
            if (MidEdges[i].IsNull)
            {
                for (j = i + 1; j < NbSecFaces; j++)
                {
                    if (!MidEdges[j].IsNull)
                    {
                        break;
                    }
                }
                // from i to j-1 all edges are null
                NCollection_HArray1<Point3D> thePoints = new(1, j - i + 1);
                NCollection_Array1<Vector3D> theTangents = new(1, j - i + 1);
                NCollection_HArray1<bool> theFlags = new(1, j - i + 1);
                for (k = i; k <= j; k++)
                {
                    thePoints.SetValue(k - i + 1, Centers[k]);
                }
                for (k = i; k <= j; k++)
                {
                    NCollection_Sequence<Point3D> PntSeq = new();
                    for (int indp = 1; indp <= myPaths.Length(); indp++)
                    {
                        Vector3D aTangent;
                        if (k == i)
                        {
                            if (myPaths[indp][k].ShapeType() == ShapeType.Vertex)
                            {
                                continue;
                            }
                            aTangent = TangentOfEdge(myPaths[indp][k], true); // at begin
                        }
                        else if (k == j)
                        {
                            if (myPaths[indp][k - 1].ShapeType() == ShapeType.Vertex)
                            {
                                continue;
                            }
                            aTangent = TangentOfEdge(myPaths[indp][k - 1], false); // at end
                        }
                        else
                        {
                            if (myPaths[indp][k - 1].ShapeType() == ShapeType.Vertex || myPaths[indp][k].ShapeType() == ShapeType.Vertex)
                            {
                                continue;
                            }
                            Vector3D Tangent1 = TangentOfEdge(myPaths[indp][k - 1], false); // at end
                            Vector3D Tangent2 = TangentOfEdge(myPaths[indp][k], true); // at begin
                            aTangent = Tangent1 + Tangent2;
                        }
                        aTangent.Normalize();
                        Point3D aPnt = new(aTangent.XYZ());
                        PntSeq.Append(aPnt);
                    }
                    NCollection_Array1<Point3D> PntArray = new(1, PntSeq.Length());
                    for (int ip = 1; ip <= PntSeq.Length(); ip++)
                    {
                        PntArray[ip] = PntSeq[ip];
                    }
                    global::Kerf.Engine.Geometry.Library.GeomLib.Inertia(PntArray, out Point3D theBary, out _, out _, out _, out _, out _);
                    Vector3D theTangent = new(theBary.XYZ());
                    theTangents[k - i + 1] = theTangent;
                }
                theFlags.Init(true);

                GeomAPI_Interpolate Interpol = new(thePoints, false, LinTol);
                Interpol.Load(theTangents, theFlags);
                Interpol.Perform();
                // (OCCT writes "Interpolation failed" to std::cout here when it fails; Curve() then throws.)
                Curve3D InterCurve = Interpol.Curve();
                MidEdges[i] = new BRepLib_MakeEdge(InterCurve).InternalEdge();
                i = j;
            }
        }

        BRepLib_MakeWire MakeFinalWire = new();
        for (i = 1; i < NbSecFaces; i++)
        {
            if (!MidEdges[i].IsNull)
            {
                MakeFinalWire.Add(ShapeCasts.Edge(MidEdges[i]));
            }
        }

        myShape = MakeFinalWire.InternalWire();

        Done();
    }
}
