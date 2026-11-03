// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_MiddlePath.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_MiddlePath.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2012-2014 OPEN CASCADE SAS; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Curves;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Construction.Values;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Healing.Upgrade;
using Kerf.Engine.Measurement;
using Kerf.Engine.Measurement.Distance;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using Schematiq.Collections;
using static Kerf.Engine.Measurement.Distance.SupportType;
using static Kerf.Geometry.CurveKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Construction.Api;

/// <summary>
/// The middle path of a pipe-like shape: the wire through the centres of mass of the sections between a start and an
/// end section (wires or faces of the shape).
/// </summary>
/// <remarks>
/// It only reads the shape: ShapeUpgrade_UnifySameDomain works on a copy of a frozen shape (decision 928), and every
/// edge it builds is new. The result is frozen when returned (design 4.2).
/// </remarks>
public class MiddlePath : MakeShape
{
    private readonly Shape _initialShape;
    private readonly Wire _startWire;
    private readonly Wire _endWire;
    private readonly bool _closedSection;
    private readonly bool _closedRing;
    private readonly NCollection_Map<Shape> _startWireEdges = new(ShapeHasher.Instance);
    private readonly NCollection_Map<Shape> _endWireEdges = new(ShapeHasher.Instance);
    private readonly NCollection_Sequence<NCollection_Sequence<Shape>> _paths = new();

    private static bool IsLinear(in Edge anEdge, out Line3D aLine)
    {
        aLine = new Line3D();
        Curve3D curve = Tool.Curve(anEdge, out _, out _)!;
        if (curve.GetType() == typeof(TrimmedCurve3D))
        {
            curve = ((TrimmedCurve3D)curve).BasisCurve();
        }

        Point3D pnt1, pnt2;
        if (curve is LineCurve3D geomLine)
        {
            aLine = geomLine.Lin();
            return true;
        }
        else if (curve is BezierCurve3D theBezier)
        {
            if (theBezier.PoleCount == 2)
            {
                pnt1 = theBezier.Pole(1);
                pnt2 = theBezier.Pole(2);
                aLine = new MakeLin(pnt1, pnt2).Value();
                return true;
            }
        }
        else if (curve is BSplineCurve3D theBSpline)
        {
            if (theBSpline.PoleCount == 2)
            {
                pnt1 = theBSpline.Pole(1);
                pnt2 = theBSpline.Pole(2);
                aLine = new MakeLin(pnt1, pnt2).Value();
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

        EdgeCurveAdaptor bAcurve = new(anEdge);
        return bAcurve.GetType();
    }

    private static Vector3D TangentOfEdge(in Shape aShape, bool onFirst)
    {
        Edge edge = ShapeCasts.Edge(aShape);
        Orientation or = edge.Orientation();

        Curve3D curve = Tool.Curve(edge, out double fpar, out double lpar)!;
        double thePar;
        if (onFirst)
        {
            thePar = (or == Forward) ? fpar : lpar;
        }
        else
        {
            thePar = (or == Forward) ? lpar : fpar;
        }

        curve.D1(thePar, out _, out Vector3D theTangent);
        if (or == Reversed)
        {
            theTangent = theTangent.Reversed();
        }

        return theTangent;
    }

    private static bool IsValidEdge(in Edge edge, in Face face)
    {
        Exploration.Vertices(edge, out Vertex v1, out Vertex v2);

        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        int i;

        Explorer explo = new(face, ShapeType.Edge);
        for (; explo.More(); explo.Next())
        {
            Shape anEdge = explo.Current();
            DistanceShapeShape distMini = new(edge, anEdge);
            if (distMini.Value() <= tol)
            {
                for (i = 1; i <= distMini.SolutionCount; i++)
                {
                    SupportType theType = distMini.SupportTypeShape2(i);
                    if (theType == SupportType.IsOnEdge)
                    {
                        return false;
                    }
                    // theType is "IsVertex"
                    Shape vertex = distMini.SupportOnShape2(i);
                    if (!(vertex.IsSame(v1) || vertex.IsSame(v2)))
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

    private static Wire GetUnifiedWire(in Wire wire, UnifySameDomain unifier)
    {
        LibraryMakeWire wMaker = new();
        WireExplorer wexp = new(wire);
        NCollection_Map<Shape> generatedEdges = new(ShapeHasher.Instance);
        for (; wexp.More(); wexp.Next())
        {
            Shape edge = wexp.Current();
            ISchematiqList<Shape> ls = unifier.History()!.Modified(edge);
            if (ls.Count != 0)
            {
                foreach (Shape shape in ls)
                {
                    // wire shouldn't contain duplicated generated edges
                    if (generatedEdges.Add(shape))
                    {
                        wMaker.Add(ShapeCasts.Edge(shape));
                    }
                }
            }
            else
            {
                // no change, put original edge
                wMaker.Add(ShapeCasts.Edge(edge));
            }
        }
        return wMaker.Wire();
    }

    //=================================================================================================

    /// <summary>
    /// The middle path of <paramref name="aShape"/> (a shell or a solid) from <paramref name="startShape"/> to
    /// <paramref name="endShape"/>, each a wire or a face of it (the face's outer wire).
    /// </summary>
    public MiddlePath(in Shape aShape, in Shape startShape, in Shape endShape)
    {
        UnifySameDomain unifier = new(aShape);
        unifier.Build();
        _initialShape = unifier.Shape();

        Wire startWire, endWire;
        if (startShape.ShapeType() == ShapeType.Face)
        {
            Face startFace = ShapeCasts.Face(startShape);
            startWire = BrepTools.OuterWire(startFace);
        }
        else
        {
            startWire = ShapeCasts.Wire(startShape);
        }

        if (endShape.ShapeType() == ShapeType.Face)
        {
            Face endFace = ShapeCasts.Face(endShape);
            endWire = BrepTools.OuterWire(endFace);
        }
        else
        {
            endWire = ShapeCasts.Wire(endShape);
        }

        _startWire = GetUnifiedWire(startWire, unifier);
        _endWire = GetUnifiedWire(endWire, unifier);

        _closedSection = _startWire.Closed();
        _closedRing = _startWire.IsSame(_endWire);
        _ = _closedRing;
    }

    //=================================================================================================

    /// <summary>Builds the middle path.</summary>
    public override void Build(ProgressRange range = default)
    {
        NCollection_List<Shape>.Iterator itl = new();

        NCollection_Sequence<Shape> startVertices = new();
        NCollection_Map<Shape> endVertices = new(ShapeHasher.Instance);
        NCollection_Map<Shape> endEdges = new(ShapeHasher.Instance);
        NCollection_Sequence<NCollection_Sequence<Shape>> sectionsEdges = new();

        WireExplorer wexp = new(_startWire);
        NCollection_Sequence<Shape> edgeSeq = new();
        for (; wexp.More(); wexp.Next())
        {
            startVertices.Append(wexp.CurrentVertex());
            edgeSeq.Append(wexp.Current());
        }
        if (!_closedSection)
        {
            startVertices.Append(wexp.CurrentVertex());
        }
        sectionsEdges.Append(new NCollection_Sequence<Shape>(edgeSeq));

        for (wexp.Init(_endWire); wexp.More(); wexp.Next())
        {
            endVertices.Add(wexp.CurrentVertex());
            endEdges.Add(wexp.Current());
        }
        if (!_closedSection)
        {
            endVertices.Add(wexp.CurrentVertex());
        }

        Iterator itw = new(_startWire);
        for (; itw.More(); itw.Next())
        {
            _startWireEdges.Add(itw.Value());
        }
        for (itw.Initialize(_endWire); itw.More(); itw.Next())
        {
            _endWireEdges.Add(itw.Value());
        }

        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> vEmap = new(ShapeHasher.Instance);
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> eFmap = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(_initialShape, ShapeType.Vertex, ShapeType.Edge, vEmap);
        Exploration.MapShapesAndAncestors(_initialShape, ShapeType.Edge, ShapeType.Face, eFmap);

        NCollection_Map<Shape> curVertices = new(ShapeHasher.Instance);

        int i, j, k;
        Edge edge = new();
        Vertex v1, v2, nextVertex;
        // Initialization of <myPaths>
        for (i = 1; i <= startVertices.Length(); i++)
        {
            NCollection_Sequence<Shape> edges = new();
            NCollection_List<Shape> le = vEmap.FindFromKey(startVertices[i]);
            for (itl.Initialize(le); itl.More(); itl.Next())
            {
                edge = ShapeCasts.Edge(itl.Value());
                if (!_startWireEdges.Contains(edge))
                {
                    Exploration.Vertices(edge, out v1, out v2, true);
                    if (v1.IsSame(startVertices[i]))
                    {
                        curVertices.Add(v2);
                    }
                    else
                    {
                        edge.Reverse();
                        curVertices.Add(v1);
                    }
                    edges.Append(edge);
                    break;
                }
            }
            if (!edges.IsEmpty())
            {
                _paths.Append(edges);
            }
            else
            {
                return;
            }
        }

        // Filling of "myPaths"
        NCollection_List<Shape> nextVertices = new();
        for (;;)
        {
            for (i = 1; i <= _paths.Length(); i++)
            {
                Shape theShape = _paths[i].Last();
                Edge theEdge;
                Vertex theVertex;
                if (theShape.ShapeType() == ShapeType.Edge)
                {
                    theEdge = ShapeCasts.Edge(theShape);
                    theVertex = Exploration.LastVertex(theEdge, true);
                }
                else // last segment of path was punctual
                {
                    theEdge = ShapeCasts.Edge(_paths[i][_paths[i].Length() - 1]);
                    theVertex = ShapeCasts.Vertex(theShape);
                }

                if (endVertices.Contains(theVertex))
                {
                    continue;
                }
                NCollection_List<Shape> le = vEmap.FindFromKey(theVertex);
                NCollection_Map<Shape> nextEdgeCandidates = new(ShapeHasher.Instance);
                for (itl.Initialize(le); itl.More(); itl.Next())
                {
                    edge = ShapeCasts.Edge(itl.Value());
                    if (edge.IsSame(theEdge))
                    {
                        continue;
                    }
                    Exploration.Vertices(edge, out v1, out v2, true);
                    if (v1.IsSame(theVertex))
                    {
                        nextVertex = v2;
                    }
                    else
                    {
                        edge.Reverse();
                        nextVertex = v1;
                    }
                    if (!curVertices.Contains(nextVertex))
                    {
                        nextEdgeCandidates.Add(edge);
                    }
                }
                if (!nextEdgeCandidates.IsEmpty())
                {
                    if (nextEdgeCandidates.Extent() > 1)
                    {
                        _paths[i].Append(theVertex); // punctual segment of path
                    }
                    else
                    {
                        foreach (Shape key in nextEdgeCandidates)
                        {
                            edge = ShapeCasts.Edge(key);
                        }
                        _paths[i].Append(edge);
                        nextVertex = Exploration.LastVertex(edge, true);
                        nextVertices.Append(nextVertex);
                    }
                }
            }
            if (nextVertices.IsEmpty())
            {
                break;
            }
            for (itl.Initialize(nextVertices); itl.More(); itl.Next())
            {
                curVertices.Add(itl.Value());
            }
            nextVertices.Clear();
        }

        // Building of set of sections
        int nbE = edgeSeq.Length();
        int nbPaths = _paths.Length();
        int nbVer = _paths.Length();
        if (_closedSection)
        {
            nbVer++;
        }
        i = 1;
        for (;;)
        {
            for (j = 1; j <= edgeSeq.Length(); j++)
            {
                edgeSeq[j].Nullify();
            }

            bool toInsertVertex = false;

            for (j = 2; j <= nbVer; j++)
            {
                if (!edgeSeq[j - 1].IsNull)
                {
                    continue;
                }

                NCollection_Sequence<Shape> path1 = _paths[j - 1];
                NCollection_Sequence<Shape> path2 = _paths[(j <= nbPaths) ? j : 1];

                // for the end of initial shape
                if (path1.Length() < i)
                {
                    Edge aE1 = ShapeCasts.Edge(path1[i - 1]);
                    Shape lastVer = Exploration.LastVertex(aE1, true);
                    path1.Append(lastVer);
                }
                if (path2.Length() < i)
                {
                    Edge aE2 = ShapeCasts.Edge(path2[i - 1]);
                    Shape lastVer = Exploration.LastVertex(aE2, true);
                    path2.Append(lastVer);
                }
                //////////////////////////////

                if (toInsertVertex)
                {
                    if (path1[i].ShapeType() == ShapeType.Edge)
                    {
                        Edge aE1 = ShapeCasts.Edge(path1[i]);
                        Shape fver = Exploration.FirstVertex(aE1, true);
                        path1.InsertBefore(i, fver);
                    }
                    if (path2[i].ShapeType() == ShapeType.Edge)
                    {
                        Edge aE2 = ShapeCasts.Edge(path2[i]);
                        Shape fver = Exploration.FirstVertex(aE2, true);
                        path2.InsertBefore(i, fver);
                    }
                    toInsertVertex = false;
                }

                Edge e1 = new(), e2 = new();
                if (path1[i].ShapeType() == ShapeType.Edge)
                {
                    e1 = ShapeCasts.Edge(path1[i]);
                }
                if (path2[i].ShapeType() == ShapeType.Edge)
                {
                    e2 = ShapeCasts.Edge(path2[i]);
                }
                Edge e12 = ShapeCasts.Edge(sectionsEdges[i][j - 1]);
                // Find the face on which (E1 or E2) and E12 lie
                Shape e1orE2 = (e1.IsNull) ? e2 : e1;
                if (e1orE2.IsNull) // both E1 and E2 are vertices =>
                {
                    edgeSeq[j - 1] = e12; // => proper edge is the edge of previous section between them
                    continue;
                }
                NCollection_List<Shape> lf = eFmap.FindFromKey(e1orE2);
                Face theFace = new();
                for (itl.Initialize(lf); itl.More(); itl.Next())
                {
                    Shape face = itl.Value();
                    NCollection_List<Shape> lf2 = eFmap.FindFromKey(e12);
                    NCollection_List<Shape>.Iterator itl2 = new(lf2);
                    for (; itl2.More(); itl2.Next())
                    {
                        Shape face2 = itl2.Value();
                        if (face.IsSame(face2))
                        {
                            theFace = ShapeCasts.Face(face);
                            break;
                        }
                    }
                    if (!theFace.IsNull)
                    {
                        break;
                    }
                }

                Vertex prevVertex = (e1.IsNull) ? ShapeCasts.Vertex(path1[i]) : Exploration.LastVertex(e1, true);
                Vertex curVertex = (e2.IsNull) ? ShapeCasts.Vertex(path2[i]) : Exploration.LastVertex(e2, true);

                Edge properEdge = new();
                NCollection_List<Shape> le = vEmap.FindFromKey(prevVertex);
                NCollection_IndexedMap<Shape> edgesOfTheFace = new(ShapeHasher.Instance);
                Exploration.MapShapes(theFace, ShapeType.Edge, edgesOfTheFace);
                for (itl.Initialize(le); itl.More(); itl.Next())
                {
                    edge = ShapeCasts.Edge(itl.Value());
                    Exploration.Vertices(edge, out v1, out v2);
                    if (((v1.IsSame(prevVertex) && v2.IsSame(curVertex)) || (v1.IsSame(curVertex) && v2.IsSame(prevVertex)))
                        && edgesOfTheFace.Contains(edge) && // this condition is for a section of two edges
                        !edge.IsSame(e1))                   // the last condition is for torus-like shape
                    {
                        properEdge = edge;
                        break;
                    }
                }

                if ((path1[i]).ShapeType() == ShapeType.Vertex && (path2[i]).ShapeType() == ShapeType.Vertex)
                {
                    edgeSeq[j - 1] = properEdge;
                    continue;
                }

                Vertex prevPrevVer = (e1.IsNull) ? prevVertex : Exploration.FirstVertex(e1, true);
                Vertex prevCurVer = (e2.IsNull) ? curVertex : Exploration.FirstVertex(e2, true);
                if (properEdge.IsNull) // no connection between these two vertices
                {
                    // Find the face on which E1, E2 and E12 lie
                    // ToInsertVertex = false;
                    NCollection_List<Shape> listOneFace = new();
                    listOneFace.Append(theFace);

                    if (e1.IsNull || e2.IsNull)
                    {
                        if (e1.IsNull)
                        {
                            e1 = ShapeCasts.Edge(path1[i - 1]);
                        }
                        if (e2.IsNull)
                        {
                            e2 = ShapeCasts.Edge(path2[i - 1]);
                        }
                        double lastPar1, lastPar2;
                        Curve2D pCurve1 = Tool.CurveOnSurface(e1, theFace, out double fpar1, out double lpar1)!;
                        Curve2D pCurve2 = Tool.CurveOnSurface(e2, theFace, out double fpar2, out double lpar2)!;
                        if (e1.Orientation() == Forward)
                        {
                            lastPar1 = lpar1;
                        }
                        else
                        {
                            lastPar1 = fpar1;
                        }
                        if (e2.Orientation() == Forward)
                        {
                            lastPar2 = lpar2;
                        }
                        else
                        {
                            lastPar2 = fpar2;
                        }
                        Point2D firstPnt2d = pCurve1.Value(lastPar1);
                        Point2D lastPnt2d = pCurve2.Value(lastPar2);
                        Surface theSurf = Tool.Surface(theFace)!;
                        LineCurve2D theLine = new MakeLine2d(firstPnt2d, lastPnt2d).Value();
                        double len_ne = firstPnt2d.Distance(lastPnt2d);
                        Edge newEdge = new LibraryMakeEdge(theLine, theSurf, prevVertex, curVertex, 0.0, len_ne).InternalEdge();
                        BrepLibrary.BuildCurve3d(newEdge);
                        edgeSeq[j - 1] = newEdge;
                        eFmap.Add(newEdge, listOneFace);
                    }
                    else // E1 is edge
                    {
                        // Extract points 2d
                        double firstPar1, lastPar1, firstPar2, lastPar2;
                        Curve2D pCurve1 = Tool.CurveOnSurface(e1, theFace, out double fpar1, out double lpar1)!;
                        Curve2D pCurve2 = Tool.CurveOnSurface(e2, theFace, out double fpar2, out double lpar2)!;
                        if (e1.Orientation() == Forward)
                        {
                            firstPar1 = fpar1;
                            lastPar1 = lpar1;
                        }
                        else
                        {
                            firstPar1 = lpar1;
                            lastPar1 = fpar1;
                        }
                        if (e2.Orientation() == Forward)
                        {
                            firstPar2 = fpar2;
                            lastPar2 = lpar2;
                        }
                        else
                        {
                            firstPar2 = lpar2;
                            lastPar2 = fpar2;
                        }
                        Point2D firstPnt2d = pCurve1.Value(lastPar1);
                        Point2D lastPnt2d = pCurve2.Value(lastPar2);
                        Surface theSurf = Tool.Surface(theFace)!;
                        LineCurve2D theLine = new MakeLine2d(firstPnt2d, lastPnt2d).Value();
                        double len_ne = firstPnt2d.Distance(lastPnt2d);
                        Edge newEdge = new LibraryMakeEdge(theLine, theSurf, prevVertex, curVertex, 0.0, len_ne).InternalEdge();
                        BrepLibrary.BuildCurve3d(newEdge);
                        Point2D prevFirstPnt2d = pCurve1.Value(firstPar1);
                        Point2D prevLastPnt2d = pCurve2.Value(firstPar2);
                        LineCurve2D line1 = new MakeLine2d(prevFirstPnt2d, lastPnt2d).Value();
                        LineCurve2D line2 = new MakeLine2d(firstPnt2d, prevLastPnt2d).Value();
                        double len_ne1 = prevFirstPnt2d.Distance(lastPnt2d);
                        Edge newEdge1 = new LibraryMakeEdge(line1, theSurf, prevPrevVer, curVertex, 0.0, len_ne1).InternalEdge();
                        BrepLibrary.BuildCurve3d(newEdge1);
                        double len_ne2 = firstPnt2d.Distance(prevLastPnt2d);
                        Edge newEdge2 = new LibraryMakeEdge(line2, theSurf, prevVertex, prevCurVer, 0.0, len_ne2).InternalEdge();
                        BrepLibrary.BuildCurve3d(newEdge2);
                        bool good_ne = IsValidEdge(newEdge, theFace);
                        bool good_ne1 = IsValidEdge(newEdge1, theFace);

                        CurveKind type_E1 = TypeOfEdge(e1);
                        CurveKind type_E2 = TypeOfEdge(e2);

                        int chooseEdge = 0;

                        if (!good_ne || type_E1 != type_E2)
                        {
                            if (type_E1 == type_E2) //! good_ne
                            {
                                if (good_ne1)
                                {
                                    chooseEdge = 1;
                                }
                                else
                                {
                                    chooseEdge = 2;
                                }
                            }
                            else // types are different
                            {
                                if (type_E1 == Line)
                                {
                                    chooseEdge = 1;
                                }
                                else if (type_E2 == Line)
                                {
                                    chooseEdge = 2;
                                }
                                else // to be developed later...
                                {
                                }
                            }
                        }

                        if (chooseEdge == 0)
                        {
                            edgeSeq[j - 1] = newEdge;
                            eFmap.Add(newEdge, listOneFace);
                        }
                        else if (chooseEdge == 1)
                        {
                            edgeSeq[j - 1] = newEdge1;
                            eFmap.Add(newEdge1, listOneFace);
                            for (k = 1; k < j - 1; k++)
                            {
                                edgeSeq[k].Nullify();
                            }
                            for (k = 1; k <= j - 1; k++)
                            {
                                Edge lastEdge = ShapeCasts.Edge(_paths[k][i]);
                                Shape vertexAsEdge = Exploration.FirstVertex(lastEdge, true);
                                _paths[k].InsertBefore(i, vertexAsEdge);
                            }
                            j = 1; // start from beginning
                        }
                        else if (chooseEdge == 2)
                        {
                            edgeSeq[j - 1] = newEdge2;
                            eFmap.Add(newEdge2, listOneFace);
                            toInsertVertex = true;
                        }
                    } // else //E1 is edge
                } // if (ProperEdge.IsNull())
                else // connecting edge exists
                {
                    edgeSeq[j - 1] = properEdge;
                }
            } // for (j = 2; j <= NbVer; j++)
            sectionsEdges.Append(new NCollection_Sequence<Shape>(edgeSeq));

            // check for exit from for(;;)
            int nbEndEdges = 0;
            for (j = 1; j <= edgeSeq.Length(); j++)
            {
                if (endEdges.Contains(edgeSeq[j]))
                {
                    nbEndEdges++;
                }
            }
            if (nbEndEdges == nbE)
            {
                break;
            }

            i++;
        } // for (;;)

        // final phase: building of middle path
        int nbSecFaces = sectionsEdges.Length();
        NCollection_Array1<Shape> secFaces = new(1, nbSecFaces);
        for (i = 1; i <= nbSecFaces; i++)
        {
            LibraryMakeWire mw = new();
            for (j = 1; j <= nbE; j++)
            {
                edge = ShapeCasts.Edge(sectionsEdges[i][j]);
                mw.Add(edge);
            }
            if (!_closedSection)
            {
                Exploration.Vertices(mw.InternalWire(), out v1, out v2);
                edge = new LibraryMakeEdge(v2, v1).InternalEdge();
                mw.Add(edge);
            }
            Wire wire = mw.InternalWire();
            LibraryMakeFace mf = new(wire, true); // Only plane
            if (mf.IsDone)
            {
                secFaces[i] = mf.InternalFace();
            }
            else
            {
                secFaces[i] = wire;
            }
        }

        NCollection_Array1<Point3D> centers = new(1, nbSecFaces);
        for (i = 1; i <= nbSecFaces; i++)
        {
            Properties properties = new();
            if (secFaces[i].ShapeType() == ShapeType.Face)
            {
                MassPropertiesBuilder.SurfaceProperties(secFaces[i], properties);
            }
            else
            { // wire
                MassPropertiesBuilder.LinearProperties(secFaces[i], properties);
            }

            centers[i] = properties.CentreOfMass();
        }

        NCollection_Array1<Shape> midEdges = new(1, nbSecFaces - 1);
        double linTol = 1.0e-5;
        double angTol = 1.0e-7;
        Point3D pnt1, pnt2;
        for (i = 1; i < nbSecFaces; i++)
        {
            CurveKind typeOfMidEdge = OtherCurve;
            for (j = 1; j <= _paths.Length(); j++)
            {
                Shape shape = _paths[j][i];
                if (shape.ShapeType() == ShapeType.Vertex)
                {
                    typeOfMidEdge = OtherCurve;
                    break;
                }
                edge = ShapeCasts.Edge(shape);
                CurveKind type = TypeOfEdge(edge);
                if (j == 1)
                {
                    typeOfMidEdge = type;
                }
                else
                {
                    if (type != typeOfMidEdge)
                    {
                        typeOfMidEdge = OtherCurve;
                        break;
                    }
                }
            }
            if (typeOfMidEdge == Line)
            {
                midEdges[i] = new LibraryMakeEdge(centers[i], centers[i + 1]).InternalEdge();
            }
            else if (typeOfMidEdge == Circle)
            {
                Axis3D theAxis = new();
                Direction3D theDir1 = new(), theDir2 = new();
                double theAngle = 0.0;
                Vector3D theTangent = new();
                bool similarArcs = true;
                for (j = 1; j <= _paths.Length(); j++)
                {
                    edge = ShapeCasts.Edge(_paths[j][i]);
                    Curve3D curve = Tool.Curve(edge, out double fpar, out double lpar)!;
                    if (curve.GetType() == typeof(TrimmedCurve3D))
                    {
                        curve = ((TrimmedCurve3D)curve).BasisCurve();
                    }
                    pnt1 = curve.Value(fpar);
                    pnt2 = curve.Value(lpar);
                    CircleCurve3D circle = (CircleCurve3D)curve;
                    Circle3D circ = circle.Circ();
                    if (j == 1)
                    {
                        theAxis = circ.Axis();
                        theDir1 = new Direction3D(new Vector3D(circ.Location(), pnt1));
                        theDir2 = new Direction3D(new Vector3D(circ.Location(), pnt2));
                        theAngle = lpar - fpar;
                        double theParam = (edge.Orientation() == Forward) ? fpar : lpar;
                        curve.D1(theParam, out pnt1, out theTangent);
                        if (edge.Orientation() == Reversed)
                        {
                            theTangent = theTangent.Reversed();
                        }
                    }
                    else
                    {
                        Axis3D axis = circ.Axis();
                        Line3D lin = new(axis);
                        if (!lin.Contains(theAxis.Location(), linTol) || !axis.IsParallel(theAxis, angTol))
                        {
                            similarArcs = false;
                            break;
                        }
                        Direction3D dir1 = new(new Vector3D(circ.Location(), pnt1));
                        Direction3D dir2 = new(new Vector3D(circ.Location(), pnt2));
                        if (!((dir1.IsEqual(theDir1, angTol) && dir2.IsEqual(theDir2, angTol))
                              || (dir1.IsEqual(theDir2, angTol) && dir2.IsEqual(theDir1, angTol))))
                        {
                            similarArcs = false;
                            break;
                        }
                    }
                }
                if (similarArcs)
                {
                    Xyz axisLoc = theAxis.Location().XYZ();
                    Xyz axisDir = theAxis.Direction().XYZ();
                    double parameter = (centers[i].XYZ() - axisLoc) * axisDir;
                    Point3D theCenterOfCirc = new(axisLoc + parameter * axisDir);

                    Vector3D vec1 = new(theCenterOfCirc, centers[i]);
                    Vector3D vec2 = new(theCenterOfCirc, centers[i + 1]);

                    double angle = vec1.AngleWithRef(vec2, new Vector3D(theAxis.Direction()));
                    if (angle < 0.0)
                    {
                        angle += 2.0 * M_PI;
                    }
                    if (Math.Abs(angle - theAngle) > angTol)
                    {
                        theAxis = theAxis.Reversed();
                    }
                    Frame3D theAx2 = new(theCenterOfCirc, theAxis.Direction(), new Direction3D(vec1));
                    CircleCurve3D theCircle = new MakeCircle(theAx2, vec1.Magnitude()).Value();
                    theCircle.D1(0.0, out pnt1, out Vector3D tangent);
                    if (tangent * theTangent < 0.0)
                    {
                        theAxis = theAxis.Reversed();
                        theAx2 = new Frame3D(theCenterOfCirc, theAxis.Direction(), new Direction3D(vec1));
                        theCircle = new MakeCircle(theAx2, vec1.Magnitude()).Value();
                    }
                    LibraryMakeEdge me = new(theCircle, 0.0, theAngle);
                    me.Build();

                    midEdges[i] = me.IsDone ? me.InternalEdge() : new Edge();
                }
            }
        }

        // Build missed edges
        for (i = 1; i < nbSecFaces; i++)
        {
            if (midEdges[i].IsNull)
            {
                for (j = i + 1; j < nbSecFaces; j++)
                {
                    if (!midEdges[j].IsNull)
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
                    thePoints.SetValue(k - i + 1, centers[k]);
                }
                for (k = i; k <= j; k++)
                {
                    NCollection_Sequence<Point3D> pntSeq = new();
                    for (int indp = 1; indp <= _paths.Length(); indp++)
                    {
                        Vector3D tangent;
                        if (k == i)
                        {
                            if (_paths[indp][k].ShapeType() == ShapeType.Vertex)
                            {
                                continue;
                            }
                            tangent = TangentOfEdge(_paths[indp][k], true); // at begin
                        }
                        else if (k == j)
                        {
                            if (_paths[indp][k - 1].ShapeType() == ShapeType.Vertex)
                            {
                                continue;
                            }
                            tangent = TangentOfEdge(_paths[indp][k - 1], false); // at end
                        }
                        else
                        {
                            if (_paths[indp][k - 1].ShapeType() == ShapeType.Vertex || _paths[indp][k].ShapeType() == ShapeType.Vertex)
                            {
                                continue;
                            }
                            Vector3D tangent1 = TangentOfEdge(_paths[indp][k - 1], false); // at end
                            Vector3D tangent2 = TangentOfEdge(_paths[indp][k], true); // at begin
                            tangent = tangent1 + tangent2;
                        }
                        tangent = tangent.Normalized();
                        Point3D pnt = new(tangent.XYZ());
                        pntSeq.Append(pnt);
                    }
                    NCollection_Array1<Point3D> pntArray = new(1, pntSeq.Length());
                    for (int ip = 1; ip <= pntSeq.Length(); ip++)
                    {
                        pntArray[ip] = pntSeq[ip];
                    }
                    global::Kerf.Engine.Geometry.Library.GeomLib.Inertia(pntArray, out Point3D theBary, out _, out _, out _, out _, out _);
                    Vector3D theTangent = new(theBary.XYZ());
                    theTangents[k - i + 1] = theTangent;
                }
                theFlags.Init(true);

                ApiInterpolate interpol = new(thePoints, false, linTol);
                interpol.Load(theTangents, theFlags);
                interpol.Perform();
                // (OCCT writes "Interpolation failed" to std::cout here when it fails; Curve() then throws.)
                Curve3D interCurve = interpol.Curve();
                midEdges[i] = new LibraryMakeEdge(interCurve).InternalEdge();
                i = j;
            }
        }

        LibraryMakeWire makeFinalWire = new();
        for (i = 1; i < nbSecFaces; i++)
        {
            if (!midEdges[i].IsNull)
            {
                makeFinalWire.Add(ShapeCasts.Edge(midEdges[i]));
            }
        }

        _shape = makeFinalWire.InternalWire();

        Done();
    }
}
