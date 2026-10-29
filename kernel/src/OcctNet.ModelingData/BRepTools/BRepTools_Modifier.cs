// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_Modifier.hxx
// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_Modifier.lxx
// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_Modifier.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Applies a <see cref="BRepTools_Modification"/> to a shape: rebuilds every sub-shape whose geometry, or one of
/// whose sub-shapes, the modification changes, and shares the others.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21). The input shape is never written into: the rebuilt shapes are new. OCCT's
/// "mutable input" mode, which updates the input's vertices in place, is internal (<c>BRepTools_Modifier(true)</c>,
/// <c>SetMutableInput</c>), since every shape a caller holds is frozen and would throw.
/// </para>
/// <para>
/// <see cref="ModifiedShape"/> is a publication point (docs/phase3-design.md, section 4.2): it freezes the shape it
/// returns. Ported code that goes on building on the result reads it through <see cref="InternalModifiedShape"/>.
/// </para>
/// </remarks>
public sealed class BRepTools_Modifier
{
    private struct NewCurveInfo
    {
        public Geom_Curve? myCurve;
        public TopLoc_Location myLoc;
        public double myToler;
    }

    private struct NewSurfaceInfo
    {
        public Geom_Surface? mySurface;
        public TopLoc_Location myLoc;
        public double myToler;
        public bool myRevWires;
        public bool myRevFace;
    }

    private readonly NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> myMap = new(TopTools_ShapeMapHasher.Instance);
    private TopoDS_Shape myShape = new();
    private bool myDone;
    private readonly NCollection_DataMap<TopoDS_Shape, NewCurveInfo> myNCInfo = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_DataMap<TopoDS_Shape, NewSurfaceInfo> myNSInfo = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_Map<TopoDS_Shape> myNonUpdFace = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_Map<TopoDS_Shape> myHasNewGeom = new(TopTools_ShapeMapHasher.Instance);
    private bool myMutableInput;

    /// <summary>An empty modifier.</summary>
    public BRepTools_Modifier()
        : this(false)
    {
    }

    /// <summary>An empty modifier; with <paramref name="theMutableInput"/> the input's vertices may be updated in place.</summary>
    internal BRepTools_Modifier(bool theMutableInput)
    {
        myDone = false;
        myMutableInput = theMutableInput;
    }

    /// <summary>A modifier on the shape.</summary>
    public BRepTools_Modifier(in TopoDS_Shape S)
    {
        myShape = S;
        myDone = false;
        myMutableInput = false;
        Put(S);
    }

    /// <summary>A modifier on the shape, performing the modification.</summary>
    public BRepTools_Modifier(in TopoDS_Shape S, BRepTools_Modification M)
    {
        myShape = S;
        myDone = false;
        myMutableInput = false;
        Put(S);
        Perform(M);
    }

    /// <summary>Initializes the modifier with the shape.</summary>
    public void Init(in TopoDS_Shape S)
    {
        myShape = S;
        myDone = false;
        Put(S);
    }

    /// <summary>Performs the modification.</summary>
    /// <exception cref="Standard_NullObject">The modifier has no shape.</exception>
    public void Perform(BRepTools_Modification M, Message_ProgressRange theProgress = default)
    {
        if (myShape.IsNull())
        {
            throw new Standard_NullObject();
        }

        using var aPS = new Message_ProgressScope(theProgress, "Converting Shape", 2);

        var aMVE = new NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
        var aMEF = new NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(myShape, TopAbs_VERTEX, TopAbs_EDGE, aMVE);
        TopExp.MapShapesAndAncestors(myShape, TopAbs_EDGE, TopAbs_FACE, aMEF);

        CreateNewVertices(aMVE, M);

        FillNewCurveInfo(aMEF, M);

        FillNewSurfaceInfo(M);

        if (!myMutableInput)
        {
            CreateOtherVertices(aMVE, aMEF, M);
        }

        Rebuild(myShape, M, out _, aPS.Next());

        if (!aPS.More())
        {
            // The processing was broken
            return;
        }

        if (myShape.ShapeType() == TopAbs_FACE)
        {
            if (myShape.Orientation() == TopAbs_REVERSED)
            {
                myMap[myShape].Reverse();
            }
            else
            {
                myMap[myShape].Orientation(myShape.Orientation());
            }
        }
        else
        {
            myMap[myShape].Orientation(myShape.Orientation());
        }

        // Update the continuities

        BRep_Builder aBB = new BRep_Builder();

        for (int ii = 1; ii <= aMEF.Extent(); ii++)
        {
            TopoDS_Edge CurE = TopoDS.Edge(aMEF.FindKey(ii));
            TopoDS_Edge NewE = TopoDS.Edge(myMap[CurE]);
            if (!CurE.IsSame(NewE))
            {
                var it = new NCollection_List<TopoDS_Shape>.Iterator();
                it.Initialize(aMEF.FindFromKey(CurE));
                TopoDS_Face F1 = new(), F2 = new();
                while (it.More() && F2.IsNull())
                {
                    if (F1.IsNull())
                    {
                        F1 = TopoDS.Face(it.Value());
                    }
                    else
                    {
                        F2 = TopoDS.Face(it.Value());
                    }
                    it.Next();
                }
                if (!F2.IsNull())
                {
                    TopoDS_Face newf1 = TopoDS.Face(myMap[F1]);
                    TopoDS_Face newf2 = TopoDS.Face(myMap[F2]);
                    GeomAbs_Shape Newcont = M.Continuity(CurE, F1, F2, NewE, newf1, newf2);
                    // OCCT.NET: frozen: an edge rebuilt by an earlier Perform (Init keeps the images, occt-issues.md)
                    // may have been published since; OCCT sets its continuity again, which changes nothing when it
                    // already has it
                    if (Newcont > GeomAbs_C0
                        && !(NewE.TShape()!.IsFrozen() && BRep_Tool.HasContinuity(NewE, newf1, newf2)
                             && BRep_Tool.Continuity(NewE, newf1, newf2) == Newcont))
                    {
                        aBB.Continuity(NewE, newf1, newf2, Newcont);
                    }
                }
            }
        }

        myDone = true;
    }

    /// <summary>True if the modification has been computed successfully.</summary>
    public bool IsDone() => myDone;

    /// <summary>True if the input shape may be modified (OCCT's mutable input mode).</summary>
    public bool IsMutableInput() => myMutableInput;

    /// <summary>Sets the mutable input mode: the input's vertices are then updated in place.</summary>
    internal void SetMutableInput(bool theMutableInput) => myMutableInput = theMutableInput;

    /// <summary>The shape that the sub-shape S became, frozen (a publication point).</summary>
    /// <exception cref="Standard_NoSuchObject">S isn't a sub-shape of the modified shape.</exception>
    public TopoDS_Shape ModifiedShape(in TopoDS_Shape S)
    {
        TopoDS_Shape aResult = InternalModifiedShape(S);
        aResult.TShape()?.FreezeGraph();
        return aResult;
    }

    /// <summary>The shape that the sub-shape S became, unfrozen if it was rebuilt: for ported code that goes on building on it.</summary>
    /// <exception cref="Standard_NoSuchObject">S isn't a sub-shape of the modified shape.</exception>
    internal TopoDS_Shape InternalModifiedShape(in TopoDS_Shape S)
    {
        if (!myMap.IsBound(S))
        {
            throw new Standard_NoSuchObject();
        }
        return myMap[S];
    }

    private void Put(in TopoDS_Shape S)
    {
        if (!myMap.IsBound(S))
        {
            myMap.Bind(S, new TopoDS_Shape());
            for (var theIterator = new TopoDS_Iterator(S, false); theIterator.More(); theIterator.Next())
            {
                Put(theIterator.Value());
            }
        }
    }

    private bool Rebuild(in TopoDS_Shape S, BRepTools_Modification M, out bool theNewGeom, Message_ProgressRange theProgress)
    {
        TopAbs_ShapeEnum ts = S.ShapeType();
        // OCCT.NET: a reference into the map, as OCCT's TopoDS_Shape& result = myMap(S). The map gains no keys while
        // rebuilding (Put bound every sub-shape), so the reference stays valid through the recursion.
        ref TopoDS_Shape result = ref myMap.ChangeFind(S);
        if (!result.IsNull())
        {
            theNewGeom = myHasNewGeom.Contains(S);
            return !S.IsSame(result);
        }
        bool rebuild = false, RevWires = false;
        TopAbs_Orientation ResOr = TopAbs_FORWARD;
        BRep_Builder B = new BRep_Builder();
        double tol = 0.0;
        bool No3DCurve = false; // in fact, if we have no
        // 3D geometry modification, it is necessary to test the existence of a 3D curve.

        // new geometry ?

        switch (ts)
        {
            case TopAbs_FACE:
            {
                rebuild = myNSInfo.IsBound(TopoDS.Face(S));
                if (rebuild)
                {
                    NewSurfaceInfo aNSinfo = myNSInfo[TopoDS.Face(S)];
                    RevWires = aNSinfo.myRevWires;
                    TopoDS_Face aResultF = TopoDS.Face(result);
                    B.MakeFace(ref aResultF,
                               aNSinfo.mySurface,
                               aNSinfo.myLoc.Predivided(S.Location()),
                               aNSinfo.myToler);
                    result = aResultF;
                    result.Location(S.Location(), false);
                    if (aNSinfo.myRevFace)
                    {
                        ResOr = TopAbs_REVERSED;
                    }
                    // set specifics flags of a Face
                    B.NaturalRestriction(TopoDS.Face(result), BRep_Tool.NaturalRestriction(TopoDS.Face(S)));
                }

                // update triangulation on the copied face
                if (M.NewTriangulation(TopoDS.Face(S), out Poly_Triangulation? aTriangulation))
                {
                    if (rebuild)
                    { // the copied face already exists => update it
                        B.UpdateFace(TopoDS.Face(result), aTriangulation);
                    }
                    else
                    { // create new face with bare triangulation
                        // OCCT: also when the modification keeps the face's surface (NurbsConvert on a B-spline or
                        // Bezier face returns false from NewSurface), so a meshed face loses its surface
                        // (occt-issues.md)
                        TopoDS_Face aResultF = TopoDS.Face(result);
                        B.MakeFace(ref aResultF, aTriangulation);
                        result = aResultF;
                        result.Location(S.Location(), false);
                    }
                    rebuild = true;
                }
            }
                break;

            case TopAbs_EDGE:
            {
                rebuild = myNCInfo.IsBound(TopoDS.Edge(S));
                if (rebuild)
                {
                    NewCurveInfo aNCinfo = myNCInfo[TopoDS.Edge(S)];
                    TopoDS_Edge aResultE = TopoDS.Edge(result);
                    if (aNCinfo.myCurve is null)
                    {
                        B.MakeEdge(ref aResultE);
                        B.Degenerated(aResultE, BRep_Tool.Degenerated(TopoDS.Edge(S)));
                        B.UpdateEdge(aResultE, aNCinfo.myToler); // OCC217
                        No3DCurve = true;
                    }
                    else
                    {
                        B.MakeEdge(ref aResultE,
                                   aNCinfo.myCurve,
                                   aNCinfo.myLoc.Predivided(S.Location()),
                                   aNCinfo.myToler);
                        No3DCurve = false;
                    }
                    result = aResultE;
                    result.Location(S.Location(), false);
                    //	result.Orientation(S.Orientation());

                    // set specifics flags of an Edge
                    B.SameParameter(TopoDS.Edge(result), BRep_Tool.SameParameter(TopoDS.Edge(S)));
                    B.SameRange(TopoDS.Edge(result), BRep_Tool.SameRange(TopoDS.Edge(S)));
                }

                // update polygonal structure on the edge
                if (M.NewPolygon(TopoDS.Edge(S), out Poly_Polygon3D? aPolygon))
                {
                    if (rebuild)
                    { // the copied edge already exists => update it
                        B.UpdateEdge(TopoDS.Edge(result), aPolygon, S.Location());
                    }
                    else
                    { // create new edge with bare polygon
                        TopoDS_Edge aResultE = TopoDS.Edge(result);
                        B.MakeEdge(ref aResultE, aPolygon);
                        result = aResultE;
                        result.Location(S.Location(), false);
                    }
                    rebuild = true;
                }
            }
                break;
            default:
                break;
        }

        // rebuild sub-shapes and test new sub-shape ?

        bool newgeom = rebuild;
        theNewGeom = rebuild;

        TopoDS_Iterator it = new TopoDS_Iterator();

        {
            int aShapeCount = 0;
            {
                for (it.Initialize(S, false); it.More(); it.Next())
                {
                    ++aShapeCount;
                }
            }

            using var aPS = new Message_ProgressScope(theProgress, "Converting SubShapes", aShapeCount);
            //
            for (it.Initialize(S, false); it.More() && aPS.More(); it.Next())
            {
                // always call Rebuild
                bool subrebuilt = Rebuild(it.Value(), M, out bool isSubNewGeom, aPS.Next());
                rebuild = subrebuilt || rebuild;
                theNewGeom = theNewGeom || isSubNewGeom;
            }
            if (!aPS.More())
            {
                // The processing was broken
                return false;
            }
        }
        if (theNewGeom)
        {
            myHasNewGeom.Add(S);
        }

        // make an empty copy
        if (rebuild && !newgeom)
        {
            result = S.EmptyCopied();
            result.Orientation(TopAbs_FORWARD);
        }

        // copy the sub-elements

        if (rebuild)
        {
            TopAbs_Orientation orient;
            for (it.Initialize(S, false); it.More(); it.Next())
            {
                orient = it.Value().Orientation();
                if (RevWires || myMap[it.Value()].Orientation() == TopAbs_REVERSED)
                {
                    orient = TopAbs.Reverse(orient);
                }
                B.Add(result, myMap[it.Value()].Oriented(orient));
            }

            if (ts == TopAbs_FACE)
            {
                // pcurves
                TopoDS_Face face = TopoDS.Face(S);
                TopAbs_Orientation fcor = face.Orientation();
                if (fcor != TopAbs_REVERSED)
                {
                    fcor = TopAbs_FORWARD;
                }

                var ex = new TopExp_Explorer(face.Oriented(fcor), TopAbs_EDGE);
                for (; ex.More(); ex.Next())
                {
                    TopoDS_Edge edge = TopoDS.Edge(ex.Current());

                    Geom2d_Curve? curve2d = null;
                    if (theNewGeom
                        && M.NewCurve2d(edge,
                                        face,
                                        TopoDS.Edge(myMap[ex.Current()]),
                                        TopoDS.Face(result),
                                        out curve2d,
                                        out tol))
                    {
                        // rem dub 16/09/97 : Make constant topology or not make at all.
                        // Do not make if CopySurface = 1
                        // Attention, TRUE sewing edges (ReallyClosed)
                        // stay even if  CopySurface is true.

                        // check that edge contains two pcurves on this surface:
                        // either it is true seam on the current face, or belongs to two faces
                        // built on that same surface (see OCC21772)
                        // Note: this check could be made separate method in BRepTools
                        bool isClosed = false;
                        if (BRep_Tool.IsClosed(edge, face))
                        {
                            isClosed = (!newgeom || BRepTools.IsReallyClosed(edge, face));
                            if (!isClosed)
                            {
                                TopoDS_Shape resface = (myMap.IsBound(face) ? myMap[face] : face);
                                if (resface.IsNull())
                                {
                                    resface = face;
                                }
                                Geom_Surface? aSurf = BRep_Tool.Surface(TopoDS.Face(resface), out TopLoc_Location aLoc);
                                // check other faces sharing the same surface
                                var aExpF = new TopExp_Explorer(myShape, TopAbs_FACE);
                                for (; aExpF.More() && !isClosed; aExpF.Next())
                                {
                                    TopoDS_Face anOther = TopoDS.Face(aExpF.Current());
                                    if (anOther.IsSame(face))
                                    {
                                        continue;
                                    }
                                    TopoDS_Shape resface2 = (myMap.IsBound(anOther) ? myMap[anOther] : anOther);
                                    if (resface2.IsNull())
                                    {
                                        resface2 = anOther;
                                    }
                                    Geom_Surface? anOtherSurf =
                                        BRep_Tool.Surface(TopoDS.Face(resface2), out TopLoc_Location anOtherLoc);
                                    if (ReferenceEquals(aSurf, anOtherSurf) && aLoc.IsEqual(anOtherLoc))
                                    {
                                        var aExpE = new TopExp_Explorer(anOther, TopAbs_EDGE);
                                        for (; aExpE.More() && !isClosed; aExpE.Next())
                                        {
                                            isClosed = edge.IsSame(aExpE.Current());
                                        }
                                    }
                                }
                            }
                        }
                        if (isClosed)
                        {
                            TopoDS_Edge CurE = TopoDS.Edge(myMap[edge]);
                            TopoDS_Shape aLocalResult = result;
                            aLocalResult.Orientation(TopAbs_FORWARD);
                            TopoDS_Face CurF = TopoDS.Face(aLocalResult);
                            Geom2d_Curve? curve2d1;
                            double f, l;
                            if ((!RevWires && fcor != edge.Orientation())
                                || (RevWires && fcor == edge.Orientation()))
                            {
                                CurE.Orientation(TopAbs_FORWARD);
                                curve2d1 = BRep_Tool.CurveOnSurface(CurE, CurF, out f, out l);
                                if (curve2d1 is null)
                                {
                                    curve2d1 = new Geom2d_Line(gp.OX2d());
                                }
                                B.UpdateEdge(CurE, curve2d1, curve2d, CurF, 0.0);
                            }
                            else
                            {
                                CurE.Orientation(TopAbs_REVERSED);
                                curve2d1 = BRep_Tool.CurveOnSurface(CurE, CurF, out f, out l);
                                if (curve2d1 is null)
                                {
                                    curve2d1 = new Geom2d_Line(gp.OX2d());
                                }
                                B.UpdateEdge(CurE, curve2d, curve2d1, CurF, 0.0);
                            }
                            BRep_Tool.CurveOnSurface(CurE, CurF, out f, out l);
                            B.Range(CurE, f, l);
                        }
                        else
                        {
                            B.UpdateEdge(TopoDS.Edge(myMap[ex.Current()]), curve2d, TopoDS.Face(result), 0.0);
                        }

                        Geom_Curve? C3D =
                            BRep_Tool.Curve(TopoDS.Edge(myMap[ex.Current()]), out _, out _, out _);
                        if (C3D is null)
                        { // Update vertices
                            double param = 0.0;
                            var ex2 = new TopExp_Explorer(edge, TopAbs_VERTEX);
                            while (ex2.More())
                            {
                                TopoDS_Vertex vertex = TopoDS.Vertex(ex2.Current());
                                if (!M.NewParameter(vertex, edge, ref param, ref tol))
                                {
                                    tol = BRep_Tool.Tolerance(vertex);
                                    param = BRep_Tool.Parameter(vertex, edge);
                                }

                                TopAbs_Orientation vtxrelat = vertex.Orientation();
                                if (edge.Orientation() == TopAbs_REVERSED)
                                {
                                    // Update considers the edge as FORWARD, and the vertex relatively
                                    vtxrelat = TopAbs.Reverse(vtxrelat);
                                }
                                // if (myMap(edge).Orientation() == TopAbs_REVERSED) {
                                //   vtxrelat= TopAbs::Reverse(vtxrelat);
                                // }

                                TopoDS_Vertex aLocalVertex = TopoDS.Vertex(myMap[vertex]);
                                aLocalVertex.Orientation(vtxrelat);
                                // B.UpdateVertex(TopoDS::Vertex
                                //(myMap(vertex).Oriented(vtxrelat)),
                                B.UpdateVertex(aLocalVertex, param, TopoDS.Edge(myMap[edge]), tol);
                                ex2.Next();
                            }
                        }
                    }

                    // Copy polygon on triangulation
                    bool aNewPonT = M.NewPolygonOnTriangulation(edge, face, out Poly_PolygonOnTriangulation? aPolyOnTria_1);
                    Poly_PolygonOnTriangulation? aPolyOnTria_2 = null;
                    if (BRepTools.IsReallyClosed(edge, face))
                    {
                        // Obtain triangulation on reversed edge
                        TopoDS_Edge anEdgeRev = edge;
                        anEdgeRev.Reverse();
                        aNewPonT = M.NewPolygonOnTriangulation(anEdgeRev, face, out aPolyOnTria_2) || aNewPonT;
                        // It there is only one polygon on triangulation, store it to aPolyOnTria_1
                        if (aPolyOnTria_1 is null && aPolyOnTria_2 is not null)
                        {
                            aPolyOnTria_1 = aPolyOnTria_2;
                            aPolyOnTria_2 = null;
                        }
                    }
                    if (aNewPonT)
                    {
                        Poly_Triangulation? aNewFaceTria =
                            BRep_Tool.Triangulation(TopoDS.Face(myMap[face]), out TopLoc_Location aLocation);
                        TopoDS_Edge aNewEdge = TopoDS.Edge(myMap[edge]);
                        if (aPolyOnTria_2 is null)
                        {
                            B.UpdateEdge(aNewEdge, aPolyOnTria_1, aNewFaceTria, aLocation);
                        }
                        else
                        {
                            if (edge.Orientation() == TopAbs_FORWARD)
                            {
                                B.UpdateEdge(aNewEdge, aPolyOnTria_1, aPolyOnTria_2, aNewFaceTria, aLocation);
                            }
                            else
                            {
                                B.UpdateEdge(aNewEdge, aPolyOnTria_2, aPolyOnTria_1, aNewFaceTria, aLocation);
                            }
                        }
                    }
                }
            }

            //    else if (ts == TopAbs_EDGE) {
            else if (ts == TopAbs_EDGE && !No3DCurve)
            {
                // Vertices
                double param = 0.0;
                TopoDS_Edge edge = TopoDS.Edge(S);
                TopAbs_Orientation edor = edge.Orientation();
                if (edor != TopAbs_REVERSED)
                {
                    edor = TopAbs_FORWARD;
                }
                var ex = new TopExp_Explorer(edge.Oriented(edor), TopAbs_VERTEX);
                while (ex.More())
                {
                    TopoDS_Vertex vertex = TopoDS.Vertex(ex.Current());

                    if (!M.NewParameter(vertex, edge, ref param, ref tol))
                    {
                        tol = BRep_Tool.Tolerance(vertex);
                        param = BRep_Tool.Parameter(vertex, edge);
                    }

                    TopAbs_Orientation vtxrelat = vertex.Orientation();
                    if (edor == TopAbs_REVERSED)
                    {
                        // Update considers the edge as FORWARD, and the vertex relatively
                        vtxrelat = TopAbs.Reverse(vtxrelat);
                    }

                    // if (result.Orientation() == TopAbs_REVERSED) {
                    //   vtxrelat= TopAbs::Reverse(vtxrelat);
                    // }
                    TopoDS_Vertex aLocalVertex = TopoDS.Vertex(myMap[vertex]);
                    aLocalVertex.Orientation(vtxrelat);
                    // B.UpdateVertex(TopoDS::Vertex(myMap(vertex).Oriented(vtxrelat)),
                    if (myMutableInput || !aLocalVertex.IsSame(vertex))
                    {
                        B.UpdateVertex(aLocalVertex, param, TopoDS.Edge(result), tol);
                    }
                    ex.Next();
                }
            }

            // update flags

            result.Orientable(S.Orientable());
            result.Closed(S.Closed());
            result.Infinite(S.Infinite());
        }
        else
        {
            result = S;
        }

        // Set flag of the shape.
        result.Orientation(ResOr);

        // OCCT.NET: frozen: an unrebuilt shape is the input itself, whose flags SetShapeFlags copies onto themselves;
        // a frozen one can't be written, and keeps them.
        if (!(ReferenceEquals(result.TShape(), S.TShape()) && S.TShape()!.IsFrozen()))
        {
            SetShapeFlags(S, result);
        }

        return rebuild;
    }

    private void CreateNewVertices(NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theMVE,
                                   BRepTools_Modification M)
    {
        BRep_Builder aBB = new BRep_Builder();
        for (int i = 1; i <= theMVE.Extent(); i++)
        {
            // fill MyMap only with vertices with NewPoint == true
            TopoDS_Vertex aV = TopoDS.Vertex(theMVE.FindKey(i));
            bool IsNewP = M.NewPoint(aV, out gp_Pnt aPnt, out double aToler);
            if (IsNewP)
            {
                TopoDS_Vertex aNewV = new();
                aBB.MakeVertex(ref aNewV, aPnt, aToler);
                SetShapeFlags(aV, aNewV);
                myMap[aV] = aNewV;
                myHasNewGeom.Add(aV);
            }
            else if (myMutableInput)
            {
                myMap[aV] = aV.Oriented(TopAbs_FORWARD);
            }
        }
    }

    private void FillNewCurveInfo(NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theMEF,
                                  BRepTools_Modification M)
    {
        NewCurveInfo aNCinfo = new NewCurveInfo();
        for (int i = 1; i <= theMEF.Extent(); i++)
        {
            TopoDS_Edge anE = TopoDS.Edge(theMEF.FindKey(i));
            bool IsNewCur = M.NewCurve(anE, out Geom_Curve? aCurve, out TopLoc_Location aLocation, out double aToler);
            if (IsNewCur)
            {
                aNCinfo.myCurve = aCurve;
                aNCinfo.myLoc = aLocation;
                aNCinfo.myToler = aToler;
                myNCInfo.Bind(anE, aNCinfo);
                myHasNewGeom.Add(anE);
            }
        }
    }

    private void FillNewSurfaceInfo(BRepTools_Modification M)
    {
        var aMF = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(myShape, TopAbs_FACE, aMF);
        NewSurfaceInfo aNSinfo = new NewSurfaceInfo();
        for (int i = 1; i <= aMF.Extent(); i++)
        {
            TopoDS_Face aF = TopoDS.Face(aMF[i]);
            bool IsNewSur = M.NewSurface(aF,
                                         out Geom_Surface? aSurface,
                                         out TopLoc_Location aLocation,
                                         out double aToler1,
                                         out bool RevWires,
                                         out bool RevFace);
            if (IsNewSur)
            {
                aNSinfo.mySurface = aSurface;
                aNSinfo.myLoc = aLocation;
                aNSinfo.myToler = aToler1;
                aNSinfo.myRevWires = RevWires;
                aNSinfo.myRevFace = RevFace;
                myNSInfo.Bind(aF, aNSinfo);
                myHasNewGeom.Add(aF);
            }
            else
            {
                // check if subshapes will be modified
                bool notRebuilded = true;
                var exE = new TopExp_Explorer(aF, TopAbs_EDGE);
                while (exE.More() && notRebuilded)
                {
                    TopoDS_Edge anEE = TopoDS.Edge(exE.Current());
                    if (myNCInfo.IsBound(anEE))
                    {
                        notRebuilded = false;
                        break;
                    }
                    var exV = new TopExp_Explorer(anEE, TopAbs_VERTEX);
                    while (exV.More() && notRebuilded)
                    {
                        TopoDS_Vertex aVV = TopoDS.Vertex(exV.Current());
                        if (!myMap[aVV].IsNull())
                        {
                            notRebuilded = false;
                            break;
                        }
                        exV.Next();
                    }
                    exE.Next();
                }
                if (notRebuilded)
                {
                    // subshapes is not going to be modified
                    myNonUpdFace.Add(aF);
                }
            }
        }
    }

    private void CreateOtherVertices(NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theMVE,
                                     NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theMEF,
                                     BRepTools_Modification M)
    {
        // The following logic in some ways repeats the logic from the Rebuild() method.
        // If the face with its subshapes is not going to be modified
        //(i.e. NewSurface() for this face and NewCurve(), NewPoint() for its edges/vertices returns
        // false) then the calling of NewCurve2d() for this face with its edges is not performed.
        // Therefore, the updating of vertices will not present in such cases and
        // the EmptyCopied() operation for vertices from this face is not needed.

        for (int i = 1; i <= theMVE.Extent(); i++)
        {
            TopoDS_Vertex aV = TopoDS.Vertex(theMVE.FindKey(i));
            TopoDS_Vertex aNewV = TopoDS.Vertex(myMap[aV]);
            if (aNewV.IsNull())
            {
                NCollection_List<TopoDS_Shape> aLEdges = theMVE[i];
                bool toReplace = false;
                var it = new NCollection_List<TopoDS_Shape>.Iterator(aLEdges);
                for (; it.More() && !toReplace; it.Next())
                {
                    TopoDS_Edge anE = TopoDS.Edge(it.Value());
                    if (myNCInfo.IsBound(anE) && myNCInfo[anE].myCurve is not null)
                    {
                        toReplace = true;
                    }

                    if (!toReplace)
                    {
                        NCollection_List<TopoDS_Shape> aLFaces = theMEF.FindFromKey(anE);
                        var it2 = new NCollection_List<TopoDS_Shape>.Iterator(aLFaces);
                        for (; it2.More(); it2.Next())
                        {
                            TopoDS_Face aF = TopoDS.Face(it2.Value());
                            if (!myNonUpdFace.Contains(aF))
                            {
                                // some NewCurve2d()s may use NewE arg internally, so the
                                // null TShape as an arg may lead to the exceptions
                                TopoDS_Edge aDummyE = TopoDS.Edge(anE.EmptyCopied());
                                if (M.NewCurve2d(anE, aF, aDummyE, new TopoDS_Face(), out _, out _))
                                {
                                    toReplace = true;
                                    break;
                                }
                            }
                        }
                    }
                }
                if (toReplace)
                {
                    aNewV = TopoDS.Vertex(aV.EmptyCopied());
                }
                else
                {
                    aNewV = aV;
                }
                aNewV.Orientation(TopAbs_FORWARD);
                myMap[aV] = aNewV;
            }
        }
    }

    private static void SetShapeFlags(in TopoDS_Shape theInSh, in TopoDS_Shape theOutSh)
    {
        theOutSh.Modified(theInSh.Modified());
        theOutSh.Checked(theInSh.Checked());
        theOutSh.Orientable(theInSh.Orientable());
        theOutSh.Closed(theInSh.Closed());
        theOutSh.Infinite(theInSh.Infinite());
        theOutSh.Convex(theInSh.Convex());
    }
}
