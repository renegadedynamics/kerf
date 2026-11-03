// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Meshing;
using Kerf.Geometry;
using Kerf.Topology;
using Schematiq.Collections;
using OcctNet.ModelingData.Tests;
using static Kerf.Engine.Foundation.Status;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// The mesher's writing into shapes (decisions 488, 489): BRepMesh_ShapeTool's non-freezing attach, freezing on
/// publication, and the copy for write of BRepMesh_IncrementalMesh, run with a stub mesher (unit 4D brings the real
/// one). Also IMeshTools_MeshBuilder's statuses and IMeshTools_ShapeExplorer's order.
/// </summary>
public class BRepMeshShellTests
{
    // ------------------------------------------------------------------------------------------------ the stub mesher

    /// <summary>Builds a model of the shape's faces (forward) and edges, as IMeshTools_ShapeExplorer visits them.</summary>
    private sealed class StubModelBuilder : IMeshTools_ModelBuilder
    {
        private sealed class Visitor(BRepMeshData_Model theModel) : IMeshTools_ShapeVisitor
        {
            private readonly NCollection_Map<TopoDS_Shape> myEdges = new(TopTools_ShapeMapHasher.Instance);

            public override void Visit(in TopoDS_Face theFace) => theModel.AddFace(theFace);

            public override void Visit(in TopoDS_Edge theEdge)
            {
                if (myEdges.Add(theEdge))
                {
                    theModel.AddEdge(theEdge);
                }
            }
        }

        public bool ToThrow;

        protected override IMeshData_Model? performInternal(in TopoDS_Shape theShape, in IMeshTools_Parameters theParameters)
        {
            if (ToThrow)
            {
                throw new FailureException("stub");
            }

            if (theShape.IsNull())
            {
                SetStatus(Fail1);
                return null;
            }

            var aModel = new BRepMeshData_Model(theShape);
            new IMeshTools_ShapeExplorer(theShape).Accept(new Visitor(aModel));
            SetStatus(Done1);
            return aModel;
        }
    }

    /// <summary>A model algorithm returning the given result (or throwing).</summary>
    private sealed class StubAlgo(bool theResult, bool theToThrow = false) : IMeshTools_ModelAlgo
    {
        public int Calls;

        protected override bool performInternal(IMeshData_Model? theModel, in IMeshTools_Parameters theParameters,
                                                ProgressRange theRange)
        {
            Calls++;
            if (theToThrow)
            {
                throw new ConstructionErrorException("stub");
            }
            return theResult;
        }
    }

    /// <summary>
    /// Meshes each planar face without a triangulation (or each one, if asked) with a fan of its outer wire's vertices
    /// through BRepMesh_ShapeTool, with the polygons of its edges on it.
    /// </summary>
    private sealed class StubFaceDiscret(bool theToReplace = false) : IMeshTools_ModelAlgo
    {
        protected override bool performInternal(IMeshData_Model? theModel, in IMeshTools_Parameters theParameters,
                                                ProgressRange theRange)
        {
            for (int i = 0; i < theModel!.FacesNb(); i++)
            {
                TopoDS_Face aFace = theModel.GetFace(i).GetFace();
                if (!theToReplace && BRep_Tool.Triangulation(aFace, out _) is not null)
                {
                    continue;
                }
                FanMesh(aFace);
            }
            return true;
        }
    }

    /// <summary>Writes each face's deflection into its triangulation, in place, as ModelPostProcessor does.</summary>
    private sealed class StubPostProcessor : IMeshTools_ModelAlgo
    {
        protected override bool performInternal(IMeshData_Model? theModel, in IMeshTools_Parameters theParameters,
                                                ProgressRange theRange)
        {
            for (int i = 0; i < theModel!.FacesNb(); i++)
            {
                Triangulation? aTri = BRep_Tool.Triangulation(theModel.GetFace(i).GetFace(), out _);
                if (aTri is not null && !aTri.IsFrozen)
                {
                    aTri.Deflection(theParameters.Deflection);
                }
            }
            return true;
        }
    }

    private static IMeshTools_Context StubContext(bool theToReplace = false)
    {
        var aContext = new IMeshTools_Context();
        aContext.SetModelBuilder(new StubModelBuilder());
        aContext.SetEdgeDiscret(new StubAlgo(true));
        aContext.SetFaceDiscret(new StubFaceDiscret(theToReplace));
        aContext.SetPostProcessor(new StubPostProcessor());
        return aContext;
    }

    /// <summary>A fan triangulation of the planar face's outer wire, attached with its edges' polygons by BRepMesh_ShapeTool.</summary>
    private static void FanMesh(in TopoDS_Face theFace)
    {
        TopoDS_Face aFwd = TopoDS.Face(theFace.Oriented(TopAbs_FORWARD));
        Plane3D aPln = ((Geom_Plane)BRep_Tool.Surface(aFwd)!).Pln();
        var aVertices = new List<TopoDS_Vertex>();
        for (var anExp = new TopExp_Explorer(aFwd, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            TopExp.Vertices(TopoDS.Edge(anExp.Current()), out TopoDS_Vertex aFirst, out _, true);
            aVertices.Add(aFirst);
        }

        int n = aVertices.Count;
        var aTri = new Triangulation(n, n - 2, true);
        for (int i = 0; i < n; i++)
        {
            Point3D aP = BRep_Tool.Pnt(aVertices[i]).Transformed(theFace.Location().Transformation());
            aTri.SetNode(i + 1, aP);
            global::Kerf.Engine.Geometry.ElSLib.Parameters(aPln, BRep_Tool.Pnt(aVertices[i]), out double u, out double v);
            aTri.SetUVNode(i + 1, new Point2D(u, v));
        }
        for (int i = 1; i <= n - 2; i++)
        {
            aTri.SetTriangle(i, new Triangle1(1, i + 1, i + 2));
        }
        BRepMesh_ShapeTool.AddInFace(theFace, aTri);

        for (var anExp = new TopExp_Explorer(aFwd, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            TopoDS_Edge anE = TopoDS.Edge(anExp.Current());
            TopExp.Vertices(anE, out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);
            var aNodes = new NCollection_Array1<int>(1, 2);
            aNodes[1] = aVertices.FindIndex(v => v.IsSame(aV1)) + 1;
            aNodes[2] = aVertices.FindIndex(v => v.IsSame(aV2)) + 1;
            BRepMesh_ShapeTool.UpdateEdge(anE, new PolygonOnTriangulation(aNodes), aTri, theFace.Location());
        }
    }

    private static List<TopoDS_Face> Faces(in TopoDS_Shape theShape) => BRepGPropTestShapes.Faces(theShape);

    private static Triangulation? Triangulation(in TopoDS_Face theFace) => BRep_Tool.Triangulation(theFace, out _);

    // ------------------------------------------------------------------------------------------- the copy for write

    [Fact]
    public void Remeshing_an_adequately_meshed_frozen_shape_gives_it_back()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 2, 3));
        var aSnapshot = ShapeSnapshot.Of(aBox);

        // Unit 4D's default context meshes the copy; the input is never written.
        var aMesh = new BRepMesh_IncrementalMesh(aBox, 0.1);
        Assert.True(aMesh.IsDone());
        Assert.Equal(0, aMesh.GetStatusFlags());
        TopoDS_Shape aMeshed = aMesh.Shape();
        Assert.NotSame(aBox.TShape(), aMeshed.TShape());
        Assert.Single(aMesh.History().Modified(Faces(aBox)[0]));
        foreach (TopoDS_Face aF in Faces(aMeshed))
        {
            Assert.NotNull(Triangulation(aF));
        }
        aSnapshot.AssertUnchanged(aBox);

        // Meshing it again with the same parameters keeps every mesh: KeepUnchanged gives back the result itself.
        var aMesh2 = new BRepMesh_IncrementalMesh(aMeshed, 0.1);
        Assert.Same(aMeshed.TShape(), aMesh2.Shape().TShape());
        Assert.True(aMesh2.Shape().IsEqual(aMeshed));
        Assert.Empty(aMesh2.History().Modified(Faces(aMeshed)[0]));

        TopoDS_Shape aTwin = BRepMesh_IncrementalMesh.Meshed(aMeshed, new IMeshTools_Parameters { Deflection = 0.1 }, out BRepTools_History aHistory);
        Assert.Same(aMeshed.TShape(), aTwin.TShape());
        Assert.Empty(aHistory.Modified(Faces(aMeshed)[0]));
    }

    [Fact]
    public void Meshing_a_frozen_shape_writes_a_copy_and_shares_what_is_unchanged()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 2, 3));
        var aSnapshot = ShapeSnapshot.Of(aBox);

        var aMesh = new BRepMesh_IncrementalMesh();
        aMesh.SetShape(aBox);
        aMesh.ChangeParameters().Deflection = 0.25;
        aMesh.Perform(StubContext());
        Assert.True(aMesh.IsDone());
        TopoDS_Shape aResult = aMesh.Shape();

        // The input is never written.
        aSnapshot.AssertUnchanged(aBox);
        foreach (TopoDS_Face aF in Faces(aBox))
        {
            Assert.Null(Triangulation(aF));
        }

        // The result is meshed and frozen, meshes included; the post-processor could write them before publication.
        Assert.NotSame(aBox.TShape(), aResult.TShape());
        Assert.True(aResult.TShape()!.IsFrozen);
        List<TopoDS_Face> aFaces = Faces(aResult);
        Assert.Equal(6, aFaces.Count);
        foreach (TopoDS_Face aF in aFaces)
        {
            Triangulation aTri = Triangulation(aF)!;
            Assert.True(aTri.IsFrozen);
            Assert.Equal(0.25, aTri.Deflection());
            Assert.Throws<ImmutableObjectException>(() => aTri.Deflection(1.0));
            for (var anExp = new TopExp_Explorer(aF, TopAbs_EDGE); anExp.More(); anExp.Next())
            {
                PolygonOnTriangulation aPoly = BRep_Tool.PolygonOnTriangulation(TopoDS.Edge(anExp.Current()), aTri, aF.Location())!;
                Assert.True(aPoly.IsFrozen);
            }
        }

        // The vertices are the input's own; each face is mapped to its meshed image.
        var aInVertices = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(aBox, TopAbs_VERTEX, aInVertices);
        for (var anExp = new TopExp_Explorer(aResult, TopAbs_VERTEX); anExp.More(); anExp.Next())
        {
            Assert.True(aInVertices.Contains(anExp.Current()));
        }
        foreach (TopoDS_Face aF in Faces(aBox))
        {
            ISchematiqList<TopoDS_Shape> aModified = aMesh.History().Modified(aF);
            Assert.Single(aModified);
            Assert.NotNull(Triangulation(TopoDS.Face(aModified[0])));
        }

        // Meshing the meshed result again with a mesher that keeps existing meshes gives it back itself.
        var aAgain = new BRepMesh_IncrementalMesh();
        aAgain.SetShape(aResult);
        aAgain.ChangeParameters().Deflection = 0.25;
        aAgain.Perform(StubContext());
        Assert.Same(aResult.TShape(), aAgain.Shape().TShape());

        // Replacing the meshes changes the faces and edges, never the frozen result.
        var aSnapshot2 = ShapeSnapshot.Of(aResult);
        var aReplace = new BRepMesh_IncrementalMesh();
        aReplace.SetShape(aResult);
        aReplace.ChangeParameters().Deflection = 0.5;
        aReplace.Perform(StubContext(theToReplace: true));
        aSnapshot2.AssertUnchanged(aResult);
        Assert.NotSame(aResult.TShape(), aReplace.Shape().TShape());
        Assert.Equal(0.5, Triangulation(Faces(aReplace.Shape())[0])!.Deflection());
        Assert.Equal(0.25, Triangulation(Faces(aResult)[0])!.Deflection());
    }

    [Fact]
    public void Meshing_an_unfrozen_shape_writes_it_in_place()
    {
        TopoDS_Solid aBox = TestShapes.Box(1, 2, 3);
        Assert.False(aBox.TShape()!.IsFrozen);

        var aMesh = new BRepMesh_IncrementalMesh();
        aMesh.SetShape(aBox);
        aMesh.ChangeParameters().Deflection = 0.25;
        aMesh.Perform(StubContext());
        Assert.True(aMesh.Shape().IsEqual(aBox));
        Assert.Empty(aMesh.History().Modified(Faces(aBox)[0]));

        // Attached unfrozen (decision 488): the shape isn't published.
        foreach (TopoDS_Face aF in Faces(aBox))
        {
            Triangulation aTri = Triangulation(aF)!;
            Assert.False(aTri.IsFrozen);
            Assert.Equal(0.25, aTri.Deflection());
        }

        // Publishing freezes the graph with its meshes.
        aBox.TShape()!.FreezeGraph();
        foreach (TopoDS_Face aF in Faces(aBox))
        {
            Triangulation aTri = Triangulation(aF)!;
            Assert.True(aTri.IsFrozen);
            for (var anExp = new TopExp_Explorer(aF, TopAbs_EDGE); anExp.More(); anExp.Next())
            {
                Assert.True(BRep_Tool.PolygonOnTriangulation(TopoDS.Edge(anExp.Current()), aTri, aF.Location())!.IsFrozen);
            }
        }
    }

    [Fact]
    public void Meshed_never_writes_its_input_even_unfrozen()
    {
        TopoDS_Solid aBox = TestShapes.Box(1, 1, 1);
        TopoDS_Shape aResult = BRepMesh_IncrementalMesh.Meshed(aBox, new IMeshTools_Parameters { Deflection = 0.1 });
        Assert.True(aResult.TShape()!.IsFrozen);
        foreach (TopoDS_Face aF in Faces(aResult))
        {
            Assert.NotNull(Triangulation(aF));
        }
        foreach (TopoDS_Face aF in Faces(aBox))
        {
            Assert.Null(Triangulation(aF));
        }
        Assert.True(BRepMesh_IncrementalMesh.Meshed(new TopoDS_Shape(), new IMeshTools_Parameters()).IsNull());
    }

    [Fact]
    public void Parameters_are_checked_and_completed()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        Assert.Throws<NumericErrorException>(() => new BRepMesh_IncrementalMesh(aBox, 1e-8));
        Assert.Throws<NumericErrorException>(() => new BRepMesh_IncrementalMesh(aBox, 0.1, false, 1e-13));

        var aMesh = new BRepMesh_IncrementalMesh(aBox, 0.2, true, 0.3, true);
        IMeshTools_Parameters aP = aMesh.Parameters();
        Assert.Equal(0.2, aP.Deflection);
        Assert.Equal(0.2, aP.DeflectionInterior);
        Assert.Equal(0.3, aP.Angle);
        Assert.Equal(0.6, aP.AngleInterior);
        Assert.Equal(0.1 * 0.2, aP.MinSize, 1e-17);
        Assert.True(aP.Relative);
        Assert.True(aP.InParallel);
        Assert.False(aMesh.IsModified());

        var aExplicit = new BRepMesh_IncrementalMesh(aBox, new IMeshTools_Parameters { Deflection = 1e-7, MinSize = 0.5 });
        Assert.Equal(0.5, aExplicit.Parameters().MinSize);
        Assert.Equal(1e-7, aExplicit.Parameters().DeflectionInterior);
    }

    [Fact]
    public void Discret_takes_the_parallel_default()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        Assert.False(BRepMesh_IncrementalMesh.IsParallelDefault());
        Assert.Equal(0, BRepMesh_IncrementalMesh.Discret(aBox, 0.3, 0.4, out BRepMesh_DiscretRoot anAlgo));
        var aMesh = Assert.IsType<BRepMesh_IncrementalMesh>(anAlgo);
        Assert.False(aMesh.IsDone());
        Assert.True(aMesh.Shape().IsSame(aBox));
        Assert.Equal(0.3, aMesh.Parameters().Deflection);
        Assert.Equal(0.4, aMesh.Parameters().Angle);
        Assert.False(aMesh.Parameters().InParallel);
        aMesh.Perform();
        Assert.True(aMesh.IsDone());
    }

    [Fact]
    public void User_break_leaves_the_input_and_reports_it()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        using var aCancel = new CancellationTokenSource();
        aCancel.Cancel();
        var anIndicator = new ProgressIndicator(aCancel.Token);
        var aMesh = new BRepMesh_IncrementalMesh();
        aMesh.SetShape(aBox);
        aMesh.Perform(StubContext(), anIndicator.Start());
        Assert.False(aMesh.IsDone());
        Assert.Equal((int)IMeshData_Status.IMeshData_UserBreak, aMesh.GetStatusFlags());
        Assert.Same(aBox.TShape(), aMesh.Shape().TShape());
    }

    // ------------------------------------------------------------------------------------------- IMeshTools_MeshBuilder

    [Fact]
    public void MeshBuilder_statuses()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        TopoDS_Shape aCopy = BRepTools.ExactCopyForWrite(aBox, out _);

        var aNoContext = new IMeshTools_MeshBuilder();
        aNoContext.Perform(new ProgressRange());
        Assert.True(aNoContext.GetStatus().IsSet(Fail1));

        Status Run(IMeshTools_Context theContext, in TopoDS_Shape theShape)
        {
            theContext.SetShape(theShape);
            var aBuilder = new IMeshTools_MeshBuilder(theContext);
            aBuilder.Perform(new ProgressRange());
            foreach (Status aS in Enum.GetValues<Status>())
            {
                if (aS != None && aBuilder.GetStatus().IsSet(aS))
                {
                    return aS;
                }
            }
            return None;
        }

        Assert.Equal(Fail1, Run(new IMeshTools_Context(), aCopy)); // no model builder
        Assert.Equal(Warn1, Run(StubContext(), new TopoDS_Shape())); // nothing to mesh
        IMeshTools_Context aThrowing = StubContext();
        ((StubModelBuilder)aThrowing.GetModelBuilder()!).ToThrow = true;
        Assert.Equal(Fail2, Run(aThrowing, aCopy));

        IMeshTools_Context aNoEdges = StubContext();
        aNoEdges.SetEdgeDiscret(null);
        Assert.Equal(Fail3, Run(aNoEdges, aCopy));
        IMeshTools_Context aHealer = StubContext();
        aHealer.SetModelHealer(new StubAlgo(false));
        Assert.Equal(Fail4, Run(aHealer, aCopy));
        IMeshTools_Context aPre = StubContext();
        aPre.SetPreProcessor(new StubAlgo(true, theToThrow: true)); // an exception is a failure
        Assert.Equal(Fail5, Run(aPre, aCopy));
        IMeshTools_Context aFaces = StubContext();
        aFaces.SetFaceDiscret(new StubAlgo(false));
        Assert.Equal(Fail6, Run(aFaces, aCopy));
        IMeshTools_Context aPost = StubContext();
        aPost.SetPostProcessor(new StubAlgo(false));
        Assert.Equal(Fail7, Run(aPost, aCopy));

        // Done, and the model is cleaned when the parameters ask for it.
        IMeshTools_Context aGood = StubContext();
        Assert.Equal(Done1, Run(aGood, aCopy));
        Assert.Null(aGood.GetModel());
        IMeshTools_Context aKeep = StubContext();
        aKeep.ChangeParameters().CleanModel = false;
        Assert.Equal(Done1, Run(aKeep, BRepTools.ExactCopyForWrite(aBox, out _)));
        Assert.Equal(6, aKeep.GetModel()!.FacesNb());
        Assert.Equal(12, aKeep.GetModel()!.EdgesNb());
    }

    [Fact]
    public void ModelAlgo_lets_writes_into_frozen_shapes_through()
    {
        // decision 423's rule: a write into a shared input can't fail silently
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        var aContext = new IMeshTools_Context();
        aContext.SetModelBuilder(new StubModelBuilder());
        aContext.SetEdgeDiscret(new StubAlgo(true));
        aContext.SetFaceDiscret(new StubFaceDiscret());
        aContext.SetShape(aBox);
        var aBuilder = new IMeshTools_MeshBuilder(aContext);
        Assert.ThrowsAny<DomainErrorException>(() => aBuilder.Perform(new ProgressRange()));
    }

    // ------------------------------------------------------------------------------------------- IMeshTools_ShapeExplorer

    private sealed class RecordingVisitor : IMeshTools_ShapeVisitor
    {
        public readonly List<TopoDS_Shape> Visited = [];

        public override void Visit(in TopoDS_Face theFace) => Visited.Add(theFace);

        public override void Visit(in TopoDS_Edge theEdge) => Visited.Add(theEdge);
    }

    [Fact]
    public void ShapeExplorer_visits_free_edges_then_each_face_once()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        TopoDS_Edge aFree = TestShapes.Frozen(TestShapes.Edge(new Point3D(5, 0, 0), new Point3D(6, 0, 0)));
        var aTrsf = new Transform3D();
        aTrsf.SetTranslation(new Vector3D(0, 0, 10));
        TopoDS_Edge aMovedFree = TopoDS.Edge(aFree.Moved(new Placement(aTrsf)));
        TopoDS_Face aFace = Faces(aBox)[0];

        TopoDS_Compound aCompound = TopoDS_Builder.MakeCompound([aMovedFree, aBox, aFace.Reversed()]);
        var aVisitor = new RecordingVisitor();
        new IMeshTools_ShapeExplorer(aCompound).Accept(aVisitor);

        // the free edge first, at the identity location
        Assert.Equal(TopAbs_EDGE, aVisitor.Visited[0].ShapeType());
        Assert.True(aVisitor.Visited[0].IsSame(aFree));
        Assert.True(aVisitor.Visited[0].Location().IsIdentity);

        // then each face once (the reversed copy of a box face is the same face), forward, after its edges
        List<TopoDS_Shape> aFaces = aVisitor.Visited.Where(s => s.ShapeType() == TopAbs_FACE).ToList();
        Assert.Equal(6, aFaces.Count);
        Assert.All(aFaces, f => Assert.Equal(TopAbs_FORWARD, f.Orientation()));
        Assert.Equal(1 + 6 * 4 + 6, aVisitor.Visited.Count);
        int aFirstFace = aVisitor.Visited.FindIndex(s => s.ShapeType() == TopAbs_FACE);
        Assert.Equal(5, aFirstFace); // the free edge and the first face's four edges
    }

    // ------------------------------------------------------------------------------------------- BRepMesh_ShapeTool

    [Fact]
    public void ShapeTool_tolerances_ranges_and_points()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(2, 3, 4));
        TopoDS_Face aFace = Faces(aBox)[0];
        double aMax = BRep_Tool.Tolerance(aFace);
        for (var anExp = new TopExp_Explorer(aFace, TopAbs_VERTEX); anExp.More(); anExp.Next())
        {
            aMax = Math.Max(aMax, BRep_Tool.Tolerance(TopoDS.Vertex(anExp.Current())));
        }
        for (var anExp = new TopExp_Explorer(aFace, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            aMax = Math.Max(aMax, BRep_Tool.Tolerance(TopoDS.Edge(anExp.Current())));
        }
        Assert.Equal(aMax, BRepMesh_ShapeTool.MaxFaceTolerance(aFace));

        double aDim = -1.0;
        BRepMesh_ShapeTool.BoxMaxDimension(new Box(), ref aDim);
        Assert.Equal(-1.0, aDim); // void box: unchanged
        var aBnd = new Box(new Point3D(0, 0, 0), new Point3D(1, 5, 2));
        BRepMesh_ShapeTool.BoxMaxDimension(aBnd, ref aDim);
        Assert.Equal(5.0, aDim);

        // A located edge: the 3D range is the located curve's, reversed with the orientation if asked.
        var aTrsf = new Transform3D();
        aTrsf.SetTranslation(new Vector3D(0, 0, 7));
        TopoDS_Edge anEdge = TestShapes.Frozen(TestShapes.Edge(new Geom_Line(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), 1, 3));
        TopoDS_Edge aMoved = TopoDS.Edge(anEdge.Moved(new Placement(aTrsf)).Reversed());
        Assert.True(BRepMesh_ShapeTool.Range(aMoved, out Geom_Curve? aCurve, out double aF, out double aL));
        Assert.Equal((1.0, 3.0), (aF, aL));
        Assert.True(aCurve!.Value(aF).IsEqual(new Point3D(1, 0, 7), 1e-15));
        Assert.True(BRepMesh_ShapeTool.Range(aMoved, out _, out aF, out aL, true));
        Assert.Equal((3.0, 1.0), (aF, aL));

        // Pcurve range and UV points on a box face.
        TopoDS_Edge aBoxEdge = TopoDS.Edge(new TopExp_Explorer(aFace, TopAbs_EDGE).Current());
        Assert.True(BRepMesh_ShapeTool.Range(aBoxEdge, aFace, out Geom2d_Curve? aPC, out aF, out aL));
        Assert.NotNull(aPC);
        Assert.True(BRepMesh_ShapeTool.UVPoints(aBoxEdge, aFace, out Point2D aUV1, out Point2D aUV2));
        Assert.True(aUV1.IsEqual(aPC!.Value(aF), 0.0));
        Assert.True(aUV2.IsEqual(aPC.Value(aL), 0.0));
        // (an edge not on a plane still has a pcurve there: BRep_Tool projects it)
        Assert.True(BRepMesh_ShapeTool.Range(anEdge, aFace, out _, out _, out _));

        Assert.True(BRepMesh_ShapeTool.UseLocation(new Point3D(1, 2, 3), new Placement(aTrsf)).IsEqual(new Point3D(1, 2, 10), 0.0));
        Assert.True(BRepMesh_ShapeTool.UseLocation(new Point3D(1, 2, 3), new Placement()).IsEqual(new Point3D(1, 2, 3), 0.0));
    }

    [Fact]
    public void ShapeTool_CheckAndUpdateFlags()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(2, 3, 4));
        TopoDS_Face aFace = TopoDS.Face(Faces(aBox)[0].Oriented(TopAbs_FORWARD));
        TopoDS_Edge aBoxEdge = TopoDS.Edge(new TopExp_Explorer(aFace, TopAbs_EDGE).Current());
        var aDFace = new BRepMeshData_Face(aFace);
        var aDEdge = new BRepMeshData_Edge(aBoxEdge);
        IMeshData_PCurve aPCurve = aDEdge.AddPCurve(aDFace, aBoxEdge.Orientation());
        bool aSameRange = aDEdge.GetSameRange();
        BRepMesh_ShapeTool.CheckAndUpdateFlags(aDEdge, aPCurve);
        Assert.Equal(aSameRange, aDEdge.GetSameRange());
        Assert.False(aDEdge.GetDegenerated());

        // Flags already at their worst: nothing to do.
        aDEdge.SetSameParam(false);
        aDEdge.SetSameRange(false);
        aDEdge.SetDegenerated(true);
        BRepMesh_ShapeTool.CheckAndUpdateFlags(aDEdge, aPCurve);
        Assert.True(aDEdge.GetDegenerated());

        // A closed edge shorter than its vertex tolerance is degenerated.
        TopoDS_Vertex aV = TestShapes.Vertex(new Point3D(0, 0, 0));
        var aB = new BRep_Builder();
        aB.UpdateVertex(aV, 0.1);
        Geom_Circle aTiny = new(new Frame3D(new Point3D(0.001, 0, 0), global::Kerf.Engine.Geometry.Constants.DZ()), 0.001);
        TopoDS_Edge aLoop = TestShapes.Edge(aTiny, 0, 2 * Math.PI, aV, TopoDS.Vertex(aV.Reversed()));
        aB.UpdateEdge(aLoop, new Geom2d_Line(new Point2D(0, 0), new Direction2D(1, 0)), aFace, 1e-7);
        var aDLoop = new BRepMeshData_Edge(aLoop);
        IMeshData_PCurve aLoopPCurve = aDLoop.AddPCurve(aDFace, TopAbs_FORWARD);
        Assert.False(aDLoop.GetDegenerated());
        BRepMesh_ShapeTool.CheckAndUpdateFlags(aDLoop, aLoopPCurve);
        Assert.True(aDLoop.GetDegenerated());
    }

    [Fact]
    public void ShapeTool_attaches_unfrozen_and_moves_nodes_by_the_face_location()
    {
        TopoDS_Solid aBox = TestShapes.Box(1, 1, 1); // unfrozen: ported code meshing in place
        var aTrsf = new Transform3D();
        aTrsf.SetTranslation(new Vector3D(10, 0, 0));
        TopoDS_Face aFace = Faces(aBox)[0];
        TopoDS_Face aMoved = TopoDS.Face(aFace.Moved(new Placement(aTrsf)));

        var aTri = new Triangulation(3, 1, false);
        aTri.SetNode(1, new Point3D(10, 0, 0));
        aTri.SetNode(2, new Point3D(11, 0, 0));
        aTri.SetNode(3, new Point3D(10, 1, 0));
        aTri.SetTriangle(1, new Triangle1(1, 2, 3));
        BRepMesh_ShapeTool.AddInFace(aMoved, aTri);

        // The nodes are in the face's definition: the location is taken off.
        Triangulation aStored = BRep_Tool.Triangulation(aFace, out Placement aLoc)!;
        Assert.Same(aTri, aStored);
        Assert.True(aLoc.IsIdentity);
        Assert.True(aStored.Node(2).IsEqual(new Point3D(1, 0, 0), 1e-15));
        Assert.False(aStored.IsFrozen);
        aStored.Deflection(0.5); // writable until published

        // Polygons: on the triangulation, a seam's pair, a 3D polygon; all unfrozen; removed by Nullify.
        TopoDS_Edge anEdge = TopoDS.Edge(new TopExp_Explorer(aFace, TopAbs_EDGE).Current());
        var aNodes = new NCollection_Array1<int>(1, 2);
        aNodes[1] = 1;
        aNodes[2] = 2;
        var aPoly = new PolygonOnTriangulation(aNodes);
        BRepMesh_ShapeTool.UpdateEdge(anEdge, aPoly, aTri, new Placement());
        Assert.Same(aPoly, BRep_Tool.PolygonOnTriangulation(anEdge, aTri, new Placement()));
        Assert.False(aPoly.IsFrozen);

        var aPoly2 = new PolygonOnTriangulation(aNodes);
        TopoDS_Edge anOther = TopoDS.Edge(new TopExp_Explorer(Faces(aBox)[1], TopAbs_EDGE).Current());
        BRepMesh_ShapeTool.UpdateEdge(anOther, aPoly, aPoly2, aTri, new Placement());
        Assert.False(aPoly2.IsFrozen);

        var aP3dNodes = new NCollection_Array1<Point3D>(1, 2);
        aP3dNodes[1] = new Point3D(0, 0, 0);
        aP3dNodes[2] = new Point3D(1, 0, 0);
        var aP3d = new Polygon3D(aP3dNodes);
        BRepMesh_ShapeTool.UpdateEdge(anEdge, aP3d);
        Assert.Same(aP3d, BRep_Tool.Polygon3D(anEdge, out _));
        Assert.False(aP3d.IsFrozen);

        // Publication freezes all of them.
        aBox.TShape()!.FreezeGraph();
        Assert.True(aTri.IsFrozen);
        Assert.True(aPoly.IsFrozen);
        Assert.True(aPoly2.IsFrozen);
        Assert.True(aP3d.IsFrozen);
        Assert.ThrowsAny<DomainErrorException>(() => BRepMesh_ShapeTool.NullifyFace(aFace));
        Assert.ThrowsAny<DomainErrorException>(() => BRepMesh_ShapeTool.NullifyEdge(anEdge, new Placement()));
    }

    [Fact]
    public void ShapeTool_nullifies_meshes_on_an_exact_copy()
    {
        TopoDS_Solid aBox = TestShapes.Box(1, 1, 1);
        foreach (TopoDS_Face aF in Faces(aBox))
        {
            FanMesh(aF);
        }
        TestShapes.Frozen(aBox);

        TopoDS_Shape aCopy = BRepTools.ExactCopyForWrite(aBox, out _);
        TopoDS_Face aFace = Faces(aCopy)[0];
        Triangulation aTri = Triangulation(aFace)!;
        Assert.True(aTri.IsFrozen); // shared with the frozen input

        TopoDS_Edge anEdge = TopoDS.Edge(new TopExp_Explorer(aFace, TopAbs_EDGE).Current());
        Assert.NotNull(BRep_Tool.PolygonOnTriangulation(anEdge, aTri, aFace.Location()));
        BRepMesh_ShapeTool.NullifyEdge(anEdge, aTri, aFace.Location());
        Assert.Null(BRep_Tool.PolygonOnTriangulation(anEdge, aTri, aFace.Location()));
        BRepMesh_ShapeTool.NullifyFace(aFace);
        Assert.Null(Triangulation(aFace));

        // the input keeps its meshes
        Assert.Same(aTri, Triangulation(Faces(aBox)[0]));
    }
}
