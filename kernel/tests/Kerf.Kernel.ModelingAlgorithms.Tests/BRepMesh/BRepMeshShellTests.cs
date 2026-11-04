// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Meshing;
using Kerf.Engine.Meshing.Data;
using Kerf.Engine.Meshing.Model;
using Kerf.Engine.Meshing.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Schematiq.Collections;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Engine.Foundation.Status;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The mesher's writing into shapes (decisions 489 and 17304): BRepMesh_ShapeTool's attach, frozen (decision 488's
/// unfrozen attach until publication retired by unit 10L5), and the copy for write of BRepMesh_IncrementalMesh, run with a stub mesher (unit 4D brings the real
/// one). Also IMeshTools_MeshBuilder's statuses and IMeshTools_ShapeExplorer's order.
/// </summary>
public class BRepMeshShellTests
{
    // ------------------------------------------------------------------------------------------------ the stub mesher

    /// <summary>Builds a model of the shape's faces (forward) and edges, as IMeshTools_ShapeExplorer visits them.</summary>
    private sealed class StubModelBuilder : ModelBuilder
    {
        private sealed class Visitor(ModelData theModel) : ShapeVisitor
        {
            private readonly HashSet<Shape> myEdges = new(ShapeHasher.Instance);

            public override void Visit(in Face theFace) => theModel.AddFace(theFace);

            public override void Visit(in Edge theEdge)
            {
                if (myEdges.Add(theEdge))
                {
                    theModel.AddEdge(theEdge);
                }
            }
        }

        public bool ToThrow;

        protected override ModelModel? PerformInternal(in Shape theShape, in ToolParameters theParameters)
        {
            if (ToThrow)
            {
                throw new FailureException("stub");
            }

            if (theShape.IsNull)
            {
                SetStatus(Fail1);
                return null;
            }

            var aModel = new ModelData(theShape);
            new ShapeExplorer(theShape).Accept(new Visitor(aModel));
            SetStatus(Done1);
            return aModel;
        }
    }

    /// <summary>A model algorithm returning the given result (or throwing).</summary>
    private sealed class StubAlgo(bool theResult, bool theToThrow = false) : ModelAlgo
    {
        public int Calls;

        protected override bool PerformInternal(ModelModel? theModel, in ToolParameters theParameters,
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
    /// Meshes each planar face without a triangulation (or each one, if asked) with a fan of its outer wire's vertices,
    /// handed to the post-processor through BRepMesh_ShapeTool, as the mesher's algorithms do (unit 10L5).
    /// </summary>
    private sealed class StubFaceDiscret(bool theToReplace = false) : ModelAlgo
    {
        protected override bool PerformInternal(ModelModel? theModel, in ToolParameters theParameters,
                                                ProgressRange theRange)
        {
            for (int i = 0; i < theModel!.FacesNb(); i++)
            {
                FaceModel aDFace = theModel.GetFace(i);
                if (!theToReplace && Tool.Triangulation(aDFace.GetFace(), out _) is not null)
                {
                    continue;
                }
                ShapeTool.AddInFace(aDFace, FanTriangulation(aDFace.GetFace(), out _));
            }
            return true;
        }
    }

    /// <summary>
    /// Finishes each new triangulation with the face's deflection, then attaches it with the polygons of its edges, as
    /// ModelPostProcessor does (unit 10L5: finished, then attached frozen).
    /// </summary>
    private sealed class StubPostProcessor : ModelAlgo
    {
        protected override bool PerformInternal(ModelModel? theModel, in ToolParameters theParameters,
                                                ProgressRange theRange)
        {
            for (int i = 0; i < theModel!.FacesNb(); i++)
            {
                FaceModel aDFace = theModel.GetFace(i);
                if (aDFace.MadeTriangulation is { } aTri)
                {
                    aDFace.MadeTriangulation = null;
                    aTri.Deflection(theParameters.Deflection);
                    AttachFan(aDFace.GetFace(), aTri);
                }
            }
            return true;
        }
    }

    private static ToolContext StubContext(bool theToReplace = false)
    {
        var aContext = new ToolContext();
        aContext.SetModelBuilder(new StubModelBuilder());
        aContext.SetEdgeDiscret(new StubAlgo(true));
        aContext.SetFaceDiscret(new StubFaceDiscret(theToReplace));
        aContext.SetPostProcessor(new StubPostProcessor());
        return aContext;
    }

    /// <summary>A fan triangulation of the planar face's outer wire, attached with its edges' polygons by BRepMesh_ShapeTool.</summary>
    private static void FanMesh(in Face theFace) => AttachFan(theFace, FanTriangulation(theFace, out _));

    /// <summary>The fan triangulation's nodes in the face's definition, and its edges' polygons on it, attached.</summary>
    private static void AttachFan(in Face theFace, Triangulation theTri)
    {
        FanTriangulation(theFace, out List<Vertex> aVertices);
        Face aFwd = ShapeCasts.Face(theFace.Oriented(Forward));
        ShapeTool.AttachToFace(theFace, theTri);
        for (var anExp = new Explorer(aFwd, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Edge anE = ShapeCasts.Edge(anExp.Current());
            Exploration.Vertices(anE, out Vertex aV1, out Vertex aV2);
            var aNodes = new NCollection_Array1<int>(1, 2);
            aNodes[1] = aVertices.FindIndex(v => v.IsSame(aV1)) + 1;
            aNodes[2] = aVertices.FindIndex(v => v.IsSame(aV2)) + 1;
            ShapeTool.UpdateEdge(anE, new PolygonOnTriangulation(aNodes), theTri, theFace.Placement);
        }
    }

    /// <summary>A fan triangulation of the planar face's outer wire, its nodes in the face's definition.</summary>
    private static Triangulation FanTriangulation(in Face theFace, out List<Vertex> aVertices)
    {
        Face aFwd = ShapeCasts.Face(theFace.Oriented(Forward));
        Plane3D aPln = ((PlaneSurface)Tool.Surface(aFwd)!).Plane;
        aVertices = [];
        for (var anExp = new Explorer(aFwd, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Exploration.Vertices(ShapeCasts.Edge(anExp.Current()), out Vertex aFirst, out _, true);
            aVertices.Add(aFirst);
        }

        int n = aVertices.Count;
        var aTri = new Triangulation(n, n - 2, true);
        for (int i = 0; i < n; i++)
        {
            Point3D aP = Tool.Pnt(aVertices[i]).Transformed(theFace.Placement.Transformation);
            aTri.SetNode(i + 1, aP);
            global::Kerf.Engine.Geometry.ElSLib.Parameters(aPln, Tool.Pnt(aVertices[i]), out double u, out double v);
            aTri.SetUVNode(i + 1, new Point2D(u, v));
        }
        for (int i = 1; i <= n - 2; i++)
        {
            aTri.SetTriangle(i, new Triangle1(1, i + 1, i + 2));
        }
        // the nodes in the face's definition, as ShapeTool.AddInFace stores them
        Transform3D anInverse = theFace.Placement.Transformation.Inverted();
        for (int i = 1; i <= n; i++)
        {
            aTri.SetNode(i, aTri.Node(i).Transformed(anInverse));
        }
        return aTri;
    }

    private static List<Face> Faces(in Shape theShape) => BRepGPropTestShapes.Faces(theShape);

    private static Triangulation? Triangulation(in Face theFace) => Tool.Triangulation(theFace, out _);

    // ------------------------------------------------------------------------------------------- the copy for write

    [Fact]
    public void Remeshing_an_adequately_meshed_frozen_shape_gives_it_back()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 2, 3));
        var aSnapshot = ShapeSnapshot.Of(aBox);

        // Unit 4D's default context meshes the copy; the input is never written.
        var aMesh = new IncrementalMesh(aBox, 0.1);
        Assert.True(aMesh.IsDone);
        Assert.Equal(0, aMesh.GetStatusFlags());
        Shape aMeshed = aMesh.Shape();
        Assert.NotSame(aBox.TShape(), aMeshed.TShape());
        Assert.Single(aMesh.History().Modified(Faces(aBox)[0]));
        foreach (Face aF in Faces(aMeshed))
        {
            Assert.NotNull(Triangulation(aF));
        }
        aSnapshot.AssertUnchanged(aBox);

        // Meshing it again with the same parameters keeps every mesh: KeepUnchanged gives back the result itself.
        var aMesh2 = new IncrementalMesh(aMeshed, 0.1);
        Assert.Same(aMeshed.TShape(), aMesh2.Shape().TShape());
        Assert.True(aMesh2.Shape().IsEqual(aMeshed));
        Assert.Empty(aMesh2.History().Modified(Faces(aMeshed)[0]));

        Shape aTwin = IncrementalMesh.Meshed(aMeshed, new ToolParameters { Deflection = 0.1 }, out History aHistory);
        Assert.Same(aMeshed.TShape(), aTwin.TShape());
        Assert.Empty(aHistory.Modified(Faces(aMeshed)[0]));
    }

    [Fact]
    public void Meshing_a_frozen_shape_writes_a_copy_and_shares_what_is_unchanged()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 2, 3));
        var aSnapshot = ShapeSnapshot.Of(aBox);

        var aMesh = new IncrementalMesh();
        aMesh.SetShape(aBox);
        aMesh.ChangeParameters().Deflection = 0.25;
        aMesh.Perform(StubContext());
        Assert.True(aMesh.IsDone);
        Shape aResult = aMesh.Shape();

        // The input is never written.
        aSnapshot.AssertUnchanged(aBox);
        foreach (Face aF in Faces(aBox))
        {
            Assert.Null(Triangulation(aF));
        }

        // The result is meshed and frozen, meshes included; the post-processor could write them before publication.
        Assert.NotSame(aBox.TShape(), aResult.TShape());
        Assert.True(aResult.TShape()!.IsFrozen);
        List<Face> aFaces = Faces(aResult);
        Assert.Equal(6, aFaces.Count);
        foreach (Face aF in aFaces)
        {
            Triangulation aTri = Triangulation(aF)!;
            Assert.True(aTri.IsFrozen);
            Assert.Equal(0.25, aTri.Deflection());
            Assert.Throws<ImmutableObjectException>(() => aTri.Deflection(1.0));
            for (var anExp = new Explorer(aF, ShapeType.Edge); anExp.More(); anExp.Next())
            {
                PolygonOnTriangulation aPoly = Tool.PolygonOnTriangulation(ShapeCasts.Edge(anExp.Current()), aTri, aF.Placement)!;
                Assert.True(aPoly.IsFrozen);
            }
        }

        // The vertices are the input's own; each face is mapped to its meshed image.
        var aInVertices = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(aBox, ShapeType.Vertex, aInVertices);
        for (var anExp = new Explorer(aResult, ShapeType.Vertex); anExp.More(); anExp.Next())
        {
            Assert.True(aInVertices.Contains(anExp.Current()));
        }
        foreach (Face aF in Faces(aBox))
        {
            ISchematiqList<Shape> aModified = aMesh.History().Modified(aF);
            Assert.Single(aModified);
            Assert.NotNull(Triangulation(ShapeCasts.Face(aModified[0])));
        }

        // Meshing the meshed result again with a mesher that keeps existing meshes gives it back itself.
        var aAgain = new IncrementalMesh();
        aAgain.SetShape(aResult);
        aAgain.ChangeParameters().Deflection = 0.25;
        aAgain.Perform(StubContext());
        Assert.Same(aResult.TShape(), aAgain.Shape().TShape());

        // Replacing the meshes changes the faces and edges, never the frozen result.
        var aSnapshot2 = ShapeSnapshot.Of(aResult);
        var aReplace = new IncrementalMesh();
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
        Solid aBox = TestShapes.Box(1, 2, 3);
        Assert.False(aBox.TShape()!.IsFrozen);

        var aMesh = new IncrementalMesh();
        aMesh.SetShape(aBox);
        aMesh.ChangeParameters().Deflection = 0.25;
        aMesh.Perform(StubContext());
        Assert.True(aMesh.Shape().IsEqual(aBox));
        Assert.Empty(aMesh.History().Modified(Faces(aBox)[0]));

        // Attached finished and frozen, though the shape isn't published (decision 17304; decision 488 attached them
        // unfrozen until publication).
        foreach (Face aF in Faces(aBox))
        {
            Triangulation aTri = Triangulation(aF)!;
            Assert.True(aTri.IsFrozen);
            Assert.Equal(0.25, aTri.Deflection());
        }

        // Publishing freezes the graph with its meshes.
        aBox.TShape()!.FreezeGraph();
        foreach (Face aF in Faces(aBox))
        {
            Triangulation aTri = Triangulation(aF)!;
            Assert.True(aTri.IsFrozen);
            for (var anExp = new Explorer(aF, ShapeType.Edge); anExp.More(); anExp.Next())
            {
                Assert.True(Tool.PolygonOnTriangulation(ShapeCasts.Edge(anExp.Current()), aTri, aF.Placement)!.IsFrozen);
            }
        }
    }

    [Fact]
    public void Meshed_never_writes_its_input_even_unfrozen()
    {
        Solid aBox = TestShapes.Box(1, 1, 1);
        Shape aResult = IncrementalMesh.Meshed(aBox, new ToolParameters { Deflection = 0.1 });
        Assert.True(aResult.TShape()!.IsFrozen);
        foreach (Face aF in Faces(aResult))
        {
            Assert.NotNull(Triangulation(aF));
        }
        foreach (Face aF in Faces(aBox))
        {
            Assert.Null(Triangulation(aF));
        }
        Assert.True(IncrementalMesh.Meshed(new Shape(), new ToolParameters()).IsNull);
    }

    [Fact]
    public void Parameters_are_checked_and_completed()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        Assert.Throws<NumericErrorException>(() => new IncrementalMesh(aBox, 1e-8));
        Assert.Throws<NumericErrorException>(() => new IncrementalMesh(aBox, 0.1, false, 1e-13));

        var aMesh = new IncrementalMesh(aBox, 0.2, true, 0.3, true);
        ToolParameters aP = aMesh.Parameters();
        Assert.Equal(0.2, aP.Deflection);
        Assert.Equal(0.2, aP.DeflectionInterior);
        Assert.Equal(0.3, aP.Angle);
        Assert.Equal(0.6, aP.AngleInterior);
        Assert.Equal(0.1 * 0.2, aP.MinSize, 1e-17);
        Assert.True(aP.Relative);
        Assert.True(aP.InParallel);
        Assert.False(aMesh.IsModified);

        var aExplicit = new IncrementalMesh(aBox, new ToolParameters { Deflection = 1e-7, MinSize = 0.5 });
        Assert.Equal(0.5, aExplicit.Parameters().MinSize);
        Assert.Equal(1e-7, aExplicit.Parameters().DeflectionInterior);
    }

    [Fact]
    public void Discret_takes_the_parallel_default()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        Assert.False(IncrementalMesh.IsParallelDefault);
        Assert.Equal(0, IncrementalMesh.Discret(aBox, 0.3, 0.4, out DiscretRoot anAlgo));
        var aMesh = Assert.IsType<IncrementalMesh>(anAlgo);
        Assert.False(aMesh.IsDone);
        Assert.True(aMesh.Shape().IsSame(aBox));
        Assert.Equal(0.3, aMesh.Parameters().Deflection);
        Assert.Equal(0.4, aMesh.Parameters().Angle);
        Assert.False(aMesh.Parameters().InParallel);
        aMesh.Perform();
        Assert.True(aMesh.IsDone);
    }

    [Fact]
    public void User_break_leaves_the_input_and_reports_it()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        using var aCancel = new CancellationTokenSource();
        aCancel.Cancel();
        var anIndicator = new ProgressIndicator(aCancel.Token);
        var aMesh = new IncrementalMesh();
        aMesh.SetShape(aBox);
        aMesh.Perform(StubContext(), anIndicator.Start());
        Assert.False(aMesh.IsDone);
        Assert.Equal((int)StatusModel.UserBreak, aMesh.GetStatusFlags());
        Assert.Same(aBox.TShape(), aMesh.Shape().TShape());
    }

    // ------------------------------------------------------------------------------------------- IMeshTools_MeshBuilder

    [Fact]
    public void MeshBuilder_statuses()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        Shape aCopy = TestCopies.ExactCopyForWrite(aBox, out _);

        var aNoContext = new MeshBuilder();
        aNoContext.Perform(new ProgressRange());
        Assert.True(aNoContext.GetStatus().IsSet(Fail1));

        Status Run(ToolContext theContext, in Shape theShape)
        {
            theContext.SetShape(theShape);
            var aBuilder = new MeshBuilder(theContext);
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

        Assert.Equal(Fail1, Run(new ToolContext(), aCopy)); // no model builder
        Assert.Equal(Warn1, Run(StubContext(), new Shape())); // nothing to mesh
        ToolContext aThrowing = StubContext();
        ((StubModelBuilder)aThrowing.GetModelBuilder()!).ToThrow = true;
        Assert.Equal(Fail2, Run(aThrowing, aCopy));

        ToolContext aNoEdges = StubContext();
        aNoEdges.SetEdgeDiscret(null);
        Assert.Equal(Fail3, Run(aNoEdges, aCopy));
        ToolContext aHealer = StubContext();
        aHealer.SetModelHealer(new StubAlgo(false));
        Assert.Equal(Fail4, Run(aHealer, aCopy));
        ToolContext aPre = StubContext();
        aPre.SetPreProcessor(new StubAlgo(true, theToThrow: true)); // an exception is a failure
        Assert.Equal(Fail5, Run(aPre, aCopy));
        ToolContext aFaces = StubContext();
        aFaces.SetFaceDiscret(new StubAlgo(false));
        Assert.Equal(Fail6, Run(aFaces, aCopy));
        ToolContext aPost = StubContext();
        aPost.SetPostProcessor(new StubAlgo(false));
        Assert.Equal(Fail7, Run(aPost, aCopy));

        // Done, and the model is cleaned when the parameters ask for it.
        ToolContext aGood = StubContext();
        Assert.Equal(Done1, Run(aGood, aCopy));
        Assert.Null(aGood.GetModel());
        ToolContext aKeep = StubContext();
        aKeep.ChangeParameters().CleanModel = false;
        Assert.Equal(Done1, Run(aKeep, TestCopies.ExactCopyForWrite(aBox, out _)));
        Assert.Equal(6, aKeep.GetModel()!.FacesNb());
        Assert.Equal(12, aKeep.GetModel()!.EdgesNb());
    }

    [Fact]
    public void ModelAlgo_lets_writes_into_frozen_shapes_through()
    {
        // decision 423's rule: a write into a shared input can't fail silently
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        var aContext = new ToolContext();
        aContext.SetModelBuilder(new StubModelBuilder());
        aContext.SetEdgeDiscret(new StubAlgo(true));
        aContext.SetFaceDiscret(new StubFaceDiscret());
        aContext.SetPostProcessor(new StubPostProcessor()); // the post-processor attaches (decision 17304)
        aContext.SetShape(aBox);
        var aBuilder = new MeshBuilder(aContext);
        Assert.ThrowsAny<DomainErrorException>(() => aBuilder.Perform(new ProgressRange()));
    }

    // ------------------------------------------------------------------------------------------- IMeshTools_ShapeExplorer

    private sealed class RecordingVisitor : ShapeVisitor
    {
        public readonly List<Shape> Visited = [];

        public override void Visit(in Face theFace) => Visited.Add(theFace);

        public override void Visit(in Edge theEdge) => Visited.Add(theEdge);
    }

    [Fact]
    public void ShapeExplorer_visits_free_edges_then_each_face_once()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        Edge aFree = TestShapes.Frozen(TestShapes.Edge(new Point3D(5, 0, 0), new Point3D(6, 0, 0)));
        var aTrsf = new Transform3D();
        aTrsf = aTrsf.WithTranslation(new Vector3D(0, 0, 10));
        Edge aMovedFree = ShapeCasts.Edge(aFree.Moved(new Placement(aTrsf)));
        Face aFace = Faces(aBox)[0];

        Compound aCompound = Builder.MakeCompound([aMovedFree, aBox, aFace.Reversed()]);
        var aVisitor = new RecordingVisitor();
        new ShapeExplorer(aCompound).Accept(aVisitor);

        // the free edge first, at the identity location
        Assert.Equal(ShapeType.Edge, aVisitor.Visited[0].Type);
        Assert.True(aVisitor.Visited[0].IsSame(aFree));
        Assert.True(aVisitor.Visited[0].Placement.IsIdentity);

        // then each face once (the reversed copy of a box face is the same face), forward, after its edges
        List<Shape> aFaces = aVisitor.Visited.Where(s => s.Type == ShapeType.Face).ToList();
        Assert.Equal(6, aFaces.Count);
        Assert.All(aFaces, f => Assert.Equal(Forward, f.Orientation));
        Assert.Equal(1 + 6 * 4 + 6, aVisitor.Visited.Count);
        int aFirstFace = aVisitor.Visited.FindIndex(s => s.Type == ShapeType.Face);
        Assert.Equal(5, aFirstFace); // the free edge and the first face's four edges
    }

    // ------------------------------------------------------------------------------------------- BRepMesh_ShapeTool

    [Fact]
    public void ShapeTool_tolerances_ranges_and_points()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(2, 3, 4));
        Face aFace = Faces(aBox)[0];
        double aMax = Tool.Tolerance(aFace);
        for (var anExp = new Explorer(aFace, ShapeType.Vertex); anExp.More(); anExp.Next())
        {
            aMax = Math.Max(aMax, Tool.Tolerance(ShapeCasts.Vertex(anExp.Current())));
        }
        for (var anExp = new Explorer(aFace, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            aMax = Math.Max(aMax, Tool.Tolerance(ShapeCasts.Edge(anExp.Current())));
        }
        Assert.Equal(aMax, ShapeTool.MaxFaceTolerance(aFace));

        double aDim = -1.0;
        ShapeTool.BoxMaxDimension(new Box(), ref aDim);
        Assert.Equal(-1.0, aDim); // void box: unchanged
        var aBnd = new Box(new Point3D(0, 0, 0), new Point3D(1, 5, 2));
        ShapeTool.BoxMaxDimension(aBnd, ref aDim);
        Assert.Equal(5.0, aDim);

        // A located edge: the 3D range is the located curve's, reversed with the orientation if asked.
        var aTrsf = new Transform3D();
        aTrsf = aTrsf.WithTranslation(new Vector3D(0, 0, 7));
        Edge anEdge = TestShapes.Frozen(TestShapes.Edge(new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), 1, 3));
        Edge aMoved = ShapeCasts.Edge(anEdge.Moved(new Placement(aTrsf)).Reversed());
        Assert.True(ShapeTool.Range(aMoved, out Curve3D? aCurve, out double aF, out double aL));
        Assert.Equal((1.0, 3.0), (aF, aL));
        Assert.True(aCurve!.Value(aF).IsEqual(new Point3D(1, 0, 7), 1e-15));
        Assert.True(ShapeTool.Range(aMoved, out _, out aF, out aL, true));
        Assert.Equal((3.0, 1.0), (aF, aL));

        // Pcurve range and UV points on a box face.
        Edge aBoxEdge = ShapeCasts.Edge(new Explorer(aFace, ShapeType.Edge).Current());
        Assert.True(ShapeTool.Range(aBoxEdge, aFace, out Curve2D? aPC, out aF, out aL));
        Assert.NotNull(aPC);
        Assert.True(ShapeTool.UVPoints(aBoxEdge, aFace, out Point2D aUV1, out Point2D aUV2));
        Assert.True(aUV1.IsEqual(aPC!.Value(aF), 0.0));
        Assert.True(aUV2.IsEqual(aPC.Value(aL), 0.0));
        // (an edge not on a plane still has a pcurve there: BRep_Tool projects it)
        Assert.True(ShapeTool.Range(anEdge, aFace, out _, out _, out _));

        Assert.True(ShapeTool.UseLocation(new Point3D(1, 2, 3), new Placement(aTrsf)).IsEqual(new Point3D(1, 2, 10), 0.0));
        Assert.True(ShapeTool.UseLocation(new Point3D(1, 2, 3), new Placement()).IsEqual(new Point3D(1, 2, 3), 0.0));
    }

    [Fact]
    public void ShapeTool_CheckAndUpdateFlags()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(2, 3, 4));
        Face aFace = ShapeCasts.Face(Faces(aBox)[0].Oriented(Forward));
        Edge aBoxEdge = ShapeCasts.Edge(new Explorer(aFace, ShapeType.Edge).Current());
        var aDFace = new FaceData(aFace);
        var aDEdge = new EdgeData(aBoxEdge);
        PCurveModel aPCurve = aDEdge.AddPCurve(aDFace, aBoxEdge.Orientation);
        bool aSameRange = aDEdge.GetSameRange();
        ShapeTool.CheckAndUpdateFlags(aDEdge, aPCurve);
        Assert.Equal(aSameRange, aDEdge.GetSameRange());
        Assert.False(aDEdge.GetDegenerated());

        // Flags already at their worst: nothing to do.
        aDEdge.SetSameParam(false);
        aDEdge.SetSameRange(false);
        aDEdge.SetDegenerated(true);
        ShapeTool.CheckAndUpdateFlags(aDEdge, aPCurve);
        Assert.True(aDEdge.GetDegenerated());

        // A closed edge shorter than its vertex tolerance is degenerated.
        Vertex aV = TestShapes.Vertex(new Point3D(0, 0, 0));
        var aB = new BrepBuilder();
        aB.UpdateVertex(aV, 0.1);
        CircleCurve3D aTiny = new(new Frame3D(new Point3D(0.001, 0, 0), global::Kerf.Engine.Geometry.Constants.DZ()), 0.001);
        Edge aLoop = TestShapes.Edge(aTiny, 0, 2 * Math.PI, aV, ShapeCasts.Vertex(aV.Reversed()));
        aB.UpdateEdge(aLoop, new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0)), aFace, 1e-7);
        var aDLoop = new EdgeData(aLoop);
        PCurveModel aLoopPCurve = aDLoop.AddPCurve(aDFace, Forward);
        Assert.False(aDLoop.GetDegenerated());
        ShapeTool.CheckAndUpdateFlags(aDLoop, aLoopPCurve);
        Assert.True(aDLoop.GetDegenerated());
    }

    [Fact]
    public void ShapeTool_attaches_frozen_and_moves_nodes_by_the_face_location()
    {
        Solid aBox = TestShapes.Box(1, 1, 1); // unfrozen: ported code meshing in place
        var aTrsf = new Transform3D();
        aTrsf = aTrsf.WithTranslation(new Vector3D(10, 0, 0));
        Face aFace = Faces(aBox)[0];
        Face aMoved = ShapeCasts.Face(aFace.Moved(new Placement(aTrsf)));

        var aTri = new Triangulation(3, 1, false);
        aTri.SetNode(1, new Point3D(10, 0, 0));
        aTri.SetNode(2, new Point3D(11, 0, 0));
        aTri.SetNode(3, new Point3D(10, 1, 0));
        aTri.SetTriangle(1, new Triangle1(1, 2, 3));
        ShapeTool.AddInFace(aMoved, aTri);

        // The nodes are in the face's definition: the location is taken off.
        Triangulation aStored = Tool.Triangulation(aFace, out Placement aLoc)!;
        Assert.Same(aTri, aStored);
        Assert.True(aLoc.IsIdentity);
        Assert.True(aStored.Node(2).IsEqual(new Point3D(1, 0, 0), 1e-15));
        // frozen once attached (decision 17304; decision 488 left it writable until published)
        Assert.True(aStored.IsFrozen);
        Assert.ThrowsAny<DomainErrorException>(() => aStored.Deflection(0.5));

        // Polygons: on the triangulation, a seam's pair, a 3D polygon; all frozen; removed by Nullify.
        Edge anEdge = ShapeCasts.Edge(new Explorer(aFace, ShapeType.Edge).Current());
        var aNodes = new NCollection_Array1<int>(1, 2);
        aNodes[1] = 1;
        aNodes[2] = 2;
        var aPoly = new PolygonOnTriangulation(aNodes);
        ShapeTool.UpdateEdge(anEdge, aPoly, aTri, new Placement());
        Assert.Same(aPoly, Tool.PolygonOnTriangulation(anEdge, aTri, new Placement()));
        Assert.True(aPoly.IsFrozen);

        var aPoly2 = new PolygonOnTriangulation(aNodes);
        Edge anOther = ShapeCasts.Edge(new Explorer(Faces(aBox)[1], ShapeType.Edge).Current());
        ShapeTool.UpdateEdge(anOther, aPoly, aPoly2, aTri, new Placement());
        Assert.True(aPoly2.IsFrozen);

        var aP3dNodes = new NCollection_Array1<Point3D>(1, 2);
        aP3dNodes[1] = new Point3D(0, 0, 0);
        aP3dNodes[2] = new Point3D(1, 0, 0);
        var aP3d = new Polygon3D(aP3dNodes);
        ShapeTool.UpdateEdge(anEdge, aP3d);
        Assert.Same(aP3d, Tool.Polygon3D(anEdge, out _));
        Assert.True(aP3d.IsFrozen);

        // Publication freezes the shape.
        aBox.TShape()!.FreezeGraph();
        Assert.ThrowsAny<DomainErrorException>(() => ShapeTool.NullifyFace(aFace));
        Assert.ThrowsAny<DomainErrorException>(() => ShapeTool.NullifyEdge(anEdge, new Placement()));
    }

    [Fact]
    public void ShapeTool_nullifies_meshes_on_an_exact_copy()
    {
        Solid aBox = TestShapes.Box(1, 1, 1);
        foreach (Face aF in Faces(aBox))
        {
            FanMesh(aF);
        }
        TestShapes.Frozen(aBox);

        Shape aCopy = TestCopies.ExactCopyForWrite(aBox, out _);
        Face aFace = Faces(aCopy)[0];
        Triangulation aTri = Triangulation(aFace)!;
        Assert.True(aTri.IsFrozen); // shared with the frozen input

        Edge anEdge = ShapeCasts.Edge(new Explorer(aFace, ShapeType.Edge).Current());
        Assert.NotNull(Tool.PolygonOnTriangulation(anEdge, aTri, aFace.Placement));
        ShapeTool.NullifyEdge(anEdge, aTri, aFace.Placement);
        Assert.Null(Tool.PolygonOnTriangulation(anEdge, aTri, aFace.Placement));
        ShapeTool.NullifyFace(aFace);
        Assert.Null(Triangulation(aFace));

        // the input keeps its meshes
        Assert.Same(aTri, Triangulation(Faces(aBox)[0]));
    }
}
