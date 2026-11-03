// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Measurement;
using Kerf.Engine.Meshing;
using Kerf.Engine.Meshing.Model;
using Kerf.Engine.Meshing.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Engine.Meshing.Model.StatusModel;
using static Kerf.Engine.Foundation.Status;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Kernel.ModelingAlgorithms.Tests.BRepMeshModelKit;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Unit 4C: the discrete model (BRepMesh_ModelBuilder, BRepMesh_ShapeVisitor), the edges' discretisation
/// (BRepMesh_EdgeDiscret, CurveTessellator, EdgeTessellationExtractor), the healer (ModelHealer, FaceChecker), and the
/// pre- and post-processors, on BRepPrimAPI primitives, built faces and data/occ shapes.
/// </summary>
public class BRepMeshModelTests
{
    private static Shape Box() => new MakeBox(1, 2, 3).Solid();

    private static List<Shape> Explore(in Shape theShape, ShapeType theType)
    {
        var aList = new List<Shape>();
        for (var anExp = new Explorer(theShape, theType); anExp.More(); anExp.Next())
        {
            aList.Add(anExp.Current());
        }
        return aList;
    }

    // ------------------------------------------------------------------------------------------------------- model

    [Fact]
    public void Box_model_has_its_faces_wires_edges_and_pcurves_with_their_orientations()
    {
        Shape aBox = Box();
        ModelModel aModel = Model(aBox, Params(0.1), Stage.Built);

        Assert.Equal(6, aModel.FacesNb());
        Assert.Equal(12, aModel.EdgesNb());
        for (int i = 0; i < aModel.FacesNb(); i++)
        {
            FaceModel aDFace = aModel.GetFace(i);
            Assert.Equal(Forward, aDFace.GetFace().Orientation());
            Assert.True(aDFace.IsEqual(StatusModel.NoError));
            Assert.Equal(1, aDFace.WiresNb());
            WireModel aDWire = aDFace.GetWire(0);
            Assert.Equal(4, aDWire.EdgesNb());

            List<Shape> aFaceEdges = Explore(aDFace.GetFace(), ShapeType.Edge);
            for (int e = 0; e < aDWire.EdgesNb(); e++)
            {
                EdgeModel aDEdge = aDWire.GetEdge(e);
                Orientation anOri = aDWire.GetEdgeOrientation(e);

                // The orientation is the edge's in the face; the pcurve on this face has it.
                Assert.Contains(aFaceEdges, s => s.IsSame(aDEdge.GetEdge()) && s.Orientation() == anOri);
                PCurveModel aPCurve = aDEdge.GetPCurve(aDFace, anOri);
                Assert.Same(aDFace, aPCurve.GetFace());
                Assert.Equal(anOri, aPCurve.GetOrientation());

                // The wire is ordered: each edge starts where the previous one ends.
                EdgeModel aNext = aDWire.GetEdge((e + 1) % 4);
                Vertex aLast = Exploration.LastVertex(ShapeCasts.Edge(aDEdge.GetEdge().Oriented(anOri)), true);
                Vertex aFirst = Exploration.FirstVertex(ShapeCasts.Edge(aNext.GetEdge().Oriented(aDWire.GetEdgeOrientation((e + 1) % 4))), true);
                Assert.True(aLast.IsSame(aFirst));
            }
        }

        // Each edge once, with a pcurve on each of its two faces.
        for (int i = 0; i < aModel.EdgesNb(); i++)
        {
            EdgeModel aDEdge = aModel.GetEdge(i);
            Assert.False(aDEdge.IsFree);
            Assert.Equal(2, aDEdge.PCurvesNb());
            Assert.NotSame(aDEdge.GetPCurve(0).GetFace(), aDEdge.GetPCurve(1).GetFace());
            Assert.Equal(1, aDEdge.GetPCurves(aDEdge.GetPCurve(0).GetFace()).Size());
        }
    }

    [Fact]
    public void Seam_edges_have_two_pcurves_on_their_face_and_degenerated_edges_are_flagged()
    {
        // Cylinder: the lateral face's wire is ordered 1, 4, 3, 2 by ShapeAnalysis_Wire (DONE5, decision 518).
        ModelModel aModel = Model(new ApiMakeCylinder(2, 5).Solid(), Params(0.05), Stage.EdgesDiscretized);
        Assert.Equal(3, aModel.FacesNb());
        Assert.Equal(3, aModel.EdgesNb());
        EdgeModel? aSeam = null;
        for (int i = 0; i < aModel.FacesNb(); i++)
        {
            FaceModel aDFace = aModel.GetFace(i);
            Assert.True(aDFace.IsEqual(StatusModel.NoError));
            if (aDFace.GetWire(0).EdgesNb() == 4)
            {
                WireModel aDWire = aDFace.GetWire(0);
                for (int e = 0; e < 4; e++)
                {
                    EdgeModel aDEdge = aDWire.GetEdge(e);
                    if (aDEdge.PCurvesNb() == 2 && ReferenceEquals(aDEdge.GetPCurve(0).GetFace(), aDEdge.GetPCurve(1).GetFace()))
                    {
                        aSeam = aDEdge;
                    }
                }
                Assert.NotNull(aSeam);
                Assert.NotSame(aSeam.GetPCurve(aDFace, Forward), aSeam.GetPCurve(aDFace, Reversed));
                Assert.Equal(Forward, aSeam.GetPCurve(aDFace, Forward).GetOrientation());
                Assert.Equal(Reversed, aSeam.GetPCurve(aDFace, Reversed).GetOrientation());
            }
        }

        // The seam's two pcurves are at u = 0 and u = 2pi, with the same parameters.
        Assert.NotNull(aSeam);
        PCurveModel aP1 = aSeam!.GetPCurve(0), aP2 = aSeam.GetPCurve(1);
        Assert.Equal(aSeam.GetCurve().ParametersNb(), aP1.ParametersNb());
        Assert.Equal(aP1.ParametersNb(), aP2.ParametersNb());
        for (int k = 0; k < aP1.ParametersNb(); k++)
        {
            Assert.Equal(aP1.GetParameter(k), aP2.GetParameter(k));
            Assert.Equal(2 * Math.PI, Math.Abs(aP1.GetPoint(k).X() - aP2.GetPoint(k).X()), 1e-12);
        }

        // Sphere: the poles' degenerated edges.
        ModelModel aSphere = Model(new MakeSphere(3).Solid(), Params(0.05), Stage.EdgesDiscretized);
        int aDegenerated = 0;
        for (int i = 0; i < aSphere.EdgesNb(); i++)
        {
            EdgeModel aDEdge = aSphere.GetEdge(i);
            if (Tool.Degenerated(aDEdge.GetEdge()))
            {
                aDegenerated++;
                Assert.True(aDEdge.GetDegenerated());
                Assert.Equal(2, aDEdge.GetCurve().ParametersNb()); // the ends only
                Assert.Equal(aDEdge.GetCurve().ParametersNb(), aDEdge.GetPCurve(0).ParametersNb());
            }
            Assert.False(aDEdge.IsSet(StatusModel.Failure));
        }
        Assert.Equal(2, aDegenerated);
    }

    [Fact]
    public void Outer_wire_comes_first_whatever_the_face_order()
    {
        Face aFace = ShapeAnalysis_WireTests.FaceWithHoles(out Wire anOuter, out Wire aHole1, out Wire aHole2);
        ModelModel aModel = Model(aFace, Params(0.1), Stage.Built);
        Assert.Equal(1, aModel.FacesNb());
        FaceModel aDFace = aModel.GetFace(0);
        Assert.Equal(3, aDFace.WiresNb());
        Assert.True(aDFace.GetWire(0).GetWire().IsSame(anOuter));
        Assert.True(aDFace.GetWire(1).GetWire().IsSame(aHole1));
        Assert.True(aDFace.GetWire(2).GetWire().IsSame(aHole2));
        Assert.True(aDFace.IsEqual(StatusModel.NoError));
        Assert.Equal(12, aModel.EdgesNb());
    }

    [Fact]
    public void Free_edges_come_first_and_the_max_size_follows_the_parameters()
    {
        var aB = new BrepBuilder();
        Compound aComp = new();
        aB.MakeCompound(ref aComp);
        aB.Add(aComp, Box());
        Edge aFree = new MakeEdge(new Point3D(0, 0, 0), new Point3D(10, 0, 0)).Edge();
        aB.Add(aComp, aFree);

        ModelModel aModel = Model(aComp, Params(0.1), Stage.Built);
        Assert.Equal(13, aModel.EdgesNb());
        Assert.True(aModel.GetEdge(0).IsFree);
        Assert.True(aModel.GetEdge(0).GetEdge().IsSame(aFree));
        Assert.Equal(0.1, aModel.GetMaxSize()); // max(Deflection, DeflectionInterior)

        var aBox = new Box();
        Bounds.Add(aComp, ref aBox, false);
        double aMax = 0.0;
        ShapeTool.BoxMaxDimension(aBox, ref aMax);
        ModelModel aRelative = Model(aComp, Params(0.1, isRelative: true), Stage.Built);
        Assert.Equal(aMax, aRelative.GetMaxSize());
        Assert.True(aMax > 10.0);
    }

    [Fact]
    public void Empty_shape_gives_no_model_and_Fail1()
    {
        var aB = new BrepBuilder();
        Compound aComp = new();
        aB.MakeCompound(ref aComp);
        var aBuilder = new MeshModelBuilder();
        Assert.Null(aBuilder.Perform(aComp, Params(0.1)));
        Assert.True(aBuilder.GetStatus().IsSet(Fail1));

        ToolContext aContext = Context(aComp, Params(0.1));
        Assert.False(aContext.BuildModel());
        Assert.False(aContext.DiscretizeEdges());
        Assert.False(new EdgeDiscret().Perform(null, Params(0.1), new global::Kerf.Engine.Foundation.ProgressRange()));
        Assert.False(new ModelHealer().Perform(null, Params(0.1), new global::Kerf.Engine.Foundation.ProgressRange()));
        Assert.False(new ModelPreProcessor().Perform(null, Params(0.1), new global::Kerf.Engine.Foundation.ProgressRange()));
        Assert.False(new ModelPostProcessor().Perform(null, Params(0.1), new global::Kerf.Engine.Foundation.ProgressRange()));
    }

    [Fact]
    public void A_frozen_shape_without_meshes_is_only_read()
    {
        foreach (Shape aShape in new[]
                 {
                     TestShapes.Frozen(Box()), TestShapes.Frozen((Shape)new ApiMakeCone(2, 1, 4).Solid()),
                     ReadDataOcc("wing.brep"), ReadDataOcc("face1.brep"),
                 })
        {
            Assert.True(aShape.TShape()!.IsFrozen);
            var aSnapshot = ShapeSnapshot.Of(aShape);
            ModelModel aModel = Model(aShape, Params(0.05, isRelative: true));
            Assert.True(aModel.FacesNb() > 0);
            aSnapshot.AssertUnchanged(aShape);
        }
    }

    [Fact]
    public void Faces_of_data_occ_shapes_are_modelled_with_all_their_edges()
    {
        foreach (string aName in new[] { "solid.brep", "wedge_ok.brep", "face1.brep", "Ball.brep", "mal_vis.brep", "wing.brep" })
        {
            Shape aShape = ReadDataOcc(aName);
            ModelModel aModel = Model(aShape, Params(0.05, isRelative: true), Stage.Built);

            var aFaces = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
            var anEdges = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
            foreach (Shape aF in Explore(aShape, ShapeType.Face))
            {
                aFaces.Add(aF.Located(new Placement()));
            }
            Exploration.MapShapes(aShape, ShapeType.Edge, anEdges);
            Assert.Equal(aFaces.Extent(), aModel.FacesNb());
            Assert.Equal(anEdges.Extent(), aModel.EdgesNb());

            for (int i = 0; i < aModel.FacesNb(); i++)
            {
                FaceModel aDFace = aModel.GetFace(i);
                Assert.True(aDFace.IsValid, $"{aName} face {i}: {aDFace.GetStatusMask():X}");
                int aWireEdges = 0;
                for (int w = 0; w < aDFace.WiresNb(); w++)
                {
                    aWireEdges += aDFace.GetWire(w).EdgesNb();
                }
                int aFaceEdges = Explore(aDFace.GetFace(), ShapeType.Edge).Count(e => e.Orientation() != External);
                Assert.Equal(aFaceEdges, aWireEdges);
            }
        }
    }

    // ------------------------------------------------------------------------------------------------------- edges

    [Fact]
    public void Straight_edges_get_their_ends_and_pcurves_their_points()
    {
        ModelModel aModel = Model(Box(), Params(0.1), Stage.EdgesDiscretized);
        for (int i = 0; i < aModel.EdgesNb(); i++)
        {
            EdgeModel aDEdge = aModel.GetEdge(i);
            Assert.True(aDEdge.IsEqual(StatusModel.NoError));
            Assert.Equal(0.1, aDEdge.GetDeflection());
            Assert.Equal(0.5, aDEdge.GetAngularDeflection());
            CurveModel aCurve = aDEdge.GetCurve();
            Assert.Equal(2, aCurve.ParametersNb());
            Exploration.Vertices(aDEdge.GetEdge(), out Vertex aV1, out Vertex aV2);
            Assert.Equal(Tool.Pnt(aV1), aCurve.GetPoint(0));
            Assert.Equal(Tool.Pnt(aV2), aCurve.GetPoint(1));

            for (int p = 0; p < aDEdge.PCurvesNb(); p++)
            {
                PCurveModel aPCurve = aDEdge.GetPCurve(p);
                Assert.Equal(2, aPCurve.ParametersNb());
                Curve2D aC2d = Tool.CurveOnSurface(ShapeCasts.Edge(aDEdge.GetEdge().Oriented(aPCurve.GetOrientation())),
                                                             aPCurve.GetFace().GetFace(), out _, out _)!;
                for (int k = 0; k < 2; k++)
                {
                    Assert.Equal(aCurve.GetParameter(k), aPCurve.GetParameter(k));
                    Assert.Equal(aC2d.Value(aPCurve.GetParameter(k)), aPCurve.GetPoint(k));
                }
            }
        }
    }

    [Fact]
    public void Circles_are_discretized_within_half_the_deflection()
    {
        const double R = 2.0;
        ModelModel aModel = Model(new ApiMakeCylinder(R, 5).Solid(), Params(0.01, 0.5), Stage.EdgesDiscretized);
        int aCircles = 0;
        for (int i = 0; i < aModel.EdgesNb(); i++)
        {
            EdgeModel aDEdge = aModel.GetEdge(i);
            if (new EdgeCurveAdaptor(aDEdge.GetEdge()).GetType() != CurveKind.Circle)
            {
                continue;
            }

            aCircles++;
            CurveModel aCurve = aDEdge.GetCurve();
            Assert.True(aCurve.ParametersNb() >= 4);
            Circle3D aCirc = new EdgeCurveAdaptor(aDEdge.GetEdge()).Circle();
            for (int k = 1; k < aCurve.ParametersNb(); k++)
            {
                Assert.True(aCurve.GetParameter(k) > aCurve.GetParameter(k - 1));
                Assert.Equal(R, aCirc.Distance(aCurve.GetPoint(k)) + R, 1e-12); // on the circle
                double aStep = aCurve.GetParameter(k) - aCurve.GetParameter(k - 1);
                double aSagitta = R * (1.0 - Math.Cos(aStep / 2.0));
                Assert.True(aSagitta <= 0.5 * aDEdge.GetDeflection() * (1.0 + 1e-9), $"sagitta {aSagitta}");
            }
            for (int p = 0; p < aDEdge.PCurvesNb(); p++)
            {
                Assert.Equal(aCurve.ParametersNb(), aDEdge.GetPCurve(p).ParametersNb());
            }
        }
        Assert.Equal(2, aCircles);
    }

    [Fact]
    public void Polygons_on_triangulation_that_fit_are_read_back()
    {
        foreach (string aName in new[] { "Room.brep", "solid.brep", "mal_tige.brep", "MODERN_MVCooker_1.brep" })
        {
            Shape aShape = ReadDataOcc(aName);
            var aSnapshot = ShapeSnapshot.Of(aShape);

            // A coarse deflection: every polygon with parameters fits (BRepMesh_Deflection.IsConsistent).
            ModelModel aModel = Model(aShape, Params(1000.0), Stage.EdgesDiscretized);
            int aRead = 0;
            for (int i = 0; i < aModel.EdgesNb(); i++)
            {
                EdgeModel aDEdge = aModel.GetEdge(i);
                if (aDEdge.IsFree || aDEdge.IsSet(StatusModel.Failure))
                {
                    continue;
                }

                // The polygon read back: the one of least deflection among the edge's faces.
                PolygonOnTriangulation? aBest = null;
                Placement aBestLoc = new();
                Triangulation? aBestTri = null;
                for (int p = 0; p < aDEdge.PCurvesNb(); p++)
                {
                    Triangulation? aTri = Tool.Triangulation(aDEdge.GetPCurve(p).GetFace().GetFace(), out Placement aLoc);
                    PolygonOnTriangulation? aPoly = aTri is null ? null : Tool.PolygonOnTriangulation(aDEdge.GetEdge(), aTri, aLoc);
                    if (aPoly is not null && aPoly.HasParameters && (aBest is null || aPoly.Deflection() < aBest.Deflection()))
                    {
                        (aBest, aBestLoc, aBestTri) = (aPoly, aLoc, aTri);
                    }
                }
                if (aBest is null)
                {
                    continue;
                }

                aRead++;
                Assert.Equal(aBest.Deflection(), aDEdge.GetDeflection());
                CurveModel aCurve = aDEdge.GetCurve();
                Assert.Equal(aBest.NodeCount, aCurve.ParametersNb());
                for (int k = 2; k < aBest.NodeCount; k++) // the ends are the vertices' points
                {
                    Point3D aNode = aBestTri!.Node(aBest.Node(k)).Transformed(aBestLoc.Transformation());
                    Assert.Equal(aNode, aCurve.GetPoint(k - 1));
                }
                if (aDEdge.GetSameParam())
                {
                    for (int k = 1; k <= aBest.NodeCount; k++)
                    {
                        Assert.Equal(aBest.Parameters()![k], aCurve.GetParameter(k - 1));
                    }
                }
            }
            Assert.True(aRead > 0, aName);

            // The faces' triangulations fit too: reused, and nothing is written into the (frozen) shape.
            ModelModel aPre = Model(aShape, Params(1000.0));
            for (int i = 0; i < aPre.FacesNb(); i++)
            {
                FaceModel aDFace = aPre.GetFace(i);
                if (Tool.Triangulation(aDFace.GetFace(), out _) is not null && !aDFace.IsSet(StatusModel.Outdated))
                {
                    Assert.True(aDFace.IsSet(StatusModel.Reused), $"{aName} face {i}");
                }
            }
            aSnapshot.AssertUnchanged(aShape);
        }
    }

    [Fact]
    public void Outdated_meshes_are_removed_from_a_copy_never_from_the_frozen_input()
    {
        Shape aShape = ReadDataOcc("solid.brep");
        Assert.True(aShape.TShape()!.IsFrozen);
        var aSnapshot = ShapeSnapshot.Of(aShape);

        // A finer deflection: every triangulation and polygon is outdated.
        ToolContext aContext = Context(aShape, Params(0.01));
        Assert.True(aContext.BuildModel() && aContext.DiscretizeEdges() && aContext.HealModel());
        // The reader's shapes are frozen and locked: the first removal throws (TopoDS_LockedShape), never writes.
        Exception anError = Assert.ThrowsAny<global::Kerf.Engine.Foundation.FailureException>(() => aContext.PreProcessModel());
        Assert.True(anError is global::Kerf.Engine.Foundation.ImmutableObjectException or LockedShapeException, global::Kerf.Engine.Foundation.OcctTypeNames.Of(anError.GetType()));
        aSnapshot.AssertUnchanged(aShape);

        Shape aCopy = BrepTools.ExactCopyForWrite(aShape, out _);
        var aFaceMeshes = new HashSet<Triangulation>(ReferenceEqualityComparer.Instance);
        foreach (Shape aF in Explore(aCopy, ShapeType.Face))
        {
            aFaceMeshes.Add(Tool.Triangulation(ShapeCasts.Face(aF), out _)!);
        }
        ModelModel aModel = Model(aCopy, Params(0.01));
        int anOutdated = 0;
        for (int i = 0; i < aModel.FacesNb(); i++)
        {
            FaceModel aDFace = aModel.GetFace(i);
            Assert.False(aDFace.IsSet(StatusModel.Reused));
            if (aDFace.IsSet(StatusModel.Outdated))
            {
                anOutdated++;
                Assert.Null(Tool.Triangulation(aDFace.GetFace(), out _));
            }
        }
        Assert.Equal(aModel.FacesNb(), anOutdated);
        // The edges' polygons on those triangulations are gone; the file's stale polygons, on triangulations no face
        // holds, stay, as in OCCT.
        int aStale = 0;
        foreach (Shape anE in Explore(aCopy, ShapeType.Edge))
        {
            for (int p = 1; ; p++)
            {
                Tool.PolygonOnTriangulation(ShapeCasts.Edge(anE), out PolygonOnTriangulation? aPoly,
                                                 out Triangulation? aTri, out _, p);
                if (aPoly is null)
                {
                    break;
                }
                Assert.DoesNotContain(aTri!, aFaceMeshes);
                aStale++;
            }
        }
        Assert.True(aStale > 0);
        aSnapshot.AssertUnchanged(aShape);
    }

    // ------------------------------------------------------------------------------------------------------- healer

    [Fact]
    public void Healer_reconnects_the_ends_of_pcurves()
    {
        ModelModel aModel = Model(Box(), Params(0.1), Stage.EdgesDiscretized);
        FaceModel aDFace = aModel.GetFace(0);
        WireModel aDWire = aDFace.GetWire(0);
        PCurveModel aPCurve = aDWire.GetEdge(0).GetPCurve(aDFace, aDWire.GetEdgeOrientation(0));
        Point2D anOriginal = aPCurve.GetPoint(0);
        aPCurve.GetPoint(0) = new Point2D(anOriginal.X() + 1e-3, anOriginal.Y() - 1e-3);

        Assert.True(new ModelHealer().Perform(aModel, Params(0.1), new global::Kerf.Engine.Foundation.ProgressRange()));
        Assert.True(aPCurve.GetPoint(0).Distance(anOriginal) < 1e-12);
        Assert.True(aDFace.IsEqual(StatusModel.NoError));
        Assert.Equal(0.1, aDWire.GetDeflection());
        Assert.Equal(0.1, aDFace.GetDeflection());
    }

    /// <summary>A disk of radius 10 with a hole of radius 1 whose far point is 0.01 from the rim, at the given angle.</summary>
    private static Face DiskWithHole(double theAngle)
    {
        var anOuter = new MakeWire(new MakeEdge(new Circle3D(new Frame3D(), 10)).Edge()).Wire();
        var aMaker = new MakeFace(anOuter, true);
        if (!double.IsNaN(theAngle))
        {
            var aDir = new Direction3D(Math.Cos(theAngle), Math.Sin(theAngle), 0);
            var aCenter = new Point3D(8.99 * aDir.X(), 8.99 * aDir.Y(), 0);
            var aHole = new MakeWire(new MakeEdge(new Circle3D(new Frame3D(aCenter, new Direction3D(0, 0, 1), aDir), 1)).Edge()).Wire();
            aMaker.Add(ShapeCasts.Wire(aHole.Reversed()));
        }
        return aMaker.Face();
    }

    [Fact]
    public void Healer_amplifies_edges_whose_polygons_intersect()
    {
        // The rim's polygon, alone: its first chord's middle is where the hole will touch.
        ModelModel aDisk = Model(DiskWithHole(double.NaN), Params(1.0), Stage.EdgesDiscretized);
        double anAngle = 0.5 * aDisk.GetEdge(0).GetCurve().GetParameter(1);

        Face aFace = DiskWithHole(anAngle);
        ModelModel aModel = Model(aFace, Params(1.0), Stage.EdgesDiscretized);
        FaceModel aDFace = aModel.GetFace(0);
        Assert.Equal(2, aDFace.WiresNb());
        Assert.True(aDFace.IsEqual(StatusModel.NoError));
        var aBefore = new double[aModel.EdgesNb()];
        for (int i = 0; i < aModel.EdgesNb(); i++)
        {
            aBefore[i] = aModel.GetEdge(i).GetDeflection();
        }

        // The polygons as discretized cross each other.
        Deflection.ComputeDeflection(aDFace.GetWire(0), Params(1.0));
        Deflection.ComputeDeflection(aDFace.GetWire(1), Params(1.0));
        Deflection.ComputeDeflection(aDFace, Params(1.0));
        var aChecker = new FaceChecker(aDFace, Params(1.0));
        Assert.False(aChecker.Perform());
        Assert.Equal(2, aChecker.GetIntersectingEdges()!.Count);

        // The healer refines both edges (deflection / 3 each round) until they don't.
        Assert.True(new ModelHealer().Perform(aModel, Params(1.0), new global::Kerf.Engine.Foundation.ProgressRange()));
        Assert.False(aDFace.IsSet(StatusModel.SelfIntersectingWire));
        Assert.False(aDFace.IsSet(StatusModel.Failure));
        for (int i = 0; i < aModel.EdgesNb(); i++)
        {
            Assert.True(aModel.GetEdge(i).GetDeflection() < aBefore[i] / 3.0 * 1.0000001);
        }
        Assert.True(new FaceChecker(aDFace, Params(1.0)).Perform());
    }

    /// <summary>A planar face bounded by a bow tie: (0,0), (1,1), (1,0), (0,1), its edges forward or reversed.</summary>
    private static Face BowTie(bool isReversed)
    {
        Wire aBowTie = new MakePolygon(new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(1, 0, 0),
                                                             new Point3D(0, 1, 0), true).Wire();
        var aB = new BrepBuilder();
        Face aFace = new();
        aB.MakeFace(ref aFace, new PlaneSurface(new Plane3D()), global::Kerf.Engine.Foundation.Precision.Confusion());
        aB.Add(aFace, isReversed ? aBowTie.Reversed() : aBowTie);
        return aFace;
    }

    [Fact]
    public void Healer_fails_a_face_whose_wire_really_intersects_itself()
    {
        ModelModel aModel = Model(BowTie(false), Params(0.01), Stage.Healed);
        FaceModel aDFace = aModel.GetFace(0);
        Assert.Equal(Forward, aDFace.GetWire(0).GetEdgeOrientation(0));
        Assert.True(aDFace.IsSet(StatusModel.SelfIntersectingWire));
        Assert.True(aDFace.IsSet(StatusModel.Failure));
        Assert.False(aDFace.IsValid);
        // Five rounds of amplification of the two crossing edges: 0.01 / 3^5.
        int anAmplified = 0;
        for (int i = 0; i < aModel.EdgesNb(); i++)
        {
            if (aModel.GetEdge(i).GetDeflection() < 0.01)
            {
                anAmplified++;
                Assert.Equal(0.01 / 3 / 3 / 3 / 3 / 3, aModel.GetEdge(i).GetDeflection(), 1e-15);
            }
        }
        Assert.Equal(2, anAmplified);
    }

    [Fact]
    public void Face_checker_misses_a_crossing_of_reversed_edges_as_OCCT_does()
    {
        // OCCT: FaceChecker's loop-area test sums the segments' second points in each pcurve's parametric order, not the
        // wire's. Along reversed edges the bow tie's loop measures zero area, and the crossing is taken for a tangency
        // (occt-issues.md, 4C).
        ModelModel aModel = Model(BowTie(true), Params(0.01), Stage.Healed);
        FaceModel aDFace = aModel.GetFace(0);
        Assert.Equal(Reversed, aDFace.GetWire(0).GetEdgeOrientation(0));
        Assert.True(aDFace.IsEqual(StatusModel.NoError));
        Assert.True(new FaceChecker(aDFace, Params(0.01)).Perform());
    }

    // ------------------------------------------------------------------------------------------------ pre-processor

    [Fact]
    public void Cone_seams_get_the_steps_of_the_cone_splitter()
    {
        ModelModel aEdges = Model(new ApiMakeCone(2, 1, 4).Solid(), Params(0.01), Stage.Healed);
        ModelModel aModel = Model(new ApiMakeCone(2, 1, 4).Solid(), Params(0.01));
        int aSeams = 0;
        for (int i = 0; i < aModel.EdgesNb(); i++)
        {
            EdgeModel aDEdge = aModel.GetEdge(i);
            if (aDEdge.PCurvesNb() != 2 || !ReferenceEquals(aDEdge.GetPCurve(0).GetFace(), aDEdge.GetPCurve(1).GetFace()))
            {
                continue;
            }

            aSeams++;
            Assert.Equal(2, aEdges.GetEdge(i).GetCurve().ParametersNb());
            CurveModel aCurve = aDEdge.GetCurve();
            Assert.True(aCurve.ParametersNb() > 2);
            Assert.Equal(aCurve.ParametersNb(), aDEdge.GetPCurve(0).ParametersNb());
            Assert.Equal(aCurve.ParametersNb(), aDEdge.GetPCurve(1).ParametersNb());
            double aStep = aCurve.GetParameter(1) - aCurve.GetParameter(0);
            for (int k = 1; k < aCurve.ParametersNb() - 1; k++)
            {
                Assert.Equal(aStep, aCurve.GetParameter(k) - aCurve.GetParameter(k - 1), 1e-12);
                Assert.Equal(aCurve.GetParameter(k), aDEdge.GetPCurve(0).GetParameter(k));
                Assert.Equal(aCurve.GetParameter(k), aDEdge.GetPCurve(1).GetParameter(k));
            }
            // The face had no triangulation: it isn't outdated.
            Assert.False(aDEdge.GetPCurve(0).GetFace().IsSet(StatusModel.Outdated));
        }
        Assert.Equal(1, aSeams);
    }

    // ----------------------------------------------------------------------------------------------- post-processor

    [Fact]
    public void Post_processor_commits_polygons_and_reuse_gives_back_the_mesh()
    {
        var aB = new BrepBuilder();
        Compound aComp = new();
        aB.MakeCompound(ref aComp);
        aB.Add(aComp, new ApiMakeCylinder(1, 2).Solid());
        aB.Add(aComp, new MakeEdge(new Circle3D(new Frame3D(new Point3D(5, 0, 0), new Direction3D(0, 0, 1)), 1), 0, 2).Edge());
        Shape anInput = TestShapes.Frozen((Shape)aComp);
        var aSnapshot = ShapeSnapshot.Of(anInput);

        ToolParameters aParams = Params(0.05);
        var aMesh = new IncrementalMesh();
        aMesh.SetShape(anInput);
        aMesh.ChangeParameters() = aParams;
        var aFaces = new FanFaceDiscret();
        ToolContext aContext = Context(new Shape(), aParams, aFaces);
        aMesh.Perform(aContext);
        Assert.True(aMesh.IsDone);
        Assert.Equal(3, aFaces.Meshed);
        aSnapshot.AssertUnchanged(anInput);

        Shape aResult = aMesh.Shape();
        Assert.True(aResult.TShape()!.IsFrozen);

        // Each face's triangulation, with the deflection measured and the parameters it was made with.
        ModelModel aModel = aContext.GetModel()!; // of the copy, whose meshed sub-shapes the result holds
        foreach (Shape aF in Explore(aResult, ShapeType.Face))
        {
            Triangulation aTri = Tool.Triangulation(ShapeCasts.Face(aF), out _)!;
            Assert.True(aTri.IsFrozen);
            Assert.Equal(0.05, aTri.Parameters()!.Deflection());
            Assert.Equal(0.5, aTri.Parameters()!.Angle());
            Assert.Equal(aParams.MinSize, aTri.Parameters()!.MinSize());
        }

        // Each edge's polygons: its discrete pcurves' parameters, nodes that are its points, its deflection.
        int aPolygons = 0;
        var anEdges = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(aResult, ShapeType.Edge, anEdges);
        for (int anEdgeIt = 1; anEdgeIt <= anEdges.Extent(); anEdgeIt++)
        {
            Edge anEdge = ShapeCasts.Edge(anEdges.FindKey(anEdgeIt));
            EdgeModel? aDEdge = null;
            for (int i = 0; i < aModel.EdgesNb() && aDEdge is null; i++)
            {
                if (aModel.GetEdge(i).GetEdge().IsSame(anEdge))
                {
                    aDEdge = aModel.GetEdge(i);
                }
            }
            Assert.NotNull(aDEdge);
            CurveModel aCurve = aDEdge!.GetCurve();
            for (int p = 1; ; p++)
            {
                Tool.PolygonOnTriangulation(anEdge, out PolygonOnTriangulation? aPoly, out Triangulation? aTri,
                                                 out Placement aLoc, p);
                if (aPoly is null)
                {
                    break;
                }
                aPolygons++;
                Assert.True(aPoly.IsFrozen);
                Assert.Equal(aDEdge.GetDeflection(), aPoly.Deflection());
                Assert.Equal(aCurve.ParametersNb(), aPoly.NodeCount);
                for (int k = 1; k <= aPoly.NodeCount; k++)
                {
                    Assert.Equal(aCurve.GetParameter(k - 1), aPoly.Parameters()![k]);
                    Assert.True(aTri!.Node(aPoly.Node(k)).Transformed(aLoc.Transformation()).Distance(aCurve.GetPoint(k - 1)) < 1e-12);
                }
            }
            if (Tool.Degenerated(anEdge))
            {
                continue;
            }
            if (aDEdge.IsFree)
            {
                Polygon3D aPoly3D = Tool.Polygon3D(anEdge, out _)!;
                Assert.True(aPoly3D.IsFrozen);
                Assert.Equal(aCurve.ParametersNb(), aPoly3D.NodeCount);
                Assert.Equal(aDEdge.GetDeflection(), aPoly3D.Deflection());
            }
        }
        Assert.Equal(5, aPolygons); // two circles on two faces each, the seam's pair once

        // Meshing the result again with the same parameters reuses everything: the result itself comes back.
        var aRemesh = new IncrementalMesh();
        aRemesh.SetShape(aResult);
        aRemesh.ChangeParameters() = aParams;
        var aNoFaces = new FanFaceDiscret();
        aRemesh.Perform(Context(new Shape(), aParams, aNoFaces));
        Assert.Equal(0, aNoFaces.Meshed);
        Assert.Same(aResult.TShape(), aRemesh.Shape().TShape());
        Assert.Equal((int)StatusModel.Reused, aRemesh.GetStatusFlags());

        // A finer deflection replaces them on a copy.
        var aFiner = new IncrementalMesh();
        aFiner.SetShape(aResult);
        aFiner.ChangeParameters() = Params(0.01);
        var aFinerFaces = new FanFaceDiscret();
        aFiner.Perform(Context(new Shape(), Params(0.01), aFinerFaces));
        Assert.Equal(3, aFinerFaces.Meshed);
        Assert.NotSame(aResult.TShape(), aFiner.Shape().TShape());
        foreach (Shape aF in Explore(aFiner.Shape(), ShapeType.Face))
        {
            Assert.Equal(0.01, Tool.Triangulation(ShapeCasts.Face(aF), out _)!.Parameters()!.Deflection());
        }
    }

    [Fact]
    public void Seam_polygons_are_stored_in_the_model_order_as_OCCT_does()
    {
        // OCCT: ModelPostProcessor stores a seam's first discrete pcurve on the face as the polygon of the forward edge
        // and the last as the reversed edge's, whatever their orientations. On BRepPrimAPI's cylinder the wire is
        // ordered top, seam (reversed, u = 0), bottom, seam (forward, u = 2pi): the forward edge's polygon gets the
        // nodes at u = 0 (occt-issues.md, 4C).
        Shape anInput = TestShapes.Frozen((Shape)new ApiMakeCylinder(1, 2).Solid());
        ToolParameters aParams = Params(0.05);
        var aMesh = new IncrementalMesh();
        aMesh.SetShape(anInput);
        aMesh.ChangeParameters() = aParams;
        ToolContext aContext = Context(new Shape(), aParams, new FanFaceDiscret());
        aMesh.Perform(aContext);
        ModelModel aModel = aContext.GetModel()!;

        int aSeams = 0;
        for (int i = 0; i < aModel.EdgesNb(); i++)
        {
            EdgeModel aDEdge = aModel.GetEdge(i);
            if (aDEdge.PCurvesNb() != 2 || !ReferenceEquals(aDEdge.GetPCurve(0).GetFace(), aDEdge.GetPCurve(1).GetFace()))
            {
                continue;
            }

            aSeams++;
            PCurveModel aFirst = aDEdge.GetPCurve(0), aLast = aDEdge.GetPCurve(1);
            Assert.Equal(Reversed, aFirst.GetOrientation());
            Assert.Equal(0.0, aFirst.GetPoint(0).X(), 1e-12);
            Assert.Equal(2 * Math.PI, aLast.GetPoint(0).X(), 1e-12);

            Face aFace = aFirst.GetFace().GetFace();
            Triangulation aTri = Tool.Triangulation(aFace, out Placement aLoc)!;
            PolygonOnTriangulation aForward =
                Tool.PolygonOnTriangulation(ShapeCasts.Edge(aDEdge.GetEdge().Oriented(Forward)), aTri, aLoc)!;
            PolygonOnTriangulation aReversed =
                Tool.PolygonOnTriangulation(ShapeCasts.Edge(aDEdge.GetEdge().Oriented(Reversed)), aTri, aLoc)!;
            for (int k = 1; k <= aForward.NodeCount; k++)
            {
                Assert.Equal(aFirst.GetIndex(k - 1), aForward.Node(k));
                Assert.Equal(aLast.GetIndex(k - 1), aReversed.Node(k));
                Assert.Equal(0.0, aTri.UVNode(aForward.Node(k)).X(), 1e-12); // the reversed pcurve's side
            }
        }
        Assert.Equal(1, aSeams);
    }

    // ------------------------------------------------------------------------------------------------ determinism

    [Fact]
    public void Parallel_models_are_the_serial_model_bit_for_bit()
    {
        var aShapes = new List<(string, Shape)>
        {
            ("box", Box()),
            ("cylinder", new ApiMakeCylinder(2, 5).Solid()),
            ("cone", new ApiMakeCone(2, 1, 4).Solid()),
            ("sphere", new MakeSphere(3).Solid()),
            ("torus", new MakeTorus(5, 1).Solid()),
            ("holes", ShapeAnalysis_WireTests.FaceWithHoles(out _, out _, out _)),
            ("disk", DiskWithHole(0.12)),
        };
        foreach (string aFile in Directory.GetFiles(DataOcc, "*.brep").Order(StringComparer.Ordinal))
        {
            if (new FileInfo(aFile).Length < 600_000)
            {
                aShapes.Add((Path.GetFileName(aFile), ReadDataOcc(Path.GetFileName(aFile))));
            }
        }

        foreach ((string aName, Shape aShape) in aShapes)
        {
            foreach (ToolParameters aBase in new[] { Params(0.05, 0.5, isRelative: true), Params(0.5, 0.3) })
            {
                ToolParameters aSerial = aBase, aParallel = aBase;
                aSerial.InParallel = false;
                aParallel.InParallel = true;
                string aReference = Dump(Model(BrepTools.ExactCopyForWrite(aShape, out _), aSerial));
                for (int aRun = 0; aRun < 2; aRun++)
                {
                    string aDump = Dump(Model(BrepTools.ExactCopyForWrite(aShape, out _), aParallel));
                    Assert.True(aReference == aDump, $"{aName}: parallel run {aRun} differs");
                }
                using (global::Kerf.Engine.Foundation.OsdParallel.ForceSerialScope())
                {
                    Assert.True(aReference == Dump(Model(BrepTools.ExactCopyForWrite(aShape, out _), aParallel)), $"{aName}: forced serial");
                }
            }
        }
    }
}
