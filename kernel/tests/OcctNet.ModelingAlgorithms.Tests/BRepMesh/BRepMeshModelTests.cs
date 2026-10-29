// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.ModelingData.Tests;
using static Occt.IMeshData_Status;
using static Occt.Message_Status;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static OcctNet.ModelingAlgorithms.Tests.BRepMeshModelKit;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Unit 4C: the discrete model (BRepMesh_ModelBuilder, BRepMesh_ShapeVisitor), the edges' discretisation
/// (BRepMesh_EdgeDiscret, CurveTessellator, EdgeTessellationExtractor), the healer (ModelHealer, FaceChecker), and the
/// pre- and post-processors, on BRepPrimAPI primitives, built faces and data/occ shapes.
/// </summary>
public class BRepMeshModelTests
{
    private static TopoDS_Shape Box() => new BRepPrimAPI_MakeBox(1, 2, 3).Solid();

    private static List<TopoDS_Shape> Explore(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType)
    {
        var aList = new List<TopoDS_Shape>();
        for (var anExp = new TopExp_Explorer(theShape, theType); anExp.More(); anExp.Next())
        {
            aList.Add(anExp.Current());
        }
        return aList;
    }

    // ------------------------------------------------------------------------------------------------------- model

    [Fact]
    public void Box_model_has_its_faces_wires_edges_and_pcurves_with_their_orientations()
    {
        TopoDS_Shape aBox = Box();
        IMeshData_Model aModel = Model(aBox, Params(0.1), Stage.Built);

        Assert.Equal(6, aModel.FacesNb());
        Assert.Equal(12, aModel.EdgesNb());
        for (int i = 0; i < aModel.FacesNb(); i++)
        {
            IMeshData_Face aDFace = aModel.GetFace(i);
            Assert.Equal(TopAbs_FORWARD, aDFace.GetFace().Orientation());
            Assert.True(aDFace.IsEqual(IMeshData_NoError));
            Assert.Equal(1, aDFace.WiresNb());
            IMeshData_Wire aDWire = aDFace.GetWire(0);
            Assert.Equal(4, aDWire.EdgesNb());

            List<TopoDS_Shape> aFaceEdges = Explore(aDFace.GetFace(), TopAbs_EDGE);
            for (int e = 0; e < aDWire.EdgesNb(); e++)
            {
                IMeshData_Edge aDEdge = aDWire.GetEdge(e);
                TopAbs_Orientation anOri = aDWire.GetEdgeOrientation(e);

                // The orientation is the edge's in the face; the pcurve on this face has it.
                Assert.Contains(aFaceEdges, s => s.IsSame(aDEdge.GetEdge()) && s.Orientation() == anOri);
                IMeshData_PCurve aPCurve = aDEdge.GetPCurve(aDFace, anOri);
                Assert.Same(aDFace, aPCurve.GetFace());
                Assert.Equal(anOri, aPCurve.GetOrientation());

                // The wire is ordered: each edge starts where the previous one ends.
                IMeshData_Edge aNext = aDWire.GetEdge((e + 1) % 4);
                TopoDS_Vertex aLast = TopExp.LastVertex(TopoDS.Edge(aDEdge.GetEdge().Oriented(anOri)), true);
                TopoDS_Vertex aFirst = TopExp.FirstVertex(TopoDS.Edge(aNext.GetEdge().Oriented(aDWire.GetEdgeOrientation((e + 1) % 4))), true);
                Assert.True(aLast.IsSame(aFirst));
            }
        }

        // Each edge once, with a pcurve on each of its two faces.
        for (int i = 0; i < aModel.EdgesNb(); i++)
        {
            IMeshData_Edge aDEdge = aModel.GetEdge(i);
            Assert.False(aDEdge.IsFree());
            Assert.Equal(2, aDEdge.PCurvesNb());
            Assert.NotSame(aDEdge.GetPCurve(0).GetFace(), aDEdge.GetPCurve(1).GetFace());
            Assert.Equal(1, aDEdge.GetPCurves(aDEdge.GetPCurve(0).GetFace()).Size());
        }
    }

    [Fact]
    public void Seam_edges_have_two_pcurves_on_their_face_and_degenerated_edges_are_flagged()
    {
        // Cylinder: the lateral face's wire is ordered 1, 4, 3, 2 by ShapeAnalysis_Wire (DONE5, decision 518).
        IMeshData_Model aModel = Model(new BRepPrimAPI_MakeCylinder(2, 5).Solid(), Params(0.05), Stage.EdgesDiscretized);
        Assert.Equal(3, aModel.FacesNb());
        Assert.Equal(3, aModel.EdgesNb());
        IMeshData_Edge? aSeam = null;
        for (int i = 0; i < aModel.FacesNb(); i++)
        {
            IMeshData_Face aDFace = aModel.GetFace(i);
            Assert.True(aDFace.IsEqual(IMeshData_NoError));
            if (aDFace.GetWire(0).EdgesNb() == 4)
            {
                IMeshData_Wire aDWire = aDFace.GetWire(0);
                for (int e = 0; e < 4; e++)
                {
                    IMeshData_Edge aDEdge = aDWire.GetEdge(e);
                    if (aDEdge.PCurvesNb() == 2 && ReferenceEquals(aDEdge.GetPCurve(0).GetFace(), aDEdge.GetPCurve(1).GetFace()))
                    {
                        aSeam = aDEdge;
                    }
                }
                Assert.NotNull(aSeam);
                Assert.NotSame(aSeam.GetPCurve(aDFace, TopAbs_FORWARD), aSeam.GetPCurve(aDFace, TopAbs_REVERSED));
                Assert.Equal(TopAbs_FORWARD, aSeam.GetPCurve(aDFace, TopAbs_FORWARD).GetOrientation());
                Assert.Equal(TopAbs_REVERSED, aSeam.GetPCurve(aDFace, TopAbs_REVERSED).GetOrientation());
            }
        }

        // The seam's two pcurves are at u = 0 and u = 2pi, with the same parameters.
        Assert.NotNull(aSeam);
        IMeshData_PCurve aP1 = aSeam!.GetPCurve(0), aP2 = aSeam.GetPCurve(1);
        Assert.Equal(aSeam.GetCurve().ParametersNb(), aP1.ParametersNb());
        Assert.Equal(aP1.ParametersNb(), aP2.ParametersNb());
        for (int k = 0; k < aP1.ParametersNb(); k++)
        {
            Assert.Equal(aP1.GetParameter(k), aP2.GetParameter(k));
            Assert.Equal(2 * Math.PI, Math.Abs(aP1.GetPoint(k).X() - aP2.GetPoint(k).X()), 1e-12);
        }

        // Sphere: the poles' degenerated edges.
        IMeshData_Model aSphere = Model(new BRepPrimAPI_MakeSphere(3).Solid(), Params(0.05), Stage.EdgesDiscretized);
        int aDegenerated = 0;
        for (int i = 0; i < aSphere.EdgesNb(); i++)
        {
            IMeshData_Edge aDEdge = aSphere.GetEdge(i);
            if (BRep_Tool.Degenerated(aDEdge.GetEdge()))
            {
                aDegenerated++;
                Assert.True(aDEdge.GetDegenerated());
                Assert.Equal(2, aDEdge.GetCurve().ParametersNb()); // the ends only
                Assert.Equal(aDEdge.GetCurve().ParametersNb(), aDEdge.GetPCurve(0).ParametersNb());
            }
            Assert.False(aDEdge.IsSet(IMeshData_Failure));
        }
        Assert.Equal(2, aDegenerated);
    }

    [Fact]
    public void Outer_wire_comes_first_whatever_the_face_order()
    {
        TopoDS_Face aFace = ShapeAnalysis_WireTests.FaceWithHoles(out TopoDS_Wire anOuter, out TopoDS_Wire aHole1, out TopoDS_Wire aHole2);
        IMeshData_Model aModel = Model(aFace, Params(0.1), Stage.Built);
        Assert.Equal(1, aModel.FacesNb());
        IMeshData_Face aDFace = aModel.GetFace(0);
        Assert.Equal(3, aDFace.WiresNb());
        Assert.True(aDFace.GetWire(0).GetWire().IsSame(anOuter));
        Assert.True(aDFace.GetWire(1).GetWire().IsSame(aHole1));
        Assert.True(aDFace.GetWire(2).GetWire().IsSame(aHole2));
        Assert.True(aDFace.IsEqual(IMeshData_NoError));
        Assert.Equal(12, aModel.EdgesNb());
    }

    [Fact]
    public void Free_edges_come_first_and_the_max_size_follows_the_parameters()
    {
        var aB = new BRep_Builder();
        TopoDS_Compound aComp = new();
        aB.MakeCompound(ref aComp);
        aB.Add(aComp, Box());
        TopoDS_Edge aFree = new BRepBuilderAPI_MakeEdge(new gp_Pnt(0, 0, 0), new gp_Pnt(10, 0, 0)).Edge();
        aB.Add(aComp, aFree);

        IMeshData_Model aModel = Model(aComp, Params(0.1), Stage.Built);
        Assert.Equal(13, aModel.EdgesNb());
        Assert.True(aModel.GetEdge(0).IsFree());
        Assert.True(aModel.GetEdge(0).GetEdge().IsSame(aFree));
        Assert.Equal(0.1, aModel.GetMaxSize()); // max(Deflection, DeflectionInterior)

        var aBox = new Bnd_Box();
        BRepBndLib.Add(aComp, ref aBox, false);
        double aMax = 0.0;
        BRepMesh_ShapeTool.BoxMaxDimension(aBox, ref aMax);
        IMeshData_Model aRelative = Model(aComp, Params(0.1, isRelative: true), Stage.Built);
        Assert.Equal(aMax, aRelative.GetMaxSize());
        Assert.True(aMax > 10.0);
    }

    [Fact]
    public void Empty_shape_gives_no_model_and_Fail1()
    {
        var aB = new BRep_Builder();
        TopoDS_Compound aComp = new();
        aB.MakeCompound(ref aComp);
        var aBuilder = new BRepMesh_ModelBuilder();
        Assert.Null(aBuilder.Perform(aComp, Params(0.1)));
        Assert.True(aBuilder.GetStatus().IsSet(Message_Fail1));

        IMeshTools_Context aContext = Context(aComp, Params(0.1));
        Assert.False(aContext.BuildModel());
        Assert.False(aContext.DiscretizeEdges());
        Assert.False(new BRepMesh_EdgeDiscret().Perform(null, Params(0.1), new Message_ProgressRange()));
        Assert.False(new BRepMesh_ModelHealer().Perform(null, Params(0.1), new Message_ProgressRange()));
        Assert.False(new BRepMesh_ModelPreProcessor().Perform(null, Params(0.1), new Message_ProgressRange()));
        Assert.False(new BRepMesh_ModelPostProcessor().Perform(null, Params(0.1), new Message_ProgressRange()));
    }

    [Fact]
    public void A_frozen_shape_without_meshes_is_only_read()
    {
        foreach (TopoDS_Shape aShape in new[]
                 {
                     TestShapes.Frozen(Box()), TestShapes.Frozen((TopoDS_Shape)new BRepPrimAPI_MakeCone(2, 1, 4).Solid()),
                     ReadDataOcc("wing.brep"), ReadDataOcc("face1.brep"),
                 })
        {
            Assert.True(aShape.TShape()!.IsFrozen());
            var aSnapshot = ShapeSnapshot.Of(aShape);
            IMeshData_Model aModel = Model(aShape, Params(0.05, isRelative: true));
            Assert.True(aModel.FacesNb() > 0);
            aSnapshot.AssertUnchanged(aShape);
        }
    }

    [Fact]
    public void Faces_of_data_occ_shapes_are_modelled_with_all_their_edges()
    {
        foreach (string aName in new[] { "solid.brep", "wedge_ok.brep", "face1.brep", "Ball.brep", "mal_vis.brep", "wing.brep" })
        {
            TopoDS_Shape aShape = ReadDataOcc(aName);
            IMeshData_Model aModel = Model(aShape, Params(0.05, isRelative: true), Stage.Built);

            var aFaces = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            var anEdges = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            foreach (TopoDS_Shape aF in Explore(aShape, TopAbs_FACE))
            {
                aFaces.Add(aF.Located(new TopLoc_Location()));
            }
            TopExp.MapShapes(aShape, TopAbs_EDGE, anEdges);
            Assert.Equal(aFaces.Extent(), aModel.FacesNb());
            Assert.Equal(anEdges.Extent(), aModel.EdgesNb());

            for (int i = 0; i < aModel.FacesNb(); i++)
            {
                IMeshData_Face aDFace = aModel.GetFace(i);
                Assert.True(aDFace.IsValid(), $"{aName} face {i}: {aDFace.GetStatusMask():X}");
                int aWireEdges = 0;
                for (int w = 0; w < aDFace.WiresNb(); w++)
                {
                    aWireEdges += aDFace.GetWire(w).EdgesNb();
                }
                int aFaceEdges = Explore(aDFace.GetFace(), TopAbs_EDGE).Count(e => e.Orientation() != TopAbs_EXTERNAL);
                Assert.Equal(aFaceEdges, aWireEdges);
            }
        }
    }

    // ------------------------------------------------------------------------------------------------------- edges

    [Fact]
    public void Straight_edges_get_their_ends_and_pcurves_their_points()
    {
        IMeshData_Model aModel = Model(Box(), Params(0.1), Stage.EdgesDiscretized);
        for (int i = 0; i < aModel.EdgesNb(); i++)
        {
            IMeshData_Edge aDEdge = aModel.GetEdge(i);
            Assert.True(aDEdge.IsEqual(IMeshData_NoError));
            Assert.Equal(0.1, aDEdge.GetDeflection());
            Assert.Equal(0.5, aDEdge.GetAngularDeflection());
            IMeshData_Curve aCurve = aDEdge.GetCurve();
            Assert.Equal(2, aCurve.ParametersNb());
            TopExp.Vertices(aDEdge.GetEdge(), out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);
            Assert.Equal(BRep_Tool.Pnt(aV1), aCurve.GetPoint(0));
            Assert.Equal(BRep_Tool.Pnt(aV2), aCurve.GetPoint(1));

            for (int p = 0; p < aDEdge.PCurvesNb(); p++)
            {
                IMeshData_PCurve aPCurve = aDEdge.GetPCurve(p);
                Assert.Equal(2, aPCurve.ParametersNb());
                Geom2d_Curve aC2d = BRep_Tool.CurveOnSurface(TopoDS.Edge(aDEdge.GetEdge().Oriented(aPCurve.GetOrientation())),
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
        IMeshData_Model aModel = Model(new BRepPrimAPI_MakeCylinder(R, 5).Solid(), Params(0.01, 0.5), Stage.EdgesDiscretized);
        int aCircles = 0;
        for (int i = 0; i < aModel.EdgesNb(); i++)
        {
            IMeshData_Edge aDEdge = aModel.GetEdge(i);
            if (new BRepAdaptor_Curve(aDEdge.GetEdge()).GetType() != GeomAbs_CurveType.GeomAbs_Circle)
            {
                continue;
            }

            aCircles++;
            IMeshData_Curve aCurve = aDEdge.GetCurve();
            Assert.True(aCurve.ParametersNb() >= 4);
            gp_Circ aCirc = new BRepAdaptor_Curve(aDEdge.GetEdge()).Circle();
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
            TopoDS_Shape aShape = ReadDataOcc(aName);
            var aSnapshot = ShapeSnapshot.Of(aShape);

            // A coarse deflection: every polygon with parameters fits (BRepMesh_Deflection.IsConsistent).
            IMeshData_Model aModel = Model(aShape, Params(1000.0), Stage.EdgesDiscretized);
            int aRead = 0;
            for (int i = 0; i < aModel.EdgesNb(); i++)
            {
                IMeshData_Edge aDEdge = aModel.GetEdge(i);
                if (aDEdge.IsFree() || aDEdge.IsSet(IMeshData_Failure))
                {
                    continue;
                }

                // The polygon read back: the one of least deflection among the edge's faces.
                Poly_PolygonOnTriangulation? aBest = null;
                TopLoc_Location aBestLoc = new();
                Poly_Triangulation? aBestTri = null;
                for (int p = 0; p < aDEdge.PCurvesNb(); p++)
                {
                    Poly_Triangulation? aTri = BRep_Tool.Triangulation(aDEdge.GetPCurve(p).GetFace().GetFace(), out TopLoc_Location aLoc);
                    Poly_PolygonOnTriangulation? aPoly = aTri is null ? null : BRep_Tool.PolygonOnTriangulation(aDEdge.GetEdge(), aTri, aLoc);
                    if (aPoly is not null && aPoly.HasParameters() && (aBest is null || aPoly.Deflection() < aBest.Deflection()))
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
                IMeshData_Curve aCurve = aDEdge.GetCurve();
                Assert.Equal(aBest.NbNodes(), aCurve.ParametersNb());
                for (int k = 2; k < aBest.NbNodes(); k++) // the ends are the vertices' points
                {
                    gp_Pnt aNode = aBestTri!.Node(aBest.Node(k)).Transformed(aBestLoc.Transformation());
                    Assert.Equal(aNode, aCurve.GetPoint(k - 1));
                }
                if (aDEdge.GetSameParam())
                {
                    for (int k = 1; k <= aBest.NbNodes(); k++)
                    {
                        Assert.Equal(aBest.Parameters()![k], aCurve.GetParameter(k - 1));
                    }
                }
            }
            Assert.True(aRead > 0, aName);

            // The faces' triangulations fit too: reused, and nothing is written into the (frozen) shape.
            IMeshData_Model aPre = Model(aShape, Params(1000.0));
            for (int i = 0; i < aPre.FacesNb(); i++)
            {
                IMeshData_Face aDFace = aPre.GetFace(i);
                if (BRep_Tool.Triangulation(aDFace.GetFace(), out _) is not null && !aDFace.IsSet(IMeshData_Outdated))
                {
                    Assert.True(aDFace.IsSet(IMeshData_Reused), $"{aName} face {i}");
                }
            }
            aSnapshot.AssertUnchanged(aShape);
        }
    }

    [Fact]
    public void Outdated_meshes_are_removed_from_a_copy_never_from_the_frozen_input()
    {
        TopoDS_Shape aShape = ReadDataOcc("solid.brep");
        Assert.True(aShape.TShape()!.IsFrozen());
        var aSnapshot = ShapeSnapshot.Of(aShape);

        // A finer deflection: every triangulation and polygon is outdated.
        IMeshTools_Context aContext = Context(aShape, Params(0.01));
        Assert.True(aContext.BuildModel() && aContext.DiscretizeEdges() && aContext.HealModel());
        // The reader's shapes are frozen and locked: the first removal throws (TopoDS_LockedShape), never writes.
        Exception anError = Assert.ThrowsAny<Standard_Failure>(() => aContext.PreProcessModel());
        Assert.True(anError is Standard_ImmutableObject or TopoDS_LockedShape, anError.GetType().Name);
        aSnapshot.AssertUnchanged(aShape);

        TopoDS_Shape aCopy = BRepTools.ExactCopyForWrite(aShape, out _);
        var aFaceMeshes = new HashSet<Poly_Triangulation>(ReferenceEqualityComparer.Instance);
        foreach (TopoDS_Shape aF in Explore(aCopy, TopAbs_FACE))
        {
            aFaceMeshes.Add(BRep_Tool.Triangulation(TopoDS.Face(aF), out _)!);
        }
        IMeshData_Model aModel = Model(aCopy, Params(0.01));
        int anOutdated = 0;
        for (int i = 0; i < aModel.FacesNb(); i++)
        {
            IMeshData_Face aDFace = aModel.GetFace(i);
            Assert.False(aDFace.IsSet(IMeshData_Reused));
            if (aDFace.IsSet(IMeshData_Outdated))
            {
                anOutdated++;
                Assert.Null(BRep_Tool.Triangulation(aDFace.GetFace(), out _));
            }
        }
        Assert.Equal(aModel.FacesNb(), anOutdated);
        // The edges' polygons on those triangulations are gone; the file's stale polygons, on triangulations no face
        // holds, stay, as in OCCT.
        int aStale = 0;
        foreach (TopoDS_Shape anE in Explore(aCopy, TopAbs_EDGE))
        {
            for (int p = 1; ; p++)
            {
                BRep_Tool.PolygonOnTriangulation(TopoDS.Edge(anE), out Poly_PolygonOnTriangulation? aPoly,
                                                 out Poly_Triangulation? aTri, out _, p);
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
        IMeshData_Model aModel = Model(Box(), Params(0.1), Stage.EdgesDiscretized);
        IMeshData_Face aDFace = aModel.GetFace(0);
        IMeshData_Wire aDWire = aDFace.GetWire(0);
        IMeshData_PCurve aPCurve = aDWire.GetEdge(0).GetPCurve(aDFace, aDWire.GetEdgeOrientation(0));
        gp_Pnt2d anOriginal = aPCurve.GetPoint(0);
        aPCurve.GetPoint(0) = new gp_Pnt2d(anOriginal.X() + 1e-3, anOriginal.Y() - 1e-3);

        Assert.True(new BRepMesh_ModelHealer().Perform(aModel, Params(0.1), new Message_ProgressRange()));
        Assert.True(aPCurve.GetPoint(0).Distance(anOriginal) < 1e-12);
        Assert.True(aDFace.IsEqual(IMeshData_NoError));
        Assert.Equal(0.1, aDWire.GetDeflection());
        Assert.Equal(0.1, aDFace.GetDeflection());
    }

    /// <summary>A disk of radius 10 with a hole of radius 1 whose far point is 0.01 from the rim, at the given angle.</summary>
    private static TopoDS_Face DiskWithHole(double theAngle)
    {
        var anOuter = new BRepBuilderAPI_MakeWire(new BRepBuilderAPI_MakeEdge(new gp_Circ(new gp_Ax2(), 10)).Edge()).Wire();
        var aMaker = new BRepBuilderAPI_MakeFace(anOuter, true);
        if (!double.IsNaN(theAngle))
        {
            var aDir = new gp_Dir(Math.Cos(theAngle), Math.Sin(theAngle), 0);
            var aCenter = new gp_Pnt(8.99 * aDir.X(), 8.99 * aDir.Y(), 0);
            var aHole = new BRepBuilderAPI_MakeWire(new BRepBuilderAPI_MakeEdge(new gp_Circ(new gp_Ax2(aCenter, new gp_Dir(0, 0, 1), aDir), 1)).Edge()).Wire();
            aMaker.Add(TopoDS.Wire(aHole.Reversed()));
        }
        return aMaker.Face();
    }

    [Fact]
    public void Healer_amplifies_edges_whose_polygons_intersect()
    {
        // The rim's polygon, alone: its first chord's middle is where the hole will touch.
        IMeshData_Model aDisk = Model(DiskWithHole(double.NaN), Params(1.0), Stage.EdgesDiscretized);
        double anAngle = 0.5 * aDisk.GetEdge(0).GetCurve().GetParameter(1);

        TopoDS_Face aFace = DiskWithHole(anAngle);
        IMeshData_Model aModel = Model(aFace, Params(1.0), Stage.EdgesDiscretized);
        IMeshData_Face aDFace = aModel.GetFace(0);
        Assert.Equal(2, aDFace.WiresNb());
        Assert.True(aDFace.IsEqual(IMeshData_NoError));
        var aBefore = new double[aModel.EdgesNb()];
        for (int i = 0; i < aModel.EdgesNb(); i++)
        {
            aBefore[i] = aModel.GetEdge(i).GetDeflection();
        }

        // The polygons as discretized cross each other.
        BRepMesh_Deflection.ComputeDeflection(aDFace.GetWire(0), Params(1.0));
        BRepMesh_Deflection.ComputeDeflection(aDFace.GetWire(1), Params(1.0));
        BRepMesh_Deflection.ComputeDeflection(aDFace, Params(1.0));
        var aChecker = new BRepMesh_FaceChecker(aDFace, Params(1.0));
        Assert.False(aChecker.Perform());
        Assert.Equal(2, aChecker.GetIntersectingEdges()!.Size());

        // The healer refines both edges (deflection / 3 each round) until they don't.
        Assert.True(new BRepMesh_ModelHealer().Perform(aModel, Params(1.0), new Message_ProgressRange()));
        Assert.False(aDFace.IsSet(IMeshData_SelfIntersectingWire));
        Assert.False(aDFace.IsSet(IMeshData_Failure));
        for (int i = 0; i < aModel.EdgesNb(); i++)
        {
            Assert.True(aModel.GetEdge(i).GetDeflection() < aBefore[i] / 3.0 * 1.0000001);
        }
        Assert.True(new BRepMesh_FaceChecker(aDFace, Params(1.0)).Perform());
    }

    /// <summary>A planar face bounded by a bow tie: (0,0), (1,1), (1,0), (0,1), its edges forward or reversed.</summary>
    private static TopoDS_Face BowTie(bool isReversed)
    {
        TopoDS_Wire aBowTie = new BRepBuilderAPI_MakePolygon(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 1, 0), new gp_Pnt(1, 0, 0),
                                                             new gp_Pnt(0, 1, 0), true).Wire();
        var aB = new BRep_Builder();
        TopoDS_Face aFace = new();
        aB.MakeFace(ref aFace, new Geom_Plane(new gp_Pln()), Precision.Confusion());
        aB.Add(aFace, isReversed ? aBowTie.Reversed() : aBowTie);
        return aFace;
    }

    [Fact]
    public void Healer_fails_a_face_whose_wire_really_intersects_itself()
    {
        IMeshData_Model aModel = Model(BowTie(false), Params(0.01), Stage.Healed);
        IMeshData_Face aDFace = aModel.GetFace(0);
        Assert.Equal(TopAbs_FORWARD, aDFace.GetWire(0).GetEdgeOrientation(0));
        Assert.True(aDFace.IsSet(IMeshData_SelfIntersectingWire));
        Assert.True(aDFace.IsSet(IMeshData_Failure));
        Assert.False(aDFace.IsValid());
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
        IMeshData_Model aModel = Model(BowTie(true), Params(0.01), Stage.Healed);
        IMeshData_Face aDFace = aModel.GetFace(0);
        Assert.Equal(TopAbs_REVERSED, aDFace.GetWire(0).GetEdgeOrientation(0));
        Assert.True(aDFace.IsEqual(IMeshData_NoError));
        Assert.True(new BRepMesh_FaceChecker(aDFace, Params(0.01)).Perform());
    }

    // ------------------------------------------------------------------------------------------------ pre-processor

    [Fact]
    public void Cone_seams_get_the_steps_of_the_cone_splitter()
    {
        IMeshData_Model aEdges = Model(new BRepPrimAPI_MakeCone(2, 1, 4).Solid(), Params(0.01), Stage.Healed);
        IMeshData_Model aModel = Model(new BRepPrimAPI_MakeCone(2, 1, 4).Solid(), Params(0.01));
        int aSeams = 0;
        for (int i = 0; i < aModel.EdgesNb(); i++)
        {
            IMeshData_Edge aDEdge = aModel.GetEdge(i);
            if (aDEdge.PCurvesNb() != 2 || !ReferenceEquals(aDEdge.GetPCurve(0).GetFace(), aDEdge.GetPCurve(1).GetFace()))
            {
                continue;
            }

            aSeams++;
            Assert.Equal(2, aEdges.GetEdge(i).GetCurve().ParametersNb());
            IMeshData_Curve aCurve = aDEdge.GetCurve();
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
            Assert.False(aDEdge.GetPCurve(0).GetFace().IsSet(IMeshData_Outdated));
        }
        Assert.Equal(1, aSeams);
    }

    // ----------------------------------------------------------------------------------------------- post-processor

    [Fact]
    public void Post_processor_commits_polygons_and_reuse_gives_back_the_mesh()
    {
        var aB = new BRep_Builder();
        TopoDS_Compound aComp = new();
        aB.MakeCompound(ref aComp);
        aB.Add(aComp, new BRepPrimAPI_MakeCylinder(1, 2).Solid());
        aB.Add(aComp, new BRepBuilderAPI_MakeEdge(new gp_Circ(new gp_Ax2(new gp_Pnt(5, 0, 0), new gp_Dir(0, 0, 1)), 1), 0, 2).Edge());
        TopoDS_Shape anInput = TestShapes.Frozen((TopoDS_Shape)aComp);
        var aSnapshot = ShapeSnapshot.Of(anInput);

        IMeshTools_Parameters aParams = Params(0.05);
        var aMesh = new BRepMesh_IncrementalMesh();
        aMesh.SetShape(anInput);
        aMesh.ChangeParameters() = aParams;
        var aFaces = new FanFaceDiscret();
        IMeshTools_Context aContext = Context(new TopoDS_Shape(), aParams, aFaces);
        aMesh.Perform(aContext);
        Assert.True(aMesh.IsDone());
        Assert.Equal(3, aFaces.Meshed);
        aSnapshot.AssertUnchanged(anInput);

        TopoDS_Shape aResult = aMesh.Shape();
        Assert.True(aResult.TShape()!.IsFrozen());

        // Each face's triangulation, with the deflection measured and the parameters it was made with.
        IMeshData_Model aModel = aContext.GetModel()!; // of the copy, whose meshed sub-shapes the result holds
        foreach (TopoDS_Shape aF in Explore(aResult, TopAbs_FACE))
        {
            Poly_Triangulation aTri = BRep_Tool.Triangulation(TopoDS.Face(aF), out _)!;
            Assert.True(aTri.IsFrozen());
            Assert.Equal(0.05, aTri.Parameters()!.Deflection());
            Assert.Equal(0.5, aTri.Parameters()!.Angle());
            Assert.Equal(aParams.MinSize, aTri.Parameters()!.MinSize());
        }

        // Each edge's polygons: its discrete pcurves' parameters, nodes that are its points, its deflection.
        int aPolygons = 0;
        var anEdges = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(aResult, TopAbs_EDGE, anEdges);
        for (int anEdgeIt = 1; anEdgeIt <= anEdges.Extent(); anEdgeIt++)
        {
            TopoDS_Edge anEdge = TopoDS.Edge(anEdges.FindKey(anEdgeIt));
            IMeshData_Edge? aDEdge = null;
            for (int i = 0; i < aModel.EdgesNb() && aDEdge is null; i++)
            {
                if (aModel.GetEdge(i).GetEdge().IsSame(anEdge))
                {
                    aDEdge = aModel.GetEdge(i);
                }
            }
            Assert.NotNull(aDEdge);
            IMeshData_Curve aCurve = aDEdge!.GetCurve();
            for (int p = 1; ; p++)
            {
                BRep_Tool.PolygonOnTriangulation(anEdge, out Poly_PolygonOnTriangulation? aPoly, out Poly_Triangulation? aTri,
                                                 out TopLoc_Location aLoc, p);
                if (aPoly is null)
                {
                    break;
                }
                aPolygons++;
                Assert.True(aPoly.IsFrozen());
                Assert.Equal(aDEdge.GetDeflection(), aPoly.Deflection());
                Assert.Equal(aCurve.ParametersNb(), aPoly.NbNodes());
                for (int k = 1; k <= aPoly.NbNodes(); k++)
                {
                    Assert.Equal(aCurve.GetParameter(k - 1), aPoly.Parameters()![k]);
                    Assert.True(aTri!.Node(aPoly.Node(k)).Transformed(aLoc.Transformation()).Distance(aCurve.GetPoint(k - 1)) < 1e-12);
                }
            }
            if (BRep_Tool.Degenerated(anEdge))
            {
                continue;
            }
            if (aDEdge.IsFree())
            {
                Poly_Polygon3D aPoly3D = BRep_Tool.Polygon3D(anEdge, out _)!;
                Assert.True(aPoly3D.IsFrozen());
                Assert.Equal(aCurve.ParametersNb(), aPoly3D.NbNodes());
                Assert.Equal(aDEdge.GetDeflection(), aPoly3D.Deflection());
            }
        }
        Assert.Equal(5, aPolygons); // two circles on two faces each, the seam's pair once

        // Meshing the result again with the same parameters reuses everything: the result itself comes back.
        var aRemesh = new BRepMesh_IncrementalMesh();
        aRemesh.SetShape(aResult);
        aRemesh.ChangeParameters() = aParams;
        var aNoFaces = new FanFaceDiscret();
        aRemesh.Perform(Context(new TopoDS_Shape(), aParams, aNoFaces));
        Assert.Equal(0, aNoFaces.Meshed);
        Assert.Same(aResult.TShape(), aRemesh.Shape().TShape());
        Assert.Equal((int)IMeshData_Reused, aRemesh.GetStatusFlags());

        // A finer deflection replaces them on a copy.
        var aFiner = new BRepMesh_IncrementalMesh();
        aFiner.SetShape(aResult);
        aFiner.ChangeParameters() = Params(0.01);
        var aFinerFaces = new FanFaceDiscret();
        aFiner.Perform(Context(new TopoDS_Shape(), Params(0.01), aFinerFaces));
        Assert.Equal(3, aFinerFaces.Meshed);
        Assert.NotSame(aResult.TShape(), aFiner.Shape().TShape());
        foreach (TopoDS_Shape aF in Explore(aFiner.Shape(), TopAbs_FACE))
        {
            Assert.Equal(0.01, BRep_Tool.Triangulation(TopoDS.Face(aF), out _)!.Parameters()!.Deflection());
        }
    }

    [Fact]
    public void Seam_polygons_are_stored_in_the_model_order_as_OCCT_does()
    {
        // OCCT: ModelPostProcessor stores a seam's first discrete pcurve on the face as the polygon of the forward edge
        // and the last as the reversed edge's, whatever their orientations. On BRepPrimAPI's cylinder the wire is
        // ordered top, seam (reversed, u = 0), bottom, seam (forward, u = 2pi): the forward edge's polygon gets the
        // nodes at u = 0 (occt-issues.md, 4C).
        TopoDS_Shape anInput = TestShapes.Frozen((TopoDS_Shape)new BRepPrimAPI_MakeCylinder(1, 2).Solid());
        IMeshTools_Parameters aParams = Params(0.05);
        var aMesh = new BRepMesh_IncrementalMesh();
        aMesh.SetShape(anInput);
        aMesh.ChangeParameters() = aParams;
        IMeshTools_Context aContext = Context(new TopoDS_Shape(), aParams, new FanFaceDiscret());
        aMesh.Perform(aContext);
        IMeshData_Model aModel = aContext.GetModel()!;

        int aSeams = 0;
        for (int i = 0; i < aModel.EdgesNb(); i++)
        {
            IMeshData_Edge aDEdge = aModel.GetEdge(i);
            if (aDEdge.PCurvesNb() != 2 || !ReferenceEquals(aDEdge.GetPCurve(0).GetFace(), aDEdge.GetPCurve(1).GetFace()))
            {
                continue;
            }

            aSeams++;
            IMeshData_PCurve aFirst = aDEdge.GetPCurve(0), aLast = aDEdge.GetPCurve(1);
            Assert.Equal(TopAbs_REVERSED, aFirst.GetOrientation());
            Assert.Equal(0.0, aFirst.GetPoint(0).X(), 1e-12);
            Assert.Equal(2 * Math.PI, aLast.GetPoint(0).X(), 1e-12);

            TopoDS_Face aFace = aFirst.GetFace().GetFace();
            Poly_Triangulation aTri = BRep_Tool.Triangulation(aFace, out TopLoc_Location aLoc)!;
            Poly_PolygonOnTriangulation aForward =
                BRep_Tool.PolygonOnTriangulation(TopoDS.Edge(aDEdge.GetEdge().Oriented(TopAbs_FORWARD)), aTri, aLoc)!;
            Poly_PolygonOnTriangulation aReversed =
                BRep_Tool.PolygonOnTriangulation(TopoDS.Edge(aDEdge.GetEdge().Oriented(TopAbs_REVERSED)), aTri, aLoc)!;
            for (int k = 1; k <= aForward.NbNodes(); k++)
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
        var aShapes = new List<(string, TopoDS_Shape)>
        {
            ("box", Box()),
            ("cylinder", new BRepPrimAPI_MakeCylinder(2, 5).Solid()),
            ("cone", new BRepPrimAPI_MakeCone(2, 1, 4).Solid()),
            ("sphere", new BRepPrimAPI_MakeSphere(3).Solid()),
            ("torus", new BRepPrimAPI_MakeTorus(5, 1).Solid()),
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

        foreach ((string aName, TopoDS_Shape aShape) in aShapes)
        {
            foreach (IMeshTools_Parameters aBase in new[] { Params(0.05, 0.5, isRelative: true), Params(0.5, 0.3) })
            {
                IMeshTools_Parameters aSerial = aBase, aParallel = aBase;
                aSerial.InParallel = false;
                aParallel.InParallel = true;
                string aReference = Dump(Model(BRepTools.ExactCopyForWrite(aShape, out _), aSerial));
                for (int aRun = 0; aRun < 2; aRun++)
                {
                    string aDump = Dump(Model(BRepTools.ExactCopyForWrite(aShape, out _), aParallel));
                    Assert.True(aReference == aDump, $"{aName}: parallel run {aRun} differs");
                }
                using (OSD_Parallel.ForceSerialScope())
                {
                    Assert.True(aReference == Dump(Model(BRepTools.ExactCopyForWrite(aShape, out _), aParallel)), $"{aName}: forced serial");
                }
            }
        }
    }
}
