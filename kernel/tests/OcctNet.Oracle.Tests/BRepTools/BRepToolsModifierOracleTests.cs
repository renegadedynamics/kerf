// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Meshing;
using Kerf.Geometry;
using Kerf.Topology;
using Schematiq.Collections;
using Xunit.Abstractions;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// BRepTools_Modifier with the copy, transformation and general transformation modifications against native
/// BRepBuilderAPI_Copy, BRepBuilderAPI_Transform and BRepBuilderAPI_GTransform, which run the same modifier with the
/// same modification, on shapes built identically on both sides (<see cref="TwinShapes"/>), the port's frozen. The
/// results are compared as the topology oracle compares shapes (flags, exploration, identity, every vertex, edge and
/// face), with their triangulations, and each input sub-shape's ModifiedShape by its place in the result. Also
/// BRepTools_History's Merge against native OCCT on random histories, and CopyForWrite against BRepBuilderAPI_Copy.
/// </summary>
public class BRepToolsModifierOracleTests(ITestOutputHelper output)
{
    private const int Scenes = 48;

    private static readonly TopAbs_ShapeEnum[] theAllTypes =
        [TopAbs_COMPOUND, TopAbs_COMPSOLID, TopAbs_SOLID, TopAbs_SHELL, TopAbs_FACE, TopAbs_WIRE, TopAbs_EDGE, TopAbs_VERTEX];

    [Fact]
    public void Copy_answers_as_BRepBuilderAPI_Copy()
    {
        var anOracle = new Oracle(output);
        var aCounts = new TopologyOracleTests.Counts();
        foreach (int aScene in Tape.Budget(Scenes))
        {
            Twin<N.TopoDS_Shape, TopoDS_Shape> aShape = MeshedScene(anOracle, aScene);
            bool aCopyGeom = anOracle.Bool(), aCopyMesh = anOracle.Bool();
            using var aNCopy = new N.BRepBuilderAPI_Copy(aShape.N, aCopyGeom, aCopyMesh);
            var aModifier = new BRepTools_Modifier(aShape.P, new BRepTools_CopyModification(aCopyGeom, aCopyMesh));
            CompareResult(anOracle, aShape, aNCopy.Shape(), aModifier, aNCopy.ModifiedShape, aCounts);
        }

        output.WriteLine($"{Scenes} scenes: {aCounts.Shapes} explored sub-shapes, {aCounts.Edges} edges, {aCounts.Faces} faces");
        anOracle.Report("BRepTools_Modifier with BRepTools_CopyModification against BRepBuilderAPI_Copy");
    }

    [Fact]
    public void Transformation_answers_as_BRepBuilderAPI_Transform()
    {
        var anOracle = new Oracle(output);
        var aCounts = new TopologyOracleTests.Counts();
        foreach (int aScene in Tape.Budget(Scenes))
        {
            Twin<N.TopoDS_Shape, TopoDS_Shape> aShape = MeshedScene(anOracle, aScene);
            (N.gp_Trsf aNT, Transform3D aT) = RandomTrsf(anOracle, aScene % 4);
            bool aCopyMesh = anOracle.Bool();
            // the modifier's path: a copy of the geometry asked, or a transformation that isn't a rigid motion
            using var aNTransform = new N.BRepBuilderAPI_Transform(aShape.N, aNT, true, aCopyMesh);
            var aModification = new BRepTools_TrsfModification(aT);
            aModification.IsCopyMesh() = aCopyMesh;
            var aModifier = new BRepTools_Modifier(aShape.P, aModification);
            // TrsfModification gives planar faces no pcurves: BRep_Tool computes them on the fly (CurveOnPlane)
            CompareResult(anOracle, aShape, aNTransform.Shape(), aModifier, aNTransform.ModifiedShape, aCounts);
            aNT.Dispose();
        }

        output.WriteLine($"{Scenes} scenes: {aCounts.Shapes} explored sub-shapes, {aCounts.Edges} edges, {aCounts.Faces} faces");
        anOracle.Report("BRepTools_Modifier with BRepTools_TrsfModification against BRepBuilderAPI_Transform");
    }

    [Fact]
    public void General_transformation_answers_as_BRepBuilderAPI_GTransform()
    {
        var anOracle = new Oracle(output);
        var aCounts = new TopologyOracleTests.Counts();
        foreach (int aScene in Tape.Budget(Scenes))
        {
            Twin<N.TopoDS_Shape, TopoDS_Shape> aShape = BSplineScene(anOracle);
            (N.gp_GTrsf aNG, GeneralTransform3D aG) = RandomGTrsf(anOracle);
            using (aNG)
            {
                // GTransform converts to NURBS first, which leaves a shape of B-splines with B-spline pcurves as it is
                using var aNGTransform = new N.BRepBuilderAPI_GTransform(aShape.N, aNG, false);
                var aModifier = new BRepTools_Modifier(aShape.P, new BRepTools_GTrsfModification(aG));
                CompareResult(anOracle, aShape, aNGTransform.Shape(), aModifier, aNGTransform.ModifiedShape, aCounts);
            }
        }

        output.WriteLine($"{Scenes} scenes: {aCounts.Shapes} explored sub-shapes, {aCounts.Edges} edges, {aCounts.Faces} faces");
        anOracle.Report("BRepTools_Modifier with BRepTools_GTrsfModification against BRepBuilderAPI_GTransform");
    }

    /// <summary>
    /// BRepTools_NurbsConvertModification on shapes with faces. BRepBuilderAPI_NurbsConvert isn't bound, but
    /// BRepBuilderAPI_GTransform runs it first (the modifier with the NURBS conversion, then CorrectVertexTol) and
    /// then the modifier with the general transformation; the port's side runs those two steps (its
    /// BRepBuilderAPI_NurbsConvert, then the modifier). The scenes are the
    /// global properties oracle's (boxes, whose pcurves on planes are projected with ProjLib_ComputeApprox; closed
    /// cylinders, whose circles and seam are projected with ProjLib_ComputeApproxOnPolarSurface; holed faces;
    /// compounds sharing solids at rigid, reversed and scaled locations; natural spheres, tori and sphere patches;
    /// B-spline faces; meshed boxes), with an identity or a random affine transformation.
    /// </summary>
    [Fact]
    public void NURBS_conversion_answers_as_BRepBuilderAPI_GTransform()
    {
        var anOracle = new Oracle(output, 20261030);
        var aCounts = new TopologyOracleTests.Counts();
        int aReplaced = 0;
        foreach (int aScene in Tape.Budget(Scenes))
        {
            // the mesh-only tetrahedron (scene 7) has no geometry to convert
            (Twin<N.TopoDS_Shape, TopoDS_Shape> aShape, _) = BRepGPropOracleTests.RandomScene(anOracle, aScene % 7);
            aShape.P.TShape()!.FreezeGraph();
            (N.gp_GTrsf aNG, GeneralTransform3D aG) = anOracle.Int(0, 2) == 0
                ? (new N.gp_GTrsf(new N.gp_Mat(1, 0, 0, 0, 1, 0, 0, 0, 1), new N.gp_XYZ(0, 0, 0)),
                   new GeneralTransform3D(new Matrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1), new Xyz(0, 0, 0)))
                : RandomGTrsf(anOracle);
            using (aNG)
            {
                using var aNGTransform = new N.BRepBuilderAPI_GTransform(aShape.N, aNG, false);
                TopoDS_Shape aConverted = NurbsConvert(aShape.P, ref aReplaced);
                var aModifier = new BRepTools_Modifier(aConverted, new BRepTools_GTrsfModification(aG));
                Assert.True(aModifier.IsDone());
                Twin<N.TopoDS_Shape, TopoDS_Shape> aResult = new(aNGTransform.Shape(), aModifier.InternalModifiedShape(aConverted));
                TopologyOracleTests.CompareFlags(anOracle, aResult);
                aResult.P.TShape()!.FreezeGraph();
                TopologyOracleTests.CompareTopology(anOracle, aResult, aCounts);
                TopologyOracleTests.CompareGeometry(anOracle, aResult, aCounts);
                CompareMeshes(anOracle, aResult);
            }
        }

        output.WriteLine($"{Scenes} scenes: {aCounts.Shapes} explored sub-shapes, {aCounts.Edges} edges, {aCounts.Faces} faces, "
                         + $"{aReplaced} vertices replaced by CorrectVertexTol");
        anOracle.Report("BRepTools_Modifier with BRepTools_NurbsConvertModification (through BRepBuilderAPI_GTransform)");
    }

    // BRepBuilderAPI_NurbsConvert: the modifier with the NURBS conversion, then CorrectVertexTol, which raises the
    // tolerance of the vertices of the edges whose tolerance the conversion raised (replacing the input's own vertices
    // by new ones). The result is read unfrozen; the count is of the input's vertices replaced.
    private static TopoDS_Shape NurbsConvert(in TopoDS_Shape S, ref int theReplaced)
    {
        var aNC = new BRepBuilderAPI_NurbsConvert(S);
        Assert.True(aNC.IsDone());
        var aPlain = new BRepTools_Modifier(S, new BRepTools_NurbsConvertModification());
        for (var anExp = new TopExp_Explorer(S, TopAbs_VERTEX); anExp.More(); anExp.Next())
        {
            // a vertex the conversion keeps, which CorrectVertexTol replaced
            if (aPlain.InternalModifiedShape(anExp.Current()).IsSame(anExp.Current())
                && !aNC.InternalModifiedShape(anExp.Current()).IsSame(anExp.Current()))
            {
                theReplaced++;
            }
        }

        return aNC.InternalShape();
    }

    [Fact]
    public void CopyForWrite_answers_as_BRepBuilderAPI_Copy_without_geometry()
    {
        var anOracle = new Oracle(output);
        var aCounts = new TopologyOracleTests.Counts();
        foreach (int aScene in Tape.Budget(Scenes))
        {
            Twin<N.TopoDS_Shape, TopoDS_Shape> aShape = MeshedScene(anOracle, aScene);
            using var aNCopy = new N.BRepBuilderAPI_Copy(aShape.N, false, true);
            TopoDS_Shape aCopy = BRepTools.CopyForWrite(aShape.P, out BRepTools_History aHistory);
            Assert.False(aCopy.TShape()!.IsFrozen);
            Twin<N.TopoDS_Shape, TopoDS_Shape> aResult = new(aNCopy.Shape(), aCopy);
            TopologyOracleTests.CompareFlags(anOracle, aResult);
            aCopy.TShape()!.FreezeGraph();
            TopologyOracleTests.CompareTopology(anOracle, aResult, aCounts);
            TopologyOracleTests.CompareGeometry(anOracle, aResult, aCounts);
            CompareMeshes(anOracle, aResult);

            // the history: each vertex, edge, face and solid modified into its copy, as BRepBuilderAPI_Copy's Modified
            foreach (Twin<N.TopoDS_Shape, TopoDS_Shape> aS in AllSubShapes(aShape))
            {
                if (!BRepTools_History.IsSupportedType(aS.P))
                {
                    continue;
                }

                N.TopoDS_Shape[] aNModified = aNCopy.Modified(aS.N).ToArray();
                ISchematiqList<TopoDS_Shape> aModified = aHistory.Modified(aS.P);
                anOracle.Same(aNModified.Length, aModified.Count, "history Modified count");
                anOracle.Same(false, aHistory.IsRemoved(aS.P), "history IsRemoved");
                CompareImage(anOracle, aResult, aNModified[0], aModified[0], "history Modified");
            }
        }

        anOracle.Report("BRepTools.CopyForWrite against BRepBuilderAPI_Copy(copyGeom = false, copyMesh = true)");
    }

    [Fact]
    public void History_merges_as_native_OCCT()
    {
        var anOracle = new Oracle(output);
        int aQueries = 0;
        foreach (int aRound in Tape.Budget(200))
        {
            Twin<N.TopoDS_Shape, TopoDS_Shape> aShape = TopologyOracleTests.RandomScene(anOracle, aRound);
            List<Twin<N.TopoDS_Shape, TopoDS_Shape>> aShapes = AllSubShapes(aShape)
                .Where(s => BRepTools_History.IsSupportedType(s.P)).ToList();
            if (aShapes.Count < 3)
            {
                continue;
            }

            var aNH12 = new N.BRepTools_History();
            var aH12 = new BRepTools_History();
            var aNH23 = new N.BRepTools_History();
            var aH23 = new BRepTools_History();
            RandomHistory(anOracle, aShapes, aNH12, aH12);
            RandomHistory(anOracle, aShapes, aNH23, aH23);
            aNH12.Merge(aNH23);
            aH12.Merge(aH23);

            anOracle.Same(aNH12.HasModified(), aH12.HasModified(), "HasModified");
            anOracle.Same(aNH12.HasGenerated(), aH12.HasGenerated(), "HasGenerated");
            anOracle.Same(aNH12.HasRemoved(), aH12.HasRemoved(), "HasRemoved");
            foreach (Twin<N.TopoDS_Shape, TopoDS_Shape> aS in aShapes)
            {
                aQueries++;
                anOracle.Same(aNH12.IsRemoved(aS.N), aH12.IsRemoved(aS.P), "IsRemoved");
                CompareSet(anOracle, aShapes, aNH12.Modified(aS.N).ToArray(), aH12.Modified(aS.P), "Modified");
                CompareSet(anOracle, aShapes, aNH12.Generated(aS.N).ToArray(), aH12.Generated(aS.P), "Generated");
            }
        }

        output.WriteLine($"{aQueries} shapes queried");
        anOracle.Report("BRepTools_History.Merge");
    }

    // Histories

    private static void RandomHistory(Oracle theOracle, List<Twin<N.TopoDS_Shape, TopoDS_Shape>> theShapes,
                                      N.BRepTools_History theNH, BRepTools_History theH)
    {
        int aNbOps = theOracle.Int(1, 2 * theShapes.Count);
        for (int k = 0; k < aNbOps; k++)
        {
            Twin<N.TopoDS_Shape, TopoDS_Shape> a = theShapes[theOracle.Int(0, theShapes.Count - 1)];
            Twin<N.TopoDS_Shape, TopoDS_Shape> b = theShapes[theOracle.Int(0, theShapes.Count - 1)];
            switch (theOracle.Int(0, 4))
            {
                case 0:
                    theNH.AddModified(a.N, b.N);
                    theH.AddModified(a.P, b.P);
                    break;
                case 1:
                    theNH.AddGenerated(a.N, b.N);
                    theH.AddGenerated(a.P, b.P);
                    break;
                case 2:
                    theNH.Remove(a.N);
                    theH.Remove(a.P);
                    break;
                case 3:
                    theNH.ReplaceModified(a.N, b.N);
                    theH.ReplaceModified(a.P, b.P);
                    break;
                default:
                    theNH.ReplaceGenerated(a.N, b.N);
                    theH.ReplaceGenerated(a.P, b.P);
                    break;
            }
        }
    }

    // The lists as multisets of (index of the twin, orientation): the merge's order follows hash maps on both sides.
    private static void CompareSet(Oracle theOracle, List<Twin<N.TopoDS_Shape, TopoDS_Shape>> theShapes, N.TopoDS_Shape[] theN,
                                   ISchematiqList<TopoDS_Shape> theP, string theWhat)
    {
        theOracle.Same(theN.Length, theP.Count, theWhat + " count");
        var aNKeys = theN.Select(n => (theShapes.FindIndex(s => s.N.IsSame(n)), (int)n.Orientation())).OrderBy(k => k).ToList();
        var aKeys = theP.Select(p => (theShapes.FindIndex(s => s.P.IsSame(p)), (int)p.Orientation())).OrderBy(k => k).ToList();
        theOracle.Same(true, aNKeys.SequenceEqual(aKeys), theWhat + " elements");
    }

    // Scenes

    // A random scene of the topology oracle, with a one-triangle triangulation on some of its faces.
    internal static Twin<N.TopoDS_Shape, TopoDS_Shape> MeshedScene(Oracle theOracle, int theScene)
    {
        Twin<N.TopoDS_Shape, TopoDS_Shape> aShape = TopologyOracleTests.RandomScene(theOracle, theScene);
        if (theOracle.Bool())
        {
            var aTwins = new TwinShapes();
            var aDone = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            foreach (Twin<N.TopoDS_Shape, TopoDS_Shape> aF in TopologyOracleTests.Explore(aShape, TopAbs_FACE))
            {
                if (aDone.Add(aF.P) && theOracle.Bool())
                {
                    aTwins.AttachTriangulation(new(N.TopoDS.Face(aF.N), TopoDS.Face(aF.P)), theOracle.Real(0.01, 0.1));
                }
            }
        }

        aShape.P.TShape()!.FreezeGraph();
        return aShape;
    }

    // A B-spline face (iso-curve edges, degree-1 B-spline pcurves), alone, placed, or in a compound with a placed and
    // reversed copy of itself; the port's frozen.
    internal static Twin<N.TopoDS_Shape, TopoDS_Shape> BSplineScene(Oracle theOracle)
    {
        var aTwins = new TwinShapes();
        (N.Geom_BSplineSurface aNS, Geom_BSplineSurface aS) = AdaptorOracleInputs.RandomBSplineSurface(theOracle, 0, false);
        aS.Bounds(out double u1, out double u2, out double v1, out double v2);
        Twin<N.TopoDS_Vertex, TopoDS_Vertex> V(double u, double v) => aTwins.Vertex(aS.Value(u, v), 1e-6);
        Twin<N.TopoDS_Vertex, TopoDS_Vertex> v11 = V(u1, v1), v21 = V(u2, v1), v22 = V(u2, v2), v12 = V(u1, v2);
        Twin<N.TopoDS_Edge, TopoDS_Edge> e1 = aTwins.EdgeOn(aNS.VIso(v1)!, aS.VIso(v1), new Placement(), null, u1, u2, v11, v21);
        Twin<N.TopoDS_Edge, TopoDS_Edge> e2 = aTwins.EdgeOn(aNS.UIso(u2)!, aS.UIso(u2), new Placement(), null, v1, v2, v21, v22);
        Twin<N.TopoDS_Edge, TopoDS_Edge> e3 = aTwins.EdgeOn(aNS.VIso(v2)!, aS.VIso(v2), new Placement(), null, u1, u2, v12, v22);
        Twin<N.TopoDS_Edge, TopoDS_Edge> e4 = aTwins.EdgeOn(aNS.UIso(u1)!, aS.UIso(u1), new Placement(), null, v1, v2, v11, v12);
        Twin<N.TopoDS_Face, TopoDS_Face> aFace = aTwins.FaceOn(aNS, aS,
                                                               (e1, false, NLine(u1, v1, u2, v1, u1, u2), Line(u1, v1, u2, v1, u1, u2)),
                                                               (e2, false, NLine(u2, v1, u2, v2, v1, v2), Line(u2, v1, u2, v2, v1, v2)),
                                                               (e3, true, NLine(u1, v2, u2, v2, u1, u2), Line(u1, v2, u2, v2, u1, u2)),
                                                               (e4, true, NLine(u1, v1, u1, v2, v1, v2), Line(u1, v1, u1, v2, v1, v2)));
        Twin<N.TopoDS_Shape, TopoDS_Shape> aShape = TwinShapes.Up(aFace);
        switch (theOracle.Int(0, 2))
        {
            case 0:
                break;
            case 1:
            {
                (N.TopLoc_Location aNL, Placement aL) = TopologyOracleTests.RandomLocation(theOracle);
                aShape = new(aShape.N.Moved(aNL), aShape.P.Moved(aL));
                break;
            }
            default:
            {
                (N.TopLoc_Location aNL, Placement aL) = TopologyOracleTests.RandomLocation(theOracle);
                aShape = TwinShapes.Up(aTwins.Compound(aShape, new(aShape.N.Moved(aNL).Reversed(), aShape.P.Moved(aL).Reversed())));
                break;
            }
        }

        aShape.P.TShape()!.FreezeGraph();
        return aShape;
    }

    // A degree-1 B-spline from (x1, y1) to (x2, y2) over [t1, t2], on both sides.
    private static NCollection_Array1<Point2D> LinePoles(double x1, double y1, double x2, double y2)
    {
        var aPoles = new NCollection_Array1<Point2D>(1, 2);
        aPoles[1] = new Point2D(x1, y1);
        aPoles[2] = new Point2D(x2, y2);
        return aPoles;
    }

    private static (NCollection_Array1<double>, NCollection_Array1<int>) LineKnots(double t1, double t2)
    {
        var aKnots = new NCollection_Array1<double>(1, 2);
        aKnots[1] = t1;
        aKnots[2] = t2;
        var aMults = new NCollection_Array1<int>(1, 2);
        aMults[1] = aMults[2] = 2;
        return (aKnots, aMults);
    }

    private static N.Geom2d_Curve NLine(double x1, double y1, double x2, double y2, double t1, double t2)
    {
        (NCollection_Array1<double> aKnots, NCollection_Array1<int> aMults) = LineKnots(t1, t2);
        return Oracle.ToNativeCurve(LinePoles(x1, y1, x2, y2), null, aKnots, aMults, 1, false);
    }

    private static Geom2d_Curve Line(double x1, double y1, double x2, double y2, double t1, double t2)
    {
        (NCollection_Array1<double> aKnots, NCollection_Array1<int> aMults) = LineKnots(t1, t2);
        return new Geom2d_BSplineCurve(LinePoles(x1, y1, x2, y2), aKnots, aMults, 1);
    }

    // Transformations

    // A rigid motion composed with a scaling (0), a point mirror (1), a plane mirror (2) or nothing (3), from the same
    // raw numbers on both sides.
    internal static (N.gp_Trsf, Transform3D) RandomTrsf(Oracle theOracle, int theKind)
    {
        (N.TopLoc_Location aNL, Placement aL) = TopologyOracleTests.RandomLocation(theOracle);
        using N.gp_Trsf aNRigid = aNL.Transformation();
        Transform3D aRigid = aL.Transformation();
        var aNT = new N.gp_Trsf();
        var aT = new Transform3D();
        Point3D aP = theOracle.Pnt();
        using N.gp_Pnt aNP = Oracle.ToNative(aP);
        switch (theKind)
        {
            case 0:
            {
                double aScale = theOracle.Bool() ? theOracle.Real(0.2, 0.9) : theOracle.Real(1.1, 5.0);
                aNT.SetScale(aNP, aScale);
                aT.SetScale(aP, aScale);
                break;
            }
            case 1:
                aNT.SetMirror(aNP);
                aT.SetMirror(aP);
                break;
            case 2:
            {
                Vector3D aV = theOracle.Vec();
                using var aND = new N.gp_Dir(aV.X(), aV.Y(), aV.Z());
                using var aNAx = new N.gp_Ax2(aNP, aND);
                aNT.SetMirror(aNAx);
                aT.SetMirror(new Frame3D(aP, new Direction3D(aV.X(), aV.Y(), aV.Z())));
                break;
            }
            default:
                break;
        }

        return (aNRigid.Multiplied(aNT), aRigid.Multiplied(aT));
    }

    // An affine transformation near a rotation, scaled, from the same raw numbers on both sides.
    internal static (N.gp_GTrsf, GeneralTransform3D) RandomGTrsf(Oracle theOracle)
    {
        double[] m = new double[9];
        for (int i = 0; i < 9; i++)
        {
            m[i] = (i % 4 == 0 ? theOracle.Real(0.5, 2.0) * (theOracle.Int(0, 5) == 0 ? -1 : 1) : 0.0) + theOracle.Real(-0.3, 0.3);
        }

        Vector3D aV = theOracle.Vec();
        using var aNM = new N.gp_Mat(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
        using var aNV = new N.gp_XYZ(aV.X(), aV.Y(), aV.Z());
        return (new N.gp_GTrsf(aNM, aNV),
                new GeneralTransform3D(new Matrix3x3(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]), new Xyz(aV.X(), aV.Y(), aV.Z())));
    }

    // Comparisons

    internal static List<Twin<N.TopoDS_Shape, TopoDS_Shape>> AllSubShapes(Twin<N.TopoDS_Shape, TopoDS_Shape> theShape)
    {
        var aList = new List<Twin<N.TopoDS_Shape, TopoDS_Shape>> { theShape };
        var aDone = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance) { theShape.P };
        foreach (TopAbs_ShapeEnum aType in theAllTypes)
        {
            foreach (Twin<N.TopoDS_Shape, TopoDS_Shape> aS in TopologyOracleTests.Explore(theShape, aType))
            {
                if (aDone.Add(aS.P))
                {
                    aList.Add(aS);
                }
            }
        }

        return aList;
    }

    private static void CompareResult(Oracle theOracle, Twin<N.TopoDS_Shape, TopoDS_Shape> theInput, N.TopoDS_Shape theNResult,
                                      BRepTools_Modifier theModifier, Func<N.TopoDS_Shape, N.TopoDS_Shape> theNModifiedShape,
                                      TopologyOracleTests.Counts theCounts)
    {
        Assert.True(theModifier.IsDone());
        CompareResult(theOracle, theInput, theNResult, theModifier.InternalModifiedShape(theInput.P),
                      () => theModifier.ModifiedShape(theInput.P), theNModifiedShape, s => theModifier.ModifiedShape(s),
                      theCounts);
    }

    /// <summary>
    /// A modification's result against native OCCT's: its flags before it is published (<paramref name="theResult"/>,
    /// unfrozen), then the published result (the same shape, frozen), its topology, geometry and meshes, and each input
    /// sub-shape's ModifiedShape by its place in the result.
    /// </summary>
    internal static void CompareResult(Oracle theOracle, Twin<N.TopoDS_Shape, TopoDS_Shape> theInput, N.TopoDS_Shape theNResult,
                                       TopoDS_Shape theResult, Func<TopoDS_Shape> thePublish,
                                       Func<N.TopoDS_Shape, N.TopoDS_Shape> theNModifiedShape,
                                       Func<TopoDS_Shape, TopoDS_Shape> theModifiedShape,
                                       TopologyOracleTests.Counts theCounts, bool theCompareGeometry = true)
    {
        Twin<N.TopoDS_Shape, TopoDS_Shape> aTwin = new(theNResult, theResult);
        // a result that is the input moved, or an earlier result, holds frozen TShapes
        TopologyOracleTests.CompareFlags(theOracle, aTwin, theResult.TShape()!.IsFrozen);

        // the public result is the same shape, frozen
        TopoDS_Shape aPublished = thePublish();
        Assert.True(aPublished.IsEqual(theResult));
        Assert.True(aPublished.TShape()!.IsFrozen);

        TopologyOracleTests.CompareTopology(theOracle, aTwin, theCounts);
        if (theCompareGeometry)
        {
            TopologyOracleTests.CompareGeometry(theOracle, aTwin, theCounts);
        }
        CompareMeshes(theOracle, aTwin);

        foreach (Twin<N.TopoDS_Shape, TopoDS_Shape> aS in AllSubShapes(theInput))
        {
            CompareImage(theOracle, aTwin, theNModifiedShape(aS.N), theModifiedShape(aS.P), "ModifiedShape");
        }
    }

    // A modified shape: its type, orientation and location, and its place among the result's sub-shapes.
    internal static void CompareImage(Oracle theOracle, Twin<N.TopoDS_Shape, TopoDS_Shape> theResult, N.TopoDS_Shape theN,
                                     in TopoDS_Shape theP, string theWhat)
    {
        theOracle.Same((int)theN.ShapeType(), (int)theP.ShapeType(), theWhat + " type");
        theOracle.Same((int)theN.Orientation(), (int)theP.Orientation(), theWhat + " orientation");
        using (N.gp_Trsf aNT = theN.Location().Transformation())
        {
            theOracle.Same(aNT, theP.Location().Transformation(), theWhat + " location");
        }

        var aNMap = new N.TopTools_IndexedMapOfShape();
        N.TopExp.MapShapes(theResult.N, aNMap, false, true);
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theResult.P, aMap, false, true);
        theOracle.Same(aNMap.FindIndex(theN), aMap.FindIndex(theP), theWhat + " place in the result");
    }

    internal static void CompareMeshes(Oracle theOracle, Twin<N.TopoDS_Shape, TopoDS_Shape> theShape)
    {
        foreach (Twin<N.TopoDS_Shape, TopoDS_Shape> aF in TopologyOracleTests.Explore(theShape, TopAbs_FACE))
        {
            using var aNLoc = new N.TopLoc_Location();
            N.Poly_Triangulation? aNT = N.BRep_Tool.Triangulation(N.TopoDS.Face(aF.N), aNLoc);
            Triangulation? aT = BRep_Tool.Triangulation(TopoDS.Face(aF.P), out Placement aLoc);
            theOracle.Same(aNT is null, aT is null, "triangulation is null");
            if (aNT is null || aT is null)
            {
                continue;
            }

            using (N.gp_Trsf aNTrsf = aNLoc.Transformation())
            {
                theOracle.Same(aNTrsf, aLoc.Transformation(), "triangulation location");
            }

            theOracle.Same(aNT.NbNodes(), aT.NodeCount, "triangulation nodes");
            theOracle.Same(aNT.Deflection(), aT.Deflection(), "triangulation deflection");
            for (int i = 1; i <= aT.NodeCount; i++)
            {
                using N.gp_Pnt aNP = aNT.Node(i);
                theOracle.Same(aNP, aT.Node(i), "triangulation node");
            }
        }
    }
}
