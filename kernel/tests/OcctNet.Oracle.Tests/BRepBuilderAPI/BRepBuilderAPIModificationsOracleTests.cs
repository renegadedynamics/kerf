// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Geometry;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Schematiq.Collections;
using Xunit.Abstractions;
using static Kerf.Topology.ShapeType;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// The port's BRepBuilderAPI_Transform, BRepBuilderAPI_Copy and BRepBuilderAPI_GTransform (with its
/// BRepBuilderAPI_NurbsConvert) against native OCCT's, on shapes built identically on both sides, the port's frozen:
/// the result before and after publication, compared as the topology oracle compares shapes (flags, exploration,
/// identity, every vertex, edge and face, triangulations), each input sub-shape's ModifiedShape by its place in the
/// result, and Modified. One framework is reused on several shapes, as Draw's commands do.
/// </summary>
public sealed class BRepBuilderAPIModificationsOracleTests(ITestOutputHelper output) : IDisposable
{
    private const int Scenes = 48;

    private readonly string myDirectory = Directory.CreateTempSubdirectory("occtnet-brepbuilderapi-mod-").FullName;
    private int myFileNumber;

    public void Dispose()
    {
        try
        {
            Directory.Delete(myDirectory, true);
        }
        catch (IOException)
        {
        }
    }

    // The native shape written to a file and read back by both sides (the port's frozen).
    private Twin<N.TopoDS_Shape, Shape> ReadBoth(N.TopoDS_Shape theShape)
    {
        string aFile = Path.Combine(myDirectory, $"s{myFileNumber++}.brep");
        Assert.True(N.BRepTools.Write(theShape, aFile));
        var aNShape = new N.TopoDS_Shape();
        using (var aBuilder = new N.BRep_Builder())
        {
            Assert.True(N.BRepTools.Read(aNShape, aFile, aBuilder));
        }

        Assert.True(BrepTools.Read(out Shape aPShape, aFile));
        return new(aNShape, aPShape);
    }

    // Modified(S) of every input sub-shape: the number of images and each one's place in the result.
    private static void CompareModified(Oracle theOracle, Twin<N.TopoDS_Shape, Shape> theInput, N.TopoDS_Shape theNResult,
                                        in Shape theResult, Func<N.TopoDS_Shape, N.TopoDS_Shape[]> theNModified,
                                        Func<Shape, ISchematiqList<Shape>> theModified)
    {
        Twin<N.TopoDS_Shape, Shape> aTwin = new(theNResult, theResult);
        foreach (Twin<N.TopoDS_Shape, Shape> aS in BRepToolsModifierOracleTests.AllSubShapes(theInput))
        {
            N.TopoDS_Shape[] aN = theNModified(aS.N);
            ISchematiqList<Shape> aP = theModified(aS.P);
            theOracle.Same(aN.Length, aP.Count, "Modified count");
            for (int i = 0; i < Math.Min(aN.Length, aP.Count); i++)
            {
                Assert.True(aP[i].TShape() is null || aP[i].TShape()!.IsFrozen);
                BRepToolsModifierOracleTests.CompareImage(theOracle, aTwin, aN[i], aP[i], "Modified");
            }
        }
    }

    [Fact]
    public void Transform_answers_as_native_OCCT()
    {
        var anOracle = new Oracle(output);
        var aCounts = new TopologyOracleTests.Counts();
        int aMoved = 0;
        foreach (int aScene in Tape.Budget(Scenes))
        {
            (N.gp_Trsf aNT, Transform3D aT) = BRepToolsModifierOracleTests.RandomTrsf(anOracle, aScene % 4);
            using (aNT)
            {
                using var aNTransform = new N.BRepBuilderAPI_Transform(aNT);
                var aTransform = new Transform(aT);
                // two shapes through one framework
                for (int aRound = 0; aRound < 2; aRound++)
                {
                    Twin<N.TopoDS_Shape, Shape> aShape = BRepToolsModifierOracleTests.MeshedScene(anOracle, 2 * aScene + aRound);
                    bool aCopyGeom = anOracle.Bool(), aCopyMesh = anOracle.Bool();
                    aNTransform.Perform(aShape.N, aCopyGeom, aCopyMesh);
                    aTransform.Perform(aShape.P, aCopyGeom, aCopyMesh);
                    anOracle.Same(aNTransform.IsDone(), aTransform.IsDone, "IsDone");
                    Shape aResult = aTransform.InternalShape();
                    if (aResult.IsPartner(aShape.P))
                    {
                        aMoved++;
                    }

                    BRepToolsModifierOracleTests.CompareResult(anOracle, aShape, aNTransform.Shape(), aResult, aTransform.Shape,
                                                               aNTransform.ModifiedShape, s => aTransform.ModifiedShape(s), aCounts);
                    CompareModified(anOracle, aShape, aNTransform.Shape(), aTransform.Shape(),
                                    s => aNTransform.Modified(s).ToArray(), s => aTransform.Modified(s));
                }
            }
        }

        output.WriteLine($"{2 * Scenes} shapes ({aMoved} moved, the others rebuilt): {aCounts.Shapes} explored sub-shapes, "
                         + $"{aCounts.Edges} edges, {aCounts.Faces} faces");
        anOracle.Report("BRepBuilderAPI_Transform against native OCCT");
    }

    [Fact]
    public void Copy_answers_as_native_OCCT()
    {
        var anOracle = new Oracle(output);
        var aCounts = new TopologyOracleTests.Counts();
        foreach (int aScene in Tape.Budget(Scenes))
        {
            Twin<N.TopoDS_Shape, Shape> aShape = BRepToolsModifierOracleTests.MeshedScene(anOracle, aScene);
            bool aCopyGeom = anOracle.Bool(), aCopyMesh = anOracle.Bool();
            using var aNCopy = new N.BRepBuilderAPI_Copy();
            var aCopy = new Copy();
            // the same shape twice through one framework (OCCT's second copy keeps the first one's shapes but makes
            // new vertices), then another shape
            Twin<N.TopoDS_Shape, Shape>[] aShapes = [aShape, aShape, BRepToolsModifierOracleTests.MeshedScene(anOracle, aScene + 1)];
            foreach (Twin<N.TopoDS_Shape, Shape> anInput in aShapes)
            {
                aNCopy.Perform(anInput.N, aCopyGeom, aCopyMesh);
                aCopy.Perform(anInput.P, aCopyGeom, aCopyMesh);
                anOracle.Same(aNCopy.IsDone(), aCopy.IsDone, "IsDone");
                BRepToolsModifierOracleTests.CompareResult(anOracle, anInput, aNCopy.Shape(), aCopy.InternalShape(), aCopy.Shape,
                                                           aNCopy.ModifiedShape, s => aCopy.ModifiedShape(s), aCounts);
                CompareModified(anOracle, anInput, aNCopy.Shape(), aCopy.Shape(),
                                s => aNCopy.Modified(s).ToArray(), s => aCopy.Modified(s));
            }
        }

        output.WriteLine($"{3 * Scenes} copies: {aCounts.Shapes} explored sub-shapes, {aCounts.Edges} edges, {aCounts.Faces} faces");
        anOracle.Report("BRepBuilderAPI_Copy against native OCCT");
    }

    /// <summary>
    /// GTransform (NurbsConvert then the general transformation) on the global properties oracle's scenes and on
    /// B-spline faces, with an identity or a random affine transformation.
    /// </summary>
    [Fact]
    public void GTransform_answers_as_native_OCCT()
    {
        var anOracle = new Oracle(output, 20261031);
        var aCounts = new TopologyOracleTests.Counts();
        foreach (int aScene in Tape.Budget(Scenes))
        {
            Twin<N.TopoDS_Shape, Shape> aShape;
            if (aScene % 8 == 7)
            {
                aShape = BRepToolsModifierOracleTests.BSplineScene(anOracle);
            }
            else
            {
                (aShape, _) = BRepGPropOracleTests.RandomScene(anOracle, aScene % 7);
                aShape.P.TShape()!.FreezeGraph();
            }

            (N.gp_GTrsf aNG, GeneralTransform3D aG) = anOracle.Int(0, 2) == 0
                ? (new N.gp_GTrsf(new N.gp_Mat(1, 0, 0, 0, 1, 0, 0, 0, 1), new N.gp_XYZ(0, 0, 0)),
                   new GeneralTransform3D(new Matrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1), new Xyz(0, 0, 0)))
                : BRepToolsModifierOracleTests.RandomGTrsf(anOracle);
            using (aNG)
            {
                using var aNGTransform = new N.BRepBuilderAPI_GTransform(aShape.N, aNG, false);
                var aGTransform = new GTransform(aShape.P, aG);
                anOracle.Same(aNGTransform.IsDone(), aGTransform.IsDone, "IsDone");
                BRepToolsModifierOracleTests.CompareResult(anOracle, aShape, aNGTransform.Shape(), aGTransform.InternalShape(),
                                                           aGTransform.Shape, aNGTransform.ModifiedShape,
                                                           s => aGTransform.ModifiedShape(s), aCounts);
                CompareModified(anOracle, aShape, aNGTransform.Shape(), aGTransform.Shape(),
                                s => aNGTransform.Modified(s).ToArray(), s => aGTransform.Modified(s));
            }
        }

        output.WriteLine($"{Scenes} scenes: {aCounts.Shapes} explored sub-shapes, {aCounts.Edges} edges, {aCounts.Faces} faces");
        anOracle.Report("BRepBuilderAPI_GTransform (BRepBuilderAPI_NurbsConvert, then the transformation) against native OCCT");
    }

    /// <summary>
    /// Transform (with a copy of the geometry and a scaling, mirror or rigid motion), Copy and GTransform on random
    /// native shapes (primitives, sweeps, Booleans, fillets, lofts, placed and meshed at random) that both sides read
    /// from native OCCT's file.
    /// </summary>
    [Fact]
    public void Modifications_of_native_shapes_answer_as_native_OCCT()
    {
        var anOracle = new Oracle(output, 31013);
        var aCounts = new TopologyOracleTests.Counts();
        const int Shapes = 60;
        int aSurfaceless = 0;
        foreach (int k in Tape.Budget(Shapes))
        {
            Twin<N.TopoDS_Shape, Shape> aShape = ReadBoth(BRepToolsIOOracleTests.NativeShape(anOracle, k, out _));
            switch (k % 3)
            {
                case 0:
                {
                    (N.gp_Trsf aNT, Transform3D aT) = BRepToolsModifierOracleTests.RandomTrsf(anOracle, k % 4);
                    using (aNT)
                    {
                        bool aCopyMesh = anOracle.Bool();
                        using var aN = new N.BRepBuilderAPI_Transform(aShape.N, aNT, true, aCopyMesh);
                        var aP = new Transform(aShape.P, aT, true, aCopyMesh);
                        BRepToolsModifierOracleTests.CompareResult(anOracle, aShape, aN.Shape(), aP.InternalShape(), aP.Shape,
                                                                   aN.ModifiedShape, s => aP.ModifiedShape(s), aCounts);
                    }
                    break;
                }
                case 1:
                {
                    // twice through one framework: the second Perform sets the continuities of the first result's
                    // edges again (fillets, sweeps), which the port skips on its published, frozen edges
                    bool aCopyGeom = anOracle.Bool(), aCopyMesh = anOracle.Bool();
                    using var aN = new N.BRepBuilderAPI_Copy();
                    var aP = new Copy();
                    for (int aRound = 0; aRound < 2; aRound++)
                    {
                        aN.Perform(aShape.N, aCopyGeom, aCopyMesh);
                        aP.Perform(aShape.P, aCopyGeom, aCopyMesh);
                        BRepToolsModifierOracleTests.CompareResult(anOracle, aShape, aN.Shape(), aP.InternalShape(), aP.Shape,
                                                                   aN.ModifiedShape, s => aP.ModifiedShape(s), aCounts);
                    }
                    break;
                }
                default:
                {
                    (N.gp_GTrsf aNG, GeneralTransform3D aG) = BRepToolsModifierOracleTests.RandomGTrsf(anOracle);
                    using (aNG)
                    {
                        using var aN = new N.BRepBuilderAPI_GTransform(aShape.N, aNG, false);
                        var aP = new GTransform(aShape.P, aG);
                        // OCCT: a meshed face on a B-spline or Bezier surface comes out of the NURBS conversion without
                        // its surface (occt-issues.md), on both sides; native BRepTools::UVBounds can't read such a face
                        bool aGeometry = true;
                        foreach (Twin<N.TopoDS_Shape, Shape> aF in TopologyOracleTests.Explore(new(aN.Shape(), aP.InternalShape()), ShapeType.Face))
                        {
                            bool aNNull = N.BRep_Tool.Surface(N.TopoDS.Face(aF.N)) is null;
                            anOracle.Same(aNNull, Tool.Surface(ShapeCasts.Face(aF.P)) is null, "face without surface");
                            aGeometry &= !aNNull;
                        }
                        if (!aGeometry)
                        {
                            aSurfaceless++;
                        }

                        BRepToolsModifierOracleTests.CompareResult(anOracle, aShape, aN.Shape(), aP.InternalShape(), aP.Shape,
                                                                   aN.ModifiedShape, s => aP.ModifiedShape(s), aCounts, aGeometry);
                    }
                    break;
                }
            }
        }

        output.WriteLine($"{Shapes} native shapes: {aCounts.Shapes} explored sub-shapes, {aCounts.Edges} edges, {aCounts.Faces} faces; "
                         + $"{aSurfaceless} GTransform results with faces left without surfaces (geometry not compared)");
        anOracle.Report("BRepBuilderAPI_Transform, Copy and GTransform on native shapes against native OCCT");
    }
}
