#if HAS_ORACLE
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Bento.Collections;
using Xunit.Abstractions;
using N = Build123d.Occt;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// The port's BRepBuilderAPI_Transform, BRepBuilderAPI_Copy and BRepBuilderAPI_GTransform (with its
/// BRepBuilderAPI_NurbsConvert) against native OCCT's, on shapes built identically on both sides, the port's frozen:
/// the result before and after publication, compared as the topology oracle compares shapes (flags, exploration,
/// identity, every vertex, edge and face, triangulations), each input sub-shape's ModifiedShape by its place in the
/// result, and Modified. One framework is reused on several shapes, as Draw's commands do.
/// </summary>
public class BRepBuilderAPIModificationsOracleTests(ITestOutputHelper output)
{
    private const int Scenes = 48;

    // Modified(S) of every input sub-shape: the number of images and each one's place in the result.
    private static void CompareModified(Oracle theOracle, Twin<N.TopoDS_Shape, TopoDS_Shape> theInput, N.TopoDS_Shape theNResult,
                                        in TopoDS_Shape theResult, Func<N.TopoDS_Shape, N.TopoDS_Shape[]> theNModified,
                                        Func<TopoDS_Shape, IBentoList<TopoDS_Shape>> theModified)
    {
        Twin<N.TopoDS_Shape, TopoDS_Shape> aTwin = new(theNResult, theResult);
        foreach (Twin<N.TopoDS_Shape, TopoDS_Shape> aS in BRepToolsModifierOracleTests.AllSubShapes(theInput))
        {
            N.TopoDS_Shape[] aN = theNModified(aS.N);
            IBentoList<TopoDS_Shape> aP = theModified(aS.P);
            theOracle.Same(aN.Length, aP.Count, "Modified count");
            for (int i = 0; i < Math.Min(aN.Length, aP.Count); i++)
            {
                Assert.True(aP[i].TShape() is null || aP[i].TShape()!.IsFrozen());
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
        for (int aScene = 0; aScene < Scenes; aScene++)
        {
            (N.gp_Trsf aNT, gp_Trsf aT) = BRepToolsModifierOracleTests.RandomTrsf(anOracle, aScene % 4);
            using (aNT)
            {
                using var aNTransform = new N.BRepBuilderAPI_Transform(aNT);
                var aTransform = new BRepBuilderAPI_Transform(aT);
                // two shapes through one framework
                for (int aRound = 0; aRound < 2; aRound++)
                {
                    Twin<N.TopoDS_Shape, TopoDS_Shape> aShape = BRepToolsModifierOracleTests.MeshedScene(anOracle, 2 * aScene + aRound);
                    bool aCopyGeom = anOracle.Bool(), aCopyMesh = anOracle.Bool();
                    aNTransform.Perform(aShape.N, aCopyGeom, aCopyMesh);
                    aTransform.Perform(aShape.P, aCopyGeom, aCopyMesh);
                    anOracle.Same(aNTransform.IsDone(), aTransform.IsDone(), "IsDone");
                    TopoDS_Shape aResult = aTransform.InternalShape();
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
        for (int aScene = 0; aScene < Scenes; aScene++)
        {
            Twin<N.TopoDS_Shape, TopoDS_Shape> aShape = BRepToolsModifierOracleTests.MeshedScene(anOracle, aScene);
            bool aCopyGeom = anOracle.Bool(), aCopyMesh = anOracle.Bool();
            using var aNCopy = new N.BRepBuilderAPI_Copy();
            var aCopy = new BRepBuilderAPI_Copy();
            // the same shape twice through one framework (OCCT's second copy keeps the first one's shapes but makes
            // new vertices), then another shape
            Twin<N.TopoDS_Shape, TopoDS_Shape>[] aShapes = [aShape, aShape, BRepToolsModifierOracleTests.MeshedScene(anOracle, aScene + 1)];
            foreach (Twin<N.TopoDS_Shape, TopoDS_Shape> anInput in aShapes)
            {
                aNCopy.Perform(anInput.N, aCopyGeom, aCopyMesh);
                aCopy.Perform(anInput.P, aCopyGeom, aCopyMesh);
                anOracle.Same(aNCopy.IsDone(), aCopy.IsDone(), "IsDone");
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
        for (int aScene = 0; aScene < Scenes; aScene++)
        {
            Twin<N.TopoDS_Shape, TopoDS_Shape> aShape;
            if (aScene % 8 == 7)
            {
                aShape = BRepToolsModifierOracleTests.BSplineScene(anOracle);
            }
            else
            {
                (aShape, _) = BRepGPropOracleTests.RandomScene(anOracle, aScene % 7);
                aShape.P.TShape()!.FreezeGraph();
            }

            (N.gp_GTrsf aNG, gp_GTrsf aG) = anOracle.Int(0, 2) == 0
                ? (new N.gp_GTrsf(new N.gp_Mat(1, 0, 0, 0, 1, 0, 0, 0, 1), new N.gp_XYZ(0, 0, 0)),
                   new gp_GTrsf(new gp_Mat(1, 0, 0, 0, 1, 0, 0, 0, 1), new gp_XYZ(0, 0, 0)))
                : BRepToolsModifierOracleTests.RandomGTrsf(anOracle);
            using (aNG)
            {
                using var aNGTransform = new N.BRepBuilderAPI_GTransform(aShape.N, aNG, false);
                var aGTransform = new BRepBuilderAPI_GTransform(aShape.P, aG);
                anOracle.Same(aNGTransform.IsDone(), aGTransform.IsDone(), "IsDone");
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
}
#endif
