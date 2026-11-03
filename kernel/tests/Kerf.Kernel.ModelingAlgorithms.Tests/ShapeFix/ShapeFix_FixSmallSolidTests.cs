// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Reflection;
using Kerf.Kernel.ModelingData.Tests;
using Xunit.Abstractions;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Unit tests of ShapeFix_FixSmallSolid (unit 6H3): the heal group's <c>drop_small_solids</c> grid (OCCT's
/// DropSmallSolids operator on <c>test.brep</c>, compared with its reference files as the grid's <c>end</c> script does:
/// shape counts and volume, surface and linear properties), on the frozen input.
/// </summary>
public class ShapeFix_FixSmallSolidTests(ITestOutputHelper output)
{
    private static string Metadata(string theKey) =>
        typeof(ShapeFix_FixSmallSolidTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == theKey).Value!;

    private static string? DataFile(string theName)
    {
        string aRoot = Metadata("OcctTestDataDir");
        if (string.IsNullOrEmpty(aRoot) || !Directory.Exists(aRoot))
        {
            return null;
        }
        string aFile = Path.Combine(aRoot, "brep", theName);
        return File.Exists(aFile) ? aFile : null;
    }

    private static Shape ReadFrozen(string theFile)
    {
        Assert.True(BrepTools.Read(out Shape aShape, theFile), theFile);
        aShape.TShape()!.FreezeGraph();
        return aShape;
    }

    private static int Count(in Shape theShape, ShapeType theType)
    {
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, theType, aMap);
        return aMap.Count;
    }

    // The center of gravity and the matrix of inertia, as Draw's vprops/sprops/lprops print them.
    private static double[] Props(in Shape theShape, int theKind)
    {
        var aProps = new Properties();
        switch (theKind)
        {
            case 0:
                MassPropertiesBuilder.VolumeProperties(theShape, aProps);
                break;
            case 1:
                MassPropertiesBuilder.SurfaceProperties(theShape, aProps);
                break;
            default:
                MassPropertiesBuilder.LinearProperties(theShape, aProps);
                break;
        }
        Point3D aG = aProps.CentreOfMass();
        Matrix3x3 aM = aProps.MatrixOfInertia();
        return [aG.X, aG.Y, aG.Z,
                aM.Value(1, 1), aM.Value(1, 2), aM.Value(1, 3),
                aM.Value(2, 1), aM.Value(2, 2), aM.Value(2, 3),
                aM.Value(3, 1), aM.Value(3, 2), aM.Value(3, 3)];
    }

    // The grid's cases: name, merge, volume threshold, width factor threshold, reference file.
    public static TheoryData<string, bool, double, double, string> Cases() => new()
    {
        { "M1 merge0v", true, 400, -1, "test.brep" },
        { "M2 merge1v", true, 500, -1, "merge1.brep" },
        { "M3 merge2v", true, 1000, -1, "merge2.brep" },
        { "R1 remove0v", false, 400, -1, "test.brep" },
        { "R2 remove1v", false, 500, -1, "remove1.brep" },
        { "R3 remove2v", false, 1000, -1, "remove2.brep" },
        { "R4 remove0w", false, -1, 2, "test.brep" },
        { "R5 remove1w", false, -1, 3, "remove1.brep" },
        { "R6 remove2w", false, -1, 4, "remove2.brep" },
        { "R7 remove1vw", false, 500, 4, "remove1.brep" },
        { "R8 remove1wv", false, 1000, 3, "remove1.brep" },
    };

    [Theory]
    [MemberData(nameof(Cases))]
    public void DropSmallSolids_HealGroupGrid_MatchesTheReference(string theCase, bool theMerge, double theVolume, double theWidth, string theReference)
    {
        string? anInput = DataFile("test.brep");
        string? aRefFile = DataFile(theReference);
        if (anInput is null || aRefFile is null)
        {
            output.WriteLine("OcctTestDataDir isn't set: nothing to do");
            return;
        }
        Shape aShape = ReadFrozen(anInput);
        Shape aRef = ReadFrozen(aRefFile);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aShape);

        // as ShapeProcess's DropSmallSolids operator
        var aFSS = new FixSmallSolidFixer();
        if (theVolume >= 0)
        {
            aFSS.SetVolumeThreshold(theVolume);
        }
        if (theWidth >= 0)
        {
            aFSS.SetWidthFactorThreshold(theWidth);
        }
        var aReShape = new ReShapeBuilder();
        Shape aResult = theMerge ? aFSS.Merge(aShape, aReShape) : aFSS.Remove(aShape, aReShape);

        aSnap.AssertUnchanged(aShape);
        Assert.True(aResult.TShape()!.IsFrozen);
        foreach (ShapeType aType in new[] { ShapeType.Solid, ShapeType.Shell, ShapeType.Face, ShapeType.Edge, ShapeType.Vertex })
        {
            Assert.True(Count(aRef, aType) == Count(aResult, aType), $"{theCase}: {aType}");
        }
        for (int aKind = 0; aKind < 3; aKind++)
        {
            double[] aR = Props(aResult, aKind), aE = Props(aRef, aKind);
            for (int i = 0; i < aR.Length; i++)
            {
                Assert.True(Math.Abs(aR[i] - aE[i]) <= 1e-8, $"{theCase}: props {aKind}, value {i}: {aR[i]} != {aE[i]}");
            }
        }

        // the faces of the result are the caller's own (removing and merging only rebuild shells and solids)
        var anInputFaces = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(aShape, ShapeType.Face, anInputFaces);
        for (var anExp = new Explorer(aResult, ShapeType.Face); anExp.More(); anExp.Next())
        {
            Assert.True(anInputFaces.Contains(anExp.Current()), theCase);
        }
        if (theReference == "test.brep")
        {
            Assert.True(aResult.IsSame(aShape), theCase); // nothing small: the input itself
        }
    }

    private static Shape Box(double x, double y, double z, double dx, double dy, double dz) =>
        new MakeBox(new Point3D(x, y, z), dx, dy, dz).Shape();

    private static Compound Compound(params Shape[] theShapes)
    {
        var aC = new Compound();
        var aB = new BrepBuilder();
        aB.MakeCompound(ref aC);
        foreach (Shape aS in theShapes)
        {
            aB.Add(aC, aS);
        }
        aC.TShape()!.FreezeGraph();
        return aC;
    }

    [Fact]
    public void Remove_ByVolumeOrWidth_AndFixModes()
    {
        Shape aBig = Box(0, 0, 0, 10, 10, 10);   // volume 1000, width factor 2 * 1000 / 600 = 3.33
        Shape aThin = Box(20, 0, 0, 10, 10, 0.1); // volume 10, width factor 2 * 10 / 204 = 0.098
        Compound aShape = Compound(aBig, aThin);

        // no threshold: nothing done, the input itself
        var aFSS = new FixSmallSolidFixer();
        Assert.True(aFSS.Remove(aShape, new ReShapeBuilder()).IsSame(aShape));

        // volume only
        aFSS.SetVolumeThreshold(100);
        Shape aResult = aFSS.Remove(aShape, new ReShapeBuilder());
        Assert.Equal(1, Count(aResult, ShapeType.Solid));
        Assert.True(Count(aResult, ShapeType.Face) == 6);

        // both thresholds in mode 0: the thin solid must pass both (width factor 0.098 < 1, volume 10 < 100)
        aFSS.SetWidthFactorThreshold(0.05);
        Assert.Equal(2, Count(aFSS.Remove(aShape, new ReShapeBuilder()), ShapeType.Solid));
        aFSS.SetWidthFactorThreshold(1);
        Assert.Equal(1, Count(aFSS.Remove(aShape, new ReShapeBuilder()), ShapeType.Solid));

        // mode 2: volume only; mode 1: width only
        aFSS.SetWidthFactorThreshold(0.05);
        aFSS.SetFixMode(2);
        Assert.Equal(1, Count(aFSS.Remove(aShape, new ReShapeBuilder()), ShapeType.Solid));
        aFSS.SetFixMode(1);
        Assert.Equal(2, Count(aFSS.Remove(aShape, new ReShapeBuilder()), ShapeType.Solid));
        aFSS.SetWidthFactorThreshold(5); // both solids are thinner than that
        Assert.Equal(0, Count(aFSS.Remove(aShape, new ReShapeBuilder()), ShapeType.Solid));
        aFSS.SetFixMode(7); // out of range: 0
        aFSS.SetVolumeThreshold(-1); // cleared
        aFSS.SetWidthFactorThreshold(-1);
        Assert.True(aFSS.Remove(aShape, new ReShapeBuilder()).IsSame(aShape));

        // a face or a shell isn't a valid input
        Shape aFace = new Explorer(aBig, ShapeType.Face).Current();
        aFSS.SetVolumeThreshold(1e9);
        Assert.True(aFSS.Remove(aFace, new ReShapeBuilder()).IsSame(aFace));
    }

    [Fact]
    public void Merge_IntoTheAdjacentSolid_SharingTheFace()
    {
        // a big and a small box sharing the face x = 10 (a compsolid-like compound with one shared face)
        Shape aBig = Box(0, 0, 0, 10, 10, 10);
        Shape aSmall = Box(10, 0, 0, 1, 10, 10);
        // build the shared-face compound by hand: the small box's face at x = 10 replaced by the big box's
        Face aBigFace = new(), aSmallFace = new();
        for (var anExp = new Explorer(aBig, ShapeType.Face); anExp.More(); anExp.Next())
        {
            var aProps = new Properties();
            MassPropertiesBuilder.SurfaceProperties(anExp.Current(), aProps);
            if (Math.Abs(aProps.CentreOfMass().X - 10) < 1e-9)
            {
                aBigFace = ShapeCasts.Face(anExp.Current());
            }
        }
        for (var anExp = new Explorer(aSmall, ShapeType.Face); anExp.More(); anExp.Next())
        {
            var aProps = new Properties();
            MassPropertiesBuilder.SurfaceProperties(anExp.Current(), aProps);
            if (Math.Abs(aProps.CentreOfMass().X - 10) < 1e-9)
            {
                aSmallFace = ShapeCasts.Face(anExp.Current());
            }
        }
        var aRe = new ReShape();
        aRe.Replace(aSmallFace, aBigFace.Reversed());
        Shape aSmallShared = aRe.Apply(aSmall);
        Compound aShape = Compound(aBig, aSmallShared);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aShape);

        var aFSS = new FixSmallSolidFixer();
        aFSS.SetFixMode(2);
        aFSS.SetVolumeThreshold(200);
        var aContext = new ReShapeBuilder();
        Shape aResult = aFSS.Merge(aShape, aContext);

        aSnap.AssertUnchanged(aShape);
        Assert.True(aResult.TShape()!.IsFrozen);
        Assert.Equal(1, Count(aResult, ShapeType.Solid));
        // the merged shell: 5 + 5 faces, the shared one dropped
        Assert.Equal(10, Count(aResult, ShapeType.Face));
        var aProps2 = new Properties();
        MassPropertiesBuilder.VolumeProperties(aResult, aProps2);
        Assert.Equal(1100, aProps2.Mass(), 6);
        Assert.True(aContext.IsRecorded(new Explorer(aSmallShared, ShapeType.Solid).Current()));
    }
}
