// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Reflection;
using OcctNet.ModelingData.Tests;
using Xunit.Abstractions;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

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

    private static TopoDS_Shape ReadFrozen(string theFile)
    {
        Assert.True(BRepTools.Read(out TopoDS_Shape aShape, theFile), theFile);
        aShape.TShape()!.FreezeGraph();
        return aShape;
    }

    private static int Count(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, theType, aMap);
        return aMap.Extent();
    }

    // The center of gravity and the matrix of inertia, as Draw's vprops/sprops/lprops print them.
    private static double[] Props(in TopoDS_Shape theShape, int theKind)
    {
        var aProps = new GProp_GProps();
        switch (theKind)
        {
            case 0:
                BRepGProp.VolumeProperties(theShape, aProps);
                break;
            case 1:
                BRepGProp.SurfaceProperties(theShape, aProps);
                break;
            default:
                BRepGProp.LinearProperties(theShape, aProps);
                break;
        }
        gp_Pnt aG = aProps.CentreOfMass();
        gp_Mat aM = aProps.MatrixOfInertia();
        return [aG.X(), aG.Y(), aG.Z(),
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
        TopoDS_Shape aShape = ReadFrozen(anInput);
        TopoDS_Shape aRef = ReadFrozen(aRefFile);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aShape);

        // as ShapeProcess's DropSmallSolids operator
        var aFSS = new ShapeFix_FixSmallSolid();
        if (theVolume >= 0)
        {
            aFSS.SetVolumeThreshold(theVolume);
        }
        if (theWidth >= 0)
        {
            aFSS.SetWidthFactorThreshold(theWidth);
        }
        var aReShape = new ShapeBuild_ReShape();
        TopoDS_Shape aResult = theMerge ? aFSS.Merge(aShape, aReShape) : aFSS.Remove(aShape, aReShape);

        aSnap.AssertUnchanged(aShape);
        Assert.True(aResult.TShape()!.IsFrozen());
        foreach (TopAbs_ShapeEnum aType in new[] { TopAbs_SOLID, TopAbs_SHELL, TopAbs_FACE, TopAbs_EDGE, TopAbs_VERTEX })
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
        var anInputFaces = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(aShape, TopAbs_FACE, anInputFaces);
        for (var anExp = new TopExp_Explorer(aResult, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            Assert.True(anInputFaces.Contains(anExp.Current()), theCase);
        }
        if (theReference == "test.brep")
        {
            Assert.True(aResult.IsSame(aShape), theCase); // nothing small: the input itself
        }
    }

    private static TopoDS_Shape Box(double x, double y, double z, double dx, double dy, double dz) =>
        new BRepPrimAPI_MakeBox(new gp_Pnt(x, y, z), dx, dy, dz).Shape();

    private static TopoDS_Compound Compound(params TopoDS_Shape[] theShapes)
    {
        var aC = new TopoDS_Compound();
        var aB = new BRep_Builder();
        aB.MakeCompound(ref aC);
        foreach (TopoDS_Shape aS in theShapes)
        {
            aB.Add(aC, aS);
        }
        aC.TShape()!.FreezeGraph();
        return aC;
    }

    [Fact]
    public void Remove_ByVolumeOrWidth_AndFixModes()
    {
        TopoDS_Shape aBig = Box(0, 0, 0, 10, 10, 10);   // volume 1000, width factor 2 * 1000 / 600 = 3.33
        TopoDS_Shape aThin = Box(20, 0, 0, 10, 10, 0.1); // volume 10, width factor 2 * 10 / 204 = 0.098
        TopoDS_Compound aShape = Compound(aBig, aThin);

        // no threshold: nothing done, the input itself
        var aFSS = new ShapeFix_FixSmallSolid();
        Assert.True(aFSS.Remove(aShape, new ShapeBuild_ReShape()).IsSame(aShape));

        // volume only
        aFSS.SetVolumeThreshold(100);
        TopoDS_Shape aResult = aFSS.Remove(aShape, new ShapeBuild_ReShape());
        Assert.Equal(1, Count(aResult, TopAbs_SOLID));
        Assert.True(Count(aResult, TopAbs_FACE) == 6);

        // both thresholds in mode 0: the thin solid must pass both (width factor 0.098 < 1, volume 10 < 100)
        aFSS.SetWidthFactorThreshold(0.05);
        Assert.Equal(2, Count(aFSS.Remove(aShape, new ShapeBuild_ReShape()), TopAbs_SOLID));
        aFSS.SetWidthFactorThreshold(1);
        Assert.Equal(1, Count(aFSS.Remove(aShape, new ShapeBuild_ReShape()), TopAbs_SOLID));

        // mode 2: volume only; mode 1: width only
        aFSS.SetWidthFactorThreshold(0.05);
        aFSS.SetFixMode(2);
        Assert.Equal(1, Count(aFSS.Remove(aShape, new ShapeBuild_ReShape()), TopAbs_SOLID));
        aFSS.SetFixMode(1);
        Assert.Equal(2, Count(aFSS.Remove(aShape, new ShapeBuild_ReShape()), TopAbs_SOLID));
        aFSS.SetWidthFactorThreshold(5); // both solids are thinner than that
        Assert.Equal(0, Count(aFSS.Remove(aShape, new ShapeBuild_ReShape()), TopAbs_SOLID));
        aFSS.SetFixMode(7); // out of range: 0
        aFSS.SetVolumeThreshold(-1); // cleared
        aFSS.SetWidthFactorThreshold(-1);
        Assert.True(aFSS.Remove(aShape, new ShapeBuild_ReShape()).IsSame(aShape));

        // a face or a shell isn't a valid input
        TopoDS_Shape aFace = new TopExp_Explorer(aBig, TopAbs_FACE).Current();
        aFSS.SetVolumeThreshold(1e9);
        Assert.True(aFSS.Remove(aFace, new ShapeBuild_ReShape()).IsSame(aFace));
    }

    [Fact]
    public void Merge_IntoTheAdjacentSolid_SharingTheFace()
    {
        // a big and a small box sharing the face x = 10 (a compsolid-like compound with one shared face)
        TopoDS_Shape aBig = Box(0, 0, 0, 10, 10, 10);
        TopoDS_Shape aSmall = Box(10, 0, 0, 1, 10, 10);
        // build the shared-face compound by hand: the small box's face at x = 10 replaced by the big box's
        TopoDS_Face aBigFace = new(), aSmallFace = new();
        for (var anExp = new TopExp_Explorer(aBig, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            var aProps = new GProp_GProps();
            BRepGProp.SurfaceProperties(anExp.Current(), aProps);
            if (Math.Abs(aProps.CentreOfMass().X() - 10) < 1e-9)
            {
                aBigFace = TopoDS.Face(anExp.Current());
            }
        }
        for (var anExp = new TopExp_Explorer(aSmall, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            var aProps = new GProp_GProps();
            BRepGProp.SurfaceProperties(anExp.Current(), aProps);
            if (Math.Abs(aProps.CentreOfMass().X() - 10) < 1e-9)
            {
                aSmallFace = TopoDS.Face(anExp.Current());
            }
        }
        var aRe = new BRepTools_ReShape();
        aRe.Replace(aSmallFace, aBigFace.Reversed());
        TopoDS_Shape aSmallShared = aRe.Apply(aSmall);
        TopoDS_Compound aShape = Compound(aBig, aSmallShared);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aShape);

        var aFSS = new ShapeFix_FixSmallSolid();
        aFSS.SetFixMode(2);
        aFSS.SetVolumeThreshold(200);
        var aContext = new ShapeBuild_ReShape();
        TopoDS_Shape aResult = aFSS.Merge(aShape, aContext);

        aSnap.AssertUnchanged(aShape);
        Assert.True(aResult.TShape()!.IsFrozen());
        Assert.Equal(1, Count(aResult, TopAbs_SOLID));
        // the merged shell: 5 + 5 faces, the shared one dropped
        Assert.Equal(10, Count(aResult, TopAbs_FACE));
        var aProps2 = new GProp_GProps();
        BRepGProp.VolumeProperties(aResult, aProps2);
        Assert.Equal(1100, aProps2.Mass(), 6);
        Assert.True(aContext.IsRecorded(new TopExp_Explorer(aSmallShared, TopAbs_SOLID).Current()));
    }
}
