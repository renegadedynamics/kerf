// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using OcctNet.TestHarness;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>BOPAlgo_CheckerSI (unit 6G): self-interference of a single shape, by level of check.</summary>
public class BOPAlgo_CheckerSITests
{
    internal static Shape Box(double x, double y, double z, double dx, double dy, double dz) =>
        new BRepPrimAPI_MakeBox(new Point3D(x, y, z), dx, dy, dz).Shape();

    internal static Compound Compound(params Shape[] theShapes)
    {
        BrepBuilder aB = new();
        Compound aC = new();
        aB.MakeCompound(ref aC);
        foreach (Shape aS in theShapes)
        {
            aB.Add(aC, aS);
        }
        return aC;
    }

    private static BOPAlgo_CheckerSI Check(in Shape theS, int theLevel = 9, bool theParallel = false)
    {
        BOPAlgo_CheckerSI aChecker = new();
        NCollection_List<Shape> aLS = new();
        aLS.Append(theS);
        aChecker.SetArguments(aLS);
        aChecker.SetLevelOfCheck(theLevel);
        aChecker.SetRunParallel(theParallel);
        aChecker.Perform();
        return aChecker;
    }

    // The interferences between source shapes, as "type1/type2" counts
    private static Dictionary<string, int> Kinds(BOPAlgo_CheckerSI theChecker)
    {
        BOPDS_DS aDS = theChecker.PDS()!;
        Dictionary<string, int> aKinds = [];
        foreach (BOPDS_Pair aPK in aDS.Interferences())
        {
            aPK.Indices(out int n1, out int n2);
            if (aDS.IsNewShape(n1) || aDS.IsNewShape(n2))
            {
                continue;
            }
            string aKey = $"{Kerf.Engine.Foundation.OcctTypeNames.Of(aDS.ShapeInfo(n1).ShapeType())}/{Kerf.Engine.Foundation.OcctTypeNames.Of(aDS.ShapeInfo(n2).ShapeType())}";
            aKinds[aKey] = aKinds.GetValueOrDefault(aKey) + 1;
        }
        return aKinds;
    }

    [Fact]
    public void A_valid_box_has_no_interference()
    {
        BOPAlgo_CheckerSI aChecker = Check(Box(0, 0, 0, 1, 2, 3));
        Assert.False(aChecker.HasErrors());
        Assert.Empty(Kinds(aChecker));
    }

    [Fact]
    public void Overlapping_boxes_in_a_compound_interfere()
    {
        BOPAlgo_CheckerSI aChecker = Check(Compound(Box(0, 0, 0, 10, 10, 10), Box(5, 5, 5, 10, 10, 10)));
        Assert.False(aChecker.HasErrors());
        Dictionary<string, int> aKinds = Kinds(aChecker);
        Assert.True(aKinds.Values.Sum() > 0);
        Assert.Contains(aKinds.Keys, k => k.Contains("TopAbs_FACE", StringComparison.Ordinal));
    }

    [Fact]
    public void A_box_inside_another_interferes_as_solids()
    {
        BOPAlgo_CheckerSI aChecker = Check(Compound(Box(0, 0, 0, 10, 10, 10), Box(2, 2, 2, 1, 1, 1)));
        BOPDS_DS aDS = aChecker.PDS()!;
        Assert.True(aDS.InterfVZ().Length() > 0);
        Assert.True(aDS.InterfZZ().Length() > 0);
        // below the solid levels, nothing is found
        Assert.Empty(Kinds(Check(Compound(Box(0, 0, 0, 10, 10, 10), Box(2, 2, 2, 1, 1, 1)), 5)));
    }

    [Fact]
    public void The_level_of_check_limits_the_interference_types()
    {
        Shape aC = Compound(Box(0, 0, 0, 10, 10, 10), Box(5, 5, 5, 10, 10, 10));
        Assert.Empty(Kinds(Check(aC, 0)));
        Assert.DoesNotContain(Kinds(Check(aC, 3)).Keys, k => k.Contains("TopAbs_FACE/TopAbs_FACE", StringComparison.Ordinal));
        // an invalid level is ignored (the default, 9, stays)
        BOPAlgo_CheckerSI aChecker = new();
        aChecker.SetLevelOfCheck(12);
        aChecker.SetLevelOfCheck(-1);
        NCollection_List<Shape> aLS = new();
        aLS.Append(aC);
        aChecker.SetArguments(aLS);
        aChecker.Perform();
        Assert.Equal(Kinds(Check(aC)), Kinds(aChecker));
    }

    [Fact]
    public void Several_arguments_are_an_error()
    {
        BOPAlgo_CheckerSI aChecker = new();
        NCollection_List<Shape> aLS = new();
        aLS.Append(Box(0, 0, 0, 1, 1, 1));
        aLS.Append(Box(2, 0, 0, 1, 1, 1));
        aChecker.SetArguments(aLS);
        aChecker.Perform();
        Assert.True(aChecker.HasError(typeof(BOPAlgo_AlertMultipleArguments)));
    }

    [Fact]
    public void A_frozen_argument_is_never_written()
    {
        Shape aC = Compound(Box(0, 0, 0, 10, 10, 10), Box(5, 5, 5, 10, 10, 10), Box(10, 0, 0, 3, 3, 3));
        aC.TShape()!.FreezeGraph();
        string aBefore = BRepTextComparison.Write(aC);
        BOPAlgo_CheckerSI aChecker = Check(aC);
        Assert.False(aChecker.HasErrors());
        Assert.Equal(aBefore, BRepTextComparison.Write(aC));
        // the same on an unfrozen copy: the checker is non-destructive
        Shape aCopy = BrepTools.ExactCopyForWrite(aC, out _);
        string aCopyBefore = BRepTextComparison.Write(aCopy);
        Assert.Equal(Kinds(aChecker), Kinds(Check(aCopy)));
        Assert.Equal(aCopyBefore, BRepTextComparison.Write(aCopy));
    }

    [Fact]
    public void Serial_equals_parallel()
    {
        Shape aC = Compound(Box(0, 0, 0, 10, 10, 10), Box(5, 5, 5, 10, 10, 10),
                                   new BRepPrimAPI_MakeSphere(new Point3D(10, 10, 10), 4).Shape(),
                                   new BRepPrimAPI_MakeTorus(new Frame3D(new Point3D(3, 3, 3), new Direction3D(0, 0, 1)), 4, 1).Shape());
        string aSerial = DeterminismHarness.AssertSerialEqualsParallel(() => PaveFillerDump.Write(Check(aC, 9, true).PDS()!), 2);
        Assert.Equal(PaveFillerDump.Write(Check(aC).PDS()!), aSerial);
    }

    [Fact]
    public void The_interference_table_iterates_in_OCCT_order()
    {
        // OCCT's NCollection_Map<BOPDS_Pair>: MurmurHash64A of the sorted indices, bucket = hash % buckets + 1
        BOPDS_MapOfPair aMap = new(100);
        BOPDS_Pair aP = new();
        List<(int, int)> anAdded = [];
        for (int i = 0; i < 300; i += 7)
        {
            aP.SetIndices(i, i / 3);
            aMap.Add(aP);
            anAdded.Add((i, i / 3));
        }
        aP.SetIndices(7 / 3, 7);
        Assert.False(aMap.Add(aP)); // a pair is unordered
        Assert.Equal(anAdded.Count, aMap.Extent());
        List<BOPDS_Pair> anOrder = [.. aMap];
        Assert.Equal(anAdded.Count, anOrder.Count);
        // not insertion order, and each bucket's chain holds the latest first
        Assert.NotEqual(anAdded, anOrder.Select(p => { p.Indices(out int a, out int b); return (a, b); }));
        aMap.Clear();
        Assert.Equal(0, aMap.Extent());
    }
}
