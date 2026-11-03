// Port of: src/FoundationClasses/TKernel/GTests/TopLoc_Location_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Topology;

using Kerf.Geometry;

using Kerf.Topology;

namespace OcctNet.Foundation.Tests;

public class TopLoc_Location_Test
{
    /// <remarks>
    /// OCCT moves a vertex by each location and reads its point back through BRep_Tool, concurrently. TopoDS and
    /// BRep aren't ported yet, so this applies each location's transformation to the origin instead, which is what
    /// BRep_Tool::Pnt does with the vertex's location.
    /// </remarks>
    [Fact]
    public void OCC25545_ConcurrentTransformationAccess()
    {
        // Bug OCC25545: TopLoc_Location::Transformation() provokes data races
        // This test verifies that concurrent access to TopLoc_Location::Transformation()
        // does not cause data races or incorrect geometry results

        // Give the i-th location the transformation that translates the origin on the vector (i,0,0)
        int n = 1000;
        var aLocVec = new Placement[n];
        for (int i = 1; i < n; ++i)
        {
            Transform3D aTrsf = Transform3D.Translation(new Vector3D(1, 0, 0));
            aLocVec[i] = aLocVec[i - 1] * new Placement(aTrsf);
        }

        // Evaluator function will access the transformations concurrently
        int anIsRaceDetected = 0;
        Parallel.For(0, n, i =>
        {
            if (Volatile.Read(ref anIsRaceDetected) == 0)
            {
                Point3D aPoint = Constants.Origin().Transformed(aLocVec[i].Transformation());
                if (aPoint.X() != i)
                {
                    Interlocked.Increment(ref anIsRaceDetected);
                }
            }
        });

        // Verify no data race was detected
        Assert.Equal(0, anIsRaceDetected);
    }
}

/// <summary>TopLoc_Location's composition rules and its identity-based equality, which OCCT has no GTests for.</summary>
public class TopLoc_LocationSemanticsTests
{
    private static Transform3D Rotation(double theAngle)
        => Transform3D.Rotation(new Axis3D(new Point3D(1, 2, 3), new Direction3D(0.2, 0.3, 0.9)), theAngle);

    private static Transform3D Translation(double theX) => Transform3D.Translation(new Vector3D(theX, 2, -1));

    private static void AssertSameTrsf(in Transform3D theExpected, in Transform3D theActual, double theTol = 1e-12)
    {
        for (int i = 1; i <= 3; i++)
        {
            for (int j = 1; j <= 4; j++)
            {
                Assert.Equal(theExpected.Value(i, j), theActual.Value(i, j), theTol);
            }
        }
    }

    [Fact]
    public void Default_is_identity()
    {
        Placement aDefault = default;
        var aNew = new Placement();
        Assert.True(aDefault.IsIdentity);
        Assert.True(aNew.IsIdentity);
        Assert.Equal(aNew, aDefault);
        Assert.Equal(0, aDefault.HashCode());
        Assert.Equal(new Transform3D(), aDefault.Transformation());
        Assert.True(aDefault.NextLocation().IsIdentity);
        Assert.ThrowsAny<NoSuchObjectException>(() => aDefault.FirstDatum());
        Assert.ThrowsAny<NoSuchObjectException>(() => aDefault.FirstPower());
    }

    [Fact]
    public void Equality_is_by_datum_identity()
    {
        Transform3D aT = Rotation(0.7);
        var aL1 = new Placement(aT);
        var aL2 = new Placement(aT);
        Placement aCopy = aL1;

        Assert.True(aL1.IsEqual(aCopy));
        Assert.True(aL1 == aCopy);
        Assert.Equal(aL1.GetHashCode(), aCopy.GetHashCode());

        // Same transformation, different datums
        Assert.True(aL1.IsDifferent(aL2));
        Assert.True(aL1 != aL2);
        Assert.False(aL1.Equals(aL2));
        Assert.Equal(aL1.Transformation(), aL2.Transformation());

        // Same datum, built separately
        var aDatum = new Datum3D(aT);
        Assert.Equal(new Placement(aDatum), new Placement(aDatum));
        Assert.Same(aDatum, new Placement(aDatum).FirstDatum());
    }

    [Fact]
    public void Equal_chains_built_separately_are_equal_with_equal_hashes()
    {
        var aD1 = new Datum3D(Rotation(0.3));
        var aD2 = new Datum3D(Translation(4));
        Placement aA = new Placement(aD1) * new Placement(aD2).Powered(3);
        Placement aB = new Placement(aD1) * new Placement(aD2) * new Placement(aD2).Powered(2);
        Assert.True(aA.IsEqual(aB));
        Assert.Equal(aA.HashCode(), aB.HashCode());
        Assert.Equal(aA.GetHashCode(), aB.GetHashCode());

        // Lists are not shared, yet the structure is the same: (aD2^3) then aD1
        Assert.Same(aD2, aA.FirstDatum());
        Assert.Equal(3, aA.FirstPower());
        Assert.Same(aD1, aA.NextLocation().FirstDatum());
        Assert.True(aA.NextLocation().NextLocation().IsIdentity);
    }

    [Fact]
    public void Multiplication_cancels_inverse_datums()
    {
        var aL = new Placement(Rotation(1.1)) * new Placement(Translation(-2));
        Assert.True((aL * aL.Inverted()).IsIdentity);
        Assert.True((aL.Inverted() * aL).IsIdentity);
        Assert.True((aL / aL).IsIdentity);
        Assert.True(aL.Predivided(aL).IsIdentity);
        Assert.True(aL.Powered(0).IsIdentity);
        Assert.Equal(aL, aL.Powered(1));
        Assert.Equal(aL * aL, aL.Powered(2));
        Assert.Equal(aL.Inverted() * aL.Inverted(), aL.Powered(-2));
        Assert.Equal(new Placement(), aL * new Placement(aL.FirstDatum()).Powered(-aL.FirstPower()) * aL.NextLocation().Inverted());
    }

    [Fact]
    public void Transformations_compose()
    {
        Transform3D aT1 = Rotation(0.4), aT2 = Translation(3);
        var aL1 = new Placement(aT1);
        var aL2 = new Placement(aT2);

        AssertSameTrsf(aT1.Multiplied(aT2), (aL1 * aL2).Transformation());
        AssertSameTrsf(aT1.Multiplied(aT2.Inverted()), (aL1 / aL2).Transformation());
        AssertSameTrsf(aT2.Inverted().Multiplied(aT1), aL1.Predivided(aL2).Transformation());
        AssertSameTrsf(aT1.Powered(3), aL1.Powered(3).Transformation());
        AssertSameTrsf(aT1.Multiplied(aT2).Powered(-2), (aL1 * aL2).Powered(-2).Transformation(), 1e-11);
        AssertSameTrsf(aT1.Inverted(), aL1.Inverted().Transformation());
        Assert.Equal(aL1.Transformation(), (Transform3D)aL1);
    }

    [Fact]
    public void Locations_work_as_map_keys()
    {
        var aL1 = new Placement(Rotation(0.2));
        var aL2 = new Placement(Rotation(0.2));
        var aMap = new NCollection_Map<Placement>();
        Assert.True(aMap.Add(aL1));
        Assert.False(aMap.Add(aL1 * aL2 * aL2.Inverted()));
        Assert.True(aMap.Add(aL2));
        Assert.True(aMap.Add(new Placement()));
        Assert.False(aMap.Add(default));
        Assert.Equal(3, aMap.Extent());

        var aSet = new HashSet<Placement> { aL1, aL1.Powered(1), aL2 };
        Assert.Equal(2, aSet.Count);
    }

    [Fact]
    public void ShallowDump_lists_the_powers()
    {
        var aL = new Placement(Rotation(0.5)).Powered(2);
        var aWriter = new StringWriter();
        aL.ShallowDump(aWriter);
        Assert.Contains("Exponent : 2", aWriter.ToString(), StringComparison.Ordinal);

        aWriter = new StringWriter();
        new Placement().ShallowDump(aWriter);
        Assert.StartsWith("TopLoc_Location : Identity", aWriter.ToString(), StringComparison.Ordinal);
    }

    [Fact]
    public void SList_shares_its_nodes()
    {
        var aD = new Datum3D(Translation(1));
        var anItem = new ItemLocation(aD, 2);
        var anEmpty = new SListOfItemLocation();
        var aList = new SListOfItemLocation(anItem, anEmpty);
        var aLonger = new SListOfItemLocation(new ItemLocation(aD, 1), aList);

        Assert.True(anEmpty.IsEmpty);
        Assert.True(anEmpty.Tail().IsEmpty);
        Assert.Equal(aList, aLonger.Tail());
        Assert.Equal(2, aLonger.Tail().Value()._power);
        // Each node holds the product of its own transformation and its tail's
        AssertSameTrsf(Translation(1).Powered(3), aLonger.Value()._trsf);
        Assert.ThrowsAny<NoSuchObjectException>(() => anEmpty.Value());
    }
}
