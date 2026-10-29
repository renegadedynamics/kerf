namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>IntTools_MarkedRangeSet, against OCCT's semantics traced by hand.</summary>
public class IntTools_MarkedRangeSetTests
{
    // The ranges and flags of the set, as (first, last, flag).
    private static (double, double, int)[] Content(IntTools_MarkedRangeSet theSet)
    {
        var aResult = new (double, double, int)[theSet.Length()];
        for (int i = 1; i <= theSet.Length(); ++i)
        {
            IntTools_Range aRange = theSet.Range(i);
            aResult[i - 1] = (aRange.First(), aRange.Last(), theSet.Flag(i));
        }
        return aResult;
    }

    [Fact]
    public void Constructors()
    {
        Assert.Equal(0, new IntTools_MarkedRangeSet().Length());

        IntTools_MarkedRangeSet aSet = new(0.0, 10.0, 3);
        Assert.Equal(new[] { (0.0, 10.0, 3) }, Content(aSet));

        NCollection_Array1<double> aValues = NCollection_Array1<double>.From([0.0, 1.0, 4.0, 9.0]);
        IntTools_MarkedRangeSet aSorted = new(aValues, 7);
        Assert.Equal(new[] { (0.0, 1.0, 7), (1.0, 4.0, 7), (4.0, 9.0, 7) }, Content(aSorted));
    }

    [Fact]
    public void InsertRange_InsideOneRange()
    {
        IntTools_MarkedRangeSet aSet = new(0.0, 10.0, 0);
        Assert.True(aSet.InsertRange(2.0, 5.0, 1));
        Assert.Equal(new[] { (0.0, 2.0, 0), (2.0, 5.0, 1), (5.0, 10.0, 0) }, Content(aSet));
    }

    [Fact]
    public void InsertRange_AcrossRanges()
    {
        IntTools_MarkedRangeSet aSet = new(0.0, 10.0, 0);
        aSet.InsertRange(new IntTools_Range(2.0, 5.0), 1);
        Assert.True(aSet.InsertRange(4.0, 7.0, 2));
        Assert.Equal(new[] { (0.0, 2.0, 0), (2.0, 4.0, 1), (4.0, 5.0, 2), (5.0, 7.0, 2), (7.0, 10.0, 0) }, Content(aSet));
    }

    [Fact]
    public void InsertRange_OutsideOrReversedFails()
    {
        IntTools_MarkedRangeSet aSet = new(0.0, 10.0, 0);
        Assert.False(aSet.InsertRange(11.0, 12.0, 1));
        Assert.False(aSet.InsertRange(-2.0, 1.0, 1));
        aSet.InsertRange(2.0, 5.0, 1);
        Assert.False(aSet.InsertRange(3.0, 1.0, 1)); // reversed, across ranges
        Assert.Equal(3, aSet.Length());
    }

    [Fact]
    public void InsertRange_AtIndex()
    {
        IntTools_MarkedRangeSet aSet = new(0.0, 10.0, 0);
        Assert.True(aSet.InsertRange(2.0, 5.0, 1, 1));
        Assert.Equal(new[] { (0.0, 2.0, 0), (2.0, 5.0, 1), (5.0, 10.0, 0) }, Content(aSet));

        // a range starting on the range's lower boundary only re-flags it before splitting
        IntTools_MarkedRangeSet aSet2 = new(0.0, 10.0, 0);
        Assert.True(aSet2.InsertRange(new IntTools_Range(0.0, 5.0), 1, 1));
        Assert.Equal(new[] { (0.0, 5.0, 1), (5.0, 10.0, 0) }, Content(aSet2));

        // a bad index fails; a range not inside the indexed one is inserted as without index
        Assert.False(aSet2.InsertRange(1.0, 2.0, 3, 0));
        Assert.False(aSet2.InsertRange(1.0, 2.0, 3, 3));
        Assert.True(aSet2.InsertRange(4.0, 6.0, 3, 1));
        Assert.Equal(new[] { (0.0, 4.0, 1), (4.0, 5.0, 3), (5.0, 6.0, 3), (6.0, 10.0, 0) }, Content(aSet2));
    }

    [Fact]
    public void GetIndexAndIndices()
    {
        IntTools_MarkedRangeSet aSet = new(0.0, 10.0, 0);
        aSet.InsertRange(2.0, 5.0, 1);
        aSet.InsertRange(4.0, 7.0, 2);
        // boundaries 0, 2, 4, 5, 7, 10

        Assert.Equal(0, aSet.GetIndex(-1.0));
        Assert.Equal(0, aSet.GetIndex(11.0));
        Assert.Equal(1, aSet.GetIndex(0.0));
        Assert.Equal(3, aSet.GetIndex(5.0)); // upper boundary included
        Assert.Equal(4, aSet.GetIndex(5.0, true)); // lower boundary included
        Assert.Equal(3, aSet.GetIndex(5.0, false));
        Assert.Equal(0, aSet.GetIndex(0.0, false));
        Assert.Equal(1, aSet.GetIndex(0.0, true));
        Assert.Equal(0, aSet.GetIndex(10.0, true));
        Assert.Equal(5, aSet.GetIndex(10.0, false));

        Assert.Equal(new[] { 3, 4 }, aSet.GetIndices(5.0));
        Assert.Equal(new[] { 2 }, aSet.GetIndices(3.0));
        Assert.Empty(aSet.GetIndices(-1.0));
        Assert.Equal(new[] { 5 }, aSet.GetIndices(10.0));
    }

    [Fact]
    public void SetFlagAndCopy()
    {
        IntTools_MarkedRangeSet aSet = new(0.0, 10.0, 0);
        aSet.InsertRange(2.0, 5.0, 1);
        IntTools_MarkedRangeSet aCopy = new(aSet);
        aCopy.SetFlag(2, 9);
        aCopy.InsertRange(6.0, 8.0, 4);
        Assert.Equal(1, aSet.Flag(2));
        Assert.Equal(3, aSet.Length());
        Assert.Equal(9, aCopy.Flag(2));
        Assert.Equal(5, aCopy.Length());
    }
}
