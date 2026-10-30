// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_BlockIterator.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_BlockIterator.lxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_BlockIterator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>An iterator on the elements of a block of a <see cref="TopOpeBRepBuild_BlockBuilder"/>: an index range.</summary>
/// <remarks>A value, as in OCCT. <c>new()</c> is OCCT's default (an empty range starting at 1); <c>default</c> starts at
/// 0. The iteration methods are internal.</remarks>
public partial struct TopOpeBRepBuild_BlockIterator : IEquatable<TopOpeBRepBuild_BlockIterator>
{
    private int myLower;
    private int myUpper;
    private int myValue;

    /// <summary>An empty iterator.</summary>
    public TopOpeBRepBuild_BlockIterator()
    {
        myLower = 0;
        myUpper = 0;
        myValue = 1;
    }

    /// <summary>An iterator on the indices <paramref name="Lower"/> to <paramref name="Upper"/>.</summary>
    public TopOpeBRepBuild_BlockIterator(int Lower, int Upper)
    {
        myLower = Lower;
        myUpper = Upper;
        myValue = Lower;
    }

    internal void Initialize()
    {
        myValue = myLower;
    }

    /// <summary>Whether the current index is in the range.</summary>
    public readonly bool More()
    {
        bool b = myValue <= myUpper;
        return b;
    }

    internal void Next()
    {
        myValue++;
    }

    /// <summary>The current index.</summary>
    public readonly int Value() => myValue;

    /// <summary>The number of indices (0 for an empty iterator, whose lower bound is 0).</summary>
    public readonly int Extent()
    {
        if (myLower != 0)
        {
            int n = myUpper - myLower + 1;
            return n;
        }
        return 0;
    }

    // OCCT.NET: value equality

    public readonly bool Equals(TopOpeBRepBuild_BlockIterator other)
        => myLower == other.myLower && myUpper == other.myUpper && myValue == other.myValue;

    public override readonly bool Equals(object? obj) => obj is TopOpeBRepBuild_BlockIterator anOther && Equals(anOther);

    public override readonly int GetHashCode() => HashCode.Combine(myLower, myUpper, myValue);

    public static bool operator ==(in TopOpeBRepBuild_BlockIterator theLeft, in TopOpeBRepBuild_BlockIterator theRight)
        => theLeft.Equals(theRight);

    public static bool operator !=(in TopOpeBRepBuild_BlockIterator theLeft, in TopOpeBRepBuild_BlockIterator theRight)
        => !theLeft.Equals(theRight);
}
