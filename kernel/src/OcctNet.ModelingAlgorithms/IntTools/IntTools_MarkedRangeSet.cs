// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_MarkedRangeSet.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_MarkedRangeSet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A continuous set of ranges, each marked with a flag. Ranges are indexed from 1 to <see cref="Length"/>.</summary>
/// <remarks>
/// A class, a working object of the edge/face intersector (plan section 6), because it holds sequences. OCCT copies
/// it only as a member of a working object; the copy constructor is written out for ported code.
/// </remarks>
public sealed class IntTools_MarkedRangeSet
{
    private readonly NCollection_Sequence<double> myRangeSetStorer = new();
    private int myRangeNumber;
    private readonly NCollection_Sequence<int> myFlags = new();
    private readonly NCollection_Sequence<int> myFoundIndices = new();

    /// <summary>An empty set.</summary>
    public IntTools_MarkedRangeSet()
    {
        myRangeNumber = 0;
    }

    /// <summary>One range [<paramref name="theFirstBoundary"/>, <paramref name="theLastBoundary"/>] with the flag.</summary>
    public IntTools_MarkedRangeSet(double theFirstBoundary, double theLastBoundary, int theInitFlag)
    {
        SetBoundaries(theFirstBoundary, theLastBoundary, theInitFlag);
    }

    /// <summary>
    /// The ranges between consecutive values of the sorted array, all with the flag. The order is not checked.
    /// </summary>
    public IntTools_MarkedRangeSet(NCollection_Array1<double> theSortedArray, int theInitFlag)
    {
        SetRanges(theSortedArray, theInitFlag);
    }

    /// <summary>A copy of <paramref name="theOther"/> (OCCT's implicit copy constructor).</summary>
    public IntTools_MarkedRangeSet(IntTools_MarkedRangeSet theOther)
    {
        myRangeSetStorer.Assign(theOther.myRangeSetStorer);
        myRangeNumber = theOther.myRangeNumber;
        myFlags.Assign(theOther.myFlags);
        myFoundIndices.Assign(theOther.myFoundIndices);
    }

    /// <summary>Makes the set one range with the flag.</summary>
    internal void SetBoundaries(double theFirstBoundary, double theLastBoundary, int theInitFlag)
    {
        myRangeSetStorer.Clear();
        myRangeSetStorer.Append(theFirstBoundary);
        myRangeSetStorer.Append(theLastBoundary);
        myRangeNumber = 1;
        myFlags.Clear();
        myFlags.Append(theInitFlag);
    }

    /// <summary>Makes the set the ranges between the sorted values (not checked), all with the flag.</summary>
    internal void SetRanges(NCollection_Array1<double> theSortedArray, int theInitFlag)
    {
        myRangeSetStorer.Clear();
        myFlags.Clear();
        for (int anIdx = theSortedArray.Lower(); anIdx <= theSortedArray.Upper(); ++anIdx)
        {
            myRangeSetStorer.Append(theSortedArray.Value(anIdx));
        }
        myRangeNumber = myRangeSetStorer.Length() - 1;

        for (int i = 1; i <= myRangeNumber; i++)
        {
            myFlags.Append(theInitFlag);
        }
    }

    /// <summary>
    /// Inserts a range marked with the flag, replacing the ranges or parts of ranges it covers. False if the range is
    /// outside the set's boundaries or reversed.
    /// </summary>
    internal bool InsertRange(double theFirstBoundary, double theLastBoundary, int theFlag)
    {
        int anIndex1 = GetIndex(theFirstBoundary, true);

        if (anIndex1 == 0)
        {
            return false;
        }

        int anIndex2 = GetIndex(theLastBoundary, false);

        if (anIndex2 == 0)
        {
            return false;
        }

        if (anIndex2 < anIndex1)
        {
            // it can be if theLastBoundary==theFirstBoundary==boundary_of_a_range or theFirstBoundary >
            // theLastBoundary
            int atmpindex = anIndex1;
            anIndex1 = anIndex2;
            anIndex2 = atmpindex;

            if (theLastBoundary < theFirstBoundary)
            {
                return false;
            }
        }

        bool areEqualIndices = anIndex1 == anIndex2;
        int aPrevFlag = myFlags.Value(anIndex1);

        myRangeSetStorer.InsertAfter(anIndex1, theFirstBoundary);
        anIndex2++;
        myFlags.InsertAfter(anIndex1, theFlag);
        myRangeNumber = myRangeSetStorer.Length() - 1;

        myRangeSetStorer.InsertAfter(anIndex2, theLastBoundary);

        if (areEqualIndices)
        {
            myFlags.InsertAfter(anIndex2, aPrevFlag);
        }
        else
        {
            myFlags.InsertBefore(anIndex2, theFlag);
        }

        if (!areEqualIndices)
        {
            anIndex1++;
            anIndex2++;

            for (int i = anIndex1; i < anIndex2; i++)
            {
                myFlags.SetValue(i, theFlag);
            }
        }

        myRangeNumber = myRangeSetStorer.Length() - 1;

        return true;
    }

    /// <inheritdoc cref="InsertRange(double, double, int)"/>
    internal bool InsertRange(in IntTools_Range theRange, int theFlag) =>
        InsertRange(theRange.First(), theRange.Last(), theFlag);

    /// <summary>
    /// Inserts a range marked with the flag into the range with the index <paramref name="theIndex"/>; a range not
    /// inside that one, or of zero length, is inserted as by <see cref="InsertRange(double, double, int)"/>. False if
    /// the index is out of range.
    /// </summary>
    internal bool InsertRange(double theFirstBoundary, double theLastBoundary, int theFlag, int theIndex)
    {
        double aTolerance = 1.0e-15;
        int anIndex = theIndex;

        if ((theIndex <= 0) || (theIndex > myRangeNumber))
        {
            return false;
        }

        if ((theFirstBoundary < myRangeSetStorer.Value(theIndex)) || (theLastBoundary > myRangeSetStorer.Value(theIndex + 1))
            || (Math.Abs(theFirstBoundary - theLastBoundary) < aTolerance))
        {
            return InsertRange(theFirstBoundary, theLastBoundary, theFlag);
        }
        else
        {
            int aPrevFlag = myFlags.Value(anIndex);

            if ((Math.Abs(theFirstBoundary - myRangeSetStorer.Value(anIndex)) > aTolerance)
                && (Math.Abs(theFirstBoundary - myRangeSetStorer.Value(anIndex + 1)) > aTolerance))
            {
                myRangeSetStorer.InsertAfter(anIndex, theFirstBoundary);
                myFlags.InsertAfter(anIndex, theFlag);
                anIndex++;
                myRangeNumber = myRangeSetStorer.Length() - 1;
            }
            else
            {
                myFlags.SetValue(anIndex, theFlag);
            }

            if ((Math.Abs(theLastBoundary - myRangeSetStorer.Value(anIndex)) > aTolerance)
                && (Math.Abs(theLastBoundary - myRangeSetStorer.Value(anIndex + 1)) > aTolerance))
            {
                myRangeSetStorer.InsertAfter(anIndex, theLastBoundary);
                myRangeNumber = myRangeSetStorer.Length() - 1;
                myFlags.InsertAfter(anIndex, aPrevFlag);
            }
        }

        return true;
    }

    /// <inheritdoc cref="InsertRange(double, double, int, int)"/>
    internal bool InsertRange(in IntTools_Range theRange, int theFlag, int theIndex) =>
        InsertRange(theRange.First(), theRange.Last(), theFlag, theIndex);

    /// <summary>Sets the flag of the range with the index.</summary>
    internal void SetFlag(int theIndex, int theFlag) => myFlags.SetValue(theIndex, theFlag);

    /// <summary>The flag of the range with the index.</summary>
    public int Flag(int theIndex) => myFlags.Value(theIndex);

    /// <summary>The index of the range containing the value (upper bounds included), or 0.</summary>
    public int GetIndex(double theValue)
    {
        int anIndex = 0;

        if (theValue < myRangeSetStorer.Value(1))
        {
            anIndex = 0;
        }
        else
        {
            for (int i = 2; i <= myRangeSetStorer.Length(); i++)
            {
                if (theValue <= myRangeSetStorer.Value(i))
                {
                    anIndex = i - 1;
                    break;
                }
            }
        }

        return anIndex;
    }

    /// <summary>
    /// The indices of the ranges containing the value: two where the value is a common boundary. The set's own
    /// sequence, overwritten by the next call (OCCT returns a reference to its member).
    /// </summary>
    internal NCollection_Sequence<int> GetIndices(double theValue)
    {
        myFoundIndices.Clear();

        if (theValue < myRangeSetStorer.Value(1))
        {
            return myFoundIndices;
        }
        else
        {
            bool found = false;

            for (int i = 2; i <= myRangeSetStorer.Length(); i++)
            {
                if (found)
                {
                    if (theValue >= myRangeSetStorer.Value(i - 1))
                    {
                        myFoundIndices.Append(i - 1);
                    }
                    else
                    {
                        break;
                    }
                }
                else
                {
                    if (theValue <= myRangeSetStorer.Value(i))
                    {
                        myFoundIndices.Append(i - 1);
                        found = true;
                    }
                    else
                    {
                        if (found)
                        {
                            break;
                        }
                    }
                }
            }
        }
        return myFoundIndices;
    }

    /// <summary>
    /// The index of the range containing the value, or 0. With <paramref name="UseLower"/> a range's lower boundary may
    /// equal the value, otherwise its upper boundary may.
    /// </summary>
    public int GetIndex(double theValue, bool UseLower)
    {
        int anIndex = 0;

        if ((UseLower && (theValue < myRangeSetStorer.Value(1))) || (!UseLower && (theValue <= myRangeSetStorer.Value(1))))
        {
            anIndex = 0;
        }
        else
        {
            for (int i = 2; i <= myRangeSetStorer.Length(); i++)
            {
                if ((UseLower && theValue < myRangeSetStorer.Value(i)) || (!UseLower && theValue <= myRangeSetStorer.Value(i)))
                {
                    anIndex = i - 1;
                    break;
                }
            }
        }

        return anIndex;
    }

    /// <summary>The number of ranges.</summary>
    public int Length() => myRangeNumber;

    /// <summary>The range with the index, from 1 to <see cref="Length"/>.</summary>
    public IntTools_Range Range(int theIndex)
    {
        IntTools_Range aRange = new(myRangeSetStorer.Value(theIndex), myRangeSetStorer.Value(theIndex + 1));
        return aRange;
    }
}
