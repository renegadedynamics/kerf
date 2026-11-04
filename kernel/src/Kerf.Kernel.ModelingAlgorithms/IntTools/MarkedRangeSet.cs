// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_MarkedRangeSet.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_MarkedRangeSet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Intersection.Tools;

/// <summary>A continuous set of ranges, each marked with a flag. Ranges are indexed from 1 to <see cref="Length"/>.</summary>
/// <remarks>
/// A class, a working object of the edge/face intersector (plan section 6), because it holds sequences. OCCT copies
/// it only as a member of a working object; the copy constructor is written out for ported code.
/// </remarks>
internal sealed class MarkedRangeSet
{
    private readonly NCollection_Sequence<double> _rangeSetStorer = new();
    private int _rangeNumber;
    private readonly NCollection_Sequence<int> _flags = new();
    private readonly NCollection_Sequence<int> _foundIndices = new();

    /// <summary>An empty set.</summary>
    public MarkedRangeSet()
    {
        _rangeNumber = 0;
    }

    /// <summary>One range [<paramref name="firstBoundary"/>, <paramref name="lastBoundary"/>] with the flag.</summary>
    public MarkedRangeSet(double firstBoundary, double lastBoundary, int initFlag)
    {
        SetBoundaries(firstBoundary, lastBoundary, initFlag);
    }

    /// <summary>
    /// The ranges between consecutive values of the sorted array, all with the flag. The order is not checked.
    /// </summary>
    public MarkedRangeSet(NCollection_Array1<double> sortedArray, int initFlag)
    {
        SetRanges(sortedArray, initFlag);
    }

    /// <summary>A copy of <paramref name="other"/> (OCCT's implicit copy constructor).</summary>
    public MarkedRangeSet(MarkedRangeSet other)
    {
        _rangeSetStorer.Assign(other._rangeSetStorer);
        _rangeNumber = other._rangeNumber;
        _flags.Assign(other._flags);
        _foundIndices.Assign(other._foundIndices);
    }

    /// <summary>Makes the set one range with the flag.</summary>
    internal void SetBoundaries(double firstBoundary, double lastBoundary, int initFlag)
    {
        _rangeSetStorer.Clear();
        _rangeSetStorer.Append(firstBoundary);
        _rangeSetStorer.Append(lastBoundary);
        _rangeNumber = 1;
        _flags.Clear();
        _flags.Append(initFlag);
    }

    /// <summary>Makes the set the ranges between the sorted values (not checked), all with the flag.</summary>
    internal void SetRanges(NCollection_Array1<double> sortedArray, int initFlag)
    {
        _rangeSetStorer.Clear();
        _flags.Clear();
        for (int idx = sortedArray.Lower(); idx <= sortedArray.Upper(); ++idx)
        {
            _rangeSetStorer.Append(sortedArray.Value(idx));
        }
        _rangeNumber = _rangeSetStorer.Length() - 1;

        for (int i = 1; i <= _rangeNumber; i++)
        {
            _flags.Append(initFlag);
        }
    }

    /// <summary>
    /// Inserts a range marked with the flag, replacing the ranges or parts of ranges it covers. False if the range is
    /// outside the set's boundaries or reversed.
    /// </summary>
    internal bool InsertRange(double firstBoundary, double lastBoundary, int flag)
    {
        int index1 = GetIndex(firstBoundary, true);

        if (index1 == 0)
        {
            return false;
        }

        int index2 = GetIndex(lastBoundary, false);

        if (index2 == 0)
        {
            return false;
        }

        if (index2 < index1)
        {
            // it can be if theLastBoundary==theFirstBoundary==boundary_of_a_range or theFirstBoundary >
            // theLastBoundary
            int atmpindex = index1;
            index1 = index2;
            index2 = atmpindex;

            if (lastBoundary < firstBoundary)
            {
                return false;
            }
        }

        bool areEqualIndices = index1 == index2;
        int prevFlag = _flags.Value(index1);

        _rangeSetStorer.InsertAfter(index1, firstBoundary);
        index2++;
        _flags.InsertAfter(index1, flag);
        _rangeNumber = _rangeSetStorer.Length() - 1;

        _rangeSetStorer.InsertAfter(index2, lastBoundary);

        if (areEqualIndices)
        {
            _flags.InsertAfter(index2, prevFlag);
        }
        else
        {
            _flags.InsertBefore(index2, flag);
        }

        if (!areEqualIndices)
        {
            index1++;
            index2++;

            for (int i = index1; i < index2; i++)
            {
                _flags.SetValue(i, flag);
            }
        }

        _rangeNumber = _rangeSetStorer.Length() - 1;

        return true;
    }

    /// <inheritdoc cref="InsertRange(double, double, int)"/>
    internal bool InsertRange(in ParameterRange range, int flag) =>
        InsertRange(range.First(), range.Last(), flag);

    /// <summary>
    /// Inserts a range marked with the flag into the range with the index <paramref name="index"/>; a range not
    /// inside that one, or of zero length, is inserted as by <see cref="InsertRange(double, double, int)"/>. False if
    /// the index is out of range.
    /// </summary>
    internal bool InsertRange(double firstBoundary, double lastBoundary, int flag, int index)
    {
        double tolerance = 1.0e-15;
        int anIndex = index;

        if ((index <= 0) || (index > _rangeNumber))
        {
            return false;
        }

        if ((firstBoundary < _rangeSetStorer.Value(index)) || (lastBoundary > _rangeSetStorer.Value(index + 1))
            || (Math.Abs(firstBoundary - lastBoundary) < tolerance))
        {
            return InsertRange(firstBoundary, lastBoundary, flag);
        }
        else
        {
            int prevFlag = _flags.Value(anIndex);

            if ((Math.Abs(firstBoundary - _rangeSetStorer.Value(anIndex)) > tolerance)
                && (Math.Abs(firstBoundary - _rangeSetStorer.Value(anIndex + 1)) > tolerance))
            {
                _rangeSetStorer.InsertAfter(anIndex, firstBoundary);
                _flags.InsertAfter(anIndex, flag);
                anIndex++;
                _rangeNumber = _rangeSetStorer.Length() - 1;
            }
            else
            {
                _flags.SetValue(anIndex, flag);
            }

            if ((Math.Abs(lastBoundary - _rangeSetStorer.Value(anIndex)) > tolerance)
                && (Math.Abs(lastBoundary - _rangeSetStorer.Value(anIndex + 1)) > tolerance))
            {
                _rangeSetStorer.InsertAfter(anIndex, lastBoundary);
                _rangeNumber = _rangeSetStorer.Length() - 1;
                _flags.InsertAfter(anIndex, prevFlag);
            }
        }

        return true;
    }

    /// <inheritdoc cref="InsertRange(double, double, int, int)"/>
    internal bool InsertRange(in ParameterRange range, int flag, int index) =>
        InsertRange(range.First(), range.Last(), flag, index);

    /// <summary>Sets the flag of the range with the index.</summary>
    internal void SetFlag(int index, int flag) => _flags.SetValue(index, flag);

    /// <summary>The flag of the range with the index.</summary>
    public int Flag(int index) => _flags.Value(index);

    /// <summary>The index of the range containing the value (upper bounds included), or 0.</summary>
    public int GetIndex(double value)
    {
        int index = 0;

        if (value < _rangeSetStorer.Value(1))
        {
            index = 0;
        }
        else
        {
            for (int i = 2; i <= _rangeSetStorer.Length(); i++)
            {
                if (value <= _rangeSetStorer.Value(i))
                {
                    index = i - 1;
                    break;
                }
            }
        }

        return index;
    }

    /// <summary>
    /// The indices of the ranges containing the value: two where the value is a common boundary. The set's own
    /// sequence, overwritten by the next call (OCCT returns a reference to its member).
    /// </summary>
    internal NCollection_Sequence<int> GetIndices(double value)
    {
        _foundIndices.Clear();

        if (value < _rangeSetStorer.Value(1))
        {
            return _foundIndices;
        }
        else
        {
            bool found = false;

            for (int i = 2; i <= _rangeSetStorer.Length(); i++)
            {
                if (found)
                {
                    if (value >= _rangeSetStorer.Value(i - 1))
                    {
                        _foundIndices.Append(i - 1);
                    }
                    else
                    {
                        break;
                    }
                }
                else
                {
                    if (value <= _rangeSetStorer.Value(i))
                    {
                        _foundIndices.Append(i - 1);
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
        return _foundIndices;
    }

    /// <summary>
    /// The index of the range containing the value, or 0. With <paramref name="useLower"/> a range's lower boundary may
    /// equal the value, otherwise its upper boundary may.
    /// </summary>
    public int GetIndex(double value, bool useLower)
    {
        int index = 0;

        if ((useLower && (value < _rangeSetStorer.Value(1))) || (!useLower && (value <= _rangeSetStorer.Value(1))))
        {
            index = 0;
        }
        else
        {
            for (int i = 2; i <= _rangeSetStorer.Length(); i++)
            {
                if ((useLower && value < _rangeSetStorer.Value(i)) || (!useLower && value <= _rangeSetStorer.Value(i)))
                {
                    index = i - 1;
                    break;
                }
            }
        }

        return index;
    }

    /// <summary>The number of ranges.</summary>
    public int Length() => _rangeNumber;

    /// <summary>The range with the index, from 1 to <see cref="Length"/>.</summary>
    public ParameterRange Range(int index)
    {
        ParameterRange range = new(_rangeSetStorer.Value(index), _rangeSetStorer.Value(index + 1));
        return range;
    }
}
