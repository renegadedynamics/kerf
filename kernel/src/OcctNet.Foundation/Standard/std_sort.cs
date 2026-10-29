// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// C++'s <c>std::sort</c> and <c>std::stable_sort</c> with the element order on ties that the Microsoft C++ standard
/// library gives (decision 491; OCCT's reference builds and the oracle are MSVC builds). Used where OCCT sorts with a
/// comparison that can tie: IntTools' roots, BOPDS' paves (unit 6A).
/// </summary>
/// <remarks>
/// <para>
/// <see cref="sort{T, TPred}"/> follows MSVC's <c>_Sort_unchecked</c> of <c>&lt;algorithm&gt;</c>: an introsort that
/// sorts ranges of at most 32 elements (<c>_ISORT_MAX</c>) by insertion (<c>_Insertion_sort_unchecked</c>, which keeps
/// tied elements in their order), partitions larger ones around a median guess (<c>_Guess_median_unchecked</c>: the
/// median of three, Tukey's ninther above 40 elements; <c>_Partition_by_median_guess_unchecked</c>), and falls back
/// to <see cref="std_heap"/>'s heap sort after 1.5 log2(N) divisions. Ranges of up to 32 elements, the usual case
/// in the Boolean component, are MSVC's order by construction; the partition of larger ranges is written from MSVC's
/// source and is unverified against an MSVC-compiled program (decision 6110).
/// </para>
/// <para>
/// <see cref="stable_sort{T, TPred}"/> is any stable sort: a stable sort's order is unique, so it is MSVC's.
/// </para>
/// </remarks>
public static class std_sort
{
    private const int _ISORT_MAX = 32;

    /// <summary>Sorts the range by the predicate as MSVC's <c>std::sort</c> does.</summary>
    public static void sort<T, TPred>(Span<T> theRange, TPred thePred) where TPred : std_heap.ILess<T>
    {
        _Sort_unchecked(theRange, 0, theRange.Length, theRange.Length, thePred);
    }

    /// <summary>Sorts the range by the predicate, keeping tied elements in their order (<c>std::stable_sort</c>).</summary>
    public static void stable_sort<T, TPred>(Span<T> theRange, TPred thePred) where TPred : std_heap.ILess<T>
    {
        // A binary insertion sort for short ranges, a merge sort otherwise; both stable.
        if (theRange.Length <= _ISORT_MAX)
        {
            _Insertion_sort_unchecked(theRange, 0, theRange.Length, thePred);
            return;
        }
        T[] aBuffer = new T[theRange.Length];
        MergeSort(theRange, aBuffer, thePred);
    }

    private static void MergeSort<T, TPred>(Span<T> theRange, Span<T> theBuffer, TPred thePred) where TPred : std_heap.ILess<T>
    {
        int aNb = theRange.Length;
        if (aNb <= _ISORT_MAX)
        {
            _Insertion_sort_unchecked(theRange, 0, aNb, thePred);
            return;
        }
        int aMid = aNb >> 1;
        MergeSort(theRange[..aMid], theBuffer[..aMid], thePred);
        MergeSort(theRange[aMid..], theBuffer[aMid..], thePred);
        theRange.CopyTo(theBuffer);
        int i = 0, j = aMid, k = 0;
        while (i < aMid && j < aNb)
        {
            // take from the right only when strictly less: ties keep the left element first
            if (thePred.Less(theBuffer[j], theBuffer[i]))
            {
                theRange[k++] = theBuffer[j++];
            }
            else
            {
                theRange[k++] = theBuffer[i++];
            }
        }
        while (i < aMid)
        {
            theRange[k++] = theBuffer[i++];
        }
        while (j < aNb)
        {
            theRange[k++] = theBuffer[j++];
        }
    }

    private static void _Sort_unchecked<T, TPred>(Span<T> a, int _First, int _Last, int _Ideal, TPred _Pred)
        where TPred : std_heap.ILess<T>
    {
        // order [_First, _Last)
        for (;;)
        {
            if (_Last - _First <= _ISORT_MAX)
            {
                // small
                _Insertion_sort_unchecked(a, _First, _Last, _Pred);
                return;
            }

            if (_Ideal <= 0)
            {
                // heap sort if too many divisions
                std_heap.make_heap(a[_First.._Last], _Pred);
                std_heap.sort_heap(a[_First.._Last], _Pred);
                return;
            }

            // divide and conquer by quicksort
            (int _MidFirst, int _MidSecond) = _Partition_by_median_guess_unchecked(a, _First, _Last, _Pred);

            _Ideal = (_Ideal >> 1) + (_Ideal >> 2); // allow 1.5 log2(N) divisions

            if (_MidFirst - _First < _Last - _MidSecond)
            {
                // loop on second half
                _Sort_unchecked(a, _First, _MidFirst, _Ideal, _Pred);
                _First = _MidSecond;
            }
            else
            {
                // loop on first half
                _Sort_unchecked(a, _MidSecond, _Last, _Ideal, _Pred);
                _Last = _MidFirst;
            }
        }
    }

    private static void _Insertion_sort_unchecked<T, TPred>(Span<T> a, int _First, int _Last, TPred _Pred)
        where TPred : std_heap.ILess<T>
    {
        // insertion sort [_First, _Last)
        if (_First != _Last)
        {
            for (int _Mid = _First; ++_Mid != _Last;)
            {
                // order next element
                int _Hole = _Mid;
                T _Val = a[_Mid];

                if (_Pred.Less(_Val, a[_First]))
                {
                    // found new earliest element, move to front
                    a.Slice(_First, _Mid - _First).CopyTo(a.Slice(_First + 1, _Mid - _First));
                    a[_First] = _Val;
                }
                else
                {
                    // look for insertion point after first
                    for (int _Prev = _Hole; _Pred.Less(_Val, a[--_Prev]); _Hole = _Prev)
                    {
                        a[_Hole] = a[_Prev]; // move hole down
                    }

                    a[_Hole] = _Val; // insert element in hole
                }
            }
        }
    }

    private static void _Med3_unchecked<T, TPred>(Span<T> a, int _First, int _Mid, int _Last, TPred _Pred)
        where TPred : std_heap.ILess<T>
    {
        // sort median of three elements to middle
        if (_Pred.Less(a[_Mid], a[_First]))
        {
            (a[_Mid], a[_First]) = (a[_First], a[_Mid]);
        }

        if (_Pred.Less(a[_Last], a[_Mid]))
        {
            // swap middle and last, then test first again
            (a[_Last], a[_Mid]) = (a[_Mid], a[_Last]);

            if (_Pred.Less(a[_Mid], a[_First]))
            {
                (a[_Mid], a[_First]) = (a[_First], a[_Mid]);
            }
        }
    }

    private static void _Guess_median_unchecked<T, TPred>(Span<T> a, int _First, int _Mid, int _Last, TPred _Pred)
        where TPred : std_heap.ILess<T>
    {
        // sort median element to middle
        int _Count = _Last - _First;
        if (40 < _Count)
        {
            // Tukey's ninther
            int _Step = (_Count + 1) >> 3; // +1 can't overflow because range was made inclusive in caller
            int _Two_step = _Step << 1; // note: intentionally discards low-order bit
            _Med3_unchecked(a, _First, _First + _Step, _First + _Two_step, _Pred);
            _Med3_unchecked(a, _Mid - _Step, _Mid, _Mid + _Step, _Pred);
            _Med3_unchecked(a, _Last - _Two_step, _Last - _Step, _Last, _Pred);
            _Med3_unchecked(a, _First + _Step, _Mid, _Last - _Step, _Pred);
        }
        else
        {
            _Med3_unchecked(a, _First, _Mid, _Last, _Pred);
        }
    }

    private static (int, int) _Partition_by_median_guess_unchecked<T, TPred>(Span<T> a, int _First, int _Last, TPred _Pred)
        where TPred : std_heap.ILess<T>
    {
        // partition [_First, _Last)
        int _Mid = _First + ((_Last - _First) >> 1); // shift for codegen
        _Guess_median_unchecked(a, _First, _Mid, _Last - 1, _Pred);
        int _Pfirst = _Mid;
        int _Plast = _Pfirst + 1;

        while (_First < _Pfirst && !_Pred.Less(a[_Pfirst - 1], a[_Pfirst]) && !_Pred.Less(a[_Pfirst], a[_Pfirst - 1]))
        {
            --_Pfirst;
        }

        while (_Plast < _Last && !_Pred.Less(a[_Plast], a[_Pfirst]) && !_Pred.Less(a[_Pfirst], a[_Plast]))
        {
            ++_Plast;
        }

        int _Gfirst = _Plast;
        int _Glast = _Pfirst;

        for (;;)
        {
            // partition
            for (; _Gfirst < _Last; ++_Gfirst)
            {
                if (_Pred.Less(a[_Pfirst], a[_Gfirst]))
                {
                    continue;
                }
                else if (_Pred.Less(a[_Gfirst], a[_Pfirst]))
                {
                    break;
                }
                else if (_Plast != _Gfirst)
                {
                    (a[_Plast], a[_Gfirst]) = (a[_Gfirst], a[_Plast]);
                    ++_Plast;
                }
                else
                {
                    ++_Plast;
                }
            }

            for (; _First < _Glast; --_Glast)
            {
                if (_Pred.Less(a[_Glast - 1], a[_Pfirst]))
                {
                    continue;
                }
                else if (_Pred.Less(a[_Pfirst], a[_Glast - 1]))
                {
                    break;
                }
                else if (--_Pfirst != _Glast - 1)
                {
                    (a[_Pfirst], a[_Glast - 1]) = (a[_Glast - 1], a[_Pfirst]);
                }
            }

            if (_Glast == _First && _Gfirst == _Last)
            {
                return (_Pfirst, _Plast);
            }

            if (_Glast == _First)
            {
                // no room at bottom, rotate pivot upward
                if (_Plast != _Gfirst)
                {
                    (a[_Pfirst], a[_Plast]) = (a[_Plast], a[_Pfirst]);
                }

                ++_Plast;
                (a[_Pfirst], a[_Gfirst]) = (a[_Gfirst], a[_Pfirst]);
                ++_Pfirst;
                ++_Gfirst;
            }
            else if (_Gfirst == _Last)
            {
                // no room at top, rotate pivot downward
                if (--_Glast != --_Pfirst)
                {
                    (a[_Glast], a[_Pfirst]) = (a[_Pfirst], a[_Glast]);
                }

                --_Plast;
                (a[_Pfirst], a[_Plast]) = (a[_Plast], a[_Pfirst]);
            }
            else
            {
                --_Glast;
                (a[_Gfirst], a[_Glast]) = (a[_Glast], a[_Gfirst]);
                ++_Gfirst;
            }
        }
    }
}
