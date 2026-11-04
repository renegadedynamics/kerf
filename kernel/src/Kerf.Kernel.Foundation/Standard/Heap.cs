// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Foundation.Std;

/// <summary>
/// C++'s heap algorithms (<c>std::make_heap</c>, <c>std::push_heap</c>, <c>std::pop_heap</c>, <c>std::sort_heap</c>) as
/// the Microsoft C++ standard library implements them, so that elements with equal keys end in MSVC's order. OCCT's
/// reference builds (and the oracle) use MSVC; <c>Array.Sort</c> and other libraries order ties differently.
/// </summary>
/// <remarks>
/// Written from MSVC's <c>&lt;__msvc_heap_algorithms.hpp&gt;</c> (Visual Studio 2022, toolset 14.44:
/// <c>PushHeapByIndex</c>, <c>PopHeapHoleByIndex</c>, <c>_Pop_heap_hole_unchecked</c>,
/// <c>PopHeapUnchecked</c>, <c>_Make_heap_unchecked</c>) and <c>&lt;algorithm&gt;</c> (<c>_Sort_heap_unchecked</c>):
/// the same holes, the same comparisons in the same order, the same moves. The predicate is C++'s "less", called as
/// <c>_Pred(left, right)</c> exactly where MSVC calls it. Decision 491; the order is checked against orders recorded
/// from MSVC-compiled programs (tests).
/// </remarks>
internal static class Heap
{
    /// <summary>A strict weak ordering, C++'s <c>Compare</c> (<c>_Pred(a, b)</c> is "a is less than b").</summary>
    public interface ILess<in T>
    {
        bool Less(T left, T right);
    }

    /// <summary>Makes the range a max-heap (<c>std::make_heap</c>).</summary>
    public static void make_heap<T, TPred>(Span<T> range, TPred pred) where TPred : ILess<T>
    {
        int _Bottom = range.Length;
        for (int _Hole = _Bottom >> 1; _Hole > 0;)
        {
            // reheap top half, bottom to top
            --_Hole;
            T _Val = range[_Hole];
            PopHeapHoleByIndex(range, _Hole, _Bottom, _Val, pred);
        }
    }

    /// <summary>Sorts a heap into ascending order (<c>std::sort_heap</c>).</summary>
    public static void sort_heap<T, TPred>(Span<T> range, TPred pred) where TPred : ILess<T>
    {
        // order heap by repeatedly popping
        for (int _Last = range.Length; _Last >= 2; --_Last)
        {
            PopHeapUnchecked(range[.._Last], pred);
        }
    }

    /// <summary>Moves the largest element to the end and makes the rest a heap again (<c>std::pop_heap</c>).</summary>
    public static void pop_heap<T, TPred>(Span<T> range, TPred pred) where TPred : ILess<T>
        => PopHeapUnchecked(range, pred);

    /// <summary>Pushes the last element onto the heap before it (<c>std::push_heap</c>).</summary>
    public static void push_heap<T, TPred>(Span<T> range, TPred pred) where TPred : ILess<T>
    {
        int _Count = range.Length;
        if (2 <= _Count)
        {
            T _Val = range[--_Count];
            PushHeapByIndex(range, _Count, 0, _Val, pred);
        }
    }

    // percolate _Hole to _Top or where _Val belongs
    private static void PushHeapByIndex<T, TPred>(Span<T> _First, int _Hole, int _Top, T _Val, TPred _Pred)
        where TPred : ILess<T>
    {
        for (int _Idx = (_Hole - 1) >> 1; _Top < _Hole && _Pred.Less(_First[_Idx], _Val); _Idx = (_Hole - 1) >> 1)
        {
            // move _Hole up to parent
            _First[_Hole] = _First[_Idx];
            _Hole = _Idx;
        }

        _First[_Hole] = _Val; // drop _Val into final hole
    }

    // percolate _Hole to _Bottom, then push _Val
    private static void PopHeapHoleByIndex<T, TPred>(Span<T> _First, int _Hole, int _Bottom, T _Val, TPred _Pred)
        where TPred : ILess<T>
    {
        int _Top = _Hole;
        int _Idx = _Hole;

        // Check whether _Idx can have a child before calculating that child's index, since
        // calculating the child's index can trigger integer overflows
        int _Max_sequence_non_leaf = (_Bottom - 1) >> 1;
        while (_Idx < _Max_sequence_non_leaf)
        {
            // move _Hole down to larger child
            _Idx = 2 * _Idx + 2;
            if (_Pred.Less(_First[_Idx], _First[_Idx - 1]))
            {
                --_Idx;
            }

            _First[_Hole] = _First[_Idx];
            _Hole = _Idx;
        }

        if (_Idx == _Max_sequence_non_leaf && _Bottom % 2 == 0)
        {
            // only child at bottom, move _Hole down to it
            _First[_Hole] = _First[_Bottom - 1];
            _Hole = _Bottom - 1;
        }

        PushHeapByIndex(_First, _Hole, _Top, _Val, _Pred);
    }

    // pop *_First to *(_Last - 1) and reheap
    private static void PopHeapUnchecked<T, TPred>(Span<T> range, TPred _Pred) where TPred : ILess<T>
    {
        if (2 <= range.Length)
        {
            int _Last = range.Length - 1;
            T _Val = range[_Last];
            // _Pop_heap_hole_unchecked(_First, _Last, _Last, _Val): pop *_First to *_Dest and reheap
            range[_Last] = range[0];
            PopHeapHoleByIndex(range, 0, _Last, _Val, _Pred);
        }
    }
}
