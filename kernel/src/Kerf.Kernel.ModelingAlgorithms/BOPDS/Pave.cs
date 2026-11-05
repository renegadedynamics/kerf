// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_Pave.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_Pave.lxx
// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_Pave.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation.Std;
using Kerf.Engine.Geometry;
using Kerf.Engine;
using System.Globalization;

namespace Kerf.Engine.Booleans.Data;

/// <summary>A vertex on an edge: the vertex's index in the data structure and its parameter on the edge.</summary>
/// <remarks>
/// <para>
/// OCCT's two comparisons are kept as OCCT defines them, and they are not consistent with each other:
/// <see cref="IsLess"/> (C++'s <c>operator&lt;</c>) compares parameters only, <see cref="IsEqual"/> (C++'s
/// <c>operator==</c>, and <see cref="Equals(Pave)"/> here) compares index and parameter. Sorts of paves use
/// <see cref="LessThan"/> with <see cref="Sorting"/>, so that paves with equal parameters stay in MSVC's order.
/// </para>
/// <para>The default pave has the index -1 and the parameter 99, as OCCT's.</para>
/// </remarks>
internal partial struct Pave : IEquatable<Pave>
{
    private int _index;
    private double _parameter;

    /// <summary>The pave (-1, 99).</summary>
    public Pave()
    {
        _index = -1;
        _parameter = 99.0;
    }

    /// <summary>The pave of the vertex <paramref name="index"/> at <paramref name="parameter"/>.</summary>
    public Pave(int index, double parameter)
    {
        _index = index;
        _parameter = parameter;
    }

    internal void SetIndex(int index) => _index = index;

    /// <summary>The index of the vertex.</summary>
    public readonly int Index() => _index;

    internal void SetParameter(double parameter) => _parameter = parameter;

    /// <summary>The parameter of the vertex on the edge.</summary>
    public readonly double Parameter() => _parameter;

    /// <summary>The index and the parameter.</summary>
    public readonly void Contents(out int index, out double parameter)
    {
        index = _index;
        parameter = _parameter;
    }

    /// <summary>True if this pave's parameter is less than the other's (C++'s <c>operator&lt;</c>).</summary>
    public readonly bool IsLess(in Pave other) => _parameter < other._parameter;

    /// <summary>True if the index and the parameter are equal (C++'s <c>operator==</c>).</summary>
    public readonly bool IsEqual(in Pave other) => _index == other._index && _parameter == other._parameter;

    /// <summary>Writes the pave as OCCT's <c>Dump</c> prints it: <c> { index parameter}</c>.</summary>
    public readonly void Dump(TextWriter os)
    {
        os.Write(string.Format(CultureInfo.InvariantCulture, " {{ {0} {1,-7:F3}}}", _index, _parameter));
    }

    // ---- Kerf additions

    /// <summary>C++'s <c>operator&lt;</c> for <see cref="Sorting"/>.</summary>
    public readonly struct LessThan : Heap.ILess<Pave>
    {
        public bool Less(Pave left, Pave right) => left.IsLess(right);
    }

    public readonly Pave WithIndex(int index)
    {
        Pave copy = this;
        copy.SetIndex(index);
        return copy;
    }

    public readonly Pave WithParameter(double parameter)
    {
        Pave copy = this;
        copy.SetParameter(parameter);
        return copy;
    }

    public readonly bool Equals(Pave other) => IsEqual(other);

    public override readonly bool Equals(object? obj) => obj is Pave other && IsEqual(other);

    public override readonly int GetHashCode() => HashCode.Combine(_index, _parameter == 0.0 ? 0.0 : _parameter);

    public static bool operator ==(Pave left, Pave right) => left.IsEqual(right);

    public static bool operator !=(Pave left, Pave right) => !left.IsEqual(right);

    public override readonly string ToString() => Constants.Format("Pave", _index, _parameter);
}
