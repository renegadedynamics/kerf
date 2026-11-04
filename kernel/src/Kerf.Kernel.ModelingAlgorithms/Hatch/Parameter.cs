// Port of: src/ModelingAlgorithms/TKGeomAlgo/Hatch/Hatch_Parameter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Hatch/Hatch_Parameter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Occt;

namespace Kerf.Engine.Classification.Hatching;

/// <summary>
/// An intersection on a line of the <see cref="Hatcher"/>: a parameter, and a flag true when the parameter
/// starts an interval; with the index of the trimming element and the parameter on it.
/// </summary>
/// <remarks>
/// A value type (decision 663). Its fields are internal, as OCCT's friends Hatch_Line and Hatch_Hatcher read them;
/// nothing writes them after construction. <c>default</c> equals OCCT's default constructor.
/// </remarks>
internal readonly struct Parameter : IEquatable<Parameter>
{
    internal readonly double _par1;
    internal readonly bool _start;
    internal readonly int _index;
    internal readonly double _par2;

    /// <summary>A parameter 0, not starting an interval.</summary>
    public Parameter()
    {
        _par1 = 0.0;
        _start = false;
        _index = 0;
        _par2 = 0.0;
    }

    /// <summary>The parameter <paramref name="par1"/> on the line, starting an interval if <paramref name="start"/>.</summary>
    public Parameter(double par1, bool start, int index = 0, double par2 = 0)
    {
        _par1 = par1;
        _start = start;
        _index = index;
        _par2 = par2;
    }

    public bool Equals(Parameter other) =>
        _par1.Equals(other._par1) && _start == other._start && _index == other._index
        && _par2.Equals(other._par2);

    public override bool Equals(object? obj) => obj is Parameter other && Equals(other);

    public override int GetHashCode() => HashCode.Combine(_par1, _start, _index, _par2);

    public static bool operator ==(Parameter left, Parameter right) => left.Equals(right);

    public static bool operator !=(Parameter left, Parameter right) => !left.Equals(right);

    public override string ToString() =>
        $"{Constants.Format("Hatch_Parameter", _par1, _par2)} start={_start} index={_index}";
}
