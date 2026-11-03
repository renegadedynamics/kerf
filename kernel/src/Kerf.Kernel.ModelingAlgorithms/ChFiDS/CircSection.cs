// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_CircSection.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_CircSection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Fillets.Data;

/// <summary>A section of a fillet: a circle or a line, with its parameter range.</summary>
/// <remarks>A C++ value class, so a struct. <c>new()</c> and <c>default</c> differ only in the default circle and line.</remarks>
public partial struct CircSection : IEquatable<CircSection>
{
    private Circle3D _circ;
    private Line3D _lin;
    private double _f;
    private double _l;

    public CircSection()
    {
        _circ = new Circle3D();
        _lin = new Line3D();
        _f = 0.0;
        _l = 0.0;
    }

    internal void Set(in Circle3D c, double f, double l)
    {
        _circ = c;
        _f = f;
        _l = l;
    }

    internal void Set(in Line3D c, double f, double l)
    {
        _lin = c;
        _f = f;
        _l = l;
    }

    /// <summary>A copy with the circle <paramref name="c"/> on [<paramref name="f"/>, <paramref name="l"/>].</summary>
    public readonly CircSection WithCircle(in Circle3D c, double f, double l)
    {
        CircSection copy = this;
        copy.Set(c, f, l);
        return copy;
    }

    /// <summary>A copy with the line <paramref name="c"/> on [<paramref name="f"/>, <paramref name="l"/>].</summary>
    public readonly CircSection WithLine(in Line3D c, double f, double l)
    {
        CircSection copy = this;
        copy.Set(c, f, l);
        return copy;
    }

    public readonly void Get(out Circle3D c, out double f, out double l)
    {
        c = _circ;
        f = _f;
        l = _l;
    }

    public readonly void Get(out Line3D c, out double f, out double l)
    {
        c = _lin;
        f = _f;
        l = _l;
    }

    public readonly bool Equals(CircSection other)
        => _circ.Equals(other._circ) && _lin.Equals(other._lin) && _f.Equals(other._f) && _l.Equals(other._l);

    public override readonly bool Equals(object? obj) => obj is CircSection other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_circ, _lin, _f, _l);

    public static bool operator ==(in CircSection left, in CircSection right) => left.Equals(right);

    public static bool operator !=(in CircSection left, in CircSection right) => !left.Equals(right);

    public override readonly string ToString() => Constants.Format("ChFiDS_CircSection", _f, _l);
}
