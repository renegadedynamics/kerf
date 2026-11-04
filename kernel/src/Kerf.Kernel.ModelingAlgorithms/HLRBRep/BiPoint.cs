// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_BiPoint.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.HiddenLines;

/// <summary>A 3D segment of a polygonal hidden-line result, with the shape it comes from and its kind (smooth, sewn, outline, internal).</summary>
/// <remarks>A C++ value class, so a struct. The setters are internal, with <c>With</c> twins.</remarks>
internal partial struct BiPoint : IEquatable<BiPoint>
{
    private Point3D _p1;
    private Point3D _p2;
    private Shape _shape;
    private bool _rg1Line;
    private bool _rgNLine;
    private bool _outLine;
    private bool _intLine;

    public BiPoint()
    {
        _p1 = new Point3D();
        _p2 = new Point3D();
        _shape = new Shape();
        _rg1Line = false;
        _rgNLine = false;
        _outLine = false;
        _intLine = false;
    }

    public BiPoint(double x1, double y1, double z1, double x2, double y2, double z2, in Shape s, bool reg1, bool regn, bool outl, bool intl)
    {
        _p1 = new Point3D(x1, y1, z1);
        _p2 = new Point3D(x2, y2, z2);
        _shape = s;
        _rg1Line = reg1;
        _rgNLine = regn;
        _outLine = outl;
        _intLine = intl;
    }

    public readonly Point3D P1() => _p1;

    public readonly Point3D P2() => _p2;

    public readonly Shape Shape() => _shape;

    internal void Shape(in Shape s) => _shape = s;

    public readonly bool Rg1Line() => _rg1Line;

    internal void Rg1Line(bool b) => _rg1Line = b;

    public readonly bool RgNLine() => _rgNLine;

    internal void RgNLine(bool b) => _rgNLine = b;

    public readonly bool OutLine() => _outLine;

    internal void OutLine(bool b) => _outLine = b;

    public readonly bool IntLine() => _intLine;

    internal void IntLine(bool b) => _intLine = b;

    public readonly BiPoint WithShape(in Shape s)
    {
        BiPoint copy = this;
        copy.Shape(s);
        return copy;
    }

    public readonly BiPoint WithRg1Line(bool b)
    {
        BiPoint copy = this;
        copy.Rg1Line(b);
        return copy;
    }

    public readonly BiPoint WithRgNLine(bool b)
    {
        BiPoint copy = this;
        copy.RgNLine(b);
        return copy;
    }

    public readonly BiPoint WithOutLine(bool b)
    {
        BiPoint copy = this;
        copy.OutLine(b);
        return copy;
    }

    public readonly BiPoint WithIntLine(bool b)
    {
        BiPoint copy = this;
        copy.IntLine(b);
        return copy;
    }

    public readonly bool Equals(BiPoint other)
        => _p1 == other._p1 && _p2 == other._p2 && _shape.Equals(other._shape) && _rg1Line == other._rg1Line
           && _rgNLine == other._rgNLine && _outLine == other._outLine && _intLine == other._intLine;

    public override readonly bool Equals(object? obj) => obj is BiPoint other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_p1, _p2, _shape, _rg1Line, _rgNLine, _outLine, _intLine);

    public static bool operator ==(in BiPoint left, in BiPoint right) => left.Equals(right);

    public static bool operator !=(in BiPoint left, in BiPoint right) => !left.Equals(right);

    public override readonly string ToString() => $"HLRBRep_BiPoint({_p1}, {_p2})";
}
