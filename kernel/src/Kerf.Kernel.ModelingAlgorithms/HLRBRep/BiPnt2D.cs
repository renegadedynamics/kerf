// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_BiPnt2D.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.HiddenLines;

/// <summary>A 2D segment of a polygonal hidden-line result, with the shape it comes from and its kind (smooth, sewn, outline, internal).</summary>
/// <remarks>A C++ value class, so a struct. The setters are internal, with <c>With</c> twins.</remarks>
internal partial struct BiPnt2D : IEquatable<BiPnt2D>
{
    private Point2D _p1;
    private Point2D _p2;
    private Shape _shape;
    private bool _rg1Line;
    private bool _rgNLine;
    private bool _outLine;
    private bool _intLine;

    public BiPnt2D()
    {
        _p1 = new Point2D();
        _p2 = new Point2D();
        _shape = new Shape();
        _rg1Line = false;
        _rgNLine = false;
        _outLine = false;
        _intLine = false;
    }

    public BiPnt2D(double x1, double y1, double x2, double y2, in Shape s, bool reg1, bool regn, bool outl, bool intl)
    {
        _p1 = new Point2D(x1, y1);
        _p2 = new Point2D(x2, y2);
        _shape = s;
        _rg1Line = reg1;
        _rgNLine = regn;
        _outLine = outl;
        _intLine = intl;
    }

    public BiPnt2D(in Xy point1, in Xy point2, in Shape s, bool reg1, bool regn, bool outl, bool intl)
    {
        _p1 = new Point2D(point1);
        _p2 = new Point2D(point2);
        _shape = s;
        _rg1Line = reg1;
        _rgNLine = regn;
        _outLine = outl;
        _intLine = intl;
    }

    public readonly Point2D P1() => _p1;

    public readonly Point2D P2() => _p2;

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

    public readonly BiPnt2D WithShape(in Shape s)
    {
        BiPnt2D copy = this;
        copy.Shape(s);
        return copy;
    }

    public readonly BiPnt2D WithRg1Line(bool b)
    {
        BiPnt2D copy = this;
        copy.Rg1Line(b);
        return copy;
    }

    public readonly BiPnt2D WithRgNLine(bool b)
    {
        BiPnt2D copy = this;
        copy.RgNLine(b);
        return copy;
    }

    public readonly BiPnt2D WithOutLine(bool b)
    {
        BiPnt2D copy = this;
        copy.OutLine(b);
        return copy;
    }

    public readonly BiPnt2D WithIntLine(bool b)
    {
        BiPnt2D copy = this;
        copy.IntLine(b);
        return copy;
    }

    public readonly bool Equals(BiPnt2D other)
        => _p1 == other._p1 && _p2 == other._p2 && _shape.Equals(other._shape) && _rg1Line == other._rg1Line
           && _rgNLine == other._rgNLine && _outLine == other._outLine && _intLine == other._intLine;

    public override readonly bool Equals(object? obj) => obj is BiPnt2D other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_p1, _p2, _shape, _rg1Line, _rgNLine, _outLine, _intLine);

    public static bool operator ==(in BiPnt2D left, in BiPnt2D right) => left.Equals(right);

    public static bool operator !=(in BiPnt2D left, in BiPnt2D right) => !left.Equals(right);

    public override readonly string ToString() => $"HLRBRep_BiPnt2D({_p1}, {_p2})";
}
