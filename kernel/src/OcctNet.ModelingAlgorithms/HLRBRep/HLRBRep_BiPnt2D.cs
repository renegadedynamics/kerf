// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_BiPnt2D.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Kerf.Topology;

namespace Occt;

/// <summary>A 2D segment of a polygonal hidden-line result, with the shape it comes from and its kind (smooth, sewn, outline, internal).</summary>
/// <remarks>A C++ value class, so a struct. The setters are internal, with <c>With</c> twins.</remarks>
public partial struct HLRBRep_BiPnt2D : IEquatable<HLRBRep_BiPnt2D>
{
    private Point2D myP1;
    private Point2D myP2;
    private Shape myShape;
    private bool myRg1Line;
    private bool myRgNLine;
    private bool myOutLine;
    private bool myIntLine;

    public HLRBRep_BiPnt2D()
    {
        myP1 = new Point2D();
        myP2 = new Point2D();
        myShape = new Shape();
        myRg1Line = false;
        myRgNLine = false;
        myOutLine = false;
        myIntLine = false;
    }

    public HLRBRep_BiPnt2D(double x1, double y1, double x2, double y2, in Shape S, bool reg1, bool regn, bool outl, bool intl)
    {
        myP1 = new Point2D(x1, y1);
        myP2 = new Point2D(x2, y2);
        myShape = S;
        myRg1Line = reg1;
        myRgNLine = regn;
        myOutLine = outl;
        myIntLine = intl;
    }

    public HLRBRep_BiPnt2D(in Xy thePoint1, in Xy thePoint2, in Shape S, bool reg1, bool regn, bool outl, bool intl)
    {
        myP1 = new Point2D(thePoint1);
        myP2 = new Point2D(thePoint2);
        myShape = S;
        myRg1Line = reg1;
        myRgNLine = regn;
        myOutLine = outl;
        myIntLine = intl;
    }

    public readonly Point2D P1() => myP1;

    public readonly Point2D P2() => myP2;

    public readonly Shape Shape() => myShape;

    internal void Shape(in Shape S) => myShape = S;

    public readonly bool Rg1Line() => myRg1Line;

    internal void Rg1Line(bool B) => myRg1Line = B;

    public readonly bool RgNLine() => myRgNLine;

    internal void RgNLine(bool B) => myRgNLine = B;

    public readonly bool OutLine() => myOutLine;

    internal void OutLine(bool B) => myOutLine = B;

    public readonly bool IntLine() => myIntLine;

    internal void IntLine(bool B) => myIntLine = B;

    public readonly HLRBRep_BiPnt2D WithShape(in Shape S)
    {
        HLRBRep_BiPnt2D aCopy = this;
        aCopy.Shape(S);
        return aCopy;
    }

    public readonly HLRBRep_BiPnt2D WithRg1Line(bool B)
    {
        HLRBRep_BiPnt2D aCopy = this;
        aCopy.Rg1Line(B);
        return aCopy;
    }

    public readonly HLRBRep_BiPnt2D WithRgNLine(bool B)
    {
        HLRBRep_BiPnt2D aCopy = this;
        aCopy.RgNLine(B);
        return aCopy;
    }

    public readonly HLRBRep_BiPnt2D WithOutLine(bool B)
    {
        HLRBRep_BiPnt2D aCopy = this;
        aCopy.OutLine(B);
        return aCopy;
    }

    public readonly HLRBRep_BiPnt2D WithIntLine(bool B)
    {
        HLRBRep_BiPnt2D aCopy = this;
        aCopy.IntLine(B);
        return aCopy;
    }

    public readonly bool Equals(HLRBRep_BiPnt2D other)
        => myP1 == other.myP1 && myP2 == other.myP2 && myShape.Equals(other.myShape) && myRg1Line == other.myRg1Line
           && myRgNLine == other.myRgNLine && myOutLine == other.myOutLine && myIntLine == other.myIntLine;

    public override readonly bool Equals(object? obj) => obj is HLRBRep_BiPnt2D other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(myP1, myP2, myShape, myRg1Line, myRgNLine, myOutLine, myIntLine);

    public static bool operator ==(in HLRBRep_BiPnt2D theLeft, in HLRBRep_BiPnt2D theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in HLRBRep_BiPnt2D theLeft, in HLRBRep_BiPnt2D theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => $"HLRBRep_BiPnt2D({myP1}, {myP2})";
}
