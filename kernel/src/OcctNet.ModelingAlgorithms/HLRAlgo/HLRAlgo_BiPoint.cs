// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_BiPoint.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_BiPoint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using System.Diagnostics.CodeAnalysis;

namespace Occt;

/// <summary>
/// A segment of the polygonal hidden-line removal: its two points in 3D and projected, the indices of the shape and
/// of the faces it bounds, and its flags (regular, outline, internal, hidden).
/// </summary>
/// <remarks>
/// A C++ value class, so a struct, kept in lists and changed in place (by reference) by HLRBRep_PolyAlgo.
/// <c>new()</c> and <c>default</c> differ in the shape index (OCCT's -1), so ported code writes <c>new()</c>. The
/// nested <see cref="IndicesT"/> and <see cref="PointsT"/> are plain C++ structs with public fields, ported as such;
/// <see cref="Indices"/> and <see cref="Points"/> return references to them, and the flag setters are internal.
/// </remarks>
public partial struct HLRAlgo_BiPoint : IEquatable<HLRAlgo_BiPoint>
{
    /// <summary>The indices of a segment (plain data).</summary>
    public struct IndicesT : IEquatable<IndicesT>
    {
        public int ShapeIndex;
        public int FaceConex1;
        public int Face1Pt1;
        public int Face1Pt2;
        public int FaceConex2;
        public int Face2Pt1;
        public int Face2Pt2;
        public int MinSeg;
        public int MaxSeg;
        public int SegFlags;

        public IndicesT()
        {
            ShapeIndex = -1;
            FaceConex1 = 0;
            Face1Pt1 = 0;
            Face1Pt2 = 0;
            FaceConex2 = 0;
            Face2Pt1 = 0;
            Face2Pt2 = 0;
            MinSeg = 0;
            MaxSeg = 0;
            SegFlags = 0;
        }

        public readonly bool Equals(IndicesT other)
            => ShapeIndex == other.ShapeIndex && FaceConex1 == other.FaceConex1 && Face1Pt1 == other.Face1Pt1
               && Face1Pt2 == other.Face1Pt2 && FaceConex2 == other.FaceConex2 && Face2Pt1 == other.Face2Pt1
               && Face2Pt2 == other.Face2Pt2 && MinSeg == other.MinSeg && MaxSeg == other.MaxSeg
               && SegFlags == other.SegFlags;

        public override readonly bool Equals(object? obj) => obj is IndicesT other && Equals(other);

        public override readonly int GetHashCode() => HashCode.Combine(ShapeIndex, FaceConex1, FaceConex2, SegFlags);

        public static bool operator ==(in IndicesT theLeft, in IndicesT theRight) => theLeft.Equals(theRight);

        public static bool operator !=(in IndicesT theLeft, in IndicesT theRight) => !theLeft.Equals(theRight);
    }

    /// <summary>The points of a segment (plain data): in 3D and projected.</summary>
    public struct PointsT : IEquatable<PointsT>
    {
        public Xyz Pnt1;
        public Xyz Pnt2;
        public Xyz PntP1;
        public Xyz PntP2;

        public readonly Xy PntP12D() => new(PntP1.X(), PntP1.Y());

        public readonly Xy PntP22D() => new(PntP2.X(), PntP2.Y());

        public readonly bool Equals(PointsT other)
            => Pnt1.Equals(other.Pnt1) && Pnt2.Equals(other.Pnt2) && PntP1.Equals(other.PntP1)
               && PntP2.Equals(other.PntP2);

        public override readonly bool Equals(object? obj) => obj is PointsT other && Equals(other);

        public override readonly int GetHashCode() => HashCode.Combine(Pnt1, Pnt2, PntP1, PntP2);

        public static bool operator ==(in PointsT theLeft, in PointsT theRight) => theLeft.Equals(theRight);

        public static bool operator !=(in PointsT theLeft, in PointsT theRight) => !theLeft.Equals(theRight);
    }

    private const int EMskRg1Line = 1;
    private const int EMskRgNLine = 2;
    private const int EMskOutLine = 4;
    private const int EMskIntLine = 8;
    private const int EMskHidden = 16;

    private IndicesT myIndices;
    private PointsT myPoints;

    public HLRAlgo_BiPoint()
    {
        myIndices = new IndicesT();
        myPoints = new PointsT();
    }

    public HLRAlgo_BiPoint(double X1, double Y1, double Z1, double X2, double Y2, double Z2, double XT1, double YT1,
                           double ZT1, double XT2, double YT2, double ZT2, int Index, bool reg1, bool regn, bool outl,
                           bool intl)
        : this()
    {
        myPoints.Pnt1.SetCoord(X1, Y1, Z1);
        myPoints.Pnt2.SetCoord(X2, Y2, Z2);
        myPoints.PntP1.SetCoord(XT1, YT1, ZT1);
        myPoints.PntP2.SetCoord(XT2, YT2, ZT2);
        myIndices.ShapeIndex = Index;
        myIndices.FaceConex1 = myIndices.Face1Pt1 = myIndices.Face1Pt2 = myIndices.FaceConex2 =
            myIndices.Face2Pt1 = myIndices.Face2Pt2 = 0;
        myIndices.SegFlags = 0;
        Rg1Line(reg1);
        RgNLine(regn);
        OutLine(outl);
        IntLine(intl);
        Hidden(false);
    }

    public HLRAlgo_BiPoint(double X1, double Y1, double Z1, double X2, double Y2, double Z2, double XT1, double YT1,
                           double ZT1, double XT2, double YT2, double ZT2, int Index, int flag)
        : this()
    {
        myPoints.Pnt1 = new Xyz(X1, Y1, Z1);
        myPoints.Pnt2 = new Xyz(X2, Y2, Z2);
        myPoints.PntP1 = new Xyz(XT1, YT1, ZT1);
        myPoints.PntP2 = new Xyz(XT2, YT2, ZT2);
        myIndices.ShapeIndex = Index;
        myIndices.FaceConex1 = myIndices.Face1Pt1 = myIndices.Face1Pt2 = myIndices.FaceConex2 =
            myIndices.Face2Pt1 = myIndices.Face2Pt2 = 0;
        myIndices.SegFlags = flag;
        Hidden(false);
    }

    public HLRAlgo_BiPoint(double X1, double Y1, double Z1, double X2, double Y2, double Z2, double XT1, double YT1,
                           double ZT1, double XT2, double YT2, double ZT2, int Index, int i1, int i1p1, int i1p2,
                           bool reg1, bool regn, bool outl, bool intl)
        : this()
    {
        myPoints.Pnt1.SetCoord(X1, Y1, Z1);
        myPoints.Pnt2.SetCoord(X2, Y2, Z2);
        myPoints.PntP1.SetCoord(XT1, YT1, ZT1);
        myPoints.PntP2.SetCoord(XT2, YT2, ZT2);
        myIndices.ShapeIndex = Index;
        myIndices.FaceConex1 = i1;
        myIndices.Face1Pt1 = i1p1;
        myIndices.Face1Pt2 = i1p2;
        myIndices.FaceConex2 = myIndices.Face2Pt1 = myIndices.Face2Pt2 = 0;
        myIndices.SegFlags = 0;
        Rg1Line(reg1);
        RgNLine(regn);
        OutLine(outl);
        IntLine(intl);
        Hidden(false);
    }

    public HLRAlgo_BiPoint(double X1, double Y1, double Z1, double X2, double Y2, double Z2, double XT1, double YT1,
                           double ZT1, double XT2, double YT2, double ZT2, int Index, int i1, int i1p1, int i1p2,
                           int flag)
        : this()
    {
        myPoints.Pnt1.SetCoord(X1, Y1, Z1);
        myPoints.Pnt2.SetCoord(X2, Y2, Z2);
        myPoints.PntP1.SetCoord(XT1, YT1, ZT1);
        myPoints.PntP2.SetCoord(XT2, YT2, ZT2);
        myIndices.ShapeIndex = Index;
        myIndices.FaceConex1 = i1;
        myIndices.Face1Pt1 = i1p1;
        myIndices.Face1Pt2 = i1p2;
        myIndices.FaceConex2 = myIndices.Face2Pt1 = myIndices.Face2Pt2 = 0;
        myIndices.SegFlags = flag;
        Hidden(false);
    }

    public HLRAlgo_BiPoint(double X1, double Y1, double Z1, double X2, double Y2, double Z2, double XT1, double YT1,
                           double ZT1, double XT2, double YT2, double ZT2, int Index, int i1, int i1p1, int i1p2,
                           int i2, int i2p1, int i2p2, bool reg1, bool regn, bool outl, bool intl)
        : this()
    {
        myPoints.Pnt1.SetCoord(X1, Y1, Z1);
        myPoints.Pnt2.SetCoord(X2, Y2, Z2);
        myPoints.PntP1.SetCoord(XT1, YT1, ZT1);
        myPoints.PntP2.SetCoord(XT2, YT2, ZT2);
        myIndices.ShapeIndex = Index;
        myIndices.FaceConex1 = i1;
        myIndices.Face1Pt1 = i1p1;
        myIndices.Face1Pt2 = i1p2;
        myIndices.FaceConex2 = i2;
        myIndices.Face2Pt1 = i2p1;
        myIndices.Face2Pt2 = i2p2;
        myIndices.SegFlags = 0;
        Rg1Line(reg1);
        RgNLine(regn);
        OutLine(outl);
        IntLine(intl);
        Hidden(false);
    }

    public HLRAlgo_BiPoint(double X1, double Y1, double Z1, double X2, double Y2, double Z2, double XT1, double YT1,
                           double ZT1, double XT2, double YT2, double ZT2, int Index, int i1, int i1p1, int i1p2,
                           int i2, int i2p1, int i2p2, int flag)
        : this()
    {
        myPoints.Pnt1.SetCoord(X1, Y1, Z1);
        myPoints.Pnt2.SetCoord(X2, Y2, Z2);
        myPoints.PntP1.SetCoord(XT1, YT1, ZT1);
        myPoints.PntP2.SetCoord(XT2, YT2, ZT2);
        myIndices.ShapeIndex = Index;
        myIndices.FaceConex1 = i1;
        myIndices.Face1Pt1 = i1p1;
        myIndices.Face1Pt2 = i1p2;
        myIndices.FaceConex2 = i2;
        myIndices.Face2Pt1 = i2p1;
        myIndices.Face2Pt2 = i2p2;
        myIndices.SegFlags = flag;
        Hidden(false);
    }

    public readonly bool Rg1Line() => (myIndices.SegFlags & EMskRg1Line) != 0;

    internal void Rg1Line(bool B)
    {
        if (B)
        {
            myIndices.SegFlags |= EMskRg1Line;
        }
        else
        {
            myIndices.SegFlags &= ~EMskRg1Line;
        }
    }

    public readonly bool RgNLine() => (myIndices.SegFlags & EMskRgNLine) != 0;

    internal void RgNLine(bool B)
    {
        if (B)
        {
            myIndices.SegFlags |= EMskRgNLine;
        }
        else
        {
            myIndices.SegFlags &= ~EMskRgNLine;
        }
    }

    public readonly bool OutLine() => (myIndices.SegFlags & EMskOutLine) != 0;

    internal void OutLine(bool B)
    {
        if (B)
        {
            myIndices.SegFlags |= EMskOutLine;
        }
        else
        {
            myIndices.SegFlags &= ~EMskOutLine;
        }
    }

    public readonly bool IntLine() => (myIndices.SegFlags & EMskIntLine) != 0;

    internal void IntLine(bool B)
    {
        if (B)
        {
            myIndices.SegFlags |= EMskIntLine;
        }
        else
        {
            myIndices.SegFlags &= ~EMskIntLine;
        }
    }

    public readonly bool Hidden() => (myIndices.SegFlags & EMskHidden) != 0;

    internal void Hidden(bool B)
    {
        if (B)
        {
            myIndices.SegFlags |= EMskHidden;
        }
        else
        {
            myIndices.SegFlags &= ~EMskHidden;
        }
    }

    /// <summary>The indices, by reference (OCCT's non-const accessor).</summary>
    [UnscopedRef]
    internal ref IndicesT Indices() => ref myIndices;

    /// <summary>The points, by reference (OCCT's non-const accessor).</summary>
    [UnscopedRef]
    internal ref PointsT Points() => ref myPoints;

    /// <summary>A copy of the indices.</summary>
    public readonly IndicesT IndicesValue() => myIndices;

    /// <summary>A copy of the points.</summary>
    public readonly PointsT PointsValue() => myPoints;

    public readonly bool Equals(HLRAlgo_BiPoint other)
        => myIndices.Equals(other.myIndices) && myPoints.Equals(other.myPoints);

    public override readonly bool Equals(object? obj) => obj is HLRAlgo_BiPoint other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(myIndices, myPoints);

    public static bool operator ==(in HLRAlgo_BiPoint theLeft, in HLRAlgo_BiPoint theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in HLRAlgo_BiPoint theLeft, in HLRAlgo_BiPoint theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString()
        => $"HLRAlgo_BiPoint({myPoints.Pnt1}, {myPoints.Pnt2}, shape {myIndices.ShapeIndex}, flags {myIndices.SegFlags})";
}
