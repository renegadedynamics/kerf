// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_BiPoint.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_BiPoint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine;
using System.Diagnostics.CodeAnalysis;

namespace Kerf.Engine.HiddenLines.Algorithm;

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
internal partial struct AlgoBiPoint : IEquatable<AlgoBiPoint>
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

        public static bool operator ==(in IndicesT left, in IndicesT right) => left.Equals(right);

        public static bool operator !=(in IndicesT left, in IndicesT right) => !left.Equals(right);
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

        public static bool operator ==(in PointsT left, in PointsT right) => left.Equals(right);

        public static bool operator !=(in PointsT left, in PointsT right) => !left.Equals(right);
    }

    private const int EMskRg1Line = 1;
    private const int EMskRgNLine = 2;
    private const int EMskOutLine = 4;
    private const int EMskIntLine = 8;
    private const int EMskHidden = 16;

    private IndicesT _indices;
    private PointsT _points;

    public AlgoBiPoint()
    {
        _indices = new IndicesT();
        _points = new PointsT();
    }

    public AlgoBiPoint(double x1, double y1, double z1, double x2, double y2, double z2, double xt1, double yt1,
                           double zt1, double xt2, double yt2, double zt2, int index, bool reg1, bool regn, bool outl,
                           bool intl)
        : this()
    {
        _points.Pnt1.SetCoord(x1, y1, z1);
        _points.Pnt2.SetCoord(x2, y2, z2);
        _points.PntP1.SetCoord(xt1, yt1, zt1);
        _points.PntP2.SetCoord(xt2, yt2, zt2);
        _indices.ShapeIndex = index;
        _indices.FaceConex1 = _indices.Face1Pt1 = _indices.Face1Pt2 = _indices.FaceConex2 =
            _indices.Face2Pt1 = _indices.Face2Pt2 = 0;
        _indices.SegFlags = 0;
        Rg1Line(reg1);
        RgNLine(regn);
        OutLine(outl);
        IntLine(intl);
        Hidden(false);
    }

    public AlgoBiPoint(double x1, double y1, double z1, double x2, double y2, double z2, double xt1, double yt1,
                           double zt1, double xt2, double yt2, double zt2, int index, int flag)
        : this()
    {
        _points.Pnt1 = new Xyz(x1, y1, z1);
        _points.Pnt2 = new Xyz(x2, y2, z2);
        _points.PntP1 = new Xyz(xt1, yt1, zt1);
        _points.PntP2 = new Xyz(xt2, yt2, zt2);
        _indices.ShapeIndex = index;
        _indices.FaceConex1 = _indices.Face1Pt1 = _indices.Face1Pt2 = _indices.FaceConex2 =
            _indices.Face2Pt1 = _indices.Face2Pt2 = 0;
        _indices.SegFlags = flag;
        Hidden(false);
    }

    public AlgoBiPoint(double x1, double y1, double z1, double x2, double y2, double z2, double xt1, double yt1,
                           double zt1, double xt2, double yt2, double zt2, int index, int i1, int i1p1, int i1p2,
                           bool reg1, bool regn, bool outl, bool intl)
        : this()
    {
        _points.Pnt1.SetCoord(x1, y1, z1);
        _points.Pnt2.SetCoord(x2, y2, z2);
        _points.PntP1.SetCoord(xt1, yt1, zt1);
        _points.PntP2.SetCoord(xt2, yt2, zt2);
        _indices.ShapeIndex = index;
        _indices.FaceConex1 = i1;
        _indices.Face1Pt1 = i1p1;
        _indices.Face1Pt2 = i1p2;
        _indices.FaceConex2 = _indices.Face2Pt1 = _indices.Face2Pt2 = 0;
        _indices.SegFlags = 0;
        Rg1Line(reg1);
        RgNLine(regn);
        OutLine(outl);
        IntLine(intl);
        Hidden(false);
    }

    public AlgoBiPoint(double x1, double y1, double z1, double x2, double y2, double z2, double xt1, double yt1,
                           double zt1, double xt2, double yt2, double zt2, int index, int i1, int i1p1, int i1p2,
                           int flag)
        : this()
    {
        _points.Pnt1.SetCoord(x1, y1, z1);
        _points.Pnt2.SetCoord(x2, y2, z2);
        _points.PntP1.SetCoord(xt1, yt1, zt1);
        _points.PntP2.SetCoord(xt2, yt2, zt2);
        _indices.ShapeIndex = index;
        _indices.FaceConex1 = i1;
        _indices.Face1Pt1 = i1p1;
        _indices.Face1Pt2 = i1p2;
        _indices.FaceConex2 = _indices.Face2Pt1 = _indices.Face2Pt2 = 0;
        _indices.SegFlags = flag;
        Hidden(false);
    }

    public AlgoBiPoint(double x1, double y1, double z1, double x2, double y2, double z2, double xt1, double yt1,
                           double zt1, double xt2, double yt2, double zt2, int index, int i1, int i1p1, int i1p2,
                           int i2, int i2p1, int i2p2, bool reg1, bool regn, bool outl, bool intl)
        : this()
    {
        _points.Pnt1.SetCoord(x1, y1, z1);
        _points.Pnt2.SetCoord(x2, y2, z2);
        _points.PntP1.SetCoord(xt1, yt1, zt1);
        _points.PntP2.SetCoord(xt2, yt2, zt2);
        _indices.ShapeIndex = index;
        _indices.FaceConex1 = i1;
        _indices.Face1Pt1 = i1p1;
        _indices.Face1Pt2 = i1p2;
        _indices.FaceConex2 = i2;
        _indices.Face2Pt1 = i2p1;
        _indices.Face2Pt2 = i2p2;
        _indices.SegFlags = 0;
        Rg1Line(reg1);
        RgNLine(regn);
        OutLine(outl);
        IntLine(intl);
        Hidden(false);
    }

    public AlgoBiPoint(double x1, double y1, double z1, double x2, double y2, double z2, double xt1, double yt1,
                           double zt1, double xt2, double yt2, double zt2, int index, int i1, int i1p1, int i1p2,
                           int i2, int i2p1, int i2p2, int flag)
        : this()
    {
        _points.Pnt1.SetCoord(x1, y1, z1);
        _points.Pnt2.SetCoord(x2, y2, z2);
        _points.PntP1.SetCoord(xt1, yt1, zt1);
        _points.PntP2.SetCoord(xt2, yt2, zt2);
        _indices.ShapeIndex = index;
        _indices.FaceConex1 = i1;
        _indices.Face1Pt1 = i1p1;
        _indices.Face1Pt2 = i1p2;
        _indices.FaceConex2 = i2;
        _indices.Face2Pt1 = i2p1;
        _indices.Face2Pt2 = i2p2;
        _indices.SegFlags = flag;
        Hidden(false);
    }

    public readonly bool Rg1Line() => (_indices.SegFlags & EMskRg1Line) != 0;

    internal void Rg1Line(bool b)
    {
        if (b)
        {
            _indices.SegFlags |= EMskRg1Line;
        }
        else
        {
            _indices.SegFlags &= ~EMskRg1Line;
        }
    }

    public readonly bool RgNLine() => (_indices.SegFlags & EMskRgNLine) != 0;

    internal void RgNLine(bool b)
    {
        if (b)
        {
            _indices.SegFlags |= EMskRgNLine;
        }
        else
        {
            _indices.SegFlags &= ~EMskRgNLine;
        }
    }

    public readonly bool OutLine() => (_indices.SegFlags & EMskOutLine) != 0;

    internal void OutLine(bool b)
    {
        if (b)
        {
            _indices.SegFlags |= EMskOutLine;
        }
        else
        {
            _indices.SegFlags &= ~EMskOutLine;
        }
    }

    public readonly bool IntLine() => (_indices.SegFlags & EMskIntLine) != 0;

    internal void IntLine(bool b)
    {
        if (b)
        {
            _indices.SegFlags |= EMskIntLine;
        }
        else
        {
            _indices.SegFlags &= ~EMskIntLine;
        }
    }

    public readonly bool Hidden() => (_indices.SegFlags & EMskHidden) != 0;

    internal void Hidden(bool b)
    {
        if (b)
        {
            _indices.SegFlags |= EMskHidden;
        }
        else
        {
            _indices.SegFlags &= ~EMskHidden;
        }
    }

    /// <summary>The indices, by reference (OCCT's non-const accessor).</summary>
    [UnscopedRef]
    internal ref IndicesT Indices() => ref _indices;

    /// <summary>The points, by reference (OCCT's non-const accessor).</summary>
    [UnscopedRef]
    internal ref PointsT Points() => ref _points;

    /// <summary>A copy of the indices.</summary>
    public readonly IndicesT IndicesValue() => _indices;

    /// <summary>A copy of the points.</summary>
    public readonly PointsT PointsValue() => _points;

    public readonly bool Equals(AlgoBiPoint other)
        => _indices.Equals(other._indices) && _points.Equals(other._points);

    public override readonly bool Equals(object? obj) => obj is AlgoBiPoint other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_indices, _points);

    public static bool operator ==(in AlgoBiPoint left, in AlgoBiPoint right) => left.Equals(right);

    public static bool operator !=(in AlgoBiPoint left, in AlgoBiPoint right) => !left.Equals(right);

    public override readonly string ToString()
        => $"HLRAlgo_BiPoint({_points.Pnt1}, {_points.Pnt2}, shape {_indices.ShapeIndex}, flags {_indices.SegFlags})";
}
