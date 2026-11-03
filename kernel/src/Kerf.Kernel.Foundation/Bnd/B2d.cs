// Port of: src/FoundationClasses/TKMath/Bnd/Bnd_B2.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2005-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// OCCT's template Bnd_B2<RealType> is ported once per instantiation (Bnd_B2d, Bnd_B2f) from the same text, with
// RealType a file-level alias, so C++'s float/double promotions are kept exactly (see docs/decisions.md).

using Kerf.Geometry;
using Occt;
using System.Runtime.CompilerServices;
using static Kerf.Geometry.TransformForm;
using RealType = double;

namespace Kerf.Engine.Geometry.Bounds;

/// <summary>
/// A 2D axis-aligned bounding box stored as a center and a half-diagonal, in double precision (OCCT's
/// <c>Bnd_B2&lt;double&gt;</c>).
/// </summary>
/// <remarks>
/// OCCT's default (and <c>new Bnd_B2d()</c>) is a void box. <c>default(Bnd_B2d)</c> is not: it is a box reduced to
/// the origin. Ported code must write <c>new Bnd_B2d()</c>.
/// </remarks>
internal partial struct B2d : IEquatable<B2d>
{
    /// <summary>Two reals, as OCCT's <c>std::array&lt;RealType, 2&gt;</c>.</summary>
    [InlineArray(2)]
    public struct Array2
    {
        private RealType _element0;

        public Array2(RealType the0, RealType the1)
        {
            this[0] = the0;
            this[1] = the1;
        }
    }

    // Constant representing a very large value for void box initialization
    private const RealType THE_RealLast = (RealType)1e30;

    private Array2 _center;
    private Array2 _hSize;

    /// <summary>Creates a void box.</summary>
    public B2d()
    {
        _center = new Array2(THE_RealLast, THE_RealLast);
        _hSize = new Array2(-THE_RealLast, -THE_RealLast);
    }

    public B2d(in Xy center, in Xy hSize)
    {
        _center = new Array2((RealType)center.X(), (RealType)center.Y());
        _hSize = new Array2((RealType)hSize.X(), (RealType)hSize.Y());
    }

    public B2d(in Array2 center, in Array2 hSize)
    {
        _center = center;
        _hSize = hSize;
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void Clear()
    {
        _center[0] = THE_RealLast;
        _center[1] = THE_RealLast;
        _hSize[0] = -THE_RealLast;
        _hSize[1] = -THE_RealLast;
    }

    internal void Add(in Xy pnt)
    {
        if (IsVoid)
        {
            _center[0] = (RealType)pnt.X();
            _center[1] = (RealType)pnt.Y();
            _hSize[0] = (RealType)0.0;
            _hSize[1] = (RealType)0.0;
        }
        else
        {
            Span<RealType> diff = [(RealType)pnt.X() - _center[0], (RealType)pnt.Y() - _center[1]];
            if (diff[0] > _hSize[0])
            {
                RealType shift = (diff[0] - _hSize[0]) / 2;
                _center[0] += shift;
                _hSize[0] += shift;
            }
            else if (diff[0] < -_hSize[0])
            {
                RealType shift = (diff[0] + _hSize[0]) / 2;
                _center[0] += shift;
                _hSize[0] -= shift;
            }

            if (diff[1] > _hSize[1])
            {
                RealType shift = (diff[1] - _hSize[1]) / 2;
                _center[1] += shift;
                _hSize[1] += shift;
            }
            else if (diff[1] < -_hSize[1])
            {
                RealType shift = (diff[1] + _hSize[1]) / 2;
                _center[1] += shift;
                _hSize[1] -= shift;
            }
        }
    }

    internal void Add(in Point2D pnt) => Add(pnt.XY());

    internal void Add(in B2d box)
    {
        if (!box.IsVoid)
        {
            Add(box.CornerMin());
            Add(box.CornerMax());
        }
    }

    internal void Enlarge(double aDiff)
    {
        RealType d = (RealType)Math.Abs(aDiff);
        _hSize[0] += d;
        _hSize[1] += d;
    }

    /// <summary>Limits the box by the inside of <paramref name="box"/>; false if the boxes do not intersect.</summary>
    internal bool Limit(in B2d box)
    {
        bool result = false;
        Span<RealType> diffC = [box._center[0] - _center[0], box._center[1] - _center[1]];
        Span<RealType> sumH = [box._hSize[0] + _hSize[0], box._hSize[1] + _hSize[1]];
        // check the condition IsOut
        if (!CompareDist(sumH, diffC))
        {
            Span<RealType> diffH = [box._hSize[0] - _hSize[0], box._hSize[1] - _hSize[1]];
            if (diffC[0] - diffH[0] > 0.0)
            {
                RealType shift = (diffC[0] - diffH[0]) / 2; // positive
                _center[0] += shift;
                _hSize[0] -= shift;
            }
            else if (diffC[0] + diffH[0] < 0.0)
            {
                RealType shift = (diffC[0] + diffH[0]) / 2; // negative
                _center[0] += shift;
                _hSize[0] += shift;
            }

            if (diffC[1] - diffH[1] > 0.0)
            {
                RealType shift = (diffC[1] - diffH[1]) / 2; // positive
                _center[1] += shift;
                _hSize[1] -= shift;
            }
            else if (diffC[1] + diffH[1] < 0.0)
            {
                RealType shift = (diffC[1] + diffH[1]) / 2; // negative
                _center[1] += shift;
                _hSize[1] += shift;
            }

            result = true;
        }

        return result;
    }

    internal void SetCenter(in Xy center)
    {
        _center[0] = (RealType)center.X();
        _center[1] = (RealType)center.Y();
    }

    internal void SetCenter(in Array2 center) => _center = center;

    internal void SetHSize(in Xy hSize)
    {
        _hSize[0] = (RealType)hSize.X();
        _hSize[1] = (RealType)hSize.Y();
    }

    internal void SetHSize(in Array2 hSize) => _hSize = hSize;

    // Copies with a mutator applied (the public way to "set")

    public readonly B2d Cleared() => new();

    public readonly B2d Added(in Xy pnt)
    {
        B2d copy = this;
        copy.Add(pnt);
        return copy;
    }

    public readonly B2d Added(in Point2D pnt)
    {
        B2d copy = this;
        copy.Add(pnt);
        return copy;
    }

    public readonly B2d Added(in B2d box)
    {
        B2d copy = this;
        copy.Add(box);
        return copy;
    }

    public readonly B2d Enlarged(double diff)
    {
        B2d copy = this;
        copy.Enlarge(diff);
        return copy;
    }

    /// <summary>This box limited by the inside of <paramref name="box"/>; unchanged if they do not intersect (<see cref="IsOut(in B2d)"/>).</summary>
    public readonly B2d Limited(in B2d box)
    {
        B2d copy = this;
        copy.Limit(box);
        return copy;
    }

    public readonly B2d WithCenter(in Xy center)
    {
        B2d copy = this;
        copy.SetCenter(center);
        return copy;
    }

    public readonly B2d WithCenter(in Array2 center)
    {
        B2d copy = this;
        copy.SetCenter(center);
        return copy;
    }

    public readonly B2d WithHSize(in Xy hSize)
    {
        B2d copy = this;
        copy.SetHSize(hSize);
        return copy;
    }

    public readonly B2d WithHSize(in Array2 hSize)
    {
        B2d copy = this;
        copy.SetHSize(hSize);
        return copy;
    }

    // Queries

    /// <summary>True if the box is void (non-initialized).</summary>
    public readonly bool IsVoid => (_hSize[0] < -1e-5);

    /// <summary>Center - HSize. Irrelevant for a void box.</summary>
    public readonly Xy CornerMin() => new(_center[0] - _hSize[0], _center[1] - _hSize[1]);

    /// <summary>Center + HSize. Irrelevant for a void box.</summary>
    public readonly Xy CornerMax() => new(_center[0] + _hSize[0], _center[1] + _hSize[1]);

    /// <summary>The square diagonal; very big for a void box.</summary>
    public readonly double SquareExtent() => 4 * (_hSize[0] * _hSize[0] + _hSize[1] * _hSize[1]);

    public readonly Array2 Center() => _center;

    public readonly Array2 HSize() => _hSize;

    /// <summary>The box bounding this box transformed by <paramref name="trsf"/> (larger if it rotates).</summary>
    public readonly B2d Transformed(in Transform2D trsf)
    {
        B2d result = new();
        TransformForm form = trsf.Form;
        double scale = trsf.ScaleFactor;
        double scaleAbs = Math.Abs(scale);
        if (form == Identity)
        {
            result = this;
        }
        else if (form == Translation || form == PntMirror || form == Scale)
        {
            result._center[0] = (RealType)(_center[0] * scale + trsf.TranslationXy().X());
            result._center[1] = (RealType)(_center[1] * scale + trsf.TranslationXy().Y());
            result._hSize[0] = (RealType)(_hSize[0] * scaleAbs);
            result._hSize[1] = (RealType)(_hSize[1] * scaleAbs);
        }
        else
        {
            Xy center = new((double)_center[0], (double)_center[1]);
            trsf.Transforms(ref center);
            result._center[0] = (RealType)center.X();
            result._center[1] = (RealType)center.Y();
            Span<double> mat = stackalloc double[4];
            trsf.RotationPart.CopyTo(mat);
            result._hSize[0] =
                (RealType)(scaleAbs * (Math.Abs(mat[0]) * _hSize[0] + Math.Abs(mat[1]) * _hSize[1]));
            result._hSize[1] =
                (RealType)(scaleAbs * (Math.Abs(mat[2]) * _hSize[0] + Math.Abs(mat[3]) * _hSize[1]));
        }

        return result;
    }

    /// <summary>True if the point is outside the box.</summary>
    public readonly bool IsOut(in Xy pnt)
        => (Math.Abs((RealType)pnt.X() - _center[0]) > _hSize[0]
            || Math.Abs((RealType)pnt.Y() - _center[1]) > _hSize[1]);

    /// <summary>True if the circle (or its outline, if hollow) does not intersect the box.</summary>
    public readonly bool IsOut(in Xy center, double radius, bool isCircleHollow = false)
    {
        bool result = true;
        if (!isCircleHollow)
        {
            // vector from the center of the circle to the nearest box face
            Span<double> dist =
            [
                Math.Abs(center.X() - (double)_center[0]) - (double)_hSize[0],
                Math.Abs(center.Y() - (double)_center[1]) - (double)_hSize[1],
            ];
            double d = 0.0;
            if (dist[0] > 0.0)
            {
                d = dist[0] * dist[0];
            }

            if (dist[1] > 0.0)
            {
                d += dist[1] * dist[1];
            }

            result = (d > radius * radius);
        }
        else
        {
            Span<double> distC =
            [
                Math.Abs(center.X() - (double)_center[0]),
                Math.Abs(center.Y() - (double)_center[1]),
            ];
            // vector from the center of the circle to the nearest box face
            Span<double> dist = [distC[0] - (double)_hSize[0], distC[1] - (double)_hSize[1]];
            double d = 0.0;
            if (dist[0] > 0.0)
            {
                d = dist[0] * dist[0];
            }

            if (dist[1] > 0.0)
            {
                d += dist[1] * dist[1];
            }

            if (d < radius * radius)
            {
                // the box intersects the solid circle; check if it is completely
                // inside the circle (in such case return isOut==True)
                dist[0] = distC[0] + (double)_hSize[0];
                dist[1] = distC[1] + (double)_hSize[1];
                if (dist[0] * dist[0] + dist[1] * dist[1] > radius * radius)
                {
                    result = false;
                }
            }
        }

        return result;
    }

    /// <summary>True if <paramref name="box"/> does not intersect this box.</summary>
    public readonly bool IsOut(in B2d box)
        => (Math.Abs(box._center[0] - _center[0]) > box._hSize[0] + _hSize[0]
            || Math.Abs(box._center[1] - _center[1]) > box._hSize[1] + _hSize[1]);

    /// <summary>True if <paramref name="box"/> transformed by <paramref name="trsf"/> does not intersect this box.</summary>
    public readonly bool IsOut(in B2d box, in Transform2D trsf)
    {
        bool result = false;
        TransformForm form = trsf.Form;
        double scale = trsf.ScaleFactor;
        double scaleAbs = Math.Abs(scale);
        if (form == Translation || form == Identity || form == PntMirror || form == Scale)
        {
            result =
                (Math.Abs((RealType)(box._center[0] * scale + trsf.TranslationXy().X()) - _center[0])
                     > (RealType)(box._hSize[0] * scaleAbs) + _hSize[0]
                 || Math.Abs((RealType)(box._center[1] * scale + trsf.TranslationXy().Y()) - _center[1])
                     > (RealType)(box._hSize[1] * scaleAbs) + _hSize[1]);
        }
        else
        {
            // theBox is transformed and we check the resulting (enlarged) box against
            // 'this' box.
            Span<double> mat = stackalloc double[4];
            trsf.RotationPart.CopyTo(mat);
            Xy center = new((double)box._center[0], (double)box._center[1]);
            trsf.Transforms(ref center);
            Span<double> dist = [center.X() - (double)_center[0], center.Y() - (double)_center[1]];
            Span<double> matAbs = [Math.Abs(mat[0]), Math.Abs(mat[1]), Math.Abs(mat[2]), Math.Abs(mat[3])];
            if (Math.Abs(dist[0])
                    > (scaleAbs * (matAbs[0] * box._hSize[0] + matAbs[1] * box._hSize[1])
                       + (double)_hSize[0])
                || Math.Abs(dist[1])
                    > (scaleAbs * (matAbs[2] * box._hSize[0] + matAbs[3] * box._hSize[1])
                       + (double)_hSize[1]))
            {
                result = true;
            }
            else
            {
                // theBox is rotated, scaled and translated. We apply the reverse
                // translation and scaling then check against the rotated box 'this'
                if ((Math.Abs(mat[0] * dist[0] + mat[2] * dist[1])
                     > box._hSize[0] * scaleAbs + (matAbs[0] * _hSize[0] + matAbs[2] * _hSize[1]))
                    || (Math.Abs(mat[1] * dist[0] + mat[3] * dist[1])
                        > box._hSize[1] * scaleAbs + (matAbs[1] * _hSize[0] + matAbs[3] * _hSize[1])))
                {
                    result = true;
                }
            }
        }

        return result;
    }

    /// <summary>True if the line does not intersect the box.</summary>
    public readonly bool IsOut(in Axis2D line)
    {
        if (IsVoid)
        {
            return true;
        }

        // Intersect the line containing the segment.
        Span<double> prod =
        [
            line.Direction.XY()
                ^ (new Xy(_center[0] - line.Location.X, _center[1] - line.Location.Y)),
            line.Direction.X * (double)_hSize[1],
            line.Direction.Y * (double)_hSize[0],
        ];
        return (Math.Abs(prod[0]) > (Math.Abs(prod[1]) + Math.Abs(prod[2])));
    }

    /// <summary>True if the segment [theP0, theP1] does not intersect the box.</summary>
    public readonly bool IsOut(in Xy p0, in Xy p1)
    {
        bool result = true;
        if (!IsVoid)
        {
            // Intersect the line containing the segment.
            Xy segDelta = p1 - p0;
            Span<double> prod =
            [
                segDelta ^ (new Xy(_center[0], _center[1]) - p0),
                segDelta.X() * (double)_hSize[1],
                segDelta.Y() * (double)_hSize[0],
            ];
            if (Math.Abs(prod[0]) < (Math.Abs(prod[1]) + Math.Abs(prod[2])))
            {
                // Intersection with line detected; check the segment as bounding box
                Xy hSeg = new(0.5 * segDelta.X(), 0.5 * segDelta.Y());
                Xy hSegAbs = new(Math.Abs(hSeg.X()), Math.Abs(hSeg.Y()));
                result = CompareDistD(new Xy((double)_hSize[0], (double)_hSize[1]) + hSegAbs,
                                       p0 + hSeg - new Xy((double)_center[0], (double)_center[1]));
            }
        }

        return result;
    }

    /// <summary>True if this box is fully inside <paramref name="box"/>.</summary>
    public readonly bool IsIn(in B2d box)
        => (Math.Abs(box._center[0] - _center[0]) < box._hSize[0] - _hSize[0]
            && Math.Abs(box._center[1] - _center[1]) < box._hSize[1] - _hSize[1]);

    /// <summary>True if this box is fully inside <paramref name="box"/> transformed by <paramref name="trsf"/>.</summary>
    public readonly bool IsIn(in B2d box, in Transform2D trsf)
    {
        bool result = false;
        TransformForm form = trsf.Form;
        double scale = trsf.ScaleFactor;
        double scaleAbs = Math.Abs(scale);
        if (form == Translation || form == Identity || form == PntMirror || form == Scale)
        {
            result =
                (Math.Abs((RealType)(box._center[0] * scale + trsf.TranslationXy().X()) - _center[0])
                     < (RealType)(box._hSize[0] * scaleAbs) - _hSize[0]
                 && Math.Abs((RealType)(box._center[1] * scale + trsf.TranslationXy().Y()) - _center[1])
                     < (RealType)(box._hSize[1] * scaleAbs) - _hSize[1]);
        }
        else
        {
            // theBox is rotated, scaled and translated. We apply the reverse
            // translation and scaling then check against the rotated box 'this'
            Span<double> mat = stackalloc double[4];
            trsf.RotationPart.CopyTo(mat);
            Xy center = new((double)box._center[0], (double)box._center[1]);
            trsf.Transforms(ref center);
            Span<double> dist = [center.X() - (double)_center[0], center.Y() - (double)_center[1]];
            if ((Math.Abs(mat[0] * dist[0] + mat[2] * dist[1])
                 < box._hSize[0] * scaleAbs - (Math.Abs(mat[0]) * _hSize[0] + Math.Abs(mat[2]) * _hSize[1]))
                && (Math.Abs(mat[1] * dist[0] + mat[3] * dist[1])
                    < box._hSize[1] * scaleAbs - (Math.Abs(mat[1]) * _hSize[0] + Math.Abs(mat[3]) * _hSize[1])))
            {
                result = true;
            }
        }

        return result;
    }

    private static bool CompareDist(ReadOnlySpan<RealType> aHSize, ReadOnlySpan<RealType> aDist)
        => (Math.Abs(aDist[0]) > aHSize[0] || Math.Abs(aDist[1]) > aHSize[1]);

    private static bool CompareDistD(in Xy aHSize, in Xy aDist)
        => (Math.Abs(aDist.X()) > aHSize.X() || Math.Abs(aDist.Y()) > aHSize.Y());

    // Exact equality, for use as a value

    public readonly bool Equals(B2d other)
        => _center[0].Equals(other._center[0]) && _center[1].Equals(other._center[1])
           && _hSize[0].Equals(other._hSize[0]) && _hSize[1].Equals(other._hSize[1]);

    public override readonly bool Equals(object? obj) => obj is B2d other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_center[0], _center[1], _hSize[0], _hSize[1]);

    public static bool operator ==(in B2d left, in B2d right) => left.Equals(right);

    public static bool operator !=(in B2d left, in B2d right) => !left.Equals(right);

    /// <summary>The center and the half-diagonal.</summary>
    public override readonly string ToString() => Constants.Format("Bnd_B2d", _center[0], _center[1], _hSize[0], _hSize[1]);
}
