// Port of: src/FoundationClasses/TKMath/Bnd/Bnd_B3.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2005-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// OCCT's template Bnd_B3<RealType> is ported once per instantiation (Bnd_B3d, Bnd_B3f) from the same text, with
// RealType a file-level alias, so C++'s float/double promotions are kept exactly (see docs/decisions.md).

using Kerf.Geometry;
using Occt;
using System.Runtime.CompilerServices;
using static Kerf.Geometry.TransformForm;
using RealType = float;

namespace Kerf.Engine.Geometry.Bounds;

/// <summary>
/// A 3D axis-aligned bounding box stored as a center and a half-diagonal, in single precision (OCCT's
/// <c>Bnd_B3&lt;float&gt;</c>).
/// </summary>
/// <remarks>
/// OCCT's default (and <c>new Bnd_B3f()</c>) is a void box. <c>default(Bnd_B3f)</c> is not: it is a box reduced to
/// the origin. Ported code must write <c>new Bnd_B3f()</c>.
/// </remarks>
public partial struct B3f : IEquatable<B3f>
{
    /// <summary>Three reals, as OCCT's <c>std::array&lt;RealType, 3&gt;</c>.</summary>
    [InlineArray(3)]
    public struct Array3
    {
        private RealType _element0;

        public Array3(RealType the0, RealType the1, RealType the2)
        {
            this[0] = the0;
            this[1] = the1;
            this[2] = the2;
        }
    }

    // Constant representing a very large value for void box initialization
    private const RealType THE_RealLast = (RealType)1e30;

    private Array3 _center;
    private Array3 _hSize;

    /// <summary>Creates a void box.</summary>
    public B3f()
    {
        _center = new Array3(THE_RealLast, THE_RealLast, THE_RealLast);
        _hSize = new Array3(-THE_RealLast, -THE_RealLast, -THE_RealLast);
    }

    public B3f(in Xyz center, in Xyz hSize)
    {
        _center = new Array3((RealType)center.X(), (RealType)center.Y(), (RealType)center.Z());
        _hSize = new Array3((RealType)hSize.X(), (RealType)hSize.Y(), (RealType)hSize.Z());
    }

    public B3f(in Array3 center, in Array3 hSize)
    {
        _center = center;
        _hSize = hSize;
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void Clear()
    {
        _center[0] = THE_RealLast;
        _center[1] = THE_RealLast;
        _center[2] = THE_RealLast;
        _hSize[0] = -THE_RealLast;
        _hSize[1] = -THE_RealLast;
        _hSize[2] = -THE_RealLast;
    }

    internal void Add(in Xyz pnt)
    {
        if (IsVoid)
        {
            _center[0] = (RealType)pnt.X();
            _center[1] = (RealType)pnt.Y();
            _center[2] = (RealType)pnt.Z();
            _hSize[0] = (RealType)0.0;
            _hSize[1] = (RealType)0.0;
            _hSize[2] = (RealType)0.0;
        }
        else
        {
            Span<RealType> diff =
            [
                (RealType)pnt.X() - _center[0],
                (RealType)pnt.Y() - _center[1],
                (RealType)pnt.Z() - _center[2],
            ];
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

            if (diff[2] > _hSize[2])
            {
                RealType shift = (diff[2] - _hSize[2]) / 2;
                _center[2] += shift;
                _hSize[2] += shift;
            }
            else if (diff[2] < -_hSize[2])
            {
                RealType shift = (diff[2] + _hSize[2]) / 2;
                _center[2] += shift;
                _hSize[2] -= shift;
            }
        }
    }

    internal void Add(in Point3D pnt) => Add(pnt.XYZ());

    internal void Add(in B3f box)
    {
        if (!box.IsVoid)
        {
            Add(box.CornerMin());
            Add(box.CornerMax());
        }
    }

    internal void Enlarge(double aDiff)
    {
        double d = Math.Abs(aDiff);
        _hSize[0] += (RealType)d;
        _hSize[1] += (RealType)d;
        _hSize[2] += (RealType)d;
    }

    /// <summary>Limits the box by the inside of <paramref name="box"/>; false if the boxes do not intersect.</summary>
    internal bool Limit(in B3f box)
    {
        bool result = false;
        Span<RealType> diffC =
        [
            box._center[0] - _center[0],
            box._center[1] - _center[1],
            box._center[2] - _center[2],
        ];
        Span<RealType> sumH =
        [
            box._hSize[0] + _hSize[0],
            box._hSize[1] + _hSize[1],
            box._hSize[2] + _hSize[2],
        ];
        // check the condition IsOut
        if (!CompareDist(sumH, diffC))
        {
            Span<RealType> diffH =
            [
                box._hSize[0] - _hSize[0],
                box._hSize[1] - _hSize[1],
                box._hSize[2] - _hSize[2],
            ];
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

            if (diffC[2] - diffH[2] > 0.0)
            {
                RealType shift = (diffC[2] - diffH[2]) / 2; // positive
                _center[2] += shift;
                _hSize[2] -= shift;
            }
            else if (diffC[2] + diffH[2] < 0.0)
            {
                RealType shift = (diffC[2] + diffH[2]) / 2; // negative
                _center[2] += shift;
                _hSize[2] += shift;
            }

            result = true;
        }

        return result;
    }

    internal void SetCenter(in Xyz center)
    {
        _center[0] = (RealType)center.X();
        _center[1] = (RealType)center.Y();
        _center[2] = (RealType)center.Z();
    }

    internal void SetCenter(in Array3 center) => _center = center;

    internal void SetHSize(in Xyz hSize)
    {
        _hSize[0] = (RealType)hSize.X();
        _hSize[1] = (RealType)hSize.Y();
        _hSize[2] = (RealType)hSize.Z();
    }

    internal void SetHSize(in Array3 hSize) => _hSize = hSize;

    // Copies with a mutator applied (the public way to "set")

    public readonly B3f Cleared() => new();

    public readonly B3f Added(in Xyz pnt)
    {
        B3f copy = this;
        copy.Add(pnt);
        return copy;
    }

    public readonly B3f Added(in Point3D pnt)
    {
        B3f copy = this;
        copy.Add(pnt);
        return copy;
    }

    public readonly B3f Added(in B3f box)
    {
        B3f copy = this;
        copy.Add(box);
        return copy;
    }

    public readonly B3f Enlarged(double diff)
    {
        B3f copy = this;
        copy.Enlarge(diff);
        return copy;
    }

    /// <summary>This box limited by the inside of <paramref name="box"/>; unchanged if they do not intersect (<see cref="IsOut(in B3f)"/>).</summary>
    public readonly B3f Limited(in B3f box)
    {
        B3f copy = this;
        copy.Limit(box);
        return copy;
    }

    public readonly B3f WithCenter(in Xyz center)
    {
        B3f copy = this;
        copy.SetCenter(center);
        return copy;
    }

    public readonly B3f WithCenter(in Array3 center)
    {
        B3f copy = this;
        copy.SetCenter(center);
        return copy;
    }

    public readonly B3f WithHSize(in Xyz hSize)
    {
        B3f copy = this;
        copy.SetHSize(hSize);
        return copy;
    }

    public readonly B3f WithHSize(in Array3 hSize)
    {
        B3f copy = this;
        copy.SetHSize(hSize);
        return copy;
    }

    // Queries

    /// <summary>True if the box is void (non-initialized).</summary>
    public readonly bool IsVoid => (_hSize[0] < -1e-5);

    /// <summary>Center - HSize. Irrelevant for a void box.</summary>
    public readonly Xyz CornerMin()
        => new(_center[0] - _hSize[0], _center[1] - _hSize[1], _center[2] - _hSize[2]);

    /// <summary>Center + HSize. Irrelevant for a void box.</summary>
    public readonly Xyz CornerMax()
        => new(_center[0] + _hSize[0], _center[1] + _hSize[1], _center[2] + _hSize[2]);

    /// <summary>The square diagonal; very big for a void box.</summary>
    public readonly double SquareExtent()
        => 4 * (_hSize[0] * _hSize[0] + _hSize[1] * _hSize[1] + _hSize[2] * _hSize[2]);

    public readonly Array3 Center() => _center;

    public readonly Array3 HSize() => _hSize;

    /// <summary>The box bounding this box transformed by <paramref name="trsf"/> (larger if it rotates).</summary>
    public readonly B3f Transformed(in Transform3D trsf)
    {
        B3f result = new();
        TransformForm form = trsf.Form();
        double scale = trsf.ScaleFactor();
        double scaleAbs = Math.Abs(scale);
        if (form == Identity)
        {
            result = this;
        }
        else if (form == Translation || form == PntMirror || form == Scale)
        {
            result._center[0] = (RealType)(_center[0] * scale + trsf.TranslationPart().X());
            result._center[1] = (RealType)(_center[1] * scale + trsf.TranslationPart().Y());
            result._center[2] = (RealType)(_center[2] * scale + trsf.TranslationPart().Z());
            result._hSize[0] = (RealType)(_hSize[0] * scaleAbs);
            result._hSize[1] = (RealType)(_hSize[1] * scaleAbs);
            result._hSize[2] = (RealType)(_hSize[2] * scaleAbs);
        }
        else
        {
            Xyz center = new((double)_center[0], (double)_center[1], (double)_center[2]);
            trsf.Transforms(ref center);
            result._center[0] = (RealType)center.X();
            result._center[1] = (RealType)center.Y();
            result._center[2] = (RealType)center.Z();
            Span<double> mat = stackalloc double[9];
            trsf.HVectorialPart().CopyTo(mat);
            result._hSize[0] =
                (RealType)(scaleAbs
                           * (Math.Abs(mat[0]) * _hSize[0] + Math.Abs(mat[1]) * _hSize[1]
                              + Math.Abs(mat[2]) * _hSize[2]));
            result._hSize[1] =
                (RealType)(scaleAbs
                           * (Math.Abs(mat[3]) * _hSize[0] + Math.Abs(mat[4]) * _hSize[1]
                              + Math.Abs(mat[5]) * _hSize[2]));
            result._hSize[2] =
                (RealType)(scaleAbs
                           * (Math.Abs(mat[6]) * _hSize[0] + Math.Abs(mat[7]) * _hSize[1]
                              + Math.Abs(mat[8]) * _hSize[2]));
        }

        return result;
    }

    /// <summary>True if the point is outside the box.</summary>
    public readonly bool IsOut(in Xyz pnt)
        => (Math.Abs((RealType)pnt.X() - _center[0]) > _hSize[0]
            || Math.Abs((RealType)pnt.Y() - _center[1]) > _hSize[1]
            || Math.Abs((RealType)pnt.Z() - _center[2]) > _hSize[2]);

    /// <summary>True if the sphere (or its surface, if hollow) does not intersect the box.</summary>
    public readonly bool IsOut(in Xyz center, double radius, bool isSphereHollow = false)
    {
        bool result = true;
        if (!isSphereHollow)
        {
            // vector from the center of the sphere to the nearest box face
            Span<double> dist =
            [
                Math.Abs(center.X() - (double)_center[0]) - (double)_hSize[0],
                Math.Abs(center.Y() - (double)_center[1]) - (double)_hSize[1],
                Math.Abs(center.Z() - (double)_center[2]) - (double)_hSize[2],
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

            if (dist[2] > 0.0)
            {
                d += dist[2] * dist[2];
            }

            result = (d > radius * radius);
        }
        else
        {
            Span<double> distC =
            [
                Math.Abs(center.X() - (double)_center[0]),
                Math.Abs(center.Y() - (double)_center[1]),
                Math.Abs(center.Z() - (double)_center[2]),
            ];
            // vector from the center of the sphere to the nearest box face
            Span<double> dist =
            [
                distC[0] - (double)_hSize[0],
                distC[1] - (double)_hSize[1],
                distC[2] - (double)_hSize[2],
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

            if (dist[2] > 0.0)
            {
                d += dist[2] * dist[2];
            }

            if (d < radius * radius)
            {
                // the box intersects the solid sphere; check if it is completely
                // inside the circle (in such case return isOut==True)
                dist[0] = distC[0] + (double)_hSize[0];
                dist[1] = distC[1] + (double)_hSize[1];
                dist[2] = distC[2] + (double)_hSize[2];
                if (dist[0] * dist[0] + dist[1] * dist[1] + dist[2] * dist[2] > radius * radius)
                {
                    result = false;
                }
            }
        }

        return result;
    }

    /// <summary>True if <paramref name="box"/> does not intersect this box.</summary>
    public readonly bool IsOut(in B3f box)
        => (Math.Abs(box._center[0] - _center[0]) > box._hSize[0] + _hSize[0]
            || Math.Abs(box._center[1] - _center[1]) > box._hSize[1] + _hSize[1]
            || Math.Abs(box._center[2] - _center[2]) > box._hSize[2] + _hSize[2]);

    /// <summary>True if <paramref name="box"/> transformed by <paramref name="trsf"/> does not intersect this box.</summary>
    public readonly bool IsOut(in B3f box, in Transform3D trsf)
    {
        bool result = false;
        TransformForm form = trsf.Form();
        double scale = trsf.ScaleFactor();
        double scaleAbs = Math.Abs(scale);
        if (form == Translation || form == Identity || form == PntMirror || form == Scale)
        {
            result =
                (Math.Abs((RealType)(box._center[0] * scale + trsf.TranslationPart().X()) - _center[0])
                     > (RealType)(box._hSize[0] * scaleAbs) + _hSize[0]
                 || Math.Abs((RealType)(box._center[1] * scale + trsf.TranslationPart().Y()) - _center[1])
                     > (RealType)(box._hSize[1] * scaleAbs) + _hSize[1]
                 || Math.Abs((RealType)(box._center[2] * scale + trsf.TranslationPart().Z()) - _center[2])
                     > (RealType)(box._hSize[2] * scaleAbs) + _hSize[2]);
        }
        else
        {
            // theBox is transformed and we check the resulting (enlarged) box against
            // 'this' box.
            Span<double> mat = stackalloc double[9];
            trsf.HVectorialPart().CopyTo(mat);
            Xyz center = new((double)box._center[0], (double)box._center[1], (double)box._center[2]);
            trsf.Transforms(ref center);
            Span<double> dist =
            [
                center.X() - (double)_center[0],
                center.Y() - (double)_center[1],
                center.Z() - (double)_center[2],
            ];
            Span<double> matAbs =
            [
                Math.Abs(mat[0]), Math.Abs(mat[1]), Math.Abs(mat[2]),
                Math.Abs(mat[3]), Math.Abs(mat[4]), Math.Abs(mat[5]),
                Math.Abs(mat[6]), Math.Abs(mat[7]), Math.Abs(mat[8]),
            ];
            if (Math.Abs(dist[0]) > (scaleAbs
                                          * (matAbs[0] * box._hSize[0] + matAbs[1] * box._hSize[1]
                                             + matAbs[2] * box._hSize[2])
                                      + (double)_hSize[0])
                || Math.Abs(dist[1]) > (scaleAbs
                                             * (matAbs[3] * box._hSize[0] + matAbs[4] * box._hSize[1]
                                                + matAbs[5] * box._hSize[2])
                                         + (double)_hSize[1])
                || Math.Abs(dist[2]) > (scaleAbs
                                             * (matAbs[6] * box._hSize[0] + matAbs[7] * box._hSize[1]
                                                + matAbs[8] * box._hSize[2])
                                         + (double)_hSize[2]))
            {
                result = true;
            }
            else
            {
                // theBox is rotated, scaled and translated. We apply the reverse
                // translation and scaling then check against the rotated box 'this'
                if ((Math.Abs(mat[0] * dist[0] + mat[3] * dist[1] + mat[6] * dist[2])
                     > box._hSize[0] * scaleAbs
                         + (matAbs[0] * _hSize[0] + matAbs[3] * _hSize[1] + matAbs[6] * _hSize[2]))
                    || (Math.Abs(mat[1] * dist[0] + mat[4] * dist[1] + mat[7] * dist[2])
                        > box._hSize[1] * scaleAbs
                            + (matAbs[1] * _hSize[0] + matAbs[4] * _hSize[1] + matAbs[7] * _hSize[2]))
                    || (Math.Abs(mat[2] * dist[0] + mat[5] * dist[1] + mat[8] * dist[2])
                        > box._hSize[2] * scaleAbs
                            + (matAbs[2] * _hSize[0] + matAbs[5] * _hSize[1] + matAbs[8] * _hSize[2])))
                {
                    result = true;
                }
            }
        }

        return result;
    }

    /// <summary>
    /// True if the line (or the ray, if <paramref name="isRay"/>) does not intersect the box enlarged by
    /// <paramref name="overthickness"/>.
    /// </summary>
    public readonly bool IsOut(in Axis3D line, bool isRay = false, double overthickness = 0.0)
    {
        double res = Constants.Resolution() * 100.0;
        if (IsVoid)
        {
            return true;
        }

        Span<double> inter0 = [-RealLast(), RealLast()];
        Span<double> inter1 = [-RealLast(), RealLast()];
        Xyz dir = line.Direction().XYZ();
        Xyz diff = new((double)_center[0] - line.Location().X(),
                           (double)_center[1] - line.Location().Y(),
                           (double)_center[2] - line.Location().Z());
        double hSize = (double)_hSize[0] + overthickness;
        if (dir.X() > res)
        {
            inter0[0] = (diff.X() - hSize) / dir.X();
            inter0[1] = (diff.X() + hSize) / dir.X();
        }
        else if (dir.X() < -res)
        {
            inter0[0] = (diff.X() + hSize) / dir.X();
            inter0[1] = (diff.X() - hSize) / dir.X();
        }
        else if (Math.Abs(diff.X()) > hSize)
        {
            return true;
        }

        hSize = (double)_hSize[1] + overthickness;
        if (dir.Y() > res)
        {
            inter1[0] = (diff.Y() - hSize) / dir.Y();
            inter1[1] = (diff.Y() + hSize) / dir.Y();
        }
        else if (dir.Y() < -res)
        {
            inter1[0] = (diff.Y() + hSize) / dir.Y();
            inter1[1] = (diff.Y() - hSize) / dir.Y();
        }
        else if (Math.Abs(diff.Y()) > hSize)
        {
            return true;
        }

        if (inter0[0] > (inter1[1] + res) || inter0[1] < (inter1[0] - res))
        {
            return true;
        }

        if (inter1[0] > inter0[0])
        {
            inter0[0] = inter1[0];
        }

        if (inter1[1] < inter0[1])
        {
            inter0[1] = inter1[1];
        }

        if (isRay && inter0[1] < -res)
        {
            return true;
        }

        hSize = (double)_hSize[2] + overthickness;
        if (dir.Z() > res)
        {
            inter1[0] = (diff.Z() - hSize) / dir.Z();
            inter1[1] = (diff.Z() + hSize) / dir.Z();
        }
        else if (dir.Z() < -res)
        {
            inter1[0] = (diff.Z() + hSize) / dir.Z();
            inter1[1] = (diff.Z() - hSize) / dir.Z();
        }
        else
        {
            return (Math.Abs(diff.Z()) > hSize);
        }

        if (isRay && inter1[1] < -res)
        {
            return true;
        }

        return (inter0[0] > (inter1[1] + res) || inter0[1] < (inter1[0] - res));
    }

    /// <summary>True if the plane does not intersect the box.</summary>
    public readonly bool IsOut(in CoordinateSystem plane)
    {
        if (IsVoid)
        {
            return true;
        }

        Xyz origin = plane.Location().XYZ();
        Xyz dir = plane.Direction().XYZ();
        Xyz boxCenter = new((double)_center[0], (double)_center[1], (double)_center[2]);
        double dist0 = (boxCenter - origin) * dir;
        double dist1 = // proj of HSize on aDir
            (double)_hSize[0] * Math.Abs(dir.X()) + (double)_hSize[1] * Math.Abs(dir.Y())
            + (double)_hSize[2] * Math.Abs(dir.Z());
        return ((dist0 + dist1) * (dist0 - dist1) > 0.0);
    }

    /// <summary>True if this box is fully inside <paramref name="box"/>.</summary>
    public readonly bool IsIn(in B3f box)
        => (Math.Abs(box._center[0] - _center[0]) < box._hSize[0] - _hSize[0]
            && Math.Abs(box._center[1] - _center[1]) < box._hSize[1] - _hSize[1]
            && Math.Abs(box._center[2] - _center[2]) < box._hSize[2] - _hSize[2]);

    /// <summary>True if this box is fully inside <paramref name="box"/> transformed by <paramref name="trsf"/>.</summary>
    public readonly bool IsIn(in B3f box, in Transform3D trsf)
    {
        bool result = false;
        TransformForm form = trsf.Form();
        double scale = trsf.ScaleFactor();
        double scaleAbs = Math.Abs(scale);
        if (form == Translation || form == Identity || form == PntMirror || form == Scale)
        {
            result =
                (Math.Abs((RealType)(box._center[0] * scale + trsf.TranslationPart().X()) - _center[0])
                     < (RealType)(box._hSize[0] * scaleAbs) - _hSize[0]
                 && Math.Abs((RealType)(box._center[1] * scale + trsf.TranslationPart().Y()) - _center[1])
                     < (RealType)(box._hSize[1] * scaleAbs) - _hSize[1]
                 && Math.Abs((RealType)(box._center[2] * scale + trsf.TranslationPart().Z()) - _center[2])
                     < (RealType)(box._hSize[2] * scaleAbs) - _hSize[2]);
        }
        else
        {
            // theBox is rotated, scaled and translated. We apply the reverse
            // translation and scaling then check against the rotated box 'this'
            Span<double> mat = stackalloc double[9];
            trsf.HVectorialPart().CopyTo(mat);
            Xyz center = new((double)box._center[0], (double)box._center[1], (double)box._center[2]);
            trsf.Transforms(ref center);
            Span<double> dist =
            [
                center.X() - (double)_center[0],
                center.Y() - (double)_center[1],
                center.Z() - (double)_center[2],
            ];
            if ((Math.Abs(mat[0] * dist[0] + mat[3] * dist[1] + mat[6] * dist[2])
                 < box._hSize[0] * scaleAbs
                     - (Math.Abs(mat[0]) * _hSize[0] + Math.Abs(mat[3]) * _hSize[1] + Math.Abs(mat[6]) * _hSize[2]))
                && (Math.Abs(mat[1] * dist[0] + mat[4] * dist[1] + mat[7] * dist[2])
                    < box._hSize[1] * scaleAbs
                        - (Math.Abs(mat[1]) * _hSize[0] + Math.Abs(mat[4]) * _hSize[1] + Math.Abs(mat[7]) * _hSize[2]))
                && (Math.Abs(mat[2] * dist[0] + mat[5] * dist[1] + mat[8] * dist[2])
                    < box._hSize[2] * scaleAbs
                        - (Math.Abs(mat[2]) * _hSize[0] + Math.Abs(mat[5]) * _hSize[1] + Math.Abs(mat[8]) * _hSize[2])))
            {
                result = true;
            }
        }

        return result;
    }

    private static bool CompareDist(ReadOnlySpan<RealType> aHSize, ReadOnlySpan<RealType> aDist)
        => (Math.Abs(aDist[0]) > aHSize[0] || Math.Abs(aDist[1]) > aHSize[1] || Math.Abs(aDist[2]) > aHSize[2]);

    private static bool CompareDistD(in Xyz aHSize, in Xyz aDist)
        => (Math.Abs(aDist.X()) > aHSize.X() || Math.Abs(aDist.Y()) > aHSize.Y() || Math.Abs(aDist.Z()) > aHSize.Z());

    // Exact equality, for use as a value

    public readonly bool Equals(B3f other)
        => _center[0].Equals(other._center[0]) && _center[1].Equals(other._center[1])
           && _center[2].Equals(other._center[2]) && _hSize[0].Equals(other._hSize[0])
           && _hSize[1].Equals(other._hSize[1]) && _hSize[2].Equals(other._hSize[2]);

    public override readonly bool Equals(object? obj) => obj is B3f other && Equals(other);

    public override readonly int GetHashCode()
        => HashCode.Combine(_center[0], _center[1], _center[2], _hSize[0], _hSize[1], _hSize[2]);

    public static bool operator ==(in B3f left, in B3f right) => left.Equals(right);

    public static bool operator !=(in B3f left, in B3f right) => !left.Equals(right);

    /// <summary>The center and the half-diagonal.</summary>
    public override readonly string ToString()
        => Constants.Format("Bnd_B3f", _center[0], _center[1], _center[2], _hSize[0], _hSize[1], _hSize[2]);
}
