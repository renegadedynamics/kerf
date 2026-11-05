// Port of: src/FoundationClasses/TKMath/Bnd/Bnd_Box2d.hxx
// Port of: src/FoundationClasses/TKMath/Bnd/Bnd_Box2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Geometry.TransformForm;

namespace Kerf.Engine.Geometry.Bounds;

/// <summary>
/// A bounding box in 2D space, parallel to the axes. It may be open (infinite) in some directions, whole or void.
/// A gap is included on both sides in every direction when the finite bounds are consulted.
/// </summary>
/// <remarks>
/// OCCT's default (and <c>new Bnd_Box2d()</c>) is a void box with a null gap. <c>default(Bnd_Box2d)</c> is not: it is
/// a non-void box reduced to the origin. Ported code must write <c>new Bnd_Box2d()</c>.
/// </remarks>
internal partial struct Box2d : IEquatable<Box2d>, ISpatialTreeBox<Box2d>
{
    /// <summary>The box limits (Xmin, Xmax, Ymin, Ymax), gap and open directions included.</summary>
    public readonly record struct Limits(double Xmin, double Xmax, double Ymin, double Ymax);

    // Precision constant for infinite bounds
    private const double THE_BND_PRECISION_INFINITE = 1e+100;

    // Bit flags (OCCT's protected enum MaskFlags)
    private const int VoidMask = 0x01;
    private const int XminMask = 0x02;
    private const int XmaxMask = 0x04;
    private const int YminMask = 0x08;
    private const int YmaxMask = 0x10;
    private const int WholeMask = 0x1e;

    private double _xmin;
    private double _xmax;
    private double _ymin;
    private double _ymax;
    private double _gap;
    private int _flags;

    /// <summary>Creates an empty (void) box with a null gap.</summary>
    public Box2d()
    {
        _xmin = RealLast();
        _xmax = -RealLast();
        _ymin = RealLast();
        _ymax = -RealLast();
        _gap = 0.0;
        _flags = VoidMask;
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetWhole() => _flags = WholeMask;

    internal void SetVoid()
    {
        _xmin = RealLast();
        _xmax = -RealLast();
        _ymin = RealLast();
        _ymax = -RealLast();
        _gap = 0.0;
        _flags = VoidMask;
    }

    internal void Set(in Point2D pnt)
    {
        _flags = VoidMask;
        _gap = 0.0;
        Add(pnt);
    }

    internal void Set(in Point2D pnt, in Direction2D dir)
    {
        _flags = VoidMask;
        _gap = 0.0;
        Add(pnt, dir);
    }

    internal void Update(double x, double y, double X, double Y)
    {
        if ((_flags & VoidMask) != 0)
        {
            _xmin = x;
            _ymin = y;
            _xmax = X;
            _ymax = Y;
            _flags &= ~VoidMask;
        }
        else
        {
            if ((_flags & XminMask) == 0)
            {
                _xmin = Min(_xmin, x);
            }

            if ((_flags & XmaxMask) == 0)
            {
                _xmax = Max(_xmax, X);
            }

            if ((_flags & YminMask) == 0)
            {
                _ymin = Min(_ymin, y);
            }

            if ((_flags & YmaxMask) == 0)
            {
                _ymax = Max(_ymax, Y);
            }
        }
    }

    internal void Update(double x, double y)
    {
        if ((_flags & VoidMask) != 0)
        {
            _xmin = x;
            _ymin = y;
            _xmax = x;
            _ymax = y;
            _flags &= ~VoidMask;
        }
        else
        {
            if ((_flags & XminMask) == 0)
            {
                _xmin = Min(_xmin, x);
            }

            if ((_flags & XmaxMask) == 0)
            {
                _xmax = Max(_xmax, x);
            }

            if ((_flags & YminMask) == 0)
            {
                _ymin = Min(_ymin, y);
            }

            if ((_flags & YmaxMask) == 0)
            {
                _ymax = Max(_ymax, y);
            }
        }
    }

    internal void SetGap(double tol) => _gap = Math.Abs(tol);

    internal void Enlarge(double tol) => _gap = Max(_gap, Math.Abs(tol));

    internal void OpenXmin() => _flags |= XminMask;

    internal void OpenXmax() => _flags |= XmaxMask;

    internal void OpenYmin() => _flags |= YminMask;

    internal void OpenYmax() => _flags |= YmaxMask;

    // NCollection_UBTree/EBTree<..., Bnd_Box2d> (IMeshData::BndBox2dTree)
    void ISpatialTreeBox<Box2d>.Add(in Box2d other) => Add(other);

    internal void Add(in Box2d other)
    {
        if (IsWhole)
        {
            return;
        }
        else if (other.IsVoid)
        {
            return;
        }
        else if (other.IsWhole)
        {
            SetWhole();
        }
        else if (IsVoid)
        {
            this = other;
        }
        else
        {
            if (!IsOpenXmin)
            {
                if (other.IsOpenXmin)
                {
                    OpenXmin();
                }
                else if (_xmin > other._xmin)
                {
                    _xmin = other._xmin;
                }
            }

            if (!IsOpenXmax)
            {
                if (other.IsOpenXmax)
                {
                    OpenXmax();
                }
                else if (_xmax < other._xmax)
                {
                    _xmax = other._xmax;
                }
            }

            if (!IsOpenYmin)
            {
                if (other.IsOpenYmin)
                {
                    OpenYmin();
                }
                else if (_ymin > other._ymin)
                {
                    _ymin = other._ymin;
                }
            }

            if (!IsOpenYmax)
            {
                if (other.IsOpenYmax)
                {
                    OpenYmax();
                }
                else if (_ymax < other._ymax)
                {
                    _ymax = other._ymax;
                }
            }

            _gap = Max(_gap, other._gap);
        }
    }

    internal void Add(in Point2D pnt) => Update(pnt.X, pnt.Y);

    internal void Add(in Point2D pnt, in Direction2D dir)
    {
        Add(pnt);
        Add(dir);
    }

    internal void Add(in Direction2D d)
    {
        double dx = d.X;
        double dy = d.Y;

        if (dx < -RealEpsilon())
        {
            OpenXmin();
        }
        else if (dx > RealEpsilon())
        {
            OpenXmax();
        }

        if (dy < -RealEpsilon())
        {
            OpenYmin();
        }
        else if (dy > RealEpsilon())
        {
            OpenYmax();
        }
    }

    // Copies with a mutator applied (the public way to "set")

    /// <summary>A void box: the same as <c>new Bnd_Box2d()</c>.</summary>
    public static Box2d Void() => new();

    /// <summary>A box bounding the single point <paramref name="pnt"/> (OCCT's <c>Set(P)</c>).</summary>
    public static Box2d FromPoint(in Point2D pnt)
    {
        var box = new Box2d();
        box.Set(pnt);
        return box;
    }

    /// <summary>A box bounding the half-line from <paramref name="pnt"/> along <paramref name="dir"/> (OCCT's <c>Set(P, D)</c>).</summary>
    public static Box2d FromPoint(in Point2D pnt, in Direction2D dir)
    {
        var box = new Box2d();
        box.Set(pnt, dir);
        return box;
    }

    /// <summary>This box made infinite in all directions (its finite bounds are kept, as in OCCT).</summary>
    public readonly Box2d WithWhole()
    {
        Box2d copy = this;
        copy.SetWhole();
        return copy;
    }

    public readonly Box2d WithVoid() => new();

    public readonly Box2d WithGap(double tol)
    {
        Box2d copy = this;
        copy.SetGap(tol);
        return copy;
    }

    public readonly Box2d Updated(double x, double y, double X, double Y)
    {
        Box2d copy = this;
        copy.Update(x, y, X, Y);
        return copy;
    }

    public readonly Box2d Updated(double x, double y)
    {
        Box2d copy = this;
        copy.Update(x, y);
        return copy;
    }

    public readonly Box2d Enlarged(double tol)
    {
        Box2d copy = this;
        copy.Enlarge(tol);
        return copy;
    }

    public readonly Box2d OpenedXmin()
    {
        Box2d copy = this;
        copy.OpenXmin();
        return copy;
    }

    public readonly Box2d OpenedXmax()
    {
        Box2d copy = this;
        copy.OpenXmax();
        return copy;
    }

    public readonly Box2d OpenedYmin()
    {
        Box2d copy = this;
        copy.OpenYmin();
        return copy;
    }

    public readonly Box2d OpenedYmax()
    {
        Box2d copy = this;
        copy.OpenYmax();
        return copy;
    }

    public readonly Box2d Added(in Box2d other)
    {
        Box2d copy = this;
        copy.Add(other);
        return copy;
    }

    public readonly Box2d Added(in Point2D pnt)
    {
        Box2d copy = this;
        copy.Add(pnt);
        return copy;
    }

    public readonly Box2d Added(in Point2D pnt, in Direction2D dir)
    {
        Box2d copy = this;
        copy.Add(pnt, dir);
        return copy;
    }

    public readonly Box2d Added(in Direction2D d)
    {
        Box2d copy = this;
        copy.Add(d);
        return copy;
    }

    // Queries

    public readonly double GetGap() => _gap;

    /// <summary>The bounds, gap included; open directions give ±1e100. Throws Standard_ConstructionError if void.</summary>
    public readonly void Get(out double x, out double y, out double xm, out double ym)
    {
        if ((_flags & VoidMask) != 0)
        {
            throw new ConstructionErrorException("Bnd_Box is void");
        }

        x = GetXMin();
        xm = GetXMax();
        y = GetYMin();
        ym = GetYMax();
    }

    /// <summary>The bounds as a <see cref="Limits"/>, gap included; for a void box, the raw internal values.</summary>
    public readonly Limits Get() => new(GetXMin(), GetXMax(), GetYMin(), GetYMax());

    public readonly double GetXMin() => (_flags & XminMask) != 0 ? -THE_BND_PRECISION_INFINITE : _xmin - _gap;

    public readonly double GetXMax() => (_flags & XmaxMask) != 0 ? THE_BND_PRECISION_INFINITE : _xmax + _gap;

    public readonly double GetYMin() => (_flags & YminMask) != 0 ? -THE_BND_PRECISION_INFINITE : _ymin - _gap;

    public readonly double GetYMax() => (_flags & YmaxMask) != 0 ? THE_BND_PRECISION_INFINITE : _ymax + _gap;

    /// <summary>The center, gap included, or null if the box is void.</summary>
    public readonly Point2D? Center()
    {
        if (IsVoid)
        {
            return null;
        }

        return new Point2D(0.5 * (GetXMin() + GetXMax()), 0.5 * (GetYMin() + GetYMax()));
    }

    public readonly bool IsOpenXmin => (_flags & XminMask) != 0;

    public readonly bool IsOpenXmax => (_flags & XmaxMask) != 0;

    public readonly bool IsOpenYmin => (_flags & YminMask) != 0;

    public readonly bool IsOpenYmax => (_flags & YmaxMask) != 0;

    public readonly bool IsWhole => (_flags & WholeMask) == WholeMask;

    public readonly bool IsVoid => (_flags & VoidMask) != 0;

    /// <summary>The box bounding this box transformed by <paramref name="t"/>.</summary>
    public readonly Box2d Transformed(in Transform2D t)
    {
        TransformForm f = t.Form;
        Box2d newBox = this;
        if (IsVoid)
        {
            return newBox;
        }

        if (f == Identity)
        {
        }
        else if (f == Translation)
        {
            t.TranslationXy().Coord(out double dx, out double dy);
            if ((_flags & XminMask) == 0)
            {
                newBox._xmin += dx;
            }

            if ((_flags & XmaxMask) == 0)
            {
                newBox._xmax += dx;
            }

            if ((_flags & YminMask) == 0)
            {
                newBox._ymin += dy;
            }

            if ((_flags & YmaxMask) == 0)
            {
                newBox._ymax += dy;
            }
        }
        else
        {
            Span<Point2D> p = stackalloc Point2D[4];
            Span<bool> vertex = stackalloc bool[4];
            vertex[0] = true;
            vertex[1] = true;
            vertex[2] = true;
            vertex[3] = true;
            Span<Direction2D> d = stackalloc Direction2D[6];
            int nbDirs = 0;

            if ((_flags & XminMask) != 0)
            {
                d[nbDirs++] = new Direction2D(Direction2D.D.NX);
                vertex[0] = vertex[2] = false;
            }

            if ((_flags & XmaxMask) != 0)
            {
                d[nbDirs++] = new Direction2D(Direction2D.D.X);
                vertex[1] = vertex[3] = false;
            }

            if ((_flags & YminMask) != 0)
            {
                d[nbDirs++] = new Direction2D(Direction2D.D.NY);
                vertex[0] = vertex[1] = false;
            }

            if ((_flags & YmaxMask) != 0)
            {
                d[nbDirs++] = new Direction2D(Direction2D.D.Y);
                vertex[2] = vertex[3] = false;
            }

            newBox.SetVoid();

            for (int i = 0; i < nbDirs; i++)
            {
                d[i] = d[i].Transformed(t);
                newBox.Add(d[i]);
            }

            p[0] = p[0].WithCoord(_xmin, _ymin);
            p[1] = p[1].WithCoord(_xmax, _ymin);
            p[2] = p[2].WithCoord(_xmin, _ymax);
            p[3] = p[3].WithCoord(_xmax, _ymax);
            if (vertex[0])
            {
                p[0] = p[0].Transformed(t);
                newBox.Add(p[0]);
            }

            if (vertex[1])
            {
                p[1] = p[1].Transformed(t);
                newBox.Add(p[1]);
            }

            if (vertex[2])
            {
                p[2] = p[2].Transformed(t);
                newBox.Add(p[2]);
            }

            if (vertex[3])
            {
                p[3] = p[3].Transformed(t);
                newBox.Add(p[3]);
            }

            newBox._gap = _gap;
        }

        return newBox;
    }

    /// <summary>True if the point is outside the box.</summary>
    public readonly bool IsOut(in Point2D p)
    {
        if (IsWhole)
        {
            return false;
        }
        else if (IsVoid)
        {
            return true;
        }
        else
        {
            double x = p.X;
            double y = p.Y;
            if ((_flags & XminMask) == 0 && (x < (_xmin - _gap)))
            {
                return true;
            }
            else if ((_flags & XmaxMask) == 0 && (x > (_xmax + _gap)))
            {
                return true;
            }
            else if ((_flags & YminMask) == 0 && (y < (_ymin - _gap)))
            {
                return true;
            }
            else if ((_flags & YmaxMask) == 0 && (y > (_ymax + _gap)))
            {
                return true;
            }
            else
            {
                return false;
            }
        }
    }

    /// <summary>True if the line does not intersect the box.</summary>
    public readonly bool IsOut(in Line2D l)
    {
        if (IsWhole)
        {
            return false;
        }

        if (IsVoid)
        {
            return true;
        }

        Get(out double xMin, out double yMin, out double xMax, out double yMax);

        Xy center = new((xMin + xMax) / 2, (yMin + yMax) / 2);
        Xy heigh = new(Math.Abs(xMax - center.X()), Math.Abs(yMax - center.Y()));

        Span<double> prod =
        [
            l.Direction.XY() ^ (center - l.Location.XY()),
            l.Direction.X * heigh.Y(),
            l.Direction.Y * heigh.X(),
        ];
        bool status = (Math.Abs(prod[0]) > (Math.Abs(prod[1]) + Math.Abs(prod[2])));
        return status;
    }

    /// <summary>True if the segment [theP0, theP1] does not intersect the box.</summary>
    public readonly bool IsOut(in Point2D p0, in Point2D p1)
    {
        if (IsWhole)
        {
            return false;
        }

        if (IsVoid)
        {
            return true;
        }

        bool status = true;
        Get(out double locXMin, out double locYMin, out double locXMax, out double locYMax);

        //// Intersect the line containing the segment.
        Xy segDelta = p1.XY() - p0.XY();

        Xy center = new((locXMin + locXMax) / 2, (locYMin + locYMax) / 2);
        Xy heigh = new(Math.Abs(locXMax - center.X()), Math.Abs(locYMax - center.Y()));

        Span<double> prod =
        [
            segDelta ^ (center - p0.XY()),
            segDelta.X() * heigh.Y(),
            segDelta.Y() * heigh.X(),
        ];

        if ((Math.Abs(prod[0]) <= (Math.Abs(prod[1]) + Math.Abs(prod[2]))))
        {
            // Intersection with line detected; check the segment as bounding box
            Xy hSeg = new(0.5 * segDelta.X(), 0.5 * segDelta.Y());
            Xy hSegAbs = new(Math.Abs(hSeg.X()), Math.Abs(hSeg.Y()));
            status = ((Math.Abs((p0.XY() + hSeg - center).X()) > (heigh + hSegAbs).X())
                       || (Math.Abs((p0.XY() + hSeg - center).Y()) > (heigh + hSegAbs).Y()));
        }

        return status;
    }

    /// <summary>True if <paramref name="other"/> does not intersect this box.</summary>
    public readonly bool IsOut(in Box2d other)
    {
        // Fast path for non-open, non-void, non-whole boxes (most common case)
        if (_flags == 0 && other._flags == 0)
        {
            double delta = other._gap + _gap;
            if (_xmin - other._xmax > delta)
            {
                return true;
            }

            if (other._xmin - _xmax > delta)
            {
                return true;
            }

            if (_ymin - other._ymax > delta)
            {
                return true;
            }

            if (other._ymin - _ymax > delta)
            {
                return true;
            }

            return false;
        }

        // Handle special cases
        if (IsVoid || other.IsVoid)
        {
            return true;
        }

        if (IsWhole || other.IsWhole)
        {
            return false;
        }

        other.Get(out double oXmin, out double oYmin, out double oXmax, out double oYmax);
        if ((_flags & XminMask) == 0 && (oXmax < (_xmin - _gap)))
        {
            return true;
        }

        if ((_flags & XmaxMask) == 0 && (oXmin > (_xmax + _gap)))
        {
            return true;
        }

        if ((_flags & YminMask) == 0 && (oYmax < (_ymin - _gap)))
        {
            return true;
        }

        if ((_flags & YmaxMask) == 0 && (oYmin > (_ymax + _gap)))
        {
            return true;
        }

        return false;
    }

    public readonly bool Contains(in Point2D p) => !IsOut(p);

    public readonly bool Intersects(in Box2d other) => !IsOut(other);

    /// <summary>The minimum distance between two boxes (0 if either is void).</summary>
    public readonly double Distance(in Box2d other)
    {
        if (IsVoid || other.IsVoid)
        {
            return 0.0;
        }

        Get(out double xMin1, out double yMin1, out double xMax1, out double yMax1);
        other.Get(out double xMin2, out double yMin2, out double xMax2, out double yMax2);

        double dx = DistAxis(xMin1, xMax1, xMin2, xMax2);
        double dy = DistAxis(yMin1, yMax1, yMin2, yMax2);

        return Math.Sqrt(dx + dy);
    }

    // Compute squared distance per axis (a lambda in OCCT)
    private static double DistAxis(double min1, double max1, double min2, double max2)
    {
        if (min1 > max2)
        {
            double d = min1 - max2;
            return d * d;
        }

        if (min2 > max1)
        {
            double d = min2 - max1;
            return d * d;
        }

        return 0.0;
    }

    public readonly bool IsOut(in Box2d other, in Transform2D trsf) => IsOut(other.Transformed(trsf));

    public readonly bool IsOut(in Transform2D t1, in Box2d other, in Transform2D t2) => Transformed(t1).IsOut(other.Transformed(t2));

    /// <summary>The squared diagonal (0 if void).</summary>
    public readonly double SquareExtent()
    {
        if (IsVoid)
        {
            return 0.0;
        }

        double dx = _xmax - _xmin + _gap + _gap;
        double dy = _ymax - _ymin + _gap + _gap;
        return dx * dx + dy * dy;
    }

    // Dump is left out: ToString replaces it.

    // Exact equality, for use as a value

    public readonly bool Equals(Box2d other)
        => _xmin.Equals(other._xmin) && _xmax.Equals(other._xmax) && _ymin.Equals(other._ymin) && _ymax.Equals(other._ymax)
           && _gap.Equals(other._gap) && _flags == other._flags;

    public override readonly bool Equals(object? obj) => obj is Box2d other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_xmin, _xmax, _ymin, _ymax, _gap, _flags);

    public static bool operator ==(in Box2d left, in Box2d right) => left.Equals(right);

    public static bool operator !=(in Box2d left, in Box2d right) => !left.Equals(right);

    /// <summary>The raw fields: Xmin, Xmax, Ymin, Ymax, Gap and the flag bits.</summary>
    public override readonly string ToString() => Constants.Format("Box2d", _xmin, _xmax, _ymin, _ymax, _gap, _flags);
}
