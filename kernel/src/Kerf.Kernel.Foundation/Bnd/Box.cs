// Port of: src/FoundationClasses/TKMath/Bnd/Bnd_Box.hxx
// Port of: src/FoundationClasses/TKMath/Bnd/Bnd_Box.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.TransformForm;

namespace Kerf.Engine.Geometry.Bounds;

/// <summary>
/// A bounding box in 3D space, parallel to the axes of the coordinate system. It may be infinite (open) in one or
/// more directions, whole (infinite in all six), or void (empty). A gap is included on both sides in every direction
/// when the finite bounds are consulted.
/// </summary>
/// <remarks>
/// OCCT's default (and <c>new Bnd_Box()</c>) is a void box with a null gap. <c>default(Bnd_Box)</c> is not: it is a
/// non-void box reduced to the origin. Ported code must write <c>new Bnd_Box()</c>.
/// </remarks>
public partial struct Box : IEquatable<Box>, ISpatialTreeBox<Box>
{
    /// <summary>The box limits (Xmin, Xmax, Ymin, Ymax, Zmin, Zmax), gap and open directions included.</summary>
    public readonly record struct Limits(double Xmin, double Xmax, double Ymin, double Ymax, double Zmin, double Zmax);

    // Precision constant for infinite bounds
    private const double THE_BND_PRECISION_INFINITE = 1e+100;

    // Bit flags (OCCT's protected enum MaskFlags)
    private const int VoidMask = 0x01;
    private const int XminMask = 0x02;
    private const int XmaxMask = 0x04;
    private const int YminMask = 0x08;
    private const int YmaxMask = 0x10;
    private const int ZminMask = 0x20;
    private const int ZmaxMask = 0x40;
    private const int WholeMask = 0x7e;

    private double _xmin;
    private double _xmax;
    private double _ymin;
    private double _ymax;
    private double _zmin;
    private double _zmax;
    private double _gap;
    private int _flags;

    /// <summary>Creates an empty (void) box with a null gap.</summary>
    public Box()
    {
        _xmin = RealLast();
        _xmax = -RealLast();
        _ymin = RealLast();
        _ymax = -RealLast();
        _zmin = RealLast();
        _zmax = -RealLast();
        _gap = 0.0;
        _flags = VoidMask;
    }

    /// <summary>Creates a (non-void) box from its minimum and maximum corners, with a null gap.</summary>
    public Box(in Point3D min, in Point3D max)
    {
        _xmin = min.X();
        _xmax = max.X();
        _ymin = min.Y();
        _ymax = max.Y();
        _zmin = min.Z();
        _zmax = max.Z();
        _gap = 0.0;
        _flags = 0;
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetWhole() => _flags = WholeMask;

    internal void SetVoid()
    {
        _xmin = RealLast();
        _xmax = -RealLast();
        _ymin = RealLast();
        _ymax = -RealLast();
        _zmin = RealLast();
        _zmax = -RealLast();
        _gap = 0.0;
        _flags = VoidMask;
    }

    internal void Set(in Point3D p)
    {
        SetVoid();
        Add(p);
    }

    internal void Set(in Point3D p, in Direction3D d)
    {
        SetVoid();
        Add(p, d);
    }

    internal void Update(double x, double y, double z, double X, double Y, double Z)
    {
        if (IsVoid)
        {
            _xmin = x;
            _ymin = y;
            _zmin = z;
            _xmax = X;
            _ymax = Y;
            _zmax = Z;
            _flags &= ~VoidMask;
        }
        else
        {
            _xmin = Min(_xmin, x);
            _xmax = Max(_xmax, X);
            _ymin = Min(_ymin, y);
            _ymax = Max(_ymax, Y);
            _zmin = Min(_zmin, z);
            _zmax = Max(_zmax, Z);
        }
    }

    internal void Update(double x, double y, double z)
    {
        if (IsVoid)
        {
            _xmin = x;
            _ymin = y;
            _zmin = z;
            _xmax = x;
            _ymax = y;
            _zmax = z;
            _flags &= ~VoidMask;
        }
        else
        {
            _xmin = Min(_xmin, x);
            _xmax = Max(_xmax, x);
            _ymin = Min(_ymin, y);
            _ymax = Max(_ymax, y);
            _zmin = Min(_zmin, z);
            _zmax = Max(_zmax, z);
        }
    }

    internal void SetGap(double tol) => _gap = Math.Abs(tol);

    internal void Enlarge(double tol) => _gap = Max(_gap, Math.Abs(tol));

    internal void OpenXmin() => _flags |= XminMask;

    internal void OpenXmax() => _flags |= XmaxMask;

    internal void OpenYmin() => _flags |= YminMask;

    internal void OpenYmax() => _flags |= YmaxMask;

    internal void OpenZmin() => _flags |= ZminMask;

    internal void OpenZmax() => _flags |= ZmaxMask;

    // NCollection_UBTree's bounding-type contract (the tree mutates its own nodes' values through it)
    void ISpatialTreeBox<Box>.Add(in Box other) => Add(other);

    internal void Add(in Box other)
    {
        if (other.IsVoid)
        {
            return;
        }
        else if (IsVoid)
        {
            this = other;
            return;
        }

        _gap = Max(_gap, other._gap);

        if (IsWhole)
        {
            return;
        }
        else if (other.IsWhole)
        {
            SetWhole();
            return;
        }

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

        if (!IsOpenZmin)
        {
            if (other.IsOpenZmin)
            {
                OpenZmin();
            }
            else if (_zmin > other._zmin)
            {
                _zmin = other._zmin;
            }
        }

        if (!IsOpenZmax)
        {
            if (other.IsOpenZmax)
            {
                OpenZmax();
            }
            else if (_zmax < other._zmax)
            {
                _zmax = other._zmax;
            }
        }
    }

    internal void Add(in Point3D p)
    {
        p.Coord(out double x, out double y, out double z);
        Update(x, y, z);
    }

    internal void Add(in Point3D p, in Direction3D d)
    {
        Add(p);
        Add(d);
    }

    internal void Add(in Direction3D d)
    {
        d.Coord(out double dx, out double dy, out double dz);

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

        if (dz < -RealEpsilon())
        {
            OpenZmin();
        }
        else if (dz > RealEpsilon())
        {
            OpenZmax();
        }
    }

    // Copies with a mutator applied (the public way to "set")

    /// <summary>A void box: the same as <c>new Bnd_Box()</c>.</summary>
    public static Box Void() => new();

    /// <summary>A box bounding the single point <paramref name="p"/> (OCCT's <c>Set(P)</c>).</summary>
    public static Box FromPoint(in Point3D p)
    {
        var box = new Box();
        box.Set(p);
        return box;
    }

    /// <summary>A box bounding the half-line from <paramref name="p"/> along <paramref name="d"/> (OCCT's <c>Set(P, D)</c>).</summary>
    public static Box FromPoint(in Point3D p, in Direction3D d)
    {
        var box = new Box();
        box.Set(p, d);
        return box;
    }

    /// <summary>This box made infinite in all directions (its finite bounds are kept, as in OCCT).</summary>
    public readonly Box WithWhole()
    {
        Box copy = this;
        copy.SetWhole();
        return copy;
    }

    public readonly Box WithVoid() => new();

    public readonly Box WithGap(double tol)
    {
        Box copy = this;
        copy.SetGap(tol);
        return copy;
    }

    /// <summary>This box enlarged to contain the given intervals.</summary>
    public readonly Box Updated(double x, double y, double z, double X, double Y, double Z)
    {
        Box copy = this;
        copy.Update(x, y, z, X, Y, Z);
        return copy;
    }

    /// <summary>This box enlarged to contain the point (X, Y, Z).</summary>
    public readonly Box Updated(double x, double y, double z)
    {
        Box copy = this;
        copy.Update(x, y, z);
        return copy;
    }

    /// <summary>This box with its gap raised to at least |Tol|.</summary>
    public readonly Box Enlarged(double tol)
    {
        Box copy = this;
        copy.Enlarge(tol);
        return copy;
    }

    public readonly Box OpenedXmin()
    {
        Box copy = this;
        copy.OpenXmin();
        return copy;
    }

    public readonly Box OpenedXmax()
    {
        Box copy = this;
        copy.OpenXmax();
        return copy;
    }

    public readonly Box OpenedYmin()
    {
        Box copy = this;
        copy.OpenYmin();
        return copy;
    }

    public readonly Box OpenedYmax()
    {
        Box copy = this;
        copy.OpenYmax();
        return copy;
    }

    public readonly Box OpenedZmin()
    {
        Box copy = this;
        copy.OpenZmin();
        return copy;
    }

    public readonly Box OpenedZmax()
    {
        Box copy = this;
        copy.OpenZmax();
        return copy;
    }

    /// <summary>This box enlarged to contain <paramref name="other"/>.</summary>
    public readonly Box Added(in Box other)
    {
        Box copy = this;
        copy.Add(other);
        return copy;
    }

    /// <summary>This box enlarged to contain <paramref name="p"/>.</summary>
    public readonly Box Added(in Point3D p)
    {
        Box copy = this;
        copy.Add(p);
        return copy;
    }

    /// <summary>This box extended from <paramref name="p"/> in the direction <paramref name="d"/>.</summary>
    public readonly Box Added(in Point3D p, in Direction3D d)
    {
        Box copy = this;
        copy.Add(p, d);
        return copy;
    }

    /// <summary>This box extended in the direction <paramref name="d"/> (made open on that side).</summary>
    public readonly Box Added(in Direction3D d)
    {
        Box copy = this;
        copy.Add(d);
        return copy;
    }

    // Queries

    public readonly double GetGap() => _gap;

    /// <summary>The bounds, gap included; open directions give ±1e100. Throws Standard_ConstructionError if void.</summary>
    public readonly void Get(out double xmin, out double ymin, out double zmin,
                             out double xmax, out double ymax, out double zmax)
    {
        if (IsVoid)
        {
            throw new ConstructionErrorException("Bnd_Box is void");
        }

        xmin = GetXMin();
        xmax = GetXMax();
        ymin = GetYMin();
        ymax = GetYMax();
        zmin = GetZMin();
        zmax = GetZMax();
    }

    /// <summary>The bounds as a <see cref="Limits"/>, gap included; for a void box, the raw internal values.</summary>
    public readonly Limits Get() => new(GetXMin(), GetXMax(), GetYMin(), GetYMax(), GetZMin(), GetZMax());

    public readonly double GetXMin() => IsOpenXmin ? -THE_BND_PRECISION_INFINITE : _xmin - _gap;

    public readonly double GetXMax() => IsOpenXmax ? THE_BND_PRECISION_INFINITE : _xmax + _gap;

    public readonly double GetYMin() => IsOpenYmin ? -THE_BND_PRECISION_INFINITE : _ymin - _gap;

    public readonly double GetYMax() => IsOpenYmax ? THE_BND_PRECISION_INFINITE : _ymax + _gap;

    public readonly double GetZMin() => IsOpenZmin ? -THE_BND_PRECISION_INFINITE : _zmin - _gap;

    public readonly double GetZMax() => IsOpenZmax ? THE_BND_PRECISION_INFINITE : _zmax + _gap;

    /// <summary>The lower corner, gap included. Throws Standard_ConstructionError if void.</summary>
    public readonly Point3D CornerMin()
    {
        if (IsVoid)
        {
            throw new ConstructionErrorException("Bnd_Box is void");
        }

        return new Point3D(IsOpenXmin ? -THE_BND_PRECISION_INFINITE : _xmin - _gap,
                          IsOpenYmin ? -THE_BND_PRECISION_INFINITE : _ymin - _gap,
                          IsOpenZmin ? -THE_BND_PRECISION_INFINITE : _zmin - _gap);
    }

    /// <summary>The upper corner, gap included. Throws Standard_ConstructionError if void.</summary>
    public readonly Point3D CornerMax()
    {
        if (IsVoid)
        {
            throw new ConstructionErrorException("Bnd_Box is void");
        }

        return new Point3D(IsOpenXmax ? THE_BND_PRECISION_INFINITE : _xmax + _gap,
                          IsOpenYmax ? THE_BND_PRECISION_INFINITE : _ymax + _gap,
                          IsOpenZmax ? THE_BND_PRECISION_INFINITE : _zmax + _gap);
    }

    /// <summary>The center, gap included, or null if the box is void.</summary>
    public readonly Point3D? Center()
    {
        if (IsVoid)
        {
            return null;
        }

        return new Point3D(0.5 * (GetXMin() + GetXMax()),
                          0.5 * (GetYMin() + GetYMax()),
                          0.5 * (GetZMin() + GetZMax()));
    }

    /// <summary>True if the box has at least one open direction.</summary>
    public readonly bool IsOpen => (_flags & WholeMask) != 0;

    public readonly bool IsOpenXmin => (_flags & XminMask) != 0;

    public readonly bool IsOpenXmax => (_flags & XmaxMask) != 0;

    public readonly bool IsOpenYmin => (_flags & YminMask) != 0;

    public readonly bool IsOpenYmax => (_flags & YmaxMask) != 0;

    public readonly bool IsOpenZmin => (_flags & ZminMask) != 0;

    public readonly bool IsOpenZmax => (_flags & ZmaxMask) != 0;

    /// <summary>True if the box is infinite in all six directions.</summary>
    public readonly bool IsWhole => (_flags & WholeMask) == WholeMask;

    /// <summary>True if the box is empty.</summary>
    public readonly bool IsVoid => (_flags & VoidMask) != 0;

    /// <summary>True if xmax - xmin &lt; tol.</summary>
    public readonly bool IsXThin(double tol)
    {
        if (IsWhole)
        {
            return false;
        }

        if (IsVoid)
        {
            return true;
        }

        if (IsOpenXmin)
        {
            return false;
        }

        if (IsOpenXmax)
        {
            return false;
        }

        if (_xmax - _xmin < tol)
        {
            return true;
        }

        return false;
    }

    /// <summary>True if ymax - ymin &lt; tol.</summary>
    public readonly bool IsYThin(double tol)
    {
        if (IsWhole)
        {
            return false;
        }

        if (IsVoid)
        {
            return true;
        }

        if (IsOpenYmin)
        {
            return false;
        }

        if (IsOpenYmax)
        {
            return false;
        }

        if (_ymax - _ymin < tol)
        {
            return true;
        }

        return false;
    }

    /// <summary>True if zmax - zmin &lt; tol.</summary>
    public readonly bool IsZThin(double tol)
    {
        if (IsWhole)
        {
            return false;
        }

        if (IsVoid)
        {
            return true;
        }

        if (IsOpenZmin)
        {
            return false;
        }

        if (IsOpenZmax)
        {
            return false;
        }

        if (_zmax - _zmin < tol)
        {
            return true;
        }

        return false;
    }

    /// <summary>True if the box is thin in all three dimensions.</summary>
    public readonly bool IsThin(double tol)
    {
        if (IsWhole)
        {
            return false;
        }

        if (IsVoid)
        {
            return true;
        }

        if (IsOpenXmin || IsOpenXmax || _xmax - _xmin >= tol)
        {
            return false;
        }

        if (IsOpenYmin || IsOpenYmax || _ymax - _ymin >= tol)
        {
            return false;
        }

        if (IsOpenZmin || IsOpenZmax || _zmax - _zmin >= tol)
        {
            return false;
        }

        return true;
    }

    /// <summary>The box bounding this box transformed by <paramref name="t"/> (generally larger than needed).</summary>
    public readonly Box Transformed(in Transform3D t)
    {
        if (IsVoid)
        {
            return new Box();
        }
        else if (t.Form() == Identity)
        {
            return this;
        }
        else if (t.Form() == Translation)
        {
            if (!HasFinitePart)
            {
                return this;
            }

            Xyz delta = t.TranslationPart();
            Box newBox = this;
            newBox._xmin += delta.X();
            newBox._xmax += delta.X();
            newBox._ymin += delta.Y();
            newBox._ymax += delta.Y();
            newBox._zmin += delta.Z();
            newBox._zmax += delta.Z();
            return newBox;
        }

        Box newBox2 = new();
        if (HasFinitePart)
        {
            Span<Point3D> corners =
            [
                new Point3D(_xmin, _ymin, _zmin),
                new Point3D(_xmax, _ymin, _zmin),
                new Point3D(_xmin, _ymax, _zmin),
                new Point3D(_xmax, _ymax, _zmin),
                new Point3D(_xmin, _ymin, _zmax),
                new Point3D(_xmax, _ymin, _zmax),
                new Point3D(_xmin, _ymax, _zmax),
                new Point3D(_xmax, _ymax, _zmax),
            ];
            for (int cornerIter = 0; cornerIter < 8; ++cornerIter)
            {
                corners[cornerIter].Transform(t);
                newBox2.Add(corners[cornerIter]);
            }
        }

        newBox2._gap = _gap;
        if (!IsOpen)
        {
            return newBox2;
        }

        Span<Direction3D> dirs = stackalloc Direction3D[6];
        int nbDirs = 0;
        if (IsOpenXmin)
        {
            dirs[nbDirs++] = new Direction3D(Direction3D.D.NX);
        }

        if (IsOpenXmax)
        {
            dirs[nbDirs++] = new Direction3D(Direction3D.D.X);
        }

        if (IsOpenYmin)
        {
            dirs[nbDirs++] = new Direction3D(Direction3D.D.NY);
        }

        if (IsOpenYmax)
        {
            dirs[nbDirs++] = new Direction3D(Direction3D.D.Y);
        }

        if (IsOpenZmin)
        {
            dirs[nbDirs++] = new Direction3D(Direction3D.D.NZ);
        }

        if (IsOpenZmax)
        {
            dirs[nbDirs++] = new Direction3D(Direction3D.D.Z);
        }

        for (int dirIter = 0; dirIter < nbDirs; ++dirIter)
        {
            dirs[dirIter].Transform(t);
            newBox2.Add(dirs[dirIter]);
        }

        return newBox2;
    }

    /// <summary>True if the point is outside the box.</summary>
    public readonly bool IsOut(in Point3D p)
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
            p.Coord(out double x, out double y, out double z);
            if (!IsOpenXmin && (x < (_xmin - _gap)))
            {
                return true;
            }
            else if (!IsOpenXmax && (x > (_xmax + _gap)))
            {
                return true;
            }
            else if (!IsOpenYmin && (y < (_ymin - _gap)))
            {
                return true;
            }
            else if (!IsOpenYmax && (y > (_ymax + _gap)))
            {
                return true;
            }
            else if (!IsOpenZmin && (z < (_zmin - _gap)))
            {
                return true;
            }
            else if (!IsOpenZmax && (z > (_zmax + _gap)))
            {
                return true;
            }
            else
            {
                return false;
            }
        }
    }

    /// <summary>False if the plane intersects the box.</summary>
    public readonly bool IsOut(in Plane3D p)
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
            p.Coefficients(out double a, out double b, out double c, out double D);
            double xmin = GetXMin();
            double xmax = GetXMax();
            double ymin = GetYMin();
            double ymax = GetYMax();
            double zmin = GetZMin();
            double zmax = GetZMax();
            double d = a * xmin + b * ymin + c * zmin + D;
            bool plus = d > 0;
            if (plus != ((a * xmin + b * ymin + c * zmax + D) > 0))
            {
                return false;
            }

            if (plus != ((a * xmin + b * ymax + c * zmin + D) > 0))
            {
                return false;
            }

            if (plus != ((a * xmin + b * ymax + c * zmax + D) > 0))
            {
                return false;
            }

            if (plus != ((a * xmax + b * ymin + c * zmin + D) > 0))
            {
                return false;
            }

            if (plus != ((a * xmax + b * ymin + c * zmax + D) > 0))
            {
                return false;
            }

            if (plus != ((a * xmax + b * ymax + c * zmin + D) > 0))
            {
                return false;
            }

            return plus == ((a * xmax + b * ymax + c * zmax + D) > 0);
        }
    }

    /// <summary>False if the line intersects the box.</summary>
    public readonly bool IsOut(in Line3D l)
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
            double xmin = 0, xmax = 0, ymin = 0, ymax = 0, zmin = 0, zmax = 0;
            double parmin, parmax, par1, par2;
            bool xToSet, yToSet;
            Get(out double myXmin, out double myYmin, out double myZmin, out double myXmax, out double myYmax, out double myZmax);

            Xyz loc = l.Location().XYZ();
            Xyz dir = l.Direction().XYZ();
            double dirX = Math.Abs(dir.X());
            double dirY = Math.Abs(dir.Y());
            double dirZ = Math.Abs(dir.Z());

            if (dirX > 0.0)
            {
                par1 = (myXmin - loc.X()) / dir.X();
                par2 = (myXmax - loc.X()) / dir.X();
                parmin = Min(par1, par2);
                parmax = Max(par1, par2);
                xToSet = true;
            }
            else
            {
                if (loc.X() < myXmin || myXmax < loc.X())
                {
                    return true;
                }

                xmin = loc.X();
                xmax = loc.X();
                parmin = -THE_BND_PRECISION_INFINITE;
                parmax = THE_BND_PRECISION_INFINITE;
                xToSet = false;
            }

            if (dirY > 0.0)
            {
                par1 = (myYmin - loc.Y()) / dir.Y();
                par2 = (myYmax - loc.Y()) / dir.Y();
                // Check if parameter ranges from this axis are disjoint (early exit)
                if (parmax < Min(par1, par2) || parmin > Max(par1, par2))
                {
                    return true;
                }

                parmin = Max(parmin, Min(par1, par2));
                parmax = Min(parmax, Max(par1, par2));
                yToSet = true;
            }
            else
            {
                if (loc.Y() < myYmin || myYmax < loc.Y())
                {
                    return true;
                }

                ymin = loc.Y();
                ymax = loc.Y();
                yToSet = false;
            }

            if (dirZ > 0.0)
            {
                par1 = (myZmin - loc.Z()) / dir.Z();
                par2 = (myZmax - loc.Z()) / dir.Z();
                // Check if parameter ranges from this axis are disjoint (early exit)
                if (parmax < Min(par1, par2) || parmin > Max(par1, par2))
                {
                    return true;
                }

                parmin = Max(parmin, Min(par1, par2));
                parmax = Min(parmax, Max(par1, par2));
                par1 = loc.Z() + parmin * dir.Z();
                par2 = loc.Z() + parmax * dir.Z();
                zmin = Min(par1, par2);
                zmax = Max(par1, par2);
            }
            else
            {
                if (loc.Z() < myZmin || myZmax < loc.Z())
                {
                    return true;
                }

                zmin = loc.Z();
                zmax = loc.Z();
            }

            if (zmax < myZmin || myZmax < zmin)
            {
                return true;
            }

            if (xToSet)
            {
                par1 = loc.X() + parmin * dir.X();
                par2 = loc.X() + parmax * dir.X();
                xmin = Min(par1, par2);
                xmax = Max(par1, par2);
            }

            if (xmax < myXmin || myXmax < xmin)
            {
                return true;
            }

            if (yToSet)
            {
                par1 = loc.Y() + parmin * dir.Y();
                par2 = loc.Y() + parmax * dir.Y();
                ymin = Min(par1, par2);
                ymax = Max(par1, par2);
            }

            if (ymax < myYmin || myYmax < ymin)
            {
                return true;
            }
        }

        return false;
    }

    /// <summary>False if <paramref name="other"/> intersects or is inside this box.</summary>
    public readonly bool IsOut(in Box other)
    {
        // Fast path for non-open boxes with early exit
        if (_flags == 0 && other._flags == 0)
        {
            double delta0 = other._gap + _gap;
            // Early exit on first separating axis found
            if (_xmin - other._xmax > delta0)
            {
                return true;
            }

            if (other._xmin - _xmax > delta0)
            {
                return true;
            }

            if (_ymin - other._ymax > delta0)
            {
                return true;
            }

            if (other._ymin - _ymax > delta0)
            {
                return true;
            }

            if (_zmin - other._zmax > delta0)
            {
                return true;
            }

            if (other._zmin - _zmax > delta0)
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

        double delta = other._gap + _gap;

        // Check each axis with early exit
        if (!IsOpenXmin && !other.IsOpenXmax && _xmin - other._xmax > delta)
        {
            return true;
        }

        if (!IsOpenXmax && !other.IsOpenXmin && other._xmin - _xmax > delta)
        {
            return true;
        }

        if (!IsOpenYmin && !other.IsOpenYmax && _ymin - other._ymax > delta)
        {
            return true;
        }

        if (!IsOpenYmax && !other.IsOpenYmin && other._ymin - _ymax > delta)
        {
            return true;
        }

        if (!IsOpenZmin && !other.IsOpenZmax && _zmin - other._zmax > delta)
        {
            return true;
        }

        if (!IsOpenZmax && !other.IsOpenZmin && other._zmin - _zmax > delta)
        {
            return true;
        }

        return false;
    }

    /// <summary>False if <paramref name="other"/> transformed by <paramref name="t"/> intersects or is inside this box.</summary>
    public readonly bool IsOut(in Box other, in Transform3D t) => IsOut(other.Transformed(t));

    /// <summary>False if <paramref name="other"/> transformed by <paramref name="t2"/> intersects or is inside this box transformed by <paramref name="t1"/>.</summary>
    public readonly bool IsOut(in Transform3D t1, in Box other, in Transform3D t2) => Transformed(t1).IsOut(other.Transformed(t2));

    /// <summary>False if the flat band between two parallel lines through <paramref name="p1"/> and <paramref name="p2"/> with direction <paramref name="d"/> intersects the box.</summary>
    public readonly bool IsOut(in Point3D p1, in Point3D p2, in Direction3D d)
    {
        if (IsWhole)
        {
            return false;
        }
        else if (IsVoid)
        {
            return true;
        }

        double eps = RealSmall();
        Get(out double myXmin, out double myYmin, out double myZmin, out double myXmax, out double myYmax, out double myZmax);

        if (Math.Abs(d.X()) < eps && Math.Abs(d.Y()) < eps)
        {
            return IsSegmentOut(myXmin, myYmin, myXmax, myYmax, p1.X(), p1.Y(), p2.X(), p2.Y());
        }

        if (Math.Abs(d.X()) < eps && Math.Abs(d.Z()) < eps)
        {
            return IsSegmentOut(myXmin, myZmin, myXmax, myZmax, p1.X(), p1.Z(), p2.X(), p2.Z());
        }

        if (Math.Abs(d.Y()) < eps && Math.Abs(d.Z()) < eps)
        {
            return IsSegmentOut(myYmin, myZmin, myYmax, myZmax, p1.Y(), p1.Z(), p2.Y(), p2.Z());
        }

        if (Math.Abs(d.X()) < eps)
        {
            if (!IsSegmentOut(myXmin, myZmin, myXmax, myZmax,
                              p1.X(), (myYmin - p1.Y()) * d.Z() / d.Y() + p1.Z(),
                              p2.X(), (myYmin - p2.Y()) * d.Z() / d.Y() + p2.Z()))
            {
                return false;
            }

            if (!IsSegmentOut(myXmin, myZmin, myXmax, myZmax,
                              p1.X(), (myYmax - p1.Y()) * d.Z() / d.Y() + p1.Z(),
                              p2.X(), (myYmax - p2.Y()) * d.Z() / d.Y() + p2.Z()))
            {
                return false;
            }

            if (!IsSegmentOut(myXmin, myYmin, myXmax, myYmax,
                              p1.X(), (myZmin - p1.Z()) * d.Y() / d.Z() + p1.Y(),
                              p2.X(), (myZmin - p2.Z()) * d.Y() / d.Z() + p2.Y()))
            {
                return false;
            }

            if (!IsSegmentOut(myXmin, myYmin, myXmax, myYmax,
                              p1.X(), (myZmax - p1.Z()) * d.Y() / d.Z() + p1.Y(),
                              p2.X(), (myZmax - p2.Z()) * d.Y() / d.Z() + p2.Y()))
            {
                return false;
            }

            return true;
        } // if(D.X() == 0)

        if (Math.Abs(d.Y()) < eps)
        {
            if (!IsSegmentOut(myYmin, myZmin, myYmax, myZmax,
                              p1.Y(), (myXmin - p1.X()) * d.Z() / d.X() + p1.Z(),
                              p2.Y(), (myXmin - p2.X()) * d.Z() / d.X() + p2.Z()))
            {
                return false;
            }

            if (!IsSegmentOut(myYmin, myZmin, myYmax, myZmax,
                              p1.Y(), (myXmax - p1.X()) * d.Z() / d.X() + p1.Z(),
                              p2.Y(), (myXmax - p2.X()) * d.Z() / d.X() + p2.Z()))
            {
                return false;
            }

            if (!IsSegmentOut(myYmin, myXmin, myYmax, myXmax,
                              p1.Y(), (myZmin - p1.Z()) * d.X() / d.Z() + p1.X(),
                              p2.Y(), (myZmin - p2.Z()) * d.X() / d.Z() + p2.X()))
            {
                return false;
            }

            if (!IsSegmentOut(myYmin, myXmin, myYmax, myXmax,
                              p1.Y(), (myZmax - p1.Z()) * d.X() / d.Z() + p1.X(),
                              p2.Y(), (myZmax - p2.Z()) * d.X() / d.Z() + p2.X()))
            {
                return false;
            }

            return true;
        } // if(D.Y() == 0)

        if (Math.Abs(d.Z()) < eps)
        {
            if (!IsSegmentOut(myZmin, myXmin, myZmax, myXmax,
                              p1.Z(), (myYmax - p1.Y()) * d.X() / d.Y() + p1.X(),
                              p2.Z(), (myYmax - p2.Y()) * d.X() / d.Y() + p2.X()))
            {
                return false;
            }

            if (!IsSegmentOut(myZmin, myXmin, myZmax, myXmax,
                              p1.Z(), (myYmin - p1.Y()) * d.X() / d.Y() + p1.X(),
                              p2.Z(), (myYmin - p2.Y()) * d.X() / d.Y() + p2.X()))
            {
                return false;
            }

            if (!IsSegmentOut(myZmin, myYmin, myZmax, myYmax,
                              p1.Z(), (myXmax - p1.X()) * d.Y() / d.X() + p1.Y(),
                              p2.Z(), (myXmax - p2.X()) * d.Y() / d.X() + p2.Y()))
            {
                return false;
            }

            if (!IsSegmentOut(myZmin, myYmin, myZmax, myYmax,
                              p1.Z(), (myXmin - p1.X()) * d.Y() / d.X() + p1.Y(),
                              p2.Z(), (myXmin - p2.X()) * d.Y() / d.X() + p2.Y()))
            {
                return false;
            }

            return true;
        } // if(D.Z() == 0)

        if (!IsSegmentOut(myXmin, myZmin, myXmax, myZmax,
                          (myYmin - p1.Y()) / d.Y() * d.X() + p1.X(),
                          (myYmin - p1.Y()) / d.Y() * d.Z() + p1.Z(),
                          (myYmin - p2.Y()) / d.Y() * d.X() + p2.X(),
                          (myYmin - p2.Y()) / d.Y() * d.Z() + p2.Z()))
        {
            return false;
        }

        if (!IsSegmentOut(myXmin, myZmin, myXmax, myZmax,
                          (myYmax - p1.Y()) / d.Y() * d.X() + p1.X(),
                          (myYmax - p1.Y()) / d.Y() * d.Z() + p1.Z(),
                          (myYmax - p2.Y()) / d.Y() * d.X() + p2.X(),
                          (myYmax - p2.Y()) / d.Y() * d.Z() + p2.Z()))
        {
            return false;
        }

        if (!IsSegmentOut(myXmin, myYmin, myXmax, myYmax,
                          (myZmin - p1.Z()) / d.Z() * d.X() + p1.X(),
                          (myZmin - p1.Z()) / d.Z() * d.Y() + p1.Y(),
                          (myZmin - p2.Z()) / d.Z() * d.X() + p2.X(),
                          (myZmin - p2.Z()) / d.Z() * d.Y() + p2.Y()))
        {
            return false;
        }

        if (!IsSegmentOut(myXmin, myYmin, myXmax, myYmax,
                          (myZmax - p1.Z()) / d.Z() * d.X() + p1.X(),
                          (myZmax - p1.Z()) / d.Z() * d.Y() + p1.Y(),
                          (myZmax - p2.Z()) / d.Z() * d.X() + p2.X(),
                          (myZmax - p2.Z()) / d.Z() * d.Y() + p2.Y()))
        {
            return false;
        }

        if (!IsSegmentOut(myZmin, myYmin, myZmax, myYmax,
                          (myXmin - p1.X()) / d.X() * d.Z() + p1.Z(),
                          (myXmin - p1.X()) / d.X() * d.Y() + p1.Y(),
                          (myXmin - p2.X()) / d.X() * d.Z() + p2.Z(),
                          (myXmin - p2.X()) / d.X() * d.Y() + p2.Y()))
        {
            return false;
        }

        if (!IsSegmentOut(myZmin, myYmin, myZmax, myYmax,
                          (myXmax - p1.X()) / d.X() * d.Z() + p1.Z(),
                          (myXmax - p1.X()) / d.X() * d.Y() + p1.Y(),
                          (myXmax - p2.X()) / d.X() * d.Z() + p2.Z(),
                          (myXmax - p2.X()) / d.X() * d.Y() + p2.Y()))
        {
            return false;
        }

        return true;
    }

    /// <summary>True if the point is inside or on the boundary of this box.</summary>
    public readonly bool Contains(in Point3D p) => !IsOut(p);

    /// <summary>True if the other box intersects or is inside this box.</summary>
    public readonly bool Intersects(in Box other) => !IsOut(other);

    /// <summary>The minimum distance between two boxes (0 if either is void).</summary>
    public readonly double Distance(in Box other)
    {
        if (IsVoid || other.IsVoid)
        {
            return 0.0;
        }

        Get(out double xMinB1, out double yMinB1, out double zMinB1, out double xMaxB1, out double yMaxB1, out double zMaxB1);
        other.Get(out double xMinB2, out double yMinB2, out double zMinB2, out double xMaxB2, out double yMaxB2, out double zMaxB2);

        double distX = DistanceInDimension(xMinB1, xMaxB1, xMinB2, xMaxB2);
        double distY = DistanceInDimension(yMinB1, yMaxB1, yMinB2, yMaxB2);
        double distZ = DistanceInDimension(zMinB1, zMaxB1, zMinB2, zMaxB2);

        return Math.Sqrt(distX + distY + distZ);
    }

    /// <summary>The squared diagonal (0 if void).</summary>
    public readonly double SquareExtent()
    {
        if (IsVoid)
        {
            return 0.0;
        }

        double dx = _xmax - _xmin + _gap + _gap;
        double dy = _ymax - _ymin + _gap + _gap;
        double dz = _zmax - _zmin + _gap + _gap;
        return dx * dx + dy * dy + dz * dz;
    }

    /// <summary>
    /// The finite part of an infinite box (this box if already finite). Can be void if the box has open sides and
    /// no finite points. Relies on the open flags: infinite points added with Add are returned as is.
    /// </summary>
    public readonly Box FinitePart()
    {
        if (!HasFinitePart)
        {
            return new Box();
        }

        Box box = new();
        box.Update(_xmin, _ymin, _zmin, _xmax, _ymax, _zmax);
        box.SetGap(_gap);
        return box;
    }

    /// <summary>True if this box has a finite part.</summary>
    public readonly bool HasFinitePart => !IsVoid && _xmax >= _xmin;

    // Dump, DumpJson and InitFromJson are left out: ToString replaces them.

    // Anonymous-namespace helpers of Bnd_Box.cxx

    // Computes minimum squared distance between two 1D intervals
    private static double DistMini2Box(double r1Min, double r1Max, double r2Min, double r2Max)
    {
        double r1 = Square(r1Min - r2Max);
        double r2 = Square(r1Max - r2Min);
        return Min(r1, r2);
    }

    // Computes squared distance in one dimension, returns 0 if intervals overlap
    private static double DistanceInDimension(double min1, double max1, double min2, double max2)
    {
        // Check if intervals overlap
        if ((min1 <= min2 && min2 <= max1) || (min2 <= min1 && min1 <= max2))
        {
            return 0.0;
        }

        return DistMini2Box(min1, max1, min2, max2);
    }

    // Tests if a 2D segment is outside a 2D box
    private static bool IsSegmentOut(double x1, double y1, double x2, double y2,
                                     double xs1, double ys1, double xs2, double ys2)
    {
        double eps = RealSmall();
        double xsMin = Min(xs1, xs2);
        double xsMax = Max(xs1, xs2);
        double ysMin = Min(ys1, ys2);
        double ysMax = Max(ys1, ys2);

        if (ysMax - ysMin < eps && (y1 - ys1 < eps && ys1 - y2 < eps)
            && ((xsMin - x1 < eps && x1 - xsMax < eps)
                || (xsMin - x2 < eps && x2 - xsMax < eps)
                || (x1 - xs1 < eps && xs1 - x2 < eps)))
        {
            return false;
        }

        if (xsMax - xsMin < eps && (x1 - xs1 < eps && xs1 - x2 < eps)
            && ((ysMin - y1 < eps && y1 - ysMax < eps)
                || (ysMin - y2 < eps && y2 - ysMax < eps)
                || (y1 - ys1 < eps && ys1 - y2 < eps)))
        {
            return false;
        }

        if ((xs1 < x1 && xs2 < x1) || (xs1 > x2 && xs2 > x2)
            || (ys1 < y1 && ys2 < y1) || (ys1 > y2 && ys2 > y2))
        {
            return true;
        }

        if (Math.Abs(xs2 - xs1) > eps)
        {
            double ya = (Min(x1, x2) - xs1) * (ys2 - ys1) / (xs2 - xs1) + ys1;
            double yb = (Max(x1, x2) - xs1) * (ys2 - ys1) / (xs2 - xs1) + ys1;
            if ((ya < y1 && yb < y1) || (ya > y2 && yb > y2))
            {
                return true;
            }
        }
        else if (Math.Abs(ys2 - ys1) > eps)
        {
            double xa = (Min(y1, y2) - ys1) * (xs2 - xs1) / (ys2 - ys1) + xs1;
            double xb = (Max(y1, y2) - ys1) * (xs2 - xs1) / (ys2 - ys1) + xs1;
            if ((xa < x1 && xb < x1) || (xa > x2 && xb > x2))
            {
                return true;
            }
        }
        else
        {
            return true;
        }

        return false;
    }

    // Exact equality, for use as a value

    public readonly bool Equals(Box other)
        => _xmin.Equals(other._xmin) && _xmax.Equals(other._xmax) && _ymin.Equals(other._ymin) && _ymax.Equals(other._ymax)
           && _zmin.Equals(other._zmin) && _zmax.Equals(other._zmax) && _gap.Equals(other._gap) && _flags == other._flags;

    public override readonly bool Equals(object? obj) => obj is Box other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_xmin, _xmax, _ymin, _ymax, _zmin, _zmax, _gap, _flags);

    public static bool operator ==(in Box left, in Box right) => left.Equals(right);

    public static bool operator !=(in Box left, in Box right) => !left.Equals(right);

    /// <summary>The raw fields: Xmin, Xmax, Ymin, Ymax, Zmin, Zmax, Gap and the flag bits.</summary>
    public override readonly string ToString() => Constants.Format("Bnd_Box", _xmin, _xmax, _ymin, _ymax, _zmin, _zmax, _gap, _flags);
}
