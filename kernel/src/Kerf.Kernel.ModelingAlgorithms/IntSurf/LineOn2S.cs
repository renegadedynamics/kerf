// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_LineOn2S.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_LineOn2S.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_LineOn2S.cxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_Allocator.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Intersection.Walking;

/// <summary>
/// The points of a walking line between two surfaces (<see cref="PntOn2S"/>, indexed from 1), with lazily
/// computed boxes of the 3D points and of the parameters on each surface.
/// </summary>
/// <remarks>
/// A working object (decision 21; plan phase 5-6, notes for porters): the walking, purging and splitting algorithms
/// edit it in place, so its mutators are public, and it is never frozen. Maps keyed by it compare by reference.
/// OCCT's allocator argument (IntSurf_Allocator, a handle to an NCollection allocator) is dropped
/// (translation rules, section 9).
/// <para>
/// The boxes follow OCCT: a box is "whole" (unknown) until one of the <c>IsOut*</c> queries computes it from all the
/// points, enlarged by 1% of its largest side; after that <see cref="Add"/>, <see cref="InsertBefore"/> and
/// <see cref="SetUV"/> grow it, and the other edits make it unknown again.
/// </para>
/// </remarks>
internal class LineOn2S : Transient
{
    private readonly NCollection_Sequence<PntOn2S> _seq;
    private Box2d _buv1;
    private Box2d _buv2;
    private Box _bxyz;

    public LineOn2S()
    {
        _seq = new NCollection_Sequence<PntOn2S>();
        _buv1 = new Box2d();
        _buv2 = new Box2d();
        _bxyz = new Box();
        _buv1.SetWhole();
        _buv2.SetWhole();
        _bxyz.SetWhole();
    }

    /// <summary>Adds a point at the end of the line.</summary>
    public void Add(in PntOn2S p)
    {
        _seq.Append(p);
        if (!_bxyz.IsWhole)
        {
            _bxyz.Add(p.Value());
        }
        if (!_buv1.IsWhole)
        {
            _buv1.Add(p.ValueOnSurface(true));
        }
        if (!_buv2.IsWhole)
        {
            _buv2.Add(p.ValueOnSurface(false));
        }
    }

    /// <summary>The number of points.</summary>
    public int PointCount => _seq.Length();

    /// <summary>The point of range <paramref name="index"/>.</summary>
    public PntOn2S Value(int index) => _seq.Value(index);

    /// <summary>Reverses the order of the points.</summary>
    public void Reverse() => _seq.Reverse();

    /// <summary>Keeps the points 1 to <paramref name="index"/> - 1 and returns the others as a new line.</summary>
    public LineOn2S Split(int index)
    {
        NCollection_Sequence<PntOn2S> ss = new();
        _seq.Split(index, ss);
        LineOn2S ns = new();
        int i;
        int leng = ss.Length();
        for (i = 1; i <= leng; i++)
        {
            ns.Add(ss[i]);
        }
        return ns;
    }

    /// <summary>Replaces the point of range <paramref name="index"/>.</summary>
    public void Value(int index, in PntOn2S p)
    {
        _seq[index] = p;
        _buv1.SetWhole();
        _buv2.SetWhole();
        _bxyz.SetWhole();
    }

    /// <summary>Sets the 3D point of the point of range <paramref name="index"/>.</summary>
    public void SetPoint(int index, in Point3D pnt)
    {
        _seq[index].SetValue(pnt);
        _buv1.SetWhole();
        _buv2.SetWhole();
        _bxyz.SetWhole();
    }

    /// <summary>Sets the parameters on one of the surfaces of the point of range <paramref name="index"/>.</summary>
    public void SetUV(int index, bool onFirst, double u, double v)
    {
        _seq[index].SetValue(onFirst, u, v);

        if (onFirst && !_buv1.IsWhole)
        {
            _buv1.Add(new Point2D(u, v));
        }
        else if (!onFirst && !_buv2.IsWhole)
        {
            _buv2.Add(new Point2D(u, v));
        }
    }

    public void Clear()
    {
        _seq.Clear();
        _buv1.SetWhole();
        _buv2.SetWhole();
        _bxyz.SetWhole();
    }

    /// <summary>Inserts a point before <paramref name="index"/>, or appends it if the index is past the end.</summary>
    public void InsertBefore(int index, in PntOn2S p)
    {
        if (index > _seq.Length())
        {
            _seq.Append(p);
        }
        else
        {
            _seq.InsertBefore(index, p);
        }

        if (!_bxyz.IsWhole)
        {
            _bxyz.Add(p.Value());
        }

        if (!_buv1.IsWhole)
        {
            _buv1.Add(p.ValueOnSurface(true));
        }

        if (!_buv2.IsWhole)
        {
            _buv2.Add(p.ValueOnSurface(false));
        }
    }

    public void RemovePoint(int index)
    {
        _seq.Remove(index);
        _buv1.SetWhole();
        _buv2.SetWhole();
        _bxyz.SetWhole();
    }

    /// <summary>True if <paramref name="p"/> is out of the box of the parameters on the first surface.</summary>
    public bool IsOutSurf1Box(in Point2D p)
    {
        if (PointCount == 0)
        {
            return false;
        }
        if (_buv1.IsWhole)
        {
            int n = PointCount;
            double pu1, pu2, pv1, pv2;
            _buv1.SetVoid();
            for (int i = 1; i <= n; i++)
            {
                _seq[i].Parameters(out pu1, out pv1, out pu2, out pv2);
                _buv1.Add(new Point2D(pu1, pv1));
            }
            _buv1.Get(out pu1, out pv1, out pu2, out pv2);
            pu2 -= pu1;
            pv2 -= pv1;
            if (pu2 > pv2)
            {
                _buv1.Enlarge(pu2 * 0.01);
            }
            else
            {
                _buv1.Enlarge(pv2 * 0.01);
            }
        }
        bool @out = _buv1.IsOut(p);
        return (@out);
    }

    /// <summary>True if <paramref name="p"/> is out of the box of the parameters on the second surface.</summary>
    public bool IsOutSurf2Box(in Point2D p)
    {
        if (PointCount == 0)
        {
            return false;
        }
        if (_buv2.IsWhole)
        {
            int n = PointCount;
            double pu1, pu2, pv1, pv2;
            _buv2.SetVoid();
            for (int i = 1; i <= n; i++)
            {
                _seq[i].Parameters(out pu1, out pv1, out pu2, out pv2);
                _buv2.Add(new Point2D(pu2, pv2));
            }
            _buv2.Get(out pu1, out pv1, out pu2, out pv2);
            pu2 -= pu1;
            pv2 -= pv1;
            if (pu2 > pv2)
            {
                _buv2.Enlarge(pu2 * 0.01);
            }
            else
            {
                _buv2.Enlarge(pv2 * 0.01);
            }
        }
        bool @out = _buv2.IsOut(p);
        return (@out);
    }

    /// <summary>True if <paramref name="p"/> is out of the box of the 3D points.</summary>
    public bool IsOutBox(in Point3D p)
    {
        if (PointCount == 0)
        {
            return false;
        }
        if (_bxyz.IsWhole)
        {
            int n = PointCount;
            _bxyz.SetVoid();
            for (int i = 1; i <= n; i++)
            {
                Point3D P = _seq[i].Value();
                _bxyz.Add(P);
            }
            _bxyz.Get(out double x0, out double y0, out double z0, out double x1, out double y1, out double z1);
            x1 -= x0;
            y1 -= y0;
            z1 -= z0;
            if (x1 > y1)
            {
                if (x1 > z1)
                {
                    _bxyz.Enlarge(x1 * 0.01);
                }
                else
                {
                    _bxyz.Enlarge(z1 * 0.01);
                }
            }
            else
            {
                if (y1 > z1)
                {
                    _bxyz.Enlarge(y1 * 0.01);
                }
                else
                {
                    _bxyz.Enlarge(z1 * 0.01);
                }
            }
        }
        bool @out = _bxyz.IsOut(p);
        return (@out);
    }
}
