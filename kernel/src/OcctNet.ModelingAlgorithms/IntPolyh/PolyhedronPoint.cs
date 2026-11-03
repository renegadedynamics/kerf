// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_Point.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_Point.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Intersection.Polyhedra;

/// <summary>A point on a surface, with both its 3D point and its (U, V) parameters.</summary>
/// <remarks>
/// OCCT's default point is all zeros with <c>PartOfCommon() == 1</c>; <c>default(IntPolyh_Point)</c> has 0 there, so
/// ported code writes <c>new IntPolyh_Point()</c>.
/// </remarks>
public partial struct PolyhedronPoint : IEquatable<PolyhedronPoint>, IPolyhedronDump
{
    private double _x;
    private double _y;
    private double _z;
    private double _u;
    private double _v;
    private int _poc;
    private bool _degenerated;

    /// <summary>The origin, at parameters (0, 0).</summary>
    public PolyhedronPoint()
    {
        _x = 0.0;
        _y = 0.0;
        _z = 0.0;
        _u = 0.0;
        _v = 0.0;
        _poc = 1;
        _degenerated = false;
    }

    /// <summary>The point (x, y, z) at parameters (u, v).</summary>
    public PolyhedronPoint(double x, double y, double z, double u, double v)
    {
        _x = x;
        _y = y;
        _z = z;
        _u = u;
        _v = v;
        _poc = 1;
        _degenerated = false;
    }

    public readonly double X() => _x;

    public readonly double Y() => _y;

    public readonly double Z() => _z;

    public readonly double U() => _u;

    public readonly double V() => _v;

    /// <summary>0 if the point is not common with the other surface.</summary>
    public readonly int PartOfCommon() => _poc;

    /// <summary>The degenerated flag.</summary>
    public readonly bool Degenerated() => _degenerated;

    // Mutators (internal: see docs/translation-rules.md)

    internal void Set(double x, double y, double z, double u, double v, int ii = 1)
    {
        _x = x;
        _y = y;
        _z = z;
        _u = u;
        _v = v;
        _poc = ii;
    }

    internal void SetX(double x) => _x = x;

    internal void SetY(double y) => _y = y;

    internal void SetZ(double z) => _z = z;

    internal void SetU(double u) => _u = u;

    internal void SetV(double v) => _v = v;

    internal void SetPartOfCommon(int ii) => _poc = ii;

    internal void SetDegenerated(bool flag) => _degenerated = flag;

    /// <summary>Makes this the middle of P1 and P2 in (U, V), with its 3D point on the surface.</summary>
    internal void Middle(SurfaceAdaptorBase mySurface, in PolyhedronPoint point1, in PolyhedronPoint point2)
    {
        _u = (point1.U() + point2.U()) * 0.5;
        _v = (point1.V() + point2.V()) * 0.5;

        Point3D ptXYZ = mySurface.Value(_u, _v);

        _x = ptXYZ.X();
        _y = ptXYZ.Y();
        _z = ptXYZ.Z();
    }

    /// <summary>Makes this the cross product of a and b (U, V unchanged).</summary>
    internal void Cross(in PolyhedronPoint a, in PolyhedronPoint b)
    {
        _x = a._y * b._z - a._z * b._y;
        _y = a._z * b._x - a._x * b._z;
        _z = a._x * b._y - a._y * b._x;
    }

    // The mutators' twins

    public readonly PolyhedronPoint WithValues(double x, double y, double z, double u, double v, int ii = 1)
    {
        PolyhedronPoint copy = this;
        copy.Set(x, y, z, u, v, ii);
        return copy;
    }

    public readonly PolyhedronPoint WithX(double x)
    {
        PolyhedronPoint copy = this;
        copy.SetX(x);
        return copy;
    }

    public readonly PolyhedronPoint WithY(double y)
    {
        PolyhedronPoint copy = this;
        copy.SetY(y);
        return copy;
    }

    public readonly PolyhedronPoint WithZ(double z)
    {
        PolyhedronPoint copy = this;
        copy.SetZ(z);
        return copy;
    }

    public readonly PolyhedronPoint WithU(double u)
    {
        PolyhedronPoint copy = this;
        copy.SetU(u);
        return copy;
    }

    public readonly PolyhedronPoint WithV(double v)
    {
        PolyhedronPoint copy = this;
        copy.SetV(v);
        return copy;
    }

    public readonly PolyhedronPoint WithPartOfCommon(int ii)
    {
        PolyhedronPoint copy = this;
        copy.SetPartOfCommon(ii);
        return copy;
    }

    public readonly PolyhedronPoint WithDegenerated(bool flag)
    {
        PolyhedronPoint copy = this;
        copy.SetDegenerated(flag);
        return copy;
    }

    /// <summary>The middle of P1 and P2 in (U, V), on the surface; the twin of <c>Middle</c>.</summary>
    public readonly PolyhedronPoint Middled(SurfaceAdaptorBase mySurface, in PolyhedronPoint point1, in PolyhedronPoint point2)
    {
        PolyhedronPoint copy = this;
        copy.Middle(mySurface, point1, point2);
        return copy;
    }

    /// <summary>This point with the cross product of a and b as coordinates; the twin of <c>Cross</c>.</summary>
    public readonly PolyhedronPoint Crossed(in PolyhedronPoint a, in PolyhedronPoint b)
    {
        PolyhedronPoint copy = this;
        copy.Cross(a, b);
        return copy;
    }

    // Arithmetic: all five coordinates; the result has OCCT's default flags

    public readonly PolyhedronPoint Add(in PolyhedronPoint p1)
    {
        PolyhedronPoint res = new();
        res.SetX(_x + p1.X());
        res.SetY(_y + p1.Y());
        res.SetZ(_z + p1.Z());
        res.SetU(_u + p1.U());
        res.SetV(_v + p1.V());
        return res;
    }

    public readonly PolyhedronPoint Sub(in PolyhedronPoint p1)
    {
        PolyhedronPoint res = new();
        res.SetX(_x - p1.X());
        res.SetY(_y - p1.Y());
        res.SetZ(_z - p1.Z());
        res.SetU(_u - p1.U());
        res.SetV(_v - p1.V());
        return res;
    }

    /// <summary>The point divided by RR; the default point if |RR| is not above <c>Precision::Computational()</c>.</summary>
    public readonly PolyhedronPoint Divide(double rr)
    {
        PolyhedronPoint res = new();
        if (Math.Abs(rr) > global::Kerf.Engine.Foundation.Precision.Computational())
        {
            res.SetX(_x / rr);
            res.SetY(_y / rr);
            res.SetZ(_z / rr);
            res.SetU(_u / rr);
            res.SetV(_v / rr);
        }

        return res;
    }

    public readonly PolyhedronPoint Multiplication(double rr)
    {
        PolyhedronPoint res = new();
        res.SetX(_x * rr);
        res.SetY(_y * rr);
        res.SetZ(_z * rr);
        res.SetU(_u * rr);
        res.SetV(_v * rr);
        return res;
    }

    public static PolyhedronPoint operator +(in PolyhedronPoint left, in PolyhedronPoint right) => left.Add(right);

    public static PolyhedronPoint operator -(in PolyhedronPoint left, in PolyhedronPoint right) => left.Sub(right);

    public static PolyhedronPoint operator /(in PolyhedronPoint left, double rr) => left.Divide(rr);

    public static PolyhedronPoint operator *(in PolyhedronPoint left, double rr) => left.Multiplication(rr);

    /// <summary>The square modulus of the 3D point.</summary>
    public readonly double SquareModulus()
    {
        double res = _x * _x + _y * _y + _z * _z;
        return res;
    }

    /// <summary>The square distance between the 3D points.</summary>
    public readonly double SquareDistance(in PolyhedronPoint p2)
    {
        double res = (_x - p2._x) * (_x - p2._x) + (_y - p2._y) * (_y - p2._y)
                     + (_z - p2._z) * (_z - p2._z);
        return res;
    }

    /// <summary>The dot product of the 3D points.</summary>
    public readonly double Dot(in PolyhedronPoint b)
    {
        double t = _x * b._x + _y * b._y + _z * b._z;
        return t;
    }

    /// <summary>Writes the point as OCCT's <c>Dump()</c> prints it.</summary>
    public readonly void Dump(TextWriter stream)
    {
        stream.Write($"\nPoint : x={Format.E(_x)}g y={Format.E(_y)}g z={Format.E(_z)}g "
                        + $"u={Format.E(_u)}g v={Format.E(_v)}g\n");
    }

    /// <summary>Writes the point with its index, as OCCT's <c>Dump(i)</c> prints it.</summary>
    public readonly void Dump(int i, TextWriter stream)
    {
        stream.Write($"\nPoint({Format.D(i, 3)}) : x={Format.E(_x)}g y={Format.E(_y)}g "
                        + $"z={Format.E(_z)}g u={Format.E(_u)}g v={Format.E(_v)}g "
                        + $"poc={Format.D(_poc, 3)}\n");
    }

    public readonly bool Equals(PolyhedronPoint other)
        => _x.Equals(other._x) && _y.Equals(other._y) && _z.Equals(other._z) && _u.Equals(other._u)
           && _v.Equals(other._v) && _poc == other._poc && _degenerated == other._degenerated;

    public override readonly bool Equals(object? obj) => obj is PolyhedronPoint other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_x, _y, _z, _u, _v, _poc, _degenerated);

    public static bool operator ==(in PolyhedronPoint left, in PolyhedronPoint right) => left.Equals(right);

    public static bool operator !=(in PolyhedronPoint left, in PolyhedronPoint right) => !left.Equals(right);

    public override readonly string ToString() => Constants.Format("IntPolyh_Point", _x, _y, _z, _u, _v);
}
