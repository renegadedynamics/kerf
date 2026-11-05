// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_StartPoint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_StartPoint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Occt;

namespace Kerf.Engine.Intersection.Polyhedra;

/// <summary>
/// A point of the intersection of two triangles: its 3D point, its parameters on both surfaces, the triangles, the
/// edges it lies on (by number; -1 for a triangle's vertex, -2 for unknown) with its position on them, the angle of the
/// triangles and the section line it belongs to.
/// </summary>
/// <remarks>
/// OCCT's default start point has lambdas -1, angle -2, triangles -1, edges -2 and chain list -1;
/// <c>default(IntPolyh_StartPoint)</c> is all zeros, so ported code writes <c>new IntPolyh_StartPoint()</c>.
/// </remarks>
internal partial struct PolyhedronStartPoint : IEquatable<PolyhedronStartPoint>, IPolyhedronDump
{
    private const double MyConfusionPrecision = 10.0e-12;

    private double _x;
    private double _y;
    private double _z;
    private double _u1;
    private double _v1;
    private double _u2;
    private double _v2;
    private double _lambda1;
    private double _lambda2;
    private double _angle;
    private int _t1;
    private int _e1;
    private int _t2;
    private int _e2;
    private int _chainlist;

    public PolyhedronStartPoint()
    {
        _x = 0;
        _y = 0;
        _z = 0;
        _u1 = 0;
        _v1 = 0;
        _u2 = 0;
        _v2 = 0;
        _lambda1 = -1.0;
        _lambda2 = -1.0;
        _angle = -2.0;
        _t1 = -1;
        _e1 = -2;
        _t2 = -1;
        _e2 = -2;
        _chainlist = -1;
    }

    public PolyhedronStartPoint(double _x, double _y, double _z, double _u1, double _v1, double _u2, double _v2,
                               int _t1, int _e1, double _lambda1, int _t2, int _e2, double _lambda2, int _chainlist)
    {
        _angle = -2.0;
        this._x = _x;
        this._y = _y;
        this._z = _z;
        this._u1 = _u1;
        this._v1 = _v1;
        this._u2 = _u2;
        this._v2 = _v2;
        this._t1 = _t1;
        this._e1 = _e1;
        this._lambda1 = _lambda1;
        this._t2 = _t2;
        this._e2 = _e2;
        this._lambda2 = _lambda2;
        this._chainlist = _chainlist;
    }

    public readonly double X() => _x;

    public readonly double Y() => _y;

    public readonly double Z() => _z;

    public readonly double U1() => _u1;

    public readonly double V1() => _v1;

    public readonly double U2() => _u2;

    public readonly double V2() => _v2;

    public readonly int T1() => _t1;

    public readonly int E1() => _e1;

    public readonly double Lambda1() => _lambda1;

    public readonly int T2() => _t2;

    public readonly int E2() => _e2;

    public readonly double Lambda2() => _lambda2;

    public readonly double GetAngle() => _angle;

    public readonly int ChainList() => _chainlist;

    /// <summary>
    /// The points of the edge the start point lies on, in the given triangle, and the triangle's third point: through
    /// E1 if it is set (returns 1), else through E2 (returns 2); 0 if neither is. The outputs are left unchanged where
    /// OCCT leaves them (an edge number other than 1 to 3).
    /// </summary>
    public readonly int GetEdgePoints(in PolyhedronTriangle triangle, ref int firstEdgePoint, ref int secondEdgePoint,
                                      ref int lastPoint)
    {
        int surfID;
        if (_e1 != -1)
        {
            if (_e1 == 1)
            {
                firstEdgePoint = triangle.FirstPoint();
                secondEdgePoint = triangle.SecondPoint();
                lastPoint = triangle.ThirdPoint();
            }
            else if (_e1 == 2)
            {
                firstEdgePoint = triangle.SecondPoint();
                secondEdgePoint = triangle.ThirdPoint();
                lastPoint = triangle.FirstPoint();
            }
            else if (_e1 == 3)
            {
                firstEdgePoint = triangle.ThirdPoint();
                secondEdgePoint = triangle.FirstPoint();
                lastPoint = triangle.SecondPoint();
            }

            surfID = 1;
        }
        else if (_e2 != -1)
        {
            if (_e2 == 1)
            {
                firstEdgePoint = triangle.FirstPoint();
                secondEdgePoint = triangle.SecondPoint();
                lastPoint = triangle.ThirdPoint();
            }
            else if (_e2 == 2)
            {
                firstEdgePoint = triangle.SecondPoint();
                secondEdgePoint = triangle.ThirdPoint();
                lastPoint = triangle.FirstPoint();
            }
            else if (_e2 == 3)
            {
                firstEdgePoint = triangle.ThirdPoint();
                secondEdgePoint = triangle.FirstPoint();
                lastPoint = triangle.SecondPoint();
            }

            surfID = 2;
        }
        else
        {
            surfID = 0;
        }

        return surfID;
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetXYZ(double xx, double yy, double zz)
    {
        _x = xx;
        _y = yy;
        _z = zz;
    }

    internal void SetUV1(double uu1, double vv1)
    {
        _u1 = uu1;
        _v1 = vv1;
    }

    internal void SetUV2(double uu2, double vv2)
    {
        _u2 = uu2;
        _v2 = vv2;
    }

    internal void SetEdge1(int ie1) => _e1 = ie1;

    internal void SetLambda1(double lam1) => _lambda1 = lam1;

    internal void SetEdge2(int ie2) => _e2 = ie2;

    internal void SetLambda2(double lam2) => _lambda2 = lam2;

    internal void SetCoupleValue(int it1, int it2)
    {
        _t1 = it1;
        _t2 = it2;
    }

    internal void SetAngle(double ang) => _angle = ang;

    internal void SetChainList(int chList) => _chainlist = chList;

    public readonly PolyhedronStartPoint WithXYZ(double xx, double yy, double zz)
    {
        PolyhedronStartPoint copy = this;
        copy.SetXYZ(xx, yy, zz);
        return copy;
    }

    public readonly PolyhedronStartPoint WithUV1(double uu1, double vv1)
    {
        PolyhedronStartPoint copy = this;
        copy.SetUV1(uu1, vv1);
        return copy;
    }

    public readonly PolyhedronStartPoint WithUV2(double uu2, double vv2)
    {
        PolyhedronStartPoint copy = this;
        copy.SetUV2(uu2, vv2);
        return copy;
    }

    public readonly PolyhedronStartPoint WithEdge1(int ie1)
    {
        PolyhedronStartPoint copy = this;
        copy.SetEdge1(ie1);
        return copy;
    }

    public readonly PolyhedronStartPoint WithLambda1(double lam1)
    {
        PolyhedronStartPoint copy = this;
        copy.SetLambda1(lam1);
        return copy;
    }

    public readonly PolyhedronStartPoint WithEdge2(int ie2)
    {
        PolyhedronStartPoint copy = this;
        copy.SetEdge2(ie2);
        return copy;
    }

    public readonly PolyhedronStartPoint WithLambda2(double lam2)
    {
        PolyhedronStartPoint copy = this;
        copy.SetLambda2(lam2);
        return copy;
    }

    public readonly PolyhedronStartPoint WithCoupleValue(int it1, int it2)
    {
        PolyhedronStartPoint copy = this;
        copy.SetCoupleValue(it1, it2);
        return copy;
    }

    public readonly PolyhedronStartPoint WithAngle(double ang)
    {
        PolyhedronStartPoint copy = this;
        copy.SetAngle(ang);
        return copy;
    }

    public readonly PolyhedronStartPoint WithChainList(int chList)
    {
        PolyhedronStartPoint copy = this;
        copy.SetChainList(chList);
        return copy;
    }

    /// <summary>1 if SP is the same start point as this one (same edge and position, or the same vertex), else 0.</summary>
    /// <remarks>
    /// When both edges of this point are unknown (-2), OCCT also prints both points and "Can't Check" to standard
    /// output; the port leaves that print out (a library writes nothing by itself), the result is the same.
    /// </remarks>
    public readonly int CheckSameSP(in PolyhedronStartPoint sp)
    {
        // Returns 1 if this SP == SP
        int test = 0;
        if (((_e1 >= -1) && (_e1 == sp._e1)) || ((_e2 >= -1) && (_e2 == sp._e2)))
        {
            // The edges are defined
            if (((_lambda1 > -MyConfusionPrecision) && (Math.Abs(_lambda1 - sp._lambda1) < MyConfusionPrecision))
                || ((_lambda2 > -MyConfusionPrecision) && (Math.Abs(_lambda2 - sp._lambda2) < MyConfusionPrecision)))
            {
                test = 1;
            }
        }

        if ((test == 0) && ((_e1 == -1) || (_e2 == -1)))
        {
            // this SP is a vertex
            if ((Math.Abs(sp._u1 - _u1) < MyConfusionPrecision) && (Math.Abs(sp._v1 - _v1) < MyConfusionPrecision))
            {
                test = 1;
            }
        }
        else if ((_e1 == -2) && (_e2 == -2))
        {
            // OCCT: Dump(00200); SP.Dump(00201); printf("e1==-2 & e2==-2 Can't Check\n"); (left out, see remarks)
        }

        return test;
    }

    /// <summary>Writes the start point as OCCT's <c>Dump()</c> prints it.</summary>
    public readonly void Dump(TextWriter stream)
    {
        stream.Write($"\nPoint : x={Format.E(_x)}g y={Format.E(_y)}g z={Format.E(_z)}g "
                        + $"u1={Format.E(_u1)}g v1={Format.E(_v1)}g u2={Format.E(_u2)}g "
                        + $"v2={Format.E(_v2)}g\n");
        DumpTail(stream);
    }

    /// <summary>Writes the start point with its index, as OCCT's <c>Dump(i)</c> prints it.</summary>
    public readonly void Dump(int i, TextWriter stream)
    {
        stream.Write($"\nPoint({Format.D(i)}) : x={Format.E(_x)}g y={Format.E(_y)}g "
                        + $"z={Format.E(_z)}g u1={Format.E(_u1)}g v1={Format.E(_v1)}g "
                        + $"u2={Format.E(_u2)}g v2={Format.E(_v2)}g\n");
        DumpTail(stream);
    }

    private readonly void DumpTail(TextWriter stream)
    {
        stream.Write($"Triangle S1:{Format.D(_t1)} Edge S1:{Format.D(_e1)} "
                        + $"Lambda1:{Format.F(_lambda1)} Triangle S2:{Format.D(_t2)} "
                        + $"Edge S2:{Format.D(_e2)} Lambda2:{Format.F(_lambda2)}\n");
        stream.Write($"Angle: {Format.F(_angle)} List Number: {Format.D(_chainlist)}\n");
    }

    public readonly bool Equals(PolyhedronStartPoint other)
        => _x.Equals(other._x) && _y.Equals(other._y) && _z.Equals(other._z) && _u1.Equals(other._u1) && _v1.Equals(other._v1)
           && _u2.Equals(other._u2) && _v2.Equals(other._v2) && _lambda1.Equals(other._lambda1)
           && _lambda2.Equals(other._lambda2) && _angle.Equals(other._angle) && _t1 == other._t1 && _e1 == other._e1
           && _t2 == other._t2 && _e2 == other._e2 && _chainlist == other._chainlist;

    public override readonly bool Equals(object? obj) => obj is PolyhedronStartPoint other && Equals(other);

    public override readonly int GetHashCode()
    {
        HashCode hash = new();
        hash.Add(_x);
        hash.Add(_y);
        hash.Add(_z);
        hash.Add(_u1);
        hash.Add(_v1);
        hash.Add(_u2);
        hash.Add(_v2);
        hash.Add(_lambda1);
        hash.Add(_lambda2);
        hash.Add(_angle);
        hash.Add(_t1);
        hash.Add(_e1);
        hash.Add(_t2);
        hash.Add(_e2);
        hash.Add(_chainlist);
        return hash.ToHashCode();
    }

    public static bool operator ==(in PolyhedronStartPoint left, in PolyhedronStartPoint right) => left.Equals(right);

    public static bool operator !=(in PolyhedronStartPoint left, in PolyhedronStartPoint right) => !left.Equals(right);

    public override readonly string ToString()
        => $"IntPolyh_StartPoint({Constants.Format("xyz", _x, _y, _z)}, {Constants.Format("uv1", _u1, _v1)}, {Constants.Format("uv2", _u2, _v2)}, "
           + $"T {_t1}, {_t2}; E {_e1}, {_e2}; {Constants.Format("lambda", _lambda1, _lambda2)}, {Constants.Format("angle", _angle)}, list {_chainlist})";
}
