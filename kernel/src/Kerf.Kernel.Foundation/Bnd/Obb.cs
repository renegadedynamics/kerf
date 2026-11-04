// Port of: src/FoundationClasses/TKMath/Bnd/Bnd_OBB.hxx
// Port of: src/FoundationClasses/TKMath/Bnd/Bnd_OBB.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2017-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bvh;
using Kerf.Geometry;
using Occt;
using System.Runtime.CompilerServices;

namespace Kerf.Engine.Geometry.Bounds;

/// <summary>
/// An oriented bounding box (OBB): a centre, three axes and the halves of the three dimensions. Much tighter than an
/// axis-aligned box (AABB), so a more effective rejection test for non-interfering objects.
/// </summary>
/// <remarks>
/// OCCT's default constructor makes a void box (half-sizes -1, axes zero), and so does <c>new Bnd_OBB()</c>;
/// <c>default(Bnd_OBB)</c> has zero half-sizes and is not void. DumpJson is not ported (ToString replaces it).
/// </remarks>
internal partial struct Obb : IEquatable<Obb>
{
    /// <summary>The half-size dimensions, for deconstruction: <c>var (aHX, aHY, aHZ) = anOBB.GetHalfSizes();</c></summary>
    public readonly record struct HalfSizes(double X, double Y, double Z);

    [InlineArray(3)]
    private struct Axes3
    {
        private Xyz _element0;
    }

    [InlineArray(3)]
    private struct Dims3
    {
        private double _element0;
    }

    // Precomputed sqrt(3)
    private const double SQRT_3 = 1.7320508075688772935;

    private Xyz _center;   //!< Center of the OBB
    private Axes3 _axes;      //!< Directions of the box's axes (all vectors are already normalized)
    private Dims3 _hDims;     //!< Half-size dimensions of the OBB
    private bool _isAABox;    //!< To be set if the OBB is axis aligned box

    /// <summary>A void box.</summary>
    public Obb()
    {
        _isAABox = false;
        _hDims[0] = _hDims[1] = _hDims[2] = -1.0;
    }

    /// <summary>A box from all its defining parameters.</summary>
    public Obb(in Point3D center,
                   in Direction3D xDirection,
                   in Direction3D yDirection,
                   in Direction3D zDirection,
                   double hxSize,
                   double hySize,
                   double hzSize)
    {
        _center = center.XYZ();
        _isAABox = false;
        _axes[0] = xDirection.XYZ();
        _axes[1] = yDirection.XYZ();
        _axes[2] = zDirection.XYZ();

        // Standard_ASSERT_VOID on negative sizes: a debug-only report in OCCT, nothing here
        _hDims[0] = hxSize;
        _hDims[1] = hySize;
        _hDims[2] = hzSize;
    }

    /// <summary>A box from an axis-aligned box (void if that is void).</summary>
    public Obb(in Box box)
    {
        _isAABox = true;
        if (box.IsVoid)
        {
            _hDims[0] = _hDims[1] = _hDims[2] = -1.0;
            _isAABox = false;
            return;
        }

        box.Get(out double x1, out double y1, out double z1, out double x2, out double y2, out double z2);

        _axes[0] = new Xyz(1.0, 0.0, 0.0); // SetCoord (the analyzer treats an inline array element as a copy)
        _axes[1] = new Xyz(0.0, 1.0, 0.0); // SetCoord (the analyzer treats an inline array element as a copy)
        _axes[2] = new Xyz(0.0, 0.0, 1.0); // SetCoord (the analyzer treats an inline array element as a copy)

        _hDims[0] = 0.5 * (x2 - x1);
        _hDims[1] = 0.5 * (y2 - y1);
        _hDims[2] = 0.5 * (z2 - z1);

        _center.SetCoord(0.5 * (x2 + x1), 0.5 * (y2 + y1), 0.5 * (z2 + z1));
    }

    // Mutators (internal: see docs/translation-rules.md)

    /// <summary>
    /// Rebuilds the box to cover every point of the list. If tolerances are given (one per point), the box is enlarged
    /// by the tolerances of the points on its surface. The optimal mode checks more axes: slower, usually tighter.
    /// </summary>
    internal void ReBuild(NCollection_Array1<Point3D> listOfPoints,
                          NCollection_Array1<double>? listOfTolerances = null,
                          bool isOptimal = false)
    {
        switch (listOfPoints.Length())
        {
            case 1:
                ProcessOnePoint(listOfPoints.First());
                if (listOfTolerances is not null)
                {
                    Enlarge(listOfTolerances.First());
                }

                return;
            case 2:
            {
                double tol1 = (listOfTolerances is null) ? 0.0 : listOfTolerances.First();
                double tol2 = (listOfTolerances is null) ? 0.0 : listOfTolerances.Last();
                Xyz p1 = listOfPoints.First().XYZ(), p2 = listOfPoints.Last().XYZ();
                Xyz dp = p2 - p1;
                double dPm = dp.Modulus();
                _isAABox = false;
                _hDims[1] = _hDims[2] = Max(tol1, tol2);

                if (dPm < global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    ProcessOnePoint(new Point3D(p1));
                    // OCCT: ProcessOnePoint has just reset myHDims[1] to zero, so the tolerances are dropped
                    Enlarge(_hDims[1] + global::Kerf.Engine.Foundation.Precision.Confusion());
                    return;
                }

                _hDims[0] = 0.5 * (dPm + tol1 + tol2);
                _axes[0] = dp / dPm;
                if (Math.Abs(_axes[0].X()) > Math.Abs(_axes[0].Y()))
                {
                    // Z-coord. is maximal or X-coord. is maximal
                    _axes[1] = new Xyz(-_axes[0].Z(), 0.0, _axes[0].X()); // SetCoord (the analyzer treats an inline array element as a copy)
                }
                else
                {
                    // Z-coord. is maximal or Y-coord. is maximal
                    _axes[1] = new Xyz(0.0, -_axes[0].Z(), _axes[0].Y()); // SetCoord (the analyzer treats an inline array element as a copy)
                }

                _axes[2] = _axes[0].Crossed(_axes[1]).Normalized();
                _center = p1 + 0.5 * (dPm - tol1 + tol2) * _axes[0];
                return;
            }
            default:
                break;
        }

        OBBTool tool = new(listOfPoints, listOfTolerances, isOptimal);
        tool.ProcessDiTetrahedron();
        tool.BuildBox(ref this);
    }

    internal void SetCenter(in Point3D center) => _center = center.XYZ();

    /// <summary>Sets the X direction and half-size.</summary>
    internal void SetXComponent(in Direction3D xDirection, double hxSize)
    {
        _axes[0] = xDirection.XYZ();
        _hDims[0] = hxSize;
    }

    /// <summary>Sets the Y direction and half-size.</summary>
    internal void SetYComponent(in Direction3D yDirection, double hySize)
    {
        _axes[1] = yDirection.XYZ();
        _hDims[1] = hySize;
    }

    /// <summary>Sets the Z direction and half-size.</summary>
    internal void SetZComponent(in Direction3D zDirection, double hzSize)
    {
        _axes[2] = zDirection.XYZ();
        _hDims[2] = hzSize;
    }

    /// <summary>Clears this box.</summary>
    internal void SetVoid()
    {
        _hDims[0] = _hDims[1] = _hDims[2] = -1.0;
        _center = _axes[0] = _axes[1] = _axes[2] = new Xyz();
        _isAABox = false;
    }

    /// <summary>Sets the flag for an axis-aligned box.</summary>
    internal void SetAABox(bool flag) => _isAABox = flag;

    /// <summary>Enlarges the box by the absolute value of the gap.</summary>
    internal void Enlarge(double gapAdd)
    {
        double gap = Math.Abs(gapAdd);
        _hDims[0] += gap;
        _hDims[1] += gap;
        _hDims[2] += gap;
    }

    /// <summary>Rebuilds this box to include all the objects it was made from and the other box.</summary>
    internal void Add(in Obb other)
    {
        if (!other.IsVoid)
        {
            if (IsVoid)
            {
                _center = other._center;
                _axes[0] = other._axes[0];
                _axes[1] = other._axes[1];
                _axes[2] = other._axes[2];
                _hDims[0] = other._hDims[0];
                _hDims[1] = other._hDims[1];
                _hDims[2] = other._hDims[2];
                _isAABox = other._isAABox;
            }
            else
            {
                Point3D[] list = new Point3D[16];
                GetVertex(list.AsSpan(0, 8));
                other.GetVertex(list.AsSpan(8, 8));
                ReBuild(new NCollection_Array1<Point3D>(list, 0, 0, 15));
            }
        }
    }

    /// <summary>Rebuilds this box to include all the objects it was made from and the point.</summary>
    internal void Add(in Point3D p)
    {
        if (IsVoid)
        {
            _center = p.XYZ();
            _axes[0] = Constants.DX().XYZ();
            _axes[1] = Constants.DY().XYZ();
            _axes[2] = Constants.DZ().XYZ();
            _hDims[0] = 0.0;
            _hDims[1] = 0.0;
            _hDims[2] = 0.0;
            _isAABox = true;
        }
        else
        {
            Point3D[] list = new Point3D[9];
            GetVertex(list.AsSpan(0, 8));
            list[8] = p;
            ReBuild(new NCollection_Array1<Point3D>(list, 0, 0, 8));
        }
    }

    private void ProcessOnePoint(in Point3D p)
    {
        _isAABox = true;
        _hDims[0] = _hDims[1] = _hDims[2] = 0.0;
        _axes[0] = new Xyz(1.0, 0.0, 0.0); // SetCoord (the analyzer treats an inline array element as a copy)
        _axes[1] = new Xyz(0.0, 1.0, 0.0); // SetCoord (the analyzer treats an inline array element as a copy)
        _axes[2] = new Xyz(0.0, 0.0, 1.0); // SetCoord (the analyzer treats an inline array element as a copy)
        _center = p.XYZ();
    }

    // Copies with a mutator applied, and factories

    /// <summary>The box covering the points (see <c>ReBuild</c>).</summary>
    public static Obb FromPoints(NCollection_Array1<Point3D> listOfPoints,
                                     NCollection_Array1<double>? listOfTolerances = null,
                                     bool isOptimal = false)
    {
        Obb box = new();
        box.ReBuild(listOfPoints, listOfTolerances, isOptimal);
        return box;
    }

    /// <summary>A void box.</summary>
    public static Obb Void() => new();

    /// <summary>This box rebuilt over the points (as <c>ReBuild</c>; the flags of this box can survive in the two-point case).</summary>
    public readonly Obb ReBuilt(NCollection_Array1<Point3D> listOfPoints,
                                    NCollection_Array1<double>? listOfTolerances = null,
                                    bool isOptimal = false)
    {
        Obb copy = this;
        copy.ReBuild(listOfPoints, listOfTolerances, isOptimal);
        return copy;
    }

    public readonly Obb WithCenter(in Point3D center)
    {
        Obb copy = this;
        copy.SetCenter(center);
        return copy;
    }

    public readonly Obb WithXComponent(in Direction3D xDirection, double hxSize)
    {
        Obb copy = this;
        copy.SetXComponent(xDirection, hxSize);
        return copy;
    }

    public readonly Obb WithYComponent(in Direction3D yDirection, double hySize)
    {
        Obb copy = this;
        copy.SetYComponent(yDirection, hySize);
        return copy;
    }

    public readonly Obb WithZComponent(in Direction3D zDirection, double hzSize)
    {
        Obb copy = this;
        copy.SetZComponent(zDirection, hzSize);
        return copy;
    }

    public readonly Obb WithVoid() => new();

    public readonly Obb WithAABox(bool flag)
    {
        Obb copy = this;
        copy.SetAABox(flag);
        return copy;
    }

    public readonly Obb Enlarged(double gapAdd)
    {
        Obb copy = this;
        copy.Enlarge(gapAdd);
        return copy;
    }

    public readonly Obb Added(in Obb other)
    {
        Obb copy = this;
        copy.Add(other);
        return copy;
    }

    public readonly Obb Added(in Point3D p)
    {
        Obb copy = this;
        copy.Add(p);
        return copy;
    }

    // Queries

    /// <summary>
    /// The local coordinate system of the box: applied to the axis-aligned box (-XHSize, -YHSize, -ZHSize) to
    /// (XHSize, YHSize, ZHSize), it gives this box.
    /// </summary>
    public readonly CoordinateSystem Position() => new(new Point3D(_center), new Direction3D(ZDirection()), new Direction3D(XDirection()));

    public readonly Xyz Center() => _center;

    public readonly Xyz XDirection() => _axes[0];

    public readonly Xyz YDirection() => _axes[1];

    public readonly Xyz ZDirection() => _axes[2];

    public readonly double XHSize() => _hDims[0];

    public readonly double YHSize() => _hDims[1];

    public readonly double ZHSize() => _hDims[2];

    public readonly HalfSizes GetHalfSizes() => new(_hDims[0], _hDims[1], _hDims[2]);

    /// <summary>True if the box is empty (a negative half-size).</summary>
    public readonly bool IsVoid => (_hDims[0] < 0.0) || (_hDims[1] < 0.0) || (_hDims[2] < 0.0);

    /// <summary>True if the box is axis-aligned.</summary>
    public readonly bool IsAABox => _isAABox;

    /// <summary>
    /// The eight vertices, into a span of at least 8 points; false (and nothing written) if the box is void. Vertex i
    /// has local coordinates (±XHSize, ±YHSize, ±ZHSize), with the sign of X from bit 0 of i, Y bit 1, Z bit 2.
    /// </summary>
    public readonly bool GetVertex(Span<Point3D> p)
    {
        if (IsVoid)
        {
            return false;
        }

        p[0] = p[0].WithXYZ(_center - _hDims[0] * _axes[0] - _hDims[1] * _axes[1] - _hDims[2] * _axes[2]);
        p[1] = p[1].WithXYZ(_center + _hDims[0] * _axes[0] - _hDims[1] * _axes[1] - _hDims[2] * _axes[2]);
        p[2] = p[2].WithXYZ(_center - _hDims[0] * _axes[0] + _hDims[1] * _axes[1] - _hDims[2] * _axes[2]);
        p[3] = p[3].WithXYZ(_center + _hDims[0] * _axes[0] + _hDims[1] * _axes[1] - _hDims[2] * _axes[2]);
        p[4] = p[4].WithXYZ(_center - _hDims[0] * _axes[0] - _hDims[1] * _axes[1] + _hDims[2] * _axes[2]);
        p[5] = p[5].WithXYZ(_center + _hDims[0] * _axes[0] - _hDims[1] * _axes[1] + _hDims[2] * _axes[2]);
        p[6] = p[6].WithXYZ(_center - _hDims[0] * _axes[0] + _hDims[1] * _axes[1] + _hDims[2] * _axes[2]);
        p[7] = p[7].WithXYZ(_center + _hDims[0] * _axes[0] + _hDims[1] * _axes[1] + _hDims[2] * _axes[2]);

        return true;
    }

    /// <summary>The square of the diagonal.</summary>
    public readonly double SquareExtent()
        => 4.0 * (_hDims[0] * _hDims[0] + _hDims[1] * _hDims[1] + _hDims[2] * _hDims[2]);

    /// <summary>True if the boxes do not interfere (separating axis theorem on the 15 axes).</summary>
    public readonly bool IsOut(in Obb other)
    {
        if (IsVoid || other.IsVoid)
        {
            return true;
        }

        if (_isAABox && other._isAABox)
        {
            return (Math.Abs(other._center.X() - _center.X()) > other._hDims[0] + _hDims[0])
                   || (Math.Abs(other._center.Y() - _center.Y()) > other._hDims[1] + _hDims[1])
                   || (Math.Abs(other._center.Z() - _center.Z()) > other._hDims[2] + _hDims[2]);
        }

        // According to the Separating Axis Theorem for Oriented Bounding Boxes it is necessary to check the 15
        // separating axes (Ls): the 6 axes of the boxes and the 9 cross products of their axes. For each, the
        // "length" of box j is L(j) = Sum(myHDims[i] * |myAxes[i].Dot(Ls)|); if |(Center2 - Center1).Dot(Ls)| >
        // L(1) + L(2), the boxes do not interfere.

        // Precomputed difference between centers
        Xyz d = other._center - _center;

        // Check the axes of the this box, i.e. L is one of myAxes
        for (int i = 0; i < 3; ++i)
        {
            // Length of the second segment
            double lSegm2 = 0;
            for (int j = 0; j < 3; ++j)
            {
                lSegm2 += other._hDims[j] * Math.Abs(other._axes[j].Dot(_axes[i]));
            }

            // Distance between projected centers
            double distCC = Math.Abs(d.Dot(_axes[i]));

            if (distCC > _hDims[i] + lSegm2)
            {
                return true;
            }
        }

        // Check the axes of the Other box, i.e. L is one of theOther.myAxes
        for (int i = 0; i < 3; ++i)
        {
            // Length of the first segment
            double lSegm1 = 0.0;
            for (int j = 0; j < 3; ++j)
            {
                lSegm1 += _hDims[j] * Math.Abs(_axes[j].Dot(other._axes[i]));
            }

            // Distance between projected centers
            double distCC = Math.Abs(d.Dot(other._axes[i]));

            if (distCC > lSegm1 + other._hDims[i])
            {
                return true;
            }
        }

        double tolNull = Epsilon(1.0);

        // Check the axes produced by the cross products
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                // Separating axis
                Xyz lAxe = _axes[i].Crossed(other._axes[j]);

                double norm = lAxe.Modulus();
                if (norm < tolNull)
                {
                    continue;
                }

                lAxe.Divide(norm);

                // Length of the first segment
                double lSegm1 = 0.0;
                for (int k = 0; k < 3; ++k)
                {
                    lSegm1 += _hDims[k] * Math.Abs(_axes[k].Dot(lAxe));
                }

                // Length of the second segment
                double lSegm2 = 0.0;
                for (int k = 0; k < 3; ++k)
                {
                    lSegm2 += other._hDims[k] * Math.Abs(other._axes[k].Dot(lAxe));
                }

                // Distance between projected centers
                double distCC = Math.Abs(d.Dot(lAxe));

                if (distCC > lSegm1 + lSegm2)
                {
                    return true;
                }
            }
        }

        return false;
    }

    /// <summary>True if the point is out of the box.</summary>
    public readonly bool IsOut(in Point3D p)
    {
        // Project the point to myAxes[i] and compare the parameters with myHDims[i]
        Xyz rv = p.XYZ() - _center;

        return (Math.Abs(_axes[0].Dot(rv)) > _hDims[0]) || (Math.Abs(_axes[1].Dot(rv)) > _hDims[1])
               || (Math.Abs(_axes[2].Dot(rv)) > _hDims[2]);
    }

    /// <summary>True if the point is inside the box or on its boundary.</summary>
    public readonly bool Contains(in Point3D p) => !IsOut(p);

    /// <summary>True if the other box intersects or is inside this one.</summary>
    public readonly bool Intersects(in Obb other) => !IsOut(other);

    /// <summary>True if every vertex of the other box is inside this one.</summary>
    public readonly bool IsCompletelyInside(in Obb other)
    {
        if (IsVoid || other.IsVoid)
        {
            return false;
        }

        Span<Point3D> vert = stackalloc Point3D[8];
        other.GetVertex(vert);
        for (int i = 0; i < 8; i++)
        {
            if (IsOut(vert[i]))
            {
                return false;
            }
        }

        return true;
    }

    // Exact equality, for use as a value

    public readonly bool Equals(Obb other)
        => _center.Equals(other._center)
           && _axes[0].Equals(other._axes[0]) && _axes[1].Equals(other._axes[1]) && _axes[2].Equals(other._axes[2])
           && _hDims[0].Equals(other._hDims[0]) && _hDims[1].Equals(other._hDims[1]) && _hDims[2].Equals(other._hDims[2])
           && _isAABox == other._isAABox;

    public override readonly bool Equals(object? obj) => obj is Obb other && Equals(other);

    public override readonly int GetHashCode()
        => HashCode.Combine(_center, _axes[0], _axes[1], _axes[2], _hDims[0], _hDims[1], _hDims[2], _isAABox);

    public static bool operator ==(in Obb left, in Obb right) => left.Equals(right);

    public static bool operator !=(in Obb left, in Obb right) => !left.Equals(right);

    /// <summary>Centre, X, Y and Z directions, the three half-sizes, then 1 for an axis-aligned box (0 otherwise).</summary>
    public override readonly string ToString()
        => Constants.Format("Bnd_OBB",
                     _center.X(), _center.Y(), _center.Z(),
                     _axes[0].X(), _axes[0].Y(), _axes[0].Z(),
                     _axes[1].X(), _axes[1].Y(), _axes[1].Z(),
                     _axes[2].X(), _axes[2].Y(), _axes[2].Z(),
                     _hDims[0], _hDims[1], _hDims[2],
                     _isAABox ? 1.0 : 0.0);

    /// <summary>Selects, from the points stored in a BVH tree, the two giving the extreme projections on an axis.</summary>
    private sealed class OBB_ExtremePointsSelector
        : Traverse<double, NCollection_Vec3<double>, BoxSet<double, NCollection_Vec3<double>, Xyz>, BoundRange>
    {
        private Xyz _axis;    //!< Axis to project the points to
        private double _prmMin;  //!< Minimal projection parameter
        private double _prmMax;  //!< Maximal projection parameter
        private Xyz _pntMin;  //!< Minimal projection point
        private Xyz _pntMax;  //!< Maximal projection point

        public OBB_ExtremePointsSelector()
        {
            _prmMin = RealLast();
            _prmMax = RealFirst();
        }

        public void SetAxis(in Xyz axis) => _axis = axis;

        public void Clear()
        {
            _prmMin = RealLast();
            _prmMax = RealFirst();
        }

        public double MinPrm() => _prmMin;

        public double MaxPrm() => _prmMax;

        public Xyz MinPnt() => _pntMin;

        public Xyz MaxPnt() => _pntMax;

        public override bool RejectNode(in NCollection_Vec3<double> cMin, in NCollection_Vec3<double> cMax, out BoundRange metric)
        {
            if (_prmMin > _prmMax)
            {
                // No parameters computed yet (C++ leaves the metric as constructed: a void range)
                metric = new BoundRange();
                return false;
            }

            double prmMin = _prmMin, prmMax = _prmMax;
            bool isToReject = true;

            // Check if the current node is between already found parameters
            for (int i = 0; i < 2; ++i)
            {
                double x = i == 0 ? cMin.X() : cMax.X();
                for (int j = 0; j < 2; ++j)
                {
                    double y = j == 0 ? cMin.Y() : cMax.Y();
                    for (int k = 0; k < 2; ++k)
                    {
                        double z = k == 0 ? cMin.Z() : cMax.Z();

                        double prm = _axis.Dot(new Xyz(x, y, z));
                        if (prm < prmMin)
                        {
                            prmMin = prm;
                            isToReject = false;
                        }
                        else if (prm > prmMax)
                        {
                            prmMax = prm;
                            isToReject = false;
                        }
                    }
                }
            }

            metric = new BoundRange(prmMin, prmMax);

            return isToReject;
        }

        public override bool RejectMetric(in BoundRange metric)
        {
            if (_prmMin > _prmMax)
            {
                // no parameters computed
                return false;
            }

            double min = 0.0, max = 0.0;
            if (!metric.GetBounds(ref min, ref max))
            {
                // void metric
                return false;
            }

            // Check if the box of the branch is inside of the already computed parameters
            return min > _prmMin && max < _prmMax;
        }

        public override bool Accept(int index, in BoundRange metric)
        {
            Xyz theLeaf = _bvhSet!.Element(index);
            double prm = _axis.Dot(theLeaf);
            if (prm < _prmMin)
            {
                _prmMin = prm;
                _pntMin = theLeaf;
            }

            if (prm > _prmMax)
            {
                _prmMax = prm;
                _pntMax = theLeaf;
            }

            return true;
        }

        public override bool IsMetricBetter(in BoundRange left, in BoundRange right)
        {
            if (_prmMin > _prmMax)
            {
                // no parameters computed
                return true;
            }

            Span<double> min = stackalloc double[2];
            Span<double> max = stackalloc double[2];
            if (!left.GetBounds(ref min[0], ref max[0]) || !right.GetBounds(ref min[1], ref max[1]))
            {
                // void metrics
                return true;
            }

            // Choose branch with larger extension over computed parameters
            Span<double> ext = [0.0, 0.0];
            for (int i = 0; i < 2; ++i)
            {
                if (min[i] < _prmMin)
                {
                    ext[i] += _prmMin - min[i];
                }

                if (max[i] > _prmMax)
                {
                    ext[i] += max[i] - _prmMax;
                }
            }

            return ext[0] > ext[1];
        }
    }

    /// <summary>The tool for OBB construction (DiTO algorithm).</summary>
    private sealed class OBBTool
    {
        // Number of the initial axes.
        private const int NbInitAxes = 7;

        // Number of extremal points
        private const int NbExtremalPoints = 2 * NbInitAxes;

        private readonly NCollection_Array1<Point3D> _pntsList;               //!< The source list of points
        private readonly NCollection_Array1<double>? _listOfTolers;          //!< The array of tolerances
        private readonly int[] _triIdx = new int[5];                         //!< Points of ditetrahedron (indices in myLExtremalPoints)
        private readonly Xyz[] _lExtremalPoints = new Xyz[NbExtremalPoints]; //!< List of extremal points
        private readonly Xyz[] _axes = new Xyz[3];                     //!< The axes of the box (normalized or null)
        private double _qualityCriterion;                                     //!< The surface area of the OBB
        private readonly bool _optimal;                                       //!< Compute the OBB more tight
        private readonly BoxSet<double, NCollection_Vec3<double>, Xyz>? _pointBoxSet; //!< Point box set organized with BVH

        // Stored min/max parameters for the axes between extremal points (myParams[i][j] is [i * N + j])
        private readonly double[] _paramMin = new double[NbExtremalPoints * NbExtremalPoints];
        private readonly double[] _paramMax = new double[NbExtremalPoints * NbExtremalPoints];

        /// <summary>
        /// theL: the points; theLT: their tolerances (null: every tolerance is zero).
        /// </summary>
        public OBBTool(NCollection_Array1<Point3D> l, NCollection_Array1<double>? lt = null, bool isOptimal = false)
        {
            _pntsList = l;
            _listOfTolers = lt;
            _qualityCriterion = RealLast();
            _optimal = isOptimal;

            // Params(): (RealLast, RealFirst)
            Array.Fill(_paramMin, RealLast());
            Array.Fill(_paramMax, RealFirst());

            if (_optimal)
            {
                // Use linear builder for BVH construction with 30 elements in the leaf
                LinearBuilder<double, NCollection_Vec3<double>> lBuilder = new(30);
                _pointBoxSet = new BoxSet<double, NCollection_Vec3<double>, Xyz>(lBuilder);
                _pointBoxSet.SetSize(_pntsList.Length());

                // Add the points into Set
                for (int iP = l.Lower(); iP <= l.Upper(); ++iP)
                {
                    Point3D p = l[iP];
                    double tol = lt is not null ? lt.Value(iP) : global::Kerf.Engine.Foundation.Precision.Confusion();
                    Box<double, NCollection_Vec3<double>> box = new(new NCollection_Vec3<double>(p.X - tol, p.Y - tol, p.Z - tol),
                                                                        new NCollection_Vec3<double>(p.X + tol, p.Y + tol, p.Z + tol));
                    _pointBoxSet.Add(p.XYZ(), box);
                }

                _pointBoxSet.Build();
            }

            ComputeExtremePoints();
        }

        // Computes the extreme points on the set of initial axes
        private void ComputeExtremePoints()
        {
            // Six initial axes show great quality on the Optimal OBB, plus the performance is better, but they show
            // worse quality for the not optimal approach; OCCT uses seven.
            ReadOnlySpan<Xyz> initialAxesArray =
            [
                new Xyz(1.0, 0.0, 0.0),
                new Xyz(0.0, 1.0, 0.0),
                new Xyz(0.0, 0.0, 1.0),
                new Xyz(1.0, 1.0, 1.0) / SQRT_3,
                new Xyz(1.0, 1.0, -1.0) / SQRT_3,
                new Xyz(1.0, -1.0, 1.0) / SQRT_3,
                new Xyz(1.0, -1.0, -1.0) / SQRT_3,
            ];

            // Min and Max parameter
            Span<double> aParams = stackalloc double[NbExtremalPoints];

            // Look for the extremal points (myLExtremalPoints)
            for (int axeInd = 0, prmInd = -1; axeInd < NbInitAxes; ++axeInd)
            {
                int minInd = ++prmInd, maxInd = ++prmInd;
                aParams[minInd] = RealLast();
                aParams[maxInd] = -RealLast();
                Project(initialAxesArray[axeInd], out aParams[minInd], out aParams[maxInd], minInd, maxInd);
            }

            // For not optimal box it is necessary to compute the max axis
            // created by the maximally distant extreme points
            if (!_optimal)
            {
                for (int i = 0; i < 5; i++)
                {
                    _triIdx[i] = int.MaxValue;
                }

                // Compute myTriIdx[0] and myTriIdx[1].
                double maxSqDist = -1.0;
                for (int prmInd = 0; prmInd < NbExtremalPoints; prmInd += 2)
                {
                    Point3D p1 = new(_lExtremalPoints[prmInd]), p2 = new(_lExtremalPoints[prmInd + 1]);
                    double sqDist = p1.SquareDistanceTo(p2);
                    if (sqDist > maxSqDist)
                    {
                        maxSqDist = sqDist;
                        _triIdx[0] = prmInd;
                        _triIdx[1] = prmInd + 1;
                    }
                }

                // Compute the maximal axis orthogonal to the found one
                FillToTriangle3();
            }
        }

        // Two values of myTriIdx are known; finds myTriIdx[2], the point farthest from the line through them.
        private void FillToTriangle3()
        {
            Xyz p0 = _lExtremalPoints[_triIdx[0]];
            Xyz axis = _lExtremalPoints[_triIdx[1]] - p0;
            double maxSqDist = -1.0;
            for (int i = 0; i < NbExtremalPoints; i++)
            {
                if ((i == _triIdx[0]) || (i == _triIdx[1]))
                {
                    continue;
                }

                Xyz p = _lExtremalPoints[i];
                double distToAxe = axis.CrossSquareMagnitude(p - p0);
                if (distToAxe > maxSqDist)
                {
                    _triIdx[2] = i;
                    maxSqDist = distToAxe;
                }
            }
        }

        // Three values of myTriIdx are known; finds myTriIdx[3] and myTriIdx[4], the points farthest from the
        // triangle's plane on either side of it.
        private void FillToTriangle5(in Xyz normal, in Xyz barryCenter)
        {
            Span<double> aParams = [0.0, 0.0];
            int id3 = -1, id4 = -1;

            for (int ptIdx = 0; ptIdx < NbExtremalPoints; ptIdx++)
            {
                if ((ptIdx == _triIdx[0]) || (ptIdx == _triIdx[1]) || (ptIdx == _triIdx[2]))
                {
                    continue;
                }

                Xyz currPoint = _lExtremalPoints[ptIdx];
                double param = normal.Dot(currPoint - barryCenter);

                if (param < aParams[0])
                {
                    id3 = ptIdx;
                    aParams[0] = param;
                }
                else if (param > aParams[1])
                {
                    id4 = ptIdx;
                    aParams[1] = param;
                }
            }

            // The points must be in the different sides of the triangle plane.
            if (id3 >= 0 && aParams[0] < -global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                _triIdx[3] = id3;
            }

            if (id4 >= 0 && aParams[1] > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                _triIdx[4] = id4;
            }
        }

        // Chooses the optimal box with axes containing the normal to the triangle and an edge of it.
        private void ProcessTriangle(int idx1, int idx2, int idx3, bool isBuiltTrg)
        {
            const int nbAxes = 3;

            // All axes must be normalized in order to provide correct area computation
            // (see ComputeQuality(...) method).
            Span<int> id1 = [idx2, idx3, idx1];
            Span<int> id2 = [idx1, idx2, idx3];
            Span<Xyz> yAxis =
            [
                _lExtremalPoints[id1[0]] - _lExtremalPoints[id2[0]],
                _lExtremalPoints[id1[1]] - _lExtremalPoints[id2[1]],
                _lExtremalPoints[id1[2]] - _lExtremalPoints[id2[2]],
            ];

            // Normal to the triangle plane
            Xyz zAxis = yAxis[0].Crossed(yAxis[1]);

            double sqMod = zAxis.SquareModulus();

            if (sqMod < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
            {
                return;
            }

            zAxis.Divide(Math.Sqrt(sqMod));

            Span<Xyz> xAxis = stackalloc Xyz[nbAxes];
            Span<bool> xAxisValid = stackalloc bool[nbAxes];
            for (int i = 0; i < nbAxes; i++)
            {
                Xyz cross = yAxis[i].Crossed(zAxis);
                double crossSqMod = cross.SquareModulus();
                if (crossSqMod < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                {
                    xAxisValid[i] = false;
                    continue;
                }

                xAxis[i] = cross / Math.Sqrt(crossSqMod);
                xAxisValid[i] = true;
            }

            if (isBuiltTrg)
            {
                FillToTriangle5(zAxis, _lExtremalPoints[idx1]);
            }

            // Min and Max parameter
            const int nbPoints = 2 * nbAxes;

            // Compute Min/Max params for ZAxis
            Span<double> aParams = stackalloc double[nbPoints];
            FindMinMax(zAxis, out aParams[4], out aParams[5]); // Compute params on ZAxis once

            int minIdx = -1;
            for (int axeInd = 0; axeInd < nbAxes; axeInd++)
            {
                if (!xAxisValid[axeInd])
                {
                    continue;
                }

                Xyz ax = xAxis[axeInd];

                // Compute params on XAxis
                FindMinMax(ax, out aParams[0], out aParams[1]);

                // Compute params on YAxis checking for stored values
                ComputeParams(id1[axeInd], id2[axeInd], out aParams[2], out aParams[3]);

                double area = ComputeQuality(aParams);
                if (area < _qualityCriterion)
                {
                    _qualityCriterion = area;
                    minIdx = axeInd;
                }
            }

            if (minIdx < 0)
            {
                return;
            }

            _axes[0] = xAxis[minIdx];
            _axes[1] = yAxis[minIdx].Normalized();
            _axes[2] = zAxis;
        }

        /// <summary>The DiTO algorithm (http://www.idt.mdh.se/~tla/publ/FastOBBs.pdf).</summary>
        public void ProcessDiTetrahedron()
        {
            // To compute the optimal OBB it is necessary to check all possible axes created by the extremal points,
            // projecting all the points on each axis.
            if (_optimal)
            {
                for (int i = 0; i < NbExtremalPoints - 2; i++)
                {
                    for (int j = i + 1; j < NbExtremalPoints - 1; j++)
                    {
                        for (int k = j + 1; k < NbExtremalPoints; k++)
                        {
                            ProcessTriangle(i, j, k, false);
                        }
                    }
                }
            }
            else
            {
                // Use the standard DiTo approach
                ProcessTriangle(_triIdx[0], _triIdx[1], _triIdx[2], true);

                if (_triIdx[3] < NbExtremalPoints)
                {
                    ProcessTriangle(_triIdx[0], _triIdx[1], _triIdx[3], false);
                    ProcessTriangle(_triIdx[1], _triIdx[2], _triIdx[3], false);
                    ProcessTriangle(_triIdx[0], _triIdx[2], _triIdx[3], false);
                }

                if (_triIdx[4] < NbExtremalPoints)
                {
                    ProcessTriangle(_triIdx[0], _triIdx[1], _triIdx[4], false);
                    ProcessTriangle(_triIdx[1], _triIdx[2], _triIdx[4], false);
                    ProcessTriangle(_triIdx[0], _triIdx[2], _triIdx[4], false);
                }
            }
        }

        /// <summary>Makes the box from the computed parameters.</summary>
        public void BuildBox(ref Obb box)
        {
            box.SetVoid();

            // In fact, use Precision::SquareConfusion().
            bool isOBB = _axes[0].SquareModulus() * _axes[1].SquareModulus() * _axes[2].SquareModulus() > 1.0e-14;

            Direction3D xDir = isOBB ? new Direction3D(_axes[0]) : new Direction3D(Direction3D.D.X);
            Direction3D yDir = isOBB ? new Direction3D(_axes[1]) : new Direction3D(Direction3D.D.Y);
            Direction3D zDir = isOBB ? new Direction3D(_axes[2]) : new Direction3D(Direction3D.D.Z);

            const int nbPoints = 6;
            Span<double> aParams = stackalloc double[nbPoints];

            Xyz fCurrPoint = _pntsList.First().XYZ();

            aParams[0] = aParams[1] = fCurrPoint.Dot(xDir.XYZ());
            aParams[2] = aParams[3] = fCurrPoint.Dot(yDir.XYZ());
            aParams[4] = aParams[5] = fCurrPoint.Dot(zDir.XYZ());

            if (_listOfTolers is not null)
            {
                double tol = _listOfTolers.First();
                aParams[0] -= tol;
                aParams[1] += tol;
                aParams[2] -= tol;
                aParams[3] += tol;
                aParams[4] -= tol;
                aParams[5] += tol;
            }

            for (int i = _pntsList.Lower() + 1; i <= _pntsList.Upper(); i++)
            {
                Xyz currPoint = _pntsList[i].XYZ();
                double dx = currPoint.Dot(xDir.XYZ()), dy = currPoint.Dot(yDir.XYZ()), dz = currPoint.Dot(zDir.XYZ());

                if (_listOfTolers is null)
                {
                    SetMinMax(aParams[0..], dx);
                    SetMinMax(aParams[2..], dy);
                    SetMinMax(aParams[4..], dz);
                }
                else
                {
                    double tol = _listOfTolers.Value(i);
                    aParams[0] = Min(aParams[0], dx - tol);
                    aParams[1] = Max(aParams[1], dx + tol);
                    aParams[2] = Min(aParams[2], dy - tol);
                    aParams[3] = Max(aParams[3], dy + tol);
                    aParams[4] = Min(aParams[4], dz - tol);
                    aParams[5] = Max(aParams[5], dz + tol);
                }
            }

            // Half-sizes
            double hx = 0.5 * (aParams[1] - aParams[0]);
            double hy = 0.5 * (aParams[3] - aParams[2]);
            double hz = 0.5 * (aParams[5] - aParams[4]);

            Xyz center = 0.5
                             * ((aParams[1] + aParams[0]) * xDir.XYZ() + (aParams[3] + aParams[2]) * yDir.XYZ()
                                + (aParams[5] + aParams[4]) * zDir.XYZ());

            box.SetCenter(new Point3D(center));
            box.SetXComponent(xDir, hx);
            box.SetYComponent(yDir, hy);
            box.SetZComponent(zDir, hz);
            box.SetAABox(!isOBB);
        }

        // Half of the surface area of the box
        private static double ComputeQuality(ReadOnlySpan<double> prmArr)
        {
            double dx = prmArr[1] - prmArr[0], dy = prmArr[3] - prmArr[2], dz = prmArr[5] - prmArr[4];
            return dx * dy + dy * dz + dx * dz;
        }

        // The min and max parameters on the axis between myLExtremalPoints[theId1] and [theId2], cached.
        private void ComputeParams(int id1, int id2, out double min, out double max)
        {
            min = _paramMin[id1 * NbExtremalPoints + id2];
            max = _paramMax[id1 * NbExtremalPoints + id2];

            if (min > max)
            {
                FindMinMax((_lExtremalPoints[id1] - _lExtremalPoints[id2]).Normalized(), out min, out max);
                _paramMin[id1 * NbExtremalPoints + id2] = _paramMin[id2 * NbExtremalPoints + id1] = min;
                _paramMax[id1 * NbExtremalPoints + id2] = _paramMax[id2 * NbExtremalPoints + id1] = max;
            }
        }

        // The min and max parameters on the axis: of all the points in the optimal case, of the extreme points otherwise.
        private void FindMinMax(in Xyz axis, out double min, out double max)
        {
            min = RealLast();
            max = RealFirst();

            if (_optimal)
            {
                Project(axis, out min, out max);
            }
            else
            {
                for (int i = 0; i < NbExtremalPoints; ++i)
                {
                    double prm = axis.Dot(_lExtremalPoints[i]);
                    if (prm < min)
                    {
                        min = prm;
                    }

                    if (prm > max)
                    {
                        max = prm;
                    }
                }
            }
        }

        // Projects the points on the axis; the extreme points are written into myLExtremalPoints at the given
        // indices (OCCT's optional pointers; -1 for none).
        private void Project(in Xyz axis, out double min, out double max, int pntMin = -1, int pntMax = -1)
        {
            min = RealLast();
            max = RealFirst();

            if (_optimal)
            {
                // Project BVH
                OBB_ExtremePointsSelector extremePointsSelector = new();
                extremePointsSelector.SetBVHSet(_pointBoxSet);
                extremePointsSelector.SetAxis(axis);
                extremePointsSelector.Select();
                min = extremePointsSelector.MinPrm();
                max = extremePointsSelector.MaxPrm();
                if (pntMin >= 0)
                {
                    _lExtremalPoints[pntMin] = extremePointsSelector.MinPnt();
                }

                if (pntMax >= 0)
                {
                    _lExtremalPoints[pntMax] = extremePointsSelector.MaxPnt();
                }
            }
            else
            {
                // Project all points
                for (int iP = _pntsList.Lower(); iP <= _pntsList.Upper(); ++iP)
                {
                    Xyz point = _pntsList[iP].XYZ();
                    double prm = axis.Dot(point);
                    if (prm < min)
                    {
                        min = prm;
                        if (pntMin >= 0)
                        {
                            _lExtremalPoints[pntMin] = point;
                        }
                    }

                    if (prm > max)
                    {
                        max = prm;
                        if (pntMax >= 0)
                        {
                            _lExtremalPoints[pntMax] = point;
                        }
                    }
                }
            }
        }

        // ATTENTION!!! thePrmArr must be initialized before this method calling.
        private static void SetMinMax(Span<double> prmArr, double newParam)
        {
            if (newParam < prmArr[0])
            {
                prmArr[0] = newParam;
            }

            if (newParam > prmArr[1])
            {
                prmArr[1] = newParam;
            }
        }
    }
}
