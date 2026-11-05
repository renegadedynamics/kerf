// Port of: src/ModelingData/TKGeomBase/GProp/GProp_PEquation.hxx
// Port of: src/ModelingData/TKGeomBase/GProp/GProp_PEquation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Measurement;

/// <summary>
/// Decides whether a set of 3D points is coincident, collinear, coplanar or spans space, within a tolerance, by
/// principal-axis analysis (eigendecomposition of the inertia matrix), and returns the fitted point, line, plane or
/// box. The barycentre, principal axes and extents are available whatever the result.
/// </summary>
/// <remarks>Computed in the constructor and immutable afterwards.</remarks>
internal sealed class PEquation
{
    /// <summary>Type of geometric entity that best fits the cloud.</summary>
    public enum Type
    {
        /// <summary>Not computed (the eigendecomposition failed).</summary>
        None,

        /// <summary>All points are coincident within tolerance.</summary>
        Point,

        /// <summary>Points are collinear within tolerance.</summary>
        Line,

        /// <summary>Points are coplanar within tolerance.</summary>
        Plane,

        /// <summary>Points span 3D space.</summary>
        Space,
    }

    private readonly Type _type = Type.None;
    private readonly Point3D _g;  // Type-specific point (mean point / box corner)
    private readonly Vector3D _v1; // Plane normal / line direction / box edge 1
    private readonly Vector3D _v2; // Box edge 2 (Space type only)
    private readonly Vector3D _v3; // Box edge 3 (Space type only)

    private readonly Point3D _barycentre;       // Centre of mass, always valid
    private readonly Vector3D[] _axes = new Vector3D[3]; // Principal axis unit vectors
    private readonly double[] _extents = new double[3]; // Extent along each principal axis

    /// <summary>Analyses <paramref name="pnts"/> with the tolerance <paramref name="tol"/>.</summary>
    public PEquation(NCollection_Array1<Point3D> pnts, double tol)
    {
        var props = new PGProps(pnts);
        _barycentre = props.CentreOfMass();
        _g = _barycentre;

        Matrix3x3 inertia = props.MatrixOfInertia();

        var mat = new Matrix(1, 3, 1, 3);
        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                mat[i, j] = inertia.Value(i, j);
            }
        }

        global::Kerf.Engine.Numerics.Utilities.MathUtils.EigenResult eigen = global::Kerf.Engine.Numerics.Linear.MathLin.Jacobi(mat, true);
        if (!eigen.IsDone)
        {
            _type = Type.None;
            _axes[0] = new Vector3D(1.0, 0.0, 0.0);
            _axes[1] = new Vector3D(0.0, 1.0, 0.0);
            _axes[2] = new Vector3D(0.0, 0.0, 1.0);
            _extents[0] = 0.0;
            _extents[1] = 0.0;
            _extents[2] = 0.0;
            return;
        }

        Matrix vecs = eigen.EigenVectors!;
        _axes[0] = new Vector3D(vecs[1, 1], vecs[2, 1], vecs[3, 1]);
        _axes[1] = new Vector3D(vecs[1, 2], vecs[2, 2], vecs[3, 2]);
        _axes[2] = new Vector3D(vecs[1, 3], vecs[2, 3], vecs[3, 3]);

        double xg = _g.X;
        double yg = _g.Y;
        double zg = _g.Z;

        double max1 = RealFirst(), min1 = RealLast();
        double max2 = RealFirst(), min2 = RealLast();
        double max3 = RealFirst(), min3 = RealLast();

        for (int i = pnts.Lower(); i <= pnts.Upper(); ++i)
        {
            double dx = pnts[i].X - xg;
            double dy = pnts[i].Y - yg;
            double dz = pnts[i].Z - zg;

            double d1 = dx * _axes[0].X + dy * _axes[0].Y + dz * _axes[0].Z;
            if (d1 > max1)
            {
                max1 = d1;
            }
            if (d1 < min1)
            {
                min1 = d1;
            }

            double d2 = dx * _axes[1].X + dy * _axes[1].Y + dz * _axes[1].Z;
            if (d2 > max2)
            {
                max2 = d2;
            }
            if (d2 < min2)
            {
                min2 = d2;
            }

            double d3 = dx * _axes[2].X + dy * _axes[2].Y + dz * _axes[2].Z;
            if (d3 > max3)
            {
                max3 = d3;
            }
            if (d3 < min3)
            {
                min3 = d3;
            }
        }

        _extents[0] = max1 - min1;
        _extents[1] = max2 - min2;
        _extents[2] = max3 - min3;

        int dimension = 3;
        int dimCode = 0;
        if (Math.Abs(_extents[0]) <= tol)
        {
            --dimension;
            dimCode = 1;
        }
        if (Math.Abs(_extents[1]) <= tol)
        {
            --dimension;
            dimCode = 2 * (dimCode + 1);
        }
        if (Math.Abs(_extents[2]) <= tol)
        {
            --dimension;
            dimCode = 3 * (dimCode + 1);
        }

        switch (dimension)
        {
            case 0:
                {
                    _type = Type.Point;
                    break;
                }
            case 1:
                {
                    _type = Type.Line;
                    if (dimCode == 4)
                    {
                        _v1 = _axes[2];
                    }
                    else if (dimCode == 6)
                    {
                        _v1 = _axes[1];
                    }
                    else
                    {
                        _v1 = _axes[0];
                    }
                    break;
                }
            case 2:
                {
                    _type = Type.Plane;
                    if (dimCode == 1)
                    {
                        _v1 = _axes[0];
                    }
                    else if (dimCode == 2)
                    {
                        _v1 = _axes[1];
                    }
                    else
                    {
                        _v1 = _axes[2];
                    }
                    break;
                }
            case 3:
                {
                    _type = Type.Space;
                    _g = _g.WithXYZ(_g.XYZ() + min1 * _axes[0].XYZ() + min2 * _axes[1].XYZ()
                               + min3 * _axes[2].XYZ());
                    _v1 = _extents[0] * _axes[0];
                    _v2 = _extents[1] * _axes[1];
                    _v3 = _extents[2] * _axes[2];
                    break;
                }
        }
    }

    /// <summary>The type of the fitted entity.</summary>
    /// <remarks>Hides <see cref="object.GetType"/> to keep OCCT's name.</remarks>
    public new Type GetType() => _type;

    /// <summary>True if the points are coplanar within tolerance.</summary>
    public bool IsPlanar => _type == Type.Plane;

    /// <summary>True if the points are collinear within tolerance.</summary>
    public bool IsLinear => _type == Type.Line;

    /// <summary>True if the points are coincident within tolerance.</summary>
    public bool IsPoint => _type == Type.Point;

    /// <summary>True if the points span 3D space.</summary>
    public bool IsSpace => _type == Type.Space;

    /// <summary>The mean plane.</summary>
    /// <exception cref="NoSuchObjectException">The points are not planar.</exception>
    public Plane3D Plane()
    {
        if (!IsPlanar)
        {
            throw new NoSuchObjectException("GProp_PEquation::Plane: result is not planar");
        }
        return new Plane3D(_g, new Direction3D(_v1));
    }

    /// <summary>The mean line.</summary>
    /// <exception cref="NoSuchObjectException">The points are not linear.</exception>
    public Line3D Line()
    {
        if (!IsLinear)
        {
            throw new NoSuchObjectException("GProp_PEquation::Line: result is not linear");
        }
        return new Line3D(_g, new Direction3D(_v1));
    }

    /// <summary>The mean point.</summary>
    /// <exception cref="NoSuchObjectException">The points are not coincident.</exception>
    public Point3D Point()
    {
        if (!IsPoint)
        {
            throw new NoSuchObjectException("GProp_PEquation::Point: result is not a point");
        }
        return _g;
    }

    /// <summary>
    /// A box aligned with the principal axes: its corner (minimum projections) and its three edge vectors.
    /// </summary>
    /// <exception cref="NoSuchObjectException">The points do not span space.</exception>
    public void Box(out Point3D p, out Vector3D v1, out Vector3D v2, out Vector3D v3)
    {
        if (!IsSpace)
        {
            throw new NoSuchObjectException("GProp_PEquation::Box: result is not a 3D space");
        }
        p = _g;
        v1 = _v1;
        v2 = _v2;
        v3 = _v3;
    }

    /// <summary>The centre of mass of the points (valid whatever the result).</summary>
    public Point3D Barycentre() => _barycentre;

    /// <summary>The unit principal axis <paramref name="index"/> (1, 2 or 3), ordered by eigenvalue.</summary>
    public Vector3D PrincipalAxis(int index) => _axes[index - 1];

    /// <summary>The extent (max - min projection) along principal axis <paramref name="index"/> (1, 2 or 3).</summary>
    public double Extent(int index) => _extents[index - 1];
}
