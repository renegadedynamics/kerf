// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomPlate/GeomPlate_BuildAveragePlane.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomPlate/GeomPlate_BuildAveragePlane.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Plate;

/// <summary>
/// The average plane of a set of points (or, when the points are aligned, their line), and the UV box of the points
/// in it; and the half-space test GeomPlate_BuildPlateSurface uses to choose a plane from the normals of its
/// constraints.
/// </summary>
/// <remarks>
/// Computed in the constructor. It keeps the caller's points, read only. <see cref="Plane"/> and <see cref="Line"/>
/// freeze the geometry they return (the same object each time).
/// </remarks>
public class BuildAveragePlane
{
    private readonly NCollection_HArray1<Point3D> _pts;
    private double _umax;
    private double _vmax;
    private double _vmin;
    private double _umin;
    private readonly PlaneSurface? _plane;
    private readonly double _tol;
    private readonly LineCurve3D? _line;
    private Vector3D _ox;
    private Vector3D _oy;
    private Point3D _g;
    private readonly int _nbBoundPoints;

    /// <summary>
    /// The average plane of <paramref name="pts"/>, the first <paramref name="nbBoundPoints"/> of which are on the
    /// boundary.
    /// </summary>
    /// <param name="pts">The points.</param>
    /// <param name="nbBoundPoints">The number of boundary points, first in <paramref name="pts"/>.</param>
    /// <param name="tol">The tolerance of the inertia axes.</param>
    /// <param name="pOption">1: the plane through the barycentre with the normal found; 2: the axes of inertia of
    /// the points projected on it.</param>
    /// <param name="nOption">1: the normal of the axes of inertia (or of the boundary polygon if they differ by more
    /// than 60 degrees); 2: the normal of the boundary polygon.</param>
    public BuildAveragePlane(NCollection_HArray1<Point3D> pts, int nbBoundPoints, double tol, int pOption,
                                       int nOption)
    {
        _pts = pts;
        _tol = tol;
        _nbBoundPoints = nbBoundPoints;
        Vector3D oz = DefPlan(nOption);

        if (oz.SquareMagnitude() > 0)
        {
            if (pOption == 1)
            {
                _plane = new PlaneSurface(_g, new Direction3D(oz));
                _ox = new Vector3D(_plane.Pln().XAxis().Direction());
                _oy = new Vector3D(_plane.Pln().YAxis().Direction());
            }
            else
            {
                BasePlan(oz);
                Direction3D nDir = new Direction3D(_ox ^ _oy);
                Direction3D uDir = new Direction3D(_ox);
                CoordinateSystem triedre = new CoordinateSystem(_g, nDir, uDir);
                _plane = new PlaneSurface(triedre);
            }
            int i, nb = _pts.Length();
            Plane3D p = _plane.Pln();
            global::Kerf.Engine.Geometry.ElSLib.Parameters(p, _g, out _umax, out _vmax);
            _umin = _umax;
            _vmin = _vmax;
            double u = 0, v = 0;
            for (i = 1; i <= nb; i++)
            {
                global::Kerf.Engine.Geometry.ElSLib.Parameters(p, _pts.Value(i), out u, out v);
                if (_umax < u)
                {
                    _umax = u;
                }
                if (_umin > u)
                {
                    _umin = u;
                }
                if (_vmax < v)
                {
                    _vmax = v;
                }
                if (_vmin > v)
                {
                    _vmin = v;
                }
            }
        }

        if (IsLine())
        {
            _line = new LineCurve3D(_g, new Direction3D(_ox));
        }
    }

    /// <summary>
    /// The plane through the axis of inertia of <paramref name="pts"/> whose normal makes the smallest largest angle
    /// with <paramref name="normals"/>.
    /// </summary>
    public BuildAveragePlane(NCollection_Sequence<Vector3D> normals, NCollection_HArray1<Point3D> pts)
    {
        _pts = pts;
        int i, j, k, n, m;

        Vector3D bestVec = new Vector3D();
        int nn = normals.Length();

        if (nn == 1)
        {
            bestVec = normals.Value(1);
        }
        else if (nn == 2)
        {
            bestVec = normals.Value(1) + normals.Value(2);
            double sqMagn = bestVec.SquareMagnitude();
            if (sqMagn < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
            {
                double sq1 = normals.Value(1).SquareMagnitude(), sq2 = normals.Value(2).SquareMagnitude();

                if (sq1 > sq2)
                {
                    bestVec = normals.Value(1).Normalized();
                }
                else
                {
                    bestVec = normals.Value(2).Normalized();
                }
            }
            else
            {
                bestVec = bestVec.Divided(Math.Sqrt(sqMagn));
            }
        }
        else // the common case
        {
            double maxAngle = 0.0;
            for (i = 1; i <= nn - 1; i++)
            {
                for (j = i + 1; j <= nn; j++)
                {
                    double angle = normals.Value(i).Angle(normals.Value(j));
                    if (angle > maxAngle)
                    {
                        maxAngle = angle;
                    }
                }
            }
            maxAngle *= 1.2;
            maxAngle /= 2.0;
            int nint = 50;

            NCollection_Array1<Vector3D> optVec = new NCollection_Array1<Vector3D>(1, nn * (nn - 1) / 2);
            NCollection_Array1<double> optScal = new NCollection_Array1<double>(1, nn * (nn - 1) / 2);
            Vector3D Vec, vec1;
            Direction3D cross1, cross2;

            k = 1;
            for (i = 1; i <= nn - 1; i++)
            {
                for (j = i + 1; j <= nn; j++, k++)
                {
                    optScal[k] = RealFirst();

                    double step = maxAngle / nint;
                    Vec = normals.Value(i) + normals.Value(j);

                    double sqMagn = Vec.SquareMagnitude();

                    if (sqMagn < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        continue;
                    }

                    Vec = Vec.Divided(Math.Sqrt(sqMagn));

                    cross1 = new Direction3D(normals.Value(i) ^ normals.Value(j));
                    cross2 = new Direction3D(Vec ^ new Vector3D(cross1));
                    Axis3D axe = new Axis3D(new Point3D(0, 0, 0), cross2);

                    vec1 = Vec.Rotated(axe, -maxAngle);

                    for (n = 0; n <= 2 * nint; n++)
                    {
                        vec1 = vec1.Rotated(axe, step);
                        double minScal = RealLast();
                        for (m = 1; m <= nn; m++)
                        {
                            double scal = vec1 * normals.Value(m);
                            if (scal < minScal)
                            {
                                minScal = scal;
                            }
                        }
                        if (minScal > optScal[k])
                        {
                            optScal[k] = minScal;
                            optVec[k] = vec1;
                        }
                    }
                } // for i, for j
            }
            // Find maximum among all maximums

            double bestScal = RealFirst();
            int index = 0;
            for (k = 1; k <= optScal.Length(); k++)
            {
                if (optScal[k] > bestScal)
                {
                    bestScal = optScal[k];
                    index = k;
                }
            }
            bestVec = optVec[index];
        }

        // Making the plane myPlane
        Frame3D axe2;
        bool isSingular;
        NCollection_Array1<Point3D> ptsArray = new NCollection_Array1<Point3D>(1, _pts.Length());
        for (i = 1; i <= _pts.Length(); i++)
        {
            ptsArray[i] = _pts.Value(i);
        }
        global::Kerf.Engine.Geometry.Library.GeomLib.AxeOfInertia(ptsArray, out axe2, out isSingular);
        Direction3D bestDir = new Direction3D(bestVec);
        Direction3D xDir = bestDir ^ axe2.XDirection();
        xDir ^= bestDir;

        CoordinateSystem axe3 = new CoordinateSystem(axe2.Location(), bestDir, xDir);
        _plane = new PlaneSurface(axe3);

        // Initializing myUmin, myVmin, myUmax, myVmax
        Plane3D pln = _plane.Pln();
        global::Kerf.Engine.Geometry.ElSLib.Parameters(pln, axe2.Location(), out _umax, out _vmax);
        _umin = _umax;
        _vmin = _vmax;
        double u, v;
        for (i = 1; i <= _pts.Length(); i++)
        {
            Vector3D vec = new Vector3D(pln.Location(), _pts.Value(i));
            Vector3D normVec = new Vector3D(pln.Axis().Direction());
            normVec = (vec * normVec) * normVec;

            global::Kerf.Engine.Geometry.ElSLib.Parameters(pln, _pts.Value(i).Translated(-normVec), out u, out v);
            if (u > _umax)
            {
                _umax = u;
            }
            if (u < _umin)
            {
                _umin = u;
            }
            if (v > _vmax)
            {
                _vmax = v;
            }
            if (v < _vmin)
            {
                _vmin = v;
            }
        }
        // Initializing myOX, myOY
        _ox = new Vector3D(_plane.Pln().XAxis().Direction());
        _oy = new Vector3D(_plane.Pln().YAxis().Direction());
    }

    /// <summary>The average plane (frozen).</summary>
    /// <exception cref="NoSuchObjectException">The points are aligned (<see cref="IsLine"/>).</exception>
    public PlaneSurface? Plane()
    {
        Standard_NoSuchObject_Raise_if(
            IsLine(),
            "Cannot use the function 'GeomPlate_BuildAveragePlane::Plane()', the Object is a 'Geom_Line'");
        _plane?.Freeze();
        return _plane;
    }

    /// <summary>The bounds of the points' parameters in the plane.</summary>
    public void MinMaxBox(out double umin, out double umax, out double vmin, out double vmax)
    {
        umax = _umax;
        umin = _umin;
        vmax = _vmax;
        vmin = _vmin;
    }

    private Vector3D DefPlan(int nOption)
    {
        Point3D gb = new Point3D();
        Vector3D a = new Vector3D(), b = new Vector3D(), c = new Vector3D(), d;
        Vector3D oz = new Vector3D();
        int i, nb = _pts.Length();
        gb = gb.WithCoord(0.0, 0.0, 0.0);
        for (i = 1; i <= nb; i++)
        {
            gb = gb.WithCoord(1, (gb.Coord(1) + _pts.Value(i).Coord(1)));
            gb = gb.WithCoord(2, (gb.Coord(2) + _pts.Value(i).Coord(2)));
            gb = gb.WithCoord(3, (gb.Coord(3) + _pts.Value(i).Coord(3)));
        }
        _g = _g.WithCoord(1, (gb.Coord(1) / nb));
        _g = _g.WithCoord(2, (gb.Coord(2) / nb));
        _g = _g.WithCoord(3, (gb.Coord(3) / nb));

        if (nOption == 1)
        {
            Frame3D axe;
            bool isSingular;
            global::Kerf.Engine.Geometry.Library.GeomLib.AxeOfInertia(_pts.Array1(), out axe, out isSingular, _tol);

            _ox = new Vector3D(axe.XDirection());
            _oy = new Vector3D(axe.YDirection());

            oz = new Vector3D(axe.Direction());

            if (_nbBoundPoints != 0 && _pts.Length() != _nbBoundPoints)
            {
                a = a.WithCoord(0.0, 0.0, 0.0);
                for (i = 3; i <= _nbBoundPoints; i++)
                {
                    b = b.WithCoord(_pts.Value(i - 1).Coord(1) - _pts.Value(1).Coord(1),
                               _pts.Value(i - 1).Coord(2) - _pts.Value(1).Coord(2),
                               _pts.Value(i - 1).Coord(3) - _pts.Value(1).Coord(3));
                    c = c.WithCoord(_pts.Value(i).Coord(1) - _pts.Value(1).Coord(1),
                               _pts.Value(i).Coord(2) - _pts.Value(1).Coord(2),
                               _pts.Value(i).Coord(3) - _pts.Value(1).Coord(3));
                    d = b ^ c;
                    a = a.WithCoord(1, a.Coord(1) + d.Coord(1));
                    a = a.WithCoord(2, a.Coord(2) + d.Coord(2));
                    a = a.WithCoord(3, a.Coord(3) + d.Coord(3));
                }
                Vector3D oz1 = a;
                double theAngle = oz.Angle(oz1);
                if (theAngle > M_PI / 2)
                {
                    theAngle = M_PI - theAngle;
                }
                if (theAngle > M_PI / 3)
                {
                    oz = oz1;
                }
            }
        }

        else if (nOption == 2)
        {
            a = a.WithCoord(0.0, 0.0, 0.0);
            for (i = 3; i <= _nbBoundPoints; i++)
            {
                b = b.WithCoord(_pts.Value(i - 1).Coord(1) - _pts.Value(1).Coord(1),
                           _pts.Value(i - 1).Coord(2) - _pts.Value(1).Coord(2),
                           _pts.Value(i - 1).Coord(3) - _pts.Value(1).Coord(3));
                c = c.WithCoord(_pts.Value(i).Coord(1) - _pts.Value(1).Coord(1),
                           _pts.Value(i).Coord(2) - _pts.Value(1).Coord(2),
                           _pts.Value(i).Coord(3) - _pts.Value(1).Coord(3));
                d = b ^ c;
                a = a.WithCoord(1, a.Coord(1) + d.Coord(1));
                a = a.WithCoord(2, a.Coord(2) + d.Coord(2));
                a = a.WithCoord(3, a.Coord(3) + d.Coord(3));
            }
            oz = a;
        }
        return oz;
    }

    private void BasePlan(in Vector3D oz)
    {
        Matrix m = new Matrix(1, 3, 1, 3);
        m.Init(0.0);
        Vector3D proj = new Vector3D();
        int i, nb = _pts.Length();
        double scal;

        for (i = 1; i <= nb; i++)
        {
            proj = proj.WithCoord(1, _pts.Value(i).Coord(1) - _g.Coord(1));
            proj = proj.WithCoord(2, _pts.Value(i).Coord(2) - _g.Coord(2));
            proj = proj.WithCoord(3, _pts.Value(i).Coord(3) - _g.Coord(3));
            scal = proj.Coord(1) * oz.Coord(1) + proj.Coord(2) * oz.Coord(2) + proj.Coord(3) * oz.Coord(3);
            scal /= oz.Coord(1) * oz.Coord(1) + oz.Coord(2) * oz.Coord(2) + oz.Coord(3) * oz.Coord(3);
            proj = proj.WithCoord(1, proj.Coord(1) - scal * oz.Coord(1));
            proj = proj.WithCoord(2, proj.Coord(2) - scal * oz.Coord(2));
            proj = proj.WithCoord(3, proj.Coord(3) - scal * oz.Coord(3));
            m[1, 1] += proj.Coord(1) * proj.Coord(1);
            m[2, 2] += proj.Coord(2) * proj.Coord(2);
            m[3, 3] += proj.Coord(3) * proj.Coord(3);
            m[1, 2] += proj.Coord(1) * proj.Coord(2);
            m[1, 3] += proj.Coord(1) * proj.Coord(3);
            m[2, 3] += proj.Coord(2) * proj.Coord(3);
        }
        m[2, 1] = m[1, 2];
        m[3, 1] = m[1, 3];
        m[3, 2] = m[2, 3];
        Jacobi j = new Jacobi(m);
        double n1, n2, n3;
        math_Vector v1 = new math_Vector(1, 3), v2 = new math_Vector(1, 3), v3 = new math_Vector(1, 3);
        n1 = j.Value(1);
        n2 = j.Value(2);
        n3 = j.Value(3);

        double r1 = Min(Min(n1, n2), n3), r2;
        int m1, m2, m3;
        if (r1 == n1)
        {
            m1 = 1;
            r2 = Min(n2, n3);
            if (r2 == n2)
            {
                m2 = 2;
                m3 = 3;
            }
            else
            {
                m2 = 3;
                m3 = 2;
            }
        }
        else
        {
            if (r1 == n2)
            {
                m1 = 2;
                r2 = Min(n1, n3);
                if (r2 == n1)
                {
                    m2 = 1;
                    m3 = 3;
                }
                else
                {
                    m2 = 3;
                    m3 = 1;
                }
            }
            else
            {
                m1 = 3;
                r2 = Min(n1, n2);
                if (r2 == n1)
                {
                    m2 = 1;
                    m3 = 2;
                }
                else
                {
                    m2 = 2;
                    m3 = 1;
                }
            }
        }
        j.Vector(m1, v1);
        j.Vector(m2, v2);
        j.Vector(m3, v3);

        if (((Math.Abs(n1) <= _tol) && (Math.Abs(n2) <= _tol))
            || ((Math.Abs(n2) <= _tol) && (Math.Abs(n3) <= _tol))
            || ((Math.Abs(n1) <= _tol) && (Math.Abs(n3) <= _tol)))
        {
            _ox = _ox.WithCoord(v3[1], v3[2], v3[3]);
            _oy = oz ^ _ox;
        }
        else
        {
            _ox = _ox.WithCoord(v3[1], v3[2], v3[3]);
            _oy = _oy.WithCoord(v2[1], v2[2], v2[3]);
        }
    }

    /// <summary>The line of the points (frozen).</summary>
    /// <exception cref="NoSuchObjectException">The points aren't aligned (<see cref="IsPlane"/>).</exception>
    public LineCurve3D? Line()
    {
        Standard_NoSuchObject_Raise_if(
            IsPlane(),
            "Cannot use the function 'GeomPlate_BuildAveragePlane::Line()', the Object is a 'Geom_Plane'");
        _line?.Freeze();
        return _line;
    }

    public bool IsPlane()
    {
        Vector3D oz = _ox ^ _oy;
        return oz.SquareMagnitude() != 0;
    }

    public bool IsLine()
    {
        Vector3D oz = _ox ^ _oy;
        return oz.SquareMagnitude() == 0;
    }

    /// <summary>
    /// Adds <paramref name="newNormals"/> to <paramref name="normals"/> if all of them still lie in a common
    /// half-space, updating the boundary pairs <paramref name="bset"/>; otherwise leaves both unchanged and returns
    /// false.
    /// </summary>
    public static bool HalfSpace(NCollection_Sequence<Vector3D> newNormals, NCollection_Sequence<Vector3D> normals,
                                 NCollection_Sequence<Aij> bset, double linTol, double angTol)
    {
        double squareTol = linTol * linTol;

        NCollection_Sequence<Vector3D> saveNormals;
        NCollection_Sequence<Aij> saveBset;
        // 1
        saveNormals = new NCollection_Sequence<Vector3D>(normals);
        saveBset = new NCollection_Sequence<Aij>(bset);

        Vector3D cross, nullVec = new Vector3D(0, 0, 0);
        NCollection_Sequence<Aij> b1set = new NCollection_Sequence<Aij>(),
                                            b2set = new NCollection_Sequence<Aij>();
        int i, j, k;

        i = 1;
        if (normals.IsEmpty())
        {
            if (newNormals.Length() == 1)
            {
                normals.Append(newNormals.Last());
                return true;
            }
            // 2
            cross = newNormals.Value(1) ^ newNormals.Value(2);
            if (cross.SquareMagnitude() <= squareTol)
            {
                return false;
            }

            cross = cross.Normalized();
            bset.Append(new Aij(1, 2, cross));
            bset.Append(new Aij(2, 1, -cross));
            normals.Append(newNormals.Value(1));
            normals.Append(newNormals.Value(2));
            i = 3;
        }

        for (; i <= newNormals.Length(); i++)
        {
            // 3
            double scal;
            for (j = 1; j <= bset.Length(); j++)
            {
                if ((scal = bset.Value(j)._vec * newNormals.Value(i)) >= -linTol)
                {
                    b2set.Append(bset.Value(j));
                }
            }

            int ii = normals.Length() + 1;
            for (j = 1; j <= ii - 1; j++)
            {
                if (normals.Value(j).SquareMagnitude() == 0.0)
                {
                    continue;
                }
                // 4
                cross = newNormals.Value(i) ^ normals.Value(j);
                if (cross.SquareMagnitude() <= squareTol)
                {
                    normals.Assign(saveNormals);
                    bset.Assign(saveBset);
                    return false;
                }
                cross = cross.Normalized();
                bool isNew = true;
                for (k = 1; k <= b2set.Length(); k++)
                {
                    if (b2set.Value(k)._vec.IsOpposite(-cross, angTol)) // if (B2set(k).Vec.IsEqual( Cross, LinTol, AngTol ))
                    {
                        Vector3D cross1, cross2;
                        int ind1 = b2set.Value(k)._ind1, ind2 = b2set.Value(k)._ind2;
                        if (ind1 == ii || ind2 == ii)
                        {
                            isNew = false;
                            break;
                        }
                        cross1 = normals.Value(ind1) ^ newNormals.Value(i);
                        cross2 = normals.Value(ind2) ^ newNormals.Value(i);
                        if (cross1.SquareMagnitude() <= squareTol || cross2.SquareMagnitude() <= squareTol)
                        {
                            normals.Assign(saveNormals);
                            bset.Assign(saveBset);
                            return false;
                        }
                        if (cross1.IsOpposite(cross2, angTol))
                        {
                            cross2 = normals.Value(ind1) ^ normals.Value(ind2);
                            if (cross1.IsOpposite(cross2, angTol))
                            {
                                normals.Assign(saveNormals);
                                bset.Assign(saveBset);
                                return false;
                            }
                        }
                        else
                        {
                            if (newNormals.Value(i).Angle(normals.Value(ind1)) > newNormals.Value(i).Angle(normals.Value(ind2)))
                            {
                                b2set[k]._ind2 = ind1;
                                b2set[k]._ind1 = ii;
                            }
                            else
                            {
                                b2set[k]._ind1 = ii;
                            }
                        }
                        isNew = false;
                        break;
                    }
                }
                if (isNew)
                {
                    b1set.Append(new Aij(ii, j, cross));
                }

                cross = cross.Reversed();
                isNew = true;
                for (k = 1; k <= b2set.Length(); k++)
                {
                    if (b2set.Value(k)._vec.IsOpposite(-cross, angTol)) // if (B2set(k).Vec.IsEqual( Cross, LinTol, AngTol ))
                    {
                        Vector3D cross1, cross2;
                        int ind1 = b2set.Value(k)._ind1, ind2 = b2set.Value(k)._ind2;
                        if (ind1 == ii || ind2 == ii)
                        {
                            isNew = false;
                            break;
                        }
                        cross1 = normals.Value(ind1) ^ newNormals.Value(i);
                        cross2 = normals.Value(ind2) ^ newNormals.Value(i);
                        if (cross1.SquareMagnitude() <= squareTol || cross2.SquareMagnitude() <= squareTol)
                        {
                            normals.Assign(saveNormals);
                            bset.Assign(saveBset);
                            return false;
                        }
                        if (cross1.IsOpposite(cross2, angTol))
                        {
                            cross2 = normals.Value(ind1) ^ normals.Value(ind2);
                            if (cross1.IsOpposite(cross2, angTol))
                            {
                                normals.Assign(saveNormals);
                                bset.Assign(saveBset);
                                return false;
                            }
                        }
                        else
                        {
                            if (newNormals.Value(i).Angle(normals.Value(ind1)) > newNormals.Value(i).Angle(normals.Value(ind2)))
                            {
                                b2set[k]._ind2 = ind1;
                                b2set[k]._ind1 = ii;
                            }
                            else
                            {
                                b2set[k]._ind1 = ii;
                            }
                        }
                        isNew = false;
                        break;
                    }
                }
                if (isNew)
                {
                    b1set.Append(new Aij(ii, j, cross));
                }
            }

            // 5
            for (j = 1; j <= b1set.Length(); j++)
            {
                bool isGEnull = true;
                for (k = 1; k <= normals.Length(); k++)
                {
                    if (normals.Value(k).SquareMagnitude() == 0.0)
                    {
                        continue;
                    }
                    if (b1set.Value(j)._vec * normals.Value(k) < -linTol)
                    {
                        isGEnull = false;
                        break;
                    }
                }
                if (isGEnull)
                {
                    b2set.Append(b1set.Value(j));
                }
            }

            // 6
            if (b2set.IsEmpty())
            {
                normals.Assign(saveNormals);
                bset.Assign(saveBset);
                return false;
            }

            // 7
            bset.Assign(b2set);
            b2set.Clear();
            b1set.Clear();
            normals.Append(newNormals.Value(i));

            // 8
            for (j = 1; j <= normals.Length(); j++)
            {
                if (normals.Value(j).SquareMagnitude() == 0.0)
                {
                    continue;
                }
                bool isFound = false;
                for (k = 1; k <= bset.Length(); k++)
                {
                    if (j == bset.Value(k)._ind1 || j == bset.Value(k)._ind2)
                    {
                        isFound = true;
                        break;
                    }
                }
                if (!isFound)
                {
                    normals[j] = nullVec;
                }
            }
        }

        return true;
    }
}
