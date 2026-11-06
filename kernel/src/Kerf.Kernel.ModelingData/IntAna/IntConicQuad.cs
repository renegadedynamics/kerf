// Port of: src/ModelingData/TKGeomBase/IntAna/IntAna_IntConicQuad.hxx
// Port of: src/ModelingData/TKGeomBase/IntAna/IntAna_IntConicQuad.lxx
// Port of: src/ModelingData/TKGeomBase/IntAna/IntAna_IntConicQuad.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Kerf.Engine.Intersection.ResultType;

namespace Kerf.Engine.Intersection;

/// <summary>
/// The analytic intersection between a conic (line, circle, ellipse, parabola or hyperbola of gp) and a quadric
/// (<see cref="Quadric"/>). The intersection between a conic and a plane is treated as a special case. The
/// results are points, with their parameters on the conic.
/// </summary>
/// <remarks>An algorithm object: construct it with the arguments, or empty and call <c>Perform</c>.</remarks>
internal class IntConicQuad
{
    private static readonly double PIpPI = M_PI + M_PI;

    private bool _done;
    private bool _parallel;
    private bool _inquadric;
    private int _nbpts;
    private readonly Point3D[] _pnts = new Point3D[4];
    private readonly double[] _paramonc = new double[4];

    //==                                          E m p t y   C o n s t r u c t o r
    public IntConicQuad()
    {
        _done = false;
        _parallel = false;
        _inquadric = false;
        _nbpts = 0;
    }

    //==                                                 L i n e  -   Q u a d r i c
    public IntConicQuad(in Line3D l, Quadric quad)
    {
        Perform(l, quad);
    }

    public void Perform(in Line3D l, Quadric quad)
    {
        double qxx, qyy, qzz, qxy, qxz, qyz, qx, qy, qz, qCte;
        _done = _inquadric = _parallel = false;

        //----------------------------------------------------------------------
        //-- Substitution of x=t Lx + Lx0       ( expressed in                 )
        //--                  y=t Ly + Ly0       (  the canonical coordinate   )
        //--                  z=t Lz + Lz0       (  system                     )
        //--
        //-- Into     Qxx x**2 + Qyy y**2 + Qzz z**2
        //--          + 2 ( Qxy x y  + Qxz x z  + Qyz y z  )
        //--          + 2 ( Qx x + Qy y + Qz z )
        //--          + QCte
        //--
        //-- Gives a polynomial in t: A2 t**2 + A1 t + A0 with:
        //----------------------------------------------------------------------

        double lx0, ly0, lz0, lx, ly, lz;

        _nbpts = 0;

        l.Direction.Coord(out lx, out ly, out lz);
        l.Location.Coord(out lx0, out ly0, out lz0);

        quad.Coefficients(out qxx, out qyy, out qzz, out qxy, out qxz, out qyz, out qx, out qy, out qz, out qCte);

        double a0 = (qCte + qxx * lx0 * lx0 + qyy * ly0 * ly0 + qzz * lz0 * lz0
                     + 2.0 * (lx0 * (qx + qxy * ly0 + qxz * lz0) + ly0 * (qy + qyz * lz0) + qz * lz0));

        double a1 =
            2.0
            * (lx * (qx + qxx * lx0 + qxy * ly0 + qxz * lz0) + ly * (qy + qxy * lx0 + qyy * ly0 + qyz * lz0)
               + lz * (qz + qxz * lx0 + qyz * ly0 + qzz * lz0));

        double a2 = (qxx * lx * lx + qyy * ly * ly + qzz * lz * lz
                     + 2.0 * (lx * (qxy * ly + qxz * lz) + qyz * ly * lz));

        DirectPolynomialRoots linQuadPol = new(a2, a1, a0);

        if (linQuadPol.IsDone)
        {
            _done = true;
            if (linQuadPol.InfiniteRoots())
            {
                _inquadric = true;
            }
            else
            {
                _nbpts = linQuadPol.SolutionCount;

                for (int i = 1; i <= _nbpts; i++)
                {
                    double t = linQuadPol.Value(i);
                    _paramonc[i - 1] = t;
                    _pnts[i - 1] = new Point3D(lx0 + lx * t, ly0 + ly * t, lz0 + lz * t);
                }
            }
        }
    }

    //==                                            C i r c l e   -   Q u a d r i c
    public IntConicQuad(in Circle3D c, Quadric quad)
    {
        Perform(c, quad);
    }

    public void Perform(in Circle3D c, Quadric quad)
    {
        double qxx, qyy, qzz, qxy, qxz, qyz, qx, qy, qz, qCte;

        //----------------------------------------------------------------------
        //-- In the reference frame associated with C.Position():
        //-- xC = R * Cos[t]
        //-- yC = R * Sin[t]
        //-- zC = 0
        //--
        //-- We express the quadric in this frame and substitute
        //-- xC, yC and zC for x, y and z
        //--
        //-- We obtain a polynomial in Cos[t] and Sin[t] of degree 2
        //----------------------------------------------------------------------
        _done = _inquadric = _parallel = false;

        quad.Coefficients(out qxx, out qyy, out qzz, out qxy, out qxz, out qyz, out qx, out qy, out qz, out qCte);
        quad.NewCoefficients(ref qxx, ref qyy, ref qzz, ref qxy, ref qxz, ref qyz, ref qx, ref qy, ref qz, ref qCte,
                             new CoordinateSystem(c.Position));

        double r = c.Radius;
        double rr = r * r;

        double P_CosCos = rr * qxx; //-- Cos Cos
        double P_SinSin = rr * qyy; //-- Sin Sin
        double P_Sin = r * qy;      //-- 2 Sin
        double P_Cos = r * qx;      //-- 2 Cos
        double P_CosSin = rr * qxy; //-- 2 Cos Sin
        double P_Cte = qCte;        //-- 1

        TrigonometricFunctionRoots circQuadPol = new(P_CosCos - P_SinSin,
                                                          P_CosSin,
                                                          P_Cos + P_Cos,
                                                          P_Sin + P_Sin,
                                                          P_Cte + P_SinSin,
                                                          0.0,
                                                          PIpPI);

        if (circQuadPol.IsDone)
        {
            _done = true;
            if (circQuadPol.InfiniteRoots())
            {
                _inquadric = true;
            }
            else
            {
                _nbpts = circQuadPol.SolutionCount;

                for (int i = 1; i <= _nbpts; i++)
                {
                    double t = circQuadPol.Value(i);
                    _paramonc[i - 1] = t;
                    _pnts[i - 1] = global::Kerf.Engine.Geometry.ElCLib.CircleValue(t, c.Position, r);
                }
            }
        }
    }

    //==                                                  E l i p s - Q u a d r i c
    public IntConicQuad(in Ellipse3D e, Quadric quad)
    {
        Perform(e, quad);
    }

    public void Perform(in Ellipse3D e, Quadric quad)
    {
        double qxx, qyy, qzz, qxy, qxz, qyz, qx, qy, qz, qCte;

        _done = _inquadric = _parallel = false;

        //----------------------------------------------------------------------
        //-- In the reference frame associated with E.Position():
        //-- xE = R * Cos[t]
        //-- yE = r * Sin[t]
        //-- zE = 0
        //--
        //-- We express the quadric in this frame and substitute
        //-- xE, yE and zE for x, y and z
        //--
        //-- We obtain a polynomial in Cos[t] and Sin[t] of degree 2
        //----------------------------------------------------------------------

        quad.Coefficients(out qxx, out qyy, out qzz, out qxy, out qxz, out qyz, out qx, out qy, out qz, out qCte);
        quad.NewCoefficients(ref qxx, ref qyy, ref qzz, ref qxy, ref qxz, ref qyz, ref qx, ref qy, ref qz, ref qCte,
                             new CoordinateSystem(e.Position));

        double R = e.MajorRadius;
        double r = e.MinorRadius;

        double P_CosCos = R * R * qxx; //-- Cos Cos
        double P_SinSin = r * r * qyy; //-- Sin Sin
        double P_Sin = r * qy;         //-- 2 Sin
        double P_Cos = R * qx;         //-- 2 Cos
        double P_CosSin = R * r * qxy; //-- 2 Cos Sin
        double P_Cte = qCte;           //-- 1

        TrigonometricFunctionRoots elipsQuadPol = new(P_CosCos - P_SinSin,
                                                           P_CosSin,
                                                           P_Cos + P_Cos,
                                                           P_Sin + P_Sin,
                                                           P_Cte + P_SinSin,
                                                           0.0,
                                                           PIpPI);

        if (elipsQuadPol.IsDone)
        {
            _done = true;
            if (elipsQuadPol.InfiniteRoots())
            {
                _inquadric = true;
            }
            else
            {
                _nbpts = elipsQuadPol.SolutionCount;
                for (int i = 1; i <= _nbpts; i++)
                {
                    double t = elipsQuadPol.Value(i);
                    _paramonc[i - 1] = t;
                    _pnts[i - 1] = global::Kerf.Engine.Geometry.ElCLib.EllipseValue(t, e.Position, R, r);
                }
            }
        }
    }

    //==                                                  P a r a b - Q u a d r i c
    public IntConicQuad(in Parabola3D p, Quadric quad)
    {
        Perform(p, quad);
    }

    public void Perform(in Parabola3D p, Quadric quad)
    {
        double qxx, qyy, qzz, qxy, qxz, qyz, qx, qy, qz, qCte;

        _done = _inquadric = _parallel = false;

        //----------------------------------------------------------------------
        //-- In the reference frame associated with P.Position():
        //-- xP = y*y / (2 p)
        //-- yP = y
        //-- zP = 0
        //--
        //-- We express the quadric in this frame and substitute
        //-- xP, yP and zP for x, y and z
        //--
        //-- We obtain a polynomial in y of degree 4
        //----------------------------------------------------------------------

        quad.Coefficients(out qxx, out qyy, out qzz, out qxy, out qxz, out qyz, out qx, out qy, out qz, out qCte);
        quad.NewCoefficients(ref qxx, ref qyy, ref qzz, ref qxy, ref qxz, ref qyz, ref qx, ref qy, ref qz, ref qCte,
                             new CoordinateSystem(p.Position));

        double f = p.Focal;
        double Un_Sur_2p = 0.25 / f;

        double a4 = qxx * Un_Sur_2p * Un_Sur_2p;
        double a3 = (qxy + qxy) * Un_Sur_2p;
        double a2 = qyy + (qx + qx) * Un_Sur_2p;
        double a1 = qy + qy;
        double a0 = qCte;

        DirectPolynomialRoots parabQuadPol = new(a4, a3, a2, a1, a0);

        if (parabQuadPol.IsDone)
        {
            _done = true;
            if (parabQuadPol.InfiniteRoots())
            {
                _inquadric = true;
            }
            else
            {
                _nbpts = parabQuadPol.SolutionCount;

                for (int i = 1; i <= _nbpts; i++)
                {
                    double t = parabQuadPol.Value(i);
                    _paramonc[i - 1] = t;
                    _pnts[i - 1] = global::Kerf.Engine.Geometry.ElCLib.ParabolaValue(t, p.Position, f);
                }
            }
        }
    }

    //==                                                    H y p r - Q u a d r i c
    public IntConicQuad(in Hyperbola3D h, Quadric quad)
    {
        Perform(h, quad);
    }

    public void Perform(in Hyperbola3D h, Quadric quad)
    {
        double qxx, qyy, qzz, qxy, qxz, qyz, qx, qy, qz, qCte;

        _done = _inquadric = _parallel = false;

        //----------------------------------------------------------------------
        //-- In the reference frame associated with H.Position():
        //-- xH = R Ch[t]
        //-- yH = r Sh[t]
        //-- zH = 0
        //--
        //-- We express the quadric in this frame and substitute
        //-- xH, yH and zH for x, y and z
        //--
        //-- We obtain a polynomial in Exp[t] of degree 4
        //----------------------------------------------------------------------

        quad.Coefficients(out qxx, out qyy, out qzz, out qxy, out qxz, out qyz, out qx, out qy, out qz, out qCte);
        quad.NewCoefficients(ref qxx, ref qyy, ref qzz, ref qxy, ref qxz, ref qyz, ref qx, ref qy, ref qz, ref qCte,
                             new CoordinateSystem(h.Position));

        double R = h.MajorRadius;
        double r = h.MinorRadius;

        double RR = R * R;
        double rr = r * r;
        double Rr = R * r;

        double a4 = RR * qxx + Rr * (qxy + qxy) + rr * qyy;
        double a3 = 4.0 * (R * qx + r * qy);
        double a2 = 2.0 * ((qCte + qCte) + qxx * RR - qyy * rr);
        double a1 = 4.0 * (R * qx - r * qy);
        double a0 = qxx * RR - Rr * (qxy + qxy) + qyy * rr;

        DirectPolynomialRoots hyperQuadPol = new(a4, a3, a2, a1, a0);

        if (hyperQuadPol.IsDone)
        {
            _done = true;
            if (hyperQuadPol.InfiniteRoots())
            {
                _inquadric = true;
            }
            else
            {
                _nbpts = hyperQuadPol.SolutionCount;
                int bonnessolutions = 0;
                for (int i = 1; i <= _nbpts; i++)
                {
                    double t = hyperQuadPol.Value(i);
                    if (t >= RealEpsilon())
                    {
                        double lnt = Math.Log(t);
                        _paramonc[bonnessolutions] = lnt;
                        _pnts[bonnessolutions] = global::Kerf.Engine.Geometry.ElCLib.HyperbolaValue(lnt, h.Position, R, r);
                        bonnessolutions++;
                    }
                }
                _nbpts = bonnessolutions;
            }
        }
    }

    /// <summary>
    /// The intersection between a line and a plane. Tolang is the angular tolerance below which the line is
    /// parallel to the plane. When Len (the length of a bounded line) is not 0, a nearly parallel line is only
    /// parallel if its end point at Len is within Tol of the plane.
    /// </summary>
    public IntConicQuad(in Line3D l, in Plane3D p, double tolang, double tol = 0, double len = 0)
    {
        Perform(l, p, tolang, tol, len);
    }

    /// <summary>The intersection between a circle and a plane.</summary>
    public IntConicQuad(in Circle3D c, in Plane3D p, double tolang, double tol)
    {
        Perform(c, p, tolang, tol);
    }

    /// <summary>The intersection between an ellipse and a plane (the tolerances are not used).</summary>
    public IntConicQuad(in Ellipse3D e, in Plane3D p, double tolang, double tol)
    {
        Perform(e, p, tolang, tol);
    }

    /// <summary>The intersection between a parabola and a plane (the tolerance is not used).</summary>
    public IntConicQuad(in Parabola3D pb, in Plane3D p, double tolang)
    {
        Perform(pb, p, tolang);
    }

    /// <summary>The intersection between a hyperbola and a plane (the tolerance is not used).</summary>
    public IntConicQuad(in Hyperbola3D h, in Plane3D p, double tolang)
    {
        Perform(h, p, tolang);
    }

    public void Perform(in Line3D l, in Plane3D p, double tolang, double tol = 0, double len = 0)
    {
        // Tolang represents the angular tolerance from which we consider
        // that the angle between 2 vectors is zero. We will reason about the cosine of this
        // angle, (we have std::cos(t) equivalent to t near Pi/2).

        _done = false;

        double a, b, c, d;
        double al, bl, cl;
        double dis, direc;

        p.Coefficients(out a, out b, out c, out d);
        Point3D orig = l.Location;
        l.Direction.Coord(out al, out bl, out cl);

        direc = a * al + b * bl + c * cl;
        dis = a * orig.X + b * orig.Y + c * orig.Z + d;
        //
        _parallel = false;
        if (Math.Abs(direc) < tolang)
        {
            _parallel = true;
            if (len != 0 && direc != 0)
            {
                // check the distance from bounding point of the line to the plane
                Point3D p1 = new(), p2 = new();
                //
                p1 = p1.WithCoord(orig.X - dis * a, orig.Y - dis * b, orig.Z - dis * c);
                p2 = p2.WithCoord(p1.X + len * al, p1.Y + len * bl, p1.Z + len * cl);
                if (p.DistanceTo(p2) > tol)
                {
                    _parallel = false;
                }
            }
        }
        if (_parallel)
        {
            _inquadric = Math.Abs(dis) < tolang;
        }
        else
        {
            _parallel = false;
            _inquadric = false;
            _nbpts = 1;
            _paramonc[0] = -dis / direc;
            _pnts[0] = _pnts[0].WithCoord(orig.X + _paramonc[0] * al,
                             orig.Y + _paramonc[0] * bl,
                             orig.Z + _paramonc[0] * cl);
        }
        _done = true;
    }

    public void Perform(in Circle3D c, in Plane3D p, double tolang, double tol)
    {
        _done = false;

        Plane3D plconic = new(new CoordinateSystem(c.Position));
        QuadQuadGeo intP = new(plconic, p, tolang, tol);
        if (!intP.IsDone)
        {
            return;
        }
        if (intP.TypeInter() == Empty)
        {
            _parallel = true;
            double distmax = p.DistanceTo(c.Location) + c.Radius * tolang;
            _inquadric = distmax < tol;
            _done = true;
        }
        else if (intP.TypeInter() == Same)
        {
            _inquadric = true;
            _done = true;
        }
        else
        {
            _inquadric = false;
            _parallel = false;
            Line3D ligsol = intP.Line(1);

            Vector3D v0 = new(plconic.Location, ligsol.Location);
            Vector3D axex = new(plconic.Position.XDirection);
            Vector3D axey = new(plconic.Position.YDirection);

            Point2D orig = new(axex.Dot(v0), axey.Dot(v0));
            Vector2D dire = new(axex.Dot(new Vector3D(ligsol.Direction)), axey.Dot(new Vector3D(ligsol.Direction)));

            Line2D ligs = new(orig, new Direction2D(dire));
            Point2D pnt2dBid = new(0.0, 0.0);
            Direction2D dir2dBid = new(Direction2D.D.X);
            Axis2D ax2dBid = new(pnt2dBid, dir2dBid);
            Circle2D cir = new(ax2dBid, c.Radius);

            AnalyticIntersection int2d = new(ligs, cir);

            if (!int2d.IsDone)
            {
                return;
            }

            _nbpts = int2d.PointCount;
            for (int i = 1; i <= _nbpts; i++)
            {
                Point2D resul = int2d.Point(i).Value();
                double x = resul.X;
                double y = resul.Y;
                _pnts[i - 1] = _pnts[i - 1].WithCoord(plconic.Location.X + x * axex.X + y * axey.X,
                                     plconic.Location.Y + x * axex.Y + y * axey.Y,
                                     plconic.Location.Z + x * axex.Z + y * axey.Z);
                _paramonc[i - 1] = int2d.Point(i).ParamOnSecond();
            }
            _done = true;
        }
    }

    public void Perform(in Ellipse3D e, in Plane3D pln, double tolang, double tol)
    {
        _ = tolang;
        _ = tol;
        // OCCT converts the plane implicitly through IntAna_Quadric's constructor
        Perform(e, new Quadric(pln));
    }

    public void Perform(in Parabola3D p, in Plane3D pln, double tolang)
    {
        _ = tolang;
        Perform(p, new Quadric(pln));
    }

    public void Perform(in Hyperbola3D h, in Plane3D pln, double tolang)
    {
        _ = tolang;
        Perform(h, new Quadric(pln));
    }

    /// <summary>True if the computation was successful.</summary>
    public bool IsDone => _done;

    /// <summary>True if the conic lies in the quadric.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public bool IsInQuadric
    {
        get
        {
            if (!_done)
            {
                throw new NotDoneException();
            }
            return _inquadric;
        }
    }

    /// <summary>True if the line is parallel to the plane, or the circle's plane parallel to the plane.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public bool IsParallel
    {
        get
        {
            if (!_done)
            {
                throw new NotDoneException();
            }
            return _parallel;
        }
    }

    /// <summary>The number of intersection points.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    /// <exception cref="DomainErrorException">The conic is parallel or in the quadric.</exception>
    public int PointCount
    {
        get
        {
            if (!_done)
            {
                throw new NotDoneException();
            }
            if (_parallel || _inquadric)
            {
                throw new DomainErrorException();
            }
            return _nbpts;
        }
    }

    /// <summary>The intersection point of index i (from 1).</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    /// <exception cref="DomainErrorException">The conic is parallel or in the quadric.</exception>
    /// <exception cref="OutOfRangeException">i is not in [1, NbPoints].</exception>
    public Point3D Point(int i)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        if (_parallel || _inquadric)
        {
            throw new DomainErrorException();
        }
        if ((i > _nbpts) || (i <= 0))
        {
            throw new OutOfRangeException();
        }
        return _pnts[i - 1];
    }

    /// <summary>The parameter on the conic of the intersection point of index i (from 1).</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    /// <exception cref="DomainErrorException">The conic is parallel or in the quadric.</exception>
    /// <exception cref="OutOfRangeException">i is not in [1, NbPoints].</exception>
    public double ParamOnConic(int i)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        if (_parallel || _inquadric)
        {
            throw new DomainErrorException();
        }
        if ((i > _nbpts) || (i <= 0))
        {
            throw new OutOfRangeException();
        }
        return _paramonc[i - 1];
    }
}
