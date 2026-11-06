// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtPElC2d.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtPElC2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Kerf.Engine.Extrema;

/// <summary>
/// All the extremum distances (minimum or maximum) between a 2D point and an elementary 2D curve (line, circle,
/// ellipse, hyperbola or parabola).
/// </summary>
/// <remarks>An algorithm object: construct it with the point and the curve, or empty and call <c>Perform</c>.</remarks>
internal class ExtPElC2d
{
    private bool _done;
    private int _nbExt;
    private readonly double[] _sqDist = new double[4];
    private readonly bool[] _isMin = new bool[4];
    private readonly POnCurv2d[] _point = [new(), new(), new(), new()];

    public ExtPElC2d()
    {
        _done = false;
        _nbExt = 0;

        for (int i = 0; i < 4; i++)
        {
            _sqDist[i] = RealLast();
            _isMin[i] = false;
        }
    }

    public ExtPElC2d(in Point2D p, in Line2D c, double tol, double uinf, double usup) => Perform(p, c, tol, uinf, usup);

    public void Perform(in Point2D p, in Line2D l, double tol, double uinf, double usup)
    {
        _done = true;
        Point2D or, myP;
        _nbExt = 0;

        Vector2D v1 = new(l.Direction);
        or = l.Location;
        Vector2D v = new(or, p);
        double mydist = v1.Dot(v);
        if ((mydist >= uinf - tol) && (mydist <= usup + tol))
        {
            _nbExt = 1;
            myP = or.Translated(mydist * v1);
            POnCurv2d myPOnCurve = new(mydist, myP);
            _sqDist[0] = p.SquareDistanceTo(myP);
            _point[0] = myPOnCurve;
            _isMin[0] = true;
        }
    }

    public ExtPElC2d(in Point2D p, in Circle2D c, double tol, double uinf, double usup) => Perform(p, c, tol, uinf, usup);

    public void Perform(in Point2D p, in Circle2D c, double tol, double uinf, double usup)
    {
        Point2D oc = c.Location;
        _nbExt = 0;

        if (oc.IsEqual(p, global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            _done = false;
        }
        else
        {
            double radius, u1, u2;
            Point2D p1, p2;

            _done = true;
            Direction2D v = new(new Vector2D(p, oc));
            radius = c.Radius;
            p1 = oc.Translated(radius * new Vector2D(v));
            u1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(c, p1);
            u2 = u1 + M_PI;
            p2 = oc.Translated(-radius * new Vector2D(v));
            double myuinf = uinf;
            global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(uinf, uinf + 2 * M_PI, global::Kerf.Engine.Foundation.Precision.PConfusion(), ref myuinf, ref u1);
            global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(uinf, uinf + 2 * M_PI, global::Kerf.Engine.Foundation.Precision.PConfusion(), ref myuinf, ref u2);
            if (((u1 - 2 * M_PI - uinf) < tol) && ((u1 - 2 * M_PI - uinf) > -tol))
            {
                u1 = uinf;
                p1 = new Point2D(oc.XY()
                                  + radius * (Math.Cos(u1) * c.XAxis.Direction.XY() + Math.Sin(u1) * c.YAxis.Direction.XY()));
            }

            if (((u2 - 2 * M_PI - uinf) < tol) && ((u2 - 2 * M_PI - uinf) > -tol))
            {
                u2 = uinf;
                p2 = new Point2D(oc.XY()
                                  + radius * (Math.Cos(u2) * c.XAxis.Direction.XY() + Math.Sin(u2) * c.YAxis.Direction.XY()));
            }

            if (((uinf - u1) < tol) && ((u1 - usup) < tol))
            {
                POnCurv2d myPOnCurve = new(u1, p1);
                _sqDist[0] = p.SquareDistanceTo(p1);
                _point[0] = myPOnCurve;
                _isMin[0] = true;
                _nbExt++;
            }

            if (((uinf - u2) < tol) && ((u2 - usup) < tol))
            {
                POnCurv2d myPOnCurve = new(u2, p2);
                _sqDist[_nbExt] = p.SquareDistanceTo(p2);
                _point[_nbExt] = myPOnCurve;
                _isMin[_nbExt] = true; // OCCT: the opposite point is a maximum
                _nbExt++;
            }
        }
    }

    public ExtPElC2d(in Point2D p, in Ellipse2D e, double tol, double uinf, double usup) => Perform(p, e, tol, uinf, usup);

    public void Perform(in Point2D p, in Ellipse2D e, double tol, double uinf, double usup)
    {
        _done = false;
        _nbExt = 0;
        Point2D or;
        or = e.Location;

        int noSol, nbSol;
        double a = e.MajorRadius;
        double b = e.MinorRadius;
        Vector2D v = new(or, p);

        if (or.IsEqual(p, global::Kerf.Engine.Foundation.Precision.Confusion()) && (Math.Abs(a - b) <= tol))
        {
            return;
        }
        else
        {
            double x = v.Dot(new Vector2D(e.XAxis.Direction));
            double y = v.Dot(new Vector2D(e.YAxis.Direction));

            TrigonometricFunctionRoots sol = new(0.0, (b * b - a * a) / 2.0, -b * y, a * x, 0.0, uinf, usup);

            if (!sol.IsDone)
            {
                return;
            }
            Point2D cu;
            double us;
            nbSol = sol.SolutionCount;
            _nbExt = 0;
            for (noSol = 1; noSol <= nbSol; noSol++)
            {
                us = sol.Value(noSol);
                cu = global::Kerf.Engine.Geometry.ElCLib.Value(us, e);
                _sqDist[_nbExt] = cu.SquareDistanceTo(p);
                _isMin[_nbExt] = (noSol == 0); // OCCT: always false (NoSol starts at 1)
                _point[_nbExt] = new POnCurv2d(us, cu);
                _nbExt++;
            }
            _done = true;
        }
    }

    public ExtPElC2d(in Point2D p, in Hyperbola2D c, double tol, double uinf, double usup) => Perform(p, c, tol, uinf, usup);

    public void Perform(in Point2D p, in Hyperbola2D h, double tol, double uinf, double usup)
    {
        Point2D o = h.Location;
        _done = false;
        _nbExt = 0;

        double R = h.MajorRadius;
        double r = h.MinorRadius;
        Vector2D oPp = new(o, p);
        double tol2 = tol * tol;
        double x = oPp.Dot(new Vector2D(h.XAxis.Direction));
        double y = oPp.Dot(new Vector2D(h.YAxis.Direction));
        double c1 = (R * R + r * r) / 4.0;
        DirectPolynomialRoots sol = new(c1, -(x * R + y * r) / 2.0, 0.0, (x * R - y * r) / 2.0, -c1);
        if (!sol.IsDone)
        {
            return;
        }
        Point2D cu;
        double us, vs;
        int nbSol = sol.SolutionCount;
        bool dejaEnr;
        int noExt;
        Span<Point2D> tbExt = [new(), new(), new(), new()];
        for (int noSol = 1; noSol <= nbSol; noSol++)
        {
            vs = sol.Value(noSol);
            if (vs > 0.0)
            {
                us = Math.Log(vs);
                if ((us >= uinf) && (us <= usup))
                {
                    cu = global::Kerf.Engine.Geometry.ElCLib.Value(us, h);
                    dejaEnr = false;
                    for (noExt = 0; noExt < _nbExt; noExt++)
                    {
                        if (tbExt[noExt].SquareDistanceTo(cu) < tol2)
                        {
                            dejaEnr = true;
                            break;
                        }
                    }
                    if (!dejaEnr)
                    {
                        tbExt[_nbExt] = cu;
                        _sqDist[_nbExt] = cu.SquareDistanceTo(p);
                        _isMin[_nbExt] = (noSol == 0); // OCCT: always false (NoSol starts at 1)
                        _point[_nbExt] = new POnCurv2d(us, cu);
                        _nbExt++;
                    }
                } // if ((Us >= Uinf) && (Us <= Usup))
            } // if (Vs > 0.)
        } // for (int NoSol = 1; ...
        _done = true;
    }

    public ExtPElC2d(in Point2D p, in Parabola2D c, double tol, double uinf, double usup) => Perform(p, c, tol, uinf, usup);

    public void Perform(in Point2D p, in Parabola2D c, double tol, double uinf, double usup)
    {
        _done = false;
        _nbExt = 0;
        Point2D o = c.Location;

        double tol2 = tol * tol;
        double f = c.Focal;
        Vector2D oPp = new(o, p);
        double x = oPp.Dot(new Vector2D(c.MirrorAxis.Direction));
        double y = oPp.Dot(new Vector2D(c.Axis.YAxis.Direction));

        DirectPolynomialRoots sol = new(1.0 / (4.0 * f), 0.0, 2.0 * f - x, -2.0 * f * y);
        if (!sol.IsDone)
        {
            return;
        }
        Point2D cu;
        double us;
        int nbSol = sol.SolutionCount;
        bool dejaEnr;
        int noExt;
        Span<Point2D> tbExt = [new(), new(), new()];
        for (int noSol = 1; noSol <= nbSol; noSol++)
        {
            us = sol.Value(noSol);
            if ((us >= uinf) && (us <= usup))
            {
                cu = global::Kerf.Engine.Geometry.ElCLib.Value(us, c);
                dejaEnr = false;
                for (noExt = 0; noExt < _nbExt; noExt++)
                {
                    if (tbExt[noExt].SquareDistanceTo(cu) < tol2)
                    {
                        dejaEnr = true;
                        break;
                    }
                }
                if (!dejaEnr)
                {
                    tbExt[_nbExt] = cu;
                    _sqDist[_nbExt] = cu.SquareDistanceTo(p);
                    _isMin[_nbExt] = (noSol == 0); // OCCT: always false (NoSol starts at 1)
                    _point[_nbExt] = new POnCurv2d(us, cu);
                    _nbExt++;
                }
            } // if ((Us >= Uinf) && (Us <= Usup))
        } // for (int NoSol = 1; ...
        _done = true;
    }

    /// <summary>True if the distances are found.</summary>
    public bool IsDone => _done;

    /// <summary>The number of extremum distances.</summary>
    public int ExtCount
    {
        get
        {
            if (!IsDone)
            {
                throw new NotDoneException();
            }
            return _nbExt;
        }
    }

    /// <summary>The value of the Nth extremum square distance.</summary>
    public double SquareDistance(int n)
    {
        if ((n < 1) || (n > ExtCount))
        {
            throw new OutOfRangeException();
        }
        return _sqDist[n - 1];
    }

    /// <summary>True if the Nth extremum distance is a minimum.</summary>
    public bool IsMin(int n)
    {
        if ((n < 1) || (n > ExtCount))
        {
            throw new OutOfRangeException();
        }
        return _isMin[n - 1];
    }

    /// <summary>The point of the Nth extremum distance.</summary>
    public POnCurv2d Point(int n)
    {
        if ((n < 1) || (n > ExtCount))
        {
            throw new OutOfRangeException();
        }
        return _point[n - 1];
    }
}
