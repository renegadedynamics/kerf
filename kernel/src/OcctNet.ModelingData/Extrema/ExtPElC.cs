// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtPElC.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtPElC.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Extrema;

/// <summary>
/// All the extremum distances (minimum or maximum) between a point and an elementary curve (line, circle, ellipse,
/// hyperbola or parabola).
/// </summary>
/// <remarks>An algorithm object: construct it with the point and the curve, or empty and call <c>Perform</c>.</remarks>
public class ExtPElC
{
    private bool _done;
    private int _nbExt;
    private readonly double[] _sqDist = new double[4];
    private readonly bool[] _isMin = new bool[4];
    private readonly POnCurv[] _point = [new(), new(), new(), new()];

    public ExtPElC()
    {
        _done = false;
        _nbExt = 0;

        for (int i = 0; i < 4; i++)
        {
            _sqDist[i] = RealLast();
            _isMin[i] = false;
        }
    }

    /// <summary>The extremum distance between the point P and the segment [Uinf,Usup] of the line L.</summary>
    public ExtPElC(in Point3D p, in Line3D l, double tol, double uinf, double usup) => Perform(p, l, tol, uinf, usup);

    public void Perform(in Point3D p, in Line3D l, double tol, double uinf, double usup)
    {
        _done = false;
        _nbExt = 0;
        Vector3D v1 = new(l.Direction());
        Point3D or = l.Location();
        Vector3D v = new(or, p);
        double mydist = v1.Dot(v);
        if ((mydist >= uinf - tol) && (mydist <= usup + tol))
        {
            Point3D myP = or.Translated(mydist * v1);
            POnCurv myPOnCurve = new(mydist, myP);
            _sqDist[0] = p.SquareDistance(myP);
            _point[0] = myPOnCurve;
            _isMin[0] = true;
            _nbExt = 1;
            _done = true;
        }
    }

    /// <summary>
    /// The 2 extremum distances between the point P and the segment [Uinf,Usup] of the circle C. Tol is used to
    /// determine if P is on the axis of the circle (infinite solutions, IsDone false) or if an extremum is on an
    /// endpoint of the segment. Uinf must be in [0, 2.PI] and Usup &gt; Uinf; only the solutions in
    /// [Uinf, Uinf+2.PI[ are computed.
    /// </summary>
    public ExtPElC(in Point3D p, in Circle3D c, double tol, double uinf, double usup) => Perform(p, c, tol, uinf, usup);

    /*-----------------------------------------------------------------------------
    Function:
      Find values of parameter u such as:
       - dist(P,C(u)) pass by an extrema,
       - Uinf <= u <= Usup.

    Method:
      Pass 3 stages:
      1- Projection of point P in the plane of the circle,
      2- Calculation of u solutions in [0.,2.*M_PI]:
          Let Pp, the projected point and
               O, the center of the circle;
         2 cases:
         - if Pp is mixed with 0, there is an infinite number of solutions;
           IsDone() renvoie false.
         - otherwise, 2 points are solutions for the complete circle:
           . Us1 = angle(OPp,OX) corresponds to the minimum,
           . let Us2 = ( Us1 + M_PI if Us1 < M_PI,
                        ( Us1 - M_PI otherwise;
             Us2 corresponds to the maximum.
      3- Calculate the extrema in [Uinf,Usup].
    -----------------------------------------------------------------------------*/
    public void Perform(in Point3D p, in Circle3D c, double tol, double uinf, double usup)
    {
        _done = false;
        _nbExt = 0;

        // 1- Projection of the point P in the plane of circle -> Pp ...

        Point3D o = c.Location();
        Vector3D axe = new(c.Axis().Direction());
        Vector3D trsl = axe.Multiplied(-(new Vector3D(o, p).Dot(axe)));
        Point3D pp = p.Translated(trsl);

        // 2- Calculate u solutions in [0.,2.*PI] ...

        Vector3D oPp = new(o, pp);
        if (oPp.Magnitude() < tol)
        {
            return;
        }
        Span<double> usol = stackalloc double[2];
        usol[0] = c.XAxis().Direction().AngleWithRef(new Direction3D(oPp), new Direction3D(axe)); // -M_PI<U1<M_PI

        double angTol = global::Kerf.Engine.Foundation.Precision.Angular();
        if (usol[0] + M_PI < angTol)
        {
            usol[0] = -M_PI;
        }
        else if (usol[0] - M_PI > -angTol)
        {
            usol[0] = M_PI;
        }

        usol[1] = usol[0] + M_PI;

        double myuinf = uinf;
        // double TolU = Tol*C.Radius();
        double tolU, r;
        r = c.Radius();
        tolU = global::Kerf.Engine.Foundation.Precision.Infinite();
        if (r > Constants.Resolution())
        {
            tolU = tol / r;
        }
        //
        global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(uinf, uinf + 2 * M_PI, tolU, ref myuinf, ref usol[0]);
        global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(uinf, uinf + 2 * M_PI, tolU, ref myuinf, ref usol[1]);
        if (((usol[0] - 2 * M_PI - uinf) < tolU) && ((usol[0] - 2 * M_PI - uinf) > -tolU))
        {
            usol[0] = uinf;
        }
        if (((usol[1] - 2 * M_PI - uinf) < tolU) && ((usol[1] - 2 * M_PI - uinf) > -tolU))
        {
            usol[1] = uinf;
        }

        // 3- Calculate extrema in [Umin,Umax] ...

        Point3D cu;
        double us;
        for (int noSol = 0; noSol <= 1; noSol++)
        {
            us = usol[noSol];
            if (((uinf - us) < tolU) && ((us - usup) < tolU))
            {
                cu = global::Kerf.Engine.Geometry.ElCLib.Value(us, c);
                _sqDist[_nbExt] = cu.SquareDistance(p);
                _isMin[_nbExt] = (noSol == 0);
                _point[_nbExt] = new POnCurv(us, cu);
                _nbExt++;
            }
        }
        _done = true;
    }

    /// <summary>
    /// The 4 extremum distances between the point P and the segment [Uinf,Usup] of the ellipse C. Tol is used to
    /// determine if the point is on the axis of the ellipse and the radii are equal (infinite solutions, IsDone
    /// false).
    /// </summary>
    public ExtPElC(in Point3D p, in Ellipse3D c, double tol, double uinf, double usup) => Perform(p, c, tol, uinf, usup);

    /*-----------------------------------------------------------------------------
    Function:
      Find values of parameter u so that:
       - dist(P,C(u)) passes by an extremum,
       - Uinf <= u <= Usup.

    Method:
      Takes 2 steps:
      1- Projection of point P in the plane of the ellipse,
      2- Calculation of the solutions:
         Let Pp, the projected point; find values u so that:
           (C(u)-Pp).C'(u) = 0. (1)
         Let Cos = cos(u) and Sin = sin(u),
              C(u) = (A*Cos,B*Sin) and Pp = (X,Y);
         Then, (1) <=> (A*Cos-X,B*Sin-Y).(-A*Sin,B*Cos) = 0.
                        (B**2-A**2)*Cos*Sin - B*Y*Cos + A*X*Sin = 0.
         Use algorithm math_TrigonometricFunctionRoots to solve this equation.
    -----------------------------------------------------------------------------*/
    public void Perform(in Point3D p, in Ellipse3D c, double tol, double uinf, double usup)
    {
        _done = false;
        _nbExt = 0;

        // 1- Projection of point P in the plane of the ellipse -> Pp ...

        Point3D o = c.Location();
        Vector3D axe = new(c.Axis().Direction());
        Vector3D trsl = axe.Multiplied(-(new Vector3D(o, p).Dot(axe)));
        Point3D pp = p.Translated(trsl);

        // 2- Calculation of solutions ...

        int noSol, nbSol;
        double a = c.MajorRadius();
        double b = c.MinorRadius();
        Vector3D oPp = new(o, pp);
        double oPpMagn = oPp.Magnitude();
        if (oPpMagn < tol)
        {
            if (Math.Abs(a - b) < tol)
            {
                return;
            }
        }
        double x = oPp.Dot(new Vector3D(c.XAxis().Direction()));
        double y = oPp.Dot(new Vector3D(c.YAxis().Direction()));

        double ko2 = (b * b - a * a) / 2.0, ko3 = -b * y, ko4 = a * x;
        if (Math.Abs(ko3) < 1.0e-16 * Max(Math.Abs(ko2), Math.Abs(ko3)))
        {
            ko3 = 0.0;
        }

        //  math_TrigonometricFunctionRoots Sol(0.,(B*B-A*A)/2.,-B*Y,A*X,0.,Uinf,Usup);
        TrigonometricFunctionRoots sol = new(0.0, ko2, ko3, ko4, 0.0, uinf, usup);

        if (!sol.IsDone)
        {
            return;
        }
        Point3D cu;
        double us;
        nbSol = sol.SolutionCount;
        for (noSol = 1; noSol <= nbSol; noSol++)
        {
            us = sol.Value(noSol);
            cu = global::Kerf.Engine.Geometry.ElCLib.Value(us, c);
            _sqDist[_nbExt] = cu.SquareDistance(p);
            _point[_nbExt] = new POnCurv(us, cu);
            cu = global::Kerf.Engine.Geometry.ElCLib.Value(us + 0.1, c);
            _isMin[_nbExt] = _sqDist[_nbExt] < cu.SquareDistance(p);
            _nbExt++;
        }
        _done = true;
    }

    /// <summary>
    /// The extremum distances between the point P and the segment [Uinf,Usup] of the hyperbola C. Two solutions u
    /// and v are identical when dist(C(u),C(v)) &lt; Tol.
    /// </summary>
    public ExtPElC(in Point3D p, in Hyperbola3D c, double tol, double uinf, double usup) => Perform(p, c, tol, uinf, usup);

    /*-----------------------------------------------------------------------------
    Function:
      Find values of parameter u so that:
       - dist(P,C(u)) passes by an extremum,
       - Uinf <= u <= Usup.

    Method:
      Takes 2 steps:
      1- Projection of point P in the plane of the hyperbola,
      2- Calculation of solutions:
          Let Pp, le point projete; on recherche les valeurs u telles que:
           (C(u)-Pp).C'(u) = 0. (1)
          Let R and r be the radiuses of the hyperbola,
              Chu = std::cosh(u) and Shu = std::sinh(u),
              C(u) = (R*Chu,r*Shu) and Pp = (X,Y);
         Then, (1) <=> (R*Chu-X,r*Shu-Y).(R*Shu,r*Chu) = 0.
                        (R**2+r**2)*Chu*Shu - X*R*Shu - Y*r*Chu = 0. (2)
         Let v = e**u;
         Then, by using Chu = (e**u+e**(-u))/2. and Sh = (e**u-e**(-u)))/2.
                (2) <=> ((R**2+r**2)/4.) * (v**2-v**(-2)) -
                    ((X*R+Y*r)/2.) * v +
                    ((X*R-Y*r)/2.) * v**(-1) = 0.
          (2)* v**2	<=> ((R**2+r**2)/4.) * v**4 -
                         ((X*R+Y*r)/2.)  * v**3 +
                 ((X*R-Y*r)/2.)  * v    -
                ((R**2+r**2)/4.)        = 0.
      Use algorithm math_DirectPolynomialRoots to solve this equation by v.
    -----------------------------------------------------------------------------*/
    public void Perform(in Point3D p, in Hyperbola3D c, double tol, double uinf, double usup)
    {
        _done = false;
        _nbExt = 0;

        // 1- Projection of point P in the plane of hyperbola -> Pp ...

        Point3D o = c.Location();
        Vector3D axe = new(c.Axis().Direction());
        Vector3D trsl = axe.Multiplied(-(new Vector3D(o, p).Dot(axe)));
        Point3D pp = p.Translated(trsl);

        // 2- Calculation of solutions ...

        double tol2 = tol * tol;
        double R = c.MajorRadius();
        double r = c.MinorRadius();
        Vector3D oPp = new(o, pp);
        double x = oPp.Dot(new Vector3D(c.XAxis().Direction()));
        double y = oPp.Dot(new Vector3D(c.YAxis().Direction()));

        double c1 = (R * R + r * r) / 4.0;
        DirectPolynomialRoots sol = new(c1, -(x * R + y * r) / 2.0, 0.0, (x * R - y * r) / 2.0, -c1);
        if (!sol.IsDone)
        {
            return;
        }
        Point3D cu;
        double us, vs;
        int nbSol = sol.SolutionCount;
        bool dejaEnr;
        int noExt;
        Span<Point3D> tbExt = [new(), new(), new(), new()];
        for (int noSol = 1; noSol <= nbSol; noSol++)
        {
            vs = sol.Value(noSol);
            if (vs > 0.0)
            {
                us = Math.Log(vs);
                if ((us >= uinf) && (us <= usup))
                {
                    cu = global::Kerf.Engine.Geometry.ElCLib.Value(us, c);
                    dejaEnr = false;
                    for (noExt = 0; noExt < _nbExt; noExt++)
                    {
                        if (tbExt[noExt].SquareDistance(cu) < tol2)
                        {
                            dejaEnr = true;
                            break;
                        }
                    }
                    if (!dejaEnr)
                    {
                        tbExt[_nbExt] = cu;
                        _sqDist[_nbExt] = cu.SquareDistance(p);
                        _isMin[_nbExt] = _sqDist[_nbExt] < p.SquareDistance(global::Kerf.Engine.Geometry.ElCLib.Value(us + 1, c));
                        _point[_nbExt] = new POnCurv(us, cu);
                        _nbExt++;
                    }
                } // if ((Us >= Uinf) && (Us <= Usup))
            } // if (Vs > 0.)
        } // for (int NoSol = 1; ...
        _done = true;
    }

    /// <summary>
    /// The extremum distances between the point P and the segment [Uinf,Usup] of the parabola C. Two solutions are
    /// identical when their points are within <see cref="Precision.Confusion"/>; Tol is not used.
    /// </summary>
    public ExtPElC(in Point3D p, in Parabola3D c, double tol, double uinf, double usup) => Perform(p, c, tol, uinf, usup);

    /*-----------------------------------------------------------------------------
    Function:
      Find values of parameter u so that:
       - dist(P,C(u)) pass by an extremum,
       - Uinf <= u <= Usup.

    Method:
      Takes 2 steps:
      1- Projection of point P in the plane of the parabola,
      2- Calculation of solutions:
         Let Pp, the projected point; find values u so that:
           (C(u)-Pp).C'(u) = 0. (1)
         Let F the focus of the parabola,
              C(u) = ((u*u)/(4.*F),u) and Pp = (X,Y);
         Alors, (1) <=> ((u*u)/(4.*F)-X,u-Y).(u/(2.*F),1) = 0.
                        (1./(4.*F)) * U**3 + (2.*F-X) * U - 2*F*Y = 0.
        Use algorithm math_DirectPolynomialRoots to solve this equation by U.
    -----------------------------------------------------------------------------*/
    public void Perform(in Point3D p, in Parabola3D c, double tol, double uinf, double usup)
    {
        _ = tol; // OCCT: the tolerance is not used
        _done = false;
        _nbExt = 0;

        // 1- Projection of point P in the plane of the parabola -> Pp ...

        Point3D o = c.Location();
        Vector3D axe = new(c.Axis().Direction());
        Vector3D trsl = axe.Multiplied(-(new Vector3D(o, p).Dot(axe)));
        Point3D pp = p.Translated(trsl);

        // 2- Calculation of solutions ...

        double f = c.Focal();
        Vector3D oPp = new(o, pp);
        double x = oPp.Dot(new Vector3D(c.XAxis().Direction()));
        double y = oPp.Dot(new Vector3D(c.YAxis().Direction()));
        DirectPolynomialRoots sol = new(1.0 / (4.0 * f), 0.0, 2.0 * f - x, -2.0 * f * y);
        if (!sol.IsDone)
        {
            return;
        }
        Point3D cu;
        double us;
        int nbSol = sol.SolutionCount;
        bool dejaEnr;
        int noExt;
        Span<Point3D> tbExt = [new(), new(), new()];
        for (int noSol = 1; noSol <= nbSol; noSol++)
        {
            us = sol.Value(noSol);
            if ((us >= uinf) && (us <= usup))
            {
                cu = global::Kerf.Engine.Geometry.ElCLib.Value(us, c);
                dejaEnr = false;
                for (noExt = 0; noExt < _nbExt; noExt++)
                {
                    if (tbExt[noExt].SquareDistance(cu) < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        dejaEnr = true;
                        break;
                    }
                }
                if (!dejaEnr)
                {
                    tbExt[_nbExt] = cu;
                    _sqDist[_nbExt] = cu.SquareDistance(p);
                    _isMin[_nbExt] = _sqDist[_nbExt] < p.SquareDistance(global::Kerf.Engine.Geometry.ElCLib.Value(us + 1, c));
                    _point[_nbExt] = new POnCurv(us, cu);
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
    public POnCurv Point(int n)
    {
        if ((n < 1) || (n > ExtCount))
        {
            throw new OutOfRangeException();
        }
        return _point[n - 1];
    }
}
