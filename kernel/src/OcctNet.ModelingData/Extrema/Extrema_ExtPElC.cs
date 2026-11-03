// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtPElC.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtPElC.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// All the extremum distances (minimum or maximum) between a point and an elementary curve (line, circle, ellipse,
/// hyperbola or parabola).
/// </summary>
/// <remarks>An algorithm object: construct it with the point and the curve, or empty and call <c>Perform</c>.</remarks>
public class Extrema_ExtPElC
{
    private bool myDone;
    private int myNbExt;
    private readonly double[] mySqDist = new double[4];
    private readonly bool[] myIsMin = new bool[4];
    private readonly Extrema_POnCurv[] myPoint = [new(), new(), new(), new()];

    public Extrema_ExtPElC()
    {
        myDone = false;
        myNbExt = 0;

        for (int i = 0; i < 4; i++)
        {
            mySqDist[i] = RealLast();
            myIsMin[i] = false;
        }
    }

    /// <summary>The extremum distance between the point P and the segment [Uinf,Usup] of the line L.</summary>
    public Extrema_ExtPElC(in Point3D P, in Line3D L, double Tol, double Uinf, double Usup) => Perform(P, L, Tol, Uinf, Usup);

    public void Perform(in Point3D P, in Line3D L, double Tol, double Uinf, double Usup)
    {
        myDone = false;
        myNbExt = 0;
        Vector3D V1 = new(L.Direction());
        Point3D OR = L.Location();
        Vector3D V = new(OR, P);
        double Mydist = V1.Dot(V);
        if ((Mydist >= Uinf - Tol) && (Mydist <= Usup + Tol))
        {
            Point3D MyP = OR.Translated(Mydist * V1);
            Extrema_POnCurv MyPOnCurve = new(Mydist, MyP);
            mySqDist[0] = P.SquareDistance(MyP);
            myPoint[0] = MyPOnCurve;
            myIsMin[0] = true;
            myNbExt = 1;
            myDone = true;
        }
    }

    /// <summary>
    /// The 2 extremum distances between the point P and the segment [Uinf,Usup] of the circle C. Tol is used to
    /// determine if P is on the axis of the circle (infinite solutions, IsDone false) or if an extremum is on an
    /// endpoint of the segment. Uinf must be in [0, 2.PI] and Usup &gt; Uinf; only the solutions in
    /// [Uinf, Uinf+2.PI[ are computed.
    /// </summary>
    public Extrema_ExtPElC(in Point3D P, in Circle3D C, double Tol, double Uinf, double Usup) => Perform(P, C, Tol, Uinf, Usup);

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
    public void Perform(in Point3D P, in Circle3D C, double Tol, double Uinf, double Usup)
    {
        myDone = false;
        myNbExt = 0;

        // 1- Projection of the point P in the plane of circle -> Pp ...

        Point3D O = C.Location();
        Vector3D Axe = new(C.Axis().Direction());
        Vector3D Trsl = Axe.Multiplied(-(new Vector3D(O, P).Dot(Axe)));
        Point3D Pp = P.Translated(Trsl);

        // 2- Calculate u solutions in [0.,2.*PI] ...

        Vector3D OPp = new(O, Pp);
        if (OPp.Magnitude() < Tol)
        {
            return;
        }
        Span<double> Usol = stackalloc double[2];
        Usol[0] = C.XAxis().Direction().AngleWithRef(new Direction3D(OPp), new Direction3D(Axe)); // -M_PI<U1<M_PI

        double aAngTol = global::Kerf.Engine.Foundation.Precision.Angular();
        if (Usol[0] + M_PI < aAngTol)
        {
            Usol[0] = -M_PI;
        }
        else if (Usol[0] - M_PI > -aAngTol)
        {
            Usol[0] = M_PI;
        }

        Usol[1] = Usol[0] + M_PI;

        double myuinf = Uinf;
        // double TolU = Tol*C.Radius();
        double TolU, aR;
        aR = C.Radius();
        TolU = global::Kerf.Engine.Foundation.Precision.Infinite();
        if (aR > Constants.Resolution())
        {
            TolU = Tol / aR;
        }
        //
        global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(Uinf, Uinf + 2 * M_PI, TolU, ref myuinf, ref Usol[0]);
        global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(Uinf, Uinf + 2 * M_PI, TolU, ref myuinf, ref Usol[1]);
        if (((Usol[0] - 2 * M_PI - Uinf) < TolU) && ((Usol[0] - 2 * M_PI - Uinf) > -TolU))
        {
            Usol[0] = Uinf;
        }
        if (((Usol[1] - 2 * M_PI - Uinf) < TolU) && ((Usol[1] - 2 * M_PI - Uinf) > -TolU))
        {
            Usol[1] = Uinf;
        }

        // 3- Calculate extrema in [Umin,Umax] ...

        Point3D Cu;
        double Us;
        for (int NoSol = 0; NoSol <= 1; NoSol++)
        {
            Us = Usol[NoSol];
            if (((Uinf - Us) < TolU) && ((Us - Usup) < TolU))
            {
                Cu = global::Kerf.Engine.Geometry.ElCLib.Value(Us, C);
                mySqDist[myNbExt] = Cu.SquareDistance(P);
                myIsMin[myNbExt] = (NoSol == 0);
                myPoint[myNbExt] = new Extrema_POnCurv(Us, Cu);
                myNbExt++;
            }
        }
        myDone = true;
    }

    /// <summary>
    /// The 4 extremum distances between the point P and the segment [Uinf,Usup] of the ellipse C. Tol is used to
    /// determine if the point is on the axis of the ellipse and the radii are equal (infinite solutions, IsDone
    /// false).
    /// </summary>
    public Extrema_ExtPElC(in Point3D P, in Ellipse3D C, double Tol, double Uinf, double Usup) => Perform(P, C, Tol, Uinf, Usup);

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
    public void Perform(in Point3D P, in Ellipse3D C, double Tol, double Uinf, double Usup)
    {
        myDone = false;
        myNbExt = 0;

        // 1- Projection of point P in the plane of the ellipse -> Pp ...

        Point3D O = C.Location();
        Vector3D Axe = new(C.Axis().Direction());
        Vector3D Trsl = Axe.Multiplied(-(new Vector3D(O, P).Dot(Axe)));
        Point3D Pp = P.Translated(Trsl);

        // 2- Calculation of solutions ...

        int NoSol, NbSol;
        double A = C.MajorRadius();
        double B = C.MinorRadius();
        Vector3D OPp = new(O, Pp);
        double OPpMagn = OPp.Magnitude();
        if (OPpMagn < Tol)
        {
            if (Math.Abs(A - B) < Tol)
            {
                return;
            }
        }
        double X = OPp.Dot(new Vector3D(C.XAxis().Direction()));
        double Y = OPp.Dot(new Vector3D(C.YAxis().Direction()));

        double ko2 = (B * B - A * A) / 2.0, ko3 = -B * Y, ko4 = A * X;
        if (Math.Abs(ko3) < 1.0e-16 * Max(Math.Abs(ko2), Math.Abs(ko3)))
        {
            ko3 = 0.0;
        }

        //  math_TrigonometricFunctionRoots Sol(0.,(B*B-A*A)/2.,-B*Y,A*X,0.,Uinf,Usup);
        TrigonometricFunctionRoots Sol = new(0.0, ko2, ko3, ko4, 0.0, Uinf, Usup);

        if (!Sol.IsDone)
        {
            return;
        }
        Point3D Cu;
        double Us;
        NbSol = Sol.SolutionCount;
        for (NoSol = 1; NoSol <= NbSol; NoSol++)
        {
            Us = Sol.Value(NoSol);
            Cu = global::Kerf.Engine.Geometry.ElCLib.Value(Us, C);
            mySqDist[myNbExt] = Cu.SquareDistance(P);
            myPoint[myNbExt] = new Extrema_POnCurv(Us, Cu);
            Cu = global::Kerf.Engine.Geometry.ElCLib.Value(Us + 0.1, C);
            myIsMin[myNbExt] = mySqDist[myNbExt] < Cu.SquareDistance(P);
            myNbExt++;
        }
        myDone = true;
    }

    /// <summary>
    /// The extremum distances between the point P and the segment [Uinf,Usup] of the hyperbola C. Two solutions u
    /// and v are identical when dist(C(u),C(v)) &lt; Tol.
    /// </summary>
    public Extrema_ExtPElC(in Point3D P, in Hyperbola3D C, double Tol, double Uinf, double Usup) => Perform(P, C, Tol, Uinf, Usup);

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
    public void Perform(in Point3D P, in Hyperbola3D C, double Tol, double Uinf, double Usup)
    {
        myDone = false;
        myNbExt = 0;

        // 1- Projection of point P in the plane of hyperbola -> Pp ...

        Point3D O = C.Location();
        Vector3D Axe = new(C.Axis().Direction());
        Vector3D Trsl = Axe.Multiplied(-(new Vector3D(O, P).Dot(Axe)));
        Point3D Pp = P.Translated(Trsl);

        // 2- Calculation of solutions ...

        double Tol2 = Tol * Tol;
        double R = C.MajorRadius();
        double r = C.MinorRadius();
        Vector3D OPp = new(O, Pp);
        double X = OPp.Dot(new Vector3D(C.XAxis().Direction()));
        double Y = OPp.Dot(new Vector3D(C.YAxis().Direction()));

        double C1 = (R * R + r * r) / 4.0;
        DirectPolynomialRoots Sol = new(C1, -(X * R + Y * r) / 2.0, 0.0, (X * R - Y * r) / 2.0, -C1);
        if (!Sol.IsDone)
        {
            return;
        }
        Point3D Cu;
        double Us, Vs;
        int NbSol = Sol.SolutionCount;
        bool DejaEnr;
        int NoExt;
        Span<Point3D> TbExt = [new(), new(), new(), new()];
        for (int NoSol = 1; NoSol <= NbSol; NoSol++)
        {
            Vs = Sol.Value(NoSol);
            if (Vs > 0.0)
            {
                Us = Math.Log(Vs);
                if ((Us >= Uinf) && (Us <= Usup))
                {
                    Cu = global::Kerf.Engine.Geometry.ElCLib.Value(Us, C);
                    DejaEnr = false;
                    for (NoExt = 0; NoExt < myNbExt; NoExt++)
                    {
                        if (TbExt[NoExt].SquareDistance(Cu) < Tol2)
                        {
                            DejaEnr = true;
                            break;
                        }
                    }
                    if (!DejaEnr)
                    {
                        TbExt[myNbExt] = Cu;
                        mySqDist[myNbExt] = Cu.SquareDistance(P);
                        myIsMin[myNbExt] = mySqDist[myNbExt] < P.SquareDistance(global::Kerf.Engine.Geometry.ElCLib.Value(Us + 1, C));
                        myPoint[myNbExt] = new Extrema_POnCurv(Us, Cu);
                        myNbExt++;
                    }
                } // if ((Us >= Uinf) && (Us <= Usup))
            } // if (Vs > 0.)
        } // for (int NoSol = 1; ...
        myDone = true;
    }

    /// <summary>
    /// The extremum distances between the point P and the segment [Uinf,Usup] of the parabola C. Two solutions are
    /// identical when their points are within <see cref="Precision.Confusion"/>; Tol is not used.
    /// </summary>
    public Extrema_ExtPElC(in Point3D P, in Parabola3D C, double Tol, double Uinf, double Usup) => Perform(P, C, Tol, Uinf, Usup);

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
    public void Perform(in Point3D P, in Parabola3D C, double Tol, double Uinf, double Usup)
    {
        _ = Tol; // OCCT: the tolerance is not used
        myDone = false;
        myNbExt = 0;

        // 1- Projection of point P in the plane of the parabola -> Pp ...

        Point3D O = C.Location();
        Vector3D Axe = new(C.Axis().Direction());
        Vector3D Trsl = Axe.Multiplied(-(new Vector3D(O, P).Dot(Axe)));
        Point3D Pp = P.Translated(Trsl);

        // 2- Calculation of solutions ...

        double F = C.Focal();
        Vector3D OPp = new(O, Pp);
        double X = OPp.Dot(new Vector3D(C.XAxis().Direction()));
        double Y = OPp.Dot(new Vector3D(C.YAxis().Direction()));
        DirectPolynomialRoots Sol = new(1.0 / (4.0 * F), 0.0, 2.0 * F - X, -2.0 * F * Y);
        if (!Sol.IsDone)
        {
            return;
        }
        Point3D Cu;
        double Us;
        int NbSol = Sol.SolutionCount;
        bool DejaEnr;
        int NoExt;
        Span<Point3D> TbExt = [new(), new(), new()];
        for (int NoSol = 1; NoSol <= NbSol; NoSol++)
        {
            Us = Sol.Value(NoSol);
            if ((Us >= Uinf) && (Us <= Usup))
            {
                Cu = global::Kerf.Engine.Geometry.ElCLib.Value(Us, C);
                DejaEnr = false;
                for (NoExt = 0; NoExt < myNbExt; NoExt++)
                {
                    if (TbExt[NoExt].SquareDistance(Cu) < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        DejaEnr = true;
                        break;
                    }
                }
                if (!DejaEnr)
                {
                    TbExt[myNbExt] = Cu;
                    mySqDist[myNbExt] = Cu.SquareDistance(P);
                    myIsMin[myNbExt] = mySqDist[myNbExt] < P.SquareDistance(global::Kerf.Engine.Geometry.ElCLib.Value(Us + 1, C));
                    myPoint[myNbExt] = new Extrema_POnCurv(Us, Cu);
                    myNbExt++;
                }
            } // if ((Us >= Uinf) && (Us <= Usup))
        } // for (int NoSol = 1; ...
        myDone = true;
    }

    /// <summary>True if the distances are found.</summary>
    public bool IsDone() => myDone;

    /// <summary>The number of extremum distances.</summary>
    public int NbExt()
    {
        if (!IsDone())
        {
            throw new NotDoneException();
        }
        return myNbExt;
    }

    /// <summary>The value of the Nth extremum square distance.</summary>
    public double SquareDistance(int N)
    {
        if ((N < 1) || (N > NbExt()))
        {
            throw new OutOfRangeException();
        }
        return mySqDist[N - 1];
    }

    /// <summary>True if the Nth extremum distance is a minimum.</summary>
    public bool IsMin(int N)
    {
        if ((N < 1) || (N > NbExt()))
        {
            throw new OutOfRangeException();
        }
        return myIsMin[N - 1];
    }

    /// <summary>The point of the Nth extremum distance.</summary>
    public Extrema_POnCurv Point(int N)
    {
        if ((N < 1) || (N > NbExt()))
        {
            throw new OutOfRangeException();
        }
        return myPoint[N - 1];
    }
}
