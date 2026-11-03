// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtPElC2d.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtPElC2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// All the extremum distances (minimum or maximum) between a 2D point and an elementary 2D curve (line, circle,
/// ellipse, hyperbola or parabola).
/// </summary>
/// <remarks>An algorithm object: construct it with the point and the curve, or empty and call <c>Perform</c>.</remarks>
public class Extrema_ExtPElC2d
{
    private bool myDone;
    private int myNbExt;
    private readonly double[] mySqDist = new double[4];
    private readonly bool[] myIsMin = new bool[4];
    private readonly Extrema_POnCurv2d[] myPoint = [new(), new(), new(), new()];

    public Extrema_ExtPElC2d()
    {
        myDone = false;
        myNbExt = 0;

        for (int i = 0; i < 4; i++)
        {
            mySqDist[i] = RealLast();
            myIsMin[i] = false;
        }
    }

    public Extrema_ExtPElC2d(in Point2D P, in Line2D C, double Tol, double Uinf, double Usup) => Perform(P, C, Tol, Uinf, Usup);

    public void Perform(in Point2D P, in Line2D L, double Tol, double Uinf, double Usup)
    {
        myDone = true;
        Point2D OR, MyP;
        myNbExt = 0;

        Vector2D V1 = new(L.Direction());
        OR = L.Location();
        Vector2D V = new(OR, P);
        double Mydist = V1.Dot(V);
        if ((Mydist >= Uinf - Tol) && (Mydist <= Usup + Tol))
        {
            myNbExt = 1;
            MyP = OR.Translated(Mydist * V1);
            Extrema_POnCurv2d MyPOnCurve = new(Mydist, MyP);
            mySqDist[0] = P.SquareDistance(MyP);
            myPoint[0] = MyPOnCurve;
            myIsMin[0] = true;
        }
    }

    public Extrema_ExtPElC2d(in Point2D P, in Circle2D C, double Tol, double Uinf, double Usup) => Perform(P, C, Tol, Uinf, Usup);

    public void Perform(in Point2D P, in Circle2D C, double Tol, double Uinf, double Usup)
    {
        Point2D OC = C.Location();
        myNbExt = 0;

        if (OC.IsEqual(P, global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            myDone = false;
        }
        else
        {
            double radius, U1, U2;
            Point2D P1, P2;

            myDone = true;
            Direction2D V = new(new Vector2D(P, OC));
            radius = C.Radius();
            P1 = OC.Translated(radius * new Vector2D(V));
            U1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(C, P1);
            U2 = U1 + M_PI;
            P2 = OC.Translated(-radius * new Vector2D(V));
            double myuinf = Uinf;
            global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(Uinf, Uinf + 2 * M_PI, global::Kerf.Engine.Foundation.Precision.PConfusion(), ref myuinf, ref U1);
            global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(Uinf, Uinf + 2 * M_PI, global::Kerf.Engine.Foundation.Precision.PConfusion(), ref myuinf, ref U2);
            if (((U1 - 2 * M_PI - Uinf) < Tol) && ((U1 - 2 * M_PI - Uinf) > -Tol))
            {
                U1 = Uinf;
                P1 = new Point2D(OC.XY()
                                  + radius * (Math.Cos(U1) * C.XAxis().Direction().XY() + Math.Sin(U1) * C.YAxis().Direction().XY()));
            }

            if (((U2 - 2 * M_PI - Uinf) < Tol) && ((U2 - 2 * M_PI - Uinf) > -Tol))
            {
                U2 = Uinf;
                P2 = new Point2D(OC.XY()
                                  + radius * (Math.Cos(U2) * C.XAxis().Direction().XY() + Math.Sin(U2) * C.YAxis().Direction().XY()));
            }

            if (((Uinf - U1) < Tol) && ((U1 - Usup) < Tol))
            {
                Extrema_POnCurv2d MyPOnCurve = new(U1, P1);
                mySqDist[0] = P.SquareDistance(P1);
                myPoint[0] = MyPOnCurve;
                myIsMin[0] = true;
                myNbExt++;
            }

            if (((Uinf - U2) < Tol) && ((U2 - Usup) < Tol))
            {
                Extrema_POnCurv2d MyPOnCurve = new(U2, P2);
                mySqDist[myNbExt] = P.SquareDistance(P2);
                myPoint[myNbExt] = MyPOnCurve;
                myIsMin[myNbExt] = true; // OCCT: the opposite point is a maximum
                myNbExt++;
            }
        }
    }

    public Extrema_ExtPElC2d(in Point2D P, in Ellipse2D E, double Tol, double Uinf, double Usup) => Perform(P, E, Tol, Uinf, Usup);

    public void Perform(in Point2D P, in Ellipse2D E, double Tol, double Uinf, double Usup)
    {
        myDone = false;
        myNbExt = 0;
        Point2D OR;
        OR = E.Location();

        int NoSol, NbSol;
        double A = E.MajorRadius();
        double B = E.MinorRadius();
        Vector2D V = new(OR, P);

        if (OR.IsEqual(P, global::Kerf.Engine.Foundation.Precision.Confusion()) && (Math.Abs(A - B) <= Tol))
        {
            return;
        }
        else
        {
            double X = V.Dot(new Vector2D(E.XAxis().Direction()));
            double Y = V.Dot(new Vector2D(E.YAxis().Direction()));

            TrigonometricFunctionRoots Sol = new(0.0, (B * B - A * A) / 2.0, -B * Y, A * X, 0.0, Uinf, Usup);

            if (!Sol.IsDone)
            {
                return;
            }
            Point2D Cu;
            double Us;
            NbSol = Sol.SolutionCount;
            myNbExt = 0;
            for (NoSol = 1; NoSol <= NbSol; NoSol++)
            {
                Us = Sol.Value(NoSol);
                Cu = global::Kerf.Engine.Geometry.ElCLib.Value(Us, E);
                mySqDist[myNbExt] = Cu.SquareDistance(P);
                myIsMin[myNbExt] = (NoSol == 0); // OCCT: always false (NoSol starts at 1)
                myPoint[myNbExt] = new Extrema_POnCurv2d(Us, Cu);
                myNbExt++;
            }
            myDone = true;
        }
    }

    public Extrema_ExtPElC2d(in Point2D P, in Hyperbola2D C, double Tol, double Uinf, double Usup) => Perform(P, C, Tol, Uinf, Usup);

    public void Perform(in Point2D P, in Hyperbola2D H, double Tol, double Uinf, double Usup)
    {
        Point2D O = H.Location();
        myDone = false;
        myNbExt = 0;

        double R = H.MajorRadius();
        double r = H.MinorRadius();
        Vector2D OPp = new(O, P);
        double Tol2 = Tol * Tol;
        double X = OPp.Dot(new Vector2D(H.XAxis().Direction()));
        double Y = OPp.Dot(new Vector2D(H.YAxis().Direction()));
        double C1 = (R * R + r * r) / 4.0;
        DirectPolynomialRoots Sol = new(C1, -(X * R + Y * r) / 2.0, 0.0, (X * R - Y * r) / 2.0, -C1);
        if (!Sol.IsDone)
        {
            return;
        }
        Point2D Cu;
        double Us, Vs;
        int NbSol = Sol.SolutionCount;
        bool DejaEnr;
        int NoExt;
        Span<Point2D> TbExt = [new(), new(), new(), new()];
        for (int NoSol = 1; NoSol <= NbSol; NoSol++)
        {
            Vs = Sol.Value(NoSol);
            if (Vs > 0.0)
            {
                Us = Math.Log(Vs);
                if ((Us >= Uinf) && (Us <= Usup))
                {
                    Cu = global::Kerf.Engine.Geometry.ElCLib.Value(Us, H);
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
                        myIsMin[myNbExt] = (NoSol == 0); // OCCT: always false (NoSol starts at 1)
                        myPoint[myNbExt] = new Extrema_POnCurv2d(Us, Cu);
                        myNbExt++;
                    }
                } // if ((Us >= Uinf) && (Us <= Usup))
            } // if (Vs > 0.)
        } // for (int NoSol = 1; ...
        myDone = true;
    }

    public Extrema_ExtPElC2d(in Point2D P, in Parabola2D C, double Tol, double Uinf, double Usup) => Perform(P, C, Tol, Uinf, Usup);

    public void Perform(in Point2D P, in Parabola2D C, double Tol, double Uinf, double Usup)
    {
        myDone = false;
        myNbExt = 0;
        Point2D O = C.Location();

        double Tol2 = Tol * Tol;
        double F = C.Focal();
        Vector2D OPp = new(O, P);
        double X = OPp.Dot(new Vector2D(C.MirrorAxis().Direction()));
        double Y = OPp.Dot(new Vector2D(C.Axis().YAxis().Direction()));

        DirectPolynomialRoots Sol = new(1.0 / (4.0 * F), 0.0, 2.0 * F - X, -2.0 * F * Y);
        if (!Sol.IsDone)
        {
            return;
        }
        Point2D Cu;
        double Us;
        int NbSol = Sol.SolutionCount;
        bool DejaEnr;
        int NoExt;
        Span<Point2D> TbExt = [new(), new(), new()];
        for (int NoSol = 1; NoSol <= NbSol; NoSol++)
        {
            Us = Sol.Value(NoSol);
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
                    myIsMin[myNbExt] = (NoSol == 0); // OCCT: always false (NoSol starts at 1)
                    myPoint[myNbExt] = new Extrema_POnCurv2d(Us, Cu);
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
    public Extrema_POnCurv2d Point(int N)
    {
        if ((N < 1) || (N > NbExt()))
        {
            throw new OutOfRangeException();
        }
        return myPoint[N - 1];
    }
}
