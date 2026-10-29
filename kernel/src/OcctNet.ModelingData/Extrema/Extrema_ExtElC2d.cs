// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtElC2d.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtElC2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// All the extremum distances (minimum or maximum) between two elementary 2D curves: a line or a circle and an
/// elementary curve.
/// </summary>
/// <remarks>An algorithm object computed by its constructor (OCCT gives it no Perform).</remarks>
public class Extrema_ExtElC2d
{
    private bool myDone;
    private bool myIsPar;
    private int myNbExt;
    private readonly double[] mySqDist = new double[8];

    // Extrema_POnCurv2d's default is all zeros, so the array needs no filling
    private readonly Extrema_POnCurv2d[,] myPoint = new Extrema_POnCurv2d[8, 2];

    public Extrema_ExtElC2d()
    {
        myDone = false;
        myIsPar = false;
        myNbExt = 0;
        for (int anIdx = 0; anIdx < mySqDist.Length; anIdx++)
        {
            mySqDist[anIdx] = RealLast();
        }
    }

    /*-----------------------------------------------------------------------------
    Function:
       Find min distance between 2 straight lines.

    Method:
      Let D1 and D2 be 2 directions of straight lines C1 and C2.
      2 cases are considered:
      1- if Angle(D1,D2) < AngTol, the straight lines are parallel.
         The distance is the distance between any point of C1 and straight line C2.
      2- if Angle(D1,D2) > AngTol:
         Let P = C1(u1) and P =C2(u2) the point intersection:

    -----------------------------------------------------------------------------*/

    /// <summary>
    /// The distance between two lines. AngTol is not used: the lines are parallel when their angle is below
    /// <see cref="Precision.Angular"/>.
    /// </summary>
    public Extrema_ExtElC2d(in gp_Lin2d C1, in gp_Lin2d C2, double AngTol)
    {
        _ = AngTol; // OCCT: the tolerance is not used
        myDone = false;
        myIsPar = false;
        myNbExt = 0;
        for (int anIdx = 0; anIdx < mySqDist.Length; anIdx++)
        {
            mySqDist[anIdx] = RealLast();
        }

        gp_Vec2d D1 = new(C1.Direction());
        gp_Vec2d D2 = new(C2.Direction());
        if (D1.IsParallel(D2, Precision.Angular()))
        {
            myIsPar = true;
            mySqDist[0] = C2.SquareDistance(C1.Location());
            myNbExt = 1;
        }
        else
        {
            // Vector from P1 to P2 (P2 - P1).
            gp_Vec2d aP1P2 = new(C1.Location(), C2.Location());

            // Solve linear system using Cramer's rule:
            // D1.X * t1 + D2.X * (-t2)  = P2.X - P1.X
            // D1.Y * t1 + D2.Y * (-t2)  = P2.Y - P1.Y

            // There is no division by zero since lines are not parallel.
            double aDelim = 1 / (D1 ^ D2);

            double aParam1 = (aP1P2 ^ D2) * aDelim;
            double aParam2 = -(D1 ^ aP1P2) * aDelim; // -1.0 coefficient before t2.

            gp_Pnt2d P1 = ElCLib.Value(aParam1, C1);
            gp_Pnt2d P2 = ElCLib.Value(aParam2, C2);

            mySqDist[myNbExt] = 0.0;
            myPoint[myNbExt, 0] = new Extrema_POnCurv2d(aParam1, P1);
            myPoint[myNbExt, 1] = new Extrema_POnCurv2d(aParam2, P2);
            myNbExt = 1;
        }

        myDone = true;
    }

    /*-----------------------------------------------------------------------------
    Function:
      Find extreme distances between straight line C1 and circle C2.

    Method:
      Let P1=C1(u1) and P2=C2(u2) be two solution points
            D the direction of straight line C1
        T the tangent at point P2;
      Then, ( P1P2.D = 0. (1)
             ( P1P2.T = 0. (2)
    -----------------------------------------------------------------------------*/

    /// <summary>The distances between a line and a circle. Tol is not used.</summary>
    public Extrema_ExtElC2d(in gp_Lin2d C1, in gp_Circ2d C2, double Tol)
    {
        _ = Tol; // OCCT: the tolerance is not used
        myIsPar = false;
        myDone = false;
        myNbExt = 0;
        for (int anIdx = 0; anIdx < mySqDist.Length; anIdx++)
        {
            mySqDist[anIdx] = RealLast();
        }

        // Calculate T1 in the reference of the circle ...
        gp_Dir2d D = C1.Direction();
        gp_Dir2d x2, y2;
        x2 = C2.XAxis().Direction();
        y2 = C2.YAxis().Direction();

        double Dx = D.Dot(x2);
        double Dy = D.Dot(y2);
        double U1;
        Span<double> teta = stackalloc double[2];
        gp_Pnt2d O1 = C1.Location();
        gp_Pnt2d P1, P2;

        if (Math.Abs(Dy) <= RealEpsilon())
        {
            teta[0] = M_PI / 2.0;
        }
        else
        {
            teta[0] = Math.Atan(-Dx / Dy);
        }
        teta[1] = teta[0] + M_PI;
        if (teta[0] < 0.0)
        {
            teta[0] = teta[0] + 2.0 * M_PI;
        }

        P2 = ElCLib.Value(teta[0], C2);
        U1 = (new gp_Vec2d(O1, P2)).Dot(new gp_Vec2d(D));
        P1 = ElCLib.Value(U1, C1);
        mySqDist[myNbExt] = P1.SquareDistance(P2);
        myPoint[myNbExt, 0] = new Extrema_POnCurv2d(U1, P1);
        myPoint[myNbExt, 1] = new Extrema_POnCurv2d(teta[0], P2);
        myNbExt++;

        P2 = ElCLib.Value(teta[1], C2);
        U1 = (new gp_Vec2d(O1, P2)).Dot(new gp_Vec2d(D));
        P1 = ElCLib.Value(U1, C1);
        mySqDist[myNbExt] = P1.SquareDistance(P2);
        myPoint[myNbExt, 0] = new Extrema_POnCurv2d(U1, P1);
        myPoint[myNbExt, 1] = new Extrema_POnCurv2d(teta[1], P2);
        myNbExt++;
        myDone = true;
    }

    /// <summary>The distances between a line and an ellipse.</summary>
    public Extrema_ExtElC2d(in gp_Lin2d C1, in gp_Elips2d C2)
    {
        myDone = true;
        myIsPar = false;
        myDone = false;
        myNbExt = 0;
        for (int anIdx = 0; anIdx < mySqDist.Length; anIdx++)
        {
            mySqDist[anIdx] = RealLast();
        }

        // Calculate T1 in the reference of the ellipse ...
        gp_Dir2d D = C1.Direction();
        gp_Dir2d x2, y2;
        x2 = C2.XAxis().Direction();
        y2 = C2.YAxis().Direction();

        double Dx = D.Dot(x2);
        double Dy = D.Dot(y2);
        double U1, r1 = C2.MajorRadius(), r2 = C2.MinorRadius();
        Span<double> teta = stackalloc double[2];
        gp_Pnt2d O1 = C1.Location(), P1, P2;

        if (Math.Abs(Dy) <= RealEpsilon())
        {
            teta[0] = M_PI / 2.0;
        }
        else
        {
            teta[0] = Math.Atan(-Dx * r2 / (Dy * r1));
        }

        teta[1] = teta[0] + M_PI;
        if (teta[0] < 0.0)
        {
            teta[0] += 2.0 * M_PI;
        }
        P2 = ElCLib.Value(teta[0], C2);
        U1 = (new gp_Vec2d(O1, P2)).Dot(new gp_Vec2d(D));
        P1 = ElCLib.Value(U1, C1);
        mySqDist[myNbExt] = P1.SquareDistance(P2);
        myPoint[myNbExt, 0] = new Extrema_POnCurv2d(U1, P1);
        myPoint[myNbExt, 1] = new Extrema_POnCurv2d(teta[0], P2);
        myNbExt++;

        P2 = ElCLib.Value(teta[1], C2);
        U1 = (new gp_Vec2d(O1, P2)).Dot(new gp_Vec2d(D));
        P1 = ElCLib.Value(U1, C1);
        mySqDist[myNbExt] = P1.SquareDistance(P2);
        myPoint[myNbExt, 0] = new Extrema_POnCurv2d(U1, P1);
        myPoint[myNbExt, 1] = new Extrema_POnCurv2d(teta[1], P2);
        myNbExt++;
        myDone = true;
    }

    /// <summary>The distance between a line and a hyperbola.</summary>
    public Extrema_ExtElC2d(in gp_Lin2d C1, in gp_Hypr2d C2)
    {
        myIsPar = false;
        myDone = false;
        myNbExt = 0;
        for (int anIdx = 0; anIdx < mySqDist.Length; anIdx++)
        {
            mySqDist[anIdx] = RealLast();
        }

        // Calculate T1 in the reference of the parabole ...
        gp_Dir2d D = C1.Direction();
        gp_Dir2d x2, y2;
        x2 = C2.XAxis().Direction();
        y2 = C2.YAxis().Direction();
        double Dx = D.Dot(x2);
        double Dy = D.Dot(y2);

        double U1, v2, U2 = 0, R = C2.MajorRadius(), r = C2.MinorRadius();
        gp_Pnt2d P1, P2;
        if (Math.Abs(Dy) < RealEpsilon())
        {
            return;
        }
        if (Math.Abs(R - r * Dx / Dy) < RealEpsilon())
        {
            return;
        }

        v2 = (R + r * Dx / Dy) / (R - r * Dx / Dy);
        if (v2 > 0.0)
        {
            U2 = Math.Log(Math.Sqrt(v2));
        }
        P2 = ElCLib.Value(U2, C2);

        U1 = (new gp_Vec2d(C1.Location(), P2)).Dot(new gp_Vec2d(D));
        P1 = ElCLib.Value(U1, C1);
        mySqDist[myNbExt] = P1.SquareDistance(P2);
        myPoint[myNbExt, 0] = new Extrema_POnCurv2d(U1, P1);
        myPoint[myNbExt, 1] = new Extrema_POnCurv2d(U2, P2);
        myNbExt++;
        myDone = true;
    }

    /// <summary>The distance between a line and a parabola.</summary>
    public Extrema_ExtElC2d(in gp_Lin2d C1, in gp_Parab2d C2)
    {
        myIsPar = false;
        myDone = false;
        myNbExt = 0;
        for (int anIdx = 0; anIdx < mySqDist.Length; anIdx++)
        {
            mySqDist[anIdx] = RealLast();
        }

        // Calculate  T1 in the reference of the parabole ...
        gp_Dir2d D = C1.Direction();
        gp_Dir2d x2, y2;
        x2 = C2.MirrorAxis().Direction();
        y2 = C2.Axis().YAxis().Direction();
        double Dx = D.Dot(x2);
        double Dy = D.Dot(y2);

        double U1, U2, P = C2.Parameter();
        gp_Pnt2d P1, P2;
        if (Math.Abs(Dy) < RealEpsilon())
        {
            return;
        }
        U2 = Dx * P / Dy;
        P2 = ElCLib.Value(U2, C2);

        U1 = (new gp_Vec2d(C1.Location(), P2)).Dot(new gp_Vec2d(D));
        P1 = ElCLib.Value(U1, C1);
        mySqDist[myNbExt] = P1.SquareDistance(P2);
        myPoint[myNbExt, 0] = new Extrema_POnCurv2d(U1, P1);
        myPoint[myNbExt, 1] = new Extrema_POnCurv2d(U2, P2);
        myNbExt++;
        myDone = true;
    }

    /// <summary>The distances between two circles. The circles can be parallel or identical.</summary>
    public Extrema_ExtElC2d(in gp_Circ2d C1, in gp_Circ2d C2)
    {
        myIsPar = false;
        myDone = false;
        myNbExt = 0;
        myDone = true;
        for (int anIdx = 0; anIdx < mySqDist.Length; anIdx++)
        {
            mySqDist[anIdx] = RealLast();
        }

        gp_Pnt2d O1 = C1.Location();
        gp_Pnt2d O2 = C2.Location();

        gp_Vec2d DO1O2 = new(O1, O2);
        double aSqDCenters = DO1O2.SquareMagnitude();
        if (aSqDCenters < Precision.SquareConfusion())
        {
            myIsPar = true;
            myNbExt = 1;
            myDone = true;
            double aDR = C1.Radius() - C2.Radius();
            mySqDist[0] = aDR * aDR;
            return;
        }

        int NoSol, kk;
        double U1, U2;
        double r1 = C1.Radius(), r2 = C2.Radius();
        Span<double> Usol2 = stackalloc double[2];
        Span<double> Usol1 = stackalloc double[2];
        Span<gp_Pnt2d> P1 = [new(), new()];
        Span<gp_Pnt2d> P2 = [new(), new()];
        gp_Vec2d O1O2 = DO1O2 / Math.Sqrt(aSqDCenters);

        P1[0] = O1.Translated(r1 * O1O2);
        Usol1[0] = ElCLib.Parameter(C1, P1[0]);
        P1[1] = O1.Translated(-r1 * O1O2);
        Usol1[1] = ElCLib.Parameter(C1, P1[1]);

        P2[0] = O2.Translated(r2 * O1O2);
        Usol2[0] = ElCLib.Parameter(C2, P2[0]);
        P2[1] = O2.Translated(-r2 * O1O2);
        Usol2[1] = ElCLib.Parameter(C2, P2[1]);

        for (NoSol = 0; NoSol <= 1; NoSol++)
        {
            U1 = Usol1[NoSol];
            for (kk = 0; kk <= 1; kk++)
            {
                U2 = Usol2[kk];
                mySqDist[myNbExt] = P2[kk].SquareDistance(P1[NoSol]);
                myPoint[myNbExt, 0] = new Extrema_POnCurv2d(U1, P1[NoSol]);
                myPoint[myNbExt, 1] = new Extrema_POnCurv2d(U2, P2[kk]);
                myNbExt++;
            }
        }
    }

    /// <summary>The distances between a circle and an ellipse.</summary>
    public Extrema_ExtElC2d(in gp_Circ2d C1, in gp_Elips2d C2)
    {
        myIsPar = false;
        myDone = false;
        myNbExt = 0;
        for (int anIdx = 0; anIdx < mySqDist.Length; anIdx++)
        {
            mySqDist[anIdx] = RealLast();
        }

        int i, j;

        Extrema_ExtPElC2d ExtElip = new(C1.Location(), C2, Precision.Confusion(), 0.0, 2.0 * M_PI);

        if (ExtElip.IsDone())
        {
            for (i = 1; i <= ExtElip.NbExt(); i++)
            {
                Extrema_ExtPElC2d ExtCirc = new(ExtElip.Point(i).Value(), C1, Precision.Confusion(), 0.0, 2.0 * M_PI);
                if (ExtCirc.IsDone())
                {
                    for (j = 1; j <= ExtCirc.NbExt(); j++)
                    {
                        mySqDist[myNbExt] = ExtCirc.SquareDistance(j);
                        myPoint[myNbExt, 0] = ExtCirc.Point(j);
                        myPoint[myNbExt, 1] = ExtElip.Point(i);
                        myNbExt++;
                    }
                }
                myDone = true;
            }
        }
    }

    /// <summary>The distances between a circle and a hyperbola.</summary>
    public Extrema_ExtElC2d(in gp_Circ2d C1, in gp_Hypr2d C2)
    {
        myIsPar = false;
        myDone = false;
        myNbExt = 0;
        for (int anIdx = 0; anIdx < mySqDist.Length; anIdx++)
        {
            mySqDist[anIdx] = RealLast();
        }

        int i, j;

        Extrema_ExtPElC2d ExtHyp = new(C1.Location(), C2, Precision.Confusion(), RealFirst(), RealLast());

        if (ExtHyp.IsDone())
        {
            for (i = 1; i <= ExtHyp.NbExt(); i++)
            {
                Extrema_ExtPElC2d ExtCirc = new(ExtHyp.Point(i).Value(), C1, Precision.Confusion(), 0.0, 2.0 * M_PI);
                if (ExtCirc.IsDone())
                {
                    for (j = 1; j <= ExtCirc.NbExt(); j++)
                    {
                        mySqDist[myNbExt] = ExtCirc.SquareDistance(j);
                        myPoint[myNbExt, 0] = ExtCirc.Point(j);
                        myPoint[myNbExt, 1] = ExtHyp.Point(i);
                        myNbExt++;
                    }
                }
                myDone = true;
            }
        }
    }

    /// <summary>The distances between a circle and a parabola.</summary>
    public Extrema_ExtElC2d(in gp_Circ2d C1, in gp_Parab2d C2)
    {
        myIsPar = false;
        myDone = false;
        myNbExt = 0;
        for (int anIdx = 0; anIdx < mySqDist.Length; anIdx++)
        {
            mySqDist[anIdx] = RealLast();
        }

        int i, j;

        Extrema_ExtPElC2d ExtParab = new(C1.Location(), C2, Precision.Confusion(), RealFirst(), RealLast());

        if (ExtParab.IsDone())
        {
            for (i = 1; i <= ExtParab.NbExt(); i++)
            {
                Extrema_ExtPElC2d ExtCirc = new(ExtParab.Point(i).Value(), C1, Precision.Confusion(), 0.0, 2.0 * M_PI);
                if (ExtCirc.IsDone())
                {
                    for (j = 1; j <= ExtCirc.NbExt(); j++)
                    {
                        mySqDist[myNbExt] = ExtCirc.SquareDistance(j);
                        myPoint[myNbExt, 0] = ExtCirc.Point(j);
                        myPoint[myNbExt, 1] = ExtParab.Point(i);
                        myNbExt++;
                    }
                }
                myDone = true;
            }
        }
    }

    /// <summary>True if the distances are found.</summary>
    public bool IsDone() => myDone;

    /// <summary>True if the two curves are parallel.</summary>
    public bool IsParallel()
    {
        if (!IsDone())
        {
            throw new StdFail_NotDone();
        }
        return myIsPar;
    }

    /// <summary>The number of extremum distances.</summary>
    public int NbExt()
    {
        if (!IsDone())
        {
            throw new StdFail_NotDone();
        }

        return myNbExt;
    }

    /// <summary>The value of the Nth extremum square distance.</summary>
    public double SquareDistance(int N = 1)
    {
        if (N < 1 || N > NbExt())
        {
            throw new Standard_OutOfRange();
        }

        return mySqDist[N - 1];
    }

    /// <summary>The points of the Nth extremum distance: P1 on the first curve, P2 on the second.</summary>
    public void Points(int N, out Extrema_POnCurv2d P1, out Extrema_POnCurv2d P2)
    {
        if (N < 1 || N > NbExt())
        {
            throw new Standard_OutOfRange();
        }
        P1 = myPoint[N - 1, 0];
        P2 = myPoint[N - 1, 1];
    }
}
