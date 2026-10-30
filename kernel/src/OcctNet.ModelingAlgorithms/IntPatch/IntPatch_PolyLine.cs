// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_PolyLine.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_PolyLine.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.IntPatch_IType;

namespace Occt;

/// <summary>
/// The 2D polygon of a walking or restriction line in the parametric space of one of its surfaces, with its
/// deflection (estimated from parabolas through consecutive points). The end points of a walking line are pushed
/// out by 1e-7 of the end segments.
/// </summary>
/// <remarks>
/// OCCT's unused field <c>pnt</c> is left out. OCCT leaves the line type uninitialised until a line is set; the
/// port's is IntPatch_Lin (and the queries fail on the missing line, as OCCT's would).
/// </remarks>
public class IntPatch_PolyLine : IntPatch_Polygo
{
    //-- lbr le 12 juin : Ajout des fleches sur les Lines
    //-- msv 13.03.2002 : compute deflection for WLine; Error() returns deflection
    private static double INITDEFLE => Precision.PConfusion() * 100.0;

    private IntPatch_IType typ;
    private bool onfirst;
    private IntPatch_WLine? wpoly;
    private IntPatch_RLine? rpoly;

    public IntPatch_PolyLine()
        : base(INITDEFLE)
    {
        onfirst = false;
    }

    /// <summary>A polygon whose initial deflection is <paramref name="InitDefle"/>.</summary>
    public IntPatch_PolyLine(double InitDefle)
        : base(InitDefle)
    {
        onfirst = false;
    }

    /// <summary>Takes the points of a walking line on the first surface (<paramref name="OnFirst"/>) or the second.</summary>
    public void SetWLine(bool OnFirst, IntPatch_WLine Line)
    {
        typ = IntPatch_Walking;
        wpoly = Line;
        onfirst = OnFirst;
        Prepare();
    }

    /// <summary>Takes the points of a restriction line on the first surface (<paramref name="OnFirst"/>) or the second.</summary>
    public void SetRLine(bool OnFirst, IntPatch_RLine Line)
    {
        typ = IntPatch_Restriction;
        rpoly = Line;
        onfirst = OnFirst;
        Prepare();
    }

    private void Prepare()
    {
        int i;
        myBox.SetVoid();
        int n = NbPoints();
        double eps_2 = myError * myError;

        gp_Pnt2d P1 = new(), P2 = new();
        if (n >= 3)
        {
            P1 = Point(1);
            P2 = Point(2);
        }
        for (i = 1; i <= n; i++)
        {
            gp_Pnt2d P3 = Point(i);
            if (i >= 3)
            {
                gp_XY V13 = P3.XY() - P1.XY();
                gp_XY V12 = P2.XY() - P1.XY();
                double d13_2 = V13.SquareModulus(), d_2;
                if (d13_2 > eps_2)
                {
                    d_2 = V13.CrossSquareMagnitude(V12) / d13_2;
                }
                else
                {
                    d_2 = eps_2;
                }
                if (d_2 > myError * myError)
                {
                    // try to compute deflection more precisely using parabola interpolation
                    gp_XY V23 = P3.XY() - P2.XY();
                    double d12 = V12.Modulus(), d23 = V23.Modulus();
                    // compute parameter of P2 (assume parameters of P1,P3 are 0,1)
                    double tm = d12 / (d12 + d23);
                    if (tm > 0.1 && tm < 0.9)
                    {
                        tm -= (tm - 0.5) * 0.6;
                        double tm1mtm = tm * (1 - tm);
                        // coefficients of parabola
                        double Ax = (tm * V13.X() - V12.X()) / tm1mtm;
                        double Bx = (V12.X() - tm * tm * V13.X()) / tm1mtm;
                        double Cx = P1.X();
                        double Ay = (tm * V13.Y() - V12.Y()) / tm1mtm;
                        double By = (V12.Y() - tm * tm * V13.Y()) / tm1mtm;
                        double Cy = P1.Y();
                        // equations of lines P1-P2 and P2-P3
                        double A1 = V12.Y() / d12;
                        double B1 = -V12.X() / d12;
                        double C1 = (P2.X() * P1.Y() - P1.X() * P2.Y()) / d12;
                        double A2 = V23.Y() / d23;
                        double B2 = -V23.X() / d23;
                        double C2 = (P3.X() * P2.Y() - P2.X() * P3.Y()) / d23;
                        // points on parabola with max deflection
                        double t1 = -0.5 * (A1 * Bx + B1 * By) / (A1 * Ax + B1 * Ay);
                        double t2 = -0.5 * (A2 * Bx + B2 * By) / (A2 * Ax + B2 * Ay);
                        double xt1 = Ax * t1 * t1 + Bx * t1 + Cx;
                        double yt1 = Ay * t1 * t1 + By * t1 + Cy;
                        double xt2 = Ax * t2 * t2 + Bx * t2 + Cx;
                        double yt2 = Ay * t2 * t2 + By * t2 + Cy;
                        // max deflection on segments P1-P2 and P2-P3
                        double d1 = Math.Abs(A1 * xt1 + B1 * yt1 + C1);
                        double d2 = Math.Abs(A2 * xt2 + B2 * yt2 + C2);
                        if (d2 > d1)
                        {
                            d1 = d2;
                        }
                        // select min deflection from linear and parabolic ones
                        if (d1 * d1 < d_2)
                        {
                            d_2 = d1 * d1;
                        }
                    }
                    if (d_2 > myError * myError)
                    {
                        myError = Math.Sqrt(d_2);
                    }
                }
                P1 = P2;
                P2 = P3;
            }
            myBox.Add(P3);
        }
        myBox.Enlarge(myError);
    }

    /// <summary>Resets the deflection to its initial value (100 times Precision.PConfusion()).</summary>
    public void ResetError() => myError = INITDEFLE;

    public override int NbPoints() => (typ == IntPatch_Walking ? wpoly!.NbPnts() : rpoly!.NbPnts());

    public override gp_Pnt2d Point(int Index)
    {
        double X, Y, X1, Y1, DX, DY;
        DX = DY = 0;
        if (onfirst)
        {
            if (typ == IntPatch_Walking)
            {
                wpoly!.Point(Index).ParametersOnS1(out X, out Y);
                if (Index == 1)
                {
                    wpoly.Point(2).ParametersOnS1(out X1, out Y1);
                    DX = 0.0000001 * (X - X1);
                    DY = 0.0000001 * (Y - Y1);
                }
                else if (Index == wpoly.NbPnts())
                {
                    wpoly.Point(Index - 1).ParametersOnS1(out X1, out Y1);
                    DX = 0.0000001 * (X - X1);
                    DY = 0.0000001 * (Y - Y1);
                }
            }
            else
            {
                rpoly!.Point(Index).ParametersOnS1(out X, out Y);
            }
        }
        else
        {
            if (typ == IntPatch_Walking)
            {
                wpoly!.Point(Index).ParametersOnS2(out X, out Y);
                if (Index == 1)
                {
                    wpoly.Point(2).ParametersOnS2(out X1, out Y1);
                    DX = 0.0000001 * (X - X1);
                    DY = 0.0000001 * (Y - Y1);
                }
                else if (Index == wpoly.NbPnts())
                {
                    wpoly.Point(Index - 1).ParametersOnS2(out X1, out Y1);
                    DX = 0.0000001 * (X - X1);
                    DY = 0.0000001 * (Y - Y1);
                }
            }
            else
            {
                rpoly!.Point(Index).ParametersOnS2(out X, out Y);
            }
        }

        return (new gp_Pnt2d(X + DX, Y + DY));
    }
}
