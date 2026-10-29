// Port of: src/ModelingData/TKGeomBase/Hermit/Hermit.hxx
// Port of: src/ModelingData/TKGeomBase/Hermit/Hermit.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Reparameterizes rational B-spline curves so that they can later be concatenated into C1 curves. It builds a 1D
/// reparameterizing function a(u) from a Hermite interpolation, inserting knots into and moving poles of the 1D
/// B-spline obtained that way, so that for a curve f(u) = N(u) / D(u) the function a(u)D(u) has value 1 at umin
/// and umax and a zero derivative there. See "Etude sur la concatenation de NURBS en vue du balayage de surfaces",
/// PFE n S85, Ensam Lille.
/// </summary>
/// <remarks>
/// Only the curves built here are modified; the curve passed in is only read. The 3D and 2D overloads of the
/// helpers are duplicated, as in OCCT.
/// </remarks>
public static class Hermit
{
    /// <summary>Calculates the Hermite coefficients of degree 3 from BS and stores them in TAB (4 coefficients).</summary>
    private static void HermiteCoeff(Geom_BSplineCurve BS, NCollection_Array1<double> TAB)
    {
        NCollection_Array1<double> Knots = new(BS.InternalKnots());
        NCollection_Array1<double> Weights = BS.InternalWeightsArray();
        NCollection_Array1<int> Mults = BS.InternalMultiplicities();
        int Degree, Index0, Index1;   // denominateur value for u=0 & u=1
        double Denom0, Denom1,        // denominator value for u=0 & u=1
            Deriv0, Deriv1;           // derivative denominator value for u=0 & 1
        bool Periodic;

        BSplCLib.Reparametrize(0.0, 1.0, Knots); // affinity on the nodal vector
        Degree = BS.Degree();
        Periodic = BS.IsPeriodic();
        Index0 = BS.FirstUKnotIndex();
        Index1 = BS.LastUKnotIndex() - 1;

        // Evaluate the weight function w(u) = sum_i N_i,p(u) * w_i and its derivative
        // at u=0 and u=1. The weight function of a rational BSpline is itself a polynomial
        // BSpline of the same degree, with weight values as scalar "poles" and no rational
        // denominator -- hence Weights is passed as Poles, with NoWeights() for unweighted evaluation.
        BSplCLib.D1(0.0, Index0, Degree, Periodic, Weights, BSplCLib.NoWeights(), Knots, Mults, out Denom0, out Deriv0);
        BSplCLib.D1(1.0, Index1, Degree, Periodic, Weights, BSplCLib.NoWeights(), Knots, Mults, out Denom1, out Deriv1);
        TAB[0] = 1 / Denom0; // Hermit coefficients
        TAB[1] = -Deriv0 / (Denom0 * Denom0);
        TAB[2] = -Deriv1 / (Denom1 * Denom1);
        TAB[3] = 1 / Denom1;
    }

    /// <summary>Calculates the Hermite coefficients of degree 3 from BS and stores them in TAB (4 coefficients).</summary>
    private static void HermiteCoeff(Geom2d_BSplineCurve BS, NCollection_Array1<double> TAB)
    {
        NCollection_Array1<double> Knots = new(BS.InternalKnots());
        NCollection_Array1<double> Weights = BS.InternalWeightsArray();
        NCollection_Array1<int> Mults = BS.InternalMultiplicities();
        int Degree, Index0, Index1;   // denominateur value for u=0 & u=1
        double Denom0, Denom1,        // denominator value for u=0 & u=1
            Deriv0, Deriv1;           // derivative denominator value for u=0 & 1
        bool Periodic;

        BSplCLib.Reparametrize(0.0, 1.0, Knots); // affinity on the nodal vector
        Degree = BS.Degree();
        Periodic = BS.IsPeriodic();
        Index0 = BS.FirstUKnotIndex();
        Index1 = BS.LastUKnotIndex() - 1;

        // Evaluate the weight function w(u) = sum_i N_i,p(u) * w_i and its derivative
        // at u=0 and u=1. The weight function of a rational BSpline is itself a polynomial
        // BSpline of the same degree, with weight values as scalar "poles" and no rational
        // denominator -- hence Weights is passed as Poles, with NoWeights() for unweighted evaluation.
        BSplCLib.D1(0.0, Index0, Degree, Periodic, Weights, BSplCLib.NoWeights(), Knots, Mults, out Denom0, out Deriv0);
        BSplCLib.D1(1.0, Index1, Degree, Periodic, Weights, BSplCLib.NoWeights(), Knots, Mults, out Denom1, out Deriv1);
        TAB[0] = 1 / Denom0; // Hermit coefficients
        TAB[1] = -Deriv0 / (Denom0 * Denom0);
        TAB[2] = -Deriv1 / (Denom1 * Denom1);
        TAB[3] = 1 / Denom1;
    }

    /// <summary>Gives the sign of Herm(0): true for positive.</summary>
    private static bool SignDenom(NCollection_Array1<gp_Pnt2d> Poles)
    {
        bool Result;

        Result = Poles[0].Y() >= 0;
        return Result;
    }

    /// <summary>Gives the max and the min of the Poles (by their index).</summary>
    private static void Polemax(NCollection_Array1<gp_Pnt2d> Poles, out int min, out int max)
    {
        int i;
        double Max, Min; // intermediate value of max and min ordinates
        min = 0;         // initialisation of the indices
        max = 0;

        Min = Poles[0].Y(); // initialisation of the intermediate value
        Max = Poles[0].Y();
        for (i = 1; i <= (Poles.Length() - 1); i++)
        {
            if (Poles[i].Y() < Min)
            {
                Min = Poles[i].Y();
                min = i;
            }
            if (Poles[i].Y() > Max)
            {
                Max = Poles[i].Y();
                max = i;
            }
        }
    }

    /// <summary>Gives the knots U4 and U5 to insert to a(u).</summary>
    private static void PolyTest(NCollection_Array1<double> Herm,
                                 Geom_BSplineCurve BS,
                                 out double U4,
                                 out double U5,
                                 ref int boucle,
                                 double TolPoles,
                                 double TolKnots,
                                 double Ux,
                                 double Uy)
    {
        _ = TolKnots; // unused, as in OCCT
        int index, i, I1 = 0, I2 = 0, I3 = 0, I4 = 0; // knots index
        NCollection_Array1<gp_Pnt2d> Polesinit = new(0, 3);
        NCollection_HArray1<double> Knots; // array of the BSpline knots + the ones inserted
        int cas = 0, mark = 0, dercas = 0, // loop marks
            min, max;                      // Pole min and max indices
        double Us1, Us2, a;                // boundaries value of the knots to be inserted

        U4 = 0.0;
        U5 = 1.0; // default value
        if (Ux != 1.0)
        {
            BS.LocateU(Ux, 0.0, out I1, out I2); // localization of the inserted knots
            if (Uy != 0.0)
            {
                BS.LocateU(Uy, 0.0, out I3, out I4);
            }
        }

        if (I1 == I2)
        { // definition and filling of the
            if ((I3 == I4) || (I3 == 0))
            { // array of knots
                Knots = new NCollection_HArray1<double>(1, BS.NbKnots());
                for (i = 1; i <= BS.NbKnots(); i++)
                {
                    Knots.SetValue(i, BS.Knot(i));
                }
            }
            else
            {
                Knots = new NCollection_HArray1<double>(1, BS.NbKnots() + 1);
                for (i = 1; i <= BS.NbKnots(); i++)
                {
                    Knots.SetValue(i, BS.Knot(i));
                }
                Knots.SetValue(BS.NbKnots() + 1, Uy);
            }
        }
        else
        {
            if ((I3 == I4) || (I3 == 0))
            {
                Knots = new NCollection_HArray1<double>(1, BS.NbKnots() + 1);
                for (i = 1; i <= BS.NbKnots(); i++)
                {
                    Knots.SetValue(i, BS.Knot(i));
                }
                Knots.SetValue(BS.NbKnots() + 1, Ux);
            }
            else
            {
                Knots = new NCollection_HArray1<double>(1, BS.NbKnots() + 2);
                for (i = 1; i <= BS.NbKnots(); i++)
                {
                    Knots.SetValue(i, BS.Knot(i));
                }
                Knots.SetValue(BS.NbKnots() + 1, Ux);
                Knots.SetValue(BS.NbKnots() + 2, Uy);
            }
        }

        NCollection_Array1<double> knots = new(1, Knots.Length());
        knots.Assign(Knots.ChangeArray1());

        // sort of the array of knots
        knots.AsSpan().Sort();

        Polesinit[0].SetCoord(0.0, Herm[0]); // poles of the Hermite polynome in the BSpline form
        Polesinit[1].SetCoord(0.0, Herm[0] + Herm[1] / 3.0);
        Polesinit[2].SetCoord(0.0, Herm[3] - Herm[2] / 3.0);
        Polesinit[3].SetCoord(0.0, Herm[3]);

        // loop to check the tolerances on poles
        if (TolPoles != 0.0)
        {
            Hermit.Polemax(Polesinit, out min, out max);
            double Polemin = Polesinit[min].Y();
            double Polemax = Polesinit[max].Y();
            if (((Polemax) >= ((1 / TolPoles) * Polemin))
                || ((Polemin == 0.0) && (Polemax >= (1 / TolPoles))))
            {
                if (Polesinit[0].Y() >= (1 / TolPoles) * Polesinit[3].Y()
                    || Polesinit[0].Y() <= TolPoles * Polesinit[3].Y())
                {
                    throw new Standard_DimensionError("Hermit Impossible Tolerance");
                }
                if ((max == 0) || (max == 3))
                {
                    for (i = 0; i <= 3; i++)
                    {
                        Polesinit[i].SetCoord(0.0, (Polesinit[i].Y() - TolPoles * Polemax));
                    }
                }
                else if ((max == 1) || (max == 2))
                {
                    if ((min == 0) || (min == 3))
                    {
                        for (i = 0; i <= 3; i++)
                        {
                            Polesinit[i].SetCoord(0.0, (Polesinit[i].Y() - (1 / TolPoles) * Polemin));
                        }
                    }
                    else
                    {
                        if ((TolPoles * Polemax < Polesinit[0].Y()) && (TolPoles * Polemax < Polesinit[3].Y()))
                        {
                            for (i = 0; i <= 3; i++)
                            {
                                Polesinit[i].SetCoord(0.0, (Polesinit[i].Y() - TolPoles * Polemax));
                            }
                            mark = 1;
                        }
                        if ((1 / TolPoles * Polemin > Polesinit[0].Y())
                            && (1 / TolPoles * Polemin > Polesinit[3].Y()) && (mark == 0))
                        {
                            for (i = 0; i <= 3; i++)
                            {
                                Polesinit[i].SetCoord(0.0, (Polesinit[i].Y() - 1 / TolPoles * Polemin));
                            }
                            mark = 1;
                        }
                        if (mark == 0)
                        {
                            double Pole0, Pole3;
                            Pole0 = Polesinit[0].Y();
                            Pole3 = Polesinit[3].Y();
                            if (Pole0 < Pole3)
                            {
                                a = Math.Log10(Pole3 / Pole0);
                                if (boucle == 2)
                                {
                                    for (i = 0; i <= 3; i++)
                                    {
                                        Polesinit[i].SetCoord(
                                            0.0,
                                            Polesinit[i].Y()
                                                - (Pole3 * (Math.Pow(10.0, (-0.5 * Math.Log10(TolPoles) - a / 2.0)))));
                                    }
                                }
                                else if (boucle == 1)
                                {
                                    for (i = 0; i <= 3; i++)
                                    {
                                        Polesinit[i].SetCoord(
                                            0.0,
                                            Polesinit[i].Y()
                                                - (Pole0 * (Math.Pow(10.0, (a / 2.0 + 0.5 * Math.Log10(TolPoles))))));
                                    }
                                    dercas = 1;
                                }
                            }
                            if (Pole0 > Pole3)
                            {
                                a = Math.Log10(Pole0 / Pole3);
                                if (boucle == 2)
                                {
                                    for (i = 0; i <= 3; i++)
                                    {
                                        Polesinit[i].SetCoord(
                                            0.0,
                                            Polesinit[i].Y()
                                                - (Pole0 * (Math.Pow(10.0, (-0.5 * Math.Log10(TolPoles) - a / 2.0)))));
                                    }
                                }
                                else if (boucle == 1)
                                {
                                    for (i = 0; i <= 3; i++)
                                    {
                                        Polesinit[i].SetCoord(
                                            0.0,
                                            Polesinit[i].Y()
                                                - (Pole3 * (Math.Pow(10.0, (a / 2.0 + 0.5 * Math.Log10(TolPoles))))));
                                    }
                                    dercas = 1;
                                }
                            }
                        }
                    }
                }
            }
        } // end of the loop

        if (!SignDenom(Polesinit)) // inversion of the polynome sign
        {
            for (index = 0; index <= 3; index++)
            {
                Polesinit[index].SetCoord(0.0, -Polesinit[index].Y());
            }
        }

        // loop of positivity
        if ((Polesinit[1].Y() < 0.0) && (Polesinit[2].Y() >= 0.0))
        {
            Us1 = Polesinit[0].Y() / (Polesinit[0].Y() - Polesinit[1].Y());
            if (boucle == 2)
            {
                Us1 = Us1 * knots[2];
            }
            if (boucle == 1)
            {
                if (Ux != 0.0)
                {
                    Us1 = Us1 * Ux;
                }
            }
            BSplCLib.LocateParameter(3, knots, Us1, false, 1, knots.Length(), out I1, out Us1);
            if (I1 < 2)
            {
                U4 = Us1;
            }
            else
            {
                U4 = knots[I1];
            }
        }

        if ((Polesinit[1].Y() >= 0.0) && (Polesinit[2].Y() < 0.0))
        {
            Us2 = Polesinit[2].Y() / (Polesinit[2].Y() - Polesinit[3].Y());
            if (boucle == 2)
            {
                Us2 = knots[knots.Length() - 1] + Us2 * (1 - knots[knots.Length() - 1]);
            }
            if (boucle == 1)
            {
                if (Ux != 0.0)
                {
                    Us2 = Uy + Us2 * (1 - Uy);
                }
            }
            BSplCLib.LocateParameter(3, knots, Us2, false, 1, knots.Length(), out I1, out Us2);
            if (I1 >= (knots.Length() - 1))
            {
                U5 = Us2;
            }
            else
            {
                U5 = knots[I1 + 1];
            }
        }

        if (dercas == 1)
        {
            boucle++;
        }

        if ((Polesinit[1].Y() < 0.0) && (Polesinit[2].Y() < 0.0))
        {
            Us1 = Polesinit[0].Y() / (Polesinit[0].Y() - Polesinit[1].Y());
            Us2 = Polesinit[2].Y() / (Polesinit[2].Y() - Polesinit[3].Y());
            if (boucle != 0)
            {
                if (Ux != 0.0)
                {
                    Us1 = Us1 * Ux;
                    Us2 = Uy + Us2 * (1 - Uy);
                }
            }
            if (Us2 <= Us1)
            {
                BSplCLib.LocateParameter(3, knots, Us1, false, 1, knots.Length(), out I1, out Us1);
                if (knots[I1] >= Us2)
                { // insertion of one knot for the two poles
                    U4 = knots[I1];
                }
                else
                {
                    if (I1 >= 2)
                    {                   // insertion to the left and
                        U4 = knots[I1]; // to the right without a new knot
                        BSplCLib.LocateParameter(3, knots, Us2, false, 1, knots.Length(), out I3, out Us2);
                        if (I3 < (BS.NbKnots() - 1))
                        {
                            U5 = knots[I3 + 1];
                            cas = 1;
                        }
                    }
                    if (cas == 0)
                    { // insertion of only one new knot
                        U4 = (Us1 + Us2) / 2;
                    }
                }
            }
            else
            { // insertion of two knots
                BSplCLib.LocateParameter(3, knots, Us1, false, 1, knots.Length(), out I1, out Us1);
                if (I1 >= 2)
                {
                    U4 = knots[I1];
                }
                else
                {
                    U4 = Us1;
                }
                BSplCLib.LocateParameter(3, knots, Us2, false, 1, knots.Length(), out I3, out Us2);
                if (I3 < (BS.NbKnots() - 1))
                {
                    U5 = knots[I3 + 1];
                }
                else
                {
                    U5 = Us2;
                }
            }
        }
    }

    /// <summary>Gives the knots U4 and U5 to insert to a(u).</summary>
    private static void PolyTest(NCollection_Array1<double> Herm,
                                 Geom2d_BSplineCurve BS,
                                 out double U4,
                                 out double U5,
                                 ref int boucle,
                                 double TolPoles,
                                 double TolKnots,
                                 double Ux,
                                 double Uy)
    {
        _ = TolKnots; // unused, as in OCCT
        int index, i, I1 = 0, I2 = 0, I3 = 0, I4 = 0; // knots index
        NCollection_Array1<gp_Pnt2d> Polesinit = new(0, 3);
        NCollection_HArray1<double> Knots; // array of the BSpline knots + the ones inserted
        int cas = 0, mark = 0, dercas = 0, // loop marks
            min, max;                      // Pole min and max indices
        double Us1, Us2, a;                // boundaries value of the knots to be inserted

        U4 = 0.0;
        U5 = 1.0; // default value
        if (Ux != 1.0)
        {
            BS.LocateU(Ux, 0.0, out I1, out I2); // localization of the inserted knots
            if (Uy != 0.0)
            {
                BS.LocateU(Uy, 0.0, out I3, out I4);
            }
        }

        if (I1 == I2)
        { // definition and filling of the
            if ((I3 == I4) || (I3 == 0))
            { // array of knots
                Knots = new NCollection_HArray1<double>(1, BS.NbKnots());
                for (i = 1; i <= BS.NbKnots(); i++)
                {
                    Knots.SetValue(i, BS.Knot(i));
                }
            }
            else
            {
                Knots = new NCollection_HArray1<double>(1, BS.NbKnots() + 1);
                for (i = 1; i <= BS.NbKnots(); i++)
                {
                    Knots.SetValue(i, BS.Knot(i));
                }
                Knots.SetValue(BS.NbKnots() + 1, Uy);
            }
        }
        else
        {
            if ((I3 == I4) || (I3 == 0))
            {
                Knots = new NCollection_HArray1<double>(1, BS.NbKnots() + 1);
                for (i = 1; i <= BS.NbKnots(); i++)
                {
                    Knots.SetValue(i, BS.Knot(i));
                }
                Knots.SetValue(BS.NbKnots() + 1, Ux);
            }
            else
            {
                Knots = new NCollection_HArray1<double>(1, BS.NbKnots() + 2);
                for (i = 1; i <= BS.NbKnots(); i++)
                {
                    Knots.SetValue(i, BS.Knot(i));
                }
                Knots.SetValue(BS.NbKnots() + 1, Ux);
                Knots.SetValue(BS.NbKnots() + 2, Uy);
            }
        }

        NCollection_Array1<double> knots = new(1, Knots.Length());
        knots.Assign(Knots.ChangeArray1());

        // sort of the array of knots
        knots.AsSpan().Sort();

        Polesinit[0].SetCoord(0.0, Herm[0]); // poles of the Hermite polynome in the BSpline form
        Polesinit[1].SetCoord(0.0, Herm[0] + Herm[1] / 3.0);
        Polesinit[2].SetCoord(0.0, Herm[3] - Herm[2] / 3.0);
        Polesinit[3].SetCoord(0.0, Herm[3]);

        // loop to check the tolerances on poles
        if (TolPoles != 0.0)
        {
            Hermit.Polemax(Polesinit, out min, out max);
            double Polemin = Polesinit[min].Y();
            double Polemax = Polesinit[max].Y();
            if (((Polemax) >= ((1 / TolPoles) * Polemin))
                || ((Polemin == 0.0) && (Polemax >= (1 / TolPoles))))
            {
                if (Polesinit[0].Y() >= (1 / TolPoles) * Polesinit[3].Y()
                    || Polesinit[0].Y() <= TolPoles * Polesinit[3].Y())
                {
                    throw new Standard_DimensionError("Hermit Impossible Tolerance");
                }
                if ((max == 0) || (max == 3))
                {
                    for (i = 0; i <= 3; i++)
                    {
                        Polesinit[i].SetCoord(0.0, (Polesinit[i].Y() - TolPoles * Polemax));
                    }
                }
                else if ((max == 1) || (max == 2))
                {
                    if ((min == 0) || (min == 3))
                    {
                        for (i = 0; i <= 3; i++)
                        {
                            Polesinit[i].SetCoord(0.0, (Polesinit[i].Y() - (1 / TolPoles) * Polemin));
                        }
                    }
                    else
                    {
                        if ((TolPoles * Polemax < Polesinit[0].Y()) && (TolPoles * Polemax < Polesinit[3].Y()))
                        {
                            for (i = 0; i <= 3; i++)
                            {
                                Polesinit[i].SetCoord(0.0, (Polesinit[i].Y() - TolPoles * Polemax));
                            }
                            mark = 1;
                        }
                        if ((1 / TolPoles * Polemin > Polesinit[0].Y())
                            && (1 / TolPoles * Polemin > Polesinit[3].Y()) && (mark == 0))
                        {
                            for (i = 0; i <= 3; i++)
                            {
                                Polesinit[i].SetCoord(0.0, (Polesinit[i].Y() - 1 / TolPoles * Polemin));
                            }
                            mark = 1;
                        }
                        if (mark == 0)
                        {
                            double Pole0, Pole3;
                            Pole0 = Polesinit[0].Y();
                            Pole3 = Polesinit[3].Y();
                            if (Pole0 < Pole3)
                            {
                                a = Math.Log10(Pole3 / Pole0);
                                if (boucle == 2)
                                {
                                    for (i = 0; i <= 3; i++)
                                    {
                                        Polesinit[i].SetCoord(
                                            0.0,
                                            Polesinit[i].Y()
                                                - (Pole3 * (Math.Pow(10.0, (-0.5 * Math.Log10(TolPoles) - a / 2.0)))));
                                    }
                                }
                                else if (boucle == 1)
                                {
                                    for (i = 0; i <= 3; i++)
                                    {
                                        Polesinit[i].SetCoord(
                                            0.0,
                                            Polesinit[i].Y()
                                                - (Pole0 * (Math.Pow(10.0, (a / 2.0 + 0.5 * Math.Log10(TolPoles))))));
                                    }
                                    dercas = 1;
                                }
                            }
                            if (Pole0 > Pole3)
                            {
                                a = Math.Log10(Pole0 / Pole3);
                                if (boucle == 2)
                                {
                                    for (i = 0; i <= 3; i++)
                                    {
                                        Polesinit[i].SetCoord(
                                            0.0,
                                            Polesinit[i].Y()
                                                - (Pole0 * (Math.Pow(10.0, (-0.5 * Math.Log10(TolPoles) - a / 2.0)))));
                                    }
                                }
                                else if (boucle == 1)
                                {
                                    for (i = 0; i <= 3; i++)
                                    {
                                        Polesinit[i].SetCoord(
                                            0.0,
                                            Polesinit[i].Y()
                                                - (Pole3 * (Math.Pow(10.0, (a / 2.0 + 0.5 * Math.Log10(TolPoles))))));
                                    }
                                    dercas = 1;
                                }
                            }
                        }
                    }
                }
            }
        } // end of the loop

        if (!SignDenom(Polesinit)) // inversion of the polynome sign
        {
            for (index = 0; index <= 3; index++)
            {
                Polesinit[index].SetCoord(0.0, -Polesinit[index].Y());
            }
        }

        // loop of positivity
        if ((Polesinit[1].Y() < 0.0) && (Polesinit[2].Y() >= 0.0))
        {
            Us1 = Polesinit[0].Y() / (Polesinit[0].Y() - Polesinit[1].Y());
            if (boucle == 2)
            {
                Us1 = Us1 * knots[2];
            }
            if (boucle == 1)
            {
                if (Ux != 0.0)
                {
                    Us1 = Us1 * Ux;
                }
            }
            BSplCLib.LocateParameter(3, knots, Us1, false, 1, knots.Length(), out I1, out Us1);
            if (I1 < 2)
            {
                U4 = Us1;
            }
            else
            {
                U4 = knots[I1];
            }
        }

        if ((Polesinit[1].Y() >= 0.0) && (Polesinit[2].Y() < 0.0))
        {
            Us2 = Polesinit[2].Y() / (Polesinit[2].Y() - Polesinit[3].Y());
            if (boucle == 2)
            {
                Us2 = knots[knots.Length() - 1] + Us2 * (1 - knots[knots.Length() - 1]);
            }
            if (boucle == 1)
            {
                if (Ux != 0.0)
                {
                    Us2 = Uy + Us2 * (1 - Uy);
                }
            }
            BSplCLib.LocateParameter(3, knots, Us2, false, 1, knots.Length(), out I1, out Us2);
            if (I1 >= (knots.Length() - 1))
            {
                U5 = Us2;
            }
            else
            {
                U5 = knots[I1 + 1];
            }
        }

        if (dercas == 1)
        {
            boucle++;
        }

        if ((Polesinit[1].Y() < 0.0) && (Polesinit[2].Y() < 0.0))
        {
            Us1 = Polesinit[0].Y() / (Polesinit[0].Y() - Polesinit[1].Y());
            Us2 = Polesinit[2].Y() / (Polesinit[2].Y() - Polesinit[3].Y());
            if (boucle != 0)
            {
                if (Ux != 0.0)
                {
                    Us1 = Us1 * Ux;
                    Us2 = Uy + Us2 * (1 - Uy);
                }
            }
            if (Us2 <= Us1)
            {
                BSplCLib.LocateParameter(3, knots, Us1, false, 1, knots.Length(), out I1, out Us1);
                if (knots[I1] >= Us2)
                { // insertion of one knot for the two poles
                    U4 = knots[I1];
                }
                else
                {
                    if (I1 >= 2)
                    {                   // insertion to the left and
                        U4 = knots[I1]; // to the right without a new knot
                        BSplCLib.LocateParameter(3, knots, Us2, false, 1, knots.Length(), out I3, out Us2);
                        if (I3 < (BS.NbKnots() - 1))
                        {
                            U5 = knots[I3 + 1];
                            cas = 1;
                        }
                    }
                    if (cas == 0)
                    { // insertion of only one new knot
                        U4 = (Us1 + Us2) / 2;
                    }
                }
            }
            else
            { // insertion of two knots
                BSplCLib.LocateParameter(3, knots, Us1, false, 1, knots.Length(), out I1, out Us1);
                if (I1 >= 2)
                {
                    U4 = knots[I1];
                }
                else
                {
                    U4 = Us1;
                }
                BSplCLib.LocateParameter(3, knots, Us2, false, 1, knots.Length(), out I3, out Us2);
                if (I3 < (BS.NbKnots() - 1))
                {
                    U5 = knots[I3 + 1];
                }
                else
                {
                    U5 = Us2;
                }
            }
        }
    }

    /// <summary>Inserts the knots in BS's knot sequence if they are not null.</summary>
    private static void InsertKnots(Geom2d_BSplineCurve BS, double U4, double U5)
    {
        if (U4 != 0.0)
        {                      // insertion of :0 knot if U4=0
            BS.InsertKnot(U4); //              1 knot if U4=U5
        }
        if ((U5 != 1.0) && (U5 != U4))
        { //              2 knots otherwise
            BS.InsertKnot(U5);
        }
    }

    /// <summary>Moves the poles above the x axis.</summary>
    private static void MovePoles(Geom2d_BSplineCurve BS)
    {
        gp_Pnt2d P = new();
        int i;

        for (i = 3; i <= (BS.NbPoles() - 2); i++)
        {
            P.SetCoord(1, (BS.Pole(i).Coord(1))); // raising of the no constrained poles to
            P.SetCoord(2, (BS.Pole(1).Coord(2))); // the first pole level
            BS.SetPole(i, P);
        }
    }

    /// <summary>Returns the correct spline a(u) which will be multiplied with BS later.</summary>
    /// <exception cref="Standard_DimensionError">The tolerances can't be met.</exception>
    public static Geom2d_BSplineCurve Solution(Geom_BSplineCurve BS, double TolPoles = 0.000001, double TolKnots = 0.000001)
    {
        NCollection_Array1<double> Herm = new(0, 3);
        double Upos1 = 0.0, Upos2 = 1.0,                  // positivity knots
            Ux = 0.0, Uy = 1.0, Utol1 = 0.0, Utol2 = 1.0, // tolerance knots
            Uint1 = 0.0, Uint2 = 1.0;                     // tolerance knots for the first loop
        int boucle = 1;                                   // loop mark
        NCollection_Array1<double> Knots = new(1, 2);
        NCollection_Array1<int> Multiplicities = new(1, 2);
        NCollection_Array1<gp_Pnt2d> Poles = new(1, 4);
        int zeroboucle = 0;
        HermiteCoeff(BS, Herm); // computation of the Hermite coefficient

        Poles[1].SetCoord(0.0, Herm[0]); // poles of the Hermite polynome in the BSpline form
        Poles[2].SetCoord(0.0, Herm[0] + Herm[1] / 3.0);
        Poles[3].SetCoord(0.0, Herm[3] - Herm[2] / 3.0);
        Poles[4].SetCoord(0.0, Herm[3]);
        Knots[1] = 0.0;
        Knots[2] = 1.0;
        Multiplicities[1] = 4;
        Multiplicities[2] = 4;

        Geom2d_BSplineCurve BS1 = new Geom2d_BSplineCurve(Poles, Knots, Multiplicities, 3); // creation of the basic
        Geom2d_BSplineCurve BS2 = new Geom2d_BSplineCurve(Poles, Knots, Multiplicities, 3); // BSpline without modif

        // computation of the positivity knots
        PolyTest(Herm, BS, out Upos1, out Upos2, ref zeroboucle, Precision.Confusion(), Precision.Confusion(), 1.0, 0.0);
        InsertKnots(BS2, Upos1, Upos2); // and insertion

        if (Upos1 != 0.0)
        {
            if (Upos2 != 1.0)
            {
                Ux = Min(Upos1, Upos2);
                Uy = Max(Upos1, Upos2);
            }
            else
            {
                Ux = Upos1;
                Uy = Upos1;
            }
        }
        else
        {
            Ux = Upos2;
            Uy = Upos2;
        }

        Herm[0] = BS2.Pole(1).Y();                         // computation of the Hermite coefficient on the
        Herm[1] = 3 * (BS2.Pole(2).Y() - BS2.Pole(1).Y()); // positive BSpline
        Herm[2] = 3 * (BS2.Pole(BS2.NbPoles()).Y() - BS2.Pole(BS2.NbPoles() - 1).Y());
        Herm[3] = BS2.Pole(BS2.NbPoles()).Y();

        PolyTest(Herm, BS, out Utol1, out Utol2, ref boucle, TolPoles, TolKnots, Ux, Uy); // computation of the tolerance knots
        InsertKnots(BS2, Utol1, Utol2); // and insertion

        if (boucle == 2)
        { // insertion of two knots
            Herm[0] = BS2.Pole(1).Y();
            Herm[1] = 3 * (BS2.Pole(2).Y() - BS2.Pole(1).Y());
            Herm[2] = 3 * (BS2.Pole(BS2.NbPoles()).Y() - BS2.Pole(BS2.NbPoles() - 1).Y());
            Herm[3] = BS2.Pole(BS2.NbPoles()).Y();
            if (Utol1 == 0.0)
            {
                Uint2 = Utol2;
                PolyTest(Herm, BS, out Utol1, out Utol2, ref boucle, TolPoles, TolKnots, Uint2, 0.0);
            }
            else
            {
                Uint1 = Utol1;
                PolyTest(Herm, BS, out Utol1, out Utol2, ref boucle, TolPoles, TolKnots, Uint1, 0.0);
            }
            InsertKnots(BS2, Utol1, Utol2);
        }
        if ((BS2.Knot(2) < TolKnots) || (BS2.Knot(BS2.NbKnots() - 1) > (1 - TolKnots)))
        { // checking of the knots tolerance
            throw new Standard_DimensionError("Hermit Impossible Tolerance");
        }
        else
        {
            if ((Upos2 == 1.0) && (Utol2 == 1.0) && (Uint2 == 1.0))
            { // test on the final inserted knots
                InsertKnots(BS1, BS2.Knot(2), 1.0);
            }
            else
            {
                if ((Upos1 == 0.0) && (Utol1 == 0.0) && (Uint1 == 0.0))
                {
                    InsertKnots(BS1, BS2.Knot(BS2.NbKnots() - 1), 1.0);
                }
                else
                {
                    InsertKnots(BS1, BS2.Knot(BS2.NbKnots() - 1), BS2.Knot(2));
                }
            }
            MovePoles(BS1); // relocation of the no-contrained knots
        }
        return BS1;
    }

    /// <summary>Returns the correct spline a(u) which will be multiplied with BS later.</summary>
    /// <exception cref="Standard_DimensionError">The tolerances can't be met.</exception>
    public static Geom2d_BSplineCurve Solution(Geom2d_BSplineCurve BS, double TolPoles = 0.000001, double TolKnots = 0.000001)
    {
        NCollection_Array1<double> Herm = new(0, 3);
        double Upos1 = 0.0, Upos2 = 1.0,                  // positivity knots
            Ux = 0.0, Uy = 1.0, Utol1 = 0.0, Utol2 = 1.0, // tolerance knots
            Uint1 = 0.0, Uint2 = 1.0;                     // tolerance knots for the first loop
        int boucle = 1;                                   // loop mark
        NCollection_Array1<double> Knots = new(1, 2);
        NCollection_Array1<int> Multiplicities = new(1, 2);
        NCollection_Array1<gp_Pnt2d> Poles = new(1, 4);
        int zeroboucle = 0;
        HermiteCoeff(BS, Herm); // computation of the Hermite coefficient

        Poles[1].SetCoord(0.0, Herm[0]); // poles of the Hermite polynome in the BSpline form
        Poles[2].SetCoord(0.0, Herm[0] + Herm[1] / 3.0);
        Poles[3].SetCoord(0.0, Herm[3] - Herm[2] / 3.0);
        Poles[4].SetCoord(0.0, Herm[3]);
        Knots[1] = 0.0;
        Knots[2] = 1.0;
        Multiplicities[1] = 4;
        Multiplicities[2] = 4;

        Geom2d_BSplineCurve BS1 = new Geom2d_BSplineCurve(Poles, Knots, Multiplicities, 3); // creation of the basic
        Geom2d_BSplineCurve BS2 = new Geom2d_BSplineCurve(Poles, Knots, Multiplicities, 3); // BSpline without modif

        // computation of the positivity knots
        PolyTest(Herm, BS, out Upos1, out Upos2, ref zeroboucle, Precision.Confusion(), Precision.Confusion(), 1.0, 0.0);
        InsertKnots(BS2, Upos1, Upos2); // and insertion

        if (Upos1 != 0.0)
        {
            if (Upos2 != 1.0)
            {
                Ux = Min(Upos1, Upos2);
                Uy = Max(Upos1, Upos2);
            }
            else
            {
                Ux = Upos1;
                Uy = Upos1;
            }
        }
        else
        {
            Ux = Upos2;
            Uy = Upos2;
        }

        Herm[0] = BS2.Pole(1).Y();                         // computation of the Hermite coefficient on the
        Herm[1] = 3 * (BS2.Pole(2).Y() - BS2.Pole(1).Y()); // positive BSpline
        Herm[2] = 3 * (BS2.Pole(BS2.NbPoles()).Y() - BS2.Pole(BS2.NbPoles() - 1).Y());
        Herm[3] = BS2.Pole(BS2.NbPoles()).Y();

        PolyTest(Herm, BS, out Utol1, out Utol2, ref boucle, TolPoles, TolKnots, Ux, Uy); // computation of the tolerance knots
        InsertKnots(BS2, Utol1, Utol2); // and insertion

        if (boucle == 2)
        { // insertion of two knots
            Herm[0] = BS2.Pole(1).Y();
            Herm[1] = 3 * (BS2.Pole(2).Y() - BS2.Pole(1).Y());
            Herm[2] = 3 * (BS2.Pole(BS2.NbPoles()).Y() - BS2.Pole(BS2.NbPoles() - 1).Y());
            Herm[3] = BS2.Pole(BS2.NbPoles()).Y();
            if (Utol1 == 0.0)
            {
                Uint2 = Utol2;
                PolyTest(Herm, BS, out Utol1, out Utol2, ref boucle, TolPoles, TolKnots, Uint2, 0.0);
            }
            else
            {
                Uint1 = Utol1;
                PolyTest(Herm, BS, out Utol1, out Utol2, ref boucle, TolPoles, TolKnots, Uint1, 0.0);
            }
            InsertKnots(BS2, Utol1, Utol2);
        }
        if ((BS2.Knot(2) < TolKnots) || (BS2.Knot(BS2.NbKnots() - 1) > (1 - TolKnots)))
        { // checking of the knots tolerance
            throw new Standard_DimensionError("Hermit Impossible Tolerance");
        }
        else
        {
            if ((Upos2 == 1.0) && (Utol2 == 1.0) && (Uint2 == 1.0))
            { // test on the final inserted knots
                InsertKnots(BS1, BS2.Knot(2), 1.0);
            }
            else
            {
                if ((Upos1 == 0.0) && (Utol1 == 0.0) && (Uint1 == 0.0))
                {
                    InsertKnots(BS1, BS2.Knot(BS2.NbKnots() - 1), 1.0);
                }
                else
                {
                    InsertKnots(BS1, BS2.Knot(BS2.NbKnots() - 1), BS2.Knot(2));
                }
            }
            MovePoles(BS1); // relocation of the no-contrained knots
        }
        return BS1;
    }

    /// <summary>Returns the knots to insert to a(u) to stay with a constant sign and in the tolerances.</summary>
    /// <remarks>
    /// <paramref name="Knotmin"/> and <paramref name="Knotmax"/> are <c>ref</c>: OCCT writes only one of them in
    /// some cases and leaves the other as the caller set it.
    /// </remarks>
    /// <exception cref="Standard_DimensionError">The tolerances can't be met.</exception>
    public static void Solutionbis(Geom_BSplineCurve BS,
                                   ref double Knotmin,
                                   ref double Knotmax,
                                   double TolPoles = 0.000001,
                                   double TolKnots = 0.000001)
    {
        NCollection_Array1<double> Herm = new(0, 3);
        double Upos1 = 0.0, Upos2 = 1.0,                  // positivity knots
            Ux = 0.0, Uy = 1.0, Utol1 = 0.0, Utol2 = 1.0, // tolerance knots
            Uint1 = 0.0, Uint2 = 1.0;                     // tolerance knots for the first loop
        int boucle = 1;                                   // loop mark
        NCollection_Array1<double> Knots = new(1, 2);
        NCollection_Array1<int> Multiplicities = new(1, 2);
        NCollection_Array1<gp_Pnt2d> Poles = new(1, 4);
        int zeroboucle = 0;
        HermiteCoeff(BS, Herm); // computation of the Hermite coefficient

        Poles[1].SetCoord(0.0, Herm[0]); // poles of the Hermite polynome in the BSpline form
        Poles[2].SetCoord(0.0, Herm[0] + Herm[1] / 3.0);
        Poles[3].SetCoord(0.0, Herm[3] - Herm[2] / 3.0);
        Poles[4].SetCoord(0.0, Herm[3]);
        Knots[1] = 0.0;
        Knots[2] = 1.0;
        Multiplicities[1] = 4;
        Multiplicities[2] = 4;

        Geom2d_BSplineCurve BS2 = new Geom2d_BSplineCurve(Poles, Knots, Multiplicities, 3); // creation of the basic BSpline without modif

        // computation of the positivity knots
        PolyTest(Herm, BS, out Upos1, out Upos2, ref zeroboucle, Precision.Confusion(), Precision.Confusion(), 1.0, 0.0);
        InsertKnots(BS2, Upos1, Upos2); // and insertion

        if (Upos1 != 0.0)
        {
            if (Upos2 != 1.0)
            {
                Ux = Min(Upos1, Upos2);
                Uy = Max(Upos1, Upos2);
            }
            else
            {
                Ux = Upos1;
                Uy = Upos1;
            }
        }
        else
        {
            Ux = Upos2;
            Uy = Upos2;
        }

        Herm[0] = BS2.Pole(1).Y();                         // computation of the Hermite coefficient on the
        Herm[1] = 3 * (BS2.Pole(2).Y() - BS2.Pole(1).Y()); // positive BSpline
        Herm[2] = 3 * (BS2.Pole(BS2.NbPoles()).Y() - BS2.Pole(BS2.NbPoles() - 1).Y());
        Herm[3] = BS2.Pole(BS2.NbPoles()).Y();

        PolyTest(Herm, BS, out Utol1, out Utol2, ref boucle, TolPoles, TolKnots, Ux, Uy); // computation of the tolerance knots
        InsertKnots(BS2, Utol1, Utol2); // and insertion

        if (boucle == 2)
        { // insertion of two knots
            Herm[0] = BS2.Pole(1).Y();
            Herm[1] = 3 * (BS2.Pole(2).Y() - BS2.Pole(1).Y());
            Herm[2] = 3 * (BS2.Pole(BS2.NbPoles()).Y() - BS2.Pole(BS2.NbPoles() - 1).Y());
            Herm[3] = BS2.Pole(BS2.NbPoles()).Y();
            if (Utol1 == 0.0)
            {
                Uint2 = Utol2;
                PolyTest(Herm, BS, out Utol1, out Utol2, ref boucle, TolPoles, TolKnots, Uint2, 0.0);
            }
            else
            {
                Uint1 = Utol1;
                PolyTest(Herm, BS, out Utol1, out Utol2, ref boucle, TolPoles, TolKnots, Uint1, 0.0);
            }
            InsertKnots(BS2, Utol1, Utol2);
        }
        if ((BS2.Knot(2) < TolKnots) || (BS2.Knot(BS2.NbKnots() - 1) > (1 - TolKnots)))
        { // checking of the knots tolerance
            throw new Standard_DimensionError("Hermit Impossible Tolerance");
        }
        else
        {
            if ((Upos2 == 1.0) && (Utol2 == 1.0) && (Uint2 == 1.0))
            { // test on the final inserted knots
                Knotmin = BS2.Knot(2);
            }
            else
            {
                if ((Upos1 == 0.0) && (Utol1 == 0.0) && (Uint1 == 0.0))
                {
                    Knotmax = BS2.Knot(BS2.NbKnots() - 1);
                }
                else
                {
                    Knotmin = BS2.Knot(2);
                    Knotmax = BS2.Knot(BS2.NbKnots() - 1);
                }
            }
        }
    }
}
