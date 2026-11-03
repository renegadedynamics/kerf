// Port of: src/ModelingData/TKGeomBase/Hermit/Hermit.hxx
// Port of: src/ModelingData/TKGeomBase/Hermit/Hermit.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Approximation;

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
    private static void HermiteCoeff(BSplineCurve3D bs, NCollection_Array1<double> tab)
    {
        NCollection_Array1<double> knots = new(bs.InternalKnots());
        NCollection_Array1<double> weights = bs.InternalWeightsArray();
        NCollection_Array1<int> mults = bs.InternalMultiplicities();
        int degree, index0, index1;   // denominateur value for u=0 & u=1
        double denom0, denom1,        // denominator value for u=0 & u=1
            deriv0, deriv1;           // derivative denominator value for u=0 & 1
        bool periodic;

        global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(0.0, 1.0, knots); // affinity on the nodal vector
        degree = bs.Degree;
        periodic = bs.IsPeriodic;
        index0 = bs.FirstUKnotIndex();
        index1 = bs.LastUKnotIndex() - 1;

        // Evaluate the weight function w(u) = sum_i N_i,p(u) * w_i and its derivative
        // at u=0 and u=1. The weight function of a rational BSpline is itself a polynomial
        // BSpline of the same degree, with weight values as scalar "poles" and no rational
        // denominator -- hence Weights is passed as Poles, with NoWeights() for unweighted evaluation.
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D1(0.0, index0, degree, periodic, weights, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), knots, mults, out denom0, out deriv0);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D1(1.0, index1, degree, periodic, weights, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), knots, mults, out denom1, out deriv1);
        tab[0] = 1 / denom0; // Hermit coefficients
        tab[1] = -deriv0 / (denom0 * denom0);
        tab[2] = -deriv1 / (denom1 * denom1);
        tab[3] = 1 / denom1;
    }

    /// <summary>Calculates the Hermite coefficients of degree 3 from BS and stores them in TAB (4 coefficients).</summary>
    private static void HermiteCoeff(BSplineCurve2D bs, NCollection_Array1<double> tab)
    {
        NCollection_Array1<double> knots = new(bs.InternalKnots());
        NCollection_Array1<double> weights = bs.InternalWeightsArray();
        NCollection_Array1<int> mults = bs.InternalMultiplicities();
        int degree, index0, index1;   // denominateur value for u=0 & u=1
        double denom0, denom1,        // denominator value for u=0 & u=1
            deriv0, deriv1;           // derivative denominator value for u=0 & 1
        bool periodic;

        global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(0.0, 1.0, knots); // affinity on the nodal vector
        degree = bs.Degree;
        periodic = bs.IsPeriodic;
        index0 = bs.FirstUKnotIndex();
        index1 = bs.LastUKnotIndex() - 1;

        // Evaluate the weight function w(u) = sum_i N_i,p(u) * w_i and its derivative
        // at u=0 and u=1. The weight function of a rational BSpline is itself a polynomial
        // BSpline of the same degree, with weight values as scalar "poles" and no rational
        // denominator -- hence Weights is passed as Poles, with NoWeights() for unweighted evaluation.
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D1(0.0, index0, degree, periodic, weights, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), knots, mults, out denom0, out deriv0);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D1(1.0, index1, degree, periodic, weights, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), knots, mults, out denom1, out deriv1);
        tab[0] = 1 / denom0; // Hermit coefficients
        tab[1] = -deriv0 / (denom0 * denom0);
        tab[2] = -deriv1 / (denom1 * denom1);
        tab[3] = 1 / denom1;
    }

    /// <summary>Gives the sign of Herm(0): true for positive.</summary>
    private static bool SignDenom(NCollection_Array1<Point2D> poles)
    {
        bool result;

        result = poles[0].Y >= 0;
        return result;
    }

    /// <summary>Gives the max and the min of the Poles (by their index).</summary>
    private static void Polemax(NCollection_Array1<Point2D> poles, out int min, out int max)
    {
        int i;
        double Max, Min; // intermediate value of max and min ordinates
        min = 0;         // initialisation of the indices
        max = 0;

        Min = poles[0].Y; // initialisation of the intermediate value
        Max = poles[0].Y;
        for (i = 1; i <= (poles.Length() - 1); i++)
        {
            if (poles[i].Y < Min)
            {
                Min = poles[i].Y;
                min = i;
            }
            if (poles[i].Y > Max)
            {
                Max = poles[i].Y;
                max = i;
            }
        }
    }

    /// <summary>Gives the knots U4 and U5 to insert to a(u).</summary>
    private static void PolyTest(NCollection_Array1<double> herm,
                                 BSplineCurve3D bs,
                                 out double u4,
                                 out double u5,
                                 ref int boucle,
                                 double tolPoles,
                                 double tolKnots,
                                 double ux,
                                 double uy)
    {
        _ = tolKnots; // unused, as in OCCT
        int index, i, i1 = 0, i2 = 0, i3 = 0, i4 = 0; // knots index
        NCollection_Array1<Point2D> polesinit = new(0, 3);
        NCollection_HArray1<double> Knots; // array of the BSpline knots + the ones inserted
        int cas = 0, mark = 0, dercas = 0, // loop marks
            min, max;                      // Pole min and max indices
        double us1, us2, a;                // boundaries value of the knots to be inserted

        u4 = 0.0;
        u5 = 1.0; // default value
        if (ux != 1.0)
        {
            bs.LocateU(ux, 0.0, out i1, out i2); // localization of the inserted knots
            if (uy != 0.0)
            {
                bs.LocateU(uy, 0.0, out i3, out i4);
            }
        }

        if (i1 == i2)
        { // definition and filling of the
            if ((i3 == i4) || (i3 == 0))
            { // array of knots
                Knots = new NCollection_HArray1<double>(1, bs.KnotCount);
                for (i = 1; i <= bs.KnotCount; i++)
                {
                    Knots.SetValue(i, bs.Knot(i));
                }
            }
            else
            {
                Knots = new NCollection_HArray1<double>(1, bs.KnotCount + 1);
                for (i = 1; i <= bs.KnotCount; i++)
                {
                    Knots.SetValue(i, bs.Knot(i));
                }
                Knots.SetValue(bs.KnotCount + 1, uy);
            }
        }
        else
        {
            if ((i3 == i4) || (i3 == 0))
            {
                Knots = new NCollection_HArray1<double>(1, bs.KnotCount + 1);
                for (i = 1; i <= bs.KnotCount; i++)
                {
                    Knots.SetValue(i, bs.Knot(i));
                }
                Knots.SetValue(bs.KnotCount + 1, ux);
            }
            else
            {
                Knots = new NCollection_HArray1<double>(1, bs.KnotCount + 2);
                for (i = 1; i <= bs.KnotCount; i++)
                {
                    Knots.SetValue(i, bs.Knot(i));
                }
                Knots.SetValue(bs.KnotCount + 1, ux);
                Knots.SetValue(bs.KnotCount + 2, uy);
            }
        }

        NCollection_Array1<double> knots = new(1, Knots.Length());
        knots.Assign(Knots.ChangeArray1());

        // sort of the array of knots
        knots.AsSpan().Sort();

        polesinit[0] = polesinit[0].WithCoord(0.0, herm[0]); // poles of the Hermite polynome in the BSpline form
        polesinit[1] = polesinit[1].WithCoord(0.0, herm[0] + herm[1] / 3.0);
        polesinit[2] = polesinit[2].WithCoord(0.0, herm[3] - herm[2] / 3.0);
        polesinit[3] = polesinit[3].WithCoord(0.0, herm[3]);

        // loop to check the tolerances on poles
        if (tolPoles != 0.0)
        {
            global::Kerf.Engine.Approximation.Hermit.Polemax(polesinit, out min, out max);
            double polemin = polesinit[min].Y;
            double polemax = polesinit[max].Y;
            if (((polemax) >= ((1 / tolPoles) * polemin))
                || ((polemin == 0.0) && (polemax >= (1 / tolPoles))))
            {
                if (polesinit[0].Y >= (1 / tolPoles) * polesinit[3].Y
                    || polesinit[0].Y <= tolPoles * polesinit[3].Y)
                {
                    throw new DimensionErrorException("Hermit Impossible Tolerance");
                }
                if ((max == 0) || (max == 3))
                {
                    for (i = 0; i <= 3; i++)
                    {
                        polesinit[i] = polesinit[i].WithCoord(0.0, (polesinit[i].Y - tolPoles * polemax));
                    }
                }
                else if ((max == 1) || (max == 2))
                {
                    if ((min == 0) || (min == 3))
                    {
                        for (i = 0; i <= 3; i++)
                        {
                            polesinit[i] = polesinit[i].WithCoord(0.0, (polesinit[i].Y - (1 / tolPoles) * polemin));
                        }
                    }
                    else
                    {
                        if ((tolPoles * polemax < polesinit[0].Y) && (tolPoles * polemax < polesinit[3].Y))
                        {
                            for (i = 0; i <= 3; i++)
                            {
                                polesinit[i] = polesinit[i].WithCoord(0.0, (polesinit[i].Y - tolPoles * polemax));
                            }
                            mark = 1;
                        }
                        if ((1 / tolPoles * polemin > polesinit[0].Y)
                            && (1 / tolPoles * polemin > polesinit[3].Y) && (mark == 0))
                        {
                            for (i = 0; i <= 3; i++)
                            {
                                polesinit[i] = polesinit[i].WithCoord(0.0, (polesinit[i].Y - 1 / tolPoles * polemin));
                            }
                            mark = 1;
                        }
                        if (mark == 0)
                        {
                            double pole0, pole3;
                            pole0 = polesinit[0].Y;
                            pole3 = polesinit[3].Y;
                            if (pole0 < pole3)
                            {
                                a = Math.Log10(pole3 / pole0);
                                if (boucle == 2)
                                {
                                    for (i = 0; i <= 3; i++)
                                    {
                                        polesinit[i] = polesinit[i].WithCoord(
                                            0.0,
                                            polesinit[i].Y
                                                - (pole3 * (Math.Pow(10.0, (-0.5 * Math.Log10(tolPoles) - a / 2.0)))));
                                    }
                                }
                                else if (boucle == 1)
                                {
                                    for (i = 0; i <= 3; i++)
                                    {
                                        polesinit[i] = polesinit[i].WithCoord(
                                            0.0,
                                            polesinit[i].Y
                                                - (pole0 * (Math.Pow(10.0, (a / 2.0 + 0.5 * Math.Log10(tolPoles))))));
                                    }
                                    dercas = 1;
                                }
                            }
                            if (pole0 > pole3)
                            {
                                a = Math.Log10(pole0 / pole3);
                                if (boucle == 2)
                                {
                                    for (i = 0; i <= 3; i++)
                                    {
                                        polesinit[i] = polesinit[i].WithCoord(
                                            0.0,
                                            polesinit[i].Y
                                                - (pole0 * (Math.Pow(10.0, (-0.5 * Math.Log10(tolPoles) - a / 2.0)))));
                                    }
                                }
                                else if (boucle == 1)
                                {
                                    for (i = 0; i <= 3; i++)
                                    {
                                        polesinit[i] = polesinit[i].WithCoord(
                                            0.0,
                                            polesinit[i].Y
                                                - (pole3 * (Math.Pow(10.0, (a / 2.0 + 0.5 * Math.Log10(tolPoles))))));
                                    }
                                    dercas = 1;
                                }
                            }
                        }
                    }
                }
            }
        } // end of the loop

        if (!SignDenom(polesinit)) // inversion of the polynome sign
        {
            for (index = 0; index <= 3; index++)
            {
                polesinit[index] = polesinit[index].WithCoord(0.0, -polesinit[index].Y);
            }
        }

        // loop of positivity
        if ((polesinit[1].Y < 0.0) && (polesinit[2].Y >= 0.0))
        {
            us1 = polesinit[0].Y / (polesinit[0].Y - polesinit[1].Y);
            if (boucle == 2)
            {
                us1 = us1 * knots[2];
            }
            if (boucle == 1)
            {
                if (ux != 0.0)
                {
                    us1 = us1 * ux;
                }
            }
            global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(3, knots, us1, false, 1, knots.Length(), out i1, out us1);
            if (i1 < 2)
            {
                u4 = us1;
            }
            else
            {
                u4 = knots[i1];
            }
        }

        if ((polesinit[1].Y >= 0.0) && (polesinit[2].Y < 0.0))
        {
            us2 = polesinit[2].Y / (polesinit[2].Y - polesinit[3].Y);
            if (boucle == 2)
            {
                us2 = knots[knots.Length() - 1] + us2 * (1 - knots[knots.Length() - 1]);
            }
            if (boucle == 1)
            {
                if (ux != 0.0)
                {
                    us2 = uy + us2 * (1 - uy);
                }
            }
            global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(3, knots, us2, false, 1, knots.Length(), out i1, out us2);
            if (i1 >= (knots.Length() - 1))
            {
                u5 = us2;
            }
            else
            {
                u5 = knots[i1 + 1];
            }
        }

        if (dercas == 1)
        {
            boucle++;
        }

        if ((polesinit[1].Y < 0.0) && (polesinit[2].Y < 0.0))
        {
            us1 = polesinit[0].Y / (polesinit[0].Y - polesinit[1].Y);
            us2 = polesinit[2].Y / (polesinit[2].Y - polesinit[3].Y);
            if (boucle != 0)
            {
                if (ux != 0.0)
                {
                    us1 = us1 * ux;
                    us2 = uy + us2 * (1 - uy);
                }
            }
            if (us2 <= us1)
            {
                global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(3, knots, us1, false, 1, knots.Length(), out i1, out us1);
                if (knots[i1] >= us2)
                { // insertion of one knot for the two poles
                    u4 = knots[i1];
                }
                else
                {
                    if (i1 >= 2)
                    {                   // insertion to the left and
                        u4 = knots[i1]; // to the right without a new knot
                        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(3, knots, us2, false, 1, knots.Length(), out i3, out us2);
                        if (i3 < (bs.KnotCount - 1))
                        {
                            u5 = knots[i3 + 1];
                            cas = 1;
                        }
                    }
                    if (cas == 0)
                    { // insertion of only one new knot
                        u4 = (us1 + us2) / 2;
                    }
                }
            }
            else
            { // insertion of two knots
                global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(3, knots, us1, false, 1, knots.Length(), out i1, out us1);
                if (i1 >= 2)
                {
                    u4 = knots[i1];
                }
                else
                {
                    u4 = us1;
                }
                global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(3, knots, us2, false, 1, knots.Length(), out i3, out us2);
                if (i3 < (bs.KnotCount - 1))
                {
                    u5 = knots[i3 + 1];
                }
                else
                {
                    u5 = us2;
                }
            }
        }
    }

    /// <summary>Gives the knots U4 and U5 to insert to a(u).</summary>
    private static void PolyTest(NCollection_Array1<double> herm,
                                 BSplineCurve2D bs,
                                 out double u4,
                                 out double u5,
                                 ref int boucle,
                                 double tolPoles,
                                 double tolKnots,
                                 double ux,
                                 double uy)
    {
        _ = tolKnots; // unused, as in OCCT
        int index, i, i1 = 0, i2 = 0, i3 = 0, i4 = 0; // knots index
        NCollection_Array1<Point2D> polesinit = new(0, 3);
        NCollection_HArray1<double> Knots; // array of the BSpline knots + the ones inserted
        int cas = 0, mark = 0, dercas = 0, // loop marks
            min, max;                      // Pole min and max indices
        double us1, us2, a;                // boundaries value of the knots to be inserted

        u4 = 0.0;
        u5 = 1.0; // default value
        if (ux != 1.0)
        {
            bs.LocateU(ux, 0.0, out i1, out i2); // localization of the inserted knots
            if (uy != 0.0)
            {
                bs.LocateU(uy, 0.0, out i3, out i4);
            }
        }

        if (i1 == i2)
        { // definition and filling of the
            if ((i3 == i4) || (i3 == 0))
            { // array of knots
                Knots = new NCollection_HArray1<double>(1, bs.KnotCount);
                for (i = 1; i <= bs.KnotCount; i++)
                {
                    Knots.SetValue(i, bs.Knot(i));
                }
            }
            else
            {
                Knots = new NCollection_HArray1<double>(1, bs.KnotCount + 1);
                for (i = 1; i <= bs.KnotCount; i++)
                {
                    Knots.SetValue(i, bs.Knot(i));
                }
                Knots.SetValue(bs.KnotCount + 1, uy);
            }
        }
        else
        {
            if ((i3 == i4) || (i3 == 0))
            {
                Knots = new NCollection_HArray1<double>(1, bs.KnotCount + 1);
                for (i = 1; i <= bs.KnotCount; i++)
                {
                    Knots.SetValue(i, bs.Knot(i));
                }
                Knots.SetValue(bs.KnotCount + 1, ux);
            }
            else
            {
                Knots = new NCollection_HArray1<double>(1, bs.KnotCount + 2);
                for (i = 1; i <= bs.KnotCount; i++)
                {
                    Knots.SetValue(i, bs.Knot(i));
                }
                Knots.SetValue(bs.KnotCount + 1, ux);
                Knots.SetValue(bs.KnotCount + 2, uy);
            }
        }

        NCollection_Array1<double> knots = new(1, Knots.Length());
        knots.Assign(Knots.ChangeArray1());

        // sort of the array of knots
        knots.AsSpan().Sort();

        polesinit[0] = polesinit[0].WithCoord(0.0, herm[0]); // poles of the Hermite polynome in the BSpline form
        polesinit[1] = polesinit[1].WithCoord(0.0, herm[0] + herm[1] / 3.0);
        polesinit[2] = polesinit[2].WithCoord(0.0, herm[3] - herm[2] / 3.0);
        polesinit[3] = polesinit[3].WithCoord(0.0, herm[3]);

        // loop to check the tolerances on poles
        if (tolPoles != 0.0)
        {
            global::Kerf.Engine.Approximation.Hermit.Polemax(polesinit, out min, out max);
            double polemin = polesinit[min].Y;
            double polemax = polesinit[max].Y;
            if (((polemax) >= ((1 / tolPoles) * polemin))
                || ((polemin == 0.0) && (polemax >= (1 / tolPoles))))
            {
                if (polesinit[0].Y >= (1 / tolPoles) * polesinit[3].Y
                    || polesinit[0].Y <= tolPoles * polesinit[3].Y)
                {
                    throw new DimensionErrorException("Hermit Impossible Tolerance");
                }
                if ((max == 0) || (max == 3))
                {
                    for (i = 0; i <= 3; i++)
                    {
                        polesinit[i] = polesinit[i].WithCoord(0.0, (polesinit[i].Y - tolPoles * polemax));
                    }
                }
                else if ((max == 1) || (max == 2))
                {
                    if ((min == 0) || (min == 3))
                    {
                        for (i = 0; i <= 3; i++)
                        {
                            polesinit[i] = polesinit[i].WithCoord(0.0, (polesinit[i].Y - (1 / tolPoles) * polemin));
                        }
                    }
                    else
                    {
                        if ((tolPoles * polemax < polesinit[0].Y) && (tolPoles * polemax < polesinit[3].Y))
                        {
                            for (i = 0; i <= 3; i++)
                            {
                                polesinit[i] = polesinit[i].WithCoord(0.0, (polesinit[i].Y - tolPoles * polemax));
                            }
                            mark = 1;
                        }
                        if ((1 / tolPoles * polemin > polesinit[0].Y)
                            && (1 / tolPoles * polemin > polesinit[3].Y) && (mark == 0))
                        {
                            for (i = 0; i <= 3; i++)
                            {
                                polesinit[i] = polesinit[i].WithCoord(0.0, (polesinit[i].Y - 1 / tolPoles * polemin));
                            }
                            mark = 1;
                        }
                        if (mark == 0)
                        {
                            double pole0, pole3;
                            pole0 = polesinit[0].Y;
                            pole3 = polesinit[3].Y;
                            if (pole0 < pole3)
                            {
                                a = Math.Log10(pole3 / pole0);
                                if (boucle == 2)
                                {
                                    for (i = 0; i <= 3; i++)
                                    {
                                        polesinit[i] = polesinit[i].WithCoord(
                                            0.0,
                                            polesinit[i].Y
                                                - (pole3 * (Math.Pow(10.0, (-0.5 * Math.Log10(tolPoles) - a / 2.0)))));
                                    }
                                }
                                else if (boucle == 1)
                                {
                                    for (i = 0; i <= 3; i++)
                                    {
                                        polesinit[i] = polesinit[i].WithCoord(
                                            0.0,
                                            polesinit[i].Y
                                                - (pole0 * (Math.Pow(10.0, (a / 2.0 + 0.5 * Math.Log10(tolPoles))))));
                                    }
                                    dercas = 1;
                                }
                            }
                            if (pole0 > pole3)
                            {
                                a = Math.Log10(pole0 / pole3);
                                if (boucle == 2)
                                {
                                    for (i = 0; i <= 3; i++)
                                    {
                                        polesinit[i] = polesinit[i].WithCoord(
                                            0.0,
                                            polesinit[i].Y
                                                - (pole0 * (Math.Pow(10.0, (-0.5 * Math.Log10(tolPoles) - a / 2.0)))));
                                    }
                                }
                                else if (boucle == 1)
                                {
                                    for (i = 0; i <= 3; i++)
                                    {
                                        polesinit[i] = polesinit[i].WithCoord(
                                            0.0,
                                            polesinit[i].Y
                                                - (pole3 * (Math.Pow(10.0, (a / 2.0 + 0.5 * Math.Log10(tolPoles))))));
                                    }
                                    dercas = 1;
                                }
                            }
                        }
                    }
                }
            }
        } // end of the loop

        if (!SignDenom(polesinit)) // inversion of the polynome sign
        {
            for (index = 0; index <= 3; index++)
            {
                polesinit[index] = polesinit[index].WithCoord(0.0, -polesinit[index].Y);
            }
        }

        // loop of positivity
        if ((polesinit[1].Y < 0.0) && (polesinit[2].Y >= 0.0))
        {
            us1 = polesinit[0].Y / (polesinit[0].Y - polesinit[1].Y);
            if (boucle == 2)
            {
                us1 = us1 * knots[2];
            }
            if (boucle == 1)
            {
                if (ux != 0.0)
                {
                    us1 = us1 * ux;
                }
            }
            global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(3, knots, us1, false, 1, knots.Length(), out i1, out us1);
            if (i1 < 2)
            {
                u4 = us1;
            }
            else
            {
                u4 = knots[i1];
            }
        }

        if ((polesinit[1].Y >= 0.0) && (polesinit[2].Y < 0.0))
        {
            us2 = polesinit[2].Y / (polesinit[2].Y - polesinit[3].Y);
            if (boucle == 2)
            {
                us2 = knots[knots.Length() - 1] + us2 * (1 - knots[knots.Length() - 1]);
            }
            if (boucle == 1)
            {
                if (ux != 0.0)
                {
                    us2 = uy + us2 * (1 - uy);
                }
            }
            global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(3, knots, us2, false, 1, knots.Length(), out i1, out us2);
            if (i1 >= (knots.Length() - 1))
            {
                u5 = us2;
            }
            else
            {
                u5 = knots[i1 + 1];
            }
        }

        if (dercas == 1)
        {
            boucle++;
        }

        if ((polesinit[1].Y < 0.0) && (polesinit[2].Y < 0.0))
        {
            us1 = polesinit[0].Y / (polesinit[0].Y - polesinit[1].Y);
            us2 = polesinit[2].Y / (polesinit[2].Y - polesinit[3].Y);
            if (boucle != 0)
            {
                if (ux != 0.0)
                {
                    us1 = us1 * ux;
                    us2 = uy + us2 * (1 - uy);
                }
            }
            if (us2 <= us1)
            {
                global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(3, knots, us1, false, 1, knots.Length(), out i1, out us1);
                if (knots[i1] >= us2)
                { // insertion of one knot for the two poles
                    u4 = knots[i1];
                }
                else
                {
                    if (i1 >= 2)
                    {                   // insertion to the left and
                        u4 = knots[i1]; // to the right without a new knot
                        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(3, knots, us2, false, 1, knots.Length(), out i3, out us2);
                        if (i3 < (bs.KnotCount - 1))
                        {
                            u5 = knots[i3 + 1];
                            cas = 1;
                        }
                    }
                    if (cas == 0)
                    { // insertion of only one new knot
                        u4 = (us1 + us2) / 2;
                    }
                }
            }
            else
            { // insertion of two knots
                global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(3, knots, us1, false, 1, knots.Length(), out i1, out us1);
                if (i1 >= 2)
                {
                    u4 = knots[i1];
                }
                else
                {
                    u4 = us1;
                }
                global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(3, knots, us2, false, 1, knots.Length(), out i3, out us2);
                if (i3 < (bs.KnotCount - 1))
                {
                    u5 = knots[i3 + 1];
                }
                else
                {
                    u5 = us2;
                }
            }
        }
    }

    /// <summary>Inserts the knots in BS's knot sequence if they are not null.</summary>
    private static void InsertKnots(BSplineCurve2D bs, double u4, double u5)
    {
        if (u4 != 0.0)
        {                      // insertion of :0 knot if U4=0
            bs.InsertKnot(u4); //              1 knot if U4=U5
        }
        if ((u5 != 1.0) && (u5 != u4))
        { //              2 knots otherwise
            bs.InsertKnot(u5);
        }
    }

    /// <summary>Moves the poles above the x axis.</summary>
    private static void MovePoles(BSplineCurve2D bs)
    {
        Point2D p = new();
        int i;

        for (i = 3; i <= (bs.PoleCount - 2); i++)
        {
            p = p.WithCoord(1, (bs.Pole(i).Coord(1))); // raising of the no constrained poles to
            p = p.WithCoord(2, (bs.Pole(1).Coord(2))); // the first pole level
            bs.SetPole(i, p);
        }
    }

    /// <summary>Returns the correct spline a(u) which will be multiplied with BS later.</summary>
    /// <exception cref="DimensionErrorException">The tolerances can't be met.</exception>
    public static BSplineCurve2D Solution(BSplineCurve3D bs, double tolPoles = 0.000001, double tolKnots = 0.000001)
    {
        NCollection_Array1<double> herm = new(0, 3);
        double upos1 = 0.0, upos2 = 1.0,                  // positivity knots
            ux = 0.0, uy = 1.0, utol1 = 0.0, utol2 = 1.0, // tolerance knots
            uint1 = 0.0, uint2 = 1.0;                     // tolerance knots for the first loop
        int boucle = 1;                                   // loop mark
        NCollection_Array1<double> knots = new(1, 2);
        NCollection_Array1<int> multiplicities = new(1, 2);
        NCollection_Array1<Point2D> poles = new(1, 4);
        int zeroboucle = 0;
        HermiteCoeff(bs, herm); // computation of the Hermite coefficient

        poles[1] = poles[1].WithCoord(0.0, herm[0]); // poles of the Hermite polynome in the BSpline form
        poles[2] = poles[2].WithCoord(0.0, herm[0] + herm[1] / 3.0);
        poles[3] = poles[3].WithCoord(0.0, herm[3] - herm[2] / 3.0);
        poles[4] = poles[4].WithCoord(0.0, herm[3]);
        knots[1] = 0.0;
        knots[2] = 1.0;
        multiplicities[1] = 4;
        multiplicities[2] = 4;

        BSplineCurve2D bs1 = new BSplineCurve2D(poles, knots, multiplicities, 3); // creation of the basic
        BSplineCurve2D bs2 = new BSplineCurve2D(poles, knots, multiplicities, 3); // BSpline without modif

        // computation of the positivity knots
        PolyTest(herm, bs, out upos1, out upos2, ref zeroboucle, global::Kerf.Engine.Foundation.Precision.Confusion(), global::Kerf.Engine.Foundation.Precision.Confusion(), 1.0, 0.0);
        InsertKnots(bs2, upos1, upos2); // and insertion

        if (upos1 != 0.0)
        {
            if (upos2 != 1.0)
            {
                ux = Min(upos1, upos2);
                uy = Max(upos1, upos2);
            }
            else
            {
                ux = upos1;
                uy = upos1;
            }
        }
        else
        {
            ux = upos2;
            uy = upos2;
        }

        herm[0] = bs2.Pole(1).Y;                         // computation of the Hermite coefficient on the
        herm[1] = 3 * (bs2.Pole(2).Y - bs2.Pole(1).Y); // positive BSpline
        herm[2] = 3 * (bs2.Pole(bs2.PoleCount).Y - bs2.Pole(bs2.PoleCount - 1).Y);
        herm[3] = bs2.Pole(bs2.PoleCount).Y;

        PolyTest(herm, bs, out utol1, out utol2, ref boucle, tolPoles, tolKnots, ux, uy); // computation of the tolerance knots
        InsertKnots(bs2, utol1, utol2); // and insertion

        if (boucle == 2)
        { // insertion of two knots
            herm[0] = bs2.Pole(1).Y;
            herm[1] = 3 * (bs2.Pole(2).Y - bs2.Pole(1).Y);
            herm[2] = 3 * (bs2.Pole(bs2.PoleCount).Y - bs2.Pole(bs2.PoleCount - 1).Y);
            herm[3] = bs2.Pole(bs2.PoleCount).Y;
            if (utol1 == 0.0)
            {
                uint2 = utol2;
                PolyTest(herm, bs, out utol1, out utol2, ref boucle, tolPoles, tolKnots, uint2, 0.0);
            }
            else
            {
                uint1 = utol1;
                PolyTest(herm, bs, out utol1, out utol2, ref boucle, tolPoles, tolKnots, uint1, 0.0);
            }
            InsertKnots(bs2, utol1, utol2);
        }
        if ((bs2.Knot(2) < tolKnots) || (bs2.Knot(bs2.KnotCount - 1) > (1 - tolKnots)))
        { // checking of the knots tolerance
            throw new DimensionErrorException("Hermit Impossible Tolerance");
        }
        else
        {
            if ((upos2 == 1.0) && (utol2 == 1.0) && (uint2 == 1.0))
            { // test on the final inserted knots
                InsertKnots(bs1, bs2.Knot(2), 1.0);
            }
            else
            {
                if ((upos1 == 0.0) && (utol1 == 0.0) && (uint1 == 0.0))
                {
                    InsertKnots(bs1, bs2.Knot(bs2.KnotCount - 1), 1.0);
                }
                else
                {
                    InsertKnots(bs1, bs2.Knot(bs2.KnotCount - 1), bs2.Knot(2));
                }
            }
            MovePoles(bs1); // relocation of the no-contrained knots
        }
        return bs1;
    }

    /// <summary>Returns the correct spline a(u) which will be multiplied with BS later.</summary>
    /// <exception cref="DimensionErrorException">The tolerances can't be met.</exception>
    public static BSplineCurve2D Solution(BSplineCurve2D bs, double tolPoles = 0.000001, double tolKnots = 0.000001)
    {
        NCollection_Array1<double> herm = new(0, 3);
        double upos1 = 0.0, upos2 = 1.0,                  // positivity knots
            ux = 0.0, uy = 1.0, utol1 = 0.0, utol2 = 1.0, // tolerance knots
            uint1 = 0.0, uint2 = 1.0;                     // tolerance knots for the first loop
        int boucle = 1;                                   // loop mark
        NCollection_Array1<double> knots = new(1, 2);
        NCollection_Array1<int> multiplicities = new(1, 2);
        NCollection_Array1<Point2D> poles = new(1, 4);
        int zeroboucle = 0;
        HermiteCoeff(bs, herm); // computation of the Hermite coefficient

        poles[1] = poles[1].WithCoord(0.0, herm[0]); // poles of the Hermite polynome in the BSpline form
        poles[2] = poles[2].WithCoord(0.0, herm[0] + herm[1] / 3.0);
        poles[3] = poles[3].WithCoord(0.0, herm[3] - herm[2] / 3.0);
        poles[4] = poles[4].WithCoord(0.0, herm[3]);
        knots[1] = 0.0;
        knots[2] = 1.0;
        multiplicities[1] = 4;
        multiplicities[2] = 4;

        BSplineCurve2D bs1 = new BSplineCurve2D(poles, knots, multiplicities, 3); // creation of the basic
        BSplineCurve2D bs2 = new BSplineCurve2D(poles, knots, multiplicities, 3); // BSpline without modif

        // computation of the positivity knots
        PolyTest(herm, bs, out upos1, out upos2, ref zeroboucle, global::Kerf.Engine.Foundation.Precision.Confusion(), global::Kerf.Engine.Foundation.Precision.Confusion(), 1.0, 0.0);
        InsertKnots(bs2, upos1, upos2); // and insertion

        if (upos1 != 0.0)
        {
            if (upos2 != 1.0)
            {
                ux = Min(upos1, upos2);
                uy = Max(upos1, upos2);
            }
            else
            {
                ux = upos1;
                uy = upos1;
            }
        }
        else
        {
            ux = upos2;
            uy = upos2;
        }

        herm[0] = bs2.Pole(1).Y;                         // computation of the Hermite coefficient on the
        herm[1] = 3 * (bs2.Pole(2).Y - bs2.Pole(1).Y); // positive BSpline
        herm[2] = 3 * (bs2.Pole(bs2.PoleCount).Y - bs2.Pole(bs2.PoleCount - 1).Y);
        herm[3] = bs2.Pole(bs2.PoleCount).Y;

        PolyTest(herm, bs, out utol1, out utol2, ref boucle, tolPoles, tolKnots, ux, uy); // computation of the tolerance knots
        InsertKnots(bs2, utol1, utol2); // and insertion

        if (boucle == 2)
        { // insertion of two knots
            herm[0] = bs2.Pole(1).Y;
            herm[1] = 3 * (bs2.Pole(2).Y - bs2.Pole(1).Y);
            herm[2] = 3 * (bs2.Pole(bs2.PoleCount).Y - bs2.Pole(bs2.PoleCount - 1).Y);
            herm[3] = bs2.Pole(bs2.PoleCount).Y;
            if (utol1 == 0.0)
            {
                uint2 = utol2;
                PolyTest(herm, bs, out utol1, out utol2, ref boucle, tolPoles, tolKnots, uint2, 0.0);
            }
            else
            {
                uint1 = utol1;
                PolyTest(herm, bs, out utol1, out utol2, ref boucle, tolPoles, tolKnots, uint1, 0.0);
            }
            InsertKnots(bs2, utol1, utol2);
        }
        if ((bs2.Knot(2) < tolKnots) || (bs2.Knot(bs2.KnotCount - 1) > (1 - tolKnots)))
        { // checking of the knots tolerance
            throw new DimensionErrorException("Hermit Impossible Tolerance");
        }
        else
        {
            if ((upos2 == 1.0) && (utol2 == 1.0) && (uint2 == 1.0))
            { // test on the final inserted knots
                InsertKnots(bs1, bs2.Knot(2), 1.0);
            }
            else
            {
                if ((upos1 == 0.0) && (utol1 == 0.0) && (uint1 == 0.0))
                {
                    InsertKnots(bs1, bs2.Knot(bs2.KnotCount - 1), 1.0);
                }
                else
                {
                    InsertKnots(bs1, bs2.Knot(bs2.KnotCount - 1), bs2.Knot(2));
                }
            }
            MovePoles(bs1); // relocation of the no-contrained knots
        }
        return bs1;
    }

    /// <summary>Returns the knots to insert to a(u) to stay with a constant sign and in the tolerances.</summary>
    /// <remarks>
    /// <paramref name="knotmin"/> and <paramref name="knotmax"/> are <c>ref</c>: OCCT writes only one of them in
    /// some cases and leaves the other as the caller set it.
    /// </remarks>
    /// <exception cref="DimensionErrorException">The tolerances can't be met.</exception>
    public static void Solutionbis(BSplineCurve3D bs,
                                   ref double knotmin,
                                   ref double knotmax,
                                   double tolPoles = 0.000001,
                                   double tolKnots = 0.000001)
    {
        NCollection_Array1<double> herm = new(0, 3);
        double upos1 = 0.0, upos2 = 1.0,                  // positivity knots
            ux = 0.0, uy = 1.0, utol1 = 0.0, utol2 = 1.0, // tolerance knots
            uint1 = 0.0, uint2 = 1.0;                     // tolerance knots for the first loop
        int boucle = 1;                                   // loop mark
        NCollection_Array1<double> knots = new(1, 2);
        NCollection_Array1<int> multiplicities = new(1, 2);
        NCollection_Array1<Point2D> poles = new(1, 4);
        int zeroboucle = 0;
        HermiteCoeff(bs, herm); // computation of the Hermite coefficient

        poles[1] = poles[1].WithCoord(0.0, herm[0]); // poles of the Hermite polynome in the BSpline form
        poles[2] = poles[2].WithCoord(0.0, herm[0] + herm[1] / 3.0);
        poles[3] = poles[3].WithCoord(0.0, herm[3] - herm[2] / 3.0);
        poles[4] = poles[4].WithCoord(0.0, herm[3]);
        knots[1] = 0.0;
        knots[2] = 1.0;
        multiplicities[1] = 4;
        multiplicities[2] = 4;

        BSplineCurve2D bs2 = new BSplineCurve2D(poles, knots, multiplicities, 3); // creation of the basic BSpline without modif

        // computation of the positivity knots
        PolyTest(herm, bs, out upos1, out upos2, ref zeroboucle, global::Kerf.Engine.Foundation.Precision.Confusion(), global::Kerf.Engine.Foundation.Precision.Confusion(), 1.0, 0.0);
        InsertKnots(bs2, upos1, upos2); // and insertion

        if (upos1 != 0.0)
        {
            if (upos2 != 1.0)
            {
                ux = Min(upos1, upos2);
                uy = Max(upos1, upos2);
            }
            else
            {
                ux = upos1;
                uy = upos1;
            }
        }
        else
        {
            ux = upos2;
            uy = upos2;
        }

        herm[0] = bs2.Pole(1).Y;                         // computation of the Hermite coefficient on the
        herm[1] = 3 * (bs2.Pole(2).Y - bs2.Pole(1).Y); // positive BSpline
        herm[2] = 3 * (bs2.Pole(bs2.PoleCount).Y - bs2.Pole(bs2.PoleCount - 1).Y);
        herm[3] = bs2.Pole(bs2.PoleCount).Y;

        PolyTest(herm, bs, out utol1, out utol2, ref boucle, tolPoles, tolKnots, ux, uy); // computation of the tolerance knots
        InsertKnots(bs2, utol1, utol2); // and insertion

        if (boucle == 2)
        { // insertion of two knots
            herm[0] = bs2.Pole(1).Y;
            herm[1] = 3 * (bs2.Pole(2).Y - bs2.Pole(1).Y);
            herm[2] = 3 * (bs2.Pole(bs2.PoleCount).Y - bs2.Pole(bs2.PoleCount - 1).Y);
            herm[3] = bs2.Pole(bs2.PoleCount).Y;
            if (utol1 == 0.0)
            {
                uint2 = utol2;
                PolyTest(herm, bs, out utol1, out utol2, ref boucle, tolPoles, tolKnots, uint2, 0.0);
            }
            else
            {
                uint1 = utol1;
                PolyTest(herm, bs, out utol1, out utol2, ref boucle, tolPoles, tolKnots, uint1, 0.0);
            }
            InsertKnots(bs2, utol1, utol2);
        }
        if ((bs2.Knot(2) < tolKnots) || (bs2.Knot(bs2.KnotCount - 1) > (1 - tolKnots)))
        { // checking of the knots tolerance
            throw new DimensionErrorException("Hermit Impossible Tolerance");
        }
        else
        {
            if ((upos2 == 1.0) && (utol2 == 1.0) && (uint2 == 1.0))
            { // test on the final inserted knots
                knotmin = bs2.Knot(2);
            }
            else
            {
                if ((upos1 == 0.0) && (utol1 == 0.0) && (uint1 == 0.0))
                {
                    knotmax = bs2.Knot(bs2.KnotCount - 1);
                }
                else
                {
                    knotmin = bs2.Knot(2);
                    knotmax = bs2.Knot(bs2.KnotCount - 1);
                }
            }
        }
    }
}
