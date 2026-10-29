// Port of: src/FoundationClasses/TKMath/Convert/Convert_PolynomialCosAndSin.hxx
// Port of: src/FoundationClasses/TKMath/Convert/Convert_PolynomialCosAndSin.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The free function of Convert_PolynomialCosAndSin.hxx (not exported by OCCT).</summary>
internal static class Convert_PolynomialCosAndSin
{
    private static double Locate(double Angfin, NCollection_Array1<gp_Pnt2d> TPoles, double Umin, double Umax)
    {
        double umin = Umin;
        double umax = Umax;
        double Ptol = Precision.Angular();
        double Utol = Precision.PConfusion();
        while (Math.Abs(umax - umin) >= Utol)
        {
            double ptest = (umax + umin) / 2.0;
            BSplCLib.D0(ptest, TPoles, BSplCLib.NoWeights(), out gp_Pnt2d valP);
            double theta = Math.Atan2(valP.Y(), valP.X());
            if (theta < 0.0)
            {
                theta += 2.0 * M_PI;
            }
            if (Math.Abs(theta - Angfin) < Ptol)
            {
                return ptest;
            }
            if (theta < Angfin)
            {
                umin = ptest;
            }
            else if (theta > Angfin)
            {
                umax = ptest;
            }
        }
        return (umin + umax) / 2.0;
    }

    /// <summary>
    /// The poles of a polynomial approximation of degree <paramref name="num_poles"/> - 1 of (cos, sin) on
    /// [<paramref name="UFirst"/>, <paramref name="ULast"/>] (Tiller's algorithm); the denominator is 1.
    /// </summary>
    internal static void BuildPolynomialCosAndSin(double UFirst,
                                                  double ULast,
                                                  int num_poles,
                                                  NCollection_Array1<double> CosNumerator,
                                                  NCollection_Array1<double> SinNumerator,
                                                  NCollection_Array1<double> Denominator)
    {
        double Delta, locUFirst,
          t_min, t_max, trim_min, trim_max, middle, Angle, PI2 = 2 * M_PI;
        int ii, degree = num_poles - 1;
        locUFirst = UFirst;

        // Return UFirst in [-2PI; 2PI]
        // to make rotations without risk
        while (locUFirst > PI2)
        {
            locUFirst -= PI2;
        }
        while (locUFirst < -PI2)
        {
            locUFirst += PI2;
        }

        // Return to the arc [0, Delta]
        Delta = ULast - UFirst;
        middle = 0.5e0 * Delta;

        // Coincide the required bisector of the angular sector with
        // axis -Ox. Definition of the circle in Bezier of degree 7 so that
        // parameter 1/2 of Bezier is exactly a point of the bisectrix
        // of the required angular sector.
        Angle = middle - M_PI;

        // Bezier control points for a unit circle (degree-7 polynomial approximation).
        // Coefficients from Tiller's algorithm for polynomial cos/sin representation.
        NCollection_Array1<gp_Pnt2d> TPoles = new(1, 8), NewTPoles = new(1, 8);
        TPoles[1].SetCoord(1.0, 0.0);
        TPoles[2].SetCoord(1.0, 1.013854);
        TPoles[3].SetCoord(-0.199043, 1.871905);
        TPoles[4].SetCoord(-1.937729, 1.057323);
        TPoles[5].SetCoord(-1.937729, -1.057323);
        TPoles[6].SetCoord(-0.199043, -1.871905);
        TPoles[7].SetCoord(1.0, -1.013854);
        TPoles[8].SetCoord(1.0, 0.0);
        gp_Trsf2d T = new();
        T.SetRotation(gp.Origin2d(), Angle);
        for (ii = 1; ii <= num_poles; ii++)
        {
            TPoles[ii].Transform(T);
        }

        t_min = 1.0e0 - (Delta * 1.3e0 / M_PI);
        t_min *= 0.5e0;
        t_min = Max(t_min, 0.0e0);
        t_max = 1.0e0 + (Delta * 1.3e0 / M_PI);
        t_max *= 0.5e0;
        t_max = Min(t_max, 1.0e0);
        trim_max = Locate(Delta, TPoles, t_min, t_max);
        //
        // as Bezier is symmetric correspondingly to the bissector
        // of the angular sector ...

        trim_min = 1.0e0 - trim_max;
        //
        // As in OCCT, which wraps the same two-element buffers in the old and the new knots and multiplicities, so
        // Trimming writes its result over its input; these views share storage in the same way.
        NCollection_Array1<double> knot_array = NCollection_Array1<double>.From([0.0e0, 1.0e0], 0);
        NCollection_Array1<int> mults_array = NCollection_Array1<int>.From([degree + 1, degree + 1], 0);

        NCollection_Array1<double> the_knots = new(knot_array, 0, 1, 2), the_new_knots = new(knot_array, 0, 1, 2);
        NCollection_Array1<int> the_mults = new(mults_array, 0, 1, 2), the_new_mults = new(mults_array, 0, 1, 2);

        BSplCLib.Trimming(degree,
                          false,
                          the_knots,
                          the_mults,
                          TPoles,
                          BSplCLib.NoWeights(),
                          trim_min,
                          trim_max,
                          the_new_knots,
                          the_new_mults,
                          NewTPoles,
                          BSplCLib.NoWeights());

        // readjustment is obviously redundant
        double SinD = Math.Sin(Delta), CosD = Math.Cos(Delta);
        gp_Pnt2d Pdeb = new(1.0, 0.0);
        gp_Pnt2d Pfin = new(CosD, SinD);

        double dtg = NewTPoles[1].Distance(NewTPoles[2]);
        NewTPoles[1] = Pdeb;
        gp_XY theXY = new(0.0, dtg);
        Pdeb.ChangeCoord() += theXY;
        NewTPoles[2] = Pdeb;

        // readjustment to Euclid
        dtg = NewTPoles[num_poles].Distance(NewTPoles[num_poles - 1]);
        NewTPoles[num_poles] = Pfin;
        theXY.SetCoord(dtg * SinD, -dtg * CosD);
        Pfin.ChangeCoord() += theXY;
        NewTPoles[num_poles - 1] = Pfin;

        // Rotation to return to the arc [LocUFirst, LocUFirst+Delta]
        T.SetRotation(gp.Origin2d(), locUFirst);
        for (ii = 1; ii <= num_poles; ii++)
        {
            NewTPoles[ii].Transform(T);
        }

        for (ii = 1; ii <= num_poles; ii++)
        {
            CosNumerator[ii] = NewTPoles[ii].X();
            SinNumerator[ii] = NewTPoles[ii].Y();
            Denominator[ii] = 1.0;
        }
    }
}
