// Port of: src/FoundationClasses/TKMath/Convert/Convert_PolynomialCosAndSin.hxx
// Port of: src/FoundationClasses/TKMath/Convert/Convert_PolynomialCosAndSin.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;

namespace Kerf.Engine.Geometry.Conversion;

/// <summary>The free function of Convert_PolynomialCosAndSin.hxx (not exported by OCCT).</summary>
internal static class PolynomialCosAndSin
{
    private static double Locate(double angfin, NCollection_Array1<Point2D> tPoles, double Umin, double Umax)
    {
        double umin = Umin;
        double umax = Umax;
        double ptol = global::Kerf.Engine.Foundation.Precision.Angular();
        double utol = global::Kerf.Engine.Foundation.Precision.PConfusion();
        while (Math.Abs(umax - umin) >= utol)
        {
            double ptest = (umax + umin) / 2.0;
            global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(ptest, tPoles, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), out Point2D valP);
            double theta = Math.Atan2(valP.Y, valP.X);
            if (theta < 0.0)
            {
                theta += 2.0 * M_PI;
            }
            if (Math.Abs(theta - angfin) < ptol)
            {
                return ptest;
            }
            if (theta < angfin)
            {
                umin = ptest;
            }
            else if (theta > angfin)
            {
                umax = ptest;
            }
        }
        return (umin + umax) / 2.0;
    }

    /// <summary>
    /// The poles of a polynomial approximation of degree <paramref name="num_poles"/> - 1 of (cos, sin) on
    /// [<paramref name="uFirst"/>, <paramref name="uLast"/>] (Tiller's algorithm); the denominator is 1.
    /// </summary>
    internal static void BuildPolynomialCosAndSin(double uFirst,
                                                  double uLast,
                                                  int num_poles,
                                                  NCollection_Array1<double> cosNumerator,
                                                  NCollection_Array1<double> sinNumerator,
                                                  NCollection_Array1<double> denominator)
    {
        double delta, locUFirst,
          t_min, t_max, trim_min, trim_max, middle, angle, pi2 = 2 * M_PI;
        int ii, degree = num_poles - 1;
        locUFirst = uFirst;

        // Return UFirst in [-2PI; 2PI]
        // to make rotations without risk
        for (int aStep = 0; locUFirst > pi2; StepLimit.Next(ref aStep))
        {
            locUFirst -= pi2;
        }
        for (int aStep = 0; locUFirst < -pi2; StepLimit.Next(ref aStep))
        {
            locUFirst += pi2;
        }

        // Return to the arc [0, Delta]
        delta = uLast - uFirst;
        middle = 0.5e0 * delta;

        // Coincide the required bisector of the angular sector with
        // axis -Ox. Definition of the circle in Bezier of degree 7 so that
        // parameter 1/2 of Bezier is exactly a point of the bisectrix
        // of the required angular sector.
        angle = middle - M_PI;

        // Bezier control points for a unit circle (degree-7 polynomial approximation).
        // Coefficients from Tiller's algorithm for polynomial cos/sin representation.
        NCollection_Array1<Point2D> tPoles = new(1, 8), newTPoles = new(1, 8);
        tPoles[1] = tPoles[1].WithCoord(1.0, 0.0);
        tPoles[2] = tPoles[2].WithCoord(1.0, 1.013854);
        tPoles[3] = tPoles[3].WithCoord(-0.199043, 1.871905);
        tPoles[4] = tPoles[4].WithCoord(-1.937729, 1.057323);
        tPoles[5] = tPoles[5].WithCoord(-1.937729, -1.057323);
        tPoles[6] = tPoles[6].WithCoord(-0.199043, -1.871905);
        tPoles[7] = tPoles[7].WithCoord(1.0, -1.013854);
        tPoles[8] = tPoles[8].WithCoord(1.0, 0.0);
        Transform2D t = new Transform2D().WithRotation(Constants.Origin2d(), angle);
        for (ii = 1; ii <= num_poles; ii++)
        {
            tPoles[ii] = tPoles[ii].Transformed(t);
        }

        t_min = 1.0e0 - (delta * 1.3e0 / M_PI);
        t_min *= 0.5e0;
        t_min = Max(t_min, 0.0e0);
        t_max = 1.0e0 + (delta * 1.3e0 / M_PI);
        t_max *= 0.5e0;
        t_max = Min(t_max, 1.0e0);
        trim_max = Locate(delta, tPoles, t_min, t_max);
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

        global::Kerf.Engine.Geometry.Splines.BSplCLib.Trimming(degree,
                          false,
                          the_knots,
                          the_mults,
                          tPoles,
                          global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(),
                          trim_min,
                          trim_max,
                          the_new_knots,
                          the_new_mults,
                          newTPoles,
                          global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights());

        // readjustment is obviously redundant
        double sinD = Math.Sin(delta), cosD = Math.Cos(delta);
        Point2D pdeb = new(1.0, 0.0);
        Point2D pfin = new(cosD, sinD);

        double dtg = newTPoles[1].DistanceTo(newTPoles[2]);
        newTPoles[1] = pdeb;
        Xy theXY = new(0.0, dtg);
        pdeb = new Point2D(pdeb.XY() + theXY);
        newTPoles[2] = pdeb;

        // readjustment to Euclid
        dtg = newTPoles[num_poles].DistanceTo(newTPoles[num_poles - 1]);
        newTPoles[num_poles] = pfin;
        theXY.SetCoord(dtg * sinD, -dtg * cosD);
        pfin = new Point2D(pfin.XY() + theXY);
        newTPoles[num_poles - 1] = pfin;

        // Rotation to return to the arc [LocUFirst, LocUFirst+Delta]
        t = t.WithRotation(Constants.Origin2d(), locUFirst);
        for (ii = 1; ii <= num_poles; ii++)
        {
            newTPoles[ii] = newTPoles[ii].Transformed(t);
        }

        for (ii = 1; ii <= num_poles; ii++)
        {
            cosNumerator[ii] = newTPoles[ii].X;
            sinNumerator[ii] = newTPoles[ii].Y;
            denominator[ii] = 1.0;
        }
    }
}
