// Port of: src/FoundationClasses/TKMath/Convert/Convert_CircleToBSplineCurve.hxx
// Port of: src/FoundationClasses/TKMath/Convert/Convert_CircleToBSplineCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

//  parametrization :
//  Reference : Rational B-spline for Curve and Surface Representation
//              Wayne Tiller  CADG September 1983
//  x(t) = (1 - t^2) / (1 + t^2)
//  y(t) =  2 t / (1 + t^2)
//  then t = std::sqrt(2) u /  ((std::sqrt(2) - 2) u + 2)
//  => u = 2 t / (std::sqrt(2) + (2 - std::sqrt(2)) t)

using static Occt.Convert_ParameterisationType;

namespace Occt;

/// <summary>
/// Converts a 2D circle into a rational B-spline curve. The circle's parametrisation is
/// P(U) = Loc + R * (cos(U) * XDir + sin(U) * YDir), U in [0, 2 Pi]; the B-spline's parametrisation range is not
/// [0, 2 Pi].
/// </summary>
public class Convert_CircleToBSplineCurve : Convert_ConicToBSplineCurve
{
    /// <summary>
    /// The periodic B-spline curve equivalent to the whole circle, with its orientation. Parameterisations other
    /// than <c>Convert_TgtThetaOver2</c> and <c>Convert_RationalC1</c> give a non-periodic curve on [0, 2 Pi].
    /// </summary>
    public Convert_CircleToBSplineCurve(in gp_Circ2d C,
                                        Convert_ParameterisationType Parameterisation = Convert_TgtThetaOver2)
        : base(0, 0, 0)
    {
        int ii;

        double R, value;
        NCollection_Array1<double> CosNumerator, SinNumerator;

        R = C.Radius();
        if (Parameterisation != Convert_TgtThetaOver2 && Parameterisation != Convert_RationalC1)
        {
            // In case if BuildCosAndSin does not know how to manage the periodicity
            // => trim on 0,2*PI
            myIsPeriodic = false;
            BuildCosAndSin(Parameterisation,
                           0,
                           2 * M_PI,
                           out CosNumerator,
                           out SinNumerator,
                           out myWeights,
                           out myDegree,
                           out myKnots,
                           out myMults);
        }
        else
        {
            myIsPeriodic = true;
            BuildCosAndSin(Parameterisation,
                           out CosNumerator,
                           out SinNumerator,
                           out myWeights,
                           out myDegree,
                           out myKnots,
                           out myMults);
        }

        myPoles = new NCollection_Array1<gp_Pnt2d>(1, CosNumerator.Length());

        gp_Dir2d Ox = C.XAxis().Direction();
        gp_Dir2d Oy = C.YAxis().Direction();
        gp_Trsf2d Trsf = new();
        Trsf.SetTransformation(C.XAxis(), gp.OX2d());
        if (Ox.X() * Oy.Y() - Ox.Y() * Oy.X() > 0.0e0)
        {
            value = R;
        }
        else
        {
            value = -R;
        }

        // Replace the bspline in the reference of the circle.
        // and calculate the weight of the bspline.

        for (ii = 1; ii <= myPoles.Length(); ii++)
        {
            myPoles[ii].SetCoord(1, R * CosNumerator[ii]);
            myPoles[ii].SetCoord(2, value * SinNumerator[ii]);
            myPoles[ii].Transform(Trsf);
        }
    }

    /// <summary>
    /// The B-spline curve equivalent to the arc of the circle from <paramref name="UFirst"/> to
    /// <paramref name="ULast"/> (radians), oriented from UFirst to ULast. Throws
    /// <see cref="Standard_DomainError"/> unless 0 &lt; ULast - UFirst &lt;= 2 Pi (+ PConfusion).
    /// </summary>
    public Convert_CircleToBSplineCurve(in gp_Circ2d C,
                                        double UFirst,
                                        double ULast,
                                        Convert_ParameterisationType Parameterisation = Convert_TgtThetaOver2)
        : base(0, 0, 0)
    {
        double delta = ULast - UFirst;
        double Eps = Precision.PConfusion();

        if ((delta > (2 * M_PI + Eps)) || (delta <= 0.0e0))
        {
            throw new Standard_DomainError("Convert_CircleToBSplineCurve");
        }

        int ii;
        double R, value;
        NCollection_Array1<double> CosNumerator, SinNumerator;

        R = C.Radius();
        myIsPeriodic = false;
        BuildCosAndSin(Parameterisation,
                       UFirst,
                       ULast,
                       out CosNumerator,
                       out SinNumerator,
                       out myWeights,
                       out myDegree,
                       out myKnots,
                       out myMults);

        myPoles = new NCollection_Array1<gp_Pnt2d>(1, CosNumerator.Length());

        gp_Dir2d Ox = C.XAxis().Direction();
        gp_Dir2d Oy = C.YAxis().Direction();
        gp_Trsf2d Trsf = new();
        Trsf.SetTransformation(C.XAxis(), gp.OX2d());
        if (Ox.X() * Oy.Y() - Ox.Y() * Oy.X() > 0.0e0)
        {
            value = R;
        }
        else
        {
            value = -R;
        }

        // Replace the bspline in the reference of the circle.
        // and calculate the weight of the bspline.

        for (ii = 1; ii <= myPoles.Length(); ii++)
        {
            myPoles[ii].SetCoord(1, R * CosNumerator[ii]);
            myPoles[ii].SetCoord(2, value * SinNumerator[ii]);
            myPoles[ii].Transform(Trsf);
        }
    }
}
