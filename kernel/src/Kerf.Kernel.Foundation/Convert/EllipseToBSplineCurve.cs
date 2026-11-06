// Port of: src/FoundationClasses/TKMath/Convert/Convert_EllipseToBSplineCurve.hxx
// Port of: src/FoundationClasses/TKMath/Convert/Convert_EllipseToBSplineCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

//  parameterization :
//  Reference : Rational B-spline for Curve and Surface Representation
//              Wayne Tiller  CADG September 1983
//  x(t) = (1 - t^2) / (1 + t^2)
//  y(t) =  2 t / (1 + t^2)
//  then t = std::sqrt(2) u /  ((std::sqrt(2) - 2) u + 2)
//  => u = 2 t / (std::sqrt(2) + (2 - std::sqrt(2)) t)

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;

namespace Kerf.Engine.Geometry.Conversion;

/// <summary>
/// Converts a 2D ellipse into a rational B-spline curve. The ellipse's parametrisation is
/// P(U) = Loc + MajorRadius * cos(U) * XDir + MinorRadius * sin(U) * YDir, U in [0, 2 Pi].
/// </summary>
internal class EllipseToBSplineCurve : ConicToBSplineCurve
{
    /// <summary>
    /// The periodic B-spline curve equivalent to the whole ellipse, with its orientation. Parameterisations other
    /// than <c>Convert_TgtThetaOver2</c> and <c>Convert_RationalC1</c> give a non-periodic curve on [0, 2 Pi].
    /// </summary>
    public EllipseToBSplineCurve(in Ellipse2D e,
                                         ParameterisationType parameterisation = TgtThetaOver2)
        : base(0, 0, 0)
    {
        int ii;

        double R, r, value;
        NCollection_Array1<double> cosNumerator, sinNumerator;

        R = e.MajorRadius;
        r = e.MinorRadius;

        if (parameterisation != TgtThetaOver2 && parameterisation != RationalC1)
        {
            // If BuildCosAndSin cannot manage the periodicity
            // => trim on 0,2*PI
            _isPeriodic = false;
            BuildCosAndSin(parameterisation,
                           0,
                           2 * M_PI,
                           out cosNumerator,
                           out sinNumerator,
                           out _weights,
                           out _degree,
                           out _knots,
                           out _mults);
        }
        else
        {
            _isPeriodic = true;
            BuildCosAndSin(parameterisation,
                           out cosNumerator,
                           out sinNumerator,
                           out _weights,
                           out _degree,
                           out _knots,
                           out _mults);
        }

        _poles = new NCollection_Array1<Point2D>(1, cosNumerator.Length());

        Direction2D ox = e.XAxis.Direction;
        Direction2D oy = e.YAxis.Direction;
        Transform2D trsf = new Transform2D().WithTransformation(e.XAxis, Constants.OX2d());
        if (ox.X * oy.Y - ox.Y * oy.X > 0.0e0)
        {
            value = r;
        }
        else
        {
            value = -r;
        }

        // Replace the bspline in the mark of the circle.
        // and calculate the weight of the bspline.

        for (ii = 1; ii <= _poles.Length(); ii++)
        {
            _poles[ii] = _poles[ii].WithCoord(1, R * cosNumerator[ii]);
            _poles[ii] = _poles[ii].WithCoord(2, value * sinNumerator[ii]);
            _poles[ii] = _poles[ii].Transformed(trsf);
        }
    }

    /// <summary>
    /// The B-spline curve equivalent to the arc of the ellipse from <paramref name="uFirst"/> to
    /// <paramref name="uLast"/>, oriented from UFirst to ULast. Throws <see cref="DomainErrorException"/> unless
    /// 0 &lt; ULast - UFirst &lt;= 2 Pi (+ PConfusion).
    /// </summary>
    public EllipseToBSplineCurve(in Ellipse2D e,
                                         double uFirst,
                                         double uLast,
                                         ParameterisationType parameterisation = TgtThetaOver2)
        : base(0, 0, 0)
    {
        double tol = global::Kerf.Engine.Foundation.Precision.PConfusion();
        double delta = uLast - uFirst;
        Standard_DomainError_Raise_if((delta > (2 * M_PI + tol)) || (delta <= 0.0e0),
                                      "Convert_EllipseToBSplineCurve");
        int ii;
        double R, r, value;
        NCollection_Array1<double> cosNumerator, sinNumerator;

        R = e.MajorRadius;
        r = e.MinorRadius;

        _isPeriodic = false;
        BuildCosAndSin(parameterisation,
                       uFirst,
                       uLast,
                       out cosNumerator,
                       out sinNumerator,
                       out _weights,
                       out _degree,
                       out _knots,
                       out _mults);

        _poles = new NCollection_Array1<Point2D>(1, cosNumerator.Length());

        Direction2D ox = e.XAxis.Direction;
        Direction2D oy = e.YAxis.Direction;
        Transform2D trsf = new Transform2D().WithTransformation(e.XAxis, Constants.OX2d());
        if (ox.X * oy.Y - ox.Y * oy.X > 0.0e0)
        {
            value = r;
        }
        else
        {
            value = -r;
        }

        // Replace the bspline in the mark of the circle.
        // and calculate the weight of the bspline.

        for (ii = 1; ii <= _poles.Length(); ii++)
        {
            _poles[ii] = _poles[ii].WithCoord(1, R * cosNumerator[ii]);
            _poles[ii] = _poles[ii].WithCoord(2, value * sinNumerator[ii]);
            _poles[ii] = _poles[ii].Transformed(trsf);
        }
    }
}
