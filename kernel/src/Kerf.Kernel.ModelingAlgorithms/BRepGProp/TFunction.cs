// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_TFunction.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_TFunction.lxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_TFunction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Measurement;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Measurement.PropertyValueType;

namespace Kerf.Engine.Measurement;

/// <summary>
/// The integrand of the outer (T) integral of <see cref="VinertGK"/>: at a parameter of the current
/// boundary curve, the integral of <see cref="UFunction"/> from UMin to the curve's U, by adaptive
/// Gauss-Kronrod integration to the set tolerance.
/// </summary>
/// <remarks>The coefficients are those of <see cref="UFunction"/>. OCCT keeps a copy of the face; so does the port.</remarks>
public class TFunction : Function
{
    private readonly FaceIntegrand _surface;
    private readonly UFunction _uFunction;
    private readonly double _uMin;
    private double _tolerance;
    private double _tolReached;
    private double _errReached;
    private double _absError;
    private PropertyValueType _valueType;
    private readonly bool _isByPoint;
    private int _nbPntOuter;

    /// <summary>
    /// The integrand on the face, with the location point, the mode, the coefficients, the lower bound
    /// <paramref name="uMin"/> of the inner integrals and their tolerance.
    /// </summary>
    public TFunction(FaceIntegrand surface,
                               in Point3D vertex,
                               bool isByPoint,
                               ReadOnlySpan<double> coeffs,
                               double uMin,
                               double tolerance)
    {
        _surface = new FaceIntegrand(surface);
        _uFunction = new UFunction(_surface, vertex, isByPoint, coeffs);
        _uMin = uMin;
        _tolerance = tolerance;
        _tolReached = 0.0;
        _errReached = 0.0;
        _absError = 0.0;
        _valueType = Unknown;
        _isByPoint = isByPoint;
        _nbPntOuter = 3;
    }

    /// <summary>Resets the errors reached.</summary>
    public void Init()
    {
        _tolReached = 0.0;
        _errReached = 0.0;
        _absError = 0.0;
    }

    /// <summary>Sets the expected number of Kronrod points of the outer integral (made odd).</summary>
    public void SetNbKronrodPoints(int nbPoints) => _nbPntOuter = (nbPoints % 2 == 0) ? nbPoints + 1 : nbPoints;

    /// <summary>Sets the type of the value returned, passed on to the U function.</summary>
    public void SetValueType(PropertyValueType type)
    {
        _valueType = type;
        _uFunction.SetValueType(_valueType);
    }

    /// <summary>Sets the tolerance of the inner integration.</summary>
    public void SetTolerance(double tolerance) => _tolerance = tolerance;

    /// <summary>The relative error reached since the last <see cref="Init"/>.</summary>
    public double ErrorReached() => _errReached;

    /// <summary>The absolute error reached since the last <see cref="Init"/>.</summary>
    public double AbsolutError() => _absError;

    /// <summary>The integral of the U function at the boundary parameter X; false if an integration failed.</summary>
    public override bool Value(double x, out double f)
    {
        const double tolU = 1.0e-9;

        Point2D p2d;
        Vector2D v2d;
        double uMax;

        _surface.D12d(x, out p2d, out v2d);
        uMax = p2d.X;

        if (uMax - _uMin < tolU)
        {
            f = 0.0;
            return true;
        }

        NCollection_HArray1<double> uKnots = _surface.GetUKnots(_uMin, uMax);
        _uFunction.SetVParam(p2d.Y);

        // Compute the integral from myUMin to aUMax of myUFunction.
        int i;
        double coeff = v2d.Y;
        // int aNbUIntervals = aUKnots->Length() - 1;
        // double    aTol          = myTolerance/aNbUIntervals;
        double tol = _tolerance;

        // aTol /= myNbPntOuter;

        if (_valueType == Mass)
        {
            if (_isByPoint)
            {
                coeff /= 3.0;
            }
        }
        else if (_valueType == CenterMassX || _valueType == CenterMassY || _valueType == CenterMassZ)
        {
            if (_isByPoint)
            {
                coeff *= 0.25;
            }
        }
        else if (_valueType == InertiaXX || _valueType == InertiaYY || _valueType == InertiaZZ
                 || _valueType == InertiaXY || _valueType == InertiaXZ || _valueType == InertiaYZ)
        {
            if (_isByPoint)
            {
                coeff *= 0.2;
            }
        }
        else
        {
            // OCCT leaves F unset here (decision 221's 0)
            f = 0.0;
            return false;
        }

        double absCoeff = Math.Abs(coeff);

        if (absCoeff <= global::Kerf.Engine.Foundation.Precision.Angular())
        {
            // No need to compute the integral. The value will be equal to 0.
            f = 0.0;
            return true;
        }
        // else if (aAbsCoeff > 10.*aTol)
        //   aTol /= aAbsCoeff;
        // else
        //   aTol = 0.1;

        int iU = uKnots.Upper();
        int nbPntsStart;
        int nbMaxIter = 1000;
        var integral = new KronrodSingleIntegration();
        double localErr = 0.0;

        i = uKnots.Lower();
        f = 0.0;

        // Epmirical criterion
        nbPntsStart = Min(15, _surface.UIntegrationOrder() / (uKnots.Length() - 1) + 1);
        nbPntsStart = Max(5, nbPntsStart);

        while (i < iU)
        {
            double u1 = uKnots.Value(i++);
            double u2 = uKnots.Value(i);

            if (u2 - u1 < tolU)
            {
                continue;
            }

            integral.Perform(_uFunction, u1, u2, nbPntsStart, tol, nbMaxIter);

            if (!integral.IsDone)
            {
                return false;
            }

            f += integral.Value();
            localErr += integral.AbsolutError();
            // std::cout << " TFunction : " << anIntegral.NbIterReached() << " " <<
            // anIntegral.AbsolutError() << std::endl;
        }

        f *= coeff;
        localErr *= absCoeff;
        _absError = Max(_absError, localErr);

        _tolReached += localErr;

        if (Math.Abs(f) > Epsilon(1.0))
        {
            localErr /= Math.Abs(f);
        }

        _errReached = Max(_errReached, localErr);

        return true;
    }

    /// <summary>Always 0; OCCT's update of the error and the number of points is commented out.</summary>
    public override int GetStateNumber()
    {
        // myErrReached  = myTolReached;
        // myTolReached  = 0.;
        // myNbPntOuter += RealToInt(0.5*myNbPntOuter);

        // if (myNbPntOuter%2 == 0)
        // myNbPntOuter++;

        return 0;
    }
}
