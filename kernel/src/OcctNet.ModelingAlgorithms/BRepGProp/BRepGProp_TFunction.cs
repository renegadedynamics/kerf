// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_TFunction.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_TFunction.lxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_TFunction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Measurement;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Kerf.Engine.Measurement.PropertyValueType;

namespace Occt;

/// <summary>
/// The integrand of the outer (T) integral of <see cref="BRepGProp_VinertGK"/>: at a parameter of the current
/// boundary curve, the integral of <see cref="BRepGProp_UFunction"/> from UMin to the curve's U, by adaptive
/// Gauss-Kronrod integration to the set tolerance.
/// </summary>
/// <remarks>The coefficients are those of <see cref="BRepGProp_UFunction"/>. OCCT keeps a copy of the face; so does the port.</remarks>
public class BRepGProp_TFunction : Function
{
    private readonly BRepGProp_Face mySurface;
    private readonly BRepGProp_UFunction myUFunction;
    private readonly double myUMin;
    private double myTolerance;
    private double myTolReached;
    private double myErrReached;
    private double myAbsError;
    private PropertyValueType myValueType;
    private readonly bool myIsByPoint;
    private int myNbPntOuter;

    /// <summary>
    /// The integrand on the face, with the location point, the mode, the coefficients, the lower bound
    /// <paramref name="theUMin"/> of the inner integrals and their tolerance.
    /// </summary>
    public BRepGProp_TFunction(BRepGProp_Face theSurface,
                               in Point3D theVertex,
                               bool IsByPoint,
                               ReadOnlySpan<double> theCoeffs,
                               double theUMin,
                               double theTolerance)
    {
        mySurface = new BRepGProp_Face(theSurface);
        myUFunction = new BRepGProp_UFunction(mySurface, theVertex, IsByPoint, theCoeffs);
        myUMin = theUMin;
        myTolerance = theTolerance;
        myTolReached = 0.0;
        myErrReached = 0.0;
        myAbsError = 0.0;
        myValueType = Unknown;
        myIsByPoint = IsByPoint;
        myNbPntOuter = 3;
    }

    /// <summary>Resets the errors reached.</summary>
    public void Init()
    {
        myTolReached = 0.0;
        myErrReached = 0.0;
        myAbsError = 0.0;
    }

    /// <summary>Sets the expected number of Kronrod points of the outer integral (made odd).</summary>
    public void SetNbKronrodPoints(int theNbPoints) => myNbPntOuter = (theNbPoints % 2 == 0) ? theNbPoints + 1 : theNbPoints;

    /// <summary>Sets the type of the value returned, passed on to the U function.</summary>
    public void SetValueType(PropertyValueType theType)
    {
        myValueType = theType;
        myUFunction.SetValueType(myValueType);
    }

    /// <summary>Sets the tolerance of the inner integration.</summary>
    public void SetTolerance(double theTolerance) => myTolerance = theTolerance;

    /// <summary>The relative error reached since the last <see cref="Init"/>.</summary>
    public double ErrorReached() => myErrReached;

    /// <summary>The absolute error reached since the last <see cref="Init"/>.</summary>
    public double AbsolutError() => myAbsError;

    /// <summary>The integral of the U function at the boundary parameter X; false if an integration failed.</summary>
    public override bool Value(double X, out double F)
    {
        const double tolU = 1.0e-9;

        Point2D aP2d;
        Vector2D aV2d;
        double aUMax;

        mySurface.D12d(X, out aP2d, out aV2d);
        aUMax = aP2d.X();

        if (aUMax - myUMin < tolU)
        {
            F = 0.0;
            return true;
        }

        NCollection_HArray1<double> aUKnots = mySurface.GetUKnots(myUMin, aUMax);
        myUFunction.SetVParam(aP2d.Y());

        // Compute the integral from myUMin to aUMax of myUFunction.
        int i;
        double aCoeff = aV2d.Y();
        // int aNbUIntervals = aUKnots->Length() - 1;
        // double    aTol          = myTolerance/aNbUIntervals;
        double aTol = myTolerance;

        // aTol /= myNbPntOuter;

        if (myValueType == Mass)
        {
            if (myIsByPoint)
            {
                aCoeff /= 3.0;
            }
        }
        else if (myValueType == CenterMassX || myValueType == CenterMassY || myValueType == CenterMassZ)
        {
            if (myIsByPoint)
            {
                aCoeff *= 0.25;
            }
        }
        else if (myValueType == InertiaXX || myValueType == InertiaYY || myValueType == InertiaZZ
                 || myValueType == InertiaXY || myValueType == InertiaXZ || myValueType == InertiaYZ)
        {
            if (myIsByPoint)
            {
                aCoeff *= 0.2;
            }
        }
        else
        {
            // OCCT leaves F unset here (decision 221's 0)
            F = 0.0;
            return false;
        }

        double aAbsCoeff = Math.Abs(aCoeff);

        if (aAbsCoeff <= global::Kerf.Engine.Foundation.Precision.Angular())
        {
            // No need to compute the integral. The value will be equal to 0.
            F = 0.0;
            return true;
        }
        // else if (aAbsCoeff > 10.*aTol)
        //   aTol /= aAbsCoeff;
        // else
        //   aTol = 0.1;

        int iU = aUKnots.Upper();
        int aNbPntsStart;
        int aNbMaxIter = 1000;
        var anIntegral = new KronrodSingleIntegration();
        double aLocalErr = 0.0;

        i = aUKnots.Lower();
        F = 0.0;

        // Epmirical criterion
        aNbPntsStart = Min(15, mySurface.UIntegrationOrder() / (aUKnots.Length() - 1) + 1);
        aNbPntsStart = Max(5, aNbPntsStart);

        while (i < iU)
        {
            double aU1 = aUKnots.Value(i++);
            double aU2 = aUKnots.Value(i);

            if (aU2 - aU1 < tolU)
            {
                continue;
            }

            anIntegral.Perform(myUFunction, aU1, aU2, aNbPntsStart, aTol, aNbMaxIter);

            if (!anIntegral.IsDone)
            {
                return false;
            }

            F += anIntegral.Value();
            aLocalErr += anIntegral.AbsolutError();
            // std::cout << " TFunction : " << anIntegral.NbIterReached() << " " <<
            // anIntegral.AbsolutError() << std::endl;
        }

        F *= aCoeff;
        aLocalErr *= aAbsCoeff;
        myAbsError = Max(myAbsError, aLocalErr);

        myTolReached += aLocalErr;

        if (Math.Abs(F) > Epsilon(1.0))
        {
            aLocalErr /= Math.Abs(F);
        }

        myErrReached = Max(myErrReached, aLocalErr);

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
