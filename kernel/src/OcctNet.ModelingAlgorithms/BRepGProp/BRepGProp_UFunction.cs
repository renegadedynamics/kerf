// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_UFunction.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_UFunction.lxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_UFunction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Measurement;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Kerf.Engine.Measurement.PropertyValueType;

namespace Occt;

/// <summary>
/// The integrand of the inner (U) integral of <see cref="BRepGProp_VinertGK"/>: for the value type set, the volume,
/// a coordinate of the centre of mass or a moment of inertia of the region between the face and a point
/// (IsByPoint) or a plane, at a fixed V parameter.
/// </summary>
/// <remarks>
/// With IsByPoint the coefficients are the X, Y and Z shift of the point the inertia is computed at from the
/// location; otherwise they are the four coefficients of the plane, its constant shifted to the location. OCCT keeps
/// a copy of the face and a pointer to the caller's coefficients; the port keeps a copy of both.
/// </remarks>
public class BRepGProp_UFunction : Function
{
    private readonly BRepGProp_Face mySurface;
    private readonly Point3D myVertex;
    private readonly double[] myCoeffs;
    private double myVParam;
    private PropertyValueType myValueType;
    private readonly bool myIsByPoint;

    /// <summary>The integrand on the face, with the location point, the mode and the coefficients (see the remarks).</summary>
    public BRepGProp_UFunction(BRepGProp_Face theSurface, in Point3D theVertex, bool IsByPoint, ReadOnlySpan<double> theCoeffs)
    {
        mySurface = new BRepGProp_Face(theSurface);
        myVertex = theVertex;
        myCoeffs = theCoeffs.ToArray();
        myVParam = 0.0;
        myValueType = Unknown;
        myIsByPoint = IsByPoint;
    }

    /// <summary>Sets the type of the value returned.</summary>
    public void SetValueType(PropertyValueType theType) => myValueType = theType;

    /// <summary>Sets the V parameter, constant during the integration.</summary>
    public void SetVParam(double theVParam) => myVParam = theVParam;

    /// <summary>The value of the function at X; false for an unknown value type.</summary>
    public override bool Value(double X, out double F)
    {
        // Volume computation
        if (myValueType == Mass)
        {
            Xyz aPMP0;
            double aTmpPar1 = 0.0;
            double aTmpPar2 = 0.0;

            F = VolumeValue(X, out aPMP0, ref aTmpPar1, ref aTmpPar2);
            return true;
        }

        // Center of mass computation
        if (myValueType == CenterMassX || myValueType == CenterMassY || myValueType == CenterMassZ)
        {
            return CenterMassValue(X, out F);
        }

        // Inertia computation
        if (myValueType == InertiaXX || myValueType == InertiaYY || myValueType == InertiaZZ
            || myValueType == InertiaXY || myValueType == InertiaXZ || myValueType == InertiaYZ)
        {
            return InertiaValue(X, out F);
        }

        // OCCT leaves F unset here (decision 221's 0)
        F = 0.0;
        return false;
    }

    // Returns the value for volume computation; thePMP0 is PSurf(X, Y) minus the location; theS and theD1 are set in
    // the plane mode only (OCCT leaves them unchanged by point)
    private double VolumeValue(double X, out Xyz thePMP0, ref double theS, ref double theD1)
    {
        mySurface.Normal(X, myVParam, out Point3D aPnt, out Vector3D aNorm);

        thePMP0 = aPnt.XYZ().Subtracted(myVertex.XYZ());

        // Volume computation for ByPoint mode.
        if (myIsByPoint)
        {
            return thePMP0.Dot(aNorm.XYZ());
        }

        // Volume and additional coefficients computation for ByPlane mode.
        double[] aCoeff = myCoeffs;
        theS = aNorm.X() * aCoeff[0] + aNorm.Y() * aCoeff[1] + aNorm.Z() * aCoeff[2];
        theD1 = thePMP0.X() * aCoeff[0] + thePMP0.Y() * aCoeff[1] + thePMP0.Z() * aCoeff[2] - aCoeff[3];

        return theS * theD1;
    }

    // Returns a value for the center of mass computation
    private bool CenterMassValue(double X, out double F)
    {
        Xyz aPmP0;
        double aS = 0.0;
        double aD1 = 0.0;

        F = VolumeValue(X, out aPmP0, ref aS, ref aD1);

        // Center of mass computation for ByPoint mode.
        if (myIsByPoint)
        {
            switch (myValueType)
            {
                case CenterMassX:
                    F *= aPmP0.X();
                    break;
                case CenterMassY:
                    F *= aPmP0.Y();
                    break;
                case CenterMassZ:
                    F *= aPmP0.Z();
                    break;
                default:
                    return false;
            }

            return true;
        }

        // Center of mass computation for ByPlane mode.
        double[] aCoeff = myCoeffs;

        switch (myValueType)
        {
            case CenterMassX:
                F *= (aPmP0.X() - 0.5 * aCoeff[0] * aD1);
                break;
            case CenterMassY:
                F *= (aPmP0.Y() - 0.5 * aCoeff[1] * aD1);
                break;
            case CenterMassZ:
                F *= (aPmP0.Z() - 0.5 * aCoeff[2] * aD1);
                break;
            default:
                return false;
        }

        return true;
    }

    // Computes the value of inertia
    private bool InertiaValue(double X, out double F)
    {
        Xyz aPmP0;
        double aS = 0.0;
        double aD1 = 0.0;
        double aParam1;
        double aParam2;
        double[] aCoeffs = myCoeffs;

        F = VolumeValue(X, out aPmP0, ref aS, ref aD1);

        // Inertia computation for ByPoint mode.
        if (myIsByPoint)
        {
            switch (myValueType)
            {
                case InertiaXX:
                case InertiaYZ:
                    aParam1 = aPmP0.Y() - aCoeffs[1];
                    aParam2 = aPmP0.Z() - aCoeffs[2];
                    break;
                case InertiaYY:
                case InertiaXZ:
                    aParam1 = aPmP0.X() - aCoeffs[0];
                    aParam2 = aPmP0.Z() - aCoeffs[2];
                    break;
                case InertiaZZ:
                case InertiaXY:
                    aParam1 = aPmP0.X() - aCoeffs[0];
                    aParam2 = aPmP0.Y() - aCoeffs[1];
                    break;
                default:
                    return false;
            }

            if (myValueType == InertiaXX || myValueType == InertiaYY || myValueType == InertiaZZ)
            {
                F *= aParam1 * aParam1 + aParam2 * aParam2;
            }
            else
            {
                F *= -aParam1 * aParam2;
            }

            return true;
        }

        // Inertia computation for ByPlane mode.
        double aD2 = aD1 * aD1;
        double aD3 = aD1 * aD2 / 3.0;
        double aPPar1;
        double aPPar2;
        double aCoeff1;
        double aCoeff2;

        // Inertia computation for XX, YY and ZZ.
        if (myValueType == InertiaXX || myValueType == InertiaYY || myValueType == InertiaZZ)
        {
            if (myValueType == InertiaXX)
            {
                aPPar1 = aPmP0.Y();
                aPPar2 = aPmP0.Z();
                aCoeff1 = aCoeffs[1];
                aCoeff2 = aCoeffs[2];
            }
            else if (myValueType == InertiaYY)
            {
                aPPar1 = aPmP0.X();
                aPPar2 = aPmP0.Z();
                aCoeff1 = aCoeffs[0];
                aCoeff2 = aCoeffs[2];
            }
            else
            { // myValueType == GProp_InertiaZZ
                aPPar1 = aPmP0.X();
                aPPar2 = aPmP0.Y();
                aCoeff1 = aCoeffs[0];
                aCoeff2 = aCoeffs[1];
            }

            aPPar1 -= aCoeff1 * aD1;
            aPPar2 -= aCoeff2 * aD1;
            aParam1 = aPPar1 * aPPar1 * aD1 + aPPar1 * aCoeff1 * aD2 + aCoeff1 * aCoeff1 * aD3;
            aParam2 = aPPar2 * aPPar2 * aD1 + aPPar2 * aCoeff2 * aD2 + aCoeff2 * aCoeff2 * aD3;

            F = (aParam1 + aParam2) * aS;

            return true;
        }

        // Inertia computation for XY, YZ and XZ.
        if (myValueType == InertiaXY || myValueType == InertiaYZ || myValueType == InertiaXZ)
        {
            if (myValueType == InertiaXY)
            {
                aPPar1 = aPmP0.X();
                aPPar2 = aPmP0.Y();
                aCoeff1 = aCoeffs[0];
                aCoeff2 = aCoeffs[1];
            }
            else if (myValueType == InertiaYZ)
            {
                aPPar1 = aPmP0.Y();
                aPPar2 = aPmP0.Z();
                aCoeff1 = aCoeffs[1];
                aCoeff2 = aCoeffs[2];
            }
            else
            { // myValueType == GProp_InertiaXZ
                aPPar1 = aPmP0.X();
                aPPar2 = aPmP0.Z();
                aCoeff1 = aCoeffs[0];
                aCoeff2 = aCoeffs[2];
            }

            aD2 *= 0.5;
            aPPar1 -= aCoeff1 * aD1;
            aPPar2 -= aCoeff2 * aD1;
            aParam1 = aPPar1 * aPPar2 * aD1 + (aPPar1 * aCoeff2 + aPPar2 * aCoeff1) * aD2 + aCoeff1 * aCoeff2 * aD3;

            F = -aParam1 * aS;

            return true;
        }

        return false;
    }
}
