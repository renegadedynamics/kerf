// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_UFunction.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_UFunction.lxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_UFunction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Kerf.Engine.Measurement.PropertyValueType;

namespace Kerf.Engine.Measurement;

/// <summary>
/// The integrand of the inner (U) integral of <see cref="VinertGK"/>: for the value type set, the volume,
/// a coordinate of the centre of mass or a moment of inertia of the region between the face and a point
/// (IsByPoint) or a plane, at a fixed V parameter.
/// </summary>
/// <remarks>
/// With IsByPoint the coefficients are the X, Y and Z shift of the point the inertia is computed at from the
/// location; otherwise they are the four coefficients of the plane, its constant shifted to the location. OCCT keeps
/// a copy of the face and a pointer to the caller's coefficients; the port keeps a copy of both.
/// </remarks>
internal class UFunction : Function
{
    private readonly FaceIntegrand _surface;
    private readonly Point3D _vertex;
    private readonly double[] _coeffs;
    private double _vParam;
    private PropertyValueType _valueType;
    private readonly bool _isByPoint;

    /// <summary>The integrand on the face, with the location point, the mode and the coefficients (see the remarks).</summary>
    public UFunction(FaceIntegrand surface, in Point3D vertex, bool isByPoint, ReadOnlySpan<double> coeffs)
    {
        _surface = new FaceIntegrand(surface);
        _vertex = vertex;
        _coeffs = coeffs.ToArray();
        _vParam = 0.0;
        _valueType = Unknown;
        _isByPoint = isByPoint;
    }

    /// <summary>Sets the type of the value returned.</summary>
    public void SetValueType(PropertyValueType type) => _valueType = type;

    /// <summary>Sets the V parameter, constant during the integration.</summary>
    public void SetVParam(double vParam) => _vParam = vParam;

    /// <summary>The value of the function at X; false for an unknown value type.</summary>
    public override bool Value(double x, out double f)
    {
        // Volume computation
        if (_valueType == Mass)
        {
            Xyz pmp0;
            double tmpPar1 = 0.0;
            double tmpPar2 = 0.0;

            f = VolumeValue(x, out pmp0, ref tmpPar1, ref tmpPar2);
            return true;
        }

        // Center of mass computation
        if (_valueType == CenterMassX || _valueType == CenterMassY || _valueType == CenterMassZ)
        {
            return CenterMassValue(x, out f);
        }

        // Inertia computation
        if (_valueType == InertiaXX || _valueType == InertiaYY || _valueType == InertiaZZ
            || _valueType == InertiaXY || _valueType == InertiaXZ || _valueType == InertiaYZ)
        {
            return InertiaValue(x, out f);
        }

        // OCCT leaves F unset here (decision 221's 0)
        f = 0.0;
        return false;
    }

    // Returns the value for volume computation; thePMP0 is PSurf(X, Y) minus the location; theS and theD1 are set in
    // the plane mode only (OCCT leaves them unchanged by point)
    private double VolumeValue(double x, out Xyz pmp0, ref double s, ref double d1)
    {
        _surface.Normal(x, _vParam, out Point3D pnt, out Vector3D norm);

        pmp0 = pnt.XYZ().Subtracted(_vertex.XYZ());

        // Volume computation for ByPoint mode.
        if (_isByPoint)
        {
            return pmp0.Dot(norm.XYZ());
        }

        // Volume and additional coefficients computation for ByPlane mode.
        double[] coeff = _coeffs;
        s = norm.X * coeff[0] + norm.Y * coeff[1] + norm.Z * coeff[2];
        d1 = pmp0.X() * coeff[0] + pmp0.Y() * coeff[1] + pmp0.Z() * coeff[2] - coeff[3];

        return s * d1;
    }

    // Returns a value for the center of mass computation
    private bool CenterMassValue(double x, out double f)
    {
        Xyz pmP0;
        double s = 0.0;
        double d1 = 0.0;

        f = VolumeValue(x, out pmP0, ref s, ref d1);

        // Center of mass computation for ByPoint mode.
        if (_isByPoint)
        {
            switch (_valueType)
            {
                case CenterMassX:
                    f *= pmP0.X();
                    break;
                case CenterMassY:
                    f *= pmP0.Y();
                    break;
                case CenterMassZ:
                    f *= pmP0.Z();
                    break;
                default:
                    return false;
            }

            return true;
        }

        // Center of mass computation for ByPlane mode.
        double[] coeff = _coeffs;

        switch (_valueType)
        {
            case CenterMassX:
                f *= (pmP0.X() - 0.5 * coeff[0] * d1);
                break;
            case CenterMassY:
                f *= (pmP0.Y() - 0.5 * coeff[1] * d1);
                break;
            case CenterMassZ:
                f *= (pmP0.Z() - 0.5 * coeff[2] * d1);
                break;
            default:
                return false;
        }

        return true;
    }

    // Computes the value of inertia
    private bool InertiaValue(double x, out double f)
    {
        Xyz pmP0;
        double s = 0.0;
        double d1 = 0.0;
        double param1;
        double param2;
        double[] coeffs = _coeffs;

        f = VolumeValue(x, out pmP0, ref s, ref d1);

        // Inertia computation for ByPoint mode.
        if (_isByPoint)
        {
            switch (_valueType)
            {
                case InertiaXX:
                case InertiaYZ:
                    param1 = pmP0.Y() - coeffs[1];
                    param2 = pmP0.Z() - coeffs[2];
                    break;
                case InertiaYY:
                case InertiaXZ:
                    param1 = pmP0.X() - coeffs[0];
                    param2 = pmP0.Z() - coeffs[2];
                    break;
                case InertiaZZ:
                case InertiaXY:
                    param1 = pmP0.X() - coeffs[0];
                    param2 = pmP0.Y() - coeffs[1];
                    break;
                default:
                    return false;
            }

            if (_valueType == InertiaXX || _valueType == InertiaYY || _valueType == InertiaZZ)
            {
                f *= param1 * param1 + param2 * param2;
            }
            else
            {
                f *= -param1 * param2;
            }

            return true;
        }

        // Inertia computation for ByPlane mode.
        double d2 = d1 * d1;
        double d3 = d1 * d2 / 3.0;
        double pPar1;
        double pPar2;
        double coeff1;
        double coeff2;

        // Inertia computation for XX, YY and ZZ.
        if (_valueType == InertiaXX || _valueType == InertiaYY || _valueType == InertiaZZ)
        {
            if (_valueType == InertiaXX)
            {
                pPar1 = pmP0.Y();
                pPar2 = pmP0.Z();
                coeff1 = coeffs[1];
                coeff2 = coeffs[2];
            }
            else if (_valueType == InertiaYY)
            {
                pPar1 = pmP0.X();
                pPar2 = pmP0.Z();
                coeff1 = coeffs[0];
                coeff2 = coeffs[2];
            }
            else
            { // myValueType == GProp_InertiaZZ
                pPar1 = pmP0.X();
                pPar2 = pmP0.Y();
                coeff1 = coeffs[0];
                coeff2 = coeffs[1];
            }

            pPar1 -= coeff1 * d1;
            pPar2 -= coeff2 * d1;
            param1 = pPar1 * pPar1 * d1 + pPar1 * coeff1 * d2 + coeff1 * coeff1 * d3;
            param2 = pPar2 * pPar2 * d1 + pPar2 * coeff2 * d2 + coeff2 * coeff2 * d3;

            f = (param1 + param2) * s;

            return true;
        }

        // Inertia computation for XY, YZ and XZ.
        if (_valueType == InertiaXY || _valueType == InertiaYZ || _valueType == InertiaXZ)
        {
            if (_valueType == InertiaXY)
            {
                pPar1 = pmP0.X();
                pPar2 = pmP0.Y();
                coeff1 = coeffs[0];
                coeff2 = coeffs[1];
            }
            else if (_valueType == InertiaYZ)
            {
                pPar1 = pmP0.Y();
                pPar2 = pmP0.Z();
                coeff1 = coeffs[1];
                coeff2 = coeffs[2];
            }
            else
            { // myValueType == GProp_InertiaXZ
                pPar1 = pmP0.X();
                pPar2 = pmP0.Z();
                coeff1 = coeffs[0];
                coeff2 = coeffs[2];
            }

            d2 *= 0.5;
            pPar1 -= coeff1 * d1;
            pPar2 -= coeff2 * d1;
            param1 = pPar1 * pPar2 * d1 + (pPar1 * coeff2 + pPar2 * coeff1) * d2 + coeff1 * coeff2 * d3;

            f = -param1 * s;

            return true;
        }

        return false;
    }
}
