// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_CheckBSplineCurve.hxx
// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_CheckBSplineCurve.lxx
// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_CheckBSplineCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Library;

/// <summary>
/// Checks the end tangents of a B-spline curve: whether they are reversed with respect to the third or n-3rd pole,
/// and builds a curve with fixed tangents.
/// </summary>
internal sealed class CheckBSplineCurve
{
    private readonly BSplineCurve3D _curve;
    private bool _done;
    private readonly bool _fixFirstTangent;
    private readonly bool _fixLastTangent;
    private readonly double _angularTolerance;
    private readonly double _tolerance;

    private readonly int _indSecondPole;
    private readonly int _indPrelastPole;

    /// <summary>Checks the end tangents of Curve, with a distance and an angular tolerance.</summary>
    public CheckBSplineCurve(BSplineCurve3D curve, double tolerance, double angularTolerance)
    {
        _curve = curve;
        _done = false;
        _fixFirstTangent = false;
        _fixLastTangent = false;
        _angularTolerance = Math.Abs(angularTolerance);
        _tolerance = Math.Abs(tolerance);
        _indSecondPole = -1;
        _indPrelastPole = -1;

        int ii, num_poles;
        double tangent_magnitude, value, vector_magnitude;
        num_poles = _curve.PoleCount;

        if ((!_curve.IsPeriodic) && num_poles >= 4)
        {

            Vector3D tangent, tangent_normalized = new(), a_vector, avector_normalized;

            double crossProdSqTol = _angularTolerance * _angularTolerance;

            // Near first
            tangent = new Vector3D(_curve.Pole(1), _curve.Pole(2));
            tangent_magnitude = tangent.Magnitude;
            if (tangent_magnitude > _tolerance)
            {
                tangent_normalized = tangent / tangent_magnitude;
            }

            for (ii = 3; ii <= num_poles; ii++)
            {
                a_vector = new Vector3D(_curve.Pole(1), _curve.Pole(ii));
                vector_magnitude = a_vector.Magnitude;

                if (tangent_magnitude > _tolerance && vector_magnitude > _tolerance)
                {
                    avector_normalized = a_vector / vector_magnitude;

                    Vector3D crossProd = tangent_normalized ^ avector_normalized;
                    double crossProdSqLength = crossProd.SquareMagnitude;
                    if (crossProdSqLength > crossProdSqTol)
                    {
                        break;
                    }

                    value = tangent.Dot(a_vector);
                    if (value < 0.0e0)
                    {
                        _fixFirstTangent = true;
                        _indSecondPole = ii;
                        break;
                    }
                }
            }

            // Near last
            tangent = new Vector3D(_curve.Pole(num_poles), _curve.Pole(num_poles - 1));
            tangent_magnitude = tangent.Magnitude;
            if (tangent_magnitude > _tolerance)
            {
                tangent_normalized = tangent / tangent_magnitude;
            }

            for (ii = num_poles - 2; ii >= 1; ii--)
            {
                a_vector = new Vector3D(_curve.Pole(num_poles), _curve.Pole(ii));
                vector_magnitude = a_vector.Magnitude;

                if (tangent_magnitude > _tolerance && vector_magnitude > _tolerance)
                {
                    avector_normalized = a_vector / vector_magnitude;

                    Vector3D crossProd = tangent_normalized ^ avector_normalized;
                    double crossProdSqLength = crossProd.SquareMagnitude;
                    if (crossProdSqLength > crossProdSqTol)
                    {
                        break;
                    }

                    value = tangent.Dot(a_vector);
                    if (value < 0.0e0)
                    {
                        _fixLastTangent = true;
                        _indPrelastPole = ii;
                        break;
                    }
                }
            }
        } // if (( ! myCurve->IsPeriodic() )&& num_poles >= 4)
        else
        {
            _done = true;
        }
    }

    /// <summary>Whether the check is complete: the curve needs no fix, or a fix was made.</summary>
    public bool IsDone => _done;

    /// <summary>Whether the first and the last tangents need a fix.</summary>
    public void NeedTangentFix(out bool firstFlag, out bool lastFlag)
    {
        firstFlag = _fixFirstTangent;
        lastFlag = _fixLastTangent;
    }

    /// <summary>
    /// Fixes the tangents of the checked curve in place, as OCCT does. Internal: only ported code calls it, on a
    /// curve it has just built (a frozen curve throws Standard_ImmutableObject). Callers use <see cref="FixedTangent"/>.
    /// </summary>
    internal void FixTangent(bool firstFlag, bool lastFlag)
    {
        FixTangentOnCurve(_curve, firstFlag, lastFlag);
    }

    /// <summary>
    /// A copy of the curve with the first and/or last tangents fixed, or null when no requested end needs a fix.
    /// </summary>
    public BSplineCurve3D? FixedTangent(bool firstFlag, bool lastFlag)
    {
        BSplineCurve3D? new_curve = null;
        if ((_fixFirstTangent && firstFlag) || (_fixLastTangent && lastFlag))
        {
            new_curve = _curve.Copy() as BSplineCurve3D;

            FixTangentOnCurve(new_curve!, firstFlag, lastFlag);
        }
        return new_curve;
    }

    private void FixTangentOnCurve(BSplineCurve3D curve, bool firstFlag, bool lastFlag)
    {
        if (_fixFirstTangent && firstFlag)
        {
            Xyz xyz1 = curve.Pole(1).XYZ();
            Xyz xyz2 = curve.Pole(_indSecondPole).XYZ();
            double nbSamples = _indSecondPole - 1;
            for (int i = 2; i < _indSecondPole; i++)
            {
                double ii = i - 1;
                Point3D newPole = new((1.0 - ii / nbSamples) * xyz1 + ii / nbSamples * xyz2);
                curve.SetPole(i, newPole);
            }
        }

        if (_fixLastTangent && lastFlag)
        {
            int num_poles = curve.PoleCount;

            Xyz xyz1 = curve.Pole(num_poles).XYZ();
            Xyz xyz2 = curve.Pole(_indPrelastPole).XYZ();
            double nbSamples = num_poles - _indPrelastPole;
            for (int i = num_poles - 1; i > _indPrelastPole; i--)
            {
                double ii = num_poles - i;
                Point3D newPole = new((1.0 - ii / nbSamples) * xyz1 + ii / nbSamples * xyz2);
                curve.SetPole(i, newPole);
            }
        }

        _done = true;
    }
}
