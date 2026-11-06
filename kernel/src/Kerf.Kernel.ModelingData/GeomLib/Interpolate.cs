// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_Interpolate.hxx
// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_Interpolate.lxx
// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_Interpolate.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Kerf.Engine.Geometry.Library.InterpolationErrors;

namespace Kerf.Engine.Geometry.Library;

/// <summary>
/// Builds a B-spline curve interpolating points at given parameters. The curve has continuity degree - 1; its
/// degree is odd (an even degree gives a curve of degree - 1), and the derivatives from degree - 1 down to
/// (degree + 1) / 2 are null at both ends.
/// </summary>
internal sealed class Interpolate
{
    private readonly BSplineCurve3D? _curve;
    private readonly bool _isDone;
    // OCCT leaves myError uninitialised on success; the port reports GeomLib_NoError.
    private readonly InterpolationErrors _error = NoError;

    /// <summary>
    /// Interpolates the first NumPoints points of PointsArray at the parameters ParametersArray (both indexed from
    /// 1), with the given degree (at least 3).
    /// </summary>
    public Interpolate(int degree, int numPoints, NCollection_Array1<Point3D> pointsArray,
                               NCollection_Array1<double> parametersArray)
    {
        _isDone = false;
        int ii, num_knots, inversion_problem, num_controls, jj;

        if (numPoints < degree || pointsArray.Lower() != 1 || pointsArray.Upper() < numPoints
            || parametersArray.Lower() != 1 || parametersArray.Upper() < numPoints)
        {
            _error = NotEnoughtPoints;
        }
        else if (degree < 3)
        {
            _error = DegreeSmallerThan3;
        }
        else
        {
            Point3D null_point = new(0.0e0, 0.0e0, 0.0e0);
            int order = degree + 1, half_order;
            if (order % 2 != 0)
            {
                order -= 1;
            }
            half_order = order / 2;
            num_knots = numPoints + 2 * order - 2;
            num_controls = num_knots - order;
            NCollection_Array1<double> flat_knots = new(1, num_knots);
            NCollection_Array1<int> contacts = new(1, num_controls);
            NCollection_Array1<int> multiplicities = new(1, numPoints);
            NCollection_Array1<double> parameters = new(1, num_controls);
            NCollection_Array1<Point3D> poles = new(1, num_controls);

            for (ii = 1; ii <= numPoints; ii++)
            {
                multiplicities[ii] = 1;
            }
            multiplicities[1] = order;
            multiplicities[numPoints] = order;
            for (ii = 1, jj = num_controls + 1; ii <= order; ii++, jj++)
            {

                flat_knots[ii] = parametersArray[1];
                flat_knots[jj] = parametersArray[numPoints];
            }
            jj = order + 1;
            for (ii = 2; ii < numPoints; ii++)
            {
                flat_knots[jj] = parametersArray[ii];
                jj += 1;
            }
            for (ii = 1; ii <= num_controls; ii++)
            {
                contacts[ii] = 0;
            }
            jj = num_controls;
            for (ii = 1; ii <= half_order; ii++)
            {
                contacts[ii] = half_order + ii - 1;
                parameters[ii] = parametersArray[1];
                poles[ii] = null_point;
                contacts[jj] = half_order + ii - 1;
                parameters[jj] = parametersArray[numPoints];
                poles[jj] = null_point;
                jj -= 1;
            }
            jj = half_order + 1;
            for (ii = 2; ii < numPoints; ii++)
            {
                parameters[jj] = parametersArray[ii];
                poles[jj] = pointsArray[ii];
                jj += 1;
            }
            contacts[1] = 0;
            contacts[num_controls] = 0;
            poles[1] = pointsArray[1];
            poles[num_controls] = pointsArray[numPoints];
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Interpolate(order - 1, flat_knots, parameters, contacts, poles, out inversion_problem);

            if (inversion_problem == 0)
            {
                _curve = new BSplineCurve3D(poles, parametersArray, multiplicities, order - 1);
                _isDone = true;
            }
            else
            {
                _error = InversionProblem;
            }
        }
    }

    /// <summary>Whether the interpolation succeeded.</summary>
    public bool IsDone => _isDone;

    /// <summary>What went wrong, if anything.</summary>
    public InterpolationErrors Error() => _error;

    /// <summary>The interpolating curve (frozen: the same object at each call), or null on failure.</summary>
    public BSplineCurve3D? Curve()
    {
        _curve?.Freeze();
        return _curve;
    }
}
