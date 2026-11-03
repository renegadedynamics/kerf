// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_Interpolate.hxx
// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_Interpolate.lxx
// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_Interpolate.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Occt.GeomLib_InterpolationErrors;

namespace Occt;

/// <summary>
/// Builds a B-spline curve interpolating points at given parameters. The curve has continuity degree - 1; its
/// degree is odd (an even degree gives a curve of degree - 1), and the derivatives from degree - 1 down to
/// (degree + 1) / 2 are null at both ends.
/// </summary>
public sealed class GeomLib_Interpolate
{
    private readonly Geom_BSplineCurve? myCurve;
    private readonly bool myIsDone;
    // OCCT leaves myError uninitialised on success; the port reports GeomLib_NoError.
    private readonly GeomLib_InterpolationErrors myError = GeomLib_NoError;

    /// <summary>
    /// Interpolates the first NumPoints points of PointsArray at the parameters ParametersArray (both indexed from
    /// 1), with the given degree (at least 3).
    /// </summary>
    public GeomLib_Interpolate(int Degree, int NumPoints, NCollection_Array1<Point3D> PointsArray,
                               NCollection_Array1<double> ParametersArray)
    {
        myIsDone = false;
        int ii, num_knots, inversion_problem, num_controls, jj;

        if (NumPoints < Degree || PointsArray.Lower() != 1 || PointsArray.Upper() < NumPoints
            || ParametersArray.Lower() != 1 || ParametersArray.Upper() < NumPoints)
        {
            myError = GeomLib_NotEnoughtPoints;
        }
        else if (Degree < 3)
        {
            myError = GeomLib_DegreeSmallerThan3;
        }
        else
        {
            Point3D null_point = new(0.0e0, 0.0e0, 0.0e0);
            int order = Degree + 1, half_order;
            if (order % 2 != 0)
            {
                order -= 1;
            }
            half_order = order / 2;
            num_knots = NumPoints + 2 * order - 2;
            num_controls = num_knots - order;
            NCollection_Array1<double> flat_knots = new(1, num_knots);
            NCollection_Array1<int> contacts = new(1, num_controls);
            NCollection_Array1<int> multiplicities = new(1, NumPoints);
            NCollection_Array1<double> parameters = new(1, num_controls);
            NCollection_Array1<Point3D> poles = new(1, num_controls);

            for (ii = 1; ii <= NumPoints; ii++)
            {
                multiplicities[ii] = 1;
            }
            multiplicities[1] = order;
            multiplicities[NumPoints] = order;
            for (ii = 1, jj = num_controls + 1; ii <= order; ii++, jj++)
            {

                flat_knots[ii] = ParametersArray[1];
                flat_knots[jj] = ParametersArray[NumPoints];
            }
            jj = order + 1;
            for (ii = 2; ii < NumPoints; ii++)
            {
                flat_knots[jj] = ParametersArray[ii];
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
                parameters[ii] = ParametersArray[1];
                poles[ii] = null_point;
                contacts[jj] = half_order + ii - 1;
                parameters[jj] = ParametersArray[NumPoints];
                poles[jj] = null_point;
                jj -= 1;
            }
            jj = half_order + 1;
            for (ii = 2; ii < NumPoints; ii++)
            {
                parameters[jj] = ParametersArray[ii];
                poles[jj] = PointsArray[ii];
                jj += 1;
            }
            contacts[1] = 0;
            contacts[num_controls] = 0;
            poles[1] = PointsArray[1];
            poles[num_controls] = PointsArray[NumPoints];
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Interpolate(order - 1, flat_knots, parameters, contacts, poles, out inversion_problem);

            if (inversion_problem == 0)
            {
                myCurve = new Geom_BSplineCurve(poles, ParametersArray, multiplicities, order - 1);
                myIsDone = true;
            }
            else
            {
                myError = GeomLib_InversionProblem;
            }
        }
    }

    /// <summary>Whether the interpolation succeeded.</summary>
    public bool IsDone() => myIsDone;

    /// <summary>What went wrong, if anything.</summary>
    public GeomLib_InterpolationErrors Error() => myError;

    /// <summary>The interpolating curve (frozen: the same object at each call), or null on failure.</summary>
    public Geom_BSplineCurve? Curve()
    {
        myCurve?.Freeze();
        return myCurve;
    }
}
