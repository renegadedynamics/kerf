// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_CompCurveToBSplineCurve.hxx
// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_CompCurveToBSplineCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Conversion;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;

namespace Kerf.Engine.Geometry.Conversion;

/// <summary>
/// Converts bounded 3D curves to B-splines and concatenates them into one B-spline curve, reparametrised for C1
/// continuity where the tangents allow it. A working object: <c>Add</c> appends curves.
/// </summary>
internal sealed class CompCurveToBSplineCurve
{
    private BSplineCurve3D? _curve;
    private double _tol;
    private readonly ParameterisationType _type;

    /// <summary>An empty concatenation; theParameterisation is used to convert the conics added.</summary>
    public CompCurveToBSplineCurve(ParameterisationType parameterisation = TgtThetaOver2)
    {
        _tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        _type = parameterisation;
    }

    /// <summary>A concatenation starting with BasisCurve; Parameterisation is used to convert conics.</summary>
    public CompCurveToBSplineCurve(BoundedCurve3D basisCurve,
                                               ParameterisationType parameterisation = TgtThetaOver2)
    {
        _tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        _type = parameterisation;
        if (basisCurve is BSplineCurve3D)
        {
            _curve = (BSplineCurve3D)basisCurve.Copy();
        }
        else
        {
            _curve = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(basisCurve, _type);
        }
    }

    /// <summary>
    /// Appends NewCurve at either end of the B-spline (reversing it if needed). Returns false if neither of its end
    /// points is within Tolerance of an end point of the B-spline. Tolerance is also used to lower the multiplicity
    /// of the joint knot, down to MinM (0: the knot can be removed). If the result would be closed, After chooses
    /// the end: true appends after the last point. WithRatio reparametrises the added curve for C1 continuity;
    /// false keeps its parametrisation, which can slow down algorithms such as CPnts_AbscissaPoint on the result.
    /// </summary>
    public bool Add(BoundedCurve3D newCurve, double tolerance, bool after = false, bool withRatio = true,
                    int minM = 0)
    {
        // conversion
        BSplineCurve3D bs;
        if (newCurve is BSplineCurve3D)
        {
            bs = (BSplineCurve3D)newCurve.Copy();
        }
        else
        {
            bs = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(newCurve, _type);
        }

        if (_curve is null)
        {
            _curve = bs;
            return true;
        }

        bool avant, apres;
        _tol = tolerance;

        // Use actual curve endpoints instead of poles for proper G0 continuity check.
        // G0 continuity (positional continuity) requires curves to share endpoints.
        // For non-clamped or periodic B-splines, first/last poles may not coincide with endpoints.
        Point3D curveStart = _curve.StartPoint;
        Point3D curveEnd = _curve.EndPoint;
        Point3D bsStart = bs.StartPoint;
        Point3D bsEnd = bs.EndPoint;

        avant = ((curveStart.DistanceTo(bsStart) < _tol) || (curveStart.DistanceTo(bsEnd) < _tol));
        apres = ((curveEnd.DistanceTo(bsStart) < _tol) || (curveEnd.DistanceTo(bsEnd) < _tol));

        // Will myCurve be (or become) closed?
        if (avant && apres)
        { // Resolve the ambiguity
            if (after)
            {
                avant = false;
            }
            else
            {
                apres = false;
            }
        }

        // The private Add may raise the degree of the current curve in place: a curve handed out by
        // BSplineCurve() (frozen) is copied first, so it never changes.
        if ((apres || avant) && _curve.IsFrozen)
        {
            _curve = (BSplineCurve3D)_curve.Copy();
        }

        // OCCT passes the field by reference; the private Add only changes its degree, then assigns myCurve.
        BSplineCurve3D current = _curve;

        // Append after?
        if (apres)
        {
            if (curveEnd.DistanceTo(bsEnd) < _tol)
            {
                bs.Reverse();
            }

            Add(ref current, ref bs, true, withRatio, minM);
            return true;
        }
        // Prepend before?
        else if (avant)
        {
            if (curveStart.DistanceTo(bsStart) < _tol)
            {
                bs.Reverse();
            }

            Add(ref bs, ref current, false, withRatio, minM);
            return true;
        }

        return false;
    }

    // Concat two BSplineCurves.
    private void Add(ref BSplineCurve3D firstCurve, ref BSplineCurve3D secondCurve, bool after, bool withRatio,
                     int minM)
    {
        // Harmonize the degrees.
        int deg = Max(firstCurve.Degree, secondCurve.Degree);
        if (firstCurve.Degree < deg)
        {
            firstCurve.IncreaseDegree(deg);
        }

        if (secondCurve.Degree < deg)
        {
            secondCurve.IncreaseDegree(deg);
        }

        // Declarations
        double l1, l2;
        int ii, jj;
        double ratio = 1, ratio1, ratio2, delta1, delta2;
        int nbP1 = firstCurve.PoleCount, nbP2 = secondCurve.PoleCount;
        int nbK1 = firstCurve.KnotCount, nbK2 = secondCurve.KnotCount;
        NCollection_Array1<double> noeuds = new(1, nbK1 + nbK2 - 1);
        NCollection_Array1<Point3D> poles = new(1, nbP1 + nbP2 - 1);
        NCollection_Array1<double> poids = new(1, nbP1 + nbP2 - 1);
        NCollection_Array1<int> mults = new(1, nbK1 + nbK2 - 1);

        // Reparameterization ratio (C1 if possible)
        if (withRatio)
        {
            l1 = firstCurve.DN(firstCurve.LastParameter, 1).Magnitude;
            l2 = secondCurve.DN(secondCurve.FirstParameter, 1).Magnitude;

            if ((l1 > global::Kerf.Engine.Foundation.Precision.Confusion()) && (l2 > global::Kerf.Engine.Foundation.Precision.Confusion()))
            {
                ratio = l1 / l2;
            }

            if ((ratio < global::Kerf.Engine.Foundation.Precision.Confusion()) || (ratio > 1 / global::Kerf.Engine.Foundation.Precision.Confusion()))
            {
                ratio = 1;
            }
        }

        if (after)
        {
            // Do not move the first curve
            ratio1 = 1;
            delta1 = 0;
            ratio2 = 1 / ratio;
            delta2 = ratio2 * secondCurve.Knot(1) - firstCurve.Knot(nbK1);
        }
        else
        {
            // Do not move the second curve
            ratio1 = ratio;
            delta1 = ratio1 * firstCurve.Knot(nbK1) - secondCurve.Knot(1);
            ratio2 = 1;
            delta2 = 0;
        }

        // The Knots
        double eps;
        for (ii = 1; ii <= nbK1; ii++)
        {
            noeuds[ii] = ratio1 * firstCurve.Knot(ii) - delta1;
            if (ii > 1)
            {
                eps = Epsilon(Math.Abs(noeuds[ii - 1]));
                if (eps < 5.0e-10)
                {
                    eps = 5.0e-10;
                }

                if (noeuds[ii] - noeuds[ii - 1] <= eps)
                {
                    noeuds[ii] += eps;
                }
            }

            mults[ii] = firstCurve.Multiplicity(ii);
        }

        mults[nbK1] = firstCurve.Degree;
        for (ii = 2, jj = nbK1 + 1; ii <= nbK2; ii++, jj++)
        {
            noeuds[jj] = ratio2 * secondCurve.Knot(ii) - delta2;
            eps = Epsilon(Math.Abs(noeuds[jj - 1]));
            if (eps < 5.0e-10)
            {
                eps = 5.0e-10;
            }

            if (noeuds[jj] - noeuds[jj - 1] <= eps)
            {
                noeuds[jj] += eps;
            }

            mults[jj] = secondCurve.Multiplicity(ii);
        }

        ratio = firstCurve.Weight(nbP1);
        ratio /= secondCurve.Weight(1);
        // The Poles and Weights
        for (ii = 1; ii < nbP1; ii++)
        {
            poles[ii] = firstCurve.Pole(ii);
            poids[ii] = firstCurve.Weight(ii);
        }

        for (ii = 1, jj = nbP1; ii <= nbP2; ii++, jj++)
        {
            poles[jj] = secondCurve.Pole(ii);
            //
            // Note: the weights may not necessarily connect with C0 continuity,
            // hence the use of Ratio
            //
            poids[jj] = ratio * secondCurve.Weight(ii);
        }

        // Create the BSpline
        BSplineCurve3D curve = new(poles, poids, noeuds, mults, deg);
        _curve = curve;

        // Optionally reduce multiplicity down to MinM
        bool ok = true;
        int m = mults[nbK1];
        while ((m > minM) && ok)
        {
            m--;
            ok = curve.RemoveKnot(nbK1, m, _tol);
        }
    }

    /// <summary>The concatenated B-spline (frozen: the same object until the next Add), or null if empty.</summary>
    public BSplineCurve3D? BSplineCurve()
    {
        _curve?.Freeze();
        return _curve;
    }

    // The result without freezing it, for ported code that discards this object (GeomConvert's statics).
    internal BSplineCurve3D? InternalBSplineCurve() => _curve;

    /// <summary>Clears the result.</summary>
    public void Clear() => _curve = null;
}
