// Port of: src/ModelingData/TKGeomBase/Geom2dConvert/Geom2dConvert_CompCurveToBSplineCurve.hxx
// Port of: src/ModelingData/TKGeomBase/Geom2dConvert/Geom2dConvert_CompCurveToBSplineCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Conversion;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;

namespace Kerf.Engine.Geometry.Conversion;

/// <summary>
/// Converts bounded 2D curves to B-splines and concatenates them into one B-spline curve, reparametrised for C1
/// continuity where the tangents allow it. A working object: <c>Add</c> appends curves.
/// </summary>
internal sealed class CompCurveToBSplineCurve2D
{
    private BSplineCurve2D? _curve;
    private double _tol;
    private readonly ParameterisationType _type;

    /// <summary>An empty concatenation; Parameterisation is used to convert the conics added.</summary>
    public CompCurveToBSplineCurve2D(ParameterisationType parameterisation = TgtThetaOver2)
    {
        _tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        _type = parameterisation;
    }

    /// <summary>A concatenation starting with BasisCurve; Parameterisation is used to convert conics.</summary>
    public CompCurveToBSplineCurve2D(BoundedCurve2D basisCurve,
                                                 ParameterisationType parameterisation = TgtThetaOver2)
    {
        _tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        _type = parameterisation;
        if (basisCurve is BSplineCurve2D)
        {
            _curve = (BSplineCurve2D)basisCurve.Copy();
        }
        else
        {
            _curve = Geom2dConvert2D.CurveToBSplineCurve(basisCurve, _type);
        }
    }

    /// <summary>
    /// Appends NewCurve at either end of the B-spline (reversing it if needed). Returns false if neither of its ends
    /// is within Tolerance of an end of the B-spline. Tolerance is also used to lower the multiplicity of the joint
    /// knot. If the result would be closed, After chooses the end: true appends after the last point.
    /// </summary>
    public bool Add(BoundedCurve2D newCurve, double tolerance, bool after = false)
    {
        // conversion
        BSplineCurve2D bs;
        if (newCurve is BSplineCurve2D)
        {
            bs = (BSplineCurve2D)newCurve.Copy();
        }
        else
        {
            bs = Geom2dConvert2D.CurveToBSplineCurve(newCurve, _type);
        }

        if (_curve is null)
        {
            _curve = bs;
            return true;
        }

        _tol = tolerance;
        double sqTol = tolerance * tolerance;

        int lBs = bs.PoleCount, lCb = _curve.PoleCount;
        double d1 = _curve.Pole(1).SquareDistanceTo(bs.Pole(1));
        double d2 = _curve.Pole(1).SquareDistanceTo(bs.Pole(lBs));

        bool isBeforeReversed = (_curve.Pole(1).SquareDistanceTo(bs.Pole(1)) < sqTol) && (d1 < d2);
        bool isBefore = (_curve.Pole(1).SquareDistanceTo(bs.Pole(lBs)) < sqTol) || isBeforeReversed;

        d1 = _curve.Pole(lCb).SquareDistanceTo(bs.Pole(1));
        d2 = _curve.Pole(lCb).SquareDistanceTo(bs.Pole(lBs));

        bool isAfterReversed = (_curve.Pole(lCb).SquareDistanceTo(bs.Pole(lBs)) < sqTol) && (d2 < d1);
        bool isAfter = (_curve.Pole(lCb).SquareDistanceTo(bs.Pole(1)) < sqTol) || isAfterReversed;

        // myCurve and NewCurve together form a closed curve
        if (isBefore && isAfter)
        {
            if (after)
            {
                isBefore = false;
            }
            else
            {
                isAfter = false;
            }
        }

        // The private Add may raise the degree of the current curve in place: a curve handed out by
        // BSplineCurve() (frozen) is copied first, so it never changes.
        if ((isAfter || isBefore) && _curve.IsFrozen)
        {
            _curve = (BSplineCurve2D)_curve.Copy();
        }

        // OCCT passes the field by reference; the private Add only reads it, then assigns myCurve.
        BSplineCurve2D current = _curve;
        if (isAfter)
        {
            if (isAfterReversed)
            {
                bs.Reverse();
            }

            Add(ref current, ref bs, true);
            return true;
        }
        else if (isBefore)
        {
            if (isBeforeReversed)
            {
                bs.Reverse();
            }

            Add(ref bs, ref current, false);
            return true;
        }

        return false;
    }

    // Concat two BSplineCurves.
    private void Add(ref BSplineCurve2D firstCurve, ref BSplineCurve2D secondCurve, bool after)
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
        double l1, l2, U_de_raccord;
        int ii, jj;
        double ratio = 1, ratio1, ratio2, delta1, delta2;
        int nbP1 = firstCurve.PoleCount, nbP2 = secondCurve.PoleCount;
        int nbK1 = firstCurve.KnotCount, nbK2 = secondCurve.KnotCount;
        NCollection_Array1<double> noeuds = new(1, nbK1 + nbK2 - 1);
        NCollection_Array1<Point2D> poles = new(1, nbP1 + nbP2 - 1);
        NCollection_Array1<double> poids = new(1, nbP1 + nbP2 - 1);
        NCollection_Array1<int> mults = new(1, nbK1 + nbK2 - 1);

        // Reparameterization ratio (C1 if possible)
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

        if (after)
        {
            // Do not move the first curve
            ratio1 = 1;
            delta1 = 0;
            ratio2 = 1 / ratio;
            delta2 = ratio2 * secondCurve.Knot(1) - firstCurve.Knot(nbK1);
            U_de_raccord = firstCurve.LastParameter;
        }
        else
        {
            // Do not move the second curve
            ratio1 = ratio;
            delta1 = ratio1 * firstCurve.Knot(nbK1) - secondCurve.Knot(1);
            ratio2 = 1;
            delta2 = 0;
            U_de_raccord = secondCurve.FirstParameter;
        }

        // The Knots

        for (ii = 1; ii < nbK1; ii++)
        {
            noeuds[ii] = ratio1 * firstCurve.Knot(ii) - delta1;
            mults[ii] = firstCurve.Multiplicity(ii);
        }

        noeuds[nbK1] = U_de_raccord;
        mults[nbK1] = firstCurve.Degree;
        for (ii = 2, jj = nbK1 + 1; ii <= nbK2; ii++, jj++)
        {
            noeuds[jj] = ratio2 * secondCurve.Knot(ii) - delta2;
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
        BSplineCurve2D curve = new(poles, poids, noeuds, mults, deg);
        _curve = curve;

        // Optionally reduce multiplicity
        bool ok = true;
        int m = mults[nbK1];
        while ((m > 0) && ok)
        {
            m--;
            ok = curve.RemoveKnot(nbK1, m, _tol);
        }
    }

    /// <summary>The concatenated B-spline (frozen: the same object until the next Add), or null if empty.</summary>
    public BSplineCurve2D? BSplineCurve()
    {
        _curve?.Freeze();
        return _curve;
    }

    // The result without freezing it, for ported code that discards this object (Geom2dConvert's statics).
    internal BSplineCurve2D? InternalBSplineCurve() => _curve;

    /// <summary>Clears the result.</summary>
    public void Clear() => _curve = null;
}
