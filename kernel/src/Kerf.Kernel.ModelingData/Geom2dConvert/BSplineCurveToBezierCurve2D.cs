// Port of: src/ModelingData/TKGeomBase/Geom2dConvert/Geom2dConvert_BSplineCurveToBezierCurve.hxx
// Port of: src/ModelingData/TKGeomBase/Geom2dConvert/Geom2dConvert_BSplineCurveToBezierCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Kerf.Engine.Geometry.Conversion;

/// <summary>
/// Converts a 2D B-spline curve into adjacent Bezier arcs, one per knot span (W. Boehm, "Generating the Bezier points
/// of B-spline curves and surfaces", CAD 13(6), 1981). The constructor prepares a copy of the curve; Arc(i) builds
/// each arc.
/// </summary>
internal sealed class BSplineCurveToBezierCurve2D
{
    private readonly BSplineCurve2D _curve;

    /// <summary>Prepares the conversion of the whole of BasisCurve.</summary>
    public BSplineCurveToBezierCurve2D(BSplineCurve2D basisCurve)
    {
        _curve = (BSplineCurve2D)basisCurve.Copy();
        // periodic curve can't be converted correctly by two main reasons:
        // last pole (equal to first one) is missing;
        // poles recomputation using default boor scheme is fails.
        if (_curve.IsPeriodic)
        {
            _curve.SetNotPeriodic();
        }

        double uf = _curve.FirstParameter;
        double ul = _curve.LastParameter;
        _curve.Segment(uf, ul);
        _curve.IncreaseMultiplicity(_curve.FirstUKnotIndex(), _curve.LastUKnotIndex(), _curve.Degree);
    }

    /// <summary>
    /// Prepares the conversion of BasisCurve between U1 and U2. A bound within ParametricTolerance / 2 of a knot,
    /// beyond it, is moved onto the knot.
    /// </summary>
    /// <exception cref="DomainErrorException">U2 - U1 is below ParametricTolerance.</exception>
    public BSplineCurveToBezierCurve2D(BSplineCurve2D basisCurve, double u1, double u2,
                                                   double parametricTolerance)
    {
        if (u2 - u1 < parametricTolerance)
        {
            throw new DomainErrorException("GeomConvert_BSplineCurveToBezierSurface");
        }

        double uf = u1, ul = u2;
        double pTol = parametricTolerance / 2;

        int i1, i2;
        _curve = (BSplineCurve2D)basisCurve.Copy();
        if (_curve.IsPeriodic)
        {
            _curve.SetNotPeriodic();
        }

        _curve.LocateU(u1, pTol, out i1, out i2);
        if (i1 == i2)
        { // We are on the knot
            if (_curve.Knot(i1) > u1)
            {
                uf = _curve.Knot(i1);
            }
        }

        _curve.LocateU(u2, pTol, out i1, out i2);
        if (i1 == i2)
        { // We are on the knot
            if (_curve.Knot(i1) < u2)
            {
                ul = _curve.Knot(i1);
            }
        }

        _curve.Segment(uf, ul);
        _curve.IncreaseMultiplicity(_curve.FirstUKnotIndex(), _curve.LastUKnotIndex(), _curve.Degree);
    }

    /// <summary>A new Bezier curve for arc Index (1 to NbArcs()), oriented as the B-spline.</summary>
    /// <exception cref="OutOfRangeException">Index is out of range.</exception>
    public BezierCurve2D Arc(int index)
    {
        if (index < 1 || index > _curve.KnotCount - 1)
        {
            throw new OutOfRangeException("Geom2dConvert_BSplineCurveToBezierCurve");
        }

        int deg = _curve.Degree;

        NCollection_Array1<Point2D> poles = new(1, deg + 1);

        BezierCurve2D c;
        if (_curve.IsRational)
        {
            NCollection_Array1<double> weights = new(1, deg + 1);
            for (int i = 1; i <= deg + 1; i++)
            {
                poles[i] = _curve.Pole(i + deg * (index - 1));
                weights[i] = _curve.Weight(i + deg * (index - 1));
            }

            c = new BezierCurve2D(poles, weights);
        }
        else
        {
            for (int i = 1; i <= deg + 1; i++)
            {
                poles[i] = _curve.Pole(i + deg * (index - 1));
            }

            c = new BezierCurve2D(poles);
        }

        return c;
    }

    /// <summary>
    /// Fills Curves, indexed from 1 to NbArcs() (OCCT documents a Standard_DimensionError for other bounds but
    /// doesn't check them), with new Bezier curves for all the arcs.
    /// </summary>
    public void Arcs(NCollection_Array1<BezierCurve2D> curves)
    {
        int n = ArcCount;
        for (int i = 1; i <= n; i++)
        {
            curves[i] = Arc(i);
        }
    }

    /// <summary>Copies the knots bounding the arcs (NbArcs() + 1 values) into TKnots from its lower bound.</summary>
    public void Knots(NCollection_Array1<double> tKnots)
    {
        int ii, kk;
        for (ii = 1, kk = tKnots.Lower(); ii <= _curve.KnotCount; ii++, kk++)
        {
            tKnots[kk] = _curve.Knot(ii);
        }
    }

    /// <summary>The number of Bezier arcs.</summary>
    public int ArcCount => (_curve.KnotCount - 1);
}
