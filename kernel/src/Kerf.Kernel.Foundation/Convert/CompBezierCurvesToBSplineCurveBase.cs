// Port of: src/FoundationClasses/TKMath/Convert/Convert_CompBezierCurvesToBSplineCurveBase.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

using Occt;

namespace Kerf.Engine.Geometry.Conversion;

/// <summary>
/// The operations <see cref="CompBezierCurvesToBSplineCurveBase{PointType, VecType, Traits}"/> needs on
/// its point and vector types, which OCCT's template resolves by overload at instantiation.
/// </summary>
/// <remarks>
/// Public only because the base class is public and generic over it (C# forbids a public class deriving from a
/// base built on an internal type); implemented by <see cref="CompBezierCurvesTraits3d"/> and
/// <see cref="CompBezierCurvesTraits2d"/>.
/// </remarks>
public interface ICompBezierCurvesTraits<PointType, VecType>
    where PointType : struct
    where VecType : struct
{
    /// <summary>True for gp_Pnt (OCCT's <c>std::is_same_v&lt;PointType, gp_Pnt&gt;</c>).</summary>
    static abstract bool Is3d { get; }

    /// <summary><c>VecType(theP1, theP2)</c>.</summary>
    static abstract VecType Vec(in PointType p1, in PointType p2);

    static abstract double SquareMagnitude(in VecType v);

    static abstract bool IsParallel(in VecType v1, in VecType v2, double angularTolerance);

    /// <summary><c>BSplCLib::IncreaseDegree(theNewDegree, thePoles, NoWeights(), theNewPoles, NoWeights())</c>.</summary>
    static abstract void IncreaseDegree(int newDegree, NCollection_Array1<PointType> poles,
                                        NCollection_Array1<PointType> newPoles);
}

/// <summary>The traits of <see cref="CompBezierCurvesToBSplineCurve"/>: gp_Pnt and gp_Vec.</summary>
public readonly struct CompBezierCurvesTraits3d : ICompBezierCurvesTraits<Point3D, Vector3D>
{
    public static bool Is3d => true;

    public static Vector3D Vec(in Point3D p1, in Point3D p2) => new(p1, p2);

    public static double SquareMagnitude(in Vector3D v) => v.SquareMagnitude();

    public static bool IsParallel(in Vector3D v1, in Vector3D v2, double angularTolerance)
        => v1.IsParallel(v2, angularTolerance);

    public static void IncreaseDegree(int newDegree, NCollection_Array1<Point3D> poles,
                                      NCollection_Array1<Point3D> newPoles)
        => global::Kerf.Engine.Geometry.Splines.BSplCLib.IncreaseDegree(newDegree, poles, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), newPoles, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights());
}

/// <summary>The traits of <see cref="CompBezierCurves2dToBSplineCurve2d"/>: gp_Pnt2d and gp_Vec2d.</summary>
public readonly struct CompBezierCurvesTraits2d : ICompBezierCurvesTraits<Point2D, Vector2D>
{
    public static bool Is3d => false;

    public static Vector2D Vec(in Point2D p1, in Point2D p2) => new(p1, p2);

    public static double SquareMagnitude(in Vector2D v) => v.SquareMagnitude();

    public static bool IsParallel(in Vector2D v1, in Vector2D v2, double angularTolerance)
        => v1.IsParallel(v2, angularTolerance);

    public static void IncreaseDegree(int newDegree, NCollection_Array1<Point2D> poles,
                                      NCollection_Array1<Point2D> newPoles)
        => global::Kerf.Engine.Geometry.Splines.BSplCLib.IncreaseDegree(newDegree, poles, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), newPoles, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights());
}

/// <summary>
/// Converts a sequence of adjacent non-rational Bezier curves into a B-spline curve: <see cref="AddCurve"/> each
/// curve, then <see cref="Perform"/>. Where the tangents at a junction are parallel, the B-spline is made at least
/// C1 there.
/// </summary>
/// <remarks>
/// OCCT's template on (PointType, VecType), ported generically with a traits type (as BSplCLib_CurveComputation).
/// A builder: <see cref="AddCurve"/> and <see cref="Perform"/> change it.
/// </remarks>
public class CompBezierCurvesToBSplineCurveBase<PointType, VecType, Traits>
    where PointType : struct
    where VecType : struct
    where Traits : struct, ICompBezierCurvesTraits<PointType, VecType>
{
    private readonly NCollection_Sequence<NCollection_Array1<PointType>> _sequence = new();
    private readonly NCollection_Sequence<PointType> _curvePoles = new();
    private readonly NCollection_Sequence<double> _curveKnots = new();
    private readonly NCollection_Sequence<int> _knotsMults = new();
    private int _degree;
    private readonly double _angular;

    /// <summary>
    /// A converter checking the parallelism of the tangents at the junctions with
    /// <paramref name="angularTolerance"/> (radians).
    /// </summary>
    public CompBezierCurvesToBSplineCurveBase(double angularTolerance = 1.0e-4)
    {
        _degree = 0;
        _angular = angularTolerance;
    }

    /// <summary>Adds the Bezier curve with the poles <paramref name="poles"/> (copied) to the sequence.</summary>
    public void AddCurve(NCollection_Array1<PointType> poles) { _sequence.Append(new NCollection_Array1<PointType>(poles)); }

    /// <summary>Computes the B-spline curve equivalent to the sequence of Bezier curves.</summary>
    public void Perform()
    {
        _curvePoles.Clear();
        _curveKnots.Clear();
        _knotsMults.Clear();
        if (_sequence.IsEmpty())
        {
            return;
        }
        const int lowerI = 1;
        int upperI = _sequence.Length();
        int nbrCurv = upperI - lowerI + 1;
        NCollection_Array1<double> curveKnVals = new(1, nbrCurv);

        _degree = 0;
        for (int i = 1; i <= _sequence.Length(); i++)
        {
            _degree = Max(_degree, _sequence[i].Length() - 1);
        }

        double det = 0;
        PointType p1 = new(), p2, p3;
        int maxDegree = _degree;
        NCollection_Array1<PointType> points = new(1, _degree + 1);

        for (int i = lowerI; i <= upperI; i++)
        {
            // 1- Raise the Bezier curve to the maximum degree.
            int deg = _sequence[i].Length() - 1;
            int inc = _degree - deg;
            if (inc > 0)
            {
                Traits.IncreaseDegree(_degree, _sequence[i], points);
            }
            else
            {
                points.Assign(_sequence[i]);
            }

            // 2- Process the node of junction between 2 Bezier curves.
            if (i == lowerI)
            {
                // Processing of the initial node of the BSpline.
                for (int j = 1; j <= maxDegree; j++)
                {
                    _curvePoles.Append(points[j]);
                }
                curveKnVals[1] = 1.0; // To begin the series.
                _knotsMults.Append(maxDegree + 1);
                det = 1.0;
            }

            if (i != lowerI)
            {
                p2 = points[1];
                p3 = points[2];
                VecType v1 = Traits.Vec(p1, p2), v2 = Traits.Vec(p2, p3);

                // Processing of the tangency between Bezier and the previous.
                // This allows to guarantee at least a C1 continuity if the tangents are coherent.
                double d1 = Traits.SquareMagnitude(v1);
                double d2 = Traits.SquareMagnitude(v2);
                if (maxDegree > 1 && d1 > Constants.Resolution() && d2 > Constants.Resolution()
                    && Traits.IsParallel(v1, v2, _angular))
                {
                    double lambda = Math.Sqrt(d2 / d1);
                    if (Traits.Is3d)
                    {
                        // 3D-specific epsilon guard to avoid numerical issues
                        // when accumulated knot values become too small relative to Det.
                        if (curveKnVals[i - 1] * lambda > 10.0 * Epsilon(det))
                        {
                            _knotsMults.Append(maxDegree - 1);
                            curveKnVals[i] = curveKnVals[i - 1] * lambda;
                        }
                        else
                        {
                            _curvePoles.Append(points[1]);
                            _knotsMults.Append(maxDegree);
                            curveKnVals[i] = 1.0;
                        }
                    }
                    else
                    {
                        _knotsMults.Append(maxDegree - 1);
                        curveKnVals[i] = curveKnVals[i - 1] * lambda;
                    }
                }
                else
                {
                    _curvePoles.Append(points[1]);
                    _knotsMults.Append(maxDegree);
                    curveKnVals[i] = 1.0;
                }
                det += curveKnVals[i];

                // Store the poles.
                for (int j = 2; j <= maxDegree; j++)
                {
                    _curvePoles.Append(points[j]);
                }
            }

            if (i == upperI)
            {
                // Processing of the end node of the BSpline.
                _curvePoles.Append(points[maxDegree + 1]);
                _knotsMults.Append(maxDegree + 1);
            }
            p1 = points[maxDegree];
        }

        // Correct nodal values to make them variable within [0.,1.].
        _curveKnots.Append(0.0);
        for (int i = 2; i <= nbrCurv; i++)
        {
            _curveKnots.Append(_curveKnots[i - 1] + (curveKnVals[i - 1] / det));
        }
        _curveKnots.Append(1.0);
    }

    /// <summary>The degree of the B-spline curve.</summary>
    public int Degree() { return _degree; }

    /// <summary>The number of poles of the B-spline curve.</summary>
    public int PoleCount {
        get
        { return _curvePoles.Length(); }
    }

    /// <summary>Copies the poles of the B-spline curve into <paramref name="poles"/>.</summary>
    public void Poles(NCollection_Array1<PointType> poles)
    {
        int k = 1;
        for (int i = poles.Lower(); i <= poles.Upper(); i++)
        {
            poles[i] = _curvePoles[k++];
        }
    }

    /// <summary>The number of knots of the B-spline curve.</summary>
    public int KnotCount {
        get
        { return _curveKnots.Length(); }
    }

    /// <summary>Copies the knots and their multiplicities into <paramref name="knots"/> and <paramref name="mults"/>.</summary>
    public void KnotsAndMults(NCollection_Array1<double> knots, NCollection_Array1<int> mults)
    {
        int k = 1;
        for (int i = knots.Lower(); i <= knots.Upper(); i++)
        {
            knots[i] = _curveKnots[k++];
        }
        k = 1;
        for (int i = mults.Lower(); i <= mults.Upper(); i++)
        {
            mults[i] = _knotsMults[k++];
        }
    }
}
