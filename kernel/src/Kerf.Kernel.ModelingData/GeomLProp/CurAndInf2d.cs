// Port of: src/ModelingData/TKGeomBase/GeomLProp/GeomLProp_CurAndInf2d.hxx
// Port of: src/ModelingData/TKGeomBase/GeomLProp/GeomLProp_CurAndInf2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Geometry.Properties;

/// <summary>
/// The extrema of curvature and the inflection points of a 2D curve: analytically for conics, numerically for
/// other curves (B-splines span by span of C3 continuity).
/// </summary>
/// <remarks>
/// A working object, as in OCCT. Each Perform method clears the previous results. As in OCCT, <see cref="Perform"/>
/// reports the status of the inflection search for B-spline curves, which can hide a failure of the curvature search.
/// </remarks>
internal class CurAndInf2d : CurAndInf
{
    private bool _isDone;

    /// <summary>An empty framework: a Perform method gives the curve.</summary>
    public CurAndInf2d()
    {
        _isDone = false;
    }

    /// <summary>Computes the extrema of curvature and the inflection points of the curve.</summary>
    public void Perform(Curve2D c)
    {
        Clear();
        _isDone = true;
        PerformCurExtCore(c);
        // OCCT: performInf sets myIsDone again, hiding a failure of performCurExt (occt-issues.md)
        PerformInfCore(c);
    }

    /// <summary>Computes the extrema of curvature of the curve.</summary>
    public void PerformCurExt(Curve2D c)
    {
        Clear();
        _isDone = true;
        PerformCurExtCore(c);
    }

    private void PerformCurExtCore(Curve2D curve)
    {
        var adaptor = new CurveAdaptor2D(curve);
        var numericInf = new NumericCurInf2d();
        CurveKind curveType = adaptor.GetType();

        switch (curveType)
        {
            case Line:
                break;
            case Circle:
                break;
            case Ellipse:
            {
                double uFirst = adaptor.FirstParameter();
                double uLast = adaptor.LastParameter();
                double ufPlus2PI = uFirst + 2.0 * Math.PI;
                ReadOnlySpan<double> quarterPts = [0.0, Math.PI / 2.0, Math.PI, 3.0 * Math.PI / 2.0];
                ReadOnlySpan<bool> isMin = [true, false, true, false];
                for (int i = 0; i < 4; ++i)
                {
                    double u = global::Kerf.Engine.Geometry.ElCLib.InPeriod(quarterPts[i], uFirst, ufPlus2PI);
                    if (uFirst <= u && u <= uLast)
                    {
                        AddExtCur(u, isMin[i]);
                    }
                }

                break;
            }
            case Hyperbola:
            {
                double uFirst = adaptor.FirstParameter();
                double uLast = adaptor.LastParameter();
                if (uFirst <= 0.0 && uLast >= 0.0)
                {
                    AddExtCur(0.0, true);
                }

                break;
            }
            case Parabola:
            {
                double uFirst = adaptor.FirstParameter();
                double uLast = adaptor.LastParameter();
                if (uFirst <= 0.0 && uLast >= 0.0)
                {
                    AddExtCur(0.0, true);
                }

                break;
            }
            case BSplineCurve:
                if (adaptor.Continuity() >= C3)
                {
                    numericInf.PerformCurExt(curve, this);
                    _isDone = numericInf.IsDone;
                }
                else
                {
                    _isDone = true;
                    int nbIntervals = adaptor.NbIntervals(C3);
                    var aParams = new NCollection_Array1<double>(1, nbIntervals + 1);
                    adaptor.Intervals(aParams, C3);
                    for (int i = 1; i <= nbIntervals; i++)
                    {
                        numericInf.PerformCurExt(curve, aParams[i], aParams[i + 1], this);
                        if (!numericInf.IsDone)
                        {
                            _isDone = false;
                        }
                    }
                }

                break;

            default:
            {
                numericInf.PerformCurExt(curve, this);
                _isDone = numericInf.IsDone;
            }
                break;
        }
    }

    /// <summary>
    /// Computes the inflection points of the curve. Afterwards <see cref="IsDone"/> tells whether the computation
    /// succeeded, and NbPoints, Parameter and Type give the points.
    /// </summary>
    public void PerformInf(Curve2D c)
    {
        Clear();
        _isDone = true;
        PerformInfCore(c);
    }

    private void PerformInfCore(Curve2D curve)
    {
        var adaptor = new CurveAdaptor2D(curve);
        CurveKind curveType = adaptor.GetType();
        var numericInf = new NumericCurInf2d();

        switch (curveType)
        {
            case Line:
                break;
            case Circle:
                break;
            case Ellipse:
                break;
            case Hyperbola:
                break;
            case Parabola:
                break;
            case BSplineCurve:
                if (adaptor.Continuity() >= C3)
                {
                    numericInf.PerformInf(curve, this);
                    _isDone = numericInf.IsDone;
                }
                else
                {
                    _isDone = true;
                    int nbIntervals = adaptor.NbIntervals(C3);
                    var aParams = new NCollection_Array1<double>(1, nbIntervals + 1);
                    adaptor.Intervals(aParams, C3);

                    var numericResult = new CurAndInf();
                    for (int i = 1; i <= nbIntervals; i++)
                    {
                        numericInf.PerformInf(curve, aParams[i], aParams[i + 1], numericResult);
                        if (!numericInf.IsDone)
                        {
                            _isDone = false;
                        }
                    }

                    AddInflectionsWithKnots(curve, aParams, numericResult);
                }

                break;

            default:
            {
                numericInf.PerformInf(curve, this);
                _isDone = numericInf.IsDone;
            }
                break;
        }
    }

    /// <summary>True if the solutions were found.</summary>
    public bool IsDone
    {
        get
        {
            return _isDone;
        }
    }

    /// <summary>
    /// Upstream 8f817f5da5 (decision 2423): a G1 join of two smooth intervals at which the curve changes concavity is an
    /// inflection, though the curvature is zero on neither side, so the numeric search of each interval can't see it.
    /// The joins' inflections and the intervals' are added in order, the join's exact parameter kept where one of the
    /// intervals' falls on it.
    /// </summary>
    private void AddInflectionsWithKnots(Curve2D curve, NCollection_Array1<double> parameters, CurAndInf numericResult)
    {
        var knotInflections = new List<double>();
        for (int i = parameters.Lower() + 1; !NativeInflections.IsNative && i < parameters.Upper(); i++)
        {
            double parameter = parameters[i];
            var left = new CurveAdaptor2D(curve, parameters[i - 1], parameter);
            var right = new CurveAdaptor2D(curve, parameter, parameters[i + 1]);
            left.D2(parameter, out Point2D _, out Vector2D leftD1, out Vector2D leftD2);
            right.D2(parameter, out Point2D _, out Vector2D rightD1, out Vector2D rightD2);
            if (leftD1.Magnitude <= Constants.Resolution() || rightD1.Magnitude <= Constants.Resolution()
                || leftD1.Dot(rightD1) <= 0.0
                || !leftD1.IsParallel(rightD1, global::Kerf.Engine.Foundation.Precision.Angular()))
            {
                continue;
            }

            double leftBend = leftD1.Crossed(leftD2);
            double rightBend = rightD1.Crossed(rightD2);
            if ((leftBend < 0.0 && rightBend > 0.0) || (leftBend > 0.0 && rightBend < 0.0))
            {
                knotInflections.Add(parameter);
            }
        }

        double pConfusion = global::Kerf.Engine.Foundation.Precision.PConfusion();
        int knotIndex = 0;
        for (int i = 1; i <= numericResult.PointCount; i++)
        {
            double parameter = numericResult.Parameter(i);
            while (knotIndex < knotInflections.Count && knotInflections[knotIndex] < parameter - pConfusion)
            {
                AddInflection(knotInflections[knotIndex++]);
            }

            if (knotIndex >= knotInflections.Count || Math.Abs(knotInflections[knotIndex] - parameter) > pConfusion)
            {
                AddInflection(parameter);
            }
        }

        while (knotIndex < knotInflections.Count)
        {
            AddInflection(knotInflections[knotIndex++]);
        }
    }
}
