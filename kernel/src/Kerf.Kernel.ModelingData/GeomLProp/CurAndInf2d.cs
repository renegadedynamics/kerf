// Port of: src/ModelingData/TKGeomBase/GeomLProp/GeomLProp_CurAndInf2d.hxx
// Port of: src/ModelingData/TKGeomBase/GeomLProp/GeomLProp_CurAndInf2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;
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

                    for (int i = 1; i <= nbIntervals; i++)
                    {
                        numericInf.PerformInf(curve, aParams[i], aParams[i + 1], this);
                        if (!numericInf.IsDone)
                        {
                            _isDone = false;
                        }
                    }
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
}
