// Port of: src/ModelingData/TKGeomBase/GeomLProp/GeomLProp_CurAndInf2d.hxx
// Port of: src/ModelingData/TKGeomBase/GeomLProp/GeomLProp_CurAndInf2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// The extrema of curvature and the inflection points of a 2D curve: analytically for conics, numerically for
/// other curves (B-splines span by span of C3 continuity).
/// </summary>
/// <remarks>
/// A working object, as in OCCT. Each Perform method clears the previous results. As in OCCT, <see cref="Perform"/>
/// reports the status of the inflection search for B-spline curves, which can hide a failure of the curvature search.
/// </remarks>
public class GeomLProp_CurAndInf2d : LProp_CurAndInf
{
    private bool myIsDone;

    /// <summary>An empty framework: a Perform method gives the curve.</summary>
    public GeomLProp_CurAndInf2d()
    {
        myIsDone = false;
    }

    /// <summary>Computes the extrema of curvature and the inflection points of the curve.</summary>
    public void Perform(Geom2d_Curve C)
    {
        Clear();
        myIsDone = true;
        performCurExt(C);
        performInf(C);
    }

    /// <summary>Computes the extrema of curvature of the curve.</summary>
    public void PerformCurExt(Geom2d_Curve C)
    {
        Clear();
        myIsDone = true;
        performCurExt(C);
    }

    private void performCurExt(Geom2d_Curve theCurve)
    {
        var anAdaptor = new Geom2dAdaptor_Curve(theCurve);
        var aNumericInf = new GeomLProp_NumericCurInf2d();
        GeomAbs_CurveType aCurveType = anAdaptor.GetType();

        switch (aCurveType)
        {
            case GeomAbs_Line:
                break;
            case GeomAbs_Circle:
                break;
            case GeomAbs_Ellipse:
            {
                double aUFirst = anAdaptor.FirstParameter();
                double aULast = anAdaptor.LastParameter();
                double aUFPlus2PI = aUFirst + 2.0 * Math.PI;
                ReadOnlySpan<double> aQuarterPts = [0.0, Math.PI / 2.0, Math.PI, 3.0 * Math.PI / 2.0];
                ReadOnlySpan<bool> aIsMin = [true, false, true, false];
                for (int i = 0; i < 4; ++i)
                {
                    double aU = ElCLib.InPeriod(aQuarterPts[i], aUFirst, aUFPlus2PI);
                    if (aUFirst <= aU && aU <= aULast)
                    {
                        AddExtCur(aU, aIsMin[i]);
                    }
                }

                break;
            }
            case GeomAbs_Hyperbola:
            {
                double aUFirst = anAdaptor.FirstParameter();
                double aULast = anAdaptor.LastParameter();
                if (aUFirst <= 0.0 && aULast >= 0.0)
                {
                    AddExtCur(0.0, true);
                }

                break;
            }
            case GeomAbs_Parabola:
            {
                double aUFirst = anAdaptor.FirstParameter();
                double aULast = anAdaptor.LastParameter();
                if (aUFirst <= 0.0 && aULast >= 0.0)
                {
                    AddExtCur(0.0, true);
                }

                break;
            }
            case GeomAbs_BSplineCurve:
                if (anAdaptor.Continuity() >= GeomAbs_C3)
                {
                    aNumericInf.PerformCurExt(theCurve, this);
                    myIsDone = aNumericInf.IsDone();
                }
                else
                {
                    myIsDone = true;
                    int aNbIntervals = anAdaptor.NbIntervals(GeomAbs_C3);
                    var aParams = new NCollection_Array1<double>(1, aNbIntervals + 1);
                    anAdaptor.Intervals(aParams, GeomAbs_C3);
                    for (int i = 1; i <= aNbIntervals; i++)
                    {
                        aNumericInf.PerformCurExt(theCurve, aParams[i], aParams[i + 1], this);
                        if (!aNumericInf.IsDone())
                        {
                            myIsDone = false;
                        }
                    }
                }

                break;

            default:
            {
                aNumericInf.PerformCurExt(theCurve, this);
                myIsDone = aNumericInf.IsDone();
            }
                break;
        }
    }

    /// <summary>
    /// Computes the inflection points of the curve. Afterwards <see cref="IsDone"/> tells whether the computation
    /// succeeded, and NbPoints, Parameter and Type give the points.
    /// </summary>
    public void PerformInf(Geom2d_Curve C)
    {
        Clear();
        myIsDone = true;
        performInf(C);
    }

    private void performInf(Geom2d_Curve theCurve)
    {
        var anAdaptor = new Geom2dAdaptor_Curve(theCurve);
        GeomAbs_CurveType aCurveType = anAdaptor.GetType();
        var aNumericInf = new GeomLProp_NumericCurInf2d();

        switch (aCurveType)
        {
            case GeomAbs_Line:
                break;
            case GeomAbs_Circle:
                break;
            case GeomAbs_Ellipse:
                break;
            case GeomAbs_Hyperbola:
                break;
            case GeomAbs_Parabola:
                break;
            case GeomAbs_BSplineCurve:
                if (anAdaptor.Continuity() >= GeomAbs_C3)
                {
                    aNumericInf.PerformInf(theCurve, this);
                    myIsDone = aNumericInf.IsDone();
                }
                else
                {
                    myIsDone = true;
                    int aNbIntervals = anAdaptor.NbIntervals(GeomAbs_C3);
                    var aParams = new NCollection_Array1<double>(1, aNbIntervals + 1);
                    anAdaptor.Intervals(aParams, GeomAbs_C3);

                    for (int i = 1; i <= aNbIntervals; i++)
                    {
                        aNumericInf.PerformInf(theCurve, aParams[i], aParams[i + 1], this);
                        if (!aNumericInf.IsDone())
                        {
                            myIsDone = false;
                        }
                    }
                }

                break;

            default:
            {
                aNumericInf.PerformInf(theCurve, this);
                myIsDone = aNumericInf.IsDone();
            }
                break;
        }
    }

    /// <summary>True if the solutions were found.</summary>
    public bool IsDone()
    {
        return myIsDone;
    }
}
