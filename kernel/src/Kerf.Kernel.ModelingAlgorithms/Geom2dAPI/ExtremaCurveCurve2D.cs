// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dAPI/Geom2dAPI_ExtremaCurveCurve.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dAPI/Geom2dAPI_ExtremaCurveCurve.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dAPI/Geom2dAPI_ExtremaCurveCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Api;

/// <summary>
/// All the extrema of the distance between two 2D curves: the pairs of points, one on each curve, at the ends of
/// the common perpendiculars.
/// </summary>
/// <remarks>
/// An algorithm object (decision 117). The nearest points of two curves may instead be an end of a curve and its
/// projection, two ends, or an intersection point. OCCT copies the Extrema_ExtCC2d it builds into its field; the
/// port keeps the object it builds (decision 224). OCCT's implicit conversions to int and double are explicit
/// operators (decision 327).
/// </remarks>
internal class ExtremaCurveCurve2D
{
    private readonly bool _isDone;
    private readonly int _index;
    private readonly ExtCC2d _extCC;
    private readonly CurveAdaptor2D _c1 = new();
    private readonly CurveAdaptor2D _c2 = new();

    /// <summary>
    /// The extrema between the part of <paramref name="c1"/> between <paramref name="u1min"/> and
    /// <paramref name="u1max"/> and the part of <paramref name="c2"/> between <paramref name="u2min"/> and
    /// <paramref name="u2max"/>.
    /// </summary>
    public ExtremaCurveCurve2D(Curve2D c1, Curve2D c2, double u1min, double u1max, double u2min,
                                       double u2max)
    {
        _c1.Load(c1, u1min, u1max);
        _c2.Load(c2, u2min, u2max);
        ExtCC2d theExtCC = new(_c1, _c2);
        _extCC = theExtCC;
        _isDone = _extCC.IsDone && (_extCC.ExtCount > 0);

        if (_isDone)
        {
            // evaluate the lower distance and its index;
            double dist2, dist2Min = _extCC.SquareDistance(1);
            _index = 1;

            for (int i = 2; i <= _extCC.ExtCount; i++)
            {
                dist2 = _extCC.SquareDistance(i);
                if (dist2 < dist2Min)
                {
                    dist2Min = dist2;
                    _index = i;
                }
            }
        }
    }

    /// <summary>The number of extrema (0 if the computation failed).</summary>
    public int NbExtrema()
    {
        if (_isDone)
        {
            return _extCC.ExtCount;
        }
        else
        {
            return 0;
        }
    }

    /// <summary>The points on the first and the second curve of the extremum of index <paramref name="index"/>.</summary>
    /// <exception cref="OutOfRangeException"><paramref name="index"/> is not in [1, NbExtrema].</exception>
    public void Points(int index, out Point2D p1, out Point2D p2)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > NbExtrema(), "Geom2dAPI_ExtremaCurveCurve::Points");
        _extCC.Points(index, out POnCurv2d pc1, out POnCurv2d pc2);

        p1 = pc1.Value();
        p2 = pc2.Value();
    }

    /// <summary>The parameters on the first and the second curve of the extremum of index <paramref name="index"/>.</summary>
    /// <exception cref="OutOfRangeException"><paramref name="index"/> is not in [1, NbExtrema].</exception>
    public void Parameters(int index, out double u1, out double u2)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > NbExtrema(), "Geom2dAPI_ExtremaCurveCurve::Parameters");
        _extCC.Points(index, out POnCurv2d pc1, out POnCurv2d pc2);

        u1 = pc1.Parameter();
        u2 = pc2.Parameter();
    }

    /// <summary>The length of the extremum of index <paramref name="index"/>.</summary>
    /// <exception cref="OutOfRangeException"><paramref name="index"/> is not in [1, NbExtrema].</exception>
    public double Distance(int index)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > NbExtrema(), "Geom2dAPI_ExtremaCurveCurve:Distance");
        return Math.Sqrt(_extCC.SquareDistance(index));
    }

    /// <summary>The points of the shortest extremum.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public void NearestPoints(out Point2D p1, out Point2D p2)
    {
        StdFail_NotDone_Raise_if(!_isDone, "Geom2dAPI_ExtremaCurveCurve:NearestPoints");
        Points(_index, out p1, out p2);
    }

    /// <summary>The parameters of the shortest extremum.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public void LowerDistanceParameters(out double u1, out double u2)
    {
        StdFail_NotDone_Raise_if(!_isDone, "Geom2dAPI_ExtremaCurveCurve:LowerDistanceParameters");
        Parameters(_index, out u1, out u2);
    }

    /// <summary>The length of the shortest extremum.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public double LowerDistance()
    {
        StdFail_NotDone_Raise_if(!_isDone, "Geom2dAPI_ExtremaCurveCurve:LowerDistance");
        return Math.Sqrt(_extCC.SquareDistance(_index));
    }

    /// <summary>The Extrema algorithm used.</summary>
    public ExtCC2d Extrema() => _extCC;

    /// <summary>OCCT's <c>operator double()</c>: <see cref="LowerDistance"/>.</summary>
    public static explicit operator double(ExtremaCurveCurve2D extrema) => extrema.LowerDistance();

    /// <summary>OCCT's <c>operator int()</c>: the number of extrema of the Extrema algorithm.</summary>
    /// <exception cref="NotDoneException">The Extrema algorithm failed.</exception>
    public static explicit operator int(ExtremaCurveCurve2D extrema) => extrema._extCC.ExtCount;
}
