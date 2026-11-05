// Port of: src/ModelingData/TKGeomBase/GCPnts/GCPnts_UniformDeflection.hxx
// Port of: src/ModelingData/TKGeomBase/GCPnts/GCPnts_UniformDeflection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Measurement.Points;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Engine.Measurement.DeflectionType;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Measurement;

/// <summary>
/// Computes points on a C2 curve such that the deflection between the curve and the polygon through the points
/// stays below a given value. Slower than GCPnts_QuasiUniformDeflection, but gives fewer points. 2D curves give
/// points with Z = 0.
/// </summary>
/// <remarks>
/// A working object (decision 21): <c>Initialize</c> recomputes. The templates over the curve type are ported per
/// instantiation (decision 187).
/// </remarks>
internal sealed class UniformDeflection
{
    private bool _done;
    private double _deflection;
    private readonly NCollection_Sequence<double> _params = new();
    private readonly NCollection_Sequence<Point3D> _points = new();

    /// <summary>An empty algorithm; use <c>Initialize</c>.</summary>
    public UniformDeflection()
    {
        _done = false;
        _deflection = 0.0;
    }

    /// <summary>
    /// Points on the part theU1..theU2 of the curve with the deflection theDeflection. theWithControl checks the
    /// deflection of the result (slower), adding points where needed.
    /// </summary>
    public UniformDeflection(CurveAdaptorBase c, double deflection, double u1, double u2,
        bool withControl = true)
    {
        _done = false;
        _deflection = deflection;
        Initialize(c, deflection, u1, u2, withControl);
    }

    /// <summary>Points on the curve with the deflection theDeflection.</summary>
    public UniformDeflection(CurveAdaptorBase c, double deflection, bool withControl = true)
    {
        _done = false;
        _deflection = deflection;
        Initialize(c, deflection, withControl);
    }

    /// <summary>Points on the part theU1..theU2 of the curve with the deflection theDeflection.</summary>
    public UniformDeflection(Curve2DAdaptorBase c, double deflection, double u1, double u2,
        bool withControl = true)
    {
        _done = false;
        _deflection = deflection;
        Initialize(c, deflection, u1, u2, withControl);
    }

    /// <summary>Points on the curve with the deflection theDeflection.</summary>
    public UniformDeflection(Curve2DAdaptorBase c, double deflection, bool withControl = true)
    {
        _done = false;
        _deflection = deflection;
        Initialize(c, deflection, withControl);
    }

    /// <summary>Recomputes the points on the curve.</summary>
    public void Initialize(CurveAdaptorBase c, double deflection, bool withControl = true)
    {
        Initialize(c, deflection, c.FirstParameter(), c.LastParameter(), withControl);
    }

    /// <summary>Recomputes the points on the curve.</summary>
    public void Initialize(Curve2DAdaptorBase c, double deflection, bool withControl = true)
    {
        Initialize(c, deflection, c.FirstParameter(), c.LastParameter(), withControl);
    }

    /// <summary>
    /// Recomputes the points on the part theU1..theU2 of the curve (the bounds are swapped if theU1 &gt; theU2).
    /// </summary>
    public void Initialize(CurveAdaptorBase c, double deflection, double u1, double u2,
        bool withControl = true)
    {
        InitializeCore(c, deflection, u1, u2, withControl);
    }

    /// <summary>
    /// Recomputes the points on the part theU1..theU2 of the curve (the bounds are swapped if theU1 &gt; theU2).
    /// </summary>
    public void Initialize(Curve2DAdaptorBase c, double deflection, double u1, double u2,
        bool withControl = true)
    {
        InitializeCore(c, deflection, u1, u2, withControl);
    }

    /// <summary>True if the computation was successful.</summary>
    public bool IsDone => _done;

    /// <summary>The number of points computed.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public int PointCount
    {
        get
        {
            StdFail_NotDone_Raise_if(!_done, "GCPnts_UniformDeflection::NbPoints()");
            return _params.Length();
        }
    }

    /// <summary>The parameter of the point of index Index (1 to NbPoints).</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public double Parameter(int index)
    {
        StdFail_NotDone_Raise_if(!_done, "GCPnts_UniformDeflection::Parameter()");
        return _params[index];
    }

    /// <summary>The point of index theIndex (1 to NbPoints).</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public Point3D Value(int index)
    {
        // OCCT names the wrong class and method in the message (occt-issues.md).
        StdFail_NotDone_Raise_if(!_done, "GCPnts_UniformAbscissa::Parameter()");
        return _points.Value(index);
    }

    /// <summary>The deflection given to the algorithm.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public double Deflection()
    {
        StdFail_NotDone_Raise_if(!_done, "GCPnts_UniformDeflection::Deflection()");
        return _deflection;
    }

    // mask the return of a Adaptor2d_Curve2d as a gp_Pnt
    private static Point3D Value(CurveAdaptorBase c, double parameter)
    {
        return c.Value(parameter);
    }

    private static Point3D Value(Curve2DAdaptorBase c, double parameter)
    {
        Point2D a2dPoint = c.Value(parameter);
        return new Point3D(a2dPoint.X, a2dPoint.Y, 0.0);
    }

    // ---- Templates over the curve type (3D instantiation) ----

    // Control of the last points.
    private static void Controle(CurveAdaptorBase c,
                                 NCollection_Sequence<double> parameters,
                                 NCollection_Sequence<Point3D> points,
                                 double u2)
    {
        int nbPnts = points.Length();
        if (nbPnts > 2)
        {
            double ua = parameters[nbPnts - 2];
            double ub = parameters[nbPnts - 1];
            if (u2 - ub < 0.33 * (u2 - ua))
            {
                double uc = (u2 + ua) * 0.5;
                parameters[nbPnts - 1] = uc;
                points[nbPnts - 1] = Value(c, uc);
            }
        }
    }

    private static bool PerformLinear(CurveAdaptorBase c,
                                      NCollection_Sequence<double> parameters,
                                      NCollection_Sequence<Point3D> points,
                                      double u1,
                                      double u2)
    {
        parameters.Append(u1);
        Point3D point = Value(c, u1);
        points.Append(point);

        parameters.Append(u2);
        point = Value(c, u2);
        points.Append(point);
        return true;
    }

    private static bool PerformCircular(CurveAdaptorBase c,
                                        NCollection_Sequence<double> parameters,
                                        NCollection_Sequence<Point3D> points,
                                        double deflection,
                                        double u1,
                                        double u2)
    {
        Point3D point;
        double angle = Max(1.0 - (deflection / c.Circle().Radius), 0.0);
        angle = 2.0e0 * Math.Acos(angle);
        int nbPoints = (int)((u2 - u1) / angle);
        nbPoints += 2;
        angle = (u2 - u1) / (double)(nbPoints - 1);
        double u = u1;
        for (int i = 1; i <= nbPoints; ++i)
        {
            parameters.Append(u);
            point = Value(c, u);
            points.Append(point);
            u += angle;
        }
        return true;
    }

    private static DeflectionType GetDefType(CurveAdaptorBase c)
    {
        if (c.NbIntervals(C2) > 1)
        {
            return DefComposite;
        }

        switch (c.GetType())
        {
            case Line:
                return Linear;
            case Circle:
                return Circular;
            case BSplineCurve:
            {
                BSplineCurve3D bSpline = c.BSpline();
                return (bSpline.PoleCount == 2) ? Linear : Curved;
            }
            case BezierCurve:
            {
                BezierCurve3D bezier = c.Bezier();
                return (bezier.PoleCount == 2) ? Linear : Curved;
            }
            default:
            {
                return Curved;
            }
        }
    }

    private static bool PerformCurve(NCollection_Sequence<double> parameters,
                                     NCollection_Sequence<Point3D> points,
                                     CurveAdaptorBase c,
                                     double deflection,
                                     double u1,
                                     double u2,
                                     double epsilon,
                                     bool withControl)
    {
        CurveUniformDeflection iterator = new(c, deflection, u1, u2, epsilon, withControl);
        for (; iterator.More(); iterator.Next())
        {
            parameters.Append(iterator.Value());
            points.Append(iterator.Point());
        }
        return iterator.IsAllDone;
    }

    private static bool PerformComposite(NCollection_Sequence<double> parameters,
                                         NCollection_Sequence<Point3D> points,
                                         CurveAdaptorBase c,
                                         double deflection,
                                         double u1,
                                         double u2,
                                         double epsilon,
                                         bool withControl)
    {
        int nbIntervals = c.NbIntervals(C2);
        int pIndex;
        NCollection_Array1<double> ti = new(1, nbIntervals + 1);
        c.Intervals(ti, C2);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Hunt(ti, u1, out pIndex);

        // iterate by continuous segments
        double ua = u1;
        for (int index = pIndex; ;)
        {
            double ub = index + 1 <= ti.Upper() ? Min(u2, ti[index + 1]) : u2;
            if (!PerformCurve(parameters,
                              points,
                              c,
                              deflection,
                              ua,
                              ub,
                              epsilon,
                              withControl))
            {
                return false;
            }
            ++index;
            if (index > nbIntervals || u2 < ti[index])
            {
                return true;
            }

            // remove last point to avoid duplication
            parameters.Remove(parameters.Length());
            points.Remove(points.Length());

            ua = ub;
        }
    }

    private void InitializeCore(CurveAdaptorBase c, double deflection, double u1, double u2,
        bool withControl)
    {
        double epsilon = c.Resolution(global::Kerf.Engine.Foundation.Precision.Confusion());
        _deflection = deflection;
        _done = false;
        _params.Clear();
        _points.Clear();

        double aU1 = Min(u1, u2);
        double aU2 = Max(u1, u2);
        DeflectionType type = GetDefType(c);
        switch (type)
        {
            case Linear:
                _done = PerformLinear(c, _params, _points, aU1, aU2);
                break;
            case Circular:
                _done = PerformCircular(c, _params, _points, deflection, aU1, aU2);
                break;
            case Curved:
                _done = PerformCurve(_params, _points, c, deflection, aU1, aU2, epsilon, withControl);
                break;
            case DefComposite:
                _done = PerformComposite(_params,
                                          _points,
                                          c,
                                          deflection,
                                          aU1,
                                          aU2,
                                          epsilon,
                                          withControl);
                break;
        }

        // control of the last points
        Controle(c, _params, _points, aU2);
    }

    // ---- Templates over the curve type (2D instantiation) ----

    // Control of the last points.
    private static void Controle(Curve2DAdaptorBase c,
                                 NCollection_Sequence<double> parameters,
                                 NCollection_Sequence<Point3D> points,
                                 double u2)
    {
        int nbPnts = points.Length();
        if (nbPnts > 2)
        {
            double ua = parameters[nbPnts - 2];
            double ub = parameters[nbPnts - 1];
            if (u2 - ub < 0.33 * (u2 - ua))
            {
                double uc = (u2 + ua) * 0.5;
                parameters[nbPnts - 1] = uc;
                points[nbPnts - 1] = Value(c, uc);
            }
        }
    }

    private static bool PerformLinear(Curve2DAdaptorBase c,
                                      NCollection_Sequence<double> parameters,
                                      NCollection_Sequence<Point3D> points,
                                      double u1,
                                      double u2)
    {
        parameters.Append(u1);
        Point3D point = Value(c, u1);
        points.Append(point);

        parameters.Append(u2);
        point = Value(c, u2);
        points.Append(point);
        return true;
    }

    private static bool PerformCircular(Curve2DAdaptorBase c,
                                        NCollection_Sequence<double> parameters,
                                        NCollection_Sequence<Point3D> points,
                                        double deflection,
                                        double u1,
                                        double u2)
    {
        Point3D point;
        double angle = Max(1.0 - (deflection / c.Circle().Radius), 0.0);
        angle = 2.0e0 * Math.Acos(angle);
        int nbPoints = (int)((u2 - u1) / angle);
        nbPoints += 2;
        angle = (u2 - u1) / (double)(nbPoints - 1);
        double u = u1;
        for (int i = 1; i <= nbPoints; ++i)
        {
            parameters.Append(u);
            point = Value(c, u);
            points.Append(point);
            u += angle;
        }
        return true;
    }

    private static DeflectionType GetDefType(Curve2DAdaptorBase c)
    {
        if (c.NbIntervals(C2) > 1)
        {
            return DefComposite;
        }

        switch (c.GetType())
        {
            case Line:
                return Linear;
            case Circle:
                return Circular;
            case BSplineCurve:
            {
                BSplineCurve2D bSpline = c.BSpline();
                return (bSpline.PoleCount == 2) ? Linear : Curved;
            }
            case BezierCurve:
            {
                BezierCurve2D bezier = c.Bezier();
                return (bezier.PoleCount == 2) ? Linear : Curved;
            }
            default:
            {
                return Curved;
            }
        }
    }

    private static bool PerformCurve(NCollection_Sequence<double> parameters,
                                     NCollection_Sequence<Point3D> points,
                                     Curve2DAdaptorBase c,
                                     double deflection,
                                     double u1,
                                     double u2,
                                     double epsilon,
                                     bool withControl)
    {
        CurveUniformDeflection iterator = new(c, deflection, u1, u2, epsilon, withControl);
        for (; iterator.More(); iterator.Next())
        {
            parameters.Append(iterator.Value());
            points.Append(iterator.Point());
        }
        return iterator.IsAllDone;
    }

    private static bool PerformComposite(NCollection_Sequence<double> parameters,
                                         NCollection_Sequence<Point3D> points,
                                         Curve2DAdaptorBase c,
                                         double deflection,
                                         double u1,
                                         double u2,
                                         double epsilon,
                                         bool withControl)
    {
        int nbIntervals = c.NbIntervals(C2);
        int pIndex;
        NCollection_Array1<double> ti = new(1, nbIntervals + 1);
        c.Intervals(ti, C2);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Hunt(ti, u1, out pIndex);

        // iterate by continuous segments
        double ua = u1;
        for (int index = pIndex; ;)
        {
            double ub = index + 1 <= ti.Upper() ? Min(u2, ti[index + 1]) : u2;
            if (!PerformCurve(parameters,
                              points,
                              c,
                              deflection,
                              ua,
                              ub,
                              epsilon,
                              withControl))
            {
                return false;
            }
            ++index;
            if (index > nbIntervals || u2 < ti[index])
            {
                return true;
            }

            // remove last point to avoid duplication
            parameters.Remove(parameters.Length());
            points.Remove(points.Length());

            ua = ub;
        }
    }

    private void InitializeCore(Curve2DAdaptorBase c, double deflection, double u1, double u2,
        bool withControl)
    {
        double epsilon = c.Resolution(global::Kerf.Engine.Foundation.Precision.Confusion());
        _deflection = deflection;
        _done = false;
        _params.Clear();
        _points.Clear();

        double aU1 = Min(u1, u2);
        double aU2 = Max(u1, u2);
        DeflectionType type = GetDefType(c);
        switch (type)
        {
            case Linear:
                _done = PerformLinear(c, _params, _points, aU1, aU2);
                break;
            case Circular:
                _done = PerformCircular(c, _params, _points, deflection, aU1, aU2);
                break;
            case Curved:
                _done = PerformCurve(_params, _points, c, deflection, aU1, aU2, epsilon, withControl);
                break;
            case DefComposite:
                _done = PerformComposite(_params,
                                          _points,
                                          c,
                                          deflection,
                                          aU1,
                                          aU2,
                                          epsilon,
                                          withControl);
                break;
        }

        // control of the last points
        Controle(c, _params, _points, aU2);
    }
}
