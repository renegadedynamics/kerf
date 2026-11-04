// Port of: src/ModelingData/TKGeomBase/GCPnts/GCPnts_AbscissaPoint.hxx
// Port of: src/ModelingData/TKGeomBase/GCPnts/GCPnts_AbscissaPoint.cxx
// Port of: src/ModelingData/TKGeomBase/GCPnts/GCPnts_TCurveTypes.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Measurement.Points;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Measurement.AbscissaType;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Measurement;

/// <summary>
/// Computes the length of a curve, or a point on a curve at a given distance (abscissa) from another point.
/// </summary>
/// <remarks>
/// OCCT's function templates over the curve type are ported once per instantiation (Adaptor3d_Curve and
/// Adaptor2d_Curve2d), with identical bodies (decision 187). GCPnts_TCurveTypes, the traits that resolve the 2D
/// and 3D classes, is replaced by the types written out in each instantiation.
/// </remarks>
internal sealed class AbscissaPoint
{
    private readonly CurveAbscissaPoint _computer = new();

    /// <summary>The length of the curve.</summary>
    public static double Length(CurveAdaptorBase c)
    {
        return AbscissaPoint.Length(c, c.FirstParameter(), c.LastParameter());
    }

    /// <summary>The length of the curve.</summary>
    public static double Length(Curve2DAdaptorBase c)
    {
        return AbscissaPoint.Length(c, c.FirstParameter(), c.LastParameter());
    }

    /// <summary>The length of the curve with the given tolerance.</summary>
    public static double Length(CurveAdaptorBase c, double tol)
    {
        return AbscissaPoint.Length(c, c.FirstParameter(), c.LastParameter(), tol);
    }

    /// <summary>The length of the curve with the given tolerance.</summary>
    public static double Length(Curve2DAdaptorBase c, double tol)
    {
        return AbscissaPoint.Length(c, c.FirstParameter(), c.LastParameter(), tol);
    }

    /// <summary>The length of the part of the curve between the parameters theU1 and theU2.</summary>
    public static double Length(CurveAdaptorBase c, double u1, double u2)
    {
        return LengthCore(c, u1, u2, null);
    }

    /// <summary>The length of the part of the curve between the parameters theU1 and theU2.</summary>
    public static double Length(Curve2DAdaptorBase c, double u1, double u2)
    {
        return LengthCore(c, u1, u2, null);
    }

    /// <summary>The length of the part of the curve between theU1 and theU2 with the given tolerance.</summary>
    public static double Length(CurveAdaptorBase c, double u1, double u2, double tol)
    {
        return LengthCore(c, u1, u2, tol);
    }

    /// <summary>The length of the part of the curve between theU1 and theU2 with the given tolerance.</summary>
    public static double Length(Curve2DAdaptorBase c, double u1, double u2, double tol)
    {
        return LengthCore(c, u1, u2, tol);
    }

    /// <summary>An empty algorithm.</summary>
    public AbscissaPoint()
    {
    }

    /// <summary>The point at the distance theAbscissa from the point of parameter theU0.</summary>
    /// <exception cref="ConstructionErrorException">The curve's length is below Precision::Confusion().</exception>
    public AbscissaPoint(CurveAdaptorBase c, double abscissa, double u0)
    {
        ComputeCore(c, abscissa, u0);
    }

    /// <summary>The point at the distance theAbscissa from the point of parameter theU0.</summary>
    /// <exception cref="ConstructionErrorException">The curve's length is below Precision::Confusion().</exception>
    public AbscissaPoint(Curve2DAdaptorBase c, double abscissa, double u0)
    {
        ComputeCore(c, abscissa, u0);
    }

    /// <summary>The point at the distance theAbscissa from the point of parameter theU0, with the tolerance theTol.</summary>
    public AbscissaPoint(double tol, CurveAdaptorBase c, double abscissa, double u0)
    {
        AdvComputeCore(tol, c, abscissa, u0);
    }

    /// <summary>The point at the distance theAbscissa from the point of parameter theU0, with the tolerance theTol.</summary>
    public AbscissaPoint(double tol, Curve2DAdaptorBase c, double abscissa, double u0)
    {
        AdvComputeCore(tol, c, abscissa, u0);
    }

    /// <summary>The point at the distance theAbscissa from the point of parameter theU0, starting the search at theUi.</summary>
    public AbscissaPoint(CurveAdaptorBase c, double abscissa, double u0, double ui)
    {
        double abscis = abscissa, uu0 = u0, uUi = ui;
        Compute(_computer, c, ref abscis, ref uu0, ref uUi, c.Resolution(global::Kerf.Engine.Foundation.Precision.Confusion()));
    }

    /// <summary>The point at the distance theAbscissa from the point of parameter theU0, starting the search at theUi.</summary>
    public AbscissaPoint(Curve2DAdaptorBase c, double abscissa, double u0, double ui)
    {
        double abscis = abscissa, uu0 = u0, uUi = ui;
        Compute(_computer, c, ref abscis, ref uu0, ref uUi, c.Resolution(global::Kerf.Engine.Foundation.Precision.Confusion()));
    }

    /// <summary>
    /// The point at the distance theAbscissa from the point of parameter theU0, starting the search at theUi, with the
    /// parametric tolerance theTol.
    /// </summary>
    public AbscissaPoint(CurveAdaptorBase c, double abscissa, double u0, double ui, double tol)
    {
        double abscis = abscissa, uu0 = u0, uUi = ui;
        AdvCompute(_computer, c, ref abscis, ref uu0, ref uUi, tol);
    }

    /// <summary>
    /// The point at the distance theAbscissa from the point of parameter theU0, starting the search at theUi, with the
    /// parametric tolerance theTol.
    /// </summary>
    public AbscissaPoint(Curve2DAdaptorBase c, double abscissa, double u0, double ui, double tol)
    {
        double abscis = abscissa, uu0 = u0, uUi = ui;
        AdvCompute(_computer, c, ref abscis, ref uu0, ref uUi, tol);
    }

    /// <summary>True if the computation was successful.</summary>
    public bool IsDone => _computer.IsDone;

    /// <summary>The parameter of the point found.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public double Parameter() => _computer.Parameter();

    // ---- Templates over the curve type (3D instantiation) ----

    // Dimension independent used to implement GCPnts_AbscissaPoint
    // compute the type and the length ratio if GCPnts_LengthParametrized.
    private static AbscissaType ComputeType(CurveAdaptorBase c, ref double ratio)
    {
        if (c.NbIntervals(CN) > 1)
        {
            return AbsComposite;
        }

        switch (c.GetType())
        {
            case Line:
            {
                ratio = 1.0;
                return LengthParametrized;
            }
            case Circle:
            {
                ratio = c.Circle().Radius;
                return LengthParametrized;
            }
            case BezierCurve:
            {
                BezierCurve3D bz = c.Bezier();
                if (bz.PoleCount == 2 && !bz.IsRational)
                {
                    ratio = bz.DN(0, 1).Magnitude;
                    return LengthParametrized;
                }
                return Parametrized;
            }
            case BSplineCurve:
            {
                BSplineCurve3D bs = c.BSpline();
                if (bs.PoleCount == 2 && !bs.IsRational)
                {
                    ratio = bs.DN(bs.FirstParameter, 1).Magnitude;
                    return LengthParametrized;
                }
                return Parametrized;
            }
            default:
            {
                return Parametrized;
            }
        }
    }

    // Compute a point at distance theAbscis from parameter theU0 using theUi as initial guess
    private static void Compute(CurveAbscissaPoint computer,
                                CurveAdaptorBase c,
                                ref double abscis,
                                ref double u0,
                                ref double ui,
                                double epsilon)
    {
        // test for easy solution
        if (Math.Abs(abscis) <= global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            computer.SetParameter(u0);
            return;
        }

        double ratio = 1.0;
        AbscissaType type = ComputeType(c, ref ratio);
        switch (type)
        {
            case LengthParametrized:
            {
                computer.SetParameter(u0 + abscis / ratio);
                return;
            }
            case Parametrized:
            {
                computer.Init(c);
                computer.Perform(abscis, u0, ui, epsilon);
                return;
            }
            case AbsComposite:
            {
                int nbIntervals = c.NbIntervals(CN);
                NCollection_Array1<double> ti = new(1, nbIntervals + 1);
                c.Intervals(ti, CN);
                double l = 0.0, sign = 1.0;
                int index;
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Hunt(ti, u0, out index);
                int direction = 1;
                if (abscis < 0)
                {
                    direction = 0;
                    abscis = -abscis;
                    sign = -1.0;
                }

                while (index >= 1 && index <= nbIntervals)
                {
                    l = CurveAbscissaPoint.Length(c, u0, ti[index + direction]);
                    if (Math.Abs(l - abscis) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        computer.SetParameter(ti[index + direction]);
                        return;
                    }

                    if (l > abscis)
                    {
                        if (ui < ti[index] || ui > ti[index + 1])
                        {
                            ui = (abscis / l) * (ti[index + 1] - u0);
                            if (direction != 0)
                            {
                                ui = u0 + ui;
                            }
                            else
                            {
                                ui = u0 - ui;
                            }
                        }
                        computer.Init(c, ti[index], ti[index + 1]);
                        computer.Perform(sign * abscis, u0, ui, epsilon);
                        return;
                    }
                    else
                    {
                        u0 = ti[index + direction];
                        abscis -= l;
                    }
                    if (direction != 0)
                    {
                        ++index;
                    }
                    else
                    {
                        --index;
                    }
                }

                // Push a little bit outside the limits (hairy !!!)
                ui = u0 + 0.1;
                computer.Init(c, u0, u0 + 0.2);
                computer.Perform(sign * abscis, u0, ui, epsilon);
                return;
            }
        }
    }

    // Introduced by rbv for curvilinear parametrization
    // performs more appropriate tolerance management.
    private static void AdvCompute(CurveAbscissaPoint computer,
                                   CurveAdaptorBase c,
                                   ref double abscis,
                                   ref double u0,
                                   ref double ui,
                                   double epsilon)
    {
        double ratio = 1.0;
        AbscissaType type = ComputeType(c, ref ratio);
        switch (type)
        {
            case LengthParametrized:
            {
                computer.SetParameter(u0 + abscis / ratio);
                return;
            }
            case Parametrized:
            {
                // theComputer.Init (theC);
                computer.Init(c, epsilon); // rbv's modification
                computer.AdvPerform(abscis, u0, ui, epsilon);
                return;
            }
            case AbsComposite:
            {
                int nbIntervals = c.NbIntervals(CN);
                NCollection_Array1<double> ti = new(1, nbIntervals + 1);
                c.Intervals(ti, CN);
                double l = 0.0, sign = 1.0;
                int index;
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Hunt(ti, u0, out index);

                int direction = 1;
                if (abscis < 0)
                {
                    direction = 0;
                    abscis = -abscis;
                    sign = -1.0;
                }

                if (index == 0 && direction > 0)
                {
                    l = CurveAbscissaPoint.Length(c, u0, ti[index + direction], epsilon);
                    if (Math.Abs(l - abscis) <= /*Precision::Confusion()*/ epsilon)
                    {
                        computer.SetParameter(ti[index + direction]);
                        return;
                    }

                    if (l > abscis)
                    {
                        if (ui > ti[index + 1])
                        {
                            ui = (abscis / l) * (ti[index + 1] - u0);
                            ui = u0 + ui;
                        }
                        computer.Init(c, u0, ti[index + 1], epsilon);
                        computer.AdvPerform(sign * abscis, u0, ui, epsilon);
                        return;
                    }
                    else
                    {
                        u0 = ti[index + direction];
                        abscis -= l;
                    }
                    ++index;
                }

                while (index >= 1 && index <= nbIntervals)
                {
                    l = CurveAbscissaPoint.Length(c, u0, ti[index + direction], epsilon);
                    if (Math.Abs(l - abscis) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        computer.SetParameter(ti[index + direction]);
                        return;
                    }

                    if (l > abscis)
                    {
                        if (ui < ti[index] || ui > ti[index + 1])
                        {
                            ui = (abscis / l) * (ti[index + 1] - u0);
                            if (direction != 0)
                            {
                                ui = u0 + ui;
                            }
                            else
                            {
                                ui = u0 - ui;
                            }
                        }
                        computer.Init(c, ti[index], ti[index + 1], epsilon);
                        computer.AdvPerform(sign * abscis, u0, ui, epsilon);
                        return;
                    }
                    else
                    {
                        u0 = ti[index + direction];
                        abscis -= l;
                    }
                    if (direction != 0)
                    {
                        ++index;
                    }
                    else
                    {
                        --index;
                    }
                }

                // Push a little bit outside the limits (hairy !!!)
                bool isNonPeriodic = !c.IsPeriodic;
                ui = u0 + sign * 0.1;
                double u1 = u0 + sign * 0.2;
                if (isNonPeriodic)
                {
                    if (sign > 0)
                    {
                        ui = Min(ui, c.LastParameter());
                        u1 = Min(u1, c.LastParameter());
                    }
                    else
                    {
                        ui = Max(ui, c.FirstParameter());
                        u1 = Max(u1, c.FirstParameter());
                    }
                }

                computer.Init(c, u0, u1, epsilon);
                computer.AdvPerform(sign * abscis, u0, ui, epsilon);
                return;
            }
        }
    }

    // theTol is null for no tolerance (OCCT's const double*).
    private static double LengthCore(CurveAdaptorBase c, double u1, double u2, double? tol)
    {
        double ratio = 1.0;
        AbscissaType type = ComputeType(c, ref ratio);
        switch (type)
        {
            case LengthParametrized:
            {
                return Math.Abs(u2 - u1) * ratio;
            }
            case Parametrized:
            {
                return tol != null ? CurveAbscissaPoint.Length(c, u1, u2, tol.Value)
                                      : CurveAbscissaPoint.Length(c, u1, u2);
            }
            case AbsComposite:
            {
                int nbIntervals = c.NbIntervals(CN);
                NCollection_Array1<double> ti = new(1, nbIntervals + 1);
                c.Intervals(ti, CN);
                double uu1 = Min(u1, u2);
                double uu2 = Max(u1, u2);
                double l = 0.0;
                for (int index = 1; index <= nbIntervals; ++index)
                {
                    if (ti[index] > uu2)
                    {
                        break;
                    }
                    if (ti[index + 1] < uu1)
                    {
                        continue;
                    }
                    if (tol != null)
                    {
                        l += CurveAbscissaPoint.Length(c,
                                                         Max(ti[index], uu1),
                                                         Min(ti[index + 1], uu2),
                                                         tol.Value);
                    }
                    else
                    {
                        l += CurveAbscissaPoint.Length(c,
                                                         Max(ti[index], uu1),
                                                         Min(ti[index + 1], uu2));
                    }
                }
                return l;
            }
        }
        return RealLast();
    }

    private void ComputeCore(CurveAdaptorBase c, double abscissa, double u0)
    {
        double l = AbscissaPoint.Length(c);
        if (l < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            throw new ConstructionErrorException();
        }

        double abscis = abscissa;
        double uu0 = u0;
        double uUi = u0 + (abscis / l) * (c.LastParameter() - c.FirstParameter());
        Compute(_computer, c, ref abscis, ref uu0, ref uUi, c.Resolution(global::Kerf.Engine.Foundation.Precision.Confusion()));
    }

    private void AdvComputeCore(double tol, CurveAdaptorBase c, double abscissa, double u0)
    {
        double l = AbscissaPoint.Length(c, tol);
        /*if (aL < Precision::Confusion())
        {
          throw Standard_ConstructionError ("GCPnts_AbscissaPoint::GCPnts_AbscissaPoint");
        }*/
        double abscis = abscissa;
        double uu0 = u0;
        double uUi = 0.0;
        if (l >= global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            uUi = u0 + (abscis / l) * (c.LastParameter() - c.FirstParameter());
        }
        else
        {
            uUi = u0;
        }
        AdvCompute(_computer, c, ref abscis, ref uu0, ref uUi, tol);
    }

    // ---- Templates over the curve type (2D instantiation) ----

    // Dimension independent used to implement GCPnts_AbscissaPoint
    // compute the type and the length ratio if GCPnts_LengthParametrized.
    private static AbscissaType ComputeType(Curve2DAdaptorBase c, ref double ratio)
    {
        if (c.NbIntervals(CN) > 1)
        {
            return AbsComposite;
        }

        switch (c.GetType())
        {
            case Line:
            {
                ratio = 1.0;
                return LengthParametrized;
            }
            case Circle:
            {
                ratio = c.Circle().Radius;
                return LengthParametrized;
            }
            case BezierCurve:
            {
                BezierCurve2D bz = c.Bezier();
                if (bz.PoleCount == 2 && !bz.IsRational)
                {
                    ratio = bz.DN(0, 1).Magnitude;
                    return LengthParametrized;
                }
                return Parametrized;
            }
            case BSplineCurve:
            {
                BSplineCurve2D bs = c.BSpline();
                if (bs.PoleCount == 2 && !bs.IsRational)
                {
                    ratio = bs.DN(bs.FirstParameter, 1).Magnitude;
                    return LengthParametrized;
                }
                return Parametrized;
            }
            default:
            {
                return Parametrized;
            }
        }
    }

    // Compute a point at distance theAbscis from parameter theU0 using theUi as initial guess
    private static void Compute(CurveAbscissaPoint computer,
                                Curve2DAdaptorBase c,
                                ref double abscis,
                                ref double u0,
                                ref double ui,
                                double epsilon)
    {
        // test for easy solution
        if (Math.Abs(abscis) <= global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            computer.SetParameter(u0);
            return;
        }

        double ratio = 1.0;
        AbscissaType type = ComputeType(c, ref ratio);
        switch (type)
        {
            case LengthParametrized:
            {
                computer.SetParameter(u0 + abscis / ratio);
                return;
            }
            case Parametrized:
            {
                computer.Init(c);
                computer.Perform(abscis, u0, ui, epsilon);
                return;
            }
            case AbsComposite:
            {
                int nbIntervals = c.NbIntervals(CN);
                NCollection_Array1<double> ti = new(1, nbIntervals + 1);
                c.Intervals(ti, CN);
                double l = 0.0, sign = 1.0;
                int index;
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Hunt(ti, u0, out index);
                int direction = 1;
                if (abscis < 0)
                {
                    direction = 0;
                    abscis = -abscis;
                    sign = -1.0;
                }

                while (index >= 1 && index <= nbIntervals)
                {
                    l = CurveAbscissaPoint.Length(c, u0, ti[index + direction]);
                    if (Math.Abs(l - abscis) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        computer.SetParameter(ti[index + direction]);
                        return;
                    }

                    if (l > abscis)
                    {
                        if (ui < ti[index] || ui > ti[index + 1])
                        {
                            ui = (abscis / l) * (ti[index + 1] - u0);
                            if (direction != 0)
                            {
                                ui = u0 + ui;
                            }
                            else
                            {
                                ui = u0 - ui;
                            }
                        }
                        computer.Init(c, ti[index], ti[index + 1]);
                        computer.Perform(sign * abscis, u0, ui, epsilon);
                        return;
                    }
                    else
                    {
                        u0 = ti[index + direction];
                        abscis -= l;
                    }
                    if (direction != 0)
                    {
                        ++index;
                    }
                    else
                    {
                        --index;
                    }
                }

                // Push a little bit outside the limits (hairy !!!)
                ui = u0 + 0.1;
                computer.Init(c, u0, u0 + 0.2);
                computer.Perform(sign * abscis, u0, ui, epsilon);
                return;
            }
        }
    }

    // Introduced by rbv for curvilinear parametrization
    // performs more appropriate tolerance management.
    private static void AdvCompute(CurveAbscissaPoint computer,
                                   Curve2DAdaptorBase c,
                                   ref double abscis,
                                   ref double u0,
                                   ref double ui,
                                   double epsilon)
    {
        double ratio = 1.0;
        AbscissaType type = ComputeType(c, ref ratio);
        switch (type)
        {
            case LengthParametrized:
            {
                computer.SetParameter(u0 + abscis / ratio);
                return;
            }
            case Parametrized:
            {
                // theComputer.Init (theC);
                computer.Init(c, epsilon); // rbv's modification
                computer.AdvPerform(abscis, u0, ui, epsilon);
                return;
            }
            case AbsComposite:
            {
                int nbIntervals = c.NbIntervals(CN);
                NCollection_Array1<double> ti = new(1, nbIntervals + 1);
                c.Intervals(ti, CN);
                double l = 0.0, sign = 1.0;
                int index;
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Hunt(ti, u0, out index);

                int direction = 1;
                if (abscis < 0)
                {
                    direction = 0;
                    abscis = -abscis;
                    sign = -1.0;
                }

                if (index == 0 && direction > 0)
                {
                    l = CurveAbscissaPoint.Length(c, u0, ti[index + direction], epsilon);
                    if (Math.Abs(l - abscis) <= /*Precision::Confusion()*/ epsilon)
                    {
                        computer.SetParameter(ti[index + direction]);
                        return;
                    }

                    if (l > abscis)
                    {
                        if (ui > ti[index + 1])
                        {
                            ui = (abscis / l) * (ti[index + 1] - u0);
                            ui = u0 + ui;
                        }
                        computer.Init(c, u0, ti[index + 1], epsilon);
                        computer.AdvPerform(sign * abscis, u0, ui, epsilon);
                        return;
                    }
                    else
                    {
                        u0 = ti[index + direction];
                        abscis -= l;
                    }
                    ++index;
                }

                while (index >= 1 && index <= nbIntervals)
                {
                    l = CurveAbscissaPoint.Length(c, u0, ti[index + direction], epsilon);
                    if (Math.Abs(l - abscis) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        computer.SetParameter(ti[index + direction]);
                        return;
                    }

                    if (l > abscis)
                    {
                        if (ui < ti[index] || ui > ti[index + 1])
                        {
                            ui = (abscis / l) * (ti[index + 1] - u0);
                            if (direction != 0)
                            {
                                ui = u0 + ui;
                            }
                            else
                            {
                                ui = u0 - ui;
                            }
                        }
                        computer.Init(c, ti[index], ti[index + 1], epsilon);
                        computer.AdvPerform(sign * abscis, u0, ui, epsilon);
                        return;
                    }
                    else
                    {
                        u0 = ti[index + direction];
                        abscis -= l;
                    }
                    if (direction != 0)
                    {
                        ++index;
                    }
                    else
                    {
                        --index;
                    }
                }

                // Push a little bit outside the limits (hairy !!!)
                bool isNonPeriodic = !c.IsPeriodic;
                ui = u0 + sign * 0.1;
                double u1 = u0 + sign * 0.2;
                if (isNonPeriodic)
                {
                    if (sign > 0)
                    {
                        ui = Min(ui, c.LastParameter());
                        u1 = Min(u1, c.LastParameter());
                    }
                    else
                    {
                        ui = Max(ui, c.FirstParameter());
                        u1 = Max(u1, c.FirstParameter());
                    }
                }

                computer.Init(c, u0, u1, epsilon);
                computer.AdvPerform(sign * abscis, u0, ui, epsilon);
                return;
            }
        }
    }

    // theTol is null for no tolerance (OCCT's const double*).
    private static double LengthCore(Curve2DAdaptorBase c, double u1, double u2, double? tol)
    {
        double ratio = 1.0;
        AbscissaType type = ComputeType(c, ref ratio);
        switch (type)
        {
            case LengthParametrized:
            {
                return Math.Abs(u2 - u1) * ratio;
            }
            case Parametrized:
            {
                return tol != null ? CurveAbscissaPoint.Length(c, u1, u2, tol.Value)
                                      : CurveAbscissaPoint.Length(c, u1, u2);
            }
            case AbsComposite:
            {
                int nbIntervals = c.NbIntervals(CN);
                NCollection_Array1<double> ti = new(1, nbIntervals + 1);
                c.Intervals(ti, CN);
                double uu1 = Min(u1, u2);
                double uu2 = Max(u1, u2);
                double l = 0.0;
                for (int index = 1; index <= nbIntervals; ++index)
                {
                    if (ti[index] > uu2)
                    {
                        break;
                    }
                    if (ti[index + 1] < uu1)
                    {
                        continue;
                    }
                    if (tol != null)
                    {
                        l += CurveAbscissaPoint.Length(c,
                                                         Max(ti[index], uu1),
                                                         Min(ti[index + 1], uu2),
                                                         tol.Value);
                    }
                    else
                    {
                        l += CurveAbscissaPoint.Length(c,
                                                         Max(ti[index], uu1),
                                                         Min(ti[index + 1], uu2));
                    }
                }
                return l;
            }
        }
        return RealLast();
    }

    private void ComputeCore(Curve2DAdaptorBase c, double abscissa, double u0)
    {
        double l = AbscissaPoint.Length(c);
        if (l < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            throw new ConstructionErrorException();
        }

        double abscis = abscissa;
        double uu0 = u0;
        double uUi = u0 + (abscis / l) * (c.LastParameter() - c.FirstParameter());
        Compute(_computer, c, ref abscis, ref uu0, ref uUi, c.Resolution(global::Kerf.Engine.Foundation.Precision.Confusion()));
    }

    private void AdvComputeCore(double tol, Curve2DAdaptorBase c, double abscissa, double u0)
    {
        double l = AbscissaPoint.Length(c, tol);
        /*if (aL < Precision::Confusion())
        {
          throw Standard_ConstructionError ("GCPnts_AbscissaPoint::GCPnts_AbscissaPoint");
        }*/
        double abscis = abscissa;
        double uu0 = u0;
        double uUi = 0.0;
        if (l >= global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            uUi = u0 + (abscis / l) * (c.LastParameter() - c.FirstParameter());
        }
        else
        {
            uUi = u0;
        }
        AdvCompute(_computer, c, ref abscis, ref uu0, ref uUi, tol);
    }
}
