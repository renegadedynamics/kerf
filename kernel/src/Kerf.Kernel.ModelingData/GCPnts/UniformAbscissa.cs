// Port of: src/ModelingData/TKGeomBase/GCPnts/GCPnts_UniformAbscissa.hxx
// Port of: src/ModelingData/TKGeomBase/GCPnts/GCPnts_UniformAbscissa.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Measurement.AbscissaType;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Measurement;

/// <summary>
/// Computes points on a curve spaced at a uniform curvilinear distance (abscissa), given either the distance or the
/// number of points.
/// </summary>
/// <remarks>
/// A working object (decision 21): <c>Initialize</c> recomputes. A tolerance of -1 (the default) means
/// Precision::Confusion(). The templates over the curve type are ported per instantiation (decision 187).
/// </remarks>
internal sealed class UniformAbscissa
{
    private bool _done;
    private int _nbPoints;
    private double _abscissa;
    private NCollection_HArray1<double>? _params;

    /// <summary>An empty algorithm; use <c>Initialize</c>.</summary>
    public UniformAbscissa()
    {
        _done = false;
        _nbPoints = 0;
        _abscissa = 0.0;
    }

    /// <summary>Points on the curve at the uniform distance theAbscissa.</summary>
    public UniformAbscissa(CurveAdaptorBase c, double abscissa, double toler = -1)
        : this()
    {
        Initialize(c, abscissa, toler);
    }

    /// <summary>Points on the curve at the uniform distance theAbscissa.</summary>
    public UniformAbscissa(Curve2DAdaptorBase c, double abscissa, double toler = -1)
        : this()
    {
        Initialize(c, abscissa, toler);
    }

    /// <summary>Points on the part theU1..theU2 of the curve at the uniform distance theAbscissa.</summary>
    public UniformAbscissa(CurveAdaptorBase c, double abscissa, double u1, double u2,
        double toler = -1)
        : this()
    {
        Initialize(c, abscissa, u1, u2, toler);
    }

    /// <summary>Points on the part theU1..theU2 of the curve at the uniform distance theAbscissa.</summary>
    public UniformAbscissa(Curve2DAdaptorBase c, double abscissa, double u1, double u2,
        double toler = -1)
        : this()
    {
        Initialize(c, abscissa, u1, u2, toler);
    }

    /// <summary>theNbPoints points on the curve at a uniform distance.</summary>
    /// <exception cref="ConstructionErrorException">theNbPoints is below 2.</exception>
    public UniformAbscissa(CurveAdaptorBase c, int nbPoints, double toler = -1)
        : this()
    {
        Initialize(c, nbPoints, toler);
    }

    /// <summary>theNbPoints points on the curve at a uniform distance.</summary>
    /// <exception cref="ConstructionErrorException">theNbPoints is below 2.</exception>
    public UniformAbscissa(Curve2DAdaptorBase c, int nbPoints, double toler = -1)
        : this()
    {
        Initialize(c, nbPoints, toler);
    }

    /// <summary>theNbPoints points on the part theU1..theU2 of the curve at a uniform distance.</summary>
    /// <exception cref="ConstructionErrorException">theNbPoints is below 2.</exception>
    public UniformAbscissa(CurveAdaptorBase c, int nbPoints, double u1, double u2,
        double toler = -1)
        : this()
    {
        Initialize(c, nbPoints, u1, u2, toler);
    }

    /// <summary>theNbPoints points on the part theU1..theU2 of the curve at a uniform distance.</summary>
    /// <exception cref="ConstructionErrorException">theNbPoints is below 2.</exception>
    public UniformAbscissa(Curve2DAdaptorBase c, int nbPoints, double u1, double u2,
        double toler = -1)
        : this()
    {
        Initialize(c, nbPoints, u1, u2, toler);
    }

    /// <summary>Recomputes points on the curve at the uniform distance theAbscissa.</summary>
    public void Initialize(CurveAdaptorBase c, double abscissa, double toler = -1)
    {
        Initialize(c, abscissa, c.FirstParameter(), c.LastParameter(), toler);
    }

    /// <summary>Recomputes points on the curve at the uniform distance theAbscissa.</summary>
    public void Initialize(Curve2DAdaptorBase c, double abscissa, double toler = -1)
    {
        Initialize(c, abscissa, c.FirstParameter(), c.LastParameter(), toler);
    }

    /// <summary>Recomputes points on the part theU1..theU2 of the curve at the uniform distance theAbscissa.</summary>
    public void Initialize(CurveAdaptorBase c, double abscissa, double u1, double u2, double toler = -1)
    {
        InitializeCore(c, abscissa, u1, u2, toler);
    }

    /// <summary>Recomputes points on the part theU1..theU2 of the curve at the uniform distance theAbscissa.</summary>
    public void Initialize(Curve2DAdaptorBase c, double abscissa, double u1, double u2,
        double toler = -1)
    {
        InitializeCore(c, abscissa, u1, u2, toler);
    }

    /// <summary>Recomputes theNbPoints points on the curve at a uniform distance.</summary>
    public void Initialize(CurveAdaptorBase c, int nbPoints, double toler = -1)
    {
        Initialize(c, nbPoints, c.FirstParameter(), c.LastParameter(), toler);
    }

    /// <summary>Recomputes theNbPoints points on the curve at a uniform distance.</summary>
    public void Initialize(Curve2DAdaptorBase c, int nbPoints, double toler = -1)
    {
        Initialize(c, nbPoints, c.FirstParameter(), c.LastParameter(), toler);
    }

    /// <summary>Recomputes theNbPoints points on the part theU1..theU2 of the curve at a uniform distance.</summary>
    public void Initialize(CurveAdaptorBase c, int nbPoints, double u1, double u2, double toler = -1)
    {
        InitializeCore(c, nbPoints, u1, u2, toler);
    }

    /// <summary>Recomputes theNbPoints points on the part theU1..theU2 of the curve at a uniform distance.</summary>
    public void Initialize(Curve2DAdaptorBase c, int nbPoints, double u1, double u2,
        double toler = -1)
    {
        InitializeCore(c, nbPoints, u1, u2, toler);
    }

    /// <summary>True if the computation was successful.</summary>
    public bool IsDone => _done;

    /// <summary>The number of points computed.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public int PointCount
    {
        get
        {
            StdFail_NotDone_Raise_if(!_done, "GCPnts_UniformAbscissa::NbPoints()");
            return _nbPoints;
        }
    }

    /// <summary>The parameter of the point of index Index (1 to NbPoints).</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public double Parameter(int index)
    {
        StdFail_NotDone_Raise_if(!_done, "GCPnts_UniformAbscissa::Parameter()");
        return _params!.Value(index);
    }

    // NbPoints and Parameter without the StdFail_NotDone check, which OCCT's release build compiles out
    // (No_Exception): for ported callers that read a failed result as OCCT does (decision 189).
    internal int PointsUncheckedCount => _nbPoints;

    internal double ParameterUnchecked(int index) => _params!.Value(index);

    /// <summary>The distance between consecutive points (computed, when a number of points was given).</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public double Abscissa()
    {
        StdFail_NotDone_Raise_if(!_done, "GCPnts_UniformAbscissa::Abscissa()");
        return _abscissa;
    }

    // ---- Templates over the curve type (3D instantiation) ----

    private static double GetParameterLengthRatio(CurveAdaptorBase c)
    {
        switch (c.GetType())
        {
            case Circle:
            {
                return c.Circle().Radius;
            }
            case Line:
            {
                return 1.0;
            }
            case BezierCurve:
            case BSplineCurve:
            {
                if (!c.IsRational)
                {
                    return c.DN(0.0, 1).Magnitude;
                }
                return RealLast();
            }
            default:
            {
                return RealLast();
            }
        }
    }

    private static AbscissaType GetAbsType(CurveAdaptorBase c)
    {
        if (c.NbIntervals(C1) > 1)
        {
            return AbsComposite;
        }

        switch (c.GetType())
        {
            case Line:
            case Circle:
            {
                return LengthParametrized;
            }
            case BezierCurve:
            {
                BezierCurve3D bz = c.Bezier();
                if (bz.PoleCount == 2 && !bz.IsRational)
                {
                    return LengthParametrized;
                }
                return Parametrized;
            }
            case BSplineCurve:
            {
                BSplineCurve3D bs = c.BSpline();
                if (bs.PoleCount == 2 && !bs.IsRational)
                {
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

    private static bool Perform(NCollection_Array1<double> parameters,
                                CurveAdaptorBase c,
                                double abscissa,
                                double u1,
                                double u2,
                                double totalLength,
                                out int nbPoints,
                                double epsilon)
    {
        bool isLocalDone = true;
        double uu1 = Min(u1, u2), uu2 = Max(u1, u2);
        nbPoints = 0;

        // this initialization avoids the computation of the Length of the curve
        double delta = (abscissa / totalLength) * (uu2 - uu1);
        int index = 1;
        parameters.SetValue(index, uu1);
        for (bool isNotDone = true; isNotDone;)
        {
            double ui = parameters.Value(index) + delta;
            if (ui > uu2)
            {
                // MSV 21.04.2004: OCC5739 (GCPnts_UniformAbscissa gives incorrect distribution of points)
                // if (aUU2 - theParameters.Value (anIndex) > 0.01 * aDelta) { Index += 1; }
                // theParameters.SetValue (anIndex, aUU2);
                // isNotDone = false;
                // break;
                ui = uu2;
            }
            AbscissaPoint abscissaFinder = new(c,
                                                        abscissa,
                                                        parameters.Value(index),
                                                        ui,
                                                        epsilon);
            if (abscissaFinder.IsDone)
            {
                index += 1;
                ui = abscissaFinder.Parameter();
                if (Math.Abs(ui - uu2) <= epsilon)
                {
                    parameters.SetValue(index, uu2);
                    isNotDone = false;
                }
                else if (ui < uu2)
                {
                    parameters.SetValue(index, ui);
                }
                else
                {
                    parameters.SetValue(index, uu2);
                    isNotDone = false;
                }
                isNotDone = isNotDone && (index + 1 <= parameters.Length());
            }
            else
            {
                isLocalDone = false;
                isNotDone = true;
                delta -= delta / 10;
                if (delta <= global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    break;
                }
            }
        }
        nbPoints = index;
        return isLocalDone;
    }

    private static bool PerformLengthParametrized(NCollection_Array1<double> parameters,
                                                  CurveAdaptorBase c,
                                                  double abscissa,
                                                  double u1,
                                                  double u2,
                                                  double totalLength,
                                                  out int nbPoints,
                                                  double epsilon)
    {
        double uu1 = Min(u1, u2);
        double uu2 = Max(u1, u2);

        // Ratio is defined as dl = Ratio * du
        // for a circle of gp Ratio is equal to the radius of the circle.
        // for a line of gp ratio is equal to 1.0
        double ratio = GetParameterLengthRatio(c);

        if (abscissa < 0.0e0)
        {
            uu2 = Min(u1, u2);
            uu1 = Max(u1, u2);
        }

        double delta = (abscissa / totalLength) * (uu2 - uu1);
        int index = 1;
        nbPoints = 0;
        parameters.SetValue(index, uu1);
        for (bool isNotDone = true; isNotDone;)
        {
            index += 1;
            double ui = parameters.Value(index - 1) + delta;
            if (Math.Abs(ui - uu2) <= epsilon)
            {
                parameters.SetValue(index, uu2);
                isNotDone = false;
            }
            else if (ui < uu2)
            {
                parameters.SetValue(index, ui);
            }
            else
            {
                isNotDone = false;
                if (Math.Abs(parameters.Value(index - 1) - uu2) * ratio / abscissa < 0.1)
                {
                    parameters.SetValue(index - 1, uu2);
                    index -= 1;
                }
                else
                {
                    parameters.SetValue(index, uu2);
                }
            }
            isNotDone = (index + 1 <= parameters.Length()) && isNotDone;
        }
        nbPoints = index;
        return true;
    }

    private void InitializeCore(CurveAdaptorBase c, double abscissa, double u1, double u2, double tol)
    {
        _abscissa = abscissa;
        _nbPoints = 0;
        _done = false;

        double epsilon = c.Resolution(Max(tol, global::Kerf.Engine.Foundation.Precision.Confusion()));
        double l = AbscissaPoint.Length(c, u1, u2, epsilon);
        if (l <= global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            return;
        }

        // compute the total Length here so that we can guess
        // the number of points instead of letting the constructor
        // of CPnts_AbscissaPoint do that and lose the information
        double sizeR = l / Math.Abs(abscissa) + 5;
        if (sizeR >= IntegerLast()) // modified by Igor Motchalov 23/04/2001
        {
            return;
        }

        int size = (int)sizeR;
        if (_params is not null)
        {
            if (_params.Length() < size)
            {
                _params = null;
                _params = new NCollection_HArray1<double>(1, size);
            }
        }
        else
        {
            _params = new NCollection_HArray1<double>(1, size);
        }

        AbscissaType type = GetAbsType(c);
        switch (type)
        {
            case LengthParametrized:
            {
                _done = PerformLengthParametrized(_params.ChangeArray1(),
                                                   c,
                                                   abscissa,
                                                   u1,
                                                   u2,
                                                   l,
                                                   out _nbPoints,
                                                   epsilon);
                break;
            }
            case Parametrized:
            case AbsComposite:
            {
                _done = Perform(_params.ChangeArray1(),
                                 c,
                                 abscissa,
                                 u1,
                                 u2,
                                 l,
                                 out _nbPoints,
                                 epsilon);
                break;
            }
        }
    }

    private void InitializeCore(CurveAdaptorBase c, int nbPoints, double u1, double u2, double tol)
    {
        Standard_ConstructionError_Raise_if(
            nbPoints <= 1,
            "GCPnts_UniformAbscissa::Initialize() - number of points should be >= 2");
        _nbPoints = 0;
        _done = false;

        double epsilon = c.Resolution(Max(tol, global::Kerf.Engine.Foundation.Precision.Confusion()));
        // although very similar to Initialize with Abscissa this avoid
        // the computation of the total length of the curve twice
        double l = AbscissaPoint.Length(c, u1, u2, epsilon);
        if (l <= global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            return;
        }

        double abscissa = _abscissa = l / (nbPoints - 1);

        // compute the total Length here so that we can guess
        // the number of points instead of letting the constructor
        // of CPnts_AbscissaPoint do that and lose the information
        int size = nbPoints + 5;
        if (_params is not null)
        {
            if (_params.Length() < size)
            {
                _params = null;
                _params = new NCollection_HArray1<double>(1, size);
            }
        }
        else
        {
            _params = new NCollection_HArray1<double>(1, size);
        }

        _nbPoints = 0;
        AbscissaType type = GetAbsType(c);
        switch (type)
        {
            case LengthParametrized:
            {
                _done = PerformLengthParametrized(_params.ChangeArray1(),
                                                   c,
                                                   abscissa,
                                                   u1,
                                                   u2,
                                                   l,
                                                   out _nbPoints,
                                                   epsilon);
                break;
            }
            case Parametrized:
            case AbsComposite:
            {
                _done = Perform(_params.ChangeArray1(),
                                 c,
                                 abscissa,
                                 u1,
                                 u2,
                                 l,
                                 out _nbPoints,
                                 epsilon);
                break;
            }
        }
    }

    // ---- Templates over the curve type (2D instantiation) ----

    private static double GetParameterLengthRatio(Curve2DAdaptorBase c)
    {
        switch (c.GetType())
        {
            case Circle:
            {
                return c.Circle().Radius;
            }
            case Line:
            {
                return 1.0;
            }
            case BezierCurve:
            case BSplineCurve:
            {
                if (!c.IsRational)
                {
                    return c.DN(0.0, 1).Magnitude;
                }
                return RealLast();
            }
            default:
            {
                return RealLast();
            }
        }
    }

    private static AbscissaType GetAbsType(Curve2DAdaptorBase c)
    {
        if (c.NbIntervals(C1) > 1)
        {
            return AbsComposite;
        }

        switch (c.GetType())
        {
            case Line:
            case Circle:
            {
                return LengthParametrized;
            }
            case BezierCurve:
            {
                BezierCurve2D bz = c.Bezier();
                if (bz.PoleCount == 2 && !bz.IsRational)
                {
                    return LengthParametrized;
                }
                return Parametrized;
            }
            case BSplineCurve:
            {
                BSplineCurve2D bs = c.BSpline();
                if (bs.PoleCount == 2 && !bs.IsRational)
                {
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

    private static bool Perform(NCollection_Array1<double> parameters,
                                Curve2DAdaptorBase c,
                                double abscissa,
                                double u1,
                                double u2,
                                double totalLength,
                                out int nbPoints,
                                double epsilon)
    {
        bool isLocalDone = true;
        double uu1 = Min(u1, u2), uu2 = Max(u1, u2);
        nbPoints = 0;

        // this initialization avoids the computation of the Length of the curve
        double delta = (abscissa / totalLength) * (uu2 - uu1);
        int index = 1;
        parameters.SetValue(index, uu1);
        for (bool isNotDone = true; isNotDone;)
        {
            double ui = parameters.Value(index) + delta;
            if (ui > uu2)
            {
                // MSV 21.04.2004: OCC5739 (GCPnts_UniformAbscissa gives incorrect distribution of points)
                // if (aUU2 - theParameters.Value (anIndex) > 0.01 * aDelta) { Index += 1; }
                // theParameters.SetValue (anIndex, aUU2);
                // isNotDone = false;
                // break;
                ui = uu2;
            }
            AbscissaPoint abscissaFinder = new(c,
                                                        abscissa,
                                                        parameters.Value(index),
                                                        ui,
                                                        epsilon);
            if (abscissaFinder.IsDone)
            {
                index += 1;
                ui = abscissaFinder.Parameter();
                if (Math.Abs(ui - uu2) <= epsilon)
                {
                    parameters.SetValue(index, uu2);
                    isNotDone = false;
                }
                else if (ui < uu2)
                {
                    parameters.SetValue(index, ui);
                }
                else
                {
                    parameters.SetValue(index, uu2);
                    isNotDone = false;
                }
                isNotDone = isNotDone && (index + 1 <= parameters.Length());
            }
            else
            {
                isLocalDone = false;
                isNotDone = true;
                delta -= delta / 10;
                if (delta <= global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    break;
                }
            }
        }
        nbPoints = index;
        return isLocalDone;
    }

    private static bool PerformLengthParametrized(NCollection_Array1<double> parameters,
                                                  Curve2DAdaptorBase c,
                                                  double abscissa,
                                                  double u1,
                                                  double u2,
                                                  double totalLength,
                                                  out int nbPoints,
                                                  double epsilon)
    {
        double uu1 = Min(u1, u2);
        double uu2 = Max(u1, u2);

        // Ratio is defined as dl = Ratio * du
        // for a circle of gp Ratio is equal to the radius of the circle.
        // for a line of gp ratio is equal to 1.0
        double ratio = GetParameterLengthRatio(c);

        if (abscissa < 0.0e0)
        {
            uu2 = Min(u1, u2);
            uu1 = Max(u1, u2);
        }

        double delta = (abscissa / totalLength) * (uu2 - uu1);
        int index = 1;
        nbPoints = 0;
        parameters.SetValue(index, uu1);
        for (bool isNotDone = true; isNotDone;)
        {
            index += 1;
            double ui = parameters.Value(index - 1) + delta;
            if (Math.Abs(ui - uu2) <= epsilon)
            {
                parameters.SetValue(index, uu2);
                isNotDone = false;
            }
            else if (ui < uu2)
            {
                parameters.SetValue(index, ui);
            }
            else
            {
                isNotDone = false;
                if (Math.Abs(parameters.Value(index - 1) - uu2) * ratio / abscissa < 0.1)
                {
                    parameters.SetValue(index - 1, uu2);
                    index -= 1;
                }
                else
                {
                    parameters.SetValue(index, uu2);
                }
            }
            isNotDone = (index + 1 <= parameters.Length()) && isNotDone;
        }
        nbPoints = index;
        return true;
    }

    private void InitializeCore(Curve2DAdaptorBase c, double abscissa, double u1, double u2, double tol)
    {
        _abscissa = abscissa;
        _nbPoints = 0;
        _done = false;

        double epsilon = c.Resolution(Max(tol, global::Kerf.Engine.Foundation.Precision.Confusion()));
        double l = AbscissaPoint.Length(c, u1, u2, epsilon);
        if (l <= global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            return;
        }

        // compute the total Length here so that we can guess
        // the number of points instead of letting the constructor
        // of CPnts_AbscissaPoint do that and lose the information
        double sizeR = l / Math.Abs(abscissa) + 5;
        if (sizeR >= IntegerLast()) // modified by Igor Motchalov 23/04/2001
        {
            return;
        }

        int size = (int)sizeR;
        if (_params is not null)
        {
            if (_params.Length() < size)
            {
                _params = null;
                _params = new NCollection_HArray1<double>(1, size);
            }
        }
        else
        {
            _params = new NCollection_HArray1<double>(1, size);
        }

        AbscissaType type = GetAbsType(c);
        switch (type)
        {
            case LengthParametrized:
            {
                _done = PerformLengthParametrized(_params.ChangeArray1(),
                                                   c,
                                                   abscissa,
                                                   u1,
                                                   u2,
                                                   l,
                                                   out _nbPoints,
                                                   epsilon);
                break;
            }
            case Parametrized:
            case AbsComposite:
            {
                _done = Perform(_params.ChangeArray1(),
                                 c,
                                 abscissa,
                                 u1,
                                 u2,
                                 l,
                                 out _nbPoints,
                                 epsilon);
                break;
            }
        }
    }

    private void InitializeCore(Curve2DAdaptorBase c, int nbPoints, double u1, double u2, double tol)
    {
        Standard_ConstructionError_Raise_if(
            nbPoints <= 1,
            "GCPnts_UniformAbscissa::Initialize() - number of points should be >= 2");
        _nbPoints = 0;
        _done = false;

        double epsilon = c.Resolution(Max(tol, global::Kerf.Engine.Foundation.Precision.Confusion()));
        // although very similar to Initialize with Abscissa this avoid
        // the computation of the total length of the curve twice
        double l = AbscissaPoint.Length(c, u1, u2, epsilon);
        if (l <= global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            return;
        }

        double abscissa = _abscissa = l / (nbPoints - 1);

        // compute the total Length here so that we can guess
        // the number of points instead of letting the constructor
        // of CPnts_AbscissaPoint do that and lose the information
        int size = nbPoints + 5;
        if (_params is not null)
        {
            if (_params.Length() < size)
            {
                _params = null;
                _params = new NCollection_HArray1<double>(1, size);
            }
        }
        else
        {
            _params = new NCollection_HArray1<double>(1, size);
        }

        _nbPoints = 0;
        AbscissaType type = GetAbsType(c);
        switch (type)
        {
            case LengthParametrized:
            {
                _done = PerformLengthParametrized(_params.ChangeArray1(),
                                                   c,
                                                   abscissa,
                                                   u1,
                                                   u2,
                                                   l,
                                                   out _nbPoints,
                                                   epsilon);
                break;
            }
            case Parametrized:
            case AbsComposite:
            {
                _done = Perform(_params.ChangeArray1(),
                                 c,
                                 abscissa,
                                 u1,
                                 u2,
                                 l,
                                 out _nbPoints,
                                 epsilon);
                break;
            }
        }
    }
}
