// Port of: src/ModelingData/TKGeomBase/CPnts/CPnts_AbscissaPoint.hxx
// Port of: src/ModelingData/TKGeomBase/CPnts/CPnts_AbscissaPoint.lxx
// Port of: src/ModelingData/TKGeomBase/CPnts/CPnts_AbscissaPoint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

//  cases processed: straight segment, arc of circle, parameterized curve
//  curve should be C1
//  for a parameterized curve:
//  calculate the total length of the curve
//  calculate an approached point by assimilating the curve to a staight line
//  calculate the length of the curve between the start point and the approached point
//  by successive iteration find the point and its associated parameter
//  call to FunctionRoot

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Measurement.Points;

/// <summary>
/// The algorithm computing a point on a curve at a given distance from another point on the curve, and the
/// lengths of curves (Gauss integration of the derivative's norm).
/// </summary>
/// <remarks>
/// A working object (decision 21): <c>Init</c> keeps a reference to the curve, which must outlive the next
/// <c>Perform</c>.
/// </remarks>
public sealed class CurveAbscissaPoint
{
    private bool _done;
    private double _l;
    private double _param;
    private double _uMin;
    private double _uMax;
    private readonly MyRootFunction _f = new();

    // auxiliary functions to compute the length of the derivative
    private static double F3d(double x, object? c)
    {
        ((CurveAdaptorBase)c!).D1(x, out _, out Vector3D v);
        return v.Magnitude;
    }

    private static double F2d(double x, object? c)
    {
        ((Curve2DAdaptorBase)c!).D1(x, out _, out Vector2D v);
        return v.Magnitude;
    }

    // The delegates, allocated once (OCCT passes function pointers).
    private static readonly RealFunction S_f3d = F3d;
    private static readonly RealFunction S_f2d = F2d;

    private static int Order(CurveAdaptorBase c)
    {
        switch (c.GetType())
        {
            case Line:
                return 2;

            case Parabola:
                return 5;

            case BezierCurve:
                return Min(24, 2 * c.Degree());

            case BSplineCurve:
                return Min(24, 2 * c.PoleCount - 1);

            default:
                return 10;
        }
    }

    private static int Order(Curve2DAdaptorBase c)
    {
        switch (c.GetType())
        {
            case Line:
                return 2;

            case Parabola:
                return 5;

            case BezierCurve:
                return Min(24, 2 * c.Bezier().Degree);

            case BSplineCurve:
                return Min(24, 2 * c.BSpline().PoleCount - 1);

            default:
                return 10;
        }
    }

    /// <summary>The length of the curve.</summary>
    public static double Length(CurveAdaptorBase c)
    {
        return CurveAbscissaPoint.Length(c, c.FirstParameter(), c.LastParameter());
    }

    /// <summary>The length of the curve.</summary>
    public static double Length(Curve2DAdaptorBase c)
    {
        return CurveAbscissaPoint.Length(c, c.FirstParameter(), c.LastParameter());
    }

    /// <summary>The length of the curve with the given tolerance.</summary>
    public static double Length(CurveAdaptorBase c, double tol)
    {
        return CurveAbscissaPoint.Length(c, c.FirstParameter(), c.LastParameter(), tol);
    }

    /// <summary>The length of the curve with the given tolerance.</summary>
    public static double Length(Curve2DAdaptorBase c, double tol)
    {
        return CurveAbscissaPoint.Length(c, c.FirstParameter(), c.LastParameter(), tol);
    }

    /// <summary>The length of the curve between the parameters U1 and U2.</summary>
    /// <exception cref="ConstructionErrorException">The integration fails.</exception>
    public static double Length(CurveAdaptorBase c, double u1, double u2)
    {
        MyGaussFunction fg = new();
        RealFunction rf = S_f3d;
        fg.Init(rf, c);
        GaussSingleIntegration theLength = new(fg, u1, u2, Order(c));
        if (!theLength.IsDone)
        {
            throw new ConstructionErrorException();
        }
        return Math.Abs(theLength.Value());
    }

    /// <summary>The length of the curve between the parameters U1 and U2.</summary>
    /// <exception cref="ConstructionErrorException">The integration fails.</exception>
    public static double Length(Curve2DAdaptorBase c, double u1, double u2)
    {
        MyGaussFunction fg = new();
        RealFunction rf = S_f2d;
        fg.Init(rf, c);
        GaussSingleIntegration theLength = new(fg, u1, u2, Order(c));
        if (!theLength.IsDone)
        {
            throw new ConstructionErrorException();
        }
        return Math.Abs(theLength.Value());
    }

    /// <summary>The length of the curve between the parameters U1 and U2 with the given tolerance.</summary>
    /// <exception cref="ConstructionErrorException">The integration fails.</exception>
    public static double Length(CurveAdaptorBase c, double u1, double u2, double tol)
    {
        MyGaussFunction fg = new();
        RealFunction rf = S_f3d;
        fg.Init(rf, c);
        GaussSingleIntegration theLength = new(fg, u1, u2, Order(c), tol);
        if (!theLength.IsDone)
        {
            throw new ConstructionErrorException();
        }
        return Math.Abs(theLength.Value());
    }

    /// <summary>The length of the curve between the parameters U1 and U2 with the given tolerance.</summary>
    /// <exception cref="ConstructionErrorException">The integration fails.</exception>
    public static double Length(Curve2DAdaptorBase c, double u1, double u2, double tol)
    {
        MyGaussFunction fg = new();
        RealFunction rf = S_f2d;
        fg.Init(rf, c);
        GaussSingleIntegration theLength = new(fg, u1, u2, Order(c), tol);
        if (!theLength.IsDone)
        {
            throw new ConstructionErrorException();
        }
        return Math.Abs(theLength.Value());
    }

    public CurveAbscissaPoint()
    {
        _done = false;
        _l = 0.0;
        _param = 0.0;
        _uMin = 0.0;
        _uMax = 0.0;
    }

    /// <summary>The point at the distance Abscissa from the point of parameter U0.</summary>
    public CurveAbscissaPoint(CurveAdaptorBase c, double abscissa, double u0, double resolution)
    {
        //  Init(C);
        Init(c, resolution); // rbv's modification
        //
        Perform(abscissa, u0, resolution);
    }

    /// <summary>The point at the distance Abscissa from the point of parameter U0.</summary>
    public CurveAbscissaPoint(Curve2DAdaptorBase c, double abscissa, double u0, double resolution)
    {
        Init(c);
        Perform(abscissa, u0, resolution);
    }

    /// <summary>The point at the distance Abscissa from the point of parameter U0, starting from Ui.</summary>
    public CurveAbscissaPoint(CurveAdaptorBase c, double abscissa, double u0, double ui, double resolution)
    {
        Init(c);
        Perform(abscissa, u0, ui, resolution);
    }

    /// <summary>The point at the distance Abscissa from the point of parameter U0, starting from Ui.</summary>
    public CurveAbscissaPoint(Curve2DAdaptorBase c, double abscissa, double u0, double ui, double resolution)
    {
        Init(c);
        Perform(abscissa, u0, ui, resolution);
    }

    /// <summary>Initializes the algorithm on the whole curve.</summary>
    public void Init(CurveAdaptorBase c)
    {
        Init(c, c.FirstParameter(), c.LastParameter());
    }

    /// <summary>Initializes the algorithm on the whole curve.</summary>
    public void Init(Curve2DAdaptorBase c)
    {
        Init(c, c.FirstParameter(), c.LastParameter());
    }

    /// <summary>Initializes the algorithm on the whole curve with the given tolerance (rbv, curvilinear parametrisation).</summary>
    public void Init(CurveAdaptorBase c, double tol)
    {
        Init(c, c.FirstParameter(), c.LastParameter(), tol);
    }

    /// <summary>Initializes the algorithm on the whole curve with the given tolerance.</summary>
    public void Init(Curve2DAdaptorBase c, double tol)
    {
        Init(c, c.FirstParameter(), c.LastParameter(), tol);
    }

    /// <summary>Initializes the algorithm on the curve between U1 and U2.</summary>
    public void Init(CurveAdaptorBase c, double u1, double u2)
    {
        RealFunction rf = S_f3d;
        _f.Init(rf, c, Order(c));
        _l = CurveAbscissaPoint.Length(c, u1, u2);
        _uMin = Min(u1, u2);
        _uMax = Max(u1, u2);
        double du = _uMax - _uMin;
        _uMin = _uMin - du;
        _uMax = _uMax + du;
    }

    /// <summary>Initializes the algorithm on the curve between U1 and U2.</summary>
    public void Init(Curve2DAdaptorBase c, double u1, double u2)
    {
        RealFunction rf = S_f2d;
        _f.Init(rf, c, Order(c));
        _l = CurveAbscissaPoint.Length(c, u1, u2);
        _uMin = Min(u1, u2);
        _uMax = Max(u1, u2);
        double du = _uMax - _uMin;
        _uMin = _uMin - du;
        _uMax = _uMax + du;
    }

    /// <summary>Initializes the algorithm on the curve between U1 and U2 with the given tolerance (rbv).</summary>
    public void Init(CurveAdaptorBase c, double u1, double u2, double tol)
    {
        RealFunction rf = S_f3d;
        _f.Init(rf, c, Order(c));
        _l = CurveAbscissaPoint.Length(c, u1, u2, tol);
        _uMin = Min(u1, u2);
        _uMax = Max(u1, u2);
        double du = _uMax - _uMin;
        _uMin = _uMin - du;
        _uMax = _uMax + du;
    }

    /// <summary>Initializes the algorithm on the curve between U1 and U2 with the given tolerance.</summary>
    public void Init(Curve2DAdaptorBase c, double u1, double u2, double tol)
    {
        RealFunction rf = S_f2d;
        _f.Init(rf, c, Order(c));
        _l = CurveAbscissaPoint.Length(c, u1, u2, tol);
        _uMin = Min(u1, u2);
        _uMax = Max(u1, u2);
        double du = _uMax - _uMin;
        _uMin = _uMin - du;
        _uMax = _uMax + du;
    }

    /// <summary>Computes the point at the distance Abscissa from the point of parameter U0.</summary>
    public void Perform(double abscissa, double u0, double resolution)
    {
        if (_l < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            //
            //  leave less violently : it is expected that
            //  the increment of the level of myParam will not be great
            //
            _done = true;
            _param = u0;
        }
        else
        {
            double ui = u0 + (abscissa / _l) * (_uMax - _uMin) / 3.0;
            // exercise : why 3 ?
            Perform(abscissa, u0, ui, resolution);
        }
    }

    /// <summary>Computes the point at the distance Abscissa from the point of parameter U0, starting from Ui.</summary>
    public void Perform(double abscissa, double u0, double ui, double resolution)
    {
        if (_l < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            //
            //  leave less violently :
            //
            _done = true;
            _param = u0;
        }
        else
        {
            _done = false;
            _f.Init(u0, abscissa);

            FunctionRoot solution = new(_f, ui, resolution, _uMin, _uMax);

            // Temporarily suspend the validity test of the solution
            // it is necessary to make a tolreached as soon as one will make a cdl
            // lbo 21/03/97
            if (solution.IsDone)
            {
                _done = true;
                _param = solution.Root();
            }
        }
    }

    /// <summary>
    /// Computes the point at the distance Abscissa from the point of parameter U0, starting from Ui, integrating
    /// with the tolerance Resolution / 10 (rbv).
    /// </summary>
    public void AdvPerform(double abscissa, double u0, double ui, double resolution)
    {
        if (_l < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            //
            //  leave less violently :
            //
            _done = true;
            _param = u0;
        }
        else
        {
            _done = false;
            //    myF.Init(U0, Abscissa);
            _f.Init(u0, abscissa, resolution / 10); // rbv's modification

            FunctionRoot solution = new(_f, ui, resolution, _uMin, _uMax);

            // Temporarily suspend the validity test of the solution
            // it is necessary to make a tolreached as soon as one will make a cdl
            // lbo 21/03/97
            if (solution.IsDone)
            {
                _done = true;
                _param = solution.Root();
            }
        }
    }

    /// <summary>True if the computation was successful.</summary>
    public bool IsDone => _done;

    /// <summary>The parameter of the solution.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public double Parameter()
    {
        StdFail_NotDone_Raise_if(!_done, "CPnts_AbscissaPoint::Parameter() - no result");
        return _param;
    }

    /// <summary>Enforces the parameter of the solution (used when the solution is known).</summary>
    public void SetParameter(double p)
    {
        _param = p;
        _done = true;
    }
}
