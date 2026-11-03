// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntImp/IntImp_IntCS.gxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Intersection.Implicit;

/// <summary>
/// The static tool over a curve that the generic <c>IntImp_IntCS</c> uses (OCCT's TheCurveTool macro:
/// IntCurveSurface_TheHCurveTool, HLRBRep_LineTool, IntPatch_HCurve2dTool).
/// </summary>
public interface IImplicitCurveTool<TheCurve>
{
    static abstract double FirstParameter(in TheCurve c);

    static abstract double LastParameter(in TheCurve c);

    static abstract double Resolution(in TheCurve c, double r3d);
}

/// <summary>
/// The static tool over a parametric surface that the generic <c>IntImp_IntCS</c> uses (OCCT's ThePSurfaceTool
/// macro: Adaptor3d_HSurfaceTool, HLRBRep_SurfaceTool).
/// </summary>
public interface IPSurfaceTool<ThePSurface>
{
    static abstract double FirstUParameter(in ThePSurface s);

    static abstract double FirstVParameter(in ThePSurface s);

    static abstract double LastUParameter(in ThePSurface s);

    static abstract double LastVParameter(in ThePSurface s);

    static abstract double UResolution(in ThePSurface s, double r3d);

    static abstract double VResolution(in ThePSurface s, double r3d);
}

/// <summary>
/// What the generic <c>IntImp_IntCS</c> reads from its function besides its values (OCCT's TheFunction macro:
/// IntCurveSurface_TheCSFunctionOfHInter, IntPatch_CSFunction, HLRBRep_TheCSFunctionOfInterCSurf).
/// </summary>
public interface ICSFunction<ThePSurface, TheCurve>
{
    /// <summary>The intersection point of the last evaluation.</summary>
    Point3D Point();

    /// <summary>The value whose zero is a root (a square distance).</summary>
    double Root();

    ThePSurface AuxillarSurface();

    TheCurve AuxillarCurve();
}

/// <summary>
/// The exact intersection of a curve and a parametric surface from a starting point (u, v, w): math_FunctionSetRoot
/// on the function, within the surface's and the curve's bounds, retried from both ends of the curve.
/// </summary>
/// <remarks>
/// OCCT's generic file IntImp_IntCS.gxx, ported once as a generic class (decision 202's approach). OCCT's
/// instantiations (IntCurveSurface_TheExactHInter, and IntPatch's and HLRBRep's later) are sealed subclasses with
/// OCCT's constructors forwarded. OCCT copies the function into the object; the port keeps the caller's (a working
/// object that the caller hands over).
/// </remarks>
public class IntCS<TheFunction, ThePSurface, ThePSurfaceTool, TheCurve, TheCurveTool>
    where TheFunction : FunctionSetWithDerivatives, ICSFunction<ThePSurface, TheCurve>
    where ThePSurfaceTool : IPSurfaceTool<ThePSurface>
    where TheCurveTool : IImplicitCurveTool<TheCurve>
{
    private bool _done;
    private bool _empty;
    private readonly TheFunction _function;
    private double _w;
    private double _u;
    private double _v;
    private readonly double _tol;

    /// <summary>
    /// Searches an intersection from the starting point (<paramref name="u"/>, <paramref name="v"/>,
    /// <paramref name="w"/>). <paramref name="marginCoef"/> extends the finite bounds of the surface by that ratio
    /// of their range.
    /// </summary>
    public IntCS(double u, double v, double w, TheFunction f, double tolTangency, double marginCoef = 0.0)
    {
        _done = true;
        _empty = true;
        _function = f;
        _w = 0.0;
        _u = 0.0;
        _v = 0.0;
        _tol = tolTangency * tolTangency;
        if (_tol < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            _tol = global::Kerf.Engine.Foundation.Precision.SquareConfusion();
        }
        FunctionSetRoot rsnld = new(_function);
        double u0, u1, v0, v1, w0, w1;
        ThePSurface s = _function.AuxillarSurface();
        TheCurve c = _function.AuxillarCurve();

        w0 = TheCurveTool.FirstParameter(c);
        w1 = TheCurveTool.LastParameter(c);

        u0 = ThePSurfaceTool.FirstUParameter(s);
        v0 = ThePSurfaceTool.FirstVParameter(s);
        u1 = ThePSurfaceTool.LastUParameter(s);
        v1 = ThePSurfaceTool.LastVParameter(s);

        if (marginCoef > 0.0)
        {
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(u0) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(u1))
            {
                double marg = (u1 - u0) * marginCoef;
                if (u0 > u1)
                {
                    marg = -marg;
                }
                u0 -= marg;
                u1 += marg;
            }
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(v0) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(v1))
            {
                double marg = (v1 - v0) * marginCoef;
                if (v0 > v1)
                {
                    marg = -marg;
                }
                v0 -= marg;
                v1 += marg;
            }
        }

        Perform(u, v, w, rsnld, u0, u1, v0, v1, w0, w1);
    }

    /// <summary>Prepares a search; <see cref="Perform"/> runs it.</summary>
    public IntCS(TheFunction f, double tolTangency)
    {
        _done = true;
        _empty = true;
        _function = f;
        _tol = tolTangency * tolTangency;
        if (_tol < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            _tol = global::Kerf.Engine.Foundation.Precision.SquareConfusion();
        }
    }

    /// <summary>
    /// Searches an intersection from the starting point (<paramref name="u"/>, <paramref name="v"/>,
    /// <paramref name="w"/>) with the solver <paramref name="rsnld"/>, within the given bounds.
    /// </summary>
    public void Perform(double u, double v, double w, FunctionSetRoot rsnld, double u0, double u1, double v0,
                        double v1, double w0, double w1)
    {
        _done = true;
        math_Vector bornInf = new(1, 3), bornSup = new(1, 3), tolerance = new(1, 3), uVap = new(1, 3);
        uVap[1] = u;
        uVap[2] = v;
        uVap[3] = w;
        ThePSurface s = _function.AuxillarSurface();
        TheCurve c = _function.AuxillarCurve();

        bornInf[1] = u0;
        bornInf[2] = v0;
        bornSup[1] = u1;
        bornSup[2] = v1;

        bornInf[3] = w0;
        bornSup[3] = w1;

        tolerance[1] = ThePSurfaceTool.UResolution(s, global::Kerf.Engine.Foundation.Precision.Confusion());
        tolerance[2] = ThePSurfaceTool.VResolution(s, global::Kerf.Engine.Foundation.Precision.Confusion());
        tolerance[3] = TheCurveTool.Resolution(c, global::Kerf.Engine.Foundation.Precision.Confusion());
        rsnld.SetTolerance(tolerance);
        int autretentative = 0;
        _done = false;
        do
        {
            if (autretentative == 1)
            {
                uVap[3] = w0;
            }
            else if (autretentative == 2)
            {
                uVap[3] = w1;
            }
            autretentative++;
            rsnld.Perform(_function, uVap, bornInf, bornSup);
            if (rsnld.IsDone)
            {
                double absmyFunctionRoot = Math.Abs(_function.Root());
                if (absmyFunctionRoot <= _tol)
                {
                    rsnld.Root(uVap);
                    _u = uVap[1];
                    _v = uVap[2];
                    _w = uVap[3];
                    _empty = false;
                    _done = true;
                }
            }
        } while (_done == false && autretentative < 3);
    }

    public bool IsDone => _done;

    public bool IsEmpty
    {
        get
        {
            if (!_done)
            {
                throw new NotDoneException();
            }
            return _empty;
        }
    }

    public Point3D Point()
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        if (_empty)
        {
            throw new DomainErrorException();
        }
        return _function.Point();
    }

    public void ParameterOnSurface(out double u, out double v)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        if (_empty)
        {
            throw new DomainErrorException();
        }
        u = _u;
        v = _v;
    }

    public double ParameterOnCurve()
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        if (_empty)
        {
            throw new DomainErrorException();
        }
        return _w;
    }

    /// <summary>The function (shared).</summary>
    public TheFunction Function() => _function;
}
