// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntImp/IntImp_IntCS.gxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The static tool over a curve that the generic <c>IntImp_IntCS</c> uses (OCCT's TheCurveTool macro:
/// IntCurveSurface_TheHCurveTool, HLRBRep_LineTool, IntPatch_HCurve2dTool).
/// </summary>
public interface IIntImp_CurveTool<TheCurve>
{
    static abstract double FirstParameter(in TheCurve C);

    static abstract double LastParameter(in TheCurve C);

    static abstract double Resolution(in TheCurve C, double R3d);
}

/// <summary>
/// The static tool over a parametric surface that the generic <c>IntImp_IntCS</c> uses (OCCT's ThePSurfaceTool
/// macro: Adaptor3d_HSurfaceTool, HLRBRep_SurfaceTool).
/// </summary>
public interface IIntImp_PSurfaceTool<ThePSurface>
{
    static abstract double FirstUParameter(in ThePSurface S);

    static abstract double FirstVParameter(in ThePSurface S);

    static abstract double LastUParameter(in ThePSurface S);

    static abstract double LastVParameter(in ThePSurface S);

    static abstract double UResolution(in ThePSurface S, double R3d);

    static abstract double VResolution(in ThePSurface S, double R3d);
}

/// <summary>
/// What the generic <c>IntImp_IntCS</c> reads from its function besides its values (OCCT's TheFunction macro:
/// IntCurveSurface_TheCSFunctionOfHInter, IntPatch_CSFunction, HLRBRep_TheCSFunctionOfInterCSurf).
/// </summary>
public interface IIntImp_CSFunction<ThePSurface, TheCurve>
{
    /// <summary>The intersection point of the last evaluation.</summary>
    gp_Pnt Point();

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
public class IntImp_IntCS<TheFunction, ThePSurface, ThePSurfaceTool, TheCurve, TheCurveTool>
    where TheFunction : math_FunctionSetWithDerivatives, IIntImp_CSFunction<ThePSurface, TheCurve>
    where ThePSurfaceTool : IIntImp_PSurfaceTool<ThePSurface>
    where TheCurveTool : IIntImp_CurveTool<TheCurve>
{
    private bool done;
    private bool empty;
    private readonly TheFunction myFunction;
    private double w;
    private double u;
    private double v;
    private readonly double tol;

    /// <summary>
    /// Searches an intersection from the starting point (<paramref name="U"/>, <paramref name="V"/>,
    /// <paramref name="W"/>). <paramref name="MarginCoef"/> extends the finite bounds of the surface by that ratio
    /// of their range.
    /// </summary>
    public IntImp_IntCS(double U, double V, double W, TheFunction F, double TolTangency, double MarginCoef = 0.0)
    {
        done = true;
        empty = true;
        myFunction = F;
        w = 0.0;
        u = 0.0;
        v = 0.0;
        tol = TolTangency * TolTangency;
        if (tol < Precision.SquareConfusion())
        {
            tol = Precision.SquareConfusion();
        }
        math_FunctionSetRoot Rsnld = new(myFunction);
        double u0, u1, v0, v1, w0, w1;
        ThePSurface S = myFunction.AuxillarSurface();
        TheCurve C = myFunction.AuxillarCurve();

        w0 = TheCurveTool.FirstParameter(C);
        w1 = TheCurveTool.LastParameter(C);

        u0 = ThePSurfaceTool.FirstUParameter(S);
        v0 = ThePSurfaceTool.FirstVParameter(S);
        u1 = ThePSurfaceTool.LastUParameter(S);
        v1 = ThePSurfaceTool.LastVParameter(S);

        if (MarginCoef > 0.0)
        {
            if (!Precision.IsInfinite(u0) && !Precision.IsInfinite(u1))
            {
                double marg = (u1 - u0) * MarginCoef;
                if (u0 > u1)
                {
                    marg = -marg;
                }
                u0 -= marg;
                u1 += marg;
            }
            if (!Precision.IsInfinite(v0) && !Precision.IsInfinite(v1))
            {
                double marg = (v1 - v0) * MarginCoef;
                if (v0 > v1)
                {
                    marg = -marg;
                }
                v0 -= marg;
                v1 += marg;
            }
        }

        Perform(U, V, W, Rsnld, u0, u1, v0, v1, w0, w1);
    }

    /// <summary>Prepares a search; <see cref="Perform"/> runs it.</summary>
    public IntImp_IntCS(TheFunction F, double TolTangency)
    {
        done = true;
        empty = true;
        myFunction = F;
        tol = TolTangency * TolTangency;
        if (tol < Precision.SquareConfusion())
        {
            tol = Precision.SquareConfusion();
        }
    }

    /// <summary>
    /// Searches an intersection from the starting point (<paramref name="U"/>, <paramref name="V"/>,
    /// <paramref name="W"/>) with the solver <paramref name="Rsnld"/>, within the given bounds.
    /// </summary>
    public void Perform(double U, double V, double W, math_FunctionSetRoot Rsnld, double u0, double u1, double v0,
                        double v1, double w0, double w1)
    {
        done = true;
        math_Vector BornInf = new(1, 3), BornSup = new(1, 3), Tolerance = new(1, 3), UVap = new(1, 3);
        UVap[1] = U;
        UVap[2] = V;
        UVap[3] = W;
        ThePSurface S = myFunction.AuxillarSurface();
        TheCurve C = myFunction.AuxillarCurve();

        BornInf[1] = u0;
        BornInf[2] = v0;
        BornSup[1] = u1;
        BornSup[2] = v1;

        BornInf[3] = w0;
        BornSup[3] = w1;

        Tolerance[1] = ThePSurfaceTool.UResolution(S, Precision.Confusion());
        Tolerance[2] = ThePSurfaceTool.VResolution(S, Precision.Confusion());
        Tolerance[3] = TheCurveTool.Resolution(C, Precision.Confusion());
        Rsnld.SetTolerance(Tolerance);
        int autretentative = 0;
        done = false;
        do
        {
            if (autretentative == 1)
            {
                UVap[3] = w0;
            }
            else if (autretentative == 2)
            {
                UVap[3] = w1;
            }
            autretentative++;
            Rsnld.Perform(myFunction, UVap, BornInf, BornSup);
            if (Rsnld.IsDone())
            {
                double AbsmyFunctionRoot = Math.Abs(myFunction.Root());
                if (AbsmyFunctionRoot <= tol)
                {
                    Rsnld.Root(UVap);
                    u = UVap[1];
                    v = UVap[2];
                    w = UVap[3];
                    empty = false;
                    done = true;
                }
            }
        } while (done == false && autretentative < 3);
    }

    public bool IsDone() => done;

    public bool IsEmpty()
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        return empty;
    }

    public gp_Pnt Point()
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        if (empty)
        {
            throw new Standard_DomainError();
        }
        return myFunction.Point();
    }

    public void ParameterOnSurface(out double U, out double V)
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        if (empty)
        {
            throw new Standard_DomainError();
        }
        U = u;
        V = v;
    }

    public double ParameterOnCurve()
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        if (empty)
        {
            throw new Standard_DomainError();
        }
        return w;
    }

    /// <summary>The function (shared).</summary>
    public TheFunction Function() => myFunction;
}
