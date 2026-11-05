// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_PrjResolve.hxx
// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_PrjResolve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Created on: 1997-11-06
// Created by: Roman BORISOV

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Projection;

/// <summary>
/// Finds the orthogonal projection of a curve point on a surface near a starting point, one of the variables
/// (t, u, v) fixed (<see cref="PrjFunc"/>): Newton's method, then math_FunctionSetRoot if it fails.
/// </summary>
/// <remarks>
/// A working object (decision 21). It keeps the caller's adaptors, as OCCT keeps its pointers: they must outlive
/// the solver.
/// </remarks>
internal class PrjResolve
{
    private readonly CurveAdaptorBase _curve;
    private readonly SurfaceAdaptorBase _surface;
    private bool _done;
    private Point2D _solution;
    private readonly int _fix;

    /// <exception cref="ConstructionErrorException"><paramref name="fix"/> is not 1, 2 or 3.</exception>
    public PrjResolve(CurveAdaptorBase c, SurfaceAdaptorBase s, int fix)
    {
        _done = false;
        _fix = fix;
        if (_fix > 3 || _fix < 1)
        {
            throw new ConstructionErrorException();
        }
        _solution = new Point2D(0.0, 0.0);
        _curve = c;
        _surface = s;
    }

    // void ProjLib_PrjResolve::Perform(const double t, const double U, const
    // double  V, const gp_Pnt2d& Tol2d, const gp_Pnt2d& Inf, const gp_Pnt2d& Sup, const
    // double FuncTol, const bool StrictInside)

    /// <summary>
    /// Calculates the orthogonal projection from C(t) to S near the point (U, V): the zero of the gradient of
    /// <c>distance(S(u, v), C(t))</c> in the free variables, within [Inf, Sup] widened by the tolerances.
    /// <paramref name="strictInside"/> is not used (as in OCCT).
    /// </summary>
    public void Perform(double t,
                        double u,
                        double v,
                        in Point2D tol2d,
                        in Point2D inf,
                        in Point2D sup,
                        double funcTol = -1,
                        bool strictInside = false)
    {
        _ = strictInside;
        _done = false;
        double fixVal = 0.0;
        Point2D extInf = new(0.0, 0.0), extSup = new(0.0, 0.0);
        double extU = 10 * tol2d.X, extV = 10 * tol2d.Y;
        math_Vector tol = new(1, 2), start = new(1, 2), bInf = new(1, 2), bSup = new(1, 2);

        extInf = extInf.WithCoord(inf.X - extU, inf.Y - extV);
        extSup = extSup.WithCoord(sup.X + extU, sup.Y + extV);
        bInf[1] = extInf.X;
        bInf[2] = extInf.Y;
        bSup[1] = extSup.X;
        bSup[2] = extSup.Y;
        tol[1] = tol2d.X;
        tol[2] = tol2d.Y;

        switch (_fix)
        {
            case 1:
                start[1] = u;
                start[2] = v;
                fixVal = t;
                break;
            case 2:
                start[1] = t;
                start[2] = v;
                fixVal = u;
                break;
            case 3:
                start[1] = t;
                start[2] = u;
                fixVal = v;
                break;
        }

        PrjFunc f = new(_curve, fixVal, _surface, _fix);

        //  int option = 1;//2;
        //  if (option == 1) {
        //    math_FunctionSetRoot S1 (F, Start,Tol, BInf, BSup);
        //    if (!S1.IsDone()) { return; }
        //  }
        //  else {
        NewtonFunctionSetRoot sr = new(f, tol, funcTol);
        sr.Perform(f, start, bInf, bSup);
        //    if (!SR.IsDone()) { return; }
        if (!sr.IsDone)
        {
            FunctionSetRoot s1 = new(f, tol);
            s1.Perform(f, start, bInf, bSup);

            if (!s1.IsDone)
            {
                return;
            }
        }

        _solution = _solution.WithXY(f.Solution().XY());

        // computation of myDone
        _done = true;

        double extraU, extraV;
        //  if(!StrictInside) {
        extraU = 2.0 * tol2d.X;
        extraV = 2.0 * tol2d.Y;
        //  }
        if (_solution.X > inf.X - tol2d.X && _solution.X < inf.X)
        {
            _solution = _solution.WithX(inf.X);
        }
        if (_solution.X > sup.X && _solution.X < sup.X + tol2d.X)
        {
            _solution = _solution.WithX(sup.X);
        }
        if (_solution.Y > inf.Y - tol2d.Y && _solution.Y < inf.Y)
        {
            _solution = _solution.WithY(inf.Y);
        }
        if (_solution.Y > sup.Y && _solution.Y < sup.Y + tol2d.Y)
        {
            _solution = _solution.WithY(sup.Y);
        }
        if (_solution.X < inf.X - extraU || _solution.X > sup.X + extraU
            || _solution.Y < inf.Y - extraV || _solution.Y > sup.Y + extraV)
        {
            _done = false;
        }
        else if (funcTol > 0)
        {
            math_Vector x = new(1, 2, 0.0), fVal = new(1, 2, 0.0);
            x[1] = _solution.X;
            x[2] = _solution.Y;

            f.Value(x, fVal);

            if (!sr.IsDone)
            {
                if ((fVal[1] * fVal[1] + fVal[2] * fVal[2]) > funcTol)
                {
                    _done = false;
                }
            }
        }
    }

    /// <summary>True if the distance is found.</summary>
    public bool IsDone => _done;

    /// <summary>The point of the extremum distance in the free variables.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public Point2D Solution()
    {
        if (!IsDone)
        {
            throw new NotDoneException();
        }
        return _solution;
    }
}
