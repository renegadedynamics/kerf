// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GenLocateExtPS.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GenLocateExtPS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;
using Kerf.Geometry;

namespace Kerf.Engine.Extrema;

/// <summary>
/// The extremum of the distance between a point and a surface near a starting point (u0, v0): by the normal
/// projection criteria (math_FunctionSetRoot, then Newton) or the distance criteria (math_BFGS, then math_FRPR).
/// </summary>
/// <remarks>The surface adaptor is kept, as OCCT keeps a reference: it must outlive the algorithm.</remarks>
internal class GenLocateExtPS
{
    // Input.
    private readonly SurfaceAdaptorBase _surf;
    private readonly double _tolU, _tolV;

    // State.
    private bool _done;

    // Result.
    private double _sqDist;
    private POnSurf _point = new();

    private static void CorrectTol(double u0, double v0, math_Vector theTol)
    {
        // Correct tolerance for large values of UV parameters
        double tolRef = global::Kerf.Engine.Foundation.Precision.PConfusion();
        double epsRef = Epsilon(1.0);
        double epsu = Epsilon(u0);
        const double tolog10 = 0.43429;
        if (epsu > epsRef)
        {
            int n = RealToInt(tolog10 * Math.Log(epsu / epsRef) + 1) + 1;
            int i;
            double tol = tolRef;
            for (i = 1; i <= n; ++i)
            {
                tol *= 10.0;
            }
            theTol[1] = Max(theTol[1], tol);
        }
        double epsv = Epsilon(v0);
        if (epsv > epsRef)
        {
            int n = RealToInt(tolog10 * Math.Log(epsv / epsRef) + 1) + 1;
            int i;
            double tol = tolRef;
            for (i = 1; i <= n; ++i)
            {
                tol *= 10.0;
            }
            theTol[2] = Max(theTol[2], tol);
        }
    }

    /// <summary>
    /// True if (theU0, theV0) is a local minimum of the square distance between theP and theS(U, V) over a small area
    /// around it.
    /// </summary>
    public static bool IsMinDist(in Point3D p, SurfaceAdaptorBase s, double u0, double v0)
    {
        double du = Max(s.UResolution(10.0 * global::Kerf.Engine.Foundation.Precision.Confusion()), 10.0 * global::Kerf.Engine.Foundation.Precision.PConfusion());
        double dv = Max(s.VResolution(10.0 * global::Kerf.Engine.Foundation.Precision.Confusion()), 10.0 * global::Kerf.Engine.Foundation.Precision.PConfusion());
        double u, v;
        Point3D p0 = s.Value(u0, v0);
        double d0 = p.SquareDistanceTo(p0);
        int iu, iv;
        for (iu = -1; iu <= 1; ++iu)
        {
            u = u0 + iu * du;
            if (!s.IsUPeriodic)
            {
                u = Max(u, s.FirstUParameter());
                u = Min(u, s.LastUParameter());
            }
            for (iv = -1; iv <= 1; ++iv)
            {
                if (iu == 0 && iv == 0)
                {
                    continue;
                }

                v = v0 + iv * dv;
                if (!s.IsVPeriodic)
                {
                    v = Max(v, s.FirstVParameter());
                    v = Min(v, s.LastVParameter());
                }
                double d = p.SquareDistanceTo(s.Value(u, v));
                if (d < d0)
                {
                    return false;
                }
            }
        }
        return true;
    }

    public GenLocateExtPS(SurfaceAdaptorBase s, double tolU = 1e-7 * 0.01,
        double tolV = 1e-7 * 0.01) // Precision::PConfusion() (decision 129)
    {
        _surf = s;
        _tolU = tolU;
        _tolV = tolV;
        _done = false;
        _sqDist = -1.0;
    }

    /// <summary>
    /// Computes the extremum near (theU0, theV0): normal projection criteria if <paramref name="isDistanceCriteria"/>
    /// is false, distance criteria otherwise.
    /// </summary>
    public void Perform(in Point3D p, double u0, double v0, bool isDistanceCriteria = false)
    {
        _done = false;

        // Prepare initial data structures.
        math_Vector tol = new(1, 2), start = new(1, 2), boundInf = new(1, 2), boundSup = new(1, 2);

        // Tolerance.
        tol[1] = _tolU;
        tol[2] = _tolV;

        // Initial solution approximation.
        start[1] = u0;
        start[2] = v0;

        // Borders.
        boundInf[1] = _surf.FirstUParameter();
        boundInf[2] = _surf.FirstVParameter();
        boundSup[1] = _surf.LastUParameter();
        boundSup[2] = _surf.LastVParameter();

        if (isDistanceCriteria)
        {
            // Distance criteria.
            double relTol = 1.0e-8;
            math_Vector resPnt = new(1, 2);

            FuncPSDist f = new(_surf, p);

            Bfgs solver = new(2, relTol);
            solver.Perform(f, start);

            if (!solver.IsDone)
            {
                // Try another method
                Frpr solver1 = new(f, relTol);
                solver1.Perform(f, start);
                if (!solver1.IsDone)
                {
                    return;
                }
                solver1.Location(resPnt);
                _sqDist = solver1.Minimum();
            }
            else
            {
                solver.Location(resPnt);
                _sqDist = solver.Minimum();
            }

            _point.SetParameters(resPnt[1], resPnt[2], _surf.Value(resPnt[1], resPnt[2]));
            _done = true;
        }
        else
        {
            // Normal projection criteria.
            FuncPSNorm f = new(p, _surf);

            if (_surf.GetType() == SurfaceKind.BSplineSurface)
            {
                tol[1] = _tolU;
                tol[2] = _tolV;
                CorrectTol(u0, v0, tol);
            }

            bool isCorrectTol = (Math.Abs(tol[1] - _tolU) > global::Kerf.Engine.Foundation.Precision.PConfusion()
                                 || Math.Abs(tol[2] - _tolV) > global::Kerf.Engine.Foundation.Precision.PConfusion());

            FunctionSetRoot sr = new(f, tol);
            sr.Perform(f, start, boundInf, boundSup);

            if (!sr.IsDone || isCorrectTol)
            {
                if (isCorrectTol)
                {
                    tol[1] = _tolU;
                    tol[2] = _tolV;
                }
                NewtonFunctionSetRoot nsr = new(f, tol, global::Kerf.Engine.Foundation.Precision.Confusion());
                nsr.Perform(f, start, boundInf, boundSup);
                if (!sr.IsDone && !nsr.IsDone)
                {
                    return;
                }
            }

            double nbExt = f.ExtCount;
            _sqDist = f.SquareDistance(1);
            _point = f.Point(1);
            int i;
            for (i = 2; i <= nbExt; ++i)
            {
                if (f.SquareDistance(i) < _sqDist)
                {
                    _sqDist = f.SquareDistance(i);
                    _point = f.Point(i);
                }
            }
            _done = true;
        }
    }

    /// <summary>True if the distance is found.</summary>
    public bool IsDone => _done;

    /// <summary>The square distance of the extremum.</summary>
    public double SquareDistance()
    {
        if (!IsDone)
        {
            throw new NotDoneException();
        }
        return _sqDist;
    }

    /// <summary>The point of the extremum.</summary>
    public POnSurf Point()
    {
        if (!IsDone)
        {
            throw new NotDoneException();
        }
        return _point;
    }
}
