// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GenLocateExtCS.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GenLocateExtCS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;
using Occt;

namespace Kerf.Engine.Extrema;

/// <summary>
/// The extremum of the distance between a curve and a surface near starting parameters T on the curve and (U, V)
/// on the surface: a zero of the gradient of F(t, u, v) = distance(C(t), S(u, v)), found by math_FunctionSetRoot.
/// </summary>
public class GenLocateExtCS
{
    private bool _done;
    private double _sqDist;
    private POnCurv _point1 = new();
    private POnSurf _point2 = new();

    public GenLocateExtCS()
    {
        _done = false;
        _sqDist = RealLast();
    }

    /// <summary>Computes the extremum near T on C and (U, V) on S (see <see cref="Perform"/>).</summary>
    public GenLocateExtCS(CurveAdaptorBase c, SurfaceAdaptorBase s, double t, double u, double v, double tol1,
        double tol2)
    {
        Perform(c, s, t, u, v, tol1, tol2);
    }

    /// <summary>Computes the extremum near T on C and (U, V) on S, with tolerances Tol1 on t and Tol2 on u and v.</summary>
    public void Perform(CurveAdaptorBase c, SurfaceAdaptorBase s, double t, double u, double v, double tol1, double tol2)
    {
        _done = false;

        double tinf, tsup;
        tinf = c.FirstParameter();
        tsup = c.LastParameter();

        double uinf, usup, vinf, vsup;
        uinf = s.FirstUParameter();
        usup = s.LastUParameter();
        vinf = s.FirstVParameter();
        vsup = s.LastVParameter();

        FuncExtCS f = new(c, s);
        math_Vector tol = new(1, 3), start = new(1, 3), bInf = new(1, 3), bSup = new(1, 3);
        tol[1] = tol1;
        tol[2] = tol2;
        tol[3] = tol2;

        start[1] = t;
        start[2] = u;
        start[3] = v;

        bInf[1] = tinf;
        bInf[2] = uinf;
        bInf[3] = vinf;

        bSup[1] = tsup;
        bSup[2] = usup;
        bSup[3] = vsup;

        FunctionSetRoot sr = new(f, tol);
        sr.Perform(f, start, bInf, bSup);
        if (!sr.IsDone)
        {
            return;
        }

        _sqDist = f.SquareDistance(1);
        _point1 = f.PointOnCurve(1);
        _point2 = f.PointOnSurface(1);
        _done = true;
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

    /// <summary>The point of the extremum on the curve.</summary>
    public POnCurv PointOnCurve()
    {
        if (!IsDone)
        {
            throw new NotDoneException();
        }
        return _point1;
    }

    /// <summary>The point of the extremum on the surface.</summary>
    public POnSurf PointOnSurface()
    {
        if (!IsDone)
        {
            throw new NotDoneException();
        }
        return _point2;
    }
}
