// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GenLocateExtSS.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GenLocateExtSS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;
using Kerf.Engine;

namespace Kerf.Engine.Extrema;

/// <summary>
/// The extremum of the distance between two surfaces near starting parameters (U1, V1) on S1 and (U2, V2) on S2: a
/// zero of the gradient of F = distance(S1(u1, v1), S2(u2, v2)), found by math_FunctionSetRoot.
/// </summary>
internal class GenLocateExtSS
{
    private bool _done;
    private double _sqDist;
    private POnSurf _point1 = new();
    private POnSurf _point2 = new();

    public GenLocateExtSS()
    {
        _done = false;
        _sqDist = RealLast();
    }

    /// <summary>Computes the extremum near (U1, V1) on S1 and (U2, V2) on S2 (see <see cref="Perform"/>).</summary>
    public GenLocateExtSS(SurfaceAdaptorBase s1, SurfaceAdaptorBase s2, double u1, double v1, double u2,
        double v2, double tol1, double tol2)
    {
        Perform(s1, s2, u1, v1, u2, v2, tol1, tol2);
    }

    /// <summary>
    /// Computes the extremum near (U1, V1) on S1 and (U2, V2) on S2, with the tolerances Tol1 on S1's parameters and
    /// Tol2 on S2's.
    /// </summary>
    public void Perform(SurfaceAdaptorBase s1, SurfaceAdaptorBase s2, double u1, double v1, double u2, double v2,
        double tol1, double tol2)
    {
        _done = false;

        double uinf1, usup1, vinf1, vsup1;
        uinf1 = s1.FirstUParameter();
        usup1 = s1.LastUParameter();
        vinf1 = s1.FirstVParameter();
        vsup1 = s1.LastVParameter();

        double uinf2, usup2, vinf2, vsup2;
        uinf2 = s2.FirstUParameter();
        usup2 = s2.LastUParameter();
        vinf2 = s2.FirstVParameter();
        vsup2 = s2.LastVParameter();

        FuncExtSS f = new(s1, s2);
        math_Vector tol = new(1, 4), start = new(1, 4), bInf = new(1, 4), bSup = new(1, 4);

        tol[1] = tol1;
        tol[2] = tol1;
        tol[3] = tol2;
        tol[4] = tol2;

        start[1] = u1;
        start[2] = v1;
        start[3] = u2;
        start[4] = v2;

        bInf[1] = uinf1;
        bInf[2] = vinf1;
        bInf[3] = uinf2;
        bInf[4] = vinf2;
        bSup[1] = usup1;
        bSup[2] = vsup1;
        bSup[3] = usup2;
        bSup[4] = vsup2;

        FunctionSetRoot sr = new(f, tol);
        sr.Perform(f, start, bInf, bSup);
        if (!sr.IsDone)
        {
            return;
        }

        _sqDist = f.SquareDistance(1);
        _point1 = f.PointOnS1(1);
        _point2 = f.PointOnS2(1);
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

    /// <summary>The point of the extremum on S1.</summary>
    public POnSurf PointOnS1()
    {
        if (!IsDone)
        {
            throw new NotDoneException();
        }
        return _point1;
    }

    /// <summary>The point of the extremum on S2.</summary>
    public POnSurf PointOnS2()
    {
        if (!IsDone)
        {
            throw new NotDoneException();
        }
        return _point2;
    }
}
