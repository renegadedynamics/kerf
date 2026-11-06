// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GenLocateExtCC.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_LocECC2d.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

namespace Kerf.Engine.Extrema;

/// <summary>
/// The extremum distance between two 2D curves nearest the starting parameters (U0, V0): the root of
/// <see cref="CCLocFOfLocEcc2d"/> found by math_FunctionSetRoot, kept if both functions are below 1e-10 there.
/// </summary>
/// <remarks>The 2D instantiation of OCCT's template Extrema_GenLocateExtCC (Extrema_LocECC2d), computed by its
/// constructor.</remarks>
internal class LocEcc2d
{
    private readonly bool _done;
    private readonly double _sqDist;
    private readonly POnCurv2d _point1 = new();
    private readonly POnCurv2d _point2 = new();

    /// <summary>The extremum between theC1 and theC2 nearest (theU0, theV0), with the parametric tolerances theTolU and
    /// theTolV.</summary>
    public LocEcc2d(Curve2DAdaptorBase c1, Curve2DAdaptorBase c2, double u0, double v0, double tolU,
        double tolV)
    {
        _done = false;
        _sqDist = RealLast();

        double uinf = Curve2dTool.FirstParameter(c1);
        double usup = Curve2dTool.LastParameter(c1);
        double uu;
        if (uinf > usup)
        {
            uu = uinf;
            uinf = usup;
            usup = uu;
        }

        double vinf = Curve2dTool.FirstParameter(c2);
        double vsup = Curve2dTool.LastParameter(c2);
        if (vinf > vsup)
        {
            uu = vinf;
            vinf = vsup;
            vsup = uu;
        }

        CCLocFOfLocEcc2d f = new(c1, c2);
        math_Vector tol = new(1, 2);
        tol[1] = tolU;
        tol[2] = tolV;
        double tolf = 1.0e-10;

        math_Vector start = new(1, 2);
        math_Vector uuinf = new(1, 2);
        math_Vector uusup = new(1, 2);

        start[1] = u0;
        start[2] = v0;

        uuinf[1] = uinf;
        uuinf[2] = vinf;
        uusup[1] = usup;
        uusup[2] = vsup;

        FunctionSetRoot s = new(f, tol);
        s.Perform(f, start, uuinf, uusup);

        if (s.IsDone && f.ExtCount > 0)
        {
            _sqDist = f.SquareDistance(1);
            f.Points(1, out _point1, out _point2);
            start[1] = _point1.Parameter();
            start[2] = _point2.Parameter();
            math_Vector ff = new(1, 2);
            f.Value(start, ff);
            // OCCT compares the signed values, not their absolute values: any negative residual passes
            if ((ff[1] < tolf) && (ff[2] < tolf))
            {
                _done = true;
            }
        }
    }

    /// <summary>True if the distance is found.</summary>
    public bool IsDone => _done;

    /// <summary>The square distance of the extremum.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public double SquareDistance()
    {
        if (!IsDone)
        {
            throw new NotDoneException();
        }
        return _sqDist;
    }

    /// <summary>The points of the extremum, on the first and on the second curve.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public void Point(out POnCurv2d p1, out POnCurv2d p2)
    {
        if (!IsDone)
        {
            throw new NotDoneException();
        }
        p1 = _point1;
        p2 = _point2;
    }
}
