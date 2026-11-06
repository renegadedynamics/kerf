// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_GenChamfInv.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_GenChamfInv.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2015-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Fillets.Blend;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

namespace Kerf.Engine.Fillets.BlendFunctions;

/// <summary>The function of a general chamfer with one end on a surface's boundary.</summary>
internal abstract class GenChamfInv : FuncInv
{
    protected readonly SurfaceAdaptorBase _surf1;
    protected readonly SurfaceAdaptorBase _surf2;
    protected readonly CurveAdaptorBase _curv;
    protected Curve2DAdaptorBase? _csurf;
    protected int _choix;
    protected bool _first;

    protected GenChamfInv(SurfaceAdaptorBase s1, SurfaceAdaptorBase s2, CurveAdaptorBase c)
    {
        _surf1 = s1;
        _surf2 = s2;
        _curv = c;
        _choix = 0;
        _first = false;
    }

    /// <summary>Returns 4.</summary>
    public override int NbEquations()
    {
        return 4;
    }

    public override void Set(bool onFirst, Curve2DAdaptorBase c)
    {
        _first = onFirst;
        _csurf = c;
    }

    public override void GetTolerance(math_Vector tolerance, double tol)
    {
        tolerance[1] = _csurf!.Resolution(tol);
        tolerance[2] = _curv.Resolution(tol);
        if (_first)
        {
            tolerance[3] = _surf2.UResolution(tol);
            tolerance[4] = _surf2.VResolution(tol);
        }
        else
        {
            tolerance[3] = _surf1.UResolution(tol);
            tolerance[4] = _surf1.VResolution(tol);
        }
    }

    public override void GetBounds(math_Vector infBound, math_Vector supBound)
    {
        infBound[1] = _csurf!.FirstParameter();
        infBound[2] = _curv.FirstParameter();
        supBound[1] = _csurf.LastParameter();
        supBound[2] = _curv.LastParameter();

        if (_first)
        {
            infBound[3] = _surf2.FirstUParameter();
            infBound[4] = _surf2.FirstVParameter();
            supBound[3] = _surf2.LastUParameter();
            supBound[4] = _surf2.LastVParameter();
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(infBound[3]) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(supBound[3]))
            {
                double range = (supBound[3] - infBound[3]);
                infBound[3] -= range;
                supBound[3] += range;
            }

            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(infBound[4]) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(supBound[4]))
            {
                double range = (supBound[4] - infBound[4]);
                infBound[4] -= range;
                supBound[4] += range;
            }
        }
        else
        {
            infBound[3] = _surf1.FirstUParameter();
            infBound[4] = _surf1.FirstVParameter();
            supBound[3] = _surf1.LastUParameter();
            supBound[4] = _surf1.LastVParameter();
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(infBound[3]) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(supBound[3]))
            {
                double range = (supBound[3] - infBound[3]);
                infBound[3] -= range;
                supBound[3] += range;
            }

            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(infBound[4]) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(supBound[4]))
            {
                double range = (supBound[4] - infBound[4]);
                infBound[4] -= range;
                supBound[4] += range;
            }
        }
    }

    /// <summary>Value then Derivatives; always true.</summary>
    public override bool Values(math_Vector x, math_Vector f, Matrix d)
    {
        Value(x, f);
        Derivatives(x, d);
        return true;
    }

    /// <summary>Sets the distances and the "quadrant".</summary>
    public abstract void Set(double dist1, double dist2, int choix);
}
