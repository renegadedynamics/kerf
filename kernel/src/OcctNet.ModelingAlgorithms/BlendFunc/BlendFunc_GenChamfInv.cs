// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_GenChamfInv.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_GenChamfInv.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2015-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

namespace Occt;

/// <summary>The function of a general chamfer with one end on a surface's boundary.</summary>
public abstract class BlendFunc_GenChamfInv : Blend_FuncInv
{
    protected readonly Adaptor3d_Surface surf1;
    protected readonly Adaptor3d_Surface surf2;
    protected readonly Adaptor3d_Curve curv;
    protected Adaptor2d_Curve2d? csurf;
    protected int choix;
    protected bool first;

    protected BlendFunc_GenChamfInv(Adaptor3d_Surface S1, Adaptor3d_Surface S2, Adaptor3d_Curve C)
    {
        surf1 = S1;
        surf2 = S2;
        curv = C;
        choix = 0;
        first = false;
    }

    /// <summary>Returns 4.</summary>
    public override int NbEquations()
    {
        return 4;
    }

    public override void Set(bool OnFirst, Adaptor2d_Curve2d C)
    {
        first = OnFirst;
        csurf = C;
    }

    public override void GetTolerance(math_Vector Tolerance, double Tol)
    {
        Tolerance[1] = csurf!.Resolution(Tol);
        Tolerance[2] = curv.Resolution(Tol);
        if (first)
        {
            Tolerance[3] = surf2.UResolution(Tol);
            Tolerance[4] = surf2.VResolution(Tol);
        }
        else
        {
            Tolerance[3] = surf1.UResolution(Tol);
            Tolerance[4] = surf1.VResolution(Tol);
        }
    }

    public override void GetBounds(math_Vector InfBound, math_Vector SupBound)
    {
        InfBound[1] = csurf!.FirstParameter();
        InfBound[2] = curv.FirstParameter();
        SupBound[1] = csurf.LastParameter();
        SupBound[2] = curv.LastParameter();

        if (first)
        {
            InfBound[3] = surf2.FirstUParameter();
            InfBound[4] = surf2.FirstVParameter();
            SupBound[3] = surf2.LastUParameter();
            SupBound[4] = surf2.LastVParameter();
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(InfBound[3]) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(SupBound[3]))
            {
                double range = (SupBound[3] - InfBound[3]);
                InfBound[3] -= range;
                SupBound[3] += range;
            }

            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(InfBound[4]) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(SupBound[4]))
            {
                double range = (SupBound[4] - InfBound[4]);
                InfBound[4] -= range;
                SupBound[4] += range;
            }
        }
        else
        {
            InfBound[3] = surf1.FirstUParameter();
            InfBound[4] = surf1.FirstVParameter();
            SupBound[3] = surf1.LastUParameter();
            SupBound[4] = surf1.LastVParameter();
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(InfBound[3]) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(SupBound[3]))
            {
                double range = (SupBound[3] - InfBound[3]);
                InfBound[3] -= range;
                SupBound[3] += range;
            }

            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(InfBound[4]) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(SupBound[4]))
            {
                double range = (SupBound[4] - InfBound[4]);
                InfBound[4] -= range;
                SupBound[4] += range;
            }
        }
    }

    /// <summary>Value then Derivatives; always true.</summary>
    public override bool Values(math_Vector X, math_Vector F, Matrix D)
    {
        Value(X, F);
        Derivatives(X, D);
        return true;
    }

    /// <summary>Sets the distances and the "quadrant".</summary>
    public abstract void Set(double Dist1, double Dist2, int Choix);
}
