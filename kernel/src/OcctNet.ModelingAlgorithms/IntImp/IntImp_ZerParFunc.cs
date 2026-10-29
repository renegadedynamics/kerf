// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntImp/IntImp_ZerParFunc.gxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntImp/IntImp_ZerParFunc.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.IntImp_ConstIsoparametric;

namespace Occt;

/// <summary>
/// The function whose zeros are the intersection points of two parametric surfaces along an isoparametric of one of
/// them: F = S1(u1, v1) - S2(u2, v2) with one of the four parameters fixed, three equations in the other three, with
/// its derivatives, for math_FunctionSetRoot; and, at a root, the tangent of the intersection line and the best
/// isoparametric to march along it.
/// </summary>
/// <remarks>
/// OCCT's generic file IntImp_ZerParFunc.gxx, ported once as a generic class over the surfaces and their tool
/// (decisions 202, 337 and 5101). OCCT's instantiations (IntWalk_TheFunctionOfTheInt2S, GeomInt's and BRepApprox's
/// TheFunctionOfTheInt2S...) are sealed subclasses, or are the generic itself where only
/// <see cref="IntImp_Int2S{ThePSurface, ThePSurfaceTool}"/> builds them. OCCT keeps pointers to the caller's surface
/// handles; the port keeps the surfaces (adaptors, by reference, as the handles).
/// </remarks>
public class IntImp_ZerParFunc<ThePSurface, ThePSurfaceTool> : math_FunctionSetWithDerivatives
    where ThePSurfaceTool : IIntImp_ZerCSParPSurfaceTool<ThePSurface>
{
    private readonly ThePSurface surf1;
    private readonly ThePSurface surf2;
    private gp_Pnt pntsol1;
    private gp_Pnt pntsol2;
    private readonly double[] f = new double[3];
    // OCCT's field compute is written, never read: left out (decision 143).
    private bool tangent;
    private readonly double[] tgduv = new double[4];
    private readonly gp_Vec[] dpuv = [new gp_Vec(), new gp_Vec(), new gp_Vec(), new gp_Vec()];
    private IntImp_ConstIsoparametric chxIso;
    private double paramConst;
    private readonly double ua0;
    private readonly double va0;
    private readonly double ua1;
    private readonly double va1;
    private readonly double ub0;
    private readonly double vb0;
    private readonly double ub1;
    private readonly double vb1;
    private readonly double ures1;
    private readonly double ures2;
    private readonly double vres1;
    private readonly double vres2;

    public IntImp_ZerParFunc(ThePSurface S1, ThePSurface S2)
    {
        tangent = false;
        paramConst = 0.0;
        surf1 = S1;
        surf2 = S2;

        ua0 = ThePSurfaceTool.FirstUParameter(surf1); //-- ThePSurfaceTool::UIntervalFirst(surf1);
        va0 = ThePSurfaceTool.FirstVParameter(surf1); //-- ThePSurfaceTool::VIntervalFirst(surf1);
        ua1 = ThePSurfaceTool.LastUParameter(surf1); //-- ThePSurfaceTool::UIntervalLast(surf1);
        va1 = ThePSurfaceTool.LastVParameter(surf1); //-- ThePSurfaceTool::VIntervalLast(surf1);

        ub0 = ThePSurfaceTool.FirstUParameter(surf2); //-- ThePSurfaceTool::UIntervalFirst(SURF2);
        vb0 = ThePSurfaceTool.FirstVParameter(surf2); //-- ThePSurfaceTool::VIntervalFirst(surf2);
        ub1 = ThePSurfaceTool.LastUParameter(surf2); //-- ThePSurfaceTool::UIntervalLast(surf2);
        vb1 = ThePSurfaceTool.LastVParameter(surf2); //-- ThePSurfaceTool::VIntervalLast(surf2);

        ures1 = ThePSurfaceTool.UResolution(surf1, Precision.Confusion());
        vres1 = ThePSurfaceTool.VResolution(surf1, Precision.Confusion());

        ures2 = ThePSurfaceTool.UResolution(surf2, Precision.Confusion());
        vres2 = ThePSurfaceTool.VResolution(surf2, Precision.Confusion());

        // memset(f, 0, sizeof(f)): the array is new
        tangent = false;
        pntsol1 = new gp_Pnt();
        pntsol2 = new gp_Pnt();
    }

    public override int NbVariables() => 3;

    public override int NbEquations() => 3;

    public override bool Value(math_Vector X, math_Vector F)
    {
        switch (chxIso)
        {
            case IntImp_UIsoparametricOnCaro1:
            {
                pntsol1 = ThePSurfaceTool.Value(surf1, paramConst, X[1]);
                pntsol2 = ThePSurfaceTool.Value(surf2, X[2], X[3]);
            }
            break;
            case IntImp_VIsoparametricOnCaro1:
            {
                pntsol1 = ThePSurfaceTool.Value(surf1, X[1], paramConst);
                pntsol2 = ThePSurfaceTool.Value(surf2, X[2], X[3]);
            }
            break;
            case IntImp_UIsoparametricOnCaro2:
            {
                pntsol1 = ThePSurfaceTool.Value(surf1, X[1], X[2]);
                pntsol2 = ThePSurfaceTool.Value(surf2, paramConst, X[3]);
            }
            break;
            case IntImp_VIsoparametricOnCaro2:
            {
                pntsol1 = ThePSurfaceTool.Value(surf1, X[1], X[2]);
                pntsol2 = ThePSurfaceTool.Value(surf2, X[3], paramConst);
            }
            break;
        }

        f[0] = F[1] = pntsol1.X() - pntsol2.X();
        f[1] = F[2] = pntsol1.Y() - pntsol2.Y();
        f[2] = F[3] = pntsol1.Z() - pntsol2.Z();

        return true;
    }

    public override bool Derivatives(math_Vector X, math_Matrix D)
    {
        FillDerivatives(X, D);
        return true;
    }

    public override bool Values(math_Vector X, math_Vector F, math_Matrix D)
    {
        FillDerivatives(X, D);

        f[0] = F[1] = pntsol1.X() - pntsol2.X();
        f[1] = F[2] = pntsol1.Y() - pntsol2.Y();
        f[2] = F[3] = pntsol1.Z() - pntsol2.Z();

        return true;
    }

    // The switch that OCCT writes out twice, verbatim, in Derivatives and Values (decision 340's approach).
    private void FillDerivatives(math_Vector X, math_Matrix D)
    {
        switch (chxIso)
        {
            case IntImp_UIsoparametricOnCaro1:
            {
                ThePSurfaceTool.D1(surf1, paramConst, X[1], out pntsol1, out dpuv[0], out dpuv[1]);
                ThePSurfaceTool.D1(surf2, X[2], X[3], out pntsol2, out dpuv[2], out dpuv[3]);
                D[1, 1] = dpuv[1].X();
                D[1, 2] = -dpuv[2].X();
                D[1, 3] = -dpuv[3].X();
                D[2, 1] = dpuv[1].Y();
                D[2, 2] = -dpuv[2].Y();
                D[2, 3] = -dpuv[3].Y();
                D[3, 1] = dpuv[1].Z();
                D[3, 2] = -dpuv[2].Z();
                D[3, 3] = -dpuv[3].Z();
            }
            break;
            case IntImp_VIsoparametricOnCaro1:
            {
                ThePSurfaceTool.D1(surf1, X[1], paramConst, out pntsol1, out dpuv[0], out dpuv[1]);
                ThePSurfaceTool.D1(surf2, X[2], X[3], out pntsol2, out dpuv[2], out dpuv[3]);
                D[1, 1] = dpuv[0].X();
                D[1, 2] = -dpuv[2].X();
                D[1, 3] = -dpuv[3].X();
                D[2, 1] = dpuv[0].Y();
                D[2, 2] = -dpuv[2].Y();
                D[2, 3] = -dpuv[3].Y();
                D[3, 1] = dpuv[0].Z();
                D[3, 2] = -dpuv[2].Z();
                D[3, 3] = -dpuv[3].Z();
            }
            break;
            case IntImp_UIsoparametricOnCaro2:
            {
                ThePSurfaceTool.D1(surf1, X[1], X[2], out pntsol1, out dpuv[0], out dpuv[1]);
                ThePSurfaceTool.D1(surf2, paramConst, X[3], out pntsol2, out dpuv[2], out dpuv[3]);
                D[1, 1] = dpuv[0].X();
                D[1, 2] = dpuv[1].X();
                D[1, 3] = -dpuv[3].X();
                D[2, 1] = dpuv[0].Y();
                D[2, 2] = dpuv[1].Y();
                D[2, 3] = -dpuv[3].Y();
                D[3, 1] = dpuv[0].Z();
                D[3, 2] = dpuv[1].Z();
                D[3, 3] = -dpuv[3].Z();
            }
            break;
            case IntImp_VIsoparametricOnCaro2:
            {
                ThePSurfaceTool.D1(surf1, X[1], X[2], out pntsol1, out dpuv[0], out dpuv[1]);
                ThePSurfaceTool.D1(surf2, X[3], paramConst, out pntsol2, out dpuv[2], out dpuv[3]);
                D[1, 1] = dpuv[0].X();
                D[1, 2] = dpuv[1].X();
                D[1, 3] = -dpuv[2].X();
                D[2, 1] = dpuv[0].Y();
                D[2, 2] = dpuv[1].Y();
                D[2, 3] = -dpuv[2].Y();
                D[3, 1] = dpuv[0].Z();
                D[3, 2] = dpuv[1].Z();
                D[3, 3] = -dpuv[2].Z();
            }
            break;
        }
    }

    /// <summary>
    /// Fixes the isoparametric <paramref name="ChoixIso"/> at its value in <paramref name="Param"/> (u1, v1, u2, v2) and
    /// gives the starting point, the bounds (the surfaces' bounds widened by 1% of their range) and the tolerances of
    /// the three other parameters.
    /// </summary>
    public void ComputeParameters(IntImp_ConstIsoparametric ChoixIso, NCollection_Array1<double> Param,
                                  math_Vector UVap, math_Vector BornInf, math_Vector BornSup, math_Vector Tolerance)
    {
        chxIso = ChoixIso;
        switch (chxIso)
        {
            case IntImp_UIsoparametricOnCaro1:
            {
                paramConst = Param[1];
                UVap[1] = Param[2];
                UVap[2] = Param[3];
                UVap[3] = Param[4];

                BornInf[1] = va0;
                BornSup[1] = va1;
                BornInf[2] = ub0;
                BornInf[3] = vb0;
                BornSup[2] = ub1;
                BornSup[3] = vb1;

                Tolerance[1] = vres1;
                Tolerance[2] = ures2;
                Tolerance[3] = vres2;
            }
            break;
            case IntImp_VIsoparametricOnCaro1:
            {
                paramConst = Param[2];
                UVap[1] = Param[1];
                UVap[2] = Param[3];
                UVap[3] = Param[4];

                BornInf[1] = ua0;
                BornSup[1] = ua1;
                BornInf[2] = ub0;
                BornSup[2] = ub1;
                BornInf[3] = vb0;
                BornSup[3] = vb1;

                Tolerance[1] = ures1;
                Tolerance[2] = ures2;
                Tolerance[3] = vres2;
            }
            break;
            case IntImp_UIsoparametricOnCaro2:
            {
                paramConst = Param[3];
                UVap[1] = Param[1];
                UVap[2] = Param[2];
                UVap[3] = Param[4];

                BornInf[1] = ua0;
                BornSup[1] = ua1;
                BornInf[2] = va0;
                BornSup[2] = va1;
                BornInf[3] = vb0;
                BornSup[3] = vb1;

                Tolerance[1] = ures1;
                Tolerance[2] = vres1;
                Tolerance[3] = vres2;
            }
            break;
            case IntImp_VIsoparametricOnCaro2:
            {
                paramConst = Param[4];
                UVap[1] = Param[1];
                UVap[2] = Param[2];
                UVap[3] = Param[3];

                BornInf[1] = ua0;
                BornSup[1] = ua1;
                BornInf[2] = va0;
                BornSup[2] = va1;
                BornInf[3] = ub0;
                BornSup[3] = ub1;

                Tolerance[1] = ures1;
                Tolerance[2] = vres1;
                Tolerance[3] = ures2;
            }
            break;
        }

        double Incr1 = (BornSup[1] - BornInf[1]) * 0.01;
        double Incr2 = (BornSup[2] - BornInf[2]) * 0.01;
        double Incr3 = (BornSup[3] - BornInf[3]) * 0.01;
        BornInf[1] -= Incr1;
        BornSup[1] += Incr1;
        BornInf[2] -= Incr2;
        BornSup[2] += Incr2;
        BornInf[3] -= Incr3;
        BornSup[3] += Incr3;
    }

    /// <summary>The sum of the squares of the last evaluated values.</summary>
    public double Root() => f[0] * f[0] + f[1] * f[1] + f[2] * f[2];

    /// <summary>The middle of the two last evaluated points.</summary>
    public gp_Pnt Point() => new gp_Pnt((pntsol1.XYZ() + pntsol2.XYZ()) / 2.0);

    /// <summary>
    /// Writes the solution <paramref name="UVap"/> into <paramref name="Param"/> (u1, v1, u2, v2), and returns true
    /// if the surfaces are tangent there; otherwise computes the tangent and the best isoparametric to march along
    /// it (<paramref name="BestChoix"/>).
    /// </summary>
    public bool IsTangent(math_Vector UVap, NCollection_Array1<double> Param, out IntImp_ConstIsoparametric BestChoix)
    {
        switch (chxIso)
        {
            case IntImp_UIsoparametricOnCaro1:
            {
                Param[1] = paramConst;
                Param[2] = UVap[1];
                Param[3] = UVap[2];
                Param[4] = UVap[3];
            }
            break;
            case IntImp_VIsoparametricOnCaro1:
            {
                Param[2] = paramConst;
                Param[1] = UVap[1];
                Param[3] = UVap[2];
                Param[4] = UVap[3];
            }
            break;
            case IntImp_UIsoparametricOnCaro2:
            {
                Param[3] = paramConst;
                Param[1] = UVap[1];
                Param[2] = UVap[2];
                Param[4] = UVap[3];
            }
            break;
            case IntImp_VIsoparametricOnCaro2:
            {
                Param[4] = paramConst;
                Param[1] = UVap[1];
                Param[2] = UVap[2];
                Param[3] = UVap[3];
            }
            break;
        }
        Span<IntImp_ConstIsoparametric> TabIso = stackalloc IntImp_ConstIsoparametric[4];
        Span<double> EpsUV = stackalloc double[4];
        EpsUV[0] = ures1;
        EpsUV[1] = vres1;
        EpsUV[2] = ures2;
        EpsUV[3] = vres2;

        tangent = IntImp_ComputeTangence.ComputeTangence(dpuv, EpsUV, tgduv, TabIso);
        if (!tangent)
        {
            chxIso = TabIso[0];
        }
        BestChoix = chxIso;
        return tangent;
    }

    /// <summary>The tangent of the intersection line. Throws StdFail_UndefinedDerivative at a tangent point.</summary>
    public gp_Dir Direction()
    {
        if (tangent)
        {
            throw new StdFail_UndefinedDerivative();
        }
        return new gp_Dir(tgduv[0] * dpuv[0].XYZ() + tgduv[1] * dpuv[1].XYZ());
    }

    /// <summary>The tangent in the parametric space of the first surface.</summary>
    public gp_Dir2d DirectionOnS1()
    {
        if (tangent)
        {
            throw new StdFail_UndefinedDerivative();
        }
        return new gp_Dir2d(tgduv[0], tgduv[1]);
    }

    /// <summary>The tangent in the parametric space of the second surface.</summary>
    public gp_Dir2d DirectionOnS2()
    {
        if (tangent)
        {
            throw new StdFail_UndefinedDerivative();
        }
        return new gp_Dir2d(tgduv[2], tgduv[3]);
    }

    public ThePSurface AuxillarSurface1() => surf1;

    public ThePSurface AuxillarSurface2() => surf2;
}
