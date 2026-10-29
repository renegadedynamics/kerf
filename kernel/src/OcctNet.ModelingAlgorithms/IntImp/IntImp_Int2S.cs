// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntImp/IntImp_Int2S.gxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntImp/IntImp_Int2S.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// An intersection point of two parametric surfaces from a nearby point (u1, v1, u2, v2): Newton
/// (math_FunctionSetRoot) on <see cref="IntImp_ZerParFunc{ThePSurface, ThePSurfaceTool}"/> along the best
/// isoparametric, kept within the surfaces' bounds; with the tangent of the intersection line there.
/// </summary>
/// <remarks>
/// OCCT's generic file IntImp_Int2S.gxx, ported once as a generic class over the surfaces and their tool (decisions
/// 202, 337 and 5101). OCCT's macro IntImp_TheFunction is, in every instantiation (IntWalk_TheInt2S, GeomInt's and
/// BRepApprox's TheInt2SOfThePrmPrmSvSurfaces...), IntImp_ZerParFunc over the same surface and tool, so the body
/// names the generic function and <see cref="Function"/> returns it. OCCT's instantiations are sealed subclasses with
/// the constructors forwarded.
/// <para>
/// OCCT's <c>Perform</c> builds its vectors on stack buffers; the port keeps them in the object and reuses them, since
/// <c>Perform</c> runs once per step of a walking line (every element is written before it is read).
/// <see cref="ChangePoint"/> returns a reference to the point (OCCT's non-const reference).
/// </para>
/// </remarks>
public class IntImp_Int2S<ThePSurface, ThePSurfaceTool>
    where ThePSurfaceTool : IIntImp_ZerCSParPSurfaceTool<ThePSurface>
{
    private bool done;
    private bool empty;
    private IntSurf_PntOn2S pint;
    private bool tangent;
    private gp_Dir d3d;
    private gp_Dir2d d2d1;
    private gp_Dir2d d2d2;
    private readonly IntImp_ZerParFunc<ThePSurface, ThePSurfaceTool> myZerParFunc;
    private readonly double tol;
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

    // OCCT's stack buffers of the Perform overloads (see the remarks).
    private readonly math_Vector myBornInf = new(1, 3);
    private readonly math_Vector myBornSup = new(1, 3);
    private readonly math_Vector myTolerance = new(1, 3);
    private readonly math_Vector myUVap = new(1, 3);
    private readonly NCollection_Array1<double> myUvres = new(1, 4);

    /// <summary>
    /// Prepares the search; <see cref="Perform(NCollection_Array1{double}, math_FunctionSetRoot)"/> runs it, with a
    /// solver built on <see cref="Function"/> once for many points.
    /// </summary>
    public IntImp_Int2S(ThePSurface surf1, ThePSurface surf2, double TolTangency)
    {
        done = true;
        empty = true;
        tangent = false;
        myZerParFunc = new IntImp_ZerParFunc<ThePSurface, ThePSurfaceTool>(surf1, surf2);
        tol = TolTangency * TolTangency;
        pint = new IntSurf_PntOn2S();
        d3d = new gp_Dir();
        d2d1 = new gp_Dir2d();
        d2d2 = new gp_Dir2d();

        ua0 = ThePSurfaceTool.FirstUParameter(surf1); //-- ThePSurfaceTool::UIntervalFirst(surf1);
        va0 = ThePSurfaceTool.FirstVParameter(surf1); //-- ThePSurfaceTool::VIntervalFirst(surf1);
        ua1 = ThePSurfaceTool.LastUParameter(surf1); //-- ThePSurfaceTool::UIntervalLast(surf1);
        va1 = ThePSurfaceTool.LastVParameter(surf1); //-- ThePSurfaceTool::VIntervalLast(surf1);

        ub0 = ThePSurfaceTool.FirstUParameter(surf2); //-- ThePSurfaceTool::UIntervalFirst(surf2);
        vb0 = ThePSurfaceTool.FirstVParameter(surf2); //-- ThePSurfaceTool::VIntervalFirst(surf2);
        ub1 = ThePSurfaceTool.LastUParameter(surf2); //-- ThePSurfaceTool::UIntervalLast(surf2);
        vb1 = ThePSurfaceTool.LastVParameter(surf2); //-- ThePSurfaceTool::VIntervalLast(surf2);

        ures1 = ThePSurfaceTool.UResolution(surf1, Precision.Confusion());
        vres1 = ThePSurfaceTool.VResolution(surf1, Precision.Confusion());

        ures2 = ThePSurfaceTool.UResolution(surf2, Precision.Confusion());
        vres2 = ThePSurfaceTool.VResolution(surf2, Precision.Confusion());
    }

    /// <summary>Computes the intersection point near <paramref name="Param"/> (u1, v1, u2, v2).</summary>
    public IntImp_Int2S(NCollection_Array1<double> Param, ThePSurface surf1, ThePSurface surf2, double TolTangency)
        : this(surf1, surf2, TolTangency)
    {
        // OCCT leaves tangent uninitialised here; the field's false is never read before Perform writes it.
        math_FunctionSetRoot Rsnld = new(myZerParFunc, 15); //-- Modif lbr 18 MAI ?????????????
        Perform(Param, Rsnld);
    }

    /// <summary>
    /// Searches the intersection point near <paramref name="Param"/> along the isoparametric
    /// <paramref name="ChoixIso"/>, and returns the best isoparametric to find the next point.
    /// </summary>
    public IntImp_ConstIsoparametric Perform(NCollection_Array1<double> Param, math_FunctionSetRoot Rsnld,
                                             IntImp_ConstIsoparametric ChoixIso)
    {
        math_Vector BornInf = myBornInf, BornSup = myBornSup, Tolerance = myTolerance, UVap = myUVap;
        NCollection_Array1<double> Uvres = myUvres;
        IntImp_ConstIsoparametric BestChoix;

        myZerParFunc.ComputeParameters(ChoixIso, Param, UVap, BornInf, BornSup, Tolerance);
        Rsnld.SetTolerance(Tolerance);
        Rsnld.Perform(myZerParFunc, UVap, BornInf, BornSup);
        BestChoix = ChoixIso;
        if (Rsnld.IsDone())
        {
            if (Math.Abs(myZerParFunc.Root()) <= tol)
            { // distance des 2 points
              // dans la tolerance
                Rsnld.Root(UVap);
                empty = false;
                tangent = myZerParFunc.IsTangent(UVap, Uvres, out BestChoix);
                pint.SetValue(myZerParFunc.Point(), Uvres[1], Uvres[2], Uvres[3], Uvres[4]);
                if (!tangent)
                {
                    d3d = myZerParFunc.Direction();
                    d2d1 = myZerParFunc.DirectionOnS1();
                    d2d2 = myZerParFunc.DirectionOnS2();
                }
            }
            else
            {
                empty = true;
            }
        }
        else
        {
            empty = true;
        }
        return BestChoix;
    }

    /// <summary>
    /// Searches the intersection point near <paramref name="Param"/> along the best isoparametric there, staying
    /// within the surfaces' bounds, and returns the best isoparametric to find the next point.
    /// </summary>
    public IntImp_ConstIsoparametric Perform(NCollection_Array1<double> Param, math_FunctionSetRoot Rsnld)
    {
        Span<gp_Vec> DPUV = stackalloc gp_Vec[4];
        Span<double> Epsuv = stackalloc double[4];
        NCollection_Array1<double> Duv = new(1, 4);
        Span<double> UVd = stackalloc double[4];
        Span<double> UVf = stackalloc double[4];
        Span<IntImp_ConstIsoparametric> ChoixIso = stackalloc IntImp_ConstIsoparametric[4];
        IntImp_ConstIsoparametric BestChoix = IntImp_ComputeTangence.ChoixRef(0);
        ThePSurface Caro1 = myZerParFunc.AuxillarSurface1();
        ThePSurface Caro2 = myZerParFunc.AuxillarSurface2();

        ThePSurfaceTool.D1(Caro1, Param[1], Param[2], out _, out DPUV[0], out DPUV[1]);
        ThePSurfaceTool.D1(Caro2, Param[3], Param[4], out _, out DPUV[2], out DPUV[3]);
        Epsuv[0] = ThePSurfaceTool.UResolution(Caro1, Precision.Confusion());
        Epsuv[1] = ThePSurfaceTool.VResolution(Caro1, Precision.Confusion());
        Epsuv[2] = ThePSurfaceTool.UResolution(Caro2, Precision.Confusion());
        Epsuv[3] = ThePSurfaceTool.VResolution(Caro2, Precision.Confusion());

        for (int j = 0; j <= 3; j++)
        {
            UVd[j] = Param[j + 1];
        }

        empty = true;

        bool Tangent = IntImp_ComputeTangence.ComputeTangence(DPUV, Epsuv, UVd, ChoixIso);
        if (Tangent)
        {
            return BestChoix;
        }

        int i = 0;
        IntImp_ConstIsoparametric CurrentChoix = BestChoix; //-- Modif 17 Mai 93
        while (empty && i <= 3)
        {
            CurrentChoix = Perform(Param, Rsnld, ChoixIso[i]);
            if (!empty)
            {
                BestChoix = CurrentChoix;
            }
            i++;
        }
        if (!empty)
        { // verifier que l on ne deborde pas les frontieres
            pint.Parameters(out Duv[1], out Duv[2], out Duv[3], out Duv[4]);
            UVd[0] = ua0; //-- ThePSurfaceTool::UIntervalFirst(Caro1);
            UVd[1] = va0; //-- ThePSurfaceTool::VIntervalFirst(Caro1);
            UVf[0] = ua1; //-- ThePSurfaceTool::UIntervalLast(Caro1);
            UVf[1] = va1; //-- ThePSurfaceTool::VIntervalLast(Caro1);
            UVd[2] = ub0; //-- ThePSurfaceTool::UIntervalFirst(Caro2);
            UVd[3] = vb0; //-- ubThePSurfaceTool::VIntervalFirst(Caro2);
            UVf[2] = ub1; //-- ThePSurfaceTool::UIntervalLast(Caro2);
            UVf[3] = vb1; //-- ThePSurfaceTool::VIntervalLast(Caro2);

            int Nc, Iiso;
            if (Duv[1] <= UVd[0] - Epsuv[0])
            {
                Duv[1] = UVd[0];
                Nc = 0;
                Iiso = 0;
            }
            else if (Duv[1] >= UVf[0] + Epsuv[0])
            {
                Duv[1] = UVf[0];
                Nc = 0;
                Iiso = 0;
            }
            else if (Duv[2] <= UVd[1] - Epsuv[1])
            {
                Duv[2] = UVd[1];
                Nc = 0;
                Iiso = 1;
            }
            else if (Duv[2] >= UVf[1] + Epsuv[1])
            {
                Duv[2] = UVf[1];
                Nc = 0;
                Iiso = 1;
            }
            else if (Duv[3] <= UVd[2] - Epsuv[2])
            {
                Duv[3] = UVd[2];
                Nc = 2;
                Iiso = 2;
            }
            else if (Duv[3] >= UVf[2] + Epsuv[2])
            {
                Duv[3] = UVf[2];
                Nc = 2;
                Iiso = 2;
            }
            else if (Duv[4] <= UVd[3] - Epsuv[3])
            {
                Duv[4] = UVd[3];
                Nc = 2;
                Iiso = 3;
            }
            else if (Duv[4] >= UVf[3] + Epsuv[3])
            {
                Duv[4] = UVf[3];
                Nc = 2;
                Iiso = 3;
            }
            else
            {
                return BestChoix; // on a gagne
            }
            empty = true;
            BestChoix = IntImp_ComputeTangence.ChoixRef(Iiso); // en attendant
            BestChoix = Perform(Duv, Rsnld, BestChoix);
            if (!empty)
            { // verification si l on ne deborde pas sur le carreau
              // reciproque
                Nc = 3 - Nc;
                if (Duv[Nc] <= UVd[Nc - 1] - Epsuv[Nc - 1])
                {
                    Duv[Nc] = UVd[Nc - 1];
                }
                else if (Duv[Nc] >= UVf[Nc - 1] + Epsuv[Nc - 1])
                {
                    Duv[Nc] = UVf[Nc - 1];
                }
                else if (Duv[Nc + 1] <= UVd[Nc])
                {
                    Nc = Nc + 1;
                    Duv[Nc] = UVd[Nc - 1];
                }
                else if (Duv[Nc + 1] >= UVf[Nc])
                {
                    Nc = Nc + 1;
                    Duv[Nc] = UVf[Nc - 1];
                }
                else
                {
                    return BestChoix;
                }
                empty = true;
                if (Nc == 4)
                {
                    Nc = 0;
                }
                BestChoix = IntImp_ComputeTangence.ChoixRef(Nc); // en attendant
                BestChoix = Perform(Duv, Rsnld, BestChoix);
            }
        }
        return BestChoix;
    }

    /// <summary>True if the creation completed without failure.</summary>
    public bool IsDone() => done;

    /// <summary>True when there is no solution.</summary>
    public bool IsEmpty()
    {
        if (!done)
        {
            throw new StdFail_NotDone(" IntImp_Int2S::IsEmpty() ");
        }
        return empty;
    }

    /// <summary>The intersection point.</summary>
    public IntSurf_PntOn2S Point()
    {
        if (!done)
        {
            throw new StdFail_NotDone(" IntImp_Int2S::Point() ");
        }
        if (empty)
        {
            throw new Standard_DomainError(" IntImp_Int2S::Point() ");
        }
        return pint;
    }

    /// <summary>True if the surfaces are tangent at the intersection point.</summary>
    public bool IsTangent()
    {
        if (!done)
        {
            throw new StdFail_NotDone(" IntImp_Int2S::IsTangent () ");
        }
        if (empty)
        {
            throw new Standard_DomainError(" IntImp_Int2S::IsTangent () ");
        }
        return tangent;
    }

    /// <summary>The tangent of the intersection line.</summary>
    public gp_Dir Direction()
    {
        if (!done)
        {
            throw new StdFail_NotDone(" IntImp_Int2S::Direction () ");
        }
        if (empty)
        {
            throw new Standard_DomainError(" IntImp_Int2S::Direction () ");
        }
        if (tangent)
        {
            throw new StdFail_UndefinedDerivative(" IntImp_Int2S::Direction () ");
        }
        return d3d;
    }

    /// <summary>The tangent in the parametric space of the first surface.</summary>
    public gp_Dir2d DirectionOnS1()
    {
        if (!done)
        {
            throw new StdFail_NotDone(" IntImp_Int2S::DirectionOnS1 () ");
        }
        if (empty)
        {
            throw new Standard_DomainError(" IntImp_Int2S::DirectionOnS1 () ");
        }
        if (tangent)
        {
            throw new StdFail_UndefinedDerivative(" IntImp_Int2S::DirectionOnS1 () ");
        }
        return d2d1;
    }

    /// <summary>The tangent in the parametric space of the second surface.</summary>
    public gp_Dir2d DirectionOnS2()
    {
        if (!done)
        {
            throw new StdFail_NotDone(" IntImp_Int2S::DirectionOnS2 () ");
        }
        if (empty)
        {
            throw new Standard_DomainError(" IntImp_Int2S::DirectionOnS2 () ");
        }
        if (tangent)
        {
            throw new StdFail_UndefinedDerivative(" IntImp_Int2S::DirectionOnS2 () ");
        }
        return d2d2;
    }

    /// <summary>The function the intersection is computed with (to build a solver once for many points).</summary>
    public IntImp_ZerParFunc<ThePSurface, ThePSurfaceTool> Function() => myZerParFunc;

    /// <summary>The intersection point, to change.</summary>
    public ref IntSurf_PntOn2S ChangePoint() => ref pint;
}
