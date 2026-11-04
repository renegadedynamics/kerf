// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntImp/IntImp_Int2S.gxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntImp/IntImp_Int2S.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Intersection.Walking;
using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Intersection.Implicit;

/// <summary>
/// An intersection point of two parametric surfaces from a nearby point (u1, v1, u2, v2): Newton
/// (math_FunctionSetRoot) on <see cref="ZerParFunc{ThePSurface, ThePSurfaceTool}"/> along the best
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
internal class Int2S<ThePSurface, ThePSurfaceTool>
    where ThePSurfaceTool : IZerCSParPSurfaceTool<ThePSurface>
{
    private bool _done;
    private bool _empty;
    private PntOn2S _pint;
    private bool _tangent;
    private Direction3D _d3d;
    private Direction2D _d2d1;
    private Direction2D _d2d2;
    private readonly ZerParFunc<ThePSurface, ThePSurfaceTool> _zerParFunc;
    private readonly double _tol;
    private readonly double _ua0;
    private readonly double _va0;
    private readonly double _ua1;
    private readonly double _va1;
    private readonly double _ub0;
    private readonly double _vb0;
    private readonly double _ub1;
    private readonly double _vb1;
    private readonly double _ures1;
    private readonly double _ures2;
    private readonly double _vres1;
    private readonly double _vres2;

    // OCCT's stack buffers of the Perform overloads (see the remarks).
    private readonly math_Vector _bornInf = new(1, 3);
    private readonly math_Vector _bornSup = new(1, 3);
    private readonly math_Vector _tolerance = new(1, 3);
    private readonly math_Vector _uVap = new(1, 3);
    private readonly NCollection_Array1<double> _uvres = new(1, 4);

    /// <summary>
    /// Prepares the search; <see cref="Perform(NCollection_Array1{double}, FunctionSetRoot)"/> runs it, with a
    /// solver built on <see cref="Function"/> once for many points.
    /// </summary>
    public Int2S(ThePSurface surf1, ThePSurface surf2, double tolTangency)
    {
        _done = true;
        _empty = true;
        _tangent = false;
        _zerParFunc = new ZerParFunc<ThePSurface, ThePSurfaceTool>(surf1, surf2);
        _tol = tolTangency * tolTangency;
        _pint = new PntOn2S();
        _d3d = new Direction3D();
        _d2d1 = new Direction2D();
        _d2d2 = new Direction2D();

        _ua0 = ThePSurfaceTool.FirstUParameter(surf1); //-- ThePSurfaceTool::UIntervalFirst(surf1);
        _va0 = ThePSurfaceTool.FirstVParameter(surf1); //-- ThePSurfaceTool::VIntervalFirst(surf1);
        _ua1 = ThePSurfaceTool.LastUParameter(surf1); //-- ThePSurfaceTool::UIntervalLast(surf1);
        _va1 = ThePSurfaceTool.LastVParameter(surf1); //-- ThePSurfaceTool::VIntervalLast(surf1);

        _ub0 = ThePSurfaceTool.FirstUParameter(surf2); //-- ThePSurfaceTool::UIntervalFirst(surf2);
        _vb0 = ThePSurfaceTool.FirstVParameter(surf2); //-- ThePSurfaceTool::VIntervalFirst(surf2);
        _ub1 = ThePSurfaceTool.LastUParameter(surf2); //-- ThePSurfaceTool::UIntervalLast(surf2);
        _vb1 = ThePSurfaceTool.LastVParameter(surf2); //-- ThePSurfaceTool::VIntervalLast(surf2);

        _ures1 = ThePSurfaceTool.UResolution(surf1, global::Kerf.Engine.Foundation.Precision.Confusion());
        _vres1 = ThePSurfaceTool.VResolution(surf1, global::Kerf.Engine.Foundation.Precision.Confusion());

        _ures2 = ThePSurfaceTool.UResolution(surf2, global::Kerf.Engine.Foundation.Precision.Confusion());
        _vres2 = ThePSurfaceTool.VResolution(surf2, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    /// <summary>Computes the intersection point near <paramref name="param"/> (u1, v1, u2, v2).</summary>
    public Int2S(NCollection_Array1<double> param, ThePSurface surf1, ThePSurface surf2, double tolTangency)
        : this(surf1, surf2, tolTangency)
    {
        // OCCT leaves tangent uninitialised here; the field's false is never read before Perform writes it.
        FunctionSetRoot rsnld = new(_zerParFunc, 15); //-- Modif lbr 18 MAI ?????????????
        Perform(param, rsnld);
    }

    /// <summary>
    /// Searches the intersection point near <paramref name="param"/> along the isoparametric
    /// <paramref name="choixIso"/>, and returns the best isoparametric to find the next point.
    /// </summary>
    public ConstIsoparametric Perform(NCollection_Array1<double> param, FunctionSetRoot rsnld,
                                             ConstIsoparametric choixIso)
    {
        math_Vector bornInf = _bornInf, bornSup = _bornSup, tolerance = _tolerance, uVap = _uVap;
        NCollection_Array1<double> uvres = _uvres;
        ConstIsoparametric bestChoix;

        _zerParFunc.ComputeParameters(choixIso, param, uVap, bornInf, bornSup, tolerance);
        rsnld.SetTolerance(tolerance);
        rsnld.Perform(_zerParFunc, uVap, bornInf, bornSup);
        bestChoix = choixIso;
        if (rsnld.IsDone)
        {
            if (Math.Abs(_zerParFunc.Root()) <= _tol)
            { // distance des 2 points
              // dans la tolerance
                rsnld.Root(uVap);
                _empty = false;
                _tangent = _zerParFunc.IsTangent(uVap, uvres, out bestChoix);
                _pint.SetValue(_zerParFunc.Point(), uvres[1], uvres[2], uvres[3], uvres[4]);
                if (!_tangent)
                {
                    _d3d = _zerParFunc.Direction();
                    _d2d1 = _zerParFunc.DirectionOnS1();
                    _d2d2 = _zerParFunc.DirectionOnS2();
                }
            }
            else
            {
                _empty = true;
            }
        }
        else
        {
            _empty = true;
        }
        return bestChoix;
    }

    /// <summary>
    /// Searches the intersection point near <paramref name="param"/> along the best isoparametric there, staying
    /// within the surfaces' bounds, and returns the best isoparametric to find the next point.
    /// </summary>
    public ConstIsoparametric Perform(NCollection_Array1<double> param, FunctionSetRoot rsnld)
    {
        Span<Vector3D> dpuv = stackalloc Vector3D[4];
        Span<double> epsuv = stackalloc double[4];
        NCollection_Array1<double> duv = new(1, 4);
        Span<double> uVd = stackalloc double[4];
        Span<double> uVf = stackalloc double[4];
        Span<ConstIsoparametric> choixIso = stackalloc ConstIsoparametric[4];
        ConstIsoparametric bestChoix = Tangence.ChoixRef(0);
        ThePSurface caro1 = _zerParFunc.AuxillarSurface1();
        ThePSurface caro2 = _zerParFunc.AuxillarSurface2();

        ThePSurfaceTool.D1(caro1, param[1], param[2], out _, out dpuv[0], out dpuv[1]);
        ThePSurfaceTool.D1(caro2, param[3], param[4], out _, out dpuv[2], out dpuv[3]);
        epsuv[0] = ThePSurfaceTool.UResolution(caro1, global::Kerf.Engine.Foundation.Precision.Confusion());
        epsuv[1] = ThePSurfaceTool.VResolution(caro1, global::Kerf.Engine.Foundation.Precision.Confusion());
        epsuv[2] = ThePSurfaceTool.UResolution(caro2, global::Kerf.Engine.Foundation.Precision.Confusion());
        epsuv[3] = ThePSurfaceTool.VResolution(caro2, global::Kerf.Engine.Foundation.Precision.Confusion());

        for (int j = 0; j <= 3; j++)
        {
            uVd[j] = param[j + 1];
        }

        _empty = true;

        bool tangent = Tangence.ComputeTangence(dpuv, epsuv, uVd, choixIso);
        if (tangent)
        {
            return bestChoix;
        }

        int i = 0;
        ConstIsoparametric currentChoix = bestChoix; //-- Modif 17 Mai 93
        while (_empty && i <= 3)
        {
            currentChoix = Perform(param, rsnld, choixIso[i]);
            if (!_empty)
            {
                bestChoix = currentChoix;
            }
            i++;
        }
        if (!_empty)
        { // verifier que l on ne deborde pas les frontieres
            _pint.Parameters(out duv[1], out duv[2], out duv[3], out duv[4]);
            uVd[0] = _ua0; //-- ThePSurfaceTool::UIntervalFirst(Caro1);
            uVd[1] = _va0; //-- ThePSurfaceTool::VIntervalFirst(Caro1);
            uVf[0] = _ua1; //-- ThePSurfaceTool::UIntervalLast(Caro1);
            uVf[1] = _va1; //-- ThePSurfaceTool::VIntervalLast(Caro1);
            uVd[2] = _ub0; //-- ThePSurfaceTool::UIntervalFirst(Caro2);
            uVd[3] = _vb0; //-- ubThePSurfaceTool::VIntervalFirst(Caro2);
            uVf[2] = _ub1; //-- ThePSurfaceTool::UIntervalLast(Caro2);
            uVf[3] = _vb1; //-- ThePSurfaceTool::VIntervalLast(Caro2);

            int nc, iiso;
            if (duv[1] <= uVd[0] - epsuv[0])
            {
                duv[1] = uVd[0];
                nc = 0;
                iiso = 0;
            }
            else if (duv[1] >= uVf[0] + epsuv[0])
            {
                duv[1] = uVf[0];
                nc = 0;
                iiso = 0;
            }
            else if (duv[2] <= uVd[1] - epsuv[1])
            {
                duv[2] = uVd[1];
                nc = 0;
                iiso = 1;
            }
            else if (duv[2] >= uVf[1] + epsuv[1])
            {
                duv[2] = uVf[1];
                nc = 0;
                iiso = 1;
            }
            else if (duv[3] <= uVd[2] - epsuv[2])
            {
                duv[3] = uVd[2];
                nc = 2;
                iiso = 2;
            }
            else if (duv[3] >= uVf[2] + epsuv[2])
            {
                duv[3] = uVf[2];
                nc = 2;
                iiso = 2;
            }
            else if (duv[4] <= uVd[3] - epsuv[3])
            {
                duv[4] = uVd[3];
                nc = 2;
                iiso = 3;
            }
            else if (duv[4] >= uVf[3] + epsuv[3])
            {
                duv[4] = uVf[3];
                nc = 2;
                iiso = 3;
            }
            else
            {
                return bestChoix; // on a gagne
            }
            _empty = true;
            bestChoix = Tangence.ChoixRef(iiso); // en attendant
            bestChoix = Perform(duv, rsnld, bestChoix);
            if (!_empty)
            { // verification si l on ne deborde pas sur le carreau
              // reciproque
                nc = 3 - nc;
                if (duv[nc] <= uVd[nc - 1] - epsuv[nc - 1])
                {
                    duv[nc] = uVd[nc - 1];
                }
                else if (duv[nc] >= uVf[nc - 1] + epsuv[nc - 1])
                {
                    duv[nc] = uVf[nc - 1];
                }
                else if (duv[nc + 1] <= uVd[nc])
                {
                    nc = nc + 1;
                    duv[nc] = uVd[nc - 1];
                }
                else if (duv[nc + 1] >= uVf[nc])
                {
                    nc = nc + 1;
                    duv[nc] = uVf[nc - 1];
                }
                else
                {
                    return bestChoix;
                }
                _empty = true;
                if (nc == 4)
                {
                    nc = 0;
                }
                bestChoix = Tangence.ChoixRef(nc); // en attendant
                bestChoix = Perform(duv, rsnld, bestChoix);
            }
        }
        return bestChoix;
    }

    /// <summary>True if the creation completed without failure.</summary>
    public bool IsDone => _done;

    /// <summary>True when there is no solution.</summary>
    public bool IsEmpty
    {
        get
        {
            if (!_done)
            {
                throw new NotDoneException(" IntImp_Int2S::IsEmpty() ");
            }
            return _empty;
        }
    }

    /// <summary>The intersection point.</summary>
    public PntOn2S Point()
    {
        if (!_done)
        {
            throw new NotDoneException(" IntImp_Int2S::Point() ");
        }
        if (_empty)
        {
            throw new DomainErrorException(" IntImp_Int2S::Point() ");
        }
        return _pint;
    }

    /// <summary>True if the surfaces are tangent at the intersection point.</summary>
    public bool IsTangent
    {
        get
        {
            if (!_done)
            {
                throw new NotDoneException(" IntImp_Int2S::IsTangent () ");
            }
            if (_empty)
            {
                throw new DomainErrorException(" IntImp_Int2S::IsTangent () ");
            }
            return _tangent;
        }
    }

    /// <summary>The tangent of the intersection line.</summary>
    public Direction3D Direction()
    {
        if (!_done)
        {
            throw new NotDoneException(" IntImp_Int2S::Direction () ");
        }
        if (_empty)
        {
            throw new DomainErrorException(" IntImp_Int2S::Direction () ");
        }
        if (_tangent)
        {
            throw new UndefinedDerivativeException(" IntImp_Int2S::Direction () ");
        }
        return _d3d;
    }

    /// <summary>The tangent in the parametric space of the first surface.</summary>
    public Direction2D DirectionOnS1()
    {
        if (!_done)
        {
            throw new NotDoneException(" IntImp_Int2S::DirectionOnS1 () ");
        }
        if (_empty)
        {
            throw new DomainErrorException(" IntImp_Int2S::DirectionOnS1 () ");
        }
        if (_tangent)
        {
            throw new UndefinedDerivativeException(" IntImp_Int2S::DirectionOnS1 () ");
        }
        return _d2d1;
    }

    /// <summary>The tangent in the parametric space of the second surface.</summary>
    public Direction2D DirectionOnS2()
    {
        if (!_done)
        {
            throw new NotDoneException(" IntImp_Int2S::DirectionOnS2 () ");
        }
        if (_empty)
        {
            throw new DomainErrorException(" IntImp_Int2S::DirectionOnS2 () ");
        }
        if (_tangent)
        {
            throw new UndefinedDerivativeException(" IntImp_Int2S::DirectionOnS2 () ");
        }
        return _d2d2;
    }

    /// <summary>The function the intersection is computed with (to build a solver once for many points).</summary>
    public ZerParFunc<ThePSurface, ThePSurfaceTool> Function() => _zerParFunc;

    /// <summary>The intersection point, to change.</summary>
    public ref PntOn2S ChangePoint() => ref _pint;
}
