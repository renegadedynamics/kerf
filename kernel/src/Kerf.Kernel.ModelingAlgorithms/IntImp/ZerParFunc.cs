// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntImp/IntImp_ZerParFunc.gxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntImp/IntImp_ZerParFunc.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Numerics;
using Kerf.Geometry;

namespace Kerf.Engine.Intersection.Implicit;

/// <summary>
/// The function whose zeros are the intersection points of two parametric surfaces along an isoparametric of one of
/// them: F = S1(u1, v1) - S2(u2, v2) with one of the four parameters fixed, three equations in the other three, with
/// its derivatives, for math_FunctionSetRoot; and, at a root, the tangent of the intersection line and the best
/// isoparametric to march along it.
/// </summary>
/// <remarks>
/// OCCT's generic file IntImp_ZerParFunc.gxx, ported once as a generic class over the surfaces and their tool
/// (decisions 202, 337 and 607). OCCT's instantiations (IntWalk_TheFunctionOfTheInt2S, GeomInt's and BRepApprox's
/// TheFunctionOfTheInt2S...) are sealed subclasses, or are the generic itself where only
/// <see cref="Int2S{ThePSurface, ThePSurfaceTool}"/> builds them. OCCT keeps pointers to the caller's surface
/// handles; the port keeps the surfaces (adaptors, by reference, as the handles).
/// </remarks>
internal class ZerParFunc<ThePSurface, ThePSurfaceTool> : FunctionSetWithDerivatives
    where ThePSurfaceTool : IZerCSParPSurfaceTool<ThePSurface>
{
    private readonly ThePSurface _surf1;
    private readonly ThePSurface _surf2;
    private Point3D _pntsol1;
    private Point3D _pntsol2;
    private readonly double[] _f = new double[3];
    // OCCT's field compute is written, never read: left out (decision 143).
    private bool _tangent;
    private readonly double[] _tgduv = new double[4];
    private readonly Vector3D[] _dpuv = [new Vector3D(), new Vector3D(), new Vector3D(), new Vector3D()];
    private ConstIsoparametric _chxIso;
    private double _paramConst;
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

    public ZerParFunc(ThePSurface s1, ThePSurface s2)
    {
        _tangent = false;
        _paramConst = 0.0;
        _surf1 = s1;
        _surf2 = s2;

        _ua0 = ThePSurfaceTool.FirstUParameter(_surf1); //-- ThePSurfaceTool::UIntervalFirst(surf1);
        _va0 = ThePSurfaceTool.FirstVParameter(_surf1); //-- ThePSurfaceTool::VIntervalFirst(surf1);
        _ua1 = ThePSurfaceTool.LastUParameter(_surf1); //-- ThePSurfaceTool::UIntervalLast(surf1);
        _va1 = ThePSurfaceTool.LastVParameter(_surf1); //-- ThePSurfaceTool::VIntervalLast(surf1);

        _ub0 = ThePSurfaceTool.FirstUParameter(_surf2); //-- ThePSurfaceTool::UIntervalFirst(SURF2);
        _vb0 = ThePSurfaceTool.FirstVParameter(_surf2); //-- ThePSurfaceTool::VIntervalFirst(surf2);
        _ub1 = ThePSurfaceTool.LastUParameter(_surf2); //-- ThePSurfaceTool::UIntervalLast(surf2);
        _vb1 = ThePSurfaceTool.LastVParameter(_surf2); //-- ThePSurfaceTool::VIntervalLast(surf2);

        _ures1 = ThePSurfaceTool.UResolution(_surf1, global::Kerf.Engine.Foundation.Precision.Confusion());
        _vres1 = ThePSurfaceTool.VResolution(_surf1, global::Kerf.Engine.Foundation.Precision.Confusion());

        _ures2 = ThePSurfaceTool.UResolution(_surf2, global::Kerf.Engine.Foundation.Precision.Confusion());
        _vres2 = ThePSurfaceTool.VResolution(_surf2, global::Kerf.Engine.Foundation.Precision.Confusion());

        // memset(f, 0, sizeof(f)): the array is new
        _tangent = false;
        _pntsol1 = new Point3D();
        _pntsol2 = new Point3D();
    }

    public override int NbVariables() => 3;

    public override int NbEquations() => 3;

    public override bool Value(math_Vector x, math_Vector f)
    {
        switch (_chxIso)
        {
            case ConstIsoparametric.UIsoparametricOnCaro1:
            {
                _pntsol1 = ThePSurfaceTool.Value(_surf1, _paramConst, x[1]);
                _pntsol2 = ThePSurfaceTool.Value(_surf2, x[2], x[3]);
            }
            break;
            case ConstIsoparametric.VIsoparametricOnCaro1:
            {
                _pntsol1 = ThePSurfaceTool.Value(_surf1, x[1], _paramConst);
                _pntsol2 = ThePSurfaceTool.Value(_surf2, x[2], x[3]);
            }
            break;
            case ConstIsoparametric.UIsoparametricOnCaro2:
            {
                _pntsol1 = ThePSurfaceTool.Value(_surf1, x[1], x[2]);
                _pntsol2 = ThePSurfaceTool.Value(_surf2, _paramConst, x[3]);
            }
            break;
            case ConstIsoparametric.VIsoparametricOnCaro2:
            {
                _pntsol1 = ThePSurfaceTool.Value(_surf1, x[1], x[2]);
                _pntsol2 = ThePSurfaceTool.Value(_surf2, x[3], _paramConst);
            }
            break;
        }

        _f[0] = f[1] = _pntsol1.X - _pntsol2.X;
        _f[1] = f[2] = _pntsol1.Y - _pntsol2.Y;
        _f[2] = f[3] = _pntsol1.Z - _pntsol2.Z;

        return true;
    }

    public override bool Derivatives(math_Vector x, Matrix d)
    {
        FillDerivatives(x, d);
        return true;
    }

    public override bool Values(math_Vector x, math_Vector f, Matrix d)
    {
        FillDerivatives(x, d);

        _f[0] = f[1] = _pntsol1.X - _pntsol2.X;
        _f[1] = f[2] = _pntsol1.Y - _pntsol2.Y;
        _f[2] = f[3] = _pntsol1.Z - _pntsol2.Z;

        return true;
    }

    // The switch that OCCT writes out twice, verbatim, in Derivatives and Values (decision 340's approach).
    private void FillDerivatives(math_Vector x, Matrix d)
    {
        switch (_chxIso)
        {
            case ConstIsoparametric.UIsoparametricOnCaro1:
            {
                ThePSurfaceTool.D1(_surf1, _paramConst, x[1], out _pntsol1, out _dpuv[0], out _dpuv[1]);
                ThePSurfaceTool.D1(_surf2, x[2], x[3], out _pntsol2, out _dpuv[2], out _dpuv[3]);
                d[1, 1] = _dpuv[1].X;
                d[1, 2] = -_dpuv[2].X;
                d[1, 3] = -_dpuv[3].X;
                d[2, 1] = _dpuv[1].Y;
                d[2, 2] = -_dpuv[2].Y;
                d[2, 3] = -_dpuv[3].Y;
                d[3, 1] = _dpuv[1].Z;
                d[3, 2] = -_dpuv[2].Z;
                d[3, 3] = -_dpuv[3].Z;
            }
            break;
            case ConstIsoparametric.VIsoparametricOnCaro1:
            {
                ThePSurfaceTool.D1(_surf1, x[1], _paramConst, out _pntsol1, out _dpuv[0], out _dpuv[1]);
                ThePSurfaceTool.D1(_surf2, x[2], x[3], out _pntsol2, out _dpuv[2], out _dpuv[3]);
                d[1, 1] = _dpuv[0].X;
                d[1, 2] = -_dpuv[2].X;
                d[1, 3] = -_dpuv[3].X;
                d[2, 1] = _dpuv[0].Y;
                d[2, 2] = -_dpuv[2].Y;
                d[2, 3] = -_dpuv[3].Y;
                d[3, 1] = _dpuv[0].Z;
                d[3, 2] = -_dpuv[2].Z;
                d[3, 3] = -_dpuv[3].Z;
            }
            break;
            case ConstIsoparametric.UIsoparametricOnCaro2:
            {
                ThePSurfaceTool.D1(_surf1, x[1], x[2], out _pntsol1, out _dpuv[0], out _dpuv[1]);
                ThePSurfaceTool.D1(_surf2, _paramConst, x[3], out _pntsol2, out _dpuv[2], out _dpuv[3]);
                d[1, 1] = _dpuv[0].X;
                d[1, 2] = _dpuv[1].X;
                d[1, 3] = -_dpuv[3].X;
                d[2, 1] = _dpuv[0].Y;
                d[2, 2] = _dpuv[1].Y;
                d[2, 3] = -_dpuv[3].Y;
                d[3, 1] = _dpuv[0].Z;
                d[3, 2] = _dpuv[1].Z;
                d[3, 3] = -_dpuv[3].Z;
            }
            break;
            case ConstIsoparametric.VIsoparametricOnCaro2:
            {
                ThePSurfaceTool.D1(_surf1, x[1], x[2], out _pntsol1, out _dpuv[0], out _dpuv[1]);
                ThePSurfaceTool.D1(_surf2, x[3], _paramConst, out _pntsol2, out _dpuv[2], out _dpuv[3]);
                d[1, 1] = _dpuv[0].X;
                d[1, 2] = _dpuv[1].X;
                d[1, 3] = -_dpuv[2].X;
                d[2, 1] = _dpuv[0].Y;
                d[2, 2] = _dpuv[1].Y;
                d[2, 3] = -_dpuv[2].Y;
                d[3, 1] = _dpuv[0].Z;
                d[3, 2] = _dpuv[1].Z;
                d[3, 3] = -_dpuv[2].Z;
            }
            break;
        }
    }

    /// <summary>
    /// Fixes the isoparametric <paramref name="choixIso"/> at its value in <paramref name="param"/> (u1, v1, u2, v2) and
    /// gives the starting point, the bounds (the surfaces' bounds widened by 1% of their range) and the tolerances of
    /// the three other parameters.
    /// </summary>
    public void ComputeParameters(ConstIsoparametric choixIso, NCollection_Array1<double> param,
                                  math_Vector uVap, math_Vector bornInf, math_Vector bornSup, math_Vector tolerance)
    {
        _chxIso = choixIso;
        switch (_chxIso)
        {
            case ConstIsoparametric.UIsoparametricOnCaro1:
            {
                _paramConst = param[1];
                uVap[1] = param[2];
                uVap[2] = param[3];
                uVap[3] = param[4];

                bornInf[1] = _va0;
                bornSup[1] = _va1;
                bornInf[2] = _ub0;
                bornInf[3] = _vb0;
                bornSup[2] = _ub1;
                bornSup[3] = _vb1;

                tolerance[1] = _vres1;
                tolerance[2] = _ures2;
                tolerance[3] = _vres2;
            }
            break;
            case ConstIsoparametric.VIsoparametricOnCaro1:
            {
                _paramConst = param[2];
                uVap[1] = param[1];
                uVap[2] = param[3];
                uVap[3] = param[4];

                bornInf[1] = _ua0;
                bornSup[1] = _ua1;
                bornInf[2] = _ub0;
                bornSup[2] = _ub1;
                bornInf[3] = _vb0;
                bornSup[3] = _vb1;

                tolerance[1] = _ures1;
                tolerance[2] = _ures2;
                tolerance[3] = _vres2;
            }
            break;
            case ConstIsoparametric.UIsoparametricOnCaro2:
            {
                _paramConst = param[3];
                uVap[1] = param[1];
                uVap[2] = param[2];
                uVap[3] = param[4];

                bornInf[1] = _ua0;
                bornSup[1] = _ua1;
                bornInf[2] = _va0;
                bornSup[2] = _va1;
                bornInf[3] = _vb0;
                bornSup[3] = _vb1;

                tolerance[1] = _ures1;
                tolerance[2] = _vres1;
                tolerance[3] = _vres2;
            }
            break;
            case ConstIsoparametric.VIsoparametricOnCaro2:
            {
                _paramConst = param[4];
                uVap[1] = param[1];
                uVap[2] = param[2];
                uVap[3] = param[3];

                bornInf[1] = _ua0;
                bornSup[1] = _ua1;
                bornInf[2] = _va0;
                bornSup[2] = _va1;
                bornInf[3] = _ub0;
                bornSup[3] = _ub1;

                tolerance[1] = _ures1;
                tolerance[2] = _vres1;
                tolerance[3] = _ures2;
            }
            break;
        }

        double incr1 = (bornSup[1] - bornInf[1]) * 0.01;
        double incr2 = (bornSup[2] - bornInf[2]) * 0.01;
        double incr3 = (bornSup[3] - bornInf[3]) * 0.01;
        bornInf[1] -= incr1;
        bornSup[1] += incr1;
        bornInf[2] -= incr2;
        bornSup[2] += incr2;
        bornInf[3] -= incr3;
        bornSup[3] += incr3;
    }

    /// <summary>The sum of the squares of the last evaluated values.</summary>
    public double Root() => _f[0] * _f[0] + _f[1] * _f[1] + _f[2] * _f[2];

    /// <summary>The middle of the two last evaluated points.</summary>
    public Point3D Point() => new Point3D((_pntsol1.XYZ() + _pntsol2.XYZ()) / 2.0);

    /// <summary>
    /// Writes the solution <paramref name="uVap"/> into <paramref name="param"/> (u1, v1, u2, v2), and returns true
    /// if the surfaces are tangent there; otherwise computes the tangent and the best isoparametric to march along
    /// it (<paramref name="bestChoix"/>).
    /// </summary>
    public bool IsTangent(math_Vector uVap, NCollection_Array1<double> param, out ConstIsoparametric bestChoix)
    {
        switch (_chxIso)
        {
            case ConstIsoparametric.UIsoparametricOnCaro1:
            {
                param[1] = _paramConst;
                param[2] = uVap[1];
                param[3] = uVap[2];
                param[4] = uVap[3];
            }
            break;
            case ConstIsoparametric.VIsoparametricOnCaro1:
            {
                param[2] = _paramConst;
                param[1] = uVap[1];
                param[3] = uVap[2];
                param[4] = uVap[3];
            }
            break;
            case ConstIsoparametric.UIsoparametricOnCaro2:
            {
                param[3] = _paramConst;
                param[1] = uVap[1];
                param[2] = uVap[2];
                param[4] = uVap[3];
            }
            break;
            case ConstIsoparametric.VIsoparametricOnCaro2:
            {
                param[4] = _paramConst;
                param[1] = uVap[1];
                param[2] = uVap[2];
                param[3] = uVap[3];
            }
            break;
        }
        Span<ConstIsoparametric> tabIso = stackalloc ConstIsoparametric[4];
        Span<double> epsUV = stackalloc double[4];
        epsUV[0] = _ures1;
        epsUV[1] = _vres1;
        epsUV[2] = _ures2;
        epsUV[3] = _vres2;

        _tangent = Tangence.ComputeTangence(_dpuv, epsUV, _tgduv, tabIso);
        if (!_tangent)
        {
            _chxIso = tabIso[0];
        }
        bestChoix = _chxIso;
        return _tangent;
    }

    /// <summary>The tangent of the intersection line. Throws StdFail_UndefinedDerivative at a tangent point.</summary>
    public Direction3D Direction()
    {
        if (_tangent)
        {
            throw new UndefinedDerivativeException();
        }
        return new Direction3D(_tgduv[0] * _dpuv[0].XYZ() + _tgduv[1] * _dpuv[1].XYZ());
    }

    /// <summary>The tangent in the parametric space of the first surface.</summary>
    public Direction2D DirectionOnS1()
    {
        if (_tangent)
        {
            throw new UndefinedDerivativeException();
        }
        return new Direction2D(_tgduv[0], _tgduv[1]);
    }

    /// <summary>The tangent in the parametric space of the second surface.</summary>
    public Direction2D DirectionOnS2()
    {
        if (_tangent)
        {
            throw new UndefinedDerivativeException();
        }
        return new Direction2D(_tgduv[2], _tgduv[3]);
    }

    public ThePSurface AuxillarSurface1() => _surf1;

    public ThePSurface AuxillarSurface2() => _surf2;
}
