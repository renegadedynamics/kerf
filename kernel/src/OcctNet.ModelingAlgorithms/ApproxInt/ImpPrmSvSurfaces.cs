// Port of: src/ModelingAlgorithms/TKGeomAlgo/ApproxInt/ApproxInt_ImpPrmSvSurfaces.gxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.Implicit;
using Kerf.Engine.Intersection.Walking;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Intersection.Approximation;

/// <summary>
/// The points and tangents of the intersection line of a quadric and a parametric surface, refined (when
/// <see cref="SvSurfaces.GetUseSolver"/>) by math_FunctionSetRoot on
/// <see cref="ZerImpFunc{ThePSurface, ThePSurfaceTool}"/>.
/// </summary>
/// <remarks>
/// OCCT's generic file ApproxInt_ImpPrmSvSurfaces.gxx, ported once as a generic class (decision 607). TheISurface and
/// TheISurfaceTool are IntSurf_Quadric and IntSurf_QuadricTool, and ApproxInt_TheZerImpFunc is IntImp_ZerImpFunc over
/// the same surface and tool, in every instantiation (GeomInt's and BRepApprox's), so the body names them; TheLine is
/// never used and is dropped. The function keeps the parametric surface (by reference) and a copy of the quadric
/// (decision 608): ApproxInt_Approx builds the quadric before this object and never changes it.
/// </remarks>
public class ImpPrmSvSurfaces<ThePSurface, ThePSurfaceTool> : SvSurfaces
    where ThePSurfaceTool : IApproxSurfaceTool<ThePSurface>
{
    private Point2D _myParOnS1;
    private Point2D _myParOnS2;
    private Point3D _myPnt;
    private Vector2D _myTguv1;
    private Vector2D _myTguv2;
    private Vector3D _myTg;
    private bool _myIsTangent;
    private bool _myHasBeenComputed;
    private Point2D _myParOnS1bis;
    private Point2D _myParOnS2bis;
    private Point3D _myPntbis;
    private Vector2D _myTguv1bis;
    private Vector2D _myTguv2bis;
    private Vector3D _myTgbis;
    private bool _myIsTangentbis;
    private bool _myHasBeenComputedbis;
    private readonly bool _myImplicitFirst;
    private readonly ZerImpFunc<ThePSurface, ThePSurfaceTool> _myZerImpFunc;

    //=================================================================================================
    // function : IsSingular
    // purpose  :    Returns TRUE if vectors theDU || theDV or if at least one
    //            of them has null-magnitude.
    //              theSqLinTol is square of linear tolerance.
    //              theAngTol is angular tolerance.
    //=================================================================================================

    private static bool IsSingular(in Vector3D du, in Vector3D dv, double sqLinTol, double angTol)
    {
        Vector3D aDU = du, aDV = dv;

        double sqMagnDU = aDU.SquareMagnitude(), sqMagnDV = aDV.SquareMagnitude();

        if (sqMagnDU < sqLinTol)
        {
            return true;
        }

        aDU.Divide(Math.Sqrt(sqMagnDU));

        if (sqMagnDV < sqLinTol)
        {
            return true;
        }

        aDV.Divide(Math.Sqrt(sqMagnDV));

        // Here aDU and aDV vectors have magnitude 1.0.

        if (aDU.Crossed(aDV).SquareMagnitude() < angTol * angTol)
        {
            return true;
        }

        return false;
    }

    //=================================================================================================
    // function : SingularProcessing
    // purpose  :    Computes 2D-representation (in UV-coordinates) of
    //            theTg3D vector on the surface in case when
    //            theDU.Crossed(theDV).Magnitude() == 0.0. Stores result in
    //            theTg2D variable.
    //              theDU and theDV are vectors of 1st derivative
    //            (with respect to U and V variables correspondingly).
    //              If theIsTo3DTgCompute == TRUE then theTg3D has not been
    //            defined yet (it should be computed).
    //              theLinTol is SQUARE of the tolerance.
    //
    // Algorithm:
    //              Condition
    //                  Tg=theDU*theTg2D.X()+theDV*theTg2D.Y()
    //            has to be satisfied strictly.
    //              More over, vector Tg has to be NORMALIZED
    //            (if theIsTo3DTgCompute == TRUE then new computed vector will
    //            always have magnitude 1.0).
    //=================================================================================================

    private static bool SingularProcessing(in Vector3D du, in Vector3D dv, bool isTo3DTgCompute,
                                           double linTol, double angTol, ref Vector3D tg3D,
                                           ref Vector2D tg2D)
    {
        // Attention: @ \sin theAngTol \approx theAngTol @ (for cross-product)

        // Really, vector theTg3D has to be normalized (if theIsTo3DTgCompute == FALSE).
        double sqTan = tg3D.SquareMagnitude();

        double sqMagnDU = du.SquareMagnitude(), sqMagnDV = dv.SquareMagnitude();

        // There are some reasons of singularity

        // 1.
        if ((sqMagnDU < linTol) && (sqMagnDV < linTol))
        {
            // For future, this case can be processed as same as in case of
            // osculating surfaces (expanding in Taylor series). Here,
            // we return only.

            return false;
        }

        // 2.
        if (sqMagnDU < linTol)
        {
            // In this case, theTg3D vector will be parallel with theDV.
            // Its true direction shall be precised later (the algorithm is
            // based on array of Walking-points).

            if (isTo3DTgCompute)
            {
                // theTg3D will be normalized. Its magnitude is
                const double tgMagn = 1.0;

                double norm = Math.Sqrt(sqMagnDV);
                tg3D = dv.Divided(norm);
                tg2D.SetCoord(0.0, tgMagn / norm);
            }
            else
            {
                // theTg3D is already defined.
                // Here we check only, if this tangent is parallel to theDV.

                if (dv.Crossed(tg3D).SquareMagnitude() < angTol * angTol * sqMagnDV * sqTan)
                {
                    // theTg3D is parallel to theDV

                    // Use sign "+" if theTg3D and theDV are codirectional
                    // and sign "-" if opposite
                    double dp = tg3D.Dot(dv);
                    tg2D.SetCoord(0.0, Math.CopySign(Math.Sqrt(sqTan / sqMagnDV), dp));
                }
                else
                {
                    // theTg3D is not parallel to theDV
                    // It is abnormal

                    return false;
                }
            }

            return true;
        }

        // 3.
        if (sqMagnDV < linTol)
        {
            // In this case, theTg3D vector will be parallel with theDU.
            // Its true direction shall be precised later (the algorithm is
            // based on array of Walking-points).

            if (isTo3DTgCompute)
            {
                // theTg3D will be normalized. Its magnitude is
                const double tgMagn = 1.0;

                double norm = Math.Sqrt(sqMagnDU);
                tg3D = du.Divided(norm);
                tg2D.SetCoord(tgMagn / norm, 0.0);
            }
            else
            {
                // theTg3D is already defined.
                // Here we check only, if this tangent is parallel to theDU.

                if (du.Crossed(tg3D).SquareMagnitude() < angTol * angTol * sqMagnDU * sqTan)
                {
                    // theTg3D is parallel to theDU

                    // Use sign "+" if theTg3D and theDU are codirectional
                    // and sign "-" if opposite
                    double dp = tg3D.Dot(du);
                    tg2D.SetCoord(Math.CopySign(Math.Sqrt(sqTan / sqMagnDU), dp), 0.0);
                }
                else
                {
                    // theTg3D is not parallel to theDU
                    // It is abnormal

                    return false;
                }
            }

            return true;
        }

        // 4. If aSqMagnDU > 0.0 && aSqMagnDV > 0.0 but theDV || theDU.

        double lenU = Math.Sqrt(sqMagnDU), lenV = Math.Sqrt(sqMagnDV);

        // aLenSum > 0.0 definitely
        double lenSum = lenU + lenV;

        if (dv.Dot(du) > 0.0)
        {
            // Vectors theDV and theDU are codirectional.

            if (isTo3DTgCompute)
            {
                tg2D.SetCoord(1.0 / lenSum, 1.0 / lenSum);
                tg3D = du * tg2D.X() + dv * tg2D.Y();
            }
            else
            {
                // theTg3D is already defined.
                // Here we check only, if this tangent is parallel to theDU
                //(and theDV together).

                if (du.Crossed(tg3D).SquareMagnitude() < angTol * angTol * sqMagnDU * sqTan)
                {
                    // theTg3D is parallel to theDU

                    double dp = tg3D.Dot(du);
                    double lenTg = Math.CopySign(Math.Sqrt(sqTan), dp);
                    tg2D.SetCoord(lenTg / lenSum, lenTg / lenSum);
                }
                else
                {
                    // theTg3D is not parallel to theDU
                    // It is abnormal

                    return false;
                }
            }
        }
        else
        {
            // Vectors theDV and theDU are opposite.

            if (isTo3DTgCompute)
            {
                // Here we chose theDU as direction of theTg3D.
                // True direction shall be precised later (the algorithm is
                // based on array of Walking-points).

                tg2D.SetCoord(1.0 / lenSum, -1.0 / lenSum);
                tg3D = du * tg2D.X() + dv * tg2D.Y();
            }
            else
            {
                // theTg3D is already defined.
                // Here we check only, if this tangent is parallel to theDU
                //(and theDV together).

                if (du.Crossed(tg3D).SquareMagnitude() < angTol * angTol * sqMagnDU * sqTan)
                {
                    // theTg3D is parallel to theDU

                    double dp = tg3D.Dot(du);
                    double lenTg = Math.CopySign(Math.Sqrt(sqTan), dp);
                    tg2D.SetCoord(lenTg / lenSum, -lenTg / lenSum);
                }
                else
                {
                    // theTg3D is not parallel to theDU
                    // It is abnormal

                    return false;
                }
            }
        }

        return true;
    }

    //=================================================================================================
    // function : NonSingularProcessing
    // purpose  :    Computes 2D-representation (in UV-coordinates) of
    //            theTg3D vector on the surface in case when
    //            theDU.Crossed(theDV).Magnitude() > 0.0. Stores result in
    //            theTg2D variable.
    //              theDU and theDV are vectors of 1st derivative
    //            (with respect to U and V variables correspondingly).
    //              theLinTol is SQUARE of the tolerance.
    //
    // Algorithm:
    //              Condition
    //                  Tg=theDU*theTg2D.X()+theDV*theTg2D.Y()
    //            has to be satisfied strictly.
    //              More over, vector Tg has always to be NORMALIZED.
    //=================================================================================================

    private static bool NonSingularProcessing(in Vector3D du, in Vector3D dv, in Vector3D tg3D, double linTol,
                                              double angTol, ref Vector2D tg2D)
    {
        Vector3D normal = du.Crossed(dv);
        double sqMagn = normal.SquareMagnitude();

        if (IsSingular(du, dv, linTol, angTol))
        {
            Vector3D tg = tg3D;
            return SingularProcessing(du, dv, false, linTol, angTol, ref tg, ref tg2D);
        }

        // If @\vec{T}=\vec{A}*U+\vec{B}*V@ then

        //  \left\{\begin{matrix}
        //  \vec{A} \times \vec{T} = (\vec{A} \times \vec{B})*V
        //  \vec{B} \times \vec{T} = (\vec{B} \times \vec{A})*U
        //  \end{matrix}\right.

        // From here, values of U and V can be found very easily
        //(if @\left \| \vec{A} \times \vec{B} \right \| > 0.0 @,
        // else it is singular case).

        Vector3D tgU = tg3D.Crossed(du), tgV = tg3D.Crossed(dv);
        double deltaU = tgV.SquareMagnitude() / sqMagn;
        double deltaV = tgU.SquareMagnitude() / sqMagn;

        tg2D.SetCoord(Math.CopySign(Math.Sqrt(deltaU), tgV.Dot(normal)),
                         -Math.CopySign(Math.Sqrt(deltaV), tgU.Dot(normal)));

        return true;
    }

    /// <summary>The intersection of the quadric <paramref name="iSurf"/> (first) and <paramref name="pSurf"/>.</summary>
    public ImpPrmSvSurfaces(in SurfaceQuadric iSurf, ThePSurface pSurf)
        : this(pSurf, iSurf, true)
    {
    }

    /// <summary>The intersection of <paramref name="pSurf"/> (first) and the quadric <paramref name="iSurf"/>.</summary>
    public ImpPrmSvSurfaces(ThePSurface pSurf, in SurfaceQuadric iSurf)
        : this(pSurf, iSurf, false)
    {
    }

    private ImpPrmSvSurfaces(ThePSurface pSurf, in SurfaceQuadric iSurf, bool implicitFirst)
    {
        _myParOnS1 = new Point2D();
        _myParOnS2 = new Point2D();
        _myPnt = new Point3D();
        _myTguv1 = new Vector2D();
        _myTguv2 = new Vector2D();
        _myTg = new Vector3D();
        _myIsTangent = false;
        _myHasBeenComputed = false;
        _myParOnS1bis = new Point2D();
        _myParOnS2bis = new Point2D();
        _myPntbis = new Point3D();
        _myTguv1bis = new Vector2D();
        _myTguv2bis = new Vector2D();
        _myTgbis = new Vector3D();
        _myIsTangentbis = false;
        _myHasBeenComputedbis = false;
        _myImplicitFirst = implicitFirst;
        _myZerImpFunc = new ZerImpFunc<ThePSurface, ThePSurfaceTool>(pSurf, iSurf);
        SetUseSolver(true);
    }

    public override void Pnt(double u1, double v1, double u2, double v2, ref Point3D p)
    {
        Point3D aP = new();
        Vector3D t = new();
        Vector2D ts1 = new(), ts2 = new();
        double tu1 = u1;
        double tu2 = u2;
        double tv1 = v1;
        double tv2 = v2;
        this.Compute(ref tu1, ref tv1, ref tu2, ref tv2, ref aP, ref t, ref ts1, ref ts2);
        p = _myPnt;
    }

    public override bool Tangency(double u1, double v1, double u2, double v2, ref Vector3D T)
    {
        Point3D p = new();
        Vector3D aT = new();
        Vector2D ts1 = new(), ts2 = new();
        double tu1 = u1;
        double tu2 = u2;
        double tv1 = v1;
        double tv2 = v2;
        bool t = this.Compute(ref tu1, ref tv1, ref tu2, ref tv2, ref p, ref aT, ref ts1, ref ts2);
        T = _myTg;
        return (t);
    }

    public override bool TangencyOnSurf1(double u1, double v1, double u2, double v2, ref Vector2D T)
    {
        Point3D p = new();
        Vector3D aT = new();
        Vector2D ts1 = new(), ts2 = new();
        double tu1 = u1;
        double tu2 = u2;
        double tv1 = v1;
        double tv2 = v2;
        bool t = this.Compute(ref tu1, ref tv1, ref tu2, ref tv2, ref p, ref aT, ref ts1, ref ts2);
        T = _myTguv1;
        return (t);
    }

    public override bool TangencyOnSurf2(double u1, double v1, double u2, double v2, ref Vector2D T)
    {
        Point3D p = new();
        Vector3D aT = new();
        Vector2D ts1 = new(), ts2 = new();
        double tu1 = u1;
        double tu2 = u2;
        double tv1 = v1;
        double tv2 = v2;
        bool t = this.Compute(ref tu1, ref tv1, ref tu2, ref tv2, ref p, ref aT, ref ts1, ref ts2);
        T = _myTguv2;
        return (t);
    }

    //=================================================================================================
    // function : Compute
    // purpose  :    Computes point on curve, 3D and 2D-tangents of a curve and
    //            parameters on the surfaces.
    //=================================================================================================

    public override bool Compute(ref double u1, ref double v1, ref double u2, ref double v2, ref Point3D p,
                                 ref Vector3D tg, ref Vector2D tguv1, ref Vector2D tguv2)
    {
        SurfaceQuadric qSurf = _myZerImpFunc.ISurface();
        ThePSurface pSurf = _myZerImpFunc.PSurface();
        ref Vector2D quadTg = ref _myImplicitFirst ? ref tguv1 : ref tguv2;
        ref Vector2D prmTg = ref _myImplicitFirst ? ref tguv2 : ref tguv1;

        // for square
        double nullValue = global::Kerf.Engine.Foundation.Precision.Approximation() * global::Kerf.Engine.Foundation.Precision.Approximation(),
               angTol = global::Kerf.Engine.Foundation.Precision.Angular();

        double tu1 = u1;
        double tu2 = u2;
        double tv1 = v1;
        double tv2 = v2;

        if (_myHasBeenComputed)
        {
            if ((_myParOnS1.X() == u1) && (_myParOnS1.Y() == v1) && (_myParOnS2.X() == u2)
                && (_myParOnS2.Y() == v2))
            {
                return (_myIsTangent); // OCCT: a cached result leaves the outputs as given
            }
            else if (_myHasBeenComputedbis == false)
            {
                _myTgbis = _myTg;
                _myTguv1bis = _myTguv1;
                _myTguv2bis = _myTguv2;
                _myPntbis = _myPnt;
                _myParOnS1bis = _myParOnS1;
                _myParOnS2bis = _myParOnS2;
                _myIsTangentbis = _myIsTangent;
                _myHasBeenComputedbis = _myHasBeenComputed;
            }
        }

        if (_myHasBeenComputedbis)
        {
            if ((_myParOnS1bis.X() == u1) && (_myParOnS1bis.Y() == v1) && (_myParOnS2bis.X() == u2)
                && (_myParOnS2bis.Y() == v2))
            {
                Vector3D tv = _myTg;
                Vector2D TV1 = _myTguv1;
                Vector2D TV2 = _myTguv2;
                Point3D tp = _myPnt;
                Point2D tp1 = _myParOnS1;
                Point2D tp2 = _myParOnS2;
                bool tb = _myIsTangent;

                _myTg = _myTgbis;
                _myTguv1 = _myTguv1bis;
                _myTguv2 = _myTguv2bis;
                _myPnt = _myPntbis;
                _myParOnS1 = _myParOnS1bis;
                _myParOnS2 = _myParOnS2bis;
                _myIsTangent = _myIsTangentbis;

                _myTgbis = tv;
                _myTguv1bis = TV1;
                _myTguv2bis = TV2;
                _myPntbis = tp;
                _myParOnS1bis = tp1;
                _myParOnS2bis = tp2;
                _myIsTangentbis = tb;

                return (_myIsTangent); // OCCT: a cached result leaves the outputs as given
            }
        }

        math_Vector x = new(1, 2);
        math_Vector bornInf = new(1, 2), bornSup = new(1, 2), tolerance = new(1, 2);
        //--- ThePSurfaceTool::GetResolution(aPSurf,Tolerance(1),Tolerance(2));
        tolerance[1] = 1.0e-8;
        tolerance[2] = 1.0e-8;
        double binfu, bsupu, binfv, bsupv;
        binfu = ThePSurfaceTool.FirstUParameter(pSurf);
        binfv = ThePSurfaceTool.FirstVParameter(pSurf);
        bsupu = ThePSurfaceTool.LastUParameter(pSurf);
        bsupv = ThePSurfaceTool.LastVParameter(pSurf);
        bornInf[1] = binfu;
        bornSup[1] = bsupu;
        bornInf[2] = binfv;
        bornSup[2] = bsupv;

        if (!FillInitialVectorOfSolution(u1, v1, u2, v2, binfu, bsupu, binfv, bsupv, x, out double translationU,
                                         out double translationV))
        {
            _myIsTangent = _myIsTangentbis = false;
            _myHasBeenComputed = _myHasBeenComputedbis = false;
            return (false);
        }

        bool rsnldIsDone = false;
        double pourTesterU = x[1];
        double pourTesterV = x[2];
        if (GetUseSolver())
        {
            FunctionSetRoot rsnld = new(_myZerImpFunc);
            rsnld.SetTolerance(tolerance);
            rsnld.Perform(_myZerImpFunc, x, bornInf, bornSup);
            rsnldIsDone = rsnld.IsDone;
            if (rsnldIsDone)
            {
                rsnld.Root(x);
            }
        }
        if (rsnldIsDone || !GetUseSolver())
        {
            _myHasBeenComputed = true;

            double distAvantApresU = Math.Abs(pourTesterU - x[1]);
            double distAvantApresV = Math.Abs(pourTesterV - x[2]);

            _myPnt = p = ThePSurfaceTool.Value(pSurf, x[1], x[2]);

            if ((distAvantApresV <= 0.001) && (distAvantApresU <= 0.001))
            {
                Vector3D d1uPrm, d1vPrm;
                Vector3D d1uQuad = new(), d1vQuad = new();

                if (_myImplicitFirst)
                {
                    u2 = x[1] - translationU;
                    v2 = x[2] - translationV;

                    if (qSurf.TypeQuadric() != Plane)
                    {
                        while (u1 - tu1 > M_PI)
                        {
                            u1 -= M_PI + M_PI;
                        }
                        while (tu1 - u1 > M_PI)
                        {
                            u1 += M_PI + M_PI;
                        }
                    }

                    _myParOnS1.SetCoord(tu1, tv1);
                    _myParOnS2.SetCoord(tu2, tv2);

                    Point3D p2 = new();

                    ThePSurfaceTool.D1(pSurf, x[1], x[2], out p, out d1uPrm, out d1vPrm);
                    qSurf.D1(u1, v1, ref p2, ref d1uQuad, ref d1vQuad);

                    // Middle-point of P-P2 segment
                    p.BaryCenter(1.0, p2, 1.0);
                }
                else
                {
                    u1 = x[1] - translationU;
                    v1 = x[2] - translationV;
                    // aQSurf.Parameters(P, u2, v2);
                    if (qSurf.TypeQuadric() != Plane)
                    {
                        while (u2 - tu2 > M_PI)
                        {
                            u2 -= M_PI + M_PI;
                        }
                        while (tu2 - u2 > M_PI)
                        {
                            u2 += M_PI + M_PI;
                        }
                    }

                    _myParOnS1.SetCoord(tu1, tv1);
                    _myParOnS2.SetCoord(tu2, tu2); // OCCT: tu2 for tv2, so the cache never matches this point

                    Point3D p2 = new();
                    ThePSurfaceTool.D1(pSurf, x[1], x[2], out p, out d1uPrm, out d1vPrm);

                    qSurf.D1(u2, v2, ref p2, ref d1uQuad, ref d1vQuad);

                    // Middle-point of P-P2 segment
                    p.BaryCenter(1.0, p2, 1.0);
                }

                _myPnt = p;

                // Normals to the surfaces
                Vector3D normalPrm = d1uPrm.Crossed(d1vPrm), normalImp = qSurf.Normale(_myPnt);

                double sqMagnPrm = normalPrm.SquareMagnitude(),
                       sqMagnImp = normalImp.SquareMagnitude();

                bool isPrmSingular = false, isImpSingular = false;

                if (IsSingular(d1uPrm, d1vPrm, nullValue, angTol))
                {
                    isPrmSingular = true;
                    if (!SingularProcessing(d1uPrm, d1vPrm, true, nullValue, angTol, ref tg, ref prmTg))
                    {
                        _myIsTangent = false;
                        _myHasBeenComputed = _myHasBeenComputedbis = false;
                        return false;
                    }

                    _myTg = tg;
                }
                else
                {
                    normalPrm.Divide(Math.Sqrt(sqMagnPrm));
                }

                // Analogically for implicit surface
                if (sqMagnImp < nullValue)
                {
                    isImpSingular = true;

                    if (!SingularProcessing(d1uQuad, d1vQuad, !isPrmSingular, nullValue, angTol, ref tg,
                                            ref quadTg))
                    {
                        _myIsTangent = false;
                        _myHasBeenComputed = _myHasBeenComputedbis = false;
                        return false;
                    }

                    _myTg = tg;
                }
                else
                {
                    normalImp.Divide(Math.Sqrt(sqMagnImp));
                }

                if (isImpSingular && isPrmSingular)
                {
                    // All is OK. All abnormal cases were processed above.

                    _myTguv1 = tguv1;
                    _myTguv2 = tguv2;

                    _myIsTangent = true;
                    return _myIsTangent;
                }
                else if (!(isImpSingular || isPrmSingular))
                {
                    // Processing pure non-singular case
                    //(3D- and 2D-tangents are still not defined)

                    // Ask to pay attention to the fact that here
                    // aNormalImp and aNormalPrm are normalized.
                    // Therefore, @ \left \| \vec{Tg} \right \| = 0.0 @
                    // if and only if (aNormalImp || aNormalPrm).
                    tg = normalImp.Crossed(normalPrm);
                }

                double sqMagnTg = tg.SquareMagnitude();

                if (sqMagnTg < nullValue)
                {
                    _myIsTangent = false;
                    _myHasBeenComputed = _myHasBeenComputedbis = false;
                    return false;
                }

                // Normalize Tg vector
                tg.Divide(Math.Sqrt(sqMagnTg));
                _myTg = tg;

                if (!isPrmSingular)
                {
                    // If isPrmSingular==TRUE then aPrmTg has already been computed.

                    if (!NonSingularProcessing(d1uPrm, d1vPrm, tg, nullValue, angTol, ref prmTg))
                    {
                        _myIsTangent = false;
                        _myHasBeenComputed = _myHasBeenComputedbis = false;
                        return false;
                    }
                }

                if (!isImpSingular)
                {
                    // If isImpSingular==TRUE then aQuadTg has already been computed.

                    if (!NonSingularProcessing(d1uQuad, d1vQuad, tg, nullValue, angTol, ref quadTg))
                    {
                        _myIsTangent = false;
                        _myHasBeenComputed = _myHasBeenComputedbis = false;
                        return false;
                    }
                }

                _myTguv1 = tguv1;
                _myTguv2 = tguv2;

                _myIsTangent = true;

                return true;
            }
            else
            {
                _myIsTangent = false;
                _myHasBeenComputed = _myHasBeenComputedbis = false;
                return false;
            }
        }
        else
        {
            _myIsTangent = false;
            _myHasBeenComputed = _myHasBeenComputedbis = false;
            return false;
        }
    }

    //=================================================================================================
    // function : SeekPoint
    // purpose  :    Computes point on curve and
    //            parameters on the surfaces.
    //=================================================================================================

    public override bool SeekPoint(double u1, double v1, double u2, double v2, ref PntOn2S point)
    {
        SurfaceQuadric qSurf = _myZerImpFunc.ISurface();
        ThePSurface pSurf = _myZerImpFunc.PSurface();

        math_Vector x = new(1, 2);
        math_Vector bornInf = new(1, 2), bornSup = new(1, 2), tolerance = new(1, 2);
        //--- ThePSurfaceTool::GetResolution(aPSurf,Tolerance(1),Tolerance(2));
        tolerance[1] = 1.0e-8;
        tolerance[2] = 1.0e-8;
        double binfu, bsupu, binfv, bsupv;
        binfu = ThePSurfaceTool.FirstUParameter(pSurf);
        binfv = ThePSurfaceTool.FirstVParameter(pSurf);
        bsupu = ThePSurfaceTool.LastUParameter(pSurf);
        bsupv = ThePSurfaceTool.LastVParameter(pSurf);
        bornInf[1] = binfu;
        bornSup[1] = bsupu;
        bornInf[2] = binfv;
        bornSup[2] = bsupv;

        if (!FillInitialVectorOfSolution(u1, v1, u2, v2, binfu, bsupu, binfv, bsupv, x, out double translationU,
                                         out double translationV))
        {
            return false;
        }

        double newU1 = 0.0, newV1 = 0.0, newU2 = 0.0, newV2 = 0.0;

        FunctionSetRoot rsnld = new(_myZerImpFunc);
        rsnld.SetTolerance(tolerance);
        rsnld.Perform(_myZerImpFunc, x, bornInf, bornSup);
        if (rsnld.IsDone)
        {
            _myHasBeenComputed = true;
            rsnld.Root(x);

            _myPnt = ThePSurfaceTool.Value(pSurf, x[1], x[2]);

            if (_myImplicitFirst)
            {
                newU2 = x[1] - translationU;
                newV2 = x[2] - translationV;

                qSurf.Parameters(_myPnt, ref newU1, ref newV1);
                // adjust U
                if (qSurf.TypeQuadric() != Plane)
                {
                    double sign = (newU1 > u1) ? -1 : 1;
                    while (Math.Abs(u1 - newU1) > M_PI)
                    {
                        newU1 += sign * (M_PI + M_PI);
                    }
                }
            }
            else
            {
                newU1 = x[1] - translationU;
                newV1 = x[2] - translationV;

                qSurf.Parameters(_myPnt, ref newU2, ref newV2);
                // adjust U
                if (qSurf.TypeQuadric() != Plane)
                {
                    double sign = (newU2 > u2) ? -1 : 1;
                    while (Math.Abs(u2 - newU2) > M_PI)
                    {
                        newU2 += sign * (M_PI + M_PI);
                    }
                }
            }
        }
        else
        {
            return false;
        }

        point.SetValue(_myPnt, newU1, newV1, newU2, newV2);
        return true;
    }

    //=================================================================================================

    private bool FillInitialVectorOfSolution(double u1, double v1, double u2, double v2, double binfu, double bsupu,
                                             double binfv, double bsupv, math_Vector x, out double translationU,
                                             out double translationV)
    {
        ThePSurface pSurf = _myZerImpFunc.PSurface();

        translationU = 0.0;
        translationV = 0.0;

        if (_myImplicitFirst)
        {
            if (u2 < binfu - 0.0000000001)
            {
                if (ThePSurfaceTool.IsUPeriodic(pSurf))
                {
                    double d = ThePSurfaceTool.UPeriod(pSurf);
                    do
                    {
                        translationU += d;
                    } while (u2 + translationU < binfu);
                }
                else
                {
                    return (false);
                }
            }
            else if (u2 > bsupu + 0.0000000001)
            {
                if (ThePSurfaceTool.IsUPeriodic(pSurf))
                {
                    double d = ThePSurfaceTool.UPeriod(pSurf);
                    do
                    {
                        translationU -= d;
                    } while (u2 + translationU > bsupu);
                }
                else
                {
                    return (false);
                }
            }
            if (v2 < binfv - 0.0000000001)
            {
                if (ThePSurfaceTool.IsVPeriodic(pSurf))
                {
                    double d = ThePSurfaceTool.VPeriod(pSurf);
                    do
                    {
                        translationV += d;
                    } while (v2 + translationV < binfv);
                }
                else
                {
                    return (false);
                }
            }
            else if (v2 > bsupv + 0.0000000001)
            {
                if (ThePSurfaceTool.IsVPeriodic(pSurf))
                {
                    double d = ThePSurfaceTool.VPeriod(pSurf);
                    do
                    {
                        translationV -= d;
                    } while (v2 + translationV > bsupv);
                }
                else
                {
                    return (false);
                }
            }
            x[1] = u2 + translationU;
            x[2] = v2 + translationV;
        }
        else
        {
            if (u1 < binfu - 0.0000000001)
            {
                if (ThePSurfaceTool.IsUPeriodic(pSurf))
                {
                    double d = ThePSurfaceTool.UPeriod(pSurf);
                    do
                    {
                        translationU += d;
                    } while (u1 + translationU < binfu);
                }
                else
                {
                    return (false);
                }
            }
            else if (u1 > bsupu + 0.0000000001)
            {
                if (ThePSurfaceTool.IsUPeriodic(pSurf))
                {
                    double d = ThePSurfaceTool.UPeriod(pSurf);
                    do
                    {
                        translationU -= d;
                    } while (u1 + translationU > bsupu);
                }
                else
                {
                    return (false);
                }
            }
            if (v1 < binfv - 0.0000000001)
            {
                if (ThePSurfaceTool.IsVPeriodic(pSurf))
                {
                    double d = ThePSurfaceTool.VPeriod(pSurf);
                    do
                    {
                        translationV += d;
                    } while (v1 + translationV < binfv);
                }
                else
                {
                    return (false);
                }
            }
            else if (v1 > bsupv + 0.0000000001)
            {
                if (ThePSurfaceTool.IsVPeriodic(pSurf))
                {
                    double d = ThePSurfaceTool.VPeriod(pSurf);
                    do
                    {
                        translationV -= d;
                    } while (v1 + translationV > bsupv);
                }
                else
                {
                    return (false);
                }
            }
            x[1] = u1 + translationU;
            x[2] = v1 + translationV;
        }

        //=================================================================================================

        // Make a small step from boundaries in order to avoid
        // finding "outboundaried" solution (Rsnld -> NotDone).
        if (GetUseSolver())
        {
            double du = Max(global::Kerf.Engine.Foundation.Precision.Confusion(), ThePSurfaceTool.UResolution(pSurf, global::Kerf.Engine.Foundation.Precision.Confusion()));
            double dv = Max(global::Kerf.Engine.Foundation.Precision.Confusion(), ThePSurfaceTool.VResolution(pSurf, global::Kerf.Engine.Foundation.Precision.Confusion()));
            if (x[1] - 0.0000000001 <= binfu)
            {
                x[1] = x[1] + du;
            }
            if (x[1] + 0.0000000001 >= bsupu)
            {
                x[1] = x[1] - du;
            }
            if (x[2] - 0.0000000001 <= binfv)
            {
                x[2] = x[2] + dv;
            }
            if (x[2] + 0.0000000001 >= bsupv)
            {
                x[2] = x[2] - dv;
            }
        }

        return true;
    }
}
