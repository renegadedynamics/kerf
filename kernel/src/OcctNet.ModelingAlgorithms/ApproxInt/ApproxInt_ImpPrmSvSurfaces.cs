// Port of: src/ModelingAlgorithms/TKGeomAlgo/ApproxInt/ApproxInt_ImpPrmSvSurfaces.gxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_SurfaceType;

namespace Occt;

/// <summary>
/// The points and tangents of the intersection line of a quadric and a parametric surface, refined (when
/// <see cref="ApproxInt_SvSurfaces.GetUseSolver"/>) by math_FunctionSetRoot on
/// <see cref="IntImp_ZerImpFunc{ThePSurface, ThePSurfaceTool}"/>.
/// </summary>
/// <remarks>
/// OCCT's generic file ApproxInt_ImpPrmSvSurfaces.gxx, ported once as a generic class (decision 607). TheISurface and
/// TheISurfaceTool are IntSurf_Quadric and IntSurf_QuadricTool, and ApproxInt_TheZerImpFunc is IntImp_ZerImpFunc over
/// the same surface and tool, in every instantiation (GeomInt's and BRepApprox's), so the body names them; TheLine is
/// never used and is dropped. The function keeps the parametric surface (by reference) and a copy of the quadric
/// (decision 608): ApproxInt_Approx builds the quadric before this object and never changes it.
/// </remarks>
public class ApproxInt_ImpPrmSvSurfaces<ThePSurface, ThePSurfaceTool> : ApproxInt_SvSurfaces
    where ThePSurfaceTool : IApproxInt_SurfaceTool<ThePSurface>
{
    private gp_Pnt2d MyParOnS1;
    private gp_Pnt2d MyParOnS2;
    private gp_Pnt MyPnt;
    private gp_Vec2d MyTguv1;
    private gp_Vec2d MyTguv2;
    private gp_Vec MyTg;
    private bool MyIsTangent;
    private bool MyHasBeenComputed;
    private gp_Pnt2d MyParOnS1bis;
    private gp_Pnt2d MyParOnS2bis;
    private gp_Pnt MyPntbis;
    private gp_Vec2d MyTguv1bis;
    private gp_Vec2d MyTguv2bis;
    private gp_Vec MyTgbis;
    private bool MyIsTangentbis;
    private bool MyHasBeenComputedbis;
    private readonly bool MyImplicitFirst;
    private readonly IntImp_ZerImpFunc<ThePSurface, ThePSurfaceTool> MyZerImpFunc;

    //=================================================================================================
    // function : IsSingular
    // purpose  :    Returns TRUE if vectors theDU || theDV or if at least one
    //            of them has null-magnitude.
    //              theSqLinTol is square of linear tolerance.
    //              theAngTol is angular tolerance.
    //=================================================================================================

    private static bool IsSingular(in gp_Vec theDU, in gp_Vec theDV, double theSqLinTol, double theAngTol)
    {
        gp_Vec aDU = theDU, aDV = theDV;

        double aSqMagnDU = aDU.SquareMagnitude(), aSqMagnDV = aDV.SquareMagnitude();

        if (aSqMagnDU < theSqLinTol)
        {
            return true;
        }

        aDU.Divide(Math.Sqrt(aSqMagnDU));

        if (aSqMagnDV < theSqLinTol)
        {
            return true;
        }

        aDV.Divide(Math.Sqrt(aSqMagnDV));

        // Here aDU and aDV vectors have magnitude 1.0.

        if (aDU.Crossed(aDV).SquareMagnitude() < theAngTol * theAngTol)
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

    private static bool SingularProcessing(in gp_Vec theDU, in gp_Vec theDV, bool theIsTo3DTgCompute,
                                           double theLinTol, double theAngTol, ref gp_Vec theTg3D,
                                           ref gp_Vec2d theTg2D)
    {
        // Attention: @ \sin theAngTol \approx theAngTol @ (for cross-product)

        // Really, vector theTg3D has to be normalized (if theIsTo3DTgCompute == FALSE).
        double aSQTan = theTg3D.SquareMagnitude();

        double aSqMagnDU = theDU.SquareMagnitude(), aSqMagnDV = theDV.SquareMagnitude();

        // There are some reasons of singularity

        // 1.
        if ((aSqMagnDU < theLinTol) && (aSqMagnDV < theLinTol))
        {
            // For future, this case can be processed as same as in case of
            // osculating surfaces (expanding in Taylor series). Here,
            // we return only.

            return false;
        }

        // 2.
        if (aSqMagnDU < theLinTol)
        {
            // In this case, theTg3D vector will be parallel with theDV.
            // Its true direction shall be precised later (the algorithm is
            // based on array of Walking-points).

            if (theIsTo3DTgCompute)
            {
                // theTg3D will be normalized. Its magnitude is
                const double aTgMagn = 1.0;

                double aNorm = Math.Sqrt(aSqMagnDV);
                theTg3D = theDV.Divided(aNorm);
                theTg2D.SetCoord(0.0, aTgMagn / aNorm);
            }
            else
            {
                // theTg3D is already defined.
                // Here we check only, if this tangent is parallel to theDV.

                if (theDV.Crossed(theTg3D).SquareMagnitude() < theAngTol * theAngTol * aSqMagnDV * aSQTan)
                {
                    // theTg3D is parallel to theDV

                    // Use sign "+" if theTg3D and theDV are codirectional
                    // and sign "-" if opposite
                    double aDP = theTg3D.Dot(theDV);
                    theTg2D.SetCoord(0.0, Math.CopySign(Math.Sqrt(aSQTan / aSqMagnDV), aDP));
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
        if (aSqMagnDV < theLinTol)
        {
            // In this case, theTg3D vector will be parallel with theDU.
            // Its true direction shall be precised later (the algorithm is
            // based on array of Walking-points).

            if (theIsTo3DTgCompute)
            {
                // theTg3D will be normalized. Its magnitude is
                const double aTgMagn = 1.0;

                double aNorm = Math.Sqrt(aSqMagnDU);
                theTg3D = theDU.Divided(aNorm);
                theTg2D.SetCoord(aTgMagn / aNorm, 0.0);
            }
            else
            {
                // theTg3D is already defined.
                // Here we check only, if this tangent is parallel to theDU.

                if (theDU.Crossed(theTg3D).SquareMagnitude() < theAngTol * theAngTol * aSqMagnDU * aSQTan)
                {
                    // theTg3D is parallel to theDU

                    // Use sign "+" if theTg3D and theDU are codirectional
                    // and sign "-" if opposite
                    double aDP = theTg3D.Dot(theDU);
                    theTg2D.SetCoord(Math.CopySign(Math.Sqrt(aSQTan / aSqMagnDU), aDP), 0.0);
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

        double aLenU = Math.Sqrt(aSqMagnDU), aLenV = Math.Sqrt(aSqMagnDV);

        // aLenSum > 0.0 definitely
        double aLenSum = aLenU + aLenV;

        if (theDV.Dot(theDU) > 0.0)
        {
            // Vectors theDV and theDU are codirectional.

            if (theIsTo3DTgCompute)
            {
                theTg2D.SetCoord(1.0 / aLenSum, 1.0 / aLenSum);
                theTg3D = theDU * theTg2D.X() + theDV * theTg2D.Y();
            }
            else
            {
                // theTg3D is already defined.
                // Here we check only, if this tangent is parallel to theDU
                //(and theDV together).

                if (theDU.Crossed(theTg3D).SquareMagnitude() < theAngTol * theAngTol * aSqMagnDU * aSQTan)
                {
                    // theTg3D is parallel to theDU

                    double aDP = theTg3D.Dot(theDU);
                    double aLenTg = Math.CopySign(Math.Sqrt(aSQTan), aDP);
                    theTg2D.SetCoord(aLenTg / aLenSum, aLenTg / aLenSum);
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

            if (theIsTo3DTgCompute)
            {
                // Here we chose theDU as direction of theTg3D.
                // True direction shall be precised later (the algorithm is
                // based on array of Walking-points).

                theTg2D.SetCoord(1.0 / aLenSum, -1.0 / aLenSum);
                theTg3D = theDU * theTg2D.X() + theDV * theTg2D.Y();
            }
            else
            {
                // theTg3D is already defined.
                // Here we check only, if this tangent is parallel to theDU
                //(and theDV together).

                if (theDU.Crossed(theTg3D).SquareMagnitude() < theAngTol * theAngTol * aSqMagnDU * aSQTan)
                {
                    // theTg3D is parallel to theDU

                    double aDP = theTg3D.Dot(theDU);
                    double aLenTg = Math.CopySign(Math.Sqrt(aSQTan), aDP);
                    theTg2D.SetCoord(aLenTg / aLenSum, -aLenTg / aLenSum);
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

    private static bool NonSingularProcessing(in gp_Vec theDU, in gp_Vec theDV, in gp_Vec theTg3D, double theLinTol,
                                              double theAngTol, ref gp_Vec2d theTg2D)
    {
        gp_Vec aNormal = theDU.Crossed(theDV);
        double aSQMagn = aNormal.SquareMagnitude();

        if (IsSingular(theDU, theDV, theLinTol, theAngTol))
        {
            gp_Vec aTg = theTg3D;
            return SingularProcessing(theDU, theDV, false, theLinTol, theAngTol, ref aTg, ref theTg2D);
        }

        // If @\vec{T}=\vec{A}*U+\vec{B}*V@ then

        //  \left\{\begin{matrix}
        //  \vec{A} \times \vec{T} = (\vec{A} \times \vec{B})*V
        //  \vec{B} \times \vec{T} = (\vec{B} \times \vec{A})*U
        //  \end{matrix}\right.

        // From here, values of U and V can be found very easily
        //(if @\left \| \vec{A} \times \vec{B} \right \| > 0.0 @,
        // else it is singular case).

        gp_Vec aTgU = theTg3D.Crossed(theDU), aTgV = theTg3D.Crossed(theDV);
        double aDeltaU = aTgV.SquareMagnitude() / aSQMagn;
        double aDeltaV = aTgU.SquareMagnitude() / aSQMagn;

        theTg2D.SetCoord(Math.CopySign(Math.Sqrt(aDeltaU), aTgV.Dot(aNormal)),
                         -Math.CopySign(Math.Sqrt(aDeltaV), aTgU.Dot(aNormal)));

        return true;
    }

    /// <summary>The intersection of the quadric <paramref name="ISurf"/> (first) and <paramref name="PSurf"/>.</summary>
    public ApproxInt_ImpPrmSvSurfaces(in IntSurf_Quadric ISurf, ThePSurface PSurf)
        : this(PSurf, ISurf, true)
    {
    }

    /// <summary>The intersection of <paramref name="PSurf"/> (first) and the quadric <paramref name="ISurf"/>.</summary>
    public ApproxInt_ImpPrmSvSurfaces(ThePSurface PSurf, in IntSurf_Quadric ISurf)
        : this(PSurf, ISurf, false)
    {
    }

    private ApproxInt_ImpPrmSvSurfaces(ThePSurface PSurf, in IntSurf_Quadric ISurf, bool theImplicitFirst)
    {
        MyParOnS1 = new gp_Pnt2d();
        MyParOnS2 = new gp_Pnt2d();
        MyPnt = new gp_Pnt();
        MyTguv1 = new gp_Vec2d();
        MyTguv2 = new gp_Vec2d();
        MyTg = new gp_Vec();
        MyIsTangent = false;
        MyHasBeenComputed = false;
        MyParOnS1bis = new gp_Pnt2d();
        MyParOnS2bis = new gp_Pnt2d();
        MyPntbis = new gp_Pnt();
        MyTguv1bis = new gp_Vec2d();
        MyTguv2bis = new gp_Vec2d();
        MyTgbis = new gp_Vec();
        MyIsTangentbis = false;
        MyHasBeenComputedbis = false;
        MyImplicitFirst = theImplicitFirst;
        MyZerImpFunc = new IntImp_ZerImpFunc<ThePSurface, ThePSurfaceTool>(PSurf, ISurf);
        SetUseSolver(true);
    }

    public override void Pnt(double u1, double v1, double u2, double v2, ref gp_Pnt P)
    {
        gp_Pnt aP = new();
        gp_Vec aT = new();
        gp_Vec2d aTS1 = new(), aTS2 = new();
        double tu1 = u1;
        double tu2 = u2;
        double tv1 = v1;
        double tv2 = v2;
        this.Compute(ref tu1, ref tv1, ref tu2, ref tv2, ref aP, ref aT, ref aTS1, ref aTS2);
        P = MyPnt;
    }

    public override bool Tangency(double u1, double v1, double u2, double v2, ref gp_Vec T)
    {
        gp_Pnt aP = new();
        gp_Vec aT = new();
        gp_Vec2d aTS1 = new(), aTS2 = new();
        double tu1 = u1;
        double tu2 = u2;
        double tv1 = v1;
        double tv2 = v2;
        bool t = this.Compute(ref tu1, ref tv1, ref tu2, ref tv2, ref aP, ref aT, ref aTS1, ref aTS2);
        T = MyTg;
        return (t);
    }

    public override bool TangencyOnSurf1(double u1, double v1, double u2, double v2, ref gp_Vec2d T)
    {
        gp_Pnt aP = new();
        gp_Vec aT = new();
        gp_Vec2d aTS1 = new(), aTS2 = new();
        double tu1 = u1;
        double tu2 = u2;
        double tv1 = v1;
        double tv2 = v2;
        bool t = this.Compute(ref tu1, ref tv1, ref tu2, ref tv2, ref aP, ref aT, ref aTS1, ref aTS2);
        T = MyTguv1;
        return (t);
    }

    public override bool TangencyOnSurf2(double u1, double v1, double u2, double v2, ref gp_Vec2d T)
    {
        gp_Pnt aP = new();
        gp_Vec aT = new();
        gp_Vec2d aTS1 = new(), aTS2 = new();
        double tu1 = u1;
        double tu2 = u2;
        double tv1 = v1;
        double tv2 = v2;
        bool t = this.Compute(ref tu1, ref tv1, ref tu2, ref tv2, ref aP, ref aT, ref aTS1, ref aTS2);
        T = MyTguv2;
        return (t);
    }

    //=================================================================================================
    // function : Compute
    // purpose  :    Computes point on curve, 3D and 2D-tangents of a curve and
    //            parameters on the surfaces.
    //=================================================================================================

    public override bool Compute(ref double u1, ref double v1, ref double u2, ref double v2, ref gp_Pnt P,
                                 ref gp_Vec Tg, ref gp_Vec2d Tguv1, ref gp_Vec2d Tguv2)
    {
        IntSurf_Quadric aQSurf = MyZerImpFunc.ISurface();
        ThePSurface aPSurf = MyZerImpFunc.PSurface();
        ref gp_Vec2d aQuadTg = ref MyImplicitFirst ? ref Tguv1 : ref Tguv2;
        ref gp_Vec2d aPrmTg = ref MyImplicitFirst ? ref Tguv2 : ref Tguv1;

        // for square
        double aNullValue = Precision.Approximation() * Precision.Approximation(),
               anAngTol = Precision.Angular();

        double tu1 = u1;
        double tu2 = u2;
        double tv1 = v1;
        double tv2 = v2;

        if (MyHasBeenComputed)
        {
            if ((MyParOnS1.X() == u1) && (MyParOnS1.Y() == v1) && (MyParOnS2.X() == u2)
                && (MyParOnS2.Y() == v2))
            {
                return (MyIsTangent); // OCCT: a cached result leaves the outputs as given
            }
            else if (MyHasBeenComputedbis == false)
            {
                MyTgbis = MyTg;
                MyTguv1bis = MyTguv1;
                MyTguv2bis = MyTguv2;
                MyPntbis = MyPnt;
                MyParOnS1bis = MyParOnS1;
                MyParOnS2bis = MyParOnS2;
                MyIsTangentbis = MyIsTangent;
                MyHasBeenComputedbis = MyHasBeenComputed;
            }
        }

        if (MyHasBeenComputedbis)
        {
            if ((MyParOnS1bis.X() == u1) && (MyParOnS1bis.Y() == v1) && (MyParOnS2bis.X() == u2)
                && (MyParOnS2bis.Y() == v2))
            {
                gp_Vec TV = MyTg;
                gp_Vec2d TV1 = MyTguv1;
                gp_Vec2d TV2 = MyTguv2;
                gp_Pnt TP = MyPnt;
                gp_Pnt2d TP1 = MyParOnS1;
                gp_Pnt2d TP2 = MyParOnS2;
                bool TB = MyIsTangent;

                MyTg = MyTgbis;
                MyTguv1 = MyTguv1bis;
                MyTguv2 = MyTguv2bis;
                MyPnt = MyPntbis;
                MyParOnS1 = MyParOnS1bis;
                MyParOnS2 = MyParOnS2bis;
                MyIsTangent = MyIsTangentbis;

                MyTgbis = TV;
                MyTguv1bis = TV1;
                MyTguv2bis = TV2;
                MyPntbis = TP;
                MyParOnS1bis = TP1;
                MyParOnS2bis = TP2;
                MyIsTangentbis = TB;

                return (MyIsTangent); // OCCT: a cached result leaves the outputs as given
            }
        }

        math_Vector X = new(1, 2);
        math_Vector BornInf = new(1, 2), BornSup = new(1, 2), Tolerance = new(1, 2);
        //--- ThePSurfaceTool::GetResolution(aPSurf,Tolerance(1),Tolerance(2));
        Tolerance[1] = 1.0e-8;
        Tolerance[2] = 1.0e-8;
        double binfu, bsupu, binfv, bsupv;
        binfu = ThePSurfaceTool.FirstUParameter(aPSurf);
        binfv = ThePSurfaceTool.FirstVParameter(aPSurf);
        bsupu = ThePSurfaceTool.LastUParameter(aPSurf);
        bsupv = ThePSurfaceTool.LastVParameter(aPSurf);
        BornInf[1] = binfu;
        BornSup[1] = bsupu;
        BornInf[2] = binfv;
        BornSup[2] = bsupv;

        if (!FillInitialVectorOfSolution(u1, v1, u2, v2, binfu, bsupu, binfv, bsupv, X, out double TranslationU,
                                         out double TranslationV))
        {
            MyIsTangent = MyIsTangentbis = false;
            MyHasBeenComputed = MyHasBeenComputedbis = false;
            return (false);
        }

        bool aRsnldIsDone = false;
        double PourTesterU = X[1];
        double PourTesterV = X[2];
        if (GetUseSolver())
        {
            math_FunctionSetRoot Rsnld = new(MyZerImpFunc);
            Rsnld.SetTolerance(Tolerance);
            Rsnld.Perform(MyZerImpFunc, X, BornInf, BornSup);
            aRsnldIsDone = Rsnld.IsDone();
            if (aRsnldIsDone)
            {
                Rsnld.Root(X);
            }
        }
        if (aRsnldIsDone || !GetUseSolver())
        {
            MyHasBeenComputed = true;

            double DistAvantApresU = Math.Abs(PourTesterU - X[1]);
            double DistAvantApresV = Math.Abs(PourTesterV - X[2]);

            MyPnt = P = ThePSurfaceTool.Value(aPSurf, X[1], X[2]);

            if ((DistAvantApresV <= 0.001) && (DistAvantApresU <= 0.001))
            {
                gp_Vec aD1uPrm, aD1vPrm;
                gp_Vec aD1uQuad = new(), aD1vQuad = new();

                if (MyImplicitFirst)
                {
                    u2 = X[1] - TranslationU;
                    v2 = X[2] - TranslationV;

                    if (aQSurf.TypeQuadric() != GeomAbs_Plane)
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

                    MyParOnS1.SetCoord(tu1, tv1);
                    MyParOnS2.SetCoord(tu2, tv2);

                    gp_Pnt aP2 = new();

                    ThePSurfaceTool.D1(aPSurf, X[1], X[2], out P, out aD1uPrm, out aD1vPrm);
                    aQSurf.D1(u1, v1, ref aP2, ref aD1uQuad, ref aD1vQuad);

                    // Middle-point of P-P2 segment
                    P.BaryCenter(1.0, aP2, 1.0);
                }
                else
                {
                    u1 = X[1] - TranslationU;
                    v1 = X[2] - TranslationV;
                    // aQSurf.Parameters(P, u2, v2);
                    if (aQSurf.TypeQuadric() != GeomAbs_Plane)
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

                    MyParOnS1.SetCoord(tu1, tv1);
                    MyParOnS2.SetCoord(tu2, tu2); // OCCT: tu2 for tv2, so the cache never matches this point

                    gp_Pnt aP2 = new();
                    ThePSurfaceTool.D1(aPSurf, X[1], X[2], out P, out aD1uPrm, out aD1vPrm);

                    aQSurf.D1(u2, v2, ref aP2, ref aD1uQuad, ref aD1vQuad);

                    // Middle-point of P-P2 segment
                    P.BaryCenter(1.0, aP2, 1.0);
                }

                MyPnt = P;

                // Normals to the surfaces
                gp_Vec aNormalPrm = aD1uPrm.Crossed(aD1vPrm), aNormalImp = aQSurf.Normale(MyPnt);

                double aSQMagnPrm = aNormalPrm.SquareMagnitude(),
                       aSQMagnImp = aNormalImp.SquareMagnitude();

                bool isPrmSingular = false, isImpSingular = false;

                if (IsSingular(aD1uPrm, aD1vPrm, aNullValue, anAngTol))
                {
                    isPrmSingular = true;
                    if (!SingularProcessing(aD1uPrm, aD1vPrm, true, aNullValue, anAngTol, ref Tg, ref aPrmTg))
                    {
                        MyIsTangent = false;
                        MyHasBeenComputed = MyHasBeenComputedbis = false;
                        return false;
                    }

                    MyTg = Tg;
                }
                else
                {
                    aNormalPrm.Divide(Math.Sqrt(aSQMagnPrm));
                }

                // Analogically for implicit surface
                if (aSQMagnImp < aNullValue)
                {
                    isImpSingular = true;

                    if (!SingularProcessing(aD1uQuad, aD1vQuad, !isPrmSingular, aNullValue, anAngTol, ref Tg,
                                            ref aQuadTg))
                    {
                        MyIsTangent = false;
                        MyHasBeenComputed = MyHasBeenComputedbis = false;
                        return false;
                    }

                    MyTg = Tg;
                }
                else
                {
                    aNormalImp.Divide(Math.Sqrt(aSQMagnImp));
                }

                if (isImpSingular && isPrmSingular)
                {
                    // All is OK. All abnormal cases were processed above.

                    MyTguv1 = Tguv1;
                    MyTguv2 = Tguv2;

                    MyIsTangent = true;
                    return MyIsTangent;
                }
                else if (!(isImpSingular || isPrmSingular))
                {
                    // Processing pure non-singular case
                    //(3D- and 2D-tangents are still not defined)

                    // Ask to pay attention to the fact that here
                    // aNormalImp and aNormalPrm are normalized.
                    // Therefore, @ \left \| \vec{Tg} \right \| = 0.0 @
                    // if and only if (aNormalImp || aNormalPrm).
                    Tg = aNormalImp.Crossed(aNormalPrm);
                }

                double aSQMagnTg = Tg.SquareMagnitude();

                if (aSQMagnTg < aNullValue)
                {
                    MyIsTangent = false;
                    MyHasBeenComputed = MyHasBeenComputedbis = false;
                    return false;
                }

                // Normalize Tg vector
                Tg.Divide(Math.Sqrt(aSQMagnTg));
                MyTg = Tg;

                if (!isPrmSingular)
                {
                    // If isPrmSingular==TRUE then aPrmTg has already been computed.

                    if (!NonSingularProcessing(aD1uPrm, aD1vPrm, Tg, aNullValue, anAngTol, ref aPrmTg))
                    {
                        MyIsTangent = false;
                        MyHasBeenComputed = MyHasBeenComputedbis = false;
                        return false;
                    }
                }

                if (!isImpSingular)
                {
                    // If isImpSingular==TRUE then aQuadTg has already been computed.

                    if (!NonSingularProcessing(aD1uQuad, aD1vQuad, Tg, aNullValue, anAngTol, ref aQuadTg))
                    {
                        MyIsTangent = false;
                        MyHasBeenComputed = MyHasBeenComputedbis = false;
                        return false;
                    }
                }

                MyTguv1 = Tguv1;
                MyTguv2 = Tguv2;

                MyIsTangent = true;

                return true;
            }
            else
            {
                MyIsTangent = false;
                MyHasBeenComputed = MyHasBeenComputedbis = false;
                return false;
            }
        }
        else
        {
            MyIsTangent = false;
            MyHasBeenComputed = MyHasBeenComputedbis = false;
            return false;
        }
    }

    //=================================================================================================
    // function : SeekPoint
    // purpose  :    Computes point on curve and
    //            parameters on the surfaces.
    //=================================================================================================

    public override bool SeekPoint(double u1, double v1, double u2, double v2, ref IntSurf_PntOn2S Point)
    {
        IntSurf_Quadric aQSurf = MyZerImpFunc.ISurface();
        ThePSurface aPSurf = MyZerImpFunc.PSurface();

        math_Vector X = new(1, 2);
        math_Vector BornInf = new(1, 2), BornSup = new(1, 2), Tolerance = new(1, 2);
        //--- ThePSurfaceTool::GetResolution(aPSurf,Tolerance(1),Tolerance(2));
        Tolerance[1] = 1.0e-8;
        Tolerance[2] = 1.0e-8;
        double binfu, bsupu, binfv, bsupv;
        binfu = ThePSurfaceTool.FirstUParameter(aPSurf);
        binfv = ThePSurfaceTool.FirstVParameter(aPSurf);
        bsupu = ThePSurfaceTool.LastUParameter(aPSurf);
        bsupv = ThePSurfaceTool.LastVParameter(aPSurf);
        BornInf[1] = binfu;
        BornSup[1] = bsupu;
        BornInf[2] = binfv;
        BornSup[2] = bsupv;

        if (!FillInitialVectorOfSolution(u1, v1, u2, v2, binfu, bsupu, binfv, bsupv, X, out double TranslationU,
                                         out double TranslationV))
        {
            return false;
        }

        double NewU1 = 0.0, NewV1 = 0.0, NewU2 = 0.0, NewV2 = 0.0;

        math_FunctionSetRoot Rsnld = new(MyZerImpFunc);
        Rsnld.SetTolerance(Tolerance);
        Rsnld.Perform(MyZerImpFunc, X, BornInf, BornSup);
        if (Rsnld.IsDone())
        {
            MyHasBeenComputed = true;
            Rsnld.Root(X);

            MyPnt = ThePSurfaceTool.Value(aPSurf, X[1], X[2]);

            if (MyImplicitFirst)
            {
                NewU2 = X[1] - TranslationU;
                NewV2 = X[2] - TranslationV;

                aQSurf.Parameters(MyPnt, ref NewU1, ref NewV1);
                // adjust U
                if (aQSurf.TypeQuadric() != GeomAbs_Plane)
                {
                    double sign = (NewU1 > u1) ? -1 : 1;
                    while (Math.Abs(u1 - NewU1) > M_PI)
                    {
                        NewU1 += sign * (M_PI + M_PI);
                    }
                }
            }
            else
            {
                NewU1 = X[1] - TranslationU;
                NewV1 = X[2] - TranslationV;

                aQSurf.Parameters(MyPnt, ref NewU2, ref NewV2);
                // adjust U
                if (aQSurf.TypeQuadric() != GeomAbs_Plane)
                {
                    double sign = (NewU2 > u2) ? -1 : 1;
                    while (Math.Abs(u2 - NewU2) > M_PI)
                    {
                        NewU2 += sign * (M_PI + M_PI);
                    }
                }
            }
        }
        else
        {
            return false;
        }

        Point.SetValue(MyPnt, NewU1, NewV1, NewU2, NewV2);
        return true;
    }

    //=================================================================================================

    private bool FillInitialVectorOfSolution(double u1, double v1, double u2, double v2, double binfu, double bsupu,
                                             double binfv, double bsupv, math_Vector X, out double TranslationU,
                                             out double TranslationV)
    {
        ThePSurface aPSurf = MyZerImpFunc.PSurface();

        TranslationU = 0.0;
        TranslationV = 0.0;

        if (MyImplicitFirst)
        {
            if (u2 < binfu - 0.0000000001)
            {
                if (ThePSurfaceTool.IsUPeriodic(aPSurf))
                {
                    double d = ThePSurfaceTool.UPeriod(aPSurf);
                    do
                    {
                        TranslationU += d;
                    } while (u2 + TranslationU < binfu);
                }
                else
                {
                    return (false);
                }
            }
            else if (u2 > bsupu + 0.0000000001)
            {
                if (ThePSurfaceTool.IsUPeriodic(aPSurf))
                {
                    double d = ThePSurfaceTool.UPeriod(aPSurf);
                    do
                    {
                        TranslationU -= d;
                    } while (u2 + TranslationU > bsupu);
                }
                else
                {
                    return (false);
                }
            }
            if (v2 < binfv - 0.0000000001)
            {
                if (ThePSurfaceTool.IsVPeriodic(aPSurf))
                {
                    double d = ThePSurfaceTool.VPeriod(aPSurf);
                    do
                    {
                        TranslationV += d;
                    } while (v2 + TranslationV < binfv);
                }
                else
                {
                    return (false);
                }
            }
            else if (v2 > bsupv + 0.0000000001)
            {
                if (ThePSurfaceTool.IsVPeriodic(aPSurf))
                {
                    double d = ThePSurfaceTool.VPeriod(aPSurf);
                    do
                    {
                        TranslationV -= d;
                    } while (v2 + TranslationV > bsupv);
                }
                else
                {
                    return (false);
                }
            }
            X[1] = u2 + TranslationU;
            X[2] = v2 + TranslationV;
        }
        else
        {
            if (u1 < binfu - 0.0000000001)
            {
                if (ThePSurfaceTool.IsUPeriodic(aPSurf))
                {
                    double d = ThePSurfaceTool.UPeriod(aPSurf);
                    do
                    {
                        TranslationU += d;
                    } while (u1 + TranslationU < binfu);
                }
                else
                {
                    return (false);
                }
            }
            else if (u1 > bsupu + 0.0000000001)
            {
                if (ThePSurfaceTool.IsUPeriodic(aPSurf))
                {
                    double d = ThePSurfaceTool.UPeriod(aPSurf);
                    do
                    {
                        TranslationU -= d;
                    } while (u1 + TranslationU > bsupu);
                }
                else
                {
                    return (false);
                }
            }
            if (v1 < binfv - 0.0000000001)
            {
                if (ThePSurfaceTool.IsVPeriodic(aPSurf))
                {
                    double d = ThePSurfaceTool.VPeriod(aPSurf);
                    do
                    {
                        TranslationV += d;
                    } while (v1 + TranslationV < binfv);
                }
                else
                {
                    return (false);
                }
            }
            else if (v1 > bsupv + 0.0000000001)
            {
                if (ThePSurfaceTool.IsVPeriodic(aPSurf))
                {
                    double d = ThePSurfaceTool.VPeriod(aPSurf);
                    do
                    {
                        TranslationV -= d;
                    } while (v1 + TranslationV > bsupv);
                }
                else
                {
                    return (false);
                }
            }
            X[1] = u1 + TranslationU;
            X[2] = v1 + TranslationV;
        }

        //=================================================================================================

        // Make a small step from boundaries in order to avoid
        // finding "outboundaried" solution (Rsnld -> NotDone).
        if (GetUseSolver())
        {
            double du = Max(Precision.Confusion(), ThePSurfaceTool.UResolution(aPSurf, Precision.Confusion()));
            double dv = Max(Precision.Confusion(), ThePSurfaceTool.VResolution(aPSurf, Precision.Confusion()));
            if (X[1] - 0.0000000001 <= binfu)
            {
                X[1] = X[1] + du;
            }
            if (X[1] + 0.0000000001 >= bsupu)
            {
                X[1] = X[1] - du;
            }
            if (X[2] - 0.0000000001 <= binfv)
            {
                X[2] = X[2] + dv;
            }
            if (X[2] + 0.0000000001 >= bsupv)
            {
                X[2] = X[2] - dv;
            }
        }

        return true;
    }
}
