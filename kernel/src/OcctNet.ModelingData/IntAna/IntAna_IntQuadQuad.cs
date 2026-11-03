// Port of: src/ModelingData/TKGeomBase/IntAna/IntAna_IntQuadQuad.hxx
// Port of: src/ModelingData/TKGeomBase/IntAna/IntAna_IntQuadQuad.lxx
// Port of: src/ModelingData/TKGeomBase/IntAna/IntAna_IntQuadQuad.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The analytic intersection between a cylinder or a cone and another quadric (<see cref="IntAna_Quadric"/>), used
/// when the geometric intersection (IntAna_QuadQuadGeo) has no geometric solution. The result is curves
/// (<see cref="IntAna_Curve"/>) and points.
/// </summary>
/// <remarks>An algorithm object (decision 117): construct it with the surfaces, or empty and call <c>Perform</c>.
/// OCCT's debug function <c>DumpCurve</c> is left out.</remarks>
public class IntAna_IntQuadQuad
{
    protected bool done;
    protected bool identical;
    protected readonly IntAna_Curve[] TheCurve = NewCurves();
    protected readonly int[] previouscurve = new int[12];
    protected readonly int[] nextcurve = new int[12];
    protected int NbCurves;
    protected int Nbpoints;
    protected readonly Point3D[] Thepoints = [new Point3D(), new Point3D()];
    protected int myNbMaxCurves;
    protected double myEpsilon;
    protected double myEpsilonCoeffPolyNull;

    // OCCT's `IntAna_Curve TheCurve[12]` default-constructs its elements
    private static IntAna_Curve[] NewCurves()
    {
        IntAna_Curve[] aCurves = new IntAna_Curve[12];
        for (int i = 0; i < aCurves.Length; i++)
        {
            aCurves[i] = new IntAna_Curve();
        }
        return aCurves;
    }

    //=======================================================================
    // function : AddSpecialPoints
    // purpose  : Sometimes the boundaries theTheta1 and theTheta2 are
    //            computed with some inaccuracy. At that, some special points
    //            (cone apex or sphere pole(s)), which are true intersection
    //            points lie out of the domain [theTheta1, theTheta2] of the ALine.
    //           This function corrects these boundaries to make them be included
    //            in the domain of the ALine.
    //           Parameters Theta1 and Theta2 must be initialized
    //            before calling this function.
    //=======================================================================
    // A template in OCCT, instantiated for gp_Cylinder only.
    private static void AddSpecialPoints(IntAna_Quadric theQuad,
                                         in Cylinder theGpObj,
                                         ref double theTheta1,
                                         ref double theTheta2)
    {
        double aPeriod = M_PI + M_PI;
        NCollection_List<Point3D> aLSP = theQuad.InternalSpecialPoints();

        if (aLSP.IsEmpty())
        {
            return;
        }

        double aU = 0.0, aV = 0.0;
        double aMaxDelta = 0.0;
        for (NCollection_List<Point3D>.Iterator anItr = new(aLSP); anItr.More(); anItr.Next())
        {
            Point3D aPt = anItr.Value();
            global::Kerf.Engine.Geometry.ElSLib.Parameters(theGpObj, aPt, out aU, out aV);
            Point3D aPProj = global::Kerf.Engine.Geometry.ElSLib.Value(aU, aV, theGpObj);

            if (aPt.SquareDistance(aPProj) > global::Kerf.Engine.Foundation.Precision.SquareConfusion())
            {
                // aPt is not an intersection point
                continue;
            }

            double aDelta1 = Min(aU - theTheta1, 0.0), aDelta2 = Max(aU - theTheta2, 0.0);

            if (aDelta1 < -M_PI)
            {
                // Must be aDelta1 = std::min(aU - theTheta1 + aPeriod, 0.0).
                // But aU - theTheta1 + aPeriod >= 0 always.
                aDelta1 = 0.0;
            }

            if (aDelta2 > M_PI)
            {
                // Must be aDelta2 = std::max(aU - theTheta2 - aPeriod, 0.0).
                // But aU - theTheta2 - aPeriod <= 0 always.
                aDelta2 = 0.0;
            }

            double aDelta = Max(-aDelta1, aDelta2);
            aMaxDelta = Max(aMaxDelta, aDelta);
        }

        if (aMaxDelta != 0.0)
        {
            theTheta1 -= aMaxDelta;
            theTheta2 += aMaxDelta;
            if ((theTheta2 - theTheta1) > aPeriod)
            {
                theTheta2 = theTheta1 + aPeriod;
            }
        }
    }

    //=================================================================================================
    // class : TrigonometricRoots
    // purpose: Internal class (provides sorted roots of a trigonometric polynomial)
    //=================================================================================================
    // IntAna's own copy (Extrema_ExtElC has another, which differs: decision 120).
    private sealed class TrigonometricRoots
    {
        private readonly double[] Roots = new double[4];
        private readonly bool done;
        private readonly int NbRoots; // OCCT: uninitialised when MTFR fails or has infinite roots (0 here)
        private readonly bool infinite_roots;

        public TrigonometricRoots(double CC,
                                  double SC,
                                  double C,
                                  double S,
                                  double Cte,
                                  double Binf,
                                  double Bsup)
        {
            infinite_roots = false;
            int i, j, SvNbRoots;
            bool Triee;
            double PIpPI = M_PI + M_PI;
            //
            done = false;
            //
            //-- F= AA*CN*CN+2*BB*CN*SN+CC*CN+DD*SN+EE;
            TrigonometricFunctionRoots MTFR = new(CC, SC, C, S, Cte, Binf, Bsup);
            if (!MTFR.IsDone)
            {
                return;
            }
            //
            done = true;
            if (MTFR.InfiniteRoots())
            {
                infinite_roots = true;
                return;
            }
            //
            NbRoots = MTFR.SolutionCount;
            //
            for (i = 0; i < NbRoots; ++i)
            {
                Roots[i] = MTFR.Value(i + 1);
                if (Roots[i] < 0.0)
                {
                    Roots[i] += PIpPI;
                }
                if (Roots[i] > PIpPI)
                {
                    Roots[i] -= PIpPI;
                }
            }
            //
            //-- The direct search gives unreliable results.
            SvNbRoots = NbRoots;
            for (i = 0; i < SvNbRoots; ++i)
            {
                double y;
                double co = Math.Cos(Roots[i]);
                double si = Math.Sin(Roots[i]);
                y = co * (CC * co + (SC + SC) * si + C) + S * si + Cte;
                if (Math.Abs(y) > 1e-8)
                {
                    done = false;
                    return; //-- le 1er avril 98
                }
            }
            //
            do
            {
                Triee = true;
                for (i = 1, j = 0; i < SvNbRoots; ++i, ++j)
                {
                    if (Roots[i] < Roots[j])
                    {
                        Triee = false;
                        double t = Roots[i];
                        Roots[i] = Roots[j];
                        Roots[j] = t;
                    }
                }
            } while (!Triee);
            //
            infinite_roots = false;
            //
            if (NbRoots == 0)
            { //--!!!!! Detection du cas Pol = Cte ( 1e-50 ) !!!!
                if ((Math.Abs(CC) + Math.Abs(SC) + Math.Abs(C) + Math.Abs(S)) < 1e-10)
                {
                    if (Math.Abs(Cte) < 1e-10)
                    {
                        infinite_roots = true;
                    }
                }
            }
        }

        // IsDone
        public bool IsDone() => done;

        // IsARoot
        public bool IsARoot(double u)
        {
            int i;
            double aEps = RealEpsilon();
            double PIpPI = M_PI + M_PI;
            //
            for (i = 0; i < NbRoots; ++i)
            {
                if (Math.Abs(u - Roots[i]) <= aEps)
                {
                    return true;
                }
                if (Math.Abs(u - Roots[i] - PIpPI) <= aEps)
                {
                    return true;
                }
            }
            return false;
        }

        // NbSolutions
        public int NbSolutions()
        {
            if (!done)
            {
                throw new NotDoneException();
            }
            return NbRoots;
        }

        // InfiniteRoots
        public bool InfiniteRoots()
        {
            if (!done)
            {
                throw new NotDoneException();
            }
            return infinite_roots;
        }

        // Value
        public double Value(int n)
        {
            if ((!done) || (n > NbRoots))
            {
                throw new NotDoneException();
            }
            return Roots[n - 1];
        }
    }

    //=================================================================================================
    // class    : MyTrigonometricFunction
    // purpose  : Internal class: provides Value and Derivative for a
    //            trigonometric polynomial
    //=================================================================================================
    private readonly struct MyTrigonometricFunction
    {
        private readonly double CC, SS, SC, S, C, Cte;

        //
        public MyTrigonometricFunction(double xCC,
                                       double xSS,
                                       double xSC,
                                       double xC,
                                       double xS,
                                       double xCte)
        {
            CC = xCC;
            SS = xSS;
            SC = xSC;
            S = xS;
            C = xC;
            Cte = xCte;
        }

        public double Value(double U)
        {
            double sinus, cosinus, aRet;
            //
            sinus = Math.Sin(U);
            cosinus = Math.Cos(U);
            aRet = CC * cosinus * cosinus + SS * sinus * sinus
                   + 2.0 * (sinus * (SC * cosinus + S) + cosinus * C) + Cte;
            //
            return aRet;
        }

        //
        public double Derivative(double U)
        {
            double sinus, cosinus;
            //
            sinus = Math.Sin(U);
            cosinus = Math.Cos(U);
            //
            return (2.0
                    * ((sinus * cosinus) * (SS - CC) + S * cosinus - C * sinus
                       + SC * (cosinus * cosinus - sinus * sinus)));
        }
    }

    //////////

    /// <summary>The empty algorithm.</summary>
    public IntAna_IntQuadQuad()
    {
        done = false;
        identical = false;
        NbCurves = 0;
        Nbpoints = 0;
        myNbMaxCurves = 12;
        myEpsilon = 0.00000001;
        myEpsilonCoeffPolyNull = 0.00000001;
        Array.Clear(nextcurve);
        Array.Clear(previouscurve);
    }

    //=================================================================================================
    // function : IntAna_IntQuadQuad::IntAna_IntQuadQuad
    // purpose  : Intersection Cylinder - Quadric
    //=================================================================================================

    /// <summary>The intersection between a cylinder and a quadric. Tol is not used.</summary>
    public IntAna_IntQuadQuad(in Cylinder Cyl, IntAna_Quadric Quad, double Tol)
    {
        myNbMaxCurves = 12;
        myEpsilon = 0.00000001;
        myEpsilonCoeffPolyNull = 0.00000001;
        Perform(Cyl, Quad, Tol);
    }

    //=================================================================================================
    // function : Perform
    // purpose  : Intersection Cylinder - Quadric
    //=================================================================================================

    /// <summary>Intersects a cylinder and a quadric. Tol is not used.</summary>
    public void Perform(in Cylinder Cyl, IntAna_Quadric Quad, double Tol)
    {
        _ = Tol;
        done = true;
        identical = false;
        NbCurves = 0;
        Nbpoints = 0;
        //
        bool UN_SEUL_Z_PAR_THETA, DEUX_Z_PAR_THETA, Z_POSITIF, Z_INDIFFERENT, Z_NEGATIF;
        //
        UN_SEUL_Z_PAR_THETA = false;
        DEUX_Z_PAR_THETA = true;
        Z_POSITIF = true;
        Z_INDIFFERENT = true;
        Z_NEGATIF = false;
        //
        double Qxx, Qyy, Qzz, Qxy, Qxz, Qyz, Qx, Qy, Qz, Q1, aRealEpsilon, RCyl, R2;
        double PIpPI = M_PI + M_PI;
        //
        for (int raz = 0; raz < myNbMaxCurves; raz++)
        {
            previouscurve[raz] = nextcurve[raz] = 0;
        }
        //
        RCyl = Cyl.Radius();
        aRealEpsilon = RealEpsilon();
        //----------------------------------------------------------------------
        //-- Coefficients of the quadric:
        //--      2       2       2
        //-- Qxx x + Qyy y + Qzz z + 2 ( Qxy x y + Qxz x z + Qyz y z )
        //-- + 2 ( Qx x + Qy y + Qz z ) + Q1
        //----------------------------------------------------------------------
        Quad.Coefficients(out Qxx, out Qyy, out Qzz, out Qxy, out Qxz, out Qyz, out Qx, out Qy, out Qz, out Q1);

        //----------------------------------------------------------------------
        //-- Writing the quadric coefficients in the reference frame
        //-- associated with the Cylinder
        //--                 Cyl.Position() -> Ax2
        //----------------------------------------------------------------------
        Quad.NewCoefficients(ref Qxx, ref Qyy, ref Qzz, ref Qxy, ref Qxz, ref Qyz, ref Qx, ref Qy, ref Qz, ref Q1,
                             Cyl.Position());

        //----------------------------------------------------------------------
        //-- Parameterization of Cylinder Cyl:
        //--     X = Rcyl * std::cos(Theta)
        //--     Y = Rcyl * std::sin(Theta)
        //--     Z = Z
        //----------------------------------------------------------------------
        //-- Gives an equation of the form:
        //--   F(std::sin(Theta),std::cos(Theta),ZCyl) = 0
        //--   (Quadratic equation in ZCyl)
        //--    ZCyl**2  CoeffZ2(Theta) + ZCyl CoeffZ1(Theta) + CoeffZ0(Theta)
        //----------------------------------------------------------------------
        //-- CoeffZ0 = Q1 + 2*Qx*RCyl*Cos[Theta] + Qxx*RCyl^2*Cos[Theta]^2
        //--           2*RCyl*Sin[Theta]* ( Qy + Qxy*RCyl*Cos[Theta]);
        //--           Qyy*RCyl^2*Sin[Theta]^2;
        //-- CoeffZ1 =2.0 * ( Qz + RCyl*(Qxz*Cos[Theta] + Sin[Theta]*Qyz)) ;
        //-- CoeffZ2 =  Qzz;
        //-- !!!! Warning: if normalizing by Qzz to detect the linear case
        //----------------------------------------------------------------------
        //-- We search for the Theta roots of the discriminant of this equation:
        //-- DIS(Theta) = C_1 + C_SS std::sin()**2 + C_CC std::cos()**2 + 2 C_SC std::sin() std::cos()
        //--                  + 2 C_S  std::sin()    + 2 C_C std::cos()
        //--
        //-- If Qzz = 0   Then we solve Z=Fct(Theta) on the Theta domain
        //----------------------------------------------------------------------

        if (Math.Abs(Qzz) < myEpsilonCoeffPolyNull)
        {
            done = false; //-- le 12 mars 98
            return;
        }
        else
        { // #0
            //----------------------------------------------------------------------
            //--- Case where F(Z,Theta) is quadratic in Z                       ----
            //----------------------------------------------------------------------
            R2 = RCyl * RCyl;

            double C_1 = Qz * Qz - Qzz * Q1;
            double C_SS = R2 * (Qyz * Qyz - Qyy * Qzz);
            double C_CC = R2 * (Qxz * Qxz - Qxx * Qzz);
            double C_S = RCyl * (Qyz * Qz - Qy * Qzz);
            double C_C = RCyl * (Qxz * Qz - Qx * Qzz);
            double C_SC = R2 * (Qxz * Qyz - Qxy * Qzz);
            //
            MyTrigonometricFunction MTF = new(C_CC, C_SS, C_SC, C_C, C_S, C_1);
            TrigonometricRoots PolDIS = new(C_CC - C_SS, C_SC, C_C + C_C, C_S + C_S, C_1 + C_SS, 0.0, PIpPI);
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            if (!PolDIS.IsDone())
            {
                done = false;
                return;
            }
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            if (PolDIS.InfiniteRoots())
            {
                TheCurve[0].SetCylinderQuadValues(Cyl, Qxx, Qyy, Qzz, Qxy, Qxz, Qyz, Qx, Qy, Qz, Q1,
                                                  myEpsilon, 0.0, PIpPI, UN_SEUL_Z_PAR_THETA, Z_POSITIF);
                TheCurve[1].SetCylinderQuadValues(Cyl, Qxx, Qyy, Qzz, Qxy, Qxz, Qyz, Qx, Qy, Qz, Q1,
                                                  myEpsilon, 0.0, PIpPI, UN_SEUL_Z_PAR_THETA, Z_NEGATIF);
                NbCurves = 2;
            }

            else
            { // #1
                //---------------------------------------------------------------
                //-- The search for zeros of DIS has succeeded
                //---------------------------------------------------------------
                int nbsolDIS = PolDIS.NbSolutions();
                if (nbsolDIS == 0)
                {
                    //--------------------------------------------------------------
                    //-- The Discriminant has a constant sign:
                    //--
                    //-- If Positive ---> 2 Curves
                    //-- Otherwise   ---> No solution
                    //--------------------------------------------------------------
                    if (MTF.Value(M_PI) >= -aRealEpsilon)
                    {
                        TheCurve[0].SetCylinderQuadValues(Cyl, Qxx, Qyy, Qzz, Qxy, Qxz, Qyz, Qx, Qy, Qz, Q1,
                                                          myEpsilon, 0.0, PIpPI, UN_SEUL_Z_PAR_THETA, Z_POSITIF);
                        TheCurve[1].SetCylinderQuadValues(Cyl, Qxx, Qyy, Qzz, Qxy, Qxz, Qyz, Qx, Qy, Qz, Q1,
                                                          myEpsilon, 0.0, PIpPI, UN_SEUL_Z_PAR_THETA, Z_NEGATIF);

                        NbCurves = 2;
                    }
                    else
                    {
                        //------------------------------------------------------------
                        //-- The Discriminant is always negative
                        //------------------------------------------------------------
                        NbCurves = 0;
                    }
                }
                else
                { // #2
                    //------------------------------------------------------------
                    //-- The Discriminant has roots
                    //-- (The Discriminant is a periodic function, hence ...)
                    //------------------------------------------------------------
                    if (nbsolDIS == 1)
                    {
                        //------------------------------------------------------------
                        //-- Tangent point for this Theta solution
                        //-- If Discriminant sign >= 0 for all Theta
                        //--     Then
                        //--       Solution curve for the positive part
                        //--       between the 2 roots (here the entire domain)
                        //-- Otherwise only a tangent point
                        //------------------------------------------------------------
                        if (MTF.Value(PolDIS.Value(1) + M_PI) >= -aRealEpsilon)
                        {
                            //------------------------------------------------------------
                            //-- We have a tangent point + a solution curve
                            //------------------------------------------------------------
                            TheCurve[0].SetCylinderQuadValues(Cyl, Qxx, Qyy, Qzz, Qxy, Qxz, Qyz, Qx, Qy, Qz, Q1,
                                                              myEpsilon, 0.0, PIpPI, UN_SEUL_Z_PAR_THETA, Z_POSITIF);
                            TheCurve[1].SetCylinderQuadValues(Cyl, Qxx, Qyy, Qzz, Qxy, Qxz, Qyz, Qx, Qy, Qz, Q1,
                                                              myEpsilon, 0.0, PIpPI, UN_SEUL_Z_PAR_THETA, Z_NEGATIF);

                            NbCurves = 2;
                        }
                        else
                        {
                            //------------------------------------------------------------
                            //-- We simply have a tangent point
                            //------------------------------------------------------------
                            //--double Theta = PolDIS(1);
                            //--double SecPar= -0.5 * MTFZ1.Value(Theta) / MTFZ2.Value(Theta);
                            //--Thepoints[Nbpoints] =
                            // ElSLib::CylinderValue(Theta,SecPar,Cyl.Position(),Cyl.Radius());
                            //--Nbpoints++;
                            NbCurves = 0;
                        }
                    }
                    else
                    { // #3
                        //------------------------------------------------------------
                        //-- We detect   :  Double roots
                        //--              :  Positive zones [modulo 2 PI]
                        //-- The solution curves will be obtained for the values
                        //-- of Theta where Discriminant(Theta) > 0 (>=0? at boundary)
                        //-- By solving the implicit equation with fixed Theta
                        //------------------------------------------------------------
                        //-- If everything is a solution, then we are on an iso
                        //-- This case should be handled upstream
                        //------------------------------------------------------------
                        //-- We build the function that, given a Theta, computes
                        //-- the corresponding Z+ and Z-.
                        //-------------------------------------------------------------

                        //-------------------------------------------------------------
                        //-- Computation of intervals where Discriminant >= 0
                        //-- At most nbsol intervals (in practice 2)
                        //--  (1,2) (2,3) .. (nbsol,1+2PI)
                        //-------------------------------------------------------------
                        int i;
                        double Theta1, Theta2, Theta3, autrepar, qwet;
                        bool UnPtTg = false;
                        //
                        NbCurves = 0;
                        if (nbsolDIS == 2)
                        {
                            for (i = 1; i <= nbsolDIS; i++)
                            {
                                Theta1 = PolDIS.Value(i);
                                Theta2 = (i < nbsolDIS) ? PolDIS.Value(i + 1) : (PolDIS.Value(1) + PIpPI);
                                //----------------------------------------------------------------
                                //-- We detect double roots
                                //-- If there are only 2 roots then we take the entire interval
                                //----------------------------------------------------------------
                                if (Math.Abs(Theta2 - Theta1) <= aRealEpsilon)
                                {
                                    UnPtTg = true;
                                    autrepar = Theta1 - 0.1;
                                    if (autrepar < 0.0)
                                    {
                                        autrepar = Theta1 + 0.1;
                                    }
                                    //
                                    qwet = MTF.Value(autrepar);
                                    if (qwet >= 0.0)
                                    {
                                        double aParam = Theta1 + PIpPI;
                                        AddSpecialPoints(Quad, Cyl, ref Theta1, ref aParam);
                                        TheCurve[NbCurves].SetCylinderQuadValues(Cyl, Qxx, Qyy, Qzz, Qxy, Qxz, Qyz, Qx, Qy, Qz,
                                                                                 Q1, myEpsilon, Theta1, aParam,
                                                                                 UN_SEUL_Z_PAR_THETA, Z_POSITIF);
                                        NbCurves++;
                                        TheCurve[NbCurves].SetCylinderQuadValues(Cyl, Qxx, Qyy, Qzz, Qxy, Qxz, Qyz, Qx, Qy, Qz,
                                                                                 Q1, myEpsilon, Theta1, aParam,
                                                                                 UN_SEUL_Z_PAR_THETA, Z_NEGATIF);
                                        NbCurves++;
                                    }
                                }
                            }
                        }

                        for (i = 1; !UnPtTg && (i <= nbsolDIS); i++)
                        {
                            Theta1 = PolDIS.Value(i);
                            Theta2 = (i < nbsolDIS) ? PolDIS.Value(i + 1) : (PolDIS.Value(1) + PIpPI);
                            //----------------------------------------------------------------
                            //-- On detecte les racines doubles
                            //-- Si il n y a que 2 racines alors on prend tout l interval
                            //----------------------------------------------------------------
                            if (Math.Abs(Theta2 - Theta1) <= 1e-12)
                            {
                                //-- std::cout<<"\n####### Un Point de Tangence en Theta = "<<Theta1<<std::endl;
                                //-- i++;
                            }
                            else
                            { //-- We avoid numerical issues (everything has the same sign between the roots)
                                qwet = MTF.Value(0.5 * (Theta1 + Theta2)) + MTF.Value(0.4 * Theta1 + 0.6 * Theta2)
                                       + MTF.Value(0.6 * Theta1 + 0.4 * Theta2);
                                if (qwet >= 0.0)
                                {
                                    //------------------------------------------------------------
                                    //-- We are positive starting from Theta1
                                    //-- The interval Theta1, Theta2 is retained
                                    //------------------------------------------------------------

                                    //-- October 8, 1997:
                                    //-- PB: Root at 0    pi/2 pi/2  and 2pi
                                    //-- We create 2 curves: 0    -> pi/2    2zpertheta
                                    //--                     pi/2 -> 2pi     2zpertheta
                                    //--
                                    //-- The curve 0 pi/2 passes through the double tangent point pi/2
                                    //-- so this curve must be split in 2
                                    //--
                                    Theta3 = ((i + 1) < nbsolDIS) ? PolDIS.Value(i + 2) : (PolDIS.Value(1) + PIpPI);
                                    // ft
                                    if ((Theta3 - Theta2) < 5.0e-8)
                                    {
                                        //
                                        AddSpecialPoints(Quad, Cyl, ref Theta1, ref Theta2);
                                        TheCurve[NbCurves].SetCylinderQuadValues(Cyl, Qxx, Qyy, Qzz, Qxy, Qxz, Qyz, Qx, Qy, Qz,
                                                                                 Q1, myEpsilon, Theta1, Theta2,
                                                                                 UN_SEUL_Z_PAR_THETA, Z_POSITIF);
                                        NbCurves++;
                                        TheCurve[NbCurves].SetCylinderQuadValues(Cyl, Qxx, Qyy, Qzz, Qxy, Qxz, Qyz, Qx, Qy, Qz,
                                                                                 Q1, myEpsilon, Theta1, Theta2,
                                                                                 UN_SEUL_Z_PAR_THETA, Z_NEGATIF);
                                        NbCurves++;
                                    }
                                    else
                                    {
                                        AddSpecialPoints(Quad, Cyl, ref Theta1, ref Theta2);
                                        TheCurve[NbCurves].SetCylinderQuadValues(Cyl, Qxx, Qyy, Qzz, Qxy, Qxz, Qyz, Qx, Qy, Qz,
                                                                                 Q1, myEpsilon, Theta1, Theta2,
                                                                                 DEUX_Z_PAR_THETA, Z_INDIFFERENT);
                                        NbCurves++;
                                    }
                                } // if(qwet >= 0.)
                            } // else {  //-- On evite les pbs numerique ...
                        } // for(i=1; UnPtTg == (false) && (i<=nbsolDIS) ; i++) {
                    } // else { // #3
                } // else { //#2
            } // else { //#1
        } // else { //#0
    }

    /// <summary>The intersection between a cone and a quadric. Tol is not used.</summary>
    public IntAna_IntQuadQuad(in Cone Cone, IntAna_Quadric Quad, double Tol)
    {
        myNbMaxCurves = 12;
        myEpsilon = 0.00000001;
        myEpsilonCoeffPolyNull = 0.00000001;
        Perform(Cone, Quad, Tol);
    }

    /// <summary>Intersects a cone and a quadric. Tol is not used.</summary>
    public void Perform(in Cone Cone, IntAna_Quadric Quad, double Tol)
    {
        _ = Tol;
        //
        bool UN_SEUL_Z_PAR_THETA, Z_POSITIF, Z_NEGATIF;
        //
        UN_SEUL_Z_PAR_THETA = false;
        Z_POSITIF = true;
        Z_NEGATIF = false;
        //
        int i;
        double Qxx, Qyy, Qzz, Qxy, Qxz, Qyz, Qx, Qy, Qz, Q1;
        double Theta1, Theta2, TgAngle;
        double PIpPI = M_PI + M_PI;
        //
        done = true;
        identical = false;
        NbCurves = 0;
        Nbpoints = 0;
        //
        for (i = 0; i < myNbMaxCurves; ++i)
        {
            previouscurve[i] = 0;
            nextcurve[i] = 0;
        }
        //
        Quad.Coefficients(out Qxx, out Qyy, out Qzz, out Qxy, out Qxz, out Qyz, out Qx, out Qy, out Qz, out Q1);
        //
        CoordinateSystem tAx3 = Cone.Position();
        tAx3.SetLocation(Cone.Apex());
        Quad.NewCoefficients(ref Qxx, ref Qyy, ref Qzz, ref Qxy, ref Qxz, ref Qyz, ref Qx, ref Qy, ref Qz, ref Q1, tAx3);
        //
        TgAngle = 1.0 / (Math.Tan(Cone.SemiAngle()));
        //
        // The parametrization of the Cone
        //
        // x= z*tan(beta)*cos(t)
        // y= z*tan(beta)*sin(t)
        // z=z
        //
        // The intersection curves are defined by the equation
        //
        //                        2
        //          f(z,t)= A(t)*z + B(t)*z + C(t)=0
        //
        //
        // 1. Try to solve A(t)=0 -> PolZ2
        //
        int nbsol, nbsol1, nbsolZ2;
        double Z2CC, Z2SS, Z2Cte, Z2SC, Z2C, Z2S;
        double Z1CC, Z1SS, Z1Cte, Z1SC, Z1C, Z1S;
        double C_1, C_SS, C_CC, C_S, C_C, C_SC;
        //
        Z2CC = Qxx;
        Z2SS = Qyy;
        Z2Cte = Qzz * TgAngle * TgAngle;
        Z2SC = Qxy;
        Z2C = (TgAngle) * Qxz;
        Z2S = (TgAngle) * Qyz;
        //
        TrigonometricRoots PolZ2 = new(Z2CC - Z2SS, Z2SC, Z2C + Z2C, Z2S + Z2S, Z2Cte + Z2SS, 0.0, PIpPI);
        if (!PolZ2.IsDone())
        {
            done = !done;
            return;
        }
        //
        // MyTrigonometricFunction MTF2(Z2CC,Z2SS,Z2SC,Z2C,Z2S,Z2Cte);
        nbsolZ2 = PolZ2.NbSolutions();
        //
        // 2. Try to solve B(t)=0  -> PolZ1
        //
        Z1Cte = 2.0 * (TgAngle) * Qz;
        Z1SS = 0.0;
        Z1CC = 0.0;
        Z1S = Qy;
        Z1C = Qx;
        Z1SC = 0.0;
        //
        TrigonometricRoots PolZ1 = new(Z1CC - Z1SS, Z1SC, Z1C + Z1C, Z1S + Z1S, Z1Cte + Z1SS, 0.0, PIpPI);
        if (!PolZ1.IsDone())
        {
            done = !done;
            return;
        }
        MyTrigonometricFunction MTFZ1 = new(Z1CC, Z1SS, Z1SC, Z1C, Z1S, Z1Cte);
        //
        nbsol1 = PolZ1.NbSolutions();
        if (PolZ2.InfiniteRoots())
        { // i.e A(t)=0 for any t
            if (!PolZ1.InfiniteRoots())
            {
                if (nbsol1 == 0)
                {
                    //-- B(t)*z + C(t) = 0    with C(t) != 0
                    TheCurve[0].SetConeQuadValues(Cone, Qxx, Qyy, Qzz, Qxy, Qxz, Qyz, Qx, Qy, Qz, Q1,
                                                  myEpsilon, 0.0, PIpPI, UN_SEUL_Z_PAR_THETA, Z_POSITIF);
                    NbCurves = 1;
                }
            }
            else
            {
                if (Math.Abs(Q1) <= myEpsilon)
                {
                    done = !done;
                    return;
                }
                else
                {
                    //-- No solutions
                }
            }
            return;
        }
        //
        // else { //#5
        //
        //                2
        //-- f(z,t)=A(t)*z + B(t)*z + C(t)=0   where A is not always zero
        //
        //                                              2
        // 3. Try to explore s.c. Discriminant:  D(t)=B(t)-4*A(t)*C(t) => Pol
        //
        // The function D(t) is just a formula that has sense for quadratic
        // equation above.
        // For cases when A(t)=0 (say at t=ti, t=tj. etc) the equation
        // will be
        //
        //     f(z,t)=B(t)*z + C(t)=0, where B(t)!=0,
        //
        // and the D(t) is nonsense for it.
        //
        C_1 = TgAngle * TgAngle * (Qz * Qz - Qzz * Q1);
        C_SS = Qy * Qy - Qyy * Q1;
        C_CC = Qx * Qx - Qxx * Q1;
        C_S = TgAngle * (Qy * Qz - Qyz * Q1);
        C_C = TgAngle * (Qx * Qz - Qxz * Q1);
        C_SC = Qx * Qy - Qxy * Q1;
        //
        TrigonometricRoots Pol = new(C_CC - C_SS, C_SC, C_C + C_C, C_S + C_S, C_1 + C_SS, 0.0, PIpPI);
        if (!Pol.IsDone())
        {
            done = !done;
            return;
        }
        //
        nbsol = Pol.NbSolutions();
        MyTrigonometricFunction MTF = new(C_CC, C_SS, C_SC, C_C, C_S, C_1);
        //
        if (Pol.InfiniteRoots())
        {
            //                             2
            //         f(z,t)= (z(t)-zo(t)) = 0       Discriminant(t)=0 pour tout t
            //
            TheCurve[0].SetConeQuadValues(Cone, Qxx, Qyy, Qzz, Qxy, Qxz, Qyz, Qx, Qy, Qz, Q1,
                                          myEpsilon, 0.0, PIpPI, UN_SEUL_Z_PAR_THETA, Z_POSITIF);
            TheCurve[1].SetConeQuadValues(Cone, Qxx, Qyy, Qzz, Qxy, Qxz, Qyz, Qx, Qy, Qz, Q1,
                                          myEpsilon, 0.0, PIpPI, UN_SEUL_Z_PAR_THETA, Z_NEGATIF);
            NbCurves = 2;
            return;
        }
        // else {//#4
        //                      2
        //         f(z,t)=A(t)*z + B(t)*z + C(t)      Discriminant(t) != 0
        //
        if (nbsol == 0 && (MTF.Value(M_PI) < 0.0))
        {
            //-- Discriminant has constant negative sign
            return;
        }
        // else {//# 3
        //
        //-- We handle the case: Discriminant(t) > 0 for all t by simulating
        //   a double root at 0
        bool DiscriminantConstantPositif = false;
        if (nbsol == 0)
        {
            nbsol = 1;
            DiscriminantConstantPositif = true;
        }
        //----------------------------------------------------------------------
        //-- The discriminant has at least one root ( -> Tangent Point )
        //-- or is constant positive.
        //----------------------------------------------------------------------
        for (i = 1; i <= nbsol; ++i)
        {
            if (DiscriminantConstantPositif)
            {
                Theta1 = 0.0;
                Theta2 = PIpPI - myEpsilon;
            }
            else
            {
                Theta1 = Pol.Value(i);
                Theta2 = (i < nbsol) ? Pol.Value(i + 1) : (Pol.Value(1) + PIpPI);
            }
            //
            if (Math.Abs(Theta2 - Theta1) <= myEpsilon)
            {
                done = false;
                return; // !!! pkv
            }
            // else {// #2
            double qwet = MTF.Value(0.5 * (Theta1 + Theta2)) + MTF.Value(0.4 * Theta1 + 0.6 * Theta2)
                          + MTF.Value(0.6 * Theta1 + 0.4 * Theta2);
            if (qwet < 0.0)
            {
                continue;
            }
            //---------------------------------------------------------------------
            //-- We have solutions between Theta1 and Theta2
            //---------------------------------------------------------------------

            //---------------------------------------------------------------------
            //-- We subdivide if necessary Theta1-->Theta2
            //-- into Theta1-->t1   t1--->t2   ...  tn--->Theta2
            //--  where t1,t2,...tn are roots of A(t)
            //--
            //-- Only the Z_NEGATIF curve is affected
            //----------------------------------------------------------------------
            bool RacinesdePolZ2DansTheta1Theta2;
            int i2;
            double r;
            //
            // nbsolZ2=PolZ2.NbSolutions();
            RacinesdePolZ2DansTheta1Theta2 = false;
            for (i2 = 1; i2 <= nbsolZ2 && !RacinesdePolZ2DansTheta1Theta2; ++i2)
            {
                r = PolZ2.Value(i2);
                if (r > Theta1 && r < Theta2)
                {
                    RacinesdePolZ2DansTheta1Theta2 = true;
                }
                else
                {
                    r += PIpPI;
                    if (r > Theta1 && r < Theta2)
                    {
                        RacinesdePolZ2DansTheta1Theta2 = true;
                    }
                }
            }
            //
            if (!RacinesdePolZ2DansTheta1Theta2)
            {
                //--------------------------------------------------------------------
                //-- No infinite branches
                TheCurve[NbCurves].SetConeQuadValues(Cone, Qxx, Qyy, Qzz, Qxy, Qxz, Qyz, Qx, Qy, Qz, Q1,
                                                     myEpsilon, Theta1, Theta2, UN_SEUL_Z_PAR_THETA, Z_POSITIF);
                NbCurves++;
                TheCurve[NbCurves].SetConeQuadValues(Cone, Qxx, Qyy, Qzz, Qxy, Qxz, Qyz, Qx, Qy, Qz, Q1,
                                                     myEpsilon, Theta1, Theta2, UN_SEUL_Z_PAR_THETA, Z_NEGATIF);
                NbCurves++;
            }

            else
            { // #1
                bool NoChanges;
                double NewMin, NewMax, to;
                //
                NewMin = Theta1;
                NewMax = Theta2;
                NoChanges = true;
                //
                for (i2 = 1; i2 <= (nbsolZ2 + nbsolZ2); ++i2)
                {
                    if (i2 > nbsolZ2)
                    {
                        to = PolZ2.Value(i2 - nbsolZ2) + PIpPI;
                    }
                    else
                    {
                        to = PolZ2.Value(i2);
                    }
                    //
                    // to is value of the parameter t where A(t)=0, i.e.
                    // f(z,to)=B(to)*z + C(to)=0, B(to)!=0.
                    //
                    // z=-C(to)/B(to) is one and only
                    // solution in spite of the fact that D(t)>0 for any value of t.
                    //
                    if (to < NewMax && to > NewMin)
                    {
                        //-----------------------------------------------------------------
                        //-- We split at least once the domain Theta1 Theta2
                        //-----------------------------------------------------------------
                        NoChanges = false;
                        TheCurve[NbCurves].SetConeQuadValues(Cone, Qxx, Qyy, Qzz, Qxy, Qxz, Qyz, Qx, Qy, Qz, Q1,
                                                             myEpsilon, NewMin, to, UN_SEUL_Z_PAR_THETA, Z_NEGATIF);
                        //
                        NbCurves++;
                        TheCurve[NbCurves].SetConeQuadValues(Cone, Qxx, Qyy, Qzz, Qxy, Qxz, Qyz, Qx, Qy, Qz, Q1,
                                                             myEpsilon, NewMin, to, UN_SEUL_Z_PAR_THETA, Z_POSITIF);
                        //------------------------------------------------------------
                        //-- A == 0
                        //-- Si B < 0    Alors Infini sur   Z+
                        //-- Sinon             Infini sur   Z-
                        //------------------------------------------------------------
                        if (PolZ2.IsARoot(NewMin))
                        {
                            if (MTFZ1.Value(NewMin) < 0.0)
                            {
                                TheCurve[NbCurves].SetIsFirstOpen(true);
                            }
                            else
                            {
                                TheCurve[NbCurves - 1].SetIsFirstOpen(true);
                            }
                        }
                        if (MTFZ1.Value(to) < 0.0)
                        {
                            TheCurve[NbCurves].SetIsLastOpen(true);
                        }
                        else
                        {
                            TheCurve[NbCurves - 1].SetIsLastOpen(true);
                        }
                        //------------------------------------------------------------
                        NbCurves++;
                        NewMin = to;
                    } // if(to<NewMax && to>NewMin)
                } // for(i2=1; i2<= (nbsolZ2+nbsolZ2) ; ++i2)
                //
                if (NoChanges)
                {
                    TheCurve[NbCurves].SetConeQuadValues(Cone, Qxx, Qyy, Qzz, Qxy, Qxz, Qyz, Qx, Qy, Qz, Q1,
                                                         myEpsilon, Theta1, Theta2, UN_SEUL_Z_PAR_THETA, Z_NEGATIF);
                    NbCurves++;
                    TheCurve[NbCurves].SetConeQuadValues(Cone, Qxx, Qyy, Qzz, Qxy, Qxz, Qyz, Qx, Qy, Qz, Q1,
                                                         myEpsilon, Theta1, Theta2, UN_SEUL_Z_PAR_THETA, Z_POSITIF);
                    //------------------------------------------------------------
                    //-- A == 0
                    //-- If B < 0    Then infinity on   Z+
                    //-- Otherwise         infinity on   Z-
                    //------------------------------------------------------------
                    if (PolZ2.IsARoot(Theta1))
                    {
                        if (MTFZ1.Value(Theta1) < 0.0)
                        {
                            TheCurve[NbCurves].SetIsFirstOpen(true);
                        }
                        else
                        {
                            TheCurve[NbCurves - 1].SetIsFirstOpen(true);
                        }
                    }
                    if (PolZ2.IsARoot(Theta2))
                    {
                        if (MTFZ1.Value(Theta2) < 0.0)
                        {
                            TheCurve[NbCurves].SetIsLastOpen(true);
                        }
                        else
                        {
                            TheCurve[NbCurves - 1].SetIsLastOpen(true);
                        }
                    }
                    //------------------------------------------------------------
                    NbCurves++;
                } // if(NoChanges) {
                else
                { // #0
                    TheCurve[NbCurves].SetConeQuadValues(Cone, Qxx, Qyy, Qzz, Qxy, Qxz, Qyz, Qx, Qy, Qz, Q1,
                                                         myEpsilon, NewMin, Theta2, UN_SEUL_Z_PAR_THETA, Z_NEGATIF);
                    NbCurves++;
                    TheCurve[NbCurves].SetConeQuadValues(Cone, Qxx, Qyy, Qzz, Qxy, Qxz, Qyz, Qx, Qy, Qz, Q1,
                                                         myEpsilon, NewMin, Theta2, UN_SEUL_Z_PAR_THETA, Z_POSITIF);
                    //------------------------------------------------------------
                    //-- A == 0
                    //-- If B < 0    Then infinity on   Z+
                    //-- Otherwise         infinity on   Z-
                    //------------------------------------------------------------
                    if (PolZ2.IsARoot(NewMin))
                    {
                        if (MTFZ1.Value(NewMin) < 0.0)
                        {
                            TheCurve[NbCurves].SetIsFirstOpen(true);
                        }
                        else
                        {
                            TheCurve[NbCurves - 1].SetIsFirstOpen(true);
                        }
                    }
                    if (PolZ2.IsARoot(Theta2))
                    {
                        if (MTFZ1.Value(Theta2) < 0.0)
                        {
                            TheCurve[NbCurves].SetIsLastOpen(true);
                        }
                        else
                        {
                            TheCurve[NbCurves - 1].SetIsLastOpen(true);
                        }
                    }
                    //------------------------------------------------------------

                    NbCurves++;
                } // else {// #0
            } // else { //#1
        } // for(i=1; i<=nbsol ; i++) {
        //}//else { //#5
        InternalSetNextAndPrevious();
    }

    //=================================================================================================
    // function : InternalSetNextAndPrevious
    // purpose  : End-to-end connection of curves
    //--    - Uses the field: IsFirstOpen
    //--    -                 IsLastOpen
    //-- No verification if these fields return True.
    //--
    //--
    //-- 1: Test if  End(C1)   = Start(C2)   ->N(C1) and P(C2)
    //-- 2:          Start(C1) = End(C2)     ->P(C1) and N(C2)
    //=================================================================================================

    /// <summary>Sets the next and previous fields (only after a cone intersection, as in OCCT).</summary>
    protected void InternalSetNextAndPrevious()
    {
        bool NotLastOpenC2, NotFirstOpenC2;
        int c1, c2;
        double aEps, aEPSILON_DISTANCE;
        double DInfC1, DSupC1, DInfC2, DSupC2;
        //
        aEps = 0.0000001;
        aEPSILON_DISTANCE = 0.0000000001;
        //
        for (c1 = 0; c1 < NbCurves; c1++)
        {
            nextcurve[c1] = 0;
            previouscurve[c1] = 0;
        }
        //
        for (c1 = 0; c1 < NbCurves; c1++)
        {
            TheCurve[c1].Domain(out DInfC1, out DSupC1);

            // OCCT: the loop stops at c2 == c1, so only the curves before c1 are compared with it
            for (c2 = 0; (c2 < NbCurves) && (c2 != c1); c2++)
            {
                NotLastOpenC2 = !TheCurve[c2].IsLastOpen();
                NotFirstOpenC2 = !TheCurve[c2].IsFirstOpen();
                TheCurve[c2].Domain(out DInfC2, out DSupC2);
                if (!TheCurve[c1].IsFirstOpen())
                {
                    if (NotLastOpenC2)
                    {
                        if (Math.Abs(DInfC1 - DSupC2) <= aEps
                            && (TheCurve[c1].Value(DInfC1).Distance(TheCurve[c2].Value(DSupC2))
                                < aEPSILON_DISTANCE))
                        {
                            previouscurve[c1] = c2 + 1;
                            nextcurve[c2] = c1 + 1;
                        }
                    }
                    if (NotFirstOpenC2)
                    {
                        if (Math.Abs(DInfC1 - DInfC2) <= aEps
                            && (TheCurve[c1].Value(DInfC1).Distance(TheCurve[c2].Value(DInfC2))
                                < aEPSILON_DISTANCE))
                        {
                            previouscurve[c1] = -(c2 + 1);
                            previouscurve[c2] = -(c1 + 1);
                        }
                    }
                }
                if (!TheCurve[c1].IsLastOpen())
                {
                    if (NotLastOpenC2)
                    {
                        if (Math.Abs(DSupC1 - DSupC2) <= aEps
                            && (TheCurve[c1].Value(DSupC1).Distance(TheCurve[c2].Value(DSupC2))
                                < aEPSILON_DISTANCE))
                        {
                            nextcurve[c1] = -(c2 + 1);
                            nextcurve[c2] = -(c1 + 1);
                        }
                    }
                    if (NotFirstOpenC2)
                    {
                        if (Math.Abs(DSupC1 - DInfC2) <= aEps
                            && (TheCurve[c1].Value(DSupC1).Distance(TheCurve[c2].Value(DInfC2))
                                < aEPSILON_DISTANCE))
                        {
                            nextcurve[c1] = c2 + 1;
                            previouscurve[c2] = c1 + 1;
                        }
                    }
                }
            }
        }
    }

    /// <summary>True if the computation was successful.</summary>
    public bool IsDone() => (done);

    /// <summary>True if the cylinder, the cone or the sphere is identical to the quadric (never set by OCCT).</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public bool IdenticalElements()
    {
        if (!done)
        {
            throw new NotDoneException("IntQuadQuad Not done");
        }
        return (identical);
    }

    /// <summary>The number of curves.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    /// <exception cref="DomainErrorException">The elements are identical.</exception>
    public int NbCurve()
    {
        if (!done)
        {
            throw new NotDoneException("IntQuadQuad Not done");
        }
        if (identical)
        {
            throw new DomainErrorException();
        }
        return (NbCurves);
    }

    /// <summary>The number of contact points (always 0 in OCCT).</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    /// <exception cref="DomainErrorException">The elements are identical.</exception>
    public int NbPnt()
    {
        if (!done)
        {
            throw new NotDoneException("IntQuadQuad Not done");
        }
        if (identical)
        {
            throw new DomainErrorException();
        }
        return (Nbpoints);
    }

    /// <summary>True if the curve I shares its first bound with another curve.</summary>
    /// <exception cref="OutOfRangeException">I is not in [1, NbCurve()].</exception>
    public bool HasPreviousCurve(int I)
    {
        if (!done)
        {
            throw new NotDoneException("IntQuadQuad Not done");
        }
        if (identical)
        {
            throw new DomainErrorException("IntQuadQuad identical");
        }
        if ((I > NbCurves) || (I <= 0))
        {
            throw new OutOfRangeException("Incorrect Curve Number 'HasPrevious Curve'");
        }
        if (previouscurve[I - 1] != 0)
        {
            return true;
        }
        return false;
    }

    /// <summary>True if the curve I shares its last bound with another curve.</summary>
    /// <exception cref="OutOfRangeException">I is not in [1, NbCurve()].</exception>
    public bool HasNextCurve(int I)
    {
        if (!done)
        {
            throw new NotDoneException("IntQuadQuad Not done");
        }
        if (identical)
        {
            throw new DomainErrorException("IntQuadQuad identical");
        }
        if ((I > NbCurves) || (I <= 0))
        {
            throw new OutOfRangeException("Incorrect Curve Number 'HasNextCurve'");
        }
        if (nextcurve[I - 1] != 0)
        {
            return true;
        }
        return (false);
    }

    /// <summary>
    /// The index J of the curve sharing the first bound of the curve I. If theOpposite is true, the first parameters
    /// of I and J give the same point; otherwise the first parameter of I and the last of J do.
    /// </summary>
    /// <exception cref="DomainErrorException">The curve I has no previous curve.</exception>
    public int PreviousCurve(int I, out bool theOpposite)
    {
        if (HasPreviousCurve(I))
        {
            if (previouscurve[I - 1] > 0)
            {
                theOpposite = false;
                return (previouscurve[I - 1]);
            }
            else
            {
                theOpposite = true;
                return (-previouscurve[I - 1]);
            }
        }
        else
        {
            throw new DomainErrorException("Incorrect Curve Number 'PreviousCurve'");
        }
    }

    /// <summary>
    /// The index J of the curve sharing the last bound of the curve I. If theOpposite is true, the last parameters
    /// of I and J give the same point; otherwise the last parameter of I and the first of J do.
    /// </summary>
    /// <exception cref="DomainErrorException">The curve I has no next curve.</exception>
    public int NextCurve(int I, out bool theOpposite)
    {
        if (HasNextCurve(I))
        {
            theOpposite = nextcurve[I - 1] < 0;
            return Math.Abs(nextcurve[I - 1]);
        }
        else
        {
            throw new DomainErrorException("Incorrect Curve Number 'NextCurve'");
        }
    }

    /// <summary>The curve of range i.</summary>
    /// <exception cref="OutOfRangeException">i is not in [1, NbCurve()].</exception>
    public IntAna_Curve Curve(int i)
    {
        if (!done)
        {
            throw new NotDoneException("IntQuadQuad Not done");
        }
        if (identical)
        {
            throw new DomainErrorException("IntQuadQuad identical");
        }
        if ((i <= 0) || (i > NbCurves))
        {
            throw new OutOfRangeException("Incorrect Curve Number");
        }
        return TheCurve[i - 1];
    }

    /// <summary>The point of range i.</summary>
    /// <exception cref="OutOfRangeException">i is not in [1, NbPnt()].</exception>
    public Point3D Point(int i)
    {
        if (!done)
        {
            throw new NotDoneException("IntQuadQuad Not done");
        }
        if (identical)
        {
            throw new DomainErrorException("IntQuadQuad identical");
        }
        if ((i <= 0) || (i > Nbpoints))
        {
            throw new OutOfRangeException("Incorrect Point Number");
        }
        return Thepoints[i - 1];
    }

    /// <summary>
    /// The parameters on the cylinder or cone of the point of range N. Not implemented in OCCT: leaves U1 and U2
    /// unchanged.
    /// </summary>
    public void Parameters(int N, ref double U1, ref double U2)
    {
        // OCCT prints "IntAna_IntQuadQuad::Parameters(...) is not yet implemented" to std::cout (left out)
        _ = N;
        _ = U1;
        _ = U2;
    }
}
