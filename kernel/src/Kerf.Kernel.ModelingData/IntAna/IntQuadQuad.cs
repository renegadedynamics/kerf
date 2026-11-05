// Port of: src/ModelingData/TKGeomBase/IntAna/IntAna_IntQuadQuad.hxx
// Port of: src/ModelingData/TKGeomBase/IntAna/IntAna_IntQuadQuad.lxx
// Port of: src/ModelingData/TKGeomBase/IntAna/IntAna_IntQuadQuad.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Intersection;

/// <summary>
/// The analytic intersection between a cylinder or a cone and another quadric (<see cref="Quadric"/>), used
/// when the geometric intersection (IntAna_QuadQuadGeo) has no geometric solution. The result is curves
/// (<see cref="Curve"/>) and points.
/// </summary>
/// <remarks>An algorithm object (decision 117): construct it with the surfaces, or empty and call <c>Perform</c>.
/// OCCT's debug function <c>DumpCurve</c> is left out.</remarks>
internal class IntQuadQuad
{
    protected bool _done;
    protected bool _identical;
    protected readonly Curve[] _theCurve = NewCurves();
    protected readonly int[] _previouscurve = new int[12];
    protected readonly int[] _nextcurve = new int[12];
    protected int _nbCurves;
    protected int _nbpoints;
    protected readonly Point3D[] _thepoints = [new Point3D(), new Point3D()];
    protected int _nbMaxCurves;
    protected double _epsilon;
    protected double _epsilonCoeffPolyNull;

    // OCCT's `IntAna_Curve TheCurve[12]` default-constructs its elements
    private static Curve[] NewCurves()
    {
        Curve[] curves = new Curve[12];
        for (int i = 0; i < curves.Length; i++)
        {
            curves[i] = new Curve();
        }
        return curves;
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
    private static void AddSpecialPoints(Quadric quad,
                                         in Cylinder3D gpObj,
                                         ref double theta1,
                                         ref double theta2)
    {
        double period = M_PI + M_PI;
        NCollection_List<Point3D> lsp = quad.InternalSpecialPoints();

        if (lsp.IsEmpty())
        {
            return;
        }

        double u = 0.0, v = 0.0;
        double maxDelta = 0.0;
        for (NCollection_List<Point3D>.Iterator itr = new(lsp); itr.More(); itr.Next())
        {
            Point3D pt = itr.Value();
            global::Kerf.Engine.Geometry.ElSLib.Parameters(gpObj, pt, out u, out v);
            Point3D pProj = global::Kerf.Engine.Geometry.ElSLib.Value(u, v, gpObj);

            if (pt.SquareDistanceTo(pProj) > global::Kerf.Engine.Foundation.Precision.SquareConfusion())
            {
                // aPt is not an intersection point
                continue;
            }

            double delta1 = Min(u - theta1, 0.0), delta2 = Max(u - theta2, 0.0);

            if (delta1 < -M_PI)
            {
                // Must be aDelta1 = std::min(aU - theTheta1 + aPeriod, 0.0).
                // But aU - theTheta1 + aPeriod >= 0 always.
                delta1 = 0.0;
            }

            if (delta2 > M_PI)
            {
                // Must be aDelta2 = std::max(aU - theTheta2 - aPeriod, 0.0).
                // But aU - theTheta2 - aPeriod <= 0 always.
                delta2 = 0.0;
            }

            double delta = Max(-delta1, delta2);
            maxDelta = Max(maxDelta, delta);
        }

        if (maxDelta != 0.0)
        {
            theta1 -= maxDelta;
            theta2 += maxDelta;
            if ((theta2 - theta1) > period)
            {
                theta2 = theta1 + period;
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
        private readonly double[] _roots = new double[4];
        private readonly bool _done;
        private readonly int _nbRoots; // OCCT: uninitialised when MTFR fails or has infinite roots (0 here)
        private readonly bool infinite_roots;

        public TrigonometricRoots(double cc,
                                  double sc,
                                  double c,
                                  double s,
                                  double cte,
                                  double binf,
                                  double bsup)
        {
            infinite_roots = false;
            int i, j, svNbRoots;
            bool triee;
            double pIpPI = M_PI + M_PI;
            //
            _done = false;
            //
            //-- F= AA*CN*CN+2*BB*CN*SN+CC*CN+DD*SN+EE;
            TrigonometricFunctionRoots mtfr = new(cc, sc, c, s, cte, binf, bsup);
            if (!mtfr.IsDone)
            {
                return;
            }
            //
            _done = true;
            if (mtfr.InfiniteRoots())
            {
                infinite_roots = true;
                return;
            }
            //
            _nbRoots = mtfr.SolutionCount;
            //
            for (i = 0; i < _nbRoots; ++i)
            {
                _roots[i] = mtfr.Value(i + 1);
                if (_roots[i] < 0.0)
                {
                    _roots[i] += pIpPI;
                }
                if (_roots[i] > pIpPI)
                {
                    _roots[i] -= pIpPI;
                }
            }
            //
            //-- The direct search gives unreliable results.
            svNbRoots = _nbRoots;
            for (i = 0; i < svNbRoots; ++i)
            {
                double y;
                double co = Math.Cos(_roots[i]);
                double si = Math.Sin(_roots[i]);
                y = co * (cc * co + (sc + sc) * si + c) + s * si + cte;
                if (Math.Abs(y) > 1e-8)
                {
                    _done = false;
                    return; //-- le 1er avril 98
                }
            }
            //
            do
            {
                triee = true;
                for (i = 1, j = 0; i < svNbRoots; ++i, ++j)
                {
                    if (_roots[i] < _roots[j])
                    {
                        triee = false;
                        double t = _roots[i];
                        _roots[i] = _roots[j];
                        _roots[j] = t;
                    }
                }
            } while (!triee);
            //
            infinite_roots = false;
            //
            if (_nbRoots == 0)
            { //--!!!!! Detection du cas Pol = Cte ( 1e-50 ) !!!!
                if ((Math.Abs(cc) + Math.Abs(sc) + Math.Abs(c) + Math.Abs(s)) < 1e-10)
                {
                    if (Math.Abs(cte) < 1e-10)
                    {
                        infinite_roots = true;
                    }
                }
            }
        }

        // IsDone
        public bool IsDone => _done;

        // IsARoot
        public bool IsARoot(double u)
        {
            int i;
            double eps = RealEpsilon();
            double pIpPI = M_PI + M_PI;
            //
            for (i = 0; i < _nbRoots; ++i)
            {
                if (Math.Abs(u - _roots[i]) <= eps)
                {
                    return true;
                }
                if (Math.Abs(u - _roots[i] - pIpPI) <= eps)
                {
                    return true;
                }
            }
            return false;
        }

        // NbSolutions
        public int SolutionCount
        {
            get
            {
                if (!_done)
                {
                    throw new NotDoneException();
                }
                return _nbRoots;
            }
        }

        // InfiniteRoots
        public bool InfiniteRoots()
        {
            if (!_done)
            {
                throw new NotDoneException();
            }
            return infinite_roots;
        }

        // Value
        public double Value(int n)
        {
            if ((!_done) || (n > _nbRoots))
            {
                throw new NotDoneException();
            }
            return _roots[n - 1];
        }
    }

    //=================================================================================================
    // class    : MyTrigonometricFunction
    // purpose  : Internal class: provides Value and Derivative for a
    //            trigonometric polynomial
    //=================================================================================================
    private readonly struct MyTrigonometricFunction
    {
        private readonly double _cc, _ss, _sc, _s, _c, _cte;

        //
        public MyTrigonometricFunction(double xCC,
                                       double xSS,
                                       double xSC,
                                       double xC,
                                       double xS,
                                       double xCte)
        {
            _cc = xCC;
            _ss = xSS;
            _sc = xSC;
            _s = xS;
            _c = xC;
            _cte = xCte;
        }

        public double Value(double u)
        {
            double sinus, cosinus, ret;
            //
            sinus = Math.Sin(u);
            cosinus = Math.Cos(u);
            ret = _cc * cosinus * cosinus + _ss * sinus * sinus
                   + 2.0 * (sinus * (_sc * cosinus + _s) + cosinus * _c) + _cte;
            //
            return ret;
        }

        //
        public double Derivative(double u)
        {
            double sinus, cosinus;
            //
            sinus = Math.Sin(u);
            cosinus = Math.Cos(u);
            //
            return (2.0
                    * ((sinus * cosinus) * (_ss - _cc) + _s * cosinus - _c * sinus
                       + _sc * (cosinus * cosinus - sinus * sinus)));
        }
    }

    //////////

    /// <summary>The empty algorithm.</summary>
    public IntQuadQuad()
    {
        _done = false;
        _identical = false;
        _nbCurves = 0;
        _nbpoints = 0;
        _nbMaxCurves = 12;
        _epsilon = 0.00000001;
        _epsilonCoeffPolyNull = 0.00000001;
        Array.Clear(_nextcurve);
        Array.Clear(_previouscurve);
    }

    //=================================================================================================
    // function : IntAna_IntQuadQuad::IntAna_IntQuadQuad
    // purpose  : Intersection Cylinder - Quadric
    //=================================================================================================

    /// <summary>The intersection between a cylinder and a quadric. Tol is not used.</summary>
    public IntQuadQuad(in Cylinder3D cyl, Quadric quad, double tol)
    {
        _nbMaxCurves = 12;
        _epsilon = 0.00000001;
        _epsilonCoeffPolyNull = 0.00000001;
        Perform(cyl, quad, tol);
    }

    //=================================================================================================
    // function : Perform
    // purpose  : Intersection Cylinder - Quadric
    //=================================================================================================

    /// <summary>Intersects a cylinder and a quadric. Tol is not used.</summary>
    public void Perform(in Cylinder3D cyl, Quadric quad, double tol)
    {
        _ = tol;
        _done = true;
        _identical = false;
        _nbCurves = 0;
        _nbpoints = 0;
        //
        bool UN_SEUL_Z_PAR_THETA, DEUX_Z_PAR_THETA, Z_POSITIF, Z_INDIFFERENT, Z_NEGATIF;
        //
        UN_SEUL_Z_PAR_THETA = false;
        DEUX_Z_PAR_THETA = true;
        Z_POSITIF = true;
        Z_INDIFFERENT = true;
        Z_NEGATIF = false;
        //
        double qxx, qyy, qzz, qxy, qxz, qyz, qx, qy, qz, q1, realEpsilon, rCyl, r2;
        double pIpPI = M_PI + M_PI;
        //
        for (int raz = 0; raz < _nbMaxCurves; raz++)
        {
            _previouscurve[raz] = _nextcurve[raz] = 0;
        }
        //
        rCyl = cyl.Radius;
        realEpsilon = RealEpsilon();
        //----------------------------------------------------------------------
        //-- Coefficients of the quadric:
        //--      2       2       2
        //-- Qxx x + Qyy y + Qzz z + 2 ( Qxy x y + Qxz x z + Qyz y z )
        //-- + 2 ( Qx x + Qy y + Qz z ) + Q1
        //----------------------------------------------------------------------
        quad.Coefficients(out qxx, out qyy, out qzz, out qxy, out qxz, out qyz, out qx, out qy, out qz, out q1);

        //----------------------------------------------------------------------
        //-- Writing the quadric coefficients in the reference frame
        //-- associated with the Cylinder
        //--                 Cyl.Position() -> Ax2
        //----------------------------------------------------------------------
        quad.NewCoefficients(ref qxx, ref qyy, ref qzz, ref qxy, ref qxz, ref qyz, ref qx, ref qy, ref qz, ref q1,
                             cyl.Position);

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

        if (Math.Abs(qzz) < _epsilonCoeffPolyNull)
        {
            _done = false; //-- le 12 mars 98
            return;
        }
        else
        { // #0
            //----------------------------------------------------------------------
            //--- Case where F(Z,Theta) is quadratic in Z                       ----
            //----------------------------------------------------------------------
            r2 = rCyl * rCyl;

            double C_1 = qz * qz - qzz * q1;
            double C_SS = r2 * (qyz * qyz - qyy * qzz);
            double C_CC = r2 * (qxz * qxz - qxx * qzz);
            double C_S = rCyl * (qyz * qz - qy * qzz);
            double C_C = rCyl * (qxz * qz - qx * qzz);
            double C_SC = r2 * (qxz * qyz - qxy * qzz);
            //
            MyTrigonometricFunction mtf = new(C_CC, C_SS, C_SC, C_C, C_S, C_1);
            TrigonometricRoots polDIS = new(C_CC - C_SS, C_SC, C_C + C_C, C_S + C_S, C_1 + C_SS, 0.0, pIpPI);
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            if (!polDIS.IsDone)
            {
                _done = false;
                return;
            }
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            if (polDIS.InfiniteRoots())
            {
                _theCurve[0].SetCylinderQuadValues(cyl, qxx, qyy, qzz, qxy, qxz, qyz, qx, qy, qz, q1,
                                                  _epsilon, 0.0, pIpPI, UN_SEUL_Z_PAR_THETA, Z_POSITIF);
                _theCurve[1].SetCylinderQuadValues(cyl, qxx, qyy, qzz, qxy, qxz, qyz, qx, qy, qz, q1,
                                                  _epsilon, 0.0, pIpPI, UN_SEUL_Z_PAR_THETA, Z_NEGATIF);
                _nbCurves = 2;
            }

            else
            { // #1
                //---------------------------------------------------------------
                //-- The search for zeros of DIS has succeeded
                //---------------------------------------------------------------
                int nbsolDIS = polDIS.SolutionCount;
                if (nbsolDIS == 0)
                {
                    //--------------------------------------------------------------
                    //-- The Discriminant has a constant sign:
                    //--
                    //-- If Positive ---> 2 Curves
                    //-- Otherwise   ---> No solution
                    //--------------------------------------------------------------
                    if (mtf.Value(M_PI) >= -realEpsilon)
                    {
                        _theCurve[0].SetCylinderQuadValues(cyl, qxx, qyy, qzz, qxy, qxz, qyz, qx, qy, qz, q1,
                                                          _epsilon, 0.0, pIpPI, UN_SEUL_Z_PAR_THETA, Z_POSITIF);
                        _theCurve[1].SetCylinderQuadValues(cyl, qxx, qyy, qzz, qxy, qxz, qyz, qx, qy, qz, q1,
                                                          _epsilon, 0.0, pIpPI, UN_SEUL_Z_PAR_THETA, Z_NEGATIF);

                        _nbCurves = 2;
                    }
                    else
                    {
                        //------------------------------------------------------------
                        //-- The Discriminant is always negative
                        //------------------------------------------------------------
                        _nbCurves = 0;
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
                        if (mtf.Value(polDIS.Value(1) + M_PI) >= -realEpsilon)
                        {
                            //------------------------------------------------------------
                            //-- We have a tangent point + a solution curve
                            //------------------------------------------------------------
                            _theCurve[0].SetCylinderQuadValues(cyl, qxx, qyy, qzz, qxy, qxz, qyz, qx, qy, qz, q1,
                                                              _epsilon, 0.0, pIpPI, UN_SEUL_Z_PAR_THETA, Z_POSITIF);
                            _theCurve[1].SetCylinderQuadValues(cyl, qxx, qyy, qzz, qxy, qxz, qyz, qx, qy, qz, q1,
                                                              _epsilon, 0.0, pIpPI, UN_SEUL_Z_PAR_THETA, Z_NEGATIF);

                            _nbCurves = 2;
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
                            _nbCurves = 0;
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
                        double theta1, theta2, theta3, autrepar, qwet;
                        bool unPtTg = false;
                        //
                        _nbCurves = 0;
                        if (nbsolDIS == 2)
                        {
                            for (i = 1; i <= nbsolDIS; i++)
                            {
                                theta1 = polDIS.Value(i);
                                theta2 = (i < nbsolDIS) ? polDIS.Value(i + 1) : (polDIS.Value(1) + pIpPI);
                                //----------------------------------------------------------------
                                //-- We detect double roots
                                //-- If there are only 2 roots then we take the entire interval
                                //----------------------------------------------------------------
                                if (Math.Abs(theta2 - theta1) <= realEpsilon)
                                {
                                    unPtTg = true;
                                    autrepar = theta1 - 0.1;
                                    if (autrepar < 0.0)
                                    {
                                        autrepar = theta1 + 0.1;
                                    }
                                    //
                                    qwet = mtf.Value(autrepar);
                                    if (qwet >= 0.0)
                                    {
                                        double param = theta1 + pIpPI;
                                        AddSpecialPoints(quad, cyl, ref theta1, ref param);
                                        _theCurve[_nbCurves].SetCylinderQuadValues(cyl, qxx, qyy, qzz, qxy, qxz, qyz, qx, qy, qz,
                                                                                 q1, _epsilon, theta1, param,
                                                                                 UN_SEUL_Z_PAR_THETA, Z_POSITIF);
                                        _nbCurves++;
                                        _theCurve[_nbCurves].SetCylinderQuadValues(cyl, qxx, qyy, qzz, qxy, qxz, qyz, qx, qy, qz,
                                                                                 q1, _epsilon, theta1, param,
                                                                                 UN_SEUL_Z_PAR_THETA, Z_NEGATIF);
                                        _nbCurves++;
                                    }
                                }
                            }
                        }

                        for (i = 1; !unPtTg && (i <= nbsolDIS); i++)
                        {
                            theta1 = polDIS.Value(i);
                            theta2 = (i < nbsolDIS) ? polDIS.Value(i + 1) : (polDIS.Value(1) + pIpPI);
                            //----------------------------------------------------------------
                            //-- On detecte les racines doubles
                            //-- Si il n y a que 2 racines alors on prend tout l interval
                            //----------------------------------------------------------------
                            if (Math.Abs(theta2 - theta1) <= 1e-12)
                            {
                                //-- std::cout<<"\n####### Un Point de Tangence en Theta = "<<Theta1<<std::endl;
                                //-- i++;
                            }
                            else
                            { //-- We avoid numerical issues (everything has the same sign between the roots)
                                qwet = mtf.Value(0.5 * (theta1 + theta2)) + mtf.Value(0.4 * theta1 + 0.6 * theta2)
                                       + mtf.Value(0.6 * theta1 + 0.4 * theta2);
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
                                    theta3 = ((i + 1) < nbsolDIS) ? polDIS.Value(i + 2) : (polDIS.Value(1) + pIpPI);
                                    // ft
                                    if ((theta3 - theta2) < 5.0e-8)
                                    {
                                        //
                                        AddSpecialPoints(quad, cyl, ref theta1, ref theta2);
                                        _theCurve[_nbCurves].SetCylinderQuadValues(cyl, qxx, qyy, qzz, qxy, qxz, qyz, qx, qy, qz,
                                                                                 q1, _epsilon, theta1, theta2,
                                                                                 UN_SEUL_Z_PAR_THETA, Z_POSITIF);
                                        _nbCurves++;
                                        _theCurve[_nbCurves].SetCylinderQuadValues(cyl, qxx, qyy, qzz, qxy, qxz, qyz, qx, qy, qz,
                                                                                 q1, _epsilon, theta1, theta2,
                                                                                 UN_SEUL_Z_PAR_THETA, Z_NEGATIF);
                                        _nbCurves++;
                                    }
                                    else
                                    {
                                        AddSpecialPoints(quad, cyl, ref theta1, ref theta2);
                                        _theCurve[_nbCurves].SetCylinderQuadValues(cyl, qxx, qyy, qzz, qxy, qxz, qyz, qx, qy, qz,
                                                                                 q1, _epsilon, theta1, theta2,
                                                                                 DEUX_Z_PAR_THETA, Z_INDIFFERENT);
                                        _nbCurves++;
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
    public IntQuadQuad(in Cone3D cone, Quadric quad, double tol)
    {
        _nbMaxCurves = 12;
        _epsilon = 0.00000001;
        _epsilonCoeffPolyNull = 0.00000001;
        Perform(cone, quad, tol);
    }

    /// <summary>Intersects a cone and a quadric. Tol is not used.</summary>
    public void Perform(in Cone3D cone, Quadric quad, double tol)
    {
        _ = tol;
        //
        bool UN_SEUL_Z_PAR_THETA, Z_POSITIF, Z_NEGATIF;
        //
        UN_SEUL_Z_PAR_THETA = false;
        Z_POSITIF = true;
        Z_NEGATIF = false;
        //
        int i;
        double qxx, qyy, qzz, qxy, qxz, qyz, qx, qy, qz, q1;
        double theta1, theta2, tgAngle;
        double pIpPI = M_PI + M_PI;
        //
        _done = true;
        _identical = false;
        _nbCurves = 0;
        _nbpoints = 0;
        //
        for (i = 0; i < _nbMaxCurves; ++i)
        {
            _previouscurve[i] = 0;
            _nextcurve[i] = 0;
        }
        //
        quad.Coefficients(out qxx, out qyy, out qzz, out qxy, out qxz, out qyz, out qx, out qy, out qz, out q1);
        //
        CoordinateSystem tAx3 = cone.Position;
        tAx3 = tAx3.WithLocation(cone.Apex);
        quad.NewCoefficients(ref qxx, ref qyy, ref qzz, ref qxy, ref qxz, ref qyz, ref qx, ref qy, ref qz, ref q1, tAx3);
        //
        tgAngle = 1.0 / (Math.Tan(cone.SemiAngle));
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
        double z2CC, z2SS, z2Cte, z2SC, z2C, z2S;
        double z1CC, z1SS, z1Cte, z1SC, z1C, z1S;
        double C_1, C_SS, C_CC, C_S, C_C, C_SC;
        //
        z2CC = qxx;
        z2SS = qyy;
        z2Cte = qzz * tgAngle * tgAngle;
        z2SC = qxy;
        z2C = (tgAngle) * qxz;
        z2S = (tgAngle) * qyz;
        //
        TrigonometricRoots polZ2 = new(z2CC - z2SS, z2SC, z2C + z2C, z2S + z2S, z2Cte + z2SS, 0.0, pIpPI);
        if (!polZ2.IsDone)
        {
            _done = !_done;
            return;
        }
        //
        // MyTrigonometricFunction MTF2(Z2CC,Z2SS,Z2SC,Z2C,Z2S,Z2Cte);
        nbsolZ2 = polZ2.SolutionCount;
        //
        // 2. Try to solve B(t)=0  -> PolZ1
        //
        z1Cte = 2.0 * (tgAngle) * qz;
        z1SS = 0.0;
        z1CC = 0.0;
        z1S = qy;
        z1C = qx;
        z1SC = 0.0;
        //
        TrigonometricRoots polZ1 = new(z1CC - z1SS, z1SC, z1C + z1C, z1S + z1S, z1Cte + z1SS, 0.0, pIpPI);
        if (!polZ1.IsDone)
        {
            _done = !_done;
            return;
        }
        MyTrigonometricFunction mtfz1 = new(z1CC, z1SS, z1SC, z1C, z1S, z1Cte);
        //
        nbsol1 = polZ1.SolutionCount;
        if (polZ2.InfiniteRoots())
        { // i.e A(t)=0 for any t
            if (!polZ1.InfiniteRoots())
            {
                if (nbsol1 == 0)
                {
                    //-- B(t)*z + C(t) = 0    with C(t) != 0
                    _theCurve[0].SetConeQuadValues(cone, qxx, qyy, qzz, qxy, qxz, qyz, qx, qy, qz, q1,
                                                  _epsilon, 0.0, pIpPI, UN_SEUL_Z_PAR_THETA, Z_POSITIF);
                    _nbCurves = 1;
                }
            }
            else
            {
                if (Math.Abs(q1) <= _epsilon)
                {
                    _done = !_done;
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
        C_1 = tgAngle * tgAngle * (qz * qz - qzz * q1);
        C_SS = qy * qy - qyy * q1;
        C_CC = qx * qx - qxx * q1;
        C_S = tgAngle * (qy * qz - qyz * q1);
        C_C = tgAngle * (qx * qz - qxz * q1);
        C_SC = qx * qy - qxy * q1;
        //
        TrigonometricRoots pol = new(C_CC - C_SS, C_SC, C_C + C_C, C_S + C_S, C_1 + C_SS, 0.0, pIpPI);
        if (!pol.IsDone)
        {
            _done = !_done;
            return;
        }
        //
        nbsol = pol.SolutionCount;
        MyTrigonometricFunction mtf = new(C_CC, C_SS, C_SC, C_C, C_S, C_1);
        //
        if (pol.InfiniteRoots())
        {
            //                             2
            //         f(z,t)= (z(t)-zo(t)) = 0       Discriminant(t)=0 pour tout t
            //
            _theCurve[0].SetConeQuadValues(cone, qxx, qyy, qzz, qxy, qxz, qyz, qx, qy, qz, q1,
                                          _epsilon, 0.0, pIpPI, UN_SEUL_Z_PAR_THETA, Z_POSITIF);
            _theCurve[1].SetConeQuadValues(cone, qxx, qyy, qzz, qxy, qxz, qyz, qx, qy, qz, q1,
                                          _epsilon, 0.0, pIpPI, UN_SEUL_Z_PAR_THETA, Z_NEGATIF);
            _nbCurves = 2;
            return;
        }
        // else {//#4
        //                      2
        //         f(z,t)=A(t)*z + B(t)*z + C(t)      Discriminant(t) != 0
        //
        if (nbsol == 0 && (mtf.Value(M_PI) < 0.0))
        {
            //-- Discriminant has constant negative sign
            return;
        }
        // else {//# 3
        //
        //-- We handle the case: Discriminant(t) > 0 for all t by simulating
        //   a double root at 0
        bool discriminantConstantPositif = false;
        if (nbsol == 0)
        {
            nbsol = 1;
            discriminantConstantPositif = true;
        }
        //----------------------------------------------------------------------
        //-- The discriminant has at least one root ( -> Tangent Point )
        //-- or is constant positive.
        //----------------------------------------------------------------------
        for (i = 1; i <= nbsol; ++i)
        {
            if (discriminantConstantPositif)
            {
                theta1 = 0.0;
                theta2 = pIpPI - _epsilon;
            }
            else
            {
                theta1 = pol.Value(i);
                theta2 = (i < nbsol) ? pol.Value(i + 1) : (pol.Value(1) + pIpPI);
            }
            //
            if (Math.Abs(theta2 - theta1) <= _epsilon)
            {
                _done = false;
                return; // !!! pkv
            }
            // else {// #2
            double qwet = mtf.Value(0.5 * (theta1 + theta2)) + mtf.Value(0.4 * theta1 + 0.6 * theta2)
                          + mtf.Value(0.6 * theta1 + 0.4 * theta2);
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
            bool racinesdePolZ2DansTheta1Theta2;
            int i2;
            double r;
            //
            // nbsolZ2=PolZ2.NbSolutions();
            racinesdePolZ2DansTheta1Theta2 = false;
            for (i2 = 1; i2 <= nbsolZ2 && !racinesdePolZ2DansTheta1Theta2; ++i2)
            {
                r = polZ2.Value(i2);
                if (r > theta1 && r < theta2)
                {
                    racinesdePolZ2DansTheta1Theta2 = true;
                }
                else
                {
                    r += pIpPI;
                    if (r > theta1 && r < theta2)
                    {
                        racinesdePolZ2DansTheta1Theta2 = true;
                    }
                }
            }
            //
            if (!racinesdePolZ2DansTheta1Theta2)
            {
                //--------------------------------------------------------------------
                //-- No infinite branches
                _theCurve[_nbCurves].SetConeQuadValues(cone, qxx, qyy, qzz, qxy, qxz, qyz, qx, qy, qz, q1,
                                                     _epsilon, theta1, theta2, UN_SEUL_Z_PAR_THETA, Z_POSITIF);
                _nbCurves++;
                _theCurve[_nbCurves].SetConeQuadValues(cone, qxx, qyy, qzz, qxy, qxz, qyz, qx, qy, qz, q1,
                                                     _epsilon, theta1, theta2, UN_SEUL_Z_PAR_THETA, Z_NEGATIF);
                _nbCurves++;
            }

            else
            { // #1
                bool noChanges;
                double newMin, newMax, to;
                //
                newMin = theta1;
                newMax = theta2;
                noChanges = true;
                //
                for (i2 = 1; i2 <= (nbsolZ2 + nbsolZ2); ++i2)
                {
                    if (i2 > nbsolZ2)
                    {
                        to = polZ2.Value(i2 - nbsolZ2) + pIpPI;
                    }
                    else
                    {
                        to = polZ2.Value(i2);
                    }
                    //
                    // to is value of the parameter t where A(t)=0, i.e.
                    // f(z,to)=B(to)*z + C(to)=0, B(to)!=0.
                    //
                    // z=-C(to)/B(to) is one and only
                    // solution in spite of the fact that D(t)>0 for any value of t.
                    //
                    if (to < newMax && to > newMin)
                    {
                        //-----------------------------------------------------------------
                        //-- We split at least once the domain Theta1 Theta2
                        //-----------------------------------------------------------------
                        noChanges = false;
                        _theCurve[_nbCurves].SetConeQuadValues(cone, qxx, qyy, qzz, qxy, qxz, qyz, qx, qy, qz, q1,
                                                             _epsilon, newMin, to, UN_SEUL_Z_PAR_THETA, Z_NEGATIF);
                        //
                        _nbCurves++;
                        _theCurve[_nbCurves].SetConeQuadValues(cone, qxx, qyy, qzz, qxy, qxz, qyz, qx, qy, qz, q1,
                                                             _epsilon, newMin, to, UN_SEUL_Z_PAR_THETA, Z_POSITIF);
                        //------------------------------------------------------------
                        //-- A == 0
                        //-- Si B < 0    Alors Infini sur   Z+
                        //-- Sinon             Infini sur   Z-
                        //------------------------------------------------------------
                        if (polZ2.IsARoot(newMin))
                        {
                            if (mtfz1.Value(newMin) < 0.0)
                            {
                                _theCurve[_nbCurves].SetIsFirstOpen(true);
                            }
                            else
                            {
                                _theCurve[_nbCurves - 1].SetIsFirstOpen(true);
                            }
                        }
                        if (mtfz1.Value(to) < 0.0)
                        {
                            _theCurve[_nbCurves].SetIsLastOpen(true);
                        }
                        else
                        {
                            _theCurve[_nbCurves - 1].SetIsLastOpen(true);
                        }
                        //------------------------------------------------------------
                        _nbCurves++;
                        newMin = to;
                    } // if(to<NewMax && to>NewMin)
                } // for(i2=1; i2<= (nbsolZ2+nbsolZ2) ; ++i2)
                //
                if (noChanges)
                {
                    _theCurve[_nbCurves].SetConeQuadValues(cone, qxx, qyy, qzz, qxy, qxz, qyz, qx, qy, qz, q1,
                                                         _epsilon, theta1, theta2, UN_SEUL_Z_PAR_THETA, Z_NEGATIF);
                    _nbCurves++;
                    _theCurve[_nbCurves].SetConeQuadValues(cone, qxx, qyy, qzz, qxy, qxz, qyz, qx, qy, qz, q1,
                                                         _epsilon, theta1, theta2, UN_SEUL_Z_PAR_THETA, Z_POSITIF);
                    //------------------------------------------------------------
                    //-- A == 0
                    //-- If B < 0    Then infinity on   Z+
                    //-- Otherwise         infinity on   Z-
                    //------------------------------------------------------------
                    if (polZ2.IsARoot(theta1))
                    {
                        if (mtfz1.Value(theta1) < 0.0)
                        {
                            _theCurve[_nbCurves].SetIsFirstOpen(true);
                        }
                        else
                        {
                            _theCurve[_nbCurves - 1].SetIsFirstOpen(true);
                        }
                    }
                    if (polZ2.IsARoot(theta2))
                    {
                        if (mtfz1.Value(theta2) < 0.0)
                        {
                            _theCurve[_nbCurves].SetIsLastOpen(true);
                        }
                        else
                        {
                            _theCurve[_nbCurves - 1].SetIsLastOpen(true);
                        }
                    }
                    //------------------------------------------------------------
                    _nbCurves++;
                } // if(NoChanges) {
                else
                { // #0
                    _theCurve[_nbCurves].SetConeQuadValues(cone, qxx, qyy, qzz, qxy, qxz, qyz, qx, qy, qz, q1,
                                                         _epsilon, newMin, theta2, UN_SEUL_Z_PAR_THETA, Z_NEGATIF);
                    _nbCurves++;
                    _theCurve[_nbCurves].SetConeQuadValues(cone, qxx, qyy, qzz, qxy, qxz, qyz, qx, qy, qz, q1,
                                                         _epsilon, newMin, theta2, UN_SEUL_Z_PAR_THETA, Z_POSITIF);
                    //------------------------------------------------------------
                    //-- A == 0
                    //-- If B < 0    Then infinity on   Z+
                    //-- Otherwise         infinity on   Z-
                    //------------------------------------------------------------
                    if (polZ2.IsARoot(newMin))
                    {
                        if (mtfz1.Value(newMin) < 0.0)
                        {
                            _theCurve[_nbCurves].SetIsFirstOpen(true);
                        }
                        else
                        {
                            _theCurve[_nbCurves - 1].SetIsFirstOpen(true);
                        }
                    }
                    if (polZ2.IsARoot(theta2))
                    {
                        if (mtfz1.Value(theta2) < 0.0)
                        {
                            _theCurve[_nbCurves].SetIsLastOpen(true);
                        }
                        else
                        {
                            _theCurve[_nbCurves - 1].SetIsLastOpen(true);
                        }
                    }
                    //------------------------------------------------------------

                    _nbCurves++;
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
        bool notLastOpenC2, notFirstOpenC2;
        int c1, c2;
        double eps, epsilon_DISTANCE;
        double dInfC1, dSupC1, dInfC2, dSupC2;
        //
        eps = 0.0000001;
        epsilon_DISTANCE = 0.0000000001;
        //
        for (c1 = 0; c1 < _nbCurves; c1++)
        {
            _nextcurve[c1] = 0;
            _previouscurve[c1] = 0;
        }
        //
        for (c1 = 0; c1 < _nbCurves; c1++)
        {
            _theCurve[c1].Domain(out dInfC1, out dSupC1);

            // OCCT: the loop stops at c2 == c1, so only the curves before c1 are compared with it
            for (c2 = 0; (c2 < _nbCurves) && (c2 != c1); c2++)
            {
                notLastOpenC2 = !_theCurve[c2].IsLastOpen;
                notFirstOpenC2 = !_theCurve[c2].IsFirstOpen;
                _theCurve[c2].Domain(out dInfC2, out dSupC2);
                if (!_theCurve[c1].IsFirstOpen)
                {
                    if (notLastOpenC2)
                    {
                        if (Math.Abs(dInfC1 - dSupC2) <= eps
                            && (_theCurve[c1].Value(dInfC1).DistanceTo(_theCurve[c2].Value(dSupC2))
                                < epsilon_DISTANCE))
                        {
                            _previouscurve[c1] = c2 + 1;
                            _nextcurve[c2] = c1 + 1;
                        }
                    }
                    if (notFirstOpenC2)
                    {
                        if (Math.Abs(dInfC1 - dInfC2) <= eps
                            && (_theCurve[c1].Value(dInfC1).DistanceTo(_theCurve[c2].Value(dInfC2))
                                < epsilon_DISTANCE))
                        {
                            _previouscurve[c1] = -(c2 + 1);
                            _previouscurve[c2] = -(c1 + 1);
                        }
                    }
                }
                if (!_theCurve[c1].IsLastOpen)
                {
                    if (notLastOpenC2)
                    {
                        if (Math.Abs(dSupC1 - dSupC2) <= eps
                            && (_theCurve[c1].Value(dSupC1).DistanceTo(_theCurve[c2].Value(dSupC2))
                                < epsilon_DISTANCE))
                        {
                            _nextcurve[c1] = -(c2 + 1);
                            _nextcurve[c2] = -(c1 + 1);
                        }
                    }
                    if (notFirstOpenC2)
                    {
                        if (Math.Abs(dSupC1 - dInfC2) <= eps
                            && (_theCurve[c1].Value(dSupC1).DistanceTo(_theCurve[c2].Value(dInfC2))
                                < epsilon_DISTANCE))
                        {
                            _nextcurve[c1] = c2 + 1;
                            _previouscurve[c2] = c1 + 1;
                        }
                    }
                }
            }
        }
    }

    /// <summary>True if the computation was successful.</summary>
    public bool IsDone => (_done);

    /// <summary>True if the cylinder, the cone or the sphere is identical to the quadric (never set by OCCT).</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public bool IdenticalElements()
    {
        if (!_done)
        {
            throw new NotDoneException("IntQuadQuad Not done");
        }
        return (_identical);
    }

    /// <summary>The number of curves.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    /// <exception cref="DomainErrorException">The elements are identical.</exception>
    public int CurveCount
    {
        get
        {
            if (!_done)
            {
                throw new NotDoneException("IntQuadQuad Not done");
            }
            if (_identical)
            {
                throw new DomainErrorException();
            }
            return (_nbCurves);
        }
    }

    /// <summary>The number of contact points (always 0 in OCCT).</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    /// <exception cref="DomainErrorException">The elements are identical.</exception>
    public int PntCount
    {
        get
        {
            if (!_done)
            {
                throw new NotDoneException("IntQuadQuad Not done");
            }
            if (_identical)
            {
                throw new DomainErrorException();
            }
            return (_nbpoints);
        }
    }

    /// <summary>True if the curve I shares its first bound with another curve.</summary>
    /// <exception cref="OutOfRangeException">I is not in [1, NbCurve()].</exception>
    public bool HasPreviousCurve(int i)
    {
        if (!_done)
        {
            throw new NotDoneException("IntQuadQuad Not done");
        }
        if (_identical)
        {
            throw new DomainErrorException("IntQuadQuad identical");
        }
        if ((i > _nbCurves) || (i <= 0))
        {
            throw new OutOfRangeException("Incorrect Curve Number 'HasPrevious Curve'");
        }
        if (_previouscurve[i - 1] != 0)
        {
            return true;
        }
        return false;
    }

    /// <summary>True if the curve I shares its last bound with another curve.</summary>
    /// <exception cref="OutOfRangeException">I is not in [1, NbCurve()].</exception>
    public bool HasNextCurve(int i)
    {
        if (!_done)
        {
            throw new NotDoneException("IntQuadQuad Not done");
        }
        if (_identical)
        {
            throw new DomainErrorException("IntQuadQuad identical");
        }
        if ((i > _nbCurves) || (i <= 0))
        {
            throw new OutOfRangeException("Incorrect Curve Number 'HasNextCurve'");
        }
        if (_nextcurve[i - 1] != 0)
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
    public int PreviousCurve(int i, out bool opposite)
    {
        if (HasPreviousCurve(i))
        {
            if (_previouscurve[i - 1] > 0)
            {
                opposite = false;
                return (_previouscurve[i - 1]);
            }
            else
            {
                opposite = true;
                return (-_previouscurve[i - 1]);
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
    public int NextCurve(int i, out bool opposite)
    {
        if (HasNextCurve(i))
        {
            opposite = _nextcurve[i - 1] < 0;
            return Math.Abs(_nextcurve[i - 1]);
        }
        else
        {
            throw new DomainErrorException("Incorrect Curve Number 'NextCurve'");
        }
    }

    /// <summary>The curve of range i.</summary>
    /// <exception cref="OutOfRangeException">i is not in [1, NbCurve()].</exception>
    public Curve Curve(int i)
    {
        if (!_done)
        {
            throw new NotDoneException("IntQuadQuad Not done");
        }
        if (_identical)
        {
            throw new DomainErrorException("IntQuadQuad identical");
        }
        if ((i <= 0) || (i > _nbCurves))
        {
            throw new OutOfRangeException("Incorrect Curve Number");
        }
        return _theCurve[i - 1];
    }

    /// <summary>The point of range i.</summary>
    /// <exception cref="OutOfRangeException">i is not in [1, NbPnt()].</exception>
    public Point3D Point(int i)
    {
        if (!_done)
        {
            throw new NotDoneException("IntQuadQuad Not done");
        }
        if (_identical)
        {
            throw new DomainErrorException("IntQuadQuad identical");
        }
        if ((i <= 0) || (i > _nbpoints))
        {
            throw new OutOfRangeException("Incorrect Point Number");
        }
        return _thepoints[i - 1];
    }

    /// <summary>
    /// The parameters on the cylinder or cone of the point of range N. Not implemented in OCCT: leaves U1 and U2
    /// unchanged.
    /// </summary>
    public void Parameters(int n, ref double u1, ref double u2)
    {
        // OCCT prints "IntAna_IntQuadQuad::Parameters(...) is not yet implemented" to std::cout (left out)
        _ = n;
        _ = u1;
        _ = u2;
    }
}
