// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_OptimizationHelpers.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Kerf.Engine.Numerics.Utilities.MathUtils;

namespace Occt;

/// <summary>
/// PSO+Brent and PSO+Powell optimization wrappers for precise bounding box computation (OCCT's namespace
/// GeomBndLib_OptimizationHelpers).
/// </summary>
/// <remarks>
/// The function objects are private readonly structs implementing the MathUtils functor interfaces, which the JIT
/// specialises (decision 51). Where OCCT's <c>Value</c> returns false without writing its result, the port writes 0
/// (decision 32); the solvers discard it. OCCT's template <c>AdjustExtrT</c>, which only chooses between the 3D and
/// 2D functions, is the pair of overloads of the same name.
/// </remarks>
internal static class GeomBndLib_OptimizationHelpers
{
    // ============================================================
    // Internal function objects for optimization
    // ============================================================

    /// <summary>1D function object for coordinate minimization/maximization along a 3D curve.</summary>
    private readonly struct CurveCoordFunc(Adaptor3d_Curve theCurve, double theUMin, double theUMax, int theCoordIdx, double theSign)
        : IScalarFunction
    {
        private readonly Adaptor3d_Curve myCurve = theCurve;
        private readonly double myUMin = theUMin;
        private readonly double myUMax = theUMax;
        private readonly int myCoordIdx = theCoordIdx;
        private readonly double mySign = theSign;

        public bool Value(double theX, out double theF)
        {
            if (theX < myUMin || theX > myUMax)
            {
                theF = 0.0;
                return false;
            }
            Point3D aP = myCurve.Value(theX);
            theF = mySign * aP.Coord(myCoordIdx);
            return true;
        }
    }

    /// <summary>Multi-variable function for PSO on a 3D curve.</summary>
    private readonly struct CurveCoordMVarFunc(Adaptor3d_Curve theCurve, double theUMin, double theUMax, int theCoordIdx, double theSign)
        : IVectorFunction
    {
        private readonly Adaptor3d_Curve myCurve = theCurve;
        private readonly double myUMin = theUMin;
        private readonly double myUMax = theUMax;
        private readonly int myCoordIdx = theCoordIdx;
        private readonly double mySign = theSign;

        public bool Value(math_Vector theX, out double theF)
        {
            if (theX[1] < myUMin || theX[1] > myUMax)
            {
                theF = 0.0;
                return false;
            }
            Point3D aP = myCurve.Value(theX[1]);
            theF = mySign * aP.Coord(myCoordIdx);
            return true;
        }

        public int NbVariables() => 1;
    }

    /// <summary>1D function object for coordinate minimization/maximization along a 2D curve.</summary>
    private readonly struct Curve2dCoordFunc(Adaptor2d_Curve2d theCurve, double theUMin, double theUMax, int theCoordIdx, double theSign)
        : IScalarFunction
    {
        private readonly Adaptor2d_Curve2d myCurve = theCurve;
        private readonly double myUMin = theUMin;
        private readonly double myUMax = theUMax;
        private readonly int myCoordIdx = theCoordIdx;
        private readonly double mySign = theSign;

        public bool Value(double theX, out double theF)
        {
            if (theX < myUMin || theX > myUMax)
            {
                theF = 0.0;
                return false;
            }
            Point2D aP = myCurve.Value(theX);
            theF = mySign * aP.Coord(myCoordIdx);
            return true;
        }
    }

    /// <summary>Multi-variable function for PSO on a 2D curve.</summary>
    private readonly struct Curve2dCoordMVarFunc(Adaptor2d_Curve2d theCurve, double theUMin, double theUMax, int theCoordIdx, double theSign)
        : IVectorFunction
    {
        private readonly Adaptor2d_Curve2d myCurve = theCurve;
        private readonly double myUMin = theUMin;
        private readonly double myUMax = theUMax;
        private readonly int myCoordIdx = theCoordIdx;
        private readonly double mySign = theSign;

        public bool Value(math_Vector theX, out double theF)
        {
            if (theX[1] < myUMin || theX[1] > myUMax)
            {
                theF = 0.0;
                return false;
            }
            Point2D aP = myCurve.Value(theX[1]);
            theF = mySign * aP.Coord(myCoordIdx);
            return true;
        }

        public int NbVariables() => 1;
    }

    /// <summary>Multi-variable function for PSO+Powell on a surface.</summary>
    private readonly struct SurfCoordFunc : IVectorFunction
    {
        private readonly Adaptor3d_Surface mySurf;
        private readonly double myUMin;
        private readonly double myUMax;
        private readonly double myVMin;
        private readonly double myVMax;
        private readonly int myCoordIdx;
        private readonly double mySign;
        private readonly double myPenalty;

        public SurfCoordFunc(Adaptor3d_Surface theSurf,
                             double theUMin,
                             double theUMax,
                             double theVMin,
                             double theVMax,
                             int theCoordIdx,
                             double theSign)
        {
            mySurf = theSurf;
            myUMin = theUMin;
            myUMax = theUMax;
            myVMin = theVMin;
            myVMax = theVMax;
            myCoordIdx = theCoordIdx;
            mySign = theSign;
            myPenalty = 0.0;

            math_Vector aX = new(1, 2);
            aX[1] = theUMin;
            aX[2] = (theVMin + theVMax) / 2.0;
            Value(aX, out double aF1);
            aX[1] = theUMax;
            Value(aX, out double aF2);
            double aDU = Math.Abs((aF2 - aF1) / (theUMax - theUMin));
            aX[1] = (theUMin + theUMax) / 2.0;
            aX[2] = theVMin;
            Value(aX, out aF1);
            aX[2] = theVMax;
            Value(aX, out aF2);
            double aDV = Math.Abs((aF2 - aF1) / (theVMax - theVMin));
            double aPenalty = 10.0 * Max(aDU, aDV);
            myPenalty = Max(aPenalty, 1.0);
        }

        public bool Value(math_Vector theX, out double theF)
        {
            if (theX[1] >= myUMin && theX[1] <= myUMax && theX[2] >= myVMin && theX[2] <= myVMax)
            {
                Point3D aP = mySurf.Value(theX[1], theX[2]);
                theF = mySign * aP.Coord(myCoordIdx);
            }
            else
            {
                double aUPen = 0.0, aVPen = 0.0, u0, v0;
                if (theX[1] < myUMin)
                {
                    aUPen = myPenalty * (myUMin - theX[1]);
                    u0 = myUMin;
                }
                else if (theX[1] > myUMax)
                {
                    aUPen = myPenalty * (theX[1] - myUMax);
                    u0 = myUMax;
                }
                else
                {
                    u0 = theX[1];
                }
                if (theX[2] < myVMin)
                {
                    aVPen = myPenalty * (myVMin - theX[2]);
                    v0 = myVMin;
                }
                else if (theX[2] > myVMax)
                {
                    aVPen = myPenalty * (theX[2] - myVMax);
                    v0 = myVMax;
                }
                else
                {
                    v0 = theX[2];
                }
                Point3D aP = mySurf.Value(u0, v0);
                theF = mySign * aP.Coord(myCoordIdx) + aUPen + aVPen;
            }
            return true;
        }

        public int NbVariables() => 2;
    }

    // ============================================================
    // Public helper functions
    // ============================================================

    /// <summary>
    /// Adjust coordinate extremum using PSO + Brent for 1D parameter curves: returns the adjusted extremum of
    /// coordinate theCoordIdx (1=X, 2=Y, 3=Z) on [theUMin, theUMax], starting from theExtr0; a minimum if theIsMin.
    /// </summary>
    internal static double AdjustExtrCurve(Adaptor3d_Curve theCurve,
                                           double theUMin,
                                           double theUMax,
                                           double theExtr0,
                                           int theCoordIdx,
                                           double theTol,
                                           bool theIsMin)
    {
        double aSign = theIsMin ? 1.0 : -1.0;
        double extr = aSign * theExtr0;

        double uTol = Max(theCurve.Resolution(theTol), global::Kerf.Engine.Foundation.Precision.PConfusion());
        double Du = (theCurve.LastParameter() - theCurve.FirstParameter());

        double reltol = uTol / Max(Math.Abs(theUMin), Math.Abs(theUMax));
        Config aBrentCfg = new();
        aBrentCfg.XTolerance = reltol;
        aBrentCfg.FTolerance = uTol;
        aBrentCfg.MaxIterations = 100;

        if (theUMax - theUMin < 0.01 * Du)
        {
            CurveCoordFunc aFunc0 = new(theCurve, theUMin, theUMax, theCoordIdx, aSign);
            ScalarResult anOptLoc0 = global::Kerf.Engine.Numerics.Optimisation.MathOpt.Brent(aFunc0, theUMin, theUMax, aBrentCfg);
            if (anOptLoc0.Status == Status.OK || anOptLoc0.Status == Status.MaxIterations)
            {
                if (anOptLoc0.Value.HasValue)
                {
                    extr = anOptLoc0.Value.Value;
                }
                return aSign * extr;
            }
        }

        int aNbParticles = Max(8, RealToInt(32 * (theUMax - theUMin) / Du));
        math_Vector aLowBorder = new(1, 1);
        math_Vector aUppBorder = new(1, 1);
        aLowBorder[1] = theUMin;
        aUppBorder[1] = theUMax;

        CurveCoordMVarFunc aFunc = new(theCurve, theUMin, theUMax, theCoordIdx, aSign);
        global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOConfig aPsoCfg = new(aNbParticles);
        VectorResult aPsoRes = global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSO(aFunc, aLowBorder, aUppBorder, aPsoCfg);

        math_Vector aT = new(1, 1);
        if (aPsoRes.Status == Status.OK || aPsoRes.Status == Status.MaxIterations)
        {
            if (aPsoRes.Solution is not null)
            {
                aT[1] = aPsoRes.Solution[1];
            }
            else
            {
                aT[1] = (theUMin + theUMax) / 2.0;
            }
            if (aPsoRes.Value.HasValue)
            {
                extr = aPsoRes.Value.Value;
            }
        }
        else
        {
            aT[1] = (theUMin + theUMax) / 2.0;
        }

        CurveCoordFunc aFunc1 = new(theCurve, theUMin, theUMax, theCoordIdx, aSign);
        double aStep = Min(0.1 * Du, (theUMax - theUMin) / (aNbParticles + 1));
        ScalarResult anOptLoc = global::Kerf.Engine.Numerics.Optimisation.MathOpt.Brent(aFunc1,
                                              Max(aT[1] - aStep, theUMin),
                                              Min(aT[1] + aStep, theUMax),
                                              aBrentCfg);

        if (anOptLoc.Status == Status.OK || anOptLoc.Status == Status.MaxIterations)
        {
            if (anOptLoc.Value.HasValue)
            {
                extr = anOptLoc.Value.Value;
                // Only blend it with PSO result if both were valid
                if (aPsoRes.Value.HasValue && aPsoRes.Solution is not null
                    && aT[1] == aPsoRes.Solution[1])
                {
                    extr = Min(extr, aPsoRes.Value.Value);
                }
                return aSign * extr;
            }
        }

        return aSign * extr;
    }

    /// <summary>Adjust coordinate extremum using PSO + Brent for 1D parameter 2D curves (1=X, 2=Y).</summary>
    internal static double AdjustExtrCurve2d(Adaptor2d_Curve2d theCurve,
                                             double theUMin,
                                             double theUMax,
                                             double theExtr0,
                                             int theCoordIdx,
                                             double theTol,
                                             bool theIsMin)
    {
        double aSign = theIsMin ? 1.0 : -1.0;
        double extr = aSign * theExtr0;

        double uTol = Max(theCurve.Resolution(theTol), global::Kerf.Engine.Foundation.Precision.PConfusion());
        double Du = (theCurve.LastParameter() - theCurve.FirstParameter());

        double reltol = uTol / Max(Math.Abs(theUMin), Math.Abs(theUMax));

        Config aBrentCfg = new();
        aBrentCfg.XTolerance = reltol;
        aBrentCfg.FTolerance = uTol;
        aBrentCfg.MaxIterations = 100;

        if (theUMax - theUMin < 0.01 * Du)
        {
            Curve2dCoordFunc aFunc0 = new(theCurve, theUMin, theUMax, theCoordIdx, aSign);
            ScalarResult anOptLoc0 = global::Kerf.Engine.Numerics.Optimisation.MathOpt.Brent(aFunc0, theUMin, theUMax, aBrentCfg);
            if (anOptLoc0.Status == Status.OK || anOptLoc0.Status == Status.MaxIterations)
            {
                if (anOptLoc0.Value.HasValue)
                {
                    extr = anOptLoc0.Value.Value;
                }
                return aSign * extr;
            }
        }

        int aNbParticles = Max(8, RealToInt(32 * (theUMax - theUMin) / Du));
        math_Vector aLowBorder = new(1, 1);
        math_Vector aUppBorder = new(1, 1);
        aLowBorder[1] = theUMin;
        aUppBorder[1] = theUMax;

        Curve2dCoordMVarFunc aFunc = new(theCurve, theUMin, theUMax, theCoordIdx, aSign);
        global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOConfig aPsoCfg = new(aNbParticles);
        VectorResult aPsoRes = global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSO(aFunc, aLowBorder, aUppBorder, aPsoCfg);

        math_Vector aT = new(1, 1);
        if (aPsoRes.Status == Status.OK || aPsoRes.Status == Status.MaxIterations)
        {
            if (aPsoRes.Solution is not null)
            {
                aT[1] = aPsoRes.Solution[1];
            }
            else
            {
                aT[1] = (theUMin + theUMax) / 2.0;
            }
            if (aPsoRes.Value.HasValue)
            {
                extr = aPsoRes.Value.Value;
            }
        }
        else
        {
            aT[1] = (theUMin + theUMax) / 2.0;
        }

        double aStep = Min(0.1 * Du, (theUMax - theUMin) / (aNbParticles + 1));
        Curve2dCoordFunc aFunc1 = new(theCurve, theUMin, theUMax, theCoordIdx, aSign);
        ScalarResult anOptLoc = global::Kerf.Engine.Numerics.Optimisation.MathOpt.Brent(aFunc1,
                                              Max(aT[1] - aStep, theUMin),
                                              Min(aT[1] + aStep, theUMax),
                                              aBrentCfg);

        if (anOptLoc.Status == Status.OK || anOptLoc.Status == Status.MaxIterations)
        {
            if (anOptLoc.Value.HasValue)
            {
                extr = anOptLoc.Value.Value;
                if (aPsoRes.Value.HasValue && aPsoRes.Solution is not null
                    && aT[1] == aPsoRes.Solution[1])
                {
                    extr = Min(extr, aPsoRes.Value.Value);
                }
                return aSign * extr;
            }
        }

        return aSign * extr;
    }

    /// <summary>
    /// Adjust coordinate extremum using PSO + Powell for 2D parameter surfaces, on [theUMin, theUMax] x
    /// [theVMin, theVMax].
    /// </summary>
    internal static double AdjustExtrSurf(Adaptor3d_Surface theSurf,
                                          double theUMin,
                                          double theUMax,
                                          double theVMin,
                                          double theVMax,
                                          double theExtr0,
                                          int theCoordIdx,
                                          double theTol,
                                          bool theIsMin)
    {
        double aSign = theIsMin ? 1.0 : -1.0;
        double extr = aSign * theExtr0;
        double relTol = 2.0 * theTol;
        if (Math.Abs(extr) > theTol)
        {
            relTol /= Math.Abs(extr);
        }
        double Du = (theSurf.LastUParameter() - theSurf.FirstUParameter());
        double Dv = (theSurf.LastVParameter() - theSurf.FirstVParameter());

        math_Vector aLowBorder = new(1, 2);
        math_Vector aUppBorder = new(1, 2);
        aLowBorder[1] = theUMin;
        aUppBorder[1] = theUMax;
        aLowBorder[2] = theVMin;
        aUppBorder[2] = theVMax;

        int aNbU = Max(8, RealToInt(32 * (theUMax - theUMin) / Du));
        int aNbV = Max(8, RealToInt(32 * (theVMax - theVMin) / Dv));
        int aNbParticles = aNbU * aNbV;

        SurfCoordFunc aFunc = new(theSurf, theUMin, theUMax, theVMin, theVMax, theCoordIdx, aSign);
        global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOConfig aPsoCfg = new(aNbParticles);
        VectorResult aPsoRes = global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSO(aFunc, aLowBorder, aUppBorder, aPsoCfg);

        math_Vector aT = new(1, 2);
        if (aPsoRes.Status == Status.OK || aPsoRes.Status == Status.MaxIterations)
        {
            if (aPsoRes.Solution is not null)
            {
                aT = new math_Vector(aPsoRes.Solution);
            }
            else
            {
                aT[1] = (theUMin + theUMax) / 2.0;
                aT[2] = (theVMin + theVMax) / 2.0;
            }
            if (aPsoRes.Value.HasValue)
            {
                extr = aPsoRes.Value.Value;
            }
        }
        else
        {
            aT[1] = (theUMin + theUMax) / 2.0;
            aT[2] = (theVMin + theVMax) / 2.0;
        }

        Matrix aDir = new(1, 2, 1, 2, 0.0);
        aDir[1, 1] = 1.0;
        aDir[2, 2] = 1.0;

        Config aPowellCfg = new();
        aPowellCfg.XTolerance = relTol;
        aPowellCfg.FTolerance = theTol;
        aPowellCfg.MaxIterations = 200;

        VectorResult powell = global::Kerf.Engine.Numerics.Optimisation.MathOpt.PowellWithDirections(aFunc, aT, aDir, aPowellCfg);

        if (powell.Status == Status.OK || powell.Status == Status.MaxIterations)
        {
            if (powell.Value.HasValue)
            {
                extr = Min(extr, powell.Value.Value);
            }
        }

        return aSign * extr;
    }

    /// <summary>OCCT's <c>AdjustExtrT</c> over a 3D curve.</summary>
    internal static double AdjustExtrT(Adaptor3d_Curve theCurve, double theUMin, double theUMax, double theExtr0,
                                       int theCoordIdx, double theTol, bool theIsMin)
        => AdjustExtrCurve(theCurve, theUMin, theUMax, theExtr0, theCoordIdx, theTol, theIsMin);

    /// <summary>OCCT's <c>AdjustExtrT</c> over a 2D curve.</summary>
    internal static double AdjustExtrT(Adaptor2d_Curve2d theCurve, double theUMin, double theUMax, double theExtr0,
                                       int theCoordIdx, double theTol, bool theIsMin)
        => AdjustExtrCurve2d(theCurve, theUMin, theUMax, theExtr0, theCoordIdx, theTol, theIsMin);
}
