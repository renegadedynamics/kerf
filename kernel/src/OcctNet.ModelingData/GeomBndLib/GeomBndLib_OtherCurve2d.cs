// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_OtherCurve2d.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_OtherCurve2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Computes bounding box for a general 2D curve through its adaptor: sampling with a deflection estimate for
/// <c>Box</c>, sampling + PSO/Brent numerical optimization for <c>BoxOptimal</c>.
/// </summary>
/// <remarks>
/// Keeps a reference to the adaptor, as OCCT does. Unlike the 3D GeomBndLib_OtherCurve, OCCT's file has its own
/// optimizer (<c>AdjustExtr2dCurve</c>) over the legacy math_BrentMinimum and math_PSO, ported here as it is.
/// </remarks>
public sealed class GeomBndLib_OtherCurve2d : GeomBndLib_Curve2d.ICurve2dEvaluator
{
    private readonly Adaptor2d_Curve2d myCurve;

    public GeomBndLib_OtherCurve2d(Adaptor2d_Curve2d theCurve)
    {
        myCurve = theCurve;
    }

    Geom2d_Curve? GeomBndLib_Curve2d.ICurve2dEvaluator.CurveGeometry() => null;

    /// <summary>Fill 2D box with sampled points and return maximum deflection.</summary>
    private static double FillBox2d(ref Bnd_Box2d theBox, Adaptor2d_Curve2d theCurve, double theFirst, double theLast, int theN)
    {
        gp_Pnt2d aP1, aP2, aP3;
        theCurve.D0(theFirst, out aP1);
        theBox.Add(aP1);
        double p = theFirst, dp = theLast - theFirst, tol = 0.0;
        if (Math.Abs(dp) > Precision.PConfusion())
        {
            dp /= 2 * theN;
            for (int i = 1; i <= theN; i++)
            {
                p += dp;
                theCurve.D0(p, out aP2);
                theBox.Add(aP2);
                p += dp;
                theCurve.D0(p, out aP3);
                theBox.Add(aP3);
                gp_Pnt2d aPc = new((aP1.XY() + aP3.XY()) / 2.0);
                tol = Max(tol, aPc.Distance(aP2));
                aP1 = aP3;
            }
        }
        else
        {
            theCurve.D0(theFirst, out aP1);
            theBox.Add(aP1);
            theCurve.D0(theLast, out aP3);
            theBox.Add(aP3);
        }
        return tol;
    }

    /// <summary>1D function object for coordinate minimization/maximization along a 2D curve.</summary>
    private sealed class Curv2dCoordFunc(Adaptor2d_Curve2d theCurve, double theUMin, double theUMax, int theCoordIdx, double theSign)
        : math_Function
    {
        private readonly Adaptor2d_Curve2d myCurve = theCurve;
        private readonly double myUMin = theUMin;
        private readonly double myUMax = theUMax;
        private readonly int myCoordIdx = theCoordIdx;
        private readonly double mySign = theSign;

        public override bool Value(double theX, out double theF)
        {
            if (theX < myUMin || theX > myUMax)
            {
                theF = 0.0;
                return false;
            }
            gp_Pnt2d aP = myCurve.Value(theX);
            theF = mySign * aP.Coord(myCoordIdx);
            return true;
        }
    }

    /// <summary>Multi-variable function for PSO on a 2D curve.</summary>
    private sealed class Curv2dCoordMVarFunc(Adaptor2d_Curve2d theCurve, double theUMin, double theUMax, int theCoordIdx, double theSign)
        : math_MultipleVarFunction
    {
        private readonly Adaptor2d_Curve2d myCurve = theCurve;
        private readonly double myUMin = theUMin;
        private readonly double myUMax = theUMax;
        private readonly int myCoordIdx = theCoordIdx;
        private readonly double mySign = theSign;

        public override bool Value(math_Vector theX, out double theF)
        {
            if (theX[1] < myUMin || theX[1] > myUMax)
            {
                theF = 0.0;
                return false;
            }
            gp_Pnt2d aP = myCurve.Value(theX[1]);
            theF = mySign * aP.Coord(myCoordIdx);
            return true;
        }

        public override int NbVariables() => 1;
    }

    /// <summary>Adjust coordinate extremum using PSO + Brent for 2D curve.</summary>
    private static double AdjustExtr2dCurve(Adaptor2d_Curve2d theCurve,
                                            double theUMin,
                                            double theUMax,
                                            double theExtr0,
                                            int theCoordIdx,
                                            double theTol,
                                            bool theIsMin)
    {
        double aSign = theIsMin ? 1.0 : -1.0;
        double extr = aSign * theExtr0;

        double uTol = Max(theCurve.Resolution(theTol), Precision.PConfusion());
        double Du = (theCurve.LastParameter() - theCurve.FirstParameter());

        double reltol = uTol / Max(Math.Abs(theUMin), Math.Abs(theUMax));
        if (theUMax - theUMin < 0.01 * Du)
        {
            math_BrentMinimum anOptLoc0 = new(reltol, 100, uTol);
            Curv2dCoordFunc aFunc0 = new(theCurve, theUMin, theUMax, theCoordIdx, aSign);
            anOptLoc0.Perform(aFunc0, theUMin, (theUMin + theUMax) / 2.0, theUMax);
            if (anOptLoc0.IsDone())
            {
                extr = anOptLoc0.Minimum();
                return aSign * extr;
            }
        }

        int aNbParticles = Max(8, RealToInt(32 * (theUMax - theUMin) / Du));
        double aMaxStep = (theUMax - theUMin) / (aNbParticles + 1);
        math_Vector aT = new(1, 1);
        math_Vector aLowBorder = new(1, 1);
        math_Vector aUppBorder = new(1, 1);
        math_Vector aSteps = new(1, 1);
        aLowBorder[1] = theUMin;
        aUppBorder[1] = theUMax;
        aSteps[1] = Min(0.1 * Du, aMaxStep);

        Curv2dCoordMVarFunc aFunc = new(theCurve, theUMin, theUMax, theCoordIdx, aSign);
        math_PSO aFinder = new(aFunc, aLowBorder, aUppBorder, aSteps, aNbParticles);
        aFinder.Perform(aSteps, out extr, aT);

        math_BrentMinimum anOptLoc = new(reltol, 100, uTol);
        Curv2dCoordFunc aFunc1 = new(theCurve, theUMin, theUMax, theCoordIdx, aSign);
        anOptLoc.Perform(aFunc1, Max(aT[1] - aSteps[1], theUMin), aT[1], Min(aT[1] + aSteps[1], theUMax));

        if (anOptLoc.IsDone())
        {
            extr = anOptLoc.Minimum();
            return aSign * extr;
        }

        return aSign * extr;
    }

    /// <summary>Compute bounding box for full curve.</summary>
    public Bnd_Box2d Box(double theTol) => Box(myCurve.FirstParameter(), myCurve.LastParameter(), theTol);

    /// <summary>Compute bounding box for arc [theU1, theU2].</summary>
    public Bnd_Box2d Box(double theU1, double theU2, double theTol)
    {
        Bnd_Box2d aBox = new();
        const double weakness = 1.5;
        const int N = 33;
        Bnd_Box2d aB1 = new();
        double tol = FillBox2d(ref aB1, myCurve, theU1, theU2, N);
        aB1.Enlarge(weakness * tol);
        aB1.Get(out double x, out double y, out double X, out double Y);
        aBox.Update(x, y, X, Y);
        aBox.Enlarge(theTol);
        return aBox;
    }

    /// <summary>Compute precise bounding box for full curve.</summary>
    public Bnd_Box2d BoxOptimal(double theTol) => BoxOptimal(myCurve.FirstParameter(), myCurve.LastParameter(), theTol);

    /// <summary>Compute precise bounding box using PSO + Brent optimization.</summary>
    public Bnd_Box2d BoxOptimal(double theU1, double theU2, double theTol)
    {
        Bnd_Box2d aBox = new();
        Adaptor2d_Curve2d C = myCurve;
        int Nu = GeomBndLib_SamplingHelpers.ComputeNbSamples2d(C, theU1, theU2);

        Span<double> CoordMin = [RealLast(), RealLast()];
        Span<double> CoordMax = [-RealLast(), -RealLast()];
        Span<double> DeflMax = [-RealLast(), -RealLast()];

        gp_Pnt2d P;
        double du = (theU2 - theU1) / (Nu - 1), du2 = du / 2.0;
        NCollection_Array1<gp_XY> aPnts = new(1, Nu);
        double u;
        for (int i = 1, j = 0; i <= Nu; i++, j++)
        {
            u = theU1 + j * du;
            C.D0(u, out P);
            aPnts[i] = P.XY();
            for (int k = 0; k < 2; ++k)
            {
                if (CoordMin[k] > P.Coord(k + 1))
                {
                    CoordMin[k] = P.Coord(k + 1);
                }
                if (CoordMax[k] < P.Coord(k + 1))
                {
                    CoordMax[k] = P.Coord(k + 1);
                }
            }
            if (i > 1)
            {
                gp_XY aPm = 0.5 * (aPnts[i - 1] + aPnts[i]);
                C.D0(u - du2, out P);
                gp_XY aD = (P.XY() - aPm);
                for (int k = 0; k < 2; ++k)
                {
                    if (CoordMin[k] > P.Coord(k + 1))
                    {
                        CoordMin[k] = P.Coord(k + 1);
                    }
                    if (CoordMax[k] < P.Coord(k + 1))
                    {
                        CoordMax[k] = P.Coord(k + 1);
                    }
                    double d = Math.Abs(aD.Coord(k + 1));
                    if (DeflMax[k] < d)
                    {
                        DeflMax[k] = d;
                    }
                }
            }
        }

        double eps = Max(theTol, Precision.Confusion());
        for (int k = 0; k < 2; ++k)
        {
            double d = DeflMax[k];
            if (d <= eps)
            {
                continue;
            }
            double CMin = CoordMin[k];
            double CMax = CoordMax[k];
            for (int i = 1; i <= Nu; ++i)
            {
                if (aPnts[i].Coord(k + 1) - CMin < d)
                {
                    double umin = theU1 + Max(0, i - 2) * du;
                    double umax = theU1 + Min(Nu - 1, i) * du;
                    double cmin = AdjustExtr2dCurve(C, umin, umax, CMin, k + 1, eps, true);
                    if (cmin < CMin)
                    {
                        CMin = cmin;
                    }
                }
                else if (CMax - aPnts[i].Coord(k + 1) < d)
                {
                    double umin = theU1 + Max(0, i - 2) * du;
                    double umax = theU1 + Min(Nu - 1, i) * du;
                    double cmax = AdjustExtr2dCurve(C, umin, umax, CMax, k + 1, eps, false);
                    if (cmax > CMax)
                    {
                        CMax = cmax;
                    }
                }
            }
            CoordMin[k] = CMin;
            CoordMax[k] = CMax;
        }

        aBox.Add(new gp_Pnt2d(CoordMin[0], CoordMin[1]));
        aBox.Add(new gp_Pnt2d(CoordMax[0], CoordMax[1]));
        aBox.Enlarge(eps);
        return aBox;
    }
}
