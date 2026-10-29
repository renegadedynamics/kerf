// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_OtherCurve.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_OtherCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Computes bounding box for a general 3D curve through its adaptor: sampling with a deflection estimate for
/// <c>Box</c>, sampling + PSO/Brent numerical optimization for <c>BoxOptimal</c>.
/// </summary>
/// <remarks>Keeps a reference to the adaptor, as OCCT does (its <c>std::reference_wrapper</c>).</remarks>
public sealed class GeomBndLib_OtherCurve : GeomBndLib_Curve.ICurveEvaluator
{
    private readonly Adaptor3d_Curve myCurve;

    public GeomBndLib_OtherCurve(Adaptor3d_Curve theCurve)
    {
        myCurve = theCurve;
    }

    Geom_Curve? GeomBndLib_Curve.ICurveEvaluator.CurveGeometry() => null;

    /// <summary>Fill box with sampled points and return maximum deflection.</summary>
    private static double FillBox(ref Bnd_Box theBox, Adaptor3d_Curve theCurve, double theFirst, double theLast, int theN)
    {
        gp_Pnt aP1, aP2, aP3;
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
                gp_Pnt aPc = new((aP1.XYZ() + aP3.XYZ()) / 2.0);
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

    /// <summary>Compute bounding box for full curve.</summary>
    public Bnd_Box Box(double theTol) => Box(myCurve.FirstParameter(), myCurve.LastParameter(), theTol);

    /// <summary>Compute bounding box for arc [theU1, theU2].</summary>
    public Bnd_Box Box(double theU1, double theU2, double theTol)
    {
        Bnd_Box aBox = new();
        const double weakness = 1.5;
        const int N = 33;
        Bnd_Box aB1 = new();
        double tol = FillBox(ref aB1, myCurve, theU1, theU2, N);
        aB1.Enlarge(weakness * tol);
        aB1.Get(out double x, out double y, out double z, out double X, out double Y, out double Z);
        aBox.Update(x, y, z, X, Y, Z);
        aBox.Enlarge(theTol);
        return aBox;
    }

    /// <summary>Compute precise bounding box for full curve.</summary>
    public Bnd_Box BoxOptimal(double theTol) => BoxOptimal(myCurve.FirstParameter(), myCurve.LastParameter(), theTol);

    /// <summary>Compute precise bounding box using PSO + Brent optimization.</summary>
    public Bnd_Box BoxOptimal(double theU1, double theU2, double theTol)
    {
        Bnd_Box aBox = new();
        Adaptor3d_Curve C = myCurve;
        int Nu = GeomBndLib_SamplingHelpers.ComputeNbSamples(C, theU1, theU2);

        Span<double> CoordMin = [RealLast(), RealLast(), RealLast()];
        Span<double> CoordMax = [-RealLast(), -RealLast(), -RealLast()];
        Span<double> DeflMax = [-RealLast(), -RealLast(), -RealLast()];

        gp_Pnt P;
        double du = (theU2 - theU1) / (Nu - 1), du2 = du / 2.0;
        NCollection_Array1<gp_XYZ> aPnts = new(1, Nu);
        double u;
        for (int i = 1, j = 0; i <= Nu; i++, j++)
        {
            u = theU1 + j * du;
            C.D0(u, out P);
            aPnts[i] = P.XYZ();
            for (int k = 0; k < 3; ++k)
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
                gp_XYZ aPm = 0.5 * (aPnts[i - 1] + aPnts[i]);
                C.D0(u - du2, out P);
                gp_XYZ aD = (P.XYZ() - aPm);
                for (int k = 0; k < 3; ++k)
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
        for (int k = 0; k < 3; ++k)
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
                    double cmin = GeomBndLib_OptimizationHelpers.AdjustExtrCurve(C, umin, umax, CMin, k + 1, eps, true);
                    if (cmin < CMin)
                    {
                        CMin = cmin;
                    }
                }
                else if (CMax - aPnts[i].Coord(k + 1) < d)
                {
                    double umin = theU1 + Max(0, i - 2) * du;
                    double umax = theU1 + Min(Nu - 1, i) * du;
                    double cmax = GeomBndLib_OptimizationHelpers.AdjustExtrCurve(C, umin, umax, CMax, k + 1, eps, false);
                    if (cmax > CMax)
                    {
                        CMax = cmax;
                    }
                }
            }
            CoordMin[k] = CMin;
            CoordMax[k] = CMax;
        }

        aBox.Add(new gp_Pnt(CoordMin[0], CoordMin[1], CoordMin[2]));
        aBox.Add(new gp_Pnt(CoordMax[0], CoordMax[1], CoordMax[2]));
        aBox.Enlarge(eps);
        return aBox;
    }
}
