// OCCT.NET's own tests (not a port of an OCCT GTest): BSplSLib checked against an independent tensor-product
// evaluation (BSplCLib's B-spline basis), against its own cache, and by round trips of its operations.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Kerf.Kernel.Foundation.Tests;

public class BSplSLib_SanityTest
{
    // A random surface: knots 0, ~1, ~2, ... in each direction, end multiplicities Degree + 1 (or all equal when
    // periodic), random poles and (optionally) weights.
    private sealed class Surface
    {
        public int UDegree, VDegree;
        public bool UPeriodic, VPeriodic;
        public NCollection_Array1<double> UKnots = null!, VKnots = null!;
        public NCollection_Array1<int> UMults = null!, VMults = null!;
        public NCollection_Array1<double> UFlat = null!, VFlat = null!;
        public NCollection_Array2<Point3D> Poles = null!;
        public NCollection_Array2<double>? Weights;

        public bool Rational => Weights != null;
    }

    private static void MakeKnots(Random theRnd, int theDegree, bool thePeriodic, int theNbKnots,
                                  out NCollection_Array1<double> theKnots, out NCollection_Array1<int> theMults)
    {
        theKnots = new NCollection_Array1<double>(1, theNbKnots);
        theMults = new NCollection_Array1<int>(1, theNbKnots);
        double k = theRnd.NextDouble() - 0.5;
        for (int i = 1; i <= theNbKnots; i++)
        {
            theKnots[i] = k;
            k += 0.5 + theRnd.NextDouble();
            theMults[i] = thePeriodic ? 1 + theRnd.Next(theDegree)
                                      : (i == 1 || i == theNbKnots ? theDegree + 1 : 1 + theRnd.Next(theDegree));
        }
        if (thePeriodic)
        {
            theMults[theNbKnots] = theMults[1];
        }
    }

    private static Surface MakeSurface(Random theRnd, int theUDeg, int theVDeg, bool theRat, bool theUPer, bool theVPer)
    {
        Surface s = new() { UDegree = theUDeg, VDegree = theVDeg, UPeriodic = theUPer, VPeriodic = theVPer };
        MakeKnots(theRnd, theUDeg, theUPer, theUPer ? Math.Max(5, theUDeg + 2) : 4 + theRnd.Next(3), out s.UKnots, out s.UMults);
        MakeKnots(theRnd, theVDeg, theVPer, theVPer ? Math.Max(5, theVDeg + 2) : 4 + theRnd.Next(3), out s.VKnots, out s.VMults);
        int aNbU = global::Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(theUDeg, theUPer, s.UMults);
        int aNbV = global::Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(theVDeg, theVPer, s.VMults);
        s.Poles = new NCollection_Array2<Point3D>(1, aNbU, 1, aNbV);
        for (int i = 1; i <= aNbU; i++)
        {
            for (int j = 1; j <= aNbV; j++)
            {
                s.Poles[i, j] = new Point3D(theRnd.NextDouble() * 10, theRnd.NextDouble() * 10, theRnd.NextDouble() * 10);
            }
        }
        if (theRat)
        {
            s.Weights = new NCollection_Array2<double>(1, aNbU, 1, aNbV);
            for (int i = 1; i <= aNbU; i++)
            {
                for (int j = 1; j <= aNbV; j++)
                {
                    s.Weights[i, j] = 0.5 + theRnd.NextDouble();
                }
            }
        }
        s.UFlat = new NCollection_Array1<double>(1, global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequenceLength(s.UMults, theUDeg, theUPer));
        global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequence(s.UKnots, s.UMults, theUDeg, theUPer, s.UFlat);
        s.VFlat = new NCollection_Array1<double>(1, global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequenceLength(s.VMults, theVDeg, theVPer));
        global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequence(s.VKnots, s.VMults, theVDeg, theVPer, s.VFlat);
        return s;
    }

    private static IEnumerable<Surface> Surfaces()
    {
        Random aRnd = new(20261029);
        int[][] aDegrees = [[1, 1], [1, 3], [2, 2], [3, 2], [3, 3], [2, 5], [5, 4]];
        foreach (int[] aDeg in aDegrees)
        {
            foreach (bool aRat in new[] { false, true })
            {
                foreach ((bool aUPer, bool aVPer) in new[] { (false, false), (true, false), (false, true), (true, true) })
                {
                    yield return MakeSurface(aRnd, aDeg[0], aDeg[1], aRat, aUPer, aVPer);
                }
            }
        }
    }

    private static IEnumerable<(double U, double V)> Params(Surface s, int theNb = 7)
    {
        double u0 = s.UKnots.First(), u1 = s.UKnots.Last(), v0 = s.VKnots.First(), v1 = s.VKnots.Last();
        for (int i = 0; i <= theNb; i++)
        {
            for (int j = 0; j <= theNb; j++)
            {
                // a little inside so that every parameter is in a span of the reference's basis
                yield return (u0 + (u1 - u0) * (0.001 + 0.998 * i / theNb), v0 + (v1 - v0) * (0.001 + 0.998 * j / theNb));
            }
        }
    }

    private static void AssertNear(in Xyz a, in Xyz b, double tol, string theWhat = "")
    {
        double aScale = Math.Max(1.0, b.Modulus());
        Assert.True((a - b).Modulus() <= tol * aScale, $"{theWhat}: {a} != {b}");
    }

    // Reference: the homogeneous derivatives A(k, l) (numerator) and W(k, l) (denominator) from the basis functions,
    // then the rational derivatives by Leibniz's rule. Returns S(k, l) for k <= theN, l <= theM.
    private static Xyz[,] Reference(Surface s, double u, double v, int theN, int theM)
    {
        int aNbU = s.Poles.ColLength(), aNbV = s.Poles.RowLength();
        Matrix aUBasis = new(1, theN + 1, 1, s.UDegree + 1);
        Matrix aVBasis = new(1, theM + 1, 1, s.VDegree + 1);
        Assert.Equal(0, global::Kerf.Engine.Geometry.Splines.BSplCLib.EvalBsplineBasis(theN, s.UDegree + 1, s.UFlat, u, out int aUFirst, aUBasis));
        Assert.Equal(0, global::Kerf.Engine.Geometry.Splines.BSplCLib.EvalBsplineBasis(theM, s.VDegree + 1, s.VFlat, v, out int aVFirst, aVBasis));
        var A = new Xyz[theN + 1, theM + 1];
        var W = new double[theN + 1, theM + 1];
        for (int k = 0; k <= theN; k++)
        {
            for (int l = 0; l <= theM; l++)
            {
                Xyz aSum = new(0, 0, 0);
                double aW = 0;
                for (int i = 1; i <= s.UDegree + 1; i++)
                {
                    int ip = (aUFirst + i - 2) % aNbU + 1;
                    for (int j = 1; j <= s.VDegree + 1; j++)
                    {
                        int jp = (aVFirst + j - 2) % aNbV + 1;
                        double b = (k <= s.UDegree ? aUBasis[k + 1, i] : 0.0) * (l <= s.VDegree ? aVBasis[l + 1, j] : 0.0);
                        double w = s.Weights?[ip, jp] ?? 1.0;
                        aSum += s.Poles[ip, jp].XYZ() * (b * w);
                        aW += b * w;
                    }
                }
                A[k, l] = aSum;
                W[k, l] = aW;
            }
        }
        var S = new Xyz[theN + 1, theM + 1];
        for (int k = 0; k <= theN; k++)
        {
            for (int l = 0; l <= theM; l++)
            {
                Xyz aR = A[k, l];
                for (int p = 0; p <= k; p++)
                {
                    for (int q = 0; q <= l; q++)
                    {
                        if (p == k && q == l)
                        {
                            continue;
                        }
                        aR -= S[p, q] * (global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(k, p) * global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(l, q) * W[k - p, l - q]);
                    }
                }
                S[k, l] = aR / W[0, 0];
            }
        }
        return S;
    }

    [Fact]
    public void D0ToD3_MatchTensorProduct()
    {
        foreach (Surface s in Surfaces())
        {
            foreach ((double u, double v) in Params(s))
            {
                Xyz[,] R = Reference(s, u, v, 3, 3);
                string aWhat = $"deg {s.UDegree}x{s.VDegree} rat {s.Rational} per {s.UPeriodic}/{s.VPeriodic} at ({u}, {v})";

                global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.D0(u, v, 0, 0, s.Poles, s.Weights, s.UKnots, s.VKnots, s.UMults, s.VMults, s.UDegree,
                            s.VDegree, s.Rational, s.Rational, s.UPeriodic, s.VPeriodic, out Point3D P0);
                AssertNear(P0.XYZ(), R[0, 0], 1e-11, "D0 " + aWhat);

                global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.D1(u, v, 0, 0, s.Poles, s.Weights, s.UKnots, s.VKnots, s.UMults, s.VMults, s.UDegree,
                            s.VDegree, s.Rational, s.Rational, s.UPeriodic, s.VPeriodic, out Point3D P1, out Vector3D Du,
                            out Vector3D Dv);
                AssertNear(P1.XYZ(), R[0, 0], 1e-11, "D1.P " + aWhat);
                AssertNear(Du.XYZ(), R[1, 0], 1e-10, "D1.U " + aWhat);
                AssertNear(Dv.XYZ(), R[0, 1], 1e-10, "D1.V " + aWhat);

                global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.D2(u, v, 0, 0, s.Poles, s.Weights, s.UKnots, s.VKnots, s.UMults, s.VMults, s.UDegree,
                            s.VDegree, s.Rational, s.Rational, s.UPeriodic, s.VPeriodic, out Point3D P2, out Du, out Dv,
                            out Vector3D Duu, out Vector3D Dvv, out Vector3D Duv);
                AssertNear(P2.XYZ(), R[0, 0], 1e-11, "D2.P " + aWhat);
                AssertNear(Du.XYZ(), R[1, 0], 1e-10, "D2.U " + aWhat);
                AssertNear(Dv.XYZ(), R[0, 1], 1e-10, "D2.V " + aWhat);
                AssertNear(Duu.XYZ(), R[2, 0], 1e-10, "D2.UU " + aWhat);
                AssertNear(Dvv.XYZ(), R[0, 2], 1e-10, "D2.VV " + aWhat);
                AssertNear(Duv.XYZ(), R[1, 1], 1e-10, "D2.UV " + aWhat);

                global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.D3(u, v, 0, 0, s.Poles, s.Weights, s.UKnots, s.VKnots, s.UMults, s.VMults, s.UDegree,
                            s.VDegree, s.Rational, s.Rational, s.UPeriodic, s.VPeriodic, out Point3D P3, out Du, out Dv,
                            out Duu, out Dvv, out Duv, out Vector3D Duuu, out Vector3D Dvvv, out Vector3D Duuv, out Vector3D Duvv);
                AssertNear(P3.XYZ(), R[0, 0], 1e-11, "D3.P " + aWhat);
                AssertNear(Du.XYZ(), R[1, 0], 1e-10, "D3.U " + aWhat);
                AssertNear(Dv.XYZ(), R[0, 1], 1e-10, "D3.V " + aWhat);
                AssertNear(Duu.XYZ(), R[2, 0], 1e-10, "D3.UU " + aWhat);
                AssertNear(Dvv.XYZ(), R[0, 2], 1e-10, "D3.VV " + aWhat);
                AssertNear(Duv.XYZ(), R[1, 1], 1e-10, "D3.UV " + aWhat);
                AssertNear(Duuu.XYZ(), R[3, 0], 1e-9, "D3.UUU " + aWhat);
                AssertNear(Dvvv.XYZ(), R[0, 3], 1e-9, "D3.VVV " + aWhat);
                AssertNear(Duuv.XYZ(), R[2, 1], 1e-9, "D3.UUV " + aWhat);
                AssertNear(Duvv.XYZ(), R[1, 2], 1e-9, "D3.UVV " + aWhat);

                for (int nu = 0; nu <= 3; nu++)
                {
                    for (int nv = 0; nv <= 3; nv++)
                    {
                        if (nu + nv == 0)
                        {
                            continue;
                        }
                        global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.DN(u, v, nu, nv, 0, 0, s.Poles, s.Weights, s.UKnots, s.VKnots, s.UMults, s.VMults,
                                    s.UDegree, s.VDegree, s.Rational, s.Rational, s.UPeriodic, s.VPeriodic, out Vector3D Dn);
                        AssertNear(Dn.XYZ(), R[nu, nv], 1e-9, $"DN({nu},{nv}) " + aWhat);
                    }
                }

                // Homogeneous forms
                global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.HomogeneousD1(u, v, 0, 0, s.Poles, s.Weights, s.UKnots, s.VKnots, s.UMults, s.VMults,
                                       s.UDegree, s.VDegree, s.Rational, s.Rational, s.UPeriodic, s.VPeriodic,
                                       out Point3D N, out Vector3D Nu, out Vector3D Nv, out double D, out double Ddu, out double Ddv);
                AssertNear(N.XYZ() / D, R[0, 0], 1e-11, "HD1 " + aWhat);
                AssertNear(((Nu.XYZ() - N.XYZ() * (Ddu / D)) / D), R[1, 0], 1e-10, "HD1.U " + aWhat);
                AssertNear(((Nv.XYZ() - N.XYZ() * (Ddv / D)) / D), R[0, 1], 1e-10, "HD1.V " + aWhat);
            }
        }
    }

    [Fact]
    public void FlatKnots_and_located_indices_give_the_same_point()
    {
        foreach (Surface s in Surfaces())
        {
            foreach ((double u, double v) in Params(s, 4))
            {
                global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.D1(u, v, 0, 0, s.Poles, s.Weights, s.UKnots, s.VKnots, s.UMults, s.VMults, s.UDegree,
                            s.VDegree, s.Rational, s.Rational, s.UPeriodic, s.VPeriodic, out Point3D P, out Vector3D Du, out Vector3D Dv);
                global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.D1(u, v, 0, 0, s.Poles, s.Weights, s.UFlat, s.VFlat, null, null, s.UDegree,
                            s.VDegree, s.Rational, s.Rational, s.UPeriodic, s.VPeriodic, out Point3D FP, out Vector3D FDu, out Vector3D FDv);
                Assert.Equal(P, FP);
                Assert.Equal(Du, FDu);
                Assert.Equal(Dv, FDv);
            }
        }
    }

    // Builds the cache of the span of (u, v) with the first BuildCache, checks the second one's array against it.
    private static (NCollection_Array2<Point3D> Poles, NCollection_Array2<double>? Weights, double U0, double V0,
                    double ULen, double VLen) Cache(Surface s, double u, double v)
    {
        int aUIndex = 0, aVIndex = 0;
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(s.UDegree, s.UFlat, null, u, s.UPeriodic, ref aUIndex, out _);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(s.VDegree, s.VFlat, null, v, s.VPeriodic, ref aVIndex, out _);
        double U0 = s.UFlat[aUIndex], ULen = s.UFlat[aUIndex + 1] - U0;
        double V0 = s.VFlat[aVIndex], VLen = s.VFlat[aVIndex + 1] - V0;
        int aMax = Math.Max(s.UDegree, s.VDegree), aMin = Math.Min(s.UDegree, s.VDegree);
        NCollection_Array2<Point3D> aPoles = new(1, aMax + 1, 1, aMin + 1);
        NCollection_Array2<double>? aWeights = s.Rational ? new NCollection_Array2<double>(1, aMax + 1, 1, aMin + 1) : null;
        global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.BuildCache(U0, V0, ULen, VLen, s.UPeriodic, s.VPeriodic, s.UDegree, s.VDegree, aUIndex, aVIndex,
                            s.UFlat, s.VFlat, s.Poles, s.Weights, aPoles, aWeights);

        int aDim = s.Rational ? 4 : 3;
        NCollection_Array2<double> anArray = new(1, aMax + 1, 1, aDim * (aMin + 1));
        global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.BuildCache(U0, V0, ULen, VLen, s.UPeriodic, s.VPeriodic, s.UDegree, s.VDegree, aUIndex, aVIndex,
                            s.UFlat, s.VFlat, s.Poles, s.Weights, anArray);
        for (int r = 1; r <= aMax + 1; r++)
        {
            for (int c = 1; c <= aMin + 1; c++)
            {
                Point3D aP = aPoles[r, c];
                Assert.Equal(aP.X(), anArray[r, (c - 1) * aDim + 1]);
                Assert.Equal(aP.Y(), anArray[r, (c - 1) * aDim + 2]);
                Assert.Equal(aP.Z(), anArray[r, (c - 1) * aDim + 3]);
                if (aWeights != null)
                {
                    Assert.Equal(aWeights[r, c], anArray[r, (c - 1) * aDim + 4]);
                }
            }
        }
        return (aPoles, aWeights, U0, V0, ULen, VLen);
    }

    [Fact]
    public void CacheD0ToD2_MatchD0ToD2()
    {
        foreach (Surface s in Surfaces())
        {
            foreach ((double u, double v) in Params(s, 5))
            {
                var c = Cache(s, u, v);
                global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.D2(u, v, 0, 0, s.Poles, s.Weights, s.UKnots, s.VKnots, s.UMults, s.VMults, s.UDegree,
                            s.VDegree, s.Rational, s.Rational, s.UPeriodic, s.VPeriodic, out Point3D P, out Vector3D Du,
                            out Vector3D Dv, out Vector3D Duu, out Vector3D Dvv, out Vector3D Duv);
                string aWhat = $"deg {s.UDegree}x{s.VDegree} rat {s.Rational} per {s.UPeriodic}/{s.VPeriodic} at ({u}, {v})";

                global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.CacheD0(u, v, s.UDegree, s.VDegree, c.U0, c.V0, c.ULen, c.VLen, c.Poles, c.Weights, out Point3D CP0);
                AssertNear(CP0.XYZ(), P.XYZ(), 1e-11, "CacheD0 " + aWhat);

                global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.CacheD1(u, v, s.UDegree, s.VDegree, c.U0, c.V0, c.ULen, c.VLen, c.Poles, c.Weights,
                                 out Point3D CP1, out Vector3D CDu, out Vector3D CDv);
                AssertNear(CP1.XYZ(), P.XYZ(), 1e-11, "CacheD1.P " + aWhat);
                AssertNear(CDu.XYZ(), Du.XYZ(), 1e-10, "CacheD1.U " + aWhat);
                AssertNear(CDv.XYZ(), Dv.XYZ(), 1e-10, "CacheD1.V " + aWhat);

                global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.CacheD2(u, v, s.UDegree, s.VDegree, c.U0, c.V0, c.ULen, c.VLen, c.Poles, c.Weights,
                                 out Point3D CP2, out CDu, out CDv, out Vector3D CDuu, out Vector3D CDuv, out Vector3D CDvv);
                AssertNear(CP2.XYZ(), P.XYZ(), 1e-11, "CacheD2.P " + aWhat);
                AssertNear(CDu.XYZ(), Du.XYZ(), 1e-10, "CacheD2.U " + aWhat);
                AssertNear(CDv.XYZ(), Dv.XYZ(), 1e-10, "CacheD2.V " + aWhat);
                // OCCT: CacheD2 reads stale values as the second derivative of the weights when both degrees are
                // below 2 (occt-issues.md), so the second derivatives of a rational bilinear surface are wrong
                if (!s.Rational || Math.Max(s.UDegree, s.VDegree) >= 2)
                {
                    AssertNear(CDuu.XYZ(), Duu.XYZ(), 1e-9, "CacheD2.UU " + aWhat);
                    AssertNear(CDuv.XYZ(), Duv.XYZ(), 1e-9, "CacheD2.UV " + aWhat);
                    AssertNear(CDvv.XYZ(), Dvv.XYZ(), 1e-9, "CacheD2.VV " + aWhat);
                }
            }
        }
    }

    [Fact]
    public void BezierCoefficients_MatchEvaluation()
    {
        Random aRnd = new(7);
        foreach ((int aUDeg, int aVDeg) in new[] { (1, 1), (2, 3), (3, 2), (4, 4), (5, 2), (1, 4) })
        {
            foreach (bool aRat in new[] { false, true })
            {
                NCollection_Array2<Point3D> aPoles = new(1, aUDeg + 1, 1, aVDeg + 1);
                NCollection_Array2<double>? aWeights = aRat ? new NCollection_Array2<double>(1, aUDeg + 1, 1, aVDeg + 1) : null;
                for (int i = 1; i <= aUDeg + 1; i++)
                {
                    for (int j = 1; j <= aVDeg + 1; j++)
                    {
                        aPoles[i, j] = new Point3D(aRnd.NextDouble(), aRnd.NextDouble(), aRnd.NextDouble());
                        if (aWeights != null)
                        {
                            aWeights[i, j] = 0.5 + aRnd.NextDouble();
                        }
                    }
                }
                NCollection_Array1<double> aUKnots = new(1, 2), aVKnots = new(1, 2);
                aUKnots[1] = aVKnots[1] = 0.0;
                aUKnots[2] = aVKnots[2] = 1.0;
                NCollection_Array1<int> aUMults = new(1, 2), aVMults = new(1, 2);
                aUMults.Init(aUDeg + 1);
                aVMults.Init(aVDeg + 1);

                NCollection_Array2<Point3D> aCoefs = new(1, aUDeg + 1, 1, aVDeg + 1);
                NCollection_Array2<double>? aWCoefs = aRat ? new NCollection_Array2<double>(1, aUDeg + 1, 1, aVDeg + 1) : null;
                if (aRat)
                {
                    global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.PolesCoefficients(aPoles, aWeights, aCoefs, aWCoefs);
                }
                else
                {
                    global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.PolesCoefficients(aPoles, aCoefs);
                }
                for (int iu = 0; iu <= 4; iu++)
                {
                    for (int iv = 0; iv <= 4; iv++)
                    {
                        double u = iu / 4.0, v = iv / 4.0;
                        global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.D2(u, v, 0, 0, aPoles, aWeights, aUKnots, aVKnots, aUMults, aVMults, aUDeg, aVDeg,
                                    aRat, aRat, false, false, out Point3D P, out Vector3D Du, out Vector3D Dv,
                                    out Vector3D Duu, out Vector3D Dvv, out Vector3D Duv);

                        // The coefficients themselves, indexed (U, V): in the cache layout ([max degree][min degree])
                        // they evaluate to the surface
                        NCollection_Array2<Point3D> aLayout = aCoefs;
                        NCollection_Array2<double>? aWLayout = aWCoefs;
                        if (aUDeg <= aVDeg)
                        {
                            aLayout = new NCollection_Array2<Point3D>(1, aVDeg + 1, 1, aUDeg + 1);
                            aWLayout = aRat ? new NCollection_Array2<double>(1, aVDeg + 1, 1, aUDeg + 1) : null;
                            for (int i = 1; i <= aUDeg + 1; i++)
                            {
                                for (int j = 1; j <= aVDeg + 1; j++)
                                {
                                    aLayout[j, i] = aCoefs[i, j];
                                    if (aWLayout != null)
                                    {
                                        aWLayout[j, i] = aWCoefs![i, j];
                                    }
                                }
                            }
                        }
                        global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.CacheD0(u, v, aUDeg, aVDeg, 0.0, 0.0, 1.0, 1.0, aLayout, aWLayout, out Point3D LP);
                        AssertNear(LP.XYZ(), P.XYZ(), 1e-12, $"Coefficients {aUDeg}x{aVDeg} {aRat} ({u},{v})");

                        // OCCT: CoefsD0/D1/D2 pass the V degree as the U degree (occt-issues.md). With at least as many
                        // U poles as V poles they evaluate the surface at (V, U), the U and V derivatives exchanged, so
                        // they are called with the parameters swapped; with fewer, the result is meaningless
                        if (aUDeg < aVDeg)
                        {
                            continue;
                        }
                        global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.CoefsD0(v, u, aCoefs, aWCoefs, out Point3D CP);
                        AssertNear(CP.XYZ(), P.XYZ(), 1e-12, $"CoefsD0 {aUDeg}x{aVDeg} {aRat} ({u},{v})");
                        global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.CoefsD1(v, u, aCoefs, aWCoefs, out CP, out Vector3D CDv, out Vector3D CDu);
                        AssertNear(CDu.XYZ(), Du.XYZ(), 1e-11, $"CoefsD1.U {aUDeg}x{aVDeg} {aRat} ({u},{v})");
                        AssertNear(CDv.XYZ(), Dv.XYZ(), 1e-11, "CoefsD1.V");
                        global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.CoefsD2(v, u, aCoefs, aWCoefs, out CP, out CDv, out CDu, out Vector3D CDvv,
                                         out Vector3D CDuv, out Vector3D CDuu);
                        if (!aRat || Math.Max(aUDeg, aVDeg) >= 2) // OCCT: see CacheD0ToD2_MatchD0ToD2
                        {
                            AssertNear(CDuu.XYZ(), Duu.XYZ(), 1e-10, "CoefsD2.UU");
                            AssertNear(CDuv.XYZ(), Duv.XYZ(), 1e-10, "CoefsD2.UV");
                            AssertNear(CDvv.XYZ(), Dvv.XYZ(), 1e-10, "CoefsD2.VV");
                        }
                    }
                }
            }
        }
    }

    private static void AssertSameSurface(Surface s, NCollection_Array2<Point3D> thePoles, NCollection_Array2<double>? theWeights,
                                          NCollection_Array1<double> theUKnots, NCollection_Array1<int> theUMults, int theUDeg,
                                          bool theUPer, NCollection_Array1<double> theVKnots, NCollection_Array1<int> theVMults,
                                          int theVDeg, bool theVPer, string theWhat)
    {
        foreach ((double u, double v) in Params(s, 5))
        {
            global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.D0(u, v, 0, 0, s.Poles, s.Weights, s.UKnots, s.VKnots, s.UMults, s.VMults, s.UDegree,
                        s.VDegree, s.Rational, s.Rational, s.UPeriodic, s.VPeriodic, out Point3D P);
            global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.D0(u, v, 0, 0, thePoles, theWeights, theUKnots, theVKnots, theUMults, theVMults, theUDeg,
                        theVDeg, theWeights != null, theWeights != null, theUPer, theVPer, out Point3D Q);
            AssertNear(Q.XYZ(), P.XYZ(), 1e-10, theWhat);
        }
    }

    [Fact]
    public void KnotOperations_KeepTheSurface()
    {
        foreach (Surface s in Surfaces())
        {
            foreach (bool aUDir in new[] { true, false })
            {
                int aDeg = aUDir ? s.UDegree : s.VDegree;
                bool aPer = aUDir ? s.UPeriodic : s.VPeriodic;
                NCollection_Array1<double> aKnots = aUDir ? s.UKnots : s.VKnots;
                NCollection_Array1<int> aMults = aUDir ? s.UMults : s.VMults;
                int aNbU = s.Poles.ColLength(), aNbV = s.Poles.RowLength();

                NCollection_Array2<Point3D> Make(int theNb) => aUDir ? new(1, theNb, 1, aNbV) : new(1, aNbU, 1, theNb);
                NCollection_Array2<double>? MakeW(int theNb) =>
                    s.Rational ? (aUDir ? new(1, theNb, 1, aNbV) : new(1, aNbU, 1, theNb)) : null;
                void Check(NCollection_Array2<Point3D> P, NCollection_Array2<double>? W, NCollection_Array1<double> K,
                           NCollection_Array1<int> M, int D, bool Per, string theWhat)
                {
                    if (aUDir)
                    {
                        AssertSameSurface(s, P, W, K, M, D, Per, s.VKnots, s.VMults, s.VDegree, s.VPeriodic, theWhat);
                    }
                    else
                    {
                        AssertSameSurface(s, P, W, s.UKnots, s.UMults, s.UDegree, s.UPeriodic, K, M, D, Per, theWhat);
                    }
                }

                // Insert a knot in the middle of the first span, then remove it
                NCollection_Array1<double> anAdd = new(1, 1);
                anAdd[1] = 0.5 * (aKnots[1] + aKnots[2]);
                NCollection_Array1<int> anAddMults = new(1, 1);
                anAddMults[1] = 1;
                Assert.True(global::Kerf.Engine.Geometry.Splines.BSplCLib.PrepareInsertKnots(aDeg, aPer, aKnots, aMults, anAdd, anAddMults, out int aNbPoles,
                                                        out int aNbKnots, 0.0));
                NCollection_Array2<Point3D> aNewPoles = Make(aNbPoles);
                NCollection_Array2<double>? aNewWeights = MakeW(aNbPoles);
                NCollection_Array1<double> aNewKnots = new(1, aNbKnots);
                NCollection_Array1<int> aNewMults = new(1, aNbKnots);
                global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.InsertKnots(aUDir, aDeg, aPer, s.Poles, s.Weights, aKnots, aMults, anAdd, anAddMults, aNewPoles,
                                     aNewWeights, aNewKnots, aNewMults, 0.0);
                Check(aNewPoles, aNewWeights, aNewKnots, aNewMults, aDeg, aPer, $"InsertKnots {aUDir}");

                NCollection_Array2<Point3D> aBackPoles = Make(aNbPoles - 1);
                NCollection_Array2<double>? aBackWeights = MakeW(aNbPoles - 1);
                NCollection_Array1<double> aBackKnots = new(1, aNbKnots - 1);
                NCollection_Array1<int> aBackMults = new(1, aNbKnots - 1);
                Assert.True(global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.RemoveKnot(aUDir, 2, 0, aDeg, aPer, aNewPoles, aNewWeights, aNewKnots, aNewMults,
                                                aBackPoles, aBackWeights, aBackKnots, aBackMults, 1e-7));
                Check(aBackPoles, aBackWeights, aBackKnots, aBackMults, aDeg, aPer, $"RemoveKnot {aUDir}");
                for (int i = 1; i <= s.Poles.ColLength(); i++)
                {
                    for (int j = 1; j <= s.Poles.RowLength(); j++)
                    {
                        AssertNear(aBackPoles[i, j].XYZ(), s.Poles[i, j].XYZ(), 1e-9, "RemoveKnot poles");
                    }
                }

                // Increase the degree (non-periodic, as OCCT's periodic IncreaseDegree is broken)
                if (!aPer)
                {
                    int aNewDeg = aDeg + 1;
                    int aNbK = global::Kerf.Engine.Geometry.Splines.BSplCLib.IncreaseDegreeCountKnots(aDeg, aNewDeg, aPer, aMults);
                    int aFirst = global::Kerf.Engine.Geometry.Splines.BSplCLib.FirstUKnotIndex(aDeg, aMults), aLast = global::Kerf.Engine.Geometry.Splines.BSplCLib.LastUKnotIndex(aDeg, aMults);
                    int aNb = (aUDir ? aNbU : aNbV) + (aNewDeg - aDeg) * (aLast - aFirst);
                    NCollection_Array2<Point3D> anIncPoles = Make(aNb);
                    NCollection_Array2<double>? anIncWeights = MakeW(aNb);
                    NCollection_Array1<double> anIncKnots = new(1, aNbK);
                    NCollection_Array1<int> anIncMults = new(1, aNbK);
                    global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.IncreaseDegree(aUDir, aDeg, aNewDeg, aPer, s.Poles, s.Weights, aKnots, aMults, anIncPoles,
                                            anIncWeights, anIncKnots, anIncMults);
                    Check(anIncPoles, anIncWeights, anIncKnots, anIncMults, aNewDeg, aPer, $"IncreaseDegree {aUDir}");
                }
                else
                {
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.PrepareUnperiodize(aDeg, aMults, out int aNbK, out int aNbP);
                    NCollection_Array2<Point3D> anUnPoles = Make(aNbP);
                    NCollection_Array2<double>? anUnWeights = MakeW(aNbP);
                    NCollection_Array1<double> anUnKnots = new(1, aNbK);
                    NCollection_Array1<int> anUnMults = new(1, aNbK);
                    global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.Unperiodize(aUDir, aDeg, aMults, aKnots, s.Poles, s.Weights, anUnMults, anUnKnots, anUnPoles,
                                         anUnWeights);
                    Check(anUnPoles, anUnWeights, anUnKnots, anUnMults, aDeg, false, $"Unperiodize {aUDir}");
                }
            }
        }
    }

    [Fact]
    public void Iso_MatchesSurface()
    {
        foreach (Surface s in Surfaces())
        {
            foreach ((double u, double v) in Params(s, 3))
            {
                global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.D0(u, v, 0, 0, s.Poles, s.Weights, s.UKnots, s.VKnots, s.UMults, s.VMults, s.UDegree,
                            s.VDegree, s.Rational, s.Rational, s.UPeriodic, s.VPeriodic, out Point3D P);

                // U iso: a curve in V
                NCollection_Array1<Point3D> aCPoles = new(1, s.Poles.RowLength());
                NCollection_Array1<double> aCWeights = new(1, s.Poles.RowLength());
                global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.Iso(u, true, s.Poles, s.Weights, s.UKnots, s.UMults, s.UDegree, s.UPeriodic, aCPoles, aCWeights);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(v, 0, s.VDegree, s.VPeriodic, aCPoles, s.Rational ? aCWeights : null, s.VKnots, s.VMults, out Point3D Q);
                AssertNear(Q.XYZ(), P.XYZ(), 1e-11, "UIso");

                // V iso: a curve in U
                aCPoles = new(1, s.Poles.ColLength());
                aCWeights = new(1, s.Poles.ColLength());
                global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.Iso(v, false, s.Poles, s.Weights, s.VKnots, s.VMults, s.VDegree, s.VPeriodic, aCPoles, aCWeights);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(u, 0, s.UDegree, s.UPeriodic, aCPoles, s.Rational ? aCWeights : null, s.UKnots, s.UMults, out Q);
                AssertNear(Q.XYZ(), P.XYZ(), 1e-11, "VIso");
                if (!s.Rational)
                {
                    Assert.All(aCWeights.AsSpan().ToArray(), w => Assert.Equal(1.0, w));
                }
            }
        }
    }

    [Fact]
    public void Reverse_ReversesTheParametrisation()
    {
        foreach (Surface s in Surfaces())
        {
            if (s.UPeriodic || s.VPeriodic)
            {
                continue;
            }
            NCollection_Array2<Point3D> aPoles = new(s.Poles);
            NCollection_Array2<double>? aWeights = s.Weights == null ? null : new NCollection_Array2<double>(s.Weights);
            global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.Reverse(aPoles, aPoles.UpperRow(), true);
            if (aWeights != null)
            {
                global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.Reverse(aWeights, aWeights.UpperRow(), true);
            }
            NCollection_Array1<double> aKnots = new(s.UKnots);
            NCollection_Array1<int> aMults = new(s.UMults);
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Reverse(aKnots);
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Reverse(aMults);
            double aSum = s.UKnots.First() + s.UKnots.Last();
            foreach ((double u, double v) in Params(s, 3))
            {
                global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.D0(u, v, 0, 0, s.Poles, s.Weights, s.UKnots, s.VKnots, s.UMults, s.VMults, s.UDegree,
                            s.VDegree, s.Rational, s.Rational, false, false, out Point3D P);
                global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.D0(aSum - u, v, 0, 0, aPoles, aWeights, aKnots, s.VKnots, aMults, s.VMults, s.UDegree,
                            s.VDegree, s.Rational, s.Rational, false, false, out Point3D Q);
                AssertNear(Q.XYZ(), P.XYZ(), 1e-11, "Reverse");
            }
        }
    }

    [Fact]
    public void Interpolate_RecoversThePoles()
    {
        foreach (Surface s in Surfaces())
        {
            if (s.UPeriodic || s.VPeriodic)
            {
                continue;
            }
            int aNbU = s.Poles.ColLength(), aNbV = s.Poles.RowLength();
            NCollection_Array1<double> aUParams = new(1, aNbU), aVParams = new(1, aNbV);
            global::Kerf.Engine.Geometry.Splines.BSplCLib.BuildSchoenbergPoints(s.UDegree, s.UFlat, aUParams);
            global::Kerf.Engine.Geometry.Splines.BSplCLib.BuildSchoenbergPoints(s.VDegree, s.VFlat, aVParams);
            NCollection_Array2<Point3D> aData = new(1, aNbU, 1, aNbV);
            NCollection_Array2<double> aWData = new(1, aNbU, 1, aNbV);
            for (int i = 1; i <= aNbU; i++)
            {
                for (int j = 1; j <= aNbV; j++)
                {
                    global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.HomogeneousD0(aUParams[i], aVParams[j], 0, 0, s.Poles, s.Weights, s.UKnots, s.VKnots,
                                           s.UMults, s.VMults, s.UDegree, s.VDegree, s.Rational, s.Rational, false,
                                           false, out aWData[i, j], out aData[i, j]);
                }
            }
            int anInversion;
            if (s.Rational)
            {
                global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.Interpolate(s.UDegree, s.VDegree, s.UFlat, s.VFlat, aUParams, aVParams, aData, aWData, out anInversion);
            }
            else
            {
                global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.Interpolate(s.UDegree, s.VDegree, s.UFlat, s.VFlat, aUParams, aVParams, aData, out anInversion);
            }
            Assert.Equal(0, anInversion);
            for (int i = 1; i <= aNbU; i++)
            {
                for (int j = 1; j <= aNbV; j++)
                {
                    double w = s.Weights?[i, j] ?? 1.0;
                    AssertNear(aData[i, j].XYZ(), s.Poles[i, j].XYZ() * w, 1e-9, "Interpolate poles");
                    if (s.Rational)
                    {
                        Assert.Equal(w, aWData[i, j], 1e-9);
                    }
                }
            }
        }
    }

    private sealed class ConstantFunction(double theValue) : global::Kerf.Engine.Geometry.Splines.SurfaceEvaluatorFunction
    {
        public override void Evaluate(int theDerivativeRequest, double theUParameter, double theVParameter,
                                      out double theResult, out int theErrorCode)
        {
            theResult = theValue;
            theErrorCode = 0;
        }
    }

    [Fact]
    public void FunctionMultiply_ByConstant()
    {
        foreach (Surface s in Surfaces())
        {
            if (s.UPeriodic || s.VPeriodic || !s.Rational)
            {
                continue;
            }
            int aNbU = s.Poles.ColLength(), aNbV = s.Poles.RowLength();
            NCollection_Array2<Point3D> aNum = new(1, aNbU, 1, aNbV);
            NCollection_Array2<double> aDen = new(1, aNbU, 1, aNbV);
            global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.FunctionMultiply(new ConstantFunction(2.0), s.UDegree, s.VDegree, s.UKnots, s.VKnots, s.UMults,
                                      s.VMults, s.Poles, s.Weights, s.UFlat, s.VFlat, s.UDegree, s.VDegree, aNum, aDen,
                                      out int aStatus);
            Assert.Equal(0, aStatus);
            for (int i = 1; i <= aNbU; i++)
            {
                for (int j = 1; j <= aNbV; j++)
                {
                    AssertNear(aNum[i, j].XYZ(), s.Poles[i, j].XYZ() * (2.0 * s.Weights![i, j]), 1e-9, "FunctionMultiply");
                    Assert.Equal(2.0 * s.Weights[i, j], aDen[i, j], 1e-9);
                }
            }
        }
    }

    [Fact]
    public void MovePoint_MovesThePoint()
    {
        foreach (Surface s in Surfaces())
        {
            if (s.UPeriodic || s.VPeriodic)
            {
                continue;
            }
            int aNbU = s.Poles.ColLength(), aNbV = s.Poles.RowLength();
            NCollection_Array2<double> aWeights = s.Weights ?? global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.UnitWeights(aNbU, aNbV);
            foreach ((double u, double v) in Params(s, 3))
            {
                Vector3D aDispl = new(0.3, -0.2, 0.1);
                NCollection_Array2<Point3D> aNewPoles = new(1, aNbU, 1, aNbV);
                global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.MovePoint(u, v, aDispl, 1, aNbU, 1, aNbV, s.UDegree, s.VDegree, s.Rational, s.Poles, aWeights,
                                   s.UFlat, s.VFlat, out int aUF, out int aUL, out int aVF, out int aVL, aNewPoles);
                Assert.True(aUF >= 1 && aUL <= aNbU && aVF >= 1 && aVL <= aNbV);
                global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.D0(u, v, 0, 0, s.Poles, s.Weights, s.UKnots, s.VKnots, s.UMults, s.VMults, s.UDegree,
                            s.VDegree, s.Rational, s.Rational, false, false, out Point3D P);
                global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.D0(u, v, 0, 0, aNewPoles, s.Weights, s.UKnots, s.VKnots, s.UMults, s.VMults, s.UDegree,
                            s.VDegree, s.Rational, s.Rational, false, false, out Point3D Q);
                AssertNear(Q.XYZ(), P.XYZ() + aDispl.XYZ(), 1e-10, "MovePoint");
            }
        }
    }

    [Fact]
    public void Resolution_BoundsTheVariation()
    {
        foreach (Surface s in Surfaces())
        {
            global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.Resolution(s.Poles, s.Weights, s.UKnots, s.VKnots, s.UMults, s.VMults, s.UDegree, s.VDegree,
                                s.Rational, s.Rational, s.UPeriodic, s.VPeriodic, 1e-3, out double aUTol, out double aVTol);
            Assert.True(aUTol > 0 && aVTol > 0);
            foreach ((double u, double v) in Params(s, 3))
            {
                global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.D0(u, v, 0, 0, s.Poles, s.Weights, s.UKnots, s.VKnots, s.UMults, s.VMults, s.UDegree,
                            s.VDegree, s.Rational, s.Rational, s.UPeriodic, s.VPeriodic, out Point3D P);
                global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.D0(u + 0.5 * aUTol, v + 0.5 * aVTol, 0, 0, s.Poles, s.Weights, s.UKnots, s.VKnots, s.UMults,
                            s.VMults, s.UDegree, s.VDegree, s.Rational, s.Rational, s.UPeriodic, s.VPeriodic, out Point3D Q);
                Assert.True(P.Distance(Q) < 1e-3);
            }
        }
    }

    [Fact]
    public void IsRational_and_UnitWeights()
    {
        NCollection_Array2<double> aUnit = global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.UnitWeights(3, 4);
        Assert.Equal(1, aUnit.LowerRow());
        Assert.Equal(4, aUnit.UpperCol());
        Assert.False(global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.IsRational(aUnit, 1, 3, 1, 4));
        NCollection_Array2<double> aBig = global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.UnitWeights(100, 100);
        Assert.Equal(1.0, aBig[100, 100]);

        NCollection_Array2<double> aW = new(1, 3, 1, 3);
        aW.Init(2.0);
        Assert.False(global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.IsRational(aW, 1, 3, 1, 3));
        aW[2, 1] = 2.5;
        Assert.True(global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.IsRational(aW, 1, 3, 1, 3));
        Assert.Null(global::Kerf.Engine.Geometry.Splines.SurfaceBSplSLib.NoWeights());
    }
}
