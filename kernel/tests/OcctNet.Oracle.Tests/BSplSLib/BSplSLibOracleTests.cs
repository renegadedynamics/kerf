#if HAS_ORACLE
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Xunit.Abstractions;
using N = Build123d.Occt;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// BSplSLib against native OCCT, through the native Geom_BSplineSurface and Geom_BezierSurface, which call BSplSLib
/// with the inputs the tests mirror (see Geom_BSplineSurface_1.cxx): degrees 1 to 9 in each direction, every
/// combination of U and V periodicity, rational half the time (weights varying in both directions, along U only or
/// along V only, so the surface is rational in one direction or both).
/// </summary>
public class BSplSLibOracleTests(ITestOutputHelper output)
{
    private const int Cases = 300;
    private const int CasesBezier = 150;

    /// <summary>A random B-spline surface: what the port and the native surface share.</summary>
    private sealed class Surface
    {
        public readonly int UDegree, VDegree;
        public readonly bool UPeriodic, VPeriodic;
        public readonly bool Rational;
        public readonly NCollection_Array1<double> UKnots, VKnots;
        public readonly NCollection_Array1<int> UMults, VMults;
        public readonly NCollection_Array1<double> UFlatKnots, VFlatKnots;
        public readonly int NbUPoles, NbVPoles;
        public readonly NCollection_Array2<gp_Pnt> Poles;
        public readonly NCollection_Array2<double> AllWeights;

        public Surface(Oracle theOracle, int theMaxDegree = 9)
        {
            UDegree = theOracle.Int(1, theMaxDegree);
            VDegree = theOracle.Int(1, theMaxDegree);
            UPeriodic = theOracle.Int(0, 2) == 1;
            VPeriodic = theOracle.Int(0, 2) == 1;
            Rational = theOracle.Int(0, 1) == 1;
            theOracle.BSplineKnots(UDegree, UPeriodic, out UKnots, out UMults);
            theOracle.BSplineKnots(VDegree, VPeriodic, out VKnots, out VMults);
            NbUPoles = Occt.BSplCLib.NbPoles(UDegree, UPeriodic, UMults);
            NbVPoles = Occt.BSplCLib.NbPoles(VDegree, VPeriodic, VMults);
            UFlatKnots = new NCollection_Array1<double>(1, Occt.BSplCLib.KnotSequenceLength(UMults, UDegree, UPeriodic));
            Occt.BSplCLib.KnotSequence(UKnots, UMults, UDegree, UPeriodic, UFlatKnots);
            VFlatKnots = new NCollection_Array1<double>(1, Occt.BSplCLib.KnotSequenceLength(VMults, VDegree, VPeriodic));
            Occt.BSplCLib.KnotSequence(VKnots, VMults, VDegree, VPeriodic, VFlatKnots);

            Poles = new NCollection_Array2<gp_Pnt>(1, NbUPoles, 1, NbVPoles);
            AllWeights = new NCollection_Array2<double>(1, NbUPoles, 1, NbVPoles);
            NCollection_Array1<double> aUWeights = theOracle.Weights(NbUPoles, Rational);
            NCollection_Array1<double> aVWeights = theOracle.Weights(NbVPoles, Rational);
            int aMode = theOracle.Int(0, 2); // 0: both directions, 1: along U only, 2: along V only
            for (int i = 1; i <= NbUPoles; i++)
            {
                for (int j = 1; j <= NbVPoles; j++)
                {
                    Poles[i, j] = new gp_Pnt(theOracle.Real(0, 10), theOracle.Real(0, 10), theOracle.Real(0, 10));
                    AllWeights[i, j] = !Rational ? 1.0
                                       : aMode == 0 ? theOracle.Real(0.5, 1.5)
                                       : aMode == 1 ? aUWeights[i]
                                                    : aVWeights[j];
                }
            }
        }

        public int NbUKnots => UKnots.Length();

        public int NbVKnots => VKnots.Length();

        public N.Geom_BSplineSurface ToNative()
        {
            using N.TColgp_Array2OfPnt aPoles = Oracle.ToNative(Poles);
            using N.TColStd_Array1OfReal aUKnots = Oracle.ToNative(UKnots);
            using N.TColStd_Array1OfReal aVKnots = Oracle.ToNative(VKnots);
            using N.TColStd_Array1OfInteger aUMults = Oracle.ToNative(UMults);
            using N.TColStd_Array1OfInteger aVMults = Oracle.ToNative(VMults);
            if (!Rational)
            {
                return new N.Geom_BSplineSurface(aPoles, aUKnots, aVKnots, aUMults, aVMults, UDegree, VDegree, UPeriodic, VPeriodic);
            }

            using N.TColStd_Array2OfReal aWeights = Oracle.ToNative(AllWeights);
            return new N.Geom_BSplineSurface(aPoles, aWeights, aUKnots, aVKnots, aUMults, aVMults, UDegree, VDegree,
                                             UPeriodic, VPeriodic);
        }

        /// <summary>A parameter in the U domain, some outside the period of a periodic surface.</summary>
        public double U(int j, int n) => Param(UKnots, UPeriodic, j, n);

        public double V(int j, int n) => Param(VKnots, VPeriodic, j + 2, n);

        private static double Param(NCollection_Array1<double> theKnots, bool thePeriodic, int j, int n)
        {
            double aFirst = theKnots[theKnots.Lower()], aLast = theKnots[theKnots.Upper()];
            return aFirst + (aLast - aFirst) * (j % (n + 1)) / n + (thePeriodic && j % 5 == 3 ? (aLast - aFirst) * 1.3 : 0.0);
        }
    }

    /// <summary>The surface's state as Geom_BSplineSurface hands it to BSplSLib: rationality per direction and Weights().</summary>
    private readonly record struct Rationality(bool URat, bool VRat, NCollection_Array2<double>? Weights);

    private static Rationality RationalityOf(N.Geom_BSplineSurface theNative, Surface theSurface)
    {
        bool aURat = theNative.IsURational(), aVRat = theNative.IsVRational();
        return new Rationality(aURat, aVRat, aURat || aVRat ? theSurface.AllWeights : null);
    }

    private static void Locate(int theDegree, NCollection_Array1<double> theKnots, NCollection_Array1<int> theMults,
                               bool thePeriodic, double theU, out int theIndex, out double theNewU)
    {
        // As Geom_BSplineSurface::EvalD1/EvalD2: knots with multiplicities, then the flat index
        int anIndex = 0;
        Occt.BSplCLib.LocateParameter(theDegree, theKnots, theMults, theU, thePeriodic, ref anIndex, out theNewU);
        theIndex = Occt.BSplCLib.FlatIndex(theDegree, anIndex, theMults, thePeriodic);
    }

    [Fact]
    public void Surface_evaluation()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            var S = new Surface(anOracle);
            using N.Geom_BSplineSurface aNative = S.ToNative();
            Rationality R = RationalityOf(aNative, S);
            const int n = 5;
            for (int ju = 0; ju <= n; ju++)
            {
                for (int jv = 0; jv <= n; jv++)
                {
                    double aU = S.U(ju, n), aV = S.V(jv, n);

                    // D0: periodic normalization, then BSplSLib::D0 with index 0 on the flat knots
                    double aNU = aU, aNV = aV;
                    aNative.PeriodicNormalization(ref aNU, ref aNV);
                    Occt.BSplSLib.D0(aNU, aNV, 0, 0, S.Poles, R.Weights, S.UFlatKnots, S.VFlatKnots, null, null, S.UDegree,
                                     S.VDegree, R.URat, R.VRat, S.UPeriodic, S.VPeriodic, out gp_Pnt aP);
                    using (N.gp_Pnt aNP = aNative.EvalD0(aU, aV))
                    {
                        anOracle.Same(aNP, aP, "D0");
                    }

                    // D1, D2: located spans
                    Locate(S.UDegree, S.UKnots, S.UMults, S.UPeriodic, aU, out int anUIndex, out double aLU);
                    Locate(S.VDegree, S.VKnots, S.VMults, S.VPeriodic, aV, out int aVIndex, out double aLV);
                    Occt.BSplSLib.D1(aLU, aLV, anUIndex, aVIndex, S.Poles, R.Weights, S.UFlatKnots, S.VFlatKnots, null, null,
                                     S.UDegree, S.VDegree, R.URat, R.VRat, S.UPeriodic, S.VPeriodic,
                                     out gp_Pnt aP1, out gp_Vec aD1U, out gp_Vec aD1V);
                    using (var aNP1 = new N.gp_Pnt(0, 0, 0))
                    using (var aND1U = new N.gp_Vec(0, 0, 0))
                    using (var aND1V = new N.gp_Vec(0, 0, 0))
                    {
                        aNative.D1(aU, aV, aNP1, aND1U, aND1V);
                        anOracle.Same(aNP1, aP1, "D1 P");
                        anOracle.Same(aND1U, aD1U, "D1 U");
                        anOracle.Same(aND1V, aD1V, "D1 V");
                    }

                    Occt.BSplSLib.D2(aLU, aLV, anUIndex, aVIndex, S.Poles, R.Weights, S.UFlatKnots, S.VFlatKnots, null, null,
                                     S.UDegree, S.VDegree, R.URat, R.VRat, S.UPeriodic, S.VPeriodic,
                                     out gp_Pnt aP2, out gp_Vec aD2U1, out gp_Vec aD2V1, out gp_Vec aD2UU, out gp_Vec aD2VV,
                                     out gp_Vec aD2UV);
                    using (var aNP2 = new N.gp_Pnt(0, 0, 0))
                    using (var aNU1 = new N.gp_Vec(0, 0, 0))
                    using (var aNV1 = new N.gp_Vec(0, 0, 0))
                    using (var aNUU = new N.gp_Vec(0, 0, 0))
                    using (var aNVV = new N.gp_Vec(0, 0, 0))
                    using (var aNUV = new N.gp_Vec(0, 0, 0))
                    {
                        aNative.D2(aU, aV, aNP2, aNU1, aNV1, aNUU, aNVV, aNUV);
                        anOracle.Same(aNP2, aP2, "D2 P");
                        anOracle.Same(aNU1, aD2U1, "D2 U");
                        anOracle.Same(aNV1, aD2V1, "D2 V");
                        anOracle.Same(aNUU, aD2UU, "D2 UU");
                        anOracle.Same(aNVV, aD2VV, "D2 VV");
                        anOracle.Same(aNUV, aD2UV, "D2 UV");
                    }

                    // D3 and DN: the parameters as given, index 0
                    Occt.BSplSLib.D3(aU, aV, 0, 0, S.Poles, R.Weights, S.UFlatKnots, S.VFlatKnots, null, null,
                                     S.UDegree, S.VDegree, R.URat, R.VRat, S.UPeriodic, S.VPeriodic,
                                     out gp_Pnt aP3, out gp_Vec a3U, out gp_Vec a3V, out gp_Vec a3UU, out gp_Vec a3VV,
                                     out gp_Vec a3UV, out gp_Vec a3UUU, out gp_Vec a3VVV, out gp_Vec a3UUV, out gp_Vec a3UVV);
                    using (var aNP3 = new N.gp_Pnt(0, 0, 0))
                    using (var aN3U = new N.gp_Vec(0, 0, 0))
                    using (var aN3V = new N.gp_Vec(0, 0, 0))
                    using (var aN3UU = new N.gp_Vec(0, 0, 0))
                    using (var aN3VV = new N.gp_Vec(0, 0, 0))
                    using (var aN3UV = new N.gp_Vec(0, 0, 0))
                    using (var aN3UUU = new N.gp_Vec(0, 0, 0))
                    using (var aN3VVV = new N.gp_Vec(0, 0, 0))
                    using (var aN3UUV = new N.gp_Vec(0, 0, 0))
                    using (var aN3UVV = new N.gp_Vec(0, 0, 0))
                    {
                        aNative.D3(aU, aV, aNP3, aN3U, aN3V, aN3UU, aN3VV, aN3UV, aN3UUU, aN3VVV, aN3UUV, aN3UVV);
                        anOracle.Same(aNP3, aP3, "D3 P");
                        anOracle.Same(aN3U, a3U, "D3 U");
                        anOracle.Same(aN3V, a3V, "D3 V");
                        anOracle.Same(aN3UU, a3UU, "D3 UU");
                        anOracle.Same(aN3VV, a3VV, "D3 VV");
                        anOracle.Same(aN3UV, a3UV, "D3 UV");
                        anOracle.Same(aN3UUU, a3UUU, "D3 UUU");
                        anOracle.Same(aN3VVV, a3VVV, "D3 VVV");
                        anOracle.Same(aN3UUV, a3UUV, "D3 UUV");
                        anOracle.Same(aN3UVV, a3UVV, "D3 UVV");
                    }

                    for (int nu = 0; nu <= 3; nu++)
                    {
                        for (int nv = 0; nv <= 3; nv++)
                        {
                            if (nu + nv == 0)
                            {
                                continue;
                            }

                            Occt.BSplSLib.DN(aU, aV, nu, nv, 0, 0, S.Poles, R.Weights, S.UFlatKnots, S.VFlatKnots, null, null,
                                             S.UDegree, S.VDegree, R.URat, R.VRat, S.UPeriodic, S.VPeriodic, out gp_Vec aDN);
                            using N.gp_Vec aNDN = aNative.EvalDN(aU, aV, nu, nv);
                            anOracle.Same(aNDN, aDN, $"DN({nu},{nv})");
                        }
                    }
                }
            }
        }

        anOracle.Report("BSplSLib D0, D1, D2, D3, DN");
    }

    [Fact]
    public void Surface_local_evaluation()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            var S = new Surface(anOracle);
            using N.Geom_BSplineSurface aNative = S.ToNative();
            Rationality R = RationalityOf(aNative, S);
            const int n = 5;
            for (int ju = 0; ju <= n; ju++)
            {
                for (int jv = 0; jv <= n; jv++)
                {
                    double aU = S.U(ju, n), aV = S.V(jv, n);
                    int aUK1 = anOracle.Int(1, S.NbUKnots - 1), aVK1 = anOracle.Int(1, S.NbVKnots - 1);
                    int aUK2 = aUK1 + 1, aVK2 = aVK1 + 1;

                    // As Geom_BSplineSurface::LocalD*: flat knots between two knots, then the flat index
                    Occt.BSplCLib.LocateParameter(S.UDegree, S.UFlatKnots, aU, S.UPeriodic, aUK1, aUK2, out int anUIndex, out double aLU);
                    anUIndex = Occt.BSplCLib.FlatIndex(S.UDegree, anUIndex, S.UMults, S.UPeriodic);
                    Occt.BSplCLib.LocateParameter(S.VDegree, S.VFlatKnots, aV, S.VPeriodic, aVK1, aVK2, out int aVIndex, out double aLV);
                    aVIndex = Occt.BSplCLib.FlatIndex(S.VDegree, aVIndex, S.VMults, S.VPeriodic);

                    Occt.BSplSLib.D0(aLU, aLV, anUIndex, aVIndex, S.Poles, R.Weights, S.UFlatKnots, S.VFlatKnots, null, null,
                                     S.UDegree, S.VDegree, R.URat, R.VRat, S.UPeriodic, S.VPeriodic, out gp_Pnt aP);
                    using (var aNP = new N.gp_Pnt(0, 0, 0))
                    {
                        aNative.LocalD0(aU, aV, aUK1, aUK2, aVK1, aVK2, aNP);
                        anOracle.Same(aNP, aP, "LocalD0");
                    }

                    Occt.BSplSLib.D1(aLU, aLV, anUIndex, aVIndex, S.Poles, R.Weights, S.UFlatKnots, S.VFlatKnots, null, null,
                                     S.UDegree, S.VDegree, R.URat, R.VRat, S.UPeriodic, S.VPeriodic,
                                     out gp_Pnt aP1, out gp_Vec a1U, out gp_Vec a1V);
                    using (var aNP = new N.gp_Pnt(0, 0, 0))
                    using (var aNU = new N.gp_Vec(0, 0, 0))
                    using (var aNV = new N.gp_Vec(0, 0, 0))
                    {
                        aNative.LocalD1(aU, aV, aUK1, aUK2, aVK1, aVK2, aNP, aNU, aNV);
                        anOracle.Same(aNP, aP1, "LocalD1 P");
                        anOracle.Same(aNU, a1U, "LocalD1 U");
                        anOracle.Same(aNV, a1V, "LocalD1 V");
                    }

                    Occt.BSplSLib.D2(aLU, aLV, anUIndex, aVIndex, S.Poles, R.Weights, S.UFlatKnots, S.VFlatKnots, null, null,
                                     S.UDegree, S.VDegree, R.URat, R.VRat, S.UPeriodic, S.VPeriodic,
                                     out gp_Pnt aP2, out gp_Vec a2U, out gp_Vec a2V, out gp_Vec a2UU, out gp_Vec a2VV, out gp_Vec a2UV);
                    using (var aNP = new N.gp_Pnt(0, 0, 0))
                    using (var aNU = new N.gp_Vec(0, 0, 0))
                    using (var aNV = new N.gp_Vec(0, 0, 0))
                    using (var aNUU = new N.gp_Vec(0, 0, 0))
                    using (var aNVV = new N.gp_Vec(0, 0, 0))
                    using (var aNUV = new N.gp_Vec(0, 0, 0))
                    {
                        aNative.LocalD2(aU, aV, aUK1, aUK2, aVK1, aVK2, aNP, aNU, aNV, aNUU, aNVV, aNUV);
                        anOracle.Same(aNP, aP2, "LocalD2 P");
                        anOracle.Same(aNU, a2U, "LocalD2 U");
                        anOracle.Same(aNV, a2V, "LocalD2 V");
                        anOracle.Same(aNUU, a2UU, "LocalD2 UU");
                        anOracle.Same(aNVV, a2VV, "LocalD2 VV");
                        anOracle.Same(aNUV, a2UV, "LocalD2 UV");
                    }

                    Occt.BSplSLib.D3(aLU, aLV, anUIndex, aVIndex, S.Poles, R.Weights, S.UFlatKnots, S.VFlatKnots, null, null,
                                     S.UDegree, S.VDegree, R.URat, R.VRat, S.UPeriodic, S.VPeriodic,
                                     out gp_Pnt aP3, out gp_Vec a3U, out gp_Vec a3V, out gp_Vec a3UU, out gp_Vec a3VV,
                                     out gp_Vec a3UV, out gp_Vec a3UUU, out gp_Vec a3VVV, out gp_Vec a3UUV, out gp_Vec a3UVV);
                    using (var aNP = new N.gp_Pnt(0, 0, 0))
                    using (var aNU = new N.gp_Vec(0, 0, 0))
                    using (var aNV = new N.gp_Vec(0, 0, 0))
                    using (var aNUU = new N.gp_Vec(0, 0, 0))
                    using (var aNVV = new N.gp_Vec(0, 0, 0))
                    using (var aNUV = new N.gp_Vec(0, 0, 0))
                    using (var aNUUU = new N.gp_Vec(0, 0, 0))
                    using (var aNVVV = new N.gp_Vec(0, 0, 0))
                    using (var aNUUV = new N.gp_Vec(0, 0, 0))
                    using (var aNUVV = new N.gp_Vec(0, 0, 0))
                    {
                        aNative.LocalD3(aU, aV, aUK1, aUK2, aVK1, aVK2, aNP, aNU, aNV, aNUU, aNVV, aNUV, aNUUU, aNVVV, aNUUV, aNUVV);
                        anOracle.Same(aNP, aP3, "LocalD3 P");
                        anOracle.Same(aNU, a3U, "LocalD3 U");
                        anOracle.Same(aNV, a3V, "LocalD3 V");
                        anOracle.Same(aNUU, a3UU, "LocalD3 UU");
                        anOracle.Same(aNVV, a3VV, "LocalD3 VV");
                        anOracle.Same(aNUV, a3UV, "LocalD3 UV");
                        anOracle.Same(aNUUU, a3UUU, "LocalD3 UUU");
                        anOracle.Same(aNVVV, a3VVV, "LocalD3 VVV");
                        anOracle.Same(aNUUV, a3UUV, "LocalD3 UUV");
                        anOracle.Same(aNUVV, a3UVV, "LocalD3 UVV");
                    }

                    int aNu = anOracle.Int(0, 3), aNv = anOracle.Int(aNu == 0 ? 1 : 0, 3);
                    Occt.BSplSLib.DN(aLU, aLV, aNu, aNv, anUIndex, aVIndex, S.Poles, R.Weights, S.UFlatKnots, S.VFlatKnots,
                                     null, null, S.UDegree, S.VDegree, R.URat, R.VRat, S.UPeriodic, S.VPeriodic, out gp_Vec aDN);
                    using (N.gp_Vec aNDN = aNative.LocalDN(aU, aV, aUK1, aUK2, aVK1, aVK2, aNu, aNv))
                    {
                        anOracle.Same(aNDN, aDN, $"LocalDN({aNu},{aNv})");
                    }
                }
            }
        }

        anOracle.Report("BSplSLib LocalD0-D3, LocalDN");
    }

    [Fact]
    public void Surface_resolution()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            var S = new Surface(anOracle);
            using N.Geom_BSplineSurface aNative = S.ToNative();
            Rationality R = RationalityOf(aNative, S);
            double aNU = 0.0, aNV = 0.0;
            aNative.Resolution(1.0, ref aNU, ref aNV);
            Occt.BSplSLib.Resolution(S.Poles, R.Weights, S.UKnots, S.VKnots, S.UMults, S.VMults, S.UDegree, S.VDegree,
                                     R.URat, R.VRat, S.UPeriodic, S.VPeriodic, 1.0, out double aU, out double aV);
            anOracle.SameRelative(aNU, aU, "Resolution U");
            anOracle.SameRelative(aNV, aV, "Resolution V");
        }

        anOracle.Report("BSplSLib Resolution");
    }

    /// <summary>Compares the native surface's poles (and weights when rational) with the port's.</summary>
    private static void SamePoles(Oracle theOracle, N.Geom_BSplineSurface theNative, NCollection_Array2<gp_Pnt> thePoles,
                                  NCollection_Array2<double>? theWeights, string theWhat)
    {
        theOracle.Same(theNative.NbUPoles(), thePoles.ColLength(), theWhat + " NbUPoles");
        theOracle.Same(theNative.NbVPoles(), thePoles.RowLength(), theWhat + " NbVPoles");
        for (int i = 1; i <= thePoles.ColLength(); i++)
        {
            for (int j = 1; j <= thePoles.RowLength(); j++)
            {
                using N.gp_Pnt aNP = theNative.Pole(i, j);
                theOracle.Same(aNP, thePoles[i, j], theWhat + " pole");
                if (theWeights != null)
                {
                    theOracle.Same(theNative.Weight(i, j), theWeights[i, j], theWhat + " weight");
                }
            }
        }
    }

    private static void SameKnots(Oracle theOracle, N.Geom_BSplineSurface theNative, bool theU,
                                  NCollection_Array1<double> theKnots, NCollection_Array1<int> theMults, string theWhat)
    {
        theOracle.Same(theU ? theNative.NbUKnots() : theNative.NbVKnots(), theKnots.Length(), theWhat + " NbKnots");
        for (int k = 1; k <= theKnots.Length(); k++)
        {
            theOracle.Same(theU ? theNative.UKnot(k) : theNative.VKnot(k), theKnots[k], theWhat + " knot");
            theOracle.Same(theU ? theNative.UMultiplicity(k) : theNative.VMultiplicity(k), theMults[k], theWhat + " mult");
        }
    }

    [Fact]
    public void Surface_knot_insertion_and_removal()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            var S = new Surface(anOracle);
            using N.Geom_BSplineSurface aProbe = S.ToNative();
            Rationality R = RationalityOf(aProbe, S);
            bool isRational = R.Weights != null;
            for (int aDir = 0; aDir < 2; aDir++)
            {
                bool isU = aDir == 0;
                int aDegree = isU ? S.UDegree : S.VDegree;
                bool isPeriodic = isU ? S.UPeriodic : S.VPeriodic;
                NCollection_Array1<double> aKnots = isU ? S.UKnots : S.VKnots;
                NCollection_Array1<int> aMults = isU ? S.UMults : S.VMults;
                int aNbKnots = aKnots.Length();
                double aFirst = aKnots[1], aLast = aKnots[aNbKnots];

                // InsertUKnots/InsertVKnots: strictly increasing knots (see the curve tests), Add = true
                int aNbAdd = anOracle.Int(1, 3);
                var anAddKnots = new NCollection_Array1<double>(1, aNbAdd);
                var anAddMults = new NCollection_Array1<int>(1, aNbAdd);
                double anAdd = aFirst;
                for (int k = 1; k <= aNbAdd; k++)
                {
                    anAdd += (aLast - anAdd) * anOracle.Real(0, 0.6);
                    if (anOracle.Int(0, 3) == 0)
                    {
                        anAdd = Math.Max(anAdd, aKnots[anOracle.Int(1, aNbKnots)]);
                    }

                    anAddKnots[k] = anAdd;
                    anAddMults[k] = anOracle.Int(1, aDegree);
                }

                bool isIncreasing = true;
                for (int k = 2; k <= aNbAdd; k++)
                {
                    isIncreasing &= anAddKnots[k] > anAddKnots[k - 1];
                }

                int aNbPolesDir = isU ? S.NbUPoles : S.NbVPoles;
                if (isIncreasing
                    && Occt.BSplCLib.PrepareInsertKnots(aDegree, isPeriodic, aKnots, aMults, anAddKnots, anAddMults,
                                                        out int aNbPoles, out int aNbNewKnots, 0.0, true)
                    && aNbPoles != aNbPolesDir)
                {
                    int aRows = isU ? aNbPoles : S.NbUPoles, aCols = isU ? S.NbVPoles : aNbPoles;
                    var aNewPoles = new NCollection_Array2<gp_Pnt>(1, aRows, 1, aCols);
                    var aNewWeights = isRational ? new NCollection_Array2<double>(1, aRows, 1, aCols) : null;
                    var aNewKnots = new NCollection_Array1<double>(1, aNbNewKnots);
                    var aNewMults = new NCollection_Array1<int>(1, aNbNewKnots);
                    Occt.BSplSLib.InsertKnots(isU, aDegree, isPeriodic, S.Poles, R.Weights, aKnots, aMults, anAddKnots, anAddMults,
                                              aNewPoles, aNewWeights, aNewKnots, aNewMults, 0.0, true);

                    using N.Geom_BSplineSurface aNative = S.ToNative();
                    using N.TColStd_Array1OfReal aNKnots = Oracle.ToNative(anAddKnots);
                    using N.TColStd_Array1OfInteger aNMults = Oracle.ToNative(anAddMults);
                    if (isU)
                    {
                        aNative.InsertUKnots(aNKnots, aNMults, 0.0, true);
                    }
                    else
                    {
                        aNative.InsertVKnots(aNKnots, aNMults, 0.0, true);
                    }

                    SamePoles(anOracle, aNative, aNewPoles, aNewWeights, "InsertKnots");
                    SameKnots(anOracle, aNative, isU, aNewKnots, aNewMults, "InsertKnots");
                }

                // RemoveUKnot/RemoveVKnot of an interior knot, with a tolerance that fails or one that always succeeds
                if (aNbKnots > 2 && !isPeriodic)
                {
                    int anIndex = anOracle.Int(2, aNbKnots - 1);
                    int aMult = anOracle.Int(0, aMults[anIndex] - 1);
                    int aStep = aMults[anIndex] - aMult;
                    int aNbKnotsAfter = aNbKnots - (aMult == 0 ? 1 : 0);
                    int aRows = isU ? S.NbUPoles - aStep : S.NbUPoles, aCols = isU ? S.NbVPoles : S.NbVPoles - aStep;
                    var aNewPoles = new NCollection_Array2<gp_Pnt>(1, aRows, 1, aCols);
                    var aNewWeights = isRational ? new NCollection_Array2<double>(1, aRows, 1, aCols) : null;
                    var aNewKnots = new NCollection_Array1<double>(1, aNbKnotsAfter);
                    var aNewMults = new NCollection_Array1<int>(1, aNbKnotsAfter);
                    double aTolerance = anOracle.Int(0, 1) == 0 ? 1e-7 : 1e3;
                    bool isRemoved = Occt.BSplSLib.RemoveKnot(isU, anIndex, aMult, aDegree, isPeriodic, S.Poles, R.Weights,
                                                              aKnots, aMults, aNewPoles, aNewWeights, aNewKnots, aNewMults,
                                                              aTolerance);

                    using N.Geom_BSplineSurface aNative = S.ToNative();
                    bool isNativeRemoved = isU ? aNative.RemoveUKnot(anIndex, aMult, aTolerance)
                                               : aNative.RemoveVKnot(anIndex, aMult, aTolerance);
                    anOracle.Same(isNativeRemoved, isRemoved, "RemoveKnot result");
                    if (isRemoved && isNativeRemoved)
                    {
                        SamePoles(anOracle, aNative, aNewPoles, aNewWeights, "RemoveKnot");
                        SameKnots(anOracle, aNative, isU, aNewKnots, aNewMults, "RemoveKnot");
                    }
                }
            }
        }

        anOracle.Report("BSplSLib InsertKnots, RemoveKnot");
    }

    [Fact]
    public void Surface_degree_elevation_and_unperiodize()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            var S = new Surface(anOracle);
            using N.Geom_BSplineSurface aProbe = S.ToNative();
            Rationality R = RationalityOf(aProbe, S);
            bool isRational = R.Weights != null;
            for (int aDir = 0; aDir < 2; aDir++)
            {
                bool isU = aDir == 0;
                int aDegree = isU ? S.UDegree : S.VDegree;
                bool isPeriodic = isU ? S.UPeriodic : S.VPeriodic;
                NCollection_Array1<double> aKnots = isU ? S.UKnots : S.VKnots;
                NCollection_Array1<int> aMults = isU ? S.UMults : S.VMults;

                // IncreaseDegree in one direction (non-periodic: periodic degree elevation is broken in OCCT's BSplCLib)
                if (!isPeriodic)
                {
                    int aNewDegree = aDegree + anOracle.Int(1, 3);
                    int aFromK1 = Occt.BSplCLib.FirstUKnotIndex(aDegree, aMults), aToK2 = Occt.BSplCLib.LastUKnotIndex(aDegree, aMults);
                    int aStep = aNewDegree - aDegree;
                    int aRows = isU ? S.NbUPoles + aStep * (aToK2 - aFromK1) : S.NbUPoles;
                    int aCols = isU ? S.NbVPoles : S.NbVPoles + aStep * (aToK2 - aFromK1);
                    int aNbNewKnots = Occt.BSplCLib.IncreaseDegreeCountKnots(aDegree, aNewDegree, isPeriodic, aMults);
                    var aNewPoles = new NCollection_Array2<gp_Pnt>(1, aRows, 1, aCols);
                    var aNewWeights = isRational ? new NCollection_Array2<double>(1, aRows, 1, aCols) : null;
                    var aNewKnots = new NCollection_Array1<double>(1, aNbNewKnots);
                    var aNewMults = new NCollection_Array1<int>(1, aNbNewKnots);
                    Occt.BSplSLib.IncreaseDegree(isU, aDegree, aNewDegree, isPeriodic, S.Poles, R.Weights, aKnots, aMults,
                                                 aNewPoles, aNewWeights, aNewKnots, aNewMults);

                    using N.Geom_BSplineSurface aNative = S.ToNative();
                    aNative.IncreaseDegree(isU ? aNewDegree : S.UDegree, isU ? S.VDegree : aNewDegree);
                    SamePoles(anOracle, aNative, aNewPoles, aNewWeights, "IncreaseDegree");
                    SameKnots(anOracle, aNative, isU, aNewKnots, aNewMults, "IncreaseDegree");
                }
                else
                {
                    // SetUNotPeriodic/SetVNotPeriodic
                    Occt.BSplCLib.PrepareUnperiodize(aDegree, aMults, out int aNbKnots, out int aNbPoles);
                    int aRows = isU ? aNbPoles : S.NbUPoles, aCols = isU ? S.NbVPoles : aNbPoles;
                    var aNewPoles = new NCollection_Array2<gp_Pnt>(1, aRows, 1, aCols);
                    NCollection_Array2<double>? aNewWeights = null;
                    if (isRational)
                    {
                        aNewWeights = new NCollection_Array2<double>(1, aRows, 1, aCols);
                        aNewWeights.Init(0.0);
                    }

                    var aNewKnots = new NCollection_Array1<double>(1, aNbKnots);
                    var aNewMults = new NCollection_Array1<int>(1, aNbKnots);
                    Occt.BSplSLib.Unperiodize(isU, aDegree, aMults, aKnots, S.Poles, R.Weights, aNewMults, aNewKnots, aNewPoles,
                                              aNewWeights);

                    using N.Geom_BSplineSurface aNative = S.ToNative();
                    if (isU)
                    {
                        aNative.SetUNotPeriodic();
                    }
                    else
                    {
                        aNative.SetVNotPeriodic();
                    }

                    SamePoles(anOracle, aNative, aNewPoles, aNewWeights, "Unperiodize");
                    SameKnots(anOracle, aNative, isU, aNewKnots, aNewMults, "Unperiodize");
                }
            }
        }

        anOracle.Report("BSplSLib IncreaseDegree, Unperiodize");
    }

    [Fact]
    public void Surface_move_point()
    {
        var anOracle = new Oracle(output);
        int aNbThrows = 0, aNbThrowsPeriodic = 0, aNbOutside = 0;
        for (int i = 0; i < Cases; i++)
        {
            var S = new Surface(anOracle);
            using N.Geom_BSplineSurface aNative = S.ToNative();
            Rationality R = RationalityOf(aNative, S);
            double aU = S.UKnots[1] + (S.UKnots[S.NbUKnots] - S.UKnots[1]) * anOracle.Real(0, 1);
            double aV = S.VKnots[1] + (S.VKnots[S.NbVKnots] - S.VKnots[1]) * anOracle.Real(0, 1);
            // All the poles half the time, otherwise a random window of poles allowed to move
            bool isAll = anOracle.Int(0, 1) == 0;
            int aUI1 = isAll ? 1 : anOracle.Int(1, S.NbUPoles), aUI2 = isAll ? S.NbUPoles : anOracle.Int(aUI1, S.NbUPoles);
            int aVI1 = isAll ? 1 : anOracle.Int(1, S.NbVPoles), aVI2 = isAll ? S.NbVPoles : anOracle.Int(aVI1, S.NbVPoles);

            // Geom_BSplineSurface::MovePoint: the displacement from its own D0, the weights (ones if non-rational),
            // the flat knots
            using N.gp_Pnt aNP0 = aNative.EvalD0(aU, aV);
            using var aNTarget = new N.gp_Pnt(aNP0.X() + 1, aNP0.Y() - 0.5, aNP0.Z() + 0.25);
            int aNUF = 0, aNUL = 0, aNVF = 0, aNVL = 0;
            bool isNativeThrown = false, isPortThrown = false;
            try
            {
                aNative.MovePoint(aU, aV, aNTarget, aUI1, aUI2, aVI1, aVI2, ref aNUF, ref aNUL, ref aNVF, ref aNVL);
            }
            catch (Exception)
            {
                isNativeThrown = true;
            }

            var aNewPoles = new NCollection_Array2<gp_Pnt>(1, S.NbUPoles, 1, S.NbVPoles);
            int aUF = 0, aUL = 0, aVF = 0, aVL = 0;
            try
            {
                var aDispl = new gp_Vec(Oracle.FromNative(aNP0), Oracle.FromNative(aNTarget));
                Occt.BSplSLib.MovePoint(aU, aV, aDispl, aUI1, aUI2, aVI1, aVI2, S.UDegree, S.VDegree, R.URat || R.VRat, S.Poles,
                                        S.AllWeights, S.UFlatKnots, S.VFlatKnots, out aUF, out aUL, out aVF, out aVL, aNewPoles);
            }
            catch (Standard_OutOfRange) when (!isNativeThrown)
            {
                // The index window misses the poles that act at (U, V): OCCT then reads its basis matrices out of
                // range unchecked (No_Exception, occt-issues.md), where the port throws. Not comparable.
                aNbOutside++;
                continue;
            }
            catch (Exception)
            {
                isPortThrown = true;
            }

            anOracle.Same(isNativeThrown, isPortThrown, "MovePoint throws");
            if (isNativeThrown)
            {
                aNbThrows++;
                aNbThrowsPeriodic += S.UPeriodic || S.VPeriodic ? 1 : 0;
                continue;
            }

            anOracle.Same(aNUF, aUF, "MovePoint UFirst");
            anOracle.Same(aNUL, aUL, "MovePoint ULast");
            anOracle.Same(aNVF, aVF, "MovePoint VFirst");
            anOracle.Same(aNVL, aVL, "MovePoint VLast");
            if (aUF != 0)
            {
                SamePoles(anOracle, aNative, aNewPoles, null, "MovePoint");
            }
        }

        // Both throw (out of range) for periodic surfaces whose poles at (U, V) wrap around (MovePoint doesn't wrap
        // pole indices) and for most windows that miss the poles acting at (U, V)
        output.WriteLine($"MovePoint: native and port threw for the same {aNbThrows} surfaces ({aNbThrowsPeriodic} periodic); for {aNbOutside} more, "
                         + "the window missed the point's poles and only the port threw (OCCT reads out of range there)");
        anOracle.Report("BSplSLib MovePoint");
    }

    [Fact]
    public void Bezier_surface_evaluation()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < CasesBezier; i++)
        {
            int aUDegree = anOracle.Int(1, 12), aVDegree = anOracle.Int(1, 12);
            bool isRational = anOracle.Int(0, 1) == 1;
            var aPoles = new NCollection_Array2<gp_Pnt>(1, aUDegree + 1, 1, aVDegree + 1);
            var aWeights = new NCollection_Array2<double>(1, aUDegree + 1, 1, aVDegree + 1);
            for (int r = 1; r <= aUDegree + 1; r++)
            {
                for (int c = 1; c <= aVDegree + 1; c++)
                {
                    aPoles[r, c] = new gp_Pnt(anOracle.Real(0, 1), anOracle.Real(0, 1), anOracle.Real(0, 1));
                    aWeights[r, c] = isRational ? anOracle.Real(0.5, 1.5) : 1.0;
                }
            }

            using N.TColgp_Array2OfPnt aNPoles = Oracle.ToNative(aPoles);
            using N.TColStd_Array2OfReal aNWeights = Oracle.ToNative(aWeights);
            using N.Geom_BezierSurface aNative = isRational ? new N.Geom_BezierSurface(aNPoles, aNWeights) : new N.Geom_BezierSurface(aNPoles);
            bool aURat = aNative.IsURational(), aVRat = aNative.IsVRational();

            // As Geom_BezierSurface: knots {0, 1}, multiplicities degree + 1 (OCCT passes the U knots for V too)
            var aKnots = new NCollection_Array1<double>(1, 2);
            aKnots[1] = 0.0;
            aKnots[2] = 1.0;
            var aUMults = new NCollection_Array1<int>(1, 2);
            aUMults.Init(aUDegree + 1);
            var aVMults = new NCollection_Array1<int>(1, 2);
            aVMults.Init(aVDegree + 1);
            NCollection_Array2<double>? aW = aURat || aVRat ? aWeights : null;
            for (int ju = 0; ju <= 6; ju++)
            {
                for (int jv = 0; jv <= 6; jv++)
                {
                    double aU = ju / 6.0, aV = jv / 6.0;
                    Occt.BSplSLib.D0(aU, aV, 1, 1, aPoles, aW, aKnots, aKnots, aUMults, aVMults, aUDegree, aVDegree, aURat, aVRat,
                                     false, false, out gp_Pnt aP);
                    using (N.gp_Pnt aNP = aNative.EvalD0(aU, aV))
                    {
                        anOracle.Same(aNP, aP, "Bezier D0");
                    }

                    int aNu = anOracle.Int(0, 3), aNv = anOracle.Int(aNu == 0 ? 1 : 0, 3);
                    Occt.BSplSLib.DN(aU, aV, aNu, aNv, 0, 0, aPoles, aW, aKnots, aKnots, aUMults, aVMults, aUDegree, aVDegree,
                                     aURat, aVRat, false, false, out gp_Vec aDN);
                    using (N.gp_Vec aNDN = aNative.EvalDN(aU, aV, aNu, aNv))
                    {
                        anOracle.Same(aNDN, aDN, $"Bezier DN({aNu},{aNv})");
                    }
                }
            }
        }

        anOracle.Report("BSplSLib on Bezier surfaces: D0, DN");
    }

    /// <summary>
    /// A parameter for the cache tests: inside the domain (GeomAdaptor_Surface evaluates exactly on its bounds through
    /// LocalD1/LocalD2 instead of the cache), on an inner knot a third of the time, and shifted by a period of a
    /// periodic surface now and then.
    /// </summary>
    private static double CacheParam(Oracle theOracle, NCollection_Array1<double> theKnots, bool thePeriodic)
    {
        double aFirst = theKnots[theKnots.Lower()], aLast = theKnots[theKnots.Upper()];
        double aU = theKnots.Length() > 2 && theOracle.Int(0, 2) == 0
                      ? theKnots[theOracle.Int(theKnots.Lower() + 1, theKnots.Upper() - 1)]
                      : theOracle.Real(aFirst, aLast);
        if (aU == aFirst || aU == aLast)
        {
            aU = 0.5 * (aFirst + aLast);
        }

        return thePeriodic && theOracle.Int(0, 4) == 0 ? aU + (aLast - aFirst) * theOracle.Int(-2, 2) : aU;
    }

    [Fact]
    public void Cache_evaluation_through_GeomAdaptor()
    {
        // GeomAdaptor_Surface evaluates B-spline and Bezier surfaces through BSplSLib_Cache (GeomAdaptor_Surface.cxx,
        // RebuildCache, EvalD0/D1/D2): one cache per adaptor, rebuilt when IsCacheValid says the span changed. The
        // port's cache follows the same sequence of parameters, so it changes span exactly when native does.
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            var S = new Surface(anOracle);
            using N.Geom_BSplineSurface aNative = S.ToNative();
            using var anAdaptor = new N.GeomAdaptor_Surface(aNative);
            Rationality R = RationalityOf(aNative, S);
            BSplSLib_Cache? aCache = null;
            for (int j = 0; j < 40; j++)
            {
                double aU = CacheParam(anOracle, S.UKnots, S.UPeriodic), aV = CacheParam(anOracle, S.VKnots, S.VPeriodic);
                if (aCache is null || !aCache.IsCacheValid(aU, aV))
                {
                    aCache ??= new BSplSLib_Cache(S.UDegree, S.UPeriodic, S.UFlatKnots, S.VDegree, S.VPeriodic, S.VFlatKnots,
                                                  R.Weights);
                    aCache.BuildCache(aU, aV, S.UFlatKnots, S.VFlatKnots, S.Poles, R.Weights);
                }

                switch (j % 3)
                {
                    case 0:
                        {
                            aCache.D0(aU, aV, out gp_Pnt aP);
                            using N.gp_Pnt aNP = anAdaptor.EvalD0(aU, aV);
                            anOracle.Same(aNP, aP, "Cache D0");
                            break;
                        }
                    case 1:
                        {
                            aCache.D1(aU, aV, out gp_Pnt aP, out gp_Vec aDU, out gp_Vec aDV);
                            using var aNP = new N.gp_Pnt(0, 0, 0);
                            using var aNDU = new N.gp_Vec(0, 0, 0);
                            using var aNDV = new N.gp_Vec(0, 0, 0);
                            anAdaptor.D1(aU, aV, aNP, aNDU, aNDV);
                            anOracle.Same(aNP, aP, "Cache D1 P");
                            anOracle.Same(aNDU, aDU, "Cache D1 U");
                            anOracle.Same(aNDV, aDV, "Cache D1 V");
                            break;
                        }
                    default:
                        {
                            aCache.D2(aU, aV, out gp_Pnt aP, out gp_Vec aDU, out gp_Vec aDV, out gp_Vec aDUU, out gp_Vec aDVV,
                                      out gp_Vec aDUV);
                            using var aNP = new N.gp_Pnt(0, 0, 0);
                            using var aNDU = new N.gp_Vec(0, 0, 0);
                            using var aNDV = new N.gp_Vec(0, 0, 0);
                            using var aNDUU = new N.gp_Vec(0, 0, 0);
                            using var aNDVV = new N.gp_Vec(0, 0, 0);
                            using var aNDUV = new N.gp_Vec(0, 0, 0);
                            anAdaptor.D2(aU, aV, aNP, aNDU, aNDV, aNDUU, aNDVV, aNDUV);
                            anOracle.Same(aNP, aP, "Cache D2 P");
                            anOracle.Same(aNDU, aDU, "Cache D2 U");
                            anOracle.Same(aNDV, aDV, "Cache D2 V");
                            anOracle.Same(aNDUU, aDUU, "Cache D2 UU");
                            anOracle.Same(aNDVV, aDVV, "Cache D2 VV");
                            anOracle.Same(aNDUV, aDUV, "Cache D2 UV");
                            break;
                        }
                }
            }
        }

        // Bezier surfaces: the cache is built once, on flat Bezier knots, and never rebuilt
        for (int i = 0; i < CasesBezier; i++)
        {
            int aUDegree = anOracle.Int(1, 12), aVDegree = anOracle.Int(1, 12);
            bool isRational = anOracle.Int(0, 1) == 1;
            var aPoles = new NCollection_Array2<gp_Pnt>(1, aUDegree + 1, 1, aVDegree + 1);
            var aWeights = new NCollection_Array2<double>(1, aUDegree + 1, 1, aVDegree + 1);
            for (int r = 1; r <= aUDegree + 1; r++)
            {
                for (int c = 1; c <= aVDegree + 1; c++)
                {
                    aPoles[r, c] = new gp_Pnt(anOracle.Real(0, 1), anOracle.Real(0, 1), anOracle.Real(0, 1));
                    aWeights[r, c] = isRational ? anOracle.Real(0.5, 1.5) : 1.0;
                }
            }

            using N.TColgp_Array2OfPnt aNPoles = Oracle.ToNative(aPoles);
            using N.TColStd_Array2OfReal aNWeights = Oracle.ToNative(aWeights);
            using N.Geom_BezierSurface aNative = isRational ? new N.Geom_BezierSurface(aNPoles, aNWeights) : new N.Geom_BezierSurface(aNPoles);
            using var anAdaptor = new N.GeomAdaptor_Surface(aNative);
            NCollection_Array2<double>? aW = aNative.IsURational() || aNative.IsVRational() ? aWeights : null;
            NCollection_Array1<double> aFlatKnotsU = NCollection_Array1<double>.From(Occt.BSplCLib.FlatBezierKnots(aUDegree));
            NCollection_Array1<double> aFlatKnotsV = NCollection_Array1<double>.From(Occt.BSplCLib.FlatBezierKnots(aVDegree));
            BSplSLib_Cache? aCache = null;
            for (int j = 0; j < 30; j++)
            {
                double aU = anOracle.Real(0, 1), aV = anOracle.Real(0, 1);
                if (aCache is null)
                {
                    aCache = new BSplSLib_Cache(aUDegree, false, aFlatKnotsU, aVDegree, false, aFlatKnotsV, aW);
                    aCache.BuildCache(aU, aV, aFlatKnotsU, aFlatKnotsV, aPoles, aW);
                }

                aCache.D1(aU, aV, out gp_Pnt aP, out gp_Vec aDU, out gp_Vec aDV);
                using var aNP = new N.gp_Pnt(0, 0, 0);
                using var aNDU = new N.gp_Vec(0, 0, 0);
                using var aNDV = new N.gp_Vec(0, 0, 0);
                anAdaptor.D1(aU, aV, aNP, aNDU, aNDV);
                anOracle.Same(aNP, aP, "Bezier cache D1 P");
                anOracle.Same(aNDU, aDU, "Bezier cache D1 U");
                anOracle.Same(aNDV, aDV, "Bezier cache D1 V");
                aCache.D0(aU, aV, out gp_Pnt aP0);
                using N.gp_Pnt aNP0 = anAdaptor.EvalD0(aU, aV);
                anOracle.Same(aNP0, aP0, "Bezier cache D0");
            }
        }

        anOracle.Report("BSplSLib_Cache through GeomAdaptor_Surface: D0, D1, D2");
    }
}
#endif
