// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Laws;
using Kerf.Engine.Fillets.Blend;
using Kerf.Engine.Fillets.BlendFunctions;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Kerf.Kernel.ModelingAlgorithms.Tests.BlendRecorder;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The derivatives of BlendFunc's functions (<c>Values</c>) against central finite differences of their values, on
/// the plane/plane, plane/cylinder, cylinder/cylinder, cone/plane and B-spline/B-spline pairs of
/// <see cref="BlendRecorder"/>, at the start point and at the solution; and the places where OCCT's
/// <c>Derivatives</c> differs from its <c>Values</c>.
/// </summary>
public class BlendFuncDerivativeTests
{
    private static Matrix FiniteDifferences(FunctionSetWithDerivatives F, math_Vector X)
    {
        int n = F.NbEquations(), m = F.NbVariables();
        Matrix aD = new(1, n, 1, m);
        math_Vector aFp = new(1, n), aFm = new(1, n);
        for (int j = 1; j <= m; j++)
        {
            double h = 1.0e-6 * Math.Max(1.0, Math.Abs(X[j]));
            math_Vector Xp = new(1, m), Xm = new(1, m);
            Xp.Initialized(X);
            Xm.Initialized(X);
            Xp[j] += h;
            Xm[j] -= h;
            F.Value(Xp, aFp);
            F.Value(Xm, aFm);
            for (int i = 1; i <= n; i++)
            {
                aD[i, j] = (aFp[i] - aFm[i]) / (2.0 * h);
            }
        }

        return aD;
    }

    private static void CheckValues(FunctionSetWithDerivatives F, math_Vector X, string theCase,
                                    (int Row, int Col)? theOcctWrong = null)
    {
        int n = F.NbEquations(), m = F.NbVariables();
        math_Vector aF = new(1, n);
        Matrix aD = new(1, n, 1, m);
        Assert.True(F.Values(X, aF, aD), theCase);
        Matrix aFD = FiniteDifferences(F, X);
        for (int i = 1; i <= n; i++)
        {
            for (int j = 1; j <= m; j++)
            {
                if (theOcctWrong == (i, j))
                {
                    continue;
                }

                Assert.True(Math.Abs(aD[i, j] - aFD[i, j]) <= 1.0e-4 * (1.0 + Math.Abs(aD[i, j])),
                            $"{theCase}: D({i},{j}) = {aD[i, j]}, finite differences {aFD[i, j]}");
            }
        }
    }

    private static math_Vector Solve(FunctionSetWithDerivatives F, math_Vector X0, math_Vector Tol,
                                     math_Vector Inf, math_Vector Sup)
    {
        FunctionSetRoot aRoot = new(F, Tol, 30);
        aRoot.Perform(F, X0, Inf, Sup);
        Assert.True(aRoot.IsDone);
        return aRoot.Root();
    }

    private static IEnumerable<(string Name, BlendFunction F)> Functions(PairCase C)
    {
        SurfaceAdaptorBase S1 = Surf(C.S1), S2 = Surf(C.S2);
        CurveAdaptorBase G = Guide(C.G);
        ConstRad aConstRad = new(S1, S2, G);
        aConstRad.Set(1.0, 1);
        yield return ("constrad", aConstRad);
        Linear aLaw = new();
        aLaw.Set(G.FirstParameter(), 0.8, G.LastParameter(), 1.4);
        EvolRad anEvolRad = new(S1, S2, G, aLaw);
        anEvolRad.Set(1);
        yield return ("evolrad", anEvolRad);
        Chamfer aChamfer = new(S1, S2, G);
        aChamfer.Set(1.0, 1.5, 1);
        yield return ("chamfer", aChamfer);
        ConstThroat aThroat = new(S1, S2, G);
        aThroat.Set(0.7, 0.0, 1);
        yield return ("constthroat", aThroat);
        ConstThroatWithPenetration aPen = new(S1, S2, G);
        aPen.Set(1.0, 0.0, 1);
        yield return ("constthroatpen", aPen);
        ChAsym aChAsym = new(S1, S2, G);
        aChAsym.Set(1.0, Math.PI / 3.0, 1);
        yield return ("chasym", aChAsym);
        yield return ("ruled", new Ruled(S1, S2, G));
    }

    public static TheoryData<int> Pairs() => new() { 0, 1, 2, 3, 4 };

    [Theory]
    [MemberData(nameof(Pairs))]
    public void SurfaceFunctions_DerivativesMatchFiniteDifferences(int thePair)
    {
        PairCase C = THE_PAIRS[thePair];
        foreach ((string aName, BlendFunction F) in Functions(C))
        {
            F.Set(C.W);
            math_Vector X0 = new(1, 4), Tol = new(1, 4), Inf = new(1, 4), Sup = new(1, 4);
            for (int i = 1; i <= 4; i++)
            {
                X0[i] = C.X0[i - 1];
            }

            CheckValues(F, X0, $"{aName} {thePair} start");
            F.GetTolerance(Tol, 1.0e-7);
            F.GetBounds(Inf, Sup);
            math_Vector aSol = Solve(F, X0, Tol, Inf, Sup);
            CheckValues(F, aSol, $"{aName} {thePair} solution");

            // Derivatives is Values's matrix.
            math_Vector aF = new(1, 4);
            Matrix aD1 = new(1, 4, 1, 4), aD2 = new(1, 4, 1, 4);
            F.Derivatives(aSol, aD1);
            F.Values(aSol, aF, aD2);
            for (int i = 1; i <= 4; i++)
            {
                for (int j = 1; j <= 4; j++)
                {
                    Assert.Equal(aD2[i, j], aD1[i, j]);
                }
            }
        }
    }

    private static IEnumerable<(string Name, FuncInv F)> InverseFunctions(PairCase C)
    {
        SurfaceAdaptorBase S1 = Surf(C.S1), S2 = Surf(C.S2);
        CurveAdaptorBase G = Guide(C.G);
        ConstRadInv aConstRad = new(S1, S2, G);
        aConstRad.Set(1.0, 1);
        yield return ("constradinv", aConstRad);
        Linear aLaw = new();
        aLaw.Set(G.FirstParameter(), 0.8, G.LastParameter(), 1.4);
        EvolRadInv anEvolRad = new(S1, S2, G, aLaw);
        anEvolRad.Set(1);
        yield return ("evolradinv", anEvolRad);
        ChamfInv aChamf = new(S1, S2, G);
        aChamf.Set(1.0, 1.5, 1);
        yield return ("chamfinv", aChamf);
        ConstThroatWithPenetrationInv aPen = new(S1, S2, G);
        aPen.Set(1.0, 0.0, 1);
        yield return ("constthroatpeninv", aPen);
        ChAsymInv aChAsym = new(S1, S2, G);
        aChAsym.Set(1.0, Math.PI / 3.0, 1);
        yield return ("chasyminv", aChAsym);
        yield return ("ruledinv", new RuledInv(S1, S2, G));
    }

    [Theory]
    [InlineData(0, true, 5.0, 0.0, 0.0, 1.0, 1.0, 5.0, 5.0, -1.0)]
    [InlineData(0, false, 5.0, 0.0, 0.0, 1.0, -1.0, 5.0, 5.0, 1.0)]
    [InlineData(1, true, 0.0, 0.0, 0.3623577544766736, 0.9320390859672263, 4.0, 1.2, 1.2, 1.0)]
    [InlineData(4, true, 0.0, 0.5, 1.0, 0.0, 0.83, 1.5, 0.5, 0.75)]
    public void InverseFunctions_DerivativesMatchFiniteDifferences(int thePair, bool theOnFirst, double theX, double theY,
                                                                   double theDX, double theDY, double t, double w,
                                                                   double u, double v)
    {
        PairCase C = THE_PAIRS[thePair];
        Curve2DAdaptorBase R = new CurveAdaptor2D(
            new LineCurve2D(new Point2D(theX, theY), new Direction2D(theDX, theDY)), -10.0, 10.0);
        foreach ((string aName, FuncInv F) in InverseFunctions(C))
        {
            F.Set(theOnFirst, R);
            math_Vector X0 = new(1, 4);
            X0[1] = t;
            X0[2] = w;
            X0[3] = u;
            X0[4] = v;
            // OCCT: ChAsymInv's derivative of F(4) in the guide's parameter is wrong on S1's restriction when the
            // guide is curved (decision 1040); native gives the same value (BlendFuncNativeTests).
            CheckValues(F, X0, $"{aName} {thePair} {theOnFirst}",
                        aName == "chasyminv" && theOnFirst ? (4, 2) : null);

        }
    }

    [Fact]
    public void ConstRadInv_DerivativesUsesThePlanesOffset()
    {
        // OCCT: Derivatives computes the derivative of the plane's normal with the plane's offset where Values has
        // -nplan.d2cur, so their second columns differ off the origin (decision 1038); the other columns agree.
        PairCase C = THE_PAIRS[0];
        ConstRadInv F = new(Surf(C.S1), Surf(C.S2), Guide(C.G));
        F.Set(1.0, 1);
        F.Set(true, new CurveAdaptor2D(new LineCurve2D(new Point2D(5, 0), new Direction2D(0, 1)), -10.0, 10.0));
        math_Vector X = new(1, 4);
        X[1] = 1.0;
        X[2] = 5.0;
        X[3] = 5.0;
        X[4] = -1.0;
        math_Vector aF = new(1, 4);
        Matrix aD1 = new(1, 4, 1, 4), aD2 = new(1, 4, 1, 4);
        F.Derivatives(X, aD1);
        F.Values(X, aF, aD2);
        Assert.True(aD2[2, 2] != aD1[2, 2] || aD2[3, 2] != aD1[3, 2] || aD2[4, 2] != aD1[4, 2]);
        for (int i = 1; i <= 4; i++)
        {
            Assert.Equal(aD2[i, 1], aD1[i, 1]);
            Assert.Equal(aD2[i, 3], aD1[i, 3]);
            Assert.Equal(aD2[i, 4], aD1[i, 4]);
        }
    }

    [Fact]
    public void CSFunctions_DerivativesMatchFiniteDifferences()
    {
        SurfaceAdaptorBase S = Surf(0);
        CurveAdaptorBase C = Guide(4), G = Guide(0);
        CSConstRad aCS = new(S, C, G);
        aCS.Set(1.0, 1);
        aCS.Set(3.0);
        math_Vector X3 = new(1, 3);
        X3[1] = 3.0;
        X3[2] = -0.5;
        X3[3] = 3.0;
        CheckValues(aCS, X3, "csconstrad");

        Linear aLaw = new();
        aLaw.Set(0.0, 0.0, 10.0, 10.0);
        CSCircular aCirc = new(S, C, G, aLaw);
        aCirc.Set(1.0, 1);
        aCirc.Set(3.0);
        math_Vector X2 = new(1, 2);
        X2[1] = 3.0;
        X2[2] = -0.5;
        CheckValues(aCirc, X2, "cscircular");
    }

    [Fact]
    public void ConstRad_SecondOrderSectionMatchesFiniteDifferencesOfTheFirst()
    {
        // The derivative of the poles (Section with D2Poles) against finite differences of the first-order section
        // along the guide, on the plane/cylinder pair.
        PairCase C = THE_PAIRS[1];
        SurfaceAdaptorBase S1 = Surf(C.S1), S2 = Surf(C.S2);
        CurveAdaptorBase G = Guide(C.G);
        ConstRad F = new(S1, S2, G);
        F.Set(1.0, 1);
        F.Set(G.FirstParameter(), G.LastParameter());

        math_Vector X0 = new(1, 4), Tol = new(1, 4), Inf = new(1, 4), Sup = new(1, 4);
        for (int i = 1; i <= 4; i++)
        {
            X0[i] = C.X0[i - 1];
        }

        BlendPoint PointAt(double W)
        {
            F.Set(W);
            F.GetTolerance(Tol, 1.0e-10);
            F.GetBounds(Inf, Sup);
            math_Vector aSol = Solve(F, X0, Tol, Inf, Sup);
            Assert.True(F.IsSolution(aSol, 1.0e-9));
            return new BlendPoint(F.PointOnS1(), F.PointOnS2(), W, aSol[1], aSol[2], aSol[3], aSol[4],
                                   F.TangentOnS1(), F.TangentOnS2(), F.Tangent2dOnS1(), F.Tangent2dOnS2());
        }

        F.GetShape(out int aNbPoles, out _, out _, out _);
        NCollection_Array1<Point3D> Section1(double W, NCollection_Array1<Vector3D> theD)
        {
            var aPoles = new NCollection_Array1<Point3D>(1, aNbPoles);
            var aPoles2d = new NCollection_Array1<Point2D>(1, 2);
            var aDPoles2d = new NCollection_Array1<Vector2D>(1, 2);
            var aW = new NCollection_Array1<double>(1, aNbPoles);
            var aDW = new NCollection_Array1<double>(1, aNbPoles);
            Assert.True(F.Section(PointAt(W), aPoles, theD, aPoles2d, aDPoles2d, aW, aDW));
            return aPoles;
        }

        double h = 1.0e-5;
        var aDp = new NCollection_Array1<Vector3D>(1, aNbPoles);
        var aDm = new NCollection_Array1<Vector3D>(1, aNbPoles);
        var aD0 = new NCollection_Array1<Vector3D>(1, aNbPoles);
        NCollection_Array1<Point3D> aPp = Section1(C.W + h, aDp);
        NCollection_Array1<Point3D> aPm = Section1(C.W - h, aDm);
        Section1(C.W, aD0);

        var aPoles2 = new NCollection_Array1<Point3D>(1, aNbPoles);
        var aDPoles2 = new NCollection_Array1<Vector3D>(1, aNbPoles);
        var aD2Poles2 = new NCollection_Array1<Vector3D>(1, aNbPoles);
        var aP2d = new NCollection_Array1<Point2D>(1, 2);
        var aDP2d = new NCollection_Array1<Vector2D>(1, 2);
        var aD2P2d = new NCollection_Array1<Vector2D>(1, 2);
        var aW2 = new NCollection_Array1<double>(1, aNbPoles);
        var aDW2 = new NCollection_Array1<double>(1, aNbPoles);
        var aD2W2 = new NCollection_Array1<double>(1, aNbPoles);
        Assert.True(F.Section(PointAt(C.W), aPoles2, aDPoles2, aD2Poles2, aP2d, aDP2d, aD2P2d, aW2, aDW2, aD2W2));
        for (int i = 1; i <= aNbPoles; i++)
        {
            Vector3D aFD1 = new Vector3D(aPm[i], aPp[i]) / (2.0 * h);
            Assert.True((aFD1 - aDPoles2[i]).Magnitude <= 1.0e-4 * (1.0 + aDPoles2[i].Magnitude), $"DPoles({i})");
            Vector3D aFD2 = (aDp[i] - aDm[i]) / (2.0 * h);
            Assert.True((aFD2 - aD2Poles2[i]).Magnitude <= 1.0e-3 * (1.0 + aD2Poles2[i].Magnitude), $"D2Poles({i})");
        }
    }
}
