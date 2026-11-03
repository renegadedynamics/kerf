// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Surfaces;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Unit tests of the trihedron laws (Frenet, Darboux, ConstantBiNormal, Fixed, DiscreteTrihedron, GuideTrihedronAC,
/// GuideTrihedronPlan) and their helpers (SnglrFunc, PlanFunc): orthonormal frames, derivatives against finite
/// differences, intervals, singular points, closed forms and copies.
/// </summary>
public class GeomFill_TrihedronTests
{
    private const double Tol = 1.0e-9;

    internal readonly record struct Frame(Vector3D T, Vector3D N, Vector3D B);

    internal static Frame D0(TrihedronLaw theLaw, double theParam)
    {
        Vector3D aT = new(), aN = new(), aB = new();
        Assert.True(theLaw.D0(theParam, ref aT, ref aN, ref aB));
        return new Frame(aT, aN, aB);
    }

    internal static (Frame F, Frame D) D1(TrihedronLaw theLaw, double theParam)
    {
        Vector3D aT = new(), aN = new(), aB = new(), aDT = new(), aDN = new(), aDB = new();
        Assert.True(theLaw.D1(theParam, ref aT, ref aDT, ref aN, ref aDN, ref aB, ref aDB));
        return (new Frame(aT, aN, aB), new Frame(aDT, aDN, aDB));
    }

    internal static (Frame F, Frame D, Frame D2) D2(TrihedronLaw theLaw, double theParam)
    {
        Vector3D aT = new(), aN = new(), aB = new(), aDT = new(), aDN = new(), aDB = new(), aD2T = new(), aD2N = new(),
               aD2B = new();
        Assert.True(theLaw.D2(theParam, ref aT, ref aDT, ref aD2T, ref aN, ref aDN, ref aD2N, ref aB, ref aDB, ref aD2B));
        return (new Frame(aT, aN, aB), new Frame(aDT, aDN, aDB), new Frame(aD2T, aD2N, aD2B));
    }

    internal static void AssertNear(in Vector3D theExpected, in Vector3D theActual, double theTol, string theWhat)
    {
        Assert.True(theExpected.Subtracted(theActual).Magnitude() <= theTol,
                    $"{theWhat}: expected {theExpected}, got {theActual}");
    }

    internal static void AssertOrthonormal(in Frame theF, double theTol = Tol)
    {
        Assert.Equal(1.0, theF.T.Magnitude(), theTol);
        Assert.Equal(1.0, theF.N.Magnitude(), theTol);
        Assert.Equal(1.0, theF.B.Magnitude(), theTol);
        Assert.Equal(0.0, theF.T.Dot(theF.N), theTol);
        Assert.Equal(0.0, theF.T.Dot(theF.B), theTol);
        Assert.Equal(0.0, theF.N.Dot(theF.B), theTol);
        // right-handed: N = B ^ T
        AssertNear(theF.B.Crossed(theF.T), theF.N, theTol, "N = B ^ T");
    }

    /// <summary>D1 and D2 against central differences of D0 and D1, and D0 within D1 and D2.</summary>
    internal static void AssertDerivatives(TrihedronLaw theLaw, double theParam, double theH = 1.0e-5,
                                           double theTol = 1.0e-5, bool theCheckD2 = true)
    {
        Frame aF = D0(theLaw, theParam);
        (Frame aF1, Frame aD) = D1(theLaw, theParam);
        (Frame aF2, Frame aD1b, Frame aDD) = D2(theLaw, theParam);
        AssertNear(aF.T, aF1.T, Tol, "D1 tangent");
        AssertNear(aF.N, aF1.N, Tol, "D1 normal");
        AssertNear(aF.B, aF1.B, Tol, "D1 binormal");
        AssertNear(aF.T, aF2.T, Tol, "D2 tangent");
        AssertNear(aD.T, aD1b.T, Tol, "D2 first derivative of the tangent");

        Frame aP = D0(theLaw, theParam + theH), aM = D0(theLaw, theParam - theH);
        double aScale = Math.Max(1.0, aD.T.Magnitude() + aD.N.Magnitude() + aD.B.Magnitude());
        AssertNear((aP.T - aM.T) / (2 * theH), aD.T, theTol * aScale, "DTangent");
        AssertNear((aP.N - aM.N) / (2 * theH), aD.N, theTol * aScale, "DNormal");
        AssertNear((aP.B - aM.B) / (2 * theH), aD.B, theTol * aScale, "DBiNormal");

        if (!theCheckD2)
        {
            return;
        }
        (_, Frame aDP) = D1(theLaw, theParam + theH);
        (_, Frame aDM) = D1(theLaw, theParam - theH);
        double aScale2 = Math.Max(1.0, aDD.T.Magnitude() + aDD.N.Magnitude() + aDD.B.Magnitude());
        AssertNear((aDP.T - aDM.T) / (2 * theH), aDD.T, theTol * aScale2, "D2Tangent");
        AssertNear((aDP.N - aDM.N) / (2 * theH), aDD.N, theTol * aScale2, "D2Normal");
        AssertNear((aDP.B - aDM.B) / (2 * theH), aDD.B, theTol * aScale2, "D2BiNormal");
    }

    internal static BSplineCurve3D BSpline(params Point3D[] thePoles)
    {
        int aDeg = Math.Min(3, thePoles.Length - 1);
        int aNbKnots = thePoles.Length - aDeg + 1;
        NCollection_Array1<Point3D> aPoles = new(1, thePoles.Length);
        for (int i = 0; i < thePoles.Length; i++)
        {
            aPoles[i + 1] = thePoles[i];
        }
        NCollection_Array1<double> aKnots = new(1, aNbKnots);
        NCollection_Array1<int> aMults = new(1, aNbKnots);
        for (int i = 1; i <= aNbKnots; i++)
        {
            aKnots[i] = i - 1;
            aMults[i] = (i == 1 || i == aNbKnots) ? aDeg + 1 : 1;
        }
        return new BSplineCurve3D(aPoles, aKnots, aMults, aDeg);
    }

    /// <summary>A twisted (non-planar) cubic B-spline on [0, 2].</summary>
    internal static BSplineCurve3D Twisted() =>
        BSpline(new Point3D(0, 0, 0), new Point3D(1, 2, 0.5), new Point3D(3, 2, 2), new Point3D(4, -1, 3), new Point3D(6, 0, 2));

    // ------------------------------------------------------------------------------------------------------------
    // Frenet

    [Fact]
    public void Frenet_Circle_ClosedForm()
    {
        Frame3D anAx = new(new Point3D(1, 2, 3), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0));
        Frenet aLaw = new();
        Assert.True(aLaw.SetCurve(new CurveAdaptor(new CircleCurve3D(anAx, 2.0))));
        foreach (double u in new[] { 0.0, 0.7, 2.0, 5.5 })
        {
            Frame aF = D0(aLaw, u);
            AssertOrthonormal(aF);
            AssertNear(new Vector3D(-Math.Sin(u), Math.Cos(u), 0), aF.T, Tol, "tangent");
            AssertNear(new Vector3D(-Math.Cos(u), -Math.Sin(u), 0), aF.N, Tol, "normal to the centre");
            AssertNear(new Vector3D(0, 0, 1), aF.B, Tol, "binormal on the axis");
            AssertDerivatives(aLaw, u);
        }
        Assert.False(aLaw.IsConstant);
        Assert.True(aLaw.IsOnlyBy3dCurve);
        Assert.Equal(SurfacePipeError.PipeOk, aLaw.ErrorStatus());
    }

    [Fact]
    public void Frenet_Line_IsConstant()
    {
        Frenet aLaw = new();
        aLaw.SetCurve(new CurveAdaptor(new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 1, 0))));
        Assert.True(aLaw.IsConstant);
        Frame aF = D0(aLaw, 3.0);
        AssertOrthonormal(aF);
        AssertNear(new Vector3D(new Direction3D(1, 1, 0)), aF.T, Tol, "tangent");
        (_, Frame aD) = D1(aLaw, 3.0);
        Assert.Equal(0.0, aD.T.Magnitude() + aD.N.Magnitude() + aD.B.Magnitude(), 0.0);
    }

    [Fact]
    public void Frenet_TwistedBSpline_Derivatives()
    {
        Frenet aLaw = new();
        aLaw.SetCurve(new CurveAdaptor(Twisted()));
        foreach (double u in new[] { 0.1, 0.5, 0.9, 1.3, 1.8 })
        {
            AssertOrthonormal(D0(aLaw, u));
            AssertDerivatives(aLaw, u);
        }
        // C0 of the law asks C2 of the curve, C1 asks C3: the knot at 1 is only C2
        Assert.Equal(1, aLaw.NbIntervals(Continuity.C0));
        Assert.Equal(2, aLaw.NbIntervals(Continuity.C1));
        NCollection_Array1<double> aT = new(1, 3);
        aLaw.Intervals(aT, Continuity.C1);
        Assert.Equal([0.0, 1.0, 2.0], new[] { aT[1], aT[2], aT[3] });
    }

    [Fact]
    public void Frenet_Inflexion_IsSingularPoint()
    {
        // A planar S curve: C' ^ C'' vanishes at its inflexion, t = 0.5 by symmetry.
        BSplineCurve3D aC = BSpline(new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(2, -1, 0), new Point3D(3, 0, 0));
        Frenet aLaw = new();
        aLaw.SetCurve(new CurveAdaptor(aC));
        Assert.Equal(2, aLaw.NbIntervals(Continuity.C0));
        NCollection_Array1<double> aT = new(1, 3);
        aLaw.Intervals(aT, Continuity.C0);
        Assert.Equal(0.0, aT[1]);
        Assert.Equal(0.5, aT[2], 1.0e-7);
        Assert.Equal(1.0, aT[3]);

        // The trihedron stays orthonormal on both sides and at the singular point, the binormal along +-Z.
        foreach (double u in new[] { 0.2, 0.5 - 1.0e-7, 0.5, 0.5 + 1.0e-7, 0.8 })
        {
            Frame aF = D0(aLaw, u);
            AssertOrthonormal(aF);
            Assert.Equal(1.0, Math.Abs(aF.B.Z()), 1.0e-9);
            AssertNear(new Vector3D(new Direction3D(aC.DN(u, 1))), aF.T, 1.0e-9, "tangent");
        }
        // On a planar curve the binormal flips across the inflexion.
        Assert.True(D0(aLaw, 0.2).B.Z() * D0(aLaw, 0.8).B.Z() < 0);
        var (aF1, _) = D1(aLaw, 0.5);
        AssertOrthonormal(aF1);
        var (aF2, _, _) = D2(aLaw, 0.5);
        AssertOrthonormal(aF2);
    }

    [Fact]
    public void Frenet_StraightPart_UsesADefaultNormal()
    {
        // A degree-1 polyline segment is straight: C'' = 0, so D0 takes any normal to the tangent.
        BSplineCurve3D aC = BSpline(new Point3D(0, 0, 0), new Point3D(2, 1, 0));
        Frenet aLaw = new();
        aLaw.SetCurve(new CurveAdaptor(aC));
        Frame aF = D0(aLaw, 0.3);
        AssertOrthonormal(aF);
        AssertNear(new Vector3D(new Direction3D(2, 1, 0)), aF.T, Tol, "tangent");
        (_, Frame aD) = D1(aLaw, 0.3);
        Assert.Equal(0.0, aD.T.Magnitude() + aD.N.Magnitude() + aD.B.Magnitude(), 0.0);
    }

    [Fact]
    public void Frenet_SetInterval_GetInterval()
    {
        Frenet aLaw = new();
        aLaw.SetCurve(new CurveAdaptor(Twisted()));
        aLaw.SetInterval(0.25, 1.5);
        aLaw.GetInterval(out double aF, out double aL);
        Assert.Equal(0.25, aF);
        Assert.Equal(1.5, aL);
        AssertOrthonormal(D0(aLaw, 0.25));
    }

    [Fact]
    public void Frenet_Copy_IsIndependent()
    {
        Frenet aLaw = new();
        aLaw.SetCurve(new CurveAdaptor(Twisted()));
        Frame aBefore = D0(aLaw, 0.7);
        TrihedronLaw aCopy = aLaw.Copy();
        Assert.IsType<Frenet>(aCopy);
        aLaw.SetCurve(new CurveAdaptor(new CircleCurve3D(new Frame3D(), 1.0)));
        Frame aAfter = D0(aCopy, 0.7);
        Assert.Equal(aBefore, aAfter);
    }

    [Fact]
    public void Frenet_AverageLaw_IsOrthonormal()
    {
        Frenet aLaw = new();
        aLaw.SetCurve(new CurveAdaptor(Twisted()));
        Vector3D aT = new(), aN = new(), aB = new();
        aLaw.GetAverageLaw(ref aT, ref aN, ref aB);
        AssertOrthonormal(new Frame(aT, aN, aB));
    }

    // ------------------------------------------------------------------------------------------------------------
    // Fixed and ConstantBiNormal

    [Fact]
    public void Fixed_ClosedForm()
    {
        Fixed aLaw = new(new Vector3D(2, 0, 0), new Vector3D(0, 3, 0));
        aLaw.SetCurve(new CurveAdaptor(Twisted()));
        var (aF, aD, aDD) = D2(aLaw, 0.4);
        Assert.Equal(new Frame(new Vector3D(1, 0, 0), new Vector3D(0, 1, 0), new Vector3D(0, 0, 1)), aF);
        Assert.Equal(new Frame(new Vector3D(), new Vector3D(), new Vector3D()), aD);
        Assert.Equal(new Frame(new Vector3D(), new Vector3D(), new Vector3D()), aDD);
        Assert.True(aLaw.IsConstant);
        Assert.False(aLaw.IsOnlyBy3dCurve);
        Assert.Equal(1, aLaw.NbIntervals(Continuity.C2));
        NCollection_Array1<double> aT = new(1, 2);
        aLaw.Intervals(aT, Continuity.C2);
        Assert.Equal(-global::Kerf.Engine.Foundation.Precision.Infinite(), aT[1]);
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Infinite(), aT[2]);
        Assert.Equal(aF, D0(aLaw.Copy(), 1.9));
    }

    [Fact]
    public void Fixed_ParallelVectors_Throw()
    {
        Assert.Throws<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new Fixed(new Vector3D(1, 0, 0), new Vector3D(2, 0.001, 0)));
    }

    [Fact]
    public void ConstantBiNormal_KeepsTheBiNormal()
    {
        Direction3D aBN = new(0, 0, 1);
        ConstantBiNormal aLaw = new(aBN);
        Assert.True(aLaw.SetCurve(new CurveAdaptor(Twisted())));
        BSplineCurve3D aC = Twisted();
        foreach (double u in new[] { 0.2, 0.9, 1.6 })
        {
            Frame aF = D0(aLaw, u);
            AssertOrthonormal(aF);
            AssertNear(new Vector3D(aBN), aF.B, 0.0, "binormal");
            // the tangent is the curve's, projected on the plane normal to the binormal
            Vector3D aCT = aC.DN(u, 1);
            Vector3D aProj = aCT - new Vector3D(aBN) * aCT.Dot(new Vector3D(aBN));
            AssertNear(aProj.Normalized(), aF.T, 1.0e-12, "projected tangent");
            AssertDerivatives(aLaw, u);
        }
        Assert.IsType<ConstantBiNormal>(aLaw.Copy());
        Assert.Equal(D0(aLaw, 0.9), D0(aLaw.Copy(), 0.9));
    }

    [Fact]
    public void ConstantBiNormal_IsOnlyBy3dCurve()
    {
        CircleCurve3D aCircle = new(new Frame3D(new Point3D(), new Direction3D(0, 0, 1)), 2.0);
        ConstantBiNormal aParallel = new(new Direction3D(0, 0, 1));
        aParallel.SetCurve(new CurveAdaptor(aCircle));
        Assert.True(aParallel.IsOnlyBy3dCurve);
        ConstantBiNormal aOblique = new(new Direction3D(0, 1, 1));
        aOblique.SetCurve(new CurveAdaptor(aCircle));
        Assert.False(aOblique.IsOnlyBy3dCurve);
        ConstantBiNormal aLine = new(new Direction3D(0, 0, 1));
        aLine.SetCurve(new CurveAdaptor(new LineCurve3D(new Point3D(), new Direction3D(1, 0, 0))));
        Assert.True(aLine.IsOnlyBy3dCurve);
        aLine.SetCurve(new CurveAdaptor(Twisted()));
        Assert.False(aLine.IsOnlyBy3dCurve);
    }

    // ------------------------------------------------------------------------------------------------------------
    // Darboux

    private static CurveOnSurface CurveOnSurface(Surface theS, Curve2D theC2d) =>
        new(new CurveAdaptor2D(theC2d), new SurfaceAdaptor(theS));

    [Fact]
    public void Darboux_OnAPlane_BiNormalIsTheNormal()
    {
        PlaneSurface aPlane = new(new CoordinateSystem(new Point3D(0, 0, 1), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)));
        CircleCurve2D aC2d = new(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 1.5);
        Darboux aLaw = new();
        aLaw.SetCurve(CurveOnSurface(aPlane, aC2d));
        foreach (double u in new[] { 0.3, 2.0, 4.5 })
        {
            Frame aF = D0(aLaw, u);
            AssertOrthonormal(aF);
            AssertNear(new Vector3D(0, 0, 1), aF.B, Tol, "surface normal");
            AssertNear(new Vector3D(-Math.Sin(u), Math.Cos(u), 0), aF.T, Tol, "tangent");
            AssertDerivatives(aLaw, u);
        }
        Assert.False(aLaw.IsOnlyBy3dCurve);
        Assert.IsType<Darboux>(aLaw.Copy());
    }

    [Fact]
    public void Darboux_OnACylinder_Derivatives()
    {
        CylindricalSurface aCyl = new(new CoordinateSystem(), 2.0);
        // a helix-like line in the (u, v) plane
        LineCurve2D aL2d = new(new Point2D(0.2, 0), new Direction2D(1, 0.7));
        Darboux aLaw = new();
        aLaw.SetCurve(CurveOnSurface(aCyl, aL2d));
        foreach (double t in new[] { 0.0, 1.0, 2.5 })
        {
            Frame aF = D0(aLaw, t);
            AssertOrthonormal(aF);
            aL2d.D0(t, out Point2D aUV);
            Vector3D aRadial = new(Math.Cos(aUV.X()), Math.Sin(aUV.X()), 0);
            AssertNear(aRadial, aF.B, Tol, "cylinder normal");
            AssertDerivatives(aLaw, t);
        }
    }

    // ------------------------------------------------------------------------------------------------------------
    // DiscreteTrihedron

    [Fact]
    public void DiscreteTrihedron_FreeForm_FollowsTheTangent()
    {
        BSplineCurve3D aC = Twisted();
        DiscreteTrihedron aLaw = new();
        Assert.False(aLaw.SetCurve(new CurveAdaptor(aC)));
        foreach (double u in new[] { 0.0, 0.2, 0.33, 1.0, 1.77, 2.0 })
        {
            Frame aF = D0(aLaw, u);
            AssertOrthonormal(aF, 1.0e-12);
            AssertNear(new Vector3D(new Direction3D(aC.DN(u, 1))), aF.T, 1.0e-12, "tangent");
        }
        // a rotation-minimising frame turns little between neighbours
        Frame a1 = D0(aLaw, 0.5), a2 = D0(aLaw, 0.5001);
        Assert.True(a1.N.Subtracted(a2.N).Magnitude() < 1.0e-3);
        var (_, aD, aDD) = D2(aLaw, 0.5);
        Assert.Equal(new Frame(new Vector3D(), new Vector3D(), new Vector3D()), aD);
        Assert.Equal(new Frame(new Vector3D(), new Vector3D(), new Vector3D()), aDD);
        Assert.Equal(D0(aLaw, 0.7), D0(aLaw.Copy(), 0.7));
    }

    [Fact]
    public void DiscreteTrihedron_Conic_UsesFrenet()
    {
        CircleCurve3D aCircle = new(new Frame3D(), 2.0);
        DiscreteTrihedron aLaw = new();
        Assert.True(aLaw.SetCurve(new CurveAdaptor(aCircle)));
        Frenet aFrenet = new();
        aFrenet.SetCurve(new CurveAdaptor(aCircle));
        Assert.Equal(D0(aFrenet, 1.1), D0(aLaw, 1.1));
    }

    // ------------------------------------------------------------------------------------------------------------
    // Guides

    [Fact]
    public void GuideTrihedronAC_ParallelLines_NormalToTheGuide()
    {
        CurveAdaptor aPath = new(new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 0, 10);
        CurveAdaptor aGuide = new(new LineCurve3D(new Point3D(3, 0, 0), new Direction3D(0, 0, 1)), 0, 10);
        GuideTrihedronAC aLaw = new(aGuide);
        aLaw.SetCurve(aPath);
        Frame aF = D0(aLaw, 4.0);
        AssertOrthonormal(aF);
        AssertNear(new Vector3D(1, 0, 0), aF.N, 1.0e-9, "normal to the guide");
        AssertNear(new Vector3D(0, 0, 1), aF.T, 1.0e-9, "tangent");
        Assert.True(aLaw.CurrentPointOnGuide().IsEqual(new Point3D(3, 0, 4), 1.0e-7));
        Assert.Same(aGuide, aLaw.Guide());
        Assert.Equal(aF, D0(aLaw.Copy(), 4.0));
        Assert.Equal(1, aLaw.NbIntervals(Continuity.C2));
    }

    [Fact]
    public void GuideTrihedronAC_Derivatives()
    {
        CurveAdaptor aPath = new(Twisted());
        BSplineCurve3D aG = BSpline(new Point3D(0, 3, 0), new Point3D(1, 5, 0.5), new Point3D(3, 5, 2), new Point3D(4, 2, 3),
                                       new Point3D(6, 3, 2));
        GuideTrihedronAC aLaw = new(new CurveAdaptor(aG));
        aLaw.SetCurve(aPath);
        foreach (double u in new[] { 0.3, 1.2 })
        {
            AssertOrthonormal(D0(aLaw, u), 1.0e-9);
            // the parameter on the guide comes from a curvilinear-abscissa approximation (1e-7). D2 is not checked:
            // OCCT's D2BiNormal uses |TN|^2 (= 1) where the derivative of a unit vector needs |DTN|^2
            // (occt-issues.md), so D2BiNormal and D2Tangent are not the derivatives of D1's.
            AssertDerivatives(aLaw, u, 1.0e-4, 1.0e-3, false);
        }
    }

    [Fact]
    public void GuideTrihedronAC_D2BiNormal_KeepsOcctsError()
    {
        // OCCT's D2BiNormal misses (1 - |DTN|^2) * BiNormal (occt-issues.md): the difference from the derivative of
        // DBiNormal is along the binormal.
        CurveAdaptor aPath = new(Twisted());
        BSplineCurve3D aG = BSpline(new Point3D(0, 3, 0), new Point3D(1, 5, 0.5), new Point3D(3, 5, 2), new Point3D(4, 2, 3),
                                       new Point3D(6, 3, 2));
        GuideTrihedronAC aLaw = new(new CurveAdaptor(aG));
        aLaw.SetCurve(aPath);
        double u = 0.3, h = 1.0e-4;
        var (aF, _, aDD) = D2(aLaw, u);
        (_, Frame aDP) = D1(aLaw, u + h);
        (_, Frame aDM) = D1(aLaw, u - h);
        Vector3D anError = aDD.B - (aDP.B - aDM.B) / (2 * h);
        Assert.True(anError.Magnitude() > 1.0e-2);
        Assert.True(anError.Crossed(aF.B).Magnitude() < 1.0e-3 * anError.Magnitude());
    }

    [Fact]
    public void GuideTrihedronPlan_ParallelLines()
    {
        CurveAdaptor aPath = new(new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 0, 10);
        CurveAdaptor aGuide = new(new LineCurve3D(new Point3D(0, 2, 0), new Direction3D(0, 0, 1)), -1, 11);
        GuideTrihedronPlan aLaw = new(aGuide);
        aLaw.SetCurve(aPath);
        Assert.Equal(SurfacePipeError.PipeOk, aLaw.ErrorStatus());
        Frame aF = D0(aLaw, 5.0);
        AssertNear(new Vector3D(0, 1, 0), aF.N, 1.0e-9, "normal to the guide");
        AssertNear(new Vector3D(0, 0, 1), aF.T, 1.0e-12, "tangent");
        Assert.True(aLaw.IsConstant);
        Assert.Equal(aF, D0(aLaw.Copy(), 5.0));
        // OCCT's D2 computes only the Frenet trihedron and returns false
        Vector3D v1 = new(), v2 = new(), v3 = new(), v4 = new(), v5 = new(), v6 = new(), v7 = new(), v8 = new(), v9 = new();
        Assert.False(aLaw.D2(5.0, ref v1, ref v2, ref v3, ref v4, ref v5, ref v6, ref v7, ref v8, ref v9));
    }

    [Fact]
    public void GuideTrihedronPlan_Curved_D1MatchesD0()
    {
        // the middle of the path, and the whole path moved up as the guide: every normal plane crosses the guide
        CurveAdaptor aPath = new(Twisted(), 0.5, 1.5);
        Curve3D aG = (Curve3D)Twisted().Translated(new Vector3D(0, 0, 3));
        GuideTrihedronPlan aLaw = new(new CurveAdaptor(aG));
        aLaw.SetCurve(aPath);
        Assert.Equal(SurfacePipeError.PipeOk, aLaw.ErrorStatus());
        double u = 0.9;
        Frame aF = D0(aLaw, u);
        (Frame aF1, Frame aD) = D1(aLaw, u);
        AssertNear(aF.N, aF1.N, 1.0e-9, "normal");
        AssertNear(aF.T, aF1.T, 0.0, "tangent");
        // the normal reaches the guide in the plane normal to the tangent
        aPath.D0(u, out Point3D aP);
        Assert.Equal(0.0, aF.T.Dot(aF.N), 1.0e-9);
        double h = 1.0e-5;
        AssertNear((D0(aLaw, u + h).N - D0(aLaw, u - h).N) / (2 * h), aD.N, 1.0e-4, "DNormal");
        _ = aP;
    }

    // ------------------------------------------------------------------------------------------------------------
    // Helpers

    [Fact]
    public void SnglrFunc_IsCPrimeCrossCSecond()
    {
        BSplineCurve3D aC = Twisted();
        CurveAdaptor anAd = new(aC);
        SnglrFunc aF = new(anAd);
        aF.SetRatio(0.5);
        aC.D3(0.7, out _, out Vector3D aD1, out Vector3D aD2, out Vector3D aD3);
        Point3D aP = aF.Value(0.7);
        AssertNear((aD1 * 0.5).Crossed(aD2), new Vector3D(aP.XYZ()), 1.0e-12, "value");
        aF.D1(0.7, out _, out Vector3D aV);
        AssertNear((aD1 * 0.5).Crossed(aD3), aV, 1.0e-12, "derivative");
        Assert.Equal(CurveKind.OtherCurve, aF.GetType());
        Assert.Equal(anAd.NbIntervals(Continuity.C2), aF.NbIntervals(Continuity.C0));
        Assert.Throws<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aF.DN(0.7, 4));
        Assert.IsType<SnglrFunc>(aF.ShallowCopy());
    }

    [Fact]
    public void PlanFunc_RootIsThePlaneCrossing()
    {
        CurveAdaptor aC = new(new LineCurve3D(new Point3D(0, 0, -5), new Direction3D(0, 0, 1)));
        PlanFunc aF = new(new Point3D(0, 0, 2), new Vector3D(0, 0, 2), aC);
        Assert.True(aF.Value(7.0, out double aV));
        Assert.Equal(0.0, aV, 1.0e-15);
        Assert.True(aF.Values(8.0, out aV, out double aD));
        Assert.Equal(2.0, aV, 1.0e-15);
        Assert.Equal(2.0, aD, 1.0e-15);
        aF.DEDT(8.0, new Vector3D(0, 0, 1), new Vector3D(0, 0, 0), out double aDFDT);
        Assert.Equal(-2.0, aDFDT, 1.0e-15);
    }
}
