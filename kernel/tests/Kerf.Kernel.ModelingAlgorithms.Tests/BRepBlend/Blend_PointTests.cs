// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Fillets.Blend;
using Kerf.Engine.Foundation;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Blend_Point's forms: a point on two surfaces, on a surface and a curve, on a surface and a restriction, on two
/// restrictions, each with tangents or as a tangency point. Each form answers for what it was given, the tangents
/// refuse on a tangency point, and the copies leave the original alone.
/// </summary>
public class Blend_PointTests
{
    private static readonly Point3D P1 = new(1, 2, 3), P2 = new(4, 5, 6);
    private static readonly Vector3D T1 = new(1, 0, 0), T2 = new(0, 1, 0);
    private static readonly Vector2D U1 = new(0.5, 0.25), U2 = new(0.75, 0.125);

    private static void AssertSurfaces(in BlendPoint theP, double theU1, double theV1, double theU2, double theV2)
    {
        theP.ParametersOnS1(out double u1, out double v1);
        theP.ParametersOnS2(out double u2, out double v2);
        Assert.Equal((theU1, theV1, theU2, theV2), (u1, v1, u2, v2));
    }

    [Fact]
    public void OnTwoSurfaces_WithTangents()
    {
        BlendPoint aP = new(P1, P2, 9, 0.1, 0.2, 0.3, 0.4, T1, T2, U1, U2);
        Assert.False(aP.IsTangencyPoint);
        Assert.Equal(9, aP.Parameter());
        Assert.Equal(P1, aP.PointOnS1());
        Assert.Equal(P2, aP.PointOnS2());
        AssertSurfaces(aP, 0.1, 0.2, 0.3, 0.4);
        Assert.Equal(T1, aP.TangentOnS1());
        Assert.Equal(T2, aP.TangentOnS2());
        Assert.Equal(U1, aP.Tangent2dOnS1());
        Assert.Equal(U2, aP.Tangent2dOnS2());
    }

    [Fact]
    public void OnTwoSurfaces_Tangency_RefusesTangents()
    {
        BlendPoint aP = new(P1, P2, 9, 0.1, 0.2, 0.3, 0.4);
        Assert.True(aP.IsTangencyPoint);
        AssertSurfaces(aP, 0.1, 0.2, 0.3, 0.4);
        AssertTangentsRefused(aP);
    }

    private static void AssertTangentsRefused(BlendPoint theP)
    {
        EngineAssert.ThrowsAny<DomainErrorException>(() => theP.TangentOnS1());
        EngineAssert.ThrowsAny<DomainErrorException>(() => theP.TangentOnS2());
        EngineAssert.ThrowsAny<DomainErrorException>(() => theP.Tangent2dOnS1());
        EngineAssert.ThrowsAny<DomainErrorException>(() => theP.Tangent2dOnS2());
        EngineAssert.ThrowsAny<DomainErrorException>(() => theP.TangentOnS());
        EngineAssert.ThrowsAny<DomainErrorException>(() => theP.TangentOnC());
        EngineAssert.ThrowsAny<DomainErrorException>(() => theP.Tangent2d());
        EngineAssert.ThrowsAny<DomainErrorException>(() => theP.TangentOnC1());
        EngineAssert.ThrowsAny<DomainErrorException>(() => theP.TangentOnC2());
    }

    [Fact]
    public void OnASurfaceAndACurve_WithTangents_AnswersOnlyForThose()
    {
        BlendPoint aP = new(P1, P2, 9, 0.1, 0.2, 0.7, T1, T2, U1);
        Assert.False(aP.IsTangencyPoint);
        Assert.Equal(P1, aP.PointOnS());
        Assert.Equal(P2, aP.PointOnC());
        aP.ParametersOnS(out double u, out double v);
        Assert.Equal((0.1, 0.2), (u, v));
        Assert.Equal(0.7, aP.ParameterOnC());
        Assert.Equal(T1, aP.TangentOnS());
        Assert.Equal(T2, aP.TangentOnC());
        Assert.Equal(U1, aP.Tangent2d());
        // there's no second surface
        BlendPoint aCopy = aP;
        EngineAssert.ThrowsAny<DomainErrorException>(() => aCopy.ParametersOnS2(out _, out _));
    }

    [Fact]
    public void OnASurfaceAndACurve_Tangency_RefusesTangents()
    {
        BlendPoint aP = new(P1, P2, 9, 0.1, 0.2, 0.7);
        Assert.True(aP.IsTangencyPoint);
        Assert.Equal(0.7, aP.ParameterOnC());
        AssertTangentsRefused(aP);
    }

    [Fact]
    public void OnTwoSurfaces_WithARestrictionOnTheSecond()
    {
        BlendPoint aP = new(P1, P2, 9, 0.1, 0.2, 0.3, 0.4, 0.8, T1, T2, U1, U2);
        Assert.False(aP.IsTangencyPoint);
        AssertSurfaces(aP, 0.1, 0.2, 0.3, 0.4);
        Assert.Equal(0.8, aP.ParameterOnC2());
        Assert.Equal(T2, aP.TangentOnC2());
        BlendPoint aCopy = aP;
        EngineAssert.ThrowsAny<DomainErrorException>(() => aCopy.ParameterOnC1());

        BlendPoint aTangency = new(P1, P2, 9, 0.1, 0.2, 0.3, 0.4, 0.8);
        Assert.True(aTangency.IsTangencyPoint);
        Assert.Equal(0.8, aTangency.ParameterOnC2());
        AssertTangentsRefused(aTangency);
    }

    [Fact]
    public void OnTwoSurfaces_WithARestrictionOnEach()
    {
        BlendPoint aP = new(P1, P2, 9, 0.1, 0.2, 0.3, 0.4, 0.7, 0.8, T1, T2, U1, U2);
        Assert.False(aP.IsTangencyPoint);
        Assert.Equal(P1, aP.PointOnC1());
        Assert.Equal(P2, aP.PointOnC2());
        Assert.Equal(0.7, aP.ParameterOnC1());
        Assert.Equal(0.8, aP.ParameterOnC2());
        Assert.Equal(T1, aP.TangentOnC1());
        Assert.Equal(T2, aP.TangentOnC2());

        BlendPoint aTangency = new(P1, P2, 9, 0.1, 0.2, 0.3, 0.4, 0.7, 0.8);
        Assert.True(aTangency.IsTangencyPoint);
        AssertTangentsRefused(aTangency);
    }

    [Fact]
    public void WithValueOnTwoCurves_ForgetsTheSurfaces_LeavingTheOriginal()
    {
        BlendPoint aP = new(P1, P2, 9, 0.1, 0.2, 0.3, 0.4);
        BlendPoint aCurves = aP.WithValue(P2, P1, 3, 0.7, 0.8);
        Assert.Equal(3, aCurves.Parameter());
        Assert.Equal(0.7, aCurves.ParameterOnC1());
        Assert.Equal(0.8, aCurves.ParameterOnC2());
        Assert.True(aCurves.IsTangencyPoint);
        EngineAssert.ThrowsAny<DomainErrorException>(() => aCurves.ParametersOnS1(out _, out _));
        EngineAssert.ThrowsAny<DomainErrorException>(() => aCurves.ParametersOnS2(out _, out _));
        // the original still has them
        AssertSurfaces(aP, 0.1, 0.2, 0.3, 0.4);
        Assert.Equal(9, aP.Parameter());
    }

    [Fact]
    public void SetValue_TakesEachFormAsItsConstructorDoes()
    {
        BlendPoint aSet = new();
        aSet.SetValue(P1, P2, 9, 0.1, 0.2, 0.3, 0.4, T1, T2, U1, U2);
        Assert.Equal(new BlendPoint(P1, P2, 9, 0.1, 0.2, 0.3, 0.4, T1, T2, U1, U2), aSet);
        aSet.SetValue(P1, P2, 9, 0.1, 0.2, 0.3, 0.4);
        Assert.Equal(new BlendPoint(P1, P2, 9, 0.1, 0.2, 0.3, 0.4).IsTangencyPoint, aSet.IsTangencyPoint);
        aSet.SetValue(P1, P2, 9, 0.1, 0.2, 0.7, T1, T2, U1);
        Assert.Equal(0.7, aSet.ParameterOnC());
        Assert.False(aSet.IsTangencyPoint);
        aSet.SetValue(P1, P2, 9, 0.1, 0.2, 0.7);
        Assert.True(aSet.IsTangencyPoint);
        aSet.SetValue(P1, P2, 9, 0.1, 0.2, 0.3, 0.4, 0.8, T1, T2, U1, U2);
        Assert.Equal(0.8, aSet.ParameterOnC2());
        Assert.False(aSet.IsTangencyPoint);
        aSet.SetValue(P1, P2, 9, 0.1, 0.2, 0.3, 0.4, 0.8);
        Assert.True(aSet.IsTangencyPoint);
        aSet.SetValue(P1, P2, 9, 0.1, 0.2, 0.3, 0.4, 0.7, 0.8, T1, T2, U1, U2);
        Assert.Equal(0.7, aSet.ParameterOnC1());
        Assert.False(aSet.IsTangencyPoint);
        aSet.SetValue(P1, P2, 9, 0.1, 0.2, 0.3, 0.4, 0.7, 0.8);
        Assert.True(aSet.IsTangencyPoint);
    }

    [Fact]
    public void Equality_IsByValue_AndOverAllFields()
    {
        BlendPoint aP = new(P1, P2, 9, 0.1, 0.2, 0.3, 0.4);
        BlendPoint aSame = new(P1, P2, 9, 0.1, 0.2, 0.3, 0.4);
        Assert.True(aP == aSame);
        Assert.False(aP != aSame);
        Assert.Equal(aP.GetHashCode(), aSame.GetHashCode());
        Assert.True(aP.Equals((object)aSame));
        Assert.False(aP.Equals((object)"text"));
        BlendPoint aOther = aP.WithParameter(10);
        Assert.True(aP != aOther);
        Assert.Contains("Blend_Point", aP.ToString());
    }
}
