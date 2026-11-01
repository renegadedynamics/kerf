// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BlendFunc_SectionShape;
using static Occt.Convert_ParameterisationType;
using static Occt.GeomAbs_Shape;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>Blend's points, BlendFunc's package functions and tensor, and the blend functions' small members.</summary>
public class BlendFuncTests
{
    [Fact]
    public void BlendPoint_KnowsWhatItCarries()
    {
        Blend_Point P0 = new();
        Assert.True(P0.IsTangencyPoint());

        Blend_Point P = new(new gp_Pnt(1, 0, 0), new gp_Pnt(0, 1, 0), 0.5, 1, 2, 3, 4, new gp_Vec(1, 0, 0),
                            new gp_Vec(0, 1, 0), new gp_Vec2d(1, 1), new gp_Vec2d(2, 2));
        Assert.False(P.IsTangencyPoint());
        P.ParametersOnS2(out double u, out double v);
        Assert.Equal((3.0, 4.0), (u, v));
        Assert.Equal(new gp_Vec2d(2, 2), P.Tangent2dOnS2());
        Assert.ThrowsAny<Standard_DomainError>(() => P.ParameterOnC());
        Assert.Equal(0.5, P.Parameter());
        Assert.Equal(0.75, P.WithParameter(0.75).Parameter());

        Blend_Point C = new(new gp_Pnt(1, 0, 0), new gp_Pnt(0, 1, 0), 0.5, 1, 2, 7);
        Assert.True(C.IsTangencyPoint());
        Assert.Equal(7.0, C.ParameterOnC());
        Assert.ThrowsAny<Standard_DomainError>(() => C.ParametersOnS2(out _, out _));
        Assert.ThrowsAny<Standard_DomainError>(() => C.TangentOnS());
        Assert.ThrowsAny<Standard_DomainError>(() => C.ParameterOnC1());

        Blend_Point R = P0.WithValue(new gp_Pnt(), new gp_Pnt(1, 1, 1), 2.0, 5, 6);
        Assert.Equal((5.0, 6.0), (R.ParameterOnC1(), R.ParameterOnC2()));
        Assert.ThrowsAny<Standard_DomainError>(() => R.ParametersOnS1(out _, out _));
        Assert.Equal(P, new Blend_Point(new gp_Pnt(1, 0, 0), new gp_Pnt(0, 1, 0), 0.5, 1, 2, 3, 4, new gp_Vec(1, 0, 0),
                                        new gp_Vec(0, 1, 0), new gp_Vec2d(1, 1), new gp_Vec2d(2, 2)));
    }

    [Fact]
    public void BlendFunc_ShapesAndContinuity()
    {
        Convert_ParameterisationType T = Convert_TgtThetaOver2;
        BlendFunc.GetShape(BlendFunc_Rational, 1.0, out int nbp, out int nbk, out int deg, ref T);
        Assert.Equal((3, 2, 2, Convert_TgtThetaOver2_1), (nbp, nbk, deg, T));
        BlendFunc.GetShape(BlendFunc_Rational, 3.0, out nbp, out nbk, out deg, ref T);
        Assert.Equal((7, 2, 6, Convert_QuasiAngular), (nbp, nbk, deg, T));
        BlendFunc.GetShape(BlendFunc_Polynomial, 3.0, out nbp, out nbk, out deg, ref T);
        Assert.Equal((8, 2, 7, Convert_Polynomial), (nbp, nbk, deg, T));
        BlendFunc.GetShape(BlendFunc_Linear, 3.0, out nbp, out nbk, out deg, ref T);
        Assert.Equal((2, 2, 1, Convert_Polynomial), (nbp, nbk, deg, T)); // the conversion is left as it was

        Assert.Equal(GeomAbs_C1, BlendFunc.NextShape(GeomAbs_C0));
        Assert.Equal(GeomAbs_C3, BlendFunc.NextShape(GeomAbs_C2));
        Assert.Equal(GeomAbs_CN, BlendFunc.NextShape(GeomAbs_C3));
        Assert.Equal(GeomAbs_CN, BlendFunc.NextShape(GeomAbs_G1));

        var aW = new NCollection_Array1<double>(1, 3);
        BlendFunc.GetMinimalWeights(BlendFunc_Rational, Convert_TgtThetaOver2_1, 0.5, 1.0, aW);
        Assert.Equal(1.0, aW[1]);
        Assert.Equal(Math.Cos(0.5), aW[2], 1.0e-15); // the smaller angle gives the smaller middle weight
        BlendFunc.GetMinimalWeights(BlendFunc_Linear, Convert_TgtThetaOver2_1, 0.5, 1.0, aW);
        Assert.Equal(1.0, aW[2]);
    }

    [Fact]
    public void BlendFunc_ComputeNormalAtASingularPoint()
    {
        // At the pole of a sphere the first derivatives are parallel: the normal comes from the higher ones.
        GeomAdaptor_Surface aSphere = new(new Geom_SphericalSurface(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)),
                                                                    2.0));
        gp_Vec N = new(), DNu = new(), DNv = new();
        Assert.True(BlendFunc.ComputeNormal(aSphere, new gp_Pnt2d(0.3, Math.PI / 2.0), ref N));
        Assert.Equal(1.0, Math.Abs(N.Z()), 1.0e-12);
        Assert.True(BlendFunc.ComputeDNormal(aSphere, new gp_Pnt2d(0.3, Math.PI / 2.0), ref N, ref DNu, ref DNv));
        Assert.Equal(1.0, Math.Abs(N.Z()), 1.0e-12);

    }

    [Fact]
    public void Tensor_MultipliesByItsLastIndex()
    {
        BlendFunc_Tensor T = new(2, 3, 4);
        T.Init(1.0);
        T[2, 3, 4] = 5.0;
        Assert.Equal(5.0, T.Value(2, 3, 4));
        math_Vector V = new(1, 4, 1.0);
        math_Matrix M = new(1, 2, 1, 3);
        T.Multiply(V, M);
        Assert.Equal(4.0, M[1, 1]);
        Assert.Equal(8.0, M[2, 3]);
    }

    [Fact]
    public void Functions_SmallMembers()
    {
        Adaptor3d_Surface S1 = BlendRecorder.Surf(0), S2 = BlendRecorder.Surf(1);
        Adaptor3d_Curve G = BlendRecorder.Guide(0);
        BlendFunc_ConstRad aConstRad = new(S1, S2, G);
        Assert.Equal(4, aConstRad.NbVariables());
        Assert.Equal(4, aConstRad.NbEquations());
        Assert.True(aConstRad.IsTangencyPoint());
        Assert.ThrowsAny<Standard_DomainError>(() => aConstRad.TangentOnS1());
        Assert.True(aConstRad.IsRational());
        aConstRad.Set(BlendFunc_Linear);
        Assert.False(aConstRad.IsRational());
        Assert.Equal(RealLast(), aConstRad.GetMinimalDistance());

        BlendFunc_Chamfer aChamfer = new(S1, S2, G);
        Assert.False(aChamfer.IsRational());
        Assert.ThrowsAny<Standard_NotImplemented>(() => aChamfer.GetSectionSize());
        var aKnots = new NCollection_Array1<double>(1, 2);
        aChamfer.Knots(aKnots);
        Assert.Equal((0.0, 1.0), (aKnots[1], aKnots[2]));

        BlendFunc_Ruled aRuled = new(S1, S2, G);
        Assert.ThrowsAny<Standard_NotImplemented>(() => aRuled.Set(0.0, 1.0));

        BlendFunc_CSConstRad aCS = new(S1, BlendRecorder.Guide(4), G);
        Assert.Equal(3, aCS.NbVariables());
        Assert.Equal(3, aCS.NbEquations());
        Assert.Equal(aCS.PointOnC(), aCS.Pnt1());
        Assert.ThrowsAny<Standard_NotImplemented>(() => ((Blend_CSFunction)aCS).Set(0.0, 1.0));

        BlendFunc_CSCircular aCirc = new(S1, BlendRecorder.Guide(4), G, new Law_Linear());
        Assert.Equal(2, aCirc.NbVariables());

        Assert.Equal(4, new BlendFunc_ChamfInv(S1, S2, G).NbVariables());
        Assert.Equal(4, new BlendFunc_RuledInv(S1, S2, G).NbEquations());
    }

    [Fact]
    public void ConstRad_ScratchIsPerThread()
    {
        // OCCT's thread_local scratch of ComputeValues is [ThreadStatic] (decision 1037): the same function on two
        // threads gives the same values as on one.
        BlendRecorder.PairCase C = BlendRecorder.THE_PAIRS[1];
        string Run()
        {
            BlendFunc_ConstRad F = new(BlendRecorder.Surf(C.S1), BlendRecorder.Surf(C.S2), BlendRecorder.Guide(C.G));
            F.Set(1.0, 1);
            F.Set(C.W);
            math_Vector X = new(1, 4), aF = new(1, 4);
            for (int i = 1; i <= 4; i++)
            {
                X[i] = C.X0[i - 1];
            }

            math_Matrix aD = new(1, 4, 1, 4);
            F.Values(X, aF, aD);
            return string.Join(",", Enumerable.Range(1, 4).Select(i => BitConverter.DoubleToInt64Bits(aF[i])))
                   + string.Join(",", Enumerable.Range(1, 4).Select(j => BitConverter.DoubleToInt64Bits(aD[2, j])));
        }

        string aSerial = Run();
        string[] aParallel = new string[8];
        Parallel.For(0, 8, i => aParallel[i] = Run());
        Assert.All(aParallel, s => Assert.Equal(aSerial, s));
    }
}
