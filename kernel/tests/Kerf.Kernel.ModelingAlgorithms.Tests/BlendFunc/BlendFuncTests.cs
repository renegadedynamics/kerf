// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Laws;
using Kerf.Engine.Fillets.Blend;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Kerf.Engine.Fillets.BlendFunctions.SectionShape;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;
using static Kerf.Geometry.Continuity;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>Blend's points, BlendFunc's package functions and tensor, and the blend functions' small members.</summary>
public class BlendFuncTests
{
    [Fact]
    public void BlendPoint_KnowsWhatItCarries()
    {
        BlendPoint P0 = new();
        Assert.True(P0.IsTangencyPoint);

        BlendPoint P = new(new Point3D(1, 0, 0), new Point3D(0, 1, 0), 0.5, 1, 2, 3, 4, new Vector3D(1, 0, 0),
                            new Vector3D(0, 1, 0), new Vector2D(1, 1), new Vector2D(2, 2));
        Assert.False(P.IsTangencyPoint);
        P.ParametersOnS2(out double u, out double v);
        Assert.Equal((3.0, 4.0), (u, v));
        Assert.Equal(new Vector2D(2, 2), P.Tangent2dOnS2());
        EngineAssert.ThrowsAny<DomainErrorException>(() => P.ParameterOnC());
        Assert.Equal(0.5, P.Parameter());
        Assert.Equal(0.75, P.WithParameter(0.75).Parameter());

        BlendPoint C = new(new Point3D(1, 0, 0), new Point3D(0, 1, 0), 0.5, 1, 2, 7);
        Assert.True(C.IsTangencyPoint);
        Assert.Equal(7.0, C.ParameterOnC());
        EngineAssert.ThrowsAny<DomainErrorException>(() => C.ParametersOnS2(out _, out _));
        EngineAssert.ThrowsAny<DomainErrorException>(() => C.TangentOnS());
        EngineAssert.ThrowsAny<DomainErrorException>(() => C.ParameterOnC1());

        BlendPoint R = P0.WithValue(new Point3D(), new Point3D(1, 1, 1), 2.0, 5, 6);
        Assert.Equal((5.0, 6.0), (R.ParameterOnC1(), R.ParameterOnC2()));
        EngineAssert.ThrowsAny<DomainErrorException>(() => R.ParametersOnS1(out _, out _));
        Assert.Equal(P, new BlendPoint(new Point3D(1, 0, 0), new Point3D(0, 1, 0), 0.5, 1, 2, 3, 4, new Vector3D(1, 0, 0),
                                        new Vector3D(0, 1, 0), new Vector2D(1, 1), new Vector2D(2, 2)));
    }

    [Fact]
    public void BlendFunc_ShapesAndContinuity()
    {
        ParameterisationType T = TgtThetaOver2;
        global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.GetShape(global::Kerf.Engine.Fillets.BlendFunctions.SectionShape.Rational, 1.0, out int nbp, out int nbk, out int deg, ref T);
        Assert.Equal((3, 2, 2, TgtThetaOver2_1), (nbp, nbk, deg, T));
        global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.GetShape(global::Kerf.Engine.Fillets.BlendFunctions.SectionShape.Rational, 3.0, out nbp, out nbk, out deg, ref T);
        Assert.Equal((7, 2, 6, global::Kerf.Engine.Geometry.Conversion.ParameterisationType.QuasiAngular), (nbp, nbk, deg, T));
        global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.GetShape(global::Kerf.Engine.Fillets.BlendFunctions.SectionShape.Polynomial, 3.0, out nbp, out nbk, out deg, ref T);
        Assert.Equal((8, 2, 7, global::Kerf.Engine.Geometry.Conversion.ParameterisationType.Polynomial), (nbp, nbk, deg, T));
        global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.GetShape(global::Kerf.Engine.Fillets.BlendFunctions.SectionShape.Linear, 3.0, out nbp, out nbk, out deg, ref T);
        Assert.Equal((2, 2, 1, global::Kerf.Engine.Geometry.Conversion.ParameterisationType.Polynomial), (nbp, nbk, deg, T)); // the conversion is left as it was

        Assert.Equal(C1, global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.NextShape(C0));
        Assert.Equal(C3, global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.NextShape(C2));
        Assert.Equal(CN, global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.NextShape(C3));
        Assert.Equal(CN, global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.NextShape(G1));

        var aW = new NCollection_Array1<double>(1, 3);
        global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.GetMinimalWeights(global::Kerf.Engine.Fillets.BlendFunctions.SectionShape.Rational, TgtThetaOver2_1, 0.5, 1.0, aW);
        Assert.Equal(1.0, aW[1]);
        Assert.Equal(Math.Cos(0.5), aW[2], 1.0e-15); // the smaller angle gives the smaller middle weight
        global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.GetMinimalWeights(global::Kerf.Engine.Fillets.BlendFunctions.SectionShape.Linear, TgtThetaOver2_1, 0.5, 1.0, aW);
        Assert.Equal(1.0, aW[2]);
    }

    [Fact]
    public void BlendFunc_ComputeNormalAtASingularPoint()
    {
        // At the pole of a sphere the first derivatives are parallel: the normal comes from the higher ones.
        SurfaceAdaptor aSphere = new(new SphericalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)),
                                                                    2.0));
        Vector3D N = new(), DNu = new(), DNv = new();
        Assert.True(global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.ComputeNormal(aSphere, new Point2D(0.3, Math.PI / 2.0), ref N));
        Assert.Equal(1.0, Math.Abs(N.Z), 1.0e-12);
        Assert.True(global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.ComputeDNormal(aSphere, new Point2D(0.3, Math.PI / 2.0), ref N, ref DNu, ref DNv));
        Assert.Equal(1.0, Math.Abs(N.Z), 1.0e-12);

    }

    [Fact]
    public void Tensor_MultipliesByItsLastIndex()
    {
        global::Kerf.Engine.Fillets.BlendFunctions.Tensor T = new(2, 3, 4);
        T.Init(1.0);
        T[2, 3, 4] = 5.0;
        Assert.Equal(5.0, T.Value(2, 3, 4));
        math_Vector V = new(1, 4, 1.0);
        Matrix M = new(1, 2, 1, 3);
        T.Multiply(V, M);
        Assert.Equal(4.0, M[1, 1]);
        Assert.Equal(8.0, M[2, 3]);
    }

    [Fact]
    public void Functions_SmallMembers()
    {
        SurfaceAdaptorBase S1 = BlendRecorder.Surf(0), S2 = BlendRecorder.Surf(1);
        CurveAdaptorBase G = BlendRecorder.Guide(0);
        global::Kerf.Engine.Fillets.BlendFunctions.ConstRad aConstRad = new(S1, S2, G);
        Assert.Equal(4, aConstRad.NbVariables());
        Assert.Equal(4, aConstRad.NbEquations());
        Assert.True(aConstRad.IsTangencyPoint);
        EngineAssert.ThrowsAny<DomainErrorException>(() => aConstRad.TangentOnS1());
        Assert.True(aConstRad.IsRational);
        aConstRad.Set(global::Kerf.Engine.Fillets.BlendFunctions.SectionShape.Linear);
        Assert.False(aConstRad.IsRational);
        Assert.Equal(RealLast(), aConstRad.GetMinimalDistance());

        global::Kerf.Engine.Fillets.BlendFunctions.Chamfer aChamfer = new(S1, S2, G);
        Assert.False(aChamfer.IsRational);
        EngineAssert.ThrowsAny<MethodNotImplementedException>(() => aChamfer.GetSectionSize());
        var aKnots = new NCollection_Array1<double>(1, 2);
        aChamfer.Knots(aKnots);
        Assert.Equal((0.0, 1.0), (aKnots[1], aKnots[2]));

        global::Kerf.Engine.Fillets.BlendFunctions.Ruled aRuled = new(S1, S2, G);
        EngineAssert.ThrowsAny<MethodNotImplementedException>(() => aRuled.Set(0.0, 1.0));

        global::Kerf.Engine.Fillets.BlendFunctions.CSConstRad aCS = new(S1, BlendRecorder.Guide(4), G);
        Assert.Equal(3, aCS.NbVariables());
        Assert.Equal(3, aCS.NbEquations());
        Assert.Equal(aCS.PointOnC(), aCS.Pnt1());
        EngineAssert.ThrowsAny<MethodNotImplementedException>(() => ((CSFunction)aCS).Set(0.0, 1.0));

        global::Kerf.Engine.Fillets.BlendFunctions.CSCircular aCirc = new(S1, BlendRecorder.Guide(4), G, new Linear());
        Assert.Equal(2, aCirc.NbVariables());

        Assert.Equal(4, new global::Kerf.Engine.Fillets.BlendFunctions.ChamfInv(S1, S2, G).NbVariables());
        Assert.Equal(4, new global::Kerf.Engine.Fillets.BlendFunctions.RuledInv(S1, S2, G).NbEquations());
    }

    [Fact]
    public void ConstRad_ScratchIsPerThread()
    {
        // OCCT's thread_local scratch of ComputeValues is [ThreadStatic] (decision 1037): the same function on two
        // threads gives the same values as on one.
        BlendRecorder.PairCase C = BlendRecorder.THE_PAIRS[1];
        string Run()
        {
            global::Kerf.Engine.Fillets.BlendFunctions.ConstRad F = new(BlendRecorder.Surf(C.S1), BlendRecorder.Surf(C.S2), BlendRecorder.Guide(C.G));
            F.Set(1.0, 1);
            F.Set(C.W);
            math_Vector X = new(1, 4), aF = new(1, 4);
            for (int i = 1; i <= 4; i++)
            {
                X[i] = C.X0[i - 1];
            }

            Matrix aD = new(1, 4, 1, 4);
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
