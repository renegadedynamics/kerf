// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.IntRes2d_Position;
using static Occt.IntRes2d_Situation;
using static Occt.IntRes2d_TypeTrans;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// IntImpParGen's statics and its generic intersector, instantiated over test tools: an implicit circle and a
/// parametric segment (OCCT's own instantiations, over IntCurve's conic tools, are ported with IntCurve).
/// </summary>
public class IntImpParGenTests
{
    /// <summary>The circle of center <c>C</c> and radius <c>R</c>, as an implicit curve (counterclockwise from +X).</summary>
    public readonly struct TestCircleTool(gp_Pnt2d C, double R) : IIntImpParGen_ImpTool
    {
        public gp_Pnt2d Value(double U) => new(C.X() + R * Math.Cos(U), C.Y() + R * Math.Sin(U));

        public void D1(double U, out gp_Pnt2d P, out gp_Vec2d T)
        {
            P = Value(U);
            T = new gp_Vec2d(-R * Math.Sin(U), R * Math.Cos(U));
        }

        public void D2(double U, out gp_Pnt2d P, out gp_Vec2d T, out gp_Vec2d N)
        {
            D1(U, out P, out T);
            N = new gp_Vec2d(-R * Math.Cos(U), -R * Math.Sin(U));
        }

        public double Distance(in gp_Pnt2d P) => P.Distance(C) - R;

        public gp_Vec2d GradDistance(in gp_Pnt2d P)
        {
            gp_Vec2d aV = new(C, P);
            return aV.Divided(aV.Magnitude());
        }

        public double FindParameter(in gp_Pnt2d P)
        {
            double aU = Math.Atan2(P.Y() - C.Y(), P.X() - C.X());
            return aU < 0 ? aU + 2 * Math.PI : aU;
        }
    }

    /// <summary>The segment <c>P0 + t V</c>.</summary>
    public readonly record struct TestSegment(gp_Pnt2d P0, gp_Vec2d V)
    {
        public gp_Pnt2d Value(double t) => new(P0.X() + t * V.X(), P0.Y() + t * V.Y());
    }

    public readonly struct TestSegmentTool : IIntImpParGen_ParTool<TestSegment>,
                                             IIntImpParGen_ProjectOnPCurveTool<TestSegment>
    {
        public static double EpsX(in TestSegment C) => 1.0e-10;

        public static int NbSamples(in TestSegment C, double U0, double U1) => 10;

        public static gp_Pnt2d Value(in TestSegment C, double X) => C.Value(X);

        public static void D2(in TestSegment C, double U, out gp_Pnt2d P, out gp_Vec2d T, out gp_Vec2d N)
        {
            P = C.Value(U);
            T = C.V;
            N = new gp_Vec2d(0, 0);
        }

        public static double FindParameter(in TestSegment C, in gp_Pnt2d Pnt, double Tol)
            => new gp_Vec2d(C.P0, Pnt).Dot(C.V) / C.V.SquareMagnitude();

        public static double FindParameter(in TestSegment C, in gp_Pnt2d Pnt, double LowParameter,
                                           double HighParameter, double Tol)
            => FindParameter(C, Pnt, Tol);
    }

    /// <summary>The signed distance to the circle of the point at a parameter on the segment.</summary>
    public sealed class TestImpParTool : math_FunctionWithDerivative,
                                         IIntImpParGen_MyImpParTool<TestImpParTool, TestCircleTool, TestSegment>
    {
        private readonly TestCircleTool myImp;
        private readonly TestSegment myPar;

        private TestImpParTool(in TestCircleTool IT, in TestSegment PC)
        {
            myImp = IT;
            myPar = PC;
        }

        public static TestImpParTool Create(in TestCircleTool IT, in TestSegment PC) => new(IT, PC);

        public override bool Value(double X, out double F)
        {
            F = myImp.Distance(myPar.Value(X));
            return true;
        }

        public override bool Derivative(double X, out double D)
        {
            D = myImp.GradDistance(myPar.Value(X)).Dot(myPar.V);
            return true;
        }

        public override bool Values(double X, out double F, out double D)
        {
            Value(X, out F);
            Derivative(X, out D);
            return true;
        }
    }

    private sealed class CircleSegmentIntersector
        : IntImpParGen_Intersector<TestCircleTool, TestSegment, TestSegmentTool, TestSegmentTool, TestImpParTool>
    {
        public CircleSegmentIntersector(in TestCircleTool Imp, in IntRes2d_Domain D1, in TestSegment Par,
                                        in IntRes2d_Domain D2, double TolConf, double Tol)
            : base(Imp, D1, Par, D2, TolConf, Tol)
        {
        }
    }

    private static readonly TestCircleTool theUnitCircle = new(new gp_Pnt2d(0, 0), 1);

    private static IntRes2d_Domain CircleDomain()
        => new IntRes2d_Domain(theUnitCircle.Value(0), 0, 1e-7, theUnitCircle.Value(2 * Math.PI), 2 * Math.PI, 1e-7)
            .WithEquivalentParameters(0, 2 * Math.PI);

    private static IntRes2d_Domain SegmentDomain(in TestSegment S, double t0 = 0, double t1 = 1)
        => new(S.Value(t0), t0, 1e-7, S.Value(t1), t1, 1e-7);

    [Fact]
    public void DetermineTransition_Crossing()
    {
        gp_Vec2d aTan1 = new(0, 1), aTan2 = new(1, 0);
        IntRes2d_Transition aT1 = new(), aT2 = new();
        IntImpParGen.DetermineTransition(IntRes2d_Middle, ref aTan1, new gp_Vec2d(-1, 0), ref aT1,
                                         IntRes2d_Head, ref aTan2, new gp_Vec2d(0, 0), ref aT2, 1e-7);
        // Tan1 ^ Tan2 < 0: the first curve goes in, the second out
        Assert.Equal(IntRes2d_In, aT1.TransitionType());
        Assert.Equal(IntRes2d_Out, aT2.TransitionType());
        Assert.Equal(IntRes2d_Head, aT2.PositionOnCurve());
        Assert.False(aT1.IsTangent());

        Assert.True(IntImpParGen.DetermineTransition(IntRes2d_Middle, ref aTan2, ref aT1, IntRes2d_End, ref aTan1,
                                                     ref aT2, 1e-7));
        Assert.Equal(IntRes2d_Out, aT1.TransitionType());
        Assert.Equal(IntRes2d_In, aT2.TransitionType());
    }

    [Fact]
    public void DetermineTransition_TouchByCurvature()
    {
        // A circle of curvature 1 touching a straight line from above: tangents (1,0), the circle's normal (0,1).
        gp_Vec2d aTan1 = new(1, 0), aTan2 = new(1, 0);
        IntRes2d_Transition aT1 = new(), aT2 = new();
        IntImpParGen.DetermineTransition(IntRes2d_Middle, ref aTan1, new gp_Vec2d(0, 1), ref aT1,
                                         IntRes2d_Middle, ref aTan2, new gp_Vec2d(0, 0), ref aT2, 1e-7);
        Assert.Equal(IntRes2d_Touch, aT1.TransitionType());
        Assert.Equal(IntRes2d_Inside, aT1.Situation());
        Assert.Equal(IntRes2d_Outside, aT2.Situation());
        Assert.False(aT1.IsOpposite());

        // both straight and tangent: touch, situation unknown; the opposite flag from the tangents
        aTan1 = new gp_Vec2d(1, 0);
        aTan2 = new gp_Vec2d(-2, 0);
        IntImpParGen.DetermineTransition(IntRes2d_Middle, ref aTan1, new gp_Vec2d(0, 0), ref aT1,
                                         IntRes2d_Middle, ref aTan2, new gp_Vec2d(0, 0), ref aT2, 1e-7);
        Assert.Equal(IntRes2d_Unknown, aT1.Situation());
        Assert.True(aT2.IsOpposite());

        // the tangent-only form gives up on tangent curves
        Assert.False(IntImpParGen.DetermineTransition(IntRes2d_Middle, ref aTan1, ref aT1, IntRes2d_Middle,
                                                      ref aTan2, ref aT2, 1e-7));
    }

    [Fact]
    public void DetermineTransition_ZeroTangentUsesNormalOrIsUndecided()
    {
        gp_Vec2d aTan1 = new(0, 0), aTan2 = new(1, 0);
        IntRes2d_Transition aT1 = new(), aT2 = new();
        IntImpParGen.DetermineTransition(IntRes2d_Head, ref aTan1, new gp_Vec2d(0, 1), ref aT1,
                                         IntRes2d_Middle, ref aTan2, new gp_Vec2d(0, 0), ref aT2, 1e-7);
        Assert.Equal(new gp_Vec2d(0, 1), aTan1); // the normal replaced the null tangent
        Assert.Equal(IntRes2d_In, aT1.TransitionType()); // (0,1) ^ (1,0) < 0

        aTan1 = new gp_Vec2d(0, 0);
        IntImpParGen.DetermineTransition(IntRes2d_Head, ref aTan1, new gp_Vec2d(0, 0), ref aT1,
                                         IntRes2d_Middle, ref aTan2, new gp_Vec2d(0, 0), ref aT2, 1e-7);
        Assert.Equal(IntRes2d_Undecided, aT1.TransitionType());
        Assert.Equal(IntRes2d_Undecided, aT2.TransitionType());
    }

    [Fact]
    public void DeterminePositionAndNormalize()
    {
        var aDom = new IntRes2d_Domain(new gp_Pnt2d(0, 0), 0, 0.1, new gp_Pnt2d(1, 0), 1, 0.1);
        IntImpParGen.DeterminePosition(out IntRes2d_Position aPos, aDom, new gp_Pnt2d(0.05, 0), 0.05);
        Assert.Equal(IntRes2d_Head, aPos);
        IntImpParGen.DeterminePosition(out aPos, aDom, new gp_Pnt2d(0.5, 0), 0.5);
        Assert.Equal(IntRes2d_Middle, aPos);
        IntImpParGen.DeterminePosition(out aPos, aDom, new gp_Pnt2d(1, 0.05), 1);
        Assert.Equal(IntRes2d_End, aPos);
        // within both tolerances: the closer bound in parameter
        var aShort = new IntRes2d_Domain(new gp_Pnt2d(0, 0), 0, 1, new gp_Pnt2d(0.5, 0), 0.5, 1);
        IntImpParGen.DeterminePosition(out aPos, aShort, new gp_Pnt2d(0.3, 0), 0.3);
        Assert.Equal(IntRes2d_End, aPos);

        IntRes2d_Domain aClosed = CircleDomain();
        Assert.Equal(1.0, IntImpParGen.NormalizeOnDomain(1.0 + 4 * Math.PI, aClosed), 12);
        Assert.Equal(2 * Math.PI - 1, IntImpParGen.NormalizeOnDomain(-1.0, aClosed), 12);
        Assert.Equal(-1.0, IntImpParGen.NormalizeOnDomain(-1.0, aDom));

        // IntImpParGen_Tool's variant moves up then down by one period only
        Assert.Equal(2 * Math.PI - 1, IntImpParGen_Tool.NormalizeOnDomain(-1.0, aClosed), 12);
        Assert.Equal(1.0 + 2 * Math.PI, IntImpParGen_Tool.NormalizeOnDomain(1.0 + 4 * Math.PI, aClosed), 12);
    }

    [Fact]
    public void Intersector_SegmentCrossingCircle()
    {
        var aSeg = new TestSegment(new gp_Pnt2d(-2, 0.5), new gp_Vec2d(4, 0));
        var anInter = new CircleSegmentIntersector(theUnitCircle, CircleDomain(), aSeg, SegmentDomain(aSeg), 1e-9, 1e-9);

        Assert.True(anInter.IsDone());
        Assert.Equal(0, anInter.NbSegments());
        Assert.Equal(2, anInter.NbPoints());
        double aX = Math.Sqrt(0.75);

        // sorted by the parameter on the circle (the first curve): pi/6 (where the segment leaves) then 5pi/6
        IntRes2d_IntersectionPoint aExit = anInter.Point(1);
        Assert.Equal(Math.PI / 6, aExit.ParamOnFirst(), 9);
        Assert.Equal((2 + aX) / 4, aExit.ParamOnSecond(), 9);
        Assert.Equal(aX, aExit.Value().X(), 9);
        Assert.Equal(0.5, aExit.Value().Y(), 9);
        Assert.Equal(IntRes2d_In, aExit.TransitionOfFirst().TransitionType());
        Assert.Equal(IntRes2d_Out, aExit.TransitionOfSecond().TransitionType());
        Assert.Equal(IntRes2d_Middle, aExit.TransitionOfSecond().PositionOnCurve());

        IntRes2d_IntersectionPoint anEntry = anInter.Point(2);
        Assert.Equal(5 * Math.PI / 6, anEntry.ParamOnFirst(), 9);
        Assert.Equal((2 - aX) / 4, anEntry.ParamOnSecond(), 9);
        Assert.Equal(IntRes2d_In, anEntry.TransitionOfSecond().TransitionType());
    }

    [Fact]
    public void Intersector_ReversedParameters()
    {
        var aSeg = new TestSegment(new gp_Pnt2d(-2, 0.5), new gp_Vec2d(4, 0));
        var anInter = new CircleSegmentIntersector(theUnitCircle, CircleDomain(), aSeg, SegmentDomain(aSeg), 1e-9, 1e-9);
        var aReversed = new CircleSegmentIntersector(theUnitCircle, CircleDomain(), aSeg, SegmentDomain(aSeg), 1e-9, 1e-9);
        aReversed.SetReversedParameters(true);
        aReversed.Perform(theUnitCircle, CircleDomain(), aSeg, SegmentDomain(aSeg), 1e-9, 1e-9);

        Assert.Equal(2, aReversed.NbPoints());
        // the points are now sorted by the segment's parameter, which is reported first
        Assert.Equal(anInter.Point(2).ParamOnSecond(), aReversed.Point(1).ParamOnFirst());
        Assert.Equal(anInter.Point(2).TransitionOfSecond(), aReversed.Point(1).TransitionOfFirst());
    }

    [Fact]
    public void Intersector_SegmentEndingOnCircle()
    {
        // The segment ends at (0, 1), on the circle: one point, at the End of the segment.
        var aSeg = new TestSegment(new gp_Pnt2d(0, 0), new gp_Vec2d(0, 1));
        var anInter = new CircleSegmentIntersector(theUnitCircle, CircleDomain(), aSeg, SegmentDomain(aSeg), 1e-9, 1e-9);
        Assert.True(anInter.IsDone());
        Assert.Equal(1, anInter.NbPoints());
        Assert.Equal(Math.PI / 2, anInter.Point(1).ParamOnFirst(), 9);
        Assert.Equal(1.0, anInter.Point(1).ParamOnSecond(), 9);
        Assert.Equal(IntRes2d_End, anInter.Point(1).TransitionOfSecond().PositionOnCurve());
        Assert.Equal(IntRes2d_Out, anInter.Point(1).TransitionOfSecond().TransitionType());
    }

    [Fact]
    public void Intersector_NoIntersectionAndBadDomain()
    {
        var aSeg = new TestSegment(new gp_Pnt2d(-2, 3), new gp_Vec2d(4, 0));
        var anInter = new CircleSegmentIntersector(theUnitCircle, CircleDomain(), aSeg, SegmentDomain(aSeg), 1e-9, 1e-9);
        Assert.True(anInter.IsDone());
        Assert.True(anInter.IsEmpty());

        // the parametric curve's domain must be bounded on both sides
        Assert.ThrowsAny<Standard_ConstructionError>(
            () => new CircleSegmentIntersector(theUnitCircle, CircleDomain(), aSeg, new IntRes2d_Domain(), 1e-9, 1e-9));
    }

    [Fact]
    public void Intersector_ArcDomainRestrictsPoints()
    {
        // An open arc of the circle from 0 to pi/2: only the exit point at pi/6 is on it.
        var anArc = new IntRes2d_Domain(theUnitCircle.Value(0), 0, 1e-7, theUnitCircle.Value(Math.PI / 2),
                                        Math.PI / 2, 1e-7);
        var aSeg = new TestSegment(new gp_Pnt2d(-2, 0.5), new gp_Vec2d(4, 0));
        var anInter = new CircleSegmentIntersector(theUnitCircle, anArc, aSeg, SegmentDomain(aSeg), 1e-9, 1e-9);
        Assert.Equal(1, anInter.NbPoints());
        Assert.Equal(Math.PI / 6, anInter.Point(1).ParamOnFirst(), 9);
    }
}
