// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Kerf.Engine.Intersection.TwoD.Position2D;
using static Kerf.Engine.Intersection.TwoD.Situation;
using static Kerf.Engine.Intersection.TwoD.TypeTrans;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// IntImpParGen's statics and its generic intersector, instantiated over test tools: an implicit circle and a
/// parametric segment (OCCT's own instantiations, over IntCurve's conic tools, are ported with IntCurve).
/// </summary>
public class IntImpParGenTests
{
    /// <summary>The circle of center <c>C</c> and radius <c>R</c>, as an implicit curve (counterclockwise from +X).</summary>
    public readonly struct TestCircleTool(Point2D C, double R) : global::Kerf.Engine.Intersection.Implicit.IImpTool
    {
        public Point2D Value(double U) => new(C.X() + R * Math.Cos(U), C.Y() + R * Math.Sin(U));

        public void D1(double U, out Point2D P, out Vector2D T)
        {
            P = Value(U);
            T = new Vector2D(-R * Math.Sin(U), R * Math.Cos(U));
        }

        public void D2(double U, out Point2D P, out Vector2D T, out Vector2D N)
        {
            D1(U, out P, out T);
            N = new Vector2D(-R * Math.Cos(U), -R * Math.Sin(U));
        }

        public double Distance(in Point2D P) => P.Distance(C) - R;

        public Vector2D GradDistance(in Point2D P)
        {
            Vector2D aV = new(C, P);
            return aV.Divided(aV.Magnitude());
        }

        public double FindParameter(in Point2D P)
        {
            double aU = Math.Atan2(P.Y() - C.Y(), P.X() - C.X());
            return aU < 0 ? aU + 2 * Math.PI : aU;
        }
    }

    /// <summary>The segment <c>P0 + t V</c>.</summary>
    public readonly record struct TestSegment(Point2D P0, Vector2D V)
    {
        public Point2D Value(double t) => new(P0.X() + t * V.X(), P0.Y() + t * V.Y());
    }

    public readonly struct TestSegmentTool : global::Kerf.Engine.Intersection.Implicit.IParTool<TestSegment>,
                                             global::Kerf.Engine.Intersection.Implicit.IProjectOnPCurveTool<TestSegment>
    {
        public static double EpsX(in TestSegment C) => 1.0e-10;

        public static int NbSamples(in TestSegment C, double U0, double U1) => 10;

        public static Point2D Value(in TestSegment C, double X) => C.Value(X);

        public static void D2(in TestSegment C, double U, out Point2D P, out Vector2D T, out Vector2D N)
        {
            P = C.Value(U);
            T = C.V;
            N = new Vector2D(0, 0);
        }

        public static double FindParameter(in TestSegment C, in Point2D Pnt, double Tol)
            => new Vector2D(C.P0, Pnt).Dot(C.V) / C.V.SquareMagnitude();

        public static double FindParameter(in TestSegment C, in Point2D Pnt, double LowParameter,
                                           double HighParameter, double Tol)
            => FindParameter(C, Pnt, Tol);
    }

    /// <summary>The signed distance to the circle of the point at a parameter on the segment.</summary>
    public sealed class TestImpParTool : FunctionWithDerivative,
                                         global::Kerf.Engine.Intersection.Implicit.IMyImpParTool<TestImpParTool, TestCircleTool, TestSegment>
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
        : global::Kerf.Engine.Intersection.Implicit.Intersector<TestCircleTool, TestSegment, TestSegmentTool, TestSegmentTool, TestImpParTool>
    {
        public CircleSegmentIntersector(in TestCircleTool Imp, in TwoDDomain D1, in TestSegment Par,
                                        in TwoDDomain D2, double TolConf, double Tol)
            : base(Imp, D1, Par, D2, TolConf, Tol)
        {
        }
    }

    private static readonly TestCircleTool theUnitCircle = new(new Point2D(0, 0), 1);

    private static TwoDDomain CircleDomain()
        => new TwoDDomain(theUnitCircle.Value(0), 0, 1e-7, theUnitCircle.Value(2 * Math.PI), 2 * Math.PI, 1e-7)
            .WithEquivalentParameters(0, 2 * Math.PI);

    private static TwoDDomain SegmentDomain(in TestSegment S, double t0 = 0, double t1 = 1)
        => new(S.Value(t0), t0, 1e-7, S.Value(t1), t1, 1e-7);

    [Fact]
    public void DetermineTransition_Crossing()
    {
        Vector2D aTan1 = new(0, 1), aTan2 = new(1, 0);
        Transition aT1 = new(), aT2 = new();
        global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DetermineTransition(Position2D.Middle, ref aTan1, new Vector2D(-1, 0), ref aT1,
                                         Position2D.Head, ref aTan2, new Vector2D(0, 0), ref aT2, 1e-7);
        // Tan1 ^ Tan2 < 0: the first curve goes in, the second out
        Assert.Equal(TypeTrans.In, aT1.TransitionType());
        Assert.Equal(TypeTrans.Out, aT2.TransitionType());
        Assert.Equal(Position2D.Head, aT2.PositionOnCurve());
        Assert.False(aT1.IsTangent);

        Assert.True(global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DetermineTransition(Position2D.Middle, ref aTan2, ref aT1, Position2D.End, ref aTan1,
                                                     ref aT2, 1e-7));
        Assert.Equal(TypeTrans.Out, aT1.TransitionType());
        Assert.Equal(TypeTrans.In, aT2.TransitionType());
    }

    [Fact]
    public void DetermineTransition_TouchByCurvature()
    {
        // A circle of curvature 1 touching a straight line from above: tangents (1,0), the circle's normal (0,1).
        Vector2D aTan1 = new(1, 0), aTan2 = new(1, 0);
        Transition aT1 = new(), aT2 = new();
        global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DetermineTransition(Position2D.Middle, ref aTan1, new Vector2D(0, 1), ref aT1,
                                         Position2D.Middle, ref aTan2, new Vector2D(0, 0), ref aT2, 1e-7);
        Assert.Equal(TypeTrans.Touch, aT1.TransitionType());
        Assert.Equal(Situation.Inside, aT1.Situation());
        Assert.Equal(Situation.Outside, aT2.Situation());
        Assert.False(aT1.IsOpposite);

        // both straight and tangent: touch, situation unknown; the opposite flag from the tangents
        aTan1 = new Vector2D(1, 0);
        aTan2 = new Vector2D(-2, 0);
        global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DetermineTransition(Position2D.Middle, ref aTan1, new Vector2D(0, 0), ref aT1,
                                         Position2D.Middle, ref aTan2, new Vector2D(0, 0), ref aT2, 1e-7);
        Assert.Equal(Situation.Unknown, aT1.Situation());
        Assert.True(aT2.IsOpposite);

        // the tangent-only form gives up on tangent curves
        Assert.False(global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DetermineTransition(Position2D.Middle, ref aTan1, ref aT1, Position2D.Middle,
                                                      ref aTan2, ref aT2, 1e-7));
    }

    [Fact]
    public void DetermineTransition_ZeroTangentUsesNormalOrIsUndecided()
    {
        Vector2D aTan1 = new(0, 0), aTan2 = new(1, 0);
        Transition aT1 = new(), aT2 = new();
        global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DetermineTransition(Position2D.Head, ref aTan1, new Vector2D(0, 1), ref aT1,
                                         Position2D.Middle, ref aTan2, new Vector2D(0, 0), ref aT2, 1e-7);
        Assert.Equal(new Vector2D(0, 1), aTan1); // the normal replaced the null tangent
        Assert.Equal(TypeTrans.In, aT1.TransitionType()); // (0,1) ^ (1,0) < 0

        aTan1 = new Vector2D(0, 0);
        global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DetermineTransition(Position2D.Head, ref aTan1, new Vector2D(0, 0), ref aT1,
                                         Position2D.Middle, ref aTan2, new Vector2D(0, 0), ref aT2, 1e-7);
        Assert.Equal(TypeTrans.Undecided, aT1.TransitionType());
        Assert.Equal(TypeTrans.Undecided, aT2.TransitionType());
    }

    [Fact]
    public void DeterminePositionAndNormalize()
    {
        var aDom = new TwoDDomain(new Point2D(0, 0), 0, 0.1, new Point2D(1, 0), 1, 0.1);
        global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out Position2D aPos, aDom, new Point2D(0.05, 0), 0.05);
        Assert.Equal(Position2D.Head, aPos);
        global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out aPos, aDom, new Point2D(0.5, 0), 0.5);
        Assert.Equal(Position2D.Middle, aPos);
        global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out aPos, aDom, new Point2D(1, 0.05), 1);
        Assert.Equal(Position2D.End, aPos);
        // within both tolerances: the closer bound in parameter
        var aShort = new TwoDDomain(new Point2D(0, 0), 0, 1, new Point2D(0.5, 0), 0.5, 1);
        global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out aPos, aShort, new Point2D(0.3, 0), 0.3);
        Assert.Equal(Position2D.End, aPos);

        TwoDDomain aClosed = CircleDomain();
        Assert.Equal(1.0, global::Kerf.Engine.Intersection.Implicit.IntImpParGen.NormalizeOnDomain(1.0 + 4 * Math.PI, aClosed), 12);
        Assert.Equal(2 * Math.PI - 1, global::Kerf.Engine.Intersection.Implicit.IntImpParGen.NormalizeOnDomain(-1.0, aClosed), 12);
        Assert.Equal(-1.0, global::Kerf.Engine.Intersection.Implicit.IntImpParGen.NormalizeOnDomain(-1.0, aDom));

        // IntImpParGen_Tool's variant moves up then down by one period only
        Assert.Equal(2 * Math.PI - 1, global::Kerf.Engine.Intersection.Implicit.ImplicitTool.NormalizeOnDomain(-1.0, aClosed), 12);
        Assert.Equal(1.0 + 2 * Math.PI, global::Kerf.Engine.Intersection.Implicit.ImplicitTool.NormalizeOnDomain(1.0 + 4 * Math.PI, aClosed), 12);
    }

    [Fact]
    public void Intersector_SegmentCrossingCircle()
    {
        var aSeg = new TestSegment(new Point2D(-2, 0.5), new Vector2D(4, 0));
        var anInter = new CircleSegmentIntersector(theUnitCircle, CircleDomain(), aSeg, SegmentDomain(aSeg), 1e-9, 1e-9);

        Assert.True(anInter.IsDone);
        Assert.Equal(0, anInter.SegmentCount);
        Assert.Equal(2, anInter.PointCount);
        double aX = Math.Sqrt(0.75);

        // sorted by the parameter on the circle (the first curve): pi/6 (where the segment leaves) then 5pi/6
        IntersectionPoint2D aExit = anInter.Point(1);
        Assert.Equal(Math.PI / 6, aExit.ParamOnFirst(), 9);
        Assert.Equal((2 + aX) / 4, aExit.ParamOnSecond(), 9);
        Assert.Equal(aX, aExit.Value().X(), 9);
        Assert.Equal(0.5, aExit.Value().Y(), 9);
        Assert.Equal(TypeTrans.In, aExit.TransitionOfFirst().TransitionType());
        Assert.Equal(TypeTrans.Out, aExit.TransitionOfSecond().TransitionType());
        Assert.Equal(Position2D.Middle, aExit.TransitionOfSecond().PositionOnCurve());

        IntersectionPoint2D anEntry = anInter.Point(2);
        Assert.Equal(5 * Math.PI / 6, anEntry.ParamOnFirst(), 9);
        Assert.Equal((2 - aX) / 4, anEntry.ParamOnSecond(), 9);
        Assert.Equal(TypeTrans.In, anEntry.TransitionOfSecond().TransitionType());
    }

    [Fact]
    public void Intersector_ReversedParameters()
    {
        var aSeg = new TestSegment(new Point2D(-2, 0.5), new Vector2D(4, 0));
        var anInter = new CircleSegmentIntersector(theUnitCircle, CircleDomain(), aSeg, SegmentDomain(aSeg), 1e-9, 1e-9);
        var aReversed = new CircleSegmentIntersector(theUnitCircle, CircleDomain(), aSeg, SegmentDomain(aSeg), 1e-9, 1e-9);
        aReversed.SetReversedParameters(true);
        aReversed.Perform(theUnitCircle, CircleDomain(), aSeg, SegmentDomain(aSeg), 1e-9, 1e-9);

        Assert.Equal(2, aReversed.PointCount);
        // the points are now sorted by the segment's parameter, which is reported first
        Assert.Equal(anInter.Point(2).ParamOnSecond(), aReversed.Point(1).ParamOnFirst());
        Assert.Equal(anInter.Point(2).TransitionOfSecond(), aReversed.Point(1).TransitionOfFirst());
    }

    [Fact]
    public void Intersector_SegmentEndingOnCircle()
    {
        // The segment ends at (0, 1), on the circle: one point, at the End of the segment.
        var aSeg = new TestSegment(new Point2D(0, 0), new Vector2D(0, 1));
        var anInter = new CircleSegmentIntersector(theUnitCircle, CircleDomain(), aSeg, SegmentDomain(aSeg), 1e-9, 1e-9);
        Assert.True(anInter.IsDone);
        Assert.Equal(1, anInter.PointCount);
        Assert.Equal(Math.PI / 2, anInter.Point(1).ParamOnFirst(), 9);
        Assert.Equal(1.0, anInter.Point(1).ParamOnSecond(), 9);
        Assert.Equal(Position2D.End, anInter.Point(1).TransitionOfSecond().PositionOnCurve());
        Assert.Equal(TypeTrans.Out, anInter.Point(1).TransitionOfSecond().TransitionType());
    }

    [Fact]
    public void Intersector_NoIntersectionAndBadDomain()
    {
        var aSeg = new TestSegment(new Point2D(-2, 3), new Vector2D(4, 0));
        var anInter = new CircleSegmentIntersector(theUnitCircle, CircleDomain(), aSeg, SegmentDomain(aSeg), 1e-9, 1e-9);
        Assert.True(anInter.IsDone);
        Assert.True(anInter.IsEmpty);

        // the parametric curve's domain must be bounded on both sides
        Assert.ThrowsAny<ConstructionErrorException>(
            () => new CircleSegmentIntersector(theUnitCircle, CircleDomain(), aSeg, new TwoDDomain(), 1e-9, 1e-9));
    }

    [Fact]
    public void Intersector_ArcDomainRestrictsPoints()
    {
        // An open arc of the circle from 0 to pi/2: only the exit point at pi/6 is on it.
        var anArc = new TwoDDomain(theUnitCircle.Value(0), 0, 1e-7, theUnitCircle.Value(Math.PI / 2),
                                        Math.PI / 2, 1e-7);
        var aSeg = new TestSegment(new Point2D(-2, 0.5), new Vector2D(4, 0));
        var anInter = new CircleSegmentIntersector(theUnitCircle, anArc, aSeg, SegmentDomain(aSeg), 1e-9, 1e-9);
        Assert.Equal(1, anInter.PointCount);
        Assert.Equal(Math.PI / 6, anInter.Point(1).ParamOnFirst(), 9);
    }
}
