// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy;
using Kerf.Engine.Classification.Topological;
using Kerf.Engine.Foundation;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.Intersection.TwoD.Position2D;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// TopClass's generic face classifier on polygonal faces, through test instantiations of its template parameters:
/// straight edges, a segment/segment intersector built on IntImpParGen's transitions, a face explorer over a list
/// of wires, and a 2D classifier holding the fields as BRepClass_FClass2dOfFClassifier does.
/// </summary>
public class TopClassTests
{
    internal readonly record struct TestEdge(Point2D P0, Point2D P1)
    {
        public Vector2D V() => new(P0, P1);
    }

    /// <summary>Intersects the segment [0, P] of a line with a straight edge.</summary>
    internal sealed class TestIntersector : Intersection2D, IIntersector<TestEdge>
    {
        public void Perform(in Line2D L, double P, double Tol, in TestEdge E)
        {
            ResetFields();
            _done = true;
            Point2D anO = L.Location;
            Vector2D aD = new(L.Direction);
            Vector2D aV = E.V();
            double aDenom = aD.Crossed(aV);
            if (Math.Abs(aDenom) < 1e-12)
            {
                return; // parallel: ignored by this test intersector
            }

            Vector2D aW = new(anO, E.P0);
            double t = aW.Crossed(aV) / aDenom;
            double s = aW.Crossed(aD) / aDenom;
            if (t < -Tol || t > P + Tol || s < -1e-9 || s > 1 + 1e-9)
            {
                return;
            }

            Point2D aPnt = new(anO.X + t * aD.X, anO.Y + t * aD.Y);
            var aDomLin = new TwoDDomain(anO, 0, Tol, new Point2D(anO.X + P * aD.X, anO.Y + P * aD.Y), P, Tol);
            var aDomEdge = new TwoDDomain(E.P0, 0, 1e-9, E.P1, 1, 1e-9);
            global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out Position2D aPos1, aDomLin, aPnt, t);
            global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out Position2D aPos2, aDomEdge, aPnt, s);
            Transition aT1 = new(), aT2 = new();
            Vector2D aTan1 = aD, aTan2 = aV;
            global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DetermineTransition(aPos1, ref aTan1, new Vector2D(0, 0), ref aT1,
                                             aPos2, ref aTan2, new Vector2D(0, 0), ref aT2, Tol);
            Insert(new IntersectionPoint2D(aPnt, t, s, aT1, aT2, false));
        }

        public void LocalGeometry(in TestEdge E, double U, out Direction2D T, out Direction2D N, out double C)
        {
            T = new Direction2D(E.V());
            N = new Direction2D(-T.Y, T.X);
            C = 0.0;
        }
    }

    /// <summary>Explores the wires of a polygonal face; the half-lines aim at the given targets in turn.</summary>
    internal sealed class TestFaceExplorer(TestEdge[][] theWires, params Point2D[] theTargets)
        : IFaceExplorer<TestEdge>
    {
        private int myWire;
        private int myEdge;
        private int myTarget;

        public bool CheckPoint(ref Point2D thePoint) => true;

        public bool Reject(in Point2D P) => false;

        public bool Segment(in Point2D P, out Line2D L, out double Par)
        {
            myTarget = 0;
            return OtherSegment(P, out L, out Par);
        }

        public bool OtherSegment(in Point2D P, out Line2D L, out double Par)
        {
            if (myTarget >= theTargets.Length)
            {
                L = new Line2D();
                Par = 0;
                return false;
            }

            Point2D aTarget = theTargets[myTarget++];
            L = new Line2D(P, new Direction2D(new Vector2D(P, aTarget)));
            Par = 2.0 * P.DistanceTo(aTarget);
            return true;
        }

        public void InitWires() => myWire = 0;

        public bool MoreWires() => myWire < theWires.Length;

        public void NextWire() => myWire++;

        public bool RejectWire(in Line2D L, double Par) => false;

        public void InitEdges() => myEdge = 0;

        public bool MoreEdges() => myEdge < theWires[myWire].Length;

        public void NextEdge() => myEdge++;

        public bool RejectEdge(in Line2D L, double Par) => false;

        public void CurrentEdge(ref TestEdge E, out Orientation Or)
        {
            E = theWires[myWire][myEdge];
            Or = Forward;
        }
    }

    /// <summary>The 2D classifier, as BRepClass_FClass2dOfFClassifier: fields passed to TopClass_Classifier2d.</summary>
    internal sealed class TestFClass2d : IFClass2d<TestEdge>
    {
        private Line2D myLin = new();
        private double myParam;
        private double myTolerance;
        private Containment myState = Unknown;
        private bool myFirstCompare = true;
        private bool myFirstTrans = true;
        private int myClosest;
        private bool myIsSet;
        private bool myIsHeadOrEnd;
        private readonly TestIntersector myIntersector = new();
        private readonly CurveTransition myTrans = new();

        public void Reset(in Line2D L, double P, double Tol)
            => Classifier2d.Reset(out myLin, out myParam, out myTolerance, out myState, out myFirstCompare,
                                           out myFirstTrans, out myClosest, out myIsSet, out myIsHeadOrEnd, L, P, Tol);

        public void Compare(in TestEdge E, Orientation Or)
            => Classifier2d.Compare(ref myClosest, myIntersector, myLin, ref myParam, myTolerance,
                                             ref myState, ref myFirstCompare, ref myFirstTrans, myTrans,
                                             ref myIsHeadOrEnd, E, Or);

        public double Parameter() => myParam;

        public int ClosestIntersection() => myClosest;

        public Intersection2D Intersector() => myIntersector;

        public Containment State() => myState;

        public bool IsHeadOrEnd() => myIsHeadOrEnd;

        public bool IsSet() => myIsSet;
    }

    private static readonly TestEdge[] theSquare =
    [
        new(new Point2D(0, 0), new Point2D(1, 0)),
        new(new Point2D(1, 0), new Point2D(1, 1)),
        new(new Point2D(1, 1), new Point2D(0, 1)),
        new(new Point2D(0, 1), new Point2D(0, 0)),
    ];

    // A clockwise hole (0.4, 0.4)-(0.6, 0.6) in the square
    private static readonly TestEdge[] theHole =
    [
        new(new Point2D(0.4, 0.4), new Point2D(0.4, 0.6)),
        new(new Point2D(0.4, 0.6), new Point2D(0.6, 0.6)),
        new(new Point2D(0.6, 0.6), new Point2D(0.6, 0.4)),
        new(new Point2D(0.6, 0.4), new Point2D(0.4, 0.4)),
    ];

    private static (Containment State, TestEdge Edge, Position2D Position, double Parameter) Classify(
        Point2D theP, TestEdge[][] theWires, params Point2D[] theTargets)
    {
        var aClassifier = new TestFClass2d();
        var anExplorer = new TestFaceExplorer(theWires, theTargets);
        TestEdge anEdge = default;
        Position2D aPosition = Position2D.Middle;
        double aParameter = 0;
        bool aNoWires = false;
        TopologicalFaceClassifier.Perform(out bool aRejected, ref aNoWires, aClassifier, ref anEdge, ref aPosition,
                                        ref aParameter, anExplorer, theP, 1e-9);
        Assert.True(aClassifier.IsSet());
        Containment aState = TopologicalFaceClassifier.State(aRejected, aNoWires, aClassifier);
        if (aClassifier.ClosestIntersection() != 0)
        {
            Assert.Equal(aParameter, TopologicalFaceClassifier.EdgeParameter(aRejected, aParameter));
            Assert.Equal(anEdge, TopologicalFaceClassifier.Edge(aRejected, anEdge));
        }

        return (aState, anEdge, aPosition, aParameter);
    }

    [Fact]
    public void Inside_ThroughTheMiddleOfAnEdge()
    {
        var aResult = Classify(new Point2D(0.5, 0.25), [theSquare], new Point2D(1, 0.4));
        Assert.Equal(In, aResult.State);
        Assert.Equal(theSquare[1], aResult.Edge);
        Assert.Equal(Position2D.Middle, aResult.Position);
        Assert.Equal(0.4, aResult.Parameter, 12);
    }

    [Fact]
    public void Outside_TheClosestEdgeDecides()
    {
        // From (2, 0.5) towards (0.5, 0): the half-line crosses the right edge first, entering the face.
        var aResult = Classify(new Point2D(2, 0.5), [theSquare], new Point2D(0.5, 0));
        Assert.Equal(Out, aResult.State);
        Assert.Equal(theSquare[1], aResult.Edge);
    }

    [Fact]
    public void On_AnEdge()
    {
        var aResult = Classify(new Point2D(1, 0.5), [theSquare], new Point2D(0, 0.3));
        Assert.Equal(On, aResult.State);
        Assert.Equal(theSquare[1], aResult.Edge);
        Assert.Equal(0.5, aResult.Parameter, 12);
    }

    [Fact]
    public void Inside_ThroughAVertex()
    {
        // Through the corner (1, 1): two edge ends, decided by the complex transition (TopTrans).
        var aResult = Classify(new Point2D(0.5, 0.5), [theSquare], new Point2D(1, 1));
        Assert.Equal(In, aResult.State);
        Assert.True(aResult.Position is Position2D.Head or Position2D.End);
    }

    [Fact]
    public void Outside_ThroughAVertex()
    {
        var aResult = Classify(new Point2D(2, 2), [theSquare], new Point2D(1, 1));
        Assert.Equal(Out, aResult.State);
    }

    [Fact]
    public void Hole_InsideTheHoleIsOutside()
    {
        Assert.Equal(Out, Classify(new Point2D(0.5, 0.5), [theSquare, theHole], new Point2D(1, 0.55)).State);
        Assert.Equal(In, Classify(new Point2D(0.2, 0.5), [theSquare, theHole], new Point2D(1, 0.55)).State);
        Assert.Equal(In, Classify(new Point2D(0.8, 0.5), [theSquare, theHole], new Point2D(0, 0.45)).State);
    }

    [Fact]
    public void NoWires_IsInside()
    {
        Assert.Equal(In, Classify(new Point2D(5, 5), [], new Point2D(0, 0)).State);
    }

    [Fact]
    public void Rejected_IsOutside()
    {
        Assert.Equal(Out, TopologicalFaceClassifier.State(true, false, new TestFClass2d()));
        EngineAssert.ThrowsAny<DomainErrorException>(() => TopologicalFaceClassifier.EdgeParameter(true, 0.5));
        EngineAssert.ThrowsAny<DomainErrorException>(() => TopologicalFaceClassifier.Edge(true, theSquare[0]));
    }
}
