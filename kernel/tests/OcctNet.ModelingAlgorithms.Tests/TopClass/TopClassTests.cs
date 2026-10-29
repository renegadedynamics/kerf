// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.IntRes2d_Position;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_State;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// TopClass's generic face classifier on polygonal faces, through test instantiations of its template parameters:
/// straight edges, a segment/segment intersector built on IntImpParGen's transitions, a face explorer over a list
/// of wires, and a 2D classifier holding the fields as BRepClass_FClass2dOfFClassifier does.
/// </summary>
public class TopClassTests
{
    internal readonly record struct TestEdge(gp_Pnt2d P0, gp_Pnt2d P1)
    {
        public gp_Vec2d V() => new(P0, P1);
    }

    /// <summary>Intersects the segment [0, P] of a line with a straight edge.</summary>
    internal sealed class TestIntersector : IntRes2d_Intersection, ITopClass_Intersector<TestEdge>
    {
        public void Perform(in gp_Lin2d L, double P, double Tol, in TestEdge E)
        {
            ResetFields();
            done = true;
            gp_Pnt2d anO = L.Location();
            gp_Vec2d aD = new(L.Direction());
            gp_Vec2d aV = E.V();
            double aDenom = aD.Crossed(aV);
            if (Math.Abs(aDenom) < 1e-12)
            {
                return; // parallel: ignored by this test intersector
            }

            gp_Vec2d aW = new(anO, E.P0);
            double t = aW.Crossed(aV) / aDenom;
            double s = aW.Crossed(aD) / aDenom;
            if (t < -Tol || t > P + Tol || s < -1e-9 || s > 1 + 1e-9)
            {
                return;
            }

            gp_Pnt2d aPnt = new(anO.X() + t * aD.X(), anO.Y() + t * aD.Y());
            var aDomLin = new IntRes2d_Domain(anO, 0, Tol, new gp_Pnt2d(anO.X() + P * aD.X(), anO.Y() + P * aD.Y()), P, Tol);
            var aDomEdge = new IntRes2d_Domain(E.P0, 0, 1e-9, E.P1, 1, 1e-9);
            IntImpParGen.DeterminePosition(out IntRes2d_Position aPos1, aDomLin, aPnt, t);
            IntImpParGen.DeterminePosition(out IntRes2d_Position aPos2, aDomEdge, aPnt, s);
            IntRes2d_Transition aT1 = new(), aT2 = new();
            gp_Vec2d aTan1 = aD, aTan2 = aV;
            IntImpParGen.DetermineTransition(aPos1, ref aTan1, new gp_Vec2d(0, 0), ref aT1,
                                             aPos2, ref aTan2, new gp_Vec2d(0, 0), ref aT2, Tol);
            Insert(new IntRes2d_IntersectionPoint(aPnt, t, s, aT1, aT2, false));
        }

        public void LocalGeometry(in TestEdge E, double U, out gp_Dir2d T, out gp_Dir2d N, out double C)
        {
            T = new gp_Dir2d(E.V());
            N = new gp_Dir2d(-T.Y(), T.X());
            C = 0.0;
        }
    }

    /// <summary>Explores the wires of a polygonal face; the half-lines aim at the given targets in turn.</summary>
    internal sealed class TestFaceExplorer(TestEdge[][] theWires, params gp_Pnt2d[] theTargets)
        : ITopClass_FaceExplorer<TestEdge>
    {
        private int myWire;
        private int myEdge;
        private int myTarget;

        public bool CheckPoint(ref gp_Pnt2d thePoint) => true;

        public bool Reject(in gp_Pnt2d P) => false;

        public bool Segment(in gp_Pnt2d P, out gp_Lin2d L, out double Par)
        {
            myTarget = 0;
            return OtherSegment(P, out L, out Par);
        }

        public bool OtherSegment(in gp_Pnt2d P, out gp_Lin2d L, out double Par)
        {
            if (myTarget >= theTargets.Length)
            {
                L = new gp_Lin2d();
                Par = 0;
                return false;
            }

            gp_Pnt2d aTarget = theTargets[myTarget++];
            L = new gp_Lin2d(P, new gp_Dir2d(new gp_Vec2d(P, aTarget)));
            Par = 2.0 * P.Distance(aTarget);
            return true;
        }

        public void InitWires() => myWire = 0;

        public bool MoreWires() => myWire < theWires.Length;

        public void NextWire() => myWire++;

        public bool RejectWire(in gp_Lin2d L, double Par) => false;

        public void InitEdges() => myEdge = 0;

        public bool MoreEdges() => myEdge < theWires[myWire].Length;

        public void NextEdge() => myEdge++;

        public bool RejectEdge(in gp_Lin2d L, double Par) => false;

        public void CurrentEdge(ref TestEdge E, out TopAbs_Orientation Or)
        {
            E = theWires[myWire][myEdge];
            Or = TopAbs_FORWARD;
        }
    }

    /// <summary>The 2D classifier, as BRepClass_FClass2dOfFClassifier: fields passed to TopClass_Classifier2d.</summary>
    internal sealed class TestFClass2d : ITopClass_FClass2d<TestEdge>
    {
        private gp_Lin2d myLin = new();
        private double myParam;
        private double myTolerance;
        private TopAbs_State myState = TopAbs_UNKNOWN;
        private bool myFirstCompare = true;
        private bool myFirstTrans = true;
        private int myClosest;
        private bool myIsSet;
        private bool myIsHeadOrEnd;
        private readonly TestIntersector myIntersector = new();
        private readonly TopTrans_CurveTransition myTrans = new();

        public void Reset(in gp_Lin2d L, double P, double Tol)
            => TopClass_Classifier2d.Reset(out myLin, out myParam, out myTolerance, out myState, out myFirstCompare,
                                           out myFirstTrans, out myClosest, out myIsSet, out myIsHeadOrEnd, L, P, Tol);

        public void Compare(in TestEdge E, TopAbs_Orientation Or)
            => TopClass_Classifier2d.Compare(ref myClosest, myIntersector, myLin, ref myParam, myTolerance,
                                             ref myState, ref myFirstCompare, ref myFirstTrans, myTrans,
                                             ref myIsHeadOrEnd, E, Or);

        public double Parameter() => myParam;

        public int ClosestIntersection() => myClosest;

        public IntRes2d_Intersection Intersector() => myIntersector;

        public TopAbs_State State() => myState;

        public bool IsHeadOrEnd() => myIsHeadOrEnd;

        public bool IsSet() => myIsSet;
    }

    private static readonly TestEdge[] theSquare =
    [
        new(new gp_Pnt2d(0, 0), new gp_Pnt2d(1, 0)),
        new(new gp_Pnt2d(1, 0), new gp_Pnt2d(1, 1)),
        new(new gp_Pnt2d(1, 1), new gp_Pnt2d(0, 1)),
        new(new gp_Pnt2d(0, 1), new gp_Pnt2d(0, 0)),
    ];

    // A clockwise hole (0.4, 0.4)-(0.6, 0.6) in the square
    private static readonly TestEdge[] theHole =
    [
        new(new gp_Pnt2d(0.4, 0.4), new gp_Pnt2d(0.4, 0.6)),
        new(new gp_Pnt2d(0.4, 0.6), new gp_Pnt2d(0.6, 0.6)),
        new(new gp_Pnt2d(0.6, 0.6), new gp_Pnt2d(0.6, 0.4)),
        new(new gp_Pnt2d(0.6, 0.4), new gp_Pnt2d(0.4, 0.4)),
    ];

    private static (TopAbs_State State, TestEdge Edge, IntRes2d_Position Position, double Parameter) Classify(
        gp_Pnt2d theP, TestEdge[][] theWires, params gp_Pnt2d[] theTargets)
    {
        var aClassifier = new TestFClass2d();
        var anExplorer = new TestFaceExplorer(theWires, theTargets);
        TestEdge anEdge = default;
        IntRes2d_Position aPosition = IntRes2d_Middle;
        double aParameter = 0;
        bool aNoWires = false;
        TopClass_FaceClassifier.Perform(out bool aRejected, ref aNoWires, aClassifier, ref anEdge, ref aPosition,
                                        ref aParameter, anExplorer, theP, 1e-9);
        Assert.True(aClassifier.IsSet());
        TopAbs_State aState = TopClass_FaceClassifier.State(aRejected, aNoWires, aClassifier);
        if (aClassifier.ClosestIntersection() != 0)
        {
            Assert.Equal(aParameter, TopClass_FaceClassifier.EdgeParameter(aRejected, aParameter));
            Assert.Equal(anEdge, TopClass_FaceClassifier.Edge(aRejected, anEdge));
        }

        return (aState, anEdge, aPosition, aParameter);
    }

    [Fact]
    public void Inside_ThroughTheMiddleOfAnEdge()
    {
        var aResult = Classify(new gp_Pnt2d(0.5, 0.25), [theSquare], new gp_Pnt2d(1, 0.4));
        Assert.Equal(TopAbs_IN, aResult.State);
        Assert.Equal(theSquare[1], aResult.Edge);
        Assert.Equal(IntRes2d_Middle, aResult.Position);
        Assert.Equal(0.4, aResult.Parameter, 12);
    }

    [Fact]
    public void Outside_TheClosestEdgeDecides()
    {
        // From (2, 0.5) towards (0.5, 0): the half-line crosses the right edge first, entering the face.
        var aResult = Classify(new gp_Pnt2d(2, 0.5), [theSquare], new gp_Pnt2d(0.5, 0));
        Assert.Equal(TopAbs_OUT, aResult.State);
        Assert.Equal(theSquare[1], aResult.Edge);
    }

    [Fact]
    public void On_AnEdge()
    {
        var aResult = Classify(new gp_Pnt2d(1, 0.5), [theSquare], new gp_Pnt2d(0, 0.3));
        Assert.Equal(TopAbs_ON, aResult.State);
        Assert.Equal(theSquare[1], aResult.Edge);
        Assert.Equal(0.5, aResult.Parameter, 12);
    }

    [Fact]
    public void Inside_ThroughAVertex()
    {
        // Through the corner (1, 1): two edge ends, decided by the complex transition (TopTrans).
        var aResult = Classify(new gp_Pnt2d(0.5, 0.5), [theSquare], new gp_Pnt2d(1, 1));
        Assert.Equal(TopAbs_IN, aResult.State);
        Assert.True(aResult.Position is IntRes2d_Head or IntRes2d_End);
    }

    [Fact]
    public void Outside_ThroughAVertex()
    {
        var aResult = Classify(new gp_Pnt2d(2, 2), [theSquare], new gp_Pnt2d(1, 1));
        Assert.Equal(TopAbs_OUT, aResult.State);
    }

    [Fact]
    public void Hole_InsideTheHoleIsOutside()
    {
        Assert.Equal(TopAbs_OUT, Classify(new gp_Pnt2d(0.5, 0.5), [theSquare, theHole], new gp_Pnt2d(1, 0.55)).State);
        Assert.Equal(TopAbs_IN, Classify(new gp_Pnt2d(0.2, 0.5), [theSquare, theHole], new gp_Pnt2d(1, 0.55)).State);
        Assert.Equal(TopAbs_IN, Classify(new gp_Pnt2d(0.8, 0.5), [theSquare, theHole], new gp_Pnt2d(0, 0.45)).State);
    }

    [Fact]
    public void NoWires_IsInside()
    {
        Assert.Equal(TopAbs_IN, Classify(new gp_Pnt2d(5, 5), [], new gp_Pnt2d(0, 0)).State);
    }

    [Fact]
    public void Rejected_IsOutside()
    {
        Assert.Equal(TopAbs_OUT, TopClass_FaceClassifier.State(true, false, new TestFClass2d()));
        Assert.ThrowsAny<Standard_DomainError>(() => TopClass_FaceClassifier.EdgeParameter(true, 0.5));
        Assert.ThrowsAny<Standard_DomainError>(() => TopClass_FaceClassifier.Edge(true, theSquare[0]));
    }
}
