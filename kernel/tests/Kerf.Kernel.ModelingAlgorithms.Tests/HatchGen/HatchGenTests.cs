// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Classification.Hatching;
using Kerf.Engine.Foundation;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using static Kerf.Engine.Classification.Hatching.IntersectionType;
using static Kerf.Engine.Intersection.TwoD.Position2D;
using static Kerf.Engine.Intersection.TwoD.Situation;
using static Kerf.Engine.Intersection.TwoD.TypeTrans;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>HatchGen's points and domains: construction from intersection points, merging, copies.</summary>
public class HatchGenTests
{
    private static IntersectionPoint2D Crossing(TypeTrans theOnHatching, Position2D thePosE)
    {
        Transition aTrsH = new(false, Position2D.Middle, theOnHatching);
        Transition aTrsE = new(false, thePosE, theOnHatching == TypeTrans.In ? TypeTrans.Out : TypeTrans.In);
        return new IntersectionPoint2D(new Point2D(1, 2), 0.25, 0.75, aTrsH, aTrsE, false);
    }

    [Fact]
    public void PointOnElement_FromCrossings()
    {
        PointOnElement anIn = new(Crossing(TypeTrans.In, Position2D.Middle));
        Assert.Equal(0.75, anIn.Parameter());
        Assert.Equal(Internal, anIn.Position());
        Assert.Equal(global::Kerf.Topology.Containment.Out, anIn.StateBefore());
        Assert.Equal(global::Kerf.Topology.Containment.In, anIn.StateAfter());
        Assert.Equal(IntersectionType.True, anIn.IntersectionType());

        PointOnElement anOutAtEnd = new(Crossing(TypeTrans.Out, Position2D.End));
        Assert.Equal(Reversed, anOutAtEnd.Position());
        Assert.Equal(global::Kerf.Topology.Containment.In, anOutAtEnd.StateBefore());
        Assert.Equal(global::Kerf.Topology.Containment.Out, anOutAtEnd.StateAfter());
        Assert.Equal(IntersectionType.Touch, anOutAtEnd.IntersectionType());

        PointOnHatching aPntH = new(Crossing(TypeTrans.In, Position2D.Head));
        Assert.Equal(0.25, aPntH.Parameter());
        Assert.Equal(Internal, aPntH.Position());
        Assert.Equal(global::Kerf.Topology.Containment.Unknown, aPntH.StateBefore());
        Assert.Equal(0, aPntH.PointCount);
    }

    [Fact]
    public void PointOnElement_Touch()
    {
        // A touch inside at the middle of the element: IN on both sides, tangent
        Transition aTrsH = new(true, Position2D.Middle, Situation.Inside, false);
        Transition aTrsE = new(true, Position2D.Middle, Situation.Inside, false);
        PointOnElement aPntE = new(new IntersectionPoint2D(new Point2D(), 0.0, 1.0, aTrsH, aTrsE, false));
        Assert.Equal(IntersectionType.Tangent, aPntE.IntersectionType());
        Assert.Equal(global::Kerf.Topology.Containment.In, aPntE.StateBefore());
        Assert.Equal(global::Kerf.Topology.Containment.In, aPntE.StateAfter());

        // Undecided
        Transition anUndecided = new(Position2D.Head);
        PointOnElement aPntU = new(new IntersectionPoint2D(new Point2D(), 0.0, 1.0, anUndecided, anUndecided, false));
        Assert.Equal(IntersectionType.Undetermined, aPntU.IntersectionType());
        Assert.Equal(global::Kerf.Topology.Containment.Unknown, aPntU.StateBefore());
        Assert.Equal(Forward, aPntU.Position());
    }

    [Fact]
    public void PointOnHatching_MergesIdenticalPointsOnElement()
    {
        PointOnHatching aPntH = new();
        PointOnElement aPntE = new(Crossing(TypeTrans.In, Position2D.Middle));
        aPntE.SetIndex(3);
        aPntH.AddPoint(aPntE, 1.0e-8);
        PointOnElement aClose = new(aPntE);
        aClose.SetParameter(aPntE.Parameter() + 1.0e-9);
        Assert.True(aClose.IsIdentical(aPntE, 1.0e-8));
        Assert.False(aClose.IsDifferent(aPntE, 1.0e-8));
        aPntH.AddPoint(aClose, 1.0e-8);
        Assert.Equal(1, aPntH.PointCount);
        PointOnElement anOther = new(aPntE);
        anOther.SetIndex(4);
        aPntH.AddPoint(anOther, 1.0e-8);
        Assert.Equal(2, aPntH.PointCount);
        Assert.Equal(4, aPntH.Point(2).Index());

        PointOnHatching aCopy = new(aPntH);
        aCopy.RemPoint(1);
        Assert.Equal(1, aCopy.PointCount);
        Assert.Equal(2, aPntH.PointCount);
        aPntH.ClrPoints();
        Assert.Equal(0, aPntH.PointCount);
        Assert.Equal(1, aCopy.PointCount);
    }

    [Fact]
    public void PointOnHatching_Order()
    {
        PointOnHatching aP1 = new(), aP2 = new();
        aP1.SetParameter(1.0);
        aP2.SetParameter(1.0 + 1.0e-9);
        Assert.True(aP1.IsEqual(aP2, 1.0e-8));
        Assert.False(aP1.IsLower(aP2, 1.0e-8));
        aP2.SetParameter(2.0);
        Assert.True(aP1.IsLower(aP2, 1.0e-8));
        Assert.True(aP2.IsGreater(aP1, 1.0e-8));
        Assert.Equal(RealLast(), new PointOnHatching().Parameter());
        Assert.Contains("Parameter on hatching = 1", aP1.ToString());
    }

    [Fact]
    public void Domain_KeepsItsOwnPoints()
    {
        PointOnHatching aP = new();
        aP.SetParameter(1.5);
        HatchingDomain aDomain = new(aP, true);
        Assert.True(aDomain.HasFirstPoint);
        Assert.False(aDomain.HasSecondPoint);
        Assert.Throws<DomainErrorException>(() => aDomain.SecondPoint());
        aP.SetParameter(9.0); // the domain has its copy
        Assert.Equal(1.5, aDomain.FirstPoint().Parameter());

        HatchingDomain aBoth = aDomain.WithSecondPoint(aP);
        Assert.Equal(9.0, aBoth.SecondPoint().Parameter());
        Assert.False(aDomain.HasSecondPoint);
        Assert.False(aBoth.WithPoints().HasFirstPoint);
        Assert.False(new HatchingDomain().HasFirstPoint);
        Assert.False(default(HatchingDomain).HasSecondPoint);
        Assert.Equal(9.0, new HatchingDomain(aP, aP).FirstPoint().Parameter());
        Assert.False(aBoth.WithFirstPoint().HasFirstPoint);
        Assert.Contains("Has not a second point", aDomain.ToString());
    }
}
