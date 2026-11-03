// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Occt.HatchGen_IntersectionType;
using static Occt.IntRes2d_Position;
using static Occt.IntRes2d_Situation;
using static Occt.IntRes2d_TypeTrans;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>HatchGen's points and domains: construction from intersection points, merging, copies.</summary>
public class HatchGenTests
{
    private static IntRes2d_IntersectionPoint Crossing(IntRes2d_TypeTrans theOnHatching, IntRes2d_Position thePosE)
    {
        IntRes2d_Transition aTrsH = new(false, IntRes2d_Middle, theOnHatching);
        IntRes2d_Transition aTrsE = new(false, thePosE, theOnHatching == IntRes2d_In ? IntRes2d_Out : IntRes2d_In);
        return new IntRes2d_IntersectionPoint(new Point2D(1, 2), 0.25, 0.75, aTrsH, aTrsE, false);
    }

    [Fact]
    public void PointOnElement_FromCrossings()
    {
        HatchGen_PointOnElement anIn = new(Crossing(IntRes2d_In, IntRes2d_Middle));
        Assert.Equal(0.75, anIn.Parameter());
        Assert.Equal(Internal, anIn.Position());
        Assert.Equal(Out, anIn.StateBefore());
        Assert.Equal(In, anIn.StateAfter());
        Assert.Equal(HatchGen_TRUE, anIn.IntersectionType());

        HatchGen_PointOnElement anOutAtEnd = new(Crossing(IntRes2d_Out, IntRes2d_End));
        Assert.Equal(Reversed, anOutAtEnd.Position());
        Assert.Equal(In, anOutAtEnd.StateBefore());
        Assert.Equal(Out, anOutAtEnd.StateAfter());
        Assert.Equal(HatchGen_TOUCH, anOutAtEnd.IntersectionType());

        HatchGen_PointOnHatching aPntH = new(Crossing(IntRes2d_In, IntRes2d_Head));
        Assert.Equal(0.25, aPntH.Parameter());
        Assert.Equal(Internal, aPntH.Position());
        Assert.Equal(Unknown, aPntH.StateBefore());
        Assert.Equal(0, aPntH.NbPoints());
    }

    [Fact]
    public void PointOnElement_Touch()
    {
        // A touch inside at the middle of the element: IN on both sides, tangent
        IntRes2d_Transition aTrsH = new(true, IntRes2d_Middle, IntRes2d_Inside, false);
        IntRes2d_Transition aTrsE = new(true, IntRes2d_Middle, IntRes2d_Inside, false);
        HatchGen_PointOnElement aPntE = new(new IntRes2d_IntersectionPoint(new Point2D(), 0.0, 1.0, aTrsH, aTrsE, false));
        Assert.Equal(HatchGen_TANGENT, aPntE.IntersectionType());
        Assert.Equal(In, aPntE.StateBefore());
        Assert.Equal(In, aPntE.StateAfter());

        // Undecided
        IntRes2d_Transition anUndecided = new(IntRes2d_Head);
        HatchGen_PointOnElement aPntU = new(new IntRes2d_IntersectionPoint(new Point2D(), 0.0, 1.0, anUndecided, anUndecided, false));
        Assert.Equal(HatchGen_UNDETERMINED, aPntU.IntersectionType());
        Assert.Equal(Unknown, aPntU.StateBefore());
        Assert.Equal(Forward, aPntU.Position());
    }

    [Fact]
    public void PointOnHatching_MergesIdenticalPointsOnElement()
    {
        HatchGen_PointOnHatching aPntH = new();
        HatchGen_PointOnElement aPntE = new(Crossing(IntRes2d_In, IntRes2d_Middle));
        aPntE.SetIndex(3);
        aPntH.AddPoint(aPntE, 1.0e-8);
        HatchGen_PointOnElement aClose = new(aPntE);
        aClose.SetParameter(aPntE.Parameter() + 1.0e-9);
        Assert.True(aClose.IsIdentical(aPntE, 1.0e-8));
        Assert.False(aClose.IsDifferent(aPntE, 1.0e-8));
        aPntH.AddPoint(aClose, 1.0e-8);
        Assert.Equal(1, aPntH.NbPoints());
        HatchGen_PointOnElement anOther = new(aPntE);
        anOther.SetIndex(4);
        aPntH.AddPoint(anOther, 1.0e-8);
        Assert.Equal(2, aPntH.NbPoints());
        Assert.Equal(4, aPntH.Point(2).Index());

        HatchGen_PointOnHatching aCopy = new(aPntH);
        aCopy.RemPoint(1);
        Assert.Equal(1, aCopy.NbPoints());
        Assert.Equal(2, aPntH.NbPoints());
        aPntH.ClrPoints();
        Assert.Equal(0, aPntH.NbPoints());
        Assert.Equal(1, aCopy.NbPoints());
    }

    [Fact]
    public void PointOnHatching_Order()
    {
        HatchGen_PointOnHatching aP1 = new(), aP2 = new();
        aP1.SetParameter(1.0);
        aP2.SetParameter(1.0 + 1.0e-9);
        Assert.True(aP1.IsEqual(aP2, 1.0e-8));
        Assert.False(aP1.IsLower(aP2, 1.0e-8));
        aP2.SetParameter(2.0);
        Assert.True(aP1.IsLower(aP2, 1.0e-8));
        Assert.True(aP2.IsGreater(aP1, 1.0e-8));
        Assert.Equal(RealLast(), new HatchGen_PointOnHatching().Parameter());
        Assert.Contains("Parameter on hatching = 1", aP1.ToString());
    }

    [Fact]
    public void Domain_KeepsItsOwnPoints()
    {
        HatchGen_PointOnHatching aP = new();
        aP.SetParameter(1.5);
        HatchGen_Domain aDomain = new(aP, true);
        Assert.True(aDomain.HasFirstPoint());
        Assert.False(aDomain.HasSecondPoint());
        Assert.Throws<DomainErrorException>(() => aDomain.SecondPoint());
        aP.SetParameter(9.0); // the domain has its copy
        Assert.Equal(1.5, aDomain.FirstPoint().Parameter());

        HatchGen_Domain aBoth = aDomain.WithSecondPoint(aP);
        Assert.Equal(9.0, aBoth.SecondPoint().Parameter());
        Assert.False(aDomain.HasSecondPoint());
        Assert.False(aBoth.WithPoints().HasFirstPoint());
        Assert.False(new HatchGen_Domain().HasFirstPoint());
        Assert.False(default(HatchGen_Domain).HasSecondPoint());
        Assert.Equal(9.0, new HatchGen_Domain(aP, aP).FirstPoint().Parameter());
        Assert.False(aBoth.WithFirstPoint().HasFirstPoint());
        Assert.Contains("Has not a second point", aDomain.ToString());
    }
}
