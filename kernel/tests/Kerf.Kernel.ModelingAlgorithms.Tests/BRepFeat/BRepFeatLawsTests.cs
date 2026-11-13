// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Features;
using Kerf.Engine.Features.Local;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Kernel.ModelingData.Tests;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// BRepFeat's makers by their limits (decision 1753): every way of ending a prism, a draft prism, a revolution and a
/// pipe, and every way of drilling a hole. The expected volumes are geometry's, not recordings: a prism is area times
/// height, a revolution or a pipe along an arc is Pappus's area times centroid path.
/// </summary>
public class BRepFeatLawsTests
{
    private const double Area = 24; // of the 4 x 6 profile
    private const double BoxVolume = 6000;
    private static readonly Direction3D Up = new(0, 0, 1);
    private static readonly Direction3D Down = new(0, 0, -1);

    private static double Volume(in Shape theShape) => LocOpeTests.Volume(theShape);

    private static int Count(in Shape theShape, ShapeType theType) => LocOpeTests.Count(theShape, theType);

    private static (Shape Box, Face Top, Face Profile) Scene()
    {
        Shape aBox = LocOpeTests.Box();
        return (aBox, LocOpeTests.TopFace(aBox), BRepFeatTests.Rect(3, 7, 7, 13, 30));
    }

    // A plane face far wider than the box, at a height
    private static Face Level(double theZ) => BRepFeatTests.Rect(-50, 60, -50, 70, theZ);

    // A plane face through the axis along x, leaving it at the angle above +y
    private static Face HalfPlane(in Axis3D theAxis, double theAngle)
    {
        Point3D anO = theAxis.Location;
        const double aLength = 200;
        double aY = aLength * Math.Cos(theAngle), aZ = aLength * Math.Sin(theAngle);
        var aPoly = new MakePolygon(new Point3D(-100, anO.Y, anO.Z), new Point3D(110, anO.Y, anO.Z),
                                    new Point3D(110, anO.Y + aY, anO.Z + aZ), new Point3D(-100, anO.Y + aY, anO.Z + aZ), true);
        return new MakeFace(aPoly.Wire(), true).Face();
    }

    private static Face FaceAt(in Shape theShape, Func<Point3D, bool> theWhere)
    {
        for (var anExp = new Explorer(theShape, ShapeType.Face); anExp.More(); anExp.Next())
        {
            var aProps = new Properties();
            MassPropertiesBuilder.SurfaceProperties(anExp.Current(), aProps);
            if (theWhere(aProps.CentreOfMass()))
            {
                return ShapeCasts.Face(anExp.Current());
            }
        }
        throw new InvalidOperationException("no such face");
    }

    private static Edge EdgeAt(in Shape theShape, Func<Point3D, bool> theWhere)
    {
        for (var anExp = new Explorer(theShape, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            var aProps = new Properties();
            MassPropertiesBuilder.LinearProperties(anExp.Current(), aProps);
            if (theWhere(aProps.CentreOfMass()))
            {
                return ShapeCasts.Edge(anExp.Current());
            }
        }
        throw new InvalidOperationException("no such edge");
    }

    private static (double Min, double Max) ZRange(in Shape theShape)
    {
        double aMin = double.MaxValue, aMax = double.MinValue;
        for (var anExp = new Explorer(theShape, ShapeType.Vertex); anExp.More(); anExp.Next())
        {
            double aZ = Tool.Pnt(ShapeCasts.Vertex(anExp.Current())).Z;
            aMin = Math.Min(aMin, aZ);
            aMax = Math.Max(aMax, aZ);
        }
        return (aMin, aMax);
    }

    private static void AssertSound(in Shape theResult)
    {
        Assert.False(theResult.IsNull);
        Assert.True(new Analyzer(theResult).IsValid());
        Assert.True(LocOpeTests.AllFrozen(theResult));
    }

    public enum Limit
    {
        Length, Until, FromUntil, UntilFrom, UntilEnd, FromEnd, ThruAll, UntilHeight,
    }

    private static void Perform(MakePrism thePrism, Limit theLimit, double theFrom, double theUntil)
    {
        switch (theLimit)
        {
            case Limit.Length: thePrism.Perform(5.0); break;
            case Limit.Until: thePrism.Perform(Level(theUntil)); break;
            case Limit.FromUntil: thePrism.Perform(Level(theFrom), Level(theUntil)); break;
            case Limit.UntilFrom: thePrism.Perform(Level(theUntil), Level(theFrom)); break;
            case Limit.UntilEnd: thePrism.PerformUntilEnd(); break;
            case Limit.FromEnd: thePrism.PerformFromEnd(Level(theUntil)); break;
            case Limit.ThruAll: thePrism.PerformThruAll(); break;
            case Limit.UntilHeight: thePrism.PerformUntilHeight(Level(theUntil), 7); break;
        }
    }

    [Theory]
    [InlineData(Limit.Length, 5 * Area)]
    [InlineData(Limit.Until, 20 * Area)]
    [InlineData(Limit.FromUntil, 15 * Area)]
    [InlineData(Limit.UntilFrom, 15 * Area)]
    [InlineData(Limit.UntilEnd, 30 * Area)]
    [InlineData(Limit.FromEnd, 20 * Area)]
    [InlineData(Limit.ThruAll, 30 * Area)]
    [InlineData(Limit.UntilHeight, 7 * Area)]
    public void A_cut_prism_removes_its_area_times_the_height_between_its_limits(Limit theLimit, double theRemoved)
    {
        foreach (bool aModify in new[] { true, false })
        {
            (Shape aBox, Face aTop, Face aProfile) = Scene();
            ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
            var aPrism = new MakePrism(aBox, aProfile, aTop, Down, 0, aModify);
            Perform(aPrism, theLimit, 25, 10);
            Assert.True(aPrism.IsDone);
            AssertSound(aPrism.Shape());
            Assert.Equal(BoxVolume - theRemoved, Volume(aPrism.Shape()), 6);
            aSnap.AssertUnchanged(aBox);
        }
    }

    [Theory]
    [InlineData(Limit.Length, 5 * Area, 1)]
    [InlineData(Limit.Until, 8 * Area, 1)]
    [InlineData(Limit.FromUntil, 5 * Area, 2)] // from 33 to 38: above the box, so apart from it
    [InlineData(Limit.UntilFrom, 5 * Area, 2)]
    [InlineData(Limit.UntilHeight, 7 * Area, 1)]
    public void A_fused_prism_adds_its_area_times_the_height_between_its_limits(Limit theLimit, double theAdded, int theSolids)
    {
        (Shape aBox, Face aTop, Face aProfile) = Scene();
        var aPrism = new MakePrism(aBox, aProfile, aTop, Up, 1, true);
        Perform(aPrism, theLimit, 33, 38);
        Assert.True(aPrism.IsDone);
        AssertSound(aPrism.Shape());
        Assert.Equal(BoxVolume + theAdded, Volume(aPrism.Shape()), 6);
        Assert.Equal(theSolids, Count(aPrism.Shape(), ShapeType.Solid));
    }

    [Theory]
    [InlineData(Limit.ThruAll)]
    [InlineData(Limit.UntilEnd)]
    public void A_fused_prism_without_a_limit_of_its_own_is_refused(Limit theLimit)
    {
        (Shape aBox, Face aTop, Face aProfile) = Scene();
        var aPrism = new MakePrism(aBox, aProfile, aTop, Up, 1, true);
        Perform(aPrism, theLimit, 0, 0);
        Assert.False(aPrism.IsDone);
    }

    [Fact]
    public void A_fused_prism_from_the_end_runs_back_through_the_box_and_beyond()
    {
        (Shape aBox, Face aTop, Face aProfile) = Scene();
        var aPrism = new MakePrism(aBox, aProfile, aTop, Up, 1, true);
        aPrism.PerformFromEnd(Level(38));
        Assert.True(aPrism.IsDone);
        Shape aRes = aPrism.Shape();
        AssertSound(aRes);
        // the box, the profile up to the limit, and the profile below the box as far as the prism's own length reaches
        (double aMin, double aMax) = ZRange(aRes);
        Assert.Equal(38, aMax, 9);
        Assert.True(aMin < 0);
        Assert.Equal(BoxVolume + Area * (38 - 30 - aMin), Volume(aRes), 4);
    }

    [Fact]
    public void A_feature_on_its_own_holds_only_the_prism()
    {
        (Shape aBox, Face aTop, Face aProfile) = Scene();
        var aPrism = new MakePrism(aBox, aProfile, aTop, Up, 2, true);
        aPrism.Perform(5.0);
        Assert.Equal(5 * Area, Volume(aPrism.Shape()), 6);
        Assert.Equal(1, Count(aPrism.Shape(), ShapeType.Solid));
    }

    [Fact]
    public void The_limits_must_be_shapes_with_faces()
    {
        (Shape aBox, Face aTop, Face aProfile) = Scene();
        var aPrism = new MakePrism(aBox, aProfile, aTop, Up, 1, true);
        var aVertex = new MakeVertex(new Point3D(0, 0, 40)).Vertex();
        Assert.Throws<ConstructionErrorException>(() => aPrism.Perform(new Shape()));
        Assert.Throws<ConstructionErrorException>(() => aPrism.Perform(aVertex));
        Assert.Throws<ConstructionErrorException>(() => aPrism.Perform(new Shape(), Level(38)));
        Assert.Throws<ConstructionErrorException>(() => aPrism.Perform(Level(33), new Shape()));
        Assert.Throws<ConstructionErrorException>(() => aPrism.Perform(aVertex, Level(38)));
        Assert.Throws<ConstructionErrorException>(() => aPrism.Perform(Level(33), aVertex));
        Assert.Throws<ConstructionErrorException>(() => aPrism.PerformFromEnd(new Shape()));
        Assert.Throws<ConstructionErrorException>(() => aPrism.PerformFromEnd(aVertex));
        Assert.Throws<ConstructionErrorException>(() => aPrism.PerformUntilHeight(aVertex, 7));
        // no Until at all is a prism of the height (decision 2352; 8.0.1 threw here too)
        aPrism.PerformUntilHeight(new Shape(), 7);
        Assert.True(aPrism.IsDone);
    }

    [Theory]
    [InlineData(0, -1)]
    [InlineData(1, 1)]
    public void A_prism_sliding_on_a_face_keeps_the_same_volume_and_loses_the_shared_side(int theMode, double theSign)
    {
        Shape aBox = LocOpeTests.Box();
        Face aTop = LocOpeTests.TopFace(aBox);
        Face aProfile = BRepFeatTests.Rect(0, 4, 7, 13, 30); // flush with the side x = 0
        Face aSide = FaceAt(aBox, p => Math.Abs(p.X) < 1e-9);
        Edge anEdge = EdgeAt(aProfile, p => Math.Abs(p.X) < 1e-9);
        var aPrism = new MakePrism(aBox, aProfile, aTop, theSign > 0 ? Up : Down, theMode, true);
        // the wrong way round: neither the edge of the box nor a face of the profile
        Assert.Throws<ConstructionErrorException>(() => aPrism.Add(anEdge, aProfile));
        Assert.Throws<ConstructionErrorException>(() => aPrism.Add(EdgeAt(aBox, _ => true), aSide));
        aPrism.Add(anEdge, aSide);
        aPrism.Add(anEdge, aSide); // the second time changes nothing
        aPrism.Perform(5.0);
        Shape aRes = aPrism.Shape();
        AssertSound(aRes);
        Assert.Equal(BoxVolume + theSign * 5 * Area, Volume(aRes), 6);
    }

    [Theory]
    [InlineData(0, -1)]
    [InlineData(1, 1)]
    public void A_prism_from_a_side_sliding_under_the_top_face(int theMode, double theSign)
    {
        Shape aBox = LocOpeTests.Box();
        Face aSide = FaceAt(aBox, p => Math.Abs(p.X) < 1e-9);
        Face aTop = LocOpeTests.TopFace(aBox);
        var aPoly = new MakePolygon(new Point3D(0, 7, 30), new Point3D(0, 7, 20), new Point3D(0, 13, 20), new Point3D(0, 13, 30), true);
        Face aProfile = new MakeFace(aPoly.Wire(), true).Face();
        var aPrism = new MakePrism(aBox, aProfile, aSide, new Direction3D(-theSign, 0, 0), theMode, true);
        aPrism.Add(EdgeAt(aProfile, p => Math.Abs(p.Z - 30) < 1e-9), aTop);
        aPrism.Perform(4.0);
        AssertSound(aPrism.Shape());
        Assert.Equal(BoxVolume + theSign * 4 * 6 * 10, Volume(aPrism.Shape()), 6);
    }

    [Fact]
    public void A_prism_answers_with_its_curves_through_the_profile()
    {
        (Shape aBox, Face aTop, Face aProfile) = Scene();
        var aPrism = new MakePrism(aBox, aProfile, aTop, Up, 1, true);
        aPrism.Perform(Level(38));
        var aCurves = new NCollection_Sequence<Curve3D>();
        aPrism.Curves(aCurves);
        Assert.True(aCurves.Length() > 0);
        foreach (Curve3D aCurve in aCurves)
        {
            // each runs along the direction, from within the profile's rectangle
            Point3D aP = aCurve.Value(0);
            Assert.InRange(aP.X, 3 - 1e-9, 7 + 1e-9);
            Assert.InRange(aP.Y, 7 - 1e-9, 13 + 1e-9);
            aCurve.D1(0, out _, out Vector3D aV);
            Assert.Equal(0, aV.X, 12);
            Assert.Equal(0, aV.Y, 12);
        }
        Curve3D? aBaryc = aPrism.BarycCurve();
        Assert.NotNull(aBaryc);
        Assert.Equal(5, aBaryc.Value(0).X, 9);
        Assert.Equal(10, aBaryc.Value(0).Y, 9);
    }

    public enum DraftLimit
    {
        Until, FromUntil, FromEnd, UntilHeight,
    }

    // A draft prism at 3 degrees: the rectangle narrows or widens by tan(3 degrees) per unit of height on every side.
    // Narrowing keeps its corners, so it's a frustum. Widening rounds them, as an offset wire does, so a quarter cone
    // takes the place of each mitred corner: pi d^2 rather than 4 d^2 of section
    private static readonly double Tan3 = Math.Tan(3 * Math.PI / 180);

    private static double Narrowing(double theH) => 24 * theH - 10 * Tan3 * theH * theH + 4 * Tan3 * Tan3 * theH * theH * theH / 3;

    private static double Widening(double theH) => 24 * theH + 10 * Tan3 * theH * theH + Math.PI * Tan3 * Tan3 * theH * theH * theH / 3;

    [Theory]
    [InlineData(1.0)]
    [InlineData(5.0)]
    [InlineData(20.0)]
    public void A_draft_prism_by_height_narrows_or_widens_with_the_sign_of_its_angle(double theHeight)
    {
        (Shape aBox, Face aTop, Face aProfile) = Scene();
        var aNarrow = new MakeDraftPrism(aBox, aProfile, aTop, 3 * Math.PI / 180, 1, true);
        aNarrow.Perform(theHeight);
        AssertSound(aNarrow.Shape());
        Assert.Equal(BoxVolume + Narrowing(theHeight), Volume(aNarrow.Shape()), 5);
        var aWide = new MakeDraftPrism(aBox, aProfile, aTop, -3 * Math.PI / 180, 1, true);
        aWide.Perform(theHeight);
        AssertSound(aWide.Shape());
        Assert.Equal(BoxVolume + Widening(theHeight), Volume(aWide.Shape()), 5);
        // cut downwards, the signs of the angle swap
        var aCut = new MakeDraftPrism(aBox, aProfile, aTop, -3 * Math.PI / 180, 0, true);
        aCut.Perform(-theHeight);
        AssertSound(aCut.Shape());
        Assert.Equal(BoxVolume - Narrowing(theHeight), Volume(aCut.Shape()), 5);
        var aCutWide = new MakeDraftPrism(aBox, aProfile, aTop, 3 * Math.PI / 180, 0, true);
        aCutWide.Perform(-theHeight);
        AssertSound(aCutWide.Shape());
        Assert.Equal(BoxVolume - Widening(theHeight), Volume(aCutWide.Shape()), 5);
        // and a cut upwards finds nothing to cut
        var anUp = new MakeDraftPrism(aBox, aProfile, aTop, 3 * Math.PI / 180, 0, true);
        anUp.Perform(theHeight);
        Assert.Equal(BoxVolume, Volume(anUp.Shape()), 6);
    }

    [Theory]
    [InlineData(-3, 20, 0)]
    [InlineData(3, 20, 0)]
    public void A_cut_draft_prism_up_to_a_level(double theDegrees, double theH2, double theH1)
    {
        double Removed(double h) => theDegrees < 0 ? Narrowing(h) : Widening(h);
        (Shape aBox, Face aTop, Face aProfile) = Scene();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        var aUntil = new MakeDraftPrism(aBox, aProfile, aTop, theDegrees * Math.PI / 180, 0, true);
        aUntil.Perform(Level(10));
        Assert.True(aUntil.IsDone);
        AssertSound(aUntil.Shape());
        Assert.Equal(BoxVolume - (Removed(theH2) - Removed(theH1)), Volume(aUntil.Shape()), 4);
        aSnap.AssertUnchanged(aBox);
        var aFromEnd = new MakeDraftPrism(aBox, aProfile, aTop, theDegrees * Math.PI / 180, 0, true);
        aFromEnd.PerformFromEnd(Level(10));
        Assert.Equal(Volume(aUntil.Shape()), Volume(aFromEnd.Shape()), 6);
    }

    [Theory]
    [InlineData(-3)]
    [InlineData(3)]
    public void A_cut_draft_prism_between_two_levels_is_the_difference_of_those_up_to_them(double theDegrees)
    {
        double Removed(double h) => theDegrees < 0 ? Narrowing(h) : Widening(h);
        (Shape aBox, Face aTop, Face aProfile) = Scene();
        var aPrism = new MakeDraftPrism(aBox, aProfile, aTop, theDegrees * Math.PI / 180, 0, true);
        aPrism.Perform(Level(25), Level(10));
        Assert.True(aPrism.IsDone);
        AssertSound(aPrism.Shape());
        Assert.Equal(BoxVolume - (Removed(20) - Removed(5)), Volume(aPrism.Shape()), 4);
    }

    [Fact]
    public void A_fused_draft_prism_up_to_or_between_levels()
    {
        (Shape aBox, Face aTop, Face aProfile) = Scene();
        var aUntil = new MakeDraftPrism(aBox, aProfile, aTop, 3 * Math.PI / 180, 1, true);
        aUntil.Perform(Level(38));
        AssertSound(aUntil.Shape());
        Assert.Equal(BoxVolume + Narrowing(8), Volume(aUntil.Shape()), 4);
        Assert.Equal(1, Count(aUntil.Shape(), ShapeType.Solid));
        var aBetween = new MakeDraftPrism(aBox, aProfile, aTop, 3 * Math.PI / 180, 1, true);
        aBetween.Perform(Level(33), Level(38));
        AssertSound(aBetween.Shape());
        Assert.Equal(BoxVolume + Narrowing(8) - Narrowing(3), Volume(aBetween.Shape()), 4);
        Assert.Equal(2, Count(aBetween.Shape(), ShapeType.Solid));
    }

    // Through everything, and up to a height, the prism starts from below or above its profile, where the offset wire
    // has already rounded its corners: a little less than the mitred frustum, by much less than the corners' share
    [Fact]
    public void A_draft_prism_through_everything_or_up_to_a_height_is_a_frustum_with_rounder_corners()
    {
        (Shape aBox, Face aTop, Face aProfile) = Scene();
        var aThru = new MakeDraftPrism(aBox, aProfile, aTop, -3 * Math.PI / 180, 0, true);
        aThru.PerformThruAll();
        AssertSound(aThru.Shape());
        Assert.InRange(BoxVolume - Volume(aThru.Shape()), 0.999 * Narrowing(30), Narrowing(30));
        var aHeight = new MakeDraftPrism(aBox, aProfile, aTop, 3 * Math.PI / 180, 1, true);
        aHeight.PerformUntilHeight(Level(38), 7);
        AssertSound(aHeight.Shape());
        Assert.InRange(Volume(aHeight.Shape()) - BoxVolume, 0.998 * Narrowing(7), Narrowing(7));
        var aFused = new MakeDraftPrism(aBox, aProfile, aTop, 3 * Math.PI / 180, 1, true);
        aFused.PerformThruAll();
        Assert.False(aFused.IsDone);
        var anEnd = new MakeDraftPrism(aBox, aProfile, aTop, 3 * Math.PI / 180, 1, true);
        anEnd.PerformUntilEnd();
        Assert.False(anEnd.IsDone);
    }

    [Fact]
    public void A_draft_prism_checks_its_limits_and_its_sliding_edges()
    {
        (Shape aBox, Face aTop, Face aProfile) = Scene();
        var aPrism = new MakeDraftPrism(aBox, aProfile, aTop, 3 * Math.PI / 180, 1, true);
        Assert.Throws<ConstructionErrorException>(() => aPrism.Perform(new Shape()));
        Assert.Throws<ConstructionErrorException>(() => aPrism.Perform(new Shape(), Level(38)));
        Assert.Throws<ConstructionErrorException>(() => aPrism.Perform(Level(33), new Shape()));
        Assert.Throws<ConstructionErrorException>(() => aPrism.PerformFromEnd(new Shape()));
        Edge anEdge = EdgeAt(aProfile, _ => true);
        Assert.Throws<ConstructionErrorException>(() => aPrism.Add(anEdge, aProfile));
        Assert.Throws<ConstructionErrorException>(() => aPrism.Add(EdgeAt(aBox, _ => true), aTop));
        aPrism.Add(anEdge, aTop);
        aPrism.Add(anEdge, aTop);
    }

    [Theory]
    [InlineData(1, 0, 4)] // the first shape has no free edges; the last shape has the 4 of the top
    [InlineData(2, 4, 4)]
    [InlineData(-2, 4, 0)]
    public void The_boss_edges_of_a_draft_prism(int theSignature, int theTop, int theLateral)
    {
        (Shape aBox, Face aTop, Face aProfile) = Scene();
        var aPrism = new MakeDraftPrism(aBox, aProfile, aTop, 3 * Math.PI / 180, 1, true);
        aPrism.Perform(5.0);
        aPrism.BossEdges(theSignature);
        if (theSignature == 1)
        {
            Assert.Empty(aPrism.TopEdges());
            Assert.Empty(aPrism.LatEdges());
            return;
        }
        Assert.Equal(theTop, aPrism.TopEdges().Count);
        Assert.Equal(theLateral, aPrism.LatEdges().Count);
        // the top edges are at the prism's height
        foreach (Shape anEdge in aPrism.TopEdges())
        {
            var aProps = new Properties();
            MassPropertiesBuilder.LinearProperties(anEdge, aProps);
            Assert.Equal(35, aProps.CentreOfMass().Z, 9);
        }
    }

    [Fact]
    public void A_signature_out_of_range_finds_no_boss_edges()
    {
        (Shape aBox, Face aTop, Face aProfile) = Scene();
        var aPrism = new MakeDraftPrism(aBox, aProfile, aTop, 3 * Math.PI / 180, 1, true);
        aPrism.Perform(5.0);
        aPrism.BossEdges(0);
        Assert.Empty(aPrism.TopEdges());
        Assert.Empty(aPrism.LatEdges());
    }

    // Revolutions about the x axis through (0, -20, 30): the profile's centroid is 30 from it

    private static readonly Axis3D Axis = new(new Point3D(0, -20, 30), new Direction3D(1, 0, 0));
    private const double Radius = 30;
    private const double Overlap = 604.685793; // what a whole turn takes out of the box, by the cut below

    [Theory]
    [InlineData(0, -0.3, -1)]
    [InlineData(1, 0.3, 1)]
    public void A_revolution_by_an_angle_follows_Pappus(int theMode, double theAngle, double theSign)
    {
        (Shape aBox, Face aTop, Face aProfile) = Scene();
        var aRevol = new MakeRevol(aBox, aProfile, aTop, Axis, theMode, true);
        aRevol.Perform(theAngle);
        AssertSound(aRevol.Shape());
        Assert.Equal(BoxVolume + theSign * Area * Radius * Math.Abs(theAngle), Volume(aRevol.Shape()), 6);
    }

    [Fact]
    public void A_whole_turn_fused_exceeds_a_whole_turn_cut_by_the_whole_ring()
    {
        (Shape aBox, Face aTop, Face aProfile) = Scene();
        var aCut = new MakeRevol(aBox, aProfile, aTop, Axis, 0, true);
        aCut.PerformThruAll();
        var aFuse = new MakeRevol(aBox, aProfile, aTop, Axis, 1, true);
        aFuse.PerformThruAll();
        AssertSound(aCut.Shape());
        AssertSound(aFuse.Shape());
        Assert.Equal(BoxVolume - Overlap, Volume(aCut.Shape()), 5);
        Assert.Equal(2 * Math.PI * Radius * Area, Volume(aFuse.Shape()) - Volume(aCut.Shape()), 5);
    }

    [Fact]
    public void A_revolution_until_a_half_plane_stops_there()
    {
        (Shape aBox, Face aTop, Face aProfile) = Scene();
        var aRevol = new MakeRevol(aBox, aProfile, aTop, Axis, 1, true);
        aRevol.Perform(HalfPlane(Axis, 0.5));
        AssertSound(aRevol.Shape());
        Assert.Equal(BoxVolume + Area * Radius * 0.5, Volume(aRevol.Shape()), 6);
        var anAngle = new MakeRevol(aBox, aProfile, aTop, Axis, 1, true);
        anAngle.PerformUntilAngle(HalfPlane(Axis, 0.5), 0.2);
        AssertSound(anAngle.Shape());
        Assert.Equal(BoxVolume + Area * Radius * 0.2, Volume(anAngle.Shape()), 6);
    }

    [Fact]
    public void A_revolution_turns_one_way_only_so_a_limit_behind_it_is_reached_the_long_way_round()
    {
        (Shape aBox, Face aTop, Face aProfile) = Scene();
        var aRevol = new MakeRevol(aBox, aProfile, aTop, Axis, 0, true);
        aRevol.Perform(HalfPlane(Axis, -0.5));
        AssertSound(aRevol.Shape());
        // all of the whole turn's overlap but the 0.5 radians behind the start, which stays in the box
        Assert.Equal(BoxVolume - Overlap + Area * Radius * 0.5, Volume(aRevol.Shape()), 5);
    }

    [Fact]
    public void A_revolution_checks_its_limits_and_its_sliding_edges()
    {
        (Shape aBox, Face aTop, Face aProfile) = Scene();
        var aRevol = new MakeRevol(aBox, aProfile, aTop, Axis, 1, true);
        Assert.Throws<ConstructionErrorException>(() => aRevol.Perform(new Shape()));
        Assert.Throws<ConstructionErrorException>(() => aRevol.Perform(new Shape(), Level(38)));
        Assert.Throws<ConstructionErrorException>(() => aRevol.Perform(Level(33), new Shape()));
        Assert.Throws<ConstructionErrorException>(() => aRevol.PerformUntilAngle(new Shape(), 0.3));
        Edge anEdge = EdgeAt(aProfile, _ => true);
        Assert.Throws<ConstructionErrorException>(() => aRevol.Add(anEdge, aProfile));
        Assert.Throws<ConstructionErrorException>(() => aRevol.Add(EdgeAt(aBox, _ => true), aTop));
        aRevol.Add(anEdge, aTop);
        aRevol.Add(anEdge, aTop);
    }

    // Pipes along a parabola in the plane y = 9, starting at (4, 9, 30) and leaving along z, going down when cutting
    // (mirrored in z = 30): the limits ask for the curves of the sweep, so these can't be lines (see below)

    private static Wire Parabola(int theMode)
    {
        double aSign = theMode == 1 ? 1 : -1;
        var aPoles = new NCollection_Array1<Point3D>(1, 3);
        aPoles[1] = new Point3D(4, 9, 30);
        aPoles[2] = new Point3D(4, 9, 30 + aSign * 5);
        aPoles[3] = new Point3D(7, 9, 30 + aSign * 10);
        return new MakeWire(new MakeEdge(new BezierCurve3D(aPoles)).Edge()).Wire();
    }

    [Fact]
    public void A_pipe_along_an_arc_follows_Pappus()
    {
        foreach ((int aMode, double aSign) in new[] { (0, -1.0), (1, 1.0) })
        {
            var aCircle = new Circle3D(new Frame3D(new Point3D(24, 9, 30), new Direction3D(0, aSign, 0), new Direction3D(-1, 0, 0)), 20);
            (Shape aBox, Face aTop, Face aProfile) = Scene();
            var aPipe = new MakePipe(aBox, aProfile, aTop, new MakeWire(new MakeEdge(aCircle, 0, 0.6).Edge()).Wire(), aMode, true);
            aPipe.Perform();
            AssertSound(aPipe.Shape());
            // the profile's centroid (5, 10) is 19 from the arc's axis, along x
            Assert.Equal(BoxVolume + aSign * Area * 19 * 0.6, Volume(aPipe.Shape()), 5);
        }
    }

    [Fact]
    public void A_pipe_up_to_a_level_or_between_levels()
    {
        double Up(double z)
        {
            (Shape aBox, Face aTop, Face aProfile) = Scene();
            var aPipe = new MakePipe(aBox, aProfile, aTop, Parabola(1), 1, true);
            aPipe.Perform(Level(z));
            Assert.True(aPipe.IsDone);
            AssertSound(aPipe.Shape());
            Assert.Equal(z, ZRange(aPipe.Shape()).Max, 7);
            return Volume(aPipe.Shape());
        }
        double aV32 = Up(32), aV35 = Up(35);
        Assert.InRange(aV32, BoxVolume, aV35);

        (Shape aBox2, Face aTop2, Face aProfile2) = Scene();
        var aWhole = new MakePipe(aBox2, aProfile2, aTop2, Parabola(1), 1, true);
        aWhole.Perform();
        Assert.InRange(aV35, BoxVolume, Volume(aWhole.Shape()));

        var aBetween = new MakePipe(aBox2, aProfile2, aTop2, Parabola(1), 1, true);
        aBetween.Perform(Level(32), Level(35));
        Assert.True(aBetween.IsDone);
        AssertSound(aBetween.Shape());
        Assert.Equal(2, Count(aBetween.Shape(), ShapeType.Solid));
        Assert.Equal(BoxVolume + aV35 - aV32, Volume(aBetween.Shape()), 5);
    }

    [Fact]
    public void A_cut_pipe_mirrors_a_fused_one_in_the_top_of_the_box()
    {
        (Shape aBox, Face aTop, Face aProfile) = Scene();
        var aFused = new MakePipe(aBox, aProfile, aTop, Parabola(1), 1, true);
        aFused.Perform(Level(35));
        var aCut = new MakePipe(aBox, aProfile, aTop, Parabola(0), 0, true);
        aCut.Perform(Level(25));
        Assert.True(aCut.IsDone);
        AssertSound(aCut.Shape());
        Assert.Equal(BoxVolume - (Volume(aFused.Shape()) - BoxVolume), Volume(aCut.Shape()), 5);
        var aBetween = new MakePipe(aBox, aProfile, aTop, Parabola(0), 0, true);
        aBetween.Perform(Level(28), Level(25));
        Assert.True(aBetween.IsDone);
        AssertSound(aBetween.Shape());
        Assert.InRange(Volume(aBetween.Shape()), Volume(aCut.Shape()), BoxVolume);
    }

    [Fact]
    public void A_limited_pipe_along_a_line_or_an_arc_throws_where_OCCT_does()
    {
        // occt-issues.md: LocOpe_Pipe's curves convert the sweep's lines to B-splines, which GeomConvert refuses for a
        // whole line; and BRepFill_Pipe::PipeLine finds no edge for a point swept along a circle
        (Shape aBox, Face aTop, Face aProfile) = Scene();
        Wire aLine = new MakeWire(new MakeEdge(new Point3D(4, 9, 30), new Point3D(4, 9, 45)).Edge()).Wire();
        var aStraight = new MakePipe(aBox, aProfile, aTop, aLine, 1, true);
        Assert.Throws<DomainErrorException>(() => aStraight.Perform(Level(35)));
        var aPlain = new MakePipe(aBox, aProfile, aTop, aLine, 1, true);
        aPlain.Perform();
        Assert.Equal(BoxVolume + 15 * Area, Volume(aPlain.Shape()), 6);

        var aCircle = new Circle3D(new Frame3D(new Point3D(24, 9, 30), new Direction3D(0, 1, 0), new Direction3D(-1, 0, 0)), 20);
        Wire anArc = new MakeWire(new MakeEdge(aCircle, 0, 0.6).Edge()).Wire();
        var aCurved = new MakePipe(aBox, aProfile, aTop, anArc, 1, true);
        Assert.Throws<OutOfRangeException>(() => aCurved.Perform(Level(35)));
    }

    [Fact]
    public void A_pipe_checks_its_limits_and_its_sliding_edges()
    {
        (Shape aBox, Face aTop, Face aProfile) = Scene();
        var aPipe = new MakePipe(aBox, aProfile, aTop, Parabola(1), 1, true);
        var aVertex = new MakeVertex(new Point3D(0, 0, 40)).Vertex();
        Assert.Throws<ConstructionErrorException>(() => aPipe.Perform(new Shape()));
        Assert.Throws<ConstructionErrorException>(() => aPipe.Perform(aVertex));
        Assert.Throws<ConstructionErrorException>(() => aPipe.Perform(new Shape(), Level(38)));
        Assert.Throws<ConstructionErrorException>(() => aPipe.Perform(aVertex, Level(38)));
        Assert.Throws<ConstructionErrorException>(() => aPipe.Perform(Level(33), aVertex));
        Edge anEdge = EdgeAt(aProfile, _ => true);
        Assert.Throws<ConstructionErrorException>(() => aPipe.Add(anEdge, aProfile));
        Assert.Throws<ConstructionErrorException>(() => aPipe.Add(EdgeAt(aBox, _ => true), aTop));
        aPipe.Add(anEdge, aTop);
        aPipe.Add(anEdge, aTop);
    }

    // Holes down through a C: a box 10 x 20 x 30 with a slot from x = 0 to 8 and z = 10 to 20, drilled at x = 4 from
    // z = 31, so through a prong from 30 to 20, a gap, and a prong from 10 to 0. Hole parameters are along that axis

    private const double CVolume = BoxVolume - 8 * 20 * 10;
    private const double Disc = Math.PI * 2 * 2;

    private static MakeCylindricalHole CHole(double theX = 4)
    {
        Shape aSlot = new MakeBox(new Point3D(-1, -1, 10), new Point3D(8, 21, 20)).Shape();
        Shape aCut = new Cut(LocOpeTests.Box(), aSlot).Shape();
        var aHole = new MakeCylindricalHole();
        aHole.Init(new Explorer(aCut, ShapeType.Solid).Current(), new Axis3D(new Point3D(theX, 10, 31), Down));
        return aHole;
    }

    private static void AssertHole(MakeCylindricalHole theHole, double theRemoved)
    {
        Assert.Equal(FeatureStatus.NoError, theHole.Status());
        theHole.Build();
        Assert.Equal(FeatureStatus.NoError, theHole.Status());
        AssertSound(theHole.Shape());
        Assert.Equal(CVolume - theRemoved, Volume(theHole.Shape()), 6);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void A_hole_through_goes_through_both_prongs_and_a_hole_to_the_next_through_one(bool theControl)
    {
        MakeCylindricalHole aThru = CHole();
        aThru.Perform(2);
        AssertHole(aThru, Disc * 20);
        MakeCylindricalHole aNext = CHole();
        aNext.PerformThruNext(2, theControl);
        AssertHole(aNext, Disc * 10);
        MakeCylindricalHole anEnd = CHole();
        anEnd.PerformUntilEnd(2, theControl);
        AssertHole(anEnd, Disc * 20);
    }

    [Theory]
    [InlineData(2, 8, 10)] // inside the first prong, so the whole of that prong
    [InlineData(5, 25, 20)] // from the first prong to the second
    [InlineData(25, 5, 20)] // either way round
    public void A_hole_between_parameters_takes_the_material_it_reaches(double theFrom, double theTo, double theHeight)
    {
        MakeCylindricalHole aHole = CHole();
        aHole.Perform(2, theFrom, theTo);
        AssertHole(aHole, Disc * theHeight);
        MakeCylindricalHole aFree = CHole();
        aFree.Perform(2, theFrom, theTo, false);
        AssertHole(aFree, Disc * theHeight);
    }

    [Fact]
    public void A_hole_between_parameters_in_the_gap_has_no_place()
    {
        MakeCylindricalHole aHole = CHole();
        aHole.Perform(2, 13, 18);
        Assert.Equal(FeatureStatus.InvalidPlacement, aHole.Status());
        aHole.Build(); // nothing to do: the status stays
        Assert.Equal(FeatureStatus.InvalidPlacement, aHole.Status());
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void A_blind_hole_is_measured_from_the_axis_origin(bool theControl)
    {
        MakeCylindricalHole aHole = CHole();
        aHole.PerformBlind(2, 5, theControl);
        AssertHole(aHole, Disc * 4); // from 31 to 26, in material from 30
    }

    [Theory]
    [InlineData(15)] // ends in the gap
    [InlineData(25)] // goes through the first prong and the gap
    [InlineData(80)] // past everything
    public void A_blind_hole_past_the_first_stretch_of_material_is_too_long(double theLength)
    {
        MakeCylindricalHole aHole = CHole();
        aHole.PerformBlind(2, theLength);
        Assert.Equal(FeatureStatus.HoleTooLong, aHole.Status());
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void A_hole_that_misses_the_shape_has_no_place(int theForm)
    {
        MakeCylindricalHole aHole = CHole(55);
        switch (theForm)
        {
            case 0: aHole.PerformThruNext(2); break;
            case 1: aHole.PerformUntilEnd(2); break;
            case 2: aHole.Perform(2, 5, 25); break;
            case 3: aHole.PerformBlind(2, 5); break;
        }
        Assert.Equal(FeatureStatus.InvalidPlacement, aHole.Status());
    }

    [Fact]
    public void A_hole_breaking_out_of_the_side_is_still_a_sound_cut()
    {
        // radius 4.5 at x = 4 leaves the box at x = 0 and passes the slot's end at x = 8
        MakeCylindricalHole aHole = CHole();
        aHole.PerformThruNext(4.5);
        aHole.Build();
        Assert.Equal(FeatureStatus.NoError, aHole.Status());
        AssertSound(aHole.Shape());
        Assert.InRange(Volume(aHole.Shape()), CVolume - Math.PI * 4.5 * 4.5 * 30, CVolume - 1);
    }

    [Fact]
    public void A_blind_hole_breaking_out_of_the_side_fails_the_control_because_its_bottom_is_cut()
    {
        MakeCylindricalHole aHole = CHole();
        aHole.PerformBlind(4.5, 5);
        Assert.Equal(FeatureStatus.NoError, aHole.Status());
        aHole.Build();
        Assert.Equal(FeatureStatus.HoleTooLong, aHole.Status());
    }

    [Fact]
    public void A_hole_needs_a_shape_and_an_axis()
    {
        var aHole = new MakeCylindricalHole();
        aHole.Init(LocOpeTests.Box());
        Assert.Throws<ConstructionErrorException>(() => aHole.Perform(2));
        Assert.Throws<ConstructionErrorException>(() => aHole.PerformThruNext(2));
        Assert.Throws<ConstructionErrorException>(() => aHole.PerformUntilEnd(2));
        Assert.Throws<ConstructionErrorException>(() => aHole.Perform(2, 1, 5));
        Assert.Throws<ConstructionErrorException>(() => aHole.PerformBlind(2, 5));
    }
}
