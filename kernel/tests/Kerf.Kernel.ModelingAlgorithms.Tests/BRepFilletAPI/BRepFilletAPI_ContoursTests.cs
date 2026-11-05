// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Laws;
using Kerf.Engine.Fillets.Api;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The contours of BRepFilletAPI_MakeFillet and MakeChamfer on shapes with known answers (unit 11I): what the makers report
/// of a contour before building, what changing it does to the volume, and what a build leaves. On a box of side 10, a
/// fillet of radius r on an edge removes (1 - pi/4) r^2 10, and a chamfer of d1 and d2 removes d1 d2 10 / 2.
/// </summary>
public class BRepFilletAPI_ContoursTests
{
    private const double Side = 10.0;
    private const double FilletCut = (1.0 - Math.PI / 4.0) * Side;

    private static Shape Box() => new MakeBox(Side, Side, Side).Shape();

    private static Edge EdgeOf(in Shape S, int k)
    {
        IndexedSet<Shape> M = new(ShapeHasher.Instance);
        Exploration.MapShapes(S, ShapeType.Edge, M);
        return ShapeCasts.Edge(M[k - 1]);
    }

    private static Face FaceOn(in Shape S, in Edge E)
    {
        IndexedMap<Shape, NCollection_List<Shape>> M = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(S, ShapeType.Edge, ShapeType.Face, M);
        return ShapeCasts.Face(M.GetValue(E).First());
    }


    private static void Bookkeeping(LocalOperation theMaker, Shape theBox)
    {
        Edge E1 = EdgeOf(theBox, 1), E2 = EdgeOf(theBox, 7);
        Assert.Equal(0, theMaker.ContourCount);

        theMaker.Add(E1);
        theMaker.Add(E2);
        Assert.Equal(2, theMaker.ContourCount);
        Assert.Equal(1, theMaker.Contour(E1));
        Assert.Equal(2, theMaker.Contour(E2));
        Assert.Equal(0, theMaker.Contour(EdgeOf(theBox, 12)));
        Assert.Equal(1, theMaker.NbEdges(1));
        Assert.True(theMaker.Edge(1, 1).IsSame(E1));
        Assert.True(theMaker.Edge(2, 1).IsSame(E2));
        Assert.Equal(Side, theMaker.Length(1), 1e-9);
        Assert.False(theMaker.Closed(1));
        Assert.False(theMaker.ClosedAndTangent(1));

        // the two ends are the edge's vertices, at the two ends of the abscissa
        Vertex aFirst = theMaker.FirstVertex(1), aLast = theMaker.LastVertex(1);
        Assert.False(aFirst.IsSame(aLast));
        Assert.Equal(0.0, theMaker.Abscissa(1, aFirst), 1e-9);
        Assert.Equal(Side, theMaker.Abscissa(1, aLast), 1e-9);
        Assert.Equal(0.0, theMaker.RelativeAbscissa(1, aFirst), 1e-9);
        Assert.Equal(1.0, theMaker.RelativeAbscissa(1, aLast), 1e-9);

        theMaker.Remove(E1);
        Assert.Equal(1, theMaker.ContourCount);
        Assert.Equal(0, theMaker.Contour(E1));
        Assert.Equal(1, theMaker.Contour(E2));

        theMaker.Reset();
        Assert.Equal(1, theMaker.ContourCount);
    }

    [Fact]
    public void A_fillet_reports_its_contours_as_it_has_them()
    {
        Shape aBox = Box();
        Bookkeeping(new MakeFillet(aBox), aBox);
    }

    [Fact]
    public void A_chamfer_reports_its_contours_as_it_has_them()
    {
        Shape aBox = Box();
        Bookkeeping(new MakeChamfer(aBox), aBox);
    }

    [Fact]
    public void A_circular_edge_is_one_closed_tangent_contour_of_its_circumference()
    {
        // a cylinder of radius 3 and height 8: filleting a rim of radius 1 removes a torus-like ring
        Shape aCylinder = new ApiMakeCylinder(3.0, 8.0).Shape();
        Edge aRim = default!;
        for (Explorer ex = new(aCylinder, ShapeType.Edge); ex.More(); ex.Next())
        {
            Edge E = ShapeCasts.Edge(ex.Current());
            if (!Tool.Degenerated(E) && !Tool.IsClosed(E, FaceOn(aCylinder, E)))
            {
                aRim = E;
                break;
            }
        }

        MakeFillet aFillet = new(aCylinder);
        aFillet.Add(1.0, aRim);
        Assert.True(aFillet.Closed(1));
        Assert.True(aFillet.ClosedAndTangent(1));
        Assert.Equal(2.0 * Math.PI * 3.0, aFillet.Length(1), 1e-6);
        Assert.True(aFillet.FirstVertex(1).IsSame(aFillet.LastVertex(1)));
        aFillet.Build();
        Assert.True(aFillet.IsDone);
        Assert.True(new Analyzer(aFillet.Shape()).IsValid());
        // Pappus: the removed section's area (1 - pi/4) by its centroid's path, 0.2234 in from the corner
        double aRemoved = 2.0 * Math.PI * (3.0 - 0.2234) * (1.0 - Math.PI / 4.0);
        Assert.Equal(Math.PI * 9.0 * 8.0 - aRemoved, ReconComparison.Volume(aFillet.Shape()), 0.01);
    }

    [Fact]
    public void Changing_a_fillets_radius_changes_what_it_removes()
    {
        Shape aBox = Box();
        Edge E = EdgeOf(aBox, 1);

        MakeFillet aFillet = new(aBox);
        aFillet.Add(2.0, E);
        Assert.True(aFillet.IsConstant(1));
        Assert.Equal(2.0, aFillet.Radius(1));
        Assert.True(aFillet.IsConstant(1, E));
        Assert.Equal(2.0, aFillet.Radius(1, E));

        // by index, by edge
        aFillet.SetRadius(1.0, 1, 1);
        Assert.Equal(1.0, aFillet.Radius(1));
        aFillet.SetRadius(1.5, 1, E);
        Assert.Equal(1.5, aFillet.Radius(1, E));
        Assert.Equal(Side * Side * Side - FilletCut * 1.5 * 1.5, ReconComparison.Volume(aFillet.Shape()), 1e-3);
    }

    [Fact]
    public void A_radius_set_from_one_value_to_another_is_a_linear_radius()
    {
        Shape aBox = Box();
        MakeFillet aFillet = new(aBox);
        aFillet.Add(EdgeOf(aBox, 1));
        aFillet.SetRadius(1.0, 3.0, 1, 1);
        Assert.False(aFillet.IsConstant(1));

        // as BRepFilletAPITests: (1 - pi/4) times the integral of r^2 over the edge, with r from 1 to 3
        Assert.Equal(1000.0 - (1.0 - Math.PI / 4.0) * (27.0 - 1.0) / 0.6, ReconComparison.Volume(aFillet.Shape()), 0.5);
    }

    [Fact]
    public void A_radius_set_at_a_vertex_and_by_pairs_makes_the_radius_vary()
    {
        Shape aBox = Box();
        Edge E = EdgeOf(aBox, 1);

        MakeFillet aVertex = new(aBox);
        aVertex.Add(1.0, E);
        Vertex aLast = aVertex.LastVertex(1);
        aVertex.SetRadius(3.0, 1, aLast);
        Assert.False(aVertex.IsConstant(1));
        Assert.Equal(1000.0 - (1.0 - Math.PI / 4.0) * (27.0 - 1.0) / 0.6, ReconComparison.Volume(aVertex.Shape()), 0.5);

        NCollection_Array1<Point2D> UandR = new(1, 2);
        UandR.SetValue(1, new Point2D(0.0, 1.0));
        UandR.SetValue(2, new Point2D(1.0, 3.0));
        MakeFillet aPairs = new(aBox);
        aPairs.Add(E);
        aPairs.SetRadius(UandR, 1, 1);
        Assert.False(aPairs.IsConstant(1));
        Assert.Equal(1000.0 - (1.0 - Math.PI / 4.0) * (27.0 - 1.0) / 0.6, ReconComparison.Volume(aPairs.Shape()), 0.5);
    }

    [Fact]
    public void An_edges_law_can_be_read_and_replaced_once_computed()
    {
        Shape aBox = Box();
        Edge E = EdgeOf(aBox, 1);
        MakeFillet aFillet = new(aBox);
        aFillet.Add(1.0, 3.0, E);
        // the limits are known once the contour has been computed (ChFiDS_FilSpine::ChangeLaw)
        double aFirst = 0, aLast = 0;
        EngineAssert.Throws<DomainErrorException>(() => aFillet.GetBounds(1, E, ref aFirst, ref aLast));
        aFillet.Simulate(1);

        Assert.True(aFillet.GetBounds(1, E, ref aFirst, ref aLast));
        Assert.Equal(Side, aLast - aFirst, 1e-9);
        LawsFunction? aLaw = aFillet.GetLaw(1, E);
        Assert.NotNull(aLaw);
        Assert.Equal(1.0, aLaw!.Value(aFirst), 1e-9);
        Assert.Equal(3.0, aLaw.Value(aLast), 1e-9);

        // a constant radius of 2 by law
        Constant aTwo = new();
        aTwo.Set(2.0, aFirst, aLast);
        aFillet.SetLaw(1, E, aTwo);
        Assert.Equal(2.0, aFillet.GetLaw(1, E)!.Value(aFirst), 1e-12);
    }

    [Fact]
    public void A_fillet_built_reports_its_surfaces_and_no_faults()
    {
        Shape aBox = Box();
        Edge E = EdgeOf(aBox, 1);
        MakeFillet aFillet = new(aBox);
        aFillet.Add(1.0, E);
        Assert.Equal(1000.0 - FilletCut, ReconComparison.Volume(aFillet.Shape()), 1e-3);

        // the data structure's surfaces include any tried and dropped, but only the fillet has a face
        Assert.Equal(1, Enumerable.Range(1, aFillet.SurfaceCount).Sum(i => aFillet.NewFaces(i).Count));
        Assert.Equal(1, aFillet.NbSurf(1));
        Assert.Equal(0, aFillet.FaultyContourCount);
        Assert.Equal(0, aFillet.FaultyVertexCount);
        Assert.Equal(1, aFillet.NbComputedSurfaces(1));
        Assert.NotNull(aFillet.ComputedSurface(1, 1));
        Assert.False(aFillet.HasResult);
        Assert.Equal(ErrorStatus.Ok, aFillet.StripeStatus(1));
        Assert.Single(aFillet.Generated(E));
        Assert.False(aFillet.IsDeleted(FaceOn(aBox, E)));
    }

    [Fact]
    public void A_simulated_fillet_gives_sections_of_the_radius()
    {
        Shape aBox = Box();
        MakeFillet aFillet = new(aBox);
        aFillet.Add(2.0, EdgeOf(aBox, 1));
        aFillet.Simulate(1);

        Assert.True(aFillet.NbSurf(1) >= 1);
        NCollection_HArray1<CircSection>? aSections = aFillet.Sect(1, 1);
        Assert.NotNull(aSections);
        Assert.True(aSections!.Length() > 1);
        for (int i = aSections.Lower(); i <= aSections.Upper(); i++)
        {
            aSections[i].Get(out Circle3D aCircle, out double aFirst, out double aLast);
            Assert.Equal(2.0, aCircle.Radius, 1e-6);
            // a quarter of the circle: the two faces meet at a right angle
            Assert.Equal(Math.PI / 2.0, aLast - aFirst, 1e-6);
        }
    }

    [Fact]
    public void The_fillet_settings_are_kept_and_build_the_same_fillet()
    {
        Shape aBox = Box();
        MakeFillet aFillet = new(aBox, Kerf.Fillets.FilletShape.Polynomial);
        Assert.Equal(Kerf.Fillets.FilletShape.Polynomial, aFillet.GetFilletShape());
        aFillet.SetFilletShape(Kerf.Fillets.FilletShape.Rational);
        Assert.Equal(Kerf.Fillets.FilletShape.Rational, aFillet.GetFilletShape());
        aFillet.SetParams(1e-2, 1e-4, 1e-5, 1e-4, 1e-5, 1e-3);
        aFillet.SetContinuity(Continuity.C1, 1e-2);
        aFillet.Add(1.0, EdgeOf(aBox, 1));
        Assert.Equal(1000.0 - FilletCut, ReconComparison.Volume(aFillet.Shape()), 1e-3);
    }

    [Fact]
    public void A_contour_without_a_radius_can_be_reset()
    {
        // ChFiDS_FilSpine::Reset reads the first and last radii, which aren't there until one is set: decision 18601
        Shape aBox = Box();
        MakeFillet aFillet = new(aBox);
        aFillet.Add(EdgeOf(aBox, 1));
        aFillet.Reset();
        Assert.Equal(1, aFillet.ContourCount);
        aFillet.SetRadius(1.0, 1, 1);
        Assert.Equal(1000.0 - FilletCut, ReconComparison.Volume(aFillet.Shape()), 1e-3);
    }

    [Fact]
    public void A_contour_removed_leaves_the_rest()
    {
        Shape aBox = Box();
        Edge E1 = EdgeOf(aBox, 1), E2 = EdgeOf(aBox, 7);
        MakeFillet aFillet = new(aBox);
        aFillet.Add(1.0, E1);
        aFillet.Add(1.0, E2);
        aFillet.Remove(E1);
        Assert.Equal(1000.0 - FilletCut, ReconComparison.Volume(aFillet.Shape()), 1e-3);
    }

    // Chamfers

    [Fact]
    public void A_chamfers_three_kinds_report_their_values_and_remove_their_volumes()
    {
        Shape aBox = Box();
        Edge E = EdgeOf(aBox, 1);
        Face F = FaceOn(aBox, E);

        MakeChamfer aSymmetric = new(aBox);
        aSymmetric.Add(2.0, E);
        Assert.True(aSymmetric.IsSymetric(1));
        Assert.False(aSymmetric.IsTwoDistances(1));
        Assert.False(aSymmetric.IsDistanceAngle(1));
        double aDist = 0;
        aSymmetric.GetDist(1, ref aDist);
        Assert.Equal(2.0, aDist);
        aSymmetric.SetDist(1.0, 1, F);
        aSymmetric.GetDist(1, ref aDist);
        Assert.Equal(1.0, aDist);
        Assert.Equal(1000.0 - 1.0 / 2.0 * Side, ReconComparison.Volume(aSymmetric.Shape()), 1e-6);

        MakeChamfer aTwo = new(aBox);
        aTwo.Add(1.0, 2.0, E, F);
        Assert.False(aTwo.IsSymetric(1));
        Assert.True(aTwo.IsTwoDistances(1));
        aTwo.SetDists(2.0, 3.0, 1, F);
        double aD1 = 0, aD2 = 0;
        aTwo.Dists(1, ref aD1, ref aD2);
        Assert.Equal((2.0, 3.0), (aD1, aD2));
        Assert.Equal(1000.0 - 2.0 * 3.0 / 2.0 * Side, ReconComparison.Volume(aTwo.Shape()), 1e-6);

        // a distance of 2 and 45 degrees is a symmetric chamfer of 2; at atan(1/2) the other side is 1
        MakeChamfer anAngle = new(aBox);
        anAngle.AddDA(2.0, Math.PI / 4.0, E, F);
        Assert.True(anAngle.IsDistanceAngle(1));
        Assert.False(anAngle.IsSymetric(1));
        Assert.False(anAngle.IsTwoDistances(1));
        double aD = 0, anA = 0;
        anAngle.GetDistAngle(1, ref aD, ref anA);
        Assert.Equal((2.0, Math.PI / 4.0), (aD, anA));
        Assert.Equal(1000.0 - 2.0 * 2.0 / 2.0 * Side, ReconComparison.Volume(anAngle.Shape()), 1e-6);

        MakeChamfer aReset = new(aBox);
        aReset.AddDA(2.0, Math.PI / 4.0, E, F);
        aReset.SetDistAngle(2.0, Math.Atan(0.5), 1, F);
        Assert.Equal(1000.0 - 2.0 * 1.0 / 2.0 * Side, ReconComparison.Volume(aReset.Shape()), 1e-6);
    }

    [Fact]
    public void A_simulated_chamfer_gives_straight_sections()
    {
        Shape aBox = Box();
        MakeChamfer aChamfer = new(aBox);
        aChamfer.Add(1.0, EdgeOf(aBox, 1));
        aChamfer.Simulate(1);

        Assert.True(aChamfer.NbSurf(1) >= 1);
        NCollection_HArray1<CircSection>? aSections = aChamfer.Sect(1, 1);
        Assert.NotNull(aSections);
        Assert.True(aSections!.Length() > 1);
        for (int i = aSections.Lower(); i <= aSections.Upper(); i++)
        {
            // from one face to the other: the diagonal of a square of side 1
            aSections[i].Get(out Line3D _, out double aFirst, out double aLast);
            Assert.Equal(Math.Sqrt(2.0), Math.Abs(aLast - aFirst), 1e-6);
        }
    }
}
