// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Unit 5S1's laws and sections: BRepFill_LocationLaw and its derived laws (Edge3DLaw, ACRLaw, EdgeOnSurfLaw),
/// BRepFill_ShapeLaw, BRepFill_NSections, BRepFill_Section, BRepFill_SectionPlacement and
/// ShapeUpgrade_RemoveLocations; the frozen-input and publication rules (decisions 899-903). Native parity is in
/// <see cref="BRepFill_LawsNativeTests"/>.
/// </summary>
public class BRepFill_LawsTests
{
    // ------------------------------------------------------------------------------------------------ inputs

    private static Wire Polyline(params Point3D[] thePoints)
    {
        BRepBuilderAPI_MakePolygon aP = new();
        foreach (Point3D aQ in thePoints)
        {
            aP.Add(aQ);
        }
        return aP.Wire();
    }

    private static Wire ClosedSquare(double theSide, double theZ = 0)
    {
        BRepBuilderAPI_MakePolygon aP = new(new Point3D(0, 0, theZ), new Point3D(theSide, 0, theZ),
                                            new Point3D(theSide, theSide, theZ), new Point3D(0, theSide, theZ), true);
        return aP.Wire();
    }

    // A smooth spine: one cubic B-spline cut into two edges at its middle (a G1 vertex).
    private static Wire SmoothSpine()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 5);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(3, 2, 0);
        aPoles[3] = new Point3D(6, -1, 1);
        aPoles[4] = new Point3D(9, 1, 2);
        aPoles[5] = new Point3D(12, 0, 2);
        NCollection_Array1<double> aK = new(1, 3);
        aK[1] = 0;
        aK[2] = 1;
        aK[3] = 2;
        NCollection_Array1<int> aM = new(1, 3);
        aM[1] = 4;
        aM[2] = 1;
        aM[3] = 4;
        BSplineCurve3D aC = new(aPoles, aK, aM, 3);
        Vertex V0 = new BRepBuilderAPI_MakeVertex(aC.Value(0)).Vertex();
        Vertex V1 = new BRepBuilderAPI_MakeVertex(aC.Value(0.7)).Vertex();
        Vertex V2 = new BRepBuilderAPI_MakeVertex(aC.Value(2)).Vertex();
        BRepBuilderAPI_MakeWire aMW = new(new BRepBuilderAPI_MakeEdge(aC, V0, V1, 0, 0.7).Edge(),
                                          new BRepBuilderAPI_MakeEdge(aC, V1, V2, 0.7, 2).Edge());
        return aMW.Wire();
    }

    private static GeomFill_LocationLaw Frenet() => new GeomFill_CurveAndTrihedron(new GeomFill_CorrectedFrenet());

    private static string Text(in Shape theShape)
    {
        StringWriter aW = new();
        BrepTools.Write(theShape, aW);
        return aW.ToString();
    }

    private static bool AllFrozen(in Shape theShape)
    {
        NCollection_IndexedMap<Shape> aMap = new(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, aMap);
        for (int i = 1; i <= aMap.Extent(); i++)
        {
            if (!aMap.FindKey(i).TShape()!.IsFrozen)
            {
                return false;
            }
        }
        return theShape.TShape()!.IsFrozen;
    }

    // ------------------------------------------------------------------------------------------------ location laws

    [Fact]
    public void Edge3DLaw_on_a_polyline_has_a_law_per_edge_and_holes_at_its_corners()
    {
        Wire aW = Polyline(new Point3D(0, 0, 0), new Point3D(5, 0, 0), new Point3D(5, 5, 0), new Point3D(5, 5, 5));
        BRepFill_Edge3DLaw aLaw = new(aW, Frenet());
        Assert.Equal(3, aLaw.NbLaw());
        Assert.False(aLaw.IsClosed());
        Assert.Equal(-1, aLaw.IsG1(0)); // open path
        Assert.Equal(-1, aLaw.IsG1(1)); // corner: the trihedron jumps
        Assert.Equal(3, aLaw.NbHoles()); // the two corners and the open end
        NCollection_Array1<int> aH = new(1, 3);
        aLaw.Holes(aH);
        Assert.Equal(2, aH[1]);
        Assert.Equal(3, aH[2]);
        Assert.Equal(4, aH[3]);

        // curvilinear bounds are the lengths of the edges
        aLaw.CurvilinearBounds(2, out double f, out double l);
        Assert.Equal(5.0, f, 1e-9);
        Assert.Equal(10.0, l, 1e-9);

        // parameter and abscissa invert each other
        double U = 0;
        aLaw.Parameter(7.5, out int anIdx, ref U);
        Assert.Equal(2, anIdx);
        Assert.Equal(7.5, aLaw.Abscissa(anIdx, U), 1e-9);
        aLaw.Parameter(16, out anIdx, ref U);
        Assert.Equal(0, anIdx); // past the end

        // vertices of the path
        Assert.True(Tool.Pnt(aLaw.Vertex(1)).IsEqual(new Point3D(0, 0, 0), 1e-12));
        Assert.True(Tool.Pnt(aLaw.Vertex(4)).IsEqual(new Point3D(5, 5, 5), 1e-12));
        Assert.True(aLaw.Vertex(5).IsNull);
    }

    [Fact]
    public void Edge3DLaw_on_a_smooth_spine_is_G1_inside()
    {
        // Frenet's frame is continuous along the C2 curve; each edge's corrected Frenet would start its own angle law
        BRepFill_Edge3DLaw aLaw = new(SmoothSpine(), new GeomFill_CurveAndTrihedron(new GeomFill_Frenet()));
        Assert.Equal(2, aLaw.NbLaw());
        Assert.Equal(1, aLaw.IsG1(1, 1e-7, 1e-4));
        Assert.Equal(1, aLaw.NbHoles()); // only the open end
        NCollection_Array1<int> aH = new(1, 1);
        aLaw.Holes(aH);
        Assert.Equal(3, aH[1]);
    }

    [Fact]
    public void Each_edge_gets_its_own_copy_of_the_law()
    {
        GeomFill_LocationLaw aUser = Frenet();
        BRepFill_Edge3DLaw aLaw = new(Polyline(new Point3D(0, 0, 0), new Point3D(5, 0, 0), new Point3D(5, 5, 0)), aUser);
        Assert.NotSame(aUser, aLaw.Law(1));
        Assert.NotSame(aLaw.Law(1), aLaw.Law(2));
        Assert.Null(aUser.GetCurve()); // the caller's law is never given a curve
    }

    [Fact]
    public void Abscissa_before_the_lengths_are_known_takes_the_edge_end_as_OCCT_does()
    {
        // OCCT: Abscissa reads CurvilinearBounds' Last as the edge's start when the lengths aren't computed yet
        Wire aW = Polyline(new Point3D(0, 0, 0), new Point3D(4, 0, 0), new Point3D(4, 3, 0));
        BRepFill_Edge3DLaw aFresh = new(aW, Frenet());
        aFresh.Law(2).GetDomain(out double f, out double l);
        double aBuggy = aFresh.Abscissa(2, 0.5 * (f + l));
        Assert.Equal(7 + 1.5, aBuggy, 1e-9); // 4 + 3 (the end of edge 2) + 1.5
        Assert.Equal(4 + 1.5, aFresh.Abscissa(2, 0.5 * (f + l)), 1e-9); // the lengths are now known
    }

    [Fact]
    public void D0_places_a_copy_of_the_section_and_freezes_it()
    {
        BRepFill_Edge3DLaw aLaw = new(Polyline(new Point3D(0, 0, 0), new Point3D(10, 0, 0)),
                                      new GeomFill_CurveAndTrihedron(new GeomFill_Fixed(new Vector3D(1, 0, 0), new Vector3D(0, 1, 0))));
        Shape aSection = new BRepBuilderAPI_MakeVertex(new Point3D(0, 0, 0)).Vertex();
        Shape aOrig = aSection;
        aLaw.D0(4, ref aSection);
        Assert.False(aSection.IsSame(aOrig));
        Assert.True(AllFrozen(aSection));
        Assert.True(Tool.Pnt(ShapeCasts.Vertex(aSection)).IsEqual(new Point3D(4, 0, 0), 1e-9));
        aLaw.D0(11, ref aSection);
        Assert.True(aSection.IsNull);
    }

    [Fact]
    public void PerformVertex_is_frozen_and_InternalPerformVertex_is_not()
    {
        BRepFill_Edge3DLaw aLaw = new(Polyline(new Point3D(0, 0, 0), new Point3D(10, 0, 0), new Point3D(10, 10, 0)), Frenet());
        Vertex anIn = new BRepBuilderAPI_MakeVertex(new Point3D(0, 1, 0)).Vertex();
        aLaw.PerformVertex(1, anIn, 1e-7, out Vertex aPublic);
        aLaw.InternalPerformVertex(1, anIn, 1e-7, out Vertex anInternal);
        Assert.True(aPublic.TShape()!.IsFrozen);
        Assert.False(anInternal.TShape()!.IsFrozen);
        Assert.Equal(Tool.Pnt(aPublic), Tool.Pnt(anInternal));
        // the average of both laws at a corner has a tolerance covering half their gap
        Assert.True(Tool.Tolerance(aPublic) > 1e-7);
        aLaw.PerformVertex(1, anIn, 1e-7, out Vertex aBefore, -1);
        Assert.Equal(1e-7, Tool.Tolerance(aBefore));
    }

    [Fact]
    public void TransformInG0Law_makes_the_law_continuous_and_DeleteTransform_restores_it()
    {
        BRepFill_Edge3DLaw aLaw = new(Polyline(new Point3D(0, 0, 0), new Point3D(5, 0, 0), new Point3D(5, 5, 0)), Frenet());
        Assert.Equal(-1, aLaw.IsG1(1));
        aLaw.TransformInG0Law();
        Assert.NotEqual(-1, aLaw.IsG1(1, 1e-7));
        aLaw.DeleteTransform();
        Assert.Equal(-1, aLaw.IsG1(1));
    }

    [Fact]
    public void Closed_path_is_closed_and_IsG1_checks_its_closing_vertex()
    {
        BRepFill_Edge3DLaw aLaw = new(ClosedSquare(4), Frenet());
        Assert.True(aLaw.IsClosed());
        Assert.Equal(4, aLaw.NbLaw());
        Assert.Equal(-1, aLaw.IsG1(0));
        Assert.Equal(-1, aLaw.IsG1(4));
    }

    [Fact]
    public void ACRLaw_sets_each_edge_origin_to_its_reduced_abscissae()
    {
        Wire aSpine = Polyline(new Point3D(0, 0, 0), new Point3D(4, 0, 0), new Point3D(4, 0, 6));
        Transform3D aT = new();
        aT.SetTranslation(new Vector3D(0, 3, 0));
        Wire aGuide = ShapeCasts.Wire(new BRepBuilderAPI_Transform(aSpine, aT, true).Shape());
        GeomFill_LocationGuide aLoc = new(new GeomFill_GuideTrihedronAC(new WireCurveAdaptor(aGuide)));
        BRepFill_ACRLaw aLaw = new(aSpine, aLoc);
        Assert.Equal(2, aLaw.NbLaw());
        Assert.NotSame(aLoc, aLaw.Law(1));
        NCollection_Array1<double> anACR = new(0, 2);
        BRepFill.ComputeACR(aSpine, anACR);
        Assert.Equal(10.0, anACR[0], 1e-9);
        Assert.Equal(0.4, anACR[1], 1e-9);
        Matrix3x3 M = new();
        Vector3D V = new();
        aLaw.Law(1).GetDomain(out double f, out _);
        Assert.True(aLaw.Law(1).D0(f, ref M, ref V));
        Assert.True(new Point3D(V.XYZ()).IsEqual(new Point3D(0, 0, 0), 1e-7));
    }

    [Fact]
    public void EdgeOnSurfLaw_has_no_result_when_an_edge_is_off_the_shape()
    {
        CylindricalSurface aCyl = new(new CoordinateSystem(), 3);
        Face aCylFace = new BRepBuilderAPI_MakeFace(aCyl, 0, 6, 0, 10, 1e-7).Face();
        Wire aW = Polyline(new Point3D(0, 0, 0), new Point3D(5, 0, 0));
        BRepFill_EdgeOnSurfLaw aLaw = new(aW, aCylFace); // the segment has no pcurve on the cylinder
        Assert.False(aLaw.HasResult());

        Edge anE = new BRepBuilderAPI_MakeEdge(new LineCurve2D(new Point2D(0, 1), new Direction2D(1, 0.2)), aCyl, 0, 3).InternalEdge();
        BRepLib.BuildCurves3d(anE);
        BRepFill_EdgeOnSurfLaw aLaw2 = new(new BRepBuilderAPI_MakeWire(anE).Wire(), aCylFace);
        Assert.True(aLaw2.HasResult());
        Assert.Equal(1, aLaw2.NbLaw());
        Matrix3x3 M = new();
        Vector3D V = new();
        aLaw2.Law(1).D0(1.0, ref M, ref V);
        // Darboux: the normal (column 2) is the cylinder's normal, horizontal
        Assert.Equal(0.0, M[3, 2], 1e-9);
    }

    // ------------------------------------------------------------------------------------------------ section laws

    [Fact]
    public void ShapeLaw_on_a_vertex_is_a_constant_degenerate_section()
    {
        Vertex aV = new BRepBuilderAPI_MakeVertex(new Point3D(1, 2, 3)).Vertex();
        BRepFill_ShapeLaw aLaw = new(aV);
        Assert.True(aLaw.IsVertex());
        Assert.True(aLaw.IsConstant());
        Assert.True(aLaw.IsVClosed());
        Assert.False(aLaw.IsUClosed());
        Assert.Equal(1, aLaw.NbLaw());
        Shape S = new();
        aLaw.D0(0.3, ref S);
        Assert.True(S.IsSame(aV)); // the caller's vertex
    }

    [Fact]
    public void ShapeLaw_on_a_closed_wire_reverses_reversed_edges_into_new_curves()
    {
        Wire aW = ClosedSquare(2);
        string aBefore = Text(aW);
        BRepFill_ShapeLaw aLaw = new(aW);
        Assert.Equal(4, aLaw.NbLaw());
        Assert.True(aLaw.IsUClosed());
        Assert.Equal(C0, aLaw.Continuity(1, 1e-4));
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Confusion(), aLaw.VertexTol(1, 0), 1e-12);
        int i = 1;
        for (Explorer anExp = new(aW, ShapeType.Edge); anExp.More(); anExp.Next(), i++)
        {
            Assert.Equal(i, aLaw.IndexOfEdge(anExp.Current()));
        }
        GeomFill_SectionLaw? aC = aLaw.ConcatenedLaw();
        Assert.NotNull(aC);
        Assert.Equal(aBefore, Text(aW));
    }

    [Fact]
    public void ShapeLaw_with_a_law_scales_the_section_and_its_vertices()
    {
        Wire aW = Polyline(new Point3D(1, 0, 0), new Point3D(2, 0, 0));
        Law_Linear aL = new();
        aL.Set(0, 1, 1, 3);
        BRepFill_ShapeLaw aLaw = new(aW, aL);
        Assert.False(aLaw.IsConstant());
        Vertex aV = aLaw.Vertex(2, 0.5);
        Assert.True(Tool.Pnt(aV).IsEqual(new Point3D(4, 0, 0), 1e-9));
        Assert.True(aV.TShape()!.IsFrozen);
        Shape S = new();
        aLaw.D0(1, ref S);
        Assert.True(AllFrozen(S));
        Exploration.Vertices(ShapeCasts.Wire(S), out Vertex V1, out Vertex V2);
        Assert.True(Tool.Pnt(V1).IsEqual(new Point3D(3, 0, 0), 1e-9));
        Assert.True(Tool.Pnt(V2).IsEqual(new Point3D(6, 0, 0), 1e-9));
    }

    [Fact]
    public void NSections_interpolates_the_wires_and_keeps_its_surface_unfrozen()
    {
        NCollection_Sequence<Shape> aS = new();
        aS.Append(ClosedSquare(2, 0));
        aS.Append(ClosedSquare(3, 5));
        aS.Append(ClosedSquare(2, 10));
        string[] aBefore = [Text(aS[1]), Text(aS[2]), Text(aS[3])];
        BRepFill_NSections aLaw = new(aS);
        Assert.True(aLaw.IsDone());
        Assert.Equal(4, aLaw.NbLaw());
        Assert.True(aLaw.IsUClosed());
        Assert.False(aLaw.IsVClosed());
        Assert.False(aLaw.IsConstant());
        GeomFill_NSections aL1 = (GeomFill_NSections)aLaw.Law(1);
        Assert.False(aL1.InternalBSplineSurface()!.IsFrozen);

        // the section at the middle wire's parameter passes through its corner
        Vertex aV = aLaw.Vertex(1, 1.0);
        Assert.True(Tool.Pnt(aV).IsEqual(new Point3D(0, 0, 5), 1e-5));
        Assert.True(aV.TShape()!.IsFrozen);
        Shape W = new();
        aLaw.D0(1.0, ref W);
        Assert.True(AllFrozen(W));
        Assert.False(aL1.InternalBSplineSurface()!.IsFrozen);

        Assert.Equal(aBefore, new[] { Text(aS[1]), Text(aS[2]), Text(aS[3]) });
    }

    [Fact]
    public void NSections_with_decreasing_parameters_is_not_done()
    {
        NCollection_Sequence<Shape> aS = new();
        aS.Append(ClosedSquare(2, 0));
        aS.Append(ClosedSquare(3, 5));
        NCollection_Sequence<Transform3D> aT = new();
        NCollection_Sequence<double> aP = new();
        aP.Append(1);
        aP.Append(0);
        Assert.False(new BRepFill_NSections(aS, aT, aP, 0, 1).IsDone());
    }

    // ------------------------------------------------------------------------------------------------ sections

    [Fact]
    public void Section_of_a_located_wire_removes_its_locations()
    {
        Transform3D aT = new();
        aT.SetTranslation(new Vector3D(1, 2, 3));
        Wire aProfile = ShapeCasts.Wire(ClosedSquare(1).Moved(new Placement(aT)));
        Vertex aV = new BRepBuilderAPI_MakeVertex(new Point3D()).Vertex();
        BRepFill_Section aSec = new(aProfile, aV, true, false);
        Assert.False(aSec.IsPunctual());
        Assert.True(aSec.WithContact());
        Assert.False(aSec.WithCorrection());
        Assert.False(aSec.InternalWire().TShape()!.IsFrozen);
        Wire W = aSec.Wire();
        Assert.True(AllFrozen(W));
        for (Explorer anExp = new(W, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Assert.True(anExp.Current().Location().IsIdentity);
        }
        Explorer anOrig = new(aProfile, ShapeType.Edge);
        Shape aMod = aSec.ModifiedShape(anOrig.Current());
        Assert.Equal(ShapeType.Edge, aMod.ShapeType());
        Assert.True(aSec.ModifiedShape(aProfile).IsSame(W));
        Assert.True(aSec.OriginalShape().IsEqual(aProfile));

        BRepFill_Section aLaw = aSec.WithIsLaw(true);
        Assert.True(aLaw.IsLaw());
        Assert.False(aSec.IsLaw());
        Assert.NotEqual(aSec, aLaw);
        Assert.Equal(aSec, aLaw.WithIsLaw(false));
    }

    [Fact]
    public void Section_of_a_vertex_is_a_closed_wire_of_one_degenerated_edge()
    {
        Vertex aV = new BRepBuilderAPI_MakeVertex(new Point3D(1, 1, 1)).Vertex();
        BRepFill_Section aSec = new(aV, new Vertex(), false, false);
        Assert.True(aSec.IsPunctual());
        Wire W = aSec.Wire();
        Assert.True(W.Closed());
        Explorer anExp = new(W, ShapeType.Edge);
        Assert.True(Tool.Degenerated(ShapeCasts.Edge(anExp.Current())));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.FailureException>(() => new BRepFill_Section(new BRepBuilderAPI_MakeFace(new Plane3D(), 0, 1, 0, 1).Face(),
                                                                      aV, false, false));
    }

    [Fact]
    public void SectionPlacement_places_the_section_at_the_nearest_point_of_the_path()
    {
        Wire aW = Polyline(new Point3D(0, 0, 0), new Point3D(10, 0, 0), new Point3D(10, 10, 0));
        BRepFill_Edge3DLaw aLaw = new(aW, new GeomFill_CurveAndTrihedron(new GeomFill_Fixed(new Vector3D(1, 0, 0), new Vector3D(0, 1, 0))));
        Vertex aSec = new BRepBuilderAPI_MakeVertex(new Point3D(3, 0.5, 0)).Vertex();
        BRepFill_SectionPlacement aP = new(aLaw, aSec);
        Assert.Equal(3.0, aP.AbscissaOnPath(), 1e-6);
        BRepFill_SectionPlacement aOnV = new(aLaw, aSec, aLaw.Vertex(2));
        Assert.Equal(10.0, aOnV.AbscissaOnPath(), 1e-6);
        BRepFill_SectionPlacement aC = new(aLaw, aSec, true, false);
        Assert.Equal(3.0, aC.AbscissaOnPath(), 1e-6);
    }

    // ------------------------------------------------------------------------------------------------ RemoveLocations

    [Fact]
    public void RemoveLocations_rebuilds_a_located_box_without_locations()
    {
        Transform3D aT = new();
        aT.SetRotation(new Axis3D(new Point3D(), new Direction3D(0, 0, 1)), 0.3);
        Shape aBox = new BRepPrimAPI_MakeBox(1, 2, 3).Shape().Moved(new Placement(aT));
        string aBefore = Text(aBox);
        ShapeUpgrade_RemoveLocations aRL = new();
        Assert.Equal(ShapeType.Shape, aRL.RemoveLevel());
        Assert.True(aRL.Remove(aBox));
        Shape R = aRL.GetResult();
        Assert.True(AllFrozen(R));
        Assert.True(R.Location().IsIdentity);
        for (Explorer anExp = new(R, ShapeType.Face); anExp.More(); anExp.Next())
        {
            Tool.Surface(ShapeCasts.Face(anExp.Current()), out Placement L);
            Assert.True(L.IsIdentity);
        }
        Properties aG1 = new(), aG2 = new();
        BRepGProp.VolumeProperties(aBox, aG1);
        BRepGProp.VolumeProperties(R, aG2);
        Assert.Equal(aG1.Mass(), aG2.Mass(), 1e-9);
        Assert.True(aG1.CentreOfMass().IsEqual(aG2.CentreOfMass(), 1e-9));
        Assert.Equal(aBefore, Text(aBox));

        // levels below the face are the face
        aRL.SetRemoveLevel(ShapeType.Edge);
        Assert.Equal(ShapeType.Face, aRL.RemoveLevel());
        Assert.True(aRL.ModifiedShape(aBox).IsSame(R));
        Assert.True(aRL.GetModifiedShapesMap().Extent() > 0);
    }
}
