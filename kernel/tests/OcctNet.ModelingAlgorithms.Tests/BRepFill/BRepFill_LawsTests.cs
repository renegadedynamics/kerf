// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

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

    private static TopoDS_Wire Polyline(params gp_Pnt[] thePoints)
    {
        BRepBuilderAPI_MakePolygon aP = new();
        foreach (gp_Pnt aQ in thePoints)
        {
            aP.Add(aQ);
        }
        return aP.Wire();
    }

    private static TopoDS_Wire ClosedSquare(double theSide, double theZ = 0)
    {
        BRepBuilderAPI_MakePolygon aP = new(new gp_Pnt(0, 0, theZ), new gp_Pnt(theSide, 0, theZ),
                                            new gp_Pnt(theSide, theSide, theZ), new gp_Pnt(0, theSide, theZ), true);
        return aP.Wire();
    }

    // A smooth spine: one cubic B-spline cut into two edges at its middle (a G1 vertex).
    private static TopoDS_Wire SmoothSpine()
    {
        NCollection_Array1<gp_Pnt> aPoles = new(1, 5);
        aPoles[1] = new gp_Pnt(0, 0, 0);
        aPoles[2] = new gp_Pnt(3, 2, 0);
        aPoles[3] = new gp_Pnt(6, -1, 1);
        aPoles[4] = new gp_Pnt(9, 1, 2);
        aPoles[5] = new gp_Pnt(12, 0, 2);
        NCollection_Array1<double> aK = new(1, 3);
        aK[1] = 0;
        aK[2] = 1;
        aK[3] = 2;
        NCollection_Array1<int> aM = new(1, 3);
        aM[1] = 4;
        aM[2] = 1;
        aM[3] = 4;
        Geom_BSplineCurve aC = new(aPoles, aK, aM, 3);
        TopoDS_Vertex V0 = new BRepBuilderAPI_MakeVertex(aC.Value(0)).Vertex();
        TopoDS_Vertex V1 = new BRepBuilderAPI_MakeVertex(aC.Value(0.7)).Vertex();
        TopoDS_Vertex V2 = new BRepBuilderAPI_MakeVertex(aC.Value(2)).Vertex();
        BRepBuilderAPI_MakeWire aMW = new(new BRepBuilderAPI_MakeEdge(aC, V0, V1, 0, 0.7).Edge(),
                                          new BRepBuilderAPI_MakeEdge(aC, V1, V2, 0.7, 2).Edge());
        return aMW.Wire();
    }

    private static GeomFill_LocationLaw Frenet() => new GeomFill_CurveAndTrihedron(new GeomFill_CorrectedFrenet());

    private static string Text(in TopoDS_Shape theShape)
    {
        StringWriter aW = new();
        BRepTools.Write(theShape, aW);
        return aW.ToString();
    }

    private static bool AllFrozen(in TopoDS_Shape theShape)
    {
        NCollection_IndexedMap<TopoDS_Shape> aMap = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, aMap);
        for (int i = 1; i <= aMap.Extent(); i++)
        {
            if (!aMap.FindKey(i).TShape()!.IsFrozen())
            {
                return false;
            }
        }
        return theShape.TShape()!.IsFrozen();
    }

    // ------------------------------------------------------------------------------------------------ location laws

    [Fact]
    public void Edge3DLaw_on_a_polyline_has_a_law_per_edge_and_holes_at_its_corners()
    {
        TopoDS_Wire aW = Polyline(new gp_Pnt(0, 0, 0), new gp_Pnt(5, 0, 0), new gp_Pnt(5, 5, 0), new gp_Pnt(5, 5, 5));
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
        Assert.True(BRep_Tool.Pnt(aLaw.Vertex(1)).IsEqual(new gp_Pnt(0, 0, 0), 1e-12));
        Assert.True(BRep_Tool.Pnt(aLaw.Vertex(4)).IsEqual(new gp_Pnt(5, 5, 5), 1e-12));
        Assert.True(aLaw.Vertex(5).IsNull());
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
        BRepFill_Edge3DLaw aLaw = new(Polyline(new gp_Pnt(0, 0, 0), new gp_Pnt(5, 0, 0), new gp_Pnt(5, 5, 0)), aUser);
        Assert.NotSame(aUser, aLaw.Law(1));
        Assert.NotSame(aLaw.Law(1), aLaw.Law(2));
        Assert.Null(aUser.GetCurve()); // the caller's law is never given a curve
    }

    [Fact]
    public void Abscissa_before_the_lengths_are_known_takes_the_edge_end_as_OCCT_does()
    {
        // OCCT: Abscissa reads CurvilinearBounds' Last as the edge's start when the lengths aren't computed yet
        TopoDS_Wire aW = Polyline(new gp_Pnt(0, 0, 0), new gp_Pnt(4, 0, 0), new gp_Pnt(4, 3, 0));
        BRepFill_Edge3DLaw aFresh = new(aW, Frenet());
        aFresh.Law(2).GetDomain(out double f, out double l);
        double aBuggy = aFresh.Abscissa(2, 0.5 * (f + l));
        Assert.Equal(7 + 1.5, aBuggy, 1e-9); // 4 + 3 (the end of edge 2) + 1.5
        Assert.Equal(4 + 1.5, aFresh.Abscissa(2, 0.5 * (f + l)), 1e-9); // the lengths are now known
    }

    [Fact]
    public void D0_places_a_copy_of_the_section_and_freezes_it()
    {
        BRepFill_Edge3DLaw aLaw = new(Polyline(new gp_Pnt(0, 0, 0), new gp_Pnt(10, 0, 0)),
                                      new GeomFill_CurveAndTrihedron(new GeomFill_Fixed(new gp_Vec(1, 0, 0), new gp_Vec(0, 1, 0))));
        TopoDS_Shape aSection = new BRepBuilderAPI_MakeVertex(new gp_Pnt(0, 0, 0)).Vertex();
        TopoDS_Shape aOrig = aSection;
        aLaw.D0(4, ref aSection);
        Assert.False(aSection.IsSame(aOrig));
        Assert.True(AllFrozen(aSection));
        Assert.True(BRep_Tool.Pnt(TopoDS.Vertex(aSection)).IsEqual(new gp_Pnt(4, 0, 0), 1e-9));
        aLaw.D0(11, ref aSection);
        Assert.True(aSection.IsNull());
    }

    [Fact]
    public void PerformVertex_is_frozen_and_InternalPerformVertex_is_not()
    {
        BRepFill_Edge3DLaw aLaw = new(Polyline(new gp_Pnt(0, 0, 0), new gp_Pnt(10, 0, 0), new gp_Pnt(10, 10, 0)), Frenet());
        TopoDS_Vertex anIn = new BRepBuilderAPI_MakeVertex(new gp_Pnt(0, 1, 0)).Vertex();
        aLaw.PerformVertex(1, anIn, 1e-7, out TopoDS_Vertex aPublic);
        aLaw.InternalPerformVertex(1, anIn, 1e-7, out TopoDS_Vertex anInternal);
        Assert.True(aPublic.TShape()!.IsFrozen());
        Assert.False(anInternal.TShape()!.IsFrozen());
        Assert.Equal(BRep_Tool.Pnt(aPublic), BRep_Tool.Pnt(anInternal));
        // the average of both laws at a corner has a tolerance covering half their gap
        Assert.True(BRep_Tool.Tolerance(aPublic) > 1e-7);
        aLaw.PerformVertex(1, anIn, 1e-7, out TopoDS_Vertex aBefore, -1);
        Assert.Equal(1e-7, BRep_Tool.Tolerance(aBefore));
    }

    [Fact]
    public void TransformInG0Law_makes_the_law_continuous_and_DeleteTransform_restores_it()
    {
        BRepFill_Edge3DLaw aLaw = new(Polyline(new gp_Pnt(0, 0, 0), new gp_Pnt(5, 0, 0), new gp_Pnt(5, 5, 0)), Frenet());
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
        TopoDS_Wire aSpine = Polyline(new gp_Pnt(0, 0, 0), new gp_Pnt(4, 0, 0), new gp_Pnt(4, 0, 6));
        gp_Trsf aT = new();
        aT.SetTranslation(new gp_Vec(0, 3, 0));
        TopoDS_Wire aGuide = TopoDS.Wire(new BRepBuilderAPI_Transform(aSpine, aT, true).Shape());
        GeomFill_LocationGuide aLoc = new(new GeomFill_GuideTrihedronAC(new BRepAdaptor_CompCurve(aGuide)));
        BRepFill_ACRLaw aLaw = new(aSpine, aLoc);
        Assert.Equal(2, aLaw.NbLaw());
        Assert.NotSame(aLoc, aLaw.Law(1));
        NCollection_Array1<double> anACR = new(0, 2);
        BRepFill.ComputeACR(aSpine, anACR);
        Assert.Equal(10.0, anACR[0], 1e-9);
        Assert.Equal(0.4, anACR[1], 1e-9);
        gp_Mat M = new();
        gp_Vec V = new();
        aLaw.Law(1).GetDomain(out double f, out _);
        Assert.True(aLaw.Law(1).D0(f, ref M, ref V));
        Assert.True(new gp_Pnt(V.XYZ()).IsEqual(new gp_Pnt(0, 0, 0), 1e-7));
    }

    [Fact]
    public void EdgeOnSurfLaw_has_no_result_when_an_edge_is_off_the_shape()
    {
        Geom_CylindricalSurface aCyl = new(new gp_Ax3(), 3);
        TopoDS_Face aCylFace = new BRepBuilderAPI_MakeFace(aCyl, 0, 6, 0, 10, 1e-7).Face();
        TopoDS_Wire aW = Polyline(new gp_Pnt(0, 0, 0), new gp_Pnt(5, 0, 0));
        BRepFill_EdgeOnSurfLaw aLaw = new(aW, aCylFace); // the segment has no pcurve on the cylinder
        Assert.False(aLaw.HasResult());

        TopoDS_Edge anE = new BRepBuilderAPI_MakeEdge(new Geom2d_Line(new gp_Pnt2d(0, 1), new gp_Dir2d(1, 0.2)), aCyl, 0, 3).InternalEdge();
        BRepLib.BuildCurves3d(anE);
        BRepFill_EdgeOnSurfLaw aLaw2 = new(new BRepBuilderAPI_MakeWire(anE).Wire(), aCylFace);
        Assert.True(aLaw2.HasResult());
        Assert.Equal(1, aLaw2.NbLaw());
        gp_Mat M = new();
        gp_Vec V = new();
        aLaw2.Law(1).D0(1.0, ref M, ref V);
        // Darboux: the normal (column 2) is the cylinder's normal, horizontal
        Assert.Equal(0.0, M[3, 2], 1e-9);
    }

    // ------------------------------------------------------------------------------------------------ section laws

    [Fact]
    public void ShapeLaw_on_a_vertex_is_a_constant_degenerate_section()
    {
        TopoDS_Vertex aV = new BRepBuilderAPI_MakeVertex(new gp_Pnt(1, 2, 3)).Vertex();
        BRepFill_ShapeLaw aLaw = new(aV);
        Assert.True(aLaw.IsVertex());
        Assert.True(aLaw.IsConstant());
        Assert.True(aLaw.IsVClosed());
        Assert.False(aLaw.IsUClosed());
        Assert.Equal(1, aLaw.NbLaw());
        TopoDS_Shape S = new();
        aLaw.D0(0.3, ref S);
        Assert.True(S.IsSame(aV)); // the caller's vertex
    }

    [Fact]
    public void ShapeLaw_on_a_closed_wire_reverses_reversed_edges_into_new_curves()
    {
        TopoDS_Wire aW = ClosedSquare(2);
        string aBefore = Text(aW);
        BRepFill_ShapeLaw aLaw = new(aW);
        Assert.Equal(4, aLaw.NbLaw());
        Assert.True(aLaw.IsUClosed());
        Assert.Equal(GeomAbs_C0, aLaw.Continuity(1, 1e-4));
        Assert.Equal(Precision.Confusion(), aLaw.VertexTol(1, 0), 1e-12);
        int i = 1;
        for (TopExp_Explorer anExp = new(aW, TopAbs_EDGE); anExp.More(); anExp.Next(), i++)
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
        TopoDS_Wire aW = Polyline(new gp_Pnt(1, 0, 0), new gp_Pnt(2, 0, 0));
        Law_Linear aL = new();
        aL.Set(0, 1, 1, 3);
        BRepFill_ShapeLaw aLaw = new(aW, aL);
        Assert.False(aLaw.IsConstant());
        TopoDS_Vertex aV = aLaw.Vertex(2, 0.5);
        Assert.True(BRep_Tool.Pnt(aV).IsEqual(new gp_Pnt(4, 0, 0), 1e-9));
        Assert.True(aV.TShape()!.IsFrozen());
        TopoDS_Shape S = new();
        aLaw.D0(1, ref S);
        Assert.True(AllFrozen(S));
        TopExp.Vertices(TopoDS.Wire(S), out TopoDS_Vertex V1, out TopoDS_Vertex V2);
        Assert.True(BRep_Tool.Pnt(V1).IsEqual(new gp_Pnt(3, 0, 0), 1e-9));
        Assert.True(BRep_Tool.Pnt(V2).IsEqual(new gp_Pnt(6, 0, 0), 1e-9));
    }

    [Fact]
    public void NSections_interpolates_the_wires_and_keeps_its_surface_unfrozen()
    {
        NCollection_Sequence<TopoDS_Shape> aS = new();
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
        Assert.False(aL1.InternalBSplineSurface()!.IsFrozen());

        // the section at the middle wire's parameter passes through its corner
        TopoDS_Vertex aV = aLaw.Vertex(1, 1.0);
        Assert.True(BRep_Tool.Pnt(aV).IsEqual(new gp_Pnt(0, 0, 5), 1e-5));
        Assert.True(aV.TShape()!.IsFrozen());
        TopoDS_Shape W = new();
        aLaw.D0(1.0, ref W);
        Assert.True(AllFrozen(W));
        Assert.False(aL1.InternalBSplineSurface()!.IsFrozen());

        Assert.Equal(aBefore, new[] { Text(aS[1]), Text(aS[2]), Text(aS[3]) });
    }

    [Fact]
    public void NSections_with_decreasing_parameters_is_not_done()
    {
        NCollection_Sequence<TopoDS_Shape> aS = new();
        aS.Append(ClosedSquare(2, 0));
        aS.Append(ClosedSquare(3, 5));
        NCollection_Sequence<gp_Trsf> aT = new();
        NCollection_Sequence<double> aP = new();
        aP.Append(1);
        aP.Append(0);
        Assert.False(new BRepFill_NSections(aS, aT, aP, 0, 1).IsDone());
    }

    // ------------------------------------------------------------------------------------------------ sections

    [Fact]
    public void Section_of_a_located_wire_removes_its_locations()
    {
        gp_Trsf aT = new();
        aT.SetTranslation(new gp_Vec(1, 2, 3));
        TopoDS_Wire aProfile = TopoDS.Wire(ClosedSquare(1).Moved(new TopLoc_Location(aT)));
        TopoDS_Vertex aV = new BRepBuilderAPI_MakeVertex(new gp_Pnt()).Vertex();
        BRepFill_Section aSec = new(aProfile, aV, true, false);
        Assert.False(aSec.IsPunctual());
        Assert.True(aSec.WithContact());
        Assert.False(aSec.WithCorrection());
        Assert.False(aSec.InternalWire().TShape()!.IsFrozen());
        TopoDS_Wire W = aSec.Wire();
        Assert.True(AllFrozen(W));
        for (TopExp_Explorer anExp = new(W, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            Assert.True(anExp.Current().Location().IsIdentity());
        }
        TopExp_Explorer anOrig = new(aProfile, TopAbs_EDGE);
        TopoDS_Shape aMod = aSec.ModifiedShape(anOrig.Current());
        Assert.Equal(TopAbs_EDGE, aMod.ShapeType());
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
        TopoDS_Vertex aV = new BRepBuilderAPI_MakeVertex(new gp_Pnt(1, 1, 1)).Vertex();
        BRepFill_Section aSec = new(aV, new TopoDS_Vertex(), false, false);
        Assert.True(aSec.IsPunctual());
        TopoDS_Wire W = aSec.Wire();
        Assert.True(W.Closed());
        TopExp_Explorer anExp = new(W, TopAbs_EDGE);
        Assert.True(BRep_Tool.Degenerated(TopoDS.Edge(anExp.Current())));
        Assert.ThrowsAny<Standard_Failure>(() => new BRepFill_Section(new BRepBuilderAPI_MakeFace(new gp_Pln(), 0, 1, 0, 1).Face(),
                                                                      aV, false, false));
    }

    [Fact]
    public void SectionPlacement_places_the_section_at_the_nearest_point_of_the_path()
    {
        TopoDS_Wire aW = Polyline(new gp_Pnt(0, 0, 0), new gp_Pnt(10, 0, 0), new gp_Pnt(10, 10, 0));
        BRepFill_Edge3DLaw aLaw = new(aW, new GeomFill_CurveAndTrihedron(new GeomFill_Fixed(new gp_Vec(1, 0, 0), new gp_Vec(0, 1, 0))));
        TopoDS_Vertex aSec = new BRepBuilderAPI_MakeVertex(new gp_Pnt(3, 0.5, 0)).Vertex();
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
        gp_Trsf aT = new();
        aT.SetRotation(new gp_Ax1(new gp_Pnt(), new gp_Dir(0, 0, 1)), 0.3);
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(1, 2, 3).Shape().Moved(new TopLoc_Location(aT));
        string aBefore = Text(aBox);
        ShapeUpgrade_RemoveLocations aRL = new();
        Assert.Equal(TopAbs_SHAPE, aRL.RemoveLevel());
        Assert.True(aRL.Remove(aBox));
        TopoDS_Shape R = aRL.GetResult();
        Assert.True(AllFrozen(R));
        Assert.True(R.Location().IsIdentity());
        for (TopExp_Explorer anExp = new(R, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            BRep_Tool.Surface(TopoDS.Face(anExp.Current()), out TopLoc_Location L);
            Assert.True(L.IsIdentity());
        }
        GProp_GProps aG1 = new(), aG2 = new();
        BRepGProp.VolumeProperties(aBox, aG1);
        BRepGProp.VolumeProperties(R, aG2);
        Assert.Equal(aG1.Mass(), aG2.Mass(), 1e-9);
        Assert.True(aG1.CentreOfMass().IsEqual(aG2.CentreOfMass(), 1e-9));
        Assert.Equal(aBefore, Text(aBox));

        // levels below the face are the face
        aRL.SetRemoveLevel(TopAbs_EDGE);
        Assert.Equal(TopAbs_FACE, aRL.RemoveLevel());
        Assert.True(aRL.ModifiedShape(aBox).IsSame(R));
        Assert.True(aRL.GetModifiedShapesMap().Extent() > 0);
    }
}
