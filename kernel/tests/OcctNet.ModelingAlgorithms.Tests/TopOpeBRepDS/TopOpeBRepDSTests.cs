// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;
using static Occt.TopOpeBRepDS_Config;
using static Occt.TopOpeBRepDS_Kind;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// The legacy data structure's core (unit 7L) against OCCT's semantics, on data structures filled by hand: transitions,
/// interferences, the data structure's shapes, points, curves and surfaces, its iterators and explorers, the handle's
/// storing and sorting functions, and the text dump's round trip. Native data structures are in
/// <see cref="TopOpeBRepDSNativeTests"/>.
/// </summary>
public class TopOpeBRepDSTests
{
    private static readonly Shape theBox = new BRepPrimAPI_MakeBox(10.0, 20.0, 30.0).Shape();

    private static Shape Sub(in Shape S, ShapeType T, int k)
    {
        NCollection_IndexedMap<Shape> M = new(global::Kerf.Engine.Topology.ShapeHasher.Instance);
        global::Kerf.Engine.Topology.Exploration.MapShapes(S, T, M);
        return M.FindKey(k);
    }

    // ---- Transition ----

    [Fact]
    public void Transition_DefaultIsUnknownOnFaces()
    {
        TopOpeBRepDS_Transition T = new();
        Assert.Equal(Unknown, T.Before());
        Assert.Equal(Unknown, T.After());
        Assert.Equal(ShapeType.Face, T.ShapeBefore());
        Assert.Equal(ShapeType.Face, T.ONAfter());
        Assert.True(T.IsUnknown());
        Assert.Equal(0, T.Index());
        Assert.Equal("UNKNOWN/UNKNOWN FACE/FACE 0/0", T.ToString());
    }

    [Theory]
    [InlineData(Forward, Out, In)]
    [InlineData(Reversed, In, Out)]
    [InlineData(Internal, In, In)]
    [InlineData(External, Out, Out)]
    public void Transition_OfAnOrientation(Orientation O, Containment theBefore, Containment theAfter)
    {
        TopOpeBRepDS_Transition T = new(O);
        Assert.Equal(theBefore, T.Before());
        Assert.Equal(theAfter, T.After());
        // the orientation for IN gives the orientation back
        Assert.Equal(O, T.Orientation(In));
        // and the complement exchanges IN and OUT
        Assert.Equal(O, T.Complement().Orientation(Out));
    }

    [Fact]
    public void Transition_OrientationWithOnStates()
    {
        Assert.Equal(Internal, new TopOpeBRepDS_Transition(On, On).Orientation(In));
        Assert.Equal(External, new TopOpeBRepDS_Transition(On, On).Orientation(Out));
        Assert.Equal(Internal, new TopOpeBRepDS_Transition(On, On).Orientation(On));
        // OCCT: (ON, ON) for UNKNOWN keeps the default FORWARD
        Assert.Equal(Forward, new TopOpeBRepDS_Transition(On, On).Orientation(Unknown));
        Assert.Equal(Forward, new TopOpeBRepDS_Transition(On, In).Orientation(In));
        Assert.Equal(Reversed, new TopOpeBRepDS_Transition(On, Out).Orientation(In));
        Assert.Equal(Reversed, new TopOpeBRepDS_Transition(In, On).Orientation(In));
        Assert.Equal(Forward, new TopOpeBRepDS_Transition(Out, On).Orientation(In));

        // the complement of an ON transition exchanges before and after, states and shapes
        TopOpeBRepDS_Transition T = new(On, In, ShapeType.Edge, ShapeType.Face);
        TopOpeBRepDS_Transition C = T.Complement();
        Assert.Equal(In, C.Before());
        Assert.Equal(On, C.After());
        Assert.Equal(ShapeType.Face, C.ShapeBefore());
        Assert.Equal(ShapeType.Edge, C.ShapeAfter());
    }

    [Fact]
    public void Transition_IndexNeedsEqualIndices()
    {
        TopOpeBRepDS_Transition T = new TopOpeBRepDS_Transition(Out, In).WithIndex(3);
        Assert.Equal(3, T.Index());
        T.IndexAfter(4);
        Assert.Equal(3, T.IndexBefore());
        Assert.Equal(4, T.IndexAfter());
        Assert.Throws<global::Kerf.Engine.Foundation.FailureException>(() => T.Index());
        // the complement keeps the indices
        Assert.Equal(4, T.Complement().IndexAfter());
    }

    // ---- Interferences ----

    [Fact]
    public void Interferences_HoldTheirData()
    {
        TopOpeBRepDS_Transition T = new(Forward);
        TopOpeBRepDS_CurvePointInterference CPI = new(T, TopOpeBRepDS_CURVE, 2, TopOpeBRepDS_POINT, 5, 1.5);
        CPI.GKGSKS(out TopOpeBRepDS_Kind GK, out int G, out TopOpeBRepDS_Kind SK, out int S);
        Assert.Equal((TopOpeBRepDS_POINT, 5, TopOpeBRepDS_CURVE, 2), (GK, G, SK, S));
        Assert.Equal(1.5, CPI.Parameter());
        Assert.Equal(T, CPI.Transition());
        CPI.ChangeTransition().Index(7);
        Assert.Equal(7, CPI.Transition().Index());

        TopOpeBRepDS_EdgeVertexInterference EVI = new(T, 3, 4, true, TopOpeBRepDS_DIFFORIENTED, 0.25);
        Assert.Equal(TopOpeBRepDS_EDGE, EVI.SupportType());
        Assert.Equal(TopOpeBRepDS_VERTEX, EVI.GeometryType());
        Assert.True(EVI.GBound());
        Assert.Equal(TopOpeBRepDS_DIFFORIENTED, EVI.Config());

        TopOpeBRepDS_FaceEdgeInterference FEI = new(T, 1, 2, false, TopOpeBRepDS_SAMEORIENTED);
        Assert.Equal(TopOpeBRepDS_FACE, FEI.SupportType());
        Assert.Equal(TopOpeBRepDS_EDGE, FEI.GeometryType());

        LineCurve2D PC = new(new Point2D(0, 0), new Direction2D(1, 0));
        TopOpeBRepDS_SurfaceCurveInterference SCI = new(T, TopOpeBRepDS_FACE, 1, TopOpeBRepDS_CURVE, 3, PC);
        Assert.Same(PC, SCI.PCurve());
        // the copy keeps the transition, support and geometry, not the pcurve
        TopOpeBRepDS_SurfaceCurveInterference SCI2 = new(SCI);
        Assert.Null(SCI2.PCurve());
        Assert.True(SCI2.HasSameGeometry(SCI));
        Assert.True(SCI2.HasSameSupport(SCI));
        Assert.False(SCI2.HasSameGeometry(CPI));
    }

    // ---- The data structure: shapes ----

    [Fact]
    public void DataStructure_ShapesAreIndexedOnceWhateverTheirOrientation()
    {
        TopOpeBRepDS_DataStructure DS = new();
        Shape F1 = Sub(theBox, ShapeType.Face, 1);
        Assert.Equal(1, DS.AddShape(F1));
        Assert.Equal(1, DS.AddShape(F1.Reversed(), 2));
        Assert.Equal(0, DS.AncestorRank(1)); // a shape already there keeps its rank
        Assert.Equal(2, DS.AddShape(theBox, 1));
        Assert.Equal(1, DS.AncestorRank(theBox));
        Assert.Equal(2, DS.NbShapes());

        // a new shape is its own same-domain reference, same oriented
        Assert.Equal(1, DS.SameDomainRef(1));
        Assert.Equal(TopOpeBRepDS_SAMEORIENTED, DS.SameDomainOri(F1));
        Assert.Equal(0, DS.SameDomainInd(1));
        Assert.Equal(TopOpeBRepDS_UNSHGEOMETRY, DS.SameDomainOri(5));

        Assert.Equal(F1.Orientation(), DS.Shape(1).Orientation());
        Assert.Equal(1, DS.Shape(F1.Reversed()));
        Assert.True(DS.Shape(3).IsNull);
        Assert.False(DS.HasGeometry(F1));

        // a shape not kept is hidden from the default queries
        DS.ChangeKeepShape(1, false);
        Assert.True(DS.Shape(1).IsNull);
        Assert.False(DS.Shape(1, false).IsNull);
        Assert.Equal(0, DS.Shape(F1));
        Assert.Equal(1, DS.Shape(F1, false));
        Assert.False(DS.HasShape(F1));
    }

    [Fact]
    public void DataStructure_ShapeInterferences()
    {
        TopOpeBRepDS_DataStructure DS = new();
        Shape F1 = Sub(theBox, ShapeType.Face, 1);
        int iF = DS.AddShape(F1);
        TopOpeBRepDS_Interference I1 = new(new TopOpeBRepDS_Transition(), TopOpeBRepDS_FACE, iF, TopOpeBRepDS_CURVE, 1);
        TopOpeBRepDS_Interference I2 = new(new TopOpeBRepDS_Transition(), TopOpeBRepDS_FACE, iF, TopOpeBRepDS_CURVE, 2);
        DS.AddShapeInterference(F1, I1);
        DS.ChangeShapeInterferences(iF).Append(I2);
        Assert.True(DS.HasGeometry(F1));
        Assert.Equal(2, DS.ShapeInterferences(F1).Extent());
        Assert.Same(DS.ShapeInterferences(F1), DS.ShapeInterferences(iF));

        DS.RemoveShapeInterference(F1, I1);
        Assert.Same(I2, DS.ShapeInterferences(F1).First());

        // not kept: the default query gives an empty list
        DS.ChangeKeepShape(F1, false);
        Assert.True(DS.ShapeInterferences(F1).IsEmpty());
        Assert.Equal(1, DS.ShapeInterferences(F1, false).Extent());
    }

    [Fact]
    public void DataStructure_TheSharedEmptyListTakesWhatIsWrittenThere()
    {
        // OCCT: the Change* accessors give a shape not in the data structure the data structure's own empty list, for
        // writing; what is appended there is then every missing shape's list (decision 1066, occt-issues 7L)
        TopOpeBRepDS_DataStructure DS = new();
        Shape F1 = Sub(theBox, ShapeType.Face, 1), F2 = Sub(theBox, ShapeType.Face, 2);
        TopOpeBRepDS_Interference I = new(new TopOpeBRepDS_Transition(), TopOpeBRepDS_FACE, 1, TopOpeBRepDS_CURVE, 1);
        DS.AddShapeInterference(F1, I);
        Assert.Equal(0, DS.NbShapes());
        Assert.Same(I, DS.ShapeInterferences(F2).First());
        Assert.Same(I, DS.CurveInterferences(9).First());

        // other data structures have their own
        Assert.True(new TopOpeBRepDS_DataStructure().ShapeInterferences(F2).IsEmpty());
    }

    [Fact]
    public void DataStructure_SameDomainFaces()
    {
        // two boxes, one on the other: the top of the first and the bottom of the second share their plane
        Shape B1 = new BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();
        Shape B2 = new BRepPrimAPI_MakeBox(new Point3D(0, 0, 10), 10.0, 10.0, 10.0).Shape();
        Shape Top = TopoDS_Shape_Of(B1, 10.0), Bottom = TopoDS_Shape_Of(B2, 10.0);

        TopOpeBRepDS_HDataStructure HDS = new();
        TopOpeBRepDS_DataStructure DS = HDS.ChangeDS();
        DS.FillShapesSameDomain(Top, Bottom);
        int i1 = DS.Shape(Top), i2 = DS.Shape(Bottom);
        Assert.Equal((1, 2), (i1, i2));
        Assert.Equal((1, 2), (DS.AncestorRank(Top), DS.AncestorRank(Bottom)));
        Assert.Equal((1, 2), (DS.SameDomainInd(Top), DS.SameDomainInd(Bottom)));
        Assert.Equal(i1, DS.SameDomainRef(Bottom));
        Assert.Equal(i1, HDS.SameDomainReference(Top));
        Assert.Equal(TopOpeBRepDS_SAMEORIENTED, DS.SameDomainOri(Top));
        // their outward normals are opposite
        Assert.Equal(TopOpeBRepDS_DIFFORIENTED, HDS.SameDomainOrientation(Bottom));
        Assert.True(HDS.HasSameDomain(Top));
        NCollection_List<Shape>.Iterator it = HDS.SameDomain(Top);
        Assert.True(it.More() && it.Value().IsSame(Bottom));

        // filling again changes nothing
        DS.FillShapesSameDomain(Top, Bottom);
        Assert.Equal(1, DS.ShapeSameDomain(Top).Extent());

        DS.UnfillShapesSameDomain(Top, Bottom);
        Assert.False(HDS.HasSameDomain(Top));
    }

    // the face of the box at height z
    private static Shape TopoDS_Shape_Of(in Shape theBox, double z)
    {
        for (global::Kerf.Engine.Topology.Explorer ex = new(theBox, ShapeType.Face); ex.More(); ex.Next())
        {
            FaceSurfaceAdaptor S = new(global::Kerf.Engine.Topology.ShapeCasts.Face(ex.Current()));
            if (S.GetType() == SurfaceKind.Plane && Math.Abs(S.Plane().Axis().Direction().Z()) > 0.5
                && Math.Abs(S.Plane().Location().Z() - z) < 1.0e-9)
            {
                return ex.Current();
            }
        }
        throw new InvalidOperationException();
    }

    // ---- The data structure: points, curves, surfaces and their explorers ----

    [Fact]
    public void DataStructure_Points()
    {
        TopOpeBRepDS_DataStructure DS = new();
        Assert.Throws<global::Kerf.Engine.Foundation.ProgramErrorException>(() => DS.Point(1));
        Assert.Equal(1, DS.AddPoint(new TopOpeBRepDS_Point(new Point3D(1, 2, 3), 0.1)));
        Assert.Equal(2, DS.AddPointSS(new TopOpeBRepDS_Point(new Point3D(1, 2, 3.05), 0.01), theBox, theBox));
        Assert.True(DS.Point(1).IsEqual(DS.Point(2))); // within the larger tolerance
        Assert.False(DS.Point(2).IsEqual(new TopOpeBRepDS_Point(new Point3D(1, 2, 3.2), 0.01)));
        Assert.Throws<global::Kerf.Engine.Foundation.ProgramErrorException>(() => DS.Point(3));

        DS.ChangePoint(1).Tolerance(0.5);
        Assert.Equal(0.5, DS.Point(1).Tolerance());
        DS.RemovePoint(1);
        Assert.False(DS.KeepPoint(1));

        TopOpeBRepDS_PointExplorer pex = new(DS);
        Assert.True(pex.More());
        Assert.Equal(2, pex.Index());
        pex.Next();
        Assert.False(pex.More());
        Assert.Equal(1, pex.NbPoint());
        Assert.Equal(2, new TopOpeBRepDS_PointExplorer(DS, false).NbPoint());

        // a vertex's point and tolerance
        Shape V = Sub(theBox, ShapeType.Vertex, 1);
        TopOpeBRepDS_Point PV = new(V);
        Assert.True(PV.Point().IsEqual(Tool.Pnt(global::Kerf.Engine.Topology.ShapeCasts.Vertex(V)), 0.0));
        Assert.Equal(Tool.Tolerance(global::Kerf.Engine.Topology.ShapeCasts.Vertex(V)), PV.Tolerance());
        Assert.Throws<global::Kerf.Engine.Foundation.ProgramErrorException>(() => new TopOpeBRepDS_Point(theBox));
    }

    [Fact]
    public void DataStructure_Curves()
    {
        TopOpeBRepDS_DataStructure DS = new();
        Shape F1 = Sub(theBox, ShapeType.Face, 1), F2 = Sub(theBox, ShapeType.Face, 3);
        DS.AddShape(F1);
        DS.AddShape(F2);

        LineCurve3D L = new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        TopOpeBRepDS_Curve C = new(L, 1.0e-5);
        Assert.Equal(0, C.DSIndex());
        Assert.False(C.Range(out _, out _));
        Assert.True(C.Keep());
        Assert.Null(C.Curve1());

        TopOpeBRepDS_SurfaceCurveInterference SCI1 = new(new TopOpeBRepDS_Transition(Forward), TopOpeBRepDS_FACE, 1,
                                                         TopOpeBRepDS_CURVE, 1, new LineCurve2D(new Point2D(), new Direction2D()));
        TopOpeBRepDS_SurfaceCurveInterference SCI2 = new(new TopOpeBRepDS_Transition(Reversed), TopOpeBRepDS_FACE, 2,
                                                         TopOpeBRepDS_CURVE, 1, null);
        C.SetShapes(F1, F2);
        C.SetSCI(SCI1, SCI2);
        DS.AddShapeInterference(F1, SCI1);
        DS.AddShapeInterference(F2, SCI2);

        // OCCT sets the caller's index too
        Assert.Equal(1, DS.AddCurve(ref C));
        Assert.Equal(1, C.DSIndex());
        Assert.Equal(2, DS.AddCurve(new TopOpeBRepDS_Curve(L, 1.0e-6)));
        Assert.Equal(2, DS.Curve(2).DSIndex());
        Assert.Same(SCI1.PCurve(), DS.Curve(1).Curve1());
        Assert.Null(DS.Curve(1).Curve2());

        DS.ChangeCurve(1).SetRange(0.0, 5.0);
        Assert.True(DS.Curve(1).Range(out double f, out double l));
        Assert.Equal((0.0, 5.0), (f, l));
        Assert.False(C.Range(out _, out _)); // the stored curve is a copy

        // a curve that isn't there is the data structure's empty curve
        Assert.Null(DS.Curve(7).Curve());
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Confusion(), DS.Curve(7).Tolerance());

        // removing a curve takes its interferences off its shapes and stops keeping it
        DS.RemoveCurve(1);
        Assert.False(DS.KeepCurve(1));
        Assert.True(DS.ShapeInterferences(F1).IsEmpty());
        Assert.True(DS.ShapeInterferences(F2).IsEmpty());
        Assert.Equal(2, DS.NbCurves());

        TopOpeBRepDS_CurveExplorer cex = new(DS);
        Assert.Equal(2, cex.Index());
        Assert.Equal(1.0e-6, cex.Curve().Tolerance());
        Assert.Equal(1, cex.NbCurve());
        Assert.False(cex.More());
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Confusion(), cex.Curve().Tolerance()); // the empty curve
        Assert.Equal(2, new TopOpeBRepDS_CurveExplorer(DS, false).NbCurve());
        Assert.True(cex.IsCurve(1));
        Assert.False(cex.IsCurveKeep(1));
    }

    [Fact]
    public void DataStructure_SurfacesAreNotKeptByDefault()
    {
        TopOpeBRepDS_DataStructure DS = new();
        PlaneSurface P = new(new CoordinateSystem());
        Assert.Equal(1, DS.AddSurface(new TopOpeBRepDS_Surface(P, 1.0e-7)));
        Assert.Equal(2, DS.AddSurface(new TopOpeBRepDS_Surface(P, 1.0e-6)));
        Assert.False(DS.KeepSurface(1));
        Assert.Equal(0, new TopOpeBRepDS_SurfaceExplorer(DS).NbSurface());
        DS.ChangeKeepSurface(2, true);
        TopOpeBRepDS_SurfaceExplorer sex = new(DS);
        Assert.True(sex.More());
        Assert.Equal(2, sex.Index());
        Assert.Same(P, sex.Surface().Surface());

        DS.RemoveSurface(1);
        Assert.Null(DS.Surface(1).Surface());
        Assert.Equal(2, DS.NbSurfaces());
        Assert.Equal(1, new TopOpeBRepDS_SurfaceExplorer(DS, false).NbSurface());

        Shape F1 = Sub(theBox, ShapeType.Face, 1);
        Assert.False(DS.HasNewSurface(F1));
        DS.SetNewSurface(F1, P);
        Assert.True(DS.HasNewSurface(F1.Reversed()));
        Assert.Same(P, DS.NewSurface(F1));
    }

    [Fact]
    public void DataStructure_InitEmptiesIt()
    {
        TopOpeBRepDS_DataStructure DS = new();
        DS.AddShape(theBox);
        DS.AddPoint(new TopOpeBRepDS_Point());
        DS.AddCurve(new TopOpeBRepDS_Curve());
        DS.AddSurface(new TopOpeBRepDS_Surface());
        DS.AddSectionEdge(global::Kerf.Engine.Topology.ShapeCasts.Edge(Sub(theBox, ShapeType.Edge, 1)));
        Assert.Equal(1, DS.NbSectionEdges());
        // a section edge is one only once it is a shape of the data structure too (OCCT)
        Assert.False(DS.IsSectionEdge(global::Kerf.Engine.Topology.ShapeCasts.Edge(Sub(theBox, ShapeType.Edge, 1)), false));
        DS.AddShape(Sub(theBox, ShapeType.Edge, 1));
        Assert.True(DS.IsSectionEdge(global::Kerf.Engine.Topology.ShapeCasts.Edge(Sub(theBox, ShapeType.Edge, 1))));
        Assert.Equal(1, DS.SectionEdge(global::Kerf.Engine.Topology.ShapeCasts.Edge(Sub(theBox, ShapeType.Edge, 1))));
        DS.Init();
        Assert.Equal((0, 0, 0, 0, 0), (DS.NbShapes(), DS.NbPoints(), DS.NbCurves(), DS.NbSurfaces(), DS.NbSectionEdges()));
    }

    [Fact]
    public void DataStructure_ShapesWithStateHaveAnEmptyMapPerDataStructure()
    {
        TopOpeBRepDS_DataStructure DS = new();
        NCollection_IndexedDataMap<Shape, TopOpeBRepDS_ShapeWithState> M = DS.ChangeMapOfShapeWithState(theBox, out bool aFound);
        Assert.False(aFound);
        Assert.True(M.IsEmpty());
        TopOpeBRepDS_ShapeWithState SWS = new();
        SWS.AddPart(theBox, In);
        SWS.SetState(On);
        DS.ChangeMapOfShapeWithStateObj().Add(theBox, SWS);
        Assert.Same(DS.ChangeMapOfShapeWithStateObj(), DS.ChangeMapOfShapeWithState(theBox, out aFound));
        Assert.True(aFound);
        Assert.Equal(On, DS.GetShapeWithState(theBox).State());
        Assert.Equal(1, DS.GetShapeWithState(theBox).Part(In).Extent());
        Assert.True(DS.GetShapeWithState(theBox).Part(Unknown).IsEmpty());
        Assert.Equal(Unknown, new TopOpeBRepDS_DataStructure().GetShapeWithState(theBox).State());
    }

    // ---- Iterators ----

    private static NCollection_List<TopOpeBRepDS_Interference> MixedList()
    {
        TopOpeBRepDS_Transition F = new(Forward), R = new(Reversed);
        NCollection_List<TopOpeBRepDS_Interference> L = new();
        L.Append(new TopOpeBRepDS_SurfaceCurveInterference(F, TopOpeBRepDS_FACE, 1, TopOpeBRepDS_CURVE, 4, null));
        L.Append(new TopOpeBRepDS_CurvePointInterference(R, TopOpeBRepDS_CURVE, 1, TopOpeBRepDS_POINT, 2, 3.0));
        L.Append(new TopOpeBRepDS_SolidSurfaceInterference(F, TopOpeBRepDS_SOLID, 1, TopOpeBRepDS_SURFACE, 7));
        L.Append(new TopOpeBRepDS_EdgeVertexInterference(F, 1, 5, true, TopOpeBRepDS_SAMEORIENTED, 1.0));
        L.Append(new TopOpeBRepDS_CurvePointInterference(F, TopOpeBRepDS_CURVE, 1, TopOpeBRepDS_POINT, 3, 2.0));
        return L;
    }

    [Fact]
    public void Iterators_MatchTheirGeometries()
    {
        NCollection_List<TopOpeBRepDS_Interference> L = MixedList();

        List<int> aPoints = [];
        List<double> aParams = [];
        for (TopOpeBRepDS_PointIterator it = new(L); it.More(); it.Next())
        {
            aPoints.Add(it.Current());
            aParams.Add(it.Parameter());
        }
        Assert.Equal([2, 5, 3], aPoints);
        Assert.Equal([3.0, 1.0, 2.0], aParams);

        TopOpeBRepDS_PointIterator pit = new(L);
        Assert.True(pit.IsPoint());
        Assert.Equal(Reversed, pit.Orientation(In));
        Assert.Throws<global::Kerf.Engine.Foundation.ProgramErrorException>(() => pit.SameOriented());
        pit.Next();
        Assert.True(pit.IsVertex());
        Assert.True(pit.SameOriented());
        Assert.False(pit.DiffOriented());
        Assert.Equal(1, pit.Support());

        TopOpeBRepDS_CurveIterator cit = new(L);
        Assert.Equal(4, cit.Current());
        Assert.Null(cit.PCurve());
        cit.Next();
        Assert.False(cit.More());

        TopOpeBRepDS_InterferenceIterator iit = new(L);
        iit.SupportKind(TopOpeBRepDS_CURVE);
        iit.Geometry(3);
        iit.Match();
        Assert.Equal(TopOpeBRepDS_POINT, iit.Value().GeometryType());
        Assert.Equal(3, iit.Value().Geometry());
    }

    [Fact]
    public void Iterators_TheSurfaceIteratorsFirstValueIsUnmatched()
    {
        // OCCT sets the SURFACE condition without matching again: the first value is the list's first (occt-issues 7L)
        NCollection_List<TopOpeBRepDS_Interference> L = MixedList();
        TopOpeBRepDS_SurfaceIterator sit = new(L);
        Assert.Equal(TopOpeBRepDS_CURVE, sit.Value().GeometryType());
        sit.Next();
        Assert.Equal(7, sit.Current());
        Assert.Equal(Forward, sit.Orientation(In));
        sit.Next();
        Assert.False(sit.More());
    }

    // ---- The handle ----

    [Fact]
    public void HDataStructure_StoresOnShapeAndGeometry()
    {
        TopOpeBRepDS_HDataStructure HDS = new();
        TopOpeBRepDS_DataStructure DS = HDS.ChangeDS();
        Shape F1 = Sub(theBox, ShapeType.Face, 1), E1 = Sub(theBox, ShapeType.Edge, 1);
        int iF = DS.AddShape(F1);
        DS.AddShape(E1);
        int iC = DS.AddCurve(new TopOpeBRepDS_Curve(new LineCurve3D(new Axis3D()), 0.0));
        int iS = DS.AddSurface(new TopOpeBRepDS_Surface(new PlaneSurface(new CoordinateSystem()), 0.0));

        TopOpeBRepDS_SurfaceCurveInterference SCI = new(new TopOpeBRepDS_Transition(Forward), TopOpeBRepDS_FACE, iF,
                                                        TopOpeBRepDS_CURVE, iC, null);
        HDS.StoreInterference(SCI, F1);
        Assert.Same(SCI, DS.ShapeInterferences(F1).First());
        Assert.Same(SCI, DS.CurveInterferences(iC).First());

        TopOpeBRepDS_SolidSurfaceInterference SSI = new(new TopOpeBRepDS_Transition(Forward), TopOpeBRepDS_FACE, iF,
                                                        TopOpeBRepDS_SURFACE, iS);
        HDS.StoreInterference(SSI, iF);
        Assert.Same(SSI, DS.SurfaceInterferences(iS).First());

        // a point interference goes on the shape only
        TopOpeBRepDS_CurvePointInterference CPI = new(new TopOpeBRepDS_Transition(), TopOpeBRepDS_EDGE, 2, TopOpeBRepDS_POINT, 1, 0.5);
        HDS.StoreInterference(CPI, E1);
        Assert.Equal(1, DS.ShapeInterferences(E1).Extent());
        Assert.True(HDS.EdgePoints(E1).More());

        Assert.Throws<global::Kerf.Engine.Foundation.ProgramErrorException>(() => HDS.StoreInterference(CPI, theBox));
        Assert.Throws<global::Kerf.Engine.Foundation.ProgramErrorException>(() => HDS.StoreInterference(CPI, 3));

        Assert.Equal(1, HDS.FaceCurves(F1).Current());
        Assert.Equal(iC, HDS.FaceCurves(iF).Current());

        NCollection_List<TopOpeBRepDS_Interference> L = new();
        L.Append(CPI);
        HDS.ClearStoreInterferences(L, F1);
        Assert.Same(CPI, DS.ShapeInterferences(F1).First());
        Assert.Equal(1, DS.ShapeInterferences(F1).Extent());

        Assert.Equal((1, 1, 0), (HDS.NbGeometry(TopOpeBRepDS_CURVE), HDS.NbGeometry(TopOpeBRepDS_SURFACE), HDS.NbGeometry(TopOpeBRepDS_POINT)));
        Assert.Equal(0, HDS.NbGeometry(TopOpeBRepDS_FACE));
        Assert.Equal((1, 1, 0), (HDS.NbTopology(TopOpeBRepDS_FACE), HDS.NbTopology(TopOpeBRepDS_EDGE), HDS.NbTopology(TopOpeBRepDS_SOLID)));
        Assert.True(HDS.EdgesSameParameter());
    }

    [Fact]
    public void HDataStructure_AncestorsOfShapesInIt()
    {
        TopOpeBRepDS_HDataStructure HDS = new();
        TopOpeBRepDS_DataStructure DS = HDS.ChangeDS();
        DS.AddShape(theBox, 1);
        DS.AddShape(Sub(theBox, ShapeType.Edge, 1));
        HDS.AddAncestors(theBox);
        // the edge's two wires, their two faces, the shell and the solid
        Assert.Equal(2, HDS.NbTopology(TopOpeBRepDS_WIRE));
        Assert.Equal(2, HDS.NbTopology(TopOpeBRepDS_FACE));
        Assert.Equal(1, HDS.NbTopology(TopOpeBRepDS_SHELL));
        Assert.Equal(1, DS.AncestorRank(Sub(theBox, ShapeType.Shell, 1)));
    }

    [Fact]
    public void HDataStructure_SortsOnParameterAndFindsBounds()
    {
        TopOpeBRepDS_HDataStructure HDS = new();
        NCollection_List<TopOpeBRepDS_Interference> L = new();
        TopOpeBRepDS_Transition F = new(Forward), R = new(Reversed);
        L.Append(new TopOpeBRepDS_CurvePointInterference(F, TopOpeBRepDS_CURVE, 1, TopOpeBRepDS_POINT, 1, 3.0));
        L.Append(new TopOpeBRepDS_CurvePointInterference(R, TopOpeBRepDS_CURVE, 1, TopOpeBRepDS_POINT, 2, 1.0));
        L.Append(new TopOpeBRepDS_CurvePointInterference(F, TopOpeBRepDS_CURVE, 1, TopOpeBRepDS_POINT, 3, 2.0));
        L.Append(new TopOpeBRepDS_CurvePointInterference(R, TopOpeBRepDS_CURVE, 1, TopOpeBRepDS_POINT, 4, 4.0));

        double aMin = 0.0, aMax = 0.0;
        HDS.MinMaxOnParameter(L, ref aMin, ref aMax);
        Assert.Equal((1.0, 4.0), (aMin, aMax));

        // sorted by parameter (1, 2, 3, 4), then from the first FORWARD one: 2, 3, 4, 1
        HDS.SortOnParameter(L);
        List<int> aOrder = [];
        for (NCollection_List<TopOpeBRepDS_Interference>.Iterator it = new(L); it.More(); it.Next())
        {
            aOrder.Add(it.Value().Geometry());
        }
        Assert.Equal([3, 1, 4, 2], aOrder);
    }

    [Fact]
    public void HDataStructure_MakeCurveAndScan()
    {
        TopOpeBRepDS_HDataStructure HDS = new();
        TopOpeBRepDS_DataStructure DS = HDS.ChangeDS();
        Shape F1 = Sub(theBox, ShapeType.Face, 1), F2 = Sub(theBox, ShapeType.Face, 3);
        DS.AddShape(F1);
        DS.AddShape(F2);
        TopOpeBRepDS_Curve C = new(new LineCurve3D(new Axis3D()), 1.0e-7);
        TopOpeBRepDS_SurfaceCurveInterference SCI1 = new(new TopOpeBRepDS_Transition(Forward), TopOpeBRepDS_FACE, 1,
                                                         TopOpeBRepDS_CURVE, 1, null);
        C.SetShapes(F1, F2);
        C.SetSCI(SCI1, null);
        int iC = DS.AddCurve(ref C);

        TopOpeBRepDS_Curve N = new(new LineCurve3D(new Axis3D()), 1.0e-6);
        int iN = HDS.MakeCurve(DS.Curve(iC), ref N);
        Assert.Equal(2, iN);
        Assert.Equal(iC, DS.Curve(iN).Mother());
        Assert.True(DS.Curve(iN).Shape1().IsSame(F1));
        TopOpeBRepDS_Interference? aNewSCI = DS.Curve(iN).GetSCI1();
        Assert.NotNull(aNewSCI);
        Assert.NotSame(SCI1, aNewSCI);
        Assert.Equal(iN, aNewSCI.Geometry());
        Assert.Null(DS.Curve(iN).GetSCI2());
        Assert.Same(aNewSCI, DS.ShapeInterferences(F1).First());

        // a point interference matched by its 3D point
        int iP = DS.AddPoint(new TopOpeBRepDS_Point(new Point3D(1, 1, 1), 1.0e-3));
        NCollection_List<TopOpeBRepDS_Interference> L = new();
        L.Append(SCI1);
        L.Append(new TopOpeBRepDS_CurvePointInterference(new TopOpeBRepDS_Transition(), TopOpeBRepDS_CURVE, 1, TopOpeBRepDS_POINT, iP, 0.0));
        NCollection_List<TopOpeBRepDS_Interference>.Iterator it = new(L);
        int G = 0;
        TopOpeBRepDS_Kind K = TopOpeBRepDS_UNKNOWN;
        Assert.True(HDS.GetGeometry(it, new TopOpeBRepDS_Point(new Point3D(1, 1, 1.0005), 1.0e-4), ref G, ref K));
        Assert.Equal((iP, TopOpeBRepDS_POINT), (G, K));
        Assert.False(HDS.ScanInterfList(new NCollection_List<TopOpeBRepDS_Interference>.Iterator(L),
                                        new TopOpeBRepDS_Point(new Point3D(2, 1, 1), 1.0e-4)));

        HDS.RemoveCurve(iN);
        Assert.False(DS.KeepCurve(iN));
        Assert.True(DS.ShapeInterferences(F1).IsEmpty());
    }

    // ---- The text dump ----

    [Fact]
    public void TextDump_RoundTripsAHandBuiltDataStructure()
    {
        TopOpeBRepDS_HDataStructure HDS = new();
        TopOpeBRepDS_DataStructure DS = HDS.ChangeDS();
        Shape S = Sub(theBox, ShapeType.Solid, 1);
        Shape F1 = Sub(theBox, ShapeType.Face, 1), F2 = Sub(theBox, ShapeType.Face, 2);
        Shape E1 = Sub(theBox, ShapeType.Edge, 1), V1 = Sub(theBox, ShapeType.Vertex, 1);
        int iS = DS.AddShape(S, 1);
        int iF1 = DS.AddShape(F1.Reversed(), 1);
        DS.FillShapesSameDomain(F1, F2);
        int iE = DS.AddShape(E1);
        int iV = DS.AddShape(V1);
        DS.ChangeKeepShape(iV, false);

        int iP = DS.AddPoint(new TopOpeBRepDS_Point(new Point3D(0.1, -0.0, 1.0e-20), 1.0e-7));
        DS.AddPoint(new TopOpeBRepDS_Point(new Point3D(1, 2, 3), 0.5));
        DS.RemovePoint(2);

        TopOpeBRepDS_Transition T = new TopOpeBRepDS_Transition(Out, In, ShapeType.Edge, ShapeType.Face).WithIndex(2);
        TopOpeBRepDS_SurfaceCurveInterference SCI1 = new(T, TopOpeBRepDS_FACE, iF1, TopOpeBRepDS_CURVE, 1,
                                                         new LineCurve2D(new Point2D(0.5, 1.0 / 3.0), new Direction2D(0, 1)));
        TopOpeBRepDS_SurfaceCurveInterference SCI2 = new(T.Complement(), TopOpeBRepDS_FACE, 2, TopOpeBRepDS_CURVE, 1, null);
        TopOpeBRepDS_Curve C = new(new CircleCurve3D(new Frame3D(new Point3D(1, 2, 3), new Direction3D(0, 0, 1)), 2.5), 1.0e-5, true);
        C.SetShapes(F1, F2);
        C.SetSCI(SCI1, SCI2);
        C.SetRange(-1.0, Math.PI);
        DS.AddCurve(ref C);
        DS.AddCurve(new TopOpeBRepDS_Curve()); // a degenerated curve
        DS.AddCurve(new TopOpeBRepDS_Curve(new LineCurve3D(new Axis3D()), 0.0));
        DS.myCurves.UnBind(3);
        HDS.StoreInterference(SCI1, F1);
        DS.ChangeCurveInterferences(1).Append(new TopOpeBRepDS_CurvePointInterference(T, TopOpeBRepDS_CURVE, 1, TopOpeBRepDS_POINT, iP, 0.75));
        DS.ChangeCurveInterferences(1).Append(new TopOpeBRepDS_CurvePointInterference(T, TopOpeBRepDS_CURVE, 1, TopOpeBRepDS_VERTEX, iV, -0.0));
        DS.AddShapeInterference(E1, new TopOpeBRepDS_EdgeVertexInterference(T, iE, iV, true, TopOpeBRepDS_DIFFORIENTED, 1.0e300));
        DS.AddShapeInterference(F2, new TopOpeBRepDS_FaceEdgeInterference(T, 2, iE, false, TopOpeBRepDS_SAMEORIENTED));
        DS.AddShapeInterference(F2, new TopOpeBRepDS_ShapeShapeInterference(T, TopOpeBRepDS_FACE, 2, TopOpeBRepDS_EDGE, iE, true, TopOpeBRepDS_UNSHGEOMETRY));
        DS.ChangePointInterferences(iP).Append(new TopOpeBRepDS_Interference(T, TopOpeBRepDS_EDGE, iE, TopOpeBRepDS_POINT, iP));

        PlaneSurface P = new(new CoordinateSystem(new Point3D(0, 0, 1), new Direction3D(0, 0, 1)));
        int iSu = DS.AddSurface(new TopOpeBRepDS_Surface(P, 1.0e-3));
        DS.ChangeKeepSurface(iSu, true);
        TopOpeBRepDS_SolidSurfaceInterference SSI = new(new TopOpeBRepDS_Transition(Reversed), TopOpeBRepDS_SOLID, iS,
                                                        TopOpeBRepDS_SURFACE, iSu);
        HDS.StoreInterference(SSI, S);
        DS.AddSurface(new TopOpeBRepDS_Surface(P, 0.0));
        DS.RemoveSurface(2);
        DS.SetNewSurface(F2, P);
        DS.AddSectionEdge(global::Kerf.Engine.Topology.ShapeCasts.Edge(E1));
        DS.Isfafa(true);

        string aText = TopOpeBRepDS_TextDump.Write(DS, theBox);
        Assert.StartsWith("dsdump 1\nshapes 5\nshape 1 SOLID FORWARD in 1 keep 1 sdref 1 sdori SAME sdind 0 rank 1 sd 0\n"
                          + "interferences 1\nI #1 SSI IN/OUT FACE/FACE 0/0 SOLID 1 SURFACE 1\n"
                          + "shape 2 FACE FORWARD in 1 keep 1 sdref 2 sdori SAME sdind 1 rank 1 sd 1 3\n", aText);
        Assert.Contains("curve 1 1.0000000000000001e-05 range 1 -1 3.1415926535897931 keep 1 mother 0 index 1 walk 1 sci #", aText);
        Assert.Contains("curve 3 unbound\n", aText);
        Assert.Contains("surface 2 unbound\n", aText);
        Assert.Contains(" 1.0000000000000001e+300\n", aText);
        Assert.Contains("R #", aText); // SCI1 is on the face and on the curve
        Assert.EndsWith("sectionedges 1\nsectionedge EDGE FORWARD in 1\nisfafa 1\nend\n", aText);

        TopOpeBRepDS_HDataStructure aRead = TopOpeBRepDS_TextDump.Read(aText, theBox);
        Assert.Equal(aText, TopOpeBRepDS_TextDump.Write(aRead.DS(), theBox));

        // the shared interference is shared again, and the curve's index survives
        TopOpeBRepDS_DataStructure RD = aRead.DS();
        Assert.Same(RD.ShapeInterferences(F1).First(), RD.Curve(1).GetSCI1());
        Assert.Same(RD.CurveInterferences(1).First(), RD.Curve(1).GetSCI1());
        Assert.Equal(1, RD.Curve(1).DSIndex());
        Assert.False(RD.KeepShape(5));
        Assert.True(RD.HasNewSurface(F2));
    }

    [Fact]
    public void TextDump_AShapeOutsideTheInputIsWrittenButNotRead()
    {
        TopOpeBRepDS_DataStructure DS = new();
        Shape V = new BRepBuilderAPI_MakeVertex(new Point3D(1, 2, 3)).Shape();
        DS.AddShape(V);
        string aText = TopOpeBRepDS_TextDump.Write(DS, theBox);
        Assert.Contains("shape 1 VERTEX FORWARD new 1 2 3 keep 1", aText);
        Assert.Throws<FormatException>(() => TopOpeBRepDS_TextDump.Read(aText, theBox));
    }
}
