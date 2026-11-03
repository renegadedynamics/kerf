// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy.Data;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;
using static Kerf.Engine.Booleans.Legacy.Data.DsConfig;
using static Kerf.Engine.Booleans.Legacy.Data.DsKind;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The legacy data structure's core (unit 7L) against OCCT's semantics, on data structures filled by hand: transitions,
/// interferences, the data structure's shapes, points, curves and surfaces, its iterators and explorers, the handle's
/// storing and sorting functions, and the text dump's round trip. Native data structures are in
/// <see cref="TopOpeBRepDSNativeTests"/>.
/// </summary>
public class TopOpeBRepDSTests
{
    private static readonly Shape theBox = new MakeBox(10.0, 20.0, 30.0).Shape();

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
        DsTransition T = new();
        Assert.Equal(global::Kerf.Topology.Containment.Unknown, T.Before());
        Assert.Equal(global::Kerf.Topology.Containment.Unknown, T.After());
        Assert.Equal(ShapeType.Face, T.ShapeBefore());
        Assert.Equal(ShapeType.Face, T.ONAfter());
        Assert.True(T.IsUnknown);
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
        DsTransition T = new(O);
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
        Assert.Equal(Internal, new DsTransition(On, On).Orientation(In));
        Assert.Equal(External, new DsTransition(On, On).Orientation(Out));
        Assert.Equal(Internal, new DsTransition(On, On).Orientation(On));
        // OCCT: (ON, ON) for UNKNOWN keeps the default FORWARD
        Assert.Equal(Forward, new DsTransition(On, On).Orientation(global::Kerf.Topology.Containment.Unknown));
        Assert.Equal(Forward, new DsTransition(On, In).Orientation(In));
        Assert.Equal(Reversed, new DsTransition(On, Out).Orientation(In));
        Assert.Equal(Reversed, new DsTransition(In, On).Orientation(In));
        Assert.Equal(Forward, new DsTransition(Out, On).Orientation(In));

        // the complement of an ON transition exchanges before and after, states and shapes
        DsTransition T = new(On, In, ShapeType.Edge, ShapeType.Face);
        DsTransition C = T.Complement();
        Assert.Equal(In, C.Before());
        Assert.Equal(On, C.After());
        Assert.Equal(ShapeType.Face, C.ShapeBefore());
        Assert.Equal(ShapeType.Edge, C.ShapeAfter());
    }

    [Fact]
    public void Transition_IndexNeedsEqualIndices()
    {
        DsTransition T = new DsTransition(Out, In).WithIndex(3);
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
        DsTransition T = new(Forward);
        DsCurvePointInterference CPI = new(T, DsKind.Curve, 2, DsKind.Point, 5, 1.5);
        CPI.GKGSKS(out DsKind GK, out int G, out DsKind SK, out int S);
        Assert.Equal((DsKind.Point, 5, DsKind.Curve, 2), (GK, G, SK, S));
        Assert.Equal(1.5, CPI.Parameter());
        Assert.Equal(T, CPI.Transition());
        CPI.ChangeTransition().Index(7);
        Assert.Equal(7, CPI.Transition().Index());

        DsEdgeVertexInterference EVI = new(T, 3, 4, true, DsConfig.Difforiented, 0.25);
        Assert.Equal(DsKind.Edge, EVI.SupportType());
        Assert.Equal(DsKind.Vertex, EVI.GeometryType());
        Assert.True(EVI.GBound());
        Assert.Equal(DsConfig.Difforiented, EVI.Config());

        DsFaceEdgeInterference FEI = new(T, 1, 2, false, DsConfig.Sameoriented);
        Assert.Equal(DsKind.Face, FEI.SupportType());
        Assert.Equal(DsKind.Edge, FEI.GeometryType());

        LineCurve2D PC = new(new Point2D(0, 0), new Direction2D(1, 0));
        DsSurfaceCurveInterference SCI = new(T, DsKind.Face, 1, DsKind.Curve, 3, PC);
        Assert.Same(PC, SCI.PCurve());
        // the copy keeps the transition, support and geometry, not the pcurve
        DsSurfaceCurveInterference SCI2 = new(SCI);
        Assert.Null(SCI2.PCurve());
        Assert.True(SCI2.HasSameGeometry(SCI));
        Assert.True(SCI2.HasSameSupport(SCI));
        Assert.False(SCI2.HasSameGeometry(CPI));
    }

    // ---- The data structure: shapes ----

    [Fact]
    public void DataStructure_ShapesAreIndexedOnceWhateverTheirOrientation()
    {
        DsDataStructure DS = new();
        Shape F1 = Sub(theBox, ShapeType.Face, 1);
        Assert.Equal(1, DS.AddShape(F1));
        Assert.Equal(1, DS.AddShape(F1.Reversed(), 2));
        Assert.Equal(0, DS.AncestorRank(1)); // a shape already there keeps its rank
        Assert.Equal(2, DS.AddShape(theBox, 1));
        Assert.Equal(1, DS.AncestorRank(theBox));
        Assert.Equal(2, DS.NbShapes());

        // a new shape is its own same-domain reference, same oriented
        Assert.Equal(1, DS.SameDomainRef(1));
        Assert.Equal(DsConfig.Sameoriented, DS.SameDomainOri(F1));
        Assert.Equal(0, DS.SameDomainInd(1));
        Assert.Equal(DsConfig.Unshgeometry, DS.SameDomainOri(5));

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
        DsDataStructure DS = new();
        Shape F1 = Sub(theBox, ShapeType.Face, 1);
        int iF = DS.AddShape(F1);
        DsInterference I1 = new(new DsTransition(), DsKind.Face, iF, DsKind.Curve, 1);
        DsInterference I2 = new(new DsTransition(), DsKind.Face, iF, DsKind.Curve, 2);
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
        DsDataStructure DS = new();
        Shape F1 = Sub(theBox, ShapeType.Face, 1), F2 = Sub(theBox, ShapeType.Face, 2);
        DsInterference I = new(new DsTransition(), DsKind.Face, 1, DsKind.Curve, 1);
        DS.AddShapeInterference(F1, I);
        Assert.Equal(0, DS.NbShapes());
        Assert.Same(I, DS.ShapeInterferences(F2).First());
        Assert.Same(I, DS.CurveInterferences(9).First());

        // other data structures have their own
        Assert.True(new DsDataStructure().ShapeInterferences(F2).IsEmpty());
    }

    [Fact]
    public void DataStructure_SameDomainFaces()
    {
        // two boxes, one on the other: the top of the first and the bottom of the second share their plane
        Shape B1 = new MakeBox(10.0, 10.0, 10.0).Shape();
        Shape B2 = new MakeBox(new Point3D(0, 0, 10), 10.0, 10.0, 10.0).Shape();
        Shape Top = TopoDS_Shape_Of(B1, 10.0), Bottom = TopoDS_Shape_Of(B2, 10.0);

        DsHDataStructure HDS = new();
        DsDataStructure DS = HDS.ChangeDS();
        DS.FillShapesSameDomain(Top, Bottom);
        int i1 = DS.Shape(Top), i2 = DS.Shape(Bottom);
        Assert.Equal((1, 2), (i1, i2));
        Assert.Equal((1, 2), (DS.AncestorRank(Top), DS.AncestorRank(Bottom)));
        Assert.Equal((1, 2), (DS.SameDomainInd(Top), DS.SameDomainInd(Bottom)));
        Assert.Equal(i1, DS.SameDomainRef(Bottom));
        Assert.Equal(i1, HDS.SameDomainReference(Top));
        Assert.Equal(DsConfig.Sameoriented, DS.SameDomainOri(Top));
        // their outward normals are opposite
        Assert.Equal(DsConfig.Difforiented, HDS.SameDomainOrientation(Bottom));
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
        DsDataStructure DS = new();
        Assert.Throws<global::Kerf.Engine.Foundation.ProgramErrorException>(() => DS.Point(1));
        Assert.Equal(1, DS.AddPoint(new DsPoint(new Point3D(1, 2, 3), 0.1)));
        Assert.Equal(2, DS.AddPointSS(new DsPoint(new Point3D(1, 2, 3.05), 0.01), theBox, theBox));
        Assert.True(DS.Point(1).IsEqual(DS.Point(2))); // within the larger tolerance
        Assert.False(DS.Point(2).IsEqual(new DsPoint(new Point3D(1, 2, 3.2), 0.01)));
        Assert.Throws<global::Kerf.Engine.Foundation.ProgramErrorException>(() => DS.Point(3));

        DS.ChangePoint(1).Tolerance(0.5);
        Assert.Equal(0.5, DS.Point(1).Tolerance());
        DS.RemovePoint(1);
        Assert.False(DS.KeepPoint(1));

        DsPointExplorer pex = new(DS);
        Assert.True(pex.More());
        Assert.Equal(2, pex.Index());
        pex.Next();
        Assert.False(pex.More());
        Assert.Equal(1, pex.NbPoint());
        Assert.Equal(2, new DsPointExplorer(DS, false).NbPoint());

        // a vertex's point and tolerance
        Shape V = Sub(theBox, ShapeType.Vertex, 1);
        DsPoint PV = new(V);
        Assert.True(PV.Point().IsEqual(Tool.Pnt(global::Kerf.Engine.Topology.ShapeCasts.Vertex(V)), 0.0));
        Assert.Equal(Tool.Tolerance(global::Kerf.Engine.Topology.ShapeCasts.Vertex(V)), PV.Tolerance());
        Assert.Throws<global::Kerf.Engine.Foundation.ProgramErrorException>(() => new DsPoint(theBox));
    }

    [Fact]
    public void DataStructure_Curves()
    {
        DsDataStructure DS = new();
        Shape F1 = Sub(theBox, ShapeType.Face, 1), F2 = Sub(theBox, ShapeType.Face, 3);
        DS.AddShape(F1);
        DS.AddShape(F2);

        LineCurve3D L = new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        DsCurve C = new(L, 1.0e-5);
        Assert.Equal(0, C.DSIndex());
        Assert.False(C.Range(out _, out _));
        Assert.True(C.Keep());
        Assert.Null(C.Curve1());

        DsSurfaceCurveInterference SCI1 = new(new DsTransition(Forward), DsKind.Face, 1,
                                                         DsKind.Curve, 1, new LineCurve2D(new Point2D(), new Direction2D()));
        DsSurfaceCurveInterference SCI2 = new(new DsTransition(Reversed), DsKind.Face, 2,
                                                         DsKind.Curve, 1, null);
        C.SetShapes(F1, F2);
        C.SetSCI(SCI1, SCI2);
        DS.AddShapeInterference(F1, SCI1);
        DS.AddShapeInterference(F2, SCI2);

        // OCCT sets the caller's index too
        Assert.Equal(1, DS.AddCurve(ref C));
        Assert.Equal(1, C.DSIndex());
        Assert.Equal(2, DS.AddCurve(new DsCurve(L, 1.0e-6)));
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

        DsCurveExplorer cex = new(DS);
        Assert.Equal(2, cex.Index());
        Assert.Equal(1.0e-6, cex.Curve().Tolerance());
        Assert.Equal(1, cex.NbCurve());
        Assert.False(cex.More());
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Confusion(), cex.Curve().Tolerance()); // the empty curve
        Assert.Equal(2, new DsCurveExplorer(DS, false).NbCurve());
        Assert.True(cex.IsCurve(1));
        Assert.False(cex.IsCurveKeep(1));
    }

    [Fact]
    public void DataStructure_SurfacesAreNotKeptByDefault()
    {
        DsDataStructure DS = new();
        PlaneSurface P = new(new CoordinateSystem());
        Assert.Equal(1, DS.AddSurface(new DsSurface(P, 1.0e-7)));
        Assert.Equal(2, DS.AddSurface(new DsSurface(P, 1.0e-6)));
        Assert.False(DS.KeepSurface(1));
        Assert.Equal(0, new DsSurfaceExplorer(DS).NbSurface());
        DS.ChangeKeepSurface(2, true);
        DsSurfaceExplorer sex = new(DS);
        Assert.True(sex.More());
        Assert.Equal(2, sex.Index());
        Assert.Same(P, sex.Surface().Surface());

        DS.RemoveSurface(1);
        Assert.Null(DS.Surface(1).Surface());
        Assert.Equal(2, DS.NbSurfaces());
        Assert.Equal(1, new DsSurfaceExplorer(DS, false).NbSurface());

        Shape F1 = Sub(theBox, ShapeType.Face, 1);
        Assert.False(DS.HasNewSurface(F1));
        DS.SetNewSurface(F1, P);
        Assert.True(DS.HasNewSurface(F1.Reversed()));
        Assert.Same(P, DS.NewSurface(F1));
    }

    [Fact]
    public void DataStructure_InitEmptiesIt()
    {
        DsDataStructure DS = new();
        DS.AddShape(theBox);
        DS.AddPoint(new DsPoint());
        DS.AddCurve(new DsCurve());
        DS.AddSurface(new DsSurface());
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
        DsDataStructure DS = new();
        NCollection_IndexedDataMap<Shape, DsShapeWithState> M = DS.ChangeMapOfShapeWithState(theBox, out bool aFound);
        Assert.False(aFound);
        Assert.True(M.IsEmpty());
        DsShapeWithState SWS = new();
        SWS.AddPart(theBox, In);
        SWS.SetState(On);
        DS.ChangeMapOfShapeWithStateObj().Add(theBox, SWS);
        Assert.Same(DS.ChangeMapOfShapeWithStateObj(), DS.ChangeMapOfShapeWithState(theBox, out aFound));
        Assert.True(aFound);
        Assert.Equal(On, DS.GetShapeWithState(theBox).State());
        Assert.Equal(1, DS.GetShapeWithState(theBox).Part(In).Extent());
        Assert.True(DS.GetShapeWithState(theBox).Part(global::Kerf.Topology.Containment.Unknown).IsEmpty());
        Assert.Equal(global::Kerf.Topology.Containment.Unknown, new DsDataStructure().GetShapeWithState(theBox).State());
    }

    // ---- Iterators ----

    private static NCollection_List<DsInterference> MixedList()
    {
        DsTransition F = new(Forward), R = new(Reversed);
        NCollection_List<DsInterference> L = new();
        L.Append(new DsSurfaceCurveInterference(F, DsKind.Face, 1, DsKind.Curve, 4, null));
        L.Append(new DsCurvePointInterference(R, DsKind.Curve, 1, DsKind.Point, 2, 3.0));
        L.Append(new DsSolidSurfaceInterference(F, DsKind.Solid, 1, DsKind.Surface, 7));
        L.Append(new DsEdgeVertexInterference(F, 1, 5, true, DsConfig.Sameoriented, 1.0));
        L.Append(new DsCurvePointInterference(F, DsKind.Curve, 1, DsKind.Point, 3, 2.0));
        return L;
    }

    [Fact]
    public void Iterators_MatchTheirGeometries()
    {
        NCollection_List<DsInterference> L = MixedList();

        List<int> aPoints = [];
        List<double> aParams = [];
        for (DsPointIterator it = new(L); it.More(); it.Next())
        {
            aPoints.Add(it.Current());
            aParams.Add(it.Parameter());
        }
        Assert.Equal([2, 5, 3], aPoints);
        Assert.Equal([3.0, 1.0, 2.0], aParams);

        DsPointIterator pit = new(L);
        Assert.True(pit.IsPoint);
        Assert.Equal(Reversed, pit.Orientation(In));
        Assert.Throws<global::Kerf.Engine.Foundation.ProgramErrorException>(() => pit.SameOriented());
        pit.Next();
        Assert.True(pit.IsVertex);
        Assert.True(pit.SameOriented());
        Assert.False(pit.DiffOriented());
        Assert.Equal(1, pit.Support());

        DsCurveIterator cit = new(L);
        Assert.Equal(4, cit.Current());
        Assert.Null(cit.PCurve());
        cit.Next();
        Assert.False(cit.More());

        DsInterferenceIterator iit = new(L);
        iit.SupportKind(DsKind.Curve);
        iit.Geometry(3);
        iit.Match();
        Assert.Equal(DsKind.Point, iit.Value().GeometryType());
        Assert.Equal(3, iit.Value().Geometry());
    }

    [Fact]
    public void Iterators_TheSurfaceIteratorsFirstValueIsUnmatched()
    {
        // OCCT sets the SURFACE condition without matching again: the first value is the list's first (occt-issues 7L)
        NCollection_List<DsInterference> L = MixedList();
        DsSurfaceIterator sit = new(L);
        Assert.Equal(DsKind.Curve, sit.Value().GeometryType());
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
        DsHDataStructure HDS = new();
        DsDataStructure DS = HDS.ChangeDS();
        Shape F1 = Sub(theBox, ShapeType.Face, 1), E1 = Sub(theBox, ShapeType.Edge, 1);
        int iF = DS.AddShape(F1);
        DS.AddShape(E1);
        int iC = DS.AddCurve(new DsCurve(new LineCurve3D(new Axis3D()), 0.0));
        int iS = DS.AddSurface(new DsSurface(new PlaneSurface(new CoordinateSystem()), 0.0));

        DsSurfaceCurveInterference SCI = new(new DsTransition(Forward), DsKind.Face, iF,
                                                        DsKind.Curve, iC, null);
        HDS.StoreInterference(SCI, F1);
        Assert.Same(SCI, DS.ShapeInterferences(F1).First());
        Assert.Same(SCI, DS.CurveInterferences(iC).First());

        DsSolidSurfaceInterference SSI = new(new DsTransition(Forward), DsKind.Face, iF,
                                                        DsKind.Surface, iS);
        HDS.StoreInterference(SSI, iF);
        Assert.Same(SSI, DS.SurfaceInterferences(iS).First());

        // a point interference goes on the shape only
        DsCurvePointInterference CPI = new(new DsTransition(), DsKind.Edge, 2, DsKind.Point, 1, 0.5);
        HDS.StoreInterference(CPI, E1);
        Assert.Equal(1, DS.ShapeInterferences(E1).Extent());
        Assert.True(HDS.EdgePoints(E1).More());

        Assert.Throws<global::Kerf.Engine.Foundation.ProgramErrorException>(() => HDS.StoreInterference(CPI, theBox));
        Assert.Throws<global::Kerf.Engine.Foundation.ProgramErrorException>(() => HDS.StoreInterference(CPI, 3));

        Assert.Equal(1, HDS.FaceCurves(F1).Current());
        Assert.Equal(iC, HDS.FaceCurves(iF).Current());

        NCollection_List<DsInterference> L = new();
        L.Append(CPI);
        HDS.ClearStoreInterferences(L, F1);
        Assert.Same(CPI, DS.ShapeInterferences(F1).First());
        Assert.Equal(1, DS.ShapeInterferences(F1).Extent());

        Assert.Equal((1, 1, 0), (HDS.NbGeometry(DsKind.Curve), HDS.NbGeometry(DsKind.Surface), HDS.NbGeometry(DsKind.Point)));
        Assert.Equal(0, HDS.NbGeometry(DsKind.Face));
        Assert.Equal((1, 1, 0), (HDS.NbTopology(DsKind.Face), HDS.NbTopology(DsKind.Edge), HDS.NbTopology(DsKind.Solid)));
        Assert.True(HDS.EdgesSameParameter());
    }

    [Fact]
    public void HDataStructure_AncestorsOfShapesInIt()
    {
        DsHDataStructure HDS = new();
        DsDataStructure DS = HDS.ChangeDS();
        DS.AddShape(theBox, 1);
        DS.AddShape(Sub(theBox, ShapeType.Edge, 1));
        HDS.AddAncestors(theBox);
        // the edge's two wires, their two faces, the shell and the solid
        Assert.Equal(2, HDS.NbTopology(DsKind.Wire));
        Assert.Equal(2, HDS.NbTopology(DsKind.Face));
        Assert.Equal(1, HDS.NbTopology(DsKind.Shell));
        Assert.Equal(1, DS.AncestorRank(Sub(theBox, ShapeType.Shell, 1)));
    }

    [Fact]
    public void HDataStructure_SortsOnParameterAndFindsBounds()
    {
        DsHDataStructure HDS = new();
        NCollection_List<DsInterference> L = new();
        DsTransition F = new(Forward), R = new(Reversed);
        L.Append(new DsCurvePointInterference(F, DsKind.Curve, 1, DsKind.Point, 1, 3.0));
        L.Append(new DsCurvePointInterference(R, DsKind.Curve, 1, DsKind.Point, 2, 1.0));
        L.Append(new DsCurvePointInterference(F, DsKind.Curve, 1, DsKind.Point, 3, 2.0));
        L.Append(new DsCurvePointInterference(R, DsKind.Curve, 1, DsKind.Point, 4, 4.0));

        double aMin = 0.0, aMax = 0.0;
        HDS.MinMaxOnParameter(L, ref aMin, ref aMax);
        Assert.Equal((1.0, 4.0), (aMin, aMax));

        // sorted by parameter (1, 2, 3, 4), then from the first FORWARD one: 2, 3, 4, 1
        HDS.SortOnParameter(L);
        List<int> aOrder = [];
        for (NCollection_List<DsInterference>.Iterator it = new(L); it.More(); it.Next())
        {
            aOrder.Add(it.Value().Geometry());
        }
        Assert.Equal([3, 1, 4, 2], aOrder);
    }

    [Fact]
    public void HDataStructure_MakeCurveAndScan()
    {
        DsHDataStructure HDS = new();
        DsDataStructure DS = HDS.ChangeDS();
        Shape F1 = Sub(theBox, ShapeType.Face, 1), F2 = Sub(theBox, ShapeType.Face, 3);
        DS.AddShape(F1);
        DS.AddShape(F2);
        DsCurve C = new(new LineCurve3D(new Axis3D()), 1.0e-7);
        DsSurfaceCurveInterference SCI1 = new(new DsTransition(Forward), DsKind.Face, 1,
                                                         DsKind.Curve, 1, null);
        C.SetShapes(F1, F2);
        C.SetSCI(SCI1, null);
        int iC = DS.AddCurve(ref C);

        DsCurve N = new(new LineCurve3D(new Axis3D()), 1.0e-6);
        int iN = HDS.MakeCurve(DS.Curve(iC), ref N);
        Assert.Equal(2, iN);
        Assert.Equal(iC, DS.Curve(iN).Mother());
        Assert.True(DS.Curve(iN).Shape1().IsSame(F1));
        DsInterference? aNewSCI = DS.Curve(iN).GetSCI1();
        Assert.NotNull(aNewSCI);
        Assert.NotSame(SCI1, aNewSCI);
        Assert.Equal(iN, aNewSCI.Geometry());
        Assert.Null(DS.Curve(iN).GetSCI2());
        Assert.Same(aNewSCI, DS.ShapeInterferences(F1).First());

        // a point interference matched by its 3D point
        int iP = DS.AddPoint(new DsPoint(new Point3D(1, 1, 1), 1.0e-3));
        NCollection_List<DsInterference> L = new();
        L.Append(SCI1);
        L.Append(new DsCurvePointInterference(new DsTransition(), DsKind.Curve, 1, DsKind.Point, iP, 0.0));
        NCollection_List<DsInterference>.Iterator it = new(L);
        int G = 0;
        DsKind K = DsKind.Unknown;
        Assert.True(HDS.GetGeometry(it, new DsPoint(new Point3D(1, 1, 1.0005), 1.0e-4), ref G, ref K));
        Assert.Equal((iP, DsKind.Point), (G, K));
        Assert.False(HDS.ScanInterfList(new NCollection_List<DsInterference>.Iterator(L),
                                        new DsPoint(new Point3D(2, 1, 1), 1.0e-4)));

        HDS.RemoveCurve(iN);
        Assert.False(DS.KeepCurve(iN));
        Assert.True(DS.ShapeInterferences(F1).IsEmpty());
    }

    // ---- The text dump ----

    [Fact]
    public void TextDump_RoundTripsAHandBuiltDataStructure()
    {
        DsHDataStructure HDS = new();
        DsDataStructure DS = HDS.ChangeDS();
        Shape S = Sub(theBox, ShapeType.Solid, 1);
        Shape F1 = Sub(theBox, ShapeType.Face, 1), F2 = Sub(theBox, ShapeType.Face, 2);
        Shape E1 = Sub(theBox, ShapeType.Edge, 1), V1 = Sub(theBox, ShapeType.Vertex, 1);
        int iS = DS.AddShape(S, 1);
        int iF1 = DS.AddShape(F1.Reversed(), 1);
        DS.FillShapesSameDomain(F1, F2);
        int iE = DS.AddShape(E1);
        int iV = DS.AddShape(V1);
        DS.ChangeKeepShape(iV, false);

        int iP = DS.AddPoint(new DsPoint(new Point3D(0.1, -0.0, 1.0e-20), 1.0e-7));
        DS.AddPoint(new DsPoint(new Point3D(1, 2, 3), 0.5));
        DS.RemovePoint(2);

        DsTransition T = new DsTransition(Out, In, ShapeType.Edge, ShapeType.Face).WithIndex(2);
        DsSurfaceCurveInterference SCI1 = new(T, DsKind.Face, iF1, DsKind.Curve, 1,
                                                         new LineCurve2D(new Point2D(0.5, 1.0 / 3.0), new Direction2D(0, 1)));
        DsSurfaceCurveInterference SCI2 = new(T.Complement(), DsKind.Face, 2, DsKind.Curve, 1, null);
        DsCurve C = new(new CircleCurve3D(new Frame3D(new Point3D(1, 2, 3), new Direction3D(0, 0, 1)), 2.5), 1.0e-5, true);
        C.SetShapes(F1, F2);
        C.SetSCI(SCI1, SCI2);
        C.SetRange(-1.0, Math.PI);
        DS.AddCurve(ref C);
        DS.AddCurve(new DsCurve()); // a degenerated curve
        DS.AddCurve(new DsCurve(new LineCurve3D(new Axis3D()), 0.0));
        DS._curves.UnBind(3);
        HDS.StoreInterference(SCI1, F1);
        DS.ChangeCurveInterferences(1).Append(new DsCurvePointInterference(T, DsKind.Curve, 1, DsKind.Point, iP, 0.75));
        DS.ChangeCurveInterferences(1).Append(new DsCurvePointInterference(T, DsKind.Curve, 1, DsKind.Vertex, iV, -0.0));
        DS.AddShapeInterference(E1, new DsEdgeVertexInterference(T, iE, iV, true, DsConfig.Difforiented, 1.0e300));
        DS.AddShapeInterference(F2, new DsFaceEdgeInterference(T, 2, iE, false, DsConfig.Sameoriented));
        DS.AddShapeInterference(F2, new DsShapeShapeInterference(T, DsKind.Face, 2, DsKind.Edge, iE, true, DsConfig.Unshgeometry));
        DS.ChangePointInterferences(iP).Append(new DsInterference(T, DsKind.Edge, iE, DsKind.Point, iP));

        PlaneSurface P = new(new CoordinateSystem(new Point3D(0, 0, 1), new Direction3D(0, 0, 1)));
        int iSu = DS.AddSurface(new DsSurface(P, 1.0e-3));
        DS.ChangeKeepSurface(iSu, true);
        DsSolidSurfaceInterference SSI = new(new DsTransition(Reversed), DsKind.Solid, iS,
                                                        DsKind.Surface, iSu);
        HDS.StoreInterference(SSI, S);
        DS.AddSurface(new DsSurface(P, 0.0));
        DS.RemoveSurface(2);
        DS.SetNewSurface(F2, P);
        DS.AddSectionEdge(global::Kerf.Engine.Topology.ShapeCasts.Edge(E1));
        DS.Isfafa(true);

        string aText = TextDump.Write(DS, theBox);
        Assert.StartsWith("dsdump 1\nshapes 5\nshape 1 SOLID FORWARD in 1 keep 1 sdref 1 sdori SAME sdind 0 rank 1 sd 0\n"
                          + "interferences 1\nI #1 SSI IN/OUT FACE/FACE 0/0 SOLID 1 SURFACE 1\n"
                          + "shape 2 FACE FORWARD in 1 keep 1 sdref 2 sdori SAME sdind 1 rank 1 sd 1 3\n", aText);
        Assert.Contains("curve 1 1.0000000000000001e-05 range 1 -1 3.1415926535897931 keep 1 mother 0 index 1 walk 1 sci #", aText);
        Assert.Contains("curve 3 unbound\n", aText);
        Assert.Contains("surface 2 unbound\n", aText);
        Assert.Contains(" 1.0000000000000001e+300\n", aText);
        Assert.Contains("R #", aText); // SCI1 is on the face and on the curve
        Assert.EndsWith("sectionedges 1\nsectionedge EDGE FORWARD in 1\nisfafa 1\nend\n", aText);

        DsHDataStructure aRead = TextDump.Read(aText, theBox);
        Assert.Equal(aText, TextDump.Write(aRead.DS(), theBox));

        // the shared interference is shared again, and the curve's index survives
        DsDataStructure RD = aRead.DS();
        Assert.Same(RD.ShapeInterferences(F1).First(), RD.Curve(1).GetSCI1());
        Assert.Same(RD.CurveInterferences(1).First(), RD.Curve(1).GetSCI1());
        Assert.Equal(1, RD.Curve(1).DSIndex());
        Assert.False(RD.KeepShape(5));
        Assert.True(RD.HasNewSurface(F2));
    }

    [Fact]
    public void TextDump_AShapeOutsideTheInputIsWrittenButNotRead()
    {
        DsDataStructure DS = new();
        Shape V = new MakeVertex(new Point3D(1, 2, 3)).Shape();
        DS.AddShape(V);
        string aText = TextDump.Write(DS, theBox);
        Assert.Contains("shape 1 VERTEX FORWARD new 1 2 3 keep 1", aText);
        Assert.Throws<FormatException>(() => TextDump.Read(aText, theBox));
    }
}
