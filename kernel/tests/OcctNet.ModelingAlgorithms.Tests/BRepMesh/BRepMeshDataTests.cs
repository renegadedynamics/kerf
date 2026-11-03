// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using OcctNet.ModelingData.Tests;
using static Occt.IMeshData_Status;
using static Occt.IMeshTools_MeshAlgoType;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// IMeshTools_Parameters as a value, and BRepMeshData's default model built by hand, as BRepMesh_ShapeVisitor (unit
/// 4C) will build it.
/// </summary>
public class BRepMeshDataTests
{
    [Fact]
    public void Parameters_have_OCCT_defaults_and_compare_as_values()
    {
        var aP = new IMeshTools_Parameters();
        Assert.Equal(IMeshTools_MeshAlgoType_DEFAULT, aP.MeshAlgo);
        Assert.Equal(0.5, aP.Angle);
        Assert.Equal(0.001, aP.Deflection);
        Assert.Equal(-1.0, aP.AngleInterior);
        Assert.Equal(-1.0, aP.DeflectionInterior);
        Assert.Equal(-1.0, aP.MinSize);
        Assert.False(aP.InParallel);
        Assert.False(aP.Relative);
        Assert.True(aP.InternalVerticesMode);
        Assert.True(aP.ControlSurfaceDeflection);
        Assert.False(aP.EnableControlSurfaceDeflectionAllSurfaces);
        Assert.True(aP.CleanModel);
        Assert.False(aP.AdjustMinSize);
        Assert.False(aP.ForceFaceDeflection);
        Assert.False(aP.AllowQualityDecrease);
        Assert.Equal(0.1, IMeshTools_Parameters.RelMinSize());

        // default(T) isn't OCCT's default
        Assert.NotEqual(aP, default(IMeshTools_Parameters));

        var aQ = new IMeshTools_Parameters { Deflection = 0.1, Angle = 0.5, InParallel = true };
        var aR = new IMeshTools_Parameters { Deflection = 0.1, InParallel = true };
        Assert.Equal(aQ, aR);
        Assert.True(aQ == aR);
        Assert.Equal(aQ.GetHashCode(), aR.GetHashCode());
        Assert.Equal(aQ.ToString(), aR.ToString());

        // A value: changing a copy changes nothing shared.
        IMeshTools_Parameters aCopy = aQ;
        aCopy.Relative = true;
        Assert.False(aQ.Relative);
        Assert.NotEqual(aQ, aCopy);
        Assert.True(aQ != aCopy);

        // Every field takes part in equality and the hash.
        var aVariants = new List<IMeshTools_Parameters>
        {
            aP with { MeshAlgo = IMeshTools_MeshAlgoType_Delabella },
            aP with { Angle = 0.6 }, aP with { Deflection = 0.002 }, aP with { AngleInterior = 1 },
            aP with { DeflectionInterior = 1 }, aP with { MinSize = 1 }, aP with { InParallel = true },
            aP with { Relative = true }, aP with { InternalVerticesMode = false },
            aP with { ControlSurfaceDeflection = false }, aP with { EnableControlSurfaceDeflectionAllSurfaces = true },
            aP with { CleanModel = false }, aP with { AdjustMinSize = true }, aP with { ForceFaceDeflection = true },
            aP with { AllowQualityDecrease = true },
        };
        var aSet = new HashSet<IMeshTools_Parameters>(aVariants) { aP };
        Assert.Equal(aVariants.Count + 1, aSet.Count);

        // NaN equals itself, as double.Equals: usable as a dictionary key
        Assert.Equal(aP with { Deflection = double.NaN }, aP with { Deflection = double.NaN });
    }

    [Fact]
    public void Curve_and_pcurve_points_and_parameters()
    {
        var aCurve = new BRepMeshData_Curve();
        Assert.Equal(0, aCurve.ParametersNb());
        aCurve.AddPoint(new Point3D(0, 0, 0), 0.0);
        aCurve.AddPoint(new Point3D(3, 0, 0), 3.0);
        aCurve.InsertPoint(1, new Point3D(1, 0, 0), 1.0);
        aCurve.InsertPoint(2, new Point3D(2, 0, 0), 2.0);
        Assert.Equal(4, aCurve.ParametersNb());
        for (int i = 0; i < 4; i++)
        {
            Assert.Equal(i, aCurve.GetParameter(i));
            Assert.Equal(i, aCurve.GetPoint(i).X());
        }

        // references for writing, as OCCT's
        aCurve.GetParameter(1) = 1.5;
        aCurve.GetPoint(1).SetY(7);
        Assert.Equal(1.5, aCurve.GetParameter(1));
        Assert.Equal(7, aCurve.GetPoint(1).Y());

        aCurve.RemovePoint(1);
        Assert.Equal(3, aCurve.ParametersNb());
        Assert.Equal(2.0, aCurve.GetParameter(1));
        Assert.Throws<ArgumentOutOfRangeException>(() => aCurve.GetParameter(3));

        var anAdaptor = new IMeshData_ParametersListArrayAdaptor<IMeshData_Curve>(aCurve);
        Assert.Equal(0, anAdaptor.Lower());
        Assert.Equal(2, anAdaptor.Upper());
        Assert.Equal(2.0, anAdaptor.Value(1));

        aCurve.Clear(true);
        Assert.Equal(2, aCurve.ParametersNb());
        Assert.Equal(0.0, aCurve.GetParameter(0));
        Assert.Equal(3.0, aCurve.GetParameter(1));
        Assert.Equal(3.0, aCurve.GetPoint(1).X());
        aCurve.Clear(false);
        Assert.Equal(0, aCurve.ParametersNb());

        BRepMeshData_Face aDFace = new(new BRepBuilderAPI_MakeFace(new Plane3D(), 0, 1, 0, 1).Face());
        var aPCurve = new BRepMeshData_PCurve(aDFace, Reversed);
        Assert.False(aPCurve.IsForward());
        Assert.False(aPCurve.IsInternal());
        Assert.Same(aDFace, aPCurve.GetFace());
        aPCurve.AddPoint(new Point2D(0, 0), 0.0);
        aPCurve.AddPoint(new Point2D(1, 0), 1.0);
        aPCurve.InsertPoint(1, new Point2D(0.5, 0), 0.5);
        Assert.Equal(0, aPCurve.GetIndex(1));
        aPCurve.GetIndex(1) = 42;
        Assert.Equal(42, aPCurve.GetIndex(1));
        Assert.Equal(0.5, aPCurve.GetPoint(1).X());
        Assert.Throws<OutOfRangeException>(() => aPCurve.GetPoint(3));
        Assert.Throws<OutOfRangeException>(() => aPCurve.GetIndex(-1));
        Assert.Throws<OutOfRangeException>(() => aPCurve.GetParameter(3));
        aPCurve.RemovePoint(0);
        Assert.Equal(2, aPCurve.ParametersNb());
        Assert.Equal(42, aPCurve.GetIndex(0));
        aPCurve.Clear(true);
        Assert.Equal(2, aPCurve.ParametersNb());
        Assert.True(new BRepMeshData_PCurve(aDFace, Internal).IsInternal());
    }

    [Fact]
    public void Model_of_a_box_built_by_hand()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(2, 3, 4));
        var aModel = new BRepMeshData_Model(aBox);
        Assert.True(aModel.GetShape().IsSame(aBox));
        aModel.SetMaxSize(4.0);
        Assert.Equal(4.0, aModel.GetMaxSize());

        // Edges first, mapped by shape, then faces with their wires and pcurves (as BRepMesh_ShapeVisitor does).
        var anEdgeMap = new NCollection_DataMap<Shape, int>(ShapeHasher.Instance);
        for (var anExp = new Explorer(aBox, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            if (!anEdgeMap.IsBound(anExp.Current()))
            {
                anEdgeMap.Bind(anExp.Current(), aModel.EdgesNb());
                IMeshData_Edge aDEdge = aModel.AddEdge(ShapeCasts.Edge(anExp.Current()));
                Assert.Same(aDEdge, aModel.GetEdge(aModel.EdgesNb() - 1));
                Assert.True(aDEdge.IsFree());
                Assert.Equal(RealLast(), aDEdge.GetDeflection());
                Assert.Equal(RealLast(), aDEdge.GetAngularDeflection());
                Assert.Equal(Tool.SameParameter(aDEdge.GetEdge()), aDEdge.GetSameParam());
                Assert.Equal(Tool.SameRange(aDEdge.GetEdge()), aDEdge.GetSameRange());
                Assert.False(aDEdge.GetDegenerated());
                Assert.NotNull(aDEdge.GetCurve());
            }
        }
        Assert.Equal(12, aModel.EdgesNb());

        for (var aFaceExp = new Explorer(aBox, ShapeType.Face); aFaceExp.More(); aFaceExp.Next())
        {
            Face aFace = ShapeCasts.Face(aFaceExp.Current().Oriented(Forward));
            IMeshData_Face aDFace = aModel.AddFace(aFace);
            Assert.True(aDFace.IsValid());
            Assert.True(aDFace.GetSurface().Face().IsEqual(aFace));
            Wire aWire = BrepTools.OuterWire(aFace);
            IMeshData_Wire aDWire = aDFace.AddWire(aWire, 4);
            Assert.Same(aDWire, aDFace.GetWire(0));
            Assert.True(aDWire.GetWire().IsSame(aWire));
            for (var anIt = new Iterator(aWire); anIt.More(); anIt.Next())
            {
                IMeshData_Edge aDEdge = aModel.GetEdge(anEdgeMap.Find(anIt.Value()));
                int anIndex = aDWire.AddEdge(aDEdge, anIt.Value().Orientation());
                Assert.Same(aDEdge, aDWire.GetEdge(anIndex));
                Assert.Equal(anIt.Value().Orientation(), aDWire.GetEdgeOrientation(anIndex));
                IMeshData_PCurve aPCurve = aDEdge.AddPCurve(aDFace, anIt.Value().Orientation());
                Assert.Same(aPCurve, aDEdge.GetPCurve(aDFace, anIt.Value().Orientation()));
            }
            Assert.Equal(4, aDWire.EdgesNb());
        }
        Assert.Equal(6, aModel.FacesNb());

        // Each edge of a box is on two faces, with one pcurve on each.
        for (int i = 0; i < aModel.EdgesNb(); i++)
        {
            IMeshData_Edge aDEdge = aModel.GetEdge(i);
            Assert.Equal(2, aDEdge.PCurvesNb());
            Assert.False(aDEdge.IsFree());
            IMeshData_Face aF0 = aDEdge.GetPCurve(0).GetFace();
            Assert.Equal([0], aDEdge.GetPCurves(aF0));
            Assert.NotSame(aF0, aDEdge.GetPCurve(1).GetFace());
        }

        // Clear keeps the end points of the curve and of every pcurve.
        IMeshData_Edge anE = aModel.GetEdge(0);
        anE.GetCurve().AddPoint(new Point3D(), 0);
        anE.GetCurve().AddPoint(new Point3D(), 1);
        anE.GetCurve().AddPoint(new Point3D(), 2);
        anE.GetPCurve(1).AddPoint(new Point2D(), 0);
        anE.GetPCurve(1).AddPoint(new Point2D(), 1);
        anE.GetPCurve(1).AddPoint(new Point2D(), 2);
        anE.Clear(true);
        Assert.Equal(2, anE.GetCurve().ParametersNb());
        Assert.Equal(2, anE.GetPCurve(1).ParametersNb());
        anE.Clear(false);
        Assert.Equal(0, anE.GetCurve().ParametersNb());
    }

    [Fact]
    public void Seam_edge_has_two_pcurves_on_one_face()
    {
        Face aCyl = TestShapes.CylinderFace(2, 3);
        BRepMeshData_Face aDFace = new(aCyl);
        Edge aSeam = new();
        for (var anExp = new Explorer(aCyl, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            if (Tool.IsClosed(ShapeCasts.Edge(anExp.Current()), aCyl))
            {
                aSeam = ShapeCasts.Edge(anExp.Current());
            }
        }
        Assert.False(aSeam.IsNull);

        BRepMeshData_Edge aDEdge = new(aSeam);
        IMeshData_PCurve aFwd = aDEdge.AddPCurve(aDFace, Forward);
        IMeshData_PCurve aRev = aDEdge.AddPCurve(aDFace, Reversed);
        Assert.Same(aFwd, aDEdge.GetPCurve(aDFace, Forward));
        Assert.Same(aRev, aDEdge.GetPCurve(aDFace, Reversed));
        Assert.Equal([0, 1], aDEdge.GetPCurves(aDFace));

        // The pcurves of another face aren't found by this one.
        BRepMeshData_Face anOther = new(aCyl);
        Assert.Throws<NoSuchObjectException>(() => aDEdge.GetPCurves(anOther));
    }

    [Fact]
    public void Status_owners_combine_flags()
    {
        BRepMeshData_Face aDFace = new(new BRepBuilderAPI_MakeFace(new Plane3D(), 0, 1, 0, 1).Face());
        Assert.True(aDFace.IsEqual(IMeshData_NoError));
        aDFace.SetStatus(IMeshData_ReMesh);
        Assert.True(aDFace.IsValid());
        aDFace.SetStatus(IMeshData_UnorientedWire);
        Assert.False(aDFace.IsValid()); // valid only with exactly one of NoError, ReMesh, UnorientedWire
        Assert.True(aDFace.IsSet(IMeshData_ReMesh));
        Assert.Equal(0x18, aDFace.GetStatusMask());
        aDFace.UnsetStatus(IMeshData_ReMesh);
        Assert.True(aDFace.IsEqual(IMeshData_UnorientedWire));
        Assert.True(aDFace.IsValid());
        aDFace.SetStatus(IMeshData_Failure);
        Assert.False(aDFace.IsValid());

        IMeshData_StatusOwner aWire = new BRepMeshData_Wire(new Wire(), 0);
        aWire.SetStatus(IMeshData_OpenWire);
        Assert.Equal((int)IMeshData_OpenWire, aWire.GetStatusMask());

        IMeshData_StatusOwner anEdge = new BRepMeshData_Edge(TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(1, 0, 0)));
        anEdge.SetStatus(IMeshData_Reused);
        anEdge.SetStatus(IMeshData_Outdated);
        Assert.Equal(0xC0, anEdge.GetStatusMask());
    }

    [Fact]
    public void Pointer_keyed_maps_iterate_in_insertion_order()
    {
        // Decision 490: IMeshData's maps keyed by IFacePtr/IEdgePtr compare by reference and iterate in insertion
        // order, whatever the objects' hash codes.
        Edge anEdge = TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(1, 0, 0));
        var anEdges = Enumerable.Range(0, 200).Select(_ => (IMeshData_Edge)new BRepMeshData_Edge(anEdge)).ToList();
        var aMap = new NCollection_Map<IMeshData_Edge>(ReferenceEqualityComparer.Instance);
        foreach (IMeshData_Edge aE in anEdges)
        {
            Assert.True(aMap.Add(aE));
        }
        Assert.False(aMap.Add(anEdges[7]));
        Assert.Equal(anEdges, aMap.ToList());

        var aUnited = new NCollection_Map<IMeshData_Edge>(ReferenceEqualityComparer.Instance);
        aUnited.Add(anEdges[150]);
        NCollection_MapAlgo.Unite(aUnited, aMap);
        Assert.Equal([anEdges[150], .. anEdges.Where(e => e != anEdges[150])], aUnited.ToList());
    }

    [Fact]
    public void EdgeParameterProvider_scales_parameters_of_an_edge_not_SameParameter()
    {
        // A straight edge on a plane: 3D range [0, 10], pcurve range [0, 10]; stored parameters on [0, 1].
        Face aFace = new BRepBuilderAPI_MakeFace(new Plane3D(), -1, 11, -1, 1).Face();
        Edge anEdge = ShapeCasts.Edge(TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(10, 0, 0)));
        BRepMeshData_Face aDFace = new(aFace);
        BRepMeshData_Edge aDEdge = new(anEdge);
        var aStored = new NCollection_HArray1<double>(1, 3);
        aStored[1] = 0.0;
        aStored[2] = 0.5;
        aStored[3] = 1.0;

        // SameParameter: the stored values as they are.
        Assert.True(aDEdge.GetSameParam());
        var aSame = new BRepMesh_EdgeParameterProvider<NCollection_HArray1<double>>(aDEdge, Forward, aDFace, aStored);
        Assert.Equal(0.5, aSame.Parameter(2, new Point3D(5, 0, 0)));
        Assert.NotNull(aSame.GetPCurve());

        // Not SameParameter: scaled to the pcurve's range and refined by projection.
        aDEdge.SetSameParam(false);
        var aScaled = new BRepMesh_EdgeParameterProvider<NCollection_HArray1<double>>(aDEdge, Forward, aDFace, aStored);
        Assert.Equal(0.0, aScaled.Parameter(1, new Point3D(0, 0, 0)), 1e-12);
        Assert.Equal(5.0, aScaled.Parameter(2, new Point3D(5, 0, 0)), 1e-9);
        Assert.Equal(10.0, aScaled.Parameter(3, new Point3D(10, 0, 0)), 1e-9);

        // Over a discrete curve through its array adaptor
        var aCurve = new BRepMeshData_Curve();
        aCurve.AddPoint(new Point3D(0, 0, 0), 0.0);
        aCurve.AddPoint(new Point3D(10, 0, 0), 10.0);
        var aProvider = new BRepMesh_EdgeParameterProvider<IMeshData_ParametersListArrayAdaptor<IMeshData_Curve>>();
        aProvider.Init(aDEdge, Forward, aDFace, new IMeshData_ParametersListArrayAdaptor<IMeshData_Curve>(aCurve));
        Assert.Equal(10.0, aProvider.Parameter(1, new Point3D(10, 0, 0)), 1e-9);
    }
}
