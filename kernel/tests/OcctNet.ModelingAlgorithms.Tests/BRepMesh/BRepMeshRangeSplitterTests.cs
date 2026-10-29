// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_SurfaceType;
using static OcctNet.ModelingAlgorithms.Tests.BRepMeshModelKit;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Unit 4D: the range splitters (UVParam, NURBS, Cylinder, Sphere, Torus, Extrusion, Undefined, BoundaryParams) on the
/// faces of discrete models, as the face algorithms drive them: Reset, the border points, AdjustRange, then
/// GenerateSurfaceNodes.
/// </summary>
public class BRepMeshRangeSplitterTests
{
    private static TopoDS_Shape BezierFace()
    {
        var aPoles = new NCollection_Array2<gp_Pnt>(1, 3, 1, 3);
        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                aPoles[i, j] = new gp_Pnt(i * 2.0, j * 2.0, (i == 2 && j == 2) ? 3.0 : 0.0);
            }
        }
        return new BRepBuilderAPI_MakeFace(new Geom_BezierSurface(aPoles), Precision.Confusion()).Face();
    }

    private static TopoDS_Shape ExtrusionFace()
    {
        var aPoles = new NCollection_Array1<gp_Pnt>(1, 4);
        aPoles[1] = new gp_Pnt(0, 0, 0);
        aPoles[2] = new gp_Pnt(1, 2, 0);
        aPoles[3] = new gp_Pnt(3, -1, 0);
        aPoles[4] = new gp_Pnt(4, 1, 0);
        TopoDS_Edge anEdge = new BRepBuilderAPI_MakeEdge(new Geom_BezierCurve(aPoles)).Edge();
        return new BRepPrimAPI_MakePrism(anEdge, new gp_Vec(0, 0, 5)).Shape();
    }

    // The first face of the model on a surface of the type.
    private static IMeshData_Face FaceOfType(IMeshData_Model theModel, GeomAbs_SurfaceType theType)
    {
        for (int i = 0; i < theModel.FacesNb(); ++i)
        {
            IMeshData_Face aDFace = theModel.GetFace(i);
            if (aDFace.GetSurface().GetType() == theType)
            {
                return aDFace;
            }
        }
        Assert.Fail($"no face of type {theType}");
        return null!;
    }

    // Drives the splitter as the face algorithms do; returns the generated nodes.
    private static NCollection_List<gp_Pnt2d>? Split(BRepMesh_DefaultRangeSplitter theSplitter, IMeshData_Face theDFace,
                                                    in IMeshTools_Parameters theParams)
    {
        theSplitter.Reset(theDFace, theParams);
        for (int w = 0; w < theDFace.WiresNb(); ++w)
        {
            IMeshData_Wire aDWire = theDFace.GetWire(w);
            for (int e = 0; e < aDWire.EdgesNb(); ++e)
            {
                IMeshData_Edge aDEdge = aDWire.GetEdge(e);
                for (int p = 0; p < aDEdge.PCurvesNb(); ++p)
                {
                    IMeshData_PCurve aDPCurve = aDEdge.GetPCurve(p);
                    if (!ReferenceEquals(aDPCurve.GetFace(), theDFace))
                    {
                        continue;
                    }
                    for (int k = 0; k < aDPCurve.ParametersNb(); ++k)
                    {
                        theSplitter.AddPoint(aDPCurve.GetPoint(k));
                    }
                }
            }
        }
        theSplitter.AdjustRange();
        Assert.True(theSplitter.IsValid());
        return theSplitter.GenerateSurfaceNodes(theParams);
    }

    private static List<(long, long)> Bits(NCollection_List<gp_Pnt2d>? theNodes)
    {
        var aBits = new List<(long, long)>();
        if (theNodes is not null)
        {
            foreach (gp_Pnt2d aNode in theNodes)
            {
                aBits.Add((BitConverter.DoubleToInt64Bits(aNode.X()), BitConverter.DoubleToInt64Bits(aNode.Y())));
            }
        }
        return aBits;
    }

    private static void AssertInside(BRepMesh_DefaultRangeSplitter theSplitter, NCollection_List<gp_Pnt2d> theNodes)
    {
        (double first, double second) aU = theSplitter.GetRangeU();
        (double first, double second) aV = theSplitter.GetRangeV();
        foreach (gp_Pnt2d aNode in theNodes)
        {
            Assert.InRange(aNode.X(), aU.first, aU.second);
            Assert.InRange(aNode.Y(), aV.first, aV.second);
        }
    }

    // Checks the splitter's nodes: inside the range, at least theMinNodes, the same bits from a new splitter and
    // after a Reset of the same one.
    private static NCollection_List<gp_Pnt2d> Check<TSplitter>(in TopoDS_Shape theShape, GeomAbs_SurfaceType theType,
                                                               int theMinNodes, double theDeflection = 0.1)
        where TSplitter : BRepMesh_DefaultRangeSplitter, new()
    {
        IMeshTools_Parameters aParams = Params(theDeflection);
        IMeshData_Model aModel = Model(theShape, aParams);
        IMeshData_Face aDFace = FaceOfType(aModel, theType);

        var aSplitter = new TSplitter();
        NCollection_List<gp_Pnt2d>? aNodes = Split(aSplitter, aDFace, aParams);
        Assert.NotNull(aNodes);
        Assert.True(aNodes.Size() >= theMinNodes, $"{typeof(TSplitter).Name}: {aNodes.Size()} nodes");
        AssertInside(aSplitter, aNodes);

        Assert.Equal(Bits(aNodes), Bits(Split(new TSplitter(), aDFace, aParams)));
        Assert.Equal(Bits(aNodes), Bits(Split(aSplitter, aDFace, aParams)));
        return aNodes;
    }

    [Fact]
    public void UVParam_CollectsNothingByItself_AndResetClears()
    {
        IMeshTools_Parameters aParams = Params(0.1);
        IMeshData_Face aDFace = Model(new BRepPrimAPI_MakeBox(1, 2, 3).Solid(), aParams).GetFace(0);
        var aSplitter = new BRepMesh_UVParamRangeSplitter();
        aSplitter.Reset(aDFace, aParams);
        aSplitter.GetParametersU().Add(0.5);
        aSplitter.GetParametersV().Add(0.25);
        Assert.Equal(1, aSplitter.GetParametersU().Extent());
        aSplitter.Reset(aDFace, aParams);
        Assert.Equal(0, aSplitter.GetParametersU().Extent());
        Assert.Equal(0, aSplitter.GetParametersV().Extent());
        Assert.Null(aSplitter.GenerateSurfaceNodes(aParams));
    }

    [Fact]
    public void Cylinder_NoInnerNodes_DeltaFromAngularStep()
    {
        // OCCT leaves the V subdivision out, so a cylinder gets no grid nodes: one row at most would sit on the
        // border, which the loop's half-step margin excludes.
        NCollection_List<gp_Pnt2d> aNodes = Check<BRepMesh_CylinderRangeSplitter>(
            new BRepPrimAPI_MakeCylinder(2, 5).Solid(), GeomAbs_Cylinder, 0);
        Assert.Equal(0, aNodes.Size());

        IMeshTools_Parameters aParams = Params(0.1);
        IMeshData_Face aDFace = FaceOfType(Model(new BRepPrimAPI_MakeCylinder(2, 5).Solid(), aParams), GeomAbs_Cylinder);
        var aSplitter = new BRepMesh_CylinderRangeSplitter();
        Split(aSplitter, aDFace, aParams);
        Assert.Equal(1.0, aSplitter.GetDelta().second);
        double aDu = GCPnts_TangentialDeflection.ArcAngularStep(2.0, aDFace.GetDeflection(), aParams.Angle, aParams.MinSize);
        (double first, double second) aV = aSplitter.GetRangeV();
        Assert.Equal(aDu / Math.Max(5.0, aV.second - aV.first), aSplitter.GetDelta().first, 1e-9);
    }

    [Fact]
    public void Sphere_StaggeredGrid()
    {
        NCollection_List<gp_Pnt2d> aNodes =
            Check<BRepMesh_SphereRangeSplitter>(new BRepPrimAPI_MakeSphere(3).Solid(), GeomAbs_Sphere, 20);

        // Rows alternate between an offset of half a step and none.
        var aRows = new SortedDictionary<double, double>();
        foreach (gp_Pnt2d aNode in aNodes)
        {
            if (!aRows.ContainsKey(aNode.Y()))
            {
                aRows[aNode.Y()] = aNode.X();
            }
        }
        Assert.True(aRows.Count >= 2);
        Assert.NotEqual(aRows.Values.First(), aRows.Values.Skip(1).First());
    }

    [Fact]
    public void Torus_GridFromBorderParameters()
    {
        Check<BRepMesh_TorusRangeSplitter>(new BRepPrimAPI_MakeTorus(5, 1).Solid(), GeomAbs_Torus, 20);
        Check<BRepMesh_TorusRangeSplitter>(new BRepPrimAPI_MakeTorus(5, 1, Math.PI).Solid(), GeomAbs_Torus, 10);
    }

    [Fact]
    public void Torus_RegistersBorderParameters()
    {
        IMeshTools_Parameters aParams = Params(0.1);
        IMeshData_Face aDFace = FaceOfType(Model(new BRepPrimAPI_MakeTorus(5, 1).Solid(), aParams), GeomAbs_Torus);
        var aSplitter = new BRepMesh_TorusRangeSplitter();
        Split(aSplitter, aDFace, aParams);
        Assert.True(aSplitter.GetParametersU().Extent() > 2);
        Assert.True(aSplitter.GetParametersV().Extent() > 2);
    }

    [Fact]
    public void NURBS_BezierFace()
    {
        NCollection_List<gp_Pnt2d> aNodes = Check<BRepMesh_NURBSRangeSplitter>(BezierFace(), GeomAbs_BezierSurface, 4, 0.01);
        Assert.All(aNodes, aNode => Assert.InRange(aNode.X(), 0.0, 1.0));
    }

    [Fact]
    public void NURBS_BezierOutOfRangeIsInvalid()
    {
        IMeshTools_Parameters aParams = Params(0.1);
        IMeshData_Face aDFace = Model(BezierFace(), aParams).GetFace(0);
        var aSplitter = new BRepMesh_NURBSRangeSplitter();
        aSplitter.Reset(aDFace, aParams);
        // Outside the surface's domain without overlapping it, so AdjustRange keeps the range.
        aSplitter.AddPoint(new gp_Pnt2d(-3.0, -3.0));
        aSplitter.AddPoint(new gp_Pnt2d(-2.0, -2.0));
        aSplitter.AdjustRange();
        Assert.False(aSplitter.IsValid());
    }

    [Fact]
    public void BoundaryParams_BezierFace()
    {
        Check<BRepMesh_BoundaryParamsRangeSplitter>(BezierFace(), GeomAbs_BezierSurface, 4, 0.01);
    }

    [Fact]
    public void Extrusion_BezierProfile()
    {
        Check<BRepMesh_ExtrusionRangeSplitter>(ExtrusionFace(), GeomAbs_SurfaceOfExtrusion, 4, 0.01);
    }

    [Fact]
    public void Undefined_OneIntervalEachWay()
    {
        // One interval along each direction (the surface's own bounds), split at its middle where the normal is
        // defined: a grid of at most 3 by 3 parameters.
        NCollection_List<gp_Pnt2d> aNodes =
            Check<BRepMesh_UndefinedRangeSplitter>(BezierFace(), GeomAbs_BezierSurface, 1, 0.01);
        Assert.True(aNodes.Size() <= 9);
    }

    [Fact]
    public void Undefined_OnExtrusion_NoParameters()
    {
        // An extrusion's V range is infinite: its single interval's bounds are outside the face's range, so the
        // splitter has no parameters and generates nothing (null), as OCCT's.
        IMeshTools_Parameters aParams = Params(0.01);
        IMeshData_Face aDFace = FaceOfType(Model(ExtrusionFace(), aParams), GeomAbs_SurfaceOfExtrusion);
        Assert.Null(Split(new BRepMesh_UndefinedRangeSplitter(), aDFace, aParams));
    }
}
