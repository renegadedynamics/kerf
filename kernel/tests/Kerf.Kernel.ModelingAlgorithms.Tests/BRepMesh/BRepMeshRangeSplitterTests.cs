// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Measurement;
using Kerf.Engine.Meshing;
using Kerf.Engine.Meshing.Model;
using Kerf.Engine.Meshing.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Kernel.ModelingAlgorithms.Tests.BRepMeshModelKit;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Unit 4D: the range splitters (UVParam, NURBS, Cylinder, Sphere, Torus, Extrusion, Undefined, BoundaryParams) on the
/// faces of discrete models, as the face algorithms drive them: Reset, the border points, AdjustRange, then
/// GenerateSurfaceNodes.
/// </summary>
public class BRepMeshRangeSplitterTests
{
    private static Shape BezierFace()
    {
        var aPoles = new NCollection_Array2<Point3D>(1, 3, 1, 3);
        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                aPoles[i, j] = new Point3D(i * 2.0, j * 2.0, (i == 2 && j == 2) ? 3.0 : 0.0);
            }
        }
        return new MakeFace(new global::Kerf.Geometry.BezierSurface(aPoles), global::Kerf.Engine.Foundation.Precision.Confusion()).Face();
    }

    private static Shape ExtrusionFace()
    {
        var aPoles = new NCollection_Array1<Point3D>(1, 4);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(1, 2, 0);
        aPoles[3] = new Point3D(3, -1, 0);
        aPoles[4] = new Point3D(4, 1, 0);
        Edge anEdge = new MakeEdge(new BezierCurve3D(aPoles)).Edge();
        return new ApiMakePrism(anEdge, new Vector3D(0, 0, 5)).Shape();
    }

    // The first face of the model on a surface of the type.
    private static FaceModel FaceOfType(ModelModel theModel, SurfaceKind theType)
    {
        for (int i = 0; i < theModel.FacesNb(); ++i)
        {
            FaceModel aDFace = theModel.GetFace(i);
            if (aDFace.GetSurface().GetType() == theType)
            {
                return aDFace;
            }
        }
        Assert.Fail($"no face of type {theType}");
        return null!;
    }

    // Drives the splitter as the face algorithms do; returns the generated nodes.
    private static NCollection_List<Point2D>? Split(DefaultRangeSplitter theSplitter, FaceModel theDFace,
                                                    in ToolParameters theParams)
    {
        theSplitter.Reset(theDFace, theParams);
        for (int w = 0; w < theDFace.WiresNb(); ++w)
        {
            WireModel aDWire = theDFace.GetWire(w);
            for (int e = 0; e < aDWire.EdgesNb(); ++e)
            {
                EdgeModel aDEdge = aDWire.GetEdge(e);
                for (int p = 0; p < aDEdge.PCurvesNb(); ++p)
                {
                    PCurveModel aDPCurve = aDEdge.GetPCurve(p);
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
        Assert.True(theSplitter.IsValid);
        return theSplitter.GenerateSurfaceNodes(theParams);
    }

    private static List<(long, long)> Bits(NCollection_List<Point2D>? theNodes)
    {
        var aBits = new List<(long, long)>();
        if (theNodes is not null)
        {
            foreach (Point2D aNode in theNodes)
            {
                aBits.Add((BitConverter.DoubleToInt64Bits(aNode.X()), BitConverter.DoubleToInt64Bits(aNode.Y())));
            }
        }
        return aBits;
    }

    private static void AssertInside(DefaultRangeSplitter theSplitter, NCollection_List<Point2D> theNodes)
    {
        (double first, double second) aU = theSplitter.GetRangeU();
        (double first, double second) aV = theSplitter.GetRangeV();
        foreach (Point2D aNode in theNodes)
        {
            Assert.InRange(aNode.X(), aU.first, aU.second);
            Assert.InRange(aNode.Y(), aV.first, aV.second);
        }
    }

    // Checks the splitter's nodes: inside the range, at least theMinNodes, the same bits from a new splitter and
    // after a Reset of the same one.
    private static NCollection_List<Point2D> Check<TSplitter>(in Shape theShape, SurfaceKind theType,
                                                               int theMinNodes, double theDeflection = 0.1)
        where TSplitter : DefaultRangeSplitter, new()
    {
        ToolParameters aParams = Params(theDeflection);
        ModelModel aModel = Model(theShape, aParams);
        FaceModel aDFace = FaceOfType(aModel, theType);

        var aSplitter = new TSplitter();
        NCollection_List<Point2D>? aNodes = Split(aSplitter, aDFace, aParams);
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
        ToolParameters aParams = Params(0.1);
        FaceModel aDFace = Model(new MakeBox(1, 2, 3).Solid(), aParams).GetFace(0);
        var aSplitter = new UVParamRangeSplitter();
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
        NCollection_List<Point2D> aNodes = Check<CylinderRangeSplitter>(
            new ApiMakeCylinder(2, 5).Solid(), SurfaceKind.Cylinder, 0);
        Assert.Equal(0, aNodes.Size());

        ToolParameters aParams = Params(0.1);
        FaceModel aDFace = FaceOfType(Model(new ApiMakeCylinder(2, 5).Solid(), aParams), SurfaceKind.Cylinder);
        var aSplitter = new CylinderRangeSplitter();
        Split(aSplitter, aDFace, aParams);
        Assert.Equal(1.0, aSplitter.GetDelta().second);
        double aDu = TangentialDeflection.ArcAngularStep(2.0, aDFace.GetDeflection(), aParams.Angle, aParams.MinSize);
        (double first, double second) aV = aSplitter.GetRangeV();
        Assert.Equal(aDu / Math.Max(5.0, aV.second - aV.first), aSplitter.GetDelta().first, 1e-9);
    }

    [Fact]
    public void Sphere_StaggeredGrid()
    {
        NCollection_List<Point2D> aNodes =
            Check<SphereRangeSplitter>(new MakeSphere(3).Solid(), SurfaceKind.Sphere, 20);

        // Rows alternate between an offset of half a step and none.
        var aRows = new SortedDictionary<double, double>();
        foreach (Point2D aNode in aNodes)
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
        Check<TorusRangeSplitter>(new MakeTorus(5, 1).Solid(), SurfaceKind.Torus, 20);
        Check<TorusRangeSplitter>(new MakeTorus(5, 1, Math.PI).Solid(), SurfaceKind.Torus, 10);
    }

    [Fact]
    public void Torus_RegistersBorderParameters()
    {
        ToolParameters aParams = Params(0.1);
        FaceModel aDFace = FaceOfType(Model(new MakeTorus(5, 1).Solid(), aParams), SurfaceKind.Torus);
        var aSplitter = new TorusRangeSplitter();
        Split(aSplitter, aDFace, aParams);
        Assert.True(aSplitter.GetParametersU().Extent() > 2);
        Assert.True(aSplitter.GetParametersV().Extent() > 2);
    }

    [Fact]
    public void NURBS_BezierFace()
    {
        NCollection_List<Point2D> aNodes = Check<NurbsRangeSplitter>(BezierFace(), SurfaceKind.BezierSurface, 4, 0.01);
        Assert.All(aNodes, aNode => Assert.InRange(aNode.X(), 0.0, 1.0));
    }

    [Fact]
    public void NURBS_BezierOutOfRangeIsInvalid()
    {
        ToolParameters aParams = Params(0.1);
        FaceModel aDFace = Model(BezierFace(), aParams).GetFace(0);
        var aSplitter = new NurbsRangeSplitter();
        aSplitter.Reset(aDFace, aParams);
        // Outside the surface's domain without overlapping it, so AdjustRange keeps the range.
        aSplitter.AddPoint(new Point2D(-3.0, -3.0));
        aSplitter.AddPoint(new Point2D(-2.0, -2.0));
        aSplitter.AdjustRange();
        Assert.False(aSplitter.IsValid);
    }

    [Fact]
    public void BoundaryParams_BezierFace()
    {
        Check<BoundaryParamsRangeSplitter>(BezierFace(), SurfaceKind.BezierSurface, 4, 0.01);
    }

    [Fact]
    public void Extrusion_BezierProfile()
    {
        Check<ExtrusionRangeSplitter>(ExtrusionFace(), SurfaceOfExtrusion, 4, 0.01);
    }

    [Fact]
    public void Undefined_OneIntervalEachWay()
    {
        // One interval along each direction (the surface's own bounds), split at its middle where the normal is
        // defined: a grid of at most 3 by 3 parameters.
        NCollection_List<Point2D> aNodes =
            Check<UndefinedRangeSplitter>(BezierFace(), SurfaceKind.BezierSurface, 1, 0.01);
        Assert.True(aNodes.Size() <= 9);
    }

    [Fact]
    public void Undefined_OnExtrusion_NoParameters()
    {
        // An extrusion's V range is infinite: its single interval's bounds are outside the face's range, so the
        // splitter has no parameters and generates nothing (null), as OCCT's.
        ToolParameters aParams = Params(0.01);
        FaceModel aDFace = FaceOfType(Model(ExtrusionFace(), aParams), SurfaceOfExtrusion);
        Assert.Null(Split(new UndefinedRangeSplitter(), aDFace, aParams));
    }
}
