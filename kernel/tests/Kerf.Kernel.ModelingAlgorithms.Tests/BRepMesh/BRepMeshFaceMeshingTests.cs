// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Geometry;
using Kerf.Engine.Measurement;
using Kerf.Engine.Meshing;
using Kerf.Engine.Meshing.Model;
using Kerf.Engine.Meshing.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.Meshing.Tools.MeshAlgoType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Unit 4D's face meshing through BRepMesh_IncrementalMesh, for what native OCCT's binding can't set (the Delabella
/// algorithm and the parameters beyond the five arguments): every face meshed, valid triangles, areas close to the
/// exact ones, parallel equal to serial. Native Watson meshes are compared bit for bit by the oracle.
/// </summary>
public class BRepMeshFaceMeshingTests
{
    private static IEnumerable<(string Name, Shape Shape)> Shapes()
    {
        Frame3D anAx2 = new(new Point3D(1, 2, 3), new Direction3D(0.2, 0.3, 1.0));
        yield return ("box", new MakeBox(anAx2, 3, 4, 5).Shape());
        yield return ("cylinder", new ApiMakeCylinder(anAx2, 2, 5).Shape());
        yield return ("cone", new ApiMakeCone(anAx2, 2, 0.5, 4).Shape());
        yield return ("sphere", new MakeSphere(anAx2, 3).Shape());
        yield return ("torus", new MakeTorus(anAx2, 5, 1.5).Shape());

        // A surface of revolution and an extrusion of a B-spline (the NURBS, BoundaryParams and Extrusion splitters).
        NCollection_Array1<Point3D> aPoles = new(1, 5);
        for (int i = 1; i <= 5; i++)
        {
            aPoles[i] = new Point3D(2 + 0.4 * Math.Sin(i), 0, i);
        }
        BezierCurve3D aCurve = new(aPoles);
        Edge anEdge = new MakeEdge(aCurve).Edge();
        yield return ("revolution", new ApiMakeRevol(anEdge, new Axis3D(Constants.Origin(), Constants.DZ()), 4.0).Shape());
        yield return ("extrusion", new ApiMakePrism(anEdge, new Vector3D(3, 1, 0)).Shape());
        NCollection_Array2<Point3D> aNet = new(1, 3, 1, 3);
        for (int i = 1; i <= 3; i++)
        {
            for (int j = 1; j <= 3; j++)
            {
                aNet[i, j] = new Point3D(i, j, (i == 2 && j == 2) ? 1.5 : 0.0);
            }
        }
        BezierSurface aSurface = new(aNet);
        yield return ("bezier", new MakeFace(aSurface, global::Kerf.Engine.Foundation.Precision.Confusion()).Face());
    }

    private static List<Triangulation?> Triangulations(in Shape theShape)
    {
        List<Triangulation?> aList = [];
        for (Explorer anExp = new(theShape, ShapeType.Face); anExp.More(); anExp.Next())
        {
            aList.Add(Tool.Triangulation(ShapeCasts.Face(anExp.Current()), out _));
        }
        return aList;
    }

    // Every face meshed with valid triangles; the mesh's area within the given share of the exact area.
    private static void CheckMesh(string theWhat, in Shape theShape, double theAreaTolerance)
    {
        double aMeshArea = 0.0;
        foreach (Triangulation? aT in Triangulations(theShape))
        {
            Assert.True(aT is not null, theWhat + ": a face without mesh");
            Assert.True(aT.TriangleCount > 0, theWhat);
            Assert.True(aT.HasUVNodes, theWhat);
            for (int i = 1; i <= aT.TriangleCount; i++)
            {
                aT.Triangle(i).Get(out int a, out int b, out int c);
                Assert.True(a >= 1 && b >= 1 && c >= 1 && a <= aT.NodeCount && b <= aT.NodeCount && c <= aT.NodeCount, theWhat);
                Assert.True(a != b && b != c && a != c, theWhat + ": degenerate triangle");
                Xyz p = aT.Node(a).XYZ(), q = aT.Node(b).XYZ(), r = aT.Node(c).XYZ();
                aMeshArea += 0.5 * (q - p).Crossed(r - p).Modulus();
            }
        }
        double anArea = MassPropertiesBuilder.SurfaceProperties(theShape).Mass();
        Assert.True(Math.Abs(aMeshArea - anArea) <= theAreaTolerance * anArea,
                    $"{theWhat}: mesh area {aMeshArea}, exact {anArea}");
    }

    private static string Dump(in Shape theShape)
    {
        System.Text.StringBuilder aBuilder = new();
        foreach (Triangulation? aT in Triangulations(theShape))
        {
            if (aT is null)
            {
                aBuilder.Append("null\n");
                continue;
            }
            for (int i = 1; i <= aT.NodeCount; i++)
            {
                Point3D p = aT.Node(i);
                Point2D uv = aT.UVNode(i);
                aBuilder.Append(BitConverter.DoubleToInt64Bits(p.X)).Append(BitConverter.DoubleToInt64Bits(p.Y))
                        .Append(BitConverter.DoubleToInt64Bits(p.Z)).Append(BitConverter.DoubleToInt64Bits(uv.X))
                        .Append(BitConverter.DoubleToInt64Bits(uv.Y)).Append(';');
            }
            for (int i = 1; i <= aT.TriangleCount; i++)
            {
                aT.Triangle(i).Get(out int a, out int b, out int c);
                aBuilder.Append(a).Append(',').Append(b).Append(',').Append(c).Append(';');
            }
            aBuilder.Append(BitConverter.DoubleToInt64Bits(aT.Deflection())).Append('\n');
        }
        return aBuilder.ToString();
    }

    [Theory]
    [InlineData(MeshAlgoType.MeshAlgoType_Watson)]
    [InlineData(MeshAlgoType.MeshAlgoType_Delabella)]
    public void Every_face_is_meshed_close_to_its_surface(object theAlgoValue)
    {
        var theAlgo = (MeshAlgoType)theAlgoValue; // the theory's data as object: the engine type is internal
        foreach ((string aName, Shape aShape) in Shapes())
        {
            if (theAlgo == MeshAlgoType.MeshAlgoType_Delabella && aName == "extrusion")
            {
                continue; // OCCT's Delabella factory can't mesh it (Delabella_leaves_an_extrusion_unmeshed_as_OCCT_does)
            }
            ToolParameters aParams = new() { Deflection = 0.01, Angle = 0.3, MeshAlgo = theAlgo };
            Shape aMeshed = IncrementalMesh.Meshed(aShape, aParams);
            CheckMesh($"{aName} {theAlgo}", aMeshed, 0.01);
        }
    }

    [Fact]
    public void Delabella_meshes_in_parallel_as_serially_and_every_run()
    {
        foreach ((string aName, Shape aShape) in Shapes())
        {
            ToolParameters aParams = new() { Deflection = 0.02, Angle = 0.2, MeshAlgo = MeshAlgoType.MeshAlgoType_Delabella };
            string aSerial = Dump(IncrementalMesh.Meshed(aShape, aParams));
            aParams.InParallel = true;
            for (int aRun = 0; aRun < 3; aRun++)
            {
                Assert.True(aSerial == Dump(IncrementalMesh.Meshed(aShape, aParams)), aName);
            }
        }
    }

    /// <summary>
    /// OCCT's Delabella factory meshes a surface of extrusion with the NURBS range splitter, which asks the surface for
    /// its number of V poles: GeomAdaptor_Surface throws Standard_NoSuchObject, the face algorithm's catch swallows it,
    /// and the face is left without a mesh and without a status (occt-issues.md, unit 4D).
    /// </summary>
    [Fact]
    public void Delabella_leaves_an_extrusion_unmeshed_as_OCCT_does_ReproducesOcctError()
    {
        (string _, Shape anExtrusion) = Shapes().Single(s => s.Name == "extrusion");
        ToolParameters aParams = new() { Deflection = 0.01, Angle = 0.3, MeshAlgo = MeshAlgoType.MeshAlgoType_Delabella };
        IncrementalMesh aMesh = new(anExtrusion, aParams);
        Assert.Equal(0, aMesh.GetStatusFlags());
        Assert.Null(Triangulations(aMesh.Shape()).Single());

        aParams.MeshAlgo = MeshAlgoType.MeshAlgoType_Watson;
        Assert.NotNull(Triangulations(IncrementalMesh.Meshed(anExtrusion, aParams)).Single());
    }

    [Fact]
    public void Surface_deflection_control_on_all_surfaces_refines_planes_and_cylinders()
    {
        Shape aCylinder = new ApiMakeCylinder(2, 5).Shape();
        ToolParameters aParams = new() { Deflection = 0.05, Angle = 0.5 };
        int aDefault = Triangulations(IncrementalMesh.Meshed(aCylinder, aParams)).Sum(t => t!.NodeCount);
        aParams.EnableControlSurfaceDeflectionAllSurfaces = true;
        Shape aControlled = IncrementalMesh.Meshed(aCylinder, aParams);
        CheckMesh("cylinder, all surfaces controlled", aControlled, 0.01);
        Assert.True(Triangulations(aControlled).Sum(t => t!.NodeCount) >= aDefault);
    }

    [Fact]
    public void A_larger_min_size_and_no_surface_control_give_coarser_meshes()
    {
        (string _, Shape aSphere) = Shapes().Single(s => s.Name == "sphere");
        (string _, Shape aBezier) = Shapes().Single(s => s.Name == "bezier");
        foreach (Shape aShape in new[] { aSphere, aBezier })
        {
            ToolParameters aParams = new() { Deflection = 0.002, Angle = 0.2 };
            int aDefault = Triangulations(IncrementalMesh.Meshed(aShape, aParams)).Sum(t => t!.NodeCount);

            ToolParameters aMin = aParams;
            aMin.MinSize = 0.3;
            int aWithMin = Triangulations(IncrementalMesh.Meshed(aShape, aMin)).Sum(t => t!.NodeCount);
            Assert.True(aWithMin < aDefault, $"{aWithMin} nodes with MinSize, {aDefault} without");

            ToolParameters aNoControl = aParams;
            aNoControl.ControlSurfaceDeflection = false;
            Shape aMeshed = IncrementalMesh.Meshed(aShape, aNoControl);
            Assert.True(Triangulations(aMeshed).Sum(t => t!.NodeCount) <= aDefault);
        }
    }

    [Fact]
    public void Failed_faces_are_reported_and_the_rest_meshed()
    {
        // A face whose wire crosses itself (a bow tie) fails with SelfIntersectingWire; the box beside it is meshed.
        Wire aBowTieWire = new MakePolygon(new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(1, 0, 0),
                                                                 new Point3D(0, 1, 0), true).Wire();
        BrepBuilder aFaceBuilder = new();
        Face aBowTie = new();
        aFaceBuilder.MakeFace(ref aBowTie, new PlaneSurface(new Plane3D()), global::Kerf.Engine.Foundation.Precision.Confusion());
        aFaceBuilder.Add(aBowTie, aBowTieWire);
        BrepBuilder aBuilder = new();
        Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);
        aBuilder.Add(aCompound, aBowTie);
        aBuilder.Add(aCompound, new MakeBox(new Point3D(5, 0, 0), 1, 1, 1).Shape());

        IncrementalMesh aMesh = new(aCompound, 0.01);
        Assert.True(aMesh.IsDone);
        Assert.NotEqual(0, aMesh.GetStatusFlags() & (int)StatusModel.SelfIntersectingWire);
        List<Triangulation?> aTris = Triangulations(aMesh.Shape());
        Assert.Equal(7, aTris.Count);
        Assert.Equal(6, aTris.Count(t => t is not null));
    }
}
