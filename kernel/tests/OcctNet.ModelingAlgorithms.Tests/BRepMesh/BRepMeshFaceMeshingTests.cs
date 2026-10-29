// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.IMeshTools_MeshAlgoType;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Unit 4D's face meshing through BRepMesh_IncrementalMesh, for what native OCCT's binding can't set (the Delabella
/// algorithm and the parameters beyond the five arguments): every face meshed, valid triangles, areas close to the
/// exact ones, parallel equal to serial. Native Watson meshes are compared bit for bit by the oracle.
/// </summary>
public class BRepMeshFaceMeshingTests
{
    private static IEnumerable<(string Name, TopoDS_Shape Shape)> Shapes()
    {
        gp_Ax2 anAx2 = new(new gp_Pnt(1, 2, 3), new gp_Dir(0.2, 0.3, 1.0));
        yield return ("box", new BRepPrimAPI_MakeBox(anAx2, 3, 4, 5).Shape());
        yield return ("cylinder", new BRepPrimAPI_MakeCylinder(anAx2, 2, 5).Shape());
        yield return ("cone", new BRepPrimAPI_MakeCone(anAx2, 2, 0.5, 4).Shape());
        yield return ("sphere", new BRepPrimAPI_MakeSphere(anAx2, 3).Shape());
        yield return ("torus", new BRepPrimAPI_MakeTorus(anAx2, 5, 1.5).Shape());

        // A surface of revolution and an extrusion of a B-spline (the NURBS, BoundaryParams and Extrusion splitters).
        NCollection_Array1<gp_Pnt> aPoles = new(1, 5);
        for (int i = 1; i <= 5; i++)
        {
            aPoles[i] = new gp_Pnt(2 + 0.4 * Math.Sin(i), 0, i);
        }
        Geom_BezierCurve aCurve = new(aPoles);
        TopoDS_Edge anEdge = new BRepBuilderAPI_MakeEdge(aCurve).Edge();
        yield return ("revolution", new BRepPrimAPI_MakeRevol(anEdge, new gp_Ax1(gp.Origin(), gp.DZ()), 4.0).Shape());
        yield return ("extrusion", new BRepPrimAPI_MakePrism(anEdge, new gp_Vec(3, 1, 0)).Shape());
        NCollection_Array2<gp_Pnt> aNet = new(1, 3, 1, 3);
        for (int i = 1; i <= 3; i++)
        {
            for (int j = 1; j <= 3; j++)
            {
                aNet[i, j] = new gp_Pnt(i, j, (i == 2 && j == 2) ? 1.5 : 0.0);
            }
        }
        Geom_BezierSurface aSurface = new(aNet);
        yield return ("bezier", new BRepBuilderAPI_MakeFace(aSurface, Precision.Confusion()).Face());
    }

    private static List<Poly_Triangulation?> Triangulations(in TopoDS_Shape theShape)
    {
        List<Poly_Triangulation?> aList = [];
        for (TopExp_Explorer anExp = new(theShape, TopAbs_ShapeEnum.TopAbs_FACE); anExp.More(); anExp.Next())
        {
            aList.Add(BRep_Tool.Triangulation(TopoDS.Face(anExp.Current()), out _));
        }
        return aList;
    }

    // Every face meshed with valid triangles; the mesh's area within the given share of the exact area.
    private static void CheckMesh(string theWhat, in TopoDS_Shape theShape, double theAreaTolerance)
    {
        double aMeshArea = 0.0;
        foreach (Poly_Triangulation? aT in Triangulations(theShape))
        {
            Assert.True(aT is not null, theWhat + ": a face without mesh");
            Assert.True(aT.NbTriangles() > 0, theWhat);
            Assert.True(aT.HasUVNodes(), theWhat);
            for (int i = 1; i <= aT.NbTriangles(); i++)
            {
                aT.Triangle(i).Get(out int a, out int b, out int c);
                Assert.True(a >= 1 && b >= 1 && c >= 1 && a <= aT.NbNodes() && b <= aT.NbNodes() && c <= aT.NbNodes(), theWhat);
                Assert.True(a != b && b != c && a != c, theWhat + ": degenerate triangle");
                gp_XYZ p = aT.Node(a).XYZ(), q = aT.Node(b).XYZ(), r = aT.Node(c).XYZ();
                aMeshArea += 0.5 * (q - p).Crossed(r - p).Modulus();
            }
        }
        double anArea = BRepGProp.SurfaceProperties(theShape).Mass();
        Assert.True(Math.Abs(aMeshArea - anArea) <= theAreaTolerance * anArea,
                    $"{theWhat}: mesh area {aMeshArea}, exact {anArea}");
    }

    private static string Dump(in TopoDS_Shape theShape)
    {
        System.Text.StringBuilder aBuilder = new();
        foreach (Poly_Triangulation? aT in Triangulations(theShape))
        {
            if (aT is null)
            {
                aBuilder.Append("null\n");
                continue;
            }
            for (int i = 1; i <= aT.NbNodes(); i++)
            {
                gp_Pnt p = aT.Node(i);
                gp_Pnt2d uv = aT.UVNode(i);
                aBuilder.Append(BitConverter.DoubleToInt64Bits(p.X())).Append(BitConverter.DoubleToInt64Bits(p.Y()))
                        .Append(BitConverter.DoubleToInt64Bits(p.Z())).Append(BitConverter.DoubleToInt64Bits(uv.X()))
                        .Append(BitConverter.DoubleToInt64Bits(uv.Y())).Append(';');
            }
            for (int i = 1; i <= aT.NbTriangles(); i++)
            {
                aT.Triangle(i).Get(out int a, out int b, out int c);
                aBuilder.Append(a).Append(',').Append(b).Append(',').Append(c).Append(';');
            }
            aBuilder.Append(BitConverter.DoubleToInt64Bits(aT.Deflection())).Append('\n');
        }
        return aBuilder.ToString();
    }

    [Theory]
    [InlineData(IMeshTools_MeshAlgoType_Watson)]
    [InlineData(IMeshTools_MeshAlgoType_Delabella)]
    public void Every_face_is_meshed_close_to_its_surface(IMeshTools_MeshAlgoType theAlgo)
    {
        foreach ((string aName, TopoDS_Shape aShape) in Shapes())
        {
            if (theAlgo == IMeshTools_MeshAlgoType_Delabella && aName == "extrusion")
            {
                continue; // OCCT's Delabella factory can't mesh it (Delabella_leaves_an_extrusion_unmeshed_as_OCCT_does)
            }
            IMeshTools_Parameters aParams = new() { Deflection = 0.01, Angle = 0.3, MeshAlgo = theAlgo };
            TopoDS_Shape aMeshed = BRepMesh_IncrementalMesh.Meshed(aShape, aParams);
            CheckMesh($"{aName} {theAlgo}", aMeshed, 0.01);
        }
    }

    [Fact]
    public void Delabella_meshes_in_parallel_as_serially_and_every_run()
    {
        foreach ((string aName, TopoDS_Shape aShape) in Shapes())
        {
            IMeshTools_Parameters aParams = new() { Deflection = 0.02, Angle = 0.2, MeshAlgo = IMeshTools_MeshAlgoType_Delabella };
            string aSerial = Dump(BRepMesh_IncrementalMesh.Meshed(aShape, aParams));
            aParams.InParallel = true;
            for (int aRun = 0; aRun < 3; aRun++)
            {
                Assert.True(aSerial == Dump(BRepMesh_IncrementalMesh.Meshed(aShape, aParams)), aName);
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
        (string _, TopoDS_Shape anExtrusion) = Shapes().Single(s => s.Name == "extrusion");
        IMeshTools_Parameters aParams = new() { Deflection = 0.01, Angle = 0.3, MeshAlgo = IMeshTools_MeshAlgoType_Delabella };
        BRepMesh_IncrementalMesh aMesh = new(anExtrusion, aParams);
        Assert.Equal(0, aMesh.GetStatusFlags());
        Assert.Null(Triangulations(aMesh.Shape()).Single());

        aParams.MeshAlgo = IMeshTools_MeshAlgoType_Watson;
        Assert.NotNull(Triangulations(BRepMesh_IncrementalMesh.Meshed(anExtrusion, aParams)).Single());
    }

    [Fact]
    public void Surface_deflection_control_on_all_surfaces_refines_planes_and_cylinders()
    {
        TopoDS_Shape aCylinder = new BRepPrimAPI_MakeCylinder(2, 5).Shape();
        IMeshTools_Parameters aParams = new() { Deflection = 0.05, Angle = 0.5 };
        int aDefault = Triangulations(BRepMesh_IncrementalMesh.Meshed(aCylinder, aParams)).Sum(t => t!.NbNodes());
        aParams.EnableControlSurfaceDeflectionAllSurfaces = true;
        TopoDS_Shape aControlled = BRepMesh_IncrementalMesh.Meshed(aCylinder, aParams);
        CheckMesh("cylinder, all surfaces controlled", aControlled, 0.01);
        Assert.True(Triangulations(aControlled).Sum(t => t!.NbNodes()) >= aDefault);
    }

    [Fact]
    public void A_larger_min_size_and_no_surface_control_give_coarser_meshes()
    {
        (string _, TopoDS_Shape aSphere) = Shapes().Single(s => s.Name == "sphere");
        (string _, TopoDS_Shape aBezier) = Shapes().Single(s => s.Name == "bezier");
        foreach (TopoDS_Shape aShape in new[] { aSphere, aBezier })
        {
            IMeshTools_Parameters aParams = new() { Deflection = 0.002, Angle = 0.2 };
            int aDefault = Triangulations(BRepMesh_IncrementalMesh.Meshed(aShape, aParams)).Sum(t => t!.NbNodes());

            IMeshTools_Parameters aMin = aParams;
            aMin.MinSize = 0.3;
            int aWithMin = Triangulations(BRepMesh_IncrementalMesh.Meshed(aShape, aMin)).Sum(t => t!.NbNodes());
            Assert.True(aWithMin < aDefault, $"{aWithMin} nodes with MinSize, {aDefault} without");

            IMeshTools_Parameters aNoControl = aParams;
            aNoControl.ControlSurfaceDeflection = false;
            TopoDS_Shape aMeshed = BRepMesh_IncrementalMesh.Meshed(aShape, aNoControl);
            Assert.True(Triangulations(aMeshed).Sum(t => t!.NbNodes()) <= aDefault);
        }
    }

    [Fact]
    public void Failed_faces_are_reported_and_the_rest_meshed()
    {
        // A face whose wire crosses itself (a bow tie) fails with SelfIntersectingWire; the box beside it is meshed.
        TopoDS_Wire aBowTieWire = new BRepBuilderAPI_MakePolygon(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 1, 0), new gp_Pnt(1, 0, 0),
                                                                 new gp_Pnt(0, 1, 0), true).Wire();
        BRep_Builder aFaceBuilder = new();
        TopoDS_Face aBowTie = new();
        aFaceBuilder.MakeFace(ref aBowTie, new Geom_Plane(new gp_Pln()), Precision.Confusion());
        aFaceBuilder.Add(aBowTie, aBowTieWire);
        BRep_Builder aBuilder = new();
        TopoDS_Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);
        aBuilder.Add(aCompound, aBowTie);
        aBuilder.Add(aCompound, new BRepPrimAPI_MakeBox(new gp_Pnt(5, 0, 0), 1, 1, 1).Shape());

        BRepMesh_IncrementalMesh aMesh = new(aCompound, 0.01);
        Assert.True(aMesh.IsDone());
        Assert.NotEqual(0, aMesh.GetStatusFlags() & (int)IMeshData_Status.IMeshData_SelfIntersectingWire);
        List<Poly_Triangulation?> aTris = Triangulations(aMesh.Shape());
        Assert.Equal(7, aTris.Count);
        Assert.Equal(6, aTris.Count(t => t is not null));
    }
}
