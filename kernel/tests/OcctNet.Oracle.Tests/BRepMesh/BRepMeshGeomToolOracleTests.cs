#if HAS_ORACLE
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Xunit.Abstractions;
using N = Build123d.Occt;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// BRepMesh_GeomTool's curve discretisation and BRepMesh_Deflection's edge deflection against native OCCT, indirectly:
/// native BRepMesh_IncrementalMesh meshes a free edge into a 3D polygon whose parameters are those of
/// GCPnts_TangentialDeflection run with half the edge's deflections (BRepMesh_CurveTessellator), and whose deflection
/// is the edge's (BRepMesh_Deflection::ComputeDeflection). The port computes the same with BRepMesh_Deflection and
/// BRepMesh_GeomTool on the edge it reads.
/// </summary>
/// <remarks>
/// Decision 494's scheme: native writes the unmeshed edge (F1), reads F1, meshes it and writes F2; the port reads F1
/// and F2, discretises F1's edge and compares with F2's polygon: the same number of points everywhere. BRep text writes
/// 15 significant digits, so parameters and nodes agree to about 5e-15 (the share that survives the text bit for bit
/// is reported); unit 4D compares the meshes bit for bit through the native binding.
/// </remarks>
public sealed class BRepMeshGeomToolOracleTests(ITestOutputHelper output) : IDisposable
{
    private readonly string myDirectory = Directory.CreateTempSubdirectory("occtnet-brepmesh-").FullName;
    private int myFileNumber;

    public void Dispose()
    {
        try
        {
            Directory.Delete(myDirectory, true);
        }
        catch (IOException)
        {
        }
    }

    private string NewFile() => Path.Combine(myDirectory, $"e{myFileNumber++}.brep");

    private static TopoDS_Shape PortRead(string theFile)
    {
        Assert.True(BRepTools.Read(out TopoDS_Shape aShape, theFile), theFile);
        return aShape;
    }

    private static N.TopoDS_Shape NativeRead(string theFile)
    {
        var aShape = new N.TopoDS_Shape();
        using var aBuilder = new N.BRep_Builder();
        Assert.True(N.BRepTools.Read(aShape, theFile, aBuilder), theFile);
        return aShape;
    }

    /// <summary>A random free edge on a circle or an ellipse arc, built natively.</summary>
    private static N.TopoDS_Shape NativeEdge(Oracle theO, out bool theIsConic)
    {
        using var aNP = new N.gp_Pnt(theO.Real(-10, 10), theO.Real(-10, 10), theO.Real(-10, 10));
        gp_Dir aZ = theO.Dir();
        gp_Dir aRef = theO.Dir();
        gp_Dir aX = aZ.Crossed(aRef.IsParallel(aZ, 0.1) ? (aZ.IsParallel(new gp_Dir(1, 0, 0), 0.1) ? new gp_Dir(0, 1, 0) : new gp_Dir(1, 0, 0)) : aRef);
        using var aNZ = new N.gp_Dir(aZ.X(), aZ.Y(), aZ.Z());
        using var aNX = new N.gp_Dir(aX.X(), aX.Y(), aX.Z());
        using var aNAx2 = new N.gp_Ax2(aNP, aNZ, aNX);
        double a = theO.Real(-3, 3);
        double b = a + theO.Real(0.2, 2 * Math.PI - 0.1);
        theIsConic = true;
        switch (theO.Int(0, 2))
        {
            case 0:
            {
                N.Geom_Circle aC = new(aNAx2, theO.Real(0.5, 20));
                return new N.BRepBuilderAPI_MakeEdge(aC, a, b).Edge();
            }
            case 1:
            {
                double aMajor = theO.Real(1, 20);
                N.Geom_Ellipse anE = new(aNAx2, aMajor, aMajor * theO.Real(0.1, 1.0));
                return new N.BRepBuilderAPI_MakeEdge(anE, a, b).Edge();
            }
            default:
            {
                using var aNQ = new N.gp_Pnt(theO.Real(-10, 10), theO.Real(-10, 10), theO.Real(-10, 10));
                theIsConic = false;
                return new N.BRepBuilderAPI_MakeEdge(aNP, aNQ).Edge();
            }
        }
    }

    [Fact]
    public void Free_edge_polygons_match_GeomTool_and_Deflection()
    {
        var anOracle = new Oracle(output, 40101);
        int aPoints = 0;
        for (int i = 0; i < 300; i++)
        {
            double aDeflection = anOracle.Real(0.001, 0.5);
            double anAngle = anOracle.Real(0.1, 0.5);
            N.TopoDS_Shape aNative = NativeEdge(anOracle, out bool isConic);

            string aF1 = NewFile();
            Assert.True(N.BRepTools.Write(aNative, aF1));
            N.TopoDS_Shape aNativeF1 = NativeRead(aF1);
            using (new N.BRepMesh_IncrementalMesh(aNativeF1, aDeflection, false, anAngle, false))
            {
            }
            string aF2 = NewFile();
            Assert.True(N.BRepTools.Write(aNativeF1, aF2));

            TopoDS_Edge anEdge = TopoDS.Edge(PortRead(aF1));
            TopoDS_Edge aMeshed = TopoDS.Edge(PortRead(aF2));
            Poly_Polygon3D aPoly = BRep_Tool.Polygon3D(aMeshed, out _)!;
            Assert.NotNull(aPoly);

            // The edge's deflection (BRepMesh_Deflection), as the mesher's parameters are completed
            // (BRepMesh_IncrementalMesh::initParameters).
            var aParams = new IMeshTools_Parameters { Deflection = aDeflection, Angle = anAngle };
            aParams.MinSize = Math.Max(IMeshTools_Parameters.RelMinSize() * aDeflection, Precision.Confusion());
            var aDEdge = new BRepMeshData_Edge(anEdge);
            BRepMesh_Deflection.ComputeDeflection(aDEdge, 0.0, aParams);
            anOracle.Same(aPoly.Deflection(), aDEdge.GetDeflection(), $"edge {i} deflection");

            // The tessellation (BRepMesh_CurveTessellator's GCPnts_TangentialDeflection) through BRepMesh_GeomTool.
            var aCurve = new BRepAdaptor_Curve(anEdge);
            double aLinDef = Math.Max(0.5 * aDEdge.GetDeflection(), Precision.Confusion());
            double anAngDef = Math.Max(0.5 * aDEdge.GetAngularDeflection(), Precision.Angular());
            var aTool = new BRepMesh_GeomTool(aCurve, aCurve.FirstParameter(), aCurve.LastParameter(), aLinDef, anAngDef,
                                              isConic ? 4 : 2, aParams.MinSize);
            anOracle.Same(aPoly.NbNodes(), aTool.NbPoints(), $"edge {i} points");
            NCollection_Array1<double> aParameters = aPoly.Parameters()!;
            NCollection_Array1<gp_Pnt> aNodes = aPoly.Nodes();
            for (int k = 1; k <= aTool.NbPoints(); k++)
            {
                aTool.Value(k, 0.0, out double aParam, out gp_Pnt aPnt, out _);
                anOracle.Same(aParameters[k], aParam, $"edge {i} parameter {k}");
                if (k > 1 && k < aTool.NbPoints()) // the ends are the vertices' points
                {
                    anOracle.Same(aNodes[k].XYZ(), aPnt.XYZ(), $"edge {i} node {k}");
                }
                aPoints++;
            }
        }

        output.WriteLine($"300 edges, {aPoints} points");
        anOracle.Report("BRepMesh_GeomTool on free edges against native polygons");
    }
}
#endif
