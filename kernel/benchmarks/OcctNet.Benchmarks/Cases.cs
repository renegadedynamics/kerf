// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Reflection;
using Occt;
#if HAS_NATIVE
using N = Build123d.Occt;
#endif

namespace OcctNet.Benchmarks;

/// <summary>A shape to mesh: its BRep text without meshes (F1) and the viewer's deflection for it.</summary>
public sealed record Case(string Name, string F1, double Deflection, int Faces);

/// <summary>
/// The benchmark's shapes (phase4-plan 2.5): data/occ files of several sizes, and shapes built by native OCCT (a filleted
/// box cut by cylinders, a fused part, a sphere, a torus), all written as BRep text without meshes so both sides read
/// the same input. The deflection is the viewer's: 0.1% of the bounding box's diagonal (build123d.NET's
/// MeshTolerances.For), with an angle of 0.2 rad.
/// </summary>
public static class Cases
{
    public const double Angle = 0.2;

    public static readonly string[] DataOccFiles =
    [
        "Pump_Nut.brep", "bottle.brep", "CrankArm.brep", "hammer.brep", "MODERN_Chair_1.brep", "Motor-c.brep", "Axis_of_bearing.brep",
    ];

    private static string Metadata(string theKey)
        => typeof(Cases).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == theKey).Value!;

    public static string DataOcc => Path.Combine(Metadata("OcctSourceDir"), "data", "occ");

    private static readonly Lazy<string> TheDirectory = new(() => Directory.CreateTempSubdirectory("occtnet-bench-").FullName);

    private static int theFileNumber;

    private static string NewFile(string theName)
        => Path.Combine(TheDirectory.Value, $"{Interlocked.Increment(ref theFileNumber)}-{theName.Replace(' ', '_')}.brep");

    public static TopoDS_Shape PortRead(string theFile)
    {
        if (!BRepTools.Read(out TopoDS_Shape aShape, theFile))
        {
            throw new InvalidOperationException("cannot read " + theFile);
        }
        return aShape;
    }

    /// <summary>The viewer's deflection: 0.1% of the bounding box's diagonal.</summary>
    public static double ViewerDeflection(in TopoDS_Shape theShape)
    {
        var aBox = new Bnd_Box();
        BRepBndLib.Add(theShape, ref aBox, true);
        double aDiagonal = aBox.IsVoid() ? 0.0 : Math.Sqrt(aBox.SquareExtent());
        return aDiagonal > 1e-6 ? aDiagonal * 0.1 / 100 : 1e-3;
    }

    public static List<Case> All()
    {
        var aCases = new List<Case>();
        foreach (string aName in DataOccFiles)
        {
            string aPath = Path.Combine(DataOcc, aName);
            if (File.Exists(aPath))
            {
                aCases.Add(Make(aName, F1OfFile(aPath)));
            }
        }
#if HAS_NATIVE
        foreach ((string aName, N.TopoDS_Shape aShape) in NativeShapes())
        {
            string aFile = NewFile(aName);
            N.BRepTools.Write(aShape, aFile, false, false, N.TopTools_FormatVersion.TopTools_FormatVersion_CURRENT);
            aShape.Dispose();
            aCases.Add(Make(aName, aFile));
        }
#endif
        return aCases;
    }

    private static Case Make(string theName, string theF1)
    {
        TopoDS_Shape aShape = PortRead(theF1);
        return new Case(theName, theF1, ViewerDeflection(aShape), MeshExtraction.Faces(aShape).Count);
    }

    /// <summary>The file without its meshes, written again (by native OCCT where it is available, as the oracle's F1).</summary>
    private static string F1OfFile(string thePath)
    {
        string aFile = NewFile(Path.GetFileNameWithoutExtension(thePath));
#if HAS_NATIVE
        var aShape = new N.TopoDS_Shape();
        using (var aBuilder = new N.BRep_Builder())
        {
            N.BRepTools.Read(aShape, thePath, aBuilder);
        }
        N.BRepTools.Clean(aShape);
        N.BRepTools.Write(aShape, aFile, false, false, N.TopTools_FormatVersion.TopTools_FormatVersion_CURRENT);
        aShape.Dispose();
#else
        BRepTools.Write(BRepTools.Cleaned(PortRead(thePath)), aFile, false, false, TopTools_FormatVersion.TopTools_FormatVersion_CURRENT);
#endif
        return aFile;
    }

#if HAS_NATIVE
    private static N.gp_Pnt P(double x, double y, double z) => new(x, y, z);

    private static N.gp_Dir D(double x, double y, double z) => new(x, y, z);

    /// <summary>Shapes built natively: what Bento's users make (fillets, Booleans) and the analytic primitives.</summary>
    private static IEnumerable<(string, N.TopoDS_Shape)> NativeShapes()
    {
        using var anAx2 = new N.gp_Ax2(P(0, 0, 0), D(0, 0, 1));
        // A filleted block cut by three holes: a typical part
        using (var aBox = new N.BRepPrimAPI_MakeBox(anAx2, 60, 40, 20))
        using (var aFillet = new N.BRepFilletAPI_MakeFillet(aBox.Shape()))
        {
            using var anExp = new N.TopExp_Explorer(aBox.Shape(), N.TopAbs_ShapeEnum.TopAbs_EDGE);
            for (; anExp.More(); anExp.Next())
            {
                using N.TopoDS_Edge anEdge = N.TopoDS.Edge(anExp.Current());
                aFillet.Add(3.0, anEdge);
            }
            N.TopoDS_Shape aPart = aFillet.Shape();
            foreach ((double x, double y, double r) in new[] { (15.0, 20.0, 6.0), (35.0, 12.0, 4.0), (47.0, 28.0, 5.0) })
            {
                using var anAxis = new N.gp_Ax2(P(x, y, -5), D(0, 0, 1));
                using var aHole = new N.BRepPrimAPI_MakeCylinder(anAxis, r, 30);
                using var aCut = new N.BRepAlgoAPI_Cut(aPart, aHole.Shape());
                aPart = aCut.Shape();
            }
            yield return ("filleted block with holes", aPart);
        }
        // A fuse of a box, a cylinder and a sphere
        using (var aBox = new N.BRepPrimAPI_MakeBox(anAx2, 30, 30, 10))
        using (var aCylAx = new N.gp_Ax2(P(15, 15, 0), D(0, 0, 1)))
        using (var aCyl = new N.BRepPrimAPI_MakeCylinder(aCylAx, 8, 30))
        using (var aSphere = new N.BRepPrimAPI_MakeSphere(P(15, 15, 30), 10))
        using (var aFuse1 = new N.BRepAlgoAPI_Fuse(aBox.Shape(), aCyl.Shape()))
        using (var aFuse2 = new N.BRepAlgoAPI_Fuse(aFuse1.Shape(), aSphere.Shape()))
        {
            yield return ("fuse of box, cylinder and sphere", aFuse2.Shape());
        }
        using (var aSphere = new N.BRepPrimAPI_MakeSphere(anAx2, 25))
        {
            yield return ("sphere", aSphere.Shape());
        }
        using (var aTorus = new N.BRepPrimAPI_MakeTorus(anAx2, 30, 8))
        {
            yield return ("torus", aTorus.Shape());
        }
    }

    /// <summary>build123d.NET's <c>TopoUtil.Entities</c>: distinct sub-shapes in exploration order, the last orientation kept.</summary>
    public static List<N.TopoDS_Shape> NativeEntities(N.TopoDS_Shape theShape, N.TopAbs_ShapeEnum theType)
    {
        var anIndex = new Dictionary<ulong, int>();
        var aFound = new List<N.TopoDS_Shape>();
        using var anExp = new N.TopExp_Explorer(theShape, theType);
        for (; anExp.More(); anExp.Next())
        {
            N.TopoDS_Shape anItem = anExp.Current();
            ulong aHash = anItem.StdHash();
            if (anIndex.TryGetValue(aHash, out int i))
            {
                aFound[i].Dispose();
                aFound[i] = anItem;
            }
            else
            {
                anIndex[aHash] = aFound.Count;
                aFound.Add(anItem);
            }
        }
        return aFound;
    }

    public static N.TopoDS_Shape NativeRead(string theFile)
    {
        var aShape = new N.TopoDS_Shape();
        using var aBuilder = new N.BRep_Builder();
        N.BRepTools.Read(aShape, theFile, aBuilder);
        return aShape;
    }

    /// <summary>build123d.NET's ShapeMesh.Of: a private copy, meshed, then the bulk extraction.</summary>
    public static N.MeshBuffers NativeMeshAndExtract(N.TopoDS_Shape theShape, double theDeflection, bool theParallel)
    {
        using var aCopier = new N.BRepBuilderAPI_Copy(theShape, false, false);
        using N.TopoDS_Shape aCopy = aCopier.Shape();
        new N.BRepMesh_IncrementalMesh(aCopy, theDeflection, false, Angle, theParallel).Dispose();
        return NativeExtract(aCopy, theDeflection);
    }

    public static N.MeshBuffers NativeExtract(N.TopoDS_Shape theMeshed, double theDeflection)
    {
        List<N.TopoDS_Shape> aFaces = NativeEntities(theMeshed, N.TopAbs_ShapeEnum.TopAbs_FACE);
        List<N.TopoDS_Shape> anEdges = NativeEntities(theMeshed, N.TopAbs_ShapeEnum.TopAbs_EDGE);
        try
        {
            return N.Extras.ExtractMesh(theMeshed, aFaces, anEdges, theDeflection, Angle);
        }
        finally
        {
            foreach (N.TopoDS_Shape aShape in aFaces.Concat(anEdges))
            {
                aShape.Dispose();
            }
        }
    }
#endif

    /// <summary>What Bento's viewer does with the port: mesh (a copy of the frozen input) and extract.</summary>
    public static MeshBuffers PortMeshAndExtract(in TopoDS_Shape theShape, double theDeflection, bool theParallel)
    {
        TopoDS_Shape aMeshed = new BRepMesh_IncrementalMesh(theShape, theDeflection, false, Angle, theParallel).Shape();
        return MeshExtraction.Extract(aMeshed, theDeflection, Angle);
    }
}
