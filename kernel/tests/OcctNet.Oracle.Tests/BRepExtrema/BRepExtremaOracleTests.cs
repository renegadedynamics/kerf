#if HAS_ORACLE
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Text;
using Xunit.Abstractions;
using N = Build123d.Occt;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// BRepExtrema_DistShapeShape against native OCCT: IsDone, NbSolution, Value, InnerSolution, and for each solution
/// the points, the support types, the supports (by their place in the TopExp maps of the inputs, with their
/// orientation, or for a support built by the algorithm, such as the trimmed copy of an infinite edge, its BRep
/// text) and the parameters on edges and faces. Solutions are compared in order: OCCT splits the work by the number
/// of threads of its default pool, and the port takes the same number (the machine's logical processors), so the
/// order is OCCT's in both modes.
/// </summary>
/// <remarks>
/// Inputs: pairs of the random scenes of <see cref="ClassificationScenes"/> (built identically on both sides), their
/// vertices, edges and faces against each other, points inside solids (inner solutions), infinite lines and planes
/// (trimmed by the algorithm); and random native shapes (primitives, sweeps, Booleans, fillets, lofts) read by both
/// sides from native OCCT's BRep file. Each computation runs with a random deflection, in single- or multi-threaded
/// mode, with a random (obsolete) flag and algorithm.
/// </remarks>
public sealed class BRepExtremaOracleTests(ITestOutputHelper output) : IDisposable
{
    private readonly string myDirectory = Directory.CreateTempSubdirectory("occtnet-brepextrema-").FullName;
    private int myFileNumber;
    private int myNbComputations;
    private int myNbSolutions;
    private int myNbInner;
    private int myNbBuiltSupports;
    private int myNbRacingInner;
    private readonly int[] mySupportKinds = new int[3];

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

    private string NewFile() => Path.Combine(myDirectory, $"s{myFileNumber++}.brep");

    private string NativeText(N.TopoDS_Shape theShape)
    {
        string aFile = NewFile();
        Assert.True(N.BRepTools.Write(theShape, aFile));
        return File.ReadAllText(aFile, Encoding.Latin1);
    }

    private string PortText(in TopoDS_Shape theShape)
    {
        string aFile = NewFile();
        Assert.True(BRepTools.Write(theShape, aFile));
        return File.ReadAllText(aFile, Encoding.Latin1);
    }

    /// <summary>The vertex, edge and face maps of an input, on both sides.</summary>
    private sealed class Maps
    {
        public readonly N.TopTools_IndexedMapOfShape[] NM = new N.TopTools_IndexedMapOfShape[3];
        public readonly NCollection_IndexedMap<TopoDS_Shape>[] PM = new NCollection_IndexedMap<TopoDS_Shape>[3];

        public Maps(N.TopoDS_Shape theN, in TopoDS_Shape theP)
        {
            TopAbs_ShapeEnum[] aTypes = [TopAbs_VERTEX, TopAbs_EDGE, TopAbs_FACE];
            for (int i = 0; i < 3; i++)
            {
                NM[i] = new N.TopTools_IndexedMapOfShape();
                N.TopExp.MapShapes(theN, TwinShapes.ToNative(aTypes[i]), NM[i]);
                PM[i] = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
                TopExp.MapShapes(theP, aTypes[i], PM[i]);
                Assert.Equal(NM[i].Extent(), PM[i].Extent());
            }
        }
    }

    private void CompareSupport(Oracle theOracle, N.TopoDS_Shape theN, in TopoDS_Shape theP, Maps theMaps1, Maps theMaps2, string theWhat)
    {
        theOracle.Same((int)theN.ShapeType(), (int)theP.ShapeType(), theWhat + " type");
        int aKind = theP.ShapeType() switch
        {
            TopAbs_VERTEX => 0,
            TopAbs_EDGE => 1,
            _ => 2,
        };

        // the support may come from either input (an inner solution's vertex is the other shape's)
        int aNIndex = theMaps1.NM[aKind].FindIndex(theN);
        int aPIndex = theMaps1.PM[aKind].FindIndex(theP);
        int aNIndex2 = theMaps2.NM[aKind].FindIndex(theN);
        int aPIndex2 = theMaps2.PM[aKind].FindIndex(theP);
        theOracle.Same(aNIndex, aPIndex, theWhat + " index in shape 1");
        theOracle.Same(aNIndex2, aPIndex2, theWhat + " index in shape 2");
        theOracle.Same((int)theN.Orientation(), (int)theP.Orientation(), theWhat + " orientation");
        if (aPIndex == 0 && aPIndex2 == 0)
        {
            // built by the algorithm (the trimmed copy of an infinite edge or face): compare it as a file
            myNbBuiltSupports++;
            Assert.True(theP.TShape()!.IsFrozen(), theWhat + ": a support built by the algorithm is frozen");
            Assert.True(NativeText(theN) == PortText(theP), theWhat + ": the built support differs from native OCCT's");
        }
    }

    /// <summary>Runs both sides on the pair with the same settings and compares everything.</summary>
    private void Compare(Oracle theOracle, Twin<N.TopoDS_Shape, TopoDS_Shape> theS1, Twin<N.TopoDS_Shape, TopoDS_Shape> theS2, string theWhat)
    {
        double aDeflection = theOracle.Int(0, 3) switch
        {
            0 => Precision.Confusion(),
            1 => 1e-5,
            2 => 1e-3,
            _ => theOracle.Real(1e-7, 1e-1),
        };
        bool aMulti = theOracle.Bool();
        int aFlag = theOracle.Int(0, 2);
        int anAlgo = theOracle.Int(0, 1);
        string aWhat = $"{theWhat} (deflection {aDeflection:R}, {(aMulti ? "multi" : "single")}-threaded)";

        using var aN = new N.BRepExtrema_DistShapeShape();
        aN.SetDeflection(aDeflection);
        aN.SetMultiThread(aMulti);
        aN.SetFlag((N.Extrema_ExtFlag)aFlag);
        aN.SetAlgo((N.Extrema_ExtAlgo)anAlgo);
        aN.LoadS1(theS1.N);
        aN.LoadS2(theS2.N);
        bool aNDone = aN.Perform();

        var aP = new BRepExtrema_DistShapeShape();
        aP.SetDeflection(aDeflection);
        aP.SetMultiThread(aMulti);
        aP.SetFlag((Extrema_ExtFlag)aFlag);
        aP.SetAlgo((Extrema_ExtAlgo)anAlgo);
        aP.LoadS1(theS1.P);
        aP.LoadS2(theS2.P);
        bool aPDone = aP.Perform();

        myNbComputations++;
        theOracle.Same(aNDone, aPDone, aWhat + " Perform");
        theOracle.Same(aN.IsDone(), aP.IsDone(), aWhat + " IsDone");
        theOracle.Same(aN.InnerSolution(), aP.InnerSolution(), aWhat + " InnerSolution");
        if (aMulti && aP.InnerSolution())
        {
            // OCCT's threads race to an inner point: which vertices (and how many) are found depends on the
            // scheduling, on both sides. Each solution must be a vertex of one input, at distance 0.
            myNbInner++;
            myNbRacingInner++;
            theOracle.Same(aN.Value(), aP.Value(), aWhat + " Value");
            var aMapsA = new Maps(theS1.N, theS1.P);
            var aMapsB = new Maps(theS2.N, theS2.P);
            for (int i = 1; i <= aP.NbSolution(); i++)
            {
                TopoDS_Shape aV = aP.SupportOnShape1(i);
                Assert.True(aP.SupportOnShape2(i).IsSame(aV), aWhat);
                Assert.True(aMapsA.PM[0].Contains(aV) || aMapsB.PM[0].Contains(aV), aWhat);
                Assert.Equal(BRep_Tool.Pnt(TopoDS.Vertex(aV)), aP.PointOnShape1(i));
            }
            return;
        }

        theOracle.Same(aN.NbSolution(), aP.NbSolution(), aWhat + " NbSolution");
        if (!aP.IsDone())
        {
            return;
        }

        if (aP.InnerSolution())
        {
            myNbInner++;
        }

        theOracle.Same(aN.Value(), aP.Value(), aWhat + " Value");
        if (Environment.GetEnvironmentVariable("OCCTNET_DEBUG") == "1")
        {
            output.WriteLine($"{aWhat}: value {aN.Value():R} / {aP.Value():R}");
            for (int i = 1; i <= aP.NbSolution(); i++)
            {
                using N.gp_Pnt a1 = aN.PointOnShape1(i);
                using N.gp_Pnt a2 = aN.PointOnShape2(i);
                output.WriteLine($"  N {i}: {a1.X():R} {a1.Y():R} {a1.Z():R} | {a2.X():R} {a2.Y():R} {a2.Z():R} {aN.SupportTypeShape1(i)} {aN.SupportTypeShape2(i)}");
                output.WriteLine($"  P {i}: {aP.PointOnShape1(i)} | {aP.PointOnShape2(i)} {aP.SupportTypeShape1(i)} {aP.SupportTypeShape2(i)}");
            }
        }
        var aMaps1 = new Maps(theS1.N, theS1.P);
        var aMaps2 = new Maps(theS2.N, theS2.P);
        for (int i = 1; i <= aP.NbSolution(); i++)
        {
            myNbSolutions++;
            string aSol = $"{aWhat} solution {i}";
            using (N.gp_Pnt aNP = aN.PointOnShape1(i))
            {
                theOracle.Same(aNP, aP.PointOnShape1(i), aSol + " PointOnShape1");
            }
            using (N.gp_Pnt aNP = aN.PointOnShape2(i))
            {
                theOracle.Same(aNP, aP.PointOnShape2(i), aSol + " PointOnShape2");
            }

            for (int aSide = 1; aSide <= 2; aSide++)
            {
                BRepExtrema_SupportType aType = aSide == 1 ? aP.SupportTypeShape1(i) : aP.SupportTypeShape2(i);
                N.BRepExtrema_SupportType aNType = aSide == 1 ? aN.SupportTypeShape1(i) : aN.SupportTypeShape2(i);
                theOracle.Same((int)aNType, (int)aType, $"{aSol} SupportTypeShape{aSide}");
                mySupportKinds[(int)aType]++;
                using (N.TopoDS_Shape aNS = aSide == 1 ? aN.SupportOnShape1(i) : aN.SupportOnShape2(i))
                {
                    CompareSupport(theOracle, aNS, aSide == 1 ? aP.SupportOnShape1(i) : aP.SupportOnShape2(i), aMaps1, aMaps2,
                                   $"{aSol} SupportOnShape{aSide}");
                }

                if (aType == BRepExtrema_SupportType.BRepExtrema_IsOnEdge)
                {
                    double aNT = 0;
                    double aPT;
                    if (aSide == 1)
                    {
                        aN.ParOnEdgeS1(i, ref aNT);
                        aP.ParOnEdgeS1(i, out aPT);
                    }
                    else
                    {
                        aN.ParOnEdgeS2(i, ref aNT);
                        aP.ParOnEdgeS2(i, out aPT);
                    }
                    theOracle.Same(aNT, aPT, $"{aSol} ParOnEdgeS{aSide}");
                }
                else if (aType == BRepExtrema_SupportType.BRepExtrema_IsInFace)
                {
                    double aNU = 0, aNV = 0;
                    double aPU, aPV;
                    if (aSide == 1)
                    {
                        aN.ParOnFaceS1(i, ref aNU, ref aNV);
                        aP.ParOnFaceS1(i, out aPU, out aPV);
                    }
                    else
                    {
                        aN.ParOnFaceS2(i, ref aNU, ref aNV);
                        aP.ParOnFaceS2(i, out aPU, out aPV);
                    }
                    theOracle.Same(aNU, aPU, $"{aSol} ParOnFaceS{aSide} U");
                    theOracle.Same(aNV, aPV, $"{aSol} ParOnFaceS{aSide} V");
                }
            }
        }
    }

    private void Summary(Oracle theOracle, string theWhat)
    {
        output.WriteLine($"{myNbComputations} computations, {myNbSolutions} solutions ({myNbInner} inner); supports: "
                         + $"{mySupportKinds[0]} vertices, {mySupportKinds[1]} on edges, {mySupportKinds[2]} in faces; "
                         + $"{myNbBuiltSupports} built by the algorithm; {myNbRacingInner} multi-threaded inner solutions checked as valid, not compared");
        theOracle.Report(theWhat);
    }

    private static Twin<N.TopoDS_Shape, TopoDS_Shape> Pick(Oracle theOracle, Twin<N.TopoDS_Shape, TopoDS_Shape> theShape, TopAbs_ShapeEnum theType)
    {
        List<Twin<N.TopoDS_Shape, TopoDS_Shape>> aList = ClassificationScenes.Explore(theShape, theType);
        return aList.Count == 0 ? theShape : aList[theOracle.Int(0, aList.Count - 1)];
    }

    private static Twin<N.TopoDS_Shape, TopoDS_Shape> PointVertex(in gp_Pnt theP)
    {
        using var aNP = Oracle.ToNative(theP);
        using var aNMaker = new N.BRepBuilderAPI_MakeVertex(aNP);
        // TODO(3K-merge): BRepBuilderAPI_MakeVertex
        return new(aNMaker.Vertex(), new BRepLib_MakeVertex(theP).Vertex());
    }

    // A direction that native OCCT and the port normalise to the same numbers.
    private static gp_Dir Dir(Oracle theOracle)
    {
        while (true)
        {
            gp_Dir aD = Oracle.Settled(theOracle.Dir());
            using var aND = Oracle.ToNative(aD);
            if (aND.X() == aD.X() && aND.Y() == aD.Y() && aND.Z() == aD.Z())
            {
                return aD;
            }
        }
    }

    /// <summary>An infinite line through a point of the shape's box, as an edge.</summary>
    private static Twin<N.TopoDS_Shape, TopoDS_Shape> InfiniteLine(Oracle theOracle, in gp_Pnt theP)
    {
        gp_Dir aD = Dir(theOracle);
        using var aNP = Oracle.ToNative(theP);
        using var aND = Oracle.ToNative(aD);
        using var aNL = new N.gp_Lin(aNP, aND);
        using var aNMaker = new N.BRepBuilderAPI_MakeEdge(aNL);
        // TODO(3K-merge): BRepBuilderAPI_MakeEdge
        return new(aNMaker.Edge(), new BRepLib_MakeEdge(new gp_Lin(theP, aD)).Edge());
    }

    /// <summary>An infinite horizontal plane at the point's height, as a face.</summary>
    private static Twin<N.TopoDS_Shape, TopoDS_Shape> InfinitePlane(in gp_Pnt theP)
    {
        using var aNP = Oracle.ToNative(theP);
        using var aND = new N.gp_Dir(0, 0, 1);
        using var aNPl = new N.gp_Pln(aNP, aND);
        using var aNMaker = new N.BRepBuilderAPI_MakeFace(aNPl);
        // TODO(3K-merge): BRepBuilderAPI_MakeFace
        return new(aNMaker.Face(), new BRepLib_MakeFace(new gp_Pln(theP, new gp_Dir(0, 0, 1))).Face());
    }

    [Fact]
    public void DistShapeShape_on_twin_scenes_matches_native_OCCT()
    {
        var anOracle = new Oracle(output, 31011);
        const int Pairs = 800;
        for (int k = 0; k < Pairs; k++)
        {
            int aScene1 = anOracle.Int(0, 3 * ClassificationScenes.Kinds - 1);
            int aScene2 = anOracle.Int(0, 3 * ClassificationScenes.Kinds - 1);
            if (!ClassificationScenes.HasSurfaces(aScene1) || !ClassificationScenes.HasSurfaces(aScene2))
            {
                continue;
            }

            Twin<N.TopoDS_Shape, TopoDS_Shape> aShape1 = ClassificationScenes.Scene(anOracle, aScene1);
            Twin<N.TopoDS_Shape, TopoDS_Shape> aShape2 = ClassificationScenes.Scene(anOracle, aScene2);
            string aWhat = $"pair {k}: {ClassificationScenes.Name(aScene1)} / {ClassificationScenes.Name(aScene2)}";
            List<gp_Pnt> aPoints = ClassificationScenes.SamplePoints(anOracle, aShape1.P, 3, 1);
            int aCase = k % 8;
            switch (aCase)
            {
                case 0:
                    Compare(anOracle, aShape1, aShape2, aWhat + " (shapes)");
                    break;
                case 1:
                    Compare(anOracle, Pick(anOracle, aShape1, TopAbs_VERTEX), aShape2, aWhat + " (vertex, shape)");
                    break;
                case 2:
                    Compare(anOracle, Pick(anOracle, aShape1, TopAbs_EDGE), Pick(anOracle, aShape2, TopAbs_EDGE), aWhat + " (edges)");
                    break;
                case 3:
                    Compare(anOracle, Pick(anOracle, aShape1, TopAbs_EDGE), Pick(anOracle, aShape2, TopAbs_FACE), aWhat + " (edge, face)");
                    break;
                case 4:
                    Compare(anOracle, Pick(anOracle, aShape1, TopAbs_FACE), Pick(anOracle, aShape2, TopAbs_FACE), aWhat + " (faces)");
                    break;
                case 5:
                    // points in and around the first shape: inner solutions for the solids
                    Compare(anOracle, aShape1, PointVertex(aPoints[anOracle.Int(0, aPoints.Count - 1)]), aWhat + " (shape, point)");
                    break;
                case 6:
                    Compare(anOracle, InfiniteLine(anOracle, aPoints[0]), Pick(anOracle, aShape2, TopAbs_EDGE), aWhat + " (infinite line, edge)");
                    break;
                default:
                    Compare(anOracle, Pick(anOracle, aShape2, TopAbs_EDGE), InfinitePlane(aPoints[0]), aWhat + " (edge, infinite plane)");
                    break;
            }
        }

        Summary(anOracle, "BRepExtrema_DistShapeShape on twin scenes");
    }

    /// <summary>The native shape written to a file and read back by both sides.</summary>
    private Twin<N.TopoDS_Shape, TopoDS_Shape> ReadBoth(N.TopoDS_Shape theShape)
    {
        string aFile = NewFile();
        Assert.True(N.BRepTools.Write(theShape, aFile));
        var aNShape = new N.TopoDS_Shape();
        using (var aBuilder = new N.BRep_Builder())
        {
            Assert.True(N.BRepTools.Read(aNShape, aFile, aBuilder));
        }

        Assert.True(BRepTools.Read(out TopoDS_Shape aPShape, aFile));
        return new(aNShape, aPShape);
    }

    [Fact]
    public void DistShapeShape_on_native_shapes_matches_native_OCCT()
    {
        var anOracle = new Oracle(output, 31012);
        const int Pairs = 160;
        for (int k = 0; k < Pairs; k++)
        {
            Twin<N.TopoDS_Shape, TopoDS_Shape> aShape1 = ReadBoth(BRepToolsIOOracleTests.NativeShape(anOracle, k, out string aWhat1));
            Twin<N.TopoDS_Shape, TopoDS_Shape> aShape2 = ReadBoth(BRepToolsIOOracleTests.NativeShape(anOracle, k + 5, out string aWhat2));
            (N.TopLoc_Location aNL, TopLoc_Location aL) = ClassificationScenes.RandomLocation(anOracle, false);
            if (k % 4 != 1)
            {
                aShape2 = new(aShape2.N.Moved(aNL), aShape2.P.Moved(aL));
            }
            string aWhat = $"pair {k}: {aWhat1} / {aWhat2}";
            switch (k % 4)
            {
                case 0:
                    Compare(anOracle, aShape1, aShape2, aWhat + " (shapes)");
                    break;
                case 1:
                    Compare(anOracle, aShape1, aShape2, aWhat + " (shapes, overlapping)");
                    break;
                case 2:
                    Compare(anOracle, Pick(anOracle, aShape1, TopAbs_EDGE), Pick(anOracle, aShape2, TopAbs_FACE), aWhat + " (edge, face)");
                    break;
                default:
                    Compare(anOracle, Pick(anOracle, aShape1, TopAbs_FACE), Pick(anOracle, aShape2, TopAbs_FACE), aWhat + " (faces)");
                    break;
            }
        }

        Summary(anOracle, "BRepExtrema_DistShapeShape on native shapes");
    }
}
#endif
