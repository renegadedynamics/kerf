// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Extrema;
using Kerf.Engine.Measurement.Distance;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Text;
using Xunit.Abstractions;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.Oracle.Tests;

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

    private string PortText(in Shape theShape)
    {
        string aFile = NewFile();
        Assert.True(BrepTools.Write(theShape, aFile));
        return File.ReadAllText(aFile, Encoding.Latin1);
    }

    /// <summary>The vertex, edge and face maps of an input, on both sides.</summary>
    private sealed class Maps
    {
        public readonly N.TopTools_IndexedMapOfShape[] NM = new N.TopTools_IndexedMapOfShape[3];
        public readonly IndexedSet<Shape>[] PM = new IndexedSet<Shape>[3];

        public Maps(N.TopoDS_Shape theN, in Shape theP)
        {
            ShapeType[] aTypes = [ShapeType.Vertex, ShapeType.Edge, ShapeType.Face];
            for (int i = 0; i < 3; i++)
            {
                NM[i] = new N.TopTools_IndexedMapOfShape();
                N.TopExp.MapShapes(theN, TwinShapes.ToNative(aTypes[i]), NM[i]);
                PM[i] = new IndexedSet<Shape>(ShapeHasher.Instance);
                Exploration.MapShapes(theP, aTypes[i], PM[i]);
                Assert.Equal(NM[i].Extent(), PM[i].Count);
            }
        }
    }

    private void CompareSupport(Oracle theOracle, N.TopoDS_Shape theN, in Shape theP, Maps theMaps1, Maps theMaps2, string theWhat)
    {
        theOracle.Same((int)theN.ShapeType(), (int)theP.Type, theWhat + " type");
        int aKind = theP.Type switch
        {
            ShapeType.Vertex => 0,
            ShapeType.Edge => 1,
            _ => 2,
        };

        // the support may come from either input (an inner solution's vertex is the other shape's)
        int aNIndex = theMaps1.NM[aKind].FindIndex(theN);
        int aPIndex = theMaps1.PM[aKind].IndexOf(theP) + 1;
        int aNIndex2 = theMaps2.NM[aKind].FindIndex(theN);
        int aPIndex2 = theMaps2.PM[aKind].IndexOf(theP) + 1;
        theOracle.Same(aNIndex, aPIndex, theWhat + " index in shape 1");
        theOracle.Same(aNIndex2, aPIndex2, theWhat + " index in shape 2");
        theOracle.Same((int)theN.Orientation(), (int)theP.Orientation, theWhat + " orientation");
        if (aPIndex == 0 && aPIndex2 == 0)
        {
            // built by the algorithm (the trimmed copy of an infinite edge or face): compare it as a file
            myNbBuiltSupports++;
            Assert.True(theP.TShape()!.IsFrozen, theWhat + ": a support built by the algorithm is frozen");
            Assert.True(NativeText(theN) == PortText(theP), theWhat + ": the built support differs from native OCCT's");
        }
    }

    /// <summary>Runs both sides on the pair with the same settings and compares everything.</summary>
    private void Compare(Oracle theOracle, Twin<N.TopoDS_Shape, Shape> theS1, Twin<N.TopoDS_Shape, Shape> theS2, string theWhat)
    {
        double aDeflection = theOracle.Int(0, 3) switch
        {
            0 => global::Kerf.Engine.Foundation.Precision.Confusion(),
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

        var aP = new DistanceShapeShape();
        aP.SetDeflection(aDeflection);
        aP.SetMultiThread(aMulti);
        aP.SetFlag((ExtFlag)aFlag);
        aP.SetAlgo((ExtAlgo)anAlgo);
        aP.LoadS1(theS1.P);
        aP.LoadS2(theS2.P);
        bool aPDone = aP.Perform();

        myNbComputations++;
        theOracle.Same(aNDone, aPDone, aWhat + " Perform");
        theOracle.Same(aN.IsDone(), aP.IsDone, aWhat + " IsDone");
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
            for (int i = 1; i <= aP.SolutionCount; i++)
            {
                Shape aV = aP.SupportOnShape1(i);
                Assert.True(aP.SupportOnShape2(i).IsSame(aV), aWhat);
                Assert.True(aMapsA.PM[0].Contains(aV) || aMapsB.PM[0].Contains(aV), aWhat);
                Assert.Equal(Tool.Pnt(ShapeCasts.Vertex(aV)), aP.PointOnShape1(i));
            }
            return;
        }

        theOracle.Same(aN.NbSolution(), aP.SolutionCount, aWhat + " NbSolution");
        if (!aP.IsDone)
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
            for (int i = 1; i <= aP.SolutionCount; i++)
            {
                using N.gp_Pnt a1 = aN.PointOnShape1(i);
                using N.gp_Pnt a2 = aN.PointOnShape2(i);
                output.WriteLine($"  N {i}: {a1.X():R} {a1.Y():R} {a1.Z():R} | {a2.X():R} {a2.Y():R} {a2.Z():R} {aN.SupportTypeShape1(i)} {aN.SupportTypeShape2(i)}");
                output.WriteLine($"  P {i}: {aP.PointOnShape1(i)} | {aP.PointOnShape2(i)} {aP.SupportTypeShape1(i)} {aP.SupportTypeShape2(i)}");
            }
        }
        var aMaps1 = new Maps(theS1.N, theS1.P);
        var aMaps2 = new Maps(theS2.N, theS2.P);
        for (int i = 1; i <= aP.SolutionCount; i++)
        {
            myNbSolutions++;
            string aSol = $"{aWhat} solution {i}";
            using (N.gp_Pnt aNP = aN.PointOnShape1(i))
            {
                SamePoint(theOracle, aNP, aP.PointOnShape1(i), aSol + " PointOnShape1");
            }
            using (N.gp_Pnt aNP = aN.PointOnShape2(i))
            {
                SamePoint(theOracle, aNP, aP.PointOnShape2(i), aSol + " PointOnShape2");
            }

            for (int aSide = 1; aSide <= 2; aSide++)
            {
                SupportType aType = aSide == 1 ? aP.SupportTypeShape1(i) : aP.SupportTypeShape2(i);
                N.BRepExtrema_SupportType aNType = aSide == 1 ? aN.SupportTypeShape1(i) : aN.SupportTypeShape2(i);
                theOracle.Same((int)aNType, (int)aType, $"{aSol} SupportTypeShape{aSide}");
                mySupportKinds[(int)aType]++;
                using (N.TopoDS_Shape aNS = aSide == 1 ? aN.SupportOnShape1(i) : aN.SupportOnShape2(i))
                {
                    CompareSupport(theOracle, aNS, aSide == 1 ? aP.SupportOnShape1(i) : aP.SupportOnShape2(i), aMaps1, aMaps2,
                                   $"{aSol} SupportOnShape{aSide}");
                }

                if (aType == SupportType.IsOnEdge)
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
                else if (aType == SupportType.IsInFace)
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
                         + $"{myNbBuiltSupports} built by the algorithm; {myNbRacingInner} multi-threaded inner solutions checked as valid, not compared; "
                         + $"{myNbOriginPoints} native points at the origin (intended differences)");
        theOracle.Report(theWhat);
    }

    private static Twin<N.TopoDS_Shape, Shape> Pick(Oracle theOracle, Twin<N.TopoDS_Shape, Shape> theShape, ShapeType theType)
    {
        List<Twin<N.TopoDS_Shape, Shape>> aList = ClassificationScenes.Explore(theShape, theType);
        return aList.Count == 0 ? theShape : aList[theOracle.Int(0, aList.Count - 1)];
    }

    private static Twin<N.TopoDS_Shape, Shape> PointVertex(in Point3D theP)
    {
        using var aNP = Oracle.ToNative(theP);
        using var aNMaker = new N.BRepBuilderAPI_MakeVertex(aNP);
        return new(aNMaker.Vertex(), new MakeVertex(theP).Vertex());
    }

    // A direction that native OCCT and the port normalise to the same numbers.
    private static Direction3D Dir(Oracle theOracle)
    {
        while (true)
        {
            Direction3D aD = Oracle.Settled(theOracle.Dir());
            using var aND = Oracle.ToNative(aD);
            if (aND.X() == aD.X && aND.Y() == aD.Y && aND.Z() == aD.Z)
            {
                return aD;
            }
        }
    }

    /// <summary>An infinite line through a point of the shape's box, as an edge.</summary>
    private static Twin<N.TopoDS_Shape, Shape> InfiniteLine(Oracle theOracle, in Point3D theP)
    {
        Direction3D aD = Dir(theOracle);
        using var aNP = Oracle.ToNative(theP);
        using var aND = Oracle.ToNative(aD);
        using var aNL = new N.gp_Lin(aNP, aND);
        using var aNMaker = new N.BRepBuilderAPI_MakeEdge(aNL);
        return new(aNMaker.Edge(), new MakeEdge(new Line3D(theP, aD)).Edge());
    }

    /// <summary>An infinite horizontal plane at the point's height, as a face.</summary>
    private static Twin<N.TopoDS_Shape, Shape> InfinitePlane(in Point3D theP)
    {
        using var aNP = Oracle.ToNative(theP);
        using var aND = new N.gp_Dir(0, 0, 1);
        using var aNPl = new N.gp_Pln(aNP, aND);
        using var aNMaker = new N.BRepBuilderAPI_MakeFace(aNPl);
        return new(aNMaker.Face(), new MakeFace(new Plane3D(theP, new Direction3D(0, 0, 1))).Face());
    }

    [Fact]
    public void DistShapeShape_on_twin_scenes_matches_native_OCCT()
    {
        var anOracle = new Oracle(output, 31011);
        const int Pairs = 800;
        foreach (int k in Tape.Budget(Pairs))
        {
            int aScene1 = anOracle.Int(0, 3 * ClassificationScenes.Kinds - 1);
            int aScene2 = anOracle.Int(0, 3 * ClassificationScenes.Kinds - 1);
            if (!ClassificationScenes.HasSurfaces(aScene1) || !ClassificationScenes.HasSurfaces(aScene2))
            {
                continue;
            }

            Twin<N.TopoDS_Shape, Shape> aShape1 = ClassificationScenes.Scene(anOracle, aScene1);
            Twin<N.TopoDS_Shape, Shape> aShape2 = ClassificationScenes.Scene(anOracle, aScene2);
            string aWhat = $"pair {k}: {ClassificationScenes.Name(aScene1)} / {ClassificationScenes.Name(aScene2)}";
            List<Point3D> aPoints = ClassificationScenes.SamplePoints(anOracle, aShape1.P, 3, 1);
            int aCase = k % 8;
            switch (aCase)
            {
                case 0:
                    Compare(anOracle, aShape1, aShape2, aWhat + " (shapes)");
                    break;
                case 1:
                    Compare(anOracle, Pick(anOracle, aShape1, ShapeType.Vertex), aShape2, aWhat + " (vertex, shape)");
                    break;
                case 2:
                    Compare(anOracle, Pick(anOracle, aShape1, ShapeType.Edge), Pick(anOracle, aShape2, ShapeType.Edge), aWhat + " (edges)");
                    break;
                case 3:
                    Compare(anOracle, Pick(anOracle, aShape1, ShapeType.Edge), Pick(anOracle, aShape2, ShapeType.Face), aWhat + " (edge, face)");
                    break;
                case 4:
                    Compare(anOracle, Pick(anOracle, aShape1, ShapeType.Face), Pick(anOracle, aShape2, ShapeType.Face), aWhat + " (faces)");
                    break;
                case 5:
                    // points in and around the first shape: inner solutions for the solids
                    Compare(anOracle, aShape1, PointVertex(aPoints[anOracle.Int(0, aPoints.Count - 1)]), aWhat + " (shape, point)");
                    break;
                case 6:
                    Compare(anOracle, InfiniteLine(anOracle, aPoints[0]), Pick(anOracle, aShape2, ShapeType.Edge), aWhat + " (infinite line, edge)");
                    break;
                default:
                    Compare(anOracle, Pick(anOracle, aShape2, ShapeType.Edge), InfinitePlane(aPoints[0]), aWhat + " (edge, infinite plane)");
                    break;
            }
        }

        Summary(anOracle, "BRepExtrema_DistShapeShape on twin scenes");
    }

    /// <summary>The native shape written to a file and read back by both sides.</summary>
    private Twin<N.TopoDS_Shape, Shape> ReadBoth(N.TopoDS_Shape theShape)
    {
        string aFile = NewFile();
        Assert.True(N.BRepTools.Write(theShape, aFile));
        var aNShape = new N.TopoDS_Shape();
        using (var aBuilder = new N.BRep_Builder())
        {
            Assert.True(N.BRepTools.Read(aNShape, aFile, aBuilder));
        }

        Assert.True(BrepTools.Read(out Shape aPShape, aFile));
        return new(aNShape, aPShape);
    }

    [Fact]
    public void DistShapeShape_on_native_shapes_matches_native_OCCT()
    {
        var anOracle = new Oracle(output, 31012);
        const int Pairs = 160;
        foreach (int k in Tape.Budget(Pairs))
        {
            Twin<N.TopoDS_Shape, Shape> aShape1 = ReadBoth(BRepToolsIOOracleTests.NativeShape(anOracle, k, out string aWhat1));
            Twin<N.TopoDS_Shape, Shape> aShape2 = ReadBoth(BRepToolsIOOracleTests.NativeShape(anOracle, k + 5, out string aWhat2));
            (N.TopLoc_Location aNL, Placement aL) = ClassificationScenes.RandomLocation(anOracle, false);
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
                    Compare(anOracle, Pick(anOracle, aShape1, ShapeType.Edge), Pick(anOracle, aShape2, ShapeType.Face), aWhat + " (edge, face)");
                    break;
                default:
                    Compare(anOracle, Pick(anOracle, aShape1, ShapeType.Face), Pick(anOracle, aShape2, ShapeType.Face), aWhat + " (faces)");
                    break;
            }
        }

        Summary(anOracle, "BRepExtrema_DistShapeShape on native shapes");
    }

    /// <summary>
    /// Plan 3.2's third level (occt-issues.md, unit 4A's Extrema_ExtCS row; Kerf unit 10K1, decision 1430): where an
    /// edge's range clipped to the face's box is shorter than Precision::Confusion, native reported the origin as the
    /// point on the edge, the port the range's middle. Native's origin is accepted for any port point.
    /// </summary>
    private void SamePoint(Oracle theOracle, N.gp_Pnt theNative, in Point3D thePort, string theWhat)
    {
        // native's coordinates read once each, in the recorded order
        double x = theNative.X(), y = theNative.Y(), z = theNative.Z();
        if (x == 0.0 && y == 0.0 && z == 0.0 && thePort.DistanceTo(new Point3D(0, 0, 0)) > 0.0)
        {
            myNbOriginPoints++;
            return;
        }
        theOracle.Same(x, y, z, thePort, theWhat);
    }

    private int myNbOriginPoints;
}
