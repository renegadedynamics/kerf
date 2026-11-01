#if HAS_ORACLE
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Reflection;
using System.Text.RegularExpressions;
using OcctNet.DrawLite;
using Xunit.Abstractions;
using N = Build123d.Occt;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// The BRep round-trip milestone (phase3-plan.md, section 4): Draw-lite's restore, nbshapes, explode, lprops, sprops,
/// vprops, tolerance and maxtolerance on the data/occ files, against native OCCT reading the same file; and OCCT's
/// own check procedures (checknbshapes, checkprops, checkmaxtol) on them with the native values as references.
/// </summary>
/// <remarks>
/// The props commands are run with <c>-full</c>, which prints every value with <c>%.17g</c>, so the comparison is
/// exact. Native values are computed as the Draw commands compute them: BRepTools_ShapeSet's extent as the distinct
/// TShapes reached through TopoDS_Iterator (TopTools_ShapeSet::Add), BRepGProp with the GProp_GProps that
/// BRepTest_GPropCommands holds, the tolerances through TopExp_Explorer and BRep_Tool::Tolerance as
/// ShapeAnalysis_ShapeTolerance sums them.
/// </remarks>
public partial class DrawLiteShapesOracleTests(ITestOutputHelper output)
{
    private static string OcctSourceDirectory
        => typeof(DrawLiteShapesOracleTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == "OcctSourceDir").Value!;

    private static string ResourcesDirectory => Path.Combine(OcctSourceDirectory, "resources", "DrawResources");

    private static readonly string[] theTypeNames = ["COMPOUND", "COMPSOLID", "SOLID", "SHELL", "FACE", "WIRE", "EDGE", "VERTEX"];

    [GeneratedRegex(@"[-+]?(?:inf|nan(?:\(ind\))?|\d+(?:\.\d*)?(?:e[-+]?\d+)?)")]
    private static partial Regex Number();

    private static double ParseNumber(string theText) => theText switch
    {
        "inf" or "+inf" => double.PositiveInfinity,
        "-inf" => double.NegativeInfinity,
        _ when theText.Contains("nan", StringComparison.Ordinal) => double.NaN,
        _ => double.Parse(theText, NumberStyles.Float, CultureInfo.InvariantCulture),
    };

    private static List<double> Numbers(string theText) => Number().Matches(theText).Select(m => ParseNumber(m.Value)).ToList();

    private static string TclPath(string thePath) => thePath.Replace('\\', '/');

    private static string[] Words(string theList) => theList.Split(' ', StringSplitOptions.RemoveEmptyEntries);

    // TopTools_ShapeSet::Add: the distinct TShapes, each located at the identity
    private static void AddTShapes(N.TopoDS_Shape theShape, N.TopTools_IndexedMapOfShape theMap)
    {
        using var anIdentity = new N.TopLoc_Location();
        N.TopoDS_Shape aShape = theShape.Located(anIdentity);
        if (theMap.Contains(aShape))
        {
            return;
        }

        using (var anIt = new N.TopoDS_Iterator(aShape, false, false))
        {
            for (; anIt.More(); anIt.Next())
            {
                AddTShapes(anIt.Value(), theMap);
            }
        }
        GC.KeepAlive(aShape); // native TopoDS_Iterator holds no reference to the shape it iterates (decision 10409)

        theMap.Add(aShape);
    }

    private static int[] NativeExtent(N.TopoDS_Shape theShape)
    {
        using var aMap = new N.TopTools_IndexedMapOfShape();
        AddTShapes(theShape, aMap);
        int[] aCounts = new int[8];
        for (int i = 1; i <= aMap.Extent(); i++)
        {
            aCounts[(int)aMap.FindKey(i).ShapeType()]++;
        }

        return aCounts;
    }

    private static int[] NativeLocatedExtent(N.TopoDS_Shape theShape)
    {
        int[] aCounts = new int[8];
        for (int t = 0; t < 8; t++)
        {
            using var aMap = new N.TopTools_IndexedMapOfShape();
            N.TopExp.MapShapes(theShape, (N.TopAbs_ShapeEnum)t, aMap);
            aCounts[t] = aMap.Extent();
        }

        return aCounts;
    }

    // explode name type: the distinct sub-shapes of the type other than the shape itself
    private static int NativeExplodeCount(N.TopoDS_Shape theShape, int theType)
    {
        using var aMap = new N.TopTools_MapOfShape();
        aMap.Add(theShape);
        int aCount = 0;
        using var anEx = new N.TopExp_Explorer(theShape, (N.TopAbs_ShapeEnum)theType);
        for (; anEx.More(); anEx.Next())
        {
            if (aMap.Add(anEx.Current()))
            {
                aCount++;
            }
        }

        return aCount;
    }

    private static List<double> NativeTolerances(N.TopoDS_Shape theShape, int theType)
    {
        var aTols = new List<double>();
        using var anEx = new N.TopExp_Explorer(theShape, (N.TopAbs_ShapeEnum)theType);
        for (; anEx.More(); anEx.Next())
        {
            aTols.Add(theType switch
            {
                4 => N.BRep_Tool.Tolerance(N.TopoDS.Face(anEx.Current())),
                6 => N.BRep_Tool.Tolerance(N.TopoDS.Edge(anEx.Current())),
                _ => N.BRep_Tool.Tolerance(N.TopoDS.Vertex(anEx.Current())),
            });
        }

        return aTols;
    }

    // ShapeAnalysis_ShapeTolerance's MAX, AVG and MIN of a list of tolerances (0 for none)
    private static (double Max, double Avg, double Min) Summary(List<double> theTols)
    {
        if (theTols.Count == 0)
        {
            return (0, 0, 0);
        }

        double aSum = theTols[0];
        for (int i = 1; i < theTols.Count; i++)
        {
            aSum += theTols[i];
        }

        double aMin = theTols.Min(), aMax = theTols.Max();
        return (aMax, aMin == aMax ? aMin : aSum / theTols.Count, aMin);
    }

    private static (double Mass, double Error, N.GProp_GProps Props) NativeProps(N.TopoDS_Shape theShape, char theKind, bool theWithEps)
    {
        var aProps = new N.GProp_GProps();
        double anError = 0;
        switch (theKind)
        {
            case 'l':
                N.BRepGProp.LinearProperties(theShape, aProps, false);
                break;
            case 's':
                if (theWithEps)
                {
                    anError = N.BRepGProp.SurfaceProperties(theShape, aProps, 1.0e-4, false);
                }
                else
                {
                    N.BRepGProp.SurfaceProperties(theShape, aProps, false, false);
                }

                break;
            default:
                if (theWithEps)
                {
                    anError = N.BRepGProp.VolumeProperties(theShape, aProps, 1.0e-4, false, false);
                }
                else
                {
                    N.BRepGProp.VolumeProperties(theShape, aProps, false, false, false);
                }

                break;
        }

        return (aProps.Mass(), anError, aProps);
    }

    /// <summary>The values of "props -full" in their order: mass, [error], X, Y, Z, the matrix by rows, IX, IY, IZ.</summary>
    private static List<double> NativePropsValues(N.TopoDS_Shape theShape, char theKind, bool theWithEps)
    {
        (double aMass, double anError, N.GProp_GProps aProps) = NativeProps(theShape, theKind, theWithEps);
        using (aProps)
        {
            var aValues = new List<double> { aMass };
            if (theWithEps && theKind != 'l')
            {
                aValues.Add(anError);
            }

            using (N.gp_Pnt aCentre = aProps.CentreOfMass())
            {
                aValues.AddRange([aCentre.X(), aCentre.Y(), aCentre.Z()]);
            }

            using (N.gp_Mat aMatrix = aProps.MatrixOfInertia())
            {
                for (int r = 1; r <= 3; r++)
                {
                    for (int c = 1; c <= 3; c++)
                    {
                        aValues.Add(aMatrix.Value(r, c));
                    }
                }
            }

            double aIx = 0, aIy = 0, aIz = 0;
            using (N.GProp_PrincipalProps aPrincipal = aProps.PrincipalProperties())
            {
                aPrincipal.Moments(ref aIx, ref aIy, ref aIz);
            }

            aValues.AddRange([aIx, aIy, aIz]);
            return aValues;
        }
    }

    private void CompareValues(Oracle theOracle, List<double> theNative, List<double> thePort, string theWhat)
    {
        Assert.True(theNative.Count == thePort.Count, $"{theWhat}: {thePort.Count} values printed, {theNative.Count} expected");
        // the inertia and the moments are compared relative to the largest of them (they mix magnitudes)
        double aScale = Math.Max(1.0, theNative.Select(Math.Abs).Where(double.IsFinite).DefaultIfEmpty(0).Max());
        for (int i = 0; i < theNative.Count; i++)
        {
            if (double.IsNaN(theNative[i]))
            {
                Assert.True(double.IsNaN(thePort[i]), $"{theWhat}[{i}]: native NaN, port {thePort[i]}");
                continue;
            }

            double anEntry = Math.Max(1.0, Math.Abs(theNative[i]));
            theOracle.Same(theNative[i], thePort[i], $"{theWhat}[{i}]", Oracle.Tolerance * aScale / anEntry);
        }
    }

    private static string[] DataOccFiles()
        => Directory.GetFiles(Path.Combine(OcctSourceDirectory, "data", "occ"))
                    .Where(f => f.EndsWith(".brep", StringComparison.Ordinal) || f.EndsWith(".rle", StringComparison.Ordinal))
                    .Order(StringComparer.Ordinal)
                    .ToArray();

    [Fact]
    public void Draw_commands_on_the_data_occ_files_answer_as_native_OCCT()
    {
        string[] aFiles = DataOccFiles();
        Assert.Equal(42, aFiles.Length); // 37 .brep and 5 .rle
        var anOracle = new Oracle(output);
        int aChecks = 0;
        foreach (string aFile in aFiles)
        {
            string aName = Path.GetFileName(aFile);
            var aLog = new StringWriter { NewLine = "\n" };
            var aDI = new DrawInterpreter(aLog);
            aDI.LoadDrawResources(ResourcesDirectory);
            Assert.Equal("s", aDI.Eval($"restore {{{TclPath(aFile)}}} s"));
            Assert.True(aDI.GetShape("s").TShape()!.IsFrozen());

            var aNS = new N.TopoDS_Shape();
            using (var aBuilder = new N.BRep_Builder())
            {
                Assert.True(N.BRepTools.Read(aNS, aFile, aBuilder), aFile);
            }

            // nbshapes and nbshapes -t
            int[] anExtent = NativeExtent(aNS);
            int[] aLocated = NativeLocatedExtent(aNS);
            foreach ((string aCommand, int[] aCounts) in new[] { ("nbshapes s", anExtent), ("nbshapes s -t", aLocated) })
            {
                string anExpected = "Number of shapes in s\n"
                                    + string.Concat(Enumerable.Range(0, 8).Reverse().Select(t => $" {theTypeNames[t],-9} : {aCounts[t]}\n"))
                                    + $" SHAPE     : {aCounts.Sum()}\n";
                Assert.Equal(anExpected, aDI.Eval(aCommand));
            }

            // checknbshapes with the native counts
            aLog.GetStringBuilder().Clear();
            aDI.Eval($"checknbshapes s -vertex {anExtent[7]} -edge {anExtent[6]} -wire {anExtent[5]} -face {anExtent[4]} "
                     + $"-shell {anExtent[3]} -solid {anExtent[2]} -compsolid {anExtent[1]} -compound {anExtent[0]} -shape {anExtent.Sum()}");
            Assert.DoesNotContain("Error", aLog.ToString());
            Assert.Contains("OK", aLog.ToString());

            // explode: the children, then each type
            using (var anIt = new N.TopoDS_Iterator(aNS))
            {
                int aChildren = 0;
                for (; anIt.More(); anIt.Next())
                {
                    aChildren++;
                }

                Assert.Equal(aChildren, Words(aDI.Eval("explode s")).Length);
            }
            GC.KeepAlive(aNS); // native TopoDS_Iterator holds no reference to the shape it iterates (decision 10409)

            foreach ((string aCode, int aType) in new[] { ("so", 2), ("sh", 3), ("f", 4), ("w", 5), ("e", 6), ("v", 7) })
            {
                string[] aNames = Words(aDI.Eval($"explode s {aCode}"));
                Assert.Equal(NativeExplodeCount(aNS, aType), aNames.Length);
                Assert.All(aNames, n => Assert.Equal("1", aDI.Eval($"isdraw {n}")));
                aChecks++;
            }

            // lprops, sprops, vprops: as checkprops runs them (epsilon 1.0e-4), and without epsilon
            foreach (char aKind in "lsv")
            {
                foreach (bool aWithEps in new[] { true, false })
                {
                    string aCommand = $"{aKind}props s{(aWithEps ? " 1.0e-4" : "")} -full";
                    List<double> aPort = Numbers(aDI.Eval(aCommand));
                    CompareValues(anOracle, NativePropsValues(aNS, aKind, aWithEps), aPort, $"{aName} {aCommand}");
                    aChecks++;
                }
            }

            // checkprops with the native values, which must find them within its 1%
            double aLength = NativeProps(aNS, 'l', true).Mass, anArea = NativeProps(aNS, 's', true).Mass, aVolume = NativeProps(aNS, 'v', true).Mass;
            aLog.GetStringBuilder().Clear();
            string R(double theValue) => theValue.ToString("R", CultureInfo.InvariantCulture);
            aDI.Eval($"checkprops s -l {(aLength == 0 ? "empty" : R(aLength))} -s {(anArea == 0 ? "empty" : R(anArea))} -v {(aVolume == 0 ? "empty" : R(aVolume))}");
            Assert.DoesNotContain("Error", aLog.ToString());

            // tolerance and checkmaxtol
            List<double> aFaces = NativeTolerances(aNS, 4), anEdges = NativeTolerances(aNS, 6), aVertices = NativeTolerances(aNS, 7);
            var anAll = aFaces.Concat(anEdges).Concat(aVertices).ToList();
            List<double> aPortTols = Numbers(aDI.Eval("tolerance s"));
            var aNativeTols = new List<double>();
            foreach (List<double> aList in new[] { anAll, aFaces, anEdges, aVertices })
            {
                (double aMax, double anAvg, double aMin) = Summary(aList);
                aNativeTols.AddRange([aMax, anAvg, aMin]);
            }

            CompareValues(anOracle, aNativeTols, aPortTols, $"{aName} tolerance");
            aLog.GetStringBuilder().Clear();
            Assert.Equal(aDI.Eval("checkmaxtol s -ref " + R(Summary(anAll).Max)), Standard_Stream.FormatReal(Summary(anAll).Max, 17));
            Assert.DoesNotContain("Error", aLog.ToString());

            // maxtolerance: the counts and the extreme tolerances
            string aMaxTol = aDI.Eval("maxtolerance s");
            Assert.Contains($"(nbFaces:{DistinctCount(aNS, 4)}  nbEdges:{DistinctCount(aNS, 6)} nbVtx:{DistinctCount(aNS, 7)})", aMaxTol);
            aChecks += 3;
            aNS.Dispose();
        }

        output.WriteLine($"{aFiles.Length} data/occ files, {aChecks} command checks");
        anOracle.Report("Draw-lite props and tolerance output against native OCCT");
    }

    /// <summary>
    /// ttranslate, trotate, tscale, tmirror (with and without -copy) and tcopy against native BRepBuilderAPI_Transform
    /// and BRepBuilderAPI_Copy, which the commands call in OCCT: the transformed shapes' extents and properties.
    /// </summary>
    [Fact]
    public void Transformation_commands_answer_as_BRepBuilderAPI_Transform()
    {
        string[] aFiles = ["face1.brep", "wedge_ok.brep", "Room.brep", "wing.brep", "mal_ecrou.brep", "face.brep", "solid.brep", "fuse.brep", "Ball.brep", "Pump_Nut.brep"];
        var anOracle = new Oracle(output);
        int aChecks = 0;
        (string Command, Func<N.gp_Trsf> Trsf)[] aTransforms =
        [
            ("ttranslate s 10 -20 30.5", () => { var t = new N.gp_Trsf(); t.SetTranslation(new N.gp_Vec(10, -20, 30.5)); return t; }),
            ("trotate s 1 2 3 0 1 1 37", () => { var t = new N.gp_Trsf(); t.SetRotation(new N.gp_Ax1(new N.gp_Pnt(1, 2, 3), new N.gp_Dir(0, 1, 1)), 37 * (Math.PI / 180.0)); return t; }),
            ("tscale s 1 -2 3 1.75", () => { var t = new N.gp_Trsf(); t.SetScale(new N.gp_Pnt(1, -2, 3), 1.75); return t; }),
            ("tmirror s 5 0 0 1 1 0", () => { var t = new N.gp_Trsf(); t.SetMirror(new N.gp_Ax2(new N.gp_Pnt(5, 0, 0), new N.gp_Dir(1, 1, 0))); return t; }),
        ];
        foreach (string aFileName in aFiles)
        {
            string aFile = Path.Combine(OcctSourceDirectory, "data", "occ", aFileName);
            var aNS = new N.TopoDS_Shape();
            using (var aBuilder = new N.BRep_Builder())
            {
                Assert.True(N.BRepTools.Read(aNS, aFile, aBuilder), aFile);
            }

            foreach ((string aCommand, Func<N.gp_Trsf> aTrsf) in aTransforms)
            {
                foreach (bool aCopy in new[] { false, true })
                {
                    var aDI = new DrawInterpreter();
                    aDI.LoadDrawResources(ResourcesDirectory);
                    aDI.Eval($"restore {{{TclPath(aFile)}}} s");
                    aDI.Eval(aCommand + (aCopy ? " -copy" : ""));
                    using N.gp_Trsf aT = aTrsf();
                    using var aTransform = new N.BRepBuilderAPI_Transform(aNS, aT, aCopy);
                    CompareShape(anOracle, aDI, aTransform.Shape(), $"{aFileName} {aCommand}{(aCopy ? " -copy" : "")}");
                    aChecks++;
                }
            }

            foreach (bool aCopyGeom in new[] { true, false })
            {
                var aDI = new DrawInterpreter();
                aDI.LoadDrawResources(ResourcesDirectory);
                aDI.Eval($"restore {{{TclPath(aFile)}}} r");
                Assert.Equal("s ", aDI.Eval(aCopyGeom ? "tcopy r s" : "tcopy -n r s"));
                using var aCopy = new N.BRepBuilderAPI_Copy(aNS, aCopyGeom, false);
                CompareShape(anOracle, aDI, aCopy.Shape(), $"{aFileName} tcopy {aCopyGeom}");
                Assert.False(aDI.GetShape("s").IsPartner(aDI.GetShape("r")));
                aChecks++;
            }

            aNS.Dispose();
        }

        output.WriteLine($"{aFiles.Length} files, {aChecks} transformed shapes");
        anOracle.Report("Draw-lite transformation and copy commands against BRepBuilderAPI_Transform and BRepBuilderAPI_Copy");
    }

    // the variable s against the native shape: its extent, its linear, surface and volume properties
    private void CompareShape(Oracle theOracle, DrawInterpreter theDI, N.TopoDS_Shape theNative, string theWhat)
    {
        int[] anExtent = NativeExtent(theNative);
        Assert.Equal(anExtent.Sum(), (int)Numbers(theDI.Eval("nbshapes s"))[^1]);
        foreach (char aKind in "lsv")
        {
            CompareValues(theOracle, NativePropsValues(theNative, aKind, false), Numbers(theDI.Eval($"{aKind}props s -full")), $"{theWhat} {aKind}props");
        }
    }

    // the distinct sub-shapes of a type (TopExp_Explorer into a map of shapes)
    private static int DistinctCount(N.TopoDS_Shape theShape, int theType)
    {
        using var aMap = new N.TopTools_MapOfShape();
        using var anEx = new N.TopExp_Explorer(theShape, (N.TopAbs_ShapeEnum)theType);
        for (; anEx.More(); anEx.Next())
        {
            aMap.Add(anEx.Current());
        }

        return aMap.Extent();
    }
}
#endif
