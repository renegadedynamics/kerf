// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Collections;
using Kerf.Engine.Healing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using System.Diagnostics;
using System.Reflection;
using System.Text;
using System.Text.RegularExpressions;
using Xunit.Abstractions;
using static Kerf.Engine.Exchange.Brep.FormatVersion;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>
/// BRepBuilderAPI_Sewing against native OCCT's, as Draw's <c>sewing result tol shapes</c> sets it up (tolerance, min
/// tolerance tol*1e-4, both at least Precision::Confusion(), max tolerance infinite, other options default).
/// Both sides read the same file (native OCCT writes it in the current format) and sew the same inputs, the port's
/// frozen as callers hold them. Compared through BRep text, written by each side: the sewn shape; compounds of the
/// free, multiple, contiguous and degenerated edges and of the deleted faces, in their order; the compound of
/// <c>Modified</c> of every input and that of <c>ModifiedSubShape</c> of every input edge (with the
/// <c>IsModifiedSubShape</c> flags); and every count. A frozen input sub-shape added to a new shape keeps its Free flag
/// where native OCCT clears it, so texts that differ are compared again with the Free column masked; the test requires
/// every text to match that way and reports the share that is byte-identical as written.
/// </summary>
/// <remarks>
/// Inputs: the faces of each data/occ shape, each copied apart natively (BRepBuilderAPI_Copy) so that no two share a
/// sub-shape, at 1e-6, 1e-2, 1 and 100; and the sewing group's cases (tests/sewing, each grid's tolerance) whose data
/// is in the public dataset, at their grid's tolerance and at 1e-6.
/// </remarks>
public sealed class BRepBuilderAPISewingOracleTests(ITestOutputHelper output) : IDisposable
{
    private readonly string myDirectory = Directory.CreateTempSubdirectory("occtnet-sewing-").FullName;
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

    private static string Metadata(string theKey)
        => typeof(BRepBuilderAPISewingOracleTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>()
                                                        .Single(a => a.Key == theKey).Value!;

    private string NewFile() => Path.Combine(myDirectory, $"s{myFileNumber++}.brep");

    private string NativeText(N.TopoDS_Shape theShape)
    {
        string aFile = NewFile();
        Assert.True(N.BRepTools.Write(theShape, aFile));
        string aText = File.ReadAllText(aFile, Encoding.Latin1);
        File.Delete(aFile);
        return aText;
    }

    private string PortText(in Shape theShape)
    {
        string aFile = NewFile();
        Assert.True(BrepTools.Write(theShape, aFile));
        string aText = File.ReadAllText(aFile, Encoding.Latin1);
        File.Delete(aFile);
        return aText;
    }

    // The Free flag is the first of the seven flag digits on their own line.
    private static string MaskFree(string theText) => Regex.Replace(theText, "(?m)^[01]([01]{6})$", "x$1");

    private sealed class Tally(string theWhat)
    {
        public int Cases;
        public int Texts;
        public int Identical;
        public int MaskedEqual;
        public int Values;
        public int ValuesEqual;
        public int CasesIdentical;
        public int Exceptions;
        public int NativeWritten;
        public readonly List<string> WrittenCases = [];
        public readonly List<string> NativeVaries = [];
        public double NativeSeconds;
        public double PortSeconds;
        public readonly List<string> Differences = [];

        public string Report() =>
            $"{theWhat}: {Cases} sewings ({CasesIdentical} with every text byte-identical; {Exceptions} raising on both sides), "
            + $"{Texts} texts: {Identical} byte-identical, {MaskedEqual} equal with the Free column masked, "
            + $"{Texts - Identical - MaskedEqual} different; {Values} counts and flags, {ValuesEqual} equal; "
            + $"{NativeWritten} where native OCCT wrote into its inputs; native {NativeSeconds:F1} s, port {PortSeconds:F1} s";
    }

    private void Note(Tally theTally, string theWhat)
    {
        if (theTally.Differences.Count < 12)
        {
            theTally.Differences.Add(theWhat);
        }
    }

    private bool CompareText(Tally theTally, string theCase, string theWhat, string theNative, string thePort)
    {
        theTally.Texts++;
        if (theNative == thePort)
        {
            theTally.Identical++;
            return true;
        }
        if (MaskFree(theNative) == MaskFree(thePort))
        {
            theTally.MaskedEqual++;
            return false;
        }
        Note(theTally, $"{theCase}: {theWhat} differs\n{FirstDifference(theNative, thePort)}");
        return false;
    }

    // The lines around the first line that differs, on both sides.
    private static string FirstDifference(string theNative, string thePort)
    {
        string[] aN = theNative.Split('\n'), aP = thePort.Split('\n');
        int i = 0;
        while (i < aN.Length && i < aP.Length && aN[i] == aP[i])
        {
            i++;
        }
        int aFrom = Math.Max(0, i - 5);
        return $"first difference at line {i + 1} of {aN.Length} (native) and {aP.Length} (port)\n--- native\n"
               + string.Join("\n", aN.Skip(aFrom).Take(15)) + "\n--- port\n" + string.Join("\n", aP.Skip(aFrom).Take(15));
    }

    private void CompareValue<T>(Tally theTally, string theCase, string theWhat, T theNative, T thePort)
    {
        theTally.Values++;
        if (EqualityComparer<T>.Default.Equals(theNative, thePort))
        {
            theTally.ValuesEqual++;
        }
        else
        {
            Note(theTally, $"{theCase}: {theWhat}: native {theNative}, port {thePort}");
        }
    }

    private static N.TopoDS_Shape NativeCompound(IEnumerable<N.TopoDS_Shape> theShapes)
    {
        var aC = new N.TopoDS_Compound();
        using var aB = new N.BRep_Builder();
        aB.MakeCompound(aC);
        foreach (N.TopoDS_Shape aS in theShapes)
        {
            if (!aS.IsNull())
            {
                aB.Add(aC, aS);
            }
        }
        return aC;
    }

    private static Shape PortCompound(IEnumerable<Shape> theShapes) =>
        Builder.MakeCompound(theShapes.Where(s => !s.IsNull));

    /// <summary>Native OCCT reads the file and writes it in the current format; both sides read that. Null if unreadable.</summary>
    private string? NativeF1(string theFile, bool theExplodeFaces)
    {
        var aShape = new N.TopoDS_Shape();
        using (var aBuilder = new N.BRep_Builder())
        {
            try
            {
                if (!N.BRepTools.Read(aShape, theFile, aBuilder) || aShape.IsNull())
                {
                    return null;
                }
            }
            catch (Exception)
            {
                return null;
            }
        }

        if (theExplodeFaces)
        {
            var aFaces = new List<N.TopoDS_Shape>();
            using var anExp = new N.TopExp_Explorer(aShape, N.TopAbs_ShapeEnum.TopAbs_FACE);
            for (; anExp.More(); anExp.Next())
            {
                using var aCopy = new N.BRepBuilderAPI_Copy(anExp.Current());
                aFaces.Add(aCopy.Shape());
            }
            if (aFaces.Count == 0)
            {
                return null;
            }
            aShape = NativeCompound(aFaces);
        }

        string aF1 = NewFile();
        Assert.True(N.BRepTools.Write(aShape, aF1, true, false, (N.TopTools_FormatVersion)(int)FormatVersion_CURRENT));
        return aF1;
    }

    private static N.TopoDS_Shape NativeRead(string theFile)
    {
        var aShape = new N.TopoDS_Shape();
        using var aBuilder = new N.BRep_Builder();
        Assert.True(N.BRepTools.Read(aShape, theFile, aBuilder), theFile);
        return aShape;
    }

    private static Shape PortRead(string theFile)
    {
        Assert.True(BrepTools.Read(out Shape aShape, theFile), theFile);
        aShape.TShape()!.FreezeGraph();
        return aShape;
    }

    // A fresh native sewing of the file's inputs: the sewn shape's text.
    private string NativeSewedText(string theF1, bool theChildren, double theTolerance, double theMinTol)
    {
        N.TopoDS_Shape aShape = NativeRead(theF1);
        using var aN = new N.BRepBuilderAPI_Sewing();
        aN.Init(theTolerance, true, true, true, false);
        aN.SetMinTolerance(theMinTol);
        aN.SetMaxTolerance(global::Kerf.Engine.Foundation.Precision.Infinite());
        if (theChildren)
        {
            for (var anIt = new N.TopoDS_Iterator(aShape); anIt.More(); anIt.Next())
            {
                aN.Add(anIt.Value());
            }
            GC.KeepAlive(aShape); // native TopoDS_Iterator holds no reference to the shape it iterates (decision 1109)
        }
        else
        {
            aN.Add(aShape);
        }
        aN.Perform();
        return NativeText(aN.SewedShape());
    }

    // A fresh port sewing of the file's inputs: the sewn shape's text.
    private string PortSewedText(string theF1, bool theChildren, double theTolerance, double theMinTol)
    {
        Shape aShape = PortRead(theF1);
        var aP = new Sewing();
        aP.Init(theTolerance, true, true, true, false);
        aP.SetMinTolerance(theMinTol);
        aP.SetMaxTolerance(global::Kerf.Engine.Foundation.Precision.Infinite());
        if (theChildren)
        {
            for (var anIt = new Iterator(aShape); anIt.More(); anIt.Next())
            {
                aP.Add(anIt.Value());
            }
        }
        else
        {
            aP.Add(aShape);
        }
        aP.Perform();
        return PortText(aP.SewedShape());
    }

    // Draw's sewing command: tolerance, min tolerance tol*1e-4, both at least Precision::Confusion().
    private static void Setup(double theTol, out double theTolerance, out double theMinTol)
    {
        theMinTol = theTol * 1e-4;
        theTolerance = Math.Max(theTol, global::Kerf.Engine.Foundation.Precision.Confusion());
        if (theMinTol < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            theMinTol = global::Kerf.Engine.Foundation.Precision.Confusion();
        }
    }

    /// <summary>
    /// Sews the inputs of the file (the children of its compound when <paramref name="theChildren"/>, else the shape)
    /// at the tolerance on both sides and compares everything.
    /// </summary>
    private void Sew(Tally theTally, string theCase, string theF1, bool theChildren, double theTol)
    {
        N.TopoDS_Shape aNShape = NativeRead(theF1);
        Shape aPShape = PortRead(theF1);
        var aNInputs = new List<N.TopoDS_Shape>();
        var aPInputs = new List<Shape>();
        if (theChildren)
        {
            for (var anIt = new N.TopoDS_Iterator(aNShape); anIt.More(); anIt.Next())
            {
                aNInputs.Add(anIt.Value());
            }
            GC.KeepAlive(aNShape); // native TopoDS_Iterator holds no reference to the shape it iterates (decision 1109)
            for (var anIt = new Iterator(aPShape); anIt.More(); anIt.Next())
            {
                aPInputs.Add(anIt.Value());
            }
        }
        else
        {
            aNInputs.Add(aNShape);
            aPInputs.Add(aPShape);
        }
        string aSnapshot = PortText(aPShape);
        string aNSnapshot = MaskFree(NativeText(aNShape));

        Setup(theTol, out double aTolerance, out double aMinTol);
        theTally.Cases++;
        string aCase = $"{theCase} at {theTol}";
        string? aProgress = Environment.GetEnvironmentVariable("OCCTNET_PROGRESS");
        if (aProgress is not null)
        {
            File.AppendAllText(aProgress, $"{DateTime.Now:HH:mm:ss} {aCase} ({aPInputs.Count} inputs)\n");
        }

        var aWatch = Stopwatch.StartNew();
        using var aN = new N.BRepBuilderAPI_Sewing();
        string? aNError = null;
        try
        {
            aN.Init(aTolerance, true, true, true, false);
            aN.SetSameParameterMode(true);
            aN.SetFloatingEdgesMode(false);
            aN.SetFaceMode(true);
            aN.SetMinTolerance(aMinTol);
            aN.SetMaxTolerance(global::Kerf.Engine.Foundation.Precision.Infinite());
            foreach (N.TopoDS_Shape anInput in aNInputs)
            {
                aN.Add(anInput);
            }
            aN.Perform();
        }
        catch (Exception anException)
        {
            aNError = global::Kerf.Engine.Foundation.OcctTypeNames.Of(Refusal.Unwrapped(anException).GetType());
        }
        theTally.NativeSeconds += aWatch.Elapsed.TotalSeconds;
        if (aProgress is not null)
        {
            File.AppendAllText(aProgress, $"   native {aWatch.Elapsed.TotalSeconds:F2} s\n");
        }

        aWatch.Restart();
        var aP = new Sewing();
        string? aPError = null;
        try
        {
            aP.Init(aTolerance, true, true, true, false);
            aP.SetSameParameterMode(true);
            aP.SetFloatingEdgesMode(false);
            aP.SetFaceMode(true);
            aP.SetMinTolerance(aMinTol);
            aP.SetMaxTolerance(global::Kerf.Engine.Foundation.Precision.Infinite());
            foreach (Shape anInput in aPInputs)
            {
                aP.Add(anInput);
            }
            aP.Perform();
        }
        catch (Exception anException)
        {
            aPError = global::Kerf.Engine.Foundation.OcctTypeNames.Of(Refusal.Unwrapped(anException).GetType()) + ": " + Refusal.Unwrapped(anException).Message;
        }
        theTally.PortSeconds += aWatch.Elapsed.TotalSeconds;
        if (aProgress is not null)
        {
            File.AppendAllText(aProgress, $"   port {aWatch.Elapsed.TotalSeconds:F2} s\n");
        }

        if (aNError is not null || aPError is not null)
        {
            CompareValue(theTally, aCase, "exception", aNError is not null, aPError is not null);
            if (aNError is not null && aPError is not null)
            {
                theTally.Exceptions++;
            }
            else
            {
                Note(theTally, $"{aCase}: native {aNError ?? "ok"}, port {aPError ?? "ok"}");
            }
            return;
        }

        // Native OCCT sews in place: where it wrote into its inputs (an edge's tolerance or flags), it answers that
        // they aren't modified, while the port, which never writes a frozen input, answers that they are (the copy
        // it changed). Those answers are then not compared; the texts still are.
        bool isNativeWritten = MaskFree(NativeText(aNShape)) != aNSnapshot;
        if (isNativeWritten)
        {
            theTally.NativeWritten++;
            theTally.WrittenCases.Add(aCase);
        }

        bool isIdentical = true;
        CompareValue(theTally, aCase, "NbFreeEdges", aN.NbFreeEdges(), aP.FreeEdgeCount);
        CompareValue(theTally, aCase, "NbMultipleEdges", aN.NbMultipleEdges(), aP.MultipleEdgeCount);
        CompareValue(theTally, aCase, "NbContigousEdges", aN.NbContigousEdges(), aP.ContigousEdgeCount);
        CompareValue(theTally, aCase, "NbDegeneratedShapes", aN.NbDegeneratedShapes(), aP.DegeneratedShapeCount);
        CompareValue(theTally, aCase, "NbDeletedFaces", aN.NbDeletedFaces(), aP.DeletedFaceCount);

        string aNSewed = NativeText(aN.SewedShape()), aPSewed = PortText(aP.SewedShape());
        if (MaskFree(aNSewed) != MaskFree(aPSewed))
        {
            // Native OCCT's sewn shape can vary from run to run (address-ordered maps; plan 2.4): sew again natively
            // and, if a run differs, compare the port with every native variant; the port must repeat itself. Up to
            // 16 runs, stopping at the port's: tol_0_01/Q5 has two native variants, about even serially, and once
            // all 7 runs of the earlier limit missed the port's (decision 946).
            var aVariants = new List<string> { aNSewed };
            for (int aRun = 0; aRun < 15 && !aVariants.Any(v => MaskFree(v) == MaskFree(aPSewed)); aRun++)
            {
                string aText = NativeSewedText(theF1, theChildren, aTolerance, aMinTol);
                if (!aVariants.Any(v => MaskFree(v) == MaskFree(aText)))
                {
                    aVariants.Add(aText);
                }
            }
            string aPAgain = PortSewedText(theF1, theChildren, aTolerance, aMinTol);
            CompareValue(theTally, aCase, "port repeats its sewn shape", aPSewed, aPAgain);
            if (aVariants.Count > 1)
            {
                string? aMatch = aVariants.FirstOrDefault(v => MaskFree(v) == MaskFree(aPSewed));
                theTally.NativeVaries.Add($"{aCase} ({aVariants.Count} native variants; the port's is {(aMatch is null ? "none" : "one")} of them)");
                if (aMatch is not null)
                {
                    aNSewed = aMatch;
                }
            }
        }
        isIdentical &= CompareText(theTally, aCase, "SewedShape", aNSewed, aPSewed);

        if (aN.NbFreeEdges() == aP.FreeEdgeCount)
        {
            isIdentical &= CompareText(theTally, aCase, "free edges",
                                       NativeText(NativeCompound(Enumerable.Range(1, aN.NbFreeEdges()).Select(i => (N.TopoDS_Shape)aN.FreeEdge(i)))),
                                       PortText(PortCompound(Enumerable.Range(1, aP.FreeEdgeCount).Select(i => (Shape)aP.FreeEdge(i)))));
        }
        if (aN.NbMultipleEdges() == aP.MultipleEdgeCount && aN.NbMultipleEdges() > 0)
        {
            isIdentical &= CompareText(theTally, aCase, "multiple edges",
                                       NativeText(NativeCompound(Enumerable.Range(1, aN.NbMultipleEdges()).Select(i => (N.TopoDS_Shape)aN.MultipleEdge(i)))),
                                       PortText(PortCompound(Enumerable.Range(1, aP.MultipleEdgeCount).Select(i => (Shape)aP.MultipleEdge(i)))));
        }
        if (aN.NbContigousEdges() == aP.ContigousEdgeCount)
        {
            isIdentical &= CompareText(theTally, aCase, "contiguous edges",
                                       NativeText(NativeCompound(Enumerable.Range(1, aN.NbContigousEdges()).Select(i => (N.TopoDS_Shape)aN.ContigousEdge(i)))),
                                       PortText(PortCompound(Enumerable.Range(1, aP.ContigousEdgeCount).Select(i => (Shape)aP.ContigousEdge(i)))));
            int aNBound = 0, aPBound = 0;
            for (int i = 1; i <= aN.NbContigousEdges(); i++)
            {
                aNBound += aN.ContigousEdgeCouple(i).Count(s => aN.IsSectionBound(N.TopoDS.Edge(s)));
                aPBound += aP.ContigousEdgeCouple(i).Count(s => aP.IsSectionBound(ShapeCasts.Edge(s)));
            }
            CompareValue(theTally, aCase, "bound sections", aNBound, aPBound);
        }
        if (aN.NbDegeneratedShapes() == aP.DegeneratedShapeCount && aN.NbDegeneratedShapes() > 0)
        {
            isIdentical &= CompareText(theTally, aCase, "degenerated shapes",
                                       NativeText(NativeCompound(Enumerable.Range(1, aN.NbDegeneratedShapes()).Select(i => aN.DegeneratedShape(i)))),
                                       PortText(PortCompound(Enumerable.Range(1, aP.DegeneratedShapeCount).Select(i => aP.DegeneratedShape(i)))));
        }
        if (aN.NbDeletedFaces() == aP.DeletedFaceCount && aN.NbDeletedFaces() > 0)
        {
            isIdentical &= CompareText(theTally, aCase, "deleted faces",
                                       NativeText(NativeCompound(Enumerable.Range(1, aN.NbDeletedFaces()).Select(i => (N.TopoDS_Shape)aN.DeletedFace(i)))),
                                       PortText(PortCompound(Enumerable.Range(1, aP.DeletedFaceCount).Select(i => (Shape)aP.DeletedFace(i)))));
        }

        // Modified of every input, with IsModified.
        var aNMod = new List<N.TopoDS_Shape>();
        var aPMod = new List<Shape>();
        for (int i = 0; i < aNInputs.Count; i++)
        {
            if (!isNativeWritten)
            {
                CompareValue(theTally, aCase, $"IsModified({i})", aN.IsModified(aNInputs[i]), aP.IsModified(aPInputs[i]));
            }
            aNMod.Add(aN.Modified(aNInputs[i]));
            aPMod.Add(aP.Modified(aPInputs[i]));
        }
        isIdentical &= CompareText(theTally, aCase, "Modified", NativeText(NativeCompound(aNMod)), PortText(PortCompound(aPMod)));

        // ModifiedSubShape of every input edge (TopExp::MapShapes order, the same on both sides), with IsModifiedSubShape.
        using var aNEdges = new N.TopTools_IndexedMapOfShape();
        N.TopExp.MapShapes(aNShape, N.TopAbs_ShapeEnum.TopAbs_EDGE, aNEdges);
        var aPEdges = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(aPShape, ShapeType.Edge, aPEdges);
        CompareValue(theTally, aCase, "input edges", aNEdges.Extent(), aPEdges.Count);
        if (aNEdges.Extent() == aPEdges.Count)
        {
            var aNSub = new List<N.TopoDS_Shape>();
            var aPSub = new List<Shape>();
            int aNModified = 0, aPModified = 0;
            for (int i = 1; i <= aNEdges.Extent(); i++)
            {
                using N.TopoDS_Shape anE = aNEdges.FindKey(i);
                if (aN.IsModifiedSubShape(anE))
                {
                    aNModified++;
                }
                if (aP.IsModifiedSubShape(aPEdges[i - 1]))
                {
                    aPModified++;
                }
                aNSub.Add(aN.ModifiedSubShape(anE));
                aPSub.Add(aP.ModifiedSubShape(aPEdges[i - 1]));
            }
            if (!isNativeWritten)
            {
                CompareValue(theTally, aCase, "IsModifiedSubShape count", aNModified, aPModified);
            }
            CompareValue(theTally, aCase, "null ModifiedSubShape", aNSub.Count(s => s.IsNull()), aPSub.Count(s => s.IsNull));
            isIdentical &= CompareText(theTally, aCase, "ModifiedSubShape", NativeText(NativeCompound(aNSub)), PortText(PortCompound(aPSub)));
        }

        // The inputs are never written.
        CompareValue(theTally, aCase, "input unchanged", aSnapshot, PortText(aPShape));
        if (isIdentical)
        {
            theTally.CasesIdentical++;
        }
    }

    private void Finish(params Tally[] theTallies)
    {
        foreach (Tally aTally in theTallies)
        {
            output.WriteLine(aTally.Report());
            if (aTally.WrittenCases.Count > 0)
            {
                output.WriteLine("  native OCCT wrote into its inputs in: " + string.Join(", ", aTally.WrittenCases));
            }
            if (aTally.NativeVaries.Count > 0)
            {
                output.WriteLine("  native OCCT varies from run to run in: " + string.Join(", ", aTally.NativeVaries));
            }
            foreach (string aD in aTally.Differences)
            {
                output.WriteLine(aD);
            }
        }
        foreach (Tally aTally in theTallies)
        {
            Assert.Equal(aTally.Values, aTally.ValuesEqual);
            Assert.Equal(aTally.Texts, aTally.Identical + aTally.MaskedEqual);
        }
    }

    private const int MaxFaces = 3000;

    [Fact]
    public void Exploded_faces_of_the_data_occ_shapes_are_sewn_as_native_OCCT_sews_them()
    {
        string[] aFiles = Directory.GetFiles(Path.Combine(Metadata("OcctSourceDir"), "data", "occ"), "*.brep")
                                   .Order(StringComparer.Ordinal).ToArray();
        var aTally = new Tally("data/occ exploded faces");
        int aSkipped = 0;
        foreach (string aFile in Tape.Budget(aFiles, isSpread: true))
        {
            string? aF1 = NativeF1(aFile, true);
            if (aF1 is null)
            {
                aSkipped++;
                continue;
            }
            int aNbFaces = 0;
            for (var anIt = new Iterator(PortRead(aF1)); anIt.More(); anIt.Next())
            {
                aNbFaces++;
            }
            if (aNbFaces > MaxFaces)
            {
                aSkipped++;
                continue;
            }
            foreach (double aTol in (double[])[1e-6, 1e-2, 1.0, 100.0])
            {
                Sew(aTally, Path.GetFileName(aFile), aF1, true, aTol);
            }
            File.Delete(aF1);
        }
        output.WriteLine($"{aFiles.Length} files, {aSkipped} without faces or with more than {MaxFaces} faces");
        Finish(aTally);
    }

    /// <summary>The sewing group's cases: (grid, case, data file, the grid's tolerance).</summary>
    private static List<(string Case, string Data, double Tol)> SewingCases(string theDataRoot)
    {
        var aFilesByName = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
        foreach (string aFile in Directory.EnumerateFiles(theDataRoot, "*.*", SearchOption.AllDirectories))
        {
            aFilesByName.TryAdd(Path.GetFileName(aFile), aFile);
        }

        string aGroup = Path.Combine(Metadata("OcctSourceDir"), "tests", "sewing");
        var aCases = new List<(string, string, double)>();
        foreach (string aGrid in Directory.GetDirectories(aGroup).Order(StringComparer.Ordinal))
        {
            Match aTolMatch = Regex.Match(File.ReadAllText(Path.Combine(aGrid, "begin")), @"set tol ([0-9.eE+-]+)");
            double aTol = double.Parse(aTolMatch.Groups[1].Value, System.Globalization.CultureInfo.InvariantCulture);
            foreach (string aCase in Directory.GetFiles(aGrid).Order(StringComparer.Ordinal))
            {
                if (Path.GetFileName(aCase) == "begin")
                {
                    continue;
                }
                MatchCollection aRestores = Regex.Matches(File.ReadAllText(aCase), @"restore \[locate_data_file ([^\]]+)\] a\b");
                if (aRestores.Count == 1 && aFilesByName.TryGetValue(aRestores[0].Groups[1].Value.Trim(), out string? aData)
                    && (aData.EndsWith(".rle", StringComparison.OrdinalIgnoreCase) || aData.EndsWith(".brep", StringComparison.OrdinalIgnoreCase)))
                {
                    aCases.Add(($"{Path.GetFileName(aGrid)}/{Path.GetFileName(aCase)}", aData, aTol));
                }
            }
        }
        return aCases;
    }

    [Fact]
    public void The_sewing_group_cases_are_sewn_as_native_OCCT_sews_them()
    {
        string aRoot = Metadata("OcctTestDataDir");
        if (string.IsNullOrEmpty(aRoot) || !Directory.Exists(aRoot))
        {
            output.WriteLine("OcctTestDataDir isn't set: nothing to check");
            return;
        }

        List<(string Case, string Data, double Tol)> aCases = SewingCases(aRoot);
        var aTally = new Tally("sewing group at the grids' tolerances");
        var aTight = new Tally("sewing group's data at 1e-6");
        var aSeenData = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
        int anUnreadable = 0;
        foreach ((string aCase, string aData, double aTol) in Tape.Budget(aCases, isSpread: true))
        {
            string? aF1 = NativeF1(aData, false);
            if (aF1 is null)
            {
                anUnreadable++;
                continue;
            }
            Sew(aTally, aCase, aF1, false, aTol);
            if (aSeenData.Add(aData))
            {
                Sew(aTight, Path.GetFileName(aData), aF1, false, 1e-6);
            }
            File.Delete(aF1);
        }
        output.WriteLine($"{aCases.Count} cases with public data, {anUnreadable} unreadable");
        Finish(aTally, aTight);
    }
}
