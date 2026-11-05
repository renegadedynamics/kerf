// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using System.Diagnostics;
using System.Reflection;
using System.Text.RegularExpressions;
using Xunit.Abstractions;
using static Kerf.Topology.ShapeType;
using static Kerf.Engine.Exchange.Brep.FormatVersion;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>
/// ShapeAnalysis_Shell and ShapeAnalysis_FreeBounds (the shell form, and the static ConnectEdgesToWires by distance)
/// against native OCCT on the shells of the corpus.
/// </summary>
/// <remarks>
/// <para>
/// The shapes are data/occ's .brep files, the test dataset's (up to 1 MB and 50,000 sub-shape occurrences) and 72
/// random native shapes (BRepToolsIOOracleTests' scenes), each also with every third face removed, so that the free
/// bounds aren't empty. Native OCCT writes each shape to a file that both sides read, so both start from the same
/// shape.
/// </para>
/// <para>
/// Compared: CheckOrientedShells' result (with and without INTERNAL edges counted as connections), HasBadEdges,
/// HasFreeEdges, HasConnectedEdges, NbLoaded, and the BadEdges and FreeEdges compounds; FreeBounds' closed and open
/// compounds for the four split options and, in its tolerance form (sewing at 10^-6 and 10^-2), with the default ones; ConnectEdgesToWires on the free edges by distance (10^-4), which builds new
/// vertices and edges. Shapes are compared as BRep text (format 3, with triangulations), the Free flag masked: a frozen
/// sub-shape keeps the Free flag it was frozen with where native OCCT clears it (open question 1).
/// </para>
/// </remarks>
public class ShapeAnalysisFreeBoundsOracleTests(ITestOutputHelper output) : IDisposable
{
    private readonly string myDirectory = Directory.CreateTempSubdirectory("occtnet-safb-").FullName;
    private int myFileNumber;
    private readonly List<string> myDifferences = [];
    private int myCompared, myIdentical, myFreeMasked, myExceptions;

    public void Dispose()
    {
        Directory.Delete(myDirectory, true);
        GC.SuppressFinalize(this);
    }

    private static string Metadata(string theKey)
        => typeof(ShapeAnalysisFreeBoundsOracleTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == theKey).Value!;

    private string NewFile() => Path.Combine(myDirectory, $"s{myFileNumber++}.brep");

    private string NativeText(N.TopoDS_Shape theShape)
    {
        string aFile = NewFile();
        Assert.True(N.BRepTools.Write(theShape, aFile, true, false, N.TopTools_FormatVersion.TopTools_FormatVersion_VERSION_3));
        string aText = File.ReadAllText(aFile);
        File.Delete(aFile);
        return aText;
    }

    private string PortText(in Shape theShape)
    {
        string aFile = NewFile();
        Assert.True(BrepTools.Write(theShape, aFile, true, false, FormatVersion_VERSION_3));
        string aText = File.ReadAllText(aFile);
        File.Delete(aFile);
        return aText;
    }

    private static readonly Regex theFlags = new("(?m)^[01]([01]{6})\r?$");

    // Compares the BRep texts, the Free column of the flag lines masked (and counted where it differs).
    private void SameShape(N.TopoDS_Shape theN, in Shape theP, string theWhat)
    {
        myCompared++;
        if (theN.IsNull() || theP.IsNull)
        {
            if (theN.IsNull() != theP.IsNull)
            {
                myDifferences.Add($"{theWhat}: null {theN.IsNull()} natively, {theP.IsNull} in the port");
            }
            else
            {
                myIdentical++;
            }
            return;
        }

        string aN = NativeText(theN), aP = PortText(theP);
        if (aN == aP)
        {
            myIdentical++;
            return;
        }
        string aNM = theFlags.Replace(aN, "x$1"), aPM = theFlags.Replace(aP, "x$1");
        if (aNM == aPM)
        {
            myIdentical++;
            myFreeMasked++;
            return;
        }
        myDifferences.Add($"{theWhat}: BRep texts differ");
    }

    private void SameValue(int theN, int theP, string theWhat)
    {
        myCompared++;
        if (theN == theP)
        {
            myIdentical++;
        }
        else
        {
            myDifferences.Add($"{theWhat}: {theN} natively, {theP} in the port");
        }
    }

    private static int B(bool theValue) => theValue ? 1 : 0;

    /// <summary>Native writes the shape; both read it. Null if native OCCT can't write or read it.</summary>
    private (N.TopoDS_Shape N, Shape P)? Read(N.TopoDS_Shape theShape)
    {
        string aFile = NewFile();
        if (!N.BRepTools.Write(theShape, aFile, true, false, N.TopTools_FormatVersion.TopTools_FormatVersion_VERSION_3))
        {
            return null;
        }
        var aNShape = new N.TopoDS_Shape();
        using (var aBuilder = new N.BRep_Builder())
        {
            if (!N.BRepTools.Read(aNShape, aFile, aBuilder))
            {
                return null;
            }
        }
        Assert.True(BrepTools.Read(out Shape aPShape, aFile), aFile);
        File.Delete(aFile);
        return (aNShape, aPShape);
    }

    // A native compound of the shape's faces but every third.
    private static N.TopoDS_Shape Holed(N.TopoDS_Shape theShape)
    {
        var aC = new N.TopoDS_Compound();
        using var aB = new N.BRep_Builder();
        aB.MakeCompound(aC);
        int i = 0;
        for (var anExp = new N.TopExp_Explorer(theShape, N.TopAbs_ShapeEnum.TopAbs_FACE); anExp.More(); anExp.Next(), i++)
        {
            if (i % 3 != 1)
            {
                aB.Add(aC, anExp.Current());
            }
        }
        return aC;
    }

    private static T? Try<T>(Func<T> theFunc, out string? theError) where T : class
    {
        try
        {
            theError = null;
            return theFunc();
        }
        catch (N.OcctException anException)
        {
            // "Standard_NullObject: message": the OCCT type
            string aMessage = anException.Message;
            int aColon = aMessage.IndexOf(':');
            theError = aColon > 0 ? aMessage[..aColon] : aMessage;
            return null;
        }
        catch (Exception anException) when (Refusal.IsFailure(anException))
        {
            theError = global::Kerf.Engine.Foundation.OcctTypeNames.Of(Refusal.Unwrapped(anException).GetType());
            return null;
        }
    }

    // True (the comparison done) if either side threw: both must have thrown the same OCCT exception.
    private bool Failed(string? theNError, string? thePError, string theWhat)
    {
        if (theNError is null && thePError is null)
        {
            return false;
        }
        myCompared++;
        if (theNError == thePError)
        {
            myIdentical++;
            myExceptions++;
        }
        else
        {
            myDifferences.Add($"{theWhat}: {theNError ?? "no exception"} natively, {thePError ?? "no exception"} in the port");
        }
        return true;
    }

    private void Compare(N.TopoDS_Shape theShape, string theWhat)
    {
        if (Read(theShape) is not (N.TopoDS_Shape aNS, Shape aPS))
        {
            return;
        }

        // ShapeAnalysis_Shell, both modes
        foreach (bool anInternal in new[] { false, true })
        {
            string w = $"{theWhat}, Shell (internal {anInternal})";
            using var aNSas = new N.ShapeAnalysis_Shell();
            var aPSas = new ShellAnalysis();
            object? aNRes = Try<object>(() => aNSas.CheckOrientedShells(aNS, true, anInternal), out string? aNErr);
            object? aPRes = Try<object>(() => aPSas.CheckOrientedShells(aPS, true, anInternal), out string? aPErr);
            if (Failed(aNErr, aPErr, w))
            {
                continue;
            }
            SameValue(B((bool)aNRes!), B((bool)aPRes!), w + ", result");
            SameValue(B(aNSas.HasBadEdges()), B(aPSas.HasBadEdges), w + ", HasBadEdges");
            SameValue(B(aNSas.HasFreeEdges()), B(aPSas.HasFreeEdges), w + ", HasFreeEdges");
            SameValue(B(aNSas.HasConnectedEdges()), B(aPSas.HasConnectedEdges), w + ", HasConnectedEdges");
            SameValue(aNSas.NbLoaded(), aPSas.LoadedCount, w + ", NbLoaded");
            SameShape(aNSas.BadEdges(), aPSas.BadEdges(), w + ", BadEdges");
            SameShape(aNSas.FreeEdges(), aPSas.FreeEdges(), w + ", FreeEdges");

            if (!anInternal && aNSas.HasFreeEdges())
            {
                // the free edges connected by distance: new vertices and edges where the ends aren't shared
                var aNSeq = new N.TopTools_HSequenceOfShape();
                for (var anExp = new N.TopExp_Explorer(aNSas.FreeEdges(), N.TopAbs_ShapeEnum.TopAbs_EDGE); anExp.More(); anExp.Next())
                {
                    aNSeq.Append(anExp.Current());
                }
                var aPSeq = new NCollection_HSequence<Shape>();
                for (var anExp = new Explorer(aPSas.FreeEdges(), ShapeType.Edge); anExp.More(); anExp.Next())
                {
                    aPSeq.Append(anExp.Current());
                }
                N.TopTools_HSequenceOfShape? aNW = Try(() => N.ShapeAnalysis_FreeBounds.ConnectEdgesToWires(aNSeq, 1e-4, false)!, out aNErr);
                NCollection_HSequence<Shape>? aPW = Try(() => FreeBounds.ConnectEdgesToWires(aPSeq, 1e-4, false), out aPErr);
                if (Failed(aNErr, aPErr, w + ", ConnectEdgesToWires"))
                {
                    continue;
                }
                SameValue(aNW!.Length(), aPW!.Length(), w + ", ConnectEdgesToWires count");
                for (int i = 1; i <= Math.Min(aNW.Length(), aPW!.Length()); i++)
                {
                    SameShape(aNW.Value(i), aPW.Value(i), w + $", ConnectEdgesToWires wire {i}");
                    SameValue(B(aNW.Value(i).Closed()), B(aPW.Value(i).ClosedFlag), w + $", ConnectEdgesToWires wire {i} closed");
                }
                for (int i = 1; i <= aNSeq.Length(); i++)
                {
                    // the edges it reversed, reversed in the caller's sequence
                    SameValue((int)aNSeq.Value(i).Orientation(), (int)aPSeq.Value(i).Orientation, w + $", edge {i} orientation after");
                }
            }
        }

        // ShapeAnalysis_FreeBounds, the shell form, with the four split options
        foreach ((bool aSplitClosed, bool aSplitOpen) in new[] { (false, true), (false, false), (true, false), (true, true) })
        {
            string w = $"{theWhat}, FreeBounds ({aSplitClosed}, {aSplitOpen})";
            using N.ShapeAnalysis_FreeBounds? aNFb = Try(() => new N.ShapeAnalysis_FreeBounds(aNS, aSplitClosed, aSplitOpen), out string? aNErr);
            FreeBounds? aPFb = Try(() => new FreeBounds(aPS, aSplitClosed, aSplitOpen), out string? aPErr);
            if (Failed(aNErr, aPErr, w))
            {
                continue;
            }
            SameShape(aNFb!.GetClosedWires(), aPFb!.GetClosedWires(), w + ", closed wires");
            SameShape(aNFb.GetOpenWires(), aPFb.GetOpenWires(), w + ", open wires");
        }
        // the tolerance form: the free edges sewing leaves, connected by distance
        foreach (double aTol in new[] { 1e-6, 1e-2 })
        {
            string w = $"{theWhat}, FreeBounds (tolerance {aTol})";
            using N.ShapeAnalysis_FreeBounds? aNFb = Try(() => new N.ShapeAnalysis_FreeBounds(aNS, aTol, false, true), out string? aNErr);
            FreeBounds? aPFb = Try(() => new FreeBounds(aPS, aTol, false, true), out string? aPErr);
            if (Failed(aNErr, aPErr, w))
            {
                continue;
            }
            SameShape(aNFb!.GetClosedWires(), aPFb!.GetClosedWires(), w + ", closed wires");
            SameShape(aNFb.GetOpenWires(), aPFb.GetOpenWires(), w + ", open wires");
        }
        using N.ShapeAnalysis_FreeBounds? aNFbI = Try(() => new N.ShapeAnalysis_FreeBounds(aNS, false, true, true), out string? aNErrI);
        FreeBounds? aPFbI = Try(() => new FreeBounds(aPS, false, true, true), out string? aPErrI);
        if (Failed(aNErrI, aPErrI, theWhat + ", FreeBounds (internal edges)"))
        {
            return;
        }
        SameShape(aNFbI!.GetClosedWires(), aPFbI!.GetClosedWires(), theWhat + ", FreeBounds (internal edges), closed wires");
        SameShape(aNFbI.GetOpenWires(), aPFbI.GetOpenWires(), theWhat + ", FreeBounds (internal edges), open wires");
    }

    private void Run(N.TopoDS_Shape theShape, string theWhat)
    {
        Compare(theShape, theWhat);
        Compare(Holed(theShape), theWhat + " holed");
    }

    private void Report(string theWhat)
    {
        output.WriteLine($"{theWhat}: {myIdentical}/{myCompared} values and shapes identical ({myFreeMasked} with the Free column masked, {myExceptions} the same exception on both sides), "
                         + $"{myDifferences.Count} different");
        foreach (string aD in myDifferences.Take(40))
        {
            output.WriteLine("  " + aD);
        }
    }

    private static int Occurrences(N.TopoDS_Shape theShape, int theLimit)
    {
        int aCount = 0;
        var aStack = new Stack<N.TopoDS_Shape>();
        aStack.Push(theShape);
        while (aStack.Count > 0 && aCount <= theLimit)
        {
            N.TopoDS_Shape aShape = aStack.Pop();
            aCount++;
            for (var anIt = new N.TopoDS_Iterator(aShape); anIt.More(); anIt.Next())
            {
                aStack.Push(anIt.Value());
            }
            GC.KeepAlive(aShape); // native TopoDS_Iterator holds no reference to the shape it iterates (decision 1109)
        }
        return aCount;
    }

    private static N.TopoDS_Shape? NativeRead(string theFile)
    {
        var aShape = new N.TopoDS_Shape();
        using var aBuilder = new N.BRep_Builder();
        try
        {
            return N.BRepTools.Read(aShape, theFile, aBuilder) && !aShape.IsNull() ? aShape : null;
        }
        catch (Exception)
        {
            return null;
        }
    }

    [Fact]
    public void Shell_and_FreeBounds_match_native_OCCT_on_the_corpus()
    {
        var aWatch = Stopwatch.StartNew();
        int aShapes = 0;

        foreach (string aFile in Tape.Budget(Directory.GetFiles(Path.Combine(Metadata("OcctSourceDir"), "data", "occ"), "*.brep")
                                                      .Order(StringComparer.Ordinal), isSpread: true))
        {
            if (NativeRead(aFile) is N.TopoDS_Shape aShape)
            {
                Run(aShape, "data/occ/" + Path.GetFileName(aFile));
                aShapes++;
            }
        }

        string aRoot = Metadata("OcctTestDataDir");
        if (!string.IsNullOrEmpty(aRoot) && Directory.Exists(aRoot))
        {
            foreach (string aFile in Tape.Budget(Directory.EnumerateFiles(aRoot, "*.brep", SearchOption.AllDirectories)
                                                          .Order(StringComparer.Ordinal), isSpread: true))
            {
                if (new FileInfo(aFile).Length > 1_000_000 || NativeRead(aFile) is not N.TopoDS_Shape aShape
                    || Occurrences(aShape, 50_000) > 50_000)
                {
                    continue;
                }
                Run(aShape, Path.GetRelativePath(aRoot, aFile));
                aShapes++;
            }
        }
        else
        {
            output.WriteLine("OcctTestDataDir isn't set: data/occ only");
        }

        var anOracle = new Oracle(output, 61101);
        for (int i = 0; i < 72; i++)
        {
            N.TopoDS_Shape aShape = BRepToolsIOOracleTests.NativeShape(anOracle, i, out string aWhat);
            Run(aShape, $"scene {i} ({aWhat})");
            aShapes++;
        }

        output.WriteLine($"{aShapes} shapes, {aWatch.Elapsed.TotalSeconds:F0} s");
        Report("ShapeAnalysis_Shell and ShapeAnalysis_FreeBounds");
        Assert.Empty(myDifferences);
    }
}
