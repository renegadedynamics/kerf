// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Brep;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Globalization;
using System.IO.Compression;
using System.Text;
using OcctNet.ModelingData.Tests;
using OcctNet.TestHarness;
using Xunit.Abstractions;
using static Kerf.Topology.ShapeType;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Unit 7G's algorithms that build123d.NET doesn't bind against native OCCT's recordings (tools/draftrec):
/// BRepOffsetAPI_MakeDraft (wires, faces and an open shell; by length, up to a plane and up to a shape; internal and
/// round-corner variants), BiTgte_Blend (pairs of faces of a box, of a Boolean result and bug29861's cylinders, with and
/// without the resulting shape) and BRepOffsetAPI_MakeEvolved's volume mode (BRepFill_AdvancedEvolved, four spines and
/// three profiles, solid and not). The port reads each input frozen, as a caller's, and compares the exception, the
/// statuses, and the results' BRep text: bytes, then the Free column masked, then canonically. Every input is checked
/// unchanged and every result frozen.
/// </summary>
public class Draft7GNativeTests(ITestOutputHelper output)
{
    private static readonly HashSet<string> s_blocks = ["INPUT", "DONE", "SHELL"];

    private static List<(string Head, string Body)> Records()
    {
        using Stream aRes = typeof(Draft7GNativeTests).Assembly.GetManifestResourceStream("Draft.draftrec.txt.gz")
                            ?? throw new InvalidOperationException("Draft.draftrec.txt.gz");
        using GZipStream aGz = new(aRes, CompressionMode.Decompress);
        using StreamReader aReader = new(aGz, Encoding.Latin1);
        var aRecords = new List<(string, string)>();
        string? aLine;
        while ((aLine = aReader.ReadLine()) is not null)
        {
            string aKeyword = aLine.Split(' ')[0];
            var aBody = new StringBuilder();
            if (s_blocks.Contains(aKeyword))
            {
                int aNb = int.Parse(aLine.Split(' ')[^1], CultureInfo.InvariantCulture);
                for (int k = 0; k < aNb; k++)
                {
                    aBody.Append(aReader.ReadLine()).Append('\n');
                }
            }
            aRecords.Add((aLine, aBody.ToString()));
        }
        return aRecords;
    }

    private static double FromHex(string theHex) =>
        BitConverter.Int64BitsToDouble(long.Parse(theHex, NumberStyles.HexNumber, CultureInfo.InvariantCulture));

    private static Shape Read(string theText)
    {
        BrepTools.Read(out Shape aShape, new StringReader(theText));
        aShape.TShape()?.FreezeGraph(); // a caller's shape
        return aShape;
    }

    private static string Text(in Shape theS)
    {
        var aS = new StringWriter(CultureInfo.InvariantCulture) { NewLine = "\n" };
        BrepTools.Write(theS, aS, true, false, FormatVersion.FormatVersion_VERSION_3);
        return aS.ToString();
    }

    private sealed class Tally
    {
        public int Runs, Identical, FreeMasked, Canonical, Attributed, Empty, SameThrow, NotFrozen, InputsChanged;
        public readonly List<string> Differences = [];
    }

    private static void CompareText(Tally theTally, string theWhat, string theNative, in Shape thePort)
    {
        if (!thePort.IsNull && !thePort.TShape()!.IsFrozen)
        {
            theTally.NotFrozen++;
            theTally.Differences.Add($"{theWhat}: the port's result isn't frozen");
        }
        string aPort = thePort.IsNull ? "" : Text(thePort);
        if (aPort.Length > 0 && aPort[^1] != '\n')
        {
            aPort += "\n"; // the recording ends every block with a newline
        }
        if (theNative.Length == 0 || aPort.Length == 0)
        {
            if (theNative.Length != aPort.Length)
            {
                theTally.Differences.Add($"{theWhat}: native {(theNative.Length == 0 ? "null" : "a shape")}, port {(aPort.Length == 0 ? "null" : "a shape")}");
            }
            else
            {
                theTally.Empty++;
            }
            return;
        }
        BRepTextDiff aDiff = BRepTextComparison.Compare(theNative, aPort);
        if (aDiff.IsIdentical)
        {
            theTally.Identical++;
            return;
        }
        if (aDiff.IsIdenticalMaskingFree)
        {
            theTally.FreeMasked++;
            return;
        }
        BrepTools.Read(out Shape aN, new StringReader(theNative));
        CanonicalComparison aCanon = CanonicalShapeComparer.Compare(aN, thePort, new CanonicalOptions { CheckValidity = false });
        if (aCanon.IsIdentical)
        {
            theTally.Canonical++;
            return;
        }
        if (Environment.GetEnvironmentVariable("OCCTNET_DRAFTREC_DUMP") is string aDir && theTally.Differences.Count == 0)
        {
            File.WriteAllText(Path.Combine(aDir, "native.brep"), theNative);
            File.WriteAllText(Path.Combine(aDir, "port.brep"), aPort);
        }
        // Attributed (decision 1050): the drafts of the circle and the disk up to the stop box: the same numbers of
        // vertices, edges, wires and faces and the same curves, but the Boolean of BRepFill_Draft::Fuse splits the
        // circle at its seam differently (which vertex the seam's pieces take, and one wire's Closed flag).
        if (theWhat.StartsWith("DRAFT circle-wire ", StringComparison.Ordinal) || theWhat.StartsWith("DRAFT disk-face ", StringComparison.Ordinal))
        {
            Assert.Contains(" T 0 0 stop-box DONE", theWhat, StringComparison.Ordinal);
            CanonicalComparison aCounts = CanonicalShapeComparer.Compare(aN, thePort, new CanonicalOptions { CheckValidity = false });
            Assert.True(aCounts.CountsEqual, aCounts.ToString());
            theTally.Attributed++;
            return;
        }
        theTally.Differences.Add($"{theWhat}: {aDiff}; canonically {aCanon}");
    }

    [Fact]
    public void MakeDraft_BiTgte_Blend_and_AdvancedEvolved_as_native()
    {
        List<(string Head, string Body)> aRecords = Records();
        var anInputs = new Dictionary<string, string>(StringComparer.Ordinal);
        var aTallies = new SortedDictionary<string, Tally>(StringComparer.Ordinal);
        for (int r = 0; r < aRecords.Count; r++)
        {
            (string aHead, string aBody) = aRecords[r];
            string[] w = aHead.Split(' ');
            if (w[0] == "INPUT")
            {
                anInputs[w[1]] = aBody;
                continue;
            }
            Tally aTally = aTallies.TryGetValue(w[0], out Tally? t) ? t : aTallies[w[0]] = new Tally();
            aTally.Runs++;
            // the native records that follow the header, up to the next header
            var aNative = new List<(string Head, string Body)>();
            while (r + 1 < aRecords.Count && aRecords[r + 1].Head.Split(' ')[0] is "DONE" or "SHELL" or "SURFACES" or "THROW")
            {
                aNative.Add(aRecords[++r]);
            }
            var aSnaps = new List<(Shape, ShapeSnapshot)>();
            Shape Input(string theName)
            {
                Shape aS = Read(anInputs[theName]);
                aSnaps.Add((aS, ShapeSnapshot.Of(aS)));
                return aS;
            }
            var aPort = new List<(string Head, Shape? Shape)>();
            try
            {
                switch (w[0])
                {
                    case "DRAFT":
                    {
                        var aMk = new BRepOffsetAPI_MakeDraft(Input(w[1]), new Direction3D(0, 0, 1), FromHex(w[2]));
                        if (w[5] == "1")
                        {
                            aMk.SetOptions(BRepBuilderAPI_TransitionMode.BRepBuilderAPI_RoundCorner);
                        }
                        if (w[4] == "1")
                        {
                            aMk.SetDraft(true);
                        }
                        switch (w[3])
                        {
                            case "L":
                                aMk.Perform(FromHex(w[6]));
                                break;
                            case "S":
                                aMk.Perform(new PlaneSurface(new Point3D(0, 0, FromHex(w[6])), new Direction3D(0, 0, 1)), true);
                                break;
                            default:
                                aMk.Perform(Input(w[6]), true);
                                break;
                        }
                        Shape aShape = aMk.Shape();
                        aPort.Add(("DONE " + (aMk.IsDone() ? "1" : "0"), aShape));
                        aPort.Add(("SHELL", aMk.Shell()));
                        break;
                    }
                    case "ROLL":
                    {
                        Shape aS = Input(w[1]);
                        var aFaces = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
                        Exploration.MapShapes(aS, ShapeType.Face, aFaces);
                        var aRoll = new BiTgte_Blend();
                        aRoll.Init(aS, FromHex(w[2]), 1.0e-4, false);
                        int[] anIdx = w[4].Split(',').Select(s => int.Parse(s, CultureInfo.InvariantCulture)).ToArray();
                        for (int i = 0; i + 1 < anIdx.Length; i += 2)
                        {
                            aRoll.SetFaces(ShapeCasts.Face(aFaces[anIdx[i]]), ShapeCasts.Face(aFaces[anIdx[i + 1]]));
                        }
                        aRoll.Perform(w[3] == "1");
                        // the recorder writes the contact types as it goes: an exception ends the line
                        var aSb = new StringBuilder("SURFACES ").Append(aRoll.NbSurfaces());
                        try
                        {
                            for (int i = 1; i <= aRoll.NbSurfaces(); i++)
                            {
                                int aType = (int)aRoll.ContactType(i);
                                aSb.Append(' ').Append(aType);
                            }
                        }
                        catch (FailureException e)
                        {
                            aPort.Add((aSb.Append(" THROW ").Append(global::Kerf.Engine.Foundation.OcctTypeNames.Of(e.GetType())).ToString(), null));
                            break;
                        }
                        aPort.Add((aSb.ToString(), null));
                        aPort.Add(("DONE " + (aRoll.IsDone() ? "1" : "0"), aRoll.Shape()));
                        break;
                    }
                    default:
                    {
                        var aMk = new BRepOffsetAPI_MakeEvolved(Input(w[1]), ShapeCasts.Wire(Input(w[2])), JoinKind.Arc, true,
                                                                w[3] == "1", false, 1.0e-7, true);
                        aPort.Add(("DONE " + (aMk.IsDone() ? "1" : "0"), aMk.IsDone() ? aMk.Shape() : new Shape()));
                        break;
                    }
                }
            }
            catch (Exception e) when (e is FailureException)
            {
                aPort.Add(("THROW " + global::Kerf.Engine.Foundation.OcctTypeNames.Of(e.GetType()), null));
            }
            foreach ((Shape aS, ShapeSnapshot aSnap) in aSnaps)
            {
                try
                {
                    aSnap.AssertUnchanged(aS);
                }
                catch (Exception)
                {
                    aTally.InputsChanged++;
                    aTally.Differences.Add($"{aHead}: the port's input changed");
                }
            }

            // native's records against the port's: the same keywords, statuses and results
            int aCount = Math.Min(aNative.Count, aPort.Count);
            if (aNative.Count > 0 && aNative[^1].Head.StartsWith("THROW", StringComparison.Ordinal)
                && aPort.Count > 0 && aPort[^1].Head.StartsWith("THROW", StringComparison.Ordinal))
            {
                if (aNative[^1].Head == aPort[^1].Head)
                {
                    aTally.SameThrow++;
                }
                else
                {
                    aTally.Differences.Add($"{aHead}: native {aNative[^1].Head}, port {aPort[^1].Head}");
                }
                continue;
            }
            if (aNative.Count != aPort.Count)
            {
                aTally.Differences.Add($"{aHead}: native {string.Join(" / ", aNative.Select(n => n.Head))}, port {string.Join(" / ", aPort.Select(p => p.Head))}");
                continue;
            }
            for (int k = 0; k < aCount; k++)
            {
                string aNKey = aNative[k].Head.Split(' ')[0];
                if (aNKey == "SURFACES" || aNKey == "THROW")
                {
                    if (aNative[k].Head != aPort[k].Head)
                    {
                        aTally.Differences.Add($"{aHead}: native {aNative[k].Head}, port {aPort[k].Head}");
                    }
                    continue;
                }
                // DONE <flag> <n> and SHELL <n>: the flag, then the text
                if (aNKey == "DONE" && aNative[k].Head.Split(' ')[1] != aPort[k].Head.Split(' ')[1])
                {
                    aTally.Differences.Add($"{aHead}: native {aNative[k].Head}, port {aPort[k].Head}");
                    continue;
                }
                CompareText(aTally, $"{aHead} {aNKey}", aNative[k].Body, aPort[k].Shape ?? new Shape());
            }
        }

        bool isOk = true;
        foreach ((string aKind, Tally aT) in aTallies)
        {
            output.WriteLine($"{aKind}: {aT.Runs} runs; results {aT.Identical} byte-identical, {aT.FreeMasked} Free-masked, {aT.Canonical} canonical, {aT.Attributed} attributed, "
                             + $"{aT.Empty} null on both sides; {aT.SameThrow} the same exception; inputs changed {aT.InputsChanged}, "
                             + $"not frozen {aT.NotFrozen}; {aT.Differences.Count} differences");
            foreach (string aD in aT.Differences.Take(30))
            {
                output.WriteLine("  " + aD);
            }
            isOk &= aT.Differences.Count == 0;
        }
        Assert.True(isOk);
    }
}
