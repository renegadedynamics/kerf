#if HAS_ORACLE
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.TestHarness;
using Xunit.Abstractions;
using N = Build123d.Occt;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// ShapeUpgrade_UnifySameDomain (unit 6U) against native OCCT's, on native Boolean results: random boxes, cylinders,
/// cones, spheres and tori at random placements, placed so that faces are coplanar or coaxial (where the algorithm has
/// something to unify) or at random. Native OCCT writes each Boolean result; both sides read the file (the port's
/// frozen, as a caller's), run UnifySameDomain with the same options, and the results are compared as BRep text, the
/// Free (and Checked) column masked where needed, then canonically (decision 581); the histories through the canonical
/// keys (decision 637); the port's input is checked unchanged, its result frozen, and the share of the result's faces
/// and edges that are the caller's own is counted.
/// </summary>
public class ShapeUpgradeUnifySameDomainOracleTests(ITestOutputHelper output)
{
    /// <summary>The options of one run, as build123d.NET's Clean or Draw's unifysamedom set them.</summary>
    internal sealed record Options(string Name, bool UnifyEdges = true, bool UnifyFaces = true, bool ConcatBSplines = false,
                                  bool AllowInternal = false, bool SafeInput = true, double LinTol = 0, double AngTol = 0,
                                  bool KeepAnEdge = false, bool KeepAVertex = false);

    // build123d.NET's Clean: new ShapeUpgrade_UnifySameDomain(shape, true, true, true); AllowInternalEdges(false)
    internal static readonly Options s_clean = new("clean", true, true, true, false);

    private static readonly Options[] s_others =
    [
        new("default"),
        new("faces only", UnifyEdges: false),
        new("edges only", UnifyFaces: false),
        new("internal edges", ConcatBSplines: true, AllowInternal: true),
        new("not safe", ConcatBSplines: true, SafeInput: false),
        new("tolerances", LinTol: 1.0e-4, AngTol: Math.PI / 180),
        new("keep an edge", KeepAnEdge: true),
        new("keep a vertex", ConcatBSplines: true, KeepAVertex: true),
    ];

    internal sealed class Tally
    {
        public int Cases, Compared, Identical, FreeMasked, CheckedMasked, Canonical, Exceptions, Unchanged, NotFrozen,
                   Histories, HistoriesSame, HistoriesSameButInPlace, KeptFaces, ResultFaces, KeptEdges, ResultEdges,
                   NativeUnified, Itself;
        public readonly List<string> Differences = [];
    }

    [Fact]
    public void Clean_as_build123d_on_random_Boolean_results()
    {
        var anOracle = new Oracle(output, 20261030);
        using var aRun = new ShapeFixOracleRun(output, "usd");
        var aTally = new Tally();
        foreach ((string aName, N.TopoDS_Shape aShape) in BooleanResults(anOracle, 480))
        {
            Compare(aRun, aTally, aName, aShape, [s_clean], anOracle);
        }
        Report(aTally, "build123d.NET's Clean");
    }

    [Fact]
    public void Other_options_on_random_Boolean_results()
    {
        var anOracle = new Oracle(output, 20261031);
        using var aRun = new ShapeFixOracleRun(output, "usd");
        var aTally = new Tally();
        foreach ((string aName, N.TopoDS_Shape aShape) in BooleanResults(anOracle, 160))
        {
            Compare(aRun, aTally, aName, aShape, s_others, anOracle);
        }
        Report(aTally, "other options");
    }

    /// <summary>
    /// The heal group's public inputs and data/occ's shapes (up to 300 kB), as build123d.NET's Clean and with
    /// OCCT's defaults: B-spline and offset geometry, seams, degenerated edges, shells and compounds of faces.
    /// </summary>
    [Fact]
    public void Clean_and_defaults_on_the_heal_groups_and_data_occs_shapes()
    {
        var anOracle = new Oracle(output, 20261101);
        using var aRun = new ShapeFixOracleRun(output, "usd");
        var aTally = new Tally();
        foreach ((string aFile, string _) in aRun.Inputs(300_000))
        {
            if (ShapeFixOracleRun.NativeRead(aFile) is not N.TopoDS_Shape aShape)
            {
                continue;
            }
            Compare(aRun, aTally, Path.GetFileName(aFile), aShape, [s_clean, s_others[0], s_others[3]], anOracle);
        }
        Report(aTally, "the heal group's and data/occ's shapes");
    }

    private void Report(Tally theTally, string theWhat)
    {
        output.WriteLine($"{theWhat}: {theTally.Cases} Boolean results, {theTally.Compared} runs ({theTally.NativeUnified} changed natively, "
                         + $"{theTally.Itself} given back as the input itself by the port)");
        output.WriteLine($"  equal: {theTally.Identical} ({theTally.Identical - theTally.FreeMasked - theTally.CheckedMasked - theTally.Exceptions} byte-identical, "
                         + $"{theTally.FreeMasked} Free-masked, {theTally.CheckedMasked} Free and Checked masked, {theTally.Exceptions} same exception); "
                         + $"canonically equal: {theTally.Canonical}");
        output.WriteLine($"  histories: {theTally.HistoriesSame} of {theTally.Histories} equal canonically "
                         + $"({theTally.HistoriesSameButInPlace} more equal but for sub-shapes changed in place, decision 643)");
        output.WriteLine($"  inputs unchanged: {theTally.Unchanged}; results not frozen: {theTally.NotFrozen}");
        output.WriteLine($"  the caller's own: {theTally.KeptFaces} of {theTally.ResultFaces} result faces, {theTally.KeptEdges} of {theTally.ResultEdges} edges");
        foreach (string aDifference in theTally.Differences.Take(40))
        {
            output.WriteLine("  " + aDifference);
        }
        Assert.Empty(theTally.Differences);
    }

    internal void Compare(ShapeFixOracleRun theRun, Tally theTally, string theName, N.TopoDS_Shape theNative,
                         Options[] theOptions, Oracle theOracle)
    {
        if (theNative.IsNull())
        {
            return;
        }
        theTally.Cases++;
        foreach (Options anOptions in theOptions)
        {
            // both sides read the native Boolean result from its file
            if (theRun.Read(theNative) is not (N.TopoDS_Shape aNIn, TopoDS_Shape aPIn))
            {
                continue;
            }
            string aWhat = $"{theName} [{anOptions.Name}]";
            string aPInText = theRun.PortText(aPIn);
            string aNInText = theRun.NativeText(aNIn);

            // the edge or vertex to keep, by its index in both sides' maps
            int aKeepIndex = 0;
            TopAbs_ShapeEnum aKeepType = anOptions.KeepAVertex ? TopAbs_ShapeEnum.TopAbs_VERTEX : TopAbs_ShapeEnum.TopAbs_EDGE;
            var aPMapIn = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            TopExp.MapShapes(aPIn, aKeepType, aPMapIn);
            if ((anOptions.KeepAnEdge || anOptions.KeepAVertex) && aPMapIn.Extent() > 0)
            {
                aKeepIndex = theOracle.Int(1, aPMapIn.Extent());
            }

            // native
            N.TopoDS_Shape? aNRes = null;
            string? aNError = null;
            N.BRepTools_History? aNHistory = null;
            try
            {
                using var aNU = new N.ShapeUpgrade_UnifySameDomain(aNIn, anOptions.UnifyEdges, anOptions.UnifyFaces, anOptions.ConcatBSplines);
                aNU.AllowInternalEdges(anOptions.AllowInternal);
                aNU.SetSafeInputMode(anOptions.SafeInput);
                if (anOptions.LinTol > 0)
                {
                    aNU.SetLinearTolerance(anOptions.LinTol);
                }
                if (anOptions.AngTol > 0)
                {
                    aNU.SetAngularTolerance(anOptions.AngTol);
                }
                if (aKeepIndex > 0)
                {
                    using var aNMap = new N.TopTools_IndexedMapOfShape();
                    N.TopExp.MapShapes(aNIn, aKeepType == TopAbs_ShapeEnum.TopAbs_VERTEX ? N.TopAbs_ShapeEnum.TopAbs_VERTEX : N.TopAbs_ShapeEnum.TopAbs_EDGE, aNMap);
                    aNU.KeepShape(aNMap.FindKey(aKeepIndex));
                }
                aNU.Build();
                aNRes = aNU.Shape();
                aNHistory = aNU.History();
            }
            catch (Exception e) when (ShapeFixOracleRun.ErrorOf(e) is not null)
            {
                aNError = ShapeFixOracleRun.ErrorOf(e);
            }

            // port
            TopoDS_Shape? aPRes = null;
            string? aPError = null;
            BRepTools_History? aPHistory = null;
            try
            {
                var aPU = new ShapeUpgrade_UnifySameDomain(aPIn, anOptions.UnifyEdges, anOptions.UnifyFaces, anOptions.ConcatBSplines);
                aPU.AllowInternalEdges(anOptions.AllowInternal);
                aPU.SetSafeInputMode(anOptions.SafeInput);
                if (anOptions.LinTol > 0)
                {
                    aPU.SetLinearTolerance(anOptions.LinTol);
                }
                if (anOptions.AngTol > 0)
                {
                    aPU.SetAngularTolerance(anOptions.AngTol);
                }
                if (aKeepIndex > 0)
                {
                    aPU.KeepShape(aPMapIn[aKeepIndex]);
                }
                aPU.Build();
                aPRes = aPU.Shape();
                aPHistory = aPU.History();
            }
            catch (Exception e) when (ShapeFixOracleRun.ErrorOf(e) is not null)
            {
                aPError = ShapeFixOracleRun.ErrorOf(e);
            }

            theTally.Compared++;
            if (theRun.PortText(aPIn) == aPInText)
            {
                theTally.Unchanged++;
            }
            else
            {
                theTally.Differences.Add($"{aWhat}: the port's input changed");
            }
            if (aNError is not null || aPError is not null)
            {
                if (aNError == aPError)
                {
                    theTally.Identical++;
                    theTally.Exceptions++;
                }
                else
                {
                    theTally.Differences.Add($"{aWhat}: {aNError ?? "no exception"} natively, {aPError ?? "no exception"} in the port");
                }
                continue;
            }

            TopoDS_Shape aPR = aPRes!.Value;
            if (!aPR.TShape()!.IsFrozen())
            {
                theTally.NotFrozen++;
                theTally.Differences.Add($"{aWhat}: the port's result isn't frozen");
            }
            if (ReferenceEquals(aPR.TShape(), aPIn.TShape()))
            {
                theTally.Itself++;
            }
            string aNT = theRun.NativeText(aNRes!);
            string aPT = theRun.PortText(aPR);
            if (aNT != aNInText)
            {
                theTally.NativeUnified++;
            }
            CountKept(theTally, aPIn, aPR);

            BRepTextDiff aDiff = BRepTextComparison.Compare(aNT, aPT);
            // both results as read from their texts (reading a plane orthogonalises its axes, so a shape read back
            // may differ in the last bits from the one written)
            TopoDS_Shape aNResP = ReadPort(theRun, aNT);
            TopoDS_Shape aPResP = ReadPort(theRun, aPT);
            if (aDiff.IsIdentical)
            {
                theTally.Identical++;
            }
            else if (aDiff.IsIdenticalMaskingFree)
            {
                theTally.Identical++;
                theTally.FreeMasked++;
            }
            else if (ShapeFixFaceOracleTests.MaskFreeAndChecked(aNT) == ShapeFixFaceOracleTests.MaskFreeAndChecked(aPT))
            {
                theTally.Identical++;
                theTally.CheckedMasked++;
            }
            else
            {
                CanonicalComparison aCanon = CanonicalShapeComparer.Compare(aNResP, aPResP, new CanonicalOptions { CheckValidity = false });
                if (aCanon.IsIdentical)
                {
                    theTally.Canonical++;
                }
                else
                {
                    theTally.Differences.Add($"{aWhat}: {aDiff}; canonically {aCanon}");
                    continue;
                }
            }

            // the histories over the port's input and the results read back, sub-shapes matched by their indices in
            // TopExp::MapShapes (native's input and the port's are read from the same file)
            theTally.Histories++;
            BRepTools_History aNH = NativeHistory(aNHistory!, aNIn, aNRes!, aPIn, aNResP, out Dictionary<int, TopoDS_Shape> aKeptByNative);
            BRepTools_History aPHR = PortHistory(aPHistory!, aPIn, aPR, aPResP);
            List<string> aHDiff = HistoryComparison.Compare([aPIn], aNResP, aNH, [aPIn], aPResP, aPHR);
            if (aHDiff.Count == 0)
            {
                theTally.HistoriesSame++;
                continue;
            }
            // decision 643: a sub-shape the algorithm changed in place (a pcurve added) is modified into its changed
            // copy in the port, where native has no entry; drop those entries and compare again
            BRepTools_History aPH = WithoutInPlace(aPHR, aPIn, ResultAsPort(aKeptByNative, aNResP, aPResP));
            List<string> aHDiff2 = HistoryComparison.Compare([aPIn], aNResP, aNH, [aPIn], aPResP, aPH);
            if (aHDiff2.Count == 0)
            {
                theTally.HistoriesSameButInPlace++;
            }
            else
            {
                theTally.Differences.Add($"{aWhat}: histories differ: {string.Join("; ", aHDiff2.Take(4))}");
            }
        }
    }

    private static TopoDS_Shape ReadPort(ShapeFixOracleRun theRun, string theText)
    {
        string aFile = Path.Combine(Path.GetTempPath(), $"usd-{Guid.NewGuid():N}.brep");
        File.WriteAllText(aFile, theText);
        try
        {
            Assert.True(BRepTools.Read(out TopoDS_Shape aShape, aFile));
            return aShape;
        }
        finally
        {
            File.Delete(aFile);
        }
    }

    // Native's history, over the port's readings of native's input and result (sub-shapes matched by their indices in
    // TopExp::MapShapes, which the BRep text keeps).
    private static BRepTools_History NativeHistory(N.BRepTools_History theHistory, N.TopoDS_Shape theNIn, N.TopoDS_Shape theNRes,
                                                   in TopoDS_Shape thePIn, in TopoDS_Shape thePRes,
                                                   out Dictionary<int, TopoDS_Shape> theKeptByNative)
    {
        using var aNInMap = new N.TopTools_IndexedMapOfShape();
        using var aNResMap = new N.TopTools_IndexedMapOfShape();
        N.TopExp.MapShapes(theNIn, aNInMap);
        N.TopExp.MapShapes(theNRes, aNResMap);
        var aPInMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        var aPResMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(thePIn, aPInMap);
        TopExp.MapShapes(thePRes, aPResMap);
        Assert.Equal(aNInMap.Extent(), aPInMap.Extent());
        Assert.Equal(aNResMap.Extent(), aPResMap.Extent());

        var aHistory = new BRepTools_History();
        theKeptByNative = [];
        for (int i = 1; i <= aNInMap.Extent(); i++)
        {
            using N.TopoDS_Shape aNS = aNInMap.FindKey(i);
            TopoDS_Shape aPS = aPInMap[i];
            int aKeptIndex = aNResMap.FindIndex(aNS);
            if (aKeptIndex > 0)
            {
                theKeptByNative[i] = aPResMap[aKeptIndex];
            }
            if (!BRepTools_History.IsSupportedType(aPS))
            {
                continue;
            }
            if (theHistory.IsRemoved(aNS))
            {
                aHistory.Remove(aPS);
            }
            using (N.TopTools_ListOfShape aModified = theHistory.Modified(aNS))
            {
                foreach (N.TopoDS_Shape aNM in aModified)
                {
                    int anIndex = aNResMap.FindIndex(aNM);
                    Assert.True(anIndex > 0, "a modified shape outside native's result");
                    aHistory.AddModified(aPS, aPResMap[anIndex]);
                }
            }
            using (N.TopTools_ListOfShape aGenerated = theHistory.Generated(aNS))
            {
                foreach (N.TopoDS_Shape aNG in aGenerated)
                {
                    int anIndex = aNResMap.FindIndex(aNG);
                    Assert.True(anIndex > 0, "a generated shape outside native's result");
                    aHistory.AddGenerated(aPS, aPResMap[anIndex]);
                }
            }
        }
        return aHistory;
    }

    // The port's history with its result's sub-shapes replaced by those of the result read back (by their indices).
    private static BRepTools_History PortHistory(BRepTools_History theHistory, TopoDS_Shape theIn, in TopoDS_Shape theRes, in TopoDS_Shape theResRead)
    {
        var aResMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        var aReadMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theRes, aResMap);
        TopExp.MapShapes(theResRead, aReadMap);
        Assert.Equal(aResMap.Extent(), aReadMap.Extent());
        TopoDS_Shape Read(TopoDS_Shape theS)
        {
            int anIndex = aResMap.FindIndex(theS);
            Assert.True(anIndex > 0, "a history shape outside the port's result");
            return aReadMap[anIndex];
        }
        return Remapped(theHistory, theIn, Read);
    }

    private static BRepTools_History Remapped(BRepTools_History theHistory, TopoDS_Shape theIn, Func<TopoDS_Shape, TopoDS_Shape> theMap)
    {
        // the history's own inputs are the port's input's sub-shapes; only the images move
        var aHistory = new BRepTools_History();
        foreach ((TopoDS_Shape aS, bool isRemoved, IReadOnlyList<TopoDS_Shape> aModified, IReadOnlyList<TopoDS_Shape> aGenerated) in Entries(theHistory, theIn))
        {
            if (isRemoved)
            {
                aHistory.Remove(aS);
            }
            foreach (TopoDS_Shape aM in aModified)
            {
                aHistory.AddModified(aS, theMap(aM));
            }
            foreach (TopoDS_Shape aG in aGenerated)
            {
                aHistory.AddGenerated(aS, theMap(aG));
            }
        }
        return aHistory;
    }

    // The entries of a history over the vertices, edges, faces and solids of the input.
    private static IEnumerable<(TopoDS_Shape, bool, IReadOnlyList<TopoDS_Shape>, IReadOnlyList<TopoDS_Shape>)> Entries(BRepTools_History theHistory,
                                                                                                                  TopoDS_Shape theIn)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theIn, aMap);
        for (int i = 1; i <= aMap.Extent(); i++)
        {
            TopoDS_Shape aS = aMap[i];
            if (BRepTools_History.IsSupportedType(aS))
            {
                yield return (aS, theHistory.IsRemoved(aS), theHistory.Modified(aS), theHistory.Generated(aS));
            }
        }
    }

    // The port's history without the entries "modified into its changed copy" of decision 643: the sub-shapes of the
    // input that native OCCT keeps in its result (the same TShape, changed in place or not), which native's history
    // leaves out, and whose image in the port is the corresponding sub-shape of its result (by index: the results are
    // byte-identical).
    private static BRepTools_History WithoutInPlace(BRepTools_History theHistory, TopoDS_Shape theIn,
                                                    Dictionary<int, TopoDS_Shape> theKeptByNative)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theIn, aMap);
        var aHistory = new BRepTools_History();
        for (int i = 1; i <= aMap.Extent(); i++)
        {
            TopoDS_Shape aS = aMap[i];
            if (!BRepTools_History.IsSupportedType(aS))
            {
                continue;
            }
            if (theHistory.IsRemoved(aS))
            {
                aHistory.Remove(aS);
            }
            var aModified = theHistory.Modified(aS);
            if (!(aModified.Count == 1 && theKeptByNative.TryGetValue(i, out TopoDS_Shape aKept) && aModified[0].IsSame(aKept)))
            {
                foreach (TopoDS_Shape aM in aModified)
                {
                    aHistory.AddModified(aS, aM);
                }
            }
            foreach (TopoDS_Shape aG in theHistory.Generated(aS))
            {
                aHistory.AddGenerated(aS, aG);
            }
        }
        return aHistory;
    }

    // The kept shapes, as native's result read back, moved onto the port's result read back (by index)
    private static Dictionary<int, TopoDS_Shape> ResultAsPort(Dictionary<int, TopoDS_Shape> theKept, in TopoDS_Shape theNRes, in TopoDS_Shape thePRes)
    {
        var aNMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        var aPMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theNRes, aNMap);
        TopExp.MapShapes(thePRes, aPMap);
        var aKept = new Dictionary<int, TopoDS_Shape>();
        if (aNMap.Extent() != aPMap.Extent())
        {
            return aKept;
        }
        foreach ((int i, TopoDS_Shape aS) in theKept)
        {
            aKept[i] = aPMap[aNMap.FindIndex(aS)];
        }
        return aKept;
    }

    private static void CountKept(Tally theTally, in TopoDS_Shape theInput, in TopoDS_Shape theResult)
    {
        foreach ((TopAbs_ShapeEnum aType, bool isFace) in new[] { (TopAbs_ShapeEnum.TopAbs_FACE, true), (TopAbs_ShapeEnum.TopAbs_EDGE, false) })
        {
            var anInput = new HashSet<TopoDS_TShape>(ReferenceEqualityComparer.Instance);
            foreach (TopoDS_Shape aS in CanonicalShapeComparer.SubShapes(theInput, aType))
            {
                anInput.Add(aS.TShape()!);
            }
            foreach (TopoDS_Shape aS in CanonicalShapeComparer.SubShapes(theResult, aType))
            {
                bool isKept = anInput.Contains(aS.TShape()!);
                if (isFace)
                {
                    theTally.ResultFaces++;
                    theTally.KeptFaces += isKept ? 1 : 0;
                }
                else
                {
                    theTally.ResultEdges++;
                    theTally.KeptEdges += isKept ? 1 : 0;
                }
            }
        }
    }

    // ---------------------------------------------------------------------------------------------------------------
    // The inputs: native Boolean results

    private static gp_Ax2 Frame(Oracle theO, bool theAxisAligned)
    {
        if (theAxisAligned)
        {
            return new gp_Ax2(new gp_Pnt(theO.Int(-20, 20), theO.Int(-20, 20), theO.Int(-20, 20)), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0));
        }
        gp_Dir aZ = theO.Dir();
        gp_Dir aRef = theO.Dir();
        gp_Dir aX = aZ.Crossed(aRef.IsParallel(aZ, 0.1) ? (aZ.IsParallel(new gp_Dir(1, 0, 0), 0.1) ? new gp_Dir(0, 1, 0) : new gp_Dir(1, 0, 0)) : aRef);
        return new gp_Ax2(theO.Pnt(), aZ, aX);
    }

    // The frame moved by (dx, dy, dz) in its own axes, and turned by an angle about its Z axis.
    private static gp_Ax2 Moved(in gp_Ax2 theA, double dx, double dy, double dz, double theAngle = 0)
    {
        gp_Pnt aP = theA.Location().Translated(new gp_Vec(theA.XDirection()) * dx + new gp_Vec(theA.YDirection()) * dy
                                               + new gp_Vec(theA.Direction()) * dz);
        gp_Ax2 aMoved = new gp_Ax2(aP, theA.Direction(), theA.XDirection());
        return theAngle == 0 ? aMoved : aMoved.Rotated(new gp_Ax1(aP, theA.Direction()), theAngle);
    }

    private static N.TopoDS_Shape Box(in gp_Ax2 theA, double dx, double dy, double dz)
    {
        using N.gp_Ax2 aA = Oracle.ToNative(theA);
        using var aMaker = new N.BRepPrimAPI_MakeBox(aA, dx, dy, dz);
        return aMaker.Shape();
    }

    private static N.TopoDS_Shape Cylinder(in gp_Ax2 theA, double r, double h, double theAngle = 0)
    {
        using N.gp_Ax2 aA = Oracle.ToNative(theA);
        using var aMaker = theAngle > 0 ? new N.BRepPrimAPI_MakeCylinder(aA, r, h, theAngle) : new N.BRepPrimAPI_MakeCylinder(aA, r, h);
        return aMaker.Shape();
    }

    private static N.TopoDS_Shape Cone(in gp_Ax2 theA, double r1, double r2, double h)
    {
        using N.gp_Ax2 aA = Oracle.ToNative(theA);
        using var aMaker = new N.BRepPrimAPI_MakeCone(aA, r1, r2, h);
        return aMaker.Shape();
    }

    private static N.TopoDS_Shape Sphere(in gp_Ax2 theA, double r)
    {
        using N.gp_Ax2 aA = Oracle.ToNative(theA);
        using var aMaker = new N.BRepPrimAPI_MakeSphere(aA, r);
        return aMaker.Shape();
    }

    private static N.TopoDS_Shape Torus(in gp_Ax2 theA, double r1, double r2)
    {
        using N.gp_Ax2 aA = Oracle.ToNative(theA);
        using var aMaker = new N.BRepPrimAPI_MakeTorus(aA, r1, r2);
        return aMaker.Shape();
    }

    private enum Op { Fuse, Cut, Common }

    private static N.TopoDS_Shape? Boolean(Op theOp, N.TopoDS_Shape theA, N.TopoDS_Shape theB)
    {
        try
        {
            using N.BRepAlgoAPI_BooleanOperation aBop = theOp switch
            {
                Op.Fuse => new N.BRepAlgoAPI_Fuse(theA, theB),
                Op.Cut => new N.BRepAlgoAPI_Cut(theA, theB),
                _ => new N.BRepAlgoAPI_Common(theA, theB),
            };
            return aBop.IsDone() && !aBop.HasErrors() ? aBop.Shape() : null;
        }
        catch (N.OcctException)
        {
            return null;
        }
    }

    /// <summary>Native Boolean results of random primitives, most placed so that faces share a plane or an axis.</summary>
    internal static IEnumerable<(string Name, N.TopoDS_Shape Shape)> BooleanResults(Oracle theO, int theCount)
    {
        for (int i = 0; i < theCount; i++)
        {
            bool isAligned = theO.Int(0, 1) == 0;
            gp_Ax2 aF = Frame(theO, isAligned);
            double a = isAligned ? theO.Int(2, 12) : theO.Real(2, 12);
            double b = isAligned ? theO.Int(2, 12) : theO.Real(2, 12);
            double c = isAligned ? theO.Int(2, 12) : theO.Real(2, 12);
            Op anOp = (Op)theO.Int(0, 2);
            string aName;
            N.TopoDS_Shape? aResult;
            switch (i % 14)
            {
                case 0:
                {
                    // two boxes side by side or overlapping, the same height: coplanar tops and bottoms
                    double dx = theO.Int(0, 1) == 0 ? a : a * theO.Real(0.2, 0.9);
                    double dy = isAligned ? theO.Int(-1, 1) : b * theO.Real(-0.5, 0.5);
                    aName = $"boxes side by side {anOp}";
                    aResult = Boolean(anOp == Op.Common ? Op.Fuse : anOp, Box(aF, a, b, c), Box(Moved(aF, dx, dy, 0), a, b, c));
                    break;
                }
                case 1:
                {
                    // a pocket: a box cut by a box from its top
                    double d = c * theO.Real(0.2, 0.8);
                    aName = $"box pocket";
                    aResult = Boolean(Op.Cut, Box(aF, a, b, c), Box(Moved(aF, a * 0.25, b * 0.25, c - d), a * 0.5, b * 0.5, d + 1));
                    break;
                }
                case 2:
                {
                    // two coaxial cylinders of the same radius, end to end or overlapping
                    double r = a / 2;
                    double dz = theO.Int(0, 1) == 0 ? c : c * theO.Real(0.2, 0.9);
                    double anAngle = theO.Int(0, 1) == 0 ? 0 : theO.Real(0.1, 3);
                    aName = $"coaxial cylinders {anOp}";
                    aResult = Boolean(anOp, Cylinder(aF, r, c), Cylinder(Moved(aF, 0, 0, dz, anAngle), r, c));
                    break;
                }
                case 3:
                {
                    // a shaft: coaxial cylinders of different radii
                    aName = $"shaft {anOp}";
                    aResult = Boolean(anOp, Cylinder(aF, a / 2, c), Cylinder(Moved(aF, 0, 0, c * theO.Real(0.3, 1.0)), a / 2 * theO.Real(0.3, 0.9), c));
                    break;
                }
                case 4:
                {
                    // a box with a cylinder through it, along its height
                    double r = Math.Min(a, b) * theO.Real(0.1, 0.4);
                    aName = $"box and cylinder {anOp}";
                    aResult = Boolean(anOp, Box(aF, a, b, c), Cylinder(Moved(aF, a / 2, b / 2, theO.Int(0, 1) == 0 ? 0 : -1), r, c + theO.Int(0, 2)));
                    break;
                }
                case 5:
                {
                    // partial coaxial cylinders of the same radius, turned about their axis
                    double r = a / 2;
                    double anAngle1 = theO.Real(1, 5), anAngle2 = theO.Real(1, 5);
                    aName = $"partial cylinders {anOp}";
                    aResult = Boolean(anOp == Op.Cut ? Op.Fuse : anOp, Cylinder(aF, r, c, anAngle1),
                                      Cylinder(Moved(aF, 0, 0, theO.Int(0, 1) == 0 ? 0 : c * 0.5, theO.Real(0.2, 3)), r, c, anAngle2));
                    break;
                }
                case 6:
                {
                    // coaxial cones of the same apex angle, or a cone and a cylinder
                    double r1 = a / 2, r2 = a / 4;
                    double t = theO.Real(0.2, 0.8);
                    aName = $"coaxial cones {anOp}";
                    N.TopoDS_Shape aSecond = theO.Int(0, 1) == 0
                                               ? Cone(Moved(aF, 0, 0, c * t), r1 + (r2 - r1) * t, r2 + (r2 - r1) * t, c)
                                               : Cylinder(Moved(aF, 0, 0, c), r2, c);
                    aResult = Boolean(anOp, Cone(aF, r1, r2, c), aSecond);
                    break;
                }
                case 7:
                {
                    // three boxes in a row
                    aName = $"three boxes";
                    N.TopoDS_Shape? aTwo = Boolean(Op.Fuse, Box(aF, a, b, c), Box(Moved(aF, a, 0, 0), a, b, c));
                    aResult = aTwo is null ? null : Boolean(Op.Fuse, aTwo, Box(Moved(aF, 2 * a, theO.Int(0, 1) == 0 ? 0 : b / 2, 0), a, b, c));
                    break;
                }
                case 8:
                {
                    // two boxes with a common top plane, one turned about the vertical
                    aName = $"turned boxes {anOp}";
                    aResult = Boolean(anOp == Op.Common ? Op.Fuse : anOp, Box(aF, a, b, c),
                                      Box(Moved(aF, a * 0.5, b * 0.5, 0, theO.Real(0.1, 1.4)), a, b, c));
                    break;
                }
                case 9:
                {
                    // a sphere and a coaxial cylinder of its radius, or two spheres
                    double r = a / 2;
                    aName = $"sphere {anOp}";
                    N.TopoDS_Shape aSecond = theO.Int(0, 1) == 0 ? Cylinder(aF, r, c) : Sphere(Moved(aF, 0, 0, r * theO.Real(0.2, 1.5)), r);
                    aResult = Boolean(anOp, Sphere(aF, r), aSecond);
                    break;
                }
                case 10:
                {
                    // a torus and a coaxial cylinder or box
                    aName = $"torus {anOp}";
                    N.TopoDS_Shape aSecond = theO.Int(0, 1) == 0 ? Cylinder(Moved(aF, 0, 0, -c / 2), a, c) : Box(Moved(aF, 0, -a, -c), a, 2 * a, 2 * c);
                    aResult = Boolean(anOp, Torus(aF, a, a / 4), aSecond);
                    break;
                }
                case 11:
                {
                    // a cylinder cut along its axis by a box (a half cylinder), then fused with the other half
                    double r = a / 2;
                    aName = $"halves of a cylinder";
                    N.TopoDS_Shape aCyl = Cylinder(aF, r, c);
                    N.TopoDS_Shape aBox = Box(Moved(aF, 0, -r - 1, -1), r + 1, 2 * r + 2, c + 2);
                    N.TopoDS_Shape? aHalf1 = Boolean(Op.Cut, aCyl, aBox);
                    N.TopoDS_Shape? aHalf2 = Boolean(Op.Common, aCyl, aBox);
                    aResult = aHalf1 is null || aHalf2 is null ? null : Boolean(Op.Fuse, aHalf1, aHalf2);
                    break;
                }
                case 12:
                {
                    // a stepped block: a box fused with a smaller box on its top, flush on one side
                    aName = $"stepped block {anOp}";
                    aResult = Boolean(anOp == Op.Common ? Op.Fuse : anOp, Box(aF, a, b, c), Box(Moved(aF, 0, 0, c), a * theO.Real(0.3, 1), b, c * theO.Real(0.3, 1)));
                    break;
                }
                default:
                {
                    // random primitives at random placements
                    gp_Ax2 aG = Frame(theO, false);
                    aG = new gp_Ax2(aF.Location().Translated(new gp_Vec(theO.Real(-3, 3), theO.Real(-3, 3), theO.Real(-3, 3))), aG.Direction(), aG.XDirection());
                    aName = $"random {anOp}";
                    N.TopoDS_Shape First(int k, in gp_Ax2 theA) => k switch
                    {
                        0 => Box(theA, a, b, c),
                        1 => Cylinder(theA, a / 2, c),
                        2 => Cone(theA, a / 2, a / 5, c),
                        3 => Sphere(theA, a / 2),
                        _ => Torus(theA, a / 2, a / 6),
                    };
                    aResult = Boolean(anOp, First(theO.Int(0, 4), aF), First(theO.Int(0, 4), aG));
                    break;
                }
            }
            if (aResult is not null && !aResult.IsNull())
            {
                yield return ($"{i}: {aName}", aResult);
            }
        }
    }
}
#endif
