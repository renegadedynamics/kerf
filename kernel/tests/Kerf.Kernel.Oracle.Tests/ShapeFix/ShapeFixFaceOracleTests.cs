// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Brep;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Diagnostics;
using System.Reflection;
using System.Text.RegularExpressions;
using Kerf.Kernel.TestHarness;
using Xunit.Abstractions;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>
/// ShapeFix_Face (and through it ShapeFix_ComposeShell, ShapeFix_Wire's 2d fixes, ShapeFix_IntersectionTool) against
/// native OCCT.
/// </summary>
/// <remarks>
/// <para>
/// Inputs: the faces of the heal group's public data files and data/occ (a sample per shape, up to 1 MB), fixed with
/// <c>Perform()</c>; and faces as build123d.NET makes them: planar faces from an outer wire and holes (some holes
/// misoriented) fixed with <c>FixOrientation()</c> then <c>Perform()</c> (TopoUtil.MakeFaceFromWires), and faces on a
/// cylinder, cone, sphere or torus bounded by 3d circles without pcurves (the wire fixes add the pcurves; the face
/// misses its seam, its natural bound or its apex), fixed with <c>Perform()</c>, both as a face and through
/// <c>Init(surface, precision, fwd)</c> and <c>Add(wire)</c>. Native OCCT writes each input to a file both sides read;
/// the port's input is frozen, as a caller's is.
/// </para>
/// <para>
/// Compared: the result (<c>Result()</c>) as BRep text with the Free column masked (frozen sub-shapes keep the Free
/// flag they were frozen with, open question 1), then canonically where the texts differ (decision 581); the
/// statuses OK, DONE1 to DONE8 and FAIL1 to FAIL8; the exceptions by OCCT type; and that the port's input is
/// unchanged. Native ShapeFix_Face's context isn't bound, so histories aren't compared here.
/// </para>
/// </remarks>
public class ShapeFixFaceOracleTests(ITestOutputHelper output) : IDisposable
{
    private readonly string myDirectory = Directory.CreateTempSubdirectory("occtnet-sff-").FullName;
    private int myFileNumber;
    private readonly List<string> myDifferences = [];
    private readonly int[] myStatusCounts = new int[32];
    private int myCompared, myIdentical, myFreeMasked, myCheckedMasked, myCanonical, myExceptions, myUnchanged, myFixed, myChanged;

    public void Dispose()
    {
        Directory.Delete(myDirectory, true);
        GC.SuppressFinalize(this);
    }

    private static string Metadata(string theKey)
        => typeof(ShapeFixFaceOracleTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == theKey).Value!;

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
        Assert.True(BrepTools.Write(theShape, aFile, true, false, FormatVersion.FormatVersion_VERSION_3));
        string aText = File.ReadAllText(aFile);
        File.Delete(aFile);
        return aText;
    }

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
        if (!aPShape.IsNull)
        {
            aPShape.TShape()!.FreezeGraph(); // a caller's shape
        }
        return (aNShape, aPShape);
    }

    private static string? ErrorOf(Exception theException) => Refusal.Unwrapped(theException) switch
    {
        N.OcctException e => e.Message.IndexOf(':') is int c and > 0 ? e.Message[..c] : e.Message,
        FailureException f => global::Kerf.Engine.Foundation.OcctTypeNames.Of(f.GetType()),
        _ => null,
    };

    private static readonly ExtendStatus[] s_statuses = Enum.GetValues<ExtendStatus>()
        .Where(s => s is not ExtendStatus.Done and not ExtendStatus.Fail).ToArray();

    private static int PortStatus(FaceFixer theFix)
    {
        int aBits = 0;
        for (int i = 0; i < s_statuses.Length; i++)
        {
            aBits |= theFix.Status(s_statuses[i]) ? 1 << i : 0;
        }
        return aBits;
    }

    private static int NativeStatus(N.ShapeFix_Face theFix)
    {
        int aBits = 0;
        for (int i = 0; i < s_statuses.Length; i++)
        {
            aBits |= theFix.Status((N.ShapeExtend_Status)(int)s_statuses[i]) ? 1 << i : 0;
        }
        return aBits;
    }

    /// <summary>How the face is fixed.</summary>
    private enum Mode
    {
        /// <summary><c>new ShapeFix_Face(face)</c>, <c>Perform()</c>.</summary>
        Perform,
        /// <summary>build123d.NET's MakeFaceFromWires: <c>FixOrientation()</c>, then <c>Perform()</c>.</summary>
        OrientationThenPerform,
        /// <summary><c>Init(surface, 1e-7, fwd)</c>, <c>Add</c> of each wire of the face, <c>Perform()</c>.</summary>
        SurfaceAndWires,
        /// <summary>As <see cref="SurfaceAndWires"/>, reversed.</summary>
        SurfaceAndWiresReversed,
    }

    private static N.TopoDS_Shape NativeFix(N.TopoDS_Face theFace, Mode theMode, out bool theDone, out int theStatus)
    {
        using var aFix = new N.ShapeFix_Face();
        theDone = false;
        switch (theMode)
        {
            case Mode.Perform:
                aFix.Init(theFace);
                theDone = aFix.Perform();
                break;
            case Mode.OrientationThenPerform:
                aFix.Init(theFace);
                aFix.FixOrientation();
                theDone = aFix.Perform();
                break;
            default:
            {
                aFix.Init(N.BRep_Tool.Surface(theFace), 1.0e-7, theMode == Mode.SurfaceAndWires);
                for (var anIt = new N.TopoDS_Iterator(theFace); anIt.More(); anIt.Next())
                {
                    if (anIt.Value().ShapeType() == N.TopAbs_ShapeEnum.TopAbs_WIRE)
                    {
                        aFix.Add(N.TopoDS.Wire(anIt.Value()));
                    }
                }
                GC.KeepAlive(theFace); // native TopoDS_Iterator holds no reference to the shape it iterates (decision 1109)
                theDone = aFix.Perform();
                break;
            }
        }
        theStatus = NativeStatus(aFix);
        return aFix.Result();
    }

    private static Shape PortFix(in Face theFace, Mode theMode, out bool theDone, out int theStatus)
    {
        var aFix = new FaceFixer();
        theDone = false;
        switch (theMode)
        {
            case Mode.Perform:
                aFix.Init(theFace);
                theDone = aFix.Perform();
                break;
            case Mode.OrientationThenPerform:
                aFix.Init(theFace);
                aFix.FixOrientation();
                theDone = aFix.Perform();
                break;
            default:
            {
                aFix.Init(Tool.Surface(theFace)!, 1.0e-7, theMode == Mode.SurfaceAndWires);
                for (var anIt = new Iterator(theFace); anIt.More(); anIt.Next())
                {
                    if (anIt.Value().Type == ShapeType.Wire)
                    {
                        aFix.Add(ShapeCasts.Wire(anIt.Value()));
                    }
                }
                theDone = aFix.Perform();
                break;
            }
        }
        theStatus = PortStatus(aFix);
        Shape aResult = aFix.Result();
        Assert.True(aResult.IsNull || aResult.TShape()!.IsFrozen);
        return aResult;
    }

    private void Compare(N.TopoDS_Shape theFace, string theWhat, Mode theMode)
    {
        if (Read(theFace) is not (N.TopoDS_Shape aNS, Shape aPS) || aPS.Type != ShapeType.Face)
        {
            return;
        }
        string w = $"{theWhat} ({theMode})";
        string anInputText = PortText(aPS); // the frozen input, which nothing may change

        N.TopoDS_Shape? aNRes = null;
        bool aNDone = false;
        int aNStatus = 0;
        string? aNErr = null;
        try
        {
            aNRes = NativeFix(N.TopoDS.Face(aNS), theMode, out aNDone, out aNStatus);
            for (int i = 0; i < s_statuses.Length; i++)
            {
                myStatusCounts[i] += (aNStatus >> i) & 1;
            }
            myFixed += aNDone ? 1 : 0;
            myChanged += aNRes.IsNull() || NativeText(aNRes) != NativeText(aNS) ? 1 : 0;
        }
        catch (N.OcctException e)
        {
            aNErr = ErrorOf(e);
        }

        Shape aPRes = new();
        bool aPDone = false;
        int aPStatus = 0;
        string? aPErr = null;
        try
        {
            aPRes = PortFix(ShapeCasts.Face(aPS), theMode, out aPDone, out aPStatus);
        }
        catch (Exception e) when (Refusal.IsFailure(e))
        {
            aPErr = ErrorOf(e);
        }

        myCompared++;
        if (PortText(aPS) == anInputText)
        {
            myUnchanged++;
        }
        else
        {
            myDifferences.Add($"{w}: the input changed");
        }
        if (aNErr is not null || aPErr is not null)
        {
            if (aNErr == aPErr)
            {
                myIdentical++;
                myExceptions++;
            }
            else
            {
                myDifferences.Add($"{w}: {aNErr ?? "no exception"} natively, {aPErr ?? "no exception"} in the port");
            }
            return;
        }

        if (aNDone != aPDone || aNStatus != aPStatus)
        {
            myDifferences.Add($"{w}: status {aNDone}/{aNStatus:X} natively, {aPDone}/{aPStatus:X} in the port");
        }
        if (aNRes!.IsNull() || aPRes.IsNull)
        {
            if (aNRes.IsNull() == aPRes.IsNull)
            {
                myIdentical++;
            }
            else
            {
                myDifferences.Add($"{w}: null result on one side only");
            }
            return;
        }
        string aNT = NativeText(aNRes), aPT = PortText(aPRes);
        BRepTextDiff aDiff = BRepTextComparison.Compare(aNT, aPT);
        if (aDiff.IsIdentical)
        {
            myIdentical++;
        }
        else if (aDiff.IsIdenticalMaskingFree)
        {
            myIdentical++;
            myFreeMasked++;
        }
        else if (MaskFreeAndChecked(aNT) == MaskFreeAndChecked(aPT))
        {
            // native set the Checked bit (BRepTools::Update) of a sub-shape of the caller's face in place, which the
            // port gives back as it is
            myIdentical++;
            myCheckedMasked++;
        }
        else
        {
            // both read from the native result's text: the canonical comparison of decision 581
            string aFile = NewFile();
            File.WriteAllText(aFile, aNT);
            Assert.True(BrepTools.Read(out Shape aNP, aFile));
            File.Delete(aFile);
            CanonicalComparison aCanon = CanonicalShapeComparer.Compare(aNP, aPRes, new CanonicalOptions { CheckValidity = false });
            if (Environment.GetEnvironmentVariable("KERF_SFF_DUMP") is string aDump && !aCanon.IsIdentical)
            {
                Directory.CreateDirectory(aDump);
                File.WriteAllText(Path.Combine(aDump, $"{myCompared}_native.brep"), aNT);
                File.WriteAllText(Path.Combine(aDump, $"{myCompared}_port.brep"), aPT);
                File.WriteAllText(Path.Combine(aDump, $"{myCompared}_input.brep"), anInputText);
                File.WriteAllText(Path.Combine(aDump, $"{myCompared}_what.txt"), w);
            }
            if (aCanon.IsIdentical)
            {
                myCanonical++;
            }
            else
            {
                myDifferences.Add($"{w}: {aDiff}; canonically {aCanon}");
            }
        }
    }

    /// <summary>
    /// The text with the Free and Checked flags of each TShape masked: two cache bits that native OCCT writes into the
    /// shapes it is given (BRepTools::Update marks a face Checked), where the port gives back the caller's frozen
    /// shapes as they are. The flag lines are found as BRepTextComparison finds them (TopTools_ShapeSet.Write).
    /// </summary>
    internal static string MaskFreeAndChecked(string theText)
    {
        string[] aLines = theText.Replace("\r\n", "\n", StringComparison.Ordinal).Split('\n');
        int i = Array.FindIndex(aLines, l => l.StartsWith("TShapes ", StringComparison.Ordinal));
        if (i < 0)
        {
            return theText;
        }
        int aNbShapes = int.Parse(aLines[i].AsSpan(8).Trim(), System.Globalization.CultureInfo.InvariantCulture);
        ++i;
        for (int aShape = 0; aShape < aNbShapes && i < aLines.Length; ++aShape)
        {
            ++i; // the type line
            while (i < aLines.Length && !(aLines[i].Length == 7 && aLines[i].All(c => c is '0' or '1')))
            {
                ++i;
            }
            if (i >= aLines.Length)
            {
                break;
            }
            aLines[i] = "x" + aLines[i][1] + "x" + aLines[i][3..];
            ++i;
            while (i < aLines.Length && !aLines[i].TrimEnd().EndsWith('*'))
            {
                ++i;
            }
            ++i;
        }
        return string.Join("\n", aLines);
    }

    // ---- inputs ----

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

    // Up to theMax sub-shapes of the type, evenly sampled, distinct by TShape (as an indexed map gives them).
    private static List<N.TopoDS_Shape> Sample(N.TopoDS_Shape theShape, N.TopAbs_ShapeEnum theType, int theMax)
    {
        using var aMap = new N.TopTools_IndexedMapOfShape();
        N.TopExp.MapShapes(theShape, theType, aMap);
        int aNb = aMap.Extent();
        var aList = new List<N.TopoDS_Shape>();
        int aStep = Math.Max(1, aNb / theMax);
        for (int i = 1; i <= aNb && aList.Count < theMax; i += aStep)
        {
            aList.Add(aMap.FindKey(i));
        }
        return aList;
    }

    private static List<string> HealGroupFiles(ITestOutputHelper theOutput)
    {
        var aFiles = new List<string>();
        foreach (string aFile in Directory.GetFiles(Path.Combine(Metadata("OcctSourceDir"), "data", "occ"), "*.brep").Order(StringComparer.Ordinal))
        {
            aFiles.Add(aFile);
        }
        string aRoot = Metadata("OcctTestDataDir");
        if (string.IsNullOrEmpty(aRoot) || !Directory.Exists(aRoot))
        {
            theOutput.WriteLine("OcctTestDataDir isn't set: data/occ only");
            return aFiles;
        }
        var aNames = new SortedSet<string>(StringComparer.Ordinal);
        var aLocate = new Regex(@"locate_data_file\s+([^\]\s]+)");
        foreach (string aCase in Directory.EnumerateFiles(Path.Combine(Metadata("OcctSourceDir"), "tests", "heal"), "*", SearchOption.AllDirectories))
        {
            foreach (Match aMatch in aLocate.Matches(File.ReadAllText(aCase)))
            {
                aNames.Add(aMatch.Groups[1].Value);
            }
        }
        var aByName = new Dictionary<string, string>(StringComparer.Ordinal);
        foreach (string aFile in Directory.EnumerateFiles(aRoot, "*", SearchOption.AllDirectories))
        {
            aByName.TryAdd(Path.GetFileName(aFile), aFile);
        }
        foreach (string aName in aNames)
        {
            if (aByName.TryGetValue(aName, out string? aFile)
                && (aName.EndsWith(".brep", StringComparison.OrdinalIgnoreCase) || aName.EndsWith(".rle", StringComparison.OrdinalIgnoreCase))
                && new FileInfo(aFile).Length <= 1_000_000)
            {
                aFiles.Add(aFile);
            }
        }
        return aFiles;
    }

    private static N.gp_Ax2 Ax2At(in Frame3D theAx, double theHeight)
    {
        Point3D aP = theAx.Location.Translated(new Vector3D(theAx.Direction) * theHeight);
        return new N.gp_Ax2(Oracle.ToNative(aP), Oracle.ToNative(theAx.Direction), Oracle.ToNative(theAx.XDirection));
    }

    private static N.TopoDS_Wire CircleWire(in Frame3D theAx, double theHeight, double theRadius, bool theReversed)
    {
        using var aCircle = new N.Geom_Circle(Ax2At(theAx, theHeight), theRadius);
        N.TopoDS_Wire aWire = new N.BRepBuilderAPI_MakeWire(new N.BRepBuilderAPI_MakeEdge(aCircle).Edge()).Wire();
        return theReversed ? N.TopoDS.Wire(aWire.Reversed()) : aWire;
    }

    // A face on a closed surface bounded by 3d circles (no pcurves), as build123d.NET's MakeFaceOnSurface builds it.
    private static N.TopoDS_Shape? SurfaceFace(Oracle theOracle, int theIndex, out string theWhat)
    {
        Frame3D anAx = theOracle.Ax2();
        var anAx3 = new N.gp_Ax3(Oracle.ToNative(anAx.Location), Oracle.ToNative(anAx.Direction), Oracle.ToNative(anAx.XDirection));
        double R = theOracle.Real(1.0, 20.0);
        bool aRev1 = theOracle.Int(0, 3) == 0, aRev2 = theOracle.Bool();
        N.Geom_Surface aSurf;
        var aWires = new List<N.TopoDS_Wire>();
        switch (theIndex % 7)
        {
            case 0: // cylinder band
            {
                aSurf = new N.Geom_CylindricalSurface(anAx3, R);
                double h = theOracle.Real(0.5, 20.0);
                aWires.Add(CircleWire(anAx, 0.0, R, aRev1));
                aWires.Add(CircleWire(anAx, h, R, aRev2));
                theWhat = $"cylinder band, r {R}, h {h}";
                break;
            }
            case 1: // cylinder with one circle
            {
                aSurf = new N.Geom_CylindricalSurface(anAx3, R);
                aWires.Add(CircleWire(anAx, theOracle.Real(-5.0, 5.0), R, aRev1));
                theWhat = $"cylinder, one circle, r {R}";
                break;
            }
            case 2: // sphere cap
            {
                aSurf = new N.Geom_SphericalSurface(anAx3, R);
                double v = theOracle.Real(-1.2, 1.2);
                aWires.Add(CircleWire(anAx, R * Math.Sin(v), R * Math.Cos(v), aRev1));
                theWhat = $"sphere cap, r {R}, v {v}";
                break;
            }
            case 3: // sphere zone
            {
                aSurf = new N.Geom_SphericalSurface(anAx3, R);
                double v1 = theOracle.Real(-1.2, 0.0), v2 = theOracle.Real(0.1, 1.2);
                aWires.Add(CircleWire(anAx, R * Math.Sin(v1), R * Math.Cos(v1), aRev1));
                aWires.Add(CircleWire(anAx, R * Math.Sin(v2), R * Math.Cos(v2), aRev2));
                theWhat = $"sphere zone, r {R}, v {v1}..{v2}";
                break;
            }
            case 4: // cone frustum
            case 5: // cone, one circle (the apex)
            {
                double anAng = theOracle.Real(0.1, 1.2);
                aSurf = new N.Geom_ConicalSurface(anAx3, anAng, R);
                double v1 = theOracle.Real(-R / Math.Sin(anAng) * 0.8, 0.0), v2 = theOracle.Real(0.5, 10.0);
                aWires.Add(CircleWire(anAx, v2 * Math.Cos(anAng), R + v2 * Math.Sin(anAng), aRev1));
                if (theIndex % 7 == 4)
                {
                    aWires.Add(CircleWire(anAx, v1 * Math.Cos(anAng), R + v1 * Math.Sin(anAng), aRev2));
                }
                theWhat = $"cone{(theIndex % 7 == 5 ? ", one circle" : "")}, r {R}, angle {anAng}";
                break;
            }
            default: // torus band (two parallels)
            {
                double r = theOracle.Real(0.1, 0.9) * R;
                aSurf = new N.Geom_ToroidalSurface(anAx3, R, r);
                double v1 = theOracle.Real(-3.0, 0.0), v2 = theOracle.Real(0.1, 3.0);
                aWires.Add(CircleWire(anAx, r * Math.Sin(v1), R + r * Math.Cos(v1), aRev1));
                aWires.Add(CircleWire(anAx, r * Math.Sin(v2), R + r * Math.Cos(v2), aRev2));
                theWhat = $"torus band, R {R}, r {r}, v {v1}..{v2}";
                break;
            }
        }
        theWhat += $"{(aRev1 ? ", first reversed" : "")}{(aRev2 ? ", second reversed" : "")}";
        try
        {
            using var aMaker = new N.BRepBuilderAPI_MakeFace(aSurf, aWires[0], true);
            for (int i = 1; i < aWires.Count; i++)
            {
                aMaker.Add(aWires[i]);
            }
            return aMaker.IsDone() ? aMaker.Face() : null;
        }
        catch (N.OcctException)
        {
            return null;
        }
    }

    // A planar face from a polygon or arcs and holes, some holes misoriented, as build123d.NET's MakeFaceFromWires.
    private static N.TopoDS_Shape? PlanarFace(Oracle theOracle, out string theWhat)
    {
        Frame3D anAx = theOracle.Ax2();
        double R = theOracle.Real(5.0, 50.0);
        int aNb = theOracle.Int(3, 9);
        N.gp_Pnt Point(double theU, double theV)
        {
            Point3D aP = anAx.Location.Translated(new Vector3D(anAx.XDirection) * theU).Translated(new Vector3D(anAx.YDirection) * theV);
            return Oracle.ToNative(aP);
        }
        bool anOuterReversed = theOracle.Int(0, 3) == 0;
        using var aPoly = new N.BRepBuilderAPI_MakePolygon();
        for (int i = 0; i < aNb; i++)
        {
            double a = 2.0 * Math.PI * i / aNb;
            double r = R * theOracle.Real(0.8, 1.2);
            aPoly.Add(Point(r * Math.Cos(a), r * Math.Sin(a)));
        }
        aPoly.Close();
        N.TopoDS_Wire anOuter = aPoly.Wire();
        if (anOuterReversed)
        {
            anOuter = N.TopoDS.Wire(anOuter.Reversed());
        }
        int aNbHoles = theOracle.Int(0, 3);
        int aNbMisoriented = 0;
        try
        {
            using var aMaker = new N.BRepBuilderAPI_MakeFace(anOuter, true);
            for (int k = 0; k < aNbHoles; k++)
            {
                double a = 2.0 * Math.PI * k / Math.Max(1, aNbHoles);
                double c = aNbHoles == 1 ? 0.0 : 0.45 * R;
                double hr = R * (aNbHoles == 1 ? 0.3 : 0.15);
                var aCircleAx = new N.gp_Ax2(Point(c * Math.Cos(a), c * Math.Sin(a)), Oracle.ToNative(anAx.Direction), Oracle.ToNative(anAx.XDirection));
                using var aCircle = new N.Geom_Circle(aCircleAx, hr);
                N.TopoDS_Wire aHole = new N.BRepBuilderAPI_MakeWire(new N.BRepBuilderAPI_MakeEdge(aCircle).Edge()).Wire();
                bool isMisoriented = theOracle.Bool();
                aNbMisoriented += isMisoriented ? 1 : 0;
                aMaker.Add(isMisoriented ? aHole : N.TopoDS.Wire(aHole.Reversed()));
            }
            theWhat = $"planar, {aNb} sides{(anOuterReversed ? " reversed" : "")}, {aNbHoles} holes ({aNbMisoriented} misoriented)";
            return aMaker.IsDone() ? aMaker.Face() : null;
        }
        catch (N.OcctException)
        {
            theWhat = "planar (failed)";
            return null;
        }
    }

    private void Report(string theWhat, TimeSpan theTime)
    {
        output.WriteLine($"{theWhat}: {myIdentical}/{myCompared} results identical as BRep text ({myFreeMasked} with the Free column masked, {myCheckedMasked} with the Free and Checked columns masked, "
                         + $"{myExceptions} the same exception on both sides), {myCanonical} canonically identical, {myDifferences.Count} different; "
                         + $"inputs unchanged {myUnchanged}; native fixes reported {myFixed}, native results differing from their input {myChanged}; {theTime.TotalSeconds:F0} s");
        output.WriteLine("  native statuses: " + string.Join(", ", s_statuses.Select((s, i) => $"{s} {myStatusCounts[i]}").Where((s, i) => myStatusCounts[i] > 0)));
        foreach (string aD in myDifferences.Take(80))
        {
            output.WriteLine("  " + aD);
        }
    }

    [Fact]
    public void ShapeFix_Face_on_faces_as_build123d_makes_them_matches_native_OCCT()
    {
        var aWatch = Stopwatch.StartNew();
        var anOracle = new Oracle(output, 63101);
        foreach (int i in Tape.Budget(280))
        {
            if (SurfaceFace(anOracle, i, out string aWhat) is N.TopoDS_Shape aFace)
            {
                Compare(aFace, $"face {i} ({aWhat})", Mode.Perform);
                Compare(aFace, $"face {i} ({aWhat})", i % 2 == 0 ? Mode.SurfaceAndWires : Mode.SurfaceAndWiresReversed);
            }
        }
        foreach (int i in Tape.Budget(150))
        {
            if (PlanarFace(anOracle, out string aWhat) is N.TopoDS_Shape aFace)
            {
                Compare(aFace, $"planar face {i} ({aWhat})", Mode.OrientationThenPerform);
                Compare(aFace, $"planar face {i} ({aWhat})", Mode.Perform);
            }
        }
        Report("ShapeFix_Face on faces as build123d.NET makes them", aWatch.Elapsed);
        Assert.Empty(myDifferences);
    }

    [Fact]
    public void ShapeFix_Face_on_the_faces_of_the_heal_group_and_data_occ_matches_native_OCCT()
    {
        var aWatch = Stopwatch.StartNew();
        int aShapes = 0;
        foreach (string aFile in Tape.Budget(HealGroupFiles(output), isSpread: true))
        {
            if (NativeRead(aFile) is not N.TopoDS_Shape aShape)
            {
                continue;
            }
            aShapes++;
            int i = 0;
            foreach (N.TopoDS_Shape aFace in Sample(aShape, N.TopAbs_ShapeEnum.TopAbs_FACE, 10))
            {
                Compare(aFace, $"{Path.GetFileName(aFile)}, face {i}", Mode.Perform);
                if (i % 3 == 0)
                {
                    Compare(aFace, $"{Path.GetFileName(aFile)}, face {i}", Mode.SurfaceAndWires);
                }
                i++;
            }
        }
        output.WriteLine($"{aShapes} shapes");
        Report("ShapeFix_Face on the faces of the heal group's and data/occ's shapes", aWatch.Elapsed);
        Assert.Empty(myDifferences);
    }
}
