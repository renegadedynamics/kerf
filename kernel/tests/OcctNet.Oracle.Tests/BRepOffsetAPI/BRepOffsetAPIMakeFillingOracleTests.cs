// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Brep;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Diagnostics;
using System.Reflection;
using OcctNet.TestHarness;
using Xunit.Abstractions;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// BRepOffsetAPI_MakeFilling (and through it BRepFill_Filling, GeomPlate, Plate, NLPlate's absence, LocalAnalysis)
/// against native OCCT.
/// </summary>
/// <remarks>
/// <para>
/// Inputs are built natively as one compound (boundary edges, support faces), written to a file both sides read; the
/// port's copy is frozen, as a caller's is. The operations are then replayed on both sides in the same order:
/// build123d.NET's <c>Face.MakeSurface</c> (the filling of its parameters, C0 edges, built, then points added and built
/// again), <c>Face.MakeSurfacePatch</c> (default parameters, edges with support faces at G1 or G2, C0 edges,
/// points), points on faces, free constraints, an initial surface.
/// </para>
/// <para>
/// Compared after each build: <c>IsDone</c>, the exception by OCCT type, the face as BRep text with the Free column
/// masked, then canonically (decision 581) where the texts differ; <c>G0Error</c>, <c>G1Error</c>, <c>G2Error</c>
/// and the per-constraint errors (bits, or the same exception); <c>Generated</c> of every input edge and vertex (as
/// BRep text); and that the port's input is unchanged.
/// </para>
/// </remarks>
public class BRepOffsetAPIMakeFillingOracleTests(ITestOutputHelper output) : IDisposable
{
    private readonly string myDirectory = Directory.CreateTempSubdirectory("occtnet-fill-").FullName;
    private int myFileNumber;
    private readonly List<string> myDifferences = [];
    private readonly SortedDictionary<string, int> myExceptionTypes = new(StringComparer.Ordinal);
    private int myBuilds, myIdentical, myFreeMasked, myCanonical, myExceptions, myNotDone, myValues, myValuesEqual,
                myGenerated, myGeneratedEqual, myUnchanged, myCases, myNativeVaries, myNativeUninitialised;

    public void Dispose()
    {
        Directory.Delete(myDirectory, true);
        GC.SuppressFinalize(this);
    }

    private static string Metadata(string theKey)
        => typeof(BRepOffsetAPIMakeFillingOracleTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == theKey).Value!;

    private string NewFile() => Path.Combine(myDirectory, $"s{myFileNumber++}.brep");

    private string NativeText(N.TopoDS_Shape theShape)
    {
        string aFile = NewFile();
        Assert.True(N.BRepTools.Write(theShape, aFile, false, false, N.TopTools_FormatVersion.TopTools_FormatVersion_VERSION_3));
        string aText = File.ReadAllText(aFile);
        File.Delete(aFile);
        return aText;
    }

    private string PortText(in Shape theShape)
    {
        string aFile = NewFile();
        Assert.True(BrepTools.Write(theShape, aFile, false, false, FormatVersion.FormatVersion_VERSION_3));
        string aText = File.ReadAllText(aFile);
        File.Delete(aFile);
        return aText;
    }

    private static string? ErrorOf(Exception theException) => theException switch
    {
        N.OcctException e => e.Message.IndexOf(':') is int c and > 0 ? e.Message[..c] : e.Message,
        FailureException f => global::Kerf.Engine.Foundation.OcctTypeNames.Of(f.GetType()),
        _ => global::Kerf.Engine.Foundation.OcctTypeNames.Of(theException.GetType()),
    };

    // ---- the operations replayed on both sides ----

    private enum Op
    {
        Edge,       // Add(edge A, order, bound)
        EdgeFace,   // Add(edge A, face B, order, bound)
        Face,       // Add(face A, order)
        Point,      // Add(point)
        PointOnFace,// Add(u, v, face A, order)
        Init,       // LoadInitSurface(face A)
        Build,      // Build() and compare
    }

    private sealed record Step(Op Kind, int A = 0, int B = 0, Continuity Order = Continuity.C0, bool Bound = true,
                               Point3D Point = default, double U = 0, double V = 0);

    private sealed class FillingCase
    {
        public required string What;
        public bool Build123dParams;
        public required List<N.TopoDS_Shape> Inputs;
        public List<Step> Steps = [];
        public Point3D Center;
    }

    private static N.TopoDS_Shape NativeCompound(IEnumerable<N.TopoDS_Shape> theShapes)
    {
        var aC = new N.TopoDS_Compound();
        using var aB = new N.BRep_Builder();
        aB.MakeCompound(aC);
        foreach (N.TopoDS_Shape aS in theShapes)
        {
            aB.Add(aC, aS);
        }
        return aC;
    }

    private static IEnumerable<N.TopoDS_Shape> NativeChildren(N.TopoDS_Shape theShape)
    {
        for (var anIt = new N.TopoDS_Iterator(theShape); anIt.More(); anIt.Next())
        {
            yield return anIt.Value();
        }
    }

    private static List<Shape> PortChildren(in Shape theShape)
    {
        var aList = new List<Shape>();
        for (var anIt = new Iterator(theShape); anIt.More(); anIt.Next())
        {
            aList.Add(anIt.Value());
        }
        return aList;
    }

    private void Run(FillingCase theCase)
    {
        myCases++;
        string aFile = NewFile();
        N.TopoDS_Shape aNCompound = NativeCompound(theCase.Inputs);
        Assert.True(N.BRepTools.Write(aNCompound, aFile, false, false, N.TopTools_FormatVersion.TopTools_FormatVersion_VERSION_3));
        var aNShape = new N.TopoDS_Shape();
        using (var aBuilder = new N.BRep_Builder())
        {
            Assert.True(N.BRepTools.Read(aNShape, aFile, aBuilder));
        }
        Assert.True(BrepTools.Read(out Shape aPShape, aFile), aFile);
        File.Delete(aFile);
        aPShape.TShape()!.FreezeGraph(); // a caller's shapes
        string anInputText = PortText(aPShape);
        List<N.TopoDS_Shape> aNIn = NativeChildren(aNShape).ToList();
        List<Shape> aPIn = PortChildren(aPShape);

        using var aN = theCase.Build123dParams
            ? new N.BRepOffsetAPI_MakeFilling(3, 15, 2, false, 0.00001, 0.0001, 0.01, 0.1, 8, 9)
            : new N.BRepOffsetAPI_MakeFilling();
        var aP = theCase.Build123dParams
            ? new BRepOffsetAPI_MakeFilling(3, 15, 2, false, 0.00001, 0.0001, 0.01, 0.1, 8, 9)
            : new BRepOffsetAPI_MakeFilling();
        int aNbBuild = 0;
        int aNbConstraints = 0;
        foreach (Step aStep in theCase.Steps)
        {
            string w = $"{theCase.What}, build {aNbBuild + 1}";
            switch (aStep.Kind)
            {
                case Op.Edge:
                    Same(w + ": Add index", aN.Add(N.TopoDS.Edge(aNIn[aStep.A]), (N.GeomAbs_Shape)(int)aStep.Order, aStep.Bound),
                         aP.Add(ShapeCasts.Edge(aPIn[aStep.A]), aStep.Order, aStep.Bound));
                    aNbConstraints++;
                    break;
                case Op.EdgeFace:
                    Same(w + ": Add index", aN.Add(N.TopoDS.Edge(aNIn[aStep.A]), N.TopoDS.Face(aNIn[aStep.B]), (N.GeomAbs_Shape)(int)aStep.Order, aStep.Bound),
                         aP.Add(ShapeCasts.Edge(aPIn[aStep.A]), ShapeCasts.Face(aPIn[aStep.B]), aStep.Order, aStep.Bound));
                    aNbConstraints++;
                    break;
                case Op.Face:
                    Same(w + ": Add index", aN.Add(N.TopoDS.Face(aNIn[aStep.A]), (N.GeomAbs_Shape)(int)aStep.Order),
                         aP.Add(ShapeCasts.Face(aPIn[aStep.A]), aStep.Order));
                    break;
                case Op.Point:
                    Same(w + ": Add index", aN.Add(Oracle.ToNative(aStep.Point)), aP.Add(aStep.Point));
                    break;
                case Op.PointOnFace:
                    Same(w + ": Add index", aN.Add(aStep.U, aStep.V, N.TopoDS.Face(aNIn[aStep.A]), (N.GeomAbs_Shape)(int)aStep.Order),
                         aP.Add(aStep.U, aStep.V, ShapeCasts.Face(aPIn[aStep.A]), aStep.Order));
                    break;
                case Op.Init:
                    aN.LoadInitSurface(N.TopoDS.Face(aNIn[aStep.A]));
                    aP.LoadInitSurface(ShapeCasts.Face(aPIn[aStep.A]));
                    break;
                case Op.Build:
                    aNbBuild++;
                    CompareBuild(w, aN, aP, aNIn, aPIn, aNbConstraints);
                    break;
            }
        }
        if (PortText(aPShape) == anInputText)
        {
            myUnchanged++;
        }
        else
        {
            myDifferences.Add($"{theCase.What}: the input changed");
        }
    }

    private void Same(string theWhat, int theNative, int thePort)
    {
        myValues++;
        if (theNative == thePort)
        {
            myValuesEqual++;
        }
        else
        {
            myDifferences.Add($"{theWhat}: native {theNative}, port {thePort}");
        }
    }

    private static string Evaluate(Func<double> theValue)
    {
        try
        {
            double aValue = theValue();
            return BitConverter.DoubleToInt64Bits(aValue).ToString("X16") + $" ({aValue:R})";
        }
        catch (Exception e)
        {
            return ErrorOf(e) ?? global::Kerf.Engine.Foundation.OcctTypeNames.Of(e.GetType());
        }
    }

    private static double? ValueOf(string theEvaluated)
        => theEvaluated.Length > 16 && long.TryParse(theEvaluated.AsSpan(0, 16), System.Globalization.NumberStyles.HexNumber, null, out long aBits)
            ? BitConverter.Int64BitsToDouble(aBits) : null;

    private void SameValue(string theWhat, Func<double> theNative, Func<double> thePort)
    {
        myValues++;
        string aN = Evaluate(theNative), aP = Evaluate(thePort);
        if (aN == aP)
        {
            myValuesEqual++;
        }
        else
        {
            myDifferences.Add($"{theWhat}: native {aN}, port {aP}");
        }
    }

    // OCCT's G0Error(Index) to G2Error(Index) read array elements EcartContraintesMil didn't write when the curve has
    // fewer intervals than NbPtsOnCur: natively uninitialised memory, so the value may change from call to call
    // (occt-issues.md); the port's arrays are zeroed. Native is evaluated three times: where it varies, the port's
    // value must be one of native's. Otherwise the port's curve must have had fewer intervals than NbPtsOnCur (the
    // uninitialised tail is read) and native's values must all be at least the port's (a max over more values).
    private void SameIndexedValue(string theWhat, Func<double> theNative, Func<double> thePort, bool theReadsUninitialised)
    {
        myValues++;
        string[] aN = [Evaluate(theNative), Evaluate(theNative), Evaluate(theNative)];
        string aP = Evaluate(thePort);
        if (aN.All(v => v == aP))
        {
            myValuesEqual++;
        }
        else if (aN.Distinct().Count() > 1 && aN.Contains(aP))
        {
            myValuesEqual++;
            myNativeVaries++;
        }
        else if (theReadsUninitialised && ValueOf(aP) is double aPV && aN.All(v => ValueOf(v) is double aNV && aNV >= aPV))
        {
            myValuesEqual++;
            myNativeUninitialised++;
        }
        else
        {
            myDifferences.Add($"{theWhat}: native {string.Join(" / ", aN.Distinct())}, port {aP}");
        }
    }

    private void CompareBuild(string w, N.BRepOffsetAPI_MakeFilling aN, BRepOffsetAPI_MakeFilling aP, List<N.TopoDS_Shape> aNIn,
                              List<Shape> aPIn, int theNbConstraints)
    {
        myBuilds++;
        string? aNErr = null, aPErr = null, aNMessage = null;
        N.TopoDS_Shape? aNRes = null;
        Shape aPRes = new();
        try
        {
            aN.Build();
            aNRes = aN.IsDone() ? aN.Shape() : null;
        }
        catch (N.OcctException e)
        {
            aNErr = ErrorOf(e);
            aNMessage = e.Message;
        }
        try
        {
            aP.Build();
            aPRes = aP.IsDone() ? aP.Shape() : new Shape();
        }
        catch (Exception e)
        {
            aPErr = ErrorOf(e);
        }
        if (aNErr is not null || aPErr is not null)
        {
            if (aNErr == aPErr)
            {
                myIdentical++;
                myExceptions++;
                myExceptionTypes[aNMessage!] = myExceptionTypes.GetValueOrDefault(aNMessage!) + 1;
            }
            else
            {
                myDifferences.Add($"{w}: {aNErr ?? "no exception"} natively, {aPErr ?? "no exception"} in the port");
            }
            return;
        }
        if (aNRes is null || aPRes.IsNull)
        {
            if ((aNRes is null) == aPRes.IsNull)
            {
                myIdentical++;
                myNotDone++;
            }
            else
            {
                myDifferences.Add($"{w}: done {aNRes is not null} natively, {!aPRes.IsNull} in the port");
            }
            return;
        }
        Assert.True(aPRes.TShape()!.IsFrozen);

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
        else
        {
            string aFile = NewFile();
            File.WriteAllText(aFile, aNT);
            Assert.True(BrepTools.Read(out Shape aNP, aFile));
            File.Delete(aFile);
            CanonicalComparison aCanon = CanonicalShapeComparer.Compare(aNP, aPRes, new CanonicalOptions { CheckValidity = false });
            if (Environment.GetEnvironmentVariable("OCCTNET_FILL_DUMP") is string aDump)
            {
                Directory.CreateDirectory(aDump);
                File.WriteAllText(Path.Combine(aDump, $"{myBuilds}_native.brep"), aNT);
                File.WriteAllText(Path.Combine(aDump, $"{myBuilds}_port.brep"), aPT);
                File.WriteAllText(Path.Combine(aDump, $"{myBuilds}_what.txt"), w);
            }
            if (aCanon.IsIdentical)
            {
                myCanonical++;
            }
            else
            {
                myDifferences.Add($"{w}: {aDiff}; canonically {aCanon}; surfaces {SurfaceDeviation(aNP, aPRes)}");
            }
        }

        SameValue(w + ": G0Error", aN.G0Error, aP.G0Error);
        SameValue(w + ": G1Error", aN.G1Error, aP.G1Error);
        SameValue(w + ": G2Error", aN.G2Error, aP.G2Error);
        GeomPlate_BuildPlateSurface aBuilder = aP.InternalFilling().InternalBuilder()!;
        for (int i = 1; i <= theNbConstraints; i++)
        {
            bool aReadsUninitialised = aBuilder.NbIntervalsChecked(i) < aBuilder.NbPtsOnCur();
            SameIndexedValue(w + $": G0Error({i})", () => aN.G0Error(i), () => aP.G0Error(i), aReadsUninitialised);
            SameIndexedValue(w + $": G1Error({i})", () => aN.G1Error(i), () => aP.G1Error(i), aReadsUninitialised);
            SameIndexedValue(w + $": G2Error({i})", () => aN.G2Error(i), () => aP.G2Error(i), aReadsUninitialised);
        }

        // Generated, for each input edge and vertex
        for (int k = 0; k < aNIn.Count; k++)
        {
            var aNSubs = new List<N.TopoDS_Shape>();
            var aPSubs = new List<Shape>();
            for (var anExp = new N.TopExp_Explorer(aNIn[k], N.TopAbs_ShapeEnum.TopAbs_EDGE); anExp.More(); anExp.Next())
            {
                aNSubs.Add(anExp.Current());
            }
            for (var anExp = new N.TopExp_Explorer(aNIn[k], N.TopAbs_ShapeEnum.TopAbs_VERTEX); anExp.More(); anExp.Next())
            {
                aNSubs.Add(anExp.Current());
            }
            for (var anExp = new Explorer(aPIn[k], ShapeType.Edge); anExp.More(); anExp.Next())
            {
                aPSubs.Add(anExp.Current());
            }
            for (var anExp = new Explorer(aPIn[k], ShapeType.Vertex); anExp.More(); anExp.Next())
            {
                aPSubs.Add(anExp.Current());
            }
            Assert.Equal(aNSubs.Count, aPSubs.Count);
            for (int s = 0; s < aNSubs.Count; s++)
            {
                myGenerated++;
                List<string> aNG = aN.Generated(aNSubs[s]).Select(x => BRepTextComparison.MaskFreeFlags(NativeText(x))).ToList();
                List<string> aPG = aP.Generated(aPSubs[s]).Select(x => BRepTextComparison.MaskFreeFlags(PortText(x))).ToList();
                if (aNG.SequenceEqual(aPG))
                {
                    myGeneratedEqual++;
                }
                else
                {
                    myDifferences.Add($"{w}: Generated of input {k} sub-shape {s}: {aNG.Count} natively, {aPG.Count} in the port, texts differ");
                }
            }
        }
    }

    // The largest distance between the two faces' surfaces on a grid of the native face's UV bounds.
    private static string SurfaceDeviation(in Shape theNative, in Shape thePort)
    {
        if (theNative.ShapeType() != ShapeType.Face || thePort.ShapeType() != ShapeType.Face)
        {
            return "(not faces)";
        }
        Surface aNS = Tool.Surface(ShapeCasts.Face(theNative))!, aPS = Tool.Surface(ShapeCasts.Face(thePort))!;
        BrepTools.UVBounds(ShapeCasts.Face(theNative), out double u1, out double u2, out double v1, out double v2);
        double aMax = 0.0;
        for (int i = 0; i <= 20; i++)
        {
            for (int j = 0; j <= 20; j++)
            {
                double u = u1 + (u2 - u1) * i / 20, v = v1 + (v2 - v1) * j / 20;
                aMax = Math.Max(aMax, aNS.Value(u, v).Distance(aPS.Value(u, v)));
            }
        }
        return $"max deviation {aMax:G3}";
    }

    private void Report(string theWhat, TimeSpan theTime)
    {
        output.WriteLine($"{theWhat}: {myCases} cases, {myBuilds} builds: {myIdentical} identical as BRep text ({myFreeMasked} with the Free column masked, "
                         + $"{myExceptions} the same exception, {myNotDone} not done on both sides), {myCanonical} canonically identical; "
                         + $"values {myValuesEqual}/{myValues} bit-identical ({myNativeVaries} where native varies from call to call, {myNativeUninitialised} where native reads uninitialised memory); Generated {myGeneratedEqual}/{myGenerated}; inputs unchanged {myUnchanged}/{myCases}; "
                         + $"{myDifferences.Count} differences; {theTime.TotalSeconds:F0} s");
        if (myExceptionTypes.Count > 0)
        {
            output.WriteLine("  exceptions on both sides: " + string.Join(", ", myExceptionTypes.Select(p => $"{p.Key} {p.Value}")));
        }
        foreach (string aD in myDifferences.Take(80))
        {
            output.WriteLine("  " + aD);
        }
    }

    // ---- inputs ----

    private static N.TopoDS_Vertex NVertex(in Point3D theP) => new N.BRepBuilderAPI_MakeVertex(Oracle.ToNative(theP)).Vertex();

    // An edge between two vertices: straight, an arc through a lifted midpoint, or a B-spline through jittered points.
    private static N.TopoDS_Edge NEdge(Oracle theOracle, N.TopoDS_Vertex theV1, in Point3D theP1, N.TopoDS_Vertex theV2, in Point3D theP2,
                                       int theKind, out string theWhat)
    {
        switch (theKind)
        {
            case 1:
            {
                Point3D aMid = new Point3D((theP1.XYZ() + theP2.XYZ()) * 0.5).Translated(theOracle.Vec() * (0.02 * theP1.Distance(theP2) / 100.0 * theOracle.Real(1, 10)));
                using var anArc = new N.GC_MakeArcOfCircle(Oracle.ToNative(theP1), Oracle.ToNative(aMid), Oracle.ToNative(theP2));
                theWhat = "arc";
                return new N.BRepBuilderAPI_MakeEdge(anArc.Value(), theV1, theV2).Edge();
            }
            case 2:
            {
                int aNb = theOracle.Int(3, 6);
                var aPoles = new NCollection_Array1<Point3D>(1, aNb);
                for (int i = 1; i <= aNb; i++)
                {
                    double t = (i - 1.0) / (aNb - 1.0);
                    Point3D aP = new Point3D(theP1.XYZ() * (1 - t) + theP2.XYZ() * t);
                    aPoles[i] = i == 1 || i == aNb ? aP : aP.Translated(theOracle.Vec() * (0.002 * theP1.Distance(theP2)));
                }
                using var anApprox = new N.GeomAPI_PointsToBSpline(Oracle.ToNative(aPoles));
                theWhat = "bspline";
                return new N.BRepBuilderAPI_MakeEdge(anApprox.Curve(), theV1, theV2).Edge();
            }
            default:
                theWhat = "line";
                return new N.BRepBuilderAPI_MakeEdge(theV1, theV2).Edge();
        }
    }

    // A closed contour of edges on shared vertices around a random non-planar polygon (as build123d's MakeSurface takes).
    private static List<N.TopoDS_Shape> Contour(Oracle theOracle, int theMaxKind, out string theWhat)
    {
        Frame3D anAx = theOracle.Ax2();
        double R = theOracle.Real(2.0, 50.0);
        int aNb = theOracle.Int(3, 7);
        var aPnts = new Point3D[aNb];
        for (int i = 0; i < aNb; i++)
        {
            double a = 2.0 * Math.PI * i / aNb + theOracle.Real(-0.2, 0.2);
            double r = R * theOracle.Real(0.7, 1.3);
            aPnts[i] = anAx.Location().Translated(new Vector3D(anAx.XDirection()) * (r * Math.Cos(a)))
                .Translated(new Vector3D(anAx.YDirection()) * (r * Math.Sin(a)))
                .Translated(new Vector3D(anAx.Direction()) * (R * theOracle.Real(-0.4, 0.4)));
        }
        var aVs = aPnts.Select(p => NVertex(p)).ToArray();
        var anEdges = new List<N.TopoDS_Shape>();
        var aKinds = new List<string>();
        for (int i = 0; i < aNb; i++)
        {
            int j = (i + 1) % aNb;
            anEdges.Add(NEdge(theOracle, aVs[i], aPnts[i], aVs[j], aPnts[j], theOracle.Int(0, theMaxKind), out string aKind));
            aKinds.Add(aKind);
        }
        theWhat = $"{aNb} edges ({string.Join(", ", aKinds)}), R {R:G4}";
        return anEdges;
    }

    private static Point3D InsidePoint(Oracle theOracle, List<N.TopoDS_Shape> theEdges)
    {
        // a point near the centroid of the edges' vertices, lifted a little
        var aSum = new Xyz(0, 0, 0);
        int aNb = 0;
        double aSize = 0.0;
        Point3D? aPrev = null;
        foreach (N.TopoDS_Shape anEdge in theEdges)
        {
            var aV1 = new N.TopoDS_Vertex();
            var aV2 = new N.TopoDS_Vertex();
            N.TopExp.Vertices(N.TopoDS.Edge(anEdge), aV1, aV2);
            using var aP = N.BRep_Tool.Pnt(aV1);
            var aPnt = new Point3D(aP.X(), aP.Y(), aP.Z());
            aSum += aPnt.XYZ();
            aNb++;
            if (aPrev is Point3D aQ)
            {
                aSize = Math.Max(aSize, aQ.Distance(aPnt));
            }
            aPrev = aPnt;
        }
        return new Point3D(aSum / aNb).Translated(theOracle.Vec() * (aSize * 0.001 * theOracle.Real(0, 2)));
    }

    [Fact]
    public void MakeFilling_as_build123d_MakeSurface_matches_native_OCCT()
    {
        var aWatch = Stopwatch.StartNew();
        var anOracle = new Oracle(output, 5941);
        foreach (int c in Tape.Budget(60))
        {
            List<N.TopoDS_Shape> anEdges = Contour(anOracle, c < 20 ? 0 : 2, out string aWhat);
            var aCase = new FillingCase { What = $"MakeSurface {c} ({aWhat})", Build123dParams = true, Inputs = anEdges };
            for (int i = 0; i < anEdges.Count; i++)
            {
                aCase.Steps.Add(new Step(Op.Edge, i, Order: Continuity.C0));
            }
            aCase.Steps.Add(new Step(Op.Build));
            int aNbPoints = c % 3;
            for (int k = 0; k < aNbPoints; k++)
            {
                aCase.Steps.Add(new Step(Op.Point, Point: InsidePoint(anOracle, anEdges)));
            }
            if (aNbPoints > 0)
            {
                aCase.Steps.Add(new Step(Op.Build));
            }
            Run(aCase);
        }
        Report("MakeFilling as build123d.NET's MakeSurface", aWatch.Elapsed);
        Assert.Empty(myDifferences);
    }

    // A box's faces but the top; the top's edges, each with the side face it bounds.
    private static FillingCase BoxHole(Oracle theOracle, int theIndex, Continuity theOrder, bool theFreeConstraint)
    {
        Frame3D anAx = theOracle.Ax2();
        double dx = theOracle.Real(1, 30), dy = theOracle.Real(1, 30), dz = theOracle.Real(1, 30);
        using var aBox = new N.BRepPrimAPI_MakeBox(Oracle.ToNative(anAx), dx, dy, dz);
        N.TopoDS_Face[] aSides = [aBox.FrontFace(), aBox.RightFace(), aBox.BackFace(), aBox.LeftFace()];
        var aTopEdges = new List<N.TopoDS_Shape>();
        for (var anExp = new N.TopExp_Explorer(aBox.TopFace(), N.TopAbs_ShapeEnum.TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            aTopEdges.Add(anExp.Current());
        }
        var anInputs = new List<N.TopoDS_Shape>(aSides);
        anInputs.AddRange(aTopEdges);
        anInputs.Add(aBox.TopFace()); // input 8, for the initial surface
        var aCase = new FillingCase
        {
            Center = anAx.Location().Translated(new Vector3D(anAx.XDirection()) * (dx / 2)).Translated(new Vector3D(anAx.YDirection()) * (dy / 2))
                .Translated(new Vector3D(anAx.Direction()) * (dz * theOracle.Real(0.9, 1.3))),
            What = $"box hole {theIndex} ({dx:G3} x {dy:G3} x {dz:G3}, {theOrder}{(theFreeConstraint ? ", one side free" : "")})",
            Inputs = anInputs,
        };
        for (int e = 0; e < aTopEdges.Count; e++)
        {
            int aFace = -1;
            for (int f = 0; f < 4 && aFace < 0; f++)
            {
                for (var anExp = new N.TopExp_Explorer(aSides[f], N.TopAbs_ShapeEnum.TopAbs_EDGE); anExp.More(); anExp.Next())
                {
                    if (anExp.Current().IsSame(aTopEdges[e]))
                    {
                        aFace = f;
                        break;
                    }
                }
            }
            Assert.True(aFace >= 0);
            if (theFreeConstraint && e == 0)
            {
                aCase.Steps.Add(new Step(Op.Face, aFace, Order: theOrder));
            }
            else
            {
                aCase.Steps.Add(new Step(Op.EdgeFace, 4 + e, aFace, theOrder));
            }
        }
        return aCase;
    }

    // A cylinder's lateral face (or half of it) and its top circle(s).
    private static FillingCase CylinderCap(Oracle theOracle, int theIndex, Continuity theOrder)
    {
        Frame3D anAx = theOracle.Ax2();
        double R = theOracle.Real(1, 20), H = theOracle.Real(1, 20);
        using var aCyl = new N.BRepPrimAPI_MakeCylinder(Oracle.ToNative(anAx), R, H);
        N.TopoDS_Face aLateral = aCyl.Face();
        var anInputs = new List<N.TopoDS_Shape> { aLateral };
        var aCase = new FillingCase { What = $"cylinder cap {theIndex} (R {R:G3}, H {H:G3}, {theOrder})", Inputs = anInputs };
        // the circle at the top: the edge of the lateral face whose vertex is farthest along the axis
        N.TopoDS_Shape? aTop = null;
        double aBest = double.MinValue;
        for (var anExp = new N.TopExp_Explorer(aLateral, N.TopAbs_ShapeEnum.TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            using var aCurve = new N.BRepAdaptor_Curve(N.TopoDS.Edge(anExp.Current()));
            if (aCurve.GetType_() != N.GeomAbs_CurveType.GeomAbs_Circle)
            {
                continue;
            }
            using var aP = aCurve.Value(aCurve.FirstParameter());
            double h = (new Point3D(aP.X(), aP.Y(), aP.Z()).XYZ() - anAx.Location().XYZ()).Dot(anAx.Direction().XYZ());
            if (h > aBest)
            {
                aBest = h;
                aTop = anExp.Current();
            }
        }
        anInputs.Add(aTop!);
        aCase.Steps.Add(new Step(Op.EdgeFace, 1, 0, theOrder));
        return aCase;
    }

    [Fact]
    public void MakeFilling_with_support_faces_as_build123d_MakeSurfacePatch_matches_native_OCCT()
    {
        var aWatch = Stopwatch.StartNew();
        var anOracle = new Oracle(output, 5942);
        foreach (int c in Tape.Budget(30))
        {
            Continuity anOrder = c % 3 == 2 ? Continuity.G2 : Continuity.G1;
            FillingCase aCase = BoxHole(anOracle, c, anOrder, false);
            if (c % 4 == 1)
            {
                // a point above the hole's centre
                aCase.Steps.Add(new Step(Op.Point, Point: aCase.Center));
            }
            aCase.Steps.Add(new Step(Op.Build));
            Run(aCase);
        }
        foreach (int c in Tape.Budget(20))
        {
            FillingCase aCase = CylinderCap(anOracle, c, c % 2 == 0 ? Continuity.G1 : Continuity.C0);
            aCase.Steps.Add(new Step(Op.Build));
            Run(aCase);
        }
        Report("MakeFilling with edges on support faces", aWatch.Elapsed);
        Assert.Empty(myDifferences);
    }

    [Fact]
    public void MakeFilling_with_mixed_and_point_constraints_matches_native_OCCT()
    {
        var aWatch = Stopwatch.StartNew();
        var anOracle = new Oracle(output, 5943);
        foreach (int c in Tape.Budget(20))
        {
            // a contour, with free (unbound) curve constraints and points, built with default parameters
            List<N.TopoDS_Shape> anEdges = Contour(anOracle, 2, out string aWhat);
            var aCase = new FillingCase { What = $"mixed {c} ({aWhat})", Inputs = anEdges };
            for (int i = 0; i < anEdges.Count; i++)
            {
                aCase.Steps.Add(new Step(Op.Edge, i, Order: Continuity.C0));
            }
            for (int k = 0; k < 1 + c % 4; k++)
            {
                aCase.Steps.Add(new Step(Op.Point, Point: InsidePoint(anOracle, anEdges)));
            }
            aCase.Steps.Add(new Step(Op.Build));
            Run(aCase);
        }
        foreach (int c in Tape.Budget(15))
        {
            // a box hole with a point given by U, V on a side face
            FillingCase aCase = BoxHole(anOracle, c, Continuity.G1, false);
            aCase.What = "box hole with a point on a face: " + aCase.What;
            aCase.Steps.Add(new Step(Op.PointOnFace, 0, Order: c % 2 == 0 ? Continuity.C0 : Continuity.G1,
                                     U: anOracle.Real(0.1, 0.9), V: anOracle.Real(0.1, 0.9)));
            aCase.Steps.Add(new Step(Op.Build));
            Run(aCase);
        }
        for (int c = 0; c < 10; c++)
        {
            // one side of the hole given by its face only (a free constraint)
            FillingCase aCase = BoxHole(anOracle, c, Continuity.G1, true);
            aCase.Steps.Add(new Step(Op.Build));
            Run(aCase);
        }
        for (int c = 0; c < 10; c++)
        {
            // the hole filled from the bottom face as initial surface
            FillingCase aCase = BoxHole(anOracle, c, c % 2 == 0 ? Continuity.C0 : Continuity.G1, false);
            aCase.What = "initial surface: " + aCase.What;
            aCase.Steps.Insert(0, new Step(Op.Init, 8));
            aCase.Steps.Add(new Step(Op.Build));
            Run(aCase);
        }
        Report("MakeFilling with mixed, point, free and initial-surface constraints", aWatch.Elapsed);
        Assert.Empty(myDifferences);
    }

    [Fact]
    public void MakeFilling_on_the_corpus_case_bug23380_matches_native_OCCT()
    {
        // bugs/filling/bug23380: "filling result 4 0 0 i f_1 f 1 d_3 d 1 b_2 0 b_3 0", which Draw reads as the initial
        // surface i (filling's a[5], a face) and the bounds f_1 on f (G1), d_3 on d (G1), b_2 and b_3 (C0). The bounds
        // b_2 and b_3 meet only geometrically, at a vertex that must move and grow (unit 9F).
        string aRoot = Metadata("OcctTestDataDir");
        string? aFile = string.IsNullOrEmpty(aRoot) || !Directory.Exists(aRoot) ? null
            : Directory.EnumerateFiles(aRoot, "bug23380.brep", SearchOption.AllDirectories).FirstOrDefault();
        if (aFile is null)
        {
            output.WriteLine("bug23380.brep isn't available");
            return;
        }
        var aWatch = Stopwatch.StartNew();
        var aShape = new N.TopoDS_Shape();
        using (var aBuilder = new N.BRep_Builder())
        {
            Assert.True(N.BRepTools.Read(aShape, aFile, aBuilder));
        }
        List<N.TopoDS_Shape> aB = NativeChildren(aShape).ToList(); // b_1 .. b_5
        var aD = new List<N.TopoDS_Shape>();
        for (var anExp = new N.TopExp_Explorer(aB[4], N.TopAbs_ShapeEnum.TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            if (!aD.Any(e => e.IsSame(anExp.Current()))) // explode skips the edges it has seen
            {
                aD.Add(anExp.Current());
            }
        }
        var aF = new List<N.TopoDS_Shape>();
        for (var anExp = new N.TopExp_Explorer(aB[0], N.TopAbs_ShapeEnum.TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            if (!aF.Any(e => e.IsSame(anExp.Current())))
            {
                aF.Add(anExp.Current());
            }
        }
        // Draw's variables, and filling's reading of its words (BRepTest_FillingCommands.cxx): for each bound, an edge
        // if the word names one, then a face if the next names one, then the order (Draw::Atoi, 0 for a name)
        var aVars = new Dictionary<string, N.TopoDS_Shape>
        {
            ["i"] = aB[3], ["f_1"] = aF[0], ["f"] = aB[0], ["d_3"] = aD[2], ["d"] = aB[4], ["b_2"] = aB[1], ["b_3"] = aB[2],
        };
        string[] aWords = ["i", "f_1", "f", "1", "d_3", "d", "1", "b_2", "0", "b_3", "0"];
        var aCase = new FillingCase { What = "bug23380", Inputs = [] };
        int Index(N.TopoDS_Shape theS)
        {
            aCase.Inputs.Add(theS);
            return aCase.Inputs.Count - 1;
        }
        N.TopoDS_Shape? Get(int theWord, N.TopAbs_ShapeEnum theType)
            => theWord < aWords.Length && aVars.TryGetValue(aWords[theWord], out N.TopoDS_Shape? aS) && aS.ShapeType() == theType ? aS : null;
        int w = 0;
        var aParsed = new List<string>();
        // filling's a[5]: the initial surface if it names a face, the bounds then starting at a[6]
        N.TopoDS_Shape? anInit = Get(0, N.TopAbs_ShapeEnum.TopAbs_FACE);
        if (anInit is not null)
        {
            aCase.Steps.Add(new Step(Op.Init, Index(anInit)));
            aParsed.Add(aCase.Steps[^1].ToString());
            w = 1;
        }
        for (int k = 1; k <= 4; k++)
        {
            N.TopoDS_Shape? anE = Get(w, N.TopAbs_ShapeEnum.TopAbs_EDGE);
            if (anE is not null)
            {
                w++;
            }
            N.TopoDS_Shape? aFace = Get(w, N.TopAbs_ShapeEnum.TopAbs_FACE);
            if (aFace is not null)
            {
                w++;
            }
            var anOrder = (Continuity)(int.TryParse(aWords[w++], out int o) ? o : 0);
            if (anE is not null && aFace is not null)
            {
                aCase.Steps.Add(new Step(Op.EdgeFace, Index(anE), Index(aFace), anOrder));
            }
            else if (anE is null)
            {
                Assert.NotNull(aFace);
                aCase.Steps.Add(new Step(Op.Face, Index(aFace), Order: anOrder));
            }
            else
            {
                aCase.Steps.Add(new Step(Op.Edge, Index(anE), Order: anOrder));
            }
            aParsed.Add(aCase.Steps[^1].ToString());
        }
        output.WriteLine("bug23380's bounds: " + string.Join("; ", aParsed));
        aCase.Steps.Add(new Step(Op.Build));
        Run(aCase);
        Report("MakeFilling on bug23380", aWatch.Elapsed);
        Assert.Empty(myDifferences);
    }
    // build123d.NET's FaceTests.MakeSurfacePatch: spline edges, each extruded into its support face. Natively the prism
    // writes the face's pcurve into the edge itself; the port's prism sweeps a copy of the caller's (frozen) edge, so the
    // filling finds the pcurve on the support's own edge (decision 1103; it threw NullReferenceException before). The
    // filling's edges are copies of the boundary edges with all their representations, so native's carry the pcurves
    // the prism wrote into the caller's edges: the results are compared by the face's own geometry (FaceRecord), the
    // errors and the exceptions.
    [Fact]
    public void MakeFilling_on_faces_extruded_from_the_constraint_edges_matches_native_OCCT()
    {
        var aWatch = Stopwatch.StartNew();
        Point3D[][] aSplines =
        [
            [new(0, 0, 0), new(1, 0, 0), new(10, 0, -10)],
            [new(0, 0, 0), new(0, 1, 0), new(0, 10, -10)],
            [new(10, 0, -10), new(7, 7, -10), new(0, 10, -10)],
        ];
        Vector3D[] aDirs = [new(0, -1, 0), new(-1, 0, 0), new(0, 0, -1)];
        // (edge, with its face or not, order) for each case
        (int Edge, bool OnFace, Continuity Order)[][] aCases =
        [
            [(0, true, Continuity.G1), (1, true, Continuity.G1), (2, true, Continuity.G1)],
            [(0, true, Continuity.G1), (1, true, Continuity.G1), (2, false, Continuity.C0)],
            [(0, true, Continuity.C1), (1, false, Continuity.C0), (2, false, Continuity.C0)],
            [(0, true, Continuity.G2), (1, true, Continuity.C0), (2, true, Continuity.G1)],
        ];
        for (int c = 0; c < aCases.Length; c++)
        {
            myCases++;
            var aNEdges = new List<N.TopoDS_Shape>();
            var aPEdges = new List<Shape>();
            var aNFaces = new List<N.TopoDS_Face>();
            var aPFaces = new List<Face>();
            for (int k = 0; k < 3; k++)
            {
                using var aNPnts = new N.TColgp_HArray1OfPnt(1, 3);
                var aPPnts = new NCollection_HArray1<Point3D>(1, 3);
                for (int i = 0; i < 3; i++)
                {
                    using N.gp_Pnt aNP = Oracle.ToNative(aSplines[k][i]);
                    aNPnts.SetValue(i + 1, aNP);
                    aPPnts.SetValue(i + 1, aSplines[k][i]);
                }
                using var aNI = new N.GeomAPI_Interpolate(aNPnts, false, 1.0e-6);
                aNI.Perform();
                var aPI = new GeomAPI_Interpolate(aPPnts, false, 1.0e-6);
                aPI.Perform();
                N.TopoDS_Edge aNE = new N.BRepBuilderAPI_MakeEdge(aNI.Curve()).Edge();
                Edge aPE = new BRepBuilderAPI_MakeEdge(aPI.Curve()).Edge();
                using N.gp_Vec aNV = new(aDirs[k].X(), aDirs[k].Y(), aDirs[k].Z());
                aNFaces.Add(N.TopoDS.Face(new N.BRepPrimAPI_MakePrism(aNE, aNV, false, true).Shape()));
                aPFaces.Add(ShapeCasts.Face(new BRepPrimAPI_MakePrism(aPE, aDirs[k]).Shape()));
                aNEdges.Add(aNE);
                aPEdges.Add(aPE);
            }
            Assert.True(aPEdges[0].TShape()!.IsFrozen);
            Assert.Null(Tool.CurveOnSurface(ShapeCasts.Edge(aPEdges[0]), aPFaces[0], out _, out _)); // the port's edge has none
            List<Shape> aPIn = [.. aPEdges, .. aPFaces.Select(f => (Shape)f)];
            List<string> anInputTexts = aPIn.Select(x => PortText(x)).ToList();
            void CheckInputs(string theWhat)
            {
                if (aPIn.Select(x => PortText(x)).SequenceEqual(anInputTexts))
                {
                    myUnchanged++;
                }
                else
                {
                    myDifferences.Add($"{theWhat}: an input changed");
                }
            }
            using var aN = new N.BRepOffsetAPI_MakeFilling();
            var aP = new BRepOffsetAPI_MakeFilling();
            int aNb = 0;
            foreach ((int anEdge, bool anOnFace, Continuity anOrder) in aCases[c])
            {
                string w = $"extruded supports {c}";
                if (anOnFace)
                {
                    Same(w + ": Add index", aN.Add(N.TopoDS.Edge(aNEdges[anEdge]), aNFaces[anEdge], (N.GeomAbs_Shape)(int)anOrder, true),
                         aP.Add(ShapeCasts.Edge(aPEdges[anEdge]), aPFaces[anEdge], anOrder, true));
                }
                else
                {
                    Same(w + ": Add index", aN.Add(N.TopoDS.Edge(aNEdges[anEdge]), (N.GeomAbs_Shape)(int)anOrder, true),
                         aP.Add(ShapeCasts.Edge(aPEdges[anEdge]), anOrder, true));
                }
                aNb++;
            }
            string w2 = $"extruded supports {c}";
            myBuilds++;
            string? aNErr = null, aPErr = null;
            N.TopoDS_Shape? aNRes = null;
            Shape aPRes = new();
            try
            {
                aN.Build();
                aNRes = aN.IsDone() ? aN.Shape() : null;
            }
            catch (N.OcctException e)
            {
                aNErr = ErrorOf(e);
            }
            try
            {
                aP.Build();
                aPRes = aP.IsDone() ? aP.Shape() : new Shape();
            }
            catch (Exception e)
            {
                aPErr = ErrorOf(e) ?? global::Kerf.Engine.Foundation.OcctTypeNames.Of(e.GetType());
            }
            if (aNErr is not null || aPErr is not null || aNRes is null || aPRes.IsNull)
            {
                if (aNErr == aPErr && (aNRes is null) == aPRes.IsNull)
                {
                    myIdentical++;
                    myExceptions += aNErr is null ? 0 : 1;
                    myNotDone += aNErr is null ? 1 : 0;
                }
                else
                {
                    myDifferences.Add($"{w2}: {aNErr ?? (aNRes is null ? "not done" : "done")} natively, {aPErr ?? (aPRes.IsNull ? "not done" : "done")} in the port");
                }
                CheckInputs(w2);
                continue;
            }
            string aNRec = FaceRecord(NativeText(aNRes)), aPRec = FaceRecord(PortText(aPRes));
            if (aNRec == aPRec)
            {
                myCanonical++;
            }
            else
            {
                myDifferences.Add($"{w2}: the faces differ:\n{aNRec}\n---\n{aPRec}");
            }
            SameValue(w2 + ": G0Error", aN.G0Error, aP.G0Error);
            SameValue(w2 + ": G1Error", aN.G1Error, aP.G1Error);
            SameValue(w2 + ": G2Error", aN.G2Error, aP.G2Error);
            GeomPlate_BuildPlateSurface aPlate = aP.InternalFilling().InternalBuilder()!;
            for (int i = 1; i <= aNb; i++)
            {
                bool aReadsUninitialised = aPlate.NbIntervalsChecked(i) < aPlate.NbPtsOnCur();
                SameIndexedValue(w2 + $": G0Error({i})", () => aN.G0Error(i), () => aP.G0Error(i), aReadsUninitialised);
                SameIndexedValue(w2 + $": G1Error({i})", () => aN.G1Error(i), () => aP.G1Error(i), aReadsUninitialised);
            }
            CheckInputs(w2);
        }
        Report("MakeFilling on faces extruded from the constraint edges", aWatch.Elapsed);
        Assert.Equal(4, myBuilds);
        Assert.Equal(3, myCanonical); // the fourth: the same exception on both sides
        Assert.Equal(4, myUnchanged);
        Assert.Empty(myDifferences);
    }

    // A face's own geometry, read from its BRep text: its surface and tolerance, and for each edge its orientation,
    // tolerance, 3D curve, pcurve on the face with its range, and vertices.
    private static string FaceRecord(string theText)
    {
        BrepTools.Read(out Shape aShape, new StringReader(theText));
        Face aFace = ShapeCasts.Face(aShape);
        var aW = new StringWriter(System.Globalization.CultureInfo.InvariantCulture) { NewLine = "\n" };
        SurfaceSet.PrintSurface(Tool.Surface(aFace)!, aW, true);
        aW.WriteLine($"face {aFace.Orientation()} {Tool.Tolerance(aFace):R}");
        for (var anExp = new Explorer(aFace, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Edge anE = ShapeCasts.Edge(anExp.Current());
            aW.WriteLine($"edge {anE.Orientation()} {Tool.Tolerance(anE):R}");
            Curve3D? aC = Tool.Curve(anE, out double f, out double l);
            if (aC is not null)
            {
                aW.WriteLine($"3d [{f:R}, {l:R}]");
                CurveSet.PrintCurve(aC, aW, true);
            }
            Curve2D? aPC = Tool.CurveOnSurface(anE, aFace, out double pf, out double pl);
            if (aPC is not null)
            {
                aW.WriteLine($"2d [{pf:R}, {pl:R}]");
                Curve2dSet.PrintCurve2d(aPC, aW, true);
            }
            Exploration.Vertices(anE, out Vertex aV1, out Vertex aV2);
            foreach (Vertex aV in new[] { aV1, aV2 })
            {
                Point3D aP = Tool.Pnt(aV);
                aW.WriteLine($"vertex {aP.X():R} {aP.Y():R} {aP.Z():R} {Tool.Tolerance(aV):R}");
            }
        }
        return aW.ToString();
    }
}
