#if HAS_ORACLE
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Concurrent;
using System.Globalization;
using System.Reflection;
using System.Text;
using OcctNet.DrawLite;
using Xunit.Abstractions;
using N = Build123d.Occt;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// The surface/surface milestone (plan section 5, milestone 1; unit 5G): native GeomAPI_IntSS against the port's on
/// random and special surface pairs of every kind, on the surfaces of OCCT's lowalgos/intss corpus cases and on
/// pairs of faces of the data/occ shapes. Lines are compared with <see cref="IntersectionComparison"/> (decision 614):
/// the number of lines, each line's type and record bit for bit, the largest deviation where a record differs.
/// </summary>
/// <remarks>
/// Each surface is built natively, written as the BRep text of a face (BRep_Builder::MakeFace, so the surface is
/// stored as it is), and read back by native OCCT and by the port: both sides intersect the same bits. A pair's
/// outcome is the lines, "not done", or the exception's type. Native runs twice per pair, to set native
/// nondeterminism apart; the port runs serially and from parallel threads, which must agree bit for bit.
/// </remarks>
public class GeomAPI_IntSSOracleTests(ITestOutputHelper output) : IDisposable
{
    private readonly string myDirectory = Directory.CreateTempSubdirectory("occtnet-intss-oracle-").FullName;
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
        GC.SuppressFinalize(this);
    }

    private const double Tol = 1.0e-7;

    private static string Metadata(string theKey)
        => typeof(GeomAPI_IntSSOracleTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>()
                                                 .Single(a => a.Key == theKey).Value!;

    private static readonly string[] Kinds =
        ["plane", "cylinder", "cone", "sphere", "torus", "revolution", "extrusion", "Bezier", "B-spline", "offset", "trimmed"];

    // ---------------------------------------------------------------------------------------------------------
    // Shared surfaces

    /// <summary>A pair of surfaces with the same bits on both sides.</summary>
    private sealed record Pair(string What, string Kind1, string Kind2, string Text1, string Text2);

    private string NewFile() => Path.Combine(myDirectory, $"f{Interlocked.Increment(ref myFileNumber)}.brep");

    /// <summary>The BRep text of a face on the native surface, stored as it is.</summary>
    private string FaceText(N.Geom_Surface theSurface)
    {
        using var aFace = new N.TopoDS_Face();
        using var aBuilder = new N.BRep_Builder();
        aBuilder.MakeFace(aFace, theSurface, Tol);
        string aFile = NewFile();
        Assert.True(N.BRepTools.Write(aFace, aFile));
        return File.ReadAllText(aFile, Encoding.Latin1);
    }

    /// <summary>The BRep text of a face on the port's surface (corpus surfaces, made by Draw-lite).</summary>
    private string FaceText(Geom_Surface theSurface)
    {
        var aBuilder = new BRep_Builder();
        var aFace = new TopoDS_Face();
        aBuilder.MakeFace(ref aFace, theSurface, Tol);
        string aFile = NewFile();
        Assert.True(BRepTools.Write(aFace, aFile));
        return File.ReadAllText(aFile, Encoding.Latin1);
    }

    private N.Geom_Surface NativeSurface(string theText)
    {
        string aFile = NewFile();
        File.WriteAllText(aFile, theText, Encoding.Latin1);
        using var aShape = new N.TopoDS_Shape();
        using var aBuilder = new N.BRep_Builder();
        Assert.True(N.BRepTools.Read(aShape, aFile, aBuilder));
        return N.BRep_Tool.Surface(N.TopoDS.Face(aShape))!;
    }

    private Geom_Surface PortSurface(string theText)
    {
        string aFile = NewFile();
        File.WriteAllText(aFile, theText, Encoding.Latin1);
        Assert.True(BRepTools.Read(out TopoDS_Shape aShape, aFile));
        return BRep_Tool.Surface(TopoDS.Face(aShape))!;
    }

    // ---------------------------------------------------------------------------------------------------------
    // Outcomes

    /// <summary>The outcome of one run: "done", "not done" or the exception's type, and the lines.</summary>
    private sealed record Outcome<TCurve>(string Status, List<TCurve> Lines);

    private static Outcome<N.Geom_Curve> RunNative(N.Geom_Surface theS1, N.Geom_Surface theS2)
    {
        try
        {
            using var anInter = new N.GeomAPI_IntSS(theS1, theS2, Tol);
            if (!anInter.IsDone())
            {
                return new("not done", []);
            }
            List<N.Geom_Curve> aLines = [];
            for (int i = 1; i <= anInter.NbLines(); i++)
            {
                aLines.Add(anInter.Line(i)!);
            }
            return new("done", aLines);
        }
        catch (Exception)
        {
            return new("throws", []);
        }
    }

    private static Outcome<Geom_Curve> RunPort(Geom_Surface theS1, Geom_Surface theS2)
    {
        try
        {
            var anInter = new GeomAPI_IntSS(theS1, theS2, Tol);
            if (!anInter.IsDone())
            {
                return new("not done", []);
            }
            List<Geom_Curve> aLines = [];
            for (int i = 1; i <= anInter.NbLines(); i++)
            {
                aLines.Add(anInter.Line(i));
            }
            return new("done", aLines);
        }
        catch (Standard_Failure)
        {
            return new("throws", []);
        }
        catch (Exception anError)
        {
            // not an OCCT exception: a port bug
            return new("port " + anError.GetType().Name + ": " + anError.Message, []);
        }
    }

    /// <summary>The port's lines as text (the BRep text of an edge on each), for the determinism checks.</summary>
    private string PortText(Outcome<Geom_Curve> theOutcome)
    {
        var aText = new StringBuilder(theOutcome.Status).Append('\n');
        foreach (Geom_Curve aLine in theOutcome.Lines)
        {
            string aFile = NewFile();
            Assert.True(BRepTools.Write(new BRepBuilderAPI_MakeEdge(aLine).Edge(), aFile));
            aText.Append(File.ReadAllText(aFile, Encoding.Latin1));
        }
        return aText.ToString();
    }

    private string NativeText(Outcome<N.Geom_Curve> theOutcome)
    {
        var aText = new StringBuilder(theOutcome.Status).Append('\n');
        foreach (N.Geom_Curve aLine in theOutcome.Lines)
        {
            string aFile = NewFile();
            using var anEdge = new N.BRepBuilderAPI_MakeEdge(aLine);
            Assert.True(N.BRepTools.Write(anEdge.Edge(), aFile));
            aText.Append(File.ReadAllText(aFile, Encoding.Latin1));
        }
        return aText.ToString();
    }

    // ---------------------------------------------------------------------------------------------------------
    // Tallies

    private sealed class Tally(string theWhat)
    {
        public int Pairs, PairsWithLines, IdenticalPairs, SameCountAndTypes, NativeNondeterministic;
        public int Lines, IdenticalLines, Values, IdenticalValues;
        public double MaxDeviation;
        public string MaxDeviationWhat = "";
        public readonly List<string> Differences = [];
        public readonly ConcurrentDictionary<string, int> ByKinds = new(StringComparer.Ordinal);

        public void Report(ITestOutputHelper theOutput)
        {
            theOutput.WriteLine(string.Create(CultureInfo.InvariantCulture,
                $"{theWhat}: {Pairs} pairs ({PairsWithLines} with lines natively), {IdenticalPairs} identical "
                + $"({(Pairs == 0 ? 0 : 100.0 * IdenticalPairs / Pairs):F1}%), {SameCountAndTypes} more with the same lines but "
                + $"other values; lines {IdenticalLines}/{Lines} bit-identical, values {IdenticalValues}/{Values}; largest "
                + $"deviation {MaxDeviation:E2}{(MaxDeviation > 0 ? " (" + MaxDeviationWhat + ")" : "")}; native "
                + $"nondeterministic on {NativeNondeterministic}"));
            foreach (var aKinds in ByKinds.OrderBy(k => k.Key, StringComparer.Ordinal))
            {
                theOutput.WriteLine($"    differs: {aKinds.Key} {aKinds.Value}");
            }
            foreach (string aDiff in Differences)
            {
                theOutput.WriteLine("  " + aDiff);
            }
        }
    }

    /// <summary>Runs a pair on both sides and adds the comparison to the tally.</summary>
    private void Compare(Pair thePair, Tally theTally)
    {
        using N.Geom_Surface aN1 = NativeSurface(thePair.Text1);
        using N.Geom_Surface aN2 = NativeSurface(thePair.Text2);
        Geom_Surface aP1 = PortSurface(thePair.Text1), aP2 = PortSurface(thePair.Text2);

        Outcome<N.Geom_Curve> aNative = RunNative(aN1, aN2);
        Outcome<N.Geom_Curve> aNative2 = RunNative(aN1, aN2);
        bool aNativeStable = NativeText(aNative) == NativeText(aNative2);
        Outcome<Geom_Curve> aPort = RunPort(aP1, aP2);

        theTally.Pairs++;
        if (aNative.Lines.Count > 0)
        {
            theTally.PairsWithLines++;
        }
        if (!aNativeStable)
        {
            theTally.NativeNondeterministic++;
        }

        string? aDifference = null;
        if (aNative.Status != aPort.Status)
        {
            aDifference = $"native {aNative.Status}, port {aPort.Status}";
        }
        else
        {
            using var aCompare = new IntersectionComparison();
            try
            {
                aCompare.CompareLines(aNative.Lines, aPort.Lines, thePair.What);
            }
            catch (Exception anError) when (anError is Xunit.Sdk.XunitException)
            {
                aDifference = anError.Message.Split('\n')[0];
            }
            theTally.Lines += aCompare.Lines;
            theTally.IdenticalLines += aCompare.IdenticalLines;
            theTally.Values += aCompare.Values;
            theTally.IdenticalValues += aCompare.IdenticalValues;
            if (aDifference is null && aCompare.IdenticalLines < aCompare.Lines)
            {
                theTally.SameCountAndTypes++;
                aDifference = string.Create(CultureInfo.InvariantCulture,
                    $"same lines, {aCompare.Lines - aCompare.IdenticalLines} of {aCompare.Lines} differ by up to {aCompare.MaxDeviation:E2}");
                if (aCompare.MaxDeviation > theTally.MaxDeviation)
                {
                    theTally.MaxDeviation = aCompare.MaxDeviation;
                    theTally.MaxDeviationWhat = thePair.What;
                }
            }
        }
        foreach (N.Geom_Curve aC in aNative.Lines.Concat(aNative2.Lines))
        {
            aC.Dispose();
        }

        if (aDifference is null)
        {
            theTally.IdenticalPairs++;
            return;
        }
        string aKinds = string.CompareOrdinal(thePair.Kind1, thePair.Kind2) <= 0 ? $"{thePair.Kind1}/{thePair.Kind2}" : $"{thePair.Kind2}/{thePair.Kind1}";
        theTally.ByKinds.AddOrUpdate(aKinds, 1, (_, n) => n + 1);
        theTally.Differences.Add($"{thePair.What}: {aDifference}{(aNativeStable ? "" : " [native nondeterministic]")}");
    }

    // ---------------------------------------------------------------------------------------------------------
    // Inputs

    private static readonly gp_Pnt s_centre = new(8, 8, 0);

    private static N.gp_Ax3 NativeAx3(Oracle theO, in gp_Pnt theCentre, double theRange)
    {
        gp_Dir aN = Oracle.Settled(theO.Dir());
        gp_Dir aX = Oracle.Settled(new gp_Dir(new gp_Vec(aN).Crossed(new gp_Vec(theO.Dir()))));
        var anAx = new gp_Ax3(new gp_Pnt(theCentre.X() + theO.Real(-theRange, theRange), theCentre.Y() + theO.Real(-theRange, theRange),
                                         theCentre.Z() + theO.Real(-theRange, theRange)), aN, aX);
        return Oracle.ToNative(anAx);
    }

    private static N.Geom_BSplineCurve NativeProfile(Oracle theO, in gp_Pnt theCentre, double theRange)
    {
        int aDeg = theO.Int(2, 3), aNbPoles = theO.Int(aDeg + 1, 6);
        var aKnots = new NCollection_Array1<double>(1, aNbPoles - aDeg + 1);
        var aMults = new NCollection_Array1<int>(1, aNbPoles - aDeg + 1);
        for (int i = aKnots.Lower(); i <= aKnots.Upper(); i++)
        {
            aKnots[i] = i - 1;
            aMults[i] = i == aKnots.Lower() || i == aKnots.Upper() ? aDeg + 1 : 1;
        }
        var aPoles = new NCollection_Array1<gp_Pnt>(1, aNbPoles);
        for (int i = 1; i <= aNbPoles; i++)
        {
            aPoles[i] = new gp_Pnt(theCentre.X() + theO.Real(-theRange, theRange), theCentre.Y() + theO.Real(-theRange, theRange),
                                   theCentre.Z() + theO.Real(-theRange / 2, theRange / 2));
        }
        return Oracle.ToNativeCurve(aPoles, null, aKnots, aMults, aDeg, false);
    }

    /// <summary>A native surface of the given kind (<see cref="Kinds"/>) about the free-form sheets' region.</summary>
    private static N.Geom_Surface NativeSurfaceOfKind(Oracle theO, int theKind)
    {
        switch (theKind)
        {
            case 0:
            {
                using N.gp_Ax3 anAx = NativeAx3(theO, s_centre, 3);
                return new N.Geom_Plane(anAx);
            }
            case 1:
            {
                using N.gp_Ax3 anAx = NativeAx3(theO, s_centre, 3);
                return new N.Geom_CylindricalSurface(anAx, theO.Real(1, 5));
            }
            case 2:
            {
                using N.gp_Ax3 anAx = NativeAx3(theO, s_centre, 3);
                return new N.Geom_ConicalSurface(anAx, theO.Real(0.1, 1.2), theO.Int(0, 3) == 0 ? 0.0 : theO.Real(1, 5));
            }
            case 3:
            {
                using N.gp_Ax3 anAx = NativeAx3(theO, s_centre, 3);
                return new N.Geom_SphericalSurface(anAx, theO.Real(2, 6));
            }
            case 4:
            {
                using N.gp_Ax3 anAx = NativeAx3(theO, s_centre, 3);
                double r = theO.Real(0.5, 2);
                return new N.Geom_ToroidalSurface(anAx, theO.Real(r + 1, 6), r);
            }
            case 5:
            {
                using N.Geom_BSplineCurve aProfile = NativeProfile(theO, s_centre, 4);
                using N.gp_Ax3 anAx = NativeAx3(theO, s_centre, 2);
                using N.gp_Ax1 anAxis = anAx.Axis();
                return new N.Geom_SurfaceOfRevolution(aProfile, anAxis);
            }
            case 6:
            {
                using N.Geom_BSplineCurve aProfile = NativeProfile(theO, s_centre, 5);
                using N.gp_Dir aDir = Oracle.ToNative(Oracle.Settled(theO.Dir()));
                return new N.Geom_SurfaceOfLinearExtrusion(aProfile, aDir);
            }
            case 7:
            {
                (N.Geom_BezierSurface aN, _) = AdaptorOracleInputs.RandomBezierSurface(theO);
                return aN;
            }
            case 8:
            {
                (N.Geom_BSplineSurface aN, _) = AdaptorOracleInputs.RandomBSplineSurface(theO, 0, true, 1);
                return aN;
            }
            case 9:
            {
                using N.Geom_Surface aBasis = theO.Int(0, 2) == 0
                    ? NativeSurfaceOfKind(theO, theO.Int(0, 3))
                    : AdaptorOracleInputs.RandomBSplineSurface(theO, 2, false, 3).Native;
                return new N.Geom_OffsetSurface(aBasis, theO.Real(0.1, 1.0) * (theO.Bool() ? 1 : -1), true);
            }
            default:
            {
                using N.Geom_Surface aBasis = NativeSurfaceOfKind(theO, theO.Int(0, 8));
                double aU1 = 0, aU2 = 0, aV1 = 0, aV2 = 0;
                aBasis.Bounds(ref aU1, ref aU2, ref aV1, ref aV2);
                (double, double) Range(double theF, double theL, bool thePeriodic)
                {
                    if (Precision.IsInfinite(theF) || Precision.IsInfinite(theL))
                    {
                        double aF = theO.Real(-6, 0);
                        return (aF, aF + theO.Real(2, 8));
                    }
                    double aD = theL - theF;
                    if (thePeriodic)
                    {
                        double aF = theO.Real(theF, theL);
                        return (aF, aF + theO.Real(0.3, 0.9) * aD);
                    }
                    return (theO.Real(theF, theF + 0.3 * aD), theO.Real(theF + 0.7 * aD, theL));
                }
                (double aUF, double aUL) = Range(aU1, aU2, aBasis.IsUPeriodic());
                (double aVF, double aVL) = Range(aV1, aV2, aBasis.IsVPeriodic());
                return new N.Geom_RectangularTrimmedSurface(aBasis, aUF, aUL, aVF, aVL);
            }
        }
    }

    private Pair MakePair(string theWhat, int theKind1, int theKind2, N.Geom_Surface theS1, N.Geom_Surface theS2)
    {
        using (theS1)
        using (theS2)
        {
            return new Pair(theWhat, Kinds[theKind1], Kinds[theKind2], FaceText(theS1), FaceText(theS2));
        }
    }

    /// <summary>
    /// Analytic pairs at random and special placements (ImpImp's cases): random; coaxial; parallel axes; sharing a
    /// location (a cone's apex, a sphere's centre); tangent (the radii reach the other's axis).
    /// </summary>
    private List<Pair> AnalyticPairs(int theSeed, int theRepeats)
    {
        var anO = new Oracle(output, theSeed);
        List<Pair> aPairs = [];
        string[] aCases = ["random", "coaxial", "parallel", "same location", "tangent"];
        for (int r = 0; r < theRepeats; r++)
        {
            for (int i = 0; i < 125; i++)
            {
                int aK1 = i % 5, aK2 = (i / 5) % 5, aCase = i / 25;
                gp_Dir aN1 = Oracle.Settled(anO.Dir()), aN2 = Oracle.Settled(anO.Dir());
                gp_Dir aX1 = Oracle.Settled(new gp_Dir(new gp_Vec(aN1).Crossed(new gp_Vec(anO.Dir()))));
                gp_Dir aX2 = Oracle.Settled(new gp_Dir(new gp_Vec(aN2).Crossed(new gp_Vec(anO.Dir()))));
                var anAx1 = new gp_Ax3(new gp_Pnt(anO.Real(-1, 1), anO.Real(-1, 1), anO.Real(-1, 1)), aN1, aX1);
                var anAx2 = new gp_Ax3(new gp_Pnt(anO.Real(-1, 1), anO.Real(-1, 1), anO.Real(-1, 1)), aN2, aX2);
                double aR1 = Math.Round(anO.Real(0.5, 3), 3), aR2 = Math.Round(anO.Real(0.5, 3), 3);
                double anA1 = anO.Real(0.1, 1.2), anA2 = anO.Real(0.1, 1.2);
                switch (aCase)
                {
                    case 1:
                        anAx2 = new gp_Ax3(anAx1.Location().Translated(new gp_Vec(anAx1.Direction()) * anO.Real(-2, 2)),
                                           anAx1.Direction(), anAx1.XDirection());
                        break;
                    case 2:
                        anAx2 = new gp_Ax3(anAx1.Location().Translated(new gp_Vec(anAx1.XDirection()) * anO.Real(-2, 2)),
                                           anAx1.Direction(), anAx1.XDirection());
                        break;
                    case 3:
                        anAx2 = aK1 == 4 || aK2 == 4 ? anAx1 : new gp_Ax3(anAx1.Location(), anAx2.Direction(), anAx2.XDirection());
                        aR1 = aK1 == 2 ? 0.0 : aR1;
                        aR2 = aK2 == 2 ? 0.0 : aR2;
                        break;
                    case 4:
                        anAx2 = new gp_Ax3(anAx1.Location().Translated(new gp_Vec(anAx1.XDirection()) * (aR1 + aR2)),
                                           anAx1.Direction(), anAx1.XDirection());
                        break;
                }
                N.Geom_Surface Make(int theKind, in gp_Ax3 theAx, double theR, double theAng)
                {
                    using N.gp_Ax3 anAx = Oracle.ToNative(theAx);
                    return theKind switch
                    {
                        0 => new N.Geom_Plane(anAx),
                        1 => new N.Geom_CylindricalSurface(anAx, theR),
                        2 => new N.Geom_ConicalSurface(anAx, theAng, theR),
                        3 => new N.Geom_SphericalSurface(anAx, theR),
                        _ => new N.Geom_ToroidalSurface(anAx, theR + 1.0, 0.5 * theR),
                    };
                }
                aPairs.Add(MakePair($"analytic {r}.{i} ({Kinds[aK1]}/{Kinds[aK2]}, {aCases[aCase]})", aK1, aK2,
                                    Make(aK1, anAx1, aR1, anA1), Make(aK2, anAx2, aR2, anA2)));
            }
        }
        return aPairs;
    }

    /// <summary>Pairs of every kind against every kind where at least one is not analytic.</summary>
    private List<Pair> FreeFormPairs(int theSeed, int theRepeats)
    {
        var anO = new Oracle(output, theSeed);
        List<Pair> aPairs = [];
        for (int r = 0; r < theRepeats; r++)
        {
            for (int aK1 = 0; aK1 < Kinds.Length; aK1++)
            {
                for (int aK2 = 0; aK2 < Kinds.Length; aK2++)
                {
                    if (aK1 < 5 && aK2 < 5)
                    {
                        continue;
                    }
                    aPairs.Add(MakePair($"free-form {r}.{aK1}.{aK2} ({Kinds[aK1]}/{Kinds[aK2]})", aK1, aK2,
                                        NativeSurfaceOfKind(anO, aK1), NativeSurfaceOfKind(anO, aK2)));
                }
            }
        }
        return aPairs;
    }

    private static string KindOf(Geom_Surface theS) => theS switch
    {
        Geom_Plane => Kinds[0],
        Geom_CylindricalSurface => Kinds[1],
        Geom_ConicalSurface => Kinds[2],
        Geom_SphericalSurface => Kinds[3],
        Geom_ToroidalSurface => Kinds[4],
        Geom_SurfaceOfRevolution => Kinds[5],
        Geom_SurfaceOfLinearExtrusion => Kinds[6],
        Geom_BezierSurface => Kinds[7],
        Geom_BSplineSurface => Kinds[8],
        Geom_OffsetSurface => Kinds[9],
        _ => Kinds[10],
    };

    // ---------------------------------------------------------------------------------------------------------
    // Tests

    [Fact]
    public void Analytic_pairs_intersect_as_native_OCCT()
    {
        List<Pair> aPairs = AnalyticPairs(57001, 4);
        var aTally = new Tally("analytic pairs (plane, cylinder, cone, sphere, torus; random and special placements)");
        foreach (Pair aPair in aPairs)
        {
            Compare(aPair, aTally);
        }
        aTally.Report(output);
        Assert.True(aTally.PairsWithLines > 300);
        Assert.True(aTally.IdenticalPairs >= aTally.Pairs * 0.95, $"{aTally.IdenticalPairs} of {aTally.Pairs} identical");
    }

    [Fact]
    public void Free_form_pairs_intersect_as_native_OCCT()
    {
        List<Pair> aPairs = FreeFormPairs(57002, 4);
        var aTally = new Tally("pairs with a revolution, extrusion, Bezier, B-spline, offset or trimmed surface");
        foreach (Pair aPair in aPairs)
        {
            Compare(aPair, aTally);
        }
        aTally.Report(output);
        Assert.True(aTally.PairsWithLines > 50);
        Assert.True(aTally.IdenticalPairs >= aTally.Pairs * 0.95, $"{aTally.IdenticalPairs} of {aTally.Pairs} identical");
    }

    /// <summary>
    /// The surfaces OCCT's lowalgos/intss cases intersect (their general form, <c>intersect r s1 s2 [tol]</c>, which
    /// is GeomAPI_IntSS's computation at the case's tolerance): each runnable case runs in Draw-lite with
    /// <c>intersect</c> wrapped to record its surfaces.
    /// </summary>
    [Fact]
    public void The_corpus_intss_surfaces_intersect_as_native_OCCT()
    {
        string aTests = Path.Combine(Metadata("OcctSourceDir"), "tests");
        string aResources = Path.Combine(Metadata("OcctSourceDir"), "resources", "DrawResources");
        string aData = string.Join(';', Path.Combine(Metadata("OcctSourceDir"), "data"), Metadata("OcctTestDataDir")).Replace('\\', '/');
        var aCorpus = new DrawCorpus(aTests);
        var aScanner = new DrawInterpreter();
        aScanner.LoadDrawResources(aResources);
        List<DrawTestCase> aCases = aCorpus.Cases().Where(c => c.Group == "lowalgos" && c.Grid == "intss")
                                           .Where(c => aCorpus.Scan(c, aScanner).IsRunnable).ToList();
        var aRecorded = new ConcurrentBag<(string Case, int Call, Geom_Surface S1, Geom_Surface S2)>();
        int anOtherForms = 0;
        foreach (DrawTestCase aCase in aCases)
        {
            int aCall = 0;
            aCorpus.RunAndCheck(aCase, aResources, aData, TimeSpan.FromSeconds(120), theDI =>
            {
                theDI.Eval("rename intersect __intersect_5g");
                theDI.Register("intersect", (i, a) =>
                {
                    if (a.Count is 4 or 5
                        && (i.GetDrawable(a[2]) as DrawGeometry)?.Geometry is Geom_Surface aS1
                        && (i.GetDrawable(a[3]) as DrawGeometry)?.Geometry is Geom_Surface aS2
                        && (a.Count == 4 || i.Atof(a[4]) == Tol))
                    {
                        aRecorded.Add((aCase.Id, ++aCall, aS1, aS2));
                    }
                    else
                    {
                        Interlocked.Increment(ref anOtherForms);
                    }
                    return i.Eval("__intersect_5g " + TclList.Format(a.Skip(1)));
                });
            });
        }

        var aTally = new Tally($"lowalgos/intss: {aCases.Count} runnable cases");
        foreach (var aRec in aRecorded.OrderBy(r => r.Case, StringComparer.Ordinal).ThenBy(r => r.Call))
        {
            var aPair = new Pair($"{aRec.Case} call {aRec.Call}", KindOf(aRec.S1), KindOf(aRec.S2), FaceText(aRec.S1), FaceText(aRec.S2));
            Compare(aPair, aTally);
        }
        aTally.Report(output);
        output.WriteLine($"{anOtherForms} calls with bounds, a start point, another tolerance or a curve (not GeomAPI_IntSS's)");
        Assert.True(aTally.Pairs > 20);
    }

    /// <summary>Pairs of faces' surfaces of each data/occ shape (random pairs, so neighbours and strangers).</summary>
    [Fact]
    public void The_surfaces_of_data_occ_faces_intersect_as_native_OCCT()
    {
        var anO = new Oracle(output, 57004);
        var aTally = new Tally("data/occ faces' surfaces");
        foreach (string aFile in Directory.GetFiles(Path.Combine(Metadata("OcctSourceDir"), "data", "occ"), "*.brep").Order(StringComparer.Ordinal))
        {
            using var aShape = new N.TopoDS_Shape();
            using var aBuilder = new N.BRep_Builder();
            if (!N.BRepTools.Read(aShape, aFile, aBuilder))
            {
                continue;
            }
            List<string> aTexts = [];
            List<string> aKinds = [];
            using (var anExp = new N.TopExp_Explorer(aShape, N.TopAbs_ShapeEnum.TopAbs_FACE, N.TopAbs_ShapeEnum.TopAbs_SHAPE))
            {
                for (; anExp.More() && aTexts.Count < 200; anExp.Next())
                {
                    using var aCurrent = anExp.Current();
                    using var aLoc = new N.TopLoc_Location();
                    using N.Geom_Surface? aS = N.BRep_Tool.Surface(N.TopoDS.Face(aCurrent), aLoc);
                    if (aS is null || !aLoc.IsIdentity())
                    {
                        continue;
                    }
                    string aText = FaceText(aS);
                    aTexts.Add(aText);
                    aKinds.Add(KindOf(PortSurface(aText)));
                }
            }
            for (int k = 0; k < Math.Min(6, aTexts.Count / 2); k++)
            {
                int i = anO.Int(0, aTexts.Count - 1), j = anO.Int(0, aTexts.Count - 1);
                if (i == j)
                {
                    continue;
                }
                Compare(new Pair($"{Path.GetFileName(aFile)} faces {i + 1}/{j + 1}", aKinds[i], aKinds[j], aTexts[i], aTexts[j]), aTally);
            }
        }
        aTally.Report(output);
        Assert.True(aTally.Pairs > 50);
    }

    /// <summary>The port's lines are the same serially, from parallel threads, and on every repeat.</summary>
    [Fact]
    public void The_ports_lines_are_the_same_serially_in_parallel_and_on_repeats()
    {
        List<Pair> aPairs = [.. AnalyticPairs(57005, 1).Where((_, i) => i % 3 == 0), .. FreeFormPairs(57006, 1)];
        string Run(Pair thePair) => PortText(RunPort(PortSurface(thePair.Text1), PortSurface(thePair.Text2)));
        string[] aSerial = [.. aPairs.Select(Run)];
        string[] aParallel = new string[aPairs.Count];
        Parallel.For(0, aPairs.Count, new ParallelOptions { MaxDegreeOfParallelism = 8 }, i => aParallel[i] = Run(aPairs[i]));
        string[] aRepeat = [.. aPairs.Select(Run)];
        int aLines = aSerial.Sum(t => t.Split("\nCurves ").Length - 1);
        output.WriteLine($"{aPairs.Count} pairs, {aLines} line texts: serial = parallel = repeat");
        for (int i = 0; i < aPairs.Count; i++)
        {
            Assert.True(aSerial[i] == aParallel[i], $"{aPairs[i].What}: parallel differs");
            Assert.True(aSerial[i] == aRepeat[i], $"{aPairs[i].What}: repeat differs");
        }
    }
}
#endif
