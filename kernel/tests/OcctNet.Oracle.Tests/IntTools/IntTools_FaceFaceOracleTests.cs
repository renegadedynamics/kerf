#if HAS_ORACLE
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Text;
using Xunit.Abstractions;
using N = Build123d.Occt;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// IntTools_FaceFace's analytic lines against native GeomAPI_IntSS where their parameters agree (plan, unit 6D's row:
/// before 6P's BRepAlgoAPI_Section). Native's own IntTools_FaceFace isn't bound; it is compared bit for bit through
/// tools/ffrec's recordings (decision 6855, the ModelingAlgorithms tests).
/// </summary>
/// <remarks>
/// The faces are unbounded faces of tolerance 0 on random quadrics, so that FaceFace's criteria (the faces' tolerances
/// plus half the default fuzzy value each) are GeomAPI_IntSS's 1e-7 and its domains are the surfaces' natural
/// bounds. FaceFace orders the surfaces by type (IndexType), so native gets them in that order. Both run
/// IntPatch_ImpImpIntersection on the same bits; FaceFace then trims (and splits at 0) the lines its own way, so each
/// of its lines' basis curve (a line or a conic) is compared with GeomAPI_IntSS's lines' basis curves, bit for bit
/// (<see cref="IntersectionComparison.SameRecord"/>). Torus pairs and pairs FaceFace solves itself (two planes) or
/// tunes (cylinder and torus) aren't comparable and are skipped.
/// </remarks>
public class IntTools_FaceFaceOracleTests(ITestOutputHelper output) : IDisposable
{
    private readonly string myDirectory = Directory.CreateTempSubdirectory("occtnet-ff-oracle-").FullName;
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

    private string NewFile() => Path.Combine(myDirectory, $"f{++myFileNumber}.brep");

    private static N.gp_Ax3 NativeAx3(Oracle theO, double theRange)
    {
        gp_Dir aN = Oracle.Settled(theO.Dir());
        gp_Dir aX = Oracle.Settled(new gp_Dir(new gp_Vec(aN).Crossed(new gp_Vec(theO.Dir()))));
        var anAx = new gp_Ax3(new gp_Pnt(theO.Real(-theRange, theRange), theO.Real(-theRange, theRange),
                                         theO.Real(-theRange, theRange)), aN, aX);
        return Oracle.ToNative(anAx);
    }

    // plane, cylinder, cone, sphere: FaceFace's IndexType is the kind
    private static N.Geom_Surface NativeQuadric(Oracle theO, int theKind)
    {
        using N.gp_Ax3 anAx = NativeAx3(theO, 2);
        return theKind switch
        {
            0 => new N.Geom_Plane(anAx),
            1 => new N.Geom_CylindricalSurface(anAx, theO.Real(1, 4)),
            2 => new N.Geom_ConicalSurface(anAx, theO.Real(0.1, 1.2), theO.Real(1, 4)),
            _ => new N.Geom_SphericalSurface(anAx, theO.Real(1, 5)),
        };
    }

    private string FaceText(N.Geom_Surface theSurface)
    {
        using var aFace = new N.TopoDS_Face();
        using var aBuilder = new N.BRep_Builder();
        aBuilder.MakeFace(aFace, theSurface, 0.0);
        string aFile = NewFile();
        Assert.True(N.BRepTools.Write(aFace, aFile));
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

    private TopoDS_Face PortFace(string theText)
    {
        string aFile = NewFile();
        File.WriteAllText(aFile, theText, Encoding.Latin1);
        Assert.True(BRepTools.Read(out TopoDS_Shape aShape, aFile));
        return TopoDS.Face(aShape);
    }

    private static Geom_Curve PortBasis(Geom_Curve theC) => theC is Geom_TrimmedCurve aT ? aT.BasisCurve() : theC;

    private static N.Geom_Curve NativeBasis(N.Geom_Curve theC)
        => N.Geom_TrimmedCurve.DownCast(theC) is N.Geom_TrimmedCurve aT ? aT.BasisCurve()! : theC;

    [Fact]
    public void Analytic_lines_match_GeomAPI_IntSS()
    {
        Oracle anO = new(output, 6851);
        using IntersectionComparison aCmp = new();
        int aPairs = 0, aCompared = 0, aMatched = 0, aNativeLines = 0;
        List<string> aMisses = [];
        for (int aPair = 0; aPair < 300; aPair++)
        {
            int k1 = anO.Int(0, 3), k2 = anO.Int(0, 3);
            if (k1 == 0 && k2 == 0)
            {
                continue; // PerformPlanes, FaceFace's own
            }
            string aText1 = FaceText(NativeQuadric(anO, k1));
            string aText2 = FaceText(NativeQuadric(anO, k2));
            aPairs++;

            IntTools_FaceFace aFF = new();
            aFF.SetParameters(true, false, false, 1.0e-7);
            try
            {
                aFF.Perform(PortFace(aText1), PortFace(aText2));
            }
            catch (Standard_Failure)
            {
                continue;
            }
            if (!aFF.IsDone() || aFF.TangentFaces())
            {
                continue;
            }
            List<Geom_Curve> aPort = [];
            NCollection_Sequence<IntTools_Curve> aLines = aFF.Lines();
            for (int i = 1; i <= aLines.Length(); i++)
            {
                if (aLines.Value(i).Curve() is Geom_Curve aC
                    && PortBasis(aC) is Geom_Line or Geom_Conic)
                {
                    aPort.Add(PortBasis(aC));
                }
            }
            if (aPort.Count == 0)
            {
                continue;
            }

            // FaceFace's order: the surface of the higher IndexType first
            using N.Geom_Surface aS1 = NativeSurface(k1 < k2 ? aText2 : aText1);
            using N.Geom_Surface aS2 = NativeSurface(k1 < k2 ? aText1 : aText2);
            List<N.Geom_Curve> aNative = [];
            try
            {
                using var anInter = new N.GeomAPI_IntSS(aS1, aS2, 1.0e-7);
                if (anInter.IsDone())
                {
                    for (int i = 1; i <= anInter.NbLines(); i++)
                    {
                        aNative.Add(NativeBasis(anInter.Line(i)!));
                    }
                }
            }
            catch (Exception)
            {
            }
            aNativeLines += aNative.Count;
            foreach (Geom_Curve aC in aPort)
            {
                aCompared++;
                if (aNative.Any(n => aCmp.SameRecord(n, aC)))
                {
                    aMatched++;
                }
                else
                {
                    aMisses.Add(string.Create(CultureInfo.InvariantCulture,
                                              $"pair {aPair} (kinds {k1}, {k2}): a {aC.GetType().Name} of FaceFace's isn't among GeomAPI_IntSS's {aNative.Count} lines"));
                }
            }
        }
        output.WriteLine($"{aPairs} pairs; {aMatched}/{aCompared} FaceFace analytic lines bit-identical to a GeomAPI_IntSS line ({aNativeLines} native lines)");
        foreach (string aMiss in aMisses.Take(20))
        {
            output.WriteLine(aMiss);
        }
        Assert.True(aCompared > 100);
        Assert.Empty(aMisses);
    }
}
#endif
