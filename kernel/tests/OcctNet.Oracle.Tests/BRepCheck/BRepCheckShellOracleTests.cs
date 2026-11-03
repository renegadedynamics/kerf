// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Xunit.Abstractions;
using static Occt.BRepCheck_Status;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// BRepCheck_Shell against native OCCT. Only BRepCheck_Analyzer is bound natively, so the comparison is indirect:
/// on twin shells and solids built from the faces of twin boxes (closed, with a face missing, reversed or repeated,
/// two boxes in one shell, at random locations and orientations), native <c>BRepCheck_Analyzer(S, false).IsValid()</c>
/// must agree with the port's shell statuses: the first status of <c>Minimum</c> and, in a solid, of
/// <c>InContext(solid)</c>, which is what the analyzer reads for a shell. The faces, edges and vertices of the twin
/// boxes are valid, so the shell decides.
/// </summary>
public class BRepCheckShellOracleTests(ITestOutputHelper output)
{
    private const int Scenes = 240;

    private static List<Twin<N.TopoDS_Shape, TopoDS_Shape>> Faces(Twin<N.TopoDS_Solid, TopoDS_Solid> theBox)
    {
        var aFaces = new List<Twin<N.TopoDS_Shape, TopoDS_Shape>>();
        var aNExp = new N.TopExp_Explorer(theBox.N, N.TopAbs_ShapeEnum.TopAbs_FACE);
        var anExp = new TopExp_Explorer(theBox.P, TopAbs_FACE);
        for (; anExp.More(); anExp.Next(), aNExp.Next())
        {
            Assert.True(aNExp.More());
            aFaces.Add(new(aNExp.Current(), anExp.Current()));
        }

        return aFaces;
    }

    [Fact]
    public void Shells_are_checked_as_native_OCCT_checks_them()
    {
        var anOracle = new Oracle(output);
        int aValid = 0, aSolids = 0;
        var aStatuses = new Dictionary<BRepCheck_Status, int>();
        for (int aScene = 0; aScene < Scenes; aScene++)
        {
            var aTwins = new TwinShapes();
            List<Twin<N.TopoDS_Shape, TopoDS_Shape>> aFaces =
                Faces(aTwins.Box(anOracle.Real(0.5, 10), anOracle.Real(0.5, 10), anOracle.Real(0.5, 10)));
            switch (aScene % 6)
            {
                case 0:
                    break;
                case 1:
                    aFaces.RemoveAt(anOracle.Int(0, aFaces.Count - 1));
                    break;
                case 2:
                {
                    int i = anOracle.Int(0, aFaces.Count - 1);
                    aFaces[i] = new(aFaces[i].N.Reversed(), aFaces[i].P.Reversed());
                    break;
                }
                case 3:
                    aFaces.Add(aFaces[anOracle.Int(0, aFaces.Count - 1)]);
                    break;
                case 4:
                    aFaces.AddRange(Faces(aTwins.Box(anOracle.Real(0.5, 10), anOracle.Real(0.5, 10), anOracle.Real(0.5, 10))));
                    break;
                default:
                {
                    // two faces reversed, possibly the same one twice (then coherent again)
                    for (int k = 0; k < 2; k++)
                    {
                        int i = anOracle.Int(0, aFaces.Count - 1);
                        aFaces[i] = new(aFaces[i].N.Reversed(), aFaces[i].P.Reversed());
                    }

                    break;
                }
            }

            // the shell, and the solid holding it in two scenes out of three
            var aNB = new N.BRep_Builder();
            var aB = new BRep_Builder();
            var aNShell = new N.TopoDS_Shell();
            aNB.MakeShell(aNShell);
            TopoDS_Shell aShell = new();
            aB.MakeShell(ref aShell);
            foreach (Twin<N.TopoDS_Shape, TopoDS_Shape> aF in aFaces)
            {
                aNB.Add(aNShell, aF.N);
                aB.Add(aShell, aF.P);
            }

            bool isInSolid = aScene % 3 != 0;
            N.TopoDS_Shape aNShape = aNShell;
            TopoDS_Shape aShape = aShell;
            if (isInSolid)
            {
                var aNSolid = new N.TopoDS_Solid();
                aNB.MakeSolid(aNSolid);
                aNB.Add(aNSolid, aNShell);
                TopoDS_Solid aSolid = new();
                aB.MakeSolid(ref aSolid);
                aB.Add(aSolid, aShell);
                aNShape = aNSolid;
                aShape = aSolid;
                aSolids++;
            }

            if (anOracle.Bool())
            {
                aNShape = aNShape.Reversed();
                aShape = aShape.Reversed();
            }

            aShape.TShape()!.FreezeGraph();

            bool isNativeValid = new N.BRepCheck_Analyzer(aNShape, false).IsValid();

            TopoDS_Shell aPortShell = TopoDS.Shell(isInSolid ? new TopoDS_Iterator(aShape).Value() : aShape);
            var aCheck = new BRepCheck_Shell(aPortShell);
            BRepCheck_Status aStatus = aCheck.Status().First();
            if (aStatus == BRepCheck_NoError && isInSolid)
            {
                aCheck.InContext(aShape);
                aStatus = aCheck.StatusOnShape(aShape).First();
            }

            aStatuses[aStatus] = aStatuses.GetValueOrDefault(aStatus) + 1;
            bool isPortValid = aStatus == BRepCheck_NoError;
            anOracle.Same(isNativeValid ? 1 : 0, isPortValid ? 1 : 0, $"scene {aScene}: IsValid (port status {aStatus})");
            if (isPortValid)
            {
                aValid++;
            }
        }

        output.WriteLine($"{Scenes} scenes ({aSolids} solids), {aValid} valid; port statuses: "
                         + string.Join(", ", aStatuses.OrderBy(p => p.Key).Select(p => $"{p.Key} {p.Value}")));
        anOracle.Report("BRepCheck_Shell through BRepCheck_Analyzer.IsValid");
    }
}
