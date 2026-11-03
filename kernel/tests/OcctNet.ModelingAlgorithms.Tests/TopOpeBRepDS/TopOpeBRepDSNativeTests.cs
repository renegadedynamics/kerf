// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy.Data;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;
using static Kerf.Engine.Booleans.Legacy.Data.DsKind;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// The data structures native OCCT's fillets and chamfers fill (tools/dsrec, <see cref="DsRecordings"/>): each is read
/// into the port's data structure and written back, and the text must be native's byte for byte. That checks the
/// port's data structure, its explorers and iterators, <see cref="TextDump"/> and the geometry's text
/// against native's on every kind of data the fillets write.
/// </summary>
public class TopOpeBRepDSNativeTests
{
    public static TheoryData<string> CaseNames()
    {
        TheoryData<string> aData = new();
        foreach (DsCase aCase in DsRecordings.Cases.Values)
        {
            if (aCase.PerformCalls > 0)
            {
                aData.Add(aCase.Name);
            }
        }
        return aData;
    }

    [Fact]
    public void TheRecordingHasTheCases()
    {
        // 15 cases of unit 7L, 23 of unit 7N2 (three of them failures that never reach Perform)
        Assert.Equal(38, DsRecordings.Cases.Count);
        foreach (DsCase aCase in DsRecordings.Cases.Values)
        {
            if (aCase.PerformCalls == 0)
            {
                Assert.False(aCase.Done, aCase.Name);
                Assert.StartsWith("b3d_fail_", aCase.Name);
                continue;
            }
            Assert.True(aCase.Done, aCase.Name);
            Assert.Equal(1, aCase.PerformCalls);
            // in these cases Perform leaves what the dump shows unchanged (decision 1064)
            Assert.True(aCase.UnchangedByPerform, aCase.Name);
        }
    }

    [Theory]
    [MemberData(nameof(CaseNames))]
    public void ReadAndWrittenBackIsNatives(string theName)
    {
        // Reading geometry from text normalises directions and brings a periodic curve's trim into its period, as
        // OCCT's GeomTools does: native's text and the port's differ there by an ulp or a period, nowhere else
        DsCase aCase = DsRecordings.Cases[theName];
        Shape anInput = aCase.Input;
        DsHDataStructure aHDS = TextDump.Read(aCase.DsText, anInput);
        string aText = TextDump.Write(aHDS.DS(), anInput);
        Assert.Empty(DsRecordings.Differences(aCase.DsText, aText, 1.0e-14));

        // and once read, the text is a fixed point: byte for byte
        DsHDataStructure aHDS2 = TextDump.Read(aText, anInput);
        Assert.Equal(aText, TextDump.Write(aHDS2.DS(), anInput));
    }

    [Theory]
    [MemberData(nameof(CaseNames))]
    public void TheFilletsDataHoldsTogether(string theName)
    {
        // what ChFi3d leaves for Perform: the solid's surfaces, the faces' curves, the curves' points, all in range
        DsCase aCase = DsRecordings.Cases[theName];
        Shape anInput = aCase.Input;
        DsHDataStructure aHDS = TextDump.Read(aCase.DsText, anInput);
        DsDataStructure aDS = aHDS.DS();

        int aNbSolidSurfaces = 0;
        for (int i = 1; i <= aDS.NbShapes(); i++)
        {
            Shape S = aDS.Shape(i);
            Assert.False(S.IsNull);
            Assert.Equal(i, aDS.Shape(S));
            if (S.ShapeType() == ShapeType.Solid || S.ShapeType() == ShapeType.Shell) // a fillet on a shell: its SSIs (unit 7N2)
            {
                for (DsInterferenceIterator it = new(aDS.ShapeInterferences(S)); it.More(); it.Next())
                {
                    Assert.IsType<DsSolidSurfaceInterference>(it.Value());
                    Assert.Equal(DsKind.Surface, it.Value().GeometryType());
                    Assert.InRange(it.Value().Geometry(), 1, aDS.NbSurfaces());
                    aNbSolidSurfaces++;
                }
            }
            if (S.ShapeType() == ShapeType.Face)
            {
                for (DsCurveIterator it = aHDS.FaceCurves(S); it.More(); it.Next())
                {
                    Assert.InRange(it.Current(), 1, aDS.NbCurves());
                    Assert.NotNull(it.PCurve());
                }
            }
        }
        Assert.InRange(aNbSolidSurfaces, 1, aDS.NbSurfaces());

        // ChFi3d_Builder's constructor-time pass sets every curve's range from its points (CompleteDS)
        for (DsCurveExplorer cex = new(aDS); cex.More(); cex.Next())
        {
            Assert.True(cex.Curve().Range(out double f, out double l));
            double aMin = RealLast(), aMax = RealFirst();
            aHDS.MinMaxOnParameter(aDS.CurveInterferences(cex.Index()), ref aMin, ref aMax);
            if (!aDS.CurveInterferences(cex.Index()).IsEmpty())
            {
                Assert.Equal(aMin, f);
                Assert.Equal(aMax, l);
            }
            for (DsPointIterator it = aHDS.CurvePoints(cex.Index()); it.More(); it.Next())
            {
                Assert.True(it.IsPoint || it.IsVertex);
            }
        }

        // the fillets' surfaces are never kept: the default surface explorer finds none (OCCT's default)
        Assert.Equal(0, new DsSurfaceExplorer(aDS).NbSurface());
        Assert.Equal(aDS.NbSurfaces(), new DsSurfaceExplorer(aDS, false).NbSurface());
    }
}
