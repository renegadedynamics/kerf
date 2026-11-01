// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_ShapeEnum;
using static Occt.TopOpeBRepDS_Kind;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// The data structures native OCCT's fillets and chamfers fill (tools/dsrec, <see cref="DsRecordings"/>): each is read
/// into the port's data structure and written back, and the text must be native's byte for byte. That checks the
/// port's data structure, its explorers and iterators, <see cref="TopOpeBRepDS_TextDump"/> and the geometry's text
/// against native's on every kind of data the fillets write.
/// </summary>
public class TopOpeBRepDSNativeTests
{
    public static TheoryData<string> CaseNames()
    {
        TheoryData<string> aData = new();
        foreach (string aName in DsRecordings.Cases.Keys)
        {
            aData.Add(aName);
        }
        return aData;
    }

    [Fact]
    public void TheRecordingHasTheCases()
    {
        Assert.Equal(15, DsRecordings.Cases.Count);
        foreach (DsCase aCase in DsRecordings.Cases.Values)
        {
            Assert.True(aCase.Done, aCase.Name);
            Assert.Equal(1, aCase.PerformCalls);
            // in these cases Perform leaves what the dump shows unchanged (decision 9903)
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
        TopoDS_Shape anInput = aCase.Input;
        TopOpeBRepDS_HDataStructure aHDS = TopOpeBRepDS_TextDump.Read(aCase.DsText, anInput);
        string aText = TopOpeBRepDS_TextDump.Write(aHDS.DS(), anInput);
        Assert.Empty(DsRecordings.Differences(aCase.DsText, aText, 1.0e-14));

        // and once read, the text is a fixed point: byte for byte
        TopOpeBRepDS_HDataStructure aHDS2 = TopOpeBRepDS_TextDump.Read(aText, anInput);
        Assert.Equal(aText, TopOpeBRepDS_TextDump.Write(aHDS2.DS(), anInput));
    }

    [Theory]
    [MemberData(nameof(CaseNames))]
    public void TheFilletsDataHoldsTogether(string theName)
    {
        // what ChFi3d leaves for Perform: the solid's surfaces, the faces' curves, the curves' points, all in range
        DsCase aCase = DsRecordings.Cases[theName];
        TopoDS_Shape anInput = aCase.Input;
        TopOpeBRepDS_HDataStructure aHDS = TopOpeBRepDS_TextDump.Read(aCase.DsText, anInput);
        TopOpeBRepDS_DataStructure aDS = aHDS.DS();

        int aNbSolidSurfaces = 0;
        for (int i = 1; i <= aDS.NbShapes(); i++)
        {
            TopoDS_Shape S = aDS.Shape(i);
            Assert.False(S.IsNull());
            Assert.Equal(i, aDS.Shape(S));
            if (S.ShapeType() == TopAbs_SOLID)
            {
                for (TopOpeBRepDS_InterferenceIterator it = new(aDS.ShapeInterferences(S)); it.More(); it.Next())
                {
                    Assert.IsType<TopOpeBRepDS_SolidSurfaceInterference>(it.Value());
                    Assert.Equal(TopOpeBRepDS_SURFACE, it.Value().GeometryType());
                    Assert.InRange(it.Value().Geometry(), 1, aDS.NbSurfaces());
                    aNbSolidSurfaces++;
                }
            }
            if (S.ShapeType() == TopAbs_FACE)
            {
                for (TopOpeBRepDS_CurveIterator it = aHDS.FaceCurves(S); it.More(); it.Next())
                {
                    Assert.InRange(it.Current(), 1, aDS.NbCurves());
                    Assert.NotNull(it.PCurve());
                }
            }
        }
        Assert.InRange(aNbSolidSurfaces, 1, aDS.NbSurfaces());

        // ChFi3d_Builder's constructor-time pass sets every curve's range from its points (CompleteDS)
        for (TopOpeBRepDS_CurveExplorer cex = new(aDS); cex.More(); cex.Next())
        {
            Assert.True(cex.Curve().Range(out double f, out double l));
            double aMin = RealLast(), aMax = RealFirst();
            aHDS.MinMaxOnParameter(aDS.CurveInterferences(cex.Index()), ref aMin, ref aMax);
            if (!aDS.CurveInterferences(cex.Index()).IsEmpty())
            {
                Assert.Equal(aMin, f);
                Assert.Equal(aMax, l);
            }
            for (TopOpeBRepDS_PointIterator it = aHDS.CurvePoints(cex.Index()); it.More(); it.Next())
            {
                Assert.True(it.IsPoint() || it.IsVertex());
            }
        }

        // the fillets' surfaces are never kept: the default surface explorer finds none (OCCT's default)
        Assert.Equal(0, new TopOpeBRepDS_SurfaceExplorer(aDS).NbSurface());
        Assert.Equal(aDS.NbSurfaces(), new TopOpeBRepDS_SurfaceExplorer(aDS, false).NbSurface());
    }
}
