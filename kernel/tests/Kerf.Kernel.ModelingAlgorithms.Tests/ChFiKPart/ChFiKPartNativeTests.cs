// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// ChFiKPart's analytic fillets, chamfers and corners, and ChFi3d_Builder_0's free functions, against native OCCT
/// (tools/dsrec in its kpart mode, <see cref="KPartRecorder"/>): each case's surface data (surface, orientation, the
/// curves on both faces with their pcurves and transitions, the corner points) and the data structure it fills, byte
/// for byte.
/// </summary>
public class ChFiKPartNativeTests
{
    public static TheoryData<string> CaseNames()
    {
        TheoryData<string> aData = new();
        foreach ((string Name, string _, string _) in KPartRecorder.Cases)
        {
            aData.Add(Name);
        }
        return aData;
    }

    [Fact]
    public void TheRecordingHasTheCases()
    {
        // 25 fillets and chamfers on each order of the faces, a pair that is not particular, the toric, spherical and
        // rotule corners (6), ChFi3d_Builder_0's functions (6 groups) and three elementary spines
        Assert.Equal(66, KPartRecorder.Cases.Count);
        Assert.Contains(KPartRecorder.Cases, c => c.Name == "fil_cyl_cyl_not_implemented"
                                                  && c.Text.Contains("exception class Standard_NotImplemented"));
        Assert.Contains(KPartRecorder.Cases, c => c.Name == "corner_sphere_reversed" && c.Text.Contains("ok 0\n"));
    }

    [Theory]
    [MemberData(nameof(CaseNames))]
    public void TheCaseIsNatives(string theName)
    {
        (string Name, string BRep, string Text) aCase = KPartRecorder.Cases.First(c => c.Name == theName);
        string anActual = KPartRecorder.Run(aCase.Name, aCase.BRep);
        if (anActual != aCase.Text)
        {
            Assert.Fail(theName + ":\n" + string.Join("\n", DsRecordings.Differences(aCase.Text, anActual, 0.0)));
        }
    }
}
