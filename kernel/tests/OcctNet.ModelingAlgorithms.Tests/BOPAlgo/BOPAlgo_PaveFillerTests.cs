// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Xunit.Abstractions;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>Unit tests of the pave filler on primitives.</summary>
public class BOPAlgo_PaveFillerTests(ITestOutputHelper output)
{
    internal static BOPAlgo_PaveFiller Fill(bool theParallel, params TopoDS_Shape[] theArgs)
    {
        BOPAlgo_PaveFiller aPF = new();
        NCollection_List<TopoDS_Shape> aLS = new();
        foreach (TopoDS_Shape aS in theArgs)
        {
            aLS.Append(aS);
        }
        aPF.SetArguments(aLS);
        aPF.SetRunParallel(theParallel);
        aPF.Perform();
        return aPF;
    }

    [Fact]
    public void Smoke_TwoBoxes()
    {
        TopoDS_Shape a = new BRepPrimAPI_MakeBox(new gp_Pnt(0, 0, 0), 1, 1, 1).Shape();
        TopoDS_Shape b = new BRepPrimAPI_MakeBox(new gp_Pnt(0.5, 0.25, 0.25), 1, 0.5, 0.5).Shape();
        BOPAlgo_PaveFiller aPF = Fill(false, a, b);
        using StringWriter anErr = new();
        aPF.DumpErrors(anErr);
        output.WriteLine(anErr.ToString());
        Assert.False(aPF.HasErrors());
        output.WriteLine(PaveFillerDump.Write(aPF.DS()));
    }
}
