// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Wire.cxx (from line 2130; merged into ShapeFix_Wire.cs)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

public partial class ShapeFix_Wire
{
    //PART2-BEGIN
    public bool FixDegenerated(int num) => throw new NotImplementedException();

    public bool FixLacking(int num, bool force = false) => throw new NotImplementedException();

    public bool FixNotchedEdges() => throw new NotImplementedException();

    public bool FixTails() => throw new NotImplementedException();

    protected void UpdateWire() => throw new NotImplementedException();

    private bool FixSelfIntersectingEdge(int num) => throw new NotImplementedException();

    private bool FixIntersectingEdges(int num) => throw new NotImplementedException();

    private bool FixIntersectingEdges(int num1, int num2) => throw new NotImplementedException();

    private void FixDummySeam(int num) => throw new NotImplementedException();
    //PART2-END
}
