// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace OcctNet.DrawLite;

/// <summary>The session's cells builder (BOPTest_Objects::CellsBuilder), unit 6G.</summary>
internal sealed partial class BOPTestObjects
{
    private BOPAlgo_CellsBuilder? myCellsBuilder;

    /// <summary>The cells builder of <c>bcbuild</c> and the other <c>bc*</c> commands (OCCT's static, never reset by <c>bclear</c>).</summary>
    public BOPAlgo_CellsBuilder CellsBuilder() => myCellsBuilder ??= new BOPAlgo_CellsBuilder();
}
