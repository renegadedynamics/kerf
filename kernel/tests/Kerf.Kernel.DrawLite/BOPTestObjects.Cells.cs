// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans;
using Kerf.Engine;

namespace Kerf.Kernel.DrawLite;

/// <summary>The session's cells builder (BOPTest_Objects::CellsBuilder), unit 6G.</summary>
internal sealed partial class BOPTestObjects
{
    private CellsBuilder? myCellsBuilder;

    /// <summary>The cells builder of <c>bcbuild</c> and the other <c>bc*</c> commands (OCCT's static, never reset by <c>bclear</c>).</summary>
    public CellsBuilder CellsBuilder() => myCellsBuilder ??= new CellsBuilder();
}
