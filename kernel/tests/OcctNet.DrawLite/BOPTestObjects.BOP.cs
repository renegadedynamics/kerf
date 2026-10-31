// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace OcctNet.DrawLite;

/// <summary>
/// The session's Boolean, Section and Splitter builders (BOPTest_Objects::BOP, Section, Splitter), and the pave filler
/// of <c>bop</c> (BOPTest_BOPCommands.cxx's static <c>pPF</c>), unit 6L.
/// </summary>
internal sealed partial class BOPTestObjects
{
    private BOPAlgo_BOP? myBOP;
    private BOPAlgo_Section? mySection;
    private BOPAlgo_Splitter? mySplitter;

    /// <summary>The Boolean builder of <c>bbop</c> (made on first use, as OCCT's static).</summary>
    public BOPAlgo_BOP BOP() => myBOP ??= new BOPAlgo_BOP();

    /// <summary>The Section builder of <c>bbop ... section</c>.</summary>
    public BOPAlgo_Section Section() => mySection ??= new BOPAlgo_Section();

    /// <summary>The Splitter of <c>bsplit</c>.</summary>
    public BOPAlgo_Splitter Splitter() => mySplitter ??= new BOPAlgo_Splitter();

    /// <summary>
    /// The pave filler <c>bop</c> made, which <c>bopcommon</c>, <c>bopfuse</c>, <c>bopcut</c>, <c>boptuc</c> and
    /// <c>bopsection</c> use (OCCT's file-static <c>pPF</c>, process-wide there; per interpreter here). Null until
    /// <c>bop</c> runs; <c>bclear</c> doesn't reset it.
    /// </summary>
    public BOPAlgo_PaveFiller? BopPaveFiller { get; set; }
}
