// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans;
using Occt;

namespace OcctNet.DrawLite;

/// <summary>
/// The session's Boolean, Section and Splitter builders (BOPTest_Objects::BOP, Section, Splitter), and the pave filler
/// of <c>bop</c> (BOPTest_BOPCommands.cxx's static <c>pPF</c>), unit 6L.
/// </summary>
internal sealed partial class BOPTestObjects
{
    private Bop? myBOP;
    private Section? mySection;
    private Splitter? mySplitter;

    /// <summary>The Boolean builder of <c>bbop</c> (made on first use, as OCCT's static).</summary>
    public Bop BOP() => myBOP ??= new Bop();

    /// <summary>The Section builder of <c>bbop ... section</c>.</summary>
    public Section Section() => mySection ??= new Section();

    /// <summary>The Splitter of <c>bsplit</c>.</summary>
    public Splitter Splitter() => mySplitter ??= new Splitter();

    /// <summary>
    /// The pave filler <c>bop</c> made, which <c>bopcommon</c>, <c>bopfuse</c>, <c>bopcut</c>, <c>boptuc</c> and
    /// <c>bopsection</c> use (OCCT's file-static <c>pPF</c>, process-wide there; per interpreter here). Null until
    /// <c>bop</c> runs; <c>bclear</c> doesn't reset it.
    /// </summary>
    public PaveFiller? BopPaveFiller { get; set; }
}
