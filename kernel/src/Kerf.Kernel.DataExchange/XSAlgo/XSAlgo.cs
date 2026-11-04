// Port of: src/DataExchange/TKXSBase/XSAlgo/XSAlgo.hxx
// Port of: src/DataExchange/TKXSBase/XSAlgo/XSAlgo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2000-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Healing.Process;
using Occt;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>Provides interface to the algorithms from Shape Healing and others for XSTEP processors.</summary>
/// <remarks>
/// <para>
/// OCCT's <c>Init</c> declares data exchange's statics in the process-wide <c>Interface_Static</c> once, creates the
/// default AlgoContainer and registers ShapeProcess's operators. The port (plan 2.7, decision 1203): <see cref="Init"/>
/// declares the statics in the given <see cref="StaticSet"/>, once per set; the operators are registered once
/// per process (decision 1021) and ShapeAlgo's container created once (phase 7); the default AlgoContainer is one
/// immutable-in-use object created once. <c>SetAlgoContainer</c>, which replaces it process-wide, is left out, and
/// <see cref="AlgoContainer"/> returns the default container even before <see cref="Init"/> (OCCT: null).
/// </para>
/// </remarks>
internal static class XSAlgo
{
    private static readonly FrameworkAlgoContainer Container = new();

    /// <summary>
    /// Creates and initializes default AlgoContainer: declares the statics of XSTEP's shape processing
    /// (<c>read.stdsameparameter.mode</c>, <c>xstep.cascade.unit</c>, after <see cref="StaticSet.Standards"/>)
    /// in <paramref name="statics"/>, once per set, and registers the standard shape processing operators.
    /// </summary>
    public static void Init(StaticSet statics)
    {
        if (!statics.StartInit("XSAlgo::Init"))
        {
            return;
        }
        global::Kerf.Engine.Healing.ShapeAlgo.Init();

        // init parameters
        statics.Standards();

        // #74 rln 10.03.99 S4135: adding new parameter for handling use of BRepLib::SameParameter
        statics.Init("XSTEP", "read.stdsameparameter.mode", 'e', "");
        statics.Init("XSTEP", "read.stdsameparameter.mode", '&', "ematch 0");
        statics.Init("XSTEP", "read.stdsameparameter.mode", '&', "eval Off");
        statics.Init("XSTEP", "read.stdsameparameter.mode", '&', "eval On");
        statics.SetIVal("read.stdsameparameter.mode", 0);

        // unit: supposed to be cascade unit (target unit for reading)
        statics.Init("XSTEP", "xstep.cascade.unit", 'e', "");
        statics.Init("XSTEP", "xstep.cascade.unit", '&', "enum 1");
        statics.Init("XSTEP", "xstep.cascade.unit", '&', "eval INCH"); // 1
        statics.Init("XSTEP", "xstep.cascade.unit", '&', "eval MM");   // 2
        statics.Init("XSTEP", "xstep.cascade.unit", '&', "eval ??");   // 3
        statics.Init("XSTEP", "xstep.cascade.unit", '&', "eval FT");   // 4
        statics.Init("XSTEP", "xstep.cascade.unit", '&', "eval MI");   // 5
        statics.Init("XSTEP", "xstep.cascade.unit", '&', "eval M");    // 6
        statics.Init("XSTEP", "xstep.cascade.unit", '&', "eval KM");   // 7
        statics.Init("XSTEP", "xstep.cascade.unit", '&', "eval MIL");  // 8
        statics.Init("XSTEP", "xstep.cascade.unit", '&', "eval UM");   // 9
        statics.Init("XSTEP", "xstep.cascade.unit", '&', "eval CM");   // 10
        statics.Init("XSTEP", "xstep.cascade.unit", '&', "eval UIN");  // 11
        statics.SetCVal("xstep.cascade.unit", "MM");

        // init Standard Shape Processing operators
        OperLibrary.Init();
    }

    /// <summary>Returns default AlgoContainer.</summary>
    public static FrameworkAlgoContainer AlgoContainer()
    {
        return Container;
    }
}
