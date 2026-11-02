// Port of: src/DataExchange/TKXSBase/XSAlgo/XSAlgo.hxx
// Port of: src/DataExchange/TKXSBase/XSAlgo/XSAlgo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2000-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Provides interface to the algorithms from Shape Healing and others for XSTEP processors.</summary>
/// <remarks>
/// <para>
/// OCCT's <c>Init</c> declares data exchange's statics in the process-wide <c>Interface_Static</c> once, creates the
/// default AlgoContainer and registers ShapeProcess's operators. The port (plan 2.7, decision 12405): <see cref="Init"/>
/// declares the statics in the given <see cref="Interface_StaticSet"/>, once per set; the operators are registered once
/// per process (decision 1021) and ShapeAlgo's container created once (phase 7); the default AlgoContainer is one
/// immutable-in-use object created once. <c>SetAlgoContainer</c>, which replaces it process-wide, is left out, and
/// <see cref="AlgoContainer"/> returns the default container even before <see cref="Init"/> (OCCT: null).
/// </para>
/// </remarks>
public static class XSAlgo
{
    private static readonly XSAlgo_AlgoContainer theContainer = new();

    /// <summary>
    /// Creates and initializes default AlgoContainer: declares the statics of XSTEP's shape processing
    /// (<c>read.stdsameparameter.mode</c>, <c>xstep.cascade.unit</c>, after <see cref="Interface_StaticSet.Standards"/>)
    /// in <paramref name="theStatics"/>, once per set, and registers the standard shape processing operators.
    /// </summary>
    public static void Init(Interface_StaticSet theStatics)
    {
        if (!theStatics.StartInit("XSAlgo::Init"))
        {
            return;
        }
        ShapeAlgo.Init();

        // init parameters
        theStatics.Standards();

        // #74 rln 10.03.99 S4135: adding new parameter for handling use of BRepLib::SameParameter
        theStatics.Init("XSTEP", "read.stdsameparameter.mode", 'e', "");
        theStatics.Init("XSTEP", "read.stdsameparameter.mode", '&', "ematch 0");
        theStatics.Init("XSTEP", "read.stdsameparameter.mode", '&', "eval Off");
        theStatics.Init("XSTEP", "read.stdsameparameter.mode", '&', "eval On");
        theStatics.SetIVal("read.stdsameparameter.mode", 0);

        // unit: supposed to be cascade unit (target unit for reading)
        theStatics.Init("XSTEP", "xstep.cascade.unit", 'e', "");
        theStatics.Init("XSTEP", "xstep.cascade.unit", '&', "enum 1");
        theStatics.Init("XSTEP", "xstep.cascade.unit", '&', "eval INCH"); // 1
        theStatics.Init("XSTEP", "xstep.cascade.unit", '&', "eval MM");   // 2
        theStatics.Init("XSTEP", "xstep.cascade.unit", '&', "eval ??");   // 3
        theStatics.Init("XSTEP", "xstep.cascade.unit", '&', "eval FT");   // 4
        theStatics.Init("XSTEP", "xstep.cascade.unit", '&', "eval MI");   // 5
        theStatics.Init("XSTEP", "xstep.cascade.unit", '&', "eval M");    // 6
        theStatics.Init("XSTEP", "xstep.cascade.unit", '&', "eval KM");   // 7
        theStatics.Init("XSTEP", "xstep.cascade.unit", '&', "eval MIL");  // 8
        theStatics.Init("XSTEP", "xstep.cascade.unit", '&', "eval UM");   // 9
        theStatics.Init("XSTEP", "xstep.cascade.unit", '&', "eval CM");   // 10
        theStatics.Init("XSTEP", "xstep.cascade.unit", '&', "eval UIN");  // 11
        theStatics.SetCVal("xstep.cascade.unit", "MM");

        // init Standard Shape Processing operators
        ShapeProcess_OperLibrary.Init();
    }

    /// <summary>Returns default AlgoContainer.</summary>
    public static XSAlgo_AlgoContainer AlgoContainer()
    {
        return theContainer;
    }
}
