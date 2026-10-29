// Port of: src/ModelingData/TKGeomBase/AdvApp2Var/AdvApp2Var_Data.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

// The Fortran common blocks of AdvApp2Var's constants (decision 902). OCCT returns a mutable reference to a static
// struct, but nothing writes them: the port returns `ref readonly` structs for the scalar blocks and ref structs of
// read-only f2c pointers (F2cConstPtr) for the tables. A table's fields are views of one constant array, as OCCT's
// struct is a cast of one C array, so an index past a field reads the next field as it does in C.

/// <summary>The <c>mdnombr</c> common block: angular constants.</summary>
internal struct mdnombr_1_
{
    public double pi, deuxpi, pisur2, pis180, c180pi, zero, one, a180, a360, a90;
}

/// <summary>The <c>minombr</c> common block: the integers 0 to 1000 (<c>int nbr[1001]</c>).</summary>
internal readonly ref struct minombr_1_
{
    public readonly F2cConstPtr<int> nbr;

    internal minombr_1_(ReadOnlySpan<int> theTable) => nbr = theTable;
}

/// <summary>The <c>maovpar</c> common block: machine limits.</summary>
internal struct maovpar_1_
{
    public double r8und, r8ovr, x4und, x4ovr;
    public float r4und, r4ovr;
    public int r4nbe, r8nbm, r8nbe, i4ovr, i4ovn, r4exp, r8exp, r4exn, r8exn, r4ncs, r8ncs, r4nbm;
    public short i2ovr, i2ovn;
}

/// <summary>The <c>maovpch</c> common block: machine name and formats (OCCT's fixed-size char arrays are strings).</summary>
internal struct maovpch_1_
{
    public string cnmmac, frmr4, frmr8, cdcode;
}

/// <summary>Access to AdvApp2Var's constant common blocks.</summary>
internal static partial class AdvApp2Var_Data;
