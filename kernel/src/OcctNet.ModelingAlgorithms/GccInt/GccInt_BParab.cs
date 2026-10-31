// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccInt/GccInt_BParab.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccInt/GccInt_BParab.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GccInt_IType;

namespace Occt;

/// <summary>A bisecting curve whose geometry is a 2D parabola.</summary>
public sealed class GccInt_BParab : GccInt_Bisec
{
    private readonly gp_Parab2d par;

    public GccInt_BParab(in gp_Parab2d Parab)
    {
        par = Parab;
    }

    /// <summary>The geometry of this bisecting curve.</summary>
    public override gp_Parab2d Parabola()
    {
        return par;
    }

    /// <summary>GccInt_Par.</summary>
    public override GccInt_IType ArcType()
    {
        return GccInt_Par;
    }
}
