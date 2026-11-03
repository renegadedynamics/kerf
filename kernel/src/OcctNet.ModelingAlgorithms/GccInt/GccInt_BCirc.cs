// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccInt/GccInt_BCirc.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccInt/GccInt_BCirc.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Occt.GccInt_IType;

namespace Occt;

/// <summary>A bisecting curve whose geometry is a 2D circle.</summary>
public sealed class GccInt_BCirc : GccInt_Bisec
{
    private readonly Circle2D cir;

    public GccInt_BCirc(in Circle2D Circ)
    {
        cir = Circ;
    }

    /// <summary>The geometry of this bisecting curve.</summary>
    public override Circle2D Circle()
    {
        return cir;
    }

    /// <summary>GccInt_Cir.</summary>
    public override GccInt_IType ArcType()
    {
        return GccInt_Cir;
    }
}
