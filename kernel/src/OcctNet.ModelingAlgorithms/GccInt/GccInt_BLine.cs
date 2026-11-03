// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccInt/GccInt_BLine.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccInt/GccInt_BLine.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Occt.GccInt_IType;

namespace Occt;

/// <summary>A bisecting curve whose geometry is a 2D line.</summary>
public sealed class GccInt_BLine : GccInt_Bisec
{
    private readonly Line2D lin;

    public GccInt_BLine(in Line2D Line)
    {
        lin = Line;
    }

    /// <summary>The geometry of this bisecting curve.</summary>
    public override Line2D Line()
    {
        return lin;
    }

    /// <summary>GccInt_Lin.</summary>
    public override GccInt_IType ArcType()
    {
        return GccInt_Lin;
    }
}
