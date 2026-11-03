// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccInt/GccInt_BPoint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccInt/GccInt_BPoint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Occt.GccInt_IType;

namespace Occt;

/// <summary>A bisecting curve whose geometry is a 2D point.</summary>
public sealed class GccInt_BPoint : GccInt_Bisec
{
    private readonly Point2D pnt;

    public GccInt_BPoint(in Point2D Point)
    {
        pnt = Point;
    }

    /// <summary>The geometry of this bisecting curve.</summary>
    public override Point2D Point()
    {
        return pnt;
    }

    /// <summary>GccInt_Pnt.</summary>
    public override GccInt_IType ArcType()
    {
        return GccInt_Pnt;
    }
}
