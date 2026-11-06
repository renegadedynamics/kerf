// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccInt/GccInt_BLine.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccInt/GccInt_BLine.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Engine.Construction.Constraints;

/// <summary>A bisecting curve whose geometry is a 2D line.</summary>
internal sealed class BLine : ConstraintBisec
{
    private readonly Line2D _lin;

    public BLine(in Line2D line)
    {
        _lin = line;
    }

    /// <summary>The geometry of this bisecting curve.</summary>
    public override Line2D Line()
    {
        return _lin;
    }

    /// <summary>GccInt_Lin.</summary>
    public override ConstraintCurveType ArcType()
    {
        return ConstraintCurveType.Lin;
    }
}
