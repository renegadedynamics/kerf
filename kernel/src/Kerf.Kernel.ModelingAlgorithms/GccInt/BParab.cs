// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccInt/GccInt_BParab.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccInt/GccInt_BParab.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Engine.Construction.Constraints.ConstraintCurveType;

namespace Kerf.Engine.Construction.Constraints;

/// <summary>A bisecting curve whose geometry is a 2D parabola.</summary>
internal sealed class BParab : ConstraintBisec
{
    private readonly Parabola2D _par;

    public BParab(in Parabola2D parab)
    {
        _par = parab;
    }

    /// <summary>The geometry of this bisecting curve.</summary>
    public override Parabola2D Parabola()
    {
        return _par;
    }

    /// <summary>GccInt_Par.</summary>
    public override ConstraintCurveType ArcType()
    {
        return ConstraintCurveType.Par;
    }
}
