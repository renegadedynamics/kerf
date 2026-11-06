// Port of: src/ModelingData/TKG3d/TopAbs/TopAbs_ShapeEnum.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Topology;

/// <summary>The type of a shape, from the most complex (COMPOUND) to the simplest (VERTEX); SHAPE means any type.</summary>
public enum ShapeType
{
    /// <summary>A group of any shapes.</summary>
    Compound,
    /// <summary>Solids that share faces.</summary>
    CompSolid,
    /// <summary>A region of space bounded by shells.</summary>
    Solid,
    /// <summary>Faces joined along their edges.</summary>
    Shell,
    /// <summary>Part of a surface bounded by wires.</summary>
    Face,
    /// <summary>Edges joined at their vertices.</summary>
    Wire,
    /// <summary>Part of a curve bounded by vertices.</summary>
    Edge,
    /// <summary>A point.</summary>
    Vertex,
    /// <summary>Any type: what a search or a filter uses to mean no restriction.</summary>
    Shape,
}
