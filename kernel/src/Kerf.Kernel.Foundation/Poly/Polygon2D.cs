// Port of: src/FoundationClasses/TKMath/Poly/Poly_Polygon2D.hxx
// Port of: src/FoundationClasses/TKMath/Poly/Poly_Polygon2D.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Engine;
using System.Globalization;

namespace Kerf.Engine.Meshing;

/// <summary>
/// A polygon in 2D space, generally an approximation of a curve: a table of nodes from 1 (a closed polygon repeats its
/// first node at the end) and a deflection.
/// </summary>
/// <remarks>A mutable container, as in OCCT. <c>DumpJson</c> is left out.</remarks>
internal class Polygon2D : Transient
{
    private double _deflection;
    private readonly NCollection_Array1<Point2D> _nodes;

    /// <summary>A polygon of <paramref name="nbNodes"/> nodes at the origin.</summary>
    public Polygon2D(int nbNodes)
    {
        _deflection = 0.0;
        _nodes = new NCollection_Array1<Point2D>(1, nbNodes);
    }

    /// <summary>A polygon of copies of the nodes, renumbered from 1.</summary>
    public Polygon2D(NCollection_Array1<Point2D> nodes)
    {
        _deflection = 0.0;
        _nodes = new NCollection_Array1<Point2D>(1, nodes.Length());
        int i, j = 1;
        for (i = nodes.Lower(); i <= nodes.Upper(); i++)
        {
            _nodes[j++] = nodes[i];
        }
    }

    /// <summary>A copy of this polygon.</summary>
    public virtual Polygon2D Copy()
    {
        var copy = new Polygon2D(NodeCount);
        copy.ChangeNodes().CopyValues(_nodes);
        copy.Deflection(_deflection);
        return copy;
    }

    /// <summary>
    /// The deflection of this polygon: the largest distance allowed between the curve it approximates and the polygon.
    /// 0 by default, meaning a true polygon rather than an approximation.
    /// </summary>
    public double Deflection() => _deflection;

    /// <summary>Sets the deflection of this polygon.</summary>
    public void Deflection(double defl)
    {
        CheckMutable();
        _deflection = defl;
    }

    /// <summary>The number of nodes; a closed polygon repeats its first node, so a closed triangle has 4.</summary>
    public int NodeCount => _nodes.Length();

    /// <summary>The nodes, from 1.</summary>
    public NCollection_Array1<Point2D> Nodes() => _nodes;

    /// <summary>The nodes, from 1, for writing.</summary>
    public NCollection_Array1<Point2D> ChangeNodes()
    {
        CheckMutable();
        return _nodes;
    }

    public override string ToString()
        => string.Create(CultureInfo.InvariantCulture, $"Poly_Polygon2D({NodeCount} nodes, deflection {_deflection:R})");
}
