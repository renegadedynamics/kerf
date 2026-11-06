// Port of: src/FoundationClasses/TKMath/Poly/Poly_PolygonOnTriangulation.hxx
// Port of: src/FoundationClasses/TKMath/Poly/Poly_PolygonOnTriangulation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Globalization;

namespace Kerf.Engine.Meshing;

/// <summary>
/// A polygon on a triangulation, generally the approximation of a curve on a surface: a table of node indices into a
/// triangulation (a closed polygon repeats its first index at the end), optionally with the curve parameter of each
/// node, and a deflection.
/// </summary>
/// <remarks>A mutable container, as in OCCT. <c>DumpJson</c> is left out.</remarks>
internal class PolygonOnTriangulation : Transient
{
    private double _deflection;
    private readonly NCollection_Array1<int> _nodes;
    private NCollection_HArray1<double>? _parameters;

    /// <summary>A polygon of <paramref name="nbNodes"/> node indices (0), with parameters (zero) if asked.</summary>
    public PolygonOnTriangulation(int nbNodes, bool hasParams)
    {
        _deflection = 0.0;
        _nodes = new NCollection_Array1<int>(1, nbNodes);
        if (hasParams)
        {
            _parameters = new NCollection_HArray1<double>(1, nbNodes);
        }
    }

    /// <summary>A polygon of a copy of the node indices.</summary>
    /// <remarks>As in OCCT, the table takes the bounds of <paramref name="nodes"/>.</remarks>
    public PolygonOnTriangulation(NCollection_Array1<int> nodes)
    {
        _deflection = 0.0;
        _nodes = new NCollection_Array1<int>(1, nodes.Length());
        _nodes.Assign(nodes); // OCCT: the bounds of Nodes are taken too
    }

    /// <summary>
    /// A polygon of copies of the node indices and of their curve parameters. The tables must have the same size; as in
    /// OCCT this isn't checked, and each table takes the bounds of its argument.
    /// </summary>
    public PolygonOnTriangulation(NCollection_Array1<int> nodes, NCollection_Array1<double> parameters)
    {
        _deflection = 0.0;
        _nodes = new NCollection_Array1<int>(1, nodes.Length());
        _nodes.Assign(nodes);
        _parameters = new NCollection_HArray1<double>(1, parameters.Length());
        _parameters.ChangeArray1().Assign(parameters);
    }

    /// <summary>A copy of this polygon.</summary>
    public virtual PolygonOnTriangulation Copy()
    {
        var copy = new PolygonOnTriangulation(NodeCount, _parameters is not null);
        copy.ChangeNodeArray().CopyValues(_nodes);
        if (_parameters is not null)
        {
            copy.ChangeParameterArray().CopyValues(_parameters.Array1());
        }

        copy.Deflection(_deflection);
        return copy;
    }

    /// <summary>The deflection of this polygon.</summary>
    public double Deflection() => _deflection;

    /// <summary>Sets the deflection of this polygon (see <see cref="Polygon2D.Deflection()"/>).</summary>
    public void Deflection(double defl)
    {
        CheckMutable();
        _deflection = defl;
    }

    /// <summary>The number of nodes; a closed polygon repeats its first node, so a closed triangle has 4.</summary>
    public int NodeCount => _nodes.Length();

    /// <summary>The triangulation node index at an index of this polygon.</summary>
    public int Node(int index) => _nodes.Value(index);

    /// <summary>The node indices, for writing.</summary>
    public NCollection_Array1<int> ChangeNodeArray()
    {
        CheckMutable();
        return _nodes;
    }

    /// <summary>Sets the triangulation node index at an index of this polygon.</summary>
    public void SetNode(int index, int node)
    {
        CheckMutable();
        _nodes.SetValue(index, node);
    }

    /// <summary>True if the nodes have curve parameters.</summary>
    public bool HasParameters => _parameters is not null;

    /// <summary>The curve parameter at an index. Throws <see cref="NullObjectException"/> if there are no parameters.</summary>
    public double Parameter(int index)
    {
        Standard_NullObject_Raise_if(_parameters is null, "Poly_PolygonOnTriangulation::Parameter : parameters is NULL");
        return _parameters!.Value(index);
    }

    /// <summary>Sets the curve parameter at an index. Throws <see cref="NullObjectException"/> if there are no parameters.</summary>
    public void SetParameter(int index, double value)
    {
        CheckMutable();
        Standard_NullObject_Raise_if(_parameters is null, "Poly_PolygonOnTriangulation::Parameter : parameters is NULL");
        _parameters!.SetValue(index, value);
    }

    /// <summary>The curve parameters, for writing. Throws <see cref="NullObjectException"/> if there are none.</summary>
    public NCollection_Array1<double> ChangeParameterArray()
    {
        CheckMutable();
        Standard_NullObject_Raise_if(_parameters is null, "Poly_PolygonOnTriangulation::Parameter : parameters is NULL");
        return _parameters!.ChangeArray1();
    }

    /// <summary>
    /// Sets the curve parameters (shared, not copied), or removes them with null. Throws
    /// <see cref="OutOfRangeException"/> if the bounds differ from the nodes'.
    /// </summary>
    public void SetParameters(NCollection_HArray1<double>? parameters)
    {
        CheckMutable();
        if (parameters is not null
            && (parameters.Lower() != _nodes.Lower() || parameters.Upper() != _nodes.Upper()))
        {
            throw new OutOfRangeException("Poly_PolygonOnTriangulation::SetParameters() - invalid array size");
        }

        _parameters = parameters;
    }

    /// <summary>The node indices: indices into the nodes of a triangulation.</summary>
    public NCollection_Array1<int> Nodes() => _nodes;

    /// <summary>The curve parameters, or null; check <see cref="HasParameters"/>.</summary>
    public NCollection_HArray1<double>? Parameters() => _parameters;
    public override string ToString()
        => string.Create(CultureInfo.InvariantCulture, $"Poly_PolygonOnTriangulation({NodeCount} nodes, deflection {_deflection:R})");
}
