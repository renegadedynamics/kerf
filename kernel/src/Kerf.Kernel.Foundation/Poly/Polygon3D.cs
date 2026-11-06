// Port of: src/FoundationClasses/TKMath/Poly/Poly_Polygon3D.hxx
// Port of: src/FoundationClasses/TKMath/Poly/Poly_Polygon3D.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using System.Globalization;

namespace Kerf.Engine.Meshing;

/// <summary>
/// A polygon in 3D space, generally an approximation of a curve: a table of nodes from 1 (a closed polygon repeats its
/// first node at the end), optionally with the curve parameter of each node, and a deflection.
/// </summary>
/// <remarks>A mutable container, as in OCCT. <c>DumpJson</c> is left out.</remarks>
internal class Polygon3D : Transient
{
    private double _deflection;
    private readonly NCollection_Array1<Point3D> _nodes;
    private readonly NCollection_HArray1<double>? _parameters;

    /// <summary>A polygon of <paramref name="nbNodes"/> nodes at the origin, with parameters (zero) if asked.</summary>
    public Polygon3D(int nbNodes, bool hasParams)
    {
        _deflection = 0.0;
        _nodes = new NCollection_Array1<Point3D>(1, nbNodes);
        if (hasParams)
        {
            _parameters = new NCollection_HArray1<double>(1, nbNodes);
        }
    }

    /// <summary>A polygon of copies of the nodes, renumbered from 1.</summary>
    public Polygon3D(NCollection_Array1<Point3D> nodes)
    {
        _deflection = 0.0;
        _nodes = new NCollection_Array1<Point3D>(1, nodes.Length());
        int i, j = 1;
        for (i = nodes.Lower(); i <= nodes.Upper(); i++)
        {
            _nodes[j++] = nodes[i];
        }
    }

    /// <summary>
    /// A polygon of copies of the nodes and of their curve parameters, renumbered from 1. Both tables must have the same
    /// bounds; as in OCCT, this isn't checked.
    /// </summary>
    public Polygon3D(NCollection_Array1<Point3D> nodes, NCollection_Array1<double> p)
    {
        _deflection = 0.0;
        _nodes = new NCollection_Array1<Point3D>(1, nodes.Length());
        _parameters = new NCollection_HArray1<double>(1, p.Length());
        int i, j = 1;
        for (i = nodes.Lower(); i <= nodes.Upper(); i++)
        {
            _nodes[j] = nodes[i];
            _parameters.SetValue(j, p[i]);
            j++;
        }
    }

    /// <summary>A copy of this polygon.</summary>
    public virtual Polygon3D Copy()
    {
        var copy = new Polygon3D(NodeCount, _parameters is not null);
        copy.ChangeNodes().CopyValues(_nodes);
        if (_parameters is not null)
        {
            copy.ChangeParameters().CopyValues(_parameters.Array1());
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

    /// <summary>The nodes, from 1.</summary>
    public NCollection_Array1<Point3D> Nodes() => _nodes;

    /// <summary>The nodes, from 1, for writing.</summary>
    public NCollection_Array1<Point3D> ChangeNodes()
    {
        CheckMutable();
        return _nodes;
    }

    /// <summary>True if the nodes have curve parameters.</summary>
    public bool HasParameters => _parameters is not null;

    /// <summary>
    /// The curve parameters of the nodes. Throws <see cref="NullObjectException"/> if there are none, where OCCT
    /// dereferences a null handle.
    /// </summary>
    public NCollection_Array1<double> Parameters()
    {
        // OCCT dereferences the null handle
        Standard_NullObject_Raise_if(_parameters is null, "Poly_Polygon3D::Parameters : parameters is NULL");
        return _parameters!.Array1();
    }

    /// <summary>The curve parameters of the nodes, shared, for writing. Throws if there are none.</summary>
    public NCollection_Array1<double> ChangeParameters()
    {
        CheckMutable();
        Standard_NullObject_Raise_if(_parameters is null, "Poly_Polygon3D::ChangeParameters : parameters is NULL");
        return _parameters!.ChangeArray1();
    }

    public override string ToString()
        => string.Create(CultureInfo.InvariantCulture, $"Poly_Polygon3D({NodeCount} nodes, deflection {_deflection:R})");
}
