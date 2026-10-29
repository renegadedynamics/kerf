// Port of: src/FoundationClasses/TKMath/Poly/Poly_Polygon3D.hxx
// Port of: src/FoundationClasses/TKMath/Poly/Poly_Polygon3D.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;

namespace Occt;

/// <summary>
/// A polygon in 3D space, generally an approximation of a curve: a table of nodes from 1 (a closed polygon repeats its
/// first node at the end), optionally with the curve parameter of each node, and a deflection.
/// </summary>
/// <remarks>A mutable container, as in OCCT. <c>DumpJson</c> is left out.</remarks>
public class Poly_Polygon3D : Standard_Transient
{
    private double myDeflection;
    private readonly NCollection_Array1<gp_Pnt> myNodes;
    private readonly NCollection_HArray1<double>? myParameters;

    /// <summary>A polygon of <paramref name="theNbNodes"/> nodes at the origin, with parameters (zero) if asked.</summary>
    public Poly_Polygon3D(int theNbNodes, bool theHasParams)
    {
        myDeflection = 0.0;
        myNodes = new NCollection_Array1<gp_Pnt>(1, theNbNodes);
        if (theHasParams)
        {
            myParameters = new NCollection_HArray1<double>(1, theNbNodes);
        }
    }

    /// <summary>A polygon of copies of the nodes, renumbered from 1.</summary>
    public Poly_Polygon3D(NCollection_Array1<gp_Pnt> Nodes)
    {
        myDeflection = 0.0;
        myNodes = new NCollection_Array1<gp_Pnt>(1, Nodes.Length());
        int i, j = 1;
        for (i = Nodes.Lower(); i <= Nodes.Upper(); i++)
        {
            myNodes[j++] = Nodes[i];
        }
    }

    /// <summary>
    /// A polygon of copies of the nodes and of their curve parameters, renumbered from 1. Both tables must have the same
    /// bounds; as in OCCT, this isn't checked.
    /// </summary>
    public Poly_Polygon3D(NCollection_Array1<gp_Pnt> Nodes, NCollection_Array1<double> P)
    {
        myDeflection = 0.0;
        myNodes = new NCollection_Array1<gp_Pnt>(1, Nodes.Length());
        myParameters = new NCollection_HArray1<double>(1, P.Length());
        int i, j = 1;
        for (i = Nodes.Lower(); i <= Nodes.Upper(); i++)
        {
            myNodes[j] = Nodes[i];
            myParameters.SetValue(j, P[i]);
            j++;
        }
    }

    /// <summary>A copy of this polygon.</summary>
    public virtual Poly_Polygon3D Copy()
    {
        var aCopy = new Poly_Polygon3D(NbNodes(), myParameters is not null);
        aCopy.ChangeNodes().CopyValues(myNodes);
        if (myParameters is not null)
        {
            aCopy.ChangeParameters().CopyValues(myParameters.Array1());
        }

        aCopy.Deflection(myDeflection);
        return aCopy;
    }

    /// <summary>The deflection of this polygon.</summary>
    public double Deflection() => myDeflection;

    /// <summary>Sets the deflection of this polygon (see <see cref="Poly_Polygon2D.Deflection()"/>).</summary>
    public void Deflection(double theDefl) => myDeflection = theDefl;

    /// <summary>The number of nodes; a closed polygon repeats its first node, so a closed triangle has 4.</summary>
    public int NbNodes() => myNodes.Length();

    /// <summary>The nodes, from 1.</summary>
    public NCollection_Array1<gp_Pnt> Nodes() => myNodes;

    /// <summary>The nodes, from 1, for writing.</summary>
    public NCollection_Array1<gp_Pnt> ChangeNodes() => myNodes;

    /// <summary>True if the nodes have curve parameters.</summary>
    public bool HasParameters() => myParameters is not null;

    /// <summary>
    /// The curve parameters of the nodes. Throws <see cref="Standard_NullObject"/> if there are none, where OCCT
    /// dereferences a null handle.
    /// </summary>
    public NCollection_Array1<double> Parameters()
    {
        Standard_NullObject_Raise_if(myParameters is null, "Poly_Polygon3D::Parameters : parameters is NULL");
        return myParameters!.Array1();
    }

    /// <summary>The curve parameters of the nodes, shared, for writing. Throws if there are none.</summary>
    public NCollection_Array1<double> ChangeParameters()
    {
        Standard_NullObject_Raise_if(myParameters is null, "Poly_Polygon3D::ChangeParameters : parameters is NULL");
        return myParameters!.ChangeArray1();
    }

    public override string ToString()
        => string.Create(CultureInfo.InvariantCulture, $"Poly_Polygon3D({NbNodes()} nodes, deflection {myDeflection:R})");
}
