// Port of: src/FoundationClasses/TKMath/Poly/Poly_PolygonOnTriangulation.hxx
// Port of: src/FoundationClasses/TKMath/Poly/Poly_PolygonOnTriangulation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;

namespace Occt;

/// <summary>
/// A polygon on a triangulation, generally the approximation of a curve on a surface: a table of node indices into a
/// triangulation (a closed polygon repeats its first index at the end), optionally with the curve parameter of each
/// node, and a deflection.
/// </summary>
/// <remarks>A mutable container, as in OCCT. <c>DumpJson</c> is left out.</remarks>
public class Poly_PolygonOnTriangulation : Standard_Transient
{
    private double myDeflection;
    private readonly NCollection_Array1<int> myNodes;
    private NCollection_HArray1<double>? myParameters;

    /// <summary>A polygon of <paramref name="theNbNodes"/> node indices (0), with parameters (zero) if asked.</summary>
    public Poly_PolygonOnTriangulation(int theNbNodes, bool theHasParams)
    {
        myDeflection = 0.0;
        myNodes = new NCollection_Array1<int>(1, theNbNodes);
        if (theHasParams)
        {
            myParameters = new NCollection_HArray1<double>(1, theNbNodes);
        }
    }

    /// <summary>A polygon of a copy of the node indices.</summary>
    /// <remarks>As in OCCT, the table takes the bounds of <paramref name="Nodes"/>.</remarks>
    public Poly_PolygonOnTriangulation(NCollection_Array1<int> Nodes)
    {
        myDeflection = 0.0;
        myNodes = new NCollection_Array1<int>(1, Nodes.Length());
        myNodes.Assign(Nodes); // OCCT: the bounds of Nodes are taken too
    }

    /// <summary>
    /// A polygon of copies of the node indices and of their curve parameters. The tables must have the same size; as in
    /// OCCT this isn't checked, and each table takes the bounds of its argument.
    /// </summary>
    public Poly_PolygonOnTriangulation(NCollection_Array1<int> Nodes, NCollection_Array1<double> Parameters)
    {
        myDeflection = 0.0;
        myNodes = new NCollection_Array1<int>(1, Nodes.Length());
        myNodes.Assign(Nodes);
        myParameters = new NCollection_HArray1<double>(1, Parameters.Length());
        myParameters.ChangeArray1().Assign(Parameters);
    }

    /// <summary>A copy of this polygon.</summary>
    public virtual Poly_PolygonOnTriangulation Copy()
    {
        var aCopy = new Poly_PolygonOnTriangulation(NbNodes(), myParameters is not null);
        aCopy.ChangeNodeArray().CopyValues(myNodes);
        if (myParameters is not null)
        {
            aCopy.ChangeParameterArray().CopyValues(myParameters.Array1());
        }

        aCopy.Deflection(myDeflection);
        return aCopy;
    }

    /// <summary>The deflection of this polygon.</summary>
    public double Deflection() => myDeflection;

    /// <summary>Sets the deflection of this polygon (see <see cref="Poly_Polygon2D.Deflection()"/>).</summary>
    public void Deflection(double theDefl)
    {
        CheckMutable();
        myDeflection = theDefl;
    }

    /// <summary>The number of nodes; a closed polygon repeats its first node, so a closed triangle has 4.</summary>
    public int NbNodes() => myNodes.Length();

    /// <summary>The triangulation node index at an index of this polygon.</summary>
    public int Node(int theIndex) => myNodes.Value(theIndex);

    /// <summary>The node indices, for writing.</summary>
    public NCollection_Array1<int> ChangeNodeArray()
    {
        CheckMutable();
        return myNodes;
    }

    /// <summary>Sets the triangulation node index at an index of this polygon.</summary>
    public void SetNode(int theIndex, int theNode)
    {
        CheckMutable();
        myNodes.SetValue(theIndex, theNode);
    }

    /// <summary>True if the nodes have curve parameters.</summary>
    public bool HasParameters() => myParameters is not null;

    /// <summary>The curve parameter at an index. Throws <see cref="Standard_NullObject"/> if there are no parameters.</summary>
    public double Parameter(int theIndex)
    {
        Standard_NullObject_Raise_if(myParameters is null, "Poly_PolygonOnTriangulation::Parameter : parameters is NULL");
        return myParameters!.Value(theIndex);
    }

    /// <summary>Sets the curve parameter at an index. Throws <see cref="Standard_NullObject"/> if there are no parameters.</summary>
    public void SetParameter(int theIndex, double theValue)
    {
        CheckMutable();
        Standard_NullObject_Raise_if(myParameters is null, "Poly_PolygonOnTriangulation::Parameter : parameters is NULL");
        myParameters!.SetValue(theIndex, theValue);
    }

    /// <summary>The curve parameters, for writing. Throws <see cref="Standard_NullObject"/> if there are none.</summary>
    public NCollection_Array1<double> ChangeParameterArray()
    {
        CheckMutable();
        Standard_NullObject_Raise_if(myParameters is null, "Poly_PolygonOnTriangulation::Parameter : parameters is NULL");
        return myParameters!.ChangeArray1();
    }

    /// <summary>
    /// Sets the curve parameters (shared, not copied), or removes them with null. Throws
    /// <see cref="Standard_OutOfRange"/> if the bounds differ from the nodes'.
    /// </summary>
    public void SetParameters(NCollection_HArray1<double>? theParameters)
    {
        CheckMutable();
        if (theParameters is not null
            && (theParameters.Lower() != myNodes.Lower() || theParameters.Upper() != myNodes.Upper()))
        {
            throw new Standard_OutOfRange("Poly_PolygonOnTriangulation::SetParameters() - invalid array size");
        }

        myParameters = theParameters;
    }

    /// <summary>The node indices: indices into the nodes of a triangulation.</summary>
    public NCollection_Array1<int> Nodes() => myNodes;

    /// <summary>The curve parameters, or null; check <see cref="HasParameters"/>.</summary>
    public NCollection_HArray1<double>? Parameters() => myParameters;

    [Obsolete("Deprecated method, SetNode() should be used instead")]
    public NCollection_Array1<int> ChangeNodes()
    {
        CheckMutable();
        return myNodes;
    }

    [Obsolete("Deprecated method, SetParameter() should be used instead")]
    public NCollection_Array1<double> ChangeParameters()
    {
        CheckMutable();
        return myParameters!.ChangeArray1();
    }

    public override string ToString()
        => string.Create(CultureInfo.InvariantCulture, $"Poly_PolygonOnTriangulation({NbNodes()} nodes, deflection {myDeflection:R})");
}
