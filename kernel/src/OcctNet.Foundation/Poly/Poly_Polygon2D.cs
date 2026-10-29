// Port of: src/FoundationClasses/TKMath/Poly/Poly_Polygon2D.hxx
// Port of: src/FoundationClasses/TKMath/Poly/Poly_Polygon2D.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;

namespace Occt;

/// <summary>
/// A polygon in 2D space, generally an approximation of a curve: a table of nodes from 1 (a closed polygon repeats its
/// first node at the end) and a deflection.
/// </summary>
/// <remarks>A mutable container, as in OCCT. <c>DumpJson</c> is left out.</remarks>
public class Poly_Polygon2D : Standard_Transient
{
    private double myDeflection;
    private readonly NCollection_Array1<gp_Pnt2d> myNodes;

    /// <summary>A polygon of <paramref name="theNbNodes"/> nodes at the origin.</summary>
    public Poly_Polygon2D(int theNbNodes)
    {
        myDeflection = 0.0;
        myNodes = new NCollection_Array1<gp_Pnt2d>(1, theNbNodes);
    }

    /// <summary>A polygon of copies of the nodes, renumbered from 1.</summary>
    public Poly_Polygon2D(NCollection_Array1<gp_Pnt2d> Nodes)
    {
        myDeflection = 0.0;
        myNodes = new NCollection_Array1<gp_Pnt2d>(1, Nodes.Length());
        int i, j = 1;
        for (i = Nodes.Lower(); i <= Nodes.Upper(); i++)
        {
            myNodes[j++] = Nodes[i];
        }
    }

    /// <summary>A copy of this polygon.</summary>
    public virtual Poly_Polygon2D Copy()
    {
        var aCopy = new Poly_Polygon2D(NbNodes());
        aCopy.ChangeNodes().CopyValues(myNodes);
        aCopy.Deflection(myDeflection);
        return aCopy;
    }

    /// <summary>
    /// The deflection of this polygon: the largest distance allowed between the curve it approximates and the polygon.
    /// 0 by default, meaning a true polygon rather than an approximation.
    /// </summary>
    public double Deflection() => myDeflection;

    /// <summary>Sets the deflection of this polygon.</summary>
    public void Deflection(double theDefl) => myDeflection = theDefl;

    /// <summary>The number of nodes; a closed polygon repeats its first node, so a closed triangle has 4.</summary>
    public int NbNodes() => myNodes.Length();

    /// <summary>The nodes, from 1.</summary>
    public NCollection_Array1<gp_Pnt2d> Nodes() => myNodes;

    /// <summary>The nodes, from 1, for writing.</summary>
    public NCollection_Array1<gp_Pnt2d> ChangeNodes() => myNodes;

    public override string ToString()
        => string.Create(CultureInfo.InvariantCulture, $"Poly_Polygon2D({NbNodes()} nodes, deflection {myDeflection:R})");
}
