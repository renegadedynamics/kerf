// Port of: src/ModelingAlgorithms/TKGeomAlgo/Hatch/Hatch_Line.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Hatch/Hatch_Line.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Occt.Hatch_LineForm;

namespace Occt;

/// <summary>
/// A line of the <see cref="Hatch_Hatcher"/>: its geometry, its form, and the sorted list of its intersections.
/// </summary>
/// <remarks>
/// A working object (decisions 21 and 663): a class, because it holds its intersections, which
/// <see cref="AddIntersection"/> changes in place. Its fields are internal, as OCCT's friend Hatch_Hatcher reads them.
/// </remarks>
public sealed class Hatch_Line
{
    internal Line2D myLin;
    internal Hatch_LineForm myForm;
    internal readonly NCollection_Sequence<Hatch_Parameter> myInters = new();

    /// <summary>An empty line of any form.</summary>
    public Hatch_Line()
    {
        myLin = new Line2D();
        myForm = Hatch_ANYLINE;
    }

    /// <summary>The line <paramref name="L"/> of form <paramref name="T"/>.</summary>
    public Hatch_Line(in Line2D L, Hatch_LineForm T)
    {
        myLin = L;
        myForm = T;
    }

    /// <summary>
    /// Inserts a new intersection in the sorted list; an intersection closer than <paramref name="theToler"/> to an
    /// existing one removes that one instead.
    /// </summary>
    public void AddIntersection(double Par1, bool Start, int Index, double Par2, double theToler)
    {
        Hatch_Parameter P = new(Par1, Start, Index, Par2);
        int i;
        for (i = 1; i <= myInters.Length(); i++)
        {
            double dfIntPar1 = myInters[i].myPar1;
            // akm OCC109 vvv : Two intersections too close
            if (Math.Abs(Par1 - dfIntPar1) < theToler)
            {
                myInters.Remove(i);
                return;
            }
            // akm OCC109 ^^^
            if (Par1 < dfIntPar1)
            {
                myInters.InsertBefore(i, P);
                return;
            }
        }
        myInters.Append(P);
    }
}
