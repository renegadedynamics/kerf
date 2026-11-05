// Port of: src/ModelingAlgorithms/TKGeomAlgo/Hatch/Hatch_Line.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Hatch/Hatch_Line.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Classification.Hatching.LineForm;

namespace Kerf.Engine.Classification.Hatching;

/// <summary>
/// A line of the <see cref="Hatcher"/>: its geometry, its form, and the sorted list of its intersections.
/// </summary>
/// <remarks>
/// A working object (decisions 21 and 663): a class, because it holds its intersections, which
/// <see cref="AddIntersection"/> changes in place. Its fields are internal, as OCCT's friend Hatch_Hatcher reads them.
/// </remarks>
internal sealed class HatchingLine
{
    internal Line2D _lin;
    internal LineForm _form;
    internal readonly NCollection_Sequence<Parameter> _inters = new();

    /// <summary>An empty line of any form.</summary>
    public HatchingLine()
    {
        _lin = new Line2D();
        _form = LineForm.Anyline;
    }

    /// <summary>The line <paramref name="l"/> of form <paramref name="t"/>.</summary>
    public HatchingLine(in Line2D l, LineForm t)
    {
        _lin = l;
        _form = t;
    }

    /// <summary>
    /// Inserts a new intersection in the sorted list; an intersection closer than <paramref name="toler"/> to an
    /// existing one removes that one instead.
    /// </summary>
    public void AddIntersection(double par1, bool start, int index, double par2, double toler)
    {
        Parameter p = new(par1, start, index, par2);
        int i;
        for (i = 1; i <= _inters.Length(); i++)
        {
            double dfIntPar1 = _inters[i]._par1;
            // akm OCC109 vvv : Two intersections too close
            if (Math.Abs(par1 - dfIntPar1) < toler)
            {
                _inters.Remove(i);
                return;
            }
            // akm OCC109 ^^^
            if (par1 < dfIntPar1)
            {
                _inters.InsertBefore(i, p);
                return;
            }
        }
        _inters.Append(p);
    }
}
