// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace OcctNet.DrawLite;

/// <summary>
/// SWDRAW_ShapeFix.cxx's <c>checkoverlapedges</c> (ShapeAnalysis_Edge::CheckOverlapping), which the lowalgos/intss
/// grid's CheckOverlapIntCurves runs on every pair of intersection lines.
/// </summary>
internal static class SWDRAWShapeFixCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "checkoverlapedges", CheckOverlapEdges);
    }

    private static int CheckOverlapEdges(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 3)
        {
            di.Append("Invalid number of arguments\n");
            return 1;
        }
        TopoDS_Shape Sh1 = theDI.GetShape(a[1]);
        TopoDS_Shape Sh2 = theDI.GetShape(a[2]);
        if (Sh1.IsNull() || Sh2.IsNull())
        {
            di.Append("Invalid arguments\n");
            return 1;
        }
        TopoDS_Edge e1 = TopoDS.Edge(Sh1);
        TopoDS_Edge e2 = TopoDS.Edge(Sh2);
        if (e1.IsNull() || e2.IsNull())
        {
            di.Append("Invalid type of arguments\n");
            return 1;
        }

        if (BRep_Tool.Degenerated(e1))
        {
            di.Append(a[1]).Append(" is degenerated\n");
            return 1;
        }

        if (BRep_Tool.Degenerated(e2))
        {
            di.Append(a[2]).Append(" is degenerated\n");
            return 1;
        }

        double aTol = Precision.Confusion();
        double aDistDomain = 0.0;
        int k = 3;
        if (k < n)
        {
            aTol = theDI.Atof(a[k++]);
        }
        if (k < n)
        {
            aDistDomain = theDI.Atof(a[k++]);
        }

        var sae = new ShapeAnalysis_Edge();
        if (sae.CheckOverlapping(e1, e2, ref aTol, aDistDomain))
        {
            if (aDistDomain == 0.0)
            {
                di.Append("Edges are overlapping completely\n");
            }
            else
            {
                di.Append("Edges are overlapped\n");
                di.Append("with tolerance = ").Append(aTol).Append("\n");
                di.Append("on segment length = ").Append(aDistDomain).Append("\n");
            }
        }
        else
        {
            di.Append("Edges are not overlapped\n");
        }
        return 0;
    }
}
