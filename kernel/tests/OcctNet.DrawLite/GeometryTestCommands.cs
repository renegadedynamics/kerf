// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace OcctNet.DrawLite;

/// <summary>GeometryTest_ConstraintCommands.cxx's <c>gcarc</c>: segments and arcs of circles through points (GC).</summary>
internal static class GeometryTestCommands
{
    public static void Register(DrawInterpreter theInterpreter) => DrawCxx.Add(theInterpreter, "gcarc", GcArc);

    /// <summary>
    /// gcarc name seg|cir p1 p2 [p3 [p4]]. OCCT returns 1 after binding the curve, so a successful gcarc is a Tcl
    /// error with an empty message (scripts call it inside <c>catch</c>); the usage is the result of a failure, with
    /// status 0. Both are kept.
    /// </summary>
    private static int GcArc(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n >= 5)
        {
            gp_Pnt P1 = new(), P2 = new(), P3 = new(), P4 = new();
            if (a[2] == "seg")
            {
                if (DrawTrSurf.GetPoint(theDI, a[3], ref P1) && DrawTrSurf.GetPoint(theDI, a[4], ref P2))
                {
                    DrawTrSurf.Set(theDI, a[1], new GC_MakeSegment(P1, P2).Value());
                    // OCCT
                    return 1;
                }
            }
            else if (a[2] == "cir")
            {
                // OCCT reads a[5] without checking the count: Tcl's null argv[argc], no point
                if (DrawTrSurf.GetPoint(theDI, a[3], ref P1) && DrawTrSurf.GetPoint(theDI, a[4], ref P2)
                    && DrawTrSurf.GetPoint(theDI, n > 5 ? a[5] : "", ref P3))
                {
                    if (n > 6)
                    {
                        DrawTrSurf.GetPoint(theDI, a[6], ref P4);
                        var V1 = new gp_Vec(P2, P3);
                        DrawTrSurf.Set(theDI, a[1], new GC_MakeArcOfCircle(P1, V1, P4).Value());
                        // OCCT
                        return 1;
                    }

                    DrawTrSurf.Set(theDI, a[1], new GC_MakeArcOfCircle(P1, P2, P3).Value());
                    // OCCT
                    return 1;
                }
            }
        }

        di.Append("give a name for arc and the type seg or cir then\n");
        di.Append("give passing points p1 p2 for seg    p1 p2 p3 or p1 p2 p3 p4 for cir (p2 p3 is a tgtvec)!\n");
        return 0;
    }
}
