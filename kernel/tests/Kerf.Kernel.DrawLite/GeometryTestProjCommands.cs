// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Api;
using Kerf.Geometry;
using System.Globalization;
using Kerf.Engine;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// GeometryTest_TestProjCommands.cxx's <c>xdistcs</c>: the distances from points of a curve to a surface
/// (GeomAPI_ProjectPointOnSurf), as the lowalgos/intss grid's CheckIntersectionResult checks intersection lines. The
/// markers Draw displays are left out (no viewer).
/// </summary>
internal static class GeometryTestProjCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "xdistcs", XDistCS);
    }

    // printf's "%lg" (Tcl's format is C's for %g).
    private static string G(double theValue) => TclFormat.Format("%g", [theValue.ToString("R", CultureInfo.InvariantCulture)]);

    private static int XDistCS(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 6)
        {
            theDI.Output.Write("Use: xdistcs curve surface t1 t2 nbpoints [tol [warn_tol]]\n");
            theDI.Output.Write("Measures distances from curve to surface by nbpoints probing points on a curve\n");
            theDI.Output.Write("Error will be reported for points where distance is greater than tol\n");
            theDI.Output.Write("Warning will be reported for points where distance is greater than warn_tol\n");
            return 0;
        }

        double aTol = 1.0e-7;
        Curve3D? aC = DrawTrSurf.GetCurve(theDI, a[1]);
        if (aC is null)
        {
            di.Append("Error: ").Append(a[1]).Append(" is not a curve!\n");
            return 0;
        }

        Surface? aS = DrawTrSurf.GetSurface(theDI, a[2]);
        if (aS is null)
        {
            di.Append("Error: ").Append(a[2]).Append(" is not a surface!\n");
            return 0;
        }

        double aT1 = theDI.Atof(a[3]);
        double aT2 = theDI.Atof(a[4]);
        int aNbP = 10;
        if (n > 5)
        {
            aNbP = theDI.Atoi(a[5]);
        }
        double anErrTol = n > 6 ? theDI.Atof(a[6]) : global::Kerf.Engine.Foundation.Standard.RealLast();
        double aWarnTol = n > 7 ? theDI.Atof(a[7]) : global::Kerf.Engine.Foundation.Standard.RealLast();

        double dT = (aT2 - aT1) / (aNbP - 1);
        double aMaxParam = 0.0;
        double aMaxDist = 0.0;
        var aPPS = new ProjectPointOnSurface();
        for (int i = 0; i < aNbP; ++i)
        {
            double aT = aT1 + i * dT;
            if (i == aNbP - 1)
            {
                aT = aT2;
            }

            aC.D0(aT, out Point3D aP);
            aPPS.Init(aP, aS, aTol);
            if (!aPPS.IsDone)
            {
                di.Append("Error: GeomAPI_ProjectPointOnSurf failed\n");
                return 0;
            }

            double aD = aPPS.LowerDistance();
            // report error or warning if distance is greater than tolerance
            if (aD > anErrTol)
            {
                di.Append("Error in ").Append(a[1]).Append(":");
            }
            else if (aD > aWarnTol)
            {
                di.Append("Attention (critical value of tolerance) :");
            }
            di.Append(" T=" + G(aT) + "\tD=" + G(aD) + "\n");

            if (aD > aMaxDist)
            {
                aMaxParam = aT;
                aMaxDist = aD;
            }
        }

        di.Append("Max distance = ").Append(aMaxDist).Append("\n");
        di.Append("Param = ").Append(aMaxParam).Append("\n");
        return 0;
    }
}
