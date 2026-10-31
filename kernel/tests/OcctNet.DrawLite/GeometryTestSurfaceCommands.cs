// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using static Occt.Convert_ParameterisationType;
using static Occt.GeomFill_FillingStyle;

namespace OcctNet.DrawLite;

/// <summary>
/// GeometryTest_SurfaceCommands.cxx's surfaces from curves (unit 5L3): <c>appsurf</c> (GeomFill_SectionGenerator and
/// GeomFill_AppSurf), <c>fillcurves</c> (GeomFill_BSplineCurves) and <c>ruled</c> (the GeomFill package's
/// <c>GeomFill::Surface</c>, unit 5W). <c>sweep</c>, <c>tuyau</c> and <c>partuyau</c> are
/// <see cref="GeometryTestPipeCommands"/>, <c>getsurfcontinuity</c> <see cref="GeometryTestContinuityCommands"/>.
/// </summary>
internal static class GeometryTestSurfaceCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "appsurf", appsurf);
        DrawCxx.Add(theInterpreter, "fillcurves", fillcurves);
        DrawCxx.Add(theInterpreter, "ruled", ruled);
    }

    private static int ruled(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 4)
        {
            return 1;
        }

        Geom_Curve? C1 = DrawTrSurf.GetCurve(theDI, a[2]);
        if (C1 is null)
        {
            di.Append(" C1 is not a Curve ==> Command failed\n");
            return 1;
        }
        Geom_Curve? C2 = DrawTrSurf.GetCurve(theDI, a[3]);
        if (C2 is null)
        {
            di.Append(" C2 is not a Curve ==> Command failed\n");
            return 1;
        }

        Geom_Surface? S = GeomFill.Surface(C1, C2);
        if (S is null)
        {
            return 1;
        }

        DrawTrSurf.Set(theDI, a[1], S);
        return 0;
    }

    private static int appsurf(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 4)
        {
            return 1;
        }

        GeomFill_SectionGenerator Section = new();
        int i;
        Geom_Curve? C;
        for (i = 2; i < n; i++)
        {
            C = DrawTrSurf.GetCurve(theDI, a[i]);
            if (C is null)
            {
                return 1;
            }
            Section.AddCurve(C);
        }

        Section.Perform(Precision.PConfusion());

        GeomFill_Line Line = new(n - 2);
        int NbIt = 0;
        GeomFill_AppSurf App = new(3, 8, Precision.Confusion(), Precision.PConfusion(), NbIt);

        App.Perform(Line, Section);

        if (!App.IsDone())
        {
            di.Append("    Approximation aux fraises \n");
        }

        App.SurfShape(out _, out _, out _, out _, out _, out _);

        Geom_BSplineSurface GBS = new(App.SurfPoles(), App.SurfWeights(), App.SurfUKnots(), App.SurfVKnots(),
                                      App.SurfUMults(), App.SurfVMults(), App.UDegree(), App.VDegree());

        DrawTrSurf.Set(theDI, a[1], GBS);
        return 0;
    }

    private static int fillcurves(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 6)
        {
            return 1;
        }

        int i;
        Geom_Curve? aC;
        Geom_BSplineCurve[] C = new Geom_BSplineCurve[4];
        for (i = 2; i < 6; i++)
        {
            aC = DrawTrSurf.GetCurve(theDI, a[i]);
            if (aC is null)
            {
                return 1;
            }
            C[i - 2] = GeomConvert.CurveToBSplineCurve(aC, Convert_RationalC1);
        }

        int ist = 2;
        GeomFill_FillingStyle Style = GeomFill_CoonsStyle;
        if (n > 6)
        {
            ist = theDI.Atoi(a[6]);
        }

        if (ist == 1)
        {
            Style = GeomFill_StretchStyle;
        }
        if (ist == 2)
        {
            Style = GeomFill_CoonsStyle;
        }
        if (ist == 3)
        {
            Style = GeomFill_CurvedStyle;
        }

        GeomFill_BSplineCurves aFilling = new(C[0], C[1], C[2], C[3], Style);

        Geom_BSplineSurface? GBS = aFilling.Surface();
        DrawTrSurf.Set(theDI, a[1], GBS);
        return 0;
    }
}
