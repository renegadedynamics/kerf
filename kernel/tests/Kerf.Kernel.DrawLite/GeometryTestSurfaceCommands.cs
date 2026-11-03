// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;
using static Kerf.Engine.Construction.Surfaces.FillingStyle;

namespace Kerf.Kernel.DrawLite;

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

        Curve3D? C1 = DrawTrSurf.GetCurve(theDI, a[2]);
        if (C1 is null)
        {
            di.Append(" C1 is not a Curve ==> Command failed\n");
            return 1;
        }
        Curve3D? C2 = DrawTrSurf.GetCurve(theDI, a[3]);
        if (C2 is null)
        {
            di.Append(" C2 is not a Curve ==> Command failed\n");
            return 1;
        }

        Surface? S = global::Kerf.Engine.Construction.Surfaces.GeomFill.Surface(C1, C2);
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

        global::Kerf.Engine.Construction.Surfaces.SectionGenerator Section = new();
        int i;
        Curve3D? C;
        for (i = 2; i < n; i++)
        {
            C = DrawTrSurf.GetCurve(theDI, a[i]);
            if (C is null)
            {
                return 1;
            }
            Section.AddCurve(C);
        }

        Section.Perform(global::Kerf.Engine.Foundation.Precision.PConfusion());

        global::Kerf.Engine.Construction.Surfaces.SurfaceLine Line = new(n - 2);
        int NbIt = 0;
        global::Kerf.Engine.Construction.Surfaces.SurfaceAppSurf App = new(3, 8, global::Kerf.Engine.Foundation.Precision.Confusion(), global::Kerf.Engine.Foundation.Precision.PConfusion(), NbIt);

        App.Perform(Line, Section);

        if (!App.IsDone)
        {
            di.Append("    Approximation aux fraises \n");
        }

        App.SurfShape(out _, out _, out _, out _, out _, out _);

        BSplineSurface GBS = new(App.SurfPoles(), App.SurfWeights(), App.SurfUKnots(), App.SurfVKnots(),
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
        Curve3D? aC;
        BSplineCurve3D[] C = new BSplineCurve3D[4];
        for (i = 2; i < 6; i++)
        {
            aC = DrawTrSurf.GetCurve(theDI, a[i]);
            if (aC is null)
            {
                return 1;
            }
            C[i - 2] = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(aC, RationalC1);
        }

        int ist = 2;
        global::Kerf.Engine.Construction.Surfaces.FillingStyle Style = global::Kerf.Engine.Construction.Surfaces.FillingStyle.CoonsStyle;
        if (n > 6)
        {
            ist = theDI.Atoi(a[6]);
        }

        if (ist == 1)
        {
            Style = global::Kerf.Engine.Construction.Surfaces.FillingStyle.StretchStyle;
        }
        if (ist == 2)
        {
            Style = global::Kerf.Engine.Construction.Surfaces.FillingStyle.CoonsStyle;
        }
        if (ist == 3)
        {
            Style = global::Kerf.Engine.Construction.Surfaces.FillingStyle.CurvedStyle;
        }

        global::Kerf.Engine.Construction.Surfaces.BSplineCurves aFilling = new(C[0], C[1], C[2], C[3], Style);

        BSplineSurface? GBS = aFilling.Surface();
        DrawTrSurf.Set(theDI, a[1], GBS);
        return 0;
    }
}
