// Port of: src/ModelingData/TKGeomBase/Approx/Approx_MCurvesToBSpCurve.hxx
// Port of: src/ModelingData/TKGeomBase/Approx/Approx_MCurvesToBSpCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Joins a sequence of MultiCurves (Bezier) into one MultiBSpCurve.</summary>
/// <remarks>
/// A working object (decision 21). Ported in unit 2C, ahead of the rest of Approx, because Approx_ComputeLine's
/// <c>SplineValue</c> uses it (decision 203). <see cref="Value"/> returns the object's own result (shared).
/// </remarks>
public class Approx_MCurvesToBSpCurve
{
    private AppParCurves_MultiBSpCurve mySpline = new();
    // OCCT's myDone is written but never read: left out (decision 143).
    private readonly NCollection_Sequence<AppParCurves_MultiCurve> myCurves = new();

    public Approx_MCurvesToBSpCurve() { }

    /// <summary>Removes the appended MultiCurves.</summary>
    public void Reset()
    {
        myCurves.Clear();
    }

    /// <summary>Appends a MultiCurve (a copy sharing its poles, as OCCT's sequence copies it).</summary>
    public void Append(AppParCurves_MultiCurve MC)
    {
        myCurves.Append(new AppParCurves_MultiCurve(MC));
    }

    /// <summary>Joins the appended MultiCurves.</summary>
    public void Perform()
    {
        Perform(myCurves);
    }

    /// <summary>Joins the MultiCurves <paramref name="TheSeq"/>.</summary>
    public void Perform(NCollection_Sequence<AppParCurves_MultiCurve> TheSeq)
    {
        int i, j, deg = 0;
        int nbcu = TheSeq.Length();
        AppParCurves_MultiCurve CU;
        int nbpolesspl = 0, nbknots = 0;

        if (nbcu == 1)
        {
            CU = TheSeq.Value(1);
            deg = CU.Degree();
            NCollection_Array1<double> Knots = new(1, 2);
            NCollection_Array1<int> Mults = new(1, 2);
            Knots[1] = 0.0;
            Knots[2] = 1.0;
            Mults[1] = Mults[2] = deg + 1;
            mySpline = new AppParCurves_MultiBSpCurve(CU, Knots, Mults);
        }
        else
        {
            AppParCurves_MultiPoint P = TheSeq.Value(nbcu).Value(1);
            int nb3d = P.NbPoints();
            int nb2d = P.NbPoints2d();

            Convert_CompBezierCurvesToBSplineCurve conv = new();
            Convert_CompBezierCurves2dToBSplineCurve2d conv2d = new();

            if (nb3d != 0)
            {
                for (i = 1; i <= nbcu; i++)
                {
                    CU = TheSeq.Value(i);
                    NCollection_Array1<gp_Pnt> ThePoles3d = new(1, CU.NbPoles());
                    CU.Curve(1, ThePoles3d);
                    conv.AddCurve(ThePoles3d);
                }
                conv.Perform();
            }
            else if (nb2d != 0)
            {
                for (i = 1; i <= nbcu; i++)
                {
                    CU = TheSeq.Value(i);
                    NCollection_Array1<gp_Pnt2d> ThePoles2d = new(1, CU.NbPoles());
                    CU.Curve(1 + nb3d, ThePoles2d);
                    conv2d.AddCurve(ThePoles2d);
                }
                conv2d.Perform();
            }

            // Recuperation:
            if (nb3d != 0)
            {
                nbpolesspl = conv.NbPoles();
                nbknots = conv.NbKnots();
            }
            else if (nb2d != 0)
            {
                nbpolesspl = conv2d.NbPoles();
                nbknots = conv2d.NbKnots();
            }

            NCollection_Array1<AppParCurves_MultiPoint> tabMU = new(1, nbpolesspl);
            NCollection_Array1<gp_Pnt> PolesSpl = new(1, nbpolesspl);
            NCollection_Array1<gp_Pnt2d> PolesSpl2d = new(1, nbpolesspl);
            NCollection_Array1<int> TheMults = new(1, nbknots);
            NCollection_Array1<double> TheKnots = new(1, nbknots);

            if (nb3d != 0)
            {
                conv.KnotsAndMults(TheKnots, TheMults);
                conv.Poles(PolesSpl);
                deg = conv.Degree();
            }
            else if (nb2d != 0)
            {
                conv2d.KnotsAndMults(TheKnots, TheMults);
                conv2d.Poles(PolesSpl2d);
                deg = conv2d.Degree();
            }

            for (j = 1; j <= nbpolesspl; j++)
            {
                AppParCurves_MultiPoint MP = new(nb3d, nb2d);
                if (nb3d != 0)
                {
                    MP.SetPoint(1, PolesSpl[j]);
                }
                else if (nb2d != 0)
                {
                    MP.SetPoint2d(1 + nb3d, PolesSpl2d[j]);
                }
                tabMU.SetValue(j, MP);
            }

            int kpol = 1, kpoles3d = 1, kpoles2d = 1;
            int mydegre, k;
            int first, last, Inc, thefirst;
            if (nb3d != 0)
            {
                thefirst = 1;
            }
            else
            {
                thefirst = 2;
            }

            for (i = 1; i <= nbcu; i++)
            {
                CU = TheSeq.Value(i);
                mydegre = CU.Degree();
                if (TheMults[i + 1] == deg)
                {
                    last = deg + 1; // Continuite C0
                }
                else
                {
                    last = deg; // Continuite C1
                }
                if (i == nbcu)
                {
                    last = deg + 1;
                }
                first = 1;
                if (i == 1)
                {
                    first = 1;
                }
                else if ((TheMults[i] == deg - 1) || (TheMults[i] == deg))
                {
                    first = 2;
                }

                for (j = 2; j <= nb3d; j++)
                {
                    kpol = kpoles3d;
                    NCollection_Array1<gp_Pnt> ThePoles = new(1, CU.NbPoles());
                    CU.Curve(j, ThePoles);

                    Inc = deg - mydegre;
                    NCollection_Array1<gp_Pnt> Points = new(1, deg + 1);
                    if (Inc > 0)
                    {
                        BSplCLib.IncreaseDegree(deg, ThePoles, BSplCLib.NoWeights(), Points, BSplCLib.NoWeights());
                    }
                    else
                    {
                        Points.Assign(ThePoles);
                    }

                    for (k = first; k <= last; k++)
                    {
                        tabMU[kpol++].SetPoint(j, Points[k]);
                    }
                }
                kpoles3d = kpol;

                for (j = thefirst; j <= nb2d; j++)
                {
                    kpol = kpoles2d;
                    NCollection_Array1<gp_Pnt2d> ThePoles2d = new(1, CU.NbPoles());
                    CU.Curve(j + nb3d, ThePoles2d);

                    Inc = deg - mydegre;
                    NCollection_Array1<gp_Pnt2d> Points2d = new(1, deg + 1);
                    if (Inc > 0)
                    {
                        BSplCLib.IncreaseDegree(deg, ThePoles2d, BSplCLib.NoWeights(), Points2d, BSplCLib.NoWeights());
                    }
                    else
                    {
                        Points2d.Assign(ThePoles2d);
                    }
                    for (k = first; k <= last; k++)
                    {
                        tabMU[kpol++].SetPoint2d(j + nb3d, Points2d[k]);
                    }
                }
                kpoles2d = kpol;
            }

            mySpline = new AppParCurves_MultiBSpCurve(tabMU, TheKnots, TheMults);
        }

    }

    /// <summary>The joined MultiCurves as a MultiBSpCurve: the object's own (shared).</summary>
    public AppParCurves_MultiBSpCurve Value() => mySpline;

    /// <summary>As <see cref="Value"/>.</summary>
    public AppParCurves_MultiBSpCurve ChangeValue() => mySpline;
}
