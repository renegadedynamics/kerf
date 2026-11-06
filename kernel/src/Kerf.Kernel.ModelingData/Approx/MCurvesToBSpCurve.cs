// Port of: src/ModelingData/TKGeomBase/Approx/Approx_MCurvesToBSpCurve.hxx
// Port of: src/ModelingData/TKGeomBase/Approx/Approx_MCurvesToBSpCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Conversion;

using Kerf.Geometry;

namespace Kerf.Engine.Approximation;

/// <summary>Joins a sequence of MultiCurves (Bezier) into one MultiBSpCurve.</summary>
/// <remarks>
/// A working object (decision 21). Ported in unit 2C, ahead of the rest of Approx, because Approx_ComputeLine's
/// <c>SplineValue</c> uses it (decision 203). <see cref="Value"/> returns the object's own result (shared).
/// </remarks>
internal class MCurvesToBSpCurve
{
    private MultiBSpCurve _spline = new();
    // OCCT's myDone is written but never read: left out (decision 143).
    private readonly NCollection_Sequence<MultiCurve> _curves = new();

    public MCurvesToBSpCurve() { }

    /// <summary>Removes the appended MultiCurves.</summary>
    public void Reset()
    {
        _curves.Clear();
    }

    /// <summary>Appends a MultiCurve (a copy sharing its poles, as OCCT's sequence copies it).</summary>
    public void Append(MultiCurve mc)
    {
        _curves.Append(new MultiCurve(mc));
    }

    /// <summary>Joins the appended MultiCurves.</summary>
    public void Perform()
    {
        Perform(_curves);
    }

    /// <summary>Joins the MultiCurves <paramref name="theSeq"/>.</summary>
    public void Perform(NCollection_Sequence<MultiCurve> theSeq)
    {
        int i, j, deg = 0;
        int nbcu = theSeq.Length();
        MultiCurve cu;
        int nbpolesspl = 0, nbknots = 0;

        if (nbcu == 1)
        {
            cu = theSeq.Value(1);
            deg = cu.Degree();
            NCollection_Array1<double> knots = new(1, 2);
            NCollection_Array1<int> mults = new(1, 2);
            knots[1] = 0.0;
            knots[2] = 1.0;
            mults[1] = mults[2] = deg + 1;
            _spline = new MultiBSpCurve(cu, knots, mults);
        }
        else
        {
            MultiPoint p = theSeq.Value(nbcu).Value(1);
            int nb3d = p.PointCount;
            int nb2d = p.Points2dCount;

            CompBezierCurvesToBSplineCurve conv = new();
            CompBezierCurves2dToBSplineCurve2d conv2d = new();

            if (nb3d != 0)
            {
                for (i = 1; i <= nbcu; i++)
                {
                    cu = theSeq.Value(i);
                    NCollection_Array1<Point3D> thePoles3d = new(1, cu.NbPoles());
                    cu.Curve(1, thePoles3d);
                    conv.AddCurve(thePoles3d);
                }
                conv.Perform();
            }
            else if (nb2d != 0)
            {
                for (i = 1; i <= nbcu; i++)
                {
                    cu = theSeq.Value(i);
                    NCollection_Array1<Point2D> thePoles2d = new(1, cu.NbPoles());
                    cu.Curve(1 + nb3d, thePoles2d);
                    conv2d.AddCurve(thePoles2d);
                }
                conv2d.Perform();
            }

            // Recuperation:
            if (nb3d != 0)
            {
                nbpolesspl = conv.PoleCount;
                nbknots = conv.KnotCount;
            }
            else if (nb2d != 0)
            {
                nbpolesspl = conv2d.PoleCount;
                nbknots = conv2d.KnotCount;
            }

            NCollection_Array1<MultiPoint> tabMU = new(1, nbpolesspl);
            NCollection_Array1<Point3D> polesSpl = new(1, nbpolesspl);
            NCollection_Array1<Point2D> polesSpl2d = new(1, nbpolesspl);
            NCollection_Array1<int> theMults = new(1, nbknots);
            NCollection_Array1<double> theKnots = new(1, nbknots);

            if (nb3d != 0)
            {
                conv.KnotsAndMults(theKnots, theMults);
                conv.Poles(polesSpl);
                deg = conv.Degree();
            }
            else if (nb2d != 0)
            {
                conv2d.KnotsAndMults(theKnots, theMults);
                conv2d.Poles(polesSpl2d);
                deg = conv2d.Degree();
            }

            for (j = 1; j <= nbpolesspl; j++)
            {
                MultiPoint mp = new(nb3d, nb2d);
                if (nb3d != 0)
                {
                    mp.SetPoint(1, polesSpl[j]);
                }
                else if (nb2d != 0)
                {
                    mp.SetPoint2d(1 + nb3d, polesSpl2d[j]);
                }
                tabMU.SetValue(j, mp);
            }

            int kpol = 1, kpoles3d = 1, kpoles2d = 1;
            int mydegre, k;
            int first, last, inc, thefirst;
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
                cu = theSeq.Value(i);
                mydegre = cu.Degree();
                if (theMults[i + 1] == deg)
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
                else if ((theMults[i] == deg - 1) || (theMults[i] == deg))
                {
                    first = 2;
                }

                for (j = 2; j <= nb3d; j++)
                {
                    kpol = kpoles3d;
                    NCollection_Array1<Point3D> thePoles = new(1, cu.NbPoles());
                    cu.Curve(j, thePoles);

                    inc = deg - mydegre;
                    NCollection_Array1<Point3D> points = new(1, deg + 1);
                    if (inc > 0)
                    {
                        global::Kerf.Engine.Geometry.Splines.BSplCLib.IncreaseDegree(deg, thePoles, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), points, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights());
                    }
                    else
                    {
                        points.Assign(thePoles);
                    }

                    for (k = first; k <= last; k++)
                    {
                        tabMU[kpol++].SetPoint(j, points[k]);
                    }
                }
                kpoles3d = kpol;

                for (j = thefirst; j <= nb2d; j++)
                {
                    kpol = kpoles2d;
                    NCollection_Array1<Point2D> thePoles2d = new(1, cu.NbPoles());
                    cu.Curve(j + nb3d, thePoles2d);

                    inc = deg - mydegre;
                    NCollection_Array1<Point2D> points2d = new(1, deg + 1);
                    if (inc > 0)
                    {
                        global::Kerf.Engine.Geometry.Splines.BSplCLib.IncreaseDegree(deg, thePoles2d, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), points2d, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights());
                    }
                    else
                    {
                        points2d.Assign(thePoles2d);
                    }
                    for (k = first; k <= last; k++)
                    {
                        tabMU[kpol++].SetPoint2d(j + nb3d, points2d[k]);
                    }
                }
                kpoles2d = kpol;
            }

            _spline = new MultiBSpCurve(tabMU, theKnots, theMults);
        }

    }

    /// <summary>The joined MultiCurves as a MultiBSpCurve: the object's own (shared).</summary>
    public MultiBSpCurve Value() => _spline;

    /// <summary>As <see cref="Value"/>.</summary>
    public MultiBSpCurve ChangeValue() => _spline;
}
