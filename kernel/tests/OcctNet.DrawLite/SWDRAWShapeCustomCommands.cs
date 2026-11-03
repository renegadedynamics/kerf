// Port of: src/Draw/TKTopTest/SWDRAW/SWDRAW_ShapeCustom.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.Continuity;

namespace OcctNet.DrawLite;

/// <summary>
/// The commands of SWDRAW_ShapeCustom.cxx (unit 7C): <c>directfaces</c>, <c>expshape</c>, <c>scaleshape</c>,
/// <c>bsplres</c> and <c>convtorevol</c>.
/// </summary>
/// <remarks>
/// ShapeCustom's functions never write into the shape they are given (decision 973), and return it when nothing
/// changes, so <c>result == Shape</c> is OCCT's test. <c>bsplres</c> then runs ShapeFix::SameParameter on its result in
/// place, which shares the input's unchanged sub-shapes: Draw-lite runs it through
/// <see cref="SWDRAWShapeFixCommands.InPlace"/> (decision 960), so every variable sharing a changed sub-shape is
/// rebound, as DRAWEXE's variables see the change.
/// </remarks>
internal static class SWDRAWShapeCustomCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "directfaces", directfaces);
        DrawCxx.Add(theInterpreter, "expshape", expshape);
        DrawCxx.Add(theInterpreter, "scaleshape", scaleshape);
        DrawCxx.Add(theInterpreter, "bsplres", BSplRes);
        DrawCxx.Add(theInterpreter, "convtorevol", convtorevol);
    }

    private static int ContToInteger(Continuity Cont)
    {
        int result;
        switch (Cont)
        {
            case C0:
            case G1:
                result = 0;
                break;
            case C1:
            case G2:
                result = 1;
                break;
            case C2:
                result = 2;
                break;
            case C3:
                result = 3;
                break;
            default:
                result = 4;
                break;
        }
        return result;
    }

    private static int directfaces(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count < 3)
        {
            di.Append("Donner un nom de SHAPE + un nom de RESULTAT\n");
            return 1 /* Error */;
        }
        string arg1 = argv[1];
        string arg2 = argv[2];
        TopoDS_Shape Shape = theDI.GetShape(arg2);
        if (Shape.IsNull())
        {
            di.Append("Shape unknown : ").Append(arg2).Append("\n");
            return 1 /* Error */;
        }

        TopoDS_Shape result = ShapeCustom.DirectFaces(Shape);
        if (result.IsNull())
        {
            di.Append("NO RESULT\n");
            return 1;
        }
        else if (result == Shape)
        {
            di.Append("No modif\n");
            return 0;
        }
        di.Append("DirectFaces -> Result : ").Append(arg1).Append("\n");
        theDI.SetShape(arg1, result);
        return 0; // Done
    }

    // theKnots is 0-based here (OCCT's array is 1-based: theKnots(i) is theKnots[i - 1])
    private static int ckeckKnots(IReadOnlyList<double> theKnots, double theFirst, double theLast)
    {
        int i = 1, nb = theKnots.Count;
        int aNum = 0;
        for (; i < nb; i++)
        {
            if (aNum == 0)
            {
                double aDF = theKnots[i - 1] - theFirst;
                if (Math.Abs(aDF) <= global::Kerf.Engine.Foundation.Precision.PConfusion() || aDF > global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    aNum++;
                    continue;
                }
            }
            else
            {
                double aDL = theKnots[i - 1] - theLast;
                if (Math.Abs(aDL) <= global::Kerf.Engine.Foundation.Precision.PConfusion() || aDL > global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    break;
                }
                aNum++;
            }
        }
        return aNum;
    }

    private static void expcurv2d(Geom2d_Curve? aCurve, NCollection_Array2<int> Nb, int Degree, int MaxSeg, int theCont,
                                  double theFirst, double theLast)
    {
        if (aCurve is null)
        {
            return;
        }

        if (aCurve is Geom2d_TrimmedCurve aTrimmed)
        {
            expcurv2d(aTrimmed.BasisCurve(), Nb, Degree, MaxSeg, theCont, theFirst, theLast);
            return;
        }
        if (aCurve is Geom2d_OffsetCurve anOffset)
        {
            expcurv2d(anOffset.BasisCurve(), Nb, Degree, MaxSeg, theCont, theFirst, theLast);
            return;
        }
        int aCont = ContToInteger(aCurve.Continuity());
        if (aCurve is Geom2d_BSplineCurve Bs)
        {
            if (Bs.Degree() > Degree)
            {
                Nb[1, 1]++;
            }
            if (Bs.NbKnots() - 1 > MaxSeg)
            {
                Nb[1, 2]++;
            }
            if (Bs.IsRational())
            {
                Nb[1, 3]++;
            }
            if (aCont < theCont && Bs.NbKnots() > 2)
            {
                int nbInt = ckeckKnots(Bs.Knots(), theFirst, theLast);
                if (nbInt > 1)
                {
                    Nb[1, 4]++;
                }
            }
            return;
        }
        if (aCurve is Geom2d_BezierCurve Bz)
        {
            if (Bz.Degree() > Degree)
            {
                Nb[2, 1]++;
            }
            if (Bz.IsRational())
            {
                Nb[2, 3]++;
            }
            if (aCont < theCont)
            {
                Nb[2, 4]++;
            }
            return;
        }
        else
        {
            Nb[1, 5]++;
        }
    }

    private static void expcurv(Geom_Curve? aCurve, NCollection_Array2<int> Nb, int Degree, int MaxSeg, int theCont,
                                double theFirst, double theLast)
    {
        if (aCurve is null)
        {
            return;
        }
        if (aCurve is Geom_TrimmedCurve aTrimmed)
        {
            expcurv(aTrimmed.BasisCurve(), Nb, Degree, MaxSeg, theCont, theFirst, theLast);
            return;
        }
        if (aCurve is Geom_OffsetCurve anOffset)
        {
            expcurv(anOffset.BasisCurve(), Nb, Degree, MaxSeg, theCont, theFirst, theLast);
            return;
        }

        int aCont = ContToInteger(aCurve.Continuity());
        if (aCurve is Geom_BSplineCurve Bs)
        {
            if (Bs.Degree() > Degree)
            {
                Nb[1, 1]++;
            }
            if (Bs.NbKnots() - 1 > MaxSeg)
            {
                Nb[1, 2]++;
            }
            if (Bs.IsRational())
            {
                Nb[1, 3]++;
            }
            if (aCont < theCont && Bs.NbKnots() > 2)
            {
                int nbInt = ckeckKnots(Bs.Knots(), theFirst, theLast);
                if (nbInt > 1)
                {
                    Nb[1, 4]++;
                }
            }
            return;
        }
        if (aCurve is Geom_BezierCurve Bz)
        {
            if (Bz.Degree() > Degree)
            {
                Nb[2, 1]++;
            }
            if (Bz.IsRational())
            {
                Nb[2, 3]++;
            }
            if (aCont < theCont)
            {
                Nb[2, 4]++;
            }
            return;
        }
        else
        {
            Nb[1, 5]++;
        }
    }

    private static void expsurf(Geom_Surface? aSurface, NCollection_Array2<int> NbSurf, int Degree, int MaxSeg, int theCont)
    {
        if (aSurface is null)
        {
            return;
        }
        if (aSurface is Geom_SweptSurface aSwept)
        {
            Geom_Curve BasCurve = aSwept.BasisCurve();
            expcurv(BasCurve, NbSurf, Degree, MaxSeg, theCont, BasCurve.FirstParameter(), BasCurve.LastParameter());
            return;
        }
        if (aSurface is Geom_RectangularTrimmedSurface aTrimmed)
        {
            expsurf(aTrimmed.BasisSurface(), NbSurf, Degree, MaxSeg, theCont);
            return;
        }
        if (aSurface is Geom_OffsetSurface anOffset)
        {
            expsurf(anOffset.BasisSurface(), NbSurf, Degree, MaxSeg, theCont);
            return;
        }
        int aCont = ContToInteger(aSurface.Continuity());

        if (aSurface is Geom_BSplineSurface BS)
        {
            if (BS.UDegree() > Degree || BS.VDegree() > Degree)
            {
                NbSurf[1, 1]++;
            }
            if (((BS.NbUKnots() - 1) * (BS.NbVKnots() - 1)) > MaxSeg)
            {
                NbSurf[1, 2]++;
            }
            if (BS.IsURational() || BS.IsVRational())
            {
                NbSurf[1, 3]++;
            }
            if (aCont < theCont)
            {
                NbSurf[1, 4]++;
            }
            return;
        }
        if (aSurface is Geom_BezierSurface BZ)
        {
            if (BZ.UDegree() > Degree || BZ.VDegree() > Degree)
            {
                NbSurf[2, 1]++;
            }
            if (BZ.IsURational() || BZ.IsVRational())
            {
                NbSurf[2, 3]++;
            }
            if (aCont < theCont)
            {
                NbSurf[2, 4]++;
            }
            return;
        }
        if (aSurface is Geom_Plane)
        {
            NbSurf[1, 5]++;
        }
        else
        {
            NbSurf[2, 5]++;
        }
    }

    // strcmp of the continuity names expshape and bsplres accept
    private static bool ParseCont(string theArg, out Continuity theCont)
    {
        switch (theArg)
        {
            case "C0": theCont = C0; return true;
            case "C1": theCont = C1; return true;
            case "C2": theCont = C2; return true;
            case "C3": theCont = C3; return true;
            case "CN": theCont = CN; return true;
            case "G1": theCont = C0; return true;
            case "G2": theCont = C1; return true;
            default: theCont = C0; return false;
        }
    }

    private static int expshape(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        int argc = argv.Count;
        if (argc < 4)
        {
            di.Append("Incorrect number of arguments. Must be 3\n");
            return 1 /* Error */;
        }
        string arg2 = argv[1];
        TopoDS_Shape Shape = theDI.GetShape(arg2);
        int Degree = theDI.Atoi(argv[2]);
        int MaxSeg = theDI.Atoi(argv[3]);
        Continuity aCont3 = C0;
        int k = 4;
        if (argc > k)
        {
            if (!ParseCont(argv[k], out aCont3))
            {
                di.Append("Invalid argument Cont3e\n");
                return 1;
            }
        }

        int aCont = ContToInteger(aCont3);
        var NbSurf = new NCollection_Array2<int>(1, 2, 1, 5);
        var NbCurv = new NCollection_Array2<int>(1, 2, 1, 5);
        var NbCurv2d = new NCollection_Array2<int>(1, 2, 1, 5);
        int nbSeam = 0;
        NbSurf.Init(0);
        NbCurv.Init(0);
        NbCurv2d.Init(0);
        if (Shape.IsNull())
        {
            di.Append("Shape unknown: ").Append(arg2).Append("\n");
            return 1 /* Error */;
        }

        for (var Ex = new TopExp_Explorer(Shape, TopAbs_ShapeEnum.TopAbs_FACE); Ex.More(); Ex.Next())
        {
            TopoDS_Face F = TopoDS.Face(Ex.Current());
            Geom_Surface? aSurface = BRep_Tool.Surface(F, out Placement L);
            expsurf(aSurface, NbSurf, Degree, MaxSeg, aCont);

            for (var exp = new TopExp_Explorer(F, TopAbs_ShapeEnum.TopAbs_EDGE); exp.More(); exp.Next())
            {
                TopoDS_Edge E = TopoDS.Edge(exp.Current());
                if (BRep_Tool.IsClosed(E, F))
                {
                    nbSeam++;
                }
                Geom_Curve? aCurve = BRep_Tool.Curve(E, out L, out double First, out double Last);
                expcurv(aCurve, NbCurv, Degree, MaxSeg, aCont, First, Last);
                Geom2d_Curve? aCurve2d = BRep_Tool.CurveOnSurface(E, F, out double First2d, out double Last2d);
                expcurv2d(aCurve2d, NbCurv2d, Degree, MaxSeg, aCont, First2d, Last2d);
            }
        }
        di.Append("Number of seam edges - ").Append(nbSeam).Append("\n");
        di.Append("Number of BSpline surfaces with degree more then ").Append(Degree).Append(" - ").Append(NbSurf[1, 1]).Append("\n");
        di.Append("Number of BSpline surfaces with number of spans more then ").Append(MaxSeg).Append(" - ").Append(NbSurf[1, 2]).Append("\n");
        di.Append("Number of Rational BSpline surfaces ").Append(NbSurf[1, 3]).Append("\n");
        di.Append("Number of BSpline surfaces with continuity less than specified - ").Append(NbSurf[1, 4]).Append("\n");
        di.Append("Number of Bezier surfaces with degree more then ").Append(Degree).Append(" - ").Append(NbSurf[2, 1]).Append("\n");
        di.Append("Number of Rational Bezier surfaces  - ").Append(NbSurf[2, 3]).Append("\n");
        di.Append("Number of Bezier surfaces with continuity less than specified - ").Append(NbSurf[2, 4]).Append("\n");

        di.Append("Number of Planes - ").Append(NbSurf[1, 5]).Append("\n");
        di.Append("Number of other surfaces - ").Append(NbSurf[2, 5]).Append("\n");

        di.Append("Number of BSpline curves with degree more then ").Append(Degree).Append(" - ").Append(NbCurv[1, 1]).Append("\n");
        di.Append("Number of BSpline curves with number of spans more then - ").Append(MaxSeg).Append(" - ").Append(NbCurv[1, 2]).Append("\n");
        di.Append("Number of Rational BSpline curves ").Append(NbCurv[1, 3]).Append("\n");
        di.Append("Number of  BSpline curves with less continuity - ").Append(NbCurv[1, 4]).Append("\n");
        di.Append("Number of Bezier curves with degree more then - ").Append(Degree).Append(" - ").Append(NbCurv[2, 1]).Append("\n");
        di.Append("Number of Rational Bezier curves  - ").Append(NbCurv[2, 3]).Append("\n");
        di.Append("Number of  Bezier curves with less continuity - ").Append(NbCurv[2, 4]).Append("\n");

        di.Append("Number of  other curves - ").Append(NbCurv[1, 5]).Append("\n");

        di.Append("Number of BSpline pcurves with degree more then - ").Append(Degree).Append(" - ").Append(NbCurv2d[1, 1]).Append("\n");
        di.Append("Number of BSpline pcurves with number of spans more then ").Append(MaxSeg).Append(" - ").Append(NbCurv2d[1, 2]).Append("\n");
        di.Append("Number of Rational BSpline pcurves - ").Append(NbCurv2d[1, 3]).Append("\n");
        di.Append("Number of  BSpline pcurves with less continuity - ").Append(NbCurv2d[1, 4]).Append("\n");
        di.Append("Number of Bezier pcurves with degree more then ").Append(Degree).Append(" -  - ").Append(NbCurv2d[2, 1]).Append("\n");
        di.Append("Number of Rational Bezier pcurves  - ").Append(NbCurv2d[2, 3]).Append("\n");
        di.Append("Number of  Bezier pcurves with less continuity - ").Append(NbCurv2d[2, 4]).Append("\n");

        di.Append("Number of  other pcurves - ").Append(NbCurv2d[1, 5]).Append("\n");
        return 0;
    }

    private static int scaleshape(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count != 4)
        {
            di.Append("Incorrect number of arguments. Must be 4\n");
            return 1 /* Error */;
        }
        string arg2 = argv[2];
        TopoDS_Shape Shape = theDI.GetShape(arg2);
        if (Shape.IsNull())
        {
            di.Append("Shape unknown: ").Append(arg2).Append("\n");
            return 1 /* Error */;
        }

        TopoDS_Shape result = ShapeCustom.ScaleShape(Shape, theDI.Atof(argv[3]));
        if (result.IsNull())
        {
            di.Append("NO RESULT\n");
            return 1;
        }
        else if (result == Shape)
        {
            di.Append("NO MODIFICATIONS\n");
            return 0;
        }
        theDI.SetShape(argv[1], result);
        return 0;
    }

    private static int BSplRes(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count < 11)
        {
            di.Append("Incorrect number of arguments. Must be 10\n");
            return 1 /* Error */;
        }
        string arg2 = argv[2];
        TopoDS_Shape Shape = theDI.GetShape(arg2);
        if (Shape.IsNull())
        {
            di.Append("Shape unknown: ").Append(arg2).Append("\n");
            return 1 /* Error */;
        }
        if (!ParseCont(argv[7], out Continuity aCont3))
        {
            di.Append("Invalid argument Cont3e\n");
            return 1;
        }
        if (!ParseCont(argv[8], out Continuity aCont2))
        {
            di.Append("Invalid argument Cont3e\n");
            return 1;
        }

        var aParameters = new ShapeCustom_RestrictionParameters();
        TopoDS_Shape result = ShapeCustom.BSplineRestriction(Shape, theDI.Atof(argv[3]), theDI.Atof(argv[4]),
                                                             theDI.Atoi(argv[5]), theDI.Atoi(argv[6]), aCont3, aCont2,
                                                             theDI.Atoi(argv[9]) != 0, theDI.Atoi(argv[10]) != 0, aParameters);
        if (result.IsNull())
        {
            di.Append("NO RESULT\n");
            return 1;
        }
        else if (result == Shape)
        {
            di.Append("NO MODIFICATIONS\n");
            theDI.SetShape(argv[1], result);
            return 0;
        }
        // OCCT changes the result in place, which shares the input's unchanged sub-shapes (see the class)
        result = SWDrawShapeUpgradeCommands.SameParameterInPlace(theDI, result);

        theDI.SetShape(argv[1], result);
        return 0;
    }

    private static int convtorevol(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count < 3)
        {
            di.Append("Convert surfaces to revolution: convtorevol result shape\n");
            return 1;
        }

        string arg1 = argv[1];
        string arg2 = argv[2];
        TopoDS_Shape Shape = theDI.GetShape(arg2);
        if (Shape.IsNull())
        {
            di.Append("Shape unknown : ").Append(arg2).Append("\n");
            return 1;
        }

        TopoDS_Shape result = ShapeCustom.ConvertToRevolution(Shape);
        if (result.IsNull())
        {
            di.Append("NO RESULT\n");
            return 1;
        }
        else if (result == Shape)
        {
            di.Append("No modif\n");
            return 0;
        }
        di.Append("ConvertToRevolution -> Result : ").Append(arg1).Append("\n");
        theDI.SetShape(arg1, result);
        return 0; // Done
    }
}
