// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace OcctNet.DrawLite;

/// <summary>
/// The commands of BRepTest_SurfaceCommands.cxx that need only BRepBuilderAPI, BRepLib and GeomAPI: mkface, mkshell,
/// mksurface, mkplane, pcurve, encoderegularity, getedgeregularity and projponf.
/// </summary>
/// <remarks>
/// Not here: quilt (BRepTools_Quilt), sewing and fastsewing (BRepBuilderAPI_Sewing, not ported), continuity
/// (BRepOffsetAPI_FindContigousEdges). encoderegularity codes the regularity in place in OCCT; the port codes it on a
/// copy of the graph (DrawShapeCopy) and rebinds every variable sharing its TShapes to the copy's (decisions 407,
/// 495, 11011).
/// </remarks>
internal static class BRepTestSurfaceCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "mkface", MkFace);
        DrawCxx.Add(theInterpreter, "mkshell", MkFace);
        DrawCxx.Add(theInterpreter, "mksurface", MkSurface);
        DrawCxx.Add(theInterpreter, "mkplane", MkPlane);
        DrawCxx.Add(theInterpreter, "pcurve", PCurve);
        DrawCxx.Add(theInterpreter, "encoderegularity", EncodeRegularity);
        DrawCxx.Add(theInterpreter, "getedgeregularity", GetEdgeRegularity);
        DrawCxx.Add(theInterpreter, "projponf", ProjPonF);
    }

    private static int MkFace(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 3)
        {
            return 1;
        }

        Geom_Surface? S = DrawTrSurf.GetSurface(theDI, a[2]);
        if (S is null)
        {
            theDI.Output.WriteLine($"{a[2]} is not a surface");
            return 1;
        }

        bool mkface = a[0][2] == 'f';
        TopoDS_Shape res;
        bool Segment = false;
        if (!mkface && (n == 4 || n == 8))
        {
            Segment = a[n - 1] == "1";
            n--;
        }

        if (n == 3)
        {
            res = mkface ? new BRepBuilderAPI_MakeFace(S, Precision.Confusion()).Shape()
                         : new BRepBuilderAPI_MakeShell(S, Segment).Shape();
        }
        else if (n <= 5)
        {
            if (!mkface)
            {
                return 1;
            }

            bool orient = n == 4;
            TopoDS_Shape W = theDI.GetShape(a[3], TopAbs_ShapeEnum.TopAbs_WIRE);
            if (W.IsNull())
            {
                return 1;
            }

            res = new BRepBuilderAPI_MakeFace(S, TopoDS.Wire(W), orient).Shape();
        }
        else
        {
            res = mkface
                ? new BRepBuilderAPI_MakeFace(S, theDI.Atof(a[3]), theDI.Atof(a[4]), theDI.Atof(a[5]), theDI.Atof(a[6]), Precision.Confusion()).Shape()
                : new BRepBuilderAPI_MakeShell(S, theDI.Atof(a[3]), theDI.Atof(a[4]), theDI.Atof(a[5]), theDI.Atof(a[6]), Segment).Shape();
        }

        theDI.SetShape(a[1], res);
        return 0;
    }

    private static int MkSurface(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count < 3)
        {
            return 1;
        }

        TopoDS_Shape S = theDI.GetShape(a[2], TopAbs_ShapeEnum.TopAbs_FACE);
        if (S.IsNull())
        {
            return 1;
        }

        // OCCT dereferences the surface without a test
        Geom_Surface C = BRep_Tool.Surface(TopoDS.Face(S), out TopLoc_Location L) ?? throw new Standard_NullObject();
        DrawTrSurf.Set(theDI, a[1], C.Transformed(L.Transformation()));
        return 0;
    }

    private static int MkPlane(DrawInterpreter theDI, DrawResult theResult, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 3)
        {
            return 1;
        }

        TopoDS_Shape S = theDI.GetShape(a[2], TopAbs_ShapeEnum.TopAbs_WIRE);
        if (S.IsNull())
        {
            return 1;
        }

        bool OnlyPlane = n == 4 && a[3] == "1";
        var aMF = new BRepBuilderAPI_MakeFace(TopoDS.Wire(S), OnlyPlane);
        switch (aMF.Error())
        {
            case BRepBuilderAPI_FaceError.BRepBuilderAPI_FaceDone:
                theDI.SetShape(a[1], aMF.Face());
                break;
            case BRepBuilderAPI_FaceError.BRepBuilderAPI_NoFace:
                theResult.Append("Error. mkplane has been finished with \"No Face\" status.\n");
                break;
            case BRepBuilderAPI_FaceError.BRepBuilderAPI_NotPlanar:
                theResult.Append("Error. mkplane has been finished with \"Not Planar\" status.\n");
                break;
            case BRepBuilderAPI_FaceError.BRepBuilderAPI_CurveProjectionFailed:
                theResult.Append("Error. mkplane has been finished with \"Fail in projection curve\" status.\n");
                break;
            case BRepBuilderAPI_FaceError.BRepBuilderAPI_ParametersOutOfRange:
                theResult.Append("Error. mkplane has been finished with \"Parameters are out of range\" status.\n");
                break;
            default:
                theResult.Append("Error. Undefined status. Please check the code.\n");
                break;
        }

        return 0;
    }

    // the pcurve itself when it is a trimmed part of a non-periodic curve, else the curve trimmed to the edge's range
    private static Geom2d_Curve Trimmed(Geom2d_Curve c, double f, double l)
    {
        double fr = c.FirstParameter(), lr = c.LastParameter();
        bool IsPeriodic = c.IsPeriodic();
        if (c.GetType() == typeof(Geom2d_TrimmedCurve))
        {
            Geom2d_Curve aC = ((Geom2d_TrimmedCurve)c).BasisCurve();
            IsPeriodic = aC.IsPeriodic();
            fr = aC.FirstParameter();
            lr = aC.LastParameter();
        }

        return !IsPeriodic && (fr - f > Precision.PConfusion() || l - lr > Precision.PConfusion())
             ? c
             : new Geom2d_TrimmedCurve(c, f, l);
    }

    private static int PCurve(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> theArgs)
    {
        // -mute only stops the legend of the drawn orientations
        var a = theArgs.Where((w, i) => i == 0 || !w.Equals("-mute", StringComparison.OrdinalIgnoreCase)).ToList();
        int n = a.Count;
        if (n == 2)
        {
            // pcurves of a face
            TopoDS_Shape S = theDI.GetShape(a[1], TopAbs_ShapeEnum.TopAbs_FACE);
            if (S.IsNull())
            {
                return 1;
            }

            S = S.Oriented(TopAbs_Orientation.TopAbs_FORWARD);
            var ex = new TopExp_Explorer(S, TopAbs_ShapeEnum.TopAbs_EDGE);
            for (int i = 1; ex.More(); ex.Next(), i++)
            {
                Geom2d_Curve? c = BRep_Tool.CurveOnSurface(TopoDS.Edge(ex.Current()), TopoDS.Face(S), out double f, out double l);
                if (c is null)
                {
                    theDI.Output.Write($"Error: Edge {i} does not have pcurve\n");
                    continue;
                }

                DrawTrSurf.Set(theDI, $"{a[1]}_{i}", Trimmed(c, f, l));
            }
        }
        else if (n >= 4)
        {
            TopoDS_Shape SE = theDI.GetShape(a[2], TopAbs_ShapeEnum.TopAbs_EDGE);
            if (SE.IsNull())
            {
                return 1;
            }

            TopoDS_Shape SF = theDI.GetShape(a[3], TopAbs_ShapeEnum.TopAbs_FACE);
            if (SF.IsNull())
            {
                return 1;
            }

            // OCCT dereferences the pcurve without a test
            Geom2d_Curve c = BRep_Tool.CurveOnSurface(TopoDS.Edge(SE), TopoDS.Face(SF), out double f, out double l)
                          ?? throw new Standard_NullObject();
            DrawTrSurf.Set(theDI, a[1], Trimmed(c, f, l));
        }
        else
        {
            return 1;
        }

        return 0;
    }

    /// <summary>encoderegularity: OCCT codes the shape in place; the port codes a copy and rebinds the variables to it.</summary>
    private static int EncodeRegularity(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 2)
        {
            return 1;
        }

        TopoDS_Shape sh = theDI.GetShape(a[1]);
        if (sh.IsNull())
        {
            return 1;
        }

        TopoDS_Shape aCopy = DrawShapeCopy.Copy(sh);
        if (n == 2)
        {
            BRepLib.EncodeRegularity(aCopy);
        }
        else
        {
            double Tol = theDI.Atof(a[2]);
            Tol *= Math.PI / 180.0;
            BRepLib.EncodeRegularity(aCopy, Tol);
        }

        // OCCT codes the TShapes in place: every variable sharing them (the edges exploded before) sees it (unit 7N3)
        DrawShapeRebind.Rebind(theDI, sh, aCopy);
        return 0;
    }

    private static int GetEdgeRegularity(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        int argc = argv.Count;
        if (argc < 3)
        {
            theDI.Output.WriteLine("Invalid number of arguments. Should be: checkedgeregularity edge face1 [face2]");
            return 1;
        }

        TopoDS_Shape anEdge = theDI.GetShape(argv[1], TopAbs_ShapeEnum.TopAbs_EDGE);
        TopoDS_Shape aFace1 = theDI.GetShape(argv[2], TopAbs_ShapeEnum.TopAbs_FACE);
        TopoDS_Shape aFace2 = argc > 3 ? theDI.GetShape(argv[3], TopAbs_ShapeEnum.TopAbs_FACE) : aFace1;
        if (anEdge.IsNull() || aFace1.IsNull() || aFace2.IsNull())
        {
            theDI.Output.WriteLine("Invalid number of arguments. Should be: getedgeregularity edge face1 [face2]");
            return 1;
        }

        GeomAbs_Shape aRegularity = BRep_Tool.Continuity(TopoDS.Edge(anEdge), TopoDS.Face(aFace1), TopoDS.Face(aFace2));
        string aStrReg = "Regularity of edge : " + aRegularity switch
        {
            GeomAbs_Shape.GeomAbs_G1 => "G1",
            GeomAbs_Shape.GeomAbs_C1 => "C1",
            GeomAbs_Shape.GeomAbs_G2 => "G2",
            GeomAbs_Shape.GeomAbs_C2 => "C2",
            GeomAbs_Shape.GeomAbs_C3 => "C3",
            GeomAbs_Shape.GeomAbs_CN => "CN",
            _ => "C0",
        };
        di.Append(aStrReg).Append("\n");
        return 0;
    }

    private static int ProjPonF(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 3 || n > 5)
        {
            di.Append("Project point on the face.\n");
            di.Append("Usage: projponf face pnt [extrema flag: -min/-max/-minmax] [extrema algo: -g(grad)/-t(tree)]\n");
            return 1;
        }

        TopoDS_Shape aS = theDI.GetShape(a[1]);
        if (aS.IsNull())
        {
            di.Append("the face is a null shape\n");
            return 0;
        }

        if (aS.ShapeType() != TopAbs_ShapeEnum.TopAbs_FACE)
        {
            di.Append("not a face\n");
            return 0;
        }

        TopoDS_Face aFace = TopoDS.Face(aS);
        var aP = new gp_Pnt();
        DrawTrSurf.GetPoint(theDI, a[2], ref aP);

        Extrema_ExtAlgo anExtAlgo = Extrema_ExtAlgo.Extrema_ExtAlgo_Grad;
        Extrema_ExtFlag anExtFlag = Extrema_ExtFlag.Extrema_ExtFlag_MINMAX;
        for (int i = 3; i < n; ++i)
        {
            switch (a[i].ToLowerInvariant())
            {
                case "-min":
                    anExtFlag = Extrema_ExtFlag.Extrema_ExtFlag_MIN;
                    break;
                case "-max":
                    anExtFlag = Extrema_ExtFlag.Extrema_ExtFlag_MAX;
                    break;
                case "-minmax":
                    anExtFlag = Extrema_ExtFlag.Extrema_ExtFlag_MINMAX;
                    break;
                case "-t":
                    anExtAlgo = Extrema_ExtAlgo.Extrema_ExtAlgo_Tree;
                    break;
                case "-g":
                    anExtAlgo = Extrema_ExtAlgo.Extrema_ExtAlgo_Grad;
                    break;
            }
        }

        Geom_Surface aSurf = BRep_Tool.Surface(aFace, out TopLoc_Location aLoc)!;
        aP = aP.Transformed(aLoc.Transformation().Inverted());
        aSurf.Bounds(out double aUMin, out double aUMax, out double aVMin, out double aVMax);

        var aProjPS = new GeomAPI_ProjectPointOnSurf();
        aProjPS.Init(aSurf, aUMin, aUMax, aVMin, aVMax);
        aProjPS.SetExtremaAlgo(anExtAlgo);
        aProjPS.SetExtremaFlag(anExtFlag);
        aProjPS.Perform(aP);
        if (aProjPS.NbPoints() != 0)
        {
            double aDist = aProjPS.LowerDistance();
            aProjPS.LowerDistanceParameters(out double U, out double V);
            gp_Pnt aPProj = aProjPS.NearestPoint().Transformed(aLoc.Transformation());
            di.Append("proj dist = ").Append(aDist).Append("\n");
            di.Append("uvproj = ").Append(U).Append(" ").Append(V).Append("\n");
            di.Append("pproj = ").Append(aPProj.X()).Append(" ").Append(aPProj.Y()).Append(" ").Append(aPProj.Z()).Append("\n");
        }
        else if (!aProjPS.IsDone())
        {
            di.Append("projection failed\n");
        }
        else
        {
            di.Append("no projection found\n");
        }

        return 0;
    }
}
