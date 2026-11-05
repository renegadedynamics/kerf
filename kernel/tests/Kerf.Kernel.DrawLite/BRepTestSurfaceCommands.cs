// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Kernel.DrawLite;

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

        Surface? S = DrawTrSurf.GetSurface(theDI, a[2]);
        if (S is null)
        {
            theDI.Output.WriteLine($"{a[2]} is not a surface");
            return 1;
        }

        bool mkface = a[0][2] == 'f';
        Shape res;
        bool Segment = false;
        if (!mkface && (n == 4 || n == 8))
        {
            Segment = a[n - 1] == "1";
            n--;
        }

        if (n == 3)
        {
            res = mkface ? new MakeFace(S, global::Kerf.Engine.Foundation.Precision.Confusion()).Shape()
                         : new MakeShell(S, Segment).Shape();
        }
        else if (n <= 5)
        {
            if (!mkface)
            {
                return 1;
            }

            bool orient = n == 4;
            Shape W = theDI.GetShape(a[3], ShapeType.Wire);
            if (W.IsNull)
            {
                return 1;
            }

            res = new MakeFace(S, ShapeCasts.Wire(W), orient).Shape();
        }
        else
        {
            res = mkface
                ? new MakeFace(S, theDI.Atof(a[3]), theDI.Atof(a[4]), theDI.Atof(a[5]), theDI.Atof(a[6]), global::Kerf.Engine.Foundation.Precision.Confusion()).Shape()
                : new MakeShell(S, theDI.Atof(a[3]), theDI.Atof(a[4]), theDI.Atof(a[5]), theDI.Atof(a[6]), Segment).Shape();
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

        Shape S = theDI.GetShape(a[2], ShapeType.Face);
        if (S.IsNull)
        {
            return 1;
        }

        // OCCT dereferences the surface without a test
        Surface C = Tool.Surface(ShapeCasts.Face(S), out Placement L) ?? throw new global::Kerf.Engine.Foundation.NullObjectException();
        DrawTrSurf.Set(theDI, a[1], C.Transformed(L.Transformation));
        return 0;
    }

    private static int MkPlane(DrawInterpreter theDI, DrawResult theResult, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 3)
        {
            return 1;
        }

        Shape S = theDI.GetShape(a[2], ShapeType.Wire);
        if (S.IsNull)
        {
            return 1;
        }

        bool OnlyPlane = n == 4 && a[3] == "1";
        var aMF = new MakeFace(ShapeCasts.Wire(S), OnlyPlane);
        switch (aMF.Error())
        {
            case FaceError.FaceDone:
                theDI.SetShape(a[1], aMF.Face());
                break;
            case FaceError.NoFace:
                theResult.Append("Error. mkplane has been finished with \"No Face\" status.\n");
                break;
            case FaceError.NotPlanar:
                theResult.Append("Error. mkplane has been finished with \"Not Planar\" status.\n");
                break;
            case FaceError.CurveProjectionFailed:
                theResult.Append("Error. mkplane has been finished with \"Fail in projection curve\" status.\n");
                break;
            case FaceError.ParametersOutOfRange:
                theResult.Append("Error. mkplane has been finished with \"Parameters are out of range\" status.\n");
                break;
            default:
                theResult.Append("Error. Undefined status. Please check the code.\n");
                break;
        }

        return 0;
    }

    // the pcurve itself when it is a trimmed part of a non-periodic curve, else the curve trimmed to the edge's range
    private static Curve2D Trimmed(Curve2D c, double f, double l)
    {
        double fr = c.FirstParameter, lr = c.LastParameter;
        bool IsPeriodic = c.IsPeriodic;
        if (c.GetType() == typeof(TrimmedCurve2D))
        {
            Curve2D aC = ((TrimmedCurve2D)c).BasisCurve;
            IsPeriodic = aC.IsPeriodic;
            fr = aC.FirstParameter;
            lr = aC.LastParameter;
        }

        return !IsPeriodic && (fr - f > global::Kerf.Engine.Foundation.Precision.PConfusion() || l - lr > global::Kerf.Engine.Foundation.Precision.PConfusion())
             ? c
             : new TrimmedCurve2D(c, f, l);
    }

    private static int PCurve(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> theArgs)
    {
        // -mute only stops the legend of the drawn orientations
        var a = theArgs.Where((w, i) => i == 0 || !w.Equals("-mute", StringComparison.OrdinalIgnoreCase)).ToList();
        int n = a.Count;
        if (n == 2)
        {
            // pcurves of a face
            Shape S = theDI.GetShape(a[1], ShapeType.Face);
            if (S.IsNull)
            {
                return 1;
            }

            S = S.Oriented(Orientation.Forward);
            var ex = new Explorer(S, ShapeType.Edge);
            for (int i = 1; ex.More(); ex.Next(), i++)
            {
                Curve2D? c = Tool.CurveOnSurface(ShapeCasts.Edge(ex.Current()), ShapeCasts.Face(S), out double f, out double l);
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
            Shape SE = theDI.GetShape(a[2], ShapeType.Edge);
            if (SE.IsNull)
            {
                return 1;
            }

            Shape SF = theDI.GetShape(a[3], ShapeType.Face);
            if (SF.IsNull)
            {
                return 1;
            }

            // OCCT dereferences the pcurve without a test
            Curve2D c = Tool.CurveOnSurface(ShapeCasts.Edge(SE), ShapeCasts.Face(SF), out double f, out double l)
                          ?? throw new global::Kerf.Engine.Foundation.NullObjectException();
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

        Shape sh = theDI.GetShape(a[1]);
        if (sh.IsNull)
        {
            return 1;
        }

        Shape aCopy = DrawShapeCopy.Copy(sh);
        if (n == 2)
        {
            BrepLibrary.EncodeRegularity(aCopy);
        }
        else
        {
            double Tol = theDI.Atof(a[2]);
            Tol *= Math.PI / 180.0;
            BrepLibrary.EncodeRegularity(aCopy, Tol);
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

        Shape anEdge = theDI.GetShape(argv[1], ShapeType.Edge);
        Shape aFace1 = theDI.GetShape(argv[2], ShapeType.Face);
        Shape aFace2 = argc > 3 ? theDI.GetShape(argv[3], ShapeType.Face) : aFace1;
        if (anEdge.IsNull || aFace1.IsNull || aFace2.IsNull)
        {
            theDI.Output.WriteLine("Invalid number of arguments. Should be: getedgeregularity edge face1 [face2]");
            return 1;
        }

        Continuity aRegularity = Tool.Continuity(ShapeCasts.Edge(anEdge), ShapeCasts.Face(aFace1), ShapeCasts.Face(aFace2));
        string aStrReg = "Regularity of edge : " + aRegularity switch
        {
            Continuity.G1 => "G1",
            Continuity.C1 => "C1",
            Continuity.G2 => "G2",
            Continuity.C2 => "C2",
            Continuity.C3 => "C3",
            Continuity.CN => "CN",
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

        Shape aS = theDI.GetShape(a[1]);
        if (aS.IsNull)
        {
            di.Append("the face is a null shape\n");
            return 0;
        }

        if (aS.Type != ShapeType.Face)
        {
            di.Append("not a face\n");
            return 0;
        }

        Face aFace = ShapeCasts.Face(aS);
        var aP = new Point3D();
        DrawTrSurf.GetPoint(theDI, a[2], ref aP);

        ExtAlgo anExtAlgo = ExtAlgo.ExtAlgo_Grad;
        ExtFlag anExtFlag = ExtFlag.ExtFlag_MINMAX;
        for (int i = 3; i < n; ++i)
        {
            switch (a[i].ToLowerInvariant())
            {
                case "-min":
                    anExtFlag = ExtFlag.ExtFlag_MIN;
                    break;
                case "-max":
                    anExtFlag = ExtFlag.ExtFlag_MAX;
                    break;
                case "-minmax":
                    anExtFlag = ExtFlag.ExtFlag_MINMAX;
                    break;
                case "-t":
                    anExtAlgo = ExtAlgo.ExtAlgo_Tree;
                    break;
                case "-g":
                    anExtAlgo = ExtAlgo.ExtAlgo_Grad;
                    break;
            }
        }

        Surface aSurf = Tool.Surface(aFace, out Placement aLoc)!;
        aP = aP.Transformed(aLoc.Transformation.Inverted());
        aSurf.ParameterBounds(out double aUMin, out double aUMax, out double aVMin, out double aVMax);

        var aProjPS = new ProjectPointOnSurface();
        aProjPS.Init(aSurf, aUMin, aUMax, aVMin, aVMax);
        aProjPS.SetExtremaAlgo(anExtAlgo);
        aProjPS.SetExtremaFlag(anExtFlag);
        aProjPS.Perform(aP);
        if (aProjPS.NbPoints() != 0)
        {
            double aDist = aProjPS.LowerDistance();
            aProjPS.LowerDistanceParameters(out double U, out double V);
            Point3D aPProj = aProjPS.NearestPoint().Transformed(aLoc.Transformation);
            di.Append("proj dist = ").Append(aDist).Append("\n");
            di.Append("uvproj = ").Append(U).Append(" ").Append(V).Append("\n");
            di.Append("pproj = ").Append(aPProj.X).Append(" ").Append(aPProj.Y).Append(" ").Append(aPProj.Z).Append("\n");
        }
        else if (!aProjPS.IsDone)
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
