// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Text;
using Occt;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.DrawLite;

/// <summary>
/// BRepTest's commands that Draw-lite has (BRepTest_BasicCommands.cxx, BRepTest_GPropCommands.cxx): reset, tmove,
/// ttranslate, trotate, tmirror, tscale and their b/f forms, tcopy, maxtolerance, purgeloc, checkloc, lprops,
/// sprops, vprops, vpropsgk; and SWDRAW's tolerance (SWDRAW_ShapeAnalysis.cxx).
/// </summary>
/// <remarks>
/// BRepBuilderAPI (unit 3K) isn't ported yet: BRepBuilderAPI_Transform and BRepBuilderAPI_Copy are mirrored here as
/// their Perform runs (BRepTools_Modifier with BRepTools_TrsfModification or BRepTools_CopyModification, or Moved for
/// a rigid transformation without copy). ShapeAnalysis (phase 7) isn't either: ShapeAnalysis_ShapeTolerance is
/// ported here as a private helper.
/// </remarks>
internal static class BRepTestCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        foreach (string aName in new[]
                 {
                     "reset", "tmove", "ttranslate", "trotate", "tmirror", "tscale", "bmove", "fmove", "btranslate",
                     "brotate", "bmirror", "fmirror", "bscale", "fscale",
                 })
        {
            DrawCxx.Add(theInterpreter, aName, Transform);
        }

        DrawCxx.Add(theInterpreter, "tcopy", TCopy);
        DrawCxx.Add(theInterpreter, "maxtolerance", MaxTolerance);
        DrawCxx.Add(theInterpreter, "purgeloc", PurgeLoc);
        DrawCxx.Add(theInterpreter, "checkloc", CheckLoc);
        DrawCxx.Add(theInterpreter, "lprops", Props);
        DrawCxx.Add(theInterpreter, "sprops", Props);
        DrawCxx.Add(theInterpreter, "vprops", Props);
        DrawCxx.Add(theInterpreter, "vpropsgk", VPropsGK);
        DrawCxx.Add(theInterpreter, "tolerance", Tolerance);
    }

    // -------------------------------------------------------------------------------------------------------------
    // transform

    private static int Transform(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n <= 1)
        {
            return 1;
        }

        gp_Trsf T = new();
        int last = n;
        string aName = a[0];
        bool isBasic = false;
        bool isForced = false;
        bool isCopy = false;
        bool isCopyMesh = false;

        // "copymesh" and "copy" flags
        if (a[n - 1] == "-copymesh")
        {
            isCopyMesh = true;
            last = --n;
        }

        if (a[n - 1] == "-copy")
        {
            isCopy = true;
            last = --n;
        }

        if (aName != "reset")
        {
            isBasic = aName[0] == 'b';
            isForced = aName[0] == 'f';
            aName = aName[1..];
            double F(int i) => theDI.Atof(a[i]);
            switch (aName)
            {
                case "move":
                {
                    if (n < 3)
                    {
                        return 1;
                    }

                    TopoDS_Shape SL = theDI.GetShape(a[n - 1]);
                    if (SL.IsNull())
                    {
                        return 0;
                    }

                    T = SL.Location().Transformation();
                    last = n - 1;
                    isBasic = true;
                    break;
                }

                case "translate":
                    if (n < 5)
                    {
                        return 1;
                    }

                    T.SetTranslation(new gp_Vec(F(n - 3), F(n - 2), F(n - 1)));
                    last = n - 3;
                    break;
                case "rotate":
                    if (n < 9)
                    {
                        return 1;
                    }

                    T.SetRotation(new gp_Ax1(new gp_Pnt(F(n - 7), F(n - 6), F(n - 5)), new gp_Dir(new gp_Vec(F(n - 4), F(n - 3), F(n - 2)))),
                                  F(n - 1) * (Math.PI / 180.0));
                    last = n - 7;
                    break;
                case "mirror":
                    if (n < 8)
                    {
                        return 1;
                    }

                    T.SetMirror(new gp_Ax2(new gp_Pnt(F(n - 6), F(n - 5), F(n - 4)), new gp_Dir(new gp_Vec(F(n - 3), F(n - 2), F(n - 1)))));
                    last = n - 6;
                    break;
                case "scale":
                    if (n < 6)
                    {
                        return 1;
                    }

                    T.SetScale(new gp_Pnt(F(n - 4), F(n - 3), F(n - 2)), F(n - 1));
                    last = n - 4;
                    break;
            }
        }

        if (T.Form() == gp_TrsfForm.gp_Identity || isBasic || isForced)
        {
            bool isException = !isForced;
            var L = new TopLoc_Location(T);
            for (int i = 1; i < last; i++)
            {
                TopoDS_Shape S = theDI.GetShape(a[i]);
                if (S.IsNull())
                {
                    theDI.Output.WriteLine($"Error: {a[i]} is not a valid shape");
                    return 1;
                }

                try
                {
                    theDI.SetShape(a[i], aName is "move" or "reset" ? S.Located(L, isException) : S.Moved(L, isException));
                }
                catch (Standard_DomainError)
                {
                    theDI.Output.WriteLine($"Operation is not done: {aName} is not a valid transformation - scale = {DrawFormat.AsciiString(T.ScaleFactor())}");
                    return 0;
                }
            }
        }
        else
        {
            for (int i = 1; i < last; i++)
            {
                TopoDS_Shape S = theDI.GetShape(a[i]);
                if (S.IsNull())
                {
                    theDI.Output.WriteLine($"Error: {a[i]} is not a valid shape");
                    return 1;
                }

                if (!TransformShape(S, T, isCopy, isCopyMesh, out TopoDS_Shape aResult))
                {
                    return 1;
                }

                theDI.SetShape(a[i], aResult);
            }
        }

        return 0;
    }

    /// <summary>
    /// BRepBuilderAPI_Transform::Perform(S, copyGeom, copyMesh): the modifier with BRepTools_TrsfModification for a
    /// copy, a negative or a scaling transformation; otherwise the shape moved.
    /// </summary>
    internal static bool TransformShape(in TopoDS_Shape theShape, in gp_Trsf theTrsf, bool theCopyGeom, bool theCopyMesh, out TopoDS_Shape theResult)
    {
        bool aUseModif = theCopyGeom || theTrsf.IsNegative()
                         || Math.Abs(Math.Abs(theTrsf.ScaleFactor()) - 1.0) > TopLoc_Location.ScalePrec();
        if (!aUseModif)
        {
            theResult = theShape.Moved(new TopLoc_Location(theTrsf));
            return true;
        }

        var aModification = new BRepTools_TrsfModification(theTrsf);
        aModification.IsCopyMesh() = theCopyMesh;
        var aModifier = new BRepTools_Modifier(theShape, aModification);
        theResult = aModifier.IsDone() ? aModifier.ModifiedShape(theShape) : new TopoDS_Shape();
        return aModifier.IsDone();
    }

    // tcopy [-n(ogeom)] [-m(esh)] shape1 copy1 [shape2 copy2 [...]]
    private static int TCopy(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        bool copyGeom = true;
        bool copyMesh = false;
        int iFirst = 1;
        if (n > 1)
        {
            for (int i = 1; i <= 2 && i < n; i++)
            {
                if (a[i].Length == 0 || a[i][0] != '-')
                {
                    break;
                }

                char c = a[i].Length > 1 ? a[i][1] : '\0';
                if (c == 'n')
                {
                    copyGeom = false;
                    iFirst++;
                }
                else if (c == 'm')
                {
                    copyMesh = true;
                    iFirst++;
                }
            }
        }

        if (n < 3 || (n - iFirst) % 2 != 0)
        {
            theDI.Output.WriteLine($"Use: {a[0]} [-n(ogeom)] [-m(esh)] shape1 copy1 [shape2 copy2 [...]]\n"
                                   + "Option -n forbids copying of geometry (it will be shared)\n"
                                   + "Option -m forces copying of mesh (disabled by default)");
            return 1;
        }

        int nbPairs = (n - iFirst) / 2;
        for (int i = 0; i < nbPairs; i++)
        {
            // OCCT bug: the pair number i indexes the words, where 2 * i was meant (occt-issues.md)
            // BRepBuilderAPI_Copy::Perform: the modifier with BRepTools_CopyModification
            TopoDS_Shape S = theDI.GetShape(a[i + iFirst]);
            if (S.IsNull())
            {
                throw new Standard_NullObject();
            }

            var aModifier = new BRepTools_Modifier(S, new BRepTools_CopyModification(copyGeom, copyMesh));
            theDI.SetShape(a[i + iFirst + 1], aModifier.IsDone() ? aModifier.ModifiedShape(S) : new TopoDS_Shape());
            di.Append(a[i + iFirst + 1]).Append(" ");
        }

        return 0;
    }

    // -------------------------------------------------------------------------------------------------------------
    // tolerances and locations

    private static int MaxTolerance(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count < 2)
        {
            return 1;
        }

        TopoDS_Shape TheShape = theDI.GetShape(a[1]);
        if (TheShape.IsNull())
        {
            return 1;
        }

        double TMF, TME, TMV, TmF, TmE, TmV;
        TMF = TME = TMV = -double.MaxValue;
        TmF = TmE = TmV = double.MaxValue;
        var mapS = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        for (var ex = new TopExp_Explorer(TheShape, TopAbs_FACE); ex.More(); ex.Next())
        {
            double T = BRep_Tool.Tolerance(TopoDS.Face(ex.Current()));
            TMF = Math.Max(TMF, T);
            TmF = Math.Min(TmF, T);
            mapS.Add(ex.Current());
        }

        int nbF = mapS.Extent();
        mapS.Clear();
        for (var ex = new TopExp_Explorer(TheShape, TopAbs_EDGE); ex.More(); ex.Next())
        {
            double T = BRep_Tool.Tolerance(TopoDS.Edge(ex.Current()));
            TME = Math.Max(TME, T);
            TmE = Math.Min(TmE, T);
            mapS.Add(ex.Current());
        }

        int nbE = mapS.Extent();
        mapS.Clear();
        for (var ex = new TopExp_Explorer(TheShape, TopAbs_VERTEX); ex.More(); ex.Next())
        {
            double T = BRep_Tool.Tolerance(TopoDS.Vertex(ex.Current()));
            TMV = Math.Max(TMV, T);
            TmV = Math.Min(TmV, T);
            mapS.Add(ex.Current());
        }

        int nbV = mapS.Extent();
        var sss = new StringBuilder();
        sss.Append(CultureInfo.InvariantCulture, $"\n## Tolerances on the shape {a[1]}  (nbFaces:{nbF}  nbEdges:{nbE} nbVtx:{nbV})\n");
        static string E(double theValue) => DrawFormat.Scientific(theValue, 5).PadLeft(8);
        if (TmF <= TMF)
        {
            sss.Append($"\n    Face   : Min {E(TmF)}    Max  {E(TMF)} \n ");
        }

        if (TmE <= TME)
        {
            sss.Append($"\n    Edge   : Min {E(TmE)}    Max  {E(TME)} \n ");
        }

        if (TmV <= TMV)
        {
            sss.Append($"\n    Vertex : Min {E(TmV)}    Max  {E(TMV)} \n ");
        }

        di.Append(sss.ToString());
        return 0;
    }

    // purgeloc res shape
    private static int PurgeLoc(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        TopoDS_Shape aShapeBase = theDI.GetShape(a.Count > 2 ? a[2] : "");
        if (aShapeBase.IsNull())
        {
            return 1;
        }

        var aRemLoc = new BRepTools_PurgeLocations();
        bool isDone = aRemLoc.Perform(aShapeBase);
        TopoDS_Shape Result = aRemLoc.GetResult();
        theDI.SetShape(a[1], Result);
        di.Append(isDone ? "All problematic locations are purged \n" : "Not all problematic locations are purged \n");
        return 0;
    }

    // checkloc shape
    private static int CheckLoc(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        TopoDS_Shape aShapeBase = theDI.GetShape(a.Count > 1 ? a[1] : "");
        if (aShapeBase.IsNull())
        {
            return 1;
        }

        var aLS = new NCollection_List<TopoDS_Shape>();
        BRepTools.CheckLocations(aShapeBase, aLS);
        if (aLS.IsEmpty())
        {
            di.Append("There are no problematic shapes").Append("\n");
            return 0;
        }

        int i = 1;
        foreach (TopoDS_Shape aShape in aLS)
        {
            string aName = a[1] + "_" + i.ToString(CultureInfo.InvariantCulture);
            theDI.SetShape(aName, aShape);
            di.Append(aName).Append(" ");
            i++;
        }

        di.Append("\n");
        return 0;
    }

    // -------------------------------------------------------------------------------------------------------------
    // global properties

    // lprops/sprops/vprops name [epsilon] [c[losed]] [x y z] [-skip] [-full] [-tri]
    private static int Props(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 2)
        {
            di.Append("Use: ").Append(a[0]).Append(" shape [epsilon] [c[losed]] [x y z] [-skip] [-full] [-tri]\n");
            di.Append("Compute properties of the shape, exact geometry (curves, surfaces) or\n");
            di.Append("some discrete data (polygons, triangulations) can be used for calculations\n");
            di.Append("The epsilon, if given, defines relative precision of computation\n");
            di.Append("The \"closed\" flag, if present, do computation only closed shells of the shape\n");
            di.Append("The centroid coordinates will be put to DRAW variables x y z (if given)\n");
            di.Append("Shared entities will be take in account only one time in the skip mode\n");
            di.Append("All values are outputted with the full precision in the full mode.\n");
            di.Append("Preferable source of geometry data are triangulations in case if it exists, if the -tri key is used.\n");
            di.Append("If epsilon is given, exact geometry (curves, surfaces) are used for calculations independently of using key -tri\n\n");
            return 1;
        }

        bool UseTriangulation = false;
        if (n >= 2 && a[n - 1] == "-tri")
        {
            UseTriangulation = true;
            --n;
        }

        bool isFullMode = false;
        if (n >= 2 && a[n - 1] == "-full")
        {
            isFullMode = true;
            --n;
        }

        bool SkipShared = false;
        if (n >= 2 && a[n - 1] == "-skip")
        {
            SkipShared = true;
            --n;
        }

        TopoDS_Shape S = theDI.GetShape(a[1]);
        if (S.IsNull())
        {
            return 0;
        }

        var G = new GProp_GProps();
        static bool StartsWithC(string theArg) => theArg.Length > 0 && theArg[0] == 'c';
        bool onlyClosed = (n > 2 && StartsWithC(a[2])) || (n > 3 && StartsWithC(a[3]));
        double eps = 1.0;
        bool witheps = false;
        if (n > 2 && !StartsWithC(a[2]) && n != 5)
        {
            eps = theDI.Atof(a[2]);
            witheps = true;
        }

        char aKind = a[0][0];
        if (witheps)
        {
            if (Math.Abs(eps) < Precision.Angular())
            {
                return 2;
            }

            if (aKind == 'l')
            {
                BRepGProp.LinearProperties(S, G, SkipShared);
            }
            else if (aKind == 's')
            {
                eps = BRepGProp.SurfaceProperties(S, G, eps, SkipShared);
            }
            else
            {
                eps = BRepGProp.VolumeProperties(S, G, eps, onlyClosed, SkipShared);
            }
        }
        else
        {
            if (aKind == 'l')
            {
                BRepGProp.LinearProperties(S, G, SkipShared, UseTriangulation);
            }
            else if (aKind == 's')
            {
                BRepGProp.SurfaceProperties(S, G, SkipShared, UseTriangulation);
            }
            else
            {
                BRepGProp.VolumeProperties(S, G, onlyClosed, SkipShared, UseTriangulation);
            }
        }

        gp_Pnt P = G.CentreOfMass();
        gp_Mat I = G.MatrixOfInertia();
        if (n >= 5)
        {
            int shift = n - 5;
            theDI.SetNumber(a[shift + 2], P.X());
            theDI.SetNumber(a[shift + 3], P.Y());
            theDI.SetNumber(a[shift + 4], P.Z());
        }

        GProp_PrincipalProps Pr = G.PrincipalProperties();
        Pr.Moments(out double Ix, out double Iy, out double Iz);

        if (!isFullMode)
        {
            static string W(double theValue) => DrawFormat.Setw(15, theValue);
            var aSStream1 = new StringBuilder();
            aSStream1.Append("\n\n");
            aSStream1.Append("Mass : ").Append(W(G.Mass())).Append("\n\n");
            if (witheps && aKind != 'l')
            {
                aSStream1.Append("Relative error of mass computation : ").Append(W(eps)).Append("\n\n");
            }

            aSStream1.Append("Center of gravity : \n");
            aSStream1.Append("X = ").Append(W(P.X())).Append('\n');
            aSStream1.Append("Y = ").Append(W(P.Y())).Append('\n');
            aSStream1.Append("Z = ").Append(W(P.Z())).Append('\n');
            aSStream1.Append('\n');
            aSStream1.Append("Matrix of Inertia : \n");
            for (int r = 1; r <= 3; r++)
            {
                aSStream1.Append(W(I.Value(r, 1))).Append(' ').Append(W(I.Value(r, 2))).Append(' ').Append(W(I.Value(r, 3))).Append('\n');
            }

            aSStream1.Append('\n');
            di.Append(aSStream1.ToString());

            var aSStream2 = new StringBuilder();
            aSStream2.Append("Moments : \n");
            aSStream2.Append("IX = ").Append(W(Ix)).Append('\n');
            aSStream2.Append("IY = ").Append(W(Iy)).Append('\n');
            aSStream2.Append("IZ = ").Append(W(Iz)).Append('\n');
            aSStream2.Append('\n');
            di.Append(aSStream2.ToString());
        }
        else
        {
            di.Append("\n\nMass : ").Append(G.Mass()).Append("\n\n");
            if (witheps && aKind != 'l')
            {
                di.Append("Relative error of mass computation : ").Append(eps).Append("\n\n");
            }

            di.Append("Center of gravity : \n");
            di.Append("X = ").Append(P.X()).Append("\n");
            di.Append("Y = ").Append(P.Y()).Append("\n");
            di.Append("Z = ").Append(P.Z()).Append("\n\n");
            di.Append("Matrix of Inertia :\n");
            for (int r = 1; r <= 3; r++)
            {
                di.Append(I.Value(r, 1)).Append("    ").Append(I.Value(r, 2)).Append("    ").Append(I.Value(r, 3)).Append(r == 3 ? "\n\n" : "\n");
            }

            di.Append("Moments :\n");
            di.Append("IX = ").Append(Ix).Append("\n");
            di.Append("IY = ").Append(Iy).Append("\n");
            di.Append("IZ = ").Append(Iz).Append("\n\n");
        }

        // Draw displays the axes of inertia (a Draw_Axis3D); there is no viewer
        return 0;
    }

    // vpropsgk name epsilon closed span mode [x y z] [-skip]
    private static int VPropsGK(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 2)
        {
            di.Append("Use: ").Append(a[0]).Append(" shape epsilon closed span mode [x y z] [-skip]\n");
            di.Append("Compute properties of the shape\n");
            di.Append("The epsilon defines relative precision of computation\n");
            di.Append("The \"closed\" flag, if equal 1, causes computation only closed shells of the shape\n");
            di.Append("The \"span\" flag, if equal 1, says that computation is performed on spans\n");
            di.Append("      This option makes effect only for BSpline surfaces.\n");
            di.Append("mode can be 0 - only volume calculations\n");
            di.Append("            1 - volume and gravity center\n");
            di.Append("            2 - volume, gravity center and matrix of inertia\n");
            di.Append("The centroid coordinates will be put to DRAW variables x y z (if given)\n\n");
            return 1;
        }

        if (n > 2 && n < 6)
        {
            di.Append("Wrong arguments\n");
            return 1;
        }

        TopoDS_Shape S = theDI.GetShape(a[1]);
        if (S.IsNull())
        {
            return 0;
        }

        var G = new GProp_GProps();
        bool SkipShared = false;
        if (n >= 2 && a[n - 1] == "-skip")
        {
            SkipShared = true;
            --n;
        }

        double eps = theDI.Atof(a[2]);
        bool onlyClosed = theDI.Atoi(a[3]) > 0;
        bool isUseSpan = theDI.Atoi(a[4]) > 0;
        int mode = theDI.Atoi(a[5]);
        bool CGFlag = mode is 1 or 3;
        bool IFlag = mode is 2 or 3;
        eps = BRepGProp.VolumePropertiesGK(S, G, eps, onlyClosed, isUseSpan, CGFlag, IFlag, SkipShared);

        const int anOutWidth = 24;
        static string W(double theValue) => Standard_Stream.FormatReal(theValue, 15).PadLeft(anOutWidth);
        di.Append("\n\n" + "Mass : " + W(G.Mass()) + "\n\n" + "Relative error of mass computation : " + W(eps) + "\n\n");
        if (CGFlag || IFlag)
        {
            gp_Pnt P = G.CentreOfMass();
            if (n > 6)
            {
                theDI.SetNumber(a[6], P.X());
            }

            if (n > 7)
            {
                theDI.SetNumber(a[7], P.Y());
            }

            if (n > 8)
            {
                theDI.SetNumber(a[8], P.Z());
            }

            var aSStream1 = new StringBuilder();
            aSStream1.Append("Center of gravity : \n");
            aSStream1.Append("X = ").Append(W(P.X())).Append('\n');
            aSStream1.Append("Y = ").Append(W(P.Y())).Append('\n');
            aSStream1.Append("Z = ").Append(W(P.Z())).Append('\n');
            aSStream1.Append('\n');
            if (IFlag)
            {
                gp_Mat I = G.MatrixOfInertia();
                aSStream1.Append("Matrix of Inertia : \n");
                for (int r = 1; r <= 3; r++)
                {
                    aSStream1.Append(W(I.Value(r, 1))).Append(' ').Append(W(I.Value(r, 2))).Append(' ').Append(W(I.Value(r, 3))).Append('\n');
                }

                aSStream1.Append('\n');
            }

            di.Append(aSStream1.ToString());
        }

        if (IFlag)
        {
            GProp_PrincipalProps Pr = G.PrincipalProperties();
            Pr.Moments(out double Ix, out double Iy, out double Iz);
            di.Append("Moments : \n" + "IX = " + W(Ix) + "\n" + "IY = " + W(Iy) + "\n" + "IZ = " + W(Iz) + "\n" + "\n" + "\n");
        }

        return 0;
    }

    // -------------------------------------------------------------------------------------------------------------
    // tolerance (SWDRAW_ShapeAnalysis.cxx) over ShapeAnalysis_ShapeTolerance

    /// <summary>ShapeAnalysis_ShapeTolerance (TKShHealing, phase 7), ported for the tolerance command.</summary>
    private sealed class ShapeTolerance
    {
        private int myNbTol;
        private readonly double[] myTols = new double[3];

        private static void AddTol(double tol, ref int nbt, ref double cmin, ref double cmoy, ref double cmax)
        {
            nbt++;
            if (nbt == 1)
            {
                cmin = cmoy = cmax = tol;
            }
            else
            {
                if (cmin > tol)
                {
                    cmin = tol;
                }

                if (cmax < tol)
                {
                    cmax = tol;
                }

                // #76 rln 11.03.99 S4135: average without weights (mult = 1)
                cmoy += tol;
            }
        }

        public void InitTolerance()
        {
            myNbTol = 0;
            myTols[1] = 0;
        }

        public void AddTolerance(in TopoDS_Shape shape, TopAbs_ShapeEnum type = TopAbs_SHAPE)
        {
            int nbt = 0;
            double cmin = 0, cmoy = 0, cmax = 0;
            if (type is TopAbs_FACE or TopAbs_SHAPE)
            {
                for (var ex = new TopExp_Explorer(shape, TopAbs_FACE); ex.More(); ex.Next())
                {
                    AddTol(BRep_Tool.Tolerance(TopoDS.Face(ex.Current())), ref nbt, ref cmin, ref cmoy, ref cmax);
                }
            }

            if (type is TopAbs_EDGE or TopAbs_SHAPE)
            {
                for (var ex = new TopExp_Explorer(shape, TopAbs_EDGE); ex.More(); ex.Next())
                {
                    AddTol(BRep_Tool.Tolerance(TopoDS.Edge(ex.Current())), ref nbt, ref cmin, ref cmoy, ref cmax);
                }
            }

            if (type is TopAbs_VERTEX or TopAbs_SHAPE)
            {
                for (var ex = new TopExp_Explorer(shape, TopAbs_VERTEX); ex.More(); ex.Next())
                {
                    AddTol(BRep_Tool.Tolerance(TopoDS.Vertex(ex.Current())), ref nbt, ref cmin, ref cmoy, ref cmax);
                }
            }

            if (nbt == 0)
            {
                return;
            }

            if (myNbTol == 0 || myTols[0] > cmin)
            {
                myTols[0] = cmin;
            }

            if (myNbTol == 0 || myTols[2] < cmax)
            {
                myTols[2] = cmax;
            }

            myTols[1] += cmoy;
            myNbTol += nbt;
        }

        public double GlobalTolerance(int mode)
        {
            double result = 0;
            if (myNbTol != 0)
            {
                if (mode < 0)
                {
                    result = myTols[0];
                }
                else if (mode == 0)
                {
                    result = myTols[0] == myTols[2] ? myTols[0] : myTols[1] / myNbTol;
                }
                else
                {
                    result = myTols[2];
                }
            }

            return result;
        }

        public List<TopoDS_Shape> InTolerance(in TopoDS_Shape shape, double valmin, double valmax, TopAbs_ShapeEnum type)
        {
            bool over = valmax < valmin; // no upper bound
            var sl = new List<TopoDS_Shape>();
            if (type is TopAbs_FACE or TopAbs_SHAPE)
            {
                for (var ex = new TopExp_Explorer(shape, TopAbs_FACE); ex.More(); ex.Next())
                {
                    double tol = BRep_Tool.Tolerance(TopoDS.Face(ex.Current()));
                    if (tol >= valmin && (over || tol <= valmax))
                    {
                        sl.Add(ex.Current());
                    }
                }
            }

            if (type is TopAbs_EDGE or TopAbs_SHAPE)
            {
                for (var ex = new TopExp_Explorer(shape, TopAbs_EDGE); ex.More(); ex.Next())
                {
                    double tol = BRep_Tool.Tolerance(TopoDS.Edge(ex.Current()));
                    if (tol >= valmin && (over || tol <= valmax))
                    {
                        sl.Add(ex.Current());
                    }
                }
            }

            if (type is TopAbs_VERTEX or TopAbs_SHAPE)
            {
                for (var ex = new TopExp_Explorer(shape, TopAbs_VERTEX); ex.More(); ex.Next())
                {
                    double tol = BRep_Tool.Tolerance(TopoDS.Vertex(ex.Current()));
                    // OCCT bug: "tol >= valmax" for vertices, where faces and edges test "tol <= valmax" (occt-issues.md)
                    if (tol >= valmin && (over || tol >= valmax))
                    {
                        sl.Add(ex.Current());
                    }
                }
            }

            if (type == TopAbs_SHELL)
            {
                // the shells, with their faces that are in the range
                var mapface = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
                for (var ex = new TopExp_Explorer(shape, TopAbs_SHELL); ex.More(); ex.Next())
                {
                    bool iashell = false;
                    TopoDS_Shape ash = ex.Current();
                    for (var face = new TopExp_Explorer(ash, TopAbs_FACE); face.More(); face.Next())
                    {
                        mapface.Add(face.Current());
                        List<TopoDS_Shape> fc = InTolerance(face.Current(), valmin, valmax, type);
                        if (fc.Count > 0)
                        {
                            sl.AddRange(fc);
                            iashell = true;
                        }
                    }

                    if (iashell)
                    {
                        sl.Add(ash);
                    }
                }

                // the faces (free or in shells)
                for (var ex = new TopExp_Explorer(shape, TopAbs_FACE); ex.More(); ex.Next())
                {
                    bool iaface = false;
                    if (mapface.Contains(ex.Current()))
                    {
                        continue;
                    }

                    double tol = BRep_Tool.Tolerance(TopoDS.Face(ex.Current()));
                    if (tol >= valmin && (over || tol <= valmax))
                    {
                        iaface = true;
                    }
                    else if (InTolerance(ex.Current(), valmin, valmax, TopAbs_EDGE).Count > 0
                             || InTolerance(ex.Current(), valmin, valmax, TopAbs_VERTEX).Count > 0)
                    {
                        iaface = true;
                    }

                    if (iaface)
                    {
                        sl.Add(ex.Current());
                    }
                }
            }

            return sl;
        }
    }

    private static int Tolerance(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        int argc = argv.Count;
        if (argc < 2)
        {
            di.Append("myshape  : analyses a shape\n")
              .Append("myshape [mode] valmin : sub-shapes over valmin\n")
              .Append("myshape [mode] valmin valmax : between valmin and valmax\n")
              .Append("myshape [mode] 0 valmax : below valmax\n")
              .Append("  more : all shapes(D)  v vertices  e edges  f faces  c combined(faces)\n");
            return 0;
        }

        TopoDS_Shape Shape = theDI.GetShape(argv[1]);
        if (Shape.IsNull())
        {
            di.Append("Shape unknown : ").Append(argv[1]).Append("\n");
            return 1;
        }

        var sat = new ShapeTolerance();
        if (argc == 2)
        {
            void Line(string theLabel, TopAbs_ShapeEnum theType)
            {
                sat.InitTolerance();
                sat.AddTolerance(Shape, theType);
                di.Append(theLabel).Append("MAX=").Append(sat.GlobalTolerance(1)).Append(" AVG=").Append(sat.GlobalTolerance(0))
                  .Append(" MIN=").Append(sat.GlobalTolerance(-1)).Append("\n");
            }

            Line("Tolerance ", TopAbs_SHAPE);
            Line("FACE    : ", TopAbs_FACE);
            Line("EDGE    : ", TopAbs_EDGE);
            Line("VERTEX  : ", TopAbs_VERTEX);
            return 0;
        }

        // a closer analysis
        int nextarg = 2;
        char opt = argv[2].Length > 0 ? argv[2][0] : '\0';
        TopAbs_ShapeEnum type = TopAbs_SHAPE;
        switch (opt)
        {
            case 'a':
                nextarg = 3;
                break;
            case 'c':
                nextarg = 3;
                type = TopAbs_SHELL;
                break;
            case 'e':
                nextarg = 3;
                type = TopAbs_EDGE;
                break;
            case 'f':
                nextarg = 3;
                type = TopAbs_FACE;
                break;
            case 'v':
                nextarg = 3;
                type = TopAbs_VERTEX;
                break;
        }

        double tol1 = 0, tol2 = 0;
        if (nextarg < argc)
        {
            tol1 = theDI.Atof(argv[nextarg]);
        }

        if (nextarg < argc - 1)
        {
            tol2 = theDI.Atof(argv[nextarg + 1]);
        }

        List<TopoDS_Shape> list = sat.InTolerance(Shape, tol1, tol2, type);
        int nb = list.Count;
        di.Append(type switch
        {
            TopAbs_VERTEX => "Analysing Vertices gives ",
            TopAbs_EDGE => "Analysing Edges gives ",
            TopAbs_FACE => "Analysing Faces gives ",
            TopAbs_SHELL => "Analysing Shells,Faces+content gives ",
            _ => "Analysing all sub-shapes gives ",
        });
        if (tol1 == 0)
        {
            di.Append(nb).Append(" Shapes below tol=").Append(tol2);
        }
        else if (tol2 == 0)
        {
            di.Append(nb).Append(" Shapes over tol=").Append(tol1);
        }
        else
        {
            di.Append(nb).Append(" Shapes between tol1=").Append(tol1).Append(" and tol2=").Append(tol2);
        }

        if (nb == 1)
        {
            di.Append(" , named tol_1");
        }

        if (nb > 1)
        {
            di.Append(" , named tol_1 to tol_").Append(nb);
        }

        di.Append("\n");
        for (int i = 1; i <= nb; i++)
        {
            theDI.SetShape("tol_" + i.ToString(CultureInfo.InvariantCulture), list[i - 1]);
        }

        return 0;
    }
}
