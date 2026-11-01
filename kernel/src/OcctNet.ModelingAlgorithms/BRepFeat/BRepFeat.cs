// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat.hxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;
using static Occt.BRepFeat_StatusError;

namespace Occt;

/// <summary>
/// Tools for the creation and manipulation of form features (prisms, draft prisms, revolved features, pipes,
/// cylindrical holes) and mechanical features (ribs and slots) on a boundary representation.
/// </summary>
/// <remarks>
/// A form feature adds matter (a fusion) or removes it (a cut), for a length, up to a limiting face, from a limiting face
/// at a height, or above and below a plane. BRepFeat_trace.cxx holds only <c>OCCT_DEBUG</c> trace switches: left out.
/// </remarks>
public static class BRepFeat
{
    private const int NECHANTBARYC = 11;

    /// <summary>Sample points on the edges of <paramref name="theShape"/> (LocOpe's).</summary>
    public static void SampleEdges(in TopoDS_Shape theShape, NCollection_Sequence<gp_Pnt> theSeq)
    {
        LocOpe.SampleEdges(theShape, theSeq);
    }

    /// <summary>The barycentre of the edges and vertices of a shape.</summary>
    public static void Barycenter(in TopoDS_Shape S, out gp_Pnt B)
    {
        NCollection_Map<TopoDS_Shape> theMap = new(TopTools_ShapeMapHasher.Instance);
        TopExp_Explorer exp = new(S, TopAbs_EDGE);
        Geom_Curve C;
        double prm;
        gp_XYZ Bar = new(0.0, 0.0, 0.0);
        int i, nbp = 0;

        for (; exp.More(); exp.Next())
        {
            TopoDS_Edge edg = TopoDS.Edge(exp.Current());
            if (!theMap.Add(edg))
            {
                continue;
            }
            if (!BRep_Tool.Degenerated(edg))
            {
                C = BRep_Tool.Curve(edg, out TopLoc_Location Loc, out double f, out double l)!;
                C = (Geom_Curve)C.Transformed(Loc.Transformation());
                for (i = 1; i < NECHANTBARYC; i++)
                {
                    prm = ((NECHANTBARYC - i) * f + i * l) / NECHANTBARYC;
                    Bar += C.Value(prm).XYZ();
                    nbp++;
                }
            }
        }
        // Adds every vertex
        for (exp.Init(S, TopAbs_VERTEX); exp.More(); exp.Next())
        {
            if (theMap.Add(exp.Current()))
            {
                Bar += BRep_Tool.Pnt(TopoDS.Vertex(exp.Current())).XYZ();
                nbp++;
            }
        }

        Bar.Divide((double)nbp);
        B = new gp_Pnt();
        B.SetXYZ(Bar);
    }

    /// <summary>The "parametric" barycentre of a shape on a curve.</summary>
    public static double ParametricBarycenter(in TopoDS_Shape S, Geom_Curve CC)
    {
        NCollection_Map<TopoDS_Shape> theMap = new(TopTools_ShapeMapHasher.Instance);
        TopExp_Explorer exp = new(S, TopAbs_EDGE);
        Geom_Curve C;
        double prm;
        int i, nbp = 0;
        GeomAdaptor_Curve TheCurve = new(CC);
        Extrema_ExtPC extpc = new();
        extpc.Initialize(TheCurve, CC.FirstParameter(), CC.LastParameter());
        double parbar = 0;

        for (; exp.More(); exp.Next())
        {
            TopoDS_Edge edg = TopoDS.Edge(exp.Current());
            if (!theMap.Add(edg))
            {
                continue;
            }
            if (!BRep_Tool.Degenerated(edg))
            {
                C = BRep_Tool.Curve(edg, out TopLoc_Location Loc, out double f, out double l)!;
                C = (Geom_Curve)C.Transformed(Loc.Transformation());
                for (i = 1; i < NECHANTBARYC; i++)
                {
                    prm = ((NECHANTBARYC - i) * f + i * l) / NECHANTBARYC;
                    gp_Pnt pone = C.Value(prm);
                    // On projette sur CC
                    extpc.Perform(pone);
                    if (extpc.IsDone() && extpc.NbExt() >= 1)
                    {
                        double Dist2Min = extpc.SquareDistance(1);
                        int kmin = 1;
                        for (int k = 2; k <= extpc.NbExt(); k++)
                        {
                            double Dist2 = extpc.SquareDistance(k);
                            if (Dist2 < Dist2Min)
                            {
                                Dist2Min = Dist2;
                                kmin = k;
                            }
                        }
                        nbp++;
                        double prmp = extpc.Point(kmin).Parameter();
                        parbar += prmp;
                    }
                }
            }
        }
        // Adds every vertex
        for (exp.Init(S, TopAbs_VERTEX); exp.More(); exp.Next())
        {
            if (theMap.Add(exp.Current()))
            {
                gp_Pnt pone = BRep_Tool.Pnt(TopoDS.Vertex(exp.Current()));
                // On projette sur CC
                extpc.Perform(pone);
                if (extpc.IsDone() && extpc.NbExt() >= 1)
                {
                    double Dist2Min = extpc.SquareDistance(1);
                    for (int k = 2; k <= extpc.NbExt(); k++)
                    {
                        double Dist2 = extpc.SquareDistance(k);
                        if (Dist2 < Dist2Min)
                        {
                            Dist2Min = Dist2;
                        }
                    }
                    nbp++;
                }
            }
        }

        parbar /= ((double)nbp);
        return parbar;
    }

    /// <summary>
    /// The parameters on <paramref name="CC"/> of the intersections with the shape (taking account of their orientation
    /// when <paramref name="theOri"/> is true) and of the projections of its edges and vertices.
    /// </summary>
    public static void ParametricMinMax(in TopoDS_Shape S, Geom_Curve CC, out double prmin, out double prmax,
                                        out double prbmin, out double prbmax, out bool flag, bool theOri = false)
    {
        LocOpe_CSIntersector ASI = new(S);
        NCollection_Sequence<Geom_Curve?> scur = new();
        scur.Append(CC);
        ASI.Perform(scur);
        if (ASI.IsDone() && ASI.NbPoints(1) >= 1)
        {
            if (!theOri)
            {
                prmin = Min(ASI.Point(1, 1).Parameter(), ASI.Point(1, ASI.NbPoints(1)).Parameter());
                prmax = Max(ASI.Point(1, 1).Parameter(), ASI.Point(1, ASI.NbPoints(1)).Parameter());
            }
            else
            {
                TopAbs_Orientation Ori = ASI.Point(1, 1).Orientation();
                if (Ori == TopAbs_FORWARD)
                {
                    prmin = ASI.Point(1, 1).Parameter();
                    prmax = ASI.Point(1, ASI.NbPoints(1)).Parameter();
                }
                else
                {
                    prmax = ASI.Point(1, 1).Parameter();
                    prmin = ASI.Point(1, ASI.NbPoints(1)).Parameter();
                }
            }
            flag = true;
        }
        else
        {
            prmax = RealFirst();
            prmin = RealLast();
            flag = false;
        }

        NCollection_Map<TopoDS_Shape> theMap = new(TopTools_ShapeMapHasher.Instance);
        TopExp_Explorer exp = new(S, TopAbs_EDGE);
        Geom_Curve C;
        double prm;
        int i;
        GeomAdaptor_Curve TheCurve = new(CC);
        Extrema_ExtPC extpc = new();
        extpc.Initialize(TheCurve, CC.FirstParameter(), CC.LastParameter());
        prbmin = RealLast();
        prbmax = RealFirst();
        for (; exp.More(); exp.Next())
        {
            TopoDS_Edge edg = TopoDS.Edge(exp.Current());
            if (!theMap.Add(edg))
            {
                continue;
            }
            if (!BRep_Tool.Degenerated(edg))
            {
                C = BRep_Tool.Curve(edg, out TopLoc_Location Loc, out double f, out double l)!;
                C = (Geom_Curve)C.Transformed(Loc.Transformation());
                for (i = 1; i < NECHANTBARYC; i++)
                {
                    prm = ((NECHANTBARYC - i) * f + i * l) / NECHANTBARYC;
                    gp_Pnt pone = C.Value(prm);
                    // On projette sur CC
                    extpc.Perform(pone);
                    if (extpc.IsDone() && extpc.NbExt() >= 1)
                    {
                        double Dist2Min = extpc.SquareDistance(1);
                        int kmin = 1;
                        for (int k = 2; k <= extpc.NbExt(); k++)
                        {
                            double Dist2 = extpc.SquareDistance(k);
                            if (Dist2 < Dist2Min)
                            {
                                Dist2Min = Dist2;
                                kmin = k;
                            }
                        }
                        double prmp = extpc.Point(kmin).Parameter();
                        if (prmp <= prbmin)
                        {
                            prbmin = prmp;
                        }
                        if (prmp >= prbmax)
                        {
                            prbmax = prmp;
                        }
                    }
                }
            }
        }
        // Adds every vertex
        for (exp.Init(S, TopAbs_VERTEX); exp.More(); exp.Next())
        {
            if (theMap.Add(exp.Current()))
            {
                gp_Pnt pone = BRep_Tool.Pnt(TopoDS.Vertex(exp.Current()));
                // On projette sur CC
                extpc.Perform(pone);
                if (extpc.IsDone() && extpc.NbExt() >= 1)
                {
                    double Dist2Min = extpc.SquareDistance(1);
                    int kmin = 1;
                    for (int k = 2; k <= extpc.NbExt(); k++)
                    {
                        double Dist2 = extpc.SquareDistance(k);
                        if (Dist2 < Dist2Min)
                        {
                            Dist2Min = Dist2;
                            kmin = k;
                        }
                    }
                    double prmp = extpc.Point(kmin).Parameter();
                    if (prmp <= prbmin)
                    {
                        prbmin = prmp;
                    }
                    if (prmp >= prbmax)
                    {
                        prbmax = prmp;
                    }
                }
            }
        }
    }

    private static bool IsIn(BRepTopAdaptor_FClass2d FC, Geom2dAdaptor_Curve AC)
    {
        double Def = 100 * Precision.Confusion();
        GCPnts_QuasiUniformDeflection QU = new(AC, Def);

        for (int i = 1; i <= QU.NbPoints(); i++)
        {
            gp_Pnt2d P = AC.Value(QU.Parameter(i));
            if (FC.Perform(P, false) == TopAbs_OUT)
            {
                return false;
            }
        }
        return true;
    }

    // Recadre en U.
    private static void PutInBoundsU(double umin, double umax, double eps, double period, double f, double l,
                                     ref Geom2d_Curve C2d)
    {
        gp_Pnt2d Pf = C2d.Value(f);
        gp_Pnt2d Pl = C2d.Value(l);
        gp_Pnt2d Pm = C2d.Value(0.34 * f + 0.66 * l);
        double minC = Min(Pf.X(), Pl.X());
        minC = Min(minC, Pm.X());
        double maxC = Max(Pf.X(), Pl.X());
        maxC = Max(maxC, Pm.X());
        double du = 0.0;
        if (minC < umin - eps)
        {
            du = ((int)((umin - minC) / period) + 1) * period;
        }
        if (minC > umax + eps)
        {
            du = -((int)((minC - umax) / period) + 1) * period;
        }
        if (du != 0)
        {
            gp_Vec2d T1 = new(du, 0.0);
            C2d = Writable(C2d);
            C2d.Translate(T1);
            minC += du;
            maxC += du;
        }
        // Ajuste au mieux la courbe dans le domaine.
        if (maxC > umax + 100 * eps)
        {
            double d1 = maxC - umax;
            double d2 = umin - minC + period;
            if (d2 < d1)
            {
                du = -period;
            }
            if (du != 0.0)
            {
                gp_Vec2d T2 = new(du, 0.0);
                C2d = Writable(C2d);
                C2d.Translate(T2);
            }
        }
    }

    // Recadre en V.
    private static void PutInBoundsV(double vmin, double vmax, double eps, double period, double f, double l,
                                     ref Geom2d_Curve C2d)
    {
        gp_Pnt2d Pf = C2d.Value(f);
        gp_Pnt2d Pl = C2d.Value(l);
        gp_Pnt2d Pm = C2d.Value(0.34 * f + 0.66 * l);
        double minC = Min(Pf.Y(), Pl.Y());
        minC = Min(minC, Pm.Y());
        double maxC = Max(Pf.Y(), Pl.Y());
        maxC = Max(maxC, Pm.Y());
        double dv = 0.0;
        if (minC < vmin - eps)
        {
            dv = ((int)((vmin - minC) / period) + 1) * period;
        }
        if (minC > vmax + eps)
        {
            dv = -((int)((minC - vmax) / period) + 1) * period;
        }
        if (dv != 0)
        {
            gp_Vec2d T1 = new(0.0, dv);
            C2d = Writable(C2d);
            C2d.Translate(T1);
            minC += dv;
            maxC += dv;
        }
        // Ajuste au mieux la courbe dans le domaine.
        if (maxC > vmax + 100 * eps)
        {
            double d1 = maxC - vmax;
            double d2 = vmin - minC + period;
            if (d2 < d1)
            {
                dv = -period;
            }
            if (dv != 0.0)
            {
                gp_Vec2d T2 = new(0.0, dv);
                C2d = Writable(C2d);
                C2d.Translate(T2);
            }
        }
    }

    // OCCT.NET: OCCT translates the projected curve in place; a frozen one is translated on a copy (decision 974).
    private static Geom2d_Curve Writable(Geom2d_Curve theCurve) =>
        theCurve.IsFrozen() ? (Geom2d_Curve)theCurve.Copy() : theCurve;

    /// <summary>True when the edges of <paramref name="F1"/>, projected on <paramref name="F2"/>, lie inside it.</summary>
    public static bool IsInside(in TopoDS_Face F1, in TopoDS_Face F2)
    {
        TopExp_Explorer exp = new();
        exp.Init(F1, TopAbs_EDGE);

        double uperiod = 0, vperiod = 0;
        int flagu = 0, flagv = 0;
        Geom_Surface S = BRep_Tool.Surface(F2)!;
        BRepTools.UVBounds(F2, out double umin, out double umax, out double vmin, out double vmax);

        if (S.IsUPeriodic())
        {
            flagu = 1;
            uperiod = S.UPeriod();
        }

        if (S.IsVPeriodic())
        {
            flagv = 1;
            vperiod = S.VPeriod();
        }
        TopoDS_Shape aLocalShape = F2.Oriented(TopAbs_FORWARD);
        BRepTopAdaptor_FClass2d FC = new(TopoDS.Face(aLocalShape), Precision.Confusion());
        for (; exp.More(); exp.Next())
        {
            Geom_Curve C0 = BRep_Tool.Curve(TopoDS.Edge(exp.Current()), out double f1, out double l1)!;
            Geom2d_Curve C = GeomProjLib.Curve2d(C0, f1, l1, S)!;
            TopoDS_Edge E = TopoDS.Edge(exp.Current());
            if (flagu == 1 || flagv == 1)
            {
                double eps = BRep_Tool.Tolerance(E);
                BRep_Tool.Range(E, out f1, out l1);
                if (flagu == 1)
                {
                    PutInBoundsU(umin, umax, eps, uperiod, f1, l1, ref C);
                }
                if (flagv == 1)
                {
                    PutInBoundsV(vmin, vmax, eps, vperiod, f1, l1, ref C);
                }
            }
            Geom2dAdaptor_Curve AC = new(C, f1, l1);
            if (!IsIn(FC, AC))
            {
                return false;
            }
        }
        return true;
    }

    /// <summary>
    /// Replaces <paramref name="FUntil"/> by a face on its surface (a plane, cylinder or cone, else a null face) large
    /// enough to cut through <paramref name="Sbase"/>.
    /// </summary>
    public static void FaceUntil(in TopoDS_Shape Sbase, ref TopoDS_Face FUntil)
    {
        Bnd_Box B = new();
        BRepBndLib.Add(Sbase, ref B);
        double[] x = new double[2], y = new double[2], z = new double[2];
        B.Get(out x[0], out y[0], out z[0], out x[1], out y[1], out z[1]);
        double diam = 10.0 * Math.Sqrt(B.SquareExtent());

        Geom_Surface s = BRep_Tool.Surface(FUntil)!;
        if (s is Geom_RectangularTrimmedSurface aTrimmed)
        {
            s = aTrimmed.BasisSurface();
        }
        Geom_RectangularTrimmedSurface str;
        if (s.GetType() == typeof(Geom_Plane))
        {
            gp_Pln aPln = ((Geom_Plane)s).Pln();
            double u, v, umin = RealLast(), umax = -umin, vmin = RealLast(), vmax = -vmin;
            for (int i = 0; i < 2; i++)
            {
                for (int j = 0; j < 2; j++)
                {
                    for (int k = 0; k < 2; k++)
                    {
                        gp_Pnt aP = new(x[i], y[j], z[k]);
                        ElSLib.Parameters(aPln, aP, out u, out v);
                        if (u < umin)
                        {
                            umin = u;
                        }
                        if (u > umax)
                        {
                            umax = u;
                        }
                        if (v < vmin)
                        {
                            vmin = v;
                        }
                        if (v > vmax)
                        {
                            vmax = v;
                        }
                    }
                }
            }
            umin -= diam;
            umax += diam;
            vmin -= diam;
            vmax += diam;
            str = new Geom_RectangularTrimmedSurface(s, umin, umax, vmin, vmax, true, true);
        }
        else if (s.GetType() == typeof(Geom_CylindricalSurface))
        {
            gp_Cylinder aCyl = ((Geom_CylindricalSurface)s).Cylinder();
            double u, v, vmin = RealLast(), vmax = -vmin;
            for (int i = 0; i < 2; i++)
            {
                for (int j = 0; j < 2; j++)
                {
                    for (int k = 0; k < 2; k++)
                    {
                        gp_Pnt aP = new(x[i], y[j], z[k]);
                        ElSLib.Parameters(aCyl, aP, out u, out v);
                        if (v < vmin)
                        {
                            vmin = v;
                        }
                        if (v > vmax)
                        {
                            vmax = v;
                        }
                    }
                }
            }
            vmin -= diam;
            vmax += diam;
            str = new Geom_RectangularTrimmedSurface(s, vmin, vmax, false, true);
        }
        else if (s.GetType() == typeof(Geom_ConicalSurface))
        {
            gp_Cone aCon = ((Geom_ConicalSurface)s).Cone();
            double u, v, vmin = RealLast(), vmax = -vmin;
            for (int i = 0; i < 2; i++)
            {
                for (int j = 0; j < 2; j++)
                {
                    for (int k = 0; k < 2; k++)
                    {
                        gp_Pnt aP = new(x[i], y[j], z[k]);
                        ElSLib.Parameters(aCon, aP, out u, out v);
                        if (v < vmin)
                        {
                            vmin = v;
                        }
                        if (v > vmax)
                        {
                            vmax = v;
                        }
                    }
                }
            }
            vmin -= diam;
            vmax += diam;
            str = new Geom_RectangularTrimmedSurface(s, vmin, vmax, false, true);
        }
        else
        {
            FUntil.Nullify();
            return;
        }

        FUntil = new BRepLib_MakeFace(str, Precision.Confusion()).InternalFace();
    }

    /// <summary>
    /// A solid made of the faces of <paramref name="SRef"/>, oriented so that <paramref name="Fac"/> has the
    /// orientation <paramref name="Orf"/> (a null solid when the faces don't make one shell).
    /// </summary>
    public static TopoDS_Solid Tool(in TopoDS_Shape SRef, in TopoDS_Face Fac, TopAbs_Orientation Orf)
    {
        NCollection_List<TopoDS_Shape> lfaces = new();
        TopExp_Explorer exp = new(SRef, TopAbs_FACE);
        for (; exp.More(); exp.Next())
        {
            if (exp.Current().ShapeType() == TopAbs_FACE)
            {
                lfaces.Append(exp.Current());
            }
        }

        LocOpe_BuildShape bs = new(lfaces);
        TopoDS_Shape Res = bs.InternalShape();
        TopoDS_Shell Sh = new();
        if (Res.ShapeType() == TopAbs_SHELL)
        {
            // faire un solide
            Sh = TopoDS.Shell(Res);
        }
        else if (Res.ShapeType() == TopAbs_SOLID)
        {
            exp.Init(Res, TopAbs_SHELL);
            Sh = TopoDS.Shell(exp.Current());
            exp.Next();
            if (exp.More())
            {
                Sh.Nullify();
            }
        }

        if (Sh.IsNull())
        {
            TopoDS_Solid So = new();
            return So;
        }

        Sh.Orientation(TopAbs_FORWARD);

        TopAbs_Orientation orient = TopAbs_FORWARD;

        for (exp.Init(Sh, TopAbs_FACE); exp.More(); exp.Next())
        {
            if (exp.Current().IsSame(Fac))
            {
                orient = exp.Current().Orientation();
                break;
            }
        }

        bool reverse = false;
        if ((orient == Fac.Orientation() && Orf == TopAbs_REVERSED)
            || (orient != Fac.Orientation() && Orf == TopAbs_FORWARD))
        {
            reverse = true;
        }

        if (reverse)
        {
            Sh.Reverse();
        }

        BRep_Builder B = new();
        TopoDS_Solid Soc = new();
        B.MakeSolid(ref Soc);
        B.Add(Soc, Sh);
        return Soc;
    }

    /// <summary>Writes the description of the error <paramref name="se"/> on <paramref name="s"/>.</summary>
    public static TextWriter Print(BRepFeat_StatusError se, TextWriter s)
    {
        s.Write(Description(se));
        return s;
    }

    /// <summary>The description <see cref="Print"/> writes (OCCT.NET: the switch of OCCT's Print).</summary>
    public static string Description(BRepFeat_StatusError se) => se switch
    {
        BRepFeat_OK => "No error",
        BRepFeat_BadDirect => "Directions must be opposite",
        BRepFeat_BadIntersect => "Intersection failure",
        BRepFeat_EmptyBaryCurve => "Empty BaryCurve",
        BRepFeat_EmptyCutResult => "Failure in Cut : Empty resulting shape",
        BRepFeat_FalseSide => "Verify plane and wire orientation",
        BRepFeat_IncDirection => "Incoherent Direction for shapes From and Until",
        BRepFeat_IncSlidFace => "Sliding face not in Base shape",
        BRepFeat_IncParameter => "Incoherent Parameter : shape Until before shape From",
        BRepFeat_IncTypes => "Invalid option for faces From and Until : 1 Support and 1 not",
        BRepFeat_IntervalOverlap => "Shapes From and Until overlap",
        BRepFeat_InvFirstShape => "Invalid First shape : more than 1 face",
        BRepFeat_InvOption => "Invalid option",
        BRepFeat_InvShape => "Invalid shape",
        BRepFeat_LocOpeNotDone => "Local Operation not done",
        BRepFeat_LocOpeInvNotDone => "Local Operation : intersection line conflict",
        BRepFeat_NoExtFace => "No Extreme faces",
        BRepFeat_NoFaceProf => "No Face Profile",
        BRepFeat_NoGluer => "Gluer Failure",
        BRepFeat_NoIntersectF => "No intersection between Feature and shape From",
        BRepFeat_NoIntersectU => "No intersection between Feature and shape Until",
        BRepFeat_NoParts => "No parts of tool kept",
        BRepFeat_NoProjPt => "No projection points",
        BRepFeat_NotInitialized => "Fields not initialized",
        BRepFeat_NotYetImplemented => "Not yet implemented",
        BRepFeat_NullRealTool => "Real Tool : Null DPrism",
        BRepFeat_NullToolF => "Null Tool : Invalid type for shape Form",
        BRepFeat_NullToolU => "Null Tool : Invalid type for shape Until",
        _ => "",
    };
}
