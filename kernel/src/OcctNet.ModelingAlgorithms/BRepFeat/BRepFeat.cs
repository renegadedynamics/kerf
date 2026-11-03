// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat.hxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;
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
    public static void SampleEdges(in Shape theShape, NCollection_Sequence<Point3D> theSeq)
    {
        LocOpe.SampleEdges(theShape, theSeq);
    }

    /// <summary>The barycentre of the edges and vertices of a shape.</summary>
    public static void Barycenter(in Shape S, out Point3D B)
    {
        NCollection_Map<Shape> theMap = new(ShapeHasher.Instance);
        Explorer exp = new(S, ShapeType.Edge);
        Curve3D C;
        double prm;
        Xyz Bar = new(0.0, 0.0, 0.0);
        int i, nbp = 0;

        for (; exp.More(); exp.Next())
        {
            Edge edg = ShapeCasts.Edge(exp.Current());
            if (!theMap.Add(edg))
            {
                continue;
            }
            if (!global::Kerf.Engine.Topology.Brep.Tool.Degenerated(edg))
            {
                C = global::Kerf.Engine.Topology.Brep.Tool.Curve(edg, out Placement Loc, out double f, out double l)!;
                C = (Curve3D)C.Transformed(Loc.Transformation());
                for (i = 1; i < NECHANTBARYC; i++)
                {
                    prm = ((NECHANTBARYC - i) * f + i * l) / NECHANTBARYC;
                    Bar += C.Value(prm).XYZ();
                    nbp++;
                }
            }
        }
        // Adds every vertex
        for (exp.Init(S, ShapeType.Vertex); exp.More(); exp.Next())
        {
            if (theMap.Add(exp.Current()))
            {
                Bar += global::Kerf.Engine.Topology.Brep.Tool.Pnt(ShapeCasts.Vertex(exp.Current())).XYZ();
                nbp++;
            }
        }

        Bar.Divide((double)nbp);
        B = new Point3D();
        B.SetXYZ(Bar);
    }

    /// <summary>The "parametric" barycentre of a shape on a curve.</summary>
    public static double ParametricBarycenter(in Shape S, Curve3D CC)
    {
        NCollection_Map<Shape> theMap = new(ShapeHasher.Instance);
        Explorer exp = new(S, ShapeType.Edge);
        Curve3D C;
        double prm;
        int i, nbp = 0;
        CurveAdaptor TheCurve = new(CC);
        CurvePointExtrema extpc = new();
        extpc.Initialize(TheCurve, CC.FirstParameter(), CC.LastParameter());
        double parbar = 0;

        for (; exp.More(); exp.Next())
        {
            Edge edg = ShapeCasts.Edge(exp.Current());
            if (!theMap.Add(edg))
            {
                continue;
            }
            if (!global::Kerf.Engine.Topology.Brep.Tool.Degenerated(edg))
            {
                C = global::Kerf.Engine.Topology.Brep.Tool.Curve(edg, out Placement Loc, out double f, out double l)!;
                C = (Curve3D)C.Transformed(Loc.Transformation());
                for (i = 1; i < NECHANTBARYC; i++)
                {
                    prm = ((NECHANTBARYC - i) * f + i * l) / NECHANTBARYC;
                    Point3D pone = C.Value(prm);
                    // On projette sur CC
                    extpc.Perform(pone);
                    if (extpc.IsDone && extpc.ExtCount >= 1)
                    {
                        double Dist2Min = extpc.SquareDistance(1);
                        int kmin = 1;
                        for (int k = 2; k <= extpc.ExtCount; k++)
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
        for (exp.Init(S, ShapeType.Vertex); exp.More(); exp.Next())
        {
            if (theMap.Add(exp.Current()))
            {
                Point3D pone = global::Kerf.Engine.Topology.Brep.Tool.Pnt(ShapeCasts.Vertex(exp.Current()));
                // On projette sur CC
                extpc.Perform(pone);
                if (extpc.IsDone && extpc.ExtCount >= 1)
                {
                    double Dist2Min = extpc.SquareDistance(1);
                    for (int k = 2; k <= extpc.ExtCount; k++)
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
    public static void ParametricMinMax(in Shape S, Curve3D CC, out double prmin, out double prmax,
                                        out double prbmin, out double prbmax, out bool flag, bool theOri = false)
    {
        LocOpe_CSIntersector ASI = new(S);
        NCollection_Sequence<Curve3D?> scur = new();
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
                Orientation Ori = ASI.Point(1, 1).Orientation();
                if (Ori == Forward)
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

        NCollection_Map<Shape> theMap = new(ShapeHasher.Instance);
        Explorer exp = new(S, ShapeType.Edge);
        Curve3D C;
        double prm;
        int i;
        CurveAdaptor TheCurve = new(CC);
        CurvePointExtrema extpc = new();
        extpc.Initialize(TheCurve, CC.FirstParameter(), CC.LastParameter());
        prbmin = RealLast();
        prbmax = RealFirst();
        for (; exp.More(); exp.Next())
        {
            Edge edg = ShapeCasts.Edge(exp.Current());
            if (!theMap.Add(edg))
            {
                continue;
            }
            if (!global::Kerf.Engine.Topology.Brep.Tool.Degenerated(edg))
            {
                C = global::Kerf.Engine.Topology.Brep.Tool.Curve(edg, out Placement Loc, out double f, out double l)!;
                C = (Curve3D)C.Transformed(Loc.Transformation());
                for (i = 1; i < NECHANTBARYC; i++)
                {
                    prm = ((NECHANTBARYC - i) * f + i * l) / NECHANTBARYC;
                    Point3D pone = C.Value(prm);
                    // On projette sur CC
                    extpc.Perform(pone);
                    if (extpc.IsDone && extpc.ExtCount >= 1)
                    {
                        double Dist2Min = extpc.SquareDistance(1);
                        int kmin = 1;
                        for (int k = 2; k <= extpc.ExtCount; k++)
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
        for (exp.Init(S, ShapeType.Vertex); exp.More(); exp.Next())
        {
            if (theMap.Add(exp.Current()))
            {
                Point3D pone = global::Kerf.Engine.Topology.Brep.Tool.Pnt(ShapeCasts.Vertex(exp.Current()));
                // On projette sur CC
                extpc.Perform(pone);
                if (extpc.IsDone && extpc.ExtCount >= 1)
                {
                    double Dist2Min = extpc.SquareDistance(1);
                    int kmin = 1;
                    for (int k = 2; k <= extpc.ExtCount; k++)
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

    private static bool IsIn(BRepTopAdaptor_FClass2d FC, CurveAdaptor2D AC)
    {
        double Def = 100 * global::Kerf.Engine.Foundation.Precision.Confusion();
        QuasiUniformDeflection QU = new(AC, Def);

        for (int i = 1; i <= QU.PointCount; i++)
        {
            Point2D P = AC.Value(QU.Parameter(i));
            if (FC.Perform(P, false) == Out)
            {
                return false;
            }
        }
        return true;
    }

    // Recadre en U.
    private static void PutInBoundsU(double umin, double umax, double eps, double period, double f, double l,
                                     ref Curve2D C2d)
    {
        Point2D Pf = C2d.Value(f);
        Point2D Pl = C2d.Value(l);
        Point2D Pm = C2d.Value(0.34 * f + 0.66 * l);
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
            Vector2D T1 = new(du, 0.0);
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
                Vector2D T2 = new(du, 0.0);
                C2d = Writable(C2d);
                C2d.Translate(T2);
            }
        }
    }

    // Recadre en V.
    private static void PutInBoundsV(double vmin, double vmax, double eps, double period, double f, double l,
                                     ref Curve2D C2d)
    {
        Point2D Pf = C2d.Value(f);
        Point2D Pl = C2d.Value(l);
        Point2D Pm = C2d.Value(0.34 * f + 0.66 * l);
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
            Vector2D T1 = new(0.0, dv);
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
                Vector2D T2 = new(0.0, dv);
                C2d = Writable(C2d);
                C2d.Translate(T2);
            }
        }
    }

    // OCCT.NET: OCCT translates the projected curve in place; a frozen one is translated on a copy (decision 974).
    private static Curve2D Writable(Curve2D theCurve) =>
        theCurve.IsFrozen ? (Curve2D)theCurve.Copy() : theCurve;

    /// <summary>True when the edges of <paramref name="F1"/>, projected on <paramref name="F2"/>, lie inside it.</summary>
    public static bool IsInside(in Face F1, in Face F2)
    {
        Explorer exp = new();
        exp.Init(F1, ShapeType.Edge);

        double uperiod = 0, vperiod = 0;
        int flagu = 0, flagv = 0;
        Surface S = global::Kerf.Engine.Topology.Brep.Tool.Surface(F2)!;
        BrepTools.UVBounds(F2, out double umin, out double umax, out double vmin, out double vmax);

        if (S.IsUPeriodic)
        {
            flagu = 1;
            uperiod = S.UPeriod();
        }

        if (S.IsVPeriodic)
        {
            flagv = 1;
            vperiod = S.VPeriod();
        }
        Shape aLocalShape = F2.Oriented(Forward);
        BRepTopAdaptor_FClass2d FC = new(ShapeCasts.Face(aLocalShape), global::Kerf.Engine.Foundation.Precision.Confusion());
        for (; exp.More(); exp.Next())
        {
            Curve3D C0 = global::Kerf.Engine.Topology.Brep.Tool.Curve(ShapeCasts.Edge(exp.Current()), out double f1, out double l1)!;
            Curve2D C = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(C0, f1, l1, S)!;
            Edge E = ShapeCasts.Edge(exp.Current());
            if (flagu == 1 || flagv == 1)
            {
                double eps = global::Kerf.Engine.Topology.Brep.Tool.Tolerance(E);
                global::Kerf.Engine.Topology.Brep.Tool.Range(E, out f1, out l1);
                if (flagu == 1)
                {
                    PutInBoundsU(umin, umax, eps, uperiod, f1, l1, ref C);
                }
                if (flagv == 1)
                {
                    PutInBoundsV(vmin, vmax, eps, vperiod, f1, l1, ref C);
                }
            }
            CurveAdaptor2D AC = new(C, f1, l1);
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
    public static void FaceUntil(in Shape Sbase, ref Face FUntil)
    {
        Box B = new();
        BRepBndLib.Add(Sbase, ref B);
        double[] x = new double[2], y = new double[2], z = new double[2];
        B.Get(out x[0], out y[0], out z[0], out x[1], out y[1], out z[1]);
        double diam = 10.0 * Math.Sqrt(B.SquareExtent());

        Surface s = global::Kerf.Engine.Topology.Brep.Tool.Surface(FUntil)!;
        if (s is TrimmedSurface aTrimmed)
        {
            s = aTrimmed.BasisSurface();
        }
        TrimmedSurface str;
        if (s.GetType() == typeof(PlaneSurface))
        {
            Plane3D aPln = ((PlaneSurface)s).Pln();
            double u, v, umin = RealLast(), umax = -umin, vmin = RealLast(), vmax = -vmin;
            for (int i = 0; i < 2; i++)
            {
                for (int j = 0; j < 2; j++)
                {
                    for (int k = 0; k < 2; k++)
                    {
                        Point3D aP = new(x[i], y[j], z[k]);
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(aPln, aP, out u, out v);
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
            str = new TrimmedSurface(s, umin, umax, vmin, vmax, true, true);
        }
        else if (s.GetType() == typeof(CylindricalSurface))
        {
            Cylinder aCyl = ((CylindricalSurface)s).Cylinder();
            double u, v, vmin = RealLast(), vmax = -vmin;
            for (int i = 0; i < 2; i++)
            {
                for (int j = 0; j < 2; j++)
                {
                    for (int k = 0; k < 2; k++)
                    {
                        Point3D aP = new(x[i], y[j], z[k]);
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(aCyl, aP, out u, out v);
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
            str = new TrimmedSurface(s, vmin, vmax, false, true);
        }
        else if (s.GetType() == typeof(ConicalSurface))
        {
            Cone aCon = ((ConicalSurface)s).Cone();
            double u, v, vmin = RealLast(), vmax = -vmin;
            for (int i = 0; i < 2; i++)
            {
                for (int j = 0; j < 2; j++)
                {
                    for (int k = 0; k < 2; k++)
                    {
                        Point3D aP = new(x[i], y[j], z[k]);
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(aCon, aP, out u, out v);
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
            str = new TrimmedSurface(s, vmin, vmax, false, true);
        }
        else
        {
            FUntil.Nullify();
            return;
        }

        FUntil = new BRepLib_MakeFace(str, global::Kerf.Engine.Foundation.Precision.Confusion()).InternalFace();
    }

    /// <summary>
    /// A solid made of the faces of <paramref name="SRef"/>, oriented so that <paramref name="Fac"/> has the
    /// orientation <paramref name="Orf"/> (a null solid when the faces don't make one shell).
    /// </summary>
    public static Solid Tool(in Shape SRef, in Face Fac, Orientation Orf)
    {
        NCollection_List<Shape> lfaces = new();
        Explorer exp = new(SRef, ShapeType.Face);
        for (; exp.More(); exp.Next())
        {
            if (exp.Current().ShapeType() == ShapeType.Face)
            {
                lfaces.Append(exp.Current());
            }
        }

        LocOpe_BuildShape bs = new(lfaces);
        Shape Res = bs.InternalShape();
        Shell Sh = new();
        if (Res.ShapeType() == ShapeType.Shell)
        {
            // faire un solide
            Sh = ShapeCasts.Shell(Res);
        }
        else if (Res.ShapeType() == ShapeType.Solid)
        {
            exp.Init(Res, ShapeType.Shell);
            Sh = ShapeCasts.Shell(exp.Current());
            exp.Next();
            if (exp.More())
            {
                Sh.Nullify();
            }
        }

        if (Sh.IsNull)
        {
            Solid So = new();
            return So;
        }

        Sh.Orientation(Forward);

        Orientation orient = Forward;

        for (exp.Init(Sh, ShapeType.Face); exp.More(); exp.Next())
        {
            if (exp.Current().IsSame(Fac))
            {
                orient = exp.Current().Orientation();
                break;
            }
        }

        bool reverse = false;
        if ((orient == Fac.Orientation() && Orf == Reversed)
            || (orient != Fac.Orientation() && Orf == Forward))
        {
            reverse = true;
        }

        if (reverse)
        {
            Sh.Reverse();
        }

        BrepBuilder B = new();
        Solid Soc = new();
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
