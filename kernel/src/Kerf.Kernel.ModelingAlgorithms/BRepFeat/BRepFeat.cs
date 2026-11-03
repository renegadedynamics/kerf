// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat.hxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Library;
using Kerf.Engine.Extrema;
using Kerf.Engine.Features.Local;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;
using static Kerf.Engine.Features.StatusError;

namespace Kerf.Engine.Features;

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
    private const int Nechantbaryc = 11;

    /// <summary>Sample points on the edges of <paramref name="shape"/> (LocOpe's).</summary>
    public static void SampleEdges(in Shape shape, NCollection_Sequence<Point3D> seq)
    {
        global::Kerf.Engine.Features.Local.LocOpe.SampleEdges(shape, seq);
    }

    /// <summary>The barycentre of the edges and vertices of a shape.</summary>
    public static void Barycenter(in Shape s, out Point3D b)
    {
        HashSet<Shape> theMap = new(ShapeHasher.Instance);
        Explorer exp = new(s, ShapeType.Edge);
        Curve3D c;
        double prm;
        Xyz bar = new(0.0, 0.0, 0.0);
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
                c = global::Kerf.Engine.Topology.Brep.Tool.Curve(edg, out Placement loc, out double f, out double l)!;
                c = (Curve3D)c.Transformed(loc.Transformation);
                for (i = 1; i < Nechantbaryc; i++)
                {
                    prm = ((Nechantbaryc - i) * f + i * l) / Nechantbaryc;
                    bar += c.Value(prm).XYZ();
                    nbp++;
                }
            }
        }
        // Adds every vertex
        for (exp.Init(s, ShapeType.Vertex); exp.More(); exp.Next())
        {
            if (theMap.Add(exp.Current()))
            {
                bar += global::Kerf.Engine.Topology.Brep.Tool.Pnt(ShapeCasts.Vertex(exp.Current())).XYZ();
                nbp++;
            }
        }

        bar.Divide((double)nbp);
        b = new Point3D();
        b = b.WithXYZ(bar);
    }

    /// <summary>The "parametric" barycentre of a shape on a curve.</summary>
    public static double ParametricBarycenter(in Shape s, Curve3D cc)
    {
        HashSet<Shape> theMap = new(ShapeHasher.Instance);
        Explorer exp = new(s, ShapeType.Edge);
        Curve3D c;
        double prm;
        int i, nbp = 0;
        CurveAdaptor theCurve = new(cc);
        CurvePointExtrema extpc = new();
        extpc.Initialize(theCurve, cc.FirstParameter, cc.LastParameter);
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
                c = global::Kerf.Engine.Topology.Brep.Tool.Curve(edg, out Placement loc, out double f, out double l)!;
                c = (Curve3D)c.Transformed(loc.Transformation);
                for (i = 1; i < Nechantbaryc; i++)
                {
                    prm = ((Nechantbaryc - i) * f + i * l) / Nechantbaryc;
                    Point3D pone = c.Value(prm);
                    // On projette sur CC
                    extpc.Perform(pone);
                    if (extpc.IsDone && extpc.ExtCount >= 1)
                    {
                        double dist2Min = extpc.SquareDistance(1);
                        int kmin = 1;
                        for (int k = 2; k <= extpc.ExtCount; k++)
                        {
                            double dist2 = extpc.SquareDistance(k);
                            if (dist2 < dist2Min)
                            {
                                dist2Min = dist2;
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
        for (exp.Init(s, ShapeType.Vertex); exp.More(); exp.Next())
        {
            if (theMap.Add(exp.Current()))
            {
                Point3D pone = global::Kerf.Engine.Topology.Brep.Tool.Pnt(ShapeCasts.Vertex(exp.Current()));
                // On projette sur CC
                extpc.Perform(pone);
                if (extpc.IsDone && extpc.ExtCount >= 1)
                {
                    double dist2Min = extpc.SquareDistance(1);
                    for (int k = 2; k <= extpc.ExtCount; k++)
                    {
                        double dist2 = extpc.SquareDistance(k);
                        if (dist2 < dist2Min)
                        {
                            dist2Min = dist2;
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
    /// The parameters on <paramref name="cc"/> of the intersections with the shape (taking account of their orientation
    /// when <paramref name="ori"/> is true) and of the projections of its edges and vertices.
    /// </summary>
    public static void ParametricMinMax(in Shape s, Curve3D cc, out double prmin, out double prmax,
                                        out double prbmin, out double prbmax, out bool flag, bool ori = false)
    {
        CSIntersector asi = new(s);
        NCollection_Sequence<Curve3D?> scur = new();
        scur.Append(cc);
        asi.Perform(scur);
        if (asi.IsDone && asi.NbPoints(1) >= 1)
        {
            if (!ori)
            {
                prmin = Min(asi.Point(1, 1).Parameter(), asi.Point(1, asi.NbPoints(1)).Parameter());
                prmax = Max(asi.Point(1, 1).Parameter(), asi.Point(1, asi.NbPoints(1)).Parameter());
            }
            else
            {
                Orientation Ori = asi.Point(1, 1).Orientation();
                if (Ori == Forward)
                {
                    prmin = asi.Point(1, 1).Parameter();
                    prmax = asi.Point(1, asi.NbPoints(1)).Parameter();
                }
                else
                {
                    prmax = asi.Point(1, 1).Parameter();
                    prmin = asi.Point(1, asi.NbPoints(1)).Parameter();
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

        HashSet<Shape> theMap = new(ShapeHasher.Instance);
        Explorer exp = new(s, ShapeType.Edge);
        Curve3D c;
        double prm;
        int i;
        CurveAdaptor theCurve = new(cc);
        CurvePointExtrema extpc = new();
        extpc.Initialize(theCurve, cc.FirstParameter, cc.LastParameter);
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
                c = global::Kerf.Engine.Topology.Brep.Tool.Curve(edg, out Placement loc, out double f, out double l)!;
                c = (Curve3D)c.Transformed(loc.Transformation);
                for (i = 1; i < Nechantbaryc; i++)
                {
                    prm = ((Nechantbaryc - i) * f + i * l) / Nechantbaryc;
                    Point3D pone = c.Value(prm);
                    // On projette sur CC
                    extpc.Perform(pone);
                    if (extpc.IsDone && extpc.ExtCount >= 1)
                    {
                        double dist2Min = extpc.SquareDistance(1);
                        int kmin = 1;
                        for (int k = 2; k <= extpc.ExtCount; k++)
                        {
                            double dist2 = extpc.SquareDistance(k);
                            if (dist2 < dist2Min)
                            {
                                dist2Min = dist2;
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
        for (exp.Init(s, ShapeType.Vertex); exp.More(); exp.Next())
        {
            if (theMap.Add(exp.Current()))
            {
                Point3D pone = global::Kerf.Engine.Topology.Brep.Tool.Pnt(ShapeCasts.Vertex(exp.Current()));
                // On projette sur CC
                extpc.Perform(pone);
                if (extpc.IsDone && extpc.ExtCount >= 1)
                {
                    double dist2Min = extpc.SquareDistance(1);
                    int kmin = 1;
                    for (int k = 2; k <= extpc.ExtCount; k++)
                    {
                        double dist2 = extpc.SquareDistance(k);
                        if (dist2 < dist2Min)
                        {
                            dist2Min = dist2;
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

    private static bool IsIn(FClass2d fc, CurveAdaptor2D ac)
    {
        double def = 100 * global::Kerf.Engine.Foundation.Precision.Confusion();
        QuasiUniformDeflection qu = new(ac, def);

        for (int i = 1; i <= qu.PointCount; i++)
        {
            Point2D p = ac.Value(qu.Parameter(i));
            if (fc.Perform(p, false) == Out)
            {
                return false;
            }
        }
        return true;
    }

    // Recadre en U.
    private static void PutInBoundsU(double umin, double umax, double eps, double period, double f, double l,
                                     ref Curve2D c2d)
    {
        Point2D pf = c2d.Value(f);
        Point2D pl = c2d.Value(l);
        Point2D pm = c2d.Value(0.34 * f + 0.66 * l);
        double minC = Min(pf.X, pl.X);
        minC = Min(minC, pm.X);
        double maxC = Max(pf.X, pl.X);
        maxC = Max(maxC, pm.X);
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
            Vector2D t1 = new(du, 0.0);
            c2d = Writable(c2d);
            c2d.Translate(t1);
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
                Vector2D t2 = new(du, 0.0);
                c2d = Writable(c2d);
                c2d.Translate(t2);
            }
        }
    }

    // Recadre en V.
    private static void PutInBoundsV(double vmin, double vmax, double eps, double period, double f, double l,
                                     ref Curve2D c2d)
    {
        Point2D pf = c2d.Value(f);
        Point2D pl = c2d.Value(l);
        Point2D pm = c2d.Value(0.34 * f + 0.66 * l);
        double minC = Min(pf.Y, pl.Y);
        minC = Min(minC, pm.Y);
        double maxC = Max(pf.Y, pl.Y);
        maxC = Max(maxC, pm.Y);
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
            Vector2D t1 = new(0.0, dv);
            c2d = Writable(c2d);
            c2d.Translate(t1);
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
                Vector2D t2 = new(0.0, dv);
                c2d = Writable(c2d);
                c2d.Translate(t2);
            }
        }
    }

    // OCCT.NET: OCCT translates the projected curve in place; a frozen one is translated on a copy (decision 974).
    private static Curve2D Writable(Curve2D curve) =>
        curve.IsFrozen ? (Curve2D)curve.Copy() : curve;

    /// <summary>True when the edges of <paramref name="F1"/>, projected on <paramref name="f2"/>, lie inside it.</summary>
    public static bool IsInside(in Face F1, in Face f2)
    {
        Explorer exp = new();
        exp.Init(F1, ShapeType.Edge);

        double uperiod = 0, vperiod = 0;
        int flagu = 0, flagv = 0;
        Surface s = global::Kerf.Engine.Topology.Brep.Tool.Surface(f2)!;
        BrepTools.UVBounds(f2, out double umin, out double umax, out double vmin, out double vmax);

        if (s.IsUPeriodic)
        {
            flagu = 1;
            uperiod = s.UPeriod;
        }

        if (s.IsVPeriodic)
        {
            flagv = 1;
            vperiod = s.VPeriod;
        }
        Shape localShape = f2.Oriented(Forward);
        FClass2d fc = new(ShapeCasts.Face(localShape), global::Kerf.Engine.Foundation.Precision.Confusion());
        for (; exp.More(); exp.Next())
        {
            Curve3D c0 = global::Kerf.Engine.Topology.Brep.Tool.Curve(ShapeCasts.Edge(exp.Current()), out double f1, out double l1)!;
            Curve2D c = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(c0, f1, l1, s)!;
            Edge e = ShapeCasts.Edge(exp.Current());
            if (flagu == 1 || flagv == 1)
            {
                double eps = global::Kerf.Engine.Topology.Brep.Tool.Tolerance(e);
                global::Kerf.Engine.Topology.Brep.Tool.Range(e, out f1, out l1);
                if (flagu == 1)
                {
                    PutInBoundsU(umin, umax, eps, uperiod, f1, l1, ref c);
                }
                if (flagv == 1)
                {
                    PutInBoundsV(vmin, vmax, eps, vperiod, f1, l1, ref c);
                }
            }
            CurveAdaptor2D ac = new(c, f1, l1);
            if (!IsIn(fc, ac))
            {
                return false;
            }
        }
        return true;
    }

    /// <summary>
    /// Replaces <paramref name="fUntil"/> by a face on its surface (a plane, cylinder or cone, else a null face) large
    /// enough to cut through <paramref name="sbase"/>.
    /// </summary>
    public static void FaceUntil(in Shape sbase, ref Face fUntil)
    {
        Box b = new();
        Bounds.Add(sbase, ref b);
        double[] x = new double[2], y = new double[2], z = new double[2];
        b.Get(out x[0], out y[0], out z[0], out x[1], out y[1], out z[1]);
        double diam = 10.0 * Math.Sqrt(b.SquareExtent());

        Surface s = global::Kerf.Engine.Topology.Brep.Tool.Surface(fUntil)!;
        if (s is TrimmedSurface trimmed)
        {
            s = trimmed.BasisSurface;
        }
        TrimmedSurface str;
        if (s.GetType() == typeof(PlaneSurface))
        {
            Plane3D pln = ((PlaneSurface)s).Plane;
            double u, v, umin = RealLast(), umax = -umin, vmin = RealLast(), vmax = -vmin;
            for (int i = 0; i < 2; i++)
            {
                for (int j = 0; j < 2; j++)
                {
                    for (int k = 0; k < 2; k++)
                    {
                        Point3D p = new(x[i], y[j], z[k]);
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(pln, p, out u, out v);
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
            Cylinder cyl = ((CylindricalSurface)s).Cylinder;
            double u, v, vmin = RealLast(), vmax = -vmin;
            for (int i = 0; i < 2; i++)
            {
                for (int j = 0; j < 2; j++)
                {
                    for (int k = 0; k < 2; k++)
                    {
                        Point3D p = new(x[i], y[j], z[k]);
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(cyl, p, out u, out v);
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
            Cone con = ((ConicalSurface)s).Cone;
            double u, v, vmin = RealLast(), vmax = -vmin;
            for (int i = 0; i < 2; i++)
            {
                for (int j = 0; j < 2; j++)
                {
                    for (int k = 0; k < 2; k++)
                    {
                        Point3D p = new(x[i], y[j], z[k]);
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(con, p, out u, out v);
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
            fUntil.Nullify();
            return;
        }

        fUntil = new LibraryMakeFace(str, global::Kerf.Engine.Foundation.Precision.Confusion()).InternalFace();
    }

    /// <summary>
    /// A solid made of the faces of <paramref name="sRef"/>, oriented so that <paramref name="fac"/> has the
    /// orientation <paramref name="orf"/> (a null solid when the faces don't make one shell).
    /// </summary>
    public static Solid Tool(in Shape sRef, in Face fac, Orientation orf)
    {
        NCollection_List<Shape> lfaces = new();
        Explorer exp = new(sRef, ShapeType.Face);
        for (; exp.More(); exp.Next())
        {
            if (exp.Current().Type == ShapeType.Face)
            {
                lfaces.Append(exp.Current());
            }
        }

        BuildShape bs = new(lfaces);
        Shape res = bs.InternalShape();
        Shell sh = new();
        if (res.Type == ShapeType.Shell)
        {
            // faire un solide
            sh = ShapeCasts.Shell(res);
        }
        else if (res.Type == ShapeType.Solid)
        {
            exp.Init(res, ShapeType.Shell);
            sh = ShapeCasts.Shell(exp.Current());
            exp.Next();
            if (exp.More())
            {
                sh.Nullify();
            }
        }

        if (sh.IsNull)
        {
            Solid so = new();
            return so;
        }

        sh.Orient(Forward);

        Orientation orient = Forward;

        for (exp.Init(sh, ShapeType.Face); exp.More(); exp.Next())
        {
            if (exp.Current().IsSame(fac))
            {
                orient = exp.Current().Orientation;
                break;
            }
        }

        bool reverse = false;
        if ((orient == fac.Orientation && orf == Reversed)
            || (orient != fac.Orientation && orf == Forward))
        {
            reverse = true;
        }

        if (reverse)
        {
            sh.Reverse();
        }

        BrepBuilder b = new();
        Solid soc = new();
        b.MakeSolid(ref soc);
        b.Add(soc, sh);
        return soc;
    }

    /// <summary>Writes the description of the error <paramref name="se"/> on <paramref name="s"/>.</summary>
    public static TextWriter Print(StatusError se, TextWriter s)
    {
        s.Write(Description(se));
        return s;
    }

    /// <summary>The description <see cref="Print"/> writes (OCCT.NET: the switch of OCCT's Print).</summary>
    public static string Description(StatusError se) => se switch
    {
        StatusError.OK => "No error",
        StatusError.BadDirect => "Directions must be opposite",
        StatusError.BadIntersect => "Intersection failure",
        StatusError.EmptyBaryCurve => "Empty BaryCurve",
        StatusError.EmptyCutResult => "Failure in Cut : Empty resulting shape",
        StatusError.FalseSide => "Verify plane and wire orientation",
        StatusError.IncDirection => "Incoherent Direction for shapes From and Until",
        StatusError.IncSlidFace => "Sliding face not in Base shape",
        StatusError.IncParameter => "Incoherent Parameter : shape Until before shape From",
        StatusError.IncTypes => "Invalid option for faces From and Until : 1 Support and 1 not",
        StatusError.IntervalOverlap => "Shapes From and Until overlap",
        StatusError.InvFirstShape => "Invalid First shape : more than 1 face",
        StatusError.InvOption => "Invalid option",
        StatusError.InvShape => "Invalid shape",
        StatusError.LocOpeNotDone => "Local Operation not done",
        StatusError.LocOpeInvNotDone => "Local Operation : intersection line conflict",
        StatusError.NoExtFace => "No Extreme faces",
        StatusError.NoFaceProf => "No Face Profile",
        StatusError.NoGluer => "Gluer Failure",
        StatusError.NoIntersectF => "No intersection between Feature and shape From",
        StatusError.NoIntersectU => "No intersection between Feature and shape Until",
        StatusError.NoParts => "No parts of tool kept",
        StatusError.NoProjPt => "No projection points",
        StatusError.NotInitialized => "Fields not initialized",
        StatusError.NotYetImplemented => "Not yet implemented",
        StatusError.NullRealTool => "Real Tool : Null DPrism",
        StatusError.NullToolF => "Null Tool : Invalid type for shape Form",
        StatusError.NullToolU => "Null Tool : Invalid type for shape Until",
        _ => "",
    };
}
