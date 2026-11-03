// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_CheckSmallFace.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_CheckSmallFace.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_CheckSmallFace.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.ShapeExtend_Status;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// Analysis of the size of faces: spots (faces whose vertices and edges are confused), strips (faces of two confused
/// edges), splitting vertices, pins and twisted supports. Only reads the faces.
/// </summary>
/// <remarks>
/// A working object (decision 21) recording a status per kind of check. Outputs that OCCT may leave unwritten are
/// <c>ref</c>. <see cref="CheckSplittingVertices"/> adds the splitting vertices to a compound: the public form builds a
/// new frozen compound holding the caller's items and the new ones; ported code uses
/// <see cref="InternalCheckSplittingVertices"/>, which adds to the caller's (unfrozen) compound in place, as OCCT does.
/// </remarks>
public sealed class ShapeAnalysis_CheckSmallFace
{
    private int myStatus;
    private int myStatusSpot;
    private int myStatusStrip;
    private int myStatusPin;
    private int myStatusTwisted;
    private int myStatusSplitVert;
    private int myStatusPinFace;
    private int myStatusPinEdges;
    private double myPrecision;

    /// <summary>Creates an empty tool.</summary>
    public ShapeAnalysis_CheckSmallFace()
    {
        myStatusSpot = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        myStatusStrip = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        myStatusPin = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        myStatusTwisted = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        myStatusSplitVert = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        // OCCT leaves myStatus, myStatusPinFace, myStatusPinEdges and myPrecision uninitialised (occt-issues.md): the
        // port starts the statuses at OK and the precision at -1 (local tolerance zones, the documented default).
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        myStatusPinFace = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        myStatusPinEdges = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        myPrecision = -1.0;
    }

    private static void MinMaxPnt(in Point3D p, ref int nb, ref double minx, ref double miny, ref double minz,
                                  ref double maxx, ref double maxy, ref double maxz)
    {
        p.Coord(out double x, out double y, out double z);
        if (nb < 1)
        {
            minx = maxx = x;
            miny = maxy = y;
            minz = maxz = z;
        }
        else
        {
            if (minx > x)
            {
                minx = x;
            }
            if (maxx < x)
            {
                maxx = x;
            }
            if (miny > y)
            {
                miny = y;
            }
            if (maxy < y)
            {
                maxy = y;
            }
            if (minz > z)
            {
                minz = z;
            }
            if (maxz < z)
            {
                maxz = z;
            }
        }
        nb++;
    }

    private static bool MinMaxSmall(double minx, double miny, double minz, double maxx, double maxy, double maxz, double toler)
    {
        double dx = maxx - minx;
        double dy = maxy - miny;
        double dz = maxz - minz;

        return (dx <= toler || Kerf.Engine.Foundation.Precision.IsInfinite(dx)) && (dy <= toler || Kerf.Engine.Foundation.Precision.IsInfinite(dy))
               && (dz <= toler || Kerf.Engine.Foundation.Precision.IsInfinite(dz));
    }

    /// <summary>
    /// Checks if a face is a spot: 0 if not, 1 if yes, 2 if yes and all its vertices are the same. By default the
    /// tolerance zones of the vertices are considered; <paramref name="tol"/> gives a size instead. For a spot,
    /// <paramref name="spot"/> is its location and <paramref name="spotol"/> half the largest dimension of its box. A
    /// face without wires gives 1 with <paramref name="spot"/> and <paramref name="spotol"/> unchanged, as in OCCT.
    /// </summary>
    public int IsSpotFace(in Face F, ref Point3D spot, ref double spotol, double tol = -1.0)
    {
        double toler = tol;
        double tolv = tol;
        //  Compute tolerance to get : from greatest tol of vertices
        //  In addition, also computes min-max of vertices
        //  To finally compare mini-max box with tolerance
        // gka Mar2000 Protection against faces without wires
        // but they occur due to bugs in the algorithm itself, it needs to be fixed
        bool isWir = false;
        for (var itw = new Iterator(F, false); itw.More(); itw.Next())
        {
            if (itw.Value().ShapeType() != ShapeType.Wire)
            {
                continue;
            }
            Wire w1 = ShapeCasts.Wire(itw.Value());
            if (!w1.IsNull)
            {
                isWir = true;
                break;
            }
        }
        if (!isWir)
        {
            return 1; // OCCT returns true (1) for a face without wires (occt-issues.md)
        }
        int nbv = 0;
        double minx = 0, miny = 0, minz = 0, maxx = Kerf.Engine.Foundation.Precision.Infinite(), maxy = Kerf.Engine.Foundation.Precision.Infinite(),
               maxz = Kerf.Engine.Foundation.Precision.Infinite();
        Vertex V0 = new();
        bool same = true;
        for (var iv = new Explorer(F, ShapeType.Vertex); iv.More(); iv.Next())
        {
            Vertex V = ShapeCasts.Vertex(iv.Current());
            if (V0.IsNull)
            {
                V0 = V;
            }
            else if (same)
            {
                if (!V0.IsSame(V))
                {
                    same = false;
                }
            }

            Point3D pnt = Tool.Pnt(V);
            MinMaxPnt(pnt, ref nbv, ref minx, ref miny, ref minz, ref maxx, ref maxy, ref maxz);

            if (tol < 0)
            {
                tolv = Tool.Tolerance(V);
                if (tolv > toler)
                {
                    toler = tolv;
                }
            }
        }

        //   Now, testing
        if (!MinMaxSmall(minx, miny, minz, maxx, maxy, maxz, toler))
        {
            return 0;
        }

        //   All vertices are confused
        //   Check edges (a closed edge may be a non-null length edge !)
        //   By picking intermediate point on each one
        for (var ie = new Explorer(F, ShapeType.Edge); ie.More(); ie.Next())
        {
            Edge E = ShapeCasts.Edge(ie.Current());
            Curve3D? C3D = Tool.Curve(E, out double cf, out double cl);
            if (C3D is null)
            {
                continue;
            }
            Point3D debut = C3D.Value(cf);
            Point3D milieu = C3D.Value((cf + cl) / 2);
            if (debut.SquareDistance(milieu) > toler * toler)
            {
                return 0;
            }
        }

        spot.SetCoord((minx + maxx) / 2.0, (miny + maxy) / 2.0, (minz + maxz) / 2.0);
        spotol = maxx - minx;
        spotol = Max(spotol, maxy - miny);
        spotol = Max(spotol, maxz - minz);
        spotol = spotol / 2.0;

        return (same ? 2 : 1);
    }

    /// <summary>As <see cref="IsSpotFace"/>, recording the result in <see cref="StatusSpot"/> (DONE1, DONE2).</summary>
    public bool CheckSpotFace(in Face F, double tol = -1.0)
    {
        Point3D spot = new();
        double spotol = 0.0;
        int stat = IsSpotFace(F, ref spot, ref spotol, tol);
        if (stat == 0)
        {
            return false;
        }
        switch (stat)
        {
            case 1:
                myStatusSpot = ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
                break;
            case 2:
                myStatusSpot = ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
                break;
            default:
                break;
        }
        return true;
    }

    /// <summary>
    /// Checks if a face lies on a B-spline or Bezier surface that is a strip (by its poles), small in V
    /// (<see cref="StatusStrip"/> DONE2) or in U (DONE1). <paramref name="tol"/> is the largest width (1e-7 by default).
    /// </summary>
    public bool IsStripSupport(in Face F, double tol = -1.0)
    {
        double toler = tol;
        if (toler < 0)
        {
            toler = 1.0e-07; // ?? better to compute tolerance zones
        }

        Surface? surf = Tool.Surface(F, out Placement _);
        if (surf is null)
        {
            return false;
        }

        //  Checking on poles for bezier-bspline
        //  A more general way is to check Values by scanning ISOS (slower)

        var bs = surf as BSplineSurface;
        var bz = surf as BezierSurface;

        if (bs is not null || bz is not null)
        {
            bool cbz = (bz is not null);
            int iu, iv, nbu, nbv;
            if (cbz)
            {
                nbu = bz!.UPoleCount;
                nbv = bz.VPoleCount;
            }
            else
            {
                nbu = bs!.UPoleCount;
                nbv = bs.VPoleCount;
            }
            double minx = 0.0, miny = 0.0, minz = 0.0, maxx = 0.0, maxy = 0.0, maxz = 0.0;
            bool issmall = true;

            for (iu = 1; iu <= nbu; iu++)
            {
                //    for each U line, scan poles in V (V direction)
                int nb = 0;
                for (iv = 1; iv <= nbv; iv++)
                {
                    Point3D unp = (cbz ? bz!.Pole(iu, iv) : bs!.Pole(iu, iv));
                    MinMaxPnt(unp, ref nb, ref minx, ref miny, ref minz, ref maxx, ref maxy, ref maxz);
                }
                if (!MinMaxSmall(minx, miny, minz, maxx, maxy, maxz, toler))
                {
                    issmall = false;
                    break;
                } // small in V ?
            }
            if (issmall)
            {
                myStatusStrip = ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
                return issmall; // OK, small in V
            }
            issmall = true;
            for (iv = 1; iv <= nbv; iv++)
            {
                //    for each V line, scan poles in U (U direction)
                int nb = 0;
                for (iu = 1; iu <= nbu; iu++)
                {
                    Point3D unp = (cbz ? bz!.Pole(iu, iv) : bs!.Pole(iu, iv));
                    MinMaxPnt(unp, ref nb, ref minx, ref miny, ref minz, ref maxx, ref maxy, ref maxz);
                }
                if (!MinMaxSmall(minx, miny, minz, maxx, maxy, maxz, toler))
                {
                    issmall = false;
                    break;
                } // small in U ?
            }
            if (issmall)
            {
                myStatusStrip = ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
                return issmall;
            } // OK, small in U
        }

        return false;
    }

    /// <summary>
    /// Checks if two edges define a strip: the largest distance <paramref name="dmax"/> between them (sampled both
    /// ways) below <paramref name="tol"/>, or half the sum of the edges' tolerances when <paramref name="tol"/> &lt; 0.
    /// </summary>
    public bool CheckStripEdges(in Edge E1, in Edge E2, double tol, out double dmax)
    {
        //  We have the topological configuration OK : 2 edges, 2 vertices
        //  But, are these two edges well confused ?
        double toler = tol;
        if (tol < 0)
        {
            double tole = Tool.Tolerance(E1) + Tool.Tolerance(E2);
            if (toler < tole / 2.0)
            {
                toler = tole / 2.0;
            }
        }

        //   We project a list of points from each curve, on the opposite one,
        //   we check the distance
        int nbint = 10;

        ShapeAnalysis_Curve SAC = new();
        double u;
        dmax = 0;
        Curve3D? C1 = Tool.Curve(E1, out double cf1, out double cl1);
        Curve3D? C2 = Tool.Curve(E2, out double cf2, out double cl2);
        if (C1 is null || C2 is null)
        {
            return false;
        }
        cf1 = Max(cf1, C1.FirstParameter());
        cl1 = Min(cl1, C1.LastParameter());
        TrimmedCurve3D C1T = new(C1, cf1, cl1, true);
        // pdn protection against feature in Trimmed_Curve
        cf1 = C1T.FirstParameter();
        cl1 = C1T.LastParameter();
        TrimmedCurve3D CC;
        cf2 = Max(cf2, C2.FirstParameter());
        cl2 = Min(cl2, C2.LastParameter());
        TrimmedCurve3D C2T = new(C2, cf2, cl2, true);
        cf2 = C2T.FirstParameter();
        cl2 = C2T.LastParameter();

        double cd1 = (cl1 - cf1) / nbint;
        double cd2 = (cl2 - cf2) / nbint;
        double f, l;
        f = cf2;
        l = cl2;
        for (int numcur = 0; numcur < 2; numcur++)
        {
            u = cf1;
            if (numcur != 0)
            {
                CC = C1T;
                C1T = C2T;
                C2T = CC;
                cd1 = cd2; // smh added replacing step and replacing first
                u = cf2;   // parameter
                f = cf1;
                l = cl1;
            }
            for (int nump = 0; nump <= nbint; nump++)
            {
                Point3D p1 = C1T.Value(u);
                // pdn Adaptor curve is used to avoid of enhancing of domain.
                CurveAdaptor GAC = new(C2T);
                double dist = SAC.Project(GAC, p1, toler, out Point3D _, out double para);
                // pdn check if parameter of projection is in the domain of the edge.
                if (para < f || para > l)
                {
                    return false;
                }
                if (dist > dmax)
                {
                    dmax = dist;
                }
                if (dist > toler)
                {
                    return false;
                }
                u += cd1;
            }
        }
        return (dmax < toler);
    }

    /// <summary>
    /// Searches the face for two and only two edges that aren't small; true if they define a strip, with their largest
    /// distance in <paramref name="dmax"/> (unchanged when they aren't found or aren't checked).
    /// </summary>
    public bool FindStripEdges(in Face F, out Edge E1, out Edge E2, double tol, ref double dmax)
    {
        E1 = new Edge();
        E2 = new Edge();
        int nb = 0;
        for (var ex = new Explorer(F, ShapeType.Edge); ex.More(); ex.Next())
        {
            Edge E = ShapeCasts.Edge(ex.Current());
            if (nb == 1 && E.IsSame(E1))
            {
                continue; // ignore seam edge
            }
            Exploration.Vertices(E, out Vertex V1, out Vertex V2);
            Point3D p1, p2;
            p1 = Tool.Pnt(V1);
            p2 = Tool.Pnt(V2);
            double toler = tol;
            if (toler <= 0)
            {
                toler = (Tool.Tolerance(V1) + Tool.Tolerance(V2)) / 2.0;
            }

            //    Extremities
            double dist = p1.Distance(p2);
            //    Middle point
            Curve3D? CC = Tool.Curve(E, out double cf, out double cl);
            bool isNullLength = true;
            if (CC is not null)
            {
                Point3D pp = CC.Value((cf + cl) / 2.0);
                if (pp.Distance(p1) < toler && pp.Distance(p2) < toler)
                {
                    continue;
                }
                isNullLength = false;
            }
            if (dist <= toler && isNullLength)
            {
                continue; // smh
            }
            nb++;
            if (nb == 1)
            {
                E1 = E;
            }
            else if (nb == 2)
            {
                E2 = E;
            }
            else
            {
                return false;
            }
        }
        //   Now, check these two edge to define a strip !
        if (!E1.IsNull && !E2.IsNull)
        {
            if (!CheckStripEdges(E1, E2, tol, out dmax))
            {
                return false;
            }
            else
            {
                myStatusStrip = ShapeExtend.EncodeStatus(ShapeExtend_DONE3);
                return true;
            }
        }
        return false;
    }

    /// <summary>
    /// Checks if a face is a single strip: two vertices, two great edges confused along their whole length, possibly
    /// small edges. Records <see cref="StatusStrip"/> DONE3. <paramref name="E1"/> and <paramref name="E2"/> get the
    /// great edges as they are found (unchanged where OCCT doesn't write them).
    /// </summary>
    public bool CheckSingleStrip(in Face F, ref Edge E1, ref Edge E2, double tol = -1.0)
    {
        double toler = tol;
        double minx = 0.0, miny = 0.0, minz = 0.0, maxx = 0.0, maxy = 0.0, maxz = 0.0;

        // In this case, we have 2 vertices and 2 great edges. Plus possibly 2 small
        //    edges, one on each vertex
        Vertex V1 = new(), V2 = new();
        int nb = 0;
        for (var itv = new Explorer(F, ShapeType.Vertex); itv.More(); itv.Next())
        {
            Vertex V = ShapeCasts.Vertex(itv.Current());
            if (V1.IsNull)
            {
                V1 = V;
            }
            else if (V1.IsSame(V))
            {
                continue;
            }
            else if (V2.IsNull)
            {
                V2 = V;
            }
            else if (V2.IsSame(V))
            {
                continue;
            }
            else
            {
                return false;
            }
        }

        // Checking edges
        nb = 0;
        for (var ite = new Explorer(F, ShapeType.Edge); ite.More(); ite.Next())
        {
            Edge E = ShapeCasts.Edge(ite.Current());
            if (nb == 1 && E.IsSame(E1))
            {
                continue; // ignore seam edge
            }
            Exploration.Vertices(E, out Vertex VA, out Vertex VB);
            if (tol < 0)
            {
                double tolv;
                tolv = Tool.Tolerance(VA);
                if (toler < tolv)
                {
                    toler = tolv;
                }
                tolv = Tool.Tolerance(VB);
                if (toler < tolv)
                {
                    toler = tolv;
                }
            }

            //    Edge on same vertex : small one ?
            if (VA.IsSame(VB))
            {
                double cf = 0.0, cl = 0.0;
                Curve3D? C3D = null;
                if (!Tool.Degenerated(E))
                {
                    C3D = Tool.Curve(E, out cf, out cl);
                }
                if (C3D is null)
                {
                    continue; // DGNR
                }
                int np = 0;
                Point3D deb = C3D.Value(cf);
                MinMaxPnt(deb, ref np, ref minx, ref miny, ref minz, ref maxx, ref maxy, ref maxz);
                Point3D fin = C3D.Value(cl);
                MinMaxPnt(fin, ref np, ref minx, ref miny, ref minz, ref maxx, ref maxy, ref maxz);
                Point3D mid = C3D.Value((cf + cl) / 2.0);
                MinMaxPnt(mid, ref np, ref minx, ref miny, ref minz, ref maxx, ref maxy, ref maxz);
                if (!MinMaxSmall(minx, miny, minz, maxx, maxy, maxz, toler))
                {
                    return false;
                }
            }
            else
            {
                //    Other case : two maximum allowed
                nb++;
                if (nb > 2)
                {
                    return false;
                }
                if (nb == 1)
                {
                    V1 = VA;
                    V2 = VB;
                    E1 = E;
                }
                else if (nb == 2)
                {
                    if (V1.IsSame(VA) && !V2.IsSame(VB))
                    {
                        return false;
                    }
                    if (V1.IsSame(VB) && !V2.IsSame(VA))
                    {
                        return false;
                    }
                    E2 = E;
                }
                else
                {
                    return false;
                }
            }
        }

        if (nb < 2)
        {
            return false; // only one vertex : cannot be a strip ...
        }

        //   Checking if E1 and E2 define a Strip
        if (!CheckStripEdges(E1, E2, tol, out double _))
        {
            return false;
        }
        myStatusStrip = ShapeExtend.EncodeStatus(ShapeExtend_DONE3);
        return true;
    }

    /// <summary>
    /// Checks if a face is a strip: <see cref="CheckSingleStrip"/>, then <see cref="FindStripEdges"/>. The two edges
    /// delimiting it are returned in <paramref name="E1"/> and <paramref name="E2"/>.
    /// </summary>
    public bool CheckStripFace(in Face F, ref Edge E1, ref Edge E2, double tol = -1.0)
    {
        if (CheckSingleStrip(F, ref E1, ref E2, tol))
        {
            return true; // it is a strip
        }

        //    IsStripSupport used as rejection. But this kind of test may be done
        //    on ANY face, once we are SURE that FindStripEdges is reliable (and fast
        //    enough)
        double dmax = 0.0;
        return FindStripEdges(F, out E1, out E2, tol, ref dmax);
    }

    /// <summary>
    /// Finds the vertices of a face that split one of its edges (confused with an inner point of a non-adjacent edge).
    /// For each, <paramref name="MapEdges"/> binds the split edges and <paramref name="MapParam"/> the parameters, and
    /// the vertex is added to the compound: <paramref name="theAllVert"/> becomes a new frozen compound holding its
    /// previous items (if not null) and the splitting vertices. Returns the count of splitting vertices.
    /// </summary>
    public int CheckSplittingVertices(in Face F,
                                      NCollection_DataMap<Shape, NCollection_List<Shape>> MapEdges,
                                      NCollection_DataMap<Shape, NCollection_List<double>> MapParam,
                                      ref Compound theAllVert)
    {
        // OCCT.NET: OCCT adds to the caller's compound in place; the public form adds to a new compound holding the
        // caller's items, and publishes it frozen (design 4.6).
        BrepBuilder aB = new();
        Compound aCompound = new();
        aB.MakeCompound(ref aCompound);
        if (!theAllVert.IsNull)
        {
            for (var anIt = new Iterator(theAllVert, false, false); anIt.More(); anIt.Next())
            {
                aB.Add(aCompound, anIt.Value());
            }
        }
        int aNb = InternalCheckSplittingVertices(F, MapEdges, MapParam, aCompound);
        aCompound.TShape()!.FreezeGraph();
        theAllVert = aCompound;
        return aNb;
    }

    /// <summary>
    /// <see cref="CheckSplittingVertices"/> as OCCT writes it: adds the splitting vertices to
    /// <paramref name="theAllVert"/> in place (throws on a frozen compound). For ported code that owns the compound.
    /// </summary>
    internal int InternalCheckSplittingVertices(in Face F,
                                                NCollection_DataMap<Shape, NCollection_List<Shape>> MapEdges,
                                                NCollection_DataMap<Shape, NCollection_List<double>> MapParam,
                                                in Compound theAllVert)
    {
        //  Prepare array of vertices with their locations //TopTools
        int nbv = 0, nbp = 0;
        BrepBuilder theBuilder = new();
        Explorer itv = new(); // svv Jan11 2000 : porting on DEC
        for (itv.Init(F, ShapeType.Vertex); itv.More(); itv.Next())
        {
            nbv++;
        }

        if (nbv == 0)
        {
            return 0;
        }
        NCollection_Array1<Shape> vtx = new(1, nbv);
        NCollection_Array1<Point3D> vtp = new(1, nbv);
        NCollection_Array1<double> vto = new(1, nbv);

        nbp = 0;
        for (itv.Init(F, ShapeType.Vertex); itv.More(); itv.Next())
        {
            nbp++;
            Vertex unv = ShapeCasts.Vertex(itv.Current());
            vtx.SetValue(nbp, unv);
            Point3D unp = Tool.Pnt(unv);
            vtp.SetValue(nbp, unp);
            double unt = myPrecision;
            if (unt < 0)
            {
                unt = Tool.Tolerance(unv);
            }
            vto.SetValue(nbp, unt);
        }
        nbv = nbp;
        nbp = 0; // now, counting splitting vertices

        //  Check edges : are vertices (other than extremities) confused with it ?
        ShapeAnalysis_Curve SAC = new();
        for (int iv = 1; iv <= nbv; iv++)
        {
            Vertex V = ShapeCasts.Vertex(vtx.Value(iv));
            NCollection_List<Shape> listEdge = new();
            NCollection_List<double> listParam = new();
            bool issplit = false;
            for (var ite = new Explorer(F, ShapeType.Edge); ite.More(); ite.Next())
            {
                Edge E = ShapeCasts.Edge(ite.Current());
                Exploration.Vertices(E, out Vertex V1, out Vertex V2);
                Curve3D? C3D = Tool.Curve(E, out double cf, out double cl);
                if (C3D is null)
                {
                    continue;
                }
                if (V.IsSame(V1) || V.IsSame(V2))
                {
                    continue;
                }
                Point3D unp = vtp.Value(iv);
                double unt = vto.Value(iv);
                double dist = SAC.Project(C3D, unp, unt * 10.0, out Point3D _, out double param, cf, cl);
                if (dist == 0.0)
                {
                    continue; // smh
                }
                //  Splitting Vertex to record ?
                if (dist < unt)
                {
                    //  If Split occurs at beginning or end, it is not a split ...
                    double fpar, lpar, eps = 1.0e-06;
                    if (param >= cl || param <= cf)
                    {
                        continue; // Out of range
                    }
                    fpar = param - cf;
                    lpar = param - cl;
                    if ((Math.Abs(fpar) < eps) || (Math.Abs(lpar) < eps))
                    {
                        continue; // Near end or start
                    }
                    listEdge.Append(E);
                    listParam.Append(param);
                    issplit = true;
                }
            }
            if (issplit)
            {
                nbp++;
                theBuilder.Add(theAllVert, V);
                MapEdges.Bind(V, listEdge);
                MapParam.Bind(V, listParam);
            }
        }
        if (nbp != 0)
        {
            myStatusSplitVert = ShapeExtend.EncodeStatus(ShapeExtend_DONE);
        }
        return nbp;
    }

    private static int IsoStat(NCollection_Array2<Point3D> poles, int uorv, int rank, double tolpin, double toler)
    {
        int i, np = 0;
        int i0 = (uorv == 1 ? poles.LowerCol() : poles.LowerRow());
        int i1 = (uorv == 1 ? poles.UpperCol() : poles.UpperRow());
        double xmin = 0.0, ymin = 0.0, zmin = 0.0, xmax = 0.0, ymax = 0.0, zmax = 0.0;
        for (i = i0; i <= i1; i++)
        {
            if (uorv == 1)
            {
                MinMaxPnt(poles.Value(rank, i), ref np, ref xmin, ref ymin, ref zmin, ref xmax, ref ymax, ref zmax);
            }
            else
            {
                MinMaxPnt(poles.Value(i, rank), ref np, ref xmin, ref ymin, ref zmin, ref xmax, ref ymax, ref zmax);
            }
        }
        if (MinMaxSmall(xmin, ymin, zmin, xmax, ymax, zmax, tolpin))
        {
            return 0;
        }
        if (MinMaxSmall(xmin, ymin, zmin, xmax, ymax, zmax, toler))
        {
            return 1;
        }
        return 2;
    }

    private static bool CheckPoles(NCollection_Array2<Point3D> poles, int uorv, int rank)
    {
        int i0 = (uorv == 1 ? poles.LowerCol() : poles.LowerRow());
        int i1 = (uorv == 1 ? poles.UpperCol() : poles.UpperRow());
        for (int i = i0; i <= i1 - 1; i++)
        {
            if (uorv == 1)
            {
                if (poles.Value(rank, i).IsEqual(poles.Value(rank, i + 1), 1e-15))
                {
                    return true;
                }
            }
            else if (poles.Value(i, rank).IsEqual(poles.Value(i + 1, rank), 1e-15))
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>
    /// Checks if a face on a B-spline or Bezier surface has a pin (a natural bound whose poles are confused):
    /// <paramref name="sence"/> 1 in U, 2 in V, and the row in <paramref name="whatrow"/> (both unchanged for an
    /// elementary or other surface). <see cref="StatusPin"/>: DONE1 smooth pin, DONE2 sharp, DONE3 equal poles.
    /// </summary>
    public bool CheckPin(in Face F, ref int whatrow, ref int sence)
    {
        Surface? surf = Tool.Surface(F, out Placement _);
        // OCCT dereferences a null surface here; the port throws NullReferenceException.
        if (surf!.IsKind(typeof(ElementarySurface)))
        {
            return false;
        }

        double toler = myPrecision;
        if (toler < 0)
        {
            toler = 1.0e-4;
        }
        double tolpin = 1.0e-9; // for sharp sharp pin

        //  Checking the poles

        //  Take the poles : they give good idea of sharpness of a pin
        int nbu = 0, nbv = 0;
        var bs = surf as BSplineSurface;
        var bz = surf as BezierSurface;
        if (bs is not null)
        {
            nbu = bs.UPoleCount;
            nbv = bs.VPoleCount;
        }
        if (bz is not null)
        {
            nbu = bz.UPoleCount;
            nbv = bz.VPoleCount;
        }
        if (nbu == 0 || nbv == 0)
        {
            return false;
        }

        NCollection_Array2<Point3D> allpoles = null!;
        if (bs is not null)
        {
            allpoles = bs.InternalPoles();
        }
        if (bz is not null)
        {
            allpoles = bz.InternalPoles();
        }

        //  Check each natural bound if it is a singularity (i.e. a pin)

        sence = 0;
        int stat = 0; // 0 none, 1 in U, 2 in V
        whatrow = 0;  // 0 no row, else rank of row
        stat = IsoStat(allpoles, 1, 1, tolpin, toler);
        if (stat != 0)
        {
            sence = 1;
            whatrow = nbu; // OCCT: row 1 is reported as row nbu (occt-issues.md)
        }

        stat = IsoStat(allpoles, 1, nbu, tolpin, toler);
        if (stat != 0)
        {
            sence = 1;
            whatrow = nbu;
        }

        stat = IsoStat(allpoles, 2, 1, tolpin, toler);
        if (stat != 0)
        {
            sence = 2;
            whatrow = 1;
        }

        stat = IsoStat(allpoles, 2, nbv, tolpin, toler);
        if (stat != 0)
        {
            sence = 2;
            whatrow = nbv;
        }

        if (sence == 0)
        {
            return false; // no pin
        }

        // OCCT: the status is taken from the last bound checked only (occt-issues.md)
        switch (stat)
        {
            case 1:
                myStatusPin = ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
                break;
            case 2:
                myStatusPin = ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
                break;
            default:
                break;
        }
        if (stat == 1)
        {
            if (CheckPoles(allpoles, 2, nbv) || CheckPoles(allpoles, 2, 1) || CheckPoles(allpoles, 1, nbu)
                || CheckPoles(allpoles, 1, 1))
            {
                myStatusPin = ShapeExtend.EncodeStatus(ShapeExtend_DONE3);
            }
        }

        return true;
    }

    private static double TwistedNorm(double x1, double y1, double z1, double x2, double y2, double z2)
    {
        return (x1 * x2) + (y1 * y2) + (z1 * z2);
    }

    /// <summary>
    /// Checks if the support of a face is twisted (two normals of a 5 x 5 grid at more than 90 degrees); then
    /// <paramref name="paramu"/> and <paramref name="paramv"/> locate it (unchanged otherwise).
    /// </summary>
    public bool CheckTwisted(in Face F, ref double paramu, ref double paramv)
    {
        Surface? surf = Tool.Surface(F, out Placement _);
        // OCCT dereferences a null surface here; the port throws NullReferenceException.
        if (surf!.IsKind(typeof(ElementarySurface)))
        {
            return false;
        }

        double toler = myPrecision;
        if (toler < 0)
        {
            toler = 1.0e-4;
        }
        _ = toler; // OCCT computes it for a commented-out GeomLProp_SLProps
        SurfaceAdaptor GAS = new(surf);

        // to be done : on isos of the surface
        //  and on edges, at least of outer wire
        int nbint = 5;
        NCollection_Array2<double> nx = new(1, nbint + 1, 1, nbint + 1);
        NCollection_Array2<double> ny = new(1, nbint + 1, 1, nbint + 1);
        NCollection_Array2<double> nz = new(1, nbint + 1, 1, nbint + 1);
        int iu, iv;
        surf.Bounds(out double umin, out double umax, out double vmin, out double vmax);
        double u = umin, du = (umax - umin) / nbint;
        double v = vmin, dv = (umax - umin) / nbint; // OCCT: the V step is computed from the U range (occt-issues.md)

        for (iu = 1; iu <= nbint; iu++)
        {
            for (iv = 1; iv <= nbint; iv++)
            {
                GAS.D1(u, v, out Point3D _, out Vector3D V1, out Vector3D V2);
                Vector3D VXnorm = V1.Crossed(V2);
                nx.SetValue(iu, iv, VXnorm.X());
                ny.SetValue(iu, iv, VXnorm.Y());
                nz.SetValue(iu, iv, VXnorm.Z());
                v += dv;
            }
            u += du;
            v = vmin;
        }

        //  Now, comparing normals on support surface, in both senses
        //  In principle, it suffuces to check within outer bound

        for (iu = 1; iu < nbint; iu++)
        {
            for (iv = 1; iv < nbint; iv++)
            {
                // We here check each normal (iu,iv) with (iu,iv+1) and with (iu+1,iv)
                // if for each test, we have negative scalar product, this means angle > 90deg
                // it is the criterion to say it is twisted
                if (TwistedNorm(nx[iu, iv], ny[iu, iv], nz[iu, iv], nx[iu, iv + 1], ny[iu, iv + 1], nz[iu, iv + 1]) < 0.0
                    || TwistedNorm(nx[iu, iv], ny[iu, iv], nz[iu, iv], nx[iu + 1, iv], ny[iu + 1, iv], nz[iu + 1, iv]) < 0.0)
                {
                    myStatusTwisted = ShapeExtend.EncodeStatus(ShapeExtend_DONE);
                    paramu = umin + du * iu - du / 2;
                    paramv = vmin + dv * iv - dv / 2;
                    return true;
                }
            }
        }

        //   Now, comparing normals on edges ... to be done

        return false;
    }

    /// <summary>
    /// Finds pairs of consecutive pin edges of the face (short edges, by their vertices, folded on each other) and
    /// binds them in <paramref name="mapEdges"/> (the caller's working map). OCCT: "not tested on many examples".
    /// </summary>
    public bool CheckPinFace(in Face F, NCollection_DataMap<Shape, Shape> mapEdges, double toler = -1.0)
    {
        Explorer exp_w = new(F, ShapeType.Wire);
        exp_w.More();
        double coef1 = 0, coef2; // =0 for deleting warning (skl)
        Wire theCurWire = ShapeCasts.Wire(exp_w.Current());
        ShapeAnalysis_WireOrder wi = new();
        ShapeAnalysis_Wire sfw = new();
        ShapeExtend_WireData sbwd = new(theCurWire);
        sfw.Load(sbwd);
        sfw.CheckOrder(wi);
        NCollection_HSequence<Shape> newedges = new();
        int nb = wi.NbEdges();
        int i = 0;
        for (i = 1; i <= nb; i++)
        {
            newedges.Append(sbwd.Edge(wi.Ordered(i)));
        }
        for (i = 1; i <= nb; i++)
        {
            sbwd.Set(ShapeCasts.Edge(newedges.Value(i)), i);
        }
        // OCCT rebuilds the reordered wire and never uses it (the edges below are the face's).
        theCurWire = sbwd.InternalWire();
        _ = theCurWire;
        i = 1;
        bool done = false;
        double tol = Kerf.Engine.Foundation.Precision.Confusion();
        Edge theFirstEdge = new(), theSecondEdge;
        double d1 = 0, d2 = 0;
        for (var exp_e = new Explorer(F, ShapeType.Edge); exp_e.More(); exp_e.Next())
        {
            Vertex V1, V2;
            Point3D p1, p2;
            if (i == 1)
            {
                theFirstEdge = ShapeCasts.Edge(exp_e.Current());
                V1 = Exploration.FirstVertex(theFirstEdge);
                V2 = Exploration.LastVertex(theFirstEdge);
                p1 = Tool.Pnt(V1);
                p2 = Tool.Pnt(V2);
                tol = Max(Tool.Tolerance(V1), Tool.Tolerance(V2));
                if (toler > 0)
                { // tol = std::max(tol, toler); gka
                    tol = toler;
                }
                d1 = p1.Distance(p2);
                if (d1 == 0)
                {
                    return false;
                }
                if (d1 / tol >= 1)
                {
                    coef1 = d1 / tol;
                }
                else
                {
                    continue;
                }
                if (coef1 <= 3)
                {
                    continue;
                }
                i++;
                continue;
            }
            // Check the length of edge
            theSecondEdge = ShapeCasts.Edge(exp_e.Current());
            V1 = Exploration.FirstVertex(theSecondEdge);
            V2 = Exploration.LastVertex(theSecondEdge);

            p1 = Tool.Pnt(V1);
            p2 = Tool.Pnt(V2);
            if (toler == -1)
            {
                tol = Max(Tool.Tolerance(V1), Tool.Tolerance(V2));
            }
            else
            {
                tol = toler;
            }
            if (p1.Distance(p2) > tol)
            {
                continue;
            }
            // If there are two pin edges, record them in diagnostic
            d2 = p1.Distance(p2); // gka
            if (d2 == 0)
            {
                return false;
            }
            if (d2 / tol >= 1)
            {
                coef2 = d2 / tol;
            }
            else
            {
                continue;
            }
            if (coef2 <= 3)
            {
                continue;
            }
            if (coef1 > coef2 * 10)
            {
                continue;
            }
            if (coef2 > coef1 * 10)
            {
                theFirstEdge = theSecondEdge;
                coef1 = coef2;
                continue;
            }

            if (CheckPinEdges(theFirstEdge, theSecondEdge, coef1, coef2, toler))
            {
                mapEdges.Bind(theFirstEdge, theSecondEdge);
                myStatusPinFace = ShapeExtend.EncodeStatus(ShapeExtend_DONE);
                done = true;
            }

            theFirstEdge = theSecondEdge;
            coef1 = coef2;
        }
        return done;
    }

    /// <summary>
    /// Checks if two edges sharing the first one's last vertex are pin edges (folded on each other near the vertex,
    /// with tangent derivatives). OCCT: "not tested on many examples".
    /// </summary>
    public bool CheckPinEdges(in Edge theFirstEdge, in Edge theSecondEdge, double coef1, double coef2, double toler)
    {
        Curve3D? C1, C2, C3;
        C1 = Tool.Curve(theFirstEdge, out double cf1, out double cl1);
        C2 = Tool.Curve(theSecondEdge, out double cf2, out double cl2);
        Point3D p1, p2, pp1, pp2, pv;
        double d1 = (cf1 - cl1) / coef1;
        double d2 = (cf2 - cl2) / coef2;
        // OCCT dereferences null curves here; the port throws NullReferenceException.
        p1 = C1!.Value(cf1);
        p2 = C1.Value(cl1);
        pp1 = C2!.Value(cf2);
        pp2 = C2.Value(cl2);
        double tol;
        double paramc1 = 0, paramc2 = 0; // =0 for deleting warning (skl)
        Vertex theSharedV = Exploration.LastVertex(theFirstEdge);
        if (toler == -1)
        {
            tol = Tool.Tolerance(theSharedV);
        }
        else
        {
            tol = toler;
        }
        pv = Tool.Pnt(theSharedV);
        if (pv.Distance(p1) <= tol)
        {
            paramc1 = cf1;
        }
        else if (pv.Distance(p2) <= tol)
        {
            paramc1 = cl1;
        }
        if (pv.Distance(pp1) <= tol)
        {
            paramc2 = cf2;
        }
        else if (pv.Distance(pp2) <= tol)
        {
            paramc2 = cl2;
        }
        Point3D proj;
        if (p1.Distance(p2) < pp1.Distance(pp2))
        {
            C3 = C1;
            if (paramc1 == cf1)
            {
                proj = C1.Value(paramc1 + (coef1 - 3) * d1);
            }
            else
            {
                proj = C1.Value(paramc1 - 3 * d1);
            }
        }
        else
        {
            C3 = C2;
            if (paramc2 == cf2)
            {
                proj = C2.Value(paramc2 + (coef2 - 3) * d2);
            }
            else
            {
                proj = C2.Value(paramc2 - 3 * d2);
            }
        }
        CurveAdaptor GAC = new(C3);
        double f = C3.FirstParameter();
        double l = C3.LastParameter();
        ShapeAnalysis_Curve SAC = new();
        double dist = SAC.Project(GAC, proj, tol, out Point3D _, out double param);
        // pdn check if parameter of projection is in the domain of the edge.
        if (param < f || param > l)
        {
            return false;
        }
        if (dist > tol)
        {
            return false;
        }
        if (dist <= tol)
        {
            // Computing first derivative vectors and compare angle
            C1.D2(paramc1, out Point3D _, out Vector3D V11, out Vector3D V21);
            C2.D2(paramc2, out Point3D _, out Vector3D V12, out Vector3D V22);
            double angle1 = 0, angle2 = 0;
            try
            {
                angle1 = V11.Angle(V12);
                angle2 = V21.Angle(V22);
            }
            catch (FailureException)
            {
                return false;
            }
            // OCCT tests (M_PI - angle2) twice where the second test is likely meant for angle1 (occt-issues.md)
            return (angle1 <= 0.001 && angle2 <= 0.01)
                   || ((M_PI - angle2) <= 0.001 && (M_PI - angle2) <= 0.01);
        }

        return false;
    }

    /// <summary>The status of the last call to Perform (no ported method sets it; always OK).</summary>
    public bool Status(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatus, status);

    /// <summary>Sets a fixed tolerance to check small faces; a negative value considers the local tolerance zones.</summary>
    public void SetTolerance(double tol)
    {
        myPrecision = tol;
    }

    /// <summary>The tolerance to check small faces; negative if the local tolerance zones are considered.</summary>
    public double Tolerance() => myPrecision;

    /// <summary>The status of the spot check.</summary>
    public bool StatusSpot(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatusSpot, status);

    /// <summary>The status of the strip checks.</summary>
    public bool StatusStrip(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatusStrip, status);

    /// <summary>The status of the pin check.</summary>
    public bool StatusPin(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatusPin, status);

    /// <summary>The status of the twisted check.</summary>
    public bool StatusTwisted(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatusTwisted, status);

    /// <summary>The status of the splitting-vertices check.</summary>
    public bool StatusSplitVert(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatusSplitVert, status);

    /// <summary>The status of the pin-face check.</summary>
    public bool StatusPinFace(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatusPinFace, status);

    /// <summary>The status of the pin-edges check (no ported method sets it).</summary>
    public bool StatusPinEdges(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatusPinEdges, status);
}
