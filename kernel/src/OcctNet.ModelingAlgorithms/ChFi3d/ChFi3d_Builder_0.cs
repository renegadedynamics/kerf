// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder_0.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder_0.cxx (to ChFi3d_IsInFront; the rest in the other
//          ChFi3d_Builder_0.*.cs files)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using static Occt.ChFiDS_State;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>The fillet builders' free functions (<c>ChFi3d_*</c>), as a static class named after the file.</summary>
/// <remarks>
/// The whole of <c>ChFi3d_Builder_0.cxx</c> (unit 7N1; <c>ChFi3d_Coefficient</c> came first with unit 7E). Its
/// <c>OCCT_DEBUG</c> and <c>DRAW</c> blocks (traces, chronometers, the <c>ChFi3d_Settrace*</c> flags) are left out.
/// The functions write only into the builder's own data: the data structure (<see cref="TopOpeBRepDS_DataStructure"/>),
/// the stripes, surface data and spines, and the geometry they make. None writes into a shape of the input: the
/// vertices whose tolerance the builder raises are updated by the builder itself (<c>ChFi3d_Builder</c>, unit 7N2),
/// not here (decision 1087).
/// </remarks>
internal static partial class ChFi3d_Builder_0
{
    //=================================================================================================

    /// <summary><paramref name="U"/> brought into [<paramref name="UFirst"/>, <paramref name="ULast"/>], up to
    /// <paramref name="Eps"/>.</summary>
    internal static double ChFi3d_InPeriod(double U, double UFirst, double ULast, double Eps)
    {
        double period = ULast - UFirst;
        double u = U;
        while (Eps < (UFirst - u))
        {
            u += period;
        }
        while (Eps > (ULast - u))
        {
            u -= period;
        }
        if (u < UFirst)
        {
            u = UFirst;
        }
        return u;
    }

    // namespace { ChFi3d_BaseSurface, ChFi3d_ApplyBounds, ChFi3d_BoundSrfImpl }: the template's two instances are
    // written out through the adaptor's own surface and loading.

    private static Geom_Surface? ChFi3d_BaseSurface(GeomAdaptor_Surface theSurface) => theSurface.Surface();

    private static Geom_Surface? ChFi3d_BaseSurface(BRepAdaptor_Surface theSurface) =>
        theSurface.GeomSurfaceOriginal();

    private static void ChFi3d_ApplyBounds(GeomAdaptor_Surface theSurface, Geom_Surface theGeomSurface,
                                           double theUFirst, double theULast, double theVFirst, double theVLast)
    {
        theSurface.Load(theGeomSurface, theUFirst, theULast, theVFirst, theVLast, theSurface.ToleranceU(),
                        theSurface.ToleranceV());
    }

    private static void ChFi3d_ApplyBounds(BRepAdaptor_Surface theSurface, Geom_Surface theGeomSurface,
                                           double theUFirst, double theULast, double theVFirst, double theVLast)
    {
        theSurface.Load(theGeomSurface, theUFirst, theULast, theVFirst, theVLast, theSurface.Trsf(),
                        theSurface.ToleranceU(), theSurface.ToleranceV());
    }

    private static void ChFi3d_BoundSrfImpl(Adaptor3d_Surface theSurface, Geom_Surface? theBase, double theUmin,
                                            double theUmax, double theVmin, double theVmax,
                                            bool theCheckNaturalBounds, out Geom_Surface aSurface,
                                            out double aUU1, out double aUU2, out double aVV1, out double aVV2)
    {
        double aUmin = theUmin;
        double aUmax = theUmax;
        double aVmin = theVmin;
        double aVmax = theVmax;
        aSurface = theBase!;
        if (aSurface is Geom_RectangularTrimmedSurface aTrimmedSurface)
        {
            aSurface = aTrimmedSurface.BasisSurface();
        }

        aSurface.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
        double aPeriodU = 0.0;
        double aPeriodV = 0.0;
        if (aSurface.IsUPeriodic())
        {
            aPeriodU = aSurface.UPeriod();
        }
        if (aSurface.IsVPeriodic())
        {
            aPeriodV = aSurface.VPeriod();
        }

        double aStepU = aUmax - aUmin;
        double aStepV = aVmax - aVmin;
        double aScaleU = theSurface.UResolution(1.0);
        double aScaleV = theSurface.VResolution(1.0);
        double aStep3dU = aStepU / aScaleU;
        double aStep3dV = aStepV / aScaleV;

        if (aStep3dU > aStep3dV)
        {
            aStepV = aStep3dU * aScaleV;
        }
        if (aStep3dV > aStep3dU)
        {
            aStepU = aStep3dV * aScaleU;
        }
        if (aPeriodU > 0.0)
        {
            aStepU = 0.1 * (aPeriodU - (aUmax - aUmin));
        }
        if (aPeriodV > 0.0)
        {
            aStepV = 0.1 * (aPeriodV - (aVmax - aVmin));
        }

        aUU1 = aUmin - aStepU;
        aUU2 = aUmax + aStepU;
        aVV1 = aVmin - aStepV;
        aVV2 = aVmax + aStepV;
        if (theCheckNaturalBounds)
        {
            if (!theSurface.IsUPeriodic())
            {
                aUU1 = Math.Max(aUU1, aU1);
                aUU2 = Math.Min(aUU2, aU2);
            }
            if (!theSurface.IsVPeriodic())
            {
                aVV1 = Math.Max(aVV1, aV1);
                aVV2 = Math.Min(aVV2, aV2);
            }
        }
    }

    //=======================================================================
    // function : Box
    // purpose  : Calculation of min/max uv of the fillet to intersect.
    //=======================================================================

    /// <summary>The bounds in u and v of the two points.</summary>
    internal static void ChFi3d_Boite(in Point2D p1, in Point2D p2, out double mu, out double Mu, out double mv,
                                      out double Mv)
    {
        mu = Math.Min(p1.X(), p2.X());
        Mu = Math.Max(p1.X(), p2.X());
        mv = Math.Min(p1.Y(), p2.Y());
        Mv = Math.Max(p1.Y(), p2.Y());
    }

    //=======================================================================
    // function : Box
    // purpose  : Calculation of min/max uv of the fillet to intersect.
    //=======================================================================

    /// <summary>The bounds in u and v of the four points, and their spans.</summary>
    internal static void ChFi3d_Boite(in Point2D p1, in Point2D p2, in Point2D p3, in Point2D p4, out double Du,
                                      out double Dv, out double mu, out double Mu, out double mv, out double Mv)
    {
        double a, b;
        a = Math.Min(p1.X(), p2.X());
        b = Math.Min(p3.X(), p4.X());
        mu = Math.Min(a, b);
        a = Math.Max(p1.X(), p2.X());
        b = Math.Max(p3.X(), p4.X());
        Mu = Math.Max(a, b);
        a = Math.Min(p1.Y(), p2.Y());
        b = Math.Min(p3.Y(), p4.Y());
        mv = Math.Min(a, b);
        a = Math.Max(p1.Y(), p2.Y());
        b = Math.Max(p3.Y(), p4.Y());
        Mv = Math.Max(a, b);
        Du = Mu - mu;
        Dv = Mv - mv;
    }

    //=================================================================================================

    private static Adaptor3d_Surface? Geometry(TopOpeBRepDS_DataStructure DStr, int ind)
    {
        if (ind == 0)
        {
            return null;
        }
        if (ind > 0)
        {
            TopoDS_Face F = TopoDS.Face(DStr.Shape(ind));
            if (F.IsNull())
            {
                return null;
            }
            BRepAdaptor_Surface HS = new();
            HS.Initialize(F, false);
            return HS;
        }
        else
        {
            Geom_Surface? S = DStr.Surface(-ind).Surface();
            if (S is null)
            {
                return null;
            }
            return new GeomAdaptor_Surface(S);
        }
    }

    //=================================================================================================

    /// <summary>Sets the tolerance of the point <paramref name="IP"/> to half again the diagonal of
    /// <paramref name="box"/>.</summary>
    internal static void ChFi3d_SetPointTolerance(TopOpeBRepDS_DataStructure DStr, in Box box, int IP)
    {
        double a, b, c, d, e, f, vtol;
        box.Get(out a, out b, out c, out d, out e, out f);
        d -= a;
        e -= b;
        f -= c;
        d *= d;
        e *= e;
        f *= f;
        vtol = Math.Sqrt(d + e + f) * 1.5; // on prend un petit rab.
        DStr.ChangePoint(IP).Tolerance(vtol);
    }

    //=================================================================================================

    /// <summary>Adds the points of <paramref name="C"/> at <paramref name="wd"/> and <paramref name="wf"/> to the
    /// boxes.</summary>
    internal static void ChFi3d_EnlargeBox(Geom_Curve C, double wd, double wf, ref Box box1, ref Box box2)
    {
        box1.Add(C.Value(wd));
        box2.Add(C.Value(wf));
    }

    //=================================================================================================

    /// <summary>Adds the points of <paramref name="PC"/> on <paramref name="S"/> at <paramref name="wd"/> and
    /// <paramref name="wf"/> to the boxes.</summary>
    internal static void ChFi3d_EnlargeBox(Adaptor3d_Surface S, Geom2d_Curve PC, double wd, double wf,
                                           ref Box box1, ref Box box2)
    {
        double u, v;
        PC.Value(wd).Coord(out u, out v);
        box1.Add(S.Value(u, v));
        PC.Value(wf).Coord(out u, out v);
        box2.Add(S.Value(u, v));
    }

    //=================================================================================================

    /// <summary>Adds the point of <paramref name="E"/> at <paramref name="w"/>, by its 3D curve and its pcurves on
    /// the faces <paramref name="LF"/>, to the box.</summary>
    internal static void ChFi3d_EnlargeBox(in TopoDS_Edge E, NCollection_List<TopoDS_Shape> LF, double w,
                                           ref Box box)
    {
        BRepAdaptor_Curve BC = new(E);
        box.Add(BC.Value(w));
        foreach (TopoDS_Shape It in LF)
        {
            TopoDS_Face F = TopoDS.Face(It);
            if (!F.IsNull())
            {
                BC.Initialize(E, F);
                box.Add(BC.Value(w));
            }
        }
    }

    //=================================================================================================

    /// <summary>Adds the extremities of the surface data <paramref name="sd"/> (and of the stripe's end curve) to the
    /// boxes of its two sides.</summary>
    internal static void ChFi3d_EnlargeBox(TopOpeBRepDS_DataStructure DStr, ChFiDS_Stripe? st, ChFiDS_SurfData sd,
                                           ref Box b1, ref Box b2, bool isfirst)
    {
        double u, v;
        ChFiDS_CommonPoint cp1 = sd.Vertex(isfirst, 1);
        ChFiDS_CommonPoint cp2 = sd.Vertex(isfirst, 2);
        b1.Add(cp1.Point());
        b2.Add(cp2.Point());
        ChFiDS_FaceInterference fi1 = sd.InterferenceOnS1();
        ChFiDS_FaceInterference fi2 = sd.InterferenceOnS2();
        Geom_Surface S = DStr.Surface(sd.Surf()).Surface()!;
        Geom2d_Curve? pcs1 = fi1.PCurveOnSurf();
        Geom2d_Curve? pcs2 = fi2.PCurveOnSurf();
        Geom_Curve? c3d1 = DStr.Curve(fi1.LineIndex()).Curve();
        Geom_Curve? c3d2 = DStr.Curve(fi2.LineIndex()).Curve();
        Adaptor3d_Surface? F1 = Geometry(DStr, sd.IndexOfS1());
        Adaptor3d_Surface? F2 = Geometry(DStr, sd.IndexOfS2());
        double p1 = fi1.Parameter(isfirst);
        if (c3d1 is not null)
        {
            b1.Add(c3d1.Value(p1));
        }
        if (pcs1 is not null)
        {
            pcs1.Value(p1).Coord(out u, out v);
            b1.Add(S.Value(u, v));
        }
        if (F1 is not null)
        {
            Geom2d_Curve? pcf1 = fi1.PCurveOnFace();
            if (pcf1 is not null)
            {
                pcf1.Value(p1).Coord(out u, out v);
                b1.Add(F1.Value(u, v));
            }
        }
        double p2 = fi2.Parameter(isfirst);
        if (c3d2 is not null)
        {
            b2.Add(c3d2.Value(p2));
        }
        if (pcs2 is not null)
        {
            pcs2.Value(p2).Coord(out u, out v);
            b2.Add(S.Value(u, v));
        }
        if (F2 is not null)
        {
            Geom2d_Curve? pcf2 = fi2.PCurveOnFace();
            if (pcf2 is not null)
            {
                pcf2.Value(p2).Coord(out u, out v);
                b2.Add(F2.Value(u, v));
            }
        }
        if (st is not null)
        {
            Geom_Curve? c3d = DStr.Curve(st.Curve(isfirst)).Curve();
            Geom2d_Curve? c2d = st.PCurve(isfirst);
            if (st.Orientation(isfirst) == TopAbs_FORWARD)
            {
                st.Parameters(isfirst, out p1, out p2);
            }
            else
            {
                st.Parameters(isfirst, out p2, out p1);
            }
            if (c3d is not null)
            {
                b1.Add(c3d.Value(p1));
                b2.Add(c3d.Value(p2));
            }
            if (c2d is not null)
            {
                c2d.Value(p1).Coord(out u, out v);
                b1.Add(S.Value(u, v));
                c2d.Value(p2).Coord(out u, out v);
                b2.Add(S.Value(u, v));
            }
        }
    }

    //=================================================================================================

    /// <summary>The faces of <paramref name="EFMap"/> on either side of <paramref name="E"/> (the second one null
    /// for a free boundary).</summary>
    internal static void ChFi3d_conexfaces(in TopoDS_Edge E, out TopoDS_Face F1, out TopoDS_Face F2, ChFiDS_Map EFMap)
    {
        F1 = new TopoDS_Face();
        F2 = new TopoDS_Face();
        foreach (TopoDS_Shape It in EFMap[E])
        {
            if (F1.IsNull())
            {
                F1 = TopoDS.Face(It);
            }
            else
            {
                F2 = TopoDS.Face(It);
                if (!F2.IsSame(F1) || BRep_Tool.IsClosed(E, F1))
                {
                    break;
                }
                else
                {
                    F2.Nullify();
                }
            }
        }
    }

    //=======================================================================
    // function : EdgeState
    // purpose  : check concavities for the tops with 3 edges.
    //=======================================================================

    /// <summary>The state of a vertex where the three edges <paramref name="E"/> meet.</summary>
    internal static ChFiDS_State ChFi3d_EdgeState(TopoDS_Edge[] E, ChFiDS_Map EFMap)
    {
        ChFiDS_State sst;
        int i, j;
        // TopoDS_Face F[3]; (and the commented-out choice of the three faces)
        ChFi3d_conexfaces(E[0], out TopoDS_Face F1, out TopoDS_Face F2, EFMap);
        ChFi3d_conexfaces(E[1], out TopoDS_Face F3, out TopoDS_Face F4, EFMap);
        ChFi3d_conexfaces(E[2], out TopoDS_Face F5, out TopoDS_Face F6, EFMap);

        // if(F[0].IsNull() || F[1].IsNull() || F[2].IsNull()) sst = ChFiDS_FreeBoundary;
        if (F2.IsNull() || F4.IsNull() || F6.IsNull())
        {
            sst = ChFiDS_FreeBoundary;
        }
        else
        {
            TopAbs_Orientation o01, o02, o11, o12, o21, o22;
            i = ChFi3d.ConcaveSide(new BRepAdaptor_Surface(F1), new BRepAdaptor_Surface(F2), E[0], out o01, out o02);
            i = ChFi3d.ConcaveSide(new BRepAdaptor_Surface(F3), new BRepAdaptor_Surface(F4), E[1], out o11, out o12);
            j = ChFi3d.ConcaveSide(new BRepAdaptor_Surface(F5), new BRepAdaptor_Surface(F6), E[2], out o21, out o22);

            if (o01 == o11 && o02 == o21 && o12 == o22)
            {
                sst = ChFiDS_AllSame;
            }
            else if (o12 == o22 || i == 10 || j == 10)
            {
                sst = ChFiDS_OnDiff;
            }
            else
            {
                sst = ChFiDS_OnSame;
            }
        }
        return sst;
    }

    //=======================================================================
    // function : evalconti
    // purpose  : Method very fast to code regularities CN. It is necessary to
    //           refine the processing.
    //=======================================================================

    /// <summary>The continuity across an edge: CN on a seam of a cone, a sphere or a torus, else G1.</summary>
    internal static Continuity ChFi3d_evalconti(in TopoDS_Edge E, in TopoDS_Face F1, in TopoDS_Face F2)
    {
        Continuity cont = G1;
        if (!F1.IsSame(F2))
        {
            return cont;
        }
        TopoDS_Face F = F1;
        F.Orientation(TopAbs_FORWARD);
        BRepAdaptor_Surface S = new(F, false);
        SurfaceKind typ = S.GetType();
        if (typ != SurfaceKind.Cone && typ != SurfaceKind.Sphere && typ != SurfaceKind.Torus)
        {
            return cont;
        }
        return CN;
    }

    // modified by NIZNHY-PKV Wed Dec 15 11:22:35 2010f
    //=================================================================================================

    /// <summary>Whether the edge <paramref name="IE"/> of the spine is one of ChFiKPart's particular cases between
    /// <paramref name="S1"/> and <paramref name="S2"/>.</summary>
    internal static bool ChFi3d_KParticular(ChFiDS_Spine Spine, int IE, BRepAdaptor_Surface S1,
                                            BRepAdaptor_Surface S2)
    {
        bool bRet;
        //
        bRet = true;
        //
        if (Spine is ChFiDS_FilSpine fs && !fs.IsConstant(IE))
        {
            return !bRet;
        }
        //
        bool bIsPlane1, bIsPlane2;
        double aPA;
        CurveKind aCT;
        SurfaceKind aST1, aST2;
        //
        aST1 = S1.GetType();
        aST2 = S2.GetType();
        bIsPlane1 = (aST1 == Plane);
        bIsPlane2 = (aST2 == Plane);
        if (!(bIsPlane1 || bIsPlane2))
        {
            return !bRet;
        }
        //
        BRepAdaptor_Surface aS1 = (bIsPlane1) ? S1 : S2;
        BRepAdaptor_Surface aS2 = (bIsPlane1) ? S2 : S1;
        aST1 = aS1.GetType();
        aST2 = aS2.GetType();
        //
        if (aST2 != Plane && aST2 != SurfaceKind.Cylinder && aST2 != SurfaceKind.Cone)
        {
            return !bRet;
        }
        //
        BRepAdaptor_Curve bc = Spine.CurrentElementarySpine(IE);
        aCT = bc.GetType();
        if (aCT != Line && aCT != Circle)
        {
            return !bRet;
        }
        //
        aPA = global::Kerf.Engine.Foundation.Precision.Angular();
        //
        if (aST2 == Plane)
        {
            if (aCT == Line)
            {
                return bRet;
            }
        }
        else if (aST2 == SurfaceKind.Cylinder)
        {
            Direction3D aD1 = aS1.Plane().Axis().Direction();
            Direction3D aD2 = aS2.Cylinder().Axis().Direction();
            //
            if (aCT == Line && aD1.IsNormal(aD2, aPA))
            {
                return bRet;
            }
            else if (aCT == Circle && aD1.IsParallel(aD2, aPA))
            {
                return bRet;
            }
        }
        else if (aST2 == SurfaceKind.Cone)
        {
            Direction3D aD1 = aS1.Plane().Axis().Direction();
            Direction3D aD2 = aS2.Cone().Axis().Direction();
            if (aCT == Circle && aD1.IsParallel(aD2, aPA))
            {
                return bRet;
            }
        }
        return !bRet;
    }

    // modified by NIZNHY-PKV Wed Dec 15 11:22:43 2010t
    //=======================================================================
    // function : BoundFac
    // purpose  : Resize the limits of surface adjacent to the given box
    //            Useful for intersections with known extremities.
    //=======================================================================

    /// <summary>Reloads the face adaptor <paramref name="S"/> on the box widened in proportion to its size (or a
    /// tenth of the rest of the period).</summary>
    internal static void ChFi3d_BoundFac(BRepAdaptor_Surface S, double uumin, double uumax, double vvmin,
                                         double vvmax, bool checknaturalbounds = true)
    {
        ChFi3d_BoundSrfImpl(S, ChFi3d_BaseSurface(S), uumin, uumax, vvmin, vvmax, checknaturalbounds,
                            out Geom_Surface aSurface, out double aUU1, out double aUU2, out double aVV1,
                            out double aVV2);
        ChFi3d_ApplyBounds(S, aSurface, aUU1, aUU2, aVV1, aVV2);
    }

    //=======================================================================
    // function : ChFi3d_BoundSrf
    // purpose  : Resize the limits of surface adjacent to the given box
    //           Useful for intersections with known extremities.
    //=======================================================================

    /// <summary>Reloads the surface adaptor <paramref name="S"/> on the box widened in proportion to its size (or a
    /// tenth of the rest of the period).</summary>
    internal static void ChFi3d_BoundSrf(GeomAdaptor_Surface S, double uumin, double uumax, double vvmin,
                                         double vvmax, bool checknaturalbounds = true)
    {
        ChFi3d_BoundSrfImpl(S, ChFi3d_BaseSurface(S), uumin, uumax, vvmin, vvmax, checknaturalbounds,
                            out Geom_Surface aSurface, out double aUU1, out double aUU2, out double aVV1,
                            out double aVV2);
        ChFi3d_ApplyBounds(S, aSurface, aUU1, aUU2, aVV1, aVV2);
    }

    //=================================================================================================

    /// <summary>The parameter on <paramref name="C"/> of its first (<paramref name="Sens"/>) or last intersection
    /// with the plane.</summary>
    internal static bool ChFi3d_InterPlaneEdge(Adaptor3d_Surface Plan, Adaptor3d_Curve C, ref double W, bool Sens,
                                               double tolc)
    {
        IntCurveSurface_HInter Intersection = new();
        int isol = 0, nbp, iip;
        double uf = C.FirstParameter(), ul = C.LastParameter();
        double CW;

        Intersection.Perform(C, Plan);

        if (Intersection.IsDone())
        {
            nbp = Intersection.NbPoints();
            for (iip = 1; iip <= nbp; iip++)
            {
                CW = Intersection.Point(iip).W();
                if (C.IsPeriodic())
                {
                    CW = global::Kerf.Engine.Geometry.ElCLib.InPeriod(CW, uf - tolc, uf - tolc + C.Period());
                }
                if (uf - tolc <= CW && ul + tolc >= CW)
                {
                    if (isol == 0)
                    {
                        isol = iip;
                        W = CW;
                    }
                    else
                    {
                        if (Sens && CW < W)
                        {
                            W = CW;
                            isol = iip;
                        }
                        else if (!Sens && CW > W)
                        {
                            W = CW;
                            isol = iip;
                        }
                    }
                }
            }
        }
        if (isol == 0)
        {
            return false;
        }
        return true;
    }

    //=================================================================================================

    /// <summary>The point, the guide's tangent and the radius at the parameter <paramref name="p"/> of the side
    /// <paramref name="jf"/> of the surface data <paramref name="i"/> of the stripe.</summary>
    internal static void ChFi3d_ExtrSpineCarac(TopOpeBRepDS_DataStructure DStr, ChFiDS_Stripe cd, int i, double p,
                                               int jf, int sens, out Point3D P, out Vector3D V, out double R)
    {
        // check if it is necessary to add D1,D2 and DR
        // Attention for approximated surfaces it is assumed that e
        // the parameters of the pcurve are the same as of
        // elspine used for its construction.
        Geom_Surface fffil = DStr.Surface(cd.SetOfSurfData()!.Value(i).Surf()).Surface()!;
        Point2D pp = cd.SetOfSurfData()!.Value(i).Interference(jf).PCurveOnSurf()!.Value(p);
        GeomAdaptor_Surface gs = new(fffil);
        P = fffil.Value(pp.X(), pp.Y());
        Point3D Pbid;
        Vector3D Vbid;
        switch (gs.GetType())
        {
            case SurfaceKind.Cylinder:
            {
                Cylinder cyl = gs.Cylinder();
                R = cyl.Radius();
                global::Kerf.Engine.Geometry.ElSLib.D1(pp.X(), pp.Y(), cyl, out Pbid, out Vbid, out V);
            }
                break;
            case SurfaceKind.Torus:
            {
                Torus tor = gs.Torus();
                R = tor.MinorRadius();
                global::Kerf.Engine.Geometry.ElSLib.D1(pp.X(), pp.Y(), tor, out Pbid, out V, out Vbid);
            }
                break;
            default:
            {
                int nbelspine;
                ChFiDS_Spine sp = cd.Spine()!;
                ChFiDS_FilSpine fsp = (ChFiDS_FilSpine)sp;
                nbelspine = sp.NbEdges();
                ChFiDS_ElSpine hels;
                if (nbelspine == 1)
                {
                    hels = sp.ElSpine(1)!;
                }
                else
                {
                    hels = sp.ElSpine(p)!;
                }
                if (fsp.IsConstant())
                {
                    R = fsp.Radius();
                }
                else
                {
                    R = fsp.Law(hels)!.Value(p);
                }
                hels.D1(p, out Pbid, out V);
            }
                break;
        }
        V.Normalize();
        if (sens == 1)
        {
            V.Reverse();
        }
    }

    //=======================================================================
    // function : ChFi3d_CircularSpine
    // purpose  : Calculate a circular guideline for the corner created from
    //           tangent points and vectors calculated at the extremities
    //           of guidelines of start and end fillets.
    //=======================================================================

    /// <summary>The circular guide of a corner from the points and tangents at the ends of the two fillets' guides,
    /// or null when they turn the same way.</summary>
    internal static Geom_Circle? ChFi3d_CircularSpine(ref double WFirst, ref double WLast, in Point3D Pdeb,
                                                      in Vector3D Vdeb, in Point3D Pfin, in Vector3D Vfin, double rad)
    {
        Circle3D ccc = new();
        Plane3D Pl1 = new(Pdeb, new Direction3D(Vdeb)), Pl2 = new(Pfin, new Direction3D(Vfin));
        IntAna_QuadQuadGeo LInt = new(Pl1, Pl2, global::Kerf.Engine.Foundation.Precision.Angular(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Line3D li;
        if (LInt.IsDone())
        {
            li = LInt.Line(1);
            Point3D cendeb = global::Kerf.Engine.Geometry.ElCLib.Value(global::Kerf.Engine.Geometry.ElCLib.Parameter(li, Pdeb), li);
            Point3D cenfin = global::Kerf.Engine.Geometry.ElCLib.Value(global::Kerf.Engine.Geometry.ElCLib.Parameter(li, Pfin), li);
            Vector3D vvdeb = new(cendeb, Pdeb);
            Vector3D vvfin = new(cenfin, Pfin);
            Direction3D dddeb = new(vvdeb);
            Direction3D ddfin = new(vvfin);
            if (Vdeb.Crossed(vvdeb).Dot(Vfin.Crossed(vvfin)) > 0.0)
            {
                return null;
            }
            Frame3D circax2 = new(cendeb, dddeb ^ ddfin, dddeb);
            ccc.SetPosition(circax2);
            ccc.SetRadius(rad);
            WFirst = 0.0;
            WLast = dddeb.Angle(ddfin);
            return new Geom_Circle(ccc);
        }

        return null;
    }

    //=======================================================================
    // function : ChFi3d_Spine
    // purpose  : Calculates the poles of the guideline for the corner from
    //           tangent points and vectors calculated at the extremities of
    //           guidelines of start and end fillets.
    //=======================================================================

    /// <summary>The cubic Bézier guide of a corner from the points and tangents at the ends of the two fillets'
    /// guides; scales <paramref name="vd"/> and <paramref name="vf"/>.</summary>
    internal static Geom_BezierCurve ChFi3d_Spine(in Point3D pd, ref Vector3D vd, in Point3D pf, ref Vector3D vf, double R)
    {
        NCollection_Array1<Point3D> pol = new(1, 4);
        double fac = 0.5 * Math.Tan((Math.PI - vd.Angle(vf)) * 0.5);
        pol[1] = pd;
        vd.Multiply(fac * R);
        pol[2].SetCoord(pd.X() + vd.X(), pd.Y() + vd.Y(), pd.Z() + vd.Z());
        pol[4] = pf;
        vf.Multiply(fac * R);
        pol[3].SetCoord(pf.X() + vf.X(), pf.Y() + vf.Y(), pf.Z() + vf.Z());
        return new Geom_BezierCurve(pol);
    }

    //=======================================================================
    // function : IsInFront
    // purpose  : Checks if surfdata i1 and i2 are face to face
    //=======================================================================

    /// <summary>
    /// Whether the surface data <paramref name="i1"/> of <paramref name="cd1"/> and <paramref name="i2"/> of
    /// <paramref name="cd2"/> lean on a common face whose traces cross; their crossing parameters, the face, the sides
    /// and whether they are on the same side of it.
    /// </summary>
    /// <remarks>The outputs are <c>ref</c>: OCCT leaves them unchanged (or puts back the first crossing found) when
    /// a later pair of sides does not cross.</remarks>
    internal static bool ChFi3d_IsInFront(TopOpeBRepDS_DataStructure DStr, ChFiDS_Stripe cd1, ChFiDS_Stripe cd2,
                                          int i1, int i2, int sens1, int sens2, ref double p1, ref double p2,
                                          ref TopoDS_Face face, ref bool sameside, ref int jf1, ref int jf2,
                                          ref bool visavis, in TopoDS_Vertex Vtx, bool Check2dDistance = false,
                                          bool enlarge = false)
    {
        bool isf1 = (sens1 == 1), isf2 = (sens2 == 1);
        ChFiDS_SurfData fd1 = cd1.SetOfSurfData()!.Value(i1);
        ChFiDS_SurfData fd2 = cd2.SetOfSurfData()!.Value(i2);

        TopAbs_Orientation Or, OrSave1, OrSave2, OrFace1, OrFace2;
        visavis = false;
        double u1 = 0.0, u2 = 0.0;
        bool ss = false, ok = false;
        int j1 = 0, j2 = 0;
        TopoDS_Face ff = new();
        TopoDS_Vertex vtx = Vtx; // a local function cannot capture an in parameter
        if (fd1.IndexOfS1() == fd2.IndexOfS1())
        {
            jf1 = 1;
            jf2 = 1;
            face = TopoDS.Face(DStr.Shape(fd1.Index(jf1)));
            if (face.IsNull())
            {
                throw new NullObjectException("ChFi3d_IsInFront : Trying to check orientation of NULL face");
            }
            OrSave1 = cd1.Orientation(jf1);
            Or = OrFace1 = face.Orientation();
            OrSave2 = cd2.Orientation(jf2);
            TopoDS_Shape shape2 = DStr.Shape(fd2.Index(jf2));
            if (shape2.IsNull())
            {
                throw new NullObjectException("ChFi3d_IsInFront : Trying to check orientation of NULL shape");
            }
            OrFace2 = shape2.Orientation();
            visavis = true;
            sameside = ChFi3d.SameSide(Or, OrSave1, OrSave2, OrFace1, OrFace2);
            // The parameters of the other side are not used for orientation. This would raise problems
            int kf1 = jf1, kf2 = jf2;
            double pref1 = fd1.Interference(kf1).Parameter(isf1);
            double pref2 = fd2.Interference(kf2).Parameter(isf2);
            Point2D P2d = new();
            if (Check2dDistance)
            {
                P2d = BRep_Tool.Parameters(Vtx, face);
            }
            if (ChFi3d_IntTraces(fd1, pref1, ref p1, jf1, sens1, fd2, pref2, ref p2, jf2, sens2, P2d,
                                 Check2dDistance, enlarge))
            {
                u1 = p1;
                u2 = p2;
                ss = sameside;
                j1 = jf1;
                j2 = jf2;
                ff = face;
                ok = true;
            }
        }
        if (fd1.IndexOfS2() == fd2.IndexOfS1())
        {
            OtherSides(2, 1, ref p1, ref p2, ref face, ref sameside, ref jf1, ref jf2, ref visavis);
        }
        if (fd1.IndexOfS1() == fd2.IndexOfS2())
        {
            OtherSides(1, 2, ref p1, ref p2, ref face, ref sameside, ref jf1, ref jf2, ref visavis);
        }
        if (fd1.IndexOfS2() == fd2.IndexOfS2())
        {
            OtherSides(2, 2, ref p1, ref p2, ref face, ref sameside, ref jf1, ref jf2, ref visavis);
        }
        return ok;

        // OCCT writes the next three cases out in full; they differ only by the sides compared.
        void OtherSides(int s1, int s2, ref double p1, ref double p2, ref TopoDS_Face face, ref bool sameside,
                        ref int jf1, ref int jf2, ref bool visavis)
        {
            jf1 = s1;
            jf2 = s2;
            face = TopoDS.Face(DStr.Shape(fd1.Index(jf1)));
            if (face.IsNull())
            {
                throw new NullObjectException("ChFi3d_IsInFront : Trying to check orientation of NULL face");
            }
            OrSave1 = cd1.Orientation(jf1);
            Or = OrFace1 = face.Orientation();
            OrSave2 = cd2.Orientation(jf2);
            TopoDS_Shape shape2 = DStr.Shape(fd2.Index(jf2));
            if (shape2.IsNull())
            {
                throw new NullObjectException("ChFi3d_IsInFront : Trying to check orientation of NULL shape");
            }
            OrFace2 = shape2.Orientation();
            visavis = true;
            sameside = ChFi3d.SameSide(Or, OrSave1, OrSave2, OrFace1, OrFace2);
            // The parameters of the other side are not used for orientation. This would raise problems
            int kf1 = jf1, kf2 = jf2;
            double pref1 = fd1.Interference(kf1).Parameter(isf1);
            double pref2 = fd2.Interference(kf2).Parameter(isf2);
            Point2D P2d = new();
            if (Check2dDistance)
            {
                P2d = BRep_Tool.Parameters(vtx, face);
            }
            if (ChFi3d_IntTraces(fd1, pref1, ref p1, jf1, sens1, fd2, pref2, ref p2, jf2, sens2, P2d,
                                 Check2dDistance, enlarge))
            {
                bool restore = ok
                               && ((j1 == jf1 && sens1 * (p1 - u1) > 0.0) || (j2 == jf2 && sens2 * (p2 - u2) > 0.0));
                ok = true;
                if (restore)
                {
                    p1 = u1;
                    p2 = u2;
                    sameside = ss;
                    jf1 = j1;
                    jf2 = j2;
                    face = ff;
                }
                else
                {
                    u1 = p1;
                    u2 = p2;
                    ss = sameside;
                    j1 = jf1;
                    j2 = jf2;
                    ff = face;
                }
            }
            // the re-initialization is added in case p1,... take wrong values
            else if (ok)
            {
                p1 = u1;
                p2 = u2;
                sameside = ss;
                jf1 = j1;
                jf2 = j2;
                face = ff;
            }
        }
    }

    //=================================================================================================

    private static double recadre(double p, double @ref, int sens, double first, double last)
    {
        double pp = p + (sens > 0 ? (first - last) : (last - first));
        return ((Math.Abs(pp - @ref) < Math.Abs(p - @ref)) ? pp : p);
    }

    /// <summary>
    /// The components (<paramref name="DU"/>, <paramref name="DV"/>) in the parametric space of the vector
    /// <paramref name="V3d"/> on the tangent plane spanned by <paramref name="D1u"/> and <paramref name="D1v"/>
    /// (least squares).
    /// </summary>
    internal static void ChFi3d_Coefficient(in Vector3D V3d, in Vector3D D1u, in Vector3D D1v, out double DU, out double DV)
    {
        double AA = D1u.SquareMagnitude();
        double BB = D1u.Dot(D1v);
        double CC = D1v.SquareMagnitude();
        double DD = D1u.Dot(V3d);
        double EE = D1v.Dot(V3d);
        double Delta = AA * CC - BB * BB;
        DU = (DD * CC - EE * BB) / Delta;
        DV = (AA * EE - BB * DD) / Delta;
    }
}
