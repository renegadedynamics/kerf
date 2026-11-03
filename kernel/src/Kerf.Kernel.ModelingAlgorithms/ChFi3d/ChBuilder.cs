// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_ChBuilder.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_ChBuilder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Booleans.Legacy.Data;
using Kerf.Engine.Extrema;
using Kerf.Engine.Fillets.Blend;
using Kerf.Engine.Fillets.BlendFunctions;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Fillets.Builder_0;
using static Kerf.Engine.Fillets.Data.ChamfMethod;
using static Kerf.Engine.Fillets.Data.ChamfMode;
using static Kerf.Engine.Fillets.Data.ErrorStatus;
using static Kerf.Engine.Fillets.Data.State;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;
using static Kerf.Engine.Booleans.Legacy.Data.DsKind;

namespace Kerf.Engine.Fillets;

/// <summary>Computes chamfers on the edges of a shape: one distance, two distances, or a distance and an angle.</summary>
/// <remarks>
/// A working object of the fillet algorithms (decision 21), ported over OCCT's files: this file (the header and
/// <c>ChFi3d_ChBuilder.cxx</c>), <c>_C2</c> and <c>_C3</c> (corners). Ported in unit 7N2 so that the builder can be
/// compared with native OCCT end to end; the API wrappers are unit 7N3's. Edges and faces given by the caller are mapped
/// onto the builder's working copy (<see cref="FilletBuilderBase.ToWork"/>). OCCT_DEBUG blocks (traces) are left out.
/// </remarks>
public partial class ChBuilder : FilletBuilderBase
{
    private ChamfMode _mode;

    /// <summary>Initializes the builder on <paramref name="s"/>.</summary>
    public ChBuilder(in Shape s, double ta = 1.0e-2)
        : base(s, ta)
    {
        _mode = ChamfMode.ClassicChamfer;
    }

    private static void SearchCommonFaces(Map efMap, in Edge e, ref Face f1, ref Face f2)
    {
        Face fc = new();
        NCollection_List<Shape>.Iterator it = new();

        f1.Nullify();
        f2.Nullify();
        for (it.Initialize(efMap[e]); it.More(); it.Next())
        {
            fc = ShapeCasts.Face(it.Value());
            if (f1.IsNull)
            {
                f1 = fc;
            }
            else if (!fc.IsSame(f1))
            {
                f2 = fc;
                break;
            }
        }

        if (!f1.IsNull && f2.IsNull && BrepTools.IsReallyClosed(e, f1))
        {
            f2 = f1;
        }
    }

    private static void ExtentSpineOnCommonFace(Spine? spine1, Spine? spine2, in Vertex v, double dis1, double dis2, bool isfirst1, bool isfirst2)
    {
        double tolesp = 1.0e-7;

        // alpha, the opening angle between two
        // tangents of two guidelines in V is found
        double tga1 = 0.0, tga2 = 0.0;
        double d1plus = 0.0, d2plus = 0.0;

        Point3D tmp = new();
        Vector3D tg1 = new(), tg2 = new();
        spine1.D1(spine1.Absc(v), out tmp, out tg1);
        spine2.D1(spine2.Absc(v), out tmp, out tg2);
        tg1 = tg1.Normalized();
        tg2 = tg2.Normalized();
        if (isfirst1)
        {
            tg1 = tg1.Reversed();
        }
        if (isfirst2)
        {
            tg2 = tg2.Reversed();
        }

        double cosalpha = 0.0, sinalpha = 0.0;
        cosalpha = tg1.Dot(tg2);
        sinalpha = Math.Sqrt(1 - cosalpha * cosalpha);

        // a1+a2 = alpha
        double temp = 0.0;
        temp = cosalpha + dis2 / dis1;
        if (Math.Abs(temp) > tolesp)
        {
            tga1 = sinalpha / temp;
            d1plus = dis1 / tga1;
        }
        temp = cosalpha + dis1 / dis2;
        if (Math.Abs(temp) > tolesp)
        {
            tga2 = sinalpha / temp;
            d2plus = dis2 / tga2;
        }

        // extension by the calculated distance
        if (d1plus > 0.0)
        {
            d1plus *= 3.0;
            if (isfirst1)
            {
                spine1.SetFirstParameter(-d1plus);
                spine1.SetFirstTgt(0.0);
            }
            else
            {
                double param = spine1.LastParameter(spine1.EdgeCount);
                spine1.SetLastParameter(d1plus + param);
                spine1.SetLastTgt(param);
            }
        }
        if (d2plus > 0.0)
        {
            d2plus *= 1.5;
            if (isfirst2)
            {
                spine2.SetFirstParameter(-d2plus);
                spine2.SetFirstTgt(0.0);
            }
            else
            {
                double param = spine2.LastParameter(spine2.EdgeCount);
                spine2.SetLastParameter(d2plus + param);
                spine2.SetLastTgt(param);
            }
        }
    }





    //=======================================================================
    // function : SearchCommonFaces
    // purpose  : search the 2 common faces <F1> and <F2> of the edge <E>
    //           uses the EFMap and take the 2 first good faces
    //=======================================================================

    //=======================================================================
    // function : ExtentSpinesOnCommonFace
    // purpose  : Extend spines of two chamfers by distance dis1,dis2
    //           on their common face
    //           Two guide lines Spine1 and Spine2 cross in V
    //           isfirst(i) = False if Spine(i) is oriented to V (i = 1,2)
    //=======================================================================

    //=================================================================================================

    //=======================================================================
    // function : Add
    // purpose  : create a new stripe with a spine containing the edge <E>
    //           add on the spine the tangential edges to <E>
    //=======================================================================

    public void Add(in Edge e)
    {
        Edge E = ShapeCasts.Edge(ToWork(e)); // the working copy's (OCCT.NET)
        Face dummy = new();

        if (Contains(E) == 0 && _efMap.Contains(E))
        {
            Stripe? stripe = new Stripe();
            ref Spine? sp = ref stripe.ChangeSpine();
            sp = new ChamfSpine(_tolesp);
            ChamfSpine? spine = (sp as ChamfSpine);

            Edge E_wnt = E;
            E_wnt.Orientation(Forward);
            spine.SetEdges(E_wnt);
            if (PerformElement(spine, -1, dummy))
            {
                PerformExtremity(spine);
                spine.Load();
                _listStripe.Append(stripe);
            }
        }
    }

    //=======================================================================
    // function : Add
    // purpose  : create a new stripe with a ChamfSpine containing the edge <E>
    //           with the distance <Dis>   on the face <F>
    //           . Add the tangential edges continuous to E in the spine
    //
    //=======================================================================

    public void Add(double dis, in Edge e)
    {
        Edge E = ShapeCasts.Edge(ToWork(e)); // the working copy's (OCCT.NET)
        if (Contains(E) == 0 && _efMap.Contains(E))
        {

            Face dummy = new();

            Edge E_wnt = E;
            E_wnt.Orientation(Forward);

            Stripe? stripe = new Stripe();
            ref Spine? sp = ref stripe.ChangeSpine();
            sp = new ChamfSpine(_tolesp);
            ChamfSpine? spine = (sp as ChamfSpine);

            spine.SetMode(_mode);

            spine.SetEdges(E_wnt);
            if (PerformElement(spine, -1, dummy))
            {
                spine.Load();
                _listStripe.Append(stripe);

                spine.SetDist(dis);

                PerformExtremity(spine);
            }
        }
    }

    //=======================================================================
    // function : SetDist
    // purpose  : set the distances <Dis>of the contour of
    //           index IC
    //=======================================================================

    public void SetDist(double dis, int ic, in Face f)
    {
        Face F = ShapeCasts.Face(ToWork(f)); // the working copy's (OCCT.NET)

        if (ic <= NbElements())
        {
            ChamfSpine? csp = (Value(ic) as ChamfSpine);

            // Search the first edge which has a common face equal to F
            Face f1 = new(), f2 = new(), firstF1 = new(), firstF2 = new();
            // TopAbs_Orientation Or1,Or2;
            // int Choix, ChoixConge;
            FaceSurfaceAdaptor sb1 = new(), sb2 = new();
            int i = 1;
            bool found = false;
            while ((i <= csp.EdgeCount) && (!found))
            {
                SearchCommonFaces(_efMap, csp.Edges(i), ref f1, ref f2);
                if (i == 1)
                {
                    firstF1 = f1;
                    firstF2 = f2;
                }
                found = (f1.IsSame(F) || f2.IsSame(F));
                i++;
            }

            if (found)
            {
                if (f2.IsSame(F))
                {
                    f2 = f1;
                    f1 = F;
                }
                csp.SetDist(dis);
            }
            else
            {
                throw new DomainErrorException("the face is not common to any of edges of the contour");
            }
        }
    }

    //=================================================================================================

    public void GetDist(int ic, ref double dis)
    {
        ChamfSpine? chsp = (Value(ic) as ChamfSpine);
        chsp.GetDist(out dis);
    }

    //=======================================================================
    // function : Add
    // purpose  : create a new stripe with a ChAsymSpine containing the edge <E>
    //           with the distance <Dis> and the angle Angle  on the face <F>
    //           . Add the tangential edges continuous to E in the spine
    //
    //=======================================================================

    public void Add(double dis1, double dis2, in Edge e, in Face f)
    {
        Edge E = ShapeCasts.Edge(ToWork(e)); // the working copy's (OCCT.NET)
        Face F = ShapeCasts.Face(ToWork(f)); // the working copy's (OCCT.NET)
        if (Contains(E) == 0 && _efMap.Contains(E))
        {

            Edge E_wnt = E;
            E_wnt.Orientation(Forward);

            Stripe? stripe = new Stripe();
            ref Spine? sp = ref stripe.ChangeSpine();
            sp = new ChamfSpine(_tolesp);
            ChamfSpine? spine = (sp as ChamfSpine);

            spine.SetMode(_mode);
            double offset = -1;
            if (_mode == ChamfMode.ConstThroatWithPenetrationChamfer)
            {
                offset = Min(dis1, dis2);
            }

            spine.SetEdges(E_wnt);
            if (PerformElement(spine, offset, F))
            {
                spine.Load();
                _listStripe.Append(stripe);

                spine.SetDists(dis1, dis2);

                PerformExtremity(spine);
            }
        }
    }

    //=======================================================================
    // function : SetDists
    // purpose  : set the distances <Dis1> and <Dis2> of the contour of
    //           index IC
    //=======================================================================

    public void SetDists(double dis1, double dis2, int ic, in Face f)
    {
        Face F = ShapeCasts.Face(ToWork(f)); // the working copy's (OCCT.NET)

        if (ic <= NbElements())
        {
            ChamfSpine? csp = (Value(ic) as ChamfSpine);

            // Search the first edge which has a common face equal to F
            Face f1 = new(), f2 = new(), firstF1 = new(), firstF2 = new();
            Orientation or1 = default, or2 = default;
            int choix = 0, choixConge = 0;
            FaceSurfaceAdaptor sb1 = new(), sb2 = new();
            int i = 1;
            bool found = false;
            while ((i <= csp.EdgeCount) && (!found))
            {
                SearchCommonFaces(_efMap, csp.Edges(i), ref f1, ref f2);
                if (i == 1)
                {
                    firstF1 = f1;
                    firstF2 = f2;
                }
                found = (f1.IsSame(F) || f2.IsSame(F));
                i++;
            }

            if (found)
            {
                if (f2.IsSame(F))
                {
                    f2 = f1;
                    f1 = F;
                }
                sb1.Initialize(f1);
                sb2.Initialize(f2);
                choix = global::Kerf.Engine.Fillets.ChFi3d.ConcaveSide(sb1, sb2, csp.Edges(i - 1), out or1, out or2);
                sb1.Initialize(firstF1);
                sb2.Initialize(firstF2);
                choixConge = global::Kerf.Engine.Fillets.ChFi3d.ConcaveSide(sb1, sb2, csp.Edges(1), out or1, out or2);
                if (choixConge % 2 != choix % 2)
                {
                    csp.SetDists(dis2, dis1);
                }
                else
                {
                    csp.SetDists(dis1, dis2);
                }
            }
            else
            {
                throw new DomainErrorException("the face is not common to any of edges of the contour");
            }
        }
    }

    //=================================================================================================

    public void Dists(int ic, ref double dis1, ref double dis2)
    {
        ChamfSpine? chsp = (Value(ic) as ChamfSpine);
        double temp1 = 0.0, temp2 = 0.0;
        chsp.Dists(out temp1, out temp2);
        dis1 = temp1;
        dis2 = temp2;
    }

    //=======================================================================
    // function : Add
    // purpose  : create a new stripe with a ChAsymSpine containing the edge <E>
    //           with the distance <Dis> and the angle Angle  on the face <F>
    //           . Add the tangential edges continuous to E in the spine
    //
    //=======================================================================

    public void AddDA(double dis1, double angle, in Edge e, in Face f)
    {
        Edge E = ShapeCasts.Edge(ToWork(e)); // the working copy's (OCCT.NET)
        Face F = ShapeCasts.Face(ToWork(f)); // the working copy's (OCCT.NET)
        if (Contains(E) == 0 && _efMap.Contains(E))
        {

            Edge E_wnt = E;
            E_wnt.Orientation(Forward);

            Stripe? stripe = new Stripe();
            ref Spine? sp = ref stripe.ChangeSpine();
            sp = new ChamfSpine(_tolesp);
            ChamfSpine? spine = (sp as ChamfSpine);

            spine.SetEdges(E_wnt);
            if (PerformElement(spine, -1, F))
            {
                spine.Load();
                _listStripe.Append(stripe);

                spine.SetDistAngle(dis1, angle);

                PerformExtremity(spine);
            }
        }
    }

    //=======================================================================
    // function : SetDistAngle
    // purpose  : set the distances <Dis> and <Angle> of the contour of
    //           index IC
    //=======================================================================

    public void SetDistAngle(double dis, double angle, int ic, in Face f)
    {
        Face F = ShapeCasts.Face(ToWork(f)); // the working copy's (OCCT.NET)

        if (ic <= NbElements())
        {
            ChamfSpine? csp = (Value(ic) as ChamfSpine);

            // Search the first edge which has a common face equal to F
            Face f1 = new(), f2 = new(), firstF1 = new(), firstF2 = new();
            FaceSurfaceAdaptor sb1 = new(), sb2 = new();
            int i = 1;
            bool found = false;

            while ((i <= csp.EdgeCount) && (!found))
            {
                SearchCommonFaces(_efMap, csp.Edges(i), ref f1, ref f2);
                if (i == 1)
                {
                    firstF1 = f1;
                    firstF2 = f2;
                }
                found = (f1.IsSame(F) || f2.IsSame(F));
                i++;
            }

            if (found)
            {
                if (f2.IsSame(F))
                {
                    f2 = f1;
                    f1 = F;
                }
                sb1.Initialize(f1);
                sb2.Initialize(f2);
                sb1.Initialize(firstF1);
                sb2.Initialize(firstF2);
                csp.SetDistAngle(dis, angle);
            }
            else
            {
                throw new DomainErrorException("the face is not common to any edges of the contour");
            }
        }
    }

    //=================================================================================================

    public void GetDistAngle(int ic, ref double dis, ref double angle)
    {
        ChamfSpine? chsp = (Value(ic) as ChamfSpine);

        chsp.GetDistAngle(out dis, out angle);
    }

    //=================================================================================================

    public void SetMode(ChamfMode mode)
    {
        _mode = mode;
    }

    //=================================================================================================

    public ChamfMethod IsChamfer(int ic)
    {
        ChamfSpine? chsp = (Value(ic) as ChamfSpine);

        ChamfMethod ret = chsp.IsChamfer;

        return ret;
    }

    //=================================================================================================

    public ChamfMode Mode()
    {
        return _mode;
    }

    //=================================================================================================

    public void ResetContour(int ic)
    {
        if (ic <= NbElements())
        {
            ChamfSpine? chsp = (Value(ic) as ChamfSpine);
            chsp.Reset(true);
        }
    }

    //--------------------------------AJOUT----------------------------------
    //=================================================================================================

    public void Simulate(int ic)
    {
        NCollection_List<Stripe>.Iterator itel = new();
        int i = 1;
        for (itel.Initialize(_listStripe); itel.More(); itel.Next(), i++)
        {
            if (i == ic)
            {
                PerformSetOfSurf(itel.ChangeValue(), true);
                break;
            }
        }
    }

    //---------------------------AJOUT---------------------------------------
    //=================================================================================================

    public int NbSurf(int ic)
    {
        NCollection_List<Stripe>.Iterator itel = new();
        int i = 1;
        for (itel.Initialize(_listStripe); itel.More(); itel.Next(), i++)
        {
            if (i == ic)
            {
                return itel.Value().SetOfSurfData().Length();
            }
        }
        return 0;
    }

    //--------------------------AJOUT---------------------------------------
    //=================================================================================================

    public NCollection_HArray1<CircSection>? Sect(int ic, int IS)
    {
        NCollection_List<Stripe>.Iterator itel = new();
        int i = 1;
        NCollection_HArray1<CircSection>? res = null;
        for (itel.Initialize(_listStripe); itel.More(); itel.Next(), i++)
        {
            if (i == ic)
            {
                object? bid = itel.Value().SetOfSurfData().Value(IS).Simul();
                res = (bid as NCollection_HArray1<CircSection>);
                return res;
            }
        }
        return null;
    }

    //-------------------MODIFS---------------------------------------------
    //=======================================================================
    // function : SimulKPart
    // purpose  : Stores simulating sections in simul
    //=======================================================================

    internal override void SimulKPart(SurfData? sd)
    {
        DsDataStructure dStr = _ds.ChangeDS();
        Surface? s = dStr.Surface(sd.Surf()).Surface();
        Point2D p1f = 
            sd.InterferenceOnS1().PCurveOnSurf().Value(sd.InterferenceOnS1().FirstParameter());
        Point2D p1l = 
            sd.InterferenceOnS1().PCurveOnSurf().Value(sd.InterferenceOnS1().LastParameter());
        Point2D p2f = 
            sd.InterferenceOnS2().PCurveOnSurf().Value(sd.InterferenceOnS2().FirstParameter());
        Point2D p2l = 
            sd.InterferenceOnS2().PCurveOnSurf().Value(sd.InterferenceOnS2().LastParameter());
        SurfaceAdaptor AS = new(s);
        NCollection_HArray1<CircSection>? sec = null;
        double u1 = 0.0, v1 = 0.0, u2 = 0.0, v2 = 0.0;
        SurfaceKind typ = AS.GetType();
        switch (typ)
        {
            case Plane: {
                v1 = p1f.Y();
                v2 = p2f.Y();
                u1 = Max(p1f.X(), p2f.X());
                u2 = Min(p1l.X(), p2l.X());
                sec = new NCollection_HArray1<CircSection>(1, 2);
                Plane3D pl = AS.Plane();
                ref CircSection sec1 = ref sec.ChangeValue(1);
                ref CircSection sec2 = ref sec.ChangeValue(2);
                sec1.Set(global::Kerf.Engine.Geometry.ElSLib.PlaneUIso(pl.Position(), u1), v1, v2);
                sec2.Set(global::Kerf.Engine.Geometry.ElSLib.PlaneUIso(pl.Position(), u2), v1, v2);
            }
            break;
            case SurfaceKind.Cone: {
                v1 = p1f.Y();
                v2 = p2f.Y();
                u1 = Max(p1f.X(), p2f.X());
                u2 = Min(p1l.X(), p2l.X());
                double ang = (u2 - u1);
                Cone co = AS.Cone();
                double rad = co.RefRadius(), sang = co.SemiAngle();
                int n = (int)(36.0 * ang / M_PI + 1);
                if (n < 2)
                {
                    n = 2;
                }
                sec = new NCollection_HArray1<CircSection>(1, n);
                for (int i = 1; i <= n; i++)
                {
                    ref CircSection isec = ref sec.ChangeValue(i);
                    double u = u1 + (i - 1) * (u2 - u1) / (n - 1);
                    isec.Set(global::Kerf.Engine.Geometry.ElSLib.ConeUIso(co.Position(), rad, sang, u), v1, v2);
                }
            }
            break;
            default:
                break;
        }
        sd.SetSimul(sec);
    }

    //------------------------MODIFS---------------------------------------
    //=================================================================================================

    internal override bool SimulSurf(SurfData? data, ElSpine? hGuide, Spine? spine, int choix, FaceSurfaceAdaptor? s1, TopolTool? i1, FaceSurfaceAdaptor? s2, TopolTool? i2, double tolGuide, ref double first, ref double last, bool inside, bool appro, bool forward, bool recOnS1, bool recOnS2, math_Vector soldep, ref int intf, ref int intl)
    {
        ChamfSpine? chsp = (spine as ChamfSpine);

        if (chsp is null)
        {
            throw new ConstructionErrorException("SimulSurf : this is not the spine of a chamfer");
        }

        double radius = 0.0;
        // Flexible parameters!
        double la = hGuide.LastParameter(), fi = hGuide.FirstParameter();
        double longueur = la - fi;
        double maxStep = longueur * 0.05;
        double radiusspine = 0, locfleche, w;
        Point3D pbid = new();
        Vector3D d1 = new(), d2 = new();
        // As ElSpine is parameterized by a curvilinear quasi-abscissa,
        // the min radius is estimated as 1/D2 max;
        // for(int i = 0; i <= 20; i++){
        int i = 0;
        for (i = 0; i <= 20; i++)
        {
            w = fi + i * maxStep;
            hGuide.D2(w, out pbid, out d1, out d2);
            double temp = d2.SquareMagnitude();
            if (temp > radiusspine)
            {
                radiusspine = temp;
            }
        }

        global::Kerf.Engine.Fillets.Blend.Line? lin = null;
        double pFirst = first;
        if (intf != 0)
        {
            first = chsp.FirstParameter(1);
        }
        if (intl != 0)
        {
            last = chsp.LastParameter(chsp.EdgeCount);
        }

        ElSpine? offsetHGuide = null;

        if (chsp.IsChamfer == ChamfMethod.Sym)
        {
            double dis = 0.0;
            chsp.GetDist(out dis);
            radius = Max(dis, radiusspine);
            locfleche = radius * 1.0e-2; // graphic criterion

            GenChamfer? pFunc = null; // std::unique_ptr in OCCT
            GenChamfInv? pFInv = null; // std::unique_ptr in OCCT
            if (chsp.Mode() == ChamfMode.ClassicChamfer)
            {
                pFunc = new Chamfer(s1, s2, hGuide);
                pFInv = new ChamfInv(s1, s2, hGuide);
            }
            else
            {
                pFunc = new ConstThroat(s1, s2, hGuide);
                pFInv = new ConstThroatInv(s1, s2, hGuide);
            }
            pFunc.Set(dis, dis, choix);
            pFInv.Set(dis, dis, choix);

            _done = SimulData(data, hGuide, offsetHGuide, ref lin, s1, i1, s2, i2, pFunc, pFInv, pFirst, maxStep, locfleche,
                                              tolGuide, ref first, ref last, inside, appro, forward, soldep, 4, recOnS1, recOnS2);

            if (!_done)
            {
                return false;
            }
            NCollection_HArray1<CircSection>? sec = null;
            Point2D pf1 = new(), pl1 = new(), pf2 = new(), pl2 = new();

            int nbp = lin.NbPoints();
            sec = new NCollection_HArray1<CircSection>(1, nbp);
            for (i = 1; i <= nbp; i++)
            {
                ref CircSection isec = ref sec.ChangeValue(i);
                double u1 = 0.0, v1 = 0.0, u2 = 0.0, v2 = 0.0, ww = 0.0, p1 = 0.0, p2 = 0.0;
                Line3D line = new();
                BlendPoint p = lin.Point(i);
                p.ParametersOnS1(out u1, out v1);
                p.ParametersOnS2(out u2, out v2);
                ww = p.Parameter();
                pFunc.Section(ww, u1, v1, u2, v2, out p1, out p2, ref line);
                isec.Set(line, p1, p2);
                if (i == 1)
                {
                    pf1 = pf1.WithCoord(u1, v1);
                    pf2 = pf2.WithCoord(u2, v2);
                }
                if (i == nbp)
                {
                    pl1 = pl1.WithCoord(u1, v1);
                    pl2 = pl2.WithCoord(u2, v2);
                }
            }

            data.SetSimul(sec);
            data.Set2dPoints(pf1, pl1, pf2, pl2);
            ChFi3d_FilCommonPoint(lin.StartPointOnFirst(), lin.TransitionOnS1(), true, ref data.ChangeVertexFirstOnS1(),
                                                        _tolapp3d);
            ChFi3d_FilCommonPoint(lin.EndPointOnFirst(), lin.TransitionOnS1(), false, ref data.ChangeVertexLastOnS1(),
                                                        _tolapp3d);
            ChFi3d_FilCommonPoint(lin.StartPointOnSecond(), lin.TransitionOnS2(), true, ref data.ChangeVertexFirstOnS2(),
                                                        _tolapp3d);
            ChFi3d_FilCommonPoint(lin.EndPointOnSecond(), lin.TransitionOnS2(), false, ref data.ChangeVertexLastOnS2(),
                                                        _tolapp3d);

            bool reverse = (!forward || inside);
            if (intf != 0 && reverse)
            {
                bool ok = false;
                CommonPoint cp1 = data.VertexFirstOnS1();
                if (cp1.IsOnArc)
                {
                    Face f1 = s1.Face();
                    Face bid = new();
                    intf = !SearchFace(spine, cp1, f1, ref bid) ? 1 : 0;
                    ok = intf != 0;
                }
                CommonPoint cp2 = data.VertexFirstOnS2();
                if (cp2.IsOnArc && !ok)
                {
                    Face f2 = s2.Face();
                    Face bid = new();
                    intf = !SearchFace(spine, cp2, f2, ref bid) ? 1 : 0;
                }
            }
            if (intl != 0)
            {
                bool ok = false;
                CommonPoint cp1 = data.VertexLastOnS1();
                if (cp1.IsOnArc)
                {
                    Face f1 = s1.Face();
                    Face bid = new();
                    intl = !SearchFace(spine, cp1, f1, ref bid) ? 1 : 0;
                    ok = intl != 0;
                }
                CommonPoint cp2 = data.VertexLastOnS2();
                if (cp2.IsOnArc && !ok)
                {
                    Face f2 = s2.Face();
                    Face bid = new();
                    intl = !SearchFace(spine, cp2, f2, ref bid) ? 1 : 0;
                }
            }
        }
        else if (chsp.IsChamfer == ChamfMethod.TwoDist)
        {
            double dis1 = 0.0, dis2 = 0.0;
            chsp.Dists(out dis1, out dis2);
            radius = Max(dis1, dis2);
            radius = Max(radius, radiusspine);
            locfleche = radius * 1.0e-2; // graphic criterion

            GenChamfer? pFunc = null; // std::unique_ptr in OCCT
            GenChamfInv? pFInv = null; // std::unique_ptr in OCCT
            if (chsp.Mode() == ChamfMode.ClassicChamfer)
            {
                pFunc = new Chamfer(s1, s2, hGuide);
                pFInv = new ChamfInv(s1, s2, hGuide);
                pFunc.Set(dis1, dis2, choix);
                pFInv.Set(dis1, dis2, choix);
            }
            else
            {
                NCollection_List<ElSpine> ll = spine.ChangeElSpines();
                NCollection_List<ElSpine> ll_offset = spine.ChangeOffsetElSpines();
                NCollection_List<ElSpine>.Iterator iles = new(ll), ILES_offset = new(ll_offset);
                for (; iles.More(); iles.Next(), ILES_offset.Next())
                {
                    ElSpine? hElSpine = iles.Value();
                    if (hElSpine == hGuide)
                    {
                        offsetHGuide = ILES_offset.Value();
                    }
                }

                if (offsetHGuide is null)
                {
                    // OCCT prints "Construction of offset guide failed!" to std::cout here
                    // exception
                }
                pFunc = new ConstThroatWithPenetration(s1, s2, offsetHGuide);
                pFInv = new ConstThroatWithPenetrationInv(s1, s2, offsetHGuide);
                double throat = Max(dis1, dis2);
                pFunc.Set(throat, throat, choix);
                pFInv.Set(throat, throat, choix);
            }

            _done = SimulData(data, hGuide, offsetHGuide, ref lin, s1, i1, s2, i2, pFunc, pFInv, pFirst, maxStep, locfleche,
                                              tolGuide, ref first, ref last, inside, appro, forward, soldep, 4, recOnS1, recOnS2);

            if (!_done)
            {
                return false;
            }
            NCollection_HArray1<CircSection>? sec = null;
            Point2D pf1 = new(), pl1 = new(), pf2 = new(), pl2 = new();

            int nbp = lin.NbPoints();
            sec = new NCollection_HArray1<CircSection>(1, nbp);
            for (i = 1; i <= nbp; i++)
            {
                ref CircSection isec = ref sec.ChangeValue(i);
                double u1 = 0.0, v1 = 0.0, u2 = 0.0, v2 = 0.0, ww = 0.0, p1 = 0.0, p2 = 0.0;
                Line3D line = new();
                BlendPoint p = lin.Point(i);
                p.ParametersOnS1(out u1, out v1);
                p.ParametersOnS2(out u2, out v2);
                ww = p.Parameter();
                pFunc.Section(ww, u1, v1, u2, v2, out p1, out p2, ref line);
                isec.Set(line, p1, p2);
                if (i == 1)
                {
                    pf1 = pf1.WithCoord(u1, v1);
                    pf2 = pf2.WithCoord(u2, v2);
                }
                if (i == nbp)
                {
                    pl1 = pl1.WithCoord(u1, v1);
                    pl2 = pl2.WithCoord(u2, v2);
                }
            }

            data.SetSimul(sec);
            data.Set2dPoints(pf1, pl1, pf2, pl2);
            ChFi3d_FilCommonPoint(lin.StartPointOnFirst(), lin.TransitionOnS1(), true, ref data.ChangeVertexFirstOnS1(),
                                                        _tolapp3d);
            ChFi3d_FilCommonPoint(lin.EndPointOnFirst(), lin.TransitionOnS1(), false, ref data.ChangeVertexLastOnS1(),
                                                        _tolapp3d);
            ChFi3d_FilCommonPoint(lin.StartPointOnSecond(), lin.TransitionOnS2(), true, ref data.ChangeVertexFirstOnS2(),
                                                        _tolapp3d);
            ChFi3d_FilCommonPoint(lin.EndPointOnSecond(), lin.TransitionOnS2(), false, ref data.ChangeVertexLastOnS2(),
                                                        _tolapp3d);

            bool reverse = (!forward || inside);
            if (intf != 0 && reverse)
            {
                bool ok = false;
                CommonPoint cp1 = data.VertexFirstOnS1();
                if (cp1.IsOnArc)
                {
                    Face f1 = s1.Face();
                    Face bid = new();
                    intf = !SearchFace(spine, cp1, f1, ref bid) ? 1 : 0;
                    ok = intf != 0;
                }
                CommonPoint cp2 = data.VertexFirstOnS2();
                if (cp2.IsOnArc && !ok)
                {
                    Face f2 = s2.Face();
                    Face bid = new();
                    intf = !SearchFace(spine, cp2, f2, ref bid) ? 1 : 0;
                }
            }
            if (intl != 0)
            {
                bool ok = false;
                CommonPoint cp1 = data.VertexLastOnS1();
                if (cp1.IsOnArc)
                {
                    Face f1 = s1.Face();
                    Face bid = new();
                    intl = !SearchFace(spine, cp1, f1, ref bid) ? 1 : 0;
                    ok = intl != 0;
                }
                CommonPoint cp2 = data.VertexLastOnS2();
                if (cp2.IsOnArc && !ok)
                {
                    Face f2 = s2.Face();
                    Face bid = new();
                    intl = !SearchFace(spine, cp2, f2, ref bid) ? 1 : 0;
                }
            }
        }
        else
        { // distance and angle
            double dis = 0.0, angle = 0.0;
            chsp.GetDistAngle(out dis, out angle);
            radius = Max(dis, dis * Math.Tan(angle));
            radius = Max(radius, radiusspine);
            locfleche = radius * 1.0e-2; // graphic criterion

            int ch = choix;

            ChAsym func = new(s1, s2, hGuide);
            ChAsymInv fInv = new(s1, s2, hGuide);

            func.Set(dis, angle, ch);
            fInv.Set(dis, angle, ch);

            _done = SimulData(data, hGuide, offsetHGuide, ref lin, s1, i1, s2, i2, func, fInv, pFirst, maxStep, locfleche,
                                              tolGuide, ref first, ref last, inside, appro, forward, soldep, 4, recOnS1, recOnS2);

            if (!_done)
            {
                return false;
            }
            NCollection_HArray1<CircSection>? sec = null;
            Point2D pf1 = new(), pl1 = new(), pf2 = new(), pl2 = new();

            int nbp = lin.NbPoints();
            sec = new NCollection_HArray1<CircSection>(1, nbp);
            for (i = 1; i <= nbp; i++)
            {
                ref CircSection isec = ref sec.ChangeValue(i);
                double u1 = 0.0, v1 = 0.0, u2 = 0.0, v2 = 0.0, ww = 0.0, p1 = 0.0, p2 = 0.0;
                Line3D line = new();
                BlendPoint p = lin.Point(i);
                p.ParametersOnS1(out u1, out v1);
                p.ParametersOnS2(out u2, out v2);
                ww = p.Parameter();
                func.Section(ww, u1, v1, u2, v2, out p1, out p2, ref line);
                isec.Set(line, p1, p2);
                if (i == 1)
                {
                    pf1 = pf1.WithCoord(u1, v1);
                    pf2 = pf2.WithCoord(u2, v2);
                }
                if (i == nbp)
                {
                    pl1 = pl1.WithCoord(u1, v1);
                    pl2 = pl2.WithCoord(u2, v2);
                }
            }

            data.SetSimul(sec);
            data.Set2dPoints(pf1, pl1, pf2, pl2);
            ChFi3d_FilCommonPoint(lin.StartPointOnFirst(), lin.TransitionOnS1(), true, ref data.ChangeVertexFirstOnS1(),
                                                        _tolapp3d);
            ChFi3d_FilCommonPoint(lin.EndPointOnFirst(), lin.TransitionOnS1(), false, ref data.ChangeVertexLastOnS1(),
                                                        _tolapp3d);
            ChFi3d_FilCommonPoint(lin.StartPointOnSecond(), lin.TransitionOnS2(), true, ref data.ChangeVertexFirstOnS2(),
                                                        _tolapp3d);
            ChFi3d_FilCommonPoint(lin.EndPointOnSecond(), lin.TransitionOnS2(), false, ref data.ChangeVertexLastOnS2(),
                                                        _tolapp3d);

            bool reverse = (!forward || inside);
            if (intf != 0 && reverse)
            {
                bool ok = false;
                CommonPoint cp1 = data.VertexFirstOnS1();
                if (cp1.IsOnArc)
                {
                    Face f1 = s1.Face();
                    Face bid = new();
                    intf = !SearchFace(spine, cp1, f1, ref bid) ? 1 : 0;
                    ok = intf != 0;
                }
                CommonPoint cp2 = data.VertexFirstOnS2();
                if (cp2.IsOnArc && !ok)
                {
                    Face f2 = s2.Face();
                    Face bid = new();
                    intf = !SearchFace(spine, cp2, f2, ref bid) ? 1 : 0;
                }
            }

            if (intl != 0)
            {
                bool ok = false;
                CommonPoint cp1 = data.VertexLastOnS1();
                if (cp1.IsOnArc)
                {
                    Face f1 = s1.Face();
                    Face bid = new();
                    intl = !SearchFace(spine, cp1, f1, ref bid) ? 1 : 0;
                    ok = intl != 0;
                }
                CommonPoint cp2 = data.VertexLastOnS2();
                if (cp2.IsOnArc && !ok)
                {
                    Face f2 = s2.Face();
                    Face bid = new();
                    intl = !SearchFace(spine, cp2, f2, ref bid) ? 1 : 0;
                }
            }
        } // distance and angle
        return true;
    }

    internal override void SimulSurf(SurfData? _unused1, ElSpine? _unused2, Spine? _unused3, int _unused4, FaceSurfaceAdaptor? _unused5, TopolTool? _unused6, EdgeCurve2DAdaptor? _unused7, FaceSurfaceAdaptor? _unused8, EdgeCurve2DAdaptor? _unused9, ref bool _unused10, FaceSurfaceAdaptor? _unused11, TopolTool? _unused12, Orientation _unused13, double _unused14, double _unused15, ref double _unused16, ref double _unused17, bool _unused18, bool _unused19, bool _unused20, bool _unused21, bool _unused22, bool _unused23, math_Vector _unused24)
    {
        throw new FailureException("SimulSurf Not Implemented");
    }

    internal override void SimulSurf(SurfData? _unused25, ElSpine? _unused26, Spine? _unused27, int _unused28, FaceSurfaceAdaptor? _unused29, TopolTool? _unused30, Orientation _unused31, FaceSurfaceAdaptor? _unused32, TopolTool? _unused33, EdgeCurve2DAdaptor? _unused34, FaceSurfaceAdaptor? _unused35, EdgeCurve2DAdaptor? _unused36, ref bool _unused37, double _unused38, double _unused39, ref double _unused40, ref double _unused41, bool _unused42, bool _unused43, bool _unused44, bool _unused45, bool _unused46, bool _unused47, math_Vector _unused48)
    {
        throw new FailureException("SimulSurf Not Implemented");
    }

    internal override void SimulSurf(SurfData? _unused49, ElSpine? _unused50, Spine? _unused51, int _unused52, FaceSurfaceAdaptor? _unused53, TopolTool? _unused54, EdgeCurve2DAdaptor? _unused55, FaceSurfaceAdaptor? _unused56, EdgeCurve2DAdaptor? _unused57, ref bool _unused58, Orientation _unused59, FaceSurfaceAdaptor? _unused60, TopolTool? _unused61, EdgeCurve2DAdaptor? _unused62, FaceSurfaceAdaptor? _unused63, EdgeCurve2DAdaptor? _unused64, ref bool _unused65, Orientation _unused66, double _unused67, double _unused68, ref double _unused69, ref double _unused70, bool _unused71, bool _unused72, bool _unused73, bool _unused74, bool _unused75, bool _unused76, bool _unused77, math_Vector _unused78)
    {
        throw new FailureException("SimulSurf Not Implemented");
    }

    //------------------------MODIFS---------------------------------------
    //=======================================================================
    // function : PerformFirstSection
    // purpose  : to implement the first section if there is no KPart
    //=======================================================================

    internal override bool PerformFirstSection(Spine? spine, ElSpine? hGuide, int choix, FaceSurfaceAdaptor? s1, FaceSurfaceAdaptor? s2, TopolTool? i1, TopolTool? i2, double par, math_Vector solDep, ref Containment pos1, ref Containment pos2)
    {
        ChamfSpine? chsp = (spine as ChamfSpine);

        if (chsp is null)
        {
            throw new ConstructionErrorException("PerformSurf : this is not the spine of a chamfer");
        }

        double tolGuide = hGuide.Resolution(_tolapp3d);

        if (chsp.IsChamfer == ChamfMethod.Sym)
        {
            double dis = 0.0;
            chsp.GetDist(out dis);

            GenChamfer? pFunc = null; // std::unique_ptr in OCCT
            if (chsp.Mode() == ChamfMode.ClassicChamfer)
            {
                pFunc = new Chamfer(s1, s2, hGuide);
            }
            else
            {
                pFunc = new ConstThroat(s1, s2, hGuide);
            }
            pFunc.Set(dis, dis, choix);
            Walking theWalk = new(s1, s2, i1, i2, hGuide);

            // calculate an approximate starting solution
            Vector3D tgF = new(), tgL = new(), tmp1 = new(), tmp2 = new(), d1gui = new();
            Point3D pt1 = new(), pt2 = new(), ptgui = new();
            Xyz temp = new();

            hGuide.D1(par, out ptgui, out d1gui);
            //  ptgui = S1->Value(SolDep[1],SolDep[2]);

            pFunc.Set(par);
            pFunc.Tangent(solDep[1], solDep[2], solDep[3], solDep[4], out tgF, out tgL, out tmp1, out tmp2);

            bool rev1 = false;
            bool rev2 = false;
            double sign = (tgF.Crossed(d1gui)).Dot(tgL);

            if (choix % 2 == 1)
            {
                rev1 = true;
            }
            else
            {
                rev2 = true;
            }

            if (sign < 0.0)
            {
                rev1 = !rev1;
                rev2 = !rev2;
            }

            if (rev1)
            {
                tgF = tgF.Reversed();
            }
            if (rev2)
            {
                tgL = tgL.Reversed();
            }

            temp = (tgF.XYZ()).Multiplied(dis);
            pt1 = pt1.WithXYZ((ptgui.XYZ()).Added(temp));
            temp = (tgL.XYZ()).Multiplied(dis);
            pt2 = pt2.WithXYZ((ptgui.XYZ()).Added(temp));

            double tol = _tolesp * 1.0e2;
            //    double u,v;
            GenLocateExtPS proj1 = new(s1, tol, tol);
            proj1.Perform(pt1, solDep[1], solDep[2]);
            GenLocateExtPS proj2 = new(s2, tol, tol);
            proj2.Perform(pt2, solDep[3], solDep[4]);

            if (proj1.IsDone)
            {
                (proj1.Point()).Parameter(out solDep[1], out solDep[2]);
            }
            if (proj2.IsDone)
            {
                (proj2.Point()).Parameter(out solDep[3], out solDep[4]);
            }

            return theWalk.PerformFirstSection(pFunc, par, solDep, _tolapp3d, tolGuide, out pos1, out pos2);
        }
        else if (chsp.IsChamfer == ChamfMethod.TwoDist)
        {
            double dis1 = 0.0, dis2 = 0.0;
            chsp.Dists(out dis1, out dis2);

            GenChamfer? pFunc = null; // std::unique_ptr in OCCT
            if (chsp.Mode() == ChamfMode.ClassicChamfer)
            {
                pFunc = new Chamfer(s1, s2, hGuide);
                pFunc.Set(dis1, dis2, choix);
            }
            else
            {
                ElSpine? offsetHGuide = null;
                NCollection_List<ElSpine> ll = spine.ChangeElSpines();
                NCollection_List<ElSpine> ll_offset = spine.ChangeOffsetElSpines();
                NCollection_List<ElSpine>.Iterator iles = new(ll), ILES_offset = new(ll_offset);
                for (; iles.More(); iles.Next(), ILES_offset.Next())
                {
                    ElSpine? hElSpine = iles.Value();
                    if (hElSpine == hGuide)
                    {
                        offsetHGuide = ILES_offset.Value();
                    }
                }

                if (offsetHGuide is null)
                {
                    // OCCT prints "Construction of offset guide failed!" to std::cout here
                    // exception
                }
                pFunc = new ConstThroatWithPenetration(s1, s2, offsetHGuide);
                double throat = Max(dis1, dis2);
                pFunc.Set(throat, throat, choix); // dis2?
            }
            Walking theWalk = new(s1, s2, i1, i2, hGuide);

            // calculate an approximate starting solution
            Vector3D tgF = new(), tgL = new(), tmp1 = new(), tmp2 = new(), d1gui = new();
            Point3D pt1 = new(), pt2 = new(), ptgui = new();
            Xyz temp = new();

            hGuide.D1(par, out ptgui, out d1gui);
            //  ptgui = S1->Value(SolDep[1],SolDep[2]);

            pFunc.Set(par);
            pFunc.Tangent(solDep[1], solDep[2], solDep[3], solDep[4], out tgF, out tgL, out tmp1, out tmp2);

            bool rev1 = false;
            bool rev2 = false;
            double sign = (tgF.Crossed(d1gui)).Dot(tgL);

            if (choix % 2 == 1)
            {
                rev1 = true;
            }
            else
            {
                rev2 = true;
            }

            if (sign < 0.0)
            {
                rev1 = !rev1;
                rev2 = !rev2;
            }

            if (rev1)
            {
                tgF = tgF.Reversed();
            }
            if (rev2)
            {
                tgL = tgL.Reversed();
            }

            double dist1 = dis1, dist2 = dis2;
            if (chsp.Mode() == ChamfMode.ConstThroatWithPenetrationChamfer)
            {
                /*
                double Alpha = TgF.Angle(TgL);
                double SinAlpha = Math.Sin(Alpha);
                double CosAlpha = Math.Cos(Alpha);
                double TanAlpha = Math.Tan(Alpha);
                double dis1dis1 = dis1*dis1, dis2dis2 = dis2*dis2;
                aDist2 = Math.Sqrt(dis1dis1 - dis2dis2) - dis2/TanAlpha;
                double CosBeta = Math.Sqrt(1-dis2dis2/dis1dis1)*CosAlpha + dis2/dis1*SinAlpha;
                double FullDist1 = dis1/CosBeta;
                aDist1 = FullDist1 - dis2/SinAlpha;
                */
            }

            temp = (tgF.XYZ()).Multiplied(dist1);
            pt1 = pt1.WithXYZ((ptgui.XYZ()).Added(temp));
            temp = (tgL.XYZ()).Multiplied(dist2);
            pt2 = pt2.WithXYZ((ptgui.XYZ()).Added(temp));

            double tol = _tolesp * 1.0e2;
            //    double u,v;

            GenLocateExtPS proj1 = new(s1, tol, tol);
            proj1.Perform(pt1, solDep[1], solDep[2]);
            GenLocateExtPS proj2 = new(s2, tol, tol);
            proj2.Perform(pt2, solDep[3], solDep[4]);

            if (proj1.IsDone)
            {
                (proj1.Point()).Parameter(out solDep[1], out solDep[2]);
            }
            if (proj2.IsDone)
            {
                (proj2.Point()).Parameter(out solDep[3], out solDep[4]);
            }

            return theWalk.PerformFirstSection(pFunc, par, solDep, _tolapp3d, tolGuide, out pos1, out pos2);
        }
        else
        { // distance and angle
            double dis1 = 0.0, angle = 0.0;
            chsp.GetDistAngle(out dis1, out angle);

            int ch = choix;

            ChAsym func = new(s1, s2, hGuide);
            func.Set(dis1, angle, ch);
            Walking theWalk = new(s1, s2, i1, i2, hGuide);

            // calculate an approximate starting solution
            Vector3D tgF = new(), tgL = new(), tmp1 = new(), tmp2 = new(), d1gui = new();
            Point3D pt1 = new(), pt2 = new(), ptgui = new();
            Xyz temp = new();

            hGuide.D1(par, out ptgui, out d1gui);
            //  ptgui = S1->Value(SolDep[1],SolDep[2]);

            func.Set(par);
            func.Tangent(solDep[1], solDep[2], solDep[3], solDep[4], out tgF, out tgL, out tmp1, out tmp2);

            bool rev1 = false;
            bool rev2 = false;
            double sign = (tgF.Crossed(d1gui)).Dot(tgL);

            if (ch % 2 == 1)
            {
                rev1 = true;
            }
            else
            {
                rev2 = true;
            }

            if (sign < 0.0)
            {
                rev1 = !rev1;
                rev2 = !rev2;
            }

            if (rev1)
            {
                tgF = tgF.Reversed();
            }
            if (rev2)
            {
                tgL = tgL.Reversed();
            }

            temp = (tgF.XYZ()).Multiplied(dis1);
            pt1 = pt1.WithXYZ((ptgui.XYZ()).Added(temp));

            double dis2 = 0.0, tmpcos = 0.0, tmpsin = 0.0;
            tmpcos = tgF.Dot(tgL);
            tmpsin = Math.Sqrt(1.0 - tmpcos * tmpcos);

            dis2 = dis1 / (tmpcos + tmpsin / Math.Tan(angle));

            temp = (tgL.XYZ()).Multiplied(dis2);
            pt2 = pt2.WithXYZ((ptgui.XYZ()).Added(temp));

            double tol = _tolesp * 1.0e2;
            //      double u,v;
            GenLocateExtPS proj1 = new(s1, tol, tol);
            proj1.Perform(pt1, solDep[1], solDep[2]);
            GenLocateExtPS proj2 = new(s2, tol, tol);
            proj2.Perform(pt2, solDep[3], solDep[4]);
            if (proj1.IsDone)
            {
                (proj1.Point()).Parameter(out solDep[1], out solDep[2]);
            }
            if (proj2.IsDone)
            {
                (proj2.Point()).Parameter(out solDep[3], out solDep[4]);
            }

            return theWalk.PerformFirstSection(func, par, solDep, _tolapp3d, tolGuide, out pos1, out pos2);
        } // distance and angle
    }

    //=================================================================================================

    internal override bool PerformSurf(NCollection_Sequence<SurfData> seqData, ElSpine? hGuide, Spine? spine, int choix, FaceSurfaceAdaptor? s1, TopolTool? i1, FaceSurfaceAdaptor? s2, TopolTool? i2, double maxStep, double fleche, double tolGuide, ref double first, ref double last, bool inside, bool appro, bool forward, bool recOnS1, bool recOnS2, math_Vector soldep, ref int intf, ref int intl)
    {
        SurfData? data = seqData[1];
        ChamfSpine? chsp = (spine as ChamfSpine);

        if (chsp is null)
        {
            throw new ConstructionErrorException("PerformSurf : this is not the spine of a chamfer");
        }

        bool gd1 = false, gd2 = false, gf1 = false, gf2 = false;
        global::Kerf.Engine.Fillets.Blend.Line? lin = null;
        Orientation or = s1.Face().Orientation();
        double pFirst = first;
        if (intf != 0)
        {
            first = chsp.FirstParameter(1);
        }
        if (intl != 0)
        {
            last = chsp.LastParameter(chsp.EdgeCount);
        }

        if (chsp.IsChamfer == ChamfMethod.Sym)
        {

            GenChamfer? pFunc = null; // std::unique_ptr in OCCT
            GenChamfInv? pFInv = null; // std::unique_ptr in OCCT
            if (chsp.Mode() == ChamfMode.ClassicChamfer)
            {
                pFunc = new Chamfer(s1, s2, hGuide);
                pFInv = new ChamfInv(s1, s2, hGuide);
            }
            else
            {
                pFunc = new ConstThroat(s1, s2, hGuide);
                pFInv = new ConstThroatInv(s1, s2, hGuide);
            }
            double dis = 0.0;
            chsp.GetDist(out dis);
            pFunc.Set(dis, dis, choix);
            pFInv.Set(dis, dis, choix);

            _done = ComputeData(data, hGuide, spine, ref lin, s1, i1, s2, i2, pFunc, pFInv, pFirst, maxStep, fleche,
                                                  tolGuide, ref first, ref last, inside, appro, forward, soldep, ref intf, ref intl, ref gd1,
                                                  ref gd2, ref gf1, ref gf2, recOnS1, recOnS2);
            if (!_done)
            {
                return false; // ratrappage possible PMN 14/05/1998
            }
            _done = CompleteData(data, pFunc, lin, s1, s2, or, gd1, gd2, gf1, gf2);
            if (!_done)
            {
                throw new FailureException("PerformSurf : Fail of approximation!");
            }
        }
        else if (chsp.IsChamfer == ChamfMethod.TwoDist)
        {
            double d1 = 0.0, d2 = 0.0;
            chsp.Dists(out d1, out d2);

            GenChamfer? pFunc = null; // std::unique_ptr in OCCT
            GenChamfInv? pFInv = null; // std::unique_ptr in OCCT
            if (chsp.Mode() == ChamfMode.ClassicChamfer)
            {
                pFunc = new Chamfer(s1, s2, hGuide);
                pFInv = new ChamfInv(s1, s2, hGuide);
                pFunc.Set(d1, d2, choix);
                pFInv.Set(d1, d2, choix);
            }
            else
            {
                ElSpine? offsetHGuide = null;
                NCollection_List<ElSpine> ll = spine.ChangeElSpines();
                NCollection_List<ElSpine> ll_offset = spine.ChangeOffsetElSpines();
                NCollection_List<ElSpine>.Iterator iles = new(ll), ILES_offset = new(ll_offset);
                for (; iles.More(); iles.Next(), ILES_offset.Next())
                {
                    ElSpine? hElSpine = iles.Value();
                    if (hElSpine == hGuide)
                    {
                        offsetHGuide = ILES_offset.Value();
                    }
                }

                if (offsetHGuide is null)
                {
                    // OCCT prints "Construction of offset guide failed!" to std::cout here
                    // exception
                }
                pFunc = new ConstThroatWithPenetration(s1, s2, offsetHGuide);
                pFInv = new ConstThroatWithPenetrationInv(s1, s2, offsetHGuide);
                double throat = Max(d1, d2);
                pFunc.Set(throat, throat, choix);
                pFInv.Set(throat, throat, choix);
            }

            _done = ComputeData(data, hGuide, spine, ref lin, s1, i1, s2, i2, pFunc, pFInv, pFirst, maxStep, fleche,
                                                  tolGuide, ref first, ref last, inside, appro, forward, soldep, ref intf, ref intl, ref gd1,
                                                  ref gd2, ref gf1, ref gf2, recOnS1, recOnS2);
            if (!_done)
            {
                return false; // ratrappage possible PMN 14/05/1998
            }
            _done = CompleteData(data, pFunc, lin, s1, s2, or, gd1, gd2, gf1, gf2);
            if (!_done)
            {
                throw new FailureException("PerformSurf : Fail of approximation!");
            }
        }
        else
        { // distance and angle
            double d1 = 0.0, angle = 0.0;
            chsp.GetDistAngle(out d1, out angle);

            int ch = choix;

            ChAsym func = new(s1, s2, hGuide);
            ChAsymInv fInv = new(s1, s2, hGuide);
            func.Set(d1, angle, ch);
            fInv.Set(d1, angle, ch);

            _done = ComputeData(data, hGuide, spine, ref lin, s1, i1, s2, i2, func, fInv, pFirst, maxStep, fleche, tolGuide,
                                                  ref first, ref last, inside, appro, forward, soldep, ref intf, ref intl, ref gd1, ref gd2,
                                                  ref gf1, ref gf2, recOnS1, recOnS2);

            if (!_done)
            {
                return false; // ratrappage possible PMN 14/05/1998
            }
            _done = CompleteData(data, func, lin, s1, s2, or, gd1, gd2, gf1, gf2);
            if (!_done)
            {
                throw new FailureException("PerformSurf : Fail of approximation!");
            }
        }

        return true;
    }

    internal override void PerformSurf(NCollection_Sequence<SurfData> _unused79, ElSpine? _unused80, Spine? _unused81, int _unused82, FaceSurfaceAdaptor? _unused83, TopolTool? _unused84, EdgeCurve2DAdaptor? _unused85, FaceSurfaceAdaptor? _unused86, EdgeCurve2DAdaptor? _unused87, ref bool _unused88, FaceSurfaceAdaptor? _unused89, TopolTool? _unused90, Orientation _unused91, double _unused92, double _unused93, double _unused94, ref double _unused95, ref double _unused96, bool _unused97, bool _unused98, bool _unused99, bool _unused100, bool _unused101, bool _unused102, math_Vector _unused103)
    {
        throw new FailureException("PerformSurf Not Implemented");
    }

    internal override void PerformSurf(NCollection_Sequence<SurfData> _unused104, ElSpine? _unused105, Spine? _unused106, int _unused107, FaceSurfaceAdaptor? _unused108, TopolTool? _unused109, Orientation _unused110, FaceSurfaceAdaptor? _unused111, TopolTool? _unused112, EdgeCurve2DAdaptor? _unused113, FaceSurfaceAdaptor? _unused114, EdgeCurve2DAdaptor? _unused115, ref bool _unused116, double _unused117, double _unused118, double _unused119, ref double _unused120, ref double _unused121, bool _unused122, bool _unused123, bool _unused124, bool _unused125, bool _unused126, bool _unused127, math_Vector _unused128)
    {
        throw new FailureException("PerformSurf Not Implemented");
    }

    internal override void PerformSurf(NCollection_Sequence<SurfData> _unused129, ElSpine? _unused130, Spine? _unused131, int _unused132, FaceSurfaceAdaptor? _unused133, TopolTool? _unused134, EdgeCurve2DAdaptor? _unused135, FaceSurfaceAdaptor? _unused136, EdgeCurve2DAdaptor? _unused137, ref bool _unused138, Orientation _unused139, FaceSurfaceAdaptor? _unused140, TopolTool? _unused141, EdgeCurve2DAdaptor? _unused142, FaceSurfaceAdaptor? _unused143, EdgeCurve2DAdaptor? _unused144, ref bool _unused145, Orientation _unused146, double _unused147, double _unused148, double _unused149, ref double _unused150, ref double _unused151, bool _unused152, bool _unused153, bool _unused154, bool _unused155, bool _unused156, bool _unused157, bool _unused158, math_Vector _unused159)
    {
        throw new FailureException("PerformSurf Not Implemented");
    }

    //=======================================================================
    // function : ExtentOneCorner
    // purpose  : extends the spine of the stripe S on the side of the vertex V
    // PMN : 28/11/97 : Reproduces the code of fillets, and it seems to work better...
    //=======================================================================

    internal override void ExtentOneCorner(in Vertex v, Stripe? s)
    {
        int sens = 0;
        double coeff = 0.5;
        Spine? spine = s.Spine();
        ChFi3d_IndexOfSurfData(v, s, out sens);
        if (spine.IsTangencyExtremity((sens == 1)))
        {
            return; // No extension on queue
        }
        double dU = spine.LastParameter(spine.EdgeCount);
        if (sens == 1)
        {
            spine.SetFirstParameter(-dU * coeff);
            spine.SetFirstTgt(0.0);
        }
        else
        {
            spine.SetLastParameter(dU * (1.0 + coeff));
            spine.SetLastTgt(dU);
        }
    }

    //=======================================================================
    // function : ExtentTwoCorner
    // purpose  : extends the spines of the stripes contained in the list LS,
    //           on the side of the vertex V
    //=======================================================================

    internal override void ExtentTwoCorner(in Vertex v, NCollection_List<Stripe> ls)
    {
        int sens = 0;
        NCollection_List<Stripe>.Iterator itel = new(ls);
        bool ff = true;
        bool[] isfirst = new bool[2];
        int[] iedge = new int[2];
        iedge[0] = 1;
        iedge[1] = 1;
        Stripe[] stripe = new Stripe[2];
        Spine[] spine = new Spine[2];

        int i = 0;
        for (; itel.More(); itel.Next(), i++)
        {
            ChFi3d_IndexOfSurfData(v, itel.Value(), out sens);
            if (!ff)
            {
                if (stripe[1] == itel.Value())
                {
                    sens = -sens;
                }
            }

            stripe[i] = itel.Value();
            isfirst[i] = (sens == 1);
            spine[i] = stripe[i].Spine();
            if (!isfirst[i])
            {
                iedge[i] = spine[i].EdgeCount;
            }
            ff = false;
        }

        ChamfSpine[] chsp = new ChamfSpine[2];
        double[] d = new double[4], dis = { 0.0, 0.0 };
        int j = 0;
        Face[] f = new Face[4];
        double tmpang = 0.0, tmd = 0.0;

        for (i = 0, j = 0; i < 2; i++, j += 2)
        {
            chsp[i] = (spine[i] as ChamfSpine);
            ConexFaces(spine[i], iedge[i], ref f[j], ref f[j + 1]);

            if (chsp[i].IsChamfer == ChamfMethod.Sym)
            {
                chsp[i].GetDist(out d[j]);
                d[j + 1] = d[j];
            }
            else if (chsp[i].IsChamfer == ChamfMethod.TwoDist)
            {
                chsp[i].Dists(out d[j], out d[j + 1]);
            }
            else
            {
                chsp[i].GetDistAngle(out tmd, out tmpang);
                // an approximate calculation of distance 2 is done
                d[j] = tmd;
                d[j + 1] = tmd * Math.Tan(tmpang);
            }
        }

        bool notfound = true;
        i = 0;
        while (notfound && (i < 2))
        {
            j = 0;
            while (notfound && (j < 2))
            {
                if (f[i].IsSame(f[j + 2]))
                {
                    dis[0] = d[i];
                    //	dOnArc[0] = d[(i+1)%2];

                    dis[1] = d[j + 2];
                    //	dOnArc[1] = d[(j+1)%2 + 2];
                    notfound = false;
                }
                j++;
            }
            i++;
        }
        // ExtentTwoCorner

        State[] state = new State[2];

        for (i = 0; i < 2; i++)
        {
            if (isfirst[i])
            {
                state[i] = spine[i].FirstStatus();
            }
            else
            {
                state[i] = spine[i].LastStatus();
            }
        }

        if (state[0] == global::Kerf.Engine.Fillets.Data.State.AllSame)
        {
            /*
                  // The greatest intersection of the chamfer is found (on the incident edge)
                    // with the face at end
                    i = 0;
                    j = 1;
                    if(dOnArc[j] > dOnArc[i]) {
                        int temp = i;
                        i = j;
                        j = temp;
                    }
                    ExtentOneCorner( V, Stripe[i] ); */

            // it is necessary that two chamfers touch the face at end
            for (j = 0; j < 2; j++)
            {
                ExtentOneCorner(v, stripe[j]);
            }
        }
        else if ((state[0] == global::Kerf.Engine.Fillets.Data.State.OnSame) && (state[1] == global::Kerf.Engine.Fillets.Data.State.OnSame))
        {

            ExtentSpineOnCommonFace(spine[0], spine[1], v, dis[0], dis[1], isfirst[0], isfirst[1]);
        }
    }

    //=================================================================================================

    internal override void ExtentThreeCorner(in Vertex v, NCollection_List<Stripe> ls)
    {
        int sens = 0;
        NCollection_List<Stripe> check = new();
        bool[] isfirst = new bool[3];
        int[] iedge = new int[3];
        iedge[0] = 1;
        iedge[1] = 1;
        iedge[2] = 1;
        Spine[] spine = new Spine[3];

        int i = 0;
        for (NCollection_List<Stripe>.Iterator itel = new(ls); itel.More();
                  itel.Next(), i++)
        {
            Stripe? stripe = itel.Value();
            ChFi3d_IndexOfSurfData(v, stripe, out sens);
            for (NCollection_List<Stripe>.Iterator ich = new(check); ich.More(); ich.Next())
            {
                if (stripe == ich.Value())
                {
                    sens = -sens;
                    break;
                }
            }

            isfirst[i] = (sens == 1);
            spine[i] = stripe.Spine();
            if (!isfirst[i])
            {
                iedge[i] = spine[i].EdgeCount;
            }

            check.Append(stripe);
        }

        double[,] d = new double[3, 2];
        double tmd, tmpangle;
        int j = 0;
        Face[,] f = new Face[3, 2];

        ChamfSpine[] chsp = new ChamfSpine[3];

        for (i = 0; i < 3; i++)
        {
            chsp[i] = (spine[i] as ChamfSpine);
            ConexFaces(spine[i], iedge[i], ref f[i, 0], ref f[i, 1]);

            if (chsp[i].IsChamfer == ChamfMethod.Sym)
            {
                chsp[i].GetDist(out d[i, 0]);
                d[i, 1] = d[i, 0];
            }
            else if (chsp[i].IsChamfer == ChamfMethod.TwoDist)
            {
                chsp[i].Dists(out d[i, 0], out d[i, 1]);
            }
            else
            {
                chsp[i].GetDistAngle(out tmd, out tmpangle);
                // an approximate calculation of distance 2 is done

                d[i, 0] = tmd;
                d[i, 1] = tmd * Math.Tan(tmpangle);
            }
        }

        // dis[i, j] distance from chamfer i on the common face with
        // chamfer j
        double[,] dis = new double[3, 3];

        for (i = 0; i < 3; i++)
        {
            //    for (int ii=0; ii<3; ii++) {
            //      j = (i+ii)%3;
            j = (i + 1) % 3;
            bool notfound = true;
            int k = 0, l = 0;
            k = 0;
            while (notfound && (k < 2))
            {
                l = 0;
                while (notfound && (l < 2))
                {
                    if (f[i, k].IsSame(f[j, l]))
                    {
                        dis[i, j] = d[i, k];
                        dis[j, i] = d[j, l];
                        notfound = false;
                    }
                    l++;
                }
                k++;
            }
            //    }
        }

        // ExtentThreeCorner
        for (i = 0; i < 3; i++)
        {
            j = (i + 1) % 3;
            ExtentSpineOnCommonFace(spine[i], spine[j], v, dis[i, j], dis[j, i], isfirst[i], isfirst[j]);
        }
    }

    //=================================================================================================

    internal override void SetRegul()
    {
        NCollection_List<Regul>.Iterator it = new();
        NCollection_List<Shape>.Iterator itc = new();
        NCollection_List<Shape>.Iterator its1 = new();
        NCollection_List<Shape>.Iterator its2 = new();
        FaceSurfaceAdaptor s = new();
        EdgeCurve2DAdaptor pc = new();
        double u = 0.0, v = 0.0, t = 0.0;
        Point3D p = new();
        Vector3D n1 = new(), n2 = new(), du = new(), dv = new();
        BrepBuilder b = new();
        double seuil = M_PI / 360.0;
        double seuil2 = seuil * seuil;
        for (it.Initialize(_regul); it.More(); it.Next())
        {
            Regul reg = it.Value();
            itc.Initialize(_coup.NewEdges(reg.Curve()));
            if (itc.More())
            {
                Edge e = ShapeCasts.Edge(itc.Value());
                if (reg.IsSurface1 && reg.IsSurface2)
                {
                    its1.Initialize(_coup.NewFaces(reg.S1()));
                    its2.Initialize(_coup.NewFaces(reg.S2()));
                    if (its1.More() && its2.More())
                    {
                        Face f1 = ShapeCasts.Face(its1.Value());
                        Face f2 = ShapeCasts.Face(its2.Value());
                        s.Initialize(f1, false);
                        pc.Initialize(e, f1);
                        t = 0.5 * (pc.FirstParameter() + pc.LastParameter());
                        pc.Value(t).Coord(out u, out v);
                        s.D1(u, v, out p, out du, out dv);
                        n1 = du.Crossed(dv);

                        s.Initialize(f2, false);
                        pc.Initialize(e, f2);
                        pc.Value(t).Coord(out u, out v);
                        s.D1(u, v, out p, out du, out dv);
                        n2 = du.Crossed(dv);

                        if (n1.SquareMagnitude() > 1.0e-14 && n2.SquareMagnitude() > 1.0e-14)
                        {
                            n1 = n1.Normalized();
                            n2 = n2.Normalized();
                            double sina2 = n1.Crossed(n2).SquareMagnitude();
                            if (sina2 < seuil2)
                            {
                                Continuity cont = ChFi3d_evalconti(e, f1, f2);
                                b.Continuity(e, f1, f2, cont);
                            }
                        }
                    }
                }
            }
        }
    }

    //=======================================================================
    // function : ConexFaces
    // purpose  : F1, F2 are connected to edge so that F1 corresponds to distance
    //=======================================================================

    private void ConexFaces(Spine? spine, int iEdge, ref Face F1, ref Face F2)
    {
        FaceSurfaceAdaptor sb1 = new(), sb2 = new();
        Orientation tmp1 = default, tmp2 = default;
        int rc = 0, choix = 0;
        Face f1 = new(), f2 = new(), ff1 = new(), ff2 = new();

        // calculate the reference orientation
        //  ChFi3d_Builder::StripeOrientations is private
        SearchCommonFaces(_efMap, spine.Edges(1), ref ff1, ref ff2);
        ff1.Orientation(Forward);
        sb1.Initialize(ff1);
        ff2.Orientation(Forward);
        sb2.Initialize(ff2);
        rc = global::Kerf.Engine.Fillets.ChFi3d.ConcaveSide(sb1, sb2, spine.Edges(1), out tmp1, out tmp2);

        // calculate the connected faces
        SearchCommonFaces(_efMap, spine.Edges(iEdge), ref f1, ref f2);
        sb1.Initialize(f1);
        sb2.Initialize(f2);
        choix = global::Kerf.Engine.Fillets.ChFi3d.ConcaveSide(sb1, sb2, spine.Edges(iEdge), out tmp1, out tmp2);

        if (rc % 2 != choix % 2)
        {
            F1 = f2;
            F2 = f1;
        }
        else
        {
            F1 = f1;
            F2 = f2;
        }
    }
}
