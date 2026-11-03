// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_ChBuilder.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_ChBuilder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Occt.ChFi3d_Builder_0;
using static Occt.ChFiDS_ChamfMethod;
using static Occt.ChFiDS_ChamfMode;
using static Occt.ChFiDS_ErrorStatus;
using static Occt.ChFiDS_State;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;
using static Occt.TopOpeBRepDS_Kind;

namespace Occt;

/// <summary>Computes chamfers on the edges of a shape: one distance, two distances, or a distance and an angle.</summary>
/// <remarks>
/// A working object of the fillet algorithms (decision 21), ported over OCCT's files: this file (the header and
/// <c>ChFi3d_ChBuilder.cxx</c>), <c>_C2</c> and <c>_C3</c> (corners). Ported in unit 7N2 so that the builder can be
/// compared with native OCCT end to end; the API wrappers are unit 7N3's. Edges and faces given by the caller are mapped
/// onto the builder's working copy (<see cref="ChFi3d_Builder.ToWork"/>). OCCT_DEBUG blocks (traces) are left out.
/// </remarks>
public partial class ChFi3d_ChBuilder : ChFi3d_Builder
{
    private ChFiDS_ChamfMode myMode;

    /// <summary>Initializes the builder on <paramref name="S"/>.</summary>
    public ChFi3d_ChBuilder(in TopoDS_Shape S, double Ta = 1.0e-2)
        : base(S, Ta)
    {
        myMode = ChFiDS_ClassicChamfer;
    }

    private static void SearchCommonFaces(ChFiDS_Map EFMap, in TopoDS_Edge E, ref TopoDS_Face F1, ref TopoDS_Face F2)
    {
        TopoDS_Face Fc = new();
        NCollection_List<TopoDS_Shape>.Iterator It = new();

        F1.Nullify();
        F2.Nullify();
        for (It.Initialize(EFMap[E]); It.More(); It.Next())
        {
            Fc = TopoDS.Face(It.Value());
            if (F1.IsNull())
            {
                F1 = Fc;
            }
            else if (!Fc.IsSame(F1))
            {
                F2 = Fc;
                break;
            }
        }

        if (!F1.IsNull() && F2.IsNull() && BRepTools.IsReallyClosed(E, F1))
        {
            F2 = F1;
        }
    }

    private static void ExtentSpineOnCommonFace(ChFiDS_Spine? Spine1, ChFiDS_Spine? Spine2, in TopoDS_Vertex V, double dis1, double dis2, bool isfirst1, bool isfirst2)
    {
        double tolesp = 1.0e-7;

        // alpha, the opening angle between two
        // tangents of two guidelines in V is found
        double tga1 = 0.0, tga2 = 0.0;
        double d1plus = 0.0, d2plus = 0.0;

        Point3D tmp = new();
        Vector3D tg1 = new(), tg2 = new();
        Spine1.D1(Spine1.Absc(V), out tmp, out tg1);
        Spine2.D1(Spine2.Absc(V), out tmp, out tg2);
        tg1.Normalize();
        tg2.Normalize();
        if (isfirst1)
        {
            tg1.Reverse();
        }
        if (isfirst2)
        {
            tg2.Reverse();
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
                Spine1.SetFirstParameter(-d1plus);
                Spine1.SetFirstTgt(0.0);
            }
            else
            {
                double param = Spine1.LastParameter(Spine1.NbEdges());
                Spine1.SetLastParameter(d1plus + param);
                Spine1.SetLastTgt(param);
            }
        }
        if (d2plus > 0.0)
        {
            d2plus *= 1.5;
            if (isfirst2)
            {
                Spine2.SetFirstParameter(-d2plus);
                Spine2.SetFirstTgt(0.0);
            }
            else
            {
                double param = Spine2.LastParameter(Spine2.NbEdges());
                Spine2.SetLastParameter(d2plus + param);
                Spine2.SetLastTgt(param);
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

    public void Add(in TopoDS_Edge theE)
    {
        TopoDS_Edge E = TopoDS.Edge(ToWork(theE)); // the working copy's (OCCT.NET)
        TopoDS_Face dummy = new();

        if (Contains(E) == 0 && myEFMap.Contains(E))
        {
            ChFiDS_Stripe? Stripe = new ChFiDS_Stripe();
            ref ChFiDS_Spine? Sp = ref Stripe.ChangeSpine();
            Sp = new ChFiDS_ChamfSpine(tolesp);
            ChFiDS_ChamfSpine? Spine = (Sp as ChFiDS_ChamfSpine);

            TopoDS_Edge E_wnt = E;
            E_wnt.Orientation(TopAbs_FORWARD);
            Spine.SetEdges(E_wnt);
            if (PerformElement(Spine, -1, dummy))
            {
                PerformExtremity(Spine);
                Spine.Load();
                myListStripe.Append(Stripe);
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

    public void Add(double Dis, in TopoDS_Edge theE)
    {
        TopoDS_Edge E = TopoDS.Edge(ToWork(theE)); // the working copy's (OCCT.NET)
        if (Contains(E) == 0 && myEFMap.Contains(E))
        {

            TopoDS_Face dummy = new();

            TopoDS_Edge E_wnt = E;
            E_wnt.Orientation(TopAbs_FORWARD);

            ChFiDS_Stripe? Stripe = new ChFiDS_Stripe();
            ref ChFiDS_Spine? Sp = ref Stripe.ChangeSpine();
            Sp = new ChFiDS_ChamfSpine(tolesp);
            ChFiDS_ChamfSpine? Spine = (Sp as ChFiDS_ChamfSpine);

            Spine.SetMode(myMode);

            Spine.SetEdges(E_wnt);
            if (PerformElement(Spine, -1, dummy))
            {
                Spine.Load();
                myListStripe.Append(Stripe);

                Spine.SetDist(Dis);

                PerformExtremity(Spine);
            }
        }
    }

    //=======================================================================
    // function : SetDist
    // purpose  : set the distances <Dis>of the contour of
    //           index IC
    //=======================================================================

    public void SetDist(double Dis, int IC, in TopoDS_Face theF)
    {
        TopoDS_Face F = TopoDS.Face(ToWork(theF)); // the working copy's (OCCT.NET)

        if (IC <= NbElements())
        {
            ChFiDS_ChamfSpine? csp = (Value(IC) as ChFiDS_ChamfSpine);

            // Search the first edge which has a common face equal to F
            TopoDS_Face F1 = new(), F2 = new(), FirstF1 = new(), FirstF2 = new();
            // TopAbs_Orientation Or1,Or2;
            // int Choix, ChoixConge;
            BRepAdaptor_Surface Sb1 = new(), Sb2 = new();
            int i = 1;
            bool Found = false;
            while ((i <= csp.NbEdges()) && (!Found))
            {
                SearchCommonFaces(myEFMap, csp.Edges(i), ref F1, ref F2);
                if (i == 1)
                {
                    FirstF1 = F1;
                    FirstF2 = F2;
                }
                Found = (F1.IsSame(F) || F2.IsSame(F));
                i++;
            }

            if (Found)
            {
                if (F2.IsSame(F))
                {
                    F2 = F1;
                    F1 = F;
                }
                csp.SetDist(Dis);
            }
            else
            {
                throw new DomainErrorException("the face is not common to any of edges of the contour");
            }
        }
    }

    //=================================================================================================

    public void GetDist(int IC, ref double Dis)
    {
        ChFiDS_ChamfSpine? chsp = (Value(IC) as ChFiDS_ChamfSpine);
        chsp.GetDist(out Dis);
    }

    //=======================================================================
    // function : Add
    // purpose  : create a new stripe with a ChAsymSpine containing the edge <E>
    //           with the distance <Dis> and the angle Angle  on the face <F>
    //           . Add the tangential edges continuous to E in the spine
    //
    //=======================================================================

    public void Add(double Dis1, double Dis2, in TopoDS_Edge theE, in TopoDS_Face theF)
    {
        TopoDS_Edge E = TopoDS.Edge(ToWork(theE)); // the working copy's (OCCT.NET)
        TopoDS_Face F = TopoDS.Face(ToWork(theF)); // the working copy's (OCCT.NET)
        if (Contains(E) == 0 && myEFMap.Contains(E))
        {

            TopoDS_Edge E_wnt = E;
            E_wnt.Orientation(TopAbs_FORWARD);

            ChFiDS_Stripe? Stripe = new ChFiDS_Stripe();
            ref ChFiDS_Spine? Sp = ref Stripe.ChangeSpine();
            Sp = new ChFiDS_ChamfSpine(tolesp);
            ChFiDS_ChamfSpine? Spine = (Sp as ChFiDS_ChamfSpine);

            Spine.SetMode(myMode);
            double Offset = -1;
            if (myMode == ChFiDS_ConstThroatWithPenetrationChamfer)
            {
                Offset = Min(Dis1, Dis2);
            }

            Spine.SetEdges(E_wnt);
            if (PerformElement(Spine, Offset, F))
            {
                Spine.Load();
                myListStripe.Append(Stripe);

                Spine.SetDists(Dis1, Dis2);

                PerformExtremity(Spine);
            }
        }
    }

    //=======================================================================
    // function : SetDists
    // purpose  : set the distances <Dis1> and <Dis2> of the contour of
    //           index IC
    //=======================================================================

    public void SetDists(double Dis1, double Dis2, int IC, in TopoDS_Face theF)
    {
        TopoDS_Face F = TopoDS.Face(ToWork(theF)); // the working copy's (OCCT.NET)

        if (IC <= NbElements())
        {
            ChFiDS_ChamfSpine? csp = (Value(IC) as ChFiDS_ChamfSpine);

            // Search the first edge which has a common face equal to F
            TopoDS_Face F1 = new(), F2 = new(), FirstF1 = new(), FirstF2 = new();
            TopAbs_Orientation Or1 = default, Or2 = default;
            int Choix = 0, ChoixConge = 0;
            BRepAdaptor_Surface Sb1 = new(), Sb2 = new();
            int i = 1;
            bool Found = false;
            while ((i <= csp.NbEdges()) && (!Found))
            {
                SearchCommonFaces(myEFMap, csp.Edges(i), ref F1, ref F2);
                if (i == 1)
                {
                    FirstF1 = F1;
                    FirstF2 = F2;
                }
                Found = (F1.IsSame(F) || F2.IsSame(F));
                i++;
            }

            if (Found)
            {
                if (F2.IsSame(F))
                {
                    F2 = F1;
                    F1 = F;
                }
                Sb1.Initialize(F1);
                Sb2.Initialize(F2);
                Choix = ChFi3d.ConcaveSide(Sb1, Sb2, csp.Edges(i - 1), out Or1, out Or2);
                Sb1.Initialize(FirstF1);
                Sb2.Initialize(FirstF2);
                ChoixConge = ChFi3d.ConcaveSide(Sb1, Sb2, csp.Edges(1), out Or1, out Or2);
                if (ChoixConge % 2 != Choix % 2)
                {
                    csp.SetDists(Dis2, Dis1);
                }
                else
                {
                    csp.SetDists(Dis1, Dis2);
                }
            }
            else
            {
                throw new DomainErrorException("the face is not common to any of edges of the contour");
            }
        }
    }

    //=================================================================================================

    public void Dists(int IC, ref double dis1, ref double dis2)
    {
        ChFiDS_ChamfSpine? chsp = (Value(IC) as ChFiDS_ChamfSpine);
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

    public void AddDA(double Dis1, double Angle, in TopoDS_Edge theE, in TopoDS_Face theF)
    {
        TopoDS_Edge E = TopoDS.Edge(ToWork(theE)); // the working copy's (OCCT.NET)
        TopoDS_Face F = TopoDS.Face(ToWork(theF)); // the working copy's (OCCT.NET)
        if (Contains(E) == 0 && myEFMap.Contains(E))
        {

            TopoDS_Edge E_wnt = E;
            E_wnt.Orientation(TopAbs_FORWARD);

            ChFiDS_Stripe? Stripe = new ChFiDS_Stripe();
            ref ChFiDS_Spine? Sp = ref Stripe.ChangeSpine();
            Sp = new ChFiDS_ChamfSpine(tolesp);
            ChFiDS_ChamfSpine? Spine = (Sp as ChFiDS_ChamfSpine);

            Spine.SetEdges(E_wnt);
            if (PerformElement(Spine, -1, F))
            {
                Spine.Load();
                myListStripe.Append(Stripe);

                Spine.SetDistAngle(Dis1, Angle);

                PerformExtremity(Spine);
            }
        }
    }

    //=======================================================================
    // function : SetDistAngle
    // purpose  : set the distances <Dis> and <Angle> of the contour of
    //           index IC
    //=======================================================================

    public void SetDistAngle(double Dis, double Angle, int IC, in TopoDS_Face theF)
    {
        TopoDS_Face F = TopoDS.Face(ToWork(theF)); // the working copy's (OCCT.NET)

        if (IC <= NbElements())
        {
            ChFiDS_ChamfSpine? csp = (Value(IC) as ChFiDS_ChamfSpine);

            // Search the first edge which has a common face equal to F
            TopoDS_Face F1 = new(), F2 = new(), FirstF1 = new(), FirstF2 = new();
            BRepAdaptor_Surface Sb1 = new(), Sb2 = new();
            int i = 1;
            bool Found = false;

            while ((i <= csp.NbEdges()) && (!Found))
            {
                SearchCommonFaces(myEFMap, csp.Edges(i), ref F1, ref F2);
                if (i == 1)
                {
                    FirstF1 = F1;
                    FirstF2 = F2;
                }
                Found = (F1.IsSame(F) || F2.IsSame(F));
                i++;
            }

            if (Found)
            {
                if (F2.IsSame(F))
                {
                    F2 = F1;
                    F1 = F;
                }
                Sb1.Initialize(F1);
                Sb2.Initialize(F2);
                Sb1.Initialize(FirstF1);
                Sb2.Initialize(FirstF2);
                csp.SetDistAngle(Dis, Angle);
            }
            else
            {
                throw new DomainErrorException("the face is not common to any edges of the contour");
            }
        }
    }

    //=================================================================================================

    public void GetDistAngle(int IC, ref double Dis, ref double Angle)
    {
        ChFiDS_ChamfSpine? chsp = (Value(IC) as ChFiDS_ChamfSpine);

        chsp.GetDistAngle(out Dis, out Angle);
    }

    //=================================================================================================

    public void SetMode(ChFiDS_ChamfMode theMode)
    {
        myMode = theMode;
    }

    //=================================================================================================

    public ChFiDS_ChamfMethod IsChamfer(int IC)
    {
        ChFiDS_ChamfSpine? chsp = (Value(IC) as ChFiDS_ChamfSpine);

        ChFiDS_ChamfMethod ret = chsp.IsChamfer();

        return ret;
    }

    //=================================================================================================

    public ChFiDS_ChamfMode Mode()
    {
        return myMode;
    }

    //=================================================================================================

    public void ResetContour(int IC)
    {
        if (IC <= NbElements())
        {
            ChFiDS_ChamfSpine? chsp = (Value(IC) as ChFiDS_ChamfSpine);
            chsp.Reset(true);
        }
    }

    //--------------------------------AJOUT----------------------------------
    //=================================================================================================

    public void Simulate(int IC)
    {
        NCollection_List<ChFiDS_Stripe>.Iterator itel = new();
        int i = 1;
        for (itel.Initialize(myListStripe); itel.More(); itel.Next(), i++)
        {
            if (i == IC)
            {
                PerformSetOfSurf(itel.ChangeValue(), true);
                break;
            }
        }
    }

    //---------------------------AJOUT---------------------------------------
    //=================================================================================================

    public int NbSurf(int IC)
    {
        NCollection_List<ChFiDS_Stripe>.Iterator itel = new();
        int i = 1;
        for (itel.Initialize(myListStripe); itel.More(); itel.Next(), i++)
        {
            if (i == IC)
            {
                return itel.Value().SetOfSurfData().Length();
            }
        }
        return 0;
    }

    //--------------------------AJOUT---------------------------------------
    //=================================================================================================

    public NCollection_HArray1<ChFiDS_CircSection>? Sect(int IC, int IS)
    {
        NCollection_List<ChFiDS_Stripe>.Iterator itel = new();
        int i = 1;
        NCollection_HArray1<ChFiDS_CircSection>? res = null;
        for (itel.Initialize(myListStripe); itel.More(); itel.Next(), i++)
        {
            if (i == IC)
            {
                object? bid = itel.Value().SetOfSurfData().Value(IS).Simul();
                res = (bid as NCollection_HArray1<ChFiDS_CircSection>);
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

    internal override void SimulKPart(ChFiDS_SurfData? SD)
    {
        TopOpeBRepDS_DataStructure DStr = myDS.ChangeDS();
        Geom_Surface? S = DStr.Surface(SD.Surf()).Surface();
        Point2D p1f = 
            SD.InterferenceOnS1().PCurveOnSurf().Value(SD.InterferenceOnS1().FirstParameter());
        Point2D p1l = 
            SD.InterferenceOnS1().PCurveOnSurf().Value(SD.InterferenceOnS1().LastParameter());
        Point2D p2f = 
            SD.InterferenceOnS2().PCurveOnSurf().Value(SD.InterferenceOnS2().FirstParameter());
        Point2D p2l = 
            SD.InterferenceOnS2().PCurveOnSurf().Value(SD.InterferenceOnS2().LastParameter());
        GeomAdaptor_Surface AS = new(S);
        NCollection_HArray1<ChFiDS_CircSection>? sec = null;
        double u1 = 0.0, v1 = 0.0, u2 = 0.0, v2 = 0.0;
        SurfaceKind typ = AS.GetType();
        switch (typ)
        {
            case Plane: {
                v1 = p1f.Y();
                v2 = p2f.Y();
                u1 = Max(p1f.X(), p2f.X());
                u2 = Min(p1l.X(), p2l.X());
                sec = new NCollection_HArray1<ChFiDS_CircSection>(1, 2);
                Plane3D Pl = AS.Plane();
                ref ChFiDS_CircSection sec1 = ref sec.ChangeValue(1);
                ref ChFiDS_CircSection sec2 = ref sec.ChangeValue(2);
                sec1.Set(global::Kerf.Engine.Geometry.ElSLib.PlaneUIso(Pl.Position(), u1), v1, v2);
                sec2.Set(global::Kerf.Engine.Geometry.ElSLib.PlaneUIso(Pl.Position(), u2), v1, v2);
            }
            break;
            case SurfaceKind.Cone: {
                v1 = p1f.Y();
                v2 = p2f.Y();
                u1 = Max(p1f.X(), p2f.X());
                u2 = Min(p1l.X(), p2l.X());
                double ang = (u2 - u1);
                Cone Co = AS.Cone();
                double rad = Co.RefRadius(), sang = Co.SemiAngle();
                int n = (int)(36.0 * ang / M_PI + 1);
                if (n < 2)
                {
                    n = 2;
                }
                sec = new NCollection_HArray1<ChFiDS_CircSection>(1, n);
                for (int i = 1; i <= n; i++)
                {
                    ref ChFiDS_CircSection isec = ref sec.ChangeValue(i);
                    double u = u1 + (i - 1) * (u2 - u1) / (n - 1);
                    isec.Set(global::Kerf.Engine.Geometry.ElSLib.ConeUIso(Co.Position(), rad, sang, u), v1, v2);
                }
            }
            break;
            default:
                break;
        }
        SD.SetSimul(sec);
    }

    //------------------------MODIFS---------------------------------------
    //=================================================================================================

    internal override bool SimulSurf(ChFiDS_SurfData? Data, ChFiDS_ElSpine? HGuide, ChFiDS_Spine? Spine, int Choix, BRepAdaptor_Surface? S1, Adaptor3d_TopolTool? I1, BRepAdaptor_Surface? S2, Adaptor3d_TopolTool? I2, double TolGuide, ref double First, ref double Last, bool Inside, bool Appro, bool Forward, bool RecOnS1, bool RecOnS2, math_Vector Soldep, ref int intf, ref int intl)
    {
        ChFiDS_ChamfSpine? chsp = (Spine as ChFiDS_ChamfSpine);

        if (chsp is null)
        {
            throw new ConstructionErrorException("SimulSurf : this is not the spine of a chamfer");
        }

        double radius = 0.0;
        // Flexible parameters!
        double la = HGuide.LastParameter(), fi = HGuide.FirstParameter();
        double longueur = la - fi;
        double MaxStep = longueur * 0.05;
        double radiusspine = 0, locfleche, w;
        Point3D Pbid = new();
        Vector3D d1 = new(), d2 = new();
        // As ElSpine is parameterized by a curvilinear quasi-abscissa,
        // the min radius is estimated as 1/D2 max;
        // for(int i = 0; i <= 20; i++){
        int i = 0;
        for (i = 0; i <= 20; i++)
        {
            w = fi + i * MaxStep;
            HGuide.D2(w, out Pbid, out d1, out d2);
            double temp = d2.SquareMagnitude();
            if (temp > radiusspine)
            {
                radiusspine = temp;
            }
        }

        BRepBlend_Line? lin = null;
        double PFirst = First;
        if (intf != 0)
        {
            First = chsp.FirstParameter(1);
        }
        if (intl != 0)
        {
            Last = chsp.LastParameter(chsp.NbEdges());
        }

        ChFiDS_ElSpine? OffsetHGuide = null;

        if (chsp.IsChamfer() == ChFiDS_Sym)
        {
            double dis = 0.0;
            chsp.GetDist(out dis);
            radius = Max(dis, radiusspine);
            locfleche = radius * 1.0e-2; // graphic criterion

            BlendFunc_GenChamfer? pFunc = null; // std::unique_ptr in OCCT
            BlendFunc_GenChamfInv? pFInv = null; // std::unique_ptr in OCCT
            if (chsp.Mode() == ChFiDS_ClassicChamfer)
            {
                pFunc = new BlendFunc_Chamfer(S1, S2, HGuide);
                pFInv = new BlendFunc_ChamfInv(S1, S2, HGuide);
            }
            else
            {
                pFunc = new BlendFunc_ConstThroat(S1, S2, HGuide);
                pFInv = new BlendFunc_ConstThroatInv(S1, S2, HGuide);
            }
            pFunc.Set(dis, dis, Choix);
            pFInv.Set(dis, dis, Choix);

            done = SimulData(Data, HGuide, OffsetHGuide, ref lin, S1, I1, S2, I2, pFunc, pFInv, PFirst, MaxStep, locfleche,
                                              TolGuide, ref First, ref Last, Inside, Appro, Forward, Soldep, 4, RecOnS1, RecOnS2);

            if (!done)
            {
                return false;
            }
            NCollection_HArray1<ChFiDS_CircSection>? sec = null;
            Point2D pf1 = new(), pl1 = new(), pf2 = new(), pl2 = new();

            int nbp = lin.NbPoints();
            sec = new NCollection_HArray1<ChFiDS_CircSection>(1, nbp);
            for (i = 1; i <= nbp; i++)
            {
                ref ChFiDS_CircSection isec = ref sec.ChangeValue(i);
                double u1 = 0.0, v1 = 0.0, u2 = 0.0, v2 = 0.0, ww = 0.0, p1 = 0.0, p2 = 0.0;
                Line3D line = new();
                Blend_Point p = lin.Point(i);
                p.ParametersOnS1(out u1, out v1);
                p.ParametersOnS2(out u2, out v2);
                ww = p.Parameter();
                pFunc.Section(ww, u1, v1, u2, v2, out p1, out p2, ref line);
                isec.Set(line, p1, p2);
                if (i == 1)
                {
                    pf1.SetCoord(u1, v1);
                    pf2.SetCoord(u2, v2);
                }
                if (i == nbp)
                {
                    pl1.SetCoord(u1, v1);
                    pl2.SetCoord(u2, v2);
                }
            }

            Data.SetSimul(sec);
            Data.Set2dPoints(pf1, pl1, pf2, pl2);
            ChFi3d_FilCommonPoint(lin.StartPointOnFirst(), lin.TransitionOnS1(), true, ref Data.ChangeVertexFirstOnS1(),
                                                        tolapp3d);
            ChFi3d_FilCommonPoint(lin.EndPointOnFirst(), lin.TransitionOnS1(), false, ref Data.ChangeVertexLastOnS1(),
                                                        tolapp3d);
            ChFi3d_FilCommonPoint(lin.StartPointOnSecond(), lin.TransitionOnS2(), true, ref Data.ChangeVertexFirstOnS2(),
                                                        tolapp3d);
            ChFi3d_FilCommonPoint(lin.EndPointOnSecond(), lin.TransitionOnS2(), false, ref Data.ChangeVertexLastOnS2(),
                                                        tolapp3d);

            bool reverse = (!Forward || Inside);
            if (intf != 0 && reverse)
            {
                bool ok = false;
                ChFiDS_CommonPoint cp1 = Data.VertexFirstOnS1();
                if (cp1.IsOnArc())
                {
                    TopoDS_Face F1 = S1.Face();
                    TopoDS_Face bid = new();
                    intf = !SearchFace(Spine, cp1, F1, ref bid) ? 1 : 0;
                    ok = intf != 0;
                }
                ChFiDS_CommonPoint cp2 = Data.VertexFirstOnS2();
                if (cp2.IsOnArc() && !ok)
                {
                    TopoDS_Face F2 = S2.Face();
                    TopoDS_Face bid = new();
                    intf = !SearchFace(Spine, cp2, F2, ref bid) ? 1 : 0;
                }
            }
            if (intl != 0)
            {
                bool ok = false;
                ChFiDS_CommonPoint cp1 = Data.VertexLastOnS1();
                if (cp1.IsOnArc())
                {
                    TopoDS_Face F1 = S1.Face();
                    TopoDS_Face bid = new();
                    intl = !SearchFace(Spine, cp1, F1, ref bid) ? 1 : 0;
                    ok = intl != 0;
                }
                ChFiDS_CommonPoint cp2 = Data.VertexLastOnS2();
                if (cp2.IsOnArc() && !ok)
                {
                    TopoDS_Face F2 = S2.Face();
                    TopoDS_Face bid = new();
                    intl = !SearchFace(Spine, cp2, F2, ref bid) ? 1 : 0;
                }
            }
        }
        else if (chsp.IsChamfer() == ChFiDS_TwoDist)
        {
            double dis1 = 0.0, dis2 = 0.0;
            chsp.Dists(out dis1, out dis2);
            radius = Max(dis1, dis2);
            radius = Max(radius, radiusspine);
            locfleche = radius * 1.0e-2; // graphic criterion

            BlendFunc_GenChamfer? pFunc = null; // std::unique_ptr in OCCT
            BlendFunc_GenChamfInv? pFInv = null; // std::unique_ptr in OCCT
            if (chsp.Mode() == ChFiDS_ClassicChamfer)
            {
                pFunc = new BlendFunc_Chamfer(S1, S2, HGuide);
                pFInv = new BlendFunc_ChamfInv(S1, S2, HGuide);
                pFunc.Set(dis1, dis2, Choix);
                pFInv.Set(dis1, dis2, Choix);
            }
            else
            {
                NCollection_List<ChFiDS_ElSpine> ll = Spine.ChangeElSpines();
                NCollection_List<ChFiDS_ElSpine> ll_offset = Spine.ChangeOffsetElSpines();
                NCollection_List<ChFiDS_ElSpine>.Iterator ILES = new(ll), ILES_offset = new(ll_offset);
                for (; ILES.More(); ILES.Next(), ILES_offset.Next())
                {
                    ChFiDS_ElSpine? aHElSpine = ILES.Value();
                    if (aHElSpine == HGuide)
                    {
                        OffsetHGuide = ILES_offset.Value();
                    }
                }

                if (OffsetHGuide is null)
                {
                    // OCCT prints "Construction of offset guide failed!" to std::cout here
                    // exception
                }
                pFunc = new BlendFunc_ConstThroatWithPenetration(S1, S2, OffsetHGuide);
                pFInv = new BlendFunc_ConstThroatWithPenetrationInv(S1, S2, OffsetHGuide);
                double Throat = Max(dis1, dis2);
                pFunc.Set(Throat, Throat, Choix);
                pFInv.Set(Throat, Throat, Choix);
            }

            done = SimulData(Data, HGuide, OffsetHGuide, ref lin, S1, I1, S2, I2, pFunc, pFInv, PFirst, MaxStep, locfleche,
                                              TolGuide, ref First, ref Last, Inside, Appro, Forward, Soldep, 4, RecOnS1, RecOnS2);

            if (!done)
            {
                return false;
            }
            NCollection_HArray1<ChFiDS_CircSection>? sec = null;
            Point2D pf1 = new(), pl1 = new(), pf2 = new(), pl2 = new();

            int nbp = lin.NbPoints();
            sec = new NCollection_HArray1<ChFiDS_CircSection>(1, nbp);
            for (i = 1; i <= nbp; i++)
            {
                ref ChFiDS_CircSection isec = ref sec.ChangeValue(i);
                double u1 = 0.0, v1 = 0.0, u2 = 0.0, v2 = 0.0, ww = 0.0, p1 = 0.0, p2 = 0.0;
                Line3D line = new();
                Blend_Point p = lin.Point(i);
                p.ParametersOnS1(out u1, out v1);
                p.ParametersOnS2(out u2, out v2);
                ww = p.Parameter();
                pFunc.Section(ww, u1, v1, u2, v2, out p1, out p2, ref line);
                isec.Set(line, p1, p2);
                if (i == 1)
                {
                    pf1.SetCoord(u1, v1);
                    pf2.SetCoord(u2, v2);
                }
                if (i == nbp)
                {
                    pl1.SetCoord(u1, v1);
                    pl2.SetCoord(u2, v2);
                }
            }

            Data.SetSimul(sec);
            Data.Set2dPoints(pf1, pl1, pf2, pl2);
            ChFi3d_FilCommonPoint(lin.StartPointOnFirst(), lin.TransitionOnS1(), true, ref Data.ChangeVertexFirstOnS1(),
                                                        tolapp3d);
            ChFi3d_FilCommonPoint(lin.EndPointOnFirst(), lin.TransitionOnS1(), false, ref Data.ChangeVertexLastOnS1(),
                                                        tolapp3d);
            ChFi3d_FilCommonPoint(lin.StartPointOnSecond(), lin.TransitionOnS2(), true, ref Data.ChangeVertexFirstOnS2(),
                                                        tolapp3d);
            ChFi3d_FilCommonPoint(lin.EndPointOnSecond(), lin.TransitionOnS2(), false, ref Data.ChangeVertexLastOnS2(),
                                                        tolapp3d);

            bool reverse = (!Forward || Inside);
            if (intf != 0 && reverse)
            {
                bool ok = false;
                ChFiDS_CommonPoint cp1 = Data.VertexFirstOnS1();
                if (cp1.IsOnArc())
                {
                    TopoDS_Face F1 = S1.Face();
                    TopoDS_Face bid = new();
                    intf = !SearchFace(Spine, cp1, F1, ref bid) ? 1 : 0;
                    ok = intf != 0;
                }
                ChFiDS_CommonPoint cp2 = Data.VertexFirstOnS2();
                if (cp2.IsOnArc() && !ok)
                {
                    TopoDS_Face F2 = S2.Face();
                    TopoDS_Face bid = new();
                    intf = !SearchFace(Spine, cp2, F2, ref bid) ? 1 : 0;
                }
            }
            if (intl != 0)
            {
                bool ok = false;
                ChFiDS_CommonPoint cp1 = Data.VertexLastOnS1();
                if (cp1.IsOnArc())
                {
                    TopoDS_Face F1 = S1.Face();
                    TopoDS_Face bid = new();
                    intl = !SearchFace(Spine, cp1, F1, ref bid) ? 1 : 0;
                    ok = intl != 0;
                }
                ChFiDS_CommonPoint cp2 = Data.VertexLastOnS2();
                if (cp2.IsOnArc() && !ok)
                {
                    TopoDS_Face F2 = S2.Face();
                    TopoDS_Face bid = new();
                    intl = !SearchFace(Spine, cp2, F2, ref bid) ? 1 : 0;
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

            int Ch = Choix;

            BlendFunc_ChAsym Func = new(S1, S2, HGuide);
            BlendFunc_ChAsymInv FInv = new(S1, S2, HGuide);

            Func.Set(dis, angle, Ch);
            FInv.Set(dis, angle, Ch);

            done = SimulData(Data, HGuide, OffsetHGuide, ref lin, S1, I1, S2, I2, Func, FInv, PFirst, MaxStep, locfleche,
                                              TolGuide, ref First, ref Last, Inside, Appro, Forward, Soldep, 4, RecOnS1, RecOnS2);

            if (!done)
            {
                return false;
            }
            NCollection_HArray1<ChFiDS_CircSection>? sec = null;
            Point2D pf1 = new(), pl1 = new(), pf2 = new(), pl2 = new();

            int nbp = lin.NbPoints();
            sec = new NCollection_HArray1<ChFiDS_CircSection>(1, nbp);
            for (i = 1; i <= nbp; i++)
            {
                ref ChFiDS_CircSection isec = ref sec.ChangeValue(i);
                double u1 = 0.0, v1 = 0.0, u2 = 0.0, v2 = 0.0, ww = 0.0, p1 = 0.0, p2 = 0.0;
                Line3D line = new();
                Blend_Point p = lin.Point(i);
                p.ParametersOnS1(out u1, out v1);
                p.ParametersOnS2(out u2, out v2);
                ww = p.Parameter();
                Func.Section(ww, u1, v1, u2, v2, out p1, out p2, ref line);
                isec.Set(line, p1, p2);
                if (i == 1)
                {
                    pf1.SetCoord(u1, v1);
                    pf2.SetCoord(u2, v2);
                }
                if (i == nbp)
                {
                    pl1.SetCoord(u1, v1);
                    pl2.SetCoord(u2, v2);
                }
            }

            Data.SetSimul(sec);
            Data.Set2dPoints(pf1, pl1, pf2, pl2);
            ChFi3d_FilCommonPoint(lin.StartPointOnFirst(), lin.TransitionOnS1(), true, ref Data.ChangeVertexFirstOnS1(),
                                                        tolapp3d);
            ChFi3d_FilCommonPoint(lin.EndPointOnFirst(), lin.TransitionOnS1(), false, ref Data.ChangeVertexLastOnS1(),
                                                        tolapp3d);
            ChFi3d_FilCommonPoint(lin.StartPointOnSecond(), lin.TransitionOnS2(), true, ref Data.ChangeVertexFirstOnS2(),
                                                        tolapp3d);
            ChFi3d_FilCommonPoint(lin.EndPointOnSecond(), lin.TransitionOnS2(), false, ref Data.ChangeVertexLastOnS2(),
                                                        tolapp3d);

            bool reverse = (!Forward || Inside);
            if (intf != 0 && reverse)
            {
                bool ok = false;
                ChFiDS_CommonPoint cp1 = Data.VertexFirstOnS1();
                if (cp1.IsOnArc())
                {
                    TopoDS_Face F1 = S1.Face();
                    TopoDS_Face bid = new();
                    intf = !SearchFace(Spine, cp1, F1, ref bid) ? 1 : 0;
                    ok = intf != 0;
                }
                ChFiDS_CommonPoint cp2 = Data.VertexFirstOnS2();
                if (cp2.IsOnArc() && !ok)
                {
                    TopoDS_Face F2 = S2.Face();
                    TopoDS_Face bid = new();
                    intf = !SearchFace(Spine, cp2, F2, ref bid) ? 1 : 0;
                }
            }

            if (intl != 0)
            {
                bool ok = false;
                ChFiDS_CommonPoint cp1 = Data.VertexLastOnS1();
                if (cp1.IsOnArc())
                {
                    TopoDS_Face F1 = S1.Face();
                    TopoDS_Face bid = new();
                    intl = !SearchFace(Spine, cp1, F1, ref bid) ? 1 : 0;
                    ok = intl != 0;
                }
                ChFiDS_CommonPoint cp2 = Data.VertexLastOnS2();
                if (cp2.IsOnArc() && !ok)
                {
                    TopoDS_Face F2 = S2.Face();
                    TopoDS_Face bid = new();
                    intl = !SearchFace(Spine, cp2, F2, ref bid) ? 1 : 0;
                }
            }
        } // distance and angle
        return true;
    }

    internal override void SimulSurf(ChFiDS_SurfData? _unused1, ChFiDS_ElSpine? _unused2, ChFiDS_Spine? _unused3, int _unused4, BRepAdaptor_Surface? _unused5, Adaptor3d_TopolTool? _unused6, BRepAdaptor_Curve2d? _unused7, BRepAdaptor_Surface? _unused8, BRepAdaptor_Curve2d? _unused9, ref bool _unused10, BRepAdaptor_Surface? _unused11, Adaptor3d_TopolTool? _unused12, TopAbs_Orientation _unused13, double _unused14, double _unused15, ref double _unused16, ref double _unused17, bool _unused18, bool _unused19, bool _unused20, bool _unused21, bool _unused22, bool _unused23, math_Vector _unused24)
    {
        throw new FailureException("SimulSurf Not Implemented");
    }

    internal override void SimulSurf(ChFiDS_SurfData? _unused25, ChFiDS_ElSpine? _unused26, ChFiDS_Spine? _unused27, int _unused28, BRepAdaptor_Surface? _unused29, Adaptor3d_TopolTool? _unused30, TopAbs_Orientation _unused31, BRepAdaptor_Surface? _unused32, Adaptor3d_TopolTool? _unused33, BRepAdaptor_Curve2d? _unused34, BRepAdaptor_Surface? _unused35, BRepAdaptor_Curve2d? _unused36, ref bool _unused37, double _unused38, double _unused39, ref double _unused40, ref double _unused41, bool _unused42, bool _unused43, bool _unused44, bool _unused45, bool _unused46, bool _unused47, math_Vector _unused48)
    {
        throw new FailureException("SimulSurf Not Implemented");
    }

    internal override void SimulSurf(ChFiDS_SurfData? _unused49, ChFiDS_ElSpine? _unused50, ChFiDS_Spine? _unused51, int _unused52, BRepAdaptor_Surface? _unused53, Adaptor3d_TopolTool? _unused54, BRepAdaptor_Curve2d? _unused55, BRepAdaptor_Surface? _unused56, BRepAdaptor_Curve2d? _unused57, ref bool _unused58, TopAbs_Orientation _unused59, BRepAdaptor_Surface? _unused60, Adaptor3d_TopolTool? _unused61, BRepAdaptor_Curve2d? _unused62, BRepAdaptor_Surface? _unused63, BRepAdaptor_Curve2d? _unused64, ref bool _unused65, TopAbs_Orientation _unused66, double _unused67, double _unused68, ref double _unused69, ref double _unused70, bool _unused71, bool _unused72, bool _unused73, bool _unused74, bool _unused75, bool _unused76, bool _unused77, math_Vector _unused78)
    {
        throw new FailureException("SimulSurf Not Implemented");
    }

    //------------------------MODIFS---------------------------------------
    //=======================================================================
    // function : PerformFirstSection
    // purpose  : to implement the first section if there is no KPart
    //=======================================================================

    internal override bool PerformFirstSection(ChFiDS_Spine? Spine, ChFiDS_ElSpine? HGuide, int Choix, BRepAdaptor_Surface? S1, BRepAdaptor_Surface? S2, Adaptor3d_TopolTool? I1, Adaptor3d_TopolTool? I2, double Par, math_Vector SolDep, ref TopAbs_State Pos1, ref TopAbs_State Pos2)
    {
        ChFiDS_ChamfSpine? chsp = (Spine as ChFiDS_ChamfSpine);

        if (chsp is null)
        {
            throw new ConstructionErrorException("PerformSurf : this is not the spine of a chamfer");
        }

        double TolGuide = HGuide.Resolution(tolapp3d);

        if (chsp.IsChamfer() == ChFiDS_Sym)
        {
            double dis = 0.0;
            chsp.GetDist(out dis);

            BlendFunc_GenChamfer? pFunc = null; // std::unique_ptr in OCCT
            if (chsp.Mode() == ChFiDS_ClassicChamfer)
            {
                pFunc = new BlendFunc_Chamfer(S1, S2, HGuide);
            }
            else
            {
                pFunc = new BlendFunc_ConstThroat(S1, S2, HGuide);
            }
            pFunc.Set(dis, dis, Choix);
            BRepBlend_Walking TheWalk = new(S1, S2, I1, I2, HGuide);

            // calculate an approximate starting solution
            Vector3D TgF = new(), TgL = new(), tmp1 = new(), tmp2 = new(), d1gui = new();
            Point3D pt1 = new(), pt2 = new(), ptgui = new();
            Xyz temp = new();

            HGuide.D1(Par, out ptgui, out d1gui);
            //  ptgui = S1->Value(SolDep[1],SolDep[2]);

            pFunc.Set(Par);
            pFunc.Tangent(SolDep[1], SolDep[2], SolDep[3], SolDep[4], out TgF, out TgL, out tmp1, out tmp2);

            bool rev1 = false;
            bool rev2 = false;
            double sign = (TgF.Crossed(d1gui)).Dot(TgL);

            if (Choix % 2 == 1)
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
                TgF.Reverse();
            }
            if (rev2)
            {
                TgL.Reverse();
            }

            temp = (TgF.XYZ()).Multiplied(dis);
            pt1.SetXYZ((ptgui.XYZ()).Added(temp));
            temp = (TgL.XYZ()).Multiplied(dis);
            pt2.SetXYZ((ptgui.XYZ()).Added(temp));

            double tol = tolesp * 1.0e2;
            //    double u,v;
            Extrema_GenLocateExtPS proj1 = new(S1, tol, tol);
            proj1.Perform(pt1, SolDep[1], SolDep[2]);
            Extrema_GenLocateExtPS proj2 = new(S2, tol, tol);
            proj2.Perform(pt2, SolDep[3], SolDep[4]);

            if (proj1.IsDone())
            {
                (proj1.Point()).Parameter(out SolDep[1], out SolDep[2]);
            }
            if (proj2.IsDone())
            {
                (proj2.Point()).Parameter(out SolDep[3], out SolDep[4]);
            }

            return TheWalk.PerformFirstSection(pFunc, Par, SolDep, tolapp3d, TolGuide, out Pos1, out Pos2);
        }
        else if (chsp.IsChamfer() == ChFiDS_TwoDist)
        {
            double dis1 = 0.0, dis2 = 0.0;
            chsp.Dists(out dis1, out dis2);

            BlendFunc_GenChamfer? pFunc = null; // std::unique_ptr in OCCT
            if (chsp.Mode() == ChFiDS_ClassicChamfer)
            {
                pFunc = new BlendFunc_Chamfer(S1, S2, HGuide);
                pFunc.Set(dis1, dis2, Choix);
            }
            else
            {
                ChFiDS_ElSpine? OffsetHGuide = null;
                NCollection_List<ChFiDS_ElSpine> ll = Spine.ChangeElSpines();
                NCollection_List<ChFiDS_ElSpine> ll_offset = Spine.ChangeOffsetElSpines();
                NCollection_List<ChFiDS_ElSpine>.Iterator ILES = new(ll), ILES_offset = new(ll_offset);
                for (; ILES.More(); ILES.Next(), ILES_offset.Next())
                {
                    ChFiDS_ElSpine? aHElSpine = ILES.Value();
                    if (aHElSpine == HGuide)
                    {
                        OffsetHGuide = ILES_offset.Value();
                    }
                }

                if (OffsetHGuide is null)
                {
                    // OCCT prints "Construction of offset guide failed!" to std::cout here
                    // exception
                }
                pFunc = new BlendFunc_ConstThroatWithPenetration(S1, S2, OffsetHGuide);
                double Throat = Max(dis1, dis2);
                pFunc.Set(Throat, Throat, Choix); // dis2?
            }
            BRepBlend_Walking TheWalk = new(S1, S2, I1, I2, HGuide);

            // calculate an approximate starting solution
            Vector3D TgF = new(), TgL = new(), tmp1 = new(), tmp2 = new(), d1gui = new();
            Point3D pt1 = new(), pt2 = new(), ptgui = new();
            Xyz temp = new();

            HGuide.D1(Par, out ptgui, out d1gui);
            //  ptgui = S1->Value(SolDep[1],SolDep[2]);

            pFunc.Set(Par);
            pFunc.Tangent(SolDep[1], SolDep[2], SolDep[3], SolDep[4], out TgF, out TgL, out tmp1, out tmp2);

            bool rev1 = false;
            bool rev2 = false;
            double sign = (TgF.Crossed(d1gui)).Dot(TgL);

            if (Choix % 2 == 1)
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
                TgF.Reverse();
            }
            if (rev2)
            {
                TgL.Reverse();
            }

            double aDist1 = dis1, aDist2 = dis2;
            if (chsp.Mode() == ChFiDS_ConstThroatWithPenetrationChamfer)
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

            temp = (TgF.XYZ()).Multiplied(aDist1);
            pt1.SetXYZ((ptgui.XYZ()).Added(temp));
            temp = (TgL.XYZ()).Multiplied(aDist2);
            pt2.SetXYZ((ptgui.XYZ()).Added(temp));

            double tol = tolesp * 1.0e2;
            //    double u,v;

            Extrema_GenLocateExtPS proj1 = new(S1, tol, tol);
            proj1.Perform(pt1, SolDep[1], SolDep[2]);
            Extrema_GenLocateExtPS proj2 = new(S2, tol, tol);
            proj2.Perform(pt2, SolDep[3], SolDep[4]);

            if (proj1.IsDone())
            {
                (proj1.Point()).Parameter(out SolDep[1], out SolDep[2]);
            }
            if (proj2.IsDone())
            {
                (proj2.Point()).Parameter(out SolDep[3], out SolDep[4]);
            }

            return TheWalk.PerformFirstSection(pFunc, Par, SolDep, tolapp3d, TolGuide, out Pos1, out Pos2);
        }
        else
        { // distance and angle
            double dis1 = 0.0, angle = 0.0;
            chsp.GetDistAngle(out dis1, out angle);

            int Ch = Choix;

            BlendFunc_ChAsym Func = new(S1, S2, HGuide);
            Func.Set(dis1, angle, Ch);
            BRepBlend_Walking TheWalk = new(S1, S2, I1, I2, HGuide);

            // calculate an approximate starting solution
            Vector3D TgF = new(), TgL = new(), tmp1 = new(), tmp2 = new(), d1gui = new();
            Point3D pt1 = new(), pt2 = new(), ptgui = new();
            Xyz temp = new();

            HGuide.D1(Par, out ptgui, out d1gui);
            //  ptgui = S1->Value(SolDep[1],SolDep[2]);

            Func.Set(Par);
            Func.Tangent(SolDep[1], SolDep[2], SolDep[3], SolDep[4], out TgF, out TgL, out tmp1, out tmp2);

            bool rev1 = false;
            bool rev2 = false;
            double sign = (TgF.Crossed(d1gui)).Dot(TgL);

            if (Ch % 2 == 1)
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
                TgF.Reverse();
            }
            if (rev2)
            {
                TgL.Reverse();
            }

            temp = (TgF.XYZ()).Multiplied(dis1);
            pt1.SetXYZ((ptgui.XYZ()).Added(temp));

            double dis2 = 0.0, tmpcos = 0.0, tmpsin = 0.0;
            tmpcos = TgF.Dot(TgL);
            tmpsin = Math.Sqrt(1.0 - tmpcos * tmpcos);

            dis2 = dis1 / (tmpcos + tmpsin / Math.Tan(angle));

            temp = (TgL.XYZ()).Multiplied(dis2);
            pt2.SetXYZ((ptgui.XYZ()).Added(temp));

            double tol = tolesp * 1.0e2;
            //      double u,v;
            Extrema_GenLocateExtPS proj1 = new(S1, tol, tol);
            proj1.Perform(pt1, SolDep[1], SolDep[2]);
            Extrema_GenLocateExtPS proj2 = new(S2, tol, tol);
            proj2.Perform(pt2, SolDep[3], SolDep[4]);
            if (proj1.IsDone())
            {
                (proj1.Point()).Parameter(out SolDep[1], out SolDep[2]);
            }
            if (proj2.IsDone())
            {
                (proj2.Point()).Parameter(out SolDep[3], out SolDep[4]);
            }

            return TheWalk.PerformFirstSection(Func, Par, SolDep, tolapp3d, TolGuide, out Pos1, out Pos2);
        } // distance and angle
    }

    //=================================================================================================

    internal override bool PerformSurf(NCollection_Sequence<ChFiDS_SurfData> SeqData, ChFiDS_ElSpine? HGuide, ChFiDS_Spine? Spine, int Choix, BRepAdaptor_Surface? S1, Adaptor3d_TopolTool? I1, BRepAdaptor_Surface? S2, Adaptor3d_TopolTool? I2, double MaxStep, double Fleche, double TolGuide, ref double First, ref double Last, bool Inside, bool Appro, bool Forward, bool RecOnS1, bool RecOnS2, math_Vector Soldep, ref int intf, ref int intl)
    {
        ChFiDS_SurfData? Data = SeqData[1];
        ChFiDS_ChamfSpine? chsp = (Spine as ChFiDS_ChamfSpine);

        if (chsp is null)
        {
            throw new ConstructionErrorException("PerformSurf : this is not the spine of a chamfer");
        }

        bool gd1 = false, gd2 = false, gf1 = false, gf2 = false;
        BRepBlend_Line? lin = null;
        TopAbs_Orientation Or = S1.Face().Orientation();
        double PFirst = First;
        if (intf != 0)
        {
            First = chsp.FirstParameter(1);
        }
        if (intl != 0)
        {
            Last = chsp.LastParameter(chsp.NbEdges());
        }

        if (chsp.IsChamfer() == ChFiDS_Sym)
        {

            BlendFunc_GenChamfer? pFunc = null; // std::unique_ptr in OCCT
            BlendFunc_GenChamfInv? pFInv = null; // std::unique_ptr in OCCT
            if (chsp.Mode() == ChFiDS_ClassicChamfer)
            {
                pFunc = new BlendFunc_Chamfer(S1, S2, HGuide);
                pFInv = new BlendFunc_ChamfInv(S1, S2, HGuide);
            }
            else
            {
                pFunc = new BlendFunc_ConstThroat(S1, S2, HGuide);
                pFInv = new BlendFunc_ConstThroatInv(S1, S2, HGuide);
            }
            double dis = 0.0;
            chsp.GetDist(out dis);
            pFunc.Set(dis, dis, Choix);
            pFInv.Set(dis, dis, Choix);

            done = ComputeData(Data, HGuide, Spine, ref lin, S1, I1, S2, I2, pFunc, pFInv, PFirst, MaxStep, Fleche,
                                                  TolGuide, ref First, ref Last, Inside, Appro, Forward, Soldep, ref intf, ref intl, ref gd1,
                                                  ref gd2, ref gf1, ref gf2, RecOnS1, RecOnS2);
            if (!done)
            {
                return false; // ratrappage possible PMN 14/05/1998
            }
            done = CompleteData(Data, pFunc, lin, S1, S2, Or, gd1, gd2, gf1, gf2);
            if (!done)
            {
                throw new FailureException("PerformSurf : Fail of approximation!");
            }
        }
        else if (chsp.IsChamfer() == ChFiDS_TwoDist)
        {
            double d1 = 0.0, d2 = 0.0;
            chsp.Dists(out d1, out d2);

            BlendFunc_GenChamfer? pFunc = null; // std::unique_ptr in OCCT
            BlendFunc_GenChamfInv? pFInv = null; // std::unique_ptr in OCCT
            if (chsp.Mode() == ChFiDS_ClassicChamfer)
            {
                pFunc = new BlendFunc_Chamfer(S1, S2, HGuide);
                pFInv = new BlendFunc_ChamfInv(S1, S2, HGuide);
                pFunc.Set(d1, d2, Choix);
                pFInv.Set(d1, d2, Choix);
            }
            else
            {
                ChFiDS_ElSpine? OffsetHGuide = null;
                NCollection_List<ChFiDS_ElSpine> ll = Spine.ChangeElSpines();
                NCollection_List<ChFiDS_ElSpine> ll_offset = Spine.ChangeOffsetElSpines();
                NCollection_List<ChFiDS_ElSpine>.Iterator ILES = new(ll), ILES_offset = new(ll_offset);
                for (; ILES.More(); ILES.Next(), ILES_offset.Next())
                {
                    ChFiDS_ElSpine? aHElSpine = ILES.Value();
                    if (aHElSpine == HGuide)
                    {
                        OffsetHGuide = ILES_offset.Value();
                    }
                }

                if (OffsetHGuide is null)
                {
                    // OCCT prints "Construction of offset guide failed!" to std::cout here
                    // exception
                }
                pFunc = new BlendFunc_ConstThroatWithPenetration(S1, S2, OffsetHGuide);
                pFInv = new BlendFunc_ConstThroatWithPenetrationInv(S1, S2, OffsetHGuide);
                double Throat = Max(d1, d2);
                pFunc.Set(Throat, Throat, Choix);
                pFInv.Set(Throat, Throat, Choix);
            }

            done = ComputeData(Data, HGuide, Spine, ref lin, S1, I1, S2, I2, pFunc, pFInv, PFirst, MaxStep, Fleche,
                                                  TolGuide, ref First, ref Last, Inside, Appro, Forward, Soldep, ref intf, ref intl, ref gd1,
                                                  ref gd2, ref gf1, ref gf2, RecOnS1, RecOnS2);
            if (!done)
            {
                return false; // ratrappage possible PMN 14/05/1998
            }
            done = CompleteData(Data, pFunc, lin, S1, S2, Or, gd1, gd2, gf1, gf2);
            if (!done)
            {
                throw new FailureException("PerformSurf : Fail of approximation!");
            }
        }
        else
        { // distance and angle
            double d1 = 0.0, angle = 0.0;
            chsp.GetDistAngle(out d1, out angle);

            int Ch = Choix;

            BlendFunc_ChAsym Func = new(S1, S2, HGuide);
            BlendFunc_ChAsymInv FInv = new(S1, S2, HGuide);
            Func.Set(d1, angle, Ch);
            FInv.Set(d1, angle, Ch);

            done = ComputeData(Data, HGuide, Spine, ref lin, S1, I1, S2, I2, Func, FInv, PFirst, MaxStep, Fleche, TolGuide,
                                                  ref First, ref Last, Inside, Appro, Forward, Soldep, ref intf, ref intl, ref gd1, ref gd2,
                                                  ref gf1, ref gf2, RecOnS1, RecOnS2);

            if (!done)
            {
                return false; // ratrappage possible PMN 14/05/1998
            }
            done = CompleteData(Data, Func, lin, S1, S2, Or, gd1, gd2, gf1, gf2);
            if (!done)
            {
                throw new FailureException("PerformSurf : Fail of approximation!");
            }
        }

        return true;
    }

    internal override void PerformSurf(NCollection_Sequence<ChFiDS_SurfData> _unused79, ChFiDS_ElSpine? _unused80, ChFiDS_Spine? _unused81, int _unused82, BRepAdaptor_Surface? _unused83, Adaptor3d_TopolTool? _unused84, BRepAdaptor_Curve2d? _unused85, BRepAdaptor_Surface? _unused86, BRepAdaptor_Curve2d? _unused87, ref bool _unused88, BRepAdaptor_Surface? _unused89, Adaptor3d_TopolTool? _unused90, TopAbs_Orientation _unused91, double _unused92, double _unused93, double _unused94, ref double _unused95, ref double _unused96, bool _unused97, bool _unused98, bool _unused99, bool _unused100, bool _unused101, bool _unused102, math_Vector _unused103)
    {
        throw new FailureException("PerformSurf Not Implemented");
    }

    internal override void PerformSurf(NCollection_Sequence<ChFiDS_SurfData> _unused104, ChFiDS_ElSpine? _unused105, ChFiDS_Spine? _unused106, int _unused107, BRepAdaptor_Surface? _unused108, Adaptor3d_TopolTool? _unused109, TopAbs_Orientation _unused110, BRepAdaptor_Surface? _unused111, Adaptor3d_TopolTool? _unused112, BRepAdaptor_Curve2d? _unused113, BRepAdaptor_Surface? _unused114, BRepAdaptor_Curve2d? _unused115, ref bool _unused116, double _unused117, double _unused118, double _unused119, ref double _unused120, ref double _unused121, bool _unused122, bool _unused123, bool _unused124, bool _unused125, bool _unused126, bool _unused127, math_Vector _unused128)
    {
        throw new FailureException("PerformSurf Not Implemented");
    }

    internal override void PerformSurf(NCollection_Sequence<ChFiDS_SurfData> _unused129, ChFiDS_ElSpine? _unused130, ChFiDS_Spine? _unused131, int _unused132, BRepAdaptor_Surface? _unused133, Adaptor3d_TopolTool? _unused134, BRepAdaptor_Curve2d? _unused135, BRepAdaptor_Surface? _unused136, BRepAdaptor_Curve2d? _unused137, ref bool _unused138, TopAbs_Orientation _unused139, BRepAdaptor_Surface? _unused140, Adaptor3d_TopolTool? _unused141, BRepAdaptor_Curve2d? _unused142, BRepAdaptor_Surface? _unused143, BRepAdaptor_Curve2d? _unused144, ref bool _unused145, TopAbs_Orientation _unused146, double _unused147, double _unused148, double _unused149, ref double _unused150, ref double _unused151, bool _unused152, bool _unused153, bool _unused154, bool _unused155, bool _unused156, bool _unused157, bool _unused158, math_Vector _unused159)
    {
        throw new FailureException("PerformSurf Not Implemented");
    }

    //=======================================================================
    // function : ExtentOneCorner
    // purpose  : extends the spine of the stripe S on the side of the vertex V
    // PMN : 28/11/97 : Reproduces the code of fillets, and it seems to work better...
    //=======================================================================

    internal override void ExtentOneCorner(in TopoDS_Vertex V, ChFiDS_Stripe? S)
    {
        int Sens = 0;
        double Coeff = 0.5;
        ChFiDS_Spine? Spine = S.Spine();
        ChFi3d_IndexOfSurfData(V, S, out Sens);
        if (Spine.IsTangencyExtremity((Sens == 1)))
        {
            return; // No extension on queue
        }
        double dU = Spine.LastParameter(Spine.NbEdges());
        if (Sens == 1)
        {
            Spine.SetFirstParameter(-dU * Coeff);
            Spine.SetFirstTgt(0.0);
        }
        else
        {
            Spine.SetLastParameter(dU * (1.0 + Coeff));
            Spine.SetLastTgt(dU);
        }
    }

    //=======================================================================
    // function : ExtentTwoCorner
    // purpose  : extends the spines of the stripes contained in the list LS,
    //           on the side of the vertex V
    //=======================================================================

    internal override void ExtentTwoCorner(in TopoDS_Vertex V, NCollection_List<ChFiDS_Stripe> LS)
    {
        int Sens = 0;
        NCollection_List<ChFiDS_Stripe>.Iterator itel = new(LS);
        bool FF = true;
        bool[] isfirst = new bool[2];
        int[] Iedge = new int[2];
        Iedge[0] = 1;
        Iedge[1] = 1;
        ChFiDS_Stripe[] Stripe = new ChFiDS_Stripe[2];
        ChFiDS_Spine[] Spine = new ChFiDS_Spine[2];

        int i = 0;
        for (; itel.More(); itel.Next(), i++)
        {
            ChFi3d_IndexOfSurfData(V, itel.Value(), out Sens);
            if (!FF)
            {
                if (Stripe[1] == itel.Value())
                {
                    Sens = -Sens;
                }
            }

            Stripe[i] = itel.Value();
            isfirst[i] = (Sens == 1);
            Spine[i] = Stripe[i].Spine();
            if (!isfirst[i])
            {
                Iedge[i] = Spine[i].NbEdges();
            }
            FF = false;
        }

        ChFiDS_ChamfSpine[] chsp = new ChFiDS_ChamfSpine[2];
        double[] d = new double[4], dis = { 0.0, 0.0 };
        int j = 0;
        TopoDS_Face[] F = new TopoDS_Face[4];
        double tmpang = 0.0, tmd = 0.0;

        for (i = 0, j = 0; i < 2; i++, j += 2)
        {
            chsp[i] = (Spine[i] as ChFiDS_ChamfSpine);
            ConexFaces(Spine[i], Iedge[i], ref F[j], ref F[j + 1]);

            if (chsp[i].IsChamfer() == ChFiDS_Sym)
            {
                chsp[i].GetDist(out d[j]);
                d[j + 1] = d[j];
            }
            else if (chsp[i].IsChamfer() == ChFiDS_TwoDist)
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
                if (F[i].IsSame(F[j + 2]))
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

        ChFiDS_State[] State = new ChFiDS_State[2];

        for (i = 0; i < 2; i++)
        {
            if (isfirst[i])
            {
                State[i] = Spine[i].FirstStatus();
            }
            else
            {
                State[i] = Spine[i].LastStatus();
            }
        }

        if (State[0] == ChFiDS_AllSame)
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
                ExtentOneCorner(V, Stripe[j]);
            }
        }
        else if ((State[0] == ChFiDS_OnSame) && (State[1] == ChFiDS_OnSame))
        {

            ExtentSpineOnCommonFace(Spine[0], Spine[1], V, dis[0], dis[1], isfirst[0], isfirst[1]);
        }
    }

    //=================================================================================================

    internal override void ExtentThreeCorner(in TopoDS_Vertex V, NCollection_List<ChFiDS_Stripe> LS)
    {
        int Sens = 0;
        NCollection_List<ChFiDS_Stripe> check = new();
        bool[] isfirst = new bool[3];
        int[] Iedge = new int[3];
        Iedge[0] = 1;
        Iedge[1] = 1;
        Iedge[2] = 1;
        ChFiDS_Spine[] Spine = new ChFiDS_Spine[3];

        int i = 0;
        for (NCollection_List<ChFiDS_Stripe>.Iterator itel = new(LS); itel.More();
                  itel.Next(), i++)
        {
            ChFiDS_Stripe? Stripe = itel.Value();
            ChFi3d_IndexOfSurfData(V, Stripe, out Sens);
            for (NCollection_List<ChFiDS_Stripe>.Iterator ich = new(check); ich.More(); ich.Next())
            {
                if (Stripe == ich.Value())
                {
                    Sens = -Sens;
                    break;
                }
            }

            isfirst[i] = (Sens == 1);
            Spine[i] = Stripe.Spine();
            if (!isfirst[i])
            {
                Iedge[i] = Spine[i].NbEdges();
            }

            check.Append(Stripe);
        }

        double[,] d = new double[3, 2];
        double tmd, tmpangle;
        int j = 0;
        TopoDS_Face[,] F = new TopoDS_Face[3, 2];

        ChFiDS_ChamfSpine[] chsp = new ChFiDS_ChamfSpine[3];

        for (i = 0; i < 3; i++)
        {
            chsp[i] = (Spine[i] as ChFiDS_ChamfSpine);
            ConexFaces(Spine[i], Iedge[i], ref F[i, 0], ref F[i, 1]);

            if (chsp[i].IsChamfer() == ChFiDS_Sym)
            {
                chsp[i].GetDist(out d[i, 0]);
                d[i, 1] = d[i, 0];
            }
            else if (chsp[i].IsChamfer() == ChFiDS_TwoDist)
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
                    if (F[i, k].IsSame(F[j, l]))
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
            ExtentSpineOnCommonFace(Spine[i], Spine[j], V, dis[i, j], dis[j, i], isfirst[i], isfirst[j]);
        }
    }

    //=================================================================================================

    internal override void SetRegul()
    {
        NCollection_List<ChFiDS_Regul>.Iterator it = new();
        NCollection_List<TopoDS_Shape>.Iterator itc = new();
        NCollection_List<TopoDS_Shape>.Iterator its1 = new();
        NCollection_List<TopoDS_Shape>.Iterator its2 = new();
        BRepAdaptor_Surface S = new();
        BRepAdaptor_Curve2d PC = new();
        double u = 0.0, v = 0.0, t = 0.0;
        Point3D p = new();
        Vector3D n1 = new(), n2 = new(), du = new(), dv = new();
        BRep_Builder B = new();
        double Seuil = M_PI / 360.0;
        double Seuil2 = Seuil * Seuil;
        for (it.Initialize(myRegul); it.More(); it.Next())
        {
            ChFiDS_Regul reg = it.Value();
            itc.Initialize(myCoup.NewEdges(reg.Curve()));
            if (itc.More())
            {
                TopoDS_Edge E = TopoDS.Edge(itc.Value());
                if (reg.IsSurface1() && reg.IsSurface2())
                {
                    its1.Initialize(myCoup.NewFaces(reg.S1()));
                    its2.Initialize(myCoup.NewFaces(reg.S2()));
                    if (its1.More() && its2.More())
                    {
                        TopoDS_Face F1 = TopoDS.Face(its1.Value());
                        TopoDS_Face F2 = TopoDS.Face(its2.Value());
                        S.Initialize(F1, false);
                        PC.Initialize(E, F1);
                        t = 0.5 * (PC.FirstParameter() + PC.LastParameter());
                        PC.Value(t).Coord(out u, out v);
                        S.D1(u, v, out p, out du, out dv);
                        n1 = du.Crossed(dv);

                        S.Initialize(F2, false);
                        PC.Initialize(E, F2);
                        PC.Value(t).Coord(out u, out v);
                        S.D1(u, v, out p, out du, out dv);
                        n2 = du.Crossed(dv);

                        if (n1.SquareMagnitude() > 1.0e-14 && n2.SquareMagnitude() > 1.0e-14)
                        {
                            n1.Normalize();
                            n2.Normalize();
                            double sina2 = n1.Crossed(n2).SquareMagnitude();
                            if (sina2 < Seuil2)
                            {
                                Continuity cont = ChFi3d_evalconti(E, F1, F2);
                                B.Continuity(E, F1, F2, cont);
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

    private void ConexFaces(ChFiDS_Spine? Spine, int IEdge, ref TopoDS_Face F1, ref TopoDS_Face F2)
    {
        BRepAdaptor_Surface Sb1 = new(), Sb2 = new();
        TopAbs_Orientation tmp1 = default, tmp2 = default;
        int RC = 0, Choix = 0;
        TopoDS_Face f1 = new(), f2 = new(), ff1 = new(), ff2 = new();

        // calculate the reference orientation
        //  ChFi3d_Builder::StripeOrientations is private
        SearchCommonFaces(myEFMap, Spine.Edges(1), ref ff1, ref ff2);
        ff1.Orientation(TopAbs_FORWARD);
        Sb1.Initialize(ff1);
        ff2.Orientation(TopAbs_FORWARD);
        Sb2.Initialize(ff2);
        RC = ChFi3d.ConcaveSide(Sb1, Sb2, Spine.Edges(1), out tmp1, out tmp2);

        // calculate the connected faces
        SearchCommonFaces(myEFMap, Spine.Edges(IEdge), ref f1, ref f2);
        Sb1.Initialize(f1);
        Sb2.Initialize(f2);
        Choix = ChFi3d.ConcaveSide(Sb1, Sb2, Spine.Edges(IEdge), out tmp1, out tmp2);

        if (RC % 2 != Choix % 2)
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
