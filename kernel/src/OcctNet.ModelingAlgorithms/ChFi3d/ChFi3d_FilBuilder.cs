// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_FilBuilder.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_FilBuilder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Occt.BlendFunc_SectionShape;
using static Occt.ChFi3d_Builder_0;
using static Occt.ChFi3d_FilletShape;
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

/// <summary>Computes fillets of constant or evolving radius on the edges of a shape.</summary>
/// <remarks>
/// A working object of the fillet algorithms (decision 21), ported over OCCT's files: this file (the header and
/// <c>ChFi3d_FilBuilder.cxx</c>), <c>_C2</c> (two-fillet corners) and <c>_C3</c> (three-fillet corners). Ported in
/// unit 7N2 so that the builder can be compared with native OCCT end to end; the API wrappers are unit 7N3's. Edges and
/// vertices given by the caller are mapped onto the builder's working copy (<see cref="ChFi3d_Builder.ToWork"/>).
/// OCCT_DEBUG blocks (traces) are left out.
/// </remarks>
public partial class ChFi3d_FilBuilder : ChFi3d_Builder
{
    private BlendFunc_SectionShape myShape;

    /// <summary>Initializes the builder on <paramref name="S"/>.</summary>
    public ChFi3d_FilBuilder(in TopoDS_Shape S, ChFi3d_FilletShape FShape = ChFi3d_Rational, double Ta = 1.0e-2)
        : base(S, Ta)
    {
        SetFilletShape(FShape);
    }

    private static double MaxRad(ChFiDS_FilSpine? fsp, in TopoDS_Edge E)
    {
        int IE = fsp.Index(E);

        // 1: case of constant R
        if (fsp.IsConstant(IE))
        {
            return (fsp.Radius(IE));
        }
        else
        { // 2,3: case of sequence ParAndRad and(or) Laws
            return (fsp.MaxRadFromSeqAndLaws());
        }

        /*
          ChFiDS_ElSpine? HGuide = fsp.ElSpine(IE);
          double la = HGuide.LastParameter(), fi = HGuide.FirstParameter();
          double longueur = la - fi, temp, w;
        //#ifndef OCCT_DEBUG
          double radiussect = 0.0;
        //#else
        // double radiussect;
        //#endif
          Law_Composite? lc = fsp.Law(HGuide);
          for(int i = 0; i <= 5; i++){
              w = fi + i*longueur*0.2;
              temp = lc.Value(w);
              if(temp>radiussect) radiussect = temp;
          }
          return radiussect;
        */
    }

    private static void SimulParams(ChFiDS_ElSpine? HGuide, ChFiDS_FilSpine? fsp, ref double MaxStep, ref double Fleche)
    {
        double la = HGuide.LastParameter(), fi = HGuide.FirstParameter();
        double longueur = la - fi;
        MaxStep = longueur * 0.05;
        double w = 0.0;
        // gp_Pnt Pbid;
        // gp_Vec d1,d2;
        double radiussect = 0.0;
        if (fsp.IsConstant())
        {
            radiussect = fsp.Radius();
        }
        else
        {
            radiussect = 0.0;
            Law_Composite? lc = fsp.Law(HGuide);
            for (int i = 0; i <= 5; i++)
            {
                w = fi + i * longueur * 0.2;
                double temp = lc.Value(w);
                if (temp > radiussect)
                {
                    radiussect = temp;
                }
            }
        }
        Fleche = radiussect * 0.05;
    }



    // Modified:    MPS :  (10-04-97) portage WNT pour GetFilletShape



    //=================================================================================================

    //=================================================================================================

    public void SetFilletShape(ChFi3d_FilletShape FShape)
    {
        switch (FShape)
        {
            case ChFi3d_Rational:
                myShape = BlendFunc_Rational;
                break;
            case ChFi3d_QuasiAngular:
                myShape = BlendFunc_QuasiAngular;
                break;
            case ChFi3d_Polynomial:
                myShape = BlendFunc_Polynomial;
                break;
        }
    }

    //=================================================================================================

    public ChFi3d_FilletShape GetFilletShape()
    {
        ChFi3d_FilletShape filshape = ChFi3d_Rational;//  need to set default value
        switch (myShape)
        {
            case BlendFunc_Rational:
                filshape = ChFi3d_Rational;
                break;
            case BlendFunc_QuasiAngular:
                filshape = ChFi3d_QuasiAngular;
                break;
            case BlendFunc_Polynomial:
                filshape = ChFi3d_Polynomial;
                break;
            default:
                break;
        }
        return filshape;
    }

    //=================================================================================================

    public void Add(in TopoDS_Edge theE)
    {
        TopoDS_Edge E = TopoDS.Edge(ToWork(theE)); // the working copy's (OCCT.NET)
        TopoDS_Face dummy = new();

        if (Contains(E) == 0 && myEFMap.Contains(E))
        {
            ChFiDS_Stripe? Stripe = new ChFiDS_Stripe();
            ref ChFiDS_Spine? Sp = ref Stripe.ChangeSpine();
            Sp = new ChFiDS_FilSpine(tolesp);
            ChFiDS_FilSpine? Spine = (Sp as ChFiDS_FilSpine);

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

    //=================================================================================================

    public void Add(double Radius, in TopoDS_Edge E)
    {
        Add(E);
        int IC = Contains(E);
        if (IC != 0)
        {
            SetRadius(Radius, IC, E);
        }
    }

    //=================================================================================================

    public void SetRadius(Law_Function? C, int IC, int IinC)
    {
        if (IC <= NbElements())
        {
            ChFiDS_FilSpine? fsp = (Value(IC) as ChFiDS_FilSpine);
            fsp.SetRadius(C, IinC);
        }
    }

    //=================================================================================================

    public bool IsConstant(int IC)
    {
        if (IC <= NbElements())
        {
            ChFiDS_FilSpine? fsp = (Value(IC) as ChFiDS_FilSpine);
            return fsp.IsConstant();
        }
        return false;
    }

    //=================================================================================================

    public double Radius(int IC)
    {
        if (IC <= NbElements())
        {
            ChFiDS_FilSpine? fsp = (Value(IC) as ChFiDS_FilSpine);
            return fsp.Radius();
        }
        return -1.0;
    }

    //=================================================================================================

    public void ResetContour(int IC)
    {
        if (IC <= NbElements())
        {
            ChFiDS_FilSpine? fsp = (Value(IC) as ChFiDS_FilSpine);
            fsp.Reset(true);
        }
    }

    //=================================================================================================

    public void SetRadius(double Radius, int IC, in TopoDS_Edge theE)
    {
        TopoDS_Edge E = TopoDS.Edge(ToWork(theE)); // the working copy's (OCCT.NET)
        if (IC <= NbElements())
        {
            ChFiDS_FilSpine? fsp = (Value(IC) as ChFiDS_FilSpine);
            fsp.SetRadius(Radius, E);
        }
    }

    //=================================================================================================

    public void UnSet(int IC, in TopoDS_Edge theE)
    {
        TopoDS_Edge E = TopoDS.Edge(ToWork(theE)); // the working copy's (OCCT.NET)
        if (IC <= NbElements())
        {
            ChFiDS_FilSpine? fsp = (Value(IC) as ChFiDS_FilSpine);
            fsp.UnSetRadius(E);
        }
    }

    //=================================================================================================

    public void SetRadius(double Radius, int IC, in TopoDS_Vertex theV)
    {
        TopoDS_Vertex V = TopoDS.Vertex(ToWork(theV)); // the working copy's (OCCT.NET)
        if (IC <= NbElements())
        {
            ChFiDS_FilSpine? fsp = (Value(IC) as ChFiDS_FilSpine);
            fsp.SetRadius(Radius, V);
        }
    }

    //=================================================================================================

    public void UnSet(int IC, in TopoDS_Vertex theV)
    {
        TopoDS_Vertex V = TopoDS.Vertex(ToWork(theV)); // the working copy's (OCCT.NET)
        if (IC <= NbElements())
        {
            ChFiDS_FilSpine? fsp = (Value(IC) as ChFiDS_FilSpine);
            fsp.UnSetRadius(V);
        }
    }

    //=================================================================================================

    public void SetRadius(in Xy UandR, int IC, int IinC)
    {
        if (IC <= NbElements())
        {
            ChFiDS_FilSpine? fsp = (Value(IC) as ChFiDS_FilSpine);
            fsp.SetRadius(UandR, IinC);
        }
    }

    //=================================================================================================

    public bool IsConstant(int IC, in TopoDS_Edge theE)
    {
        TopoDS_Edge E = TopoDS.Edge(ToWork(theE)); // the working copy's (OCCT.NET)
        if (IC <= NbElements())
        {
            ChFiDS_FilSpine? fsp = (Value(IC) as ChFiDS_FilSpine);
            return fsp.IsConstant(fsp.Index(E));
        }
        return false;
    }

    //=================================================================================================

    public double Radius(int IC, in TopoDS_Edge theE)
    {
        TopoDS_Edge E = TopoDS.Edge(ToWork(theE)); // the working copy's (OCCT.NET)
        if (IC <= NbElements())
        {
            ChFiDS_FilSpine? fsp = (Value(IC) as ChFiDS_FilSpine);
            return fsp.Radius(E);
        }
        return -1.0;
    }

    //=================================================================================================

    public bool GetBounds(int IC, in TopoDS_Edge theE, ref double F, ref double L)
    {
        TopoDS_Edge E = TopoDS.Edge(ToWork(theE)); // the working copy's (OCCT.NET)
        if (IC <= NbElements())
        {
            ChFiDS_FilSpine? fsp = (Value(IC) as ChFiDS_FilSpine);
            ref Law_Function? loi = ref fsp.ChangeLaw(E);
            if (loi is not null)
            {
                loi.Bounds(out F, out L);
                return true;
            }
        }
        return false;
    }

    //=================================================================================================

    public Law_Function? GetLaw(int IC, in TopoDS_Edge theE)
    {
        TopoDS_Edge E = TopoDS.Edge(ToWork(theE)); // the working copy's (OCCT.NET)
        if (IC <= NbElements())
        {
            ChFiDS_FilSpine? fsp = (Value(IC) as ChFiDS_FilSpine);
            return fsp.ChangeLaw(E);
        }
        return null;
    }

    //=================================================================================================

    public void SetLaw(int IC, in TopoDS_Edge theE, Law_Function? L)
    {
        TopoDS_Edge E = TopoDS.Edge(ToWork(theE)); // the working copy's (OCCT.NET)
        // Check if it is necessary to check borders!
        if (IC <= NbElements())
        {
            ChFiDS_FilSpine? fsp = (Value(IC) as ChFiDS_FilSpine);
            fsp.ChangeLaw(E) = L;
        }
    }

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

    //=================================================================================================

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
            case SurfaceKind.Cylinder: {
                u1 = p1f.X();
                u2 = p2f.X();
                v1 = Max(p1f.Y(), p2f.Y());
                v2 = Min(p1l.Y(), p2l.Y());
                sec = new NCollection_HArray1<ChFiDS_CircSection>(1, 2);
                Cylinder Cy = AS.Cylinder();
                ref ChFiDS_CircSection sec1 = ref sec.ChangeValue(1);
                ref ChFiDS_CircSection sec2 = ref sec.ChangeValue(2);
                sec1.Set(global::Kerf.Engine.Geometry.ElSLib.CylinderVIso(Cy.Position(), Cy.Radius(), v1), u1, u2);
                sec2.Set(global::Kerf.Engine.Geometry.ElSLib.CylinderVIso(Cy.Position(), Cy.Radius(), v2), u1, u2);
            }
            break;
            case SurfaceKind.Torus: {
                v1 = p1f.Y();
                v2 = p2f.Y();
                u1 = Max(p1f.X(), p2f.X());
                u2 = Min(p1l.X(), p2l.X());
                double ang = (u2 - u1);
                Torus To = AS.Torus();
                double majr = To.MajorRadius(), minr = To.MinorRadius();
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
                    isec.Set(global::Kerf.Engine.Geometry.ElSLib.TorusUIso(To.Position(), majr, minr, u), v1, v2);
                }
            }
            break;
            case SurfaceKind.Sphere: {
                v1 = p1f.Y();
                v2 = p2f.Y();
                u1 = Max(p1f.X(), p2f.X());
                u2 = Min(p1l.X(), p2l.X());
                double ang = (u2 - u1);
                Sphere Sp = AS.Sphere();
                double rad = Sp.Radius();
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
                    isec.Set(global::Kerf.Engine.Geometry.ElSLib.SphereUIso(Sp.Position(), rad, u), v1, v2);
                }
            }
            break;
            default:
                break;
        }
        SD.SetSimul(sec);
    }

    //=================================================================================================

    internal override bool SimulSurf(ChFiDS_SurfData? Data, ChFiDS_ElSpine? HGuide, ChFiDS_Spine? Spine, int Choix, BRepAdaptor_Surface? S1, Adaptor3d_TopolTool? I1, BRepAdaptor_Surface? S2, Adaptor3d_TopolTool? I2, double TolGuide, ref double First, ref double Last, bool Inside, bool Appro, bool Forward, bool RecOnS1, bool RecOnS2, math_Vector Soldep, ref int intf, ref int intl)
    {
        ChFiDS_FilSpine? fsp = (Spine as ChFiDS_FilSpine);
        if (fsp is null)
        {
            throw new ConstructionErrorException("SimulSurf : this is not the spine of the fillet");
        }
        BRepBlend_Line? lin = null;
        // Flexible parameters!!!
        double locfleche = 0.0, MaxStep = 0.0;
        SimulParams(HGuide, fsp, ref MaxStep, ref locfleche);
        NCollection_HArray1<ChFiDS_CircSection>? sec = null;
        Point2D pf1 = new(), pl1 = new(), pf2 = new(), pl2 = new();

        ChFiDS_ElSpine? EmptyHGuide = null;

        double PFirst = First;
        if (intf != 0)
        {
            First = fsp.FirstParameter(1);
        }
        if (intl != 0)
        {
            Last = fsp.LastParameter(fsp.NbEdges());
        }
        if (fsp.IsConstant())
        {
            BlendFunc_ConstRad Func = new(S1, S2, HGuide);
            BlendFunc_ConstRadInv FInv = new(S1, S2, HGuide);
            Func.Set(fsp.Radius(), Choix);
            FInv.Set(fsp.Radius(), Choix);
            Func.Set(myShape);
            done = SimulData(Data, HGuide, EmptyHGuide, ref lin, S1, I1, S2, I2, Func, FInv, PFirst, MaxStep, locfleche,
                                              TolGuide, ref First, ref Last, Inside, Appro, Forward, Soldep, 4, RecOnS1, RecOnS2);
            if (!done)
            {
                return false;
            }
            int nbp = lin.NbPoints();
            sec = new NCollection_HArray1<ChFiDS_CircSection>(1, nbp);
            for (int i = 1; i <= nbp; i++)
            {
                ref ChFiDS_CircSection isec = ref sec.ChangeValue(i);
                double u1 = 0.0, v1 = 0.0, u2 = 0.0, v2 = 0.0, w = 0.0, p1 = 0.0, p2 = 0.0;
                Circle3D ci = new();
                Blend_Point p = lin.Point(i);
                p.ParametersOnS1(out u1, out v1);
                p.ParametersOnS2(out u2, out v2);
                w = p.Parameter();
                Func.Section(w, u1, v1, u2, v2, out p1, out p2, ref ci);
                isec.Set(ci, p1, p2);
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
        }
        else
        {
            BlendFunc_EvolRad Func = new(S1, S2, HGuide, fsp.Law(HGuide));
            BlendFunc_EvolRadInv FInv = new(S1, S2, HGuide, fsp.Law(HGuide));
            Func.Set(Choix);
            FInv.Set(Choix);
            Func.Set(myShape);
            done = SimulData(Data, HGuide, EmptyHGuide, ref lin, S1, I1, S2, I2, Func, FInv, PFirst, MaxStep, locfleche,
                                              TolGuide, ref First, ref Last, Inside, Appro, Forward, Soldep, 4, RecOnS1, RecOnS2);
            if (!done)
            {
                return false;
            }
            int nbp = lin.NbPoints();
            sec = new NCollection_HArray1<ChFiDS_CircSection>(1, nbp);
            for (int i = 1; i <= nbp; i++)
            {
                ref ChFiDS_CircSection isec = ref sec.ChangeValue(i);
                double u1 = 0.0, v1 = 0.0, u2 = 0.0, v2 = 0.0, w = 0.0, p1 = 0.0, p2 = 0.0;
                Circle3D ci = new();
                Blend_Point p = lin.Point(i);
                p.ParametersOnS1(out u1, out v1);
                p.ParametersOnS2(out u2, out v2);
                w = p.Parameter();
                Func.Section(w, u1, v1, u2, v2, out p1, out p2, ref ci);
                isec.Set(ci, p1, p2);
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
        return true;
    }

    //=================================================================================================

    internal override void SimulSurf(ChFiDS_SurfData? Data, ChFiDS_ElSpine? HGuide, ChFiDS_Spine? Spine, int Choix, BRepAdaptor_Surface? HS1, Adaptor3d_TopolTool? I1, BRepAdaptor_Curve2d? PC1, BRepAdaptor_Surface? HSref1, BRepAdaptor_Curve2d? PCref1, ref bool Decroch1, BRepAdaptor_Surface? HS2, Adaptor3d_TopolTool? I2, TopAbs_Orientation Or2, double _unused1, double TolGuide, ref double First, ref double Last, bool Inside, bool Appro, bool Forward, bool RecP, bool RecS, bool RecRst, math_Vector Soldep)
    {
        ChFiDS_FilSpine? fsp = (Spine as ChFiDS_FilSpine);
        if (fsp is null)
        {
            throw new ConstructionErrorException("PerformSurf : this is not the spine of the fillet");
        }
        BRepBlend_Line? lin = null;

        // Flexible parameters!
        double locfleche = 0.0, MaxStep = 0.0;
        SimulParams(HGuide, fsp, ref MaxStep, ref locfleche);
        NCollection_HArray1<ChFiDS_CircSection>? sec = null;
        Point2D pf = new(), pl = new(), ppcf = new(), ppcl = new();

        double PFirst = First;
        if (fsp.IsConstant())
        {
            BRepBlend_SurfRstConstRad func = new(HS2, HS1, PC1, HGuide);
            func.Set(HSref1, PCref1);
            Adaptor3d_CurveOnSurface? HC = new Adaptor3d_CurveOnSurface();
            HC.Load(PC1, HS1);
            BRepBlend_SurfCurvConstRadInv finvc = new(HS2, HC, HGuide);
            BRepBlend_SurfPointConstRadInv finvp = new(HS2, HGuide);
            BlendFunc_ConstRadInv finv = new(HS2, HSref1, HGuide);
            finv.Set(false, PCref1);

            double rad = fsp.Radius();
            int petitchoix = 1;
            if (Or2 == TopAbs_REVERSED)
            {
                petitchoix = 3;
            }
            if (Choix % 2 == 0)
            {
                petitchoix++;
            }
            finv.Set(rad, Choix);
            finvc.Set(rad, petitchoix);
            finvp.Set(rad, petitchoix);
            func.Set(rad, petitchoix);
            func.Set(myShape);

            done = SimulData(Data, HGuide, ref lin, HS2, I2, HS1, PC1, I1, ref Decroch1, func, finv, finvp, finvc, PFirst,
                                              MaxStep, locfleche, TolGuide, ref First, ref Last, Soldep, 4, Inside, Appro, Forward, RecP, RecS,
                                              RecRst);
            if (!done)
            {
                throw new FailureException("SimulSurf : Failed process!");
            }
            int nbp = lin.NbPoints();
            sec = new NCollection_HArray1<ChFiDS_CircSection>(1, nbp);
            for (int i = 1; i <= nbp; i++)
            {
                ref ChFiDS_CircSection isec = ref sec.ChangeValue(i);
                double u = 0.0, v = 0.0, w = 0.0, param = 0.0, p1 = 0.0, p2 = 0.0;
                Circle3D ci = new();
                Blend_Point p = lin.Point(i);
                p.ParametersOnS(out u, out v);
                w = p.ParameterOnC();
                param = p.Parameter();
                func.Section(param, u, v, w, out p1, out p2, ref ci);
                isec.Set(ci, p1, p2);
                if (i == 1)
                {
                    pf.SetCoord(u, v);
                    p.ParametersOnS2(out u, out v);
                    ppcf.SetCoord(u, v);
                }
                if (i == nbp)
                {
                    pl.SetCoord(u, v);
                    p.ParametersOnS2(out u, out v);
                    ppcl.SetCoord(u, v);
                }
            }
        }
        else
        {
            BRepBlend_SurfRstEvolRad func = new(HS2, HS1, PC1, HGuide, fsp.Law(HGuide));
            Adaptor3d_CurveOnSurface? HC = new Adaptor3d_CurveOnSurface();
            HC.Load(PC1, HS1);
            BRepBlend_SurfCurvEvolRadInv finvc = new(HS2, HC, HGuide, fsp.Law(HGuide));
            BRepBlend_SurfPointEvolRadInv finvp = new(HS2, HGuide, fsp.Law(HGuide));
            BlendFunc_EvolRadInv finv = new(HS2, HSref1, HGuide, fsp.Law(HGuide));
            finv.Set(false, PCref1);
            int petitchoix = 1;
            if (Or2 == TopAbs_REVERSED)
            {
                petitchoix = 3;
            }
            if (Choix % 2 == 0)
            {
                petitchoix++;
            }
            finv.Set(Choix);
            finvc.Set(petitchoix);
            finvp.Set(petitchoix);
            func.Set(petitchoix);
            func.Set(myShape);
            done = SimulData(Data, HGuide, ref lin, HS2, I2, HS1, PC1, I1, ref Decroch1, func, finv, finvp, finvc, PFirst,
                                              MaxStep, locfleche, TolGuide, ref First, ref Last, Soldep, 4, Inside, Appro, Forward, RecP, RecS,
                                              RecRst);
            if (!done)
            {
                throw new FailureException("SimulSurf : Fail !");
            }
            int nbp = lin.NbPoints();
            sec = new NCollection_HArray1<ChFiDS_CircSection>(1, nbp);
            for (int i = 1; i <= nbp; i++)
            {
                ref ChFiDS_CircSection isec = ref sec.ChangeValue(i);
                double u = 0.0, v = 0.0, w = 0.0, param = 0.0, p1 = 0.0, p2 = 0.0;
                Circle3D ci = new();
                Blend_Point p = lin.Point(i);
                p.ParametersOnS(out u, out v);
                w = p.ParameterOnC();
                param = p.Parameter();
                func.Section(param, u, v, w, out p1, out p2, ref ci);
                isec.Set(ci, p1, p2);
                if (i == 1)
                {
                    pf.SetCoord(u, v);
                    p.ParametersOnS2(out u, out v);
                    ppcf.SetCoord(u, v);
                }
                if (i == nbp)
                {
                    pl.SetCoord(u, v);
                    p.ParametersOnS2(out u, out v);
                    ppcl.SetCoord(u, v);
                }
            }
        }
        Data.SetSimul(sec);
        //  gp_Pnt2d pbid;
        Data.Set2dPoints(ppcf, ppcl, pf, pl);
        ChFi3d_FilCommonPoint(lin.StartPointOnFirst(), lin.TransitionOnS1(), true, ref Data.ChangeVertexFirstOnS2(),
                                                    tolapp3d);
        ChFi3d_FilCommonPoint(lin.EndPointOnFirst(), lin.TransitionOnS1(), false, ref Data.ChangeVertexLastOnS2(),
                                                    tolapp3d);
        ChFi3d_FilCommonPoint(lin.StartPointOnSecond(), lin.TransitionOnS2(), true, ref Data.ChangeVertexFirstOnS1(),
                                                    tolapp3d);
        ChFi3d_FilCommonPoint(lin.EndPointOnSecond(), lin.TransitionOnS2(), false, ref Data.ChangeVertexLastOnS1(),
                                                    tolapp3d);
    }

    //=================================================================================================

    internal override void SimulSurf(ChFiDS_SurfData? Data, ChFiDS_ElSpine? HGuide, ChFiDS_Spine? Spine, int Choix, BRepAdaptor_Surface? HS1, Adaptor3d_TopolTool? I1, TopAbs_Orientation Or1, BRepAdaptor_Surface? HS2, Adaptor3d_TopolTool? I2, BRepAdaptor_Curve2d? PC2, BRepAdaptor_Surface? HSref2, BRepAdaptor_Curve2d? PCref2, ref bool Decroch2, double _unused2, double TolGuide, ref double First, ref double Last, bool Inside, bool Appro, bool Forward, bool RecP, bool RecS, bool RecRst, math_Vector Soldep)
    {
        ChFiDS_FilSpine? fsp = (Spine as ChFiDS_FilSpine);
        if (fsp is null)
        {
            throw new ConstructionErrorException("PerformSurf : it is not the spine of a fillet");
        }
        BRepBlend_Line? lin = null;

        // Flexible parameters!
        double locfleche = 0.0, MaxStep = 0.0;
        SimulParams(HGuide, fsp, ref MaxStep, ref locfleche);
        NCollection_HArray1<ChFiDS_CircSection>? sec = null;
        Point2D pf = new(), pl = new(), ppcf = new(), ppcl = new();

        double PFirst = First;
        if (fsp.IsConstant())
        {
            BRepBlend_SurfRstConstRad func = new(HS1, HS2, PC2, HGuide);
            func.Set(HSref2, PCref2);
            Adaptor3d_CurveOnSurface? HC = new Adaptor3d_CurveOnSurface();
            HC.Load(PC2, HS2);
            BRepBlend_SurfCurvConstRadInv finvc = new(HS1, HC, HGuide);
            BRepBlend_SurfPointConstRadInv finvp = new(HS1, HGuide);
            BlendFunc_ConstRadInv finv = new(HS1, HSref2, HGuide);
            finv.Set(false, PCref2);

            double rad = fsp.Radius();
            int petitchoix = 1;
            if (Or1 == TopAbs_REVERSED)
            {
                petitchoix = 3;
            }
            if (Choix % 2 == 0)
            {
                petitchoix++;
            }
            finv.Set(rad, Choix);
            finvc.Set(rad, petitchoix);
            finvp.Set(rad, petitchoix);
            func.Set(rad, petitchoix);
            func.Set(myShape);

            done = SimulData(Data, HGuide, ref lin, HS1, I1, HS2, PC2, I2, ref Decroch2, func, finv, finvp, finvc, PFirst,
                                              MaxStep, locfleche, TolGuide, ref First, ref Last, Soldep, 4, Inside, Appro, Forward, RecP, RecS,
                                              RecRst);
            if (!done)
            {
                throw new FailureException("SimulSurf : Failed Processing!");
            }
            int nbp = lin.NbPoints();
            sec = new NCollection_HArray1<ChFiDS_CircSection>(1, nbp);
            for (int i = 1; i <= nbp; i++)
            {
                ref ChFiDS_CircSection isec = ref sec.ChangeValue(i);
                double u = 0.0, v = 0.0, w = 0.0, param = 0.0, p1 = 0.0, p2 = 0.0;
                Circle3D ci = new();
                Blend_Point p = lin.Point(i);
                p.ParametersOnS(out u, out v);
                w = p.ParameterOnC();
                param = p.Parameter();
                func.Section(param, u, v, w, out p1, out p2, ref ci);
                isec.Set(ci, p1, p2);
                if (i == 1)
                {
                    pf.SetCoord(u, v);
                    p.ParametersOnS2(out u, out v);
                    ppcf.SetCoord(u, v);
                }
                if (i == nbp)
                {
                    pl.SetCoord(u, v);
                    p.ParametersOnS2(out u, out v);
                    ppcl.SetCoord(u, v);
                }
            }
        }
        else
        {
            BRepBlend_SurfRstEvolRad func = new(HS1, HS2, PC2, HGuide, fsp.Law(HGuide));
            Adaptor3d_CurveOnSurface? HC = new Adaptor3d_CurveOnSurface();
            HC.Load(PC2, HS2);
            BRepBlend_SurfCurvEvolRadInv finvc = new(HS1, HC, HGuide, fsp.Law(HGuide));
            BRepBlend_SurfPointEvolRadInv finvp = new(HS1, HGuide, fsp.Law(HGuide));
            BlendFunc_EvolRadInv finv = new(HS1, HSref2, HGuide, fsp.Law(HGuide));
            finv.Set(false, PCref2);
            int petitchoix = 1;
            if (Or1 == TopAbs_REVERSED)
            {
                petitchoix = 3;
            }
            if (Choix % 2 == 0)
            {
                petitchoix++;
            }
            finv.Set(Choix);
            finvc.Set(petitchoix);
            finvp.Set(petitchoix);
            func.Set(petitchoix);
            func.Set(myShape);
            done = SimulData(Data, HGuide, ref lin, HS1, I1, HS2, PC2, I2, ref Decroch2, func, finv, finvp, finvc, PFirst,
                                              MaxStep, locfleche, TolGuide, ref First, ref Last, Soldep, 4, Inside, Appro, Forward, RecP, RecS,
                                              RecRst);
            if (!done)
            {
                throw new FailureException("SimulSurf : Fail !");
            }
            int nbp = lin.NbPoints();
            sec = new NCollection_HArray1<ChFiDS_CircSection>(1, nbp);
            for (int i = 1; i <= nbp; i++)
            {
                ref ChFiDS_CircSection isec = ref sec.ChangeValue(i);
                double u = 0.0, v = 0.0, w = 0.0, param = 0.0, p1 = 0.0, p2 = 0.0;
                Circle3D ci = new();
                Blend_Point p = lin.Point(i);
                p.ParametersOnS(out u, out v);
                w = p.ParameterOnC();
                param = p.Parameter();
                func.Section(param, u, v, w, out p1, out p2, ref ci);
                isec.Set(ci, p1, p2);
                if (i == 1)
                {
                    pf.SetCoord(u, v);
                    p.ParametersOnS2(out u, out v);
                    ppcf.SetCoord(u, v);
                }
                if (i == nbp)
                {
                    pl.SetCoord(u, v);
                    p.ParametersOnS2(out u, out v);
                    ppcl.SetCoord(u, v);
                }
            }
        }
        Data.SetSimul(sec);
        // gp_Pnt2d pbid;
        Data.Set2dPoints(pf, pl, ppcf, ppcl);
        ChFi3d_FilCommonPoint(lin.StartPointOnFirst(), lin.TransitionOnS1(), true, ref Data.ChangeVertexFirstOnS1(),
                                                    tolapp3d);
        ChFi3d_FilCommonPoint(lin.EndPointOnFirst(), lin.TransitionOnS1(), false, ref Data.ChangeVertexLastOnS1(),
                                                    tolapp3d);
        ChFi3d_FilCommonPoint(lin.StartPointOnSecond(), lin.TransitionOnS2(), true, ref Data.ChangeVertexFirstOnS2(),
                                                    tolapp3d);
        ChFi3d_FilCommonPoint(lin.EndPointOnSecond(), lin.TransitionOnS2(), false, ref Data.ChangeVertexLastOnS2(),
                                                    tolapp3d);
    }

    //=================================================================================================

    internal override void SimulSurf(ChFiDS_SurfData? Data, ChFiDS_ElSpine? HGuide, ChFiDS_Spine? Spine, int Choix, BRepAdaptor_Surface? HS1, Adaptor3d_TopolTool? I1, BRepAdaptor_Curve2d? PC1, BRepAdaptor_Surface? HSref1, BRepAdaptor_Curve2d? PCref1, ref bool Decroch1, TopAbs_Orientation Or1, BRepAdaptor_Surface? HS2, Adaptor3d_TopolTool? I2, BRepAdaptor_Curve2d? PC2, BRepAdaptor_Surface? HSref2, BRepAdaptor_Curve2d? PCref2, ref bool Decroch2, TopAbs_Orientation Or2, double _unused3, double TolGuide, ref double First, ref double Last, bool Inside, bool Appro, bool Forward, bool RecP1, bool RecRst1, bool RecP2, bool RecRst2, math_Vector Soldep)
    {
        ChFiDS_FilSpine? fsp = (Spine as ChFiDS_FilSpine);
        if (fsp is null)
        {
            throw new ConstructionErrorException("PerformSurf : it is not the spine of a fillet");
        }
        BRepBlend_Line? lin = null;

        // Flexible parameters!
        double locfleche = 0.0, MaxStep = 0.0;
        SimulParams(HGuide, fsp, ref MaxStep, ref locfleche);
        NCollection_HArray1<ChFiDS_CircSection>? sec = null;
        //  gp_Pnt2d pf,pl;

        int ch1 = 1, ch2 = 2;
        double PFirst = First;

        if (fsp.IsConstant())
        {
            BRepBlend_RstRstConstRad func = new(HS1, PC1, HS2, PC2, HGuide);
            func.Set(HSref1, PCref1, HSref2, PCref2);
            Adaptor3d_CurveOnSurface? HC1 = new Adaptor3d_CurveOnSurface();
            HC1.Load(PC1, HS1);
            Adaptor3d_CurveOnSurface? HC2 = new Adaptor3d_CurveOnSurface();
            HC2.Load(PC2, HS2);
            BRepBlend_SurfCurvConstRadInv finv1 = new(HSref1, HC2, HGuide);
            BRepBlend_CurvPointRadInv finvp1 = new(HGuide, HC2);
            BRepBlend_SurfCurvConstRadInv finv2 = new(HSref2, HC1, HGuide);
            BRepBlend_CurvPointRadInv finvp2 = new(HGuide, HC1);

            finv1.Set(PCref1);
            finv2.Set(PCref2);

            double rad = fsp.Radius();
            if (Or1 == TopAbs_REVERSED)
            {
                ch1 = 3;
            }
            if (Or2 == TopAbs_REVERSED)
            {
                ch2 = 3;
            }

            finv1.Set(rad, ch1);
            finvp1.Set(Choix);
            finv2.Set(rad, ch2);
            finvp2.Set(Choix);
            func.Set(rad, Choix);
            func.Set(myShape);

            done = SimulData(Data, HGuide, ref lin, HS1, PC1, I1, ref Decroch1, HS2, PC2, I2, ref Decroch2, func, finv1,
                                              finvp1, finv2, finvp2, PFirst, MaxStep, locfleche, TolGuide, ref First, ref Last, Soldep, 4,
                                              Inside, Appro, Forward, RecP1, RecRst1, RecP2, RecRst2);
            if (!done)
            {
                throw new FailureException("SimulSurf : Failed processing!");
            }
            int nbp = lin.NbPoints();
            sec = new NCollection_HArray1<ChFiDS_CircSection>(1, nbp);
            for (int i = 1; i <= nbp; i++)
            {
                ref ChFiDS_CircSection isec = ref sec.ChangeValue(i);
                double u = 0.0, v = 0.0, param = 0.0, p1 = 0.0, p2 = 0.0;
                Circle3D ci = new();
                Blend_Point p = lin.Point(i);
                u = p.ParameterOnC1();
                v = p.ParameterOnC2();
                param = p.Parameter();
                func.Section(param, u, v, out p1, out p2, ref ci);
                isec.Set(ci, p1, p2);
                //      if(i == 1) {pf.SetCoord(u,v);}
                //      if(i == nbp) {pl.SetCoord(u,v);}
            }
        }
        else
        {
            BRepBlend_RstRstEvolRad func = new(HS1, PC1, HS2, PC2, HGuide, fsp.Law(HGuide));
            func.Set(HSref1, PCref1, HSref2, PCref2);
            Adaptor3d_CurveOnSurface? HC1 = new Adaptor3d_CurveOnSurface();
            HC1.Load(PC1, HS1);
            Adaptor3d_CurveOnSurface? HC2 = new Adaptor3d_CurveOnSurface();
            HC2.Load(PC2, HS2);

            BRepBlend_SurfCurvEvolRadInv finv1 = new(HSref1, HC2, HGuide, fsp.Law(HGuide));
            BRepBlend_CurvPointRadInv finvp1 = new(HGuide, HC2);
            BRepBlend_SurfCurvEvolRadInv finv2 = new(HSref2, HC1, HGuide, fsp.Law(HGuide));
            BRepBlend_CurvPointRadInv finvp2 = new(HGuide, HC1);

            finv1.Set(PCref1);
            finv2.Set(PCref2);

            int ch11 = 1, ch22 = 2;

            if (Or1 == TopAbs_REVERSED)
            {
                ch11 = 3;
            }
            if (Or2 == TopAbs_REVERSED)
            {
                ch22 = 3;
            }

            finv1.Set(ch11);
            finvp1.Set(Choix);
            finv2.Set(ch22);
            finvp2.Set(Choix);
            func.Set(Choix);
            func.Set(myShape);

            done = SimulData(Data, HGuide, ref lin, HS1, PC1, I1, ref Decroch1, HS2, PC2, I2, ref Decroch2, func, finv1,
                                              finvp1, finv2, finvp2, PFirst, MaxStep, locfleche, TolGuide, ref First, ref Last, Soldep, 4,
                                              Inside, Appro, Forward, RecP1, RecRst1, RecP2, RecRst2);

            if (!done)
            {
                throw new FailureException("SimulSurf : Fail !");
            }
            int nbp = lin.NbPoints();
            sec = new NCollection_HArray1<ChFiDS_CircSection>(1, nbp);
            for (int i = 1; i <= nbp; i++)
            {
                ref ChFiDS_CircSection isec = ref sec.ChangeValue(i);
                double u = 0.0, v = 0.0, param = 0.0, p1 = 0.0, p2 = 0.0;
                Circle3D ci = new();
                Blend_Point p = lin.Point(i);
                u = p.ParameterOnC1();
                v = p.ParameterOnC2();
                param = p.Parameter();
                func.Section(param, u, v, out p1, out p2, ref ci);
                isec.Set(ci, p1, p2);
                //      if(i == 1) {pf.SetCoord(u,v);}
                //      if(i == nbp) {pl.SetCoord(u,v);}
            }
        }
        Data.SetSimul(sec);
        //  gp_Pnt2d pbid;
        //  Data->Set2dPoints(pf,pl,pbid,pbid);

        ChFi3d_FilCommonPoint(lin.StartPointOnFirst(), lin.TransitionOnS1(), true, ref Data.ChangeVertexFirstOnS1(),
                                                    tolapp3d);
        ChFi3d_FilCommonPoint(lin.EndPointOnFirst(), lin.TransitionOnS1(), false, ref Data.ChangeVertexLastOnS1(),
                                                    tolapp3d);
        ChFi3d_FilCommonPoint(lin.StartPointOnSecond(), lin.TransitionOnS2(), true, ref Data.ChangeVertexFirstOnS2(),
                                                    tolapp3d);
        ChFi3d_FilCommonPoint(lin.EndPointOnSecond(), lin.TransitionOnS2(), false, ref Data.ChangeVertexLastOnS2(),
                                                    tolapp3d);
    }

    //=================================================================================================

    internal override bool PerformFirstSection(ChFiDS_Spine? Spine, ChFiDS_ElSpine? HGuide, int Choix, BRepAdaptor_Surface? S1, BRepAdaptor_Surface? S2, Adaptor3d_TopolTool? I1, Adaptor3d_TopolTool? I2, double Par, math_Vector SolDep, ref TopAbs_State Pos1, ref TopAbs_State Pos2)
    {
        ChFiDS_FilSpine? fsp = (Spine as ChFiDS_FilSpine);
        if (fsp is null)
        {
            throw new ConstructionErrorException("PerformSurf : this is not the spine of a fillet");
        }
        double TolGuide = HGuide.Resolution(tolapp3d);
        if (fsp.IsConstant())
        {
            BlendFunc_ConstRad Func = new(S1, S2, HGuide);
            Func.Set(fsp.Radius(), Choix);
            Func.Set(myShape);
            BRepBlend_Walking TheWalk = new(S1, S2, I1, I2, HGuide);
            return TheWalk.PerformFirstSection(Func, Par, SolDep, tolapp3d, TolGuide, out Pos1, out Pos2);
        }
        else
        {
            BlendFunc_EvolRad Func = new(S1, S2, HGuide, fsp.Law(HGuide));
            Func.Set(Choix);
            Func.Set(myShape);
            BRepBlend_Walking TheWalk = new(S1, S2, I1, I2, HGuide);
            return TheWalk.PerformFirstSection(Func, Par, SolDep, tolapp3d, TolGuide, out Pos1, out Pos2);
        }
    }

    //=================================================================================================

    internal override bool PerformSurf(NCollection_Sequence<ChFiDS_SurfData> SeqData, ChFiDS_ElSpine? HGuide, ChFiDS_Spine? Spine, int Choix, BRepAdaptor_Surface? S1, Adaptor3d_TopolTool? I1, BRepAdaptor_Surface? S2, Adaptor3d_TopolTool? I2, double MaxStep, double Fleche, double TolGuide, ref double First, ref double Last, bool Inside, bool Appro, bool Forward, bool RecOnS1, bool RecOnS2, math_Vector Soldep, ref int intf, ref int intl)
    {
        ChFiDS_SurfData? Data = SeqData[1];
        ChFiDS_FilSpine? fsp = (Spine as ChFiDS_FilSpine);
        if (fsp is null)
        {
            throw new ConstructionErrorException("PerformSurf : this is not the spine of a fillet");
        }
        bool gd1 = false, gd2 = false, gf1 = false, gf2 = false, maybesingular = false;
        BRepBlend_Line? lin = null;
        TopAbs_Orientation Or = S1.Face().Orientation();
        double PFirst = First;
        if (intf != 0)
        {
            First = fsp.FirstParameter(1);
        }
        if (intl != 0)
        {
            Last = fsp.LastParameter(fsp.NbEdges());
        }
        if (fsp.IsConstant())
        {
            BlendFunc_ConstRad Func = new(S1, S2, HGuide);
            BlendFunc_ConstRadInv FInv = new(S1, S2, HGuide);
            Func.Set(fsp.Radius(), Choix);
            FInv.Set(fsp.Radius(), Choix);
            Func.Set(myShape);


            done = ComputeData(Data, HGuide, Spine, ref lin, S1, I1, S2, I2, Func, FInv, PFirst, MaxStep, Fleche, TolGuide,
                                                  ref First, ref Last, Inside, Appro, Forward, Soldep, ref intf, ref intl, ref gd1, ref gd2,
                                                  ref gf1, ref gf2, RecOnS1, RecOnS2);


            if (!done)
            {
                return false; // recovery is possible PMN 14/05/1998
            }


            done = CompleteData(Data, Func, lin, S1, S2, Or, gd1, gd2, gf1, gf2);


            if (!done)
            {
                throw new FailureException("PerformSurf : Failed approximation!");
            }
            maybesingular = (Func.GetMinimalDistance() <= 100 * tolapp3d);
        }
        else
        {
            BlendFunc_EvolRad Func = new(S1, S2, HGuide, fsp.Law(HGuide));
            BlendFunc_EvolRadInv FInv = new(S1, S2, HGuide, fsp.Law(HGuide));
            Func.Set(Choix);
            FInv.Set(Choix);
            Func.Set(myShape);


            done = ComputeData(Data, HGuide, Spine, ref lin, S1, I1, S2, I2, Func, FInv, PFirst, MaxStep, Fleche, TolGuide,
                                                  ref First, ref Last, Inside, Appro, Forward, Soldep, ref intf, ref intl, ref gd1, ref gd2,
                                                  ref gf1, ref gf2, RecOnS1, RecOnS2);

            if (!done)
            {
                return false;
            }


            done = CompleteData(Data, Func, lin, S1, S2, Or, gd1, gd2, gf1, gf2);


            if (!done)
            {
                throw new FailureException("PerformSurf : Failed approximation!");
            }
            maybesingular = (Func.GetMinimalDistance() <= 100 * tolapp3d);
        }
        if (maybesingular)
        {
            SplitSurf(SeqData, lin);
        }
        return true;
    }

    //=================================================================================================

    internal override void PerformSurf(NCollection_Sequence<ChFiDS_SurfData> SeqData, ChFiDS_ElSpine? HGuide, ChFiDS_Spine? Spine, int Choix, BRepAdaptor_Surface? HS1, Adaptor3d_TopolTool? I1, BRepAdaptor_Curve2d? PC1, BRepAdaptor_Surface? HSref1, BRepAdaptor_Curve2d? PCref1, ref bool Decroch1, BRepAdaptor_Surface? HS2, Adaptor3d_TopolTool? I2, TopAbs_Orientation Or2, double MaxStep, double Fleche, double TolGuide, ref double First, ref double Last, bool Inside, bool Appro, bool Forward, bool RecP, bool RecS, bool RecRst, math_Vector Soldep)
    {
        ChFiDS_SurfData? Data = SeqData[1];
        ChFiDS_FilSpine? fsp = (Spine as ChFiDS_FilSpine);
        if (fsp is null)
        {
            throw new ConstructionErrorException("PerformSurf : this is not the spine of a fillet");
        }
        BRepBlend_Line? lin = null;
        double PFirst = First;
        bool maybesingular = false;

        if (fsp.IsConstant())
        {
            BRepBlend_SurfRstConstRad func = new(HS2, HS1, PC1, HGuide);
            func.Set(HSref1, PCref1);
            Adaptor3d_CurveOnSurface? HC = new Adaptor3d_CurveOnSurface();
            HC.Load(PC1, HS1);
            BRepBlend_SurfCurvConstRadInv finvc = new(HS2, HC, HGuide);
            BRepBlend_SurfPointConstRadInv finvp = new(HS2, HGuide);
            BlendFunc_ConstRadInv finv = new(HS2, HSref1, HGuide);
            finv.Set(false, PCref1);

            double rad = fsp.Radius();
            int petitchoix = 1;
            if (Or2 == TopAbs_REVERSED)
            {
                petitchoix = 3;
            }
            if (Choix % 2 == 0)
            {
                petitchoix++;
            }
            finv.Set(rad, Choix);
            finvc.Set(rad, petitchoix);
            finvp.Set(rad, petitchoix);
            func.Set(rad, petitchoix);
            func.Set(myShape);

            done = ComputeData(Data, HGuide, ref lin, HS2, I2, HS1, PC1, I1, ref Decroch1, func, finv, finvp, finvc, PFirst, MaxStep,
                                                  Fleche, TolGuide, ref First, ref Last, Soldep, Inside, Appro, Forward, RecP, RecS, RecRst);
            if (!done)
            {
                Spine.SetErrorStatus(ChFiDS_WalkingFailure);
                throw new FailureException("PerformSurf : Failed processing!");
            }
            TopAbs_Orientation Or = HS2.Face().Orientation();
            done = CompleteData(Data, func, lin, HS1, HS2, Or, true);
            if (!done)
            {
                throw new FailureException("PerformSurf : Failed approximation!");
            }
            maybesingular = (func.GetMinimalDistance() <= 100 * tolapp3d);
        }
        else
        {
            BRepBlend_SurfRstEvolRad func = new(HS2, HS1, PC1, HGuide, fsp.Law(HGuide));
            func.Set(HSref1, PCref1);
            Adaptor3d_CurveOnSurface? HC = new Adaptor3d_CurveOnSurface();
            HC.Load(PC1, HS1);
            BRepBlend_SurfCurvEvolRadInv finvc = new(HS2, HC, HGuide, fsp.Law(HGuide));
            BRepBlend_SurfPointEvolRadInv finvp = new(HS2, HGuide, fsp.Law(HGuide));
            BlendFunc_EvolRadInv finv = new(HS2, HSref1, HGuide, fsp.Law(HGuide));
            finv.Set(false, PCref1);
            int petitchoix = 1;
            if (Or2 == TopAbs_REVERSED)
            {
                petitchoix = 3;
            }
            if (Choix % 2 == 0)
            {
                petitchoix++;
            }
            finv.Set(Choix);
            finvc.Set(petitchoix);
            finvp.Set(petitchoix);
            func.Set(petitchoix);
            func.Set(myShape);
            done = ComputeData(Data, HGuide, ref lin, HS2, I2, HS1, PC1, I1, ref Decroch1, func, finv, finvp, finvc, PFirst, MaxStep,
                                                  Fleche, TolGuide, ref First, ref Last, Soldep, Inside, Appro, Forward, RecP, RecS, RecRst);
            if (!done)
            {
                Spine.SetErrorStatus(ChFiDS_WalkingFailure);
                throw new FailureException("PerformSurf : Failed processing!");
            }
            TopAbs_Orientation Or = HS2.Face().Orientation();
            done = CompleteData(Data, func, lin, HS1, HS2, Or, true);
            if (!done)
            {
                throw new FailureException("PerformSurf : Failed approximation!");
            }
            maybesingular = (func.GetMinimalDistance() <= 100 * tolapp3d);
        }
        if (maybesingular)
        {
            SplitSurf(SeqData, lin);
        }
    }

    //=================================================================================================

    internal override void PerformSurf(NCollection_Sequence<ChFiDS_SurfData> SeqData, ChFiDS_ElSpine? HGuide, ChFiDS_Spine? Spine, int Choix, BRepAdaptor_Surface? HS1, Adaptor3d_TopolTool? I1, TopAbs_Orientation Or1, BRepAdaptor_Surface? HS2, Adaptor3d_TopolTool? I2, BRepAdaptor_Curve2d? PC2, BRepAdaptor_Surface? HSref2, BRepAdaptor_Curve2d? PCref2, ref bool Decroch2, double MaxStep, double Fleche, double TolGuide, ref double First, ref double Last, bool Inside, bool Appro, bool Forward, bool RecP, bool RecS, bool RecRst, math_Vector Soldep)
    {
        ChFiDS_SurfData? Data = SeqData[1];
        ChFiDS_FilSpine? fsp = (Spine as ChFiDS_FilSpine);
        if (fsp is null)
        {
            throw new ConstructionErrorException("PerformSurf : this is not the spine of a fillet");
        }
        BRepBlend_Line? lin = null;
        double PFirst = First;
        bool maybesingular = false;

        if (fsp.IsConstant())
        {
            BRepBlend_SurfRstConstRad func = new(HS1, HS2, PC2, HGuide);
            func.Set(HSref2, PCref2);
            Adaptor3d_CurveOnSurface? HC = new Adaptor3d_CurveOnSurface();
            HC.Load(PC2, HS2);
            BRepBlend_SurfCurvConstRadInv finvc = new(HS1, HC, HGuide);
            BRepBlend_SurfPointConstRadInv finvp = new(HS1, HGuide);
            BlendFunc_ConstRadInv finv = new(HS1, HSref2, HGuide);
            finv.Set(false, PCref2);

            double rad = fsp.Radius();
            int petitchoix = 1;
            if (Or1 == TopAbs_REVERSED)
            {
                petitchoix = 3;
            }
            if (Choix % 2 == 0)
            {
                petitchoix++;
            }
            finv.Set(rad, Choix);
            finvc.Set(rad, petitchoix);
            finvp.Set(rad, petitchoix);
            func.Set(rad, petitchoix);
            func.Set(myShape);

            done = ComputeData(Data, HGuide, ref lin, HS1, I1, HS2, PC2, I2, ref Decroch2, func, finv, finvp, finvc, PFirst, MaxStep,
                                                  Fleche, TolGuide, ref First, ref Last, Soldep, Inside, Appro, Forward, RecP, RecS, RecRst);
            if (!done)
            {
                Spine.SetErrorStatus(ChFiDS_WalkingFailure);
                throw new FailureException("PerformSurf : Failed processing!");
            }
            TopAbs_Orientation Or = HS1.Face().Orientation();
            done = CompleteData(Data, func, lin, HS1, HS2, Or, false);
            if (!done)
            {
                throw new FailureException("PerformSurf : Failed approximation!");
            }
            maybesingular = (func.GetMinimalDistance() <= 100 * tolapp3d);
        }
        else
        {
            BRepBlend_SurfRstEvolRad func = new(HS1, HS2, PC2, HGuide, fsp.Law(HGuide));
            func.Set(HSref2, PCref2);
            Adaptor3d_CurveOnSurface? HC = new Adaptor3d_CurveOnSurface();
            HC.Load(PC2, HS2);
            BRepBlend_SurfCurvEvolRadInv finvc = new(HS1, HC, HGuide, fsp.Law(HGuide));
            BRepBlend_SurfPointEvolRadInv finvp = new(HS1, HGuide, fsp.Law(HGuide));
            BlendFunc_EvolRadInv finv = new(HS1, HSref2, HGuide, fsp.Law(HGuide));
            finv.Set(false, PCref2);
            int petitchoix = 1;
            if (Or1 == TopAbs_REVERSED)
            {
                petitchoix = 3;
            }
            if (Choix % 2 == 0)
            {
                petitchoix++;
            }
            finv.Set(Choix);
            finvc.Set(petitchoix);
            finvp.Set(petitchoix);
            func.Set(petitchoix);
            func.Set(myShape);

            done = ComputeData(Data, HGuide, ref lin, HS1, I1, HS2, PC2, I2, ref Decroch2, func, finv, finvp, finvc, PFirst, MaxStep,
                                                  Fleche, TolGuide, ref First, ref Last, Soldep, Inside, Appro, Forward, RecP, RecS, RecRst);
            if (!done)
            {
                Spine.SetErrorStatus(ChFiDS_WalkingFailure);
                throw new FailureException("PerformSurf : Failed processing!");
            }
            TopAbs_Orientation Or = HS1.Face().Orientation();
            done = CompleteData(Data, func, lin, HS1, HS2, Or, false);
            if (!done)
            {
                throw new FailureException("PerformSurf : Failed approximation!");
            }
            maybesingular = (func.GetMinimalDistance() <= 100 * tolapp3d);
        }
        if (maybesingular)
        {
            SplitSurf(SeqData, lin);
        }
    }

    //=================================================================================================

    internal override void PerformSurf(NCollection_Sequence<ChFiDS_SurfData> SeqData, ChFiDS_ElSpine? HGuide, ChFiDS_Spine? Spine, int Choix, BRepAdaptor_Surface? HS1, Adaptor3d_TopolTool? I1, BRepAdaptor_Curve2d? PC1, BRepAdaptor_Surface? HSref1, BRepAdaptor_Curve2d? PCref1, ref bool Decroch1, TopAbs_Orientation Or1, BRepAdaptor_Surface? HS2, Adaptor3d_TopolTool? I2, BRepAdaptor_Curve2d? PC2, BRepAdaptor_Surface? HSref2, BRepAdaptor_Curve2d? PCref2, ref bool Decroch2, TopAbs_Orientation Or2, double MaxStep, double Fleche, double TolGuide, ref double First, ref double Last, bool Inside, bool Appro, bool Forward, bool RecP1, bool RecRst1, bool RecP2, bool RecRst2, math_Vector Soldep)
    {
        ChFiDS_SurfData? Data = SeqData[1];
        ChFiDS_FilSpine? fsp = (Spine as ChFiDS_FilSpine);
        if (fsp is null)
        {
            throw new ConstructionErrorException("PerformSurf : this is not the spine of a fillet");
        }
        BRepBlend_Line? lin = null;
        double PFirst = First;
        bool maybesingular = false;

        if (fsp.IsConstant())
        {
            BRepBlend_RstRstConstRad func = new(HS1, PC1, HS2, PC2, HGuide);
            func.Set(HSref1, PCref1, HSref2, PCref2);
            Adaptor3d_CurveOnSurface? HC1 = new Adaptor3d_CurveOnSurface();
            HC1.Load(PC1, HS1);
            Adaptor3d_CurveOnSurface? HC2 = new Adaptor3d_CurveOnSurface();
            HC2.Load(PC2, HS2);
            BRepBlend_SurfCurvConstRadInv finv1 = new(HSref1, HC2, HGuide);
            BRepBlend_CurvPointRadInv finvp1 = new(HGuide, HC2);
            BRepBlend_SurfCurvConstRadInv finv2 = new(HSref2, HC1, HGuide);
            BRepBlend_CurvPointRadInv finvp2 = new(HGuide, HC1);

            finv1.Set(PCref1);
            finv2.Set(PCref2);

            int ch1 = 1, ch2 = 2;
            double rad = fsp.Radius();
            if (Or1 == TopAbs_REVERSED)
            {
                ch1 = 3;
            }
            if (Or2 == TopAbs_REVERSED)
            {
                ch2 = 3;
            }

            finv1.Set(rad, ch1);
            finvp1.Set(Choix);
            finv2.Set(rad, ch2);
            finvp2.Set(Choix);
            func.Set(rad, Choix);
            func.Set(myShape);

            done = ComputeData(Data, HGuide, ref lin, HS1, PC1, I1, ref Decroch1, HS2, PC2, I2, ref Decroch2, func, finv1,
                                                  finvp1, finv2, finvp2, PFirst, MaxStep, Fleche, TolGuide, ref First, ref Last, Soldep, Inside,
                                                  Appro, Forward, RecP1, RecRst1, RecP2, RecRst2);
            if (!done)
            {
                Spine.SetErrorStatus(ChFiDS_WalkingFailure);
                throw new FailureException("PerformSurf : Failed processing!");
            }
            TopAbs_Orientation Or = HS1.Face().Orientation();
            done = CompleteData(Data, func, lin, HS1, HS2, Or);
            if (!done)
            {
                throw new FailureException("PerformSurf : Failed approximation!");
            }
            maybesingular = (func.GetMinimalDistance() <= 100 * tolapp3d);
        }
        else
        {
            BRepBlend_RstRstEvolRad func = new(HS1, PC1, HS2, PC2, HGuide, fsp.Law(HGuide));
            func.Set(HSref1, PCref1, HSref2, PCref2);
            Adaptor3d_CurveOnSurface? HC1 = new Adaptor3d_CurveOnSurface();
            HC1.Load(PC1, HS1);
            Adaptor3d_CurveOnSurface? HC2 = new Adaptor3d_CurveOnSurface();
            HC2.Load(PC2, HS2);

            BRepBlend_SurfCurvEvolRadInv finv1 = new(HSref1, HC2, HGuide, fsp.Law(HGuide));
            BRepBlend_CurvPointRadInv finvp1 = new(HGuide, HC2);
            BRepBlend_SurfCurvEvolRadInv finv2 = new(HSref2, HC1, HGuide, fsp.Law(HGuide));
            BRepBlend_CurvPointRadInv finvp2 = new(HGuide, HC1);

            finv1.Set(PCref1);
            finv2.Set(PCref2);

            int ch1 = 1, ch2 = 2;

            if (Or1 == TopAbs_REVERSED)
            {
                ch1 = 3;
            }
            if (Or2 == TopAbs_REVERSED)
            {
                ch2 = 3;
            }

            finv1.Set(ch1);
            finvp1.Set(Choix);
            finv2.Set(ch2);
            finvp2.Set(Choix);
            func.Set(Choix);
            func.Set(myShape);

            done = ComputeData(Data, HGuide, ref lin, HS1, PC1, I1, ref Decroch1, HS2, PC2, I2, ref Decroch2, func, finv1,
                                                  finvp1, finv2, finvp2, PFirst, MaxStep, Fleche, TolGuide, ref First, ref Last, Soldep, Inside,
                                                  Appro, Forward, RecP1, RecRst1, RecP2, RecRst2);

            if (!done)
            {
                Spine.SetErrorStatus(ChFiDS_WalkingFailure);
                throw new FailureException("PerformSurf : Failed processing!");
            }
            TopAbs_Orientation Or = HS1.Face().Orientation();
            done = CompleteData(Data, func, lin, HS1, HS2, Or);
            if (!done)
            {
                throw new FailureException("PerformSurf : Failed approximation!");
            }
            maybesingular = (func.GetMinimalDistance() <= 100 * tolapp3d);
        }
        if (maybesingular)
        {
            SplitSurf(SeqData, lin);
        }
    }

    //=================================================================================================

    internal void SplitSurf(NCollection_Sequence<ChFiDS_SurfData> SeqData, BRepBlend_Line? Line)
    {
        int ii = 0, Nbpnt = Line.NbPoints();
        if (Nbpnt < 3)
        {
            return;
        }
        double UFirst = 0.0, ULast = 0.0, VFirst = 0.0, VLast = 0.0;
        TopOpeBRepDS_DataStructure DStr = myDS.ChangeDS();
        int ISurf = 0;
        ChFiDS_SurfData @ref = SeqData[1]; // "ref" in OCCT
        Blend_Point P = new();

        ISurf = @ref.Surf();
        Geom_Surface? Surf = DStr.Surface(ISurf).Surface();
        Surf.Bounds(out UFirst, out ULast, out VFirst, out VLast);
        Geom_Curve? Courbe1 = Surf.UIso(UFirst);
        Geom_Curve? Courbe2 = Surf.UIso(ULast);
        ChFi3d_SearchSing Fonc = new(Courbe1, Courbe2);

        NCollection_Sequence<double> LesVi = new();
        double precedant = 0.0, suivant = 0.0, courant = 0.0;
        double a = 0.0, b = 0.0, c = 0.0;

        // (1) Finds vi so that iso v=vi is punctual
        VFirst =
            Min(@ref.InterferenceOnS1().FirstParameter(), @ref.InterferenceOnS2().FirstParameter());
        VLast =
            Max(@ref.InterferenceOnS1().LastParameter(), @ref.InterferenceOnS2().LastParameter());

        // (1.1) Finds the first point inside
        for (ii = 1; ii <= Nbpnt && Line.Point(ii).Parameter() < VFirst; ii++)
        {
        }
        if (ii == 1)
        {
            ii++;
        }
        P = Line.Point(ii);
        b = P.Parameter();
        courant = P.PointOnS1().Distance(P.PointOnS2());
        P = Line.Point(ii - 1);
        a = P.Parameter();
        precedant = P.PointOnS1().Distance(P.PointOnS2());
        ii++;

        // (1.2) Find a minimum by "points"
        for (; ii <= Nbpnt && Line.Point(ii).Parameter() <= VLast; ii++)
        {
            for (; ii <= Nbpnt && Line.Point(ii).Parameter() < VLast
                          && Line.Point(ii).Parameter() - b < global::Kerf.Engine.Foundation.Precision.PConfusion();
                      ii++)
            {
            }

            Blend_Point pnt = Line.Point(ii);
            c = pnt.Parameter();
            suivant = pnt.PointOnS1().Distance(pnt.PointOnS2());
            if ((courant < precedant) && (courant < suivant))
            {
                // (1.3) Find the exact minimum
                FunctionRoot Resol = new(Fonc, (a + c) / 2,
                                              tol2d, // Surf->VResolution(toleps),
                                              a, c, 50);
                if (Resol.IsDone)
                {
                    double Val = 0.0, racine = Resol.Root();

                    Fonc.Value(Resol.Root(), out Val);
                    if (Val < tolapp3d)
                    {
                        // the solution (avoiding the risks of confusion)
                        if (LesVi.Length() == 0)
                        {
                            if ((racine > VFirst + tol2d) && (racine < VLast - tol2d))
                            {
                                LesVi.Append(racine);
                            }
                        }
                        else
                        {
                            if ((racine > LesVi[LesVi.Length()] + tol2d) && (racine < VLast - tol2d))
                            {
                                LesVi.Append(racine);
                            }
                        }
                    }
                }
                // else: a CHFI3D_DEB trace, left out
            }
            // update if non duplication
            a = b;
            precedant = courant;
            b = c;
            courant = suivant;
        }

        // (2) Update of the sequence of SurfData
        if (LesVi.Length() > 0)
        {
            TopOpeBRepDS_DataStructure DStru = myDS.ChangeDS();
            ChFiDS_SurfData? SD = null;
            TopOpeBRepDS_Surface S = new();
            TopOpeBRepDS_Curve C1 = new(), C2 = new();
            double T = 0.0, VertexTol = 0.0;
            Point3D P3d = new(), P1 = new(), P2 = new();
            ChFiDS_CommonPoint LePoint = new();
            for (ii = 1; ii <= LesVi.Length(); ii++)
            {

                T = LesVi[ii];
                // (2.0) copy and insertion
                SD = new ChFiDS_SurfData();
                SD.Copy(@ref);
                SeqData.InsertBefore(ii, SD);
                S = DStru.Surface(@ref.Surf());
                SD.ChangeSurf(DStru.AddSurface(S));
                C1 = DStru.Curve(SD.InterferenceOnS1().LineIndex());
                SD.ChangeInterferenceOnS1().SetLineIndex(DStru.AddCurve(C1));
                C2 = DStru.Curve(SD.InterferenceOnS2().LineIndex());
                SD.ChangeInterferenceOnS2().SetLineIndex(DStru.AddCurve(C2));

                // (2.1) Modification of common Point
                SD.ChangeVertexLastOnS1().Reset();
                SD.ChangeVertexLastOnS2().Reset();
                @ref.ChangeVertexFirstOnS1().Reset();
                @ref.ChangeVertexFirstOnS2().Reset();
                Courbe1.D0(T, out P1);
                Courbe2.D0(T, out P2);
                P3d.SetXYZ((P1.XYZ() + P2.XYZ()) / 2);
                VertexTol = P1.Distance(P2);
                VertexTol += Max(C1.Tolerance(), C2.Tolerance());

                SD.ChangeVertexLastOnS1().SetPoint(P3d);
                SD.ChangeVertexLastOnS2().SetPoint(P3d);
                @ref.ChangeVertexFirstOnS1().SetPoint(P3d);
                @ref.ChangeVertexFirstOnS2().SetPoint(P3d);
                SD.ChangeVertexLastOnS1().SetTolerance(VertexTol);
                SD.ChangeVertexLastOnS2().SetTolerance(VertexTol);
                @ref.ChangeVertexFirstOnS1().SetTolerance(VertexTol);
                @ref.ChangeVertexFirstOnS2().SetTolerance(VertexTol);

                // (2.2) Modification of interferences
                SD.ChangeInterferenceOnS1().SetLastParameter(T);
                SD.ChangeInterferenceOnS2().SetLastParameter(T);
                @ref.ChangeInterferenceOnS1().SetFirstParameter(T);
                @ref.ChangeInterferenceOnS2().SetFirstParameter(T);

                // Parameters on ElSpine
                SD.LastSpineParam(T);
                @ref.FirstSpineParam(T);
            }
        }
    }

    //=================================================================================================

    internal override void ExtentOneCorner(in TopoDS_Vertex V, ChFiDS_Stripe? S)
    {
        // review by using the data at end of fillets (point, radius, normal
        // to the faces and tangents of the guideline).
        int Sens = 0;
        double Coeff = 0.5;
        ChFiDS_Spine? Spine = S.Spine();
        ChFi3d_IndexOfSurfData(V, S, out Sens);
        double dU = Spine.LastParameter(Spine.NbEdges());
        if (Spine.IsTangencyExtremity((Sens == 1)))
        {
            return; // No extension in the queue
        }

        if (Spine.Status((Sens == 1)) == ChFiDS_FreeBoundary)
        {
            Coeff *= 2; // It is necessary to go to the end and to evaluate the length
        }

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

    //=================================================================================================

    internal override void ExtentTwoCorner(in TopoDS_Vertex V, NCollection_List<ChFiDS_Stripe> LS)
    {
        // Review by using the data at end of fillets (point, radius, normal
        // to faces and tangents to the guideline.
        int Sens = 0;
        double Coeff = 0.3, Eval = 0.0, dU, rad;
        NCollection_List<ChFiDS_Stripe>.Iterator itel = new(LS);
        bool FF = true;
        ChFiDS_Stripe? Stripe = null;
        ChFiDS_Spine? Spine = null;

        // A value of symmetric extension is calculated
        for (; itel.More(); itel.Next())
        {
            Stripe = itel.Value();
            Spine = Stripe.Spine();
            dU = Spine.LastParameter(Spine.NbEdges()) * Coeff;
            ChFiDS_FilSpine? fsp = (Spine as ChFiDS_FilSpine);
            if (fsp.IsConstant())
            {
                rad = fsp.Radius();
            }
            else
            {
                TopoDS_Edge E = ChFi3d_EdgeFromV1(V, itel.Value(), out Sens);
                rad = MaxRad(fsp, E);
                /*
                IE = ChFi3d_IndexOfSurfData(V,itel.Value(),out Sens);
                rad = MaxRad(fsp, IE);
                */
            }
            rad *= 1.5;
            if (rad > dU)
            {
                dU = rad;
            }
            if (dU > Eval)
            {
                Eval = dU;
            }
        }

        // One applies
        for (itel.Initialize(LS); itel.More(); itel.Next())
        {
            ChFi3d_IndexOfSurfData(V, itel.Value(), out Sens);
            if (!FF && Stripe == itel.Value())
            {
                Sens = -Sens;
            }
            Stripe = itel.Value();
            Spine = Stripe.Spine();
            if (!Spine.IsTangencyExtremity((Sens == 1)))
            { // No extension on queue
                if (Sens == 1)
                {
                    Spine.SetFirstParameter(-Eval);
                    Spine.SetFirstTgt(0.0);
                }
                else
                {
                    dU = Spine.LastParameter(Spine.NbEdges());
                    Spine.SetLastParameter(dU + Eval);
                    Spine.SetLastTgt(dU);
                }
                FF = false;
            }
        }
    }

    //=================================================================================================

    internal override void ExtentThreeCorner(in TopoDS_Vertex V, NCollection_List<ChFiDS_Stripe> LS)
    {
        // Review by using the data at end of fillets (point, radius, normal
        // to faces and tangents to the guideline.
        int Sens = 0;
        double Coeff = 0.1;
        NCollection_List<ChFiDS_Stripe> check = new();
        //  bool FF = true;
        for (NCollection_List<ChFiDS_Stripe>.Iterator itel = new(LS); itel.More(); itel.Next())
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
            ChFiDS_Spine? Spine = Stripe.Spine();
            if (Spine.IsTangencyExtremity((Sens == 1)))
            {
                return; // No extension on queue
            }
            double dU = Spine.LastParameter(Spine.NbEdges());
            if (Sens == 1)
            {
                if (Spine.GetTypeOfConcavity() != ChFiDS_TypeOfConcavity.ChFiDS_Convex || Spine.FirstStatus() != ChFiDS_OnSame)
                {
                    Spine.SetFirstParameter(-dU * Coeff);
                    Spine.SetFirstTgt(0.0);
                }
            }
            else
            {
                if (Spine.GetTypeOfConcavity() != ChFiDS_TypeOfConcavity.ChFiDS_Convex || Spine.LastStatus() != ChFiDS_OnSame)
                {
                    Spine.SetLastParameter(dU * (1.0 + Coeff));
                    Spine.SetLastTgt(dU);
                }
            }
            check.Append(Stripe);
        }
    }

    //=================================================================================================

    internal override void SetRegul()
    {
        NCollection_List<ChFiDS_Regul>.Iterator it = new();
        NCollection_List<TopoDS_Shape>.Iterator itc = new();
        NCollection_List<TopoDS_Shape>.Iterator its1 = new();
        NCollection_List<TopoDS_Shape>.Iterator its2 = new();
        BRep_Builder B = new();
        for (it.Initialize(myRegul); it.More(); it.Next())
        {
            ChFiDS_Regul reg = it.Value();
            itc.Initialize(myCoup.NewEdges(reg.Curve()));
            if (itc.More())
            {
                TopoDS_Edge E = TopoDS.Edge(itc.Value());
                if (reg.IsSurface1())
                {
                    its1.Initialize(myCoup.NewFaces(reg.S1()));
                }
                else
                {
                    its1.Initialize(myCoup.Merged(myDS.Shape(reg.S1()), TopAbs_IN));
                }
                if (reg.IsSurface2())
                {
                    its2.Initialize(myCoup.NewFaces(reg.S2()));
                }
                else
                {
                    its2.Initialize(myCoup.Merged(myDS.Shape(reg.S2()), TopAbs_IN));
                }
                if (its1.More() && its2.More())
                {
                    TopoDS_Face F1 = TopoDS.Face(its1.Value());
                    TopoDS_Face F2 = TopoDS.Face(its2.Value());
                    Continuity cont = ChFi3d_evalconti(E, F1, F2);
                    B.Continuity(E, F1, F2, cont);
                }
            }
        }
    }
}
