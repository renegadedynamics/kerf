// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_FilBuilder.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_FilBuilder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Booleans.Legacy.Data;
using Kerf.Engine.Construction.Laws;
using Kerf.Engine.Fillets.Blend;
using Kerf.Engine.Fillets.BlendFunctions;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Fillets.BlendFunctions.SectionShape;
using static Kerf.Engine.Fillets.Builder_0;
using static Kerf.Engine.Fillets.FilletShape;
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

/// <summary>Computes fillets of constant or evolving radius on the edges of a shape.</summary>
/// <remarks>
/// A working object of the fillet algorithms (decision 21), ported over OCCT's files: this file (the header and
/// <c>ChFi3d_FilBuilder.cxx</c>), <c>_C2</c> (two-fillet corners) and <c>_C3</c> (three-fillet corners). Ported in
/// unit 7N2 so that the builder can be compared with native OCCT end to end; the API wrappers are unit 7N3's. Edges and
/// vertices given by the caller are mapped onto the builder's working copy (<see cref="FilletBuilderBase.ToWork"/>).
/// OCCT_DEBUG blocks (traces) are left out.
/// </remarks>
public partial class FilBuilder : FilletBuilderBase
{
    private SectionShape _shape;

    /// <summary>Initializes the builder on <paramref name="s"/>.</summary>
    public FilBuilder(in Shape s, FilletShape fShape = FilletShape.Rational, double ta = 1.0e-2)
        : base(s, ta)
    {
        SetFilletShape(fShape);
    }

    private static double MaxRad(FilSpine? fsp, in Edge e)
    {
        int ie = fsp.Index(e);

        // 1: case of constant R
        if (fsp.IsConstant(ie))
        {
            return (fsp.Radius(ie));
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

    private static void SimulParams(ElSpine? hGuide, FilSpine? fsp, ref double maxStep, ref double fleche)
    {
        double la = hGuide.LastParameter(), fi = hGuide.FirstParameter();
        double longueur = la - fi;
        maxStep = longueur * 0.05;
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
            Composite? lc = fsp.Law(hGuide);
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
        fleche = radiussect * 0.05;
    }



    // Modified:    MPS :  (10-04-97) portage WNT pour GetFilletShape



    //=================================================================================================

    //=================================================================================================

    public void SetFilletShape(FilletShape fShape)
    {
        switch (fShape)
        {
            case FilletShape.Rational:
                _shape = SectionShape.Rational;
                break;
            case FilletShape.QuasiAngular:
                _shape = SectionShape.QuasiAngular;
                break;
            case FilletShape.Polynomial:
                _shape = SectionShape.Polynomial;
                break;
        }
    }

    //=================================================================================================

    public FilletShape GetFilletShape()
    {
        FilletShape filshape = FilletShape.Rational;//  need to set default value
        switch (_shape)
        {
            case SectionShape.Rational:
                filshape = FilletShape.Rational;
                break;
            case SectionShape.QuasiAngular:
                filshape = FilletShape.QuasiAngular;
                break;
            case SectionShape.Polynomial:
                filshape = FilletShape.Polynomial;
                break;
            default:
                break;
        }
        return filshape;
    }

    //=================================================================================================

    public void Add(in Edge e)
    {
        Edge E = ShapeCasts.Edge(ToWork(e)); // the working copy's (OCCT.NET)
        Face dummy = new();

        if (Contains(E) == 0 && _efMap.Contains(E))
        {
            Stripe? stripe = new Stripe();
            ref Spine? sp = ref stripe.ChangeSpine();
            sp = new FilSpine(_tolesp);
            FilSpine? spine = (sp as FilSpine);

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

    //=================================================================================================

    public void Add(double radius, in Edge e)
    {
        Add(e);
        int ic = Contains(e);
        if (ic != 0)
        {
            SetRadius(radius, ic, e);
        }
    }

    //=================================================================================================

    public void SetRadius(LawsFunction? c, int ic, int iinC)
    {
        if (ic <= NbElements())
        {
            FilSpine? fsp = (Value(ic) as FilSpine);
            fsp.SetRadius(c, iinC);
        }
    }

    //=================================================================================================

    public bool IsConstant(int ic)
    {
        if (ic <= NbElements())
        {
            FilSpine? fsp = (Value(ic) as FilSpine);
            return fsp.IsConstant();
        }
        return false;
    }

    //=================================================================================================

    public double Radius(int ic)
    {
        if (ic <= NbElements())
        {
            FilSpine? fsp = (Value(ic) as FilSpine);
            return fsp.Radius();
        }
        return -1.0;
    }

    //=================================================================================================

    public void ResetContour(int ic)
    {
        if (ic <= NbElements())
        {
            FilSpine? fsp = (Value(ic) as FilSpine);
            fsp.Reset(true);
        }
    }

    //=================================================================================================

    public void SetRadius(double radius, int ic, in Edge e)
    {
        Edge E = ShapeCasts.Edge(ToWork(e)); // the working copy's (OCCT.NET)
        if (ic <= NbElements())
        {
            FilSpine? fsp = (Value(ic) as FilSpine);
            fsp.SetRadius(radius, E);
        }
    }

    //=================================================================================================

    public void UnSet(int ic, in Edge e)
    {
        Edge E = ShapeCasts.Edge(ToWork(e)); // the working copy's (OCCT.NET)
        if (ic <= NbElements())
        {
            FilSpine? fsp = (Value(ic) as FilSpine);
            fsp.UnSetRadius(E);
        }
    }

    //=================================================================================================

    public void SetRadius(double radius, int ic, in Vertex v)
    {
        Vertex V = ShapeCasts.Vertex(ToWork(v)); // the working copy's (OCCT.NET)
        if (ic <= NbElements())
        {
            FilSpine? fsp = (Value(ic) as FilSpine);
            fsp.SetRadius(radius, V);
        }
    }

    //=================================================================================================

    public void UnSet(int ic, in Vertex v)
    {
        Vertex V = ShapeCasts.Vertex(ToWork(v)); // the working copy's (OCCT.NET)
        if (ic <= NbElements())
        {
            FilSpine? fsp = (Value(ic) as FilSpine);
            fsp.UnSetRadius(V);
        }
    }

    //=================================================================================================

    public void SetRadius(in Xy uandR, int ic, int iinC)
    {
        if (ic <= NbElements())
        {
            FilSpine? fsp = (Value(ic) as FilSpine);
            fsp.SetRadius(uandR, iinC);
        }
    }

    //=================================================================================================

    public bool IsConstant(int ic, in Edge e)
    {
        Edge E = ShapeCasts.Edge(ToWork(e)); // the working copy's (OCCT.NET)
        if (ic <= NbElements())
        {
            FilSpine? fsp = (Value(ic) as FilSpine);
            return fsp.IsConstant(fsp.Index(E));
        }
        return false;
    }

    //=================================================================================================

    public double Radius(int ic, in Edge e)
    {
        Edge E = ShapeCasts.Edge(ToWork(e)); // the working copy's (OCCT.NET)
        if (ic <= NbElements())
        {
            FilSpine? fsp = (Value(ic) as FilSpine);
            return fsp.Radius(E);
        }
        return -1.0;
    }

    //=================================================================================================

    public bool GetBounds(int ic, in Edge e, ref double f, ref double l)
    {
        Edge E = ShapeCasts.Edge(ToWork(e)); // the working copy's (OCCT.NET)
        if (ic <= NbElements())
        {
            FilSpine? fsp = (Value(ic) as FilSpine);
            ref LawsFunction? loi = ref fsp.ChangeLaw(E);
            if (loi is not null)
            {
                loi.Bounds(out f, out l);
                return true;
            }
        }
        return false;
    }

    //=================================================================================================

    public LawsFunction? GetLaw(int ic, in Edge e)
    {
        Edge E = ShapeCasts.Edge(ToWork(e)); // the working copy's (OCCT.NET)
        if (ic <= NbElements())
        {
            FilSpine? fsp = (Value(ic) as FilSpine);
            return fsp.ChangeLaw(E);
        }
        return null;
    }

    //=================================================================================================

    public void SetLaw(int ic, in Edge e, LawsFunction? l)
    {
        Edge E = ShapeCasts.Edge(ToWork(e)); // the working copy's (OCCT.NET)
        // Check if it is necessary to check borders!
        if (ic <= NbElements())
        {
            FilSpine? fsp = (Value(ic) as FilSpine);
            fsp.ChangeLaw(E) = l;
        }
    }

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

    //=================================================================================================

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
            case SurfaceKind.Cylinder: {
                u1 = p1f.X();
                u2 = p2f.X();
                v1 = Max(p1f.Y(), p2f.Y());
                v2 = Min(p1l.Y(), p2l.Y());
                sec = new NCollection_HArray1<CircSection>(1, 2);
                Cylinder cy = AS.Cylinder();
                ref CircSection sec1 = ref sec.ChangeValue(1);
                ref CircSection sec2 = ref sec.ChangeValue(2);
                sec1.Set(global::Kerf.Engine.Geometry.ElSLib.CylinderVIso(cy.Position(), cy.Radius(), v1), u1, u2);
                sec2.Set(global::Kerf.Engine.Geometry.ElSLib.CylinderVIso(cy.Position(), cy.Radius(), v2), u1, u2);
            }
            break;
            case SurfaceKind.Torus: {
                v1 = p1f.Y();
                v2 = p2f.Y();
                u1 = Max(p1f.X(), p2f.X());
                u2 = Min(p1l.X(), p2l.X());
                double ang = (u2 - u1);
                Torus to = AS.Torus();
                double majr = to.MajorRadius(), minr = to.MinorRadius();
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
                    isec.Set(global::Kerf.Engine.Geometry.ElSLib.TorusUIso(to.Position(), majr, minr, u), v1, v2);
                }
            }
            break;
            case SurfaceKind.Sphere: {
                v1 = p1f.Y();
                v2 = p2f.Y();
                u1 = Max(p1f.X(), p2f.X());
                u2 = Min(p1l.X(), p2l.X());
                double ang = (u2 - u1);
                Sphere sp = AS.Sphere();
                double rad = sp.Radius();
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
                    isec.Set(global::Kerf.Engine.Geometry.ElSLib.SphereUIso(sp.Position(), rad, u), v1, v2);
                }
            }
            break;
            default:
                break;
        }
        sd.SetSimul(sec);
    }

    //=================================================================================================

    internal override bool SimulSurf(SurfData? data, ElSpine? hGuide, Spine? spine, int choix, FaceSurfaceAdaptor? s1, TopolTool? i1, FaceSurfaceAdaptor? s2, TopolTool? i2, double tolGuide, ref double first, ref double last, bool inside, bool appro, bool forward, bool recOnS1, bool recOnS2, math_Vector soldep, ref int intf, ref int intl)
    {
        FilSpine? fsp = (spine as FilSpine);
        if (fsp is null)
        {
            throw new ConstructionErrorException("SimulSurf : this is not the spine of the fillet");
        }
        global::Kerf.Engine.Fillets.Blend.Line? lin = null;
        // Flexible parameters!!!
        double locfleche = 0.0, maxStep = 0.0;
        SimulParams(hGuide, fsp, ref maxStep, ref locfleche);
        NCollection_HArray1<CircSection>? sec = null;
        Point2D pf1 = new(), pl1 = new(), pf2 = new(), pl2 = new();

        ElSpine? emptyHGuide = null;

        double pFirst = first;
        if (intf != 0)
        {
            first = fsp.FirstParameter(1);
        }
        if (intl != 0)
        {
            last = fsp.LastParameter(fsp.EdgeCount);
        }
        if (fsp.IsConstant())
        {
            ConstRad func = new(s1, s2, hGuide);
            ConstRadInv fInv = new(s1, s2, hGuide);
            func.Set(fsp.Radius(), choix);
            fInv.Set(fsp.Radius(), choix);
            func.Set(_shape);
            _done = SimulData(data, hGuide, emptyHGuide, ref lin, s1, i1, s2, i2, func, fInv, pFirst, maxStep, locfleche,
                                              tolGuide, ref first, ref last, inside, appro, forward, soldep, 4, recOnS1, recOnS2);
            if (!_done)
            {
                return false;
            }
            int nbp = lin.NbPoints();
            sec = new NCollection_HArray1<CircSection>(1, nbp);
            for (int i = 1; i <= nbp; i++)
            {
                ref CircSection isec = ref sec.ChangeValue(i);
                double u1 = 0.0, v1 = 0.0, u2 = 0.0, v2 = 0.0, w = 0.0, p1 = 0.0, p2 = 0.0;
                Circle3D ci = new();
                BlendPoint p = lin.Point(i);
                p.ParametersOnS1(out u1, out v1);
                p.ParametersOnS2(out u2, out v2);
                w = p.Parameter();
                func.Section(w, u1, v1, u2, v2, out p1, out p2, ref ci);
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
            EvolRad func = new(s1, s2, hGuide, fsp.Law(hGuide));
            EvolRadInv fInv = new(s1, s2, hGuide, fsp.Law(hGuide));
            func.Set(choix);
            fInv.Set(choix);
            func.Set(_shape);
            _done = SimulData(data, hGuide, emptyHGuide, ref lin, s1, i1, s2, i2, func, fInv, pFirst, maxStep, locfleche,
                                              tolGuide, ref first, ref last, inside, appro, forward, soldep, 4, recOnS1, recOnS2);
            if (!_done)
            {
                return false;
            }
            int nbp = lin.NbPoints();
            sec = new NCollection_HArray1<CircSection>(1, nbp);
            for (int i = 1; i <= nbp; i++)
            {
                ref CircSection isec = ref sec.ChangeValue(i);
                double u1 = 0.0, v1 = 0.0, u2 = 0.0, v2 = 0.0, w = 0.0, p1 = 0.0, p2 = 0.0;
                Circle3D ci = new();
                BlendPoint p = lin.Point(i);
                p.ParametersOnS1(out u1, out v1);
                p.ParametersOnS2(out u2, out v2);
                w = p.Parameter();
                func.Section(w, u1, v1, u2, v2, out p1, out p2, ref ci);
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
        return true;
    }

    //=================================================================================================

    internal override void SimulSurf(SurfData? data, ElSpine? hGuide, Spine? spine, int choix, FaceSurfaceAdaptor? hs1, TopolTool? i1, EdgeCurve2DAdaptor? pc1, FaceSurfaceAdaptor? hSref1, EdgeCurve2DAdaptor? pCref1, ref bool decroch1, FaceSurfaceAdaptor? hs2, TopolTool? i2, Orientation or2, double _unused1, double tolGuide, ref double first, ref double last, bool inside, bool appro, bool forward, bool recP, bool recS, bool recRst, math_Vector soldep)
    {
        FilSpine? fsp = (spine as FilSpine);
        if (fsp is null)
        {
            throw new ConstructionErrorException("PerformSurf : this is not the spine of the fillet");
        }
        global::Kerf.Engine.Fillets.Blend.Line? lin = null;

        // Flexible parameters!
        double locfleche = 0.0, maxStep = 0.0;
        SimulParams(hGuide, fsp, ref maxStep, ref locfleche);
        NCollection_HArray1<CircSection>? sec = null;
        Point2D pf = new(), pl = new(), ppcf = new(), ppcl = new();

        double pFirst = first;
        if (fsp.IsConstant())
        {
            SurfRstConstRad func = new(hs2, hs1, pc1, hGuide);
            func.Set(hSref1, pCref1);
            CurveOnSurface? hc = new CurveOnSurface();
            hc.Load(pc1, hs1);
            SurfCurvConstRadInv finvc = new(hs2, hc, hGuide);
            SurfPointConstRadInv finvp = new(hs2, hGuide);
            ConstRadInv finv = new(hs2, hSref1, hGuide);
            finv.Set(false, pCref1);

            double rad = fsp.Radius();
            int petitchoix = 1;
            if (or2 == Reversed)
            {
                petitchoix = 3;
            }
            if (choix % 2 == 0)
            {
                petitchoix++;
            }
            finv.Set(rad, choix);
            finvc.Set(rad, petitchoix);
            finvp.Set(rad, petitchoix);
            func.Set(rad, petitchoix);
            func.Set(_shape);

            _done = SimulData(data, hGuide, ref lin, hs2, i2, hs1, pc1, i1, ref decroch1, func, finv, finvp, finvc, pFirst,
                                              maxStep, locfleche, tolGuide, ref first, ref last, soldep, 4, inside, appro, forward, recP, recS,
                                              recRst);
            if (!_done)
            {
                throw new FailureException("SimulSurf : Failed process!");
            }
            int nbp = lin.NbPoints();
            sec = new NCollection_HArray1<CircSection>(1, nbp);
            for (int i = 1; i <= nbp; i++)
            {
                ref CircSection isec = ref sec.ChangeValue(i);
                double u = 0.0, v = 0.0, w = 0.0, param = 0.0, p1 = 0.0, p2 = 0.0;
                Circle3D ci = new();
                BlendPoint p = lin.Point(i);
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
            SurfRstEvolRad func = new(hs2, hs1, pc1, hGuide, fsp.Law(hGuide));
            CurveOnSurface? hc = new CurveOnSurface();
            hc.Load(pc1, hs1);
            SurfCurvEvolRadInv finvc = new(hs2, hc, hGuide, fsp.Law(hGuide));
            SurfPointEvolRadInv finvp = new(hs2, hGuide, fsp.Law(hGuide));
            EvolRadInv finv = new(hs2, hSref1, hGuide, fsp.Law(hGuide));
            finv.Set(false, pCref1);
            int petitchoix = 1;
            if (or2 == Reversed)
            {
                petitchoix = 3;
            }
            if (choix % 2 == 0)
            {
                petitchoix++;
            }
            finv.Set(choix);
            finvc.Set(petitchoix);
            finvp.Set(petitchoix);
            func.Set(petitchoix);
            func.Set(_shape);
            _done = SimulData(data, hGuide, ref lin, hs2, i2, hs1, pc1, i1, ref decroch1, func, finv, finvp, finvc, pFirst,
                                              maxStep, locfleche, tolGuide, ref first, ref last, soldep, 4, inside, appro, forward, recP, recS,
                                              recRst);
            if (!_done)
            {
                throw new FailureException("SimulSurf : Fail !");
            }
            int nbp = lin.NbPoints();
            sec = new NCollection_HArray1<CircSection>(1, nbp);
            for (int i = 1; i <= nbp; i++)
            {
                ref CircSection isec = ref sec.ChangeValue(i);
                double u = 0.0, v = 0.0, w = 0.0, param = 0.0, p1 = 0.0, p2 = 0.0;
                Circle3D ci = new();
                BlendPoint p = lin.Point(i);
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
        data.SetSimul(sec);
        //  gp_Pnt2d pbid;
        data.Set2dPoints(ppcf, ppcl, pf, pl);
        ChFi3d_FilCommonPoint(lin.StartPointOnFirst(), lin.TransitionOnS1(), true, ref data.ChangeVertexFirstOnS2(),
                                                    _tolapp3d);
        ChFi3d_FilCommonPoint(lin.EndPointOnFirst(), lin.TransitionOnS1(), false, ref data.ChangeVertexLastOnS2(),
                                                    _tolapp3d);
        ChFi3d_FilCommonPoint(lin.StartPointOnSecond(), lin.TransitionOnS2(), true, ref data.ChangeVertexFirstOnS1(),
                                                    _tolapp3d);
        ChFi3d_FilCommonPoint(lin.EndPointOnSecond(), lin.TransitionOnS2(), false, ref data.ChangeVertexLastOnS1(),
                                                    _tolapp3d);
    }

    //=================================================================================================

    internal override void SimulSurf(SurfData? data, ElSpine? hGuide, Spine? spine, int choix, FaceSurfaceAdaptor? hs1, TopolTool? i1, Orientation or1, FaceSurfaceAdaptor? hs2, TopolTool? i2, EdgeCurve2DAdaptor? pc2, FaceSurfaceAdaptor? hSref2, EdgeCurve2DAdaptor? pCref2, ref bool decroch2, double _unused2, double tolGuide, ref double first, ref double last, bool inside, bool appro, bool forward, bool recP, bool recS, bool recRst, math_Vector soldep)
    {
        FilSpine? fsp = (spine as FilSpine);
        if (fsp is null)
        {
            throw new ConstructionErrorException("PerformSurf : it is not the spine of a fillet");
        }
        global::Kerf.Engine.Fillets.Blend.Line? lin = null;

        // Flexible parameters!
        double locfleche = 0.0, maxStep = 0.0;
        SimulParams(hGuide, fsp, ref maxStep, ref locfleche);
        NCollection_HArray1<CircSection>? sec = null;
        Point2D pf = new(), pl = new(), ppcf = new(), ppcl = new();

        double pFirst = first;
        if (fsp.IsConstant())
        {
            SurfRstConstRad func = new(hs1, hs2, pc2, hGuide);
            func.Set(hSref2, pCref2);
            CurveOnSurface? hc = new CurveOnSurface();
            hc.Load(pc2, hs2);
            SurfCurvConstRadInv finvc = new(hs1, hc, hGuide);
            SurfPointConstRadInv finvp = new(hs1, hGuide);
            ConstRadInv finv = new(hs1, hSref2, hGuide);
            finv.Set(false, pCref2);

            double rad = fsp.Radius();
            int petitchoix = 1;
            if (or1 == Reversed)
            {
                petitchoix = 3;
            }
            if (choix % 2 == 0)
            {
                petitchoix++;
            }
            finv.Set(rad, choix);
            finvc.Set(rad, petitchoix);
            finvp.Set(rad, petitchoix);
            func.Set(rad, petitchoix);
            func.Set(_shape);

            _done = SimulData(data, hGuide, ref lin, hs1, i1, hs2, pc2, i2, ref decroch2, func, finv, finvp, finvc, pFirst,
                                              maxStep, locfleche, tolGuide, ref first, ref last, soldep, 4, inside, appro, forward, recP, recS,
                                              recRst);
            if (!_done)
            {
                throw new FailureException("SimulSurf : Failed Processing!");
            }
            int nbp = lin.NbPoints();
            sec = new NCollection_HArray1<CircSection>(1, nbp);
            for (int i = 1; i <= nbp; i++)
            {
                ref CircSection isec = ref sec.ChangeValue(i);
                double u = 0.0, v = 0.0, w = 0.0, param = 0.0, p1 = 0.0, p2 = 0.0;
                Circle3D ci = new();
                BlendPoint p = lin.Point(i);
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
            SurfRstEvolRad func = new(hs1, hs2, pc2, hGuide, fsp.Law(hGuide));
            CurveOnSurface? hc = new CurveOnSurface();
            hc.Load(pc2, hs2);
            SurfCurvEvolRadInv finvc = new(hs1, hc, hGuide, fsp.Law(hGuide));
            SurfPointEvolRadInv finvp = new(hs1, hGuide, fsp.Law(hGuide));
            EvolRadInv finv = new(hs1, hSref2, hGuide, fsp.Law(hGuide));
            finv.Set(false, pCref2);
            int petitchoix = 1;
            if (or1 == Reversed)
            {
                petitchoix = 3;
            }
            if (choix % 2 == 0)
            {
                petitchoix++;
            }
            finv.Set(choix);
            finvc.Set(petitchoix);
            finvp.Set(petitchoix);
            func.Set(petitchoix);
            func.Set(_shape);
            _done = SimulData(data, hGuide, ref lin, hs1, i1, hs2, pc2, i2, ref decroch2, func, finv, finvp, finvc, pFirst,
                                              maxStep, locfleche, tolGuide, ref first, ref last, soldep, 4, inside, appro, forward, recP, recS,
                                              recRst);
            if (!_done)
            {
                throw new FailureException("SimulSurf : Fail !");
            }
            int nbp = lin.NbPoints();
            sec = new NCollection_HArray1<CircSection>(1, nbp);
            for (int i = 1; i <= nbp; i++)
            {
                ref CircSection isec = ref sec.ChangeValue(i);
                double u = 0.0, v = 0.0, w = 0.0, param = 0.0, p1 = 0.0, p2 = 0.0;
                Circle3D ci = new();
                BlendPoint p = lin.Point(i);
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
        data.SetSimul(sec);
        // gp_Pnt2d pbid;
        data.Set2dPoints(pf, pl, ppcf, ppcl);
        ChFi3d_FilCommonPoint(lin.StartPointOnFirst(), lin.TransitionOnS1(), true, ref data.ChangeVertexFirstOnS1(),
                                                    _tolapp3d);
        ChFi3d_FilCommonPoint(lin.EndPointOnFirst(), lin.TransitionOnS1(), false, ref data.ChangeVertexLastOnS1(),
                                                    _tolapp3d);
        ChFi3d_FilCommonPoint(lin.StartPointOnSecond(), lin.TransitionOnS2(), true, ref data.ChangeVertexFirstOnS2(),
                                                    _tolapp3d);
        ChFi3d_FilCommonPoint(lin.EndPointOnSecond(), lin.TransitionOnS2(), false, ref data.ChangeVertexLastOnS2(),
                                                    _tolapp3d);
    }

    //=================================================================================================

    internal override void SimulSurf(SurfData? data, ElSpine? hGuide, Spine? spine, int choix, FaceSurfaceAdaptor? hs1, TopolTool? i1, EdgeCurve2DAdaptor? pc1, FaceSurfaceAdaptor? hSref1, EdgeCurve2DAdaptor? pCref1, ref bool decroch1, Orientation or1, FaceSurfaceAdaptor? hs2, TopolTool? i2, EdgeCurve2DAdaptor? pc2, FaceSurfaceAdaptor? hSref2, EdgeCurve2DAdaptor? pCref2, ref bool decroch2, Orientation or2, double _unused3, double tolGuide, ref double first, ref double last, bool inside, bool appro, bool forward, bool recP1, bool recRst1, bool recP2, bool recRst2, math_Vector soldep)
    {
        FilSpine? fsp = (spine as FilSpine);
        if (fsp is null)
        {
            throw new ConstructionErrorException("PerformSurf : it is not the spine of a fillet");
        }
        global::Kerf.Engine.Fillets.Blend.Line? lin = null;

        // Flexible parameters!
        double locfleche = 0.0, maxStep = 0.0;
        SimulParams(hGuide, fsp, ref maxStep, ref locfleche);
        NCollection_HArray1<CircSection>? sec = null;
        //  gp_Pnt2d pf,pl;

        int ch1 = 1, ch2 = 2;
        double pFirst = first;

        if (fsp.IsConstant())
        {
            RstRstConstRad func = new(hs1, pc1, hs2, pc2, hGuide);
            func.Set(hSref1, pCref1, hSref2, pCref2);
            CurveOnSurface? hc1 = new CurveOnSurface();
            hc1.Load(pc1, hs1);
            CurveOnSurface? hc2 = new CurveOnSurface();
            hc2.Load(pc2, hs2);
            SurfCurvConstRadInv finv1 = new(hSref1, hc2, hGuide);
            CurvPointRadInv finvp1 = new(hGuide, hc2);
            SurfCurvConstRadInv finv2 = new(hSref2, hc1, hGuide);
            CurvPointRadInv finvp2 = new(hGuide, hc1);

            finv1.Set(pCref1);
            finv2.Set(pCref2);

            double rad = fsp.Radius();
            if (or1 == Reversed)
            {
                ch1 = 3;
            }
            if (or2 == Reversed)
            {
                ch2 = 3;
            }

            finv1.Set(rad, ch1);
            finvp1.Set(choix);
            finv2.Set(rad, ch2);
            finvp2.Set(choix);
            func.Set(rad, choix);
            func.Set(_shape);

            _done = SimulData(data, hGuide, ref lin, hs1, pc1, i1, ref decroch1, hs2, pc2, i2, ref decroch2, func, finv1,
                                              finvp1, finv2, finvp2, pFirst, maxStep, locfleche, tolGuide, ref first, ref last, soldep, 4,
                                              inside, appro, forward, recP1, recRst1, recP2, recRst2);
            if (!_done)
            {
                throw new FailureException("SimulSurf : Failed processing!");
            }
            int nbp = lin.NbPoints();
            sec = new NCollection_HArray1<CircSection>(1, nbp);
            for (int i = 1; i <= nbp; i++)
            {
                ref CircSection isec = ref sec.ChangeValue(i);
                double u = 0.0, v = 0.0, param = 0.0, p1 = 0.0, p2 = 0.0;
                Circle3D ci = new();
                BlendPoint p = lin.Point(i);
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
            RstRstEvolRad func = new(hs1, pc1, hs2, pc2, hGuide, fsp.Law(hGuide));
            func.Set(hSref1, pCref1, hSref2, pCref2);
            CurveOnSurface? hc1 = new CurveOnSurface();
            hc1.Load(pc1, hs1);
            CurveOnSurface? hc2 = new CurveOnSurface();
            hc2.Load(pc2, hs2);

            SurfCurvEvolRadInv finv1 = new(hSref1, hc2, hGuide, fsp.Law(hGuide));
            CurvPointRadInv finvp1 = new(hGuide, hc2);
            SurfCurvEvolRadInv finv2 = new(hSref2, hc1, hGuide, fsp.Law(hGuide));
            CurvPointRadInv finvp2 = new(hGuide, hc1);

            finv1.Set(pCref1);
            finv2.Set(pCref2);

            int ch11 = 1, ch22 = 2;

            if (or1 == Reversed)
            {
                ch11 = 3;
            }
            if (or2 == Reversed)
            {
                ch22 = 3;
            }

            finv1.Set(ch11);
            finvp1.Set(choix);
            finv2.Set(ch22);
            finvp2.Set(choix);
            func.Set(choix);
            func.Set(_shape);

            _done = SimulData(data, hGuide, ref lin, hs1, pc1, i1, ref decroch1, hs2, pc2, i2, ref decroch2, func, finv1,
                                              finvp1, finv2, finvp2, pFirst, maxStep, locfleche, tolGuide, ref first, ref last, soldep, 4,
                                              inside, appro, forward, recP1, recRst1, recP2, recRst2);

            if (!_done)
            {
                throw new FailureException("SimulSurf : Fail !");
            }
            int nbp = lin.NbPoints();
            sec = new NCollection_HArray1<CircSection>(1, nbp);
            for (int i = 1; i <= nbp; i++)
            {
                ref CircSection isec = ref sec.ChangeValue(i);
                double u = 0.0, v = 0.0, param = 0.0, p1 = 0.0, p2 = 0.0;
                Circle3D ci = new();
                BlendPoint p = lin.Point(i);
                u = p.ParameterOnC1();
                v = p.ParameterOnC2();
                param = p.Parameter();
                func.Section(param, u, v, out p1, out p2, ref ci);
                isec.Set(ci, p1, p2);
                //      if(i == 1) {pf.SetCoord(u,v);}
                //      if(i == nbp) {pl.SetCoord(u,v);}
            }
        }
        data.SetSimul(sec);
        //  gp_Pnt2d pbid;
        //  Data->Set2dPoints(pf,pl,pbid,pbid);

        ChFi3d_FilCommonPoint(lin.StartPointOnFirst(), lin.TransitionOnS1(), true, ref data.ChangeVertexFirstOnS1(),
                                                    _tolapp3d);
        ChFi3d_FilCommonPoint(lin.EndPointOnFirst(), lin.TransitionOnS1(), false, ref data.ChangeVertexLastOnS1(),
                                                    _tolapp3d);
        ChFi3d_FilCommonPoint(lin.StartPointOnSecond(), lin.TransitionOnS2(), true, ref data.ChangeVertexFirstOnS2(),
                                                    _tolapp3d);
        ChFi3d_FilCommonPoint(lin.EndPointOnSecond(), lin.TransitionOnS2(), false, ref data.ChangeVertexLastOnS2(),
                                                    _tolapp3d);
    }

    //=================================================================================================

    internal override bool PerformFirstSection(Spine? spine, ElSpine? hGuide, int choix, FaceSurfaceAdaptor? s1, FaceSurfaceAdaptor? s2, TopolTool? i1, TopolTool? i2, double par, math_Vector solDep, ref Containment pos1, ref Containment pos2)
    {
        FilSpine? fsp = (spine as FilSpine);
        if (fsp is null)
        {
            throw new ConstructionErrorException("PerformSurf : this is not the spine of a fillet");
        }
        double tolGuide = hGuide.Resolution(_tolapp3d);
        if (fsp.IsConstant())
        {
            ConstRad func = new(s1, s2, hGuide);
            func.Set(fsp.Radius(), choix);
            func.Set(_shape);
            Walking theWalk = new(s1, s2, i1, i2, hGuide);
            return theWalk.PerformFirstSection(func, par, solDep, _tolapp3d, tolGuide, out pos1, out pos2);
        }
        else
        {
            EvolRad func = new(s1, s2, hGuide, fsp.Law(hGuide));
            func.Set(choix);
            func.Set(_shape);
            Walking theWalk = new(s1, s2, i1, i2, hGuide);
            return theWalk.PerformFirstSection(func, par, solDep, _tolapp3d, tolGuide, out pos1, out pos2);
        }
    }

    //=================================================================================================

    internal override bool PerformSurf(NCollection_Sequence<SurfData> seqData, ElSpine? hGuide, Spine? spine, int choix, FaceSurfaceAdaptor? s1, TopolTool? i1, FaceSurfaceAdaptor? s2, TopolTool? i2, double maxStep, double fleche, double tolGuide, ref double first, ref double last, bool inside, bool appro, bool forward, bool recOnS1, bool recOnS2, math_Vector soldep, ref int intf, ref int intl)
    {
        SurfData? data = seqData[1];
        FilSpine? fsp = (spine as FilSpine);
        if (fsp is null)
        {
            throw new ConstructionErrorException("PerformSurf : this is not the spine of a fillet");
        }
        bool gd1 = false, gd2 = false, gf1 = false, gf2 = false, maybesingular = false;
        global::Kerf.Engine.Fillets.Blend.Line? lin = null;
        Orientation or = s1.Face().Orientation();
        double pFirst = first;
        if (intf != 0)
        {
            first = fsp.FirstParameter(1);
        }
        if (intl != 0)
        {
            last = fsp.LastParameter(fsp.EdgeCount);
        }
        if (fsp.IsConstant())
        {
            ConstRad func = new(s1, s2, hGuide);
            ConstRadInv fInv = new(s1, s2, hGuide);
            func.Set(fsp.Radius(), choix);
            fInv.Set(fsp.Radius(), choix);
            func.Set(_shape);


            _done = ComputeData(data, hGuide, spine, ref lin, s1, i1, s2, i2, func, fInv, pFirst, maxStep, fleche, tolGuide,
                                                  ref first, ref last, inside, appro, forward, soldep, ref intf, ref intl, ref gd1, ref gd2,
                                                  ref gf1, ref gf2, recOnS1, recOnS2);


            if (!_done)
            {
                return false; // recovery is possible PMN 14/05/1998
            }


            _done = CompleteData(data, func, lin, s1, s2, or, gd1, gd2, gf1, gf2);


            if (!_done)
            {
                throw new FailureException("PerformSurf : Failed approximation!");
            }
            maybesingular = (func.GetMinimalDistance() <= 100 * _tolapp3d);
        }
        else
        {
            EvolRad func = new(s1, s2, hGuide, fsp.Law(hGuide));
            EvolRadInv fInv = new(s1, s2, hGuide, fsp.Law(hGuide));
            func.Set(choix);
            fInv.Set(choix);
            func.Set(_shape);


            _done = ComputeData(data, hGuide, spine, ref lin, s1, i1, s2, i2, func, fInv, pFirst, maxStep, fleche, tolGuide,
                                                  ref first, ref last, inside, appro, forward, soldep, ref intf, ref intl, ref gd1, ref gd2,
                                                  ref gf1, ref gf2, recOnS1, recOnS2);

            if (!_done)
            {
                return false;
            }


            _done = CompleteData(data, func, lin, s1, s2, or, gd1, gd2, gf1, gf2);


            if (!_done)
            {
                throw new FailureException("PerformSurf : Failed approximation!");
            }
            maybesingular = (func.GetMinimalDistance() <= 100 * _tolapp3d);
        }
        if (maybesingular)
        {
            SplitSurf(seqData, lin);
        }
        return true;
    }

    //=================================================================================================

    internal override void PerformSurf(NCollection_Sequence<SurfData> seqData, ElSpine? hGuide, Spine? spine, int choix, FaceSurfaceAdaptor? hs1, TopolTool? i1, EdgeCurve2DAdaptor? pc1, FaceSurfaceAdaptor? hSref1, EdgeCurve2DAdaptor? pCref1, ref bool decroch1, FaceSurfaceAdaptor? hs2, TopolTool? i2, Orientation or2, double maxStep, double fleche, double tolGuide, ref double first, ref double last, bool inside, bool appro, bool forward, bool recP, bool recS, bool recRst, math_Vector soldep)
    {
        SurfData? data = seqData[1];
        FilSpine? fsp = (spine as FilSpine);
        if (fsp is null)
        {
            throw new ConstructionErrorException("PerformSurf : this is not the spine of a fillet");
        }
        global::Kerf.Engine.Fillets.Blend.Line? lin = null;
        double pFirst = first;
        bool maybesingular = false;

        if (fsp.IsConstant())
        {
            SurfRstConstRad func = new(hs2, hs1, pc1, hGuide);
            func.Set(hSref1, pCref1);
            CurveOnSurface? hc = new CurveOnSurface();
            hc.Load(pc1, hs1);
            SurfCurvConstRadInv finvc = new(hs2, hc, hGuide);
            SurfPointConstRadInv finvp = new(hs2, hGuide);
            ConstRadInv finv = new(hs2, hSref1, hGuide);
            finv.Set(false, pCref1);

            double rad = fsp.Radius();
            int petitchoix = 1;
            if (or2 == Reversed)
            {
                petitchoix = 3;
            }
            if (choix % 2 == 0)
            {
                petitchoix++;
            }
            finv.Set(rad, choix);
            finvc.Set(rad, petitchoix);
            finvp.Set(rad, petitchoix);
            func.Set(rad, petitchoix);
            func.Set(_shape);

            _done = ComputeData(data, hGuide, ref lin, hs2, i2, hs1, pc1, i1, ref decroch1, func, finv, finvp, finvc, pFirst, maxStep,
                                                  fleche, tolGuide, ref first, ref last, soldep, inside, appro, forward, recP, recS, recRst);
            if (!_done)
            {
                spine.SetErrorStatus(ErrorStatus.WalkingFailure);
                throw new FailureException("PerformSurf : Failed processing!");
            }
            Orientation or = hs2.Face().Orientation();
            _done = CompleteData(data, func, lin, hs1, hs2, or, true);
            if (!_done)
            {
                throw new FailureException("PerformSurf : Failed approximation!");
            }
            maybesingular = (func.GetMinimalDistance() <= 100 * _tolapp3d);
        }
        else
        {
            SurfRstEvolRad func = new(hs2, hs1, pc1, hGuide, fsp.Law(hGuide));
            func.Set(hSref1, pCref1);
            CurveOnSurface? hc = new CurveOnSurface();
            hc.Load(pc1, hs1);
            SurfCurvEvolRadInv finvc = new(hs2, hc, hGuide, fsp.Law(hGuide));
            SurfPointEvolRadInv finvp = new(hs2, hGuide, fsp.Law(hGuide));
            EvolRadInv finv = new(hs2, hSref1, hGuide, fsp.Law(hGuide));
            finv.Set(false, pCref1);
            int petitchoix = 1;
            if (or2 == Reversed)
            {
                petitchoix = 3;
            }
            if (choix % 2 == 0)
            {
                petitchoix++;
            }
            finv.Set(choix);
            finvc.Set(petitchoix);
            finvp.Set(petitchoix);
            func.Set(petitchoix);
            func.Set(_shape);
            _done = ComputeData(data, hGuide, ref lin, hs2, i2, hs1, pc1, i1, ref decroch1, func, finv, finvp, finvc, pFirst, maxStep,
                                                  fleche, tolGuide, ref first, ref last, soldep, inside, appro, forward, recP, recS, recRst);
            if (!_done)
            {
                spine.SetErrorStatus(ErrorStatus.WalkingFailure);
                throw new FailureException("PerformSurf : Failed processing!");
            }
            Orientation or = hs2.Face().Orientation();
            _done = CompleteData(data, func, lin, hs1, hs2, or, true);
            if (!_done)
            {
                throw new FailureException("PerformSurf : Failed approximation!");
            }
            maybesingular = (func.GetMinimalDistance() <= 100 * _tolapp3d);
        }
        if (maybesingular)
        {
            SplitSurf(seqData, lin);
        }
    }

    //=================================================================================================

    internal override void PerformSurf(NCollection_Sequence<SurfData> seqData, ElSpine? hGuide, Spine? spine, int choix, FaceSurfaceAdaptor? hs1, TopolTool? i1, Orientation or1, FaceSurfaceAdaptor? hs2, TopolTool? i2, EdgeCurve2DAdaptor? pc2, FaceSurfaceAdaptor? hSref2, EdgeCurve2DAdaptor? pCref2, ref bool decroch2, double maxStep, double fleche, double tolGuide, ref double first, ref double last, bool inside, bool appro, bool forward, bool recP, bool recS, bool recRst, math_Vector soldep)
    {
        SurfData? data = seqData[1];
        FilSpine? fsp = (spine as FilSpine);
        if (fsp is null)
        {
            throw new ConstructionErrorException("PerformSurf : this is not the spine of a fillet");
        }
        global::Kerf.Engine.Fillets.Blend.Line? lin = null;
        double pFirst = first;
        bool maybesingular = false;

        if (fsp.IsConstant())
        {
            SurfRstConstRad func = new(hs1, hs2, pc2, hGuide);
            func.Set(hSref2, pCref2);
            CurveOnSurface? hc = new CurveOnSurface();
            hc.Load(pc2, hs2);
            SurfCurvConstRadInv finvc = new(hs1, hc, hGuide);
            SurfPointConstRadInv finvp = new(hs1, hGuide);
            ConstRadInv finv = new(hs1, hSref2, hGuide);
            finv.Set(false, pCref2);

            double rad = fsp.Radius();
            int petitchoix = 1;
            if (or1 == Reversed)
            {
                petitchoix = 3;
            }
            if (choix % 2 == 0)
            {
                petitchoix++;
            }
            finv.Set(rad, choix);
            finvc.Set(rad, petitchoix);
            finvp.Set(rad, petitchoix);
            func.Set(rad, petitchoix);
            func.Set(_shape);

            _done = ComputeData(data, hGuide, ref lin, hs1, i1, hs2, pc2, i2, ref decroch2, func, finv, finvp, finvc, pFirst, maxStep,
                                                  fleche, tolGuide, ref first, ref last, soldep, inside, appro, forward, recP, recS, recRst);
            if (!_done)
            {
                spine.SetErrorStatus(ErrorStatus.WalkingFailure);
                throw new FailureException("PerformSurf : Failed processing!");
            }
            Orientation or = hs1.Face().Orientation();
            _done = CompleteData(data, func, lin, hs1, hs2, or, false);
            if (!_done)
            {
                throw new FailureException("PerformSurf : Failed approximation!");
            }
            maybesingular = (func.GetMinimalDistance() <= 100 * _tolapp3d);
        }
        else
        {
            SurfRstEvolRad func = new(hs1, hs2, pc2, hGuide, fsp.Law(hGuide));
            func.Set(hSref2, pCref2);
            CurveOnSurface? hc = new CurveOnSurface();
            hc.Load(pc2, hs2);
            SurfCurvEvolRadInv finvc = new(hs1, hc, hGuide, fsp.Law(hGuide));
            SurfPointEvolRadInv finvp = new(hs1, hGuide, fsp.Law(hGuide));
            EvolRadInv finv = new(hs1, hSref2, hGuide, fsp.Law(hGuide));
            finv.Set(false, pCref2);
            int petitchoix = 1;
            if (or1 == Reversed)
            {
                petitchoix = 3;
            }
            if (choix % 2 == 0)
            {
                petitchoix++;
            }
            finv.Set(choix);
            finvc.Set(petitchoix);
            finvp.Set(petitchoix);
            func.Set(petitchoix);
            func.Set(_shape);

            _done = ComputeData(data, hGuide, ref lin, hs1, i1, hs2, pc2, i2, ref decroch2, func, finv, finvp, finvc, pFirst, maxStep,
                                                  fleche, tolGuide, ref first, ref last, soldep, inside, appro, forward, recP, recS, recRst);
            if (!_done)
            {
                spine.SetErrorStatus(ErrorStatus.WalkingFailure);
                throw new FailureException("PerformSurf : Failed processing!");
            }
            Orientation or = hs1.Face().Orientation();
            _done = CompleteData(data, func, lin, hs1, hs2, or, false);
            if (!_done)
            {
                throw new FailureException("PerformSurf : Failed approximation!");
            }
            maybesingular = (func.GetMinimalDistance() <= 100 * _tolapp3d);
        }
        if (maybesingular)
        {
            SplitSurf(seqData, lin);
        }
    }

    //=================================================================================================

    internal override void PerformSurf(NCollection_Sequence<SurfData> seqData, ElSpine? hGuide, Spine? spine, int choix, FaceSurfaceAdaptor? hs1, TopolTool? i1, EdgeCurve2DAdaptor? pc1, FaceSurfaceAdaptor? hSref1, EdgeCurve2DAdaptor? pCref1, ref bool decroch1, Orientation or1, FaceSurfaceAdaptor? hs2, TopolTool? i2, EdgeCurve2DAdaptor? pc2, FaceSurfaceAdaptor? hSref2, EdgeCurve2DAdaptor? pCref2, ref bool decroch2, Orientation or2, double maxStep, double fleche, double tolGuide, ref double first, ref double last, bool inside, bool appro, bool forward, bool recP1, bool recRst1, bool recP2, bool recRst2, math_Vector soldep)
    {
        SurfData? data = seqData[1];
        FilSpine? fsp = (spine as FilSpine);
        if (fsp is null)
        {
            throw new ConstructionErrorException("PerformSurf : this is not the spine of a fillet");
        }
        global::Kerf.Engine.Fillets.Blend.Line? lin = null;
        double pFirst = first;
        bool maybesingular = false;

        if (fsp.IsConstant())
        {
            RstRstConstRad func = new(hs1, pc1, hs2, pc2, hGuide);
            func.Set(hSref1, pCref1, hSref2, pCref2);
            CurveOnSurface? hc1 = new CurveOnSurface();
            hc1.Load(pc1, hs1);
            CurveOnSurface? hc2 = new CurveOnSurface();
            hc2.Load(pc2, hs2);
            SurfCurvConstRadInv finv1 = new(hSref1, hc2, hGuide);
            CurvPointRadInv finvp1 = new(hGuide, hc2);
            SurfCurvConstRadInv finv2 = new(hSref2, hc1, hGuide);
            CurvPointRadInv finvp2 = new(hGuide, hc1);

            finv1.Set(pCref1);
            finv2.Set(pCref2);

            int ch1 = 1, ch2 = 2;
            double rad = fsp.Radius();
            if (or1 == Reversed)
            {
                ch1 = 3;
            }
            if (or2 == Reversed)
            {
                ch2 = 3;
            }

            finv1.Set(rad, ch1);
            finvp1.Set(choix);
            finv2.Set(rad, ch2);
            finvp2.Set(choix);
            func.Set(rad, choix);
            func.Set(_shape);

            _done = ComputeData(data, hGuide, ref lin, hs1, pc1, i1, ref decroch1, hs2, pc2, i2, ref decroch2, func, finv1,
                                                  finvp1, finv2, finvp2, pFirst, maxStep, fleche, tolGuide, ref first, ref last, soldep, inside,
                                                  appro, forward, recP1, recRst1, recP2, recRst2);
            if (!_done)
            {
                spine.SetErrorStatus(ErrorStatus.WalkingFailure);
                throw new FailureException("PerformSurf : Failed processing!");
            }
            Orientation or = hs1.Face().Orientation();
            _done = CompleteData(data, func, lin, hs1, hs2, or);
            if (!_done)
            {
                throw new FailureException("PerformSurf : Failed approximation!");
            }
            maybesingular = (func.GetMinimalDistance() <= 100 * _tolapp3d);
        }
        else
        {
            RstRstEvolRad func = new(hs1, pc1, hs2, pc2, hGuide, fsp.Law(hGuide));
            func.Set(hSref1, pCref1, hSref2, pCref2);
            CurveOnSurface? hc1 = new CurveOnSurface();
            hc1.Load(pc1, hs1);
            CurveOnSurface? hc2 = new CurveOnSurface();
            hc2.Load(pc2, hs2);

            SurfCurvEvolRadInv finv1 = new(hSref1, hc2, hGuide, fsp.Law(hGuide));
            CurvPointRadInv finvp1 = new(hGuide, hc2);
            SurfCurvEvolRadInv finv2 = new(hSref2, hc1, hGuide, fsp.Law(hGuide));
            CurvPointRadInv finvp2 = new(hGuide, hc1);

            finv1.Set(pCref1);
            finv2.Set(pCref2);

            int ch1 = 1, ch2 = 2;

            if (or1 == Reversed)
            {
                ch1 = 3;
            }
            if (or2 == Reversed)
            {
                ch2 = 3;
            }

            finv1.Set(ch1);
            finvp1.Set(choix);
            finv2.Set(ch2);
            finvp2.Set(choix);
            func.Set(choix);
            func.Set(_shape);

            _done = ComputeData(data, hGuide, ref lin, hs1, pc1, i1, ref decroch1, hs2, pc2, i2, ref decroch2, func, finv1,
                                                  finvp1, finv2, finvp2, pFirst, maxStep, fleche, tolGuide, ref first, ref last, soldep, inside,
                                                  appro, forward, recP1, recRst1, recP2, recRst2);

            if (!_done)
            {
                spine.SetErrorStatus(ErrorStatus.WalkingFailure);
                throw new FailureException("PerformSurf : Failed processing!");
            }
            Orientation or = hs1.Face().Orientation();
            _done = CompleteData(data, func, lin, hs1, hs2, or);
            if (!_done)
            {
                throw new FailureException("PerformSurf : Failed approximation!");
            }
            maybesingular = (func.GetMinimalDistance() <= 100 * _tolapp3d);
        }
        if (maybesingular)
        {
            SplitSurf(seqData, lin);
        }
    }

    //=================================================================================================

    internal void SplitSurf(NCollection_Sequence<SurfData> seqData, global::Kerf.Engine.Fillets.Blend.Line? line)
    {
        int ii = 0, nbpnt = line.NbPoints();
        if (nbpnt < 3)
        {
            return;
        }
        double uFirst = 0.0, uLast = 0.0, vFirst = 0.0, vLast = 0.0;
        DsDataStructure dStr = _ds.ChangeDS();
        int iSurf = 0;
        SurfData @ref = seqData[1]; // "ref" in OCCT
        BlendPoint p = new();

        iSurf = @ref.Surf();
        Surface? surf = dStr.Surface(iSurf).Surface();
        surf.Bounds(out uFirst, out uLast, out vFirst, out vLast);
        Curve3D? courbe1 = surf.UIso(uFirst);
        Curve3D? courbe2 = surf.UIso(uLast);
        SearchSing fonc = new(courbe1, courbe2);

        NCollection_Sequence<double> lesVi = new();
        double precedant = 0.0, suivant = 0.0, courant = 0.0;
        double a = 0.0, b = 0.0, c = 0.0;

        // (1) Finds vi so that iso v=vi is punctual
        vFirst =
            Min(@ref.InterferenceOnS1().FirstParameter(), @ref.InterferenceOnS2().FirstParameter());
        vLast =
            Max(@ref.InterferenceOnS1().LastParameter(), @ref.InterferenceOnS2().LastParameter());

        // (1.1) Finds the first point inside
        for (ii = 1; ii <= nbpnt && line.Point(ii).Parameter() < vFirst; ii++)
        {
        }
        if (ii == 1)
        {
            ii++;
        }
        p = line.Point(ii);
        b = p.Parameter();
        courant = p.PointOnS1().Distance(p.PointOnS2());
        p = line.Point(ii - 1);
        a = p.Parameter();
        precedant = p.PointOnS1().Distance(p.PointOnS2());
        ii++;

        // (1.2) Find a minimum by "points"
        for (; ii <= nbpnt && line.Point(ii).Parameter() <= vLast; ii++)
        {
            for (; ii <= nbpnt && line.Point(ii).Parameter() < vLast
                          && line.Point(ii).Parameter() - b < global::Kerf.Engine.Foundation.Precision.PConfusion();
                      ii++)
            {
            }

            BlendPoint pnt = line.Point(ii);
            c = pnt.Parameter();
            suivant = pnt.PointOnS1().Distance(pnt.PointOnS2());
            if ((courant < precedant) && (courant < suivant))
            {
                // (1.3) Find the exact minimum
                FunctionRoot resol = new(fonc, (a + c) / 2,
                                              _tol2d, // Surf->VResolution(toleps),
                                              a, c, 50);
                if (resol.IsDone)
                {
                    double val = 0.0, racine = resol.Root();

                    fonc.Value(resol.Root(), out val);
                    if (val < _tolapp3d)
                    {
                        // the solution (avoiding the risks of confusion)
                        if (lesVi.Length() == 0)
                        {
                            if ((racine > vFirst + _tol2d) && (racine < vLast - _tol2d))
                            {
                                lesVi.Append(racine);
                            }
                        }
                        else
                        {
                            if ((racine > lesVi[lesVi.Length()] + _tol2d) && (racine < vLast - _tol2d))
                            {
                                lesVi.Append(racine);
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
        if (lesVi.Length() > 0)
        {
            DsDataStructure dStru = _ds.ChangeDS();
            SurfData? sd = null;
            DsSurface s = new();
            DsCurve c1 = new(), c2 = new();
            double t = 0.0, vertexTol = 0.0;
            Point3D p3d = new(), p1 = new(), p2 = new();
            CommonPoint lePoint = new();
            for (ii = 1; ii <= lesVi.Length(); ii++)
            {

                t = lesVi[ii];
                // (2.0) copy and insertion
                sd = new SurfData();
                sd.Copy(@ref);
                seqData.InsertBefore(ii, sd);
                s = dStru.Surface(@ref.Surf());
                sd.ChangeSurf(dStru.AddSurface(s));
                c1 = dStru.Curve(sd.InterferenceOnS1().LineIndex());
                sd.ChangeInterferenceOnS1().SetLineIndex(dStru.AddCurve(c1));
                c2 = dStru.Curve(sd.InterferenceOnS2().LineIndex());
                sd.ChangeInterferenceOnS2().SetLineIndex(dStru.AddCurve(c2));

                // (2.1) Modification of common Point
                sd.ChangeVertexLastOnS1().Reset();
                sd.ChangeVertexLastOnS2().Reset();
                @ref.ChangeVertexFirstOnS1().Reset();
                @ref.ChangeVertexFirstOnS2().Reset();
                courbe1.D0(t, out p1);
                courbe2.D0(t, out p2);
                p3d.SetXYZ((p1.XYZ() + p2.XYZ()) / 2);
                vertexTol = p1.Distance(p2);
                vertexTol += Max(c1.Tolerance(), c2.Tolerance());

                sd.ChangeVertexLastOnS1().SetPoint(p3d);
                sd.ChangeVertexLastOnS2().SetPoint(p3d);
                @ref.ChangeVertexFirstOnS1().SetPoint(p3d);
                @ref.ChangeVertexFirstOnS2().SetPoint(p3d);
                sd.ChangeVertexLastOnS1().SetTolerance(vertexTol);
                sd.ChangeVertexLastOnS2().SetTolerance(vertexTol);
                @ref.ChangeVertexFirstOnS1().SetTolerance(vertexTol);
                @ref.ChangeVertexFirstOnS2().SetTolerance(vertexTol);

                // (2.2) Modification of interferences
                sd.ChangeInterferenceOnS1().SetLastParameter(t);
                sd.ChangeInterferenceOnS2().SetLastParameter(t);
                @ref.ChangeInterferenceOnS1().SetFirstParameter(t);
                @ref.ChangeInterferenceOnS2().SetFirstParameter(t);

                // Parameters on ElSpine
                sd.LastSpineParam(t);
                @ref.FirstSpineParam(t);
            }
        }
    }

    //=================================================================================================

    internal override void ExtentOneCorner(in Vertex v, Stripe? s)
    {
        // review by using the data at end of fillets (point, radius, normal
        // to the faces and tangents of the guideline).
        int sens = 0;
        double coeff = 0.5;
        Spine? spine = s.Spine();
        ChFi3d_IndexOfSurfData(v, s, out sens);
        double dU = spine.LastParameter(spine.EdgeCount);
        if (spine.IsTangencyExtremity((sens == 1)))
        {
            return; // No extension in the queue
        }

        if (spine.Status((sens == 1)) == State.FreeBoundary)
        {
            coeff *= 2; // It is necessary to go to the end and to evaluate the length
        }

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

    //=================================================================================================

    internal override void ExtentTwoCorner(in Vertex v, NCollection_List<Stripe> ls)
    {
        // Review by using the data at end of fillets (point, radius, normal
        // to faces and tangents to the guideline.
        int sens = 0;
        double coeff = 0.3, eval = 0.0, dU, rad;
        NCollection_List<Stripe>.Iterator itel = new(ls);
        bool ff = true;
        Stripe? stripe = null;
        Spine? spine = null;

        // A value of symmetric extension is calculated
        for (; itel.More(); itel.Next())
        {
            stripe = itel.Value();
            spine = stripe.Spine();
            dU = spine.LastParameter(spine.EdgeCount) * coeff;
            FilSpine? fsp = (spine as FilSpine);
            if (fsp.IsConstant())
            {
                rad = fsp.Radius();
            }
            else
            {
                Edge e = ChFi3d_EdgeFromV1(v, itel.Value(), out sens);
                rad = MaxRad(fsp, e);
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
            if (dU > eval)
            {
                eval = dU;
            }
        }

        // One applies
        for (itel.Initialize(ls); itel.More(); itel.Next())
        {
            ChFi3d_IndexOfSurfData(v, itel.Value(), out sens);
            if (!ff && stripe == itel.Value())
            {
                sens = -sens;
            }
            stripe = itel.Value();
            spine = stripe.Spine();
            if (!spine.IsTangencyExtremity((sens == 1)))
            { // No extension on queue
                if (sens == 1)
                {
                    spine.SetFirstParameter(-eval);
                    spine.SetFirstTgt(0.0);
                }
                else
                {
                    dU = spine.LastParameter(spine.EdgeCount);
                    spine.SetLastParameter(dU + eval);
                    spine.SetLastTgt(dU);
                }
                ff = false;
            }
        }
    }

    //=================================================================================================

    internal override void ExtentThreeCorner(in Vertex v, NCollection_List<Stripe> ls)
    {
        // Review by using the data at end of fillets (point, radius, normal
        // to faces and tangents to the guideline.
        int sens = 0;
        double coeff = 0.1;
        NCollection_List<Stripe> check = new();
        //  bool FF = true;
        for (NCollection_List<Stripe>.Iterator itel = new(ls); itel.More(); itel.Next())
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
            Spine? spine = stripe.Spine();
            if (spine.IsTangencyExtremity((sens == 1)))
            {
                return; // No extension on queue
            }
            double dU = spine.LastParameter(spine.EdgeCount);
            if (sens == 1)
            {
                if (spine.GetTypeOfConcavity() != TypeOfConcavity.Convex || spine.FirstStatus() != State.OnSame)
                {
                    spine.SetFirstParameter(-dU * coeff);
                    spine.SetFirstTgt(0.0);
                }
            }
            else
            {
                if (spine.GetTypeOfConcavity() != TypeOfConcavity.Convex || spine.LastStatus() != State.OnSame)
                {
                    spine.SetLastParameter(dU * (1.0 + coeff));
                    spine.SetLastTgt(dU);
                }
            }
            check.Append(stripe);
        }
    }

    //=================================================================================================

    internal override void SetRegul()
    {
        NCollection_List<Regul>.Iterator it = new();
        NCollection_List<Shape>.Iterator itc = new();
        NCollection_List<Shape>.Iterator its1 = new();
        NCollection_List<Shape>.Iterator its2 = new();
        BrepBuilder b = new();
        for (it.Initialize(_regul); it.More(); it.Next())
        {
            Regul reg = it.Value();
            itc.Initialize(_coup.NewEdges(reg.Curve()));
            if (itc.More())
            {
                Edge e = ShapeCasts.Edge(itc.Value());
                if (reg.IsSurface1)
                {
                    its1.Initialize(_coup.NewFaces(reg.S1()));
                }
                else
                {
                    its1.Initialize(_coup.Merged(_ds.Shape(reg.S1()), In));
                }
                if (reg.IsSurface2)
                {
                    its2.Initialize(_coup.NewFaces(reg.S2()));
                }
                else
                {
                    its2.Initialize(_coup.Merged(_ds.Shape(reg.S2()), In));
                }
                if (its1.More() && its2.More())
                {
                    Face f1 = ShapeCasts.Face(its1.Value());
                    Face f2 = ShapeCasts.Face(its2.Value());
                    Continuity cont = ChFi3d_evalconti(e, f1, f2);
                    b.Continuity(e, f1, f2, cont);
                }
            }
        }
    }
}
