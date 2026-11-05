// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Pipe.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Pipe.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Pipe.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Properties;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Engine.Construction.Surfaces.ApproxStyle;
using static Kerf.Engine.Construction.Surfaces.SurfacePipeError;
using static Kerf.Engine.Construction.Surfaces.Trihedron;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// Builds pipes: the surface swept by a section along a path. A circle of constant radius, a constant section (with
/// a corrected Frenet, Frenet, fixed, constant-normal or Darboux trihedron, or a constant binormal), a section
/// evolving between two or N sections, a section following a guide, or a pipe of constant radius between two guide
/// lines (a pipe on an edge). Elementary surfaces are recognised where possible; otherwise the result is a B-spline
/// surface. The u direction usually follows the section and v the path; <see cref="ExchangeUV"/> tells when an
/// elementary surface is parametrised the other way.
/// </summary>
/// <remarks>
/// A working object, as OCCT's: it keeps the caller's adaptors and builds its laws on copies of the caller's paths
/// (where OCCT copies them) and on sections placed by <see cref="SurfaceSectionPlacement"/> (new geometry).
/// <see cref="Surface"/> hands out the surface frozen.
/// </remarks>
internal sealed class SurfacePipe
{
    private SurfacePipeError _status; //!< Execution status
    private double _radius;
    private double _error;
    private CurveAdaptorBase? _adpPath;
    private CurveAdaptorBase? _adpFirstSect;
    private CurveAdaptorBase? _adpLastSect;
    private Surface? _surface;
    private SurfaceLocationLaw? _loc;
    private SurfaceSectionLaw? _sec;
    private int _type;
    private bool _exchUV;
    private bool _kPart;
    private bool _polynomial;

    private static bool CheckSense(NCollection_Sequence<Curve3D> seq1, NCollection_Sequence<Curve3D> seq2)
    {
        // initialisation
        bool no_sing = true;
        seq2.Clear();

        Curve3D c1 = seq1.Value(1);
        double f = c1.FirstParameter, l = c1.LastParameter;
        int iP, np = 21;
        NCollection_Array1<Point3D> tab = new(1, np);
        double u = f, h = Math.Abs(f - l) / 20.0;
        for (iP = 1; iP <= np; iP++)
        {
            c1.D0(u, out tab[iP]);
            u += h;
            if ((u - f) * (u - l) > 0.0)
            {
                u = l;
            }
        }
        Frame3D axeRef, axe;
        Point3D pos;
        bool sing;
        global::Kerf.Engine.Geometry.Library.GeomLib.AxeOfInertia(tab, out axeRef, out sing);

        // si la section est une droite, ca ne marche pas
        if (sing)
        {
            no_sing = false;
        }

        pos = axeRef.Location;
        double alpha1, alpha2, alpha3;
        Point3D p1, p2;
        u = (f + l - h) / 2 - h;
        c1.D0(u, out p1);
        u += h;
        c1.D0(u, out p2);
        alpha1 = new Vector3D(pos, p1).AngleWithRef(new Vector3D(pos, p2), new Vector3D(axeRef.Direction));
        p1 = p2;
        u += h;
        c1.D0(u, out p2);
        alpha2 = new Vector3D(pos, p1).AngleWithRef(new Vector3D(pos, p2), new Vector3D(axeRef.Direction));
        p1 = p2;
        u += h;
        c1.D0(u, out p2);
        alpha3 = new Vector3D(pos, p1).AngleWithRef(new Vector3D(pos, p2), new Vector3D(axeRef.Direction));
        seq2.Append(c1);

        for (int iseq = 2; iseq <= seq1.Length(); iseq++)
        {
            // discretisation de C2
            Curve3D c2 = seq1.Value(iseq);
            f = c2.FirstParameter;
            l = c2.LastParameter;
            u = f;
            for (iP = 1; iP <= np; iP++)
            {
                c2.D0(u, out tab[iP]);
                u += h;
                if ((u - f) * (u - l) > 0.0)
                {
                    u = l;
                }
            }
            global::Kerf.Engine.Geometry.Library.GeomLib.AxeOfInertia(tab, out axe, out sing);

            // si la section est une droite, ca ne marche pas
            if (sing)
            {
                no_sing = false;
            }

            pos = axe.Location;
            double beta1, beta2, beta3;
            u = (f + l - h) / 2 - h;
            c2.D0(u, out p1);
            u += h;
            c2.D0(u, out p2);
            beta1 = new Vector3D(pos, p1).AngleWithRef(new Vector3D(pos, p2), new Vector3D(axeRef.Direction));
            p1 = p2;
            u += h;
            c2.D0(u, out p2);
            beta2 = new Vector3D(pos, p1).AngleWithRef(new Vector3D(pos, p2), new Vector3D(axeRef.Direction));
            p1 = p2;
            u += h;
            c2.D0(u, out p2);
            beta3 = new Vector3D(pos, p1).AngleWithRef(new Vector3D(pos, p2), new Vector3D(axeRef.Direction));

            // meme sens ?
            bool ok = true,
                 pasnul1 = (Math.Abs(alpha1) > global::Kerf.Engine.Foundation.Precision.Confusion()) && (Math.Abs(beta1) > global::Kerf.Engine.Foundation.Precision.Confusion()),
                 pasnul2 = (Math.Abs(alpha2) > global::Kerf.Engine.Foundation.Precision.Confusion()) && (Math.Abs(beta2) > global::Kerf.Engine.Foundation.Precision.Confusion()),
                 pasnul3 = (Math.Abs(alpha3) > global::Kerf.Engine.Foundation.Precision.Confusion()) && (Math.Abs(beta3) > global::Kerf.Engine.Foundation.Precision.Confusion());
            if (pasnul1 && pasnul2 && pasnul3)
            {
                if (alpha1 * beta1 > 0.0)
                {
                    ok = (alpha2 * beta2 > 0.0) || (alpha3 * beta3 > 0.0);
                }
                else
                {
                    ok = (alpha2 * beta2 > 0.0) && (alpha3 * beta3 > 0.0);
                }
            }
            else if (pasnul1 && pasnul2 && !pasnul3)
            {
                ok = (alpha1 * beta1 > 0.0) || (alpha2 * beta2 > 0.0);
            }
            else if (pasnul1 && !pasnul2 && pasnul3)
            {
                ok = (alpha1 * beta1 > 0.0) || (alpha3 * beta3 > 0.0);
            }
            else if (!pasnul1 && pasnul2 && pasnul3)
            {
                ok = (alpha2 * beta2 > 0.0) || (alpha3 * beta3 > 0.0);
            }
            else if (pasnul1)
            {
                ok = (alpha1 * beta1 > 0.0);
            }
            else if (pasnul2)
            {
                ok = (alpha2 * beta2 > 0.0);
            }
            else if (pasnul3)
            {
                ok = (alpha3 * beta3 > 0.0);
            }

            if (no_sing && !ok)
            {
                // The sections are the placed sections of SectionPlacement: new curves the pipe owns.
                c2.Reverse();
            }
            seq2.Append(c2);
        }

        return no_sing;
    }

    /// <summary>An empty algorithm; use an Init method.</summary>
    public SurfacePipe()
    {
        _status = SurfacePipeError.PipeNotOk;
        _exchUV = false;
        _kPart = false;
        Init();
    }

    /// <summary>A pipe of constant radius Radius along Path.</summary>
    public SurfacePipe(Curve3D path, double radius)
        : this()
    {
        Init(path, radius);
    }

    /// <summary>A pipe of the constant section FirstSect along Path, with the trihedron Option.</summary>
    public SurfacePipe(Curve3D path, Curve3D firstSect, Trihedron option = Trihedron.IsCorrectedFrenet)
        : this()
    {
        Init(path, firstSect, option);
    }

    /// <summary>A pipe of the constant section FirstSect along the curve Path on Support (Darboux trihedron).</summary>
    public SurfacePipe(Curve2D path, Surface support, Curve3D firstSect)
        : this()
    {
        Init(path, support, firstSect);
    }

    /// <summary>A pipe of the constant section FirstSect along Path, with the fixed binormal Dir.</summary>
    public SurfacePipe(Curve3D path, Curve3D firstSect, in Direction3D dir)
    {
        _status = SurfacePipeError.PipeNotOk;
        _exchUV = false;
        _kPart = false;
        Init(path, firstSect, dir);
    }

    /// <summary>A pipe whose section evolves from FirstSect to LastSect along Path.</summary>
    public SurfacePipe(Curve3D path, Curve3D firstSect, Curve3D lastSect)
        : this()
    {
        Init(path, firstSect, lastSect);
    }

    /// <summary>A pipe through the sections NSections along Path.</summary>
    public SurfacePipe(Curve3D path, NCollection_Sequence<Curve3D> nSections)
        : this()
    {
        Init(path, nSections);
    }

    /// <summary>A pipe of constant radius Radius between the guide lines Curve1 and Curve2 around Path.</summary>
    public SurfacePipe(Curve3D path, Curve3D curve1, Curve3D curve2, double radius)
        : this()
    {
        CurveAdaptor adpPath = new CurveAdaptor(path);
        CurveAdaptor adpCurve1 = new CurveAdaptor(curve1);
        CurveAdaptor adpCurve2 = new CurveAdaptor(curve2);

        Init(adpPath, adpCurve1, adpCurve2, radius);
    }

    /// <summary>A pipe of constant radius Radius between the guide lines Curve1 and Curve2 around Path.</summary>
    public SurfacePipe(CurveAdaptorBase path, CurveAdaptorBase curve1, CurveAdaptorBase curve2, double radius)
        : this()
    {
        Init(path, curve1, curve2, radius);
    }

    /// <summary>
    /// A pipe of the constant section FirstSect along Path following the guide Guide: by curvilinear abscissa if
    /// ByACR, else by orthogonal planes; rotat lets the section rotate to follow the guide.
    /// </summary>
    public SurfacePipe(Curve3D path, CurveAdaptorBase guide, Curve3D firstSect, bool byACR, bool rotat)
        : this()
    // Path : trajectoire
    // Guide : courbe guide
    // FirstSect : section
    // NbPt : nb de points pour le calcul du triedre
    {
        Init(path, guide, firstSect, byACR, rotat);
    }

    public void Init(Curve3D path, CurveAdaptorBase guide, Curve3D firstSect, bool byACR, bool rotat)
    // Path : trajectoire
    // Guide : courbe guide
    // FirstSect : section
    // rotat : vrai si on veu la rotation false sinon
    // triedre : AC pour absc. curv. ou P pour plan ortho
    {
        double angle = 0;
        _adpPath = new CurveAdaptor((Curve3D)path.Copy());

        TrihedronWithGuide tLaw;

        // loi de triedre
        if (byACR)
        {
            tLaw = new GuideTrihedronAC(guide);
            tLaw.SetCurve(_adpPath);
        }
        else
        {
            tLaw = new GuideTrihedronPlan(guide);
            tLaw.SetCurve(_adpPath);
        }

        // loi de positionnement
        LocationGuide theLoc = new LocationGuide(tLaw);
        theLoc.SetCurve(_adpPath);

        SurfaceSectionPlacement place = new SurfaceSectionPlacement(theLoc, firstSect);
        place.Perform(global::Kerf.Engine.Foundation.Precision.Confusion());

        // loi de section
        _sec = new UniformSection(place.Section(false), _adpPath.FirstParameter(),
                                            _adpPath.LastParameter());

        if (rotat)
        {
            theLoc.Set(_sec, rotat, _adpPath.FirstParameter(), _adpPath.LastParameter(), 0.0, ref angle);
        }
        _loc = theLoc;
    }

    private void Init()
    {
        _type = 0;
        _error = 0;
        _radius = 0.0;
        _kPart = true;
        _polynomial = false;
        _adpPath = null;
        _adpFirstSect = null;
        _adpLastSect = null;
        _loc = null;
        _sec = null;
    }

    public void Init(Curve3D path, double radius)
    {
        // Ancienne methode
        _type = 1;
        _error = 0;
        _radius = radius;

        // Nouvelle methode
        _adpPath = new CurveAdaptor(path);
        CircleCurve3D c = new CircleCurve3D(Constants.XOY(), radius);
        c.Rotate(Constants.OZ(), M_PI / 2.0);

        _sec = new UniformSection(c, path.FirstParameter, path.LastParameter);
        CorrectedFrenet tLaw = new CorrectedFrenet();
        _loc = new CurveAndTrihedron(tLaw);
        _loc.SetCurve(_adpPath);
    }

    public void Init(Curve3D path, Curve3D firstSect, Trihedron option = Trihedron.IsCorrectedFrenet)
    {
        Curve3D sect;
        TrihedronLaw? tLaw = null;
        _adpPath = new CurveAdaptor((Curve3D)path.Copy());
        double param = path.FirstParameter;

        // Construction de la loi de triedre
        switch (option)
        {
            case Trihedron.IsCorrectedFrenet:
                {
                    tLaw = new CorrectedFrenet();
                    break;
                }

            case Trihedron.IsDarboux:
            case Trihedron.IsFrenet:
                {
                    tLaw = new Frenet();
                    break;
                }

            case Trihedron.IsFixed:
                {
                    double eps = 1.0e-9;
                    Vector3D v1 = new Vector3D(0, 0, 1), v2 = new Vector3D(0, 1, 0);
                    Direction3D d;
                    CLProps cp = new CLProps(path, param, 2, eps);
                    if (cp.IsTangentDefined())
                    {
                        cp.Tangent(out d);
                        v1 = v1.WithXYZ(d.XYZ());
                        v1 = v1.Normalized();
                        if (cp.Curvature() > eps)
                        {
                            cp.Normal(out d);
                            v2 = v2.WithXYZ(d.XYZ());
                            v2 = v2.Normalized();
                        }
                        else
                        {
                            Point3D p0 = new Point3D(0.0, 0.0, 0.0);
                            Frame3D axe = new Frame3D(p0, d);
                            d = axe.XDirection;
                            v2 = v2.WithXYZ(d.XYZ());
                            v2 = v2.Normalized();
                        }
                    }
                    tLaw = new Fixed(v1, v2);
                    break;
                }

            case Trihedron.IsConstantNormal:
                {
                    tLaw = new Frenet();
                    _loc = new CurveAndTrihedron(tLaw);
                    _loc.SetCurve(_adpPath);
                    SurfaceSectionPlacement place0 = new SurfaceSectionPlacement(_loc, firstSect);
                    place0.Perform(global::Kerf.Engine.Foundation.Precision.Confusion());
                    double ponsec = place0.ParameterOnSection();

                    double eps = 1.0e-9;
                    Vector3D v = new Vector3D(0, 1, 0);
                    Direction3D d;
                    CLProps cp = new CLProps(firstSect, ponsec, 2, eps);
                    if (cp.IsTangentDefined())
                    {
                        cp.Tangent(out d);
                        if (cp.Curvature() > eps)
                        {
                            cp.Normal(out d);
                            v = v.WithXYZ(d.XYZ());
                            v = v.Normalized();
                        }
                        else
                        {
                            Point3D p0 = new Point3D(0.0, 0.0, 0.0);
                            Frame3D axe = new Frame3D(p0, d);
                            d = axe.XDirection;
                            v = v.WithXYZ(d.XYZ());
                            v = v.Normalized();
                        }
                    }
                    tLaw = new ConstantBiNormal(new Direction3D(v));
                    break;
                }

            default:
                {
                    throw new ConstructionErrorException("GeomFill::Init : Unknown Option");
                }
        }

        if (tLaw is not null)
        {
            _loc = new CurveAndTrihedron(tLaw);
            _loc.SetCurve(_adpPath);
            SurfaceSectionPlacement place = new SurfaceSectionPlacement(_loc, firstSect);
            place.Perform(global::Kerf.Engine.Foundation.Precision.Confusion());
            param = place.ParameterOnPath();
            sect = place.Section(false);

            _sec = new UniformSection(sect, path.FirstParameter, path.LastParameter);
        }
    }

    //=======================================================================
    // function : Init
    // purpose  : sweep using Darboux's trihedron
    //=======================================================================

    public void Init(Curve2D path, Surface support, Curve3D firstSect)
    {
        Curve3D sect;
        TrihedronLaw tLaw = new Darboux();
        _adpPath = new CurveOnSurface(new CurveAdaptor2D(path), new SurfaceAdaptor(support));

        _loc = new CurveAndTrihedron(tLaw);
        _loc.SetCurve(_adpPath);
        SurfaceSectionPlacement place = new SurfaceSectionPlacement(_loc, firstSect);
        place.Perform(_adpPath, global::Kerf.Engine.Foundation.Precision.Confusion());
        sect = place.Section(false);

        _sec = new UniformSection(sect, _adpPath.FirstParameter(), _adpPath.LastParameter());
    }

    public void Init(Curve3D path, Curve3D firstSect, in Direction3D direction)
    {
        Init();

        Curve3D sect;
        _adpPath = new CurveAdaptor((Curve3D)path.Copy());
        Vector3D v = new Vector3D().WithXYZ(direction.XYZ());
        ConstantBiNormal tLaw = new ConstantBiNormal(new Direction3D(v));

        _loc = new CurveAndTrihedron(tLaw);
        _loc.SetCurve(_adpPath);
        SurfaceSectionPlacement place = new SurfaceSectionPlacement(_loc, firstSect);
        place.Perform(global::Kerf.Engine.Foundation.Precision.Confusion());
        sect = place.Section(false);

        _sec = new UniformSection(sect, path.FirstParameter, path.LastParameter);
    }

    public void Init(Curve3D path, NCollection_Sequence<Curve3D> nSections)
    {
        _type = 3;
        _error = 0;
        _radius = 0;

        TrihedronLaw? tLaw;
        tLaw = new CorrectedFrenet();
        _adpPath = new CurveAdaptor((Curve3D)path.Copy());
        if (tLaw is not null)
        {
            _loc = new CurveAndTrihedron(tLaw);
            _loc.SetCurve(_adpPath);
            NCollection_Sequence<Curve3D> seqC = new();
            NCollection_Sequence<double> seqP = new();
            seqC.Clear();
            seqP.Clear();
            int i;
            for (i = 1; i <= nSections.Length(); i++)
            {
                SurfaceSectionPlacement place = new SurfaceSectionPlacement(_loc, nSections[i]);
                place.Perform(global::Kerf.Engine.Foundation.Precision.Confusion());
                seqP.Append(place.ParameterOnPath());
                seqC.Append(place.Section(false));
            }

            // verification des orientations
            NCollection_Sequence<Curve3D> newSeq = new();
            if (CheckSense(seqC, newSeq))
            {
                seqC.Assign(newSeq);
            }

            // verification des parametres
            bool play_again = true;
            while (play_again)
            {
                play_again = false;
                for (i = 1; i <= nSections.Length(); i++)
                {
                    for (int j = i; j <= nSections.Length(); j++)
                    {
                        if (seqP.Value(i) > seqP.Value(j))
                        {
                            seqP.Exchange(i, j);
                            seqC.Exchange(i, j);
                            play_again = true;
                        }
                    }
                }
            }
            for (i = 1; i < nSections.Length(); i++)
            {
                if (Math.Abs(seqP.Value(i + 1) - seqP.Value(i)) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    throw new ConstructionErrorException("GeomFill_Pipe::Init with NSections : invalid parameters");
                }
            }

            // creation de la NSections
            double first = path.FirstParameter, last = path.LastParameter;
            double deb, fin;
            deb = seqC.First().FirstParameter;
            fin = seqC.First().LastParameter;
            _sec = new SurfaceNSections(seqC, seqP, deb, fin, first, last);
        }
    }

    public void Init(Curve3D path, Curve3D firstSect, Curve3D lastSect)
    {
        _type = 3;
        _error = 0;
        _radius = 0;
        double first = path.FirstParameter, last = path.LastParameter;
        TrihedronLaw? tLaw;
        tLaw = new CorrectedFrenet();
        _adpPath = new CurveAdaptor((Curve3D)path.Copy());

        if (tLaw is not null)
        {
            _loc = new CurveAndTrihedron(tLaw);

            // CorrectedFrenet's SetCurve returns whether the plain Frenet frame needs no correction, not whether it
            // failed: OCCT gave up on its false, so on most 3D paths (decision 1436)
            _loc.SetCurve(_adpPath);

            NCollection_Sequence<Curve3D> seqC = new();
            NCollection_Sequence<double> seqP = new();
            seqC.Clear();
            seqP.Clear();
            // sequence of sections
            SurfaceSectionPlacement pl1 = new SurfaceSectionPlacement(_loc, firstSect);
            pl1.Perform(first, global::Kerf.Engine.Foundation.Precision.Confusion());
            seqC.Append(pl1.Section(false));
            SurfaceSectionPlacement pl2 = new SurfaceSectionPlacement(_loc, lastSect);
            pl2.Perform(first, global::Kerf.Engine.Foundation.Precision.Confusion());
            seqC.Append(pl2.Section(false));
            // sequence of associated parameters
            seqP.Append(first);
            seqP.Append(last);

            // orientation verification
            NCollection_Sequence<Curve3D> newSeq = new();
            if (CheckSense(seqC, newSeq))
            {
                seqC.Assign(newSeq);
            }

            // creation of the NSections
            double deb, fin;
            deb = seqC.First().FirstParameter;
            fin = seqC.First().LastParameter;
            _sec = new SurfaceNSections(seqC, seqP, deb, fin, first, last);
        }
    }

    public void Init(CurveAdaptorBase path, CurveAdaptorBase curve1, CurveAdaptorBase curve2, double radius)
    {
        _type = 4;
        _error = 0;
        _radius = radius;
        _adpPath = path;
        _adpFirstSect = curve1;
        _adpLastSect = curve2;
    }

    /// <summary>
    /// Builds the pipe (the old method): the laws' sweep at 1e-4 if they are set, else the pipe on an edge, with its
    /// particular cases, by AppSweep. WithParameters makes the surface follow the path's parametrisation.
    /// </summary>
    public void Perform(bool withParameters = false, bool polynomial = false)
    {
        if ((_loc is not null) && (_sec is not null))
        {
            Perform(1.0e-4, polynomial);
            return;
        }

        _polynomial = polynomial;
        // on traite la cas tuyau sur arete Type = 4
        if (_polynomial)
        {
            ApproxSurf(withParameters);
            return;
        }
        if (!KPartT4())
        {
            ApproxSurf(withParameters);
        }
    }

    /// <summary>
    /// Builds the pipe: a particular case where one is detected, else an approximation to Tol with the continuity
    /// Conti, at most degree MaxDegree and NbMaxSegment spans.
    /// </summary>
    public void Perform(double tol, bool polynomial, Continuity conti = C1, int degMax = 11,
                        int nbMaxSegment = 30)
    {
        if (_status == SurfacePipeError.ImpossibleContact)
        {
            return;
        }

        Continuity theConti;
        switch (conti)
        {
            case C0:
                {
                    theConti = C0;
                }
                break;
            case G1:
            case C1:
                {
                    theConti = C1;
                }
                break;
            case G2:
            case C2:
                {
                    theConti = C2;
                }
                break;
            default:
                theConti = C2; // On ne sait pas faire mieux !
                break;
        }
        SweepFunction? func;
        func = null;

        if (_type == 4)
        {
            if (!KPartT4())
            {
                func = new CircularBlendFunc(_adpPath!, _adpFirstSect!, _adpLastSect!, _radius, polynomial);

                SweepApproximation app = new SweepApproximation(func);
                app.Perform(_adpPath!.FirstParameter(),
                            _adpPath.LastParameter(),
                            tol,
                            tol,
                            0.0,
                            0.01,
                            theConti,
                            degMax,
                            nbMaxSegment);
                if (app.IsDone)
                {
                    _surface = new BSplineSurface(app.SurfPoles(),
                                                        app.SurfWeights(),
                                                        app.SurfUKnots(),
                                                        app.SurfVKnots(),
                                                        app.SurfUMults(),
                                                        app.SurfVMults(),
                                                        app.UDegree(),
                                                        app.VDegree());
                    _error = app.MaxErrorOnSurf();
                    _status = SurfacePipeError.PipeOk;
                }
                // else {
                //   throw Standard_ConstructionError("GeomFill_Pipe::Perform : Cannot make a surface");
                // }
            }
        }
        else if ((_loc is not null) && (_sec is not null))
        {
            SurfaceSweep sweep = new SurfaceSweep(_loc, _kPart);
            sweep.SetTolerance(tol);
            sweep.Build(_sec, ApproxStyle.Location, theConti, degMax, nbMaxSegment);
            if (sweep.IsDone)
            {
                _surface = sweep.InternalSurface();
                _error = sweep.ErrorOnSurface();
                _status = SurfacePipeError.PipeOk;
            }
            // else {
            //   throw Standard_ConstructionError("GeomFill_Pipe::Perform : Cannot make a surface");
            // }
        }
        else
        {
            Perform(true, polynomial);
        }
    }

    //=======================================================================
    // function : KPartT4
    // purpose  : Pour gerer les cas particulier de type 4
    //=======================================================================
    private bool KPartT4()
    {
        bool ok = false;
        // -------    Cas du Cylindre  --------------------------
        if (_adpPath!.GetType() == Line && _adpFirstSect!.GetType() == Line
            && _adpLastSect!.GetType() == Line)
        {
            // try to generate a cylinder.
            Axis3D a0 = _adpPath.Line().Position;
            Axis3D a1 = _adpFirstSect.Line().Position;
            Axis3D a2 = _adpLastSect.Line().Position;
            // direction must be the same.
            Direction3D d0 = a0.Direction;
            Direction3D d1 = a1.Direction;
            Direction3D d2 = a2.Direction;
            if (!d0.IsEqual(d1, global::Kerf.Engine.Foundation.Precision.Angular()) || !d1.IsEqual(d2, global::Kerf.Engine.Foundation.Precision.Angular()))
            {
                return ok;
            }

            // the length of the line must be te same
            double l0 = _adpPath.LastParameter() - _adpPath.FirstParameter();
            double l1 = _adpFirstSect.LastParameter() - _adpFirstSect.FirstParameter();
            double l2 = _adpLastSect.LastParameter() - _adpLastSect.FirstParameter();
            if (Math.Abs(l1 - l0) > global::Kerf.Engine.Foundation.Precision.Confusion() || Math.Abs(l2 - l0) > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                return ok;
            }

            // the first points must be normal to the path.
            Point3D p0 = _adpPath.Value(_adpPath.FirstParameter());
            Point3D p1 = _adpFirstSect.Value(_adpFirstSect.FirstParameter());
            Point3D p2 = _adpLastSect.Value(_adpLastSect.FirstParameter());
            Direction3D v1 = new Direction3D(new Vector3D(p0, p1));
            Direction3D v2 = new Direction3D(new Vector3D(p0, p2));
            if (Math.Abs(v1.Dot(d0)) > global::Kerf.Engine.Foundation.Precision.Confusion() || Math.Abs(v2.Dot(d0)) > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                return ok;
            }

            // the result is a cylindrical surface.
            Direction3D x = v1, y = v2, zRef;
            zRef = x.Crossed(y);

            CoordinateSystem axis = new CoordinateSystem(a0.Location, d0, x);
            if (zRef.Dot(d0) < 0.0)
            {
                axis = axis.YReversed();
            }

            // rotate the surface to set the iso U = 0 not in the result.
            axis = axis.Rotated(new Axis3D(p0, zRef), -M_PI / 2.0);

            _surface = new CylindricalSurface(axis, _radius);
            double alpha = v1.AngleWithRef(v2, zRef);
            _surface = new TrimmedSurface(_surface,
                                                           M_PI / 2.0,
                                                           M_PI / 2.0 + alpha,
                                                           _adpPath.FirstParameter(),
                                                           _adpPath.LastParameter());
            ok = true; // C'est bien un cylindre
            _status = SurfacePipeError.PipeOk;
        }
        // -----------    Cas du tore  ----------------------------------
        else if (_adpPath.GetType() == Circle && _adpFirstSect!.GetType() == Circle
                 && _adpLastSect!.GetType() == Circle)
        {
            // try to generate a toroidal surface.
            // les 3 cercles doivent avoir meme angle d'ouverture
            double alp0 = _adpPath.FirstParameter() - _adpPath.LastParameter();
            double alp1 = _adpFirstSect.FirstParameter() - _adpFirstSect.LastParameter();
            double alp2 = _adpLastSect.FirstParameter() - _adpLastSect.LastParameter();

            if (Math.Abs(alp0 - alp1) > global::Kerf.Engine.Foundation.Precision.Angular() || Math.Abs(alp0 - alp2) > global::Kerf.Engine.Foundation.Precision.Angular())
            {
                return ok;
            }

            Frame3D a0 = _adpPath.Circle().Position;
            Frame3D a1 = _adpFirstSect.Circle().Position;
            Frame3D a2 = _adpLastSect.Circle().Position;
            Direction3D d0 = a0.Direction;
            Direction3D d1 = a1.Direction;
            Direction3D d2 = a2.Direction;
            Point3D p0 = _adpPath.Value(_adpPath.FirstParameter());
            Point3D p1 = _adpFirstSect.Value(_adpFirstSect.FirstParameter());
            Point3D p2 = _adpLastSect.Value(_adpLastSect.FirstParameter());

            // les 3 directions doivent etre egales.
            if (!d0.IsEqual(d1, global::Kerf.Engine.Foundation.Precision.Angular()) || !d1.IsEqual(d2, global::Kerf.Engine.Foundation.Precision.Angular()))
            {
                return ok;
            }

            // les 3 ax1 doivent etre confondus.
            Line3D l = new Line3D(a0.Axis);
            if (!l.Contains(a1.Location, global::Kerf.Engine.Foundation.Precision.Confusion()) || !l.Contains(a2.Location, global::Kerf.Engine.Foundation.Precision.Confusion()))
            {
                return ok;
            }

            // les 3 premiers points doivent etre dans la meme section.
            Direction3D v1 = new Direction3D(new Vector3D(p0, p1));
            Direction3D v2 = new Direction3D(new Vector3D(p0, p2));
            Circle3D ci = _adpPath.Circle();
            Vector3D yRef = global::Kerf.Engine.Geometry.ElCLib.CircleDN(_adpPath.FirstParameter(), a0, ci.Radius, 1);
            if (Math.Abs(v1.Dot(new Direction3D(yRef))) > global::Kerf.Engine.Foundation.Precision.Confusion()
                || Math.Abs(v2.Dot(new Direction3D(yRef))) > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                return ok;
            }

            // OK it`s a Toroidal Surface !!  OUF !!
            Torus3D t = new Torus3D(new CoordinateSystem(a0), ci.Radius, _radius);
            Vector3D xRef = new Vector3D(a0.Location, p0);
            // au maximum on fait un tore d`ouverture en V = PI
            double vv1 = v1.AngleWithRef(new Direction3D(xRef), new Direction3D(yRef));
            double vv2 = v2.AngleWithRef(new Direction3D(xRef), new Direction3D(yRef));
            double deltaV = v2.AngleWithRef(v1, new Direction3D(yRef));
            if (deltaV < 0.0)
            {
                t = t.VReversed();
                vv1 = -vv1;
                vv2 = 2 * M_PI + vv1 - deltaV;
            }
            _surface = new TrimmedSurface(new ToroidalSurface(t),
                                                           _adpPath.FirstParameter(),
                                                           _adpPath.LastParameter(),
                                                           vv1,
                                                           vv2);
            _exchUV = true;
            ok = true;
            _status = SurfacePipeError.PipeOk;
        }

        return ok;
    }

    private void ApproxSurf(bool withParameters)
    {
        // Traitment of general case.
        // generate a sequence of the section by <SweepSectionGenerator>
        // and approximate this sequence.

        if (_type != 4)
        {
            throw new ConstructionErrorException("GeomFill_Pipe");
        }
        SweepSectionGenerator section = new SweepSectionGenerator(_adpPath!, _adpFirstSect!,
                                                                                    _adpLastSect!, _radius);

        section.Perform(_polynomial);

        SurfaceLine line = new SurfaceLine(section.SectionCount);
        int nbIt = 0;
        double t3d = global::Kerf.Engine.Foundation.Precision.Approximation();
        double t2d = global::Kerf.Engine.Foundation.Precision.PApproximation();
        AppSweep app = new AppSweep(4, 8, t3d, t2d, nbIt, withParameters);

        app.Perform(line, section, 30);

        if (!app.IsDone)
        {
            // throw StdFail_NotDone("Pipe : App not done");
        }
        else
        {
            app.SurfShape(out _, out _, out _, out _, out _, out _);

            _surface = new BSplineSurface(app.SurfPoles(),
                                                app.SurfWeights(),
                                                app.SurfUKnots(),
                                                app.SurfVKnots(),
                                                app.SurfUMults(),
                                                app.SurfVMults(),
                                                app.UDegree(),
                                                app.VDegree());
            app.TolReached(out _error, out _);
            _status = SurfacePipeError.PipeOk;
        }
    }

    /// <summary>The surface built (frozen); null before it is built.</summary>
    public Surface? Surface()
    {
        _surface?.Freeze();
        return _surface;
    }

    /// <summary>The surface built, not frozen, for ported callers.</summary>
    internal Surface? InternalSurface() => _surface;

    /// <summary>Whether the surface is parametrised with u along the path (some elementary surfaces).</summary>
    public bool ExchangeUV() => _exchUV;

    /// <summary>Sets whether particular cases (planes, cylinders, ...) are built where possible.</summary>
    public void GenerateParticularCase(bool b)
    {
        _kPart = b;
    }

    /// <summary>Whether particular cases are built where possible.</summary>
    public bool GenerateParticularCase() => _kPart;

    /// <summary>The approximation error (0 for an elementary surface).</summary>
    public double ErrorOnSurf() => _error;

    /// <summary>Whether a surface was built.</summary>
    public bool IsDone => _status == SurfacePipeError.PipeOk;

    /// <summary>The execution status.</summary>
    public SurfacePipeError GetStatus() => _status;
}
