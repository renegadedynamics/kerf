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
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Occt.GeomFill_ApproxStyle;
using static Occt.GeomFill_PipeError;
using static Occt.GeomFill_Trihedron;

namespace Occt;

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
/// (where OCCT copies them) and on sections placed by <see cref="GeomFill_SectionPlacement"/> (new geometry).
/// <see cref="Surface"/> hands out the surface frozen.
/// </remarks>
public sealed class GeomFill_Pipe
{
    private GeomFill_PipeError myStatus; //!< Execution status
    private double myRadius;
    private double myError;
    private CurveAdaptorBase? myAdpPath;
    private CurveAdaptorBase? myAdpFirstSect;
    private CurveAdaptorBase? myAdpLastSect;
    private Surface? mySurface;
    private GeomFill_LocationLaw? myLoc;
    private GeomFill_SectionLaw? mySec;
    private int myType;
    private bool myExchUV;
    private bool myKPart;
    private bool myPolynomial;

    private static bool CheckSense(NCollection_Sequence<Curve3D> Seq1, NCollection_Sequence<Curve3D> Seq2)
    {
        // initialisation
        bool no_sing = true;
        Seq2.Clear();

        Curve3D C1 = Seq1.Value(1);
        double f = C1.FirstParameter(), l = C1.LastParameter();
        int iP, NP = 21;
        NCollection_Array1<Point3D> Tab = new(1, NP);
        double u = f, h = Math.Abs(f - l) / 20.0;
        for (iP = 1; iP <= NP; iP++)
        {
            C1.D0(u, out Tab[iP]);
            u += h;
            if ((u - f) * (u - l) > 0.0)
            {
                u = l;
            }
        }
        Frame3D AxeRef, Axe;
        Point3D Pos;
        bool sing;
        global::Kerf.Engine.Geometry.Library.GeomLib.AxeOfInertia(Tab, out AxeRef, out sing);

        // si la section est une droite, ca ne marche pas
        if (sing)
        {
            no_sing = false;
        }

        Pos = AxeRef.Location();
        double alpha1, alpha2, alpha3;
        Point3D P1, P2;
        u = (f + l - h) / 2 - h;
        C1.D0(u, out P1);
        u += h;
        C1.D0(u, out P2);
        alpha1 = new Vector3D(Pos, P1).AngleWithRef(new Vector3D(Pos, P2), new Vector3D(AxeRef.Direction()));
        P1 = P2;
        u += h;
        C1.D0(u, out P2);
        alpha2 = new Vector3D(Pos, P1).AngleWithRef(new Vector3D(Pos, P2), new Vector3D(AxeRef.Direction()));
        P1 = P2;
        u += h;
        C1.D0(u, out P2);
        alpha3 = new Vector3D(Pos, P1).AngleWithRef(new Vector3D(Pos, P2), new Vector3D(AxeRef.Direction()));
        Seq2.Append(C1);

        for (int iseq = 2; iseq <= Seq1.Length(); iseq++)
        {
            // discretisation de C2
            Curve3D C2 = Seq1.Value(iseq);
            f = C2.FirstParameter();
            l = C2.LastParameter();
            u = f;
            for (iP = 1; iP <= NP; iP++)
            {
                C2.D0(u, out Tab[iP]);
                u += h;
                if ((u - f) * (u - l) > 0.0)
                {
                    u = l;
                }
            }
            global::Kerf.Engine.Geometry.Library.GeomLib.AxeOfInertia(Tab, out Axe, out sing);

            // si la section est une droite, ca ne marche pas
            if (sing)
            {
                no_sing = false;
            }

            Pos = Axe.Location();
            double beta1, beta2, beta3;
            u = (f + l - h) / 2 - h;
            C2.D0(u, out P1);
            u += h;
            C2.D0(u, out P2);
            beta1 = new Vector3D(Pos, P1).AngleWithRef(new Vector3D(Pos, P2), new Vector3D(AxeRef.Direction()));
            P1 = P2;
            u += h;
            C2.D0(u, out P2);
            beta2 = new Vector3D(Pos, P1).AngleWithRef(new Vector3D(Pos, P2), new Vector3D(AxeRef.Direction()));
            P1 = P2;
            u += h;
            C2.D0(u, out P2);
            beta3 = new Vector3D(Pos, P1).AngleWithRef(new Vector3D(Pos, P2), new Vector3D(AxeRef.Direction()));

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
                C2.Reverse();
            }
            Seq2.Append(C2);
        }

        return no_sing;
    }

    /// <summary>An empty algorithm; use an Init method.</summary>
    public GeomFill_Pipe()
    {
        myStatus = GeomFill_PipeNotOk;
        myExchUV = false;
        myKPart = false;
        Init();
    }

    /// <summary>A pipe of constant radius Radius along Path.</summary>
    public GeomFill_Pipe(Curve3D Path, double Radius)
        : this()
    {
        Init(Path, Radius);
    }

    /// <summary>A pipe of the constant section FirstSect along Path, with the trihedron Option.</summary>
    public GeomFill_Pipe(Curve3D Path, Curve3D FirstSect, GeomFill_Trihedron Option = GeomFill_IsCorrectedFrenet)
        : this()
    {
        Init(Path, FirstSect, Option);
    }

    /// <summary>A pipe of the constant section FirstSect along the curve Path on Support (Darboux trihedron).</summary>
    public GeomFill_Pipe(Curve2D Path, Surface Support, Curve3D FirstSect)
        : this()
    {
        Init(Path, Support, FirstSect);
    }

    /// <summary>A pipe of the constant section FirstSect along Path, with the fixed binormal Dir.</summary>
    public GeomFill_Pipe(Curve3D Path, Curve3D FirstSect, in Direction3D Dir)
    {
        myStatus = GeomFill_PipeNotOk;
        myExchUV = false;
        myKPart = false;
        Init(Path, FirstSect, Dir);
    }

    /// <summary>A pipe whose section evolves from FirstSect to LastSect along Path.</summary>
    public GeomFill_Pipe(Curve3D Path, Curve3D FirstSect, Curve3D LastSect)
        : this()
    {
        Init(Path, FirstSect, LastSect);
    }

    /// <summary>A pipe through the sections NSections along Path.</summary>
    public GeomFill_Pipe(Curve3D Path, NCollection_Sequence<Curve3D> NSections)
        : this()
    {
        Init(Path, NSections);
    }

    /// <summary>A pipe of constant radius Radius between the guide lines Curve1 and Curve2 around Path.</summary>
    public GeomFill_Pipe(Curve3D Path, Curve3D Curve1, Curve3D Curve2, double Radius)
        : this()
    {
        CurveAdaptor AdpPath = new CurveAdaptor(Path);
        CurveAdaptor AdpCurve1 = new CurveAdaptor(Curve1);
        CurveAdaptor AdpCurve2 = new CurveAdaptor(Curve2);

        Init(AdpPath, AdpCurve1, AdpCurve2, Radius);
    }

    /// <summary>A pipe of constant radius Radius between the guide lines Curve1 and Curve2 around Path.</summary>
    public GeomFill_Pipe(CurveAdaptorBase Path, CurveAdaptorBase Curve1, CurveAdaptorBase Curve2, double Radius)
        : this()
    {
        Init(Path, Curve1, Curve2, Radius);
    }

    /// <summary>
    /// A pipe of the constant section FirstSect along Path following the guide Guide: by curvilinear abscissa if
    /// ByACR, else by orthogonal planes; rotat lets the section rotate to follow the guide.
    /// </summary>
    public GeomFill_Pipe(Curve3D Path, CurveAdaptorBase Guide, Curve3D FirstSect, bool ByACR, bool rotat)
        : this()
    // Path : trajectoire
    // Guide : courbe guide
    // FirstSect : section
    // NbPt : nb de points pour le calcul du triedre
    {
        Init(Path, Guide, FirstSect, ByACR, rotat);
    }

    public void Init(Curve3D Path, CurveAdaptorBase Guide, Curve3D FirstSect, bool byACR, bool rotat)
    // Path : trajectoire
    // Guide : courbe guide
    // FirstSect : section
    // rotat : vrai si on veu la rotation false sinon
    // triedre : AC pour absc. curv. ou P pour plan ortho
    {
        double angle = 0;
        myAdpPath = new CurveAdaptor((Curve3D)Path.Copy());

        GeomFill_TrihedronWithGuide TLaw;

        // loi de triedre
        if (byACR)
        {
            TLaw = new GeomFill_GuideTrihedronAC(Guide);
            TLaw.SetCurve(myAdpPath);
        }
        else
        {
            TLaw = new GeomFill_GuideTrihedronPlan(Guide);
            TLaw.SetCurve(myAdpPath);
        }

        // loi de positionnement
        GeomFill_LocationGuide TheLoc = new GeomFill_LocationGuide(TLaw);
        TheLoc.SetCurve(myAdpPath);

        GeomFill_SectionPlacement Place = new GeomFill_SectionPlacement(TheLoc, FirstSect);
        Place.Perform(global::Kerf.Engine.Foundation.Precision.Confusion());

        // loi de section
        mySec = new GeomFill_UniformSection(Place.Section(false), myAdpPath.FirstParameter(),
                                            myAdpPath.LastParameter());

        if (rotat)
        {
            TheLoc.Set(mySec, rotat, myAdpPath.FirstParameter(), myAdpPath.LastParameter(), 0.0, ref angle);
        }
        myLoc = TheLoc;
    }

    private void Init()
    {
        myType = 0;
        myError = 0;
        myRadius = 0.0;
        myKPart = true;
        myPolynomial = false;
        myAdpPath = null;
        myAdpFirstSect = null;
        myAdpLastSect = null;
        myLoc = null;
        mySec = null;
    }

    public void Init(Curve3D Path, double Radius)
    {
        // Ancienne methode
        myType = 1;
        myError = 0;
        myRadius = Radius;

        // Nouvelle methode
        myAdpPath = new CurveAdaptor(Path);
        CircleCurve3D C = new CircleCurve3D(Constants.XOY(), Radius);
        C.Rotate(Constants.OZ(), M_PI / 2.0);

        mySec = new GeomFill_UniformSection(C, Path.FirstParameter(), Path.LastParameter());
        GeomFill_CorrectedFrenet TLaw = new GeomFill_CorrectedFrenet();
        myLoc = new GeomFill_CurveAndTrihedron(TLaw);
        myLoc.SetCurve(myAdpPath);
    }

    public void Init(Curve3D Path, Curve3D FirstSect, GeomFill_Trihedron Option = GeomFill_IsCorrectedFrenet)
    {
        Curve3D Sect;
        GeomFill_TrihedronLaw? TLaw = null;
        myAdpPath = new CurveAdaptor((Curve3D)Path.Copy());
        double param = Path.FirstParameter();

        // Construction de la loi de triedre
        switch (Option)
        {
            case GeomFill_IsCorrectedFrenet:
                {
                    TLaw = new GeomFill_CorrectedFrenet();
                    break;
                }

            case GeomFill_IsDarboux:
            case GeomFill_IsFrenet:
                {
                    TLaw = new GeomFill_Frenet();
                    break;
                }

            case GeomFill_IsFixed:
                {
                    double Eps = 1.0e-9;
                    Vector3D V1 = new Vector3D(0, 0, 1), V2 = new Vector3D(0, 1, 0);
                    Direction3D D;
                    CLProps CP = new CLProps(Path, param, 2, Eps);
                    if (CP.IsTangentDefined())
                    {
                        CP.Tangent(out D);
                        V1.SetXYZ(D.XYZ());
                        V1.Normalize();
                        if (CP.Curvature() > Eps)
                        {
                            CP.Normal(out D);
                            V2.SetXYZ(D.XYZ());
                            V2.Normalize();
                        }
                        else
                        {
                            Point3D P0 = new Point3D(0.0, 0.0, 0.0);
                            Frame3D Axe = new Frame3D(P0, D);
                            D = Axe.XDirection();
                            V2.SetXYZ(D.XYZ());
                            V2.Normalize();
                        }
                    }
                    TLaw = new GeomFill_Fixed(V1, V2);
                    break;
                }

            case GeomFill_IsConstantNormal:
                {
                    TLaw = new GeomFill_Frenet();
                    myLoc = new GeomFill_CurveAndTrihedron(TLaw);
                    myLoc.SetCurve(myAdpPath);
                    GeomFill_SectionPlacement Place0 = new GeomFill_SectionPlacement(myLoc, FirstSect);
                    Place0.Perform(global::Kerf.Engine.Foundation.Precision.Confusion());
                    double ponsec = Place0.ParameterOnSection();

                    double Eps = 1.0e-9;
                    Vector3D V = new Vector3D(0, 1, 0);
                    Direction3D D;
                    CLProps CP = new CLProps(FirstSect, ponsec, 2, Eps);
                    if (CP.IsTangentDefined())
                    {
                        CP.Tangent(out D);
                        if (CP.Curvature() > Eps)
                        {
                            CP.Normal(out D);
                            V.SetXYZ(D.XYZ());
                            V.Normalize();
                        }
                        else
                        {
                            Point3D P0 = new Point3D(0.0, 0.0, 0.0);
                            Frame3D Axe = new Frame3D(P0, D);
                            D = Axe.XDirection();
                            V.SetXYZ(D.XYZ());
                            V.Normalize();
                        }
                    }
                    TLaw = new GeomFill_ConstantBiNormal(new Direction3D(V));
                    break;
                }

            default:
                {
                    throw new ConstructionErrorException("GeomFill::Init : Unknown Option");
                }
        }

        if (TLaw is not null)
        {
            myLoc = new GeomFill_CurveAndTrihedron(TLaw);
            myLoc.SetCurve(myAdpPath);
            GeomFill_SectionPlacement Place = new GeomFill_SectionPlacement(myLoc, FirstSect);
            Place.Perform(global::Kerf.Engine.Foundation.Precision.Confusion());
            param = Place.ParameterOnPath();
            Sect = Place.Section(false);

            mySec = new GeomFill_UniformSection(Sect, Path.FirstParameter(), Path.LastParameter());
        }
    }

    //=======================================================================
    // function : Init
    // purpose  : sweep using Darboux's trihedron
    //=======================================================================

    public void Init(Curve2D Path, Surface Support, Curve3D FirstSect)
    {
        Curve3D Sect;
        GeomFill_TrihedronLaw TLaw = new GeomFill_Darboux();
        myAdpPath = new CurveOnSurface(new CurveAdaptor2D(Path), new SurfaceAdaptor(Support));

        myLoc = new GeomFill_CurveAndTrihedron(TLaw);
        myLoc.SetCurve(myAdpPath);
        GeomFill_SectionPlacement Place = new GeomFill_SectionPlacement(myLoc, FirstSect);
        Place.Perform(myAdpPath, global::Kerf.Engine.Foundation.Precision.Confusion());
        Sect = Place.Section(false);

        mySec = new GeomFill_UniformSection(Sect, myAdpPath.FirstParameter(), myAdpPath.LastParameter());
    }

    public void Init(Curve3D Path, Curve3D FirstSect, in Direction3D Direction)
    {
        Init();

        Curve3D Sect;
        myAdpPath = new CurveAdaptor((Curve3D)Path.Copy());
        Vector3D V = new Vector3D();
        V.SetXYZ(Direction.XYZ());
        GeomFill_ConstantBiNormal TLaw = new GeomFill_ConstantBiNormal(new Direction3D(V));

        myLoc = new GeomFill_CurveAndTrihedron(TLaw);
        myLoc.SetCurve(myAdpPath);
        GeomFill_SectionPlacement Place = new GeomFill_SectionPlacement(myLoc, FirstSect);
        Place.Perform(global::Kerf.Engine.Foundation.Precision.Confusion());
        Sect = Place.Section(false);

        mySec = new GeomFill_UniformSection(Sect, Path.FirstParameter(), Path.LastParameter());
    }

    public void Init(Curve3D Path, NCollection_Sequence<Curve3D> NSections)
    {
        myType = 3;
        myError = 0;
        myRadius = 0;

        GeomFill_TrihedronLaw? TLaw;
        TLaw = new GeomFill_CorrectedFrenet();
        myAdpPath = new CurveAdaptor((Curve3D)Path.Copy());
        if (TLaw is not null)
        {
            myLoc = new GeomFill_CurveAndTrihedron(TLaw);
            myLoc.SetCurve(myAdpPath);
            NCollection_Sequence<Curve3D> SeqC = new();
            NCollection_Sequence<double> SeqP = new();
            SeqC.Clear();
            SeqP.Clear();
            int i;
            for (i = 1; i <= NSections.Length(); i++)
            {
                GeomFill_SectionPlacement Place = new GeomFill_SectionPlacement(myLoc, NSections[i]);
                Place.Perform(global::Kerf.Engine.Foundation.Precision.Confusion());
                SeqP.Append(Place.ParameterOnPath());
                SeqC.Append(Place.Section(false));
            }

            // verification des orientations
            NCollection_Sequence<Curve3D> NewSeq = new();
            if (CheckSense(SeqC, NewSeq))
            {
                SeqC.Assign(NewSeq);
            }

            // verification des parametres
            bool play_again = true;
            while (play_again)
            {
                play_again = false;
                for (i = 1; i <= NSections.Length(); i++)
                {
                    for (int j = i; j <= NSections.Length(); j++)
                    {
                        if (SeqP.Value(i) > SeqP.Value(j))
                        {
                            SeqP.Exchange(i, j);
                            SeqC.Exchange(i, j);
                            play_again = true;
                        }
                    }
                }
            }
            for (i = 1; i < NSections.Length(); i++)
            {
                if (Math.Abs(SeqP.Value(i + 1) - SeqP.Value(i)) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    throw new ConstructionErrorException("GeomFill_Pipe::Init with NSections : invalid parameters");
                }
            }

            // creation de la NSections
            double first = Path.FirstParameter(), last = Path.LastParameter();
            double deb, fin;
            deb = SeqC.First().FirstParameter();
            fin = SeqC.First().LastParameter();
            mySec = new GeomFill_NSections(SeqC, SeqP, deb, fin, first, last);
        }
    }

    public void Init(Curve3D Path, Curve3D FirstSect, Curve3D LastSect)
    {
        myType = 3;
        myError = 0;
        myRadius = 0;
        double first = Path.FirstParameter(), last = Path.LastParameter();
        GeomFill_TrihedronLaw? TLaw;
        TLaw = new GeomFill_CorrectedFrenet();
        myAdpPath = new CurveAdaptor((Curve3D)Path.Copy());

        if (TLaw is not null)
        {
            myLoc = new GeomFill_CurveAndTrihedron(TLaw);

            // OCCT: CorrectedFrenet's SetCurve returns whether the plain Frenet frame needs no correction, not whether
            // it failed, so most 3D paths end here (occt-issues.md).
            if (!(myLoc.SetCurve(myAdpPath)))
            {
                myStatus = GeomFill_ImpossibleContact;
                return;
            }

            NCollection_Sequence<Curve3D> SeqC = new();
            NCollection_Sequence<double> SeqP = new();
            SeqC.Clear();
            SeqP.Clear();
            // sequence of sections
            GeomFill_SectionPlacement Pl1 = new GeomFill_SectionPlacement(myLoc, FirstSect);
            Pl1.Perform(first, global::Kerf.Engine.Foundation.Precision.Confusion());
            SeqC.Append(Pl1.Section(false));
            GeomFill_SectionPlacement Pl2 = new GeomFill_SectionPlacement(myLoc, LastSect);
            Pl2.Perform(first, global::Kerf.Engine.Foundation.Precision.Confusion());
            SeqC.Append(Pl2.Section(false));
            // sequence of associated parameters
            SeqP.Append(first);
            SeqP.Append(last);

            // orientation verification
            NCollection_Sequence<Curve3D> NewSeq = new();
            if (CheckSense(SeqC, NewSeq))
            {
                SeqC.Assign(NewSeq);
            }

            // creation of the NSections
            double deb, fin;
            deb = SeqC.First().FirstParameter();
            fin = SeqC.First().LastParameter();
            mySec = new GeomFill_NSections(SeqC, SeqP, deb, fin, first, last);
        }
    }

    public void Init(CurveAdaptorBase Path, CurveAdaptorBase Curve1, CurveAdaptorBase Curve2, double Radius)
    {
        myType = 4;
        myError = 0;
        myRadius = Radius;
        myAdpPath = Path;
        myAdpFirstSect = Curve1;
        myAdpLastSect = Curve2;
    }

    /// <summary>
    /// Builds the pipe (the old method): the laws' sweep at 1e-4 if they are set, else the pipe on an edge, with its
    /// particular cases, by AppSweep. WithParameters makes the surface follow the path's parametrisation.
    /// </summary>
    public void Perform(bool WithParameters = false, bool Polynomial = false)
    {
        if ((myLoc is not null) && (mySec is not null))
        {
            Perform(1.0e-4, Polynomial);
            return;
        }

        myPolynomial = Polynomial;
        // on traite la cas tuyau sur arete Type = 4
        if (myPolynomial)
        {
            ApproxSurf(WithParameters);
            return;
        }
        if (!KPartT4())
        {
            ApproxSurf(WithParameters);
        }
    }

    /// <summary>
    /// Builds the pipe: a particular case where one is detected, else an approximation to Tol with the continuity
    /// Conti, at most degree MaxDegree and NbMaxSegment spans.
    /// </summary>
    public void Perform(double Tol, bool Polynomial, Continuity Conti = C1, int DegMax = 11,
                        int NbMaxSegment = 30)
    {
        if (myStatus == GeomFill_ImpossibleContact)
        {
            return;
        }

        Continuity TheConti;
        switch (Conti)
        {
            case C0:
                {
                    TheConti = C0;
                }
                break;
            case G1:
            case C1:
                {
                    TheConti = C1;
                }
                break;
            case G2:
            case C2:
                {
                    TheConti = C2;
                }
                break;
            default:
                TheConti = C2; // On ne sait pas faire mieux !
                break;
        }
        SweepFunction? Func;
        Func = null;

        if (myType == 4)
        {
            if (!KPartT4())
            {
                Func = new GeomFill_CircularBlendFunc(myAdpPath!, myAdpFirstSect!, myAdpLastSect!, myRadius, Polynomial);

                SweepApproximation App = new SweepApproximation(Func);
                App.Perform(myAdpPath!.FirstParameter(),
                            myAdpPath.LastParameter(),
                            Tol,
                            Tol,
                            0.0,
                            0.01,
                            TheConti,
                            DegMax,
                            NbMaxSegment);
                if (App.IsDone)
                {
                    mySurface = new BSplineSurface(App.SurfPoles(),
                                                        App.SurfWeights(),
                                                        App.SurfUKnots(),
                                                        App.SurfVKnots(),
                                                        App.SurfUMults(),
                                                        App.SurfVMults(),
                                                        App.UDegree(),
                                                        App.VDegree());
                    myError = App.MaxErrorOnSurf();
                    myStatus = GeomFill_PipeOk;
                }
                // else {
                //   throw Standard_ConstructionError("GeomFill_Pipe::Perform : Cannot make a surface");
                // }
            }
        }
        else if ((myLoc is not null) && (mySec is not null))
        {
            GeomFill_Sweep Sweep = new GeomFill_Sweep(myLoc, myKPart);
            Sweep.SetTolerance(Tol);
            Sweep.Build(mySec, GeomFill_Location, TheConti, DegMax, NbMaxSegment);
            if (Sweep.IsDone())
            {
                mySurface = Sweep.InternalSurface();
                myError = Sweep.ErrorOnSurface();
                myStatus = GeomFill_PipeOk;
            }
            // else {
            //   throw Standard_ConstructionError("GeomFill_Pipe::Perform : Cannot make a surface");
            // }
        }
        else
        {
            Perform(true, Polynomial);
        }
    }

    //=======================================================================
    // function : KPartT4
    // purpose  : Pour gerer les cas particulier de type 4
    //=======================================================================
    private bool KPartT4()
    {
        bool Ok = false;
        // -------    Cas du Cylindre  --------------------------
        if (myAdpPath!.GetType() == Line && myAdpFirstSect!.GetType() == Line
            && myAdpLastSect!.GetType() == Line)
        {
            // try to generate a cylinder.
            Axis3D A0 = myAdpPath.Line().Position();
            Axis3D A1 = myAdpFirstSect.Line().Position();
            Axis3D A2 = myAdpLastSect.Line().Position();
            // direction must be the same.
            Direction3D D0 = A0.Direction();
            Direction3D D1 = A1.Direction();
            Direction3D D2 = A2.Direction();
            if (!D0.IsEqual(D1, global::Kerf.Engine.Foundation.Precision.Angular()) || !D1.IsEqual(D2, global::Kerf.Engine.Foundation.Precision.Angular()))
            {
                return Ok;
            }

            // the length of the line must be te same
            double L0 = myAdpPath.LastParameter() - myAdpPath.FirstParameter();
            double L1 = myAdpFirstSect.LastParameter() - myAdpFirstSect.FirstParameter();
            double L2 = myAdpLastSect.LastParameter() - myAdpLastSect.FirstParameter();
            if (Math.Abs(L1 - L0) > global::Kerf.Engine.Foundation.Precision.Confusion() || Math.Abs(L2 - L0) > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                return Ok;
            }

            // the first points must be normal to the path.
            Point3D P0 = myAdpPath.Value(myAdpPath.FirstParameter());
            Point3D P1 = myAdpFirstSect.Value(myAdpFirstSect.FirstParameter());
            Point3D P2 = myAdpLastSect.Value(myAdpLastSect.FirstParameter());
            Direction3D V1 = new Direction3D(new Vector3D(P0, P1));
            Direction3D V2 = new Direction3D(new Vector3D(P0, P2));
            if (Math.Abs(V1.Dot(D0)) > global::Kerf.Engine.Foundation.Precision.Confusion() || Math.Abs(V2.Dot(D0)) > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                return Ok;
            }

            // the result is a cylindrical surface.
            Direction3D X = V1, Y = V2, ZRef;
            ZRef = X.Crossed(Y);

            CoordinateSystem Axis = new CoordinateSystem(A0.Location(), D0, X);
            if (ZRef.Dot(D0) < 0.0)
            {
                Axis.YReverse();
            }

            // rotate the surface to set the iso U = 0 not in the result.
            Axis.Rotate(new Axis3D(P0, ZRef), -M_PI / 2.0);

            mySurface = new CylindricalSurface(Axis, myRadius);
            double Alpha = V1.AngleWithRef(V2, ZRef);
            mySurface = new TrimmedSurface(mySurface,
                                                           M_PI / 2.0,
                                                           M_PI / 2.0 + Alpha,
                                                           myAdpPath.FirstParameter(),
                                                           myAdpPath.LastParameter());
            Ok = true; // C'est bien un cylindre
            myStatus = GeomFill_PipeOk;
        }
        // -----------    Cas du tore  ----------------------------------
        else if (myAdpPath.GetType() == Circle && myAdpFirstSect!.GetType() == Circle
                 && myAdpLastSect!.GetType() == Circle)
        {
            // try to generate a toroidal surface.
            // les 3 cercles doivent avoir meme angle d'ouverture
            double Alp0 = myAdpPath.FirstParameter() - myAdpPath.LastParameter();
            double Alp1 = myAdpFirstSect.FirstParameter() - myAdpFirstSect.LastParameter();
            double Alp2 = myAdpLastSect.FirstParameter() - myAdpLastSect.LastParameter();

            if (Math.Abs(Alp0 - Alp1) > global::Kerf.Engine.Foundation.Precision.Angular() || Math.Abs(Alp0 - Alp2) > global::Kerf.Engine.Foundation.Precision.Angular())
            {
                return Ok;
            }

            Frame3D A0 = myAdpPath.Circle().Position();
            Frame3D A1 = myAdpFirstSect.Circle().Position();
            Frame3D A2 = myAdpLastSect.Circle().Position();
            Direction3D D0 = A0.Direction();
            Direction3D D1 = A1.Direction();
            Direction3D D2 = A2.Direction();
            Point3D P0 = myAdpPath.Value(myAdpPath.FirstParameter());
            Point3D P1 = myAdpFirstSect.Value(myAdpFirstSect.FirstParameter());
            Point3D P2 = myAdpLastSect.Value(myAdpLastSect.FirstParameter());

            // les 3 directions doivent etre egales.
            if (!D0.IsEqual(D1, global::Kerf.Engine.Foundation.Precision.Angular()) || !D1.IsEqual(D2, global::Kerf.Engine.Foundation.Precision.Angular()))
            {
                return Ok;
            }

            // les 3 ax1 doivent etre confondus.
            Line3D L = new Line3D(A0.Axis());
            if (!L.Contains(A1.Location(), global::Kerf.Engine.Foundation.Precision.Confusion()) || !L.Contains(A2.Location(), global::Kerf.Engine.Foundation.Precision.Confusion()))
            {
                return Ok;
            }

            // les 3 premiers points doivent etre dans la meme section.
            Direction3D V1 = new Direction3D(new Vector3D(P0, P1));
            Direction3D V2 = new Direction3D(new Vector3D(P0, P2));
            Circle3D Ci = myAdpPath.Circle();
            Vector3D YRef = global::Kerf.Engine.Geometry.ElCLib.CircleDN(myAdpPath.FirstParameter(), A0, Ci.Radius(), 1);
            if (Math.Abs(V1.Dot(new Direction3D(YRef))) > global::Kerf.Engine.Foundation.Precision.Confusion()
                || Math.Abs(V2.Dot(new Direction3D(YRef))) > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                return Ok;
            }

            // OK it`s a Toroidal Surface !!  OUF !!
            Torus T = new Torus(new CoordinateSystem(A0), Ci.Radius(), myRadius);
            Vector3D XRef = new Vector3D(A0.Location(), P0);
            // au maximum on fait un tore d`ouverture en V = PI
            double VV1 = V1.AngleWithRef(new Direction3D(XRef), new Direction3D(YRef));
            double VV2 = V2.AngleWithRef(new Direction3D(XRef), new Direction3D(YRef));
            double deltaV = V2.AngleWithRef(V1, new Direction3D(YRef));
            if (deltaV < 0.0)
            {
                T.VReverse();
                VV1 = -VV1;
                VV2 = 2 * M_PI + VV1 - deltaV;
            }
            mySurface = new TrimmedSurface(new ToroidalSurface(T),
                                                           myAdpPath.FirstParameter(),
                                                           myAdpPath.LastParameter(),
                                                           VV1,
                                                           VV2);
            myExchUV = true;
            Ok = true;
            myStatus = GeomFill_PipeOk;
        }

        return Ok;
    }

    private void ApproxSurf(bool WithParameters)
    {
        // Traitment of general case.
        // generate a sequence of the section by <SweepSectionGenerator>
        // and approximate this sequence.

        if (myType != 4)
        {
            throw new ConstructionErrorException("GeomFill_Pipe");
        }
        GeomFill_SweepSectionGenerator Section = new GeomFill_SweepSectionGenerator(myAdpPath!, myAdpFirstSect!,
                                                                                    myAdpLastSect!, myRadius);

        Section.Perform(myPolynomial);

        GeomFill_Line Line = new GeomFill_Line(Section.NbSections());
        int NbIt = 0;
        double T3d = global::Kerf.Engine.Foundation.Precision.Approximation();
        double T2d = global::Kerf.Engine.Foundation.Precision.PApproximation();
        GeomFill_AppSweep App = new GeomFill_AppSweep(4, 8, T3d, T2d, NbIt, WithParameters);

        App.Perform(Line, Section, 30);

        if (!App.IsDone())
        {
            // throw StdFail_NotDone("Pipe : App not done");
        }
        else
        {
            App.SurfShape(out _, out _, out _, out _, out _, out _);

            mySurface = new BSplineSurface(App.SurfPoles(),
                                                App.SurfWeights(),
                                                App.SurfUKnots(),
                                                App.SurfVKnots(),
                                                App.SurfUMults(),
                                                App.SurfVMults(),
                                                App.UDegree(),
                                                App.VDegree());
            App.TolReached(out myError, out _);
            myStatus = GeomFill_PipeOk;
        }
    }

    /// <summary>The surface built (frozen); null before it is built.</summary>
    public Surface? Surface()
    {
        mySurface?.Freeze();
        return mySurface;
    }

    /// <summary>The surface built, not frozen, for ported callers.</summary>
    internal Surface? InternalSurface() => mySurface;

    /// <summary>Whether the surface is parametrised with u along the path (some elementary surfaces).</summary>
    public bool ExchangeUV() => myExchUV;

    /// <summary>Sets whether particular cases (planes, cylinders, ...) are built where possible.</summary>
    public void GenerateParticularCase(bool B)
    {
        myKPart = B;
    }

    /// <summary>Whether particular cases are built where possible.</summary>
    public bool GenerateParticularCase() => myKPart;

    /// <summary>The approximation error (0 for an elementary surface).</summary>
    public double ErrorOnSurf() => myError;

    /// <summary>Whether a surface was built.</summary>
    public bool IsDone() => myStatus == GeomFill_PipeOk;

    /// <summary>The execution status.</summary>
    public GeomFill_PipeError GetStatus() => myStatus;
}
