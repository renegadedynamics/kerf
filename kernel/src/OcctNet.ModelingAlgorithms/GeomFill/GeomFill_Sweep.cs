// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Sweep.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Sweep.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_Shape;
using static Occt.GeomFill_ApproxStyle;

namespace Occt;

/// <summary>
/// Builds the surface swept by a section law along a location law: an elementary surface where it can (plane,
/// cylinder, cone, sphere, torus, extrusion, revolution; <see cref="ExchangeUV"/> then tells whether the sections are
/// u-isos), else a B-spline surface approximated by <see cref="Approx_SweepApproximation"/>.
/// </summary>
/// <remarks>
/// A working object (decision 21). It keeps the caller's laws, as OCCT does, and changes their interval while it
/// approximates (<see cref="Approx_SweepFunction.SetInterval"/>). <see cref="Surface"/>, <see cref="Restriction"/> and
/// <see cref="Trace"/> hand out frozen geometry; ported code reads the unfrozen results through the
/// <c>Internal*</c> accessors.
/// </remarks>
public sealed class GeomFill_Sweep
{
    //=======================================================================
    // class : GeomFill_Sweep_Eval
    // purpose: The evaluator for curve approximation
    //=======================================================================
    private sealed class GeomFill_Sweep_Eval : AdvApprox_EvaluatorFunction
    {
        private readonly GeomFill_LocFunction theAncore;

        public GeomFill_Sweep_Eval(GeomFill_LocFunction theTool)
        {
            theAncore = theTool;
        }

        public override void Evaluate(int Dimension, ReadOnlySpan<double> StartEnd, double Parameter,
                                      int DerivativeRequest, Span<double> Result, out int ErrorCode)
        {
            ErrorCode = 0;
            theAncore.DN(Parameter, StartEnd[0], StartEnd[1], DerivativeRequest, Result, ref ErrorCode);
        }
    }

    private double First;
    private double Last;
    private double SFirst;
    private double SLast;
    private double Tol3d;
    private double BoundTol;
    private double Tol2d;
    private double TolAngular;
    private double SError;
    private bool myForceApproxC1;
    private readonly GeomFill_LocationLaw myLoc;
    private GeomFill_SectionLaw? mySec;
    private Geom_Surface? mySurface;
    private NCollection_HArray1<Geom2d_Curve?>? myCurve2d;
    private NCollection_HArray2<double>? CError;
    private bool done;
    private bool myExchUV;
    private bool isUReversed;
    private bool isVReversed;
    private readonly bool myKPart;

    /// <summary>A sweep along Location; WithKpart lets <see cref="Build"/> look for elementary surfaces.</summary>
    public GeomFill_Sweep(GeomFill_LocationLaw Location, bool WithKpart = true)
    {
        done = false;

        myLoc = Location;
        myKPart = WithKpart;
        SetTolerance(1.0e-4);
        myForceApproxC1 = false;

        myLoc.GetDomain(out First, out Last);
        SFirst = SLast = 30.081996;
        SError = RealLast();
    }

    /// <summary>
    /// Sets the parametric bounds [First, Last] of the surface and the matching bounds SectionFirst and SectionLast of
    /// the section law: the v-iso at v is Location(v) applied to Section(w), with
    /// w = SectionFirst + (v - First) / (Last - First) * (SectionLast - SectionFirst).
    /// </summary>
    public void SetDomain(double LocFirst, double LocLast, double SectionFirst, double SectionLast)
    {
        First = LocFirst;
        Last = LocLast;
        SFirst = SectionFirst;
        SLast = SectionLast;
    }

    /// <summary>
    /// Sets the tolerances: Tolerance3d of the surface's approximation, BoundTolerance on its boundaries,
    /// Tolerance2d of the 2D curves, ToleranceAngular (radians) between the section law's tangents and the iso-v's.
    /// </summary>
    public void SetTolerance(double Tolerance3d, double BoundTolerance = 1.0, double Tolerance2d = 1.0e-5,
                             double ToleranceAngular = 1.0)
    {
        Tol3d = Tolerance3d;
        BoundTol = BoundTolerance;
        Tol2d = Tolerance2d;
        TolAngular = ToleranceAngular;
    }

    /// <summary>Asks for a C1 surface to be tried when the swept surface proves to be C0.</summary>
    public void SetForceApproxC1(bool ForceApproxC1)
    {
        myForceApproxC1 = ForceApproxC1;
    }

    /// <summary>Whether the sections are u-isos (some elementary surfaces).</summary>
    public bool ExchangeUV() => myExchUV;

    /// <summary>Whether the u parametrisation runs against the section's (the path's if <see cref="ExchangeUV"/>).</summary>
    public bool UReversed() => isUReversed;

    /// <summary>Whether the v parametrisation runs against the path's (the section's if <see cref="ExchangeUV"/>).</summary>
    public bool VReversed() => isVReversed;

    /// <summary>
    /// Builds the swept surface of Section with the approximation strategy Methode, the continuity Continuity in v,
    /// at most degree Degmax and Segmax spans in v.
    /// </summary>
    public void Build(GeomFill_SectionLaw Section,
                      GeomFill_ApproxStyle Methode = GeomFill_Location,
                      GeomAbs_Shape Continuity = GeomAbs_C2,
                      int Degmax = 10,
                      int Segmax = 30)
    {
        // Inits
        done = false;
        myExchUV = false;
        isUReversed = isVReversed = false;
        mySec = Section;

        if ((SFirst == SLast) && (SLast == 30.081996))
        {
            mySec.GetDomain(out SFirst, out SLast);
        }

        bool isKPart = false, isProduct = false;

        // Traitement des KPart
        if (myKPart)
        {
            isKPart = BuildKPart();
        }

        if (!isKPart)
        {
            myExchUV = false;
            isUReversed = isVReversed = false;
        }

        // Traitement des produits Formelles
        if ((!isKPart) && (Methode == GeomFill_Location))
        {
            Geom_BSplineSurface? BS;
            BS = mySec.BSplineSurface();
            if (BS is not null)
            {
                // Approx de la loi
                // OCCT: the call is commented out, so GeomFill_Location approximates globally (occt-issues.md).
                //    isProduct = BuildProduct(Continuity, Degmax, Segmax);
            }
        }

        if (isKPart || isProduct)
        {
            // Approx du 2d
            done = Build2d(Continuity, Degmax, Segmax);
        }
        else
        {
            // Approx globale
            done = BuildAll(Continuity, Degmax, Segmax);
        }
    }

    private bool Build2d(GeomAbs_Shape Continuity, int Degmax, int Segmax)
    {
        bool Ok = false;
        if (myLoc.Nb2dCurves() == 0)
        {
            Ok = true;
        }
        return Ok;
    }

    private bool BuildAll(GeomAbs_Shape Continuity, int Degmax, int Segmax)
    {
        bool Ok = false;

        GeomFill_SweepFunction Func = new GeomFill_SweepFunction(mySec!, myLoc, First, SFirst,
                                                                 (SLast - SFirst) / (Last - First));
        Approx_SweepApproximation Approx = new Approx_SweepApproximation(Func);

        Approx.Perform(First, Last, Tol3d, BoundTol, Tol2d, TolAngular, Continuity, Degmax, Segmax);

        if (Approx.IsDone())
        {
            Ok = true;

            // La surface
            Approx.SurfShape(out _, out _, out int NbUPoles, out int NbVPoles, out int NbUKnots, out int NbVKnots);

            NCollection_Array2<gp_Pnt> Poles = new(1, NbUPoles, 1, NbVPoles);
            NCollection_Array2<double> Weights = new(1, NbUPoles, 1, NbVPoles);
            NCollection_Array1<double> UKnots = new(1, NbUKnots), VKnots = new(1, NbVKnots);
            NCollection_Array1<int> UMults = new(1, NbUKnots), VMults = new(1, NbVKnots);

            Approx.Surface(Poles, Weights, UKnots, VKnots, UMults, VMults);

            mySurface = new Geom_BSplineSurface(Poles,
                                                Weights,
                                                UKnots,
                                                VKnots,
                                                UMults,
                                                VMults,
                                                Approx.UDegree(),
                                                Approx.VDegree(),
                                                mySec!.IsUPeriodic());
            SError = Approx.MaxErrorOnSurf();

            if (myForceApproxC1 && !mySurface.IsCNv(1))
            {
                double theTol = 1.0e-4;
                GeomAbs_Shape theUCont = GeomAbs_C1, theVCont = GeomAbs_C1;
                int degU = 14, degV = 14;
                int nmax = 16;
                int thePrec = 1;

                GeomConvert_ApproxSurface ConvertApprox = new GeomConvert_ApproxSurface(mySurface, theTol, theUCont,
                                                                                        theVCont, degU, degV, nmax,
                                                                                        thePrec);
                if (ConvertApprox.HasResult())
                {
                    mySurface = ConvertApprox.Surface();
                    myCurve2d = new NCollection_HArray1<Geom2d_Curve?>(1, 2);
                    CError = new NCollection_HArray2<double>(1, 2, 1, 2);

                    Geom_BSplineSurface BSplSurf = (Geom_BSplineSurface)mySurface!;

                    gp_Dir2d D = new gp_Dir2d(gp_Dir2d.D.Y);
                    gp_Pnt2d P = new gp_Pnt2d(BSplSurf.UKnot(1), 0);
                    Geom2d_Line LC1 = new Geom2d_Line(P, D);
                    Geom2d_TrimmedCurve TC1 = new Geom2d_TrimmedCurve(LC1, 0, BSplSurf.VKnot(BSplSurf.NbVKnots()));

                    myCurve2d.SetValue(1, TC1);
                    CError.SetValue(1, 1, 0.0);
                    CError.SetValue(2, 1, 0.0);

                    P.SetCoord(BSplSurf.UKnot(BSplSurf.NbUKnots()), 0);
                    Geom2d_Line LC2 = new Geom2d_Line(P, D);
                    Geom2d_TrimmedCurve TC2 = new Geom2d_TrimmedCurve(LC2, 0, BSplSurf.VKnot(BSplSurf.NbVKnots()));

                    myCurve2d.SetValue(myCurve2d.Length(), TC2);
                    CError.SetValue(1, myCurve2d.Length(), 0.0);
                    CError.SetValue(2, myCurve2d.Length(), 0.0);

                    SError = theTol;
                }
            } // if (!mySurface->IsCNv(1))

            // Les Courbes 2d
            if (myCurve2d is null)
            {
                myCurve2d = new NCollection_HArray1<Geom2d_Curve?>(1, 2 + myLoc.TraceNumber());
                CError = new NCollection_HArray2<double>(1, 2, 1, 2 + myLoc.TraceNumber());
                int kk, ii, ifin = 1, ideb;

                if (myLoc.HasFirstRestriction())
                {
                    ideb = 1;
                }
                else
                {
                    ideb = 2;
                }
                ifin += myLoc.TraceNumber();
                if (myLoc.HasLastRestriction())
                {
                    ifin++;
                }

                for (ii = ideb, kk = 1; ii <= ifin; ii++, kk++)
                {
                    Geom2d_BSplineCurve C = new Geom2d_BSplineCurve(Approx.Curve2dPoles(kk),
                                                                    Approx.Curves2dKnots(),
                                                                    Approx.Curves2dMults(),
                                                                    Approx.Curves2dDegree());
                    myCurve2d.SetValue(ii, C);
                    CError.SetValue(1, ii, Approx.Max2dError(kk));
                    CError.SetValue(2, ii, Approx.Max2dError(kk));
                }

                // Si les courbes de restriction, ne sont pas calcules, on prend
                // les iso Bords.
                if (!myLoc.HasFirstRestriction())
                {
                    gp_Dir2d D = new gp_Dir2d(gp_Dir2d.D.Y);
                    gp_Pnt2d P = new gp_Pnt2d(UKnots[UKnots.Lower()], 0);
                    Geom2d_Line LC = new Geom2d_Line(P, D);
                    Geom2d_TrimmedCurve TC = new Geom2d_TrimmedCurve(LC, First, Last);

                    myCurve2d.SetValue(1, TC);
                    CError.SetValue(1, 1, 0.0);
                    CError.SetValue(2, 1, 0.0);
                }

                if (!myLoc.HasLastRestriction())
                {
                    gp_Dir2d D = new gp_Dir2d(gp_Dir2d.D.Y);
                    gp_Pnt2d P = new gp_Pnt2d(UKnots[UKnots.Upper()], 0);
                    Geom2d_Line LC = new Geom2d_Line(P, D);
                    Geom2d_TrimmedCurve TC = new Geom2d_TrimmedCurve(LC, First, Last);
                    myCurve2d.SetValue(myCurve2d.Length(), TC);
                    CError.SetValue(1, myCurve2d.Length(), 0.0);
                    CError.SetValue(2, myCurve2d.Length(), 0.0);
                }
            } // if (myCurve2d.IsNull())
        }
        return Ok;
    }

    // OCCT: BuildProduct is never called (Build's call is commented out); kept as OCCT has it.
    private bool BuildProduct(GeomAbs_Shape Continuity, int Degmax, int Segmax)
    {
        bool Ok = false;

        Geom_BSplineSurface? BSurf = mySec!.BSplineSurface()?.Copy() as Geom_BSplineSurface;
        if (BSurf is null)
        {
            return Ok; // Ce mode de construction est impossible
        }

        int NbIntervalC2, NbIntervalC3;
        GeomFill_LocFunction Func = new GeomFill_LocFunction(myLoc);

        NbIntervalC2 = myLoc.NbIntervals(GeomAbs_C2);
        NbIntervalC3 = myLoc.NbIntervals(GeomAbs_C3);
        NCollection_Array1<double> Param_de_decoupeC2 = new(1, NbIntervalC2 + 1);
        myLoc.Intervals(Param_de_decoupeC2, GeomAbs_C2);
        NCollection_Array1<double> Param_de_decoupeC3 = new(1, NbIntervalC3 + 1);
        myLoc.Intervals(Param_de_decoupeC3, GeomAbs_C3);

        AdvApprox_PrefAndRec Preferentiel = new AdvApprox_PrefAndRec(Param_de_decoupeC2, Param_de_decoupeC3);

        NCollection_HArray1<double> ThreeDTol = new NCollection_HArray1<double>(1, 4);
        ThreeDTol.Init(Tol3d); // A Affiner...

        GeomFill_Sweep_Eval eval = new GeomFill_Sweep_Eval(Func);
        AdvApprox_ApproxAFunction Approx = new AdvApprox_ApproxAFunction(0,
                                                                         0,
                                                                         4,
                                                                         ThreeDTol,
                                                                         ThreeDTol,
                                                                         ThreeDTol,
                                                                         First,
                                                                         Last,
                                                                         Continuity,
                                                                         Degmax,
                                                                         Segmax,
                                                                         eval,
                                                                         Preferentiel);

        Ok = Approx.HasResult();
        if (Ok)
        {
            // OCCT: the tensorial product is commented out; the section's surface is kept as it is.
            mySurface = BSurf;
        }
        return Ok;
    }

    //  Modified by skv - Thu Feb  5 18:05:03 2004 OCC5073 Begin
    //  Conditions:
    //     * theSec should be constant
    //     * the type of section should be a line
    //     * theLoc should represent a translation.
    private static bool IsSweepParallelSpine(GeomFill_LocationLaw theLoc, GeomFill_SectionLaw theSec, double theTol)
    {
        // Get the first and last transformations of the location
        double aFirst;
        double aLast;
        gp_Vec VBegin = new gp_Vec();
        gp_Vec VEnd = new gp_Vec();
        gp_Mat M = new gp_Mat();
        gp_GTrsf GTfBegin = new gp_GTrsf();
        gp_Trsf TfBegin = new gp_Trsf();
        gp_GTrsf GTfEnd = new gp_GTrsf();
        gp_Trsf TfEnd = new gp_Trsf();

        theLoc.GetDomain(out aFirst, out aLast);

        // Get the first transformation
        theLoc.D0(aFirst, ref M, ref VBegin);

        GTfBegin.SetVectorialPart(M);
        GTfBegin.SetTranslationPart(VBegin.XYZ());

        TfBegin.SetValues(GTfBegin[1, 1],
                          GTfBegin[1, 2],
                          GTfBegin[1, 3],
                          GTfBegin[1, 4],
                          GTfBegin[2, 1],
                          GTfBegin[2, 2],
                          GTfBegin[2, 3],
                          GTfBegin[2, 4],
                          GTfBegin[3, 1],
                          GTfBegin[3, 2],
                          GTfBegin[3, 3],
                          GTfBegin[3, 4]);

        // Get the last transformation
        theLoc.D0(aLast, ref M, ref VEnd);

        GTfEnd.SetVectorialPart(M);
        GTfEnd.SetTranslationPart(VEnd.XYZ());

        TfEnd.SetValues(GTfEnd[1, 1],
                        GTfEnd[1, 2],
                        GTfEnd[1, 3],
                        GTfEnd[1, 4],
                        GTfEnd[2, 1],
                        GTfEnd[2, 2],
                        GTfEnd[2, 3],
                        GTfEnd[2, 4],
                        GTfEnd[3, 1],
                        GTfEnd[3, 2],
                        GTfEnd[3, 3],
                        GTfEnd[3, 4]);

        Geom_Surface aSurf = theSec.BSplineSurface()!;
        double Umin;
        double Umax;
        double Vmin;
        double Vmax;

        aSurf.Bounds(out Umin, out Umax, out Vmin, out Vmax);

        // Get and transform the first section
        Geom_Curve FirstSection = theSec.ConstantSection();
        GeomAdaptor_Curve ACFirst = new GeomAdaptor_Curve(FirstSection);

        double UFirst = ACFirst.FirstParameter();
        gp_Lin L = ACFirst.Line();

        L.Transform(TfBegin);

        // Get and transform the last section
        Geom_Curve aLastSection = aSurf.VIso(Vmax);
        double aFirstParameter = aLastSection.FirstParameter();
        gp_Pnt aPntLastSec = aLastSection.Value(aFirstParameter);

        aPntLastSec.Transform(TfEnd);

        gp_Pnt aPntFirstSec = ElCLib.Value(UFirst, L);
        gp_Vec aVecSec = new gp_Vec(aPntFirstSec, aPntLastSec);
        gp_Vec aVecSpine = VEnd - VBegin;

        bool isParallel = aVecSec.IsParallel(aVecSpine, theTol);

        return isParallel;
    }

    //  Modified by skv - Thu Feb  5 18:05:01 2004 OCC5073 End

    private bool BuildKPart()
    {
        bool Ok = false;
        bool isUPeriodic = false;
        bool isVPeriodic = false;
        bool IsTrsf = true;

        isUPeriodic = mySec!.IsUPeriodic();
        Geom_Surface? S = null;
        GeomAbs_CurveType SectionType;
        gp_Vec V = new gp_Vec();
        gp_Mat M = new gp_Mat();
        double levier, error = 0;
        double UFirst = 0, VFirst = First, ULast = 0, VLast = Last;
        double Tol = Min(Tol3d, BoundTol);

        // (1) Trajectoire Rectilignes -------------------------
        if (myLoc.IsTranslation(ref error))
        {
            // Donne de la translation
            gp_Vec DP, DS = new gp_Vec();
            myLoc.D0(1, ref M, ref DS);
            myLoc.D0(0, ref M, ref V);
            DP = DS - V;
            DP.Normalize();
            gp_GTrsf Tf = new gp_GTrsf();
            gp_Trsf Tf2 = new gp_Trsf();
            Tf.SetVectorialPart(M);
            Tf.SetTranslationPart(V.XYZ());
            try
            { // Pas joli mais il n'y as pas d'autre moyens de tester SetValues
                Tf2.SetValues(Tf[1, 1],
                              Tf[1, 2],
                              Tf[1, 3],
                              Tf[1, 4],
                              Tf[2, 1],
                              Tf[2, 2],
                              Tf[2, 3],
                              Tf[2, 4],
                              Tf[3, 1],
                              Tf[3, 2],
                              Tf[3, 3],
                              Tf[3, 4]);
            }
            catch (Standard_ConstructionError)
            {
                IsTrsf = false;
            }
            if (!IsTrsf)
            {
                return false;
            }

            // (1.1) Cas Extrusion
            if (mySec.IsConstant(ref error))
            {
                Geom_Curve Section;
                Section = mySec.ConstantSection();
                GeomAdaptor_Curve AC = new GeomAdaptor_Curve(Section);
                SectionType = AC.GetType();
                UFirst = AC.FirstParameter();
                ULast = AC.LastParameter();
                // (1.1.a) Cas Plan
                if ((SectionType == GeomAbs_Line) && IsTrsf)
                {
                    //  Modified by skv - Thu Feb  5 11:39:06 2004 OCC5073 Begin
                    if (!IsSweepParallelSpine(myLoc, mySec, Tol))
                    {
                        return false;
                    }
                    //  Modified by skv - Thu Feb  5 11:39:08 2004 OCC5073 End
                    gp_Lin L = AC.Line();
                    L.Transform(Tf2);
                    DS.SetXYZ(L.Position().Direction().XYZ());
                    DS.Normalize();
                    levier = Math.Abs(DS.Dot(DP));
                    SError = error + levier * Math.Abs(Last - First);
                    if (SError <= Tol)
                    {
                        Ok = true;
                        gp_Ax2 AxisOfPlane = new gp_Ax2(L.Location(), new gp_Dir(DS ^ DP), new gp_Dir(DS));
                        S = new Geom_Plane(new gp_Ax3(AxisOfPlane));
                    }
                    else
                    {
                        SError = 0.0;
                    }
                }

                // (1.1.b) Cas Cylindrique
                if ((SectionType == GeomAbs_Circle) && IsTrsf)
                {
                    const double TolProd = 1.0e-6;

                    gp_Circ C = AC.Circle();
                    C.Transform(Tf2);

                    DS.SetXYZ(C.Position().Direction().XYZ());
                    DS.Normalize();
                    levier = Math.Abs(DS.CrossMagnitude(DP)) * C.Radius();
                    SError = levier * Math.Abs(Last - First);
                    if (SError <= TolProd)
                    {
                        Ok = true;
                        gp_Ax3 axe = new gp_Ax3(C.Location(), new gp_Dir(DP), C.Position().XDirection());
                        S = new Geom_CylindricalSurface(axe, C.Radius());
                        if (C.Position().Direction().IsOpposite(axe.Direction(), 0.1))
                        {
                            double f, l;
                            // L'orientation parametrique est inversee
                            l = 2 * M_PI - UFirst;
                            f = 2 * M_PI - ULast;
                            UFirst = f;
                            ULast = l;
                            isUReversed = true;
                        }
                    }
                    else
                    {
                        SError = 0.0;
                    }
                }

                // (1.1.c) C'est bien une extrusion
                if (!Ok)
                {
                    if (IsTrsf)
                    {
                        Section.Transform(Tf2);
                        S = new Geom_SurfaceOfLinearExtrusion(Section, new gp_Dir(DP));
                        SError = 0.0;
                        Ok = true;
                    }
                    else
                    { // extrusion sur BSpline
                    }
                }
            }

            // (1.2) Cas conique
            else if (mySec.IsConicalLaw(ref error))
            {
                gp_Pnt P1, P2, Centre0, Centre1, Centre2;
                gp_Vec dsection;
                Geom_Curve Section;
                GeomAdaptor_Curve AC = new GeomAdaptor_Curve();
                gp_Circ C;
                double R1, R2;

                Section = mySec.CirclSection(SLast);
                Section.Transform(Tf2);
                Section.Translate(Last * DP);
                AC.Load(Section);
                C = AC.Circle();
                Centre2 = C.Location();
                AC.D1(0, out P2, out dsection);
                R2 = C.Radius();

                Section = mySec.CirclSection(SFirst);
                Section.Transform(Tf2);
                Section.Translate(First * DP);
                AC.Load(Section);
                C = AC.Circle();
                Centre1 = C.Location();
                P1 = AC.Value(0);
                R1 = C.Radius();

                Section = mySec.CirclSection(SFirst - First * (SLast - SFirst) / (Last - First));
                Section.Transform(Tf2);
                AC.Load(Section);
                C = AC.Circle();
                Centre0 = C.Location();

                double Angle;
                gp_Vec N = new gp_Vec(Centre1, P1);
                if (N.Magnitude() < 1.0e-9)
                {
                    gp_Vec Bis = new gp_Vec(Centre2, P2);
                    N = Bis;
                }
                gp_Vec L = new gp_Vec(P1, P2), Dir = new gp_Vec(Centre1, Centre2);

                Angle = L.Angle(Dir);
                if ((Angle > 0.01) && (Angle < M_PI / 2 - 0.01))
                {
                    if (R2 < R1)
                    {
                        Angle = -Angle;
                    }
                    SError = error;
                    gp_Ax3 Axis = new gp_Ax3(Centre0, new gp_Dir(Dir), new gp_Dir(N));
                    S = new Geom_ConicalSurface(Axis, Angle, C.Radius());
                    // Calcul du glissement parametrique
                    VFirst = First / Math.Cos(Angle);
                    VLast = Last / Math.Cos(Angle);

                    // Bornes en U
                    UFirst = AC.FirstParameter();
                    ULast = AC.LastParameter();
                    S.VIso(VLast).D1(0, out _, out gp_Vec diso);
                    if (diso.Magnitude() > 1.0e-9 && dsection.Magnitude() > 1.0e-9)
                    {
                        isUReversed = diso.IsOpposite(dsection, 0.1);
                    }
                    if (isUReversed)
                    {
                        double f, l;
                        // L'orientation parametrique est inversee
                        l = 2 * M_PI - UFirst;
                        f = 2 * M_PI - ULast;
                        UFirst = f;
                        ULast = l;
                    }

                    // C'est un cone
                    Ok = true;
                }
            }
        }

        // (2) Trajectoire Circulaire
        if (myLoc.IsRotation(ref error))
        {
            if (mySec.IsConstant(ref error))
            {
                // La trajectoire
                gp_Pnt Centre = new gp_Pnt();
                isVPeriodic = (Math.Abs(Last - First - 2 * M_PI) < 1.0e-15);
                double RotRadius;
                gp_Vec DP, DS = new gp_Vec(), DN;
                myLoc.D0(0.1, ref M, ref DS);
                myLoc.D0(0, ref M, ref V);
                myLoc.Rotation(ref Centre);

                DP = DS - V;
                DS.SetXYZ(V.XYZ() - Centre.XYZ());
                RotRadius = DS.Magnitude();
                if (RotRadius > 1.0e-15)
                {
                    DS.Normalize();
                }
                else
                {
                    return false; // Pas de KPart, rotation degeneree
                }
                DN = DS ^ DP;
                DN.Normalize();
                DP = DN ^ DS;
                DP.Normalize();

                gp_GTrsf Tf = new gp_GTrsf();
                gp_Trsf Tf2 = new gp_Trsf();
                Tf.SetVectorialPart(M);
                Tf.SetTranslationPart(V.XYZ());
                // TODO: finding #9 - exception handling is commented out unlike the translation
                // branch (~600-619). Restoring try/catch caused blend regressions. Needs
                // investigation to understand why the catch was intentionally disabled.
                Tf2.SetValues(Tf[1, 1],
                              Tf[1, 2],
                              Tf[1, 3],
                              Tf[1, 4],
                              Tf[2, 1],
                              Tf[2, 2],
                              Tf[2, 3],
                              Tf[2, 4],
                              Tf[3, 1],
                              Tf[3, 2],
                              Tf[3, 3],
                              Tf[3, 4]);
                // La section
                Geom_Curve Section;
                Section = mySec.ConstantSection();
                GeomAdaptor_Curve AC = new GeomAdaptor_Curve(Section);
                SectionType = AC.GetType();
                UFirst = AC.FirstParameter();
                ULast = AC.LastParameter();

                // (2.1) Tore/Sphere ?
                if ((SectionType == GeomAbs_Circle) && IsTrsf)
                {
                    gp_Circ C = AC.Circle();
                    double Radius;
                    bool IsGoodSide = true;
                    C.Transform(Tf2);
                    gp_Vec DC = new gp_Vec();
                    // On calcul le centre eventuel
                    DC.SetXYZ(C.Location().XYZ() - Centre.XYZ());
                    Centre.ChangeCoord() += (DC.Dot(DN)) * DN.XYZ();
                    DC.SetXYZ(C.Location().XYZ() - Centre.XYZ());
                    Radius = DC.Magnitude(); // grand Rayon du tore
                    if ((Radius > Tol) && (DC.Dot(DS) < 0))
                    {
                        IsGoodSide = false;
                    }
                    if (Radius < Tol / 100)
                    {
                        DC = DS; // Pour definir le tore
                    }

                    // On verifie d'abord que le plan de la section est // a
                    // l'axe de rotation
                    gp_Vec NC = new gp_Vec();
                    NC.SetXYZ(C.Position().Direction().XYZ());
                    NC.Normalize();
                    error = Math.Abs(NC.Dot(DN));
                    // Puis on evalue l'erreur commise sur la section,
                    // en pivotant son plan ( pour contenir l'axe de rotation)
                    error += Math.Abs(NC.Dot(DS));
                    error *= C.Radius();
                    if (error <= Tol)
                    {
                        SError = error;
                        error += Radius;
                        if (Radius <= Tol)
                        {
                            // (2.1.a) Sphere
                            double f = UFirst, l = ULast, aRadius = 0.0;
                            SError = error;
                            Centre.BaryCenter(1.0, C.Location(), 1.0);
                            gp_Ax3 AxisOfSphere = new gp_Ax3(Centre, new gp_Dir(DN), new gp_Dir(DS));
                            aRadius = C.Radius();
                            gp_Sphere theSphere = new gp_Sphere(AxisOfSphere, aRadius);
                            S = new Geom_SphericalSurface(theSphere);
                            // Pour les spheres on ne peut pas controler le parametre
                            // V (donc U car  myExchUV = true)
                            // Il faut donc modifier UFirst, ULast...
                            double fpar = AC.FirstParameter();
                            double lpar = AC.LastParameter();
                            Geom_Curve theSection = new Geom_TrimmedCurve(Section, fpar, lpar);
                            theSection.Transform(Tf2);
                            gp_Pnt FirstPoint = theSection.Value(theSection.FirstParameter());
                            gp_Pnt LastPoint = theSection.Value(theSection.LastParameter());
                            double UfirstOnSec, VfirstOnSec, UlastOnSec, VlastOnSec;
                            ElSLib.Parameters(theSphere, FirstPoint, out UfirstOnSec, out VfirstOnSec);
                            ElSLib.Parameters(theSphere, LastPoint, out UlastOnSec, out VlastOnSec);
                            if (VfirstOnSec < VlastOnSec)
                            {
                                f = VfirstOnSec;
                                l = VlastOnSec;
                            }
                            else
                            {
                                // L'orientation parametrique est inversee
                                f = VlastOnSec;
                                l = VfirstOnSec;
                                isUReversed = true;
                            }

                            if (Math.Abs(l - f) <= Precision.PConfusion()
                                || Math.Abs(UlastOnSec - UfirstOnSec) > M_PI_2)
                            {
                                // l == f - "degenerated" surface
                                // UlastOnSec - UfirstOnSec > M_PI_2 - "twisted" surface,
                                // it is impossible to represent with help of trimmed sphere
                                isUReversed = false;
                                return Ok;
                            }

                            if ((f >= -M_PI / 2) && (l <= M_PI / 2))
                            {
                                Ok = true;
                                myExchUV = true;
                                UFirst = f;
                                ULast = l;
                            }
                            else
                            { // On restaure ce qu'il faut
                                isUReversed = false;
                            }
                        }
                        else if (IsGoodSide)
                        {
                            // (2.1.b) Tore
                            gp_Ax3 AxisOfTore = new gp_Ax3(Centre, new gp_Dir(DN), new gp_Dir(DC));
                            S = new Geom_ToroidalSurface(AxisOfTore, Radius, C.Radius());

                            // Pour les tores on ne peut pas controler le parametre
                            // V (donc U car  myExchUV = true)
                            // Il faut donc modifier UFirst, ULast...
                            Geom_Circle? Iso;
                            Iso = S.UIso(0.0) as Geom_Circle;
                            gp_Ax2 axeiso;
                            axeiso = Iso!.Circ().Position();

                            if (C.Position().Direction().IsOpposite(axeiso.Direction(), 0.1))
                            {
                                double f, l;
                                // L'orientation parametrique est inversee
                                l = 2 * M_PI - UFirst;
                                f = 2 * M_PI - ULast;
                                UFirst = f;
                                ULast = l;
                                isUReversed = true;
                            }
                            // On calcul le "glissement" parametrique.
                            double rot;
                            rot = C.Position().XDirection().AngleWithRef(axeiso.XDirection(), axeiso.Direction());
                            UFirst -= rot;
                            ULast -= rot;

                            myExchUV = true;
                            // Attention l'arete de couture dans le cas periodique
                            // n'est peut etre pas a la bonne place...
                            if (isUPeriodic && Math.Abs(UFirst) > Precision.PConfusion())
                            {
                                isUPeriodic = false; // Pour trimmer la surface...
                            }
                            Ok = true;
                        }
                    }
                    else
                    {
                        SError = 0.0;
                    }
                }
                // (2.2) Cone / Cylindre
                if ((SectionType == GeomAbs_Line) && IsTrsf)
                {
                    gp_Lin L = AC.Line();
                    L.Transform(Tf2);
                    gp_Vec DL = new gp_Vec();
                    DL.SetXYZ(L.Direction().XYZ());
                    levier = Max(Math.Abs(AC.FirstParameter()), AC.LastParameter());
                    // si la line est ortogonale au cercle de rotation
                    SError = error + levier * Math.Abs(DL.Dot(DP));
                    if (SError <= Tol)
                    {
                        bool reverse;
                        gp_Lin Dir = new gp_Lin(Centre, new gp_Dir(DN));
                        double aux;
                        aux = DL.Dot(DN);
                        reverse = (aux < 0); // On choisit ici le sens de parametrisation

                        // Calcul du centre du vecteur supportant la "XDirection"
                        gp_Pnt CentreOfSurf;
                        gp_Vec O1O2 = new gp_Vec(Centre, L.Location()), trans;
                        trans = DN;
                        trans *= DN.Dot(O1O2);
                        CentreOfSurf = Centre.Translated(trans);
                        DS.SetXYZ(L.Location().XYZ() - CentreOfSurf.XYZ());

                        error = SError;
                        error += (DL.XYZ()).CrossMagnitude(DN.XYZ()) * levier;
                        if (error <= Tol)
                        {
                            // (2.2.a) Cylindre
                            // si la line est orthogonale au plan de rotation
                            SError = error;
                            //
                            gp_Ax3 Axis = new gp_Ax3(CentreOfSurf, Dir.Direction());
                            if (DS.SquareMagnitude() > gp.Resolution())
                            {
                                Axis.SetXDirection(new gp_Dir(DS));
                            }
                            S = new Geom_CylindricalSurface(Axis, L.Distance(CentreOfSurf));
                            Ok = true;
                            myExchUV = true;
                        }
                        else
                        {
                            // On evalue l'angle du cone
                            double Angle = Math.Abs(Dir.Angle(L));
                            if (Angle > M_PI / 2)
                            {
                                Angle = M_PI - Angle;
                            }
                            if (reverse)
                            {
                                Angle = -Angle;
                            }
                            aux = DS.Dot(DL);
                            if (aux < 0)
                            {
                                Angle = -Angle;
                            }
                            if (Math.Abs(Math.Abs(Angle) - M_PI / 2) > 0.01)
                            {
                                // (2.2.b) Cone
                                // si les 2 droites ne sont pas orthogonales
                                double Radius = CentreOfSurf.Distance(L.Location());
                                gp_Ax3 Axis = new gp_Ax3(CentreOfSurf, Dir.Direction(), new gp_Dir(DS));
                                S = new Geom_ConicalSurface(Axis, Angle, Radius);
                                myExchUV = true;
                                Ok = true;
                            }
                            else
                            {
                                // On n'as pas conclue, on remet l'erreur a 0.
                                SError = 0.0;
                            }
                        }
                        if (Ok && reverse)
                        {
                            // On reverse le parametre
                            double uf, ul;
                            Geom_Line CL = new Geom_Line(L);
                            uf = CL.ReversedParameter(ULast);
                            ul = CL.ReversedParameter(UFirst);

                            // Following the example of the code for the sphere:
                            // "we cannot control U because myExchUV = true,
                            // so it is necessary to change UFirst and ULast"
                            UFirst = ul;
                            ULast = uf;
                        }
                    }
                    else
                    {
                        SError = 0.0;
                    }
                }

                // (2.3) Revolution
                if (!Ok)
                {
                    if (IsTrsf)
                    {
                        Section.Transform(Tf2);
                        gp_Ax1 Axis = new gp_Ax1(Centre, new gp_Dir(DN));
                        S = new Geom_SurfaceOfRevolution(Section, Axis);
                        myExchUV = true;
                        SError = 0.0;
                        Ok = true;
                    }
                }
            }
        }

        if (Ok)
        { // On trimme la surface
            if (myExchUV)
            {
                bool b;
                b = isUPeriodic;
                isUPeriodic = isVPeriodic;
                isVPeriodic = b;
                double r;
                r = UFirst;
                UFirst = VFirst;
                VFirst = r;
                r = ULast;
                ULast = VLast;
                VLast = r;
            }

            if (!isUPeriodic && !isVPeriodic)
            {
                mySurface = new Geom_RectangularTrimmedSurface(S!, UFirst, ULast, VFirst, VLast);
            }
            else if (isUPeriodic)
            {
                if (isVPeriodic)
                {
                    mySurface = S;
                }
                else
                {
                    mySurface = new Geom_RectangularTrimmedSurface(S!, VFirst, VLast, false);
                }
            }
            else
            {
                mySurface = new Geom_RectangularTrimmedSurface(S!, UFirst, ULast, true);
            }
        }

        return Ok;
    }

    /// <summary>Whether the surface is built.</summary>
    public bool IsDone() => done;

    /// <summary>The approximation error on the surface.</summary>
    public double ErrorOnSurface() => SError;

    /// <summary>The approximation errors of the first or last restriction.</summary>
    public void ErrorOnRestriction(bool IsFirst, out double UError, out double VError)
    {
        int ind;
        if (IsFirst)
        {
            ind = 1;
        }
        else
        {
            ind = myCurve2d!.Length();
        }

        UError = CError!.Value(1, ind);
        VError = CError.Value(2, ind);
    }

    /// <summary>The approximation errors of the trace IndexOfTrace.</summary>
    public void ErrorOnTrace(int IndexOfTrace, out double UError, out double VError)
    {
        int ind = IndexOfTrace + 1;
        if (IndexOfTrace > myLoc.TraceNumber())
        {
            throw new Standard_OutOfRange(" GeomFill_Sweep::ErrorOnTrace");
        }

        UError = CError!.Value(1, ind);
        VError = CError.Value(2, ind);
    }

    /// <summary>The swept surface (frozen), or null before a successful <see cref="Build"/>.</summary>
    public Geom_Surface? Surface()
    {
        mySurface?.Freeze();
        return mySurface;
    }

    /// <summary>The swept surface, not frozen, for ported callers.</summary>
    internal Geom_Surface? InternalSurface() => mySurface;

    /// <summary>The first or last restriction, a 2D curve on the surface (frozen).</summary>
    public Geom2d_Curve? Restriction(bool IsFirst)
    {
        Geom2d_Curve? C = InternalRestriction(IsFirst);
        C?.Freeze();
        return C;
    }

    /// <summary>The first or last restriction, not frozen, for ported callers.</summary>
    internal Geom2d_Curve? InternalRestriction(bool IsFirst)
    {
        if (IsFirst)
        {
            return myCurve2d!.Value(1);
        }
        return myCurve2d!.Value(myCurve2d.Length());
    }

    /// <summary>The number of traces of the location law.</summary>
    public int NumberOfTrace() => myLoc.TraceNumber();

    /// <summary>The trace IndexOfTrace, a 2D curve on the surface (frozen).</summary>
    public Geom2d_Curve? Trace(int IndexOfTrace)
    {
        Geom2d_Curve? C = InternalTrace(IndexOfTrace);
        C?.Freeze();
        return C;
    }

    /// <summary>The trace IndexOfTrace, not frozen, for ported callers.</summary>
    internal Geom2d_Curve? InternalTrace(int IndexOfTrace)
    {
        int ind = IndexOfTrace + 1;
        if (IndexOfTrace > myLoc.TraceNumber())
        {
            throw new Standard_OutOfRange(" GeomFill_Sweep::Trace");
        }
        return myCurve2d!.Value(ind);
    }
}
