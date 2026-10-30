// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_SectionPlacement.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_SectionPlacement.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;

namespace Occt;

/// <summary>
/// Places a section (a curve or a point) on a sweep's path: finds the parameter on the path and on the section
/// where they meet best, and the transformation that takes the section into the location law's frame there.
/// </summary>
/// <remarks>
/// A working object (decision 21). It keeps the caller's location law and section as OCCT keeps the handles and
/// only evaluates them (it adapts the section with an adaptor of its own). <see cref="Section"/> and
/// <see cref="ModifiedSection"/> return new, unfrozen curves.
/// </remarks>
public sealed class GeomFill_SectionPlacement
{
    private bool done;
    private bool isplan;
    private gp_Ax1 TheAxe;
    private readonly double Gabarit;
    private GeomFill_LocationLaw myLaw;
    private readonly GeomAdaptor_Curve myAdpSection;
    private readonly Geom_Curve? mySection;
    private double SecParam;
    private double PathParam;
    private double Dist;
    private double AngleMax;
    private readonly Extrema_ExtPC myExt;
    private readonly bool myIsPoint;
    private readonly gp_Pnt myPoint;

    private static void Tangente(Adaptor3d_Curve Path, double Param, out gp_Pnt P, out gp_Vec Tang)
    {
        Path.D1(Param, out P, out Tang);
        double Norm = Tang.Magnitude();

        for (int ii = 2; (ii < 12) && (Norm < Precision.Confusion()); ii++)
        {
            Tang = Path.DN(Param, ii);
            Norm = Tang.Magnitude();
        }

        if (Norm > 100 * gp.Resolution())
        {
            Tang /= Norm;
        }
    }

    private static double Penalite(double angle, double dist)
    {
        double penal;

        if (dist < 1)
        {
            penal = Math.Sqrt(dist);
        }
        else if (dist < 2)
        {
            penal = Math.Pow(dist, 2);
        }
        else
        {
            penal = dist + 2;
        }

        if (angle > 1.0e-3)
        {
            penal += 1.0 / angle - 2.0 / M_PI;
        }
        else
        {
            penal += 1.0e3;
        }

        return penal;
    }

    private static double EvalAngle(in gp_Vec V1, in gp_Vec V2)
    {
        double angle;
        angle = V1.Angle(V2);
        if (angle > M_PI / 2)
        {
            angle = M_PI - angle;
        }
        return angle;
    }

    // Examines an extremum to update Dist and Param.
    private static void DistMini(Extrema_ExtPC Ext, Adaptor3d_Curve C, ref double Dist, ref double Param)
    {
        double dist1, dist2;
        int ii;
        double Dist2 = RealLast();

        Ext.TrimmedSquareDistances(out dist1, out dist2, out _, out _);
        if ((dist1 < Dist2) || (dist2 < Dist2))
        {
            if (dist1 < dist2)
            {
                Dist2 = dist1;
                Param = C.FirstParameter();
            }
            else
            {
                Dist2 = dist2;
                Param = C.LastParameter();
            }
        }

        if (Ext.IsDone())
        {
            for (ii = 1; ii <= Ext.NbExt(); ii++)
            {
                if (Ext.SquareDistance(ii) < Dist2)
                {
                    Dist2 = Ext.SquareDistance(ii);
                    Param = Ext.Point(ii).Parameter();
                }
            }
        }
        Dist = Math.Sqrt(Dist2);
    }

    /// <summary>Prepares the placement of Section (a Geom_Curve or a Geom_CartesianPoint) along the location law L.</summary>
    public GeomFill_SectionPlacement(GeomFill_LocationLaw L, Geom_Geometry Section)
    {
        myLaw = L;
        SecParam = 0.0;
        PathParam = 0.0;
        Dist = RealLast();
        AngleMax = 0.0;
        myAdpSection = new GeomAdaptor_Curve();
        myExt = new Extrema_ExtPC();
        TheAxe = new gp_Ax1();

        done = false;
        isplan = false;
        myIsPoint = false;

        if (Section is Geom_CartesianPoint aPoint && Section.GetType() == typeof(Geom_CartesianPoint))
        {
            myIsPoint = true;
            myPoint = aPoint.Pnt();
            isplan = true;
        }
        else
        {
            Geom_Curve CurveSection = (Geom_Curve)Section;
            myAdpSection.Load(CurveSection);
            mySection = CurveSection;
        }

        int i, j, NbPoles = 0;
        double aXmin, aYmin, aZmin, aXmax, aYmax, aZmax;

        // Boite d'encombrement de la section pour en deduire le gabarit
        Bnd_Box box = new Bnd_Box();
        if (myIsPoint)
        {
            box.Add(myPoint);
        }
        else
        {
            BndLib_Add3dCurve.Add(myAdpSection, 1.0e-4, ref box);
        }
        box.Get(out aXmin, out aYmin, out aZmin, out aXmax, out aYmax, out aZmax);

        double DX = aXmax - aXmin;
        double DY = aYmax - aYmin;
        double DZ = aZmax - aZmin;
        Gabarit = Math.Sqrt(DX * DX + DY * DY + DZ * DZ) / 2.0;

        Gabarit += Precision.Confusion(); // Cas des toute petite

        // Initialisation de TheAxe pour les cas singulier
        if (!myIsPoint)
        {
            Tangente(myAdpSection, (myAdpSection.FirstParameter() + myAdpSection.LastParameter()) / 2, out gp_Pnt P,
                     out gp_Vec V);
            TheAxe.SetLocation(P);
            TheAxe.SetDirection(new gp_Dir(V));

            // y a t'il un Plan moyen ?
            GeomAbs_CurveType TheType = myAdpSection.GetType();
            switch (TheType)
            {
                case GeomAbs_Circle:
                    isplan = true;
                    TheAxe = myAdpSection.Circle().Axis();
                    break;
                case GeomAbs_Ellipse:
                    isplan = true;
                    TheAxe = myAdpSection.Ellipse().Axis();
                    break;
                case GeomAbs_Hyperbola:
                    isplan = true;
                    TheAxe = myAdpSection.Hyperbola().Axis();
                    break;
                case GeomAbs_Parabola:
                    isplan = true;
                    TheAxe = myAdpSection.Parabola().Axis();
                    break;
                case GeomAbs_Line:
                    NbPoles = 0; // Pas de Plan !!
                    break;
                case GeomAbs_BezierCurve:
                case GeomAbs_BSplineCurve:
                    NbPoles = myAdpSection.NbPoles();
                    break;
                default:
                    NbPoles = 21;
                    break;
            }

            if (!isplan && NbPoles > 2)
            {
                // Calcul d'un plan moyen.
                NCollection_HArray1<gp_Pnt> Pnts;
                double first = myAdpSection.FirstParameter();
                double last = myAdpSection.LastParameter();
                if (myAdpSection.IsPeriodic())
                {
                    // Correct boundaries to avoid mistake of LocateU
                    Geom_Curve aCurve = myAdpSection.Curve()!;
                    if (aCurve.GetType() == typeof(Geom_TrimmedCurve))
                    {
                        aCurve = ((Geom_TrimmedCurve)aCurve).BasisCurve();
                    }
                    double Ufirst = aCurve.FirstParameter();
                    double aPeriod = aCurve.Period();
                    double U1 = Ufirst + Math.Floor((first - Ufirst) / aPeriod) * aPeriod;
                    double U2 = U1 + aPeriod;
                    if (Math.Abs(first - U1) <= Precision.PConfusion())
                    {
                        first = U1;
                    }
                    if (Math.Abs(last - U2) <= Precision.PConfusion())
                    {
                        last = U2;
                    }
                }
                double t, delta;
                if (myAdpSection.GetType() == GeomAbs_BSplineCurve)
                {
                    Geom_BSplineCurve BC = (Geom_BSplineCurve)myAdpSection.Curve()!;
                    int I1, I2, I3, I4;
                    BC.LocateU(first, Precision.Confusion(), out I1, out I2);
                    BC.LocateU(last, Precision.Confusion(), out I3, out I4);
                    int NbKnots = I3 - I2 + 1;

                    int NbLocalPnts = 10;
                    int NbPnts = (NbKnots - 1) * NbLocalPnts;
                    if (NbPnts < 0)
                    {
                        NbPnts = 0;
                    }
                    if (I1 != I2)
                    {
                        NbPnts += NbLocalPnts;
                    }
                    if (I3 != I4 && first < BC.Knot(I3))
                    {
                        NbPnts += NbLocalPnts;
                    }
                    if (!myAdpSection.IsClosed())
                    {
                        NbPnts++;
                    }
                    Pnts = new NCollection_HArray1<gp_Pnt>(1, NbPnts);
                    int nb = 1;
                    if (I1 != I2)
                    {
                        double locallast = (BC.Knot(I2) < last) ? BC.Knot(I2) : last;
                        delta = (locallast - first) / NbLocalPnts;
                        for (j = 0; j < NbLocalPnts; j++)
                        {
                            t = first + j * delta;
                            Pnts.SetValue(nb++, myAdpSection.Value(t));
                        }
                    }
                    for (i = I2; i < I3; i++)
                    {
                        t = BC.Knot(i);
                        delta = (BC.Knot(i + 1) - t) / NbLocalPnts;
                        for (j = 0; j < NbLocalPnts; j++)
                        {
                            Pnts.SetValue(nb++, myAdpSection.Value(t));
                            t += delta;
                        }
                    }
                    if (I3 != I4 && first < BC.Knot(I3))
                    {
                        t = BC.Knot(I3);
                        delta = (last - t) / NbLocalPnts;
                        for (j = 0; j < NbLocalPnts; j++)
                        {
                            Pnts.SetValue(nb++, myAdpSection.Value(t));
                            t += delta;
                        }
                    }
                    if (!myAdpSection.IsClosed())
                    {
                        Pnts.SetValue(nb, myAdpSection.Value(last));
                    }
                }
                else // other type
                {
                    int NbPnts = NbPoles - 1;
                    if (!myAdpSection.IsClosed())
                    {
                        NbPnts++;
                    }
                    Pnts = new NCollection_HArray1<gp_Pnt>(1, NbPnts);
                    delta = (last - first) / (NbPoles - 1);
                    for (i = 0; i < NbPoles - 1; i++)
                    {
                        t = first + i * delta;
                        Pnts.SetValue(i + 1, myAdpSection.Value(t));
                    }
                    if (!myAdpSection.IsClosed())
                    {
                        Pnts.SetValue(NbPnts, myAdpSection.Value(last));
                    }
                }

                GeomLib.AxeOfInertia(Pnts, out gp_Ax2 axe, out bool issing, Precision.Confusion());
                if (!issing)
                {
                    isplan = true;
                    TheAxe.SetLocation(axe.Location());
                    TheAxe.SetDirection(axe.Direction());
                }
            }

            myExt.Initialize(myAdpSection, myAdpSection.FirstParameter(), myAdpSection.LastParameter(),
                             Precision.Confusion());
        }
    }

    /// <summary>Changes the location law.</summary>
    public void SetLocation(GeomFill_LocationLaw L)
    {
        myLaw = L;
    }

    /// <summary>Computes the placement on the curve of the location law.</summary>
    public void Perform(double Tol)
    {
        Adaptor3d_Curve Path;
        Path = myLaw.GetCurve()!;
        Perform(Path, Tol);
    }

    /// <summary>Computes the placement on Path.</summary>
    public void Perform(Adaptor3d_Curve Path, double Tol)
    {
        double IntTol = 1.0e-5;
        double DistCenter = Precision.Infinite();

        if (myIsPoint)
        {
            Extrema_ExtPC Projector = new Extrema_ExtPC(myPoint, Path, Precision.Confusion());
            DistMini(Projector, Path, ref Dist, ref PathParam);
            AngleMax = M_PI / 2;
        }
        else
        {
            PathParam = Path.FirstParameter();
            SecParam = myAdpSection.FirstParameter();

            double distaux = 0.0, taux = 0.0, alpha;
            gp_Pnt PonPath, PonSec, P;
            gp_Vec VRef = new gp_Vec(), dp1;
            VRef.SetXYZ(TheAxe.Direction().XYZ());

            Tangente(Path, PathParam, out PonPath, out dp1);
            PonSec = myAdpSection.Value(SecParam);
            Dist = PonPath.Distance(PonSec);
            if (Dist > Tol)
            { // On Cherche un meilleur point sur la section
                myExt.Perform(PonPath);
                if (myExt.IsDone())
                {
                    DistMini(myExt, myAdpSection, ref Dist, ref SecParam);
                    PonSec = myAdpSection.Value(SecParam);
                }
            }
            AngleMax = EvalAngle(VRef, dp1);
            if (isplan)
            {
                AngleMax = M_PI / 2 - AngleMax;
            }

            bool Trouve = false;
            int ii;

            if (isplan)
            {
                // (1.1) Distances Point-Plan
                double DistPlan;
                gp_Vec V1 = new gp_Vec(PonPath, TheAxe.Location());
                DistPlan = Math.Abs(V1.Dot(VRef));
                if (DistPlan <= IntTol)
                {
                    DistCenter = V1.Magnitude();
                }

                gp_Pnt Plast = Path.Value(Path.LastParameter());
                V1.SetXYZ(TheAxe.Location().XYZ() - Plast.XYZ());
                DistPlan = Math.Abs(V1.Dot(VRef));
                if (DistPlan <= IntTol)
                {
                    double aDist = V1.Magnitude();
                    if (aDist < DistCenter)
                    {
                        DistCenter = aDist;
                        PonPath = Plast;
                        PathParam = Path.LastParameter();
                    }
                }

                // (1.2) Intersection Plan-courbe
                gp_Ax3 axe = new gp_Ax3(TheAxe.Location(), TheAxe.Direction());
                Geom_Plane plan = new Geom_Plane(axe);
                GeomAdaptor_Surface adplan = new GeomAdaptor_Surface(plan);
                IntCurveSurface_HInter Intersector = new IntCurveSurface_HInter();
                Intersector.Perform(Path, adplan);
                if (Intersector.IsDone())
                {
                    double w;
                    double aDist;
                    for (ii = 1; ii <= Intersector.NbPoints(); ii++)
                    {
                        w = Intersector.Point(ii).W();
                        P = Path.Value(w);
                        aDist = P.Distance(TheAxe.Location());
                        if (aDist < DistCenter)
                        {
                            DistCenter = aDist;
                            PonPath = P;
                            PathParam = w;
                        }
                    }
                }
                if (!Intersector.IsDone() || Intersector.NbPoints() == 0)
                {
                    // Comparing the distances from the path's endpoints to the best matching plane of the
                    // profile.
                    gp_Pnt firstPoint = Path.Value(Path.FirstParameter());
                    gp_Pnt lastPoint = Path.Value(Path.LastParameter());
                    gp_Pln plane = plan.Pln();
                    double firstDistance = plane.SquareDistance(firstPoint);
                    double lastDistance = plane.SquareDistance(lastPoint);

                    if (((Math.Abs(firstDistance) < Precision.SquareConfusion())
                         && Math.Abs(lastDistance) < Precision.SquareConfusion())
                        || firstDistance < lastDistance)
                    {
                        PathParam = Path.FirstParameter();
                    }
                    else
                    {
                        PathParam = Path.LastParameter();
                        Tangente(Path, PathParam, out PonPath, out dp1);
                        PonSec = myAdpSection.Value(SecParam);
                        Dist = PonPath.Distance(PonSec);
                        if (Dist > Tol)
                        { // On Cherche un meilleur point sur la section
                            myExt.Perform(PonPath);
                            if (myExt.IsDone())
                            {
                                DistMini(myExt, myAdpSection, ref Dist, ref SecParam);
                                PonSec = myAdpSection.Value(SecParam);
                            }
                        }
                        AngleMax = EvalAngle(VRef, dp1);
                        AngleMax = M_PI / 2 - AngleMax;
                    }
                }
                // OCCT keeps an older version of this block commented out; it is not ported.
            }

            // Cas General
            if (!isplan)
            {
                // (2.1) Distance avec les extremites ...
                myExt.Perform(PonPath);
                if (myExt.IsDone())
                {
                    DistMini(myExt, myAdpSection, ref distaux, ref taux);
                    if (distaux < Dist)
                    {
                        Dist = distaux;
                        SecParam = taux;
                    }
                }
                Trouve = (Dist <= Tol);
                if (!Trouve)
                {
                    Tangente(Path, Path.LastParameter(), out P, out dp1);
                    alpha = EvalAngle(VRef, dp1);
                    myExt.Perform(P);
                    if (myExt.IsDone())
                    {
                        if (myExt.IsDone())
                        {
                            DistMini(myExt, myAdpSection, ref distaux, ref taux);
                            if (Choix(distaux, alpha))
                            {
                                Dist = distaux;
                                SecParam = taux;
                                AngleMax = alpha;
                                PonPath = P;
                                PathParam = Path.LastParameter();
                            }
                        }
                    }
                    Trouve = (Dist <= Tol);
                }

                // (2.2) Distance courbe-courbe
                if (!Trouve)
                {
                    Extrema_ExtCC Ext = new Extrema_ExtCC(Path, myAdpSection, Path.FirstParameter(), Path.LastParameter(),
                                                          myAdpSection.FirstParameter(), myAdpSection.LastParameter(),
                                                          Path.Resolution(Tol / 100), myAdpSection.Resolution(Tol / 100));
                    if (Ext.IsDone() && !Ext.IsParallel())
                    {
                        for (ii = 1; ii <= Ext.NbExt(); ii++)
                        {
                            distaux = Math.Sqrt(Ext.SquareDistance(ii));
                            Ext.Points(ii, out Extrema_POnCurv P1, out Extrema_POnCurv P2);
                            Tangente(Path, P1.Parameter(), out P, out dp1);
                            alpha = EvalAngle(VRef, dp1);
                            if (Choix(distaux, alpha))
                            {
                                Trouve = true;
                                Dist = distaux;
                                PathParam = P1.Parameter();
                                SecParam = P2.Parameter();
                                PonSec = P2.Value();
                                PonPath = P;
                                AngleMax = alpha;
                            }
                        }
                    }
                    if (!Trouve)
                    {
                        // Si l'on a toujours rien, on essai une distance point/path
                        // c'est la derniere chance.
                        Extrema_ExtPC PExt = new Extrema_ExtPC();
                        PExt.Initialize(Path, Path.FirstParameter(), Path.LastParameter(), Precision.Confusion());
                        PExt.Perform(PonSec);
                        if (PExt.IsDone())
                        {
                            // modified for OCC13595  Tue Oct 17 15:00:08 2006.BEGIN
                            DistMini(PExt, Path, ref distaux, ref taux);
                            // modified for OCC13595  Tue Oct 17 15:00:11 2006.END
                            Tangente(Path, taux, out P, out dp1);
                            alpha = EvalAngle(VRef, dp1);
                            if (Choix(distaux, alpha))
                            {
                                Dist = distaux;
                                PonPath = P;
                                AngleMax = alpha;
                                PathParam = taux;
                            }
                        }
                    }
                }
            }
        }

        done = true;
    }

    /// <summary>Computes the placement for the given parameter on the path.</summary>
    public void Perform(double Param, double Tol)
    {
        done = true;
        Adaptor3d_Curve Path;
        Path = myLaw.GetCurve()!;

        PathParam = Param;
        if (myIsPoint)
        {
            gp_Pnt PonPath = Path.Value(PathParam);
            Dist = PonPath.Distance(myPoint);
            AngleMax = M_PI / 2;
        }
        else
        {
            SecParam = myAdpSection.FirstParameter();

            gp_Pnt PonPath, PonSec;
            gp_Vec VRef = new gp_Vec(), dp1;
            VRef.SetXYZ(TheAxe.Direction().XYZ());

            Tangente(Path, PathParam, out PonPath, out dp1);
            PonSec = myAdpSection.Value(SecParam);
            Dist = PonPath.Distance(PonSec);
            if (Dist > Tol)
            { // On Cherche un meilleur point sur la section
                myExt.Perform(PonPath);
                if (myExt.IsDone())
                {
                    DistMini(myExt, myAdpSection, ref Dist, ref SecParam);
                    PonSec = myAdpSection.Value(SecParam);
                }
            }
            AngleMax = EvalAngle(VRef, dp1);
            if (isplan)
            {
                AngleMax = M_PI / 2 - AngleMax;
            }
        }

        done = true;
    }

    public bool IsDone() => done;

    public double ParameterOnPath() => PathParam;

    public double ParameterOnSection() => SecParam;

    public double Distance() => Dist;

    public double Angle() => AngleMax;

    /// <summary>
    /// The transformation from the section's coordinates to the location law's frame at the path parameter.
    /// WithTranslation forces the contact of the section and the path; WithCorrection rotates a planar section so
    /// that its normal is the law's tangent.
    /// </summary>
    public gp_Trsf Transformation(bool WithTranslation, bool WithCorrection = false)
    {
        gp_Vec V = new gp_Vec();
        gp_Mat M = new gp_Mat();
        gp_Dir DN = new gp_Dir(), D = new gp_Dir();
        // modified by NIZHNY-MKK  Fri Oct 17 15:27:07 2003
        gp_Pnt P = new gp_Pnt(0.0, 0.0, 0.0), PSection = new gp_Pnt(0.0, 0.0, 0.0);

        // Calcul des reperes
        myLaw.D0(PathParam, ref M, ref V);

        P.SetXYZ(V.XYZ());
        D.SetXYZ(M.Column(3));
        DN.SetXYZ(M.Column(1));
        gp_Ax3 Paxe = new gp_Ax3(P, D, DN);

        if (WithTranslation || WithCorrection)
        {
            if (myIsPoint)
            {
                PSection = myPoint;
            }
            else
            {
                PSection = mySection!.Value(SecParam);
            }
        }
        // modified by NIZHNY-MKK  Wed Oct  8 15:03:19 2003.BEGIN
        gp_Trsf Rot = new gp_Trsf();

        if (WithCorrection && !myIsPoint)
        {
            double angle;

            if (!isplan)
            {
                throw new Standard_Failure("Illegal usage: can't rotate non-planar profile");
            }

            gp_Dir ProfileNormal = TheAxe.Direction();
            gp_Dir SpineStartDir = Paxe.Direction();
            if (!ProfileNormal.IsParallel(SpineStartDir, Precision.Angular()))
            {
                gp_Dir DirAxeOfRotation = ProfileNormal ^ SpineStartDir;
                angle = ProfileNormal.AngleWithRef(SpineStartDir, DirAxeOfRotation);
                gp_Ax1 AxeOfRotation = new gp_Ax1(TheAxe.Location(), DirAxeOfRotation);
                Rot.SetRotation(AxeOfRotation, angle);
            }
            PSection.Transform(Rot);
        }
        // modified by NIZHNY-MKK  Wed Oct  8 15:03:21 2003.END

        if (WithTranslation)
        {
            P.ChangeCoord().SetLinearForm(-1, PSection.XYZ(), V.XYZ());
        }
        else
        {
            P.SetCoord(0.0, 0.0, 0.0);
        }

        gp_Ax3 Saxe = new gp_Ax3(P, gp.DZ(), gp.DX());

        // Transfo
        gp_Trsf Tf = new gp_Trsf();
        Tf.SetTransformation(Saxe, Paxe);

        if (WithCorrection)
        {
            Tf *= Rot;
        }

        return Tf;
    }

    /// <summary>
    /// The section in the frame of the location law (a new curve); WithTranslation forces the contact of the
    /// section and the path.
    /// </summary>
    public Geom_Curve Section(bool WithTranslation)
    {
        Geom_Curve TheSection = (Geom_Curve)mySection!.Copy();
        TheSection.Transform(Transformation(WithTranslation, false));
        return TheSection;
    }

    /// <summary>
    /// The section in the frame of the location law, rotated so that its normal is the law's (a new curve);
    /// WithTranslation forces the contact of the section and the path.
    /// </summary>
    public Geom_Curve ModifiedSection(bool WithTranslation)
    {
        Geom_Curve TheSection = (Geom_Curve)mySection!.Copy();
        TheSection.Transform(Transformation(WithTranslation, true));
        return TheSection;
    }

    // OCCT's private SectionAxis is defined but never called; it is internal here so that the port keeps it.
    internal void SectionAxis(in gp_Mat M, ref gp_Vec T, ref gp_Vec N, ref gp_Vec BN)
    {
        double Eps = 1.0e-10;
        gp_Dir D;
        gp_Vec PathNormal = new gp_Vec();
        GeomLProp_CLProps CP = new GeomLProp_CLProps(mySection!, SecParam, 2, Eps);
        if (CP.IsTangentDefined())
        {
            CP.Tangent(out D);
            T.SetXYZ(D.XYZ());
            T.Normalize();
            if (CP.Curvature() > Eps)
            {
                CP.Normal(out D);
                N.SetXYZ(D.XYZ());
            }
            else
            {
                // Cas ambigu, on essai de recuperer la normal a la trajectoire
                // Reste le probleme des points d'inflexions, qui n'est pas
                // bien traiter par LProp (pas de recuperation de la normal) :
                // A voir...
                PathNormal.SetXYZ(M.Column(1));
                PathNormal.Normalize();
                BN = T ^ PathNormal;
                if (BN.Magnitude() > Eps)
                {
                    BN.Normalize();
                }
                N = BN ^ T;
            }
        }
        else
        { // Cas indefinie, on prend le triedre
            // complet sur la trajectoire
            T.SetXYZ(M.Column(3));
            N.SetXYZ(M.Column(2));
        }
        BN = T ^ N;
    }

    // Whether the pair (dist, angle) is better than the current one.
    private bool Choix(double dist, double angle)
    {
        double evoldist, evolangle;
        evoldist = dist - Dist;
        evolangle = angle - AngleMax;

        // (1) Si la gain en distance est > que le gabarit, on prend
        if (evoldist < -Gabarit)
        {
            return true;
        }

        //  (2) si l'ecart en distance est de l'ordre du gabarit
        if (Math.Abs(evoldist) < Gabarit)
        {
            //  (2.1) si le gain en angle est important on garde
            if (evolangle > 0.5)
            {
                return true;
            }

            //  (2.2) si la variation d'angle est moderee on evalue
            //  une fonction de penalite
            if (Penalite(angle, dist / Gabarit) < Penalite(AngleMax, Dist / Gabarit))
            {
                return true;
            }
        }

        return false;
    }
}
