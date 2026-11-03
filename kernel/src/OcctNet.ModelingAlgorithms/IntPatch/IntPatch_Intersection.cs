// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_Intersection.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_Intersection.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_Intersection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using static Kerf.Geometry.SurfaceKind;
using static Occt.IntPatch_IType;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// The intersection of two surfaces (or the self-intersection of one) bounded by their domains: analytic pairs
/// through IntPatch_ImpImpIntersection (analytic lines converted to walking lines), a quadric and a parametric
/// surface through IntPatch_ImpPrmIntersection, parametric pairs through IntPatch_PrmPrmIntersection; then the
/// walking lines purged (IntPatch_WLineTool).
/// </summary>
/// <remarks>
/// A working object, as OCCT's: Perform fills it, and its lines are the caller's to edit (decision 609).
/// <see cref="Line"/> returns the line itself, <see cref="Point"/> a copy. The file-static functions of OCCT's
/// .cxx are private static methods (IntPatch_Intersection.Tools.cs).
/// </remarks>
public partial class IntPatch_Intersection
{
    private bool done;
    private bool empt;
    private bool tgte;
    private bool oppo;
    private readonly NCollection_Sequence<IntPatch_Point> spnt = new();
    private readonly NCollection_Sequence<IntPatch_Line> slin = new();
    private double myTolArc;
    private double myTolTang;
    private double myUVMaxStep;
    private double myFleche;
    private bool myIsStartPnt;
    private double myU1Start;
    private double myV1Start;
    private double myU2Start;
    private double myV2Start;

    /// <summary>The lines of the intersection (the sequence itself).</summary>
    public NCollection_Sequence<IntPatch_Line> SequenceOfLine() => slin;

    public IntPatch_Intersection()
    {
        done = false;
        empt = true;
        tgte = false;
        oppo = false;
        myTolArc = 0.0;
        myTolTang = 0.0;
        myUVMaxStep = 0.0;
        myFleche = 0.0;
        myIsStartPnt = false;
        myU1Start = 0.0;
        myV1Start = 0.0;
        myU2Start = 0.0;
        myV2Start = 0.0;
    }

    public IntPatch_Intersection(Adaptor3d_Surface S1, Adaptor3d_TopolTool D1, Adaptor3d_Surface S2,
                                 Adaptor3d_TopolTool D2, double TolArc, double TolTang)
        : this()
    {
        myTolArc = TolArc;
        myTolTang = TolTang;
        if (myTolArc < 1e-8)
        {
            myTolArc = 1e-8;
        }
        if (myTolTang < 1e-8)
        {
            myTolTang = 1e-8;
        }
        if (myTolArc > 0.5)
        {
            myTolArc = 0.5;
        }
        if (myTolTang > 0.5)
        {
            myTolTang = 0.5;
        }
        Perform(S1, D1, S2, D2, TolArc, TolTang);
    }

    public IntPatch_Intersection(Adaptor3d_Surface S1, Adaptor3d_TopolTool D1, double TolArc, double TolTang)
        : this()
    {
        myTolArc = TolArc;
        myTolTang = TolTang;
        Perform(S1, D1, TolArc, TolTang);
    }

    /// <summary>
    /// Sets the tolerances: TolArc and TolTang clamped to [1e-8, 0.5], Fleche (the deflection of walking lines) to
    /// [1e-3, 10], UVMaxStep (the largest step of the walking in parameters) at most 0.5.
    /// </summary>
    public void SetTolerances(double TolArc, double TolTang, double UVMaxStep, double Fleche)
    {
        myTolArc = TolArc;
        myTolTang = TolTang;
        myUVMaxStep = UVMaxStep;
        myFleche = Fleche;
        if (myTolArc < 1e-8)
        {
            myTolArc = 1e-8;
        }
        if (myTolTang < 1e-8)
        {
            myTolTang = 1e-8;
        }
        if (myTolArc > 0.5)
        {
            myTolArc = 0.5;
        }
        if (myTolTang > 0.5)
        {
            myTolTang = 0.5;
        }
        if (myFleche < 1.0e-3)
        {
            myFleche = 1e-3;
        }
        // if(myUVMaxStep<1.0e-3) myUVMaxStep=1e-3;
        if (myFleche > 10)
        {
            myFleche = 10;
        }
        if (myUVMaxStep > 0.5)
        {
            myUVMaxStep = 0.5;
        }
    }

    /// <summary>The self-intersection of S1 (parametric surfaces only; quadrics and extrusions of curves without
    /// self-intersection give an empty result).</summary>
    public void Perform(Adaptor3d_Surface S1, Adaptor3d_TopolTool D1, double TolArc, double TolTang)
    {
        myTolArc = TolArc;
        myTolTang = TolTang;
        if (myFleche == 0.0)
        {
            myFleche = 0.01;
        }
        if (myUVMaxStep == 0.0)
        {
            myUVMaxStep = 0.01;
        }

        done = true;
        spnt.Clear();
        slin.Clear();

        empt = true;
        tgte = false;
        oppo = false;

        switch (S1.GetType())
        {
            case Plane:
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
            case global::Kerf.Geometry.SurfaceKind.Sphere:
            case global::Kerf.Geometry.SurfaceKind.Cone:
            case global::Kerf.Geometry.SurfaceKind.Torus:
                break;
            case SurfaceOfExtrusion:
            {
                Direction3D aDirection = S1.Direction();
                CoordinateSystem anAxis = new(Constants.Origin(), aDirection);
                Adaptor3d_Curve aBasisCurve = S1.BasisCurve();
                ProjLib_ProjectOnPlane Projector = new(anAxis);
                Projector.Load(aBasisCurve, global::Kerf.Engine.Foundation.Precision.Confusion());
                GeomAdaptor_Curve aProjCurve = Projector.GetResult()!;
                Geom_Plane aPlane = new(anAxis);
                GeomAdaptor_Surface aGAHsurf = new(aPlane);
                ProjLib_ProjectedCurve aProjectedCurve = new(aGAHsurf, aProjCurve);
                ProjLib.MakePCurveOfType(aProjectedCurve, out Geom2d_Curve aPCurve);
                Geom2dAdaptor_Curve AC = new(aPCurve, aProjectedCurve.FirstParameter(),
                                             aProjectedCurve.LastParameter());
                Geom2dInt_GInter Intersector = new(AC, global::Kerf.Engine.Foundation.Precision.Confusion(), global::Kerf.Engine.Foundation.Precision.Confusion());
                if (Intersector.IsDone() && (Intersector.IsEmpty() || (AC.IsClosed() && Intersector.NbPoints() == 1)))
                {
                    break;
                }
            }
                goto default; // [[fallthrough]]
            default:
            {
                IntPatch_PrmPrmIntersection interpp = new();
                interpp.Perform(S1, D1, TolTang, TolArc, myFleche, myUVMaxStep);
                if (interpp.IsDone())
                {
                    done = true;
                    tgte = false;
                    empt = interpp.IsEmpty();
                    int nblm = interpp.NbLines();
                    for (int i = 1; i <= nblm; i++)
                    {
                        slin.Append(interpp.Line(i));
                    }
                }
            }
                break;
        }
    }

    //=================================================================================================

    /// <summary>
    /// Intersects S1 and S2 within their domains. isGeomInt false sends analytic pairs to the parametric
    /// intersector; theIsReqToKeepRLine keeps the restriction lines of analytic pairs; theIsReqToPostWLProc false
    /// skips the purging of the walking lines.
    /// </summary>
    public void Perform(Adaptor3d_Surface theS1, Adaptor3d_TopolTool theD1, Adaptor3d_Surface theS2,
                        Adaptor3d_TopolTool theD2, double TolArc, double TolTang, bool isGeomInt = true,
                        bool theIsReqToKeepRLine = false, bool theIsReqToPostWLProc = true)
    {
        myTolArc = TolArc;
        myTolTang = TolTang;
        if (myFleche <= global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            myFleche = 0.01;
        }
        if (myUVMaxStep <= global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            myUVMaxStep = 0.01;
        }

        done = false;
        spnt.Clear();
        slin.Clear();
        empt = true;
        tgte = false;
        oppo = false;

        SurfaceKind typs1 = theS1.GetType();
        SurfaceKind typs2 = theS2.GetType();

        // treatment of the cases with cone or torus
        bool TreatAsBiParametric = false;
        int bGeomGeom = 0;
        //
        if (typs1 == global::Kerf.Geometry.SurfaceKind.Cone || typs2 == global::Kerf.Geometry.SurfaceKind.Cone || typs1 == global::Kerf.Geometry.SurfaceKind.Torus || typs2 == global::Kerf.Geometry.SurfaceKind.Torus)
        {
            Axis3D aCTAx = new(), aGeomAx = new();
            SurfaceKind aCTType;
            bool bToCheck;
            //
            Adaptor3d_Surface aCTSurf = (typs1 == global::Kerf.Geometry.SurfaceKind.Cone || typs1 == global::Kerf.Geometry.SurfaceKind.Torus) ? theS1 : theS2;
            Adaptor3d_Surface aGeomSurf = (typs1 == global::Kerf.Geometry.SurfaceKind.Cone || typs1 == global::Kerf.Geometry.SurfaceKind.Torus) ? theS2 : theS1;
            //
            aCTType = aCTSurf.GetType();
            bToCheck = false;
            //
            if (typs1 == global::Kerf.Geometry.SurfaceKind.Cone || typs2 == global::Kerf.Geometry.SurfaceKind.Cone)
            {
                Cone aCon1 = (aCTType == global::Kerf.Geometry.SurfaceKind.Cone) ? aCTSurf.Cone() : aGeomSurf.Cone();
                double a1 = Math.Abs(aCon1.SemiAngle());
                bToCheck = (a1 < 0.02) || (a1 > 1.55);
                //
                if (typs1 == typs2)
                {
                    Cone aCon2 = aGeomSurf.Cone();
                    double a2 = Math.Abs(aCon2.SemiAngle());
                    bToCheck = bToCheck || (a2 < 0.02) || (a2 > 1.55);
                    //
                    if (a1 > 1.55 && a2 > 1.55)
                    { // quasi-planes: if same domain, treat as canonic
                        Axis3D A1 = aCon1.Axis(), A2 = aCon2.Axis();
                        if (A1.IsParallel(A2, global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            Point3D Apex1 = aCon1.Apex(), Apex2 = aCon2.Apex();
                            Plane3D Plan1 = new(Apex1, A1.Direction());
                            if (Plan1.Distance(Apex2) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                            {
                                bToCheck = false;
                            }
                        }
                    }
                }
                //
                TreatAsBiParametric = bToCheck;
                if (aCTType == global::Kerf.Geometry.SurfaceKind.Cone)
                {
                    aCTAx = aCon1.Axis();
                }
            }
            //
            if (typs1 == global::Kerf.Geometry.SurfaceKind.Torus || typs2 == global::Kerf.Geometry.SurfaceKind.Torus)
            {
                Torus aTor1 = (aCTType == global::Kerf.Geometry.SurfaceKind.Torus) ? aCTSurf.Torus() : aGeomSurf.Torus();
                bToCheck = aTor1.MajorRadius() > aTor1.MinorRadius();
                if (typs1 == typs2)
                {
                    Torus aTor2 = aGeomSurf.Torus();
                    // OCCT: this overload ignores aTor1's test for two tori (the list-of-points Perform combines
                    // both, and also accepts equal radii); ported as written.
                    bToCheck = aTor2.MajorRadius() > aTor2.MinorRadius();
                }
                //
                if (aCTType == global::Kerf.Geometry.SurfaceKind.Torus)
                {
                    aCTAx = aTor1.Axis();
                }
            }
            //
            CheckGeomGeom(ref bToCheck, ref bGeomGeom, ref TreatAsBiParametric, aCTAx, ref aGeomAx, aCTType, aCTSurf,
                          aGeomSurf);
        }
        //

        if (theD1.DomainIsInfinite() || theD2.DomainIsInfinite())
        {
            TreatAsBiParametric = false;
        }

        //  Modified by skv - Mon Sep 26 14:58:30 2005 Begin
        //   if(TreatAsBiParametric) { typs1 = typs2 = GeomAbs_BezierSurface; }
        if (TreatAsBiParametric)
        {
            if (typs1 == global::Kerf.Geometry.SurfaceKind.Cone && typs2 == Plane)
            {
                typs1 = BezierSurface; // Using Imp-Prm Intersector
            }
            else if (typs1 == Plane && typs2 == global::Kerf.Geometry.SurfaceKind.Cone)
            {
                typs2 = BezierSurface; // Using Imp-Prm Intersector
            }
            else
            {
                // Using Prm-Prm Intersector
                typs1 = BezierSurface;
                typs2 = BezierSurface;
            }
        }
        //  Modified by skv - Mon Sep 26 14:58:30 2005 End

        // Surface type definition
        int ts1 = SurfaceKind(typs1, bGeomGeom);
        int ts2 = SurfaceKind(typs2, bGeomGeom);
        //
        // treatment of the cases with torus and any other geom surface
        //
        // Possible intersection types: 1. ts1 == ts2 == 1 <Geom-Geom>
        //                              2. ts1 != ts2      <Geom-Param>
        //                              3. ts1 == ts2 == 0 <Param-Param>

        // Geom - Geom
        if (ts1 == ts2 && ts1 == 1)
        {
            NCollection_List<IntSurf_PntOn2S> ListOfPnts = new();
            ListOfPnts.Clear();
            if (isGeomInt)
            {
                GeomGeomPerfom(theS1, theD1, theS2, theD2, TolArc, TolTang, ListOfPnts, typs1, typs2,
                               theIsReqToKeepRLine);
            }
            else
            {
                ParamParamPerfom(theS1, theD1, theS2, theD2, TolArc, TolTang, ListOfPnts, typs1, typs2);
            }
        }

        // Geom - Param
        if (ts1 != ts2)
        {
            GeomParamPerfom(theS1, theD1, theS2, theD2, ts1 == 0, typs1, typs2);
        }

        // Param - Param
        if (ts1 == ts2 && ts1 == 0)
        {
            NCollection_List<IntSurf_PntOn2S> ListOfPnts = new();
            ListOfPnts.Clear();

            ParamParamPerfom(theS1, theD1, theS2, theD2, TolArc, TolTang, ListOfPnts, typs1, typs2);
        }

        if (!theIsReqToPostWLProc)
        {
            return;
        }

        PurgeWLines(theS1, theD1, theS2, theD2);
    }

    //=================================================================================================

    /// <summary>
    /// As the Perform above, starting from the given points too (ListOfPnts, for the parametric intersector).
    /// The list may gain the points' images across the seams of closed surfaces (decision 706).
    /// </summary>
    public void Perform(Adaptor3d_Surface theS1, Adaptor3d_TopolTool theD1, Adaptor3d_Surface theS2,
                        Adaptor3d_TopolTool theD2, double TolArc, double TolTang,
                        NCollection_List<IntSurf_PntOn2S> ListOfPnts, bool isGeomInt = true,
                        bool theIsReqToKeepRLine = false, bool theIsReqToPostWLProc = true)
    {
        myTolArc = TolArc;
        myTolTang = TolTang;
        if (myFleche <= global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            myFleche = 0.01;
        }
        if (myUVMaxStep <= global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            myUVMaxStep = 0.01;
        }
        done = false;
        spnt.Clear();
        slin.Clear();
        empt = true;
        tgte = false;
        oppo = false;

        SurfaceKind typs1 = theS1.GetType();
        SurfaceKind typs2 = theS2.GetType();
        //
        // treatment of the cases with cone or torus
        bool TreatAsBiParametric = false;
        int bGeomGeom = 0;
        //
        if (typs1 == global::Kerf.Geometry.SurfaceKind.Cone || typs2 == global::Kerf.Geometry.SurfaceKind.Cone || typs1 == global::Kerf.Geometry.SurfaceKind.Torus || typs2 == global::Kerf.Geometry.SurfaceKind.Torus)
        {
            Axis3D aCTAx = new(), aGeomAx = new();
            SurfaceKind aCTType;
            bool bToCheck;
            //
            Adaptor3d_Surface aCTSurf = (typs1 == global::Kerf.Geometry.SurfaceKind.Cone || typs1 == global::Kerf.Geometry.SurfaceKind.Torus) ? theS1 : theS2;
            Adaptor3d_Surface aGeomSurf = (typs1 == global::Kerf.Geometry.SurfaceKind.Cone || typs1 == global::Kerf.Geometry.SurfaceKind.Torus) ? theS2 : theS1;
            //
            aCTType = aCTSurf.GetType();
            bToCheck = false;
            //
            if (typs1 == global::Kerf.Geometry.SurfaceKind.Cone || typs2 == global::Kerf.Geometry.SurfaceKind.Cone)
            {
                Cone aCon1 = (aCTType == global::Kerf.Geometry.SurfaceKind.Cone) ? aCTSurf.Cone() : aGeomSurf.Cone();
                double a1 = Math.Abs(aCon1.SemiAngle());
                bToCheck = (a1 < 0.02) || (a1 > 1.55);
                //
                if (typs1 == typs2)
                {
                    Cone aCon2 = aGeomSurf.Cone();
                    double a2 = Math.Abs(aCon2.SemiAngle());
                    bToCheck = bToCheck || (a2 < 0.02) || (a2 > 1.55);
                    //
                    if (a1 > 1.55 && a2 > 1.55)
                    { // quasi-planes: if same domain, treat as canonic
                        Axis3D A1 = aCon1.Axis(), A2 = aCon2.Axis();
                        if (A1.IsParallel(A2, global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            Point3D Apex1 = aCon1.Apex(), Apex2 = aCon2.Apex();
                            Plane3D Plan1 = new(Apex1, A1.Direction());
                            if (Plan1.Distance(Apex2) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                            {
                                bToCheck = false;
                            }
                        }
                    }
                }
                //
                TreatAsBiParametric = bToCheck;
                if (aCTType == global::Kerf.Geometry.SurfaceKind.Cone)
                {
                    aCTAx = aCon1.Axis();
                }
            }
            //
            if (typs1 == global::Kerf.Geometry.SurfaceKind.Torus || typs2 == global::Kerf.Geometry.SurfaceKind.Torus)
            {
                Torus aTor1 = (aCTType == global::Kerf.Geometry.SurfaceKind.Torus) ? aCTSurf.Torus() : aGeomSurf.Torus();
                bToCheck = aTor1.MajorRadius() > aTor1.MinorRadius();
                if (typs1 == typs2)
                {
                    Torus aTor2 = aGeomSurf.Torus();
                    bToCheck = (bToCheck && (aTor2.MajorRadius() > aTor2.MinorRadius()))
                               || (Math.Abs(aTor1.MajorRadius() - aTor2.MajorRadius()) < TolTang
                                   && Math.Abs(aTor1.MinorRadius() - aTor2.MinorRadius()) < TolTang);
                }
                //
                if (aCTType == global::Kerf.Geometry.SurfaceKind.Torus)
                {
                    aCTAx = aTor1.Axis();
                }
            }
            //
            CheckGeomGeom(ref bToCheck, ref bGeomGeom, ref TreatAsBiParametric, aCTAx, ref aGeomAx, aCTType, aCTSurf,
                          aGeomSurf);
        }
        //

        if (theD1.DomainIsInfinite() || theD2.DomainIsInfinite())
        {
            TreatAsBiParametric = false;
        }

        if (TreatAsBiParametric)
        {
            // Using Prm-Prm Intersector
            typs1 = BezierSurface;
            typs2 = BezierSurface;
        }

        // Surface type definition
        int ts1 = SurfaceKind(typs1, bGeomGeom);
        int ts2 = SurfaceKind(typs2, bGeomGeom);
        //
        // Possible intersection types: 1. ts1 == ts2 == 1 <Geom-Geom>
        //                              2. ts1 != ts2      <Geom-Param>
        //                              3. ts1 == ts2 == 0 <Param-Param>

        if (!isGeomInt)
        {
            ParamParamPerfom(theS1, theD1, theS2, theD2, TolArc, TolTang, ListOfPnts, typs1, typs2);
        }
        else if (ts1 != ts2)
        {
            GeomParamPerfom(theS1, theD1, theS2, theD2, ts1 == 0, typs1, typs2);
        }
        else if (ts1 == 0)
        {
            ParamParamPerfom(theS1, theD1, theS2, theD2, TolArc, TolTang, ListOfPnts, typs1, typs2);
        }
        else if (ts1 == 1)
        {
            GeomGeomPerfom(theS1, theD1, theS2, theD2, TolArc, TolTang, ListOfPnts, typs1, typs2,
                           theIsReqToKeepRLine);
        }

        if (!theIsReqToPostWLProc)
        {
            return;
        }

        PurgeWLines(theS1, theD1, theS2, theD2);
    }

    // The common tail of OCCT's two Performs above, written out in each: the switch on the type of the surface
    // that isn't the cone or the torus, then the coaxial test (decision 671's pattern for verbatim repeats).
    private static void CheckGeomGeom(ref bool bToCheck, ref int bGeomGeom, ref bool TreatAsBiParametric,
                                      in Axis3D aCTAx, ref Axis3D aGeomAx, SurfaceKind aCTType,
                                      Adaptor3d_Surface aCTSurf, Adaptor3d_Surface aGeomSurf)
    {
        if (bToCheck)
        {
            Line3D aL1 = new(aCTAx);
            //
            switch (aGeomSurf.GetType())
            {
                case Plane:
                {
                    aGeomAx = aGeomSurf.Plane().Axis();
                    if (aCTType == global::Kerf.Geometry.SurfaceKind.Cone)
                    {
                        bGeomGeom = 1;
                        if (Math.Abs(aCTSurf.Cone().SemiAngle()) < 0.02)
                        {
                            double ps = Math.Abs(aCTAx.Direction().Dot(aGeomAx.Direction()));
                            if (ps < 0.015)
                            {
                                bGeomGeom = 0;
                            }
                        }
                    }
                    else
                    {
                        if (aCTAx.IsParallel(aGeomAx, global::Kerf.Engine.Foundation.Precision.Angular())
                            || (aCTAx.IsNormal(aGeomAx, global::Kerf.Engine.Foundation.Precision.Angular())
                                && (aGeomSurf.Plane().Distance(aCTAx.Location()) < global::Kerf.Engine.Foundation.Precision.Confusion())))
                        {
                            bGeomGeom = 1;
                        }
                    }
                    bToCheck = false;
                    break;
                }
                case global::Kerf.Geometry.SurfaceKind.Sphere:
                {
                    if (aL1.Distance(aGeomSurf.Sphere().Location()) < global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        bGeomGeom = 1;
                    }
                    bToCheck = false;
                    break;
                }
                case global::Kerf.Geometry.SurfaceKind.Cylinder:
                    aGeomAx = aGeomSurf.Cylinder().Axis();
                    break;
                case global::Kerf.Geometry.SurfaceKind.Cone:
                    aGeomAx = aGeomSurf.Cone().Axis();
                    break;
                case global::Kerf.Geometry.SurfaceKind.Torus:
                    aGeomAx = aGeomSurf.Torus().Axis();
                    break;
                default:
                    bToCheck = false;
                    break;
            }
            //
            if (bToCheck)
            {
                if (aCTAx.IsParallel(aGeomAx, global::Kerf.Engine.Foundation.Precision.Angular())
                    && (aL1.Distance(aGeomAx.Location()) <= global::Kerf.Engine.Foundation.Precision.Confusion()))
                {
                    bGeomGeom = 1;
                }
            }
            //
            if (bGeomGeom == 1)
            {
                TreatAsBiParametric = false;
            }
        }
    }

    // OCCT's two "Surface type definition" switches (repeated verbatim for ts1 and ts2).
    private static int SurfaceKind(SurfaceKind typs, int bGeomGeom)
    {
        int ts = 0;
        switch (typs)
        {
            case Plane:
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
            case global::Kerf.Geometry.SurfaceKind.Sphere:
            case global::Kerf.Geometry.SurfaceKind.Cone:
                ts = 1;
                break;
            case global::Kerf.Geometry.SurfaceKind.Torus:
                ts = bGeomGeom;
                break;
            default:
                break;
        }
        return ts;
    }

    // The purging loop at the end of OCCT's three two-surface Performs (repeated verbatim).
    private void PurgeWLines(Adaptor3d_Surface theS1, Adaptor3d_TopolTool theD1, Adaptor3d_Surface theS2,
                             Adaptor3d_TopolTool theD2)
    {
        for (int i = NCollection_Sequence<IntPatch_Line>.Lower(); i <= slin.Upper(); i++)
        {
            IntPatch_WLine? aWL = slin.Value(i) as IntPatch_WLine;

            if (aWL is null)
            {
                continue;
            }

            if (!aWL.IsPurgingAllowed())
            {
                continue;
            }

            IntPatch_WLine? aRW = IntPatch_WLineTool.ComputePurgedWLine(aWL, theS1, theS2, theD1, theD2);

            if (aRW is null)
            {
                continue;
            }

            slin.InsertAfter(i, aRW);
            slin.Remove(i);
        }
    }

    //=================================================================================================

    private void ParamParamPerfom(Adaptor3d_Surface theS1, Adaptor3d_TopolTool theD1, Adaptor3d_Surface theS2,
                                  Adaptor3d_TopolTool theD2, double TolArc, double TolTang,
                                  NCollection_List<IntSurf_PntOn2S> ListOfPnts, SurfaceKind typs1,
                                  SurfaceKind typs2)
    {
        IntPatch_PrmPrmIntersection interpp = new();
        //
        if (!theD1.DomainIsInfinite() && !theD2.DomainIsInfinite())
        {
            bool ClearFlag = true;
            if (!ListOfPnts.IsEmpty())
            {
                interpp.Perform(theS1, theD1, theS2, theD2, TolTang, TolArc, myFleche, myUVMaxStep, ListOfPnts);
                ClearFlag = false;
            }
            interpp.Perform(theS1, theD1, theS2, theD2, TolTang, TolArc, myFleche, myUVMaxStep, ClearFlag);
        }
        else if ((theD1.DomainIsInfinite()) ^ (theD2.DomainIsInfinite()))
        {
            if (theD1.DomainIsInfinite())
            {
                FUN_GetMinMaxXYZPnt(theS2, out Point3D pMinXYZ, out Point3D pMaxXYZ);
                double MU = Max(Math.Abs(theS2.FirstUParameter()), Math.Abs(theS2.LastUParameter()));
                double MV = Max(Math.Abs(theS2.FirstVParameter()), Math.Abs(theS2.LastVParameter()));
                double AP = Max(MU, MV);
                FUN_TrimInfSurf(pMinXYZ, pMaxXYZ, theS1, AP, out Adaptor3d_Surface? SS);
                interpp.Perform(SS!, theD1, theS2, theD2, TolTang, TolArc, myFleche, myUVMaxStep);
            }
            else
            {
                FUN_GetMinMaxXYZPnt(theS1, out Point3D pMinXYZ, out Point3D pMaxXYZ);
                double MU = Max(Math.Abs(theS1.FirstUParameter()), Math.Abs(theS1.LastUParameter()));
                double MV = Max(Math.Abs(theS1.FirstVParameter()), Math.Abs(theS1.LastVParameter()));
                double AP = Max(MU, MV);
                FUN_TrimInfSurf(pMinXYZ, pMaxXYZ, theS2, AP, out Adaptor3d_Surface? SS);
                interpp.Perform(theS1, theD1, SS!, theD2, TolTang, TolArc, myFleche, myUVMaxStep);
            }
        } //(theD1->DomainIsInfinite()) ^ (theD2->DomainIsInfinite())
        else
        {
            if (typs1 == OtherSurface || typs2 == OtherSurface)
            {
                done = false;
                return;
            }

            NCollection_Sequence<Point3D> sop = new();
            FUN_PL_Intersection(theS1, typs1, theS2, typs2, out bool IsPLInt, sop, out Vector3D v);

            if (IsPLInt)
            {
                if (sop.Length() > 0)
                {
                    for (int ip = 1; ip <= sop.Length(); ip++)
                    {
                        Line3D lin = new(sop.Value(ip), new Direction3D(v));
                        IntPatch_Line gl = new IntPatch_GLine(lin, false);
                        slin.Append(gl);
                    }

                    done = true;
                }
                else
                {
                    done = false;
                }

                return;
            } // 'COLLINEAR LINES'
            else
            {
                Adaptor3d_Surface nS1 = theS1;
                Adaptor3d_Surface nS2 = theS2;
                FUN_TrimBothSurf(theS1, typs1, theS2, typs2, 1.0e+8, ref nS1, ref nS2);
                interpp.Perform(nS1, theD1, nS2, theD2, TolTang, TolArc, myFleche, myUVMaxStep);
            } // 'NON - COLLINEAR LINES'
        } // both domains are infinite

        if (interpp.IsDone())
        {
            done = true;
            tgte = false;
            empt = interpp.IsEmpty();

            for (int i = 1; i <= interpp.NbLines(); i++)
            {
                if (interpp.Line(i).ArcType() != IntPatch_Walking)
                {
                    slin.Append(interpp.Line(i));
                }
            }

            for (int i = 1; i <= interpp.NbLines(); i++)
            {
                if (interpp.Line(i).ArcType() == IntPatch_Walking)
                {
                    slin.Append(interpp.Line(i));
                }
            }
        }
    }

    //=================================================================================================

    private void GeomGeomPerfom(Adaptor3d_Surface theS1, Adaptor3d_TopolTool theD1, Adaptor3d_Surface theS2,
                                Adaptor3d_TopolTool theD2, double TolArc, double TolTang,
                                NCollection_List<IntSurf_PntOn2S> ListOfPnts, SurfaceKind theTyps1,
                                SurfaceKind theTyps2, bool theIsReqToKeepRLine)
    {
        IntPatch_ImpImpIntersection interii = new(theS1, theD1, theS2, theD2, myTolArc, myTolTang,
                                                  theIsReqToKeepRLine);

        if (!interii.IsDone())
        {
            done = false;
            ParamParamPerfom(theS1, theD1, theS2, theD2, TolArc, TolTang, ListOfPnts, theTyps1, theTyps2);
            return;
        }

        done = (interii.GetStatus() == IntPatch_ImpImpIntersection.IntStatus.IntStatus_OK);
        empt = interii.IsEmpty();

        if (empt)
        {
            return;
        }

        const int aNbPointsInALine = 200;

        tgte = interii.TangentFaces();
        if (tgte)
        {
            oppo = interii.OppositeFaces();
        }

        bool isWLExist = false;
        IntPatch_ALineToWLine AToW = new(theS1, theS2, aNbPointsInALine);

        for (int i = 1; i <= interii.NbLines(); i++)
        {
            IntPatch_Line line = interii.Line(i);
            if (line.ArcType() == IntPatch_Analytic)
            {
                isWLExist = true;
                AToW.MakeWLine((IntPatch_ALine)line, slin);
            }
            else
            {
                if (line.ArcType() == IntPatch_Walking)
                {
                    ((IntPatch_WLine)line).EnablePurging(false);
                }

                if ((line.ArcType() != IntPatch_Restriction) || theIsReqToKeepRLine)
                {
                    slin.Append(line);
                }
            }
        }

        for (int i = 1; i <= interii.NbPnts(); i++)
        {
            spnt.Append(interii.Point(i));
        }

        if ((theTyps1 == global::Kerf.Geometry.SurfaceKind.Cylinder) && (theTyps2 == global::Kerf.Geometry.SurfaceKind.Cylinder))
        {
            IntPatch_WLineTool.JoinWLines(slin, spnt, theS1, theS2, TolTang);
        }

        if (isWLExist)
        {
            Box2d aBx1 = new(), aBx2 = new();
            double aU1F = theS1.FirstUParameter(), aU1L = theS1.LastUParameter(),
                   aV1F = theS1.FirstVParameter(), aV1L = theS1.LastVParameter(),
                   aU2F = theS2.FirstUParameter(), aU2L = theS2.LastUParameter(),
                   aV2F = theS2.FirstVParameter(), aV2L = theS2.LastVParameter();
            aBx1.Add(new Point2D(aU1F, aV1F));
            aBx1.Add(new Point2D(aU1L, aV1F));
            aBx1.Add(new Point2D(aU1L, aV1L));
            aBx1.Add(new Point2D(aU1F, aV1L));
            aBx2.Add(new Point2D(aU2F, aV2F));
            aBx2.Add(new Point2D(aU2L, aV2F));
            aBx2.Add(new Point2D(aU2L, aV2L));
            aBx2.Add(new Point2D(aU2F, aV2L));

            aBx1.Enlarge(global::Kerf.Engine.Foundation.Precision.PConfusion());
            aBx2.Enlarge(global::Kerf.Engine.Foundation.Precision.PConfusion());

            ReadOnlySpan<double> anArrOfPeriod = [theS1.IsUPeriodic() ? theS1.UPeriod() : 0.0,
                                                  theS1.IsVPeriodic() ? theS1.VPeriod() : 0.0,
                                                  theS2.IsUPeriodic() ? theS2.UPeriod() : 0.0,
                                                  theS2.IsVPeriodic() ? theS2.VPeriod() : 0.0];

            NCollection_List<Point3D> aListOfCriticalPoints = new();

            if (theS1.GetType() == global::Kerf.Geometry.SurfaceKind.Cone)
            {
                aListOfCriticalPoints.Append(theS1.Cone().Apex());
            }
            else if (theS1.GetType() == global::Kerf.Geometry.SurfaceKind.Sphere)
            {
                aListOfCriticalPoints.Append(theS1.Value(0.0, M_PI_2));
                aListOfCriticalPoints.Append(theS1.Value(0.0, -M_PI_2));
            }

            if (theS2.GetType() == global::Kerf.Geometry.SurfaceKind.Cone)
            {
                aListOfCriticalPoints.Append(theS2.Cone().Apex());
            }
            else if (theS2.GetType() == global::Kerf.Geometry.SurfaceKind.Sphere)
            {
                aListOfCriticalPoints.Append(theS2.Value(0.0, M_PI_2));
                aListOfCriticalPoints.Append(theS2.Value(0.0, -M_PI_2));
            }

            IntPatch_WLineTool.ExtendTwoWLines(slin, theS1, theS2, TolTang, anArrOfPeriod, aBx1, aBx2,
                                               aListOfCriticalPoints);
        }
    }

    //=================================================================================================

    private void GeomParamPerfom(Adaptor3d_Surface theS1, Adaptor3d_TopolTool theD1, Adaptor3d_Surface theS2,
                                 Adaptor3d_TopolTool theD2, bool isNotAnalitical, SurfaceKind typs1,
                                 SurfaceKind typs2)
    {
        IntPatch_ImpPrmIntersection interip = new();
        if (myIsStartPnt)
        {
            if (isNotAnalitical /*ts1 == 0*/)
            {
                interip.SetStartPoint(myU1Start, myV1Start);
            }
            else
            {
                interip.SetStartPoint(myU2Start, myV2Start);
            }
        }

        if (theD1.DomainIsInfinite() && theD2.DomainIsInfinite())
        {
            NCollection_Sequence<Point3D> sop = new();
            FUN_PL_Intersection(theS1, typs1, theS2, typs2, out bool IsPLInt, sop, out Vector3D v);

            if (IsPLInt)
            {
                if (sop.Length() > 0)
                {
                    for (int ip = 1; ip <= sop.Length(); ip++)
                    {
                        Line3D lin = new(sop.Value(ip), new Direction3D(v));
                        IntPatch_Line gl = new IntPatch_GLine(lin, false);
                        slin.Append(gl);
                    }

                    done = true;
                }
                else
                {
                    done = false;
                }

                return;
            }
            else
            {
                Adaptor3d_Surface nS1 = theS1;
                Adaptor3d_Surface nS2 = theS2;
                FUN_TrimBothSurf(theS1, typs1, theS2, typs2, 1.0e+5, ref nS1, ref nS2);
                interip.Perform(nS1, theD1, nS2, theD2, myTolArc, myTolTang, myFleche, myUVMaxStep);
            }
        }
        else
        {
            interip.Perform(theS1, theD1, theS2, theD2, myTolArc, myTolTang, myFleche, myUVMaxStep);
        }

        if (interip.IsDone())
        {
            done = true;
            empt = interip.IsEmpty();

            if (!empt)
            {
                int aNbLines = interip.NbLines();
                for (int i = 1; i <= aNbLines; i++)
                {
                    if (interip.Line(i).ArcType() != IntPatch_Walking)
                    {
                        slin.Append(interip.Line(i));
                    }
                }

                for (int i = 1; i <= aNbLines; i++)
                {
                    if (interip.Line(i).ArcType() == IntPatch_Walking)
                    {
                        slin.Append(interip.Line(i));
                    }
                }

                for (int i = 1; i <= interip.NbPnts(); i++)
                {
                    spnt.Append(interip.Point(i));
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>Intersects S1 and S2 from a start point (U1, V1) on S1 and (U2, V2) on S2: through the analytic
    /// dispatch (the point given to IntPatch_ImpPrmIntersection) when one surface is a plane, cylinder, sphere or
    /// cone, by marching from the point (IntPatch_PrmPrmIntersection) otherwise.</summary>
    public void Perform(Adaptor3d_Surface S1, Adaptor3d_TopolTool D1, Adaptor3d_Surface S2, Adaptor3d_TopolTool D2,
                        double U1, double V1, double U2, double V2, double TolArc, double TolTang)
    {
        myTolArc = TolArc;
        myTolTang = TolTang;
        if (myFleche == 0.0)
        {
            myFleche = 0.01;
        }
        if (myUVMaxStep == 0.0)
        {
            myUVMaxStep = 0.01;
        }

        done = false;
        spnt.Clear();
        slin.Clear();

        empt = true;
        tgte = false;
        oppo = false;

        SurfaceKind typs1 = S1.GetType();
        SurfaceKind typs2 = S2.GetType();

        if (typs1 == Plane || typs1 == global::Kerf.Geometry.SurfaceKind.Cylinder || typs1 == global::Kerf.Geometry.SurfaceKind.Sphere || typs1 == global::Kerf.Geometry.SurfaceKind.Cone
            || typs2 == Plane || typs2 == global::Kerf.Geometry.SurfaceKind.Cylinder || typs2 == global::Kerf.Geometry.SurfaceKind.Sphere
            || typs2 == global::Kerf.Geometry.SurfaceKind.Cone)
        {
            myIsStartPnt = true;
            myU1Start = U1;
            myV1Start = V1;
            myU2Start = U2;
            myV2Start = V2;
            Perform(S1, D1, S2, D2, TolArc, TolTang);
            myIsStartPnt = false;
        }
        else
        {
            IntPatch_PrmPrmIntersection interpp = new();
            interpp.Perform(S1, D1, S2, D2, U1, V1, U2, V2, TolTang, TolArc, myFleche, myUVMaxStep);
            if (interpp.IsDone())
            {
                done = true;
                tgte = false;
                empt = interpp.IsEmpty();
                int nblm = interpp.NbLines();
                int i = 1;
                for (; i <= nblm; i++)
                {
                    slin.Append(interpp.Line(i));
                }
            }
        }

        PurgeWLines(S1, D1, S2, D2);
    }

    /// <summary>
    /// Writes the dump of the intersection. OCCT's body is compiled only with DUMPOFIntPatch_Intersection (a debug
    /// define), so a release build, which the port follows, writes nothing.
    /// </summary>
    public void Dump(int Mode, Adaptor3d_Surface S1, Adaptor3d_TopolTool D1, Adaptor3d_Surface S2,
                     Adaptor3d_TopolTool D2, TextWriter theOut)
    {
        _ = Mode;
        _ = S1;
        _ = D1;
        _ = S2;
        _ = D2;
        _ = theOut;
    }

    // ----- IntPatch_Intersection.lxx -----

    /// <summary>True if the computation succeeded.</summary>
    public bool IsDone() => done;

    /// <summary>True if there is no intersection. Throws StdFail_NotDone if not done.</summary>
    public bool IsEmpty()
    {
        if (!done)
        {
            throw new NotDoneException();
        }
        return empt;
    }

    /// <summary>True if the surfaces are tangent (identical within the tolerances). Throws StdFail_NotDone if not
    /// done.</summary>
    public bool TangentFaces()
    {
        if (!done)
        {
            throw new NotDoneException();
        }
        return tgte;
    }

    /// <summary>True if the tangent surfaces' normals are opposite. Throws StdFail_NotDone if not done,
    /// Standard_DomainError if the surfaces aren't tangent.</summary>
    public bool OppositeFaces()
    {
        if (!done)
        {
            throw new NotDoneException();
        }
        if (!tgte)
        {
            throw new DomainErrorException();
        }
        return oppo;
    }

    /// <summary>The number of isolated points. Throws StdFail_NotDone if not done.</summary>
    public int NbPnts()
    {
        if (!done)
        {
            throw new NotDoneException();
        }
        return spnt.Length();
    }

    /// <summary>The isolated point of index Index (a copy). Throws StdFail_NotDone if not done.</summary>
    public IntPatch_Point Point(int Index)
    {
        if (!done)
        {
            throw new NotDoneException();
        }
        return spnt[Index];
    }

    /// <summary>The number of intersection lines. Throws StdFail_NotDone if not done.</summary>
    public int NbLines()
    {
        if (!done)
        {
            throw new NotDoneException();
        }
        return slin.Length();
    }

    /// <summary>The line of index Index (the line itself). Throws StdFail_NotDone if not done.</summary>
    public IntPatch_Line Line(int Index)
    {
        if (!done)
        {
            throw new NotDoneException();
        }
        return slin[Index];
    }

    //=================================================================================================

    /// <summary>
    /// True if a boundary of theS1 is degenerate (a pole or an apex); theDist is then the smallest distance from
    /// such a point to theS2, Precision.Infinite() otherwise.
    /// </summary>
    public static bool CheckSingularPoints(Adaptor3d_Surface theS1, Adaptor3d_TopolTool theD1,
                                           Adaptor3d_Surface theS2, out double theDist)
    {
        theDist = global::Kerf.Engine.Foundation.Precision.Infinite();
        bool isSingular = false;
        if (ReferenceEquals(theS1, theS2))
        {
            return isSingular;
        }
        //
        const int aNbBndPnts = 5;
        double aTol = global::Kerf.Engine.Foundation.Precision.Confusion();
        int i;
        theD1.Init();
        bool isU = true;
        for (; theD1.More(); theD1.Next())
        {
            Adaptor2d_Curve2d aBnd = theD1.Value();
            double pinf = aBnd.FirstParameter(), psup = aBnd.LastParameter();
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(pinf) || global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(psup))
            {
                continue;
            }
            double t, dt = (psup - pinf) / (aNbBndPnts - 1);
            aBnd.D1((pinf + psup) / 2.0, out Point2D aP1, out Vector2D aDir);
            isU = Math.Abs(aDir.X()) > Math.Abs(aDir.Y());
            Point3D aPP1 = new();
            double aD1NormMax = 0.0;
            Xyz aPmid = new(0.0, 0.0, 0.0);
            int aNb = 0;
            for (t = pinf; t <= psup; t += dt)
            {
                aP1 = aBnd.Value(t);
                theS1.D1(aP1.X(), aP1.Y(), out aPP1, out Vector3D aDU, out Vector3D aDV);
                if (isU)
                {
                    aD1NormMax = Max(aD1NormMax, aDU.Magnitude());
                }
                else
                {
                    aD1NormMax = Max(aD1NormMax, aDV.Magnitude());
                }

                aPmid += aPP1.XYZ();
                aNb++;

                if (aD1NormMax > aTol)
                {
                    break;
                }
            }

            if (aD1NormMax <= aTol)
            {
                // Singular point aPP1;
                aPmid /= aNb;
                aPP1.SetXYZ(aPmid);
                double aTolU = global::Kerf.Engine.Foundation.Precision.PConfusion(), aTolV = global::Kerf.Engine.Foundation.Precision.PConfusion();
                Extrema_ExtPS aProj = new(aPP1, theS2, aTolU, aTolV, Extrema_ExtFlag.Extrema_ExtFlag_MIN);

                if (aProj.IsDone())
                {
                    int aNbExt = aProj.NbExt();
                    for (i = 1; i <= aNbExt; ++i)
                    {
                        theDist = Min(theDist, aProj.SquareDistance(i));
                    }
                }
            }
        }
        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(theDist))
        {
            theDist = Math.Sqrt(theDist);
            isSingular = true;
        }

        return isSingular;
    }

    //=================================================================================================

    /// <summary>The largest step in parameters for the walking: 1e-4 when a singular point of one surface is
    /// within 1e-5 (but farther than Confusion) of the other, 1e-3 otherwise.</summary>
    public static double DefineUVMaxStep(Adaptor3d_Surface theS1, Adaptor3d_TopolTool theD1, Adaptor3d_Surface theS2,
                                         Adaptor3d_TopolTool theD2)
    {
        double anUVMaxStep = 0.001;
        double aTolMin = global::Kerf.Engine.Foundation.Precision.Confusion(), aTolMax = 1.0e-5;
        if (!ReferenceEquals(theS1, theS2))
        {
            bool isSing1 = CheckSingularPoints(theS1, theD1, theS2, out double aDistToSing1);
            if (isSing1)
            {
                if (aDistToSing1 > aTolMin && aDistToSing1 < aTolMax)
                {
                    anUVMaxStep = 0.0001;
                }
                else
                {
                    isSing1 = false;
                }
            }
            if (!isSing1)
            {
                bool isSing2 = CheckSingularPoints(theS2, theD2, theS1, out double aDistToSing2);
                if (isSing2)
                {
                    if (aDistToSing2 > aTolMin && aDistToSing2 < aTolMax)
                    {
                        anUVMaxStep = 0.0001;
                    }
                }
            }
        }
        return anUVMaxStep;
    }

    //=================================================================================================

    private static void splitCone(Adaptor3d_Surface theS, Adaptor3d_TopolTool theD, double theTol,
                                  NCollection_DynamicArray<Adaptor3d_Surface> theVecHS)
    {
        if (theS.GetType() != global::Kerf.Geometry.SurfaceKind.Cone)
        {
            throw new NoSuchObjectException("IntPatch_Intersection : Surface is not Cone");
        }

        Cone aCone = theS.Cone();

        Adaptor3d_TopolTool.GetConeApexParam(aCone, out double aU0, out double aV0);

        TopAbs_State aState = theD.Classify(new Point2D(aU0, aV0), theTol);

        if (aState == TopAbs_IN || aState == TopAbs_ON)
        {
            Adaptor3d_Surface aHSDn = theS.VTrim(theS.FirstVParameter(), aV0, global::Kerf.Engine.Foundation.Precision.PConfusion());
            Adaptor3d_Surface aHSUp = theS.VTrim(aV0, theS.LastVParameter(), global::Kerf.Engine.Foundation.Precision.PConfusion());

            theVecHS.Append(aHSDn);
            theVecHS.Append(aHSUp);
        }
        else
        {
            theVecHS.Append(theS);
        }
    }

    //=======================================================================
    // function : PrepareSurfaces
    // purpose  : Prepares surfaces for intersection
    //=======================================================================
    /// <summary>
    /// Prepares the surfaces for the intersection: a cone whose semi-angle is within theTol of π/2 is split at its
    /// apex when the apex is in its domain; every other surface is appended as is.
    /// </summary>
    public static void PrepareSurfaces(Adaptor3d_Surface theS1, Adaptor3d_TopolTool theD1, Adaptor3d_Surface theS2,
                                       Adaptor3d_TopolTool theD2, double theTol,
                                       NCollection_DynamicArray<Adaptor3d_Surface> theVecHS1,
                                       NCollection_DynamicArray<Adaptor3d_Surface> theVecHS2)
    {
        if ((theS1.GetType() == global::Kerf.Geometry.SurfaceKind.Cone) && (Math.Abs(M_PI / 2.0 - Math.Abs(theS1.Cone().SemiAngle())) < theTol))
        {
            splitCone(theS1, theD1, theTol, theVecHS1);
        }
        else
        {
            theVecHS1.Append(theS1);
        }

        if ((theS2.GetType() == global::Kerf.Geometry.SurfaceKind.Cone) && (Math.Abs(M_PI / 2.0 - Math.Abs(theS2.Cone().SemiAngle())) < theTol))
        {
            splitCone(theS2, theD2, theTol, theVecHS2);
        }
        else
        {
            theVecHS2.Append(theS2);
        }
    }
}
