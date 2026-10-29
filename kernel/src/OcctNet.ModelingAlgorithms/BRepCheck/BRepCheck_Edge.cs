// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck_Edge.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck_Edge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BRepCheck_Status;
using static Occt.gp_TrsfForm;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// The checks of an edge: one 3D curve and a valid range (<see cref="Minimum"/>); in a face, its same-parameter and
/// same-range flags and the distance between its 3D curve and its pcurves (<see cref="InContext"/>); in a solid, that
/// it bounds two faces.
/// </summary>
public class BRepCheck_Edge : BRepCheck_Result
{
    private const int NCONTROL = 23;

    private BRep_CurveRepresentation? myCref;
    private Adaptor3d_Curve? myHCurve;
    private bool myGctrl;
    private bool myIsExactMethod;

    /// <summary>Checks the edge on its own (<see cref="Minimum"/>).</summary>
    public BRepCheck_Edge(in TopoDS_Edge E)
    {
        Init(E);
        myGctrl = true;
        myIsExactMethod = false;
    }

    /// <summary>
    /// Checks the flags and the 3D representation of the edge, and chooses the reference curve: the first 3D curve,
    /// or else the first curve on a surface.
    /// </summary>
    public override void Minimum()
    {
        if (!myMin)
        {
            var aNewList = new NCollection_List<BRepCheck_Status>();
            NCollection_List<BRepCheck_Status> lst = myMap.Bound(myShape, aNewList);
            myCref = null;

            // Existence et unicite d`une representation 3D
            var TE = (BRep_TEdge)myShape.TShape()!;
            var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(TE.Curves());
            bool exist = false;
            bool unique = true;
            // Search for a 3D reference. If no existent one, creates it with the
            // first encountered CurveOnSurf; if multiple, chooses the first one...

            bool Degenerated = TE.Degenerated();
            bool SameParameter = TE.SameParameter();
            bool SameRange = TE.SameRange();
            if (!SameRange && SameParameter)
            {
                BRepCheck.Add(lst, BRepCheck_InvalidSameParameterFlag);
            }

            while (itcr.More())
            {
                BRep_CurveRepresentation cr = itcr.Value();
                if (cr.IsCurve3D())
                {
                    if (!exist)
                    {
                        exist = true;
                    }
                    else
                    {
                        unique = false;
                    }

                    if (myCref is null && cr.Curve3D() is not null)
                    {
                        myCref = cr;
                    }
                }

                itcr.Next();
            }

            if (!exist)
            {
                BRepCheck.Add(lst, BRepCheck_No3DCurve);
                // myCref est nulle
            }
            else if (!unique)
            {
                BRepCheck.Add(lst, BRepCheck_Multiple3DCurve);
            }

            if (myCref is null && !Degenerated)
            {
                itcr.Initialize(TE.Curves());
                while (itcr.More())
                {
                    BRep_CurveRepresentation cr = itcr.Value();
                    if (cr.IsCurveOnSurface())
                    {
                        myCref = cr;
                        break;
                    }

                    itcr.Next();
                }
            }
            else if (myCref is not null && Degenerated)
            {
                BRepCheck.Add(lst, BRepCheck_InvalidDegeneratedFlag);
            }

            if (myCref is not null)
            {
                var GCref = (BRep_GCurve)myCref;
                double eps = Precision.PConfusion();
                GCref.Range(out double First, out double Last);
                if (Last <= First)
                {
                    myCref = null;
                    BRepCheck.Add(lst, BRepCheck_InvalidRange);
                }
                else
                {
                    if (myCref.IsCurve3D())
                    {
                        // eap 6 Jun 2002 occ332
                        // better transform C3d instead of transforming Surf upto C3d initial location,
                        // on transformed BSpline surface 'same parameter' may seem wrong
                        TopLoc_Location L = myShape.Location() * myCref.Location();
                        var C3d = (Geom_Curve)myCref.Curve3D()!.Transformed(L.Transformation());
                        bool IsPeriodic = C3d.IsPeriodic();
                        double aPeriod = RealLast();
                        if (IsPeriodic)
                        {
                            aPeriod = C3d.Period();
                        }

                        double f = C3d.FirstParameter(), l = C3d.LastParameter();
                        if (C3d.GetType() == typeof(Geom_TrimmedCurve))
                        {
                            Geom_Curve aC = ((Geom_TrimmedCurve)C3d).BasisCurve();
                            f = aC.FirstParameter();
                            l = aC.LastParameter();
                            IsPeriodic = aC.IsPeriodic();
                            if (IsPeriodic)
                            {
                                aPeriod = aC.Period();
                            }
                        }

                        if (IsPeriodic && (Last - First > aPeriod + eps))
                        {
                            myCref = null;
                            BRepCheck.Add(lst, BRepCheck_InvalidRange);
                        }
                        else if (!IsPeriodic && (First < f - eps || Last > l + eps))
                        {
                            myCref = null;
                            BRepCheck.Add(lst, BRepCheck_InvalidRange);
                        }
                        else
                        {
                            var GAC3d = new GeomAdaptor_Curve(C3d,
                                                              C3d.TransformedParameter(First, L.Transformation()),
                                                              C3d.TransformedParameter(Last, L.Transformation()));
                            myHCurve = GAC3d;
                        }
                    }
                    else
                    {
                        // curve on surface
                        Geom_Surface Sref = myCref.Surface()!;
                        Sref = (Geom_Surface)Sref.Transformed(myCref.Location().Transformation());
                        Geom2d_Curve PCref = myCref.PCurve()!;
                        bool IsPeriodic = PCref.IsPeriodic();
                        double aPeriod = RealLast();
                        if (IsPeriodic)
                        {
                            aPeriod = PCref.Period();
                        }

                        double f = PCref.FirstParameter(), l = PCref.LastParameter();
                        if (PCref.GetType() == typeof(Geom2d_TrimmedCurve))
                        {
                            Geom2d_Curve aC = ((Geom2d_TrimmedCurve)PCref).BasisCurve();
                            f = aC.FirstParameter();
                            l = aC.LastParameter();
                            IsPeriodic = aC.IsPeriodic();
                            if (IsPeriodic)
                            {
                                aPeriod = aC.Period();
                            }
                        }

                        if (IsPeriodic && (Last - First > aPeriod + eps))
                        {
                            myCref = null;
                            BRepCheck.Add(lst, BRepCheck_InvalidRange);
                        }
                        else if (!IsPeriodic && (First < f - eps || Last > l + eps))
                        {
                            myCref = null;
                            BRepCheck.Add(lst, BRepCheck_InvalidRange);
                        }
                        else
                        {
                            var GAHSref = new GeomAdaptor_Surface(Sref);
                            var GHPCref = new Geom2dAdaptor_Curve(PCref, First, Last);
                            var ACSref = new Adaptor3d_CurveOnSurface(GHPCref, GAHSref);
                            myHCurve = ACSref;
                        }
                    }
                }
            }

            if (lst.IsEmpty())
            {
                lst.Append(BRepCheck_NoError);
            }

            myMin = true;
        }
    }

    /// <summary>
    /// Checks the edge in a wire (nothing), a face (flags, range and the distance between the reference curve and
    /// the pcurves on the face, or the projection on a plane without pcurve) or a solid (bounds two faces).
    /// </summary>
    public override void InContext(in TopoDS_Shape S)
    {
        NCollection_List<BRepCheck_Status> aHList;
        {
            bool aLocked = myIsParallel;
            if (aLocked)
            {
                myMutex.Enter();
            }

            try
            {
                if (myMap.IsBound(S))
                {
                    return;
                }

                var aNewList = new NCollection_List<BRepCheck_Status>();
                aHList = myMap.Bound(S, aNewList);
            }
            finally
            {
                if (aLocked)
                {
                    myMutex.Exit();
                }
            }
        }

        NCollection_List<BRepCheck_Status> lst = aHList;

        var TE = (BRep_TEdge)myShape.TShape()!;
        double Tol = BRep_Tool.Tolerance(TopoDS.Edge(myShape));

        TopAbs_ShapeEnum styp = S.ShapeType();
        var exp = new TopExp_Explorer(S, TopAbs_EDGE);
        for (; exp.More(); exp.Next())
        {
            if (exp.Current().IsSame(myShape))
            {
                break;
            }
        }

        if (!exp.More())
        {
            BRepCheck.Add(lst, BRepCheck_SubshapeNotInShape);
            return;
        }

        switch (styp)
        {
            case TopAbs_WIRE:
            {
            }

                break;

            case TopAbs_FACE:
                if (myCref is not null)
                {
                    bool SameParameter = TE.SameParameter();
                    bool SameRange = TE.SameRange();
                    //  Modified by skv - Tue Apr 27 11:48:13 2004 Begin
                    if (!SameParameter || !SameRange)
                    {
                        if (!SameParameter)
                        {
                            BRepCheck.Add(lst, BRepCheck_InvalidSameParameterFlag);
                        }

                        if (!SameRange)
                        {
                            BRepCheck.Add(lst, BRepCheck_InvalidSameRangeFlag);
                        }

                        return;
                    }

                    //  Modified by skv - Tue Apr 27 11:48:14 2004 End
                    // myCref is not null only when Minimum built myHCurve
                    double First = myHCurve!.FirstParameter();
                    double Last = myHCurve.LastParameter();
                    // OCCT.NET: in parallel mode the analyzer may check this edge in several faces at once, and the
                    // reference adaptor's evaluation cache isn't thread-safe; each call evaluates its own copy
                    // (OCCT shares the adaptor: occt-issues.md)
                    Adaptor3d_Curve aHCurve = myIsParallel ? myHCurve.ShallowCopy() : myHCurve;

                    var TF = (BRep_TFace)S.TShape()!;
                    TopLoc_Location Floc = S.Location();
                    TopLoc_Location TFloc = TF.Location();
                    Geom_Surface? Su = TF.Surface();
                    TopLoc_Location L = (Floc * TFloc).Predivided(myShape.Location());
                    TopLoc_Location LE = myShape.Location() * myCref.Location();
                    gp_Trsf Etrsf = LE.Transformation();
                    bool pcurvefound = false;

                    var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(TE.Curves());
                    double eps = Precision.PConfusion();
                    bool toRunParallel = myIsParallel;
                    while (itcr.More())
                    {
                        BRep_CurveRepresentation cr = itcr.Value();
                        if (cr != myCref && cr.IsCurveOnSurface(Su, L))
                        {
                            pcurvefound = true;
                            var GC = (BRep_GCurve)cr;
                            GC.Range(out double f, out double l);
                            double ff = f, ll = l;
                            if (myCref.IsCurve3D())
                            {
                                ff = myCref.Curve3D()!.TransformedParameter(f, Etrsf);
                                ll = myCref.Curve3D()!.TransformedParameter(l, Etrsf);
                            }

                            // gka OCC
                            //  Modified by skv - Tue Apr 27 11:50:35 2004 Begin
                            if (Math.Abs(ff - First) > eps || Math.Abs(ll - Last) > eps)
                            {
                                BRepCheck.Add(lst, BRepCheck_InvalidSameRangeFlag);
                                BRepCheck.Add(lst, BRepCheck_InvalidSameParameterFlag);
                            }

                            //  Modified by skv - Tue Apr 27 11:50:37 2004 End
                            //
                            Geom2d_Curve pc = cr.PCurve()!;
                            bool IsPeriodic = pc.IsPeriodic();
                            double aPeriod = RealLast();
                            if (IsPeriodic)
                            {
                                aPeriod = pc.Period();
                            }

                            double fp = pc.FirstParameter(), lp = pc.LastParameter();
                            if (pc.GetType() == typeof(Geom2d_TrimmedCurve))
                            {
                                Geom2d_Curve aC = ((Geom2d_TrimmedCurve)pc).BasisCurve();
                                fp = aC.FirstParameter();
                                lp = aC.LastParameter();
                                IsPeriodic = aC.IsPeriodic();
                                if (IsPeriodic)
                                {
                                    aPeriod = aC.Period();
                                }
                            }

                            if (IsPeriodic && (l - f > aPeriod + eps))
                            {
                                BRepCheck.Add(lst, BRepCheck_InvalidRange);
                                return;
                            }
                            else if (!IsPeriodic && (f < fp - eps || l > lp + eps))
                            {
                                BRepCheck.Add(lst, BRepCheck_InvalidRange);
                                return;
                            }

                            if (myGctrl)
                            {
                                var Sb = (Geom_Surface)Su!.Transformed((Floc * TFloc).Transformation());
                                Geom2d_Curve PC = cr.PCurve()!;
                                var GAHS = new GeomAdaptor_Surface(Sb);
                                var GHPC = new Geom2dAdaptor_Curve(PC, f, l);
                                var ACS = new Adaptor3d_CurveOnSurface(GHPC, GAHS);

                                var aValidateEdge = new BRepLib_ValidateEdge(aHCurve, ACS, SameParameter);
                                aValidateEdge.SetExitIfToleranceExceeded(Tol);
                                aValidateEdge.SetExactMethod(myIsExactMethod);
                                aValidateEdge.SetParallel(toRunParallel);
                                aValidateEdge.Process();
                                if (!aValidateEdge.IsDone() || !aValidateEdge.CheckTolerance(Tol))
                                {
                                    if (cr.IsCurveOnClosedSurface())
                                    {
                                        BRepCheck.Add(lst, BRepCheck_InvalidCurveOnClosedSurface);
                                    }
                                    else
                                    {
                                        BRepCheck.Add(lst, BRepCheck_InvalidCurveOnSurface);
                                    }

                                    //  Modified by skv - Tue Apr 27 11:53:00 2004 Begin
                                    BRepCheck.Add(lst, BRepCheck_InvalidSameParameterFlag);
                                    //  Modified by skv - Tue Apr 27 11:53:01 2004 End
                                }

                                if (cr.IsCurveOnClosedSurface())
                                {
                                    GHPC.Load(cr.PCurve2()!, f, l); // same bounds
                                    ACS.Load(GHPC, GAHS); // sans doute inutile

                                    var aValidateEdgeOnClosedSurf = new BRepLib_ValidateEdge(aHCurve, ACS, SameParameter);
                                    aValidateEdgeOnClosedSurf.SetExitIfToleranceExceeded(Tol);
                                    aValidateEdgeOnClosedSurf.SetExactMethod(myIsExactMethod);
                                    aValidateEdgeOnClosedSurf.SetParallel(toRunParallel);
                                    aValidateEdgeOnClosedSurf.Process();
                                    if (!aValidateEdgeOnClosedSurf.IsDone()
                                        || !aValidateEdgeOnClosedSurf.CheckTolerance(Tol))
                                    {
                                        BRepCheck.Add(lst, BRepCheck_InvalidCurveOnClosedSurface);
                                        //  Modified by skv - Tue Apr 27 11:53:20 2004 Begin
                                        if (SameParameter)
                                        {
                                            BRepCheck.Add(lst, BRepCheck_InvalidSameParameterFlag);
                                        }

                                        //  Modified by skv - Tue Apr 27 11:53:23 2004 End
                                    }
                                }
                            }
                        }

                        itcr.Next();
                    }

                    if (!pcurvefound)
                    {
                        Geom_Plane? P;
                        if (Su is Geom_RectangularTrimmedSurface aRTS && Su.GetType() == typeof(Geom_RectangularTrimmedSurface))
                        {
                            P = aRTS.BasisSurface() as Geom_Plane;
                        }
                        else
                        {
                            P = Su as Geom_Plane;
                        }

                        if (P is null)
                        {
                            // not a plane
                            BRepCheck.Add(lst, BRepCheck_NoCurveOnSurface);
                        }
                        else
                        {
                            // on fait la projection a la volee, comme BRep_Tool
                            // plan en position
                            if (myGctrl)
                            {
                                P = (Geom_Plane)P.Transformed((Floc * TFloc).Transformation()); // eap occ332
                                // on projette Cref sur ce plan
                                var GAHS = new GeomAdaptor_Surface(P);

                                // Dub - Normalement myHCurve est une GeomAdaptor_Curve
                                // OCCT down-casts and dereferences: a reference curve on a surface (no 3D curve)
                                // gives a null handle here, which the analyzer catches as a failure
                                var Gac = aHCurve as GeomAdaptor_Curve
                                          ?? throw new Standard_NullObject("BRepCheck_Edge::InContext - no 3D curve");
                                Geom_Curve C3d = Gac.Curve()!;
                                Geom_Curve ProjOnPlane =
                                    GeomProjLib.ProjectOnPlane(new Geom_TrimmedCurve(C3d, First, Last),
                                                               P,
                                                               P.Position().Direction(),
                                                               true)!;
                                var aProjCurve = new GeomAdaptor_Curve(ProjOnPlane);

                                var proj = new ProjLib_ProjectedCurve(GAHS, aProjCurve);
                                Geom2d_Curve PC = Geom2dAdaptor.MakeCurve(proj);
                                var GHPC = new Geom2dAdaptor_Curve(PC, aHCurve.FirstParameter(), aHCurve.LastParameter());

                                var ACS = new Adaptor3d_CurveOnSurface(GHPC, GAHS);

                                var aValidateEdgeProj = new BRepLib_ValidateEdge(aHCurve, ACS, SameParameter);
                                aValidateEdgeProj.SetExitIfToleranceExceeded(Tol);
                                aValidateEdgeProj.SetExactMethod(myIsExactMethod);
                                aValidateEdgeProj.SetParallel(toRunParallel);
                                aValidateEdgeProj.Process();
                                if (!aValidateEdgeProj.IsDone() || !aValidateEdgeProj.CheckTolerance(Tol))
                                {
                                    BRepCheck.Add(lst, BRepCheck_InvalidCurveOnSurface);
                                }
                            }
                        }
                    }
                }

                break;
            case TopAbs_SOLID:
            {
                // on verifie que l`edge est bien connectee 2 fois (pas de bord libre)
                int nbconnection = 0;
                for (exp.Init(S, TopAbs_FACE); exp.More(); exp.Next())
                {
                    TopoDS_Face fac = TopoDS.Face(exp.Current());
                    var exp2 = new TopExp_Explorer();
                    for (exp2.Init(fac, TopAbs_EDGE); exp2.More(); exp2.Next())
                    {
                        if (exp2.Current().IsSame(myShape))
                        {
                            nbconnection++;
                        }
                    }
                }

                if (nbconnection < 2 && !TE.Degenerated())
                {
                    BRepCheck.Add(lst, BRepCheck_FreeEdge);
                }
                else if (nbconnection > 2)
                {
                    BRepCheck.Add(lst, BRepCheck_InvalidMultiConnexity);
                }
                else
                {
                    BRepCheck.Add(lst, BRepCheck_NoError);
                }
            }

                break;
            default:
                break;
        }

        if (lst.IsEmpty())
        {
            lst.Append(BRepCheck_NoError);
        }
    }

    /// <summary>Nothing more than the minimum (OCCT removed the body).</summary>
    public override void Blind()
    {
        //  Modified by skv - Tue Apr 27 11:36:01 2004 Begin
        // The body of this function is removed because of its useless.
        if (!myBlind)
        {
            myBlind = true;
        }
    }

    /// <summary>Sets whether <see cref="InContext"/> measures the pcurves against the reference curve.</summary>
    public void GeometricControls(bool B) => myGctrl = B;

    /// <summary>Whether <see cref="InContext"/> measures the pcurves against the reference curve.</summary>
    public bool GeometricControls() => myGctrl;

    /// <summary>Adds <paramref name="theStatus"/> to the statuses of the edge.</summary>
    public void SetStatus(BRepCheck_Status theStatus)
    {
        bool aLocked = myIsParallel;
        if (aLocked)
        {
            myMutex.Enter();
        }

        try
        {
            BRepCheck.Add(myMap.Find(myShape), theStatus);
        }
        finally
        {
            if (aLocked)
            {
                myMutex.Exit();
            }
        }
    }

    /// <summary>
    /// Sets the method of <see cref="BRepLib_ValidateEdge"/>: exact (BRepLib_CheckCurveOnSurface, slower, used only
    /// when the edge is same-parameter) or in a finite number of points (the default).
    /// </summary>
    public void SetExactMethod(bool theIsExact) => myIsExactMethod = theIsExact;

    /// <summary>Whether the exact method is selected.</summary>
    public bool IsExactMethod() => myIsExactMethod;

    /// <summary>
    /// The largest distance between the edge's representations at 23 parameters, with a 5% margin
    /// (<see cref="Precision.Confusion"/> for an edge with one representation).
    /// </summary>
    public double Tolerance()
    {
        var TE = (BRep_TEdge)myShape.TShape()!;
        int it, iRep = 1, nbRep = TE.Curves().Extent();
        if (nbRep <= 1)
        {
            return Precision.Confusion();
        }

        var theRep = new NCollection_Array1<Adaptor3d_Curve?>(1, nbRep * 2);
        double First, Last;
        if (myHCurve is not null)
        {
            First = myHCurve.FirstParameter();
            Last = myHCurve.LastParameter();
        }
        else
        {
            BRep_Tool.Range(TopoDS.Edge(myShape), out First, out Last);
        }

        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(TE.Curves());
        for (; itcr.More(); itcr.Next())
        {
            BRep_CurveRepresentation cr = itcr.Value();
            if (cr.IsCurve3D() && !TE.Degenerated())
            {
                //// modified by jgv, 20.03.03 ////
                TopLoc_Location Loc = myShape.Location() * cr.Location();
                var C3d = (Geom_Curve)cr.Curve3D()!.Transformed(Loc.Transformation());
                ///////////////////////////////////
                var GAC3d = new GeomAdaptor_Curve(C3d, First, Last);
                it = iRep;
                if (iRep > 1)
                {
                    theRep[iRep] = theRep[1];
                    it = 1;
                }

                theRep[it] = GAC3d;
                iRep++;
            }
            else if (cr.IsCurveOnSurface())
            {
                {
                    Geom_Surface Sref = cr.Surface()!;
                    //// modified by jgv, 20.03.03 ////
                    TopLoc_Location Loc = myShape.Location() * cr.Location();
                    Sref = (Geom_Surface)Sref.Transformed(Loc.Transformation());
                    ///////////////////////////////////
                    Geom2d_Curve PCref = cr.PCurve()!;
                    var GAHSref = new GeomAdaptor_Surface(Sref);
                    var GHPCref = new Geom2dAdaptor_Curve(PCref, First, Last);
                    theRep[iRep] = new Adaptor3d_CurveOnSurface(GHPCref, GAHSref);
                    iRep++;
                }

                if (cr.IsCurveOnClosedSurface())
                {
                    Geom_Surface Sref = cr.Surface()!;
                    // OCCT: the second pcurve's surface is placed without the edge's location, unlike the first
                    Sref = (Geom_Surface)Sref.Transformed(cr.Location().Transformation());
                    Geom2d_Curve PCref = cr.PCurve2()!;
                    var GAHSref = new GeomAdaptor_Surface(Sref);
                    var GHPCref = new Geom2dAdaptor_Curve(PCref, First, Last);
                    theRep[iRep] = new Adaptor3d_CurveOnSurface(GHPCref, GAHSref);
                    iRep++;
                    nbRep++;
                }
            }
            else
            {
                nbRep--;
            }
        }

        double dist2, tol2, tolCal = 0.0, prm;
        gp_Pnt center, othP;
        int i;
        for (i = 0; i < NCONTROL; i++)
        {
            prm = ((NCONTROL - 1 - i) * First + i * Last) / (NCONTROL - 1);
            tol2 = dist2 = 0.0;
            center = theRep[1]!.Value(prm);
            if (Precision.IsInfinite(center.X()) || Precision.IsInfinite(center.Y())
                || Precision.IsInfinite(center.Z()))
            {
                return Precision.Infinite();
            }

            for (iRep = 2; iRep <= nbRep; iRep++)
            {
                othP = theRep[iRep]!.Value(prm);
                if (Precision.IsInfinite(othP.X()) || Precision.IsInfinite(othP.Y())
                    || Precision.IsInfinite(othP.Z()))
                {
                    return Precision.Infinite();
                }

                dist2 = center.SquareDistance(othP);
                if (dist2 > tolCal)
                {
                    tolCal = dist2;
                }
            }

            if (tol2 > tolCal)
            {
                tolCal = tol2;
            }
        }

        // On prend 5% de marge car au dessus on crontrole severement
        return Math.Sqrt(tolCal) * 1.05;
    }

    /// <summary>
    /// Checks that the edge's polygons on triangulations lie within their deflection plus the edge's tolerance of
    /// its 3D curve.
    /// </summary>
    public BRepCheck_Status CheckPolygonOnTriangulation(in TopoDS_Edge theEdge)
    {
        // OCCT.NET: OCCT takes ChangeCurves() and only reads it
        NCollection_List<BRep_CurveRepresentation> aListOfCR = ((BRep_TEdge)theEdge.TShape()!).Curves();
        bool aHasPolygonOnTriangulation = false;
        bool aHasCurve3D = false;
        for (var anITCR = new NCollection_List<BRep_CurveRepresentation>.Iterator(aListOfCR);
             anITCR.More();
             anITCR.Next())
        {
            BRep_CurveRepresentation aCR = anITCR.Value();
            if (aCR.IsPolygonOnTriangulation())
            {
                aHasPolygonOnTriangulation = true;
            }

            if (aCR.IsCurve3D() && aCR.Curve3D() is not null)
            {
                aHasCurve3D = true;
            }

            if (aHasPolygonOnTriangulation && aHasCurve3D)
            {
                break;
            }
        }

        if (!aHasPolygonOnTriangulation || !aHasCurve3D)
        {
            return BRepCheck_NoError;
        }

        var aBC = new BRepAdaptor_Curve();
        aBC.Initialize(theEdge);

        if (!aBC.Is3DCurve())
        {
            return BRepCheck_NoError;
        }

        var anIt = new NCollection_List<BRep_CurveRepresentation>.Iterator(aListOfCR);

        while (anIt.More())
        {
            if (!anIt.Value().IsPolygonOnTriangulation())
            {
                anIt.Next();
                continue;
            }

            BRep_CurveRepresentation aCR = anIt.Value();
            var aPT = (BRep_PolygonOnTriangulation)aCR;

            TopLoc_Location aLL = theEdge.Location() * aPT.Location();
            gp_Trsf aTrsf = aLL.Transformation();

            Poly_Triangulation aTriang = aCR.Triangulation()!;
            Poly_PolygonOnTriangulation aPOnTriag = aCR.IsPolygonOnClosedTriangulation()
                ? aCR.PolygonOnTriangulation2()!
                : aCR.PolygonOnTriangulation()!;
            NCollection_Array1<int> anIndices = aPOnTriag.Nodes();
            int aNbNodes = anIndices.Length();

            double aTol = aPOnTriag.Deflection() + BRep_Tool.Tolerance(theEdge);

            if (aPOnTriag.HasParameters())
            {
                NCollection_HArray1<double> aParams = aPOnTriag.Parameters()!;
                for (int i = aParams.Lower(); i <= aParams.Upper(); i++)
                {
                    double aParam = aParams.Value(i);
                    gp_Pnt aPE = aBC.Value(aParam);
                    gp_Pnt aPnt = aTriang.Node(anIndices[i]).Transformed(aTrsf);

                    double aSQDist = aPE.SquareDistance(aPnt);
                    if (aSQDist > aTol * aTol)
                    {
                        return BRepCheck_InvalidPolygonOnTriangulation;
                    }
                }
            }
            else
            {
                // If aPOnTriag does not have any parameters we will check if it
                // inscribes into Bounding box, which is built on the edge triangulation.

                var aB = new Bnd_Box();

                for (int i = 1; i <= aNbNodes; i++)
                {
                    if (aTrsf.Form() == gp_Identity)
                    {
                        aB.Add(aTriang.Node(anIndices[i]));
                    }
                    else
                    {
                        aB.Add(aTriang.Node(anIndices[i]).Transformed(aTrsf));
                    }
                }

                aB.Enlarge(aTol);

                double aFP = aBC.FirstParameter();
                double aLP = aBC.LastParameter();

                double aStep = (aLP - aFP) / (double)NCONTROL;
                gp_Pnt aP;
                double aPar = aFP;

                for (int i = 1; i < NCONTROL; i++)
                {
                    aBC.D0(aPar, out aP);
                    if (aB.IsOut(aP))
                    {
                        return BRepCheck_InvalidPolygonOnTriangulation;
                    }

                    aPar += aStep;
                }

                aBC.D0(aLP, out aP);
                if (aB.IsOut(aP))
                {
                    return BRepCheck_InvalidPolygonOnTriangulation;
                }
            }

            anIt.Next();
        }

        return BRepCheck_NoError;
    }
}
