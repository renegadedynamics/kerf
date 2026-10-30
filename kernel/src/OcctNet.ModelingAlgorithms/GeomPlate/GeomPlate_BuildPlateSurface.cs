// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomPlate/GeomPlate_BuildPlateSurface.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomPlate/GeomPlate_BuildPlateSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// Builds a plate surface through curve and point constraints: an initial surface (given, or an average plane, or a
/// first plate's approximation), deformed by a plate of minimal energy that meets the constraints, refined over
/// several iterations until the G0/G1 criteria of the curve constraints are met.
/// </summary>
/// <remarks>
/// A working object (decision 21), as OCCT's: load constraints with <c>Add</c>, then <see cref="Perform"/>. It keeps
/// the caller's constraints and changes them as OCCT does (their 2D curves, numbers of points and point projections;
/// <c>Perform</c> also reorders its list of curve constraints). <see cref="Surface"/> and <see cref="SurfInit"/>
/// freeze the surfaces they return, and <see cref="Curves2d"/> the 2D curves.
/// </remarks>
public class GeomPlate_BuildPlateSurface
{
    private NCollection_HSequence<GeomPlate_CurveConstraint> myLinCont;
    private NCollection_HArray1<NCollection_Sequence<double>>? myParCont;
    private NCollection_HArray1<NCollection_Sequence<double>>? myPlateCont;
    private NCollection_HSequence<GeomPlate_PointConstraint> myPntCont;
    private Geom_Surface? mySurfInit;
    private Geom_Surface? myPlanarSurfInit;
    private GeomPlate_Surface? myGeomPlateSurface;
    private readonly Plate_Plate myPlate = new Plate_Plate();
    private readonly Plate_Plate myPrevPlate = new Plate_Plate();
    private readonly bool myAnisotropie;
    private NCollection_HArray1<int>? mySense;
    private readonly int myDegree;
    private NCollection_HArray1<int>? myInitOrder;
    private double myG0Error;
    private double myG1Error;
    private double myG2Error;
    private int myNbPtsOnCur;
    private bool mySurfInitIsGive;
    private readonly int myNbIter;
    private readonly Extrema_ExtPS myProj = new Extrema_ExtPS();
    private readonly double myTol2d;
    private readonly double myTol3d;
    private readonly double myTolAng;
    private double myTolU;
    private double myTolV;
    private int myNbBounds;
    private bool myIsLinear;
    private bool myFree;

    // Arrays of sequences: OCCT default-constructs the elements, a C# array of classes holds nulls.
    private static NCollection_HArray1<NCollection_Sequence<double>> NewSequences(int theUpper)
    {
        NCollection_HArray1<NCollection_Sequence<double>> anArray = new NCollection_HArray1<NCollection_Sequence<double>>(1, theUpper);
        for (int i = 1; i <= theUpper; i++)
        {
            anArray[i] = new NCollection_Sequence<double>();
        }
        return anArray;
    }

    /// <summary>The constructor compatible with the old version: one curve constraint per curve.</summary>
    /// <exception cref="Standard_ConstructionError">Fewer than one iteration, no curve, no order, no point, or a
    /// degree below 2.</exception>
    public GeomPlate_BuildPlateSurface(NCollection_HArray1<int> NPoints, NCollection_HArray1<Adaptor3d_Curve> TabCurve,
                                       NCollection_HArray1<int> Tang, int Degree, int NbIter = 3, double Tol2d = 0.00001,
                                       double Tol3d = 0.0001, double TolAng = 0.01, double TolCurv = 0.1,
                                       bool Anisotropie = false)
    {
        myAnisotropie = Anisotropie;
        myDegree = Degree;
        myNbIter = NbIter;
        myTol2d = Tol2d;
        myTol3d = Tol3d;
        myTolAng = TolAng;
        myNbBounds = 0;
        int NTCurve = TabCurve.Length(); // Number of linear constraints
        myNbPtsOnCur = 0; // Different calculation of the number of points depending on the length
        myLinCont = new NCollection_HSequence<GeomPlate_CurveConstraint>();
        myPntCont = new NCollection_HSequence<GeomPlate_PointConstraint>();
        if (myNbIter < 1)
        {
            throw new Standard_ConstructionError("GeomPlate :  Number of iteration must be >= 1");
        }

        if (NTCurve == 0)
        {
            throw new Standard_ConstructionError("GeomPlate : the bounds Array is null");
        }
        if (Tang.Length() == 0)
        {
            throw new Standard_ConstructionError("GeomPlate : the constraints Array is null");
        }
        int nbp = 0;
        int i;
        for (i = 1; i <= NTCurve; i++)
        {
            nbp += NPoints.Value(i);
        }
        if (nbp == 0)
        {
            throw new Standard_ConstructionError(
                "GeomPlate : the resolution is impossible if the number of constraints points is 0");
        }
        if (myDegree < 2)
        {
            throw new Standard_ConstructionError("GeomPlate ; the degree resolution must be upper of 2");
        }
        // Filling fields passing from the old constructor to the new one
        for (i = 1; i <= NTCurve; i++)
        {
            GeomPlate_CurveConstraint Cont = new GeomPlate_CurveConstraint(TabCurve.Value(i), Tang.Value(i), NPoints.Value(i));
            myLinCont.Append(Cont);
        }
        mySurfInitIsGive = false;

        myIsLinear = true;
        myFree = false;
    }

    /// <summary>A builder deforming the initial surface <paramref name="Surf"/>.</summary>
    /// <exception cref="Standard_ConstructionError">Fewer than one iteration, or a degree below 2.</exception>
    public GeomPlate_BuildPlateSurface(Geom_Surface Surf, int Degree = 3, int NbPtsOnCur = 10, int NbIter = 3,
                                       double Tol2d = 0.00001, double Tol3d = 0.0001, double TolAng = 0.01,
                                       double TolCurv = 0.1, bool Anisotropie = false)
    {
        mySurfInit = Surf;
        myAnisotropie = Anisotropie;
        myDegree = Degree;
        myNbPtsOnCur = NbPtsOnCur;
        myNbIter = NbIter;
        myTol2d = Tol2d;
        myTol3d = Tol3d;
        myTolAng = TolAng;
        myNbBounds = 0;
        if (myNbIter < 1)
        {
            throw new Standard_ConstructionError("GeomPlate :  Number of iteration must be >= 1");
        }
        if (myDegree < 2)
        {
            throw new Standard_ConstructionError("GeomPlate : the degree must be above 2");
        }
        myLinCont = new NCollection_HSequence<GeomPlate_CurveConstraint>();
        myPntCont = new NCollection_HSequence<GeomPlate_PointConstraint>();
        mySurfInitIsGive = true;

        myIsLinear = true;
        myFree = false;
    }

    /// <summary>A builder that computes its initial surface.</summary>
    /// <exception cref="Standard_ConstructionError">Fewer than one iteration, or a degree below 2.</exception>
    public GeomPlate_BuildPlateSurface(int Degree = 3, int NbPtsOnCur = 10, int NbIter = 3, double Tol2d = 0.00001,
                                       double Tol3d = 0.0001, double TolAng = 0.01, double TolCurv = 0.1,
                                       bool Anisotropie = false)
    {
        myAnisotropie = Anisotropie;
        myDegree = Degree;
        myNbPtsOnCur = NbPtsOnCur;
        myNbIter = NbIter;
        myTol2d = Tol2d;
        myTol3d = Tol3d;
        myTolAng = TolAng;
        myNbBounds = 0;
        if (myNbIter < 1)
        {
            throw new Standard_ConstructionError("GeomPlate :  Number of iteration must be >= 1");
        }
        if (myDegree < 2)
        {
            throw new Standard_ConstructionError("GeomPlate : the degree resolution must be upper of 2");
        }
        myLinCont = new NCollection_HSequence<GeomPlate_CurveConstraint>();
        myPntCont = new NCollection_HSequence<GeomPlate_PointConstraint>();
        mySurfInitIsGive = false;

        myIsLinear = true;
        myFree = false;
    }

    // Reorder the table of transformations
    // After the call of CourbeJointive the order of curves is modified
    // Ex : initial order of curves ==> A B C D E F
    //      In TabInit we note ==> 1 2 3 4 5 6
    //      after CourbeJointive ==> A E C B D F
    //            TabInit ==> 1 5 3 2 4 6
    //      after TrierTab the Table contains ==> 1 4 3 5 2 6
    // It is also possible to access the 2nd curve by taking TabInit[2]
    // i.e. the 4th from the table of classified curves
    private static void TrierTab(NCollection_HArray1<int> Tab)
    {
        // Parse the table of transformations to find the initial order
        int Nb = Tab.Length();
        NCollection_Array1<int> TabTri = new NCollection_Array1<int>(1, Nb);
        for (int i = 1; i <= Nb; i++)
        {
            TabTri.SetValue(Tab.Value(i), i);
        }
        Tab.ChangeArray1().Assign(TabTri);
    }

    private Geom2d_Curve? ProjectCurve(Adaptor3d_Curve Curv)
    {
        // Project a curve on a plane
        Geom2d_Curve? Curve2d;
        GeomAdaptor_Surface hsur = new GeomAdaptor_Surface(mySurfInit!);
        gp_Pnt2d P2d;

        ProjLib_CompProjectedCurve HProjector = new ProjLib_CompProjectedCurve(hsur, Curv, myTol3d / 10, myTol3d / 10);

        double UdebCheck, UfinCheck, ProjUdeb, ProjUfin;
        UdebCheck = Curv.FirstParameter();
        UfinCheck = Curv.LastParameter();
        HProjector.Bounds(1, out ProjUdeb, out ProjUfin);

        if (HProjector.NbCurves() != 1 || Math.Abs(UdebCheck - ProjUdeb) > Precision.PConfusion()
            || Math.Abs(UfinCheck - ProjUfin) > Precision.PConfusion())
        {
            if (HProjector.IsSinglePnt(1, out P2d))
            {
                // solution in a point
                NCollection_Array1<gp_Pnt2d> poles = new NCollection_Array1<gp_Pnt2d>(1, 2);
                poles.Init(P2d);
                Curve2d = new Geom2d_BezierCurve(poles);
            }
            else
            {
                Curve2d = null; // No continuous solution
            }
        }
        else
        {
            GeomAbs_Shape Continuity = GeomAbs_C1;
            int MaxDegree = 10, MaxSeg;
            double Udeb, Ufin;
            HProjector.Bounds(1, out Udeb, out Ufin);

            MaxSeg = 20 + HProjector.NbIntervals(GeomAbs_C3);
            Approx_CurveOnSurface appr = new Approx_CurveOnSurface(HProjector, hsur, Udeb, Ufin, myTol3d);
            appr.Perform(MaxSeg, MaxDegree, Continuity, false, true);

            Curve2d = appr.Curve2d();
        }
        return Curve2d;
    }

    // Projection of a curve on the initial surface (unused in OCCT, as the declaration in the header says)
    private Adaptor2d_Curve2d? ProjectedCurve(Adaptor3d_Curve Curv)
    {
        GeomAdaptor_Surface hsur = new GeomAdaptor_Surface(mySurfInit!);

        ProjLib_CompProjectedCurve? HProjector = new ProjLib_CompProjectedCurve(hsur, Curv, myTolU / 10, myTolV / 10);
        if (HProjector.NbCurves() != 1)
        {
            HProjector = null; // No continuous solution
        }
        else
        {
            double First1, Last1, First2, Last2;
            First1 = Curv.FirstParameter();
            Last1 = Curv.LastParameter();
            HProjector.Bounds(1, out First2, out Last2);

            if (Math.Abs(First1 - First2) <= Max(myTolU, myTolV) && Math.Abs(Last1 - Last2) <= Max(myTolU, myTolV))
            {
                HProjector = HProjector.Trim(First2, Last2, Precision.PConfusion()) as ProjLib_CompProjectedCurve;
            }
            else
            {
                HProjector = null; // No continuous solution
            }
        }
        return HProjector;
    }

    // Projects a point on the initial surface
    private gp_Pnt2d ProjectPoint(in gp_Pnt p3d)
    {
        Extrema_POnSurf P;
        myProj.Perform(p3d);
        int nearest = 1;
        if (myProj.NbExt() > 1)
        {
            double dist2mini = myProj.SquareDistance(1);
            for (int i = 2; i <= myProj.NbExt(); i++)
            {
                if (myProj.SquareDistance(i) < dist2mini)
                {
                    dist2mini = myProj.SquareDistance(i);
                    nearest = i;
                }
            }
        }
        P = myProj.Point(nearest);
        double u, v;
        P.Parameter(out u, out v);
        gp_Pnt2d p2d = new gp_Pnt2d();
        p2d.SetCoord(u, v);

        return p2d;
    }

    /// <summary>Removes the curve and point constraints.</summary>
    public void Init()
    {
        myLinCont.Clear();
        myPntCont.Clear();
        myPntCont = new NCollection_HSequence<GeomPlate_PointConstraint>();
        myLinCont = new NCollection_HSequence<GeomPlate_CurveConstraint>();
    }

    /// <summary>Loads the initial surface.</summary>
    public void LoadInitSurface(Geom_Surface Surf)
    {
        mySurfInit = Surf;
        mySurfInitIsGive = true;
    }

    /// <summary>Adds a curve constraint.</summary>
    public void Add(GeomPlate_CurveConstraint Cont)
    {
        myLinCont.Append(Cont);
    }

    public void SetNbBounds(int NbBounds)
    {
        myNbBounds = NbBounds;
    }

    /// <summary>Adds a point constraint.</summary>
    public void Add(GeomPlate_PointConstraint Cont)
    {
        myPntCont.Append(Cont);
    }

    /// <summary>Computes the surface filled with the loaded constraints.</summary>
    public void Perform(Message_ProgressRange theProgress = default)
    {
        myGeomPlateSurface = null;

        if (myNbBounds == 0)
        {
            myNbBounds = myLinCont.Length();
        }

        myPlate.Init();
        //=====================================================================
        // Declaration of variables.
        //=====================================================================
        int NTLinCont = myLinCont.Length(), NTPntCont = myPntCont.Length(), NbBoucle = 0;
        bool Fini = true;
        if ((NTLinCont + NTPntCont) == 0)
        {
            return;
        }

        //======================================================================
        // Initial Surface
        //======================================================================
        using Message_ProgressScope aPS = new Message_ProgressScope(theProgress, "Calculating the surface filled", 100, true);
        if (!mySurfInitIsGive)
        {
            ComputeSurfInit(aPS.Next(10));
            if (aPS.UserBreak())
            {
                return;
            }
        }

        else
        {
            if (NTLinCont >= 2)
            { // Table of transformations to preserve the initial order, see TrierTab
                myInitOrder = new NCollection_HArray1<int>(1, NTLinCont);
                for (int l = 1; l <= NTLinCont; l++)
                {
                    myInitOrder.SetValue(l, l);
                }
                if (!CourbeJointive(myTol3d))
                { //    throw Standard_Failure("Curves are not joined");
                }
                TrierTab(myInitOrder); // Reorder the table of transformations
            }
            else if (NTLinCont > 0) // Patch
            {
                mySense = new NCollection_HArray1<int>(1, NTLinCont, 0);
                myInitOrder = new NCollection_HArray1<int>(1, NTLinCont, 1);
            }
        }

        if (mySurfInit is null)
        {
            return;
        }

        double u1, v1, u2, v2;
        mySurfInit.Bounds(out u1, out v1, out u2, out v2);
        GeomAdaptor_Surface aSurfInit = new GeomAdaptor_Surface(mySurfInit);
        myTolU = aSurfInit.UResolution(myTol3d);
        myTolV = aSurfInit.VResolution(myTol3d);
        myProj.Initialize(aSurfInit, u1, v1, u2, v2, myTolU, myTolV);

        //======================================================================
        // Projection of curves
        //======================================================================
        bool Ok = true;
        for (int i = 1; i <= NTLinCont; i++)
        {
            if (myLinCont.Value(i).Curve2dOnSurf() is null)
            {
                Geom2d_Curve? Curve2d = ProjectCurve(myLinCont.Value(i).Curve3d());
                if (Curve2d is null)
                {
                    Ok = false;
                    break;
                }
                myLinCont.ChangeValue(i).SetCurve2dOnSurf(Curve2d);
            }
        }
        if (!Ok)
        {
            // OCCT: myGeomPlateSurface was nullified at the start of Perform, and is set again only when
            // ComputeSurfInit computed a first plate; otherwise native OCCT dereferences a null handle here
            // (an access violation). The port throws Standard_NullObject (occt-issues.md).
            if (myGeomPlateSurface is null)
            {
                throw new Standard_NullObject("GeomPlate_BuildPlateSurface::Perform: no plate surface to approximate");
            }
            GeomPlate_MakeApprox App = new GeomPlate_MakeApprox(myGeomPlateSurface, myTol3d, 1, 3, 15 * myTol3d, -1, GeomAbs_C0, 1.3);
            mySurfInit = App.Surface()!;

            mySurfInit.Bounds(out u1, out v1, out u2, out v2);
            GeomAdaptor_Surface Surf = new GeomAdaptor_Surface(mySurfInit);
            myTolU = Surf.UResolution(myTol3d);
            myTolV = Surf.VResolution(myTol3d);
            myProj.Initialize(Surf, u1, v1, u2, v2, myTolU, myTolV);

            Ok = true;
            for (int i = 1; i <= NTLinCont; i++)
            {
                Geom2d_Curve? Curve2d = ProjectCurve(myLinCont.Value(i).Curve3d());
                if (Curve2d is null)
                {
                    Ok = false;
                    break;
                }
                myLinCont.ChangeValue(i).SetCurve2dOnSurf(Curve2d);
            }
            if (!Ok)
            {
                mySurfInit = myPlanarSurfInit!;

                mySurfInit.Bounds(out u1, out v1, out u2, out v2);
                GeomAdaptor_Surface SurfNew = new GeomAdaptor_Surface(mySurfInit);
                myTolU = SurfNew.UResolution(myTol3d);
                myTolV = SurfNew.VResolution(myTol3d);
                myProj.Initialize(SurfNew, u1, v1, u2, v2, myTolU, myTolV);

                for (int i = 1; i <= NTLinCont; i++)
                {
                    myLinCont.ChangeValue(i).SetCurve2dOnSurf(ProjectCurve(myLinCont.Value(i).Curve3d()));
                }
            }
            else
            { // Project the points
                for (int i = 1; i <= NTPntCont; i++)
                {
                    gp_Pnt P;
                    myPntCont.Value(i).D0(out P);
                    myPntCont.ChangeValue(i).SetPnt2dOnSurf(ProjectPoint(P));
                }
            }
        }

        //======================================================================
        // Projection of points
        //======================================================================
        for (int i = 1; i <= NTPntCont; i++)
        {
            if (!myPntCont.Value(i).HasPnt2dOnSurf())
            {
                gp_Pnt P;
                myPntCont.Value(i).D0(out P);
                myPntCont.ChangeValue(i).SetPnt2dOnSurf(ProjectPoint(P));
            }
        }

        //======================================================================
        // Number of points by curve
        //======================================================================
        if ((NTLinCont != 0) && (myNbPtsOnCur != 0))
        {
            CalculNbPtsInit();
        }

        //======================================================================
        // Management of incompatibilites between curves
        //======================================================================
        NCollection_HArray1<NCollection_Sequence<double>>? PntInter = null;
        NCollection_HArray1<NCollection_Sequence<double>>? PntG1G1 = null;
        if (NTLinCont != 0)
        {
            PntInter = NewSequences(NTLinCont);
            PntG1G1 = NewSequences(NTLinCont);
            Intersect(PntInter, PntG1G1);
        }

        //======================================================================
        // Loop to obtain a better surface
        //======================================================================

        myFree = !myIsLinear;

        do
        {
            NbBoucle++;
            if (NTLinCont != 0)
            { //====================================================================
              // Calculate the total number of points and the maximum of points by curve
              //====================================================================
                int NPointMax = 0;
                for (int i = 1; i <= NTLinCont; i++)
                {
                    if ((myLinCont.Value(i).NbPoints()) > NPointMax)
                    {
                        NPointMax = (int)(myLinCont.Value(i).NbPoints());
                    }
                }
                //====================================================================
                // Discretization of curves
                //====================================================================
                Discretise(PntInter!, PntG1G1!);
                //====================================================================
                // Preparation of constraint points for plate
                //====================================================================
                LoadCurve(NbBoucle);
                if (myPntCont.Length() != 0)
                {
                    LoadPoint(NbBoucle);
                }
                //====================================================================
                // Construction of the surface
                //====================================================================

                myPlate.SolveTI(myDegree, ComputeAnisotropie(), aPS.Next(90));

                if (aPS.UserBreak())
                {
                    return;
                }

                if (!myPlate.IsDone())
                {
                    return;
                }

                myGeomPlateSurface = new GeomPlate_Surface(mySurfInit, myPlate);
                double Umin, Umax, Vmin, Vmax;
                myPlate.UVBox(out Umin, out Umax, out Vmin, out Vmax);
                myGeomPlateSurface.SetBounds(Umin, Umax, Vmin, Vmax);

                Fini = VerifSurface(NbBoucle);
                if ((NbBoucle >= myNbIter) && (!Fini))
                {
                    Fini = true;
                }

                if ((NTPntCont != 0) && (Fini))
                {
                    double di, an, cu;
                    VerifPoints(out di, out an, out cu);
                }
            }
            else
            {
                LoadPoint(NbBoucle);
                //====================================================================
                // Construction of the surface
                //====================================================================
                myPlate.SolveTI(myDegree, ComputeAnisotropie(), aPS.Next(90));

                if (aPS.UserBreak())
                {
                    return;
                }

                if (!myPlate.IsDone())
                {
                    return;
                }

                myGeomPlateSurface = new GeomPlate_Surface(mySurfInit, myPlate);
                double Umin, Umax, Vmin, Vmax;
                myPlate.UVBox(out Umin, out Umax, out Vmin, out Vmax);
                myGeomPlateSurface.SetBounds(Umin, Umax, Vmin, Vmax);
                Fini = true;
                double di, an, cu;
                VerifPoints(out di, out an, out cu);
            }
        } while (!Fini); // End loop for better surface
    }

    private void EcartContraintesMil(int c, NCollection_HArray1<double> d, NCollection_HArray1<double> an,
                                     NCollection_HArray1<double> courb)
    {
        int NbPt = myParCont!.Value(c).Length();
        double U;
        if (NbPt < 3)
        {
            NbPt = 4;
        }
        else
        {
            NbPt = myParCont.Value(c).Length();
        }
        gp_Vec v1i, v1f, v2i, v2f, v3i, v3f;
        gp_Pnt Pi, Pf;
        gp_Pnt2d P2d;
        int i;
        GeomPlate_CurveConstraint LinCont = myLinCont.Value(c);
        switch (LinCont.Order())
        {
            case 0:
                for (i = 1; i < NbPt; i++)
                {
                    U = (myParCont.Value(c).Value(i) + myParCont.Value(c).Value(i + 1)) / 2;
                    LinCont.D0(U, out Pi);
                    if (LinCont.ProjectedCurve() is not null)
                    {
                        P2d = LinCont.ProjectedCurve()!.Value(U);
                    }
                    else
                    {
                        if (LinCont.Curve2dOnSurf() is not null)
                        {
                            P2d = LinCont.Curve2dOnSurf()!.Value(U);
                        }
                        else
                        {
                            P2d = ProjectPoint(Pi);
                        }
                    }
                    myGeomPlateSurface!.D0(P2d.Coord(1), P2d.Coord(2), out Pf);
                    an.Init(0);
                    courb.Init(0);
                    d.ChangeValue(i) = Pf.Distance(Pi);
                }
                break;
            case 1:
                for (i = 1; i < NbPt; i++)
                {
                    U = (myParCont.Value(c).Value(i) + myParCont.Value(c).Value(i + 1)) / 2;
                    LinCont.D1(U, out Pi, out v1i, out v2i);
                    if (LinCont.ProjectedCurve() is not null)
                    {
                        P2d = LinCont.ProjectedCurve()!.Value(U);
                    }
                    else
                    {
                        if (LinCont.Curve2dOnSurf() is not null)
                        {
                            P2d = LinCont.Curve2dOnSurf()!.Value(U);
                        }
                        else
                        {
                            P2d = ProjectPoint(Pi);
                        }
                    }
                    myGeomPlateSurface!.D1(P2d.Coord(1), P2d.Coord(2), out Pf, out v1f, out v2f);
                    d.ChangeValue(i) = Pf.Distance(Pi);
                    v3i = v1i ^ v2i;
                    v3f = v1f ^ v2f;
                    double angle = v3f.Angle(v3i);
                    if (angle > (M_PI / 2))
                    {
                        an.ChangeValue(i) = M_PI - angle;
                    }
                    else
                    {
                        an.ChangeValue(i) = angle;
                    }
                    courb.Init(0);
                }
                break;
            case 2:
                Geom_Surface Splate = myGeomPlateSurface!;
                LocalAnalysis_SurfaceContinuity CG2 = new LocalAnalysis_SurfaceContinuity();
                for (i = 1; i < NbPt; i++)
                {
                    U = (myParCont.Value(c).Value(i) + myParCont.Value(c).Value(i + 1)) / 2;
                    LinCont.D0(U, out Pi);
                    if (LinCont.ProjectedCurve() is not null)
                    {
                        P2d = LinCont.ProjectedCurve()!.Value(U);
                    }
                    else
                    {
                        if (LinCont.Curve2dOnSurf() is not null)
                        {
                            P2d = LinCont.Curve2dOnSurf()!.Value(U);
                        }
                        else
                        {
                            P2d = ProjectPoint(Pi);
                        }
                    }
                    GeomLProp_SLProps Prop = new GeomLProp_SLProps(Splate, P2d.Coord(1), P2d.Coord(2), 2, 0.001);
                    CG2.ComputeAnalysis(Prop, myLinCont.Value(c).LPropSurf(U), GeomAbs_G2);
                    d.ChangeValue(i) = CG2.C0Value();
                    an.ChangeValue(i) = CG2.G1Angle();
                    courb.ChangeValue(i) = CG2.G2CurvatureGap();
                }
                break;
        }
    }

    /// <summary>
    /// The UV points of the constraints (order other than -1) on the initial surface: the point constraints, then
    /// each curve's first point and, on each interval, its points at 1/4, 1/2, 3/4 and the end.
    /// </summary>
    public void Disc2dContour(int nbp, NCollection_Sequence<gp_XY> Seq2d)
    {
        //  initialization
        Seq2d.Clear();

        //  sampling in "cosine" + 3 points on each interval
        int NTCurve = myLinCont.Length();
        int NTPntCont = myPntCont.Length();
        gp_Pnt2d P2d;
        gp_XY UV = new gp_XY();
        gp_Pnt PP;
        double u1, v1, u2, v2;
        int i;

        mySurfInit!.Bounds(out u1, out v1, out u2, out v2);
        GeomAdaptor_Surface Surf = new GeomAdaptor_Surface(mySurfInit);
        myProj.Initialize(Surf, u1, v1, u2, v2, myTolU, myTolV);

        for (i = 1; i <= NTPntCont; i++)
        {
            if (myPntCont.Value(i).Order() != -1)
            {
                P2d = myPntCont.Value(i).Pnt2dOnSurf();
                UV.SetX(P2d.Coord(1));
                UV.SetY(P2d.Coord(2));
                Seq2d.Append(UV);
            }
        }
        for (i = 1; i <= NTCurve; i++)
        {
            GeomPlate_CurveConstraint LinCont = myLinCont.Value(i);
            if (LinCont.Order() != -1)
            {
                int NbPt = myParCont!.Value(i).Length();
                // first point of constraint (j=0)
                if (LinCont.ProjectedCurve() is not null)
                {
                    P2d = LinCont.ProjectedCurve()!.Value(myParCont.Value(i).Value(1));
                }
                else
                {
                    if (LinCont.Curve2dOnSurf() is not null)
                    {
                        P2d = LinCont.Curve2dOnSurf()!.Value(myParCont.Value(i).Value(1));
                    }
                    else
                    {
                        LinCont.D0(myParCont.Value(i).Value(1), out PP);
                        P2d = ProjectPoint(PP);
                    }
                }

                UV.SetX(P2d.Coord(1));
                UV.SetY(P2d.Coord(2));
                Seq2d.Append(UV);
                for (int j = 2; j < NbPt; j++)
                {
                    double Uj = myParCont.Value(i).Value(j), Ujp1 = myParCont.Value(i).Value(j + 1);
                    if (LinCont.ProjectedCurve() is not null)
                    {
                        P2d = LinCont.ProjectedCurve()!.Value((Ujp1 + 3 * Uj) / 4);
                    }
                    else
                    {
                        if (LinCont.Curve2dOnSurf() is not null)
                        {
                            P2d = LinCont.Curve2dOnSurf()!.Value((Ujp1 + 3 * Uj) / 4);
                        }
                        else
                        {
                            LinCont.D0((Ujp1 + 3 * Uj) / 4, out PP);
                            P2d = ProjectPoint(PP);
                        }
                    }
                    UV.SetX(P2d.Coord(1));
                    UV.SetY(P2d.Coord(2));
                    Seq2d.Append(UV);
                    // point 1/2 previous
                    if (LinCont.ProjectedCurve() is not null)
                    {
                        P2d = LinCont.ProjectedCurve()!.Value((Ujp1 + Uj) / 2);
                    }
                    else
                    {
                        if (LinCont.Curve2dOnSurf() is not null)
                        {
                            P2d = LinCont.Curve2dOnSurf()!.Value((Ujp1 + Uj) / 2);
                        }
                        else
                        {
                            LinCont.D0((Ujp1 + Uj) / 2, out PP);
                            P2d = ProjectPoint(PP);
                        }
                    }

                    UV.SetX(P2d.Coord(1));
                    UV.SetY(P2d.Coord(2));
                    Seq2d.Append(UV);
                    //  point 3/4 previous
                    if (LinCont.ProjectedCurve() is not null)
                    {
                        P2d = LinCont.ProjectedCurve()!.Value((3 * Ujp1 + Uj) / 4);
                    }
                    else
                    {
                        if (LinCont.Curve2dOnSurf() is not null)
                        {
                            P2d = LinCont.Curve2dOnSurf()!.Value((3 * Ujp1 + Uj) / 4);
                        }
                        else
                        {
                            LinCont.D0((3 * Ujp1 + Uj) / 4, out PP);
                            P2d = ProjectPoint(PP);
                        }
                    }

                    UV.SetX(P2d.Coord(1));
                    UV.SetY(P2d.Coord(2));
                    Seq2d.Append(UV);
                    // current constraint point
                    if (LinCont.ProjectedCurve() is not null)
                    {
                        P2d = LinCont.ProjectedCurve()!.Value(Ujp1);
                    }
                    else
                    {
                        if (LinCont.Curve2dOnSurf() is not null)
                        {
                            P2d = LinCont.Curve2dOnSurf()!.Value(Ujp1);
                        }
                        else
                        {
                            LinCont.D0(Ujp1, out PP);
                            P2d = ProjectPoint(PP);
                        }
                    }

                    UV.SetX(P2d.Coord(1));
                    UV.SetY(P2d.Coord(2));
                    Seq2d.Append(UV);
                }
            }
        }
    }

    /// <summary>
    /// The 3D points (<paramref name="iordre"/> 0) or normals (1) of the constraints at the points of
    /// <see cref="Disc2dContour"/>.
    /// </summary>
    public void Disc3dContour(int nbp, int iordre, NCollection_Sequence<gp_XYZ> Seq3d)
    {
        //  initialization
        Seq3d.Clear();
        //  sampling in "cosine" + 3 points on each interval
        double u1, v1, u2, v2;
        mySurfInit!.Bounds(out u1, out v1, out u2, out v2);
        GeomAdaptor_Surface Surf = new GeomAdaptor_Surface(mySurfInit);
        myProj.Initialize(Surf, u1, v1, u2, v2, Surf.UResolution(myTol3d), Surf.VResolution(myTol3d));
        int NTCurve = myLinCont.Length();
        int NTPntCont = myPntCont.Length();
        gp_Pnt P3d;
        gp_Vec v1h, v2h, v3h;
        gp_XYZ Pos = new gp_XYZ();
        int i;

        for (i = 1; i <= NTPntCont; i++)
        {
            if (myPntCont.Value(i).Order() != -1)
            {
                if (iordre == 0)
                {
                    myPntCont.Value(i).D0(out P3d);
                    Pos.SetX(P3d.X());
                    Pos.SetY(P3d.Y());
                    Pos.SetZ(P3d.Z());
                    Seq3d.Append(Pos);
                }
                else
                {
                    myPntCont.Value(i).D1(out P3d, out v1h, out v2h);
                    v3h = v1h ^ v2h;
                    Pos.SetX(v3h.X());
                    Pos.SetY(v3h.Y());
                    Pos.SetZ(v3h.Z());
                    Seq3d.Append(Pos);
                }
            }
        }

        for (i = 1; i <= NTCurve; i++)
        {
            if (myLinCont.Value(i).Order() != -1)
            {
                int NbPt = myParCont!.Value(i).Length();
                // first constraint point (j=0)
                if (iordre == 0)
                {
                    myLinCont.Value(i).D0(myParCont.Value(i).Value(1), out P3d);
                    Pos.SetX(P3d.X());
                    Pos.SetY(P3d.Y());
                    Pos.SetZ(P3d.Z());
                    Seq3d.Append(Pos);
                }
                else
                {
                    myLinCont.Value(i).D1(myParCont.Value(i).Value(1), out P3d, out v1h, out v2h);
                    v3h = v1h ^ v2h;
                    Pos.SetX(v3h.X());
                    Pos.SetY(v3h.Y());
                    Pos.SetZ(v3h.Z());
                    Seq3d.Append(Pos);
                }

                for (int j = 2; j < NbPt; j++)
                {
                    double Uj = myParCont.Value(i).Value(j), Ujp1 = myParCont.Value(i).Value(j + 1);
                    if (iordre == 0)
                    {
                        // point 1/4 previous
                        myLinCont.Value(i).D0((Ujp1 + 3 * Uj) / 4, out P3d);
                        Pos.SetX(P3d.X());
                        Pos.SetY(P3d.Y());
                        Pos.SetZ(P3d.Z());
                        Seq3d.Append(Pos);
                        // point 1/2 previous
                        myLinCont.Value(i).D0((Ujp1 + Uj) / 2, out P3d);
                        Pos.SetX(P3d.X());
                        Pos.SetY(P3d.Y());
                        Pos.SetZ(P3d.Z());
                        Seq3d.Append(Pos);
                        // point 3/4 previous
                        myLinCont.Value(i).D0((3 * Ujp1 + Uj) / 4, out P3d);
                        Pos.SetX(P3d.X());
                        Pos.SetY(P3d.Y());
                        Pos.SetZ(P3d.Z());
                        Seq3d.Append(Pos);
                        // current constraint point
                        myLinCont.Value(i).D0(Ujp1, out P3d);
                        Pos.SetX(P3d.X());
                        Pos.SetY(P3d.Y());
                        Pos.SetZ(P3d.Z());
                        Seq3d.Append(Pos);
                    }
                    else
                    {
                        // point 1/4 previous
                        myLinCont.Value(i).D1((Ujp1 + 3 * Uj) / 4, out P3d, out v1h, out v2h);
                        v3h = v1h ^ v2h;
                        Pos.SetX(v3h.X());
                        Pos.SetY(v3h.Y());
                        Pos.SetZ(v3h.Z());
                        Seq3d.Append(Pos);
                        // point 1/2 previous
                        myLinCont.Value(i).D1((Ujp1 + Uj) / 2, out P3d, out v1h, out v2h);
                        v3h = v1h ^ v2h;
                        Pos.SetX(v3h.X());
                        Pos.SetY(v3h.Y());
                        Pos.SetZ(v3h.Z());
                        Seq3d.Append(Pos);
                        // point 3/4 previous
                        myLinCont.Value(i).D1((3 * Ujp1 + Uj) / 4, out P3d, out v1h, out v2h);
                        v3h = v1h ^ v2h;
                        Pos.SetX(v3h.X());
                        Pos.SetY(v3h.Y());
                        Pos.SetZ(v3h.Z());
                        Seq3d.Append(Pos);
                        // current constraint point
                        myLinCont.Value(i).D1(Ujp1, out P3d, out v1h, out v2h);
                        v3h = v1h ^ v2h;
                        Pos.SetX(v3h.X());
                        Pos.SetY(v3h.Y());
                        Pos.SetZ(v3h.Z());
                        Seq3d.Append(Pos);
                    }
                }
            }
        }
    }

    /// <summary>True if the plate was solved (OCCT's plate also reports done before any solve, occt-issues.md).</summary>
    public bool IsDone() => myPlate.IsDone();

    /// <summary>The plate surface (frozen), or null.</summary>
    public GeomPlate_Surface? Surface()
    {
        myGeomPlateSurface?.Freeze();
        return myGeomPlateSurface;
    }

    /// <summary>The initial surface (frozen), or null.</summary>
    public Geom_Surface? SurfInit()
    {
        mySurfInit?.Freeze();
        return mySurfInit;
    }

    /// <summary>For each curve constraint, in the order of loading, 1 if it is reversed in the contour.</summary>
    public NCollection_HArray1<int> Sense()
    {
        int NTCurve = myLinCont.Length();
        NCollection_HArray1<int> Sens = new NCollection_HArray1<int>(1, NTCurve);
        for (int i = 1; i <= NTCurve; i++)
        {
            Sens.SetValue(i, mySense!.Value(myInitOrder!.Value(i)));
        }
        return Sens;
    }

    /// <summary>The 2D curves of the curve constraints on the initial surface, in the order of loading (frozen).</summary>
    public NCollection_HArray1<Geom2d_Curve?> Curves2d()
    {
        int NTCurve = myLinCont.Length();
        NCollection_HArray1<Geom2d_Curve?> C2dfin = new NCollection_HArray1<Geom2d_Curve?>(1, NTCurve);

        for (int i = 1; i <= NTCurve; i++)
        {
            Geom2d_Curve? aCurve = myLinCont.Value(myInitOrder!.Value(i)).Curve2dOnSurf();
            aCurve?.Freeze();
            C2dfin.SetValue(i, aCurve);
        }
        return C2dfin;
    }

    /// <summary>For each curve constraint in the order of loading, its index in the reordered contour.</summary>
    public NCollection_HArray1<int> Order()
    {
        NCollection_HArray1<int> result = new NCollection_HArray1<int>(1, myLinCont.Length());
        for (int i = 1; i <= myLinCont.Length(); i++)
        {
            result.SetValue(myInitOrder!.Value(i), i);
        }
        return result;
    }

    /// <summary>The largest distance between the surface and the curve constraints (0 before a curve is checked).</summary>
    public double G0Error() => myG0Error;

    /// <summary>The largest angle between the surface and the curve constraints.</summary>
    public double G1Error() => myG1Error;

    /// <summary>The largest curvature gap between the surface and the curve constraints.</summary>
    public double G2Error() => myG2Error;

    /// <summary>The largest distance between the surface and the curve constraint <paramref name="Index"/>.</summary>
    /// <exception cref="Standard_OutOfRange">The constraint has more points than the builder's NbPtsOnCur (OCCT's bug).</exception>
    public double G0Error(int Index)
    {
        // OCCT: the arrays have myNbPtsOnCur elements, but EcartContraintesMil writes one per interval of the curve's
        // discretisation, which has more points once Perform raised them: Standard_OutOfRange (occt-issues.md)
        NCollection_HArray1<double> tdistance = new NCollection_HArray1<double>(1, myNbPtsOnCur);
        NCollection_HArray1<double> tangle = new NCollection_HArray1<double>(1, myNbPtsOnCur);
        NCollection_HArray1<double> tcurvature = new NCollection_HArray1<double>(1, myNbPtsOnCur);
        EcartContraintesMil(Index, tdistance, tangle, tcurvature);
        double MaxDistance = 0.0;
        for (int i = 1; i <= myNbPtsOnCur; i++)
        {
            if (tdistance.Value(i) > MaxDistance)
            {
                MaxDistance = tdistance.Value(i);
            }
        }
        return MaxDistance;
    }

    /// <summary>The largest angle between the surface and the curve constraint <paramref name="Index"/>.</summary>
    public double G1Error(int Index)
    {
        // OCCT: the arrays have myNbPtsOnCur elements, but EcartContraintesMil writes one per interval of the curve's
        // discretisation, which has more points once Perform raised them: Standard_OutOfRange (occt-issues.md)
        NCollection_HArray1<double> tdistance = new NCollection_HArray1<double>(1, myNbPtsOnCur);
        NCollection_HArray1<double> tangle = new NCollection_HArray1<double>(1, myNbPtsOnCur);
        NCollection_HArray1<double> tcurvature = new NCollection_HArray1<double>(1, myNbPtsOnCur);
        EcartContraintesMil(Index, tdistance, tangle, tcurvature);
        double MaxAngle = 0.0;
        for (int i = 1; i <= myNbPtsOnCur; i++)
        {
            if (tangle.Value(i) > MaxAngle)
            {
                MaxAngle = tangle.Value(i);
            }
        }
        return MaxAngle;
    }

    /// <summary>The largest curvature gap between the surface and the curve constraint <paramref name="Index"/>.</summary>
    public double G2Error(int Index)
    {
        // OCCT: the arrays have myNbPtsOnCur elements, but EcartContraintesMil writes one per interval of the curve's
        // discretisation, which has more points once Perform raised them: Standard_OutOfRange (occt-issues.md)
        NCollection_HArray1<double> tdistance = new NCollection_HArray1<double>(1, myNbPtsOnCur);
        NCollection_HArray1<double> tangle = new NCollection_HArray1<double>(1, myNbPtsOnCur);
        NCollection_HArray1<double> tcurvature = new NCollection_HArray1<double>(1, myNbPtsOnCur);
        EcartContraintesMil(Index, tdistance, tangle, tcurvature);
        double MaxCurvature = 0.0;
        for (int i = 1; i <= myNbPtsOnCur; i++)
        {
            if (tcurvature.Value(i) > MaxCurvature)
            {
                MaxCurvature = tcurvature.Value(i);
            }
        }
        return MaxCurvature;
    }

    /// <summary>The curve constraint at <paramref name="order"/> in the (reordered) list.</summary>
    public GeomPlate_CurveConstraint CurveConstraint(int order) => myLinCont.Value(order);

    /// <summary>The point constraint at <paramref name="order"/>.</summary>
    public GeomPlate_PointConstraint PointConstraint(int order) => myPntCont.Value(order);

    // Create a chain of curves to calculate the initial surface with the method of max flow.
    // Return true if it is a closed contour.
    private bool CourbeJointive(double tolerance)
    {
        int nbf = myLinCont.Length();
        double Ufinal1, Uinit1, Ufinal2, Uinit2;
        mySense = new NCollection_HArray1<int>(1, nbf, 0);
        bool result = true;
        int j = 1, i;
        gp_Pnt P1, P2;

        while (j <= (myNbBounds - 1))
        {
            int a = 0;
            i = j + 1;
            if (i > myNbBounds)
            {
                result = false;
                a = 2;
            }
            while (a < 1)
            {
                if (i > myNbBounds)
                {
                    result = false;
                    a = 2;
                }
                else
                {
                    Uinit1 = myLinCont.Value(j).FirstParameter();
                    Ufinal1 = myLinCont.Value(j).LastParameter();
                    Uinit2 = myLinCont.Value(i).FirstParameter();
                    Ufinal2 = myLinCont.Value(i).LastParameter();
                    if (mySense.Value(j) == 1)
                    {
                        Ufinal1 = Uinit1;
                    }
                    myLinCont.Value(j).D0(Ufinal1, out P1);
                    myLinCont.Value(i).D0(Uinit2, out P2);
                    if (P1.Distance(P2) < tolerance)
                    {
                        if (i != j + 1)
                        {
                            GeomPlate_CurveConstraint tampon = myLinCont.Value(j + 1);
                            myLinCont.SetValue(j + 1, myLinCont.Value(i));
                            myLinCont.SetValue(i, tampon);
                            int Tmp = myInitOrder!.Value(j + 1);
                            // See function TrierTab for the functioning of myInitOrder
                            myInitOrder.SetValue(j + 1, myInitOrder.Value(i));
                            myInitOrder.SetValue(i, Tmp);
                        }
                        a = 2;
                        mySense.SetValue(j + 1, 0);
                    }
                    else
                    {
                        myLinCont.Value(i).D0(Ufinal2, out P2);

                        if (P1.Distance(P2) < tolerance)
                        {
                            if (i != j + 1)
                            {
                                GeomPlate_CurveConstraint tampon = myLinCont.Value(j + 1);
                                myLinCont.SetValue(j + 1, myLinCont.Value(i));
                                myLinCont.SetValue(i, tampon);
                                int Tmp = myInitOrder!.Value(j + 1);
                                // See function TrierTab for the functioning of myInitOrder
                                myInitOrder.SetValue(j + 1, myInitOrder.Value(i));
                                myInitOrder.SetValue(i, Tmp);
                            }
                            a = 2;
                            mySense.SetValue(j + 1, 1);
                        }
                    }
                }
                i++;
            }
            j++;
        }
        Uinit1 = myLinCont.Value(myNbBounds).FirstParameter();
        Ufinal1 = myLinCont.Value(myNbBounds).LastParameter();
        Uinit2 = myLinCont.Value(1).FirstParameter();
        Ufinal2 = myLinCont.Value(1).LastParameter();
        myLinCont.Value(myNbBounds).D0(Ufinal1, out P1);
        myLinCont.Value(1).D0(Uinit2, out P2);
        if ((mySense.Value(myNbBounds) == 0) && (P1.Distance(P2) < tolerance))
        {
            return result;
        }
        myLinCont.Value(myNbBounds).D0(Uinit1, out P1);
        if ((mySense.Value(myNbBounds) == 1) && (P1.Distance(P2) < tolerance))
        {
            return result;
        }
        else
        {
            return false;
        }
    }

    // Calculate the initial surface either by the method of max flow or by
    // the method of the plane of inertia if the contour is not closed or if
    // there are point constraints.
    private void ComputeSurfInit(Message_ProgressRange theProgress)
    {
        int nopt = 2, popt = 2, Np = 1;
        bool isHalfSpace = true;
        double LinTol = 0.001, AngTol = 0.001; // AngTol = 0.0001; //LinTol = 0.0001

        // Option to calculate the initial plane
        int NTLinCont = myLinCont.Length(), NTPntCont = myPntCont.Length();
        // Table of transformation to preserve the initial order see TrierTab
        if (NTLinCont != 0)
        {
            myInitOrder = new NCollection_HArray1<int>(1, NTLinCont);
            for (int i = 1; i <= NTLinCont; i++)
            {
                myInitOrder.SetValue(i, i);
            }
        }

        bool CourbeJoint = (NTLinCont != 0) && CourbeJointive(myTol3d);
        if (CourbeJoint && IsOrderG1())
        {
            nopt = 3;
            // Table contains the cloud of points for calculation of the plane
            int NbPoint = 20, Discr = NbPoint / 4, pnum = 0;
            NCollection_HArray1<gp_Pnt> Pts = new NCollection_HArray1<gp_Pnt>(1, (NbPoint + 1) * NTLinCont + NTPntCont);
            NCollection_Sequence<gp_Vec> Vecs = new NCollection_Sequence<gp_Vec>(), NewVecs = new NCollection_Sequence<gp_Vec>();
            NCollection_Sequence<GeomPlate_Aij> Aset = new NCollection_Sequence<GeomPlate_Aij>();
            double Uinit, Ufinal, Uif;
            gp_Vec LastVec = new gp_Vec();
            int i;
            for (i = 1; i <= NTLinCont; i++)
            {
                int Order = myLinCont.Value(i).Order();

                NewVecs.Clear();

                Uinit = myLinCont.Value(i).FirstParameter();
                Ufinal = myLinCont.Value(i).LastParameter();
                Uif = Ufinal - Uinit;
                if (mySense!.Value(i) == 1)
                {
                    Uinit = Ufinal;
                    Uif = -Uif;
                }

                gp_Vec Vec1, Vec2, Normal = new gp_Vec();
                bool ToReverse = false;
                if (i > 1 && Order >= (int)GeomAbs_G1)
                {
                    gp_Pnt P;
                    myLinCont.Value(i).D1(Uinit, out P, out Vec1, out Vec2);
                    Normal = Vec1 ^ Vec2;
                    if (LastVec.IsOpposite(Normal, AngTol))
                    {
                        ToReverse = true;
                    }
                }

                for (int j = 0; j <= NbPoint; j++)
                { // Number of points per curve = 20
                  // Linear distribution
                    double Inter = j * Uif / (NbPoint);
                    if (Order < (int)GeomAbs_G1 || j % Discr != 0)
                    {
                        myLinCont.Value(i).D0(Uinit + Inter, out Pts.ChangeValue(++pnum));
                    }
                    else
                    {
                        myLinCont.Value(i).D1(Uinit + Inter, out Pts.ChangeValue(++pnum), out Vec1, out Vec2);
                        Normal = Vec1 ^ Vec2;
                        Normal.Normalize();
                        if (ToReverse)
                        {
                            Normal.Reverse();
                        }
                        bool isNew = true;
                        int k;
                        for (k = 1; k <= Vecs.Length(); k++)
                        {
                            if (Vecs.Value(k).IsEqual(Normal, LinTol, AngTol))
                            {
                                isNew = false;
                                break;
                            }
                        }
                        if (isNew)
                        {
                            for (k = 1; k <= NewVecs.Length(); k++)
                            {
                                if (NewVecs.Value(k).IsEqual(Normal, LinTol, AngTol))
                                {
                                    isNew = false;
                                    break;
                                }
                            }
                        }
                        if (isNew)
                        {
                            NewVecs.Append(Normal);
                        }
                    }
                }
                if (Order >= (int)GeomAbs_G1)
                {
                    isHalfSpace = GeomPlate_BuildAveragePlane.HalfSpace(NewVecs, Vecs, Aset, LinTol, AngTol);
                    if (!isHalfSpace)
                    {
                        break;
                    }
                    LastVec = Normal;
                }
            } // for (i = 1; i <= NTLinCont; i++)

            if (isHalfSpace)
            {
                for (i = 1; i <= NTPntCont; i++)
                {
                    int Order = myPntCont.Value(i).Order();

                    NewVecs.Clear();
                    gp_Vec Vec1, Vec2, Normal;
                    if (Order < (int)GeomAbs_G1)
                    {
                        myPntCont.Value(i).D0(out Pts.ChangeValue(++pnum));
                    }
                    else
                    {
                        myPntCont.Value(i).D1(out Pts.ChangeValue(++pnum), out Vec1, out Vec2);
                        Normal = Vec1 ^ Vec2;
                        Normal.Normalize();
                        bool isNew = true;
                        for (int k = 1; k <= Vecs.Length(); k++)
                        {
                            if (Vecs.Value(k).IsEqual(Normal, LinTol, AngTol))
                            {
                                isNew = false;
                                break;
                            }
                        }
                        if (isNew)
                        {
                            NewVecs.Append(Normal);
                            isHalfSpace = GeomPlate_BuildAveragePlane.HalfSpace(NewVecs, Vecs, Aset, LinTol, AngTol);
                            if (!isHalfSpace)
                            {
                                NewVecs.ChangeValue(1).Reverse();
                                isHalfSpace = GeomPlate_BuildAveragePlane.HalfSpace(NewVecs, Vecs, Aset, LinTol, AngTol);
                            }
                            if (!isHalfSpace)
                            {
                                break;
                            }
                        }
                    }
                } // for (i = 1; i <= NTPntCont; i++)

                if (isHalfSpace)
                {
                    bool NullExist = true;
                    while (NullExist)
                    {
                        NullExist = false;
                        for (i = 1; i <= Vecs.Length(); i++)
                        {
                            if (Vecs.Value(i).SquareMagnitude() == 0.0)
                            {
                                NullExist = true;
                                Vecs.Remove(i);
                                break;
                            }
                        }
                    }
                    GeomPlate_BuildAveragePlane BAP = new GeomPlate_BuildAveragePlane(Vecs, Pts);
                    double u1, u2, v1, v2;
                    BAP.MinMaxBox(out u1, out u2, out v1, out v2);
                    // The space is greater for projections
                    double du = u2 - u1;
                    double dv = v2 - v1;
                    u1 -= du;
                    u2 += du;
                    v1 -= dv;
                    v2 += dv;
                    mySurfInit = new Geom_RectangularTrimmedSurface(BAP.Plane()!, u1, u2, v1, v2);
                }
            } // if (isHalfSpace)
            if (!isHalfSpace)
            {
                myIsLinear = false;
                nopt = 2;
            }
        } // if (NTLinCont != 0 && (CourbeJoint = CourbeJointive( myTol3d )) && IsOrderG1())

        if (NTLinCont != 0)
        {
            TrierTab(myInitOrder!); // Reorder the table of transformations
        }

        if (nopt != 3)
        {
            if (NTPntCont != 0)
            {
                nopt = 1; // Calculate by the method of plane of inertia
            }
            else if (!CourbeJoint || NTLinCont != myNbBounds)
            { //    throw Standard_Failure("Curves are not joined");
                nopt = 1;
            }

            double LenT = 0;
            int Npt = 0;
            int NTPoint = 20 * NTLinCont;
            int i;
            for (i = 1; i <= NTLinCont; i++)
            {
                LenT += myLinCont.Value(i).Length();
            }
            for (i = 1; i <= NTLinCont; i++)
            {
                int NbPoint1 = (int)(NTPoint * (myLinCont.Value(i).Length()) / LenT);
                if (NbPoint1 < 10)
                {
                    NbPoint1 = 10;
                }

                Npt += NbPoint1; // unused but set for debug
            }
            _ = Npt;
            // Table containing a cloud of points for calculation of the plane
            NCollection_HArray1<gp_Pnt> Pts = new NCollection_HArray1<gp_Pnt>(1, 20 * NTLinCont + NTPntCont);
            int NbPoint = 20;
            double Uinit, Ufinal, Uif;
            for (i = 1; i <= NTLinCont; i++)
            {
                Uinit = myLinCont.Value(i).FirstParameter();
                Ufinal = myLinCont.Value(i).LastParameter();
                Uif = Ufinal - Uinit;
                if (mySense!.Value(i) == 1)
                {
                    Uinit = Ufinal;
                    Uif = -Uif;
                }
                for (int j = 0; j < NbPoint; j++)
                { // Number of points per curve = 20
                  // Linear distribution
                    double Inter = j * Uif / (NbPoint);
                    gp_Pnt P;
                    myLinCont.Value(i).D0(Uinit + Inter, out P);
                    Pts.SetValue(Np++, P);
                }
            }
            for (i = 1; i <= NTPntCont; i++)
            {
                gp_Pnt P;
                myPntCont.Value(i).D0(out P);
                Pts.SetValue(Np++, P);
            }
            if (!CourbeJoint)
            {
                myNbBounds = 0;
            }
            GeomPlate_BuildAveragePlane BAP = new GeomPlate_BuildAveragePlane(Pts, NbPoint * myNbBounds, myTol3d / 1000, popt, nopt);
            if (!BAP.IsPlane())
            {
                return;
            }
            double u1, u2, v1, v2;
            BAP.MinMaxBox(out u1, out u2, out v1, out v2);
            // The space is greater for projections
            double du = u2 - u1;
            double dv = v2 - v1;
            u1 -= du;
            u2 += du;
            v1 -= dv;
            v2 += dv;
            mySurfInit = new Geom_RectangularTrimmedSurface(BAP.Plane()!, u1, u2, v1, v2);
        } // if (nopt != 3)

        // Comparing metrics of curves and projected curves
        if (NTLinCont != 0 && myIsLinear)
        {
            Geom_Surface InitPlane = ((Geom_RectangularTrimmedSurface)mySurfInit!).BasisSurface();

            double Ratio = 0.0, R1 = 2.0, R2 = 0.6; //R1 = 3, R2 = 0.5;//R1 = 1.4, R2 = 0.8; //R1 = 5., R2 = 0.2;
            GeomAdaptor_Surface hsur = new GeomAdaptor_Surface(InitPlane);
            int NbPoint = 20;

            for (int i = 1; i <= NTLinCont && myIsLinear; i++)
            {
                double FirstPar = myLinCont.Value(i).FirstParameter();
                double LastPar = myLinCont.Value(i).LastParameter();
                double Uif = (LastPar - FirstPar) / (NbPoint);

                Adaptor3d_Curve Curve = myLinCont.Value(i).Curve3d();
                ProjLib_CompProjectedCurve ProjCurve = new ProjLib_CompProjectedCurve(hsur, Curve, myTol3d, myTol3d);
                Adaptor3d_CurveOnSurface AProj = new Adaptor3d_CurveOnSurface(ProjCurve, hsur);

                gp_Pnt P;
                gp_Vec DerC, DerCproj;
                for (int j = 1; j < NbPoint && myIsLinear; j++)
                {
                    double Inter = FirstPar + j * Uif;
                    Curve.D1(Inter, out P, out DerC);
                    AProj.D1(Inter, out P, out DerCproj);

                    double A1 = DerC.Magnitude();
                    double A2 = DerCproj.Magnitude();
                    if (A2 <= 1.0e-20)
                    {
                        Ratio = 1.0e20;
                    }
                    else
                    {
                        Ratio = A1 / A2;
                    }
                    if (Ratio > R1 || Ratio < R2)
                    {
                        myIsLinear = false;
                        break;
                    }
                }
            }
        } // comparing metrics of curves and projected curves

        if (!myIsLinear)
        {
            myPlanarSurfInit = mySurfInit;
            double u1, v1, u2, v2;
            mySurfInit!.Bounds(out u1, out v1, out u2, out v2);
            GeomAdaptor_Surface Surf = new GeomAdaptor_Surface(mySurfInit);
            myTolU = Surf.UResolution(myTol3d);
            myTolV = Surf.VResolution(myTol3d);
            myProj.Initialize(Surf, u1, v1, u2, v2, myTolU, myTolV);

            //======================================================================
            // Projection of curves
            //======================================================================
            int i;
            for (i = 1; i <= NTLinCont; i++)
            {
                if (myLinCont.Value(i).Curve2dOnSurf() is null)
                {
                    myLinCont.ChangeValue(i).SetCurve2dOnSurf(ProjectCurve(myLinCont.Value(i).Curve3d()));
                }
            }

            //======================================================================
            // Projection of points
            //======================================================================
            for (i = 1; i <= NTPntCont; i++)
            {
                gp_Pnt P;
                myPntCont.Value(i).D0(out P);
                if (!myPntCont.Value(i).HasPnt2dOnSurf())
                {
                    myPntCont.ChangeValue(i).SetPnt2dOnSurf(ProjectPoint(P));
                }
            }

            //======================================================================
            // Number of points by curve
            //======================================================================
            if ((NTLinCont != 0) && (myNbPtsOnCur != 0))
            {
                CalculNbPtsInit();
            }

            //======================================================================
            // Management of incompatibilities between curves
            //======================================================================
            NCollection_HArray1<NCollection_Sequence<double>>? PntInter = null;
            NCollection_HArray1<NCollection_Sequence<double>>? PntG1G1 = null;
            if (NTLinCont != 0)
            {
                PntInter = NewSequences(NTLinCont);
                PntG1G1 = NewSequences(NTLinCont);
                Intersect(PntInter, PntG1G1);
            }

            //====================================================================
            // Discretization of curves
            //====================================================================
            Discretise(PntInter!, PntG1G1!);
            //====================================================================
            // Preparation of points of constraint for plate
            //====================================================================
            LoadCurve(0, 0);
            if (myPntCont.Length() != 0)
            {
                LoadPoint(0, 0);
            }
            //====================================================================
            // Construction of the surface
            //====================================================================
            using Message_ProgressScope aPS = new Message_ProgressScope(theProgress, "ComputeSurfInit", 1);
            myPlate.SolveTI(2, ComputeAnisotropie(), aPS.Next());
            if (theProgress.UserBreak())
            {
                return;
            }

            if (!myPlate.IsDone())
            {
                return;
            }

            myGeomPlateSurface = new GeomPlate_Surface(mySurfInit, myPlate);

            GeomPlate_MakeApprox App = new GeomPlate_MakeApprox(myGeomPlateSurface, myTol3d, 1, 3, 15 * myTol3d, -1, GeomAbs_C0);
            mySurfInit = App.Surface();

            mySurfInitIsGive = true;
            myPlate.Init(); // Reset

            for (i = 1; i <= NTLinCont; i++)
            {
                Geom2d_Curve? NullCurve = null;
                myLinCont.ChangeValue(i).SetCurve2dOnSurf(NullCurve);
            }
        }
    }

    // Find intersections between 2d curves
    // If the intersection is compatible (in cases G1-G1)
    // remove the point on one of two curves
    private void Intersect(NCollection_HArray1<NCollection_Sequence<double>> PntInter,
                           NCollection_HArray1<NCollection_Sequence<double>> PntG1G1)
    {
        int NTLinCont = myLinCont.Length();
        Geom2dInt_GInter Intersection = new Geom2dInt_GInter();
        Geom2dAdaptor_Curve Ci = new Geom2dAdaptor_Curve(), Cj = new Geom2dAdaptor_Curve();
        IntRes2d_IntersectionPoint int2d;
        gp_Pnt P1, P2;
        gp_Pnt2d P2d;
        gp_Vec2d V2d;
        for (int i = 1; i <= NTLinCont; i++)
        {
            //  Find the intersection with each curve including the curve itself
            Ci.Load(myLinCont.Value(i).Curve2dOnSurf()!);
            for (int j = i; j <= NTLinCont; j++)
            {
                Cj.Load(myLinCont.Value(j).Curve2dOnSurf()!);
                if (i == j)
                {
                    Intersection.Perform(Ci, myTol2d * 10, myTol2d * 10);
                }
                else
                {
                    Intersection.Perform(Ci, Cj, myTol2d * 10, myTol2d * 10);
                }

                if (!Intersection.IsEmpty())
                { // there is one intersection
                    int nbpt = Intersection.NbPoints();
                    // number of points of intersection
                    for (int k = 1; k <= nbpt; k++)
                    {
                        int2d = Intersection.Point(k);
                        myLinCont.Value(i).D0(int2d.ParamOnFirst(), out P1);
                        myLinCont.Value(j).D0(int2d.ParamOnSecond(), out P2);
                        if (P1.Distance(P2) < myTol3d)
                        { // 2D intersection corresponds to close 3D points.
                          // Note the interval, in which the point needs to be removed
                          // to avoid duplications, which cause
                          // error in plate. The point on curve i is removed;
                          // the point on curve j is preserved;
                          // the length of interval is a length 2d
                          // corresponding in 3d to myTol3d
                            double tolint = Ci.Resolution(myTol3d);
                            Ci.D1(int2d.ParamOnFirst(), out P2d, out V2d);
                            double aux = V2d.Magnitude();
                            if (aux > 1.0e-7)
                            {
                                aux = myTol3d / aux;
                                if (aux > 100 * tolint)
                                {
                                    tolint *= 100;
                                }
                                else
                                {
                                    tolint = aux;
                                }
                            }
                            else
                            {
                                tolint *= 100;
                            }

                            PntInter[i].Append(int2d.ParamOnFirst() - tolint);
                            PntInter[i].Append(int2d.ParamOnFirst() + tolint);
                            // If G1-G1
                            if ((myLinCont.Value(i).Order() == 1) && (myLinCont.Value(j).Order() == 1))
                            {
                                gp_Vec v11, v12, v13, v14, v15, v16, v21, v22, v23, v24, v25, v26;
                                myLinCont.Value(i).D2(int2d.ParamOnFirst(), out P1, out v11, out v12, out v13, out v14, out v15);
                                myLinCont.Value(j).D2(int2d.ParamOnSecond(), out P2, out v21, out v22, out v23, out v24, out v25);
                                v16 = v11 ^ v12;
                                v26 = v21 ^ v22;
                                double ant = v16.Angle(v26);
                                if (ant > (M_PI / 2))
                                {
                                    ant = M_PI - ant;
                                }
                                if ((Math.Abs(v16 * v15 - v16 * v25) > (myTol3d / 1000))
                                    || (Math.Abs(ant) > myTol3d / 1000))
                                // Non-compatible ==> remove zone in constraint G1
                                // corresponding to 3D tolerance of 0.01
                                {
                                    double coin;
                                    double Tol = 100 * myTol3d;
                                    double A1;
                                    gp_Pnt2d P1temp, P2temp;
                                    gp_Vec2d V1, V2;
                                    myLinCont.Value(i).Curve2dOnSurf()!.D1(int2d.ParamOnFirst(), out P1temp, out V1);
                                    myLinCont.Value(j).Curve2dOnSurf()!.D1(int2d.ParamOnSecond(), out P2temp, out V2);
                                    A1 = V1.Angle(V2);
                                    if (A1 > (M_PI / 2))
                                    {
                                        A1 = M_PI - A1;
                                    }
                                    if (Math.Abs(Math.Abs(A1) - M_PI) < myTolAng)
                                    {
                                        Tol = 100000 * myTol3d;
                                    }

                                    coin = Ci.Resolution(Tol);
                                    double Par1 = int2d.ParamOnFirst() - coin, Par2 = int2d.ParamOnFirst() + coin;
                                    // Storage of the interval for curve i
                                    PntG1G1[i].Append(Par1);
                                    PntG1G1[i].Append(Par2);
                                    coin = Cj.Resolution(Tol);
                                    Par1 = int2d.ParamOnSecond() - coin;
                                    Par2 = int2d.ParamOnSecond() + coin;
                                    // Storage of the interval for curve j
                                    PntG1G1[j].Append(Par1);
                                    PntG1G1[j].Append(Par2);
                                }
                            }
                            // If G0-G1
                            if ((myLinCont.Value(i).Order() == 0 && myLinCont.Value(j).Order() == 1)
                                || (myLinCont.Value(i).Order() == 1 && myLinCont.Value(j).Order() == 0))
                            {
                                gp_Vec vec, vecU, vecV, N;
                                if (myLinCont.Value(i).Order() == 0)
                                {
                                    Adaptor3d_Curve theCurve = myLinCont.Value(i).Curve3d();
                                    theCurve.D1(int2d.ParamOnFirst(), out P1, out vec);
                                    myLinCont.Value(j).D1(int2d.ParamOnSecond(), out P2, out vecU, out vecV);
                                }
                                else
                                {
                                    Adaptor3d_Curve theCurve = myLinCont.Value(j).Curve3d();
                                    theCurve.D1(int2d.ParamOnSecond(), out P2, out vec);
                                    myLinCont.Value(i).D1(int2d.ParamOnFirst(), out P1, out vecU, out vecV);
                                }
                                N = vecU ^ vecV;
                                double Angle = vec.Angle(N);
                                Angle = Math.Abs(M_PI / 2 - Angle);
                                if (Angle > myTolAng / 10.0) //????????? //if (std::abs( scal ) > myTol3d/100)
                                {
                                    // Non-compatible ==> one removes zone in constraint G0 and G1
                                    // corresponding to 3D tolerance of 0.01
                                    double coin;
                                    double Tol = 100 * myTol3d;
                                    double A1;
                                    gp_Pnt2d P1temp, P2temp;
                                    gp_Vec2d V1, V2;
                                    myLinCont.Value(i).Curve2dOnSurf()!.D1(int2d.ParamOnFirst(), out P1temp, out V1);
                                    myLinCont.Value(j).Curve2dOnSurf()!.D1(int2d.ParamOnSecond(), out P2temp, out V2);
                                    A1 = V1.Angle(V2);
                                    if (A1 > M_PI / 2)
                                    {
                                        A1 = M_PI - A1;
                                    }
                                    if (Math.Abs(Math.Abs(A1) - M_PI) < myTolAng)
                                    {
                                        Tol = 100000 * myTol3d;
                                    }
                                    if (myLinCont.Value(i).Order() == 1)
                                    {
                                        coin = Ci.Resolution(Tol);
                                        coin *= Angle / myTolAng * 10.0;
                                        double Par1 = int2d.ParamOnFirst() - coin;
                                        double Par2 = int2d.ParamOnFirst() + coin;
                                        // Storage of the interval for curve i
                                        PntG1G1[i].Append(Par1);
                                        PntG1G1[i].Append(Par2);
                                    }
                                    else
                                    {
                                        coin = Cj.Resolution(Tol);
                                        coin *= Angle / myTolAng * 10.0;
                                        double Par1 = int2d.ParamOnSecond() - coin;
                                        double Par2 = int2d.ParamOnSecond() + coin;
                                        // Storage of the interval for curve j
                                        PntG1G1[j].Append(Par1);
                                        PntG1G1[j].Append(Par2);
                                    }
                                }
                            }
                        } // if (P1.Distance( P2 ) < myTol3d)
                        else
                        {
                            // 2D intersection corresponds to extended 3D points.
                            // Note the interval where it is necessary to remove
                            // the points to avoid duplications causing
                            // error in plate. The point on curve i is removed,
                            // the point on curve j is preserved.
                            // The length of interval is 2D length
                            // corresponding to the distance of points in 3D to myTol3d
                            double tolint, Dist;
                            Dist = P1.Distance(P2);
                            tolint = Ci.Resolution(Dist);
                            PntInter[i].Append(int2d.ParamOnFirst() - tolint);
                            PntInter[i].Append(int2d.ParamOnFirst() + tolint);
                            if (j != i)
                            {
                                tolint = Cj.Resolution(Dist);
                                PntInter[j].Append(int2d.ParamOnSecond() - tolint);
                                PntInter[j].Append(int2d.ParamOnSecond() + tolint);
                            }
                        }
                    }
                }
            }
        }
    }

    // Discretize curves according to parameters
    // the table of sequences Parcont contains all
    // parameter of points on curves
    // Field myPlateCont contains parameter of points on a plate;
    // it excludes duplicate points and incompatible zones.
    // The first part corresponds to verification of compatibility
    // and to removal of duplicate points.
    private void Discretise(NCollection_HArray1<NCollection_Sequence<double>> PntInter,
                            NCollection_HArray1<NCollection_Sequence<double>> PntG1G1)
    {
        int NTLinCont = myLinCont.Length();
        bool ACR;
        Geom2d_Curve? C2d;
        Geom2dAdaptor_Curve AC2d = new Geom2dAdaptor_Curve();
        Law_Interpol acrlaw = new Law_Interpol();
        myPlateCont = NewSequences(NTLinCont);
        myParCont = NewSequences(NTLinCont);

        //===========================================================================
        // Construction of the table containing parameters of constraint points
        //===========================================================================
        double Uinit, Ufinal, Length2d = 0, Inter;
        double CurLength;
        int NbPnt_i, NbPtInter, NbPtG1G1;
        GeomPlate_CurveConstraint LinCont;

        for (int i = 1; i <= NTLinCont; i++)
        {
            LinCont = myLinCont.Value(i);
            Uinit = LinCont.FirstParameter();
            Ufinal = LinCont.LastParameter();
            C2d = LinCont.Curve2dOnSurf();
            ACR = (C2d is not null);
            if (ACR)
            {
                // Construct a law close to curvilinear abscissa
                if (C2d is not null)
                {
                    AC2d.Load(C2d);
                }
                int ii, Nbint = 20;
                double U;
                NCollection_Array1<gp_Pnt2d> tabP2d = new NCollection_Array1<gp_Pnt2d>(1, Nbint + 1);
                tabP2d[1].SetY(Uinit);
                tabP2d[1].SetX(0.0);
                tabP2d[Nbint + 1].SetY(Ufinal);
                Length2d = GCPnts_AbscissaPoint.Length(AC2d, Uinit, Ufinal);

                tabP2d[Nbint + 1].SetX(Length2d);
                for (ii = 2; ii <= Nbint; ii++)
                {
                    U = Uinit + (Ufinal - Uinit) * ((1 - Math.Cos((ii - 1) * M_PI / (Nbint))) / 2);
                    tabP2d[ii].SetY(U);
                    tabP2d[ii].SetX(GCPnts_AbscissaPoint.Length(AC2d, Uinit, U));
                }
                acrlaw.Set(tabP2d);
            }

            NbPnt_i = (int)(LinCont.NbPoints());
            NbPtInter = PntInter.Value(i).Length();
            NbPtG1G1 = PntG1G1.Value(i).Length();

            for (int j = 1; j <= NbPnt_i; j++)
            {
                // Distribution of points in cosine following ACR 2D
                // To avoid points of accumulation in 2D
                if (j == NbPnt_i)
                {
                    Inter = Ufinal; // to avoid bug on Sun
                }
                else if (ACR)
                {
                    CurLength = Length2d * (1 - Math.Cos((j - 1) * M_PI / (NbPnt_i - 1))) / 2;
                    Inter = acrlaw.Value(CurLength);
                }
                else
                {
                    Inter = Uinit + (Ufinal - Uinit) * ((1 - Math.Cos((j - 1) * M_PI / (NbPnt_i - 1))) / 2);
                }
                myParCont[i].Append(Inter); // add a point
                if (NbPtInter != 0)
                {
                    for (int l = 1; l <= NbPtInter; l += 2)
                    {
                        // check if the point Inter is in the interval
                        // PntInter[i] PntInter[i+1]
                        // in which case it is not necessary to store it (problem with duplicates)
                        if ((Inter > PntInter.Value(i).Value(l)) && (Inter < PntInter.Value(i).Value(l + 1)))
                        {
                            l = NbPtInter + 2;
                            // leave the loop without storing the point
                        }
                        else
                        {
                            if (l + 1 >= NbPtInter)
                            {
                                // one has parsed the entire table : the point
                                // does not belong to a common point interval
                                if (NbPtG1G1 != 0)
                                {
                                    // if there exists an incompatible interval
                                    for (int k = 1; k <= NbPtG1G1; k += 2)
                                    {
                                        if ((Inter > PntG1G1.Value(i).Value(k)) && (Inter < PntG1G1.Value(i).Value(k + 1)))
                                        {
                                            k = NbPtG1G1 + 2; // to leave the loop
                                            // Add points of constraint G0
                                            gp_Pnt P3d, PP, Pdif = new gp_Pnt();
                                            gp_Pnt2d P2d;

                                            AC2d.D0(Inter, out P2d);
                                            LinCont.D0(Inter, out P3d);
                                            mySurfInit!.D0(P2d.Coord(1), P2d.Coord(2), out PP);
                                            Pdif.SetCoord(-PP.Coord(1) + P3d.Coord(1),
                                                          -PP.Coord(2) + P3d.Coord(2),
                                                          -PP.Coord(3) + P3d.Coord(3));
                                            Plate_PinpointConstraint PC = new Plate_PinpointConstraint(P2d.XY(), Pdif.XYZ(), 0, 0);
                                            myPlate.Load(PC);
                                        }
                                        else // the point does not belong to interval G1
                                        {
                                            if (k + 1 >= NbPtG1G1)
                                            {
                                                myPlateCont[i].Append(Inter);
                                                // add the point
                                            }
                                        }
                                    }
                                }
                                else
                                {
                                    myPlateCont[i].Append(Inter);
                                    // add the point
                                }
                            }
                        }
                    }
                }
                else
                {
                    if (NbPtG1G1 != 0) // there exist an incompatible interval
                    {
                        for (int k = 1; k <= NbPtG1G1; k += 2)
                        {
                            if ((Inter > PntG1G1.Value(i).Value(k)) && (Inter < PntG1G1.Value(i).Value(k + 1)))
                            {
                                k = NbPtG1G1 + 2; // to leave the loop
                                // Add points of constraint G0
                                gp_Pnt P3d, PP, Pdif = new gp_Pnt();
                                gp_Pnt2d P2d;

                                AC2d.D0(Inter, out P2d);
                                LinCont.D0(Inter, out P3d);
                                mySurfInit!.D0(P2d.Coord(1), P2d.Coord(2), out PP);
                                Pdif.SetCoord(-PP.Coord(1) + P3d.Coord(1),
                                              -PP.Coord(2) + P3d.Coord(2),
                                              -PP.Coord(3) + P3d.Coord(3));
                                Plate_PinpointConstraint PC = new Plate_PinpointConstraint(P2d.XY(), Pdif.XYZ(), 0, 0);
                                myPlate.Load(PC);
                            }
                            else // the point does not belong to interval G1
                            {
                                if (k + 1 >= NbPtG1G1)
                                {
                                    myPlateCont[i].Append(Inter);
                                    // add the point
                                }
                            }
                        }
                    }
                    else
                    {
                        if (((!mySurfInitIsGive)
                             && (new Geom2dAdaptor_Curve(LinCont.Curve2dOnSurf()!).GetType() != GeomAbs_CurveType.GeomAbs_Circle))
                            || ((j > 1) && (j < NbPnt_i)))
                        { // exclude extremities
                            myPlateCont[i].Append(Inter); // add the point
                        }
                    }
                }
            }
        }
    }

    // Calculate the number of points by curve depending on the
    // length for the first iteration
    private void CalculNbPtsInit()
    {
        double LenT = 0;
        int NTLinCont = myLinCont.Length();
        int NTPoint = (int)(myNbPtsOnCur * NTLinCont);
        int i;

        for (i = 1; i <= NTLinCont; i++)
        {
            LenT += myLinCont.Value(i).Length();
        }
        for (i = 1; i <= NTLinCont; i++)
        {
            int Cont = myLinCont.Value(i).Order();
            switch (Cont)
            {
                case 0: // Case G0 *1.2
                    myLinCont.ChangeValue(i).SetNbPoints((int)(1.2 * NTPoint * (myLinCont.Value(i).Length()) / LenT));
                    break;
                case 1: // Case G1 *1
                    myLinCont.ChangeValue(i).SetNbPoints((int)(NTPoint * (myLinCont.Value(i).Length()) / LenT));
                    break;
                case 2: // Case G2 *0.7
                    myLinCont.ChangeValue(i).SetNbPoints((int)(0.7 * NTPoint * (myLinCont.Value(i).Length()) / LenT));
                    break;
            }
            if (myLinCont.Value(i).NbPoints() < 3)
            {
                myLinCont.ChangeValue(i).SetNbPoints(3);
            }
        }
    }

    // Starting from table myParCont load all the points noted in plate
    private void LoadCurve(int NbBoucle, int OrderMax = 2)
    {
        gp_Pnt P3d, Pdif = new gp_Pnt(), PP;
        gp_Pnt2d P2d;
        int NTLinCont = myLinCont.Length(), i, j;
        int Tang, Nt;

        for (i = 1; i <= NTLinCont; i++)
        {
            GeomPlate_CurveConstraint CC = myLinCont.Value(i);
            if (CC.Order() != -1)
            {
                Tang = Min(CC.Order(), OrderMax);
                Nt = myPlateCont!.Value(i).Length();
                if (Tang != -1)
                {
                    for (j = 1; j <= Nt; j++)
                    { // Loading of points G0 on boundaries
                        CC.D0(myPlateCont.Value(i).Value(j), out P3d);
                        if (CC.ProjectedCurve() is not null)
                        {
                            P2d = CC.ProjectedCurve()!.Value(myPlateCont.Value(i).Value(j));
                        }
                        else
                        {
                            if (CC.Curve2dOnSurf() is not null)
                            {
                                P2d = CC.Curve2dOnSurf()!.Value(myPlateCont.Value(i).Value(j));
                            }
                            else
                            {
                                P2d = ProjectPoint(P3d);
                            }
                        }
                        mySurfInit!.D0(P2d.Coord(1), P2d.Coord(2), out PP);
                        Pdif.SetCoord(-PP.Coord(1) + P3d.Coord(1),
                                      -PP.Coord(2) + P3d.Coord(2),
                                      -PP.Coord(3) + P3d.Coord(3));
                        Plate_PinpointConstraint PC = new Plate_PinpointConstraint(P2d.XY(), Pdif.XYZ(), 0, 0);
                        myPlate.Load(PC);

                        // Loading of points G1
                        if (Tang == 1)
                        { // ==1
                            gp_Vec V1, V2, V3, V4;
                            CC.D1(myPlateCont.Value(i).Value(j), out PP, out V1, out V2);
                            mySurfInit.D1(P2d.Coord(1), P2d.Coord(2), out PP, out V3, out V4);

                            Plate_D1 D1final = new Plate_D1(V1.XYZ(), V2.XYZ());
                            Plate_D1 D1init = new Plate_D1(V3.XYZ(), V4.XYZ());
                            if (!myFree)
                            {
                                Plate_GtoCConstraint GCC = new Plate_GtoCConstraint(P2d.XY(), D1init, D1final);
                                myPlate.Load(GCC);
                            }
                            else if (NbBoucle == 1)
                            {
                                Plate_FreeGtoCConstraint FreeGCC = new Plate_FreeGtoCConstraint(P2d.XY(), D1init, D1final);
                                myPlate.Load(FreeGCC);
                            }
                            else
                            {
                                gp_Vec DU = new gp_Vec(), DV = new gp_Vec(), Normal, DerPlateU, DerPlateV;

                                Normal = V1 ^ V2;
                                // Normal.Normalize();
                                double norm = Normal.Magnitude();
                                if (norm > 1.0e-12)
                                {
                                    Normal /= norm;
                                }
                                DerPlateU = new gp_Vec(myPrevPlate.EvaluateDerivative(P2d.XY(), 1, 0));
                                DerPlateV = new gp_Vec(myPrevPlate.EvaluateDerivative(P2d.XY(), 0, 1));

                                DU.SetLinearForm(-(V3 + DerPlateU).Dot(Normal), Normal, DerPlateU);
                                DV.SetLinearForm(-(V4 + DerPlateV).Dot(Normal), Normal, DerPlateV);
                                Plate_PinpointConstraint PinU = new Plate_PinpointConstraint(P2d.XY(), DU.XYZ(), 1, 0);
                                Plate_PinpointConstraint PinV = new Plate_PinpointConstraint(P2d.XY(), DV.XYZ(), 0, 1);
                                myPlate.Load(PinU);
                                myPlate.Load(PinV);
                            }
                        }
                        // Loading of points G2
                        if (Tang == 2) // ==2
                        {
                            gp_Vec V1, V2, V3, V4, V5, V6, V7, V8, V9, V10;
                            CC.D2(myPlateCont.Value(i).Value(j), out PP, out V1, out V2, out V5, out V6, out V7);
                            mySurfInit.D2(P2d.Coord(1), P2d.Coord(2), out PP, out V3, out V4, out V8, out V9, out V10);

                            Plate_D1 D1final = new Plate_D1(V1.XYZ(), V2.XYZ());
                            Plate_D1 D1init = new Plate_D1(V3.XYZ(), V4.XYZ());
                            Plate_D2 D2final = new Plate_D2(V5.XYZ(), V6.XYZ(), V7.XYZ());
                            Plate_D2 D2init = new Plate_D2(V8.XYZ(), V9.XYZ(), V10.XYZ());
                            Plate_GtoCConstraint GCC = new Plate_GtoCConstraint(P2d.XY(), D1init, D1final, D2init, D2final);
                            myPlate.Load(GCC);
                        }
                    }
                }
            }
        }
    }

    // Loading of points of point constraints (OCCT doesn't use NbBoucle)
    private void LoadPoint(int NbBoucle, int OrderMax = 2)
    {
        gp_Pnt P3d, Pdif = new gp_Pnt(), PP;
        gp_Pnt2d P2d;
        int NTPntCont = myPntCont.Length();
        int Tang, i;

        // Loading of points of point constraints
        for (i = 1; i <= NTPntCont; i++)
        {
            myPntCont.Value(i).D0(out P3d);
            P2d = myPntCont.Value(i).Pnt2dOnSurf();
            mySurfInit!.D0(P2d.Coord(1), P2d.Coord(2), out PP);
            Pdif.SetCoord(-PP.Coord(1) + P3d.Coord(1),
                          -PP.Coord(2) + P3d.Coord(2),
                          -PP.Coord(3) + P3d.Coord(3));
            Plate_PinpointConstraint PC = new Plate_PinpointConstraint(P2d.XY(), Pdif.XYZ(), 0, 0);
            myPlate.Load(PC);
            Tang = Min(myPntCont.Value(i).Order(), OrderMax);
            if (Tang == 1)
            { // ==1
                gp_Vec V1, V2, V3, V4;
                myPntCont.Value(i).D1(out PP, out V1, out V2);
                mySurfInit.D1(P2d.Coord(1), P2d.Coord(2), out PP, out V3, out V4);
                Plate_D1 D1final = new Plate_D1(V1.XYZ(), V2.XYZ());
                Plate_D1 D1init = new Plate_D1(V3.XYZ(), V4.XYZ());
                if (!myFree)
                {
                    Plate_GtoCConstraint GCC = new Plate_GtoCConstraint(P2d.XY(), D1init, D1final);
                    myPlate.Load(GCC);
                }
                else
                {
                    Plate_FreeGtoCConstraint FreeGCC = new Plate_FreeGtoCConstraint(P2d.XY(), D1init, D1final);
                    myPlate.Load(FreeGCC);
                }
            }
            // Loading of points G2 GeomPlate_PlateG0Criterion
            if (Tang == 2) // ==2
            {
                gp_Vec V1, V2, V3, V4, V5, V6, V7, V8, V9, V10;
                myPntCont.Value(i).D2(out PP, out V1, out V2, out V5, out V6, out V7);
                mySurfInit.D2(P2d.Coord(1), P2d.Coord(2), out PP, out V3, out V4, out V8, out V9, out V10);
                Plate_D1 D1final = new Plate_D1(V1.XYZ(), V2.XYZ());
                Plate_D1 D1init = new Plate_D1(V3.XYZ(), V4.XYZ());
                Plate_D2 D2final = new Plate_D2(V5.XYZ(), V6.XYZ(), V7.XYZ());
                Plate_D2 D2init = new Plate_D2(V8.XYZ(), V9.XYZ(), V10.XYZ());
                Plate_GtoCConstraint GCC = new Plate_GtoCConstraint(P2d.XY(), D1init, D1final, D2init, D2final);
                myPlate.Load(GCC);
            }
        }
    }

    private bool VerifSurface(int NbBoucle)
    {
        //======================================================================
        //    Calculate errors
        //======================================================================
        int NTLinCont = myLinCont.Length();
        bool Result = true;

        // variable for error calculation
        myG0Error = 0;
        myG1Error = 0;
        myG2Error = 0;

        for (int i = 1; i <= NTLinCont; i++)
        {
            GeomPlate_CurveConstraint LinCont;
            LinCont = myLinCont.Value(i);
            if (LinCont.Order() != -1)
            {
                int NbPts_i = myParCont!.Value(i).Length();
                if (NbPts_i < 3)
                {
                    NbPts_i = 4;
                }
                NCollection_HArray1<double> tdist = new NCollection_HArray1<double>(1, NbPts_i - 1);
                NCollection_HArray1<double> tang = new NCollection_HArray1<double>(1, NbPts_i - 1);
                NCollection_HArray1<double> tcourb = new NCollection_HArray1<double>(1, NbPts_i - 1);

                EcartContraintesMil(i, tdist, tang, tcourb);

                double diffDistMax = 0, diffAngMax = 0;
                int NdiffDist = 0, NdiffAng = 0;

                for (int j = 1; j < NbPts_i; j++)
                {
                    if (tdist.Value(j) > myG0Error)
                    {
                        myG0Error = tdist.Value(j);
                    }
                    if (tang.Value(j) > myG1Error)
                    {
                        myG1Error = tang.Value(j);
                    }
                    if (tcourb.Value(j) > myG2Error)
                    {
                        myG2Error = tcourb.Value(j);
                    }
                    double U;
                    if (myParCont.Value(i).Length() > 3)
                    {
                        U = (myParCont.Value(i).Value(j) + myParCont.Value(i).Value(j + 1)) / 2;
                    }
                    else
                    {
                        U = LinCont.FirstParameter()
                            + (LinCont.LastParameter() - LinCont.FirstParameter()) * (j - 1) / (NbPts_i - 2);
                    }
                    double diffDist = tdist.Value(j) - LinCont.G0Criterion(U), diffAng;
                    if (LinCont.Order() > 0)
                    {
                        diffAng = tang.Value(j) - LinCont.G1Criterion(U);
                    }
                    else
                    {
                        diffAng = 0;
                    }
                    // find the maximum variation of error and calculate the average
                    if (diffDist > 0)
                    {
                        diffDist = diffDist / LinCont.G0Criterion(U);
                        if (diffDist > diffDistMax)
                        {
                            diffDistMax = diffDist;
                        }
                        NdiffDist++;
                    }
                    else if ((diffAng > 0) && (LinCont.Order() == 1))
                    {
                        diffAng = diffAng / myLinCont.Value(i).G1Criterion(U);
                        if (diffAng > diffAngMax)
                        {
                            diffAngMax = diffAng;
                        }
                        NdiffAng++;
                    }
                }

                if (NdiffDist > 0)
                { // at least one point is not acceptable in G0
                    double Coef;
                    if (LinCont.Order() == 0)
                    {
                        Coef = 0.6 * Math.Log(diffDistMax + 7.4);
                        // 7.4 corresponds to the calculation of min. coefficient = 1.2 is e^1.2/0.6
                    }
                    else
                    {
                        Coef = Math.Log(diffDistMax + 3.3);
                    }
                    // 3.3 corresponds to calculation of min. coefficient = 1.2 donc e^1.2
                    if (Coef > 3)
                    {
                        Coef = 3;
                    }
                    // experimentally after the coefficient becomes bad for L cases
                    if ((NbBoucle > 1) && (diffDistMax > 2))
                    {
                        Coef = 1.6;
                    }

                    if (LinCont.NbPoints() >= Math.Floor(LinCont.NbPoints() * Coef))
                    {
                        Coef = 2; // to provide increase of the number of points
                    }

                    LinCont.SetNbPoints((int)(LinCont.NbPoints() * Coef));
                    Result = false;
                }
                else if (NdiffAng > 0) // at least 1 point is not acceptable in G1
                {
                    double Coef = 1.5;
                    if ((LinCont.NbPoints() + 1) >= Math.Floor(LinCont.NbPoints() * Coef))
                    {
                        Coef = 2;
                    }

                    LinCont.SetNbPoints((int)(LinCont.NbPoints() * Coef));
                    Result = false;
                }
            }
        }
        if (!Result)
        {
            if (myFree && NbBoucle == 1)
            {
                myPrevPlate.Copy(myPlate);
            }
            myPlate.Init();
        }
        return Result;
    }

    // The distance, angle and curvature gap at the last point constraint (OCCT's callers discard them)
    private void VerifPoints(out double Dist, out double Ang, out double Curv)
    {
        int NTPntCont = myPntCont.Length();
        gp_Pnt Pi, Pf;
        gp_Pnt2d P2d;
        gp_Vec v1i, v1f, v2i, v2f, v3i, v3f;
        Ang = 0;
        Dist = 0;
        Curv = 0;
        GeomPlate_PointConstraint PntCont;
        for (int i = 1; i <= NTPntCont; i++)
        {
            PntCont = myPntCont.Value(i);
            switch (PntCont.Order())
            {
                case 0:
                    P2d = PntCont.Pnt2dOnSurf();
                    PntCont.D0(out Pi);
                    myGeomPlateSurface!.D0(P2d.Coord(1), P2d.Coord(2), out Pf);
                    Dist = Pf.Distance(Pi);
                    break;
                case 1:
                    PntCont.D1(out Pi, out v1i, out v2i);
                    P2d = PntCont.Pnt2dOnSurf();
                    myGeomPlateSurface!.D1(P2d.Coord(1), P2d.Coord(2), out Pf, out v1f, out v2f);
                    Dist = Pf.Distance(Pi);
                    v3i = v1i ^ v2i;
                    v3f = v1f ^ v2f;
                    Ang = v3f.Angle(v3i);
                    if (Ang > (M_PI / 2))
                    {
                        Ang = M_PI - Ang;
                    }
                    break;
                case 2:
                    Geom_Surface Splate = myGeomPlateSurface!;
                    LocalAnalysis_SurfaceContinuity CG2 = new LocalAnalysis_SurfaceContinuity();
                    P2d = PntCont.Pnt2dOnSurf();
                    GeomLProp_SLProps Prop = new GeomLProp_SLProps(Splate, P2d.Coord(1), P2d.Coord(2), 2, 0.001);
                    CG2.ComputeAnalysis(Prop, PntCont.LPropSurf(), GeomAbs_G2);
                    Dist = CG2.C0Value();
                    Ang = CG2.G1Angle();
                    Curv = CG2.G2CurvatureGap();
                    break;
            }
        }
    }

    private double ComputeAnisotropie()
    {
        if (myAnisotropie)
        {
            // Temporary
            return 1.0;
        }
        else
        {
            return 1.0;
        }
    }

    private bool IsOrderG1()
    {
        bool result = true;
        for (int i = 1; i <= myLinCont.Length(); i++)
        {
            if (myLinCont.Value(i).Order() < 1)
            {
                result = false;
                break;
            }
        }
        return result;
    }
}
