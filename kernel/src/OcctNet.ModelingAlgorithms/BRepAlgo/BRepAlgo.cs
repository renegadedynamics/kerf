// Port of: src/ModelingAlgorithms/TKBool/BRepAlgo/BRepAlgo.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepAlgo/BRepAlgo.cxx
// Port of: src/ModelingAlgorithms/TKBool/BRepAlgo/BRepAlgo_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Bento.Collections;
using static Occt.BRepCheck_Status;
using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_Shape;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Tools on wires and shapes: concatenation of a wire's edges into smoother ones, conversion of a wire into arcs and
/// segments, and validity checks.
/// </summary>
/// <remarks>
/// <para>
/// The results are frozen. <see cref="ConcatenateWireC0"/> computes missing 3D curves and fixes the wire before
/// concatenating, which OCCT does on the caller's wire: the port works on a copy of a frozen wire
/// (<see cref="BRepTools_CopyAtEntry"/>) and gives the caller's vertices back in the result where they weren't changed
/// (decision 5963).
/// </para>
/// <para>
/// <see cref="IsValid(NCollection_List{TopoDS_Shape}, in TopoDS_Shape, bool, bool)"/> resets the SameParameter flags
/// of new faces' edges found invalid and recomputes them, as OCCT does, in the result: a frozen result throws at that
/// write (BRepTools.Update skips frozen faces).
/// </para>
/// </remarks>
public static class BRepAlgo
{
    // The minimal tolerance of approximation (edges can be defined with yet smaller tolerance)
    private const double MINIMAL_TOLERANCE = 0.0001;

    private struct OrientedCurve
    {
        public Geom2d_TrimmedCurve Curve;
        public bool IsReverse;

        public readonly gp_Pnt2d Point(bool isEnd)
        {
            if (isEnd == IsReverse)
            {
                return Curve.StartPoint();
            }
            return Curve.EndPoint();
        }
    }

    /// <summary>
    /// The wire <paramref name="theWire"/> on <paramref name="theFace"/> converted into arcs of circles and segments,
    /// smooth where its tangents differ by less than <paramref name="theAngleTol"/>, within the largest tolerance of its
    /// edges (at least 1e-4); null if the wire has no edge.
    /// </summary>
    public static TopoDS_Wire ConvertWire(in TopoDS_Wire theWire, double theAngleTol, in TopoDS_Face theFace)
    {
        TopoDS_Wire aResult = InternalConvertWire(theWire, theAngleTol, theFace);
        aResult.TShape()?.FreezeGraph();
        return aResult;
    }

    /// <summary><see cref="ConvertWire"/> without freezing the result, for ported callers that build on it.</summary>
    internal static TopoDS_Wire InternalConvertWire(in TopoDS_Wire theWire, double theAngleTol, in TopoDS_Face theFace)
    {
        TopoDS_Wire aResult = new();
        double aMaxTol = 0.0;
        Geom_Surface aSurf = BRep_Tool.Surface(theFace)!;
        NCollection_DynamicArray<OrientedCurve> vecCurve = new();

        BRepTools_WireExplorer anExpE = new(theWire, theFace);
        // Explore the edges in the current wire, in their connection order
        for (; anExpE.More(); anExpE.Next())
        {
            TopoDS_Edge anEdge = anExpE.Current();
            BRepAdaptor_Curve2d aCurve = new(anEdge, theFace);
            double aTol = BRep_Tool.Tolerance(anEdge);
            if (aTol < MINIMAL_TOLERANCE)
            {
                aTol = MINIMAL_TOLERANCE;
            }
            if (aTol > aMaxTol)
            {
                aMaxTol = aTol;
            }
            Geom2dConvert_ApproxArcsSegments anAlgo = new(aCurve, aTol, theAngleTol);
            IBentoList<Geom2d_Curve> aResultApprox = anAlgo.GetResult();

            // Form the array of approximated elementary curves
            if (anEdge.Orientation() == TopAbs_REVERSED)
            {
                for (int iCrv = aResultApprox.Count; iCrv > 0; iCrv--)
                {
                    Geom2d_Curve? aCrv = aResultApprox[iCrv - 1];
                    if (aCrv is not null)
                    {
                        ref OrientedCurve anOCurve = ref vecCurve.Append(new OrientedCurve());
                        anOCurve.Curve = (Geom2d_TrimmedCurve)aCrv;
                        anOCurve.IsReverse = true;
                    }
                }
            }
            else
            {
                for (int iCrv = 1; iCrv <= aResultApprox.Count; iCrv++)
                {
                    Geom2d_Curve? aCrv = aResultApprox[iCrv - 1];
                    if (aCrv is not null)
                    {
                        ref OrientedCurve anOCurve = ref vecCurve.Append(new OrientedCurve());
                        anOCurve.Curve = (Geom2d_TrimmedCurve)aCrv;
                        anOCurve.IsReverse = false;
                    }
                }
            }
        }

        if (vecCurve.Length() > 0)
        {
            // Build the first vertex
            BRep_Builder aVBuilder = new();
            gp_Pnt2d[] aPnt = [vecCurve.Value(0).Point(false), vecCurve.Value(vecCurve.Length() - 1).Point(true)];
            double aDist = aPnt[0].Distance(aPnt[1]);
            if (aDist > aMaxTol + Precision.Confusion())
            {
                aDist = Precision.Confusion();
            }
            else
            {
                aDist = 0.5 * aDist + Precision.Confusion();
                aPnt[0] = new gp_Pnt2d(0.5 * (aPnt[0].XY() + aPnt[1].XY()));
            }
            gp_Pnt aPnt3d = aSurf.Value(aPnt[0].X(), aPnt[0].Y());
            TopoDS_Vertex aFirstVertex = new();
            aVBuilder.MakeVertex(ref aFirstVertex, aPnt3d, aDist);

            // Loop creating edges
            BRepBuilderAPI_MakeWire aMkWire = new();
            TopoDS_Edge anEdgeRes;
            TopoDS_Vertex aVertex = aFirstVertex;
            for (int iCrv = 0; iCrv < vecCurve.Length(); iCrv++)
            {
                OrientedCurve anOCurve = vecCurve.Value(iCrv);
                TopoDS_Vertex aNextVertex = new();
                aPnt[0] = anOCurve.Point(true);
                if (iCrv == vecCurve.Length() - 1)
                {
                    aPnt[1] = vecCurve.Value(0).Point(false);
                    aDist = aPnt[0].Distance(aPnt[1]);
                    if (aDist > aMaxTol + Precision.Confusion())
                    {
                        aPnt3d = aSurf.Value(aPnt[0].X(), aPnt[0].Y());
                        aVBuilder.MakeVertex(ref aNextVertex, aPnt3d, Precision.Confusion());
                    }
                    else
                    {
                        aNextVertex = aFirstVertex;
                    }
                }
                else
                {
                    aPnt[1] = vecCurve.Value(iCrv + 1).Point(false);
                    aDist = 0.5 * (aPnt[0].Distance(aPnt[1])) + Precision.Confusion();
                    aPnt[0] = new gp_Pnt2d(0.5 * (aPnt[0].XY() + aPnt[1].XY()));
                    aPnt3d = aSurf.Value(aPnt[0].X(), aPnt[0].Y());
                    aVBuilder.MakeVertex(ref aNextVertex, aPnt3d, aDist);
                }
                double[] aParam = [anOCurve.Curve.FirstParameter(), anOCurve.Curve.LastParameter()];
                if (anOCurve.IsReverse)
                {
                    BRepBuilderAPI_MakeEdge aMkEdge = new(anOCurve.Curve, aSurf, aNextVertex, aVertex, aParam[0], aParam[1]);
                    anEdgeRes = aMkEdge.InternalEdge();
                    anEdgeRes.Orientation(TopAbs_REVERSED);
                }
                else
                {
                    BRepBuilderAPI_MakeEdge aMkEdge = new(anOCurve.Curve, aSurf, aVertex, aNextVertex, aParam[0], aParam[1]);
                    anEdgeRes = aMkEdge.InternalEdge();
                }
                aVertex = aNextVertex;
                aMkWire.Add(anEdgeRes);
            }

            if (aMkWire.IsDone())
            {
                aResult = aMkWire.InternalWire();
            }
        }
        return aResult;
    }

    /// <summary>
    /// The face <paramref name="theFace"/> with every wire converted by <see cref="ConvertWire"/>; null if the face
    /// couldn't be made.
    /// </summary>
    public static TopoDS_Face ConvertFace(in TopoDS_Face theFace, double theAngleTolerance)
    {
        TopoDS_Face aResult = new();
        Geom_Surface aSurf = BRep_Tool.Surface(theFace)!;
        BRepBuilderAPI_MakeFace aMkFace = new(aSurf, Precision.Confusion());

        TopExp_Explorer anExp = new(theFace, TopAbs_WIRE);
        for (; anExp.More(); anExp.Next())
        {
            TopoDS_Wire aWire = TopoDS.Wire(anExp.Current());
            TopoDS_Wire aNewWire = InternalConvertWire(aWire, theAngleTolerance, theFace);
            aMkFace.Add(aNewWire);
        }
        if (aMkFace.IsDone())
        {
            aResult = aMkFace.InternalFace();
        }
        aResult.TShape()?.FreezeGraph();
        return aResult;
    }

    /// <summary>
    /// A wire whose edges are C1 (or G1, with <paramref name="Option"/>) made from <paramref name="W"/>, whose edges
    /// may meet with G1 continuity: the vertices between such edges are removed.
    /// </summary>
    public static TopoDS_Wire ConcatenateWire(in TopoDS_Wire W, GeomAbs_Shape Option, double TolAngular = 1.0e-4)
    {
        int nb_curve, // number of curves in the Wire
            index;
        BRepTools_WireExplorer WExp = new(W);
        TopoDS_Edge edge;
        double First = 0.0, Last = 0.0, // extremal values for the curve
            First0 = 0.0, toler = 0.0, tolleft, tolright; // Vertex tolerances
        TopoDS_Vertex Vfirst = new(), Vlast = new(); // Vertex of the Wire
        gp_Pnt Pfirst, Plast; //, Pint;  corresponding points

        BRepLib_MakeWire MakeResult = new();
        double closed_tolerance = 0.0;
        bool closed_flag = false;

        nb_curve = 0;

        while (WExp.More())
        { // computation of the curve number
            nb_curve++;
            WExp.Next();
        }

        if (nb_curve > 1)
        {
            NCollection_Array1<Geom_BSplineCurve> tab = new(0, nb_curve - 1); // array of the wire's curve
            NCollection_Array1<double> tabtolvertex = new(0, nb_curve - 2); // array of the tolerance's vertex

            WExp.Init(W);

            for (index = 0; index < nb_curve; index++)
            { // main loop
                edge = WExp.Current();
                Geom_Curve aCurve = BRep_Tool.Curve(edge, out TopLoc_Location L, out First, out Last)!;
                Geom_TrimmedCurve aTrCurve = new(aCurve, First, Last);
                tab[index] = GeomConvert.CurveToBSplineCurve(aTrCurve); // storage in a array
                tab[index].Transform(L.Transformation());
                GeomConvert.C0BSplineToC1BSplineCurve(ref tab[index], Precision.Confusion());

                if (index >= 1)
                { // continuity test loop
                    if (edge.Orientation() == TopAbs_REVERSED)
                    {
                        tab[index].Reverse();
                    }
                    tolleft = BRep_Tool.Tolerance(TopExp.LastVertex(edge));
                    tolright = BRep_Tool.Tolerance(TopExp.FirstVertex(edge));
                    tabtolvertex[index - 1] = Max(tolleft, tolright);
                }

                if (index == 0)
                { // storage of the first edge features
                    First0 = First;
                    if (edge.Orientation() == TopAbs_REVERSED)
                    { //(useful for the closed wire)
                        Vfirst = TopExp.LastVertex(edge);
                        tab[index].Reverse();
                    }
                    else
                    {
                        Vfirst = TopExp.FirstVertex(edge);
                    }
                }

                if (index == nb_curve - 1)
                { // storage of the last edge features
                    if (edge.Orientation() == TopAbs_REVERSED)
                    {
                        Vlast = TopExp.FirstVertex(edge);
                    }
                    else
                    {
                        Vlast = TopExp.LastVertex(edge);
                    }
                }
                WExp.Next();
            }

            if (BRep_Tool.Tolerance(Vfirst) > BRep_Tool.Tolerance(Vlast))
            { // computation of the closing tolerance
                toler = BRep_Tool.Tolerance(Vfirst);
            }
            else
            {
                toler = BRep_Tool.Tolerance(Vlast);
            }

            Pfirst = BRep_Tool.Pnt(Vfirst);
            Plast = BRep_Tool.Pnt(Vlast);

            if ((Pfirst.Distance(Plast) <= toler) && // C0 continuity test at the closing point
                (GeomLProp.Continuity(tab[nb_curve - 1], tab[0], Last, First0, true, true, toler, TolAngular)
                 >= GeomAbs_G1))
            {
                closed_tolerance = toler; // if ClosedG1!=0 it will be True and
                closed_flag = true;
            } // with the toler value
            NCollection_HArray1<Geom_BSplineCurve> concatcurve; // array of the concatenated curves
            if (Option == GeomAbs_G1)
            {
                GeomConvert.ConcatG1(tab,
                                     tabtolvertex,
                                     out concatcurve,
                                     ref closed_flag,
                                     closed_tolerance); // G1 concatenation
            }
            else
            {
                GeomConvert.ConcatC1(tab,
                                     tabtolvertex,
                                     out _,
                                     out concatcurve,
                                     ref closed_flag,
                                     closed_tolerance); // C1 concatenation
            }

            for (index = 0; index <= (concatcurve.Length() - 1); index++)
            { // building of the resulting Wire
                BRepLib_MakeEdge EdgeBuilder = new(concatcurve.Value(index));
                edge = EdgeBuilder.InternalEdge();
                MakeResult.Add(edge);
            }
        }
        else
        {
            WExp.Init(W);

            edge = WExp.Current();
            Geom_Curve aC = BRep_Tool.Curve(edge, out TopLoc_Location L, out First, out Last)!;
            Geom_BSplineCurve aBS = GeomConvert.CurveToBSplineCurve(new Geom_TrimmedCurve(aC, First, Last));
            aBS.Transform(L.Transformation());
            GeomConvert.C0BSplineToC1BSplineCurve(ref aBS, Precision.Confusion());
            if (edge.Orientation() == TopAbs_REVERSED)
            {
                aBS.Reverse();
            }

            BRepLib_MakeEdge EdgeBuilder = new(aBS);
            edge = EdgeBuilder.InternalEdge();
            MakeResult.Add(edge);
        }
        return MakeResult.Wire();
    }

    /// <summary>
    /// An edge made from the wire <paramref name="aWire"/> (whose edges are first given 3D curves and fixed), possibly
    /// with sharp corners: its curve may be C0. Consecutive edges on the same line or conic are merged first.
    /// </summary>
    public static TopoDS_Edge ConcatenateWireC0(in TopoDS_Wire aWire)
    {
        double LinTol = Precision.Confusion();
        double AngTol = Precision.Angular();

        TopoDS_Edge ResEdge = new();

        // OCCT.NET: copy at entry (decision 5963): OCCT computes the 3D curves on the caller's wire
        BRepTools_CopyAtEntry anEntry = new(aWire);
        TopoDS_Wire theWire = TopoDS.Wire(anEntry.Shape());
        BRepLib.BuildCurves3d(theWire);
        ShapeFix_Shape Fixer = new(theWire);
        Fixer.SetPrecision(LinTol);
        Fixer.SetMaxTolerance(LinTol);
        Fixer.Perform();
        theWire = TopoDS.Wire(Fixer.InternalShape());

        NCollection_Sequence<Geom_Curve> CurveSeq = new();
        NCollection_Sequence<double> FparSeq = new();
        NCollection_Sequence<double> LparSeq = new();
        NCollection_Sequence<double> TolSeq = new();
        NCollection_Sequence<bool> IsFwdSeq = new();
        GeomAbs_CurveType CurType = GeomAbs_OtherCurve;
        TopoDS_Vertex FirstVertex = new(), LastVertex;

        BRepTools_WireExplorer wexp = new(theWire);

        for (; wexp.More(); wexp.Next())
        {
            TopoDS_Edge anEdge = wexp.Current();
            Geom_Curve? aCurve = BRep_Tool.Curve(anEdge, out double fpar, out double lpar);
            if (aCurve is null)
            {
                continue;
            }

            GeomAdaptor_Curve aGACurve = new(aCurve);
            GeomAbs_CurveType aType = aGACurve.GetType();
            Geom_Curve aBasisCurve = aGACurve.Curve()!;
            bool isFwd = wexp.Orientation() != TopAbs_REVERSED;

            if (aBasisCurve.IsPeriodic())
            {
                ElCLib.AdjustPeriodic(aBasisCurve.FirstParameter(),
                                      aBasisCurve.LastParameter(),
                                      Precision.PConfusion(),
                                      ref fpar,
                                      ref lpar);
            }

            if (CurveSeq.IsEmpty())
            {
                CurveSeq.Append(aCurve);
                FparSeq.Append(fpar);
                LparSeq.Append(lpar);
                IsFwdSeq.Append(isFwd);
                CurType = aType;
                FirstVertex = wexp.CurrentVertex();
            }
            else
            {
                bool isSameCurve = false;
                double NewFpar = RealFirst(), NewLpar = RealLast();
                GeomAdaptor_Curve GAprevcurve = new(CurveSeq.Last());

                if (ReferenceEquals(aCurve, CurveSeq.Last()))
                {
                    NewFpar = fpar;
                    NewLpar = lpar;
                    isSameCurve = true;
                }
                else if (aType == CurType)
                {
                    switch (aType)
                    {
                        case GeomAbs_Line:
                        {
                            gp_Lin aLine = aGACurve.Line();
                            gp_Lin PrevLine = GAprevcurve.Line();

                            if (aLine.Contains(PrevLine.Location(), LinTol)
                                && aLine.Direction().IsParallel(PrevLine.Direction(), AngTol))
                            {
                                gp_Pnt P1 = ElCLib.Value(fpar, aLine);
                                gp_Pnt P2 = ElCLib.Value(lpar, aLine);

                                NewFpar = ElCLib.Parameter(PrevLine, P1);
                                NewLpar = ElCLib.Parameter(PrevLine, P2);
                                isSameCurve = true;
                            }
                            break;
                        }
                        case GeomAbs_Circle:
                        {
                            gp_Circ aCircle = aGACurve.Circle();
                            gp_Circ PrevCircle = GAprevcurve.Circle();

                            if (aCircle.Location().Distance(PrevCircle.Location()) <= LinTol
                                && Math.Abs(aCircle.Radius() - PrevCircle.Radius()) <= LinTol
                                && aCircle.Axis().IsParallel(PrevCircle.Axis(), AngTol))
                            {
                                gp_Pnt P1 = ElCLib.Value(fpar, aCircle);
                                gp_Pnt P2 = ElCLib.Value(lpar, aCircle);

                                NewFpar = ElCLib.Parameter(PrevCircle, P1);
                                NewLpar = ElCLib.Parameter(PrevCircle, P2);
                                isSameCurve = true;
                            }
                            break;
                        }
                        case GeomAbs_Ellipse:
                        {
                            gp_Elips anEllipse = aGACurve.Ellipse();
                            gp_Elips PrevEllipse = GAprevcurve.Ellipse();

                            if (anEllipse.Focus1().Distance(PrevEllipse.Focus1()) <= LinTol
                                && anEllipse.Focus2().Distance(PrevEllipse.Focus2()) <= LinTol
                                && Math.Abs(anEllipse.MajorRadius() - PrevEllipse.MajorRadius()) <= LinTol
                                && Math.Abs(anEllipse.MinorRadius() - PrevEllipse.MinorRadius()) <= LinTol
                                && anEllipse.Axis().IsParallel(PrevEllipse.Axis(), AngTol))
                            {
                                gp_Pnt P1 = ElCLib.Value(fpar, anEllipse);
                                gp_Pnt P2 = ElCLib.Value(lpar, anEllipse);

                                NewFpar = ElCLib.Parameter(PrevEllipse, P1);
                                NewLpar = ElCLib.Parameter(PrevEllipse, P2);
                                isSameCurve = true;
                            }
                            break;
                        }
                        case GeomAbs_Hyperbola:
                        {
                            gp_Hypr aHypr = aGACurve.Hyperbola();
                            gp_Hypr PrevHypr = GAprevcurve.Hyperbola();

                            if (aHypr.Focus1().Distance(PrevHypr.Focus1()) <= LinTol
                                && aHypr.Focus2().Distance(PrevHypr.Focus2()) <= LinTol
                                && Math.Abs(aHypr.MajorRadius() - PrevHypr.MajorRadius()) <= LinTol
                                && Math.Abs(aHypr.MinorRadius() - PrevHypr.MinorRadius()) <= LinTol
                                && aHypr.Axis().IsParallel(PrevHypr.Axis(), AngTol))
                            {
                                gp_Pnt P1 = ElCLib.Value(fpar, aHypr);
                                gp_Pnt P2 = ElCLib.Value(lpar, aHypr);

                                NewFpar = ElCLib.Parameter(PrevHypr, P1);
                                NewLpar = ElCLib.Parameter(PrevHypr, P2);
                                isSameCurve = true;
                            }
                            break;
                        }
                        case GeomAbs_Parabola:
                        {
                            gp_Parab aParab = aGACurve.Parabola();
                            gp_Parab PrevParab = GAprevcurve.Parabola();

                            if (aParab.Location().Distance(PrevParab.Location()) <= LinTol
                                && aParab.Focus().Distance(PrevParab.Focus()) <= LinTol
                                && Math.Abs(aParab.Focal() - PrevParab.Focal()) <= LinTol
                                && aParab.Axis().IsParallel(PrevParab.Axis(), AngTol))
                            {
                                gp_Pnt P1 = ElCLib.Value(fpar, aParab);
                                gp_Pnt P2 = ElCLib.Value(lpar, aParab);

                                NewFpar = ElCLib.Parameter(PrevParab, P1);
                                NewLpar = ElCLib.Parameter(PrevParab, P2);
                                isSameCurve = true;
                            }
                            break;
                        }
                        default:
                            break;
                    } // end of switch
                } // end of else

                if (isSameCurve)
                {
                    bool isSameDir = isFwd == IsFwdSeq.Last();

                    if (aBasisCurve.IsPeriodic())
                    {
                        // Treat periodic curves.
                        double aPeriod = aBasisCurve.Period();

                        if (isSameDir)
                        {
                            // Check if first parameter is greater then the last one.
                            while (NewFpar > NewLpar)
                            {
                                NewFpar -= aPeriod;
                            }
                        }
                        else
                        { // !isSameDir
                            // Check if last parameter is greater then the first one.
                            while (NewLpar > NewFpar)
                            {
                                NewLpar -= aPeriod;
                            }

                            // Change parameters
                            double aTmpPar = NewLpar;

                            NewLpar = NewFpar;
                            NewFpar = aTmpPar;
                        }

                        // Udjust parameters on periodic curves.
                        if (IsFwdSeq.Last())
                        {
                            // The current curve should be after the previous one.
                            ElCLib.AdjustPeriodic(LparSeq.Last(),
                                                  LparSeq.Last() + aPeriod,
                                                  Precision.PConfusion(),
                                                  ref NewFpar,
                                                  ref NewLpar);
                        }
                        else
                        {
                            // The current curve should be before the previous one.
                            ElCLib.AdjustPeriodic(FparSeq.Last() - aPeriod,
                                                  FparSeq.Last(),
                                                  Precision.PConfusion(),
                                                  ref NewFpar,
                                                  ref NewLpar);
                        }
                    }
                    else if (!isSameDir)
                    {
                        // Not periodic curves. Opposite dirs.
                        double aTmpPar = NewLpar;

                        NewLpar = NewFpar;
                        NewFpar = aTmpPar;
                    }

                    if (IsFwdSeq.Last())
                    {
                        // Update last parameter
                        LparSeq[LparSeq.Length()] = NewLpar;
                    }
                    else
                    {
                        // Update first parameter
                        FparSeq[FparSeq.Length()] = NewFpar;
                    }
                }
                else
                {
                    // Add new curve.
                    CurveSeq.Append(aCurve);
                    FparSeq.Append(fpar);
                    LparSeq.Append(lpar);
                    IsFwdSeq.Append(isFwd);
                    TolSeq.Append(BRep_Tool.Tolerance(wexp.CurrentVertex()));
                    CurType = aType;
                }
            }
        }

        LastVertex = wexp.CurrentVertex();
        TolSeq.Append(BRep_Tool.Tolerance(LastVertex));

        bool isReverse = false;
        if (!IsFwdSeq.IsEmpty())
        {
            isReverse = !IsFwdSeq.Value(1);
        }

        TopoDS_Vertex FirstVtx_final, LastVtx_final;
        if (isReverse)
        {
            FirstVtx_final = LastVertex;
            LastVtx_final = FirstVertex;
        }
        else
        {
            FirstVtx_final = FirstVertex;
            LastVtx_final = LastVertex;
        }
        FirstVtx_final.Orientation(TopAbs_FORWARD);
        LastVtx_final.Orientation(TopAbs_REVERSED);

        if (CurveSeq.IsEmpty())
        {
            anEntry.Finish(ResEdge);
            return ResEdge;
        }

        int nb_curve = CurveSeq.Length(); // number of curves
        NCollection_Array1<Geom_BSplineCurve> tab = new(0, nb_curve - 1); // array of the curves
        NCollection_Array1<double> tabtolvertex = new(0, nb_curve - 1); //(0,nb_curve-2);  //array of the tolerances

        int i;

        if (nb_curve > 1)
        {
            for (i = 1; i <= nb_curve; i++)
            {
                if (CurveSeq.Value(i).GetType() == typeof(Geom_TrimmedCurve))
                {
                    CurveSeq[i] = ((Geom_TrimmedCurve)CurveSeq.Value(i)).BasisCurve();
                }

                Geom_TrimmedCurve aTrCurve = new(CurveSeq.Value(i), FparSeq.Value(i), LparSeq.Value(i));
                tab[i - 1] = GeomConvert.CurveToBSplineCurve(aTrCurve);
                GeomConvert.C0BSplineToC1BSplineCurve(ref tab[i - 1], Precision.Confusion());

                if (!IsFwdSeq.Value(i))
                {
                    tab[i - 1].Reverse();
                }

                if (i > 1)
                {
                    tabtolvertex[i - 2] = TolSeq.Value(i - 1) * 5.0;
                }
            }
            tabtolvertex[nb_curve - 1] = TolSeq.Value(TolSeq.Length()) * 5.0;

            bool closed_flag = false;
            double closed_tolerance = 0.0;
            if (FirstVertex.IsSame(LastVertex)
                && GeomLProp.Continuity(tab[0],
                                        tab[nb_curve - 1],
                                        tab[0].FirstParameter(),
                                        tab[nb_curve - 1].LastParameter(),
                                        false,
                                        false,
                                        LinTol,
                                        AngTol)
                   >= GeomAbs_G1)
            {
                closed_flag = true;
                closed_tolerance = BRep_Tool.Tolerance(FirstVertex);
            }

            GeomConvert.ConcatC1(tab,
                                 tabtolvertex,
                                 out _,
                                 out NCollection_HArray1<Geom_BSplineCurve> concatcurve,
                                 ref closed_flag,
                                 closed_tolerance); // C1 concatenation

            if (concatcurve.Length() > 1)
            {
                double MaxTolVer = LinTol;
                for (i = 1; i <= TolSeq.Length(); i++)
                {
                    if (TolSeq.Value(i) > MaxTolVer)
                    {
                        MaxTolVer = TolSeq.Value(i);
                    }
                }
                MaxTolVer *= 5.0;

                GeomConvert_CompCurveToBSplineCurve Concat = new(concatcurve.Value(concatcurve.Lower()));
                for (i = concatcurve.Lower() + 1; i <= concatcurve.Upper(); i++)
                {
                    Concat.Add(concatcurve.Value(i), MaxTolVer, true);
                }

                concatcurve.SetValue(concatcurve.Lower(), Concat.BSplineCurve()!);
            }

            if (isReverse)
            {
                concatcurve.Value(concatcurve.Lower()).Reverse();
            }
            BRepLib_MakeEdge aMkEdge = new(concatcurve.Value(concatcurve.Lower()),
                                           FirstVtx_final,
                                           LastVtx_final,
                                           concatcurve.Value(concatcurve.Lower()).FirstParameter(),
                                           concatcurve.Value(concatcurve.Lower()).LastParameter());
            ResEdge = aMkEdge.InternalEdge();
        }
        else
        {
            if (CurveSeq.Value(1).GetType() == typeof(Geom_TrimmedCurve))
            {
                CurveSeq[1] = ((Geom_TrimmedCurve)CurveSeq.Value(1)).BasisCurve();
            }

            Geom_Curve aCopyCurve = (Geom_Curve)CurveSeq.Value(1).Copy();

            BRepLib_MakeEdge aMkEdge = new(aCopyCurve, FirstVtx_final, LastVtx_final, FparSeq.Value(1), LparSeq.Value(1));
            ResEdge = aMkEdge.InternalEdge();
        }

        if (isReverse)
        {
            ResEdge.Reverse();
        }

        ResEdge = TopoDS.Edge(anEntry.Finish(ResEdge));
        ResEdge.TShape()?.FreezeGraph();
        return ResEdge;
    }

    /// <summary>Whether the shape is valid (BRepCheck, geometry included).</summary>
    public static bool IsValid(in TopoDS_Shape S)
    {
        BRepCheck_Analyzer ana = new(S);
        return ana.IsValid();
    }

    /// <summary>
    /// Whether the faces of <paramref name="theResult"/> that are not faces of <paramref name="theArgs"/> are valid
    /// (all its faces when the arguments are empty), its shells too, and closed with <paramref name="closedSolid"/>;
    /// without <paramref name="GeomCtrl"/>, new vertices' and edges' geometry and wire self-intersections are not
    /// checked.
    /// </summary>
    public static bool IsValid(NCollection_List<TopoDS_Shape> theArgs,
                               in TopoDS_Shape theResult,
                               bool closedSolid = false,
                               bool GeomCtrl = true)
    {
        if (theResult.IsNull())
        {
            return true;
        }
        bool validate = false;

        NCollection_Map<TopoDS_Shape> allFaces = new(TopTools_ShapeMapHasher.Instance);
        TopExp_Explorer tEx = new();
        NCollection_List<TopoDS_Shape>.Iterator itLOS = new();
        for (itLOS.Initialize(theArgs); itLOS.More(); itLOS.Next())
        {
            if (itLOS.Value().IsSame(theResult))
            {
                validate = true;
                break;
            }
            for (tEx.Init(itLOS.Value(), TopAbs_FACE); tEx.More(); tEx.Next())
            {
                allFaces.Add(tEx.Current());
            }
        }

        TopoDS_Compound toCheck = new();

        if (allFaces.IsEmpty())
        {
            if (validate)
            {
                return true;
            }
            BRepCheck_Analyzer ana = new(theResult, GeomCtrl);
            if (!ana.IsValid())
            {
                return false;
            }
        }
        else if (!validate)
        {
            BRep_Builder bB = new();
            TopoDS_Face curf;
            for (tEx.Init(theResult, TopAbs_FACE); tEx.More(); tEx.Next())
            {
                curf = TopoDS.Face(tEx.Current());
                if (!allFaces.Contains(curf))
                {
                    if (toCheck.IsNull())
                    {
                        bB.MakeCompound(ref toCheck);
                    }
                    BRepTools.Update(curf);
                    bB.Add(toCheck, curf);
                }
            }
            if (toCheck.IsNull())
            {
                validate = true;
            }
            else
            {
                BRepCheck_Analyzer ana = new(toCheck, true);
                if (!ana.IsValid())
                {
                    // Check if the problem is not just BRepCheck_InvalidSameParameterFlag
                    NCollection_List<BRepCheck_Status>.Iterator itl = new();
                    BRepCheck_Status sta;
                    for (tEx.Init(toCheck, TopAbs_FACE); tEx.More(); tEx.Next())
                    {
                        if (ana.Result(tEx.Current()) is not null)
                        {
                            for (itl.Initialize(ana.Result(tEx.Current())!.Status()); itl.More(); itl.Next())
                            {
                                sta = itl.Value();
                                // If a face is incorrect
                                if (sta != BRepCheck_NoError)
                                {
                                    NCollection_List<BRepCheck_Status>.Iterator ilt = new();
                                    TopExp_Explorer exp = new();
                                    for (exp.Init(tEx.Current(), TopAbs_EDGE); exp.More(); exp.Next())
                                    {
                                        BRepCheck_Result res = ana.Result(exp.Current())!;
                                        for (res.InitContextIterator(); res.MoreShapeInContext(); res.NextShapeInContext())
                                        {
                                            if (res.ContextualShape().IsSame(tEx.Current()))
                                            {
                                                for (ilt.Initialize(res.StatusOnShape()); ilt.More(); ilt.Next())
                                                {
                                                    sta = ilt.Value();
                                                    // If an edge is BRepCheck_InvalidSameParameterFlag or
                                                    // BRepCheck_InvalidSameRangeFlag, it is forced
                                                    if (sta == BRepCheck_InvalidSameParameterFlag
                                                        || sta == BRepCheck_InvalidSameRangeFlag)
                                                    {
                                                        bB.SameRange(TopoDS.Edge(exp.Current()), false);
                                                        bB.SameParameter(TopoDS.Edge(exp.Current()), false);
                                                        BRepLib.SameParameter(TopoDS.Edge(exp.Current()),
                                                                              BRep_Tool.Tolerance(TopoDS.Edge(exp.Current())));
                                                        break;
                                                    }
                                                }
                                            }
                                        }
                                    }
                                    break;
                                }
                            }
                        }
                    }
                    // Remake control (there can be a problem of another type orb the one that cannot be
                    // corrected
                    ana.Init(toCheck, true);
                    if (!ana.IsValid())
                    {
                        return false;
                    }
                }
            }
        }

        BRepCheck_Shell? HR = null;
        for (tEx.Init(theResult, TopAbs_SHELL); tEx.More(); tEx.Next())
        {
            if (HR is null)
            {
                HR = new BRepCheck_Shell(TopoDS.Shell(tEx.Current()));
            }
            else
            {
                HR.Init(tEx.Current());
            }
            if (HR.Status().First() != BRepCheck_NoError)
            {
                return false;
            }
            if (HR.Orientation(false) != BRepCheck_NoError)
            {
                return false;
            }
            if (closedSolid)
            {
                if (HR.Closed() != BRepCheck_NoError)
                {
                    return false;
                }
            }
        }

        return true;
    }

    /// <summary>Whether the shape is valid topologically (BRepCheck without the geometric controls).</summary>
    public static bool IsTopologicallyValid(in TopoDS_Shape S)
    {
        BRepCheck_Analyzer ana = new(S, false);
        return ana.IsValid();
    }
}
