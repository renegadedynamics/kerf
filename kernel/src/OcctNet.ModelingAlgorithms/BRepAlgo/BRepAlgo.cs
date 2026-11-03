// Port of: src/ModelingAlgorithms/TKBool/BRepAlgo/BRepAlgo.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepAlgo/BRepAlgo.cxx
// Port of: src/ModelingAlgorithms/TKBool/BRepAlgo/BRepAlgo_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Schematiq.Collections;
using static Occt.BRepCheck_Status;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// Tools on wires and shapes: concatenation of a wire's edges into smoother ones, conversion of a wire into arcs and
/// segments, and validity checks.
/// </summary>
/// <remarks>
/// <para>
/// The results are frozen. <see cref="ConcatenateWireC0"/> computes missing 3D curves and fixes the wire before
/// concatenating, which OCCT does on the caller's wire: the port works on a copy of a frozen wire
/// (<see cref="CopyAtEntry"/>) and gives the caller's vertices back in the result where they weren't changed
/// (decision 793).
/// </para>
/// <para>
/// <see cref="IsValid(NCollection_List{Shape}, in Shape, bool, bool)"/> resets the SameParameter flags
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
        public TrimmedCurve2D Curve;
        public bool IsReverse;

        public readonly Point2D Point(bool isEnd)
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
    public static Wire ConvertWire(in Wire theWire, double theAngleTol, in Face theFace)
    {
        Wire aResult = InternalConvertWire(theWire, theAngleTol, theFace);
        aResult.TShape()?.FreezeGraph();
        return aResult;
    }

    /// <summary><see cref="ConvertWire"/> without freezing the result, for ported callers that build on it.</summary>
    internal static Wire InternalConvertWire(in Wire theWire, double theAngleTol, in Face theFace)
    {
        Wire aResult = new();
        double aMaxTol = 0.0;
        Surface aSurf = Tool.Surface(theFace)!;
        NCollection_DynamicArray<OrientedCurve> vecCurve = new();

        WireExplorer anExpE = new(theWire, theFace);
        // Explore the edges in the current wire, in their connection order
        for (; anExpE.More(); anExpE.Next())
        {
            Edge anEdge = anExpE.Current();
            EdgeCurve2DAdaptor aCurve = new(anEdge, theFace);
            double aTol = Tool.Tolerance(anEdge);
            if (aTol < MINIMAL_TOLERANCE)
            {
                aTol = MINIMAL_TOLERANCE;
            }
            if (aTol > aMaxTol)
            {
                aMaxTol = aTol;
            }
            ApproxArcsSegments2D anAlgo = new(aCurve, aTol, theAngleTol);
            ISchematiqList<Curve2D> aResultApprox = anAlgo.GetResult();

            // Form the array of approximated elementary curves
            if (anEdge.Orientation() == Reversed)
            {
                for (int iCrv = aResultApprox.Count; iCrv > 0; iCrv--)
                {
                    Curve2D? aCrv = aResultApprox[iCrv - 1];
                    if (aCrv is not null)
                    {
                        ref OrientedCurve anOCurve = ref vecCurve.Append(new OrientedCurve());
                        anOCurve.Curve = (TrimmedCurve2D)aCrv;
                        anOCurve.IsReverse = true;
                    }
                }
            }
            else
            {
                for (int iCrv = 1; iCrv <= aResultApprox.Count; iCrv++)
                {
                    Curve2D? aCrv = aResultApprox[iCrv - 1];
                    if (aCrv is not null)
                    {
                        ref OrientedCurve anOCurve = ref vecCurve.Append(new OrientedCurve());
                        anOCurve.Curve = (TrimmedCurve2D)aCrv;
                        anOCurve.IsReverse = false;
                    }
                }
            }
        }

        if (vecCurve.Length() > 0)
        {
            // Build the first vertex
            BrepBuilder aVBuilder = new();
            Point2D[] aPnt = [vecCurve.Value(0).Point(false), vecCurve.Value(vecCurve.Length() - 1).Point(true)];
            double aDist = aPnt[0].Distance(aPnt[1]);
            if (aDist > aMaxTol + global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                aDist = global::Kerf.Engine.Foundation.Precision.Confusion();
            }
            else
            {
                aDist = 0.5 * aDist + global::Kerf.Engine.Foundation.Precision.Confusion();
                aPnt[0] = new Point2D(0.5 * (aPnt[0].XY() + aPnt[1].XY()));
            }
            Point3D aPnt3d = aSurf.Value(aPnt[0].X(), aPnt[0].Y());
            Vertex aFirstVertex = new();
            aVBuilder.MakeVertex(ref aFirstVertex, aPnt3d, aDist);

            // Loop creating edges
            BRepBuilderAPI_MakeWire aMkWire = new();
            Edge anEdgeRes;
            Vertex aVertex = aFirstVertex;
            for (int iCrv = 0; iCrv < vecCurve.Length(); iCrv++)
            {
                OrientedCurve anOCurve = vecCurve.Value(iCrv);
                Vertex aNextVertex = new();
                aPnt[0] = anOCurve.Point(true);
                if (iCrv == vecCurve.Length() - 1)
                {
                    aPnt[1] = vecCurve.Value(0).Point(false);
                    aDist = aPnt[0].Distance(aPnt[1]);
                    if (aDist > aMaxTol + global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        aPnt3d = aSurf.Value(aPnt[0].X(), aPnt[0].Y());
                        aVBuilder.MakeVertex(ref aNextVertex, aPnt3d, global::Kerf.Engine.Foundation.Precision.Confusion());
                    }
                    else
                    {
                        aNextVertex = aFirstVertex;
                    }
                }
                else
                {
                    aPnt[1] = vecCurve.Value(iCrv + 1).Point(false);
                    aDist = 0.5 * (aPnt[0].Distance(aPnt[1])) + global::Kerf.Engine.Foundation.Precision.Confusion();
                    aPnt[0] = new Point2D(0.5 * (aPnt[0].XY() + aPnt[1].XY()));
                    aPnt3d = aSurf.Value(aPnt[0].X(), aPnt[0].Y());
                    aVBuilder.MakeVertex(ref aNextVertex, aPnt3d, aDist);
                }
                double[] aParam = [anOCurve.Curve.FirstParameter(), anOCurve.Curve.LastParameter()];
                if (anOCurve.IsReverse)
                {
                    BRepBuilderAPI_MakeEdge aMkEdge = new(anOCurve.Curve, aSurf, aNextVertex, aVertex, aParam[0], aParam[1]);
                    anEdgeRes = aMkEdge.InternalEdge();
                    anEdgeRes.Orientation(Reversed);
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
    public static Face ConvertFace(in Face theFace, double theAngleTolerance)
    {
        Face aResult = new();
        Surface aSurf = Tool.Surface(theFace)!;
        BRepBuilderAPI_MakeFace aMkFace = new(aSurf, global::Kerf.Engine.Foundation.Precision.Confusion());

        Explorer anExp = new(theFace, ShapeType.Wire);
        for (; anExp.More(); anExp.Next())
        {
            Wire aWire = ShapeCasts.Wire(anExp.Current());
            Wire aNewWire = InternalConvertWire(aWire, theAngleTolerance, theFace);
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
    public static Wire ConcatenateWire(in Wire W, Continuity Option, double TolAngular = 1.0e-4)
    {
        int nb_curve, // number of curves in the Wire
            index;
        WireExplorer WExp = new(W);
        Edge edge;
        double First = 0.0, Last = 0.0, // extremal values for the curve
            First0 = 0.0, toler = 0.0, tolleft, tolright; // Vertex tolerances
        Vertex Vfirst = new(), Vlast = new(); // Vertex of the Wire
        Point3D Pfirst, Plast; //, Pint;  corresponding points

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
            NCollection_Array1<BSplineCurve3D> tab = new(0, nb_curve - 1); // array of the wire's curve
            NCollection_Array1<double> tabtolvertex = new(0, nb_curve - 2); // array of the tolerance's vertex

            WExp.Init(W);

            for (index = 0; index < nb_curve; index++)
            { // main loop
                edge = WExp.Current();
                Curve3D aCurve = Tool.Curve(edge, out Placement L, out First, out Last)!;
                TrimmedCurve3D aTrCurve = new(aCurve, First, Last);
                tab[index] = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(aTrCurve); // storage in a array
                tab[index].Transform(L.Transformation());
                global::Kerf.Engine.Geometry.Conversion.GeomConvert.C0BSplineToC1BSplineCurve(ref tab[index], global::Kerf.Engine.Foundation.Precision.Confusion());

                if (index >= 1)
                { // continuity test loop
                    if (edge.Orientation() == Reversed)
                    {
                        tab[index].Reverse();
                    }
                    tolleft = Tool.Tolerance(Exploration.LastVertex(edge));
                    tolright = Tool.Tolerance(Exploration.FirstVertex(edge));
                    tabtolvertex[index - 1] = Max(tolleft, tolright);
                }

                if (index == 0)
                { // storage of the first edge features
                    First0 = First;
                    if (edge.Orientation() == Reversed)
                    { //(useful for the closed wire)
                        Vfirst = Exploration.LastVertex(edge);
                        tab[index].Reverse();
                    }
                    else
                    {
                        Vfirst = Exploration.FirstVertex(edge);
                    }
                }

                if (index == nb_curve - 1)
                { // storage of the last edge features
                    if (edge.Orientation() == Reversed)
                    {
                        Vlast = Exploration.FirstVertex(edge);
                    }
                    else
                    {
                        Vlast = Exploration.LastVertex(edge);
                    }
                }
                WExp.Next();
            }

            if (Tool.Tolerance(Vfirst) > Tool.Tolerance(Vlast))
            { // computation of the closing tolerance
                toler = Tool.Tolerance(Vfirst);
            }
            else
            {
                toler = Tool.Tolerance(Vlast);
            }

            Pfirst = Tool.Pnt(Vfirst);
            Plast = Tool.Pnt(Vlast);

            if ((Pfirst.Distance(Plast) <= toler) && // C0 continuity test at the closing point
                (global::Kerf.Engine.Geometry.Properties.GeomLProp.Continuity(tab[nb_curve - 1], tab[0], Last, First0, true, true, toler, TolAngular)
                 >= G1))
            {
                closed_tolerance = toler; // if ClosedG1!=0 it will be True and
                closed_flag = true;
            } // with the toler value
            NCollection_HArray1<BSplineCurve3D> concatcurve; // array of the concatenated curves
            if (Option == G1)
            {
                global::Kerf.Engine.Geometry.Conversion.GeomConvert.ConcatG1(tab,
                                     tabtolvertex,
                                     out concatcurve,
                                     ref closed_flag,
                                     closed_tolerance); // G1 concatenation
            }
            else
            {
                global::Kerf.Engine.Geometry.Conversion.GeomConvert.ConcatC1(tab,
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
            Curve3D aC = Tool.Curve(edge, out Placement L, out First, out Last)!;
            BSplineCurve3D aBS = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(new TrimmedCurve3D(aC, First, Last));
            aBS.Transform(L.Transformation());
            global::Kerf.Engine.Geometry.Conversion.GeomConvert.C0BSplineToC1BSplineCurve(ref aBS, global::Kerf.Engine.Foundation.Precision.Confusion());
            if (edge.Orientation() == Reversed)
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
    public static Edge ConcatenateWireC0(in Wire aWire)
    {
        double LinTol = global::Kerf.Engine.Foundation.Precision.Confusion();
        double AngTol = global::Kerf.Engine.Foundation.Precision.Angular();

        Edge ResEdge = new();

        // OCCT.NET: copy at entry (decision 793): OCCT computes the 3D curves on the caller's wire
        CopyAtEntry anEntry = new(aWire);
        Wire theWire = ShapeCasts.Wire(anEntry.Shape());
        BRepLib.BuildCurves3d(theWire);
        ShapeFix_Shape Fixer = new(theWire);
        Fixer.SetPrecision(LinTol);
        Fixer.SetMaxTolerance(LinTol);
        Fixer.Perform();
        theWire = ShapeCasts.Wire(Fixer.InternalShape());

        NCollection_Sequence<Curve3D> CurveSeq = new();
        NCollection_Sequence<double> FparSeq = new();
        NCollection_Sequence<double> LparSeq = new();
        NCollection_Sequence<double> TolSeq = new();
        NCollection_Sequence<bool> IsFwdSeq = new();
        CurveKind CurType = OtherCurve;
        Vertex FirstVertex = new(), LastVertex;

        WireExplorer wexp = new(theWire);

        for (; wexp.More(); wexp.Next())
        {
            Edge anEdge = wexp.Current();
            Curve3D? aCurve = Tool.Curve(anEdge, out double fpar, out double lpar);
            if (aCurve is null)
            {
                continue;
            }

            CurveAdaptor aGACurve = new(aCurve);
            CurveKind aType = aGACurve.GetType();
            Curve3D aBasisCurve = aGACurve.Curve()!;
            bool isFwd = wexp.Orientation() != Reversed;

            if (aBasisCurve.IsPeriodic)
            {
                global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(aBasisCurve.FirstParameter(),
                                      aBasisCurve.LastParameter(),
                                      global::Kerf.Engine.Foundation.Precision.PConfusion(),
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
                CurveAdaptor GAprevcurve = new(CurveSeq.Last());

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
                        case Line:
                        {
                            Line3D aLine = aGACurve.Line();
                            Line3D PrevLine = GAprevcurve.Line();

                            if (aLine.Contains(PrevLine.Location(), LinTol)
                                && aLine.Direction().IsParallel(PrevLine.Direction(), AngTol))
                            {
                                Point3D P1 = global::Kerf.Engine.Geometry.ElCLib.Value(fpar, aLine);
                                Point3D P2 = global::Kerf.Engine.Geometry.ElCLib.Value(lpar, aLine);

                                NewFpar = global::Kerf.Engine.Geometry.ElCLib.Parameter(PrevLine, P1);
                                NewLpar = global::Kerf.Engine.Geometry.ElCLib.Parameter(PrevLine, P2);
                                isSameCurve = true;
                            }
                            break;
                        }
                        case Circle:
                        {
                            Circle3D aCircle = aGACurve.Circle();
                            Circle3D PrevCircle = GAprevcurve.Circle();

                            if (aCircle.Location().Distance(PrevCircle.Location()) <= LinTol
                                && Math.Abs(aCircle.Radius() - PrevCircle.Radius()) <= LinTol
                                && aCircle.Axis().IsParallel(PrevCircle.Axis(), AngTol))
                            {
                                Point3D P1 = global::Kerf.Engine.Geometry.ElCLib.Value(fpar, aCircle);
                                Point3D P2 = global::Kerf.Engine.Geometry.ElCLib.Value(lpar, aCircle);

                                NewFpar = global::Kerf.Engine.Geometry.ElCLib.Parameter(PrevCircle, P1);
                                NewLpar = global::Kerf.Engine.Geometry.ElCLib.Parameter(PrevCircle, P2);
                                isSameCurve = true;
                            }
                            break;
                        }
                        case Ellipse:
                        {
                            Ellipse3D anEllipse = aGACurve.Ellipse();
                            Ellipse3D PrevEllipse = GAprevcurve.Ellipse();

                            if (anEllipse.Focus1().Distance(PrevEllipse.Focus1()) <= LinTol
                                && anEllipse.Focus2().Distance(PrevEllipse.Focus2()) <= LinTol
                                && Math.Abs(anEllipse.MajorRadius() - PrevEllipse.MajorRadius()) <= LinTol
                                && Math.Abs(anEllipse.MinorRadius() - PrevEllipse.MinorRadius()) <= LinTol
                                && anEllipse.Axis().IsParallel(PrevEllipse.Axis(), AngTol))
                            {
                                Point3D P1 = global::Kerf.Engine.Geometry.ElCLib.Value(fpar, anEllipse);
                                Point3D P2 = global::Kerf.Engine.Geometry.ElCLib.Value(lpar, anEllipse);

                                NewFpar = global::Kerf.Engine.Geometry.ElCLib.Parameter(PrevEllipse, P1);
                                NewLpar = global::Kerf.Engine.Geometry.ElCLib.Parameter(PrevEllipse, P2);
                                isSameCurve = true;
                            }
                            break;
                        }
                        case Hyperbola:
                        {
                            Hyperbola3D aHypr = aGACurve.Hyperbola();
                            Hyperbola3D PrevHypr = GAprevcurve.Hyperbola();

                            if (aHypr.Focus1().Distance(PrevHypr.Focus1()) <= LinTol
                                && aHypr.Focus2().Distance(PrevHypr.Focus2()) <= LinTol
                                && Math.Abs(aHypr.MajorRadius() - PrevHypr.MajorRadius()) <= LinTol
                                && Math.Abs(aHypr.MinorRadius() - PrevHypr.MinorRadius()) <= LinTol
                                && aHypr.Axis().IsParallel(PrevHypr.Axis(), AngTol))
                            {
                                Point3D P1 = global::Kerf.Engine.Geometry.ElCLib.Value(fpar, aHypr);
                                Point3D P2 = global::Kerf.Engine.Geometry.ElCLib.Value(lpar, aHypr);

                                NewFpar = global::Kerf.Engine.Geometry.ElCLib.Parameter(PrevHypr, P1);
                                NewLpar = global::Kerf.Engine.Geometry.ElCLib.Parameter(PrevHypr, P2);
                                isSameCurve = true;
                            }
                            break;
                        }
                        case Parabola:
                        {
                            Parabola3D aParab = aGACurve.Parabola();
                            Parabola3D PrevParab = GAprevcurve.Parabola();

                            if (aParab.Location().Distance(PrevParab.Location()) <= LinTol
                                && aParab.Focus().Distance(PrevParab.Focus()) <= LinTol
                                && Math.Abs(aParab.Focal() - PrevParab.Focal()) <= LinTol
                                && aParab.Axis().IsParallel(PrevParab.Axis(), AngTol))
                            {
                                Point3D P1 = global::Kerf.Engine.Geometry.ElCLib.Value(fpar, aParab);
                                Point3D P2 = global::Kerf.Engine.Geometry.ElCLib.Value(lpar, aParab);

                                NewFpar = global::Kerf.Engine.Geometry.ElCLib.Parameter(PrevParab, P1);
                                NewLpar = global::Kerf.Engine.Geometry.ElCLib.Parameter(PrevParab, P2);
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

                    if (aBasisCurve.IsPeriodic)
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
                            global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(LparSeq.Last(),
                                                  LparSeq.Last() + aPeriod,
                                                  global::Kerf.Engine.Foundation.Precision.PConfusion(),
                                                  ref NewFpar,
                                                  ref NewLpar);
                        }
                        else
                        {
                            // The current curve should be before the previous one.
                            global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(FparSeq.Last() - aPeriod,
                                                  FparSeq.Last(),
                                                  global::Kerf.Engine.Foundation.Precision.PConfusion(),
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
                    TolSeq.Append(Tool.Tolerance(wexp.CurrentVertex()));
                    CurType = aType;
                }
            }
        }

        LastVertex = wexp.CurrentVertex();
        TolSeq.Append(Tool.Tolerance(LastVertex));

        bool isReverse = false;
        if (!IsFwdSeq.IsEmpty())
        {
            isReverse = !IsFwdSeq.Value(1);
        }

        Vertex FirstVtx_final, LastVtx_final;
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
        FirstVtx_final.Orientation(Forward);
        LastVtx_final.Orientation(Reversed);

        if (CurveSeq.IsEmpty())
        {
            anEntry.Finish(ResEdge);
            return ResEdge;
        }

        int nb_curve = CurveSeq.Length(); // number of curves
        NCollection_Array1<BSplineCurve3D> tab = new(0, nb_curve - 1); // array of the curves
        NCollection_Array1<double> tabtolvertex = new(0, nb_curve - 1); //(0,nb_curve-2);  //array of the tolerances

        int i;

        if (nb_curve > 1)
        {
            for (i = 1; i <= nb_curve; i++)
            {
                if (CurveSeq.Value(i).GetType() == typeof(TrimmedCurve3D))
                {
                    CurveSeq[i] = ((TrimmedCurve3D)CurveSeq.Value(i)).BasisCurve();
                }

                TrimmedCurve3D aTrCurve = new(CurveSeq.Value(i), FparSeq.Value(i), LparSeq.Value(i));
                tab[i - 1] = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(aTrCurve);
                global::Kerf.Engine.Geometry.Conversion.GeomConvert.C0BSplineToC1BSplineCurve(ref tab[i - 1], global::Kerf.Engine.Foundation.Precision.Confusion());

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
                && global::Kerf.Engine.Geometry.Properties.GeomLProp.Continuity(tab[0],
                                        tab[nb_curve - 1],
                                        tab[0].FirstParameter(),
                                        tab[nb_curve - 1].LastParameter(),
                                        false,
                                        false,
                                        LinTol,
                                        AngTol)
                   >= G1)
            {
                closed_flag = true;
                closed_tolerance = Tool.Tolerance(FirstVertex);
            }

            global::Kerf.Engine.Geometry.Conversion.GeomConvert.ConcatC1(tab,
                                 tabtolvertex,
                                 out _,
                                 out NCollection_HArray1<BSplineCurve3D> concatcurve,
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

                CompCurveToBSplineCurve Concat = new(concatcurve.Value(concatcurve.Lower()));
                for (i = concatcurve.Lower() + 1; i <= concatcurve.Upper(); i++)
                {
                    Concat.Add(concatcurve.Value(i), MaxTolVer, true);
                }

                // the maker is discarded: its curve unfrozen, as OCCT's handle, which the reversal below writes into
                // (bugs/modalg_7/bug29573: BSplineCurve() froze it, and Reverse threw Standard_ImmutableObject)
                concatcurve.SetValue(concatcurve.Lower(), Concat.InternalBSplineCurve()!);
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
            if (CurveSeq.Value(1).GetType() == typeof(TrimmedCurve3D))
            {
                CurveSeq[1] = ((TrimmedCurve3D)CurveSeq.Value(1)).BasisCurve();
            }

            Curve3D aCopyCurve = (Curve3D)CurveSeq.Value(1).Copy();

            BRepLib_MakeEdge aMkEdge = new(aCopyCurve, FirstVtx_final, LastVtx_final, FparSeq.Value(1), LparSeq.Value(1));
            ResEdge = aMkEdge.InternalEdge();
        }

        if (isReverse)
        {
            ResEdge.Reverse();
        }

        ResEdge = ShapeCasts.Edge(anEntry.Finish(ResEdge));
        ResEdge.TShape()?.FreezeGraph();
        return ResEdge;
    }

    /// <summary>Whether the shape is valid (BRepCheck, geometry included).</summary>
    public static bool IsValid(in Shape S)
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
    public static bool IsValid(NCollection_List<Shape> theArgs,
                               in Shape theResult,
                               bool closedSolid = false,
                               bool GeomCtrl = true)
    {
        if (theResult.IsNull)
        {
            return true;
        }
        bool validate = false;

        NCollection_Map<Shape> allFaces = new(ShapeHasher.Instance);
        Explorer tEx = new();
        NCollection_List<Shape>.Iterator itLOS = new();
        for (itLOS.Initialize(theArgs); itLOS.More(); itLOS.Next())
        {
            if (itLOS.Value().IsSame(theResult))
            {
                validate = true;
                break;
            }
            for (tEx.Init(itLOS.Value(), ShapeType.Face); tEx.More(); tEx.Next())
            {
                allFaces.Add(tEx.Current());
            }
        }

        Compound toCheck = new();

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
            BrepBuilder bB = new();
            Face curf;
            for (tEx.Init(theResult, ShapeType.Face); tEx.More(); tEx.Next())
            {
                curf = ShapeCasts.Face(tEx.Current());
                if (!allFaces.Contains(curf))
                {
                    if (toCheck.IsNull)
                    {
                        bB.MakeCompound(ref toCheck);
                    }
                    BrepTools.Update(curf);
                    bB.Add(toCheck, curf);
                }
            }
            if (toCheck.IsNull)
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
                    for (tEx.Init(toCheck, ShapeType.Face); tEx.More(); tEx.Next())
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
                                    Explorer exp = new();
                                    for (exp.Init(tEx.Current(), ShapeType.Edge); exp.More(); exp.Next())
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
                                                        bB.SameRange(ShapeCasts.Edge(exp.Current()), false);
                                                        bB.SameParameter(ShapeCasts.Edge(exp.Current()), false);
                                                        BRepLib.SameParameter(ShapeCasts.Edge(exp.Current()),
                                                                              Tool.Tolerance(ShapeCasts.Edge(exp.Current())));
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
        for (tEx.Init(theResult, ShapeType.Shell); tEx.More(); tEx.Next())
        {
            if (HR is null)
            {
                HR = new BRepCheck_Shell(ShapeCasts.Shell(tEx.Current()));
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
    public static bool IsTopologicallyValid(in Shape S)
    {
        BRepCheck_Analyzer ana = new(S, false);
        return ana.IsValid();
    }
}
