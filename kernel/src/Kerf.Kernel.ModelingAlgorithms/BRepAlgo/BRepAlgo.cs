// Port of: src/ModelingAlgorithms/TKBool/BRepAlgo/BRepAlgo.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepAlgo/BRepAlgo.cxx
// Port of: src/ModelingAlgorithms/TKBool/BRepAlgo/BRepAlgo_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using Schematiq.Collections;
using System.Runtime.InteropServices;
using static Kerf.Engine.Healing.Checks.StatusCheck;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Booleans.Algo;

/// <summary>
/// Tools on wires and shapes: concatenation of a wire's edges into smoother ones, conversion of a wire into arcs and
/// segments, and validity checks.
/// </summary>
/// <remarks>
/// <para>
/// The results are frozen. <see cref="ConcatenateWireC0"/> computes missing 3D curves and fixes the wire before
/// concatenating, which OCCT does on the caller's wire: the port does it in a working overlay, so the result holds the
/// caller's vertices where they weren't changed (decision 793) and the wire is never written.
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
                return Curve.StartPoint;
            }
            return Curve.EndPoint;
        }
    }

    /// <summary>
    /// The wire <paramref name="wire"/> on <paramref name="face"/> converted into arcs of circles and segments,
    /// smooth where its tangents differ by less than <paramref name="angleTol"/>, within the largest tolerance of its
    /// edges (at least 1e-4); null if the wire has no edge.
    /// </summary>
    public static Wire ConvertWire(in Wire wire, double angleTol, in Face face)
    {
        Wire result = InternalConvertWire(wire, angleTol, face);
        result.TShape()?.FreezeGraph();
        return result;
    }

    /// <summary><see cref="ConvertWire"/> without freezing the result, for ported callers that build on it.</summary>
    internal static Wire InternalConvertWire(in Wire wire, double angleTol, in Face face)
    {
        Wire result = new();
        double maxTol = 0.0;
        Surface surf = Tool.Surface(face)!;
        List<OrientedCurve> vecCurve = new();

        WireExplorer expE = new(wire, face);
        // Explore the edges in the current wire, in their connection order
        for (; expE.More(); expE.Next())
        {
            Edge edge = expE.Current();
            EdgeCurve2DAdaptor curve = new(edge, face);
            double tol = Tool.Tolerance(edge);
            if (tol < MINIMAL_TOLERANCE)
            {
                tol = MINIMAL_TOLERANCE;
            }
            if (tol > maxTol)
            {
                maxTol = tol;
            }
            ApproxArcsSegments2D algo = new(curve, tol, angleTol);
            ISchematiqList<Curve2D> resultApprox = algo.GetResult();

            // Form the array of approximated elementary curves
            if (edge.Orientation == Reversed)
            {
                for (int iCrv = resultApprox.Count; iCrv > 0; iCrv--)
                {
                    Curve2D? crv = resultApprox[iCrv - 1];
                    if (crv is not null)
                    {
                        vecCurve.Add(new OrientedCurve());
                        ref OrientedCurve oCurve = ref CollectionsMarshal.AsSpan(vecCurve)[^1];
                        oCurve.Curve = (TrimmedCurve2D)crv;
                        oCurve.IsReverse = true;
                    }
                }
            }
            else
            {
                for (int iCrv = 1; iCrv <= resultApprox.Count; iCrv++)
                {
                    Curve2D? crv = resultApprox[iCrv - 1];
                    if (crv is not null)
                    {
                        vecCurve.Add(new OrientedCurve());
                        ref OrientedCurve oCurve = ref CollectionsMarshal.AsSpan(vecCurve)[^1];
                        oCurve.Curve = (TrimmedCurve2D)crv;
                        oCurve.IsReverse = false;
                    }
                }
            }
        }

        if (vecCurve.Count > 0)
        {
            // Build the first vertex
            BrepBuilder vBuilder = new();
            Point2D[] pnt = [vecCurve[0].Point(false), vecCurve[vecCurve.Count - 1].Point(true)];
            double dist = pnt[0].DistanceTo(pnt[1]);
            if (dist > maxTol + global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                dist = global::Kerf.Engine.Foundation.Precision.Confusion();
            }
            else
            {
                dist = 0.5 * dist + global::Kerf.Engine.Foundation.Precision.Confusion();
                pnt[0] = new Point2D(0.5 * (pnt[0].XY() + pnt[1].XY()));
            }
            Point3D pnt3d = surf.Value(pnt[0].X, pnt[0].Y);
            Vertex firstVertex = new();
            vBuilder.MakeVertex(ref firstVertex, pnt3d, dist);

            // Loop creating edges
            MakeWire mkWire = new();
            Edge edgeRes;
            Vertex vertex = firstVertex;
            for (int iCrv = 0; iCrv < vecCurve.Count; iCrv++)
            {
                OrientedCurve oCurve = vecCurve[iCrv];
                Vertex nextVertex = new();
                pnt[0] = oCurve.Point(true);
                if (iCrv == vecCurve.Count - 1)
                {
                    pnt[1] = vecCurve[0].Point(false);
                    dist = pnt[0].DistanceTo(pnt[1]);
                    if (dist > maxTol + global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        pnt3d = surf.Value(pnt[0].X, pnt[0].Y);
                        vBuilder.MakeVertex(ref nextVertex, pnt3d, global::Kerf.Engine.Foundation.Precision.Confusion());
                    }
                    else
                    {
                        nextVertex = firstVertex;
                    }
                }
                else
                {
                    pnt[1] = vecCurve[iCrv + 1].Point(false);
                    dist = 0.5 * (pnt[0].DistanceTo(pnt[1])) + global::Kerf.Engine.Foundation.Precision.Confusion();
                    pnt[0] = new Point2D(0.5 * (pnt[0].XY() + pnt[1].XY()));
                    pnt3d = surf.Value(pnt[0].X, pnt[0].Y);
                    vBuilder.MakeVertex(ref nextVertex, pnt3d, dist);
                }
                double[] param = [oCurve.Curve.FirstParameter, oCurve.Curve.LastParameter];
                if (oCurve.IsReverse)
                {
                    MakeEdge mkEdge = new(oCurve.Curve, surf, nextVertex, vertex, param[0], param[1]);
                    edgeRes = mkEdge.InternalEdge();
                    edgeRes.Orient(Reversed);
                }
                else
                {
                    MakeEdge mkEdge = new(oCurve.Curve, surf, vertex, nextVertex, param[0], param[1]);
                    edgeRes = mkEdge.InternalEdge();
                }
                vertex = nextVertex;
                mkWire.Add(edgeRes);
            }

            if (mkWire.IsDone)
            {
                result = mkWire.InternalWire();
            }
        }
        return result;
    }

    /// <summary>
    /// The face <paramref name="face"/> with every wire converted by <see cref="ConvertWire"/>; null if the face
    /// couldn't be made.
    /// </summary>
    public static Face ConvertFace(in Face face, double angleTolerance)
    {
        Face result = new();
        Surface surf = Tool.Surface(face)!;
        MakeFace mkFace = new(surf, global::Kerf.Engine.Foundation.Precision.Confusion());

        Explorer exp = new(face, ShapeType.Wire);
        for (; exp.More(); exp.Next())
        {
            Wire wire = ShapeCasts.Wire(exp.Current());
            Wire newWire = InternalConvertWire(wire, angleTolerance, face);
            mkFace.Add(newWire);
        }
        if (mkFace.IsDone)
        {
            result = mkFace.InternalFace();
        }
        result.TShape()?.FreezeGraph();
        return result;
    }

    /// <summary>
    /// A wire whose edges are C1 (or G1, with <paramref name="option"/>) made from <paramref name="w"/>, whose edges
    /// may meet with G1 continuity: the vertices between such edges are removed.
    /// </summary>
    public static Wire ConcatenateWire(in Wire w, Continuity option, double tolAngular = 1.0e-4)
    {
        int nb_curve, // number of curves in the Wire
            index;
        WireExplorer wExp = new(w);
        Edge edge;
        double first = 0.0, last = 0.0, // extremal values for the curve
            first0 = 0.0, toler = 0.0, tolleft, tolright; // Vertex tolerances
        Vertex vfirst = new(), vlast = new(); // Vertex of the Wire
        Point3D pfirst, plast; //, Pint;  corresponding points

        LibraryMakeWire makeResult = new();
        double closed_tolerance = 0.0;
        bool closed_flag = false;

        nb_curve = 0;

        while (wExp.More())
        { // computation of the curve number
            nb_curve++;
            wExp.Next();
        }

        if (nb_curve > 1)
        {
            NCollection_Array1<BSplineCurve3D> tab = new(0, nb_curve - 1); // array of the wire's curve
            NCollection_Array1<double> tabtolvertex = new(0, nb_curve - 2); // array of the tolerance's vertex

            wExp.Init(w);

            for (index = 0; index < nb_curve; index++)
            { // main loop
                edge = wExp.Current();
                Curve3D curve = Tool.Curve(edge, out Placement l, out first, out last)!;
                TrimmedCurve3D trCurve = new(curve, first, last);
                tab[index] = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(trCurve); // storage in a array
                tab[index].Transform(l.Transformation);
                global::Kerf.Engine.Geometry.Conversion.GeomConvert.C0BSplineToC1BSplineCurve(ref tab[index], global::Kerf.Engine.Foundation.Precision.Confusion());

                if (index >= 1)
                { // continuity test loop
                    if (edge.Orientation == Reversed)
                    {
                        tab[index].Reverse();
                    }
                    tolleft = Tool.Tolerance(Exploration.LastVertex(edge));
                    tolright = Tool.Tolerance(Exploration.FirstVertex(edge));
                    tabtolvertex[index - 1] = Max(tolleft, tolright);
                }

                if (index == 0)
                { // storage of the first edge features
                    first0 = first;
                    if (edge.Orientation == Reversed)
                    { //(useful for the closed wire)
                        vfirst = Exploration.LastVertex(edge);
                        tab[index].Reverse();
                    }
                    else
                    {
                        vfirst = Exploration.FirstVertex(edge);
                    }
                }

                if (index == nb_curve - 1)
                { // storage of the last edge features
                    if (edge.Orientation == Reversed)
                    {
                        vlast = Exploration.FirstVertex(edge);
                    }
                    else
                    {
                        vlast = Exploration.LastVertex(edge);
                    }
                }
                wExp.Next();
            }

            if (Tool.Tolerance(vfirst) > Tool.Tolerance(vlast))
            { // computation of the closing tolerance
                toler = Tool.Tolerance(vfirst);
            }
            else
            {
                toler = Tool.Tolerance(vlast);
            }

            pfirst = Tool.Pnt(vfirst);
            plast = Tool.Pnt(vlast);

            if ((pfirst.DistanceTo(plast) <= toler) && // C0 continuity test at the closing point
                (global::Kerf.Engine.Geometry.Properties.GeomLProp.Continuity(tab[nb_curve - 1], tab[0], last, first0, true, true, toler, tolAngular)
                 >= G1))
            {
                closed_tolerance = toler; // if ClosedG1!=0 it will be True and
                closed_flag = true;
            } // with the toler value
            NCollection_HArray1<BSplineCurve3D> concatcurve; // array of the concatenated curves
            if (option == G1)
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
                LibraryMakeEdge edgeBuilder = new(concatcurve.Value(index));
                edge = edgeBuilder.InternalEdge();
                makeResult.Add(edge);
            }
        }
        else
        {
            wExp.Init(w);

            edge = wExp.Current();
            Curve3D c = Tool.Curve(edge, out Placement l, out first, out last)!;
            BSplineCurve3D bs = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(new TrimmedCurve3D(c, first, last));
            bs.Transform(l.Transformation);
            global::Kerf.Engine.Geometry.Conversion.GeomConvert.C0BSplineToC1BSplineCurve(ref bs, global::Kerf.Engine.Foundation.Precision.Confusion());
            if (edge.Orientation == Reversed)
            {
                bs.Reverse();
            }

            LibraryMakeEdge edgeBuilder = new(bs);
            edge = edgeBuilder.InternalEdge();
            makeResult.Add(edge);
        }
        return makeResult.Wire();
    }

    /// <summary>
    /// An edge made from the wire <paramref name="aWire"/> (whose edges are first given 3D curves and fixed), possibly
    /// with sharp corners: its curve may be C0. Consecutive edges on the same line or conic are merged first.
    /// </summary>
    public static Edge ConcatenateWireC0(in Wire aWire)
    {
        double linTol = global::Kerf.Engine.Foundation.Precision.Confusion();
        double angTol = global::Kerf.Engine.Foundation.Precision.Angular();

        Edge resEdge = new();

        // Kerf: OCCT computes the 3D curves on the caller's wire: the overlay takes the writes (decision 793)
        using var overlay = WorkingOverlay.Begin(aWire);
        Wire theWire = aWire;
        BrepLibrary.BuildCurves3d(theWire);
        ShapeFixer fixer = new(theWire);
        fixer.SetPrecision(linTol);
        fixer.SetMaxTolerance(linTol);
        fixer.Perform();
        theWire = ShapeCasts.Wire(fixer.InternalShape());

        NCollection_Sequence<Curve3D> curveSeq = new();
        NCollection_Sequence<double> fparSeq = new();
        NCollection_Sequence<double> lparSeq = new();
        NCollection_Sequence<double> tolSeq = new();
        NCollection_Sequence<bool> isFwdSeq = new();
        CurveKind curType = OtherCurve;
        Vertex firstVertex = new(), lastVertex;

        WireExplorer wexp = new(theWire);

        for (; wexp.More(); wexp.Next())
        {
            Edge edge = wexp.Current();
            Curve3D? curve = Tool.Curve(edge, out double fpar, out double lpar);
            if (curve is null)
            {
                continue;
            }

            CurveAdaptor gaCurve = new(curve);
            CurveKind type = gaCurve.GetType();
            Curve3D basisCurve = gaCurve.Curve()!;
            bool isFwd = wexp.Orientation() != Reversed;

            if (basisCurve.IsPeriodic)
            {
                global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(basisCurve.FirstParameter,
                                      basisCurve.LastParameter,
                                      global::Kerf.Engine.Foundation.Precision.PConfusion(),
                                      ref fpar,
                                      ref lpar);
            }

            if (curveSeq.IsEmpty())
            {
                curveSeq.Append(curve);
                fparSeq.Append(fpar);
                lparSeq.Append(lpar);
                isFwdSeq.Append(isFwd);
                curType = type;
                firstVertex = wexp.CurrentVertex();
            }
            else
            {
                bool isSameCurve = false;
                double newFpar = RealFirst(), newLpar = RealLast();
                CurveAdaptor gAprevcurve = new(curveSeq.Last());

                if (ReferenceEquals(curve, curveSeq.Last()))
                {
                    newFpar = fpar;
                    newLpar = lpar;
                    isSameCurve = true;
                }
                else if (type == curType)
                {
                    switch (type)
                    {
                        case Line:
                        {
                            Line3D line = gaCurve.Line();
                            Line3D prevLine = gAprevcurve.Line();

                            if (line.Contains(prevLine.Location, linTol)
                                && line.Direction.IsParallel(prevLine.Direction, angTol))
                            {
                                Point3D p1 = global::Kerf.Engine.Geometry.ElCLib.Value(fpar, line);
                                Point3D p2 = global::Kerf.Engine.Geometry.ElCLib.Value(lpar, line);

                                newFpar = global::Kerf.Engine.Geometry.ElCLib.Parameter(prevLine, p1);
                                newLpar = global::Kerf.Engine.Geometry.ElCLib.Parameter(prevLine, p2);
                                isSameCurve = true;
                            }
                            break;
                        }
                        case Circle:
                        {
                            Circle3D circle = gaCurve.Circle();
                            Circle3D prevCircle = gAprevcurve.Circle();

                            if (circle.Location.DistanceTo(prevCircle.Location) <= linTol
                                && Math.Abs(circle.Radius - prevCircle.Radius) <= linTol
                                && circle.Axis.IsParallel(prevCircle.Axis, angTol))
                            {
                                Point3D p1 = global::Kerf.Engine.Geometry.ElCLib.Value(fpar, circle);
                                Point3D p2 = global::Kerf.Engine.Geometry.ElCLib.Value(lpar, circle);

                                newFpar = global::Kerf.Engine.Geometry.ElCLib.Parameter(prevCircle, p1);
                                newLpar = global::Kerf.Engine.Geometry.ElCLib.Parameter(prevCircle, p2);
                                isSameCurve = true;
                            }
                            break;
                        }
                        case Ellipse:
                        {
                            Ellipse3D ellipse = gaCurve.Ellipse();
                            Ellipse3D prevEllipse = gAprevcurve.Ellipse();

                            if (ellipse.Focus1.DistanceTo(prevEllipse.Focus1) <= linTol
                                && ellipse.Focus2.DistanceTo(prevEllipse.Focus2) <= linTol
                                && Math.Abs(ellipse.MajorRadius - prevEllipse.MajorRadius) <= linTol
                                && Math.Abs(ellipse.MinorRadius - prevEllipse.MinorRadius) <= linTol
                                && ellipse.Axis.IsParallel(prevEllipse.Axis, angTol))
                            {
                                Point3D p1 = global::Kerf.Engine.Geometry.ElCLib.Value(fpar, ellipse);
                                Point3D p2 = global::Kerf.Engine.Geometry.ElCLib.Value(lpar, ellipse);

                                newFpar = global::Kerf.Engine.Geometry.ElCLib.Parameter(prevEllipse, p1);
                                newLpar = global::Kerf.Engine.Geometry.ElCLib.Parameter(prevEllipse, p2);
                                isSameCurve = true;
                            }
                            break;
                        }
                        case Hyperbola:
                        {
                            Hyperbola3D hypr = gaCurve.Hyperbola();
                            Hyperbola3D prevHypr = gAprevcurve.Hyperbola();

                            if (hypr.Focus1.DistanceTo(prevHypr.Focus1) <= linTol
                                && hypr.Focus2.DistanceTo(prevHypr.Focus2) <= linTol
                                && Math.Abs(hypr.MajorRadius - prevHypr.MajorRadius) <= linTol
                                && Math.Abs(hypr.MinorRadius - prevHypr.MinorRadius) <= linTol
                                && hypr.Axis.IsParallel(prevHypr.Axis, angTol))
                            {
                                Point3D p1 = global::Kerf.Engine.Geometry.ElCLib.Value(fpar, hypr);
                                Point3D p2 = global::Kerf.Engine.Geometry.ElCLib.Value(lpar, hypr);

                                newFpar = global::Kerf.Engine.Geometry.ElCLib.Parameter(prevHypr, p1);
                                newLpar = global::Kerf.Engine.Geometry.ElCLib.Parameter(prevHypr, p2);
                                isSameCurve = true;
                            }
                            break;
                        }
                        case Parabola:
                        {
                            Parabola3D parab = gaCurve.Parabola();
                            Parabola3D prevParab = gAprevcurve.Parabola();

                            if (parab.Location.DistanceTo(prevParab.Location) <= linTol
                                && parab.Focus.DistanceTo(prevParab.Focus) <= linTol
                                && Math.Abs(parab.Focal - prevParab.Focal) <= linTol
                                && parab.Axis.IsParallel(prevParab.Axis, angTol))
                            {
                                Point3D p1 = global::Kerf.Engine.Geometry.ElCLib.Value(fpar, parab);
                                Point3D p2 = global::Kerf.Engine.Geometry.ElCLib.Value(lpar, parab);

                                newFpar = global::Kerf.Engine.Geometry.ElCLib.Parameter(prevParab, p1);
                                newLpar = global::Kerf.Engine.Geometry.ElCLib.Parameter(prevParab, p2);
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
                    bool isSameDir = isFwd == isFwdSeq.Last();

                    if (basisCurve.IsPeriodic)
                    {
                        // Treat periodic curves.
                        double period = basisCurve.Period;

                        if (isSameDir)
                        {
                            // Check if first parameter is greater then the last one.
                            while (newFpar > newLpar)
                            {
                                newFpar -= period;
                            }
                        }
                        else
                        { // !isSameDir
                            // Check if last parameter is greater then the first one.
                            while (newLpar > newFpar)
                            {
                                newLpar -= period;
                            }

                            // Change parameters
                            double tmpPar = newLpar;

                            newLpar = newFpar;
                            newFpar = tmpPar;
                        }

                        // Udjust parameters on periodic curves.
                        if (isFwdSeq.Last())
                        {
                            // The current curve should be after the previous one.
                            global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(lparSeq.Last(),
                                                  lparSeq.Last() + period,
                                                  global::Kerf.Engine.Foundation.Precision.PConfusion(),
                                                  ref newFpar,
                                                  ref newLpar);
                        }
                        else
                        {
                            // The current curve should be before the previous one.
                            global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(fparSeq.Last() - period,
                                                  fparSeq.Last(),
                                                  global::Kerf.Engine.Foundation.Precision.PConfusion(),
                                                  ref newFpar,
                                                  ref newLpar);
                        }
                    }
                    else if (!isSameDir)
                    {
                        // Not periodic curves. Opposite dirs.
                        double tmpPar = newLpar;

                        newLpar = newFpar;
                        newFpar = tmpPar;
                    }

                    if (isFwdSeq.Last())
                    {
                        // Update last parameter
                        lparSeq[lparSeq.Length()] = newLpar;
                    }
                    else
                    {
                        // Update first parameter
                        fparSeq[fparSeq.Length()] = newFpar;
                    }
                }
                else
                {
                    // Add new curve.
                    curveSeq.Append(curve);
                    fparSeq.Append(fpar);
                    lparSeq.Append(lpar);
                    isFwdSeq.Append(isFwd);
                    tolSeq.Append(Tool.Tolerance(wexp.CurrentVertex()));
                    curType = type;
                }
            }
        }

        lastVertex = wexp.CurrentVertex();
        tolSeq.Append(Tool.Tolerance(lastVertex));

        bool isReverse = false;
        if (!isFwdSeq.IsEmpty())
        {
            isReverse = !isFwdSeq.Value(1);
        }

        Vertex FirstVtx_final, LastVtx_final;
        if (isReverse)
        {
            FirstVtx_final = lastVertex;
            LastVtx_final = firstVertex;
        }
        else
        {
            FirstVtx_final = firstVertex;
            LastVtx_final = lastVertex;
        }
        FirstVtx_final.Orient(Forward);
        LastVtx_final.Orient(Reversed);

        if (curveSeq.IsEmpty())
        {
            overlay.Finish(resEdge);
            return resEdge;
        }

        int nb_curve = curveSeq.Length(); // number of curves
        NCollection_Array1<BSplineCurve3D> tab = new(0, nb_curve - 1); // array of the curves
        NCollection_Array1<double> tabtolvertex = new(0, nb_curve - 1); //(0,nb_curve-2);  //array of the tolerances

        int i;

        if (nb_curve > 1)
        {
            for (i = 1; i <= nb_curve; i++)
            {
                if (curveSeq.Value(i).GetType() == typeof(TrimmedCurve3D))
                {
                    curveSeq[i] = ((TrimmedCurve3D)curveSeq.Value(i)).BasisCurve;
                }

                TrimmedCurve3D trCurve = new(curveSeq.Value(i), fparSeq.Value(i), lparSeq.Value(i));
                tab[i - 1] = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(trCurve);
                global::Kerf.Engine.Geometry.Conversion.GeomConvert.C0BSplineToC1BSplineCurve(ref tab[i - 1], global::Kerf.Engine.Foundation.Precision.Confusion());

                if (!isFwdSeq.Value(i))
                {
                    tab[i - 1].Reverse();
                }

                if (i > 1)
                {
                    tabtolvertex[i - 2] = tolSeq.Value(i - 1) * 5.0;
                }
            }
            tabtolvertex[nb_curve - 1] = tolSeq.Value(tolSeq.Length()) * 5.0;

            bool closed_flag = false;
            double closed_tolerance = 0.0;
            if (firstVertex.IsSame(lastVertex)
                && global::Kerf.Engine.Geometry.Properties.GeomLProp.Continuity(tab[0],
                                        tab[nb_curve - 1],
                                        tab[0].FirstParameter,
                                        tab[nb_curve - 1].LastParameter,
                                        false,
                                        false,
                                        linTol,
                                        angTol)
                   >= G1)
            {
                closed_flag = true;
                closed_tolerance = Tool.Tolerance(firstVertex);
            }

            global::Kerf.Engine.Geometry.Conversion.GeomConvert.ConcatC1(tab,
                                 tabtolvertex,
                                 out _,
                                 out NCollection_HArray1<BSplineCurve3D> concatcurve,
                                 ref closed_flag,
                                 closed_tolerance); // C1 concatenation

            if (concatcurve.Length() > 1)
            {
                double maxTolVer = linTol;
                for (i = 1; i <= tolSeq.Length(); i++)
                {
                    if (tolSeq.Value(i) > maxTolVer)
                    {
                        maxTolVer = tolSeq.Value(i);
                    }
                }
                maxTolVer *= 5.0;

                CompCurveToBSplineCurve concat = new(concatcurve.Value(concatcurve.Lower()));
                for (i = concatcurve.Lower() + 1; i <= concatcurve.Upper(); i++)
                {
                    concat.Add(concatcurve.Value(i), maxTolVer, true);
                }

                // the maker is discarded: its curve unfrozen, as OCCT's handle, which the reversal below writes into
                // (bugs/modalg_7/bug29573: BSplineCurve() froze it, and Reverse threw Standard_ImmutableObject)
                concatcurve.SetValue(concatcurve.Lower(), concat.InternalBSplineCurve()!);
            }

            if (isReverse)
            {
                concatcurve.Value(concatcurve.Lower()).Reverse();
            }
            LibraryMakeEdge mkEdge = new(concatcurve.Value(concatcurve.Lower()),
                                           FirstVtx_final,
                                           LastVtx_final,
                                           concatcurve.Value(concatcurve.Lower()).FirstParameter,
                                           concatcurve.Value(concatcurve.Lower()).LastParameter);
            resEdge = mkEdge.InternalEdge();
        }
        else
        {
            if (curveSeq.Value(1).GetType() == typeof(TrimmedCurve3D))
            {
                curveSeq[1] = ((TrimmedCurve3D)curveSeq.Value(1)).BasisCurve;
            }

            Curve3D copyCurve = (Curve3D)curveSeq.Value(1).Copy();

            LibraryMakeEdge mkEdge = new(copyCurve, FirstVtx_final, LastVtx_final, fparSeq.Value(1), lparSeq.Value(1));
            resEdge = mkEdge.InternalEdge();
        }

        if (isReverse)
        {
            resEdge.Reverse();
        }

        resEdge = ShapeCasts.Edge(overlay.Finish(resEdge));
        resEdge.TShape()?.FreezeGraph();
        return resEdge;
    }

    /// <summary>Whether the shape is valid (BRepCheck, geometry included).</summary>
    public static bool IsValid(in Shape s)
    {
        Analyzer ana = new(s);
        return ana.IsValid();
    }

    /// <summary>
    /// Whether the faces of <paramref name="result"/> that are not faces of <paramref name="theArgs"/> are valid
    /// (all its faces when the arguments are empty), its shells too, and closed with <paramref name="closedSolid"/>;
    /// without <paramref name="geomCtrl"/>, new vertices' and edges' geometry and wire self-intersections are not
    /// checked.
    /// </summary>
    public static bool IsValid(NCollection_List<Shape> theArgs,
                               in Shape result,
                               bool closedSolid = false,
                               bool geomCtrl = true)
    {
        if (result.IsNull)
        {
            return true;
        }
        bool validate = false;

        HashSet<Shape> allFaces = new(ShapeHasher.Instance);
        Explorer tEx = new();
        NCollection_List<Shape>.Iterator itLOS = new();
        for (itLOS.Initialize(theArgs); itLOS.More(); itLOS.Next())
        {
            if (itLOS.Value().IsSame(result))
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

        if (allFaces.Count == 0)
        {
            if (validate)
            {
                return true;
            }
            Analyzer ana = new(result, geomCtrl);
            if (!ana.IsValid())
            {
                return false;
            }
        }
        else if (!validate)
        {
            BrepBuilder bB = new();
            Face curf;
            for (tEx.Init(result, ShapeType.Face); tEx.More(); tEx.Next())
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
                Analyzer ana = new(toCheck, true);
                if (!ana.IsValid())
                {
                    // Check if the problem is not just BRepCheck_InvalidSameParameterFlag
                    NCollection_List<StatusCheck>.Iterator itl = new();
                    StatusCheck sta;
                    for (tEx.Init(toCheck, ShapeType.Face); tEx.More(); tEx.Next())
                    {
                        if (ana.Result(tEx.Current()) is not null)
                        {
                            for (itl.Initialize(ana.Result(tEx.Current())!.Status()); itl.More(); itl.Next())
                            {
                                sta = itl.Value();
                                // If a face is incorrect
                                if (sta != StatusCheck.NoError)
                                {
                                    NCollection_List<StatusCheck>.Iterator ilt = new();
                                    Explorer exp = new();
                                    for (exp.Init(tEx.Current(), ShapeType.Edge); exp.More(); exp.Next())
                                    {
                                        ResultCheck res = ana.Result(exp.Current())!;
                                        for (res.InitContextIterator(); res.MoreShapeInContext(); res.NextShapeInContext())
                                        {
                                            if (res.ContextualShape().IsSame(tEx.Current()))
                                            {
                                                for (ilt.Initialize(res.StatusOnShape()); ilt.More(); ilt.Next())
                                                {
                                                    sta = ilt.Value();
                                                    // If an edge is BRepCheck_InvalidSameParameterFlag or
                                                    // BRepCheck_InvalidSameRangeFlag, it is forced
                                                    if (sta == StatusCheck.InvalidSameParameterFlag
                                                        || sta == StatusCheck.InvalidSameRangeFlag)
                                                    {
                                                        bB.SameRange(ShapeCasts.Edge(exp.Current()), false);
                                                        bB.SameParameter(ShapeCasts.Edge(exp.Current()), false);
                                                        BrepLibrary.SameParameter(ShapeCasts.Edge(exp.Current()),
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

        ShellCheck? hr = null;
        for (tEx.Init(result, ShapeType.Shell); tEx.More(); tEx.Next())
        {
            if (hr is null)
            {
                hr = new ShellCheck(ShapeCasts.Shell(tEx.Current()));
            }
            else
            {
                hr.Init(tEx.Current());
            }
            if (hr.Status().First() != StatusCheck.NoError)
            {
                return false;
            }
            if (hr.Orientation(false) != StatusCheck.NoError)
            {
                return false;
            }
            if (closedSolid)
            {
                if (hr.Closed() != StatusCheck.NoError)
                {
                    return false;
                }
            }
        }

        return true;
    }

    /// <summary>Whether the shape is valid topologically (BRepCheck without the geometric controls).</summary>
    public static bool IsTopologicallyValid(in Shape s)
    {
        Analyzer ana = new(s, false);
        return ana.IsValid();
    }
}
