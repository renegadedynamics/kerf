// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ShapeConvertToBezier.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ShapeConvertToBezier.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ShapeConvertToBezier.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// Converts the surfaces, 3D curves and pcurves of a shape to Bezier-based geometry (each kind with its mode), then
/// makes every edge on Bezier curves have the range [0, 1] and joins the ends of consecutive Bezier curves.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21). Unlike the other dividers, its last step writes into the result in place (edge
/// ranges and curves, Bezier poles, the faces' Update), and the result shares the shape's unchanged sub-shapes; so a
/// shape holding frozen shapes is copied at the entry (<see cref="CopyAtEntry"/>, decision 583) and the
/// caller's own sub-shapes are given back where nothing changed, the context re-keyed to them (decision 999).
/// </para>
/// <para>
/// OCCT changes the Bezier curves of the edges in place (Segment, SetPole). The port's curves attached to an edge are
/// frozen, so each change is made on a copy that replaces the curve in the edge's representations, in place, as
/// OCCT's change would show there (decision 998).
/// </para>
/// </remarks>
public class ShapeUpgrade_ShapeConvertToBezier : ShapeUpgrade_ShapeDivide
{
    private bool my2dMode;
    private bool my3dMode;
    private bool mySurfaceMode;
    private bool my3dLineMode;
    private bool my3dCircleMode;
    private bool my3dConicMode;
    private bool myPlaneMode;
    private bool myRevolutionMode;
    private bool myExtrusionMode;
    private bool myBSplineMode;
    private int myLevel;

    /// <summary>Empty constructor (no conversion; the special modes on).</summary>
    public ShapeUpgrade_ShapeConvertToBezier()
    {
        myLevel = 0;
        my2dMode = false;
        my3dMode = false;
        mySurfaceMode = false;
        // set spesial flags to true
        my3dLineMode = true;
        my3dCircleMode = true;
        my3dConicMode = true;
        myPlaneMode = true;
        myRevolutionMode = true;
        myExtrusionMode = true;
        myBSplineMode = true;
    }

    /// <summary>Initializes by a shape.</summary>
    public ShapeUpgrade_ShapeConvertToBezier(in Shape S) : base(S)
    {
        myLevel = 0;
        my2dMode = false;
        my3dMode = false;
        mySurfaceMode = false;
        // set spesial flags to true
        my3dLineMode = true;
        my3dCircleMode = true;
        my3dConicMode = true;
        myPlaneMode = true;
        myRevolutionMode = true;
        myExtrusionMode = true;
        myBSplineMode = true;
    }

    /// <summary>Sets the mode for converting pcurves to Beziers.</summary>
    public void Set2dConversion(bool mode)
    {
        my2dMode = mode;
    }

    /// <summary>Returns the 2D conversion mode.</summary>
    public bool Get2dConversion() => my2dMode;

    /// <summary>Sets the mode for converting 3D curves to Beziers.</summary>
    public void Set3dConversion(bool mode)
    {
        my3dMode = mode;
    }

    /// <summary>Returns the 3D conversion mode.</summary>
    public bool Get3dConversion() => my3dMode;

    /// <summary>Sets the mode for converting surfaces to a Bezier basis.</summary>
    public void SetSurfaceConversion(bool mode)
    {
        mySurfaceMode = mode;
    }

    /// <summary>Returns the surface conversion mode.</summary>
    public bool GetSurfaceConversion() => mySurfaceMode;

    /// <summary>Sets the mode for converting Geom_Line to a Bezier.</summary>
    public void Set3dLineConversion(bool mode)
    {
        my3dLineMode = mode;
    }

    /// <summary>Returns the Geom_Line conversion mode.</summary>
    public bool Get3dLineConversion() => my3dLineMode;

    /// <summary>Sets the mode for converting Geom_Circle to Beziers.</summary>
    public void Set3dCircleConversion(bool mode)
    {
        my3dCircleMode = mode;
    }

    /// <summary>Returns the Geom_Circle conversion mode.</summary>
    public bool Get3dCircleConversion() => my3dCircleMode;

    /// <summary>Sets the mode for converting Geom_Conic to Beziers.</summary>
    public void Set3dConicConversion(bool mode)
    {
        my3dConicMode = mode;
    }

    /// <summary>Returns the Geom_Conic conversion mode.</summary>
    public bool Get3dConicConversion() => my3dConicMode;

    /// <summary>Sets the mode for converting Geom_Plane to a Bezier.</summary>
    public void SetPlaneMode(bool mode)
    {
        myPlaneMode = mode;
    }

    /// <summary>Returns the Geom_Plane conversion mode.</summary>
    public bool GetPlaneMode() => myPlaneMode;

    /// <summary>Sets the mode for converting Geom_SurfaceOfRevolution to Beziers.</summary>
    public void SetRevolutionMode(bool mode)
    {
        myRevolutionMode = mode;
    }

    /// <summary>Returns the Geom_SurfaceOfRevolution conversion mode.</summary>
    public bool GetRevolutionMode() => myRevolutionMode;

    /// <summary>Sets the mode for converting Geom_SurfaceOfLinearExtrusion to Beziers.</summary>
    public void SetExtrusionMode(bool mode)
    {
        myExtrusionMode = mode;
    }

    /// <summary>Returns the Geom_SurfaceOfLinearExtrusion conversion mode.</summary>
    public bool GetExtrusionMode() => myExtrusionMode;

    /// <summary>Sets the mode for converting Geom_BSplineSurface to Beziers.</summary>
    public void SetBSplineMode(bool mode)
    {
        myBSplineMode = mode;
    }

    /// <summary>Returns the Geom_BSplineSurface conversion mode.</summary>
    public bool GetBSplineMode() => myBSplineMode;

    /// <summary>Performs the conversion and computes the resulting shape.</summary>
    public override bool Perform(bool newContext = true) => PerformAtEntry(newContext, performConversion);

    private bool performConversion(bool newContext)
    {
        myLevel++;
        bool res = false;

        if (myLevel == 1)
        {
            bool isNewContext = newContext;
            if (mySurfaceMode)
            {
                bool mode2d = my2dMode, mode3d = my3dMode;
                my2dMode = my3dMode = false;
                res = base.Perform(isNewContext);
                isNewContext = false;
                myShape = myResult;
                my2dMode = mode2d;
                my3dMode = mode3d;
            }
            if (my2dMode || my3dMode)
            {
                bool modeS = mySurfaceMode;
                mySurfaceMode = false;
                res = base.Perform(isNewContext);
                mySurfaceMode = modeS;
            }
        }
        else
        {
            res = base.Perform(newContext);
        }

        // pdn Hereafter the fix on GeomLib:SameParameter.
        // In order to fix this bug all edges that are based on
        // bezier curves (2d or 3d) and have range not equal to [0,1]
        // are performed the following sequence:
        //  1. Segment on bezier curve
        //  2. Changing rande of edge to [0,1]

        if (myLevel == 1)
        {
            var B = new BrepBuilder();
            var sae = new ShapeAnalysis_Edge();
            var sbe = new ShapeBuild_Edge();
            double preci = Kerf.Engine.Foundation.Precision.PConfusion();
            for (var exp = new Explorer(myResult, ShapeType.Face); exp.More(); exp.Next())
            {
                Face face = ShapeCasts.Face(exp.Current());
                face.Orientation(Forward);
                for (var exp1 = new Explorer(face, ShapeType.Wire); exp1.More(); exp1.Next())
                {
                    Wire wire = ShapeCasts.Wire(exp1.Current());
                    var sfw = new ShapeFix_Wire(wire, face, myPrecision);
                    sfw.FixReorder();
                    sfw.FixShifted(); // for cylinders.brep
                    ShapeExtend_WireData sewd = sfw.WireData()!;
                    for (int i = 1; i <= sewd.NbEdges(); i++)
                    {
                        Edge edge = sewd.Edge(i);
                        double first, last;
                        Exploration.Vertices(edge, out Vertex V1, out Vertex V2);
                        if (sae.Curve3d(edge, out Curve3D? c3d, out first, out last, false))
                        {
                            if (c3d is BezierCurve3D bezier3d)
                            {
                                // B.SameRange(edge, false);
                                if (first != 0 || last != 1)
                                {
                                    if (first > preci || last < 1 - preci)
                                    {
                                        bezier3d = mutable3d(bezier3d);
                                        bezier3d.Segment(first, last);
                                        B.UpdateEdge(edge, bezier3d, 0.0);
                                    }
                                    sbe.SetRange3d(edge, 0, 1);
                                }
                                { // gka fix against small edges ; merging ends of 3d curves
                                    Point3D p1 = bezier3d.Value(first);
                                    Point3D p2 = bezier3d.Value(last);
                                    Point3D p1v = Tool.Pnt(V1);
                                    Point3D p2v = Tool.Pnt(V2);
                                    if (p1.Distance(p1v) > Kerf.Engine.Foundation.Precision.Confusion())
                                    {
                                        bezier3d = mutable3d(bezier3d);
                                        bezier3d.SetPole(1, p1v);
                                    }
                                    if (p2.Distance(p2v) > Kerf.Engine.Foundation.Precision.Confusion())
                                    {
                                        bezier3d = mutable3d(bezier3d);
                                        bezier3d.SetPole(bezier3d.PoleCount, p2v);
                                    }
                                    B.UpdateEdge(edge, bezier3d, 0.0);
                                }
                            }
                            else if (my2dMode)
                            {
                                B.SameRange(edge, true);
                            }
                        }

                        Curve2D? c2drev = null;
                        BezierCurve2D? bezier = null, bezierR = null;
                        BezierCurve2D? beziernext = null, bezierRnext = null;
                        Edge edgenext = sewd.Edge((i == sewd.NbEdges() ? 1 : i + 1));
                        Curve2D? newRevCurve = null, newCurve = null, newnextCurve = null;

                        // OCCT.NET: OCCT changes the pcurves in place, so a curve held by several of these variables (a seam
                        // whose two pcurves are one curve, a wire of one edge) takes every change; the port changes copies
                        // (decision 998), carried over to each variable that held the curve changed
                        BezierCurve2D change(in Edge theEdge, BezierCurve2D theCurve,
                                                  Action<BezierCurve2D> theChange)
                        {
                            BezierCurve2D aNew = changed2d(theEdge, theEdge.IsSame(edge) ? edgenext : edge, theCurve, theChange);
                            if (!ReferenceEquals(aNew, theCurve))
                            {
                                if (ReferenceEquals(bezier, theCurve)) bezier = aNew;
                                if (ReferenceEquals(bezierR, theCurve)) bezierR = aNew;
                                if (ReferenceEquals(beziernext, theCurve)) beziernext = aNew;
                                if (ReferenceEquals(bezierRnext, theCurve)) bezierRnext = aNew;
                                if (ReferenceEquals(newCurve, theCurve)) newCurve = aNew;
                                if (ReferenceEquals(newRevCurve, theCurve)) newRevCurve = aNew;
                                if (ReferenceEquals(newnextCurve, theCurve)) newnextCurve = aNew;
                            }
                            return aNew;
                        }

                        bool isSeam = Tool.IsClosed(edge, face);
                        if (!sae.PCurve(edge, face, out Curve2D? c2d, out first, out last, false))
                        {
                            continue;
                        }

                        if (c2d is not BezierCurve2D)
                        {
                            continue;
                        }
                        bezier = (BezierCurve2D)c2d;
                        if (isSeam)
                        {
                            Shape aLocalShape = edge.Reversed();
                            Edge tmpedge = ShapeCasts.Edge(aLocalShape);
                            if (sae.PCurve(tmpedge, face, out c2drev, out first, out last, false))
                            {
                                if (c2drev is BezierCurve2D aRev)
                                {
                                    bezierR = aRev;
                                }
                            }
                        }
                        if (first != 0 || last != 1)
                        {
                            if (first > preci || last < 1 - preci)
                            {
                                if (bezier is not null)
                                {
                                    double f0 = first, l0 = last;
                                    bezier = change(edge, bezier, b => b.Segment(f0, l0));
                                    newCurve = bezier;
                                }
                                else
                                {
                                    global::Kerf.Engine.Geometry.Library.GeomLib.SameRange(preci, c2d, first, last, 0, 1, ref newCurve);
                                }
                                if (isSeam)
                                {
                                    if (bezierR is not null)
                                    {
                                        double f0 = first, l0 = last;
                                        bezierR = change(edge, bezierR, b => b.Segment(f0, l0));
                                        newRevCurve = bezierR;
                                    }
                                    else if (c2drev is not null)
                                    {
                                        global::Kerf.Engine.Geometry.Library.GeomLib.SameRange(preci, c2drev, first, last, 0, 1, ref newRevCurve);
                                    }
                                    if (edge.Orientation() == Forward)
                                    {
                                        B.UpdateEdge(edge, newCurve, newRevCurve, face, 0.0);
                                    }
                                    else
                                    {
                                        B.UpdateEdge(edge, newRevCurve, newCurve, face, 0.0);
                                    }
                                }
                            }
                            B.Range(edge, face, 0, 1);
                        }

                        Curve2D? c2drevnext = null;
                        if (!sae.PCurve(edgenext, face, out Curve2D? c2dnext, out double first2, out double last2, false))
                        {
                            continue;
                        }
                        if (c2dnext is not BezierCurve2D)
                        {
                            continue;
                        }
                        beziernext = (BezierCurve2D)c2dnext;

                        // occ::handle<Geom2d_Curve> newRevCurve;
                        if (isSeam)
                        {
                            Shape aLocalShape = edgenext.Reversed();
                            Edge tmpedge = ShapeCasts.Edge(aLocalShape);
                            if (sae.PCurve(tmpedge, face, out c2drevnext, out first, out last, false))
                            {
                                if (c2drevnext is BezierCurve2D aRevNext)
                                {
                                    bezierRnext = aRevNext;
                                }
                            }
                        }
                        if (first2 != 0 || last2 != 1)
                        {
                            if (first2 > preci || last2 < 1 - preci)
                            {
                                if (beziernext is not null)
                                {
                                    double f0 = first2, l0 = last2;
                                    beziernext = change(edgenext, beziernext, b => b.Segment(f0, l0));
                                    newnextCurve = beziernext;
                                }
                                else
                                {
                                    global::Kerf.Engine.Geometry.Library.GeomLib.SameRange(preci, c2dnext, first2, last2, 0, 1, ref newnextCurve);
                                }
                                if (isSeam)
                                {
                                    if (bezierRnext is not null)
                                    {
                                        double f0 = first2, l0 = last2;
                                        bezierRnext = change(edgenext, bezierRnext, b => b.Segment(f0, l0));
                                        newRevCurve = bezierRnext;
                                    }
                                    else if (c2drevnext is not null)
                                    {
                                        global::Kerf.Engine.Geometry.Library.GeomLib.SameRange(preci, c2drevnext, first2, last2, 0, 1, ref newRevCurve);
                                    }
                                    if (edge.Orientation() == Forward)
                                    {
                                        B.UpdateEdge(edgenext, newnextCurve, newRevCurve, face, 0.0);
                                    }
                                    else
                                    {
                                        B.UpdateEdge(edgenext, newRevCurve, newnextCurve, face, 0.0);
                                    }
                                }
                            }
                            B.Range(edgenext, face, 0, 1);
                        }

                        if (bezier is null || beziernext is null)
                        {
                            continue; // gka fix against small edges ; merging ends of pcurves
                        }

                        double f1, l1, f2, l2;
                        f1 = bezier.FirstParameter();
                        l1 = bezier.LastParameter();
                        f2 = beziernext.FirstParameter();
                        l2 = beziernext.LastParameter();
                        Point2D p2d1 = bezier.Value(f1);
                        Point2D p2d2 = bezier.Value(l1);
                        Point2D p2d1next = beziernext.Value(f2);
                        Point2D p2d2next = beziernext.Value(l2);

                        void setPoles(int theIndex, int theIndexNext, in Point2D thePnt)
                        {
                            Point2D aPnt = thePnt;
                            change(edge, bezier!, b => b.SetPole(theIndex, aPnt));
                            change(edgenext, beziernext!, b => b.SetPole(theIndexNext, aPnt));
                        }

                        if (edge.Orientation() == Forward && edgenext.Orientation() == Forward)
                        {
                            if (p2d2.Distance(p2d1next) > Kerf.Engine.Foundation.Precision.PConfusion())
                            {
                                var pmid = new Point2D(0.5 * (p2d2.XY() + p2d1next.XY()));
                                setPoles(bezier.PoleCount, 1, pmid);
                            }
                        }

                        if (edge.Orientation() == Forward && edgenext.Orientation() == Reversed)
                        {
                            if (p2d2.Distance(p2d2next) > Kerf.Engine.Foundation.Precision.PConfusion())
                            {
                                var pmid = new Point2D(0.5 * (p2d2.XY() + p2d2next.XY()));
                                setPoles(bezier.PoleCount, beziernext.PoleCount, pmid);
                            }
                        }

                        if (edge.Orientation() == Reversed && edgenext.Orientation() == Forward)
                        {
                            if (p2d1.Distance(p2d1next) > Kerf.Engine.Foundation.Precision.PConfusion())
                            {
                                var pmid = new Point2D(0.5 * (p2d1.XY() + p2d1next.XY()));
                                setPoles(1, 1, pmid);
                            }
                        }

                        if (edge.Orientation() == Reversed && edgenext.Orientation() == Reversed)
                        {
                            if (p2d1.Distance(p2d2next) > Kerf.Engine.Foundation.Precision.PConfusion())
                            {
                                var pmid = new Point2D(0.5 * (p2d1.XY() + p2d2next.XY()));
                                setPoles(1, beziernext.PoleCount, pmid);
                            }
                        }
                    }
                    BrepTools.Update(face);
                }
            }
        }

        myLevel--;
        return res;
    }

    // OCCT.NET: a 3D Bezier curve OCCT changes in place: the curve itself if ported code owns it, else a copy (the
    // edge's curve is frozen once attached); OCCT's following UpdateEdge attaches it.
    private static BezierCurve3D mutable3d(BezierCurve3D theCurve) =>
        theCurve.IsFrozen ? (BezierCurve3D)theCurve.Copy() : theCurve;

    // OCCT.NET: changes a pcurve as OCCT does in place. A frozen curve (attached to the edge) is changed on a copy,
    // which then replaces the curve in the edge's representations, where OCCT's change would show; a curve that isn't
    // the edge's (a planar face's computed pcurve) is changed alone, as in OCCT.
    private static BezierCurve2D changed2d(in Edge theEdge, in Edge theOther, BezierCurve2D theCurve,
                                                Action<BezierCurve2D> theChange)
    {
        if (!theCurve.IsFrozen)
        {
            theChange(theCurve);
            return theCurve;
        }
        var aCopy = (BezierCurve2D)theCurve.Copy();
        theChange(aCopy);
        replaced(theEdge, theCurve, aCopy);
        if (!theOther.IsSame(theEdge))
        {
            replaced(theOther, theCurve, aCopy);
        }
        return aCopy;
    }

    // The curve replaced in the edge's representations.
    private static void replaced(in Edge theEdge, BezierCurve2D theCurve, BezierCurve2D aCopy)
    {
        var aTEdge = (BrepTEdge)theEdge.TShape()!;
        foreach (CurveRepresentation aRep in aTEdge.ChangeCurves())
        {
            if (aRep.IsCurveOnSurface() && ReferenceEquals(aRep.PCurve(), theCurve))
            {
                aRep.PCurve(aCopy);
            }
            if (aRep.IsCurveOnClosedSurface && ReferenceEquals(aRep.PCurve2(), theCurve))
            {
                aRep.PCurve2(aCopy);
            }
        }
    }

    /// <summary>Returns a new tool for dividing faces with the conversion modes.</summary>
    protected override ShapeUpgrade_FaceDivide? GetSplitFaceTool()
    {
        var tool = new ShapeUpgrade_FaceDivide();
        var wtool = new ShapeUpgrade_WireDivide();
        if (my3dMode)
        {
            var curve3dConverter = new ShapeUpgrade_ConvertCurve3dToBezier();
            curve3dConverter.SetLineMode(my3dLineMode);
            curve3dConverter.SetCircleMode(my3dCircleMode);
            curve3dConverter.SetConicMode(my3dConicMode);
            wtool.SetSplitCurve3dTool(curve3dConverter);
        }
        if (my2dMode)
        {
            wtool.SetSplitCurve2dTool(new ShapeUpgrade_ConvertCurve2dToBezier());
        }
        wtool.SetEdgeMode(myEdgeMode);
        var FixSmallBezierCurvesTool = new ShapeUpgrade_FixSmallBezierCurves();
        wtool.SetFixSmallCurveTool(FixSmallBezierCurvesTool);
        tool.SetWireDivideTool(wtool);
        if (mySurfaceMode)
        {
            var stool = new ShapeUpgrade_ConvertSurfaceToBezierBasis();
            stool.SetPlaneMode(myPlaneMode);
            stool.SetRevolutionMode(myRevolutionMode);
            stool.SetExtrusionMode(myExtrusionMode);
            stool.SetBSplineMode(myBSplineMode);
            tool.SetSplitSurfaceTool(stool);
        }
        return tool;
    }

    /// <summary>The message sent for a converted face.</summary>
    protected override Msg GetFaceMsg() => new("ShapeDivide.FaceConvertToBezier.MSG0");

    /// <summary>The message sent for a converted free wire.</summary>
    protected override Msg GetWireMsg() => new("ShapeDivide.WireConvertToBezier.MSG0");

    /// <summary>The message sent for a converted free edge.</summary>
    protected override Msg GetEdgeMsg() => new("ShapeDivide.EdgeConvertToBezier.MSG0");
}
