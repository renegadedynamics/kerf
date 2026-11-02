// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslateEdge.hxx
// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslateEdge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using static Occt.BRepLib_EdgeError;
using static Occt.StepToTopoDS_TranslateEdgeError;
using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>Translates a STEP edge (an edge_curve, or an oriented_edge of one) into a TopoDS_Edge.</summary>
/// <remarks>
/// The edge is always built FORWARD, along its curve. <see cref="Value"/> freezes the result; the translators use
/// <c>InternalValue</c> (decision 1259). The 3D curves are shared through the transient process
/// (<c>BindTransient</c>): where OCCT reverses or adjusts such a curve in place after an earlier edge took it, the port
/// works on a copy (decision 1260).
/// </remarks>
public sealed class StepToTopoDS_TranslateEdge : StepToTopoDS_Root
{
    private StepToTopoDS_TranslateEdgeError myError;
    private TopoDS_Shape myResult = new TopoDS_Shape();

    //: d8
    // Used in I-DEAS-like STP processing (ssv; 15.11.2010)

    private static void DecodeMakeEdgeError(BRepLib_MakeEdge ME, Standard_Transient orig, ref Geom_Curve myCurve, in TopoDS_Vertex V1,
                                            in TopoDS_Vertex V2, double U1, double U2, StepToTopoDS_Tool aTool,
                                            StepShape_TopologicalRepresentationItem tobind)
    {
        _ = U1;
        _ = U2; // avoid compiler warning
        _ = tobind;
        Transfer_TransientProcess TP = aTool.TransientProcess();

        // OCCT_DEBUG output left out
        switch (ME.Error())
        {
            case BRepLib_EdgeDone:
                return;
            case BRepLib_PointProjectionFailed:
                TP.AddFail(orig, " Point Projection failed");
                break;
            case BRepLib_ParameterOutOfRange:
                TP.AddFail(orig, " Parameter Out Of Range");
                break;
            case BRepLib_DifferentPointsOnClosedCurve:
                TP.AddFail(orig, " Different Points on Closed Curve");
                break;
            case BRepLib_PointWithInfiniteParameter:
                TP.AddFail(orig, " Point with infinite Parameter");
                break;
            case BRepLib_DifferentsPointAndParameter:
                // OCCT.NET: the curve an earlier edge took is frozen; adjusted in a copy (decision 1260)
                if (myCurve.IsFrozen())
                {
                    myCurve = (Geom_Curve)myCurve.Copy();
                }

                if (!new ShapeConstruct_Curve().AdjustCurve(myCurve, BRep_Tool.Pnt(V1), BRep_Tool.Pnt(V2), true, true))
                {
                    TP.AddFail(orig, " Different Points and Parameters");
                }
                else
                {
                    TP.AddWarning(orig, "Different Points and Parameters, adjusted");
                }

                break;
            case BRepLib_LineThroughIdenticPoints:
                TP.AddFail(orig, " Line through identic Points");
                break;
        }
    }

    private static Geom_Curve MakeCurve(StepGeom_Curve C1, Transfer_TransientProcess TP, in StepData_Factors theLocalFactors)
    {
        Geom_Curve C2 = TP.FindTransient(C1) as Geom_Curve;
        if (C2 is not null)
        {
            return C2;
        }

        C2 = StepToGeom.MakeCurve(C1, theLocalFactors);
        if (C2 is not null)
        {
            TP.BindTransient(C1, C2);
        }

        return C2;
    }

    private static TopoDS_Edge MakeEdge(Geom_Curve C3D, in TopoDS_Vertex V1, in TopoDS_Vertex V2, double U1, double U2)
    {
        var B = new BRep_Builder();
        TopoDS_Edge E = new TopoDS_Edge();
        B.MakeEdge(ref E, C3D, Occt.Precision.Confusion());
        B.Add(E, V1);
        B.Add(E, V2);
        B.UpdateVertex(V1, U1, E, 0.0);
        B.UpdateVertex(V2, U2, E, 0.0);
        return E;
    }

    public StepToTopoDS_TranslateEdge()
    {
        myError = StepToTopoDS_TranslateEdgeOther;
        done = false;
    }

    public StepToTopoDS_TranslateEdge(StepShape_Edge E, StepToTopoDS_Tool T, StepToTopoDS_NMTool NMTool, in StepData_Factors theLocalFactors)
    {
        Init(E, T, NMTool, theLocalFactors);
    }

    public StepToTopoDS_TranslateEdge(StepShape_Edge E, StepToTopoDS_Tool T, StepToTopoDS_NMTool NMTool)
        : this(E, T, NMTool, new StepData_Factors())
    {
    }

    /// <summary>
    /// Translates the edge: an edge with two vertices and two parameters, always built FORWARD (BRepLib_MakeEdge).
    /// </summary>
    public void Init(StepShape_Edge aEdge, StepToTopoDS_Tool aTool, StepToTopoDS_NMTool NMTool, in StepData_Factors theLocalFactors)
    {
        Transfer_TransientProcess TP = aTool.TransientProcess();

        StepShape_OrientedEdge OE = aEdge as StepShape_OrientedEdge;
        StepShape_Edge wEdge = aEdge;
        if (OE is not null)
        {
            wEdge = OE.EdgeElement();
        }

        StepShape_EdgeCurve EC = wEdge as StepShape_EdgeCurve;

        if (aTool.IsBound(EC))
        {
            myResult = aTool.Find(EC);
            if (BRep_Tool.Degenerated(TopoDS.Edge(myResult)))
            {
                TP.AddWarning(EC, "Degenerated Edge in several faces : transferred for each face");
            }
            else
            {
                myError = StepToTopoDS_TranslateEdgeDone;
                done = true;
                //: S4136      B.SameRange(TopoDS::Edge(myResult), false);    //:a5 abv 11 Feb 98
                //: S4136      B.SameParameter(TopoDS::Edge(myResult), false);//:a5
                return;
            }
        }

        // [BEGIN] Proceed with non-manifold cases (ssv; 12.11.2010)
        if (NMTool.IsActive() && NMTool.IsBound(EC))
        {
            TopoDS_Shape existingShape = NMTool.Find(EC);
            // Reverse shape's orientation if needed
            if (!OE.Orientation())
            {
                existingShape.Reverse();
            }

            myResult = existingShape;
            myError = StepToTopoDS_TranslateEdgeDone;
            done = true;
            return;
        }
        // [END] Proceed with non-manifold cases (ssv; 12.11.2010)

        // [BEGIN] Proceed with I-DEAS-like STP (ssv; 15.11.2010)
        TCollection_HAsciiString anECName = EC.Name();
        if (NMTool.IsIDEASCase() && anECName is not null && !anECName.IsEmpty()
            && NMTool.IsBound(anECName.String()))
        {
            TopoDS_Shape existingShape = NMTool.Find(anECName.String());
            // Reverse shape's orientation if needed
            if (!OE.Orientation())
            {
                existingShape.Reverse();
            }

            // Register Edge for final processing (I-DEAS case)
            NMTool.RegisterNMEdge(existingShape);
            myResult = existingShape;
            myError = StepToTopoDS_TranslateEdgeDone;
            done = true;
            return;
        }
        // [END] Proceed with I-DEAS-like STP (ssv; 15.11.2010)

        var B = new BRep_Builder();

        StepGeom_Curve C = EC.EdgeGeometry();
        if (C is null)
        {
            TP.AddFail(EC, " Geom Curve in EdgeCurve is equal to 0");
            myError = StepToTopoDS_TranslateEdgeOther;
            done = false;
            return;
        }

        TopoDS_Edge E = new TopoDS_Edge();
        StepShape_Vertex Vstart, Vend;

        // -----------------------------------------------------------
        // Extract the start and end Vertices corresponding to FORWARD
        // (following the geometrical sense)
        // -----------------------------------------------------------

        bool EdgeCurveSameSense = EC.SameSense();

        if (EdgeCurveSameSense)
        {
            Vstart = EC.EdgeStart();
            Vend = EC.EdgeEnd();
        }
        else
        {
            Vend = EC.EdgeStart();
            Vstart = EC.EdgeEnd();
        }

        TopoDS_Vertex V1 = new TopoDS_Vertex(), V2 = new TopoDS_Vertex();

        var myTranVertex1 = new StepToTopoDS_TranslateVertex(Vstart, aTool, NMTool, theLocalFactors);
        var myTranVertex2 = new StepToTopoDS_TranslateVertex(Vend, aTool, NMTool, theLocalFactors);

        if (myTranVertex1.IsDone())
        {
            V1 = TopoDS.Vertex(myTranVertex1.InternalValue());
            V1.Orientation(TopAbs_FORWARD);
        }

        if (Vend == Vstart)
        {
            V2 = V1;
            V2.Orientation(TopAbs_REVERSED);
        }
        else if (myTranVertex2.IsDone())
        {
            V2 = TopoDS.Vertex(myTranVertex2.InternalValue());
            V2.Orientation(TopAbs_REVERSED);
        }

        done = true;

        // ----------------------------------------------------------
        // --- The EdgeCurve Geometry is of StepGeom_Curve Type
        // --- It can be : * a Pcurve : no 3D curve is constructed
        // ---             * a Surface Curve, Intersection Curve
        // ---               or a Seam Curve
        // ---             * a 3D Curve
        // ----------------------------------------------------------

        if (C is StepGeom_Pcurve)
        {
            B.MakeEdge(ref E);
            //: S4136    B.UpdateEdge (E,preci);
            B.Add(E, V1);
            B.Add(E, V2);
        }
        else if (C is StepGeom_SurfaceCurve)
        {
            // For SeamCurve and IntersectionCurve types
            // --- The Edge Geometry is a Surface Curve ---
            // ---     (3d + 2 Pcurve Or Surface)       ---
            StepGeom_SurfaceCurve Sc = C as StepGeom_SurfaceCurve;
            StepGeom_Curve C1 = Sc.Curve3d();
            MakeFromCurve3D(C1, EC, Vend, Precision(), ref E, ref V1, ref V2, aTool, theLocalFactors);
        }
        else
        {
            // --- The Edge Geometry is a Single 3d Curve ---
            MakeFromCurve3D(C, EC, Vend, Precision(), ref E, ref V1, ref V2, aTool, theLocalFactors);
        }

        // Force set flags SameRange and SameParameter to false
        if (done)
        {
            //: S4136    B.SameRange(E, false);
            //: S4136    B.SameParameter(E, false);
            aTool.Bind(EC, E);

            // Bind Edge in NM tool (ssv; 15.11.2010)
            if (NMTool.IsActive())
            {
                NMTool.Bind(EC, E);
                if (NMTool.IsIDEASCase() && anECName is not null && !anECName.IsEmpty())
                {
                    NMTool.Bind(anECName.String(), E);
                }
            }

            myResult = E;
            myError = StepToTopoDS_TranslateEdgeDone;
        }
    }

    public void Init(StepShape_Edge aEdge, StepToTopoDS_Tool aTool, StepToTopoDS_NMTool NMTool)
        => Init(aEdge, aTool, NMTool, new StepData_Factors());

    //: e6 abv 16 Apr 98: ProSTEP TR8, r0601_sy.stp, #14907
    private static void GetCartesianPoints(StepShape_EdgeCurve EC, ref gp_Pnt P1, ref gp_Pnt P2, in StepData_Factors theLocalFactors)
    {
        for (int i = 1; i <= 2; i++)
        {
            StepShape_Vertex V = ((i == 1) == EC.SameSense() ? EC.EdgeStart() : EC.EdgeEnd());
            StepShape_VertexPoint VP = V as StepShape_VertexPoint;
            if (VP is null)
            {
                continue;
            }

            StepGeom_CartesianPoint P = VP.VertexGeometry() as StepGeom_CartesianPoint;
            Geom_CartesianPoint CP = StepToGeom.MakeCartesianPoint(P, theLocalFactors);
            if (i == 1)
            {
                P1 = CP.Pnt();
            }
            else
            {
                P2 = CP.Pnt();
            }
        }
    }

    /// <summary>The case of a 3D curve (alone or in a surface curve). The curve is assumed to be a 3D curve.</summary>
    public void MakeFromCurve3D(StepGeom_Curve C3D, StepShape_EdgeCurve EC, StepShape_Vertex Vend, double preci, ref TopoDS_Edge E,
                                ref TopoDS_Vertex V1, ref TopoDS_Vertex V2, StepToTopoDS_Tool aTool, in StepData_Factors theLocalFactors)
    {
        Transfer_TransientProcess TP = aTool.TransientProcess();
        Geom_Curve C1 = MakeCurve(C3D, TP, theLocalFactors);
        if (C1 is null)
        {
            TP.AddFail(C3D, " Make Geom_Curve (3D) failed");
            myError = StepToTopoDS_TranslateEdgeOther;
            done = false;
            return;
        }

        // -- Statistics -- -> No Warning message
        aTool.AddContinuity(C1);
        var B = new BRep_Builder();
        double temp1, temp2, U1, U2;
        gp_Pnt pproj;
        gp_Pnt pv1 = BRep_Tool.Pnt(V1);
        gp_Pnt pv2 = BRep_Tool.Pnt(V2);

        //: e6 abv
        gp_Pnt pnt1 = pv1, pnt2 = pv2;
        if (V1.IsSame(V2))
        {
            GetCartesianPoints(EC, ref pnt1, ref pnt2, theLocalFactors);
        }

        var sac = new ShapeAnalysis_Curve();
        temp1 = sac.Project(C1, pnt1, preci, out pproj, out U1, false);
        temp2 = sac.Project(C1, pnt2, preci, out pproj, out U2, false);

        // OCCT.NET: the curve an earlier edge took is frozen; reversed in a copy (decision 1260)
        if (!StepToTopoDS_GeometricTool.UpdateParam3d(ref C1, ref U1, ref U2, preci, true))
        {
            TP.AddWarning(C3D, "Update of 3D-Parameters has failed");
        }

        //: d5: instead of AdjustCurve above which is incorrect if U1 and U2 are not ends
        var aCA = new GeomAdaptor_Curve(C1);
        gp_Pnt pU1 = aCA.Value(U1), pU2 = aCA.Value(U2);
        temp1 = pU1.Distance(pv1);
        temp2 = pU2.Distance(pv2);
        if (temp1 > preci || temp2 > preci)
        {
            // #25415: handling of special case found on some STEP files produced by FPX Expert 2013 (PCB
            // design system): edge curve is line displaced from its true position but with correct
            // direction; we can shift the line in this case so that it passes through vertices correctly
            if (Math.Abs(temp1 - temp2) < preci
                && Math.Abs(U2 - U1 - pnt1.Distance(pnt2)) < Occt.Precision.Confusion()
                && C1 is Geom_Line)
            {
                Geom_Line aLine = C1 as Geom_Line;
                gp_Lin aLin = aLine.Lin();
                gp_Pnt anOrigin = new gp_Pnt(pnt1.XYZ() - aLin.Position().Direction().XYZ() * U1);
                aLin.SetLocation(anOrigin);
                C1 = new Geom_Line(aLin);
                TP.AddWarning(C3D, "Poor result from projection vertex / line 3d, line shifted");
            }
            else
            {
                TP.AddWarning(C3D, "Poor result from projection vertex / curve 3d");
            }
        }

        B.UpdateVertex(V1, 1.000001 * temp1); //: h6 abv 14 Jul 98: PRO8845 #2746: *=1.0001
        B.UpdateVertex(V2, 1.000001 * temp2); //: h6

        var ME = new BRepLib_MakeEdge(C1, V1, V2, U1, U2);
        if (ME.IsDone())
        {
            E = ME.InternalEdge();
            B.Range(E, U1, U2); // abv 14 Mar 00: trj3_pm1-ug.stp #91739, edge 2
        }
        else
        {
            if (ME.Error() == BRepLib_DifferentPointsOnClosedCurve)
            {
                // The Edge could be closed and trimmed by 2 Different vertices
                if (C1.IsClosed())
                {
                    // Attention : topology updating
                    aTool.Bind(Vend, V1);
                    TopoDS_Shape aLocalShape = V1.Reversed();
                    V2 = TopoDS.Vertex(aLocalShape);
                    ME.Init(C1, V1, V2, U1, U2);
                    if (ME.IsDone())
                    {
                        TP.AddWarning(EC, "Wrong topology corrected : Closed Edge with TWO different Vertices");
                        E = ME.InternalEdge();
                    }
                    else
                    {
                        DecodeMakeEdgeError(ME, C3D, ref C1, V1, V2, U1, U2, aTool, EC);
                        E = MakeEdge(C1, V1, V2, U1, U2);
                        myError = StepToTopoDS_TranslateEdgeDone;
                        done = true;
                        //		return;
                    }
                }
                else
                {
                    // Then, this is should be coded as degenerated
                    // To be performed later !!!
                    myError = StepToTopoDS_TranslateEdgeDone;
                    //  Bon, on la fait cette petite edge, mais faudra repasser
                    //  pour l enlever ET FUSIONNER LES VERTEX, pour tout le shell !
                    //  courbe trop petite pour etre mise -> fait planter
                    done = true;
                    if (!V1.IsSame(V2))
                    {
                        TP.AddFail(EC, "This edge has null arc length");
                        gp_Pnt P1 = BRep_Tool.Pnt(V1);
                        gp_Pnt P2 = BRep_Tool.Pnt(V2);
                        gp_Vec avec = new gp_Vec(P1, P2);
                        gp_Dir adir = new gp_Dir(avec);
                        gp_Lin alin = new gp_Lin(P1, adir);
                        C1 = new Geom_Line(alin);
                        U1 = 0.0;
                        U2 = P1.Distance(P2);
                        E = MakeEdge(C1, V1, V2, U1, U2); //,preci
                    }
                    else
                    {
                        TP.AddFail(EC, "NULL EDGE, SKIPPED");
                        myResult.Nullify();
                        return;
                    }
                }
            }
            else
            {
                DecodeMakeEdgeError(ME, C3D, ref C1, V1, V2, U1, U2, aTool, EC);
                E = MakeEdge(C1, V1, V2, U1, U2);
                myError = StepToTopoDS_TranslateEdgeDone;
                done = true;
            }
        }
    }

    /// <summary>Computes an individual pcurve (a 2D curve) of the pcurve entity on the surface; null if it fails.</summary>
    public Geom2d_Curve MakePCurve(StepGeom_Pcurve PCU, Geom_Surface ConvSurf, in StepData_Factors theLocalFactors)
    {
        Geom2d_Curve C2d = null;
        StepRepr_DefinitionalRepresentation DRI = PCU.ReferenceToCurve();
        if (DRI is null)
        {
            return C2d;
        }

        StepGeom_Curve StepCurve = DRI.ItemsValue(1) as StepGeom_Curve;
        try
        {
            C2d = StepToGeom.MakeCurve2d(StepCurve, theLocalFactors);
            if (C2d is not null)
            {
                // -- if the surface is a RectangularTrimmedSurface,
                // -- send the BasisSurface.
                C2d = GeomConvert_Units.DegreeToRadian(C2d, ConvSurf, theLocalFactors.LengthFactor(), theLocalFactors.FactorDegreeRadian());
            }
        }
        catch (Standard_Failure)
        {
            return C2d;
        }

        return C2d;
    }

    public Geom2d_Curve MakePCurve(StepGeom_Pcurve PCU, Geom_Surface ConvSurf) => MakePCurve(PCU, ConvSurf, new StepData_Factors());

    /// <summary>The edge, frozen.</summary>
    /// <exception cref="StdFail_NotDone">The translation wasn't done.</exception>
    public TopoDS_Shape Value()
    {
        TopoDS_Shape aShape = InternalValue();
        aShape.TShape()?.FreezeGraph();
        return aShape;
    }

    /// <summary>As <see cref="Value"/>, without freezing: the reader's working shape.</summary>
    internal TopoDS_Shape InternalValue()
    {
        StdFail_NotDone_Raise_if(!done, "StepToTopoDS_TranslateEdge::Value() - no result");
        return myResult;
    }

    public StepToTopoDS_TranslateEdgeError Error() => myError;
}
