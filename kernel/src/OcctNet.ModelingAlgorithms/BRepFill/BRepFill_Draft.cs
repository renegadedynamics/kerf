// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Draft.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Draft.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Geometry.Properties;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Schematiq.Collections;
using static Occt.BOPAlgo_GlueEnum;
using static Occt.BOPAlgo_Operation;
using static Occt.BRepFill_TransitionStyle;
using static Occt.BRepLib_WireError;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Occt;

// OCCT_DEBUG blocks (traces) are left out.

/// <summary>
/// A draft surface: a ruled skin swept from a wire (or a face's outer wire, or a shell's free boundary) along a
/// direction at an angle, up to a length, a surface or a shape, sewn to the input face or shell.
/// </summary>
/// <remarks>
/// A working object (decision 21), the engine of <see cref="BRepOffsetAPI_MakeDraft"/>. It reads its input: the
/// sweep, the Booleans and the sewing build new shapes. OCCT marks the wire closed in place when its ends are the same
/// vertex; the port marks a new wire with the same edges when the caller's is frozen (decision 1045).
/// <see cref="Shape"/>, <see cref="Shell"/> and <see cref="Generated"/> freeze what they return; the internal forms
/// don't.
/// </remarks>
public sealed class BRepFill_Draft
{
    private Direction3D myDir;
    private double myAngle;
    private double angmin;
    private double angmax;
    private double myTol;
    private BRepFill_DraftLaw? myLoc;
    private BRepFill_SectionLaw? mySec;
    private NCollection_HArray2<Shape>? mySections;
    private NCollection_HArray2<Shape>? myFaces;
    private readonly NCollection_List<Shape> myGenerated = new();
    private Shape myShape = new();
    private readonly Shape myTop;
    private Shell myShell = new();
    private Wire myWire = new();
    private readonly Continuity myCont;
    private BRepFill_TransitionStyle myStyle;
    private bool IsInternal;
    private bool myDone;

    //=================================================================================================

    private static void ComputeTrsf(in Wire W, in Direction3D D, ref Box Box, ref Transform3D Tf)
    {
        // Calculate approximate barycenter
        WireExplorer Exp = new(W);
        // Class BRep_Tool without fields and without Constructor :
        //  BRep_Tool BT;
        Xyz Bary = new(0.0, 0.0, 0.0);
        int nb;

        for (nb = 0; Exp.More(); Exp.Next())
        {
            //    Bary += BT.Pnt(Exp.CurrentVertex()).XYZ();
            Bary += Tool.Pnt(Exp.CurrentVertex()).XYZ();
            nb++;
        }
        Bary /= nb;

        // Calculate the Transformation
        CoordinateSystem N = new(new Point3D(Bary), D);
        Tf.SetTransformation(N);
        EdgeCurveAdaptor AC = new();
        //  BndLib_Add3dCurve BC;

        // transformation to the wire
        Wire TheW = W;
        Placement Loc = new(Tf);
        TheW.Location(Loc);

        // Calculate the box
        Box.SetVoid();
        for (Exp.Init(TheW); Exp.More(); Exp.Next())
        {
            AC.Initialize(Exp.Current());
            //    BC.Add(AC, 0.1, Box);
            Add3dCurve.Add(AC, 0.1, ref Box);
        }
    }

    //=================================================================================================

    private static double Longueur(in Box WBox, in Box SBox, ref Direction3D D, ref Point3D P)
    {
        // face of the box most remoted from the face input in
        // the direction of skin
        double Xmin, Ymin, Zmin, Xmax, Ymax, Zmax, WZmin, WZmax, L;

        //"coord" of the box
        WBox.Get(out Xmin, out Ymin, out Zmin, out Xmax, out Ymax, out Zmax);
        WZmin = Zmin;
        WZmax = Zmax;

        SBox.Get(out Xmin, out Ymin, out Zmin, out Xmax, out Ymax, out Zmax);
        P.SetCoord((Xmin + Xmax) / 2, (Ymin + Ymax) / 2, Zmax);

        if (Zmax < WZmin)
        {
            // Skin in the wrong direction. Invert...
            D.Reverse();
            L = WZmax - Zmin;
            P.SetZ(Zmin);
        }
        else
        {
            L = Zmax - WZmin;
        }
        return L;
    }

    //=======================================================================
    // function : GoodOrientation
    // purpose  : Check if the law is oriented to have an exterior skin
    //======================================================================
    private static bool GoodOrientation(in Box B, BRepFill_LocationLaw Law, in Direction3D D)
    {
        double f, l, r, t = 0.0;

        B.Get(out double aXmin, out double aYmin, out double aZmin, out double aXmax, out double aYmax, out double aZmax);
        Point3D P1 = new(aXmin, aYmin, aZmin), P2 = new(aXmax, aYmax, aZmax);
        Vector3D V = new(P1, P2);

        Law.CurvilinearBounds(Law.NbLaw(), out f, out l);
        r = V.Magnitude() / l;

        int ii, Ind;
        int Nb = (int)(4 + (10 * r));
        r = l / Nb;

        Nb++; // Number of points

        NCollection_Array1<Point3D> Pnts = new(1, Nb);
        CurveAdaptorBase AC;
        Xyz Bary = new(0.0, 0.0, 0.0);

        for (ii = 1; ii <= Nb; ii++)
        {
            Law.Parameter((ii - 1) * r, out Ind, ref t);
            AC = Law.Law(Ind).GetCurve()!;
            AC.D0(t, out Pnts[ii]);
            Bary += Pnts[ii].XYZ();
        }

        Bary /= Nb;
        Point3D Centre = new(Bary);
        Vector3D Normal = new(D.XYZ());
        double Angle = 0;
        Vector3D Ref = new(Centre, Pnts[1]);

        for (ii = 2; ii <= Nb; ii++)
        {
            Vector3D R = new(Centre, Pnts[ii]);
            Angle += Ref.AngleWithRef(R, Normal);
            Ref = R;
        }

        return (Angle >= 0);
    }

    //=================================================================================================

    /// <summary>
    /// The draft of the wire, face or shell <paramref name="S"/> in the direction <paramref name="Dir"/> at the angle
    /// <paramref name="Angle"/>.
    /// </summary>
    /// <exception cref="ConstructionErrorException">Another shape type, or a shell without a free boundary wire.</exception>
    public BRepFill_Draft(in Shape S, in Direction3D Dir, double Angle)
    {
        myLoc = null;
        mySec = null;
        myFaces = null;
        mySections = null;

        switch (S.ShapeType())
        {
            case ShapeType.Wire:
            {
                myWire = ShapeCasts.Wire(S);
                break;
            }
            case ShapeType.Face:
            {
                Iterator Exp = new(S);
                myWire = ShapeCasts.Wire(Exp.Value());
                break;
            }
            case global::Kerf.Topology.ShapeType.Shell:
            {
                NCollection_List<Shape> List = new();
                NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> edgemap =
                    new(ShapeHasher.Instance);
                Exploration.MapShapesAndAncestors(S, ShapeType.Edge, ShapeType.Face, edgemap);
                int iedge, nbf;
                for (iedge = 1; iedge <= edgemap.Extent(); iedge++)
                {
                    Edge theEdge = ShapeCasts.Edge(edgemap.FindKey(iedge));
                    // skip degenerated edges
                    if (!Tool.Degenerated(theEdge))
                    {
                        nbf = edgemap[iedge].Extent();
                        if (nbf == 1)
                        {
                            List.Append(theEdge);
                        }
                    }
                }

                if (List.Extent() > 0)
                {
                    BRepLib_MakeWire MW = new();
                    MW.Add(List);
                    BRepLib_WireError Err = MW.Error();
                    if (Err == BRepLib_WireDone)
                    {
                        myWire = MW.InternalWire();
                    }
                    else
                    {
                        throw new ConstructionErrorException("BRepFill_Draft");
                    }
                }
                else
                {
                    throw new ConstructionErrorException("BRepFill_Draft");
                }
                break;
            }
            default:
                throw new ConstructionErrorException("BRepFill_Draft");
        }

        // Attention to closed non declared wires !
        if (!myWire.Closed())
        {
            Exploration.Vertices(myWire, out Vertex Vf, out Vertex Vl);
            if (Vf.IsSame(Vl))
            {
                // OCCT.NET: OCCT sets the flag on the caller's wire; a frozen one is replaced by a new wire with the same
                // edges (decision 1045).
                if (myWire.TShape()!.IsFrozen)
                {
                    Wire aNewWire = ShapeCasts.Wire(myWire.EmptyCopied());
                    BrepBuilder aBB = new();
                    for (Iterator anIt = new(myWire, false, false); anIt.More(); anIt.Next())
                    {
                        aBB.Add(aNewWire, anIt.Value());
                    }
                    myWire = aNewWire;
                }
                myWire.Closed(true);
            }
        }

        myAngle = Math.Abs(Angle);
        myDir = Dir;
        myTop = S;
        myDone = false;
        myTol = 1.0e-4;
        myCont = C1;
        SetOptions();
        SetDraft();
    }

    //=================================================================================================

    /// <summary>Sets the transition style at the wire's corners and the angular control of the sweep.</summary>
    public void SetOptions(BRepFill_TransitionStyle Style = BRepFill_Right, double Min = 0.01, double Max = 3.0)
    {
        myStyle = Style;
        angmin = Min;
        angmax = Max;
    }

    //=================================================================================================

    /// <summary>True for an internal draft (the skin inside the wire).</summary>
    public void SetDraft(bool Internal = false)
    {
        IsInternal = Internal;
    }

    //=======================================================================
    // function :Perform
    // purpose  : calculate a surface of skinning
    //======================================================================
    /// <summary>Builds the draft up to the length <paramref name="LengthMax"/>.</summary>
    public void Perform(double LengthMax)
    {
        Surface? S = null;
        Box WBox = new(); //, SBox;
        Transform3D Trsf = new();

        ComputeTrsf(myWire, myDir, ref WBox, ref Trsf);
        Init(S, LengthMax, WBox);
        BuildShell(S);
        Sewing();
    }

    //=======================================================================
    // function :Perform
    // purpose  : calculate a surface of skinning
    //======================================================================
    /// <summary>Builds the draft up to the surface <paramref name="Surface"/>.</summary>
    public void Perform(Surface Surface, bool KeepInsideSurface = true)
    {
        Box WBox = new(), SBox = new();
        Transform3D Trsf = new();
        Point3D Pt = new();
        double L;

        ComputeTrsf(myWire, myDir, ref WBox, ref Trsf);

        // box with bounds of the stop surface
        Surface Surf;
        Surf = (Surface)Surface.Transformed(Trsf);
        SurfaceAdaptor S1 = new(Surf);
        //  BndLib_AddSurface AS;
        //  AS.Add(S1, 0.1, SBox);
        AddSurface.Add(S1, 0.1, ref SBox);

        // calculate the maximum length of the rule.
        L = Longueur(WBox, SBox, ref myDir, ref Pt);
        L /= Math.Abs(Math.Cos(myAngle));

        // Construction
        Init(Surface, L, WBox);
        BuildShell(Surface, !KeepInsideSurface);
        Sewing();
    }

    //================================================================
    // function :Perform
    // purpose  : calculate the surface of skinning, stopped by a shape
    //================================================================
    /// <summary>Builds the draft up to the shape <paramref name="StopShape"/>.</summary>
    public void Perform(in Shape StopShape, bool KeepOutSide = true)
    {
        Box WBox = new(), SBox = new();
        Transform3D Trsf = new();
        Point3D Pt = new();
        double L;

        ComputeTrsf(myWire, myDir, ref WBox, ref Trsf);

        // bounding box of the stop shape
        Box BSurf = new(); //, TheBox;
        double Umin, Umax, Vmin, Vmax;
        //  BRepTools B;
        //  BRep_Tool BT;
        Surface Surf;

        //  BndLib_AddSurface AS;

        Explorer Ex = new(StopShape, ShapeType.Face);

        SBox.SetVoid();
        while (Ex.More())
        { // parse faces of the stop shape
            //    B.UVBounds(TopoDS::Face(Ex.Current()), Umin,Umax,Vmin,Vmax);
            BrepTools.UVBounds(ShapeCasts.Face(Ex.Current()), out Umin, out Umax, out Vmin, out Vmax);
            Surf = (Surface)Tool.Surface(ShapeCasts.Face(Ex.Current()))!.Transformed(Trsf);
            SurfaceAdaptor S1 = new(Surf);
            // bounding box of the current face
            //    AS.Add(S1, Umin, Umax, Vmin, Vmax, 0.1, BSurf);
            AddSurface.Add(S1, Umin, Umax, Vmin, Vmax, 0.1, ref BSurf);
            SBox.Add(BSurf); // group boxes
            Ex.Next();
        } // while_Ex

        // calculate the maximum length of the rule.
        L = Longueur(WBox, SBox, ref myDir, ref Pt);
        L /= Math.Abs(Math.Cos(myAngle));

        // surface of stop
        Transform3D Inv;
        Inv = Trsf.Inverted(); // inverted transformation
        Pt.Transform(Inv); // coordinate in the absolute reference
        PlaneSurface Plan = new(Pt, myDir);
        Surf = new TrimmedSurface(Plan, -L, L, -L, L);

        // Sweeping and restriction
        Init(Plan, L * 1.01, WBox);
        BuildShell(Surf, false);
        Fuse(StopShape, KeepOutSide);
        Sewing();
    }

    //=================================================================================================

    private void Init(Surface? Surf, double Length, in Box Box)
    {
        bool B;

        // law of positioning
        GeomFill_LocationDraft Loc = new(myDir, myAngle);
        myLoc = new BRepFill_DraftLaw(myWire, Loc);

        B = GoodOrientation(Box, myLoc, myDir);

        if (IsInternal ^ (!B))
        {
            myAngle = -myAngle;
            Loc.SetAngle(myAngle);
            myLoc = new BRepFill_DraftLaw(myWire, Loc);
        }

        myLoc.CleanLaw(angmin); // Clean small discontinuities.

        // law of section
        // generating line is straight and parallel to binormal.
        Point3D P = new(0, 0, 0);
        Vector3D D = new(0.0, 1.0, 0.0);

        // Control of the orientation
        myLoc.Law(1).GetDomain(out double f, out double l);
        Matrix3x3 M = new();

        Vector3D Bid = new();
        myLoc.Law(1).D0((f + l) / 2, ref M, ref Bid);
        Direction3D BN = new(M.Column(2));

        double ang = myDir.Angle(BN);
        if (ang > M_PI / 2)
        {
            D.Reverse();
        }
        LineCurve3D L = new(P, new Direction3D(D));

        Curve3D TC = new TrimmedCurve3D(L, 0, Length);

        BRepLib_MakeEdge ME = new(TC);
        Edge EG = ME.InternalEdge();

        BRepLib_MakeWire MW = new(EG);
        Wire G = MW.InternalWire();

        mySec = new BRepFill_ShapeLaw(G, true);
    }

    //=======================================================================
    // function : BuildShell
    // purpose  : Construction of the skinning surface
    //======================================================================
    private void BuildShell(Surface? Surf, bool KeepOutSide = false)
    {
        // construction of the surface
        BRepFill_Sweep Sweep = new(mySec!, myLoc!, true);
        Sweep.SetTolerance(myTol);
        Sweep.SetAngularControl(angmin, angmax);
        NCollection_Map<Shape> Dummy = new(ShapeHasher.Instance);
        NCollection_DataMap<Shape, NCollection_HArray2<Shape>> Dummy2 = new(ShapeHasher.Instance);
        NCollection_DataMap<Shape, NCollection_HArray2<Shape>> Dummy3 = new(ShapeHasher.Instance);
        Sweep.Build(Dummy, Dummy2, Dummy3, myStyle, myCont);
        if (Sweep.IsDone())
        {
            myShape = Sweep.Shape();
            myShell = ShapeCasts.Shell(myShape);
            myFaces = Sweep.SubShape();
            mySections = Sweep.Sections();
            myDone = true;
            // Control of the orientation
            bool @out = true;
            Explorer ex = new(myShell, ShapeType.Face);
            Face F;
            F = ShapeCasts.Face(ex.Current());
            FaceSurfaceAdaptor SF = new(F);
            double u, v;
            Vector3D V;
            u = SF.FirstUParameter();
            v = SF.FirstVParameter();
            SF.D1(u, v, out Point3D P, out Vector3D V1, out Vector3D V2);
            V = V1.Crossed(V2);
            if (F.Orientation() == Reversed)
            {
                V.Reverse();
            }
            if (V.Magnitude() > 1.0e-10)
            {
                @out = myDir.Angle(new Direction3D(V)) > M_PI / 2;
            }
            if (@out == IsInternal)
            {
                myShell.Reverse();
                myShape.Reverse();
            }
        }
        else
        {
            myDone = false;
            return;
        }

        if (Surf is not null)
        { // Add the face at end

            // Waiting the use of traces & restriction in BRepFill_Sweep
            // Make Fuse.
            BRepLib_MakeFace MkF = new();
            MkF.Init(Surf, true, global::Kerf.Engine.Foundation.Precision.Confusion());
            Fuse(MkF.InternalFace(), KeepOutSide);
        }
    }

    //=======================================================================
    // function : Fuse
    // purpose  : Boolean operation between the skin and the
    //           stop shape
    //======================================================================
    private bool Fuse(in Shape StopShape, bool KeepOutSide)
    {
        BrepBuilder B = new();
        bool issolid = false;
        Solid Sol1 = new(), Sol2 = new();
        Containment State1 = Out, State2 = Out;

        if (myShape.ShapeType() == ShapeType.Solid)
        {
            Sol1 = ShapeCasts.Solid(myShape);
            issolid = true;
        }
        else
        {
            B.MakeSolid(ref Sol1);
            B.Add(Sol1, myShape); // shell => solid (for fusion)
        }

        switch (StopShape.ShapeType())
        {
            case ShapeType.Compound:
            {
                Iterator It = new(StopShape);
                return Fuse(It.Value(), KeepOutSide);
            }
            case ShapeType.Solid:
            {
                Sol2 = ShapeCasts.Solid(StopShape);
                break;
            }
            case global::Kerf.Topology.ShapeType.Shell:
            {
                B.MakeSolid(ref Sol2);
                B.Add(Sol2, StopShape); // shell => solid (for fusion)
                break;
            }

            case ShapeType.Face:
            {
                Shell S = new();
                B.MakeShell(ref S);
                B.Add(S, StopShape);
                S.Closed(Tool.IsClosed(S));
                B.MakeSolid(ref Sol2);
                B.Add(Sol2, S); // shell => solid (for fusion)
                break;
            }

            default:
            {
                return false; // Impossible to do
            }
        }

        // Perform intersection of solids
        BOPAlgo_PaveFiller aPF = new();
        NCollection_List<Shape> anArgs = new();
        anArgs.Append(Sol1);
        anArgs.Append(Sol2);
        aPF.SetArguments(anArgs);
        aPF.Perform();
        if (aPF.HasErrors())
        {
            return false;
        }

        BRepAlgoAPI_Section aSec = new(Sol1, Sol2, aPF);
        Shape aSection = aSec.InternalShape();

        Explorer exp = new(aSection, ShapeType.Edge);
        if (!exp.More())
        {
            // No section edges produced
            return false;
        }

        if (StopShape.ShapeType() != ShapeType.Solid)
        {
            // It is required to choose the state by the geometry

            // We need to find the section edge, closest to myWire
            Edge aSEMin = new();
            double Dmin = global::Kerf.Engine.Foundation.Precision.Infinite();
            BRepExtrema_DistShapeShape DistTool = new();
            DistTool.LoadS1(myWire);

            for (; exp.More(); exp.Next())
            {
                Shape aSE = exp.Current();
                DistTool.LoadS2(aSE);
                DistTool.Perform();
                if (DistTool.IsDone())
                {
                    double D = DistTool.Value();
                    if (D < Dmin)
                    {
                        Dmin = D;
                        aSEMin = ShapeCasts.Edge(aSE);
                        if (Dmin < global::Kerf.Engine.Foundation.Precision.Confusion())
                        {
                            break;
                        }
                    }
                }
            }

            if (!aSEMin.IsNull)
            {
                // Get geometry of StopShape
                Surface? S = null;
                Curve2D? C2d = null;
                double f = 0.0, l = 0.0;
                Placement L = new();
                Tool.CurveOnSurface(aSEMin, ref C2d, ref S, ref L, ref f, ref l, 2);

                // Find a normal.
                // OCCT dereferences the second pcurve without a check.
                C2d!.D0((f + l) / 2, out Point2D P2d);
                SLProps SP = new(S!, P2d.X(), P2d.Y(), 1, 1.0e-12);
                if (!SP.IsNormalDefined)
                {
                    C2d.D0((3 * f + l) / 4, out P2d);
                    SP.SetParameters(P2d.X(), P2d.Y());
                    if (!SP.IsNormalDefined)
                    {
                        C2d.D0((f + 3 * l) / 4, out P2d);
                        SP.SetParameters(P2d.X(), P2d.Y());
                    }
                }

                if (SP.IsNormalDefined)
                {
                    // Subtract State1
                    if (myDir.Angle(SP.Normal()) < M_PI / 2)
                    {
                        State1 = In;
                    }
                    else
                    {
                        State1 = Out;
                    }
                }
            }
        }

        if (!KeepOutSide)
        { // Invert State2;
            if (State2 == In)
            {
                State2 = Out;
            }
            else
            {
                State2 = In;
            }
        }

        // Perform Boolean operation
        BOPAlgo_Builder aBuilder = new();
        aBuilder.AddArgument(Sol1);
        aBuilder.AddArgument(Sol2);
        aBuilder.PerformWithFiller(aPF);
        if (aBuilder.HasErrors())
        {
            return false;
        }

        Shape result = new();
        History aHistory = new();

        bool isSingleOpNeeded = true;
        // To get rid of the unnecessary parts of first solid make the cutting first
        if (State1 == Out)
        {
            NCollection_List<Shape> aLO = new(), aLT = new();
            aLO.Append(Sol1);
            aLT.Append(Sol2);
            aBuilder.BuildBOP(aLO, aLT, BOPAlgo_CUT, new ProgressRange());
            if (!aBuilder.HasErrors())
            {
                Solid aCutMin = new();
                Explorer anExpS = new(aBuilder.InternalShape(), ShapeType.Solid);
                if (anExpS.More())
                {
                    aCutMin = ShapeCasts.Solid(anExpS.Current());
                    anExpS.Next();
                    if (anExpS.More())
                    {
                        double aDMin = global::Kerf.Engine.Foundation.Precision.Infinite();
                        BRepExtrema_DistShapeShape DistTool = new();
                        DistTool.LoadS1(myWire);

                        for (anExpS.ReInit(); anExpS.More(); anExpS.Next())
                        {
                            Shape aCut = anExpS.Current();
                            DistTool.LoadS2(aCut);
                            DistTool.Perform();
                            if (DistTool.IsDone())
                            {
                                double D = DistTool.Value();
                                if (D < aDMin)
                                {
                                    aDMin = D;
                                    aCutMin = ShapeCasts.Solid(aCut);
                                }
                            }
                        }
                    }
                }

                if (!aCutMin.IsNull)
                {
                    // Save history for first argument only
                    aHistory.Merge(aLO, new BuilderHistory(aBuilder));

                    // Perform needed operation with result of Cut
                    BOPAlgo_Builder aGluer = new();
                    aGluer.AddArgument(aCutMin);
                    aGluer.AddArgument(Sol2);
                    aGluer.SetGlue(BOPAlgo_GlueShift);
                    aGluer.Perform();

                    aLO.Clear();
                    aLO.Append(aCutMin);
                    aGluer.BuildBOP(aLO, State1, aLT, State2, new ProgressRange());

                    if (!aGluer.HasErrors())
                    {
                        aHistory.Merge(aGluer.InternalHistory());

                        result = aGluer.InternalShape();
                        anExpS.Init(result, ShapeType.Solid);
                        isSingleOpNeeded = !anExpS.More();
                    }
                }
            }
        }

        if (isSingleOpNeeded)
        {
            aHistory.Clear();

            NCollection_List<Shape> aLO = new(), aLT = new();
            aLO.Append(Sol1);
            aLT.Append(Sol2);

            aBuilder.BuildBOP(aLO, State1, aLT, State2, new ProgressRange());
            if (aBuilder.HasErrors())
            {
                return false;
            }

            aHistory.Merge(aBuilder.InternalHistory());
            result = aBuilder.InternalShape();
        }

        if (issolid)
        {
            myShape = result;
        }
        else
        {
            Explorer Exp = new();
            Exp.Init(result, global::Kerf.Topology.ShapeType.Shell);
            if (Exp.More())
            {
                myShape = Exp.Current();
            }
        }

        // Update the History
        int ii;
        for (ii = 1; ii <= myLoc!.NbLaw(); ii++)
        {
            var L = aHistory.Modified(myFaces!.Value(1, ii));
            if (L.Count > 0)
            {
                myFaces.SetValue(1, ii, L[0]);
            }
        }
        for (ii = 1; ii <= myLoc.NbLaw() + 1; ii++)
        {
            var L = aHistory.Modified(mySections!.Value(1, ii));
            if (L.Count > 0)
            {
                mySections.SetValue(1, ii, L[0]);
            }
        }

        return true;
    }

    // OCCT builds a BRepTools_History from any algorithm with Modified, Generated and IsDeleted (a template); the
    // port's history takes the interface, which the Boolean builder doesn't implement.
    private sealed class BuilderHistory(BOPAlgo_BuilderShape theAlgo) : History.IAlgorithm
    {
        public bool IsDeleted(in Shape theS) => theAlgo.IsDeleted(theS);

        public IEnumerable<Shape> Modified(in Shape theS) => theAlgo.Modified(theS);

        public IEnumerable<Shape> Generated(in Shape theS) => theAlgo.Generated(theS);
    }

    //=======================================================================
    // function : Sewing
    // purpose  : Assemble the skin with the above face
    //======================================================================
    private bool Sewing()
    {
        bool ToAss;
        bool Ok = false;
        ToAss = (myTop.ShapeType() != ShapeType.Wire);

        if ((!ToAss) || (!myDone))
        {
            return false;
        }

        // Assembly make a shell from the faces of the shape + the input shape
        BRepBuilderAPI_Sewing Ass = new(5 * myTol, true, true, false);
        Ass.Add(myShape);
        Ass.Add(myTop);
        ToAss = true;

        int NbCE;

        Ass.Perform();
        // Check if the assembly is real.
        NbCE = Ass.NbContigousEdges();

        if (NbCE > 0)
        {
            Shape res;
            res = Ass.InternalSewedShape();
            if ((res.ShapeType() == global::Kerf.Topology.ShapeType.Shell) || (res.ShapeType() == ShapeType.Solid))
            {
                myShape = res;
                Ok = true;
            }
            else if (res.ShapeType() == ShapeType.Compound)
            {
                Iterator It = new(res);
                res = It.Value();
                It.Next();
                if (!It.More())
                { // Only one part => this is correct
                    myShape = res;
                    Ok = true;
                }
            }
        }

        if (Ok)
        {
            // Update the History
            int ii;
            for (ii = 1; ii <= myLoc!.NbLaw(); ii++)
            {
                if (Ass.IsModified(myFaces!.Value(1, ii)))
                {
                    myFaces.SetValue(1, ii, Ass.InternalModified(myFaces.Value(1, ii)));
                }
            }
            for (ii = 1; ii <= myLoc.NbLaw() + 1; ii++)
            {
                if (Ass.IsModified(mySections!.Value(1, ii)))
                {
                    mySections.SetValue(1, ii, Ass.InternalModified(mySections.Value(1, ii)));
                }
            }

            if (myShape.Closed())
            { // Make a Solid
                Solid solid = new();
                BrepBuilder BS = new();
                BS.MakeSolid(ref solid);
                BS.Add(solid, ShapeCasts.Shell(myShape));

                BRepClass3d_SolidClassifier SC = new(solid);
                SC.PerformInfinitePoint(global::Kerf.Engine.Foundation.Precision.Confusion());
                if (SC.State() == In)
                {
                    BS.MakeSolid(ref solid);
                    myShape.Reverse();
                    BS.Add(solid, ShapeCasts.Shell(myShape));
                }
                myShape = solid;
            }
        }
        return Ok;
    }

    //=======================================================================
    // function : Generated
    // purpose  : return a sub-part generated by sweeping
    //======================================================================
    /// <summary>The face generated from an edge of the wire (frozen).</summary>
    public ISchematiqList<Shape> Generated(in Shape S)
    {
        NCollection_List<Shape> aList = InternalGenerated(S);
        var aShapes = new List<Shape>();
        for (NCollection_List<Shape>.Iterator anIt = new(aList); anIt.More(); anIt.Next())
        {
            anIt.Value().TShape()?.FreezeGraph();
            aShapes.Add(anIt.Value());
        }
        return PackedList.Own([.. aShapes]);
    }

    /// <summary><see cref="Generated"/> without freezing, for ported code.</summary>
    internal NCollection_List<Shape> InternalGenerated(in Shape S)
    {
        myGenerated.Clear();
        Edge E;
        int ii;
        E = ShapeCasts.Edge(S);
        if (E.IsNull)
        {
            // OCCT: E is null here, so no vertex matches (a vertex's S throws in TopoDS::Edge above).
            for (ii = 0; ii <= myLoc!.NbLaw(); ii++)
            {
                if (E.IsSame(myLoc.Vertex(ii)))
                {
                    myGenerated.Append(mySections!.Value(1, ii + 1));
                    break;
                }
            }
        }
        else
        {
            for (ii = 1; ii <= myLoc!.NbLaw(); ii++)
            {
                if (E.IsSame(myLoc.Edge(ii)))
                {
                    myGenerated.Append(myFaces!.Value(1, ii));
                    break;
                }
            }
        }

        return myGenerated;
    }

    //=================================================================================================

    /// <summary>The result (frozen): the skin sewn to the input face or shell, as a shell or solid.</summary>
    public Shape Shape()
    {
        myShape.TShape()?.FreezeGraph();
        return myShape;
    }

    internal Shape InternalShape() => myShape;

    //=====================================================================
    // function : Shell
    // purpose  : surface of skinning with the input face (=>shell)
    //=====================================================================
    /// <summary>The draft skin alone (frozen).</summary>
    public Shell Shell()
    {
        myShell.TShape()?.FreezeGraph();
        return myShell;
    }

    internal Shell InternalShell() => myShell;

    //=================================================================================================

    /// <summary>True if the draft was built.</summary>
    public bool IsDone() => myDone;
}
