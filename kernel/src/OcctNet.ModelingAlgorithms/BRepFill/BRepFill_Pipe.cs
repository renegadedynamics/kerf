// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Pipe.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Pipe.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BRepFill_TransitionStyle;
using static Occt.GeomAbs_Shape;
using static Occt.GeomFill_ApproxStyle;
using static Occt.GeomFill_Trihedron;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// Creates a shape by sweeping a shape (the profile) along a wire (the spine). For each edge or vertex of the spine,
/// the shape generated from each sub-shape of the profile can be asked for.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21), as BRepSweep_Prism under BRepPrimAPI_MakePrism: it sweeps the profile in place, as
/// OCCT does, and the sweep writes into the profile's sub-shapes it shares in the result (the faces at the ends of a
/// face's pipe, the section edges). Given a frozen profile, such a write throws <see cref="Standard_ImmutableObject"/>;
/// <see cref="BRepOffsetAPI_MakePipe"/> gives it a copy of a frozen profile and maps its answers back to the caller's
/// sub-shapes (decision 910).
/// </para>
/// <para>
/// The accessors hand out frozen shapes; ported callers that keep building (BRepFill_Evolved) use the
/// <c>Internal</c> forms (decision 911).
/// </para>
/// <para>OCCT.NET: the topological sweep (BRepFill_Sweep) is ported by unit 5S2; until then every pipe construction
/// throws <see cref="Standard_NotImplemented"/> (decision 909).</para>
/// </remarks>
public sealed class BRepFill_Pipe
{
    private TopoDS_Wire mySpine = new();
    private TopoDS_Shape myProfile = new();
    private TopoDS_Shape myShape = new();
    private gp_Trsf myTrsf = new();
    private BRepFill_LocationLaw? myLoc;
    private NCollection_HArray2<TopoDS_Shape>? mySections;
    private NCollection_HArray2<TopoDS_Shape>? myFaces;
    private NCollection_HArray2<TopoDS_Shape>? myEdges;
    private readonly NCollection_Map<TopoDS_Shape> myReversedEdges = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_HArray2<TopoDS_Shape>> myTapes = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_HArray2<TopoDS_Shape>> myRails = new(TopTools_ShapeMapHasher.Instance);
    private int myCurIndexOfSectionEdge;
    private TopoDS_Shape myFirst = new();
    private TopoDS_Shape myLast = new();
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myGenMap = new(TopTools_ShapeMapHasher.Instance);
    private int myDegmax;
    private int mySegmax;
    private GeomAbs_Shape myContinuity;
    private GeomFill_Trihedron myMode;
    private bool myForceApproxC1;
    private double myErrorOnSurf; // OCCT: uninitialised until a sweep

    //=================================================================================================

    private static bool UpdateMap(in TopoDS_Shape theKey,
                                  in TopoDS_Shape theValue,
                                  NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theMap)
    {
        if (!theMap.IsBound(theKey))
        {
            NCollection_List<TopoDS_Shape> thelist = new();
            theMap.Bind(theKey, thelist);
        }
        NCollection_List<TopoDS_Shape> aList = theMap.ChangeFind(theKey);
        NCollection_List<TopoDS_Shape>.Iterator anIt = new(aList);
        bool found = false;

        for (; anIt.More(); anIt.Next())
        {
            if (theValue.IsSame(anIt.Value()))
            {
                found = true;
                break;
            }
        }

        if (!found)
        {
            aList.Append(theValue);
        }

        return !found;
    }

    private static void UpdateTolFromTopOrBottomPCurve(in TopoDS_Face aFace, in TopoDS_Edge anEdge)
    {
        Geom2d_Curve? aPCurve = BRep_Tool.CurveOnSurface(anEdge, aFace, out double fpar, out double lpar);
        if (aPCurve is null)
        {
            return;
        }

        Geom_Curve? aCurve = BRep_Tool.Curve(anEdge, out fpar, out lpar);
        if (aCurve is null)
        {
            return;
        }

        Geom2dAdaptor_Curve GAHC2d = new(aPCurve, fpar, lpar);
        Geom_Surface aSurf = BRep_Tool.Surface(aFace)!;
        GeomAdaptor_Surface GAHS = new(aSurf);
        Adaptor3d_CurveOnSurface ConS = new(GAHC2d, GAHS);

        double Tol = BRep_Tool.Tolerance(anEdge);
        double InitTol = Tol;
        double TolTol = Tol * Tol;
        const int NCONTROL = 22;
        double delta = (lpar - fpar) / NCONTROL;
        for (int i = 0; i <= NCONTROL; i++)
        {
            double par = fpar + i * delta;
            gp_Pnt pnt = aCurve.Value(par);
            gp_Pnt prj = ConS.Value(par);
            double sqdist = pnt.SquareDistance(prj);
            if (sqdist > TolTol)
            {
                TolTol = sqdist;
            }
        }
        Tol = 1.00005 * Math.Sqrt(TolTol);
        if (Tol >= InitTol)
        {
            BRep_Builder BB = new();
            BB.UpdateEdge(anEdge, Tol);
            TopoDS_Iterator itv = new(anEdge);
            for (; itv.More(); itv.Next())
            {
                TopoDS_Vertex aVertex = TopoDS.Vertex(itv.Value());
                BB.UpdateVertex(aVertex, Tol);
            }
        }
    }

    //=================================================================================================

    /// <summary>An empty pipe; <see cref="Perform"/> builds it.</summary>
    public BRepFill_Pipe()
    {
        myDegmax = 11;
        mySegmax = 100;
        myContinuity = GeomAbs_C2;
        myMode = GeomFill_IsCorrectedFrenet;
        myForceApproxC1 = false;

        myCurIndexOfSectionEdge = 1;
    }

    //=================================================================================================

    /// <summary>
    /// The pipe of <paramref name="Profile"/> along <paramref name="Spine"/>, with the trihedron
    /// <paramref name="aMode"/> (Frenet, corrected Frenet or discrete; any other mode is corrected Frenet), and an
    /// attempt at C1 surfaces where a swept surface is C0 with <paramref name="ForceApproxC1"/>.
    /// </summary>
    public BRepFill_Pipe(in TopoDS_Wire Spine,
                         in TopoDS_Shape Profile,
                         GeomFill_Trihedron aMode = GeomFill_IsCorrectedFrenet,
                         bool ForceApproxC1 = false,
                         bool KPart = false)

    {
        myDegmax = 11;
        mySegmax = 100;

        myMode = GeomFill_IsCorrectedFrenet;
        if (aMode == GeomFill_IsFrenet || aMode == GeomFill_IsCorrectedFrenet
            || aMode == GeomFill_IsDiscreteTrihedron)
        {
            myMode = aMode;
        }

        myContinuity = GeomAbs_C2;
        if (myMode == GeomFill_IsDiscreteTrihedron)
        {
            myContinuity = GeomAbs_C0;
        }

        myForceApproxC1 = ForceApproxC1;

        myCurIndexOfSectionEdge = 1;

        Perform(Spine, Profile, KPart);
    }

    //=================================================================================================

    /// <summary>Builds the pipe of <paramref name="Profile"/> along <paramref name="Spine"/>.</summary>
    /// <exception cref="Standard_DomainError">The profile contains solids.</exception>
    public void Perform(in TopoDS_Wire Spine,
                        in TopoDS_Shape Profile,
                        bool KPart = false)

    {
        _ = KPart;
        mySections = null;
        myFaces = null;
        myEdges = null;

        mySpine = Spine;
        myProfile = Profile;

        DefineRealSegmax();

        TopoDS_Shape TheProf;

        GeomFill_TrihedronLaw? TLaw = null;
        switch (myMode)
        {
            case GeomFill_IsFrenet:
                TLaw = new GeomFill_Frenet();
                break;
            case GeomFill_IsCorrectedFrenet:
                TLaw = new GeomFill_CorrectedFrenet();
                break;
            case GeomFill_IsDiscreteTrihedron:
                TLaw = new GeomFill_DiscreteTrihedron();
                break;
            default:
                break;
        }
        GeomFill_CurveAndTrihedron Loc = new(TLaw!);
        myLoc = new BRepFill_Edge3DLaw(mySpine, Loc);
        if (myLoc.NbLaw() == 0)
        {
            return; // Degenerated case
        }
        myLoc.TransformInG0Law(); // Set into continuity

        BRepFill_SectionPlacement Place = new(myLoc, Profile);
        myTrsf = Place.Transformation();

        TopLoc_Location Loc2 = new(myTrsf), Loc1;
        Loc1 = Profile.Location();
        TheProf = myProfile;
        TheProf.Location(Loc2.Multiplied(Loc1));

        // Construct First && Last Shape
        gp_Mat M = new();
        gp_Vec V = new();
        gp_Trsf fila = new();
        myLoc.Law(1).GetDomain(out double first, out double last);
        myLoc.Law(1).D0(first, ref M, ref V);
        fila.SetValues(M[1, 1],
                       M[1, 2],
                       M[1, 3],
                       V.X(),
                       M[2, 1],
                       M[2, 2],
                       M[2, 3],
                       V.Y(),
                       M[3, 1],
                       M[3, 2],
                       M[3, 3],
                       V.Z());

        fila.Multiply(myTrsf);
        TopLoc_Location LocFirst = new(fila);
        myFirst = myProfile;
        if (!LocFirst.IsIdentity())
        {
            // myFirst.Location( LocFirst.Multiplied(myProfile.Location()) );
            myFirst = new BRepBuilderAPI_Transform(myProfile, fila, true).InternalShape(); // copy
        }

        ShapeUpgrade_RemoveLocations RemLoc = new();
        RemLoc.SetRemoveLevel(TopAbs_COMPOUND);
        RemLoc.Remove(myFirst);
        myFirst = RemLoc.InternalGetResult();

        myLoc.Law(myLoc.NbLaw()).GetDomain(out first, out last);
        myLoc.Law(myLoc.NbLaw()).D0(last, ref M, ref V);
        //    try { // Not good, but there are no other means to test SetValues
        fila.SetValues(M[1, 1],
                       M[1, 2],
                       M[1, 3],
                       V.X(),
                       M[2, 1],
                       M[2, 2],
                       M[2, 3],
                       V.Y(),
                       M[3, 1],
                       M[3, 2],
                       M[3, 3],
                       V.Z());
        fila.Multiply(myTrsf);
        TopLoc_Location LocLast = new(fila);
        if (!myLoc.IsClosed() || LocFirst != LocLast)
        {
            myLast = myProfile;
            if (!LocLast.IsIdentity())
            {
                // myLast.Location(LocLast.Multiplied(myProfile.Location()) );
                myLast = new BRepBuilderAPI_Transform(myProfile, fila, true).InternalShape(); // copy
            }
        }
        else
        {
            myLast = myFirst;
        }

        RemLoc.Remove(myLast);
        myLast = RemLoc.InternalGetResult();

        myShape = MakeShape(TheProf, myProfile, myFirst, myLast);
    }

    //=================================================================================================

    /// <summary>The spine (frozen).</summary>
    public TopoDS_Shape Spine() => Published(mySpine);

    /// <summary>The profile (frozen).</summary>
    public TopoDS_Shape Profile() => Published(myProfile);

    /// <summary>The pipe (frozen).</summary>
    public TopoDS_Shape Shape() => Published(myShape);

    /// <summary><see cref="Shape"/> without freezing, for ported callers.</summary>
    internal TopoDS_Shape InternalShape() => myShape;

    /// <summary>The approximation error of the swept surfaces.</summary>
    public double ErrorOnSurface() => myErrorOnSurf;

    /// <summary>The shape at the start of the pipe (frozen).</summary>
    public TopoDS_Shape FirstShape() => Published(myFirst);

    /// <summary><see cref="FirstShape"/> without freezing, for ported callers.</summary>
    internal TopoDS_Shape InternalFirstShape() => myFirst;

    /// <summary>The shape at the end of the pipe (frozen).</summary>
    public TopoDS_Shape LastShape() => Published(myLast);

    /// <summary><see cref="LastShape"/> without freezing, for ported callers.</summary>
    internal TopoDS_Shape InternalLastShape() => myLast;

    private static TopoDS_Shape Published(TopoDS_Shape theShape)
    {
        theShape.TShape()?.FreezeGraph();
        return theShape;
    }

    //=================================================================================================

    /// <summary>
    /// Fills <paramref name="theList"/> with the shapes generated from <paramref name="theShape"/>, a sub-shape of
    /// the profile or the spine (frozen).
    /// </summary>
    public void Generated(in TopoDS_Shape theShape, NCollection_List<TopoDS_Shape> theList)
    {
        InternalGenerated(theShape, theList);
        for (NCollection_List<TopoDS_Shape>.Iterator anIt = new(theList); anIt.More(); anIt.Next())
        {
            anIt.Value().TShape()?.FreezeGraph();
        }
    }

    /// <summary><see cref="Generated"/> without freezing, for ported callers.</summary>
    internal void InternalGenerated(in TopoDS_Shape theShape, NCollection_List<TopoDS_Shape> theList)
    {
        theList.Clear();

        if (myGenMap.IsBound(theShape))
        {
            theList.Assign(myGenMap.Find(theShape));
        }
    }

    //=================================================================================================

    /// <summary>
    /// The face generated by the edge <paramref name="ESpine"/> of the spine and the edge <paramref name="EProfile"/>
    /// of the profile (frozen; null for a degenerated profile edge).
    /// </summary>
    /// <exception cref="Standard_DomainError">An edge isn't in the spine or the profile.</exception>
    public TopoDS_Face Face(in TopoDS_Edge ESpine, in TopoDS_Edge EProfile)
    {
        TopoDS_Face aFace = InternalFace(ESpine, EProfile);
        aFace.TShape()?.FreezeGraph();
        return aFace;
    }

    /// <summary><see cref="Face"/> without freezing, for ported callers.</summary>
    internal TopoDS_Face InternalFace(in TopoDS_Edge ESpine, in TopoDS_Edge EProfile)
    {
        TopoDS_Face theFace = new();

        if (BRep_Tool.Degenerated(EProfile))
        {
            return theFace;
        }

        int ii, ispin = 0, iprof = 0, count = 0;

        // *************************************************
        // Search if EProfile is an edge of myProfile
        // *************************************************
        iprof = FindEdge(myProfile, EProfile, ref count);

        if (iprof == 0)
        {
            throw new Standard_DomainError("BRepFill_Pipe::Face : Edge not in the Profile");
        }

        // *************************************************
        // Search if ESpine  is an edge of mySpine and find
        // the index of the corresponding Filler
        // *************************************************
        for (ii = 1; ii <= myLoc!.NbLaw() && (ispin == 0); ii++)
        {
            if (ESpine.IsSame(myLoc.Edge(ii)))
            {
                ispin = ii;
            }
        }

        if (ispin == 0)
        {
            throw new Standard_DomainError("BRepFill_Pipe::Edge  : Edge not in the Spine");
        }

        theFace = TopoDS.Face(myFaces!.Value(iprof, ispin));
        return theFace;
    }

    //=================================================================================================

    /// <summary>
    /// The edge generated by the edge <paramref name="ESpine"/> of the spine and the vertex
    /// <paramref name="VProfile"/> of the profile (frozen).
    /// </summary>
    /// <exception cref="Standard_DomainError">The edge or the vertex isn't in the spine or the profile.</exception>
    public TopoDS_Edge Edge(in TopoDS_Edge ESpine, in TopoDS_Vertex VProfile)
    {
        TopoDS_Edge anEdge = InternalEdge(ESpine, VProfile);
        anEdge.TShape()?.FreezeGraph();
        return anEdge;
    }

    /// <summary><see cref="Edge"/> without freezing, for ported callers.</summary>
    internal TopoDS_Edge InternalEdge(in TopoDS_Edge ESpine, in TopoDS_Vertex VProfile)
    {
        int ii, ispin = 0, iprof = 0, count = 0;

        // *************************************************
        // Search if VProfile is a Vertex of myProfile
        // *************************************************
        iprof = FindVertex(myProfile, VProfile, ref count);
        if (iprof == 0)
        {
            throw new Standard_DomainError("BRepFill_Pipe::Edge : Vertex not in the Profile");
        }

        // *************************************************
        // Search if ESpine  is an edge of mySpine and find
        // the index of the corresponding Filler
        // *************************************************

        for (ii = 1; ii <= myLoc!.NbLaw() && (ispin == 0); ii++)
        {
            if (ESpine.IsSame(myLoc.Edge(ii)))
            {
                ispin = ii;
            }
        }

        if (ispin == 0)
        {
            throw new Standard_DomainError("BRepFill_Pipe::Edge  : Edge not in the Spine");
        }

        // *************************************************
        // Generate the corresponding Shape
        // *************************************************
        TopoDS_Edge theEdge;
        theEdge = TopoDS.Edge(myEdges!.Value(iprof, ispin));

        return theEdge;
    }

    //=================================================================================================

    /// <summary>The profile at the position of the spine's vertex <paramref name="VSpine"/>, as a compound (frozen).</summary>
    /// <exception cref="Standard_DomainError">The vertex isn't in the spine.</exception>
    public TopoDS_Shape Section(in TopoDS_Vertex VSpine) => Published(InternalSection(VSpine));

    /// <summary><see cref="Section"/> without freezing, for ported callers.</summary>
    internal TopoDS_Shape InternalSection(in TopoDS_Vertex VSpine)
    {
        int ii, ispin = 0;

        // *************************************************
        // Search if ESpine  is an edge of mySpine and find
        // the index of the corresponding Filler
        // *************************************************

        // iterate on all the edges of mySpine
        for (ii = 1; ii <= myLoc!.NbLaw() + 1 && (ispin == 0); ii++)
        {
            if (VSpine.IsSame(myLoc.Vertex(ii)))
            {
                ispin = ii;
            }
        }

        if (ispin == 0)
        {
            throw new Standard_DomainError("BRepFill_Pipe::Section  : Vertex not in the Spine");
        }

        BRep_Builder B = new();
        TopoDS_Compound Comp = new();
        B.MakeCompound(ref Comp);
        for (ii = 1; ii <= mySections!.ColLength(); ii++)
        {
            B.Add(Comp, mySections.Value(ii, ispin));
        }

        return Comp;
    }

    //=======================================================================
    // function : PipeLine
    // purpose  : Construct a wire by sweeping of a point
    //=======================================================================

    /// <summary>The wire swept by <paramref name="Point"/> (placed with the profile) along the spine (frozen).</summary>
    public TopoDS_Wire PipeLine(in gp_Pnt Point)
    {
        TopoDS_Wire aWire = InternalPipeLine(Point);
        aWire.TShape()?.FreezeGraph();
        return aWire;
    }

    /// <summary><see cref="PipeLine"/> without freezing, for ported callers.</summary>
    internal TopoDS_Wire InternalPipeLine(in gp_Pnt Point)
    {
        // Positioning
        gp_Pnt P;
        P = Point;
        P.Transform(myTrsf);

        TopoDS_Vertex VertexSection = new BRepLib_MakeVertex(P).InternalVertex();
        BRepFill_ShapeLaw Section = new(VertexSection);

        // Sweeping
        BRepFill_Sweep MkSw = new(Section, myLoc!, true);
        MkSw.SetForceApproxC1(myForceApproxC1);
        MkSw.Build(myReversedEdges,
                   myTapes,
                   myRails,
                   BRepFill_Modified,
                   myContinuity,
                   GeomFill_Location,
                   myDegmax,
                   mySegmax);
        TopoDS_Shape aLocalShape = MkSw.Shape();
        myErrorOnSurf = MkSw.ErrorOnSurface();
        BuildHistory(MkSw, VertexSection);
        return TopoDS.Wire(aLocalShape);
        // return TopoDS::Wire(MkSw.Shape());
    }

    //=================================================================================================

    private TopoDS_Shape MakeShape(in TopoDS_Shape S,
                                   in TopoDS_Shape theOriginalS,
                                   in TopoDS_Shape FirstShape,
                                   in TopoDS_Shape LastShape)
    {
        TopoDS_Shape result = new();
        BRep_Builder B = new();
        bool explode = false;
        TopoDS_Shape TheS, TheFirst, TheLast;
        int InitialLength = 0;
        TheS = S;
        TheFirst = FirstShape;
        TheLast = LastShape;
        if (myFaces is not null)
        {
            InitialLength = myFaces.ColLength();
        }

        TopLoc_Location BackLoc = new(myTrsf.Inverted());
        _ = BackLoc;

        // there are two kinds of generation
        //  1. generate with S from each Filler (Vertex, Edge)
        //  2. call MakeShape recursively on the subshapes of S
        //
        // explode is True in the second case

        // create the result empty

        switch (S.ShapeType())
        {

            case TopAbs_VERTEX:
            {
                TopoDS_Wire aW = new();
                B.MakeWire(ref aW);
                result = aW;
                break;
            }

            case TopAbs_EDGE:
            {
                TopoDS_Wire W = new();
                TopoDS_Shell aSh = new();
                B.MakeShell(ref aSh);
                result = aSh;
                B.MakeWire(ref W);
                B.Add(W, S);
                W.Closed(BRep_Tool.IsClosed(S));
                TheS = W;
                if (!FirstShape.IsNull())
                {
                    B.MakeWire(ref W);
                    B.Add(W, FirstShape);
                    W.Closed(BRep_Tool.IsClosed(FirstShape));
                    TheFirst = W;
                }
                if (!LastShape.IsNull())
                {
                    B.MakeWire(ref W);
                    B.Add(W, LastShape);
                    W.Closed(BRep_Tool.IsClosed(LastShape));
                    TheLast = W;
                }
                result.Closed(BRep_Tool.IsClosed(result));
                break;
            }

            case TopAbs_WIRE:
            {
                TopoDS_Shell aSh = new();
                B.MakeShell(ref aSh);
                result = aSh;
                break;
            }

            case TopAbs_FACE:
            {
                TopoDS_Shell aSh = new();
                B.MakeShell(ref aSh);
                result = aSh;
                explode = true;
                if (!mySpine.Closed() && !TheFirst.IsNull())
                {
                    B.Add(result, TheFirst.Reversed());
                }
                result.Closed(BRep_Tool.IsClosed(result));
                break;
            }

            case TopAbs_SHELL:
            {
                TopoDS_CompSolid aCS = new();
                B.MakeCompSolid(ref aCS);
                result = aCS;
                explode = true;
                break;
            }

            case TopAbs_SOLID:
            case TopAbs_COMPSOLID:
                throw new Standard_DomainError("BRepFill_Pipe::profile contains solids");

            case TopAbs_COMPOUND:
            {
                TopoDS_Compound aC = new();
                B.MakeCompound(ref aC);
                result = aC;
                explode = true;
                break;
            }
            default:
                break;
        }

        if (explode)
        {
            // add the subshapes
            TopoDS_Iterator itFirst = new(), itLast = new();
            TopoDS_Shape first = new(), last = new();
            if (!TheFirst.IsNull())
            {
                itFirst.Initialize(TheFirst);
            }
            if (!TheLast.IsNull())
            {
                itLast.Initialize(TheLast);
            }

            TopoDS_Iterator it = new(S);
            TopoDS_Iterator itorig = new(theOriginalS);
            for (; it.More(); it.Next(), itorig.Next())
            {
                if (!TheFirst.IsNull())
                {
                    first = itFirst.Value();
                }
                if (!TheLast.IsNull())
                {
                    last = itLast.Value();
                }
                if (TheS.ShapeType() == TopAbs_FACE)
                {
                    MakeShape(it.Value(), itorig.Value(), first, last);
                }
                else
                {
                    B.Add(result, MakeShape(it.Value(), itorig.Value(), first, last));
                }

                if (!TheFirst.IsNull())
                {
                    itFirst.Next();
                }
                if (!TheLast.IsNull())
                {
                    itLast.Next();
                }
            }
        }

        else
        {
            if (TheS.ShapeType() == TopAbs_VERTEX)
            {
                BRepFill_ShapeLaw Section = new(TopoDS.Vertex(TheS));
                BRepFill_Sweep MkSw = new(Section, myLoc!, true);
                MkSw.SetForceApproxC1(myForceApproxC1);
                MkSw.Build(myReversedEdges,
                           myTapes,
                           myRails,
                           BRepFill_Modified,
                           myContinuity,
                           GeomFill_Location,
                           myDegmax,
                           mySegmax);
                result = MkSw.Shape();
                UpdateMap(theOriginalS, result, myGenMap);
                myErrorOnSurf = MkSw.ErrorOnSurface();

                NCollection_HArray2<TopoDS_Shape>? aSections = MkSw.Sections();

                if (aSections is not null)
                {
                    int aVLast = aSections.UpperCol();

                    myFirst = aSections.Value(1, 1);
                    myLast = aSections.Value(1, aVLast);
                }

                BuildHistory(MkSw, theOriginalS);
            }

            if (TheS.ShapeType() == TopAbs_WIRE)
            {
                BRepFill_ShapeLaw Section = new(TopoDS.Wire(TheS));
                BRepFill_Sweep MkSw = new(Section, myLoc!, true);
                MkSw.SetBounds(TopoDS.Wire(TheFirst), TopoDS.Wire(TheLast));
                MkSw.SetForceApproxC1(myForceApproxC1);
                MkSw.Build(myReversedEdges,
                           myTapes,
                           myRails,
                           BRepFill_Modified,
                           myContinuity,
                           GeomFill_Location,
                           myDegmax,
                           mySegmax);
                result = MkSw.Shape();
                myErrorOnSurf = MkSw.ErrorOnSurface();

                // Labeling of elements
                if (mySections is null)
                {
                    myFaces = MkSw.SubShape();
                    mySections = MkSw.Sections();
                    myEdges = MkSw.InterFaces();
                }
                else
                {
                    NCollection_HArray2<TopoDS_Shape> Aux, Somme;
                    int length;
                    int ii, jj, kk;

                    Aux = MkSw.SubShape()!;
                    length = Aux.ColLength() + myFaces!.ColLength();
                    Somme = new NCollection_HArray2<TopoDS_Shape>(1, length, 1, Aux.RowLength());
                    for (jj = 1; jj <= myFaces.RowLength(); jj++)
                    {
                        for (ii = 1; ii <= myFaces.ColLength(); ii++)
                        {
                            Somme.SetValue(ii, jj, myFaces.Value(ii, jj));
                        }

                        for (kk = 1, ii = myFaces.ColLength() + 1; kk <= Aux.ColLength(); kk++, ii++)
                        {
                            Somme.SetValue(ii, jj, Aux.Value(kk, jj));
                        }
                    }
                    myFaces = Somme;

                    Aux = MkSw.Sections()!;
                    length = Aux.ColLength() + mySections.ColLength();
                    Somme = new NCollection_HArray2<TopoDS_Shape>(1, length, 1, Aux.RowLength());
                    for (jj = 1; jj <= mySections.RowLength(); jj++)
                    {
                        for (ii = 1; ii <= mySections.ColLength(); ii++)
                        {
                            Somme.SetValue(ii, jj, mySections.Value(ii, jj));
                        }

                        myCurIndexOfSectionEdge = mySections.ColLength() + 1;

                        for (kk = 1, ii = mySections.ColLength() + 1; kk <= Aux.ColLength(); kk++, ii++)
                        {
                            Somme.SetValue(ii, jj, Aux.Value(kk, jj));
                        }
                    }
                    mySections = Somme;

                    Aux = MkSw.InterFaces()!;
                    length = Aux.ColLength() + myEdges!.ColLength();
                    Somme = new NCollection_HArray2<TopoDS_Shape>(1, length, 1, Aux.RowLength());
                    for (jj = 1; jj <= myEdges.RowLength(); jj++)
                    {
                        for (ii = 1; ii <= myEdges.ColLength(); ii++)
                        {
                            Somme.SetValue(ii, jj, myEdges.Value(ii, jj));
                        }

                        for (kk = 1, ii = myEdges.ColLength() + 1; kk <= Aux.ColLength(); kk++, ii++)
                        {
                            Somme.SetValue(ii, jj, Aux.Value(kk, jj));
                        }
                    }

                    myEdges = Somme;
                }

                BuildHistory(MkSw, theOriginalS);
            }
        }

        if (TheS.ShapeType() == TopAbs_FACE)
        {
            int ii, jj;
            // jgv
            TopExp_Explorer Explo = new(result, TopAbs_FACE);
            for (; Explo.More(); Explo.Next())
            {
                TopoDS_Shape aFace = Explo.Current();
                RebuildTopOrBottomFace(aFace.Reversed(), true); // top face was reversed
            }
            /////
            TopoDS_Face F;
            for (ii = InitialLength + 1; ii <= myFaces!.ColLength(); ii++)
            {
                for (jj = 1; jj <= myFaces.RowLength(); jj++)
                {
                    if (myFaces.Value(ii, jj).ShapeType() == TopAbs_FACE)
                    {
                        F = TopoDS.Face(myFaces.Value(ii, jj));
                        if (!F.IsNull())
                        {
                            B.Add(result, F);
                        }
                    }
                }
            }

            if (!mySpine.Closed())
            {
                // if Spine is not closed
                // add the last face of the solid

                // jgv
                RebuildTopOrBottomFace(TheLast, false); // bottom face
                /////
                B.Add(result, TopoDS.Face(TheLast));
            }

            TopoDS_Solid solid = new();
            BRep_Builder BS = new();
            BS.MakeSolid(ref solid);

            result.Closed(true);
            BS.Add(solid, TopoDS.Shell(result));

            BRepClass3d_SolidClassifier SC = new(solid);
            SC.PerformInfinitePoint(Precision.Confusion());
            if (SC.State() == TopAbs_IN)
            {
                BS.MakeSolid(ref solid);
                TopoDS_Shape aLocalShape = result.Reversed();
                BS.Add(solid, TopoDS.Shell(aLocalShape));
                //      BS.Add(solid,TopoDS::Shell(result.Reversed()));
            }
            UpdateMap(theOriginalS, solid, myGenMap);
            return solid;
        }
        else
        {
            return result;
        }
    }

    //============================================================================
    // function : FindEdge
    // purpose  : Find the number of edge corresponding to the edge of the profile.
    //============================================================================

    private int FindEdge(in TopoDS_Shape S, in TopoDS_Edge E, ref int InitialLength)
    {
        int result = 0;

        switch (S.ShapeType())
        {

            case TopAbs_EDGE:
            {
                InitialLength++;
                if (S.IsSame(E))
                {
                    result = InitialLength;
                }
                break;
            }

            case TopAbs_WIRE:
            {
                BRepFill_ShapeLaw Section = new(TopoDS.Wire(S), false);
                int NbLaw = Section.NbLaw();

                for (int ii = 1; (ii <= NbLaw) && (result == 0); ii++)
                {
                    if (E.IsSame(Section.Edge(ii)))
                    {
                        result = InitialLength + ii;
                    }
                }
                InitialLength += NbLaw;
                break;
            }

            case TopAbs_FACE:
            case TopAbs_SHELL:
            case TopAbs_COMPOUND:
            {
                for (TopoDS_Iterator it = new(S); it.More() && (result == 0); it.Next())
                {
                    result = FindEdge(it.Value(), E, ref InitialLength);
                }
                break;
            }

            case TopAbs_SOLID:
            case TopAbs_COMPSOLID:
                throw new Standard_DomainError("BRepFill_Pipe::SOLID or COMPSOLID");
            default:
                break;
        }

        return result;
    }

    //=======================================================================
    // function : FindVertex
    // purpose  : Find the number of edge corresponding to an edge of the profile.
    //=======================================================================

    private int FindVertex(in TopoDS_Shape S, in TopoDS_Vertex V, ref int InitialLength)
    {
        int result = 0;

        switch (S.ShapeType())
        {
            case TopAbs_VERTEX:
            {
                InitialLength++;
                if (S.IsSame(V))
                {
                    result = InitialLength;
                }
                break;
            }

            case TopAbs_EDGE:
            {
                TopExp.Vertices(TopoDS.Edge(S), out TopoDS_Vertex VF, out TopoDS_Vertex VL);
                if (S.Orientation() == TopAbs_REVERSED)
                {
                    TopoDS_Vertex aux;
                    aux = VF;
                    VF = VL;
                    VL = aux;
                }
                if (VF.IsSame(V))
                {
                    result = InitialLength + 1;
                }
                else if (VL.IsSame(V))
                {
                    result = InitialLength + 2;
                }
                InitialLength += 2;
                break;
            }

            case TopAbs_WIRE:
            {
                int ii = InitialLength + 1;
                BRepFill_ShapeLaw Section = new(TopoDS.Wire(S), false);
                InitialLength += Section.NbLaw() + 1;

                for (; (ii <= InitialLength) && (result == 0); ii++)
                {
                    if (V.IsSame(Section.Vertex(ii, 0.0)))
                    {
                        result = ii;
                    }
                }
                break;
            }

            case TopAbs_FACE:
            case TopAbs_SHELL:
            case TopAbs_COMPOUND:
            {
                for (TopoDS_Iterator it = new(S); it.More() && (result == 0); it.Next())
                {
                    result = FindVertex(it.Value(), V, ref InitialLength);
                }
                break;
            }

            case TopAbs_SOLID:
            case TopAbs_COMPSOLID:
                throw new Standard_DomainError("BRepFill_Pipe::SOLID or COMPSOLID");
            default:
                break;
        }

        return result;
    }

    //=======================================================================
    // function : DefineRealSegmax
    // purpose  : Defines the real number of segments
    //           required in the case of bspline spine
    //=======================================================================

    private void DefineRealSegmax()
    {
        int RealSegmax = 0;

        TopoDS_Iterator iter = new(mySpine);
        for (; iter.More(); iter.Next())
        {
            TopoDS_Edge E = TopoDS.Edge(iter.Value());
            Geom_Curve? C = BRep_Tool.Curve(E, out double first, out double last);
            if (C is null)
            {
                continue;
            }
            while (C.GetType() == typeof(Geom_TrimmedCurve)
                   || C.GetType() == typeof(Geom_OffsetCurve))
            {
                if (C.GetType() == typeof(Geom_TrimmedCurve))
                {
                    C = ((Geom_TrimmedCurve)C).BasisCurve();
                }
                if (C.GetType() == typeof(Geom_OffsetCurve))
                {
                    C = ((Geom_OffsetCurve)C).BasisCurve();
                }
            }
            if (C.GetType() == typeof(Geom_BSplineCurve))
            {
                Geom_BSplineCurve BC = (Geom_BSplineCurve)C;
                int NbKnots = BC.NbKnots();
                int RealNbKnots = NbKnots;
                if (first > BC.FirstParameter())
                {
                    BC.LocateU(first, Precision.PConfusion(), out int I1, out _);
                    RealNbKnots -= I1 - 1;
                }
                if (last < BC.LastParameter())
                {
                    BC.LocateU(last, Precision.PConfusion(), out _, out int I2);
                    RealNbKnots -= NbKnots - I2;
                }
                RealSegmax += RealNbKnots - 1;
            }
        }

        if (mySegmax < RealSegmax)
        {
            mySegmax = RealSegmax;
        }
    }

    //=======================================================================
    // function : RebuildTopOrBottomFace
    // purpose  : Correct orientation of v-iso edges
    //           according to new 3d and 2d curves taken from swept surfaces
    //=======================================================================

    private void RebuildTopOrBottomFace(in TopoDS_Shape aFace, bool IsTop)
    {
        int IndexOfSection = (IsTop) ? 1 : mySections!.RowLength();

        int ii;
        BRep_Builder BB = new();
        TopoDS_Iterator itf = new(aFace);
        for (; itf.More(); itf.Next())
        {
            TopoDS_Shape aWire = itf.Value();
            NCollection_Sequence<TopoDS_Shape> InitEdges = new();
            NCollection_Sequence<TopoDS_Shape> ResEdges = new();
            TopoDS_Iterator itw = new(aWire);
            for (; itw.More(); itw.Next())
            {
                TopoDS_Shape anEdge = itw.Value();
                for (ii = myCurIndexOfSectionEdge; ii <= mySections!.ColLength(); ii++)
                {
                    TopoDS_Shape aVisoEdge = mySections.Value(ii, IndexOfSection);
                    if (anEdge.IsSame(aVisoEdge))
                    {
                        InitEdges.Append(anEdge);
                        ResEdges.Append(aVisoEdge);
                        break;
                    }
                }
            }
            // OCCT.NET: the writes below reach the profile's own face when the pipe shares it (decision 910)
            aWire.Free(true);
            for (ii = 1; ii <= InitEdges.Length(); ii++)
            {
                BB.Remove(aWire, InitEdges[ii]);
                UpdateTolFromTopOrBottomPCurve(TopoDS.Face(aFace), TopoDS.Edge(ResEdges[ii]));
                BB.Add(aWire, ResEdges[ii]);
            }
        }
    }

    //=======================================================================
    // function : BuildHistory
    // purpose  : Builds history for edges and vertices
    //           of section and path
    //=======================================================================
    private void BuildHistory(BRepFill_Sweep theSweep, in TopoDS_Shape theSection)
    {
        // Filling of <myGenMap>
        NCollection_HArray2<TopoDS_Shape> anUEdges = theSweep.InterFaces()!;

        int inde;

        if (theSection.ShapeType() == TopAbs_WIRE)
        {
            TopoDS_Wire aSection = TopoDS.Wire(theSection);
            BRepTools_WireExplorer wexp_sec = new(aSection);
            for (inde = 0; wexp_sec.More(); wexp_sec.Next())
            {
                inde++;
                TopoDS_Edge anEdge = TopoDS.Edge(wexp_sec.Current());
                if (BRep_Tool.Degenerated(anEdge))
                {
                    continue;
                }
                if (myGenMap.IsBound(anEdge))
                {
                    continue;
                }

                TopoDS_Vertex[] aVertex = new TopoDS_Vertex[2];
                TopExp.Vertices(anEdge, out aVertex[0], out aVertex[1]);

                // For an edge generated shape is a "tape" -
                // a shell usually containing this edge and
                // passing from beginning of path to its end
                TopoDS_Shell aTape = TopoDS.Shell(theSweep.Tape(inde));

                // Processing of vertices of <anEdge>
                // We should choose right index in <anUEdges>
                // for each vertex of edge
                int[] UIndex = new int[2];
                UIndex[0] = inde;
                UIndex[1] = inde + 1;

                if (anEdge.Orientation() == TopAbs_REVERSED)
                {
                    int Tmp = UIndex[0];
                    UIndex[0] = UIndex[1];
                    UIndex[1] = Tmp;
                }

                for (int kk = 0; kk < 2; kk++)
                {
                    if (myGenMap.IsBound(aVertex[kk]))
                    {
                        continue;
                    }

                    // Assemble the list of edges ("rail" along the path)
                    NCollection_List<TopoDS_Shape> Elist = myGenMap.Bound(aVertex[kk], new NCollection_List<TopoDS_Shape>());
                    int jj;
                    for (jj = 1; jj <= anUEdges.UpperCol(); jj++)
                    {
                        TopoDS_Shape anUedge = anUEdges.Value(UIndex[kk], jj);
                        if (!anUedge.IsNull())
                        {
                            Elist.Append(anUedge);
                        }
                    }
                } // for (int kk = 0; kk < 2; kk++)

                NCollection_List<TopoDS_Shape> Flist = myGenMap.Bound(anEdge, new NCollection_List<TopoDS_Shape>());
                TopoDS_Iterator itsh = new(aTape);
                for (; itsh.More(); itsh.Next())
                {
                    Flist.Append(itsh.Value());
                }
            } // for (inde = 0; wexp_sec.More(); wexp_sec.Next())
        } // if (theSection.ShapeType() == TopAbs_WIRE)

        // For subshapes of spine
        NCollection_HArray2<TopoDS_Shape> aFaces = theSweep.SubShape()!;
        NCollection_HArray2<TopoDS_Shape> aVEdges = theSweep.Sections()!;

        BRepTools_WireExplorer wexp = new(mySpine);
        inde = 0;
        bool ToExit = false;
        for (;;)
        {
            if (!wexp.More())
            {
                ToExit = true;
            }

            inde++;

            if (!ToExit)
            {
                TopoDS_Edge anEdgeOfSpine = wexp.Current();
                for (int i = 1; i <= aFaces.UpperRow(); i++)
                {
                    TopoDS_Shape aFace = aFaces.Value(i, inde);
                    UpdateMap(anEdgeOfSpine, aFace, myGenMap);
                }
            }

            TopoDS_Vertex aVertexOfSpine = wexp.CurrentVertex();
            for (int i = 1; i <= aVEdges.UpperRow(); i++)
            {
                TopoDS_Shape aVedge = aVEdges.Value(i, inde);
                UpdateMap(aVertexOfSpine, aVedge, myGenMap);
            }

            if (ToExit)
            {
                break;
            }

            if (wexp.More())
            {
                wexp.Next();
            }
        }
    }
}
