// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_MakeOffset.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_MakeOffset.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Occt.BRepBuilderAPI_FaceError;
using static Occt.Extrema_ExtFlag;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.JoinKind;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// Offsets the wires of a planar face, or planar wires, in their plane (the 2D wire offset): one
/// <see cref="BRepFill_OffsetWire"/> per domain the closed wires bound.
/// </summary>
/// <remarks>
/// <para>A positive offset goes outside the closed wires, a negative one inside. The join type is GeomAbs_Arc (arcs
/// of circles around salient vertices) or GeomAbs_Intersection (the parallels prolonged to their intersection). With
/// IsOpenResult an open wire gives an open parallel on each side instead of a closed one around it.</para>
/// <para>A maker (decisions 414 and 454): <see cref="BRepBuilderAPI_MakeShape.Shape"/> and
/// <see cref="BRepBuilderAPI_MakeShape.Generated"/> hand out frozen shapes; <see cref="ConvertFace"/> returns a frozen
/// face. The algorithm never writes into the caller's face or wires.</para>
/// </remarks>
public class BRepOffsetAPI_MakeOffset : BRepBuilderAPI_MakeShape
{
    private bool myIsInitialized;
    private bool myLastIsLeft;
    private JoinKind myJoin;
    private bool myIsOpenResult;
    private bool myIsToApprox;
    private TopoDS_Face myFace = new();
    private readonly NCollection_List<TopoDS_Shape> myWires = new();
    private readonly NCollection_List<BRepFill_OffsetWire> myLeft = new();
    private readonly NCollection_List<BRepFill_OffsetWire> myRight = new();

    private static bool NeedsConvertion(in TopoDS_Wire theWire)
    {
        TopoDS_Iterator anIter = new(theWire);
        for (; anIter.More(); anIter.Next())
        {
            TopoDS_Edge anEdge = TopoDS.Edge(anIter.Value());
            BRepAdaptor_Curve aBAcurve = new(anEdge);
            CurveKind aType = aBAcurve.GetType();
            if (aType != Line && aType != Circle)
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>
    /// Converts each wire of the face into a contour of arcs of circles and segments only (with new 3D curves); wires
    /// already made of lines and circles are kept. The result is frozen.
    /// </summary>
    public static TopoDS_Face ConvertFace(in TopoDS_Face theFace, double theAngleTolerance)
    {
        TopoDS_Face aNewFace = InternalConvertFace(theFace, theAngleTolerance);
        aNewFace.TShape()?.FreezeGraph();
        return aNewFace;
    }

    /// <summary><see cref="ConvertFace"/> without freezing, for <see cref="Perform"/>.</summary>
    internal static TopoDS_Face InternalConvertFace(in TopoDS_Face theFace, double theAngleTolerance)
    {
        TopAbs_Orientation anOr = theFace.Orientation();
        TopoDS_Face aFace = theFace;
        aFace.Orientation(TopAbs_FORWARD);
        TopoDS_Face aNewFace = TopoDS.Face(aFace.EmptyCopied());
        BRep_Builder aBB = new();
        TopoDS_Iterator anIter = new(aFace);
        for (; anIter.More(); anIter.Next())
        {
            TopoDS_Wire aWire = TopoDS.Wire(anIter.Value());
            if (NeedsConvertion(aWire))
            {
                TopAbs_Orientation anOrOfWire = aWire.Orientation();
                aWire.Orientation(TopAbs_FORWARD);
                // OCCT.NET: the converted wire is new; BuildCurves3d writes into it (decision 915)
                aWire = BRepAlgo.InternalConvertWire(aWire, theAngleTolerance, aFace);
                BRepLib.BuildCurves3d(aWire);
                aWire.Orientation(anOrOfWire);
            }
            aBB.Add(aNewFace, aWire);
        }
        aNewFace.Orientation(anOr);
        return aNewFace;
    }

    /// <summary>An empty offset.</summary>
    public BRepOffsetAPI_MakeOffset()
    {
        myIsInitialized = false;
        myJoin = Arc;
        myIsOpenResult = false;
        myIsToApprox = false;
    }

    /// <summary>The offset of the wires of the planar face <paramref name="Spine"/>.</summary>
    public BRepOffsetAPI_MakeOffset(in TopoDS_Face Spine, JoinKind Join = Arc, bool IsOpenResult = false)
    {
        Init(Spine, Join, IsOpenResult);
    }

    /// <summary>
    /// Initializes the algorithm to construct parallels to the wires of <paramref name="Spine"/>. Join defines the
    /// parallels around the salient vertices: arcs of circles (GeomAbs_Arc) or the adjacent parallels prolonged to
    /// their intersection (GeomAbs_Intersection).
    /// </summary>
    public void Init(in TopoDS_Face Spine, JoinKind Join = Arc, bool IsOpenResult = false)
    {
        myFace = Spine;
        myIsInitialized = true;
        myJoin = Join;
        myIsOpenResult = IsOpenResult;
        myIsToApprox = false;
        TopExp_Explorer exp = new();
        for (exp.Init(myFace, TopAbs_WIRE); exp.More(); exp.Next())
        {
            myWires.Append(exp.Current());
        }
    }

    /// <summary>The offset of the planar wire <paramref name="Spine"/>.</summary>
    public BRepOffsetAPI_MakeOffset(in TopoDS_Wire Spine, JoinKind Join = Arc, bool IsOpenResult = false)
    {
        myWires.Append(Spine);
        myIsInitialized = true;
        myJoin = Join;
        myIsOpenResult = IsOpenResult;
        myIsToApprox = false;
    }

    /// <summary>Sets the join type and the open-result flag.</summary>
    public void Init(JoinKind Join = Arc, bool IsOpenResult = false)
    {
        myJoin = Join;
        myIsOpenResult = IsOpenResult;
    }

    /// <summary>
    /// Sets the approximation flag: the input contours are first converted into contours of 2D arcs of circles and
    /// segments.
    /// </summary>
    public void SetApprox(bool ToApprox)
    {
        myIsToApprox = ToApprox;
    }

    /// <summary>Adds a wire to offset.</summary>
    public void AddWire(in TopoDS_Wire Spine)
    {
        myIsInitialized = true;
        myWires.Append(Spine);
    }

    private static void BuildDomains(ref TopoDS_Face myFace,
                                     NCollection_List<TopoDS_Shape> WorkWires,
                                     NCollection_List<BRepFill_OffsetWire> myAlgos,
                                     JoinKind myJoin,
                                     bool myIsOpenResult,
                                     bool isPositive,
                                     ref bool isWasReversed)
    {
        BRepAlgo_FaceRestrictor FR = new();
        TopoDS_Vertex VF, VL;
        NCollection_List<TopoDS_Shape> LOW = new();
        BRep_Builder B = new();

        if (myFace.IsNull())
        {
            myFace = new BRepBuilderAPI_MakeFace(TopoDS.Wire(WorkWires.First()), true).Face();
            if (myFace.IsNull())
            {
                throw new NotDoneException("BRepOffsetAPI_MakeOffset : the wire is not planar");
            }
        }
        //  Modified by Sergey KHROMOV - Thu Apr 26 16:04:43 2001 Begin
        TopExp_Explorer anExp = new(myFace, TopAbs_WIRE);
        TopoDS_Shape aWire1 = WorkWires.First();
        TopoDS_Shape aWire2;
        if (anExp.More())
        {
            aWire2 = anExp.Current();
            if ((aWire1.Orientation() == aWire2.Orientation() && isPositive)
                || (aWire1.Orientation() == TopAbs.Complement(aWire2.Orientation()) && !isPositive))
            {
                NCollection_List<TopoDS_Shape> LWires = new();
                NCollection_List<TopoDS_Shape>.Iterator itl1 = new();
                for (itl1.Initialize(WorkWires); itl1.More(); itl1.Next())
                {
                    TopoDS_Shape W = itl1.Value();
                    LWires.Append(W.Reversed());
                }
                isWasReversed = true;
                // the caller's myWires, reversed (OCCT assigns the list through the reference)
                WorkWires.Assign(LWires);
            }
        }
        //  Modified by Sergey KHROMOV - Thu Apr 26 16:04:44 2001 End

        FR.Init(myFace, true);
        //====================================================
        // Construction of faces limited by closed wires.
        //====================================================
        NCollection_List<TopoDS_Shape>.Iterator itl = new(WorkWires);
        for (; itl.More(); itl.Next())
        {
            TopoDS_Wire W = TopoDS.Wire(itl.Value());
            if (W.Closed())
            {
                FR.Add(W);
                continue;
            }
            TopExp.Vertices(W, out VF, out VL);
            if (VF.IsSame(VL))
            {
                FR.Add(W);
            }
            else
            {
                LOW.Append(W);
            }
        }
        // OCCT.NET: the restrictor copies frozen wires at the entry and gives the caller's back; its faces stay
        // unfrozen, as the open wires are added to them (decision 914)
        FR.Perform();
        if (!FR.IsDone())
        {
            throw new NotDoneException("BRepOffsetAPI_MakeOffset : Build Domains");
        }
        NCollection_List<TopoDS_Shape> Faces = new();
        for (; FR.More(); FR.Next())
        {
            Faces.Append(FR.InternalCurrent());
        }

        //===========================================
        // No closed wire => only one domain
        //===========================================
        if (Faces.IsEmpty())
        {
            TopoDS_Shape aLocalShape = myFace.EmptyCopied();
            TopoDS_Face F = TopoDS.Face(aLocalShape);
            NCollection_List<TopoDS_Shape>.Iterator itW = new(LOW);
            for (; itW.More(); itW.Next())
            {
                B.Add(F, itW.Value());
            }
            BRepFill_OffsetWire Algo = new(F, myJoin, myIsOpenResult);
            myAlgos.Append(Algo);
            return;
        }

        //====================================================
        // Classification of open wires.
        //====================================================
        NCollection_List<TopoDS_Shape>.Iterator itF = new();
        for (itF.Initialize(Faces); itF.More(); itF.Next())
        {
            TopoDS_Face F = TopoDS.Face(itF.Value());
            BRepAdaptor_Surface S = new(F, false);
            double Tol = BRep_Tool.Tolerance(F);

            BRepTopAdaptor_FClass2d CL = new(F, global::Kerf.Engine.Foundation.Precision.Confusion());

            NCollection_List<TopoDS_Shape>.Iterator itW = new(LOW);
            while (itW.More())
            {
                TopoDS_Wire W = TopoDS.Wire(itW.Value());
                //=======================================================
                // Choice of a point on the wire. + projection on the face.
                //=======================================================
                TopExp_Explorer exp = new(W, TopAbs_VERTEX);
                TopoDS_Vertex V = TopoDS.Vertex(exp.Current());
                Point2D PV = new();
                Point3D P3d = BRep_Tool.Pnt(V);
                Extrema_ExtPS ExtPS = new(P3d, S, Tol, Tol, Extrema_ExtFlag_MIN);
                double Dist2Min = global::Kerf.Engine.Foundation.Precision.Infinite();
                // OCCT declares Found as a double
                bool Found = false;
                for (int ie = 1; ie <= ExtPS.NbExt(); ie++)
                {
                    double X, Y;
                    if (ExtPS.SquareDistance(ie) < Dist2Min)
                    {
                        Dist2Min = ExtPS.SquareDistance(ie);
                        Found = true;
                        ExtPS.Point(ie).Parameter(out X, out Y);
                        PV.SetCoord(X, Y);
                    }
                }
                if (Found && (CL.Perform(PV) == TopAbs_IN))
                {
                    // The face that contains a wire is found and it is removed from the list
                    B.Add(F, W);
                    LOW.Remove(itW);
                }
                else
                {
                    itW.Next();
                }
            }
        }

        //========================================
        // Creation of algorithms on each domain.
        //========================================
        for (itF.Initialize(Faces); itF.More(); itF.Next())
        {
            BRepFill_OffsetWire Algo = new(TopoDS.Face(itF.Value()), myJoin, myIsOpenResult);
            myAlgos.Append(Algo);
        }
    }

    /// <summary>
    /// Computes a parallel to the spine at the distance <paramref name="Offset"/> (outside the closed wires if
    /// positive), at the altitude <paramref name="Alt"/> along the normal of the spine's plane. Failures leave the
    /// maker not done.
    /// </summary>
    /// <exception cref="NotDoneException">The maker has neither a face nor a wire.</exception>
    public void Perform(double Offset, double Alt = 0.0)
    {
        StdFail_NotDone_Raise_if(!myIsInitialized, "BRepOffsetAPI_MakeOffset : Perform without Init");

        try
        {
            if (myIsToApprox)
            {
                double aTol = 0.1;
                if (myFace.IsNull())
                {
                    TopoDS_Face aFace = new();
                    bool OnlyPlane = true;
                    NCollection_List<TopoDS_Shape>.Iterator anItl = new(myWires);
                    for (; anItl.More(); anItl.Next())
                    {
                        BRepBuilderAPI_MakeFace aFaceMaker = new(TopoDS.Wire(anItl.Value()), OnlyPlane);
                        if (aFaceMaker.Error() == BRepBuilderAPI_FaceDone)
                        {
                            aFace = aFaceMaker.Face();
                            break;
                        }
                    }
                    for (anItl.Initialize(myWires); anItl.More(); anItl.Next())
                    {
                        TopoDS_Wire aWire = TopoDS.Wire(anItl.Value());
                        if (NeedsConvertion(aWire))
                        {
                            TopoDS_Wire aNewWire = BRepAlgo.InternalConvertWire(aWire, aTol, aFace);
                            BRepLib.BuildCurves3d(aNewWire);
                            aNewWire.Orientation(aWire.Orientation());
                            anItl.ChangeValue() = aNewWire;
                        }
                    }
                }
                else
                {
                    myFace = InternalConvertFace(myFace, aTol);
                    BRepLib.BuildCurves3d(myFace);
                    myWires.Clear();
                    TopoDS_Iterator anIter = new(myFace);
                    for (; anIter.More(); anIter.Next())
                    {
                        myWires.Append(anIter.Value());
                    }
                }
            }

            int i = 1;
            NCollection_List<BRepFill_OffsetWire>.Iterator itOW = new();
            TopoDS_Compound Res = new();
            BRep_Builder B = new();
            B.MakeCompound(ref Res);
            myLastIsLeft = (Offset <= 0);

            bool isWasReversed = false;
            if (Offset <= 0.0)
            {
                if (myLeft.IsEmpty())
                {
                    //  Modified by Sergey KHROMOV - Fri Apr 27 14:35:26 2001 Begin
                    BuildDomains(ref myFace, myWires, myLeft, myJoin, myIsOpenResult, false, ref isWasReversed);
                    //  Modified by Sergey KHROMOV - Fri Apr 27 14:35:26 2001 End
                }

                for (itOW.Initialize(myLeft); itOW.More(); itOW.Next())
                {
                    BRepFill_OffsetWire Algo = itOW.Value();
                    Algo.Perform(Math.Abs(Offset), Alt);
                    if (Algo.IsDone() && !Algo.InternalShape().IsNull())
                    {
                        B.Add(Res, isWasReversed ? Algo.InternalShape().Reversed() : Algo.InternalShape());
                        if (i == 1)
                        {
                            myShape = isWasReversed ? Algo.InternalShape().Reversed() : Algo.InternalShape();
                        }

                        i++;
                    }
                }
            }
            else
            {
                if (myRight.IsEmpty())
                {
                    //  Modified by Sergey KHROMOV - Fri Apr 27 14:35:28 2001 Begin
                    BuildDomains(ref myFace, myWires, myRight, myJoin, myIsOpenResult, true, ref isWasReversed);
                    //  Modified by Sergey KHROMOV - Fri Apr 27 14:35:35 2001 End
                }

                for (itOW.Initialize(myRight); itOW.More(); itOW.Next())
                {
                    BRepFill_OffsetWire Algo = itOW.Value();
                    Algo.Perform(Offset, Alt);

                    if (Algo.IsDone() && !Algo.InternalShape().IsNull())
                    {
                        B.Add(Res, isWasReversed ? Algo.InternalShape().Reversed() : Algo.InternalShape());
                        if (i == 1)
                        {
                            myShape = isWasReversed ? Algo.InternalShape().Reversed() : Algo.InternalShape();
                        }

                        i++;
                    }
                }
            }

            if (i > 2)
            {
                myShape = Res;
            }

            if (myShape.IsNull())
            {
                NotDone();
            }
            else
            {
                Done();
            }
        }
        catch (Exception anException) when (anException is FailureException or NullReferenceException
                                            && anException is not (ImmutableObjectException or TopoDS_LockedShape))
        {
            // OCCT catches Standard_Failure; with the signals Draw and build123d.NET set, an access violation (the
            // medial axis's null bisector, occt-issues.md) arrives as one: the port's NullReferenceException
            // (decision 917)
            NotDone();
            myShape.Nullify();
        }
    }

    /// <summary>Does nothing: the shape is built by <see cref="Perform"/>.</summary>
    public override void Build(ProgressRange theRange = default)
    {
        Done();
    }

    private protected override void BeforePublishingLists()
    {
        myShape.TShape()?.FreezeGraph();
    }

    /// <summary>The shapes of the last parallel generated by the edge or vertex <paramref name="S"/> of a spine.</summary>
    internal override NCollection_List<TopoDS_Shape> InternalGenerated(in TopoDS_Shape S)
    {
        myGenerated.Clear();
        NCollection_List<BRepFill_OffsetWire>.Iterator itOW = new();
        NCollection_List<BRepFill_OffsetWire> Algos;
        Algos = myLeft;
        if (!myLastIsLeft)
        {
            Algos = myRight;
        }
        for (itOW.Initialize(Algos); itOW.More(); itOW.Next())
        {
            BRepFill_OffsetWire OW = itOW.Value();
            NCollection_List<TopoDS_Shape> L;
            L = new NCollection_List<TopoDS_Shape>(OW.InternalGeneratedShapes(S.Oriented(TopAbs_FORWARD)));
            myGenerated.Append(L);
            L = new NCollection_List<TopoDS_Shape>(OW.InternalGeneratedShapes(S.Oriented(TopAbs_REVERSED)));
            myGenerated.Append(L);
        }
        return myGenerated;
    }
}
