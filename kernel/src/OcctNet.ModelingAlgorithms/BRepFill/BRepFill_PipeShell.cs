// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_PipeShell.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_PipeShell.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.BRepFill_TransitionStyle;
using static Occt.BRepFill_TypeOfContact;
using static Kerf.Geometry.Continuity;
using static Occt.GeomFill_ApproxStyle;
using static Occt.GeomFill_PipeError;
using static Occt.GeomFill_Trihedron;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Occt;

/// <summary>
/// Builds a shell (or a solid) by sweeping one or more sections along a wire (the spine), under a trihedron mode
/// (corrected Frenet by default, Frenet, discrete, fixed, constant binormal, Darboux on a support, or guided by an
/// auxiliary spine), with a transition mode at the spine's corners and optionally a law of homothety.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21). The sections are placed on the spine as transformed copies, as in OCCT, so the
/// sweep never writes into the caller's profiles. The one write into a caller's shape, the spine's closed flag set
/// when its ends meet, goes to a new wire holding the caller's edges when the spine is frozen (decision 926).
/// </para>
/// <para>
/// <see cref="Shape"/>, <see cref="FirstShape"/>, <see cref="LastShape"/>, <see cref="Generated"/>,
/// <see cref="Simulate"/> and <see cref="Spine"/> hand out frozen shapes; the <c>Internal</c> forms don't. The history
/// is keyed by the caller's own sub-shapes of the profiles and the spine.
/// </para>
/// </remarks>
public sealed class BRepFill_PipeShell : Transient
{
    private Wire mySpine;
    private Shape myFirst = new();
    private Shape myLast = new();
    private Shape myShape = new();
    private readonly NCollection_Sequence<BRepFill_Section> mySeq = new();
    private readonly NCollection_Sequence<Shape> WSeq = new();
    private readonly NCollection_Sequence<int> myIndOfSec = new();
    private readonly NCollection_DataMap<Shape, NCollection_List<Shape>> myEdgeNewEdges = new(ShapeHasher.Instance);
    private readonly NCollection_DataMap<Shape, NCollection_List<Shape>> myGenMap = new(ShapeHasher.Instance);
    private double myTol3d;
    private double myBoundTol;
    private double myTolAngular;
    private double angmin; // OCCT: uninitialised until SetTransition (MakePipeShell's constructor calls it)
    private double angmax;
    private int myMaxDegree;
    private int myMaxSegments;
    private bool myForceApproxC1;
    private Law_Function? myLaw;
    private bool myIsAutomaticLaw;
    private BRepFill_LocationLaw? myLocation;
    private BRepFill_SectionLaw? mySection;
    private GeomFill_Trihedron myTrihedron;
    private BRepFill_TransitionStyle myTransition;
    private GeomFill_PipeError myStatus;
    private double myErrorOnSurf; // OCCT: uninitialised until a successful Build
    private bool myIsBuildHistory;

    //=======================================================================
    // function :  ComputeSection
    // purpose  : Construct an intermediary section
    //=======================================================================

    private static bool ComputeSection(in Wire W1, in Wire W2, double p1, double p2, out Wire Wres)
    {
        NCollection_Sequence<double> SR = new();
        NCollection_Sequence<Shape> SSh = new();
        SR.Clear();
        SR.Append(0.0);
        SR.Append(1.0);
        SSh.Clear();
        SSh.Append(W1);
        SSh.Append(W2);
        BRepFill_CompatibleWires CW = new(SSh);
        CW.SetPercent(0.1);
        CW.Perform();
        if (!CW.IsDone())
        {
            throw new NotDoneException("Uncompatible wires");
        }
        NCollection_Sequence<Transform3D> EmptyTrsfs = new();
        BRepFill_NSections SL = new(CW.InternalShape(), EmptyTrsfs, SR, 0.0, 1.0);
        double US = p1 / (p1 + p2);
        Shape aW = new();
        SL.D0(US, ref aW);
        Wres = ShapeCasts.Wire(aW);
        return true;
    }

    //=======================================================================
    // function : PerformTransition
    // purpose  : Modify a law of location depending on Transition
    //=======================================================================

    private static void PerformTransition(BRepFill_TransitionStyle Mode, BRepFill_LocationLaw? Loc, double angmin)
    {
        if (Loc is not null)
        {
            Loc.DeleteTransform();
            if (Mode == BRepFill_Modified)
            {
                Loc.TransformInG0Law();
            }
            else
            {
                Loc.TransformInCompatibleLaw(angmin);
            }
        }
    }

    //=======================================================================
    // function :  PerformPlan
    // purpose  : Construct a plane of filling if exists
    //=======================================================================

    private static bool PerformPlan(ref Shape S)
    {
        bool isDegen = true;
        Explorer explo = new(S, ShapeType.Edge);
        for (; explo.More(); explo.Next())
        {
            Edge anEdge = ShapeCasts.Edge(explo.Current());
            if (!Tool.Degenerated(anEdge))
            {
                isDegen = false;
            }
        }
        if (isDegen)
        {
            S.Nullify();
            return true;
        }

        Wire W = ShapeCasts.Wire(S);
        bool Ok = false;
        if (!W.IsNull)
        {
            BRepLib_MakeFace mkplan = new(W, true);
            if (mkplan.IsDone())
            {
                S = mkplan.InternalFace();
                Ok = true;
            }
        }
        return Ok;
    }

    //=================================================================================================

    private static bool IsSameOriented(in Shape theFace, in Shape theShell)
    {
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> anEdgeFaceMap = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(theShell, ShapeType.Edge, ShapeType.Face, anEdgeFaceMap);

        for (Explorer aFaceExplorer = new(theFace, ShapeType.Edge); aFaceExplorer.More(); aFaceExplorer.Next())
        {
            Shape aCurrentEdge = aFaceExplorer.Current();
            if (!anEdgeFaceMap.Contains(aCurrentEdge))
            {
                continue;
            }

            Orientation aCurrentEdgeOrientation = aCurrentEdge.Orientation();
            Shape anAdjFace = anEdgeFaceMap.FindFromKey(aCurrentEdge).First();
            for (Explorer anAdjFaceExplorer = new(anAdjFace, ShapeType.Edge); anAdjFaceExplorer.More(); anAdjFaceExplorer.Next())
            {
                // IsSame compares TShape and Location only; orientation may differ.
                Shape anAdjEdge = anAdjFaceExplorer.Current();
                if (anAdjEdge.IsSame(aCurrentEdge))
                {
                    return aCurrentEdgeOrientation != anAdjEdge.Orientation();
                }
            }
        }

        // No shared edge found. Orientation cannot be determined reliably and this
        // should not happen for valid input. Preserve the current face orientation
        // instead of reporting an opposite orientation and forcing a reversal.
        return true;
    }

    //=================================================================================================

    /// <summary>A pipe shell along <paramref name="Spine"/>, in corrected Frenet mode until another mode is set.</summary>
    public BRepFill_PipeShell(in Wire Spine)
    {
        mySpine = Spine;
        myForceApproxC1 = false;
        myIsAutomaticLaw = false;
        myTrihedron = GeomFill_IsCorrectedFrenet;
        myTransition = BRepFill_Modified;
        myStatus = GeomFill_PipeOk;
        myIsBuildHistory = true;
        myLocation = null;
        mySection = null;
        myLaw = null;
        SetTolerance();

        myMaxDegree = 11;
        myMaxSegments = 100;

        // Attention to closed non-declared wire !
        if (!mySpine.Closed())
        {
            Exploration.Vertices(mySpine, out Vertex Vf, out Vertex Vl);
            if (Vf.IsSame(Vl))
            {
                if (mySpine.TShape()!.IsFrozen)
                {
                    // OCCT.NET: OCCT sets the flag on the caller's wire; the port on a new wire of the same edges
                    // (decision 926)
                    BrepBuilder B = new();
                    Wire aWire = new();
                    B.MakeWire(ref aWire);
                    for (Iterator anIt = new(mySpine); anIt.More(); anIt.Next())
                    {
                        B.Add(aWire, anIt.Value());
                    }
                    mySpine = aWire;
                }
                mySpine.Closed(true);
            }
        }
    }

    //=======================================================================
    // function : Set
    // purpose  : Define a law of Frenet (Correct)
    //=======================================================================
    /// <summary>The Frenet trihedron (<paramref name="IsFrenet"/>) or the corrected Frenet trihedron.</summary>
    public void Set(bool IsFrenet = false)
    {
        GeomFill_TrihedronLaw TLaw;
        if (IsFrenet)
        {
            myTrihedron = GeomFill_IsFrenet;
            TLaw = new GeomFill_Frenet();
        }
        else
        {
            myTrihedron = GeomFill_IsFrenet; // OCCT
            TLaw = new GeomFill_CorrectedFrenet();
        }
        GeomFill_CurveAndTrihedron Loc = new(TLaw);
        myLocation = new BRepFill_Edge3DLaw(mySpine, Loc);
        mySection = null; // It is required to relocalize sections.
    }

    //=======================================================================
    // function : SetDiscrete
    // purpose  : Define a law of Discrete Trihedron
    //=======================================================================
    /// <summary>The discrete trihedron.</summary>
    public void SetDiscrete()
    {
        GeomFill_TrihedronLaw TLaw;

        myTrihedron = GeomFill_IsDiscreteTrihedron;
        TLaw = new GeomFill_DiscreteTrihedron();

        GeomFill_CurveAndTrihedron Loc = new(TLaw);
        myLocation = new BRepFill_Edge3DLaw(mySpine, Loc);
        mySection = null; // It is required to relocalize sections.
    }

    //=================================================================================================

    /// <summary>A fixed trihedron: the sections keep the orientation they have relative to <paramref name="Axe"/>.</summary>
    public void Set(in Frame3D Axe)
    {
        myTrihedron = GeomFill_IsFixed;
        Vector3D V1 = new(), V2 = new();
        V1.SetXYZ(Axe.Direction().XYZ());
        V2.SetXYZ(Axe.XDirection().XYZ());
        GeomFill_Fixed TLaw = new(V1, V2);
        GeomFill_CurveAndTrihedron Loc = new(TLaw);
        myLocation = new BRepFill_Edge3DLaw(mySpine, Loc);
        mySection = null; // It is required to relocalize sections.
    }

    //=======================================================================
    // function : Set
    // purpose  : Construct a law of location of binormal fixed type
    //=======================================================================
    /// <summary>A trihedron with the constant binormal <paramref name="BiNormal"/>.</summary>
    public void Set(in Direction3D BiNormal)
    {
        myTrihedron = GeomFill_IsConstantNormal;

        GeomFill_ConstantBiNormal TLaw = new(BiNormal);
        GeomFill_CurveAndTrihedron Loc = new(TLaw);
        myLocation = new BRepFill_Edge3DLaw(mySpine, Loc);
        mySection = null; // Il faut relocaliser les sections.
    }

    //=======================================================================
    // function : Set
    // purpose  : Construct a law of location of Darboux type
    //=======================================================================
    /// <summary>
    /// The Darboux trihedron, the normal being the normal of <paramref name="SpineSupport"/> (a shape whose faces hold
    /// the spine's edges). False if the spine isn't on it.
    /// </summary>
    public bool Set(in Shape SpineSupport)
    {
        bool B;

        // A special law of location is required
        BRepFill_EdgeOnSurfLaw loc = new(mySpine, SpineSupport);
        B = loc.HasResult();
        if (B)
        {
            myLocation = loc;
            myTrihedron = GeomFill_IsDarboux;
            mySection = null; // It is required to relocalize the sections.
        }
        return B;
    }

    //=======================================================================
    // function : Set
    // purpose  : Defines a lawv with help of a guided contour
    //=======================================================================
    /// <summary>
    /// A trihedron guided by <paramref name="AuxiliarySpine"/>: by the reduced curvilinear abscissa
    /// (<paramref name="CurvilinearEquivalence"/>) or by the plane normal to the spine, with or without contact
    /// (<paramref name="KeepContact"/>).
    /// </summary>
    public void Set(in Wire AuxiliarySpine,
                    bool CurvilinearEquivalence = true,
                    BRepFill_TypeOfContact KeepContact = BRepFill_NoContact)
    {
        // Reorganization of the guide (pb of orientation and origin)
        Wire TheGuide;
        TheGuide = AuxiliarySpine;
        bool SpClose = mySpine.Closed(), GuideClose = AuxiliarySpine.Closed();

        if (KeepContact == BRepFill_ContactOnBorder)
        {
            myIsAutomaticLaw = true;
        }

        if (!SpClose && !GuideClose)
        {
            // Case open reorientation of the guide
            Wire sp = mySpine;
            NCollection_Sequence<Shape> Seq = new();
            Seq.Append(sp);
            Seq.Append(TheGuide);
            BRepFill_CompatibleWires CW = new(Seq);
            CW.SetPercent(0.1);
            CW.Perform();
            if (!CW.IsDone())
            {
                throw new NotDoneException("Uncompatible wires");
            }
            TheGuide = ShapeCasts.Wire(CW.InternalShape().Value(2));
        }
        else if (GuideClose)
        {
            // Case guide closed : Determination of the origin
            // & reorientation of the guide
            Vector3D Dir;
            Point3D SpOr;
            if (!SpClose)
            {
                Exploration.Vertices(mySpine, out Vertex Vf, out Vertex Vl);
                SpOr = Tool.Pnt(Vf);
                Point3D P = Tool.Pnt(Vl);
                Vector3D V = new(P, SpOr);
                SpOr.BaryCenter(0.5, P, 0.5);
                Dir = V;
            }
            else
            {
                WireCurveAdaptor BC = new(mySpine);
                BC.D1(0, out SpOr, out Dir);
            }
            BRepFill.SearchOrigin(ref TheGuide, SpOr, Dir, 100 * myTol3d);
        }

        // transform the guide in a single curve
        WireCurveAdaptor Guide = new(TheGuide);

        if (CurvilinearEquivalence)
        { // trihedron by curvilinear reduced abscissa
            if (KeepContact == BRepFill_Contact || KeepContact == BRepFill_ContactOnBorder)
            {
                myTrihedron = GeomFill_IsGuideACWithContact; // with rotation
            }
            else
            {
                myTrihedron = GeomFill_IsGuideAC; // without rotation
            }

            GeomFill_GuideTrihedronAC TLaw = new(Guide);
            GeomFill_LocationGuide Loc = new(TLaw);
            myLocation = new BRepFill_ACRLaw(mySpine, Loc);
        }
        else
        { // trihedron by plane
            if (KeepContact == BRepFill_Contact || KeepContact == BRepFill_ContactOnBorder)
            {
                myTrihedron = GeomFill_IsGuidePlanWithContact; // with rotation
            }
            else
            {
                myTrihedron = GeomFill_IsGuidePlan; // without rotation
            }

            GeomFill_GuideTrihedronPlan TLaw = new(Guide);
            GeomFill_LocationGuide Loc = new(TLaw);
            myLocation = new BRepFill_Edge3DLaw(mySpine, Loc);
        }
        mySection = null; // It is required to relocalize the sections.
    }

    //=================================================================================================

    /// <summary>The maximal degree of the swept surfaces (11 by default).</summary>
    public void SetMaxDegree(int NewMaxDegree)
    {
        myMaxDegree = NewMaxDegree;
    }

    //=================================================================================================

    /// <summary>The maximal number of segments of the swept surfaces (100 by default).</summary>
    public void SetMaxSegments(int NewMaxSegments)
    {
        myMaxSegments = NewMaxSegments;
    }

    //=======================================================================
    // function : SetForceApproxC1
    // purpose  : Set the flag that indicates attempt to approximate
    //           a C1-continuous surface if a swept surface proved
    //           to be C0.
    //=======================================================================
    /// <summary>Tries to approximate C1 surfaces where a swept surface is C0.</summary>
    public void SetForceApproxC1(bool ForceApproxC1)
    {
        myForceApproxC1 = ForceApproxC1;
    }

    /// <summary>Whether <see cref="Build"/> builds the history (true by default).</summary>
    public void SetIsBuildHistory(bool theIsBuildHistory)
    {
        myIsBuildHistory = theIsBuildHistory;
    }

    /// <summary>True if <see cref="Build"/> builds the history.</summary>
    public bool IsBuildHistory() => myIsBuildHistory;

    //=================================================================================================

    /// <summary>
    /// Adds the section <paramref name="Profile"/> (a wire or a vertex), placed where it is, translated onto the
    /// spine with <paramref name="WithContact"/>, and rotated to be orthogonal to it with
    /// <paramref name="WithCorrection"/>.
    /// </summary>
    public void Add(in Shape Profile, bool WithContact = false, bool WithCorrection = false)
    {
        Vertex V = new();
        V.Nullify();
        Add(Profile, V, WithContact, WithCorrection);
        ResetLoc();
    }

    //=================================================================================================

    /// <summary>
    /// Adds the section <paramref name="Profile"/> placed at the spine's vertex <paramref name="Location"/>.
    /// </summary>
    public void Add(in Shape Profile, in Vertex Location, bool WithContact = false, bool WithCorrection = false)
    {
        DeleteProfile(Profile); // No duplication
        if (myIsAutomaticLaw)
        {
            mySeq.Clear();
            BRepFill_Section S = new(Profile, Location, WithContact, WithCorrection);
            S.Set(true);
            mySeq.Append(S);
            mySection = null;
            ResetLoc();

            GeomFill_LocationGuide Loc = (GeomFill_LocationGuide)myLocation!.Law(1);
            Loc.ComputeAutomaticLaw(out NCollection_HArray1<Point2D> ParAndRad);

            // Compuite initial width of section (this will be 1.)
            Properties GlobalProps = new();
            BRepGProp.LinearProperties(Profile, GlobalProps);
            Point3D BaryCenter = GlobalProps.CentreOfMass();

            Face ProfileFace = new BRepLib_MakeFace(ShapeCasts.Wire(Profile), true).InternalFace(); // only plane
            Surface thePlane = Tool.Surface(ProfileFace)!;
            SurfaceAdaptor GAHplane = new(thePlane);
            IntCurveSurface_HInter Intersector = new();
            CurveAdaptorBase[] aHCurve = new CurveAdaptorBase[2];
            aHCurve[0] = Loc.GetCurve()!;
            aHCurve[1] = Loc.Guide();
            Point3D[] PointsOnSpines = [new(), new()];
            int i, j;

            for (i = 0; i < 2; i++)
            {
                Intersector.Perform(aHCurve[i], GAHplane);
                double MinDist = RealLast();
                for (j = 1; j <= Intersector.NbPoints(); j++)
                {
                    Point3D aPint = Intersector.Point(j).Pnt();
                    double aDist = BaryCenter.Distance(aPint);
                    if (aDist < MinDist)
                    {
                        MinDist = aDist;
                        PointsOnSpines[i] = aPint;
                    }
                }
            }

            // Correct <ParAndRad> according to <InitialWidth>
            double InitialWidth = PointsOnSpines[0].Distance(PointsOnSpines[1]);
            int NbParRad = ParAndRad.Upper();
            for (i = 1; i <= NbParRad; i++)
            {
                Point2D aParRad = ParAndRad.Value(i);
                aParRad.SetY(aParRad.Y() / InitialWidth);
                ParAndRad.SetValue(i, aParRad);
            }

            myLaw = new Law_Interpol();

            bool IsPeriodic = (Math.Abs(ParAndRad.Value(1).Y() - ParAndRad.Value(NbParRad).Y()) < global::Kerf.Engine.Foundation.Precision.Confusion());

            ((Law_Interpol)myLaw).Set(ParAndRad.Array1(), IsPeriodic);
        }
        else
        {
            BRepFill_Section S = new(Profile, Location, WithContact, WithCorrection);
            mySeq.Append(S);
            mySection = null;
            ResetLoc();
        }
    }

    //=======================================================================
    // function : SetLaw
    // purpose  : Section + law of homothety
    //=======================================================================
    /// <summary>The only section <paramref name="Profile"/>, scaled along the spine by the law <paramref name="L"/>.</summary>
    public void SetLaw(in Shape Profile, Law_Function L, bool WithContact = false, bool WithCorrection = false)
    {
        Vertex V = new();
        V.Nullify();
        SetLaw(Profile, L, V, WithContact, WithCorrection);
        ResetLoc();
    }

    //=======================================================================
    // function : SetLaw
    // purpose  :  Section + Law of homothety
    //=======================================================================
    /// <summary>
    /// The only section <paramref name="Profile"/>, placed at <paramref name="Location"/> and scaled along the spine by
    /// the law <paramref name="L"/>.
    /// </summary>
    public void SetLaw(in Shape Profile,
                       Law_Function L,
                       in Vertex Location,
                       bool WithContact = false,
                       bool WithCorrection = false)
    {
        mySeq.Clear();
        BRepFill_Section S = new(Profile, Location, WithContact, WithCorrection);
        S.Set(true);
        mySeq.Append(S);
        myLaw = L;
        mySection = null;
        ResetLoc();
    }

    //=================================================================================================

    /// <summary>Removes the section <paramref name="Profile"/>.</summary>
    public void DeleteProfile(in Shape Profile)
    {
        bool Trouve = false;
        int ii;
        for (ii = 1; ii <= mySeq.Length() && !Trouve; ii++)
        {
            Shape aSection = mySeq.Value(ii).OriginalShape();
            if (Profile.IsSame(aSection))
            {
                Trouve = true;
                mySeq.Remove(ii);
            }
        }

        if (Trouve)
        {
            mySection = null;
        }
        ResetLoc();
    }

    //=================================================================================================

    /// <summary>True if a section was added.</summary>
    public bool IsReady() => (mySeq.Length() != 0);

    //=================================================================================================

    /// <summary>The status of the last build.</summary>
    public GeomFill_PipeError GetStatus() => myStatus;

    //=================================================================================================

    /// <summary>The tolerances: 3D, of the bounds, and angular.</summary>
    public void SetTolerance(double Tol3d = 1.0e-4, double BoundTol = 1.0e-4, double TolAngular = 1.0e-2)
    {
        myTol3d = Tol3d;
        myBoundTol = BoundTol;
        myTolAngular = TolAngular;
    }

    //=======================================================================
    // function : SetTransition
    // purpose  : Defines the mode of processing of corners
    //=======================================================================
    /// <summary>The treatment of the spine's corners, and the angles bounding it.</summary>
    public void SetTransition(BRepFill_TransitionStyle Mode = BRepFill_Modified, double Angmin = 1.0e-2, double Angmax = 6.0)
    {
        if (myTransition != Mode)
        {
            mySection = null; // It is required to relocalize the sections.
        }
        myTransition = Mode;
        angmin = Angmin;
        angmax = Angmax;
    }

    //=================================================================================================

    /// <summary>
    /// Fills <paramref name="List"/> with <paramref name="N"/> sections placed along the spine, without building the
    /// sweep (frozen).
    /// </summary>
    public void Simulate(int N, NCollection_List<Shape> List)
    {
        // Preparation
        Prepare();
        List.Clear();

        double First, Last, Length, Delta, U, US, DeltaS, FirstS;
        int ii, NbL = myLocation!.NbLaw();
        bool Finis = false;
        Shape W = new();

        // Calculate the parameters of digitalization
        mySection!.Law(1).GetDomain(out FirstS, out Last);
        DeltaS = Last - FirstS;
        myLocation.CurvilinearBounds(NbL, out First, out Length);
        Delta = Length;
        if (N > 1)
        {
            Delta /= (N - 1);
        }

        myLocation.CurvilinearBounds(1, out First, out Last); // Initiation of Last
        for (U = 0.0, ii = 1; !Finis; U += Delta)
        {
            if (U >= Length)
            {
                U = Length;
                Finis = true;
            }
            else
            {
                if (ii < NbL)
                {
                    myLocation.CurvilinearBounds(NbL, out First, out Last);
                }
                if (U > Last)
                {
                    U = (Last + First) / 2; // The edge is not skipped
                }
                if (U > First)
                {
                    ii++;
                }
            }
            US = FirstS + (U / Length) * DeltaS;
            // Calcul d'une section
            mySection.D0(US, ref W);
            myLocation.D0(U, ref W);
            List.Append(W);
        }
    }

    //=======================================================================
    // function : Build
    // purpose  : Construct the Shell and the history
    //=======================================================================
    /// <summary>Builds the shell and its history. False if it fails (see <see cref="GetStatus"/>).</summary>
    public bool Build()
    {
        bool Ok;
        // 1) Preparation
        Prepare();

        if (myStatus != GeomFill_PipeOk)
        {
            BrepBuilder B = new();
            Shell Sh = new();
            B.MakeShell(ref Sh);
            myShape = Sh; // Nullify
            return false;
        }

        // 2) Calculate myFirst and myLast
        mySection!.Law(1).GetDomain(out double FirstS, out double LastS);
        mySection.D0(FirstS, ref myFirst);
        myLocation!.InternalD0(0, ref myFirst);
        if (mySection.IsVClosed() && myLocation.IsClosed())
        {
            if (myLocation.IsG1(0) >= 0)
            {
                myLast = myFirst;
            }
            else
            {
                myFirst.Nullify();
                myLast.Nullify();
            }
        }
        else
        {
            myLocation.CurvilinearBounds(myLocation.NbLaw(), out FirstS, out double Length);
            mySection.D0(LastS, ref myLast);
            myLocation.InternalD0(Length, ref myLast);
            // eap 5 Jun 2002 occ332, myLast and myFirst must not share one TShape,
            // tolerances of shapes built on them may be quite different
            if (myFirst.IsPartner(myLast))
            {
                BRepBuilderAPI_Copy copy = new(myLast);
                if (copy.IsDone())
                {
                    myLast = copy.InternalShape();
                }
            }
            // eap 5 Jun 2002 occ332, end modif
        }

        // 3) Construction
        BRepFill_Sweep MkSw = new(mySection, myLocation, true);
        MkSw.SetTolerance(myTol3d, myBoundTol, 1.0e-5, myTolAngular);
        MkSw.SetAngularControl(angmin, angmax);
        MkSw.SetForceApproxC1(myForceApproxC1);
        MkSw.SetBounds(ShapeCasts.Wire(myFirst), ShapeCasts.Wire(myLast));

        Continuity theContinuity = C2;
        if (myTrihedron == GeomFill_IsDiscreteTrihedron)
        {
            theContinuity = C0;
        }
        NCollection_Map<Shape> Dummy = new(ShapeHasher.Instance);
        NCollection_DataMap<Shape, NCollection_HArray2<Shape>> Dummy2 = new(ShapeHasher.Instance);
        NCollection_DataMap<Shape, NCollection_HArray2<Shape>> Dummy3 = new(ShapeHasher.Instance);
        MkSw.Build(Dummy, Dummy2, Dummy3, myTransition, theContinuity, GeomFill_Location, myMaxDegree, myMaxSegments);

        myStatus = myLocation.GetStatus();
        Ok = (MkSw.IsDone() && (myStatus == GeomFill_PipeOk));

        if (Ok)
        {
            myShape = MkSw.Shape();
            myErrorOnSurf = MkSw.ErrorOnSurface();

            Shape aBottomWire = myFirst;
            Shape aTopWire = myLast;

            if (BuildBoundaries(MkSw, mySection, ref aBottomWire, ref aTopWire))
            {
                myFirst = aBottomWire;
                myLast = aTopWire;
            }

            if (mySection.IsUClosed())
            {
                Explorer explo = new();
                bool DegenFirst = true, DegenLast = true;

                for (explo.Init(myFirst, ShapeType.Edge); explo.More(); explo.Next())
                {
                    Edge anEdge = ShapeCasts.Edge(explo.Current());
                    DegenFirst = DegenFirst && Tool.Degenerated(anEdge);
                }

                for (explo.Init(myLast, ShapeType.Edge); explo.More(); explo.Next())
                {
                    Edge anEdge = ShapeCasts.Edge(explo.Current());
                    DegenLast = DegenLast && Tool.Degenerated(anEdge);
                }

                if (DegenFirst && DegenLast)
                {
                    myShape.Closed(true);
                }
            }

            if (myIsBuildHistory)
            {
                BuildHistory(MkSw);
            }
        }
        else
        {
            BrepBuilder B = new();
            Shell Sh = new();
            B.MakeShell(ref Sh);
            myShape = Sh; // Nullify
            if (myStatus == GeomFill_PipeOk)
            {
                myStatus = GeomFill_PipeNotOk;
            }
        }
        return Ok;
    }

    //=================================================================================================

    /// <summary>
    /// Closes the shell with planar faces at its ends and makes a solid of it. False if it can't.
    /// </summary>
    /// <exception cref="NotDoneException">The shell isn't built.</exception>
    public bool MakeSolid()
    {
        if (myShape.IsNull)
        {
            throw new NotDoneException("PipeShell is not built");
        }
        bool B = myShape.Closed();
        BrepBuilder BS = new();

        if (!B)
        {
            if (!myFirst.IsNull && !myLast.IsNull)
            {
                B = (myFirst.Closed() && myLast.Closed());
            }
            if (B)
            {
                // It is necessary to block the extremities
                B = PerformPlan(ref myFirst);
                if (B)
                {
                    B = PerformPlan(ref myLast);
                    if (B)
                    {
                        if (myShape.TShape()!.IsFrozen)
                        {
                            // OCCT.NET: OCCT adds the end faces to the shell the caller may already hold (Shape()); a
                            // published shell is frozen, so the port adds them to a new shell of the same faces
                            // (decision 926)
                            Shape aShell = myShape.EmptyCopied();
                            for (Iterator anIt = new(myShape, false, false); anIt.More(); anIt.Next())
                            {
                                BS.Add(aShell, anIt.Value());
                            }
                            myShape = aShell;
                        }
                        if (!myFirst.IsNull && !IsSameOriented(myFirst, myShape))
                        {
                            myFirst.Reverse();
                        }
                        if (!myLast.IsNull && !IsSameOriented(myLast, myShape))
                        {
                            myLast.Reverse();
                        }

                        if (!myFirst.IsNull)
                        {
                            BS.Add(myShape, ShapeCasts.Face(myFirst));
                        }
                        if (!myLast.IsNull)
                        {
                            BS.Add(myShape, ShapeCasts.Face(myLast));
                        }

                        myShape.Closed(true);
                    }
                }
            }
        }

        if (B)
        {
            Solid solid = new();
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
            myShape.Closed(true);
        }
        return B;
    }

    //=================================================================================================

    private static Shape Published(Shape theShape)
    {
        theShape.TShape()?.FreezeGraph();
        return theShape;
    }

    /// <summary>The shell or solid built (frozen).</summary>
    public Shape Shape() => Published(myShape);

    /// <summary><see cref="Shape"/> without freezing, for ported callers.</summary>
    internal Shape InternalShape() => myShape;

    //=================================================================================================

    /// <summary>The approximation error of the swept surfaces.</summary>
    public double ErrorOnSurface() => myErrorOnSurf;

    //=================================================================================================

    /// <summary>The shape at the start of the sweep (frozen).</summary>
    public Shape FirstShape() => Published(myFirst);

    /// <summary><see cref="FirstShape"/> without freezing, for ported callers.</summary>
    internal Shape InternalFirstShape() => myFirst;

    //=================================================================================================

    /// <summary>The shape at the end of the sweep (frozen).</summary>
    public Shape LastShape() => Published(myLast);

    /// <summary><see cref="LastShape"/> without freezing, for ported callers.</summary>
    internal Shape InternalLastShape() => myLast;

    /// <summary>Appends the profiles, as given, to <paramref name="theProfiles"/>.</summary>
    public void Profiles(NCollection_List<Shape> theProfiles)
    {
        for (int i = 1; i <= mySeq.Length(); ++i)
        {
            theProfiles.Append(mySeq[i].OriginalShape());
        }
    }

    /// <summary>The spine (frozen; a new wire of the caller's edges when the port had to close it).</summary>
    public Wire Spine() => ShapeCasts.Wire(Published(mySpine));

    //=================================================================================================

    /// <summary>
    /// Fills <paramref name="theList"/> with the shapes generated from <paramref name="theShape"/>, a profile or one
    /// of its edges or vertices, or an edge or vertex of the spine (frozen).
    /// </summary>
    public void Generated(in Shape theShape, NCollection_List<Shape> theList)
    {
        InternalGenerated(theShape, theList);
        for (NCollection_List<Shape>.Iterator anIt = new(theList); anIt.More(); anIt.Next())
        {
            anIt.Value().TShape()?.FreezeGraph();
        }
    }

    /// <summary><see cref="Generated"/> without freezing, for ported callers.</summary>
    internal void InternalGenerated(in Shape theShape, NCollection_List<Shape> theList)
    {
        theList.Clear();

        if (myGenMap.IsBound(theShape))
        {
            theList.Assign(myGenMap.Find(theShape));
        }
    }

    //=======================================================================
    // function : Prepare
    // purpose  : - Check that everything is ready
    //           - Construct the law of section
    //           - Construct the law of location if required
    //           - Calculate First & Last
    //=======================================================================
    private void Prepare()
    {
        WSeq.Clear();
        myEdgeNewEdges.Clear(); // OCCT: myIndOfSec isn't cleared, so a second Prepare appends to it

        Wire theSect;
        if (!IsReady())
        {
            throw new NotDoneException("PipeShell");
        }
        if (myLocation is not null && mySection is not null)
        {
            return; // It is ready
        }

        // Check set of section for right configuration of punctual sections
        int i;
        Iterator iter = new();
        for (i = 2; i <= mySeq.Length() - 1; i++)
        {
            bool wdeg = true;
            for (iter.Initialize(mySeq[i].InternalWire()); iter.More(); iter.Next())
            {
                Edge anEdge = ShapeCasts.Edge(iter.Value());
                wdeg = wdeg && (Tool.Degenerated(anEdge));
            }
            if (wdeg)
            {
                throw new FailureException("Wrong usage of punctual sections");
            }
        }
        if (mySeq.Length() <= 2)
        {
            bool wdeg = true;
            for (i = 1; i <= mySeq.Length(); i++)
            {
                for (iter.Initialize(mySeq[i].InternalWire()); iter.More(); iter.Next())
                {
                    Edge anEdge = ShapeCasts.Edge(iter.Value());
                    wdeg = wdeg && (Tool.Degenerated(anEdge));
                }
            }
            if (wdeg)
            {
                throw new FailureException("Wrong usage of punctual sections");
            }
        }

        // Construction of the law of location
        if (myLocation is null)
        {
            switch (myTrihedron)
            {
                case GeomFill_IsCorrectedFrenet:
                {
                    GeomFill_TrihedronLaw TLaw = new GeomFill_CorrectedFrenet();
                    GeomFill_CurveAndTrihedron Loc = new(TLaw);
                    myLocation = new BRepFill_Edge3DLaw(mySpine, Loc);
                    break;
                }
                default:
                { // Not planned!
                    throw new ConstructionErrorException("PipeShell");
                }
            }
        }

        // Transformation of the law (Transition Management)
        PerformTransition(myTransition, myLocation, angmin);

        // Construction of the section law
        if (mySeq.Length() == 1)
        {
            Place(mySeq[1], out theSect, out _, out _);
            Wire aLocalShape = theSect;
            if (mySeq[1].IsLaw())
            {
                mySection = new BRepFill_ShapeLaw(aLocalShape, myLaw!);
            }
            else
            {
                mySection = new BRepFill_ShapeLaw(aLocalShape);
            }

            WSeq.Append(theSect);
            // Simple case of single section
            myIndOfSec.Append(1);
            Iterator itw = new(theSect);
            for (; itw.More(); itw.Next())
            {
                Shape anEdge = itw.Value();
                NCollection_List<Shape> Elist = new();
                Elist.Append(anEdge);
                myEdgeNewEdges.Bind(anEdge, Elist);
            }
            ///////////////////////////////
        }
        else
        {
            NCollection_Sequence<double> Param = new();
            NCollection_Sequence<int> IndSec = new();
            NCollection_Sequence<Transform3D> Transformations = new();
            int NbL = myLocation.NbLaw();
            double V1, V2;
            myLocation.CurvilinearBounds(NbL, out V1, out V2);
            V1 = 0.0;
            int ideb = 0, ifin = 0;
            int iseq;
            for (iseq = 1; iseq <= mySeq.Length(); iseq++)
            {
                IndSec.Append(iseq);
                Place(mySeq[iseq], out theSect, out Transform3D aTrsf, out double param);
                Param.Append(param);
                WSeq.Append(theSect);
                Transformations.Append(aTrsf);
                if (param == V1)
                {
                    ideb = iseq;
                }
                if (param == V2)
                {
                    ifin = iseq;
                }
            }

            // looping sections ?
            if (myLocation.IsClosed())
            {
                if (ideb > 0)
                {
                    // place the initial section at the final position
                    Param.Append(V2);
                    WSeq.Append(WSeq[ideb]);
                }
                else if (ifin > 0)
                {
                    // place the final section at the initial position
                    Param.Append(V1);
                    WSeq.Append(WSeq[ifin]);
                }
                else
                {
                    // it is necessary to find a medium section to impose by V1 and by V2
                    double pmin = RealLast(), pmax = RealFirst();
                    Wire Wmin = new(), Wmax = new();
                    for (iseq = 1; iseq <= WSeq.Length(); iseq++)
                    {
                        if (Param.Value(iseq) < pmin)
                        {
                            pmin = Param.Value(iseq);
                            Wmin = ShapeCasts.Wire(WSeq.Value(iseq));
                        }
                        if (Param.Value(iseq) > pmax)
                        {
                            pmax = Param.Value(iseq);
                            Wmax = ShapeCasts.Wire(WSeq.Value(iseq));
                        }
                    }
                    // medium section between Wmin and Wmax
                    double dmin = Math.Abs(pmin - V1);
                    double dmax = Math.Abs(pmax - V2);
                    if (ComputeSection(Wmin, Wmax, dmin, dmax, out Wire Wres))
                    {
                        // impose section Wres at the beginning and the end
                        Param.Append(V1);
                        WSeq.Append(Wres);
                        IndSec.Append(WSeq.Length());
                        Param.Append(V2);
                        WSeq.Append(Wres);
                        IndSec.Append(WSeq.Length());
                    }
                }
            }

            // parse sections by increasing parameter
            bool play_again = true;
            while (play_again)
            {
                play_again = false;
                for (iseq = 1; iseq <= WSeq.Length(); iseq++)
                {
                    for (int jseq = iseq + 1; jseq <= WSeq.Length(); jseq++)
                    {
                        if (Param.Value(iseq) > Param.Value(jseq))
                        {
                            Param.Exchange(iseq, jseq);
                            WSeq.Exchange(iseq, jseq);
                            IndSec.Exchange(iseq, jseq);
                            play_again = true;
                        }
                    }
                }
            }
            // Fill the array of real indices of sections
            for (int ii = 1; ii <= mySeq.Length(); ii++)
            {
                for (int jj = 1; jj <= IndSec.Length(); jj++)
                {
                    if (IndSec[jj] == ii)
                    {
                        myIndOfSec.Append(jj);
                        break;
                    }
                }
            }

            //  Calculate work sections
            NCollection_Sequence<Shape> WorkingSections = new();
            WorkingSections.Clear();
            BRepFill_CompatibleWires Georges = new(WSeq);
            Georges.SetPercent(0.1);
            Georges.Perform(false);
            if (Georges.IsDone())
            {
                WorkingSections.Assign(Georges.InternalShape());
                // (OCCT copies Georges.Generated() into a WorkingMap it never reads.)
                // For each sub-edge of each section
                // we save its splits
                for (int ii = 1; ii <= WSeq.Length(); ii++)
                {
                    Explorer Explo = new(WSeq[ii], ShapeType.Edge);
                    for (; Explo.More(); Explo.Next())
                    {
                        Edge anEdge = ShapeCasts.Edge(Explo.Current());
                        NCollection_List<Shape> aNewEdges = Georges.InternalGeneratedShapes(anEdge);
                        myEdgeNewEdges.Bind(anEdge, new NCollection_List<Shape>(aNewEdges));
                    }
                }
            }
            else
            {
                throw new ConstructionErrorException("PipeShell : uncompatible wires");
            }
            mySection = new BRepFill_NSections(WorkingSections, Transformations, Param, V1, V2);
        } // else

        //  modify the law of location if contact
        if ((myTrihedron == GeomFill_IsGuidePlanWithContact) || (myTrihedron == GeomFill_IsGuideACWithContact))
        {
            double fs, f, l, Delta, Length;
            GeomFill_LocationGuide Loc;
            GeomFill_SectionLaw Sec = mySection.ConcatenedLaw()!;
            myLocation.CurvilinearBounds(myLocation.NbLaw(), out f, out Length);
            Sec.GetDomain(out fs, out l);
            Delta = (l - fs) / Length;

            double angle = 0, old_angle = 0;
            for (int ipath = 1; ipath <= myLocation.NbLaw(); ipath++)
            {
                myLocation.CurvilinearBounds(ipath, out f, out l);
                Loc = (GeomFill_LocationGuide)myLocation.Law(ipath);
                Loc.Set(Sec, true, fs + f * Delta, fs + l * Delta, old_angle, ref angle); // force the rotation
                old_angle = angle;
            }
        }

        myStatus = myLocation.GetStatus();
        if (!mySection.IsDone())
        {
            myStatus = GeomFill_PipeNotOk;
        }
    }

    //=======================================================================
    // function : Place
    // purpose  : Implement a Section in the local reference frame
    //           and return its parameter on the trajectory
    //=======================================================================
    private void Place(in BRepFill_Section Sec, out Wire W, out Transform3D aTrsf, out double param)
    {
        BRepFill_SectionPlacement Place = new(myLocation!, Sec.InternalWire(), Sec.Vertex(), Sec.WithContact(), Sec.WithCorrection());
        Wire TmpWire = Sec.InternalWire();
        aTrsf = Place.Transformation();
        // Transform the copy
        W = ShapeCasts.Wire(new BRepBuilderAPI_Transform(TmpWire, aTrsf, true).InternalShape());
        ////////////////////////////////////
        param = Place.AbscissaOnPath();
    }

    //=======================================================================
    // function : ResetLoc
    // purpose  : Remove references to the sections in the laws of location
    //=======================================================================
    private void ResetLoc()
    {
        if ((myTrihedron == GeomFill_IsGuidePlanWithContact) || (myTrihedron == GeomFill_IsGuideACWithContact))
        {
            GeomFill_LocationGuide Loc;
            for (int isec = 1; isec <= myLocation!.NbLaw(); isec++)
            {
                Loc = (GeomFill_LocationGuide)myLocation.Law(isec);
                Loc.EraseRotation(); // remove the rotation
            }
        }
    }

    //=======================================================================
    // function : BuildHistory
    // purpose  : Builds history for edges and vertices of sections,
    //           for edges and vertices of spine
    //=======================================================================
    private void BuildHistory(BRepFill_Sweep theSweep)
    {
        // Filling of <myGenMap>
        NCollection_HArray2<Shape> anUEdges = theSweep.InterFaces()!;
        BrepBuilder BB = new();

        NCollection_DataMap<int, Shape> IndWireMap = new();

        int indw, inde;
        Iterator itw = new();
        for (indw = 1; indw <= mySeq.Length(); indw++)
        {
            Shape Section = mySeq[indw].OriginalShape();
            Wire aSection;
            bool IsPunctual = mySeq[indw].IsPunctual();
            if (IsPunctual)
            {
                // for punctual sections (first or last)
                // we take all the wires generated along the path

                NCollection_List<Shape> Elist = myGenMap.Bound(Section, new NCollection_List<Shape>());
                for (int i = 1; i <= anUEdges.UpperRow(); i++)
                {
                    for (int j = 1; j <= anUEdges.UpperCol(); j++)
                    {
                        Elist.Append(anUEdges.Value(i, j));
                    }
                }

                continue;
            }
            else
            {
                aSection = ShapeCasts.Wire(Section);
            }
            // Take the real index of section on the path
            int IndOfW = myIndOfSec[indw];
            Wire theWire = ShapeCasts.Wire(WSeq[IndOfW]);
            WireExplorer wexp_sec = new(aSection);
            for (inde = 1; wexp_sec.More(); wexp_sec.Next())
            {
                Edge anOriginalEdge = ShapeCasts.Edge(wexp_sec.Current());
                Edge anEdge = ShapeCasts.Edge(mySeq[indw].InternalModifiedShape(anOriginalEdge));
                if (Tool.Degenerated(anEdge))
                {
                    continue;
                }

                Shell aShell = new();
                BB.MakeShell(ref aShell);
                Vertex[] aVertex = new Vertex[2];
                Exploration.Vertices(anOriginalEdge, out aVertex[0], out aVertex[1]);
                int SignOfAnEdge = (anOriginalEdge.Orientation() == Forward) ? 1 : -1;

                // For each non-degenerated inde-th edge of <aSection>
                // we find inde-th edge in <theWire>
                Edge theEdge = new();
                WireExplorer wexp = new(theWire);
                for (int i = 1; wexp.More(); wexp.Next())
                {
                    theEdge = ShapeCasts.Edge(wexp.Current());
                    if (Tool.Degenerated(anEdge)) // OCCT: tests anEdge, not theEdge
                    {
                        continue;
                    }
                    if (i == inde)
                    {
                        break;
                    }
                    i++;
                }

                // Take the list of splits for <theEdge>
                NCollection_List<Shape> NewEdges = myEdgeNewEdges.Find(theEdge);
                int SignOfANewEdge = 0, SignOfIndex = 0;
                NCollection_List<Shape>.Iterator iter = new(NewEdges);
                for (; iter.More(); iter.Next())
                {
                    Edge aNewEdge = ShapeCasts.Edge(iter.Value());
                    SignOfANewEdge = (aNewEdge.Orientation() == Forward) ? 1 : -1;
                    int anIndE = mySection!.IndexOfEdge(aNewEdge);
                    SignOfIndex = (anIndE > 0) ? 1 : -1;
                    anIndE = Math.Abs(anIndE);
                    // For an edge generated shape is a "tape" -
                    // a shell usually containing this edge and
                    // passing from beginning of path to its end
                    Shape aTape = theSweep.Tape(anIndE);
                    Iterator itsh = new(aTape);
                    for (; itsh.More(); itsh.Next())
                    {
                        BB.Add(aShell, itsh.Value());
                    }
                }

                // Processing of vertices of <anEdge>
                // We should choose right index in <anUEdges>
                // for each vertex of edge
                int ToReverse = SignOfAnEdge * SignOfANewEdge * SignOfIndex;
                int[] UIndex = new int[2];
                UIndex[0] = Math.Abs(mySection!.IndexOfEdge(NewEdges.First()));
                UIndex[1] = Math.Abs(mySection.IndexOfEdge(NewEdges.Last())) + ToReverse;
                if (ToReverse == -1)
                {
                    UIndex[0]++;
                    UIndex[1]++;
                }
                if (mySection.IsUClosed())
                {
                    if (UIndex[0] > mySection.NbLaw())
                    {
                        UIndex[0] = 1;
                    }
                    if (UIndex[1] > mySection.NbLaw())
                    {
                        UIndex[1] = 1;
                    }
                }
                // if (SignOfAnEdge * SignOfANewEdge == -1)
                if (SignOfAnEdge == -1 || SignOfANewEdge == -1)
                {
                    int Tmp = UIndex[0];
                    UIndex[0] = UIndex[1];
                    UIndex[1] = Tmp;
                }

                NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> VEmap = new(ShapeHasher.Instance);
                Exploration.MapShapesAndAncestors(aShell, ShapeType.Vertex, ShapeType.Edge, VEmap);
                for (int kk = 0; kk < 2; kk++)
                {
                    if (myGenMap.IsBound(aVertex[kk]))
                    {
                        continue;
                    }
                    if (IndWireMap.IsBound(UIndex[kk]))
                    {
                        NCollection_List<Shape> Elist = myGenMap.Bound(aVertex[kk], new NCollection_List<Shape>());

                        for (itw.Initialize(IndWireMap.Find(UIndex[kk])); itw.More(); itw.Next())
                        {
                            Elist.Append(itw.Value());
                        }

                        continue;
                    }

                    // Collect u-edges
                    NCollection_Sequence<Shape> SeqEdges = new();
                    int jj;
                    for (jj = 1; jj <= anUEdges.UpperCol(); jj++)
                    {
                        SeqEdges.Append(anUEdges.Value(UIndex[kk], jj));
                    }

                    // Assemble the wire ("rail" along the path)
                    // checking for possible holes
                    //(they appear with option "Round Corner")
                    // and filling them
                    // Missed edges are taken from <aShell>
                    Wire aWire = new();
                    BB.MakeWire(ref aWire);
                    Edge FirstEdge = ShapeCasts.Edge(SeqEdges[1]);
                    if (FirstEdge.IsNull)
                    {
                        continue;
                    }
                    BB.Add(aWire, FirstEdge);
                    Exploration.Vertices(FirstEdge, out Vertex FirstVertex, out Vertex CurVertex);
                    Edge CurEdge = new();
                    for (jj = 2; jj <= SeqEdges.Length(); jj++)
                    {
                        CurEdge = ShapeCasts.Edge(SeqEdges[jj]);
                        Exploration.Vertices(CurEdge, out Vertex Vfirst, out Vertex Vlast);
                        if (CurVertex.IsSame(Vfirst))
                        {
                            CurVertex = Vlast;
                        }
                        else // a hole
                        {
                            NCollection_List<Shape> Elist = VEmap.FindFromKey(Vfirst);
                            NCollection_List<Shape>.Iterator itl = new(Elist);
                            for (; itl.More(); itl.Next())
                            {
                                Edge Candidate = ShapeCasts.Edge(itl.Value());
                                if (Candidate.IsSame(CurEdge))
                                {
                                    continue;
                                }
                                Exploration.Vertices(Candidate, out Vertex V1, out Vertex V2);
                                if (V1.IsSame(CurVertex) || V2.IsSame(CurVertex))
                                {
                                    BB.Add(aWire, Candidate);
                                    break;
                                }
                            }
                        }
                        CurVertex = Vlast;
                        BB.Add(aWire, CurEdge);
                    } // for (jj = 2; jj <= SeqEdges.Length(); jj++)
                    // case of closed wire
                    if (myLocation!.IsClosed() && !CurVertex.IsSame(FirstVertex))
                    {
                        NCollection_List<Shape> Elist = VEmap.FindFromKey(CurVertex);
                        NCollection_List<Shape>.Iterator itl = new(Elist);
                        for (; itl.More(); itl.Next())
                        {
                            Edge Candidate = ShapeCasts.Edge(itl.Value());
                            if (Candidate.IsSame(CurEdge))
                            {
                                continue;
                            }
                            Exploration.Vertices(Candidate, out Vertex V1, out Vertex V2);
                            if (V1.IsSame(FirstVertex) || V2.IsSame(FirstVertex))
                            {
                                BB.Add(aWire, Candidate);
                                break;
                            }
                        }
                    }

                    NCollection_List<Shape> Elist2 = myGenMap.Bound(aVertex[kk], new NCollection_List<Shape>());

                    for (itw.Initialize(aWire); itw.More(); itw.Next())
                    {
                        Elist2.Append(itw.Value());
                    }

                    // Save already built wire with its index
                    IndWireMap.Bind(UIndex[kk], aWire);
                } // for (int kk = 0; kk < 2; kk++)
                ////////////////////////////////////

                NCollection_List<Shape> Flist = myGenMap.Bound(anOriginalEdge, new NCollection_List<Shape>());
                Iterator itsh2 = new(aShell);
                for (; itsh2.More(); itsh2.Next())
                {
                    Flist.Append(itsh2.Value());
                }
                ////////////////////////

                inde++;
            }
        }

        // For subshapes of spine
        NCollection_HArray2<Shape> aFaces = theSweep.SubShape()!;
        NCollection_HArray2<Shape> aVEdges = theSweep.Sections()!;

        WireExplorer wexp2 = new(mySpine);
        inde = 0;
        bool ToExit = false;
        for (;;)
        {
            if (!wexp2.More())
            {
                ToExit = true;
            }

            inde++;

            if (!ToExit)
            {
                Edge anEdgeOfSpine = wexp2.Current();

                NCollection_List<Shape> Flist = myGenMap.Bound(anEdgeOfSpine, new NCollection_List<Shape>());

                for (int i = 1; i <= aFaces.UpperRow(); i++)
                {
                    Shape aFace = aFaces.Value(i, inde);
                    if (aFace.ShapeType() == ShapeType.Face)
                    {
                        Flist.Append(aFace);
                    }
                }
            }

            Vertex aVertexOfSpine = wexp2.CurrentVertex();
            NCollection_List<Shape> ListVshapes = myGenMap.Bound(aVertexOfSpine, new NCollection_List<Shape>());
            for (int i = 1; i <= aVEdges.UpperRow(); i++)
            {
                Shape aVshape = aVEdges.Value(i, inde);
                if (aVshape.IsNull)
                {
                    continue;
                }
                if (aVshape.ShapeType() == ShapeType.Edge || aVshape.ShapeType() == ShapeType.Face)
                {
                    ListVshapes.Append(aVshape);
                }
                else
                {
                    Iterator itvshape = new(aVshape);
                    for (; itvshape.More(); itvshape.Next())
                    {
                        Shape aSubshape = itvshape.Value();
                        if (aSubshape.ShapeType() == ShapeType.Edge || aSubshape.ShapeType() == ShapeType.Face)
                        {
                            ListVshapes.Append(aSubshape);
                        }
                        else
                        {
                            // it is wire
                            for (itw.Initialize(aSubshape); itw.More(); itw.Next())
                            {
                                ListVshapes.Append(itw.Value());
                            }
                        }
                    }
                }
            }

            if (ToExit)
            {
                break;
            }

            if (wexp2.More())
            {
                wexp2.Next();
            }
        }
    }

    //=================================================================================================

    private static bool BuildBoundaries(BRepFill_Sweep theSweep,
                                        BRepFill_SectionLaw theSection,
                                        ref Shape theBottom,
                                        ref Shape theTop)
    {
        Wire aBottomWire = new();
        Wire aTopWire = new();
        BrepBuilder aB = new();
        aB.MakeWire(ref aBottomWire);
        aB.MakeWire(ref aTopWire);
        bool bfoundbottom = false;
        bool bfoundtop = false;
        NCollection_HArray2<Shape> aVEdges = theSweep.Sections()!;
        int i = 0;
        bool bAllSame = true;

        for (i = 1; i <= theSection.NbLaw(); i++)
        {
            Shape aBottomEdge = aVEdges.Value(i, aVEdges.LowerCol());

            if (!aBottomEdge.IsNull && (aBottomEdge.ShapeType() == ShapeType.Edge))
            {
                aB.Add(aBottomWire, aBottomEdge);
                bfoundbottom = true;
            }
            Shape aTopEdge = aVEdges.Value(i, aVEdges.UpperCol());

            if (!aTopEdge.IsNull && (aTopEdge.ShapeType() == ShapeType.Edge))
            {
                aB.Add(aTopWire, aTopEdge);
                bfoundtop = true;
            }

            if (!aBottomEdge.IsNull && !aTopEdge.IsNull && !aBottomEdge.IsSame(aTopEdge))
            {
                bAllSame = false;
            }
        }

        if (theSection.IsUClosed())
        {
            aBottomWire.Closed(true);
            aTopWire.Closed(true);
        }

        if (bfoundbottom)
        {
            theBottom = aBottomWire;
        }

        if (bfoundtop)
        {
            theTop = aTopWire;
        }

        if (bAllSame && bfoundbottom && bfoundtop)
        {
            theTop = theBottom;
        }

        return bfoundbottom || bfoundtop;
    }
}
