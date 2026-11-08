// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_PipeShell.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_PipeShell.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Classification;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Laws;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Construction.Surfaces;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Construction.Filling;

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
internal sealed class PipeShell : Transient
{
    private Wire _spine;
    private Shape _first = new();
    private Shape _last = new();
    private Shape _shape = new();
    private readonly NCollection_Sequence<FillingSection> _seq = new();
    private readonly NCollection_Sequence<Shape> _wSeq = new();
    private readonly NCollection_Sequence<int> _indOfSec = new();
    private readonly Dictionary<Shape, NCollection_List<Shape>> _edgeNewEdges = new(ShapeHasher.Instance);
    private readonly Dictionary<Shape, NCollection_List<Shape>> _genMap = new(ShapeHasher.Instance);
    private double _tol3d;
    private double _boundTol;
    private double _tolAngular;
    private double _angmin; // OCCT: uninitialised until SetTransition (MakePipeShell's constructor calls it)
    private double _angmax;
    private int _maxDegree;
    private int _maxSegments;
    private bool _forceApproxC1;
    private LawsFunction? _law;
    private bool _isAutomaticLaw;
    private LocationLaw? _location;
    private SectionLaw? _section;
    private Trihedron _trihedron;
    private TransitionStyle _transition;
    private SurfacePipeError _status;
    private double _errorOnSurf; // OCCT: uninitialised until a successful Build
    private bool _isBuildHistory;

    //=======================================================================
    // function :  ComputeSection
    // purpose  : Construct an intermediary section
    //=======================================================================

    private static bool ComputeSection(in Wire w1, in Wire w2, double p1, double p2, out Wire wres)
    {
        NCollection_Sequence<double> sr = new();
        NCollection_Sequence<Shape> sSh = new();
        sr.Clear();
        sr.Append(0.0);
        sr.Append(1.0);
        sSh.Clear();
        sSh.Append(w1);
        sSh.Append(w2);
        CompatibleWires cw = new(sSh);
        cw.SetPercent(0.1);
        cw.Perform();
        if (!cw.IsDone)
        {
            throw new NotDoneException("Uncompatible wires");
        }
        NCollection_Sequence<Transform3D> emptyTrsfs = new();
        NSections sl = new(cw.InternalShape(), emptyTrsfs, sr, 0.0, 1.0);
        double us = p1 / (p1 + p2);
        Shape w = new();
        sl.D0(us, ref w);
        wres = ShapeCasts.Wire(w);
        return true;
    }

    //=======================================================================
    // function : PerformTransition
    // purpose  : Modify a law of location depending on Transition
    //=======================================================================

    private static void PerformTransition(TransitionStyle mode, LocationLaw? loc, double angmin)
    {
        if (loc is not null)
        {
            loc.DeleteTransform();
            if (mode == TransitionStyle.Modified)
            {
                loc.TransformInG0Law();
            }
            else
            {
                loc.TransformInCompatibleLaw(angmin);
            }
        }
    }

    //=======================================================================
    // function :  PerformPlan
    // purpose  : Construct a plane of filling if exists
    //=======================================================================

    private static bool PerformPlan(ref Shape s)
    {
        bool isDegen = true;
        Explorer explo = new(s, ShapeType.Edge);
        for (; explo.More(); explo.Next())
        {
            Edge edge = ShapeCasts.Edge(explo.Current());
            if (!Tool.Degenerated(edge))
            {
                isDegen = false;
            }
        }
        if (isDegen)
        {
            s.Nullify();
            return true;
        }

        Wire w = ShapeCasts.Wire(s);
        bool ok = false;
        if (!w.IsNull)
        {
            LibraryMakeFace mkplan = new(w, true);
            if (mkplan.IsDone)
            {
                s = mkplan.InternalFace();
                ok = true;
            }
        }
        return ok;
    }

    //=================================================================================================

    private static bool IsSameOriented(in Shape face, in Shape shell)
    {
        IndexedMap<Shape, NCollection_List<Shape>> edgeFaceMap = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(shell, ShapeType.Edge, ShapeType.Face, edgeFaceMap);

        for (Explorer faceExplorer = new(face, ShapeType.Edge); faceExplorer.More(); faceExplorer.Next())
        {
            Shape currentEdge = faceExplorer.Current();
            if (!edgeFaceMap.ContainsKey(currentEdge))
            {
                continue;
            }

            Orientation currentEdgeOrientation = currentEdge.Orientation;
            Shape adjFace = edgeFaceMap.GetValue(currentEdge).First();
            for (Explorer adjFaceExplorer = new(adjFace, ShapeType.Edge); adjFaceExplorer.More(); adjFaceExplorer.Next())
            {
                // IsSame compares TShape and Location only; orientation may differ.
                Shape adjEdge = adjFaceExplorer.Current();
                if (adjEdge.IsSame(currentEdge))
                {
                    return currentEdgeOrientation != adjEdge.Orientation;
                }
            }
        }

        // No shared edge found. Orientation cannot be determined reliably and this
        // should not happen for valid input. Preserve the current face orientation
        // instead of reporting an opposite orientation and forcing a reversal.
        return true;
    }

    //=================================================================================================

    /// <summary>A pipe shell along <paramref name="spine"/>, in corrected Frenet mode until another mode is set.</summary>
    public PipeShell(in Wire spine)
    {
        _spine = spine;
        _forceApproxC1 = false;
        _isAutomaticLaw = false;
        _trihedron = Trihedron.IsCorrectedFrenet;
        _transition = TransitionStyle.Modified;
        _status = SurfacePipeError.PipeOk;
        _isBuildHistory = true;
        _location = null;
        _section = null;
        _law = null;
        SetTolerance();

        _maxDegree = 11;
        _maxSegments = 100;

        // Attention to closed non-declared wire !
        if (!_spine.ClosedFlag)
        {
            Exploration.Vertices(_spine, out Vertex vf, out Vertex vl);
            if (vf.IsSame(vl))
            {
                if (_spine.TShape()!.IsFrozen)
                {
                    // Kerf: OCCT sets the flag on the caller's wire; the port on a new wire of the same edges
                    // (decision 926)
                    BrepBuilder b = new();
                    Wire wire = new();
                    b.MakeWire(ref wire);
                    for (Iterator it = new(_spine); it.More(); it.Next())
                    {
                        b.Add(wire, it.Value());
                    }
                    _spine = wire;
                }
                _spine.Closed(true);
            }
        }
    }

    //=======================================================================
    // function : Set
    // purpose  : Define a law of Frenet (Correct)
    //=======================================================================
    /// <summary>The Frenet trihedron (<paramref name="isFrenet"/>) or the corrected Frenet trihedron.</summary>
    public void Set(bool isFrenet = false)
    {
        TrihedronLaw tLaw;
        if (isFrenet)
        {
            _trihedron = Trihedron.IsFrenet;
            tLaw = new Frenet();
        }
        else
        {
            _trihedron = Trihedron.IsFrenet; // OCCT
            tLaw = new CorrectedFrenet();
        }
        CurveAndTrihedron loc = new(tLaw);
        _location = new Edge3DLaw(_spine, loc);
        _section = null; // It is required to relocalize sections.
    }

    //=======================================================================
    // function : SetDiscrete
    // purpose  : Define a law of Discrete Trihedron
    //=======================================================================
    /// <summary>The discrete trihedron.</summary>
    public void SetDiscrete()
    {
        TrihedronLaw tLaw;

        _trihedron = Trihedron.IsDiscreteTrihedron;
        tLaw = new DiscreteTrihedron();

        CurveAndTrihedron loc = new(tLaw);
        _location = new Edge3DLaw(_spine, loc);
        _section = null; // It is required to relocalize sections.
    }

    //=================================================================================================

    /// <summary>A fixed trihedron: the sections keep the orientation they have relative to <paramref name="axe"/>.</summary>
    public void Set(in Frame3D axe)
    {
        _trihedron = Trihedron.IsFixed;
        Vector3D v1 = new(), v2 = new();
        v1 = v1.WithXYZ(axe.Direction.XYZ());
        v2 = v2.WithXYZ(axe.XDirection.XYZ());
        Fixed tLaw = new(v1, v2);
        CurveAndTrihedron loc = new(tLaw);
        _location = new Edge3DLaw(_spine, loc);
        _section = null; // It is required to relocalize sections.
    }

    //=======================================================================
    // function : Set
    // purpose  : Construct a law of location of binormal fixed type
    //=======================================================================
    /// <summary>A trihedron with the constant binormal <paramref name="biNormal"/>.</summary>
    public void Set(in Direction3D biNormal)
    {
        _trihedron = Trihedron.IsConstantNormal;

        ConstantBiNormal tLaw = new(biNormal);
        CurveAndTrihedron loc = new(tLaw);
        _location = new Edge3DLaw(_spine, loc);
        _section = null; // Il faut relocaliser les sections.
    }

    //=======================================================================
    // function : Set
    // purpose  : Construct a law of location of Darboux type
    //=======================================================================
    /// <summary>
    /// The Darboux trihedron, the normal being the normal of <paramref name="spineSupport"/> (a shape whose faces hold
    /// the spine's edges). False if the spine isn't on it.
    /// </summary>
    public bool Set(in Shape spineSupport)
    {
        bool b;

        // A special law of location is required
        EdgeOnSurfLaw loc = new(_spine, spineSupport);
        b = loc.HasResult;
        if (b)
        {
            _location = loc;
            _trihedron = Trihedron.IsDarboux;
            _section = null; // It is required to relocalize the sections.
        }
        return b;
    }

    //=======================================================================
    // function : Set
    // purpose  : Defines a lawv with help of a guided contour
    //=======================================================================
    /// <summary>
    /// A trihedron guided by <paramref name="auxiliarySpine"/>: by the reduced curvilinear abscissa
    /// (<paramref name="curvilinearEquivalence"/>) or by the plane normal to the spine, with or without contact
    /// (<paramref name="keepContact"/>).
    /// </summary>
    public void Set(in Wire auxiliarySpine,
                    bool curvilinearEquivalence = true,
                    TypeOfContact keepContact = TypeOfContact.NoContact)
    {
        // Reorganization of the guide (pb of orientation and origin)
        Wire theGuide;
        theGuide = auxiliarySpine;
        bool spClose = _spine.ClosedFlag, guideClose = auxiliarySpine.ClosedFlag;

        if (keepContact == TypeOfContact.ContactOnBorder)
        {
            _isAutomaticLaw = true;
        }

        if (!spClose && !guideClose)
        {
            // Case open reorientation of the guide
            Wire sp = _spine;
            NCollection_Sequence<Shape> seq = new();
            seq.Append(sp);
            seq.Append(theGuide);
            CompatibleWires cw = new(seq);
            cw.SetPercent(0.1);
            cw.Perform();
            if (!cw.IsDone)
            {
                throw new NotDoneException("Uncompatible wires");
            }
            theGuide = ShapeCasts.Wire(cw.InternalShape().Value(2));
        }
        else if (guideClose)
        {
            // Case guide closed : Determination of the origin
            // & reorientation of the guide
            Vector3D dir;
            Point3D spOr;
            if (!spClose)
            {
                Exploration.Vertices(_spine, out Vertex vf, out Vertex vl);
                spOr = Tool.Pnt(vf);
                Point3D p = Tool.Pnt(vl);
                Vector3D v = new(p, spOr);
                spOr = spOr.BaryCentered(0.5, p, 0.5);
                dir = v;
            }
            else
            {
                WireCurveAdaptor bc = new(_spine);
                bc.D1(0, out spOr, out dir);
            }
            Fill.SearchOrigin(ref theGuide, spOr, dir, 100 * _tol3d);
        }

        // transform the guide in a single curve
        WireCurveAdaptor guide = new(theGuide);

        if (curvilinearEquivalence)
        { // trihedron by curvilinear reduced abscissa
            if (keepContact == TypeOfContact.Contact || keepContact == TypeOfContact.ContactOnBorder)
            {
                _trihedron = Trihedron.IsGuideACWithContact; // with rotation
            }
            else
            {
                _trihedron = Trihedron.IsGuideAC; // without rotation
            }

            GuideTrihedronAC tLaw = new(guide);
            LocationGuide loc = new(tLaw);
            _location = new AcrLaw(_spine, loc);
        }
        else
        { // trihedron by plane
            if (keepContact == TypeOfContact.Contact || keepContact == TypeOfContact.ContactOnBorder)
            {
                _trihedron = Trihedron.IsGuidePlanWithContact; // with rotation
            }
            else
            {
                _trihedron = Trihedron.IsGuidePlan; // without rotation
            }

            GuideTrihedronPlan tLaw = new(guide);
            LocationGuide loc = new(tLaw);
            _location = new Edge3DLaw(_spine, loc);
        }
        _section = null; // It is required to relocalize the sections.
    }

    //=================================================================================================

    /// <summary>The maximal degree of the swept surfaces (11 by default).</summary>
    public void SetMaxDegree(int newMaxDegree)
    {
        _maxDegree = newMaxDegree;
    }

    //=================================================================================================

    /// <summary>The maximal number of segments of the swept surfaces (100 by default).</summary>
    public void SetMaxSegments(int newMaxSegments)
    {
        _maxSegments = newMaxSegments;
    }

    //=======================================================================
    // function : SetForceApproxC1
    // purpose  : Set the flag that indicates attempt to approximate
    //           a C1-continuous surface if a swept surface proved
    //           to be C0.
    //=======================================================================
    /// <summary>Tries to approximate C1 surfaces where a swept surface is C0.</summary>
    public void SetForceApproxC1(bool forceApproxC1)
    {
        _forceApproxC1 = forceApproxC1;
    }

    /// <summary>Whether <see cref="Build"/> builds the history (true by default).</summary>
    public void SetIsBuildHistory(bool isBuildHistory)
    {
        _isBuildHistory = isBuildHistory;
    }

    /// <summary>True if <see cref="Build"/> builds the history.</summary>
    public bool IsBuildHistory => _isBuildHistory;

    //=================================================================================================

    /// <summary>
    /// Adds the section <paramref name="profile"/> (a wire or a vertex), placed where it is, translated onto the
    /// spine with <paramref name="withContact"/>, and rotated to be orthogonal to it with
    /// <paramref name="withCorrection"/>.
    /// </summary>
    public void Add(in Shape profile, bool withContact = false, bool withCorrection = false)
    {
        Vertex v = new();
        v.Nullify();
        Add(profile, v, withContact, withCorrection);
        ResetLoc();
    }

    //=================================================================================================

    /// <summary>
    /// Adds the section <paramref name="profile"/> placed at the spine's vertex <paramref name="location"/>.
    /// </summary>
    public void Add(in Shape profile, in Vertex location, bool withContact = false, bool withCorrection = false)
    {
        DeleteProfile(profile); // No duplication
        if (_isAutomaticLaw)
        {
            _seq.Clear();
            FillingSection s = new(profile, location, withContact, withCorrection);
            s.Set(true);
            _seq.Append(s);
            _section = null;
            ResetLoc();

            LocationGuide loc = (LocationGuide)_location!.Law(1);
            loc.ComputeAutomaticLaw(out NCollection_HArray1<Point2D> parAndRad);

            // Compuite initial width of section (this will be 1.)
            Properties globalProps = new();
            MassPropertiesBuilder.LinearProperties(profile, globalProps);
            Point3D baryCenter = globalProps.CentreOfMass();

            Face profileFace = new LibraryMakeFace(ShapeCasts.Wire(profile), true).InternalFace(); // only plane
            Surface thePlane = Tool.Surface(profileFace)!;
            SurfaceAdaptor gaHplane = new(thePlane);
            HInter intersector = new();
            CurveAdaptorBase[] hCurve = new CurveAdaptorBase[2];
            hCurve[0] = loc.GetCurve()!;
            hCurve[1] = loc.Guide();
            Point3D[] pointsOnSpines = [new(), new()];
            int i, j;

            for (i = 0; i < 2; i++)
            {
                intersector.Perform(hCurve[i], gaHplane);
                double minDist = RealLast();
                for (j = 1; j <= intersector.PointCount; j++)
                {
                    Point3D pint = intersector.Point(j).Pnt();
                    double dist = baryCenter.DistanceTo(pint);
                    if (dist < minDist)
                    {
                        minDist = dist;
                        pointsOnSpines[i] = pint;
                    }
                }
            }

            // Correct <ParAndRad> according to <InitialWidth>
            double initialWidth = pointsOnSpines[0].DistanceTo(pointsOnSpines[1]);
            int nbParRad = parAndRad.Upper();
            for (i = 1; i <= nbParRad; i++)
            {
                Point2D parRad = parAndRad.Value(i);
                parRad = parRad.WithY(parRad.Y / initialWidth);
                parAndRad.SetValue(i, parRad);
            }

            _law = new Interpol();

            bool isPeriodic = (Math.Abs(parAndRad.Value(1).Y - parAndRad.Value(nbParRad).Y) < global::Kerf.Engine.Foundation.Precision.Confusion());

            ((Interpol)_law).Set(parAndRad.Array1(), isPeriodic);
        }
        else
        {
            FillingSection s = new(profile, location, withContact, withCorrection);
            _seq.Append(s);
            _section = null;
            ResetLoc();
        }
    }

    //=======================================================================
    // function : SetLaw
    // purpose  : Section + law of homothety
    //=======================================================================
    /// <summary>The only section <paramref name="profile"/>, scaled along the spine by the law <paramref name="l"/>.</summary>
    public void SetLaw(in Shape profile, LawsFunction l, bool withContact = false, bool withCorrection = false)
    {
        Vertex v = new();
        v.Nullify();
        SetLaw(profile, l, v, withContact, withCorrection);
        ResetLoc();
    }

    //=======================================================================
    // function : SetLaw
    // purpose  :  Section + Law of homothety
    //=======================================================================
    /// <summary>
    /// The only section <paramref name="profile"/>, placed at <paramref name="location"/> and scaled along the spine by
    /// the law <paramref name="l"/>.
    /// </summary>
    public void SetLaw(in Shape profile,
                       LawsFunction l,
                       in Vertex location,
                       bool withContact = false,
                       bool withCorrection = false)
    {
        _seq.Clear();
        FillingSection s = new(profile, location, withContact, withCorrection);
        s.Set(true);
        _seq.Append(s);
        _law = l;
        _section = null;
        ResetLoc();
    }

    //=================================================================================================

    /// <summary>Removes the section <paramref name="profile"/>.</summary>
    public void DeleteProfile(in Shape profile)
    {
        bool trouve = false;
        int ii;
        for (ii = 1; ii <= _seq.Length() && !trouve; ii++)
        {
            Shape section = _seq.Value(ii).OriginalShape();
            if (profile.IsSame(section))
            {
                trouve = true;
                _seq.Remove(ii);
            }
        }

        if (trouve)
        {
            _section = null;
        }
        ResetLoc();
    }

    //=================================================================================================

    /// <summary>True if a section was added.</summary>
    public bool IsReady => (_seq.Length() != 0);

    //=================================================================================================

    /// <summary>The status of the last build.</summary>
    public SurfacePipeError GetStatus() => _status;

    //=================================================================================================

    /// <summary>The tolerances: 3D, of the bounds, and angular.</summary>
    public void SetTolerance(double tol3d = 1.0e-4, double boundTol = 1.0e-4, double tolAngular = 1.0e-2)
    {
        _tol3d = tol3d;
        _boundTol = boundTol;
        _tolAngular = tolAngular;
    }

    //=======================================================================
    // function : SetTransition
    // purpose  : Defines the mode of processing of corners
    //=======================================================================
    /// <summary>The treatment of the spine's corners, and the angles bounding it.</summary>
    public void SetTransition(TransitionStyle mode = TransitionStyle.Modified, double angmin = 1.0e-2, double angmax = 6.0)
    {
        if (_transition != mode)
        {
            _section = null; // It is required to relocalize the sections.
        }
        _transition = mode;
        _angmin = angmin;
        _angmax = angmax;
    }

    //=================================================================================================

    /// <summary>
    /// Fills <paramref name="list"/> with <paramref name="n"/> sections placed along the spine, without building the
    /// sweep (frozen).
    /// </summary>
    public void Simulate(int n, NCollection_List<Shape> list)
    {
        // Preparation
        Prepare();
        list.Clear();

        double first, last, length, delta, u, us, deltaS, firstS;
        int ii, nbL = _location!.LawCount;
        bool finis = false;
        Shape w = new();

        // Calculate the parameters of digitalization
        _section!.Law(1).GetDomain(out firstS, out last);
        deltaS = last - firstS;
        _location.CurvilinearBounds(nbL, out first, out length);
        delta = length;
        if (n > 1)
        {
            delta /= (n - 1);
        }

        _location.CurvilinearBounds(1, out first, out last); // Initiation of Last
        for (u = 0.0, ii = 1; !finis; u += delta)
        {
            if (u >= length)
            {
                u = length;
                finis = true;
            }
            else
            {
                if (ii < nbL)
                {
                    _location.CurvilinearBounds(nbL, out first, out last);
                }
                if (u > last)
                {
                    u = (last + first) / 2; // The edge is not skipped
                }
                if (u > first)
                {
                    ii++;
                }
            }
            us = firstS + (u / length) * deltaS;
            // Calcul d'une section
            _section.D0(us, ref w);
            _location.D0(u, ref w);
            list.Append(w);
        }
    }

    //=======================================================================
    // function : Build
    // purpose  : Construct the Shell and the history
    //=======================================================================
    /// <summary>Builds the shell and its history. False if it fails (see <see cref="GetStatus"/>).</summary>
    public bool Build()
    {
        bool ok;
        // 1) Preparation
        Prepare();

        if (_status != SurfacePipeError.PipeOk)
        {
            BrepBuilder b = new();
            Shell sh = new();
            b.MakeShell(ref sh);
            _shape = sh; // Nullify
            return false;
        }

        // 2) Calculate myFirst and myLast
        _section!.Law(1).GetDomain(out double firstS, out double lastS);
        _section.D0(firstS, ref _first);
        _location!.InternalD0(0, ref _first);
        if (_section.IsVClosed && _location.IsClosed())
        {
            if (_location.IsG1(0) >= 0)
            {
                _last = _first;
            }
            else
            {
                _first.Nullify();
                _last.Nullify();
            }
        }
        else
        {
            _location.CurvilinearBounds(_location.LawCount, out firstS, out double length);
            _section.D0(lastS, ref _last);
            _location.InternalD0(length, ref _last);
            // eap 5 Jun 2002 occ332, myLast and myFirst must not share one TShape,
            // tolerances of shapes built on them may be quite different
            if (_first.IsPartner(_last))
            {
                Copy copy = new(_last);
                if (copy.IsDone)
                {
                    _last = copy.InternalShape();
                }
            }
            // eap 5 Jun 2002 occ332, end modif
        }

        // 3) Construction
        Sweep mkSw = new(_section, _location, true);
        mkSw.SetTolerance(_tol3d, _boundTol, 1.0e-5, _tolAngular);
        mkSw.SetAngularControl(_angmin, _angmax);
        mkSw.SetForceApproxC1(_forceApproxC1);
        mkSw.SetBounds(ShapeCasts.Wire(_first), ShapeCasts.Wire(_last));

        Continuity theContinuity = C2;
        if (_trihedron == Trihedron.IsDiscreteTrihedron)
        {
            theContinuity = C0;
        }
        HashSet<Shape> dummy = new(ShapeHasher.Instance);
        Dictionary<Shape, NCollection_HArray2<Shape>> dummy2 = new(ShapeHasher.Instance);
        Dictionary<Shape, NCollection_HArray2<Shape>> dummy3 = new(ShapeHasher.Instance);
        mkSw.Build(dummy, dummy2, dummy3, _transition, theContinuity, ApproxStyle.Location, _maxDegree, _maxSegments);

        _status = _location.GetStatus();
        ok = (mkSw.IsDone && (_status == SurfacePipeError.PipeOk));

        if (ok)
        {
            _shape = mkSw.Shape();
            _errorOnSurf = mkSw.ErrorOnSurface();

            Shape bottomWire = _first;
            Shape topWire = _last;

            if (BuildBoundaries(mkSw, _section, ref bottomWire, ref topWire))
            {
                _first = bottomWire;
                _last = topWire;
            }

            if (_section.IsUClosed)
            {
                Explorer explo = new();
                bool degenFirst = true, degenLast = true;

                for (explo.Init(_first, ShapeType.Edge); explo.More(); explo.Next())
                {
                    Edge edge = ShapeCasts.Edge(explo.Current());
                    degenFirst = degenFirst && Tool.Degenerated(edge);
                }

                for (explo.Init(_last, ShapeType.Edge); explo.More(); explo.Next())
                {
                    Edge edge = ShapeCasts.Edge(explo.Current());
                    degenLast = degenLast && Tool.Degenerated(edge);
                }

                if (degenFirst && degenLast)
                {
                    _shape.Closed(true);
                }
            }

            if (_isBuildHistory)
            {
                BuildHistory(mkSw);
            }
        }
        else
        {
            BrepBuilder b = new();
            Shell sh = new();
            b.MakeShell(ref sh);
            _shape = sh; // Nullify
            if (_status == SurfacePipeError.PipeOk)
            {
                _status = SurfacePipeError.PipeNotOk;
            }
        }
        return ok;
    }

    //=================================================================================================

    /// <summary>
    /// Closes the shell with planar faces at its ends and makes a solid of it. False if it can't.
    /// </summary>
    /// <exception cref="NotDoneException">The shell isn't built.</exception>
    public bool MakeSolid()
    {
        if (_shape.IsNull)
        {
            throw new NotDoneException("PipeShell is not built");
        }
        bool b = _shape.ClosedFlag;
        BrepBuilder bs = new();

        if (!b)
        {
            if (!_first.IsNull && !_last.IsNull)
            {
                b = (_first.ClosedFlag && _last.ClosedFlag);
            }
            if (b)
            {
                // It is necessary to block the extremities
                b = PerformPlan(ref _first);
                if (b)
                {
                    b = PerformPlan(ref _last);
                    if (b)
                    {
                        if (_shape.TShape()!.IsFrozen)
                        {
                            // Kerf: OCCT adds the end faces to the shell the caller may already hold (Shape()); a
                            // published shell is frozen, so the port adds them to a new shell of the same faces
                            // (decision 926)
                            Shape shell = _shape.EmptyCopied();
                            for (Iterator it = new(_shape, false, false); it.More(); it.Next())
                            {
                                bs.Add(shell, it.Value());
                            }
                            _shape = shell;
                        }
                        if (!_first.IsNull && !IsSameOriented(_first, _shape))
                        {
                            _first.Reverse();
                        }
                        if (!_last.IsNull && !IsSameOriented(_last, _shape))
                        {
                            _last.Reverse();
                        }

                        if (!_first.IsNull)
                        {
                            bs.Add(_shape, ShapeCasts.Face(_first));
                        }
                        if (!_last.IsNull)
                        {
                            bs.Add(_shape, ShapeCasts.Face(_last));
                        }

                        _shape.Closed(true);
                    }
                }
            }
        }

        if (b)
        {
            Solid solid = new();
            bs.MakeSolid(ref solid);
            bs.Add(solid, ShapeCasts.Shell(_shape));
            SolidClassifier sc = new(solid);
            sc.PerformInfinitePoint(global::Kerf.Engine.Foundation.Precision.Confusion());
            if (sc.State() == In)
            {
                bs.MakeSolid(ref solid);
                _shape.Reverse();
                bs.Add(solid, ShapeCasts.Shell(_shape));
            }
            _shape = solid;
            _shape.Closed(true);
        }
        return b;
    }

    //=================================================================================================

    private static Shape Published(Shape shape)
    {
        shape.TShape()?.FreezeGraph();
        return shape;
    }

    /// <summary>The shell or solid built (frozen).</summary>
    public Shape Shape() => Published(_shape);

    /// <summary><see cref="Shape"/> without freezing, for ported callers.</summary>
    internal Shape InternalShape() => _shape;

    //=================================================================================================

    /// <summary>The approximation error of the swept surfaces.</summary>
    public double ErrorOnSurface() => _errorOnSurf;

    //=================================================================================================

    /// <summary>The shape at the start of the sweep (frozen).</summary>
    public Shape FirstShape() => Published(_first);

    /// <summary><see cref="FirstShape"/> without freezing, for ported callers.</summary>
    internal Shape InternalFirstShape() => _first;

    //=================================================================================================

    /// <summary>The shape at the end of the sweep (frozen).</summary>
    public Shape LastShape() => Published(_last);

    /// <summary><see cref="LastShape"/> without freezing, for ported callers.</summary>
    internal Shape InternalLastShape() => _last;

    /// <summary>Appends the profiles, as given, to <paramref name="profiles"/>.</summary>
    public void Profiles(NCollection_List<Shape> profiles)
    {
        for (int i = 1; i <= _seq.Length(); ++i)
        {
            profiles.Append(_seq[i].OriginalShape());
        }
    }

    /// <summary>The spine (frozen; a new wire of the caller's edges when the port had to close it).</summary>
    public Wire Spine() => ShapeCasts.Wire(Published(_spine));

    //=================================================================================================

    /// <summary>
    /// Fills <paramref name="list"/> with the shapes generated from <paramref name="shape"/>, a profile or one
    /// of its edges or vertices, or an edge or vertex of the spine (frozen).
    /// </summary>
    public void Generated(in Shape shape, NCollection_List<Shape> list)
    {
        InternalGenerated(shape, list);
        for (NCollection_List<Shape>.Iterator it = new(list); it.More(); it.Next())
        {
            it.Value().TShape()?.FreezeGraph();
        }
    }

    /// <summary><see cref="Generated"/> without freezing, for ported callers.</summary>
    internal void InternalGenerated(in Shape shape, NCollection_List<Shape> list)
    {
        list.Clear();

        if (_genMap.ContainsKey(shape))
        {
            list.Assign(_genMap.GetValue(shape));
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
        _wSeq.Clear();
        _edgeNewEdges.Clear(); // OCCT: myIndOfSec isn't cleared, so a second Prepare appends to it

        Wire theSect;
        if (!IsReady)
        {
            throw new NotDoneException("PipeShell");
        }
        if (_location is not null && _section is not null)
        {
            return; // It is ready
        }

        // Check set of section for right configuration of punctual sections
        int i;
        Iterator iter = new();
        for (i = 2; i <= _seq.Length() - 1; i++)
        {
            bool wdeg = true;
            for (iter.Initialize(_seq[i].InternalWire()); iter.More(); iter.Next())
            {
                Edge edge = ShapeCasts.Edge(iter.Value());
                wdeg = wdeg && (Tool.Degenerated(edge));
            }
            if (wdeg)
            {
                throw new FailureException("Wrong usage of punctual sections");
            }
        }
        if (_seq.Length() <= 2)
        {
            bool wdeg = true;
            for (i = 1; i <= _seq.Length(); i++)
            {
                for (iter.Initialize(_seq[i].InternalWire()); iter.More(); iter.Next())
                {
                    Edge edge = ShapeCasts.Edge(iter.Value());
                    wdeg = wdeg && (Tool.Degenerated(edge));
                }
            }
            if (wdeg)
            {
                throw new FailureException("Wrong usage of punctual sections");
            }
        }

        // Construction of the law of location
        if (_location is null)
        {
            switch (_trihedron)
            {
                case Trihedron.IsCorrectedFrenet:
                {
                    TrihedronLaw tLaw = new CorrectedFrenet();
                    CurveAndTrihedron loc = new(tLaw);
                    _location = new Edge3DLaw(_spine, loc);
                    break;
                }
                default:
                { // Not planned!
                    throw new ConstructionErrorException("PipeShell");
                }
            }
        }

        // Transformation of the law (Transition Management)
        PerformTransition(_transition, _location, _angmin);

        // Construction of the section law
        if (_seq.Length() == 1)
        {
            Place(_seq[1], out theSect, out _, out _);
            Wire localShape = theSect;
            if (_seq[1].IsLaw)
            {
                _section = new ShapeLaw(localShape, _law!);
            }
            else
            {
                _section = new ShapeLaw(localShape);
            }

            _wSeq.Append(theSect);
            // Simple case of single section
            _indOfSec.Append(1);
            Iterator itw = new(theSect);
            for (; itw.More(); itw.Next())
            {
                Shape edge = itw.Value();
                NCollection_List<Shape> elist = new();
                elist.Append(edge);
                _edgeNewEdges[edge] = elist;
            }
            ///////////////////////////////
        }
        else
        {
            NCollection_Sequence<double> Param = new();
            NCollection_Sequence<int> indSec = new();
            NCollection_Sequence<Transform3D> transformations = new();
            int nbL = _location.LawCount;
            double v1, v2;
            _location.CurvilinearBounds(nbL, out v1, out v2);
            v1 = 0.0;
            int ideb = 0, ifin = 0;
            int iseq;
            for (iseq = 1; iseq <= _seq.Length(); iseq++)
            {
                indSec.Append(iseq);
                Place(_seq[iseq], out theSect, out Transform3D trsf, out double param);
                Param.Append(param);
                _wSeq.Append(theSect);
                transformations.Append(trsf);
                if (param == v1)
                {
                    ideb = iseq;
                }
                if (param == v2)
                {
                    ifin = iseq;
                }
            }

            // looping sections ?
            if (_location.IsClosed())
            {
                if (ideb > 0)
                {
                    // place the initial section at the final position
                    Param.Append(v2);
                    _wSeq.Append(_wSeq[ideb]);
                }
                else if (ifin > 0)
                {
                    // place the final section at the initial position
                    Param.Append(v1);
                    _wSeq.Append(_wSeq[ifin]);
                }
                else
                {
                    // it is necessary to find a medium section to impose by V1 and by V2
                    double pmin = RealLast(), pmax = RealFirst();
                    Wire wmin = new(), wmax = new();
                    for (iseq = 1; iseq <= _wSeq.Length(); iseq++)
                    {
                        if (Param.Value(iseq) < pmin)
                        {
                            pmin = Param.Value(iseq);
                            wmin = ShapeCasts.Wire(_wSeq.Value(iseq));
                        }
                        if (Param.Value(iseq) > pmax)
                        {
                            pmax = Param.Value(iseq);
                            wmax = ShapeCasts.Wire(_wSeq.Value(iseq));
                        }
                    }
                    // medium section between Wmin and Wmax
                    double dmin = Math.Abs(pmin - v1);
                    double dmax = Math.Abs(pmax - v2);
                    if (ComputeSection(wmin, wmax, dmin, dmax, out Wire wres))
                    {
                        // impose section Wres at the beginning and the end
                        Param.Append(v1);
                        _wSeq.Append(wres);
                        indSec.Append(_wSeq.Length());
                        Param.Append(v2);
                        _wSeq.Append(wres);
                        indSec.Append(_wSeq.Length());
                    }
                }
            }

            // parse sections by increasing parameter
            bool play_again = true;
            while (play_again)
            {
                play_again = false;
                for (iseq = 1; iseq <= _wSeq.Length(); iseq++)
                {
                    for (int jseq = iseq + 1; jseq <= _wSeq.Length(); jseq++)
                    {
                        if (Param.Value(iseq) > Param.Value(jseq))
                        {
                            Param.Exchange(iseq, jseq);
                            _wSeq.Exchange(iseq, jseq);
                            indSec.Exchange(iseq, jseq);
                            play_again = true;
                        }
                    }
                }
            }
            // Fill the array of real indices of sections
            for (int ii = 1; ii <= _seq.Length(); ii++)
            {
                for (int jj = 1; jj <= indSec.Length(); jj++)
                {
                    if (indSec[jj] == ii)
                    {
                        _indOfSec.Append(jj);
                        break;
                    }
                }
            }

            //  Calculate work sections
            NCollection_Sequence<Shape> workingSections = new();
            workingSections.Clear();
            CompatibleWires georges = new(_wSeq);
            georges.SetPercent(0.1);
            georges.Perform(false);
            if (georges.IsDone)
            {
                workingSections.Assign(georges.InternalShape());
                // (OCCT copies Georges.Generated() into a WorkingMap it never reads.)
                // For each sub-edge of each section
                // we save its splits
                for (int ii = 1; ii <= _wSeq.Length(); ii++)
                {
                    Explorer explo = new(_wSeq[ii], ShapeType.Edge);
                    for (; explo.More(); explo.Next())
                    {
                        Edge edge = ShapeCasts.Edge(explo.Current());
                        NCollection_List<Shape> newEdges = georges.InternalGeneratedShapes(edge);
                        _edgeNewEdges[edge] = new NCollection_List<Shape>(newEdges);
                    }
                }
            }
            else
            {
                throw new ConstructionErrorException("PipeShell : uncompatible wires");
            }
            _section = new NSections(workingSections, transformations, Param, v1, v2);
        } // else

        //  modify the law of location if contact
        if ((_trihedron == Trihedron.IsGuidePlanWithContact) || (_trihedron == Trihedron.IsGuideACWithContact))
        {
            double fs, f, l, delta, length;
            LocationGuide loc;
            SurfaceSectionLaw sec = _section.ConcatenedLaw()!;
            _location.CurvilinearBounds(_location.LawCount, out f, out length);
            sec.GetDomain(out fs, out l);
            delta = (l - fs) / length;

            double angle = 0, old_angle = 0;
            for (int ipath = 1; ipath <= _location.LawCount; ipath++)
            {
                _location.CurvilinearBounds(ipath, out f, out l);
                loc = (LocationGuide)_location.Law(ipath);
                loc.Set(sec, true, fs + f * delta, fs + l * delta, old_angle, ref angle); // force the rotation
                old_angle = angle;
            }
        }

        _status = _location.GetStatus();
        if (!_section.IsDone)
        {
            _status = SurfacePipeError.PipeNotOk;
        }
    }

    //=======================================================================
    // function : Place
    // purpose  : Implement a Section in the local reference frame
    //           and return its parameter on the trajectory
    //=======================================================================
    private void Place(in FillingSection sec, out Wire w, out Transform3D aTrsf, out double param)
    {
        SectionPlacement place = new(_location!, sec.InternalWire(), sec.Vertex(), sec.WithContact(), sec.WithCorrection());
        Wire tmpWire = sec.InternalWire();
        aTrsf = place.Transformation();
        // Transform the copy
        w = ShapeCasts.Wire(new Transform(tmpWire, aTrsf, true).InternalShape());
        ////////////////////////////////////
        param = place.AbscissaOnPath();
    }

    //=======================================================================
    // function : ResetLoc
    // purpose  : Remove references to the sections in the laws of location
    //=======================================================================
    private void ResetLoc()
    {
        if ((_trihedron == Trihedron.IsGuidePlanWithContact) || (_trihedron == Trihedron.IsGuideACWithContact))
        {
            LocationGuide loc;
            for (int isec = 1; isec <= _location!.LawCount; isec++)
            {
                loc = (LocationGuide)_location.Law(isec);
                loc.EraseRotation(); // remove the rotation
            }
        }
    }

    //=======================================================================
    // function : BuildHistory
    // purpose  : Builds history for edges and vertices of sections,
    //           for edges and vertices of spine
    //=======================================================================
    private void BuildHistory(Sweep sweep)
    {
        // Filling of <myGenMap>
        NCollection_HArray2<Shape> uEdges = sweep.InterFaces()!;
        BrepBuilder bb = new();

        Dictionary<int, Shape> indWireMap = new();

        int indw, inde;
        Iterator itw = new();
        for (indw = 1; indw <= _seq.Length(); indw++)
        {
            Shape Section = _seq[indw].OriginalShape();
            Wire section;
            bool isPunctual = _seq[indw].IsPunctual;
            if (isPunctual)
            {
                // for punctual sections (first or last)
                // we take all the wires generated along the path

                NCollection_List<Shape> elist = _genMap.SetValueRef(Section, new NCollection_List<Shape>());
                for (int i = 1; i <= uEdges.UpperRow(); i++)
                {
                    for (int j = 1; j <= uEdges.UpperCol(); j++)
                    {
                        elist.Append(uEdges.Value(i, j));
                    }
                }

                continue;
            }
            else
            {
                section = ShapeCasts.Wire(Section);
            }
            // Take the real index of section on the path
            int indOfW = _indOfSec[indw];
            Wire theWire = ShapeCasts.Wire(_wSeq[indOfW]);
            WireExplorer wexp_sec = new(section);
            for (inde = 1; wexp_sec.More(); wexp_sec.Next())
            {
                Edge originalEdge = ShapeCasts.Edge(wexp_sec.Current());
                Edge edge = ShapeCasts.Edge(_seq[indw].InternalModifiedShape(originalEdge));
                if (Tool.Degenerated(edge))
                {
                    continue;
                }

                Shell shell = new();
                bb.MakeShell(ref shell);
                Vertex[] vertex = new Vertex[2];
                Exploration.Vertices(originalEdge, out vertex[0], out vertex[1]);
                int signOfAnEdge = (originalEdge.Orientation == Forward) ? 1 : -1;

                // For each non-degenerated inde-th edge of <aSection>
                // we find inde-th edge in <theWire>
                Edge theEdge = new();
                WireExplorer wexp = new(theWire);
                for (int i = 1; wexp.More(); wexp.Next())
                {
                    theEdge = ShapeCasts.Edge(wexp.Current());
                    if (Tool.Degenerated(edge)) // OCCT: tests anEdge, not theEdge
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
                NCollection_List<Shape> newEdges = _edgeNewEdges.GetValue(theEdge);
                int signOfANewEdge = 0, signOfIndex = 0;
                NCollection_List<Shape>.Iterator iter = new(newEdges);
                for (; iter.More(); iter.Next())
                {
                    Edge newEdge = ShapeCasts.Edge(iter.Value());
                    signOfANewEdge = (newEdge.Orientation == Forward) ? 1 : -1;
                    int indE = _section!.IndexOfEdge(newEdge);
                    signOfIndex = (indE > 0) ? 1 : -1;
                    indE = Math.Abs(indE);
                    // For an edge generated shape is a "tape" -
                    // a shell usually containing this edge and
                    // passing from beginning of path to its end
                    Shape tape = sweep.Tape(indE);
                    Iterator itsh = new(tape);
                    for (; itsh.More(); itsh.Next())
                    {
                        bb.Add(shell, itsh.Value());
                    }
                }

                // Processing of vertices of <anEdge>
                // We should choose right index in <anUEdges>
                // for each vertex of edge
                int toReverse = signOfAnEdge * signOfANewEdge * signOfIndex;
                int[] uIndex = new int[2];
                uIndex[0] = Math.Abs(_section!.IndexOfEdge(newEdges.First()));
                uIndex[1] = Math.Abs(_section.IndexOfEdge(newEdges.Last())) + toReverse;
                if (toReverse == -1)
                {
                    uIndex[0]++;
                    uIndex[1]++;
                }
                if (_section.IsUClosed)
                {
                    if (uIndex[0] > _section.LawCount)
                    {
                        uIndex[0] = 1;
                    }
                    if (uIndex[1] > _section.LawCount)
                    {
                        uIndex[1] = 1;
                    }
                }
                // if (SignOfAnEdge * SignOfANewEdge == -1)
                if (signOfAnEdge == -1 || signOfANewEdge == -1)
                {
                    int tmp = uIndex[0];
                    uIndex[0] = uIndex[1];
                    uIndex[1] = tmp;
                }

                IndexedMap<Shape, NCollection_List<Shape>> vEmap = new(ShapeHasher.Instance);
                Exploration.MapShapesAndAncestors(shell, ShapeType.Vertex, ShapeType.Edge, vEmap);
                for (int kk = 0; kk < 2; kk++)
                {
                    if (_genMap.ContainsKey(vertex[kk]))
                    {
                        continue;
                    }
                    if (indWireMap.ContainsKey(uIndex[kk]))
                    {
                        NCollection_List<Shape> elist = _genMap.SetValueRef(vertex[kk], new NCollection_List<Shape>());

                        for (itw.Initialize(indWireMap.GetValue(uIndex[kk])); itw.More(); itw.Next())
                        {
                            elist.Append(itw.Value());
                        }

                        continue;
                    }

                    // Collect u-edges
                    NCollection_Sequence<Shape> seqEdges = new();
                    int jj;
                    for (jj = 1; jj <= uEdges.UpperCol(); jj++)
                    {
                        seqEdges.Append(uEdges.Value(uIndex[kk], jj));
                    }

                    // Assemble the wire ("rail" along the path)
                    // checking for possible holes
                    //(they appear with option "Round Corner")
                    // and filling them
                    // Missed edges are taken from <aShell>
                    Wire wire = new();
                    bb.MakeWire(ref wire);
                    Edge firstEdge = ShapeCasts.Edge(seqEdges[1]);
                    if (firstEdge.IsNull)
                    {
                        continue;
                    }
                    bb.Add(wire, firstEdge);
                    Exploration.Vertices(firstEdge, out Vertex firstVertex, out Vertex curVertex);
                    Edge curEdge = new();
                    for (jj = 2; jj <= seqEdges.Length(); jj++)
                    {
                        curEdge = ShapeCasts.Edge(seqEdges[jj]);
                        Exploration.Vertices(curEdge, out Vertex vfirst, out Vertex vlast);
                        if (curVertex.IsSame(vfirst))
                        {
                            curVertex = vlast;
                        }
                        else // a hole
                        {
                            NCollection_List<Shape> elist = vEmap.GetValue(vfirst);
                            NCollection_List<Shape>.Iterator itl = new(elist);
                            for (; itl.More(); itl.Next())
                            {
                                Edge candidate = ShapeCasts.Edge(itl.Value());
                                if (candidate.IsSame(curEdge))
                                {
                                    continue;
                                }
                                Exploration.Vertices(candidate, out Vertex v1, out Vertex v2);
                                if (v1.IsSame(curVertex) || v2.IsSame(curVertex))
                                {
                                    bb.Add(wire, candidate);
                                    break;
                                }
                            }
                        }
                        curVertex = vlast;
                        bb.Add(wire, curEdge);
                    } // for (jj = 2; jj <= SeqEdges.Length(); jj++)
                    // case of closed wire
                    if (_location!.IsClosed() && !curVertex.IsSame(firstVertex))
                    {
                        NCollection_List<Shape> elist = vEmap.GetValue(curVertex);
                        NCollection_List<Shape>.Iterator itl = new(elist);
                        for (; itl.More(); itl.Next())
                        {
                            Edge candidate = ShapeCasts.Edge(itl.Value());
                            if (candidate.IsSame(curEdge))
                            {
                                continue;
                            }
                            Exploration.Vertices(candidate, out Vertex v1, out Vertex v2);
                            if (v1.IsSame(firstVertex) || v2.IsSame(firstVertex))
                            {
                                bb.Add(wire, candidate);
                                break;
                            }
                        }
                    }

                    NCollection_List<Shape> elist2 = _genMap.SetValueRef(vertex[kk], new NCollection_List<Shape>());

                    for (itw.Initialize(wire); itw.More(); itw.Next())
                    {
                        elist2.Append(itw.Value());
                    }

                    // Save already built wire with its index
                    indWireMap[uIndex[kk]] = wire;
                } // for (int kk = 0; kk < 2; kk++)
                ////////////////////////////////////

                NCollection_List<Shape> flist = _genMap.SetValueRef(originalEdge, new NCollection_List<Shape>());
                Iterator itsh2 = new(shell);
                for (; itsh2.More(); itsh2.Next())
                {
                    flist.Append(itsh2.Value());
                }
                ////////////////////////

                inde++;
            }
        }

        // For subshapes of spine
        NCollection_HArray2<Shape> faces = sweep.SubShape()!;
        NCollection_HArray2<Shape> vEdges = sweep.Sections()!;

        WireExplorer wexp2 = new(_spine);
        inde = 0;
        bool toExit = false;
        for (;;)
        {
            if (!wexp2.More())
            {
                toExit = true;
            }

            inde++;

            if (!toExit)
            {
                Edge edgeOfSpine = wexp2.Current();

                NCollection_List<Shape> flist = _genMap.SetValueRef(edgeOfSpine, new NCollection_List<Shape>());

                for (int i = 1; i <= faces.UpperRow(); i++)
                {
                    Shape face = faces.Value(i, inde);
                    if (face.Type == ShapeType.Face)
                    {
                        flist.Append(face);
                    }
                }
            }

            Vertex vertexOfSpine = wexp2.CurrentVertex();
            NCollection_List<Shape> listVshapes = _genMap.SetValueRef(vertexOfSpine, new NCollection_List<Shape>());
            for (int i = 1; i <= vEdges.UpperRow(); i++)
            {
                Shape vshape = vEdges.Value(i, inde);
                if (vshape.IsNull)
                {
                    continue;
                }
                if (vshape.Type == ShapeType.Edge || vshape.Type == ShapeType.Face)
                {
                    listVshapes.Append(vshape);
                }
                else
                {
                    Iterator itvshape = new(vshape);
                    for (; itvshape.More(); itvshape.Next())
                    {
                        Shape subshape = itvshape.Value();
                        if (subshape.Type == ShapeType.Edge || subshape.Type == ShapeType.Face)
                        {
                            listVshapes.Append(subshape);
                        }
                        else
                        {
                            // it is wire
                            for (itw.Initialize(subshape); itw.More(); itw.Next())
                            {
                                listVshapes.Append(itw.Value());
                            }
                        }
                    }
                }
            }

            if (toExit)
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

    private static bool BuildBoundaries(Sweep sweep,
                                        SectionLaw section,
                                        ref Shape bottom,
                                        ref Shape top)
    {
        Wire bottomWire = new();
        Wire topWire = new();
        BrepBuilder b = new();
        b.MakeWire(ref bottomWire);
        b.MakeWire(ref topWire);
        bool bfoundbottom = false;
        bool bfoundtop = false;
        NCollection_HArray2<Shape> vEdges = sweep.Sections()!;
        int i = 0;
        bool bAllSame = true;

        for (i = 1; i <= section.LawCount; i++)
        {
            Shape bottomEdge = vEdges.Value(i, vEdges.LowerCol());

            if (!bottomEdge.IsNull && (bottomEdge.Type == ShapeType.Edge))
            {
                b.Add(bottomWire, bottomEdge);
                bfoundbottom = true;
            }
            Shape topEdge = vEdges.Value(i, vEdges.UpperCol());

            if (!topEdge.IsNull && (topEdge.Type == ShapeType.Edge))
            {
                b.Add(topWire, topEdge);
                bfoundtop = true;
            }

            if (!bottomEdge.IsNull && !topEdge.IsNull && !bottomEdge.IsSame(topEdge))
            {
                bAllSame = false;
            }
        }

        if (section.IsUClosed)
        {
            bottomWire.Closed(true);
            topWire.Closed(true);
        }

        if (bfoundbottom)
        {
            bottom = bottomWire;
        }

        if (bfoundtop)
        {
            top = topWire;
        }

        if (bAllSame && bfoundbottom && bfoundtop)
        {
            top = bottom;
        }

        return bfoundbottom || bfoundtop;
    }
}
