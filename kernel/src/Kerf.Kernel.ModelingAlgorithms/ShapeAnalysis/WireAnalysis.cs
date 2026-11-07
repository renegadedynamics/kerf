// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_Wire.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_Wire.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_Wire.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Healing.Analysis;

/// <summary>
/// Analysis of a wire for compliance with OCCT's requirements: the consistency of the 2d and 3d senses of its edges,
/// the connection of adjacent edges (by vertices, pcurves and 3d curves), the adjacency of each edge's vertices to its
/// curves, whether the wire is closed (in 3d and 2d) and whether it is outer on its face.
/// </summary>
/// <remarks>
/// A working object (decision 21): load a wire, set the face and the precision, then call the checks. The public
/// checks (<see cref="Perform"/>, <see cref="CheckOrder(bool, bool)"/>, ...) set their own status, queried by the
/// <c>Status...</c> methods; the advanced ones (taking an edge number) share one status, queried by
/// <see cref="LastCheckStatus"/>. The checks only read the wire, its face and their geometry.
/// </remarks>
internal class WireAnalysis : Transient
{
    protected ExtendWireData?  _wire;
    protected Face            _face;
    protected SurfaceAnalysis? _surf;
    protected double                 _precision;
    protected double                 _min3d;
    protected double                 _min2d;
    protected double                 _max3d;
    protected double                 _max2d;
    protected int                    _statusOrder;
    protected int                    _statusConnected;
    protected int                    _statusEdgeCurves;
    protected int                    _statusDegenerated;
    protected int                    _statusClosed;
    protected int                    _statusSmall;
    protected int                    _statusSelfIntersection;
    protected int                    _statusLacking;
    protected int                    _statusGaps3d;
    protected int                    _statusGaps2d;
    protected int                    _statusCurveGaps;
    protected int                    _statusLoop;
    protected int                    _status;

    /// <summary>Empty constructor.</summary>
    public WireAnalysis()
    {
        _face = new Face();
        ClearStatuses();
        _precision = Kerf.Engine.Foundation.Precision.Confusion();
    }

    /// <summary>Creates the object with a wire, a face and a precision.</summary>
    public WireAnalysis(in Wire wire, in Face face, double precision)
    {
        _face = new Face();
        Init(wire, face, precision);
    }

    /// <summary>Creates the object with a WireData object, a face and a precision.</summary>
    public WireAnalysis(ExtendWireData? sbwd, in Face face, double precision)
    {
        _face = new Face();
        Init(sbwd, face, precision);
    }

    /// <summary>Initializes the object with a wire, a face and a precision.</summary>
    public void Init(in Wire wire, in Face face, double precision)
    {
        Init(new ExtendWireData(wire), face, precision);
    }

    /// <summary>Initializes the object with a WireData object, a face and a precision.</summary>
    public void Init(ExtendWireData? sbwd, in Face face, double precision)
    {
        Load(sbwd);
        SetFace(face);
        SetPrecision(precision);
    }

    /// <summary>Loads the object with a wire.</summary>
    public void Load(in Wire wire)
    {
        ClearStatuses();
        _wire = new ExtendWireData(wire);
    }

    /// <summary>Loads the object with a WireData object (shared, not copied).</summary>
    public void Load(ExtendWireData? sbwd)
    {
        ClearStatuses();
        _wire = sbwd;
    }

    /// <summary>Loads the face the wire lies on.</summary>
    public void SetFace(in Face face)
    {
        _face = face;
        if (_face.IsNull)
        {
            return;
        }

        Surface? surface = Tool.Surface(_face);
        if (!face.IsNull && surface is not null)
        {
            _surf = new SurfaceAnalysis(surface);
        }
    }

    /// <summary>Loads the face the wire lies on and its surface analysis object.</summary>
    public void SetFace(in Face face, SurfaceAnalysis? surfaceAnalysis)
    {
        _face = face;
        _surf = surfaceAnalysis;
    }

    /// <summary>Loads the surface analysis object.</summary>
    public void SetSurface(SurfaceAnalysis? surfaceAnalysis)
    {
        _surf = surfaceAnalysis;
    }

    /// <summary>Loads the surface the wire lies on.</summary>
    public void SetSurface(Surface? surface)
    {
        SetSurface(surface, new Placement());
    }

    /// <summary>Loads the surface the wire lies on, with a location; a new face on it becomes the working face.</summary>
    public void SetSurface(Surface? surface, in Placement location)
    {
        BrepBuilder b    = new BrepBuilder();
        Face  face = new Face();
        b.MakeFace(ref face, surface, location, Kerf.Engine.Foundation.Precision.Confusion());
        // Kerf: the face is reachable through Face(), so it is published frozen (design 4.2)
        face.TShape()!.FreezeGraph();
        SetFace(face);
    }

    /// <summary>Sets the precision.</summary>
    public void SetPrecision(double precision)
    {
        _precision = precision;
    }

    /// <summary>Unsets all the status and distance fields; the wire, face and precision are kept.</summary>
    public void ClearStatuses()
    {
        _statusOrder = _statusConnected = _statusEdgeCurves = _statusDegenerated = _statusClosed =
            _statusLacking = _statusSelfIntersection = _statusSmall = _statusGaps3d = _statusGaps2d =
                _statusCurveGaps = _statusLoop = _status = 0;

        _min3d = _min2d = _max3d = _max2d = 0.0;
    }

    /// <summary>Returns true if a wire is loaded and has at least one edge.</summary>
    public bool IsLoaded
    {
        get
        {
            return _wire is not null && _wire.EdgeCount > 0;
        }
    }

    /// <summary>Returns true if <see cref="IsLoaded"/> and the face isn't null.</summary>
    public bool IsReady
    {
        get
        {
            return IsLoaded && !_face.IsNull;
        }
    }

    /// <summary>The precision.</summary>
    public double Precision()
    {
        return _precision;
    }

    /// <summary>The wire being analysed.</summary>
    public ExtendWireData? WireData()
    {
        return _wire;
    }

    /// <summary>The number of edges in the wire, 0 if no wire is loaded.</summary>
    public int EdgeCount
    {
        get
        {
            return _wire is null ? 0 : _wire.EdgeCount;
        }
    }

    /// <summary>The working face.</summary>
    public Face Face()
    {
        return _face;
    }

    /// <summary>The working surface.</summary>
    public SurfaceAnalysis? Surface()
    {
        return _surf;
    }

    public bool StatusOrder(ExtendStatus status) => global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_statusOrder, status);

    public bool StatusConnected(ExtendStatus status) => global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_statusConnected, status);

    public bool StatusEdgeCurves(ExtendStatus status) => global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_statusEdgeCurves, status);

    public bool StatusDegenerated(ExtendStatus status) => global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_statusDegenerated, status);

    public bool StatusClosed(ExtendStatus status) => global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_statusClosed, status);

    public bool StatusSmall(ExtendStatus status) => global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_statusSmall, status);

    public bool StatusSelfIntersection(ExtendStatus status) =>
        global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_statusSelfIntersection, status);

    public bool StatusLacking(ExtendStatus status) => global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_statusLacking, status);

    public bool StatusGaps3d(ExtendStatus status) => global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_statusGaps3d, status);

    public bool StatusGaps2d(ExtendStatus status) => global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_statusGaps2d, status);

    public bool StatusCurveGaps(ExtendStatus status) => global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_statusCurveGaps, status);

    public bool StatusLoop(ExtendStatus status) => global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_statusLoop, status);

    /// <summary>Queries the status of the last performed advanced check.</summary>
    public bool LastCheckStatus(ExtendStatus status) => global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_status, status);

    /// <summary>The last lowest distance in 3D computed by the checks.</summary>
    public double MinDistance3d() => _min3d;

    /// <summary>The last lowest distance in 2D (UV) computed by the checks.</summary>
    public double MinDistance2d() => _min2d;

    /// <summary>The last maximal distance in 3D computed by the checks.</summary>
    public double MaxDistance3d() => _max3d;

    /// <summary>The last maximal distance in 2D (UV) computed by the checks.</summary>
    public double MaxDistance2d() => _max2d;

    /// <summary>
    /// Performs all the checks in the order CheckOrder, CheckSmall, CheckConnected, CheckEdgeCurves, CheckDegenerated,
    /// CheckSelfIntersection, CheckLacking, CheckClosed. True if at least one of them returned true.
    /// </summary>
    public bool Perform()
    {
        bool result = false;
        result |= CheckOrder();
        result |= CheckSmall();
        result |= CheckConnected();
        result |= CheckEdgeCurves();
        result |= CheckDegenerated();
        result |= CheckSelfIntersection();
        result |= CheckLacking();
        result |= CheckClosed();
        return result;
    }

    /// <summary>
    /// Calls CheckOrder and returns false if the wire is already ordered (tail to head), true otherwise.
    /// </summary>
    public bool CheckOrder(bool isClosed = true, bool mode3d = true)
    {
        WireOrderAnalysis sawo = new WireOrderAnalysis();
        CheckOrder(sawo, isClosed, mode3d, false);
        _statusOrder = _status;
        return StatusOrder(ExtendStatus.Done);
    }

    /// <summary>Calls CheckSmall for each edge; true if at least one small edge was detected.</summary>
    public bool CheckSmall(double precsmall = 0.0)
    {
        for (int i = 1; i <= _wire!.EdgeCount; i++)
        {
            CheckSmall(i, precsmall);
            _statusSmall |= _status;
        }

        return StatusSmall(ExtendStatus.Done);
    }

    /// <summary>Calls CheckConnected for each edge; true if at least one pair of disconnected edges was detected.</summary>
    public bool CheckConnected(double prec = 0.0)
    {
        for (int i = 1; i <= _wire!.EdgeCount; i++)
        {
            CheckConnected(i, prec);
            _statusConnected |= _status;
        }

        return StatusConnected(ExtendStatus.Done);
    }

    /// <summary>
    /// Checks the edges' geometry: ShapeAnalysis_Edge's CheckCurve3dWithPCurve (1), CheckVerticesWithPCurve (2),
    /// CheckVerticesWithCurve3d (3), CheckSeam (4), CheckGap3d (5), CheckGap2d (6) and ShapeAnalysis_Edge's
    /// CheckSameParameter (7); the numbers are the DONEi and FAILi flags of the status.
    /// </summary>
    public bool CheckEdgeCurves()
    {
        _statusEdgeCurves = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (!IsReady)
        {
            return false;
        }

        int                i, nb = _wire!.EdgeCount;
        EdgeAnalysis sae = new EdgeAnalysis();

        for (i = 1; i <= nb; i++)
        {
            Edge e = _wire.Edge(i);

            sae.CheckCurve3dWithPCurve(e, _face);
            if (sae.Status(ExtendStatus.Done))
            {
                _statusEdgeCurves |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
            }

            if (sae.Status(ExtendStatus.Fail))
            {
                _statusEdgeCurves |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
            }

            sae.CheckVerticesWithPCurve(e, _face);
            if (sae.Status(ExtendStatus.Done))
            {
                _statusEdgeCurves |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
            }

            if (sae.Status(ExtendStatus.Fail))
            {
                _statusEdgeCurves |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
            }

            sae.CheckVerticesWithCurve3d(e);
            if (sae.Status(ExtendStatus.Done))
            {
                _statusEdgeCurves |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE3);
            }

            if (sae.Status(ExtendStatus.Fail))
            {
                _statusEdgeCurves |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL3);
            }

            CheckSeam(i);
            if (LastCheckStatus(ExtendStatus.Done))
            {
                _statusEdgeCurves |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE4);
            }

            if (LastCheckStatus(ExtendStatus.Fail))
            {
                _statusEdgeCurves |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL4);
            }

            CheckGap3d(i);
            if (LastCheckStatus(ExtendStatus.Done))
            {
                _statusEdgeCurves |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE5);
            }

            if (LastCheckStatus(ExtendStatus.Fail))
            {
                _statusEdgeCurves |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL5);
            }

            CheckGap2d(i);
            if (LastCheckStatus(ExtendStatus.Done))
            {
                _statusEdgeCurves |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE6);
            }

            if (LastCheckStatus(ExtendStatus.Fail))
            {
                _statusEdgeCurves |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL6);
            }

            double maxdev = 0.0;
            sae.CheckSameParameter(_wire.Edge(i), ref maxdev);
            if (sae.Status(ExtendStatus.Done))
            {
                _statusEdgeCurves |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE7);
            }

            if (sae.Status(ExtendStatus.Fail))
            {
                _statusEdgeCurves |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL7);
            }
        }

        return StatusEdgeCurves(ExtendStatus.Done);
    }

    /// <summary>Calls CheckDegenerated for each edge; true if at least one incorrect degenerated edge was detected.</summary>
    public bool CheckDegenerated()
    {
        for (int i = 1; i <= _wire!.EdgeCount; i++)
        {
            CheckDegenerated(i);
            _statusDegenerated |= _status;
        }

        return StatusDegenerated(ExtendStatus.Done);
    }

    /// <summary>
    /// Checks the self-intersection of the wire (on pcurves): each edge with itself (DONE1/FAIL1), each two adjacent
    /// edges (DONE2/FAIL2), and each two non-adjacent edges whose boxes overlap (DONE3/FAIL3).
    /// </summary>
    public bool CheckSelfIntersection()
    {
        _statusSelfIntersection = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (!IsReady)
        {
            return false;
        }

        int i, nb = _wire!.EdgeCount;
        for (i = 1; i <= nb; i++)
        {
            CheckSelfIntersectingEdge(i);
            if (LastCheckStatus(ExtendStatus.Done))
            {
                _statusSelfIntersection |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
            }

            if (LastCheckStatus(ExtendStatus.Fail))
            {
                _statusSelfIntersection |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
            }

            CheckIntersectingEdges(i);
            if (LastCheckStatus(ExtendStatus.Done))
            {
                _statusSelfIntersection |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
            }

            if (LastCheckStatus(ExtendStatus.Fail))
            {
                _statusSelfIntersection |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
            }
        }

        NCollection_Array1<Box2d> boxes = new NCollection_Array1<Box2d>(1, nb);
        Surface?        s    = Tool.Surface(Face(), out Placement l);
        EdgeAnalysis   sae  = new EdgeAnalysis();
        ExtendWireData sbwd = WireData()!;
        for (i = 1; i <= nb; i++)
        {
            Edge e = sbwd.Edge(i);
            if (sae.PCurve(e, s, l, out Curve2D? c2d, out double cf, out double cl, false))
            {
                Box2d           box = new Box2d();
                CurveAdaptor2D gac = new CurveAdaptor2D(c2d!, cf, cl);
                Add2dCurve.Add(gac, Kerf.Engine.Foundation.Precision.Confusion(), ref box);
                boxes[i] = box;
            }
        }

        bool isFail = false, isDone = false;
        for (int num1 = 1; num1 < nb - 1; num1++)
        {
            int fin = nb;
            if (CheckClosed(Kerf.Engine.Foundation.Precision.Confusion()) && 1 == num1)
            {
                fin = nb - 1;
            }

            for (int num2 = num1 + 2; num2 <= fin; num2++)
            {
                if (!boxes[num1].IsOut(boxes[num2]))
                {
                    CheckIntersectingEdges(num1, num2);
                    isFail |= LastCheckStatus(ExtendStatus.FAIL1);
                    isDone |= LastCheckStatus(ExtendStatus.DONE1);
                }
            }
        }

        if (isFail)
        {
            _statusSelfIntersection |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL3);
        }

        if (isDone)
        {
            _statusSelfIntersection |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE3);
        }

        return StatusSelfIntersection(ExtendStatus.Done);
    }

    /// <summary>Calls CheckLacking for each edge; true if at least one lacking edge was detected.</summary>
    public bool CheckLacking()
    {
        if (!IsReady || EdgeCount < 2)
        {
            return false;
        }

        for (int i = 1; i <= _wire!.EdgeCount; i++)
        {
            CheckLacking(i);
            _statusLacking |= _status;
        }

        return StatusLacking(ExtendStatus.Done);
    }

    /// <summary>
    /// Checks if the wire is closed: CheckConnected (DONE1/FAIL1) and CheckDegenerated (DONE2/FAIL2) of the first edge
    /// after the last.
    /// </summary>
    public bool CheckClosed(double prec = 0.0)
    {
        _statusClosed = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (!IsReady || EdgeCount < 1)
        {
            return false;
        }

        CheckConnected(1, prec);
        if (LastCheckStatus(ExtendStatus.Done))
        {
            _statusClosed |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
        }

        if (LastCheckStatus(ExtendStatus.Fail))
        {
            _statusClosed |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
        }

        CheckDegenerated(1);
        if (LastCheckStatus(ExtendStatus.Done))
        {
            _statusClosed |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
        }

        if (LastCheckStatus(ExtendStatus.Fail))
        {
            _statusClosed |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
        }

        return StatusClosed(ExtendStatus.Done);
    }

    /// <summary>Calls CheckGap3d for each edge; the largest gap is in <see cref="MinDistance3d"/> and <see cref="MaxDistance3d"/>.</summary>
    public bool CheckGaps3d()
    {
        _statusGaps3d = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (!IsLoaded || EdgeCount < 1)
        {
            return false; // gka IsLoaded
        }

        double dist, maxdist = 0.0;

        for (int i = 1; i <= EdgeCount; i++)
        {
            CheckGap3d(i);
            _statusGaps3d |= _status;
            if (!LastCheckStatus(ExtendStatus.FAIL1))
            {
                dist = MinDistance3d();
                if (maxdist < dist)
                {
                    maxdist = dist;
                }
            }
        }

        _min3d = _max3d = maxdist;

        return StatusGaps3d(ExtendStatus.Done);
    }

    /// <summary>Calls CheckGap2d for each edge; the largest gap is in <see cref="MinDistance2d"/> and <see cref="MaxDistance2d"/>.</summary>
    public bool CheckGaps2d()
    {
        _statusGaps2d = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (!IsReady || EdgeCount < 1)
        {
            return false;
        }

        double dist, maxdist = 0.0;

        for (int i = 1; i <= EdgeCount; i++)
        {
            CheckGap2d(i);
            _statusGaps2d |= _status;
            if (!LastCheckStatus(ExtendStatus.FAIL1))
            {
                dist = MinDistance2d();
                if (maxdist < dist)
                {
                    maxdist = dist;
                }
            }
        }

        _min2d = _max2d = maxdist;

        return StatusGaps2d(ExtendStatus.Done);
    }

    /// <summary>Calls CheckCurveGap for each edge; the largest gap is in <see cref="MinDistance3d"/> and <see cref="MaxDistance3d"/>.</summary>
    public bool CheckCurveGaps()
    {
        _statusCurveGaps = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (!IsReady || EdgeCount < 1)
        {
            return false;
        }

        double dist, maxdist = 0.0;

        for (int i = 1; i <= EdgeCount; i++)
        {
            CheckCurveGap(i);
            _statusCurveGaps |= _status;
            if (!LastCheckStatus(ExtendStatus.FAIL1))
            {
                dist = MinDistance3d();
                if (maxdist < dist)
                {
                    maxdist = dist;
                }
            }
        }

        _min3d = _max3d = maxdist;

        return StatusCurveGaps(ExtendStatus.Done);
    }

    /// <summary>
    /// Analyzes the order of the edges in the wire with <paramref name="sawo"/>, in 3d (vertices) or 2d (pcurve ends), or
    /// both when <paramref name="modeBoth"/> is set. Returns false if the wire is already ordered (tail to head).
    /// </summary>
    /// <remarks>
    /// Status: OK the same orientations and sequence; DONE1 the same orientations, another sequence; DONE2 as DONE1 with
    /// gaps over the precision; DONE3 some edges must be reversed; DONE4 as DONE3 with gaps; DONE5 only shifted; FAIL the
    /// order couldn't be found (FAIL2: no face in 2d mode, no vertices in 3d mode, or no pcurve in 2d mode).
    /// </remarks>
    public bool CheckOrder(WireOrderAnalysis sawo, bool isClosed = true, bool mode3D = true, bool modeBoth = false)
    {
        if ((!mode3D || modeBoth) && _face.IsNull)
        {
            _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
            return false;
        }

        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        sawo.SetMode(mode3D, 0.0, modeBoth);
        int                nb = _wire!.EdgeCount;
        EdgeAnalysis ea = new EdgeAnalysis();
        bool               isAll2dEdgesOk = true;
        for (int i = 1; i <= nb; i++)
        {
            Edge e = _wire.Edge(i);
            Xyz      p1XYZ = new Xyz(), p2XYZ = new Xyz();
            Xy       p1XY = new Xy(), p2XY = new Xy();
            if (mode3D || modeBoth)
            {
                Vertex v1 = ea.FirstVertex(e);
                Vertex v2 = ea.LastVertex(e);
                if (v1.IsNull || v2.IsNull)
                {
                    _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
                    return false;
                }
                else
                {
                    p1XYZ = Tool.Pnt(v1).XYZ();
                    p2XYZ = Tool.Pnt(v2).XYZ();
                }
            }

            if (!mode3D || modeBoth)
            {
                Shape tmpF = _face.Oriented(Forward);
                if (!ea.PCurve(e, ShapeCasts.Face(tmpF), out Curve2D? c2d, out double f, out double l))
                {
                    // if mode is 2d, then we can nothing to do, else we can switch to 3d mode
                    if (!mode3D && !modeBoth)
                    {
                        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
                        return false;
                    }
                    else
                    {
                        isAll2dEdgesOk = false;
                    }
                }
                else
                {
                    p1XY = c2d!.Value(f).XY();
                    p2XY = c2d.Value(l).XY();
                }
            }

            if (mode3D && !modeBoth)
            {
                sawo.Add(p1XYZ, p2XYZ);
            }
            else if (!mode3D && !modeBoth)
            {
                sawo.Add(p1XY, p2XY);
            }
            else
            {
                sawo.Add(p1XYZ, p2XYZ, p1XY, p2XY);
            }
        }

        // need to switch to 3d mode
        if (modeBoth && !isAll2dEdgesOk)
        {
            sawo.SetMode(true, 0.0, false);
        }

        sawo.Perform(isClosed);
        int stat = sawo.Status();
        switch (stat)
        {
            case 0:
                _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
                break;
            case 1:
                _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
                break;
            case 2:
                _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
                break; // this value is not returned
            case -1:
                _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE3);
                break;
            case -2:
                _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE4);
                break; // this value is not returned
            case 3:
                _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE5);
                break; // only shifted
            case -10:
                _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
                break; // this value is not returned
        }

        return LastCheckStatus(ExtendStatus.Done);
    }

    /// <summary>
    /// Checks that the num-th edge and the preceding one are connected (by their vertices). Returns false if they share
    /// the vertex.
    /// </summary>
    /// <remarks>
    /// Status: OK the same vertex; DONE1 confused at gp::Resolution; DONE2 at the precision; DONE3 at
    /// <paramref name="prec"/> only; FAIL1 not confused; FAIL2 not confused, but would be if the num-th edge were reversed
    /// (or a vertex is missing).
    /// </remarks>
    public bool CheckConnected(int num, double prec = 0.0)
    {
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (!IsLoaded || EdgeCount < 1)
        {
            return false;
        }

        int n2 = (num > 0 ? num : EdgeCount);
        int n1 = (n2 > 1 ? n2 - 1 : EdgeCount);
        //  if (n1 == n2) return 0;

        Edge        e1  = WireData()!.Edge(n1);
        Edge        e2  = WireData()!.Edge(n2);
        EdgeAnalysis sae = new EdgeAnalysis();
        Vertex      v1  = sae.LastVertex(e1);
        Vertex      v2  = sae.FirstVertex(e2);
        if (v1.IsNull || v2.IsNull)
        {
            _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
            return false;
        }

        if (v1.IsSame(v2))
        {
            return false;
        }

        Point3D p1 = Tool.Pnt(v1);
        Point3D p2 = Tool.Pnt(v2);
        _min3d   = p1.DistanceTo(p2);
        if (_min3d <= Constants.Resolution())
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
        }
        else if (_min3d <= _precision)
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
        }
        else if (_min3d <= prec)
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE3);
        }
        else
        {
            // et en inversant la derniere edge ?
            if (n1 == n2)
            {
                _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
            }
            else
            {
                v2          = sae.LastVertex(e2);
                p2          = Tool.Pnt(v2);
                double dist = p1.DistanceTo(p2);
                if (dist > _precision)
                {
                    _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
                }
                else
                {
                    _min3d  = dist;
                    _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
                }
            }

            return false;
        }

        return true;
    }

    /// <summary>
    /// Checks if an edge is no longer than <paramref name="precsmall"/>. Status: DONE1 small, with the same vertices;
    /// DONE2 small, with different vertices; FAIL no 3d curve and no pcurve (FAIL2: a vertex is missing).
    /// </summary>
    public bool CheckSmall(int num, double precsmall = 0.0)
    {
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (!IsLoaded || EdgeCount <= 1)
        {
            return false;
        }

        Edge        e   = _wire!.Edge(num != 0 ? num : EdgeCount);
        EdgeAnalysis sae = new EdgeAnalysis();

        if (Tool.Degenerated(e))
        {
            //: n2 abv 22 Jan 99: ma-test5.igs -> IGES (brep) -> read (pref3d):
            // degen edge with no pcurve should be removed
            if (!_face.IsNull && sae.HasPCurve(e, Face()))
            {
                return false;
            }

            _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
        }

        Vertex v1 = sae.FirstVertex(e);
        Vertex v2 = sae.LastVertex(e);
        if (v1.IsNull || v2.IsNull)
        {
            _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
            return false;
        }

        Point3D p1   = Tool.Pnt(v1);
        Point3D p2   = Tool.Pnt(v2);
        double dist = p1.DistanceTo(p2);
        double prec = precsmall; // Min ( myPrecision, precsmall );
        if (dist > prec)
        {
            return false; // not small enough
        }

        // The 3D curve now: is it CLOSED or ZERO LENGTH...???
        // To do this, we take the midpoint (is there anything better?)
        // If there's no 3D curve, we try 2D...
        Point3D midpoint;
        if (sae.Curve3d(e, out Curve3D? c3d, out double cf, out double cl, false))
        {
            midpoint = c3d!.Value((cf + cl) / 2.0);
        }
        else
        {
            Curve2D? c2d = null;
            if (!_face.IsNull && _surf is not null && sae.PCurve(e, _face, out c2d, out cf, out cl, false))
            {
                Point2D p2m = c2d!.Value((cf + cl) / 2.0);
                midpoint    = _surf.Value(p2m);
            }
            else
            {
                _status  = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
                midpoint = p1;
            }
        }

        if (midpoint.DistanceTo(p1) > prec || midpoint.DistanceTo(p2) > prec)
        {
            return false;
        }

        _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(v1.IsSame(v2) ? ExtendStatus.DONE1 : ExtendStatus.DONE2);
        return true;
    }

    /// <summary>
    /// Checks if the pcurves of a seam edge are correctly oriented. Returns false (status OK) if the edge isn't a seam or
    /// if they are; true (DONE1) if they should be swapped. <paramref name="c1"/> and <paramref name="c2"/> get the
    /// pcurves of the forward and reversed edge, <paramref name="cf"/> and <paramref name="cl"/> their range (left
    /// unchanged when the edge isn't a seam).
    /// </summary>
    public bool CheckSeam(int num, ref Curve2D? c1, ref Curve2D? c2, ref double cf, ref double cl)
    {
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (!IsReady)
        {
            return false;
        }

        int n = num;
        if (n == 0)
        {
            n = EdgeCount;
        }

        Edge e = _wire!.Edge(n);
        if (!new EdgeAnalysis().IsSeam(e, _face))
        {
            return false;
        }

        // Extract the Two PCurves of the Seam
        Face forwardFace = _face;
        forwardFace.Orient(Forward);
        // szv#4:S4163:12Mar99 SGI warns
        Shape ef = e.Oriented(Forward);
        Shape er = e.Oriented(Reversed);
        c1              = Tool.CurveOnSurface(ShapeCasts.Edge(ef), forwardFace, out cf, out cl);
        c2              = Tool.CurveOnSurface(ShapeCasts.Edge(er), forwardFace, out cf, out cl);
        if (c1 is null || c2 is null)
        {
            return false;
        }

        //  SelectForward est destine a devenir un outil distinct
        int theCurveIndice = new CurveAnalysis().SelectForwardSeam(c1, c2);
        if (theCurveIndice != 2)
        {
            return false;
        }

        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
        return true;
    }

    /// <summary>Checks if the pcurves of a seam edge are correctly oriented (see the other overload).</summary>
    public bool CheckSeam(int num)
    {
        Curve2D? c1 = null, c2 = null;
        double        cf = 0.0, cl = 0.0;
        return CheckSeam(num, ref c1, ref c2, ref cf, ref cl);
    }

    /// <summary>
    /// Checks for a degenerated edge between two adjacent ones, filling <paramref name="p2d1"/> and
    /// <paramref name="p2d2"/> with the parametric ends of the singularity (a gap to fill by a degenerated edge, or one
    /// already filled).
    /// </summary>
    /// <remarks>
    /// Status: OK no singularity, or the edge is already degenerated; DONE1 a degenerated edge should be inserted (gap in
    /// 2D); DONE2 the edge should be made degenerated; FAIL1 a neighbour of the degenerated edge has no pcurve; FAIL2 an
    /// edge marked degenerated has no pcurve, but no singularity is detected.
    /// </remarks>
    public bool CheckDegenerated(int num, ref Point2D p2d1, ref Point2D p2d2)
    {
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (!IsReady || EdgeCount < 1)
        {
            return false;
        }

        int         n2 = (num > 0) ? num : EdgeCount;
        int         n1 = (n2 > 1) ? n2 - 1 : EdgeCount;
        int         n3 = (n2 < EdgeCount) ? n2 + 1 : 1;
        Edge e1 = _wire!.Edge(n1);
        Edge e2 = _wire.Edge(n2);
        Edge e3 = _wire.Edge(n3);

        EdgeAnalysis sae = new EdgeAnalysis();

        // skip if edge is already marked as degenerated and has pcurve
        if (Tool.Degenerated(e2) && sae.HasPCurve(e2, Face()))
        {
            // skl 30.12.2004 for OCC7630 - we have to check pcurve
            if (sae.HasPCurve(e1, Face()) && sae.HasPCurve(e3, Face()))
            {
                sae.PCurve(e2, _face, out Curve2D? c2d, out double fp, out double lp, true);
                Point2D p21 = c2d!.Value(fp);
                Point2D p22 = c2d.Value(lp);
                sae.PCurve(e1, _face, out c2d, out fp, out lp, true);
                Point2D p12 = c2d!.Value(lp);
                sae.PCurve(e3, _face, out c2d, out fp, out lp, true);
                Point2D p31 = c2d!.Value(fp);
                if (Math.Abs(p12.DistanceTo(p31) - p21.DistanceTo(p22)) > 2 * Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    // pcurve is bad => we can remove this edge in ShapeFix
                    // if set needed status
                    _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
                }
            }

            return false;
        }

        // pdn allows to insert two sequences of degenerated edges (on separate bounds of surfaces)
        if (n1 != n2 && Tool.Degenerated(e1) && !sae.HasPCurve(e1, Face()))
        {
            //: abv 13.05.02: OCC320 - fail (to remove edge) if two consecutive degenerated edges w/o
            //: pcurves
            if (Tool.Degenerated(e2))
            {
                _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
            }

            return false;
        }

        Vertex vp = sae.FirstVertex(e1); //: i9
        Vertex v0 = sae.LastVertex(e1);
        Vertex v1 = sae.FirstVertex(e2);
        Vertex v2 = sae.LastVertex(e2);

        if (vp.IsNull || v0.IsNull || v1.IsNull || v2.IsNull)
        {
            return false;
        }

        Point3D pp = Tool.Pnt(vp); //: i9
        Point3D p0 = Tool.Pnt(v0);
        Point3D p1 = Tool.Pnt(v1);
        Point3D p2 = Tool.Pnt(v2);
        double par1 = 0.0, par2 = 0.0;
        bool   lack = false;
        bool   dgnr = false;
        // pdn 12.03.99 minimal value processing first
        double precFirst = Min(_precision, Tool.Tolerance(v1));
        double precFin   = Max(_precision, Tool.Tolerance(v1));
        double precVtx   = (_precision < Tool.Tolerance(v1) ? 2 * precFin : precFin);

        //  forward : si Edge <num> FWD/REV. Si LACK, toujours True
        bool forward = (e2.Orientation == Forward);
        //  FIX FEV 1998 : recompute singularity according precision
        if (p1.DistanceTo(p2) <= precFirst)
        { // edge DGNR
            dgnr = _surf!.DegeneratedValues(p1, precVtx, ref p2d1, ref p2d2, ref par1, ref par2, forward); // smh#9
            if (dgnr)
            { // abv 24 Feb 00: trj3_as1-ac-214.stp #6065: avoid making closed edge degenerated
                Curve3D? c3d = Tool.Curve(e2, out double a, out double b);
                if (c3d is not null)
                {
                    Point3D p = c3d.Value(0.5 * (a + b));
                    if (p.SquareDistanceTo(p1) > precVtx * precVtx)
                    {
                        dgnr = false;
                    }
                }
            }
        }

        if (!dgnr)
        {
            //: i9 abv 23 Sep 98: CTS20315-2 #63231: check that previous edge is not degenerated
            if (n1 != n2 && p1.DistanceTo(pp) <= precFirst && _surf!.IsDegenerated(pp, precFirst)
                && !Tool.Degenerated(e1))
            {
                return false;
            }

            // rln S4135 ShapeAnalysis_Surface new algorithms for singularities
            //: 45 by abv 16 Dec 97: BUC60035 2659: precision increased to vertex tolerance
            //: 51 abv 22 Dec 97: recompute singularities if necessary
            if (p0.DistanceTo(p1) <= precFin)
            { // ou DGNR manquante ?
                // rln S4135 singularity with precision = 2 * prec, but distance <= prec
                double tmpPreci = 0.0;
                Point3D tmpP3d   = new Point3D();
                bool   tmpUIsoDeg = false;
                // #77 rln S4135: using singularity which has minimum gap between singular point and input 3D
                // point
                int    indMin  = -1;
                double gapMin2 = RealLast();
                for (int i = 1; i <= _surf!.NbSingularities(precVtx); i++)
                {
                    _surf.Singularity(i, ref tmpPreci, ref tmpP3d, ref p2d1, ref p2d2, ref par1, ref par2, ref tmpUIsoDeg);
                    double gap2 = p1.SquareDistanceTo(tmpP3d);
                    if (gap2 <= precVtx * precVtx)
                    {
                        if (gapMin2 > gap2)
                        {
                            gapMin2 = gap2;
                            indMin  = i;
                        }
                    }
                }

                if (indMin >= 1)
                {
                    _surf.Singularity(indMin, ref tmpPreci, ref tmpP3d, ref p2d1, ref p2d2, ref par1, ref par2, ref tmpUIsoDeg);
                    lack = true;
                }
            }
        }

        // there you go, we either have degenerate or lack
        if (!lack && !dgnr)
        {
            //: abv 29.08.01: if singularity not detected but edge is marked
            // as degenerated, report fail
            if (Tool.Degenerated(e2) && !sae.HasPCurve(e2, Face()))
            {
                _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
            }

            return false;
        }

        // OK, degenerated case detected; we will find its start and end in 2d

        if (lack)
        {
            forward = true;
        }

        //: 24 by abv 28 Nov 97:
        // make degenerative pcurve parametrized exactly from end of pcurve of the
        // previous edge to the start of the next one
        if (lack || n1 != n2)
        { //: i8 abv 18 Sep 98: ProSTEP TR9 r0501-ug.stp #182180: single degedge is a wire at apex of a
            //: cone
            if (sae.PCurve(e1, _face, out Curve2D? c2d, out double a, out double b, true))
            {
                p2d1 = c2d!.Value(b);
            }
            else
            {
                _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
            }

            // pdn pcurves (fixing regression in f0 in degenerated case)
            if (sae.PCurve((dgnr ? e3 : e2), _face, out c2d, out a, out b, true))
            {
                p2d2 = c2d!.Value(a);
            }
            else
            {
                _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
            }
        }

        // #84 rln 18.03.99 if pcurve is not degenerate anymore, the fix is postponned
        // to ShapeFix_Wire::FixLacking
        if (!_surf!.IsDegenerated(p2d1, p2d2, precVtx, 10.0))
        { //: s1 abv 22 Apr 99: PRO7226 #489490 //smh#9
            //: abv 24.05.02: OCC320 - fail (to remove edge) if two consecutive degenerated edges w/o
            //: pcurves
            if (Tool.Degenerated(e2))
            {
                _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
            }

            return false;
        }

        // added by rln 18/12/97 CSR# CTS18544 entity 13638
        // the situation when degenerated edge already exists but flag is not set
        //(i.e. the parametric space is closed)
        SurfaceAdaptor ads = _surf.Adaptor3d()!;
        double              max = Max(ads.UResolution(_precision), ads.VResolution(_precision));
        if (p2d1.DistanceTo(p2d2) /*Abs (par1 - par2)*/ <= max + Constants.Resolution())
        {
            return false;
        }

        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(dgnr ? ExtendStatus.DONE2 : ExtendStatus.DONE1);
        return true;
    }

    /// <summary>Checks for a degenerated edge between two adjacent ones (see the other overload).</summary>
    public bool CheckDegenerated(int num)
    {
        Point2D p2d1 = new Point2D(), p2d2 = new Point2D();
        return CheckDegenerated(num, ref p2d1, ref p2d2);
    }

    /// <summary>
    /// Checks the 3D gap between the ends of the 3d curves of the num-th and the preceding edge (in
    /// <see cref="MinDistance3d"/>). Status: DONE the gap is over the precision; FAIL an edge has no 3d curve.
    /// </summary>
    public bool CheckGap3d(int num = 0)
    {
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        // szv#4:S4163:12Mar99 optimized
        if (!IsLoaded || EdgeCount < 1)
        {
            return false; // szvsh was nbedges < 2
        }

        int                n2 = (num > 0 ? num : EdgeCount);
        int                n1 = (n2 > 1 ? n2 - 1 : EdgeCount);
        Edge        e1 = _wire!.Edge(n1);
        Edge        e2 = _wire.Edge(n2);
        Curve3D?        c2 = null;
        double             uf2 = 0.0, ul1;
        EdgeAnalysis sae = new EdgeAnalysis();
        if (!sae.Curve3d(e1, out Curve3D? c1, out _, out ul1) || !sae.Curve3d(e2, out c2, out uf2, out _))
        {
            _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
            return false;
        }

        Point3D p1 = c1!.Value(ul1);
        Point3D p2 = c2!.Value(uf2);
        _min3d = _max3d = p1.DistanceTo(p2);
        if (_min3d > _precision)
        {
            _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
        }

        return LastCheckStatus(ExtendStatus.Done);
    }

    /// <summary>
    /// Checks the 2D gap between the ends of the pcurves of the num-th and the preceding edge (in
    /// <see cref="MinDistance2d"/>). Status: DONE the gap is over the parametric precision; FAIL an edge has no pcurve.
    /// </summary>
    public bool CheckGap2d(int num = 0)
    {
        if (_face.IsNull || _surf is null)
        {
            _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
            return false;
        }

        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        // szv#4:S4163:12Mar99 optimized
        if (!IsReady || EdgeCount < 1)
        {
            return false; // szvsh was nbedges < 2
        }

        int                n2 = (num > 0 ? num : EdgeCount);
        int                n1 = (n2 > 1 ? n2 - 1 : EdgeCount);
        Edge        e1 = _wire!.Edge(n1);
        Edge        e2 = _wire.Edge(n2);
        Curve2D?      c2 = null;
        double             uf2 = 0.0, ul1;
        EdgeAnalysis sae = new EdgeAnalysis();
        if (!sae.PCurve(e1, _face, out Curve2D? c1, out _, out ul1) || !sae.PCurve(e2, _face, out c2, out uf2, out _))
        {
            _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
            return false;
        }

        Point2D p1 = c1!.Value(ul1);
        Point2D p2 = c2!.Value(uf2);
        _min2d = _max2d = p1.DistanceTo(p2);
        SurfaceAdaptor sa = _surf.Adaptor3d()!;
        if (_min2d > (Max(sa.UResolution(_precision), sa.VResolution(_precision)) + Kerf.Engine.Foundation.Precision.PConfusion()))
        {
            _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
        }

        return LastCheckStatus(ExtendStatus.Done);
    }

    /// <summary>
    /// Checks the gap between the 3d curve of the num-th edge and its pcurve on the surface, at 45 points (in
    /// <see cref="MinDistance3d"/>). Status: DONE the gap is over the precision; FAIL no 3d curve or no pcurve.
    /// </summary>
    public bool CheckCurveGap(int num = 0)
    {
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (!IsLoaded || EdgeCount < 1)
        {
            return false;
        }

        int                n   = (num > 0 ? num : EdgeCount);
        Edge        e   = _wire!.Edge(n);
        EdgeAnalysis sae = new EdgeAnalysis();
        if (!sae.Curve3d(e, out Curve3D? c, out double cuf, out double cul, false))
        {
            _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
            return false;
        }

        if (!sae.PCurve(e, _face, out Curve2D? pc, out double pcuf, out double pcul, false))
        {
            _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
            return false;
        }

        CurveAdaptor2D      ac  = new CurveAdaptor2D(pc!, pcuf, pcul);
        SurfaceAdaptor      AS  = new SurfaceAdaptor(_surf!.Surface()!);
        CurveOnSurface acs = new CurveOnSurface(ac, AS);
        Point3D                   cpnt, pcpnt;
        int                      nbp = 45;
        double                   dist, maxdist = 0.0;
        for (int i = 0; i < nbp; i++)
        {
            cpnt  = c!.Value(cuf + i * (cul - cuf) / (nbp - 1));
            pcpnt = acs.Value(pcuf + i * (pcul - pcuf) / (nbp - 1));
            dist  = cpnt.SquareDistanceTo(pcpnt);
            if (maxdist < dist)
            {
                maxdist = dist;
            }
        }

        _min3d = _max3d = Math.Sqrt(maxdist);
        if (_min3d > _precision)
        {
            _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
        }

        return LastCheckStatus(ExtendStatus.Done);
    }

    // auxiliary function
    //: h0 abv 29 May 98: PRO10105 1949: like in BRepCheck, point is to be taken
    // from 3d curve (but only if edge is SameParameter)
    private static Point3D GetPointOnEdge(in Edge edge, SurfaceAnalysis surf, CurveAdaptor2D crv2d, double param)
    {
        if (Tool.SameParameter(edge))
        {
            Curve3D? conS = Tool.Curve(edge, out Placement l, out _, out _);
            if (conS is not null)
            {
                return conS.Value(param).Transformed(l.Transformation);
            }
        }

        Point2D p2d = crv2d.Value(param);
        return surf.Adaptor3d()!.Value(p2d.X, p2d.Y);
    }

    /// <summary>
    /// Checks if the num-th edge is self-intersecting (on its pcurve), away from both its end vertices. Fills the
    /// intersection points and their 3d points. Status: DONE1 found; FAIL1 no pcurve; FAIL2 no vertices.
    /// </summary>
    public bool CheckSelfIntersectingEdge(int num, NCollection_Sequence<IntersectionPoint2D> points2d,
                                          NCollection_Sequence<Point3D> points3d)
    {
        points2d.Clear();
        points3d.Clear();
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (!IsReady)
        {
            return false;
        }

        Edge        edge = WireData()!.Edge(num > 0 ? num : EdgeCount);
        EdgeAnalysis sae  = new EdgeAnalysis();

        if (!sae.PCurve(edge, _face, out Curve2D? crv, out double a, out double b, false))
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
            return false;
        }

        if (Math.Abs(a - b) <= Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return false;
        }

        double tolint = 1.0e-10;
        // szv#4:S4163:12Mar99 warning
        TwoDDomain     domain = new TwoDDomain(crv!.Value(a), a, tolint, crv.Value(b), b, tolint);
        CurveAdaptor2D ac     = new CurveAdaptor2D(crv);
        GInter    inter  = new GInter(ac, domain, tolint, tolint);
        if (!inter.IsDone)
        {
            return false;
        }

        Vertex v1 = sae.FirstVertex(edge);
        Vertex v2 = sae.LastVertex(edge);
        if (v1.IsNull || v2.IsNull)
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
            return false;
        }

        double tol1 = Tool.Tolerance(v1);
        double tol2 = Tool.Tolerance(v2);
        Point3D pnt1 = Tool.Pnt(v1);
        Point3D pnt2 = Tool.Pnt(v2);
        for (int i = 1; i <= inter.PointCount; i++)
        {
            IntersectionPoint2D ip  = inter.Point(i);
            Transition        tr1 = ip.TransitionOfFirst();
            Transition        tr2 = ip.TransitionOfSecond();
            if (tr1.PositionOnCurve() != Position2D.Middle && tr2.PositionOnCurve() != Position2D.Middle)
            {
                continue;
            }

            Point3D pint   = GetPointOnEdge(edge, _surf!, ac, ip.ParamOnFirst());
            double dist21 = pnt1.SquareDistanceTo(pint);
            double dist22 = pnt2.SquareDistanceTo(pint);
            if (dist21 > tol1 * tol1 && dist22 > tol2 * tol2)
            {
                points2d.Append(ip);
                points3d.Append(pint);
                _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
            }
        }

        return LastCheckStatus(ExtendStatus.Done);
    }

    /// <summary>Checks if the num-th edge is self-intersecting (see the other overload).</summary>
    public bool CheckSelfIntersectingEdge(int num)
    {
        NCollection_Sequence<IntersectionPoint2D> points2d = new NCollection_Sequence<IntersectionPoint2D>();
        NCollection_Sequence<Point3D>                     points3d = new NCollection_Sequence<Point3D>();
        return CheckSelfIntersectingEdge(num, points2d, points3d);
    }

    /// <summary>
    /// Checks if the num-th edge and the preceding one intersect (on their pcurves) at a point not enclosed by their
    /// common vertex, as BRepCheck_Wire does. Fills the intersection points, their 3d points and errors (half the
    /// distance between the 3d points from either edge). Status: DONE1 found; FAIL1 no vertices; FAIL2 the edges don't
    /// share a vertex; FAIL3 no pcurve.
    /// </summary>
    public bool CheckIntersectingEdges(int num, NCollection_Sequence<IntersectionPoint2D> points2d,
                                       NCollection_Sequence<Point3D> points3d, NCollection_Sequence<double> errors)
    {
        points2d.Clear();
        points3d.Clear();
        errors.Clear();
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (!IsReady || EdgeCount < 2)
        {
            return false;
        }

        // szv#4:S4163:12Mar99 optimized
        int         n2    = (num > 0) ? num : EdgeCount;
        int         n1    = (n2 > 1) ? n2 - 1 : EdgeCount;
        Edge edge1 = _wire!.Edge(n1);
        Edge edge2 = _wire.Edge(n2);

        EdgeAnalysis sae = new EdgeAnalysis();
        Vertex      v1  = sae.LastVertex(edge1);
        Vertex      v2  = sae.FirstVertex(edge2);
        if (v1.IsNull || v2.IsNull)
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
            return false;
        }

        if (!BrepTools.Compare(v1, v2))
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
            return false;
        }

        Vertex vp = sae.FirstVertex(edge1);
        Vertex vn = sae.LastVertex(edge2);

        if (!sae.PCurve(edge1, _face, out Curve2D? crv1, out double a1, out double b1, false))
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL3);
            return false;
        }

        if (!sae.PCurve(edge2, _face, out Curve2D? crv2, out double a2, out double b2, false))
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL3);
            return false;
        }

        if (Math.Abs(a1 - b1) <= Kerf.Engine.Foundation.Precision.PConfusion() || Math.Abs(a2 - b2) <= Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return false; //:f7 abv 6 May 98: BUC50070 on #42276
        }

        bool isForward1 = (edge1.Orientation == Forward);
        bool isForward2 = (edge2.Orientation == Forward);

        double tol0 = Max(Tool.Tolerance(v1), Tool.Tolerance(v2));
        double tol  = tol0;

        Point3D pnt = Tool.Pnt(v1);

        double tolint = 1.0e-10;
        // szv#4:S4163:12Mar99 warning
        CurveAdaptor2D c1 = new CurveAdaptor2D(crv1!), c2 = new CurveAdaptor2D(crv2!);
        TwoDDomain     d1 = new TwoDDomain(c1.Value(a1), a1, tolint, c1.Value(b1), b1, tolint);
        TwoDDomain     d2 = new TwoDDomain(c2.Value(a2), a2, tolint, c2.Value(b2), b2, tolint);

        //: 64 abv 25 Dec 97: Attention!
        // Since Intersection algorithm is not symmetrical, for consistency with BRepCheck
        // edge with lower order number should be intersecting with edge with higher one
        // i.e., for intersection of last and first edges, they should go in reversed order
        // Example: entity #38285 from bug CSR #CTS17806
        // NOTE: Tr1 and Tr2 are not reordered because they are used in the same manner
        GInter inter = new GInter();
        if (num == 1)
        {
            inter.Perform(c2, d2, c1, d1, tolint, tolint);
        }
        else
        {
            inter.Perform(c1, d1, c2, d2, tolint, tolint);
        }

        if (!inter.IsDone)
        {
            return false;
        }

        //: 86 abv 22 Jan 98: fix self-intersection even if tolerance of vertex is enough
        // to annihilate it. This is done to prevent wrong effects if vertex tolerance
        // will be decreased (e.g., in FixLacking)
        double tole = Max((Tool.SameParameter(edge1) ? Tool.Tolerance(edge1) : tol0),
                          (Tool.SameParameter(edge2) ? Tool.Tolerance(edge2) : tol0));
        double tolt = Min(tol, Max(tole, _precision));
        int isLacking = -1; //: l0 abv: CATIA01 #1727: protect against adding lacking
        // #83 rln 19.03.99 sim2.igs, entity 4292
        // processing also segments as in BRepCheck
        int nbPoints = inter.PointCount, nbSegments = inter.SegmentCount;
        for (int i = 1; i <= nbPoints + nbSegments; i++)
        {
            IntersectionPoint2D ip;
            Transition        tr1, tr2;
            if (i <= nbPoints)
            {
                ip = inter.Point(i);
            }
            else
            {
                IntersectionSegment2D seg = inter.Segment(i - nbPoints);
                if (!seg.HasFirstPoint || !seg.HasLastPoint)
                {
                    continue;
                }

                ip  = seg.FirstPoint();
                tr1 = ip.TransitionOfFirst();
                tr2 = ip.TransitionOfSecond();
                if (tr1.PositionOnCurve() == Position2D.Middle || tr2.PositionOnCurve() == Position2D.Middle)
                {
                    ip = seg.LastPoint();
                }
            }

            tr1 = ip.TransitionOfFirst();
            tr2 = ip.TransitionOfSecond();
            if (tr1.PositionOnCurve() != Position2D.Middle && tr2.PositionOnCurve() != Position2D.Middle)
            {
                continue;
            }

            double param1, param2;
            param1 = (num == 1 ? ip.ParamOnSecond() : ip.ParamOnFirst());
            param2 = (num == 1 ? ip.ParamOnFirst() : ip.ParamOnSecond());

            //: r6 abv 8 Apr 99: r_47-sd.stp #173850: protect against working out of curve range
            if (a1 - param1 > Kerf.Engine.Foundation.Precision.PConfusion() || param1 - b1 > Kerf.Engine.Foundation.Precision.PConfusion()
                || a2 - param2 > Kerf.Engine.Foundation.Precision.PConfusion() || param2 - b2 > Kerf.Engine.Foundation.Precision.PConfusion())
            {
                continue;
            }

            //: 82 abv 21 Jan 98: point of intersection on Crv1 and Crv2 is different
            Point3D pi1   = GetPointOnEdge(edge1, _surf!, c1, param1); //:h0: thesurf.Value ( Crv1->Value ( param1 ) );
            Point3D pi2   = GetPointOnEdge(edge2, _surf!, c2, param2); //:h0: thesurf.Value ( Crv2->Value ( param2 ) );
            Point3D pint  = new Point3D(0.5 * (pi1.XYZ() + pi2.XYZ()));
            double di1   = pi1.SquareDistanceTo(pnt);
            double di2   = pi2.SquareDistanceTo(pnt);
            double dist2 = Max(di1, di2);

            // rln 03/02/98: CSR#BUC50004 entity 56 (to avoid later inserting lacking edge)
            if (isLacking < 0)
            { //: l0
                Point2D end1 = crv1!.Value(isForward1 ? b1 : a1);
                Point2D end2 = crv2!.Value(isForward2 ? a2 : b2);
                //: l0 test like in BRepCheck
                SurfaceAdaptor ads   = _surf!.Adaptor3d()!;
                double              tol2d = 2 * Max(ads.UResolution(tol), ads.VResolution(tol));
                isLacking                 = (end1.SquareDistanceTo(end2) >= tol2d * tol2d) ? 1 : 0;
            }

            if ((dist2 > tolt * tolt || //: 86: tol -> tolt
                 isLacking != 0)
                && //: l0
                (!BrepTools.Compare(vp, vn) || //: 63
                 dist2 < pint.SquareDistanceTo(Tool.Pnt(vp))))
            { //: 63
                points2d.Append(ip);
                points3d.Append(pint);
                errors.Append(0.5 * pi1.DistanceTo(pi2));
                _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
            }
        }

        return LastCheckStatus(ExtendStatus.Done);
    }

    /// <summary>Checks if the num-th edge and the preceding one intersect (see the other overload).</summary>
    public bool CheckIntersectingEdges(int num)
    {
        NCollection_Sequence<IntersectionPoint2D> points2d = new NCollection_Sequence<IntersectionPoint2D>();
        NCollection_Sequence<Point3D>                     points3d = new NCollection_Sequence<Point3D>();
        NCollection_Sequence<double>                     errors   = new NCollection_Sequence<double>();
        return CheckIntersectingEdges(num, points2d, points3d, errors);
    }

    /// <summary>
    /// Checks if the num1-th and num2-th edges intersect (on their pcurves) at a point not enclosed by their vertices.
    /// Status: DONE1 found; FAIL3 no pcurve.
    /// </summary>
    public bool CheckIntersectingEdges(int num1, int num2, NCollection_Sequence<IntersectionPoint2D> points2d,
                                       NCollection_Sequence<Point3D> points3d, NCollection_Sequence<double> errors)
    {
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (!IsReady)
        {
            return false;
        }

        ExtendWireData sbwd = WireData()!;
        int                  n2   = (num2 > 0 ? num2 : sbwd.EdgeCount);
        int                  n1   = (num1 > 0 ? num1 : sbwd.EdgeCount);

        Edge edge1 = sbwd.Edge(n1);
        Edge edge2 = sbwd.Edge(n2);

        EdgeAnalysis sae = new EdgeAnalysis();

        if (!sae.PCurve(edge1, _face, out Curve2D? crv1, out double a1, out double b1, false))
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL3);
            return false;
        }

        if (!sae.PCurve(edge2, _face, out Curve2D? crv2, out double a2, out double b2, false))
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL3);
            return false;
        }

        if (Math.Abs(a1 - b1) <= Kerf.Engine.Foundation.Precision.PConfusion() || Math.Abs(a2 - b2) <= Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return false;
        }

        points2d.Clear();
        points3d.Clear();
        errors.Clear();
        NCollection_Array1<Point3D> vertexPoints = new NCollection_Array1<Point3D>(1, 4);
        NCollection_Array1<double> vertexTolers = new NCollection_Array1<double>(1, 4);
        vertexPoints[1] = Tool.Pnt(sae.FirstVertex(edge1));
        vertexTolers[1] = Tool.Tolerance(sae.FirstVertex(edge1));
        vertexPoints[2] = Tool.Pnt(sae.LastVertex(edge1));
        vertexTolers[2] = Tool.Tolerance(sae.LastVertex(edge1));
        vertexPoints[3] = Tool.Pnt(sae.FirstVertex(edge2));
        vertexTolers[3] = Tool.Tolerance(sae.FirstVertex(edge2));
        vertexPoints[4] = Tool.Pnt(sae.LastVertex(edge2));
        vertexTolers[4] = Tool.Tolerance(sae.LastVertex(edge2));

        double tolint = 1.0e-10;
        TwoDDomain     d1 = new TwoDDomain(crv1!.Value(a1), a1, tolint, crv1.Value(b1), b1, tolint);
        TwoDDomain     d2 = new TwoDDomain(crv2!.Value(a2), a2, tolint, crv2.Value(b2), b2, tolint);
        CurveAdaptor2D c1 = new CurveAdaptor2D(crv1), c2 = new CurveAdaptor2D(crv2);

        GInter inter = new GInter();
        inter.Perform(c1, d1, c2, d2, tolint, tolint);
        if (!inter.IsDone)
        {
            return false;
        }

        // #83 rln 19.03.99 sim2.igs, entity 4292
        // processing also segments as in BRepCheck
        int nbPoints = inter.PointCount, nbSegments = inter.SegmentCount;
        for (int i = 1; i <= nbPoints + nbSegments; i++)
        {
            IntersectionPoint2D ip;
            Transition        tr1, tr2;
            if (i <= nbPoints)
            {
                ip = inter.Point(i);
            }
            else
            {
                IntersectionSegment2D seg = inter.Segment(i - nbPoints);
                if (!seg.HasFirstPoint || !seg.HasLastPoint)
                {
                    continue;
                }

                ip  = seg.FirstPoint();
                tr1 = ip.TransitionOfFirst();
                tr2 = ip.TransitionOfSecond();
                if (tr1.PositionOnCurve() == Position2D.Middle || tr2.PositionOnCurve() == Position2D.Middle)
                {
                    ip = seg.LastPoint();
                }
            }

            tr1 = ip.TransitionOfFirst();
            tr2 = ip.TransitionOfSecond();
            if (tr1.PositionOnCurve() != Position2D.Middle && tr2.PositionOnCurve() != Position2D.Middle)
            {
                continue;
            }

            double param1 = ip.ParamOnFirst();
            double param2 = ip.ParamOnSecond();
            Point3D pi1    = GetPointOnEdge(edge1, _surf!, c1, param1); //:h0: thesurf.Value ( Crv1->Value ( param1 ) );
            Point3D pi2    = GetPointOnEdge(edge2, _surf!, c2, param2);
            bool   ok1    = false;
            bool   ok2    = false;

            for (int j = 1; (j <= 2) && !ok1; j++)
            {
                double di1 = pi1.SquareDistanceTo(vertexPoints[j]);
                if (di1 < vertexTolers[j] * vertexTolers[j])
                {
                    ok1 = true;
                }
            }

            for (int j = 3; (j <= 4) && !ok2; j++)
            {
                double di2 = pi2.SquareDistanceTo(vertexPoints[j]);
                if (di2 < vertexTolers[j] * vertexTolers[j])
                {
                    ok2 = true;
                }
            }

            if (!ok1 || !ok2)
            {
                Point3D pint = new Point3D(0.5 * (pi1.XYZ() + pi2.XYZ()));
                points2d.Append(ip);
                points3d.Append(pint);
                errors.Append(0.5 * pi1.DistanceTo(pi2));
                _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
            }
        }

        return LastCheckStatus(ExtendStatus.Done);
    }

    /// <summary>Checks if the num1-th and num2-th edges intersect (see the other overload).</summary>
    public bool CheckIntersectingEdges(int num1, int num2)
    {
        NCollection_Sequence<IntersectionPoint2D> points2d = new NCollection_Sequence<IntersectionPoint2D>();
        NCollection_Sequence<Point3D>                     points3d = new NCollection_Sequence<Point3D>();
        NCollection_Sequence<double>                     errors   = new NCollection_Sequence<double>();
        return CheckIntersectingEdges(num1, num2, points2d, points3d, errors);
    }

    /// <summary>
    /// Checks if there is a 2d gap between the num-th edge and the preceding one not covered by the tolerance of their
    /// common vertex (or by <paramref name="tolerance"/> when it is positive and smaller). <paramref name="p2d1"/> and
    /// <paramref name="p2d2"/> get the end of the preceding pcurve and the start of the num-th one.
    /// </summary>
    /// <remarks>
    /// Status: DONE1 a gap is found; DONE2 with DONE1 when the gap goes against the pcurves (an angle over 0.9 π); FAIL1
    /// no vertices; FAIL2 the edges don't share a vertex; FAIL3 no pcurve.
    /// </remarks>
    public bool CheckLacking(int num, double tolerance, ref Point2D p2d1, ref Point2D p2d2)
    {
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (!IsReady)
        {
            return false;
        }

        // szv#4:S4163:12Mar99 optimized
        int         n2 = (num > 0) ? num : EdgeCount;
        int         n1 = (n2 > 1) ? n2 - 1 : EdgeCount;
        Edge e1 = _wire!.Edge(n1);
        Edge e2 = _wire.Edge(n2);

        EdgeAnalysis sae = new EdgeAnalysis();
        Vertex      V1  = sae.LastVertex(e1);
        Vertex      V2  = sae.FirstVertex(e2);

        // CKY 4 MAR 1998 : protection against null vertex
        if (V1.IsNull || V2.IsNull)
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
            return false;
        }

        if (!BrepTools.Compare(V1, V2))
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
            return false;
        }

        Vector2D v1, v2, v12;

        if (!sae.PCurve(e1, _face, out Curve2D? c2d, out double a, out double b, true))
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL3);
            return false;
        }

        CurveAdaptor2D adapt = new CurveAdaptor2D(c2d!);
        adapt.D1(b, out p2d1, out v1);
        if (e1.Orientation == Reversed)
        {
            v1 = v1.Reversed();
        }

        if (!sae.PCurve(e2, _face, out c2d, out a, out b, true))
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL3);
            return false;
        }

        adapt.Load(c2d!);
        adapt.D1(a, out p2d2, out v2);
        if (e2.Orientation == Reversed)
        {
            v2 = v2.Reversed();
        }

        v12     = new Vector2D(p2d2.XY() - p2d1.XY());
        _max2d = v12.SquareMagnitude;

        // test like in BRepCheck
        double tol = Max(Tool.Tolerance(V1), Tool.Tolerance(V2));
        tol        = (tolerance > Constants.Resolution() && tolerance < tol ? tolerance : tol);
        SurfaceAdaptor ads   = _surf!.Adaptor3d()!;
        double              tol2d = 2 * Max(ads.UResolution(tol), ads.VResolution(tol));
        if ( // tol2d < gp::Resolution() || //#2 smh 26.03.99 S4163 Zero divide
            _max2d < tol2d * tol2d)
        {
            return false;
        }

        _max2d = Math.Sqrt(_max2d);
        _max3d = tol * _max2d / Max(tol2d, Constants.Resolution());
        _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
        if (_max2d < Kerf.Engine.Foundation.Precision.PConfusion() || //: abv 03.06.02 CTS21866.stp
            (v1.SquareMagnitude > Constants.Resolution() && Math.Abs(v12.AngleTo(v1)) > 0.9 * M_PI)
            || (v2.SquareMagnitude > Constants.Resolution() && Math.Abs(v12.AngleTo(v2)) > 0.9 * M_PI))
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
        }

        return true;
    }

    /// <summary>
    /// Checks if there is a 2d gap between the num-th edge and the preceding one not covered by the vertex tolerance
    /// (see the other overload).
    /// </summary>
    public bool CheckLacking(int num, double tolerance = 0.0)
    {
        Point2D p1 = new Point2D(), p2 = new Point2D();
        return CheckLacking(num, tolerance, ref p1, ref p2);
    }

    /// <summary>
    /// Checks if the wire is an outer bound of its face (ShapeAnalysis.IsOuterBound on a copy of the face holding only
    /// it). With <paramref name="apiMake"/> the wire is built with BRepBuilderAPI_MakeWire, else with BRep_Builder (only
    /// when the edges share their vertices). Status: DONE1 not outer.
    /// </summary>
    public bool CheckOuterBound(bool apiMake = true)
    {
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (!IsReady)
        {
            return false;
        }

        Wire wire;
        if (apiMake)
        {
            wire = _wire!.InternalWireAPIMake();
        }
        else
        {
            wire = _wire!.InternalWire();
        }

        Shape sh   = _face.EmptyCopied(); // szv#4:S4163:12Mar99 SGI warns
        Face  face = ShapeCasts.Face(sh);
        BrepBuilder b    = new BrepBuilder();
        b.Add(face, wire);
        if (ShapeAnalysisAnalysis.IsOuterBound(face))
        {
            return false;
        }

        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
        return true;
    }

    private static double ProjectInside(CurveOnSurface ad, in Point3D pnt, double preci, out Point3D proj,
                                        out double param, bool adjustToEnds = true)
    {
        CurveAnalysis sac    = new CurveAnalysis();
        double              dist   = sac.Project(ad, pnt, preci, out proj, out param, adjustToEnds);
        double              uFirst = ad.FirstParameter();
        double              uLast  = ad.LastParameter();
        if (param < uFirst)
        {
            param = uFirst;
            proj  = ad.Value(uFirst);
            return proj.DistanceTo(pnt);
        }

        if (param > uLast)
        {
            param = uLast;
            proj  = ad.Value(uLast);
            return proj.DistanceTo(pnt);
        }

        return dist;
    }

    /// <summary>
    /// Detects a notch: the num-th edge and the preceding one leave their common vertex in the same direction (within
    /// 0.1 rad) and one runs along the other within <paramref name="tolerance"/>. <paramref name="shortNum"/> gets the
    /// number of the shorter edge and <paramref name="param"/> the parameter on the longer one where the shorter ends
    /// (both left unchanged when false is returned early). Status: FAIL1 no vertices; FAIL2 the edges don't share a
    /// vertex; FAIL3 no pcurve.
    /// </summary>
    public bool CheckNotchedEdges(int num, ref int shortNum, ref double param, double tolerance = 0.0)
    {
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (!IsReady)
        {
            return false;
        }

        int         n2 = (num > 0) ? num : EdgeCount;
        int         n1 = (n2 > 1) ? n2 - 1 : EdgeCount;
        Edge e1 = _wire!.Edge(n1);
        Edge e2 = _wire.Edge(n2);

        if (Tool.Degenerated(e1) || Tool.Degenerated(e2))
        {
            return false;
        }

        EdgeAnalysis sae = new EdgeAnalysis();
        Vertex      V1  = sae.LastVertex(e1);
        Vertex      V2  = sae.FirstVertex(e2);

        if (V1.IsNull || V2.IsNull)
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
            return false;
        }

        if (!BrepTools.Compare(V1, V2))
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
            return false;
        }

        Point2D p2d1, p2d2;
        Vector2D v1, v2;
        if (!sae.PCurve(e1, _face, out Curve2D? c2d1, out double a1, out double b1, false))
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL3);
            return false;
        }

        if (e1.Orientation == Reversed)
        {
            c2d1!.D1(a1, out p2d1, out v1);
        }
        else
        {
            c2d1!.D1(b1, out p2d1, out v1);
            v1 = v1.Reversed();
        }

        if (!sae.PCurve(e2, _face, out Curve2D? c2d2, out double a2, out double b2, false))
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL3);
            return false;
        }

        if (e2.Orientation == Reversed)
        {
            c2d2!.D1(b2, out p2d2, out v2);
            v2 = v2.Reversed();
        }
        else
        {
            c2d2!.D1(a2, out p2d2, out v2);
        }

        if (v2.Magnitude < Constants.Resolution() || v1.Magnitude < Constants.Resolution())
        {
            return false;
        }

        if (Math.Abs(v2.AngleTo(v1)) > 0.1 || p2d1.DistanceTo(p2d2) > tolerance)
        {
            return false;
        }

        CurveAdaptor2D      ac2d1 = new CurveAdaptor2D(c2d1, a1, b1);
        SurfaceAdaptor      adS1  = new SurfaceAdaptor(new PlaneSurface(new Plane3D()));
        CurveOnSurface ad1   = new CurveOnSurface(ac2d1, adS1);

        CurveAdaptor2D      ac2d2 = new CurveAdaptor2D(c2d2, a2, b2);
        SurfaceAdaptor      adS2  = new SurfaceAdaptor(new PlaneSurface(new Plane3D()));
        CurveOnSurface ad2   = new CurveOnSurface(ac2d2, adS2);

        CurveOnSurface longAD, shortAD;
        double                   lenP, firstP;

        CurveAnalysis sac = new CurveAnalysis();

        p2d2         = c2d2.Value(e2.Orientation == Forward ? b2 : a2);
        p2d1         = c2d1.Value(e1.Orientation == Forward ? a1 : b1);
        double dist1 = ProjectInside(ad1, new Point3D(p2d2.X, p2d2.Y, 0), tolerance, out Point3D proj1, out double param1, false);
        double dist2 = ProjectInside(ad2, new Point3D(p2d1.X, p2d1.Y, 0), tolerance, out _, out double param2, false);

        if (dist1 > tolerance && dist2 > tolerance)
        {
            return false;
        }

        if (dist1 < dist2)
        {
            shortAD  = ad2;
            longAD   = ad1;
            lenP     = b2 - a2;
            firstP   = a2;
            shortNum = n2;
            param    = param1;
        }
        else
        {
            shortAD  = ad1;
            longAD   = ad2;
            lenP     = b1 - a1;
            firstP   = a1;
            shortNum = n1;
            param    = param2;
        }

        double step = lenP / 23;
        for (int i = 1; i < 23; i++, firstP += step)
        {
            double d1 = sac.Project(longAD, shortAD.Value(firstP), tolerance, out proj1, out param1);
            if (d1 > tolerance)
            {
                return false;
            }
        }

        return true;
    }

    /// <summary>
    /// Checks if the wire has a parametric area less than the precision. Status: DONE1 small; FAIL1 not ready; FAIL2 an
    /// edge has no pcurve or no 3d curve.
    /// </summary>
    public bool CheckSmallArea(in Wire wire)
    {
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
        const int nbControl = 23;
        int nbEdges = _wire!.EdgeCount;
        if (!IsReady || nbEdges < 1)
        {
            return false;
        }

        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        double       f, l, length = 0.0;
        double       inv     = 1.0 / (double)(nbControl - 1);
        Xy        center2d = new Xy(0.0, 0.0);

        // try to find mid point for closed contour
        for (int j = 1; j <= nbEdges; ++j)
        {
            EdgeAnalysis analyzer = new EdgeAnalysis();
            if (!analyzer.PCurve(_wire.Edge(j), _face, out Curve2D? curve2d, out f, out l))
            {
                _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
                return false;
            }

            for (int i = 1; i < nbControl; ++i)
            {
                double v = inv * ((nbControl - 1 - i) * f + i * l);
                center2d += curve2d!.Value(v).XY();
            }
        }

        center2d *= 1.0 / (double)(nbEdges * (nbControl - 1));

        // check approximated area in 3D
        Point3D pnt3d  = new Point3D();
        Xyz prev3d = new Xyz(), cross = new Xyz(0.0, 0.0, 0.0);
        Xyz center = _surf!.Value(center2d.X(), center2d.Y()).XYZ();

        for (int j = 1; j <= nbEdges; ++j)
        {
            EdgeAnalysis analizer = new EdgeAnalysis();
            if (!analizer.Curve3d(_wire.Edge(j), out Curve3D? curve3d, out f, out l))
            {
                _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
                return false;
            }

            if (Kerf.Engine.Foundation.Precision.IsInfinite(f) || Kerf.Engine.Foundation.Precision.IsInfinite(l))
            {
                continue;
            }

            int begin = 0;
            if (j == 1)
            {
                begin  = 1;
                pnt3d  = curve3d!.Value(f);
                prev3d = pnt3d.XYZ() - center;
            }

            for (int i = begin; i < nbControl; ++i)
            {
                double u    = inv * ((nbControl - 1 - i) * f + i * l);
                Point3D pnt     = curve3d!.Value(u);
                Xyz current = pnt.XYZ();
                Xyz vec     = current - center;

                cross += prev3d ^ vec;
                length += pnt3d.DistanceTo(pnt);

                pnt3d  = pnt;
                prev3d = vec;
            }
        }

        double tolerance = length * _precision;
        if (cross.Modulus() < tolerance)
        {
            // check real area in 3D
            Properties props  = new Properties();
            Properties lProps = new Properties();
            Face  face   = ShapeCasts.Face(_face.EmptyCopied());
            new BrepBuilder().Add(face, wire);
            MassPropertiesBuilder.SurfaceProperties(face, props);
            MassPropertiesBuilder.LinearProperties(face, lProps);

            double newTolerance = lProps.Mass() * _precision;
            if (Math.Abs(props.Mass()) < 0.5 * newTolerance)
            {
                _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
                return true;
            }
        }

        return false;
    }

    /// <summary>
    /// Checks with what orientation <paramref name="shape"/> (a wire or an edge) can be connected to the wire (see the
    /// other overload).
    /// </summary>
    public bool CheckShapeConnect(in Shape shape, double prec = 0.0)
    {
        return CheckShapeConnect(out _, out _, out _, out _, shape, prec);
    }

    /// <summary>
    /// Checks with what orientation <paramref name="shape"/> (a wire or an edge) can be connected to the wire, and gives
    /// the distances between each end of the wire and each end of the shape (<paramref name="tailhead"/>: the wire's
    /// tail to the shape's head).
    /// </summary>
    /// <remarks>
    /// Status: DONE1 the shape follows the wire, direct; DONE2 follows, reversed; DONE3 precedes, direct; DONE4
    /// precedes, reversed; DONE5 to the wire's tail the shape is closer direct; DONE6 to its head the shape is closer
    /// direct; FAIL1 neither an edge nor a wire; FAIL2 can't be connected. The distances are 0 when FAIL1.
    /// </remarks>
    public bool CheckShapeConnect(out double tailhead, out double tailtail, out double headtail, out double headhead,
                                  in Shape shape, double prec = 0.0)
    {
        // Kerf: OCCT leaves the distances unchanged when it returns FAIL1; they are out here, set to 0
        tailhead = tailtail = headtail = headhead = 0.0;
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
        if (!IsLoaded || shape.IsNull)
        {
            return false;
        }

        Vertex      v1, v2;
        Edge        e;
        Wire        w;
        EdgeAnalysis sae = new EdgeAnalysis();
        if (shape.Type == ShapeType.Edge)
        {
            e  = ShapeCasts.Edge(shape);
            v1 = sae.FirstVertex(e);
            v2 = sae.LastVertex(e);
        }
        else if (shape.Type == ShapeType.Wire)
        {
            w = ShapeCasts.Wire(shape);
            ShapeAnalysisAnalysis.FindBounds(w, out v1, out v2);
        }
        else
        {
            return false;
        }

        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
        //  on va comparer les points avec ceux de thevfirst et thevlast
        Point3D p1 = Tool.Pnt(v1);
        Point3D p2 = Tool.Pnt(v2);

        Vertex vfirst = sae.FirstVertex(_wire!.Edge(1)),
                      vlast  = sae.LastVertex(_wire.Edge(EdgeCount));
        Point3D pf = Tool.Pnt(vfirst);
        Point3D pl = Tool.Pnt(vlast);

        tailhead   = p1.DistanceTo(pl);
        tailtail   = p2.DistanceTo(pl);
        headhead   = p1.DistanceTo(pf);
        headtail   = p2.DistanceTo(pf);
        double dm1 = tailhead, dm2 = headtail;
        int    res1 = 0, res2 = 0;
        if (tailhead > tailtail)
        {
            res1 = 1;
            dm1  = tailtail;
        }

        if (headtail > headhead)
        {
            res2 = 1;
            dm2  = headhead;
        }

        int result = res1;
        _min3d    = Min(dm1, dm2);
        _max3d    = Max(dm1, dm2);
        if (dm1 > dm2)
        {
            dm1    = dm2;
            result = res2 + 2;
        }

        switch (result)
        {
            case 1:
                _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
                break;
            case 2:
                _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE3);
                break;
            case 3:
                _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE4);
                break;
        }

        if (res1 == 0)
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE5);
        }

        if (res2 == 0)
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE6);
        }

        if (_min3d > Max(_precision, prec))
        {
            _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
        }

        return LastCheckStatus(ExtendStatus.Done);
    }

    private static bool IsMultiVertex(NCollection_List<Shape> alshape, HashSet<Shape> aMapSmallEdges,
                                      HashSet<Shape> aMapSeemEdges)
    {
        NCollection_List<Shape>.Iterator lIt1 = new NCollection_List<Shape>.Iterator(alshape);
        int                                     nbNotAccount = 0;

        for (; lIt1.More(); lIt1.Next())
        {
            if (aMapSmallEdges.Contains(lIt1.Value()))
            {
                nbNotAccount++;
            }
            else if (aMapSeemEdges.Contains(lIt1.Value()))
            {
                nbNotAccount++;
            }
        }

        return ((alshape.Extent() - nbNotAccount) > 2);
    }

    /// <summary>
    /// Checks the wire for loops: finds the vertices shared by more than two edges (seam, degenerated and small closed
    /// edges not counted). Fills the maps (keyed with TopTools_ShapeMapHasher) of loop vertices, vertex to edges, small
    /// edges and seam edges. Status: DONE1 loops found; FAIL2 an edge has no vertex.
    /// </summary>
    public bool CheckLoop(IndexedSet<Shape> aMapLoopVertices,
                          Dictionary<Shape, NCollection_List<Shape>> aMapVertexEdges,
                          HashSet<Shape> aMapSmallEdges, HashSet<Shape> aMapSeemEdges)
    {
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (!IsLoaded || EdgeCount < 2)
        {
            return false;
        }

        double savPreci = Precision();
        SetPrecision(Kerf.Engine.Foundation.Precision.Infinite());
        int i = 1;

        for (; i <= _wire!.EdgeCount; i++)
        {
            Edge aedge = _wire.Edge(i);
            Exploration.Vertices(aedge, out Vertex v1, out Vertex v2);
            if (v1.IsNull || v2.IsNull)
            {
                // OCCT: the precision set to Infinite above isn't restored on this return
                _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
                return false;
            }

            bool isSame = v1.IsSame(v2);
            if (_wire.IsSeam(i))
            {
                aMapSeemEdges.Add(aedge); // continue;
            }
            else if (Tool.Degenerated(aedge))
            {
                aMapSmallEdges.Add(aedge);
            }
            else if (isSame && CheckSmall(i, Tool.Tolerance(v1)))
            {
                aMapSmallEdges.Add(aedge);
            }

            if (!aMapVertexEdges.ContainsKey(v1))
            {
                NCollection_List<Shape> alshape = new NCollection_List<Shape>();
                aMapVertexEdges[v1] = alshape;
            }

            if (!aMapVertexEdges.ContainsKey(v2))
            {
                NCollection_List<Shape> alshape = new NCollection_List<Shape>();
                aMapVertexEdges[v2] = alshape;
            }

            if (isSame)
            {
                NCollection_List<Shape> alshape = aMapVertexEdges.GetValue(v1);
                alshape.Append(aedge);
                alshape.Append(aedge);
                if (alshape.Extent() > 2 && IsMultiVertex(alshape, aMapSmallEdges, aMapSeemEdges))
                {
                    aMapLoopVertices.Add(v1);
                }
            }
            else
            {
                NCollection_List<Shape> alshape = aMapVertexEdges.GetValue(v1);
                alshape.Append(aedge);
                if (alshape.Extent() > 2 && IsMultiVertex(alshape, aMapSmallEdges, aMapSeemEdges))
                {
                    aMapLoopVertices.Add(v1);
                }

                NCollection_List<Shape> alshape2 = aMapVertexEdges.GetValue(v2);
                alshape2.Append(aedge);
                if (alshape2.Extent() > 2 && IsMultiVertex(alshape2, aMapSmallEdges, aMapSeemEdges))
                {
                    aMapLoopVertices.Add(v2);
                }
            }
        }

        SetPrecision(savPreci);
        if (aMapLoopVertices.Count != 0)
        {
            _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
            _statusLoop |= _status;
            return true;
        }

        return false;
    }

    private static double Project(Curve3D curve, double firstParameter, double lastParameter,
                                  in Point3D point, double precision, out double parameter, out Point3D projection)
    {
        double dist = new CurveAnalysis().Project(curve, point, precision, out projection,
                                                         out parameter, firstParameter, lastParameter);
        if (parameter >= firstParameter && parameter <= lastParameter)
        {
            return dist;
        }

        double p0 = firstParameter, p1 = lastParameter;
        Point3D prj0 = curve.Value(p0), prj1 = curve.Value(p1);
        double dist0 = point.DistanceTo(prj0), dist1 = point.DistanceTo(prj1);
        int    pi    = (dist0 <= dist1) ? 0 : 1;
        parameter  = pi == 0 ? p0 : p1;
        projection = pi == 0 ? prj0 : prj1;
        return pi == 0 ? dist0 : dist1;
    }

    /// <summary>
    /// Checks if two adjacent edges form a tail (a thin spike) narrower than <paramref name="maxWidth"/>, at an angle
    /// whose sine is under <paramref name="maxSine"/> (no angle check when negative). When true, gives the parts of
    /// the edges to remove (<paramref name="edge11"/>, <paramref name="edge21"/>) and to keep
    /// (<paramref name="edge12"/>, <paramref name="edge22"/>): either the whole edges, or new edges cut from them.
    /// </summary>
    /// <remarks>
    /// The edge parts it cuts are new edges, published frozen (design 4.2); the outputs it doesn't set are left
    /// unchanged, as in OCCT.
    /// </remarks>
    public bool CheckTail(in Edge edge1, in Edge edge2, double maxSine, double maxWidth,
                          double maxTolerance, ref Edge edge11, ref Edge edge12,
                          ref Edge edge21, ref Edge edge22) =>
        CheckTailCore(edge1, edge2, maxSine, maxWidth, maxTolerance, ref edge11, ref edge12, ref edge21,
                  ref edge22, true);

    /// <summary>
    /// <see cref="CheckTail"/> without freezing the edge parts it cuts, for ShapeFix_Wire.FixTails, which puts them
    /// into the wire it goes on fixing (the parts share the vertices of its working edges).
    /// </summary>
    internal bool InternalCheckTail(in Edge edge1, in Edge edge2, double maxSine, double maxWidth,
                                    double maxTolerance, ref Edge edge11, ref Edge edge12,
                                    ref Edge edge21, ref Edge edge22) =>
        CheckTailCore(edge1, edge2, maxSine, maxWidth, maxTolerance, ref edge11, ref edge12, ref edge21,
                  ref edge22, false);

    private bool CheckTailCore(in Edge edge1, in Edge edge2, double maxSine, double maxWidth,
                           double maxTolerance, ref Edge edge11, ref Edge edge12,
                           ref Edge edge21, ref Edge edge22, bool toFreeze)
    {
        Edge[] es = [edge1, edge2];
        if (!IsReady || Tool.Degenerated(es[0]) || Tool.Degenerated(es[1]))
        {
            return false;
        }

        // Check the distance between the edge common ends.
        double       tol2   = maxWidth + 0.5 * Kerf.Engine.Foundation.Precision.Confusion();
        double       tol3   = maxWidth + Kerf.Engine.Foundation.Precision.Confusion();
        double       tol4   = maxWidth + 1.5 * Kerf.Engine.Foundation.Precision.Confusion();
        double       sqTol2 = tol2 * tol2;
        double       sqTol3 = tol3 * tol3;
        Curve3D?[] cs    = new Curve3D?[2];
        double[,]    ls     = new double[2, 2];
        int[]        vIs    = new int[2];
        Point3D[]     vPs    = [new Point3D(), new Point3D()];
        {
            for (int ei = 0; ei < 2; ++ei)
            {
                if (!new EdgeAnalysis().Curve3d(es[ei], out cs[ei], out ls[ei, 0], out ls[ei, 1], false))
                {
                    return false;
                }

                vIs[ei] = (es[ei].Orientation == Reversed) ? ei : 1 - ei;
                vPs[ei] = cs[ei]!.Value(ls[ei, vIs[ei]]);
            }

            if (vPs[0].SquareDistanceTo(vPs[1]) > sqTol2)
            {
                return false;
            }
        }

        // Check the angle between the edges.
        if (maxSine >= 0)
        {
            double   sqMaxSine = maxSine * maxSine;
            Xyz[] ds        = [new Xyz(), new Xyz()];
            int      reverse   = 0;
            for (int ei = 0; ei < 2; ++ei)
            {
                CurveAdaptor ca = new CurveAdaptor(cs[ei]!);
                if (AbscissaPoint.Length(ca, ls[ei, 0], ls[ei, 1], 0.25 * Kerf.Engine.Foundation.Precision.Confusion())
                    < 0.5 * Kerf.Engine.Foundation.Precision.Confusion())
                {
                    return false;
                }

                AbscissaPoint ap = new AbscissaPoint(0.25 * Kerf.Engine.Foundation.Precision.Confusion(),
                                                                    ca,
                                                                    0.5 * Kerf.Engine.Foundation.Precision.Confusion() * (1 - 2 * vIs[ei]),
                                                                    ls[ei, vIs[ei]]);
                if (!ap.IsDone)
                {
                    return false;
                }

                Xyz[] ps = [new Xyz(), new Xyz()];
                ps[vIs[ei]]     = vPs[ei].XYZ();
                ps[1 - vIs[ei]] = cs[ei]!.Value(ap.Parameter()).XYZ();
                ds[ei]           = ps[1] - ps[0];
                double dn         = ds[ei].Modulus();
                if (dn < 0.1 * Kerf.Engine.Foundation.Precision.Confusion())
                {
                    return false;
                }

                ds[ei] *= 1 / dn;
                reverse ^= vIs[ei];
            }

            if (reverse != 0)
            {
                ds[0].Reverse();
            }

            if (ds[0] * ds[1] < 0 || ds[0].CrossSquareMagnitude(ds[1]) > sqMaxSine)
            {
                return false;
            }
        }

        // Calculate the tail bounds.
        Point3D[] ps2     = [new Point3D(), new Point3D()];
        Point3D[] prjs    = [new Point3D(), new Point3D()];
        double[] params1 = new double[2];
        double[] params2 = new double[2];
        double[] dists   = new double[2];
        bool[]   isWholes = [true, true];
        for (int ei = 0; ei < 2; ++ei)
        {
            double param1 = ls[ei, vIs[ei]];
            params1[ei]  = ls[ei, 1 - vIs[ei]];
            cs[ei]!.D0(params1[ei], out ps2[ei]);
            dists[ei] = Project(cs[1 - ei]!,
                                  ls[1 - ei, 0],
                                  ls[1 - ei, 1],
                                  ps2[ei],
                                  0.25 * Kerf.Engine.Foundation.Precision.Confusion(),
                                  out params2[ei],
                                  out prjs[ei]);
            if (dists[ei] <= tol2)
            {
                continue;
            }

            isWholes[ei] = false;
            // Halving: past this count the interval has closed without the distance landing between the tolerances (decision 1883)
            for (int halved = 0; ; halved++)
            {
                if (halved > 128)
                {
                    return false;
                }
                double param = (param1 + params1[ei]) * 0.5;
                cs[ei]!.D0(param, out ps2[ei]);
                double dist = Project(cs[1 - ei]!,
                                       ls[1 - ei, 0],
                                       ls[1 - ei, 1],
                                       ps2[ei],
                                       0.25 * Kerf.Engine.Foundation.Precision.Confusion(),
                                       out params2[ei],
                                       out prjs[ei]);
                if (dist <= tol2)
                {
                    param1 = param;
                }
                else
                {
                    params1[ei] = param;
                    if (dist <= tol3)
                    {
                        break;
                    }
                }
            }
        }

        // Check the tail bounds.
        for (int ei = 0; ei < 2; ++ei)
        {
            double param1 = ls[ei, vIs[ei]];
            double param2 = params1[ei];
            double stepL  = (param2 - param1) / 23;
            for (int stepN = 1; stepN < 23; ++stepN)
            {
                double param = param1 + stepN * stepL;
                Point3D p     = cs[ei]!.Value(param);
                if (Project(cs[1 - ei]!,
                            ls[1 - ei, 0],
                            ls[1 - ei, 1],
                            p,
                            0.25 * Kerf.Engine.Foundation.Precision.Confusion(),
                            out param,
                            out _)
                    > tol4)
                {
                    return false;
                }
            }
        }

        // Check whether both edges must be removed.
        if (isWholes[0] && isWholes[1] && ps2[0].SquareDistanceTo(ps2[1]) <= sqTol3)
        {
            edge11 = edge1;
            edge21 = edge2;
            return true;
        }

        // Cut and remove the edges.
        int fi = 0;
        if (isWholes[0] || isWholes[1])
        {
            // Determine an edge to remove and the other one to cut.
            fi = isWholes[0] ? 0 : 1;
            if (dists[1 - fi] < dists[fi] && isWholes[1 - fi])
            {
                fi = 1 - fi;
            }
        }

        double[] aParams = new double[2];
        aParams[fi]     = params1[fi];
        aParams[1 - fi] = params2[fi];
        // Correct the cut for the parametrization tolerance.
        // OCCT's pointers to the four outputs: a local table copied back at the end
        Edge[,] eParts  = { { edge11, edge12 }, { edge21, edge22 } };
        int[]          results = [1, 1];
        for (int ei = 0; ei < 2; ++ei)
        {
            if (Math.Abs(aParams[ei] - ls[ei, 1 - vIs[ei]]) <= Kerf.Engine.Foundation.Precision.PConfusion())
            {
                results[ei]     = 2;
                eParts[ei, 0] = es[ei];
            }
            else if (Math.Abs(aParams[ei] - ls[ei, vIs[ei]]) <= Kerf.Engine.Foundation.Precision.PConfusion())
            {
                results[ei] = 0;
            }
        }

        // Correct the cut for the distance tolerance.
        for (int ei = 0; ei < 2; ++ei)
        {
            if (results[ei] != 1)
            {
                continue;
            }

            // Create the parts of the edge.
            Edge                          fe   = ShapeCasts.Edge(es[ei].Oriented(Forward));
            TransferParametersProjAnalysis satpp = new TransferParametersProjAnalysis(fe, new Face());
            satpp.SetMaxTolerance(maxTolerance);
            Vertex splitV = new Vertex();
            new BrepBuilder().MakeVertex(ref splitV, cs[ei]!.Value(aParams[ei]), Kerf.Engine.Foundation.Precision.Confusion());
            Edge[] eParts2 =
            [
                new BuildEdgeBuilder().InternalCopyReplaceVertices(fe, new Vertex(), ShapeCasts.Vertex(splitV.Oriented(Reversed))),
                new BuildEdgeBuilder().InternalCopyReplaceVertices(fe, splitV, new Vertex()),
            ];
            new BuildEdgeBuilder().CopyPCurves(eParts2[0], fe);
            new BuildEdgeBuilder().CopyPCurves(eParts2[1], fe);
            new BrepBuilder().SameRange(eParts2[0], false);
            new BrepBuilder().SameRange(eParts2[1], false);
            new BrepBuilder().SameParameter(eParts2[0], false);
            new BrepBuilder().SameParameter(eParts2[1], false);
            satpp.TransferRange(eParts2[0], ls[ei, 0], aParams[ei], false);
            satpp.TransferRange(eParts2[1], aParams[ei], ls[ei, 1], false);
            // Kerf: the new parts are published frozen (design 4.2)
            if (toFreeze)
            {
                eParts2[0].TShape()!.FreezeGraph();
                eParts2[1].TShape()!.FreezeGraph();
            }
            Properties linProps = new Properties();
            MassPropertiesBuilder.LinearProperties(eParts2[1 - vIs[ei]], linProps);
            if (linProps.Mass() <= Kerf.Engine.Foundation.Precision.Confusion())
            {
                results[ei]     = 2;
                eParts[ei, 0] = es[ei];
            }
            else
            {
                MassPropertiesBuilder.LinearProperties(eParts2[vIs[ei]], linProps);
                if (linProps.Mass() <= Kerf.Engine.Foundation.Precision.Confusion())
                {
                    results[ei] = 0;
                }
                else
                {
                    eParts[ei, 0] = eParts2[0];
                    eParts[ei, 1] = eParts2[1];
                }
            }
        }

        edge11 = eParts[0, 0];
        edge12 = eParts[0, 1];
        edge21 = eParts[1, 0];
        edge22 = eParts[1, 1];
        return results[0] + results[1] != 0;
    }
}
