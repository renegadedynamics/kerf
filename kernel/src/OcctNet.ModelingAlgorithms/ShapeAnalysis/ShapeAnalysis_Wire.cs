// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_Wire.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_Wire.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_Wire.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.IntRes2d_Position;
using static Occt.ShapeExtend_Status;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

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
public class ShapeAnalysis_Wire : Standard_Transient
{
    protected ShapeExtend_WireData?  myWire;
    protected TopoDS_Face            myFace;
    protected ShapeAnalysis_Surface? mySurf;
    protected double                 myPrecision;
    protected double                 myMin3d;
    protected double                 myMin2d;
    protected double                 myMax3d;
    protected double                 myMax2d;
    protected int                    myStatusOrder;
    protected int                    myStatusConnected;
    protected int                    myStatusEdgeCurves;
    protected int                    myStatusDegenerated;
    protected int                    myStatusClosed;
    protected int                    myStatusSmall;
    protected int                    myStatusSelfIntersection;
    protected int                    myStatusLacking;
    protected int                    myStatusGaps3d;
    protected int                    myStatusGaps2d;
    protected int                    myStatusCurveGaps;
    protected int                    myStatusLoop;
    protected int                    myStatus;

    /// <summary>Empty constructor.</summary>
    public ShapeAnalysis_Wire()
    {
        myFace = new TopoDS_Face();
        ClearStatuses();
        myPrecision = Occt.Precision.Confusion();
    }

    /// <summary>Creates the object with a wire, a face and a precision.</summary>
    public ShapeAnalysis_Wire(in TopoDS_Wire wire, in TopoDS_Face face, double precision)
    {
        myFace = new TopoDS_Face();
        Init(wire, face, precision);
    }

    /// <summary>Creates the object with a WireData object, a face and a precision.</summary>
    public ShapeAnalysis_Wire(ShapeExtend_WireData? sbwd, in TopoDS_Face face, double precision)
    {
        myFace = new TopoDS_Face();
        Init(sbwd, face, precision);
    }

    /// <summary>Initializes the object with a wire, a face and a precision.</summary>
    public void Init(in TopoDS_Wire wire, in TopoDS_Face face, double precision)
    {
        Init(new ShapeExtend_WireData(wire), face, precision);
    }

    /// <summary>Initializes the object with a WireData object, a face and a precision.</summary>
    public void Init(ShapeExtend_WireData? sbwd, in TopoDS_Face face, double precision)
    {
        Load(sbwd);
        SetFace(face);
        SetPrecision(precision);
    }

    /// <summary>Loads the object with a wire.</summary>
    public void Load(in TopoDS_Wire wire)
    {
        ClearStatuses();
        myWire = new ShapeExtend_WireData(wire);
    }

    /// <summary>Loads the object with a WireData object (shared, not copied).</summary>
    public void Load(ShapeExtend_WireData? sbwd)
    {
        ClearStatuses();
        myWire = sbwd;
    }

    /// <summary>Loads the face the wire lies on.</summary>
    public void SetFace(in TopoDS_Face face)
    {
        myFace = face;
        if (myFace.IsNull())
        {
            return;
        }

        Geom_Surface? aSurface = BRep_Tool.Surface(myFace);
        if (!face.IsNull() && aSurface is not null)
        {
            mySurf = new ShapeAnalysis_Surface(aSurface);
        }
    }

    /// <summary>Loads the face the wire lies on and its surface analysis object.</summary>
    public void SetFace(in TopoDS_Face theFace, ShapeAnalysis_Surface? theSurfaceAnalysis)
    {
        myFace = theFace;
        mySurf = theSurfaceAnalysis;
    }

    /// <summary>Loads the surface analysis object.</summary>
    public void SetSurface(ShapeAnalysis_Surface? theSurfaceAnalysis)
    {
        mySurf = theSurfaceAnalysis;
    }

    /// <summary>Loads the surface the wire lies on.</summary>
    public void SetSurface(Geom_Surface? surface)
    {
        SetSurface(surface, new TopLoc_Location());
    }

    /// <summary>Loads the surface the wire lies on, with a location; a new face on it becomes the working face.</summary>
    public void SetSurface(Geom_Surface? surface, in TopLoc_Location location)
    {
        BRep_Builder B    = new BRep_Builder();
        TopoDS_Face  face = new TopoDS_Face();
        B.MakeFace(ref face, surface, location, Occt.Precision.Confusion());
        // OCCT.NET: the face is reachable through Face(), so it is published frozen (design 4.2)
        face.TShape()!.FreezeGraph();
        SetFace(face);
    }

    /// <summary>Sets the precision.</summary>
    public void SetPrecision(double precision)
    {
        myPrecision = precision;
    }

    /// <summary>Unsets all the status and distance fields; the wire, face and precision are kept.</summary>
    public void ClearStatuses()
    {
        myStatusOrder = myStatusConnected = myStatusEdgeCurves = myStatusDegenerated = myStatusClosed =
            myStatusLacking = myStatusSelfIntersection = myStatusSmall = myStatusGaps3d = myStatusGaps2d =
                myStatusCurveGaps = myStatusLoop = myStatus = 0;

        myMin3d = myMin2d = myMax3d = myMax2d = 0.0;
    }

    /// <summary>Returns true if a wire is loaded and has at least one edge.</summary>
    public bool IsLoaded()
    {
        return myWire is not null && myWire.NbEdges() > 0;
    }

    /// <summary>Returns true if <see cref="IsLoaded"/> and the face isn't null.</summary>
    public bool IsReady()
    {
        return IsLoaded() && !myFace.IsNull();
    }

    /// <summary>The precision.</summary>
    public double Precision()
    {
        return myPrecision;
    }

    /// <summary>The wire being analysed.</summary>
    public ShapeExtend_WireData? WireData()
    {
        return myWire;
    }

    /// <summary>The number of edges in the wire, 0 if no wire is loaded.</summary>
    public int NbEdges()
    {
        return myWire is null ? 0 : myWire.NbEdges();
    }

    /// <summary>The working face.</summary>
    public TopoDS_Face Face()
    {
        return myFace;
    }

    /// <summary>The working surface.</summary>
    public ShapeAnalysis_Surface? Surface()
    {
        return mySurf;
    }

    public bool StatusOrder(ShapeExtend_Status Status) => ShapeExtend.DecodeStatus(myStatusOrder, Status);

    public bool StatusConnected(ShapeExtend_Status Status) => ShapeExtend.DecodeStatus(myStatusConnected, Status);

    public bool StatusEdgeCurves(ShapeExtend_Status Status) => ShapeExtend.DecodeStatus(myStatusEdgeCurves, Status);

    public bool StatusDegenerated(ShapeExtend_Status Status) => ShapeExtend.DecodeStatus(myStatusDegenerated, Status);

    public bool StatusClosed(ShapeExtend_Status Status) => ShapeExtend.DecodeStatus(myStatusClosed, Status);

    public bool StatusSmall(ShapeExtend_Status Status) => ShapeExtend.DecodeStatus(myStatusSmall, Status);

    public bool StatusSelfIntersection(ShapeExtend_Status Status) =>
        ShapeExtend.DecodeStatus(myStatusSelfIntersection, Status);

    public bool StatusLacking(ShapeExtend_Status Status) => ShapeExtend.DecodeStatus(myStatusLacking, Status);

    public bool StatusGaps3d(ShapeExtend_Status Status) => ShapeExtend.DecodeStatus(myStatusGaps3d, Status);

    public bool StatusGaps2d(ShapeExtend_Status Status) => ShapeExtend.DecodeStatus(myStatusGaps2d, Status);

    public bool StatusCurveGaps(ShapeExtend_Status Status) => ShapeExtend.DecodeStatus(myStatusCurveGaps, Status);

    public bool StatusLoop(ShapeExtend_Status Status) => ShapeExtend.DecodeStatus(myStatusLoop, Status);

    /// <summary>Queries the status of the last performed advanced check.</summary>
    public bool LastCheckStatus(ShapeExtend_Status Status) => ShapeExtend.DecodeStatus(myStatus, Status);

    /// <summary>The last lowest distance in 3D computed by the checks.</summary>
    public double MinDistance3d() => myMin3d;

    /// <summary>The last lowest distance in 2D (UV) computed by the checks.</summary>
    public double MinDistance2d() => myMin2d;

    /// <summary>The last maximal distance in 3D computed by the checks.</summary>
    public double MaxDistance3d() => myMax3d;

    /// <summary>The last maximal distance in 2D (UV) computed by the checks.</summary>
    public double MaxDistance2d() => myMax2d;

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
        ShapeAnalysis_WireOrder sawo = new ShapeAnalysis_WireOrder();
        CheckOrder(sawo, isClosed, mode3d, false);
        myStatusOrder = myStatus;
        return StatusOrder(ShapeExtend_DONE);
    }

    /// <summary>Calls CheckSmall for each edge; true if at least one small edge was detected.</summary>
    public bool CheckSmall(double precsmall = 0.0)
    {
        for (int i = 1; i <= myWire!.NbEdges(); i++)
        {
            CheckSmall(i, precsmall);
            myStatusSmall |= myStatus;
        }

        return StatusSmall(ShapeExtend_DONE);
    }

    /// <summary>Calls CheckConnected for each edge; true if at least one pair of disconnected edges was detected.</summary>
    public bool CheckConnected(double prec = 0.0)
    {
        for (int i = 1; i <= myWire!.NbEdges(); i++)
        {
            CheckConnected(i, prec);
            myStatusConnected |= myStatus;
        }

        return StatusConnected(ShapeExtend_DONE);
    }

    /// <summary>
    /// Checks the edges' geometry: ShapeAnalysis_Edge's CheckCurve3dWithPCurve (1), CheckVerticesWithPCurve (2),
    /// CheckVerticesWithCurve3d (3), CheckSeam (4), CheckGap3d (5), CheckGap2d (6) and ShapeAnalysis_Edge's
    /// CheckSameParameter (7); the numbers are the DONEi and FAILi flags of the status.
    /// </summary>
    public bool CheckEdgeCurves()
    {
        myStatusEdgeCurves = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsReady())
        {
            return false;
        }

        int                i, nb = myWire!.NbEdges();
        ShapeAnalysis_Edge SAE = new ShapeAnalysis_Edge();

        for (i = 1; i <= nb; i++)
        {
            TopoDS_Edge E = myWire.Edge(i);

            SAE.CheckCurve3dWithPCurve(E, myFace);
            if (SAE.Status(ShapeExtend_DONE))
            {
                myStatusEdgeCurves |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
            }

            if (SAE.Status(ShapeExtend_FAIL))
            {
                myStatusEdgeCurves |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
            }

            SAE.CheckVerticesWithPCurve(E, myFace);
            if (SAE.Status(ShapeExtend_DONE))
            {
                myStatusEdgeCurves |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
            }

            if (SAE.Status(ShapeExtend_FAIL))
            {
                myStatusEdgeCurves |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
            }

            SAE.CheckVerticesWithCurve3d(E);
            if (SAE.Status(ShapeExtend_DONE))
            {
                myStatusEdgeCurves |= ShapeExtend.EncodeStatus(ShapeExtend_DONE3);
            }

            if (SAE.Status(ShapeExtend_FAIL))
            {
                myStatusEdgeCurves |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL3);
            }

            CheckSeam(i);
            if (LastCheckStatus(ShapeExtend_DONE))
            {
                myStatusEdgeCurves |= ShapeExtend.EncodeStatus(ShapeExtend_DONE4);
            }

            if (LastCheckStatus(ShapeExtend_FAIL))
            {
                myStatusEdgeCurves |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL4);
            }

            CheckGap3d(i);
            if (LastCheckStatus(ShapeExtend_DONE))
            {
                myStatusEdgeCurves |= ShapeExtend.EncodeStatus(ShapeExtend_DONE5);
            }

            if (LastCheckStatus(ShapeExtend_FAIL))
            {
                myStatusEdgeCurves |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL5);
            }

            CheckGap2d(i);
            if (LastCheckStatus(ShapeExtend_DONE))
            {
                myStatusEdgeCurves |= ShapeExtend.EncodeStatus(ShapeExtend_DONE6);
            }

            if (LastCheckStatus(ShapeExtend_FAIL))
            {
                myStatusEdgeCurves |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL6);
            }

            double maxdev = 0.0;
            SAE.CheckSameParameter(myWire.Edge(i), ref maxdev);
            if (SAE.Status(ShapeExtend_DONE))
            {
                myStatusEdgeCurves |= ShapeExtend.EncodeStatus(ShapeExtend_DONE7);
            }

            if (SAE.Status(ShapeExtend_FAIL))
            {
                myStatusEdgeCurves |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL7);
            }
        }

        return StatusEdgeCurves(ShapeExtend_DONE);
    }

    /// <summary>Calls CheckDegenerated for each edge; true if at least one incorrect degenerated edge was detected.</summary>
    public bool CheckDegenerated()
    {
        for (int i = 1; i <= myWire!.NbEdges(); i++)
        {
            CheckDegenerated(i);
            myStatusDegenerated |= myStatus;
        }

        return StatusDegenerated(ShapeExtend_DONE);
    }

    /// <summary>
    /// Checks the self-intersection of the wire (on pcurves): each edge with itself (DONE1/FAIL1), each two adjacent
    /// edges (DONE2/FAIL2), and each two non-adjacent edges whose boxes overlap (DONE3/FAIL3).
    /// </summary>
    public bool CheckSelfIntersection()
    {
        myStatusSelfIntersection = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsReady())
        {
            return false;
        }

        int i, nb = myWire!.NbEdges();
        for (i = 1; i <= nb; i++)
        {
            CheckSelfIntersectingEdge(i);
            if (LastCheckStatus(ShapeExtend_DONE))
            {
                myStatusSelfIntersection |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
            }

            if (LastCheckStatus(ShapeExtend_FAIL))
            {
                myStatusSelfIntersection |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
            }

            CheckIntersectingEdges(i);
            if (LastCheckStatus(ShapeExtend_DONE))
            {
                myStatusSelfIntersection |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
            }

            if (LastCheckStatus(ShapeExtend_FAIL))
            {
                myStatusSelfIntersection |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
            }
        }

        NCollection_Array1<Bnd_Box2d> boxes = new NCollection_Array1<Bnd_Box2d>(1, nb);
        Geom_Surface?        S    = BRep_Tool.Surface(Face(), out TopLoc_Location L);
        ShapeAnalysis_Edge   sae  = new ShapeAnalysis_Edge();
        ShapeExtend_WireData sbwd = WireData()!;
        for (i = 1; i <= nb; i++)
        {
            TopoDS_Edge E = sbwd.Edge(i);
            if (sae.PCurve(E, S, L, out Geom2d_Curve? c2d, out double cf, out double cl, false))
            {
                Bnd_Box2d           box = new Bnd_Box2d();
                Geom2dAdaptor_Curve gac = new Geom2dAdaptor_Curve(c2d!, cf, cl);
                BndLib_Add2dCurve.Add(gac, Occt.Precision.Confusion(), ref box);
                boxes[i] = box;
            }
        }

        bool isFail = false, isDone = false;
        for (int num1 = 1; num1 < nb - 1; num1++)
        {
            int fin = nb;
            if (CheckClosed(Occt.Precision.Confusion()) && 1 == num1)
            {
                fin = nb - 1;
            }

            for (int num2 = num1 + 2; num2 <= fin; num2++)
            {
                if (!boxes[num1].IsOut(boxes[num2]))
                {
                    CheckIntersectingEdges(num1, num2);
                    isFail |= LastCheckStatus(ShapeExtend_FAIL1);
                    isDone |= LastCheckStatus(ShapeExtend_DONE1);
                }
            }
        }

        if (isFail)
        {
            myStatusSelfIntersection |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL3);
        }

        if (isDone)
        {
            myStatusSelfIntersection |= ShapeExtend.EncodeStatus(ShapeExtend_DONE3);
        }

        return StatusSelfIntersection(ShapeExtend_DONE);
    }

    /// <summary>Calls CheckLacking for each edge; true if at least one lacking edge was detected.</summary>
    public bool CheckLacking()
    {
        if (!IsReady() || NbEdges() < 2)
        {
            return false;
        }

        for (int i = 1; i <= myWire!.NbEdges(); i++)
        {
            CheckLacking(i);
            myStatusLacking |= myStatus;
        }

        return StatusLacking(ShapeExtend_DONE);
    }

    /// <summary>
    /// Checks if the wire is closed: CheckConnected (DONE1/FAIL1) and CheckDegenerated (DONE2/FAIL2) of the first edge
    /// after the last.
    /// </summary>
    public bool CheckClosed(double prec = 0.0)
    {
        myStatusClosed = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsReady() || NbEdges() < 1)
        {
            return false;
        }

        CheckConnected(1, prec);
        if (LastCheckStatus(ShapeExtend_DONE))
        {
            myStatusClosed |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
        }

        if (LastCheckStatus(ShapeExtend_FAIL))
        {
            myStatusClosed |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
        }

        CheckDegenerated(1);
        if (LastCheckStatus(ShapeExtend_DONE))
        {
            myStatusClosed |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
        }

        if (LastCheckStatus(ShapeExtend_FAIL))
        {
            myStatusClosed |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
        }

        return StatusClosed(ShapeExtend_DONE);
    }

    /// <summary>Calls CheckGap3d for each edge; the largest gap is in <see cref="MinDistance3d"/> and <see cref="MaxDistance3d"/>.</summary>
    public bool CheckGaps3d()
    {
        myStatusGaps3d = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsLoaded() || NbEdges() < 1)
        {
            return false; // gka IsLoaded
        }

        double dist, maxdist = 0.0;

        for (int i = 1; i <= NbEdges(); i++)
        {
            CheckGap3d(i);
            myStatusGaps3d |= myStatus;
            if (!LastCheckStatus(ShapeExtend_FAIL1))
            {
                dist = MinDistance3d();
                if (maxdist < dist)
                {
                    maxdist = dist;
                }
            }
        }

        myMin3d = myMax3d = maxdist;

        return StatusGaps3d(ShapeExtend_DONE);
    }

    /// <summary>Calls CheckGap2d for each edge; the largest gap is in <see cref="MinDistance2d"/> and <see cref="MaxDistance2d"/>.</summary>
    public bool CheckGaps2d()
    {
        myStatusGaps2d = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsReady() || NbEdges() < 1)
        {
            return false;
        }

        double dist, maxdist = 0.0;

        for (int i = 1; i <= NbEdges(); i++)
        {
            CheckGap2d(i);
            myStatusGaps2d |= myStatus;
            if (!LastCheckStatus(ShapeExtend_FAIL1))
            {
                dist = MinDistance2d();
                if (maxdist < dist)
                {
                    maxdist = dist;
                }
            }
        }

        myMin2d = myMax2d = maxdist;

        return StatusGaps2d(ShapeExtend_DONE);
    }

    /// <summary>Calls CheckCurveGap for each edge; the largest gap is in <see cref="MinDistance3d"/> and <see cref="MaxDistance3d"/>.</summary>
    public bool CheckCurveGaps()
    {
        myStatusCurveGaps = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsReady() || NbEdges() < 1)
        {
            return false;
        }

        double dist, maxdist = 0.0;

        for (int i = 1; i <= NbEdges(); i++)
        {
            CheckCurveGap(i);
            myStatusCurveGaps |= myStatus;
            if (!LastCheckStatus(ShapeExtend_FAIL1))
            {
                dist = MinDistance3d();
                if (maxdist < dist)
                {
                    maxdist = dist;
                }
            }
        }

        myMin3d = myMax3d = maxdist;

        return StatusCurveGaps(ShapeExtend_DONE);
    }

    /// <summary>
    /// Analyzes the order of the edges in the wire with <paramref name="sawo"/>, in 3d (vertices) or 2d (pcurve ends), or
    /// both when <paramref name="theModeBoth"/> is set. Returns false if the wire is already ordered (tail to head).
    /// </summary>
    /// <remarks>
    /// Status: OK the same orientations and sequence; DONE1 the same orientations, another sequence; DONE2 as DONE1 with
    /// gaps over the precision; DONE3 some edges must be reversed; DONE4 as DONE3 with gaps; DONE5 only shifted; FAIL the
    /// order couldn't be found (FAIL2: no face in 2d mode, no vertices in 3d mode, or no pcurve in 2d mode).
    /// </remarks>
    public bool CheckOrder(ShapeAnalysis_WireOrder sawo, bool isClosed = true, bool theMode3D = true, bool theModeBoth = false)
    {
        if ((!theMode3D || theModeBoth) && myFace.IsNull())
        {
            myStatus = ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
            return false;
        }

        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        sawo.SetMode(theMode3D, 0.0, theModeBoth);
        int                nb = myWire!.NbEdges();
        ShapeAnalysis_Edge EA = new ShapeAnalysis_Edge();
        bool               isAll2dEdgesOk = true;
        for (int i = 1; i <= nb; i++)
        {
            TopoDS_Edge E = myWire.Edge(i);
            gp_XYZ      aP1XYZ = new gp_XYZ(), aP2XYZ = new gp_XYZ();
            gp_XY       aP1XY = new gp_XY(), aP2XY = new gp_XY();
            if (theMode3D || theModeBoth)
            {
                TopoDS_Vertex V1 = EA.FirstVertex(E);
                TopoDS_Vertex V2 = EA.LastVertex(E);
                if (V1.IsNull() || V2.IsNull())
                {
                    myStatus = ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
                    return false;
                }
                else
                {
                    aP1XYZ = BRep_Tool.Pnt(V1).XYZ();
                    aP2XYZ = BRep_Tool.Pnt(V2).XYZ();
                }
            }

            if (!theMode3D || theModeBoth)
            {
                TopoDS_Shape tmpF = myFace.Oriented(TopAbs_FORWARD);
                if (!EA.PCurve(E, TopoDS.Face(tmpF), out Geom2d_Curve? c2d, out double f, out double l))
                {
                    // if mode is 2d, then we can nothing to do, else we can switch to 3d mode
                    if (!theMode3D && !theModeBoth)
                    {
                        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
                        return false;
                    }
                    else
                    {
                        isAll2dEdgesOk = false;
                    }
                }
                else
                {
                    aP1XY = c2d!.Value(f).XY();
                    aP2XY = c2d.Value(l).XY();
                }
            }

            if (theMode3D && !theModeBoth)
            {
                sawo.Add(aP1XYZ, aP2XYZ);
            }
            else if (!theMode3D && !theModeBoth)
            {
                sawo.Add(aP1XY, aP2XY);
            }
            else
            {
                sawo.Add(aP1XYZ, aP2XYZ, aP1XY, aP2XY);
            }
        }

        // need to switch to 3d mode
        if (theModeBoth && !isAll2dEdgesOk)
        {
            sawo.SetMode(true, 0.0, false);
        }

        sawo.Perform(isClosed);
        int stat = sawo.Status();
        switch (stat)
        {
            case 0:
                myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
                break;
            case 1:
                myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
                break;
            case 2:
                myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
                break; // this value is not returned
            case -1:
                myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE3);
                break;
            case -2:
                myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE4);
                break; // this value is not returned
            case 3:
                myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE5);
                break; // only shifted
            case -10:
                myStatus = ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
                break; // this value is not returned
        }

        return LastCheckStatus(ShapeExtend_DONE);
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
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsLoaded() || NbEdges() < 1)
        {
            return false;
        }

        int n2 = (num > 0 ? num : NbEdges());
        int n1 = (n2 > 1 ? n2 - 1 : NbEdges());
        //  if (n1 == n2) return 0;

        TopoDS_Edge        E1  = WireData()!.Edge(n1);
        TopoDS_Edge        E2  = WireData()!.Edge(n2);
        ShapeAnalysis_Edge sae = new ShapeAnalysis_Edge();
        TopoDS_Vertex      V1  = sae.LastVertex(E1);
        TopoDS_Vertex      V2  = sae.FirstVertex(E2);
        if (V1.IsNull() || V2.IsNull())
        {
            myStatus = ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
            return false;
        }

        if (V1.IsSame(V2))
        {
            return false;
        }

        gp_Pnt p1 = BRep_Tool.Pnt(V1);
        gp_Pnt p2 = BRep_Tool.Pnt(V2);
        myMin3d   = p1.Distance(p2);
        if (myMin3d <= gp.Resolution())
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
        }
        else if (myMin3d <= myPrecision)
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
        }
        else if (myMin3d <= prec)
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE3);
        }
        else
        {
            // et en inversant la derniere edge ?
            if (n1 == n2)
            {
                myStatus = ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
            }
            else
            {
                V2          = sae.LastVertex(E2);
                p2          = BRep_Tool.Pnt(V2);
                double dist = p1.Distance(p2);
                if (dist > myPrecision)
                {
                    myStatus = ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
                }
                else
                {
                    myMin3d  = dist;
                    myStatus = ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
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
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsLoaded() || NbEdges() <= 1)
        {
            return false;
        }

        TopoDS_Edge        E   = myWire!.Edge(num != 0 ? num : NbEdges());
        ShapeAnalysis_Edge sae = new ShapeAnalysis_Edge();

        if (BRep_Tool.Degenerated(E))
        {
            //: n2 abv 22 Jan 99: ma-test5.igs -> IGES (brep) -> read (pref3d):
            // degen edge with no pcurve should be removed
            if (!myFace.IsNull() && sae.HasPCurve(E, Face()))
            {
                return false;
            }

            myStatus = ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
        }

        TopoDS_Vertex V1 = sae.FirstVertex(E);
        TopoDS_Vertex V2 = sae.LastVertex(E);
        if (V1.IsNull() || V2.IsNull())
        {
            myStatus = ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
            return false;
        }

        gp_Pnt p1   = BRep_Tool.Pnt(V1);
        gp_Pnt p2   = BRep_Tool.Pnt(V2);
        double dist = p1.Distance(p2);
        double prec = precsmall; // Min ( myPrecision, precsmall );
        if (dist > prec)
        {
            return false; // not small enough
        }

        // The 3D curve now: is it CLOSED or ZERO LENGTH...???
        // To do this, we take the midpoint (is there anything better?)
        // If there's no 3D curve, we try 2D...
        gp_Pnt aMidpoint;
        if (sae.Curve3d(E, out Geom_Curve? c3d, out double cf, out double cl, false))
        {
            aMidpoint = c3d!.Value((cf + cl) / 2.0);
        }
        else
        {
            Geom2d_Curve? c2d = null;
            if (!myFace.IsNull() && mySurf is not null && sae.PCurve(E, myFace, out c2d, out cf, out cl, false))
            {
                gp_Pnt2d p2m = c2d!.Value((cf + cl) / 2.0);
                aMidpoint    = mySurf.Value(p2m);
            }
            else
            {
                myStatus  = ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
                aMidpoint = p1;
            }
        }

        if (aMidpoint.Distance(p1) > prec || aMidpoint.Distance(p2) > prec)
        {
            return false;
        }

        myStatus |= ShapeExtend.EncodeStatus(V1.IsSame(V2) ? ShapeExtend_DONE1 : ShapeExtend_DONE2);
        return true;
    }

    /// <summary>
    /// Checks if the pcurves of a seam edge are correctly oriented. Returns false (status OK) if the edge isn't a seam or
    /// if they are; true (DONE1) if they should be swapped. <paramref name="C1"/> and <paramref name="C2"/> get the
    /// pcurves of the forward and reversed edge, <paramref name="cf"/> and <paramref name="cl"/> their range (left
    /// unchanged when the edge isn't a seam).
    /// </summary>
    public bool CheckSeam(int num, ref Geom2d_Curve? C1, ref Geom2d_Curve? C2, ref double cf, ref double cl)
    {
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsReady())
        {
            return false;
        }

        int n = num;
        if (n == 0)
        {
            n = NbEdges();
        }

        TopoDS_Edge E = myWire!.Edge(n);
        if (!new ShapeAnalysis_Edge().IsSeam(E, myFace))
        {
            return false;
        }

        // Extract the Two PCurves of the Seam
        TopoDS_Face ForwardFace = myFace;
        ForwardFace.Orientation(TopAbs_FORWARD);
        // szv#4:S4163:12Mar99 SGI warns
        TopoDS_Shape EF = E.Oriented(TopAbs_FORWARD);
        TopoDS_Shape ER = E.Oriented(TopAbs_REVERSED);
        C1              = BRep_Tool.CurveOnSurface(TopoDS.Edge(EF), ForwardFace, out cf, out cl);
        C2              = BRep_Tool.CurveOnSurface(TopoDS.Edge(ER), ForwardFace, out cf, out cl);
        if (C1 is null || C2 is null)
        {
            return false;
        }

        //  SelectForward est destine a devenir un outil distinct
        int theCurveIndice = new ShapeAnalysis_Curve().SelectForwardSeam(C1, C2);
        if (theCurveIndice != 2)
        {
            return false;
        }

        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
        return true;
    }

    /// <summary>Checks if the pcurves of a seam edge are correctly oriented (see the other overload).</summary>
    public bool CheckSeam(int num)
    {
        Geom2d_Curve? C1 = null, C2 = null;
        double        cf = 0.0, cl = 0.0;
        return CheckSeam(num, ref C1, ref C2, ref cf, ref cl);
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
    public bool CheckDegenerated(int num, ref gp_Pnt2d p2d1, ref gp_Pnt2d p2d2)
    {
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsReady() || NbEdges() < 1)
        {
            return false;
        }

        int         n2 = (num > 0) ? num : NbEdges();
        int         n1 = (n2 > 1) ? n2 - 1 : NbEdges();
        int         n3 = (n2 < NbEdges()) ? n2 + 1 : 1;
        TopoDS_Edge E1 = myWire!.Edge(n1);
        TopoDS_Edge E2 = myWire.Edge(n2);
        TopoDS_Edge E3 = myWire.Edge(n3);

        ShapeAnalysis_Edge sae = new ShapeAnalysis_Edge();

        // skip if edge is already marked as degenerated and has pcurve
        if (BRep_Tool.Degenerated(E2) && sae.HasPCurve(E2, Face()))
        {
            // skl 30.12.2004 for OCC7630 - we have to check pcurve
            if (sae.HasPCurve(E1, Face()) && sae.HasPCurve(E3, Face()))
            {
                sae.PCurve(E2, myFace, out Geom2d_Curve? c2d, out double fp, out double lp, true);
                gp_Pnt2d p21 = c2d!.Value(fp);
                gp_Pnt2d p22 = c2d.Value(lp);
                sae.PCurve(E1, myFace, out c2d, out fp, out lp, true);
                gp_Pnt2d p12 = c2d!.Value(lp);
                sae.PCurve(E3, myFace, out c2d, out fp, out lp, true);
                gp_Pnt2d p31 = c2d!.Value(fp);
                if (Math.Abs(p12.Distance(p31) - p21.Distance(p22)) > 2 * Occt.Precision.PConfusion())
                {
                    // pcurve is bad => we can remove this edge in ShapeFix
                    // if set needed status
                    myStatus = ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
                }
            }

            return false;
        }

        // pdn allows to insert two sequences of degenerated edges (on separate bounds of surfaces)
        if (n1 != n2 && BRep_Tool.Degenerated(E1) && !sae.HasPCurve(E1, Face()))
        {
            //: abv 13.05.02: OCC320 - fail (to remove edge) if two consecutive degenerated edges w/o
            //: pcurves
            if (BRep_Tool.Degenerated(E2))
            {
                myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
            }

            return false;
        }

        TopoDS_Vertex Vp = sae.FirstVertex(E1); //: i9
        TopoDS_Vertex V0 = sae.LastVertex(E1);
        TopoDS_Vertex V1 = sae.FirstVertex(E2);
        TopoDS_Vertex V2 = sae.LastVertex(E2);

        if (Vp.IsNull() || V0.IsNull() || V1.IsNull() || V2.IsNull())
        {
            return false;
        }

        gp_Pnt pp = BRep_Tool.Pnt(Vp); //: i9
        gp_Pnt p0 = BRep_Tool.Pnt(V0);
        gp_Pnt p1 = BRep_Tool.Pnt(V1);
        gp_Pnt p2 = BRep_Tool.Pnt(V2);
        double par1 = 0.0, par2 = 0.0;
        bool   lack = false;
        bool   dgnr = false;
        // pdn 12.03.99 minimal value processing first
        double precFirst = Min(myPrecision, BRep_Tool.Tolerance(V1));
        double precFin   = Max(myPrecision, BRep_Tool.Tolerance(V1));
        double precVtx   = (myPrecision < BRep_Tool.Tolerance(V1) ? 2 * precFin : precFin);

        //  forward : si Edge <num> FWD/REV. Si LACK, toujours True
        bool forward = (E2.Orientation() == TopAbs_FORWARD);
        //  FIX FEV 1998 : recompute singularity according precision
        if (p1.Distance(p2) <= precFirst)
        { // edge DGNR
            dgnr = mySurf!.DegeneratedValues(p1, precVtx, ref p2d1, ref p2d2, ref par1, ref par2, forward); // smh#9
            if (dgnr)
            { // abv 24 Feb 00: trj3_as1-ac-214.stp #6065: avoid making closed edge degenerated
                Geom_Curve? C3d = BRep_Tool.Curve(E2, out double a, out double b);
                if (C3d is not null)
                {
                    gp_Pnt p = C3d.Value(0.5 * (a + b));
                    if (p.SquareDistance(p1) > precVtx * precVtx)
                    {
                        dgnr = false;
                    }
                }
            }
        }

        if (!dgnr)
        {
            //: i9 abv 23 Sep 98: CTS20315-2 #63231: check that previous edge is not degenerated
            if (n1 != n2 && p1.Distance(pp) <= precFirst && mySurf!.IsDegenerated(pp, precFirst)
                && !BRep_Tool.Degenerated(E1))
            {
                return false;
            }

            // rln S4135 ShapeAnalysis_Surface new algorithms for singularities
            //: 45 by abv 16 Dec 97: BUC60035 2659: precision increased to vertex tolerance
            //: 51 abv 22 Dec 97: recompute singularities if necessary
            if (p0.Distance(p1) <= precFin)
            { // ou DGNR manquante ?
                // rln S4135 singularity with precision = 2 * prec, but distance <= prec
                double tmpPreci = 0.0;
                gp_Pnt tmpP3d   = new gp_Pnt();
                bool   tmpUIsoDeg = false;
                // #77 rln S4135: using singularity which has minimum gap between singular point and input 3D
                // point
                int    indMin  = -1;
                double gapMin2 = RealLast();
                for (int i = 1; i <= mySurf!.NbSingularities(precVtx); i++)
                {
                    mySurf.Singularity(i, ref tmpPreci, ref tmpP3d, ref p2d1, ref p2d2, ref par1, ref par2, ref tmpUIsoDeg);
                    double gap2 = p1.SquareDistance(tmpP3d);
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
                    mySurf.Singularity(indMin, ref tmpPreci, ref tmpP3d, ref p2d1, ref p2d2, ref par1, ref par2, ref tmpUIsoDeg);
                    lack = true;
                }
            }
        }

        // there you go, we either have degenerate or lack
        if (!lack && !dgnr)
        {
            //: abv 29.08.01: if singularity not detected but edge is marked
            // as degenerated, report fail
            if (BRep_Tool.Degenerated(E2) && !sae.HasPCurve(E2, Face()))
            {
                myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
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
            if (sae.PCurve(E1, myFace, out Geom2d_Curve? c2d, out double a, out double b, true))
            {
                p2d1 = c2d!.Value(b);
            }
            else
            {
                myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
            }

            // pdn pcurves (fixing regression in f0 in degenerated case)
            if (sae.PCurve((dgnr ? E3 : E2), myFace, out c2d, out a, out b, true))
            {
                p2d2 = c2d!.Value(a);
            }
            else
            {
                myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
            }
        }

        // #84 rln 18.03.99 if pcurve is not degenerate anymore, the fix is postponned
        // to ShapeFix_Wire::FixLacking
        if (!mySurf!.IsDegenerated(p2d1, p2d2, precVtx, 10.0))
        { //: s1 abv 22 Apr 99: PRO7226 #489490 //smh#9
            //: abv 24.05.02: OCC320 - fail (to remove edge) if two consecutive degenerated edges w/o
            //: pcurves
            if (BRep_Tool.Degenerated(E2))
            {
                myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
            }

            return false;
        }

        // added by rln 18/12/97 CSR# CTS18544 entity 13638
        // the situation when degenerated edge already exists but flag is not set
        //(i.e. the parametric space is closed)
        GeomAdaptor_Surface Ads = mySurf.Adaptor3d()!;
        double              max = Max(Ads.UResolution(myPrecision), Ads.VResolution(myPrecision));
        if (p2d1.Distance(p2d2) /*Abs (par1 - par2)*/ <= max + gp.Resolution())
        {
            return false;
        }

        myStatus = ShapeExtend.EncodeStatus(dgnr ? ShapeExtend_DONE2 : ShapeExtend_DONE1);
        return true;
    }

    /// <summary>Checks for a degenerated edge between two adjacent ones (see the other overload).</summary>
    public bool CheckDegenerated(int num)
    {
        gp_Pnt2d p2d1 = new gp_Pnt2d(), p2d2 = new gp_Pnt2d();
        return CheckDegenerated(num, ref p2d1, ref p2d2);
    }

    /// <summary>
    /// Checks the 3D gap between the ends of the 3d curves of the num-th and the preceding edge (in
    /// <see cref="MinDistance3d"/>). Status: DONE the gap is over the precision; FAIL an edge has no 3d curve.
    /// </summary>
    public bool CheckGap3d(int num = 0)
    {
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        // szv#4:S4163:12Mar99 optimized
        if (!IsLoaded() || NbEdges() < 1)
        {
            return false; // szvsh was nbedges < 2
        }

        int                n2 = (num > 0 ? num : NbEdges());
        int                n1 = (n2 > 1 ? n2 - 1 : NbEdges());
        TopoDS_Edge        E1 = myWire!.Edge(n1);
        TopoDS_Edge        E2 = myWire.Edge(n2);
        Geom_Curve?        C2 = null;
        double             uf2 = 0.0, ul1;
        ShapeAnalysis_Edge SAE = new ShapeAnalysis_Edge();
        if (!SAE.Curve3d(E1, out Geom_Curve? C1, out _, out ul1) || !SAE.Curve3d(E2, out C2, out uf2, out _))
        {
            myStatus = ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
            return false;
        }

        gp_Pnt p1 = C1!.Value(ul1);
        gp_Pnt p2 = C2!.Value(uf2);
        myMin3d = myMax3d = p1.Distance(p2);
        if (myMin3d > myPrecision)
        {
            myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
        }

        return LastCheckStatus(ShapeExtend_DONE);
    }

    /// <summary>
    /// Checks the 2D gap between the ends of the pcurves of the num-th and the preceding edge (in
    /// <see cref="MinDistance2d"/>). Status: DONE the gap is over the parametric precision; FAIL an edge has no pcurve.
    /// </summary>
    public bool CheckGap2d(int num = 0)
    {
        if (myFace.IsNull() || mySurf is null)
        {
            myStatus = ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
            return false;
        }

        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        // szv#4:S4163:12Mar99 optimized
        if (!IsReady() || NbEdges() < 1)
        {
            return false; // szvsh was nbedges < 2
        }

        int                n2 = (num > 0 ? num : NbEdges());
        int                n1 = (n2 > 1 ? n2 - 1 : NbEdges());
        TopoDS_Edge        E1 = myWire!.Edge(n1);
        TopoDS_Edge        E2 = myWire.Edge(n2);
        Geom2d_Curve?      C2 = null;
        double             uf2 = 0.0, ul1;
        ShapeAnalysis_Edge SAE = new ShapeAnalysis_Edge();
        if (!SAE.PCurve(E1, myFace, out Geom2d_Curve? C1, out _, out ul1) || !SAE.PCurve(E2, myFace, out C2, out uf2, out _))
        {
            myStatus = ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
            return false;
        }

        gp_Pnt2d p1 = C1!.Value(ul1);
        gp_Pnt2d p2 = C2!.Value(uf2);
        myMin2d = myMax2d = p1.Distance(p2);
        GeomAdaptor_Surface SA = mySurf.Adaptor3d()!;
        if (myMin2d > (Max(SA.UResolution(myPrecision), SA.VResolution(myPrecision)) + Occt.Precision.PConfusion()))
        {
            myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
        }

        return LastCheckStatus(ShapeExtend_DONE);
    }

    /// <summary>
    /// Checks the gap between the 3d curve of the num-th edge and its pcurve on the surface, at 45 points (in
    /// <see cref="MinDistance3d"/>). Status: DONE the gap is over the precision; FAIL no 3d curve or no pcurve.
    /// </summary>
    public bool CheckCurveGap(int num = 0)
    {
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsLoaded() || NbEdges() < 1)
        {
            return false;
        }

        int                n   = (num > 0 ? num : NbEdges());
        TopoDS_Edge        E   = myWire!.Edge(n);
        ShapeAnalysis_Edge SAE = new ShapeAnalysis_Edge();
        if (!SAE.Curve3d(E, out Geom_Curve? c, out double cuf, out double cul, false))
        {
            myStatus = ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
            return false;
        }

        if (!SAE.PCurve(E, myFace, out Geom2d_Curve? pc, out double pcuf, out double pcul, false))
        {
            myStatus = ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
            return false;
        }

        Geom2dAdaptor_Curve      AC  = new Geom2dAdaptor_Curve(pc!, pcuf, pcul);
        GeomAdaptor_Surface      AS  = new GeomAdaptor_Surface(mySurf!.Surface()!);
        Adaptor3d_CurveOnSurface ACS = new Adaptor3d_CurveOnSurface(AC, AS);
        gp_Pnt                   cpnt, pcpnt;
        int                      nbp = 45;
        double                   dist, maxdist = 0.0;
        for (int i = 0; i < nbp; i++)
        {
            cpnt  = c!.Value(cuf + i * (cul - cuf) / (nbp - 1));
            pcpnt = ACS.Value(pcuf + i * (pcul - pcuf) / (nbp - 1));
            dist  = cpnt.SquareDistance(pcpnt);
            if (maxdist < dist)
            {
                maxdist = dist;
            }
        }

        myMin3d = myMax3d = Math.Sqrt(maxdist);
        if (myMin3d > myPrecision)
        {
            myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
        }

        return LastCheckStatus(ShapeExtend_DONE);
    }

    // auxiliary function
    //: h0 abv 29 May 98: PRO10105 1949: like in BRepCheck, point is to be taken
    // from 3d curve (but only if edge is SameParameter)
    private static gp_Pnt GetPointOnEdge(in TopoDS_Edge edge, ShapeAnalysis_Surface surf, Geom2dAdaptor_Curve Crv2d, double param)
    {
        if (BRep_Tool.SameParameter(edge))
        {
            Geom_Curve? ConS = BRep_Tool.Curve(edge, out TopLoc_Location L, out _, out _);
            if (ConS is not null)
            {
                return ConS.Value(param).Transformed(L.Transformation());
            }
        }

        gp_Pnt2d aP2d = Crv2d.Value(param);
        return surf.Adaptor3d()!.Value(aP2d.X(), aP2d.Y());
    }

    /// <summary>
    /// Checks if the num-th edge is self-intersecting (on its pcurve), away from both its end vertices. Fills the
    /// intersection points and their 3d points. Status: DONE1 found; FAIL1 no pcurve; FAIL2 no vertices.
    /// </summary>
    public bool CheckSelfIntersectingEdge(int num, NCollection_Sequence<IntRes2d_IntersectionPoint> points2d,
                                          NCollection_Sequence<gp_Pnt> points3d)
    {
        points2d.Clear();
        points3d.Clear();
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsReady())
        {
            return false;
        }

        TopoDS_Edge        edge = WireData()!.Edge(num > 0 ? num : NbEdges());
        ShapeAnalysis_Edge sae  = new ShapeAnalysis_Edge();

        if (!sae.PCurve(edge, myFace, out Geom2d_Curve? Crv, out double a, out double b, false))
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
            return false;
        }

        if (Math.Abs(a - b) <= Occt.Precision.PConfusion())
        {
            return false;
        }

        double tolint = 1.0e-10;
        // szv#4:S4163:12Mar99 warning
        IntRes2d_Domain     domain = new IntRes2d_Domain(Crv!.Value(a), a, tolint, Crv.Value(b), b, tolint);
        Geom2dAdaptor_Curve AC     = new Geom2dAdaptor_Curve(Crv);
        Geom2dInt_GInter    Inter  = new Geom2dInt_GInter(AC, domain, tolint, tolint);
        if (!Inter.IsDone())
        {
            return false;
        }

        TopoDS_Vertex V1 = sae.FirstVertex(edge);
        TopoDS_Vertex V2 = sae.LastVertex(edge);
        if (V1.IsNull() || V2.IsNull())
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
            return false;
        }

        double tol1 = BRep_Tool.Tolerance(V1);
        double tol2 = BRep_Tool.Tolerance(V2);
        gp_Pnt pnt1 = BRep_Tool.Pnt(V1);
        gp_Pnt pnt2 = BRep_Tool.Pnt(V2);
        for (int i = 1; i <= Inter.NbPoints(); i++)
        {
            IntRes2d_IntersectionPoint IP  = Inter.Point(i);
            IntRes2d_Transition        Tr1 = IP.TransitionOfFirst();
            IntRes2d_Transition        Tr2 = IP.TransitionOfSecond();
            if (Tr1.PositionOnCurve() != IntRes2d_Middle && Tr2.PositionOnCurve() != IntRes2d_Middle)
            {
                continue;
            }

            gp_Pnt pint   = GetPointOnEdge(edge, mySurf!, AC, IP.ParamOnFirst());
            double dist21 = pnt1.SquareDistance(pint);
            double dist22 = pnt2.SquareDistance(pint);
            if (dist21 > tol1 * tol1 && dist22 > tol2 * tol2)
            {
                points2d.Append(IP);
                points3d.Append(pint);
                myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
            }
        }

        return LastCheckStatus(ShapeExtend_DONE);
    }

    /// <summary>Checks if the num-th edge is self-intersecting (see the other overload).</summary>
    public bool CheckSelfIntersectingEdge(int num)
    {
        NCollection_Sequence<IntRes2d_IntersectionPoint> points2d = new NCollection_Sequence<IntRes2d_IntersectionPoint>();
        NCollection_Sequence<gp_Pnt>                     points3d = new NCollection_Sequence<gp_Pnt>();
        return CheckSelfIntersectingEdge(num, points2d, points3d);
    }

    /// <summary>
    /// Checks if the num-th edge and the preceding one intersect (on their pcurves) at a point not enclosed by their
    /// common vertex, as BRepCheck_Wire does. Fills the intersection points, their 3d points and errors (half the
    /// distance between the 3d points from either edge). Status: DONE1 found; FAIL1 no vertices; FAIL2 the edges don't
    /// share a vertex; FAIL3 no pcurve.
    /// </summary>
    public bool CheckIntersectingEdges(int num, NCollection_Sequence<IntRes2d_IntersectionPoint> points2d,
                                       NCollection_Sequence<gp_Pnt> points3d, NCollection_Sequence<double> errors)
    {
        points2d.Clear();
        points3d.Clear();
        errors.Clear();
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsReady() || NbEdges() < 2)
        {
            return false;
        }

        // szv#4:S4163:12Mar99 optimized
        int         n2    = (num > 0) ? num : NbEdges();
        int         n1    = (n2 > 1) ? n2 - 1 : NbEdges();
        TopoDS_Edge edge1 = myWire!.Edge(n1);
        TopoDS_Edge edge2 = myWire.Edge(n2);

        ShapeAnalysis_Edge sae = new ShapeAnalysis_Edge();
        TopoDS_Vertex      V1  = sae.LastVertex(edge1);
        TopoDS_Vertex      V2  = sae.FirstVertex(edge2);
        if (V1.IsNull() || V2.IsNull())
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
            return false;
        }

        if (!BRepTools.Compare(V1, V2))
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
            return false;
        }

        TopoDS_Vertex Vp = sae.FirstVertex(edge1);
        TopoDS_Vertex Vn = sae.LastVertex(edge2);

        if (!sae.PCurve(edge1, myFace, out Geom2d_Curve? Crv1, out double a1, out double b1, false))
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL3);
            return false;
        }

        if (!sae.PCurve(edge2, myFace, out Geom2d_Curve? Crv2, out double a2, out double b2, false))
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL3);
            return false;
        }

        if (Math.Abs(a1 - b1) <= Occt.Precision.PConfusion() || Math.Abs(a2 - b2) <= Occt.Precision.PConfusion())
        {
            return false; //:f7 abv 6 May 98: BUC50070 on #42276
        }

        bool isForward1 = (edge1.Orientation() == TopAbs_FORWARD);
        bool isForward2 = (edge2.Orientation() == TopAbs_FORWARD);

        double tol0 = Max(BRep_Tool.Tolerance(V1), BRep_Tool.Tolerance(V2));
        double tol  = tol0;

        gp_Pnt pnt = BRep_Tool.Pnt(V1);

        double tolint = 1.0e-10;
        // szv#4:S4163:12Mar99 warning
        Geom2dAdaptor_Curve C1 = new Geom2dAdaptor_Curve(Crv1!), C2 = new Geom2dAdaptor_Curve(Crv2!);
        IntRes2d_Domain     d1 = new IntRes2d_Domain(C1.Value(a1), a1, tolint, C1.Value(b1), b1, tolint);
        IntRes2d_Domain     d2 = new IntRes2d_Domain(C2.Value(a2), a2, tolint, C2.Value(b2), b2, tolint);

        //: 64 abv 25 Dec 97: Attention!
        // Since Intersection algorithm is not symmetrical, for consistency with BRepCheck
        // edge with lower order number should be intersecting with edge with higher one
        // i.e., for intersection of last and first edges, they should go in reversed order
        // Example: entity #38285 from bug CSR #CTS17806
        // NOTE: Tr1 and Tr2 are not reordered because they are used in the same manner
        Geom2dInt_GInter Inter = new Geom2dInt_GInter();
        if (num == 1)
        {
            Inter.Perform(C2, d2, C1, d1, tolint, tolint);
        }
        else
        {
            Inter.Perform(C1, d1, C2, d2, tolint, tolint);
        }

        if (!Inter.IsDone())
        {
            return false;
        }

        //: 86 abv 22 Jan 98: fix self-intersection even if tolerance of vertex is enough
        // to annihilate it. This is done to prevent wrong effects if vertex tolerance
        // will be decreased (e.g., in FixLacking)
        double tole = Max((BRep_Tool.SameParameter(edge1) ? BRep_Tool.Tolerance(edge1) : tol0),
                          (BRep_Tool.SameParameter(edge2) ? BRep_Tool.Tolerance(edge2) : tol0));
        double tolt = Min(tol, Max(tole, myPrecision));
        int isLacking = -1; //: l0 abv: CATIA01 #1727: protect against adding lacking
        // #83 rln 19.03.99 sim2.igs, entity 4292
        // processing also segments as in BRepCheck
        int NbPoints = Inter.NbPoints(), NbSegments = Inter.NbSegments();
        for (int i = 1; i <= NbPoints + NbSegments; i++)
        {
            IntRes2d_IntersectionPoint IP;
            IntRes2d_Transition        Tr1, Tr2;
            if (i <= NbPoints)
            {
                IP = Inter.Point(i);
            }
            else
            {
                IntRes2d_IntersectionSegment Seg = Inter.Segment(i - NbPoints);
                if (!Seg.HasFirstPoint() || !Seg.HasLastPoint())
                {
                    continue;
                }

                IP  = Seg.FirstPoint();
                Tr1 = IP.TransitionOfFirst();
                Tr2 = IP.TransitionOfSecond();
                if (Tr1.PositionOnCurve() == IntRes2d_Middle || Tr2.PositionOnCurve() == IntRes2d_Middle)
                {
                    IP = Seg.LastPoint();
                }
            }

            Tr1 = IP.TransitionOfFirst();
            Tr2 = IP.TransitionOfSecond();
            if (Tr1.PositionOnCurve() != IntRes2d_Middle && Tr2.PositionOnCurve() != IntRes2d_Middle)
            {
                continue;
            }

            double param1, param2;
            param1 = (num == 1 ? IP.ParamOnSecond() : IP.ParamOnFirst());
            param2 = (num == 1 ? IP.ParamOnFirst() : IP.ParamOnSecond());

            //: r6 abv 8 Apr 99: r_47-sd.stp #173850: protect against working out of curve range
            if (a1 - param1 > Occt.Precision.PConfusion() || param1 - b1 > Occt.Precision.PConfusion()
                || a2 - param2 > Occt.Precision.PConfusion() || param2 - b2 > Occt.Precision.PConfusion())
            {
                continue;
            }

            //: 82 abv 21 Jan 98: point of intersection on Crv1 and Crv2 is different
            gp_Pnt pi1   = GetPointOnEdge(edge1, mySurf!, C1, param1); //:h0: thesurf.Value ( Crv1->Value ( param1 ) );
            gp_Pnt pi2   = GetPointOnEdge(edge2, mySurf!, C2, param2); //:h0: thesurf.Value ( Crv2->Value ( param2 ) );
            gp_Pnt pint  = new gp_Pnt(0.5 * (pi1.XYZ() + pi2.XYZ()));
            double di1   = pi1.SquareDistance(pnt);
            double di2   = pi2.SquareDistance(pnt);
            double dist2 = Max(di1, di2);

            // rln 03/02/98: CSR#BUC50004 entity 56 (to avoid later inserting lacking edge)
            if (isLacking < 0)
            { //: l0
                gp_Pnt2d end1 = Crv1!.Value(isForward1 ? b1 : a1);
                gp_Pnt2d end2 = Crv2!.Value(isForward2 ? a2 : b2);
                //: l0 test like in BRepCheck
                GeomAdaptor_Surface Ads   = mySurf!.Adaptor3d()!;
                double              tol2d = 2 * Max(Ads.UResolution(tol), Ads.VResolution(tol));
                isLacking                 = (end1.SquareDistance(end2) >= tol2d * tol2d) ? 1 : 0;
            }

            if ((dist2 > tolt * tolt || //: 86: tol -> tolt
                 isLacking != 0)
                && //: l0
                (!BRepTools.Compare(Vp, Vn) || //: 63
                 dist2 < pint.SquareDistance(BRep_Tool.Pnt(Vp))))
            { //: 63
                points2d.Append(IP);
                points3d.Append(pint);
                errors.Append(0.5 * pi1.Distance(pi2));
                myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
            }
        }

        return LastCheckStatus(ShapeExtend_DONE);
    }

    /// <summary>Checks if the num-th edge and the preceding one intersect (see the other overload).</summary>
    public bool CheckIntersectingEdges(int num)
    {
        NCollection_Sequence<IntRes2d_IntersectionPoint> points2d = new NCollection_Sequence<IntRes2d_IntersectionPoint>();
        NCollection_Sequence<gp_Pnt>                     points3d = new NCollection_Sequence<gp_Pnt>();
        NCollection_Sequence<double>                     errors   = new NCollection_Sequence<double>();
        return CheckIntersectingEdges(num, points2d, points3d, errors);
    }

    /// <summary>
    /// Checks if the num1-th and num2-th edges intersect (on their pcurves) at a point not enclosed by their vertices.
    /// Status: DONE1 found; FAIL3 no pcurve.
    /// </summary>
    public bool CheckIntersectingEdges(int num1, int num2, NCollection_Sequence<IntRes2d_IntersectionPoint> points2d,
                                       NCollection_Sequence<gp_Pnt> points3d, NCollection_Sequence<double> errors)
    {
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsReady())
        {
            return false;
        }

        ShapeExtend_WireData sbwd = WireData()!;
        int                  n2   = (num2 > 0 ? num2 : sbwd.NbEdges());
        int                  n1   = (num1 > 0 ? num1 : sbwd.NbEdges());

        TopoDS_Edge edge1 = sbwd.Edge(n1);
        TopoDS_Edge edge2 = sbwd.Edge(n2);

        ShapeAnalysis_Edge sae = new ShapeAnalysis_Edge();

        if (!sae.PCurve(edge1, myFace, out Geom2d_Curve? Crv1, out double a1, out double b1, false))
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL3);
            return false;
        }

        if (!sae.PCurve(edge2, myFace, out Geom2d_Curve? Crv2, out double a2, out double b2, false))
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL3);
            return false;
        }

        if (Math.Abs(a1 - b1) <= Occt.Precision.PConfusion() || Math.Abs(a2 - b2) <= Occt.Precision.PConfusion())
        {
            return false;
        }

        points2d.Clear();
        points3d.Clear();
        errors.Clear();
        NCollection_Array1<gp_Pnt> vertexPoints = new NCollection_Array1<gp_Pnt>(1, 4);
        NCollection_Array1<double> vertexTolers = new NCollection_Array1<double>(1, 4);
        vertexPoints[1] = BRep_Tool.Pnt(sae.FirstVertex(edge1));
        vertexTolers[1] = BRep_Tool.Tolerance(sae.FirstVertex(edge1));
        vertexPoints[2] = BRep_Tool.Pnt(sae.LastVertex(edge1));
        vertexTolers[2] = BRep_Tool.Tolerance(sae.LastVertex(edge1));
        vertexPoints[3] = BRep_Tool.Pnt(sae.FirstVertex(edge2));
        vertexTolers[3] = BRep_Tool.Tolerance(sae.FirstVertex(edge2));
        vertexPoints[4] = BRep_Tool.Pnt(sae.LastVertex(edge2));
        vertexTolers[4] = BRep_Tool.Tolerance(sae.LastVertex(edge2));

        double tolint = 1.0e-10;
        IntRes2d_Domain     d1 = new IntRes2d_Domain(Crv1!.Value(a1), a1, tolint, Crv1.Value(b1), b1, tolint);
        IntRes2d_Domain     d2 = new IntRes2d_Domain(Crv2!.Value(a2), a2, tolint, Crv2.Value(b2), b2, tolint);
        Geom2dAdaptor_Curve C1 = new Geom2dAdaptor_Curve(Crv1), C2 = new Geom2dAdaptor_Curve(Crv2);

        Geom2dInt_GInter Inter = new Geom2dInt_GInter();
        Inter.Perform(C1, d1, C2, d2, tolint, tolint);
        if (!Inter.IsDone())
        {
            return false;
        }

        // #83 rln 19.03.99 sim2.igs, entity 4292
        // processing also segments as in BRepCheck
        int NbPoints = Inter.NbPoints(), NbSegments = Inter.NbSegments();
        for (int i = 1; i <= NbPoints + NbSegments; i++)
        {
            IntRes2d_IntersectionPoint IP;
            IntRes2d_Transition        Tr1, Tr2;
            if (i <= NbPoints)
            {
                IP = Inter.Point(i);
            }
            else
            {
                IntRes2d_IntersectionSegment Seg = Inter.Segment(i - NbPoints);
                if (!Seg.HasFirstPoint() || !Seg.HasLastPoint())
                {
                    continue;
                }

                IP  = Seg.FirstPoint();
                Tr1 = IP.TransitionOfFirst();
                Tr2 = IP.TransitionOfSecond();
                if (Tr1.PositionOnCurve() == IntRes2d_Middle || Tr2.PositionOnCurve() == IntRes2d_Middle)
                {
                    IP = Seg.LastPoint();
                }
            }

            Tr1 = IP.TransitionOfFirst();
            Tr2 = IP.TransitionOfSecond();
            if (Tr1.PositionOnCurve() != IntRes2d_Middle && Tr2.PositionOnCurve() != IntRes2d_Middle)
            {
                continue;
            }

            double param1 = IP.ParamOnFirst();
            double param2 = IP.ParamOnSecond();
            gp_Pnt pi1    = GetPointOnEdge(edge1, mySurf!, C1, param1); //:h0: thesurf.Value ( Crv1->Value ( param1 ) );
            gp_Pnt pi2    = GetPointOnEdge(edge2, mySurf!, C2, param2);
            bool   OK1    = false;
            bool   OK2    = false;

            for (int j = 1; (j <= 2) && !OK1; j++)
            {
                double di1 = pi1.SquareDistance(vertexPoints[j]);
                if (di1 < vertexTolers[j] * vertexTolers[j])
                {
                    OK1 = true;
                }
            }

            for (int j = 3; (j <= 4) && !OK2; j++)
            {
                double di2 = pi2.SquareDistance(vertexPoints[j]);
                if (di2 < vertexTolers[j] * vertexTolers[j])
                {
                    OK2 = true;
                }
            }

            if (!OK1 || !OK2)
            {
                gp_Pnt pint = new gp_Pnt(0.5 * (pi1.XYZ() + pi2.XYZ()));
                points2d.Append(IP);
                points3d.Append(pint);
                errors.Append(0.5 * pi1.Distance(pi2));
                myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
            }
        }

        return LastCheckStatus(ShapeExtend_DONE);
    }

    /// <summary>Checks if the num1-th and num2-th edges intersect (see the other overload).</summary>
    public bool CheckIntersectingEdges(int num1, int num2)
    {
        NCollection_Sequence<IntRes2d_IntersectionPoint> points2d = new NCollection_Sequence<IntRes2d_IntersectionPoint>();
        NCollection_Sequence<gp_Pnt>                     points3d = new NCollection_Sequence<gp_Pnt>();
        NCollection_Sequence<double>                     errors   = new NCollection_Sequence<double>();
        return CheckIntersectingEdges(num1, num2, points2d, points3d, errors);
    }

    /// <summary>
    /// Checks if there is a 2d gap between the num-th edge and the preceding one not covered by the tolerance of their
    /// common vertex (or by <paramref name="Tolerance"/> when it is positive and smaller). <paramref name="p2d1"/> and
    /// <paramref name="p2d2"/> get the end of the preceding pcurve and the start of the num-th one.
    /// </summary>
    /// <remarks>
    /// Status: DONE1 a gap is found; DONE2 with DONE1 when the gap goes against the pcurves (an angle over 0.9 π); FAIL1
    /// no vertices; FAIL2 the edges don't share a vertex; FAIL3 no pcurve.
    /// </remarks>
    public bool CheckLacking(int num, double Tolerance, ref gp_Pnt2d p2d1, ref gp_Pnt2d p2d2)
    {
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsReady())
        {
            return false;
        }

        // szv#4:S4163:12Mar99 optimized
        int         n2 = (num > 0) ? num : NbEdges();
        int         n1 = (n2 > 1) ? n2 - 1 : NbEdges();
        TopoDS_Edge E1 = myWire!.Edge(n1);
        TopoDS_Edge E2 = myWire.Edge(n2);

        ShapeAnalysis_Edge sae = new ShapeAnalysis_Edge();
        TopoDS_Vertex      V1  = sae.LastVertex(E1);
        TopoDS_Vertex      V2  = sae.FirstVertex(E2);

        // CKY 4 MAR 1998 : protection against null vertex
        if (V1.IsNull() || V2.IsNull())
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
            return false;
        }

        if (!BRepTools.Compare(V1, V2))
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
            return false;
        }

        gp_Vec2d v1, v2, v12;

        if (!sae.PCurve(E1, myFace, out Geom2d_Curve? c2d, out double a, out double b, true))
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL3);
            return false;
        }

        Geom2dAdaptor_Curve anAdapt = new Geom2dAdaptor_Curve(c2d!);
        anAdapt.D1(b, out p2d1, out v1);
        if (E1.Orientation() == TopAbs_REVERSED)
        {
            v1.Reverse();
        }

        if (!sae.PCurve(E2, myFace, out c2d, out a, out b, true))
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL3);
            return false;
        }

        anAdapt.Load(c2d!);
        anAdapt.D1(a, out p2d2, out v2);
        if (E2.Orientation() == TopAbs_REVERSED)
        {
            v2.Reverse();
        }

        v12     = new gp_Vec2d(p2d2.XY() - p2d1.XY());
        myMax2d = v12.SquareMagnitude();

        // test like in BRepCheck
        double tol = Max(BRep_Tool.Tolerance(V1), BRep_Tool.Tolerance(V2));
        tol        = (Tolerance > gp.Resolution() && Tolerance < tol ? Tolerance : tol);
        GeomAdaptor_Surface Ads   = mySurf!.Adaptor3d()!;
        double              tol2d = 2 * Max(Ads.UResolution(tol), Ads.VResolution(tol));
        if ( // tol2d < gp::Resolution() || //#2 smh 26.03.99 S4163 Zero divide
            myMax2d < tol2d * tol2d)
        {
            return false;
        }

        myMax2d = Math.Sqrt(myMax2d);
        myMax3d = tol * myMax2d / Max(tol2d, gp.Resolution());
        myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
        if (myMax2d < Occt.Precision.PConfusion() || //: abv 03.06.02 CTS21866.stp
            (v1.SquareMagnitude() > gp.Resolution() && Math.Abs(v12.Angle(v1)) > 0.9 * M_PI)
            || (v2.SquareMagnitude() > gp.Resolution() && Math.Abs(v12.Angle(v2)) > 0.9 * M_PI))
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
        }

        return true;
    }

    /// <summary>
    /// Checks if there is a 2d gap between the num-th edge and the preceding one not covered by the vertex tolerance
    /// (see the other overload).
    /// </summary>
    public bool CheckLacking(int num, double Tolerance = 0.0)
    {
        gp_Pnt2d p1 = new gp_Pnt2d(), p2 = new gp_Pnt2d();
        return CheckLacking(num, Tolerance, ref p1, ref p2);
    }

    /// <summary>
    /// Checks if the wire is an outer bound of its face (ShapeAnalysis.IsOuterBound on a copy of the face holding only
    /// it). With <paramref name="APIMake"/> the wire is built with BRepBuilderAPI_MakeWire, else with BRep_Builder (only
    /// when the edges share their vertices). Status: DONE1 not outer.
    /// </summary>
    public bool CheckOuterBound(bool APIMake = true)
    {
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsReady())
        {
            return false;
        }

        TopoDS_Wire wire;
        if (APIMake)
        {
            wire = myWire!.InternalWireAPIMake();
        }
        else
        {
            wire = myWire!.InternalWire();
        }

        TopoDS_Shape sh   = myFace.EmptyCopied(); // szv#4:S4163:12Mar99 SGI warns
        TopoDS_Face  face = TopoDS.Face(sh);
        BRep_Builder B    = new BRep_Builder();
        B.Add(face, wire);
        if (ShapeAnalysis.IsOuterBound(face))
        {
            return false;
        }

        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
        return true;
    }

    private static double ProjectInside(Adaptor3d_CurveOnSurface AD, in gp_Pnt pnt, double preci, out gp_Pnt proj,
                                        out double param, bool adjustToEnds = true)
    {
        ShapeAnalysis_Curve sac    = new ShapeAnalysis_Curve();
        double              dist   = sac.Project(AD, pnt, preci, out proj, out param, adjustToEnds);
        double              uFirst = AD.FirstParameter();
        double              uLast  = AD.LastParameter();
        if (param < uFirst)
        {
            param = uFirst;
            proj  = AD.Value(uFirst);
            return proj.Distance(pnt);
        }

        if (param > uLast)
        {
            param = uLast;
            proj  = AD.Value(uLast);
            return proj.Distance(pnt);
        }

        return dist;
    }

    /// <summary>
    /// Detects a notch: the num-th edge and the preceding one leave their common vertex in the same direction (within
    /// 0.1 rad) and one runs along the other within <paramref name="Tolerance"/>. <paramref name="shortNum"/> gets the
    /// number of the shorter edge and <paramref name="param"/> the parameter on the longer one where the shorter ends
    /// (both left unchanged when false is returned early). Status: FAIL1 no vertices; FAIL2 the edges don't share a
    /// vertex; FAIL3 no pcurve.
    /// </summary>
    public bool CheckNotchedEdges(int num, ref int shortNum, ref double param, double Tolerance = 0.0)
    {
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsReady())
        {
            return false;
        }

        int         n2 = (num > 0) ? num : NbEdges();
        int         n1 = (n2 > 1) ? n2 - 1 : NbEdges();
        TopoDS_Edge E1 = myWire!.Edge(n1);
        TopoDS_Edge E2 = myWire.Edge(n2);

        if (BRep_Tool.Degenerated(E1) || BRep_Tool.Degenerated(E2))
        {
            return false;
        }

        ShapeAnalysis_Edge sae = new ShapeAnalysis_Edge();
        TopoDS_Vertex      V1  = sae.LastVertex(E1);
        TopoDS_Vertex      V2  = sae.FirstVertex(E2);

        if (V1.IsNull() || V2.IsNull())
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
            return false;
        }

        if (!BRepTools.Compare(V1, V2))
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
            return false;
        }

        gp_Pnt2d p2d1, p2d2;
        gp_Vec2d v1, v2;
        if (!sae.PCurve(E1, myFace, out Geom2d_Curve? c2d1, out double a1, out double b1, false))
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL3);
            return false;
        }

        if (E1.Orientation() == TopAbs_REVERSED)
        {
            c2d1!.D1(a1, out p2d1, out v1);
        }
        else
        {
            c2d1!.D1(b1, out p2d1, out v1);
            v1.Reverse();
        }

        if (!sae.PCurve(E2, myFace, out Geom2d_Curve? c2d2, out double a2, out double b2, false))
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL3);
            return false;
        }

        if (E2.Orientation() == TopAbs_REVERSED)
        {
            c2d2!.D1(b2, out p2d2, out v2);
            v2.Reverse();
        }
        else
        {
            c2d2!.D1(a2, out p2d2, out v2);
        }

        if (v2.Magnitude() < gp.Resolution() || v1.Magnitude() < gp.Resolution())
        {
            return false;
        }

        if (Math.Abs(v2.Angle(v1)) > 0.1 || p2d1.Distance(p2d2) > Tolerance)
        {
            return false;
        }

        Geom2dAdaptor_Curve      AC2d1 = new Geom2dAdaptor_Curve(c2d1, a1, b1);
        GeomAdaptor_Surface      AdS1  = new GeomAdaptor_Surface(new Geom_Plane(new gp_Pln()));
        Adaptor3d_CurveOnSurface Ad1   = new Adaptor3d_CurveOnSurface(AC2d1, AdS1);

        Geom2dAdaptor_Curve      AC2d2 = new Geom2dAdaptor_Curve(c2d2, a2, b2);
        GeomAdaptor_Surface      AdS2  = new GeomAdaptor_Surface(new Geom_Plane(new gp_Pln()));
        Adaptor3d_CurveOnSurface Ad2   = new Adaptor3d_CurveOnSurface(AC2d2, AdS2);

        Adaptor3d_CurveOnSurface longAD, shortAD;
        double                   lenP, firstP;

        ShapeAnalysis_Curve sac = new ShapeAnalysis_Curve();

        p2d2         = c2d2.Value(E2.Orientation() == TopAbs_FORWARD ? b2 : a2);
        p2d1         = c2d1.Value(E1.Orientation() == TopAbs_FORWARD ? a1 : b1);
        double dist1 = ProjectInside(Ad1, new gp_Pnt(p2d2.X(), p2d2.Y(), 0), Tolerance, out gp_Pnt Proj1, out double param1, false);
        double dist2 = ProjectInside(Ad2, new gp_Pnt(p2d1.X(), p2d1.Y(), 0), Tolerance, out _, out double param2, false);

        if (dist1 > Tolerance && dist2 > Tolerance)
        {
            return false;
        }

        if (dist1 < dist2)
        {
            shortAD  = Ad2;
            longAD   = Ad1;
            lenP     = b2 - a2;
            firstP   = a2;
            shortNum = n2;
            param    = param1;
        }
        else
        {
            shortAD  = Ad1;
            longAD   = Ad2;
            lenP     = b1 - a1;
            firstP   = a1;
            shortNum = n1;
            param    = param2;
        }

        double step = lenP / 23;
        for (int i = 1; i < 23; i++, firstP += step)
        {
            double d1 = sac.Project(longAD, shortAD.Value(firstP), Tolerance, out Proj1, out param1);
            if (d1 > Tolerance)
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
    public bool CheckSmallArea(in TopoDS_Wire theWire)
    {
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
        const int aNbControl = 23;
        int NbEdges = myWire!.NbEdges();
        if (!IsReady() || NbEdges < 1)
        {
            return false;
        }

        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        double       aF, aL, aLength = 0.0;
        double       anInv     = 1.0 / (double)(aNbControl - 1);
        gp_XY        aCenter2d = new gp_XY(0.0, 0.0);

        // try to find mid point for closed contour
        for (int j = 1; j <= NbEdges; ++j)
        {
            ShapeAnalysis_Edge anAnalyzer = new ShapeAnalysis_Edge();
            if (!anAnalyzer.PCurve(myWire.Edge(j), myFace, out Geom2d_Curve? aCurve2d, out aF, out aL))
            {
                myStatus = ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
                return false;
            }

            for (int i = 1; i < aNbControl; ++i)
            {
                double aV = anInv * ((aNbControl - 1 - i) * aF + i * aL);
                aCenter2d += aCurve2d!.Value(aV).XY();
            }
        }

        aCenter2d *= 1.0 / (double)(NbEdges * (aNbControl - 1));

        // check approximated area in 3D
        gp_Pnt aPnt3d  = new gp_Pnt();
        gp_XYZ aPrev3d = new gp_XYZ(), aCross = new gp_XYZ(0.0, 0.0, 0.0);
        gp_XYZ aCenter = mySurf!.Value(aCenter2d.X(), aCenter2d.Y()).XYZ();

        for (int j = 1; j <= NbEdges; ++j)
        {
            ShapeAnalysis_Edge anAnalizer = new ShapeAnalysis_Edge();
            if (!anAnalizer.Curve3d(myWire.Edge(j), out Geom_Curve? aCurve3d, out aF, out aL))
            {
                myStatus = ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
                return false;
            }

            if (Occt.Precision.IsInfinite(aF) || Occt.Precision.IsInfinite(aL))
            {
                continue;
            }

            int aBegin = 0;
            if (j == 1)
            {
                aBegin  = 1;
                aPnt3d  = aCurve3d!.Value(aF);
                aPrev3d = aPnt3d.XYZ() - aCenter;
            }

            for (int i = aBegin; i < aNbControl; ++i)
            {
                double anU    = anInv * ((aNbControl - 1 - i) * aF + i * aL);
                gp_Pnt aPnt     = aCurve3d!.Value(anU);
                gp_XYZ aCurrent = aPnt.XYZ();
                gp_XYZ aVec     = aCurrent - aCenter;

                aCross += aPrev3d ^ aVec;
                aLength += aPnt3d.Distance(aPnt);

                aPnt3d  = aPnt;
                aPrev3d = aVec;
            }
        }

        double aTolerance = aLength * myPrecision;
        if (aCross.Modulus() < aTolerance)
        {
            // check real area in 3D
            GProp_GProps aProps  = new GProp_GProps();
            GProp_GProps aLProps = new GProp_GProps();
            TopoDS_Face  aFace   = TopoDS.Face(myFace.EmptyCopied());
            new BRep_Builder().Add(aFace, theWire);
            BRepGProp.SurfaceProperties(aFace, aProps);
            BRepGProp.LinearProperties(aFace, aLProps);

            double aNewTolerance = aLProps.Mass() * myPrecision;
            if (Math.Abs(aProps.Mass()) < 0.5 * aNewTolerance)
            {
                myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
                return true;
            }
        }

        return false;
    }

    /// <summary>
    /// Checks with what orientation <paramref name="shape"/> (a wire or an edge) can be connected to the wire (see the
    /// other overload).
    /// </summary>
    public bool CheckShapeConnect(in TopoDS_Shape shape, double prec = 0.0)
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
                                  in TopoDS_Shape shape, double prec = 0.0)
    {
        // OCCT.NET: OCCT leaves the distances unchanged when it returns FAIL1; they are out here, set to 0
        tailhead = tailtail = headtail = headhead = 0.0;
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
        if (!IsLoaded() || shape.IsNull())
        {
            return false;
        }

        TopoDS_Vertex      V1, V2;
        TopoDS_Edge        E;
        TopoDS_Wire        W;
        ShapeAnalysis_Edge SAE = new ShapeAnalysis_Edge();
        if (shape.ShapeType() == TopAbs_EDGE)
        {
            E  = TopoDS.Edge(shape);
            V1 = SAE.FirstVertex(E);
            V2 = SAE.LastVertex(E);
        }
        else if (shape.ShapeType() == TopAbs_WIRE)
        {
            W = TopoDS.Wire(shape);
            ShapeAnalysis.FindBounds(W, out V1, out V2);
        }
        else
        {
            return false;
        }

        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
        //  on va comparer les points avec ceux de thevfirst et thevlast
        gp_Pnt p1 = BRep_Tool.Pnt(V1);
        gp_Pnt p2 = BRep_Tool.Pnt(V2);

        TopoDS_Vertex vfirst = SAE.FirstVertex(myWire!.Edge(1)),
                      vlast  = SAE.LastVertex(myWire.Edge(NbEdges()));
        gp_Pnt pf = BRep_Tool.Pnt(vfirst);
        gp_Pnt pl = BRep_Tool.Pnt(vlast);

        tailhead   = p1.Distance(pl);
        tailtail   = p2.Distance(pl);
        headhead   = p1.Distance(pf);
        headtail   = p2.Distance(pf);
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
        myMin3d    = Min(dm1, dm2);
        myMax3d    = Max(dm1, dm2);
        if (dm1 > dm2)
        {
            dm1    = dm2;
            result = res2 + 2;
        }

        switch (result)
        {
            case 1:
                myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
                break;
            case 2:
                myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE3);
                break;
            case 3:
                myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE4);
                break;
        }

        if (res1 == 0)
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE5);
        }

        if (res2 == 0)
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE6);
        }

        if (myMin3d > Max(myPrecision, prec))
        {
            myStatus = ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
        }

        return LastCheckStatus(ShapeExtend_DONE);
    }

    private static bool isMultiVertex(NCollection_List<TopoDS_Shape> alshape, NCollection_Map<TopoDS_Shape> aMapSmallEdges,
                                      NCollection_Map<TopoDS_Shape> aMapSeemEdges)
    {
        NCollection_List<TopoDS_Shape>.Iterator lIt1 = new NCollection_List<TopoDS_Shape>.Iterator(alshape);
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
    public bool CheckLoop(NCollection_IndexedMap<TopoDS_Shape> aMapLoopVertices,
                          NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMapVertexEdges,
                          NCollection_Map<TopoDS_Shape> aMapSmallEdges, NCollection_Map<TopoDS_Shape> aMapSeemEdges)
    {
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsLoaded() || NbEdges() < 2)
        {
            return false;
        }

        double aSavPreci = Precision();
        SetPrecision(Occt.Precision.Infinite());
        int i = 1;

        for (; i <= myWire!.NbEdges(); i++)
        {
            TopoDS_Edge aedge = myWire.Edge(i);
            TopExp.Vertices(aedge, out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);
            if (aV1.IsNull() || aV2.IsNull())
            {
                // OCCT: the precision set to Infinite above isn't restored on this return
                myStatus = ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
                return false;
            }

            bool isSame = aV1.IsSame(aV2);
            if (myWire.IsSeam(i))
            {
                aMapSeemEdges.Add(aedge); // continue;
            }
            else if (BRep_Tool.Degenerated(aedge))
            {
                aMapSmallEdges.Add(aedge);
            }
            else if (isSame && CheckSmall(i, BRep_Tool.Tolerance(aV1)))
            {
                aMapSmallEdges.Add(aedge);
            }

            if (!aMapVertexEdges.IsBound(aV1))
            {
                NCollection_List<TopoDS_Shape> alshape = new NCollection_List<TopoDS_Shape>();
                aMapVertexEdges.Bind(aV1, alshape);
            }

            if (!aMapVertexEdges.IsBound(aV2))
            {
                NCollection_List<TopoDS_Shape> alshape = new NCollection_List<TopoDS_Shape>();
                aMapVertexEdges.Bind(aV2, alshape);
            }

            if (isSame)
            {
                NCollection_List<TopoDS_Shape> alshape = aMapVertexEdges.ChangeFind(aV1);
                alshape.Append(aedge);
                alshape.Append(aedge);
                if (alshape.Extent() > 2 && isMultiVertex(alshape, aMapSmallEdges, aMapSeemEdges))
                {
                    aMapLoopVertices.Add(aV1);
                }
            }
            else
            {
                NCollection_List<TopoDS_Shape> alshape = aMapVertexEdges.ChangeFind(aV1);
                alshape.Append(aedge);
                if (alshape.Extent() > 2 && isMultiVertex(alshape, aMapSmallEdges, aMapSeemEdges))
                {
                    aMapLoopVertices.Add(aV1);
                }

                NCollection_List<TopoDS_Shape> alshape2 = aMapVertexEdges.ChangeFind(aV2);
                alshape2.Append(aedge);
                if (alshape2.Extent() > 2 && isMultiVertex(alshape2, aMapSmallEdges, aMapSeemEdges))
                {
                    aMapLoopVertices.Add(aV2);
                }
            }
        }

        SetPrecision(aSavPreci);
        if (aMapLoopVertices.Extent() != 0)
        {
            myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
            myStatusLoop |= myStatus;
            return true;
        }

        return false;
    }

    private static double Project(Geom_Curve theCurve, double theFirstParameter, double theLastParameter,
                                  in gp_Pnt thePoint, double thePrecision, out double theParameter, out gp_Pnt theProjection)
    {
        double aDist = new ShapeAnalysis_Curve().Project(theCurve, thePoint, thePrecision, out theProjection,
                                                         out theParameter, theFirstParameter, theLastParameter);
        if (theParameter >= theFirstParameter && theParameter <= theLastParameter)
        {
            return aDist;
        }

        double p0 = theFirstParameter, p1 = theLastParameter;
        gp_Pnt aPrj0 = theCurve.Value(p0), aPrj1 = theCurve.Value(p1);
        double aDist0 = thePoint.Distance(aPrj0), aDist1 = thePoint.Distance(aPrj1);
        int    aPI    = (aDist0 <= aDist1) ? 0 : 1;
        theParameter  = aPI == 0 ? p0 : p1;
        theProjection = aPI == 0 ? aPrj0 : aPrj1;
        return aPI == 0 ? aDist0 : aDist1;
    }

    /// <summary>
    /// Checks if two adjacent edges form a tail (a thin spike) narrower than <paramref name="theMaxWidth"/>, at an angle
    /// whose sine is under <paramref name="theMaxSine"/> (no angle check when negative). When true, gives the parts of
    /// the edges to remove (<paramref name="theEdge11"/>, <paramref name="theEdge21"/>) and to keep
    /// (<paramref name="theEdge12"/>, <paramref name="theEdge22"/>): either the whole edges, or new edges cut from them.
    /// </summary>
    /// <remarks>
    /// The edge parts it cuts are new edges, published frozen (design 4.2); the outputs it doesn't set are left
    /// unchanged, as in OCCT.
    /// </remarks>
    public bool CheckTail(in TopoDS_Edge theEdge1, in TopoDS_Edge theEdge2, double theMaxSine, double theMaxWidth,
                          double theMaxTolerance, ref TopoDS_Edge theEdge11, ref TopoDS_Edge theEdge12,
                          ref TopoDS_Edge theEdge21, ref TopoDS_Edge theEdge22) =>
        checkTail(theEdge1, theEdge2, theMaxSine, theMaxWidth, theMaxTolerance, ref theEdge11, ref theEdge12, ref theEdge21,
                  ref theEdge22, true);

    /// <summary>
    /// <see cref="CheckTail"/> without freezing the edge parts it cuts, for ShapeFix_Wire.FixTails, which puts them
    /// into the wire it goes on fixing (the parts share the vertices of its working edges).
    /// </summary>
    internal bool InternalCheckTail(in TopoDS_Edge theEdge1, in TopoDS_Edge theEdge2, double theMaxSine, double theMaxWidth,
                                    double theMaxTolerance, ref TopoDS_Edge theEdge11, ref TopoDS_Edge theEdge12,
                                    ref TopoDS_Edge theEdge21, ref TopoDS_Edge theEdge22) =>
        checkTail(theEdge1, theEdge2, theMaxSine, theMaxWidth, theMaxTolerance, ref theEdge11, ref theEdge12, ref theEdge21,
                  ref theEdge22, false);

    private bool checkTail(in TopoDS_Edge theEdge1, in TopoDS_Edge theEdge2, double theMaxSine, double theMaxWidth,
                           double theMaxTolerance, ref TopoDS_Edge theEdge11, ref TopoDS_Edge theEdge12,
                           ref TopoDS_Edge theEdge21, ref TopoDS_Edge theEdge22, bool theToFreeze)
    {
        TopoDS_Edge[] aEs = [theEdge1, theEdge2];
        if (!IsReady() || BRep_Tool.Degenerated(aEs[0]) || BRep_Tool.Degenerated(aEs[1]))
        {
            return false;
        }

        // Check the distance between the edge common ends.
        double       aTol2   = theMaxWidth + 0.5 * Occt.Precision.Confusion();
        double       aTol3   = theMaxWidth + Occt.Precision.Confusion();
        double       aTol4   = theMaxWidth + 1.5 * Occt.Precision.Confusion();
        double       aSqTol2 = aTol2 * aTol2;
        double       aSqTol3 = aTol3 * aTol3;
        Geom_Curve?[] aCs    = new Geom_Curve?[2];
        double[,]    aLs     = new double[2, 2];
        int[]        aVIs    = new int[2];
        gp_Pnt[]     aVPs    = [new gp_Pnt(), new gp_Pnt()];
        {
            for (int aEI = 0; aEI < 2; ++aEI)
            {
                if (!new ShapeAnalysis_Edge().Curve3d(aEs[aEI], out aCs[aEI], out aLs[aEI, 0], out aLs[aEI, 1], false))
                {
                    return false;
                }

                aVIs[aEI] = (aEs[aEI].Orientation() == TopAbs_REVERSED) ? aEI : 1 - aEI;
                aVPs[aEI] = aCs[aEI]!.Value(aLs[aEI, aVIs[aEI]]);
            }

            if (aVPs[0].SquareDistance(aVPs[1]) > aSqTol2)
            {
                return false;
            }
        }

        // Check the angle between the edges.
        if (theMaxSine >= 0)
        {
            double   aSqMaxSine = theMaxSine * theMaxSine;
            gp_XYZ[] aDs        = [new gp_XYZ(), new gp_XYZ()];
            int      aReverse   = 0;
            for (int aEI = 0; aEI < 2; ++aEI)
            {
                GeomAdaptor_Curve aCA = new GeomAdaptor_Curve(aCs[aEI]!);
                if (GCPnts_AbscissaPoint.Length(aCA, aLs[aEI, 0], aLs[aEI, 1], 0.25 * Occt.Precision.Confusion())
                    < 0.5 * Occt.Precision.Confusion())
                {
                    return false;
                }

                GCPnts_AbscissaPoint aAP = new GCPnts_AbscissaPoint(0.25 * Occt.Precision.Confusion(),
                                                                    aCA,
                                                                    0.5 * Occt.Precision.Confusion() * (1 - 2 * aVIs[aEI]),
                                                                    aLs[aEI, aVIs[aEI]]);
                if (!aAP.IsDone())
                {
                    return false;
                }

                gp_XYZ[] aPs = [new gp_XYZ(), new gp_XYZ()];
                aPs[aVIs[aEI]]     = aVPs[aEI].XYZ();
                aPs[1 - aVIs[aEI]] = aCs[aEI]!.Value(aAP.Parameter()).XYZ();
                aDs[aEI]           = aPs[1] - aPs[0];
                double aDN         = aDs[aEI].Modulus();
                if (aDN < 0.1 * Occt.Precision.Confusion())
                {
                    return false;
                }

                aDs[aEI] *= 1 / aDN;
                aReverse ^= aVIs[aEI];
            }

            if (aReverse != 0)
            {
                aDs[0].Reverse();
            }

            if (aDs[0] * aDs[1] < 0 || aDs[0].CrossSquareMagnitude(aDs[1]) > aSqMaxSine)
            {
                return false;
            }
        }

        // Calculate the tail bounds.
        gp_Pnt[] aPs2     = [new gp_Pnt(), new gp_Pnt()];
        gp_Pnt[] aPrjs    = [new gp_Pnt(), new gp_Pnt()];
        double[] aParams1 = new double[2];
        double[] aParams2 = new double[2];
        double[] aDists   = new double[2];
        bool[]   isWholes = [true, true];
        for (int aEI = 0; aEI < 2; ++aEI)
        {
            double aParam1 = aLs[aEI, aVIs[aEI]];
            aParams1[aEI]  = aLs[aEI, 1 - aVIs[aEI]];
            aCs[aEI]!.D0(aParams1[aEI], out aPs2[aEI]);
            aDists[aEI] = Project(aCs[1 - aEI]!,
                                  aLs[1 - aEI, 0],
                                  aLs[1 - aEI, 1],
                                  aPs2[aEI],
                                  0.25 * Occt.Precision.Confusion(),
                                  out aParams2[aEI],
                                  out aPrjs[aEI]);
            if (aDists[aEI] <= aTol2)
            {
                continue;
            }

            isWholes[aEI] = false;
            for (;;)
            {
                double aParam = (aParam1 + aParams1[aEI]) * 0.5;
                aCs[aEI]!.D0(aParam, out aPs2[aEI]);
                double aDist = Project(aCs[1 - aEI]!,
                                       aLs[1 - aEI, 0],
                                       aLs[1 - aEI, 1],
                                       aPs2[aEI],
                                       0.25 * Occt.Precision.Confusion(),
                                       out aParams2[aEI],
                                       out aPrjs[aEI]);
                if (aDist <= aTol2)
                {
                    aParam1 = aParam;
                }
                else
                {
                    aParams1[aEI] = aParam;
                    if (aDist <= aTol3)
                    {
                        break;
                    }
                }
            }
        }

        // Check the tail bounds.
        for (int aEI = 0; aEI < 2; ++aEI)
        {
            double aParam1 = aLs[aEI, aVIs[aEI]];
            double aParam2 = aParams1[aEI];
            double aStepL  = (aParam2 - aParam1) / 23;
            for (int aStepN = 1; aStepN < 23; ++aStepN)
            {
                double aParam = aParam1 + aStepN * aStepL;
                gp_Pnt aP     = aCs[aEI]!.Value(aParam);
                if (Project(aCs[1 - aEI]!,
                            aLs[1 - aEI, 0],
                            aLs[1 - aEI, 1],
                            aP,
                            0.25 * Occt.Precision.Confusion(),
                            out aParam,
                            out _)
                    > aTol4)
                {
                    return false;
                }
            }
        }

        // Check whether both edges must be removed.
        if (isWholes[0] && isWholes[1] && aPs2[0].SquareDistance(aPs2[1]) <= aSqTol3)
        {
            theEdge11 = theEdge1;
            theEdge21 = theEdge2;
            return true;
        }

        // Cut and remove the edges.
        int aFI = 0;
        if (isWholes[0] || isWholes[1])
        {
            // Determine an edge to remove and the other one to cut.
            aFI = isWholes[0] ? 0 : 1;
            if (aDists[1 - aFI] < aDists[aFI] && isWholes[1 - aFI])
            {
                aFI = 1 - aFI;
            }
        }

        double[] aParams = new double[2];
        aParams[aFI]     = aParams1[aFI];
        aParams[1 - aFI] = aParams2[aFI];
        // Correct the cut for the parametrization tolerance.
        // OCCT's pointers to the four outputs: a local table copied back at the end
        TopoDS_Edge[,] aEParts  = { { theEdge11, theEdge12 }, { theEdge21, theEdge22 } };
        int[]          aResults = [1, 1];
        for (int aEI = 0; aEI < 2; ++aEI)
        {
            if (Math.Abs(aParams[aEI] - aLs[aEI, 1 - aVIs[aEI]]) <= Occt.Precision.PConfusion())
            {
                aResults[aEI]     = 2;
                aEParts[aEI, 0] = aEs[aEI];
            }
            else if (Math.Abs(aParams[aEI] - aLs[aEI, aVIs[aEI]]) <= Occt.Precision.PConfusion())
            {
                aResults[aEI] = 0;
            }
        }

        // Correct the cut for the distance tolerance.
        for (int aEI = 0; aEI < 2; ++aEI)
        {
            if (aResults[aEI] != 1)
            {
                continue;
            }

            // Create the parts of the edge.
            TopoDS_Edge                          aFE   = TopoDS.Edge(aEs[aEI].Oriented(TopAbs_FORWARD));
            ShapeAnalysis_TransferParametersProj aSATPP = new ShapeAnalysis_TransferParametersProj(aFE, new TopoDS_Face());
            aSATPP.SetMaxTolerance(theMaxTolerance);
            TopoDS_Vertex aSplitV = new TopoDS_Vertex();
            new BRep_Builder().MakeVertex(ref aSplitV, aCs[aEI]!.Value(aParams[aEI]), Occt.Precision.Confusion());
            TopoDS_Edge[] aEParts2 =
            [
                new ShapeBuild_Edge().InternalCopyReplaceVertices(aFE, new TopoDS_Vertex(), TopoDS.Vertex(aSplitV.Oriented(TopAbs_REVERSED))),
                new ShapeBuild_Edge().InternalCopyReplaceVertices(aFE, aSplitV, new TopoDS_Vertex()),
            ];
            new ShapeBuild_Edge().CopyPCurves(aEParts2[0], aFE);
            new ShapeBuild_Edge().CopyPCurves(aEParts2[1], aFE);
            new BRep_Builder().SameRange(aEParts2[0], false);
            new BRep_Builder().SameRange(aEParts2[1], false);
            new BRep_Builder().SameParameter(aEParts2[0], false);
            new BRep_Builder().SameParameter(aEParts2[1], false);
            aSATPP.TransferRange(aEParts2[0], aLs[aEI, 0], aParams[aEI], false);
            aSATPP.TransferRange(aEParts2[1], aParams[aEI], aLs[aEI, 1], false);
            // OCCT.NET: the new parts are published frozen (design 4.2)
            if (theToFreeze)
            {
                aEParts2[0].TShape()!.FreezeGraph();
                aEParts2[1].TShape()!.FreezeGraph();
            }
            GProp_GProps aLinProps = new GProp_GProps();
            BRepGProp.LinearProperties(aEParts2[1 - aVIs[aEI]], aLinProps);
            if (aLinProps.Mass() <= Occt.Precision.Confusion())
            {
                aResults[aEI]     = 2;
                aEParts[aEI, 0] = aEs[aEI];
            }
            else
            {
                BRepGProp.LinearProperties(aEParts2[aVIs[aEI]], aLinProps);
                if (aLinProps.Mass() <= Occt.Precision.Confusion())
                {
                    aResults[aEI] = 0;
                }
                else
                {
                    aEParts[aEI, 0] = aEParts2[0];
                    aEParts[aEI, 1] = aEParts2[1];
                }
            }
        }

        theEdge11 = aEParts[0, 0];
        theEdge12 = aEParts[0, 1];
        theEdge21 = aEParts[1, 0];
        theEdge22 = aEParts[1, 1];
        return aResults[0] + aResults[1] != 0;
    }
}
