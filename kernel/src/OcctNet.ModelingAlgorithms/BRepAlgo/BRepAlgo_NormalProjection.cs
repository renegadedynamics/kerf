// Port of: src/ModelingAlgorithms/TKBool/BRepAlgo/BRepAlgo_NormalProjection.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepAlgo/BRepAlgo_NormalProjection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Schematiq.Collections;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace Occt;

// OCCT_DEBUG and OCCT_DEBUG_CHRONO blocks (traces and chronometers) are left out.

/// <summary>
/// The normal projection of edges and wires on a shape: each edge projected along the normals of each face, as edges
/// with their pcurves on the faces, trimmed by the faces' bounds; punctual projections are vertices.
/// </summary>
/// <remarks>
/// A working object (decision 21). It only reads the shapes it is given: every edge of the result is new (or a
/// section's). <see cref="Projection"/> and <see cref="Generated"/> are publication points and freeze what they
/// return; <see cref="InternalProjection"/> and <see cref="InternalGenerated"/> are the unfrozen forms for ported code.
/// </remarks>
public sealed class BRepAlgo_NormalProjection
{
    private TopoDS_Shape myShape = new();
    private bool myIsDone;
    private double myTol3d;
    private double myTol2d;
    private double myMaxDist;
    private bool myWith3d;
    private Continuity myContinuity;
    private int myMaxDegree;
    private int myMaxSeg;
    private bool myFaceBounds;
    private readonly TopoDS_Shape myToProj;
    private readonly NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> myAncestorMap = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> myCorresp = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myDescendants =
        new(TopTools_ShapeMapHasher.Instance);
    private TopoDS_Shape myRes = new();

    //=================================================================================================

    /// <summary>An empty algorithm; <see cref="Init"/> gives the shape to project on.</summary>
    public BRepAlgo_NormalProjection()
    {
        myIsDone = false;
        myMaxDist = -1.0;
        myWith3d = true;
        myFaceBounds = true;
        BRep_Builder BB = new();
        TopoDS_Compound aToProj = new();
        BB.MakeCompound(ref aToProj);
        myToProj = aToProj;
        SetDefaultParams();
    }

    //=================================================================================================

    /// <summary>The projection on the shape <paramref name="S"/>.</summary>
    public BRepAlgo_NormalProjection(in TopoDS_Shape S) : this()
    {
        Init(S);
    }

    //=================================================================================================

    /// <summary>Sets the shape to project on.</summary>
    public void Init(in TopoDS_Shape S)
    {
        myShape = S;
    }

    //=================================================================================================

    /// <summary>Adds an edge or a wire to project.</summary>
    public void Add(in TopoDS_Shape ToProj)
    {
        BRep_Builder BB = new();
        BB.Add(myToProj, ToProj);
    }

    //=================================================================================================

    /// <summary>
    /// Sets the parameters: the tolerances between the projected 3D curve and its 2D curve, the continuity of the
    /// approximation, its maximum degree and number of segments.
    /// </summary>
    public void SetParams(double Tol3D, double Tol2D, Continuity InternalContinuity, int MaxDegree, int MaxSeg)
    {
        myTol3d = Tol3D;
        myTol2d = Tol2D;
        myContinuity = InternalContinuity;
        myMaxDegree = MaxDegree;
        myMaxSeg = MaxSeg;
    }

    //=================================================================================================

    /// <summary>Sets the default parameters (1e-4, its power 2/3, C1, 14, 16).</summary>
    public void SetDefaultParams()
    {
        myTol3d = 1.0e-4;
        myTol2d = Math.Pow(myTol3d, 2.0 / 3);
        myContinuity = C1;
        myMaxDegree = 14;
        myMaxSeg = 16;
    }

    //=================================================================================================

    /// <summary>Whether the projected edges are trimmed by the faces' bounds (true by default).</summary>
    public void SetLimit(bool FaceBounds = true)
    {
        myFaceBounds = FaceBounds;
    }

    //=================================================================================================

    /// <summary>The maximum distance between the shape and what is projected (none if negative, the default).</summary>
    public void SetMaxDistance(double MaxDist)
    {
        myMaxDist = MaxDist;
    }

    //=================================================================================================

    /// <summary>False keeps the initial 3D curves for the projected edges where the projection is close enough.</summary>
    public void Compute3d(bool With3d = true)
    {
        myWith3d = With3d;
    }

    //=================================================================================================

    /// <summary>Builds the result as a compound.</summary>
    public void Build()
    {
        myIsDone = false;

        NCollection_HSequence<TopoDS_Shape> Edges = new();
        NCollection_HSequence<TopoDS_Shape> Faces = new();
        NCollection_List<TopoDS_Shape> DescenList = new();
        int NbEdges = 0, NbFaces = 0, i, j, k;
        TopExp_Explorer ExpOfWire = new(), ExpOfShape = new();
        double Udeb, Ufin;
        TopoDS_Shape VertexRes;
        bool Only3d, Only2d, Elementary;

        // for isoparametric cases
        NCollection_Array1<Point2D> Poles = new(1, 2);
        NCollection_Array1<double> Knots = new(1, 2);
        NCollection_Array1<int> Mults = new(1, 2);
        int Deg;
        Deg = 1;
        Mults[1] = Deg + 1;
        Mults[2] = Deg + 1;
        //

        for (ExpOfWire.Init(myToProj, TopAbs_EDGE); ExpOfWire.More(); ExpOfWire.Next(), NbEdges++)
        {
            Edges.Append(ExpOfWire.Current());
        }

        for (ExpOfShape.Init(myShape, TopAbs_FACE); ExpOfShape.More(); ExpOfShape.Next(), NbFaces++)
        {
            Faces.Append(ExpOfShape.Current());
        }

        BRep_Builder BB = new();
        TopoDS_Compound aRes = new(), aVertexRes = new();
        BB.MakeCompound(ref aRes);
        BB.MakeCompound(ref aVertexRes);
        myRes = aRes;
        VertexRes = aVertexRes;
        bool YaVertexRes = false;

        for (i = 1; i <= NbEdges; i++)
        {
            DescenList.Clear();
            BRepAdaptor_Curve hcur = new(TopoDS.Edge(Edges.Value(i)));
            Elementary = IsElementary(hcur);
            for (j = 1; j <= NbFaces; j++)
            {
                BRepAdaptor_Surface hsur = new(TopoDS.Face(Faces.Value(j)));

                // computation of  TolU and TolV

                double TolU, TolV;

                TolU = hsur.UResolution(myTol3d) / 20;
                TolV = hsur.VResolution(myTol3d) / 20;
                // Projection
                ProjLib_CompProjectedCurve HProjector = new(hsur, hcur, TolU, TolV, myMaxDist);
                //
                TopoDS_Shape prj = new();
                bool Degenerated = false;
                Point3D P;

                Adaptor2d_Curve2d HPCur;
                Geom2d_Curve? PCur2d = null; // Only for isoparametric projection

                for (k = 1; k <= HProjector.NbCurves(); k++)
                {
                    if (HProjector.IsSinglePnt(k, out Point2D P2d))
                    {
                        HProjector.GetSurface()!.D0(P2d.X(), P2d.Y(), out P);
                        prj = new BRepLib_MakeVertex(P).InternalShape();
                        DescenList.Append(prj);
                        BB.Add(VertexRes, prj);
                        YaVertexRes = true;

                        myAncestorMap.Bind(prj, Edges.Value(i));
                    }
                    else
                    {
                        Only2d = Only3d = false;
                        HProjector.Bounds(k, out Udeb, out Ufin);

                        /**************************************************************/
                        if (HProjector.IsUIso(k, out double UIso))
                        {
                            HProjector.D0(Udeb, out Point2D Pdeb);
                            HProjector.D0(Ufin, out Point2D Pfin);
                            Poles[1] = Pdeb;
                            Poles[2] = Pfin;
                            Knots[1] = Udeb;
                            Knots[2] = Ufin;
                            Geom2d_BSplineCurve BS2d = new(Poles, Knots, Mults, Deg);
                            PCur2d = new Geom2d_TrimmedCurve(BS2d, Udeb, Ufin);
                            HPCur = new Geom2dAdaptor_Curve(PCur2d);
                            Only3d = true;
                        }
                        else if (HProjector.IsVIso(k, out double VIso))
                        {
                            HProjector.D0(Udeb, out Point2D Pdeb);
                            HProjector.D0(Ufin, out Point2D Pfin);
                            Poles[1] = Pdeb;
                            Poles[2] = Pfin;
                            Knots[1] = Udeb;
                            Knots[2] = Ufin;
                            Geom2d_BSplineCurve BS2d = new(Poles, Knots, Mults, Deg);
                            PCur2d = new Geom2d_TrimmedCurve(BS2d, Udeb, Ufin);
                            HPCur = new Geom2dAdaptor_Curve(PCur2d);
                            Only3d = true;
                        }
                        else
                        {
                            HPCur = HProjector;
                        }

                        if ((!myWith3d || Elementary) && (HProjector.MaxDistance(k) <= myTol3d))
                        {
                            Only2d = true;
                        }

                        if (Only2d && Only3d)
                        {
                            BRepLib_MakeEdge MKed = new(GeomAdaptor.MakeCurve(hcur), Udeb, Ufin);
                            prj = MKed.InternalEdge();
                            BB.UpdateEdge(TopoDS.Edge(prj), PCur2d, TopoDS.Face(Faces.Value(j)), myTol3d);
                            BB.UpdateVertex(TopExp.FirstVertex(TopoDS.Edge(prj)), myTol3d);
                            BB.UpdateVertex(TopExp.LastVertex(TopoDS.Edge(prj)), myTol3d);
                        }
                        else
                        {
                            Approx_CurveOnSurface appr = new(HPCur, hsur, Udeb, Ufin, myTol3d);
                            appr.Perform(myMaxSeg, myMaxDegree, myContinuity, Only3d, Only2d);

                            if (appr.MaxError3d() > 1.0e3 * myTol3d)
                            {
                                continue;
                            }

                            if (!Only3d)
                            {
                                PCur2d = appr.Curve2d();
                            }
                            if (Only2d)
                            {
                                BRepLib_MakeEdge MKed = new(GeomAdaptor.MakeCurve(hcur), Udeb, Ufin);
                                prj = MKed.InternalEdge();
                            }
                            else
                            {
                                // It is tested if the solution is not degenerated to set the
                                // flag on edge, one takes several points, checks if the cloud of
                                // points has less diameter than the tolerance 3D
                                Degenerated = true;
                                double Dist;
                                Geom_BSplineCurve BS3d = appr.Curve3d()!;
                                Point3D P1 = new(0.0, 0.0, 0.0), PP; // skl : I change "P" to "PP"
                                int NbPoint, ii; // skl : I change "i" to "ii"
                                double Par, DPar;
                                // start from 3 points to reject non degenerated edges
                                // very fast
                                NbPoint = 3;
                                DPar = (BS3d.LastParameter() - BS3d.FirstParameter()) / (NbPoint - 1);
                                for (ii = 0; ii < NbPoint; ii++)
                                {
                                    Par = BS3d.FirstParameter() + ii * DPar;
                                    PP = BS3d.Value(Par);
                                    P1.SetXYZ(P1.XYZ() + PP.XYZ() / NbPoint);
                                }
                                for (ii = 0; ii < NbPoint && Degenerated; ii++)
                                {
                                    Par = BS3d.FirstParameter() + ii * DPar;
                                    PP = BS3d.Value(Par);
                                    Dist = P1.Distance(PP);
                                    if (Dist > myTol3d)
                                    {
                                        Degenerated = false;
                                        break;
                                    }
                                }
                                // if the test passes a more exact test with 10 points
                                if (Degenerated)
                                {
                                    P1.SetCoord(0.0, 0.0, 0.0);
                                    NbPoint = 10;
                                    DPar = (BS3d.LastParameter() - BS3d.FirstParameter()) / (NbPoint - 1);
                                    for (ii = 0; ii < NbPoint; ii++)
                                    {
                                        Par = BS3d.FirstParameter() + ii * DPar;
                                        PP = BS3d.Value(Par);
                                        P1.SetXYZ(P1.XYZ() + PP.XYZ() / NbPoint);
                                    }
                                    for (ii = 0; ii < NbPoint && Degenerated; ii++)
                                    {
                                        Par = BS3d.FirstParameter() + ii * DPar;
                                        PP = BS3d.Value(Par);
                                        Dist = P1.Distance(PP);
                                        if (Dist > myTol3d)
                                        {
                                            Degenerated = false;
                                            break;
                                        }
                                    }
                                }
                                if (Degenerated)
                                {
                                    TopoDS_Vertex VV = new();
                                    BB.MakeVertex(ref VV);
                                    BB.UpdateVertex(VV, P1, myTol3d);
                                    TopoDS_Edge aPrj = new();
                                    BB.MakeEdge(ref aPrj);
                                    BB.Add(aPrj, VV.Oriented(TopAbs_FORWARD));
                                    BB.Add(aPrj, VV.Oriented(TopAbs_REVERSED));
                                    BB.Degenerated(aPrj, true);
                                    prj = aPrj;
                                }
                                else
                                {
                                    prj = new BRepLib_MakeEdge(BS3d).InternalEdge();
                                }
                            }

                            BB.UpdateEdge(TopoDS.Edge(prj), PCur2d, TopoDS.Face(Faces.Value(j)), appr.MaxError3d());
                            BB.UpdateVertex(TopExp.FirstVertex(TopoDS.Edge(prj)), appr.MaxError3d());
                            BB.UpdateVertex(TopExp.LastVertex(TopoDS.Edge(prj)), appr.MaxError3d());
                            if (Degenerated)
                            {
                                BB.Range(TopoDS.Edge(prj), TopoDS.Face(Faces.Value(j)), Udeb, Ufin);
                            }
                        }

                        if (myFaceBounds)
                        {
                            // Trimming edges by face bounds
                            // if the solution is degenerated, use of BoolTool is avoided
                            if (!Degenerated)
                            {
                                // Perform Boolean COMMON operation to get parts of projected edge
                                // inside the face
                                BRepAlgoAPI_Section aSection = new(Faces.Value(j), prj);
                                if (aSection.IsDone())
                                {
                                    TopoDS_Shape aRC = aSection.InternalShape();
                                    //
                                    TopExp_Explorer aExpE = new(aRC, TopAbs_EDGE);
                                    for (; aExpE.More(); aExpE.Next())
                                    {
                                        TopoDS_Shape aE = aExpE.Current();
                                        BB.Add(myRes, aE);
                                        myAncestorMap.Bind(aE, Edges.Value(i));
                                        myCorresp.Bind(aE, Faces.Value(j));
                                    }
                                }
                                else
                                {
                                    // if the common operation has failed, try to classify the part
                                    BRepTopAdaptor_FClass2d classifier = new(TopoDS.Face(Faces.Value(j)), global::Kerf.Engine.Foundation.Precision.Confusion());
                                    double f = PCur2d!.FirstParameter();
                                    double l = PCur2d.LastParameter();
                                    double pmil = (f + l) / 2;
                                    PCur2d.D0(pmil, out Point2D Puv);
                                    TopAbs_State state;
                                    state = classifier.Perform(Puv);
                                    if (state == TopAbs_IN || state == TopAbs_ON)
                                    {
                                        BB.Add(myRes, prj);
                                        DescenList.Append(prj);
                                        myAncestorMap.Bind(prj, Edges.Value(i));
                                        myCorresp.Bind(prj, Faces.Value(j));
                                    }
                                }
                            }
                            else
                            {
                                BRepTopAdaptor_FClass2d classifier = new(TopoDS.Face(Faces.Value(j)), global::Kerf.Engine.Foundation.Precision.Confusion());
                                double f = PCur2d!.FirstParameter();
                                double l = PCur2d.LastParameter();
                                double pmil = (f + l) / 2;
                                PCur2d.D0(pmil, out Point2D Puv);
                                TopAbs_State state;
                                state = classifier.Perform(Puv);
                                if (state == TopAbs_IN || state == TopAbs_ON)
                                {
                                    BB.Add(myRes, prj);
                                    DescenList.Append(prj);
                                    myAncestorMap.Bind(prj, Edges.Value(i));
                                    myCorresp.Bind(prj, Faces.Value(j));
                                }
                            }
                        }
                        else
                        {
                            BB.Add(myRes, prj);
                            DescenList.Append(prj);
                            myAncestorMap.Bind(prj, Edges.Value(i));
                            myCorresp.Bind(prj, Faces.Value(j));
                        }
                    }
                }
            }
            // OCCT binds the list by value (a copy), then clears it for the next edge.
            myDescendants.Bind(Edges.Value(i), new NCollection_List<TopoDS_Shape>(DescenList));
        }
        // JPI : eventual wire creation is reported in a specific method
        //       BuilWire that can be called by the user. Otherwise, the
        //       relations of map myAncestorMap, myCorresp will be lost.

        if (YaVertexRes)
        {
            BB.Add(myRes, VertexRes);
        }

        myIsDone = true;
    }

    //=================================================================================================

    /// <summary>True if <see cref="Build"/> was run.</summary>
    public bool IsDone() => myIsDone;

    //=================================================================================================

    /// <summary>The result, a compound of edges (and a compound of vertices), frozen.</summary>
    public TopoDS_Shape Projection()
    {
        myRes.TShape()?.FreezeGraph();
        return myRes;
    }

    /// <summary><see cref="Projection"/> without freezing, for ported code.</summary>
    internal TopoDS_Shape InternalProjection() => myRes;

    //=================================================================================================

    /// <summary>The edge or wire's edge projected into the edge (or vertex) <paramref name="E"/> of the result.</summary>
    /// <exception cref="NoSuchObjectException">E isn't in the result.</exception>
    public TopoDS_Shape Ancestor(in TopoDS_Edge E) => myAncestorMap.Find(E);

    //=================================================================================================

    /// <summary>The face on which the edge <paramref name="E"/> of the result lies.</summary>
    /// <exception cref="NoSuchObjectException">E isn't in the result.</exception>
    public TopoDS_Shape Couple(in TopoDS_Edge E) => myCorresp.Find(E);

    //=================================================================================================

    /// <summary>The shapes of the result generated from the edge <paramref name="S"/>, frozen.</summary>
    /// <exception cref="NoSuchObjectException">S wasn't projected.</exception>
    public ISchematiqList<TopoDS_Shape> Generated(in TopoDS_Shape S)
    {
        NCollection_List<TopoDS_Shape> aList = InternalGenerated(S);
        var aShapes = new TopoDS_Shape[aList.Extent()];
        int i = 0;
        for (NCollection_List<TopoDS_Shape>.Iterator anIt = new(aList); anIt.More(); anIt.Next())
        {
            aShapes[i] = anIt.Value();
            aShapes[i].TShape()?.FreezeGraph();
            i++;
        }
        return new PackedList<TopoDS_Shape>(aShapes);
    }

    /// <summary><see cref="Generated"/> without freezing, for ported code.</summary>
    internal NCollection_List<TopoDS_Shape> InternalGenerated(in TopoDS_Shape S) => myDescendants.Find(S);

    //=================================================================================================

    /// <summary>True if the curve is a line or a conic.</summary>
    public bool IsElementary(Adaptor3d_Curve C)
    {
        CurveKind type;
        type = C.GetType();
        switch (type)
        {
            case Line:
            case Circle:
            case Ellipse:
            case Hyperbola:
            case Parabola:
                return true;
            default:
                return false;
        }
    }

    //=================================================================================================

    /// <summary>
    /// Builds the result as a wire if its edges make one, appended to <paramref name="ListOfWire"/>; false if they
    /// make several wires or there is no edge.
    /// </summary>
    public bool BuildWire(NCollection_List<TopoDS_Shape> ListOfWire)
    {
        TopExp_Explorer ExpOfShape = new();
        bool IsWire = false;
        ExpOfShape.Init(myRes, TopAbs_EDGE);
        if (ExpOfShape.More())
        {
            NCollection_List<TopoDS_Shape> List = new();

            for (; ExpOfShape.More(); ExpOfShape.Next())
            {
                TopoDS_Shape CurE = ExpOfShape.Current();
                List.Append(CurE);
            }
            BRepLib_MakeWire MW = new();
            MW.Add(List);
            if (MW.IsDone())
            {
                TopoDS_Shape Wire = MW.InternalShape();
                // If the resulting wire contains the same edge as at the beginning OK
                // otherwise the result really consists of several wires.
                TopExp_Explorer exp2 = new(Wire, TopAbs_EDGE);
                int NbEdges = 0;
                for (; exp2.More(); exp2.Next())
                {
                    NbEdges++;
                }
                if (NbEdges == List.Extent())
                {
                    ListOfWire.Append(Wire);
                    IsWire = true;
                }
            }
        }
        return IsWire;
    }
}
