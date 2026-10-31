// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_BuildTool.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_BuildTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopOpeBRepTool_OutCurveType;

namespace Occt;

/// <summary>
/// The legacy Booleans' tool that builds the topology of their results: new vertices, edges, wires, faces and shells,
/// and their geometry from the data structure.
/// </summary>
/// <remarks>
/// <para>
/// Only what the face maker of the 2D offset reaches is ported (plan 2.11, decision 585): the constructors, the
/// overwrite and translate flags, <see cref="MakeWire"/>, <see cref="CopyFace"/>, <see cref="AddWireEdge"/>,
/// <see cref="AddFaceWire"/>, <see cref="Closed"/> and <see cref="Parameter"/>. The rest (making vertices, edges, faces
/// and shells from the data structure's points, curves and surfaces, pcurve computation, approximation) works on
/// TopOpeBRepDS's data structure, not ported, and is left out.
/// </para>
/// <para>
/// A working object, copied by value in OCCT (<see cref="TopOpeBRepBuild_Builder"/> keeps a copy): the port copies it
/// with the copy constructor. Its methods write into the shapes they are given, as in OCCT; a frozen shape throws.
/// </para>
/// </remarks>
public sealed class TopOpeBRepDS_BuildTool
{
    private readonly BRep_Builder myBuilder = new();
    private readonly TopOpeBRepTool_CurveTool myCurveTool;
    private bool myOverWrite;
    private bool myTranslate;

    /// <summary>A build tool that approximates walking lines.</summary>
    public TopOpeBRepDS_BuildTool()
    {
        myCurveTool = new TopOpeBRepTool_CurveTool(TopOpeBRepTool_APPROX);
        myOverWrite = true;
        myTranslate = true;
    }

    /// <summary>A build tool computing 3D curves of kind <paramref name="O"/>.</summary>
    public TopOpeBRepDS_BuildTool(TopOpeBRepTool_OutCurveType O)
    {
        myCurveTool = new TopOpeBRepTool_CurveTool(O);
        myOverWrite = true;
        myTranslate = true;
    }

    /// <summary>A build tool with the curve settings <paramref name="GT"/>.</summary>
    public TopOpeBRepDS_BuildTool(in TopOpeBRepTool_GeomTool GT)
    {
        myCurveTool = new TopOpeBRepTool_CurveTool(GT);
        myOverWrite = true;
        myTranslate = true;
    }

    /// <summary>OCCT.NET: a copy of <paramref name="theOther"/> (OCCT copies the class by value).</summary>
    public TopOpeBRepDS_BuildTool(TopOpeBRepDS_BuildTool theOther)
    {
        myCurveTool = new TopOpeBRepTool_CurveTool(theOther.myCurveTool);
        myOverWrite = theOther.myOverWrite;
        myTranslate = theOther.myTranslate;
    }

    /// <summary>Whether existing geometry is overwritten.</summary>
    public bool OverWrite() => myOverWrite;

    internal void OverWrite(bool O)
    {
        myOverWrite = O;
    }

    /// <summary>Whether pcurves are translated on periodic surfaces.</summary>
    public bool Translate() => myTranslate;

    internal void Translate(bool T)
    {
        myTranslate = T;
    }

    /// <summary>The curve settings.</summary>
    public TopOpeBRepTool_GeomTool GetGeomTool() => myCurveTool.GetGeomTool();

    internal ref TopOpeBRepTool_GeomTool ChangeGeomTool() => ref myCurveTool.ChangeGeomTool();

    // MakeVertex, the MakeEdge forms, MakeFace, MakeShell, MakeSolid, CopyEdge, GetOrientedEdgeVertices,
    // UpdateEdgeCurveTol, ApproxCurves, ComputePCurves, PutPCurves, RecomputeCurves, AddEdgeVertex, AddShellFace,
    // AddSolidShell, Range, UpdateEdge, Parameter on a curve, Curve3D, PCurve, Orientation, Approximation,
    // UpdateSurface and TranslateOnPeriodic: not ported (TopOpeBRepDS's data structure; plan 2.11).

    /// <summary>Makes an empty wire in <paramref name="W"/>.</summary>
    internal void MakeWire(ref TopoDS_Shape W)
    {
        TopoDS_Wire aW = new();
        myBuilder.MakeWire(ref aW);
        W = aW;
    }

    /// <summary>An empty copy of the face <paramref name="Fin"/> (same surface, tolerance and orientation, no wires).</summary>
    internal void CopyFace(in TopoDS_Shape Fin, out TopoDS_Shape Fou)
    {
        Fou = Fin.EmptyCopied();
    }

    /// <summary>Adds the edge <paramref name="E"/> to the wire <paramref name="W"/>.</summary>
    internal void AddWireEdge(in TopoDS_Shape W, in TopoDS_Shape E)
    {
        myBuilder.Add(W, E);
    }

    /// <summary>Adds the wire <paramref name="W"/> to the face <paramref name="F"/>.</summary>
    internal void AddFaceWire(in TopoDS_Shape F, in TopoDS_Shape W)
    {
        myBuilder.Add(F, W);
    }

    /// <summary>
    /// Sets the parameter of the vertex <paramref name="V"/> on the edge <paramref name="E"/> to <paramref name="P"/>,
    /// brought into the period after the edge's first parameter for a REVERSED vertex on a periodic curve.
    /// </summary>
    internal void Parameter(in TopoDS_Shape E, in TopoDS_Shape V, double P)
    {
        TopoDS_Edge e = TopoDS.Edge(E);
        TopoDS_Vertex v = TopoDS.Vertex(V);
        double p = P;

        // 13/07/95 :
        Geom_Curve? C = BRep_Tool.Curve(e, out TopLoc_Location _, out double f, out double _);
        if (C is not null && C.IsPeriodic())
        {
            double per = C.Period();

            TopAbs_Orientation oV = TopAbs_FORWARD;

            TopExp_Explorer exV = new(e, TopAbs_VERTEX);
            for (; exV.More(); exV.Next())
            {
                TopoDS_Vertex vofe = TopoDS.Vertex(exV.Current());
                if (vofe.IsSame(v))
                {
                    oV = vofe.Orientation();
                    break;
                }
            }
            if (exV.More())
            {
                if (oV == TopAbs_REVERSED)
                {
                    if (p < f)
                    {
                        double pp = ElCLib.InPeriod(p, f, f + per);
                        p = pp;
                    }
                }
            }
        }
        myBuilder.UpdateVertex(v, p, e,
                               0); // NYI : Tol on new vertex ??
    }

    /// <summary>Sets the Closed flag of <paramref name="S"/>.</summary>
    internal void Closed(in TopoDS_Shape S, bool B)
    {
        S.Closed(B);
    }
}
