// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_BuildTool.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_BuildTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy.Build;
using Kerf.Engine.Booleans.Legacy.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Booleans.Legacy.Data;

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
/// A working object, copied by value in OCCT (<see cref="LegacyBuilder"/> keeps a copy): the port copies it
/// with the copy constructor. Its methods write into the shapes they are given, as in OCCT; a frozen shape throws.
/// </para>
/// </remarks>
internal sealed class DsBuildTool
{
    private readonly BrepBuilder _builder = new();
    private readonly ToolCurveTool _curveTool;
    private bool _overWrite;
    private bool _translate;

    /// <summary>A build tool that approximates walking lines.</summary>
    public DsBuildTool()
    {
        _curveTool = new ToolCurveTool(OutCurveType.Approx);
        _overWrite = true;
        _translate = true;
    }

    /// <summary>A build tool computing 3D curves of kind <paramref name="o"/>.</summary>
    public DsBuildTool(OutCurveType o)
    {
        _curveTool = new ToolCurveTool(o);
        _overWrite = true;
        _translate = true;
    }

    /// <summary>A build tool with the curve settings <paramref name="gt"/>.</summary>
    public DsBuildTool(in ToolGeomTool gt)
    {
        _curveTool = new ToolCurveTool(gt);
        _overWrite = true;
        _translate = true;
    }

    /// <summary>Kerf: a copy of <paramref name="other"/> (OCCT copies the class by value).</summary>
    public DsBuildTool(DsBuildTool other)
    {
        _curveTool = new ToolCurveTool(other._curveTool);
        _overWrite = other._overWrite;
        _translate = other._translate;
    }

    /// <summary>Whether existing geometry is overwritten.</summary>
    public bool OverWrite() => _overWrite;

    internal void OverWrite(bool o)
    {
        _overWrite = o;
    }

    /// <summary>Whether pcurves are translated on periodic surfaces.</summary>
    public bool Translate() => _translate;

    internal void Translate(bool t)
    {
        _translate = t;
    }

    /// <summary>The curve settings.</summary>
    public ToolGeomTool GetGeomTool() => _curveTool.GetGeomTool();

    internal ref ToolGeomTool ChangeGeomTool() => ref _curveTool.ChangeGeomTool();

    // MakeVertex, the MakeEdge forms, MakeFace, MakeShell, MakeSolid, CopyEdge, GetOrientedEdgeVertices,
    // UpdateEdgeCurveTol, ApproxCurves, ComputePCurves, PutPCurves, RecomputeCurves, AddEdgeVertex, AddShellFace,
    // AddSolidShell, Range, UpdateEdge, Parameter on a curve, Curve3D, PCurve, Orientation, Approximation,
    // UpdateSurface and TranslateOnPeriodic: not ported (TopOpeBRepDS's data structure; plan 2.11).

    /// <summary>Makes an empty wire in <paramref name="w"/>.</summary>
    internal void MakeWire(ref Shape w)
    {
        Wire aW = new();
        _builder.MakeWire(ref aW);
        w = aW;
    }

    /// <summary>An empty copy of the face <paramref name="fin"/> (same surface, tolerance and orientation, no wires).</summary>
    internal void CopyFace(in Shape fin, out Shape fou)
    {
        fou = fin.EmptyCopied();
    }

    /// <summary>Adds the edge <paramref name="e"/> to the wire <paramref name="w"/>.</summary>
    internal void AddWireEdge(in Shape w, in Shape e)
    {
        _builder.Add(w, e);
    }

    /// <summary>Adds the wire <paramref name="w"/> to the face <paramref name="f"/>.</summary>
    internal void AddFaceWire(in Shape f, in Shape w)
    {
        _builder.Add(f, w);
    }

    /// <summary>
    /// Sets the parameter of the vertex <paramref name="V"/> on the edge <paramref name="E"/> to <paramref name="P"/>,
    /// brought into the period after the edge's first parameter for a REVERSED vertex on a periodic curve.
    /// </summary>
    internal void Parameter(in Shape E, in Shape V, double P)
    {
        Edge e = ShapeCasts.Edge(E);
        Vertex v = ShapeCasts.Vertex(V);
        double p = P;

        // 13/07/95 :
        Curve3D? c = Tool.Curve(e, out Placement _, out double f, out double _);
        if (c is not null && c.IsPeriodic)
        {
            double per = c.Period;

            Orientation oV = Forward;

            Explorer exV = new(e, ShapeType.Vertex);
            for (; exV.More(); exV.Next())
            {
                Vertex vofe = ShapeCasts.Vertex(exV.Current());
                if (vofe.IsSame(v))
                {
                    oV = vofe.Orientation;
                    break;
                }
            }
            if (exV.More())
            {
                if (oV == Reversed)
                {
                    if (p < f)
                    {
                        double pp = global::Kerf.Engine.Geometry.ElCLib.InPeriod(p, f, f + per);
                        p = pp;
                    }
                }
            }
        }
        _builder.UpdateVertex(v, p, e,
                               0); // NYI : Tol on new vertex ??
    }

    /// <summary>Sets the Closed flag of <paramref name="s"/>.</summary>
    internal void Closed(in Shape s, bool b)
    {
        s.Closed(b);
    }
}
