// Port of: src/ModelingAlgorithms/TKFillet/BRepFilletAPI/BRepFilletAPI_MakeFillet2d.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepFilletAPI/BRepFilletAPI_MakeFillet2d.lxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepFilletAPI/BRepFilletAPI_MakeFillet2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Fillets.Planar;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Occt;
using Schematiq.Collections;
using static Kerf.Engine.Fillets.Planar.ConstructionError;

namespace Kerf.Engine.Fillets.Api;

/// <summary>
/// Builds fillets and chamfers on the vertices of a planar face. Only segments of lines and arcs of circles are treated.
/// </summary>
/// <remarks>
/// <para>A working object over a <see cref="PlanarBuilder"/>, as OCCT's. The face it is given is never written; each
/// fillet or chamfer builds a new face sharing the face's untouched edges and vertices. <see cref="MakeShape.Shape"/>
/// freezes the result; the fillets, chamfers and descendant edges it hands out are frozen with it (decision 853).</para>
/// <para><see cref="MakeShape.Modified"/> answers the descendant of an edge of the face, and throws
/// Standard_NoSuchObject for an edge that has none, as OCCT does.</para>
/// </remarks>
internal class MakeFillet2D : MakeShape
{
    private readonly PlanarBuilder _makeChFi2d = new();

    /// <summary>An empty algorithm: the face is given by <see cref="Init(in Face)"/>.</summary>
    public MakeFillet2D()
    {
    }

    /// <summary>
    /// An algorithm on the face <paramref name="f"/>: <see cref="Status"/> is ChFi2d_Ready, ChFi2d_NotPlanar or
    /// ChFi2d_NoFace.
    /// </summary>
    public MakeFillet2D(in Face f)
    {
        _makeChFi2d.Init(f);
    }

    /// <summary>
    /// Initializes the algorithm with the face <paramref name="f"/>: <see cref="Status"/> is ChFi2d_Ready,
    /// ChFi2d_NotPlanar or ChFi2d_NoFace.
    /// </summary>
    public void Init(in Face f)
    {
        _makeChFi2d.Init(f);
    }

    /// <summary>
    /// Initializes the algorithm with a face <paramref name="refFace"/> and a face <paramref name="modFace"/> derived from
    /// it, to modify the fillets and chamfers already built on <paramref name="modFace"/>.
    /// </summary>
    public void Init(in Face refFace, in Face modFace)
    {
        _makeChFi2d.Init(refFace, modFace);
    }

    /// <summary>
    /// Adds a fillet of radius <paramref name="radius"/> between the two edges adjacent to the vertex
    /// <paramref name="v"/>, and returns it (frozen). <see cref="Status"/>: ChFi2d_IsDone, ChFi2d_ConnexionError (V not on
    /// the face), ChFi2d_ComputationError (radius too large), ChFi2d_NotAuthorized (an edge is a fillet or chamfer, or
    /// not a line or circle), ChFi2d_TangencyError.
    /// </summary>
    public Edge AddFillet(in Vertex v, double radius)
    {
        return _makeChFi2d.AddFillet(v, radius);
    }

    /// <summary>Gives the fillet <paramref name="fillet"/> the radius <paramref name="radius"/>; returns the new fillet (frozen).</summary>
    public Edge ModifyFillet(in Edge fillet, double radius)
    {
        return _makeChFi2d.ModifyFillet(fillet, radius);
    }

    /// <summary>
    /// Removes the fillet <paramref name="fillet"/>; returns the vertex connecting its two adjacent edges (frozen), null
    /// if <paramref name="fillet"/> is not on the face.
    /// </summary>
    public Vertex RemoveFillet(in Edge fillet)
    {
        return _makeChFi2d.RemoveFillet(fillet);
    }

    /// <summary>
    /// Adds a chamfer between the adjacent edges <paramref name="e1"/> and <paramref name="e2"/>, its ends at the
    /// distances <paramref name="d1"/> and <paramref name="d2"/> along them; returns it (frozen).
    /// </summary>
    public Edge AddChamfer(in Edge e1, in Edge e2, double d1, double d2)
    {
        return _makeChFi2d.AddChamfer(e1, e2, d1, d2);
    }

    /// <summary>
    /// Adds a chamfer between the two edges connected by the vertex <paramref name="v"/>, making the angle
    /// <paramref name="ang"/> (radians) with <paramref name="e"/>, one end on <paramref name="e"/> at the distance
    /// <paramref name="d"/> from <paramref name="v"/>; returns it (frozen).
    /// </summary>
    public Edge AddChamfer(in Edge e, in Vertex v, double d, double ang)
    {
        return _makeChFi2d.AddChamfer(e, v, d, ang);
    }

    /// <summary>Modifies the chamfer <paramref name="chamfer"/> built between <paramref name="e1"/> and <paramref name="e2"/>.</summary>
    public Edge ModifyChamfer(in Edge chamfer, in Edge e1, in Edge e2, double d1, double d2)
    {
        return _makeChFi2d.ModifyChamfer(chamfer, e1, e2, d1, d2);
    }

    /// <summary>Modifies the chamfer <paramref name="chamfer"/>: an angle <paramref name="ang"/> with <paramref name="e"/>.</summary>
    public Edge ModifyChamfer(in Edge chamfer, in Edge e, double d, double ang)
    {
        return _makeChFi2d.ModifyChamfer(chamfer, e, d, ang);
    }

    /// <summary>
    /// Removes the chamfer <paramref name="chamfer"/>; returns the vertex connecting its two adjacent edges (frozen),
    /// null if <paramref name="chamfer"/> is not on the face.
    /// </summary>
    public Vertex RemoveChamfer(in Edge chamfer)
    {
        return _makeChFi2d.RemoveChamfer(chamfer);
    }

    /// <summary>True if the edge <paramref name="e"/> of the face is chamfered or filleted.</summary>
    public bool IsModified(in Edge e)
    {
        return _makeChFi2d.IsModified(e);
    }

    /// <summary>The fillets on the face (frozen).</summary>
    public ISchematiqList<Shape> FilletEdges()
    {
        return _makeChFi2d.FilletEdges();
    }

    /// <summary>The number of fillets on the face.</summary>
    public int FilletCount
    {
        get
        {
            return _makeChFi2d.FilletCount;
        }
    }

    /// <summary>The chamfers on the face (frozen).</summary>
    public ISchematiqList<Shape> ChamferEdges()
    {
        return _makeChFi2d.ChamferEdges();
    }

    /// <summary>The number of chamfers on the face.</summary>
    public int ChamferCount
    {
        get
        {
            return _makeChFi2d.ChamferCount;
        }
    }

    // Modified(S): the shapes modified from S, the descendant edge of the edge S.
    internal override NCollection_List<Shape> InternalModified(in Shape e)
    {
        _generated.Clear();
        // OCCT: DescendantEdge reads the history with Find, which throws Standard_NoSuchObject for an unmodified edge
        _generated.Append(_makeChFi2d.InternalDescendantEdge(ShapeCasts.Edge(e)));
        return _generated;
    }

    // The lists hold edges of the face being built: freeze it with them (decision 414).
    private protected override void BeforePublishingLists()
    {
        _makeChFi2d.FreezeResult();
    }

    /// <summary>The number of new curves (fillets and chamfers).</summary>
    public int CurveCount
    {
        get
        {
            return FilletCount + ChamferCount;
        }
    }

    /// <summary>The edges created for the curve <paramref name="i"/>: the fillets first, then the chamfers (frozen).</summary>
    public ISchematiqList<Shape> NewEdges(int i)
    {
        _generated.Clear();
        if (i <= FilletCount)
        {
            _generated.Append(_makeChFi2d.InternalFilletEdges()[i]);
        }
        else
        {
            _generated.Append(_makeChFi2d.InternalChamferEdges()[i - FilletCount]);
        }

        BeforePublishingLists();
        return PublishedList(_generated);
    }

    public bool HasDescendant(in Edge e)
    {
        return _makeChFi2d.HasDescendant(e);
    }

    /// <summary>
    /// The chamfered or filleted edge built from the edge <paramref name="e"/> of the face (frozen).
    /// </summary>
    /// <exception cref="NoSuchObjectException"><paramref name="e"/> has no descendant.</exception>
    public Edge DescendantEdge(in Edge e)
    {
        return _makeChFi2d.DescendantEdge(e);
    }

    /// <summary>
    /// The edge of the face from which the edge <paramref name="e"/> was built; <paramref name="e"/> itself if it is
    /// not a descendant.
    /// </summary>
    public Edge BasisEdge(in Edge e)
    {
        return _makeChFi2d.BasisEdge(e);
    }

    public ConstructionError Status()
    {
        return _makeChFi2d.Status();
    }

    /// <summary>Updates the result and sets the Done flag (if <see cref="Status"/> is ChFi2d_IsDone).</summary>
    public override void Build(ProgressRange range = default)
    {
        // test if the operation is done
        if (Status() == ConstructionError.IsDone)
        {
            Done();
            _shape = _makeChFi2d.InternalResult();
        }
        else
        {
            NotDone();
        }
    }
}
