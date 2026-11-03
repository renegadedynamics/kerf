// Port of: src/ModelingAlgorithms/TKFillet/BRepFilletAPI/BRepFilletAPI_MakeFillet2d.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepFilletAPI/BRepFilletAPI_MakeFillet2d.lxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepFilletAPI/BRepFilletAPI_MakeFillet2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Schematiq.Collections;
using static Occt.ChFi2d_ConstructionError;

namespace Occt;

/// <summary>
/// Builds fillets and chamfers on the vertices of a planar face. Only segments of lines and arcs of circles are treated.
/// </summary>
/// <remarks>
/// <para>A working object over a <see cref="ChFi2d_Builder"/>, as OCCT's. The face it is given is never written; each
/// fillet or chamfer builds a new face sharing the face's untouched edges and vertices. <see cref="BRepBuilderAPI_MakeShape.Shape"/>
/// freezes the result; the fillets, chamfers and descendant edges it hands out are frozen with it (decision 853).</para>
/// <para><see cref="BRepBuilderAPI_MakeShape.Modified"/> answers the descendant of an edge of the face, and throws
/// Standard_NoSuchObject for an edge that has none, as OCCT does.</para>
/// </remarks>
public class BRepFilletAPI_MakeFillet2d : BRepBuilderAPI_MakeShape
{
    private readonly ChFi2d_Builder myMakeChFi2d = new();

    /// <summary>An empty algorithm: the face is given by <see cref="Init(in Face)"/>.</summary>
    public BRepFilletAPI_MakeFillet2d()
    {
    }

    /// <summary>
    /// An algorithm on the face <paramref name="F"/>: <see cref="Status"/> is ChFi2d_Ready, ChFi2d_NotPlanar or
    /// ChFi2d_NoFace.
    /// </summary>
    public BRepFilletAPI_MakeFillet2d(in Face F)
    {
        myMakeChFi2d.Init(F);
    }

    /// <summary>
    /// Initializes the algorithm with the face <paramref name="F"/>: <see cref="Status"/> is ChFi2d_Ready,
    /// ChFi2d_NotPlanar or ChFi2d_NoFace.
    /// </summary>
    public void Init(in Face F)
    {
        myMakeChFi2d.Init(F);
    }

    /// <summary>
    /// Initializes the algorithm with a face <paramref name="RefFace"/> and a face <paramref name="ModFace"/> derived from
    /// it, to modify the fillets and chamfers already built on <paramref name="ModFace"/>.
    /// </summary>
    public void Init(in Face RefFace, in Face ModFace)
    {
        myMakeChFi2d.Init(RefFace, ModFace);
    }

    /// <summary>
    /// Adds a fillet of radius <paramref name="Radius"/> between the two edges adjacent to the vertex
    /// <paramref name="V"/>, and returns it (frozen). <see cref="Status"/>: ChFi2d_IsDone, ChFi2d_ConnexionError (V not on
    /// the face), ChFi2d_ComputationError (radius too large), ChFi2d_NotAuthorized (an edge is a fillet or chamfer, or
    /// not a line or circle), ChFi2d_TangencyError.
    /// </summary>
    public Edge AddFillet(in Vertex V, double Radius)
    {
        return myMakeChFi2d.AddFillet(V, Radius);
    }

    /// <summary>Gives the fillet <paramref name="Fillet"/> the radius <paramref name="Radius"/>; returns the new fillet (frozen).</summary>
    public Edge ModifyFillet(in Edge Fillet, double Radius)
    {
        return myMakeChFi2d.ModifyFillet(Fillet, Radius);
    }

    /// <summary>
    /// Removes the fillet <paramref name="Fillet"/>; returns the vertex connecting its two adjacent edges (frozen), null
    /// if <paramref name="Fillet"/> is not on the face.
    /// </summary>
    public Vertex RemoveFillet(in Edge Fillet)
    {
        return myMakeChFi2d.RemoveFillet(Fillet);
    }

    /// <summary>
    /// Adds a chamfer between the adjacent edges <paramref name="E1"/> and <paramref name="E2"/>, its ends at the
    /// distances <paramref name="D1"/> and <paramref name="D2"/> along them; returns it (frozen).
    /// </summary>
    public Edge AddChamfer(in Edge E1, in Edge E2, double D1, double D2)
    {
        return myMakeChFi2d.AddChamfer(E1, E2, D1, D2);
    }

    /// <summary>
    /// Adds a chamfer between the two edges connected by the vertex <paramref name="V"/>, making the angle
    /// <paramref name="Ang"/> (radians) with <paramref name="E"/>, one end on <paramref name="E"/> at the distance
    /// <paramref name="D"/> from <paramref name="V"/>; returns it (frozen).
    /// </summary>
    public Edge AddChamfer(in Edge E, in Vertex V, double D, double Ang)
    {
        return myMakeChFi2d.AddChamfer(E, V, D, Ang);
    }

    /// <summary>Modifies the chamfer <paramref name="Chamfer"/> built between <paramref name="E1"/> and <paramref name="E2"/>.</summary>
    public Edge ModifyChamfer(in Edge Chamfer, in Edge E1, in Edge E2, double D1, double D2)
    {
        return myMakeChFi2d.ModifyChamfer(Chamfer, E1, E2, D1, D2);
    }

    /// <summary>Modifies the chamfer <paramref name="Chamfer"/>: an angle <paramref name="Ang"/> with <paramref name="E"/>.</summary>
    public Edge ModifyChamfer(in Edge Chamfer, in Edge E, double D, double Ang)
    {
        return myMakeChFi2d.ModifyChamfer(Chamfer, E, D, Ang);
    }

    /// <summary>
    /// Removes the chamfer <paramref name="Chamfer"/>; returns the vertex connecting its two adjacent edges (frozen),
    /// null if <paramref name="Chamfer"/> is not on the face.
    /// </summary>
    public Vertex RemoveChamfer(in Edge Chamfer)
    {
        return myMakeChFi2d.RemoveChamfer(Chamfer);
    }

    /// <summary>True if the edge <paramref name="E"/> of the face is chamfered or filleted.</summary>
    public bool IsModified(in Edge E)
    {
        return myMakeChFi2d.IsModified(E);
    }

    /// <summary>The fillets on the face (frozen).</summary>
    public ISchematiqList<Shape> FilletEdges()
    {
        return myMakeChFi2d.FilletEdges();
    }

    /// <summary>The number of fillets on the face.</summary>
    public int NbFillet()
    {
        return myMakeChFi2d.NbFillet();
    }

    /// <summary>The chamfers on the face (frozen).</summary>
    public ISchematiqList<Shape> ChamferEdges()
    {
        return myMakeChFi2d.ChamferEdges();
    }

    /// <summary>The number of chamfers on the face.</summary>
    public int NbChamfer()
    {
        return myMakeChFi2d.NbChamfer();
    }

    // Modified(S): the shapes modified from S, the descendant edge of the edge S.
    internal override NCollection_List<Shape> InternalModified(in Shape E)
    {
        myGenerated.Clear();
        // OCCT: DescendantEdge reads the history with Find, which throws Standard_NoSuchObject for an unmodified edge
        myGenerated.Append(myMakeChFi2d.InternalDescendantEdge(ShapeCasts.Edge(E)));
        return myGenerated;
    }

    // The lists hold edges of the face being built: freeze it with them (decision 414).
    private protected override void BeforePublishingLists()
    {
        myMakeChFi2d.FreezeResult();
    }

    /// <summary>The number of new curves (fillets and chamfers).</summary>
    public int NbCurves()
    {
        return NbFillet() + NbChamfer();
    }

    /// <summary>The edges created for the curve <paramref name="I"/>: the fillets first, then the chamfers (frozen).</summary>
    public ISchematiqList<Shape> NewEdges(int I)
    {
        myGenerated.Clear();
        if (I <= NbFillet())
        {
            myGenerated.Append(myMakeChFi2d.InternalFilletEdges()[I]);
        }
        else
        {
            myGenerated.Append(myMakeChFi2d.InternalChamferEdges()[I - NbFillet()]);
        }

        BeforePublishingLists();
        return PublishedList(myGenerated);
    }

    public bool HasDescendant(in Edge E)
    {
        return myMakeChFi2d.HasDescendant(E);
    }

    /// <summary>
    /// The chamfered or filleted edge built from the edge <paramref name="E"/> of the face (frozen).
    /// </summary>
    /// <exception cref="NoSuchObjectException"><paramref name="E"/> has no descendant.</exception>
    public Edge DescendantEdge(in Edge E)
    {
        return myMakeChFi2d.DescendantEdge(E);
    }

    /// <summary>
    /// The edge of the face from which the edge <paramref name="E"/> was built; <paramref name="E"/> itself if it is
    /// not a descendant.
    /// </summary>
    public Edge BasisEdge(in Edge E)
    {
        return myMakeChFi2d.BasisEdge(E);
    }

    public ChFi2d_ConstructionError Status()
    {
        return myMakeChFi2d.Status();
    }

    /// <summary>Updates the result and sets the Done flag (if <see cref="Status"/> is ChFi2d_IsDone).</summary>
    public override void Build(ProgressRange theRange = default)
    {
        // test if the operation is done
        if (Status() == ChFi2d_IsDone)
        {
            Done();
            myShape = myMakeChFi2d.InternalResult();
        }
        else
        {
            NotDone();
        }
    }
}
