// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_FaceBuilder.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_FaceBuilder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>
/// Builds a face from a surface: the whole surface, or the area between UMin, UMax, VMin, VMax.
/// </summary>
/// <remarks>Internal (decision 3201): the face is unfrozen, for ported code that goes on building on it.</remarks>
internal sealed class BRepPrim_FaceBuilder
{
    private readonly TopoDS_Vertex[] myVertex = BRepPrim_GWedge.NewArray<TopoDS_Vertex>(4);
    private readonly TopoDS_Edge[] myEdges = BRepPrim_GWedge.NewArray<TopoDS_Edge>(4);
    private TopoDS_Face myFace = new();

    public BRepPrim_FaceBuilder()
    {
    }

    public BRepPrim_FaceBuilder(BRep_Builder B, Geom_Surface S)
    {
        Init(B, S);
    }

    public BRepPrim_FaceBuilder(BRep_Builder B, Geom_Surface S, double UMin, double UMax, double VMin, double VMax)
    {
        Init(B, S, UMin, UMax, VMin, VMax);
    }

    public void Init(BRep_Builder B, Geom_Surface S)
    {
        S.Bounds(out double UMin, out double UMax, out double VMin, out double VMax);
        Init(B, S, UMin, UMax, VMin, VMax);
    }

    /// <exception cref="Standard_ConstructionError">The bounds are empty or out of the surface's.</exception>
    public void Init(BRep_Builder B, Geom_Surface S, double UMin, double UMax, double VMin, double VMax)
    {
        // Check the values
        S.Bounds(out double USMin, out double USMax, out double VSMin, out double VSMax);

        if (UMin >= UMax)
        {
            throw new Standard_ConstructionError("BRepPrim_FaceBuilder");
        }
        if (VMin >= VMax)
        {
            throw new Standard_ConstructionError("BRepPrim_FaceBuilder");
        }
        if (UMin < USMin)
        {
            throw new Standard_ConstructionError("BRepPrim_FaceBuilder");
        }
        if (UMax > USMax)
        {
            throw new Standard_ConstructionError("BRepPrim_FaceBuilder");
        }
        if (VMin < VSMin)
        {
            throw new Standard_ConstructionError("BRepPrim_FaceBuilder");
        }
        if (VMax > VSMax)
        {
            throw new Standard_ConstructionError("BRepPrim_FaceBuilder");
        }

        // Make the vertices
        B.MakeVertex(ref myVertex[0], S.Value(UMin, VMin), Precision.Confusion());
        B.MakeVertex(ref myVertex[1], S.Value(UMax, VMin), Precision.Confusion());
        B.MakeVertex(ref myVertex[2], S.Value(UMax, VMax), Precision.Confusion());
        B.MakeVertex(ref myVertex[3], S.Value(UMin, VMax), Precision.Confusion());

        // Make the edges
        B.MakeEdge(ref myEdges[0]);
        B.MakeEdge(ref myEdges[1]);
        B.MakeEdge(ref myEdges[2]);
        B.MakeEdge(ref myEdges[3]);

        // Make the face
        B.MakeFace(ref myFace, S, Precision.Confusion());

        // set the pcurves
        Geom2d_Line L;
        L = new Geom2d_Line(new gp_Pnt2d(UMin, VMin), new gp_Dir2d(gp_Dir2d.D.X));
        B.UpdateEdge(myEdges[0], L, myFace, Precision.Confusion());
        L = new Geom2d_Line(new gp_Pnt2d(UMax, VMin), new gp_Dir2d(gp_Dir2d.D.Y));
        B.UpdateEdge(myEdges[1], L, myFace, Precision.Confusion());
        L = new Geom2d_Line(new gp_Pnt2d(UMax, VMax), new gp_Dir2d(gp_Dir2d.D.NX));
        B.UpdateEdge(myEdges[2], L, myFace, Precision.Confusion());
        L = new Geom2d_Line(new gp_Pnt2d(UMin, VMax), new gp_Dir2d(gp_Dir2d.D.NY));
        B.UpdateEdge(myEdges[3], L, myFace, Precision.Confusion());

        // set the parameters
        B.UpdateVertex(myVertex[0], 0, myEdges[0], 0);
        B.UpdateVertex(myVertex[1], UMax - UMin, myEdges[0], 0);
        B.UpdateVertex(myVertex[1], 0, myEdges[1], 0);
        B.UpdateVertex(myVertex[2], VMax - VMin, myEdges[1], 0);
        B.UpdateVertex(myVertex[2], 0, myEdges[2], 0);
        B.UpdateVertex(myVertex[3], UMax - UMin, myEdges[2], 0);
        B.UpdateVertex(myVertex[3], 0, myEdges[3], 0);
        B.UpdateVertex(myVertex[0], VMax - VMin, myEdges[3], 0);

        // insert vertices in edges
        myVertex[0].Orientation(TopAbs_REVERSED);
        B.Add(myEdges[3], myVertex[0]);
        myVertex[0].Orientation(TopAbs_FORWARD);
        B.Add(myEdges[0], myVertex[0]);
        myVertex[1].Orientation(TopAbs_REVERSED);
        B.Add(myEdges[0], myVertex[1]);
        myVertex[1].Orientation(TopAbs_FORWARD);
        B.Add(myEdges[1], myVertex[1]);
        myVertex[2].Orientation(TopAbs_REVERSED);
        B.Add(myEdges[1], myVertex[2]);
        myVertex[2].Orientation(TopAbs_FORWARD);
        B.Add(myEdges[2], myVertex[2]);
        myVertex[3].Orientation(TopAbs_REVERSED);
        B.Add(myEdges[2], myVertex[3]);
        myVertex[3].Orientation(TopAbs_FORWARD);
        B.Add(myEdges[3], myVertex[3]);

        // insert edges in a wire and the wire in the face
        TopoDS_Wire W = new();
        B.MakeWire(ref W);
        B.Add(W, myEdges[0]);
        B.Add(W, myEdges[1]);
        B.Add(W, myEdges[2]);
        B.Add(W, myEdges[3]);
        B.Add(myFace, W);

        // set the natural restriction flag
        if (UMin == USMin && UMax == USMax && VMin == VSMin && VMax == VSMax)
        {
            B.NaturalRestriction(myFace, true);
        }
    }

    public TopoDS_Face Face()
    {
        return myFace;
    }

    /// <summary>The edge of index I: 1 VMin, 2 UMax, 3 VMax, 4 UMin.</summary>
    public TopoDS_Edge Edge(int I)
    {
        Standard_OutOfRange_Raise_if(I < 1 || I > 4, "BRepPrim_FaceBuilder::Edge");
        return myEdges[I - 1];
    }

    /// <summary>The vertex of index I: 1 (UMin,VMin), 2 (UMax,VMin), 3 (UMax,VMax), 4 (UMin,VMax).</summary>
    public TopoDS_Vertex Vertex(int I)
    {
        Standard_OutOfRange_Raise_if(I < 1 || I > 4, "BRepPrim_FaceBuilder::Vertex");
        return myVertex[I - 1];
    }

    public static explicit operator TopoDS_Face(BRepPrim_FaceBuilder theBuilder) => theBuilder.Face();
}
