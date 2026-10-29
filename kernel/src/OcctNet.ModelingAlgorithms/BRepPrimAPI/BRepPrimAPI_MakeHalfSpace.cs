// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeHalfSpace.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeHalfSpace.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BRepExtrema_SupportType;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Builds half-spaces: infinite solids limited by a face or a shell, with the matter on the side of a reference point.
/// A half-space is commonly used to cut another shape.
/// </summary>
/// <remarks>
/// The solid holds the caller's face (in a new shell) or shell, shared and unchanged; <see cref="Solid"/> and
/// <see cref="BRepBuilderAPI_MakeShape.Shape"/> freeze the solid (design 4.2).
/// </remarks>
public class BRepPrimAPI_MakeHalfSpace : BRepBuilderAPI_MakeShape
{
    private TopoDS_Solid mySolid = new();

    private static gp_Dir getNormalOnFace(in TopoDS_Face theFace, double theU, double theV)
    {
        double aPrec = gp.Resolution();
        BRepLProp_SLProps aProps = new(new BRepAdaptor_Surface(theFace), theU, theV, 2, aPrec);
        gp_Dir aNormal = aProps.Normal();
        if (theFace.Orientation() == TopAbs_REVERSED)
        {
            aNormal.Reverse();
        }
        return aNormal;
    }

    // Get average normal at the point with the given parameter on the edge
    private static bool getNormalFromEdge(in TopoDS_Shape theShape, in TopoDS_Edge theEdge, double thePar, ref gp_Dir theNormal)
    {
        gp_XYZ aSum = new();
        TopExp_Explorer ex = new(theShape, TopAbs_FACE);
        for (; ex.More(); ex.Next())
        {
            TopoDS_Face aF = TopoDS.Face(ex.Current());
            TopExp_Explorer ex1 = new(aF, TopAbs_EDGE);
            for (; ex1.More(); ex1.Next())
            {
                if (ex1.Current().IsSame(theEdge))
                {
                    Geom2d_Curve? aC2d = BRep_Tool.CurveOnSurface(theEdge, aF, out _, out _);
                    gp_Pnt2d aP2d = aC2d!.Value(thePar);
                    gp_Dir aNorm = getNormalOnFace(aF, aP2d.X(), aP2d.Y());
                    aSum += aNorm.XYZ();
                }
            }
        }
        if (aSum.SquareModulus() > gp.Resolution())
        {
            theNormal = new gp_Dir(aSum);
            return true;
        }
        return false;
    }

    // Get average normal at the point of the vertex
    private static bool getNormalFromVertex(in TopoDS_Shape theShape, in TopoDS_Vertex theVer, ref gp_Dir theNormal)
    {
        gp_XYZ aSum = new();
        TopExp_Explorer ex = new(theShape, TopAbs_FACE);
        for (; ex.More(); ex.Next())
        {
            TopoDS_Face aF = TopoDS.Face(ex.Current());
            TopExp_Explorer ex1 = new(aF, TopAbs_VERTEX);
            for (; ex1.More(); ex1.Next())
            {
                if (ex1.Current().IsSame(theVer))
                {
                    gp_Pnt2d aP2d = BRep_Tool.Parameters(theVer, aF);
                    gp_Dir aNorm = getNormalOnFace(aF, aP2d.X(), aP2d.Y());
                    aSum += aNorm.XYZ();
                }
            }
        }
        if (aSum.SquareModulus() > gp.Resolution())
        {
            theNormal = new gp_Dir(aSum);
            return true;
        }
        return false;
    }

    // The nearest normal projection of thePnt on theShape: true if found, with the solution point in theMinPnt and the
    // normal to the shape there in theNormal.
    private static bool FindExtrema(in gp_Pnt thePnt, in TopoDS_Shape theShape, ref gp_Pnt theMinPnt, ref gp_Dir theNormal)
    {
        TopoDS_Vertex aRefVertex = new BRepBuilderAPI_MakeVertex(thePnt).InternalVertex();
        BRepExtrema_DistShapeShape ext = new(aRefVertex, theShape);

        if (!ext.IsDone() || ext.NbSolution() == 0)
        {
            return false;
        }

        // the point projection exist
        int nbext = ext.NbSolution();
        // try to find a projection on face
        for (int iext = 1; iext <= nbext; iext++)
        {
            if (ext.SupportTypeShape2(iext) == BRepExtrema_IsInFace)
            {
                TopoDS_Face aF = TopoDS.Face(ext.SupportOnShape2(iext));
                theMinPnt = ext.PointOnShape2(iext);
                ext.ParOnFaceS2(iext, out double aU, out double aV);
                theNormal = getNormalOnFace(aF, aU, aV);
                return true;
            }
        }

        // if not found then take any edge or vertex solution
        for (int iext = 1; iext <= nbext; iext++)
        {
            if (ext.SupportTypeShape2(iext) == BRepExtrema_IsOnEdge)
            {
                theMinPnt = ext.PointOnShape2(iext);
                ext.ParOnEdgeS2(iext, out double aPar);
                TopoDS_Edge aE = TopoDS.Edge(ext.SupportOnShape2(iext));
                if (getNormalFromEdge(theShape, aE, aPar, ref theNormal))
                {
                    return true;
                }
            }
            else if (ext.SupportTypeShape2(iext) == BRepExtrema_IsVertex)
            {
                theMinPnt = ext.PointOnShape2(iext);
                TopoDS_Vertex aV = TopoDS.Vertex(ext.SupportOnShape2(iext));
                if (getNormalFromVertex(theShape, aV, ref theNormal))
                {
                    return true;
                }
            }
        }
        return false;
    }

    private static bool isOutside(in gp_Pnt thePnt, in gp_Pnt thePonF, in gp_Dir theNormal)
    {
        gp_Dir anOppRef = new(thePnt.XYZ() - thePonF.XYZ());
        double aSca = theNormal * anOppRef;
        // outside if same directions
        return aSca > 0.0;
    }

    /// <summary>A half-space bounded by a face, on the side of the reference point.</summary>
    public BRepPrimAPI_MakeHalfSpace(in TopoDS_Face theFace, in gp_Pnt theRefPnt)
    {
        // Set the flag is <IsDone> to False.
        NotDone();

        TopoDS_Shell aShell = new();

        gp_Pnt aMinPnt = new();
        gp_Dir aNormal = new();
        if (FindExtrema(theRefPnt, theFace, ref aMinPnt, ref aNormal))
        {
            bool toReverse = isOutside(theRefPnt, aMinPnt, aNormal);

            // Construction of the open solid.
            new BRep_Builder().MakeShell(ref aShell);
            new BRep_Builder().Add(aShell, theFace);
            new BRep_Builder().MakeSolid(ref mySolid);
            if (toReverse)
            {
                aShell.Reverse();
            }

            new BRep_Builder().Add(mySolid, aShell);
            myShape = mySolid;
            Done();
        }
    }

    /// <summary>A half-space bounded by a shell, on the side of the reference point.</summary>
    public BRepPrimAPI_MakeHalfSpace(in TopoDS_Shell theShell, in gp_Pnt theRefPnt)
    {
        // Set the flag is <IsDone> to False.
        NotDone();

        // Find the point of the skin closest to the reference point.
        gp_Pnt aMinPnt = new();
        gp_Dir aNormal = new();
        if (FindExtrema(theRefPnt, theShell, ref aMinPnt, ref aNormal))
        {
            bool toReverse = isOutside(theRefPnt, aMinPnt, aNormal);

            // Construction of the open solid.
            TopoDS_Shell aShell = theShell;
            new BRep_Builder().MakeSolid(ref mySolid);
            if (toReverse)
            {
                aShell.Reverse();
            }
            new BRep_Builder().Add(mySolid, aShell);
            myShape = mySolid;
            Done();
        }
    }

    /// <summary>The half-space as a solid (frozen).</summary>
    /// <exception cref="StdFail_NotDone">It was not built.</exception>
    public TopoDS_Solid Solid()
    {
        TopoDS_Solid aSolid = InternalSolid();
        aSolid.TShape()?.FreezeGraph();
        return aSolid;
    }

    /// <summary><see cref="Solid"/> without freezing, for ported code.</summary>
    internal TopoDS_Solid InternalSolid()
    {
        StdFail_NotDone_Raise_if(!IsDone(), "BRepPrimAPI_MakeHalfSpace::Solid");
        return mySolid;
    }

    /// <summary>The solid (OCCT's conversion operator; frozen).</summary>
    public static explicit operator TopoDS_Solid(BRepPrimAPI_MakeHalfSpace theMaker) => theMaker.Solid();
}
