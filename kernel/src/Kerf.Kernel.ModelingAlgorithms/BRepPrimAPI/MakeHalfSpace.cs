// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeHalfSpace.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeHalfSpace.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Measurement.Distance;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Properties;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Measurement.Distance.SupportType;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Construction.Api;

/// <summary>
/// Builds half-spaces: infinite solids limited by a face or a shell, with the matter on the side of a reference point.
/// A half-space is commonly used to cut another shape.
/// </summary>
/// <remarks>
/// The solid holds the caller's face (in a new shell) or shell, shared and unchanged; <see cref="Solid"/> and
/// <see cref="MakeShape.Shape"/> freeze the solid (design 4.2).
/// </remarks>
internal class MakeHalfSpace : MakeShape
{
    private Solid _solid = new();

    private static Direction3D GetNormalOnFace(in Face face, double u, double v)
    {
        double prec = Constants.Resolution();
        SLProps props = new(new FaceSurfaceAdaptor(face), u, v, 2, prec);
        Direction3D normal = props.Normal();
        if (face.Orientation == Reversed)
        {
            normal = normal.Reversed();
        }
        return normal;
    }

    // Get average normal at the point with the given parameter on the edge
    private static bool GetNormalFromEdge(in Shape shape, in Edge edge, double par, ref Direction3D normal)
    {
        Xyz sum = new();
        Explorer ex = new(shape, ShapeType.Face);
        for (; ex.More(); ex.Next())
        {
            Face f = ShapeCasts.Face(ex.Current());
            Explorer ex1 = new(f, ShapeType.Edge);
            for (; ex1.More(); ex1.Next())
            {
                if (ex1.Current().IsSame(edge))
                {
                    Curve2D? c2d = Tool.CurveOnSurface(edge, f, out _, out _);
                    Point2D p2d = c2d!.Value(par);
                    Direction3D norm = GetNormalOnFace(f, p2d.X, p2d.Y);
                    sum += norm.XYZ();
                }
            }
        }
        if (sum.SquareModulus() > Constants.Resolution())
        {
            normal = new Direction3D(sum);
            return true;
        }
        return false;
    }

    // Get average normal at the point of the vertex
    private static bool GetNormalFromVertex(in Shape shape, in Vertex ver, ref Direction3D normal)
    {
        Xyz sum = new();
        Explorer ex = new(shape, ShapeType.Face);
        for (; ex.More(); ex.Next())
        {
            Face f = ShapeCasts.Face(ex.Current());
            Explorer ex1 = new(f, ShapeType.Vertex);
            for (; ex1.More(); ex1.Next())
            {
                if (ex1.Current().IsSame(ver))
                {
                    Point2D p2d = Tool.Parameters(ver, f);
                    Direction3D norm = GetNormalOnFace(f, p2d.X, p2d.Y);
                    sum += norm.XYZ();
                }
            }
        }
        if (sum.SquareModulus() > Constants.Resolution())
        {
            normal = new Direction3D(sum);
            return true;
        }
        return false;
    }

    // The nearest normal projection of thePnt on theShape: true if found, with the solution point in theMinPnt and the
    // normal to the shape there in theNormal.
    private static bool FindExtrema(in Point3D pnt, in Shape shape, ref Point3D minPnt, ref Direction3D normal)
    {
        Vertex refVertex = new MakeVertex(pnt).InternalVertex();
        DistanceShapeShape ext = new(refVertex, shape);

        if (!ext.IsDone || ext.SolutionCount == 0)
        {
            return false;
        }

        // the point projection exist
        int nbext = ext.SolutionCount;
        // try to find a projection on face
        for (int iext = 1; iext <= nbext; iext++)
        {
            if (ext.SupportTypeShape2(iext) == SupportType.IsInFace)
            {
                Face f = ShapeCasts.Face(ext.SupportOnShape2(iext));
                minPnt = ext.PointOnShape2(iext);
                ext.ParOnFaceS2(iext, out double u, out double v);
                normal = GetNormalOnFace(f, u, v);
                return true;
            }
        }

        // if not found then take any edge or vertex solution
        for (int iext = 1; iext <= nbext; iext++)
        {
            if (ext.SupportTypeShape2(iext) == SupportType.IsOnEdge)
            {
                minPnt = ext.PointOnShape2(iext);
                ext.ParOnEdgeS2(iext, out double par);
                Edge e = ShapeCasts.Edge(ext.SupportOnShape2(iext));
                if (GetNormalFromEdge(shape, e, par, ref normal))
                {
                    return true;
                }
            }
            else if (ext.SupportTypeShape2(iext) == SupportType.IsVertex)
            {
                minPnt = ext.PointOnShape2(iext);
                Vertex v = ShapeCasts.Vertex(ext.SupportOnShape2(iext));
                if (GetNormalFromVertex(shape, v, ref normal))
                {
                    return true;
                }
            }
        }
        return false;
    }

    private static bool IsOutside(in Point3D pnt, in Point3D ponF, in Direction3D normal)
    {
        Direction3D oppRef = new(pnt.XYZ() - ponF.XYZ());
        double sca = normal * oppRef;
        // outside if same directions
        return sca > 0.0;
    }

    /// <summary>A half-space bounded by a face, on the side of the reference point.</summary>
    public MakeHalfSpace(in Face face, in Point3D refPnt)
    {
        // Set the flag is <IsDone> to False.
        NotDone();

        Shell shell = new();

        Point3D minPnt = new();
        Direction3D normal = new();
        if (FindExtrema(refPnt, face, ref minPnt, ref normal))
        {
            bool toReverse = IsOutside(refPnt, minPnt, normal);

            // Construction of the open solid.
            new BrepBuilder().MakeShell(ref shell);
            new BrepBuilder().Add(shell, face);
            new BrepBuilder().MakeSolid(ref _solid);
            if (toReverse)
            {
                shell.Reverse();
            }

            new BrepBuilder().Add(_solid, shell);
            _shape = _solid;
            Done();
        }
    }

    /// <summary>A half-space bounded by a shell, on the side of the reference point.</summary>
    public MakeHalfSpace(in Shell shell, in Point3D refPnt)
    {
        // Set the flag is <IsDone> to False.
        NotDone();

        // Find the point of the skin closest to the reference point.
        Point3D minPnt = new();
        Direction3D normal = new();
        if (FindExtrema(refPnt, shell, ref minPnt, ref normal))
        {
            bool toReverse = IsOutside(refPnt, minPnt, normal);

            // Construction of the open solid.
            Shell aShell = shell;
            new BrepBuilder().MakeSolid(ref _solid);
            if (toReverse)
            {
                aShell.Reverse();
            }
            new BrepBuilder().Add(_solid, aShell);
            _shape = _solid;
            Done();
        }
    }

    /// <summary>The half-space as a solid (frozen).</summary>
    /// <exception cref="NotDoneException">It was not built.</exception>
    public Solid Solid()
    {
        Solid solid = InternalSolid();
        solid.TShape()?.FreezeGraph();
        return solid;
    }

    /// <summary><see cref="Solid"/> without freezing, for ported code.</summary>
    internal Solid InternalSolid()
    {
        StdFail_NotDone_Raise_if(!IsDone, "BRepPrimAPI_MakeHalfSpace::Solid");
        return _solid;
    }

    /// <summary>The solid (OCCT's conversion operator; frozen).</summary>
    public static explicit operator Solid(MakeHalfSpace maker) => maker.Solid();
}
