// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_FindEdgesInFace.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_FindEdgesInFace.lxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_FindEdgesInFace.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Occt;

/// <summary>
/// Finds the edges of a shape lying in a face (planar or cylindrical): the edges the face shares, and the lines and
/// circles on its surface whose ends and middle are inside it.
/// </summary>
/// <remarks>A working object (decision 21): it only reads the shapes; the edges are the shape's own.</remarks>
public sealed class LocOpe_FindEdgesInFace
{
    private Shape myShape = new();
    private Face myFace = new();
    private readonly NCollection_List<Shape> myList = new();
    private readonly NCollection_List<Shape>.Iterator myIt = new();

    public LocOpe_FindEdgesInFace()
    {
    }

    public LocOpe_FindEdgesInFace(in Shape S, in Face F)
    {
        Set(S, F);
    }

    public void Init()
    {
        myIt.Initialize(myList);
    }

    public bool More() => myIt.More();

    public Edge Edge() => ShapeCasts.Edge(myIt.Value());

    public void Next()
    {
        myIt.Next();
    }

    public void Set(in Shape Sh, in Face F)
    {
        myShape = Sh;
        myFace = F;
        myList.Clear();

        NCollection_Map<Shape> M = new(ShapeHasher.Instance);
        Explorer exp = new(), expf = new();
        Curve3D C;
        Surface S;
        double f, l;
        Type Tc, Ts;
        bool ToAdd;
        Plane3D pl = new();
        Cylinder cy = new();

        double Tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        double TolAng = global::Kerf.Engine.Foundation.Precision.Angular();

        S = Tool.Surface(F)!;
        Ts = S.GetType();
        if (Ts == typeof(TrimmedSurface))
        {
            S = ((TrimmedSurface)S).BasisSurface();
            Ts = S.GetType();
        }

        if (Ts != typeof(PlaneSurface) && Ts != typeof(CylindricalSurface))
        {
            return; // pour le moment
        }

        FaceSurfaceAdaptor HS = new(myFace);
        BRepTopAdaptor_TopolTool TPT = new(HS);

        for (exp.Init(myShape, global::Kerf.Topology.ShapeType.Edge); exp.More(); exp.Next())
        {
            ToAdd = false;
            Edge edg = ShapeCasts.Edge(exp.Current());
            if (M.Contains(edg))
            {
                continue;
            }

            for (expf.Init(myFace, global::Kerf.Topology.ShapeType.Edge); expf.More(); expf.Next())
            {
                if (expf.Current().IsSame(edg))
                {
                    break;
                }
            }
            if (expf.More())
            { // partage d`edge
                M.Add(edg);
                myList.Append(edg);
                continue;
            }

            C = Tool.Curve(edg, out Placement Loc, out f, out l)!;
            C = (Curve3D)C.Transformed(Loc.Transformation());
            Tc = C.GetType();
            if (Tc == typeof(TrimmedCurve3D))
            {
                C = ((TrimmedCurve3D)C).BasisCurve();
                Tc = C.GetType();
            }
            if (Tc != typeof(LineCurve3D) && Tc != typeof(CircleCurve3D))
            {
                continue; // pour le moment
            }
            if (Ts == typeof(PlaneSurface))
            {
                pl = ((PlaneSurface)S).Pln();
            }
            else
            {
                cy = ((CylindricalSurface)S).Cylinder();
            }

            if (Tc == typeof(LineCurve3D))
            {
                Line3D li = ((LineCurve3D)C).Lin();
                if (Ts == typeof(PlaneSurface))
                {
                    if (pl.Contains(li, Tol, TolAng))
                    {
                        ToAdd = true;
                    }
                }
                else
                { // Ts == STANDARD_TYPE(Geom_CylindricalSurface)
                    if (cy.Axis().IsParallel(li.Position(), TolAng) && Math.Abs(li.Distance(cy.Location()) - cy.Radius()) < Tol)
                    {
                        ToAdd = true;
                    }
                }
            }
            else
            { // Tt == STANDARD_TYPE(Geom_Circle)
                Circle3D ci = ((CircleCurve3D)C).Circ();
                if (Ts == typeof(PlaneSurface))
                {
                    if (pl.Position().IsCoplanar(new CoordinateSystem(ci.Position()), Tol, TolAng))
                    {
                        ToAdd = true;
                    }
                }
                else
                {
                    if (Math.Abs(cy.Radius() - ci.Radius()) < Tol && cy.Axis().IsCoaxial(ci.Axis(), TolAng, Tol))
                    {
                        ToAdd = true;
                    }
                }
            }

            if (ToAdd)
            {
                // On classifie 3 points.
                Point3D[] p = new Point3D[3];
                double U, V;
                p[0] = C.Value(f);
                p[1] = C.Value(l);
                p[2] = C.Value((f + l) / 2.0);
                int i;
                for (i = 0; i < 3; i++)
                {
                    if (Ts == typeof(PlaneSurface))
                    {
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(pl, p[i], out U, out V);
                    }
                    else
                    {
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(cy, p[i], out U, out V);
                    }
                    if (TPT.Classify(new Point2D(U, V), global::Kerf.Engine.Foundation.Precision.Confusion()) == Out)
                    {
                        break;
                    }
                }
                if (i >= 3)
                {
                    M.Add(edg);
                    myList.Append(edg);
                }
            }
        }
    }
}
