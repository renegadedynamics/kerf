// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_FindEdgesInFace.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_FindEdgesInFace.lxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_FindEdgesInFace.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// Finds the edges of a shape lying in a face (planar or cylindrical): the edges the face shares, and the lines and
/// circles on its surface whose ends and middle are inside it.
/// </summary>
/// <remarks>A working object (decision 21): it only reads the shapes; the edges are the shape's own.</remarks>
public sealed class LocOpe_FindEdgesInFace
{
    private TopoDS_Shape myShape = new();
    private TopoDS_Face myFace = new();
    private readonly NCollection_List<TopoDS_Shape> myList = new();
    private readonly NCollection_List<TopoDS_Shape>.Iterator myIt = new();

    public LocOpe_FindEdgesInFace()
    {
    }

    public LocOpe_FindEdgesInFace(in TopoDS_Shape S, in TopoDS_Face F)
    {
        Set(S, F);
    }

    public void Init()
    {
        myIt.Initialize(myList);
    }

    public bool More() => myIt.More();

    public TopoDS_Edge Edge() => TopoDS.Edge(myIt.Value());

    public void Next()
    {
        myIt.Next();
    }

    public void Set(in TopoDS_Shape Sh, in TopoDS_Face F)
    {
        myShape = Sh;
        myFace = F;
        myList.Clear();

        NCollection_Map<TopoDS_Shape> M = new(TopTools_ShapeMapHasher.Instance);
        TopExp_Explorer exp = new(), expf = new();
        Geom_Curve C;
        Geom_Surface S;
        double f, l;
        Type Tc, Ts;
        bool ToAdd;
        gp_Pln pl = new();
        gp_Cylinder cy = new();

        double Tol = Precision.Confusion();
        double TolAng = Precision.Angular();

        S = BRep_Tool.Surface(F)!;
        Ts = S.GetType();
        if (Ts == typeof(Geom_RectangularTrimmedSurface))
        {
            S = ((Geom_RectangularTrimmedSurface)S).BasisSurface();
            Ts = S.GetType();
        }

        if (Ts != typeof(Geom_Plane) && Ts != typeof(Geom_CylindricalSurface))
        {
            return; // pour le moment
        }

        BRepAdaptor_Surface HS = new(myFace);
        BRepTopAdaptor_TopolTool TPT = new(HS);

        for (exp.Init(myShape, TopAbs_EDGE); exp.More(); exp.Next())
        {
            ToAdd = false;
            TopoDS_Edge edg = TopoDS.Edge(exp.Current());
            if (M.Contains(edg))
            {
                continue;
            }

            for (expf.Init(myFace, TopAbs_EDGE); expf.More(); expf.Next())
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

            C = BRep_Tool.Curve(edg, out TopLoc_Location Loc, out f, out l)!;
            C = (Geom_Curve)C.Transformed(Loc.Transformation());
            Tc = C.GetType();
            if (Tc == typeof(Geom_TrimmedCurve))
            {
                C = ((Geom_TrimmedCurve)C).BasisCurve();
                Tc = C.GetType();
            }
            if (Tc != typeof(Geom_Line) && Tc != typeof(Geom_Circle))
            {
                continue; // pour le moment
            }
            if (Ts == typeof(Geom_Plane))
            {
                pl = ((Geom_Plane)S).Pln();
            }
            else
            {
                cy = ((Geom_CylindricalSurface)S).Cylinder();
            }

            if (Tc == typeof(Geom_Line))
            {
                gp_Lin li = ((Geom_Line)C).Lin();
                if (Ts == typeof(Geom_Plane))
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
                gp_Circ ci = ((Geom_Circle)C).Circ();
                if (Ts == typeof(Geom_Plane))
                {
                    if (pl.Position().IsCoplanar(new gp_Ax3(ci.Position()), Tol, TolAng))
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
                gp_Pnt[] p = new gp_Pnt[3];
                double U, V;
                p[0] = C.Value(f);
                p[1] = C.Value(l);
                p[2] = C.Value((f + l) / 2.0);
                int i;
                for (i = 0; i < 3; i++)
                {
                    if (Ts == typeof(Geom_Plane))
                    {
                        ElSLib.Parameters(pl, p[i], out U, out V);
                    }
                    else
                    {
                        ElSLib.Parameters(cy, p[i], out U, out V);
                    }
                    if (TPT.Classify(new gp_Pnt2d(U, V), Precision.Confusion()) == TopAbs_OUT)
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
