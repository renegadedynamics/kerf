// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>Threading and sweeping utilities of BRepFill.</summary>
/// <remarks>
/// Only <see cref="Axe"/> is ported here (unit 5O2, for BRepOffsetAPI_MakeEvolved); <c>Face</c>, <c>Shell</c>,
/// <c>SearchOrigin</c>, <c>ComputeACR</c> and <c>InsertACR</c> are the sweep units' (5S1), which add them to this
/// partial class.
/// </remarks>
public static partial class BRepFill
{
    /// <summary>
    /// Computes <paramref name="AxeProf"/>: its location is the position of the vertex V of
    /// <paramref name="Profile"/> nearest to <paramref name="Spine"/>, its X direction the tangent to the spine at the
    /// projection of V, its direction the normal to the spine (a planar wire or face).
    /// <paramref name="ProfOnSpine"/> is true when that distance is below <paramref name="Tol"/>.
    /// </summary>
    /// <exception cref="Standard_NoSuchObject">The face isn't planar.</exception>
    public static void Axe(in TopoDS_Shape Spine, in TopoDS_Wire Profile, out gp_Ax3 AxeProf, out bool ProfOnSpine, double Tol)
    {
        gp_Pnt Loc = new(), Loc1, Loc2;
        gp_Vec Tang = new(), Tang1, Tang2, Normal;

        Geom_Surface? S = null;
        TopLoc_Location L = new();

        TopoDS_Face aFace = new();

        // normal to the Spine.
        if (Spine.ShapeType() == TopAbs_FACE)
        {
            aFace = TopoDS.Face(Spine);
            S = BRep_Tool.Surface(TopoDS.Face(Spine), out L);
            if (S is not Geom_Plane)
            {
                BRepLib_FindSurface FS = new(TopoDS.Face(Spine), -1, true);
                if (FS.Found())
                {
                    S = FS.Surface();
                    L = FS.Location();
                }
                else
                {
                    throw new Standard_NoSuchObject("BRepFill_Evolved : The Face is not planar");
                }
            }
        }
        else if (Spine.ShapeType() == TopAbs_WIRE)
        {
            aFace = new BRepLib_MakeFace(TopoDS.Wire(Spine), true).InternalFace();
            S = BRep_Tool.Surface(aFace, out L);
        }

        if (S is null)
        {
            throw new Standard_DomainError("BRepFill_Evolved::Axe");
        }

        if (!L.IsIdentity())
        {
            S = (Geom_Surface)S.Transformed(L.Transformation());
        }

        Normal = new gp_Vec(((Geom_Plane)S).Pln().Axis().Direction());

        // Find vertex of the profile closest to the spine.
        double DistMin = Precision.Infinite();
        double Dist;
        double Tol2 = 1.0e-10;
        TopExp_Explorer PE = new(), SE = new();
        BRepExtrema_ExtPC BE = new();
        double Par = 0.0;
        gp_Pnt P1, P2;

        // First check if there is contact Vertex Vertex.
        bool IsOnVertex = false;
        SE.Init(aFace.Oriented(TopAbs_FORWARD), TopAbs_VERTEX);
        //  modified by NIZHNY-EAP Wed Feb 23 12:31:52 2000 ___BEGIN___
        for (; SE.More(); SE.Next())
        {
            P1 = BRep_Tool.Pnt(TopoDS.Vertex(SE.Current()));

            PE.Init(Profile, TopAbs_VERTEX);
            for (; PE.More(); PE.Next())
            {
                P2 = BRep_Tool.Pnt(TopoDS.Vertex(PE.Current()));
                double DistP1P2 = P1.SquareDistance(P2);
                IsOnVertex = (DistP1P2 <= Tol2);
                if (IsOnVertex)
                {
                    break;
                }
            }
            // otherwise SE.Next() is done and VonF is wrong
            if (IsOnVertex)
            {
                break;
            }
            //  modified by NIZHNY-EAP Wed Jan 26 09:08:36 2000 ___END___
        }

        if (IsOnVertex)
        {
            // try to find on which edge which shared this vertex,
            // the profile must be considered.
            // E1, E2 : those two edges.
            NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> Map = new(TopTools_ShapeMapHasher.Instance);
            TopExp.MapShapesAndAncestors(aFace.Oriented(TopAbs_FORWARD), TopAbs_VERTEX, TopAbs_EDGE, Map);

            TopoDS_Vertex VonF = TopoDS.Vertex(SE.Current());
            NCollection_List<TopoDS_Shape> List = Map.FindFromKey(VonF);
            TopoDS_Edge E1 = TopoDS.Edge(List.First());
            TopoDS_Edge E2 = TopoDS.Edge(List.Last());

            Geom_Curve CE1 = BRep_Tool.Curve(E1, out L, out double _, out double _)!;
            double Par1 = BRep_Tool.Parameter(VonF, E1, aFace);
            CE1.D1(Par1, out Loc1, out Tang1);
            if (!L.IsIdentity())
            {
                Tang1.Transform(L.Transformation());
                Loc1.Transform(L.Transformation());
            }
            if (E1.Orientation() == TopAbs_REVERSED)
            {
                Tang1.Reverse();
            }

            Geom_Curve CE2 = BRep_Tool.Curve(E2, out L, out double _, out double _)!;
            double Par2 = BRep_Tool.Parameter(VonF, E2, aFace);
            CE2.D1(Par2, out Loc2, out Tang2);
            if (!L.IsIdentity())
            {
                Tang2.Transform(L.Transformation());
                Loc2.Transform(L.Transformation());
            }
            if (E2.Orientation() == TopAbs_REVERSED)
            {
                Tang2.Reverse();
            }

            //  modified by NIZHNY-EAP Wed Feb  2 15:38:41 2000 ___BEGIN___
            Tang1.Normalize();
            Tang2.Normalize();
            double sca1 = 0.0, sca2 = 0.0;
            for (PE.Init(Profile, TopAbs_EDGE); PE.More(); PE.Next())
            {
                TopoDS_Edge E = TopoDS.Edge(PE.Current());
                TopExp.Vertices(E, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
                P1 = BRep_Tool.Pnt(V1);
                P2 = BRep_Tool.Pnt(V2);
                gp_Vec vec = new(P1, P2);
                sca1 += Math.Abs(Tang1.Dot(vec));
                sca2 += Math.Abs(Tang2.Dot(vec));
            }
            //  modified by NIZHNY-EAP Wed Feb  2 15:38:44 2000 ___END___

            if (Math.Abs(sca1) < Math.Abs(sca2))
            {
                Loc = Loc1;
                Tang = Tang1;
            }
            else
            {
                Loc = Loc2;
                Tang = Tang2;
            }
            DistMin = 0.0;
        }
        else
        {
            SE.Init(aFace.Oriented(TopAbs_FORWARD), TopAbs_EDGE);
            for (; SE.More(); SE.Next())
            {
                TopoDS_Edge E = TopoDS.Edge(SE.Current());
                BE.Initialize(E);
                for (PE.Init(Profile, TopAbs_VERTEX); PE.More(); PE.Next())
                {
                    Dist = Precision.Infinite();
                    TopoDS_Vertex V = TopoDS.Vertex(PE.Current());
                    BE.Perform(V);
                    if (BE.IsDone())
                    {
                        // extrema.
                        for (int i = 1; i <= BE.NbExt(); i++)
                        {
                            if (BE.IsMin(i))
                            {
                                Dist = Math.Sqrt(BE.SquareDistance(i));
                                Par = BE.Parameter(i);
                                break;
                            }
                        }
                    }
                    // save minimum.
                    if (Dist < DistMin)
                    {
                        DistMin = Dist;
                        BRepAdaptor_Curve BAC = new(E);
                        BAC.D1(Par, out Loc, out Tang);
                        if (E.Orientation() == TopAbs_REVERSED)
                        {
                            Tang.Reverse();
                        }
                    }
                }
            }
        }

        ProfOnSpine = (DistMin < Tol);
        // Construction AxeProf;
        gp_Ax3 A3 = new(Loc, new gp_Dir(Normal), new gp_Dir(Tang));
        AxeProf = A3;
    }
}
