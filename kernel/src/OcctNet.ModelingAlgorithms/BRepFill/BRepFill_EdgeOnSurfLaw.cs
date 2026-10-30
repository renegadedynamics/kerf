// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_EdgeOnSurfLaw.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_EdgeOnSurfLaw.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// A location law on a wire lying on a shape's faces: Darboux's trihedron along each edge's curve on the first face
/// of the shape holding a pcurve of it.
/// </summary>
public sealed class BRepFill_EdgeOnSurfLaw : BRepFill_LocationLaw
{
    private readonly bool hasresult;

    /// <summary>The law on <paramref name="Path"/> over the faces of <paramref name="Surf"/>.</summary>
    public BRepFill_EdgeOnSurfLaw(in TopoDS_Wire Path, in TopoDS_Shape Surf)
    {
        hasresult = true;
        Init(Path);

        bool Trouve;
        int ipath; //  ,NbEdge;
        TopAbs_Orientation Or;
        BRepTools_WireExplorer wexp = new();
        TopExp_Explorer exp = new();
        // Class BRep_Tool without fields and without Constructor :
        //  BRep_Tool B;
        TopoDS_Edge E;
        Geom2d_Curve? C = null;
        Geom2dAdaptor_Curve AC2d;
        Adaptor3d_CurveOnSurface AC;
        BRepAdaptor_Surface? AS = null;
        double First = 0.0, Last = 0.0;
        GeomFill_Darboux TLaw = new();
        GeomFill_CurveAndTrihedron Law = new(TLaw);

        for (ipath = 0, wexp.Init(myPath); wexp.More(); wexp.Next())
        {
            E = wexp.Current();
            //    if (!B.Degenerated(E)) {
            if (!BRep_Tool.Degenerated(E))
            {
                ipath++;
                myEdges!.SetValue(ipath, E);
                for (Trouve = false, exp.Init(Surf, TopAbs_FACE); exp.More() && !Trouve; exp.Next())
                {
                    TopoDS_Face F = TopoDS.Face(exp.Current());
                    C = BRep_Tool.CurveOnSurface(E, F, out First, out Last);
                    if (C is not null)
                    {
                        Trouve = true;
                        AS = new BRepAdaptor_Surface(F);
                    }
                }
                if (!Trouve)
                { // Impossible to construct the law.
                    hasresult = false;
                    myLaws = null;
                    return;
                }

                Or = E.Orientation();
                if (Or == TopAbs_REVERSED)
                {
                    Geom2d_TrimmedCurve CBis = new(C!, First, Last);
                    CBis.Reverse(); // To avoid spoiling the topology
                    C = CBis;
                    First = C.FirstParameter();
                    Last = C.LastParameter();
                }

                AC2d = new Geom2dAdaptor_Curve(C!, First, Last);
                AC = new Adaptor3d_CurveOnSurface(AC2d, AS!);
                myLaws!.SetValue(ipath, Law.Copy());
                myLaws.ChangeValue(ipath).SetCurve(AC);
            }
        }
    }

    /// <summary>
    /// False if an edge of the path has no representation on the shape's faces: the law can't be used then (it has no
    /// elementary laws).
    /// </summary>
    public bool HasResult() => hasresult;
}
