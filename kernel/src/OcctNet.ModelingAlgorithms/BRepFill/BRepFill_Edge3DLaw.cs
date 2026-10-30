// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Edge3DLaw.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Edge3DLaw.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>A location law on a wire: a copy of the given law on each non-degenerated edge's 3D curve.</summary>
/// <remarks>The caller's law is only copied (<c>Copy()</c>), never changed; each edge's law is the wire law's own.</remarks>
public sealed class BRepFill_Edge3DLaw : BRepFill_LocationLaw
{
    /// <summary>The law on <paramref name="Path"/> with copies of <paramref name="Law"/>.</summary>
    public BRepFill_Edge3DLaw(in TopoDS_Wire Path, GeomFill_LocationLaw Law)
    {
        Init(Path);

        int ipath;
        TopAbs_Orientation Or;
        BRepTools_WireExplorer wexp = new();
        // Class BRep_Tool without fields and without Constructor :
        //  BRep_Tool B;
        TopoDS_Edge E;
        Geom_Curve C;
        GeomAdaptor_Curve AC;

        for (ipath = 0, wexp.Init(myPath); wexp.More(); wexp.Next())
        {
            E = wexp.Current();
            //    if (!B.Degenerated(E)) {
            if (!BRep_Tool.Degenerated(E))
            {
                ipath++;
                myEdges!.SetValue(ipath, E);
                C = BRep_Tool.Curve(E, out double First, out double Last)!;
                Or = E.Orientation();
                if (Or == TopAbs_REVERSED)
                {
                    Geom_TrimmedCurve CBis = new(C, First, Last);
                    CBis.Reverse(); // Pour eviter de deteriorer la topologie
                    C = CBis;
                    First = C.FirstParameter();
                    Last = C.LastParameter();
                }

                AC = new GeomAdaptor_Curve(C, First, Last);
                myLaws!.SetValue(ipath, Law.Copy());
                myLaws.ChangeValue(ipath).SetCurve(AC);
            }
        }
    }
}
