// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_ACRLaw.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_ACRLaw.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;

namespace Occt;

/// <summary>
/// A location law on a wire for a guided contour, with the trihedron by reduced curvilinear abscissa: each edge gets a
/// copy of the guide law with its origin set to the edge's part of the wire's abscissa.
/// </summary>
/// <remarks>
/// As OCCT, the constructor calls <c>SetOrigine</c> on the caller's guide law before copying it for each edge, so the
/// caller's law is left with the last edge's origin (laws are working objects, decision 806).
/// </remarks>
public sealed class BRepFill_ACRLaw : BRepFill_LocationLaw
{
    private readonly NCollection_HArray1<double> OrigParam;

    /// <summary>The law on <paramref name="Path"/> with the guide law <paramref name="theLaw"/>.</summary>
    public BRepFill_ACRLaw(in Wire Path, GeomFill_LocationGuide theLaw)
    {
        Init(Path);

        // calculate the nb of edge of the path
        WireExplorer wexp = new();
        int NbEdge = 0;
        for (wexp.Init(myPath); wexp.More(); wexp.Next())
        {
            NbEdge++;
        }

        // tab to memorize ACR for each edge
        OrigParam = new NCollection_HArray1<double>(0, NbEdge);
        NCollection_Array1<double> Orig = new(0, NbEdge);
        BRepFill.ComputeACR(Path, Orig);

        int ipath;
        Orientation Or;
        // Class BRep_Tool without fields and without Constructor :
        //  BRep_Tool B;
        Edge E;
        Curve3D C;
        CurveAdaptor AC;

        // return ACR of edges of the trajectory
        OrigParam.SetValue(0, 0);
        for (ipath = 1; ipath <= NbEdge; ipath++)
        {
            OrigParam.SetValue(ipath, Orig[ipath]);
        }

        // process each edge of the trajectory
        for (ipath = 0, wexp.Init(myPath); wexp.More(); wexp.Next())
        {
            E = wexp.Current();
            //    if (!B.Degenerated(E)) {
            if (!Tool.Degenerated(E))
            {
                ipath++;
                myEdges!.SetValue(ipath, E);
                C = Tool.Curve(E, out double First, out double Last)!;
                Or = E.Orientation();
                if (Or == Reversed)
                {
                    TrimmedCurve3D CBis = new(C, First, Last);
                    CBis.Reverse(); // To avoid damaging the topology
                    C = CBis;
                    First = C.FirstParameter();
                    Last = C.LastParameter();
                }
                AC = new CurveAdaptor(C, First, Last);

                // Set the parameters for the case multi-edges
                // OCCT: OrigParam is indexed by all the wire's edges and ipath counts the non-degenerated ones only, so
                // after a degenerated edge the origins are those of earlier edges (occt-issues.md)
                double t1 = OrigParam.Value(ipath - 1);
                double t2 = OrigParam.Value(ipath);
                GeomFill_LocationGuide Loc;
                Loc = theLaw;
                Loc.SetOrigine(t1, t2);

                myLaws!.SetValue(ipath, Loc.Copy());
                myLaws.ChangeValue(ipath).SetCurve(AC);
            }
        }
    }
}
