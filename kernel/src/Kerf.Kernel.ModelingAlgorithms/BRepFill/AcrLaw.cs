// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_ACRLaw.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_ACRLaw.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Surfaces;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Construction.Filling;

/// <summary>
/// A location law on a wire for a guided contour, with the trihedron by reduced curvilinear abscissa: each edge gets a
/// copy of the guide law with its origin set to the edge's part of the wire's abscissa.
/// </summary>
/// <remarks>
/// As OCCT, the constructor calls <c>SetOrigine</c> on the caller's guide law before copying it for each edge, so the
/// caller's law is left with the last edge's origin (laws are working objects, decision 806).
/// </remarks>
internal sealed class AcrLaw : LocationLaw
{
    private readonly NCollection_HArray1<double> _origParam;

    /// <summary>The law on <paramref name="path"/> with the guide law <paramref name="law"/>.</summary>
    public AcrLaw(in Wire path, LocationGuide law)
    {
        Init(path);

        // calculate the nb of edge of the path
        WireExplorer wexp = new();
        int nbEdge = 0;
        for (wexp.Init(_path); wexp.More(); wexp.Next())
        {
            nbEdge++;
        }

        // tab to memorize ACR for each edge
        _origParam = new NCollection_HArray1<double>(0, nbEdge);
        NCollection_Array1<double> orig = new(0, nbEdge);
        Fill.ComputeACR(path, orig);

        int ipath;
        Orientation or;
        // Class BRep_Tool without fields and without Constructor :
        //  BRep_Tool B;
        Edge e;
        Curve3D c;
        CurveAdaptor ac;

        // return ACR of edges of the trajectory
        _origParam.SetValue(0, 0);
        for (ipath = 1; ipath <= nbEdge; ipath++)
        {
            _origParam.SetValue(ipath, orig[ipath]);
        }

        // process each edge of the trajectory
        for (ipath = 0, wexp.Init(_path); wexp.More(); wexp.Next())
        {
            e = wexp.Current();
            //    if (!B.Degenerated(E)) {
            if (!Tool.Degenerated(e))
            {
                ipath++;
                _edges!.SetValue(ipath, e);
                c = Tool.Curve(e, out double first, out double last)!;
                or = e.Orientation;
                if (or == Reversed)
                {
                    TrimmedCurve3D cBis = new(c, first, last);
                    cBis.Reverse(); // To avoid damaging the topology
                    c = cBis;
                    first = c.FirstParameter;
                    last = c.LastParameter;
                }
                ac = new CurveAdaptor(c, first, last);

                // Set the parameters for the case multi-edges
                // OCCT: OrigParam is indexed by all the wire's edges and ipath counts the non-degenerated ones only, so
                // after a degenerated edge the origins are those of earlier edges (occt-issues.md)
                double t1 = _origParam.Value(ipath - 1);
                double t2 = _origParam.Value(ipath);
                LocationGuide loc;
                loc = law;
                loc.SetOrigine(t1, t2);

                _laws!.SetValue(ipath, loc.Copy());
                _laws.ChangeValue(ipath).SetCurve(ac);
            }
        }
    }
}
