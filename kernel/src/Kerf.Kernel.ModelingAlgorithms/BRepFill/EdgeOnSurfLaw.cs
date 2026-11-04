// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_EdgeOnSurfLaw.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_EdgeOnSurfLaw.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Surfaces;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Construction.Filling;

/// <summary>
/// A location law on a wire lying on a shape's faces: Darboux's trihedron along each edge's curve on the first face
/// of the shape holding a pcurve of it.
/// </summary>
internal sealed class EdgeOnSurfLaw : LocationLaw
{
    private readonly bool _hasresult;

    /// <summary>The law on <paramref name="path"/> over the faces of <paramref name="surf"/>.</summary>
    public EdgeOnSurfLaw(in Wire path, in Shape surf)
    {
        _hasresult = true;
        Init(path);

        bool trouve;
        int ipath; //  ,NbEdge;
        Orientation or;
        WireExplorer wexp = new();
        Explorer exp = new();
        // Class BRep_Tool without fields and without Constructor :
        //  BRep_Tool B;
        Edge e;
        Curve2D? c = null;
        CurveAdaptor2D ac2d;
        CurveOnSurface ac;
        FaceSurfaceAdaptor? AS = null;
        double first = 0.0, last = 0.0;
        Darboux tLaw = new();
        CurveAndTrihedron law = new(tLaw);

        for (ipath = 0, wexp.Init(_path); wexp.More(); wexp.Next())
        {
            e = wexp.Current();
            //    if (!B.Degenerated(E)) {
            if (!Tool.Degenerated(e))
            {
                ipath++;
                _edges!.SetValue(ipath, e);
                for (trouve = false, exp.Init(surf, ShapeType.Face); exp.More() && !trouve; exp.Next())
                {
                    Face f = ShapeCasts.Face(exp.Current());
                    c = Tool.CurveOnSurface(e, f, out first, out last);
                    if (c is not null)
                    {
                        trouve = true;
                        AS = new FaceSurfaceAdaptor(f);
                    }
                }
                if (!trouve)
                { // Impossible to construct the law.
                    _hasresult = false;
                    _laws = null;
                    return;
                }

                or = e.Orientation;
                if (or == Reversed)
                {
                    TrimmedCurve2D cBis = new(c!, first, last);
                    cBis.Reverse(); // To avoid spoiling the topology
                    c = cBis;
                    first = c.FirstParameter;
                    last = c.LastParameter;
                }

                ac2d = new CurveAdaptor2D(c!, first, last);
                ac = new CurveOnSurface(ac2d, AS!);
                _laws!.SetValue(ipath, law.Copy());
                _laws.ChangeValue(ipath).SetCurve(ac);
            }
        }
    }

    /// <summary>
    /// False if an edge of the path has no representation on the shape's faces: the law can't be used then (it has no
    /// elementary laws).
    /// </summary>
    public bool HasResult => _hasresult;
}
