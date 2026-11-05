// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Edge3DLaw.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Edge3DLaw.cxx
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

/// <summary>A location law on a wire: a copy of the given law on each non-degenerated edge's 3D curve.</summary>
/// <remarks>The caller's law is only copied (<c>Copy()</c>), never changed; each edge's law is the wire law's own.</remarks>
internal class Edge3DLaw : LocationLaw
{
    /// <summary>The law on <paramref name="path"/> with copies of <paramref name="law"/>.</summary>
    public Edge3DLaw(in Wire path, SurfaceLocationLaw law)
    {
        Init(path);

        int ipath;
        Orientation or;
        WireExplorer wexp = new();
        // Class BRep_Tool without fields and without Constructor :
        //  BRep_Tool B;
        Edge e;
        Curve3D c;
        CurveAdaptor ac;

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
                    cBis.Reverse(); // Pour eviter de deteriorer la topologie
                    c = cBis;
                    first = c.FirstParameter;
                    last = c.LastParameter;
                }

                ac = new CurveAdaptor(c, first, last);
                _laws!.SetValue(ipath, law.Copy());
                _laws.ChangeValue(ipath).SetCurve(ac);
            }
        }
    }
}
