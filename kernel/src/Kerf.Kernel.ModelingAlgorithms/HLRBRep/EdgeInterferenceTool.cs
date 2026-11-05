// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_EdgeInterferenceTool.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_EdgeInterferenceTool.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_EdgeInterferenceTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.HiddenLines.Algorithm;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.HiddenLines;

/// <summary>
/// The tool HLRBRep_EdgeIList and HLRBRep_VertexList use to read the current edge of the hidden-line data: its two
/// vertices (as intersections), its 2D geometry at a parameter, and the geometry of an interference's boundary.
/// </summary>
/// <remarks>
/// OCCT's value class (HLRBRep_VertexList copies it); a class here with OCCT's implicit copy as
/// <see cref="EdgeInterferenceTool(EdgeInterferenceTool)"/> (the data shared, the vertices and the
/// current index copied). A working object.
/// </remarks>
internal sealed class EdgeInterferenceTool
{
    private readonly HiddenLineData _ds;
    private readonly AlgoIntersection[] _inter = new AlgoIntersection[2];
    private int _cur;

    public EdgeInterferenceTool(HiddenLineData ds)
    {
        _ds = ds;
    }

    /// <summary>OCCT's implicit copy.</summary>
    public EdgeInterferenceTool(EdgeInterferenceTool other)
    {
        _ds = other._ds;
        _inter[0] = other._inter[0];
        _inter[1] = other._inter[1];
        _cur = other._cur;
    }

    /// <summary>Reads the vertices of the data's current edge.</summary>
    public void LoadEdge()
    {
        double p1, p2;
        float t1, t2;
        NCollection_Array1<HiddenLineEdgeData> ED = _ds.EDataArray();
        HiddenLineEdgeData ed = ED[_ds.Edge()];
        ed.Status().Bounds(out p1, out t1, out p2, out t2);
        _inter[0].Parameter(p1);
        _inter[0].Tolerance(t1);
        _inter[0].Index(ed.VSta());
        _inter[1].Parameter(p2);
        _inter[1].Tolerance(t2);
        _inter[1].Index(ed.VEnd());
    }

    public void InitVertices() => _cur = 0;

    public bool MoreVertices() => _cur < 2;

    public void NextVertex() => _cur++;

    public AlgoIntersection CurrentVertex() => _inter[_cur];

    public Orientation CurrentOrientation()
    {
        if (_cur == 0)
        {
            return Forward;
        }
        else
        {
            return Reversed;
        }
    }

    public double CurrentParameter() => _inter[_cur].Parameter();

    public bool IsPeriodic => false;

    /// <summary>The tangent, normal and curvature of the current edge at Param, in the projection plane.</summary>
    public void EdgeGeometry(double param, out Direction3D tgt, out Direction3D nrm, out double crLE)
    {
        _ds.LocalLEGeometry2D(param, out Direction2D tgLE, out Direction2D nmLE, out crLE);
        tgt = new Direction3D(tgLE.X, tgLE.Y, 0);
        nrm = new Direction3D(nmLE.X, nmLE.Y, 0);
    }

    public double ParameterOfInterference(in Interference i) => i.Intersection().Parameter();

    /// <summary>True if the two interferences are at the same vertex.</summary>
    public bool SameInterferences(in Interference i1, in Interference i2)
    {
        int ind1 = i1.Intersection().Index();
        int ind2 = i2.Intersection().Index();
        if (ind1 != 0 && ind2 != 0)
        {
            return ind1 == ind2;
        }
        return false;
    }

    /// <summary>True if the interference is at the current vertex.</summary>
    public bool SameVertexAndInterference(in Interference i)
    {
        if (i.Intersection().Index() == _inter[_cur].Index())
        {
            return true;
        }
        return i.Intersection().Orientation() == ((_cur == 0) ? Forward : Reversed);
    }

    /// <summary>The tangent, normal and curvature of the interference's boundary edge, in the projection plane.</summary>
    public void InterferenceBoundaryGeometry(in Interference i, out Direction3D tang, out Direction3D norm,
                                             out double crFE)
    {
        i.Boundary().Value2D(out int fe, out double param);
        _ds.LocalFEGeometry2D(fe, param, out Direction2D tgFE, out Direction2D nmFE, out crFE);
        tang = new Direction3D(tgFE.X, tgFE.Y, 0);
        norm = new Direction3D(nmFE.X, nmFE.Y, 0);
    }
}
