// Port of: src/ModelingAlgorithms/TKOffset/Draft/Draft_EdgeInfo.hxx
// Port of: src/ModelingAlgorithms/TKOffset/Draft/Draft_EdgeInfo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Features.Draft;

/// <summary>What the draft modification knows of an edge: its new curve, its two faces and their new pcurves.</summary>
/// <remarks>
/// A value type in C++, held by value in Draft_Modification's map; a class in the port (decision 1033): the
/// modification creates one per edge and changes it only through the map, as OCCT's <c>ChangeFromKey</c> does.
/// </remarks>
public sealed class DraftEdgeInfo
{
    private bool _newGeom;
    private Curve3D? _geom;
    private Face _firstF = new();
    private Face _seconF = new();
    private Curve2D? _firstPC;
    private Curve2D? _seconPC;
    private Face _rootFace = new();
    private bool _tgt;
    private Point3D _pt = new();
    private double _tol;

    public DraftEdgeInfo()
    {
        _newGeom = false;
        _tgt = false;
        _tol = 0;
    }

    public DraftEdgeInfo(bool hasNewGeometry)
    {
        _newGeom = hasNewGeometry;
        _tgt = false;
        _tol = 0;
    }

    public void Add(in Face f)
    {
        if (_firstF.IsNull)
        {
            _firstF = f;
        }
        else if (!_firstF.IsSame(f) && _seconF.IsNull)
        {
            _seconF = f;
        }
        _tol = Max(_tol, Tool.Tolerance(f));
    }

    public void RootFace(in Face f) => _rootFace = f;

    public void Tangent(in Point3D p)
    {
        _tgt = true;
        _pt = p;
    }

    public bool IsTangent(out Point3D p)
    {
        p = _pt;
        return _tgt;
    }

    public bool NewGeometry() => _newGeom;

    public void SetNewGeometry(bool newGeom) => _newGeom = newGeom;

    public Curve3D? Geometry() => _geom;

    public Face FirstFace() => _firstF;

    public Face SecondFace() => _seconF;

    public Curve2D? FirstPC() => _firstPC;

    public Curve2D? SecondPC() => _seconPC;

    public ref Curve3D? ChangeGeometry() => ref _geom;

    public ref Curve2D? ChangeFirstPC() => ref _firstPC;

    public ref Curve2D? ChangeSecondPC() => ref _seconPC;

    public Face RootFace() => _rootFace;

    public void Tolerance(double tol) => _tol = tol;

    public double Tolerance() => _tol;
}
