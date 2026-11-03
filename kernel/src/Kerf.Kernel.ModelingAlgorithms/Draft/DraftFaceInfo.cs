// Port of: src/ModelingAlgorithms/TKOffset/Draft/Draft_FaceInfo.hxx
// Port of: src/ModelingAlgorithms/TKOffset/Draft/Draft_FaceInfo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Features.Draft;

/// <summary>What the draft modification knows of a face: its new surface and the face it was added with.</summary>
/// <remarks>A value type in C++; a class in the port (decision 1033), as <see cref="DraftEdgeInfo"/>.</remarks>
public sealed class DraftFaceInfo
{
    private bool _newGeom;
    private Surface? _geom;
    private Face _rootFace = new();
    private Face _f1 = new();
    private Face _f2 = new();
    private Curve3D? _curv;

    // OCCT leaves myNewGeom uninitialised in the default constructor; false here.
    public DraftFaceInfo()
    {
    }

    public DraftFaceInfo(Surface? s, bool hasNewGeometry)
    {
        _newGeom = hasNewGeometry;
        if (s is TrimmedSurface t)
        {
            _geom = t.BasisSurface();
        }
        else
        {
            _geom = s;
        }
    }

    public void RootFace(in Face f) => _rootFace = f;

    public void Add(in Face f)
    {
        if (_f1.IsNull)
        {
            _f1 = f;
        }
        else if (_f2.IsNull)
        {
            _f2 = f;
        }
    }

    public Face FirstFace() => _f1;

    public Face SecondFace() => _f2;

    public bool NewGeometry() => _newGeom;

    public Surface? Geometry() => _geom;

    public ref Surface? ChangeGeometry() => ref _geom;

    public Curve3D? Curve() => _curv;

    public ref Curve3D? ChangeCurve() => ref _curv;

    public Face RootFace() => _rootFace;
}
