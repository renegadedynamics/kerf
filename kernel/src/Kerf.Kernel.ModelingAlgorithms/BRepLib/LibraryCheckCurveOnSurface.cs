// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_CheckCurveOnSurface.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_CheckCurveOnSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;

using Kerf.Engine.Geometry.Library;

using Kerf.Engine.Topology;

using Kerf.Engine.Topology.Adaptors;

using Kerf.Engine.Topology.Brep;

using Kerf.Geometry;

using Kerf.Topology;

namespace Kerf.Engine.Construction.Library;

/// <summary>
/// Computes the max distance between an edge's 3D curve and its 2D representation on a face (both pcurves on a
/// closed surface). Not intended for edges that aren't SameParameter.
/// </summary>
/// <remarks>A working object over <see cref="CheckCurveOnSurface"/> (decision 293).</remarks>
internal class LibraryCheckCurveOnSurface
{
    private readonly CheckCurveOnSurface _cOnSurfGeom = new();
    private CurveOnSurface? _adaptorCurveOnSurface;
    private CurveOnSurface? _adaptorCurveOnSurface2;
    private bool _isParallel;

    /// <summary>An empty algorithm; <see cref="Init"/> sets its data.</summary>
    public LibraryCheckCurveOnSurface() => _isParallel = false;

    public LibraryCheckCurveOnSurface(in Edge edge, in Face face)
    {
        _isParallel = false;
        Init(edge, face);
    }

    /// <summary>Sets the data for the algorithm; nothing for a null, degenerated or non-geometric edge.</summary>
    public void Init(in Edge edge, in Face face)
    {
        _cOnSurfGeom.Init();

        if (edge.IsNull || face.IsNull)
        {
            return;
        }

        //
        if (Tool.Degenerated(edge) || !Tool.IsGeometric(edge))
        {
            return;
        }

        // 3D curve initialization
        CurveAdaptorBase adaptor3dCurve = new EdgeCurveAdaptor(edge);

        // Surface initialization
        Curve2D? geom2dCurve = Tool.CurveOnSurface(edge, face, out double firstParam, out double lastParam);
        Surface? geomSurface = Tool.Surface(face);

        // 2D curves initialization
        Curve2DAdaptorBase adaptorCurve = new CurveAdaptor2D(geom2dCurve!, firstParam, lastParam);
        var geomAdaptorSurface = new SurfaceAdaptor(geomSurface!);

        _adaptorCurveOnSurface = new CurveOnSurface(adaptorCurve, geomAdaptorSurface);

        if (Tool.IsClosed(edge, face))
        {
            Curve2D? geom2dReversedCurve =
                Tool.CurveOnSurface(ShapeCasts.Edge(edge.Reversed()), face, out firstParam, out lastParam);
            Curve2DAdaptorBase adaptorReversedCurve = new CurveAdaptor2D(geom2dReversedCurve!, firstParam, lastParam);
            _adaptorCurveOnSurface2 = new CurveOnSurface(adaptorReversedCurve, geomAdaptorSurface);
        }

        _cOnSurfGeom.Init(adaptor3dCurve);
    }

    /// <summary>Computes the max distance, in parallel if <see cref="SetParallel"/> asked for it.</summary>
    public void Perform()
    {
        // Compute the max distance
        Compute(_adaptorCurveOnSurface);
        if (ErrorStatus() != 0)
        {
            return;
        }

        //
        if (_adaptorCurveOnSurface2 is not null)
        {
            // compute max distance for myAdaptorCurveOnSurface2
            // (for the second curve on closed surface)
            Compute(_adaptorCurveOnSurface2);
        }
    }

    /// <summary>True if the max distance has been found.</summary>
    public bool IsDone => _cOnSurfGeom.ErrorStatus() == 0;

    public void SetParallel(bool isParallel) => _isParallel = isParallel;

    public bool IsParallel => _isParallel;

    /// <summary>0: OK; 1: null curve, surface or 2D curve; 2: invalid parametric range; 3: error in calculations.</summary>
    public int ErrorStatus() => _cOnSurfGeom.ErrorStatus();

    public double MaxDistance() => _cOnSurfGeom.MaxDistance();

    /// <summary>The parameter at which the distance is maximal.</summary>
    public double MaxParameter() => _cOnSurfGeom.MaxParameter();

    /// <summary>Computes the max distance between the 3D curve and <paramref name="curveOnSurface"/>.</summary>
    protected void Compute(CurveOnSurface? curveOnSurface)
    {
        _cOnSurfGeom.SetParallel(_isParallel);
        _cOnSurfGeom.Perform(curveOnSurface);
    }
}
