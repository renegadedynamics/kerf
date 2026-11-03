// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_CheckCurveOnSurface.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_CheckCurveOnSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;

using Kerf.Engine.Geometry.Library;

using Kerf.Engine.Topology;

using Kerf.Engine.Topology.Adaptors;

using Kerf.Engine.Topology.Brep;

using Kerf.Geometry;

using Kerf.Topology;

namespace Occt;

/// <summary>
/// Computes the max distance between an edge's 3D curve and its 2D representation on a face (both pcurves on a
/// closed surface). Not intended for edges that aren't SameParameter.
/// </summary>
/// <remarks>A working object over <see cref="CheckCurveOnSurface"/> (decision 293).</remarks>
public class BRepLib_CheckCurveOnSurface
{
    private readonly CheckCurveOnSurface myCOnSurfGeom = new();
    private CurveOnSurface? myAdaptorCurveOnSurface;
    private CurveOnSurface? myAdaptorCurveOnSurface2;
    private bool myIsParallel;

    /// <summary>An empty algorithm; <see cref="Init"/> sets its data.</summary>
    public BRepLib_CheckCurveOnSurface() => myIsParallel = false;

    public BRepLib_CheckCurveOnSurface(in Edge theEdge, in Face theFace)
    {
        myIsParallel = false;
        Init(theEdge, theFace);
    }

    /// <summary>Sets the data for the algorithm; nothing for a null, degenerated or non-geometric edge.</summary>
    public void Init(in Edge theEdge, in Face theFace)
    {
        myCOnSurfGeom.Init();

        if (theEdge.IsNull || theFace.IsNull)
        {
            return;
        }

        //
        if (Tool.Degenerated(theEdge) || !Tool.IsGeometric(theEdge))
        {
            return;
        }

        // 3D curve initialization
        CurveAdaptorBase anAdaptor3dCurve = new EdgeCurveAdaptor(theEdge);

        // Surface initialization
        Curve2D? aGeom2dCurve = Tool.CurveOnSurface(theEdge, theFace, out double aFirstParam, out double aLastParam);
        Surface? aGeomSurface = Tool.Surface(theFace);

        // 2D curves initialization
        Curve2DAdaptorBase anAdaptorCurve = new CurveAdaptor2D(aGeom2dCurve!, aFirstParam, aLastParam);
        var aGeomAdaptorSurface = new SurfaceAdaptor(aGeomSurface!);

        myAdaptorCurveOnSurface = new CurveOnSurface(anAdaptorCurve, aGeomAdaptorSurface);

        if (Tool.IsClosed(theEdge, theFace))
        {
            Curve2D? aGeom2dReversedCurve =
                Tool.CurveOnSurface(ShapeCasts.Edge(theEdge.Reversed()), theFace, out aFirstParam, out aLastParam);
            Curve2DAdaptorBase anAdaptorReversedCurve = new CurveAdaptor2D(aGeom2dReversedCurve!, aFirstParam, aLastParam);
            myAdaptorCurveOnSurface2 = new CurveOnSurface(anAdaptorReversedCurve, aGeomAdaptorSurface);
        }

        myCOnSurfGeom.Init(anAdaptor3dCurve);
    }

    /// <summary>Computes the max distance, in parallel if <see cref="SetParallel"/> asked for it.</summary>
    public void Perform()
    {
        // Compute the max distance
        Compute(myAdaptorCurveOnSurface);
        if (ErrorStatus() != 0)
        {
            return;
        }

        //
        if (myAdaptorCurveOnSurface2 is not null)
        {
            // compute max distance for myAdaptorCurveOnSurface2
            // (for the second curve on closed surface)
            Compute(myAdaptorCurveOnSurface2);
        }
    }

    /// <summary>True if the max distance has been found.</summary>
    public bool IsDone() => myCOnSurfGeom.ErrorStatus() == 0;

    public void SetParallel(bool theIsParallel) => myIsParallel = theIsParallel;

    public bool IsParallel() => myIsParallel;

    /// <summary>0: OK; 1: null curve, surface or 2D curve; 2: invalid parametric range; 3: error in calculations.</summary>
    public int ErrorStatus() => myCOnSurfGeom.ErrorStatus();

    public double MaxDistance() => myCOnSurfGeom.MaxDistance();

    /// <summary>The parameter at which the distance is maximal.</summary>
    public double MaxParameter() => myCOnSurfGeom.MaxParameter();

    /// <summary>Computes the max distance between the 3D curve and <paramref name="theCurveOnSurface"/>.</summary>
    protected void Compute(CurveOnSurface? theCurveOnSurface)
    {
        myCOnSurfGeom.SetParallel(myIsParallel);
        myCOnSurfGeom.Perform(theCurveOnSurface);
    }
}
