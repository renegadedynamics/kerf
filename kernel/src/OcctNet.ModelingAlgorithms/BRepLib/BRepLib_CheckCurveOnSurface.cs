// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_CheckCurveOnSurface.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_CheckCurveOnSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Computes the max distance between an edge's 3D curve and its 2D representation on a face (both pcurves on a
/// closed surface). Not intended for edges that aren't SameParameter.
/// </summary>
/// <remarks>A working object over <see cref="GeomLib_CheckCurveOnSurface"/> (decision 293).</remarks>
public class BRepLib_CheckCurveOnSurface
{
    private readonly GeomLib_CheckCurveOnSurface myCOnSurfGeom = new();
    private Adaptor3d_CurveOnSurface? myAdaptorCurveOnSurface;
    private Adaptor3d_CurveOnSurface? myAdaptorCurveOnSurface2;
    private bool myIsParallel;

    /// <summary>An empty algorithm; <see cref="Init"/> sets its data.</summary>
    public BRepLib_CheckCurveOnSurface() => myIsParallel = false;

    public BRepLib_CheckCurveOnSurface(in TopoDS_Edge theEdge, in TopoDS_Face theFace)
    {
        myIsParallel = false;
        Init(theEdge, theFace);
    }

    /// <summary>Sets the data for the algorithm; nothing for a null, degenerated or non-geometric edge.</summary>
    public void Init(in TopoDS_Edge theEdge, in TopoDS_Face theFace)
    {
        myCOnSurfGeom.Init();

        if (theEdge.IsNull() || theFace.IsNull())
        {
            return;
        }

        //
        if (BRep_Tool.Degenerated(theEdge) || !BRep_Tool.IsGeometric(theEdge))
        {
            return;
        }

        // 3D curve initialization
        // TODO(3B): new BRepAdaptor_Curve(theEdge)
        Adaptor3d_Curve anAdaptor3dCurve = BRepAdaptorCurve(theEdge);

        // Surface initialization
        Geom2d_Curve? aGeom2dCurve = BRep_Tool.CurveOnSurface(theEdge, theFace, out double aFirstParam, out double aLastParam);
        Geom_Surface? aGeomSurface = BRep_Tool.Surface(theFace);

        // 2D curves initialization
        Adaptor2d_Curve2d anAdaptorCurve = new Geom2dAdaptor_Curve(aGeom2dCurve!, aFirstParam, aLastParam);
        var aGeomAdaptorSurface = new GeomAdaptor_Surface(aGeomSurface!);

        myAdaptorCurveOnSurface = new Adaptor3d_CurveOnSurface(anAdaptorCurve, aGeomAdaptorSurface);

        if (BRep_Tool.IsClosed(theEdge, theFace))
        {
            Geom2d_Curve? aGeom2dReversedCurve =
                BRep_Tool.CurveOnSurface(TopoDS.Edge(theEdge.Reversed()), theFace, out aFirstParam, out aLastParam);
            Adaptor2d_Curve2d anAdaptorReversedCurve = new Geom2dAdaptor_Curve(aGeom2dReversedCurve!, aFirstParam, aLastParam);
            myAdaptorCurveOnSurface2 = new Adaptor3d_CurveOnSurface(anAdaptorReversedCurve, aGeomAdaptorSurface);
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
    protected void Compute(Adaptor3d_CurveOnSurface? theCurveOnSurface)
    {
        myCOnSurfGeom.SetParallel(myIsParallel);
        myCOnSurfGeom.Perform(theCurveOnSurface);
    }

    /// <summary>
    /// Stand-in for <c>new BRepAdaptor_Curve(E)</c> until unit 3B ports BRepAdaptor_Curve: its base class,
    /// initialised as <c>BRepAdaptor_Curve::Initialize(E)</c> does.
    /// </summary>
    // TODO(3B): replace by BRepAdaptor_Curve
    internal static GeomAdaptor_TransformedCurve BRepAdaptorCurve(in TopoDS_Edge E)
    {
        var aCurve = new GeomAdaptor_TransformedCurve();
        TopLoc_Location L;
        Geom_Curve? C = BRep_Tool.Curve(E, out L, out double pf, out double pl);
        if (C is not null)
        {
            aCurve.Load(C, pf, pl);
        }
        else
        {
            BRep_Tool.CurveOnSurface(E, out Geom2d_Curve? PC, out Geom_Surface? S, out L, out pf, out pl);
            if (PC is not null)
            {
                var HS = new GeomAdaptor_Surface();
                HS.Load(S!);
                var HC = new Geom2dAdaptor_Curve();
                HC.Load(PC, pf, pl);
                var aConSurf = new Adaptor3d_CurveOnSurface();
                aConSurf.Load(HC, HS);
                aCurve.LoadCurveOnSurface(aConSurf);
            }
            else
            {
                throw new Standard_NullObject("BRepAdaptor_Curve::No geometry");
            }
        }

        aCurve.SetTrsf(L.Transformation());
        return aCurve;
    }
}
