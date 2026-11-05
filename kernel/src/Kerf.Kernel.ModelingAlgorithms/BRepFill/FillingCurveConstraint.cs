// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_CurveConstraint.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_CurveConstraint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Plate;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Properties;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Filling;

/// <summary>
/// A curve constraint of a filling: GeomPlate's, also accepting a curve on a <see cref="FaceSurfaceAdaptor"/> (an
/// edge on a face).
/// </summary>
/// <remarks>A working object, as <see cref="CurveConstraint"/>.</remarks>
internal class FillingCurveConstraint : CurveConstraint
{
    /// <summary>A constraint of order <paramref name="tang"/> (-1 to 2) along a curve on a surface.</summary>
    /// <exception cref="FailureException">The order isn't -1 to 2, or <paramref name="boundary"/> is null.</exception>
    public FillingCurveConstraint(CurveOnSurface? boundary, int tang, int nPt = 10, double tolDist = 0.0001,
                                    double tolAng = 0.01, double tolCurv = 0.1)
    {
        _frontiere = boundary;
        _tolDist = tolDist;
        _tolAng = tolAng;
        _tolCurv = tolCurv;
        SLProps slp = new SLProps(2, tolDist);
        _lProp = slp;
        _order = tang;
        if ((tang < -1) || (tang > 2))
        {
            throw new FailureException("BRepFill : The continuity is not G0 G1 or G2");
        }
        _nbPoints = nPt;
        _constG0 = true;
        _constG1 = true;
        _constG2 = true;
        if (_frontiere is null)
        {
            throw new FailureException("BRepFill_CurveConstraint : Curve must be on a Surface");
        }
        Surface? surf;
        SurfaceAdaptor? gs1;
        gs1 = _frontiere.GetSurface() as SurfaceAdaptor;
        if (gs1 is not null)
        {
            surf = gs1.Surface();
        }
        else
        {
            FaceSurfaceAdaptor? bs1;
            bs1 = _frontiere.GetSurface() as FaceSurfaceAdaptor;
            surf = Tool.Surface(bs1!.Face());
        }
        _lProp.SetSurface(surf!);
    }

    /// <summary>A constraint of order -1 or 0 along a 3D curve.</summary>
    public FillingCurveConstraint(CurveAdaptorBase boundary, int tang, int nPt = 10, double tolDist = 0.0001)
        : base(boundary, tang, nPt, tolDist)
    {
    }
}
