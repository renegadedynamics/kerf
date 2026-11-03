// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_CurveConstraint.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_CurveConstraint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Properties;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;

namespace Occt;

/// <summary>
/// A curve constraint of a filling: GeomPlate's, also accepting a curve on a <see cref="FaceSurfaceAdaptor"/> (an
/// edge on a face).
/// </summary>
/// <remarks>A working object, as <see cref="GeomPlate_CurveConstraint"/>.</remarks>
public class BRepFill_CurveConstraint : GeomPlate_CurveConstraint
{
    /// <summary>A constraint of order <paramref name="Tang"/> (-1 to 2) along a curve on a surface.</summary>
    /// <exception cref="FailureException">The order isn't -1 to 2, or <paramref name="Boundary"/> is null.</exception>
    public BRepFill_CurveConstraint(CurveOnSurface? Boundary, int Tang, int NPt = 10, double TolDist = 0.0001,
                                    double TolAng = 0.01, double TolCurv = 0.1)
    {
        myFrontiere = Boundary;
        myTolDist = TolDist;
        myTolAng = TolAng;
        myTolCurv = TolCurv;
        SLProps SLP = new SLProps(2, TolDist);
        myLProp = SLP;
        myOrder = Tang;
        if ((Tang < -1) || (Tang > 2))
        {
            throw new FailureException("BRepFill : The continuity is not G0 G1 or G2");
        }
        myNbPoints = NPt;
        myConstG0 = true;
        myConstG1 = true;
        myConstG2 = true;
        if (myFrontiere is null)
        {
            throw new FailureException("BRepFill_CurveConstraint : Curve must be on a Surface");
        }
        Surface? Surf;
        SurfaceAdaptor? GS1;
        GS1 = myFrontiere.GetSurface() as SurfaceAdaptor;
        if (GS1 is not null)
        {
            Surf = GS1.Surface();
        }
        else
        {
            FaceSurfaceAdaptor? BS1;
            BS1 = myFrontiere.GetSurface() as FaceSurfaceAdaptor;
            Surf = Tool.Surface(BS1!.Face());
        }
        myLProp.SetSurface(Surf!);
    }

    /// <summary>A constraint of order -1 or 0 along a 3D curve.</summary>
    public BRepFill_CurveConstraint(CurveAdaptorBase Boundary, int Tang, int NPt = 10, double TolDist = 0.0001)
        : base(Boundary, Tang, NPt, TolDist)
    {
    }
}
