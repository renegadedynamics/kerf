// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeAxis2Placement3d.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeAxis2Placement3d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Export.Geometry;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Exchange.Step.FromGeometry;

/// <summary>
/// The mapping of Axis2Placement from Geom (and Ax2, Ax3 and Trsf from gp) to a STEP axis2_placement_3d.
/// </summary>
public sealed class Axis2Placement3dMaker : GeometryRoot
{
    private StepAxis2Placement3d _axis2Placement3d;

    private static StepAxis2Placement3d MakeAxis2Placement3d(in Point3D o, in Direction3D d, in Direction3D x, string nom, double aFactor)
    {
        StepAxis2Placement3d axe;
        StepCartesianPoint p;
        StepDirection d1, d2;

        MakeCartesianPoint mkPoint = new MakeCartesianPoint(o, aFactor);
        MakeDirection mkDir1 = new MakeDirection(d);
        MakeDirection mkDir2 = new MakeDirection(x);

        p = mkPoint.Value();
        d1 = mkDir1.Value();
        d2 = mkDir2.Value();

        axe = new StepAxis2Placement3d();
        axe.SetLocation(p);
        axe.SetAxis(d1);
        axe.SetRefDirection(d2);
        TCollection_HAsciiString name = new TCollection_HAsciiString(nom);
        axe.SetName(name);
        return axe;
    }

    //=============================================================================
    // Creation d' un axis2_placement_3d a l origine
    //=============================================================================

    /// <summary>The global coordinate system.</summary>
    public Axis2Placement3dMaker(in Factors localFactors)
    {
        Frame3D a = new Frame3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X));
        //   le reste inchange

        StepAxis2Placement3d axe = MakeAxis2Placement3d(a.Location, a.Direction, a.XDirection, "", localFactors.LengthFactor());
        _axis2Placement3d = axe;
        _done = true;
    }

    /// <summary>The global coordinate system (default factors).</summary>
    public Axis2Placement3dMaker() : this(new Factors())
    {
    }

    //=============================================================================
    // Creation d' un axis2_placement_3d de prostep a partir d' un Ax2 de gp
    //=============================================================================

    /// <summary>An Ax2, its location divided by the length factor.</summary>
    public Axis2Placement3dMaker(in Frame3D a, in Factors localFactors)
    {
        StepAxis2Placement3d axe = MakeAxis2Placement3d(a.Location, a.Direction, a.XDirection, "", localFactors.LengthFactor());
        _axis2Placement3d = axe;
        _done = true;
    }

    /// <summary>An Ax2 (default factors).</summary>
    public Axis2Placement3dMaker(in Frame3D a) : this(a, new Factors())
    {
    }

    //=============================================================================
    // Creation d' un axis2_placement_3d de prostep a partir d' un Ax3 de gp
    //=============================================================================

    /// <summary>An Ax3 (direct or not), its location divided by the length factor.</summary>
    public Axis2Placement3dMaker(in CoordinateSystem a, in Factors localFactors)
    {
        StepAxis2Placement3d axe = MakeAxis2Placement3d(a.Location, a.Direction, a.XDirection, "", localFactors.LengthFactor());
        _axis2Placement3d = axe;
        _done = true;
    }

    /// <summary>An Ax3 (default factors).</summary>
    public Axis2Placement3dMaker(in CoordinateSystem a) : this(a, new Factors())
    {
    }

    //=============================================================================
    // Creation d' un axis2_placement_3d de prostep a partir d' un Trsf de gp
    //=============================================================================

    /// <summary>The global coordinate system transformed by <paramref name="t"/>.</summary>
    public Axis2Placement3dMaker(in Transform3D t, in Factors localFactors)
    {
        Frame3D a = new Frame3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X));
        a = a.Transformed(t);
        //   le reste inchange

        StepAxis2Placement3d axe = MakeAxis2Placement3d(a.Location, a.Direction, a.XDirection, "", localFactors.LengthFactor());
        _axis2Placement3d = axe;
        _done = true;
    }

    /// <summary>The global coordinate system transformed (default factors).</summary>
    public Axis2Placement3dMaker(in Transform3D t) : this(t, new Factors())
    {
    }

    //=============================================================================
    // Creation d' un axis2_placement_3d de prostep a partir d' un Axis2Placement
    // de Geom
    //=============================================================================

    /// <summary>An Axis2Placement, its location divided by the length factor.</summary>
    public Axis2Placement3dMaker(Axis2Placement axis2, in Factors localFactors)
    {
        Frame3D a;
        a = axis2.Ax2();

        StepAxis2Placement3d axe = MakeAxis2Placement3d(a.Location, a.Direction, a.XDirection, "", localFactors.LengthFactor());
        _axis2Placement3d = axe;
        _done = true;
    }

    /// <summary>An Axis2Placement (default factors).</summary>
    public Axis2Placement3dMaker(Axis2Placement axis2) : this(axis2, new Factors())
    {
    }

    /// <summary>The axis2_placement_3d made.</summary>
    public StepAxis2Placement3d Value()
    {
        StdFail_NotDone_Raise_if(!_done, "GeomToStep_MakeAxis2Placement3d::Value() - no result");
        return _axis2Placement3d;
    }
}
