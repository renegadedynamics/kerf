// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeAxis2Placement3d.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeAxis2Placement3d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The mapping of Axis2Placement from Geom (and Ax2, Ax3 and Trsf from gp) to a STEP axis2_placement_3d.
/// </summary>
public sealed class GeomToStep_MakeAxis2Placement3d : GeomToStep_Root
{
    private StepGeom_Axis2Placement3d theAxis2Placement3d;

    private static StepGeom_Axis2Placement3d MakeAxis2Placement3d(in Point3D O, in Direction3D D, in Direction3D X, string nom, double aFactor)
    {
        StepGeom_Axis2Placement3d Axe;
        StepGeom_CartesianPoint P;
        StepGeom_Direction D1, D2;

        GeomToStep_MakeCartesianPoint MkPoint = new GeomToStep_MakeCartesianPoint(O, aFactor);
        GeomToStep_MakeDirection MkDir1 = new GeomToStep_MakeDirection(D);
        GeomToStep_MakeDirection MkDir2 = new GeomToStep_MakeDirection(X);

        P = MkPoint.Value();
        D1 = MkDir1.Value();
        D2 = MkDir2.Value();

        Axe = new StepGeom_Axis2Placement3d();
        Axe.SetLocation(P);
        Axe.SetAxis(D1);
        Axe.SetRefDirection(D2);
        TCollection_HAsciiString name = new TCollection_HAsciiString(nom);
        Axe.SetName(name);
        return Axe;
    }

    //=============================================================================
    // Creation d' un axis2_placement_3d a l origine
    //=============================================================================

    /// <summary>The global coordinate system.</summary>
    public GeomToStep_MakeAxis2Placement3d(in StepData_Factors theLocalFactors)
    {
        Frame3D A = new Frame3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X));
        //   le reste inchange

        StepGeom_Axis2Placement3d Axe = MakeAxis2Placement3d(A.Location(), A.Direction(), A.XDirection(), "", theLocalFactors.LengthFactor());
        theAxis2Placement3d = Axe;
        done = true;
    }

    /// <summary>The global coordinate system (default factors).</summary>
    public GeomToStep_MakeAxis2Placement3d() : this(new StepData_Factors())
    {
    }

    //=============================================================================
    // Creation d' un axis2_placement_3d de prostep a partir d' un Ax2 de gp
    //=============================================================================

    /// <summary>An Ax2, its location divided by the length factor.</summary>
    public GeomToStep_MakeAxis2Placement3d(in Frame3D A, in StepData_Factors theLocalFactors)
    {
        StepGeom_Axis2Placement3d Axe = MakeAxis2Placement3d(A.Location(), A.Direction(), A.XDirection(), "", theLocalFactors.LengthFactor());
        theAxis2Placement3d = Axe;
        done = true;
    }

    /// <summary>An Ax2 (default factors).</summary>
    public GeomToStep_MakeAxis2Placement3d(in Frame3D A) : this(A, new StepData_Factors())
    {
    }

    //=============================================================================
    // Creation d' un axis2_placement_3d de prostep a partir d' un Ax3 de gp
    //=============================================================================

    /// <summary>An Ax3 (direct or not), its location divided by the length factor.</summary>
    public GeomToStep_MakeAxis2Placement3d(in CoordinateSystem A, in StepData_Factors theLocalFactors)
    {
        StepGeom_Axis2Placement3d Axe = MakeAxis2Placement3d(A.Location(), A.Direction(), A.XDirection(), "", theLocalFactors.LengthFactor());
        theAxis2Placement3d = Axe;
        done = true;
    }

    /// <summary>An Ax3 (default factors).</summary>
    public GeomToStep_MakeAxis2Placement3d(in CoordinateSystem A) : this(A, new StepData_Factors())
    {
    }

    //=============================================================================
    // Creation d' un axis2_placement_3d de prostep a partir d' un Trsf de gp
    //=============================================================================

    /// <summary>The global coordinate system transformed by <paramref name="T"/>.</summary>
    public GeomToStep_MakeAxis2Placement3d(in Transform3D T, in StepData_Factors theLocalFactors)
    {
        Frame3D A = new Frame3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X));
        A.Transform(T);
        //   le reste inchange

        StepGeom_Axis2Placement3d Axe = MakeAxis2Placement3d(A.Location(), A.Direction(), A.XDirection(), "", theLocalFactors.LengthFactor());
        theAxis2Placement3d = Axe;
        done = true;
    }

    /// <summary>The global coordinate system transformed (default factors).</summary>
    public GeomToStep_MakeAxis2Placement3d(in Transform3D T) : this(T, new StepData_Factors())
    {
    }

    //=============================================================================
    // Creation d' un axis2_placement_3d de prostep a partir d' un Axis2Placement
    // de Geom
    //=============================================================================

    /// <summary>An Axis2Placement, its location divided by the length factor.</summary>
    public GeomToStep_MakeAxis2Placement3d(Axis2Placement Axis2, in StepData_Factors theLocalFactors)
    {
        Frame3D A;
        A = Axis2.Ax2();

        StepGeom_Axis2Placement3d Axe = MakeAxis2Placement3d(A.Location(), A.Direction(), A.XDirection(), "", theLocalFactors.LengthFactor());
        theAxis2Placement3d = Axe;
        done = true;
    }

    /// <summary>An Axis2Placement (default factors).</summary>
    public GeomToStep_MakeAxis2Placement3d(Axis2Placement Axis2) : this(Axis2, new StepData_Factors())
    {
    }

    /// <summary>The axis2_placement_3d made.</summary>
    public StepGeom_Axis2Placement3d Value()
    {
        StdFail_NotDone_Raise_if(!done, "GeomToStep_MakeAxis2Placement3d::Value() - no result");
        return theAxis2Placement3d;
    }
}
