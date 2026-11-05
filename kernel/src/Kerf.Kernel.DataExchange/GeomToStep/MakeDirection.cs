// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeDirection.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeDirection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Exchange.Step.Export.Geometry;

/// <summary>The mapping of directions (Direction from Geom and Geom2d, Dir and Dir2d from gp) to a STEP direction.</summary>
internal sealed class MakeDirection : GeometryRoot
{
    private StepDirection _direction;

    //=============================================================================
    // Creation d' une direction de prostep a partir d' une Dir de gp
    //=============================================================================

    /// <summary>A 3D direction.</summary>
    public MakeDirection(in Direction3D d)
    {
        StepDirection dir = new StepDirection();
        NCollection_HArray1<double> dirRatios = new NCollection_HArray1<double>(1, 3);
        d.Coord(out double x, out double y, out double z);
        dirRatios.SetValue(1, x);
        dirRatios.SetValue(2, y);
        dirRatios.SetValue(3, z);
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        dir.Init(name, dirRatios);
        _direction = dir;
        _done = true;
    }

    //=============================================================================
    // Creation d' une direction de prostep a partir d' une Dir2d de gp
    //=============================================================================

    /// <summary>A 2D direction.</summary>
    public MakeDirection(in Direction2D d)
    {
        StepDirection dir = new StepDirection();
        NCollection_HArray1<double> dirRatios = new NCollection_HArray1<double>(1, 2);
        d.Coord(out double x, out double y);
        dirRatios.SetValue(1, x);
        dirRatios.SetValue(2, y);
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        dir.Init(name, dirRatios);
        _direction = dir;
        _done = true;
    }

    //=============================================================================
    // Creation d' une direction de prostep a partir d' une Direction de Geom
    //=============================================================================

    /// <summary>A 3D direction.</summary>
    public MakeDirection(Direction direc)
    {
        Direction3D d;
        StepDirection dir = new StepDirection();
        NCollection_HArray1<double> dirRatios = new NCollection_HArray1<double>(1, 3);
        d = direc.Dir();
        d.Coord(out double x, out double y, out double z);
        dirRatios.SetValue(1, x);
        dirRatios.SetValue(2, y);
        dirRatios.SetValue(3, z);
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        dir.Init(name, dirRatios);
        _direction = dir;
        _done = true;
    }

    //=============================================================================
    // Creation d' une direction de prostep a partir d' une Direction de Geom2d
    //=============================================================================

    /// <summary>A 2D direction.</summary>
    public MakeDirection(Direction2 direc)
    {
        Direction2D d;
        StepDirection dir = new StepDirection();
        NCollection_HArray1<double> dirRatios = new NCollection_HArray1<double>(1, 2);
        d = direc.Dir2d();
        d.Coord(out double x, out double y);
        dirRatios.SetValue(1, x);
        dirRatios.SetValue(2, y);
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        dir.Init(name, dirRatios);
        _direction = dir;
        _done = true;
    }

    /// <summary>The direction made.</summary>
    public StepDirection Value()
    {
        StdFail_NotDone_Raise_if(!_done, "GeomToStep_MakeDirection::Value() - no result");
        return _direction;
    }
}
