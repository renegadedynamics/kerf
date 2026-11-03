// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeVector.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeVector.cxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeVector_gen.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Exchange.Step.Export.Geometry;

/// <summary>The mapping of vectors (Vector from Geom and Geom2d, Vec and Vec2d from gp) to a STEP vector.</summary>
public sealed class MakeVector : GeometryRoot
{
    private StepVector _vector;

    //=============================================================================
    // Creation d' un vector de prostep a partir d' un Vec de gp
    //=============================================================================

    /// <summary>A 3D vector, its magnitude divided by the length factor.</summary>
    public MakeVector(in Vector3D v, in Factors localFactors)
    {
        Direction3D d = new Direction3D(v);
        double lFactor = localFactors.LengthFactor();
        Make(new MakeDirection(d), v.Magnitude(), lFactor);
    }

    /// <summary>A 3D vector (default factors).</summary>
    public MakeVector(in Vector3D v) : this(v, new Factors())
    {
    }

    //=============================================================================
    // Creation d' un vector de prostep a partir d' un Vec2d de gp
    //=============================================================================

    /// <summary>A 2D vector (the factors aren't used).</summary>
    public MakeVector(in Vector2D v, in Factors localFactors)
    {
        Direction2D d = new Direction2D(v);
        double lFactor = 1.0;
        Make(new MakeDirection(d), v.Magnitude(), lFactor);
    }

    /// <summary>A 2D vector.</summary>
    public MakeVector(in Vector2D v) : this(v, new Factors())
    {
    }

    //=============================================================================
    // Creation d' un vector de prostep a partir d' un Vector de Geom
    //=============================================================================

    /// <summary>A 3D vector, its magnitude divided by the length factor.</summary>
    public MakeVector(GeometryVector3D gVector, in Factors localFactors)
    {
        Vector3D v;
        v = gVector.Vec();
        Direction3D d = new Direction3D(v);
        double lFactor = localFactors.LengthFactor();
        Make(new MakeDirection(d), v.Magnitude(), lFactor);
    }

    /// <summary>A 3D vector (default factors).</summary>
    public MakeVector(GeometryVector3D gVector) : this(gVector, new Factors())
    {
    }

    //=============================================================================
    // Creation d' un vector de prostep a partir d' un Vector de Geom2d
    //=============================================================================

    /// <summary>A 2D vector (the factors aren't used).</summary>
    public MakeVector(GeometryVector2D gVector, in Factors localFactors)
    {
        Vector2D v;
        v = gVector.Vec2d();
        Direction2D d = new Direction2D(v);
        double lFactor = 1.0;
        Make(new MakeDirection(d), v.Magnitude(), lFactor);
    }

    /// <summary>A 2D vector.</summary>
    public MakeVector(GeometryVector2D gVector) : this(gVector, new Factors())
    {
    }

    // GeomToStep_MakeVector_gen.pxx, the body OCCT includes in each constructor (its direction made first)
    private void Make(MakeDirection mkDir, double magnitude, double lFactor)
    {
        StepVector vect = new StepVector();
        StepDirection direction;
        double aMagnitude;
        direction = mkDir.Value();
        aMagnitude = magnitude;
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        vect.Init(name, direction, aMagnitude / lFactor);
        _vector = vect;
        _done = true;
    }

    /// <summary>The vector made.</summary>
    public StepVector Value()
    {
        StdFail_NotDone_Raise_if(!_done, "GeomToStep_MakeVector::Value() - no result");
        return _vector;
    }
}
