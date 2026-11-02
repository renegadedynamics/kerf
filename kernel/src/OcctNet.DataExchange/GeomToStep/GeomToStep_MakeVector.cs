// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeVector.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeVector.cxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeVector_gen.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>The mapping of vectors (Vector from Geom and Geom2d, Vec and Vec2d from gp) to a STEP vector.</summary>
public sealed class GeomToStep_MakeVector : GeomToStep_Root
{
    private StepGeom_Vector theVector;

    //=============================================================================
    // Creation d' un vector de prostep a partir d' un Vec de gp
    //=============================================================================

    /// <summary>A 3D vector, its magnitude divided by the length factor.</summary>
    public GeomToStep_MakeVector(in gp_Vec V, in StepData_Factors theLocalFactors)
    {
        gp_Dir D = new gp_Dir(V);
        double lFactor = theLocalFactors.LengthFactor();
        Make(new GeomToStep_MakeDirection(D), V.Magnitude(), lFactor);
    }

    /// <summary>A 3D vector (default factors).</summary>
    public GeomToStep_MakeVector(in gp_Vec V) : this(V, new StepData_Factors())
    {
    }

    //=============================================================================
    // Creation d' un vector de prostep a partir d' un Vec2d de gp
    //=============================================================================

    /// <summary>A 2D vector (the factors aren't used).</summary>
    public GeomToStep_MakeVector(in gp_Vec2d V, in StepData_Factors theLocalFactors)
    {
        gp_Dir2d D = new gp_Dir2d(V);
        double lFactor = 1.0;
        Make(new GeomToStep_MakeDirection(D), V.Magnitude(), lFactor);
    }

    /// <summary>A 2D vector.</summary>
    public GeomToStep_MakeVector(in gp_Vec2d V) : this(V, new StepData_Factors())
    {
    }

    //=============================================================================
    // Creation d' un vector de prostep a partir d' un Vector de Geom
    //=============================================================================

    /// <summary>A 3D vector, its magnitude divided by the length factor.</summary>
    public GeomToStep_MakeVector(Geom_Vector GVector, in StepData_Factors theLocalFactors)
    {
        gp_Vec V;
        V = GVector.Vec();
        gp_Dir D = new gp_Dir(V);
        double lFactor = theLocalFactors.LengthFactor();
        Make(new GeomToStep_MakeDirection(D), V.Magnitude(), lFactor);
    }

    /// <summary>A 3D vector (default factors).</summary>
    public GeomToStep_MakeVector(Geom_Vector GVector) : this(GVector, new StepData_Factors())
    {
    }

    //=============================================================================
    // Creation d' un vector de prostep a partir d' un Vector de Geom2d
    //=============================================================================

    /// <summary>A 2D vector (the factors aren't used).</summary>
    public GeomToStep_MakeVector(Geom2d_Vector GVector, in StepData_Factors theLocalFactors)
    {
        gp_Vec2d V;
        V = GVector.Vec2d();
        gp_Dir2d D = new gp_Dir2d(V);
        double lFactor = 1.0;
        Make(new GeomToStep_MakeDirection(D), V.Magnitude(), lFactor);
    }

    /// <summary>A 2D vector.</summary>
    public GeomToStep_MakeVector(Geom2d_Vector GVector) : this(GVector, new StepData_Factors())
    {
    }

    // GeomToStep_MakeVector_gen.pxx, the body OCCT includes in each constructor (its direction made first)
    private void Make(GeomToStep_MakeDirection MkDir, double theMagnitude, double lFactor)
    {
        StepGeom_Vector Vect = new StepGeom_Vector();
        StepGeom_Direction aDirection;
        double aMagnitude;
        aDirection = MkDir.Value();
        aMagnitude = theMagnitude;
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        Vect.Init(name, aDirection, aMagnitude / lFactor);
        theVector = Vect;
        done = true;
    }

    /// <summary>The vector made.</summary>
    public StepGeom_Vector Value()
    {
        StdFail_NotDone_Raise_if(!done, "GeomToStep_MakeVector::Value() - no result");
        return theVector;
    }
}
