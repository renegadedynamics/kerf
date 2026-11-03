// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtElSS.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtElSS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The extremum distances between two elementary surfaces. OCCT implements only two planes; the other pairs throw
/// <see cref="MethodNotImplementedException"/>.
/// </summary>
/// <remarks>An algorithm object: construct it with the surfaces, or empty and call <c>Perform</c>.</remarks>
public class Extrema_ExtElSS
{
    private bool myDone;
    private bool myIsPar;
    private int myNbExt;
    private NCollection_HArray1<double>? mySqDist;
    private NCollection_HArray1<Extrema_POnSurf>? myPOnS1 = null;
    private NCollection_HArray1<Extrema_POnSurf>? myPOnS2 = null;

    public Extrema_ExtElSS()
    {
        myDone = false;
        myIsPar = false;
        myNbExt = 0;
    }

    /// <summary>The distances between 2 planes. These planes can be parallel.</summary>
    public Extrema_ExtElSS(in Plane3D S1, in Plane3D S2) => Perform(S1, S2);

    public void Perform(in Plane3D S1, in Plane3D S2)
    {
        myDone = true;
        myIsPar = false;
        myNbExt = 0;

        if ((S1.Axis().Direction()).IsParallel(S2.Axis().Direction(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            myIsPar = true;
            myNbExt = 1;
            mySqDist = new NCollection_HArray1<double>(1, 1);
            mySqDist.SetValue(1, S1.SquareDistance(S2));
        }
    }

    /// <summary>The distances between a plane and a sphere (not implemented in OCCT).</summary>
    public Extrema_ExtElSS(in Plane3D S1, in Sphere S2) => Perform(S1, S2);

    public void Perform(in Plane3D S1, in Sphere S2)
    {
        myDone = true;
        myIsPar = false;
        myNbExt = 0;
        throw new MethodNotImplementedException();
    }

    /// <summary>The distances between 2 spheres (not implemented in OCCT).</summary>
    public Extrema_ExtElSS(in Sphere S1, in Sphere S2) => Perform(S1, S2);

    public void Perform(in Sphere S1, in Sphere S2)
    {
        myDone = true;
        myIsPar = false;
        myNbExt = 0;
        throw new MethodNotImplementedException();
    }

    /// <summary>The distances between a sphere and a cylinder (not implemented in OCCT).</summary>
    public Extrema_ExtElSS(in Sphere S1, in Cylinder S2) => Perform(S1, S2);

    public void Perform(in Sphere S1, in Cylinder S2)
    {
        myDone = true;
        myIsPar = false;
        myNbExt = 0;
        throw new MethodNotImplementedException();
    }

    /// <summary>The distances between a sphere and a cone (not implemented in OCCT).</summary>
    public Extrema_ExtElSS(in Sphere S1, in Cone S2) => Perform(S1, S2);

    public void Perform(in Sphere S1, in Cone S2)
    {
        myDone = true;
        myIsPar = false;
        myNbExt = 0;
        throw new MethodNotImplementedException();
    }

    /// <summary>The distances between a sphere and a torus (not implemented in OCCT).</summary>
    public Extrema_ExtElSS(in Sphere S1, in Torus S2) => Perform(S1, S2);

    public void Perform(in Sphere S1, in Torus S2)
    {
        myDone = true;
        myIsPar = false;
        myNbExt = 0;
        throw new MethodNotImplementedException();
    }

    /// <summary>True if the distances are found.</summary>
    public bool IsDone() => myDone;

    /// <summary>True if the two surfaces are parallel.</summary>
    public bool IsParallel()
    {
        if (!IsDone())
        {
            throw new NotDoneException();
        }
        return myIsPar;
    }

    /// <summary>The number of extremum distances.</summary>
    public int NbExt()
    {
        if (!IsDone())
        {
            throw new NotDoneException();
        }
        return myNbExt;
    }

    /// <summary>The value of the Nth extremum square distance.</summary>
    public double SquareDistance(int N = 1)
    {
        if (N < 1 || N > NbExt())
        {
            throw new OutOfRangeException();
        }

        return mySqDist!.Value(N);
    }

    /// <summary>The points for the Nth resulting distance: P1 on the first surface, P2 on the second.</summary>
    /// <exception cref="NullReferenceException">Always for two parallel planes, the only case with a result.</exception>
    public void Points(int N, out Extrema_POnSurf P1, out Extrema_POnSurf P2)
    {
        if (N < 1 || N > NbExt())
        {
            throw new OutOfRangeException();
        }

        // OCCT: no Perform fills myPOnS1 and myPOnS2, so this dereferences null handles
        P1 = myPOnS1!.Value(N);
        P2 = myPOnS2!.Value(N);
    }
}
