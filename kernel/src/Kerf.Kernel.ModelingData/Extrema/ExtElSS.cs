// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtElSS.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtElSS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Extrema;

/// <summary>
/// The extremum distances between two elementary surfaces. OCCT implements only two planes; the other pairs throw
/// <see cref="MethodNotImplementedException"/>.
/// </summary>
/// <remarks>An algorithm object: construct it with the surfaces, or empty and call <c>Perform</c>.</remarks>
internal class ExtElSS
{
    private bool _done;
    private bool _isPar;
    private int _nbExt;
    private NCollection_HArray1<double>? _sqDist;
    private NCollection_HArray1<POnSurf>? _pOnS1 = null;
    private NCollection_HArray1<POnSurf>? _pOnS2 = null;

    public ExtElSS()
    {
        _done = false;
        _isPar = false;
        _nbExt = 0;
    }

    /// <summary>The distances between 2 planes. These planes can be parallel.</summary>
    public ExtElSS(in Plane3D s1, in Plane3D s2) => Perform(s1, s2);

    public void Perform(in Plane3D s1, in Plane3D s2)
    {
        _done = true;
        _isPar = false;
        _nbExt = 0;

        if ((s1.Axis.Direction).IsParallel(s2.Axis.Direction, global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            _isPar = true;
            _nbExt = 1;
            _sqDist = new NCollection_HArray1<double>(1, 1);
            _sqDist.SetValue(1, s1.SquareDistanceTo(s2));
        }
    }

    /// <summary>The distances between a plane and a sphere (not implemented in OCCT).</summary>
    public ExtElSS(in Plane3D s1, in Sphere3D s2) => Perform(s1, s2);

    public void Perform(in Plane3D s1, in Sphere3D s2)
    {
        _done = true;
        _isPar = false;
        _nbExt = 0;
        throw new MethodNotImplementedException();
    }

    /// <summary>The distances between 2 spheres (not implemented in OCCT).</summary>
    public ExtElSS(in Sphere3D s1, in Sphere3D s2) => Perform(s1, s2);

    public void Perform(in Sphere3D s1, in Sphere3D s2)
    {
        _done = true;
        _isPar = false;
        _nbExt = 0;
        throw new MethodNotImplementedException();
    }

    /// <summary>The distances between a sphere and a cylinder (not implemented in OCCT).</summary>
    public ExtElSS(in Sphere3D s1, in Cylinder3D s2) => Perform(s1, s2);

    public void Perform(in Sphere3D s1, in Cylinder3D s2)
    {
        _done = true;
        _isPar = false;
        _nbExt = 0;
        throw new MethodNotImplementedException();
    }

    /// <summary>The distances between a sphere and a cone (not implemented in OCCT).</summary>
    public ExtElSS(in Sphere3D s1, in Cone3D s2) => Perform(s1, s2);

    public void Perform(in Sphere3D s1, in Cone3D s2)
    {
        _done = true;
        _isPar = false;
        _nbExt = 0;
        throw new MethodNotImplementedException();
    }

    /// <summary>The distances between a sphere and a torus (not implemented in OCCT).</summary>
    public ExtElSS(in Sphere3D s1, in Torus3D s2) => Perform(s1, s2);

    public void Perform(in Sphere3D s1, in Torus3D s2)
    {
        _done = true;
        _isPar = false;
        _nbExt = 0;
        throw new MethodNotImplementedException();
    }

    /// <summary>True if the distances are found.</summary>
    public bool IsDone => _done;

    /// <summary>True if the two surfaces are parallel.</summary>
    public bool IsParallel
    {
        get
        {
            if (!IsDone)
            {
                throw new NotDoneException();
            }
            return _isPar;
        }
    }

    /// <summary>The number of extremum distances.</summary>
    public int ExtCount
    {
        get
        {
            if (!IsDone)
            {
                throw new NotDoneException();
            }
            return _nbExt;
        }
    }

    /// <summary>The value of the Nth extremum square distance.</summary>
    public double SquareDistance(int n = 1)
    {
        if (n < 1 || n > ExtCount)
        {
            throw new OutOfRangeException();
        }

        return _sqDist!.Value(n);
    }

    /// <summary>The points for the Nth resulting distance: P1 on the first surface, P2 on the second.</summary>
    /// <exception cref="NullReferenceException">Always for two parallel planes, the only case with a result.</exception>
    public void Points(int n, out POnSurf p1, out POnSurf p2)
    {
        if (n < 1 || n > ExtCount)
        {
            throw new OutOfRangeException();
        }

        // OCCT: no Perform fills myPOnS1 and myPOnS2, so this dereferences null handles
        p1 = _pOnS1!.Value(n);
        p2 = _pOnS2!.Value(n);
    }
}
