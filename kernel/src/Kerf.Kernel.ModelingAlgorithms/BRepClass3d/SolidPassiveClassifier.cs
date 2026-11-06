// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass3d/BRepClass3d_SolidPassiveClassifier.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass3d/BRepClass3d_SolidPassiveClassifier.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Classification;

/// <summary>
/// A 3D classifier of a point against faces given one by one: the state of the origin of a line from its nearest
/// intersection with them.
/// </summary>
/// <remarks>A working object (decision 21).</remarks>
internal class SolidPassiveClassifier
{
    private bool _isSet;
    private Face _face = new();
    private Line3D _lin = new();
    private double _param;
    private double _tolerance;
    private Containment _state;
    private bool _hasIntersect;
    private readonly Intersector3d _intersector = new();

    /// <summary>An undefined classifier.</summary>
    public SolidPassiveClassifier()
    {
        _isSet = false;
        _param = 0.0;
        _tolerance = 0.0;
        _state = Unknown;
        _hasIntersect = false;
    }

    /// <summary>
    /// Starts a classification of the origin of <paramref name="l"/>, with the tolerance <paramref name="tol"/> of
    /// the intersections. <paramref name="p"/> is unused (OCCT's).
    /// </summary>
    public void Reset(in Line3D l, double p, double tol)
    {
        _lin = l;
        _param = RealLast();
        _tolerance = tol;
        _state = Unknown;
        _isSet = true;
    }

    /// <summary>Updates the classification with the face <paramref name="face"/> of the boundary (the orientation is unused).</summary>
    public void Compare(in Face face, Orientation or)
    {
        if (!_isSet)
        {
            return;
        }

        _hasIntersect = false;
        _intersector.Perform(_lin, _param, _tolerance, face);
        if (_intersector.IsDone)
        {
            if (_intersector.HasAPoint)
            {
                _hasIntersect = true;
                if (_intersector.WParameter() < _param)
                {
                    _param = _intersector.WParameter();
                    _face = _intersector.Face();
                    if (Math.Abs(_param) <= _tolerance)
                    {
                        _state = On;
                    }
                    else
                    {
                        //-- The intersection point between the line and a face F of the solid
                        //-- is in the face F or On a boundary of the face
                        if (_intersector.Transition() == TransitionOnCurve.Out)
                        {
                            //-- The line is going from inside the solid to outside the solid.
                            _state = global::Kerf.Topology.Containment.In;
                        }
                        else if (_intersector.Transition() == TransitionOnCurve.In)
                        {
                            _state = global::Kerf.Topology.Containment.Out;
                        }
                    }
                }
                else
                {
                    //-- No point has been found by the myIntersector.
                    //-- Or a Point has been found with a greater parameter.
                }
            } //-- myIntersector Has a point
            else
            {
                //-- The myIntersector failed.
            }
        } //-- Face has not been rejected
    }

    /// <summary>The current value of the parameter.</summary>
    public double Parameter() => _param;

    /// <summary>True if the last compared face gave an intersection.</summary>
    public bool HasIntersection => _hasIntersect;

    /// <summary>The intersector (the classifier's own object).</summary>
    public Intersector3d Intersector() => _intersector;

    /// <summary>The current state of the point.</summary>
    public Containment State() => _state;

    /// <summary>The face of the nearest intersection (OCCT stores it but has no accessor).</summary>
    internal Face NearestFace() => _face;
}
