// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_SolidClassifier.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_SolidClassifier.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Classification;
using Kerf.Engine.Collections;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Booleans.Legacy.Tools;

/// <summary>
/// Classifies points against solids and shells, keeping one <see cref="SolidClassifier"/> per solid or
/// shell met.
/// </summary>
/// <remarks>A working object. OCCT stores raw pointers to the classifiers and deletes them in its destructor; the port
/// stores references.</remarks>
internal sealed class ToolSolidClassifier
{
    private SolidClassifier? _pClassifier;
    private readonly IndexedMap<Shape, SolidClassifier> _shapeClassifierMap =
        new(ShapeHasher.Instance);
    private Containment _state;
    private Shell _shell = new();
    private Solid _solid = new();
    private readonly BrepBuilder _builder = new();

    /// <summary>An empty classifier.</summary>
    public ToolSolidClassifier()
    {
        Clear();
    }

    /// <summary>Forgets the solids and shells met.</summary>
    public void Clear()
    {
        _pClassifier = null;
        // modified by NIZNHY-PKV Mon Dec 16 10:46:04 2002 f
        _shapeClassifierMap.Clear();
        // modified by NIZNHY-PKV Mon Dec 16 10:46:06 2002 t
        _state = Unknown;
        _shell.Nullify();
        _solid.Nullify();
    }

    /// <summary>Makes <paramref name="sol"/>'s classifier current, creating it on first use.</summary>
    public void LoadSolid(in Solid sol)
    {
        bool found = _shapeClassifierMap.ContainsKey(sol);
        if (!found)
        {
            _pClassifier = new SolidClassifier(sol);
            _shapeClassifierMap.Add(sol, _pClassifier);
        }
        else
        {
            _pClassifier = _shapeClassifierMap.GetValue(sol);
        }
    }

    /// <summary>
    /// The state of <paramref name="p"/> against the solid <paramref name="sol"/>, with the state of an EXTERNAL or
    /// INTERNAL face found by the classifier taken into account.
    /// </summary>
    public Containment Classify(in Solid sol, in Point3D p, double tol)
    {
        _pClassifier = null;
        _state = Unknown;
        LoadSolid(sol);
        if (_pClassifier is null)
        {
            return _state;
        }
        _pClassifier.Perform(p, tol);
        _state = _pClassifier.State();
        Shape fres = _pClassifier.Face();
        if (fres.IsNull)
        {
            // NYI: in case of elimination of EXTERNAL and INTERNAL faces by the
            // BRepClass3d_SolidClassifier classifier, still handle these faces
            // to generate ON/Solid state when the point is IN/face INTERNAL or EXTERNAL
            return _state;
        }
        Orientation ofres = fres.Orientation;
        if (ofres == External)
        {
            if (_state == In)
            {
                _state = Out;
            }
            else if (_state == Out)
            {
                _state = Out;
            }
            else if (_state == On)
            {
                _state = On;
            }
            else if (_state == Unknown)
            {
                _state = Out;
            }
        }
        else if (ofres == Internal)
        {
            if (_state == In)
            {
                _state = In;
            }
            else if (_state == Out)
            {
                _state = In;
            }
            else if (_state == On)
            {
                _state = On;
            }
            else if (_state == Unknown)
            {
                _state = In;
            }
        }
        return _state;
    }

    /// <summary>
    /// Makes <paramref name="she"/>'s classifier current, creating it on first use on a solid made of the shell.
    /// </summary>
    public void LoadShell(in Shell she)
    {
        bool found = _shapeClassifierMap.ContainsKey(she);
        if (!found)
        {
            _builder.MakeSolid(ref _solid);
            _builder.Add(_solid, she);
            _pClassifier = new SolidClassifier(_solid);
            _shapeClassifierMap.Add(she, _pClassifier);
        }
        else
        {
            _pClassifier = _shapeClassifierMap.GetValue(she);
        }
    }

    /// <summary>The state of <paramref name="p"/> against the solid bounded by the shell <paramref name="she"/>.</summary>
    public Containment Classify(in Shell she, in Point3D p, double tol)
    {
        _pClassifier = null;
        _state = Unknown;
        LoadShell(she);
        if (_pClassifier is null)
        {
            return _state;
        }
        _pClassifier.Perform(p, tol);
        _state = _pClassifier.State();
        return _state;
    }

    /// <summary>The state found by the last classification.</summary>
    public Containment State() => _state;
}
