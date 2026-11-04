// Port of: src/Visualization/TKV3d/Prs3d/Prs3d_Drawer.hxx (the deflection settings only)
// Port of: src/Visualization/TKV3d/Prs3d/Prs3d_Drawer.cxx (the deflection settings only)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.HiddenLines.Presentation;

/// <summary>
/// The attributes that govern how a shape is meshed for display: the type of deflection (relative to the size of the
/// object or absolute), the deviation coefficient, the maximal chordial deviation, the deviation angle and automatic
/// triangulation. A drawer without its own value of a setting takes its link's, else OCCT's default.
/// </summary>
/// <remarks>
/// OCCT.NET ports only Prs3d_Drawer's deflection settings (unit 4L2), which <see cref="PresentationToolTriangulatedShape"/>
/// reads: the aspects, isolines, display units and shaders draw presentations, and the port has no viewer. A working
/// object (decision 21), as OCCT's: <see cref="PresentationToolTriangulatedShape.GetDeflection"/> stores the absolute
/// deflection it computes into the drawer.
/// </remarks>
internal class Drawer : Transient
{
    private Drawer? _link;
    private double _chordialDeviation;
    private TypeOfDeflection _typeOfDeflection;
    private bool _hasOwnTypeOfDeflection;
    private double _deviationCoefficient;
    private double _previousDeviationCoefficient;
    private double _deviationAngle;
    private double _previousDeviationAngle;
    private bool _isAutoTriangulated;
    private bool _hasOwnIsAutoTriangulated;

    /// <summary>A drawer without its own settings.</summary>
    public Drawer()
    {
        _chordialDeviation = -1.0;
        _typeOfDeflection = global::Kerf.Engine.HiddenLines.Presentation.TypeOfDeflection.TodRelative;
        _hasOwnTypeOfDeflection = false;
        _deviationCoefficient = -1.0;
        _deviationAngle = -1.0;
        _isAutoTriangulated = true;
        _hasOwnIsAutoTriangulated = false;
    }

    /// <summary>Gives the drawer its own default deflection settings.</summary>
    public void SetupOwnDefaults()
    {
        _chordialDeviation = 0.0001;
        _deviationCoefficient = 0.001;
        _deviationAngle = 20.0 * Math.PI / 180.0;
    }

    /// <summary>Sets the type of chordal deflection: absolute, or relative to the size of the object.</summary>
    public void SetTypeOfDeflection(TypeOfDeflection typeOfDeflection)
    {
        _hasOwnTypeOfDeflection = true;
        _typeOfDeflection = typeOfDeflection;
    }

    /// <summary>The type of chordal deflection: absolute, or relative to the size of the object.</summary>
    public TypeOfDeflection TypeOfDeflection()
        => _hasOwnTypeOfDeflection || _link is null ? _typeOfDeflection : _link.TypeOfDeflection();

    /// <summary>True if the drawer has its own type of deflection.</summary>
    public bool HasOwnTypeOfDeflection => _hasOwnTypeOfDeflection;

    /// <summary>Undoes <see cref="SetTypeOfDeflection"/>.</summary>
    public void UnsetOwnTypeOfDeflection()
    {
        _hasOwnTypeOfDeflection = false;
        _typeOfDeflection = global::Kerf.Engine.HiddenLines.Presentation.TypeOfDeflection.TodRelative;
    }

    /// <summary>Defines the maximal chordial deviation (the absolute deflection).</summary>
    public void SetMaximalChordialDeviation(double chordialDeviation) => _chordialDeviation = chordialDeviation;

    /// <summary>The maximal chordal deviation; 0.0001 by default.</summary>
    public double MaximalChordialDeviation()
        => _chordialDeviation > 0.0 ? _chordialDeviation : _link?.MaximalChordialDeviation() ?? 0.0001;

    /// <summary>True if the drawer has its own maximal chordial deviation.</summary>
    public bool HasOwnMaximalChordialDeviation => _chordialDeviation > 0.0;

    /// <summary>Undoes <see cref="SetMaximalChordialDeviation"/>.</summary>
    public void UnsetOwnMaximalChordialDeviation() => _chordialDeviation = -1.0;

    /// <summary>Sets the deviation coefficient (the relative deflection), keeping the previous one.</summary>
    public void SetDeviationCoefficient(double coefficient)
    {
        _previousDeviationCoefficient = DeviationCoefficient();
        _deviationCoefficient = coefficient;
    }

    /// <summary>
    /// The deviation coefficient: with a relative type of deflection, the deflection is the size of the object times
    /// this coefficient. 0.001 by default.
    /// </summary>
    public double DeviationCoefficient()
        => _deviationCoefficient > 0.0 ? _deviationCoefficient : _link?.DeviationCoefficient() ?? 0.001;

    /// <summary>Undoes <see cref="SetDeviationCoefficient(double)"/>.</summary>
    public void SetDeviationCoefficient() => _deviationCoefficient = -1.0;

    /// <summary>True if the drawer has its own deviation coefficient.</summary>
    public bool HasOwnDeviationCoefficient => _deviationCoefficient > 0.0;

    /// <summary>The previous deviation coefficient.</summary>
    public double PreviousDeviationCoefficient() => HasOwnDeviationCoefficient ? _previousDeviationCoefficient : 0.0;

    /// <summary>Makes the current deviation coefficient the previous one.</summary>
    public void UpdatePreviousDeviationCoefficient()
    {
        if (HasOwnDeviationCoefficient)
        {
            _previousDeviationCoefficient = DeviationCoefficient();
        }
    }

    /// <summary>Sets the deviation angle, keeping the previous one.</summary>
    public void SetDeviationAngle(double angle)
    {
        _previousDeviationAngle = DeviationAngle();
        _deviationAngle = angle;
    }

    /// <summary>The deviation angle in radians, 20 * PI / 180 by default.</summary>
    public double DeviationAngle()
        => _deviationAngle > 0.0 ? _deviationAngle : _link?.DeviationAngle() ?? 20.0 * Math.PI / 180.0;

    /// <summary>Undoes <see cref="SetDeviationAngle(double)"/>.</summary>
    public void SetDeviationAngle() => _deviationAngle = -1.0;

    /// <summary>True if the drawer has its own deviation angle.</summary>
    public bool HasOwnDeviationAngle => _deviationAngle > 0.0;

    /// <summary>The previous deviation angle.</summary>
    public double PreviousDeviationAngle() => HasOwnDeviationAngle ? _previousDeviationAngle : 0.0;

    /// <summary>Makes the current deviation angle the previous one.</summary>
    public void UpdatePreviousDeviationAngle()
    {
        if (HasOwnDeviationAngle)
        {
            _previousDeviationAngle = DeviationAngle();
        }
    }

    /// <summary>
    /// Turns automatic triangulation on or off: when on, shapes are meshed for display with the deflection-check
    /// logic; when off, the application meshes them.
    /// </summary>
    public void SetAutoTriangulation(bool isEnabled)
    {
        _hasOwnIsAutoTriangulated = true;
        _isAutoTriangulated = isEnabled;
    }

    /// <summary>True if automatic triangulation is on.</summary>
    public bool IsAutoTriangulation
        => _hasOwnIsAutoTriangulated || _link is null ? _isAutoTriangulated : _link.IsAutoTriangulation;

    /// <summary>True if the drawer has its own automatic triangulation setting.</summary>
    public bool HasOwnIsAutoTriangulation => _hasOwnIsAutoTriangulated;

    /// <summary>Undoes <see cref="SetAutoTriangulation"/>.</summary>
    public void UnsetOwnIsAutoTriangulation()
    {
        _hasOwnIsAutoTriangulated = false;
        _isAutoTriangulated = true;
    }

    /// <summary>The drawer whose settings this one takes where it has none of its own, or null.</summary>
    public Drawer? Link() => _link;

    /// <summary>True if the drawer has a link.</summary>
    public bool HasLink => _link is not null;

    /// <summary>Sets the drawer whose settings this one takes where it has none of its own.</summary>
    public void SetLink(Drawer? drawer) => _link = drawer;
}
