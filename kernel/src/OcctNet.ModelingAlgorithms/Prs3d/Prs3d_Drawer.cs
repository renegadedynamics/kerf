// Port of: src/Visualization/TKV3d/Prs3d/Prs3d_Drawer.hxx (the deflection settings only)
// Port of: src/Visualization/TKV3d/Prs3d/Prs3d_Drawer.cxx (the deflection settings only)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// The attributes that govern how a shape is meshed for display: the type of deflection (relative to the size of the
/// object or absolute), the deviation coefficient, the maximal chordial deviation, the deviation angle and automatic
/// triangulation. A drawer without its own value of a setting takes its link's, else OCCT's default.
/// </summary>
/// <remarks>
/// OCCT.NET ports only Prs3d_Drawer's deflection settings (unit 4L2), which <see cref="StdPrs_ToolTriangulatedShape"/>
/// reads: the aspects, isolines, display units and shaders draw presentations, and the port has no viewer. A working
/// object (decision 21), as OCCT's: <see cref="StdPrs_ToolTriangulatedShape.GetDeflection"/> stores the absolute
/// deflection it computes into the drawer.
/// </remarks>
public class Prs3d_Drawer : Transient
{
    private Prs3d_Drawer? myLink;
    private double myChordialDeviation;
    private Aspect_TypeOfDeflection myTypeOfDeflection;
    private bool myHasOwnTypeOfDeflection;
    private double myDeviationCoefficient;
    private double myPreviousDeviationCoefficient;
    private double myDeviationAngle;
    private double myPreviousDeviationAngle;
    private bool myIsAutoTriangulated;
    private bool myHasOwnIsAutoTriangulated;

    /// <summary>A drawer without its own settings.</summary>
    public Prs3d_Drawer()
    {
        myChordialDeviation = -1.0;
        myTypeOfDeflection = Aspect_TypeOfDeflection.Aspect_TOD_RELATIVE;
        myHasOwnTypeOfDeflection = false;
        myDeviationCoefficient = -1.0;
        myDeviationAngle = -1.0;
        myIsAutoTriangulated = true;
        myHasOwnIsAutoTriangulated = false;
    }

    /// <summary>Gives the drawer its own default deflection settings.</summary>
    public void SetupOwnDefaults()
    {
        myChordialDeviation = 0.0001;
        myDeviationCoefficient = 0.001;
        myDeviationAngle = 20.0 * Math.PI / 180.0;
    }

    /// <summary>Sets the type of chordal deflection: absolute, or relative to the size of the object.</summary>
    public void SetTypeOfDeflection(Aspect_TypeOfDeflection theTypeOfDeflection)
    {
        myHasOwnTypeOfDeflection = true;
        myTypeOfDeflection = theTypeOfDeflection;
    }

    /// <summary>The type of chordal deflection: absolute, or relative to the size of the object.</summary>
    public Aspect_TypeOfDeflection TypeOfDeflection()
        => myHasOwnTypeOfDeflection || myLink is null ? myTypeOfDeflection : myLink.TypeOfDeflection();

    /// <summary>True if the drawer has its own type of deflection.</summary>
    public bool HasOwnTypeOfDeflection() => myHasOwnTypeOfDeflection;

    /// <summary>Undoes <see cref="SetTypeOfDeflection"/>.</summary>
    public void UnsetOwnTypeOfDeflection()
    {
        myHasOwnTypeOfDeflection = false;
        myTypeOfDeflection = Aspect_TypeOfDeflection.Aspect_TOD_RELATIVE;
    }

    /// <summary>Defines the maximal chordial deviation (the absolute deflection).</summary>
    public void SetMaximalChordialDeviation(double theChordialDeviation) => myChordialDeviation = theChordialDeviation;

    /// <summary>The maximal chordal deviation; 0.0001 by default.</summary>
    public double MaximalChordialDeviation()
        => myChordialDeviation > 0.0 ? myChordialDeviation : myLink?.MaximalChordialDeviation() ?? 0.0001;

    /// <summary>True if the drawer has its own maximal chordial deviation.</summary>
    public bool HasOwnMaximalChordialDeviation() => myChordialDeviation > 0.0;

    /// <summary>Undoes <see cref="SetMaximalChordialDeviation"/>.</summary>
    public void UnsetOwnMaximalChordialDeviation() => myChordialDeviation = -1.0;

    /// <summary>Sets the deviation coefficient (the relative deflection), keeping the previous one.</summary>
    public void SetDeviationCoefficient(double theCoefficient)
    {
        myPreviousDeviationCoefficient = DeviationCoefficient();
        myDeviationCoefficient = theCoefficient;
    }

    /// <summary>
    /// The deviation coefficient: with a relative type of deflection, the deflection is the size of the object times
    /// this coefficient. 0.001 by default.
    /// </summary>
    public double DeviationCoefficient()
        => myDeviationCoefficient > 0.0 ? myDeviationCoefficient : myLink?.DeviationCoefficient() ?? 0.001;

    /// <summary>Undoes <see cref="SetDeviationCoefficient(double)"/>.</summary>
    public void SetDeviationCoefficient() => myDeviationCoefficient = -1.0;

    /// <summary>True if the drawer has its own deviation coefficient.</summary>
    public bool HasOwnDeviationCoefficient() => myDeviationCoefficient > 0.0;

    /// <summary>The previous deviation coefficient.</summary>
    public double PreviousDeviationCoefficient() => HasOwnDeviationCoefficient() ? myPreviousDeviationCoefficient : 0.0;

    /// <summary>Makes the current deviation coefficient the previous one.</summary>
    public void UpdatePreviousDeviationCoefficient()
    {
        if (HasOwnDeviationCoefficient())
        {
            myPreviousDeviationCoefficient = DeviationCoefficient();
        }
    }

    /// <summary>Sets the deviation angle, keeping the previous one.</summary>
    public void SetDeviationAngle(double theAngle)
    {
        myPreviousDeviationAngle = DeviationAngle();
        myDeviationAngle = theAngle;
    }

    /// <summary>The deviation angle in radians, 20 * PI / 180 by default.</summary>
    public double DeviationAngle()
        => myDeviationAngle > 0.0 ? myDeviationAngle : myLink?.DeviationAngle() ?? 20.0 * Math.PI / 180.0;

    /// <summary>Undoes <see cref="SetDeviationAngle(double)"/>.</summary>
    public void SetDeviationAngle() => myDeviationAngle = -1.0;

    /// <summary>True if the drawer has its own deviation angle.</summary>
    public bool HasOwnDeviationAngle() => myDeviationAngle > 0.0;

    /// <summary>The previous deviation angle.</summary>
    public double PreviousDeviationAngle() => HasOwnDeviationAngle() ? myPreviousDeviationAngle : 0.0;

    /// <summary>Makes the current deviation angle the previous one.</summary>
    public void UpdatePreviousDeviationAngle()
    {
        if (HasOwnDeviationAngle())
        {
            myPreviousDeviationAngle = DeviationAngle();
        }
    }

    /// <summary>
    /// Turns automatic triangulation on or off: when on, shapes are meshed for display with the deflection-check
    /// logic; when off, the application meshes them.
    /// </summary>
    public void SetAutoTriangulation(bool theIsEnabled)
    {
        myHasOwnIsAutoTriangulated = true;
        myIsAutoTriangulated = theIsEnabled;
    }

    /// <summary>True if automatic triangulation is on.</summary>
    public bool IsAutoTriangulation()
        => myHasOwnIsAutoTriangulated || myLink is null ? myIsAutoTriangulated : myLink.IsAutoTriangulation();

    /// <summary>True if the drawer has its own automatic triangulation setting.</summary>
    public bool HasOwnIsAutoTriangulation() => myHasOwnIsAutoTriangulated;

    /// <summary>Undoes <see cref="SetAutoTriangulation"/>.</summary>
    public void UnsetOwnIsAutoTriangulation()
    {
        myHasOwnIsAutoTriangulated = false;
        myIsAutoTriangulated = true;
    }

    /// <summary>The drawer whose settings this one takes where it has none of its own, or null.</summary>
    public Prs3d_Drawer? Link() => myLink;

    /// <summary>True if the drawer has a link.</summary>
    public bool HasLink() => myLink is not null;

    /// <summary>Sets the drawer whose settings this one takes where it has none of its own.</summary>
    public void SetLink(Prs3d_Drawer? theDrawer) => myLink = theDrawer;
}
