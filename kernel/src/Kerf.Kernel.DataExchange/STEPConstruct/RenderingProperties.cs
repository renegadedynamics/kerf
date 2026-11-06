// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct_RenderingProperties.hxx
// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct_RenderingProperties.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Step.Visual;
using Kerf.Engine.Foundation;
using static Kerf.Engine.Foundation.NameOfColor;

namespace Kerf.Engine.Exchange.Step.Construct;

/// <summary>
/// Class for working with STEP rendering properties. Provides functionality to create and manipulate rendering
/// properties used for specifying visual appearance in STEP format. This class handles both parsing of STEP entities
/// and creation of new ones.
/// </summary>
/// <remarks>
/// A C++ value class, a C# class here (decision 1185): callers copy it with <see cref="Copy"/> where C++ copies it.
/// Kerf: the constructors and <c>Init</c>s from XCAF's visualisation materials (<c>XCAFDoc_VisMaterial</c>,
/// <c>XCAFDoc_VisMaterialCommon</c>) and <c>CreateXCAFMaterial</c> are left out with the materials (decision 1276).
/// </remarks>
internal class RenderingProperties
{
    /// <summary>Default constructor creating an empty rendering properties object</summary>
    public RenderingProperties()
    {
        Reset();
    }

    /// <summary>Constructor from STEP rendering properties entity.</summary>
    public RenderingProperties(SurfaceStyleRenderingWithProperties renderingProperties)
    {
        Reset();
        Init(renderingProperties);
    }

    /// <summary>Constructor from RGBA color.</summary>
    public RenderingProperties(in ColorWithAlpha rgbaColor)
    {
        Reset();
        Init(rgbaColor);
    }

    /// <summary>Constructor from STEP color and transparency value.</summary>
    public RenderingProperties(Colour color, double transparency)
    {
        Reset();
        Init(color, transparency);
    }

    /// <summary>Constructor from surface color, transparency, and rendering method.</summary>
    public RenderingProperties(in LinearColor surfaceColor, double transparency = 0.0)
    {
        Reset();
        Init(surfaceColor, transparency);
    }

    /// <summary>A copy (C++ copies this value class).</summary>
    public RenderingProperties Copy() => (RenderingProperties)MemberwiseClone();

    private void Reset()
    {
        _surfaceColor = new LinearColor(NocWhite);
        _transparency = 0.0;
        _renderingMethod = ShadingSurfaceMethod.SsmNormalShading;
        _isDefined = false;
        _ambientReflectance = (0.0, false);
        _diffuseReflectance = (0.0, false);
        _specularReflectance = (0.0, false);
        _specularExponent = (0.0, false);
        _specularColour = (new LinearColor(NocWhite), false);
    }

    /// <summary>Sets ambient reflectance value</summary>
    public void SetAmbientReflectance(double ambientReflectance)
    {
        _ambientReflectance = (ambientReflectance, true);
    }

    /// <summary>Sets ambient and diffuse reflectance values</summary>
    public void SetAmbientAndDiffuseReflectance(double ambientReflectance, double diffuseReflectance)
    {
        _ambientReflectance = (ambientReflectance, true);
        _diffuseReflectance = (diffuseReflectance, true);
    }

    /// <summary>Sets ambient, diffuse and specular reflectance values</summary>
    public void SetAmbientDiffuseAndSpecularReflectance(double ambientReflectance, double diffuseReflectance,
                                                        double specularReflectance, double specularExponent,
                                                        in LinearColor specularColour)
    {
        _ambientReflectance = (ambientReflectance, true);
        _diffuseReflectance = (diffuseReflectance, true);
        _specularReflectance = (specularReflectance, true);
        _specularExponent = (specularExponent, true);
        _specularColour = (specularColour, true);
    }

    /// <summary>Creates and returns rendering properties entity</summary>
    public SurfaceStyleRenderingWithProperties CreateRenderingProperties()
    {
        return CreateRenderingProperties(Styles.EncodeColor(_surfaceColor));
    }

    /// <summary>Creates and returns rendering properties entity with the specified color</summary>
    public SurfaceStyleRenderingWithProperties CreateRenderingProperties(Colour renderColour)
    {
        if (!_isDefined)
        {
            return null;
        }

        // Create STEP RGB color or use predefined color using STEPConstruct_Styles utility
        Colour stepColor = renderColour is not null ? renderColour : Styles.EncodeColor(_surfaceColor);

        // Count and determine which properties to create.
        // Transparency is always included.
        int nbProps = 1;

        // Determine which reflectance properties to include
        bool hasFullReflectance = _specularColour.Item2 && _specularExponent.Item2 && _specularReflectance.Item2
                                  && _diffuseReflectance.Item2 && _ambientReflectance.Item2;
        bool hasDiffuseAndAmbient = _diffuseReflectance.Item2 && _ambientReflectance.Item2 && !_specularReflectance.Item2;
        bool hasAmbientOnly = _ambientReflectance.Item2 && !_diffuseReflectance.Item2 && !_specularReflectance.Item2;

        // Add reflectance property if we have any defined
        if (hasFullReflectance || hasDiffuseAndAmbient || hasAmbientOnly)
        {
            nbProps++;
        }

        // Create properties array
        NCollection_HArray1<RenderingPropertiesSelect> props =
            new NCollection_HArray1<RenderingPropertiesSelect>(1, nbProps);
        for (int i = 1; i <= nbProps; i++)
        {
            props.SetValue(i, new RenderingPropertiesSelect());
        }
        int propIndex = 1;

        // Add transparency
        SurfaceStyleTransparent transparent = new SurfaceStyleTransparent();
        transparent.Init(_transparency);
        props.Value(propIndex++).SetValue(transparent);

        // Add the appropriate reflectance model based on what we have
        if (hasAmbientOnly)
        {
            // Add ambient reflectance only
            SurfaceStyleReflectanceAmbient ambient = new SurfaceStyleReflectanceAmbient();
            ambient.Init(_ambientReflectance.Item1);
            props.Value(propIndex++).SetValue(ambient);
        }
        else if (hasDiffuseAndAmbient)
        {
            // Add ambient and diffuse reflectance
            SurfaceStyleReflectanceAmbientDiffuse ambientDiffuse = new SurfaceStyleReflectanceAmbientDiffuse();
            ambientDiffuse.Init(_ambientReflectance.Item1, _diffuseReflectance.Item1);
            props.Value(propIndex++).SetValue(ambientDiffuse);
        }
        else if (hasFullReflectance)
        {
            // Add full PBR reflectance model (ambient, diffuse, specular)
            SurfaceStyleReflectanceAmbientDiffuseSpecular fullRefl =
                new SurfaceStyleReflectanceAmbientDiffuseSpecular();
            fullRefl.Init(_ambientReflectance.Item1, _diffuseReflectance.Item1, _specularReflectance.Item1,
                           _specularExponent.Item1, Styles.EncodeColor(_specularColour.Item1));
            props.Value(propIndex++).SetValue(fullRefl);
        }

        // Create STEP surface style rendering with properties
        SurfaceStyleRenderingWithProperties ssrwp = new SurfaceStyleRenderingWithProperties();
        ssrwp.Init(_renderingMethod, stepColor, props);
        return ssrwp;
    }

    /// <summary>Initializes from STEP rendering properties entity.</summary>
    public void Init(SurfaceStyleRenderingWithProperties renderingProperties)
    {
        Reset();

        if (renderingProperties is null)
        {
            return;
        }

        _renderingMethod = renderingProperties.RenderingMethod();
        Colour color = renderingProperties.SurfaceColour();

        // Decode surface color using STEPConstruct_Styles utility
        if (color is not null)
        {
            LinearColor decodedColor = new LinearColor();
            if (Styles.DecodeColor(color, ref decodedColor))
            {
                _surfaceColor = decodedColor;
                _isDefined = true;
            }
        }

        // Process rendering properties
        NCollection_HArray1<RenderingPropertiesSelect> properties = renderingProperties.Properties();
        if (properties is not null)
        {
            for (int i = 1; i <= properties.Length(); i++)
            {
                RenderingPropertiesSelect propSelect = properties.Value(i);
                if (propSelect is null)
                {
                    continue;
                }

                // Check for transparency
                SurfaceStyleTransparent transparent = propSelect.SurfaceStyleTransparent();
                if (transparent is not null)
                {
                    _transparency = transparent.Transparency();
                    _isDefined = true;
                }

                // Check for ambient reflectance
                SurfaceStyleReflectanceAmbient ambient = propSelect.SurfaceStyleReflectanceAmbient();
                SurfaceStyleReflectanceAmbientDiffuse ambientDiffuse = ambient as SurfaceStyleReflectanceAmbientDiffuse;
                SurfaceStyleReflectanceAmbientDiffuseSpecular fullRefl =
                    ambient as SurfaceStyleReflectanceAmbientDiffuseSpecular;

                if (fullRefl is not null)
                {
                    _isDefined = true;
                    _ambientReflectance = (fullRefl.AmbientReflectance(), true);
                    _diffuseReflectance = (fullRefl.DiffuseReflectance(), true);
                    _specularReflectance = (fullRefl.SpecularReflectance(), true);
                    _specularExponent = (fullRefl.SpecularExponent(), true);
                    LinearColor specular = _specularColour.Item1;
                    bool isDecoded = Styles.DecodeColor(fullRefl.SpecularColour(), ref specular);
                    _specularColour = (specular, isDecoded);
                }
                else if (ambientDiffuse is not null)
                {
                    _ambientReflectance = (ambientDiffuse.AmbientReflectance(), true);
                    _diffuseReflectance = (ambientDiffuse.DiffuseReflectance(), true);
                    _isDefined = true;
                }
                else if (ambient is not null)
                {
                    _ambientReflectance = (ambient.AmbientReflectance(), true);
                    _isDefined = true;
                }
            }
        }
    }

    /// <summary>Initializes from RGBA color.</summary>
    public void Init(in ColorWithAlpha rgbaColor)
    {
        Reset();
        _surfaceColor = rgbaColor.GetRGB();
        _transparency = 1.0 - rgbaColor.Alpha();
        _isDefined = true;
    }

    /// <summary>Initializes from STEP color and transparency value.</summary>
    public void Init(Colour color, double transparency)
    {
        Reset();
        _transparency = transparency;

        if (color is null)
        {
            return;
        }

        // Decode color using STEPConstruct_Styles utility
        LinearColor decodedColor = new LinearColor();
        if (Styles.DecodeColor(color, ref decodedColor))
        {
            _surfaceColor = decodedColor;
            _isDefined = true;
        }
    }

    /// <summary>Initializes from surface color, transparency and rendering method.</summary>
    public void Init(in LinearColor surfaceColor, double transparency = 0.0)
    {
        Reset();
        _surfaceColor = surfaceColor;
        _transparency = transparency;
        _isDefined = true;
    }

    /// <summary>Creates the ColorRGBA object from the current color and transparency</summary>
    public ColorWithAlpha GetRGBAColor()
    {
        return new ColorWithAlpha(_surfaceColor, (float)(1.0 - _transparency));
    }

    /// <summary>Returns surface color</summary>
    public LinearColor SurfaceColor() => _surfaceColor;

    /// <summary>Returns transparency value</summary>
    public double Transparency() => _transparency;

    /// <summary>Returns rendering method</summary>
    public ShadingSurfaceMethod RenderingMethod() => _renderingMethod;

    /// <summary>Sets rendering method</summary>
    public void SetRenderingMethod(ShadingSurfaceMethod renderingMethod) => _renderingMethod = renderingMethod;

    /// <summary>Returns whether the rendering properties are defined</summary>
    public bool IsDefined => _isDefined;

    /// <summary>Returns whether material is convertible to STEP</summary>
    public bool IsMaterialConvertible
    {
        get
        {
            return _isDefined && _ambientReflectance.Item2 && _diffuseReflectance.Item2 && _specularReflectance.Item2
                   && _specularExponent.Item2 && _specularColour.Item2;
        }
    }

    /// <summary>Returns ambient reflectance value</summary>
    public double AmbientReflectance() => _ambientReflectance.Item1;

    /// <summary>Returns whether ambient reflectance is defined</summary>
    public bool IsAmbientReflectanceDefined => _ambientReflectance.Item2;

    /// <summary>Returns diffuse reflectance value</summary>
    public double DiffuseReflectance() => _diffuseReflectance.Item1;

    /// <summary>Returns whether diffuse reflectance is defined</summary>
    public bool IsDiffuseReflectanceDefined => _diffuseReflectance.Item2;

    /// <summary>Returns specular reflectance value</summary>
    public double SpecularReflectance() => _specularReflectance.Item1;

    /// <summary>Returns whether specular reflectance is defined</summary>
    public bool IsSpecularReflectanceDefined => _specularReflectance.Item2;

    /// <summary>Returns specular exponent value</summary>
    public double SpecularExponent() => _specularExponent.Item1;

    /// <summary>Returns whether specular exponent is defined</summary>
    public bool IsSpecularExponentDefined => _specularExponent.Item2;

    /// <summary>Returns specular color</summary>
    public LinearColor SpecularColour() => _specularColour.Item1;

    /// <summary>Returns whether specular color is defined</summary>
    public bool IsSpecularColourDefined => _specularColour.Item2;

    private LinearColor _surfaceColor;
    private double _transparency;
    private ShadingSurfaceMethod _renderingMethod;
    private bool _isDefined;
    private (double, bool) _ambientReflectance;
    private (double, bool) _diffuseReflectance;
    private (double, bool) _specularReflectance;
    private (double, bool) _specularExponent;
    private (LinearColor, bool) _specularColour;
}
