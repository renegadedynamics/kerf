// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct_RenderingProperties.hxx
// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct_RenderingProperties.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using static Kerf.Engine.Foundation.NameOfColor;
using static Occt.StepVisual_ShadingSurfaceMethod;

namespace Occt;

/// <summary>
/// Class for working with STEP rendering properties. Provides functionality to create and manipulate rendering
/// properties used for specifying visual appearance in STEP format. This class handles both parsing of STEP entities
/// and creation of new ones.
/// </summary>
/// <remarks>
/// A C++ value class, a C# class here (decision 1185): callers copy it with <see cref="Copy"/> where C++ copies it.
/// OCCT.NET: the constructors and <c>Init</c>s from XCAF's visualisation materials (<c>XCAFDoc_VisMaterial</c>,
/// <c>XCAFDoc_VisMaterialCommon</c>) and <c>CreateXCAFMaterial</c> are left out with the materials (decision 1276).
/// </remarks>
public class STEPConstruct_RenderingProperties
{
    /// <summary>Default constructor creating an empty rendering properties object</summary>
    public STEPConstruct_RenderingProperties()
    {
        reset();
    }

    /// <summary>Constructor from STEP rendering properties entity.</summary>
    public STEPConstruct_RenderingProperties(StepVisual_SurfaceStyleRenderingWithProperties theRenderingProperties)
    {
        reset();
        Init(theRenderingProperties);
    }

    /// <summary>Constructor from RGBA color.</summary>
    public STEPConstruct_RenderingProperties(in ColorWithAlpha theRGBAColor)
    {
        reset();
        Init(theRGBAColor);
    }

    /// <summary>Constructor from STEP color and transparency value.</summary>
    public STEPConstruct_RenderingProperties(StepVisual_Colour theColor, double theTransparency)
    {
        reset();
        Init(theColor, theTransparency);
    }

    /// <summary>Constructor from surface color, transparency, and rendering method.</summary>
    public STEPConstruct_RenderingProperties(in Color theSurfaceColor, double theTransparency = 0.0)
    {
        reset();
        Init(theSurfaceColor, theTransparency);
    }

    /// <summary>A copy (C++ copies this value class).</summary>
    public STEPConstruct_RenderingProperties Copy() => (STEPConstruct_RenderingProperties)MemberwiseClone();

    private void reset()
    {
        mySurfaceColor = new Color(NocWhite);
        myTransparency = 0.0;
        myRenderingMethod = StepVisual_ssmNormalShading;
        myIsDefined = false;
        myAmbientReflectance = (0.0, false);
        myDiffuseReflectance = (0.0, false);
        mySpecularReflectance = (0.0, false);
        mySpecularExponent = (0.0, false);
        mySpecularColour = (new Color(NocWhite), false);
    }

    /// <summary>Sets ambient reflectance value</summary>
    public void SetAmbientReflectance(double theAmbientReflectance)
    {
        myAmbientReflectance = (theAmbientReflectance, true);
    }

    /// <summary>Sets ambient and diffuse reflectance values</summary>
    public void SetAmbientAndDiffuseReflectance(double theAmbientReflectance, double theDiffuseReflectance)
    {
        myAmbientReflectance = (theAmbientReflectance, true);
        myDiffuseReflectance = (theDiffuseReflectance, true);
    }

    /// <summary>Sets ambient, diffuse and specular reflectance values</summary>
    public void SetAmbientDiffuseAndSpecularReflectance(double theAmbientReflectance, double theDiffuseReflectance,
                                                        double theSpecularReflectance, double theSpecularExponent,
                                                        in Color theSpecularColour)
    {
        myAmbientReflectance = (theAmbientReflectance, true);
        myDiffuseReflectance = (theDiffuseReflectance, true);
        mySpecularReflectance = (theSpecularReflectance, true);
        mySpecularExponent = (theSpecularExponent, true);
        mySpecularColour = (theSpecularColour, true);
    }

    /// <summary>Creates and returns rendering properties entity</summary>
    public StepVisual_SurfaceStyleRenderingWithProperties CreateRenderingProperties()
    {
        return CreateRenderingProperties(STEPConstruct_Styles.EncodeColor(mySurfaceColor));
    }

    /// <summary>Creates and returns rendering properties entity with the specified color</summary>
    public StepVisual_SurfaceStyleRenderingWithProperties CreateRenderingProperties(StepVisual_Colour theRenderColour)
    {
        if (!myIsDefined)
        {
            return null;
        }

        // Create STEP RGB color or use predefined color using STEPConstruct_Styles utility
        StepVisual_Colour aStepColor = theRenderColour is not null ? theRenderColour : STEPConstruct_Styles.EncodeColor(mySurfaceColor);

        // Count and determine which properties to create.
        // Transparency is always included.
        int aNbProps = 1;

        // Determine which reflectance properties to include
        bool hasFullReflectance = mySpecularColour.Item2 && mySpecularExponent.Item2 && mySpecularReflectance.Item2
                                  && myDiffuseReflectance.Item2 && myAmbientReflectance.Item2;
        bool hasDiffuseAndAmbient = myDiffuseReflectance.Item2 && myAmbientReflectance.Item2 && !mySpecularReflectance.Item2;
        bool hasAmbientOnly = myAmbientReflectance.Item2 && !myDiffuseReflectance.Item2 && !mySpecularReflectance.Item2;

        // Add reflectance property if we have any defined
        if (hasFullReflectance || hasDiffuseAndAmbient || hasAmbientOnly)
        {
            aNbProps++;
        }

        // Create properties array
        NCollection_HArray1<StepVisual_RenderingPropertiesSelect> aProps =
            new NCollection_HArray1<StepVisual_RenderingPropertiesSelect>(1, aNbProps);
        for (int i = 1; i <= aNbProps; i++)
        {
            aProps.SetValue(i, new StepVisual_RenderingPropertiesSelect());
        }
        int aPropIndex = 1;

        // Add transparency
        StepVisual_SurfaceStyleTransparent aTransparent = new StepVisual_SurfaceStyleTransparent();
        aTransparent.Init(myTransparency);
        aProps.Value(aPropIndex++).SetValue(aTransparent);

        // Add the appropriate reflectance model based on what we have
        if (hasAmbientOnly)
        {
            // Add ambient reflectance only
            StepVisual_SurfaceStyleReflectanceAmbient aAmbient = new StepVisual_SurfaceStyleReflectanceAmbient();
            aAmbient.Init(myAmbientReflectance.Item1);
            aProps.Value(aPropIndex++).SetValue(aAmbient);
        }
        else if (hasDiffuseAndAmbient)
        {
            // Add ambient and diffuse reflectance
            StepVisual_SurfaceStyleReflectanceAmbientDiffuse aAmbientDiffuse = new StepVisual_SurfaceStyleReflectanceAmbientDiffuse();
            aAmbientDiffuse.Init(myAmbientReflectance.Item1, myDiffuseReflectance.Item1);
            aProps.Value(aPropIndex++).SetValue(aAmbientDiffuse);
        }
        else if (hasFullReflectance)
        {
            // Add full PBR reflectance model (ambient, diffuse, specular)
            StepVisual_SurfaceStyleReflectanceAmbientDiffuseSpecular aFullRefl =
                new StepVisual_SurfaceStyleReflectanceAmbientDiffuseSpecular();
            aFullRefl.Init(myAmbientReflectance.Item1, myDiffuseReflectance.Item1, mySpecularReflectance.Item1,
                           mySpecularExponent.Item1, STEPConstruct_Styles.EncodeColor(mySpecularColour.Item1));
            aProps.Value(aPropIndex++).SetValue(aFullRefl);
        }

        // Create STEP surface style rendering with properties
        StepVisual_SurfaceStyleRenderingWithProperties aSSRWP = new StepVisual_SurfaceStyleRenderingWithProperties();
        aSSRWP.Init(myRenderingMethod, aStepColor, aProps);
        return aSSRWP;
    }

    /// <summary>Initializes from STEP rendering properties entity.</summary>
    public void Init(StepVisual_SurfaceStyleRenderingWithProperties theRenderingProperties)
    {
        reset();

        if (theRenderingProperties is null)
        {
            return;
        }

        myRenderingMethod = theRenderingProperties.RenderingMethod();
        StepVisual_Colour aColor = theRenderingProperties.SurfaceColour();

        // Decode surface color using STEPConstruct_Styles utility
        if (aColor is not null)
        {
            Color aDecodedColor = new Color();
            if (STEPConstruct_Styles.DecodeColor(aColor, ref aDecodedColor))
            {
                mySurfaceColor = aDecodedColor;
                myIsDefined = true;
            }
        }

        // Process rendering properties
        NCollection_HArray1<StepVisual_RenderingPropertiesSelect> aProperties = theRenderingProperties.Properties();
        if (aProperties is not null)
        {
            for (int i = 1; i <= aProperties.Length(); i++)
            {
                StepVisual_RenderingPropertiesSelect aPropSelect = aProperties.Value(i);
                if (aPropSelect is null)
                {
                    continue;
                }

                // Check for transparency
                StepVisual_SurfaceStyleTransparent aTransparent = aPropSelect.SurfaceStyleTransparent();
                if (aTransparent is not null)
                {
                    myTransparency = aTransparent.Transparency();
                    myIsDefined = true;
                }

                // Check for ambient reflectance
                StepVisual_SurfaceStyleReflectanceAmbient aAmbient = aPropSelect.SurfaceStyleReflectanceAmbient();
                StepVisual_SurfaceStyleReflectanceAmbientDiffuse aAmbientDiffuse = aAmbient as StepVisual_SurfaceStyleReflectanceAmbientDiffuse;
                StepVisual_SurfaceStyleReflectanceAmbientDiffuseSpecular aFullRefl =
                    aAmbient as StepVisual_SurfaceStyleReflectanceAmbientDiffuseSpecular;

                if (aFullRefl is not null)
                {
                    myIsDefined = true;
                    myAmbientReflectance = (aFullRefl.AmbientReflectance(), true);
                    myDiffuseReflectance = (aFullRefl.DiffuseReflectance(), true);
                    mySpecularReflectance = (aFullRefl.SpecularReflectance(), true);
                    mySpecularExponent = (aFullRefl.SpecularExponent(), true);
                    Color aSpecular = mySpecularColour.Item1;
                    bool isDecoded = STEPConstruct_Styles.DecodeColor(aFullRefl.SpecularColour(), ref aSpecular);
                    mySpecularColour = (aSpecular, isDecoded);
                }
                else if (aAmbientDiffuse is not null)
                {
                    myAmbientReflectance = (aAmbientDiffuse.AmbientReflectance(), true);
                    myDiffuseReflectance = (aAmbientDiffuse.DiffuseReflectance(), true);
                    myIsDefined = true;
                }
                else if (aAmbient is not null)
                {
                    myAmbientReflectance = (aAmbient.AmbientReflectance(), true);
                    myIsDefined = true;
                }
            }
        }
    }

    /// <summary>Initializes from RGBA color.</summary>
    public void Init(in ColorWithAlpha theRGBAColor)
    {
        reset();
        mySurfaceColor = theRGBAColor.GetRGB();
        myTransparency = 1.0 - theRGBAColor.Alpha();
        myIsDefined = true;
    }

    /// <summary>Initializes from STEP color and transparency value.</summary>
    public void Init(StepVisual_Colour theColor, double theTransparency)
    {
        reset();
        myTransparency = theTransparency;

        if (theColor is null)
        {
            return;
        }

        // Decode color using STEPConstruct_Styles utility
        Color aDecodedColor = new Color();
        if (STEPConstruct_Styles.DecodeColor(theColor, ref aDecodedColor))
        {
            mySurfaceColor = aDecodedColor;
            myIsDefined = true;
        }
    }

    /// <summary>Initializes from surface color, transparency and rendering method.</summary>
    public void Init(in Color theSurfaceColor, double theTransparency = 0.0)
    {
        reset();
        mySurfaceColor = theSurfaceColor;
        myTransparency = theTransparency;
        myIsDefined = true;
    }

    /// <summary>Creates the ColorRGBA object from the current color and transparency</summary>
    public ColorWithAlpha GetRGBAColor()
    {
        return new ColorWithAlpha(mySurfaceColor, (float)(1.0 - myTransparency));
    }

    /// <summary>Returns surface color</summary>
    public Color SurfaceColor() => mySurfaceColor;

    /// <summary>Returns transparency value</summary>
    public double Transparency() => myTransparency;

    /// <summary>Returns rendering method</summary>
    public StepVisual_ShadingSurfaceMethod RenderingMethod() => myRenderingMethod;

    /// <summary>Sets rendering method</summary>
    public void SetRenderingMethod(StepVisual_ShadingSurfaceMethod theRenderingMethod) => myRenderingMethod = theRenderingMethod;

    /// <summary>Returns whether the rendering properties are defined</summary>
    public bool IsDefined() => myIsDefined;

    /// <summary>Returns whether material is convertible to STEP</summary>
    public bool IsMaterialConvertible()
    {
        return myIsDefined && myAmbientReflectance.Item2 && myDiffuseReflectance.Item2 && mySpecularReflectance.Item2
               && mySpecularExponent.Item2 && mySpecularColour.Item2;
    }

    /// <summary>Returns ambient reflectance value</summary>
    public double AmbientReflectance() => myAmbientReflectance.Item1;

    /// <summary>Returns whether ambient reflectance is defined</summary>
    public bool IsAmbientReflectanceDefined() => myAmbientReflectance.Item2;

    /// <summary>Returns diffuse reflectance value</summary>
    public double DiffuseReflectance() => myDiffuseReflectance.Item1;

    /// <summary>Returns whether diffuse reflectance is defined</summary>
    public bool IsDiffuseReflectanceDefined() => myDiffuseReflectance.Item2;

    /// <summary>Returns specular reflectance value</summary>
    public double SpecularReflectance() => mySpecularReflectance.Item1;

    /// <summary>Returns whether specular reflectance is defined</summary>
    public bool IsSpecularReflectanceDefined() => mySpecularReflectance.Item2;

    /// <summary>Returns specular exponent value</summary>
    public double SpecularExponent() => mySpecularExponent.Item1;

    /// <summary>Returns whether specular exponent is defined</summary>
    public bool IsSpecularExponentDefined() => mySpecularExponent.Item2;

    /// <summary>Returns specular color</summary>
    public Color SpecularColour() => mySpecularColour.Item1;

    /// <summary>Returns whether specular color is defined</summary>
    public bool IsSpecularColourDefined() => mySpecularColour.Item2;

    private Color mySurfaceColor;
    private double myTransparency;
    private StepVisual_ShadingSurfaceMethod myRenderingMethod;
    private bool myIsDefined;
    private (double, bool) myAmbientReflectance;
    private (double, bool) myDiffuseReflectance;
    private (double, bool) mySpecularReflectance;
    private (double, bool) mySpecularExponent;
    private (Color, bool) mySpecularColour;
}
