// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Globalization;
using System.Text;
using Occt;

namespace OcctNet.DrawLite;

/// <summary>
/// A Draw command's result as Draw_Interpretor builds it: <c>di &lt;&lt; ...</c> appends to the Tcl result (an int
/// as <c>%d</c>, a real as <c>%.17g</c>), <c>AppendElement</c> appends a list element. A command that returns 1
/// fails with the result as its message.
/// </summary>
public sealed class DrawResult
{
    private readonly StringBuilder myText = new();

    public DrawResult Append(string theText)
    {
        myText.Append(theText);
        return this;
    }

    public DrawResult Append(int theValue)
    {
        myText.Append(theValue.ToString(CultureInfo.InvariantCulture));
        return this;
    }

    /// <summary>Draw_Interpretor::Append(double): <c>%.17g</c>.</summary>
    public DrawResult Append(double theValue)
    {
        myText.Append(StreamUtilities.FormatReal(theValue, 17));
        return this;
    }

    /// <summary>Tcl_AppendElement: a space first unless the result is empty or ends with a space, then the element quoted.</summary>
    public void AppendElement(string theElement)
    {
        bool anIsFirst = myText.Length == 0;
        if (!anIsFirst && !char.IsWhiteSpace(myText[^1]) && myText[^1] != '{')
        {
            myText.Append(' ');
        }

        myText.Append(TclList.Quote(theElement, anIsFirst));
    }

    public override string ToString() => myText.ToString();
}

/// <summary>C++ stream formats that Standard_Stream doesn't have.</summary>
internal static class DrawFormat
{
    /// <summary>A real in C's <c>%.*e</c> (std::scientific at the precision): at least two exponent digits.</summary>
    public static string Scientific(double theValue, int thePrecision)
    {
        if (double.IsNaN(theValue) || double.IsInfinity(theValue))
        {
            return StreamUtilities.FormatReal(theValue, thePrecision);
        }

        // .NET's "E" format is exact and rounds ties to even, as the C runtime does; it writes three exponent digits
        string aText = theValue.ToString("E" + thePrecision.ToString(CultureInfo.InvariantCulture), CultureInfo.InvariantCulture);
        int anE = aText.IndexOf('E');
        string aMantissa = aText[..anE];
        char aSign = aText[anE + 1];
        string anExponent = aText[(anE + 2)..].TrimStart('0');
        return $"{aMantissa}e{aSign}{anExponent.PadLeft(2, '0')}";
    }

    /// <summary>A real as <c>std::setw(theWidth) &lt;&lt; theValue</c> on a stream of the default precision 6.</summary>
    public static string Setw(int theWidth, double theValue) => StreamUtilities.FormatReal(theValue, 6).PadLeft(theWidth);

    /// <summary>TCollection_AsciiString(double): <c>%g</c>.</summary>
    public static string AsciiString(double theValue) => StreamUtilities.FormatReal(theValue, 6);
}
